/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_drv_voc.c
 @brief  : adpcm player codes
 @author : gexueyuan
 @history:
           2015-1-30    gexueyuan    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "voc"
#include "cv_osal_dbg.h"

    
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "assert.h"
#include "cv_cms_def.h"
#include "voc.h"


short buffer_voc[BUFFER_COUNT][BUFFER_SIZE/2]; 

static uint8_t cursor_decode;
static uint8_t cursor_play;
osal_sem_t   *sem_adpcm_start;
osal_sem_t   *sem_adpcm_data;
osal_sem_t   *sem_buffer_voc;
osal_sem_t   *sem_audio_dev;
osal_sem_t   *sem_play_complete;
osal_queue_t *queue_play;



static voc_session_t voc_session;
static uint32_t voc_status;


static ADPCMState adpcm_state;


static const int indexTable[ 16 ] = {
	-1, -1, -1, -1, 2, 4, 6, 8,
	-1, -1, -1, -1, 2, 4, 6, 8
} ;
/* define an array of index adjustments to the step index value */

#define	MAXSTEPINDEX	88
/* define the maximum value to access the top element of stepSizeTable below */

static const int stepSizeTable[ MAXSTEPINDEX + 1 ] = {
	    7,     8,     9,    10,    11,    12,    13,    14,    16,    17,
	   19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
	   50,    55,    60,    66,    73,    80,    88,    97,   107,   118,
	  130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
	  337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
	  876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
	 2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
	 5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
	15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
} ;


int DecodeADPCMC( int adpcmSample, ADPCMStatePtr decodeStatePtr )
{
    int step ;
    int delta ;
    int predictionAdjustment ;
    
    if( ( adpcmSample < 0 ) || ( adpcmSample > 15 ) ) {
		OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_ERROR,"[DecodeADPCMC] Error in ADPCM sample, aborting.\n\n");
        /* function name given since intended as internal error for programmer */
        return 0 ;
    }
    
    if( !decodeStatePtr ) {
		OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_ERROR,"[DecodeADPCMC] Error in ADPCMState, aborting.\n\n");
        /* function name given since intended as internal error for programmer */
        return 0 ;
    }
        
    step = stepSizeTable[ decodeStatePtr->stepIndex ] ;
    
    delta = adpcmSample ;

    decodeStatePtr->stepIndex += indexTable[ delta ] ;
    /* range check the stepIndex */
    if( decodeStatePtr->stepIndex < 0 ) {
        decodeStatePtr->stepIndex = 0 ;
    }
    else if( decodeStatePtr->stepIndex > MAXSTEPINDEX ) {
        decodeStatePtr->stepIndex = MAXSTEPINDEX ;
    }

    /* remove sign from delta - pull to 3-bit */
    if( delta > 7 ) {
        delta -= 8 ;
    }
    
    predictionAdjustment = ( 2*delta + 1 )* step/8 ;
    if( adpcmSample > 7 ) {
        decodeStatePtr->prediction -= predictionAdjustment ;
    }
    else {
        decodeStatePtr->prediction += predictionAdjustment ;
    }
    
    /* range check the prediction so that fits in number of bits */
    if( decodeStatePtr->prediction > ( 1 << ( MAXBITS - 1 ) ) - 1 ) {
        decodeStatePtr->prediction = ( 1 << ( MAXBITS - 1 ) ) - 1 ;
    }
    else if( decodeStatePtr->prediction < -( 1 << ( MAXBITS - 1 ) ) ) {
        decodeStatePtr->prediction = -( 1 << ( MAXBITS - 1 ) ) ;
    }

    return decodeStatePtr->prediction ;
}

int adpcm_de(char *code, short *pcm, int count)
{
    int a,b;
    short p;

    RT_ASSERT((code != NULL)&&(pcm!= NULL)&&(count >0));

    while(count--){
        a = *code ++;
        b = (a>>4)&0x0f;
        p = DecodeADPCMC(b,&adpcm_state);
        *pcm++ = p;
        *pcm++ = p; // expand to stero
        b = (a)&0x0f;
        p = DecodeADPCMC(b,&adpcm_state); 
        *pcm++ = p;
        *pcm++ = p;
    }
    return 0;
}

void voc_play_complete(void)
{
    VOC_STATUS_CLR(VOC_STATUS_DEV_BUSY);
    osal_sem_release(sem_buffer_voc);
    osal_sem_release(sem_audio_dev);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"play end\n");
    voc_session.played_length += BUFFER_SIZE;

    if(voc_session.encode_type == VOC_ENCODE_PCM){
        if ((voc_session.played_length >= voc_session.src_length)\
            ||(VOC_STATUS_TST(VOC_STATUS_STOP))) {
            osal_sem_release(sem_play_complete);
            }
    }
    else if(voc_session.encode_type == VOC_ENCODE_ADPCM){
            if ((voc_session.played_length >= 8*voc_session.src_length)\
                ||(VOC_STATUS_TST(VOC_STATUS_STOP))) {
                osal_sem_release(sem_play_complete);
            }

    }
}

static int wait_for(osal_sem_t *sem)
{
    osal_status_t err;
    #define VOC_WAIT_TIMEOUT OSAL_WAITING_FOREVER//5

    do {
        err = osal_sem_take(sem,VOC_WAIT_TIMEOUT);
        if (VOC_STATUS_TST(VOC_STATUS_STOP)) {
        	OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"Voc play is aborted.\n\n");
            return -1;
        }

    } while(err == OSAL_STATUS_TIMEOUT);

    return 0;
}

static void audio_output(uint16_t* pBuffer, uint32_t Size)
{
    VOC_STATUS_SET(VOC_STATUS_DEV_BUSY);
    Pt8211_AUDIO_Play(pBuffer, Size);
}

void adpcm_process(uint8_t *pBuffer, uint32_t Size)
{
    uint32_t decode_size;

    while(Size > 0){
        /* Try to acquire the free decode buffer */
        if (wait_for(sem_buffer_voc) < 0) {
            return;
        }    

        decode_size = (Size > BUFFER_SIZE/8)? (BUFFER_SIZE/8):Size;
        memset(buffer_voc[cursor_decode%BUFFER_COUNT], 0, BUFFER_SIZE);
        adpcm_de((char *)pBuffer,buffer_voc[cursor_decode%BUFFER_COUNT],decode_size);
        osal_sem_release(sem_adpcm_data);

        Size -= decode_size;
        pBuffer += decode_size;
        cursor_decode++;
        //OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"decode end!!\n");
    }
}

void rt_adpcm_thread_entry(void *parameter)
{
    osal_status_t err;
    
    while(1){
        err = osal_sem_take(sem_adpcm_start, OSAL_WAITING_FOREVER);
        if (err != OSAL_STATUS_SUCCESS) {
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"sem_adpcm return error(%d).\n\n", err);
            break;
        }

        adpcm_process(voc_session.src_data, voc_session.src_length);
    }
}

void adpcm_play(uint8_t *pBuffer, uint32_t Size)
{
    int32_t loop;

	OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"total size of adpcm voice is %d\n\n",Size);

    cursor_decode = 0;
    cursor_play = 0;

    loop = Size*8/BUFFER_SIZE;
    if ((Size*8) % BUFFER_SIZE) {
        loop++;
    }

    /* Start adpcm decode process */
    osal_sem_release(sem_adpcm_start);

    while(loop-- > 0){
        /* Try to acquire the next data has been decoded */
        if (wait_for(sem_adpcm_data) < 0) {
            return;
        }    
        
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"adpcm sem_audio_dev is %d\n",sem_audio_dev->value);
        /* Wait until the audio device is idle. */
        if (wait_for(sem_audio_dev) < 0) {
            return;
        }
        //osal_delay(20);
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"play!!\n");
        audio_output((uint16_t *)buffer_voc[cursor_play%BUFFER_COUNT], BUFFER_SIZE);
        cursor_play++;
    }
}

void pcm_play(uint8_t *pBuffer, uint32_t Size)
{
    uint32_t play_size;

	OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"total size of pcm voice is %d\n\n",Size);

    while(Size > 0){
        /* Wait until the audio device is idle. */
        if (wait_for(sem_audio_dev) < 0) {
            return;
        }    

        play_size = (Size > BUFFER_SIZE)? (BUFFER_SIZE):Size;
        audio_output((uint16_t *)pBuffer, play_size);
        pBuffer += play_size;
        Size -= play_size;
    }
}


void rt_play_thread_entry(void *parameter)
{
    osal_status_t err;
    uint8_t *p_msg;
    voc_session_t *session = &voc_session;


    while(1)
    {
        err = osal_queue_recv(queue_play, &p_msg, RT_WAITING_FOREVER);
        if( err == OSAL_STATUS_SUCCESS)
        {
            VOC_STATUS_SET(VOC_STATUS_PLAYING); /* Here we should set it again. */

            memcpy(session, p_msg, sizeof(voc_session_t));
            osal_free(p_msg);

            switch (session->encode_type) {
            case VOC_ENCODE_ADPCM:
                adpcm_play(session->src_data, session->src_length);
                break;

            default:
                pcm_play(session->src_data, session->src_length);
                break;
            }

            osal_sem_take(sem_play_complete,OSAL_WAITING_FOREVER);

            /* Recover to initial value */
            
            osal_sem_set(sem_adpcm_data, 0);
            osal_sem_set(sem_buffer_voc, BUFFER_COUNT);
            osal_sem_set(sem_audio_dev, 1);            
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"finish sem_audio_dev is %d\n",sem_audio_dev->value);
            VOC_STATUS_CLR(VOC_STATUS_MASK);

            if (session->complete_callback) {
                (*session->complete_callback)();
            }
        }
        else
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "%s: rt_mq_recv error [%d]\n", __FUNCTION__, err);         
        }
    }
}

void voc_init(void)
{
    osal_task_t  *thread;

    voc_status = 0;

	sem_adpcm_data = osal_sem_create("sem-play",0);
	osal_assert(sem_adpcm_data != NULL);


    thread = osal_task_create("t-play",
                           rt_play_thread_entry, NULL,
                           RT_VSA_THREAD_STACK_SIZE, RT_PLAY_THREAD_PRIORITY);
    osal_assert(thread != NULL);

    
    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "module initial\n\n");         
}


int voc_play(uint32_t encode_type, uint8_t *data, uint32_t length, voc_handler complete)
{
    int err = OSAL_STATUS_NOMEM;
    voc_session_t *session = NULL;


    session = osal_malloc(sizeof(voc_session_t));
    if (session) 
    {
        session->sample_rate = 8000;
        session->encode_type = encode_type;
        session->src_data = data;
        session->src_length = length&(~3); /* Be sure the length align to 4. */;
        session->played_length = 0;
        session->complete_callback = complete;//NULL;

        err = osal_queue_send(queue_play, session);
    }

    if (err != OSAL_STATUS_SUCCESS) {
        if (session) {
            osal_free(session);
        }
        return err;
    }

    VOC_STATUS_SET(VOC_STATUS_PLAYING);

    return OSAL_STATUS_SUCCESS;
}

void voc_stop(uint32_t b_wait)
{
    if (VOC_STATUS_TST(VOC_STATUS_PLAYING)) {
        VOC_STATUS_SET(VOC_STATUS_STOP);
        if (b_wait) {
            while(VOC_STATUS_TST(VOC_STATUS_PLAYING)) {
                osal_delay(10);
            }
        }
    }
}

