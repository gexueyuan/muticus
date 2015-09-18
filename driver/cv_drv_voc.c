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
#include "syn6288.h"

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
    //Pt8211_AUDIO_Play(pBuffer, Size);
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
        //adpcm_de((char *)pBuffer,buffer_voc[cursor_decode%BUFFER_COUNT],decode_size);
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
    uint8_t  state; 

    while(1)
    {
        err = osal_queue_recv(queue_play, &p_msg, RT_WAITING_FOREVER);
        if( err == OSAL_STATUS_SUCCESS)
        {
            VOC_STATUS_SET(VOC_STATUS_PLAYING); /* Here we should set it again. */

            memcpy(session, p_msg, sizeof(voc_session_t));
            osal_free(p_msg);
            state = syn6288_state();
            while(state){

                rt_thread_delay(10);

                state = syn6288_state();
            }
            if(VOC_STATUS_TST(VOC_STATUS_PLAYING)){
                
                osal_printf("out put string is %s\n",session->src_data);
                syn6288_play(session->src_data);
                }
                      
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
    
    syn6288_hw_init();
    
	sem_adpcm_data = osal_sem_create("sem-play",0);
	osal_assert(sem_adpcm_data != NULL);

    queue_play = osal_queue_create("q-play",  VOC_QUEUE_SIZE);
    osal_assert(queue_play != NULL);
    
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

