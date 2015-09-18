/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_manager.c
 @brief  : this file include the system manage functions
 @author : wangyifeng
 @history:
           2014-6-20    wangyifeng    Created file
           2015-9-10    gexueyuan    Modified file for syn6288
           ...
******************************************************************************/
#include "cv_osal.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

#include "voc.h"


#define SOUND_PLAY_INTERVAL MS_TO_TICK(200)
#define BIBI_PRE_COUNT 3
#define MAX_VOICE  6  /* MUST BE such value as 2,4,8,16... */
static uint8_t voice[MAX_VOICE];
static uint32_t voice_wr_idx;
static uint32_t voice_rd_idx;
static uint32_t phase = 0;

#define NOTICE_STRING  "soundc"
#define CFCW_VOC "[3]请注意[2]前车[2]距离" //"北京[3]东直门站到了"//
#define CRCW_VOC "后方超车"
#define EEBL_VOC "前方急刹"
#define VBD_VOC "[2]前方有故障车"  

/*
void voc_stop(void)
{
    syn6288_stop();
}
*/
static void sound_play_complete(void)
{
    sys_envar_t *p_sys = &cms_envar.sys;
    if (phase != 0) {
        /* Alert has been stopped. */
        osal_timer_change(p_sys->timer_voc, SOUND_PLAY_INTERVAL);
        osal_timer_start(p_sys->timer_voc);
    }
}

static void notice_di_play_once(void *complete)
{
    //syn6288_play(NOTICE_STRING);

    //sound_play_complete();
    
    voc_play(VOC_ENCODE_ADPCM, NOTICE_STRING, 0, (voc_handler)complete);
}

static void voice_play_once(uint32_t alert_type, void *complete)
{
    uint8_t *data;
    uint32_t length = 0;

    switch (alert_type) {
               
    case HI_OUT_CRD_ALERT:
        data = CFCW_VOC;
        break;

    case HI_OUT_CRD_REAR_ALERT:
        data = CRCW_VOC;
        break;

    case HI_OUT_VBD_ALERT:
        data = VBD_VOC;
        break;

    case HI_OUT_EBD_ALERT:
        data = EEBL_VOC;
        break;
    
    default:

        break;
    }

   voc_play(VOC_ENCODE_ADPCM, data, length, (voc_handler)complete);
   //syn6288_play(data);
}

void sound_notice_di(void)
{
    notice_di_play_once(NULL);
}

void sound_alert_start(uint32_t alert_type)
{
    if (alert_type != HI_OUT_NONE) {
        voice[voice_wr_idx] = alert_type;
        if (++voice_wr_idx >= MAX_VOICE) {
            voice_wr_idx = 0;
        }
    }

    if (phase == 0) {
        osal_enter_critical();
        phase = 1;
        osal_leave_critical();
        notice_di_play_once(sound_play_complete);
    }
}
void alert_start(uint32_t alert_type)
{
    sound_alert_start(alert_type);
    //voice_play_once(alert_type,NULL);
}
FINSH_FUNCTION_EXPORT(alert_start, alert_start);

void sound_alert_stop()
{
    osal_enter_critical();
    phase = 0;
    osal_leave_critical();

    voc_stop();
    syn6288_stop();
    memset(voice, 0, sizeof(voice));
    voice_wr_idx = 0;
    voice_rd_idx = 0;
}
void alert_stop(void)
{
    sound_alert_stop();
}
FINSH_FUNCTION_EXPORT(alert_stop, alert_stop);

void sound_alert_process(void* parameter)
{
    if (phase == 0) {
        /* Do nothing because alert has been stopped. */
    }
    else if (phase < BIBI_PRE_COUNT) {
        notice_di_play_once(sound_play_complete);
        osal_enter_critical();
        phase++;
        osal_leave_critical();
    }
    else {
        if (voice[voice_rd_idx] != 0) {
            voice_play_once(voice[voice_rd_idx], sound_play_complete);
            voice[voice_rd_idx] = 0;
            if (++voice_rd_idx >= MAX_VOICE) {
                voice_rd_idx = 0;
            }
        }
        else {
            notice_di_play_once(sound_play_complete);
        }
    }
}

