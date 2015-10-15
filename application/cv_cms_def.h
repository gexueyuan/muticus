/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_cms_def.h
 @brief  : This file include the cms global definition 
 @author : wangyifeng
 @history:
           2014-6-16    wangyifeng    Created file
           ...
******************************************************************************/
#ifndef __CV_CMS_DEF_H__
#define __CV_CMS_DEF_H__

#include "cv_wnet.h"
#include "cv_vam.h"
#include "cv_vsa.h"
#include "gsensor.h"
#include "cv_mda.h"
#include "led.h"
#include "voc.h"

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

/**
    prority of all the tasks in system 
*/
#define RT_INIT_THREAD_PRIORITY		10   /* highest, to ensure all initial 
                                            will be complete before running*/
#define RT_USBEMU_THREAD_PRIORITY	13
#define RT_USBH_THREAD_PRIORITY		12 //26
#define RT_SYS_THREAD_PRIORITY		15
#define RT_VAM_THREAD_PRIORITY		22
#define RT_VSA_THREAD_PRIORITY		23
#define RT_MDA_THREAD_PRIORITY		19

#define RT_GPS_THREAD_PRIORITY		21
#define RT_MEMS_THREAD_PRIORITY		22
#define RT_HI_THREAD_PRIORITY		25
#define RT_ADPCM_THREAD_PRIORITY	26
#define RT_PLAY_THREAD_PRIORITY	    24
#define RT_KEY_THREAD_PRIORITY      18

#define RT_WNETTX_THREAD_PRIORITY   20
#define RT_WNETRX_THREAD_PRIORITY   20

#define QC_INIT_THREAD_PRIORITY         (10)
#define QC_PHP_THREAD_PRIORITY         (27)



#define RT_THREAD_STACK_SIZE   (1024*2)

#define RT_INIT_THREAD_STACK_SIZE   (1024*2)
#define RT_USBH_THREAD_STACK_SIZE   (1024*2)
#define RT_SYS_THREAD_STACK_SIZE   (1024*1)
#define RT_GPS_THREAD_STACK_SIZE   (1024*1)
#define RT_MEMS_THREAD_STACK_SIZE   (1024*2)
#define RT_VAM_THREAD_STACK_SIZE   (1024*2)
#define RT_VSA_THREAD_STACK_SIZE   (1024*2)
#define RT_ADPCM_THREAD_STACK_SIZE   (1024*1)
#define RT_PLAY_THREAD_STACK_SIZE   (1024*1)
#define RT_HI_THREAD_STACK_SIZE   (1024*1)
#define RT_KEY_THREAD_STACK_SIZE    (512)

#define QC_INIT_THREAD_STACK_SIZE   (1024*2)
#define QC_PHP_THREAD_STACK_SIZE   (1024*2)


/**
    size of all the queue in system 
*/
#define SYS_QUEUE_SIZE 16
#define VAM_QUEUE_SIZE 16
#define VSA_QUEUE_SIZE 16
#define VOC_QUEUE_SIZE  4 


enum SYSTEM_MSG_TYPE
{
    SYS_MSG_BASE = 0x0000,
    SYS_MSG_INITED,
    SYS_MSG_KEY_PRESSED,
    SYS_MSG_KEY_RELEASED,
    SYS_MSG_START_ALERT,
    SYS_MSG_STOP_ALERT,
    SYS_MSG_ALARM_ACTIVE,
    SYS_MSG_ALARM_CANCEL,
    SYS_MSG_GPS_UPDATE,
    SYS_MSG_BSM_UPDATE,
    SYS_MSG_HI_IN_UPDATE,
    SYS_MSG_HI_OUT_UPDATE,
    SYS_MSG_XXX,

    VAM_MSG_BASE = 0x0200,
    VAM_MSG_START,
    VAM_MSG_STOP,
    VAM_MSG_RCPTX,
    VAM_MSG_RCPRX,
    VAM_MSG_NEIGH_TIMEOUT,
    VAM_MSG_GPSDATA,


    VSA_MSG_BASE = 0x0300,
    VSA_MSG_MANUAL_BC,   
    VSA_MSG_EEBL_BC,
    VSA_MSG_AUTO_BC,
    
    VSA_MSG_CFCW_ALARM,      /* Close front warning. */
    VSA_MSG_CRCW_ALARM,      /* Close rear warning. */
    VSA_MSG_OPPOSITE_ALARM, 
    VSA_MSG_SIDE_ALARM,

    VSA_MSG_ACC_RC,
    VSA_MSG_EEBL_RC,
    VSA_MSG_X_RC,
    VSA_MSG_XX_RC,
    VSA_MSG_XXX_RC
};

enum HI_OUT_TYPE
{
    HI_OUT_NONE = 0,
    HI_OUT_SYS_INIT,
    
    HI_OUT_BSM_UPDATE,
    HI_OUT_BSM_NONE,
    
    HI_OUT_GPS_CAPTURED,
    HI_OUT_GPS_LOST,
    
    HI_OUT_CRD_ALERT,         /* Ω¸æ‡¿Î */
    HI_OUT_CRD_CANCEL,
    HI_OUT_CRD_REAR_ALERT,    /* ∫Û∑ΩΩ¸æ‡¿Î */
    HI_OUT_CRD_REAR_CANCEL,
    
    HI_OUT_VBD_ALERT,         /* π ’œ */
    HI_OUT_VBD_CANCEL,
    HI_OUT_VBD_STATUS,
    HI_OUT_VBD_STOP,
    
    HI_OUT_EBD_ALERT,         /* ΩÙº±…≤≥µ */
    HI_OUT_EBD_CANCEL,
    HI_OUT_EBD_STATUS,
    HI_OUT_EBD_STOP,

    HI_OUT_EVA_ALERT,
    HI_OUT_EVA_CANCEL,

    HI_OUT_RSA_ALERT,
    HI_OUT_RSA_CANCEL,
    
    HI_OUT_CANCEL_ALERT,
};

enum HI_IN_TYPE
{
    HI_IN_NONE = 0,
    HI_IN_KEY_PRESSED,
    HI_IN_KEY_RELEASE,
};


/**
    misc definitions 
*/
#define MS_TO_TICK(n)     ((n)*RT_TICK_PER_SECOND/1000)
#define SECOND_TO_TICK(n) ((n)*RT_TICK_PER_SECOND)


/*****************************************************************************
 * declaration of structs                                                    *
*****************************************************************************/

/* Structure of system global message.*/
typedef struct _sys_msg_st
{
    uint16_t     id;
    uint16_t    len;
    uint32_t   argc; 
    void      *argv;
    
}sys_msg_st, *sys_msg_st_ptr;

#define SYS_MSG_ST_LEN    (sizeof(sys_msg_st))


/** structure of system configure parameters. */
typedef struct _cfg_param{

    uint8_t pid[RCP_TEMP_ID_LEN];

    vam_config_t vam;
    vsa_config_t vsa;
    gsnr_config_t gsnr;
    wnet_config_t wnet;

    uint8_t print_xxx;  /* 0 - disable, 1 - enable */
    
    /******************** AUDIO*********************/
    voc_config_t voc;
    
    /******************** CALIBRATION *********************/
}cfg_param_t;

/** 
    structure of system manager module's environment variable 
*/
typedef struct _sys_envar{
    /* working_param */
    vsa_config_t working_param;

    uint32_t status;

    uint32_t hi_timer_cnt;

    Led_Color led_color;
    Led_State led_state;
    uint8_t led_period;

	uint8_t voc_flag;

    /* os related */
    osal_task_t *task_sys_mng;
    osal_queue_t *queue_sys_mng;

    osal_task_t *task_sys_hi;
    osal_queue_t *queue_sys_hi;

    osal_timer_t *timer_voc;
    osal_timer_t *timer_cpuusage;

    osal_timer_t *timer_blink;   
}sys_envar_t;


/* structure of system global environment variable. */
typedef struct _cms_global
{
    vam_envar_t   vam;
    vsa_envar_t   vsa;
    wnet_envar_t wnet;

    sys_envar_t   sys;
    mda_envar_t   mda;
    
}cms_global_t;


static __inline uint16_t cv_ntohs(uint16_t s16)
{
	uint16_t ret = 0;
	uint8_t *s, *d;

	#ifdef BIG_ENDIAN	
	ret = s16;
	#else
	s = (uint8_t *)(&s16);
	d = (uint8_t *)(&ret) + 1;
	#endif

	*d-- = *s++;
	*d-- = *s++;

	return ret;
}

static __inline uint32_t cv_ntohl(uint32_t l32)
{
	uint32_t ret = 0;
	uint8_t *s, *d;

	#ifdef BIG_ENDIAN	
	ret = l32;
	#else
	s = (uint8_t *)(&l32);
	d = (uint8_t *)(&ret) + 3;
	#endif

	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;

	return ret;
}

static __inline float cv_ntohf(float f32)
{
	float ret;
	uint8_t *s, *d;

	#ifdef BIG_ENDIAN	
	ret = f32;
	#else
	s = (uint8_t *)(&f32);
	d = (uint8_t *)(&ret) + 3;
	#endif

	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;
	*d-- = *s++;

	return ret;
}


/*****************************************************************************
 * declare of global functions and variables                                 *
*****************************************************************************/
extern cms_global_t cms_envar;
extern cfg_param_t cms_param;

osal_status_t sys_add_event_queue(sys_envar_t *p_sys, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv);
osal_status_t hi_add_event_queue(sys_envar_t *p_sys, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv);


#endif /* __CV_CMS_DEF_H__ */

