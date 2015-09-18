/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_manager.c
 @brief  : this file include the system manage functions
 @author : wangyifeng
 @history:
           2014-6-20    wangyifeng    Created file
           ...
******************************************************************************/
//#pragma O3


#include "cv_osal.h"

#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "sysc"
#include "cv_osal_dbg.h"
OSAL_DEBUG_ENTRY_DEFINE(sysc)

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

#include "led.h"
#include "key.h"
#include "cv_vsa.h"

#include "app_interface.h"


void null_space(void)
{
}


#define HUMAN_ITERFACE_DEFAULT         MS_TO_TICK(500)

#define HUMAN_ITERFACE_VOC             SECOND_TO_TICK(3)

#define HUMAN_ITERFACE_GPS_VOC         SECOND_TO_TICK(5)

#define BREATH_CYCLE                   300 


#define AUDIO_START_ADDRESS     58 /* Offset relative to audio file header size */

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

void sound_alert_process(void* parameter);
extern void led_on(Led_TypeDef led);
extern void led_off(Led_TypeDef led);
extern void led_blink(Led_TypeDef led);

extern int param_set(uint8_t param, int32_t value);

extern void syn6288_play(char *txt);
extern void set_voc(void);
/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/


osal_status_t sys_add_event_queue(sys_envar_t *p_sys, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv)
{
    int err = OSAL_STATUS_NOMEM;
    sys_msg_t *p_msg;

    p_msg = osal_malloc(sizeof(sys_msg_t));
    if (p_msg) {
        p_msg->id = msg_id;
        p_msg->len = msg_len;
        p_msg->argc = msg_argc;
        p_msg->argv = msg_argv;
        err = osal_queue_send(p_sys->queue_sys_mng, p_msg);
    }

    if (err != OSAL_STATUS_SUCCESS) {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "%s: failed=[%d], msg=%04x\n",\
                           __FUNCTION__, err, msg_id);
        osal_free(p_msg);                   
    }

    return err;
}

osal_status_t hi_add_event_queue(sys_envar_t *p_sys, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv)
{
    osal_status_t err = OSAL_STATUS_NOMEM;
    sys_msg_t *hi_msg;
    hi_msg = osal_malloc(sizeof(sys_msg_t));
    if (hi_msg) {
        hi_msg->id = msg_id;
        hi_msg->len = msg_len;
        hi_msg->argc = msg_argc;
        hi_msg->argv = msg_argv;
        err = osal_queue_send(p_sys->queue_sys_hi, hi_msg);
    }

    if (err != OSAL_STATUS_SUCCESS) {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "%s: failed=[%d], msg=%04x\n",\
                           __FUNCTION__, err, msg_id);
        osal_free(hi_msg);                   
    }

    return err;
}

void sys_manage_proc(sys_envar_t *p_sys, sys_msg_t *p_msg)
{
    uint32_t type = 0; 
    static uint8_t keycnt = 0xff;
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    
    switch(p_msg->id)
    {
        case SYS_MSG_INITED:
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "%s: initialize complete\n", __FUNCTION__);
            vam_start();
            vsa_start();
            hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,HI_OUT_SYS_INIT, 0);
            break;
        }
    case SYS_MSG_BSM_UPDATE:
        hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,p_msg->argc, 0);            
        break;
        
    case SYS_MSG_KEY_PRESSED:
        if(p_msg->argc == C_UP_KEY){   
            //rt_kprintf("gsnr param is resetting .....\n");
            //param_set(19,0);                   
         }
        else if(p_msg->argc == C_DOWN_KEY){
            vsa_add_event_queue(p_vsa, VSA_MSG_MANUAL_BC, 0,keycnt,NULL);
            keycnt = ~keycnt;                 
        }
        break;
    case SYS_MSG_START_ALERT:
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "%s:alert start!!!.\n", __FUNCTION__);

        if (p_msg->argc == VSA_ID_CRD){
            type = HI_OUT_CRD_ALERT;
        }
        else if (p_msg->argc == VSA_ID_CRD_REAR){
            type = HI_OUT_CRD_REAR_ALERT;
        }
        else if (p_msg->argc == VSA_ID_VBD){
            type = HI_OUT_VBD_ALERT;
        }
        else if (p_msg->argc == VSA_ID_EBD){
            type = HI_OUT_EBD_ALERT;
        }  
        hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,type, 0);
        break;
        
    case SYS_MSG_STOP_ALERT:
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "%s:alert stop.\n", __FUNCTION__);
                
        if (p_msg->argc == VSA_ID_CRD){
            type = HI_OUT_CRD_CANCEL;
        }
        if (p_msg->argc == VSA_ID_CRD_REAR){
            type = HI_OUT_CRD_REAR_CANCEL;
        }
        else if (p_msg->argc == VSA_ID_VBD){
            type = HI_OUT_VBD_CANCEL;
        }
        else if (p_msg->argc == VSA_ID_EBD){
            type = HI_OUT_EBD_CANCEL;
        }
        //don't distinguish  message of  alert canceling   for the time being
        hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,type, 0);
        break;
        
    case SYS_MSG_ALARM_ACTIVE:
        
        if (p_msg->argc == VSA_ID_VBD){
            type = HI_OUT_VBD_STATUS;
        }
        else if (p_msg->argc == VSA_ID_EBD){
            type = HI_OUT_EBD_STATUS;
        }
        hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,type, 0);                
        break;
            
    case SYS_MSG_ALARM_CANCEL:
        if (p_msg->argc == VSA_ID_VBD){
            type = HI_OUT_VBD_STOP;
        }
        else if (p_msg->argc == VSA_ID_EBD){
            type = HI_OUT_EBD_STOP;
        }
        hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,type, 0);
        break;        

    case SYS_MSG_GPS_UPDATE:
        hi_add_event_queue(p_sys, SYS_MSG_HI_OUT_UPDATE,0,\
            ((p_msg->argc == 0)? HI_OUT_GPS_LOST:HI_OUT_GPS_CAPTURED) , 0);
        break;
    
    default:
        break;
    }
}

void sysc_thread_entry(void *parameter)
{
    int err;
    sys_msg_t *p_msg;
    sys_envar_t *p_sys = (sys_envar_t *)parameter;

    while(1)
    {
        err = osal_queue_recv(p_sys->queue_sys_mng, &p_msg, OSAL_WAITING_FOREVER);
        if (err == OSAL_STATUS_SUCCESS)
        {
            sys_manage_proc(p_sys, p_msg);
            osal_free(p_msg);
        }
        else
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_ERROR, "%s: osal_queue_recv error [%d]\n", __FUNCTION__, err);
        }
    }
}


void sys_human_interface_proc(sys_envar_t *p_sys, sys_msg_t *p_msg)
{
    Led_Color led_color;
    Led_State led_state;
    uint8_t led_priod = 0;
    uint32_t alarm_state;


    alarm_state = vsa_get_alarm(VSA_ID_NONE);
    
    if (p_msg->id == SYS_MSG_HI_OUT_UPDATE)
    {
        switch(p_msg->argc)
        {
            case HI_OUT_SYS_INIT:
            {
                //sound_notice_di();\
                //syn6288_play("[1]soundb");
                //osal_delay(50);
                set_voc();
                p_sys->status |= 1<<HI_OUT_GPS_LOST;
                break;
            }

        case HI_OUT_BSM_UPDATE:
            p_sys->status |= 1<<HI_OUT_BSM_UPDATE;
            break;
        case HI_OUT_BSM_NONE:
            p_sys->status &= ~(1<<HI_OUT_BSM_UPDATE);
            break;                
            
        case HI_OUT_CRD_ALERT:
        {
            //sound_alert_start(HI_OUT_CRD_ALERT);
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI CFCW alert start!!\n\n");
            if(p_sys->status&(1<<HI_OUT_CRD_ALERT))
            {
                return;
            }
            else
            {
                p_sys->status |= 1<<HI_OUT_CRD_ALERT;
            }
            break;                
        }

        case HI_OUT_CRD_REAR_ALERT:            
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI CRCW alert start!!\n\n");
            //sound_alert_start(HI_OUT_CRD_REAR_ALERT);
            if(p_sys->status&(1<<HI_OUT_CRD_REAR_ALERT)){
                return;
            }
            else{
                p_sys->status |= 1<<HI_OUT_CRD_REAR_ALERT;
            }    
            break;
            
        case HI_OUT_VBD_ALERT:  

            //sound_alert_start(HI_OUT_VBD_ALERT);
            if(p_sys->status&(1<<HI_OUT_VBD_ALERT)){
                return;
            }
            else{
                p_sys->status |= 1<<HI_OUT_VBD_ALERT;
                OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI vbd alert!!\n\n");
            }    
            break;                

        case HI_OUT_EBD_ALERT:
           
            //sound_alert_start(HI_OUT_EBD_ALERT);
            if(p_sys->status&(1<<HI_OUT_EBD_ALERT)){
                return;
            }
            else{
                p_sys->status |= 1<<HI_OUT_EBD_ALERT;
                OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI EEBL alert!!\n\n");
            }
            break;

        case HI_OUT_CRD_CANCEL:
            
            if(alarm_state == 0) {
                //sound_alert_stop();
            }
            p_sys->status &= ~(1<<HI_OUT_CRD_ALERT);               
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI CFCW  alert cancel!!\n\n");

            break;

        case HI_OUT_CRD_REAR_CANCEL:
            
            if(alarm_state == 0) {
                //sound_alert_stop();
            }
            p_sys->status &= ~(1<<HI_OUT_CRD_REAR_ALERT);
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI CRCW alert cancel!!\n\n");

            break;    

        case HI_OUT_VBD_CANCEL:

            if(alarm_state == 0) {
                //sound_alert_stop();
            }
            p_sys->status &= ~(1<<HI_OUT_VBD_ALERT);
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI vbd alert cancel!!\n\n");
            break;

        case HI_OUT_EBD_CANCEL://cancel alarm
       
            if(alarm_state == 0) {
                //sound_alert_stop();
            }
            p_sys->status &= ~(1<<HI_OUT_EBD_ALERT);
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"HI EEBL alert  cancel!!\n\n");
            break;
            
        case HI_OUT_VBD_STATUS:
            p_sys->status |= 1<<HI_OUT_VBD_STATUS;
            break;
        case HI_OUT_EBD_STATUS:
            p_sys->status |= 1<<HI_OUT_EBD_STATUS;
            break;

        case HI_OUT_VBD_STOP:
            p_sys->status &= ~(1<<HI_OUT_VBD_STATUS);//stop broadcast
            break;
        case HI_OUT_EBD_STOP:
            p_sys->status &= ~(1<<HI_OUT_EBD_STATUS);//stop broadcast
            break;    

            
        case HI_OUT_CANCEL_ALERT:
            if(alarm_state == 0) {
                //sound_alert_stop();
            }
            p_sys->status &= ~((1<<HI_OUT_EBD_ALERT)|(1<<HI_OUT_VBD_ALERT)|(1<<HI_OUT_CRD_ALERT));
            break;

        case HI_OUT_GPS_LOST:
            p_sys->status |= 1<<HI_OUT_GPS_LOST;            
            
            break;

        case HI_OUT_GPS_CAPTURED:
            p_sys->status &= ~(1<<HI_OUT_GPS_LOST);
            break;

        default:
            break;
        }
    }
    else if (p_msg->id == SYS_MSG_HI_IN_UPDATE)
    {
        
        switch(p_msg->argc){
        case HI_IN_KEY_PRESSED:
            sys_add_event_queue(p_sys,SYS_MSG_KEY_PRESSED, 0, p_msg->len, NULL);
            break;
        default:
            break;
        }

    }

    if((p_sys->status&(1<<HI_OUT_CRD_ALERT))||(p_sys->status&(1<<HI_OUT_CRD_REAR_ALERT))\
        ||(p_sys->status&(1<<HI_OUT_EBD_ALERT))||(p_sys->status&(1<<HI_OUT_VBD_ALERT))){
        led_color = RED;//
        led_state = LED_BLINK;
        led_priod = MS_TO_TICK(150);
    }
    else if((p_sys->status&(1<<HI_OUT_EBD_STATUS))||(p_sys->status&(1<<HI_OUT_VBD_STATUS))){
        led_color = YELLOW;//
        led_state = LED_BLINK;
        led_priod = MS_TO_TICK(150);
    }

    else if(p_sys->status&(1<<HI_OUT_GPS_LOST)){
        led_color = BLUE;//
        led_state = LED_BLINK;
        led_priod = MS_TO_TICK(1000);
     }
    else if(p_sys->status&(1<<HI_OUT_BSM_UPDATE)){
        led_color = BLUE;//
        led_state = LED_ON;
        led_priod = MS_TO_TICK(150);
    }
    else if(!(p_sys->status&(1<<HI_OUT_BSM_UPDATE))){
        led_color = BLUE;//
        led_state = LED_ON;
        led_priod = MS_TO_TICK(500);
    }
    if((p_sys->led_color != led_color)||(p_sys->led_state != led_state)||(p_sys->led_period != led_priod)){
        p_sys->led_color = led_color;
        p_sys->led_state = led_state;
        p_sys->led_period = led_priod;
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"led %d is %d\n",led_color,led_state);
        led_proc(p_sys->led_color, p_sys->led_state,p_sys->led_period);
    }
}


void rt_hi_thread_entry(void *parameter)
{
    osal_status_t err;
    sys_msg_t msg, *p_msg = &msg;
    sys_envar_t *p_sys = (sys_envar_t *)parameter;


    while(1)
    {
        err = osal_queue_recv(p_sys->queue_sys_hi, &p_msg, OSAL_WAITING_FOREVER);
        if (err == OSAL_STATUS_SUCCESS)
        {
            sys_human_interface_proc(p_sys, p_msg);
            osal_free(p_msg);
        }
        else
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_ERROR, "%s: osal_queue_recv error [%d]\n", __FUNCTION__, err);           
            osal_free(p_msg);
        }
    }
}


void sys_init(void)
{
    sys_envar_t *p_sys = &cms_envar.sys;
    /* object for sys */
    p_sys->queue_sys_mng = osal_queue_create("q-sys", SYS_QUEUE_SIZE);
    osal_assert(p_sys->queue_sys_mng != NULL);

    p_sys->task_sys_mng = osal_task_create("t-sys",
                           sysc_thread_entry, p_sys,
                           RT_SYS_THREAD_STACK_SIZE, RT_SYS_THREAD_PRIORITY);
    osal_assert(p_sys->task_sys_mng != NULL);

    /* object for human interface */
    p_sys->queue_sys_hi = osal_queue_create("q-hi", SYS_QUEUE_SIZE);
    osal_assert(p_sys->queue_sys_mng != NULL);


    p_sys->timer_voc = osal_timer_create("tm-voc",sound_alert_process, p_sys,\
        MS_TO_TICK(500),FALSE);                     
    osal_assert( p_sys->timer_voc != NULL);
    //osal_timer_start(p_sys->timer_voc);

    p_sys->task_sys_hi = osal_task_create("t-hi",
                           rt_hi_thread_entry, p_sys,
                           RT_HI_THREAD_STACK_SIZE, RT_HI_THREAD_PRIORITY);
    osal_assert(p_sys->task_sys_hi != NULL);

    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "module initial\n");




    
}


