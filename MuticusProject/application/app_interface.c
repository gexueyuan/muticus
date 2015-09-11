/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : app_interface.c
 @brief  : this file include the application functions for the interface.
 @author : wangxianwen
 @history:
           2015-7-15    wangxianwen    Created file
           ...
******************************************************************************/

#include "app_interface.h"

#include "drv_main.h"
#include "dat_lcd_alarm.h"
#include "dat_lcd_font.h"

#include "cv_cms_def.h"


/* User interface thread structure. */
ui_thread_st UiThread = { NULL, NULL };




/**
  * @brief  Add message to queue for interface thread.
  * @param  See below.
  * @retval None.
  */
osal_status_t ui_add_msg_queue
(
    /* Pointer to message data. */
    ui_msg_st_ptr       msg_ptr
)
{
    osal_status_t         err = OSAL_STATUS_NOMEM;
    ui_msg_st_ptr msg_mem_ptr = NULL;
    

    /* Error detection. */
    if ((UiThread.msg_queue_ptr == NULL) || (msg_ptr == NULL))
    {
        err = OSAL_STATUS_ERROR_UNDEFINED;
        return err;
    }

    /* Allocate memory for message. */    
    if ((msg_mem_ptr = osal_malloc(UI_MSG_ST_LEN)) != NULL) 
    {
        /* Send message to queue. */
        memcpy(msg_mem_ptr, msg_ptr, UI_MSG_ST_LEN);
        err = osal_queue_send(UiThread.msg_queue_ptr, msg_mem_ptr);

        /* Free message memory when error. */
        if(err != OSAL_STATUS_SUCCESS)
        {
            osal_free(msg_mem_ptr); 
        }
    }
    else
    {
        err = OSAL_STATUS_NOMEM;
    }

    /* Error report. */
    if (err != OSAL_STATUS_SUCCESS) 
    {
        OSAL_MODULE_DBGPRT(UI_THREAD_NAME, OSAL_DEBUG_WARN, "%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, msg_ptr->msg_id);
    }

    return err;
}




/**
  * @brief  Fresh alart infor to lcd.
  * @param  See below.
  * @retval None.
  */
static void ui_fresh_alart_infor
(
    /* Picture index in alarm group. */
    uint16_t pic_index
)
{
    drv_lcd_block_draw_st alart_infor = { LCD_FOREGROUND_LAYER, 200, 160, &PicAlarmGroup[pic_index] };


    /* Fresh alart information to lcd buffer.. */
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_BLOCK, &alart_infor);
}







/**
  * @brief  Message queue handler routine for interface thread.
  * @param  See below.
  * @retval None.
  */
static void ui_msg_queue_handler
(
    /* Pointer to ui thread. */
    ui_thread_st_ptr thread_ptr,

    /* Pointer to message data. */
    ui_msg_st_ptr    msg_ptr
)
{
    /* Error detection. */
    if((thread_ptr == NULL) || (msg_ptr == NULL))
    {
        return;
    }

    /*  */
    switch(msg_ptr->msg_id)
    {
        case UI_MSG_ALART_AC:
            
        case UI_MSG_BRAKING_FA:
        case UI_MSG_BRAKING_FC:
        case UI_MSG_BRAKING_RA:
        case UI_MSG_BRAKING_RC:    { ui_fresh_alart_infor(PIC_INDEX_FRONT_VEC_BRAKE); break;}
            
        case UI_MSG_NEAR_DIS_FA:
        case UI_MSG_NEAR_DIS_FC:
        case UI_MSG_NEAR_DIS_RA:
        case UI_MSG_NEAR_DIS_RC:   { ui_fresh_alart_infor(PIC_INDEX_ATT_FRONT_VEC); break;}
            
        case UI_MSG_BREAKDOWN_FA:
        case UI_MSG_BREAKDOWN_FC:
        case UI_MSG_BREAKDOWN_RA:
        case UI_MSG_BREAKDOWN_RC:  { ui_fresh_alart_infor(PIC_INDEX_FRONT_VEC_FAULTY); break;}
            
        case UI_MSG_ACCIDENT_FA:
        case UI_MSG_ACCIDENT_FC:
        case UI_MSG_ACCIDENT_RA:
        case UI_MSG_ACCIDENT_RC:   { ui_fresh_alart_infor(PIC_INDEX_FRONT_ACCIDENT); break;}

        default: { break; }        
    }
}


/**
  * @brief  Entry routine for user interface thread.
  * @param  See below.
  * @retval None.
  */
static void ui_thread_entry
(
    /* Pointer to ui thread. */
    ui_thread_st_ptr thread_ptr
)
{
    osal_status_t     err = OSAL_STATUS_SUCCESS;
    ui_msg_st_ptr msg_ptr = NULL;


    sys_envar_t *sys_envar_ptr = &p_cms_envar->sys;



    while(1)
    {



#if 0

        if(sys_envar_ptr->status & (1<<HI_OUT_VBD_ALERT))
        {
            ui_fresh_alart_infor(PIC_INDEX_FRONT_VEC_FAULTY);
        }

        osal_delay(100);

#else

    
        /* Waiting for message forever. */
        err = osal_queue_recv(thread_ptr->msg_queue_ptr, &msg_ptr, OSAL_WAITING_FOREVER);
        if (err == OSAL_STATUS_SUCCESS)
        {
            ui_msg_queue_handler(thread_ptr, msg_ptr);
        }
        else
        {
            OSAL_MODULE_DBGPRT(UI_THREAD_NAME, OSAL_DEBUG_ERROR, "%s: osal_queue_recv error [%d]\n", __FUNCTION__, err);           
        }

        /* Free the message data memory when not null. */
        if (msg_ptr != NULL)
        {
            osal_free(msg_ptr);
            msg_ptr = NULL;
        } 
#endif
        
    }
}


/**
  * @brief  Initialize user interface thread.
  * @param  None.
  * @retval None.
  */
void ui_thread_init
( 
    /* No parameter. */
    void
)
{
    /* Initialize message queue pointer. */
    UiThread.msg_queue_ptr = osal_queue_create("ui_msgqueue", UI_THREAD_QUEUE_SIZE);
    osal_assert(UiThread.msg_queue_ptr != NULL);

    /* Initialize task pointer. */
    UiThread.task_ptr = osal_task_create(UI_THREAD_NAME, (void (*)(void *))ui_thread_entry, &UiThread, UI_THREAD_STACK_SIZE, UI_THREAD_PRIORITY);
    osal_assert(UiThread.task_ptr != NULL);
    
}


