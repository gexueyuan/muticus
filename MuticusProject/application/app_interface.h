/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : app_interface.h
 @brief  : this file include the application variables and functions prototypes 
           for the interface.
 @author : wangxianwen
 @history:
           2015-7-15    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _APP_INTERFACE_H_
#define _APP_INTERFACE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

     
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "cv_osal.h"
#include "cv_osal_dbg.h"



/* Module name for debug. */ 
#define UI_THREAD_NAME          "ui-thread"

/* Thread stack size and priority. */
#define UI_THREAD_STACK_SIZE    (1024 * 2)
#define UI_THREAD_PRIORITY      24

/* Thread message queue size. */
#define UI_THREAD_QUEUE_SIZE    16
     


/* User interface message type. */
typedef enum _ui_msg_type
{
    UI_MSG_ALART_AC,       /* Message: Cancel all the alart. AC: All Cancel. */
    
    UI_MSG_BRAKING_FA,     /* Message: Active Braking in front of local. FA: Front Active. */
    UI_MSG_BRAKING_FC,     /* Message: Cancel Braking in front of local. FC: Front Cancel. */
    UI_MSG_BRAKING_RA,     /* Message: Active Braking the rear of local. RA: Rear Active. */
    UI_MSG_BRAKING_RC,     /* Message: Cancel Braking the rear of local. RC: Rear Cancel. */
    
    UI_MSG_NEAR_DIS_FA,    /* Message: Active Near distance in front of local. */
    UI_MSG_NEAR_DIS_FC,    /* Message: Cancel Near distance in front of local. */
    UI_MSG_NEAR_DIS_RA,    /* Message: Active Near distance the rear of local. */
    UI_MSG_NEAR_DIS_RC,    /* Message: Cancel Near distance the rear of local. */

    UI_MSG_BREAKDOWN_FA,   /* Message: Active Breakdown in front of local. */
    UI_MSG_BREAKDOWN_FC,   /* Message: Cancel Breakdown in front of local. */
    UI_MSG_BREAKDOWN_RA,   /* Message: Active Breakdown the rear of local. */
    UI_MSG_BREAKDOWN_RC,   /* Message: Cancel Breakdown the rear of lcoal. */

    UI_MSG_ACCIDENT_FA,    /* Message: Active Accident in front of local. */
    UI_MSG_ACCIDENT_FC,    /* Message: Cancel Accident in front of local. */
    UI_MSG_ACCIDENT_RA,    /* Message: Active Accident the rear of local. */
    UI_MSG_ACCIDENT_RC,    /* Message: Cancel Accident the rear of local. */
    
}UI_MSG_TYPE, *UI_MSG_TYPE_PTR;

#define UI_MSG_TYPE_LEN    (sizeof(UI_MSG_TYPE))

    
/* User interface message structure. */     
typedef struct _ui_msg_st
{
    UI_MSG_TYPE  msg_id;
    
}ui_msg_st, *ui_msg_st_ptr;

#define UI_MSG_ST_LEN    (sizeof(ui_msg_st))





/* Thread structure for user interface module. */
typedef struct _ui_thread_st
{	
    /************ Public objects.***************/

    /* Osal task pointer. */
    osal_task_st_ptr       task_ptr;

    /* Osal message queue pointer. */
    osal_queue_st_ptr msg_queue_ptr;


    /************ Private objects.*************/    


	
}ui_thread_st, *ui_thread_st_ptr;

#define UI_THREAD_ST_LEN    (sizeof(ui_thread_st))
     


/**
  * @brief  Add message to queue for interface thread.
  * @param  See below.
  * @retval None.
  */
extern osal_status_t ui_add_msg_queue
(
    /* Pointer to message data. */
    ui_msg_st_ptr       msg_ptr
);


/**
  * @brief  Initialize user interface thread.
  * @param  None.
  * @retval None.
  */
extern void ui_thread_init
(
    /* No parameter. */
    void
);
    

     
#ifdef __cplusplus
}
#endif

#endif /* _APP_INTERFACE_H_ */
