/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : app_tsc.c
 @brief  : this file include the application functions for the touch sensor controller.
 @author : wangxianwen
 @history:
           2015-9-15    wangxianwen    Created file
           ...
******************************************************************************/

#include "app_tsc.h"
#include "drv_main.h"
#include "cv_osal.h"
#include "cv_lcd.h"

#include "cv_drv_key.h"
#include "cv_cms_def.h"

/* Vehicle parameter. */
extern sys_param_st SysParam;


/**
  * @brief  Tsc none operation handler.
  * @param  See below.
  * @retval None.
  */
static void tsc_operation_none
(
    /* Pointr to current touch state structure. */
    drv_tsc_touch_state_st_ptr touch_cur_ptr,

    /* Pointer to previous touch state structure. */
    drv_tsc_touch_state_st_ptr touch_pre_ptr
)
{
    
}


/**
  * @brief  Tsc press operation handler.
  * @param  See below.
  * @retval None.
  */
static void tsc_operation_press
(
    /* Pointr to current touch state structure. */
    drv_tsc_touch_state_st_ptr touch_cur_ptr,

    /* Pointer to previous touch state structure. */
    drv_tsc_touch_state_st_ptr touch_pre_ptr
)
{
    
}


/**
  * @brief  Tsc hold operation handler.
  * @param  See below.
  * @retval None.
  */
static void tsc_operation_hold
(
    /* Pointr to current touch state structure. */
    drv_tsc_touch_state_st_ptr touch_cur_ptr,

    /* Pointer to previous touch state structure. */
    drv_tsc_touch_state_st_ptr touch_pre_ptr
)
{
    //rt_kprintf(" Hold operation! x = %d, y = %d.\n",touch_cur_ptr->coor_x, touch_cur_ptr->coor_y);
}


/**
  * @brief  Tsc release operation handler.
  * @param  See below.
  * @retval None.
  */
static void tsc_operation_release
(
    /* Pointr to current touch state structure. */
    drv_tsc_touch_state_st_ptr touch_cur_ptr,

    /* Pointer to previous touch state structure. */
    drv_tsc_touch_state_st_ptr touch_pre_ptr
)
{
    if(SysParam.sys_mode == SYS_SYSMODE_NORMAL)
    {
        if( ((TSC_OBJCOOR_ROADMODE_LDX <= touch_pre_ptr->coor_x) && (touch_pre_ptr->coor_x <= TSC_OBJCOOR_ROADMODE_RUX))
         && ((TSC_OBJCOOR_ROADMODE_LDY <= touch_pre_ptr->coor_y) && (touch_pre_ptr->coor_y <= TSC_OBJCOOR_ROADMODE_RUY)) )
        {
            /* Update road mode. */
            SysParam.vec_param.road_mode = (VEC_ROADMODE_INDEX_MAX <= SysParam.vec_param.road_mode)? VEC_ROADMODE_INDEX_MIN : SysParam.vec_param.road_mode + 1;

            /* Set the active road mode. */
            switch(SysParam.vec_param.road_mode)
            {
                case VEC_ROADMODE_HIGHWAY:  {  mode_change(2);  break;  }
                case VEC_ROADMODE_MOUNTAIN: {  mode_change(3);  break;  }
                case VEC_ROADMODE_CITY:     {  mode_change(4);  break;  }
                default:                    {  break;                   }
            }
            
            /* Reset system every time when change roadmode. */
            NVIC_SystemReset();
        }
        else if( ((TSC_OBJCOOR_VECMODE_LDX <= touch_pre_ptr->coor_x) && (touch_pre_ptr->coor_x <= TSC_OBJCOOR_VECMODE_RUX))
              && ((TSC_OBJCOOR_VECMODE_LDY <= touch_pre_ptr->coor_y) && (touch_pre_ptr->coor_y <= TSC_OBJCOOR_VECMODE_RUY)) )
        {
            /* Update vehicle mode. */
            SysParam.vec_param.vec_mode = (VEC_VECMODE_INDEX_MAX <= SysParam.vec_param.vec_mode)? VEC_VECMODE_INDEX_MIN : SysParam.vec_param.vec_mode + 1;
        }
        else if( ((TSC_OBJCOOR_BREAKDOWNMODE_LDX <= touch_pre_ptr->coor_x) && (touch_pre_ptr->coor_x <= TSC_OBJCOOR_BREAKDOWNMODE_RUX))
              && ((TSC_OBJCOOR_BREAKDOWNMODE_LDY <= touch_pre_ptr->coor_y) && (touch_pre_ptr->coor_y <= TSC_OBJCOOR_BREAKDOWNMODE_RUY)) )
        {
            /* Update breakdown mode. */
            SysParam.vec_param.breakdown_mode = (VEC_BREAKDOWNMODE_INDEX_MAX <= SysParam.vec_param.breakdown_mode)? VEC_BREAKDOWNMODE_INDEX_MIN : SysParam.vec_param.breakdown_mode + 1;
            sys_add_event_queue(&(cms_envar.sys), SYS_MSG_KEY_PRESSED, 0, C_DOWN_KEY, NULL);
        }
        else
        {
            
        }


        
    }
    else if(SysParam.sys_mode == SYS_SYSMODE_ALARM)
    {
        ;
    }
}


/**
  * @brief  Entry routine for tsc thread.
  * @param  See below.
  * @retval None.
  */
static void tsc_thread_entry
(
    /* Pointr to parameter for thread. */
    void *parameter
)
{
    drv_tsc_touch_state_st touch_cur = { 0 };
    drv_tsc_touch_state_st touch_pre = { 0 };
    

    while(1)
    {
        DRV_TSC_PTR->ioctl(DRV_TSC_PTR, TSC_IOCTL_GET_TOUCH_STATE, &touch_cur);

        if((touch_pre.touched == TSC_TOUCH_STATE_NO) && (touch_cur.touched == TSC_TOUCH_STATE_NO))
        {
            /* No operation. */
            tsc_operation_none(&touch_cur, &touch_pre);
        }
        else if((touch_pre.touched == TSC_TOUCH_STATE_NO) && (touch_cur.touched == TSC_TOUCH_STATE_YES))
        {
            /* Press operation. */
            tsc_operation_press(&touch_cur, &touch_pre);
        }
        else if((touch_pre.touched == TSC_TOUCH_STATE_YES) && (touch_cur.touched == TSC_TOUCH_STATE_NO))
        {
            /* Release operation. */
            tsc_operation_release(&touch_cur, &touch_pre);
        }
        else if((touch_pre.touched == TSC_TOUCH_STATE_YES) && (touch_cur.touched == TSC_TOUCH_STATE_YES))
        {
            /* Hold operation. */
            tsc_operation_hold(&touch_cur, &touch_pre);
        }
        
        touch_pre = touch_cur;

        /* Delay 40ms for other tasks. */
        osal_delay(4);
    }
}


/**
  * @brief  Initialize tsc thread.
  * @param  None.
  * @retval None.
  */
void tsc_thread_init
( 
    /* No parameter. */
    void
)
{
    osal_task_t *tid_ptr = NULL;
    
    
    /* Create lcd task. */
    tid_ptr = osal_task_create("tsc", tsc_thread_entry, NULL, TSC_THREAD_STACK_SIZE, TSC_THREAD_PRIORITY);
    osal_assert(tid_ptr != RT_NULL);
}


