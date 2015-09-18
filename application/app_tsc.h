/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_lcd.h
 @brief  : this file include the application variables and functions prototypes 
           for the touch sensor controller.
 @author : wangxianwen
 @history:
           2015-9-15    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _APP_TSC_H_
#define _APP_TSC_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "drv_tsc.h"


 

/* Screen coordinate for 'road mode' graph left down and right up. */
#define TSC_OBJCOOR_ROADMODE_LDX        290
#define TSC_OBJCOOR_ROADMODE_LDY        200

#define TSC_OBJCOOR_ROADMODE_RUX        345
#define TSC_OBJCOOR_ROADMODE_RUY        242

/* Screen coordinate for 'vehicle mode' graph left down and right up. */
#define TSC_OBJCOOR_VECMODE_LDX         360
#define TSC_OBJCOOR_VECMODE_LDY         200

#define TSC_OBJCOOR_VECMODE_RUX         415
#define TSC_OBJCOOR_VECMODE_RUY         242

/* Screen coordinate for 'breakdown mode' graph left down and right up. */
#define TSC_OBJCOOR_BREAKDOWNMODE_LDX   430
#define TSC_OBJCOOR_BREAKDOWNMODE_LDY   200

#define TSC_OBJCOOR_BREAKDOWNMODE_RUX   479
#define TSC_OBJCOOR_BREAKDOWNMODE_RUY   242






/* Thread stack size and priority. */
#define TSC_THREAD_STACK_SIZE    (1024*2)
#define TSC_THREAD_PRIORITY      25


/**
  * @brief  Initialize tsc thread.
  * @param  None.
  * @retval None.
  */
extern void tsc_thread_init
(
    /* No parameter. */
    void
);



     
#ifdef __cplusplus
}
#endif

#endif /* _APP_TSC_H_ */
