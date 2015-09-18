/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_main.h
 @brief  : This file include the prototype definitions for all driver modules.
 @author : wangxianwen
 @history:
           2015-6-02    wangxianwen    Create file. 
           ...
******************************************************************************/

#ifndef _DRV_MAIN_H_
#define _DRV_MAIN_H_

#ifdef __cplusplus
 extern "C" {
#endif





/* User configuration for driver modules. */

#define DRV_ENABLE_SDRAM  1
#define DRV_ENABLE_LCD    1   
#define DRV_ENABLE_TSC    1     
#define DRV_ENABLE_WNET   1




     
     
     
     
     
     
     
     
/* sdram module. */     
#if DRV_ENABLE_SDRAM
 #include "drv_sdram.h"
#endif


/* lcd module. */   
#if DRV_ENABLE_LCD

#if DRV_ENABLE_SDRAM     
 #include "drv_lcd.h" 
#else
 #error "!!! You must enable sdram module when using lcd module. !!!"
#endif   

#endif


/* Touch sensor control module. */
#if DRV_ENABLE_TSC
 #include "drv_tsc.h"
#endif


/* wnet module. */     
#if DRV_ENABLE_WNET
 #include "drv_wnet.h"
#endif
     
     
     
     
extern uint32_t drv_main_open(void);    
     
     
#ifdef __cplusplus
}
#endif

#endif /* _DRV_MAIN_H_ */
