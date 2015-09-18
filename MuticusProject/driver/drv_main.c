/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_main.c
 @brief  : this file include the driver functions for the all modules.
 @author : wangxianwen
 @history:
           2015-6-02    wangxianwen    Created file
           ...
******************************************************************************/

#include "drv_main.h"


uint32_t drv_main_open(void)
{
    uint32_t result = 0;
    
    
#if DRV_ENABLE_SDRAM
    
    SDRAM_Init();
    
#endif

    
#if DRV_ENABLE_LCD

    /* Open lcd module. */
    if((result = DRV_LCD_PTR->open(DRV_LCD_PTR)) != DRV_LCD_ERR_OK)
    {
        return result;
    }
    
#endif

    
#if DRV_ENABLE_TSC

    /* Open tsc module. */
    if((result = DRV_TSC_PTR->open(DRV_TSC_PTR)) != DRV_TSC_ERR_OK)
    {
        return result;
    }
    
#endif

   
#if DRV_ENABLE_WNET
    
    /* Open wnet module. */
    if((result = DRV_WNET_PTR->open(DRV_WNET_PTR)) != DRV_WNET_ERR_OK)
    {
        return result;
    }
    
#endif
    
 









    
    
}
