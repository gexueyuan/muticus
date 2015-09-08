/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_touch.h
 @brief  : drv_touch.h
 @author : gexueyuan
 @history:
           2015-8-25    gexueyuan    Created file
           ...
******************************************************************************/

#ifndef	__DRV_TOUCH_H__
#define	__DRV_TOUCH_H__

#include "Drv_lcd.h"


#define  GetMaxX  LCD_SCREEN_PIXEL_WIDTH
#define  GetMaxY  LCD_SCREEN_PIXEL_HEIGHT


#define  DEBOUNCE   2
#define  TOUCH_DELAY_PRESS  1000/DEBOUNCE	
#define  TOUCH_DELAY_MOVE   100/DEBOUNCE


#define CALIBRATION_DATA_MAGIC  0x55


typedef struct _Calibration_data {

    uint8_t  calibration_flag;
    
    uint16_t _calMaxX;
    
    uint16_t _calMinX;

    uint16_t _calMaxY;

    uint16_t _calMinY;
} Calibration_data_t;

#endif
