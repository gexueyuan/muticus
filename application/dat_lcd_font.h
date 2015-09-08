/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : dat_lcd_font.h
 @brief  : this file include the application font data prototypes for lcd module.
 @author : wangxianwen
 @history:
           2015-7-09    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _DAT_LCD_FONT_H_
#define _DAT_LCD_FONT_H_

#ifdef __cplusplus
 extern "C" {
#endif 

     
#include "drv_lcd.h"
    

/* Lcd fonts picture group. */
extern const drv_lcd_plane_st PicFontGroup[];

/* Lcd alarm pictture index in picture group. */     
#define PIC_FONT_INDEX_ASCII_16X24    0x0000
#define PIC_FONT_INDEX_ASCII_12X12    0x0001
#define PIC_FONT_INDEX_ASCII_8X12     0x0002
#define PIC_FONT_INDEX_ASCII_8X8      0x0003





#ifdef __cplusplus
}
#endif
  
#endif /* __DAT_LCD_FONT_H */
