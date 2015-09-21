/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : dat_lcd_alarm.h
 @brief  : this file include the application data prototypes for lcd module.
 @author : wangxianwen
 @history:
           2015-7-08    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _DAT_LCD_ALARM_H_
#define _DAT_LCD_ALARM_H_

#ifdef __cplusplus
 extern "C" {
#endif 

     
#include "drv_lcd.h"     
     

/* Lcd alarm picture group. */
extern const drv_lcd_plane_st PicAlarmGroup[];

/* Lcd alarm pictture index in picture group. */     
#define PIC_INDEX_ATT_FRONT_VEC          0x0000
#define PIC_INDEX_ATT_REAR_VEC           0x0001
#define PIC_INDEX_FRONT_VEC_BRAKE        0x0002
#define PIC_INDEX_FRONT_VEC_FAULTY       0x0003
#define PIC_INDEX_FRONT_ACCIDENT         0x0004
#define PIC_INDEX_ATT_FRONT_XXX          0x0005
#define PIC_INDEX_AVOID_AMBULANCE        0x0006

     
     
/* Lcd information picture group. */
extern const drv_lcd_plane_st PicInforGroup[];    

/* Lcd information picture index in picture group. */
#define PIC_INDEX_INFOR_VEC_BACKGROUND   0x0000

#define PIC_INDEX_INFOR_HIGHWAY          0x0001
#define PIC_INDEX_INFOR_MOUNTAIN         0x0002
#define PIC_INDEX_INFOR_CITY             0x0003

#define PIC_INDEX_INFOR_CAR              0x0004
#define PIC_INDEX_INFOR_AMBULANCE        0x0005

#define PIC_INDEX_INFOR_NORMAL           0x0006
#define PIC_INDEX_INFOR_BREAKDOWN        0x0007
     
     
     
     
     
     
     
     
#ifdef __cplusplus
}
#endif

#endif /* _DAT_LCD_ALARM_H_ */
