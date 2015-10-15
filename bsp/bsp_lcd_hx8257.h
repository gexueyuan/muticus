/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bsp_lcd_HX8257.h
 @brief  : This file include the prototype definitions for lcd module hx8257.
 @author : wangxianwen
 @history:
           2015-5-15    wangxianwen    Create file. 
           ...
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BSP_LCD_HX8257_H_
#define _BSP_LCD_HX8257_H_

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/
#include "stm32f4xx.h"

	 
	 
/* Exported_Constants -------------------------------------------------------*/

/* LCD Size (Width and Height). */
#define LCD_PIXEL_WIDTH          ((uint16_t)480)
#define LCD_PIXEL_HEIGHT         ((uint16_t)272)

#define LCD_FRAME_BUFFER         ((uint32_t)0xD0000000)
#define LCD_BUFFER_OFFSET        ((uint32_t)0x50000) 

/* LCD display control pin. */
#define LCD_DISP_GPIO_PIN        GPIO_Pin_8                  
#define LCD_DISP_GPIO_PORT       GPIOC
#define LCD_DISP_GPIO_CLK        RCC_AHB1Periph_GPIOC 



/* Exported_Functions -------------------------------------------------------*/

/**
  * @brief  Main routine for initializing the LCD.
  * @param  None
  * @retval None
  */
extern void LCD_init(void);

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
extern void LCD_display_on(void);

/**
  * @brief  Disable the Display.
  * @param  None
  * @retval None
  */
extern void LCD_display_off(void);


#ifdef __cplusplus
}
#endif

#endif /* _BSP_LCD_HX8257_H_ */
