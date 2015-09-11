/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bma250e.c
 @brief  : bma250e chip spi driver. ref l3gd20.h
 @author : wanglei
 @history:
           2014-8-11    wanglei       Created file
           ...
******************************************************************************/



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMA250E_H_
#define __BMA250E_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <board.h>


#define BMA250E_Sensitivity_2g    (float)0.038344f        /* m/s2 */
#define BMA250E_Sensitivity_4g    (float)0.076688f        
#define BMA250E_Sensitivity_8g    (float)0.153376f         


#define READWRITE_CMD              ((uint8_t)0x80) 
/* Multiple byte read/write command */ 
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0xff)

/**
  * @brief  BMA250E SPI Interface pins
  */

#ifdef HARDWARE_MODULE_WIFI_V1

#define BMA250E_SPI                       SPI2
#define BMA250E_SPI_CLK                   RCC_APB1Periph_SPI2

#define BMA250E_SPI_SCK_PIN               GPIO_Pin_13                 /* PB.13 */
#define BMA250E_SPI_SCK_GPIO_PORT         GPIOB                       /* GPIOB */
#define BMA250E_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOB
#define BMA250E_SPI_SCK_SOURCE            GPIO_PinSource13
#define BMA250E_SPI_SCK_AF                GPIO_AF_SPI2

#define BMA250E_SPI_MISO_PIN              GPIO_Pin_14                 /* PB.14 */
#define BMA250E_SPI_MISO_GPIO_PORT        GPIOB                       /* GPIOB */
#define BMA250E_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define BMA250E_SPI_MISO_SOURCE           GPIO_PinSource14
#define BMA250E_SPI_MISO_AF               GPIO_AF_SPI2

#define BMA250E_SPI_MOSI_PIN              GPIO_Pin_15                 /* PB.15 */
#define BMA250E_SPI_MOSI_GPIO_PORT        GPIOB                       /* GPIOB */
#define BMA250E_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define BMA250E_SPI_MOSI_SOURCE           GPIO_PinSource15
#define BMA250E_SPI_MOSI_AF               GPIO_AF_SPI2

#define BMA250E_SPI_CS_PIN                GPIO_Pin_12                 /* PB.12 */
#define BMA250E_SPI_CS_GPIO_PORT          GPIOB                       /* GPIOB */
#define BMA250E_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOB

#define BMA250E_SPI_INT1_PIN              GPIO_Pin_0                  /* PB.00 */
#define BMA250E_SPI_INT1_GPIO_PORT        GPIOB                       /* GPIOC */
#define BMA250E_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define BMA250E_SPI_INT1_EXTI_LINE        EXTI_Line0
#define BMA250E_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define BMA250E_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource0
#define BMA250E_SPI_INT1_EXTI_IRQn        EXTI0_IRQn 

#define BMA250E_SPI_INT2_PIN              GPIO_Pin_1                  /* PB.01 */
#define BMA250E_SPI_INT2_GPIO_PORT        GPIOB                       /* GPIOB */
#define BMA250E_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define BMA250E_SPI_INT2_EXTI_LINE        EXTI_Line1
#define BMA250E_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define BMA250E_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource1
#define BMA250E_SPI_INT2_EXTI_IRQn        EXTI1_IRQn 

#elif defined (HARDWARE_MODULE_WIFI_V2)

#define BMA250E_SPI                       SPI1
#define BMA250E_SPI_CLK                   RCC_APB2Periph_SPI1

#define BMA250E_SPI_SCK_PIN               GPIO_Pin_5                 /* PA.5 */
#define BMA250E_SPI_SCK_GPIO_PORT         GPIOA                       /* GPIOA */
#define BMA250E_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define BMA250E_SPI_SCK_SOURCE            GPIO_PinSource5
#define BMA250E_SPI_SCK_AF                GPIO_AF_SPI1

#define BMA250E_SPI_MISO_PIN              GPIO_Pin_6                 /* PA.6 */
#define BMA250E_SPI_MISO_GPIO_PORT        GPIOA                       /* GPIOA */
#define BMA250E_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define BMA250E_SPI_MISO_SOURCE           GPIO_PinSource6
#define BMA250E_SPI_MISO_AF               GPIO_AF_SPI1

#define BMA250E_SPI_MOSI_PIN              GPIO_Pin_7                 /* PA.7 */
#define BMA250E_SPI_MOSI_GPIO_PORT        GPIOA                       /* GPIOA */
#define BMA250E_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define BMA250E_SPI_MOSI_SOURCE           GPIO_PinSource7
#define BMA250E_SPI_MOSI_AF               GPIO_AF_SPI1

#define BMA250E_SPI_CS_PIN                GPIO_Pin_5                 /* PC.5 */
#define BMA250E_SPI_CS_GPIO_PORT          GPIOC                       /* GPIOC */
#define BMA250E_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOC

#define BMA250E_SPI_INT1_PIN              GPIO_Pin_0                  /* PB.00 */
#define BMA250E_SPI_INT1_GPIO_PORT        GPIOB                       /* GPIOC */
#define BMA250E_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define BMA250E_SPI_INT1_EXTI_LINE        EXTI_Line0
#define BMA250E_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define BMA250E_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource0
#define BMA250E_SPI_INT1_EXTI_IRQn        EXTI0_IRQn 

#define BMA250E_SPI_INT2_PIN              GPIO_Pin_1                  /* PB.01 */
#define BMA250E_SPI_INT2_GPIO_PORT        GPIOB                       /* GPIOB */
#define BMA250E_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define BMA250E_SPI_INT2_EXTI_LINE        EXTI_Line1
#define BMA250E_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOB
#define BMA250E_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource1
#define BMA250E_SPI_INT2_EXTI_IRQn        EXTI1_IRQn 


#elif defined (HARDWARE_MODULE_WIFI_V3)

#define BMA250E_SPI                       SPI5
#define BMA250E_SPI_CLK                   RCC_APB2Periph_SPI5

#define BMA250E_SPI_SCK_PIN               GPIO_Pin_7                 /* PA.5 */
#define BMA250E_SPI_SCK_GPIO_PORT         GPIOF                       /* GPIOA */
#define BMA250E_SPI_SCK_GPIO_CLK          RCC_AHB1Periph_GPIOF
#define BMA250E_SPI_SCK_SOURCE            GPIO_PinSource7
#define BMA250E_SPI_SCK_AF                GPIO_AF_SPI5

#define BMA250E_SPI_MISO_PIN              GPIO_Pin_8                 /* PA.6 */
#define BMA250E_SPI_MISO_GPIO_PORT        GPIOF                       /* GPIOA */
#define BMA250E_SPI_MISO_GPIO_CLK         RCC_AHB1Periph_GPIOF
#define BMA250E_SPI_MISO_SOURCE           GPIO_PinSource8
#define BMA250E_SPI_MISO_AF               GPIO_AF_SPI5

#define BMA250E_SPI_MOSI_PIN              GPIO_Pin_9                 /* PA.7 */
#define BMA250E_SPI_MOSI_GPIO_PORT        GPIOF                       /* GPIOA */
#define BMA250E_SPI_MOSI_GPIO_CLK         RCC_AHB1Periph_GPIOF
#define BMA250E_SPI_MOSI_SOURCE           GPIO_PinSource9
#define BMA250E_SPI_MOSI_AF               GPIO_AF_SPI5

#define BMA250E_SPI_CS_PIN                GPIO_Pin_1                 /* PC.5 */
#define BMA250E_SPI_CS_GPIO_PORT          GPIOC                       /* GPIOC */
#define BMA250E_SPI_CS_GPIO_CLK           RCC_AHB1Periph_GPIOC

#define BMA250E_SPI_INT1_PIN              GPIO_Pin_3                  /* PB.00 */
#define BMA250E_SPI_INT1_GPIO_PORT        GPIOE                       /* GPIOC */
#define BMA250E_SPI_INT1_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define BMA250E_SPI_INT1_EXTI_LINE        EXTI_Line0
#define BMA250E_SPI_INT1_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define BMA250E_SPI_INT1_EXTI_PIN_SOURCE  EXTI_PinSource0
#define BMA250E_SPI_INT1_EXTI_IRQn        EXTI0_IRQn 

#define BMA250E_SPI_INT2_PIN              GPIO_Pin_4                  /* PB.01 */
#define BMA250E_SPI_INT2_GPIO_PORT        GPIOE                       /* GPIOB */
#define BMA250E_SPI_INT2_GPIO_CLK         RCC_AHB1Periph_GPIOE
#define BMA250E_SPI_INT2_EXTI_LINE        EXTI_Line1
#define BMA250E_SPI_INT2_EXTI_PORT_SOURCE EXTI_PortSourceGPIOE
#define BMA250E_SPI_INT2_EXTI_PIN_SOURCE  EXTI_PinSource1
#define BMA250E_SPI_INT2_EXTI_IRQn        EXTI1_IRQn 

#endif
/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define BMA250E_WHO_AM_I_ADDR          0x00  /* device identification register */
#define BMA250E_OFC_CTRL               0x16  /* enable x,y,z  */
#define BMA250E_BGW_SPI3_WDT           0x34  /* spi 4 wire mode */


#define BMA250E_OUT_X_L_ADDR           0x02  /* Output Register X */
#define BMA250E_OUT_X_H_ADDR           0x03  /* Output Register X */
#define BMA250E_OUT_Y_L_ADDR           0x04  /* Output Register Y */
#define BMA250E_OUT_Y_H_ADDR           0x05  /* Output Register Y */
#define BMA250E_OUT_Z_L_ADDR           0x06  /* Output Register Z */
#define BMA250E_OUT_Z_H_ADDR           0x07  /* Output Register Z */ 




/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define BMA250E_FLAG_TIMEOUT     ((uint32_t)0x1000)




/** @defgroup Axes_Selection 
  * @{
  */
#define BMA250E_X_ENABLE            ((uint8_t)0x02)
#define BMA250E_Y_ENABLE            ((uint8_t)0x01)
#define BMA250E_Z_ENABLE            ((uint8_t)0x04)
#define BMA250E_AXES_ENABLE         ((uint8_t)0x07)
#define BMA250E_AXES_DISABLE        ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup BandWidth_Selection 
  * @{
  */
#define BMA250E_BANDWIDTH_1         ((uint8_t)0x00)
#define BMA250E_BANDWIDTH_2         ((uint8_t)0x10)
#define BMA250E_BANDWIDTH_3         ((uint8_t)0x20)
#define BMA250E_BANDWIDTH_4         ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup Full_Scale_Selection 
  * @{
  */
#define BMA250E_FULLSCALE_2g               ((uint8_t)0x03)
#define BMA250E_FULLSCALE_4g               ((uint8_t)0x05)
#define BMA250E_FULLSCALE_8g               ((uint8_t)0x08) 
#define BMA250E_FULLSCALE_16g              ((uint8_t)0x0C) 



/** @defgroup STM32F401_DISCOVERY_BMA250E_Exported_Macros
  * @{
  */
#define BMA250E_CS_LOW()       GPIO_ResetBits(BMA250E_SPI_CS_GPIO_PORT, BMA250E_SPI_CS_PIN)
#define BMA250E_CS_HIGH()      GPIO_SetBits(BMA250E_SPI_CS_GPIO_PORT, BMA250E_SPI_CS_PIN)
/**
  * @}
  */
 

void BMA250E_LowLevel_Init(void);
/* USER Callbacks: This is function for which prototype only is declared in
   MEMS accelerometre driver and that should be implemented into user applicaiton. */  
/* LSM303DLHC_TIMEOUT_UserCallback() function is called whenever a timeout condition 
   occure during communication (waiting transmit data register empty flag(TXE)
   or waiting receive data register is not empty flag (RXNE)).
   You can use the default timeout callback implementation by uncommenting the 
   define USE_DEFAULT_TIMEOUT_CALLBACK in stm32f401_discovery_LSM303DLHC.h file.
   Typically the user implementation of this callback should reset MEMS peripheral
   and re-initialize communication or in worst case reset all the application. */
uint32_t BMA250E_TIMEOUT_UserCallback(void);


/* read, write gsensor register api */
void gsnr_read_reg(uint8_t reg, uint8_t *data);
void gsnr_write_reg(uint8_t reg, uint8_t data);

void gsnr_get_acc(float *pdata);
void gsnr_drv_init(void);


#ifdef __cplusplus
}
#endif

#endif /* __BMA250E_H_ */
