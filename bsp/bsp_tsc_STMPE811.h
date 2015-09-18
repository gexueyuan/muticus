/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bsp_tsc_STMPE811.h
 @brief  : This file include the prototype definitions for touch sensor control 
           module hx8257.
 @author : gexueyuan wangxianwen
 @history:
           2015-9-07    gexueyuan wangxianwen    Create file. 
           ...
******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BSP_TSC_STMPE811_H_
#define _BSP_TSC_STMPE811_H_

#ifdef __cplusplus
 extern "C" {
#endif   


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
   

/* Touch sensor controller I2C port definitions-------------------------------*/ 
#define TSC_I2C                    I2C3
#define TSC_I2C_CLK                RCC_APB1Periph_I2C3

#define TSC_I2C_SCL_PIN            GPIO_Pin_8
#define TSC_I2C_SCL_GPIO_PORT      GPIOA
#define TSC_I2C_SCL_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define TSC_I2C_SCL_SOURCE         GPIO_PinSource8
#define TSC_I2C_SCL_AF             GPIO_AF_I2C3

#define TSC_I2C_SDA_PIN            GPIO_Pin_9
#define TSC_I2C_SDA_GPIO_PORT      GPIOC
#define TSC_I2C_SDA_GPIO_CLK       RCC_AHB1Periph_GPIOC
#define TSC_I2C_SDA_SOURCE         GPIO_PinSource9
#define TSC_I2C_SDA_AF             GPIO_AF_I2C3

#define TSC_I2C_DR                 ((uint32_t)0x40005C10)
#define I2C_SPEED                  100000

/* The 7 bits I2C addresses and chip IDs. */ 
#define TSC_STMPE811_ADDR          0x82    
#define TSC_STMPE811_ID            0x0811


/* Touch sensor controller DMA definitions------------------------------------*/
#define TSC_DMA_CLK                RCC_AHB1Periph_DMA1
#define TSC_DMA_CHANNEL            DMA_Channel_3

#define TSC_DMA_TX_STREAM          DMA1_Stream4  
#define TSC_DMA_RX_STREAM          DMA1_Stream2  

#define TSC_DMA_TX_TCFLAG          DMA_FLAG_TCIF4
#define TSC_DMA_RX_TCFLAG          DMA_FLAG_TCIF2  


/* Touch sensor controller Interrupt line on EXTI.----------------------------*/
#define TSC_IT_PIN                 GPIO_Pin_2
#define TSC_IT_GPIO_PORT           GPIOI
#define TSC_IT_GPIO_CLK            RCC_AHB1Periph_GPIOI
#define TSC_IT_EXTI_PORT_SOURCE    EXTI_PortSourceGPIOI
#define TSC_IT_EXTI_PIN_SOURCE     EXTI_PinSource2
#define TSC_IT_EXTI_LINE           EXTI_Line2
#define TSC_IT_EXTI_IRQn           EXTI2_IRQn   


/* STMPE811 device register definition.---------------------------------------*/

/* Identification registers. */ 
#define TSC_REG_CHP_ID             0x00
#define TSC_REG_ID_VER             0x02

/* General Control Registers. */ 
#define TSC_REG_SYS_CTRL1          0x03
#define TSC_REG_SYS_CTRL2          0x04
#define TSC_REG_SPI_CFG            0x08 

/* Interrupt Control register. */ 
#define TSC_REG_INT_CTRL           0x09
#define TSC_REG_INT_EN             0x0A
#define TSC_REG_INT_STA            0x0B
#define TSC_REG_GPIO_INT_EN        0x0C
#define TSC_REG_GPIO_INT_STA       0x0D

/* GPIO Registers. */ 
#define TSC_REG_GPIO_SET_PIN       0x10
#define TSC_REG_GPIO_CLR_PIN       0x11
#define TSC_REG_GPIO_MP_STA        0x12
#define TSC_REG_GPIO_DIR           0x13
#define TSC_REG_GPIO_ED            0x14
#define TSC_REG_GPIO_RE            0x15
#define TSC_REG_GPIO_FE            0x16
#define TSC_REG_GPIO_AF            0x17

/* ADC Registers. */ 
#define TSC_REG_ADC_INT_EN         0x0E
#define TSC_REG_ADC_INT_STA        0x0F
#define TSC_REG_ADC_CTRL1          0x20
#define TSC_REG_ADC_CTRL2          0x21
#define TSC_REG_ADC_CAPT           0x22
#define TSC_REG_ADC_DATA_CH0       0x30 
#define TSC_REG_ADC_DATA_CH1       0x32 
#define TSC_REG_ADC_DATA_CH2       0x34 
#define TSC_REG_ADC_DATA_CH3       0x36 
#define TSC_REG_ADC_DATA_CH4       0x38 
#define TSC_REG_ADC_DATA_CH5       0x3A 
#define TSC_REG_ADC_DATA_CH6       0x3B 
#define TSC_REG_ADC_DATA_CH7       0x3C 

/* TouchPanel Registers. */ 
#define TSC_REG_TP_CTRL            0x40
#define TSC_REG_TP_CFG             0x41
#define TSC_REG_WDM_TR_X           0x42 
#define TSC_REG_WDM_TR_Y           0x44
#define TSC_REG_WDM_BL_X           0x46
#define TSC_REG_WDM_BL_Y           0x48
#define TSC_REG_FIFO_TH            0x4A
#define TSC_REG_FIFO_STA           0x4B
#define TSC_REG_FIFO_SIZE          0x4C
#define TSC_REG_TP_DATA_X          0x4D 
#define TSC_REG_TP_DATA_Y          0x4F
#define TSC_REG_TP_DATA_Z          0x51
#define TSC_REG_TP_DATA_XYZ        0x52 
#define TSC_REG_TP_FRACT_XYZ       0x56
#define TSC_REG_TP_DATA            0x57
#define TSC_REG_TP_I_DRIVE         0x58
#define TSC_REG_TP_SHIELD          0x59


/* Functional and Interrupt Management.---------------------------------------*/

/* Functionalities definitions for 'TSC_REG_SYS_CTRL2 - 0x04'. */ 
#define TSC_ADC_FCT                0x01
#define TSC_TP_FCT                 0x02
#define TSC_IO_FCT                 0x04

/* Interrupt source configuration definitons. */ 
#define TSC_ITSRC_TP               0x01

/* Global Interrupts definitions for 'TSC_REG_INT_STA - 0x0B'. */ 
#define TSC_GIT_GPIO               0x80
#define TSC_GIT_ADC                0x40
#define TSC_GIT_TEMP               0x20
#define TSC_GIT_FE                 0x10
#define TSC_GIT_FF                 0x08
#define TSC_GIT_FOV                0x04
#define TSC_GIT_FTH                0x02
#define TSC_GIT_TOUCH              0x01

#define TSC_GIT_ALL                0xAA


/* Functions parameters defines.----------------------------------------------*/

/* IO Pins. */ 
#define IO_Pin_0                   0x01
#define IO_Pin_1                   0x02
#define IO_Pin_2                   0x04
#define IO_Pin_3                   0x08
#define IO_Pin_4                   0x10
#define IO_Pin_5                   0x20
#define IO_Pin_6                   0x40
#define IO_Pin_7                   0x80
#define IO_Pin_ALL                 0xFF

/* Touch Panel Pins definition. caution: Not match the e-circle,but no matter what value I changed the chip work well. */ 
#define TOUCH_YD                   IO_Pin_1 
#define TOUCH_XD                   IO_Pin_2 
#define TOUCH_YU                   IO_Pin_3 
#define TOUCH_XU                   IO_Pin_4 
#define TOUCH_IO_ALL               (uint8_t)(IO_Pin_1 | IO_Pin_2 | IO_Pin_3 | IO_Pin_4)

/* IO Pin directions. */ 
#define Direction_IN               0x00
#define Direction_OUT              0x01

/* Interrupt Line output parameters. */ 
#define Polarity_Low               0x00
#define Polarity_High              0x04
#define Type_Level                 0x00
#define Type_Edge                  0x02

/* IO Interrupts. */ 
#define IO_IT_0                    0x01
#define IO_IT_1                    0x02
#define IO_IT_2                    0x04
#define IO_IT_3                    0x08
#define IO_IT_4                    0x10
#define IO_IT_5                    0x20
#define IO_IT_6                    0x40
#define IO_IT_7                    0x80
#define ALL_IT                     0xFF
#define TSC_TP_IT                  (uint8_t)(IO_IT_0 | IO_IT_1 | IO_IT_2)
#define TSC_INMEMS_IT              (uint8_t)(IO_IT_2 | IO_IT_3)

/* Edge detection value. */ 
#define EDGE_FALLING               0x01
#define EDGE_RISING                0x02

/* Global interrupt Enable bit. */ 
#define TSC_GIT_EN                 0x01

/* The value of the maximal timeout for I2C waiting loops. */
#define TIMEOUT_MAX                0x3000



/* Error codes. */ 
typedef enum
{
    TSC_OK = 0,
    TSC_FAILURE, 
    TSC_TIMEOUT,
    TSC_PARAM_ERROR,
    TSC_NOT_OPERATIONAL, 
  
}TSC_Status_TypDef;


/* TSC DMA Direction. */ 
typedef enum
{
    TSC_DMA_TX = 0,
    TSC_DMA_RX = 1
  
}TSC_DMADirection_TypeDef;


/* Exported_Structure --------------------------------------------------------*/

/* Touch sensor controller state structure. */
typedef struct _tsc_state_st_
{
    /* Touched or not. */
    uint8_t Touched;

    /* Touched coordinate. */
    uint16_t      X;
    uint16_t      Y;
    uint8_t       Z;
  
}tsc_state_st, * tsc_state_st_ptr; 

#define TSC_STATE_ST_LEN    (sizeof(tsc_state_st))


/* Tsc touched status. */
#define TSC_TOUCHED_NO    0x00
#define TSC_TOUCHED_YES   0x80



/* Exported_Functions --------------------------------------------------------*/ 

/* Configuration and initialization functions. */
extern TSC_Status_TypDef TSC_init(void);


/* Touch Panel controller functions. */
extern TSC_Status_TypDef TSC_get_touch_state(tsc_state_st_ptr state_ptr);
extern TSC_Status_TypDef TSC_get_device_state(void);



#ifdef __cplusplus
}
#endif

#endif /* _BSP_TSC_STMPE811_H_ */


