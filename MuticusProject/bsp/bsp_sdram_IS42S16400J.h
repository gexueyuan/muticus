/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bsp_sdram_IS42S16400J.h
 @brief  : this file include the bsp variables and functions prototypes for 
           the sdram module IS42S16400J.
 @author : wangxianwen
 @history:
           2015-5-18    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _BSP_SDRAM_IS42S16400J_H_
#define _BSP_SDRAM_IS42S16400J_H_

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/
#include "stm32f4xx.h"

     
/** @defgroup Exported_Constants
  * @{
  */ 
     
/* FMC SDRAM Bank address. */   
#define SDRAM_BANK_ADDR       ((uint32_t)0xD0000000)
  
/* FMC SDRAM Memory Width. */  
#define SDRAM_MEMORY_WIDTH    FMC_SDMemory_Width_16b 

/* FMC SDRAM CAS Latency. */  
#define SDRAM_CAS_LATENCY     FMC_CAS_Latency_3

/* FMC SDRAM Memory clock period. */  
#define SDCLOCK_PERIOD        FMC_SDClock_Period_3      /* Default configuration used with LCD. */

/* FMC SDRAM Memory Read Burst feature. */  
#define SDRAM_READBURST       FMC_Read_Burst_Disable    /* Default configuration used with LCD. */


/* FMC SDRAM Mode definition register defines. */
#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000) 
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)      

/**
  * @}
  */ 


/** @defgroup Exported_Functions
  * @{
  */ 

/**
  * @brief  Initialize the SDRAM memory.
  *         This function must be called before any read/write operation
  *         on the SDRAM.
  * @param  None
  * @retval None
  */
extern void SDRAM_Init(void);

/**
  * @brief  Writes a Entire-word buffer to the SDRAM memory. 
  * @param  pBuffer: pointer to buffer. 
  * @param  uwWriteAddress: SDRAM memory internal address from which the data will be 
  *         written.
  * @param  uwBufferSize: number of words to write. 
  * @retval None.
  */
extern void SDRAM_WriteBuffer(uint32_t* pBuffer, uint32_t uwWriteAddress, uint32_t uwBufferSize);

/**
  * @brief  Reads data buffer from the SDRAM memory. 
  * @param  pBuffer: pointer to buffer. 
  * @param  ReadAddress: SDRAM memory internal address from which the data will be 
  *         read.
  * @param  uwBufferSize: number of words to write. 
  * @retval None.
  */
extern void SDRAM_ReadBuffer(uint32_t* pBuffer, uint32_t uwReadAddress, uint32_t uwBufferSize);

/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif /* _BSP_SDRAM_IS42S16400J_H_ */

