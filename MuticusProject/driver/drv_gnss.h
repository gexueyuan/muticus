/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_gnss.h
 @brief  : this file include the driver variables and functions prototypes for 
           GNSS module.
 @author : wangxianwen
 @history:
           2015-7-30    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _DRV_GNSS_H_
#define _DRV_GNSS_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"



/* GNSS USARTx pins definition. */
#define GNSS_USARTx                           USART1
#define GNSS_USARTx_CLK                       RCC_APB2Periph_USART1
#define GNSS_USARTx_CLK_INIT                  RCC_APB2PeriphClockCmd
#define GNSS_USARTx_IRQn                      USART1_IRQn
#define GNSS_USARTx_IRQHandler                USART1_IRQHandler

#define GNSS_USARTx_TX_PIN                    GPIO_Pin_9                
#define GNSS_USARTx_TX_GPIO_PORT              GPIOA                       
#define GNSS_USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define GNSS_USARTx_TX_SOURCE                 GPIO_PinSource9
#define GNSS_USARTx_TX_AF                     GPIO_AF_USART1

#define GNSS_USARTx_RX_PIN                    GPIO_Pin_10                
#define GNSS_USARTx_RX_GPIO_PORT              GPIOA                    
#define GNSS_USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define GNSS_USARTx_RX_SOURCE                 GPIO_PinSource10
#define GNSS_USARTx_RX_AF                     GPIO_AF_USART1

#define GNSS_USARTx_IRQ_PREPRIO               2     /* preemption priority level(0 is the highest). */
#define GNSS_USARTx_IRQ_SUBRIO                1     /* sub-priority level (0 is the highest). */


/* GNSS USARTx initialize structure data. */
#define GNSS_USARTx_BAUDRATE                  115200
#define GNSS_USARTx_WORD_LENGTH               USART_WordLength_8b
#define GNSS_USARTx_STOP_BITS                 USART_StopBits_1
#define GNSS_USARTx_PARITY                    USART_Parity_No
#define GNSS_USARTx_HARDWARE_FLOW_CTRL        USART_HardwareFlowControl_None
#define GNSS_USARTx_MODE                      (USART_Mode_Rx | USART_Mode_Tx)



/* GNSS USARTx DMAx definition. */
#define GNSS_USARTx_DMA                       DMA2
#define GNSS_USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA2
#define GNSS_USARTx_DR_ADDRESS                ((uint32_t)USART1 + 0x04) 

/* GNSS DMA transfer stream error interrupt definition. */
#define GNSS_USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF7    /* Streamx FIFO error error interrupt. */ 
#define GNSS_USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF7   /* Streamx direct mode error interrupt. */
#define GNSS_USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF7    /* Streamx transfer error interrupt. */
#define GNSS_USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF7    /* Streamx half transfer complete interrupt. */
#define GNSS_USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF7    /* Streamx transfer complete interrupt. */

/* GNSS DMA receiver stream error interrupt definition. */
#define GNSS_USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF5    /* Streamx FIFO error error interrupt. */ 
#define GNSS_USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF5   /* Streamx direct mode error interrupt. */
#define GNSS_USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF5    /* Streamx transfer error interrupt. */
#define GNSS_USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF5    /* Streamx half transfer complete interrupt. */
#define GNSS_USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF5    /* Streamx transfer complete interrupt. */

/* GNSS USARTx DMA interrupt number. */
#define GNSS_USARTx_DMA_TX_IRQn               DMA2_Stream7_IRQn
#define GNSS_USARTx_DMA_RX_IRQn               DMA2_Stream5_IRQn

/* GNSS USARTx DMA interrupt preemption priority and subpriority. */
#define GNSS_USARTx_DMA_TX_IRQ_PREPRIO        2                 /* preemption priority level(0 is the highest). */
#define GNSS_USARTx_DMA_TX_IRQ_SUBRIO         1                 /* sub-priority level (0 is the highest). */
#define GNSS_USARTx_DMA_RX_IRQ_PREPRIO        2                 /* preemption priority level(0 is the highest). */
#define GNSS_USARTx_DMA_RX_IRQ_SUBRIO         1                 /* sub-priority level (0 is the highest). */

/* GNSS USARTx DMA interrupt requst handler. */
#define GNSS_USARTx_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler
#define GNSS_USARTx_DMA_RX_IRQHandler         DMA2_Stream5_IRQHandler


/* GNSS USARTx DMAx initialize structure data. */
#define GNSS_USARTx_DMA_BUFFERSIZE            30
#define GNSS_USARTx_DMA_FIFOMODE              DMA_FIFOMode_Disable
#define GNSS_USARTx_DMA_FIFOTHRESHOLD         DMA_FIFOThreshold_1QuarterFull
#define GNSS_USARTx_DMA_MEMORYBURST           DMA_MemoryBurst_Single
#define GNSS_USARTx_DMA_MEMORY_DATASIZE       DMA_MemoryDataSize_Byte
#define GNSS_USARTx_DMA_MEMORY_INC            DMA_MemoryInc_Enable
#define GNSS_USARTx_DMA_MODE                  DMA_Mode_Normal
 
#define GNSS_USARTx_PERIPHERAL_BASE_ADDR      ((uint32_t)(&(GNSS_USARTx->DR)))
#define GNSS_USARTx_PERIPHERAL_BURST          DMA_PeripheralBurst_Single
#define GNSS_USARTx_PERIPHERAL_DATASIZE       DMA_PeripheralDataSize_Byte
#define GNSS_USARTx_PERIPHERAL_INC            DMA_PeripheralInc_Disable
#define GNSS_USARTx_DMA_PRIORITY              DMA_Priority_High

#define GNSS_USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define GNSS_USARTx_TX_DMA_DIR                DMA_DIR_MemoryToPeripheral
#define GNSS_USARTx_TX_DMA_STREAM             DMA2_Stream7

#define GNSS_USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define GNSS_USARTx_RX_DMA_DIR                DMA_DIR_PeripheralToMemory
#define GNSS_USARTx_RX_DMA_STREAM             DMA2_Stream5

     

/* Error code for GNSS driver. */
typedef enum _drv_gnss_err_code
{
    DRV_GNSS_ERR_OK = 0x0030,
    
    DRV_GNSS_ERR_OPEN,
    DRV_GNSS_ERR_CLOSE,
    DRV_GNSS_ERR_READ,
    DRV_GNSS_ERR_WRITE,
    DRV_GNSS_ERR_IOCTL
    
}DRV_GNSS_ERR_CODE, *DRV_GNSS_ERR_CODE_PTR;

#define DRV_GNSS_ERR_CODE_LEN    (sizeof(DRV_GNSS_ERR_CODE))



/* Cmd for GNSS ioctl. */
typedef enum _dev_gnss_ioctl_cmd
{
    /* ioctl command: register DMA transfer complete isr. */
    GNSS_IOCTL_REGISTER_TXTC_ISR,
    
    /* ioctl command: register DMA receiver complete isr. */
    GNSS_IOCTL_REGISTER_RXTC_ISR,
    
    /* ioctl command: register USART line idle isr. */
    GNSS_IOCTL_REGISTER_LINE_IDLE_ISR
    
}DRV_GNSS_IOCTL_CMD, *DRV_GNSS_IOCTL_CMD_PTR;


/* Device driver structure for GNSS device. */
typedef struct _drv_gnss_st
{	
    /* Object's operation group. */
    DRV_GNSS_ERR_CODE (*open) (void *);
    DRV_GNSS_ERR_CODE (*close)(void *);
    DRV_GNSS_ERR_CODE (*read) (void *, uint32_t, uint8_t *, uint32_t);
    DRV_GNSS_ERR_CODE (*write)(void *, uint32_t, uint8_t *, uint32_t);
    DRV_GNSS_ERR_CODE (*ioctl)(void *, DRV_GNSS_IOCTL_CMD, void *);
    
    /* Function pointer for isr of DMA transfer complete. */
	void (*isr_tx_complete)(void *);
    void    * param_tx_complete_ptr;
    
    /* Function pointer for isr of DMA receiver complete. */
	void (*isr_rx_complete)(void *);
    void    * param_rx_complete_ptr;
    
    /* Function pointer for isr of USART line idle. */
    void   (*isr_line_idle)(void *);
    void      * param_line_idle_ptr;

}drv_gnss_st, *drv_gnss_st_ptr;

#define DRV_GNSS_ST_LEN    (sizeof(drv_gnss_st))


/* GNSS user isr register structure. */
typedef struct _drv_gnss_isr_register_st
{
    void (*isr_function)(void *);
    void          *isr_param_ptr;
    
}drv_gnss_isr_register_st, *drv_gnss_isr_register_st_ptr;

#define DRV_GNSS_ISR_REGISTER_ST_LEN    (sizeof(drv_gnss_isr_register_st))



 /* Main object for gnss driver. */
extern drv_gnss_st GnssDriver;
#define DRV_GNSS_PTR    ((drv_gnss_st_ptr)&GnssDriver)
    
     







/************************* GNSS protecol *******************************/


/* NMEA0183 frame: RMC - Recommended Minimum Specific GNSS Data. */
typedef enum _nmea_frame_rmc_item
{
    /* $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxxxx,x.x,a,a*hh<CR><LF> */

    NMEA_RMC_HEAD_$GPRMC = 0,
    
    NMEA_RMC_UTC_TIME,
    NMEA_RMC_STATUS,
    
    NMEA_RMC_LATITUDE,
    NMEA_RMC_LATITUDE_NS,

    NMEA_RMC_LONGITUDE,
    NMEA_RMC_LONGITUDE_EW,

    NMEA_RMC_SPEED_OVER_GD,
    NMEA_RMC_COURCE_OVER_GD,

    NMEA_RMC_UTC_DATA,
    
    NMEA_RMC_MAGNETIC_VARIATION,
    NMEA_RMC_DEGREES_EW,

    NMEA_RMC_MODE_INDICATOR
    
}NMEA_FRAME_RMC_ITEM;
















     
#ifdef __cplusplus
}
#endif

#endif /* _DRV_GNSS_H_ */
