/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_wnet.h
 @brief  : this file include the driver variables and functions prototypes for 
           the wireless network transport module.
 @author : wangxianwen
 @history:
           2015-6-17    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _DRV_WNET_H_
#define _DRV_WNET_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Constants -----------------------------------------------------------------*/

/* Definition for USARTx resources. */
#define USARTx                           UART5
#define USARTx_CLK                       RCC_APB1Periph_UART5
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      UART5_IRQn
#define USARTx_IRQHandler                UART5_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_12                
#define USARTx_TX_GPIO_PORT              GPIOC                       
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOC
#define USARTx_TX_SOURCE                 GPIO_PinSource12
#define USARTx_TX_AF                     GPIO_AF_UART5

#define USARTx_RX_PIN                    GPIO_Pin_2                
#define USARTx_RX_GPIO_PORT              GPIOD                    
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOD
#define USARTx_RX_SOURCE                 GPIO_PinSource2
#define USARTx_RX_AF                     GPIO_AF_UART5

#define USARTx_IRQ_PREPRIO               2        /* preemption priority level(0 is the highest). */
#define USARTx_IRQ_SUBRIO                1        /* sub-priority level (0 is the highest). */


/* Usart initialize structure data. */
#define USARTx_BAUDRATE                  115200
#define USARTx_WORD_LENGTH               USART_WordLength_8b
#define USARTx_STOP_BITS                 USART_StopBits_1
#define USARTx_PARITY                    USART_Parity_No
#define USARTx_HARDWARE_FLOW_CTRL        USART_HardwareFlowControl_None
#define USARTx_MODE                      (USART_Mode_Rx | USART_Mode_Tx)


/* Definition for DMAx resources. */
#define USARTx_DMA                       DMA1
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1
#define USARTx_DR_ADDRESS                ((uint32_t)UART5 + 0x04)

/* DMA transfer stream error interrupt definition. */
#define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF7    /* Streamx FIFO error error interrupt. */ 
#define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF7   /* Streamx direct mode error interrupt. */
#define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF7    /* Streamx transfer error interrupt. */
#define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF7    /* Streamx half transfer complete interrupt. */
#define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF7    /* Streamx transfer complete interrupt. */

/* DMA receiver stream error interrupt definition. */
#define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF0    /* Streamx FIFO error error interrupt. */ 
#define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF0   /* Streamx direct mode error interrupt. */
#define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF0    /* Streamx transfer error interrupt. */
#define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF0    /* Streamx half transfer complete interrupt. */
#define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF0    /* Streamx transfer complete interrupt. */

/* DMA interrupt number. */
#define USARTx_DMA_TX_IRQn               DMA1_Stream7_IRQn
#define USARTx_DMA_RX_IRQn               DMA1_Stream0_IRQn

/* DMA interrupt preemption priority and subpriority. */
#define USARTx_DMA_TX_IRQ_PREPRIO        2                 /* preemption priority level(0 is the highest). */
#define USARTx_DMA_TX_IRQ_SUBRIO         2                 /* sub-priority level (0 is the highest). */
#define USARTx_DMA_RX_IRQ_PREPRIO        1                 /* preemption priority level(0 is the highest). */
#define USARTx_DMA_RX_IRQ_SUBRIO         1                 /* sub-priority level (0 is the highest). */

/* DMA interrupt requst handler. */
#define USARTx_DMA_TX_IRQHandler         DMA1_Stream7_IRQHandler
#define USARTx_DMA_RX_IRQHandler         DMA1_Stream0_IRQHandler


/* DMAx initialize structure data. */
#define USARTx_DMA_BUFFERSIZE            30
#define USARTx_DMA_FIFOMODE              DMA_FIFOMode_Disable
#define USARTx_DMA_FIFOTHRESHOLD         DMA_FIFOThreshold_1QuarterFull
#define USARTx_DMA_MEMORYBURST           DMA_MemoryBurst_Single
#define USARTx_DMA_MEMORY_DATASIZE       DMA_MemoryDataSize_Byte
#define USARTx_DMA_MEMORY_INC            DMA_MemoryInc_Enable
#define USARTx_DMA_MODE                  DMA_Mode_Normal
 
#define USARTx_PERIPHERAL_BASE_ADDR      ((uint32_t)(&(USARTx->DR)))
#define USARTx_PERIPHERAL_BURST          DMA_PeripheralBurst_Single
#define USARTx_PERIPHERAL_DATASIZE       DMA_PeripheralDataSize_Byte
#define USARTx_PERIPHERAL_INC            DMA_PeripheralInc_Disable
#define USARTx_DMA_PRIORITY              DMA_Priority_High

#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_DIR                DMA_DIR_MemoryToPeripheral
#define USARTx_TX_DMA_STREAM             DMA1_Stream7

#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_RX_DMA_DIR                DMA_DIR_PeripheralToMemory
#define USARTx_RX_DMA_STREAM             DMA1_Stream0






/* Error code for wireless network driver. */
typedef enum _drv_wnet_err_code
{
    DRV_WNET_ERR_OK = 0x0020,
    
    DRV_WNET_ERR_OPEN,
    DRV_WNET_ERR_CLOSE,
    DRV_WNET_ERR_READ,
    DRV_WNET_ERR_WRITE,
    DRV_WNET_ERR_IOCTL
    
}DRV_WNET_ERR_CODE, *DRV_WNET_ERR_CODE_PTR;

#define DRV_WNET_ERR_CODE_LEN    (sizeof(DRV_WNET_ERR_CODE))




/* Cmd for wireless network device ioctl. */
typedef enum _dev_wnet_ioctl_cmd
{
    /* ioctl command: register DMA transfer complete isr. */
    WNET_IOCTL_REGISTER_TXTC_ISR,
    
    /* ioctl command: register DMA receiver complete isr. */
    WNET_IOCTL_REGISTER_RXTC_ISR,
    
    /* ioctl command: register USART line idle isr. */
    WNET_IOCTL_REGISTER_LINE_IDLE_ISR
    
}DRV_WNET_IOCTL_CMD, *DRV_WNET_IOCTL_CMD_PTR;


/* Device driver structure for wireless network device. */
typedef struct _drv_wnet_st
{	
    /* Object's operation group. */
    DRV_WNET_ERR_CODE (*open) (void *);
    DRV_WNET_ERR_CODE (*close)(void *);
    DRV_WNET_ERR_CODE (*read) (void *, uint32_t, uint8_t *, uint32_t);
    DRV_WNET_ERR_CODE (*write)(void *, uint32_t, uint8_t *, uint32_t);
    DRV_WNET_ERR_CODE (*ioctl)(void *, DRV_WNET_IOCTL_CMD, void *);
    
    /* Function pointer for isr of DMA transfer complete. */
    void (*isr_tx_complete)(void *);
    void    * param_tx_complete_ptr;
    
    /* Function pointer for isr of DMA receiver complete. */
    void (*isr_rx_complete)(void *);
    void    * param_rx_complete_ptr;
    
    /* Function pointer for isr of USART line idle. */
    void   (*isr_line_idle)(void *);
    void      * param_line_idle_ptr;

}drv_wnet_st, *drv_wnet_st_ptr;

#define DRV_WNET_ST_LEN    (sizeof(drv_wnet_st))


/* Wnet user isr register structure. */
typedef struct _drv_wnet_isr_register_st
{
    void (*isr_function)(void *);
    void          *isr_param_ptr;
    
}drv_wnet_isr_register_st, *drv_wnet_isr_register_st_ptr;

#define DRV_WNET_ISR_REGISTER_ST_LEN    (sizeof(drv_wnet_isr_register_st))




 /* Main object for wnet driver. */
extern drv_wnet_st WnetDriver;

#define DRV_WNET_PTR    ((drv_wnet_st_ptr)&WnetDriver)
    
     
     
     
#ifdef __cplusplus
}
#endif

#endif /* _DRV_WNET_H_ */
