/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_wnet.c
 @brief  : this file include the driver functions for the wireless network 
           transport module.
 @author : wangxianwen
 @history:
           2015-6-17    wangxianwen    Created file
           ...
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "drv_wnet.h"
#include "stm32f4xx.h"

uint32_t wnet_num = 0;


/**
  * @brief  This function handles USARTx exception.
  * @param  None
  * @retval None
  */
void USARTx_IRQHandler(void)
{
    /* Idle line detection interrupt. */
    if (USART_GetITStatus(USARTx, USART_IT_IDLE))
    {
        /* Clear the specific interrupt flag(pending bit). Caution: can not 
           use 'USART_ClearITPendingBit' fuction. see 'USART_ClearITPendingBit' note. */
        USART_ReceiveData(USARTx);
        
        /* Call user's isr function. */
        if(WnetDriver.isr_line_idle != (void*)0)
        {
            WnetDriver.isr_line_idle(WnetDriver.param_line_idle_ptr);
            wnet_num ++;
        }
    }
}


/**
  * @brief  This function handles USARTx DMA transfer exception.
  * @param  None
  * @retval None
  */
void USARTx_DMA_TX_IRQHandler(void)
{
    /* Streamx FIFO error error interrupt. */
    if (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_FEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */  
        DMA_ClearFlag(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_FEIF);
    }
   
    /* Streamx direct mode error interrupt. */
    if (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_DMEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */        
        DMA_ClearFlag(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_DMEIF);
    }
   
    /* Streamx transfer error interrupt. */
    if (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_TEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_TEIF);
    }
 
    /* Streamx half transfer complete interrupt. */
    if (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_HTIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_HTIF);    
    }
    
    /* Streamx transfer complete interrupt. */
    if (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_TCIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(USARTx_TX_DMA_STREAM, USARTx_TX_DMA_FLAG_TCIF);  
        
        /* Call user's isr function. */
        if(WnetDriver.isr_tx_complete != (void*)0)
        {
            WnetDriver.isr_tx_complete(WnetDriver.param_tx_complete_ptr);
        }
    }    
}


/**
  * @brief  This function handles USARTx DMA receiver exception.
  * @param  None
  * @retval None
  */
void USARTx_DMA_RX_IRQHandler(void)
{
    /* Streamx FIFO error error interrupt. */
    if (DMA_GetFlagStatus(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_FEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */  
        DMA_ClearFlag(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_FEIF);
    }
   
    /* Streamx direct mode error interrupt. */
    if (DMA_GetFlagStatus(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_DMEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */        
        DMA_ClearFlag(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_DMEIF);
    }
   
    /* Streamx receiver error interrupt. */
    if (DMA_GetFlagStatus(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_TEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_TEIF);
    }
 
    /* Streamx half receiver complete interrupt. */
    if (DMA_GetFlagStatus(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_HTIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_HTIF);    
    }
    
    /* Streamx receiver complete interrupt. */
    if (DMA_GetFlagStatus(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_TCIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(USARTx_RX_DMA_STREAM, USARTx_RX_DMA_FLAG_TCIF);  
        
        /* Call user's isr function. */
        if(WnetDriver.isr_rx_complete != (void*)0)
        {
            WnetDriver.isr_rx_complete(WnetDriver.param_rx_complete_ptr);
        }
    }    
}


/**
  * @brief  Initialize the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_init_module(void)
{
    GPIO_InitTypeDef GPIO_init_tx = 
    { 
        /* GPIO_Pin,    GPIO_Mode,      GPIO_Speed,      GPIO_OType,    GPIO_PuPd   */
        USARTx_TX_PIN, GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP 
    };
    GPIO_InitTypeDef GPIO_init_rx = 
    { 
        /* GPIO_Pin,    GPIO_Mode,      GPIO_Speed,      GPIO_OType,    GPIO_PuPd   */
        USARTx_RX_PIN, GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP 
    };
    
    /* Caution: When using Parity the word length must be configured to 9 bits. */
    /* - Maximum BaudRate that can be achieved when using the Oversampling by 8 is: (USART APB Clock / 8) 
         Example: (USART3 APB1 Clock / 8) = (42 MHz / 8) = 5250000 baud
         Example: (USART1 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud
       - Maximum BaudRate that can be achieved when using the Oversampling by 16 is: (USART APB Clock / 16) 
         Example: (USART3 APB1 Clock / 16) = (42 MHz / 16) = 2625000 baud
         Example: (USART1 APB2 Clock / 16) = (84 MHz / 16) = 5250000 baud       */ 
    USART_InitTypeDef USART_init_st = 
    {
        /* USART_BaudRate,  USART_WordLength,   USART_StopBits,  USART_Parity,  USART_Mode,  USART_HardwareFlowControl */
        USARTx_BAUDRATE,   USARTx_WORD_LENGTH, USARTx_STOP_BITS, USARTx_PARITY, USARTx_MODE, USARTx_HARDWARE_FLOW_CTRL
    };


    NVIC_InitTypeDef NVIC_init_st = { USARTx_IRQn, USARTx_IRQ_PREPRIO, USARTx_IRQ_SUBRIO, ENABLE };

    
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock. */
    RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

    /* Enable USART clock. */
    USARTx_CLK_INIT(USARTx_CLK, ENABLE);

    
    /* USARTx GPIO configuration -----------------------------------------------*/ 
    /* Connect USART pins to AF7. */
    GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
    GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

    /* Configure USART Tx and Rx as alternate function push-pull. */
    GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_init_tx);
    GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_init_rx);


    /* USARTx configuration ----------------------------------------------------*/
    /* Enable the USART OverSampling by 8. */
    USART_OverSampling8Cmd(USARTx, ENABLE); 

    /* Initialize USART. */
    USART_Init(USARTx, &USART_init_st);
    
    /* Enable USART DMA TX Requsts. */
    USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
    
    /* Enable USART DMA RX Requsts. */
    USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);

    /* Enable the USART idle interrupt. */
    USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);

    /* Initializes the USART NVIC peripheral. */
    NVIC_Init(&NVIC_init_st);

    /* Enable USART module. */
    USART_Cmd(USARTx, ENABLE);
}


/**
  * @brief  Initialize the DMA Peripheral.
  * @param  None
  * @retval None
  */
static void DMA_init_module(void)
{
    NVIC_InitTypeDef NVIC_init_tx = { USARTx_DMA_TX_IRQn, USARTx_DMA_TX_IRQ_PREPRIO, USARTx_DMA_TX_IRQ_SUBRIO, ENABLE };
    NVIC_InitTypeDef NVIC_init_rx = { USARTx_DMA_RX_IRQn, USARTx_DMA_RX_IRQ_PREPRIO, USARTx_DMA_RX_IRQ_SUBRIO, ENABLE };
    
    
    /* Enable the DMA clock. */
    RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);
    
    /* Initializes the DMA tx/rx NVIC peripheral. */
    NVIC_Init(&NVIC_init_tx);
    NVIC_Init(&NVIC_init_rx);
}


/**
  * @brief  Main routine for wnet driver open.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_OPEN.
  */
static DRV_WNET_ERR_CODE drv_wnet_open
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    /* USART configuration. */
    USART_init_module();
    
    /* USART DMA configuration. */
    DMA_init_module();

    return DRV_WNET_ERR_OK;
}


/**
  * @brief  Main routine for wireless network driver close.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_CLOSE.
  */
static DRV_WNET_ERR_CODE drv_wnet_close
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    NVIC_InitTypeDef NVIC_deinit_tx = { USARTx_DMA_TX_IRQn, USARTx_DMA_TX_IRQ_PREPRIO, USARTx_DMA_TX_IRQ_SUBRIO, DISABLE };
    NVIC_InitTypeDef NVIC_deinit_rx = { USARTx_DMA_RX_IRQn, USARTx_DMA_RX_IRQ_PREPRIO, USARTx_DMA_RX_IRQ_SUBRIO, DISABLE };

    
    /* Disable and deinitialize USART. */
    USART_Cmd(USARTx, DISABLE);
    USART_DeInit(USARTx);
    
    /* Deinitialize the NVIC interrupt of DMA rx and tx. */
    NVIC_Init(&NVIC_deinit_tx);
    NVIC_Init(&NVIC_deinit_rx);
    
    /* Disable DMA tx and rx Stream. */
    DMA_Cmd(USARTx_TX_DMA_STREAM, DISABLE);
    DMA_Cmd(USARTx_RX_DMA_STREAM, DISABLE);
  
    /* Deinitialize DMA tx and rx stream. */
    DMA_DeInit(USARTx_TX_DMA_STREAM);
    DMA_DeInit(USARTx_RX_DMA_STREAM);

    return DRV_WNET_ERR_OK;
}


/**
  * @brief  Main routine for wireless network driver read.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_READ.
  */
static DRV_WNET_ERR_CODE drv_wnet_read
(
	/* Pointer of device driver structure. */
	void *drv_ptr,
    
    /* Data address for data fetching. */
    uint32_t addr,
    
    /* Data destination buffer address. */
    uint8_t *buffer_ptr,
    
    /* Data size based on uint8_t. */
    uint32_t size
)
{
    DMA_InitTypeDef   DMA_init_rx = 
    { 
      /* DMA_Channel,             DMA_PeripheralBaseAddr,      DMA_Memory0BaseAddr,        DMA_DIR,         DMA_BufferSize,       DMA_PeripheralInc, */
      USARTx_RX_DMA_CHANNEL,    USARTx_PERIPHERAL_BASE_ADDR, (uint32_t)buffer_ptr,       USARTx_RX_DMA_DIR,      size,          USARTx_PERIPHERAL_INC,

      /* DMA_MemoryInc,           DMA_PeripheralDataSize,      DMA_MemoryDataSize,         DMA_Mode,        DMA_Priority,         DMA_FIFOMode,      */
      USARTx_DMA_MEMORY_INC,    USARTx_PERIPHERAL_DATASIZE,  USARTx_DMA_MEMORY_DATASIZE, USARTx_DMA_MODE,  USARTx_DMA_PRIORITY, USARTx_DMA_FIFOMODE,
        
      /* DMA_FIFOThreshold,       DMA_MemoryBurst,             DMA_PeripheralBurst */
      USARTx_DMA_FIFOTHRESHOLD, USARTx_DMA_MEMORYBURST,      USARTx_PERIPHERAL_BURST
    };

    
    /* Error detection. */
    if( (drv_ptr == (void*)0) || (buffer_ptr == (void*)0) || (size == 0) )
    {
        return DRV_WNET_ERR_READ;
    }

    /* Deinitialize DMA rx stream. */
    DMA_DeInit(USARTx_RX_DMA_STREAM);
    
    /* Initialize DMA USART RX Stream. */
    DMA_Init(USARTx_RX_DMA_STREAM, &DMA_init_rx);
    
    /* Configure DMA Transfer complete interrupt.  */
    DMA_ITConfig(USARTx_RX_DMA_STREAM, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    
    /* Enable DMA USART TX Stream and start on transfer process. */
    DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE); 

    return DRV_WNET_ERR_OK;
}


/**
  * @brief  Main routine for wireless network driver write.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_WRITE.
  */
static DRV_WNET_ERR_CODE drv_wnet_write
(
    /* Pointer of device driver structure. */
	void *drv_ptr,
    
    /* Data address for data writing. */
    uint32_t addr,
    
    /* Data source buffer address. */
    uint8_t *buffer_ptr,
    
    /* Data size based on uint8_t. */
    uint32_t size
)
{
    DMA_InitTypeDef   DMA_init_tx = 
    { 
      /* DMA_Channel,             DMA_PeripheralBaseAddr,      DMA_Memory0BaseAddr,        DMA_DIR,          DMA_BufferSize,       DMA_PeripheralInc, */
      USARTx_TX_DMA_CHANNEL,    USARTx_PERIPHERAL_BASE_ADDR, (uint32_t)buffer_ptr,      USARTx_TX_DMA_DIR,     size,            USARTx_PERIPHERAL_INC,

      /* DMA_MemoryInc,           DMA_PeripheralDataSize,      DMA_MemoryDataSize,         DMA_Mode,         DMA_Priority,         DMA_FIFOMode,      */
      USARTx_DMA_MEMORY_INC,    USARTx_PERIPHERAL_DATASIZE, USARTx_DMA_MEMORY_DATASIZE, USARTx_DMA_MODE,   USARTx_DMA_PRIORITY, USARTx_DMA_FIFOMODE,
      
      /* DMA_FIFOThreshold,       DMA_MemoryBurst,             DMA_PeripheralBurst */
      USARTx_DMA_FIFOTHRESHOLD, USARTx_DMA_MEMORYBURST,     USARTx_PERIPHERAL_BURST
    };
    

    /* Error detection. Caution: do not examine "buffer_ptr" when from memory to peripheral. */
    if( (drv_ptr == (void*)0) || (size == 0) )
    {
        return DRV_WNET_ERR_WRITE;
    }
    
    /* Deinitialize DMA tx stream. */
    DMA_DeInit(USARTx_TX_DMA_STREAM);
    
    /* Initialize DMA USART TX Stream. */
    DMA_Init(USARTx_TX_DMA_STREAM, &DMA_init_tx);
    
    /* Configure DMA Transfer complete interrupt.  */
    DMA_ITConfig(USARTx_TX_DMA_STREAM, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
           
    /* Enable DMA USART TX Stream and start on transfer process. */
    DMA_Cmd(USARTx_TX_DMA_STREAM, ENABLE); 

    return DRV_WNET_ERR_OK;
}


/**
  * @brief  Register wnet transfer complete isr.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_IOCTL.
  */
static DRV_WNET_ERR_CODE wnet_ioctl_register_txtc_isr
(
    /* Pointer to device driver structure. */
    drv_wnet_st_ptr  wnet_ptr,

    /* pinter to the registered isr. */
    drv_wnet_isr_register_st_ptr isr_ptr
)
{
    /* Error detection. */
    if( (wnet_ptr == (void*)0) || (isr_ptr == (void*)0) )
    {
        return DRV_WNET_ERR_IOCTL;
    }
    
    wnet_ptr->isr_tx_complete = isr_ptr->isr_function;
    wnet_ptr->param_tx_complete_ptr = isr_ptr->isr_param_ptr;
    
    return DRV_WNET_ERR_OK;
}


/**
  * @brief  Register wnet receiver complete isr.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_IOCTL.
  */
static DRV_WNET_ERR_CODE wnet_ioctl_register_rxtc_isr
(
    /* Pointer to device driver structure. */
    drv_wnet_st_ptr  wnet_ptr,

    /* pinter to the registered isr. */
    drv_wnet_isr_register_st_ptr isr_ptr
)
{
    /* Error detection. */
    if( (wnet_ptr == (void*)0) || (isr_ptr == (void*)0) )
    {
        return DRV_WNET_ERR_IOCTL;
    }

    wnet_ptr->isr_rx_complete = isr_ptr->isr_function;
    wnet_ptr->param_rx_complete_ptr = isr_ptr->isr_param_ptr;
    
    return DRV_WNET_ERR_OK;
}


/**
  * @brief  Register wnet line idle isr.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_IOCTL.
  */
static DRV_WNET_ERR_CODE wnet_ioctl_register_line_idle_isr
(
    /* Pointer to device driver structure. */
    drv_wnet_st_ptr  wnet_ptr,

    /* pinter to the registered isr. */
    drv_wnet_isr_register_st_ptr isr_ptr
)
{
    /* Error detection. */
    if( (wnet_ptr == (void*)0) || (isr_ptr == (void*)0) )
    {
        return DRV_WNET_ERR_IOCTL;
    }

    wnet_ptr->isr_line_idle = isr_ptr->isr_function;
    wnet_ptr->param_line_idle_ptr = isr_ptr->isr_param_ptr;
    
    return DRV_WNET_ERR_OK;
}


/**
  * @brief  Main routine for wireless network driver ioctl.
  * @param  See below.
  * @retval DRV_WNET_ERR_OK or DRV_WNET_ERR_IOCTL.
  */
static DRV_WNET_ERR_CODE drv_wnet_ioctl
(
    /* Pointer of device driver structure. */
    void *drv_ptr, 
	
	/* Ioctl command. */
    DRV_WNET_IOCTL_CMD cmd, 
	
    /* Pointer of parameter. */
    void *param_ptr
)
{
    DRV_WNET_ERR_CODE err_code = DRV_WNET_ERR_IOCTL;
	
	
    switch (cmd)
    {
        /* register wnet transfer complete isr. */
		case WNET_IOCTL_REGISTER_TXTC_ISR:          {  err_code = wnet_ioctl_register_txtc_isr(drv_ptr, param_ptr);       break;  }
      
        /* register wnet receiver complete isr. */
		case WNET_IOCTL_REGISTER_RXTC_ISR:          {  err_code = wnet_ioctl_register_rxtc_isr(drv_ptr, param_ptr);       break;  }
        
        /* register wnet line idle isr. */
		case WNET_IOCTL_REGISTER_LINE_IDLE_ISR:     {  err_code = wnet_ioctl_register_line_idle_isr(drv_ptr, param_ptr);  break;  }	
        
		default:                                    {  err_code = DRV_WNET_ERR_IOCTL;                                     break;  }
			
	}
	
	return err_code;
}





/* Main object for wireless network driver. */
drv_wnet_st WnetDriver = 
{ 
    drv_wnet_open, drv_wnet_close, drv_wnet_read, drv_wnet_write, drv_wnet_ioctl, 
    0, 0,  0, 0 
};
