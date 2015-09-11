/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_gnss.c
 @brief  : this file include the driver functions for the GNSS module.
 @author : wangxianwen
 @history:
           2015-7-30    wangxianwen    Created file
           ...
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "drv_gnss.h"
#include "stm32f4xx.h"




/**
  * @brief  This function handles USARTx exception.
  * @param  None
  * @retval None
  */
void GNSS_USARTx_IRQHandler(void)
{
    /* Idle line detection interrupt. */
    if (USART_GetITStatus(GNSS_USARTx, USART_IT_IDLE))
    {
        /* Clear the specific interrupt flag(pending bit). Caution: can not 
           use 'USART_ClearITPendingBit' fuction. see 'USART_ClearITPendingBit' note. */
        USART_ReceiveData(GNSS_USARTx);
        
        /* Call user's isr function. */
        if(GnssDriver.isr_line_idle != (void*)0)
        {
            GnssDriver.isr_line_idle(GnssDriver.param_line_idle_ptr);
        }
    }
}


/**
  * @brief  This function handles USARTx DMA transfer exception.
  * @param  None
  * @retval None
  */
void GNSS_USARTx_DMA_TX_IRQHandler(void)
{
    /* Streamx FIFO error error interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_FEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */  
        DMA_ClearFlag(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_FEIF);
    }
   
    /* Streamx direct mode error interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_DMEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */        
        DMA_ClearFlag(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_DMEIF);
    }
   
    /* Streamx transfer error interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_TEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_TEIF);
    }
 
    /* Streamx half transfer complete interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_HTIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_HTIF);    
    }
    
    /* Streamx transfer complete interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_TCIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(GNSS_USARTx_TX_DMA_STREAM, GNSS_USARTx_TX_DMA_FLAG_TCIF);  
        
        /* Call user's isr function. */
        if(GnssDriver.isr_tx_complete != (void*)0)
        {
            GnssDriver.isr_tx_complete(GnssDriver.param_tx_complete_ptr);
        }
    }    
}


/**
  * @brief  This function handles USARTx DMA receiver exception.
  * @param  None
  * @retval None
  */
void GNSS_USARTx_DMA_RX_IRQHandler(void)
{
    /* Streamx FIFO error error interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_FEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */  
        DMA_ClearFlag(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_FEIF);
    }
   
    /* Streamx direct mode error interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_DMEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */        
        DMA_ClearFlag(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_DMEIF);
    }
   
    /* Streamx receiver error interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_TEIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_TEIF);
    }
 
    /* Streamx half receiver complete interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_HTIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_HTIF);    
    }
    
    /* Streamx receiver complete interrupt. */
    if (DMA_GetFlagStatus(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_TCIF))
    {
        /* Clear the specific interrupt flag(pending bit). */
        DMA_ClearFlag(GNSS_USARTx_RX_DMA_STREAM, GNSS_USARTx_RX_DMA_FLAG_TCIF);  
        
        /* Call user's isr function. */
        if(GnssDriver.isr_rx_complete != (void*)0)
        {
            GnssDriver.isr_rx_complete(GnssDriver.param_rx_complete_ptr);
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
        /* GPIO_Pin,         GPIO_Mode,      GPIO_Speed,      GPIO_OType,    GPIO_PuPd   */
        GNSS_USARTx_TX_PIN, GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP 
    };
    GPIO_InitTypeDef GPIO_init_rx = 
    { 
        /* GPIO_Pin,         GPIO_Mode,      GPIO_Speed,      GPIO_OType,    GPIO_PuPd   */
        GNSS_USARTx_RX_PIN, GPIO_Mode_AF, GPIO_Speed_100MHz, GPIO_OType_PP, GPIO_PuPd_UP 
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
        /*  USART_BaudRate,         USART_WordLength,       USART_StopBits,        USART_Parity,      USART_Mode,       USART_HardwareFlowControl */
        GNSS_USARTx_BAUDRATE,   GNSS_USARTx_WORD_LENGTH, GNSS_USARTx_STOP_BITS, GNSS_USARTx_PARITY, GNSS_USARTx_MODE, GNSS_USARTx_HARDWARE_FLOW_CTRL
    };


    NVIC_InitTypeDef NVIC_init_st = { GNSS_USARTx_IRQn, GNSS_USARTx_IRQ_PREPRIO, GNSS_USARTx_IRQ_SUBRIO, ENABLE };

    
    /* Peripheral Clock Enable -------------------------------------------------*/
    /* Enable GPIO clock. */
    RCC_AHB1PeriphClockCmd(GNSS_USARTx_TX_GPIO_CLK | GNSS_USARTx_RX_GPIO_CLK, ENABLE);

    /* Enable USART clock. */
    GNSS_USARTx_CLK_INIT(GNSS_USARTx_CLK, ENABLE);

    
    /* USARTx GPIO configuration -----------------------------------------------*/ 
    /* Connect USART pins to AF7. */
    GPIO_PinAFConfig(GNSS_USARTx_TX_GPIO_PORT, GNSS_USARTx_TX_SOURCE, GNSS_USARTx_TX_AF);
    GPIO_PinAFConfig(GNSS_USARTx_RX_GPIO_PORT, GNSS_USARTx_RX_SOURCE, GNSS_USARTx_RX_AF);

    /* Configure USART Tx and Rx as alternate function push-pull. */
    GPIO_Init(GNSS_USARTx_TX_GPIO_PORT, &GPIO_init_tx);
    GPIO_Init(GNSS_USARTx_RX_GPIO_PORT, &GPIO_init_rx);


    /* USARTx configuration ----------------------------------------------------*/
    /* Enable the USART OverSampling by 8. */
    USART_OverSampling8Cmd(GNSS_USARTx, ENABLE); 

    /* Initialize USART. */
    USART_Init(GNSS_USARTx, &USART_init_st);
    
    /* Enable USART DMA TX Requsts. */
    USART_DMACmd(GNSS_USARTx, USART_DMAReq_Tx, ENABLE);
    
    /* Enable USART DMA RX Requsts. */
    USART_DMACmd(GNSS_USARTx, USART_DMAReq_Rx, ENABLE);

    /* Enable the USART idle interrupt. */
    USART_ITConfig(GNSS_USARTx, USART_IT_IDLE, ENABLE);

    /* Initializes the USART NVIC peripheral. */
    NVIC_Init(&NVIC_init_st);

    /* Enable USART module. */
    USART_Cmd(GNSS_USARTx, ENABLE);
}


/**
  * @brief  Initialize the DMA Peripheral.
  * @param  None
  * @retval None
  */
static void DMA_init_module(void)
{
    NVIC_InitTypeDef NVIC_init_tx = { GNSS_USARTx_DMA_TX_IRQn, GNSS_USARTx_DMA_TX_IRQ_PREPRIO, GNSS_USARTx_DMA_TX_IRQ_SUBRIO, ENABLE };
    NVIC_InitTypeDef NVIC_init_rx = { GNSS_USARTx_DMA_RX_IRQn, GNSS_USARTx_DMA_RX_IRQ_PREPRIO, GNSS_USARTx_DMA_RX_IRQ_SUBRIO, ENABLE };
    
    
    /* Enable the DMA clock. */
    RCC_AHB1PeriphClockCmd(GNSS_USARTx_DMAx_CLK, ENABLE);
    
    /* Initializes the DMA tx/rx NVIC peripheral. */
    NVIC_Init(&NVIC_init_tx);
    NVIC_Init(&NVIC_init_rx);
}


/**
  * @brief  Main routine for gnss driver open.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_OPEN.
  */
static DRV_GNSS_ERR_CODE drv_gnss_open
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    /* USART configuration. */
    USART_init_module();
    
    /* USART DMA configuration. */
    DMA_init_module();

    return DRV_GNSS_ERR_OK;
}


/**
  * @brief  Main routine for gnss driver close.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_CLOSE.
  */
static DRV_GNSS_ERR_CODE drv_gnss_close
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    NVIC_InitTypeDef NVIC_deinit_tx = { GNSS_USARTx_DMA_TX_IRQn, GNSS_USARTx_DMA_TX_IRQ_PREPRIO, GNSS_USARTx_DMA_TX_IRQ_SUBRIO, DISABLE };
    NVIC_InitTypeDef NVIC_deinit_rx = { GNSS_USARTx_DMA_RX_IRQn, GNSS_USARTx_DMA_RX_IRQ_PREPRIO, GNSS_USARTx_DMA_RX_IRQ_SUBRIO, DISABLE };

    
    /* Disable and deinitialize USART. */
    USART_Cmd(GNSS_USARTx, DISABLE);
    USART_DeInit(GNSS_USARTx);
    
    /* Deinitialize the NVIC interrupt of DMA rx and tx. */
    NVIC_Init(&NVIC_deinit_tx);
    NVIC_Init(&NVIC_deinit_rx);
    
    /* Disable DMA tx and rx Stream. */
    DMA_Cmd(GNSS_USARTx_TX_DMA_STREAM, DISABLE);
    DMA_Cmd(GNSS_USARTx_RX_DMA_STREAM, DISABLE);
  
    /* Deinitialize DMA tx and rx stream. */
    DMA_DeInit(GNSS_USARTx_TX_DMA_STREAM);
    DMA_DeInit(GNSS_USARTx_RX_DMA_STREAM);

    return DRV_GNSS_ERR_OK;
}


/**
  * @brief  Main routine for wireless network driver read.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_READ.
  */
static DRV_GNSS_ERR_CODE drv_gnss_read
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
      /* DMA_Channel,                  DMA_PeripheralBaseAddr,           DMA_Memory0BaseAddr,            DMA_DIR,              DMA_BufferSize,             DMA_PeripheralInc, */
      GNSS_USARTx_RX_DMA_CHANNEL,    GNSS_USARTx_PERIPHERAL_BASE_ADDR, (uint32_t)buffer_ptr,            GNSS_USARTx_RX_DMA_DIR,    size,                  GNSS_USARTx_PERIPHERAL_INC,

      /* DMA_MemoryInc,                DMA_PeripheralDataSize,           DMA_MemoryDataSize,             DMA_Mode,              DMA_Priority,              DMA_FIFOMode,      */
      GNSS_USARTx_DMA_MEMORY_INC,    GNSS_USARTx_PERIPHERAL_DATASIZE,  GNSS_USARTx_DMA_MEMORY_DATASIZE, GNSS_USARTx_DMA_MODE, GNSS_USARTx_DMA_PRIORITY,   GNSS_USARTx_DMA_FIFOMODE,
        
      /* DMA_FIFOThreshold,            DMA_MemoryBurst,                  DMA_PeripheralBurst */
      GNSS_USARTx_DMA_FIFOTHRESHOLD, GNSS_USARTx_DMA_MEMORYBURST,      GNSS_USARTx_PERIPHERAL_BURST
    };

    
    /* Error detection. */
    if( (drv_ptr == (void*)0) || (buffer_ptr == (void*)0) || (size == 0) )
    {
        return DRV_GNSS_ERR_READ;
    }

    /* Deinitialize DMA rx stream. */
    DMA_DeInit(GNSS_USARTx_RX_DMA_STREAM);
    
    /* Initialize DMA USART RX Stream. */
    DMA_Init(GNSS_USARTx_RX_DMA_STREAM, &DMA_init_rx);
    
    /* Configure DMA Transfer complete interrupt.  */
    DMA_ITConfig(GNSS_USARTx_RX_DMA_STREAM, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
    
    /* Enable DMA USART TX Stream and start on transfer process. */
    DMA_Cmd(GNSS_USARTx_RX_DMA_STREAM, ENABLE); 

    return DRV_GNSS_ERR_OK;
}


/**
  * @brief  Main routine for GNSS driver write.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_WRITE.
  */
static DRV_GNSS_ERR_CODE drv_gnss_write
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
      /* DMA_Channel,                    DMA_PeripheralBaseAddr,         DMA_Memory0BaseAddr,                DMA_DIR,             DMA_BufferSize,             DMA_PeripheralInc, */
      GNSS_USARTx_TX_DMA_CHANNEL,    GNSS_USARTx_PERIPHERAL_BASE_ADDR, (uint32_t)buffer_ptr,            GNSS_USARTx_TX_DMA_DIR,  size,                     GNSS_USARTx_PERIPHERAL_INC,

      /* DMA_MemoryInc,                  DMA_PeripheralDataSize,         DMA_MemoryDataSize,                 DMA_Mode,            DMA_Priority,               DMA_FIFOMode,      */
      GNSS_USARTx_DMA_MEMORY_INC,    GNSS_USARTx_PERIPHERAL_DATASIZE,  GNSS_USARTx_DMA_MEMORY_DATASIZE, GNSS_USARTx_DMA_MODE,    GNSS_USARTx_DMA_PRIORITY, GNSS_USARTx_DMA_FIFOMODE,
      
      /* DMA_FIFOThreshold,              DMA_MemoryBurst,                DMA_PeripheralBurst */
      GNSS_USARTx_DMA_FIFOTHRESHOLD, GNSS_USARTx_DMA_MEMORYBURST,      GNSS_USARTx_PERIPHERAL_BURST
    };
    

    /* Error detection. Caution: do not examine "buffer_ptr" when from memory to peripheral. */
    if( (drv_ptr == (void*)0) || (size == 0) )
    {
        return DRV_GNSS_ERR_WRITE;
    }
    
    /* Deinitialize DMA tx stream. */
    DMA_DeInit(GNSS_USARTx_TX_DMA_STREAM);
    
    /* Initialize DMA USART TX Stream. */
    DMA_Init(GNSS_USARTx_TX_DMA_STREAM, &DMA_init_tx);
    
    /* Configure DMA Transfer complete interrupt.  */
    DMA_ITConfig(GNSS_USARTx_TX_DMA_STREAM, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE, ENABLE);
           
    /* Enable DMA USART TX Stream and start on transfer process. */
    DMA_Cmd(GNSS_USARTx_TX_DMA_STREAM, ENABLE); 

    return DRV_GNSS_ERR_OK;
}


/**
  * @brief  Register transfer complete isr.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_IOCTL.
  */
static DRV_GNSS_ERR_CODE gnss_ioctl_register_txtc_isr
(
    /* Pointer to device driver structure. */
    drv_gnss_st_ptr  gnss_ptr,

    /* pinter to the registered isr. */
    drv_gnss_isr_register_st_ptr isr_ptr
)
{
    /* Error detection. */
    if( (gnss_ptr == (void*)0) || (isr_ptr == (void*)0) )
    {
        return DRV_GNSS_ERR_IOCTL;
    }
    
    gnss_ptr->isr_tx_complete = isr_ptr->isr_function;
    gnss_ptr->param_tx_complete_ptr = isr_ptr->isr_param_ptr;
    
    return DRV_GNSS_ERR_OK;
}


/**
  * @brief  Register receiver complete isr.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_IOCTL.
  */
static DRV_GNSS_ERR_CODE gnss_ioctl_register_rxtc_isr
(
    /* Pointer to device driver structure. */
    drv_gnss_st_ptr  gnss_ptr,

    /* pinter to the registered isr. */
    drv_gnss_isr_register_st_ptr isr_ptr
)
{
    /* Error detection. */
    if( (gnss_ptr == (void*)0) || (isr_ptr == (void*)0) )
    {
        return DRV_GNSS_ERR_IOCTL;
    }

    gnss_ptr->isr_rx_complete = isr_ptr->isr_function;
    gnss_ptr->param_rx_complete_ptr = isr_ptr->isr_param_ptr;
    
    return DRV_GNSS_ERR_OK;
}


/**
  * @brief  Register line idle isr.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_IOCTL.
  */
static DRV_GNSS_ERR_CODE gnss_ioctl_register_line_idle_isr
(
    /* Pointer to device driver structure. */
    drv_gnss_st_ptr  gnss_ptr,

    /* pinter to the registered isr. */
    drv_gnss_isr_register_st_ptr isr_ptr
)
{
    /* Error detection. */
    if( (gnss_ptr == (void*)0) || (isr_ptr == (void*)0) )
    {
        return DRV_GNSS_ERR_IOCTL;
    }

    gnss_ptr->isr_line_idle = isr_ptr->isr_function;
    gnss_ptr->param_line_idle_ptr = isr_ptr->isr_param_ptr;
    
    return DRV_GNSS_ERR_OK;
}


/**
  * @brief  Main routine for gnss driver ioctl.
  * @param  See below.
  * @retval DRV_GNSS_ERR_OK or DRV_GNSS_ERR_IOCTL.
  */
static DRV_GNSS_ERR_CODE drv_gnss_ioctl
(
	/* Pointer of device driver structure. */
	void *drv_ptr, 
	
	/* Ioctl command. */
	DRV_GNSS_IOCTL_CMD cmd, 
	
	/* Pointer of parameter. */
	void *param_ptr
)
{
	DRV_GNSS_ERR_CODE err_code = DRV_GNSS_ERR_IOCTL;
	
	
	switch (cmd)
	{
        /* register transfer complete isr. */
		case GNSS_IOCTL_REGISTER_TXTC_ISR:          {  err_code = gnss_ioctl_register_txtc_isr(drv_ptr, param_ptr);       break;  }
      
        /* register receiver complete isr. */
		case GNSS_IOCTL_REGISTER_RXTC_ISR:          {  err_code = gnss_ioctl_register_rxtc_isr(drv_ptr, param_ptr);       break;  }
        
        /* register line idle isr. */
		case GNSS_IOCTL_REGISTER_LINE_IDLE_ISR:     {  err_code = gnss_ioctl_register_line_idle_isr(drv_ptr, param_ptr);  break;  }	
        
		default:                                    {  err_code = DRV_GNSS_ERR_IOCTL;                                     break;  }			
	}
	
	return err_code;
}



/* Main object for gnss driver. */
drv_gnss_st GnssDriver = 
{ 
    drv_gnss_open, drv_gnss_close, drv_gnss_read, drv_gnss_write, drv_gnss_ioctl, 
    0, 0,  0, 0 
};
















/***************************** NMEA0183 protecol **********************************/

/* Nmea separator index group. */
uint16_t NmeaSeparatorIndex[32] = { 0 };


#if 0

/**
  * @brief  Get the all the separator index from nmea frame.
  * @param  See below.
  * @retval None.
  */
static void nmea_update_separator_index
(
    /* Frame buffuer address. */
    char *buffer_ptr
)
{
    /* Character index and separator index. */
    uint16_t char_index = 0;
    uint16_t  sep_index = 0;


    /* Clear the separator index group. */
    memset(sep_group, 0, sep_max * sizeof(uint16_t));

    /* Search for comma and record the index. */
    for (char_index = 0; (char_index < (uint16_t)strlen(buffer_ptr)) && (sep_index < sep_max); char_index ++)
    {
        if (buffer_ptr[char_index] == ',')
        {
            sep_group[sep_index] = char_index;
            sep_index ++;
        }   
    }
}


/**
  * @brief  Get the specific item address from nmea frame.
  * @param  See below.
  * @retval Item address.
  */
char* nmea_get_item_address
(
    /* Frame buffuer address. */
    char *buffer_ptr,

    /* Separator index group. */
    uint16_t (*sep_group)[],

    /* Item index in nmea frame. */
    uint16_t item_index
)
{
    if (item_index < 1)
    {
        return &(buffer_ptr[0]);
    }
    else
    {
        return &(buffer_ptr[sep_group[item_index - 1] + 1]);
    }
    
}

#endif



