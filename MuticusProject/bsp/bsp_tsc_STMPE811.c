/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bsp_tsc_STMPE811.c
 @brief  : This file include the bsp functions for the touch sensor control 
           module STMPE811.
 @author : gexueyuan wangxianwen
 @history:
           2015-9-07    gexueyuan wangxianwen    Created file
           ...
******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "bsp_tsc_STMPE811.h"


/* Private_FunctionPrototypes. -----------------------------------------------*/ 

static void TSC_delay(__IO uint32_t nCount);

static TSC_Status_TypDef TSC_I2C_WriteDeviceRegister_1B(uint8_t RegisterAddr, uint8_t RegisterValue);
static TSC_Status_TypDef TSC_I2C_ReadDeviceRegister_1B(uint8_t RegisterAddr, uint8_t *data_ptr);
static TSC_Status_TypDef TSC_I2C_ReadDeviceRegister_2B(uint8_t RegisterAddr, uint16_t *data_ptr);

#if 0
static void              TSC_I2CDMA_init_module(TSC_DMADirection_TypeDef Direction, uint8_t* buffer);
static TSC_Status_TypDef TSC_I2CDMA_WriteDeviceRegister_1B(uint8_t RegisterAddr, uint8_t RegisterValue);
static TSC_Status_TypDef TSC_I2CDMA_ReadDeviceRegister_1B(uint8_t RegisterAddr, uint8_t *data_ptr);
static TSC_Status_TypDef TSC_I2CDMA_ReadDeviceRegister_2B(uint8_t RegisterAddr, uint16_t *data_ptr);
#endif

static TSC_Status_TypDef TSC_cmd_clock_ctrl(uint8_t Fct, FunctionalState NewState);
static TSC_Status_TypDef TSC_cmd_gpio_alt_ctrl(uint8_t IO_Pin, FunctionalState NewState);
static FlagStatus        TSC_cmd_get_int_status(uint8_t Global_IT);
static TSC_Status_TypDef TSC_cmd_clear_int_status(uint8_t Global_IT);
static TSC_Status_TypDef TSC_cmd_int_ctrl(FunctionalState NewState);
static TSC_Status_TypDef TSC_cmd_int_enable(uint8_t Global_IT, FunctionalState NewState);
static TSC_Status_TypDef TSC_cmd_reset_chip(void);


static void TSC_gpio_init(void);
static void TSC_i2c_init(void);
static TSC_Status_TypDef TSC_intfunc_init(void);
static TSC_Status_TypDef TSC_touch_panel_init(void);



/* Function definition. ------------------------------------------------------*/ 

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void TSC_delay(__IO uint32_t nCount)
{
    __IO uint32_t index = 0; 

    
    for(index = (100000 * nCount); index != 0; index--)
    {
        ;
    }
}


/**
  * @brief  Writes a value in a register of the device through I2C.
  * @param  RegisterAddr: The target register address
  * @param  RegisterValue: The target register value to be written 
  * @retval TSC_OK: if all operations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_I2C_WriteDeviceRegister_1B(uint8_t RegisterAddr, uint8_t RegisterValue)
{
    uint8_t  reg_value = 0;
    uint32_t  time_out = TIMEOUT_MAX;


    /* Begin the configuration sequence */
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on EV5 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_SB))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Transmit the slave address and enable writing operation */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    time_out = TIMEOUT_MAX;  
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_ADDR))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Read status register 2 to clear ADDR flag */
    TSC_I2C->SR2;

    /* Test on EV8_1 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_TXE))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Transmit the first address for r/w operations */
    I2C_SendData(TSC_I2C, RegisterAddr);

    /* Test on EV8 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_TXE))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }

    /* Prepare the register value to be sent */
    I2C_SendData(TSC_I2C, RegisterValue);

    /* Test on EV8_2 and clear it */
    time_out = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_TXE)) || (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_BTF)))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }

    /* End the configuration sequence */
    I2C_GenerateSTOP(TSC_I2C, ENABLE);


    /* Verify that the loaded data is correct. */
    TSC_I2C_ReadDeviceRegister_1B(RegisterAddr, &reg_value);

    return (reg_value != RegisterValue) ? TSC_FAILURE : TSC_OK;
    
TIMEOUT_FLAG:

    return TSC_TIMEOUT;  
}


/**
  * @brief  Reads a register of the device through I2C without DMA.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  *         data_ptr: The target data address
  * @retval Touch sensor status.   
  */ 
static TSC_Status_TypDef TSC_I2C_ReadDeviceRegister_1B(uint8_t RegisterAddr, uint8_t *data_ptr)
{
    uint32_t time_out = TIMEOUT_MAX; 


    /* Enable the I2C peripheral. */
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on EV5 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_SB))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }

    
    /* Disable Acknowledgement */
    I2C_AcknowledgeConfig(TSC_I2C, DISABLE);


    /* Transmit the slave address and enable writing operation */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    time_out = TIMEOUT_MAX;  
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_ADDR))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Read status register 2 to clear ADDR flag */
    TSC_I2C->SR2;

    /* Test on EV8 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_TXE))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Transmit the first address for r/w operations */
    I2C_SendData(TSC_I2C, RegisterAddr);

    /* Test on EV8 and clear it */
    time_out = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_TXE)) || (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_BTF)))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Regenerate a start condition */
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on EV5 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_SB))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Transmit the slave address and enable writing operation */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_ADDR))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Read status register 2 to clear ADDR flag */
    TSC_I2C->SR2;

    /* Test on EV7 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_RXNE))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* End the configuration sequence */
    I2C_GenerateSTOP(TSC_I2C, ENABLE);

    /* Load the register value */
    *data_ptr = I2C_ReceiveData(TSC_I2C);

    /* Enable Acknowledgement */
    I2C_AcknowledgeConfig(TSC_I2C, ENABLE);

    /* Return the read value */
    return TSC_OK;

TIMEOUT_FLAG:

    return TSC_TIMEOUT;
}


/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  RegisterAddr: The target register adress (between 00x and 0x24)
  * @retval The data in the buffer containing the two returned bytes (in halfword).   
  */
static TSC_Status_TypDef TSC_I2C_ReadDeviceRegister_2B(uint8_t RegisterAddr, uint16_t *data_ptr)
{
    uint32_t time_out = TIMEOUT_MAX; 
    uint8_t TSC_BufferRX[2] = {0x00, 0x00};  


    /* Enable the I2C peripheral */
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on EV5 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_SB))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send device address for write */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    time_out = TIMEOUT_MAX;  
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_ADDR))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Read status register 2 to clear ADDR flag */
    TSC_I2C->SR2;

    /* Test on EV8 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_TXE))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send the device's internal address to write to */
    I2C_SendData(TSC_I2C, RegisterAddr);  


    /* Send START condition a second time */  
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on EV5 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_SB))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send device address for read */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_ADDR))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Disable Acknowledgement and set Pos bit */
    I2C_AcknowledgeConfig(TSC_I2C, DISABLE);       
    I2C_NACKPositionConfig(TSC_I2C, I2C_NACKPosition_Next);

    /* Read status register 2 to clear ADDR flag */
    TSC_I2C->SR2;

    /* Test on EV7 and clear it */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_BTF))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Read the bytes from the register. */
    TSC_BufferRX[1] = I2C_ReceiveData(TSC_I2C);
    TSC_BufferRX[0] = I2C_ReceiveData(TSC_I2C);
                                         
    /* Enable Acknowledgement and reset POS bit to be ready for another reception */
    I2C_AcknowledgeConfig(TSC_I2C, ENABLE);
    I2C_NACKPositionConfig(TSC_I2C, I2C_NACKPosition_Current);

    /* Send STOP Condition */
    I2C_GenerateSTOP(TSC_I2C, ENABLE);

    *data_ptr = (uint16_t) TSC_BufferRX[0] | ((uint16_t)TSC_BufferRX[1]<< 8);

    return TSC_OK;

TIMEOUT_FLAG:

    return TSC_TIMEOUT;  
}


#if 0
/**
  * @brief  Configure the DMA Peripheral used to handle communication via I2C.
  * @param  None
  * @retval None
  */
static void TSC_I2CDMA_init_module(TSC_DMADirection_TypeDef Direction, uint8_t* buffer)
{
    DMA_InitTypeDef DMA_InitStructure = { 0 };


    RCC_AHB1PeriphClockCmd(TSC_DMA_CLK, ENABLE);

    /* Initialize the DMA_Channel member */
    DMA_InitStructure.DMA_Channel = TSC_DMA_CHANNEL;

    /* Initialize the DMA_PeripheralBaseAddr member */
    DMA_InitStructure.DMA_PeripheralBaseAddr = TSC_I2C_DR;

    /* Initialize the DMA_Memory0BaseAddr member */
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buffer;

    /* Initialize the DMA_PeripheralInc member */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

    /* Initialize the DMA_MemoryInc member */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;

    /* Initialize the DMA_PeripheralDataSize member */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

    /* Initialize the DMA_MemoryDataSize member */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

    /* Initialize the DMA_Mode member */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

    /* Initialize the DMA_Priority member */
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;

    /* Initialize the DMA_FIFOMode member */
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;

    /* Initialize the DMA_FIFOThreshold member */
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;

    /* Initialize the DMA_MemoryBurst member */
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;

    /* Initialize the DMA_PeripheralBurst member */
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    if (Direction == TSC_DMA_RX)
    {    
        /* Initialize the DMA_DIR member */
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;

        /* Initialize the DMA_BufferSize member */
        DMA_InitStructure.DMA_BufferSize = 2;

        DMA_DeInit(TSC_DMA_RX_STREAM);

        DMA_Init(TSC_DMA_RX_STREAM, &DMA_InitStructure);
    }
    else if (Direction == TSC_DMA_TX)
    { 
        /* Initialize the DMA_DIR member */
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;

        /* Initialize the DMA_BufferSize member */
        DMA_InitStructure.DMA_BufferSize = 1;

        DMA_DeInit(TSC_DMA_TX_STREAM);

        DMA_Init(TSC_DMA_TX_STREAM, &DMA_InitStructure);
    }
}


/**
  * @brief  Writes a value in a register of the device through I2C.
  * @param  RegisterAddr: The target register address
  * @param  RegisterValue: The target register value to be written 
  * @retval TSC_OK: if all operations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_I2CDMA_WriteDeviceRegister_1B(uint8_t RegisterAddr, uint8_t RegisterValue)
{
    uint8_t reg_value = 0;
    uint32_t time_out = TIMEOUT_MAX;
    

    /* Configure DMA Peripheral. */
    TSC_I2CDMA_init_module(TSC_DMA_TX, (uint8_t*)(&RegisterValue));


    /* Enable the I2C peripheral. */
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on SB Flag. */
    time_out = TIMEOUT_MAX;
    while (I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_SB) == RESET) 
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Transmit the slave address and enable writing operation. */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Transmitter);

    /* Test on ADDR Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_CheckEvent(TSC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Transmit the first address for r/w operations */
    I2C_SendData(TSC_I2C, RegisterAddr);

    /* Test on TXE FLag (data dent) */
    time_out = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_BTF)))  
    {
    if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Enable I2C DMA request. */
    I2C_DMACmd(TSC_I2C, ENABLE);

    /* Enable DMA TX Channel. */
    DMA_Cmd(TSC_DMA_TX_STREAM, ENABLE);

    /* Wait until DMA Transfer Complete. */
    time_out = TIMEOUT_MAX;
    while (!DMA_GetFlagStatus(TSC_DMA_TX_STREAM, TSC_DMA_TX_TCFLAG))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }  

    /* Wait until BTF Flag is set before generating STOP. */
    time_out = 0xFF * TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(TSC_I2C, I2C_FLAG_BTF)))  
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send STOP Condition. */
    I2C_GenerateSTOP(TSC_I2C, ENABLE);

    /* Disable DMA TX Channel */
    DMA_Cmd(TSC_DMA_TX_STREAM, DISABLE);

    /* Disable I2C DMA request */  
    I2C_DMACmd(TSC_I2C, DISABLE);

    /* Clear DMA TX Transfer Complete Flag */
    DMA_ClearFlag(TSC_DMA_TX_STREAM, TSC_DMA_TX_TCFLAG);
  


    /* Verify that the loaded data is correct  */

    /* Read the just written register*/
    TSC_I2C_ReadDeviceRegister_1B(RegisterAddr, &reg_value);

    return (reg_value != RegisterValue) ? TSC_FAILURE : TSC_OK;

TIMEOUT_FLAG:
    
    return TSC_TIMEOUT;  
}


/**
  * @brief  Reads a register of the device through I2C.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval The value of the read register (0xAA if Timeout occurred)   
  */
static TSC_Status_TypDef TSC_I2CDMA_ReadDeviceRegister_1B(uint8_t RegisterAddr, uint8_t *data_ptr)
{
    uint32_t       time_out = TIMEOUT_MAX; 
    uint8_t TSC_BufferRX[2] = {0x00, 0x00};  


    /* Configure DMA Peripheral */
    TSC_I2CDMA_init_module(TSC_DMA_RX, (uint8_t*)TSC_BufferRX);

    /* Enable DMA NACK automatic generation */
    I2C_DMALastTransferCmd(TSC_I2C, ENABLE);


    /* Enable the I2C peripheral */
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on SB Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_SB)) 
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send device address for write. */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Transmitter);

    /* Test on ADDR Flag. */
    time_out = TIMEOUT_MAX;
    while (!I2C_CheckEvent(TSC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send the device's internal address to write to */
    I2C_SendData(TSC_I2C, RegisterAddr);  

    /* Test on TXE FLag (data dent) */
    time_out = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_BTF)))  
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send START condition a second time */  
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on SB Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_SB)) 
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send TSCxpander address for read */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Receiver);

    /* Test on ADDR Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_CheckEvent(TSC_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))   
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Enable I2C DMA request */
    I2C_DMACmd(TSC_I2C,ENABLE);

    /* Enable DMA RX Channel */
    DMA_Cmd(TSC_DMA_RX_STREAM, ENABLE);

    /* Wait until DMA Transfer Complete */
    time_out = 2 * TIMEOUT_MAX;
    while (!DMA_GetFlagStatus(TSC_DMA_RX_STREAM, TSC_DMA_RX_TCFLAG))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }        


    /* Send STOP Condition */
    I2C_GenerateSTOP(TSC_I2C, ENABLE);

    /* Disable DMA RX Channel */
    DMA_Cmd(TSC_DMA_RX_STREAM, DISABLE);

    /* Disable I2C DMA request */  
    I2C_DMACmd(TSC_I2C,DISABLE);

    /* Clear DMA RX Transfer Complete Flag */
    DMA_ClearFlag(TSC_DMA_RX_STREAM,TSC_DMA_RX_TCFLAG);

    *data_ptr = TSC_BufferRX[0];

    return TSC_OK;

TIMEOUT_FLAG:

    return TSC_TIMEOUT; 
}


/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  RegisterAddr: The target register address (between 00x and 0x24)
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).  
  */
static TSC_Status_TypDef TSC_I2CDMA_ReadDeviceRegister_2B(uint8_t RegisterAddr, uint16_t *data_ptr)
{ 
    uint32_t       time_out = TIMEOUT_MAX;
    uint8_t TSC_BufferRX[2] = {0x00, 0x00};  


    /* Configure DMA Peripheral */
    TSC_I2CDMA_init_module(TSC_DMA_RX, (uint8_t*)TSC_BufferRX);

    /* Enable DMA NACK automatic generation */
    I2C_DMALastTransferCmd(TSC_I2C, ENABLE);


    /* Enable the I2C peripheral */
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on SB Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_SB)) 
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send device address for write */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Transmitter);

    /* Test on ADDR Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_CheckEvent(TSC_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send the device's internal address to write to */
    I2C_SendData(TSC_I2C, RegisterAddr);  

    /* Test on TXE FLag (data dent) */
    time_out = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_BTF)))  
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send START condition a second time. */  
    I2C_GenerateSTART(TSC_I2C, ENABLE);

    /* Test on SB Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(TSC_I2C,I2C_FLAG_SB)) 
    {
    if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Send IO Expander address for read */
    I2C_Send7bitAddress(TSC_I2C, TSC_STMPE811_ADDR, I2C_Direction_Receiver);

    /* Test on ADDR Flag */
    time_out = TIMEOUT_MAX;
    while (!I2C_CheckEvent(TSC_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))   
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }


    /* Enable I2C DMA request */
    I2C_DMACmd(TSC_I2C,ENABLE);

    /* Enable DMA RX Channel */
    DMA_Cmd(TSC_DMA_RX_STREAM, ENABLE);

    /* Wait until DMA Transfer Complete */
    time_out = 2 * TIMEOUT_MAX;
    while (!DMA_GetFlagStatus(TSC_DMA_RX_STREAM, TSC_DMA_RX_TCFLAG))
    {
        if (time_out-- == 0) goto TIMEOUT_FLAG;
    }        


    /* Send STOP Condition */
    I2C_GenerateSTOP(TSC_I2C, ENABLE);

    /* Disable DMA RX Channel */
    DMA_Cmd(TSC_DMA_RX_STREAM, DISABLE);

    /* Disable I2C DMA request */  
    I2C_DMACmd(TSC_I2C,DISABLE);

    /* Clear DMA RX Transfer Complete Flag */
    DMA_ClearFlag(TSC_DMA_RX_STREAM,TSC_DMA_RX_TCFLAG);

    *data_ptr = (uint16_t) ((uint16_t)TSC_BufferRX[1] | (uint16_t)TSC_BufferRX[0]<< 8);

    return TSC_OK;

TIMEOUT_FLAG:

    return TSC_TIMEOUT; 

}

#endif




/**
  * @brief  Configures the selected functionalities.
  * @param  Fct: the functions to be configured. could be any combination of the following values:
  *   @arg  TSC_IO_FCT : IO function
  *   @arg  TSC_TP_FCT : Touch Panel function
  *   @arg  TSC_ADC_FCT : ADC function
  * @param  NewState: can be ENABLE pr DISABLE   
  * @retval TSC_OK: if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_cmd_clock_ctrl(uint8_t Fct, FunctionalState NewState)
{
    uint8_t              tmp = 0;
    TSC_Status_TypDef status = TSC_OK;


    /* Get the register value. */
    status = TSC_I2C_ReadDeviceRegister_1B(TSC_REG_SYS_CTRL2, &tmp);

    if(status == TSC_OK)
    {
        if(NewState != DISABLE)
        {
            /* Set the Functionalities to be Enabled. */    
            tmp &= ~(uint8_t)Fct;
        }
        else
        {
            /* Set the Functionalities to be Disabled. */    
            tmp |= (uint8_t)Fct;  
        }

        /* Set the register value. */
        status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_SYS_CTRL2, tmp);
    }

    return status;    
}


/**
  * @brief  Configures the selected pin to be in Alternate function or not.
  * @param  IO_Pin: IO_Pin_x, Where x can be from 0 to 7.   
  * @param  NewState: State of the AF for the selected pin, could be ENABLE or DISABLE.       
  * @retval TSC_OK: if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_cmd_gpio_alt_ctrl(uint8_t IO_Pin, FunctionalState NewState)
{
    uint8_t              tmp = 0;
    TSC_Status_TypDef status = TSC_OK;


    /* Get the current state of the GPIO_AF register. */
    status = TSC_I2C_ReadDeviceRegister_1B(TSC_REG_GPIO_AF, &tmp);

    if(status == TSC_OK)
    {
        if(NewState != DISABLE)
        {
            /* Enable the selected pins alternate function. */
            tmp |= (uint8_t)IO_Pin;
        }
        else
        {
            /* Disable the selected pins alternate function. */   
            tmp &= ~(uint8_t)IO_Pin;   
        }

        /* Write back the new value in GPIO_AF register. */  
        status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_GPIO_AF, tmp);  
    }
    
    return status;
}


/**
  * @brief  Checks the selected Global interrupt source pending bit
  * @param  Global_IT: the Global interrupt source to be checked, could be:
  *   @arg  Global_IT_ADC : ADC interrupt    
  *   @arg  Global_IT_FE : Touch Panel Controller FIFO Error interrupt
  *   @arg  Global_IT_FF : Touch Panel Controller FIFO Full interrupt      
  *   @arg  Global_IT_FOV : Touch Panel Controller FIFO Overrun interrupt     
  *   @arg  Global_IT_FTH : Touch Panel Controller FIFO Threshold interrupt   
  *   @arg  Global_IT_TOUCH : Touch Panel Controller Touch Detected interrupt      
  * @retval Status of the checked flag. Could be SET or RESET.
  */
static FlagStatus TSC_cmd_get_int_status(uint8_t Global_IT)
{
    uint8_t tmp = 0;

 
    /* Get the Interrupt status. */
    TSC_I2C_ReadDeviceRegister_1B(TSC_REG_INT_STA, &tmp);

    if((tmp & Global_IT) != 0)
    {
        return SET;
    }
    else
    {
        return RESET;
    }
}


/**
  * @brief  Clears the selected Global interrupt pending bit(s)
  * @param  Global_IT: the Global interrupt to be cleared, could be any combination
  *         of the following values:   
  *   @arg  Global_IT_ADC : ADC interrupt    
  *   @arg  Global_IT_FE : Touch Panel Controller FIFO Error interrupt
  *   @arg  Global_IT_FF : Touch Panel Controller FIFO Full interrupt      
  *   @arg  Global_IT_FOV : Touch Panel Controller FIFO Overrun interrupt     
  *   @arg  Global_IT_FTH : Touch Panel Controller FIFO Threshold interrupt   
  *   @arg  Global_IT_TOUCH : Touch Panel Controller Touch Detected interrupt 
  * @retval TSC_OK: if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_cmd_clear_int_status(uint8_t Global_IT)
{
    /* Write 1 to the bits that have to be cleared. */
    return TSC_I2C_WriteDeviceRegister_1B(TSC_REG_INT_STA, Global_IT); 
}


/**
  * @brief  Enables or disables the Global interrupt.
  * @param  NewState: could be ENABLE or DISABLE.        
  * @retval TSC_OK: if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_cmd_int_ctrl(FunctionalState NewState)
{
    uint8_t              tmp = 0;
    TSC_Status_TypDef status = TSC_OK;


    /* Read the Interrupt Control register.  */
    status = TSC_I2C_ReadDeviceRegister_1B(TSC_REG_INT_CTRL, &tmp);

    if(status == TSC_OK)
    {
        if(NewState != DISABLE)
        {
            /* Set the global interrupts to be Enabled. */    
            tmp |= (uint8_t)TSC_GIT_EN;
        }
        else
        {
            /* Set the global interrupts to be Disabled. */    
            tmp &= ~(uint8_t)TSC_GIT_EN;
        }  

        /* Write Back the Interrupt Control register. */
        status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_INT_CTRL, tmp);
    }

    return status;     
}


/**
  * @brief  Configures the selected source to generate or not a global interrupt
  * @param Global_IT: the interrupt source to be configured, could be:
  *   @arg  Global_IT_ADC : ADC interrupt     
  *   @arg  Global_IT_FE : Touch Panel Controller FIFO Error interrupt
  *   @arg  Global_IT_FF : Touch Panel Controller FIFO Full interrupt      
  *   @arg  Global_IT_FOV : Touch Panel Controller FIFO Overrun interrupt     
  *   @arg  Global_IT_FTH : Touch Panel Controller FIFO Threshold interrupt   
  *   @arg  Global_IT_TOUCH : Touch Panel Controller Touch Detected interrupt 
  * @param  NewState: can be ENABLE pr DISABLE   
  * @retval TSC_OK: if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_cmd_int_enable(uint8_t Global_IT, FunctionalState NewState)
{
    uint8_t              tmp = 0;
    TSC_Status_TypDef status = TSC_OK;


    /* Get the current value of the INT_EN register. */
    status = TSC_I2C_ReadDeviceRegister_1B(TSC_REG_INT_EN, &tmp);

    if(status == TSC_OK)
    {
        if(NewState != DISABLE)
        {
            /* Set the interrupts to be Enabled. */    
            tmp |= (uint8_t)Global_IT;  
        }
        else
        {
            /* Set the interrupts to be Disabled. */    
            tmp &= ~(uint8_t)Global_IT;
        }
        
        /* Set the register */
        status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_INT_EN, tmp);
    }

    return status;  
}


/**
  * @brief  Resets the chip by Software (SYS_CTRL1, RESET bit).
  * @param  None
  * @retval TSC_OK: if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_cmd_reset_chip(void)
{
    TSC_Status_TypDef status = TSC_OK;

    
    /* Reset the chip. */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_SYS_CTRL1, 0x02);

    if(status == TSC_OK)
    {
        /* wait for a delay to insure registers erasing. */
        TSC_delay(2); 

        /* Cancel the reset process. */
        status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_SYS_CTRL1, 0x00);
    }
    
    return status;    
}


/**
  * @brief  Initializes the GPIO pins used by the touch sensor controller.
  * @param  None
  * @retval None
  */
static void TSC_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    
    /* Enable TSC_I2C and TSC_I2C_GPIO_PORT & Alternate Function clocks. */
    RCC_APB1PeriphClockCmd(TSC_I2C_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(TSC_I2C_SCL_GPIO_CLK | TSC_I2C_SDA_GPIO_CLK | TSC_IT_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Reset TSC_I2C IP and Release reset signal of TSC_I2C IP */
    RCC_APB1PeriphResetCmd(TSC_I2C_CLK, ENABLE);
    RCC_APB1PeriphResetCmd(TSC_I2C_CLK, DISABLE);

    /* Connect PXx to I2C_SCL and I2C_SDA. */
    GPIO_PinAFConfig(TSC_I2C_SCL_GPIO_PORT, TSC_I2C_SCL_SOURCE, TSC_I2C_SCL_AF);
    GPIO_PinAFConfig(TSC_I2C_SDA_GPIO_PORT, TSC_I2C_SDA_SOURCE, TSC_I2C_SDA_AF); 

    /* TSC_I2C SCL and SDA pins configuration */
    GPIO_InitStructure.GPIO_Pin = TSC_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(TSC_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TSC_I2C_SDA_PIN;
    GPIO_Init(TSC_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
}


/**
  * @brief  Configure the I2C Peripheral used to communicate.
  * @param  None
  * @retval None
  */
static void TSC_i2c_init(void)
{
    /* TSC_I2C configuration. */
    I2C_InitTypeDef I2C_InitStructure = 
    { 
        I2C_SPEED, I2C_Mode_I2C, I2C_DutyCycle_2, 0x00, I2C_Ack_Enable, I2C_AcknowledgedAddress_7bit 
    };


    /* Disable the I2C peripheral. */
    I2C_Cmd(TSC_I2C, DISABLE);
    I2C_DeInit(TSC_I2C);

    /* Initialize the I2C peripheral and enable the I2C peripheral. */
    I2C_Init(TSC_I2C, &I2C_InitStructure);
    I2C_Cmd(TSC_I2C, ENABLE); 
}


/**
  * @brief  Enables the touch Panel interrupt.
  * @param  None
  * @retval TSC_OK: if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_intfunc_init(void)
{  
    uint8_t reg_data = 0;
    TSC_Status_TypDef status = TSC_OK;

    
    /* Enable the Global interrupt. */  
    status = TSC_cmd_int_ctrl(ENABLE);     
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }   
    
    /* Enable the Global GPIO Interrupt. */
    status = TSC_cmd_int_enable((uint8_t)(TSC_GIT_TOUCH | TSC_GIT_FTH | TSC_GIT_FOV), ENABLE);    
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }   

    /* Read the GPIO_IT_STA to clear all pending bits if any. */
    TSC_I2C_ReadDeviceRegister_1B(TSC_REG_GPIO_INT_STA, &reg_data); 

RETURN_FLAG:

    return status;
}


/**
  * @brief  Configures the touch Panel Controller (Single point detection)
  * @param  None
  * @retval TSC_OK if all initializations are OK. Other value if error.
  */
static TSC_Status_TypDef TSC_touch_panel_init(void)
{ 
    TSC_Status_TypDef status = TSC_OK;


    /* Enable ADC funcionality. */
    status = TSC_cmd_clock_ctrl(TSC_ADC_FCT, ENABLE);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }
    
    /* Enable touch Panel functionality */
    status = TSC_cmd_clock_ctrl(TSC_TP_FCT, ENABLE);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* Select ADC Sample Time, bit number and ADC Reference. */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_ADC_CTRL1, 0x48);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* Wait for ~20 ms */
    TSC_delay(2);  

    /* Select the ADC clock speed: 3.25 MHz. */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_ADC_CTRL2, 0x01);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* Select TSC pins in non default mode. */  
    status = TSC_cmd_gpio_alt_ctrl((uint8_t)TOUCH_IO_ALL, DISABLE);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* Select 2 nF filter capacitor. */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_TP_CFG, 0x9A);   
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* Select single point reading.  */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_FIFO_TH, 0x01);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* Write 0x01 to clear the FIFO memory content. */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_FIFO_STA, 0x01);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* Write 0x00 to put the FIFO back into operation mode.  */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_FIFO_STA, 0x00);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* set the data format for Z value: 7 fractional part and 1 whole part */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_TP_FRACT_XYZ, 0x01);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* set the driving capability of the device for TSC pins: 50mA */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_TP_I_DRIVE, 0x01);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /* No tracking index, TSC operation mode (XYZ) and enable the TSC. */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_TP_CTRL, 0x03);
    if(status != TSC_OK)
    {
        goto RETURN_FLAG;
    }

    /*  Clear all the status pending bits. */
    status = TSC_I2C_WriteDeviceRegister_1B(TSC_REG_INT_STA, 0xFF); 


RETURN_FLAG:
    
    return status;  
}


/**
  * @brief  Initializes and Configures the Touch Panel Functionality.
  * @param  None
  * @retval TSC_OK if all initializations done correctly. Other value if error.
  */
TSC_Status_TypDef TSC_init(void)
{
    /* Configure the needed pins. */
    TSC_gpio_init(); 

    /* I2C initialization. */
    TSC_i2c_init();

    /* Read IO Expander ID  */
    if(TSC_get_device_status())
    {
        return TSC_NOT_OPERATIONAL;
    }

    /* Generate Software reset. */
    TSC_cmd_reset_chip(); 

    /* Touch Panel controller configuration. */
    TSC_touch_panel_init();

    return TSC_OK; 
}


/**
  * @brief  Checks if the TSC device is correctly configured and 
  *         communicates correctly on the I2C bus.
  * @param  None
  * @retval TSC_OK if TSC is operational. Other value if failure.
  */
TSC_Status_TypDef TSC_get_device_status(void)
{
    uint16_t        reg_data = 0;
    TSC_Status_TypDef status = TSC_OK;


    /* Read device id and check the validation. */
    status = TSC_I2C_ReadDeviceRegister_2B(TSC_REG_CHP_ID, &reg_data);
    if( (status == TSC_OK) && (reg_data == (uint16_t)TSC_STMPE811_ID) )
    {
        status = TSC_FAILURE;
    }
    else
    {
        status = TSC_OK;
    }

    return status;
}


/**
  * @brief  Returns Status and positions of the Touch Panel.
  * @param  Pointer to tsc_state_st structure holding Touch Panel information.
  * @retval Operation status.
  */
TSC_Status_TypDef TSC_get_touch_state(tsc_state_st_ptr state_ptr)
{
    uint8_t      reg_data = 0;
    
    uint16_t        x_cur = 0;
    uint16_t        y_cur = 0;
    static uint16_t x_pre = 0;
    static uint16_t y_pre = 0;

    uint16_t       x_diff = 0;
    uint16_t       y_diff = 0;
    
    TSC_Status_TypDef status = TSC_OK;

    
    /* Check if the Touch detect event happened. */
    status = TSC_I2C_ReadDeviceRegister_1B(TSC_REG_TP_CTRL, &reg_data);
    if(status == TSC_OK)
    {
        state_ptr->Touched = (reg_data & TSC_TOUCHED_YES);

        if(state_ptr->Touched == TSC_TOUCHED_YES) 
        {
            /* Read x value from DATA_X register. */
            status = TSC_I2C_ReadDeviceRegister_2B(TSC_REG_TP_DATA_X, &x_cur);
            if(status != TSC_OK)
            {
                goto RETURN_FLAG;
            }

            /* Read y value from DATA_Y register. */
            status = TSC_I2C_ReadDeviceRegister_2B(TSC_REG_TP_DATA_Y, &y_cur);
            if(status != TSC_OK)
            {
                goto RETURN_FLAG;
            }
            
            /* Upgrade xy previous data based on differce. */
            x_diff = (x_cur > x_pre)? (x_cur - x_pre):(x_pre - x_cur);
            y_diff = (y_cur > y_pre)? (y_cur - y_pre):(y_pre - y_cur);       
            if(4 < (x_diff + y_diff))
            {
                x_pre = x_cur;
                y_pre = y_cur;       
            }
        }
        
        /* Update the X,Y,Z position. */
        state_ptr->X = x_pre; 
        state_ptr->Y = y_pre;
        TSC_I2C_ReadDeviceRegister_1B(TSC_REG_TP_DATA_Z, &(state_ptr->Z));

        /* Clear the interrupt pending bit and enable the FIFO again. */
        TSC_I2C_WriteDeviceRegister_1B(TSC_REG_FIFO_STA, 0x01);
        TSC_I2C_WriteDeviceRegister_1B(TSC_REG_FIFO_STA, 0x00);
    }

RETURN_FLAG:
    
    return status; 
}

