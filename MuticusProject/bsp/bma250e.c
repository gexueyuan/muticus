/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bma250e.c
 @brief  : bma250e chip spi driver
 @author : wanglei
 @history:
           2014-8-11    wanglei       Created file
           ...
******************************************************************************/

#include "bma250e.h"
#include "gsensor.h"

/** @defgroup STM32F401_DISCOVERY_BMA250E_Private_Variables
  * @{
  */
__IO uint32_t  BMA250ETimeout = BMA250E_FLAG_TIMEOUT;
/**
  * @}
  */

/** @defgroup STM32F401_DISCOVERY_BMA250E_Private_FunctionPrototypes
  * @{
  */
static uint8_t BMA250E_SendByte(uint8_t byte);



/**
  * @brief  Writes one byte to the BMA250E.
  * @param  pBuffer : pointer to the buffer  containing the data to be written to the BMA250E.
  * @param  WriteAddr : BMA250E's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  * @retval None
  */
void BMA250E_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit:
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  BMA250E_CS_LOW();

  /* Send the Address of the indexed register */
  BMA250E_SendByte(WriteAddr);
  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    BMA250E_SendByte(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  BMA250E_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the BMA250E.
  * @param  pBuffer : pointer to the buffer that receives the data read from the BMA250E.
  * @param  ReadAddr : BMA250E's internal address to read from.
  * @param  NumByteToRead : number of bytes to read from the BMA250E.
  * @retval None
  */
void BMA250E_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  if(NumByteToRead > 0x01)
  {
    ReadAddr |= (uint8_t)(READWRITE_CMD | MULTIPLEBYTE_CMD);
  }
  else
  {
    ReadAddr |= (uint8_t)READWRITE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  BMA250E_CS_LOW();

  /* Send the Address of the indexed register */
  BMA250E_SendByte(ReadAddr);

  /* Receive the data that will be read from the device (MSB First) */
  while(NumByteToRead > 0x00)
  {
    /* Send dummy byte (0x00) to generate the SPI clock to BMA250E (Slave device) */
    *pBuffer = BMA250E_SendByte(0x00);
    NumByteToRead--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  BMA250E_CS_HIGH();
}
/**
  * @brief  Initializes the low level interface used to drive the BMA250E
  * @param  None
  * @retval None
  */
void BMA250E_LowLevel_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;

  /* Enable the SPI periph */
#ifdef HARDWARE_MODULE_WIFI_V1
  RCC_APB1PeriphClockCmd(BMA250E_SPI_CLK, ENABLE);
#else
  RCC_APB2PeriphClockCmd(BMA250E_SPI_CLK, ENABLE);
#endif

  /* Enable SCK, MOSI and MISO GPIO clocks */
  RCC_AHB1PeriphClockCmd(BMA250E_SPI_SCK_GPIO_CLK | BMA250E_SPI_MISO_GPIO_CLK | BMA250E_SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable CS  GPIO clock */
  RCC_AHB1PeriphClockCmd(BMA250E_SPI_CS_GPIO_CLK, ENABLE);

  /* Enable INT1 GPIO clock */
  RCC_AHB1PeriphClockCmd(BMA250E_SPI_INT1_GPIO_CLK, ENABLE);

  /* Enable INT2 GPIO clock */
  RCC_AHB1PeriphClockCmd(BMA250E_SPI_INT2_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(BMA250E_SPI_SCK_GPIO_PORT, BMA250E_SPI_SCK_SOURCE, BMA250E_SPI_SCK_AF);
  GPIO_PinAFConfig(BMA250E_SPI_MISO_GPIO_PORT, BMA250E_SPI_MISO_SOURCE, BMA250E_SPI_MISO_AF);
  GPIO_PinAFConfig(BMA250E_SPI_MOSI_GPIO_PORT, BMA250E_SPI_MOSI_SOURCE, BMA250E_SPI_MOSI_AF);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; //GPIO_PuPd_NOPULL;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = BMA250E_SPI_SCK_PIN;
  GPIO_Init(BMA250E_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* SPI  MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  BMA250E_SPI_MOSI_PIN;
  GPIO_Init(BMA250E_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

  /* SPI MISO pin configuration */
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //wanglei add
  GPIO_InitStructure.GPIO_Pin = BMA250E_SPI_MISO_PIN;
  GPIO_Init(BMA250E_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(BMA250E_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(BMA250E_SPI, &SPI_InitStructure);

  /* Enable SPI2  */
  SPI_Cmd(BMA250E_SPI, ENABLE);

  /* Configure GPIO PIN for Lis Chip select */
  GPIO_InitStructure.GPIO_Pin = BMA250E_SPI_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BMA250E_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Deselect : Chip Select high */
  GPIO_SetBits(BMA250E_SPI_CS_GPIO_PORT, BMA250E_SPI_CS_PIN);

  /* Configure GPIO PINs to detect Interrupts */
  GPIO_InitStructure.GPIO_Pin = BMA250E_SPI_INT1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(BMA250E_SPI_INT1_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BMA250E_SPI_INT2_PIN;
  GPIO_Init(BMA250E_SPI_INT2_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */
static uint8_t BMA250E_SendByte(uint8_t byte)
{

   
  /* Loop while DR register in not empty */
  BMA250ETimeout = BMA250E_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_TXE) == RESET)
  {
#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
    if((BMA250ETimeout--) == 0) return BMA250E_TIMEOUT_UserCallback();
#endif
  }

  /* Send a Byte through the SPI peripheral */
  SPI_I2S_SendData(BMA250E_SPI, (uint16_t)byte);

  /* Wait to receive a Byte */
  BMA250ETimeout = BMA250E_FLAG_TIMEOUT;
  while (SPI_I2S_GetFlagStatus(BMA250E_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
    if((BMA250ETimeout--) == 0) return BMA250E_TIMEOUT_UserCallback();
#endif
  }

  /* Return the Byte read from the SPI bus */
  return (uint8_t)SPI_I2S_ReceiveData(BMA250E_SPI);

}

#ifdef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t BMA250E_TIMEOUT_UserCallback(void)
{
  /* Block communication and all processes */
  while (1)
  {
  }
}
#endif /* USE_DEFAULT_TIMEOUT_CALLBACK */


void gsnr_int_config(FunctionalState state)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Connect EXTI Line to int1 int2 Pin */
    EXTI_InitTypeDef EXTI_InitStructure;

    SYSCFG_EXTILineConfig(BMA250E_SPI_INT1_EXTI_PORT_SOURCE, BMA250E_SPI_INT1_EXTI_PIN_SOURCE);
    SYSCFG_EXTILineConfig(BMA250E_SPI_INT2_EXTI_PORT_SOURCE, BMA250E_SPI_INT2_EXTI_PIN_SOURCE);

    EXTI_InitStructure.EXTI_Line = BMA250E_SPI_INT1_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    if(state == ENABLE)
    	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    else
    	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = BMA250E_SPI_INT2_EXTI_LINE;
    EXTI_Init(&EXTI_InitStructure);


    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 

    /* The pal timer channel interrupt in NVIC is enabled. */
    NVIC_EnableIRQ(EXTI1_IRQn); 
    NVIC_EnableIRQ(EXTI2_IRQn);

    NVIC_ClearPendingIRQ((IRQn_Type)EXTI1_IRQn);
    NVIC_ClearPendingIRQ((IRQn_Type)EXTI2_IRQn);
    
}

void EXTI1_IRQHandler(void)
{
    /* disable interrupt */
    //EXTI->IMR &= ~GPIO_Pin_1;

    if(EXTI_GetITStatus(EXTI_Line1) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

void EXTI2_IRQHandler(void)
{
    /* disable interrupt */
    //EXTI->IMR &= ~GPIO_Pin_2;

    if(EXTI_GetITStatus(EXTI_Line2) == SET)
    {
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
    
}

void gsnr_write_reg(uint8_t reg, uint8_t data)
{
    BMA250E_Write(&data, reg, 1);
}

void gsnr_read_reg(uint8_t reg, uint8_t *data)
{
    BMA250E_Read(data, reg, 1);
}

/* read x/y/z axis  acc data, and convert unit to m/s2  */
void gsnr_get_acc(float *pdata)
{
    uint16_t temp;
    uint8_t buffer[6] = {0};
    uint8_t i = 0;

    for(i=0; i<6; i++)
    {
        BMA250E_Read(&buffer[i], BMA250E_OUT_X_L_ADDR+i, 1);
    }
    
    for(i=0; i<3; i++)
    {
        temp = (((uint16_t)buffer[2*i+1] << 2) & 1020) | ((buffer[2*i]>>6) & 3);
    	if((temp>>9) == 1)
    	{
            /* 2G FULLSCALE uint:3.91mg 
              BMA250E_Sensitivity_2g = 3.91/1000 * 9.80665. Unit 3.91mg -> m/s2  */
    		pdata[i] = (0.0 - (0x1FF-(temp&0x1FF))) * BMA250E_Sensitivity_2g;
    	}
    	else
    	{
    		 pdata[i] = (temp&0x1FF) * BMA250E_Sensitivity_2g;
    	}
    }
}

void gsnr_drv_init()
{
  /* Configure the low level interface ---------------------------------------*/
    BMA250E_LowLevel_Init();
    /* Configure MEMS: data rate, power mode, full scale and axes */
    gsnr_write_reg(0x34, 0x00);       //config spi 4 wire mode
    gsnr_write_reg(0x14, 0xB6);       //software reset
    gsnr_write_reg(0x0F, 0x03);       //set full scale 2g range
    gsnr_write_reg(0x10, 0x0C);       //Selection of data filter bandwidth: 125Hz

    gsnr_write_reg(0x24, 0xC3);       //high_hy: 3, low_mode: single-axis mode, low_hy: 3
    gsnr_write_reg(0x27, 0x03);       //set slope_dur = 3
    gsnr_write_reg(0x21, 0x00);       //Interrupt mode: non-latched
    gsnr_write_reg(0x28, 0x60);       //slope_th: the threshold definition for the any-motion interrupt: 0x08
    gsnr_write_reg(0x19, 0x04);       //map slope interrupt to INT1 pin
    gsnr_write_reg(0x1B, 0x04);       //map slope interrupt to INT2 pin
    gsnr_write_reg(0x16, 0x07);       //enabled slope_en_z, slope_en_y, slope_en_x 

    //gsnr_int_config(ENABLE);
}

