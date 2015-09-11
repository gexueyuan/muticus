/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bsp_lcd_hx8257.c
 @brief  : This file include the bsp functions for the lcd control module hx8257.
 @author : wangxianwen
 @history:
           2015-5-15    wangxianwen    Created file
           ...
******************************************************************************/

/* Includes -----------------------------------------------------------------*/
#include "bsp_lcd_HX8257.h"


/* Private_FunctionPrototypes. ----------------------------------------------*/
static void LCD_Delay(__IO uint32_t nCount);

static void LCD_gpio_init(void);
static void LCD_afio_init(void);
static void LCD_ltdc_init(void);


/* Private_Functions. -------------------------------------------------------*/

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void LCD_Delay(__IO uint32_t nCount)
{
    __IO uint32_t index = 0; 


    for(index = nCount; index != 0; index--)
    {
        ;
    }
}


/**
  * @brief  Initialize LCD control lines in Output Push-Pull mode.
  * @param  None
  * @retval None
  */
static void LCD_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    
    RCC_AHB1PeriphClockCmd(LCD_DISP_GPIO_CLK, ENABLE);
	
    /* Configure display pin in Output Push-Pull mode. */
    GPIO_InitStructure.GPIO_Pin = LCD_DISP_GPIO_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(LCD_DISP_GPIO_PORT, &GPIO_InitStructure);
		
		/* Set display function. */
    GPIO_WriteBit(LCD_DISP_GPIO_PORT, LCD_DISP_GPIO_PIN, Bit_SET);
}


/**
  * @brief  GPIO alternate functions for LTDC.
  * @param  None
  * @retval None
  */
static void LCD_afio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    
    /* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOF, GPIOG AHB Clocks. */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB  | \
                           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD  | \
                           RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

    /* GPIOs Configuration */
    /*                      LCD pins assignment                                   +
    +------------------------+-----------------------+----------------------------+
    |  LCD_TFT R2 <-> PC.10  |  LCD_TFT G2 <-> PA.06 |  LCD_TFT B2 <-> PD.06      |
    |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11      |
    |  LCD_TFT R4 <-> PA.11  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PG.12      |
    |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03      |
    |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08      |
    |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PD.03 |  LCD_TFT B7 <-> PB.09      |
    -------------------------------------------------------------------------------
    |  LCD_TFT HSYNC <-> PC.06  | LCDTFT VSYNC <->  PA.04                         |                        
    |  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10                         |
    -------------------------------------------------------------------------------  */

    /* GPIOA configuration. */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_LTDC);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | \
                              GPIO_Pin_11 | GPIO_Pin_12;
                          
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIOB configuration. */  
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, 0x09);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, 0x09);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_LTDC);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  | GPIO_Pin_8  
                             | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;

    GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* GPIOC configuration. */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_LTDC);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10;
                             
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* GPIOD configuration. */
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_LTDC);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_6;
                             
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* GPIOF configuration. */
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource10, GPIO_AF_LTDC);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
                             
    GPIO_Init(GPIOF, &GPIO_InitStruct);     

    /* GPIOG configuration. */  
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource6, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource7, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource10, 0x09);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource11, GPIO_AF_LTDC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, 0x09);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | \
                              GPIO_Pin_11 | GPIO_Pin_12;

    GPIO_Init(GPIOG, &GPIO_InitStruct); 
}


/**
  * @brief  Initialize LTDC module in MCU for lcd control.
  * @param  None
  * @retval None
  */
static void LCD_ltdc_init(void)
{
    LTDC_InitTypeDef LTDC_InitStruct = { 0 };   
    
    
    /* Enable the LTDC Clock. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_LTDC, ENABLE);
 
    
    /* Polarity configuration */
    /* Initialize the horizontal synchronization polarity as active low. */
    LTDC_InitStruct.LTDC_HSPolarity = LTDC_HSPolarity_AH;     
    /* Initialize the vertical synchronization polarity as active low. */  
    LTDC_InitStruct.LTDC_VSPolarity = LTDC_VSPolarity_AH;     
    /* Initialize the data enable polarity as active low */
    LTDC_InitStruct.LTDC_DEPolarity = LTDC_DEPolarity_AL;     
    /* Initialize the pixel clock polarity as input pixel clock */ 
    LTDC_InitStruct.LTDC_PCPolarity = LTDC_PCPolarity_IPC;

    /* Configure R,G,B component values for LCD background color */                   
    LTDC_InitStruct.LTDC_BackgroundRedValue = 0;            
    LTDC_InitStruct.LTDC_BackgroundGreenValue = 0;          
    LTDC_InitStruct.LTDC_BackgroundBlueValue = 0;  


    /* Configure PLLSAI prescalers for LCD. */
  
    /* Enable Pixel Clock */
    /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
    /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAI_N = 192 Mhz */
    /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAI_R = 192/4 = 48 Mhz */
    /* LTDC clock frequency = PLLLCDCLK / RCC_PLLSAIDivR = 48/8 = 6 Mhz */
    RCC_PLLSAIConfig(192, 7, 4);
    RCC_LTDCCLKDivConfig(RCC_PLLSAIDivR_Div8);

    /* Enable PLLSAI Clock */
    RCC_PLLSAICmd(ENABLE);
    /* Wait for PLLSAI activation */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLSAIRDY) == RESET)
    {
    }

    /* Timing configuration. */  
    /* Configure horizontal synchronization width. */     
    LTDC_InitStruct.LTDC_HorizontalSync = 3;
   
	/* Configure accumulated horizontal back porch. */
    LTDC_InitStruct.LTDC_AccumulatedHBP = 43; 
    
	/* Configure accumulated active width. */  
    LTDC_InitStruct.LTDC_AccumulatedActiveW = LCD_PIXEL_WIDTH + LTDC_InitStruct.LTDC_AccumulatedHBP;
		
	/* Configure total width */
    LTDC_InitStruct.LTDC_TotalWidth = LTDC_InitStruct.LTDC_AccumulatedActiveW + 8; 
		
	
    /* Configure vertical synchronization height. */
    LTDC_InitStruct.LTDC_VerticalSync = 10;
    
    /* Configure accumulated vertical back porch. */
    LTDC_InitStruct.LTDC_AccumulatedVBP = 37; 
      
    /* Configure accumulated active height. */
    LTDC_InitStruct.LTDC_AccumulatedActiveH = LCD_PIXEL_HEIGHT + LTDC_InitStruct.LTDC_AccumulatedVBP;
    
    /* Configure total height */
    LTDC_InitStruct.LTDC_TotalHeigh = LTDC_InitStruct.LTDC_AccumulatedActiveH + 4;


    LTDC_Init(&LTDC_InitStruct);
}


/**
  * @brief  Initialize DMA2D module in MCU for lcd control.
  * @param  None
  * @retval None
  */
static void LCD_dma2d_init(void)
{
    /* Enable the DMA2D Clock. */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2D, ENABLE); 
}


/**
  * @brief  Initializes the LCD LTDC Layers.
  * @param  None
  * @retval None
  */
void LCD_ltdc_layer_init(void)
{
    LTDC_Layer_InitTypeDef LTDC_Layer_InitStruct = { 0 }; 


    /* Windowing configuration. */
    /* In this case all the active display area is used to display a picture then :
    Horizontal start = horizontal synchronization + Horizontal back porch = 30 
    Horizontal stop = Horizontal start + window width -1 = 30 + 240 -1
    Vertical start   = vertical synchronization + vertical back porch     = 4
    Vertical stop   = Vertical start + window height -1  = 4 + 320 -1      */      
    LTDC_Layer_InitStruct.LTDC_HorizontalStart = 44;
    LTDC_Layer_InitStruct.LTDC_HorizontalStop = (LCD_PIXEL_WIDTH + 44 - 1); 
    LTDC_Layer_InitStruct.LTDC_VerticalStart = 38;
    LTDC_Layer_InitStruct.LTDC_VerticalStop = (LCD_PIXEL_HEIGHT + 38 - 1);

    /* Pixel Format configuration. */
    LTDC_Layer_InitStruct.LTDC_PixelFormat = LTDC_Pixelformat_RGB565;
    
    /* Alpha constant (255 totally opaque). */
    LTDC_Layer_InitStruct.LTDC_ConstantAlpha = 255; 
    
    /* Default Color configuration (configure A,R,G,B component values). */          
    LTDC_Layer_InitStruct.LTDC_DefaultColorBlue = 0;        
    LTDC_Layer_InitStruct.LTDC_DefaultColorGreen = 0;       
    LTDC_Layer_InitStruct.LTDC_DefaultColorRed = 0;         
    LTDC_Layer_InitStruct.LTDC_DefaultColorAlpha = 0;
   
    /* Configure blending factors. */       
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_CA;    
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_CA;

    /* the length of one line of pixels in bytes + 3 then :
       Line Lenth = Active high width x number of bytes per pixel + 3 
       Active high width         = LCD_PIXEL_WIDTH 
       number of bytes per pixel = 2    (pixel_format : RGB565) */
    LTDC_Layer_InitStruct.LTDC_CFBLineLength = ((LCD_PIXEL_WIDTH * 2) + 3);
    
    /* the pitch is the increment from the start of one line of pixels to the 
       start of the next line in bytes, then :
       Pitch = Active high width x number of bytes per pixel */ 
    LTDC_Layer_InitStruct.LTDC_CFBPitch = (LCD_PIXEL_WIDTH * 2);

    /* Configure the number of lines. */  
    LTDC_Layer_InitStruct.LTDC_CFBLineNumber = LCD_PIXEL_HEIGHT;

    /* Start Address configuration : the LCD Frame buffer is defined on SDRAM. */    
    LTDC_Layer_InitStruct.LTDC_CFBStartAdress = LCD_FRAME_BUFFER;

    /* Initialize LTDC layer 1. */
    LTDC_LayerInit(LTDC_Layer1, &LTDC_Layer_InitStruct);


    /* Configure Layer2.--------------------------------------------------------------*/
    /* Start Address configuration : the LCD Frame buffer is defined on SDRAM w/ Offset. */     
    LTDC_Layer_InitStruct.LTDC_CFBStartAdress = LCD_FRAME_BUFFER + LCD_BUFFER_OFFSET;

    /* Configure blending factors. */       
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_1 = LTDC_BlendingFactor1_PAxCA;    
    LTDC_Layer_InitStruct.LTDC_BlendingFactor_2 = LTDC_BlendingFactor2_PAxCA;

    /* Initialize LTDC layer 2. */
    LTDC_LayerInit(LTDC_Layer2, &LTDC_Layer_InitStruct);


    /* LTDC configuration reload. */  
    LTDC_ReloadConfig(LTDC_IMReload);

    /* Enable foreground & background Layers. */
    LTDC_LayerCmd(LTDC_Layer1, ENABLE); 
    LTDC_LayerCmd(LTDC_Layer2, ENABLE);

    /* LTDC configuration reload. */  
    LTDC_ReloadConfig(LTDC_IMReload);

    /* dithering activation. */
    LTDC_DitherCmd(ENABLE);
}


/**
  * @brief  Main routine for initializing the LCD.
  * @param  None
  * @retval None
  */
void LCD_init(void)
{ 
    /* Initialize the LCD GPIO Control pins. */
    LCD_gpio_init();
    
    /* Initialize the LCD alternate functions GPIO pins. */
    LCD_afio_init(); 
    
    /* Initialize the LTDC Module. */
    LCD_ltdc_init();

    /* Initialize the DMA2D Module. */
    LCD_dma2d_init();
    
    /* Initialize the LTDC Layer Module. */
    LCD_ltdc_layer_init();
    
    /* Enable the LTDC */
    LTDC_Cmd(ENABLE);
}


/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void LCD_display_on(void)
{
   
}


/**
  * @brief  Disable the Display.
  * @param  None
  * @retval None
  */
void LCD_display_off(void)
{
   
}
