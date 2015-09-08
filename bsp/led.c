
#include "led.h"
#include <stm32f4xx.h>

void STM_EVAL_LEDInit(Led_TypeDef Led);

uint32_t L3GD20_TIMEOUT_UserCallback(void)
{
  return 0;
}

GPIO_TypeDef*  GPIO_PORT[LEDn] = {LED0_GPIO_PORT, LED1_GPIO_PORT, LED2_GPIO_PORT,
                               };
const uint16_t GPIO_PIN[LEDn] = {LED0_PIN, LED1_PIN, LED2_PIN,
                                 };
const uint32_t GPIO_CLK[LEDn] = {LED0_GPIO_CLK, LED1_GPIO_CLK, LED2_GPIO_CLK,
                                 };



void STM_EVAL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

void STM_EVAL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRL = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6 
  * @retval None
  */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRH = GPIO_PIN[Led];  
}


/**
  * @brief  Switch selected LED's status.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6 
  * @retval None
  */
void STM_EVAL_LEDBlink(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];  
}

