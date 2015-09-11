/*
 * File      : board.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2009 RT-Thread Develop Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 */

#include <rthw.h>
#include <rtthread.h>

#include "stm32f4xx.h"
#include "board.h"
#include "led.h"
/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/*******************************************************************************
 * Function Name  : SysTick_Configuration
 * Description    : Configures the SysTick for OS tick.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void  SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	rt_uint32_t         cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (rt_uint32_t)rcc_clocks.HCLK_Frequency / RT_TICK_PER_SECOND;
	cnts = cnts / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

#if 1

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
	/* NVIC Configuration */
	NVIC_Configuration();
    
	/* Configure the SysTick */
	SysTick_Configuration();

    /* ע�⣺�򿪸ú�����SysTick_Handle�жϾͿ��Խ����ˣ�Ŀǰԭ���������ڹرյĴ����ļ������Ǻ����������ֹ�ע�� */
	rt_hw_usart_init();
    
#ifdef RT_USING_CONSOLE
	rt_console_set_device(CONSOLE_DEVICE);
#endif
    
}

#else

/**
 * This function will initial STM32 board.
 */
void rt_hw_board_init()
{
    /* NVIC Configuration */
    NVIC_Configuration();

    /* Configure the SysTick */
//    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    
    rt_hw_usart_init();
    
    
#ifdef RT_USING_CONSOLE
    rt_console_set_device(CONSOLE_DEVICE);
#endif

//    fsmc_gpio_init();

//    mco_config();
}

#endif


/**
 * This fucntion returns milliseconds since system passed
 */
rt_uint32_t rt_hw_tick_get_millisecond(void)
{
    rt_tick_t tick;
    rt_uint32_t value;

#define TICK_MS (1000/RT_TICK_PER_SECOND)

    tick = rt_tick_get();
    value = tick * TICK_MS + (SysTick->LOAD - SysTick->VAL) * TICK_MS / SysTick->LOAD;

    return value;
}

/**
 * This fucntion returns microseconds since system passed
 */
rt_uint32_t rt_hw_tick_get_microsecond(void)
{
    rt_tick_t tick;
    rt_uint32_t value;

#define TICK_US	(1000000/RT_TICK_PER_SECOND)

    tick = rt_tick_get();
    value = tick * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;

    return value;
}

static volatile int toggle_flag;
void led_Toggle(void)
{
	toggle_flag++;
	if (toggle_flag&0x0001)
		GPIO_ResetBits(GPIOD, GPIO_Pin_3);
	else
		GPIO_SetBits(GPIOD, GPIO_Pin_3);		
}

 void mco_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* MCO configure */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
    RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
}




#ifdef RT_USING_FINSH
#include <finsh.h>
static void reset(void)
{
    NVIC_SystemReset();
}
FINSH_FUNCTION_EXPORT(reset, reset);
#endif /* RT_USING_FINSH */

/*@}*/
