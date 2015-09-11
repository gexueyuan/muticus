#ifndef __LED_H
#define __LED_H 

#include "board.h"

typedef enum 
{
  LED0 = 0,
  LED1 = 1,
  LED2 = 2,
  LED3 = 3,
}Led_TypeDef;


typedef enum 
{
	LED_ON = 0,
	LED_OFF,
	LED_BLINK,
	LED_BREATH,
}Led_State;

typedef enum 
{
    BLUE = 0,
    GREEN,
    RED,
    YELLOW,
    LIGHT,
}Led_Color;

typedef struct
{
	Led_Color color;
	Led_State state;
	int16_t   period;
}led_param_t;

#define LEDn  3
#ifdef HARDWARE_MODULE_WIFI_V1
#define LED_RED     LED0
#define LED_GREEN   LED1
#define LED_BLUE    LED2
#elif defined(HARDWARE_MODULE_WIFI_V2)
#define LED_RED     LED2
#define LED_GREEN   LED0
#define LED_BLUE    LED1
#elif defined(HARDWARE_MODULE_WIFI_V3)
#define LED_RED     LED1
#define LED_GREEN   LED2
#define LED_BLUE    LED0

#endif




#ifdef HARDWARE_MODULE_WIFI_V1

#define LED0_PIN                         GPIO_Pin_13
#define LED0_GPIO_PORT                   GPIOC
#define LED0_GPIO_CLK                    RCC_AHB1Periph_GPIOC  

#define LED1_PIN                         GPIO_Pin_3
#define LED1_GPIO_PORT                   GPIOC
#define LED1_GPIO_CLK                    RCC_AHB1Periph_GPIOC 

#define LED2_PIN                         GPIO_Pin_11
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK                    RCC_AHB1Periph_GPIOC

#elif defined (HARDWARE_MODULE_WIFI_V2)

#define LED0_PIN                         GPIO_Pin_8
#define LED0_GPIO_PORT                   GPIOA
#define LED0_GPIO_CLK                    RCC_AHB1Periph_GPIOA  

#define LED1_PIN                         GPIO_Pin_8
#define LED1_GPIO_PORT                   GPIOC
#define LED1_GPIO_CLK                    RCC_AHB1Periph_GPIOC 

#define LED2_PIN                         GPIO_Pin_9
#define LED2_GPIO_PORT                   GPIOC
#define LED2_GPIO_CLK                    RCC_AHB1Periph_GPIOC

#elif defined (HARDWARE_MODULE_WIFI_V3)

#define LED0_PIN                         GPIO_Pin_5
#define LED0_GPIO_PORT                   GPIOC
#define LED0_GPIO_CLK                    RCC_AHB1Periph_GPIOC  

#define LED1_PIN                         GPIO_Pin_3//null
#define LED1_GPIO_PORT                   GPIOE
#define LED1_GPIO_CLK                    RCC_AHB1Periph_GPIOE 

#define LED2_PIN                         GPIO_Pin_3//null
#define LED2_GPIO_PORT                   GPIOE
#define LED2_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#else

#error "HARDWARE version  not  define!!"
#endif

void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDBlink(Led_TypeDef Led);

int rt_led_init(void);
void led_proc(Led_Color color, Led_State state,uint8_t freq);
void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);




#endif
