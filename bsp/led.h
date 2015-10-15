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

#define LED_RED     LED1
#define LED_GREEN   LED2
#define LED_BLUE    LED0


#define LED0_PIN                         GPIO_Pin_5
#define LED0_GPIO_PORT                   GPIOC
#define LED0_GPIO_CLK                    RCC_AHB1Periph_GPIOC  

#define LED1_PIN                         GPIO_Pin_3//null
#define LED1_GPIO_PORT                   GPIOE
#define LED1_GPIO_CLK                    RCC_AHB1Periph_GPIOE 

#define LED2_PIN                         GPIO_Pin_3//null
#define LED2_GPIO_PORT                   GPIOE
#define LED2_GPIO_CLK                    RCC_AHB1Periph_GPIOE


void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDBlink(Led_TypeDef Led);

int rt_led_init(void);
void led_proc(Led_Color color, Led_State state,uint8_t freq);
void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);




#endif
