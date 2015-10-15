/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_drv_led.c
 @brief  : this file include the LED display functions
 @author : wangyifeng
 @history:
           2014-6-30    wangyifeng    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "led"
#include "cv_osal_dbg.h"



#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "led.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
#define LED_PERIOD           MS_TO_TICK(100)

osal_timer_t *timer_blink;
led_param_t led_param;


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

void led_init(void)
{
    STM_EVAL_LEDInit(LED_RED);
    STM_EVAL_LEDInit(LED_BLUE);
    STM_EVAL_LEDInit(LED_GREEN);
    STM_EVAL_LEDOff(LED_RED);
    STM_EVAL_LEDOff(LED_BLUE);
    STM_EVAL_LEDOff(LED_GREEN);

}


void led_on(Led_TypeDef led)
{
    if (led < LEDn){      
        STM_EVAL_LEDOn((Led_TypeDef)led);
    }

}

void led_off(Led_TypeDef led)
{
    if (led < LEDn){
        STM_EVAL_LEDOff((Led_TypeDef)led);
    }
}

void led_blink(Led_TypeDef led)
{
    if (led < LEDn){
        STM_EVAL_LEDBlink((Led_TypeDef)led);
    }


}


void  timer_blink_callback( void *parameter )
{
    led_param_t *led_tmp = (led_param_t *)parameter;
	if(led_tmp->state == LED_BLINK){
	    if(led_tmp->color == YELLOW){

	        STM_EVAL_LEDBlink((Led_TypeDef)LED_GREEN);
	        STM_EVAL_LEDBlink((Led_TypeDef)LED_RED);
	    }    
		else	
	        led_blink((Led_TypeDef)(led_tmp->color));

	}
	else if(led_tmp->state == LED_BREATH){
		
	}
}

void led_proc(Led_Color color, Led_State state,uint8_t period)
{
    led_param.color = color;
    led_param.state = state;
    if((period != 0)&&(led_param.period != period)){
        led_param.period = period;
        osal_timer_change(timer_blink,period);
        }
    switch(color){

        case RED:
            if(state == LED_ON){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_GREEN);
                    led_on((Led_TypeDef)LED_RED);
                }
            else if(state == LED_OFF){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_RED);
                }
            else if(state == LED_BLINK){
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_GREEN);
                    led_off((Led_TypeDef)LED_RED);
                    osal_timer_start(timer_blink);
                }
            break;
        case GREEN:
            if(state == LED_ON){
                
                    rt_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_RED);
                    led_on((Led_TypeDef)LED_GREEN);
                }
            else if(state == LED_OFF){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_GREEN);
                }
            else if(state == LED_BLINK){
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_GREEN);
                    led_off((Led_TypeDef)LED_RED);
                    osal_timer_start(timer_blink);
                }
            break;
        case BLUE:
            if(state == LED_ON){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_RED);
                    led_off((Led_TypeDef)LED_GREEN);
                    led_on((Led_TypeDef)LED_BLUE);                  
                }
            else if(state == LED_OFF){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_BLUE);
                }
            else if(state == LED_BLINK){
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_GREEN);
                    led_off((Led_TypeDef)LED_RED);
                    osal_timer_start(timer_blink);
                }
            break;
        case YELLOW:
            if(state == LED_ON){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_RED);
                    led_on((Led_TypeDef)LED_BLUE);
                    led_on((Led_TypeDef)LED_GREEN);
                }
            else if(state == LED_OFF){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_GREEN);
                }
            else if(state == LED_BLINK){
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_GREEN);
                    led_off((Led_TypeDef)LED_RED);
                    osal_timer_start(timer_blink);
                }
            break;
        case LIGHT:
            if(state == LED_ON){
                    osal_timer_stop(timer_blink);
                    led_on((Led_TypeDef)LED_BLUE);
                    led_on((Led_TypeDef)LED_GREEN);
                    led_on((Led_TypeDef)LED_RED);
                }
            else if(state == LED_OFF){
                    osal_timer_stop(timer_blink);
                    led_off((Led_TypeDef)LED_BLUE);
                    led_off((Led_TypeDef)LED_GREEN);
                    led_off((Led_TypeDef)LED_RED);

                }
            else if(state == LED_BLINK)
                    osal_timer_start(timer_blink);
            break;
        default:
            break;

    }


}


FINSH_FUNCTION_EXPORT(led_proc, led test);

int rt_led_init(void)
{
    led_init();   
    
    timer_blink = osal_timer_create("tm-led",\
        timer_blink_callback,&led_param,LED_PERIOD,RT_TIMER_FLAG_PERIODIC);
    osal_assert(timer_blink != NULL);	
    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "module initial\n\n");   
	return 0;
}


