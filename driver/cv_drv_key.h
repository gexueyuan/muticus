#ifndef __KEY_H
#define __KEY_H 

#include <board.h>


#ifdef HARDWARE_MODULE_WIFI_V1

#define KEY0_PIN                GPIO_Pin_9  
#define KEY0_GPIO_PORT          GPIOB      
#define KEY0_GPIO_CLK           RCC_AHB1Periph_GPIOB


#define KEY1_PIN                GPIO_Pin_9  
#define KEY1_GPIO_PORT          GPIOC      
#define KEY1_GPIO_CLK           RCC_AHB1Periph_GPIOC

#define KEY2_PIN                GPIO_Pin_2  
#define KEY2_GPIO_PORT          GPIOC      
#define KEY2_GPIO_CLK           RCC_AHB1Periph_GPIOC

#define key_up_GETVALUE()       GPIO_ReadInputDataBit(KEY0_GPIO_PORT, KEY0_PIN)
#define key_down_GETVALUE()     GPIO_ReadInputDataBit(KEY1_GPIO_PORT, KEY1_PIN)
#define key_third_GETVALUE()    GPIO_ReadInputDataBit(KEY2_GPIO_PORT, KEY2_PIN)


#elif defined (HARDWARE_MODULE_WIFI_V2)

#define KEY0_PIN                GPIO_Pin_7  
#define KEY0_GPIO_PORT          GPIOC      
#define KEY0_GPIO_CLK           RCC_AHB1Periph_GPIOC

#define key_up_GETVALUE()       GPIO_ReadInputDataBit(KEY0_GPIO_PORT, KEY0_PIN)
#define key_down_GETVALUE()     1
#define key_third_GETVALUE()    1 

#elif defined (HARDWARE_MODULE_WIFI_V3)
#define KEY0_PIN                GPIO_Pin_4  
#define KEY0_GPIO_PORT          GPIOD      
#define KEY0_GPIO_CLK           RCC_AHB1Periph_GPIOD

#define key_up_GETVALUE()       GPIO_ReadInputDataBit(KEY0_GPIO_PORT, KEY0_PIN)
#define key_down_GETVALUE()     1
#define key_third_GETVALUE()    1 

#else

#error "HARDWARE version  not  define!!"

#endif



//常规按键处理使用
#define DEBOUNCE_SHORT_TIME 	3   // 轻触按键消抖时间5（单位：20毫秒）
#define DEBOUNCE_LONG_TIME  	5  // 长按键时间DEBOUNCE_COUT_FIRST+DEBOUNCE_COUT_INTERVAL*DEBOUNCE_LONG_TIME（单位：10毫秒）
#define DEBOUNCE_COUT_FIRST 	500//50 // 连按键间隔时间100（单位：20毫秒）
#define DEBOUNCE_COUT_INTERVAL 	10  // 连按键间隔时间50（单位：20毫秒）

//特殊按键处理使用
#define C_RELASE_COUT			3
#define C_SHORT_COUT			3	//3*20ms
#define C_SPECIAL_LONG_COUT		60  //60*20ms

//按键标志
#define C_FLAG_SHORT			0x00000001
#define C_FLAG_COUNT			0x00000002
#define C_FLAG_LONG				0x00000004
#define C_FLAG_RELASE			0x00000008

//按键键值
#define  C_UP_KEY 				0x1
#define  C_DOWN_KEY 			0x2
#define  C_LEFT_KEY 			0x4
#define  C_RIGHT_KEY 			0x8
#define  C_STOP_KEY 			0x10
#define  C_MENU_KEY 			0x20
#define  C_ENTER_KEY 			0x40
#define  C_SPECIAL_KEY 			C_ENTER_KEY



/*enter键长按 我们定义为home键*/
#define  C_HOME_KEY 			C_SPECIAL_KEY+0x44




extern int32_t vam_active_alert(uint16_t alert);
extern int32_t vam_cancel_alert(uint16_t alert);

/* 使用面向对象的方式，将按键所用到的所有元素进行打包 */
struct rtgui_key
{
    rt_timer_t poll_timer;
	
	rt_uint32_t key_last;
	rt_uint32_t key_current;
	//检测到的按键值
	rt_uint32_t key_get;
	//常规按键使用
	rt_uint32_t key_debounce_count;
	rt_uint32_t key_long_count;
	//特殊按键使用
	rt_uint32_t key_special_count;	
	rt_uint32_t key_relase_count;
	//按键标志
	rt_uint32_t key_flag;
	
};

#endif
