/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : syn6288.c
 @brief  : syn6288  audio driver
 @author : gexueyuan
 @history:
           2015-8-6    gexueyuan    Created file
           ...
******************************************************************************/
#include "cv_osal.h"

#include <rthw.h>
#include <rtthread.h>
    
#include "stm32f4xx.h"
#include "board.h"
#include "usart.h"
#include <rtthread.h>
#include "finsh.h"
#include "components.h"
#include "voc.h"
#include "cv_cms_def.h"

#define SYN6288_DEVICE_NAME	"uart4"

#define SYN6288_MAX_LENGTH  200

#define SYN6288_IDLE  0x4F

#define SYN_CLK   RCC_AHB1Periph_GPIOE
#define SYN_PIN   GPIO_Pin_2
#define SYN_PORT  GPIOE

#define SYN_RB_CLK  RCC_AHB1Periph_GPIOF
#define SYN_RB_PIN  GPIO_Pin_6
#define SYN_RB_PORT  GPIOF

#define SYN_STATE()  GPIO_ReadInputDataBit(SYN_RB_PORT,SYN_RB_PIN)/*0-ready,1-busy*/

typedef enum _BAUDRATE_SYN6288 {
    BAUDRATE_SYN6288_9600 = 0,
    BAUDRATE_SYN6288_19200,
    BAUDRATE_SYN6288_38400,
} E_BAUDRATE_SYN6288;


static rt_device_t syn6288_dev;



void audio_enble(void)
{
    GPIO_SetBits(SYN_PORT, SYN_PIN);
}

void audio_disable(void)
{
    GPIO_ResetBits(SYN_PORT, SYN_PIN);
}

void audio_io_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    
    
    RCC_AHB1PeriphClockCmd(SYN_CLK | SYN_RB_CLK, ENABLE);

    /* Sound en enable*/
    GPIO_InitStructure.GPIO_Pin =  SYN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    
    GPIO_Init(SYN_PORT, &GPIO_InitStructure);

    /*SYN6288 BUSY/READY*/
    GPIO_InitStructure.GPIO_Pin =  SYN_RB_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(SYN_RB_PORT, &GPIO_InitStructure);
    
    audio_disable();//after init ,disable audio output
}




void syn6288_set_baudrate(E_BAUDRATE_SYN6288 baud)
{
    uint8_t baud_char[] = {0xfd,0x00,0x03,0x31,0x00,0xcf};
    
    uint8_t temp;

    baud_char[4] = baud;

    baud_char[5] = baud_char[5] - baud;
    
    temp = rt_device_write(syn6288_dev,0,baud_char,sizeof(baud_char));

    if(temp != sizeof(baud_char))
        rt_kprintf("data of audio miss!!\n");
}


int syn6288_play(char *txt)
{
    char tempbuff[205];
    uint8_t datalength;
    int i = 0;
    int j = 0;
    uint8_t xorcrc=0;
    uint8_t temp;
    RT_ASSERT(txt != NULL);
    
    i = 5;
    
    while(*txt != '\0'){
        
        tempbuff[i++] = *txt++;

    }
    
    datalength = i - 5;

    if(datalength > SYN6288_MAX_LENGTH){
        
        return -1;
        
    }
    
    tempbuff[0] = 0xFD;

    tempbuff[1] = 0x00;

    tempbuff[2] = datalength + 3;
    
    tempbuff[3] = 0x01;
    
    tempbuff[4] = 0x00;

    for(j = 0;j < (datalength + 5);j++){
        xorcrc=xorcrc ^ tempbuff[j];      
    }
    tempbuff[datalength + 5] = xorcrc;

    temp = rt_device_write(syn6288_dev,0,tempbuff,datalength + 6);
    
    if(temp != (datalength + 6))
        rt_kprintf("data of audio miss!!\n");

    return 0;
}


FINSH_FUNCTION_EXPORT(syn6288_play, input:string);


void syn6288_stop(void)
{
    uint8_t stop_char[] = {0xfd,0x00,0x02,0x02,0xfd};
    
    uint8_t temp;
    
    temp = rt_device_write(syn6288_dev,0,stop_char,sizeof(stop_char));

    if(temp != sizeof(stop_char))
        rt_kprintf("data of audio miss!!\n");


}
FINSH_FUNCTION_EXPORT(syn6288_stop, func:stop play audio);


void syn6288_pause(void)
{
    uint8_t pause_char[] = {0xfd,0x00,0x02,0x03,0xfc};
    
    uint8_t temp;
    
    temp = rt_device_write(syn6288_dev,0,pause_char,sizeof(pause_char));

    if(temp != sizeof(pause_char))
        rt_kprintf("data of audio miss!!\n");


}
FINSH_FUNCTION_EXPORT(syn6288_pause, func:pause play audio);

void syn6288_continue(void)
{
    uint8_t continue_char[] = {0xfd,0x00,0x02,0x04,0xfb};
    
    uint8_t temp;
    
    temp = rt_device_write(syn6288_dev,0,continue_char,sizeof(continue_char));

    if(temp != sizeof(continue_char))
        rt_kprintf("data of audio miss!!\n");

}
FINSH_FUNCTION_EXPORT(syn6288_continue, func:continue play audio);


/*****************************************************************************
 @funcname: syn6288_volume
 @brief   : control syn6288 volume
 @param   : uint8_t vol  0~16  0:Mute 16:Max
 @return  : 
*****************************************************************************/
void syn6288_volume(uint8_t vol)
{
    char vol_char[] = "[v1]";//{'[',vol,']'};

    vol_char[2] = vol + 0x30;

    syn6288_play(vol_char);

}
FINSH_FUNCTION_EXPORT(syn6288_volume, func:control volume of audio);

/*****************************************************************************
 @funcname: syn6288_speed
 @brief   : syn6288 play speed
 @param   : uint8_t speed  :0~5  5 is the fastest speed
 @return  : 
*****************************************************************************/
void syn6288_speed(uint8_t speed)
{
    char speed_char[] = "[t0]";//{'[',vol,']'};

    speed_char[2] = speed + 0x30;

    syn6288_play(speed_char);

}
FINSH_FUNCTION_EXPORT(syn6288_speed, func:control speed of audio);



void syn6288_mode(uint8_t mode)
{
    char mode_char[] = "[o0]";

    mode_char[2] = mode + 0x30;

    syn6288_play(mode_char);

}
FINSH_FUNCTION_EXPORT(syn6288_mode, var:0-nature 1-word by word);

uint8_t syn6288_state(void)
{
  return SYN_STATE();

/*
    char state_char[] = {0xfd,0x00,0x02,0x21,0xde};

    uint8_t temp;
    uint8_t state[2];
    
    temp = rt_device_write(syn6288_dev,0,state_char,sizeof(state_char));

    if(temp != sizeof(state_char))
        rt_kprintf("data of audio miss!!\n");

    rt_device_read(syn6288_dev,0,&state,2);
    rt_kprintf("state of syn6288 is %X,%X\n",state[0],state[1]);
    if(2 == rt_device_read(syn6288_dev,0,&state,2)){
        rt_kprintf("state of syn6288 is %X,%X\n",state[0],state[1]);
        //return state;
    }
    else{
        //return 0;
    }
   */     
}

void play_test(void)
{

    uint8_t test[] = {0xFD,0x00,0x0B,0x01,0x00,0xD3,0xEE,0xD2,0xF4,0xCC,0xEC,0xCF,0xC2,0xC1};
    
    uint8_t tmp;
    tmp = rt_device_write(syn6288_dev,0,test,sizeof(test));

    if(tmp == sizeof(test))
        rt_kprintf("play success!!\n");

    rt_kprintf("return %d\n",tmp);
}

FINSH_FUNCTION_EXPORT(play_test, sound test);


void syn6288_set(uint8_t fg_vol,uint8_t bg_vol,uint8_t speed)
{
    char vol_char[] = "[d][v8][m2][t5]";//[d] global default;[v8] foreground vol is 8;

    vol_char[5] = fg_vol + 0x30;

    vol_char[9] = bg_vol + 0x30;
    
    vol_char[13] = speed + 0x30;

    syn6288_play(vol_char);

}

void set_voc(void)
{
    /* load voc param from flash */
	voc_config_t *p_voc_param = NULL;	

    p_voc_param = &cms_param.voc;
    
    syn6288_set(p_voc_param->fg_volume,p_voc_param->bg_volume,p_voc_param->speed);
}


uint8_t syn6288_hw_init(void)
{
	

    audio_io_init();
    
    syn6288_dev = rt_device_find(SYN6288_DEVICE_NAME);
    
    rt_device_open(syn6288_dev,RT_DEVICE_OFLAG_RDWR);
    
    //set_voc();
    
    audio_enble();
    
    return 0;

}

