/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_manager.c
 @brief  : this file include the system manage functions
 @author : wangyifeng
 @history:
           2014-6-20    wangyifeng    Created file
           2015-9-10    gexueyuan    Modified file for syn6288
           ...
******************************************************************************/
#include "cv_osal.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

#include "voc.h"



#define SOUND_PLAY_INTERVAL MS_TO_TICK(50)
#define BIBI_PRE_COUNT 1
#define MAX_VOICE  6  /* MUST BE such value as 2,4,8,16... */
static uint8_t voice[MAX_VOICE];
static uint32_t voice_wr_idx;
static uint32_t voice_rd_idx;
static uint32_t phase = 0;

#define NOTICE_STRING  "soundb"
#define CFCW_VOC "[3]请注意[2]前车[2]距离" //"北京[3]东直门站到了"//
#define CRCW_VOC "后方超车"
#define EEBL_VOC "前方急刹"
#define VBD_VOC "[2]前方有故障车"  

#define FRONT "前方"
#define REAR "后方"
#define EEBL "急刹"
#define VBD  "故障"
#define CFCW "车距"
#define MI "米"

#define AMBU "救护车"
#define GIVEWAY "请避让"
#define CURVE "弯道"
char  play_string[50];
uint32_t voc_distance;


char* itoa(int num,char*str,int radix)
{
    /*索引表*/
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    unsigned unum;/*中间变量*/
    int i=0,j,k;
    /*确定unum的值*/
    if(radix==10&&num<0)/*十进制负数*/
    {
    unum=(unsigned)-num;
    str[i++]='-';
    }
    else unum=(unsigned)num;/*其他情况*/
    /*转换*/
    do{
    str[i++]=index[unum%(unsigned)radix];
    unum/=radix;
    }while(unum);
    str[i]='\0';
    /*逆序*/
    if(str[0]=='-')k=1;/*十进制负数*/
    else k=0;
    char temp;
    for(j=k;j<=(i-1)/2;j++)
    {
    temp=str[j];
    str[j]=str[i-1+k-j];
    str[i-1+k-j]=temp;
    }
    return str;
}

static void sound_play_complete(void)
{
    sys_envar_t *p_sys = &cms_envar.sys;
    if (phase != 0) {
        /* Alert has been stopped. */
        osal_timer_change(p_sys->timer_voc, SOUND_PLAY_INTERVAL);
        osal_timer_start(p_sys->timer_voc);
    }
}

static void notice_di_play_once(void *complete)
{
    //syn6288_play(NOTICE_STRING);

    //sound_play_complete();
    
    voc_play(VOC_ENCODE_ADPCM, NOTICE_STRING, 0, (voc_handler)complete);
}

static void voice_play_once(uint32_t alert_type, void *complete)
{
    uint8_t *data;
    uint32_t length = 0;
    char distance_char[10];
    memset(play_string,0,sizeof(play_string));   

    switch (alert_type) {
               
    case HI_OUT_CRD_ALERT:
        //data = CFCW_VOC;
        sprintf(distance_char,"%d", voc_distance);
        strcpy(play_string,FRONT);     
        strcat(play_string,CFCW);
        strcat(play_string,(const char*)distance_char);// (const char*)itoa(voc_distance,distance_char,10));
        strcat(play_string,MI);
        //osal_printf("string is %s\n",play_string);
        data= play_string;
        break;

    case HI_OUT_CRD_REAR_ALERT:
        data = CRCW_VOC;
        break;

    case HI_OUT_VBD_ALERT:
        //data = VBD_VOC;
        strcpy(play_string,FRONT);
        strcat(play_string,VBD);
        strcat(play_string,(const char*)itoa(voc_distance,distance_char,10));       
        strcat(play_string,MI);
        //osal_printf("string is %s\n",play_string);
        data= play_string;
        break;

    case HI_OUT_EBD_ALERT:
        //data = EEBL_VOC;
        strcpy(play_string,FRONT);
        strcat(play_string,EEBL);
        strcat(play_string,(const char*)itoa(voc_distance,distance_char,10));
        strcat(play_string,MI);

        data= play_string;
        break;

    case HI_OUT_EVA_ALERT:
        //data = EEBL_VOC;
        strcpy(play_string,REAR);
        //strcat(play_string,(const char*)itoa(voc_distance,distance_char,10));
        //strcat(play_string,MI);
        strcat(play_string,AMBU);
        strcat(play_string,GIVEWAY);
        //osal_printf("string is %s\n",play_string);
        data= play_string;
        break;
        
    case HI_OUT_RSA_ALERT:
        //data = EEBL_VOC;
        strcpy(play_string,FRONT);      
        strcat(play_string,CURVE);
        strcat(play_string,(const char*)itoa(voc_distance,distance_char,10));
        strcat(play_string,MI);
        data= play_string;
        break;
        
    
    default:

        break;
    }

   voc_play(VOC_ENCODE_ADPCM, data, length, (voc_handler)complete);
   //syn6288_play(data);
}

void sound_notice_di(void)
{
    notice_di_play_once(NULL);
}

void sound_alert_start(uint32_t alert_type,uint32_t distance)
{

    voc_distance = distance;
    
    if (alert_type != HI_OUT_NONE) {
        voice[voice_wr_idx] = alert_type;
        if (++voice_wr_idx >= MAX_VOICE) {
            voice_wr_idx = 0;
        }
    }

    if (phase == 0) {
        osal_enter_critical();
        phase = 1;
        osal_leave_critical();
        notice_di_play_once(sound_play_complete);
    }
}
void alert_start(uint32_t alert_type)
{
    //sound_alert_start(alert_type);
    //voice_play_once(alert_type,NULL);
    sound_alert_start(alert_type,100);

    
}
FINSH_FUNCTION_EXPORT(alert_start, alert_start);

void sound_alert_stop(void)
{
    osal_enter_critical();
    phase = 0;
    osal_leave_critical();

    voc_stop(NULL);
    syn6288_stop();
    memset(voice, 0, sizeof(voice));
    voice_wr_idx = 0;
    voice_rd_idx = 0;
}
void alert_stop(void)
{
    sound_alert_stop();
}
FINSH_FUNCTION_EXPORT(alert_stop, alert_stop);

void sound_alert_process(void* parameter)
{
    if (phase == 0) {
        /* Do nothing because alert has been stopped. */
    }
    else if (phase < BIBI_PRE_COUNT) {
        notice_di_play_once(sound_play_complete);
        osal_enter_critical();
        phase++;
        osal_leave_critical();
    }
    else {
        if (voice[voice_rd_idx] != 0) {
            voice_play_once(voice[voice_rd_idx], sound_play_complete);
            voice[voice_rd_idx] = 0;
            if (++voice_rd_idx >= MAX_VOICE) {
                voice_rd_idx = 0;
            }
        }
        else {
            notice_di_play_once(sound_play_complete);
            //notice_di_play_once(NULL);
        }
    }
}

