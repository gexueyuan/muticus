/**
  *****************************************************************************
  * Copyright(C) Beijing Carsmart Technology Co., Ltd.
  * All rights reserved.
  *
  * @file   : cv_drv_qc.c
  * @brief  : This file include QC related functions
  * @author : Fred
  * @history:
  *        2015-5-14    Fred    Created file
  *        ...
  ******************************************************************************
  */
#define _CV_DRV_QC_MODULE_

#include "cv_drv_qc.h"
#include "cv_vsa.h"
#include "cv_wnet.h"

#include "led.h"
#include "gps.h"



#define QC_WAIT_TIME    50

#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_TRACE
#define MODULE_NAME "qc"
#include "cv_osal_dbg.h"
OSAL_DEBUG_ENTRY_DEFINE(qc)


void null_function(void)
{
    
}

#define GPS_TEST_TIME          (4000)   //ms
#define LED_NUM    3





uint8_t led_site[3] = {LED_RED,LED_GREEN,LED_BLUE};
qc_led_type_e  qc_led_type;



//void ate_init(void);
void gps_set_host_baudrate(int baud);
void sound_notice_di(void);
void printAcc(gsnr_log_level_t level, char *des, float x, float y, float z);
float VectorSum(GSENSOR_INFO gsensor_date);
//void usb_init(void);
//void voc_init(void);
void gsnr_drv_init(void);
void GsensorReadAcc(float* pfData);

void param_init(void);

/* qc moudle globle var */
qc_envar_t  qc_envar,*p_qc_envar;

extern wnet_envar_t *p_wnet_envar;
extern const unsigned char test_8K_16bits[];
extern uint8_t get_gps;


/*****************************************************************************
 @funcname: OSAL_DEBUG_ENTRY_DEFINE
 @brief   : bsp gpio init
 @param   : qc  
 @return  : 
*****************************************************************************/
static void qc_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* key gpio init */
    RCC_AHB1PeriphClockCmd(KEY_CLK_SRC, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = KEY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(KEY_PORT, &GPIO_InitStructure);

    /* led gpio init */
    RCC_AHB1PeriphClockCmd(LED_CLK_SRC, ENABLE);
    GPIO_WriteBit(LED_PORT,LED_PIN,Bit_RESET);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    
}

/*****************************************************************************
 @funcname: qc_hw_init
 @brief   : qc related hardware init
 @param   : void  
 @return  : static
*****************************************************************************/
static  void qc_check_hw_init(void) 
{
    qc_gpio_init();       
}

/*****************************************************************************
 @funcname: button_state
 @brief   : obtain button state
 @param   : void  
 @return  : static
*****************************************************************************/
static __inline uint8_t  button_state(void)
{
    return GPIO_ReadInputDataBit(KEY_PORT,KEY_PIN);
}
/*****************************************************************************
 @funcname: button_check
 @brief   : check button down or not
 @param   : void  
 @return  : static
*****************************************************************************/
static uint8_t  button_check(void)
{	
    return  (KEY_PRESS_ON == button_state())? TRUE:FALSE;
}

/*****************************************************************************
 @funcname: shell_check
 @brief   : read bkp to check if start qc
 @param   : void  
 @return  : static
*****************************************************************************/
static uint8_t shell_check(void)
{
    uint32_t read_sgn;

    read_sgn = RTC_ReadBackupRegister(RTC_BKP_DR19);
    
    if (QC_SET_SGN == read_sgn) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
        PWR_BackupAccessCmd(ENABLE);
        RTC_WriteBackupRegister(RTC_BKP_DR19,0x00);
        return SHELL_QC_NORMAL;
    } 
    else if (QC_SET_SGN_RF == read_sgn){    
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
        PWR_BackupAccessCmd(ENABLE);
        RTC_WriteBackupRegister(RTC_BKP_DR19,0x00);
        return SHELL_QC_RF;
    }
    return SHELL_QC_NULL;
}

/*****************************************************************************
 @funcname: active_qc_check
 @brief   : check active qc thread yes or not
 @param   : None
 @return  : 
*****************************************************************************/
active_qc_way_e active_qc_check(void)
{

    qc_envar_t *p_qc;
    uint32_t check_time_base;
    uint8_t shell_command;
    active_qc_way_e active_qc = ACTIVE_QC_NULL;
    check_time_base = osal_get_systemtime();

    p_qc_envar = &qc_envar;
    p_qc = p_qc_envar;
    memset(p_qc,0x00,sizeof(qc_envar_t));
    /* hw init */
    qc_check_hw_init();

    do {
        /*button check*/
        if (TRUE == button_check()) {
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"button trriger qc success.\n\n");
            active_qc =  ACTIVE_QC_BUTTON;
            break;
        }

        /* shell command flash check */

        shell_command = shell_check();
        if(SHELL_QC_NORMAL == shell_command) {
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"shell trriger qc success.\n\n");
            active_qc =  ACTIVE_QC_SHELL;
            break;
        }
        else if(SHELL_QC_RF == shell_command) {
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"shell trriger qc RF mode success.\n\n");
            active_qc =  ACTIVE_QC_SHELL_RF;
            break;
        }
        
        /* command interactive */
    }
    while((osal_get_systemtime() - check_time_base) <= QC_WAIT_TIME);

    p_qc->active_qc_way = active_qc;
    return active_qc;
   
}


/*****************************************************************************
 @funcname: wnet_param_init
 @brief   : init wnet param for rf ate test
 @param   : None
 @return  : static
*****************************************************************************/
static  void  wnet_param_init(void)
{
    cfg_param_t  *p_cfg_param = &cms_param;
    wnet_envar_t *p_wnet;

    /* wnet module param init */
    p_wnet_envar = &cms_envar.wnet;
    p_wnet = p_wnet_envar;

    /* set wnet test mode */
    p_cfg_param->wnet.mode = WNET_TEST_MODE;
    
    memset(p_wnet, 0, sizeof(wnet_envar_t));
    memcpy(&p_wnet->working_param, &cms_param.wnet, sizeof(wnet_config_t));
}

/*****************************************************************************
 @funcname: ate_rf_init
 @brief   : init rf ate test
 @param   : void  
 @return  : static
*****************************************************************************/
static void ate_rf_init (void)
{
    wnet_param_init();
    //usb_init();
    //ate_init();
}

static void qc_led_init(void)
{
    STM_EVAL_LEDInit(LED_RED);
    STM_EVAL_LEDInit(LED_BLUE);
    STM_EVAL_LEDInit(LED_GREEN);
    
    STM_EVAL_LEDOn(LED_RED);
    STM_EVAL_LEDOn(LED_BLUE);
    STM_EVAL_LEDOn(LED_GREEN);
    qc_led_type = QC_LED_OK;
}

static void __inline led_light_yellow(void)
{
    STM_EVAL_LEDOn(LED_RED);
    STM_EVAL_LEDOff(LED_BLUE);
    STM_EVAL_LEDOn(LED_GREEN);
}

static void __inline led_all_off(void)
{
    STM_EVAL_LEDOff(LED_RED);
    STM_EVAL_LEDOff(LED_BLUE);
    STM_EVAL_LEDOff(LED_GREEN);
}

static void led_show(uint8_t led,uint8_t flg)
{
    //led_all_off();
    if(flg) {
        STM_EVAL_LEDOn((Led_TypeDef)led);
    }
    else {
        STM_EVAL_LEDOff((Led_TypeDef)led);
    }
}

static void led_control(qc_led_type_e led_type_in)
{
    static uint8_t led_type =0;
    static uint8_t step_count = 0;
    uint8_t led_flg;
    
    if(led_type_in != led_type) {
        led_type = led_type_in;
        step_count = 0;
    }
    if(led_type &(0x04>>step_count)) {
        led_flg =TRUE;
    }
    else {
        led_flg = FALSE;
    }
    led_show(led_site[step_count],led_flg);
    step_count++;
    if(step_count >= LED_NUM) {
        step_count = 0;
    }
}


static __inline void qc_check_done_led(void)
{
        led_control(qc_led_type);
}

/*****************************************************************************
 @funcname: timer_ate_tip_callback
 @brief   : led  tip control timer
 @param   : void * param  
 @return  : static
*****************************************************************************/
static void timer_ate_tip_callback(void * param)
{
    static uint8_t led_step = 0;
    qc_envar_t  *p_qc = p_qc_envar; 

    if(led_step) {
        led_step = 0;
        led_all_off();
    }
    else {
        led_step = 1;
        if (p_qc->qc_test_sch == TEST_ING) {
            led_light_yellow();
        }
        else if(p_qc->qc_test_sch == TEST_DONE){
            qc_check_done_led();
        }
    }
}   


static uint8_t qc_gsnr_test(void)
{
    uint8_t cir_count=50;
    uint8_t data_conut = 10;
    float   pfData[3]={0};
    GSENSOR_INFO gsensor_data;
    float  vector_value;
    
    
    gsnr_drv_init();    

    osal_delay(30);
    
    do {
        GsensorReadAcc(pfData);
        if(pfData[2] != 0) {
            //return TRUE;

            #if 1
            gsensor_data.x = pfData[0];
            gsensor_data.y = pfData[1];
            gsensor_data.z = pfData[2];
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"g-sensor:x: %f, y: %f ,z: %f \n",gsensor_data.x,gsensor_data.y,gsensor_data.z); 
            vector_value = VectorSum(gsensor_data);
	    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"g-sensor:value:%.6f \n",vector_value); 
            if ((vector_value > GSENSOR_THRESHOLD_LOW)&&(vector_value < GSENSOR_THRESHOLD_HIGH)) {
                data_conut--;
                if(data_conut <= 0) {
                    return TRUE;
                }
            }
            #endif 
        }

    }while(cir_count--);
    return FALSE;
}

static uint8_t qc_gps_test(void)
{
    rt_device_t dev ;
    uint32_t time_base;
    uint8_t error_count;
    uint8_t tmp = 0 ;
    uint8_t change_sgn = FALSE;
    
    dev = rt_device_find(RT_GPS_DEVICE_NAME);
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);
    
    time_base = osal_get_systemtime();
    //osal_printf("base time:%d\n",time_base);
   
    do {
        if(rt_device_read(dev, 0, &tmp, 1) == 1) {
             //osal_printf("data:%d\n",tmp);
            if (tmp == '$') {
                //osal_printf("test OK time:%d\n",osal_get_systemtime());
                return TRUE;
            }
            if(((tmp <= 0)||(tmp > 128))&&(change_sgn ==FALSE)) {
                error_count++;
                if(error_count >5) {
                    //osal_printf("change to 9600:%d\n");
                    gps_set_host_baudrate(BAUD_RATE_115200);
                    change_sgn = TRUE;
                }
            }
        }
    } while((osal_get_systemtime() -time_base) <= GPS_TEST_TIME);
    return FALSE;
}


static void qc_test_report(void)
{   
    qc_envar_t  *p_qc = p_qc_envar;
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"****qc peripheral test report:****\n"); 

    if(p_qc->qc_periph.gps.state) {
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,">>GPS test OK.\n");
    }else {
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,">>GPS test error:%d\n",p_qc->qc_periph.gps.error);
        qc_led_type = (qc_led_type > QC_LED_001)?QC_LED_001:qc_led_type;
    }

    if(p_qc->qc_periph.gsensor.state) {
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,">>G-sensor test OK.\n");
    }else {
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,">>G-sensor test error:%d\n",p_qc->qc_periph.gsensor.error);
        qc_led_type = (qc_led_type > QC_LED_010)?QC_LED_010:qc_led_type;
    }
}

static void qc_periph_test(void)
{
    qc_envar_t  *p_qc = p_qc_envar;
    
    if(p_qc->qc_periph.test_sw == FALSE) {
        return ;
    }
    switch (p_qc->qc_periph.periph_fsm) {

        case  TEST_BEGIN: {

            p_qc->qc_periph.periph_fsm = TEST_GPS;
        }
        break;

        case TEST_GPS:  {
            if (qc_gps_test() == TRUE) {
                p_qc->qc_periph.gps.state =TRUE;
            }
            else {
                p_qc->qc_periph.gps.state =FALSE;
            }
            p_qc->qc_periph.periph_fsm = TEST_GSENR;
        }
        break;

        case TEST_GSENR:  {

            if (qc_gsnr_test() == TRUE) {
                p_qc->qc_periph.gsensor.state =TRUE;
            }else {
                p_qc->qc_periph.gsensor.state =FALSE;
            }
            p_qc->qc_periph.periph_fsm = TEST_REPORT;
        }
        break;

        case TEST_REPORT: {
            //osal_delay(500);
            qc_test_report();
            p_qc->qc_periph.periph_fsm = TEST_FINSH;
        }
        break;

        case TEST_FINSH: {
            p_qc->qc_test_sch= TEST_DONE;
        }
        break;
    }
}
/*****************************************************************************
 @funcname: qc_thread_entry
 @brief   : ATE thread entry
 @param   : void *param  
 @return  : static
*****************************************************************************/
static void php_thread_entry(void *param)
{
    qc_envar_t  *p_qc = p_qc_envar;

    p_qc->qc_periph.test_sw = TRUE;
    
    qc_led_init();
    if(p_qc->active_qc_way != ACTIVE_QC_SHELL_RF) {
        //voc_init();
        /* dididi */
        //sound_notice_di();
        //osal_delay(50);
        //sound_notice_di();
        //osal_delay(50);
        //sound_notice_di();
        //osal_delay(50);     
    }
   
    if (p_qc->active_qc_way == ACTIVE_QC_BUTTON) {     
        osal_delay(200);
        if (TRUE == button_check()) {
            p_qc->qc_periph.test_sw =FALSE;
            OSAL_DBGPRT(OSAL_DEBUG_INFO, "close periph test\n");
        }
    }
    osal_timer_start(p_qc->tm_tip);
    p_qc->qc_periph.periph_fsm = TEST_BEGIN;
    while(1) {
        
        qc_periph_test();
        /* key press test */
        if (TRUE == button_check()) {
            OSAL_DBGPRT(OSAL_DEBUG_INFO, "key press down.\n");
        }
        osal_delay(10);
    }
}



/*****************************************************************************
 @funcname: ate_php_init
 @brief   : peripheral ate test init
 @param   : void  
 @return  : 
*****************************************************************************/
void ate_php_init(void)
{
    qc_envar_t  *p_qc;

    p_qc = p_qc_envar;

    p_qc->tm_tip= osal_timer_create("tm-tip",timer_ate_tip_callback,NULL, 50,TRUE); 
    
    p_qc->task_qc = osal_task_create("php", php_thread_entry, \
                        NULL, QC_PHP_THREAD_STACK_SIZE, QC_PHP_THREAD_PRIORITY);
    osal_assert(p_qc->task_qc != NULL);  
    
}
/*****************************************************************************
 @funcname: qc_run_init
 @brief   : init qc module
 @param   : void *param  
 @return  : 
*****************************************************************************/
void qc_run_init(void)
{    
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"<notice>:qc thread start...\n");
     
    /* RF test module init */
    ate_rf_init();

    /* Peripherals test module init */
    ate_php_init();
                   
}


/*****************************************************************************
 @funcname: run_qc
 @brief   : shell command : reset sys to run qc
 @param   : void  
 @return  : 
*****************************************************************************/
void run_qc(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RTC_WriteBackupRegister(RTC_BKP_DR19,QC_SET_SGN);

    if(QC_SET_SGN == RTC_ReadBackupRegister(RTC_BKP_DR19)) {

        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"qc run sgn set success!\n");   
    }
    NVIC_SystemReset();
}
FINSH_FUNCTION_EXPORT(run_qc, reset systerm to run qc)

/*****************************************************************************
 @funcname: run_qc
 @brief   : shell command : reset sys to run qc
 @param   : void  
 @return  : 
*****************************************************************************/
void run_qc_rf(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RTC_WriteBackupRegister(RTC_BKP_DR19,QC_SET_SGN_RF);

    if(QC_SET_SGN_RF == RTC_ReadBackupRegister(RTC_BKP_DR19)) {

        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"qc run sgn set success!\n");   
    }
    NVIC_SystemReset();
}
FINSH_FUNCTION_EXPORT(run_qc_rf, reset systerm to run RF test)



/*****************************************************************************
 @funcname: run_qc
 @brief   : shell command : reset sys to run qc
 @param   : void  
 @return  : 
*****************************************************************************/
void run_update(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    RTC_WriteBackupRegister(RTC_BKP_DR18,UPDATE_SET_SGN);

    if(UPDATE_SET_SGN == RTC_ReadBackupRegister(RTC_BKP_DR18)) {

        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"update run sgn set success!\n");   
    }
    NVIC_SystemReset();
}
FINSH_FUNCTION_EXPORT(run_update, reset systerm to update from uart)


