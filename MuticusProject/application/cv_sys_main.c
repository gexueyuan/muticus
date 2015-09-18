/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include "cv_osal.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_drv_qc.h"

#define FIRMWARE_VERSION "V2.0.000" 
#ifdef NDEBUG
#define FIRMWARE_IDEN "rel" 
#else
#define FIRMWARE_IDEN "dbg" 
#endif


#include "cv_lcd.h"
#include "app_tsc.h"
#include "app_interface.h"
#include "drv_main.h"

#include "bsp_tsc_STMPE811.h"

extern void osal_dbg_init(void);
extern void rt_platform_init(void);
extern void param_init(void);
extern void gps_init(void);
extern void led_init(void);
extern void wnet_init(void);
extern void vam_init(void);
extern void vsa_init(void);
extern void sys_init(void);
extern void gsnr_init(void);
extern int usb_init(void);
extern int rt_key_init(void);
extern void mda_init(void);
extern void voc_init(void);
extern void cpu_usage_init(void);
extern uint8_t IOE_Config(void);


cms_global_t cms_envar = { 0 };




void delay_test()
{
    uint16_t i,j;
    
    
    for(i= 10000; 0 < i; i--)
    {
        for(j = 1000; 0 < j; j--)
        {
            ;
        }
    }
}


void rt_init_thread_entry(void *parameter)
{
    	
    /* Driver main routine that open the specific drivers. */
    drv_main_open();

    
    param_init();
    //cpu_usage_init();    
    voc_init();    
    gps_init();
  	nmea_init();
    rt_led_init();
	rt_key_init();
   // usb_init();
        
    wnet_init();
    vam_init();
    mda_init();
    gsnr_init();   
    vsa_init();    
    sys_init();   
    
    
    
    lcd_thread_init();
    tsc_thread_init();
    
    
}

void qc_init_entry(void *parameter)
{
    param_init();
    qc_run_init();
}

int rt_application_init(void)
{
    rt_thread_t tid;
    active_qc_way_e   active_qc_way;
    
    
    rt_components_init();
    rt_platform_init();
    osal_dbg_init();

    osal_printf("\n\n");
    osal_printf("CID : %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n", 
                des(0), des(1), des(2), des(3), des(4), des(5), des(6), des(7), des(8), des(9), des(10), des(11));
    osal_printf("CLK : %dMHz\n", SystemCoreClock/1000000);
    osal_printf("Firm: %s[%s,%s %s]\n\n", FIRMWARE_VERSION, FIRMWARE_IDEN, __TIME__, __DATE__);

    active_qc_way = active_qc_check();
    if(ACTIVE_QC_NULL != active_qc_way) {
        /* run qc thread */
        tid = osal_task_create("qc", qc_init_entry, RT_NULL,
                                QC_INIT_THREAD_STACK_SIZE, QC_INIT_THREAD_PRIORITY);
        osal_assert(tid != NULL);
    }
    else {
        /* run app thread */
        tid = osal_task_create("init", rt_init_thread_entry, RT_NULL,
                                        RT_INIT_THREAD_STACK_SIZE, RT_INIT_THREAD_PRIORITY);
        osal_assert(tid != NULL);
    }

    return 0;
}


void get_version(void)
{
  osal_printf("\nFirm: %s[%s,%s %s]\n\n", FIRMWARE_VERSION, FIRMWARE_IDEN, __TIME__, __DATE__);
}
FINSH_FUNCTION_EXPORT(get_version, get firmware version);


