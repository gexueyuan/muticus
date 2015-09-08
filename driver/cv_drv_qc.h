/**
  *****************************************************************************
  * Copyright(C) Beijing Carsmart Technology Co., Ltd.
  * All rights reserved.
  *
  * @file   : cv_drv_qc.h
  * @brief  : cv_drv_qc.c header file
  * @author : Fred
  * @history:
  *        2015-5-14    Fred    Created file
  *        ...
  ******************************************************************************
  */
#ifndef __CV_DRV_QC_H__
#define __CV_DRV_QC_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#include "cv_osal.h"
#include "components.h"
#include "cv_cms_def.h"



#define KEY_PRESS_ON      (0)
#define KEY_PRESS_OFF     (1)

#define QC_SET_SGN         0xA5A5
#define QC_SET_SGN_RF   0x01
#define UPDATE_SET_SGN   0xA5


#define QC_MODULE_RF    (1<<0)



/* Pin Mapping */

/* PC7->key */
#define   KEY_CLK_SRC   RCC_AHB1Periph_GPIOC
#define   KEY_PORT    GPIOC
#define   KEY_PIN       GPIO_Pin_7

/* PC9->led */
#define LED_CLK_SRC   RCC_AHB1Periph_GPIOC
#define LED_PORT   GPIOC
#define LED_PIN      GPIO_Pin_9

#define bsp_led_on()    (GPIO_SetBits(LED_PORT,LED_PIN))
#define bsp_led_off()   (GPIO_ResetBits(LED_PORT,LED_PIN))



#define GSENSOR_THRESHOLD_LOW    (9.0)
#define GSENSOR_THRESHOLD_HIGH    (11.0)

#define SHELL_QC_NULL         (0)
#define SHELL_QC_NORMAL     (1)
#define SHELL_QC_RF                (2)



typedef enum {
    ACTIVE_QC_NULL = 0,
    ACTIVE_QC_BUTTON,
    ACTIVE_QC_SHELL,
    ACTIVE_QC_SHELL_RF,
    ACTIVE_QC_COMMAND
}active_qc_way_e;

/* red greed blue xxx*/
typedef enum {
    QC_LED_NULL =0,
    QC_LED_001,
    QC_LED_010,
    QC_LED_011,
    QC_LED_100,
    QC_LED_101,
    QC_LED_110,
    QC_LED_OK,
}qc_led_type_e;


typedef enum {
    TEST_ING = 0,
    TEST_DONE
}qc_test_sch_e;

typedef struct {
  uint8_t state;
  uint8_t error;
}periph_object;


typedef enum {
    TEST_BEGIN,
    TEST_GPS,
    TEST_GSENR,
    TEST_REPORT,
    TEST_FINSH
}periph_fsm_t;

typedef struct {
    uint8_t test_sw;
    periph_fsm_t periph_fsm; 
    periph_object gsensor;
    periph_object gps;
}qc_periph_t;

typedef struct {
    /* os object related */
    osal_task_t * task_qc;
    osal_timer_t  *tm_tip;

    active_qc_way_e  active_qc_way;
    qc_test_sch_e  qc_test_sch;
    qc_periph_t  qc_periph;
}qc_envar_t;

/*****************************************************************************
 * declaration of global variables and functions                             *
*****************************************************************************/
active_qc_way_e active_qc_check(void);
void qc_run_init(void);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __CV_DRV_QC_H__ */
