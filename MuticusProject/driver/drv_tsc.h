/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_tsc.h
 @brief  : this file include the driver variables and functions prototypes for 
           the touch sensor control module.
 @author : wangxianwen
 @history:
           2015-9-11    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _DRV_TSC_H_
#define _DRV_TSC_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


/* Constants -----------------------------------------------------------------*/

/* TSC X/Y adc valid range. */ 
#define TSC_XADC_MIN                 160
#define TSC_XADC_MAX                 3925
#define TSC_YADC_MIN                 130  
#define TSC_YADC_MAX                 3750

/* TSC X+Y coordinate filter. */
#define TSC_XYCOOR_FILTER            4



/* Error code for tsc device driver. */
typedef enum _drv_tsc_err_code
{
    DRV_TSC_ERR_OK = 0x0040,
    
    DRV_TSC_ERR_OPEN,
    DRV_TSC_ERR_CLOSE,
    DRV_TSC_ERR_READ,
    DRV_TSC_ERR_WRITE,
    DRV_TSC_ERR_IOCTL
    
}DRV_TSC_ERR_CODE, *DRV_TSC_ERR_CODE_PTR;

#define DRV_TSC_ERR_CODE_LEN    (sizeof(DRV_TSC_ERR_CODE))



/* Cmd for tsc device ioctl. */
typedef enum dev_tsc_ioctl_cmd
{       
    /* ioctl command: get touch state. */
	TSC_IOCTL_GET_TOUCH_STATE,

    /* ioctl command: get device state. */
    TSC_IOCTL_GET_DEVICE_STATE
	
}DRV_TSC_IOCTL_CMD, *DRV_TSC_IOCTL_CMD_PTR;


/* Exported_Structure --------------------------------------------------------*/

/* Device driver structure for tsc device. */
typedef struct _drv_tsc_st
{	
    /* Object's operation group. */
    DRV_TSC_ERR_CODE (*open) (void *);
    DRV_TSC_ERR_CODE (*close)(void *);
    DRV_TSC_ERR_CODE (*read) (void *, uint32_t, uint32_t *, uint32_t);
    DRV_TSC_ERR_CODE (*write)(void *, uint32_t, uint32_t *, uint32_t);
    DRV_TSC_ERR_CODE (*ioctl)(void *, DRV_TSC_IOCTL_CMD, void *);
	
}drv_tsc_st, *drv_tsc_st_ptr;

#define DRV_TSC_ST_LEN    (sizeof(drv_tsc_st))


/* Touch sensor controller state structure. */
typedef struct _drv_tsc_touch_state_st_
{
    /* Touched or not. */
    uint8_t touched;

    /* Touched coordinate. Caution: valid only when touched is TSC_TOUCHED_YES. */
    uint16_t coor_x;
    uint16_t coor_y;
    uint8_t  coor_z;
  
}drv_tsc_touch_state_st, * drv_tsc_touch_state_st_ptr; 

#define DRV_TSC_TOUCH_STATE_ST_LEN    (sizeof(drv_tsc_touch_state_st))

/* Tsc touched status. */
#define TSC_TOUCH_STATE_NO    0x00
#define TSC_TOUCH_STATE_YES   0x01


/* tsc device state enum definition. */    
typedef enum _drv_tsc_device_state_value     
{
    TSC_DEVICE_STATE_OK,
    TSC_DEVICE_STATE_FAILURE
    
}drv_tsc_device_state_value, *drv_tsc_device_state_value_ptr;

#define DRV_TSC_DEVICE_STATE_VALUE_LEN    (sizeof(drv_tsc_device_state_value))



/* Exported main object for tsc driver. --------------------------------------*/
extern drv_tsc_st TscDriver;

#define DRV_TSC_PTR    ((drv_tsc_st_ptr)&TscDriver)


#ifdef __cplusplus
}
#endif

#endif /* _DRV_TSC_H_ */
