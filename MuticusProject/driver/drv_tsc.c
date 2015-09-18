/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_tsc.c
 @brief  : this file include the driver functions for the touch sensor control 
           module.
 @author : wangxianwen
 @history:
           2015-9-11    wangxianwen    Created file
           ...
******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "drv_tsc.h"

#include "stm32f4xx.h"
#include "bsp_tsc_STMPE811.h"
#include "bsp_lcd_HX8257.h"



/**
  * @brief  Get touch state.
  * @param  See below.
  * @retval DRV_TSC_ERR_OK or DRV_TSC_ERR_IOCTL.
  */
static DRV_TSC_ERR_CODE tsc_ioctl_get_touch_state
(
    /* Pointer to touch state structure. */
    drv_tsc_touch_state_st_ptr state_ptr
)
{    
    tsc_state_st              tsc_state_tmp = { 0 };

    static drv_tsc_touch_state_st state_pre = { 0 };
           drv_tsc_touch_state_st state_cur = { 0 };

    uint16_t                         x_diff = 0;
    uint16_t                         y_diff = 0;

    
    /* Error detection. */
    if(state_ptr == (void*)0) 
    { 
        return DRV_TSC_ERR_IOCTL;
    }  

    /* Get the tsc touch state data from device. */
    if(TSC_get_touch_state(&tsc_state_tmp) == TSC_OK)
    {
        /* Convert the ADC value into coordinate. */
        state_cur.touched = (tsc_state_tmp.Touched == TSC_TOUCHED_YES)? TSC_TOUCH_STATE_YES : TSC_TOUCH_STATE_NO;
        state_cur.coor_x = (tsc_state_tmp.X * LCD_PIXEL_WIDTH) / (TSC_XADC_MAX - TSC_XADC_MIN);
        state_cur.coor_y = (tsc_state_tmp.Y * LCD_PIXEL_HEIGHT) / (TSC_YADC_MAX - TSC_YADC_MIN);
        state_cur.coor_z = 0;

        /* Correct coordinate when necessary. */
        if(LCD_PIXEL_WIDTH <= state_cur.coor_x)
        {
            state_cur.coor_x = LCD_PIXEL_WIDTH - 1;
        }
        if(LCD_PIXEL_HEIGHT <= state_cur.coor_y)
        {
            state_cur.coor_y = LCD_PIXEL_HEIGHT - 1;
        }

        /* Upgrade xy previous data based on differce. */
        x_diff = (state_pre.coor_x < state_cur.coor_x)? (state_cur.coor_x - state_pre.coor_x):(state_pre.coor_x - state_cur.coor_x);
        y_diff = (state_pre.coor_y < state_cur.coor_y)? (state_cur.coor_y - state_pre.coor_y):(state_pre.coor_y - state_cur.coor_y);    

        state_pre.touched = state_cur.touched;
        
        if(state_cur.touched == TSC_TOUCH_STATE_NO)
        {
            state_pre.coor_x = 0;
            state_pre.coor_y = 0;
            state_pre.coor_z = 0;
        }
        else
        {
            if(TSC_XYCOOR_FILTER < (x_diff + y_diff))
            {
                state_pre.coor_x = state_cur.coor_x;
                state_pre.coor_y = state_cur.coor_y;
                state_pre.coor_z = state_cur.coor_z;
            }
        }
        

        *state_ptr = state_pre;
        
        return DRV_TSC_ERR_OK;
    }
    else
    {
        return DRV_TSC_ERR_IOCTL;
    }   
}


/**
  * @brief  Get device state.
  * @param  See below.
  * @retval DRV_TSC_ERR_OK or DRV_TSC_ERR_IOCTL.
  */
static DRV_TSC_ERR_CODE tsc_ioctl_get_device_state
(
    /* Pointer to device state structure. */
    drv_tsc_device_state_value_ptr state_ptr
)
{    
    /* Error detection. */
    if(state_ptr == (void*)0) 
    { 
        return DRV_TSC_ERR_IOCTL;
    }  

    /* Get the tsc device state data from device. */
    if(TSC_get_device_state() == TSC_OK)
    {  
        *state_ptr = TSC_DEVICE_STATE_OK;
    }
    else
    {
        *state_ptr = TSC_DEVICE_STATE_FAILURE;
    }   
    
    return DRV_TSC_ERR_OK;  
}


/**
  * @brief  Main routine for tsc driver open.
  * @param  See below.
  * @retval DRV_TSC_ERR_OK or DRV_TSC_ERR_OPEN.
  */
static DRV_TSC_ERR_CODE drv_tsc_open
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    /* Initialize the TSC. */
    TSC_init();
    
    return DRV_TSC_ERR_OK;
}


/**
  * @brief  Main routine for tsc driver close.
  * @param  See below.
  * @retval DRV_TSC_ERR_OK or DRV_TSC_ERR_CLOSE.
  */
static DRV_TSC_ERR_CODE drv_tsc_close
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    return DRV_TSC_ERR_CLOSE;
}


/**
  * @brief  Main routine for tsc driver read.
  * @param  See below.
  * @retval DRV_TSC_ERR_OK or DRV_TSC_ERR_READ.
  */
static DRV_TSC_ERR_CODE drv_tsc_read
(
	/* Pointer of device driver structure. */
	void *drv_ptr,
    
    /* Data address for data fetching. */
    uint32_t addr,
    
    /* Data destination buffer address. */
    uint32_t * buffer_ptr,
    
    /* Data size based on uint8_t. */
    uint32_t size
)
{
    return DRV_TSC_ERR_READ;
}


/**
  * @brief  Main routine for tsc driver write.
  * @param  See below.
  * @retval DRV_TSC_ERR_OK or DRV_TSC_ERR_WRITE.
  */
static DRV_TSC_ERR_CODE drv_tsc_write
(
    /* Pointer of device driver structure. */
	void *drv_ptr,
    
    /* Data address for data writing. */
    uint32_t addr,
    
    /* Data source buffer address. */
    uint32_t * buffer_ptr,
    
    /* Data size based on uint8_t. */
    uint32_t size
)
{
    return DRV_TSC_ERR_WRITE;
}


/**
  * @brief  Main routine for tsc driver ioctl.
  * @param  See below.
  * @retval DRV_TSC_ERR_OK or DRV_TSC_ERR_IOCTL.
  */
static DRV_TSC_ERR_CODE drv_tsc_ioctl
(
	/* Pointer of device driver structure. */
	void *drv_ptr, 
	
	/* Ioctl command. */
	DRV_TSC_IOCTL_CMD cmd, 
	
	/* Pointer of parameter. */
	void *param_ptr
)
{
    DRV_TSC_ERR_CODE err_code = DRV_TSC_ERR_IOCTL;
	
	
    switch (cmd)
	{
        /* get touch state. */
		case TSC_IOCTL_GET_TOUCH_STATE:      {  err_code = tsc_ioctl_get_touch_state(param_ptr);  break;  }
       
        /* get device state. */
        case TSC_IOCTL_GET_DEVICE_STATE:     {  err_code = tsc_ioctl_get_device_state(param_ptr); break;  }

			
		default:                             {  err_code = DRV_TSC_ERR_IOCTL;                     break;  }

	}
	
	return err_code;
}


/* Main object for tsc driver. */
drv_tsc_st TscDriver = { drv_tsc_open, drv_tsc_close, drv_tsc_read, drv_tsc_write, drv_tsc_ioctl };














