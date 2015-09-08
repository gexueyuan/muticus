/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_drv_wifi_def.h
 @brief  : This file is port from ralink's linux driver
 @author : wangyifeng
 @history:
           2014-9-28    wangyifeng    Created file
           ...
******************************************************************************/
#ifndef __CV_DRV_WIFI_DEF_H__
#define __CV_DRV_WIFI_DEF_H__

#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "WiFi"
#include "cv_osal_dbg.h"



/* 
    DEBUG CODE 
*/
#define WHED_DEBUG_OFF          OSAL_DEBUG_OFF
#define WHED_DEBUG_ERROR        OSAL_DEBUG_ERROR
#define WHED_DEBUG_WARN         OSAL_DEBUG_WARN
#define WHED_DEBUG_INFO         OSAL_DEBUG_INFO
#define WHED_DEBUG_TRACE        OSAL_DEBUG_TRACE
#define WHED_DEBUG_LOUD         OSAL_DEBUG_LOUD

#define CV_DRV_WIFI_DEBUG 1

#if CV_DRV_WIFI_DEBUG
#define DBGPRINT(level, args...) \
do { \
    OSAL_MODULE_DBGPRT(MODULE_NAME, level, ##args); \
} while(0)

#else
#define __FUNCTION__        0
#define DBGPRINT (void) /*(Level...)*/ 
#endif


int usb_control_msg(void *hostspecific, unsigned int pipe, u8 request, u8 requesttype, u16 value,
    u16 index, void *data, u16 size, int timeout);

int usb_bulkout (void *HostSpecific, void *pData, unsigned int Length);


#endif //__CV_DRV_WIFI_DEF_H__

