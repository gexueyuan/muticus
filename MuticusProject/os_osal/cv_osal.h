/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_osal.h
 @brief  : This is the OS adapter layer realization.
 @author : wangyf
 @history:
           2014-11-12    wangyf    port from project 'cuckoo'. 
           ...
******************************************************************************/
#ifndef __CV_OSAL_H__
#define __CV_OSAL_H__


/**
 * Basic difinitions
 */
#ifndef NULL
 #define NULL ((void*)0)
#endif

#ifndef TRUE
 #define TRUE (1)
#endif

#ifndef FALSE
 #define FALSE (0)
#endif


/**
 * Dynamic Memory Function
 */
#define OSAL_DMEM_EN (1)

/**
 * RTOS's APIs
 */
#define OS_RT_THREAD

#if defined(OS_RT_THREAD)
#include "cv_osal_rtt.h"
#elif defined(OS_UCOS)
#include "cv_osal_ucos.h"
#endif


#endif /* __CV_OSAL_H__ */

