/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_wnet_main.c
 @brief  : this file realize wireless network managment
 @author : wangyifeng
 @history:
           2014-6-17    wangyifeng    Created file
           2014-7-29    wanglei       modified file: process evam msg 
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "wnet"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_wnet.h"


#include "drv_main.h"



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
wnet_envar_t *p_wnet_envar = NULL;

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

void wnet_tx_thread_entry(void *parameter)
{
    int err;
    wnet_envar_t *p_wnet = (wnet_envar_t *)parameter;


    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "%s: ---->\n", __FUNCTION__);

	while(1)
    {
	    err = osal_sem_take(p_wnet->sem_wnet_tx, OSAL_WAITING_FOREVER);

        if (err == OSAL_STATUS_SUCCESS)
        {
            fp_tx_handler(p_wnet);
        }
        else
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_ERROR, "%s:failed to take semaphore(%d)\n", __FUNCTION__, err);
        }
	}
}




/**
  * @brief  wireless network usart send routine.
  * @param  See below.
  * @retval OK.
  */
extern void wnet_receive_main
(
    uint8_t *buffer_ptr
);



void wnet_rx_thread_entry(void *parameter)
{
    int err;
    wnet_envar_t *p_wnet = (wnet_envar_t *)parameter;

    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "%s: ---->\n", __FUNCTION__);

    wnet_receive_main(NULL);

	while(1)
    {
	    err = osal_sem_take(p_wnet->sem_wnet_rx, OSAL_WAITING_FOREVER);
        
        if (err == OSAL_STATUS_SUCCESS)
        {
            fp_rx_handler(p_wnet);
        }
        else
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_ERROR, "%s:failed to take semaphore(%d)\n", __FUNCTION__, err);
        }
	}
}


void wnet_init(void)
{
    int i;
    
    wnet_envar_t *p_wnet;
    p_wnet_envar = &cms_envar.wnet;
    p_wnet = p_wnet_envar;

    memset(p_wnet, 0, WNET_ENVAR_T_LEN);
    memcpy(&p_wnet->working_param, &cms_param.wnet, WNET_CONFIG_T_LEN);


    /* Initialize the txbuf queue. */
    INIT_LIST_HEAD(&p_wnet->txbuf_waiting_list);
    INIT_LIST_HEAD(&p_wnet->txbuf_free_list);
    for(i = 0; i< TXBUF_NUM; i++)
    {
        list_add_tail(&p_wnet->txbuf[i].list, &p_wnet->txbuf_free_list);
    }

    /* Initialize the rxbuf queue. */
    INIT_LIST_HEAD(&p_wnet->rxbuf_waiting_list);
    INIT_LIST_HEAD(&p_wnet->rxbuf_free_list);
    for(i = 0; i < RXBUF_NUM; i++)
    {
        list_add_tail(&p_wnet->rxbuf[i].list, &p_wnet->rxbuf_free_list);
    }

    /* os object for wnet */
    p_wnet->sem_wnet_tx = osal_sem_create("wntx", 0);
    osal_assert(p_wnet->sem_wnet_tx != NULL);

    p_wnet->sem_wnet_rx = osal_sem_create("wnrx", 0);
    osal_assert(p_wnet->sem_wnet_rx != NULL);

    p_wnet->task_wnet_tx = osal_task_create("wntx",
                           wnet_tx_thread_entry, p_wnet, RT_THREAD_STACK_SIZE, RT_WNETTX_THREAD_PRIORITY);
    osal_assert(p_wnet->task_wnet_tx != NULL);

    p_wnet->task_wnet_rx = osal_task_create("wnrx",
                           wnet_rx_thread_entry, p_wnet, RT_THREAD_STACK_SIZE, RT_WNETRX_THREAD_PRIORITY);
    osal_assert(p_wnet->task_wnet_rx != NULL);
}


