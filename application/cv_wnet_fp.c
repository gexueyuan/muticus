/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_wnet_fp.c
 @brief  : This file realizes the frame process
 @author : wangyf
 @history:
           2014-12-8    wangyf    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "fp"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_wnet.h"


int wnet_dbg_trx_all = 0;
int wnet_dbg_rx_actual = 0;
int wnet_dbg_rx_fresh = 0;
int wnet_dbg_cacul_peroid = 10;  /* unit: second */
int wnet_dbg_cacul_cnt = 0;
extern float rcp_dbg_distance;

void test_comm(void)
{
    int rx_ratio;

	  if (++wnet_dbg_cacul_cnt >= wnet_dbg_cacul_peroid*10) {
        rx_ratio = wnet_dbg_rx_actual*100/wnet_dbg_cacul_cnt;
        osal_printf("\r\n[RX] Max=%d Act=%d Ratio=%d%% dis=%d\r\n\r\n", wnet_dbg_cacul_cnt,\
                    wnet_dbg_rx_actual, rx_ratio, (int)rcp_dbg_distance);

        wnet_dbg_cacul_cnt = 0;
        wnet_dbg_rx_actual = 0;
    }
		
    if (wnet_dbg_rx_fresh > 0) {
        if(wnet_dbg_rx_fresh > 1){
            osal_printf("\b");
        }
        do {
            osal_printf(".");
        } while ((--wnet_dbg_rx_fresh > 0));
        wnet_dbg_rx_fresh = 0;
    }
    else {
        osal_printf(" ");
    }

}

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
//int drv_wifi_send(wnet_txinfo_t *txinfo, uint8_t *pdata, int32_t length);
//int drv_wifi_mac_header_len(void);

extern wnet_envar_t *p_wnet_envar;

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

/**
 * Put the TXBUF into waiting list only.
 * When the prority is EMERGENCY, the element is inserted to the front of 
 * the first NORMAL one.
 */
int fp_send(wnet_envar_t *p_wnet, wnet_txinfo_t *txinfo, uint8_t *pdata, uint32_t length)
{
    wnet_txbuf_t *txbuf = WNET_TXBUF_PTR(txinfo);
    list_head_t *p_txbuf_waiting_list = &p_wnet->txbuf_waiting_list;
    list_head_t *pos = NULL;

    txbuf->data_ptr = pdata;
    txbuf->data_len = length;
    txbuf->flag = TXBUF_FLAG_NONE;

    osal_enter_critical();
    
    if (txinfo->prority == WNET_TRANS_RRORITY_EMERGENCY) 
    {
        if (!list_empty(p_txbuf_waiting_list)) 
        {
            wnet_txbuf_t *p;
            list_for_each_entry(p, wnet_txbuf_t, &p_wnet->txbuf_waiting_list, list)
            {
                if ((!(txbuf->flag & TXBUF_FLAG_PROCESSING))&&(p->info.prority != WNET_TRANS_RRORITY_EMERGENCY)) 
                    {
                        pos = &p->list;
                        break;
                    }
            }
        }
    }
    
    if (pos)
    {
        __list_add(&txbuf->list, pos->prev, pos);
    }
    else
    {
        list_add_tail(&txbuf->list, p_txbuf_waiting_list);
    }
    
    osal_leave_critical();

    osal_sem_release(p_wnet->sem_wnet_tx);

    return 0;
}


void fp_tx_handler(wnet_envar_t *p_wnet)
{
    wnet_txbuf_t               *txbuf = NULL;
    list_head_t *p_txbuf_waiting_list = &p_wnet->txbuf_waiting_list;
    int                     send_next = 0;


    osal_enter_critical();
    
    if (!list_empty(p_txbuf_waiting_list)) 
    {
        txbuf = list_first_entry(p_txbuf_waiting_list, wnet_txbuf_t, list);

        if (!(txbuf->flag & TXBUF_FLAG_PROCESSING)) 
        {
            txbuf->flag |= TXBUF_FLAG_PROCESSING;
            send_next = 1;
        }
    }
    
    osal_leave_critical();
    
    if (send_next)
    {
       // if (drv_wifi_send(&txbuf->info, txbuf->data_ptr, txbuf->data_len) < 0) 
        {
            /* At this time, the phy layer may be not ready, we should release the txbuf manually */
            fp_tx_complete(p_wnet);
        }
    }
}

/**
 * Put the TXBUF back free list only.
 */
void fp_tx_complete(wnet_envar_t *p_wnet)
{
    wnet_txbuf_t *txbuf = NULL;
    list_head_t *p_txbuf_waiting_list = &p_wnet->txbuf_waiting_list;


    osal_enter_critical();
    
    if (!list_empty(p_txbuf_waiting_list)) 
    {
        txbuf = list_first_entry(p_txbuf_waiting_list, wnet_txbuf_t, list);
        if (txbuf->flag & TXBUF_FLAG_PROCESSING) 
        {
            txbuf->flag &= ~TXBUF_FLAG_PROCESSING;
        }

        list_move_tail(&txbuf->list, &p_wnet->txbuf_free_list);
    }		
    
    osal_leave_critical();

#if 0
    /* Kick out next frame */
    if (!list_empty(p_txbuf_waiting_list)) 
    {
        osal_sem_release(p_wnet->sem_wnet_tx);
    }
#endif
}


/**
 * Allocate a RXBUF and put it into waiting list.
 */
int fp_recv(wnet_envar_t *p_wnet, wnet_rxinfo_t *rxinfo, uint8_t *pdata, uint32_t length)
{
    wnet_rxbuf_t *rxbuf = NULL;


    rxbuf = fp_get_rxbuf(p_wnet);
    if (!rxbuf)
    {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "failed to get rxbuf\n");
        return -1;
    }

    memcpy(&rxbuf->info, rxinfo, sizeof(wnet_rxinfo_t));
    memcpy(rxbuf->buffer, pdata, length);
    rxbuf->data_ptr = rxbuf->buffer;
    rxbuf->data_len = length;

    osal_enter_critical();
    list_add_tail(&rxbuf->list, &p_wnet->rxbuf_waiting_list);
    osal_leave_critical();

    osal_sem_release(p_wnet->sem_wnet_rx);

    return 0;
}


void fp_rx_handler(wnet_envar_t *p_wnet)
{
    wnet_rxbuf_t *rxbuf = NULL;
    list_head_t *p_rxbuf_waiting_list = &p_wnet->rxbuf_waiting_list;


    osal_enter_critical();
    
    if (!list_empty(p_rxbuf_waiting_list)) 
    {
        rxbuf = list_first_entry(p_rxbuf_waiting_list, wnet_rxbuf_t, list);
        list_del(&rxbuf->list);
    }		
    osal_leave_critical();
    
    if (rxbuf)
    {
        /* if (llc_recv(p_wnet, &rxbuf->info, rxbuf->data_ptr, rxbuf->data_len) < 0) */ 
        if (enet_receive(p_wnet, &rxbuf->info, rxbuf->data_ptr, rxbuf->data_len) < 0)
        {
            fp_release_rxbuf(p_wnet, rxbuf);
        }
    }
}


wnet_txbuf_t *fp_get_txbuf(wnet_envar_t *p_wnet)
{
    wnet_txbuf_t *txbuf = NULL;
    list_head_t *p_txbuf_free_list = &p_wnet->txbuf_free_list;


    osal_enter_critical();
    
    if (!list_empty(p_txbuf_free_list)) 
    {
        txbuf = list_first_entry(p_txbuf_free_list, wnet_txbuf_t, list);
        
        list_del(&txbuf->list);
        //txbuf->data_ptr = txbuf->buffer + TXBUF_RESERVE_LENGTH + drv_wifi_mac_header_len();
        txbuf->data_len = 0;
    }
    
    osal_leave_critical();

    return txbuf;   
}


void fp_release_txbuf(wnet_envar_t *p_wnet, wnet_txbuf_t *txbuf)
{
    osal_enter_critical();
    list_add_tail(&txbuf->list, &p_wnet->txbuf_free_list);
    osal_leave_critical();
}

wnet_rxbuf_t *fp_get_rxbuf(wnet_envar_t *p_wnet)
{
    wnet_rxbuf_t *rxbuf = NULL;
    list_head_t *p_rxbuf_free_list = &p_wnet->rxbuf_free_list;

    osal_enter_critical();
    if (!list_empty(p_rxbuf_free_list)) {
        rxbuf = list_first_entry(p_rxbuf_free_list, wnet_rxbuf_t, list);
        list_del(&rxbuf->list);
    }		
    osal_leave_critical();

    return rxbuf;   
}

void fp_release_rxbuf(wnet_envar_t *p_wnet, wnet_rxbuf_t *rxbuf)
{
    osal_enter_critical();
    list_add_tail(&rxbuf->list, &p_wnet->rxbuf_free_list);
    osal_leave_critical();
}

wnet_txbuf_t *wnet_get_txbuf(void)
{
    return fp_get_txbuf(p_wnet_envar);
}

void wnet_send_complete(void)
{
    fp_tx_complete(p_wnet_envar);
}

void wnet_release_txbuf(wnet_txbuf_t *txbuf)
{
    fp_release_txbuf(p_wnet_envar, txbuf);
}

void wnet_release_rxbuf(wnet_rxbuf_t *rxbuf)
{
    fp_release_rxbuf(p_wnet_envar, rxbuf);
}

int wnet_send(wnet_txinfo_t *txinfo, uint8_t *pdata, uint32_t length)
{
    int r; 
    wnet_envar_t *p_wnet = p_wnet_envar;

    
    switch (txinfo->protocol) 
    {
        case WNET_TRANS_PROT_DSMP:
        {
            r = dsmp_send(p_wnet, txinfo, pdata, length);    break;
        }
        default:
        {
            r = -1;                                          break;
        }
    }

    return r;
}

int wnet_recv(wnet_rxinfo_t *rxinfo, uint8_t *pdata, uint32_t length)
{
    if (0x1 & g_dbg_print_type){
        wnet_dbg_rx_fresh++;
        wnet_dbg_rx_actual++;
    }
    //osal_printf(" "); /* Indicate RX is in process, for debug only */
    return fp_recv(p_wnet_envar, rxinfo, pdata, length);
}

