/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cm_vam_mda.c
 @brief  : This file reliaze the funtion of Multihop Distributed Algorithm
 @author : wangyf
 @history:
           2015-1-5    wangyf    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "mda"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_wnet.h"
#include "cv_mda.h"

#define VAM_MDA_ENABLE 1

#ifdef  VAM_MDA_ENABLE

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/


extern int rcp_send_forward_msg(wnet_txbuf_t *txbuf);
extern wnet_txbuf_t *rcp_create_forward_msg(uint8_t left_hops, uint8_t *pdata, uint32_t length);

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

static int32_t alg_farest_node(mda_envar_t *p_mda, mda_msg_info_t *src_sta, mda_msg_info_t *pre_sta)
{
    int dis, delay;
    dis = vam_get_peer_relative_pos(src_sta->forward_id, 0);
    if (dis > 0){
        dis = (dis > V2V_COMMUNICATION_RANGE) ? V2V_COMMUNICATION_RANGE : dis;
        delay = (1 - dis / V2V_COMMUNICATION_RANGE) * MDA_FORWARD_DELAY_TIME_UNIT;
    }
    else {
        delay = -1;
    }
    return delay;
}

static void update_history_record(mda_envar_t *p_mda)
{
    int err;
    int i;

    err = osal_sem_take(p_mda->sem_rx_history, OSAL_WAITING_FOREVER);
    osal_assert(err == OSAL_STATUS_SUCCESS);
    
    for (i=0; i< RX_HISTORY_NUM; i++) {
        p_mda->rx_history_table[i].life_time --;
    }

    osal_sem_release(p_mda->sem_rx_history);
}

static mda_history_t *find_history_record(mda_envar_t *p_mda, mda_msg_info_t *src_sta)
{
    int err;
    int i;
    mda_history_t *pos = NULL, *result = NULL;

    err = osal_sem_take(p_mda->sem_rx_history, OSAL_WAITING_FOREVER);
    osal_assert(err == OSAL_STATUS_SUCCESS);

    for (i=0; i< RX_HISTORY_NUM; i++) {
        pos = &p_mda->rx_history_table[i];
        if (pos->life_time > 0) {
            if ((memcmp(pos->msg.temorary_id, src_sta->temorary_id, MDA_TEMP_ID_LEN) == 0)
                && (pos->msg.msg_count == src_sta->msg_count)) {
                result = pos;
                break;
            }
        }
    }

    osal_sem_release(p_mda->sem_rx_history);

    return result;
}

static mda_history_t *add_history_record(mda_envar_t *p_mda, mda_msg_info_t *src_sta)
{
    int err;
    int i;
    uint32_t oldest = MAX_HISTORY_LIFE + 1;
    mda_history_t *pos = NULL, *new = NULL;

    err = osal_sem_take(p_mda->sem_rx_history, OSAL_WAITING_FOREVER);
    osal_assert(err == OSAL_STATUS_SUCCESS);

    for (i=0; i< RX_HISTORY_NUM; i++) {
        pos = &p_mda->rx_history_table[i];
        if (pos->life_time <= 0) {
            new = pos;
            break;
        }
        else if (pos->life_time <= oldest) {
            oldest = pos->life_time;
            new = pos;
        }
    }
    new->life_time = MAX_HISTORY_LIFE;
    memcpy(&new->msg, src_sta, sizeof(mda_msg_info_t));

    osal_sem_release(p_mda->sem_rx_history);

    return new;
}

static void history_handler(mda_envar_t *p_mda)
{
    if (p_mda->flag_history_timeout) {
        p_mda->flag_history_timeout = FALSE;
        update_history_record(p_mda);
    }
}

static void add_forward_list(mda_envar_t *p_mda, wnet_txbuf_t *txbuf, mda_msg_info_t *src_sta, int32_t forward_delay_time)
{
    int err;
    wnet_txbuf_t *ptx;
    mda_forward_t *p_fw, *pos;
    int32_t delay_time_total = 0, adj = 0;
    uint8_t inserted = 0; 


    p_fw = (mda_forward_t *)(WNET_TXBUF_DATA_PTR(txbuf) - sizeof(mda_forward_t));
    memcpy(&p_fw->msg, src_sta, sizeof(mda_msg_info_t));
    p_fw->delay_time = forward_delay_time;

    err = osal_sem_take(p_mda->sem_forward, OSAL_WAITING_FOREVER);
    osal_assert(err == OSAL_STATUS_SUCCESS);

    if (list_empty(&p_mda->forward_waiting_list)) {
        list_add_tail(&txbuf->list, &p_mda->forward_waiting_list);
	}
    else {
        /**
	     * 这里每个转发节点的延迟时间采用保存增量的方式，这样做的好处是在定时处理时
	       每次只需检查队列头一个或几个节点 ，而无需遍历整个队列
         */
        
    	list_for_each_entry(ptx, wnet_txbuf_t, &p_mda->forward_waiting_list, list){
            pos = (mda_forward_t *)(WNET_TXBUF_DATA_PTR(ptx) - sizeof(mda_forward_t));
            if (inserted == 0) {
                delay_time_total += pos->delay_time;
                if ((delay_time_total) > p_fw->delay_time) {
                    /* new node should insert the front of current pos */
                    adj = p_fw->delay_time - (delay_time_total - pos->delay_time); 
                    __list_add(&txbuf->list, ptx->list.prev, &ptx->list);
                    inserted = 1;
                }
                else {
                    p_fw->delay_time -= delay_time_total;
                    list_add(&txbuf->list, &ptx->list);
                }
            }

            //Need update the delay time of all remain nodes
            if (adj > 0) {
                pos->delay_time -= adj;
            }
        }
	}
	
    osal_sem_release(p_mda->sem_forward);
}

static void remove_from_forward_list(mda_envar_t *p_mda, wnet_txbuf_t *txbuf)
{
    int err;
    
    err = osal_sem_take(p_mda->sem_forward, OSAL_WAITING_FOREVER);
    osal_assert(err == OSAL_STATUS_SUCCESS);

    list_del(&txbuf->list);
    osal_sem_release(p_mda->sem_forward);

    wnet_release_txbuf(txbuf);
}


static wnet_txbuf_t *find_forward_list(mda_envar_t *p_mda, mda_msg_info_t *src_sta)
{
    wnet_txbuf_t *txbuf = NULL, *ptx;
    mda_forward_t *p_fw;

	list_for_each_entry(ptx, wnet_txbuf_t, &p_mda->forward_waiting_list, list) {
        p_fw = (mda_forward_t *)(WNET_TXBUF_DATA_PTR(ptx) - sizeof(mda_forward_t));
	    
        if ((memcmp(p_fw->msg.temorary_id, src_sta->temorary_id, MDA_TEMP_ID_LEN) == 0) \
            && (p_fw->msg.msg_count == src_sta->msg_count)) {
            txbuf = ptx;
            break;
        }
	}

	return txbuf;
}


static void forward_handler(mda_envar_t *p_mda)
{
    int err;
    wnet_txbuf_t *txbuf = NULL, *first;
    mda_forward_t *p_fw;

    err = osal_sem_take(p_mda->sem_forward, OSAL_WAITING_FOREVER);
    osal_assert(err == OSAL_STATUS_SUCCESS);

	if (!list_empty(&p_mda->forward_waiting_list)) {
		first = list_first_entry(&p_mda->forward_waiting_list, wnet_txbuf_t, list);
        p_fw = (mda_forward_t *)(WNET_TXBUF_DATA_PTR(first) - sizeof(mda_forward_t));

        p_fw->delay_time -= (1000/RT_TICK_PER_SECOND);
		if (p_fw->delay_time <= 0) {
    		list_del(&first->list);
    		txbuf = first;
		}
	}

    osal_sem_release(p_mda->sem_forward);

    if (txbuf) {
        rcp_send_forward_msg(txbuf);
    }
}

static void timer_forward_callback(void* parameter)
{
    mda_envar_t *p_mda = (mda_envar_t *)parameter;
    osal_sem_release(p_mda->sem_mda);
}

static void timer_rx_history_callback(void* parameter)
{
    mda_envar_t *p_mda = (mda_envar_t *)parameter;
    p_mda->flag_history_timeout = TRUE;
}

static void mda_thread_entry(void *parameter)
{
    int err;
    mda_envar_t *p_mda = (mda_envar_t *)parameter;

    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "%s: ---->\n", __FUNCTION__);

	while(1){
	    err = osal_sem_take(p_mda->sem_mda, OSAL_WAITING_FOREVER);
        if (err == OSAL_STATUS_SUCCESS){
            forward_handler(p_mda);
            history_handler(p_mda);
        }
        else{
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_ERROR, "%s:failed to take semaphore(%d)\n", \
                                __FUNCTION__, err);
        }
	}
}

/**
 * Modulue intialize
 */
void mda_init(void)
{
    mda_envar_t *p_mda = &cms_envar.mda;

    memset(p_mda, 0, sizeof(mda_envar_t));

    INIT_LIST_HEAD(&p_mda->forward_waiting_list);

    /* os object for mda */
	p_mda->task_mda = osal_task_create("t-mda",
                           mda_thread_entry, p_mda,
                           RT_THREAD_STACK_SIZE, RT_MDA_THREAD_PRIORITY);
    osal_assert(p_mda->task_mda != RT_NULL)

    p_mda->timer_forward = osal_timer_create("tm-mdaf",timer_forward_callback,p_mda,\
        1, RT_TIMER_FLAG_PERIODIC); 					
    osal_assert(p_mda->timer_forward != NULL);

    p_mda->timer_rx_history = osal_timer_create("tm-bp",timer_rx_history_callback,p_mda,\
        SECOND_TO_TICK(1),RT_TIMER_FLAG_PERIODIC); 					
    osal_assert(p_mda->timer_rx_history != RT_NULL);

    p_mda->sem_mda = osal_sem_create("s-mda", 0);
    osal_assert(p_mda->sem_mda != RT_NULL);

    p_mda->sem_forward = osal_sem_create("s-mdaf", 1);
    osal_assert(p_mda->sem_forward != RT_NULL);

    p_mda->sem_rx_history = osal_sem_create("s-mdah", 1);
    osal_assert(p_mda->sem_rx_history != RT_NULL);

    osal_timer_start(p_mda->timer_forward);
    osal_timer_start(p_mda->timer_rx_history);

	OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "module initial\n");
}


/*****************************************************************************
 @funcname: mda_handle
 @brief   : The API of multihop 
 @param   : mda_envar_t *p_mda        
 @param   : vam_stastatus_t *src_sta  
 @param   : vam_stastatus_t *pre_sta  
 @param   : uint8_t *pdata            
 @param   : uint32_t length           
 @return  : 
*****************************************************************************/
int mda_handle(mda_envar_t *p_mda, 
               mda_msg_info_t *src_sta, 
               mda_msg_info_t *pre_sta, 
               uint8_t *msg_data, 
               uint32_t msg_length)
{
    mda_history_t *p_hr;
    wnet_txbuf_t *txbuf;
   
    mda_forward_t *p_fw;
    int32_t forward_delay_time = 0;

    src_sta->left_hops -=  1;
    
    /*
     * Check if the frame has beed received before
     */
    p_hr = find_history_record(p_mda, src_sta);

    if (p_hr == NULL) {
        if (src_sta->left_hops <= 0) {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "No need to forward\n");
            return 0;
        }
        else {
            /** Received a new frame */
            forward_delay_time = alg_farest_node(p_mda, src_sta, pre_sta);
            if (forward_delay_time >= 0) {
                txbuf = rcp_create_forward_msg(src_sta->left_hops, msg_data, msg_length);
                if (txbuf == NULL) {
                    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "failed to create forward message\n");
                    return -1;
                }

                add_history_record(p_mda, src_sta);
                add_forward_list(p_mda, txbuf, src_sta, forward_delay_time);
            }
        }
    }
    else {
        txbuf = find_forward_list(p_mda, src_sta);
        if (txbuf) {
            /* Received the same frame */
            p_fw = (mda_forward_t *)(WNET_TXBUF_DATA_PTR(txbuf) - sizeof(mda_forward_t));

            if (src_sta->left_hops < (p_fw->msg.left_hops)) {
                remove_from_forward_list(p_mda, txbuf);                
            }
        }
    }
    
    return 0;
}

#endif  /* VAM_MDA_ENABLE */

