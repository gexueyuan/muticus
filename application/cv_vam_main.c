/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vam_main.c
 @brief  : this file include main function of vehicle application middleware
           
 @author : wangyifeng
 @history:
           2014-6-19    wangyifeng    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "vam"
#include "cv_osal_dbg.h"


#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_wnet.h"
#include "nmea.h"




/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
#define BSM_SEND_PERIOD_DEFAULT      MS_TO_TICK(100)//SECOND_TO_TICK(1)
#define BSM_PAUSE_HOLDTIME_DEFAULT   SECOND_TO_TICK(5)
#define BSM_GPS_LIFE_DEFAULT         SECOND_TO_TICK(5)
#define NEIGHBOUR_LIFE_ACCUR         SECOND_TO_TICK(1)
#define EVAM_SEND_PERIOD_DEFAULT     MS_TO_TICK(50)

extern void timer_send_bsm_callback(void* parameter);
extern void timer_bsm_pause_callback(void* parameter);
extern void timer_send_evam_callback(void* parameter);
extern void timer_neigh_time_callback(void* parameter);
extern void timer_gps_life_callback(void* parameter);

vam_envar_t *p_vam_envar;

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/



void vam_main_proc(vam_envar_t *p_vam, sys_msg_t *p_msg)
{
    switch(p_msg->id)
    {
        case VAM_MSG_START:
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "%s: VAM_MSG_START\n", __FUNCTION__);

            p_vam->flag |= VAM_FLAG_RX;
            osal_timer_start(p_vam->timer_neighbour_life);
            
            if (p_vam->working_param.bsm_boardcast_mode != BSM_BC_MODE_DISABLE)
            {
                p_vam->flag |= VAM_FLAG_TX_BSM;
                vsm_start_bsm_broadcast(p_vam);
            }
            
            break; 
        }
       
        case VAM_MSG_STOP:
        {
            if (p_vam->flag & VAM_FLAG_TX_BSM)
            {
                vsm_stop_bsm_broadcast(p_vam);
            }

            p_vam->flag &= ~(VAM_FLAG_RX|VAM_FLAG_TX_BSM);
            osal_timer_stop(p_vam->timer_neighbour_life);
            
            break;
        }

        case VAM_MSG_RCPTX:
        {
            if (p_msg->argc == RCP_MSG_ID_BSM)
            {
                rcp_send_bsm(p_vam);
                OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "send bsm\n");
            }
            if (p_msg->argc == RCP_MSG_ID_EVAM)
            {
                rcp_send_evam(p_vam);
                OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_TRACE, "send evam\n");
            }
            if (p_msg->argc == RCP_MSG_ID_RSA)
            {
                rcp_send_rsa(p_vam);
            }

            break; 
        }

        case VAM_MSG_RCPRX:
        {
            rcp_parse_msg(p_vam, (wnet_rxinfo_t *)p_msg->argv, (uint8_t *)p_msg->argc, p_msg->len);

            wnet_release_rxbuf(WNET_RXBUF_PTR(p_msg->argv));
            
            break;
        }


        case VAM_MSG_NEIGH_TIMEOUT:
        {
            vam_update_sta(p_vam);
            break;
        }

        case VAM_MSG_GPSDATA:
        {
            lip_gps_proc(p_vam, (uint8_t *)p_msg->argv, p_msg->len);
            break;
        }
            
        default:
        {
            break;
        }
            
    }
}


void vam_thread_entry(void *parameter)
{
    rt_err_t err;
    sys_msg_t *p_msg = NULL;
    vam_envar_t *p_vam = (vam_envar_t *)parameter;


    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "%s: ---->\n", __FUNCTION__);

    while(1)
    {
        err = osal_queue_recv(p_vam->queue_vam, &p_msg, RT_WAITING_FOREVER);
        if (err == RT_EOK)
        {
            vam_main_proc(p_vam, p_msg);
            osal_free(p_msg);
        }
        else
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_ERROR, "%s: osal_queue_recv error [%d]\n",__FUNCTION__, err);
        }
    }
}


int vam_add_event_queue
(
    vam_envar_t *p_vam, 
    uint16_t msg_id, 
    uint16_t msg_len, 
    uint32_t msg_argc,
    void   * msg_argv
)
{
    int err = OSAL_STATUS_NOMEM;
    sys_msg_t *p_msg = NULL;


    p_msg = osal_malloc(sizeof(sys_msg_t));
    if (p_msg) 
    {
        p_msg->id = msg_id;
        p_msg->len = msg_len;
        p_msg->argc = msg_argc;
        p_msg->argv = msg_argv;
        
        err = osal_queue_send(p_vam->queue_vam, p_msg);

        if (err != OSAL_STATUS_SUCCESS) 
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, msg_id);
            osal_free(p_msg);                   
        }
    }
    else
    {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, msg_id); 
    }

    return err;
}


int vam_add_event_queue_2(vam_envar_t *p_vam, void *p_msg)
{
    int err = 0;

    
    err = osal_queue_send(p_vam->queue_vam, p_msg);
    if (err != OSAL_STATUS_SUCCESS)
    {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "%s: failed=[%d], msg=%04x\n", __FUNCTION__, err, ((sys_msg_t *)p_msg)->id);
    }

    return err;
}


/*****************************************************************************
 @funcname: vam_init
 @brief   : vam module initial
 @param   : None
 @return  : 
*****************************************************************************/

void vam_init(void)
{
    int i;
    vam_envar_t *p_vam = &cms_envar.vam;
    uint8_t zero_pid[RCP_TEMP_ID_LEN] = {0};

    p_vam_envar = p_vam;

    
    memset(p_vam, 0, sizeof(vam_envar_t));
    memcpy(&p_vam->working_param, &cms_param.vam, sizeof(vam_config_t));
    if (0 == memcmp(cms_param.pid, zero_pid, RCP_TEMP_ID_LEN)){
        for (i=0; i<RCP_TEMP_ID_LEN; i++){
            p_vam->local.pid[i] = des(RCP_TEMP_ID_LEN-1-i);
        }
    }
    else {
        memcpy(p_vam->local.pid, cms_param.pid, RCP_TEMP_ID_LEN);
    }

    
    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "PID: %02x %02x %02x %02x\r\n", \
                                           p_vam->local.pid[0], p_vam->local.pid[1], p_vam->local.pid[2], p_vam->local.pid[3]);

    /* Initial neibhour list head and free list head structure. */
    INIT_LIST_HEAD(&p_vam->neighbour_list);
    INIT_LIST_HEAD(&p_vam->sta_free_list);

    /* Initial list semaphore. */
    p_vam->sem_sta = osal_sem_create("s-sta", 1);
    osal_assert(p_vam->sem_sta != RT_NULL);

    /* Add all the list node to free list. */
    for(i = 0; i< VAM_NEIGHBOUR_MAXNUM; i++)
    {
        list_add_tail(&p_vam->remote[i].list, &p_vam->sta_free_list);
    }


     /* os object for vam */
    p_vam->queue_vam = osal_queue_create("q-vam", VAM_QUEUE_SIZE);
    osal_assert(p_vam->queue_vam != RT_NULL);

   
    p_vam->task_vam = osal_task_create("t-vam", vam_thread_entry, p_vam, RT_VAM_THREAD_STACK_SIZE, RT_VAM_THREAD_PRIORITY);
    osal_assert(p_vam->task_vam != RT_NULL)


    p_vam->timer_send_bsm = osal_timer_create("tm-sb", timer_send_bsm_callback, p_vam, BSM_SEND_PERIOD_DEFAULT, RT_TIMER_FLAG_PERIODIC); 					
    osal_assert(p_vam->timer_send_bsm != NULL);


    p_vam->timer_bsm_pause = osal_timer_create("tm-bp", timer_bsm_pause_callback, p_vam, BSM_PAUSE_HOLDTIME_DEFAULT, RT_TIMER_FLAG_ONE_SHOT); 					
    osal_assert(p_vam->timer_bsm_pause != RT_NULL);

    p_vam->timer_send_evam = osal_timer_create("tm-se",timer_send_evam_callback, p_vam, EVAM_SEND_PERIOD_DEFAULT, RT_TIMER_FLAG_PERIODIC); 					
    osal_assert(p_vam->timer_send_evam != RT_NULL);

    p_vam->timer_gps_life = osal_timer_create("tm-gl", timer_gps_life_callback, p_vam, BSM_GPS_LIFE_DEFAULT, RT_TIMER_FLAG_ONE_SHOT); 					
    osal_assert(p_vam->timer_gps_life != RT_NULL);

    p_vam->timer_neighbour_life = osal_timer_create("tm-nl", timer_neigh_time_callback, p_vam, NEIGHBOUR_LIFE_ACCUR, RT_TIMER_FLAG_PERIODIC); 					
    osal_assert(p_vam->timer_neighbour_life != RT_NULL);



    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "module initial\n");
}


/*****************************************************************************
 @funcname: vam_deinit
 @brief   : vam module unstall
 @param   : None
 @return  : 
*****************************************************************************/
void vam_deinit()
{
	OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "module deinit\n");
}


void dump_pos(vam_stastatus_t *p_sta)
{
    char str[64];

    osal_printf("------------sta---------\n");
    osal_printf("PID:%02x-%02x-%02x-%02x\n", p_sta->pid[0], p_sta->pid[1]\
                                          , p_sta->pid[2], p_sta->pid[3]);
    sprintf(str,"%f", p_sta->pos.lat);
    osal_printf("pos.lat:%s\n", str);
    sprintf(str,"%f", p_sta->pos.lon);
    osal_printf("pos.lon:%s\n", str);
    sprintf(str,"%f", p_sta->pos.elev);
    osal_printf("pos.elev:%s\n", str);
    sprintf(str,"%f", p_sta->pos.accu);
    osal_printf("pos.accu:%s\n", str);
    sprintf(str,"%f", p_sta->dir);
    osal_printf("pos.heading:%s\n", str);
    sprintf(str,"%f", p_sta->speed);
    osal_printf("pos.speed:%s\n", str);
    osal_printf("------------end---------\n");
}

