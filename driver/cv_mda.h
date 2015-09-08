/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_mda.h
 @brief  : the definition of mda env and info
 @author : wanglei
 @history:
           2015-1-21    wanglei    Created file
           ...
******************************************************************************/
#ifndef __CV_MDA_H__
#define __CV_MDA_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

#define V2V_COMMUNICATION_RANGE      (350.0f)
#define MDA_FORWARD_DELAY_TIME_UNIT  (70.0f)

#define RX_HISTORY_NUM   16
#define MAX_HISTORY_LIFE (10)  /* second */
#define MDA_TEMP_ID_LEN RCP_TEMP_ID_LEN

typedef struct _mda_sta_info {
    uint8_t temorary_id[MDA_TEMP_ID_LEN];
    uint8_t forward_id[MDA_TEMP_ID_LEN];
    uint8_t msg_count;
    uint8_t left_hops;
} mda_msg_info_t;

typedef struct _mda_history {
    list_head_t list;

    mda_msg_info_t msg;
    int32_t life_time;
} mda_history_t;

typedef struct _mda_forward {
    mda_msg_info_t msg;
    int32_t delay_time;
} mda_forward_t;


typedef struct _mda_envar {

    int flag_history_timeout; 

    osal_task_t *task_mda;
    osal_timer_t *timer_forward;
    osal_timer_t *timer_rx_history;
    osal_sem_t *sem_mda;
    osal_sem_t *sem_forward;
    osal_sem_t *sem_rx_history;
    list_head_t forward_waiting_list;
    mda_history_t rx_history_table[RX_HISTORY_NUM];
} mda_envar_t;

int mda_handle(mda_envar_t *p_mda, 
               mda_msg_info_t *src_sta, 
               mda_msg_info_t *pre_sta, 
               uint8_t *msg_data, 
               uint32_t msg_length);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __CV_MDA_H__ */
