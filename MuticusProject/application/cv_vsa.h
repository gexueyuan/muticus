/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vsa.h
 @brief  : this file include the definition of vsa module
 @author : wangyifeng
 @history:
           2014-6-20    wangyifeng    Created file
           ...
******************************************************************************/
#ifndef __CV_VSA_H__
#define __CV_VSA_H__

/*****************************************************************************
 * definition of micro                                                       *
*****************************************************************************/


/*down to top*/
#define  HIGHWAY_MODE    0x0F9F
#define  MOUNTAIN_MODE   0x0FBF
#define  CITY_MODE       0x0FD7
#define  CUSTOM_MODE     0x019F



enum VSA_APP_ID
{
    VSA_ID_NONE = 0,
    VSA_ID_CRD,          /* Front close range danger. */ 
    VSA_ID_CRD_REAR,     /* Rear close range danger. */
    VSA_ID_OPS,
    VSA_ID_SIDE,
    VSA_ID_VBD,          /* Vehicle break down. */
    VSA_ID_EBD,          /* Vehicle emergency braking. */
    VSA_ID_VOT,          /* Vehicle overturned. */
    VSA_ID_EVA,
    VSA_ID_RSA,
    VSM_ID_END
};


typedef struct _adpcm{

    uint32_t addr;
    uint32_t size;
    uint8_t  channel;
    uint8_t  cmd;
    
}adpcm_t;

enum VOC_CMD{
    VOC_PLAY = 0,
    VOC_PAUSE,
    VOC_STOP
};

/* target classification locations */

enum VSA_TARGET_LOCATION
{
    POSITION_ERROR = 0,
    AHEAD_LEFT,        
    AHEAD,
    AHEAD_RIGHT,   
    RIGHT,
    BEHIND_RIGHT,
    BEHIND,
    BEHIND_LEFT,   
    LEFT
};

typedef struct _vsa_info{

    uint8_t pid[RCP_TEMP_ID_LEN];
    
    uint32_t vsa_location;

    float local_speed;

    float remote_speed;

    float relative_speed;

    uint32_t v_offset;

    uint32_t h_offset;

    int32_t linear_distance;

    uint32_t safe_distance;

    float dir;

    int8_t flag_dir;

    uint8_t jump_count;

    uint8_t vsa_id;

    uint8_t period;

}vsa_info_t;



typedef struct _vsa_position_node{

    list_head_t list;

    vsa_info_t vsa_position;
}vsa_position_node_t;


/*****************************************************************************
 * definition of struct                                                      *
*****************************************************************************/

typedef struct _vsa_config{
    /*
        General
    */
    

    uint8_t danger_detect_speed_threshold;  /* unit: km/h */
    uint16_t lane_dis;  /* unit:m, min accuracy :1m*/
    /*
        Close Range Danger function:
    */
    uint8_t crd_saftyfactor;  /* 1~10 */
    uint8_t crd_oppsite_speed;/*<=255:30km/h*/
    uint8_t crd_oppsite_rear_speed;/*<=255:30km/h*/
    uint8_t crd_rear_distance;/*<=255:10m*/
    /*
        Emergency Braking Danger function:
    */
    uint8_t ebd_mode;  /* 0 - disable, 1 - enable */
    uint8_t ebd_acceleration_threshold; /* unit:m/s2 */
    uint8_t ebd_alert_hold_time;  /* unit:s */
	
}vsa_config_t;


typedef struct _vsa_envar{
    
    uint32_t vsa_mode;
    
    /* working_param */
    vsa_config_t working_param;

    uint32_t gps_status;

    uint32_t alert_mask;
    uint32_t alert_pend;

    vam_stastatus_t local;
    vam_stastatus_t remote;

    adpcm_t adpcm_data;

	/*List head*/	
    list_head_t crd_list;
    list_head_t position_list;
    vsa_position_node_t position_node[VAM_NEIGHBOUR_MAXNUM];
    

    /* Vsa local process task. */
    osal_task_t  *task_vsa_l;

    /* Vsa remote process task. */
    osal_task_t  *task_vsa_r;

    osal_sem_t   *sem_vsa_proc;
    
    osal_queue_t *queue_vsa;

    osal_queue_t *queue_voc;

    osal_timer_t *timer_ebd_send;

    osal_timer_t *timer_position_prepro;

    osal_timer_t *timer_eva_stop;
                 
}vsa_envar_t;

typedef struct _vsa_crd_node{
    /* !!!DON'T modify it!!! */
    list_head_t list;

    uint8_t pid[RCP_TEMP_ID_LEN];  //temporary ID

    /* ccw_id = 1(VSA_ID_CRD) is cfcw,ccw_id = 2(VSA_ID_CRD_REAR) is crcw.*/
    uint8_t ccw_id;

	uint8_t ccw_cnt;

    /* private */
    uint16_t life;
    

}vsa_crd_node_t;



typedef int (*vsa_app_handler)(vsa_envar_t *p_vsa, void *p_msg);


/*****************************************************************************
 * declaration of global variables and functions                             *
*****************************************************************************/
void vsa_start(void);

osal_status_t vsa_add_event_queue
(
    vsa_envar_t *p_vsa, 
    
    uint16_t msg_id, 
    uint16_t msg_len, 
    uint32_t msg_argc,
    void    *msg_argv
);

uint32_t vsa_get_alarm(uint32_t vsa_id);
#endif /* __CV_VSA_H__ */

