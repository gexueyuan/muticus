/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vsa_main.c
 @brief  : this file realize the function of vehicle safty application
 @author : wangyifeng
 @history:
           2014-6-19    wangyifeng    Created file
           ...
******************************************************************************/
//#pragma O3

#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "vsa"
#include "cv_osal_dbg.h"
OSAL_DEBUG_ENTRY_DEFINE(vsa)


#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_vsa.h"
#include "key.h"
#include "math.h"
#include "arm_math.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
void space_null(void)
{
}

#define VSA_TIMER_PERIOD         SECOND_TO_TICK(1)
#define VSA_EBD_SEND_PERIOD      SECOND_TO_TICK(5)
#define VSA_POS_PERIOD           MS_TO_TICK(100)
#define DIRECTION_DIVIDE         22.5f
//#define PI 3.1415926f

#define VSA_MSG_PROC    (VSA_MSG_BASE+1)
#define CCW_DEBOUNCE     5

float getDistanceVer2(float lat1, float lng1, float lat2, float lng2);
static int vbd_judge(vsa_envar_t *p_vsa);
static int ebd_judge(vsa_envar_t *p_vsa);

extern void test_comm(void);
extern uint8_t vam_get_gps_status(void);
extern void param_get(void);

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

/*****************************************************************************
 @funcname: vsa_position_classify
 @brief   : describe target's postion
 @param   : None
 @return  : 
*****************************************************************************/
uint32_t  vsa_position_classify(const vam_stastatus_t *local, const vam_stastatus_t *remote,
	                                  vam_pos_data *pos_data, float *delta_offset)
{

    float lat1, lng1, lat2, lng2;
    float angle, delta;

    /* reference point */
    lat1 = local->pos.lat;
    lng1 = local->pos.lon;

    /* destination point */
    lat2 = remote->pos.lat;
    lng2 = remote->pos.lon;


    angle = pos_data->angle;

    /* calculate the relative angle against north, clockwise  */
    if (lat2 >= lat1){
    /* north */
        if (lng2 >= lng1){
        /* easts */
            //equal
        }
        else{
            angle = 360-angle;
        }
    }
    else{
    /* south */
        if (lng2 >= lng1){
        /* easts */
            angle = 180-angle;
        }
        else{
            angle = 180+angle;
        }
    }

    /* calculate the angle detra between local front and remote position  */
    if (angle > local->dir){
        delta = angle - local->dir;
    }
    else {
        delta = 360.0f-(local->dir - angle);
    }

    if ((delta >360.0f)||(delta <0.0f)){
        return POSITION_ERROR;
    }
/****************
    if (delta > 180){
        delta = 360 - delta;
    }
**********************/
    
    *delta_offset = delta;  
    
		/*divide posiotion to 8 pieces*/
    if((delta > 15*DIRECTION_DIVIDE)||(delta <= DIRECTION_DIVIDE)){
       return AHEAD;
    }
    else if ((delta > DIRECTION_DIVIDE)&&(delta <= 3*DIRECTION_DIVIDE)){
       return AHEAD_RIGHT;
    }
    else if ((delta > 3*DIRECTION_DIVIDE)&&(delta <= 5*DIRECTION_DIVIDE)){
       return RIGHT;
    }
    else if ((delta > 5*DIRECTION_DIVIDE)&&(delta <= 7*DIRECTION_DIVIDE)){
       return BEHIND_RIGHT;
    }
    else if ((delta > 7*DIRECTION_DIVIDE)&&(delta <= 9*DIRECTION_DIVIDE)){
       return BEHIND;
    }
    else if ((delta > 9*DIRECTION_DIVIDE)&&(delta <= 11*DIRECTION_DIVIDE)){
       return BEHIND_LEFT;
    }
    else if ((delta > 11*DIRECTION_DIVIDE)&&(delta <= 13*DIRECTION_DIVIDE)){
       return LEFT;
    }
    else if ((delta > 13*DIRECTION_DIVIDE)&&(delta <= 15*DIRECTION_DIVIDE)){
       return AHEAD_LEFT;
    }
    else {
        return POSITION_ERROR;
    }
}

/*****************************************************************************
 @funcname: vsa_safe_distance
 @brief   : calculate the safe diatance
 @param   : position:ahead or behind
 @return  : safe distance with the remote point  
*****************************************************************************/

uint32_t vsa_safe_distance(int32_t position,vam_stastatus_t local,vam_stastatus_t remote)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;

    if (position < 0){
        return (int32_t)p_vsa->working_param.crd_rear_distance;
    }

    if (local.speed > remote.speed){
       return(int32_t)((local.speed*2.0f - remote.speed)*p_vsa->working_param.crd_saftyfactor*1000.0f/3600.0f);
    }
    else{
       return 0; 
    }


}


/*****************************************************************************
 @funcname: vsa_set_period
 @brief   : set the period of vsa process
 @param   : None
 @return  : 
*****************************************************************************/
void  vsa_set_period(vsa_info_t* vsa_node,vam_stastatus_t* remote)
{
    vsa_info_t* node_info;

    uint32_t ldistance;
    
    node_info = vsa_node;

    ldistance = abs(node_info->linear_distance);

    remote->cnt = 0;
#if 0
    if(node_info->relative_speed <= 5.0f)
        remote->cnt ++;
    if(node_info->local_speed <= 50.0f)
        remote->cnt ++;
    if(ldistance >= 500)
        remote->cnt ++;

    if((node_info->relative_speed <= 5.0f)&&(node_info->local_speed <= 20.0f)&&(ldistance >= 500))    
        remote->cnt = 10;
   // osal_printf("cnt is %d\n",remote->cnt);
#endif
}


/*****************************************************************************
 @funcname: vsa_preprocess_pos
 @brief   : process  information of points in the neighbour list in a period
 @param   : void
 @return  : void 
*****************************************************************************/

int32_t  vsa_preprocess_pos(void)
{
    vam_stastatus_t local_status;  
    vam_stastatus_t remote_status = { 0 };
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    vsa_position_node_t *p_pnt = NULL;
    vam_pos_data temp_data;
    int8_t i = 0;
    uint8_t peer_pid[VAM_NEIGHBOUR_MAXNUM][RCP_TEMP_ID_LEN];
    uint32_t peer_count;

    char strbuf[64] = {0};
    float temp_delta = 0;


    if (0x1 & g_dbg_print_type){
        test_comm();
    }
    vam_get_all_peer_pid(peer_pid,VAM_NEIGHBOUR_MAXNUM,&peer_count);

    if (peer_count != 0) {
        vam_get_local_current_status(&local_status);
        for (i = 0;i < peer_count;i++) {
            vam_get_peer_current_status(peer_pid[i],&remote_status); /*!!查询不到的风险!!*/  
            
            p_pnt = &p_vsa->position_node[i];

            memcpy(p_pnt->vsa_position.pid,remote_status.pid,RCP_TEMP_ID_LEN);

            if(remote_status.cnt == 0){

                temp_data = vsm_get_data(&local_status,&remote_status);

                p_pnt->vsa_position.vsa_location = vsa_position_classify(&local_status,&remote_status,&temp_data,&temp_delta);

                p_pnt->vsa_position.local_speed = local_status.speed;

                p_pnt->vsa_position.remote_speed = remote_status.speed;

                p_pnt->vsa_position.relative_speed = local_status.speed - remote_status.speed;                

                p_pnt->vsa_position.linear_distance = (int32_t)vsm_get_pos(&local_status,&remote_status,&temp_data);//

                p_pnt->vsa_position.v_offset = (uint32_t)(p_pnt->vsa_position.linear_distance*arm_cos_f32(temp_delta*PI/180.0f));

                p_pnt->vsa_position.h_offset = (uint32_t)(p_pnt->vsa_position.linear_distance*arm_sin_f32(temp_delta*PI/180.0f));

                p_pnt->vsa_position.safe_distance = vsa_safe_distance(p_pnt->vsa_position.linear_distance,local_status,remote_status);
                    
                p_pnt->vsa_position.dir = remote_status.dir;

                p_pnt->vsa_position.flag_dir = vam_get_peer_relative_dir(&local_status,&remote_status);

                vsa_set_period(&(p_pnt->vsa_position),&remote_status);
                
                if (0x2 & g_dbg_print_type){
                    memset(strbuf, 0x0, sizeof(strbuf));
                    sprintf(strbuf, "%3.6f", temp_delta);
                    osal_printf("(%02X %02X %02X %02X), %d, %d, %d, %d, %s, %lu\r\n", \
                        p_pnt->vsa_position.pid[0],p_pnt->vsa_position.pid[1], p_pnt->vsa_position.pid[2],p_pnt->vsa_position.pid[3],\
                        p_pnt->vsa_position.vsa_location, p_pnt->vsa_position.linear_distance, p_pnt->vsa_position.v_offset,\
                        p_pnt->vsa_position.h_offset,strbuf,p_pnt->vsa_position.safe_distance);
            }
                
            }
            else{

                remote_status.cnt--;               
            }
            vam_set_peer_cnt(remote_status.pid,remote_status.cnt);                
    						

	    }
    }
    else
        return 0; 
    return peer_count;
}

/*****************************************************************************
 @funcname: timer_preprocess_pos_callback
 @brief   : preprocess for vsa module,information from neighbourlist
 @param   : None
 @return  : 
*****************************************************************************/
void  timer_preprocess_pos_callback( void *parameter )
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;        
    osal_sem_release(p_vsa->sem_vsa_proc);
   // osal_printf("reset!!\n\n");
   // NVIC_SystemReset();
}

/*****************************************************************************
 @funcname: vsa_find_pn
 @brief   : find position node in position link list
 @param   : None
 @return  : 
*****************************************************************************/
vsa_position_node_t *vsa_find_pn(vsa_envar_t *p_vsa, uint8_t *temporary_id)
{
    vsa_position_node_t *p_pn = NULL, *pos;

    list_for_each_entry(pos, vsa_position_node_t, &p_vsa->position_list, list){
        if (memcmp(pos->vsa_position.pid, temporary_id, RCP_TEMP_ID_LEN) == 0){
            p_pn = pos;
            break;
        }
    }
    /* not found */
    if (p_pn == NULL){
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "vsa position node need updating!\n\n");
                        
    }
    return p_pn;
}


void vsa_local_status_update(void* parameter)
{
    vam_stastatus_t *p_sta = (vam_stastatus_t *)parameter;
    vsa_envar_t *p_vsa = &cms_envar.vsa;

    memcpy(&p_vsa->local, p_sta, sizeof(vam_stastatus_t));
}

void vsa_bsm_status_update(void *parameter)
{
    vam_stastatus_t *p_sta = (vam_stastatus_t *)parameter;

    if (p_sta){
        if (!(cms_envar.sys.status&(1<<HI_OUT_BSM_UPDATE))){
            sys_add_event_queue(&cms_envar.sys,SYS_MSG_BSM_UPDATE, 0, HI_OUT_BSM_UPDATE, NULL);
        }
    }
    else{
        sys_add_event_queue(&cms_envar.sys,SYS_MSG_BSM_UPDATE, 0, HI_OUT_BSM_NONE, NULL);
    }
}

void vsa_gps_status_update(void *parameter)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;

    p_vsa->gps_status = (uint32_t)parameter;
    sys_add_event_queue(&cms_envar.sys, \
                                SYS_MSG_GPS_UPDATE, 0, (uint32_t)parameter, NULL);
}



void vsa_receive_alarm_update(void *parameter)
{

    vam_stastatus_t *p_sta = (vam_stastatus_t *)parameter;
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    uint16_t peer_alert;
    vam_get_peer_alert_status(&peer_alert);
    memcpy(&p_vsa->remote, p_sta, sizeof(vam_stastatus_t));

    if ((peer_alert&VAM_ALERT_MASK_VBD)&&(vbd_judge(p_vsa))){
        if (!(p_vsa->alert_pend & (1<<VSA_ID_VBD))){
            vsa_add_event_queue(p_vsa, VSA_MSG_ACC_RC, 1,VAM_ALERT_MASK_VBD,NULL);
        }
    }
    else{
        if (p_vsa->alert_pend & (1<<VSA_ID_VBD)){
            vsa_add_event_queue(p_vsa, VSA_MSG_ACC_RC, 0,VAM_ALERT_MASK_VBD,NULL);
        }
    }

    if ((peer_alert&VAM_ALERT_MASK_EBD)&&(ebd_judge(p_vsa))){
        if (!(p_vsa->alert_pend & (1<<VSA_ID_EBD))){
            vsa_add_event_queue(p_vsa, VSA_MSG_EEBL_RC, 1,VAM_ALERT_MASK_EBD,NULL);
        }
    }
    else{
        if (p_vsa->alert_pend & (1<<VSA_ID_EBD)){
            vsa_add_event_queue(p_vsa, VSA_MSG_EEBL_RC, 0,VAM_ALERT_MASK_EBD,NULL);
        }
    } 
    
    if (peer_alert&VAM_ALERT_MASK_VOT){
        if (!(p_vsa->alert_pend & (1<<VSA_ID_VOT))){
            vsa_add_event_queue(p_vsa, VSA_MSG_ACC_RC, 1,VAM_ALERT_MASK_VOT,NULL);
        }
    }
    else{
        if (p_vsa->alert_pend & (1<<VSA_ID_VOT)){
            vsa_add_event_queue(p_vsa, VSA_MSG_ACC_RC, 0,VAM_ALERT_MASK_VOT,NULL);
        }
    }
   
}

void vsa_receive_rsa_update(void *parameter)
{
    vam_rsa_evt_info_t *param = (vam_rsa_evt_info_t*)parameter;
    vsa_envar_t *p_vsa = &cms_envar.vsa;

    if(param->rsa_mask)
        vsa_add_event_queue(p_vsa, VSA_MSG_XXX_RC, 0,0,NULL);

}

void vsa_eebl_broadcast_update(void *parameter)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    vsa_add_event_queue(p_vsa, VSA_MSG_EEBL_BC, 0,0,NULL);
}

void vsa_start(void)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;    
    vam_set_event_handler(VAM_EVT_LOCAL_UPDATE, vsa_local_status_update);
    vam_set_event_handler(VAM_EVT_PEER_UPDATE, vsa_bsm_status_update);
    vam_set_event_handler(VAM_EVT_PEER_ALARM, vsa_receive_alarm_update);
    vam_set_event_handler(VAM_EVT_GPS_STATUS, vsa_gps_status_update);
    vam_set_event_handler(VAM_EVT_GSNR_EBD_DETECT, vsa_eebl_broadcast_update);
    vam_set_event_handler(VAM_EVT_RSA_UPDATE, vsa_receive_rsa_update);
    osal_timer_start(p_vsa->timer_position_prepro);
    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "%s: --->\n", __FUNCTION__);

    if(vam_get_gps_status()){
        
        vsa_gps_status_update((void *)1);

    }

        

}

/*****************************************************************************
 @funcname: vsa_search_warning
 @brief   : this fucntion can detect whether the warning exist
 @param   : uint32_t warning_id  
 @return  : 0-no warning 1-have this warning
*****************************************************************************/
uint32_t vsa_search_warning(uint32_t warning_id)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    vsa_crd_node_t *pos = NULL;

    if (list_empty(&p_vsa->crd_list)){
        return 0;
    }

    list_for_each_entry(pos,vsa_crd_node_t,&p_vsa->crd_list,list){
        if(pos->ccw_id == warning_id){
            return 1;
        }
    }
    return 0;
}

/*****************************************************************************
 @funcname: vsa_send_ccw_warning
 @brief   : send ccw warning to sys module
 @param   : uint32_t warning_id  
 @return  : 
*****************************************************************************/

void vsa_send_ccw_warning(uint32_t warning_id,uint16_t distance)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    /* danger is detected */    
    if (p_vsa->alert_pend & (1<<warning_id)){
    /* do nothing */    
    }
    else{
    /* inform system to start alert */
        p_vsa->alert_pend |= (1<<warning_id);
        sys_add_event_queue(&cms_envar.sys, \
                            SYS_MSG_START_ALERT, distance, warning_id, NULL);
    }

}

void send_alert(uint32_t warning_id,uint16_t distance)
{

    vsa_send_ccw_warning(warning_id,distance);

}
FINSH_FUNCTION_EXPORT(send_alert,param1:type param2:distance);

/*****************************************************************************
 @funcname: vsa_cancel_ccw_warning
 @brief   : cancel ccw warning to sys module
 @param   : uint32_t warning_id  
 @return  : 
*****************************************************************************/

void vsa_cancel_ccw_warning(uint32_t warning_id)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    uint32_t alert_pend;
    uint32_t list_search;
    alert_pend = p_vsa->alert_pend;
    list_search = vsa_search_warning(warning_id);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"pend is %d,research is %d\n\n",alert_pend,list_search);
    if ((p_vsa->alert_pend & (1<<warning_id))&&(!vsa_search_warning(warning_id))){
    /* inform system to stop alert */
        p_vsa->alert_pend &= ~(1<<warning_id);
        sys_add_event_queue(&cms_envar.sys, \
                            SYS_MSG_STOP_ALERT, 0, warning_id, NULL);
    }

}

/*****************************************************************************
 @funcname: ccw_add_list
 @brief   : judge  if add the node into ccwlist
 @param   : uint32_t warning_id  ,p_pnt: collection of node's  position infromation
 @return  : 
*****************************************************************************/

static int ccw_add_list(uint32_t warning_id,vsa_position_node_t *p_pnt)
{
    
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    vsa_crd_node_t *p_crd = NULL,*pos = NULL;
    rt_bool_t  ccw_flag = RT_FALSE;

    if (warning_id == 0){
        return warning_id;
    }
    
    if (p_vsa->alert_mask & (1<<warning_id)){

        if (list_empty(&p_vsa->crd_list)){
            p_crd = (vsa_crd_node_t*)osal_malloc(sizeof(vsa_crd_node_t));
            memcpy(p_crd->pid,p_pnt->vsa_position.pid,RCP_TEMP_ID_LEN);
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"list empty,create it \n");
            p_crd->ccw_id = warning_id;
            p_crd->ccw_cnt = 1;
            list_add(&p_crd->list,&p_vsa->crd_list);
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,\
                                "pid(%02X %02X %02X %02X) create %lu list,dis=%d\n\n",p_crd->pid[0],p_crd->pid[1],p_crd->pid[2],\
                                p_crd->pid[3],warning_id,p_pnt->vsa_position.linear_distance);
        }
        else{
            list_for_each_entry(pos,vsa_crd_node_t,&p_vsa->crd_list,list){
                if (memcmp(p_pnt->vsa_position.pid,pos->pid,RCP_TEMP_ID_LEN) == 0){
                    if(pos->ccw_id == warning_id){//have this id and vsa warning do nor change
                        if(pos->ccw_cnt >= CCW_DEBOUNCE){                          
                            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,\
                                                "pid(%02X %02X %02X %02X) warning id=%d  dis=%d\n\n",p_crd->pid[0],p_crd->pid[1],p_crd->pid[2],\
                                                p_crd->pid[3],warning_id,p_pnt->vsa_position.linear_distance);
                            vsa_send_ccw_warning(warning_id,p_pnt->vsa_position.v_offset);
                        }
                        else{
                            pos->ccw_cnt++;
                        }
                        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"ccw_cnt++ is %d\n",pos->ccw_cnt);
                    }
                    else{//this didn't happen,list have this id ,but warning changed,one id to another id
                        //pos->ccw_id = warning_id;
                        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"VSA error!!\n\n");
                    }
                    ccw_flag = RT_FALSE;
                    break;
                }   
                else{
                    ccw_flag = RT_TRUE;
                }
            }
        }
        if(ccw_flag ){
            p_crd = (vsa_crd_node_t*)osal_malloc(sizeof(vsa_crd_node_t));
            memcpy(p_crd->pid,p_pnt->vsa_position.pid,RCP_TEMP_ID_LEN);
            p_crd->ccw_id = warning_id;                        
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"list exist,add it \n");
            p_crd->ccw_cnt = 1;
            list_add(&p_crd->list,&p_vsa->crd_list);
            OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,\
                                "pid(%02X %02X %02X %02X) join in %lu list,dis=%d\n\n",p_crd->pid[0],p_crd->pid[1],p_crd->pid[2],\
                                p_crd->pid[3],warning_id,p_pnt->vsa_position.linear_distance);
        }   

    }
    return 0;
}


static int ccw_del_list(uint32_t warning_id,vsa_position_node_t *p_pnt)
{

    vsa_envar_t *p_vsa = &cms_envar.vsa;
    vsa_crd_node_t *pos = NULL;

      
    list_for_each_entry(pos,vsa_crd_node_t,&p_vsa->crd_list,list){
        
        if (memcmp(p_pnt->vsa_position.pid,pos->pid,RCP_TEMP_ID_LEN) == 0){
            if (pos->ccw_cnt){
                OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"cnt--\n");
                pos->ccw_cnt--;
            }
            else{
                if (list_empty(&p_vsa->crd_list)){

                    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"list empty error!\n");
                    vsa_cancel_ccw_warning(pos->ccw_id);
                }
                OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"list del beg!\n");
                list_del(&pos->list);                    
                OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_TRACE,"list del end!\n");
                osal_free((vsa_crd_node_t*)list_entry(&pos->list,vsa_crd_node_t,list));                   
                OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,\
                                    "pid(%02X %02X %02X %02X) left \n\n",pos->pid[0],pos->pid[1],pos->pid[2],\
                                    pos->pid[3]);
                vsa_cancel_ccw_warning(pos->ccw_id);
            }
            break;
        }   
                
    }                                 
    return 0;
}

void timer_ebd_send_callback(void* parameter)
{
    vam_cancel_alert(VAM_ALERT_MASK_EBD);
    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Cancel Emergency braking \n\n");
    sys_add_event_queue(&cms_envar.sys,SYS_MSG_ALARM_CANCEL, 0, VSA_ID_EBD, NULL);
                                            
}

static int ebd_judge(vsa_envar_t *p_vsa)
{
    int32_t dis_actual;
    vsa_position_node_t *p_node;
    p_node = vsa_find_pn(p_vsa,p_vsa->remote.pid);

    /* put the beginning only in order to output debug infomations */
    dis_actual = p_node->vsa_position.linear_distance;

    if (p_node->vsa_position.local_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    }

    if (p_node->vsa_position.remote_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    } 

    if (p_node->vsa_position.flag_dir < 0){

       return 0;
    }

    /* remote is behind of local */
    if (dis_actual <= 0){
        return 0;
    }
    return 1;

}

static int vbd_judge(vsa_envar_t *p_vsa)
{
    
#if 0
    vsa_position_node_t *p_node;    
    int32_t dis_actual;
    p_node = vsa_find_pn(p_vsa,p_vsa->remote.pid);

    /* put the beginning only in order to output debug infomations */
    dis_actual = p_node->vsa_position.linear_distance;

    if (p_node->vsa_position.local_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    }
/*
    if (p_node->vsa_position.remote_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    } 
*/
    /**************horizontal distance compare with lane*******************/
    if(p_node->vsa_position.h_offset > p_vsa->working_param.lane_dis){

        return 0;    
    }

    if (p_node->vsa_position.flag_dir < 0){

        return 0;
    }

    /* remote is behind of local */
    if (dis_actual <= 0){
        return 0;
    }
    return 1;
    
#else
    
    return 1;
    
#endif
}
/*****************************************************************************
 @funcname: cfcw_judge
 @brief   : check and judge the close range danger of vehicles
 @param   : vsa_envar_t *p_vsa  
 @return  : 
            0 - no alert
            1 - alert
*****************************************************************************/
static int cfcw_judge(vsa_position_node_t *p_node)
{
    int32_t dis_actual, dis_alert;
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    /* put the beginning only in order to output debug infomations */
    dis_actual = p_node->vsa_position.linear_distance;
    dis_alert = p_node->vsa_position.safe_distance;//(int32_t)((p_vsa->local.speed*2.0f - p_vsa->remote.speed)*p_vsa->working_param.crd_saftyfactor*1000.0f/3600.0f);
    /* end */

    if (p_node->vsa_position.local_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    }
/*
    if (p_node->vsa_position.remote_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    }   
 */   
    if (p_node->vsa_position.flag_dir < 0){
        
        return 0;
    }

    /**************horizontal distance compare with lane*******************/
    if(p_node->vsa_position.h_offset > p_vsa->working_param.lane_dis){

        return 0;    
    }
        
    if (p_node->vsa_position.local_speed <= (p_node->vsa_position.remote_speed +\
        p_vsa->working_param.crd_oppsite_speed)){       
        return 0;
    }



    /* remote is behind of local */
    if (dis_actual <= 0){
        
        return 0;
    }
#if 0
    if (p_vsa->alert_pend & (1<<VSA_ID_CRD))
        {
            if (dis_actual < -10)
                return 0;
        }
    else
        {
            if (dis_actual <0)        
                return 0;
        }
#endif
    
    if (dis_actual > dis_alert){ 
        return 0;
    }
  //  OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,\
                     //   "pid(%02X %02X %02X %02X) dis=%d\n",p_node->vsa_position.pid[0],p_node->vsa_position.pid[1],\
                     //   p_node->vsa_position.pid[2],p_node->vsa_position.pid[3],p_node->vsa_position.linear_distance);
    return VSA_ID_CRD;

}





/*****************************************************************************
 @funcname: crcw_judge
 @brief   : check and judge the close range danger of vehicles
 @param   : vsa_envar_t *p_vsa  
 @return  : 
            0 - no alert
            1 - alert
*****************************************************************************/
static int crcw_judge(vsa_position_node_t *p_node)
{
    int32_t dis_actual, dis_alert;
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    /* put the beginning only in order to output debug infomations */
    dis_actual = p_node->vsa_position.linear_distance;
    dis_alert = p_vsa->working_param.crd_rear_distance;//p_node->vsa_position.safe_distance;//(int32_t)((p_vsa->local.speed*2.0f - p_vsa->remote.speed)*p_vsa->working_param.crd_saftyfactor*1000.0f/3600.0f);
    /* end */

    if (p_node->vsa_position.local_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    }

    if (p_node->vsa_position.remote_speed <= p_vsa->working_param.danger_detect_speed_threshold){
        
        return 0;
    }   
    
    if (p_node->vsa_position.flag_dir < 0){
        
        return 0;
    }
#if 0
    if((p_node->vsa_position.vsa_location >= AHEAD_LEFT)&&
        (p_node->vsa_position.vsa_location <= AHEAD_RIGHT)){
        
    if (p_node->vsa_position.local_speed <= (p_node->vsa_position.remote_speed +\
                                            p_vsa->working_param.crd_oppsite_speed)){
        
        return 0;
    }



    /* remote is behind of local */
    if (dis_actual <= 0){
        
        return 0;
    }

    if (dis_actual > dis_alert){
        return 0;
    }
    return VSA_ID_CRD;
    }
    else if((p_node->vsa_position.vsa_location >= BEHIND_LEFT)&&
        (p_node->vsa_position.vsa_location <= BEHIND_RIGHT)){
            

    return VSA_ID_CRD_REAR;
    }    
#endif
    if ((p_node->vsa_position.local_speed + p_vsa->working_param.crd_oppsite_speed) >= p_node->vsa_position.remote_speed){
        
        return 0;
    }

    /*local  is behind of remote */
#if 0    
    if (p_vsa->alert_pend & (1<<VSA_ID_CRD_REAR))
        {
            if (dis_actual >15)            
            return 0;
    }
    else
        {
            if (dis_actual >0)
                return 0;
        }
#endif
    if (dis_actual >0){
        return 0;
    }

    if ((-dis_actual) > dis_alert){
        return 0;
    }
    return VSA_ID_CRD_REAR;
}

uint32_t vsa_get_alarm(uint32_t vsa_id)
{
    vsa_envar_t *p_vsa = &cms_envar.vsa;
    if(vsa_id == VSA_ID_NONE){
        return p_vsa->alert_pend;
    }
    else{
        return p_vsa->alert_pend & (1<<vsa_id);
    }

}


static int vsa_manual_broadcast_proc(vsa_envar_t *p_vsa, void *arg)
{
  int err = 1;  /* '1' represent is not handled. */ 
  sys_msg_t *p_msg = (sys_msg_t *)arg;
  
  if(p_msg->argc){
      vam_active_alert(VAM_ALERT_MASK_VBD);
      sys_add_event_queue(&cms_envar.sys, \
                    SYS_MSG_ALARM_ACTIVE, 0, VSA_ID_VBD, NULL);
      OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Active Vihicle Break Down Alert\n");
  }   
  else {   
      vam_cancel_alert(VAM_ALERT_MASK_VBD);
      sys_add_event_queue(&cms_envar.sys, \
                    SYS_MSG_ALARM_CANCEL, 0, VSA_ID_VBD, NULL);
      OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Cancel Vihicle Break Down Alert\n");
  }
  
  
  return err;
}


static int vsa_eebl_broadcast_proc(vsa_envar_t *p_vsa, void *arg)
{
        vam_active_alert(VAM_ALERT_MASK_EBD);

        /* Start the emergency brake down alert send cancel timer. */
        rt_timer_stop(p_vsa->timer_ebd_send);
        rt_timer_start(p_vsa->timer_ebd_send);
        
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Detect Emergency braking \n\n");
        sys_add_event_queue(&cms_envar.sys, \
                                    SYS_MSG_ALARM_ACTIVE, 0, VSA_ID_EBD, NULL);
        return 0;

}
static int vsa_auto_broadcast_proc(vsa_envar_t *p_vsa, void *arg)
{

    return 0;
}

static int vsa_cfcw_alarm_proc(vsa_envar_t *p_vsa, void *arg)
{
    int count_neighour;
    int vsa_id;
    vsa_position_node_t *pos_pnt = NULL;
    
    count_neighour = *((int*)arg);
    pos_pnt = &p_vsa->position_node[count_neighour];
    vsa_id = cfcw_judge(pos_pnt);

    return vsa_id;
}

static int vsa_crcw_alarm_proc(vsa_envar_t *p_vsa, void *arg)
{
    int count_neighour;
    int vsa_id;
    vsa_position_node_t *pos_pnt = NULL;
    
    count_neighour = *((int*)arg);

    pos_pnt = &p_vsa->position_node[count_neighour];
    vsa_id = crcw_judge(pos_pnt);

    return vsa_id;

}

static int vsa_opposite_alarm_proc(vsa_envar_t *p_vsa, void *arg)
{
/*
  static int cfcw_count = 0;
  if(cfcw_count++ >40 ){
      osal_printf("oppo  occur!!\n\n");
      cfcw_count = 0;
  }
*/

  return 0;
}
static int vsa_side_alarm_proc(vsa_envar_t *p_vsa, void *arg)
{


  return 0;
}


static int vsa_accident_recieve_proc(vsa_envar_t *p_vsa, void *arg)
{
    sys_msg_t* p_msg = (sys_msg_t*)arg;
    

    switch(p_msg->argc){

    case VAM_ALERT_MASK_VBD:
        if (p_msg->len){
            p_vsa->alert_pend |= (1<<VSA_ID_VBD);
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Vehicle Breakdown Alert!!! Id:%02X%02X%02X%02X\n",\
            p_vsa->remote.pid[0],p_vsa->remote.pid[1],p_vsa->remote.pid[2],p_vsa->remote.pid[3]);
             
            sys_add_event_queue(&cms_envar.sys, \
                              SYS_MSG_START_ALERT, 0, VSA_ID_VBD, NULL);
        }
        else if (p_msg->len == 0){
                  /* inform system to stop alert */
            p_vsa->alert_pend &= ~(1<<VSA_ID_VBD);
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Vehicle Breakdown Alert!!! Id:%02X%02X%02X%02X\n",\
            p_vsa->remote.pid[0],p_vsa->remote.pid[1],p_vsa->remote.pid[2],p_vsa->remote.pid[3]);
            sys_add_event_queue(&cms_envar.sys, \
                              SYS_MSG_STOP_ALERT, 0, VSA_ID_VBD, NULL);
        } 
    break;

    default:
    break;



    }
  
    return 1;

}

static int vsa_eebl_recieve_proc(vsa_envar_t *p_vsa, void *arg)
{
    sys_msg_t* p_msg = (sys_msg_t*)arg;
    
    switch(p_msg->argc){

    case VAM_ALERT_MASK_EBD:
        if(p_msg->len){
            p_vsa->alert_pend |= (1<<VSA_ID_EBD);
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Emergency Vehicle  Alert start!!! Id:%02X%02X%02X%02X\n",\
            p_vsa->remote.pid[0],p_vsa->remote.pid[1],p_vsa->remote.pid[2],p_vsa->remote.pid[3]);
             
            sys_add_event_queue(&cms_envar.sys, \
                              SYS_MSG_START_ALERT, 0, VSA_ID_EBD, NULL);
        }
        else if (p_msg->len == 0){
                  /* inform system to stop alert */
            p_vsa->alert_pend &= ~(1<<VSA_ID_EBD);
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Emergency Vehicle  Alert cancel!!! Id:%02X%02X%02X%02X\n",\
            p_vsa->remote.pid[0],p_vsa->remote.pid[1],p_vsa->remote.pid[2],p_vsa->remote.pid[3]);
            sys_add_event_queue(&cms_envar.sys, \
                              SYS_MSG_STOP_ALERT, 0, VSA_ID_EBD, NULL);
        }
         
        break;
    default:
    break;
        }
    return 0;
}

static int vsa_x_recieve_proc(vsa_envar_t *p_vsa, void *arg)
{


  return 0;
}

static int vsa_xx_recieve_proc(vsa_envar_t *p_vsa, void *arg)
{


  return 0;
}

static int vsa_xxx_recieve_proc(vsa_envar_t *p_vsa, void *arg)
{


  return 0;
}
vsa_app_handler vsa_app_handler_tbl[] = {

    vsa_manual_broadcast_proc,    /* Set or Reset vehicle break down alert. */
    vsa_eebl_broadcast_proc,      /* Set vehicle emergancy brake down alert. */
    vsa_auto_broadcast_proc,      /* Reserved now. */

    vsa_cfcw_alarm_proc,
    vsa_crcw_alarm_proc,
    vsa_opposite_alarm_proc,
    vsa_side_alarm_proc,

    vsa_accident_recieve_proc,
    vsa_eebl_recieve_proc,
    vsa_x_recieve_proc,
    vsa_xx_recieve_proc,
    vsa_xxx_recieve_proc,
    
//ccw_proc,
   // alarm_update_proc,
   // vbd_broadcast_proc,
  //  ebd_broadcast_proc,
  //  vsa_default_proc,
  //  NULL
};

void vsa_base_proc(void *parameter)
{
    int count_neighour;
    int i;
    int vsa_id;
    vsa_envar_t *p_vsa = (vsa_envar_t *)parameter;
    
    while(1){
        osal_sem_take(p_vsa->sem_vsa_proc,RT_WAITING_FOREVER);
#if 0 //gps detect
        if(!p_vsa->gps_status)
            continue;
#endif
        count_neighour = vsa_preprocess_pos();

        if(( count_neighour < 0)||(count_neighour > VAM_NEIGHBOUR_MAXNUM)){
            continue;
        }
        /*if bsm lost,after 15s,neighour list is empty,cancel all alert*/
        if(count_neighour == 0){
            if (p_vsa->alert_pend & (1<<VSA_ID_CRD)){
                p_vsa->alert_pend &= ~(1<<VSA_ID_CRD);
                sys_add_event_queue(&cms_envar.sys, SYS_MSG_STOP_ALERT, 0, VSA_ID_CRD, NULL);
            }
            if (p_vsa->alert_pend & (1<<VSA_ID_CRD_REAR))
            {
                p_vsa->alert_pend &= ~(1<<VSA_ID_CRD_REAR);
                sys_add_event_queue(&cms_envar.sys, SYS_MSG_STOP_ALERT, 0, VSA_ID_CRD_REAR, NULL);
            }

            continue;
        }

        for(i = 0;i < count_neighour;i++){
            
            if(vsa_app_handler_tbl[VSA_MSG_CFCW_ALARM-VSA_MSG_PROC]){    
                vsa_id = vsa_app_handler_tbl[VSA_MSG_CFCW_ALARM-VSA_MSG_PROC](p_vsa,&i);
            }

            if((!vsa_id)&&(vsa_app_handler_tbl[VSA_MSG_CRCW_ALARM-VSA_MSG_PROC])){    
                vsa_id = vsa_app_handler_tbl[VSA_MSG_CRCW_ALARM-VSA_MSG_PROC](p_vsa,&i);
            }

            if((!vsa_id)&&(vsa_app_handler_tbl[VSA_MSG_OPPOSITE_ALARM-VSA_MSG_PROC])){  
                vsa_id = vsa_app_handler_tbl[VSA_MSG_OPPOSITE_ALARM-VSA_MSG_PROC](p_vsa,&i);
            }

            if((!vsa_id)&&(vsa_app_handler_tbl[VSA_MSG_SIDE_ALARM-VSA_MSG_PROC])){    
                vsa_id = vsa_app_handler_tbl[VSA_MSG_SIDE_ALARM-VSA_MSG_PROC](p_vsa,&i);
            }

            if(vsa_id){
                ccw_add_list(vsa_id,&p_vsa->position_node[i]);
            }
            else{ 
                ccw_del_list(vsa_id,&p_vsa->position_node[i]);                 
            }

        }
            

    }

}

void vsa_thread_entry(void *parameter)
{
    rt_err_t err;
    sys_msg_t* p_msg = NULL;
    vsa_envar_t *p_vsa = (vsa_envar_t *)parameter;


    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "%s: ---->\n", __FUNCTION__);

    while(1)
    {
        err = osal_queue_recv(p_vsa->queue_vsa, &p_msg, RT_WAITING_FOREVER);
        if (err == OSAL_STATUS_SUCCESS)
        {    
            if(vsa_app_handler_tbl[p_msg->id - VSA_MSG_PROC] != NULL)
            {
                err = vsa_app_handler_tbl[p_msg->id - VSA_MSG_PROC](p_vsa, p_msg);
            }        
            osal_free(p_msg);            
        }
        else
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "%s: rt_mq_recv error [%d]\n", __FUNCTION__, err);          
            osal_free(p_msg);
        }      
    }
}

osal_status_t vsa_add_event_queue(vsa_envar_t *p_vsa, 
                             uint16_t msg_id, 
                             uint16_t msg_len, 
                             uint32_t msg_argc,
                             void    *msg_argv)
{
    int err = OSAL_STATUS_NOMEM;
    sys_msg_t *p_msg;

    p_msg = osal_malloc(sizeof(sys_msg_t));
    if (p_msg){
        p_msg->id = msg_id;
        p_msg->len = msg_len;
        p_msg->argc = msg_argc;
        p_msg->argv = msg_argv;
        err = osal_queue_send(p_vsa->queue_vsa,p_msg);
    }

    if (err != OSAL_STATUS_SUCCESS){
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "%s: failed=[%d], msg=%04x\n",\
                                   __FUNCTION__, err, msg_id);
        osal_free(p_msg);
    }

    return err;
}


/*****************************************************************************
 @funcname: vsa_init
 @brief   : vsa module initial
 @param   : None
 @return  : 
*****************************************************************************/
void vsa_init()
{
    int i = 0;

    vsa_envar_t *p_vsa = &cms_envar.vsa;

    
    
    memset(p_vsa, 0, sizeof(vsa_envar_t));
    memcpy(&p_vsa->working_param, &cms_param.vsa, sizeof(vsa_config_t));
    
    p_vsa->vsa_mode = ~(mode_get());//CUSTOM_MODE;
    
    INIT_LIST_HEAD(&p_vsa->crd_list);    
    INIT_LIST_HEAD(&p_vsa->position_list);    


    for(i = 0;i< VAM_NEIGHBOUR_MAXNUM;i++){
        list_add_tail(&p_vsa->position_node[i].list, &p_vsa->position_list);
    }
    
    for(i = 0;i < sizeof(vsa_app_handler_tbl)/sizeof(vsa_app_handler);i++){

        if(p_vsa->vsa_mode&(1<<i)){
            
            vsa_app_handler_tbl[i] = NULL;

        }            
    }

    p_vsa->alert_mask = (1<<VSA_ID_CRD)|(1<<VSA_ID_CRD_REAR)|(1<<VSA_ID_VBD)|(1<<VSA_ID_EBD);

    /* os object for vsa */
    p_vsa->queue_vsa = osal_queue_create("q-vsa",  VSA_QUEUE_SIZE);
    osal_assert(p_vsa->queue_vsa != NULL);
    

    p_vsa->timer_ebd_send = osal_timer_create("tm-ebd",timer_ebd_send_callback,NULL,\
        VSA_EBD_SEND_PERIOD,RT_TIMER_FLAG_ONE_SHOT);                     
    osal_assert(p_vsa->timer_ebd_send != NULL);

    p_vsa->timer_position_prepro = osal_timer_create("tm-pos",\
        timer_preprocess_pos_callback,NULL,VSA_POS_PERIOD,RT_TIMER_FLAG_PERIODIC);
    osal_assert(p_vsa->timer_position_prepro != NULL);

    p_vsa->sem_vsa_proc = osal_sem_create("sem-vsa",0);
    osal_assert(p_vsa->sem_vsa_proc != NULL);
    
    p_vsa->task_vsa_l = osal_task_create("t-vsa-l",
                           vsa_base_proc, p_vsa,
                           RT_VSA_THREAD_STACK_SIZE, RT_VSA_THREAD_PRIORITY);
    osal_assert(p_vsa->task_vsa_l != NULL);
    
    p_vsa->task_vsa_r = osal_task_create("t-vsa-r",
                           vsa_thread_entry, p_vsa,
                           RT_VSA_THREAD_STACK_SIZE, RT_VSA_THREAD_PRIORITY);
    osal_assert(p_vsa->task_vsa_r != NULL);



    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "module initial\n");
                             
}

/*****************************************************************************
 @funcname: vsa_deinit
 @brief   : vsa module unstall
 @param   : None
 @return  : 
*****************************************************************************/
void vsa_deinit()
{
    OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "vsa module initial\n");
}

