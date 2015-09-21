/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_param.c
 @brief  : this file include the system parameter management
 @author : wangyifeng
 @history:
           2014-6-19    wangyifeng    Created file
           2014-7-28    gexueyuan     modified
           ...
******************************************************************************/

#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "param"
#include "cv_osal_dbg.h"
OSAL_DEBUG_ENTRY_DEFINE(param)


#include "cv_osal.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_vsa.h"
#include "cv_cms_def.h"
#include "cv_sys_param.h"



extern  int drv_fls_erase(uint32_t sector);
extern  int drv_fls_read(uint32_t flash_address, uint8_t *p_databuf, uint32_t length);
extern  int drv_fls_write(uint32_t flash_address, uint8_t *p_databuf, uint32_t length);

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

cfg_param_t cms_param, *p_cms_param;

uint8_t param_init_words[] = "Vanet-param1";

uint16_t param_mode;
/*****************************************************************************
* implementation of functions                                               *
*****************************************************************************/

void load_default_param_custom(cfg_param_t *param)
{
    memset(param, 0 , sizeof(cfg_param_t));
    /******************** VAM *********************/
    param->vam.bsm_hops = 1; 
    param->vam.bsm_boardcast_mode = 2;  /* 0 - disable, 1 - auto, 2 - fixed period */
    param->vam.bsm_boardcast_saftyfactor = 5;  /* 1~10 */
    param->vam.bsm_pause_mode = 1;  /* 0 - disable, 1 - enable */
    param->vam.bsm_pause_hold_time = 5;  /* unit:s */
    param->vam.bsm_boardcast_period = 100;  /* 100~3000, unit:ms, min accuracy :10ms */
    
    param->vam.evam_hops = 1; 
    param->vam.evam_broadcast_type = 2;
    param->vam.evam_broadcast_peroid = 50;

    /******************** VSA *********************/
    param->vsa.danger_detect_speed_threshold = 60;  /* unit: km/h */
    param->vsa.lane_dis = 25;  /* unit:m, min accuracy :1m */
    
    param->vsa.crd_saftyfactor = 4;  /* 1~10 */
    param->vsa.crd_oppsite_speed = 20;/* <=255:30km/h*/
    param->vsa.crd_oppsite_rear_speed = 15;/* <=255:30km/h*/
    param->vsa.crd_rear_distance = 60;/*<=255:20m*/
    
    param->vsa.ebd_mode = 1;  /* 0 - disable, 1 - enable */
    param->vsa.ebd_acceleration_threshold = 3; /* unit:m/s2 */
    param->vsa.ebd_alert_hold_time = 5;  /* unit:s */

    param->gsnr.gsnr_cal_step = 0;
    param->gsnr.gsnr_cal_thr = 4;
    param->gsnr.gsnr_ebd_thr = -30;
    param->gsnr.gsnr_ebd_cnt = 2;
    
    param->gsnr.AcceV_x = 0;
    param->gsnr.AcceV_y = 0;
    param->gsnr.AcceV_z= 0;
    param->gsnr.AcceAhead_x= 0;
    param->gsnr.AcceAhead_y = 0;
    param->gsnr.AcceAhead_z = 0;

    param->wnet.channel = 13;
    param->wnet.txrate = 1;

    param->voc.fg_volume = 8;
    param->voc.bg_volume = 4;
    param->voc.speed = 5;

}


void load_default_param_highway(cfg_param_t *param)
{
    memset(param, 0 , sizeof(cfg_param_t));

    /******************** VAM *********************/
    param->vam.bsm_hops = 1; 
    param->vam.bsm_boardcast_mode = 2;  /* 0 - disable, 1 - auto, 2 - fixed period */
    param->vam.bsm_boardcast_saftyfactor = 5;  /* 1~10 */
    param->vam.bsm_pause_mode = 1;  /* 0 - disable, 1 - enable */
    param->vam.bsm_pause_hold_time = 5;  /* unit:s */
    param->vam.bsm_boardcast_period = 100;  /* 100~3000, unit:ms, min accuracy :10ms */
    
    param->vam.evam_hops = 1; 
    param->vam.evam_broadcast_type = 2;
    param->vam.evam_broadcast_peroid = 50;

    /******************** VSA *********************/
    param->vsa.danger_detect_speed_threshold = 60;  /* unit: km/h */
    param->vsa.lane_dis = 25;  /* unit:m, min accuracy :1m */
    
    param->vsa.crd_saftyfactor = 4;  /* 1~10 */
    param->vsa.crd_oppsite_speed = 20;/* <=255:30km/h*/
    param->vsa.crd_oppsite_rear_speed = 15;/* <=255:30km/h*/
    param->vsa.crd_rear_distance = 60;/*<=255:20m*/
    
    param->vsa.ebd_mode = 1;  /* 0 - disable, 1 - enable */
    param->vsa.ebd_acceleration_threshold = 3; /* unit:m/s2 */
    param->vsa.ebd_alert_hold_time = 5;  /* unit:s */

    param->gsnr.gsnr_cal_step = 0;
    param->gsnr.gsnr_cal_thr = 4;
	param->gsnr.gsnr_ebd_thr = -30;
    param->gsnr.gsnr_ebd_cnt = 2;
    
    param->gsnr.AcceV_x = 0;
    param->gsnr.AcceV_y = 0;
    param->gsnr.AcceV_z= 0;
    param->gsnr.AcceAhead_x= 0;
    param->gsnr.AcceAhead_y = 0;
    param->gsnr.AcceAhead_z = 0;

    param->wnet.channel = 13;
    param->wnet.txrate = 1;

    param->voc.fg_volume = 8;
    param->voc.bg_volume = 4;
    param->voc.speed = 5;
    

}


void load_default_param_mountain(cfg_param_t *param)
{
    memset(param, 0 , sizeof(cfg_param_t));

    /******************** VAM *********************/
    param->vam.bsm_hops = 1; 
    param->vam.bsm_boardcast_mode = 2;  /* 0 - disable, 1 - auto, 2 - fixed period */
    param->vam.bsm_boardcast_saftyfactor = 5;  /* 1~10 */
    param->vam.bsm_pause_mode = 1;  /* 0 - disable, 1 - enable */
    param->vam.bsm_pause_hold_time = 5;  /* unit:s */
    param->vam.bsm_boardcast_period = 100;  /* 100~3000, unit:ms, min accuracy :10ms */
    
    param->vam.evam_hops = 1; 
    param->vam.evam_broadcast_type = 2;
    param->vam.evam_broadcast_peroid = 50;

    /******************** VSA *********************/
    param->vsa.danger_detect_speed_threshold = 60;  /* unit: km/h */
    param->vsa.lane_dis = 25;  /*  unit:m, min accuracy :1m */
    
    param->vsa.crd_saftyfactor = 4;  /* 1~10 */
    param->vsa.crd_oppsite_speed = 20;/* <=255:30km/h*/
    param->vsa.crd_oppsite_rear_speed = 15;/* <=255:30km/h*/
    param->vsa.crd_rear_distance = 60;/*<=255:20m*/
    
    param->vsa.ebd_mode = 1;  /* 0 - disable, 1 - enable */
    param->vsa.ebd_acceleration_threshold = 3; /* unit:m/s2 */
    param->vsa.ebd_alert_hold_time = 5;  /* unit:s */

    param->gsnr.gsnr_cal_step = 0;
    param->gsnr.gsnr_cal_thr = 4;
	param->gsnr.gsnr_ebd_thr = -30;
    param->gsnr.gsnr_ebd_cnt = 2;
    
    param->gsnr.AcceV_x = 0;
    param->gsnr.AcceV_y = 0;
    param->gsnr.AcceV_z= 0;
    param->gsnr.AcceAhead_x= 0;
    param->gsnr.AcceAhead_y = 0;
    param->gsnr.AcceAhead_z = 0;

    param->wnet.channel = 13;
    param->wnet.txrate = 1;

    param->voc.fg_volume = 8;
    param->voc.bg_volume = 4;
    param->voc.speed = 5;
    

}

void load_default_param_city(cfg_param_t *param)
{
    memset(param, 0 , sizeof(cfg_param_t));

    /******************** VAM *********************/
    param->vam.bsm_hops = 1; 
    param->vam.bsm_boardcast_mode = 2;  /* 0 - disable, 1 - auto, 2 - fixed period */
    param->vam.bsm_boardcast_saftyfactor = 5;  /* 1~10 */
    param->vam.bsm_pause_mode = 1;  /* 0 - disable, 1 - enable */
    param->vam.bsm_pause_hold_time = 5;  /* unit:s */
    param->vam.bsm_boardcast_period = 100;  /* 100~3000, unit:ms, min accuracy :10ms */
    
    param->vam.evam_hops = 1; 
    param->vam.evam_broadcast_type = 2;
    param->vam.evam_broadcast_peroid = 50;

    /******************** VSA *********************/
    param->vsa.danger_detect_speed_threshold = 60;  /* unit: km/h */
    param->vsa.lane_dis = 25;  /* unit:m, min accuracy :1m */
    
    param->vsa.crd_saftyfactor = 4;  /* 1~10 */
    param->vsa.crd_oppsite_speed = 20;/* <=255:30km/h*/
    param->vsa.crd_oppsite_rear_speed = 15;/* <=255:30km/h*/
    param->vsa.crd_rear_distance = 60;/*<=255:20m*/
    
    param->vsa.ebd_mode = 1;  /* 0 - disable, 1 - enable */
    param->vsa.ebd_acceleration_threshold = 3; /* unit:m/s2 */
    param->vsa.ebd_alert_hold_time = 5;  /* unit:s */

    param->gsnr.gsnr_cal_step = 0;
    param->gsnr.gsnr_cal_thr = 4;
	param->gsnr.gsnr_ebd_thr = -30;
    param->gsnr.gsnr_ebd_cnt = 2;
    
    param->gsnr.AcceV_x = 0;
    param->gsnr.AcceV_y = 0;
    param->gsnr.AcceV_z= 0;
    param->gsnr.AcceAhead_x= 0;
    param->gsnr.AcceAhead_y = 0;
    param->gsnr.AcceAhead_z = 0;

    param->wnet.channel = 13;
    param->wnet.txrate = 1;

    param->voc.fg_volume = 8;
    param->voc.bg_volume = 4;
    param->voc.speed = 5;
    

}

void load_default_param(cfg_param_t *param)
{
    memset(param, 0 , sizeof(cfg_param_t));

    /******************ID************************/
    param->pid[0] = 0x00;
    param->pid[1] = 0x00;    
    param->pid[2] = 0x00;
    param->pid[3] = 0x00;
    /******************** VAM *********************/
    param->vam.bsm_hops = 1; 
    param->vam.bsm_boardcast_mode = 2;  /* 0 - disable, 1 - auto, 2 - fixed period */
    param->vam.bsm_boardcast_saftyfactor = 5;  /* 1~10 */
    param->vam.bsm_pause_mode = 1;  /* 0 - disable, 1 - enable */
    param->vam.bsm_pause_hold_time = 5;  /* unit:s */
    param->vam.bsm_boardcast_period = 100;  /* 100~3000, unit:ms, min accuracy :10ms */
    
    param->vam.evam_hops = 1; 
    param->vam.evam_broadcast_type = 2;
    param->vam.evam_broadcast_peroid = 50;

    /******************** VSA *********************/
    param->vsa.danger_detect_speed_threshold = 60;  /* unit: km/h */
    param->vsa.lane_dis = 25;  /* unit:m, min accuracy :1m */
    
    param->vsa.crd_saftyfactor = 4;  /* 1~10 */
    param->vsa.crd_oppsite_speed = 20;/* <=255:30km/h*/
    param->vsa.crd_oppsite_rear_speed = 15;/* <=255:30km/h*/
    param->vsa.crd_rear_distance = 60;/*<=255:20m*/
    
    param->vsa.ebd_mode = 1;  /* 0 - disable, 1 - enable */
    param->vsa.ebd_acceleration_threshold = 3; /* unit:m/s2 */
    param->vsa.ebd_alert_hold_time = 5;  /* unit:s */

    param->gsnr.gsnr_cal_step = 0;
    param->gsnr.gsnr_cal_thr = 4;
    param->gsnr.gsnr_ebd_thr = -30;
    param->gsnr.gsnr_ebd_cnt = 2;
    
    param->gsnr.AcceV_x = 0;
    param->gsnr.AcceV_y = 0;
    param->gsnr.AcceV_z= 0;
    param->gsnr.AcceAhead_x= 0;
    param->gsnr.AcceAhead_y = 0;
    param->gsnr.AcceAhead_z = 0;

    param->wnet.channel = 13;
    param->wnet.txrate = 1;

    param->voc.fg_volume = 8;
    param->voc.bg_volume = 4;
    param->voc.speed = 5;
    

}

uint32_t get_param_addr(uint16_t mode)
{
    uint32_t param_addr;
  
    if(param_mode == CUSTOM_MODE)
        param_addr = PARAM_ADDR_CUSTOM;
    else if(param_mode == HIGHWAY_MODE)
        param_addr = PARAM_ADDR_HIGHWAY;
    else if(param_mode == MOUNTAIN_MODE)
        param_addr = PARAM_ADDR_MOUNTAIN;
    else if(param_mode == CITY_MODE)
        param_addr = PARAM_ADDR_CITY;

    return param_addr;

}


uint8_t get_param_num(uint16_t mode)
{
    uint8_t param_no;
  
    if(param_mode == CUSTOM_MODE){
        param_no = 0;
    }
    else if(param_mode == HIGHWAY_MODE){
        param_no = 1;
    }
    else if(param_mode == MOUNTAIN_MODE){
        param_no = 2;
    }
    else if(param_mode == CITY_MODE){
        param_no = 3;
    }

    return param_no;

}

uint16_t  mode_get(void)
{
    uint16_t mode;
    uint8_t get_str;
    //rt_kprintf("----------------------mode---------------------\n");
    drv_fls_read(PARAM_MODE_ADDR,(uint8_t*)&mode,sizeof(uint16_t));

    if(mode == CUSTOM_MODE){
        get_str = 0;
    }
    else if(mode == HIGHWAY_MODE){
        get_str = 1;
    }
    else if(mode == MOUNTAIN_MODE){
        get_str = 2;
    }
    else if(mode == CITY_MODE){
        get_str = 3;
    }
    else{ 
        OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"mode reading error!!\n");
    }
    
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"mode value = %04X,mode is %s\n", mode,mode_string[get_str]);

    return mode;

}
FINSH_FUNCTION_EXPORT(mode_get, get mode);

void mode_set(uint16_t mode)
{
    cfg_param_t all_param[4];
    param_mode = mode;
    
    drv_fls_read(PARAM_ADDR_CUSTOM,(uint8_t*)&all_param[0],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_HIGHWAY,(uint8_t*)&all_param[1],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_MOUNTAIN,(uint8_t*)&all_param[2],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_CITY,(uint8_t*)&all_param[3],sizeof(cfg_param_t));



    drv_fls_erase(PARAM_SECTOR);
    drv_fls_write(PARAM_FLAG_ADDR,param_init_words,sizeof(param_init_words));    
    drv_fls_write(PARAM_MODE_ADDR,(uint8_t *)&param_mode,sizeof(uint16_t));
    drv_fls_write(PARAM_ADDR_CUSTOM,(uint8_t*)&all_param[0],sizeof(cfg_param_t));
    drv_fls_write(PARAM_ADDR_HIGHWAY,(uint8_t*)&all_param[1],sizeof(cfg_param_t));
    drv_fls_write(PARAM_ADDR_MOUNTAIN,(uint8_t*)&all_param[2],sizeof(cfg_param_t));
    drv_fls_write(PARAM_ADDR_CITY,(uint8_t*)&all_param[3],sizeof(cfg_param_t));
    rt_kprintf("mode = %d  write in flash\n", param_mode);

}
FINSH_FUNCTION_EXPORT(mode_set, set mode by hex);

void mode_change(uint8_t mode_num)
{
    uint16_t mode_setting;
    
    if(mode_num == 1){
        mode_setting = CUSTOM_MODE;
    }
    else if(mode_num == 2){
        mode_setting = HIGHWAY_MODE;
    }
    else if(mode_num == 3){
        mode_setting = MOUNTAIN_MODE;
    }
    else if(mode_num == 4){
        mode_setting = CITY_MODE;
    }

    mode_set(mode_setting);

}
FINSH_FUNCTION_EXPORT(mode_change, set mode by num:1-custom;2-highway;3-mountain;4-city);

void load_param_from_fl(void)
{
    uint32_t param_addr;
    p_cms_param = &cms_param;
    
    if(param_mode == CUSTOM_MODE){
        param_addr = PARAM_ADDR_CUSTOM;
    }
    else if(param_mode == HIGHWAY_MODE){
        param_addr = PARAM_ADDR_HIGHWAY;
    }
    else if(param_mode == MOUNTAIN_MODE){
        param_addr = PARAM_ADDR_MOUNTAIN;
    }
    else if(param_mode == CITY_MODE){
        param_addr = PARAM_ADDR_CITY;
    }
    
    drv_fls_read(param_addr,(uint8_t *)p_cms_param,sizeof(cfg_param_t));
    

}

void  write_def_param(void)
{
    cfg_param_t  flash_param;

    param_mode = CUSTOM_MODE;
    
    drv_fls_erase(PARAM_SECTOR);
    drv_fls_write(PARAM_FLAG_ADDR,param_init_words,sizeof(param_init_words));
    drv_fls_write(PARAM_MODE_ADDR,(uint8_t *)&param_mode,sizeof(uint16_t));

    
    load_default_param_custom(&flash_param);
    drv_fls_write(PARAM_ADDR_CUSTOM,(uint8_t *)&flash_param,sizeof(cfg_param_t));

    
    memset(&flash_param,0,sizeof(cfg_param_t));
    load_default_param_highway(&flash_param);
    drv_fls_write(PARAM_ADDR_HIGHWAY,(uint8_t *)&flash_param,sizeof(cfg_param_t));

    
    memset(&flash_param,0,sizeof(cfg_param_t));
    load_default_param_mountain(&flash_param);
    drv_fls_write(PARAM_ADDR_MOUNTAIN,(uint8_t *)&flash_param,sizeof(cfg_param_t));

    
    memset(&flash_param,0,sizeof(cfg_param_t));
    load_default_param_city(&flash_param);
    drv_fls_write(PARAM_ADDR_CITY,(uint8_t *)&flash_param,sizeof(cfg_param_t));
/*
    if(-1 == err)
        rt_kprintf("error happened when writing default param to flash");
    else
        rt_kprintf("write default param to flash  success\n");
*/

}

FINSH_FUNCTION_EXPORT(write_def_param, debug:write default  param to flash);


void param_init(void)
{
    uint8_t magic_word[sizeof(param_init_words)];    
    drv_fls_read(PARAM_FLAG_ADDR,magic_word,sizeof(param_init_words));
    param_mode = mode_get();
    //drv_fls_read(PARAM_MODE_ADDR,(uint8_t*)&param_mode,sizeof(uint16_t));
    if(strcmp((const char*)param_init_words,(const char*)magic_word) != 0){
            p_cms_param = &cms_param;
            osal_printf("loading default param,mode is custom\n");
            load_default_param_custom(p_cms_param);            
            write_def_param();
    }
    else{
        load_param_from_fl();
    }
}

void param_get(void)
{
    //cfg_param_t  *param_temp;

    //drv_fls_read(PARAM_ADDR,(uint8_t *)param_temp,sizeof(cfg_param_t));
        
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"-------------------parameters in ram------------------\n");
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"work mode is %d\n",param_mode);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"----------------------vam---------------------\n");
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"ID(0)=%d%d%d%d\n",p_cms_param->pid[0],p_cms_param->pid[1],p_cms_param->pid[2],p_cms_param->pid[3]);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.bsm_hops(1)=%d\n", p_cms_param->vam.bsm_hops);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.bsm_boardcast_mode(2)=%d\n", p_cms_param->vam.bsm_boardcast_mode);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.bsm_boardcast_saftyfactor(3)=%d\n", p_cms_param->vam.bsm_boardcast_saftyfactor);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.bsm_pause_mode(4)=%d\n", p_cms_param->vam.bsm_pause_mode);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.bsm_pause_hold_time(5)=%d (s)\n", p_cms_param->vam.bsm_pause_hold_time);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.bsm_boardcast_period(6)=%d (ms)\n", p_cms_param->vam.bsm_boardcast_period);

    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.evam_hops(7)=%d\n", p_cms_param->vam.evam_hops);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.evam_broadcast_type(8)=%d\n", p_cms_param->vam.evam_broadcast_type);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vam.evam_broadcast_peroid(9)=%d (ms)\n\n", p_cms_param->vam.evam_broadcast_peroid);

    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"----------------------vsa---------------------\n");
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.danger_detect_speed_threshold(10)=%d (km/h)\n", p_cms_param->vsa.danger_detect_speed_threshold);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.lane_dis(11)=%d (m)\n", p_cms_param->vsa.lane_dis);
    
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.crd_saftyfactor(12)=%d\n", p_cms_param->vsa.crd_saftyfactor);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.crd_oppsite_speed(13)=%d (km/h)\n", p_cms_param->vsa.crd_oppsite_speed);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.crd_oppsite_rear_speed(14)=%d (km/h)\n", p_cms_param->vsa.crd_oppsite_rear_speed);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.crd_rear_distance(15)=%d (m)\n", p_cms_param->vsa.crd_rear_distance);

        
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.ebd_mode(16)=%d\n", p_cms_param->vsa.ebd_mode);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.ebd_acceleration_threshold(17)=%d (m/s2)\n", p_cms_param->vsa.ebd_acceleration_threshold);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"vsa.ebd_alert_hold_time(18)=%d (s)\n\n", p_cms_param->vsa.ebd_alert_hold_time);

    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"----------------------gsnr---------------------\n");
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.gsnr_cal_step(19)=%d\n",p_cms_param->gsnr.gsnr_cal_step);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.gsnr_cal_thr(20)=%d\n",p_cms_param->gsnr.gsnr_cal_thr);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.gsnr_ebd_thr(21)=%d\n",p_cms_param->gsnr.gsnr_ebd_thr);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.gsnr_ebd_cnt(22)=%d\n",p_cms_param->gsnr.gsnr_ebd_cnt);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.AcceV_x(23)=%d\n",p_cms_param->gsnr.AcceV_x);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.AcceV_y(24)=%d\n",p_cms_param->gsnr.AcceV_y);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.AcceV_z(25)=%d\n",p_cms_param->gsnr.AcceV_z);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.AcceAhead_x(26)=%d\n",p_cms_param->gsnr.AcceAhead_x);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.AcceAhead_y(27)=%d\n",p_cms_param->gsnr.AcceAhead_y);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"gsnr.AcceAhead_z(28)=%d\n",p_cms_param->gsnr.AcceAhead_z);

    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"----------------------wnet---------------------\n");
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"wnet.channel(29)=%d\n",p_cms_param->wnet.channel);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"wnet.txrate(30)=%d\n",p_cms_param->wnet.txrate);

    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"----------------------voc----------------------\n");
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"voc.fg_volume(32)=%d\n",p_cms_param->voc.fg_volume);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"voc.bg_volume(33)=%d\n",p_cms_param->voc.bg_volume);
    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"voc.speed(34)=%d\n",p_cms_param->voc.speed);

    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"...\n");

    OSAL_MODULE_DBGPRT(MODULE_NAME,OSAL_DEBUG_INFO,"----------------------end---------------------\n");
}
FINSH_FUNCTION_EXPORT(param_get, get system parameters);

void print_bn(void)
{


    rt_kprintf("vam_config_t  is %d bytes\n",sizeof(vam_config_t));


    rt_kprintf("vsa_config_t  is %d bytes\n",sizeof(vsa_config_t));

    rt_kprintf("cfg_param_t  is %d bytes\n",sizeof(cfg_param_t));

    rt_kprintf("gsnr_config_t  is %d bytes\n",sizeof(gsnr_config_t));


    rt_kprintf("param_init_words is %d bytes\n",sizeof(param_init_words));

}
FINSH_FUNCTION_EXPORT(print_bn, data struct bytes needed);



void print_init_word(void)//print  flag of initialized
{
    
    uint8_t init_word[sizeof(param_init_words)];


    drv_fls_read(PARAM_FLAG_ADDR,init_word,sizeof(param_init_words));

    rt_kprintf("init word in flash is \"%s\"\n",init_word);

}
FINSH_FUNCTION_EXPORT(print_init_word, print init words  in flash);



void print_fd(uint32_t addr)//print  data of specified  address ,e.g:print_fd(0x80E0010),
{

    uint8_t data;

    drv_fls_read(addr,&data,1);

    rt_kprintf("data in address %x  is \"%d\"\n",addr,data);
}

FINSH_FUNCTION_EXPORT(print_fd, print  data of specified  address in flash);



int param_set(uint8_t param, int32_t value)
{

    int err;

    cfg_param_t *cfg_param;

    uint16_t mode;

    uint8_t param_num;

    cfg_param_t all_param[4];

    
    drv_fls_read(PARAM_ADDR_CUSTOM,(uint8_t*)&all_param[0],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_HIGHWAY,(uint8_t*)&all_param[1],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_MOUNTAIN,(uint8_t*)&all_param[2],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_CITY,(uint8_t*)&all_param[3],sizeof(cfg_param_t));
    
    
    
    drv_fls_read(PARAM_MODE_ADDR,(uint8_t *)&mode,sizeof(uint16_t));

    param_num = get_param_num(mode);

    osal_printf("mode is %04X\n",mode);

    cfg_param = &all_param[param_num];


    if(param == 29){
        if((value < 1)&&(value > 13)){
            rt_kprintf("wrong channel!\n\n");
            return -1;
        }
    }

   if(param == 30){
        if((value != 1)&&(value != 2)&&(value != 6)){
            rt_kprintf("param of Txrate is just 1,2,6\n\n");
            return -1;
        }

   }
                
    switch(param){

    case 0:
        if(value > 9999){
                rt_kprintf("invalid  ID!!\n");
                return -1;
        }    
        cfg_param->pid[0] = value/1000;
        cfg_param->pid[1] = (value%1000)/100;
        cfg_param->pid[2] = ((value%1000)%100)/10;
        cfg_param->pid[3] =    ((value%1000)%100)%10;
        break;

    case 1:
        cfg_param->vam.bsm_hops = value;
        break;
    case 2:
        cfg_param->vam.bsm_boardcast_mode = value;
        break;        
    case 3:
        cfg_param->vam.bsm_boardcast_saftyfactor = value;
        break;
    case 4:
        cfg_param->vam.bsm_pause_mode = value;
        break;
    case 5:
        cfg_param->vam.bsm_pause_hold_time = value;
        break;
    case 6:
        cfg_param->vam.bsm_boardcast_period = value;
        break;
        
    case 7:
        cfg_param->vam.evam_hops = value;
        break;
    case 8:
        cfg_param->vam.evam_broadcast_type = value;
        break;
    case 9:
        cfg_param->vam.evam_broadcast_peroid = value;
        break;            


        
    case 10:
        cfg_param->vsa.danger_detect_speed_threshold = value;
        break;
    case 11:
        cfg_param->vsa.lane_dis = value;
        
        break;            
    case 12:
        cfg_param->vsa.crd_saftyfactor = value;
        break;
    case 13:
        cfg_param->vsa.crd_oppsite_speed = value;
        break;
    case 14:
        cfg_param->vsa.crd_oppsite_rear_speed = value;
        break;
    case 15:
        cfg_param->vsa.crd_rear_distance = value;
        break;

        
    case 16:
        cfg_param->vsa.ebd_mode = value;
        break;
    case 17:
        cfg_param->vsa.ebd_acceleration_threshold = value;
        break;            
    case 18:
        cfg_param->vsa.ebd_alert_hold_time = value;
        break;

    case 19:
        cfg_param->gsnr.gsnr_cal_step = value;
        break;
    case 20:
        cfg_param->gsnr.gsnr_cal_thr = value;
        break;
    case 21:
        cfg_param->gsnr.gsnr_ebd_thr = value;
        break;
    case 22:
        cfg_param->gsnr.gsnr_ebd_cnt = value;
        break;

    case 23:
        cfg_param->gsnr.AcceV_x = value;
        break;
    case 24:
        cfg_param->gsnr.AcceV_y = value;
        break;
    case 25:
        cfg_param->gsnr.AcceV_z = value;
        break;
    case 26:
        cfg_param->gsnr.AcceAhead_x = value;
        break;
    case 27:
        cfg_param->gsnr.AcceAhead_y = value;
        break;
    case 28:
        cfg_param->gsnr.AcceAhead_z = value;
        break;

        
    case 29:
        cfg_param->wnet.channel = value;
        break;

    case 30:
        cfg_param->wnet.txrate = value;
        break;
        
    case 31:
        cfg_param->print_xxx = value;
        break;
        
    case 32:
        cfg_param->voc.fg_volume = value;
        break;
    case 33:
        cfg_param->voc.bg_volume = value;
        break;
    case 34:
        cfg_param->voc.speed = value;
        break;
        
    default:          
        rt_kprintf("invalid  parameter  number!!\n");
        break;

    }

    memcpy((uint8_t*)p_cms_param,(uint8_t*)cfg_param,sizeof(cfg_param_t));


    rt_kprintf("param is setting .....please don't power off!\n");
        
    drv_fls_erase(PARAM_SECTOR);
    drv_fls_write(PARAM_FLAG_ADDR,param_init_words,sizeof(param_init_words));
    drv_fls_write(PARAM_MODE_ADDR,(uint8_t *)&mode,sizeof(uint16_t));
    
    drv_fls_write(PARAM_ADDR_CUSTOM,(uint8_t*)&all_param[0],sizeof(cfg_param_t));
    drv_fls_write(PARAM_ADDR_HIGHWAY,(uint8_t*)&all_param[1],sizeof(cfg_param_t));
    drv_fls_write(PARAM_ADDR_MOUNTAIN,(uint8_t*)&all_param[2],sizeof(cfg_param_t));
    err = drv_fls_write(PARAM_ADDR_CITY,(uint8_t*)&all_param[3],sizeof(cfg_param_t));

    if(err == -1){
        rt_kprintf("parameter writing process error!!!\n");
    }
    else{
        rt_kprintf("parameter set success!\n");
    }

    cfg_param = NULL;

    return 0;

}


FINSH_FUNCTION_EXPORT(param_set, set system parameters);
int erase_sector4(void)
{
    int err = 0;
    err = drv_fls_erase(FLASH_Sector_4);

    drv_fls_write(0x8010000,param_init_words,sizeof(param_init_words));
    drv_fls_write(0x8010010,param_init_words,sizeof(param_init_words));
    
    drv_fls_write(0x8010020,param_init_words,sizeof(param_init_words));
    drv_fls_write(0x8010220,param_init_words,sizeof(param_init_words));
    drv_fls_write(0x8010420,param_init_words,sizeof(param_init_words));
    err = drv_fls_write(0x8010620,param_init_words,sizeof(param_init_words));
    return err;
}

FINSH_FUNCTION_EXPORT(erase_sector4, set system parameters);

uint8_t flash_read(uint8_t mode)
{
    cfg_param_t  *param_temp;

    uint32_t param_addr;

    if(mode == 1){
        param_addr = PARAM_ADDR_CUSTOM;
    }
    else if(mode == 2){
        param_addr = PARAM_ADDR_HIGHWAY;
    }
    else if(mode == 3){
        param_addr = PARAM_ADDR_MOUNTAIN;
    }
    else if(mode == 4){
        param_addr = PARAM_ADDR_CITY;
    }
    else{ 
        osal_printf("parameter error!\n");
        return 0;
    }

    param_temp = (cfg_param_t*)rt_malloc(sizeof(cfg_param_t));

    drv_fls_read(param_addr,(uint8_t *)param_temp,sizeof(cfg_param_t));
        
    rt_kprintf("-------------------parameters in  flash------------------\n");    
    rt_kprintf("----------------------vam---------------------\n");    
    rt_kprintf("ID(0)=%d%d%d%d\n",param_temp->pid[0],param_temp->pid[1],param_temp->pid[2],param_temp->pid[3]);
    rt_kprintf("vam.bsm_hops(1)=%d\n", param_temp->vam.bsm_hops);
    rt_kprintf("vam.bsm_boardcast_mode(2)=%d\n", param_temp->vam.bsm_boardcast_mode);
    rt_kprintf("vam.bsm_boardcast_saftyfactor(3)=%d\n", param_temp->vam.bsm_boardcast_saftyfactor);
    rt_kprintf("vam.bsm_pause_mode(4)=%d\n", param_temp->vam.bsm_pause_mode);
    rt_kprintf("vam.bsm_pause_hold_time(5)=%d (s)\n", param_temp->vam.bsm_pause_hold_time);
    rt_kprintf("vam.bsm_boardcast_period(6)=%d (ms)\n", param_temp->vam.bsm_boardcast_period);

    rt_kprintf("vam.evam_hops(7)=%d\n", param_temp->vam.evam_hops);
    rt_kprintf("vam.evam_broadcast_type(8)=%d\n", param_temp->vam.evam_broadcast_type);
    rt_kprintf("vam.evam_broadcast_peroid(9)=%d (ms)\n\n", param_temp->vam.evam_broadcast_peroid);
    
    rt_kprintf("----------------------vsa---------------------\n");    
    rt_kprintf("vsa.danger_detect_speed_threshold(10)=%d (km/h)\n", param_temp->vsa.danger_detect_speed_threshold);
    rt_kprintf("vsa.lane_dis(11)=%d (m)\n", param_temp->vsa.lane_dis);
    
    rt_kprintf("vsa.crd_saftyfactor(12)=%d\n", param_temp->vsa.crd_saftyfactor);
    rt_kprintf("vsa.crd_oppsite_speed(13)=%d (km/h)\n", param_temp->vsa.crd_oppsite_speed);
    rt_kprintf("vsa.crd_oppsite_rear_speed(14)=%d (km/h)\n", param_temp->vsa.crd_oppsite_rear_speed);
    rt_kprintf("vsa.crd_rear_distance(15)=%d (m)\n", param_temp->vsa.crd_rear_distance);

        
    rt_kprintf("vsa.ebd_mode(16)=%d\n", param_temp->vsa.ebd_mode);
    rt_kprintf("vsa.ebd_acceleration_threshold(17)=%d (m/s2)\n", param_temp->vsa.ebd_acceleration_threshold);
    rt_kprintf("vsa.ebd_alert_hold_time(18)=%d (s)\n\n", param_temp->vsa.ebd_alert_hold_time);
    
    rt_kprintf("----------------------gsnr---------------------\n");    
    rt_kprintf("gsnr.gsnr_cal_step(19)=%d\n",param_temp->gsnr.gsnr_cal_step);
    rt_kprintf("gsnr.gsnr_cal_thr(20)=%d\n",param_temp->gsnr.gsnr_cal_thr);
    rt_kprintf("gsnr.gsnr_ebd_thr(21)=%d\n",param_temp->gsnr.gsnr_ebd_thr);
    rt_kprintf("gsnr.gsnr_ebd_cnt(22)=%d\n",param_temp->gsnr.gsnr_ebd_cnt);
    rt_kprintf("gsnr.AcceV_x(23)=%d\n",param_temp->gsnr.AcceV_x);
    rt_kprintf("gsnr.AcceV_y(24)=%d\n",param_temp->gsnr.AcceV_y);
    rt_kprintf("gsnr.AcceV_z(25)=%d\n",param_temp->gsnr.AcceV_z);
    rt_kprintf("gsnr.AcceAhead_x(26)=%d\n",param_temp->gsnr.AcceAhead_x);
    rt_kprintf("gsnr.AcceAhead_y(27)=%d\n",param_temp->gsnr.AcceAhead_y);
    rt_kprintf("gsnr.AcceAhead_z(28)=%d\n",param_temp->gsnr.AcceAhead_z);
    
    rt_kprintf("----------------------wnet---------------------\n");    
    rt_kprintf("wnet.channel(29)=%d\n",param_temp->wnet.channel);
    rt_kprintf("wnet.txrate(30)=%d\n",param_temp->wnet.txrate);

    rt_kprintf("----------------------voc---------------------\n");    
    rt_kprintf("voc.fg_volume(32)=%d\n",param_temp->voc.fg_volume);
    rt_kprintf("voc.bg_volume(33)=%d\n",param_temp->voc.bg_volume);
    rt_kprintf("voc.speed(34)=%d\n",param_temp->voc.speed);
    
    rt_kprintf("...\n");

    rt_kprintf("----------------------end---------------------\n");    

    rt_free(param_temp);

    param_temp = NULL;

    return 0;

}

FINSH_FUNCTION_EXPORT(flash_read, debug:reading param);


int8_t  gsnr_param_set(uint8_t gsnr_cal_step,int32_t AcceV_x,int32_t AcceV_y,int32_t AcceV_z,int32_t AcceAhead_x,int32_t AcceAhead_y,int32_t AcceAhead_z)
{

    int err;

    cfg_param_t *cfg_param;

    uint16_t mode;

    uint8_t param_num;

    cfg_param_t all_param[4];

    
    drv_fls_read(PARAM_ADDR_CUSTOM,(uint8_t*)&all_param[0],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_HIGHWAY,(uint8_t*)&all_param[1],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_MOUNTAIN,(uint8_t*)&all_param[2],sizeof(cfg_param_t));
    drv_fls_read(PARAM_ADDR_CITY,(uint8_t*)&all_param[3],sizeof(cfg_param_t));
    
    
    
    drv_fls_read(PARAM_MODE_ADDR,(uint8_t *)&mode,sizeof(uint16_t));

    param_num = get_param_num(mode);

    osal_printf("mode is %04X\n",mode);

    cfg_param = &all_param[param_num];

    cfg_param->gsnr.gsnr_cal_step = gsnr_cal_step;
    cfg_param->gsnr.AcceV_x = AcceV_x;
    cfg_param->gsnr.AcceV_y = AcceV_y;
    cfg_param->gsnr.AcceV_z = AcceV_z;
    cfg_param->gsnr.AcceAhead_x = AcceAhead_x;
    cfg_param->gsnr.AcceAhead_y = AcceAhead_y;
    cfg_param->gsnr.AcceAhead_z = AcceAhead_z;

    memcpy((uint8_t*)p_cms_param,(uint8_t*)cfg_param,sizeof(cfg_param_t));


    rt_kprintf("gsnr is setting .....please don't power off!\n");
        
    drv_fls_erase(PARAM_SECTOR);
    drv_fls_write(PARAM_FLAG_ADDR,param_init_words,sizeof(param_init_words));
    drv_fls_write(PARAM_MODE_ADDR,(uint8_t *)&mode,sizeof(uint16_t));
    drv_fls_write(PARAM_ADDR_CUSTOM,(uint8_t*)&all_param[0],sizeof(cfg_param_t));
    drv_fls_write(PARAM_ADDR_HIGHWAY,(uint8_t*)&all_param[1],sizeof(cfg_param_t));
    drv_fls_write(PARAM_ADDR_MOUNTAIN,(uint8_t*)&all_param[2],sizeof(cfg_param_t));
    err = drv_fls_write(PARAM_ADDR_CITY,(uint8_t*)&all_param[3],sizeof(cfg_param_t));

    if(err == -1)
        rt_kprintf("gsnr param setting failed!!!\n");
    else
        rt_kprintf("gsnr param setting success!\n");


    cfg_param = NULL;


    return 0;

}


FINSH_FUNCTION_EXPORT(gsnr_param_set, gsnr param setting);





