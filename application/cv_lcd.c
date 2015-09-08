/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_lcd.c
 @brief  : this file include the application functions for the lcd.
 @author : wangxianwen
 @history:
           2015-6-02    wangxianwen    Created file
           ...
******************************************************************************/

#include "cv_lcd.h"
#include "drv_main.h"

#include "cv_osal.h"


#include "cv_vam.h"


#include "app_interface.h"


#include "cv_cms_def.h"
#include "dat_lcd_alarm.h"
#include "dat_lcd_font.h"



/* Vehicles information group. */
static vec_graph_st VecGraph = 
{ 
    RESOLUTION_LONGITUDE_DEFAULT, RESOLUTION_LATITUDE_DEFAULT,
    0,
    {
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 },
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 },
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 },
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 },
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 },
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 },
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 },
        { LCD_FOREGROUND_LAYER,   0,   0, 4, LCD_CIRCLE_FILLED,          0x4444 } 
    }
};

/* Vehicle parameter. */
static sys_param_st SysParam = 
{
    SYS_SYSMODE_NORMAL,                                               /* sys_mode. */
    { VEC_ROADMODE_HIGHWAY, VEC_VECMODE_CAR, VEC_BREAKDOWNMODE_NO },  /* vec_param. */
    0x00                                                              /* alarm_stat. */
};






/**
  * @brief  Active the lcd specific layer.
  * @param  See below.
  * @retval None.
  */
static void lcd_layer_active
(
    /* The specific layer. */
    drv_lcd_layer_value lcd_layer
)
{   
    /* Structure for tranparency setting. */
    drv_lcd_layer_transparency_set_st transparency = { lcd_layer, LCD_TRANSPARENCY_NO };
    
    
    /* Set lcd layer's transparency.*/
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_SET_LAYER_TRANSPARENCY, &transparency);
}


/**
  * @brief  Inactive the lcd specific layer.
  * @param  See below.
  * @retval None.
  */
static void lcd_layer_inactive
(
    /* The specific layer. */
    drv_lcd_layer_value lcd_layer
)
{   
    /* Structure for tranparency setting. */
    drv_lcd_layer_transparency_set_st transparency = { lcd_layer, LCD_TRANSPARENCY_FULL };
    
    
    /* Set lcd layer's transparency.*/
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_SET_LAYER_TRANSPARENCY, &transparency);
}


/**
  * @brief  Initialize lcd screen background.
  * @param  See below.
  * @retval None.
  */
static void lcd_screen_background_init
(
    /* Active layer for screen background. */
    drv_lcd_layer_value lcd_layer
)
{
#if BACKGROUND_INIT_MANUAL    
    
    /* Structure for tranparency set and lcd clear. */
    drv_lcd_layer_transparency_set_st transparency_st = { LCD_FOREGROUND_LAYER, LCD_TRANSPARENCY_NO };
    drv_lcd_layer_clear_st                   clear_st = { LCD_BACK_FORE_LAYER,  LCD_COLOR_WHITE     };
    
    /* Structure for horizontal line + vertical line + circle draw.*/
    drv_lcd_hv_line_draw_st h_line = { LCD_FOREGROUND_LAYER,  35, 120, 170,1, LCD_LINE_DIR_HORIZONTAL, LCD_COLOR_BLACK };
    drv_lcd_hv_line_draw_st v_line = { LCD_FOREGROUND_LAYER, 120,  35, 170,1, LCD_LINE_DIR_VERTICAL,   LCD_COLOR_BLACK };
    
    drv_lcd_circle_draw_st        circle = { LCD_FOREGROUND_LAYER, 120, 120, 70,  LCD_CIRCLE_EMPTY, LCD_COLOR_BLACK };
    drv_lcd_circle_draw_st circle_centre = { LCD_FOREGROUND_LAYER, 120, 120,  6, LCD_CIRCLE_FILLED, LCD_COLOR_BLACK };
    
    /* Structure for rectangle draw. */
    drv_lcd_rectangle_draw_st     rectangle_st = { LCD_FOREGROUND_LAYER, 240,  5,235,100, LCD_COLOR_BLACK };
    drv_lcd_rectangle_draw_st    rectangle2_st = { LCD_FOREGROUND_LAYER, 240,130,235,137, LCD_COLOR_BLACK };
    
    
    /* Set lcd layer's transparency and clear lcd layer.*/
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_SET_LAYER_TRANSPARENCY, &transparency_st);
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_CLEAR_LAYER, &clear_st);
    
    /* Draw horizontal line + vertical line + empty circle. */
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_HV_LINE, &h_line);
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_HV_LINE, &v_line);
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_CIRCLE, &circle);
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_CIRCLE, &circle_centre);
    
    /* Draw rectangle group. */
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_RECTANGLE, &rectangle_st);
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_RECTANGLE, &rectangle2_st);
    
#else
    
    /* Background setting. */
    drv_lcd_block_draw_st background = { lcd_layer, 0, 0, &PicInforGroup[PIC_INDEX_INFOR_VEC_BACKGROUND] };
    

    /* Initial screen background. */
    DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_BLOCK, &background);
    
#endif    
}


/**
  * @brief  Initial vehicle parameters to the specific lcd layer.
  * @param  See below.
  * @retval None.
  */
static void lcd_vec_param_init
(
    /* Lcd layer. */
    drv_lcd_layer_value lcd_layer,

    /* System parameter pointer. */
    sys_param_st_ptr param_ptr
)
{
    /* Indicator structure. */
    drv_lcd_block_draw_st      road_mode = { lcd_layer, 40, 265, &PicInforGroup[PIC_INDEX_INFOR_HIGHWAY] };
    drv_lcd_block_draw_st   vehicle_mode = { lcd_layer, 40, 337, &PicInforGroup[PIC_INDEX_INFOR_CAR] };
    drv_lcd_block_draw_st breakdown_mode = { lcd_layer, 40, 409, &PicInforGroup[PIC_INDEX_INFOR_NORMAL] };

    drv_lcd_block_draw_st          alarm = { lcd_layer, 10, 260, &PicAlarmGroup[PIC_INDEX_FRONT_VEC_FAULTY] };

    
    if(param_ptr->sys_mode == SYS_SYSMODE_NORMAL)
    {
        /* Initial road mode indicator. */
        switch(param_ptr->vec_param.road_mode)
        {
            case VEC_ROADMODE_HIGHWAY:    {  road_mode.plane_ptr = &PicInforGroup[PIC_INDEX_INFOR_HIGHWAY];        break;  }
            case VEC_ROADMODE_MOUNTAIN:   {  road_mode.plane_ptr = &PicInforGroup[PIC_INDEX_INFOR_MOUNTAIN];       break;  }
            case VEC_ROADMODE_CITY:       {  road_mode.plane_ptr = &PicInforGroup[PIC_INDEX_INFOR_CITY];           break;  }
            default:                      {  road_mode.plane_ptr = NULL;                                           break;  }
        }
        
        /* Initial vechicle mode indicator. */
        switch(param_ptr->vec_param.vec_mode)
        {
            case VEC_VECMODE_CAR:         {  vehicle_mode.plane_ptr = &PicInforGroup[PIC_INDEX_INFOR_CAR];         break;  } 
            case VEC_VECMODE_AMBULANCE:   {  vehicle_mode.plane_ptr = &PicInforGroup[PIC_INDEX_INFOR_AMBULANCE];   break;  } 
            default:                      {  vehicle_mode.plane_ptr = NULL;                                        break;  }    
        }
        
        /* Initial breakdown mode indicator. */
        switch(param_ptr->vec_param.breakdown_mode)
        {
            case VEC_BREAKDOWNMODE_NO:    {  breakdown_mode.plane_ptr = &PicInforGroup[PIC_INDEX_INFOR_NORMAL];    break;  } 
            case VEC_BREAKDOWNMODE_YES:   {  breakdown_mode.plane_ptr = &PicInforGroup[PIC_INDEX_INFOR_BREAKDOWN]; break;  } 
            default:                      {  breakdown_mode.plane_ptr = NULL;                                      break;  } 
        }
        
        /* Fresh data to lcd layer. */
        DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_BLOCK, &road_mode);
        DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_BLOCK, &vehicle_mode);
        DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_BLOCK, &breakdown_mode);
    }
    else if(param_ptr->sys_mode == SYS_SYSMODE_ALARM)
    {
        if(param_ptr->alarm_stat & (1 << HI_OUT_EBD_ALERT))
        {
            alarm.plane_ptr = &PicAlarmGroup[PIC_INDEX_FRONT_VEC_BRAKE];
        }
        else if(param_ptr->alarm_stat & (1 << HI_OUT_VBD_ALERT))
        {
            alarm.plane_ptr = &PicAlarmGroup[PIC_INDEX_FRONT_VEC_FAULTY];
        }
        else if(param_ptr->alarm_stat & (1 << HI_OUT_CRD_ALERT))
        {
            alarm.plane_ptr = &PicAlarmGroup[PIC_INDEX_ATT_FRONT_VEC];
        }
/*      else if(param_ptr->alarm_stat & (1 << HI_OUT_VBD_ALERT))
        {
            alarm.plane_ptr = &PicAlarmGroup[PIC_INDEX_FRONT_VEC_FAULTY];
        }
        else if(param_ptr->alarm_stat & (1 << HI_OUT_VBD_ALERT))
        {
            alarm.plane_ptr = &PicAlarmGroup[PIC_INDEX_FRONT_VEC_FAULTY];
        }
        else if(param_ptr->alarm_stat & (1 << HI_OUT_VBD_ALERT))
        {
            alarm.plane_ptr = &PicAlarmGroup[PIC_INDEX_FRONT_VEC_FAULTY];
        }
*/
        /* Fresh data to lcd layer. */
        DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_BLOCK, &alarm);
    }
}


/**
  * @brief  Initial vehicle graph to the specific lcd layer.
  * @param  See below.
  * @retval None.
  */
static void lcd_vec_graph_init
(
    /* Pointer to vehicle graph structure. */
    vec_graph_st_ptr vec_graph_ptr
)
{
    uint32_t            vec_index = 0;
    uint8_t peer_pid[VAM_NEIGHBOUR_MAXNUM][RCP_TEMP_ID_LEN] = { 0 };
    
    vam_stastatus_t  local_status = { 0 };  
    vam_stastatus_t remote_status = { 0 };


    osal_assert(vec_graph_ptr!= RT_NULL);
   
    vam_get_all_peer_pid(peer_pid, VAM_NEIGHBOUR_MAXNUM, &(vec_graph_ptr->vec_num));
    vam_get_local_current_status(&local_status);

    /* Get all the specific peer's status. */
    for (vec_index = 0; vec_index < vec_graph_ptr->vec_num; vec_index++) 
    {
        vam_get_peer_current_status(peer_pid[vec_index], &remote_status);            
        
        /* Update vehicle's coordinate based on longitude and latitude. */
        vec_graph_ptr->vec_group[vec_index].x = LOCAL_COORDINATE_X + (remote_status.pos.lon - local_status.pos.lon) * vec_graph_ptr->resolution_longitude;
        vec_graph_ptr->vec_group[vec_index].y = LOCAL_COORDINATE_Y + (remote_status.pos.lat - local_status.pos.lat) * vec_graph_ptr->resolution_latitude;
    }

    /* Draw all the active vehicle on screen. */
    for(vec_index = 0; vec_index < vec_graph_ptr->vec_num; vec_index ++)
    {
        DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DRAW_CIRCLE, vec_graph_ptr->vec_group + vec_index);
    }  
}



/**
  * @brief  Initial vehicle information to the specific layer.
  * @param  See below.
  * @retval None.
  */
static void lcd_vec_infor_init
(
    /* Lcd layer. */
    drv_lcd_layer_value lcd_layer
)
{
    uint8_t i = 0;
    
    
    char buffer_lon[20]   = "Lon:";
    char buffer_lat[20]   = "Lat:";
    char buffer_dir[20]   = "Dir:";
    char buffer_speed[20] = "Spd:";
    
    drv_lcd_string_display_st infor_group[4] = 
    {
        { lcd_layer, 150,280, LCD_COLOR_WHITE, LCD_COLOR_BLACK, &PicFontGroup[PIC_FONT_INDEX_ASCII_8X12], buffer_lon   },
        { lcd_layer, 170,280, LCD_COLOR_WHITE, LCD_COLOR_BLACK, &PicFontGroup[PIC_FONT_INDEX_ASCII_8X12], buffer_lat   },
        { lcd_layer, 190,280, LCD_COLOR_WHITE, LCD_COLOR_BLACK, &PicFontGroup[PIC_FONT_INDEX_ASCII_8X12], buffer_dir   },
        { lcd_layer, 210,280, LCD_COLOR_WHITE, LCD_COLOR_BLACK, &PicFontGroup[PIC_FONT_INDEX_ASCII_8X12], buffer_speed }
    };
    
    vam_stastatus_t  local_status = { 0 }; 

    
    vam_get_local_current_status(&local_status);

    sprintf(&(buffer_lon[strlen(buffer_lon)]), "%f", local_status.pos.lon);
    sprintf(&(buffer_lat[strlen(buffer_lat)]), "%f", local_status.pos.lat);
    sprintf(&(buffer_dir[strlen(buffer_dir)]), "%f", local_status.dir);
    sprintf(&(buffer_speed[strlen(buffer_speed)]), "%f", local_status.speed);

    for(i = 0; i < 4; i++)
    {
        DRV_LCD_PTR->ioctl(DRV_LCD_PTR, LCD_IOCTL_DISPLAY_STRING, &infor_group[i]);
    }   
}







/**
  * @brief  Initial system parameter based on system environment.
  * @param  See below.
  * @retval None.
  */
void lcd_sys_param_init
(
    /* Pointer to system parameter. */
    sys_param_st_ptr sys_param_ptr,

    /* Pointer to system environment. */
    sys_envar_t *sys_envar_ptr
)
{
    /* Initial alarm status. */
    sys_param_ptr->alarm_stat = sys_envar_ptr->status & ((1<<HI_OUT_VBD_ALERT) | (1<<HI_OUT_CRD_ALERT) | (1<<HI_OUT_EBD_ALERT));
  
    /* Initial system mode. */
    if(sys_param_ptr->alarm_stat != 0)
    {
        sys_param_ptr->sys_mode = SYS_SYSMODE_ALARM;
    }
    else
    {
        sys_param_ptr->sys_mode = SYS_SYSMODE_NORMAL;
    }
}


/**
  * @brief  Entry routine for lcd thread.
  * @param  See below.
  * @retval None.
  */
void lcd_thread_entry
(
    /* Pointr to parameter for thread. */
    void *parameter
)
{ 
    drv_lcd_layer_value primary_layer = LCD_BACKGROUND_LAYER;
    drv_lcd_layer_value  backup_layer = LCD_FOREGROUND_LAYER;
    drv_lcd_layer_value    temp_layer = LCD_BACKGROUND_LAYER;

    
    
    /* Inform to the system. */
    if(p_cms_envar->sys.queue_sys_mng) 
    {
        sys_add_event_queue(&p_cms_envar->sys, SYS_MSG_INITED, 0, 0, 0);
    }
    
    while(1)
    {     
        /* Initial lcd system parameter. */
        lcd_sys_param_init(&SysParam, &p_cms_envar->sys);

        
        /* Initial lcd background screen. */
        lcd_screen_background_init(backup_layer);
        
        /* Initial vehicle parameter. */
        lcd_vec_param_init(backup_layer, &SysParam);
      
        /* Initial vehicle graph. */
        lcd_vec_graph_init(&VecGraph);

        /* Initial vehicle information. */
        lcd_vec_infor_init(backup_layer);
        

       
        /* Active the backup layer and inactive primary layer. */
        lcd_layer_active(backup_layer);
        lcd_layer_inactive(primary_layer);
 
        /* Swap the primary layer and backup layer. */
        temp_layer = primary_layer;
        primary_layer = backup_layer;
        backup_layer = temp_layer;
 
        
        /* Delay 100ms for lcd fresh. */
        osal_delay(10);  
    }
}


/**
  * @brief  Initialize lcd thread.
  * @param  None.
  * @retval None.
  */
void lcd_thread_init
( 
    /* No parameter. */
    void
)
{
    osal_task_t *tid_ptr = NULL;
    
    
    /* Create lcd task. */
    tid_ptr = osal_task_create("lcd", lcd_thread_entry, NULL, LCD_THREAD_STACK_SIZE, LCD_THREAD_PRIORITY);
    osal_assert(tid_ptr != RT_NULL);
}

