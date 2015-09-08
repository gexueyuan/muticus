/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_lcd.h
 @brief  : this file include the application variables and functions prototypes 
           for the lcd.
 @author : wangxianwen
 @history:
           2015-6-02    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _CV_LCD_H_
#define _CV_LCD_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "drv_lcd.h"



/* Gnss status structure. */
typedef struct _gnss_status_st
{
    uint8_t data_valid;

    
}gnss_status_st, *gnss_status_st_ptr;

#define GNSS_STATUS_ST_LEN    (gnss_status_st)



/* Gnss data validation definition. */
#define GNSS_DATA_VALID_NO    0x00
#define GNSS_DATA_VALID_YES   0x01  





/* NMEA frame status. */
#define NMEA_FRAME_DATA_ERR 0x01
#define NMEA_FRAME_DATA_OK  0x00


     
/* NMEA0183 frame common head. */
typedef struct _nmea_frame_head_st
{
    /* $aaccc,c--c*hh<CR><LF> */
    
    uint8_t          frame_start;    /* $ */
    uint8_t     address_field[5];    /* aaccc */
    uint8_t    field_delimiter_0;    /* , */

}nmea_frame_head_st, *nmea_frame_head_st_ptr;

#define NMEA_FRAME_HEAD_ST_LEN    (sizeof(nmea_frame_head_st))


/* NMEA0183 frame: RMC - Recommended Minimum Specific GNSS Data. */
typedef struct _nmea_frame_rmc_st
{
    /* $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxxxx,x.x,a,a*hh<CR><LF> */

    nmea_frame_head_st      head;    /* common head. */
    
    uint8_t  position_fix_utc[9];    /* hhmmss.ss */
    uint8_t    field_delimiter_1;    /* , */

    uint8_t               status;    /* A */
    uint8_t    field_delimiter_2;    /* , */

    uint8_t          latitude[7];    /* llll.ll */
    uint8_t    field_delimiter_3;    /* , */

    uint8_t          latitude_ns;    /* a */ 
    uint8_t    field_delimiter_4;    /* , */

    uint8_t         longitude[8];    /* yyyyy.yy */
    uint8_t    field_delimiter_5;    /* , */

    uint8_t         longitude_ew;    /* a */
    uint8_t    field_delimiter_6;    /* , */

    
    uint8_t     speed_over_gd[3];    /* x.x */    
    uint8_t    field_delimiter_7;    /* , */

    uint8_t    cource_over_gd[3];    /* x.x */   
    uint8_t    field_delimiter_8;    /* , */

    uint8_t              data[6];    /* xxxxxx */
    uint8_t    field_delimiter_9;    /* , */

    uint8_t    magnetic_varia[3];    /* x.x */ 
    uint8_t   field_delimiter_10;    /* , */

    uint8_t           degree_e_w;    /* a */
    uint8_t   field_delimiter_11;    /* , */

    uint8_t       mode_indicator;    /* a */     
    
    uint8_t   checksum_delimiter;    /* * */
    uint8_t    checksum_field[2];    /* hh */
    
    uint8_t   frame_terminate[2];    /* <CR><CF> */
    
}nmea_frame_rmc_st, *nmea_frame_rmc_st_ptr;

#define NMEA_FRAME_RMC_ST_LEN    (sizeof(nmea_frame_rmc_st))



/* NMEA0183 frame: GSA - . */
typedef struct _nmea_frame_gsa_st
{
    /* $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxxxx,x.x,a,a*hh<CR><LF> */

    nmea_frame_head_st      head;    /* common head. */

    uint8_t  position_fix_utc[9];    /* hhmmss.ss */
    uint8_t    field_delimiter_1;    /* , */

    uint8_t               status;    /* A */
    uint8_t    field_delimiter_2;    /* , */

    uint8_t          latitude[7];    /* llll.ll */
    uint8_t    field_delimiter_3;    /* , */

    uint8_t          latitude_ns;    /* a */ 
    uint8_t    field_delimiter_4;    /* , */

    uint8_t         longitude[8];    /* yyyyy.yy */
    uint8_t    field_delimiter_5;    /* , */

    uint8_t         longitude_ew;    /* a */
    uint8_t    field_delimiter_6;    /* , */

    
    uint8_t     speed_over_gd[3];    /* x.x */    
    uint8_t    field_delimiter_7;    /* , */

    uint8_t    cource_over_gd[3];    /* x.x */   
    uint8_t    field_delimiter_8;    /* , */

    uint8_t              data[6];    /* xxxxxx */
    uint8_t    field_delimiter_9;    /* , */

    uint8_t    magnetic_varia[3];    /* x.x */ 
    uint8_t   field_delimiter_10;    /* , */

    uint8_t           degree_e_w;    /* a */
    uint8_t   field_delimiter_11;    /* , */

    uint8_t       mode_indicator;    /* a */     
    
    uint8_t   checksum_delimiter;    /* * */
    uint8_t    checksum_field[2];    /* hh */
    
    uint8_t   frame_terminate[2];    /* <CR><CF> */
    
}nmea_frame_gsa_st, *nmea_frame_gsa_st_ptr;

#define NMEA_FRAME_GSA_ST_LEN    (sizeof(nmea_frame_gsa_st))











/* Longitude resolution for screen. uint: pixel/degree. */
#define RESOLUTION_LONGITUDE_DEFAULT    750//(7500)
#define RESOLUTION_LONGITUDE_MAX        (1000000)
#define RESOLUTION_LONGITUDE_MIN        (100)

/* Latitude resolution for screen. uint: pixel/degree. */
#define RESOLUTION_LATITUDE_DEFAULT     750//(7500)
#define RESOLUTION_LATITUDE_MAX         (1000000)
#define RESOLUTION_LATITUDE_MIN         (100)

/* Local vehicle's coordinate. */
#define LOCAL_COORDINATE_X               128  
#define LOCAL_COORDINATE_Y               135







/* Vehicle display structure. */  
typedef drv_lcd_circle_draw_st vec_disp_st, *vec_disp_st_ptr;

#define VEC_DISP_ST_LEN    (sizeof(vec_disp_st))
 

/* Vehicles information structure. */
typedef struct _vec_graph_st
{
    /* Screen resolution for longitude and latitude. uint: pixel/degree.*/
    uint32_t resolution_longitude;
    uint32_t  resolution_latitude;

    /* Vehicle parameters group for display. */
    uint32_t              vec_num;
    vec_disp_st     vec_group[20];

}vec_graph_st, *vec_graph_st_ptr;

#define VEC_GRAPH_ST_LEN    (sizeof(vec_graph_st))



/* Vehicle parameter structure.-------------------------------------------*/
typedef struct _vec_param_st
{
    uint8_t      road_mode;
    uint8_t       vec_mode;
    uint8_t breakdown_mode;

}vec_param_st, *vec_param_st_ptr;

#define VEC_PARAM_ST_LEN    (sizeof(vec_param_st))

/* Macro for "road mode". */
#define VEC_ROADMODE_HIGHWAY    0x00
#define VEC_ROADMODE_MOUNTAIN   0x01
#define VEC_ROADMODE_CITY       0x02

/* Macro for "vec_mode". */
#define VEC_VECMODE_CAR         0x00
#define VEC_VECMODE_AMBULANCE   0x01

/* Macro for "breakdown_mode". */
#define VEC_BREAKDOWNMODE_NO    0x00
#define VEC_BREAKDOWNMODE_YES   0x01


/* System parameter structure.--------------------------------------------*/
typedef struct _sys_param_st
{
    uint8_t       sys_mode;
    
    vec_param_st vec_param;
    uint32_t    alarm_stat;
    
}sys_param_st, *sys_param_st_ptr;

#define SYS_PARAM_ST_LEN    (sizeof(sys_param_st))

/* Macro for "sys_mode". */
#define SYS_SYSMODE_NORMAL      0x00
#define SYS_SYSMODE_ALARM       0x01





     
     


/* Thread stack size and priority. */
#define LCD_THREAD_STACK_SIZE    (1024*2)
#define LCD_THREAD_PRIORITY      25


/**
  * @brief  Initialize lcd thread.
  * @param  None.
  * @retval None.
  */
extern void lcd_thread_init
(
    /* No parameter. */
    void
);



     
#ifdef __cplusplus
}
#endif

#endif /* _CV_LCD_H_ */
