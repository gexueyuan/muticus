/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_vam_util.c
 @brief  : this file include some common function
 @author : wangyifeng
 @history:
           2014-6-24    wangyifeng    Created file
           ...
******************************************************************************/
#include <math.h>
#include "cv_osal.h"

#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "vam_util"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "nmea.h"
#include "arm_math.h"



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/

#define EARTH_RADIUS  6371.004f
//#define PI 3.1415926f
#define RAD(d) ((d)*PI/180.0f)


#define _COMPILE_INLINE__


_COMPILE_INLINE__ uint16_t cv_ntohs(uint16_t s16);

__COMPILE_INLINE__ uint32_t cv_ntohl(uint32_t l32);

__COMPILE_INLINE__ float cv_ntohf(float f32);


__COMPILE_INLINE__ int32_t encode_longtitude(float x);


__COMPILE_INLINE__ float decode_longtitude(uint32_t x);

#define encode_latitude(x) encode_longtitude(x) 
#define decode_latitude(x) decode_longtitude(x) 

#define encode_accuracy(x) encode_longtitude(x) 
#define decode_accuracy(x) decode_longtitude(x) 

__COMPILE_INLINE__ uint16_t encode_elevation(float x);

__COMPILE_INLINE__ float decode_elevation(uint16_t x);

__COMPILE_INLINE__ uint16_t encode_speed(float x);


__COMPILE_INLINE__ float decode_speed(uint16_t x);

__COMPILE_INLINE__ uint16_t encode_heading(float x);

__COMPILE_INLINE__ float decode_heading(uint16_t x);

__COMPILE_INLINE__ uint16_t encode_acce_lon(float x);

__COMPILE_INLINE__ float decode_acce_lon(uint16_t x);
__COMPILE_INLINE__ uint16_t encode_acce_lat(float x);

__COMPILE_INLINE__ float decode_acce_lat(uint16_t x);

__COMPILE_INLINE__ uint16_t encode_acce_vert(float x);

__COMPILE_INLINE__ float decode_acce_vert(uint16_t x);

__COMPILE_INLINE__ uint8_t encode_acce_yaw(float x);

__COMPILE_INLINE__ float decode_acce_yaw(uint8_t x);







/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

#if 0
static float getDistanceVer1(float lat1, float lng1, float lat2, float lng2)

{

   float radLat1 = rad(lat1);

   float radLat2 = rad(lat2);

   float radLng1 = rad(lng1);

   float radLng2 = rad(lng2);

   float s = acos(sin(radLat1)*sin(radLat2)+cos(radLat1)*cos(radLat2)*cos(radLng1-radLng2));

   s = s * EARTH_RADIUS;

   return s;

}
#endif
void print_f(float* data_f)
{
    char strbuf[64] = {0};
    memset(strbuf, 0x0, sizeof(strbuf));
    sprintf(strbuf, "%3.8f", *data_f);
    osal_printf("dis2 is %s\n",strbuf);


}


float getDistanceVer2(float lat1, float lng1, float lat2, float lng2)

{
    float pSrc;
    float pResult1;
    float pResult2;

    float pOut;

    float s;
   //float radLat1 = RAD(lat1);

   //float radLat2 = RAD(lat2);

    float a = lat1 - lat2;//radLat1 - radLat2;

    float b = lng1 - lng2;//RAD(lng1) - RAD(lng2);

    pSrc = arm_sin_f32(a/2);//sin(a/2);//
    arm_mult_f32(&pSrc,&pSrc,&pResult1,1);
    //arm_power_f32(&pSrc,1,&pResult1);


    pSrc = arm_sin_f32(b/2);//sin(b/2);//    
    arm_mult_f32(&pSrc,&pSrc,&pResult2,1);
    
    //arm_power_f32(&pSrc,1,&pResult2);

    arm_sqrt_f32((pResult1 + arm_cos_f32(lat1)*arm_cos_f32(lat2)*pResult2),&pOut);

    //pOut = sqrt((pResult1 + arm_cos_f32(lat1)*arm_cos_f32(lat2)*pResult2));

    s = 2 * asinf(pOut);

    //s = 2 * asin(sqrt(pow(arm_sin_f32(a/2),2) + arm_cos_f32(lat1)*arm_cos_f32(lat2)*pow(arm_sin_f32(b/2),2)));
    //s = 2 * asin(sqrt(pResult1 + arm_cos_f32(lat1)*arm_cos_f32(lat2)*pResult2));
    //s = 2 * asin(sqrt(pow(sin(a/2),2) + cos(lat1)*cos(lat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    return s;

}

float vsm_get_distance(vam_position_t *p_src, vam_position_t *p_dest)
{
    float d = 1000.0;

    d *= getDistanceVer2(p_src->lat, p_src->lon, p_dest->lat, p_dest->lon);

    #if 0
    {
        char str[64];
        sprintf(str,"%f", d);
        osal_printf("distance:%s\n", str);
    }
    #endif

    return d;
}


const char *_directfromangle(int angle)
{
    static const char *dir[] = {
        "北",
        "东北",
        "东",
        "东南",
        "南",
        "西南",
        "西",
        "西北",
    };

    int i;

    if (angle <= 10){
        i = 0;
    }
    else if(angle < 80){
        i = 1;
    }
    else if(angle <= 100){
        i = 2;
    }
    else if(angle < 170){
        i = 3;
    }
    else if(angle <= 190){
        i = 4;
    }
    else if(angle < 260){
        i = 5;
    }
    else if(angle <= 280){
        i = 6;
    }
    else if(angle < 350){
        i = 7;
    }
    else{
        i = 0;
    }

    return dir[i];
}
vam_pos_data vsm_get_data(vam_stastatus_t *p_src, vam_stastatus_t *p_dest)
{
    float lat1, lng1, lat2, lng2, lat3, lng3;
    float distance_1_2, distance_2_3;
    float angle;
    vam_pos_data  pos_data;
    
    /* reference point */
    lat1 = p_src->pos.lat;
    lng1 = p_src->pos.lon;

    /* destination point */
    lat2 = p_dest->pos.lat;
    lng2 = p_dest->pos.lon;

    /* temp point */
    lat3 = lat1;
    lng3 = lng2;

    distance_1_2 = getDistanceVer2(lat1, lng1, lat2, lng2);
    distance_2_3 = getDistanceVer2(lat2, lng2, lat3, lng3);
    angle = acosf(distance_2_3/distance_1_2)*180/PI;
    
    pos_data.distance_1_2 = distance_1_2;
    pos_data.distance_2_3 = distance_2_3;
    pos_data.angle = angle;

    return pos_data;

}

float vsm_get_pos(vam_stastatus_t *p_src, vam_stastatus_t *p_dest,vam_pos_data *pos_data)
{
    float lat1, lng1, lat2, lng2;
    float distance_1_2;
    float angle, delta;

    /* reference point */
    lat1 = p_src->pos.lat;
    lng1 = p_src->pos.lon;

    /* destination point */
    lat2 = p_dest->pos.lat;
    lng2 = p_dest->pos.lon;


    distance_1_2 = pos_data->distance_1_2;
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
    if (angle > p_src->dir){
        delta = angle - p_src->dir;
    }
    else{
        delta = p_src->dir - angle;
    }

    if (delta > 180){
        delta = 360 - delta;
    }

    distance_1_2 *= 1000; /* convert from Km to m */            

    return (delta <= 45)? distance_1_2:(-distance_1_2);
}

float vsm_get_relative_pos(vam_stastatus_t *p_src, vam_stastatus_t *p_dest)
{
    float lat1, lng1, lat2, lng2, lat3, lng3;
    float distance_1_2, distance_2_3;
    float angle, delta;

    /* reference point */
    lat1 = p_src->pos.lat;
    lng1 = p_src->pos.lon;

    /* destination point */
    lat2 = p_dest->pos.lat;
    lng2 = p_dest->pos.lon;

    /* temp point */
    lat3 = lat1;
    lng3 = lng2;

    distance_1_2 = getDistanceVer2(lat1, lng1, lat2, lng2);
    distance_2_3 = getDistanceVer2(lat2, lng2, lat3, lng3);
    angle = acosf(distance_2_3/distance_1_2)*180/PI;

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
    if (angle > p_src->dir){
        delta = angle - p_src->dir;
    }
    else{
        delta = p_src->dir - angle;
    }

    if (delta > 180){
        delta = 360 - delta;
    }

    distance_1_2 *= 1000; /* convert from Km to m */            

    return (delta <= 45)? distance_1_2:(-distance_1_2);
}

float vsm_get_relative_dir(const vam_stastatus_t *p_src, const  vam_stastatus_t *p_dest)
{
    float delta;

    /* calculate the angle detra between local front and remote position  */
    if (p_dest->dir > p_src->dir){
        delta = p_dest->dir - p_src->dir;
    }
    else{
        delta = p_src->dir - p_dest->dir;
    }

    if (delta > 180){
        delta = 360 - delta;
    }

    return delta;
}

int8_t vsm_get_rear_dir(vam_stastatus_t *p_dest)
{
	if((p_dest->dir>270)||(p_dest->dir<90))
		return -1;
	else return 1;
	
}


/* calculate the real time current value */
int32_t vsm_get_dr_current(vam_stastatus_t *last, vam_stastatus_t *current)
{
    float deltaT = 0.0;
    float v, s, dR;
    float dir, lon1, lat1, lon2, lat2; /* Radians */
  	uint32_t t = osal_get_systemtime();
    
	if(!last || !current)
    {
        return -1;
    }

    deltaT = ((t>=last->time) ? (t-last->time) : \
             (t+RT_UINT32_MAX - last->time)) / 1000.0f;

    memcpy(current, last, sizeof(vam_stastatus_t));
    if(deltaT <= 20 || (last->speed < 10))
    {
        return 0;
    }
    
    /* deltaT != 0, the calculate the "current" value */
    lon1 = (float)last->pos.lon;//RAD((float)last->pos.lon);
    lat1 = (float)last->pos.lat;//RAD((float)last->pos.lat);
    dir = RAD((float)last->dir);
    
    /* uniform rectilinear motion */ 
    v = last->speed / 3.6f;
    s = v*deltaT; 
    
	/* lat2 = asin( sin lat1 * cos dR + cos lat1 * sin dR * cos θ )
    lon2 = lon1 + atan2( sin θ * sin dR * cos lat1, cos dR- sin lat1 * sin lat2 )
    where lat is latitude, lon is longitude, θis the bearing (clockwise from north), 
    dR is the angular distance d/R; d being the distance travelled, R the earth’s radius */
    dR = s / EARTH_RADIUS / 1000.0f;
    lat2 = asin(arm_sin_f32(lat1)*arm_cos_f32(dR) + arm_cos_f32(lat1)*arm_sin_f32(dR)*arm_cos_f32(dir));
    lon2 = lon1 + atan2(arm_sin_f32(dir)*arm_sin_f32(dR)*arm_cos_f32(lat1), arm_cos_f32(dR)-arm_sin_f32(lat1)*arm_sin_f32(lat2));

    current->time = t;

    current->pos.lon = lon2 ;//* 180.0 / PI;
    current->pos.lat = lat2 ;//* 180.0 / PI;
#if 0
    char buf[100];
    sprintf(buf, "(lon=%f,lat=%f),h=%f,d=%f,s=%f,v=%f", current->pos.lon, current->pos.lat, current->dir, 
                                      vsm_get_relative_pos(last, current, 0), s, v);
    osal_printf("c%d%s\r\n", t, buf);
#endif
    return 0;
}

