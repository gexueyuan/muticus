
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "lip"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "nmea.h"

#define PI 3.1415926f
#define RAD(d) ((d)*PI/180.0f)

void lip_gps_proc(vam_envar_t *p_vam, uint8_t *databuf, uint32_t len)
{

    //osal_printf("%s\n",databuf);
    nmea_parse(databuf, len);
}


int lip_rmc_valid_check(t_nmea_rmc *p_rmc, uint32_t t)
{
#if 0 //TBD. 定位后才需要检查合法性
    vam_envar_t *p_vam = &cms_envar.vam;
    vam_position_t temp;
    float s1, v1, v2;
    float delta_t;
    
    temp.lon = (float)p_rmc->longitude;
    temp.lat = (float)p_rmc->latitude;
    
    delta_t = ((t >= p_vam->local.time) ? (t - p_vam->local.time) : \
             (t + RT_UINT32_MAX - p_vam->local.time)) / 1000.0;

    v1 = p_vam->local.speed / 3.6f;
    v2 = p_rmc->speed / 3.6f;

    s1 = vsm_get_distance(&temp, &p_vam->local.pos); 

#if 0
    s2 = v1*delta_t;
    char buf[100];
    sprintf(buf, "%d(lon=%f,lat=%f),h=%f,d=%f,s=%f,v=%f", t, temp.lon, temp.lat, p_rmc->heading, s1, s2, v1);
    osal_printf("g%s\r\n", buf);
#endif
    
    if(p_rmc->heading > 360.0 || (VAM_ABS(v1-v2)/delta_t > 9) || ((VAM_ABS(s1)/t) > 100))
    {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_WARN, "GPS ERROR\r\n");
        return RT_FALSE;
    }

 #endif  

    return 1;
}

void lip_update_local(t_nmea_rmc *p_rmc, float *p_accu)
{
    int res = 0;
	static uint8_t getGps = 0;
    vam_envar_t *p_vam = &cms_envar.vam;
    vam_stastatus_t last;
    vam_stastatus_t current;
    uint32_t now;
    memcpy(&last, &p_vam->local, sizeof(last));

    if (p_rmc){   
        now = osal_get_systemtime();
        res = lip_rmc_valid_check(p_rmc, now);
        if(!getGps)
        {
            p_vam->local.time = now;
            p_vam->local.pos.lat = RAD(p_rmc->latitude);
            p_vam->local.pos.lon = RAD(p_rmc->longitude);
            /* 解决停车speed<=3 heading=0的问题, 使用>5km/h的heading */
            if((p_rmc->speed >= 5.0f) && (p_rmc->heading <= 360.0f))
            {
                p_vam->local.dir = p_rmc->heading;
            }
            p_vam->local.speed = p_rmc->speed;
        }
        else if(res)
        {
            p_vam->local.time = now;
            p_vam->local.pos.lat = RAD(p_rmc->latitude);
            p_vam->local.pos.lon = RAD(p_rmc->longitude);
            /* 解决停车speed<=3 heading=0的问题, 使用>5km/h的heading */
            if((p_rmc->speed >= 5.0f) && (p_rmc->heading <= 360.0f))
            {
                p_vam->local.dir = p_rmc->heading;
            }
            p_vam->local.speed = p_rmc->speed;
        }
        else
        {
            vsm_get_dr_current(&last, &current);
            memcpy(&p_vam->local, &current, sizeof(vam_stastatus_t));
        }
        //dump_pos(&p_vam->local);
        if (last.speed != p_vam->local.speed)
        {
            vsm_update_bsm_bcast_timer(p_vam);
        }

        
        if (!(p_vam->flag&VAM_FLAG_GPS_FIXED)){
            p_vam->flag |= VAM_FLAG_GPS_FIXED;
            getGps = 1;
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "gps is captured.\n");
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "UTC: %d-%d-%d %d:%d:%d.%d\r\n",\
                    p_rmc->tt.year, p_rmc->tt.mon, p_rmc->tt.day, p_rmc->tt.hour, p_rmc->tt.min,\
                    p_rmc->tt.sec, p_rmc->tt.diffsec);
            if (p_vam->evt_handler[VAM_EVT_GPS_STATUS]){
                (p_vam->evt_handler[VAM_EVT_GPS_STATUS])((void *)1);
            }
        }

        /* refresh the timer */
        osal_timer_stop(p_vam->timer_gps_life);
        osal_timer_start(p_vam->timer_gps_life);

        if(p_vam->evt_handler[VAM_EVT_LOCAL_UPDATE]){
            (p_vam->evt_handler[VAM_EVT_LOCAL_UPDATE])(&p_vam->local); 
        }
    }

    if (p_accu){
        p_vam->local.pos.accu = *p_accu;
    }
}

void lip_update_local_acc(float x, float y, float z)
{
    vam_stastatus_t *p_local = &(p_vam_envar->local);
    p_local->acce.lon = x;   /* longitudinal axis: vehicle ahead */
    p_local->acce.lat = y;   /* lateral axis */
    p_local->acce.vert = z;  /* vertical axis */
}
