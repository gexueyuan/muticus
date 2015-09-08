#ifndef __GSENSOR_H__
#define	__GSENSOR_H__

#include "board.h"

#ifdef HARDWARE_MODULE_V1
#define GSENSOR_LSM303DLHC
#endif

#define GSENSOR_BMA250E

#define GSNR_POLL_TIME_INTERVAL MS_TO_TICK(200)

typedef enum {
    GSNR_CRIT = 0,
    GSNR_ERR,
    GSNR_WARNING,
    GSNR_NOTICE,
    GSNR_INFO,
    GSNR_DEBUG,
}gsnr_log_level_t;

#if 0
#define GSNR_LOG(format, ...)  rt_kprintf(format, ##__VA_ARGS__)
#endif

#define GSNR_LOG(level, format, ...)  \
    do\
    {\
        if(level <= gsnr_log_lvl) \
            osal_printf(format, ##__VA_ARGS__); \
    }\
    while(0)

typedef struct _GSENSOR_INFO
{
	float x;
	float y;
	float z ;
	float sum;
}GSENSOR_INFO;

typedef struct
{
    uint8_t gsnr_cal_step;   /* gsensor calibrate and init step. default 0 */
    uint8_t gsnr_cal_thr;    /* shake threshold when static calibrating. default 4 */
    int8_t  gsnr_ebd_thr;     /* ebd thr. default -30 */
    int8_t  gsnr_ebd_cnt;     /* ebd count. default 2 */
    
    int32_t AcceV_x;
    int32_t AcceV_y;
    int32_t AcceV_z;
    int32_t AcceAhead_x;    
    int32_t AcceAhead_y;
    int32_t AcceAhead_z;
}gsnr_config_t;

extern GSENSOR_INFO gSensor_Average, gSensor_Threshold, Acce_Sum, Acce_V;



//==============================================================================
//                                   0ooo
//                          ooo0     (   ) 
//                          (   )     ) /
//                           \ (     (_/
//                            \_) 
//==============================================================================
#endif	/* __GSENSOR_H__ */

