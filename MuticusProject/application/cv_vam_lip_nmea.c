#include <ctype.h>
#include <rthw.h>
#include "cv_osal.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "nmea.h"
#include "gps.h"

#define Grav_accel_value (9.80665f)	    //Gravitational acceleration
#define DRIVING_RUSH_ADD_THRESHOLD		(3.0f)

uint8_t IsLocate = __FALSE;
driving_action_st G_Action;


extern uint8_t drivint_step;
extern uint8_t	SHARP_SLOWDOWN_CNT;
extern float	SHARP_SLOWDOWN_THRESOLD;

void nmea_add(t_nmea_rmc *param);
int32_t nmea_rmc_time(char *pStrS, char *pStrE, t_time *tt);
int32_t nmea_rmc_date(char *pStrS, char *pStrE, t_time *tt);
int32_t nmea_rmc_lat(char *pStrS, char *pStrE, float *latitude);
int32_t nmea_rmc_lon(char *pStrS, char *pStrE, float *longitude);

void nmea_init(void) {
    p_mutex_gps = osal_mutex_create("mutex_gps");
}

void nmea_add(t_nmea_rmc *param) 
{
    static float speed_diff = 0.0 , old_speed = 0.0;
    static float angle_diff = 0.0 , old_heading = 0.0;
    static int rush_stop_time = 0;
    static int rush_add_time = 0;
    float vehicle_acce = 0.0;	//ahead acceleration

    char strbuf[100] = {0};

    //驾驶习惯发生点的信息
    static driving_rush_value_st G_tmp;

    float deltaT = 0.0;
    static uint32_t t0;
    uint32_t t1;


    osal_mutex_take(p_mutex_gps, OSAL_WAITING_FOREVER);

    t1 = osal_get_systemtime();

    deltaT = ((t1>=t0) ? (t1-t0) : \
    (t1+RT_UINT32_MAX - t0)) / 1000.0;

    //加速度计算
    speed_diff = param->speed - old_speed;					//速度差	km/h
    angle_diff = param->heading - old_heading;              //角度差
    G_Action.diff_angle = abs(angle_diff);
    G_Action.speed = param->speed;
    G_Action.is_locate = IsLocate;

    if (deltaT > 0.0f){
        vehicle_acce = speed_diff / 3.6f /deltaT;		//速度单位m/s 时间单位s
    }

#if 0
    sprintf(strbuf, "t=%d ms, v=%f, deltaV=%f, deltaT=%f, a=%f\r\n", t1, param->speed, 
            speed_diff, deltaT, vehicle_acce);
    osal_printf("%s\r\n", strbuf);
#else
    sprintf(strbuf, "acc=%f ", vehicle_acce);
#endif

    if (param->speed <= 1.0f && speed_diff <= 1.0f)
    {
        /* 车停 */
        G_Action.carRun = 0;
    }
    if(param->speed > 1.0f )
    {
        G_Action.carRun = 1;
        G_Action.vehicle_accel_value = vehicle_acce ;
        if(drivint_step < 2)
        {
            lip_update_local_acc(vehicle_acce, 0, 0);

            if(G_tmp.type == None)
            {
                if(vehicle_acce > DRIVING_RUSH_ADD_THRESHOLD)
                {				
                    G_tmp.type		= Rush_Add;				
                    G_tmp.value		= vehicle_acce;
                    rush_add_time++;
                    //osal_printf("NMEA->车辆开始急加速%s time=%d\n", strbuf, rush_add_time);
                }
                else if(vehicle_acce < SHARP_SLOWDOWN_THRESOLD)
                {
                    G_tmp.type		= Rush_Stop;
                    G_tmp.value		= vehicle_acce;
                    rush_stop_time++;

                    osal_printf("NMEA->车辆开始急减速%s time=%d\n", strbuf,rush_stop_time);
                }
            }
            else if(G_tmp.type == Rush_Add)
            {
                if(vehicle_acce > DRIVING_RUSH_ADD_THRESHOLD)	//持续ADD
                {
                    rush_add_time++;
                    //osal_printf("NMEA->车辆持续急加速中%s time=%d\n", strbuf, rush_add_time);
                }
                else
                {
                    rush_add_time = 0;
                    G_tmp.type = None;
                    //osal_printf("NMEA->车辆停止急加速\n");
                }
            }
            else if(G_tmp.type == Rush_Stop)
            {
                if(vehicle_acce < SHARP_SLOWDOWN_THRESOLD)	//持续STOP
                {
                    rush_stop_time++;
                    osal_printf("NMEA->车辆持续急减速中%s time=%d\n", strbuf,rush_stop_time);
                    if (rush_stop_time >= SHARP_SLOWDOWN_CNT)
                    {
                        /* 通知vsa模块处理 */
                        vam_gsnr_ebd_detected(1);
                    }
                }
                else
                {				
                    rush_stop_time = 0;
                    G_tmp.type = None;
                    osal_printf("NMEA->车辆停止急减速\n");
                }		
            }
        }
    }

    old_speed = param->speed;
    old_heading = param->heading;
    t0 = t1;

    osal_mutex_release(p_mutex_gps);
}

int32_t nmea_rmc_time(char *pStrS, char *pStrE, t_time *tt)
{
    uint8_t TmpBuff[5];

    memset(TmpBuff, 0, sizeof(TmpBuff));
    TmpBuff[0] = *(pStrS);
    TmpBuff[1] = *(pStrS + 1);
    tt->hour = (uint8_t) atoi((char *)TmpBuff);

    memset( TmpBuff, 0, sizeof(TmpBuff));
    TmpBuff[0] = *(pStrS + 2);
    TmpBuff[1] = *(pStrS + 3);
    tt->min = (uint8_t) atoi((char *)TmpBuff);

    memset( TmpBuff, 0, sizeof(TmpBuff));
    TmpBuff[0] = *(pStrS + 4);
    TmpBuff[1] = *(pStrS + 5);
    tt->sec = (uint8_t) atoi((char *)TmpBuff);

    TmpBuff[0] = *(pStrS + 7);
    TmpBuff[1] = *(pStrS + 8);
    TmpBuff[2] = *(pStrS + 9);
    tt->diffsec = atoi((char *)TmpBuff);

    return 0;
}

int32_t nmea_rmc_date(char *pStrS, char *pStrE, t_time *tt)
{
    uint8_t TmpBuff[5];

    memset(TmpBuff, 0, sizeof(TmpBuff));
    TmpBuff[0] = *(pStrS);
    TmpBuff[1] = *(pStrS + 1);
    tt->day = (uint8_t) atoi((char *)TmpBuff);

    memset(TmpBuff, 0, sizeof(TmpBuff));
    TmpBuff[0] = *(pStrS + 2);
    TmpBuff[1] = *(pStrS + 3);
    tt->mon = (uint8_t) atoi((char *)TmpBuff);

    memset(TmpBuff, 0, sizeof(TmpBuff));
    TmpBuff[0] = *(pStrS + 4);
    TmpBuff[1] = *(pStrS + 5);
    tt->year = (uint16_t) atoi((char *)TmpBuff) + 2000;

    return 0;
}

int32_t nmea_rmc_lat(char *pStrS, char *pStrE, float *latitude)
{
    uint8_t TmpBuff[15];
    uint16_t i = 0;

    if ( *(pStrE + 1) != 'N' && *(pStrE + 1) != 'S' && *(pStrS + 4) != '.'){
        return (-1);
    }

    //度
    memset(TmpBuff, 0, sizeof(TmpBuff));
    memcpy(TmpBuff, pStrS, 2);
    (*latitude) = (float)atoi((char *)TmpBuff);
    //分
    memset(TmpBuff, 0, sizeof(TmpBuff));
    for(i = 0; i < 8; i++) {
        if (isdigit(*(pStrS + 2 + i)) || *(pStrS + 2 + i) == '.') {
            TmpBuff[i] = *(pStrS + 2 + i);
        }
        else break;
    }
    (*latitude) += (atof((char *)TmpBuff) / 60.0);

    if ( *(pStrE + 1) == 'S' )  (*latitude) *= -1;

    return 0;
}

int32_t nmea_rmc_lon(char *pStrS, char *pStrE, float *longitude)
{
    uint8_t TmpBuff[15];
    uint16_t i = 0;

    if ( *(pStrE + 1) != 'W' && *(pStrE + 1) != 'E' && *(pStrS + 5) != '.'){
        return (-1);
    }

    //度
    memset(TmpBuff, 0, sizeof(TmpBuff));
    memcpy(TmpBuff, pStrS, 3);
    (*longitude) = (float)atoi((char *)TmpBuff);
    //分
    memset(TmpBuff, 0, sizeof(TmpBuff));
    //memcpy(TmpBuff, pStrS + 3, 8);
    for(i = 0; i < 8; i++) {
        if (isdigit(*(pStrS + 3 + i)) || *(pStrS + 3 + i) == '.') {
            TmpBuff[i] = *(pStrS + 3 + i);
        }
        else break;
    }
    (*longitude) += (atof((char *)TmpBuff) / 60.0);

    if ( *(pStrE + 1) == 'W' )  (*longitude) *= -1;

    return 0;
}

void nmea_parse(uint8_t *buff, uint32_t len)
{
    uint8_t crcCk = 0x00;
    uint8_t crcCp = 0x00;
    uint32_t index = 0;
    t_time tt;
    float latitude = 0.0;
    float longitude = 0.0;
    float speed = 0.0;
    float heading = 0.0;
    float accu = 0.0;
    char *pStrS = NULL;
    char *pStrE = NULL;
    e_nmea_type CurPackType = GPS_PACK_UNKNOWN;
    t_nmea_rmc nmeaRmc;

    if (memcmp(buff, "$GPRMC", 6) == 0){
        CurPackType = GPS_PACK_GPRMC;

        //NMEA_DEBUG("NMEA->%s\n", buff);
    }
    else if (memcmp(buff, "$GPGSA", 6) == 0){
        CurPackType = GPS_PACK_GPGSA;

        //NMEA_DEBUG("NMEA->%s\n", buff);
    }
    else return;
    //if (len <= 50) return;

    if (buff[len - 1] != '\n' || buff[len - 2] != '\r') {
        NMEA_DEBUG("NMEA->No \\r\\n\n");
        return;
    }
    if (buff[len - 5] != '*') {
        NMEA_DEBUG("NMEA->No *\n");
        return;
    }

    //Check CRC
    for (index = 1; index < len -5; index++) {
        crcCk ^= buff[index];
    }

    //Convent
    if (isdigit(buff[len - 4])) {
        crcCp = (buff[len - 4] << 4) & 0xf0;
    }
    else if (isalpha(buff[len - 4])) {
        crcCp = ((buff[len - 4] << 4) + 0x90 ) & 0xf0;
    }
    else {
        NMEA_DEBUG("NMEA->Error 0\n");
        return;
    }

    if (isdigit(buff[len - 3])) {
        crcCp |= (buff[len - 3] & 0x0f);
    }
    else if (isalpha(buff[len - 3])) {
        crcCp |= ((buff[len - 3] + 0x09 ) & 0x0f);
    }
    else {
        NMEA_DEBUG("NMEA->Error 1\n");
        return;
    }

    //Compare
    if (crcCk != crcCp) {
        NMEA_DEBUG("NMEA->数据帧CRC校验错误\n");
        return;
    }

    if (CurPackType == GPS_PACK_GPRMC){
        if (IsLocate != __TRUE) {
            NMEA_DEBUG("NMEA->定位精度不够\n");
            return;
        }

        pStrS = (char *)buff;
        if ((pStrS = strchr((char *)buff, ',')) != NULL) {
            pStrS++;
            if ((pStrS = strchr(pStrS, ',')) != NULL) {
                pStrS++;
            }
            else return;
        }
        else return;

        if ((*pStrS) == 'A') {
            pStrS = (char *)buff;
            //跳转一次逗号至时间
            if ((pStrS = strchr(pStrS, ',')) != NULL) {
                pStrS++;
                if ((pStrE = strchr(pStrS, ',')) == NULL) return;
            }
            else return;
            if (nmea_rmc_time(pStrS, pStrE, &tt) < 0) return;
            //跳转两次逗号至纬度
            if ((pStrS = strchr((pStrE+1), ',')) != NULL) {
                pStrS++;
                if ((pStrE = strchr(pStrS, ',')) == NULL) return;
            }
            else return;
            if (nmea_rmc_lat(pStrS, pStrE, &latitude) < 0) return;
            //跳转两次逗号至经度
            if ((pStrS = strchr((pStrE+1), ',')) != NULL) {
                pStrS++;
                if ((pStrE = strchr(pStrS, ',')) == NULL) return;
            }
            else return;
            if (nmea_rmc_lon(pStrS, pStrE, &longitude) < 0) return;
            //跳转两次逗号至速度
            if ((pStrS = strchr((pStrE+1), ',')) != NULL) {
                uint8_t Tmp[10];
                pStrS++;
                if ((pStrE = strchr(pStrS, ',')) == NULL) return;
                memset(Tmp, 0, sizeof(Tmp));
                memcpy(Tmp, pStrS, pStrE - pStrS);
                speed = atof((char *)Tmp)* 1.852f;
            }
            else return;

            // added by wangyf
            //跳转一次逗号至方向
            if ((pStrS = strchr((pStrS), ',')) != NULL) {
                uint8_t Tmp[10];
                pStrS++;
                if ((pStrE = strchr(pStrS, ',')) == NULL) return;
                memset(Tmp, 0, sizeof(Tmp));
                memcpy(Tmp, pStrS, pStrE - pStrS);
                heading = atof((char *)Tmp);
            }
            else return;
            //add end

            if ( (pStrS = strchr(pStrS + 1, ',')) != NULL &&\
                 (pStrE = strchr(pStrS + 1, ',')) != NULL &&\
                 nmea_rmc_date(pStrS + 1, pStrE, &tt) == 0)
            {
				nmeaRmc.isTrue = 1;
				nmeaRmc.latitude = latitude;
				nmeaRmc.longitude = longitude;
				nmeaRmc.speed = speed;
                nmeaRmc.heading = heading;
				memcpy(&(nmeaRmc.tt), &tt, sizeof(tt));

                lip_update_local(&nmeaRmc, NULL);
                nmea_add(&nmeaRmc);
                //UTC 时间
                //osal_printf("UTC: %d-%d-%d %d:%d:%d.%d\r\n", tt.year, tt.mon, tt.day, tt.hour, tt.min, tt.sec, tt.diffsec);
                //updata_rtc_time(&tt);
            }
            else return;
        }
        else {
            IsLocate = __FALSE;
            NMEA_DEBUG("NMEA->未定位\n");
        }
    }
    else if (CurPackType == GPS_PACK_GPGSA) {
        uint8_t Tmp[16];
        uint8_t i = 0;
        uint8_t IsError = __FALSE;

        //判断定位
        for (pStrS = (char *)buff, i = 0; i < 2; i++) {
            if ((pStrS = strchr(pStrS + 1, ',')) == NULL) {
                IsError = __TRUE;
                break;
            }
        }
        if (IsError == __FALSE) {
            pStrS++;
            if (*(pStrS) == ',') {
                IsLocate = __FALSE;
                NMEA_DEBUG("NMEA->未定位\n");
                return;
            }
            if ((pStrE = strchr(pStrS, ',')) == NULL) return;
            if (pStrE - pStrS > (sizeof(Tmp) - 1))  return;

            memset(Tmp, 0, sizeof(Tmp));
            memcpy(Tmp, pStrS, pStrE - pStrS);

            NMEA_DEBUG("NMEA->定位标志:%s\n", Tmp);

            if (atoi((char *)Tmp) == 1) {
                IsLocate = __FALSE;
                NMEA_DEBUG("NMEA->未定位\n");
                return;
            }
        }
        else {
            return;
        }
        //判断定位精度
        IsError = __FALSE;
        for (pStrS = (char *)buff, i = 0; i < 15; i++) {
            if ((pStrS = strchr(pStrS + 1, ',')) == NULL) {
                IsError = __TRUE;
                break;
            }
        }
        if (IsError == __FALSE) {
            pStrS++;
            if (*(pStrS) == ',') {
                IsLocate = __FALSE;
                NMEA_DEBUG("NMEA->定位精度不够\n");
                return;
            }
            if ((pStrE = strchr(pStrS, ',')) == NULL) return;
            if (pStrE - pStrS > (sizeof(Tmp) - 1))  return;

            memset(Tmp, 0, sizeof(Tmp));
            memcpy(Tmp, pStrS, pStrE - pStrS);

            NMEA_DEBUG("NMEA->定位精度:%s\n", Tmp);

            accu = (float)atof((char *)Tmp);

            if ( accu > 49.0f) {
                IsLocate = __FALSE;
                NMEA_DEBUG("NMEA->定位精度不够\n");
            }
            else {
                IsLocate = __TRUE;
                lip_update_local(NULL, &accu);
                NMEA_DEBUG("NMEA->有效定位精度\n");
            }
        }
        else {
            ;
        }
    }
}
