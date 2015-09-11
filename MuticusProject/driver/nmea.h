#ifndef _NMEA_H_
#define _NMEA_H_
#include <stdint.h>
#include <rtthread.h>	

#define NMEA_DEBUG(...)

#ifndef __TRUE
 #define __TRUE         1
#endif
#ifndef __FALSE
 #define __FALSE        0
#endif

#define GPS_MAX_SIZE 3
#define GPS_WRITE_IN_WAKEUP 30

//GPS
#define CYC_GPS_MIN (20)
#define CYC_GPS_DEFALUT (30)

typedef struct _t_time
{
	uint16_t year;
    uint8_t mon;
    uint8_t day;
	uint8_t hour ;
	uint8_t min;
	uint8_t sec;

	uint32_t diffsec;
} t_time;

typedef struct{
    uint8_t isTrue;
    t_time updateTime;
    double speed;
    double latitude;
    double longitude;
    double  heading;
    t_time tt;
} t_nmea_rmc;


typedef enum{
    GPS_PACK_UNKNOWN = 0,
    GPS_PACK_GPGGA,
    GPS_PACK_GPGSA,
    GPS_PACK_GPGSV,
    GPS_PACK_GPRMC,
    GPS_PACK_GPVTG
} e_nmea_type;

void nmea_init(void);
int32_t nmea_get(t_nmea_rmc *recvBuff, int8_t flag);

void nmea_parse(uint8_t *buff, uint32_t len);

#endif
