#ifndef	__GPS_H__
#define	__GPS_H__


#include <stdint.h>
#include "nmea.h"

#define RT_GPS_DEVICE_NAME	"uart6"

#define GPSEVENT           0x0001

#define GPS_RX		     GPIO_Pin_10		// pa10
#define GPS_TX		     GPIO_Pin_9		// pa9

#define GPS_BUFF_SIZE   256
#define GPS_PIPE             5

extern osal_mutex_t * p_mutex_gps;


typedef struct 
{
	uint8_t Flag;
	uint8_t Buf[GPS_BUFF_SIZE];
	uint32_t Len;
    
} t_buff;

typedef struct 
{
	uint8_t Pipe;
	t_buff PpBuf[GPS_PIPE];
    
}t_gps_buff;


typedef void (*gps_data_callback)(uint8_t *, uint32_t);



extern t_gps_buff __GPSBuff;
//extern gps_data_callback gps_recv_cb;


#define UBX_SYSN_CHAR1 0xB5
#define UBX_SYSN_CHAR2 0x62

typedef enum ubx_cfg_msg_nmea_id
{
    STD_NMEA_ID_GGA = 0x00,
    STD_NMEA_ID_GLL,
    STD_NMEA_ID_GSA,
    STD_NMEA_ID_GSV,
    STD_NMEA_ID_RMC,
    STD_NMEA_ID_VTG,
    
    STD_NMEA_ID_GRS,
    STD_NMEA_ID_GST,
    STD_NMEA_ID_ZDA,
    STD_NMEA_ID_GBS,
    STD_NMEA_ID_DTM,
    
    STD_NMEA_ID_END,
                 
}ubx_cfg_msg_nmea_id_t;

typedef struct _ubx_pkt_hdr
{
    uint8_t syncChar1;
    uint8_t syncChar2;
    uint8_t msgClass;
    uint8_t msgId;
    uint16_t length;
}gps_ubx_pkt_hdr_t;

typedef struct _ubx_cfg_msg
{
    uint8_t nmeaClass;
    uint8_t nmeaid;
    uint8_t portOn[6]; //send to port: i2c, uart1, uart2, usb, spi, other. 
}gps_ubx_cfg_msg_t;


typedef enum {
	None		= 0,
	Rush_Add	= 1,
    Rush_Stop	= 2,
    Rush_Left	= 3,
	Rush_Right	= 4	
} driving_rush_type;

typedef struct {
	driving_rush_type type;	    /* type */
	float value;				/* acc value */
	t_time time;				/* time point */
	float latitude;
	float longitude;
	float speed;
} driving_rush_value_st;

typedef struct _driving_action_st
{
	float speed ;
	float vehicle_accel_value ;
	float diff_angle ;
	uint8_t is_locate ;
    uint8_t carRun;
}driving_action_st ;

extern driving_action_st G_Action;
extern uint8_t IsLocate;


void gps_chip_config(int freq);
void gps_callback_register(gps_data_callback fp);
void gps_init(void);
void gps_deinit(void);

void gps_task (void) ;

//==============================================================================
//                                   0ooo
//                          ooo0     (   ) 
//                          (   )     ) /
//                           \ (     (_/
//                            \_) 
//==============================================================================
#endif	/* __GPS_H__ */

