/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : ublox.c
 @brief  : ublox gps  chip drvier
 @author : wanglei
 @history:
           2015-4-23    wanglei    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#include "components.h"
#include "gps.h"

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/
static void ubx_pkt_checknum_calc(uint8_t *buf, int len, uint8_t *cknumA, uint8_t *cknumB)
{
    uint16_t ckA=0, ckB=0;
    int i=0;
    for(i=0; i<len; i++)
    {
        ckA = ckA + buf[i];
        ckB = ckB + ckA;
    }
    *cknumA = ckA;
    *cknumB = ckB;
}

static void ubx_cfg_msg_std_nmea(ubx_cfg_msg_nmea_id_t nmea_id, uint8_t enable)
{
 	rt_device_t dev; 
    int len = 0;
    uint8_t buf[20];
 
    gps_ubx_pkt_hdr_t pkt;
    gps_ubx_cfg_msg_t cfg;
    memset(&cfg, 0x0, sizeof(cfg));
    pkt.syncChar1 = UBX_SYSN_CHAR1;
    pkt.syncChar2 = UBX_SYSN_CHAR2;
    pkt.msgClass = 0x06;
    pkt.msgId = 0x01;
    pkt.length = sizeof(gps_ubx_cfg_msg_t);
    len = sizeof(gps_ubx_pkt_hdr_t);
    memcpy(buf, &pkt, len);

    /* standard NMEA messages */
    cfg.nmeaClass = 0xF0;
    cfg.nmeaid = nmea_id;
    cfg.portOn[1] = enable;

    memcpy(buf+len, &cfg, pkt.length);
    len += pkt.length;

    ubx_pkt_checknum_calc(buf+2, len-2, &buf[len], &buf[len+1]); 
    len += 2;
	
    dev = rt_device_find(RT_GPS_DEVICE_NAME);

    rt_device_write(dev, 0, buf, len);    
}


/* UBX-CFG MSG. disable GPGGA/GPGLL/GPGSV/GPVTG msg */
static void ubx_cfg_needed_nmea(void)
{
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_GGA, 0);
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_GLL, 0);
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_GSV, 0);
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_VTG, 0);
}

/* UBX-CFG-PRT: set gps port baudrate to 115200 */
static void ubx_cfg_uart_port(void)
{
   /* set baut rate = 115200 */
    uint8_t cfg_pkt[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0,
        0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E
    };
    
	rt_device_t dev ; 
    dev = rt_device_find(RT_GPS_DEVICE_NAME);

    rt_device_write(dev, 0, cfg_pkt, sizeof(cfg_pkt));
}

void ubx_cfg_nmea_freq(uint8_t freq)
{
	rt_device_t dev; 
    uint8_t cfg_pkt[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 
                         0x01, 0x00, 0x01, 0x00, 0x0B, 0x77};

    switch (freq)
    {
        case 1:
        {
            cfg_pkt[6] = 0xE8;
            cfg_pkt[7] = 0x03;
            cfg_pkt[12] = 0x01;
            cfg_pkt[13] = 0x39;
            break;
        }
        case 2:
        {
            cfg_pkt[6] = 0xF4;
            cfg_pkt[7] = 0x01;
            cfg_pkt[12] = 0x0B;
            cfg_pkt[13] = 0x77;
            break;
        }
        case 5:  //5Hz
        {
            cfg_pkt[6] = 0xC8;
            cfg_pkt[7] = 0x00;
            cfg_pkt[12] = 0xDE;
            cfg_pkt[13] = 0x6A;
            break;
        }
        default:
        {
            break;
        }
    }
    dev = rt_device_find(RT_GPS_DEVICE_NAME);
    rt_device_write(dev, 0, cfg_pkt, sizeof(cfg_pkt));    
}

void gps_chip_config(int freq)
{
    /* get gps nmea. config needed nmea */
    ubx_cfg_needed_nmea();
    osal_delay(1);	
    /* conifg ublox gps rate 5Hz */
    ubx_cfg_nmea_freq(freq);
    osal_delay(1);
    /* config gps baud to 115200 */
    ubx_cfg_uart_port();
    osal_delay(1);
}
