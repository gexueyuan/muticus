/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bsp_gnss_MAX7Q.h
 @brief  : This file include the prototype definitions for gnss module ublox MAX7Q.
 @author : wangxianwen
 @history:
           2015-06-09    wangxianwen    Create file. 
           ...
******************************************************************************/

#ifndef _BSP_GNSS_MAX7Q_H_
#define _BSP_GNSS_MAX7Q_H_

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes -----------------------------------------------------------------*/
#include "stm32f4xx.h"


/** @defgroup Exported_Constants
  * @{
  */ 


#define UBX_SYSN_CHAR1 0xB5
#define UBX_SYSN_CHAR2 0x62


typedef struct _ubx_pkt_hdr_st
{
    uint8_t syncChar1;
    uint8_t syncChar2;
    uint8_t   msgClass;
    uint8_t        msgId;
    uint16_t      length;
    
}ubx_pkt_hdr_st;

#define UBX_PKT_HDR_ST_LEN    (sizeof(ubx_pkt_hdr_st))


typedef struct _ubx_cfg_msg_st
{
    uint8_t  nmeaClass;
    uint8_t       nmeaid;
    uint8_t    portOn[6];     //send to port: i2c, uart1, uart2, usb, spi, other. 
    
}ubx_cfg_msg_st;

#define UBX_CFG_MSG_ST_LEN    (sizeof(ubx_cfg_msg_st))


/* Ublox NMEA configuration message id,based on NMEA 0183 Standard. */
typedef enum _ubx_cfg_msg_nmea_id
{
    STD_NMEA_ID_GGA = 0x00,    /* Global Positioning System Fix Data. */
    STD_NMEA_ID_GLL,                /* Geographic Position - Latitude/Longitude. */
    STD_NMEA_ID_GSA,               /* GNSS DOP and Active Satellites. */
    STD_NMEA_ID_GSV,               /* GNSS Satellites in View. */
    STD_NMEA_ID_RMC,               /* Recommended Minimum Specific GNSS Data. */
    STD_NMEA_ID_VTG,               /* Cource Over Ground and Ground Speed. */
    
    STD_NMEA_ID_GRS,               /* GNSS Range Residuals. */               
    STD_NMEA_ID_GST,               /* GNSS Pseudorange Error Statistics. */
    STD_NMEA_ID_ZDA,               /* Time & Data. */
    STD_NMEA_ID_GBS,               /* GNSS Satellite Fault Detection. */
    STD_NMEA_ID_DTM,              /* Datum Reference. */
    
    STD_NMEA_ID_END               /* End of NMEA command. */
                 
}ubx_cfg_msg_nmea_id;

#define UBX_CFG_MSG_NMEA_ID_LEN    (sizeof(ubx_cfg_msg_nmea_id))

























/**
  * @}
  */ 


/** @defgroup Exported_Functions
  * @{
  */ 



/**
  * @}
  */ 








#ifdef __cplusplus
}
#endif

#endif /* _BSP_GNSS_MAX7Q_H_ */
