/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : app_wnet.h
 @brief  : this file include the application variables and functions prototypes for 
           the wireless network transport module.
 @author : wangxianwen
 @history:
           2015-6-29    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _APP_WNET_H_
#define _APP_WNET_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "drv_wnet.h"






     
     
/** @defgroup Constants
* @{
*/ 



/* Wnet receiver buffer length. */
#define WNET_RX_BUFFER_SIZE              (512)

/**
  * @}
  */
     



/*
 *  CAUTION:
 *      The sentence "declaration __attribute__ ((aligned(x)))" can not be working in this IDE. 
 *      We can use this parameter to make the storage larger then the natural size; but we can not make the storage 
 *  smaller than the natural size.
 *  
 * */

/* Save all the compiler settings. */
#pragma push

/* store data to reduce data size and off the optimization. */
#pragma pack(1)




/* Ethernet layer frame header for wnet module. */
typedef struct _wnet_enet_header_st
{
    /* Destination address. */
    uint8_t dest_address[6];

    /* Source address. */
    uint8_t  src_address[6];

    /* Ethernet type. */
    uint16_t     ether_type;

}wnet_enet_header_st, *wnet_enet_header_st_ptr;

#define WNET_ENET_HEADER_ST_LEN    (sizeof(wnet_enet_header_st))


/* Wnet main frame header for wnet module. */
typedef struct _wnet_main_header_st
{
    /* Magic number group for frame locator. */
    uint8_t    magic_num1;
    uint8_t    magic_num2;
    
    /* The rest data length based on uint8_t. */
    uint16_t       length;
    
    /* Two level frame type. */
    uint16_t f_major_type;
    uint16_t f_minor_type;
    
}wnet_main_header_st, *wnet_main_header_st_ptr;

#define WNET_MAIN_HEADER_ST_LEN    (sizeof(wnet_main_header_st))


/* Magic number for wnet main header. */
#define WNET_MAIN_HEADER_MAGIC_NUM1    0x55
#define WNET_MAIN_HEADER_MAGIC_NUM2    0xAA

/* Frame major type. */
#define WNET_F_MAJOR_TYPE_KEEPALIVE    0x0001
#define WNET_F_MAJOR_TYPE_CONFIG_ASK   0x0002
#define WNET_F_MAJOR_TYPE_EVENT_REPORT 0x0003
#define WNET_F_MAJOR_TYPE_RAW_DATA     0x0004

/* Frame minor type,difference according to major type. */
#define WNET_FT_MINOR_RAW_NULL         0x0000
#define WNET_FT_MINOR_RAW_NMEA         0x0001
#define WNET_FT_MINOR_RAW_DSRC         0x0002
#define WNET_FT_MINOR_RAW_ETHERNET     0x0003

#define WNET_FT_MINOR_RAW_USERDEFINE   0x0300





/* restore all compiler settings in stacks. */
#pragma pop








/**
  * @brief  wireless network receive main routine.
  * @param  See below.
  * @retval OK.
  */
extern void wnet_receive_main
(
    /* Dummy data pointer only for matching format. */
    void *dummy_ptr
);




     
#ifdef __cplusplus
}
#endif

#endif /* _APP_WNET_H_ */
