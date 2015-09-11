/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_dsmp.h
 @brief  : dsmp header file
 @author : wanglei
 @history:
           2014-12-11    wanglei    Created file
           ...
******************************************************************************/


#ifndef __CV_DSMP_H__
#define __CV_DSMP_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#define __COMPILE_PACK__  __packed

/*
    ----4 Byts---
    |security_hdr|sec_info|dsmp_ver|
    
*/


#define DSMP_VERSION 1
#define SEC_VERSION 2

#define DSMP_HEADER_LEN      sizeof(dsmp_hdr_t)
#define SECURITY_HEADER_LEN  sizeof(security_hdr_t)
#define SECURITY_INFO_LEN    sizeof(security_info_t)

typedef enum _WAVE_ELEMENT_ID {
    WAVE_ELEMENT_ID_RSV    = 0,
    WAVE_ELEMENT_ID_Service_info = 1,
    WAVE_ELEMENT_ID_channel_info = 2,
    WAVE_ELEMENT_ID_WRA = 3,    
    WAVE_ELEMENT_ID_Trans_Power = 4,
    WAVE_ELEMENT_ID_2D = 5,
    WAVE_ELEMENT_ID_3D = 6,
    WAVE_ELEMENT_ID_Advertiser_id = 7,
    WAVE_ELEMENT_ID_PSC = 8,
    WAVE_ELEMENT_ID_IPv6_Addr = 9,

    WAVE_ELEMENT_ID_WSM = 128,
    WAVE_ELEMENT_ID_WSMP_S = 129,
    WAVE_ELEMENT_ID_WSMP_I = 130,
    
} E_WAVE_ELEMENT_ID;

typedef enum _WAVE_AID {
    WAVE_AID_system                        = 0,
    WAVE_AID_automatic_fee_collection      = 1,
    WAVE_AID_freight_fleet_management      = 2,
    WAVE_AID_public_transport              = 3,
    WAVE_AID_traffic_traveler_information  = 4,
    WAVE_AID_traffic_control               = 5,
    WAVE_AID_parking_management            = 6,
    WAVE_AID_geographic_road_database      = 7,
    WAVE_AID_medium_range_preinformation   = 8,
    WAVE_AID_man_machine_interface         = 9,
    WAVE_AID_intersystem_interface         = 10,
    WAVE_AID_automatic_vehicle_identification = 11,
    WAVE_AID_emergency_warning             = 12,
    WAVE_AID_private                       = 13,
    WAVE_AID_multi_purpose_payment         = 14,
    WAVE_AID_dsrc_resource_manager         = 15,
    WAVE_AID_after_theft_systems           = 16,
    WAVE_AID_cruise_assist_highway_system  = 17,
    WAVE_AID_multi_purpose_information_system = 18, 
    WAVE_AID_public_safety                 = 19,
    WAVE_AID_vehicle_safety                = 20,
    WAVE_AID_general_purpose_internet_access = 21,
    WAVE_AID_onboard_diagnostics           = 22,
    WAVE_AID_security_manager              = 23,
    WAVE_AID_signed_WSA                    = 24,
} E_WAVE_AID;

typedef struct _wave_element {  
    uint8_t  element_id;
    uint8_t  length;
    uint8_t  *content;
} wave_element_t;

/* dsmp frame header, same as WSMP frame defined by IEEE1609.3 */
typedef __COMPILE_PACK__ struct _dsmp_hdr 
{
    uint8_t     version;
    uint32_t        aid;    /* "psid" of WSMP  */
    uint8_t  element_id;
    uint16_t dsm_length;
    
} dsmp_hdr_t;


/* 1609.2 frame header */
typedef struct _security_hdr 
{
    uint8_t       version;
    uint8_t security_type;
    uint8_t   reserved[2];
    
} security_hdr_t;

#define SECURITY_HEADER_LEN  sizeof(security_hdr_t)


/* encryption info. TBD */
typedef  struct _security_info 
{
    uint8_t enc_type;
    uint8_t   key[3];
    
} security_info_t;

#define SECURITY_INFO_LEN    sizeof(security_info_t)


typedef struct _dsa_hdr {
    uint8_t dsa_ver:6;
    uint8_t change_cnt:2;
    wave_element_t *ext;
} dsa_hdr_t;


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __CV_DSMP_H__ */
