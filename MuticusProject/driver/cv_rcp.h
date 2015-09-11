/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_rcp.h
 @brief  : this file include the definitions of  the remote communicate 
           protocol of vehicle
 @author : wangyifeng
 @history:
           2014-6-22    wangyifeng    Created file
           2015-1-20    wanglei       Modified. add dsrc msg
           ...
******************************************************************************/
#ifndef __CV_RCP_H__
#define __CV_RCP_H__

#include "cv_wnet.h"

#define __COMPILE_PACK__    __packed
#define __COMPILE_INLINE__ 


/*****************************************************************************
 * definition of micro                                                       *
*****************************************************************************/



/*****************************************************************************
 * definition of structs                                                     *
*****************************************************************************/

typedef enum _DSRC_MSG_ID {
	DSRCmsgID_reserved	                  = 0,
	DSRCmsgID_alaCarteMessage	          = 1, /* ACM */
	DSRCmsgID_basicSafetyMessage	      = 2, /* BSM, heartbeat msg */
	DSRCmsgID_basicSafetyMessageVerbose   = 3, /* used for testing only */
	DSRCmsgID_commonSafetyRequest	      = 4,
	DSRCmsgID_emergencyVehicleAlert	      = 5,
	DSRCmsgID_intersectionCollisionAlert  = 6,
	DSRCmsgID_mapData	                  = 7, /* MAP, GID, intersections */
	DSRCmsgID_nmeaCorrections	          = 8,
	DSRCmsgID_probeDataManagement	      = 9,
	DSRCmsgID_probeVehicleData	          = 10,
	DSRCmsgID_roadSideAlert	              = 11,
	DSRCmsgID_rtcmCorrections             = 12,
	DSRCmsgID_signalPhaseAndTimingMessage = 13,
	DSRCmsgID_signalRequestMessage	      = 14,
	DSRCmsgID_signalStatusMessage	      = 15,
	DSRCmsgID_travelerInformation	      = 16
	/*
	 * Enumeration is extensible
	 */
} E_DSRC_MSG_ID;

typedef enum _VehicleType {
	VehicleType_none	= 0,
	VehicleType_unknown	= 1,
	VehicleType_special	= 2,
	VehicleType_moto	= 3,
	VehicleType_car	    = 4,
	VehicleType_carOther	= 5,
	VehicleType_bus     	= 6,
	VehicleType_axleCnt2	= 7,
	VehicleType_axleCnt3	= 8,
	VehicleType_axleCnt4	= 9,
	VehicleType_axleCnt4Trailer	        = 10,
	VehicleType_axleCnt5Trailer     	= 11,
	VehicleType_axleCnt6Trailer	        = 12,
	VehicleType_axleCnt5MultiTrailer	= 13,
	VehicleType_axleCnt6MultiTrailer	= 14,
	VehicleType_axleCnt7MultiTrailer	= 15
} e_VehicleType;

typedef enum _ResponseType {
	ResponseType_notInUseOrNotEquipped	= 0,
	ResponseType_emergency	= 1,
	ResponseType_nonEmergency	= 2,
	ResponseType_pursuit	= 3
} e_ResponseType;

typedef enum _VehicleGroupAffected {
	VehicleGroupAffected_all_vehicles	= 9217,
	VehicleGroupAffected_bicycles	= 9218,
	VehicleGroupAffected_motorcycles	= 9219,
	VehicleGroupAffected_cars	= 9220,
	VehicleGroupAffected_light_vehicles	= 9221,
	VehicleGroupAffected_cars_and_light_vehicles	= 9222,
	VehicleGroupAffected_cars_with_trailers	= 9223,
	VehicleGroupAffected_cars_with_recreational_trailers	= 9224,
	VehicleGroupAffected_vehicles_with_trailers	= 9225,
	VehicleGroupAffected_heavy_vehicles	= 9226,
	VehicleGroupAffected_trucks	= 9227,
	VehicleGroupAffected_buses	= 9228,
	VehicleGroupAffected_articulated_buses	= 9229,
	VehicleGroupAffected_school_buses	= 9230,
	VehicleGroupAffected_vehicles_with_semi_trailers	= 9231,
	VehicleGroupAffected_vehicles_with_double_trailers	= 9232,
	VehicleGroupAffected_high_profile_vehicles	= 9233,
	VehicleGroupAffected_wide_vehicles	= 9234,
	VehicleGroupAffected_long_vehicles	= 9235,
	VehicleGroupAffected_hazardous_loads	= 9236,
	VehicleGroupAffected_exceptional_loads	= 9237,
	VehicleGroupAffected_abnormal_loads	= 9238,
	VehicleGroupAffected_convoys	= 9239,
	VehicleGroupAffected_maintenance_vehicles	= 9240,
	VehicleGroupAffected_delivery_vehicles	= 9241,
	VehicleGroupAffected_vehicles_with_even_numbered_license_plates	= 9242,
	VehicleGroupAffected_vehicles_with_odd_numbered_license_plates	= 9243,
	VehicleGroupAffected_vehicles_with_parking_permits	= 9244,
	VehicleGroupAffected_vehicles_with_catalytic_converters	= 9245,
	VehicleGroupAffected_vehicles_without_catalytic_converters	= 9246,
	VehicleGroupAffected_gas_powered_vehicles	= 9247,
	VehicleGroupAffected_diesel_powered_vehicles	= 9248,
	VehicleGroupAffected_lPG_vehicles	= 9249,
	VehicleGroupAffected_military_convoys	= 9250,
	VehicleGroupAffected_military_vehicles	= 9251
} e_VehicleGroupAffected;

typedef enum _ResponderGroupAffected {
    /* Default phrase, to be used when one of the below does not fit better */
	ResponderGroupAffected_emergency_vehicle_units	        = 9729,
	ResponderGroupAffected_federal_law_enforcement_units	= 9730,
	ResponderGroupAffected_state_police_units	            = 9731,
	ResponderGroupAffected_county_police_units	            = 9732,
	ResponderGroupAffected_local_police_units	            = 9733,
	ResponderGroupAffected_ambulance_units	                = 9734,
	ResponderGroupAffected_rescue_units	                    = 9735,
	ResponderGroupAffected_fire_units                   	= 9736,
	ResponderGroupAffected_hAZMAT_units	                    = 9737,
	ResponderGroupAffected_light_tow_unit               	= 9738,
	ResponderGroupAffected_heavy_tow_unit               	= 9739,
	ResponderGroupAffected_freeway_service_patrols         	= 9740,
	ResponderGroupAffected_transportation_response_units	= 9741,
	ResponderGroupAffected_private_contractor_response_units	= 9742
} e_ResponderGroupAffected;


typedef enum _Extent {
	Extent_useInstantlyOnly	= 0,
	Extent_useFor3meters	= 1,
	Extent_useFor10meters	= 2,
	Extent_useFor50meters	= 3,
	Extent_useFor100meters	= 4,
	Extent_useFor500meters	= 5,
	Extent_useFor1000meters	= 6,
	Extent_useFor5000meters	= 7,
	Extent_useFor10000meters	= 8,
	Extent_useFor50000meters	= 9,
	Extent_useFor100000meters	= 10,
	Extent_forever	= 127
} e_Extent;

/* vehicle event flags */
typedef enum _VehicleEventFlags {
    EventHazardLights               = 0x0001,
    EventStopLineViolation          = 0x0002, /* Intersection Violation */  
    EventABSactivated               = 0x0004,
    EventTractionControlLoss        = 0x0008,
    EventStabilityControlactivated  = 0x0010,
    EventHazardousMaterials         = 0x0020,
    EventEmergencyResponse          = 0x0040,
    EventHardBraking                = 0x0080,
    EventLightsChanged              = 0x0100,
    EventWipersChanged              = 0x0200,
    EventFlatTire                   = 0x0400,
    EventDisabledVehicle            = 0x0800,
}e_VehicleEventFlags;

typedef enum _TransmissionState {
    TRANS_STATE_neutral   = 0, /* Neutral, speed relative to the vehicle alignment */
    TRANS_STATE_park      = 1, /* Park */
    TRANS_STATE_Forward   = 2, /* Forward gears */
    TRANS_STATE_Reverse   = 3, /* Reverse gears */
    TRANS_STATE_reserved1 = 4, 
    TRANS_STATE_reserved2 = 5, 
    TRANS_STATE_reserved3 = 6, 
    TRANS_STATE_unavailable = 7,         
} e_TransmissionState;


typedef enum _BRAKE_STATE {
    BRAKE_STATE_NOT_EQUIPPED = 0,
    BRAKE_STATE_OFF          = 1,
    BRAKE_STATE_ON           = 2,
    BRAKE_STATE_ENGAGED      = 3,
} E_BRAKE_STATE;

typedef uint16_t itis_codes_t;
typedef uint16_t heading_slice_t;
    
typedef __COMPILE_PACK__ struct _rcp_msgid 
{
#if BIG_ENDIAN	
    uint8_t hops:3;
    uint8_t id:5;
#else
    uint8_t id:5;
    uint8_t hops:3;
#endif
} rcp_msgid_t;


typedef __COMPILE_PACK__ struct _rcp_position{
    int32_t lat;
    int32_t lon;
    int16_t elev;
    int32_t accu;
}rcp_position_t;

typedef __COMPILE_PACK__ struct _rcp_acceleration{
    uint16_t lon;
    uint16_t lat;
    uint16_t vert;
    uint8_t  yaw;
}rcp_acceleration_t;


typedef __COMPILE_PACK__ struct _rcp_motion{
    uint16_t speed;
    uint16_t heading; 
    rcp_acceleration_t  acce;                 
}rcp_motion_t;


typedef __COMPILE_PACK__ struct _rcp_msg_head
{
    rcp_msgid_t        msg_id;
    uint8_t         msg_count;
    uint8_t   temporary_id[4];
    
}rcp_msg_head_t;


/* DDateTime */
typedef __COMPILE_PACK__ struct DDateTime {
	uint16_t year;	    /* OPTIONAL */
	uint8_t	 month;     /* OPTIONAL */
	uint8_t	 day;	    /* OPTIONAL */
	uint8_t	 hour;	    /* OPTIONAL */
	uint8_t	 minute;	/* OPTIONAL */
	uint16_t second;	/* OPTIONAL */
} DDateTime_t;

typedef __COMPILE_PACK__ struct _ransmission_speed {
#ifdef BIG_ENDIAN	 
    uint16_t transmissionState:3;    /* e_TransmissionState */
    uint16_t speed:13;               /* (0..8191) -- Units of 0.02 m/s */
#else
    uint16_t speed:13;
    uint16_t transmissionState:3;
#endif
} transmission_speed_t;

/* FullPositionVector */
typedef __COMPILE_PACK__ struct _full_position_vector {
	//struct DDateTime utcTime;       /* OPTIONAL */
	int32_t	 lon;
	int32_t	 lat;
	int16_t  elev;  	            /* OPTIONAL */
	uint16_t heading;	            /* OPTIONAL */
	transmission_speed_t speed;	    /* OPTIONAL */
	int32_t	posAccuracy;	        /* OPTIONAL */
	int8_t	timeConfidence;	        /* OPTIONAL */
	int8_t	posConfidence;	        /* OPTIONAL */
	int16_t	speedConfidence;	    /* OPTIONAL */
} full_position_vector_t; 


/* VehicleSafetyExtension */
typedef __COMPILE_PACK__ struct _vehicle_safety_ext {
	uint16_t events;
    /* PathHistory: not supported yed */
} vehicle_safety_ext_t;

typedef __COMPILE_PACK__ struct _brake_system_status {
    uint8_t wheelBrakes:4;
    uint8_t wheelBrakesUnavailable:1; 
    uint8_t spareBit:1;
    uint8_t tcs:2;       /* TractionControlState */
    uint8_t abs:2;       /* AntiLockBrakeStatus */
    uint8_t scs:2;       /* StabilityControlStatus */
    uint8_t bba:2;       /* BrakeBoostApplied */
    uint8_t auxBrakes:2; /* AuxiliaryBrakeStatus */
} brake_system_status_t;


/* LSB: 3FF FFF */
typedef __COMPILE_PACK__ struct _VehicleSize {
#ifdef BIG_ENDIAN	 
    uint32_t width:12;       /* 0~1023, uint: 1cm */
    uint32_t length:12;      /* 0~4095, unit: 1cm */
#else
    uint32_t length:12;
    uint32_t width:12;
#endif
} vehicle_size_t;



/* MSG_BasicSafetyMessage(BSM) */
typedef __COMPILE_PACK__ struct _rcp_msg_basic_safty
{
    rcp_msg_head_t header;
    uint8_t forward_id[4];  /* 转发节点pid */
    uint16_t      dsecond;
    
    rcp_position_t        position;
    rcp_motion_t            motion;
    brake_system_status_t   brakes;
	vehicle_size_t	          size;
	vehicle_safety_ext_t safetyExt;
    
}rcp_msg_basic_safty_t;



/* MSG_RoadSideAlert(RSA)  */
typedef  __COMPILE_PACK__ struct _msg_roadside_alert{
    rcp_msgid_t   msg_id;
    uint8_t   msg_count;
    itis_codes_t typeEvent;
    itis_codes_t description[8];
	uint8_t	priority;
	heading_slice_t	heading;
	uint8_t	extent;
	full_position_vector_t	position;
	itis_codes_t	furtherInfoID;
    uint16_t  crc;
} rcp_msg_roadside_alert_t;



/* MSG_EmergencyVehicleAlert(EVA) */
typedef __COMPILE_PACK__ struct _msg_emergency_vehicle_alert{
    rcp_msgid_t   msg_id;
    uint8_t   temporary_id[4];
    uint8_t   forward_id[4];   /* 转发节点pid */
    rcp_msg_roadside_alert_t rsa;
    uint8_t	  responseType;   /* OPTIONAL */
    uint8_t	  details;        /* OPTIONAL */
    uint8_t   mass;           /* OPTIONAL */
    uint8_t	  basicType;      /* OPTIONAL */
    uint16_t  vehicleType;    /* OPTIONAL */
    uint16_t  responseEquip;  /* OPTIONAL */
    uint16_t  responderType;  /* OPTIONAL */
    uint16_t  crc;
}rcp_msg_emergency_vehicle_alert_t;



#endif /* __CV_RCP_H__ */

