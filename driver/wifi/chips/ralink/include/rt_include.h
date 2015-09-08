/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : rt_include.h
 @brief  : This is a combol include file of ralink's drivers
 @author : wangyifeng
 @history:
           2014-9-28    wangyifeng    Created file
           ...
******************************************************************************/
#include <string.h>
#include <stm32f4xx.h>
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#include "usb_bsp.h"
#include "usb_conf.h"
#include "usbh_core.h"
#include "usbh_stdreq.h"
#include "usb_bsp.h"
#include "usbh_ioreq.h"
#include "usbh_hcs.h"

#include "..\..\..\cv_drv_wifi_def.h"
#include "..\..\..\cv_drv_wifi_usb.h"

/*****************************************************************************
 * rtmp_type.h                                                                *
*****************************************************************************/
#include "rtmp_type.h"


/*****************************************************************************
 * rtmp_def.h                                                                *
*****************************************************************************/
#ifndef __RTMP_DEF_H__
#define __RTMP_DEF_H__

#define RALINK_2883_VERSION        ((UINT32)0x28830300)
#define RALINK_2880E_VERSION    ((UINT32)0x28720200)
#define RALINK_3883_VERSION        ((UINT32)0x38830400)
#define RALINK_3070_VERSION        ((UINT32)0x30700200)


#define HW_BEACON_MAX_NUM            1
#define MAX_MESH_NUM                0
#define MAX_APCLI_NUM                1
#define HW_BEACON_MAX_COUNT(__pAd)    HW_BEACON_MAX_NUM

#define DEFAULT_BBP_TX_POWER        0
#define DEFAULT_RF_TX_POWER         5
#define DEFAULT_BBP_TX_FINE_POWER_CTRL 0

/* RxFilter */
#define STANORMAL     0x17f97
#define APNORMAL     0x15f97


#define MAX_NUM_OF_CHANNELS 14    
#define MAX_AGGREGATION_SIZE    3840    /*3904 //3968 //4096 */

#define TXD_SIZE                16
#define TXWI_SIZE               16
#define RXD_SIZE                   16
#define RXWI_SIZE                 16
/* TXINFO_SIZE + TXWI_SIZE + 802.11 Header Size + AMSDU sub frame header */
#define TX_DMA_1ST_BUFFER_SIZE  96    /* only the 1st physical buffer is pre-allocated */

/* BW */
#define BAND_WIDTH_20        0
#define BAND_WIDTH_40        1
#define BAND_WIDTH_BOTH        2
#define BAND_WIDTH_10        3    /* 802.11j has 10MHz. This definition is for internal usage. doesn't fill in the IE or other field. */

#define BW_20        BAND_WIDTH_20
#define BW_40        BAND_WIDTH_40
#define BW_BOTH        BAND_WIDTH_BOTH
#define BW_10        BAND_WIDTH_10    /* 802.11j has 10MHz. This definition is for internal usage. doesn't fill in the IE or other field. */

/* All PHY rate summary in TXD */
/* Preamble MODE in TxD */
#define MODE_CCK    0
#define MODE_OFDM   1

/* MCS for CCK.  BW.SGI.STBC are reserved */
#define MCS_LONGP_RATE_1                      0    /* long preamble CCK 1Mbps */
#define MCS_LONGP_RATE_2                      1    /* long preamble CCK 1Mbps */
#define MCS_LONGP_RATE_5_5                    2
#define MCS_LONGP_RATE_11                     3
#define MCS_SHORTP_RATE_1                      4    /* long preamble CCK 1Mbps. short is forbidden in 1Mbps */
#define MCS_SHORTP_RATE_2                      5    /* short preamble CCK 2Mbps */
#define MCS_SHORTP_RATE_5_5                    6
#define MCS_SHORTP_RATE_11                     7
/* To send duplicate legacy OFDM. set BW=BW_40.  SGI.STBC are reserved */
#define MCS_RATE_6                      0    /* legacy OFDM */
#define MCS_RATE_9                      1    /* OFDM */
#define MCS_RATE_12                     2    /* OFDM */
#define MCS_RATE_18                     3    /* OFDM */
#define MCS_RATE_24                     4    /* OFDM */
#define MCS_RATE_36                     5    /* OFDM */
#define MCS_RATE_48                     6    /* OFDM */
#define MCS_RATE_54                     7    /* OFDM */

/*============================================================ */
/* ASIC WCID Table definition. */
/*============================================================ */
#define BSSID_WCID        1    /* in infra mode, always put bssid with this WCID */
#define MCAST_WCID    0x0
#define BSS0Mcast_WCID    0x0
#define BSS1Mcast_WCID    0xf8
#define BSS2Mcast_WCID    0xf9
#define BSS3Mcast_WCID    0xfa
#define BSS4Mcast_WCID    0xfb
#define BSS5Mcast_WCID    0xfc
#define BSS6Mcast_WCID    0xfd
#define BSS7Mcast_WCID    0xfe
#define RESERVED_WCID    0xff

/* pTxWI->txop */
#define IFS_HTTXOP                 0    /* The txop will be handles by ASIC. */
#define IFS_PIFS                    1
#define IFS_SIFS                    2
#define IFS_BACKOFF                 3

#endif //__RTMP_DEF_H__



/*****************************************************************************
 * rtmp_common.h                                                                *
*****************************************************************************/
#ifndef __RT_COMM_H__
#define __RT_COMM_H__

typedef enum _RTMP_INF_TYPE_
{    
    RTMP_DEV_INF_UNKNOWN = 0,
    RTMP_DEV_INF_PCI = 1,
    RTMP_DEV_INF_USB = 2,
    RTMP_DEV_INF_RBUS = 4,
    RTMP_DEV_INF_PCIE = 5,
}RTMP_INF_TYPE;

#define MAC_ADDR_LEN  6
#define COPY_MAC_ADDR(Addr1, Addr2) memcpy((Addr1), (Addr2), MAC_ADDR_LEN)

/* 2-byte Frame control field */
typedef struct GNU_PACKED {
#ifdef RT_BIG_ENDIAN
    USHORT Order:1;        /* Strict order expected */
    USHORT Wep:1;        /* Wep data */
    USHORT MoreData:1;    /* More data bit */
    USHORT PwrMgmt:1;    /* Power management bit */
    USHORT Retry:1;        /* Retry status bit */
    USHORT MoreFrag:1;    /* More fragment bit */
    USHORT FrDs:1;        /* From DS indication */
    USHORT ToDs:1;        /* To DS indication */
    USHORT SubType:4;    /* MSDU subtype */
    USHORT Type:2;        /* MSDU type */
    USHORT Ver:2;        /* Protocol version */
#else
        USHORT Ver:2;        /* Protocol version */
    USHORT Type:2;        /* MSDU type */
    USHORT SubType:4;    /* MSDU subtype */
    USHORT ToDs:1;        /* To DS indication */
    USHORT FrDs:1;        /* From DS indication */
    USHORT MoreFrag:1;    /* More fragment bit */
    USHORT Retry:1;        /* Retry status bit */
    USHORT PwrMgmt:1;    /* Power management bit */
    USHORT MoreData:1;    /* More data bit */
    USHORT Wep:1;        /* Wep data */
    USHORT Order:1;        /* Strict order expected */
#endif    /* !RT_BIG_ENDIAN */
} FRAME_CONTROL, *PFRAME_CONTROL;


typedef struct GNU_PACKED _HEADER_802_11 {
        FRAME_CONTROL   FC;
        USHORT          Duration;
        UCHAR           Addr1[MAC_ADDR_LEN];
        UCHAR           Addr2[MAC_ADDR_LEN];
    UCHAR            Addr3[MAC_ADDR_LEN];
#ifdef RT_BIG_ENDIAN
    USHORT            Sequence:12;
    USHORT            Frag:4;
#else
    USHORT            Frag:4;
    USHORT            Sequence:12;
#endif /* !RT_BIG_ENDIAN */
//    UCHAR            Octet[1];
}    HEADER_802_11, *PHEADER_802_11;

/* value domain of 802.11 header FC.Tyte, which is b3..b2 of the 1st-byte of MAC header */
#define BTYPE_MGMT                  0
#define BTYPE_CNTL                  1
#define BTYPE_DATA                  2

/* value domain of 802.11 MGMT frame's FC.subtype, which is b7..4 of the 1st-byte of MAC header */
#define SUBTYPE_ASSOC_REQ           0
#define SUBTYPE_ASSOC_RSP           1
#define SUBTYPE_REASSOC_REQ         2
#define SUBTYPE_REASSOC_RSP         3
#define SUBTYPE_PROBE_REQ           4
#define SUBTYPE_PROBE_RSP           5
#define SUBTYPE_BEACON              8
#define SUBTYPE_ATIM                9
#define SUBTYPE_DISASSOC            10
#define SUBTYPE_AUTH                11
#define SUBTYPE_DEAUTH              12
#define SUBTYPE_ACTION              13
#define SUBTYPE_ACTION_NO_ACK              14

/* value domain of 802.11 CNTL frame's FC.subtype, which is b7..4 of the 1st-byte of MAC header */
#define SUBTYPE_WRAPPER           7
#define SUBTYPE_BLOCK_ACK_REQ       8
#define SUBTYPE_BLOCK_ACK           9
#define SUBTYPE_PS_POLL             10
#define SUBTYPE_RTS                 11
#define SUBTYPE_CTS                 12
#define SUBTYPE_ACK                 13
#define SUBTYPE_CFEND               14
#define SUBTYPE_CFEND_CFACK         15

/* value domain of 802.11 DATA frame's FC.subtype, which is b7..4 of the 1st-byte of MAC header */
#define SUBTYPE_DATA                0
#define SUBTYPE_DATA_CFACK          1
#define SUBTYPE_DATA_CFPOLL         2
#define SUBTYPE_DATA_CFACK_CFPOLL   3
#define SUBTYPE_NULL_FUNC           4
#define SUBTYPE_CFACK               5
#define SUBTYPE_CFPOLL              6
#define SUBTYPE_CFACK_CFPOLL        7
#define SUBTYPE_QDATA               8
#define SUBTYPE_QDATA_CFACK         9
#define SUBTYPE_QDATA_CFPOLL        10
#define SUBTYPE_QDATA_CFACK_CFPOLL  11
#define SUBTYPE_QOS_NULL            12
#define SUBTYPE_QOS_CFACK           13
#define SUBTYPE_QOS_CFPOLL          14
#define SUBTYPE_QOS_CFACK_CFPOLL    15

#endif  //__RT_COMM_H__


/*****************************************************************************
 * rt_linux.h                                                                *
*****************************************************************************/
#ifndef __RT_LINUX_H__
#define __RT_LINUX_H__

typedef void                * PNDIS_PACKET;
typedef char                NDIS_PACKET;
typedef PNDIS_PACKET        * PPNDIS_PACKET;

#define NDIS_STATUS_SUCCESS                     0x00
#define NDIS_STATUS_FAILURE                     0x01
#define NDIS_STATUS_INVALID_DATA                0x02
#define NDIS_STATUS_RESOURCES                   0x03

struct usb_device_id {
    UINT16 idVendor;
    UINT16 idProduct;
};
typedef struct usb_device_id USB_DEVICE_ID;

#define USB_DEVICE(VID,PID) VID,PID

/*
 * Device Register I/O Access related definitions and data structures.
*/
#define RTMP_IO_FORCE_READ32(_A, _R, _pV)                                \
    RTUSBReadMACRegister((_A), (_R), (PUINT32) (_pV))

#define RTMP_IO_READ32(_A, _R, _pV)                                \
    RTUSBReadMACRegister((_A), (_R), (PUINT32) (_pV))

#define RTMP_IO_READ8(_A, _R, _pV)                                \
{                                                                \
}

#define RTMP_IO_WRITE32(_A, _R, _V)                                \
    RTUSBWriteMACRegister((_A), (_R), (UINT32) (_V))


#define RTMP_IO_WRITE8(_A, _R, _V)                                \
{                                                                \
    USHORT    _Val = _V;                                            \
    RTUSBSingleWrite((_A), (_R), (USHORT) (_Val));                                \
}


#define RTMP_IO_WRITE16(_A, _R, _V)                                \
{                                                                \
    RTUSBSingleWrite((_A), (_R), (USHORT) (_V));                                \
}

#define RTMP_IO_FORCE_WRITE32
#define RTMP_SYS_IO_READ32
#define RTMP_SYS_IO_WRITE32


#define RTMPusecDelay(us) USB_OTG_BSP_uDelay(us)
#define RtmpOsMsDelay(ms) USB_OTG_BSP_mDelay(ms)

/***********************************************************************************
 *    OS debugging and printing related definitions and data structure
 ***********************************************************************************/
#define PRINT_MAC(addr)    \
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]

#endif //__RT_LINUX_H__


/*****************************************************************************
 * rtmp_mac.h                                                                *
*****************************************************************************/
#include "rtmp_mac.h"


/*****************************************************************************
 * rt_user.h                                                                *
*****************************************************************************/
#ifndef __RT3070_USER_H__
#define __RT3070_USER_H__

#define RT3070        0x3070

#define INIT

#define USE_SYSTEM_EEPROM_DATA    1    //
#define USE_NEW_RX_MODE        0    //用处不大,暂不使用
#define USE_TEMP_VAR_IE            1    //    
#define USE_SIMPLE_TRANSMIT        0    //会造成不稳定，暂不使用

//#define RTMP_EFUSE_SUPPORT    1
#define RTMP_TIMER_TASK_SUPPORT    1
#define CONFIG_STA_SUPPORT
#define RTMP_RF_RW_SUPPORT        1
#define RT30xx                    1
#define RT33xx                    1
#define RT_CFG80211_SUPPORT
#define OS_ABL_OS_STA_SUPPORT    1    
#define WORKQUEUE_BH            1
#define IW_HANDLER_VERSION        0
#define WIRELESS_EXT            22
#define LINUX                    1
#define MEMORY_OPTIMIZATION        1
#define RTMP_USB_SUPPORT
#define RTMP_MAC_USB
#define WPA_SUPPLICANT_SUPPORT
#define SYSTEM_LOG_SUPPORT
#define ENABLE_SPECTRUM            0

#define MAC_TABLE_OFF            0

#define SHARED_KEY_NUM        1
#define USE_ONLY_ONE_KEY    1


#define IN
#define OUT
#define INOUT
#define NDIS_STATUS        INT

#endif //__RT3070_USER_H__

/*****************************************************************************
 * rtmp_phy.h                                                                *
*****************************************************************************/
#include "rtmp_phy.h"


/*****************************************************************************
 * mac_usb.h                                                                *
*****************************************************************************/
#if 0
#ifndef __MAC_USB_H__
#define __MAC_USB_H__

#define USB_CYC_CFG                0x02a4


/* 8051 firmware image for usb - use last-half base address = 0x3000 */
#define FIRMWARE_IMAGE_BASE            0x3000
#define MAX_FIRMWARE_IMAGE_SIZE        0x1000    /* 4kbyte */

#define RTMP_WRITE_FIRMWARE(_pAd, _pFwImage, _FwLen)        \
    RTUSBFirmwareWrite(_pAd, _pFwImage, _FwLen)
#endif
#else
#include "mac_usb.h"
#endif



/*****************************************************************************
 * rtmp.h                                                                *
*****************************************************************************/
#ifndef __RTMP_H__
#define __RTMP_H__

#include "rtmp_chip.h"

#define MAX_TXPOWER_ARRAY_SIZE    5


#define WAKE_MCU_CMD                0x31
#define SLEEP_MCU_CMD                0x30
#define RFOFF_MCU_CMD                0x35

/***************************************************************************
  *    Rx Path software control block related data structures
  **************************************************************************/
typedef struct _RX_BLK_
{
/*    RXD_STRUC        RxD; // sample */
    RT28XX_RXD_STRUC    RxD;
    PRXWI_STRUC            pRxWI;
    PHEADER_802_11        pHeader;
    PNDIS_PACKET        pRxPacket;
    UCHAR                *pData;
    USHORT                DataSize;
    USHORT                Flags;
    UCHAR                UserPriority;    /* for calculate TKIP MIC using */
    UCHAR                OpMode;    /* 0:OPMODE_STA 1:OPMODE_AP */
} RX_BLK;


/* structure to store channel TX power */
typedef struct _CHANNEL_TX_POWER {
    USHORT RemainingTimeForUse;    /*unit: sec */
    UCHAR Channel;
#ifdef DOT11N_DRAFT3
    BOOLEAN bEffectedChannel;    /* For BW 40 operating in 2.4GHz , the "effected channel" is the channel that is covered in 40Mhz. */
#endif                /* DOT11N_DRAFT3 */
    CHAR Power;
    CHAR Power2;
#ifdef DOT11N_SS3_SUPPORT
    CHAR Power3;
#endif                /* DOT11N_SS3_SUPPORT */
    UCHAR MaxTxPwr;
    UCHAR DfsReq;
    UCHAR RegulatoryDomain;

/*
    Channel property:
 
    CHANNEL_DISABLED: The channel is disabled.
    CHANNEL_PASSIVE_SCAN: Only passive scanning is allowed.
    CHANNEL_NO_IBSS: IBSS is not allowed.
    CHANNEL_RADAR: Radar detection is required.
    CHANNEL_NO_FAT_ABOVE: Extension channel above this channel is not allowed.
    CHANNEL_NO_FAT_BELOW: Extension channel below this channel is not allowed.
 */
#define CHANNEL_DEFAULT_PROP    0x00
#define CHANNEL_DISABLED        0x01    /* no use */
#define CHANNEL_PASSIVE_SCAN    0x02
#define CHANNEL_NO_IBSS            0x04
#define CHANNEL_RADAR            0x08
#define CHANNEL_NO_FAT_ABOVE    0x10
#define CHANNEL_NO_FAT_BELOW    0x20

    UCHAR Flags;

#ifdef RT30xx
    UCHAR Tx0FinePowerCtrl;    /* Tx0 fine power control in 0.1dB step */
    UCHAR Tx1FinePowerCtrl;    /* Tx1 fine power control in 0.1dB step */
    UCHAR Tx2FinePowerCtrl;    /* Tx2 fine power control in 0.1dB step */
#endif                /* RT30xx */
} CHANNEL_TX_POWER, *PCHANNEL_TX_POWER;


typedef struct _SOFT_RX_ANT_DIVERSITY_STRUCT {
    UCHAR EvaluatePeriod;    /* 0:not evalute status, 1: evaluate status, 2: switching status */
    UCHAR EvaluateStableCnt;
    UCHAR Pair1PrimaryRxAnt;    /* 0:Ant-E1, 1:Ant-E2 */
    UCHAR Pair1SecondaryRxAnt;    /* 0:Ant-E1, 1:Ant-E2 */
#ifdef CONFIG_STA_SUPPORT
    SHORT Pair1AvgRssi[2];    /* AvgRssi[0]:E1, AvgRssi[1]:E2 */
    SHORT Pair2AvgRssi[2];    /* AvgRssi[0]:E3, AvgRssi[1]:E4 */
#endif                /* CONFIG_STA_SUPPORT */
    SHORT Pair1LastAvgRssi;    /* */
    SHORT Pair2LastAvgRssi;    /* */
    ULONG RcvPktNumWhenEvaluate;
    BOOLEAN FirstPktArrivedWhenEvaluate;
} SOFT_RX_ANT_DIVERSITY, *PSOFT_RX_ANT_DIVERSITY;

typedef struct _COMMON_CONFIG {

    UCHAR Channel;
    UCHAR TxRate;
//    UCHAR TxPower[14];
    UCHAR MacAddr[6];
    UCHAR Mode;   /* 0: normal 1: qc test mode */
    
}COMMON_CONFIG, *PCOMMON_CONFIG;

typedef struct _RTMP_CHIP_CAP_ RTMP_CHIP_CAP;
typedef struct _RTMP_CHIP_OP_ RTMP_CHIP_OP;

struct _RTMP_ADAPTER{

    void *pUsb_Dev;

    UINT32 init_complete;
    
    rt_sem_t UsbVendorReq_semaphore;
    rt_sem_t UsbVendorReq_semaphore2;

    UCHAR LastMCUCmd;

/*****************************************************************************************/
/*      ASIC related parameters                                                          */
/*****************************************************************************************/
    UINT32 MACVersion;    /* MAC version. Record rt2860C(0x28600100) or rt2860D (0x28600101).. */

    /* --------------------------- */
    /* E2PROM */
    /* --------------------------- */
    ULONG EepromVersion;    /* byte 0: version, byte 1: revision, byte 2~3: unused */
    ULONG FirmwareVersion;    /* byte 0: Minor version, byte 1: Major version, otherwise unused. */
    USHORT EEPROMDefaultValue[NUM_EEPROM_BBP_PARMS];
    UCHAR EEPROMAddressNum;    /* 93c46=6  93c66=8 */
    BOOLEAN EepromAccess;
    UCHAR EFuseTag;


    /* --------------------------- */
    /* BBP Control */
    /* --------------------------- */
    UCHAR BbpWriteLatch[MAX_BBP_ID + 1];    /* record last BBP register value written via BBP_IO_WRITE/BBP_IO_WRITE_VY_REG_ID */
    //CHAR BbpRssiToDbmDelta;    /* change from UCHAR to CHAR for high power */
    //BBP_R66_TUNING BbpTuning;



    /* ---------------------------- */
    /* RFIC control */
    /* ---------------------------- */
    UCHAR RfIcType;        /* RFIC_xxx */
    ULONG RfFreqOffset;    /* Frequency offset for channel switching */

    RTMP_RF_REGS LatchRfRegs;    /* latch th latest RF programming value since RF IC doesn't support READ */

    EEPROM_ANTENNA_STRUC Antenna;    /* Since ANtenna definition is different for a & g. We need to save it for future reference. */
    EEPROM_NIC_CONFIG2_STRUC NicConfig2;

    /* This soft Rx Antenna Diversity mechanism is used only when user set */
    /* RX Antenna = DIVERSITY ON */
    SOFT_RX_ANT_DIVERSITY RxAnt;

    CHANNEL_TX_POWER TxPower[MAX_NUM_OF_CHANNELS];    /* Store Tx power value for all channels. */
    CHANNEL_TX_POWER ChannelList[MAX_NUM_OF_CHANNELS];    /* list all supported channels for site survey */

    UCHAR ChannelListNum;    /* number of channel in ChannelList[] */
    //UCHAR Bbp94;
    //BOOLEAN BbpForCCK;
    ULONG Tx20MPwrCfgABand[MAX_TXPOWER_ARRAY_SIZE];
    ULONG Tx20MPwrCfgGBand[MAX_TXPOWER_ARRAY_SIZE];
    ULONG Tx40MPwrCfgABand[MAX_TXPOWER_ARRAY_SIZE];
    ULONG Tx40MPwrCfgGBand[MAX_TXPOWER_ARRAY_SIZE];


#ifdef VCORECAL_SUPPORT
    UCHAR LatchTssi;
    UCHAR RefreshTssi;
#endif /* VCORECAL_SUPPORT */

    BOOLEAN bAutoTxAgcA;    /* Enable driver auto Tx Agc control */
    UCHAR TssiRefA;        /* Store Tssi reference value as 25 temperature. */
    UCHAR TssiPlusBoundaryA[5];    /* Tssi boundary for increase Tx power to compensate. */
    UCHAR TssiMinusBoundaryA[5];    /* Tssi boundary for decrease Tx power to compensate. */
    UCHAR TxAgcStepA;    /* Store Tx TSSI delta increment / decrement value */
    CHAR TxAgcCompensateA;    /* Store the compensation (TxAgcStep * (idx-1)) */

    BOOLEAN bAutoTxAgcG;    /* Enable driver auto Tx Agc control */
    UCHAR TssiRefG;        /* Store Tssi reference value as 25 temperature. */
    UCHAR TssiPlusBoundaryG[5];    /* Tssi boundary for increase Tx power to compensate. */
    UCHAR TssiMinusBoundaryG[5];    /* Tssi boundary for decrease Tx power to compensate. */
    UCHAR TxAgcStepG;    /* Store Tx TSSI delta increment / decrement value */
    CHAR TxAgcCompensateG;    /* Store the compensation (TxAgcStep * (idx-1)) */
#ifdef RTMP_INTERNAL_TX_ALC
    TX_POWER_CONTROL TxPowerCtrl;    /* The Tx power control using the internal ALC */
#endif /* RTMP_INTERNAL_TX_ALC */

#ifdef RTMP_FREQ_CALIBRATION_SUPPORT
    FREQUENCY_CALIBRATION_CONTROL FreqCalibrationCtrl;    /* The frequency calibration control */
#endif /* RTMP_FREQ_CALIBRATION_SUPPORT */

    signed char BGRssiOffset0;    /* Store B/G RSSI#0 Offset value on EEPROM 0x46h */
    signed char BGRssiOffset1;    /* Store B/G RSSI#1 Offset value */
    signed char BGRssiOffset2;    /* Store B/G RSSI#2 Offset value */

    signed char ARssiOffset0;    /* Store A RSSI#0 Offset value on EEPROM 0x4Ah */
    signed char ARssiOffset1;    /* Store A RSSI#1 Offset value */
    signed char ARssiOffset2;    /* Store A RSSI#2 Offset value */

    CHAR BLNAGain;        /* Store B/G external LNA#0 value on EEPROM 0x44h */
    CHAR ALNAGain0;        /* Store A external LNA#0 value for ch36~64 */
    CHAR ALNAGain1;        /* Store A external LNA#1 value for ch100~128 */
    CHAR ALNAGain2;        /* Store A external LNA#2 value for ch132~165 */
#ifdef RT30xx
    /* for 3572 */
    //UCHAR Bbp25;
    //UCHAR Bbp26;

    UCHAR TxMixerGain24G;    /* Tx mixer gain value from EEPROM to improve Tx EVM / Tx DAC, 2.4G */
    UCHAR TxMixerGain5G;
#endif /* RT30xx */



    /* Ori is defined in MLME struct */
    UCHAR CaliBW40RfR24;
    UCHAR CaliBW20RfR24;
    UCHAR RealRxPath;



    RTMP_CHIP_OP chipOps;
    RTMP_CHIP_CAP chipCap;

/*****************************************************************************************/
/*      802.11 related parameters                                                        */
/*****************************************************************************************/
    /* outgoing BEACON frame buffer and corresponding TXD */
    TXWI_STRUC BeaconTxWI;
    PUCHAR BeaconBuf;
    USHORT BeaconOffset[HW_BEACON_MAX_NUM];

    /* configuration: read from Registry & E2PROM */
    UCHAR PermanentAddress[MAC_ADDR_LEN];    /* Factory default MAC address */
    UCHAR CurrentAddress[MAC_ADDR_LEN];    /* User changed MAC address */

    UCHAR CurrentChannel;
    UCHAR CurrentTxRate;

    /* current TX sequence # */
    USHORT Sequence;

    /* ------------------------------------------------------ */
    /* common configuration                                   */
    /* ------------------------------------------------------ */
    COMMON_CONFIG CommonCfg;

};
typedef struct _RTMP_ADAPTER RTMP_ADAPTER;
typedef struct _RTMP_ADAPTER *PRTMP_ADAPTER;


#define RTUSBWriteMACRegister(x,y,z) do_RTUSBWriteMACRegister(x,y,z)
#define RTUSBReadMACRegister(x,y,z)     do_RTUSBReadMACRegister(x,y,z)

#define GET_LNA_GAIN(_pAd)    ((_pAd->LatchRfRegs.Channel <= 14) ? (_pAd->BLNAGain) : ((_pAd->LatchRfRegs.Channel <= 64) ? (_pAd->ALNAGain0) : ((_pAd->LatchRfRegs.Channel <= 128) ? (_pAd->ALNAGain1) : (_pAd->ALNAGain2))))

#endif /* __RTMP_H__ */


/*****************************************************************************
 * rtmp_util.h                                                                *
*****************************************************************************/
#ifndef __RTMP_UTIL_H__
#define __RTMP_UTIL_H__

#define DEVICE_VENDOR_REQUEST_OUT       0x40
#define DEVICE_VENDOR_REQUEST_IN        0xc0


#define USBD_TRANSFER_DIRECTION_OUT        0
#define USBD_TRANSFER_DIRECTION_IN        0
#define USBD_SHORT_TRANSFER_OK            0


/* Endian byte swapping codes */
#ifndef SWAP16
#define SWAP16(x) \
    ((UINT16) (\
           (((UINT16) (x) & (UINT16) 0x00ffU) << 8) | \
           (((UINT16) (x) & (UINT16) 0xff00U) >> 8))) 
#endif

#ifndef SWAP32
#define SWAP32(x) \
    ((UINT32) (\
           (((UINT32) (x) & (UINT32) 0x000000ffUL) << 24) | \
           (((UINT32) (x) & (UINT32) 0x0000ff00UL) << 8) | \
           (((UINT32) (x) & (UINT32) 0x00ff0000UL) >> 8) | \
           (((UINT32) (x) & (UINT32) 0xff000000UL) >> 24))) 
#endif

#ifndef SWAP64
#define SWAP64(x) \
    ((UINT64)( \
    (UINT64)(((UINT64)(x) & (UINT64) 0x00000000000000ffULL) << 56) | \
    (UINT64)(((UINT64)(x) & (UINT64) 0x000000000000ff00ULL) << 40) | \
    (UINT64)(((UINT64)(x) & (UINT64) 0x0000000000ff0000ULL) << 24) | \
    (UINT64)(((UINT64)(x) & (UINT64) 0x00000000ff000000ULL) <<  8) | \
    (UINT64)(((UINT64)(x) & (UINT64) 0x000000ff00000000ULL) >>  8) | \
    (UINT64)(((UINT64)(x) & (UINT64) 0x0000ff0000000000ULL) >> 24) | \
    (UINT64)(((UINT64)(x) & (UINT64) 0x00ff000000000000ULL) >> 40) | \
    (UINT64)(((UINT64)(x) & (UINT64) 0xff00000000000000ULL) >> 56) ))
#endif

#ifdef RT_BIG_ENDIAN
#define cpu2le64(x) SWAP64((x))
#define le2cpu64(x) SWAP64((x))
#define cpu2le32(x) SWAP32((x))
#define le2cpu32(x) SWAP32((x))
#define cpu2le16(x) SWAP16((x))
#define le2cpu16(x) SWAP16((x))
#define cpu2be64(x) ((UINT64)(x))
#define be2cpu64(x) ((UINT64)(x))
#define cpu2be32(x) ((UINT32)(x))
#define be2cpu32(x) ((UINT32)(x))
#define cpu2be16(x) ((UINT16)(x))
#define be2cpu16(x) ((UINT16)(x))
#else /* Little_Endian */
#define cpu2le64(x) ((UINT64)(x))
#define le2cpu64(x) ((UINT64)(x))
#define cpu2le32(x) ((UINT32)(x))
#define le2cpu32(x) ((UINT32)(x))
#define cpu2le16(x) ((UINT16)(x))
#define le2cpu16(x) ((UINT16)(x))
#define cpu2be64(x) SWAP64((x))
#define be2cpu64(x) SWAP64((x))
#define cpu2be32(x) SWAP32((x))
#define be2cpu32(x) SWAP32((x))
#define cpu2be16(x) SWAP16((x))
#define be2cpu16(x) SWAP16((x))
#endif /* RT_BIG_ENDIAN */


#endif /* __RTMP_UTIL_H__ */


/*****************************************************************************
 * declare of global variables and functions                                 *
*****************************************************************************/
NTSTATUS    RTUSBMultiWrite(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            Offset,
    IN    PUCHAR            pData,
    IN    USHORT            length);

NTSTATUS    do_RTUSBReadMACRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            Offset,
    OUT    PUINT32            pValue);

NTSTATUS    do_RTUSBWriteMACRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            Offset,
    IN    UINT32            Value);

NTSTATUS RTUSBFirmwareWrite(
    IN PRTMP_ADAPTER pAd,
    IN PUCHAR        pFwImage,
    IN ULONG        FwLen);

BOOLEAN AsicSendCommandToMcu(
    IN PRTMP_ADAPTER pAd,
    IN UCHAR         Command,
    IN UCHAR         Token,
    IN UCHAR         Arg0,
    IN UCHAR         Arg1);


INT RtmpAsicSendCommandToMcu(
    IN PRTMP_ADAPTER    pAd,
    IN UCHAR            Command,
    IN UCHAR            Token,
    IN UCHAR            Arg0,
    IN UCHAR            Arg1,
    IN BOOLEAN            FlgIsNeedLocked);

NDIS_STATUS RtmpAsicLoadFirmware(
    IN PRTMP_ADAPTER pAd);


VOID AsicSetRxAnt(
    IN PRTMP_ADAPTER    pAd,
    IN UCHAR            Ant);

VOID AsicDisableSync(
    IN PRTMP_ADAPTER pAd); 

VOID AsicSwitchChannel(
                      IN PRTMP_ADAPTER pAd, 
    IN    UCHAR            Channel,
    IN    BOOLEAN            bScan);

VOID RTMPFilterCalibration(
    IN PRTMP_ADAPTER pAd);

VOID NICInitRT3070RFRegisters(
    IN RTMP_ADAPTER *pAd);

VOID RT30xxSetRxAnt(
    IN PRTMP_ADAPTER    pAd,
    IN UCHAR            Ant);

NTSTATUS    RTUSBReadBBPRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            Id,
    IN    PUCHAR            pValue);

NTSTATUS    RTUSBWriteBBPRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            Id,
    IN    UCHAR            Value);

NTSTATUS    RTUSBWriteRFRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UINT32            Value);

NDIS_STATUS RT30xxWriteRFRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            regID,
    IN    UCHAR            value);

NDIS_STATUS RT30xxReadRFRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            regID,
    IN    PUCHAR            pValue);

VOID NICInitRFRegisters(
    IN RTMP_ADAPTER *pAd);

NTSTATUS RTUSBReadEEPROM16(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            offset,
    OUT    PUSHORT            pData);

NTSTATUS RTUSBWriteEEPROM16(
    IN RTMP_ADAPTER *pAd, 
    IN USHORT offset, 
    IN USHORT value);

VOID RT30xx_Init(
    IN PRTMP_ADAPTER        pAd);

VOID RT30xxLoadRFNormalModeSetup(
    IN PRTMP_ADAPTER     pAd);


NTSTATUS    RTUSBVenderReset(
    IN    PRTMP_ADAPTER    pAd);

VOID RT28xxUsbAsicRadioOn(
    IN PRTMP_ADAPTER pAd);

VOID RT28XXDMAEnable(
    IN PRTMP_ADAPTER pAd);

VOID    RTUSBBulkReceive(
    IN    PVOID    pAd,
    UCHAR *pData, 
    INT Length);

extern USB_DEVICE_ID rtusb_dev_id[];
extern INT const rtusb_usb_id_len;

extern const REG_PAIR  RT3020_RFRegTable[];
extern const UCHAR NUM_RF_3020_REG_PARMS;

/*****************************************************************************
 * definitions related to os                                                 *
*****************************************************************************/
#define OS_SEM_EVENT_INIT(_pSema)            do{_pSema = rt_sem_create("s-rt", 1, RT_IPC_FLAG_PRIO);}while(0)
#define OS_SEM_EVENT_DESTORY(_pSema)        rt_sem_delete(_pSema)
#define OS_SEM_EVENT_WAIT(_pSema, _status)        do{_status = rt_sem_take(_pSema, RT_WAITING_FOREVER);}while(0)
#define OS_SEM_EVENT_UP(_pSema)                rt_sem_release(_pSema)

#define RTMP_SEM_EVENT_INIT(__pSema, __pSemaList)    OS_SEM_EVENT_INIT(__pSema)
#define RTMP_SEM_EVENT_DESTORY                    OS_SEM_EVENT_DESTORY
#define RTMP_SEM_EVENT_WAIT                        OS_SEM_EVENT_WAIT
#define RTMP_SEM_EVENT_UP                        OS_SEM_EVENT_UP


/*****************************************************************************
 * definitions related to noslwan                                            *
*****************************************************************************/

#define RT_DEBUG_OFF        WHED_DEBUG_OFF
#define RT_DEBUG_ERROR      WHED_DEBUG_ERROR
#define RT_DEBUG_WARN       WHED_DEBUG_WARN
#define RT_DEBUG_INFO       WHED_DEBUG_INFO
#define RT_DEBUG_TRACE      WHED_DEBUG_TRACE
#define RT_DEBUG_LOUD       WHED_DEBUG_LOUD

#define RTUSB_CONTROL_MSG(pUsb_Dev, uEndpointAddress, Request, RequestType, Value,Index, tmpBuf, TransferBufferLength, timeout, ret)    \
          do{    \
            ret = usb_control_msg(pUsb_Dev, 0, Request, RequestType, Value, Index, tmpBuf, TransferBufferLength, timeout);    \
        }while(0)



