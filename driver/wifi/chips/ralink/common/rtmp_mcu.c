/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : rtmp_mcu.c
 @brief  : this is referenced from ralink's linux driver
 @author : wangyifeng
 @history:
           2014-9-28    wangyifeng    Created file
           ...
******************************************************************************/
#include "..\include\rt_include.h"
#include "..\include\firmware.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
/* RT2870 Firmware Spec only used 1 oct for version expression*/
#define FIRMWARE_MINOR_VERSION    7

/* New 8k byte firmware size for RT3071/RT3072*/
#define FIRMWAREIMAGE_MAX_LENGTH    0x2000
#define FIRMWAREIMAGE_LENGTH        (sizeof (FirmwareImage) / sizeof(UCHAR))
#define FIRMWARE_MAJOR_VERSION        0

#define FIRMWAREIMAGEV1_LENGTH        0x1000
#define FIRMWAREIMAGEV2_LENGTH        0x1000


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

INT RtmpAsicSendCommandToMcu(
    IN PRTMP_ADAPTER    pAd,
    IN UCHAR            Command,
    IN UCHAR            Token,
    IN UCHAR            Arg0,
    IN UCHAR            Arg1,
    IN BOOLEAN            FlgIsNeedLocked)
{
    HOST_CMD_CSR_STRUC    H2MCmd;
    H2M_MAILBOX_STRUC    H2MMailbox;
    ULONG                i = 0;

    do
    {
        RTMP_IO_READ32(pAd, H2M_MAILBOX_CSR, &H2MMailbox.word);
        if (H2MMailbox.field.Owner == 0)
            break;
        RTMPusecDelay(2);
    } while(i++ < 100);

    if (i >= 100)
    {
        DBGPRINT(RT_DEBUG_ERROR,"H2M_MAILBOX still hold by MCU. command fail\n");
        return FALSE;
    }

    H2MMailbox.field.Owner      = 1;       /* pass ownership to MCU*/
    H2MMailbox.field.CmdToken = Token;
    H2MMailbox.field.HighByte = Arg1;
    H2MMailbox.field.LowByte  = Arg0;
    RTMP_IO_WRITE32(pAd, H2M_MAILBOX_CSR, H2MMailbox.word);

    H2MCmd.word               = 0;
    H2MCmd.field.HostCommand  = Command;
    RTMP_IO_WRITE32(pAd, HOST_CMD_CSR, H2MCmd.word);

    if (Command != 0x80)
    {
    }

    if (Command == WAKE_MCU_CMD)
        pAd->LastMCUCmd = Command;

    return TRUE;
}

NDIS_STATUS isMCUnotReady(
    IN PRTMP_ADAPTER pAd)
{
    NDIS_STATUS        Status = NDIS_STATUS_SUCCESS;
    ULONG            Index;
    UINT32            MacReg;
    
    Index = 0;

    do {
        RTMP_IO_READ32(pAd, PBF_SYS_CTRL, &MacReg);

        if (MacReg & 0x80) /* check bit 7*/
            break;
        
        RTMPusecDelay(1000);
    } while (Index++ < 1000);

    if (Index >= 1000)
        Status = NDIS_STATUS_FAILURE;

    return Status;
}

/*
    ========================================================================
    
    Routine Description:
        Load 8051 firmware file into MAC ASIC

    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        NDIS_STATUS_SUCCESS         firmware image load ok
        NDIS_STATUS_FAILURE         image not found

    IRQL = PASSIVE_LEVEL
        
    ========================================================================
*/
NDIS_STATUS RtmpAsicLoadFirmware(
    IN PRTMP_ADAPTER pAd)
{
    NDIS_STATUS        Status = NDIS_STATUS_SUCCESS;
    PUCHAR            pFirmwareImage;
    ULONG            FileLength;
    /*ULONG            firm;*/
    UINT32            Version = (pAd->MACVersion >> 16);
    //UINT32            MacReg1 = 0;

    pFirmwareImage = (PUCHAR)FirmwareImage;
    FileLength = sizeof(FirmwareImage);

    /* New 8k byte firmware size for RT3071/RT3072*/
    /*DBGPRINT(RT_DEBUG_TRACE, ("Usb Chip\n"));*/
    if (FIRMWAREIMAGE_LENGTH == FIRMWAREIMAGE_MAX_LENGTH)
    /*The firmware image consists of two parts. One is the origianl and the other is the new.*/
    /*Use Second Part*/
    {
        if ((Version != 0x2860) && (Version != 0x2872) && (Version != 0x3070)) 
        {    /* Use Firmware V2.*/
            /*printk("KH:Use New Version,part2\n");*/
            pFirmwareImage = (PUCHAR)&FirmwareImage[FIRMWAREIMAGEV1_LENGTH];
            FileLength = FIRMWAREIMAGEV2_LENGTH;
        }
        else
        {
            /*printk("KH:Use New Version,part1\n");*/
            pFirmwareImage = (PUCHAR)FirmwareImage;
            FileLength = FIRMWAREIMAGEV1_LENGTH;
        }
    }
    else
    {
        DBGPRINT(RT_DEBUG_ERROR, "KH: bin file should be 8KB.\n");
        Status = NDIS_STATUS_FAILURE;
    }


    RTMP_WRITE_FIRMWARE(pAd, pFirmwareImage, FileLength);

    if (isMCUnotReady(pAd))
    {
        DBGPRINT(RT_DEBUG_ERROR, "NICLoadFirmware: MCU is not ready\n\n\n");
        Status = NDIS_STATUS_FAILURE;
    }
    else
    {
        RTUSBWriteMACRegister(pAd, H2M_BBP_AGENT, 0); /* initialize BBP R/W access agent. */
        RTUSBWriteMACRegister(pAd,H2M_MAILBOX_CSR,0);
        RTUSBWriteMACRegister(pAd, H2M_INT_SRC, 0);
        AsicSendCommandToMcu(pAd, 0x72, 0x00, 0x00, 0x00); /* reset rf by MCU supported by new firmware */
//        RTMPusecDelay(1000);
//        AsicSendCommandToMcu(pAd, 0x31, 0x00, 0x00, 0x00);/* Wakeup MCU */
    }

    DBGPRINT(RT_DEBUG_TRACE, "<=== %s (status=%d)\n", __FUNCTION__, Status);

    return Status;
}


