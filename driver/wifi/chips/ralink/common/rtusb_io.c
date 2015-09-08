/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : rtusb_io.c
 @brief  : This file port from ralink's linux driver
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

#include "..\include\rt_include.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
#define MAX_VENDOR_REQ_RETRY_COUNT  (10)
#define CONTROL_TIMEOUT_JIFFIES     (20)



/*
    ========================================================================
     Routine Description:
        RTUSB_VendorRequest - Builds a ralink specific request, sends it off to USB endpoint zero and waits for completion

    Arguments:
        @pAd:
          @TransferFlags:
          @RequestType: USB message request type value
          @Request: USB message request value
          @Value: USB message value
          @Index: USB message index value
          @TransferBuffer: USB data to be sent
          @TransferBufferLength: Lengths in bytes of the data to be sent

    Context: ! in atomic context

    Return Value:
        NDIS_STATUS_SUCCESS
        NDIS_STATUS_FAILURE
    
    Note:
        This function sends a simple control message to endpoint zero
        and waits for the message to complete, or CONTROL_TIMEOUT_JIFFIES timeout.
        Because it is synchronous transfer, so don't use this function within an atomic context, 
        otherwise system will hang, do be careful.

        TransferBuffer may located in stack region which may not in DMA'able region in some embedded platforms, 
        so need to copy TransferBuffer to UsbVendorReqBuf allocated by kmalloc to do DMA transfer.
        Use UsbVendorReq_semaphore to protect this region which may be accessed by multi task.
        Normally, coherent issue is resloved by low-level HC driver, so do not flush this zone by RTUSB_VendorRequest.
    
    ========================================================================
*/
NTSTATUS    RTUSB_VendorRequest(
    IN    PRTMP_ADAPTER    pAd,
    IN    UINT32            TransferFlags,
    IN    UCHAR            RequestType,
    IN    UCHAR            Request,
    IN    USHORT            Value,
    IN    USHORT            Index,
    IN    PVOID            TransferBuffer,
    IN    UINT32            TransferBufferLength)
{
    int                RET = 0;

    if (0)//(RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST))
    {
        return NDIS_STATUS_FAILURE;
    }
    else
    {
        int RetryCount = 0; /* RTUSB_CONTROL_MSG retry counts*/
        //assert(TransferBufferLength <MAX_PARAM_BUFFER_SIZE);
        
        RTMP_SEM_EVENT_WAIT(pAd->UsbVendorReq_semaphore, RET);
        if (RET == -1)
        {
            DBGPRINT(RT_DEBUG_ERROR, "UsbVendorReq_semaphore get failed\n");
            return NDIS_STATUS_FAILURE;
        }

        do {
            RTUSB_CONTROL_MSG(pAd->pUsb_Dev, 0, Request, RequestType, Value, Index, TransferBuffer, TransferBufferLength, CONTROL_TIMEOUT_JIFFIES, RET);
            
            if (RET < 0) {
                RetryCount++;
                RTMPusecDelay(5000); /* wait for 5ms*/
            }
        } while((RET < 0) && (RetryCount < MAX_VENDOR_REQ_RETRY_COUNT));
        
          RTMP_SEM_EVENT_UP(pAd->UsbVendorReq_semaphore);

           if (RET < 0) {
            DBGPRINT(RT_DEBUG_ERROR, "RTUSB_VendorRequest failed(%d),TxFlags=0x%x, ReqType=%s, Req=0x%x, Idx=0x%x,pAd->Flags=0x%lx\n",
                        RET, TransferFlags, (RequestType == DEVICE_VENDOR_REQUEST_OUT ? "OUT" : "IN"), Request, Index, /*pAd->Flags*/0);
        }

    }

    if (RET < 0)
        return NDIS_STATUS_FAILURE;
    else
        return NDIS_STATUS_SUCCESS;
}

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/
NTSTATUS RTUSBSingleWrite(
    IN     RTMP_ADAPTER     *pAd,
    IN    USHORT            Offset,
    IN    USHORT            Value)
{
    NTSTATUS    Status;

    Status = RTUSB_VendorRequest(
        pAd,
        USBD_TRANSFER_DIRECTION_OUT,
        DEVICE_VENDOR_REQUEST_OUT,
        0x2,
        Value,
        Offset,
        NULL,
        0);
    
    return Status;
}

NTSTATUS    RTUSBMultiWrite_ex(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            Offset,
    IN    PUCHAR            pData,
    IN    USHORT            length)
{
    NTSTATUS    Status;


    USHORT          index = 0,Value;
    PUCHAR          pSrc = pData;
    USHORT          resude = 0;

    resude = length % 2;
    length  += resude;
    do
    {
            Value =(USHORT)( *pSrc  | (*(pSrc + 1) << 8));
        Status = RTUSBSingleWrite(pAd,Offset + index,Value);
            index +=2;
            length -= 2;
            pSrc = pSrc + 2;
        }while(length > 0);

    return Status;
}

static NTSTATUS    RTUSBFirmwareRun(
    IN    PRTMP_ADAPTER    pAd)
{
    NTSTATUS    Status;
    //printk("  IO:%d,%s\r\n", g_test_cnt++, __FUNCTION__);
    Status = RTUSB_VendorRequest(
        pAd,
        USBD_TRANSFER_DIRECTION_OUT,
        DEVICE_VENDOR_REQUEST_OUT,
        0x01,
        0x8,
        0,
        NULL,
        0);
    
    return Status;
}

NTSTATUS RTUSBFirmwareWrite(
    IN PRTMP_ADAPTER pAd,
    IN PUCHAR        pFwImage,
    IN ULONG        FwLen)
{
    UINT32        MacReg;
    NTSTATUS     Status;
    USHORT        writeLen;

    Status = RTUSBReadMACRegister(pAd, MAC_CSR0, &MacReg);

    /* write firmware */
    writeLen = FwLen;
    RTUSBMultiWrite_ex(pAd, FIRMWARE_IMAGE_BASE, pFwImage, writeLen);

    Status = RTUSBWriteMACRegister(pAd, 0x7014, 0xffffffff);
    Status = RTUSBWriteMACRegister(pAd, 0x701c, 0xffffffff);

    /* change 8051 from ROM to RAM */
    Status = RTUSBFirmwareRun(pAd);

    return Status;
}


NTSTATUS    RTUSBMultiWrite(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            Offset,
    IN    PUCHAR            pData,
    IN    USHORT            length)
{
    NTSTATUS    Status;


    USHORT          index = 0,Value;
    PUCHAR          pSrc = pData;
    USHORT          resude = 0;

    resude = length % 2;
    length  += resude;
    do
    {
            Value =(USHORT)( *pSrc  | (*(pSrc + 1) << 8));
        Status = RTUSBSingleWrite(pAd,Offset + index,Value);
            index +=2;
            length -= 2;
            pSrc = pSrc + 2;
        }while(length > 0);

    return Status;
}


NTSTATUS    RTUSBVenderReset(
    IN    PRTMP_ADAPTER    pAd)
{
    NTSTATUS    Status;

    Status = RTUSB_VendorRequest(
        pAd,
        USBD_TRANSFER_DIRECTION_OUT,
        DEVICE_VENDOR_REQUEST_OUT,
        0x01,
        0x1,
        0,
        NULL,
        0);

    DBGPRINT(RT_DEBUG_TRACE, "<--RTUSBVenderReset\n");
    return Status;
}



/*
    ========================================================================
    
    Routine Description: Read 32-bit MAC register

    Arguments:

    Return Value:

    IRQL = 
    
    Note:
    
    ========================================================================
*/
NTSTATUS    do_RTUSBReadMACRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            Offset,
    OUT    PUINT32            pValue)
{
    NTSTATUS    Status = 0;
    UINT32        localVal;

    Status = RTUSB_VendorRequest(
        pAd,
        (USBD_TRANSFER_DIRECTION_IN | USBD_SHORT_TRANSFER_OK),
        DEVICE_VENDOR_REQUEST_IN,
        0x7,
        0,
        Offset,
        &localVal,
        4);
    
    *pValue = le2cpu32(localVal);

    if (Status < 0)
        *pValue = 0xffffffff;
    
    return Status;
}


/*
    ========================================================================
    
    Routine Description: Write 32-bit MAC register

    Arguments:

    Return Value:

    IRQL = 
    
    Note:
    
    ========================================================================
*/
NTSTATUS    do_RTUSBWriteMACRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            Offset,
    IN    UINT32            Value)
{
    NTSTATUS    Status;
    UINT32        localVal;

    localVal = Value;

    Status = RTUSBSingleWrite(pAd, Offset, (USHORT)(localVal & 0xffff));
    Status = RTUSBSingleWrite(pAd, Offset + 2, (USHORT)((localVal & 0xffff0000) >> 16));

    return Status;
}




/*
    ========================================================================
    
    Routine Description: Read 8-bit BBP register

    Arguments:

    Return Value:

    IRQL = 
    
    Note:
    
    ========================================================================
*/
NTSTATUS    RTUSBReadBBPRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            Id,
    IN    PUCHAR            pValue)
{
    BBP_CSR_CFG_STRUC    BbpCsr;
    UINT            i = 0;
    NTSTATUS        status;
    int                RET = 0;

    RTMP_SEM_EVENT_WAIT(pAd->UsbVendorReq_semaphore2, RET);
    RET = RET;
    
    
    /* Verify the busy condition*/
    do
    {
        status = RTUSBReadMACRegister(pAd, BBP_CSR_CFG, &BbpCsr.word);
        if(status >= 0)
        {
        if (!(BbpCsr.field.Busy == BUSY))
            break;
        }
        DBGPRINT(RT_DEBUG_TRACE, "RTUSBReadBBPRegister(BBP_CSR_CFG_1):retry count=%d!\n", i);
        i++;
    }while (i < RETRY_LIMIT);
    
    if (i == RETRY_LIMIT)
    {
        
        /* Read failed then Return Default value.*/
        
        *pValue = pAd->BbpWriteLatch[Id];
    
        DBGPRINT(RT_DEBUG_ERROR, "Retry count exhausted or device removed!!!\n");
        RTMP_SEM_EVENT_UP(pAd->UsbVendorReq_semaphore2);
        
        return STATUS_UNSUCCESSFUL;
    }

    /* Prepare for write material*/
    BbpCsr.word                 = 0;
    BbpCsr.field.fRead            = 1;
    BbpCsr.field.Busy            = 1;
    BbpCsr.field.RegNum         = Id;
    RTUSBWriteMACRegister(pAd, BBP_CSR_CFG, BbpCsr.word);

    i = 0;    
    /* Verify the busy condition*/
    do
    {
        status = RTUSBReadMACRegister(pAd, BBP_CSR_CFG, &BbpCsr.word);
        if (status >= 0)
        {
        if (!(BbpCsr.field.Busy == BUSY))
        {
            *pValue = (UCHAR)BbpCsr.field.Value;
            break;
        }
        }
        DBGPRINT(RT_DEBUG_TRACE, "RTUSBReadBBPRegister(BBP_CSR_CFG_2):retry count=%d!\n", i);
        i++;
    }while (i < RETRY_LIMIT);
    
    if (i == RETRY_LIMIT)
    {
        
        /* Read failed then Return Default value.*/
        
        *pValue = pAd->BbpWriteLatch[Id];

        DBGPRINT(RT_DEBUG_ERROR, "Retry count exhausted or device removed!!!\n");
        RTMP_SEM_EVENT_UP(pAd->UsbVendorReq_semaphore2);
        return STATUS_UNSUCCESSFUL;
    }
    RTMP_SEM_EVENT_UP(pAd->UsbVendorReq_semaphore2);
    
    return STATUS_SUCCESS;
}




/*
    ========================================================================
    
    Routine Description: Write 8-bit BBP register

    Arguments:

    Return Value:
    
    IRQL = 
    
    Note:
    
    ========================================================================
*/
NTSTATUS    RTUSBWriteBBPRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            Id,
    IN    UCHAR            Value)
{
    BBP_CSR_CFG_STRUC    BbpCsr;
    UINT            i = 0;
    NTSTATUS        status;
    int                RET = 0;

    RTMP_SEM_EVENT_WAIT(pAd->UsbVendorReq_semaphore2, RET);
    RET = RET;
    
    /* Verify the busy condition*/
    do
    {
        status = RTUSBReadMACRegister(pAd, BBP_CSR_CFG, &BbpCsr.word);
        if (status >= 0)
        {
        if (!(BbpCsr.field.Busy == BUSY))
            break;
        }
        DBGPRINT(RT_DEBUG_TRACE, "RTUSBWriteBBPRegister(BBP_CSR_CFG):retry count=%d!\n", i);
        i++;
    }
    while (i < RETRY_LIMIT);
    
    if (i == RETRY_LIMIT)
    {
        DBGPRINT(RT_DEBUG_ERROR, "Retry count exhausted or device removed!!!\n");
        RTMP_SEM_EVENT_UP(pAd->UsbVendorReq_semaphore2);
        return STATUS_UNSUCCESSFUL;
    }

    /* Prepare for write material*/
    BbpCsr.word                 = 0;
    BbpCsr.field.fRead            = 0;
    BbpCsr.field.Value            = Value;
    BbpCsr.field.Busy            = 1;
    BbpCsr.field.RegNum         = Id;
    RTUSBWriteMACRegister(pAd, BBP_CSR_CFG, BbpCsr.word);
    
    pAd->BbpWriteLatch[Id] = Value;
    RTMP_SEM_EVENT_UP(pAd->UsbVendorReq_semaphore2);

    return STATUS_SUCCESS;
}

/*
    ========================================================================
    
    Routine Description: Write RF register through MAC

    Arguments:

    Return Value:
    
    IRQL = 
    
    Note:
    
    ========================================================================
*/
NTSTATUS    RTUSBWriteRFRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UINT32            Value)
{
    PHY_CSR4_STRUC    PhyCsr4;
    UINT            i = 0;
    NTSTATUS        status;

    memset(&PhyCsr4, 0, sizeof(PHY_CSR4_STRUC));
    do
    {
        status = RTUSBReadMACRegister(pAd, RF_CSR_CFG0, &PhyCsr4.word);
        if (status >= 0)
        {
        if (!(PhyCsr4.field.Busy))
            break;
        }
        DBGPRINT(RT_DEBUG_TRACE, "RTUSBWriteRFRegister(RF_CSR_CFG0):retry count=%d!\n", i);
        i++;
    }
    while (i < RETRY_LIMIT);

    if (i == RETRY_LIMIT)
    {
        DBGPRINT(RT_DEBUG_ERROR, "Retry count exhausted or device removed!!!\n");
        return STATUS_UNSUCCESSFUL;
    }

    RTUSBWriteMACRegister(pAd, RF_CSR_CFG0, Value);
    
    return STATUS_SUCCESS;
}

#if USE_SYSTEM_EEPROM_DATA
//00-0c-43 ralink
uint8_t g_mac_addr[6] = {0x00, 0x0c,0x43,0xff,0xff,0xff};
char system_eeprom_data[256] = {
/*    0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F  */
/*0*/0x70,0x30,0x01,0x00,0x00,0x0d,0x09,0x10,0x61,0x68,0xff,0xff,0xff,0xff,0xff,0xff,
/*1*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*2*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*3*/0xff,0xff,0xff,0xff,0x11,0x05,0x84,0x04,0xff,0xff,0x4c,0x01,0xff,0xff,0xff,0xff,//0x36h change from 0x80 to 0x84;0x37h change from 0x00 to 0x04,all changes are to enable external LNA
/*4*/0xff,0xff,0xff,0xff,0x0e,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,//0x44h change from 0x00 to 0x0E,External LNA gain for 2.4G Band is 14db
#ifdef NDEBUG
/*5*/0xff,0xff,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,//0x52h to 0x6Dh are 2.4GHz TX0 & TX1 power setting registers; 52h to 5Eh maps TX0 channel1 to TX0 channel 14;60h to 6Dh maps TX1 channel1 to TX1 channel14
#else
/*5*/0xff,0xff,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//For developement, set the min TX power 
#endif
/*6*/0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,
/*7*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*8*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*9*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*A*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*B*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*C*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
/*D*/0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xaa,0xaa,
/*E*/0xaa,0xaa,0x88,0x66,0xaa,0xaa,0x88,0x66,0xaa,0xaa,0x88,0x66,0xaa,0xaa,0x88,0x66,
/*F*/0xaa,0xaa,0x88,0x66,0xaa,0xaa,0x88,0x66,0xaa,0xaa,0x88,0x66,0xff,0xff,0xff,0xff};
#endif

NTSTATUS RTUSBReadEEPROM16(
    IN    PRTMP_ADAPTER    pAd,
    IN    USHORT            offset,
    OUT    PUSHORT            pData)
{
    NTSTATUS status;
    char tmp[2];
#if USE_SYSTEM_EEPROM_DATA
        if(offset < 256){

            if(offset >= 4 && offset <= 9)
            {
                tmp[0] = g_mac_addr[offset - 4];
                tmp[1] = g_mac_addr[offset - 4 + 1];
            }else{
                tmp[0] = system_eeprom_data[offset];
                tmp[1] = system_eeprom_data[offset + 1];
            }
            memcpy(pData, tmp, 2);
        }
        else{
            DBGPRINT(RT_DEBUG_ERROR, "Invalid eeprom addr:%d", offset);
            *pData = 0xff;
        }
        status = STATUS_SUCCESS;
#else
    USHORT  localData;
        status = RTUSBReadEEPROM(pAd, offset, (PUCHAR)(&localData), 2);
        if (status == STATUS_SUCCESS)
            *pData = le2cpu16(localData);
#endif    
    return status;

}

NTSTATUS RTUSBWriteEEPROM16(
    IN RTMP_ADAPTER *pAd, 
    IN USHORT offset, 
    IN USHORT value)
{
#if USE_SYSTEM_EEPROM_DATA
    return STATUS_SUCCESS;
#else
    USHORT tmpVal;
    tmpVal = cpu2le16(value);
    return RTUSBWriteEEPROM(pAd, offset, (PUCHAR)&(tmpVal), 2);
#endif
}

