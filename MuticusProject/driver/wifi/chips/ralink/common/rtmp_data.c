/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : rtmp_data.c
 @brief  : This file is port from ralink's linux driver
 @author : wangyifeng
 @history:
           2014-9-28    wangyifeng    Created file
           ...
******************************************************************************/
#include <string.h>
#include "..\include\rt_include.h"
#include "cv_wnet.h"

extern RTMP_ADAPTER rtmp_adapter;
extern int32_t wnet_dataframe_recv(uint8_t *databuf, uint32_t datalen);

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
static UCHAR BroadcastAddr[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static UCHAR BssidForV2V[] = {0x00, 0x63, 0x73, 0x76, 0x32, 0x76};
static UCHAR BeaconFixedElement[] = 
{
//    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* timestamp, reserved for llc */
    0x64, 0x00, /* beacon interval */
    0x22, 0x04, /* CapabilityInfo: IBSS, short preamble, short slot time */
    0x00, 0x00, /* ssid(empty) */
    0xDD, 0x00, /* vendor specific */
};
#define MAC_BODY_RESERVE_LENGTH (8+sizeof(BeaconFixedElement))
#define MAC_HEADER_LENGTH (24)


#ifndef NDEBUG
UCHAR *DbgRxFilterAddrTable[] = {
    /* 
     * Developer name could be defined in the project of Keil uvision
     */
    #if defined(WANGYIFENG)
    "\x00\x11\x22\x2f\x00\x1d",
    "\x00\x11\x22\x35\x00\x34",
    #elif defined(WANGLEI)
    "\x00\x11\x22\x24\x00\x1e",
    "\x00\x11\x22\x2d\x00\x1e",
    #elif defined(GEXUEYUAN)
    
    #endif
    NULL
};
#endif

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/


VOID STARxDoneInterruptHandle(
    IN PRTMP_ADAPTER pAd,
    IN PUCHAR        pData,
    IN ULONG         RxBufferLength)
{
    PRXWI_STRUC    pRxWI;
    PRXINFO_STRUC  pRxD;
    PHEADER_802_11 pHeader;
    PUCHAR pRxPacket;
    ULONG ThisFrameLen;

    /* The RXDMA field is 4 bytes, now just use the first 2 bytes. The Length including the (RXWI + MSDU + Padding)*/
    ThisFrameLen = *pData + (*(pData+1)<<8);
    if (ThisFrameLen == 0) {        
        DBGPRINT(RT_DEBUG_TRACE, "RXDMALen is zero.\n");     
        return;
    }   
    if ((ThisFrameLen&0x3) != 0) {
        DBGPRINT(RT_DEBUG_ERROR, "RXDMALen not multiple of 4.[%ld]\n", ThisFrameLen);
        return;
    }

    if ((ThisFrameLen + 8)> RxBufferLength) {
        /* 8 for (RT2870_RXDMALEN_FIELD_SIZE + sizeof(RXINFO_STRUC))*/
        DBGPRINT(RT_DEBUG_TRACE, "FrameLen(0x%lx) outranges,RxBufLen=0x%lx\n", 
                        ThisFrameLen, RxBufferLength);
        return;
    }

    /* skip USB frame length field*/
    pData += RT2870_RXDMALEN_FIELD_SIZE;
    pRxWI = (PRXWI_STRUC)pData;

    if (pRxWI->MPDUtotalByteCount > ThisFrameLen) {
        DBGPRINT(RT_DEBUG_TRACE, "pRxWIMPDUtotalByteCount(%d) large than RxDMALen(%ld)\n", 
                                  pRxWI->MPDUtotalByteCount, ThisFrameLen);
        return;
    }

    if (pRxWI->MPDUtotalByteCount < sizeof(HEADER_802_11)) {
        DBGPRINT(RT_DEBUG_TRACE, "FrameLen(0x%lx) is less than mac802.11 header\n");
        return;
    }

    pRxD = (PRXINFO_STRUC)(pData + ThisFrameLen);
    if (pRxD->Crc) {
        DBGPRINT(RT_DEBUG_TRACE, "Crc error\n");
        return;
    }

    /* Filter the received frame */
    pHeader = (PHEADER_802_11) (pData + RXWI_SIZE);
    if (pHeader->FC.Type != BTYPE_MGMT) {
        return;
    }
    if (pHeader->FC.SubType != SUBTYPE_BEACON){
        return;
    }
    //osal_printf("<"); /* Indicate RX is in process, for debug only */
    if (memcmp(pHeader->Addr3, BssidForV2V, MAC_ADDR_LEN) != 0) {
        return;
    }

    #ifndef NDEBUG
    {
        INT i, allow = FALSE;
        for (i=0;;i++) {
            if (DbgRxFilterAddrTable[i] == NULL) {
                break;
            }
            if (memcmp(pHeader->Addr2, DbgRxFilterAddrTable[i], MAC_ADDR_LEN) == 0) {
                allow = 1;
                break;
            }
        }
        if (!allow) {
            return;
        }
    }
    #endif

    pRxPacket = (PUCHAR)(pData + RXWI_SIZE + sizeof(HEADER_802_11));

    {/* Parse the the element of V2V */
        #define SKIP_LEN 12
        PUCHAR pPayload;
        UCHAR ElementID, ElementLen;
        INT PayloadLen;

        pPayload = pRxPacket + SKIP_LEN; /* skip fixed element in the beacon */
        PayloadLen = ThisFrameLen - RXWI_SIZE + sizeof(HEADER_802_11) - SKIP_LEN;
        
        while (PayloadLen > 0) 
        {
            ElementID = *pPayload++;
            ElementLen = *pPayload++;
            if (ElementID == 0xDD) 
           {
               wnet_rxinfo_t rxinfo;

               //osal_printf("."); /* Indicate RX is in process, for debug only */
               /* construct rxinfo */ 
               memset(&rxinfo, 0, sizeof(wnet_rxinfo_t));
               memcpy(&rxinfo.src.mac.addr, pHeader->Addr2, MAC_ADDR_LEN);
               rxinfo.rssi = pRxWI->RSSI0;

               /* move LLC header */ 
               pPayload -= 8;
               memcpy(pPayload, pRxPacket, 8);

               wnet_recv(&rxinfo, pPayload, (UINT32)ElementLen+8);
               break;
            }
            pPayload += ElementLen;
            PayloadLen -= (ElementLen + 2);
        }
    }
}

VOID    RTUSBBulkReceive(
    IN    PVOID    pAd,
    UCHAR *pData, 
    INT Length)
{
    PRTMP_ADAPTER  pAdap = (PRTMP_ADAPTER) pAd;

    
    if(pAdap->CommonCfg.Mode) 
    {
        ate_rx_frame((PRTMP_ADAPTER)pAd, pData, Length);
    }
    else 
    {
        STARxDoneInterruptHandle((PRTMP_ADAPTER)pAd, pData, Length);
        usb_bulkin(((PRTMP_ADAPTER)pAd)->pUsb_Dev);
    }
}

VOID    RTUSBBulkSendDone(
    IN    PVOID    pAd)
{
    PRTMP_ADAPTER  pAdap = (PRTMP_ADAPTER) pAd;

    if(pAdap->CommonCfg.Mode) {
        ate_tx_complete();
    }
    else {
        wnet_send_complete();
    }
}

/*
    Must be run in Interrupt context
    This function handle RT2870 specific TxDesc and cpu index update and kick the packet out.
 */
int RtmpUSBMgmtKickOut(
    IN RTMP_ADAPTER     *pAd, 
    IN UCHAR             QueIdx,
    IN PUCHAR            pSrcBufVA,
    IN UINT             SrcBufLen)
{
    PTXINFO_STRUC    pTxInfo;
    ULONG            BulkOutSize;
    UCHAR            padLen;
    INT                i;
    
    pTxInfo = (PTXINFO_STRUC)(pSrcBufVA - TXINFO_SIZE);
    memset(pTxInfo, 0, TXINFO_SIZE);
    pTxInfo->USBDMATxPktLen = SrcBufLen;
    pTxInfo->QSEL = FIFO_EDCA;
    pTxInfo->USBDMANextVLD = FALSE; /*NextValid;   Need to check with Jan about this.*/
    pTxInfo->USBDMATxburst = FALSE;
    pTxInfo->WIV = TRUE;

    /* Build our URB for USBD*/
    BulkOutSize = SrcBufLen + TXINFO_SIZE;
    padLen = 4 - (BulkOutSize&3);
    for (i=0;i<padLen;i++) {
        *(pSrcBufVA+SrcBufLen+i) = 0;
    }
    
    /* Now do hardware-depened kick out.*/
    usb_bulkout(pAd->pUsb_Dev, pTxInfo, BulkOutSize+padLen);
    //rt_kprintf(">");
    #if 0
    {
        int i;
        int len = BulkOutSize+padLen;
        UCHAR *d = (UCHAR *)pTxInfo;
        
        rt_kprintf("============dump data============\n");
        for(i=0;i<len;i++){
            rt_kprintf("0x%02x, ", *(d+i));
            if ((i%8)==7) rt_kprintf("\n");
        }
        rt_kprintf("\n===============end===============\n");
    }
    #endif
    //RTUSBKickBulkOut(pAd);
    
    return 0;
}


NDIS_STATUS MlmeHardTransmitMgmt(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR    QueIdx,
    IN    PUCHAR            pSrcBufVA,
    IN    UINT            SrcBufLen)
{
    PTXWI_STRUC     pTxWI;

    pTxWI = (PTXWI_STRUC)(pSrcBufVA - TXWI_SIZE);

    {
        memset(pTxWI, 0, TXWI_SIZE);

        pTxWI->FRAG= FALSE;

        pTxWI->CFACK = FALSE;
        pTxWI->TS= FALSE;
        pTxWI->AMPDU = FALSE;
        pTxWI->ACK = FALSE;
        pTxWI->txop= IFS_BACKOFF;
        
        pTxWI->NSEQ = FALSE;
            
        pTxWI->WirelessCliID = RESERVED_WCID;
        pTxWI->MPDUtotalByteCount = SrcBufLen;
        pTxWI->PacketId = PID_MGMT;

        switch (pAd->CommonCfg.TxRate) {
        case 6: /* 6M */
            pTxWI->BW = BW_20;
            pTxWI->MCS = MCS_RATE_6;
            pTxWI->PHYMODE = MODE_OFDM;
            break;
            
        case 2: /* 2M */
            pTxWI->BW = BW_20;
            pTxWI->MCS = MCS_SHORTP_RATE_2;
            pTxWI->PHYMODE = MODE_CCK;
            break;

        default: /* others used 1M */
            pTxWI->BW = BW_20;
            pTxWI->MCS = MCS_LONGP_RATE_1;
            pTxWI->PHYMODE = MODE_CCK;
            break;
        }

        pTxWI->PacketId = pTxWI->MCS;
    }

    RtmpUSBMgmtKickOut(pAd, QueIdx, (PUCHAR)pTxWI, SrcBufLen+TXWI_SIZE);

    return NDIS_STATUS_SUCCESS;
}



int drv_wifi_mac_header_len(void)
{
    return (TXINFO_SIZE + TXWI_SIZE 
                + sizeof(HEADER_802_11) 
                + sizeof(BeaconFixedElement) /* beacon's fixed element */);
}

int drv_wifi_send(wnet_txinfo_t *txinfo, uint8_t *pdata, int32_t length)
{

#if 0
    PRTMP_ADAPTER    pAd = &rtmp_adapter;
    PHEADER_802_11    pHeader_802_11;
    PUCHAR pPayload;


    if (!pAd->init_complete) 
    {
        return -1;
    }

    /* fill the vendor specific element */
    pPayload = pdata - MAC_BODY_RESERVE_LENGTH + WNET_LLC_HEADER_LEN;
    
    /* move the LLC header */
    memcpy(pPayload, pdata, WNET_LLC_HEADER_LEN);
    memcpy(pPayload+WNET_LLC_HEADER_LEN, BeaconFixedElement, sizeof(BeaconFixedElement));
    *(pPayload+MAC_BODY_RESERVE_LENGTH-1) = length - WNET_LLC_HEADER_LEN; /* Attention! length must be less than 256 */

    /* fill the 802.11 header */
    pHeader_802_11 = (PHEADER_802_11)(pPayload - MAC_HEADER_LENGTH);
    memset(pHeader_802_11, 0, MAC_HEADER_LENGTH);
    pHeader_802_11->FC.Type = BTYPE_MGMT;
    pHeader_802_11->FC.SubType = SUBTYPE_BEACON;
    pHeader_802_11->Sequence = pAd->Sequence++;
    if (pAd->Sequence >0xfff) 
    {
        pAd->Sequence = 0;
    }
    COPY_MAC_ADDR(pHeader_802_11->Addr1, BroadcastAddr);
    COPY_MAC_ADDR(pHeader_802_11->Addr2, pAd->CurrentAddress);
    COPY_MAC_ADDR(pHeader_802_11->Addr3, BssidForV2V);

    MlmeHardTransmitMgmt(pAd, 0, (PUCHAR)pHeader_802_11, \
                         length-WNET_LLC_HEADER_LEN+MAC_BODY_RESERVE_LENGTH+MAC_HEADER_LENGTH);

    return 0; 

#else

return wnet_usart_send(txinfo, pdata, length);


#endif
    
}

