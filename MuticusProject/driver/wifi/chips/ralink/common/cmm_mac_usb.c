/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cmm_mac_usb.c
 @brief  : This file is port from ralink's linux driver
 @author : wangyifeng
 @history:
           2014-9-28    wangyifeng    Created file
           ...
******************************************************************************/
#include "..\include\rt_include.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/




/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

/*
========================================================================
Routine Description:
    Enable DMA.

Arguments:
    *pAd                the raxx interface data pointer

Return Value:
    None

Note:
========================================================================
*/
VOID RT28XXDMAEnable(
    IN PRTMP_ADAPTER pAd)
{
    WPDMA_GLO_CFG_STRUC    GloCfg;
    USB_DMA_CFG_STRUC    UsbCfg;
    int                    i = 0;
    
    
    RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0x4);
    do
    {
        RTMP_IO_READ32(pAd, WPDMA_GLO_CFG, &GloCfg.word);
        if ((GloCfg.field.TxDMABusy == 0)  && (GloCfg.field.RxDMABusy == 0))
            break;
        
        DBGPRINT(RT_DEBUG_TRACE, "==>  DMABusy\n");
        RTMPusecDelay(1000);
        i++;
    }while ( i <200);

    RTMPusecDelay(50);
    GloCfg.field.EnTXWriteBackDDONE = 1;
    GloCfg.field.EnableRxDMA = 1;
    GloCfg.field.EnableTxDMA = 1;
//    GloCfg.field.EnableRxDMA = 0;
//    GloCfg.field.EnableTxDMA = 0;
    DBGPRINT(RT_DEBUG_TRACE, "<== WRITE DMA offset 0x208 = 0x%x\n", GloCfg.word);    
    RTMP_IO_WRITE32(pAd, WPDMA_GLO_CFG, GloCfg.word);
    
    UsbCfg.word = 0;
    UsbCfg.field.phyclear = 0;
    /* usb version is 1.1,do not use bulk in aggregation */
    if (((wifi_usb_adapter_t *)pAd->pUsb_Dev)->bulk_in_ep_size == 512)
            UsbCfg.field.RxBulkAggEn = 1;
    /* for last packet, PBF might use more than limited, so minus 2 to prevent from error */
    UsbCfg.field.RxBulkAggLmt = 1;//(MAX_RXBULK_SIZE /1024)-3;
    UsbCfg.field.RxBulkAggTOut = 0x80; /* 2006-10-18 */
    UsbCfg.field.RxBulkEn = 1;
    UsbCfg.field.TxBulkEn = 1;

    RTUSBWriteMACRegister(pAd, USB_DMA_CFG, UsbCfg.word);

}




VOID RT28xxUsbAsicRadioOn(
    IN PRTMP_ADAPTER pAd)
{
    UINT32 MACValue = 0;
    UINT32 rx_filter_flag;
    WPDMA_GLO_CFG_STRUC    GloCfg;
    INT i=0;
    UCHAR    rfreg;
    RTMP_CHIP_OP *pChipOps = &pAd->chipOps;


    
    /* make some traffic to invoke EvtDeviceD0Entry callback function*/
    

    RTUSBReadMACRegister(pAd,0x1000,&MACValue);
    DBGPRINT(RT_DEBUG_TRACE,"A MAC query to invoke EvtDeviceD0Entry, MACValue = 0x%x\n",MACValue);

    /* 1. Send wake up command.*/
#if 0
    RetryRound = 0;

    do
    {
        brc = AsicSendCommandToMcu(pAd, 0x31, PowerWakeCID, 0x00, 0x02);   
        if (brc)
        {
            /* Wait command ok.*/
            brc = AsicCheckCommandOk(pAd, PowerWakeCID);
        }
        if(brc){
            break;      /* PowerWakeCID cmd successed*/
        }
        DBGPRINT(RT_DEBUG_WARN, ("PSM :WakeUp Cmd Failed, retry %d\n", RetryRound));

        /* try 10 times at most*/
        if ((RetryRound++) > 10)
            break;
        /* delay and try again*/
        RTMPusecDelay(200);
    } while (TRUE);
    if (RetryRound > 10)
        DBGPRINT(RT_DEBUG_WARN, ("PSM :ASIC 0x31 WakeUp Cmd may Fail %d*******\n", RetryRound));
#endif


    /* 2. Enable Tx DMA.*/

    RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0x4);
    do
    {
        RTMP_IO_READ32(pAd, WPDMA_GLO_CFG, &GloCfg.word);
        if ((GloCfg.field.TxDMABusy == 0)  && (GloCfg.field.RxDMABusy == 0))
            break;
        
        DBGPRINT(RT_DEBUG_TRACE, "==>  DMABusy\n");
        RTMPusecDelay(1000);
        i++;
    }while ( i <200);


    RTMPusecDelay(50);
    GloCfg.field.EnTXWriteBackDDONE = 1;
    GloCfg.field.EnableRxDMA = 1;
    GloCfg.field.EnableTxDMA = 1;
    DBGPRINT(RT_DEBUG_TRACE, "<== WRITE DMA offset 0x208 = 0x%x\n", GloCfg.word);    
    RTMP_IO_WRITE32(pAd, WPDMA_GLO_CFG, GloCfg.word);
    

    /* enable RX of MAC block*/



#ifdef XLINK_SUPPORT
        if (pAd->StaCfg.PSPXlink)
            rx_filter_flag = PSPXLINK;
        else
#endif /* XLINK_SUPPORT */    
            rx_filter_flag = STANORMAL;     /* Staion not drop control frame will fail WiFi Certification.*/
        RTMP_IO_WRITE32(pAd, RX_FILTR_CFG, rx_filter_flag);
        RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0xc);

    /* 3. Turn on RF*/
/*    RT28xxUsbAsicRFOn(pAd);*/
    if (pChipOps->AsicReverseRfFromSleepMode)
        pChipOps->AsicReverseRfFromSleepMode(pAd, FALSE);

#ifdef RTMP_RF_RW_SUPPORT
/*for 3xxx ? need to reset R07 for VO......*/
           RT30xxReadRFRegister(pAd, RF_R07, &rfreg);
           rfreg = rfreg | 0x1;
           RT30xxWriteRFRegister(pAd, RF_R07, rfreg);
#endif /* RTMP_RF_RW_SUPPORT */

    /* 4. Clear idle flag*/
//    RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_IDLE_RADIO_OFF);

#if 0
    {
        UCHAR BBPValue = 0;
        /* Let BBP register at 20MHz to do scan */
        RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R4, &BBPValue);
        BBPValue &= (~0x18);
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R4, BBPValue);
        DBGPRINT(RT_DEBUG_TRACE, ("SYNC - BBP R4 to 20MHz.l\n"));
    }
#endif
}


