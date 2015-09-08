/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cmm_asic.c
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



BOOLEAN AsicSendCommandToMcu(
    IN PRTMP_ADAPTER pAd,
    IN UCHAR         Command,
    IN UCHAR         Token,
    IN UCHAR         Arg0,
    IN UCHAR         Arg1)
{
    return RtmpAsicSendCommandToMcu(pAd, Command, Token, Arg0, Arg1, TRUE);
}


/*
    ========================================================================
    Description:
        For 1x1 chipset : 2070 / 3070 / 3090 / 3370 / 3390 / 5370 / 5390 
        Usage :    1. Set Default Antenna as initialize
                2. Antenna Diversity switching used
                3. iwpriv command switch Antenna

    Return:
    ========================================================================
 */
VOID AsicSetRxAnt(
    IN PRTMP_ADAPTER    pAd,
    IN UCHAR            Ant)
{
    if (pAd->chipOps.SetRxAnt)
        pAd->chipOps.SetRxAnt(pAd, Ant);

}



/*
    ==========================================================================
    Description:

    IRQL = PASSIVE_LEVEL
    IRQL = DISPATCH_LEVEL
    
    ==========================================================================
 */
VOID AsicDisableSync(
    IN PRTMP_ADAPTER pAd) 
{
    BCN_TIME_CFG_STRUC csr;
    
    DBGPRINT(RT_DEBUG_TRACE, "--->Disable TSF synchronization\n");

    /* 2003-12-20 disable TSF and TBTT while NIC in power-saving have side effect*/
    /*              that NIC will never wakes up because TSF stops and no more */
    /*              TBTT interrupts*/
    //pAd->TbttTickCount = 0;
    RTMP_IO_READ32(pAd, BCN_TIME_CFG, &csr.word);
    csr.field.bBeaconGen = 0;
    csr.field.bTBTTEnable = 0;
    csr.field.TsfSyncMode = 0;
    csr.field.bTsfTicking = 0;
    RTMP_IO_WRITE32(pAd, BCN_TIME_CFG, csr.word);

}


    
/*
    ==========================================================================
    Description:

    IRQL = PASSIVE_LEVEL
    IRQL = DISPATCH_LEVEL
    
    ==========================================================================
 */
VOID AsicSwitchChannel(
                      IN PRTMP_ADAPTER pAd, 
    IN    UCHAR            Channel,
    IN    BOOLEAN            bScan) 
{
    RTMP_CHIP_ASIC_SWITCH_CHANNEL(pAd, Channel, bScan);
}


