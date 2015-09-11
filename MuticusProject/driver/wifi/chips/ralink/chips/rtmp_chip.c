/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : rtmp_chip.c
 @brief  : This file is port from ralink's linux driver
 @author : wangyifeng
 @history:
           2014-9-28    wangyifeng    Created file
           ...
******************************************************************************/
#include "..\include\rt_include.h"


const FREQUENCY_ITEM RtmpFreqItems3020[] =
{    
    /* ISM : 2.4 to 2.483 GHz                         */
    /* 11g*/
    /*-CH---N-------R---K-----------*/
    {1,    241,  2,  2},
    {2,    241,     2,  7},
    {3,    242,     2,  2},
    {4,    242,     2,  7},
    {5,    243,     2,  2},
    {6,    243,     2,  7},
    {7,    244,     2,  2},
    {8,    244,     2,  7},
    {9,    245,     2,  2},
    {10,   245,     2,  7},
    {11,   246,     2,  2},
    {12,   246,     2,  7},
    {13,   247,     2,  2},
    {14,   248,     2,  4},
};

const FREQUENCY_ITEM FreqItems3020_Xtal20M[] =
{    
    /*
     * RF_R08:
     * <7:0>: pll_N<7:0>
     *
     * RF_R09:
     * <3:0>: pll_K<3:0>
     * <4>: pll_N<8>
     * <7:5>pll_N<11:9>
     *
     */
    /*-CH---N--------R---N[7:4]K[3:0]------*/
    {1,    0xE2,     2,  0x14},
    {2,    0xE3,     2,  0x14},
    {3,    0xE4,     2,  0x14},
    {4,    0xE5,     2,  0x14},
    {5,    0xE6,     2,  0x14},
    {6,    0xE7,     2,  0x14},
    {7,    0xE8,     2,  0x14},
    {8,    0xE9,     2,  0x14},
    {9,    0xEA,     2,  0x14},
    {10,   0xEB,     2,  0x14},
    {11,   0xEC,     2,  0x14},
    {12,   0xED,     2,  0x14},
    {13,   0xEE,     2,  0x14},
    {14,   0xF0,     2,  0x18},
};

const UCHAR    NUM_OF_3020_CHNL = (sizeof(RtmpFreqItems3020) / sizeof(FREQUENCY_ITEM));

const FREQUENCY_ITEM* const FreqItems3020 = (FREQUENCY_ITEM *)RtmpFreqItems3020;


/*
========================================================================
Routine Description:
    Initialize normal beacon frame architecture.

Arguments:
    pAd                - WLAN control block pointer

Return Value:
    None

Note:
========================================================================
*/
VOID RtmpChipBcnInit(
    IN RTMP_ADAPTER *pAd)
{
    RTMP_CHIP_CAP *pChipCap = &pAd->chipCap;

    pChipCap->FlgIsSupSpecBcnBuf = FALSE;
    pChipCap->BcnMaxHwNum = HW_BEACON_MAX_NUM;
    pChipCap->BcnMaxNum = (pChipCap->BcnMaxHwNum - MAX_MESH_NUM - MAX_APCLI_NUM);
    pChipCap->BcnMaxHwSize = 0x1000;

    pChipCap->BcnBase[0] = 0x7800;
#if 0
    pChipCap->BcnBase[1] = 0x7A00;
    pChipCap->BcnBase[2] = 0x7C00;
    pChipCap->BcnBase[3] = 0x7E00;
    pChipCap->BcnBase[4] = 0x7200;
    pChipCap->BcnBase[5] = 0x7400;
    pChipCap->BcnBase[6] = 0x5DC0;
    pChipCap->BcnBase[7] = 0x5BC0;
#endif

    /* If the MAX_MBSSID_NUM is larger than 6, */
    /* it shall reserve some WCID space(wcid 222~253) for beacon frames. */
    /* -    these wcid 238~253 are reserved for beacon#6(ra6).*/
    /* -    these wcid 222~237 are reserved for beacon#7(ra7).*/
    if (pChipCap->BcnMaxNum == 8)
        pChipCap->WcidHwRsvNum = 222;
    else if (pChipCap->BcnMaxNum == 7)
        pChipCap->WcidHwRsvNum = 238;
    else
        pChipCap->WcidHwRsvNum = 255;

#if 0
    DBGPRINT(RT_DEBUG_TRACE, ("<<<<< Beacon Information: >>>>>\n"));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnMaxHwNum = \t%d\n", pChipCap->BcnMaxHwNum));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnMaxNum = \t%d\n", pChipCap->BcnMaxNum));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnMaxHwSize = \t0x%x\n", pChipCap->BcnMaxHwSize));
    DBGPRINT(RT_DEBUG_TRACE, ("\tWcidHwRsvNum = \t%d\n", pChipCap->WcidHwRsvNum));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[0] = \t0x%x\n", pChipCap->BcnBase[0]));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[1] = \t0x%x\n", pChipCap->BcnBase[1]));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[2] = \t0x%x\n", pChipCap->BcnBase[2]));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[3] = \t0x%x\n", pChipCap->BcnBase[3]));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[4] = \t0x%x\n", pChipCap->BcnBase[4]));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[5] = \t0x%x\n", pChipCap->BcnBase[5]));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[6] = \t0x%x\n", pChipCap->BcnBase[6]));
    DBGPRINT(RT_DEBUG_TRACE, ("\tBcnBase[7] = \t0x%x\n", pChipCap->BcnBase[7]));
#endif
}


VOID RtmpChipOpsHook(
    IN VOID            *pCB)
{
    RTMP_ADAPTER *pAd = (RTMP_ADAPTER *)pCB;
    RTMP_CHIP_OP *pChipOps = &pAd->chipOps;
    RTMP_CHIP_CAP *pChipCap = &pAd->chipCap;
    EEPROM_ANTENNA_STRUC Antenna;
    USHORT value;

    /* get RF IC type */
    RT28xx_EEPROM_READ16(pAd, EEPROM_NIC1_OFFSET, value);
    pAd->EEPROMDefaultValue[EEPROM_NIC_CFG1_OFFSET] = value;
    Antenna.word = pAd->EEPROMDefaultValue[EEPROM_NIC_CFG1_OFFSET];
    pAd->RfIcType = (UCHAR) Antenna.field.RfIcType;
    DBGPRINT(RT_DEBUG_TRACE, "RF IC Type: %d\n", pAd->RfIcType);

    /* save the antenna for future use */
    pAd->Antenna.word = Antenna.word;

    /* init default value whatever chipsets */
    /* default pChipOps content will be 0x00 */
    pChipCap->bbpRegTbSize = 0;
    pChipCap->MaxNumOfRfId = 31;
    pChipCap->MaxNumOfBbpId = 136;
    pChipCap->SnrFormula = SNR_FORMULA1;
    pChipCap->RfReg17WtMethod = RF_REG_WT_METHOD_NONE;

    pChipCap->FlgIsVcoReCalSup = FALSE;

    RtmpChipBcnInit(pAd);

    pChipOps->RxSensitivityTuning = 0;//RxSensitivityTuning;
    pChipOps->ChipResumeMsduTransmission = 0;//ChipResumeMsduTransmission;
#ifdef CONFIG_STA_SUPPORT
    pChipOps->ChipStaBBPAdjust = 0;//ChipStaBBPAdjust;
#endif /* CONFIG_STA_SUPPORT */
    pChipOps->ChipBBPAdjust = 0;//ChipBBPAdjust;
    pChipOps->ChipSwitchChannel = 0;//ChipSwitchChannel;

#ifdef RTMP_INTERNAL_TX_ALC
    pChipOps->InitDesiredTSSITable = InitDesiredTSSITableDefault;
    pChipOps->AsicTxAlcGetAutoAgcOffset = AsicTxAlcGetAutoAgcOffset;
#endif /* RTMP_INTERNAL_TX_ALC */

    pChipOps->RTMPSetAGCInitValue = 0;//AsicSetAGCInitValue;

    pChipOps->AsicAntennaDefaultReset = 0;//AsicAntennaDefaultReset;
    pChipOps->NetDevNickNameInit = 0;//NetDevNickNameInit;
    /* Init value. If pChipOps->AsicResetBbpAgent==NULL, "AsicResetBbpAgent" as default. If your chipset has specific routine, please re-hook it at self init function */
    pChipOps->AsicResetBbpAgent = NULL;

#ifdef RT28xx
    pChipOps->ChipSwitchChannel = RT28xx_ChipSwitchChannel;
#endif /* RT28xx */

    /* We depends on RfICType and MACVersion to assign the corresponding operation callbacks. */

#if defined(RT5370) || defined(RT5372) || defined(RT5390) || defined(RT5392)
    if (IS_RT5390(pAd))
    {
        RT5390_Init(pAd);
    }
#endif /* defined(RT5370) || defined(RT5372) || defined(RT5390) || defined(RT5392) */

#ifdef RT30xx
    if (IS_RT30xx(pAd))
    {
        if (IS_RT3390(pAd)){
        //    RT33xx_Init(pAd);
        }
        else
            RT30xx_Init(pAd);
    }
#endif /* RT30xx */

    DBGPRINT(RT_DEBUG_TRACE, "Chip specific bbpRegTbSize=%d!\n", pChipCap->bbpRegTbSize);
    
}


