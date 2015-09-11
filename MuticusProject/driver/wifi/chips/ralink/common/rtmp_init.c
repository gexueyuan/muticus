/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : rtmp_init.c
 @brief  : This file is port from ralink's linux driver
 @author : wangyifeng
 @history:
           2014-9-28    wangyifeng    Created file
           ...
******************************************************************************/
#include "..\include\rt_include.h"
#include "cv_wnet.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
RTMP_ADAPTER rtmp_adapter;


#define RT3090A_DEFAULT_INTERNAL_LNA_GAIN    0x0A
UCHAR    NUM_BIT8[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};


/* BBP register initialization set*/

const REG_PAIR   BBPRegTable[] = {
    {BBP_R65,        0x2C},        /* fix rssi issue*/
    {BBP_R66,        0x38},    /* Also set this default value to pAd->BbpTuning.R66CurrentValue at initial*/
    {BBP_R69,        0x12},
    {BBP_R70,        0xa},    /* BBP_R70 will change to 0x8 in ApStartUp and LinkUp for rt2860C, otherwise value is 0xa*/
    {BBP_R73,        0x10},
    {BBP_R81,        0x37},
    {BBP_R82,        0x62},
    {BBP_R83,        0x6A},
    {BBP_R84,        0x99},    /* 0x19 is for rt2860E and after. This is for extension channel overlapping IOT. 0x99 is for rt2860D and before*/
    {BBP_R86,        0x00},    /* middle range issue, Rory @2008-01-28     */
    {BBP_R91,        0x04},    /* middle range issue, Rory @2008-01-28*/
    {BBP_R92,        0x00},    /* middle range issue, Rory @2008-01-28*/
    {BBP_R103,        0x00},     /* near range high-power issue, requested from Gary @2008-0528*/
    {BBP_R105,        0x05},    /* 0x05 is for rt2860E to turn on FEQ control. It is safe for rt2860D and before, because Bit 7:2 are reserved in rt2860D and before.*/
#ifdef DOT11_N_SUPPORT
    {BBP_R106,        0x35},    /* Optimizing the Short GI sampling request from Gray @2009-0409*/
#endif /* DOT11_N_SUPPORT */
};
#define    NUM_BBP_REG_PARMS    (sizeof(BBPRegTable) / sizeof(REG_PAIR))



/* ASIC register initialization sets*/

#ifdef SPECIFIC_BCN_BUF_SUPPORT
const RTMP_REG_PAIR    BcnSpecMACRegTable[] =    {
    /*     
        That means all beacon's size are 512 bytes 
        and their starting address are "0x4000, 0x4200, 0x4400, 0x4600, ....." 
        in the second(higher) 8KB shared memory . 

        The formula is : 0x4000 + BCNx_OFFSET*64
            ex : the address of BSS0 = 0x4000 + 0x00 * 64 = 0x4000
                 the address of BSS1 = 0x4000 + 0x08 * 64 = 0x4200
    */
    {BCN_OFFSET0,            0x18100800}, 
    {BCN_OFFSET1,            0x38302820}, 
    {BCN_OFFSET2,            0x58504840}, 
    {BCN_OFFSET3,            0x78706860}, 
};
#endif /* SPECIFIC_BCN_BUF_SUPPORT */

const RTMP_REG_PAIR    MACRegTable[] =    {
#if defined(HW_BEACON_OFFSET) && (HW_BEACON_OFFSET == 0x200)
    {BCN_OFFSET0,            0xf8f0e8e0}, /* 0x3800(e0), 0x3A00(e8), 0x3C00(f0), 0x3E00(f8), 512B for each beacon */
    {BCN_OFFSET1,            0x6f77d0c8}, /* 0x3200(c8), 0x3400(d0), 0x1DC0(77), 0x1BC0(6f), 512B for each beacon */
#elif defined(HW_BEACON_OFFSET) && (HW_BEACON_OFFSET == 0x100)
    {BCN_OFFSET0,            0xece8e4e0}, /* 0x3800, 0x3A00, 0x3C00, 0x3E00, 512B for each beacon */
    {BCN_OFFSET1,            0xfcf8f4f0}, /* 0x3800, 0x3A00, 0x3C00, 0x3E00, 512B for each beacon */
#endif /* HW_BEACON_OFFSET */

    {LEGACY_BASIC_RATE,        0x0000013f}, /*  Basic rate set bitmap*/
    {HT_BASIC_RATE,        0x00008003}, /* Basic HT rate set , 20M, MCS=3, MM. Format is the same as in TXWI.*/
    {MAC_SYS_CTRL,        0x00}, /* 0x1004, , default Disable RX*/
    {RX_FILTR_CFG,        0x17f97}, /*0x1400  , RX filter control,  */
    {BKOFF_SLOT_CFG,    0x209}, /* default set short slot time, CC_DELAY_TIME should be 2     */
    /*{TX_SW_CFG0,        0x40a06},  Gary,2006-08-23 */
    {TX_SW_CFG0,        0x0},         /* Gary,2008-05-21 for CWC test */
    {TX_SW_CFG1,        0x80606}, /* Gary,2006-08-23 */
    {TX_LINK_CFG,        0x1020},        /* Gary,2006-08-23 */
    /*{TX_TIMEOUT_CFG,    0x00182090},     CCK has some problem. So increase timieout value. 2006-10-09 MArvek RT*/
    {TX_TIMEOUT_CFG,    0x000a2090},    /* CCK has some problem. So increase timieout value. 2006-10-09 MArvek RT , Modify for 2860E ,2007-08-01*/
    {MAX_LEN_CFG,        MAX_AGGREGATION_SIZE | 0x00001000},    /* 0x3018, MAX frame length. Max PSDU = 16kbytes.*/
    {LED_CFG,        0x7f031e46}, /* Gary, 2006-08-23*/

#ifdef INF_AMAZON_SE
    {PBF_MAX_PCNT,            0x1F3F6F6F},     /*iverson modify for usb issue, 2008/09/19*/
                                            /* 6F + 6F < total page count FE*/
                                            /* so that RX doesn't occupy TX's buffer space when WMM congestion.*/
#else
    {PBF_MAX_PCNT,            0x1F3FBF9F},     /*0x1F3f7f9f},        Jan, 2006/04/20*/
#endif /* INF_AMAZON_SE */
    /*{TX_RTY_CFG,            0x6bb80408},     Jan, 2006/11/16*/
/* WMM_ACM_SUPPORT*/
/*    {TX_RTY_CFG,            0x6bb80101},     sample*/
    {TX_RTY_CFG,            0x47d01f0f},    /* Jan, 2006/11/16, Set TxWI->ACK =0 in Probe Rsp Modify for 2860E ,2007-08-03*/
    
    {AUTO_RSP_CFG,            0x00000013},    /* Initial Auto_Responder, because QA will turn off Auto-Responder*/
    {CCK_PROT_CFG,            0x05740003 /*0x01740003*/},    /* Initial Auto_Responder, because QA will turn off Auto-Responder. And RTS threshold is enabled. */
    {OFDM_PROT_CFG,            0x05740003 /*0x01740003*/},    /* Initial Auto_Responder, because QA will turn off Auto-Responder. And RTS threshold is enabled. */
#ifdef RTMP_MAC_USB
    {PBF_CFG,                 0xf40006},         /* Only enable Queue 2*/
    {MM40_PROT_CFG,            0x3F44084},        /* Initial Auto_Responder, because QA will turn off Auto-Responder*/
    {WPDMA_GLO_CFG,            0x00000030},
#endif /* RTMP_MAC_USB */
    {GF20_PROT_CFG,            0x01744004},    /* set 19:18 --> Short NAV for MIMO PS*/
    {GF40_PROT_CFG,            0x03F44084},    
    {MM20_PROT_CFG,            0x01744004},    
    {TXOP_CTRL_CFG,            0x0000583f, /*0x0000243f*/ /*0x000024bf*/},    /*Extension channel backoff.*/
    {TX_RTS_CFG,            0x00092b20},    

    {EXP_ACK_TIME,            0x002400ca},    /* default value */
    {TXOP_HLDR_ET,             0x00000002},

    /* Jerry comments 2008/01/16: we use SIFS = 10us in CCK defaultly, but it seems that 10us
        is too small for INTEL 2200bg card, so in MBSS mode, the delta time between beacon0
        and beacon1 is SIFS (10us), so if INTEL 2200bg card connects to BSS0, the ping
        will always lost. So we change the SIFS of CCK from 10us to 16us. */
    {XIFS_TIME_CFG,            0x33a41010},
    {PWR_PIN_CFG,            0x00000003},    /* patch for 2880-E*/
};


#ifdef CONFIG_STA_SUPPORT
const RTMP_REG_PAIR    STAMACRegTable[] =    {
    {WMM_AIFSN_CFG,        0x00002273},
    {WMM_CWMIN_CFG,    0x00002344},
    {WMM_CWMAX_CFG,    0x000034aa},
};
#endif /* CONFIG_STA_SUPPORT */

#ifdef SPECIFIC_BCN_BUF_SUPPORT
#define    NUM_BCN_SPEC_MAC_REG_PARMS        (sizeof(BcnSpecMACRegTable) / sizeof(RTMP_REG_PAIR))
#endif /* SPECIFIC_BCN_BUF_SUPPORT */

#define    NUM_MAC_REG_PARMS        (sizeof(MACRegTable) / sizeof(RTMP_REG_PAIR))
#ifdef CONFIG_STA_SUPPORT
#define    NUM_STA_MAC_REG_PARMS    (sizeof(STAMACRegTable) / sizeof(RTMP_REG_PAIR))
#endif /* CONFIG_STA_SUPPORT */


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/



/*
    ========================================================================
    
    Routine Description:
        Read initial channel power parameters from EEPROM
        
    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        None

    IRQL = PASSIVE_LEVEL

    Note:
        
    ========================================================================
*/
VOID    RTMPReadChannelPwr(
    IN    PRTMP_ADAPTER    pAd)
{
    UINT32                    i;
    EEPROM_TX_PWR_STRUC        Power;
    EEPROM_TX_PWR_STRUC        Power2;
#ifdef RT33xx
#endif /* RT33xx */

    /* Read Tx power value for all channels*/
    /* Value from 1 - 0x7f. Default value is 24.*/
    /* Power value : 2.4G 0x00 (0) ~ 0x1F (31)*/
    /*             : 5.5G 0xF9 (-7) ~ 0x0F (15)*/

    /* 0. 11b/g, ch1 - ch 14*/
    for (i = 0; i < 7; i++)
    {
#ifdef RT30xx
#if defined(RT5370) || defined(RT5372) || defined(RT5390) || defined(RT5392)
        if (IS_RT5390(pAd))
        {
             RT28xx_EEPROM_READ16(pAd, EEPROM_G_TX_PWR_OFFSET + i * 2,Power.word);
            if (IS_RT5392(pAd))
            {
                RT28xx_EEPROM_READ16(pAd, EEPROM_G_TX2_PWR_OFFSET + i * 2,Power2.word);
            }
            pAd->TxPower[i * 2].Channel = i * 2 + 1;
            pAd->TxPower[i * 2 + 1].Channel = i * 2 + 2;
    
            if ((Power.field.Byte0 > 0x27) || (Power.field.Byte0 < 0))
            {
                pAd->TxPower[i * 2].Power = DEFAULT_RF_TX_POWER;
            }
            else
            {
                pAd->TxPower[i * 2].Power = Power.field.Byte0;
            }
    
            if ((Power.field.Byte1 > 0x27) || (Power.field.Byte1 < 0))
            {
                pAd->TxPower[i * 2 + 1].Power = DEFAULT_RF_TX_POWER;
            }
            else
            {
                pAd->TxPower[i * 2 + 1].Power = Power.field.Byte1;
            }
    
            if (IS_RT5392(pAd))
            {
                if ((Power2.field.Byte0 > 0x27) || (Power2.field.Byte0 < 0))
                {
                    pAd->TxPower[i * 2].Power2 = DEFAULT_RF_TX_POWER;
                }
                else
                {
                    pAd->TxPower[i * 2].Power2 = Power2.field.Byte0;
                }
        
                if ((Power2.field.Byte1 > 0x27) || (Power2.field.Byte1 < 0))
                {
                    pAd->TxPower[i * 2 + 1].Power2 = DEFAULT_RF_TX_POWER;
                }
                else
                {
                    pAd->TxPower[i * 2 + 1].Power2 = Power2.field.Byte1;
                }
            }
            
            DBGPRINT(RT_DEBUG_TRACE, ("%s: TxPower[%d].Power = 0x%02X, TxPower[%d].Power = 0x%02X\n", 
                __FUNCTION__, 
                i * 2, 
                pAd->TxPower[i * 2].Power, 
                i * 2 + 1, 
                pAd->TxPower[i * 2 + 1].Power));
            
            if (IS_RT5392(pAd))
            {
                DBGPRINT(RT_DEBUG_TRACE, ("%s: TxPower[%d].Power2 = 0x%02X, TxPower[%d].Power2 = 0x%02X\n", 
                    __FUNCTION__, 
                    i * 2, 
                    pAd->TxPower[i * 2].Power2, 
                    i * 2 + 1, 
                    pAd->TxPower[i * 2 + 1].Power2));
            }
        }
        else
#endif /* defined(RT5370) || defined(RT5372) || defined(RT5390) || defined(RT5392) */
#endif /* RT30xx */
        { /* RT3070 and RT3370 */
            RT28xx_EEPROM_READ16(pAd, EEPROM_G_TX_PWR_OFFSET + i * 2, Power.word);
            RT28xx_EEPROM_READ16(pAd, EEPROM_G_TX2_PWR_OFFSET + i * 2, Power2.word);
            pAd->TxPower[i * 2].Channel = i * 2 + 1;
            pAd->TxPower[i * 2 + 1].Channel = i * 2 + 2;

            pAd->TxPower[i * 2].Power = Power.field.Byte0;
            if(!IS_RT3390(pAd))  // 3370 has different Tx power range
            {
            if ((Power.field.Byte0 > 31) || (Power.field.Byte0 < 0))
                pAd->TxPower[i * 2].Power = DEFAULT_RF_TX_POWER;
            }                

            pAd->TxPower[i * 2 + 1].Power = Power.field.Byte1;
            if(!IS_RT3390(pAd)) // 3370 has different Tx power range
            {
            if ((Power.field.Byte1 > 31) || (Power.field.Byte1 < 0))
                pAd->TxPower[i * 2 + 1].Power = DEFAULT_RF_TX_POWER;
            }                

            if ((Power2.field.Byte0 > 31) || (Power2.field.Byte0 < 0))
                pAd->TxPower[i * 2].Power2 = DEFAULT_RF_TX_POWER;
            else
                pAd->TxPower[i * 2].Power2 = Power2.field.Byte0;

            if ((Power2.field.Byte1 > 31) || (Power2.field.Byte1 < 0))
                pAd->TxPower[i * 2 + 1].Power2 = DEFAULT_RF_TX_POWER;
            else
                pAd->TxPower[i * 2 + 1].Power2 = Power2.field.Byte1;
        }
    }
    #if 0
    {
        /* 1. U-NII lower/middle band: 36, 38, 40; 44, 46, 48; 52, 54, 56; 60, 62, 64 (including central frequency in BW 40MHz)*/
        /* 1.1 Fill up channel*/
        choffset = 14;
        for (i = 0; i < 4; i++)
        {
            pAd->TxPower[3 * i + choffset + 0].Channel    = 36 + i * 8 + 0;
            pAd->TxPower[3 * i + choffset + 0].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 0].Power2    = DEFAULT_RF_TX_POWER;

            pAd->TxPower[3 * i + choffset + 1].Channel    = 36 + i * 8 + 2;
            pAd->TxPower[3 * i + choffset + 1].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 1].Power2    = DEFAULT_RF_TX_POWER;

            pAd->TxPower[3 * i + choffset + 2].Channel    = 36 + i * 8 + 4;
            pAd->TxPower[3 * i + choffset + 2].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 2].Power2    = DEFAULT_RF_TX_POWER;
        }

        /* 1.2 Fill up power*/
        for (i = 0; i < 6; i++)
        {
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TX_PWR_OFFSET + i * 2, Power.word);
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TX2_PWR_OFFSET + i * 2, Power2.word);

            if ((Power.field.Byte0 < 16) && (Power.field.Byte0 >= -7))
                pAd->TxPower[i * 2 + choffset + 0].Power = Power.field.Byte0;

            if ((Power.field.Byte1 < 16) && (Power.field.Byte1 >= -7))
                pAd->TxPower[i * 2 + choffset + 1].Power = Power.field.Byte1;            

            if ((Power2.field.Byte0 < 16) && (Power2.field.Byte0 >= -7))
                pAd->TxPower[i * 2 + choffset + 0].Power2 = Power2.field.Byte0;

            if ((Power2.field.Byte1 < 16) && (Power2.field.Byte1 >= -7))
                pAd->TxPower[i * 2 + choffset + 1].Power2 = Power2.field.Byte1;            
        }
        
        /* 2. HipperLAN 2 100, 102 ,104; 108, 110, 112; 116, 118, 120; 124, 126, 128; 132, 134, 136; 140 (including central frequency in BW 40MHz)*/
        /* 2.1 Fill up channel*/
        choffset = 14 + 12;
        for (i = 0; i < 5; i++)
        {
            pAd->TxPower[3 * i + choffset + 0].Channel    = 100 + i * 8 + 0;
            pAd->TxPower[3 * i + choffset + 0].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 0].Power2    = DEFAULT_RF_TX_POWER;

            pAd->TxPower[3 * i + choffset + 1].Channel    = 100 + i * 8 + 2;
            pAd->TxPower[3 * i + choffset + 1].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 1].Power2    = DEFAULT_RF_TX_POWER;

            pAd->TxPower[3 * i + choffset + 2].Channel    = 100 + i * 8 + 4;
            pAd->TxPower[3 * i + choffset + 2].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 2].Power2    = DEFAULT_RF_TX_POWER;
        }
        pAd->TxPower[3 * 5 + choffset + 0].Channel        = 140;
        pAd->TxPower[3 * 5 + choffset + 0].Power        = DEFAULT_RF_TX_POWER;
        pAd->TxPower[3 * 5 + choffset + 0].Power2        = DEFAULT_RF_TX_POWER;

        /* 2.2 Fill up power*/
        for (i = 0; i < 8; i++)
        {
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TX_PWR_OFFSET + (choffset - 14) + i * 2, Power.word);
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TX2_PWR_OFFSET + (choffset - 14) + i * 2, Power2.word);

            if ((Power.field.Byte0 < 16) && (Power.field.Byte0 >= -7))
                pAd->TxPower[i * 2 + choffset + 0].Power = Power.field.Byte0;

            if ((Power.field.Byte1 < 16) && (Power.field.Byte1 >= -7))
                pAd->TxPower[i * 2 + choffset + 1].Power = Power.field.Byte1;            

            if ((Power2.field.Byte0 < 16) && (Power2.field.Byte0 >= -7))
                pAd->TxPower[i * 2 + choffset + 0].Power2 = Power2.field.Byte0;

            if ((Power2.field.Byte1 < 16) && (Power2.field.Byte1 >= -7))
                pAd->TxPower[i * 2 + choffset + 1].Power2 = Power2.field.Byte1;            
        }

        /* 3. U-NII upper band: 149, 151, 153; 157, 159, 161; 165, 167, 169; 171, 173 (including central frequency in BW 40MHz)*/
        /* 3.1 Fill up channel*/
        choffset = 14 + 12 + 16;
        /*for (i = 0; i < 2; i++)*/
        for (i = 0; i < 3; i++)
        {
            pAd->TxPower[3 * i + choffset + 0].Channel    = 149 + i * 8 + 0;
            pAd->TxPower[3 * i + choffset + 0].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 0].Power2    = DEFAULT_RF_TX_POWER;

            pAd->TxPower[3 * i + choffset + 1].Channel    = 149 + i * 8 + 2;
            pAd->TxPower[3 * i + choffset + 1].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 1].Power2    = DEFAULT_RF_TX_POWER;

            pAd->TxPower[3 * i + choffset + 2].Channel    = 149 + i * 8 + 4;
            pAd->TxPower[3 * i + choffset + 2].Power    = DEFAULT_RF_TX_POWER;
            pAd->TxPower[3 * i + choffset + 2].Power2    = DEFAULT_RF_TX_POWER;
        }
        pAd->TxPower[3 * 3 + choffset + 0].Channel        = 171;
        pAd->TxPower[3 * 3 + choffset + 0].Power        = DEFAULT_RF_TX_POWER;
        pAd->TxPower[3 * 3 + choffset + 0].Power2        = DEFAULT_RF_TX_POWER;

        pAd->TxPower[3 * 3 + choffset + 1].Channel        = 173;
        pAd->TxPower[3 * 3 + choffset + 1].Power        = DEFAULT_RF_TX_POWER;
        pAd->TxPower[3 * 3 + choffset + 1].Power2        = DEFAULT_RF_TX_POWER;

        /* 3.2 Fill up power*/
        /*for (i = 0; i < 4; i++)*/
        for (i = 0; i < 6; i++)
        {
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TX_PWR_OFFSET + (choffset - 14) + i * 2, Power.word);
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TX2_PWR_OFFSET + (choffset - 14) + i * 2, Power2.word);

            if ((Power.field.Byte0 < 16) && (Power.field.Byte0 >= -7))
                pAd->TxPower[i * 2 + choffset + 0].Power = Power.field.Byte0;

            if ((Power.field.Byte1 < 16) && (Power.field.Byte1 >= -7))
                pAd->TxPower[i * 2 + choffset + 1].Power = Power.field.Byte1;            

            if ((Power2.field.Byte0 < 16) && (Power2.field.Byte0 >= -7))
                pAd->TxPower[i * 2 + choffset + 0].Power2 = Power2.field.Byte0;

            if ((Power2.field.Byte1 < 16) && (Power2.field.Byte1 >= -7))
                pAd->TxPower[i * 2 + choffset + 1].Power2 = Power2.field.Byte1;            
        }
    }

    /* 4. Print and Debug*/
    /*choffset = 14 + 12 + 16 + 7;*/
    choffset = 14 + 12 + 16 + 11;
    #endif
}

/*
    ========================================================================
    
    Routine Description:
        Read initial Tx power per MCS and BW from EEPROM
        
    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        None

    IRQL = PASSIVE_LEVEL

    Note:
        
    ========================================================================
*/
VOID    RTMPReadTxPwrPerRate(
    IN    PRTMP_ADAPTER    pAd)
{
    ULONG        data, Adata, Gdata;
    USHORT        i, value, value2;
    USHORT        value_1, value_2, value_3, value_4;
    INT            Apwrdelta, Gpwrdelta;
    UCHAR        t1,t2,t3,t4;
    BOOLEAN        bApwrdeltaMinus = TRUE, bGpwrdeltaMinus = TRUE;
    
    {    
        
        /* Get power delta for 20MHz and 40MHz.*/
        
        DBGPRINT(RT_DEBUG_TRACE, "Txpower per Rate\n");
        RT28xx_EEPROM_READ16(pAd, EEPROM_TXPOWER_DELTA, value2);
        Apwrdelta = 0;
        Gpwrdelta = 0;

        if ((value2 & 0xff) != 0xff)
        {
            if ((value2 & 0x80))
                Gpwrdelta = (value2&0xf);
            
            if ((value2 & 0x40))
                bGpwrdeltaMinus = FALSE;
            else
                bGpwrdeltaMinus = TRUE;
        }
        if ((value2 & 0xff00) != 0xff00)
        {
            if ((value2 & 0x8000))
                Apwrdelta = ((value2&0xf00)>>8);

            if ((value2 & 0x4000))
                bApwrdeltaMinus = FALSE;
            else
                bApwrdeltaMinus = TRUE;
        }    
        DBGPRINT(RT_DEBUG_TRACE, "Gpwrdelta = %x, Apwrdelta = %x .\n", Gpwrdelta, Apwrdelta);

        
        /* Get Txpower per MCS for 20MHz in 2.4G.*/
        
        for (i=0; i<5; i++)
        {
            RT28xx_EEPROM_READ16(pAd, EEPROM_TXPOWER_BYRATE_20MHZ_2_4G + i*4, value);
            data = value;

            /* use value_1 ~ value_4 for code size reduce */
            value_1 = value&0xf;
            value_2 = (value&0xf0)>>4;
            value_3 = (value&0xf00)>>8;
            value_4 = (value&0xf000)>>12;

            if (bApwrdeltaMinus == FALSE)
            {
                t1 = value_1+(Apwrdelta);
                if (t1 > 0xf)
                    t1 = 0xf;
                t2 = value_2+(Apwrdelta);
                if (t2 > 0xf)
                    t2 = 0xf;
                t3 = value_3+(Apwrdelta);
                if (t3 > 0xf)
                    t3 = 0xf;
                t4 = value_4+(Apwrdelta);
                if (t4 > 0xf)
                    t4 = 0xf;
            }
            else
            {
                if (value_1 > Apwrdelta)
                    t1 = value_1-(Apwrdelta);
                else
                    t1 = 0;
                if (value_2 > Apwrdelta)
                    t2 = value_2-(Apwrdelta);
                else
                    t2 = 0;
                if (value_3 > Apwrdelta)
                    t3 = value_3-(Apwrdelta);
                else
                    t3 = 0;
                if (value_4 > Apwrdelta)
                    t4 = value_4-(Apwrdelta);
                else
                    t4 = 0;
            }                
            Adata = t1 + (t2<<4) + (t3<<8) + (t4<<12);
            if (bGpwrdeltaMinus == FALSE)
            {
                t1 = value_1+(Gpwrdelta);
                if (t1 > 0xf)
                    t1 = 0xf;
                t2 = value_2+(Gpwrdelta);
                if (t2 > 0xf)
                    t2 = 0xf;
                t3 = value_3+(Gpwrdelta);
                if (t3 > 0xf)
                    t3 = 0xf;
                t4 = value_4+(Gpwrdelta);
                if (t4 > 0xf)
                    t4 = 0xf;
            }
            else
            {
                if (value_1 > Gpwrdelta)
                    t1 = value_1-(Gpwrdelta);
                else
                    t1 = 0;
                if (value_2 > Gpwrdelta)
                    t2 = value_2-(Gpwrdelta);
                else
                    t2 = 0;
                if (value_3 > Gpwrdelta)
                    t3 = value_3-(Gpwrdelta);
                else
                    t3 = 0;
                if (value_4 > Gpwrdelta)
                    t4 = value_4-(Gpwrdelta);
                else
                    t4 = 0;
            }                
            Gdata = t1 + (t2<<4) + (t3<<8) + (t4<<12);
            
            RT28xx_EEPROM_READ16(pAd, EEPROM_TXPOWER_BYRATE_20MHZ_2_4G + i*4 + 2, value);

            /* use value_1 ~ value_4 for code size reduce */
            value_1 = value&0xf;
            value_2 = (value&0xf0)>>4;
            value_3 = (value&0xf00)>>8;
            value_4 = (value&0xf000)>>12;

            if (bApwrdeltaMinus == FALSE)
            {
                t1 = value_1+(Apwrdelta);
                if (t1 > 0xf)
                    t1 = 0xf;
                t2 = value_2+(Apwrdelta);
                if (t2 > 0xf)
                    t2 = 0xf;
                t3 = value_3+(Apwrdelta);
                if (t3 > 0xf)
                    t3 = 0xf;
                t4 = value_4+(Apwrdelta);
                if (t4 > 0xf)
                    t4 = 0xf;
            }
            else
            {
                if (value_1 > Apwrdelta)
                    t1 = value_1-(Apwrdelta);
                else
                    t1 = 0;
                if (value_2 > Apwrdelta)
                    t2 = value_2-(Apwrdelta);
                else
                    t2 = 0;
                if (value_3 > Apwrdelta)
                    t3 = value_3-(Apwrdelta);
                else
                    t3 = 0;
                if (value_4 > Apwrdelta)
                    t4 = value_4-(Apwrdelta);
                else
                    t4 = 0;
            }                
            Adata |= ((t1<<16) + (t2<<20) + (t3<<24) + (t4<<28));
            if (bGpwrdeltaMinus == FALSE)
            {
                t1 = value_1+(Gpwrdelta);
                if (t1 > 0xf)
                    t1 = 0xf;
                t2 = value_2+(Gpwrdelta);
                if (t2 > 0xf)
                    t2 = 0xf;
                t3 = value_3+(Gpwrdelta);
                if (t3 > 0xf)
                    t3 = 0xf;
                t4 = value_4+(Gpwrdelta);
                if (t4 > 0xf)
                    t4 = 0xf;
            }
            else
            {
                if (value_1 > Gpwrdelta)
                    t1 = value_1-(Gpwrdelta);
                else
                    t1 = 0;
                if (value_2 > Gpwrdelta)
                    t2 = value_2-(Gpwrdelta);
                else
                    t2 = 0;
                if (value_3 > Gpwrdelta)
                    t3 = value_3-(Gpwrdelta);
                else
                    t3 = 0;
                if (value_4 > Gpwrdelta)
                    t4 = value_4-(Gpwrdelta);
                else
                    t4 = 0;
            }                
            Gdata |= ((t1<<16) + (t2<<20) + (t3<<24) + (t4<<28));
            data |= (value<<16);

            /* For 20M/40M Power Delta issue */        
            pAd->Tx20MPwrCfgABand[i] = data;
            pAd->Tx20MPwrCfgGBand[i] = data;
            pAd->Tx40MPwrCfgABand[i] = Adata;
            pAd->Tx40MPwrCfgGBand[i] = Gdata;
            
            if (data != 0xffffffff)
                RTMP_IO_WRITE32(pAd, TX_PWR_CFG_0 + i*4, data);
            DBGPRINT(RT_DEBUG_TRACE, "20MHz BW, 2.4G band-%lx,  Adata = %lx,  Gdata = %lx \n", 
               data, Adata, Gdata);
        }
    }
}

/*
    ========================================================================
    
    Routine Description:
        Read initial parameters from EEPROM
        
    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        None

    IRQL = PASSIVE_LEVEL

    Note:
        
    ========================================================================
*/
VOID    NICReadEEPROMParameters(
    IN    PRTMP_ADAPTER    pAd,
    IN    PSTRING            mac_addr)
{
    UINT32            data = 0;
    USHORT            i, value;
    EEPROM_TX_PWR_STRUC        Power;
    EEPROM_VERSION_STRUC    Version;
    EEPROM_ANTENNA_STRUC    Antenna;
    EEPROM_NIC_CONFIG2_STRUC    NicConfig2;
    USHORT  Addr01,Addr23,Addr45 ;
    MAC_DW0_STRUC csr2;
    MAC_DW1_STRUC csr3;


    DBGPRINT(RT_DEBUG_TRACE, "--> NICReadEEPROMParameters\n");    

    if (pAd->chipOps.AsicEeBufferInit)
        pAd->chipOps.AsicEeBufferInit(pAd);

    if (pAd->chipOps.eeinit)
    {
        pAd->chipOps.eeinit(pAd);
#ifdef RTMP_EFUSE_SUPPORT
#ifdef RT30xx
#ifdef RALINK_ATE
        if(!pAd->bFroceEEPROMBuffer && pAd->bEEPROMFile)
        {
            DBGPRINT(RT_DEBUG_TRACE, "--> NICReadEEPROMParameters::(Efuse)Load to EEPROM Buffer Mode\n");    
            eFuseLoadEEPROM(pAd);
        }
#endif /* RALINK_ATE */
#endif /* RT30xx */
#endif /* RTMP_EFUSE_SUPPORT */
    }

    /* Init EEPROM Address Number, before access EEPROM; if 93c46, 
EEPROMAddressNum=6, else if 93c66, EEPROMAddressNum=8*/
    RTMP_IO_READ32(pAd, E2PROM_CSR, &data);
    DBGPRINT(RT_DEBUG_TRACE, "--> E2PROM_CSR = 0x%x\n", data);

    if((data & 0x30) == 0)
        pAd->EEPROMAddressNum = 6;        /* 93C46*/
    else if((data & 0x30) == 0x10)
        pAd->EEPROMAddressNum = 8;     /* 93C66*/
    else
        pAd->EEPROMAddressNum = 8;     /* 93C86*/
    DBGPRINT(RT_DEBUG_TRACE, "--> EEPROMAddressNum = %d\n", pAd->EEPROMAddressNum);

    if (mac_addr){
        COPY_MAC_ADDR(pAd->PermanentAddress, mac_addr);
    }
    else{
        /* Read MAC setting from EEPROM and record as permanent MAC address */
        DBGPRINT(RT_DEBUG_TRACE, "Initialize MAC Address from E2PROM \n");

        RT28xx_EEPROM_READ16(pAd, 0x04, Addr01);
        RT28xx_EEPROM_READ16(pAd, 0x06, Addr23);
        RT28xx_EEPROM_READ16(pAd, 0x08, Addr45);

        pAd->PermanentAddress[0] = (UCHAR)(Addr01 & 0xff);
        pAd->PermanentAddress[1] = (UCHAR)(Addr01 >> 8);
        pAd->PermanentAddress[2] = (UCHAR)(Addr23 & 0xff);
        pAd->PermanentAddress[3] = (UCHAR)(Addr23 >> 8);
        pAd->PermanentAddress[4] = (UCHAR)(Addr45 & 0xff);
        pAd->PermanentAddress[5] = (UCHAR)(Addr45 >> 8);

        /*more conveninet to test mbssid, so ap's bssid &0xf1*/
        if (pAd->PermanentAddress[0] == 0xff){
            pAd->PermanentAddress[0] = 0;
        }
    }

    /* Assign the actually working MAC Address */
    {
        COPY_MAC_ADDR(pAd->CurrentAddress, pAd->PermanentAddress);
        DBGPRINT(RT_DEBUG_TRACE, "Use the MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",\
                                    PRINT_MAC(pAd->CurrentAddress));
    }

    /* Set the current MAC to ASIC */    
    csr2.word = 0;
    csr2.field.Byte0 = pAd->CurrentAddress[0];
    csr2.field.Byte1 = pAd->CurrentAddress[1];
    csr2.field.Byte2 = pAd->CurrentAddress[2];
    csr2.field.Byte3 = pAd->CurrentAddress[3];
    RTMP_IO_WRITE32(pAd, MAC_ADDR_DW0, csr2.word);
    csr3.word = 0;
    csr3.field.Byte4 = pAd->CurrentAddress[4];
    csr3.field.Byte5 = pAd->CurrentAddress[5];
    csr3.field.U2MeMask = 0xff;
    RTMP_IO_WRITE32(pAd, MAC_ADDR_DW1, csr3.word);
    DBGPRINT(RT_DEBUG_INFO,"Current MAC: <%02x:%02x:%02x:%02x:%02x:%02x>\n",
                    PRINT_MAC(pAd->CurrentAddress));

    /* if not return early. cause fail at emulation.*/
    /* Init the channel number for TX channel power*/
    RTMPReadChannelPwr(pAd);

    /* if E2PROM version mismatch with driver's expectation, then skip*/
    /* all subsequent E2RPOM retieval and set a system error bit to notify GUI*/
    RT28xx_EEPROM_READ16(pAd, EEPROM_VERSION_OFFSET, Version.word);
    pAd->EepromVersion = Version.field.Version + Version.field.FaeReleaseNumber * 256;
    DBGPRINT(RT_DEBUG_TRACE, "E2PROM: Version = %d, FAE release #%d\n", \
     Version.field.Version, Version.field.FaeReleaseNumber);

    if (Version.field.Version > VALID_EEPROM_VERSION)
    {
        DBGPRINT(RT_DEBUG_ERROR,"E2PROM: WRONG VERSION 0x%x, should be %d\n",\
          Version.field.Version, VALID_EEPROM_VERSION);
    }

    /* Read BBP default value from EEPROM and store to array(EEPROMDefaultValue) in pAd*/
    RT28xx_EEPROM_READ16(pAd, EEPROM_NIC1_OFFSET, value);
    pAd->EEPROMDefaultValue[EEPROM_NIC_CFG1_OFFSET] = value;

    RT28xx_EEPROM_READ16(pAd, EEPROM_NIC2_OFFSET, value);
    pAd->EEPROMDefaultValue[EEPROM_NIC_CFG2_OFFSET] = value;

    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_COUNTRY_REGION, value);    /* Country Region*/
        pAd->EEPROMDefaultValue[EEPROM_COUNTRY_REG_OFFSET] = value;
    }


    for(i = 0; i < 8; i++)
    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_BBP_BASE_OFFSET + i*2, value);
        pAd->EEPROMDefaultValue[i+EEPROM_BBP_ARRAY_OFFSET] = value;
    }

    /* We have to parse NIC configuration 0 at here.*/
    /* If TSSI did not have preloaded value, it should reset the TxAutoAgc to 
false*/
    /* Therefore, we have to read TxAutoAgc control beforehand.*/
    /* Read Tx AGC control bit*/
    Antenna.word = pAd->EEPROMDefaultValue[EEPROM_NIC_CFG1_OFFSET];


    if (Antenna.word == 0xFFFF)
    {
        RTMP_CHIP_ANTENNA_INFO_DEFAULT_RESET(pAd, &Antenna);
    }

#if 0 //tempoly by wangyf
    /* Choose the desired Tx&Rx stream.*/
    if ((pAd->CommonCfg.TxStream == 0) || (pAd->CommonCfg.TxStream > Antenna.field.TxPath))
        pAd->CommonCfg.TxStream = Antenna.field.TxPath;

    if ((pAd->CommonCfg.RxStream == 0) || (pAd->CommonCfg.RxStream > Antenna.field.RxPath))
    {
        pAd->CommonCfg.RxStream = Antenna.field.RxPath;

        if ((pAd->MACVersion != RALINK_3883_VERSION) &&
            (pAd->MACVersion != RALINK_2883_VERSION) &&
            (pAd->CommonCfg.RxStream > 2))
        {
            /* only 2 Rx streams for RT2860 series*/
            pAd->CommonCfg.RxStream = 2;
        }
    }
#endif

    /* EEPROM offset 0x36 - NIC Configuration 1 */
    NicConfig2.word = pAd->EEPROMDefaultValue[EEPROM_NIC_CFG2_OFFSET];



#ifdef CONFIG_STA_SUPPORT
    //IF_DEV_CONFIG_OPMODE_ON_STA(pAd)
    {
        if ((NicConfig2.word & 0x00ff) == 0xff)
        {
            NicConfig2.word &= 0xff00;
        }

        if ((NicConfig2.word >> 8) == 0xff)
        {
            NicConfig2.word &= 0x00ff;
        }
    }
#endif /* CONFIG_STA_SUPPORT */

    if (NicConfig2.field.DynamicTxAgcControl == 1)
        pAd->bAutoTxAgcA = pAd->bAutoTxAgcG = TRUE;
    else
        pAd->bAutoTxAgcA = pAd->bAutoTxAgcG = FALSE;
    
    /* Save value for future using */
    pAd->NicConfig2.word = NicConfig2.word;
    
    DBGPRINT(RT_DEBUG_TRACE, "NICReadEEPROMParameters: RxPath = %d, TxPath = %d\n", \
     Antenna.field.RxPath, Antenna.field.TxPath);

    /* Save the antenna for future use*/
    pAd->Antenna.word = Antenna.word;

    /* Set the RfICType here, then we can initialize RFIC related operation callbacks*/
    pAd->RealRxPath = (UCHAR) Antenna.field.RxPath;

    pAd->RfIcType = (UCHAR) Antenna.field.RfIcType;

    /* check if the chip supports 5G band */
#if 0
    if (PHY_MODE_IS_5G_BAND(pAd->CommonCfg.PhyMode))
    {
        if (!RFIC_IS_5G_BAND(pAd))
        {
            DBGPRINT_RAW(RT_DEBUG_ERROR,
                        ("phy mode> Error! The chip does not support 5G band %d!\n",
                        pAd->RfIcType));
#ifdef DOT11_N_SUPPORT
            /* change to bgn mode */
            Set_WirelessMode_Proc(pAd, "9");
#else
            /* change to bg mode */
            Set_WirelessMode_Proc(pAd, "0");
#endif /* DOT11_N_SUPPORT */
        }
    }
#endif

    RTMP_NET_DEV_NICKNAME_INIT(pAd);

    RtmpChipOpsHook(pAd);

#ifdef TXRX_SW_ANTDIV_SUPPORT
    if( ((Antenna.word & 0xF000) != 0xF000) && (Antenna.word & 0x2000))  
     /* EEPROM 0x34[15:12] = 0xF is invalid, 0x2~0x3 is TX/RX SW AntDiv */
    {                                                                      
        pAd->chipCap.bTxRxSwAntDiv = TRUE;
        DBGPRINT(RT_DEBUG_OFF, ("\x1b[mAntenna word %X/%d, AntDiv %d\x1b[m\n", 
                    pAd->Antenna.word, pAd->Antenna.field.BoardType, pAd->NicConfig2.field.
AntDiversity));
    }
#endif /* TXRX_SW_ANTDIV_SUPPORT */
    
    /* Reset PhyMode if we don't support 802.11a*/
    /* Only RFIC_2850 & RFIC_2750 support 802.11a*/

#if 0    
    if ((Antenna.field.RfIcType != RFIC_2850)
        && (Antenna.field.RfIcType != RFIC_2750)
        && (Antenna.field.RfIcType != RFIC_3052)
        && (Antenna.field.RfIcType != RFIC_2853)
        )
    {
        if ((pAd->CommonCfg.PhyMode == PHY_11ABG_MIXED) || 
            (pAd->CommonCfg.PhyMode == PHY_11A))
            pAd->CommonCfg.PhyMode = PHY_11BG_MIXED;
#ifdef DOT11_N_SUPPORT
        else if ((pAd->CommonCfg.PhyMode == PHY_11ABGN_MIXED)    || 
                 (pAd->CommonCfg.PhyMode == PHY_11AN_MIXED)     || 
                 (pAd->CommonCfg.PhyMode == PHY_11AGN_MIXED)     ||
                 (pAd->CommonCfg.PhyMode == PHY_11N_5G))
            pAd->CommonCfg.PhyMode = PHY_11BGN_MIXED;
#endif /* DOT11_N_SUPPORT */

        pAd->RFICType = RFIC_24GHZ; /* CRDA*/
    }
    else
    {
        pAd->RFICType = RFIC_24GHZ | RFIC_5GHZ; /* CRDA*/
    }
#endif
    
    /* Read TSSI reference and TSSI boundary for temperature compensation. This is ugly*/
    /* 0. 11b/g*/
    {
        /* these are tempature reference value (0x00 ~ 0xFE)
           ex: 0x00 0x15 0x25 0x45 0x88 0xA0 0xB5 0xD0 0xF0
           TssiPlusBoundaryG [4] [3] [2] [1] [0] (smaller) +
           TssiMinusBoundaryG[0] [1] [2] [3] [4] (larger) */
        {
            RT28xx_EEPROM_READ16(pAd, EEPROM_G_TSSI_BOUND1, Power.word);
            pAd->TssiMinusBoundaryG[4] = Power.field.Byte0;
            pAd->TssiMinusBoundaryG[3] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_G_TSSI_BOUND2, Power.word);
            pAd->TssiMinusBoundaryG[2] = Power.field.Byte0;
            pAd->TssiMinusBoundaryG[1] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_G_TSSI_BOUND3, Power.word);
            pAd->TssiRefG   = Power.field.Byte0; /* reference value [0] */
            pAd->TssiPlusBoundaryG[1] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_G_TSSI_BOUND4, Power.word);
            pAd->TssiPlusBoundaryG[2] = Power.field.Byte0;
            pAd->TssiPlusBoundaryG[3] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_G_TSSI_BOUND5, Power.word);
            pAd->TssiPlusBoundaryG[4] = Power.field.Byte0;
            pAd->TxAgcStepG = Power.field.Byte1;    
            pAd->TxAgcCompensateG = 0;
            pAd->TssiMinusBoundaryG[0] = pAd->TssiRefG;
            pAd->TssiPlusBoundaryG[0]  = pAd->TssiRefG;

            /* Disable TxAgc if the based value is not right*/
            if (pAd->TssiRefG == 0xff)
                pAd->bAutoTxAgcG = FALSE;
        }

        DBGPRINT(RT_DEBUG_TRACE,\
          "E2PROM: G Tssi[-4 .. +4] = %d %d %d %d - %d -%d %d %d %d, step=%d, tuning=%d\n",
            pAd->TssiMinusBoundaryG[4], pAd->TssiMinusBoundaryG[3], 
            pAd->TssiMinusBoundaryG[2], pAd->TssiMinusBoundaryG[1],
            pAd->TssiRefG,
            pAd->TssiPlusBoundaryG[1], pAd->TssiPlusBoundaryG[2], 
            pAd->TssiPlusBoundaryG[3], pAd->TssiPlusBoundaryG[4],
            pAd->TxAgcStepG, pAd->bAutoTxAgcG);
    }    
    /* 1. 11a*/
    {
        {
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TSSI_BOUND1, Power.word);
            pAd->TssiMinusBoundaryA[4] = Power.field.Byte0;
            pAd->TssiMinusBoundaryA[3] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TSSI_BOUND2, Power.word);
            pAd->TssiMinusBoundaryA[2] = Power.field.Byte0;
            pAd->TssiMinusBoundaryA[1] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TSSI_BOUND3, Power.word);
            pAd->TssiRefA = Power.field.Byte0;
            pAd->TssiPlusBoundaryA[1] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TSSI_BOUND4, Power.word);
            pAd->TssiPlusBoundaryA[2] = Power.field.Byte0;
            pAd->TssiPlusBoundaryA[3] = Power.field.Byte1;
            RT28xx_EEPROM_READ16(pAd, EEPROM_A_TSSI_BOUND5, Power.word);
            pAd->TssiPlusBoundaryA[4] = Power.field.Byte0;
            pAd->TxAgcStepA = Power.field.Byte1;    
            pAd->TxAgcCompensateA = 0;
            pAd->TssiMinusBoundaryA[0] = pAd->TssiRefA;
            pAd->TssiPlusBoundaryA[0]  = pAd->TssiRefA;

            /* Disable TxAgc if the based value is not right*/
            if (pAd->TssiRefA == 0xff)
                pAd->bAutoTxAgcA = FALSE;
        }

        DBGPRINT(RT_DEBUG_TRACE,\
          "E2PROM: A Tssi[-4 .. +4] = %d %d %d %d - %d -%d %d %d %d, step=%d, tuning=%d\n",
            pAd->TssiMinusBoundaryA[4], pAd->TssiMinusBoundaryA[3], 
            pAd->TssiMinusBoundaryA[2], pAd->TssiMinusBoundaryA[1],
            pAd->TssiRefA,
            pAd->TssiPlusBoundaryA[1], pAd->TssiPlusBoundaryA[2], 
            pAd->TssiPlusBoundaryA[3], pAd->TssiPlusBoundaryA[4],
            pAd->TxAgcStepA, pAd->bAutoTxAgcA);
    }    
    //pAd->BbpRssiToDbmDelta = 0x0;
    
    /* Read frequency offset setting for RF*/
        RT28xx_EEPROM_READ16(pAd, EEPROM_FREQ_OFFSET, value);

    if ((value & 0x00FF) != 0x00FF)
        pAd->RfFreqOffset = (ULONG) (value & 0x00FF);
    else
        pAd->RfFreqOffset = 0;

    DBGPRINT(RT_DEBUG_TRACE, "E2PROM: RF FreqOffset=0x%lx \n", pAd->RfFreqOffset);

    /*CountryRegion byte offset (38h)*/
#if 0
    value = pAd->EEPROMDefaultValue[EEPROM_COUNTRY_REG_OFFSET] >> 8;        /* 2.4G band*/
    value2 = pAd->EEPROMDefaultValue[EEPROM_COUNTRY_REG_OFFSET] & 0x00FF;    /* 5G band*/
    
    if ((value <= REGION_MAXIMUM_BG_BAND) || (value == REGION_32_BG_BAND) 
     || (value == REGION_33_BG_BAND))
    {
        pAd->CommonCfg.CountryRegion = ((UCHAR) value) | 0x80;
    }

    if (value2 <= REGION_MAXIMUM_A_BAND)
    {
        pAd->CommonCfg.CountryRegionForABand = ((UCHAR) value2) | 0x80;
    }
#endif

    
    /* Get RSSI Offset on EEPROM 0x9Ah & 0x9Ch.*/
    /* The valid value are (-10 ~ 10) */
    /* */
    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_RSSI_BG_OFFSET, value);
        pAd->BGRssiOffset0 = value & 0x00ff;
        pAd->BGRssiOffset1 = (value >> 8);
    }

    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_RSSI_BG_OFFSET+2, value);
/*        if (IS_RT2860(pAd))  RT2860 supports 3 Rx and the 2.4 GHz RSSI #2 offset is in the EEPROM 0x48*/
            pAd->BGRssiOffset2 = value & 0x00ff;
        pAd->ALNAGain1 = (value >> 8);
    }

    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_LNA_OFFSET, value);
        pAd->BLNAGain = value & 0x00ff;
        pAd->ALNAGain0 = (value >> 8);
    }
    
    /* Validate 11b/g RSSI_0 offset.*/
    if ((pAd->BGRssiOffset0 < -10) || (pAd->BGRssiOffset0 > 10))
        pAd->BGRssiOffset0 = 0;

    /* Validate 11b/g RSSI_1 offset.*/
    if ((pAd->BGRssiOffset1 < -10) || (pAd->BGRssiOffset1 > 10))
        pAd->BGRssiOffset1 = 0;

    /* Validate 11b/g RSSI_2 offset.*/
    if ((pAd->BGRssiOffset2 < -10) || (pAd->BGRssiOffset2 > 10))
        pAd->BGRssiOffset2 = 0;

    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_RSSI_A_OFFSET, value);
        pAd->ARssiOffset0 = value & 0x00ff;
        pAd->ARssiOffset1 = (value >> 8);
    }

    {
        RT28xx_EEPROM_READ16(pAd, (EEPROM_RSSI_A_OFFSET+2), value);
        pAd->ARssiOffset2 = value & 0x00ff;
        pAd->ALNAGain2 = (value >> 8);
    }


    if (((UCHAR)pAd->ALNAGain1 == 0xFF) || (pAd->ALNAGain1 == 0x00))
        pAd->ALNAGain1 = pAd->ALNAGain0;
    if (((UCHAR)pAd->ALNAGain2 == 0xFF) || (pAd->ALNAGain2 == 0x00))
        pAd->ALNAGain2 = pAd->ALNAGain0;

    /* Validate 11a RSSI_0 offset.*/
    if ((pAd->ARssiOffset0 < -10) || (pAd->ARssiOffset0 > 10))
        pAd->ARssiOffset0 = 0;

    /* Validate 11a RSSI_1 offset.*/
    if ((pAd->ARssiOffset1 < -10) || (pAd->ARssiOffset1 > 10))
        pAd->ARssiOffset1 = 0;

    /*Validate 11a RSSI_2 offset.*/
    if ((pAd->ARssiOffset2 < -10) || (pAd->ARssiOffset2 > 10))
        pAd->ARssiOffset2 = 0;

#ifdef RT30xx
    
    /* Get TX mixer gain setting*/
    /* 0xff are invalid value*/
    /* Note: RT30xX default value is 0x00 and will program to RF_R17 only when this value is not zero.*/
    /*       RT359X default value is 0x02*/
    
    if (IS_RT30xx(pAd) || IS_RT3572(pAd)  || IS_RT3593(pAd)  || IS_RT5390(pAd))
    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_TXMIXER_GAIN_2_4G, value);
        pAd->TxMixerGain24G = 0;
        value &= 0x00ff;
        if (value != 0xff)
        {
            value &= 0x07;
            pAd->TxMixerGain24G = (UCHAR)value;
        }

    }
#endif /* RT30xx */
    
#ifdef LED_CONTROL_SUPPORT
    /* LED Setting */
    RTMPGetLEDSetting(pAd);
#endif /* LED_CONTROL_SUPPORT */
        
        RTMPReadTxPwrPerRate(pAd);

#ifdef SINGLE_SKU
    {
        RT28xx_EEPROM_READ16(pAd, EEPROM_DEFINE_MAX_TXPWR, pAd->CommonCfg.DefineMaxTxPwr);
    }

    if ((pAd->CommonCfg.DefineMaxTxPwr & 0xFF) <= 0x50 && pAd->CommonCfg.AntGain > 0 && pAd->CommonCfg.BandedgeDelta >= 0)
    {
        DBGPRINT(RT_DEBUG_TRACE, ("Single SKU Mode is enabled\n"));
        pAd->CommonCfg.bSKUMode = TRUE;
    }
    else
    {
        DBGPRINT(RT_DEBUG_TRACE, ("Single SKU Mode is disabled\n"));
        pAd->CommonCfg.bSKUMode = FALSE;
    }
#endif /* SINGLE_SKU */


#ifdef RT30xx
#ifdef RTMP_EFUSE_SUPPORT
    RtmpEfuseSupportCheck(pAd);
#endif /* RTMP_EFUSE_SUPPORT */
#endif /* RT30xx */

#ifdef RTMP_INTERNAL_TX_ALC
    RT28xx_EEPROM_READ16(pAd, EEPROM_NIC2_OFFSET, value);

    if(value==0xFFFF) { /*EEPROM is empty*/
        pAd->TxPowerCtrl.bInternalTxALC = FALSE;
    }else if(value & 1<<13) {
        pAd->TxPowerCtrl.bInternalTxALC = TRUE;
    }else {
        pAd->TxPowerCtrl.bInternalTxALC = FALSE;
    }

    DBGPRINT(RT_DEBUG_TRACE, ("TXALC> bInternalTxALC = %d\n",
            pAd->TxPowerCtrl.bInternalTxALC));
#endif /* RTMP_INTERNAL_TX_ALC */


    DBGPRINT(RT_DEBUG_TRACE, "%s: pAd->Antenna.field.BoardType = %d, IS_MINI_CARD(pAd) = %d, IS_RT5390U(pAd) = %d\n", 
        __FUNCTION__,
        pAd->Antenna.field.BoardType,
        IS_MINI_CARD(pAd),
        IS_RT5390U(pAd));
    DBGPRINT(RT_DEBUG_TRACE, "<-- NICReadEEPROMParameters\n");
}




VOID AntCfgInit(
IN  PRTMP_ADAPTER   pAd)
{

#ifdef ANT_DIVERSITY_SUPPORT
    if (pAd->CommonCfg.bRxAntDiversity == ANT_DIVERSITY_DEFAULT)
#endif
    {
        if (pAd->NicConfig2.field.AntOpt== 1) //ant selected by efuse
        {    
            if (pAd->NicConfig2.field.AntDiversity == 0) //main
            {
                pAd->RxAnt.Pair1PrimaryRxAnt = 0;
                pAd->RxAnt.Pair1SecondaryRxAnt = 1;
            }
            else//aux
            {
                pAd->RxAnt.Pair1PrimaryRxAnt = 1;
                pAd->RxAnt.Pair1SecondaryRxAnt = 0;
            }
        }
        else if (pAd->NicConfig2.field.AntDiversity == 0) //Ant div off: default ant is main
        {
            pAd->RxAnt.Pair1PrimaryRxAnt = 0;
            pAd->RxAnt.Pair1SecondaryRxAnt = 1;
        }
        else if (pAd->NicConfig2.field.AntDiversity == 1)//Ant div on
#ifdef ANT_DIVERSITY_SUPPORT
            if (pAd->chipCap.FlgIsHwAntennaDiversitySup)
                pAd->CommonCfg.bRxAntDiversity = ANT_HW_DIVERSITY_ENABLE; //filter by PPAD_Init() --> MAC Address
#else
        {//eeprom on, but ant div support is not enabled: default ant is man
            pAd->RxAnt.Pair1PrimaryRxAnt = 0;
            pAd->RxAnt.Pair1SecondaryRxAnt = 1;
        }
#endif
    }

    DBGPRINT(RT_DEBUG_OFF, "\x1b[m%s: primary/secondary ant %d/%d\n\x1b[m", 
                    __FUNCTION__,
                    pAd->RxAnt.Pair1PrimaryRxAnt,
                    pAd->RxAnt.Pair1SecondaryRxAnt);
#ifdef ANT_DIVERSITY_SUPPORT
    DBGPRINT(RT_DEBUG_OFF, ("\x1b[m%s: AntDiv %d\n\x1b[m", 
                    __FUNCTION__,
                    pAd->CommonCfg.bRxAntDiversity));
#endif
}


/*
    ========================================================================
    
    Routine Description:
        Set default value from EEPROM
        
    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        None

    IRQL = PASSIVE_LEVEL
    
    Note:
        
    ========================================================================
*/
VOID    NICInitAsicFromEEPROM(
    IN    PRTMP_ADAPTER    pAd)
{
#ifdef CONFIG_STA_SUPPORT
    UCHAR    BBPR1 = 0; 
#endif /* CONFIG_STA_SUPPORT */
    USHORT                    i;
#ifdef RALINK_ATE
    USHORT    value;
#endif /* RALINK_ATE */
    EEPROM_NIC_CONFIG2_STRUC    NicConfig2;
    UCHAR    BBPR3 = 0;
#ifdef RT30xx
    UCHAR            bbpreg = 0;
#endif /* RT30xx */
    
    DBGPRINT(RT_DEBUG_TRACE, "--> NICInitAsicFromEEPROM\n");
    for(i = EEPROM_BBP_ARRAY_OFFSET; i < NUM_EEPROM_BBP_PARMS; i++)
    {
        UCHAR BbpRegIdx, BbpValue;
    
        if ((pAd->EEPROMDefaultValue[i] != 0xFFFF) && (pAd->EEPROMDefaultValue[i] != 0))
        {
            BbpRegIdx = (UCHAR)(pAd->EEPROMDefaultValue[i] >> 8);
            BbpValue  = (UCHAR)(pAd->EEPROMDefaultValue[i] & 0xff);
            RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BbpRegIdx, BbpValue);
        }
    }

    NicConfig2.word = pAd->NicConfig2.word;


#ifdef LED_CONTROL_SUPPORT
    /* Send LED Setting to MCU */
    RTMPInitLEDMode(pAd);    
#endif /* LED_CONTROL_SUPPORT */

    /* finally set primary ant */
    AntCfgInit(pAd);

#ifdef RTMP_RF_RW_SUPPORT
    /*Init RT30xx RFRegisters after read RFIC type from EEPROM*/
    NICInitRFRegisters(pAd);
#endif /* RTMP_RF_RW_SUPPORT */

#ifdef ANT_DIVERSITY_SUPPORT
    if ((pAd->CommonCfg.bRxAntDiversity == ANT_HW_DIVERSITY_ENABLE) &&
            (pAd->chipOps.HwAntEnable))
        pAd->chipOps.HwAntEnable(pAd);
#endif


#if 0 //def CONFIG_STA_SUPPORT
    IF_DEV_CONFIG_OPMODE_ON_STA(pAd)
    {
        /* Read Hardware controlled Radio state enable bit*/
        if (NicConfig2.field.HardwareRadioControl == 1)
        {
            pAd->StaCfg.bHardwareRadio = TRUE;

            /* Read GPIO pin2 as Hardware controlled radio state*/
            RTMP_IO_READ32(pAd, GPIO_CTRL_CFG, &data);
            if ((data & 0x04) == 0)
            {
                pAd->StaCfg.bHwRadio = FALSE;
                pAd->StaCfg.bRadio = FALSE;
/*                RTMP_IO_WRITE32(pAd, PWR_PIN_CFG, 0x00001818);*/
                RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_RADIO_OFF);
            }
        }
        else
            pAd->StaCfg.bHardwareRadio = FALSE;        

#ifdef LED_CONTROL_SUPPORT
        if (pAd->StaCfg.bRadio == FALSE)
        {
            RTMPSetLED(pAd, LED_RADIO_OFF);
        }
        else
        {
            RTMPSetLED(pAd, LED_RADIO_ON);
        }
#endif /* LED_CONTROL_SUPPORT */

    }
#ifdef PCIE_PS_SUPPORT
#endif /* PCIE_PS_SUPPORT */
#endif /* CONFIG_STA_SUPPORT */
#ifdef RTMP_MAC_USB
        if (IS_RT30xx(pAd)|| IS_RT3572(pAd))
        {
            RTMP_CHIP_OP *pChipOps = &pAd->chipOps;
            if (pChipOps->AsicReverseRfFromSleepMode)
                pChipOps->AsicReverseRfFromSleepMode(pAd, TRUE);
        }
#endif /* RTMP_MAC_USB */
    /* Turn off patching for cardbus controller*/
    if (NicConfig2.field.CardbusAcceleration == 1)
    {
/*        pAd->bTest1 = TRUE;*/
    }

    if (NicConfig2.field.DynamicTxAgcControl == 1)
        pAd->bAutoTxAgcA = pAd->bAutoTxAgcG = TRUE;
    else
        pAd->bAutoTxAgcA = pAd->bAutoTxAgcG = FALSE;

#ifdef RTMP_INTERNAL_TX_ALC
    /*
        Internal Tx ALC support is starting from RT3370 / RT3390, which combine PA / LNA in single chip.
        The old chipset don't have this, add new feature flag RTMP_INTERNAL_ALC.
     */

    /* Internal Tx ALC */
#ifdef RT3350
    if (IS_RT3350(pAd) &&
        (((NicConfig2.field.DynamicTxAgcControl == 1) && 
        (NicConfig2.field.bInternalTxALC == 1))))
    {
        pAd->TxPowerCtrl.bInternalTxALC = FALSE;
    }
    else
#endif // RT3350 //
    if (((NicConfig2.field.DynamicTxAgcControl == 1) && 
            (NicConfig2.field.bInternalTxALC == 1)) || (!IS_RT3390(pAd) && !IS_RT5390(pAd)))
    {
        /*
            If both DynamicTxAgcControl and bInternalTxALC are enabled,
            it is a wrong configuration.
            If the chipset does not support internal ALC, we shall disable it.
        */
        pAd->TxPowerCtrl.bInternalTxALC = FALSE;
    }
    else
    {
        if (NicConfig2.field.bInternalTxALC == 1)
        {
            pAd->TxPowerCtrl.bInternalTxALC = TRUE;
        }
        else
        {
            pAd->TxPowerCtrl.bInternalTxALC = FALSE;
        }
    }

    
    /* Old 5390 NIC always disables the internal ALC */
    
    if (pAd->MACVersion == 0x53900501)
    {
        pAd->TxPowerCtrl.bInternalTxALC = FALSE;
    }

    DBGPRINT(RT_DEBUG_TRACE, ("%s: pAd->TxPowerCtrl.bInternalTxALC = %d\n", 
        __FUNCTION__, 
        pAd->TxPowerCtrl.bInternalTxALC));
#endif /* RTMP_INTERNAL_TX_ALC */

#ifdef RALINK_ATE
    RT28xx_EEPROM_READ16(pAd, EEPROM_TSSI_GAIN_AND_ATTENUATION, value);
    value = (value & 0x00FF);    
    
    if (IS_RT5390(pAd))
    {
        pAd->TssiGain = 0x02;     /* RT5390 uses 2 as TSSI gain/attenuation default value */
    }
    else
    {
        pAd->TssiGain = 0x03; /* RT5392 uses 3 as TSSI gain/attenuation default value */
    }    
    
    if ((value != 0x00) && (value != 0xFF))
    {
        pAd->TssiGain =  (UCHAR) (value & 0x000F);
    }
    
    DBGPRINT(RT_DEBUG_TRACE, ("%s: EEPROM_TSSI_GAIN_AND_ATTENUATION = 0x%X, pAd->TssiGain=0x%x\n", 
                __FUNCTION__, 
                value, 
                pAd->TssiGain));
#endif // RALINK_ATE //
    /* Since BBP has been progamed, to make sure BBP setting will be */
    /* upate inside of AsicAntennaSelect, so reset to UNKNOWN_BAND!!*/
    
    //pAd->CommonCfg.BandState = UNKNOWN_BAND;
    
    RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R3, &BBPR3);
    BBPR3 &= (~0x18);
    if(pAd->Antenna.field.RxPath == 3)
    {
        BBPR3 |= (0x10);
    }
    else if(pAd->Antenna.field.RxPath == 2)
    {
        BBPR3 |= (0x8);
    }
    else if(pAd->Antenna.field.RxPath == 1)
    {
        BBPR3 |= (0x0);
    }
    RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R3, BBPR3);
    
#ifdef CONFIG_STA_SUPPORT
    //IF_DEV_CONFIG_OPMODE_ON_STA(pAd)
    {
        /* Handle the difference when 1T*/
        RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R1, &BBPR1);

        {
            if(pAd->Antenna.field.TxPath == 1)
            {
            BBPR1 &= (~0x18);
            }
        }

        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R1, BBPR1);
    
        //DBGPRINT(RT_DEBUG_TRACE, ("Use Hw Radio Control Pin=%d; if used Pin=%d;\n", 
        //            pAd->StaCfg.bHardwareRadio, pAd->StaCfg.bHardwareRadio));
    }
#endif /* CONFIG_STA_SUPPORT */

    RTMP_EEPROM_ASIC_INIT(pAd);

#ifdef RT30xx
    /* Initialize RT3070 serial MAC registers which is different from RT2870 serial*/
    if (IS_RT3090(pAd) || IS_RT3390(pAd) || IS_RT3593(pAd) || IS_RT5390(pAd))
    {
        /* enable DC filter*/
        if ((pAd->MACVersion & 0xffff) >= 0x0211)
        {
            RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R103, 0xc0);
        }

        /* improve power consumption in RT3071 Ver.E */
        if (((pAd->MACVersion & 0xffff) >= 0x0211) && !IS_RT3593(pAd))
        {
            RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R31, &bbpreg);
            bbpreg &= (~0x3);
            RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R31, bbpreg);
        }


        RTMP_IO_WRITE32(pAd, TX_SW_CFG1, 0);
        
        /* RT3071 version E has fixed this issue*/
        if ((pAd->MACVersion & 0xffff) < 0x0211)
        {
            if (pAd->NicConfig2.field.DACTestBit == 1)
            {
                RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0x2C);    /* To fix throughput drop drastically*/
            }
            else
            {
                RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0x0F);    /* To fix throughput drop drastically*/
            }
        }
        else
        {
            RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0x0);
        }
    }
    else if (IS_RT3070(pAd))
    {
        if ((pAd->MACVersion & 0xffff) >= 0x0201)
        {
            /* enable DC filter*/
            RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R103, 0xc0);
            
            /* improve power consumption in RT3070 Ver.F*/
            RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R31, &bbpreg);
            bbpreg &= (~0x3);
            RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R31, bbpreg);
        }
        /*
             RT3070(E) Version[0200]
             RT3070(F) Version[0201]
         */
        if (((pAd->MACVersion & 0xffff) < 0x0201))
        {
            RTMP_IO_WRITE32(pAd, TX_SW_CFG1, 0);
            RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0x2C);    /* To fix throughput drop drastically*/
        }
        else
        {
            RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0);
        }
    }
    else if (IS_RT3071(pAd) || IS_RT3572(pAd))
    {
        RTMP_IO_WRITE32(pAd, TX_SW_CFG1, 0);
        if (((pAd->MACVersion & 0xffff) < 0x0211))
        {
            if (pAd->NicConfig2.field.DACTestBit == 1)
            {
                RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0x1F); /* To fix throughput drop drastically*/
            }
            else
            {
                RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0x0F); /* To fix throughput drop drastically*/
            }
        }
        else
        {
            RTMP_IO_WRITE32(pAd, TX_SW_CFG2, 0x0);
        }
    }

    /* update registers from EEPROM for RT3071 or later(3572/3562/3592).*/
    if (IS_RT3090(pAd) || IS_RT3572(pAd) || IS_RT3390(pAd))
    {
        UCHAR RegIdx, RegValue;
        USHORT value;

        /* after RT3071, write BBP from EEPROM 0xF0 to 0x102*/
        for (i = 0xF0; i <= 0x102; i = i+2)
        {
            value = 0xFFFF;
            RT28xx_EEPROM_READ16(pAd, i, value);
            if ((value != 0xFFFF) && (value != 0))
            {
                RegIdx = (UCHAR)(value >> 8);
                RegValue  = (UCHAR)(value & 0xff);
                RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, RegIdx, RegValue);
                DBGPRINT(RT_DEBUG_TRACE, "Update BBP Registers from EEPROM(0x%0x), BBP(0x%x) = 0x%x\n", i, RegIdx, RegValue);
            }
        }

        /* after RT3071, write RF from EEPROM 0x104 to 0x116*/
        for (i = 0x104; i <= 0x116; i = i+2)
        {
            value = 0xFFFF;
            RT28xx_EEPROM_READ16(pAd, i, value);
            if ((value != 0xFFFF) && (value != 0))
            {
                RegIdx = (UCHAR)(value >> 8);
                RegValue  = (UCHAR)(value & 0xff);
                RT30xxWriteRFRegister(pAd, RegIdx, RegValue);
                DBGPRINT(RT_DEBUG_TRACE, "Update RF Registers from EEPROM0x%x), BBP(0x%x) = 0x%x\n", i, RegIdx, RegValue);
            }
        }
    }
#endif /* RT30xx */

#ifdef CONFIG_STA_SUPPORT
#ifdef RTMP_FREQ_CALIBRATION_SUPPORT
        /*
            Only for RT3593, RT5390 (Maybe add other chip in the future)
            Sometimes the frequency will be shift, we need to adjust it.
        */
        if (pAd->StaCfg.AdaptiveFreq == TRUE) /*Todo: iwpriv and profile support.*/
        pAd->FreqCalibrationCtrl.bEnableFrequencyCalibration = TRUE;

        DBGPRINT(RT_DEBUG_TRACE, ("%s: pAd->FreqCalibrationCtrl.bEnableFrequencyCalibration = %d\n", 
            __FUNCTION__, 
            pAd->FreqCalibrationCtrl.bEnableFrequencyCalibration));

#endif /* RTMP_FREQ_CALIBRATION_SUPPORT */
#endif /* CONFIG_STA_SUPPORT */
    DBGPRINT(RT_DEBUG_TRACE, "TxPath = %d, RxPath = %d, RFIC=%d\n", 
                pAd->Antenna.field.TxPath, pAd->Antenna.field.RxPath, pAd->RfIcType);
    DBGPRINT(RT_DEBUG_TRACE, "<-- NICInitAsicFromEEPROM\n");
}



NDIS_STATUS NICLoadFirmware(
    IN PRTMP_ADAPTER pAd)
{
    return RtmpAsicLoadFirmware(pAd);
}

/*
    ========================================================================
    
    Routine Description:
        Initialize port configuration structure

    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        None

    IRQL = PASSIVE_LEVEL

    Note:
        
    ========================================================================
*/
VOID    UserCfgInit(
    IN    PRTMP_ADAPTER pAd)
{
    extern wnet_envar_t *p_wnet_envar;
    wnet_config_t *p_cfg = &p_wnet_envar->working_param;

    if ((p_cfg->channel < 1)||(p_cfg->channel > 13)) {
        p_cfg->channel = 11; /* default */
    }

    if ((p_cfg->txrate != 1)&&(p_cfg->txrate != 2)&&(p_cfg->txrate != 6)) {
        p_cfg->txrate = 6; /* default */
    }

    pAd->CommonCfg.Channel = p_cfg->channel;
    pAd->CommonCfg.TxRate = p_cfg->txrate;
    pAd->CommonCfg.Mode =  p_cfg->mode;

    if(p_cfg->mode == WNET_TEST_MODE) {
        pAd->CommonCfg.MacAddr[0] = 0x00;
        pAd->CommonCfg.MacAddr[1] = 0x00;
        pAd->CommonCfg.MacAddr[2] = 0x00;
        pAd->CommonCfg.MacAddr[3] = 0x00;
        pAd->CommonCfg.MacAddr[4] = 0x00;
        pAd->CommonCfg.MacAddr[5] = 0x00;
    }
    else {
        /* Mac address is bind to the CPU's unique ID */
        pAd->CommonCfg.MacAddr[0] = 0x00;
        pAd->CommonCfg.MacAddr[1] = 0x11;
        pAd->CommonCfg.MacAddr[2] = 0x22;
        pAd->CommonCfg.MacAddr[3] = des(2);
        pAd->CommonCfg.MacAddr[4] = des(1);
        pAd->CommonCfg.MacAddr[5] = des(0);
    }
}
/*
    ========================================================================
    
    Routine Description:
        Initialize ASIC

    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        None

    IRQL = PASSIVE_LEVEL

    Note:
        
    ========================================================================
*/
NDIS_STATUS    NICInitializeAsic(
    IN    PRTMP_ADAPTER    pAd,
    IN  BOOLEAN        bHardReset)
{
    ULONG            Index = 0;
    UCHAR            R0 = 0xff;
    UINT32            MacCsr12 = 0, Counter = 0;
#ifdef RTMP_MAC_USB
    UINT32            MacCsr0 = 0;
#endif /* RTMP_MAC_USB */
    USHORT            KeyIdx;
    INT                i,apidx;

    DBGPRINT(RT_DEBUG_TRACE, "--> NICInitializeAsic\n");



#ifdef RTMP_MAC_USB
    
    /* Make sure MAC gets ready after NICLoadFirmware().*/
    
    Index = 0;
    
    /*To avoid hang-on issue when interface up in kernel 2.4, */
    /*we use a local variable "MacCsr0" instead of using "pAd->MACVersion" directly.*/
    do 
    {
        RTMP_IO_READ32(pAd, MAC_CSR0, &MacCsr0);

        if ((MacCsr0 != 0x00) && (MacCsr0 != 0xFFFFFFFF))
            break;

        RTMPusecDelay(10);
    } while (Index++ < 100);

    pAd->MACVersion = MacCsr0;
    DBGPRINT(RT_DEBUG_TRACE, "MAC_CSR0  [ Ver:Rev=0x%08x]\n", pAd->MACVersion);
    /* turn on bit13 (set to zero) after rt2860D. This is to solve high-current issue.*/
    RTMP_IO_READ32(pAd, PBF_SYS_CTRL, &MacCsr12);
    MacCsr12 &= (~0x2000);
    RTMP_IO_WRITE32(pAd, PBF_SYS_CTRL, MacCsr12);
    
    RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0x3);
    RTMP_IO_WRITE32(pAd, USB_DMA_CFG, 0x0);
    RTUSBVenderReset(pAd);
    RTMPusecDelay(1); 
    RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0x0);

    /* Initialize MAC register to default value*/
    for(Index=0; Index<NUM_MAC_REG_PARMS; Index++)
    {
        u32 reg_val = MACRegTable[Index].Value;
#ifdef RT30xx
        if ((MACRegTable[Index].Register == TX_SW_CFG0) &&
            (IS_RT3070(pAd) || IS_RT3071(pAd) || IS_RT3572(pAd) || IS_RT3390(pAd) || IS_RT3593(pAd)))
        {
            reg_val    = 0x00000400;
        }
        else if ((MACRegTable[Index].Register == TX_SW_CFG0) && (IS_RT5390(pAd)))
        {
            reg_val    = 0x00000404; /* Gary, 2010-6-9 */
        }
#endif /* RT30xx */
        RTMP_IO_WRITE32(pAd, (USHORT)MACRegTable[Index].Register, reg_val);
    }


#ifdef CONFIG_STA_SUPPORT
    //IF_DEV_CONFIG_OPMODE_ON_STA(pAd)
    {
        for (Index = 0; Index < NUM_STA_MAC_REG_PARMS; Index++)
        {
            RTMP_IO_WRITE32(pAd, (USHORT)STAMACRegTable[Index].Register, STAMACRegTable[Index].Value);
        }
    }
#endif /* CONFIG_STA_SUPPORT */
#endif /* RTMP_MAC_USB */


#ifdef SPECIFIC_BCN_BUF_SUPPORT
    if (pAd->chipCap.FlgIsSupSpecBcnBuf == TRUE)
    {
        /* re-set beacon offset */
        for(Index=0; Index<NUM_BCN_SPEC_MAC_REG_PARMS; Index++)
        {
            RTMP_IO_WRITE32(pAd, (USHORT)BcnSpecMACRegTable[Index].Register, BcnSpecMACRegTable[Index].Value);
        }
    }
#endif /* SPECIFIC_BCN_BUF_SUPPORT */

    /* re-set specific MAC registers */
    if (pAd->chipOps.AsicMacInit != NULL)
        pAd->chipOps.AsicMacInit(pAd);

    
    /* Before program BBP, we need to wait BBP/RF get wake up.*/
    
    Index = 0;
    do
    {
        RTMP_IO_READ32(pAd, MAC_STATUS_CFG, &MacCsr12);

        if ((MacCsr12 & 0x03) == 0)    /* if BB.RF is stable*/
            break;
        
        DBGPRINT(RT_DEBUG_TRACE, "Check MAC_STATUS_CFG  = Busy = %x\n", MacCsr12);
        RTMPusecDelay(1000);
    } while (Index++ < 100);


    /* Wait to be stable.*/
    RTMPusecDelay(1000);
    pAd->LastMCUCmd = 0x72;

    /* Read BBP register, make sure BBP is up and running before write new data*/
    Index = 0;
    do 
    {
        RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R0, &R0);
        DBGPRINT(RT_DEBUG_TRACE, "BBP version = %x\n", R0);
    } while ((++Index < 20) && ((R0 == 0xff) || (R0 == 0x00)));
    /*ASSERT(Index < 20); this will cause BSOD on Check-build driver*/

    if ((R0 == 0xff) || (R0 == 0x00))
        return NDIS_STATUS_FAILURE;

    /* Initialize BBP register to default value*/
    for (Index = 0; Index < NUM_BBP_REG_PARMS; Index++)
    {
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBPRegTable[Index].Register, BBPRegTable[Index].Value);
    }
    if (pAd->chipCap.pBBPRegTable)
    {
        REG_PAIR *pbbpRegTb = pAd->chipCap.pBBPRegTable;
        
        for (Index = 0; Index < pAd->chipCap.bbpRegTbSize; Index++)
        {
            RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, pbbpRegTb[Index].Register, pbbpRegTb[Index].Value);
            DBGPRINT(RT_DEBUG_TRACE, "BBP_R%d=%d\n", pbbpRegTb[Index].Register, pbbpRegTb[Index].Value);
        }
    }

    if (pAd->chipOps.AsicBbpInit != NULL)
        pAd->chipOps.AsicBbpInit(pAd);


    RTMP_VDR_TUNING1(pAd);

    

    /* for rt2860E and after, init BBP_R84 with 0x19. This is for extension channel overlapping IOT.*/
    /* RT3090 should not program BBP R84 to 0x19, otherwise TX will block.*/
    /*3070/71/72,3090,3090A( are included in RT30xx),3572,3390*/
#if !defined(RT5350)
    if (((pAd->MACVersion & 0xffff) != 0x0101) &&
        !(IS_RT30xx(pAd)|| IS_RT3572(pAd) || IS_RT5390(pAd)))
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R84, 0x19);
#endif /* RT5350 */

#ifdef RT30xx
    /* RF power sequence setup*/
    if (IS_RT30xx(pAd) || IS_RT3572(pAd) || IS_RT5390(pAd))
    {    /*update for RT3070/71/72/90/91/92,3572,3390.*/
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R79, 0x13);        
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R80, 0x05);    
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R81, 0x33);    
    }
#endif /* RT30xx */

    if (pAd->MACVersion == 0x28600100)
    {
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R69, 0x16);
        RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R73, 0x12);
    }
    
    if ((pAd->MACVersion == RALINK_3883_VERSION) ||
        ((pAd->MACVersion >= RALINK_2880E_VERSION) &&
        (pAd->MACVersion < RALINK_3070_VERSION))) /* 3*3*/
    {
        /* enlarge MAX_LEN_CFG*/
        UINT32 csr;
        RTMP_IO_READ32(pAd, MAX_LEN_CFG, &csr);
        {
        csr &= 0xFFF;
        csr |= 0x2000;
        }
        RTMP_IO_WRITE32(pAd, MAX_LEN_CFG, csr);
    }

#ifdef RTMP_MAC_USB
{
    UCHAR    MAC_Value[]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0,0};

    /*Initialize WCID table*/
    for(Index =0 ;Index < 254;Index++)
    {
        RTUSBMultiWrite(pAd, (USHORT)(MAC_WCID_BASE + Index * 8), MAC_Value, 8);
    }
}
#endif /* RTMP_MAC_USB */

#if 0 //def CONFIG_STA_SUPPORT
    /* Add radio off control*/
    IF_DEV_CONFIG_OPMODE_ON_STA(pAd)
    {
        if (pAd->StaCfg.bRadio == FALSE)
        {
/*            RTMP_IO_WRITE32(pAd, PWR_PIN_CFG, 0x00001818);*/
            RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_RADIO_OFF);
            DBGPRINT(RT_DEBUG_TRACE, ("Set Radio Off\n"));
        }
    }
#endif /* CONFIG_STA_SUPPORT */    

    /* Clear raw counters*/
    RTMP_IO_READ32(pAd, RX_STA_CNT0, &Counter);
    RTMP_IO_READ32(pAd, RX_STA_CNT1, &Counter);
    RTMP_IO_READ32(pAd, RX_STA_CNT2, &Counter);
    RTMP_IO_READ32(pAd, TX_STA_CNT0, &Counter);
    RTMP_IO_READ32(pAd, TX_STA_CNT1, &Counter);
    RTMP_IO_READ32(pAd, TX_STA_CNT2, &Counter);
    
    /* ASIC will keep garbage value after boot*/
    /* Clear all shared key table when initial*/
    /* This routine can be ignored in radio-ON/OFF operation. */
    if (bHardReset)
    {
        for (KeyIdx = 0; KeyIdx < 4; KeyIdx++)
        {
            RTMP_IO_WRITE32(pAd, SHARED_KEY_MODE_BASE + 4*KeyIdx, 0);
        }

        /* Clear all pairwise key table when initial*/
        for (KeyIdx = 0; KeyIdx < 256; KeyIdx++)
        {
            RTMP_IO_WRITE32(pAd, MAC_WCID_ATTRIBUTE_BASE + (KeyIdx * HW_WCID_ATTRI_SIZE), 1);
        }
    }
    
    /* assert HOST ready bit*/
/*  RTMP_IO_WRITE32(pAd, MAC_CSR1, 0x0);  2004-09-14 asked by Mark*/
/*  RTMP_IO_WRITE32(pAd, MAC_CSR1, 0x4);*/

    /* It isn't necessary to clear this space when not hard reset.     */
    if (bHardReset == TRUE)
    {
#ifdef SPECIFIC_BCN_BUF_SUPPORT
        unsigned long irqFlag = 0;
#endif /* SPECIFIC_BCN_BUF_SUPPORT */
    
        /* clear all on-chip BEACON frame space            */
#ifdef SPECIFIC_BCN_BUF_SUPPORT
        /*
            Shared memory access selection (higher 8KB shared memory)
        */
        RTMP_MAC_SHR_MSEL_LOCK(pAd, HIGHER_SHRMEM, irqFlag);
#endif /* SPECIFIC_BCN_BUF_SUPPORT */

        for (apidx = 0; apidx < HW_BEACON_MAX_COUNT(pAd); apidx++)
        {
            for (i = 0; i < HW_BEACON_OFFSET; i+=4)
                RTMP_IO_WRITE32(pAd, pAd->BeaconOffset[apidx] + i, 0x00); 
        }
        
#ifdef SPECIFIC_BCN_BUF_SUPPORT
        /*
            Shared memory access selection (lower 16KB shared memory)
        */
        RTMP_MAC_SHR_MSEL_UNLOCK(pAd, LOWER_SHRMEM, irqFlag);    
#endif /* SPECIFIC_BCN_BUF_SUPPORT */
    }
    
#ifdef RTMP_MAC_USB
    AsicDisableSync(pAd);
    /* Clear raw counters*/
    RTMP_IO_READ32(pAd, RX_STA_CNT0, &Counter);
    RTMP_IO_READ32(pAd, RX_STA_CNT1, &Counter);
    RTMP_IO_READ32(pAd, RX_STA_CNT2, &Counter);
    RTMP_IO_READ32(pAd, TX_STA_CNT0, &Counter);
    RTMP_IO_READ32(pAd, TX_STA_CNT1, &Counter);
    RTMP_IO_READ32(pAd, TX_STA_CNT2, &Counter);
    /* Default PCI clock cycle per ms is different as default setting, which is based on PCI.*/
    RTMP_IO_READ32(pAd, USB_CYC_CFG, &Counter);
    Counter&=0xffffff00;
    Counter|=0x000001e;
    RTMP_IO_WRITE32(pAd, USB_CYC_CFG, Counter);
#endif /* RTMP_MAC_USB */

#ifdef CONFIG_STA_SUPPORT
    //IF_DEV_CONFIG_OPMODE_ON_STA(pAd)
    {
        /* for rt2860E and after, init TXOP_CTRL_CFG with 0x583f. This is for extension channel overlapping IOT.*/
        if ((pAd->MACVersion&0xffff) != 0x0101)
            RTMP_IO_WRITE32(pAd, TXOP_CTRL_CFG, 0x583f);
    }
#endif /* CONFIG_STA_SUPPORT */

    DBGPRINT(RT_DEBUG_TRACE, "<-- NICInitializeAsic\n");
    return NDIS_STATUS_SUCCESS;
}


/*
    ========================================================================
    
    Routine Description:
        Initialize NIC hardware

    Arguments:
        Adapter                        Pointer to our adapter

    Return Value:
        None

    IRQL = PASSIVE_LEVEL

    Note:
        
    ========================================================================
*/
NDIS_STATUS    NICInitializeAdapter(
    IN    PRTMP_ADAPTER    pAd,
    IN   BOOLEAN    bHardReset)
{
    NDIS_STATUS     Status = NDIS_STATUS_SUCCESS;
    WPDMA_GLO_CFG_STRUC    GloCfg;
/*    INT_MASK_CSR_STRUC        IntMask;*/
    ULONG    i =0;
    ULONG    j=0;
    /*AC_TXOP_CSR0_STRUC    csr0;*/

    DBGPRINT(RT_DEBUG_TRACE, "--> NICInitializeAdapter\n");
    
    /* 3. Set DMA global configuration except TX_DMA_EN and RX_DMA_EN bits:*/
retry:
    i = 0;
    do
    {
        RTMP_IO_READ32(pAd, WPDMA_GLO_CFG, &GloCfg.word);
        if ((GloCfg.field.TxDMABusy == 0)  && (GloCfg.field.RxDMABusy == 0))
            break;
        
        RTMPusecDelay(1000);
        i++;
    }while ( i<100);
    DBGPRINT(RT_DEBUG_TRACE, "<== DMA offset 0x208 = 0x%x\n", GloCfg.word);    
    GloCfg.word &= 0xff0;
    GloCfg.field.EnTXWriteBackDDONE =1;
    RTMP_IO_WRITE32(pAd, WPDMA_GLO_CFG, GloCfg.word);
    
    /* Record HW Beacon offset*/
    for(i=0;i<HW_BEACON_MAX_NUM;i++)
    {
        pAd->BeaconOffset[i] = pAd->chipCap.BcnBase[i];
    }

    /* write all shared Ring's base address into ASIC*/
    

    /* asic simulation sequence put this ahead before loading firmware.*/
    /* pbf hardware reset*/

    /* Initialze ASIC for TX & Rx operation*/
    if (NICInitializeAsic(pAd , bHardReset) != NDIS_STATUS_SUCCESS)
    {
        if (j++ == 0)
        {
            NICLoadFirmware(pAd);
            goto retry;
        }
        return NDIS_STATUS_FAILURE;
    }

    /* reset action*/
    /* Load firmware*/
    /*  Status = NICLoadFirmware(pAd);*/

    DBGPRINT(RT_DEBUG_TRACE, "<-- NICInitializeAdapter\n");
    return Status;
}


/*
    ========================================================================
    
    Routine Description:
        Enable RX 

    Arguments:
        pAd                        Pointer to our adapter

    Return Value:
        None

    IRQL <= DISPATCH_LEVEL
    
    Note:
        Before Enable RX, make sure you have enabled Interrupt.
    ========================================================================
*/
VOID RTMPEnableRxTx(
    IN PRTMP_ADAPTER    pAd)
{
/*    WPDMA_GLO_CFG_STRUC    GloCfg;*/
/*    ULONG    i = 0;*/
    UINT32 rx_filter_flag;

    DBGPRINT(RT_DEBUG_TRACE, "==> RTMPEnableRxTx\n");

    /* Enable Rx DMA.*/
    RT28XXDMAEnable(pAd);

    /* enable RX of MAC block*/
    {
        rx_filter_flag = STANORMAL;     /* Staion not drop control frame will fail WiFi Certification.*/
        RTMP_IO_WRITE32(pAd, RX_FILTR_CFG, rx_filter_flag);
    }
    
    {
        RTMP_IO_WRITE32(pAd, MAC_SYS_CTRL, 0xc);
    }

    {
        UINT32 reg = 0;
        RTMP_IO_READ32(pAd, 0x1300, &reg);  /* clear garbage interrupts*/
        DBGPRINT(RT_DEBUG_TRACE,"0x1300 = %08x\n", reg);
    }

    { /* Added by wangyf */
        RT28xxUsbAsicRadioOn(pAd);
        pAd->CurrentChannel = pAd->CommonCfg.Channel;
        AsicSwitchChannel(pAd, pAd->CurrentChannel, TRUE);
        DBGPRINT(RT_DEBUG_INFO, "Switch to channel <%d>\n",pAd->CurrentChannel);
    }

    DBGPRINT(RT_DEBUG_TRACE, "<== RTMPEnableRxTx\n");    
}

void rt28xx_init(PRTMP_ADAPTER pAd)
{
    UINT                    index;
    NDIS_STATUS                Status;
    UINT32                     MacCsr0 = 0;
    
    /* Make sure MAC gets ready.*/
    index = 0;
    do
    {
        RTMP_IO_READ32(pAd, MAC_CSR0, &MacCsr0);
        pAd->MACVersion = MacCsr0;

        if ((pAd->MACVersion != 0x00) && (pAd->MACVersion != 0xFFFFFFFF))
            break;

        RTMPusecDelay(10);
    } while (index++ < 100);
    DBGPRINT(RT_DEBUG_TRACE, "MAC_CSR0  [ Ver:Rev=0x%08x]\n", pAd->MACVersion);

    RtmpChipOpsHook(pAd);

    /* Disable DMA*/
    //RT28XXDMADisable(pAd);

    /* Load 8051 firmware*/
    Status = NICLoadFirmware(pAd);
    if (Status != NDIS_STATUS_SUCCESS)
    {
        DBGPRINT(RT_DEBUG_ERROR, "NICLoadFirmware failed, Status[=0x%08x]\n", Status);
        goto err1;
    }

    /* Init the hardware, we need to init asic before read registry, otherwise mac register will be reset*/
    Status = NICInitializeAdapter(pAd, TRUE);
    if (Status != NDIS_STATUS_SUCCESS)
    {
        DBGPRINT(RT_DEBUG_ERROR, "NICInitializeAdapter failed, Status[=0x%08x]\n", Status);
        if (Status != NDIS_STATUS_SUCCESS)
        goto err1;
    }    

    /* We should read EEPROM for all cases.  rt2860b*/
    NICReadEEPROMParameters(pAd, (PSTRING)pAd->CommonCfg.MacAddr);    

    NICInitAsicFromEEPROM(pAd); /*rt2860b*/

#ifdef RTMP_MAC_USB
    AsicSendCommandToMcu(pAd, 0x31, 0xff, 0x00, 0x02);
    RTMPusecDelay(10000);
#endif /* RTMP_MAC_USB */

    RTMP_CHIP_SPECIFIC(pAd, RTMP_CHIP_SPEC_STATE_INIT,
                        RTMP_CHIP_SPEC_INITIALIZATION, NULL, 0);

    /* Now Enable RxTx*/
    RTMPEnableRxTx(pAd);

    DBGPRINT(RT_DEBUG_INFO, "rt28xx Initialized success\n");

    #ifdef WIFI_ATE_MODE
    DBGPRINT(RT_DEBUG_INFO, "!NOTICE!!ATE MODE IS RUNNING!\n");
    #else
    #ifndef NDEBUG
    {
        extern UCHAR *DbgRxFilterAddrTable[];

        if (DbgRxFilterAddrTable[0]) {
            int i;
            DBGPRINT(RT_DEBUG_INFO, "!NOTICE!!RX FILTER IS RUNNING FOR DEBUG!\n");
            DBGPRINT(RT_DEBUG_INFO, "Only below addresses are allowed\n");
            for (i=0;;i++) {
                uint8_t *mac;
                if (DbgRxFilterAddrTable[i] == NULL) {
                    break;
                }

                mac = DbgRxFilterAddrTable[i];
                DBGPRINT(RT_DEBUG_INFO, "  <%02x:%02x:%02x:%02x:%02x:%02x>\n",\
                    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
            }
        }
    }
    #endif
    DBGPRINT(RT_DEBUG_INFO, "Start to receive from usb...\n");
    usb_bulkin(pAd->pUsb_Dev);

    pAd->init_complete = TRUE;
    #endif

    return;
err1:
    DBGPRINT(RT_DEBUG_ERROR, "!!! rt28xx Initialized fail !!!\n");
}


void rt2870_probe(void *pUsb_Dev, void *ppAd)
{
    PRTMP_ADAPTER pAd = &rtmp_adapter;

    memset(pAd, 0, sizeof(RTMP_ADAPTER));

    pAd->pUsb_Dev = pUsb_Dev;

    {
        rt_sem_t sem;
        sem = rt_sem_create("s-usbc", 1, RT_IPC_FLAG_PRIO);
        RT_ASSERT(sem != RT_NULL);
        pAd->UsbVendorReq_semaphore = sem;

        sem = rt_sem_create("s-usbc2", 1, RT_IPC_FLAG_PRIO);
        RT_ASSERT(sem != RT_NULL);
        pAd->UsbVendorReq_semaphore2 = sem;

    }


    


    *((PRTMP_ADAPTER *)ppAd) = pAd;


    UserCfgInit(pAd);
    rt28xx_init(pAd);

}



