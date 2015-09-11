/*
 *************************************************************************
 * Ralink Tech Inc.
 * 5F., No.36, Taiyuan St., Jhubei City,
 * Hsinchu County 302,
 * Taiwan, R.O.C.
 *
 * (c) Copyright 2002-2010, Ralink Technology, Inc.
 *
 * This program is free software; you can redistribute it and/or modify  *
 * it under the terms of the GNU General Public License as published by  *
 * the Free Software Foundation; either version 2 of the License, or     *
 * (at your option) any later version.                                   *
 *                                                                       *
 * This program is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 * GNU General Public License for more details.                          *
 *                                                                       *
 * You should have received a copy of the GNU General Public License     *
 * along with this program; if not, write to the                         *
 * Free Software Foundation, Inc.,                                       *
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                       *
 *************************************************************************/


#include "..\include\rt_include.h"


#ifdef RTMP_RF_RW_SUPPORT
/*
    ========================================================================
    
    Routine Description: Write RT30xx RF register through MAC

    Arguments:

    Return Value:

    IRQL = 
    
    Note:
    
    ========================================================================
*/
NDIS_STATUS RT30xxWriteRFRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            regID,
    IN    UCHAR            value)
{
    RF_CSR_CFG_STRUC    rfcsr = { { 0 } };
    UINT                i = 0;

    {
        //assert((regID <= pAd->chipCap.MaxNumOfRfId)); /* R0~R31 or R63*/

        do
        {
            RTMP_IO_READ32(pAd, RF_CSR_CFG, &rfcsr.word);

            if (!rfcsr.field.RF_CSR_KICK)
                break;
            i++;
        }
        while ((i < RETRY_LIMIT));

        if (i == RETRY_LIMIT)
        {
            DBGPRINT(RT_DEBUG_ERROR, "Retry count exhausted or device removed!!!\n");
            return STATUS_UNSUCCESSFUL;
        }

        if ((pAd->chipCap.RfReg17WtMethod == RF_REG_WT_METHOD_STEP_ON) &&
            (regID == RF_R17))
        {
            UINT32 IdRf;
            UCHAR RfValue;

            RT30xxReadRFRegister(pAd, RF_R17, &RfValue);

            rfcsr.field.RF_CSR_WR = 1;
            rfcsr.field.RF_CSR_KICK = 1;
            rfcsr.field.TESTCSR_RFACC_REGNUM = regID; /* R0~R31*/

            if (RfValue <= value)
            {
                for(IdRf=RfValue; IdRf<=value; IdRf++)
                {
                    rfcsr.field.RF_CSR_DATA = IdRf;
                    RTMP_IO_WRITE32(pAd, RF_CSR_CFG, rfcsr.word);
                }
            }
            else
            {
                for(IdRf=RfValue; IdRf>=value; IdRf--)
                {
                    rfcsr.field.RF_CSR_DATA = IdRf;
                    RTMP_IO_WRITE32(pAd, RF_CSR_CFG, rfcsr.word);
                }
            }
        }
        else
        {
            rfcsr.field.RF_CSR_WR = 1;
            rfcsr.field.RF_CSR_KICK = 1;
            rfcsr.field.TESTCSR_RFACC_REGNUM = regID; /* R0~R31*/
            rfcsr.field.RF_CSR_DATA = value;
            RTMP_IO_WRITE32(pAd, RF_CSR_CFG, rfcsr.word);
        }
    }

    return NDIS_STATUS_SUCCESS;
}


/*
    ========================================================================
    
    Routine Description: Read RT30xx RF register through MAC

    Arguments:

    Return Value:

    IRQL = 
    
    Note:
    
    ========================================================================
*/
NDIS_STATUS RT30xxReadRFRegister(
    IN    PRTMP_ADAPTER    pAd,
    IN    UCHAR            regID,
    IN    PUCHAR            pValue)
{
    RF_CSR_CFG_STRUC    rfcsr = { { 0 } };
    UINT                i=0, k=0;

    {
        //assert((regID <= pAd->chipCap.MaxNumOfRfId)); /* R0~R63*/

        for (i=0; i<MAX_BUSY_COUNT; i++)
        {
            RTMP_IO_READ32(pAd, RF_CSR_CFG, &rfcsr.word);

            if (rfcsr.field.RF_CSR_KICK == BUSY)                                    
            {                                                                
                continue;                                                    
            }                                                                
            rfcsr.word = 0;
            rfcsr.field.RF_CSR_WR = 0;
            rfcsr.field.RF_CSR_KICK = 1;
            rfcsr.field.TESTCSR_RFACC_REGNUM = regID;
            RTMP_IO_WRITE32(pAd, RF_CSR_CFG, rfcsr.word);
            for (k=0; k<MAX_BUSY_COUNT; k++)
            {
                RTMP_IO_READ32(pAd, RF_CSR_CFG, &rfcsr.word);

                if (rfcsr.field.RF_CSR_KICK == IDLE)
                    break;
            }
            if ((rfcsr.field.RF_CSR_KICK == IDLE) &&
                (rfcsr.field.TESTCSR_RFACC_REGNUM == regID))
            {
                *pValue = (UCHAR)(rfcsr.field.RF_CSR_DATA);
                break;
            }
        }
        if (rfcsr.field.RF_CSR_KICK == BUSY)
        {                                                                    
            DBGPRINT(RT_DEBUG_ERROR, "RF read R%d=0x%X fail, i[%d], k[%d]\n", regID, rfcsr.word,i,k);
            return STATUS_UNSUCCESSFUL;
        }
    }

    return STATUS_SUCCESS;
}


VOID NICInitRFRegisters(
    IN RTMP_ADAPTER *pAd)
{
    if (pAd->chipOps.AsicRfInit)
        pAd->chipOps.AsicRfInit(pAd);
}

#endif /* RTMP_RF_RW_SUPPORT */

