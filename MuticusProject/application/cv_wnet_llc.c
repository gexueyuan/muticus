/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_wnet_llc.c
 @brief  : This file realizes the LLC layer of the network.
 @author : wangyf
 @history:
           2014-12-8    wangyf    Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "llc"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_wnet.h"


/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

int llc_send(wnet_envar_t *p_wnet, wnet_txinfo_t *txinfo, uint8_t *pdata, uint32_t length)
{
    wnet_llc_header_t *llc;


    /* Reserve room for LLC Header */
    pdata -= WNET_LLC_HEADER_LEN;
    length += WNET_LLC_HEADER_LEN;

    llc = (wnet_llc_header_t *)pdata;
    
    memset(llc, 0, WNET_LLC_HEADER_LEN);
    llc->dsap = LLC_SAP;
    llc->ssap = LLC_SAP;

    switch (txinfo->protocol) 
    {
        case WNET_TRANS_PROT_DSMP:
        { 
            llc->ether_type = cv_ntohs(LLC_ETHERTYPE_DSMPv1);    break;
        }
    
        default:
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Invalid protocol[0x%04x], dropped.\n", txinfo->protocol);
            return -1;
        }
    }
    
    return fp_send(p_wnet, txinfo, pdata, length);
}


int llc_recv(wnet_envar_t *p_wnet, wnet_rxinfo_t *rxinfo, uint8_t *pdata, uint32_t length)
{
    int r;
    wnet_llc_header_t *llc;

    if (length < WNET_LLC_HEADER_LEN)
    {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Recv frame length [%d] is too short, dropped.\n", length);
        return -1;
    }

    llc = (wnet_llc_header_t *)pdata;

    /* Prepare for next step */
    pdata += WNET_LLC_HEADER_LEN;
    length -= WNET_LLC_HEADER_LEN;

    switch (cv_ntohs(llc->ether_type)) 
    {
        case LLC_ETHERTYPE_DSMPv1:
        {
            rxinfo->protocol = WNET_TRANS_PROT_DSMP;
            r = dsmp_recv(p_wnet, rxinfo, pdata, length);    
            break;       
        }
        
        default:
        {
            r = -1;
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Invalid ether type[0x%04x], dropped.\n", llc->ether_type);
            break;
        }
    }   

    return r;
}


















