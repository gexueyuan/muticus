/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_wnet_dsmp.c
 @brief  : implement dsmp,
 @author : wanglei
 @history:
           2014-12-11    wanglei    Created file
           ...
******************************************************************************/

#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "dsmp"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "cv_wnet.h"


#include "cv_dsmp.h"



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/


/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/


int dsmp_send(wnet_envar_t *p_wnet, wnet_txinfo_t *txinfo, 
                        uint8_t *pdata, uint32_t length)
{
    int ret = 0;
    int len = 0;
    /* add dsmp header to pdata */
    dsmp_hdr_t *p_dsmp_head = NULL;
    security_hdr_t *p_sec_head = NULL;

    
    len = DSMP_HEADER_LEN;
    pdata -= len;
    p_dsmp_head = (dsmp_hdr_t *)pdata;
    memset(p_dsmp_head, 0, len);

    /* Initial dsmp frame head domain. */
    p_dsmp_head->version = DSMP_VERSION;
    p_dsmp_head->aid = cv_ntohl(WAVE_AID_vehicle_safety);
    p_dsmp_head->element_id = WAVE_ELEMENT_ID_WSM;
    p_dsmp_head->dsm_length = cv_ntohs((uint16_t)length);
    length += len;
 
    /* send to llc layer or encryption */
    if (!txinfo->encryption) 
    {
        /* add 1609.2 header.  TBD */
        len = SECURITY_HEADER_LEN;
        pdata -= len;
        length += len;
        p_sec_head = (security_hdr_t *)pdata;
        p_sec_head->version = SEC_VERSION;
        p_sec_head->security_type = 0;
        
      //  ret = llc_send(p_wnet, txinfo, pdata, length); 
        ret = enet_send(p_wnet, txinfo, pdata, length); 
    }
    else 
    {
        /* add dot2_hdr and security_hdt */

        /* sent to enc module.  enc_offset indicate the data need to be
           encrypted */
       
    }

    return ret;
}


int dsmp_recv(wnet_envar_t *p_wnet, wnet_rxinfo_t *rxinfo, 
                       uint8_t *pdata, uint32_t length)
{
    int ret = 0;
    int len = 0;
    uint8_t id = 0;
    uint32_t aid = 0;
    dsmp_hdr_t *p_dsmp;
    security_info_t *p_sec_info = NULL;    
    security_hdr_t *p_sec_hdr  = (security_hdr_t *)pdata;
    

    if(p_sec_hdr->security_type)
    {
        /* security_header */
        len = SECURITY_HEADER_LEN;
        pdata += len;
        p_sec_info = (security_info_t *)pdata;
        len += SECURITY_INFO_LEN;
        pdata += len;        
    }
    else
    {
        len = SECURITY_HEADER_LEN;
        pdata += len;
    }


    p_dsmp = (dsmp_hdr_t *)pdata;        
    aid = cv_ntohl(p_dsmp->aid);
    if (aid == WAVE_AID_vehicle_safety)
    {
        id = p_dsmp->element_id;
        switch (id) 
        {
            case WAVE_ELEMENT_ID_WSM:
            {
                pdata += DSMP_HEADER_LEN;
                length = cv_ntohs(p_dsmp->dsm_length);
                ret = vam_rcp_recv(rxinfo, pdata, length);             
                break;
            }
            default:
            {
                /* realese rxbuf */
                ret = -1;
                OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Invalid wave elementID[0x%04x], dropped.\n", id);
                break; 
            }
        }
    }
    else
    {
        /* not support. but need to realese rxbuf */
        ret = -1;
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Invalid dsmp aid[0x%04x], dropped.\n", aid);
    }       
    
    return ret;
}


