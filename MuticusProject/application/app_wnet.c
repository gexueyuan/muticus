/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : app_wnet.c
 @brief  : this file include the application functions for the wireless network 
           transport module.
 @author : wangxianwen
 @history:
           2015-6-29    wangxianwen    Created file
           ...
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "app_wnet.h"

#include "stm32f4xx.h"


#include <string.h>
#include "..\include\rt_include.h"
#include "cv_wnet.h"

#include "cv_cms_def.h"



extern wnet_envar_t *p_wnet_envar;



/* Broadcast address. */
static const uint8_t BroadcastAddr[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

/* Wnet receiver buffer. */
static uint8_t WnetRxBuffer[WNET_RX_BUFFER_SIZE] = { 0 };



/**
  * @brief  Build ethernet frame and send out.
  * @param  See below.
  * @retval Not sure.
  */
int enet_send
(
    /* Pointer to wireless network layer's environment structure. */
    wnet_envar_t *p_wnet, 

    /* Pointer to wireless network layer transfer information. */
    wnet_txinfo_t *txinfo, 

    /* Data address. */
    uint8_t *pdata, 

    /* Data length. */
    uint32_t length
)
{
    wnet_enet_header_st_ptr enet_header_ptr = NULL;

    
    /* Reserve room for ENET header. */
    pdata -= WNET_ENET_HEADER_ST_LEN;
    length += WNET_ENET_HEADER_ST_LEN;
    enet_header_ptr = (wnet_enet_header_st_ptr)pdata;

    /* Initialize enet frame header.  */
    memset(enet_header_ptr, 0, WNET_ENET_HEADER_ST_LEN);

    /* Set enet frame destination address and source address. */
    COPY_MAC_ADDR(enet_header_ptr->dest_address, BroadcastAddr);
    
    /* Mac address is bind to the CPU's unique ID */
    enet_header_ptr->src_address[0] = 0x00;
    enet_header_ptr->src_address[1] = 0x11;
    enet_header_ptr->src_address[2] = 0x22;
    enet_header_ptr->src_address[3] = des(2);
    enet_header_ptr->src_address[4] = des(1);
    enet_header_ptr->src_address[5] = des(0);


    switch (txinfo->protocol) 
    {
        case WNET_TRANS_PROT_DSMP:
        { 
            enet_header_ptr->ether_type = cv_ntohs(LLC_ETHERTYPE_DSMPv1);    break;
        }
    
        default:
        {
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Invalid protocol[0x%04x], dropped.\n", txinfo->protocol);
            return -1;
        }
    }
    
    return fp_send(p_wnet, txinfo, pdata, length);
}


/**
  * @brief  Break up ethernet frame.
  * @param  See below.
  * @retval Not sure.
  */
int enet_receive
(
    /* Pointer to wireless network layer's environment structure. */
    wnet_envar_t *p_wnet, 

    /* Pointer to wireless network layer receive information. */
    wnet_rxinfo_t *rxinfo, 

    /* Data address. */
    uint8_t *pdata, 

    /* Data length. */
    uint32_t length
)
{
    int                              result = -1;
    wnet_enet_header_st_ptr enet_header_ptr = (wnet_enet_header_st_ptr)pdata;


    /* Check the frame length. */
    if (length < WNET_ENET_HEADER_ST_LEN)
    {
        OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Recv frame length [%d] is too short, dropped.\n", length);
        return -1;
    }

    /* Prepare for next step */
    pdata += WNET_ENET_HEADER_ST_LEN;
    length -= WNET_ENET_HEADER_ST_LEN;

    switch (cv_ntohs(enet_header_ptr->ether_type)) 
    {
        case LLC_ETHERTYPE_DSMPv1:
        {
            rxinfo->protocol = WNET_TRANS_PROT_DSMP;
            result = dsmp_recv(p_wnet, rxinfo, pdata, length);    
            break;       
        }
        
        default:
        {
            result = -1;
            OSAL_MODULE_DBGPRT(MODULE_NAME, OSAL_DEBUG_INFO, "Invalid ether type[0x%04x], dropped.\n", enet_header_ptr->ether_type);
            break;
        }
    }   

    return result;
}


/**
  * @brief  wireless network usart send routine.
  * @param  See below.
  * @retval OK.
  */
int wnet_usart_send
(
    /* Pointer to wireless network layer transfer information. */
    wnet_txinfo_t *txinfo,

    /* Data address. */
    uint8_t *src_ptr,

    /* Data length. */
    int32_t length
)
{
    wnet_main_header_st_ptr main_header_ptr = NULL;
    drv_wnet_isr_register_st rx_compelete_isr = { (void (*)(void *))fp_tx_complete, p_wnet_envar };


    /* Reserve room for ENET main header. */
    src_ptr -= WNET_MAIN_HEADER_ST_LEN;
    length += WNET_MAIN_HEADER_ST_LEN;
    main_header_ptr = (wnet_main_header_st_ptr)src_ptr;

    /* Initialize enet frame header.  */
    memset(main_header_ptr, 0, WNET_MAIN_HEADER_ST_LEN);

    /* Set magic number group. */
    main_header_ptr->magic_num1 = WNET_MAIN_HEADER_MAGIC_NUM1;
    main_header_ptr->magic_num2 = WNET_MAIN_HEADER_MAGIC_NUM2;

    main_header_ptr->length = cv_ntohs(length - 4);
    main_header_ptr->f_major_type = cv_ntohs(WNET_F_MAJOR_TYPE_RAW_DATA);
    main_header_ptr->f_minor_type = cv_ntohs(WNET_FT_MINOR_RAW_ETHERNET);

    /* Register tx complete isr and start the transfer. */
    DRV_WNET_PTR->ioctl(DRV_WNET_PTR, WNET_IOCTL_REGISTER_TXTC_ISR, &rx_compelete_isr);
    DRV_WNET_PTR->write(DRV_WNET_PTR, 0, src_ptr, length); 

    return 0; 
}






/**
  * @brief  wireless network receive enet routine.
  * @param  See below.
  * @retval OK.
  */
void wnet_receive_enet
(
    /* Data pointer. */
    uint8_t *data_ptr,

    /* Data valid length. */
    uint32_t length 
)
{
    wnet_enet_header_st_ptr enet_header_ptr = (wnet_enet_header_st_ptr)data_ptr;
    wnet_rxinfo_t                    rxinfo = { 0 };


    /* Data validation check. */
    if( (enet_header_ptr == NULL) || (length < WNET_ENET_HEADER_ST_LEN) )
    {
        return;
    }
    
    /* Initialize receiver information structure. */
    memcpy(&rxinfo.src.mac.addr, enet_header_ptr->src_address, MAC_ADDR_LEN);

    /* Allocate rooms for enet frame and link into rx waiting list. */
    wnet_recv(&rxinfo, (uint8_t*)enet_header_ptr, length);
}


#if WNET_USE_IDLE_INTERRUPT


/**
  * @brief  wireless network receive main routine.
  * @param  See below.
  * @retval OK.
  */
void wnet_receive_main
(
    /* Dummy data pointer only for matching format. */
    void *dummy_ptr
)
{
    wnet_main_header_st_ptr main_header_ptr = NULL;
    drv_wnet_isr_register_st    rx_idle_isr = { wnet_receive_main, WnetRxBuffer };
    uint16_t                     data_index = 0;
    
    
    /* Search for wnet main header from receiver buffer. */
    do
    {
        main_header_ptr = (wnet_main_header_st_ptr)((uint8_t*)WnetRxBuffer + data_index);
      
        if (data_index < sizeof(WnetRxBuffer) / 2)    {  data_index ++; }
        else                                          {  break;         }
        
    }while( (main_header_ptr->magic_num1 != WNET_MAIN_HEADER_MAGIC_NUM1) 
         || (main_header_ptr->magic_num2 != WNET_MAIN_HEADER_MAGIC_NUM2) );

    /* Format byte stream to host form for next data validation check. */
    main_header_ptr->length = cv_ntohs(main_header_ptr->length);
    main_header_ptr->f_major_type = cv_ntohs(main_header_ptr->f_major_type);
    main_header_ptr->f_minor_type = cv_ntohs(main_header_ptr->f_minor_type);

    /* Go to read the next frame when data invalid. */
    if ( (data_index == sizeof(WnetRxBuffer) / 2) 
      || (main_header_ptr->length < (sizeof(main_header_ptr->f_major_type) + sizeof(main_header_ptr->f_minor_type))) )
    {
        goto READ_NEXT_FRAME;
    }

    /* Analyse frame data based on frame type. */
    switch(main_header_ptr->f_major_type)
    {
        case WNET_F_MAJOR_TYPE_KEEPALIVE:     {  break;  }
        case WNET_F_MAJOR_TYPE_CONFIG_ASK:    {  break;  }
        case WNET_F_MAJOR_TYPE_EVENT_REPORT:  {  break;  }
        case WNET_F_MAJOR_TYPE_RAW_DATA:      
        {  
            wnet_receive_enet((uint8_t *)main_header_ptr + WNET_MAIN_HEADER_ST_LEN, 
                              main_header_ptr->length - sizeof(main_header_ptr->f_major_type) - sizeof(main_header_ptr->f_minor_type));
            break;  
        }
        default:                              {  break;  }
    }

    
READ_NEXT_FRAME:

    /* Clear the receive buffer. */
    memset(WnetRxBuffer, 0, sizeof(WnetRxBuffer));
    
    /* Active the read operation for wnet module. */
    DRV_WNET_PTR->ioctl(DRV_WNET_PTR, WNET_IOCTL_REGISTER_LINE_IDLE_ISR, &rx_idle_isr);
    DRV_WNET_PTR->read(DRV_WNET_PTR, 0, WnetRxBuffer, sizeof(WnetRxBuffer));
}

#else


/**
  * @brief  wireless network frame body receive routine.
  * @param  See below.
  * @retval OK.
  */
static void wnet_data_body_rxisr
(
    /* Pointer to data buffer. */
    void *buffer_ptr
)
{
    wnet_main_header_st_ptr main_header_ptr = (wnet_main_header_st_ptr)buffer_ptr;


    /* Format byte stream to host form for data validation checking. */
    main_header_ptr->f_major_type = cv_ntohs(main_header_ptr->f_major_type);
    main_header_ptr->f_minor_type = cv_ntohs(main_header_ptr->f_minor_type);
    
    /* Analyse frame data based on frame type. */
    switch(main_header_ptr->f_major_type)
    {
        case WNET_F_MAJOR_TYPE_KEEPALIVE:     {  break;  }
        case WNET_F_MAJOR_TYPE_CONFIG_ASK:    {  break;  }
        case WNET_F_MAJOR_TYPE_EVENT_REPORT:  {  break;  }
        case WNET_F_MAJOR_TYPE_RAW_DATA:      
        {  
            wnet_receive_enet((uint8_t *)main_header_ptr + WNET_MAIN_HEADER_ST_LEN,  \
                              main_header_ptr->length - sizeof(main_header_ptr->f_major_type) - sizeof(main_header_ptr->f_minor_type));
            break;  
        }
        default:                              {  break;  }
    }

    /* Receive the next frame. */
    wnet_receive_main(NULL);
}


/**
  * @brief  frame header's length domain receive routine.
  * @param  See below.
  * @retval OK.
  */
static void wnet_length_rxisr
(
    /* Pointer to data buffer. */
    void *buffer_ptr
)
{
    wnet_main_header_st_ptr main_header_ptr = (wnet_main_header_st_ptr)buffer_ptr;
    drv_wnet_isr_register_st      rx_tc_isr = { wnet_data_body_rxisr, buffer_ptr };

    
    /* Format byte stream to host form for data validation checking. */
    main_header_ptr->length = cv_ntohs(main_header_ptr->length);

    /* Read the rest data only when length data valid. */
    if(main_header_ptr->length < (WNET_RX_BUFFER_SIZE - sizeof(main_header_ptr->magic_num1) - sizeof(main_header_ptr->magic_num2) - sizeof(main_header_ptr->length)))
    {
        /* Active the frame body read operation. */
        DRV_WNET_PTR->ioctl(DRV_WNET_PTR, WNET_IOCTL_REGISTER_RXTC_ISR, &rx_tc_isr);
        DRV_WNET_PTR->read(DRV_WNET_PTR, 0, (uint8_t *)buffer_ptr + sizeof(main_header_ptr->magic_num1) \
                                          + sizeof(main_header_ptr->magic_num2) + sizeof(main_header_ptr->length), main_header_ptr->length);
    }
    else
    {
        /* Receive the next frame. */
        wnet_receive_main(NULL);
    }
}


/**
  * @brief  frame header's magic num2 domain receive routine.
  * @param  See below.
  * @retval OK.
  */
static void wnet_magic_num2_rxisr
(
    /* Pointer to data buffer. */
    void *buffer_ptr
)
{
    wnet_main_header_st_ptr main_header_ptr = (wnet_main_header_st_ptr)buffer_ptr;
    drv_wnet_isr_register_st      rx_tc_isr = { wnet_length_rxisr, buffer_ptr };

    
    if(main_header_ptr->magic_num2 == WNET_MAIN_HEADER_MAGIC_NUM2)
    {
        /* Active the frame body read operation. */
        DRV_WNET_PTR->ioctl(DRV_WNET_PTR, WNET_IOCTL_REGISTER_RXTC_ISR, &rx_tc_isr);
        DRV_WNET_PTR->read(DRV_WNET_PTR, 0, (uint8_t *)buffer_ptr + sizeof(main_header_ptr->magic_num1) + sizeof(main_header_ptr->magic_num2), sizeof(main_header_ptr->length));   
    }
    else
    {
        /* Receive the next frame. */
        wnet_receive_main(NULL);
    }
}


/**
  * @brief  frame header's magic num1 domain receive routine.
  * @param  See below.
  * @retval OK.
  */
static void wnet_magic_num1_rxisr
(
    /* Pointer to data buffer. */
    void *buffer_ptr
)
{
    wnet_main_header_st_ptr main_header_ptr = (wnet_main_header_st_ptr)buffer_ptr;
    drv_wnet_isr_register_st      rx_tc_isr = { wnet_magic_num2_rxisr, buffer_ptr };

    
    if(main_header_ptr->magic_num1 == WNET_MAIN_HEADER_MAGIC_NUM1)
    {
        /* Active the frame body read operation. */
        DRV_WNET_PTR->ioctl(DRV_WNET_PTR, WNET_IOCTL_REGISTER_RXTC_ISR, &rx_tc_isr);
        DRV_WNET_PTR->read(DRV_WNET_PTR, 0, (uint8_t *)buffer_ptr + sizeof(main_header_ptr->magic_num1), sizeof(main_header_ptr->magic_num2));   
    }
    else
    {
        /* Receive the next frame. */
        wnet_receive_main(NULL);
    }
}


/**
  * @brief  wireless network receive main routine.
  * @param  See below.
  * @retval OK.
  */
void wnet_receive_main
(
    /* Dummy data pointer only for matching format. */
    void *dummy_ptr
)
{
    wnet_main_header_st_ptr main_header_ptr = (wnet_main_header_st_ptr)dummy_ptr;
    drv_wnet_isr_register_st      rx_tc_isr = { wnet_magic_num1_rxisr, WnetRxBuffer };


    /* Caution: We need not clear the buffer manualy,because the frame data has length domain. */
    
    /* Active the frame header read operation. */
    DRV_WNET_PTR->ioctl(DRV_WNET_PTR, WNET_IOCTL_REGISTER_RXTC_ISR, &rx_tc_isr);
    DRV_WNET_PTR->read(DRV_WNET_PTR, 0, WnetRxBuffer, sizeof(main_header_ptr->magic_num1));    
}

#endif








