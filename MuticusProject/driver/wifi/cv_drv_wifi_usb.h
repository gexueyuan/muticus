/*-------------------------------------------------------------------------
Copyright 2007-2010 CEC Huada Electronic  Design Co., Ltd.
This file is part of Huada Wireless Lan software.
File name:    usb_noswifi.h
Author:        yujun@hed.com.cn
Version:        V1.0.0
Date:        2011-3-17
Description:    
Others:        
  
Revision History:    
Who         When          What
--------    ----------    ----------------------------------------------
yujun          2011-3-17      Create the file.  
-------------------------------------------------------------------------*/
#ifndef __USB_NOSWIFI_H_100756__
#define __USB_NOSWIFI_H_100756__


#define USB_VENDOR_CLASS      0xff
#define USB_VENDOR_REQUEST_OUT       0x40
#define USB_VENDOR_REQUEST_IN        0xc0

#define NOSWIFI_USB_CTRL_TIMEOUT    2000

#define NOSWIFI_CTRL_URB_RETRY_TIMES    (3)
#define NOSWIFI_TX_URB_RETRY_TIMES    (3)
#define NOSWIFI_RX_URB_RETRY_TIMES    (3)

/*
    interface of informing event to system, should be realized by user
*/
#define NOSWIFI_CARD_PLUGIN      0
#define NOSWIFI_CARD_INITED      1
#define NOSWIFI_CARD_PLUGOUT      2
#define NOSWIFI_CARD_ERROR    3

#define CTRL_DATA_STAGE_TIMEOUT      5000 /* 5s */
#define CTRL_NODATA_STAGE_TIMEOUT      50 /* 50ms */
#define BULK_DATA_STAGE_TIMEOUT      5000 /* 5s */

#include "usbh_core.h"

typedef enum _USB_BULK_STATE {
    USB_BULK_STATE_IDLE =0,
    USB_BULK_STATE_REQUEST,
    USB_BULK_STATE_REQUEST_WAIT,
    USB_BULK_STATE_ERROR
} E_USB_BULK_STATE;

typedef struct _wifi_usb_bulkout_request {
#define  NOSWIFI_USB_TX_BUFFER_MAX_LEN    (2*1024)    
    E_USB_BULK_STATE state;
    //void * buffer;
    unsigned char buffer[NOSWIFI_USB_TX_BUFFER_MAX_LEN];
    unsigned int len;
    void * packet_buffer;
    unsigned int packet_len;
    unsigned char retry;
    unsigned short timer;
    URB_STATE status;
} wifi_usb_bulkout_request_t;

typedef struct _wifi_usb_bulkin_request {
#define  NOSWIFI_USB_RX_BUFFER_MAX_LEN    (2*1024)
    E_USB_BULK_STATE state;
    unsigned int rx_buffer_nr;
    unsigned char buffer[NOSWIFI_USB_RX_BUFFER_MAX_LEN];
    unsigned int len;
    void * packet_buffer;
    unsigned int packet_len;
    unsigned char retry;
    unsigned short timer;
    URB_STATE status;
} wifi_usb_bulkin_request_t;

typedef struct _wifi_usb_ctrl_request {
#define  NOSWIFI_USB_CTRL_BUFFER_MAX_LEN    (64)
    unsigned char hc_num_in; 
    unsigned char hc_num_out; 
    unsigned char ep0size; 
    CTRL_State state;
    
    USB_Setup_TypeDef ctrlreq;
    unsigned char buffer[NOSWIFI_USB_CTRL_BUFFER_MAX_LEN];
    unsigned int length;
    void * user_buffer;
    unsigned int user_len;
    unsigned char retry;
    unsigned short timer;
    URB_STATE status;

    int  waiting_for_complete;
} wifi_usb_ctrl_request_t;

typedef struct _wifi_usb_adapter {
    unsigned char device_matched;

    /* device infomation */
    unsigned char bulk_out_ep;
    unsigned char bulk_in_ep;
    unsigned char bulk_out_ep_size;
    unsigned char bulk_in_ep_size;

    wifi_usb_ctrl_request_t     usb_ctrl_req;
    wifi_usb_bulkout_request_t  usb_tx_req;
    wifi_usb_bulkin_request_t   usb_rx_req;
    unsigned char hc_num_in; 
    unsigned char hc_num_out; 

    void *pAdapter;

} wifi_usb_adapter_t;

extern wifi_usb_adapter_t wifi_usb_adapter;

int usb_control_msg(void *hostspecific, unsigned int pipe, u8 request, u8 requesttype, u16 value,
    u16 index, void *data, u16 size, int timeout);
int usb_bulkout (void *HostSpecific, void *pData, unsigned int Length);
int usb_bulkin(void *HostSpecific);


#endif /* __USB_NOSWIFI_H_100756__ */
 
