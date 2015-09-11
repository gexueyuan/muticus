/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_drv_wifi_usb.c
 @brief  : this file realizes the base functions of usb host porting
 @author : wangyifeng
 @history:
           2014-10-27    wangyifeng    Created file
           ...
******************************************************************************/
#include "cv_osal.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"

#include "usb_bsp.h"
#include "usb_conf.h"
#include "usbh_core.h"
#include "usbh_stdreq.h"
#include "usb_bsp.h"
#include "usbh_ioreq.h"
#include "usbh_hcs.h"

#include "cv_drv_wifi_def.h"
#include "cv_drv_wifi_usb.h"

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
extern void RTUSBBulkReceive(void *pAd, unsigned char *pData, int Length);
extern void RTUSBBulkSendDone(void *pAd);

extern USBH_Usr_cb_TypeDef USBH_USR_Cb;
void rt2870_probe(void *pUsb_Dev, void *ppAd);

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE           USB_OTG_Core_dev __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USBH_HOST                     USB_Host __ALIGN_END ;

osal_sem_t *USB_Host_Semphore;
osal_sem_t *USB_CtrlReq_Semphore;

wifi_usb_adapter_t wifi_usb_adapter;

/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/

static void _usb_bulkout_complete(wifi_usb_adapter_t *adapter)
{    
    RTUSBBulkSendDone(adapter->pAdapter);
}

static void _usb_control_complete(wifi_usb_adapter_t *adapter)
{
    wifi_usb_ctrl_request_t *usb_ctrl_req = &adapter->usb_ctrl_req;

    if (usb_ctrl_req->ctrlreq.b.bmRequestType == USB_VENDOR_REQUEST_IN) {
        memcpy(usb_ctrl_req->user_buffer, usb_ctrl_req->buffer, usb_ctrl_req->user_len);
    }

    if (usb_ctrl_req->waiting_for_complete) {
        usb_ctrl_req->waiting_for_complete = 0;
        osal_sem_release(USB_CtrlReq_Semphore);
    }
}

static void _usb_bulkin_complete(wifi_usb_adapter_t *adapter)
{
    wifi_usb_bulkin_request_t *usb_rx_req = &adapter->usb_rx_req;

    RTUSBBulkReceive(adapter->pAdapter, usb_rx_req->buffer, usb_rx_req->len);
}

static void _dev_remove(USB_OTG_CORE_HANDLE *pdev, void *phost)
{    
    return;
}

static USBH_Status _dev_request(USB_OTG_CORE_HANDLE *pdev , void *phost)
{    
    return USBH_OK; 
}

static int _usb_ctrl_request_handle(USB_OTG_CORE_HANDLE *pdev , 
                                    USBH_HOST *phost, 
                                    wifi_usb_adapter_t *adapter)
{
    wifi_usb_ctrl_request_t *ctl_req = &adapter->usb_ctrl_req;
    unsigned char direction;  
    static unsigned short timeout = 0;
    URB_STATE URB_Status = URB_IDLE;
    int status = 1;

    phost->Control.status = CTRL_START;

    switch (ctl_req->state) {
    case CTRL_IDLE:
        status = 0; /* indicate the state machine could be blocked */
        break;
        
    case CTRL_SETUP:
        /* Send a SETUP packet. */
        pdev->host.hc[ctl_req->hc_num_out].toggle_out = 0; 
        USBH_CtlSendSetup(pdev, ctl_req->ctrlreq.d8, ctl_req->hc_num_out);  
        ctl_req->state = CTRL_SETUP_WAIT;  
        break; 

    case CTRL_SETUP_WAIT:
        URB_Status = HCD_GetURB_State(pdev , ctl_req->hc_num_out); 
        ctl_req->status = URB_Status;
        if (URB_Status == URB_DONE) { 
            direction = (ctl_req->ctrlreq.b.bmRequestType & USB_REQ_DIR_MASK);

            /* check if there is a data stage. */
            if (ctl_req->ctrlreq.b.wLength.w != 0) {        
                timeout = CTRL_DATA_STAGE_TIMEOUT;
                if (direction == USB_D2H) {
                    /* Data Direction is IN. */
                    ctl_req->state = CTRL_DATA_IN;
                }
                else {
                    /* Data Direction is OUT. */
                    ctl_req->state = CTRL_DATA_OUT;

                } 
            }
            /* No DATA stage. */
            else {
                timeout = CTRL_NODATA_STAGE_TIMEOUT;

                /* If there is No Data Transfer Stage. */
                if (direction == USB_D2H) {
                    /* Data Direction is IN. */
                    ctl_req->state = CTRL_STATUS_OUT;

                }
                else {
                    /* Data Direction is OUT. */
                    ctl_req->state = CTRL_STATUS_IN;
                } 
            }
            
            /* Set the delay timer to enable timeout for data stage completion. */
            ctl_req->timer = HCD_GetCurrentFrame(pdev);
        }
        else if (URB_Status == URB_ERROR) {
            ctl_req->state = CTRL_ERROR;  
            /* To be add. */
        }    
        break;

    case CTRL_DATA_IN:  
        /* Issue an IN token. */ 
        USBH_CtlReceiveData(pdev,
                            ctl_req->buffer, 
                            ctl_req->length,
                            ctl_req->hc_num_in);

        ctl_req->state = CTRL_DATA_IN_WAIT;
        break;    

    case CTRL_DATA_IN_WAIT:
        USB_OTG_BSP_uDelay(200);

        URB_Status = HCD_GetURB_State(pdev , ctl_req->hc_num_in); 
        ctl_req->status = URB_Status;

        /* check is DATA packet transfered successfully. */
        if (URB_Status == URB_DONE) { 
            ctl_req->state = CTRL_STATUS_OUT;
        }
        
        /* manage error cases. */
        else if (URB_Status == URB_STALL) { 
            ctl_req->state =  CTRL_ERROR;
        }   
        else if (URB_Status == URB_ERROR) {
            /* Device error. */
            ctl_req->state = CTRL_ERROR;    
        }

        break;

    case CTRL_DATA_OUT:
        /* Start DATA out transfer (only one DATA packet). */
        USB_OTG_BSP_uDelay(200);
        pdev->host.hc[ctl_req->hc_num_out].toggle_out ^= 1; 

        USBH_CtlSendData(pdev,
                        ctl_req->buffer, 
                        ctl_req->length , 
                        ctl_req->hc_num_out);

        ctl_req->state = CTRL_DATA_OUT_WAIT;
        break;

    case CTRL_DATA_OUT_WAIT:
        URB_Status = HCD_GetURB_State(pdev, ctl_req->hc_num_out);
        ctl_req->status = URB_Status;
        
        if (URB_Status == URB_DONE) { 
            /* If the Setup Pkt is sent successful, then change the state. */
            ctl_req->state = CTRL_STATUS_IN;
        }

        /* handle error cases. */
        else if (URB_Status == URB_STALL) { 
            ctl_req->state = CTRL_ERROR;
        } 
        else if (URB_Status == URB_NOTREADY) { 
            /* Nack received from device. */
            ctl_req->state = CTRL_DATA_OUT;
        }    
        else if (URB_Status == URB_ERROR) {
            /* device error */
            ctl_req->state = CTRL_ERROR;   
        } 
        break;


    case CTRL_STATUS_IN:
        /* Send 0 bytes out packet. */
        USB_OTG_BSP_uDelay(200);
        USBH_CtlReceiveData(pdev,
                            0,
                            0,
                            ctl_req->hc_num_in);

        ctl_req->state = CTRL_STATUS_IN_WAIT;

        break;

    case CTRL_STATUS_IN_WAIT:
        URB_Status = HCD_GetURB_State(pdev, ctl_req->hc_num_in); 
        ctl_req->status = URB_Status;

        if (URB_Status == URB_DONE) { 
            /* Control transfers completed, Exit the State Machine */
            ctl_req->state = CTRL_IDLE;
            _usb_control_complete(adapter);
        }

        else if (URB_Status == URB_ERROR) {
            ctl_req->state = CTRL_ERROR;  
        }
        else if (URB_Status == URB_STALL) {
            ctl_req->state = URB_STALL;  /* NOTICE: here maybe a error to be fixed!!! */
        }
        
        break;

    case CTRL_STATUS_OUT:
        USB_OTG_BSP_uDelay(200);
        pdev->host.hc[ctl_req->hc_num_out].toggle_out ^= 1; 
        USBH_CtlSendData(pdev,
                        0,
                        0,
                        ctl_req->hc_num_out);

        ctl_req->state = CTRL_STATUS_OUT_WAIT;
        break;

    case CTRL_STATUS_OUT_WAIT: 
        URB_Status = HCD_GetURB_State(pdev, ctl_req->hc_num_out);
        ctl_req->status = URB_Status;
        
        if (URB_Status == URB_DONE) { 
            ctl_req->state = CTRL_IDLE;
            _usb_control_complete(adapter);
        }
        else if (URB_Status == URB_NOTREADY) { 
            ctl_req->state = CTRL_STATUS_OUT;
        }      
        else if (URB_Status == URB_ERROR) {
            ctl_req->state = CTRL_ERROR; 
        }
        break;

    case CTRL_ERROR:
        DBGPRINT(WHED_DEBUG_ERROR, "PANIC(%s - %d): %s - control urb failed(%d), bRequestType = 0x%x, bRequest = 0x%x, wValue = 0x%x, wIndex = 0x%x, wLength = 0x%x.\n", 
                __FILE__, __LINE__, __FUNCTION__, 
                ctl_req->status,
                ctl_req->ctrlreq.b.bmRequestType,
                ctl_req->ctrlreq.b.bRequest,
                ctl_req->ctrlreq.b.wValue.w,
                ctl_req->ctrlreq.b.wIndex.w,
                ctl_req->ctrlreq.b.wLength.w);
        if (ctl_req->retry) {
            ctl_req->retry--;
            /* Do the transmission again, starting from SETUP Packet. */
            ctl_req->state = CTRL_SETUP; 
        }
        else {
            ctl_req->state = CTRL_IDLE;
            _usb_control_complete(adapter);
        }
        break;

    default:
        break;
    }
    
    timeout = timeout; /* avoid compiler's warning */

    return status;
}

static int _usb_bulkout_request_handle(USB_OTG_CORE_HANDLE *pdev , 
                                       USBH_HOST *phost, 
                                       wifi_usb_adapter_t *adapter)
{
    wifi_usb_bulkout_request_t *tx_req = &adapter->usb_tx_req;
    static unsigned short timeout = 0;
    URB_STATE URB_Status = URB_IDLE;
    int status = 1;

    switch (tx_req->state) {
    case USB_BULK_STATE_IDLE:
        status = 0; /* indicate the state machine could be blocked */
        break;

    case USB_BULK_STATE_REQUEST:
        tx_req->packet_buffer = &tx_req->buffer[0];
        if (tx_req->len > pdev->host.hc[adapter->hc_num_out].max_packet) {
            tx_req->packet_len = pdev->host.hc[adapter->hc_num_out].max_packet;
        }
        else {
            tx_req->packet_len = tx_req->len;
        }
        
        USBH_BulkSendData(pdev, tx_req->packet_buffer, tx_req->packet_len, adapter->hc_num_out);

        tx_req->retry = 3;
        tx_req->state = USB_BULK_STATE_REQUEST_WAIT;
        status = 0; /* indicate the state machine could be blocked */
        timeout = BULK_DATA_STAGE_TIMEOUT;
        /* Set the delay timer to enable timeout for data stage completion. */
        tx_req->timer = HCD_GetCurrentFrame(pdev);
        break;

    case USB_BULK_STATE_REQUEST_WAIT:
        URB_Status = HCD_GetURB_State(pdev , adapter->hc_num_out);
        tx_req->status = URB_Status;
  
        if (URB_Status == URB_DONE) {
            tx_req->len -= tx_req->packet_len;
            tx_req->packet_buffer = (unsigned char *)(tx_req->packet_buffer) + tx_req->packet_len;

            if (tx_req->len == 0) {
                tx_req->state = USB_BULK_STATE_IDLE;
                _usb_bulkout_complete(adapter);
            }
            else {
                if (tx_req->len > pdev->host.hc[adapter->hc_num_out].max_packet) {
                    tx_req->packet_len = pdev->host.hc[adapter->hc_num_out].max_packet;
                }
                else {
                    tx_req->packet_len = tx_req->len;
                }

                USBH_BulkSendData(pdev, tx_req->packet_buffer, tx_req->packet_len, adapter->hc_num_out);

                tx_req->retry = 10;
                tx_req->state = USB_BULK_STATE_REQUEST_WAIT;
                status = 0; /* indicate the state machine could be blocked */
                timeout = BULK_DATA_STAGE_TIMEOUT;
                /* Set the delay timer to enable timeout for data stage completion. */
                tx_req->timer = HCD_GetCurrentFrame(pdev);
            }
        }
        else if (URB_Status == URB_ERROR) {
            tx_req->state = USB_BULK_STATE_ERROR;
        }    
        else if (URB_Status == URB_STALL) {
            tx_req->state = USB_BULK_STATE_ERROR;
        }
        else if (URB_Status == URB_NOTREADY) {
            USBH_BulkSendData(pdev, tx_req->packet_buffer, tx_req->packet_len, adapter->hc_num_out);
            tx_req->state = USB_BULK_STATE_REQUEST_WAIT;
            status = 0; /* indicate the state machine could be blocked */
            timeout = BULK_DATA_STAGE_TIMEOUT;
            /* Set the delay timer to enable timeout for data stage completion. */
            tx_req->timer = HCD_GetCurrentFrame(pdev);
        } 
        break;

    case USB_BULK_STATE_ERROR:
        DBGPRINT(WHED_DEBUG_ERROR, "PANIC(%s - %d): %s - tx urb failed(%d).\n", __FILE__, __LINE__, __FUNCTION__, tx_req->status);
        if (tx_req->retry) {
            tx_req->retry--;
            //USB_OTG_BSP_uDelay(600);
            USBH_BulkSendData(pdev, tx_req->packet_buffer, tx_req->packet_len, adapter->hc_num_out);
            tx_req->state = USB_BULK_STATE_REQUEST_WAIT;
            status = 0; /* indicate the state machine could be blocked */
            timeout = BULK_DATA_STAGE_TIMEOUT;
            /* Set the delay timer to enable timeout for data stage completion. */
            tx_req->timer = HCD_GetCurrentFrame(pdev);
        }
        else {
            tx_req->state = USB_BULK_STATE_REQUEST;
            tx_req->state = USB_BULK_STATE_IDLE;
            _usb_bulkout_complete(adapter);
        }
        break;

    default:
        break;
    }

    timeout = timeout; /* avoid compiler's warning */

    return status;
}

static int _usb_bulkin_request_handle(USB_OTG_CORE_HANDLE * pdev , 
                                      USBH_HOST * phost, 
                                      wifi_usb_adapter_t *adapter)
{
    wifi_usb_bulkin_request_t *rx_req = &adapter->usb_rx_req;
    static unsigned short timeout = 0;
    URB_STATE URB_Status = URB_IDLE;
    unsigned int actual_len = 0;
    int status = 1;

    switch (rx_req->state) {
    case USB_BULK_STATE_IDLE:
        status = 0; /* indicate the state machine could be blocked */
        break;

    case USB_BULK_STATE_REQUEST:
        rx_req->packet_buffer = &rx_req->buffer[0];
        rx_req->packet_len = pdev->host.hc[adapter->hc_num_in].max_packet;
        
        USBH_BulkReceiveData(pdev, rx_req->packet_buffer, rx_req->packet_len, adapter->hc_num_in);

        rx_req->retry = 3;
        rx_req->state = USB_BULK_STATE_REQUEST_WAIT;
        status = 0; /* indicate the state machine could be blocked */
        timeout = BULK_DATA_STAGE_TIMEOUT;
        /* Set the delay timer to enable timeout for data stage completion. */
        rx_req->timer = HCD_GetCurrentFrame(pdev);
        break;

    case USB_BULK_STATE_REQUEST_WAIT:
        URB_Status = HCD_GetURB_State(pdev , adapter->hc_num_in);
        rx_req->status = URB_Status;
  
        if (URB_Status == URB_DONE) { 
            rx_req->state = USB_BULK_STATE_IDLE;
            actual_len = pdev->host.hc[adapter->hc_num_in].xfer_count;
            rx_req->len += actual_len;
            rx_req->packet_buffer = (unsigned char *)(rx_req->packet_buffer) + actual_len;

            if (actual_len < pdev->host.hc[adapter->hc_num_in].max_packet) {
                _usb_bulkin_complete(adapter);
            }
            else {
                if ((rx_req->len + pdev->host.hc[adapter->hc_num_in].max_packet) <= NOSWIFI_USB_RX_BUFFER_MAX_LEN) {
                    rx_req->packet_len = pdev->host.hc[adapter->hc_num_in].max_packet;
                    USBH_BulkReceiveData(pdev, rx_req->packet_buffer, rx_req->packet_len, adapter->hc_num_in);

                    rx_req->retry = 3;
                    rx_req->state = USB_BULK_STATE_REQUEST_WAIT;
                    status = 0; /* indicate the state machine could be blocked */
                    timeout = BULK_DATA_STAGE_TIMEOUT;
                    /* Set the delay timer to enable timeout for data stage completion. */
                    rx_req->timer = HCD_GetCurrentFrame(pdev);
                }
                else {
                    rx_req->len = 0;
                    _usb_bulkin_complete(adapter);
                }
            }
        }
        else if (URB_Status == URB_ERROR) {
            rx_req->state = USB_BULK_STATE_ERROR;
        }    
        else if (URB_Status == URB_STALL) {
            rx_req->state = USB_BULK_STATE_ERROR;
        }
        break;

    case USB_BULK_STATE_ERROR:
        DBGPRINT(WHED_DEBUG_ERROR, "PANIC(%s - %d): %s - rx urb failed(%d).\n", __FILE__, __LINE__, __FUNCTION__, rx_req->status);
        if (rx_req->retry) {
            USB_OTG_BSP_uDelay(600);
            rx_req->retry--;
            rx_req->state = USB_BULK_STATE_REQUEST; 
        }
        else {
            rx_req->state = USB_BULK_STATE_REQUEST;
            rx_req->state = USB_BULK_STATE_IDLE;
            rx_req->len = 0;
            _usb_bulkin_complete(adapter);
        }
        break;

    default:
        break;
    }

    timeout = timeout; /* avoid compiler's warning */

    return status;
}

/*   _dev_machine   */
/*-------------------------------------------------------------------------
    Description:    
        NosWLAN netcard adapter driver state machine handler.
    Arguments:
        pdev : Selected device.
        hhost: Selected device property.
    Return Value:
        USBH_Status.
    Note:    

-------------------------------------------------------------------------*/
static USBH_Status _dev_machine(USB_OTG_CORE_HANDLE * pdev , void * phost)
{
    wifi_usb_adapter_t *adapter = (wifi_usb_adapter_t *)&wifi_usb_adapter;
    int immediate_handle = 0;
    
    immediate_handle |= _usb_ctrl_request_handle(pdev, phost, adapter);
    immediate_handle |= _usb_bulkin_request_handle(pdev, phost, adapter);
    immediate_handle |= _usb_bulkout_request_handle(pdev, phost, adapter);

    if (!immediate_handle) {
        immediate_handle = RT_WAITING_FOREVER;
    }

    osal_sem_take(USB_Host_Semphore, immediate_handle);

    return USBH_OK;
}

static void _dev_probe_entry(void *parameter)
{
    wifi_usb_adapter_t *adapter = (wifi_usb_adapter_t *)&wifi_usb_adapter;
    USB_OTG_CORE_HANDLE *pdev = &USB_OTG_Core_dev; 
    USBH_HOST *pphost = &USB_Host;

    DBGPRINT(WHED_DEBUG_TRACE, "%s: ---->\n", __FUNCTION__);

    DBGPRINT(WHED_DEBUG_TRACE, "VID:%x, PID:%x.\n",\
     pphost->device_prop.Dev_Desc.idVendor, pphost->device_prop.Dev_Desc.idProduct);

    if (!adapter->device_matched) {
        DBGPRINT(WHED_DEBUG_ERROR, "Device is not supported.\n");
        return;
    }

    /* Control channel */
    adapter->usb_ctrl_req.ep0size = pphost->Control.ep0size; /* no use */
    adapter->usb_ctrl_req.hc_num_in = pphost->Control.hc_num_in;
    adapter->usb_ctrl_req.hc_num_out = pphost->Control.hc_num_out;
    DBGPRINT(WHED_DEBUG_TRACE, "Control in pipe - 0x%x\n", adapter->usb_ctrl_req.hc_num_in);
    DBGPRINT(WHED_DEBUG_TRACE, "Control out pipe - 0x%x\n", adapter->usb_ctrl_req.hc_num_out);

    /* Data IN channel */
    adapter->hc_num_in = USBH_Alloc_Channel(pdev, adapter->bulk_in_ep);  
    USBH_Open_Channel(pdev,
                        adapter->hc_num_in,
                        pphost->device_prop.address,
                        pphost->device_prop.speed,
                        EP_TYPE_BULK,
                        adapter->bulk_in_ep_size);   
    DBGPRINT(WHED_DEBUG_TRACE, "Bulk in pipe - 0x%x\n", adapter->hc_num_in);

    /* Data OUT channel */
    adapter->hc_num_out = USBH_Alloc_Channel(pdev, adapter->bulk_out_ep);
    USBH_Open_Channel(pdev,
                        adapter->hc_num_out,
                        pphost->device_prop.address,
                        pphost->device_prop.speed,
                        EP_TYPE_BULK,
                        adapter->bulk_out_ep_size);  
    DBGPRINT(WHED_DEBUG_TRACE, "Bulk out pipe - 0x%x\n", adapter->hc_num_out);

    /*
        Initialize the usb request state machine.
    */
    adapter->usb_ctrl_req.state = CTRL_IDLE;
    adapter->usb_tx_req.state = USB_BULK_STATE_IDLE;
    adapter->usb_rx_req.state = USB_BULK_STATE_IDLE;

    adapter->usb_ctrl_req.retry = 3;
    adapter->usb_tx_req.retry = 3;
    adapter->usb_rx_req.retry = 3;

    /*
        Initialize the Wifi module,
    */
    rt2870_probe(adapter, &adapter->pAdapter);

    /*
        Inform to the system
    */
    if(p_cms_envar->sys.queue_sys_mng) {
        sys_add_event_queue(&p_cms_envar->sys, SYS_MSG_INITED, 0, 0, 0);
    }
}

static USBH_Status _dev_probe ( USB_OTG_CORE_HANDLE *pdev, void * phost)
{
    osal_task_t *task;
    
    task = osal_task_create("usbm", _dev_probe_entry, NULL, RT_THREAD_STACK_SIZE, \
                            RT_USBEMU_THREAD_PRIORITY);
    osal_assert(task != NULL);

    return USBH_OK;
}

static void usbh_thread_entry(void *parameter)
{
    rt_err_t err = 0;

    DBGPRINT(WHED_DEBUG_TRACE, "%s: ---->\n", __FUNCTION__);

    err = err;
    
    while (1) {
        /* Host Task handler. */
        USBH_Process(&USB_OTG_Core_dev , &USB_Host);
    }
}

USBH_Class_cb_TypeDef USBH_Class_Cb = 
{
    _dev_probe,
    _dev_remove,
    _dev_request,
    _dev_machine,
};

void usb_init(void)
{
    osal_sem_t *sem;
    osal_task_t *task;

    memset(&wifi_usb_adapter,0,sizeof(wifi_usb_adapter_t));

    sem = osal_sem_create("usbh", 0);
    osal_assert(sem != NULL);
    USB_Host_Semphore = sem;

    sem = osal_sem_create("usbc", 0);
    osal_assert(sem != NULL);
    USB_CtrlReq_Semphore = sem;

    USBH_Init(&USB_OTG_Core_dev, 
        USB_OTG_FS_CORE_ID,
        &USB_Host,
        &USBH_Class_Cb, 
        &USBH_USR_Cb);

    /* Finally start USB host process thread */
    task = osal_task_create("usbh", usbh_thread_entry, NULL, RT_USBH_THREAD_STACK_SIZE, \
                            RT_USBH_THREAD_PRIORITY);
    osal_assert(task != NULL);
}

int usb_control_msg(void *hostspecific, unsigned int pipe, u8 request, u8 requesttype, u16 value,
    u16 index, void *data, u16 size, int timeout)
{
    wifi_usb_adapter_t *adapter = (wifi_usb_adapter_t *)hostspecific;
    wifi_usb_ctrl_request_t *ctl_req = &adapter->usb_ctrl_req;
    int ret;

    ctl_req->ctrlreq.b.bmRequestType = requesttype;
    ctl_req->ctrlreq.b.bRequest = request;
    ctl_req->ctrlreq.b.wValue.w = value;
    ctl_req->ctrlreq.b.wIndex.w = index;
    ctl_req->ctrlreq.b.wLength.w = size;

    ctl_req->length = size;

    if (requesttype == USB_VENDOR_REQUEST_IN) {
        ctl_req->user_buffer = data;
        ctl_req->user_len = size;
        memset(ctl_req->buffer, 0, 64);
    }
    else {
        ctl_req->user_buffer = data;
        memcpy(ctl_req->buffer, data, size);
    }

    ctl_req->state = CTRL_SETUP;

    ctl_req->waiting_for_complete = 1;

    osal_sem_release(USB_Host_Semphore);
    
    ret = osal_sem_take(USB_CtrlReq_Semphore, timeout);
    ctl_req->waiting_for_complete = 0;

    return ret;
}

int usb_bulkout (void *HostSpecific, void *pData, unsigned int Length)
{
    wifi_usb_adapter_t *adapter = (wifi_usb_adapter_t *)HostSpecific;
    wifi_usb_bulkout_request_t *usb_tx_req;

    usb_tx_req = &adapter->usb_tx_req;

    if (Length > 2048) {
        Length= 2048;
    }

    memcpy(usb_tx_req->buffer, pData, Length);
    usb_tx_req->len = Length;
    
    usb_tx_req->state = USB_BULK_STATE_REQUEST;
    osal_sem_release(USB_Host_Semphore);

    return 0;
}

int usb_bulkin(void *HostSpecific)
{
    wifi_usb_adapter_t *adapter = (wifi_usb_adapter_t *)HostSpecific;
    wifi_usb_bulkin_request_t *usb_rx_req = &adapter->usb_rx_req;

    usb_rx_req->len = 0;
    usb_rx_req->state = USB_BULK_STATE_REQUEST;

    return 0;
}

