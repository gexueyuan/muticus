/**
  ******************************************************************************
  * @file    usbh_usr.c
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    22-July-2011
  * @brief   This file includes the usb host library user callbacks
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cv_osal.h"

#include "usb_bsp.h"
#include "usb_conf.h"
#include "usbh_core.h"
#include "usbh_stdreq.h"
#include "usb_bsp.h"
#include "usbh_ioreq.h"
#include "usbh_hcs.h"

#include "cv_drv_wifi_def.h"
#include "cv_drv_wifi_usb.h"

#include "chips\ralink\include\rt_include.h"

/** @defgroup USBH_USR_Private_Variables
* @{
*/ 

/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */

/**
* @}
*/

/** @defgroup USBH_USR_Private_Constants
* @{
*/ 
/*--------------- Console Messages ---------------*/
const uint8_t MSG_HOST_INIT[] = "Host Library Initialized\n";
const uint8_t MSG_DEV_ATTACHED[]  = "Device Attached \n";
const uint8_t MSG_DEV_RESET[]  = "Device Reset \n";
const uint8_t MSG_DEV_DISCONNECTED[] = "Device Disconnected\n";
const uint8_t MSG_DEV_ADDR_ATTACHED[]  = "Device Address Attached \n";
const uint8_t MSG_DEV_ENUMERATED[] = "Enumeration completed \n";
const uint8_t MSG_DEV_HIGHSPEED[] = "High speed device detected\n";
const uint8_t MSG_DEV_FULLSPEED[] = "Full speed device detected\n";
const uint8_t MSG_DEV_LOWSPEED[] = "Low speed device detected\n";
const uint8_t MSG_DEV_ERROR[] = "Device fault \n";

const uint8_t MSG_UNREC_ERROR[] = "UNRECOVERED ERROR STATE\n";

/**
* @}
*/ 


/** @defgroup USBH_USR_Private_Functions
* @{
*/ 


/**
* @brief  USBH_USR_Init 
*         Displays the message on Console for host lib initialization
* @param  None
* @retval None
*/
void USBH_USR_Init(void)
{
    static uint8_t startup = 0;  
  
    if (startup == 0) {
        startup = 1;
        DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_HOST_INIT); 
    }
}

/**
* @brief  USBH_USR_DeviceAttached 
*         Displays the message on Console on device attached
* @param  None
* @retval None
*/
void USBH_USR_DeviceAttached(void)
{
     DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_ATTACHED);
}


/**
* @brief  USBH_USR_UnrecoveredError
* @param  None
* @retval None
*/
void USBH_USR_UnrecoveredError (void)
{
    DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_UNREC_ERROR); 
}


/**
* @brief  USBH_DisconnectEvent
*         Device disconnect event
* @param  None
* @retval Staus
*/
void USBH_USR_DeviceDisconnected (void)
{
    DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_DISCONNECTED);
}
/**
* @brief  USBH_USR_ResetUSBDevice 
* @param  None
* @retval None
*/
void USBH_USR_ResetDevice(void)
{
    /* callback for USB-Reset */
   DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_RESET);
}


/**
* @brief  USBH_USR_DeviceSpeedDetected 
*         Displays the message on Console for device speed
* @param  Device speed
* @retval None
*/
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed)
{
    if (DeviceSpeed == HPRT0_PRTSPD_HIGH_SPEED) {
        DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_HIGHSPEED);
    }  
    else if (DeviceSpeed == HPRT0_PRTSPD_FULL_SPEED) {
        DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_FULLSPEED);
    }
    else if (DeviceSpeed == HPRT0_PRTSPD_LOW_SPEED) {
        DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_LOWSPEED);
    }
    else {
        DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_ERROR);
    }
}

/**
* @brief  USBH_USR_Device_DescAvailable 
*         Displays the message on Console for device descriptor
* @param  device descriptor
* @retval None
*/
void USBH_USR_Device_DescAvailable(void *DeviceDesc)
{ 
    int i;
    USB_DEVICE_ID *pDevID;
    USBH_DevDesc_TypeDef *hs;
    hs = DeviceDesc;  

    DBGPRINT(WHED_DEBUG_INFO, "VID : %04Xh\n" , (uint32_t)(*hs).idVendor); 
    DBGPRINT(WHED_DEBUG_INFO, "PID : %04Xh\n" , (uint32_t)(*hs).idProduct);

    pDevID = &rtusb_dev_id[0];
    for (i=0;i<rtusb_usb_id_len;i++) {
        if ((hs->idProduct == pDevID->idProduct)&&(hs->idVendor == pDevID->idVendor)) {
            wifi_usb_adapter.device_matched = TRUE;
            DBGPRINT(WHED_DEBUG_INFO, "Driver is matched\n");
            break;
        }
        pDevID++;
    }
}

/**
* @brief  USBH_USR_DeviceAddressAssigned 
*         USB device is successfully assigned the Address 
* @param  None
* @retval None
*/
void USBH_USR_DeviceAddressAssigned(void)
{
    DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_ADDR_ATTACHED);
}


/**
* @brief  USBH_USR_Conf_Desc 
*         Displays the message on Console for configuration descriptor
* @param  Configuration descriptor
* @retval None
*/
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc)
{
    wifi_usb_adapter_t *adapter = (wifi_usb_adapter_t *)&wifi_usb_adapter;
    /*
        RT3070 endpoint configurations are as follow:

        EP0: bulk in
        EP1~EP4: bulk out (EDCA)
        EP5:bulk out (HCCA)
        EP6: bulk out (Highest prority)
    */
    adapter->bulk_in_ep = epDesc[0].bEndpointAddress;
    adapter->bulk_in_ep_size = epDesc[0].wMaxPacketSize;
    DBGPRINT(WHED_DEBUG_INFO, "BULK In : EP-%d, Size-%d\n", \
                adapter->bulk_in_ep, adapter->bulk_in_ep_size);

    /* NOTICE: here we use EP1 */
    #define BULKIN_EP_NUM 1
    adapter->bulk_out_ep = epDesc[BULKIN_EP_NUM].bEndpointAddress;
    adapter->bulk_out_ep_size = epDesc[BULKIN_EP_NUM].wMaxPacketSize;
    DBGPRINT(WHED_DEBUG_INFO, "BULK Out : EP-%d, Size-%d\n", \
                adapter->bulk_out_ep, adapter->bulk_out_ep_size);
}

/**
* @brief  USBH_USR_Manufacturer_String 
*         Displays the message on Console for Manufacturer String 
* @param  Manufacturer String 
* @retval None
*/
void USBH_USR_Manufacturer_String(void *ManufacturerString)
{
    DBGPRINT(WHED_DEBUG_INFO, "Manufacturer : %s\n", (char *)ManufacturerString);
}

/**
* @brief  USBH_USR_Product_String 
*         Displays the message on Console for Product String
* @param  Product String
* @retval None
*/
void USBH_USR_Product_String(void *ProductString)
{
    DBGPRINT(WHED_DEBUG_INFO, "Product : %s\n", (char *)ProductString);  
}

/**
* @brief  USBH_USR_SerialNum_String 
*         Displays the message on Console for SerialNum_String 
* @param  SerialNum_String 
* @retval None
*/
void USBH_USR_SerialNum_String(void *SerialNumString)
{
    DBGPRINT(WHED_DEBUG_INFO, "Serial Number : %s\n", (char *)SerialNumString);    
} 



/**
* @brief  EnumerationDone 
*         User response request is displayed to ask application jump to class
* @param  None
* @retval None
*/
void USBH_USR_EnumerationDone(void)
{
    /* Enumeration complete */
    DBGPRINT(WHED_DEBUG_INFO, "%s", (char *)MSG_DEV_ENUMERATED);
} 


/**
* @brief  USBH_USR_DeviceNotSupported
*         Device is not supported
* @param  None
* @retval None
*/
void USBH_USR_DeviceNotSupported(void)
{
    DBGPRINT(WHED_DEBUG_INFO, "Device not supported."); 
}  


/**
* @brief  USBH_USR_UserInput
*         User Action for application state entry
* @param  None
* @retval USBH_USR_Status : User response for key button
*/
USBH_USR_Status USBH_USR_UserInput(void)
{
    USBH_USR_Status usbh_usr_status;

    usbh_usr_status = USBH_USR_RESP_OK;  

    return usbh_usr_status;
}  

/**
* @brief  USBH_USR_OverCurrentDetected
*         Over Current Detected on VBUS
* @param  None
* @retval Staus
*/
void USBH_USR_OverCurrentDetected (void)
{
    DBGPRINT(WHED_DEBUG_INFO, "Overcurrent detected.");
}

/**
* @brief  USBH_USR_DeInit
*         Deint User state and associated variables
* @param  None
* @retval None
*/
void USBH_USR_DeInit(void)
{
}

/**
* @}
*/ 

USBH_Usr_cb_TypeDef USBH_USR_Cb =
{
    USBH_USR_Init,
    USBH_USR_DeInit,
    USBH_USR_DeviceAttached,
    USBH_USR_ResetDevice,
    USBH_USR_DeviceDisconnected,
    USBH_USR_OverCurrentDetected,
    USBH_USR_DeviceSpeedDetected,
    USBH_USR_Device_DescAvailable,
    USBH_USR_DeviceAddressAssigned,
    USBH_USR_Configuration_DescAvailable,
    USBH_USR_Manufacturer_String,
    USBH_USR_Product_String,
    USBH_USR_SerialNum_String,
    USBH_USR_EnumerationDone,
    USBH_USR_UserInput,
    0,
    USBH_USR_DeviceNotSupported,
    USBH_USR_UnrecoveredError
};

