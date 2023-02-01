#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include "log.h"
#include "rom_sym_def.h"
#include "usb_usr_config.h"
#include "usb_hal_pcd.h"
#include "usb_pcd.h"
#include "hid.h"

#if( _DEF_USB_INFO_ == USB_DEV_MS)

struct ustring_descriptor str_desc;
//uint8_t report[64];

uint8_t hid_mouse_ready = 0;

uint8_t hid_remote_wakeup_rdy = HID_REMOTE_WAKEUP_IDLE;


uint8_t mouse_protocol = 1;

uint8_t mouse_idle_rate = 0;


ALIGN(4)
#if 0
const static char* hid_ustring[] = 
{
	"Language",
	"SLI Micro System",
	"USB Receiver",
	"20210800",
	"HID Configuration",
	"HID Interface",
};
#endif
const static char* hid_ustring[] = 
{
	"Language",
	"USB Receiver",
	"USB Receiver",
	"20210800",
	"HID Configuration",
	"HID Interface",
};


ALIGN(4)
const struct udevice_descriptor dev_desc =
{
    USB_DESC_LENGTH_DEVICE,     //bLength;
    USB_DESC_TYPE_DEVICE,       //type;
    USB_BCD_VERSION,            //bcdUSB;
    USB_CLASS_DEVICE,           //bDeviceClass;
    0x00,                       //bDeviceSubClass;
    0x00,                       //bDeviceProtocol;
    HID_MAX_PACKET_SIZE,        //bMaxPacketSize0;
    _VENDOR_ID,                 //idVendor;
    _PRODUCT_ID,                //idProduct;
    USB_BCD_DEVICE,             //bcdDevice;
    USB_STRING_MANU_INDEX,      //iManufacturer;
    USB_STRING_PRODUCT_INDEX,   //iProduct;
    0,//USB_STRING_SERIAL_INDEX,    //iSerialNumber;
    1,                          //bNumConfigurations;
};

ALIGN(4)
const struct usb_qualifier_descriptor dev_qualifier_desc = {
	USB_DESC_LENGTH_QUALIFIER,
	USB_DESC_TYPE_DEVICEQUALIFIER,
	USB_BCD_VERSION,
	USB_CLASS_DEVICE,
	0x0,
	0x0,
	HID_MAX_PACKET_SIZE,
	0x1,
	0x0,
};




/*Report Descriptor 
 * 
 * 
 * BIT[7:4] : bTag
 *  Main Item Tag:
 *     1000 : Input
 *     1001 : Output
 *     1010 : Collection
 *     1011 : Feature
 *     1100 : End Collection
 * 
 *  Global Item Tag:
 *     0000 : Usage Page
 *     0001 : Logical Minimum
 *     0010 : Logical Maximum
 *     0011 : Physical Minimum
 *     0100 : Physical Maximum
 *     0101 : Unit Exponent
 *     0110 : Unit
 *     0111 : Report Size
 *     1000 : Report ID
 *     1001 : Report Count
 *     1010 : Push
 *     1011 : POP
 *  
 *  Local Item Tag:  
 *     0000 : Usage
 *     0001 : Usage Minimum
 *     0010 : Usage Maximum   
 *     0011 : Designator Index
 *     0100 : Designator Minimum
 *     0101 : Designator Maximum
 *     0111 : String Index
 *     1000 : String Minimum
 *     1001 : String Maimum
 *     1010 : Delimiter
 *  
 * BIT[3:2] : bType
 *         0: Main
 *         1: Global
 *         2: Local
 *         3: Reserved
 * 
 * BIT[1:0] : bSize
 *         0: 0 Bytes
 *         1: 1 Bytes
 *         2: 2 Bytes
 *         3: 3 Bytes
 *
 * */

#if 0

ALIGN(4) const uint8_t hid_mouse_report_desc[] ={

0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
0x09, 0x02,        // Usage (Mouse)
0xA1, 0x01,        // Collection (Application)
//0x85, 0x02,         /*      Report ID (0x2)                                                    */
0x09, 0x01,        //   Usage (Pointer)
0xA1, 0x00,        //   Collection (Physical)
0x05, 0x09,        //     Usage Page (Button)
0x19, 0x01,        //     Usage Minimum (0x01)
0x29, 0x05,        //     Usage Maximum (0x05)
0x15, 0x00,        //     Logical Minimum (0)
0x25, 0x01,        //     Logical Maximum (1)
0x95, 0x05,        //     Report Count (5)
0x75, 0x01,        //     Report Size (1)
0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x95, 0x01,        //     Report Count (1)
0x75, 0x03,        //     Report Size (3)
0x81, 0x01,        //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
0x09, 0x30,        //     Usage (X)
0x09, 0x31,        //     Usage (Y)
0x16, 0x00, 0xF8,  //     Logical Minimum (-2048)
0x26, 0xFF, 0x07,  //     Logical Maximum (2047)
0x75, 0x0C,        //     Report Size (12)
0x95, 0x02,        //     Report Count (2)
0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
0x09, 0x38,        //     Usage (Wheel)
0x15, 0x81,        //     Logical Minimum (-127)
0x25, 0x7F,        //     Logical Maximum (127)
0x75, 0x08,        //     Report Size (8)
0x95, 0x01,        //     Report Count (1)
0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
0xC0,              //   End Collection
0xC0,              // End Collection

};

#else

ALIGN(4) const uint8_t hid_mouse_report_desc[] ={
	0x05, 0x01,         /* Usage Page (Generic Desktop Controls)                                   */
	0x09, 0x02,         /* Usage (Mouse)                                                           */
	0xA1, 0x01,         /* Collection (Application)                                                */
	0x85, 0x02,         /*      Report ID (0x2)                                                    */
	0x09, 0x01,         /*      Usage (Pointer)                                                    */
	0xA1, 0x00,         /*      Collection (Physical)                                              */
	0x05, 0x09,         /*          Usage Page (Button)                                            */
	0x19, 0x01,         /*          Usage Minimum (0x01)                                           */
	0x29, 0x10,         /*          Usage Maximum (0x10)                                           */
	0x15, 0x00,         /*          Logical Minimum (0x00)                                         */
	0x25, 0x01,         /*          Logical Maxmum  (0x01)                                         */
	0x95, 0x10,         /*          Report Count (0x10)                                            */
	0x75, 0x01,         /*          Report Size (0x01)                                             */
	0x81, 0x02,         /*          Input (Data, Absolute, No Wrap, Linear, 
										   Preferred State, No Pull Position Bit Field)            */
    0x05, 0x01,         /*          Usage Page (Generic Desktop Controls)                          */
	0x16, 0x01, 0xf8,   /*          Logical Minimum(0xF801)                                        */
	0x26, 0xFF, 0x07,   /*          Logical Maximum(0x07FF)                                        */
	0x75, 0x0C,         /*          Report Size (0x0C)                                             */
	0x95, 0x02,         /*          Report Count (0x02)                                            */
	0x09, 0x30,         /*          Usage (X)                                                      */
	0x09, 0x31,         /*          Usage (Y)                                                      */
	0x81, 0x06,         /*          Input (Data, Variable, Relative, No Wrap, Linear, 
										   Preferred State, No Pull Position Bit Field)            */
	0x15, 0x81,         /*          Logical Mimimum (0x81)                                         */
	0x25, 0x7F,         /*          Logical Maximum (0x7F)                                         */
	0x75, 0x08,         /*          Report Size (0x08)                                             */
	0x95, 0x01,         /*          Report Count (0x01)                                            */
	0x09, 0x38,         /*          Usage (Wheel)                                                  */
	0x81, 0x06,         /*          Input (Data, Variable, Relative, No Wrap, Linear, 
										   Preferred State, No Pull Position Bit Field)            */
	0x05, 0x0C,         /*          Usage(Consumer)                                                */
	0x0A, 0x38, 0x02,    /*          Usage (AC PAN)                                                 */
	0x95, 0x01,         /*          Report Count(0x01)                                             */
	0x81, 0x06,         /*          Input (Data, Variable, Relative, No Wrap, Linear, 
						                   Preferred State, No Pull Position Bit Field)            */
	0xC0,               /*      End Collection                                                     */
	0xC0,               /* End Collection                                                          */

};

#endif


/* communcation interface descriptor */
ALIGN(4) const struct uhid_config_descriptor config_desc =
{
	/*Configuration Descriptor */
	{
		USB_DESC_LENGTH_CONFIG,
		USB_DESC_TYPE_CONFIGURATION,
		USB_DESC_LENGTH_CONFIG + USB_DESC_LENGTH_INTERFACE + USB_DESC_LENGTH_HID \
		+ USB_DESC_LENGTH_ENDPOINT, 
		0x01,
		0x01,
		0x04,
		0xA0,
		0x31
	},
    /* Interface Descriptor */
    {
        USB_DESC_LENGTH_INTERFACE,
        USB_DESC_TYPE_INTERFACE,
        0x00,
        0x00,   
        0x01,
        USB_CLASS_HID,
        USB_HID_SUBCLASS_BOOT,
        USB_HID_PROTOCOL_MOUSE,
        0x00,
    },
    /* HID Descriptor */   
    {
        USB_DESC_LENGTH_HID,                              
        USB_DESC_TYPE_HID,
        0x0111,
		0x0,
		0x01,
		USB_DESC_TYPE_REPORT,
        //USB_DESC_LENGTH_MOUSE_REPORT,
        sizeof(hid_mouse_report_desc),
    },
    /* Endpoint Descriptor */    
    {
        USB_DESC_LENGTH_ENDPOINT,
        USB_DESC_TYPE_ENDPOINT,
        1 | USB_DIR_IN,
        USB_EP_ATTR_INT,
        0x40,
        0x1,
    }
};

static int32_t get_dev_desc(uint8_t desc_idx, uint8_t str_idx, uint8_t  **buf , uint16_t *len)
{
	switch(desc_idx)
	{
		case USB_DESC_TYPE_DEVICE:
			*buf = (uint8_t *)&dev_desc;
			*len = sizeof(struct udevice_descriptor);
			break;
		case USB_DESC_TYPE_CONFIGURATION:
			*buf = (uint8_t *)&config_desc;
			*len = sizeof(struct uhid_config_descriptor);
			break;
		case USB_DESC_TYPE_STRING:
			*buf = (uint8_t *)hid_ustring[str_idx];
			*len = osal_strlen(hid_ustring[str_idx]);
			break;

		case USB_DESC_TYPE_REPORT:

			*buf = (uint8_t *)hid_mouse_report_desc;
			*len = sizeof(hid_mouse_report_desc);
			break;
        case USB_DESC_TYPE_DEVICEQUALIFIER:
			*buf = (uint8_t *)&dev_qualifier_desc;
			*len = sizeof(dev_qualifier_desc);
            break;
			
		default:
			return -1;
	}
	return 0;
}


int32_t usb_get_desc_req_proc(usb_hal_pcd_t *hpcd, ureq_t req)
{
//    int32_t ret = 0;
    uint8_t index = 0;
    uint8_t *ptr;
    uint16_t i = 0;
    uint16_t len = 0;
	uint8_t desc_idx = (uint8_t)((req->wValue >> 8) & 0xff);
	
    if(desc_idx == USB_DESC_TYPE_DEVICE || desc_idx == USB_DESC_TYPE_CONFIGURATION){
//        LOG_DEBUG("Get Device Desc\n");
        get_dev_desc(desc_idx, 0, (uint8_t **)&hpcd->ep[0].xfer_buf, (uint16_t *)&hpcd->ep[0].xfer_length);

        if (hpcd->ep[0].xfer_length > req->wLength)
        {
            hpcd->ep[0].xfer_length = req->wLength;
        }
//        LOG_DEBUG("configuration xfer length %d\n", hpcd->ep[0].xfer_length);

	}
    else if(desc_idx == USB_DESC_TYPE_REPORT){
        index = req->wIndex & 0xff;
        get_dev_desc(USB_DESC_TYPE_REPORT, index, (uint8_t **)&hpcd->ep[0].xfer_buf, (uint16_t *)&hpcd->ep[0].xfer_length);

	}
    else if(desc_idx == USB_DESC_TYPE_DEVICEQUALIFIER){
        index = req->wIndex & 0xff;
        get_dev_desc(USB_DESC_TYPE_DEVICEQUALIFIER, index, (uint8_t **)&hpcd->ep[0].xfer_buf, (uint16_t *)&hpcd->ep[0].xfer_length);
        //hal_pcd_in_ep_fifo_stall_set(hpcd, 0);

	}
    else if(desc_idx == USB_DESC_TYPE_STRING){
//        LOG_DEBUG("Get String Desc\n");
        index = req->wValue & 0xff;
        str_desc.type = USB_DESC_TYPE_STRING;
        if (index == USB_STRING_LANGID_INDEX)
        {
            str_desc.bLength = 4;
            str_desc.String[0] = 0x09;
            str_desc.String[1] = 0x04;
            len = 4;
        }
        else
        {
            get_dev_desc(USB_DESC_TYPE_STRING, index, &ptr, &len);
            str_desc.bLength = len * 2 + 2;

            i = 0;
            while ((*ptr != '\n') && (i < 64))
            {
                str_desc.String[i] = *ptr++;
                i++;
                str_desc.String[i] = 0;
                i++;
            }
        }
        
        hpcd->ep[0].xfer_length = str_desc.bLength;
        if (req->wLength < hpcd->ep[0].xfer_length)
        {
            hpcd->ep[0].xfer_length = req->wLength;
        }
        hpcd->ep[0].xfer_buf = (uint8_t *)&str_desc;
        // hpcd->ep[0].xfer_length = len;

	}

    return 0;
}


void usb_pcd_std_req_proc(usb_hal_pcd_t *hpcd, ureq_t req)
{
    //dev_cfg_t dev_cfg;
    //ep_in_ctl_t tx_ctl;

	if(req->bRequest == USB_REQ_GET_DESCRIPTOR)
    {
		
        hid_remote_wakeup_rdy = HID_REMOTE_WAKEUP_IDLE;
        usb_get_desc_req_proc(hpcd, req);

    }
    else if(req->bRequest == USB_REQ_SET_FEATURE && req->wValue == USB_FEATURE_DEV_REMOTE_WAKEUP)
    {
        hid_remote_wakeup_rdy = HID_REMOTE_WAKEUP_READY;
    }
    
}

void usb_pcd_class_req_proc(usb_hal_pcd_t *hpcd, ureq_t req)

{
    uint8_t report_type;
    uint8_t report_id;

	if(req->bRequest == HID_SET_IDLE || req->bRequest == HID_SET_PROTOCOL)
    {

        hid_remote_wakeup_rdy = HID_REMOTE_WAKEUP_IDLE;
		mouse_idle_rate = (req->wValue >> 8);
		hid_mouse_ready = 1;

	}
	/*
    else if(req->bRequest == HID_SET_REPORT){
        report_id   = (req->wValue >> 8) & 0xff;
        report_type = req->wValue & 0xff;

        hpcd->ep[0].xfer_tmpcnt = 0;
        hpcd->ep[0].xfer_cnt = 0;
        hpcd->ep[0].xfer_buf = (uint8_t *)&report;
        hpcd->ep[0].xfer_length = req->wLength;
        // register call back TODO

	}
	 */

}





#endif
