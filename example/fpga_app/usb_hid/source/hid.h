#ifndef __HID_H__
#define __HID_H__

#ifdef __cplusplus
extern "C" {
#endif
#define USB_HID_BUFSIZE                 0x40

#define USB_HID_SUBCLASS_NONE           0x00
#define USB_HID_SUBCLASS_BOOT           0x01

#define USB_HID_PROTOCOL_NONE           0x00
#define USB_HID_PROTOCOL_KEYBOARD       0x01
#define USB_HID_PROTOCOL_MOUSE          0x02

#define HID_GET_REPORT                  0x01
#define HID_GET_IDLE                    0x02
#define HID_GET_PROTOCOL                0x03
#define HID_SET_REPORT                  0x09
#define HID_SET_IDLE                    0x0A
#define HID_SET_PROTOCOL                0x0B


#define HID_REMOTE_WAKEUP_IDLE          0
#define HID_REMOTE_WAKEUP_READY         1
#define HID_REMOTE_WAKEUP_GOING         2


#pragma pack(1)

struct uhid_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdHID;
    uint8_t bCountryCode;    
    uint8_t bNumDescriptor;
	uint8_t bClassDescriptorType;
	uint16_t wClassDescriptorLen;
};
typedef struct ucdc_hid_descriptor* uhid_desc_t;

struct uhid_config_descriptor
{
	struct uconfig_descriptor ucfg_desc;
    //struct uinterface_descriptor keyboard_intf_desc;
    //struct uhid_descriptor keyboard_hid_desc;
	//struct uendpoint_descriptor keyboard_ep_desc;
	struct uinterface_descriptor mouse_intf_desc;
    struct uhid_descriptor mouse_hid_desc;
	struct uendpoint_descriptor mouse_ep_desc;
};
typedef struct uhid_config_descriptor* uhid_config_desc_t;


typedef union {
	uint32_t d32;
	struct{
		 uint32_t X     : 12;
		 uint32_t Y     : 12;
		 uint8_t  Wheel :8;
	}b;
}mouse_ctl;

struct hid_mouse_report{
	uint8_t report_id;
	uint16_t button;
	mouse_ctl  ctl;
	uint8_t ac_pan;
};

struct hid_mouse_report_{
	uint8_t button;
	mouse_ctl  ctl;
	uint8_t report_id;
	uint8_t ac_pan;
};

struct hid_keyboard_report{
	uint8_t key_ctl[2];
	uint8_t key_val[6];
};

#pragma pack()

#ifdef __cplusplus
}
#endif

#endif    /*  __CDC_H__ */
