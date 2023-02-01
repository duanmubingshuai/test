#ifndef _USB_USR_CONFIG_H_
#define _USB_USR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/*USB Register Def*/
#define MAX_EP_IP_SUPPORT  16
#define MAX_EP_NUM         2

#define USB_DEV_KBD  	0
#define USB_DEV_MS		1 
#define USB_DEV_MSKBD	2



/*FS OTG-TLI Global CSR (Device Mode)*/
typedef struct{
	__IO uint32_t DEV_CFG;
	__IO uint32_t DEV_STS;
	__IO uint32_t DEV_INT_STS;
	__IO uint32_t DEV_INT_EN;
	__IO uint32_t DEV_OUT_EP_FIFO_SZ;
	__IO uint32_t DEV_GLB_EP_INT_EN;
	__IO uint32_t DEV_THD;
	__IO uint32_t DEV_OUT_EP_FIFO_STS;
	__IO uint32_t DEV_SETUP_FIFO_STS;
	__IO uint32_t DEV_GLB_EP_INT;
	__IO uint32_t FRAME_NUM;
	__IO uint32_t RSVD[53];
	__IO uint32_t SETUP_DATA[2];	//0x44300  -- 4
	__IO uint32_t BIU_CTL;
}AP_USB_DEV_GLB_TypeDef;

typedef struct{
	__IO uint32_t RSVD;
	__IO uint32_t DEV_EP_INFO[MAX_EP_NUM];
}AP_USB_DEV_MAC_TypeDef;

typedef struct{
	__IO uint32_t HST_INT_STS;
	__IO uint32_t HST_INT_EN;
	__IO uint32_t HST_STS;
	__IO uint32_t HST_CTL;
	__IO uint32_t HST_FIFO_SZ;
	__IO uint32_t HST_THD;
	__IO uint32_t HST_OUT_SZ;
}AP_USB_HST_GLB_TypeDef;

typedef struct{
	__I  uint32_t RSVD00[13];
	__IO uint32_t FMINTEVAL;
	__IO uint32_t FMREMAINING;
	__IO uint32_t FMNUMBER;
	__I  uint32_t RSVD40[4];
	__IO uint32_t PORTSTSCTL;
	__I  uint32_t RSVD54[15];
	__IO uint32_t TOKEN;
	__IO uint32_t OTGCTLSTS;
}AP_USB_HST_MAC_TypeDef;

typedef struct{
	__IO uint32_t DEV_IN_EP_DMA[MAX_EP_IP_SUPPORT];
	__IO uint32_t DEV_OUT_EP_DMA[MAX_EP_IP_SUPPORT];
	__IO uint32_t HST_IN_EP_DMA;
	__IO uint32_t HST_OUT_EP_DMA;
	__IO uint32_t HST_IN_TOKEN;
	__IO uint32_t HST_OUT_TOKEN;
	__IO uint32_t HST_IN_MAX_PKT;
	__IO uint32_t HST_OUT_MAX_PKT;
}AP_USB_DIU_TypeDef;


typedef struct{
	__IO uint32_t DM_PULL;
	__IO uint32_t DP_PULL;
	__IO uint32_t DM_CTL;
	__IO uint32_t DP_CTL;
	__IO uint32_t PHY_STATUS;
	__IO uint32_t PHY_CTL0;
	__IO uint32_t PHY_CTL1;
	__I  uint32_t RSVD0[5];
	__IO uint32_t OTG_STATUS;
	__IO uint32_t OTG_CTL0;
	__IO uint32_t OTG_CTL1;
	__I  uint32_t RSVD1;
	__IO uint32_t INT_STATUS;
	__IO uint32_t INT_CTL;
	__IO uint32_t RSVD2[2];
	__IO uint32_t ID_PULL;
	__IO uint32_t ID_CTL;
	__IO uint32_t RSVD3[3];
	__IO uint32_t CLK_CTL;
	__IO uint32_t RST_CTL;
}AP_USB_TOP_TypeDef;


#define USB_BASE_ADDR        (0x14000000UL)           /*USB*/

#define USB_DYNAMIC                     0x00

#define USB_CLASS_DEVICE                0x00
#define USB_CLASS_AUDIO                 0x01
#define USB_CLASS_CDC                   0x02
#define USB_CLASS_HID                   0x03
#define USB_CLASS_PHYSICAL              0x05
#define USB_CLASS_IMAGE                 0x06
#define USB_CLASS_PRINTER               0x07
#define USB_CLASS_MASS_STORAGE          0x08
#define USB_CLASS_HUB                   0x09
#define USB_CLASS_CDC_DATA              0x0a
#define USB_CLASS_SMART_CARD            0x0b
#define USB_CLASS_SECURITY              0x0d
#define USB_CLASS_VIDEO                 0x0e
#define USB_CLASS_HEALTHCARE            0x0f
#define USB_CLASS_DIAG_DEVICE           0xdc
#define USB_CLASS_WIRELESS              0xe0
#define USB_CLASS_MISC                  0xef
#define USB_CLASS_APP_SPECIFIC          0xfe
#define USB_CLASS_VEND_SPECIFIC         0xff

#define USB_DESC_TYPE_DEVICE            0x01
#define USB_DESC_TYPE_CONFIGURATION     0x02
#define USB_DESC_TYPE_STRING            0x03
#define USB_DESC_TYPE_INTERFACE         0x04
#define USB_DESC_TYPE_ENDPOINT          0x05
#define USB_DESC_TYPE_DEVICEQUALIFIER   0x06
#define USB_DESC_TYPE_OTHERSPEED        0x07
#define USB_DESC_TYPE_IAD               0x0b
#define USB_DESC_TYPE_HID               0x21
#define USB_DESC_TYPE_REPORT            0x22
#define USB_DESC_TYPE_PHYSICAL          0x23
#define USB_DESC_TYPE_HUB               0x29

#define USB_DESC_LENGTH_DEVICE          0x12
#define USB_DESC_LENGTH_CONFIG          0x9
#define USB_DESC_LENGTH_HID             0x9
#define USB_DESC_LENGTH_IAD             0x8
#define USB_DESC_LENGTH_STRING          0x4
#define USB_DESC_LENGTH_HEADER_FUNCTION 0x5
#define USB_DESC_LENGTH_CALL_MANAGEMENT 0x5
#define USB_DESC_LENGTH_ABSTRACT_CONTROL_MANAGEMENT 0x4	
#define USB_DESC_LENGTH_UNION_FUNCTION  0x5
#define USB_DESC_LENGTH_INTERFACE       0x9
#define USB_DESC_LENGTH_ENDPOINT        0x7
#define USB_DESC_LENGTH_QUALIFIER       0xA
#define USB_DESC_LENGTH_KEYBOARD_REPORT 0x3B
//#define USB_DESC_LENGTH_MOUSE_REPORT    0x40
//#define USB_DESC_LENGTH_MOUSE_REPORT    0xB1
#define USB_LENGTH_MOUSE_REPORT    8//6//0x8



#define USB_REQ_TYPE_STANDARD           0x00
#define USB_REQ_TYPE_CLASS              0x20
#define USB_REQ_TYPE_VENDOR             0x40
#define USB_REQ_TYPE_MASK               0x60

#define USB_REQ_TYPE_DIR_OUT            0x00
#define USB_REQ_TYPE_DIR_IN             0x80

#define USB_REQ_TYPE_DEVICE             0x00
#define USB_REQ_TYPE_INTERFACE          0x01
#define USB_REQ_TYPE_ENDPOINT           0x02
#define USB_REQ_TYPE_OTHER              0x03
#define USB_REQ_TYPE_RECIPIENT_MASK     0x1f

#define USB_FEATURE_ENDPOINT_HALT       0x00
#define USB_FEATURE_DEV_REMOTE_WAKEUP   0x01
#define USB_FEATURE_TEST_MODE           0x02

#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_DESCRIPTOR          0x07
#define USB_REQ_GET_CONFIGURATION       0x08
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_GET_INTERFACE           0x0A
#define USB_REQ_SET_INTERFACE           0x0B
#define USB_REQ_SYNCH_FRAME             0x0C
#define USB_REQ_SET_ENCRYPTION          0x0D
#define USB_REQ_GET_ENCRYPTION          0x0E
#define USB_REQ_RPIPE_ABORT             0x0E
#define USB_REQ_SET_HANDSHAKE           0x0F
#define USB_REQ_RPIPE_RESET             0x0F
#define USB_REQ_GET_HANDSHAKE           0x10
#define USB_REQ_SET_CONNECTION          0x11
#define USB_REQ_SET_SECURITY_DATA       0x12
#define USB_REQ_GET_SECURITY_DATA       0x13
#define USB_REQ_SET_WUSB_DATA           0x14
#define USB_REQ_LOOPBACK_DATA_WRITE     0x15
#define USB_REQ_LOOPBACK_DATA_READ      0x16
#define USB_REQ_SET_INTERFACE_DS        0x17

#define USB_STRING_LANGID_INDEX         0x00
#define USB_STRING_MANU_INDEX           0x01
#define USB_STRING_PRODUCT_INDEX        0x02
#define USB_STRING_SERIAL_INDEX         0x03
#define USB_STRING_CONFIG_INDEX         0x04
#define USB_STRING_INTERFACE_INDEX      0x05
#define USB_STRING_OS_INDEX             0x06
#define USB_STRING_MAX                  USB_STRING_OS_INDEX

#define USB_STRING_OS                   "RTOS"

#define USB_PID_OUT                     0x01
#define USB_PID_ACK                     0x02
#define USB_PID_DATA0                   0x03
#define USB_PID_SOF                     0x05
#define USB_PID_IN                      0x09
#define USB_PID_NACK                    0x0A
#define USB_PID_DATA1                   0x0B
#define USB_PID_PRE                     0x0C
#define USB_PID_SETUP                   0x0D
#define USB_PID_STALL                   0x0E

#define USB_EP_DESC_OUT                 0x00
#define USB_EP_DESC_IN                  0x80
#define USB_EP_DESC_NUM_MASK            0x0f

#define USB_EP_ATTR_CONTROL             0x00
#define USB_EP_ATTR_ISOC                0x01
#define USB_EP_ATTR_BULK                0x02
#define USB_EP_ATTR_INT                 0x03
#define USB_EP_ATTR_TYPE_MASK           0x03

#define USB_EP_ASYNC                    0x04
#define USB_EP_SYNC_TYPE_MASK           0x0C

#define USB_EPNO_MASK                   0x7f
#define USB_DIR_OUT                     0x00
#define USB_DIR_IN                      0x80
#define USB_DIR_INOUT                   0x40
#define USB_DIR_MASK                    0x80

#define ID_UNASSIGNED                   0
#define ID_ASSIGNED                     1

#define RH_GET_PORT_STATUS              0
#define RH_SET_PORT_STATUS              1
#define RH_CLEAR_PORT_FEATURE           2
#define RH_SET_PORT_FEATURE             3

#define USB_BUS_POWERED                 0
#define USB_SELF_POWERED                1
#define USB_REMOTE_WAKEUP               1
#define USB_EP_HALT                     0

/*
 * Port feature numbers
 */
#define PORT_FEAT_CONNECTION            0
#define PORT_FEAT_ENABLE                1
#define PORT_FEAT_SUSPEND               2
#define PORT_FEAT_OVER_CURRENT          3
#define PORT_FEAT_RESET                 4
#define PORT_FEAT_POWER                 8
#define PORT_FEAT_LOWSPEED              9
#define PORT_FEAT_HIGHSPEED             10
#define PORT_FEAT_C_CONNECTION          16
#define PORT_FEAT_C_ENABLE              17
#define PORT_FEAT_C_SUSPEND             18
#define PORT_FEAT_C_OVER_CURRENT        19
#define PORT_FEAT_C_RESET               20
#define LOGIC_
#ifdef LOGIC
#define _VENDOR_ID                  0x046D
#define _PRODUCT_ID                 0xC534
#else
#define _VENDOR_ID                  0x1919
#define _PRODUCT_ID                 0x1919
#endif
#define USB_BCD_DEVICE              0x2901   /* USB Specification Release Number in Binary-Coded Decimal */
#define USB_BCD_VERSION             0x0200   /* USB 2.0 */
#define EP0_IN_ADDR                 0x80
#define EP0_OUT_ADDR                0x00

#define HID_RX_BUFSIZE              128
#define HID_MAX_PACKET_SIZE         64

#define ALIGN(x) __attribute__((aligned(x)))

typedef enum
{
    USB_STATE_NOTATTACHED = 0,
    USB_STATE_ATTACHED,
    USB_STATE_POWERED,
    USB_STATE_RECONNECTING,
    USB_STATE_UNAUTHENTICATED,
    USB_STATE_DEFAULT,
    USB_STATE_ADDRESS,
    USB_STATE_CONFIGURED,
    USB_STATE_SUSPENDED
}udevice_state_t;

typedef enum
{
    STAGE_IDLE,
    STAGE_SETUP,
    STAGE_STATUS_IN,
    STAGE_STATUS_OUT,
    STAGE_DIN,
    STAGE_DOUT
} uep0_stage_t;

#pragma pack(1)

struct usb_descriptor
{
    uint8_t bLength;
    uint8_t type;
};
typedef struct usb_descriptor* udesc_t;

struct udevice_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
};
typedef struct udevice_descriptor* udev_desc_t;

struct uconfig_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t MaxPower;
};
typedef struct uconfig_descriptor* ucfg_desc_t;

struct uinterface_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
};
typedef struct uinterface_descriptor* uintf_desc_t;

/* Interface Association Descriptor (IAD) */
struct uiad_descriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bFirstInterface;
    uint8_t bInterfaceCount;
    uint8_t bFunctionClass;
    uint8_t bFunctionSubClass;
    uint8_t bFunctionProtocol;
    uint8_t iFunction;
};
typedef struct uiad_descriptor* uiad_desc_t;

struct uendpoint_descriptor
{
    uint8_t  bLength;
    uint8_t  type;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
};
typedef struct uendpoint_descriptor* uep_desc_t;

struct ustring_descriptor
{
    uint8_t bLength;
    uint8_t type;
    uint8_t String[64];
};
typedef struct ustring_descriptor* ustr_desc_t;

struct uhub_descriptor
{
    uint8_t length;
    uint8_t type;
    uint8_t num_ports;
    uint16_t characteristics;
    uint8_t pwron_to_good;        /* power on to power good */
    uint8_t current;
    uint8_t removable[8];
    uint8_t pwr_ctl[8];
};
typedef struct uhub_descriptor* uhub_desc_t;

/* USB_DESC_TYPE_DEVICEQUALIFIER: Device Qualifier descriptor */
struct usb_qualifier_descriptor
{
    uint8_t  bLength;
    uint8_t  bDescriptorType;

    uint16_t bcdUSB; // TODO: big-endian.
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint8_t  bNumConfigurations;
    uint8_t  bRESERVED;
} __attribute__ ((packed));

struct urequest
{
    uint8_t  request_type;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};
typedef struct urequest* ureq_t;


#pragma pack()

//int32_t get_dev_desc(uint8_t desc_idx, uint8_t str_idx, uint8_t **buf , uint16_t *len);


#ifdef __cplusplus
};
#endif

#endif
