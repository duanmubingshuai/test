/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/******************************************************************************

 *****************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rom_sym_def.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "gattservapp.h"
#include "hidkbdservice.h"
#include "peripheral.h"
#include "hiddev.h"
#include "battservice.h"


/*********************************************************************
	MACROS
*/

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
// HID service
CONST uint8 hidServUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_SERV_UUID), HI_UINT16(HID_SERV_UUID)
};

// HID Information characteristic
CONST uint8 hidInfoUUID[ATT_BT_UUID_SIZE] =
{
	LO_UINT16(HID_INFORMATION_UUID), HI_UINT16(HID_INFORMATION_UUID)
};

// HID Report Map characteristic
CONST uint8 hidReportMapUUID[ATT_BT_UUID_SIZE] =
{
	LO_UINT16(REPORT_MAP_UUID), HI_UINT16(REPORT_MAP_UUID)
};
	
// HID Report characteristic
CONST uint8 hidReportUUID[ATT_BT_UUID_SIZE] =
{
	LO_UINT16(REPORT_UUID), HI_UINT16(REPORT_UUID)
};

// HID Protocol Mode characteristic
CONST uint8 hidProtocolModeUUID[ATT_BT_UUID_SIZE] =
{
	LO_UINT16(PROTOCOL_MODE_UUID), HI_UINT16(PROTOCOL_MODE_UUID)
};

#if( defined(EN_MOUSE_REPORT) && EN_MOUSE_REPORT )
// HID Boot Mouse Input Report characteristic
CONST uint8 hidBootMouseInputUUID[ATT_BT_UUID_SIZE] =
{
	LO_UINT16(BOOT_MOUSE_INPUT_UUID), HI_UINT16(BOOT_MOUSE_INPUT_UUID)
};
#endif

#if( defined(EN_KEYBOARD_INPUT) && EN_KEYBOARD_INPUT )
// HID Boot Keyboard Input Report characteristic
CONST uint8 hidBootKeyInputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_KEY_INPUT_UUID), HI_UINT16(BOOT_KEY_INPUT_UUID)
};
#endif

#if( defined(EN_KEYBOARD_OUTPUT) && EN_KEYBOARD_OUTPUT )
// HID Boot Keyboard Output Report characteristic
CONST uint8 hidBootKeyOutputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_KEY_OUTPUT_UUID), HI_UINT16(BOOT_KEY_OUTPUT_UUID)
};
#endif

#if( defined(HID_SUP_CTRL_POINT_CHAR) && HID_SUP_CTRL_POINT_CHAR)
// HID Control Point characteristic
CONST uint8 hidControlPointUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_CTRL_PT_UUID), HI_UINT16(HID_CTRL_PT_UUID)
};
#endif


/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/
// HID Information characteristic value
static CONST uint8 hidInfo[HID_INFORMATION_LEN] =
{
    LO_UINT16(0x0111), HI_UINT16(0x0111),             // bcdHID (USB HID version)
    0x00,                                             // bCountryCode
    HID_KBD_FLAGS                                     // Flags
};


static CONST uint8 hidReportMap[] =
{

    #if EN_CONSUMER_MODE  
		0x05, 0x0C,                     // Usage Page (Consumer)  ---- 0x0c
		0x09, 0x01,                     // Usage (Consumer Control)
		0xA1, 0x01,                     // Collection (Application)
		0x85, HID_RPT_ID_CC_IN,                     //     Report Id (2)

		0x09, 0xCD,                     //     Usage (Play/Pause)
		0x09, 0x30,                     //     
		0x09, 0xB5,                     //     Usage (Scan Next Track)
		0x09, 0xB6,                     //     Usage (Scan Previous Track)
		0x09, 0xEA,                     //     Usage (Volume Down)
		0x09, 0xE9,                     //     Usage (Volume Up)
		0x09, 0x30,                     //     Power 
		0x09, 0x41,                     //     Usage (Menu Pick)
		
		0x15, 0x00,                     //     Logical minimum (0)
		0x25, 0x01,                     //     Logical maximum (1)
		0x75, 0x01,                     //     Report Size (1)
		0x95, 0x08,                     //     Report Count (8)
		0x81, 0x02,                     //
		0xC0 ,
    #endif
};


// HID report map length
uint16 hidReportMapLen = sizeof(hidReportMap);

// HID report mapping table
hidRptMap_t  hidRptMap[HID_NUM_REPORTS];

/*********************************************************************
    Profile Attributes - variables
*/

// HID Service attribute
static CONST gattAttrType_t hidService = { ATT_BT_UUID_SIZE, hidServUUID };

// HID Information characteristic
static uint8 hidInfoProps = GATT_PROP_READ;

// HID Report Map characteristic
static uint8 hidReportMapProps = GATT_PROP_READ;

// HID External Report Reference Descriptor
//static uint8 hidExtReportRefDesc[ATT_BT_UUID_SIZE] = { LO_UINT16(BATT_LEVEL_UUID), HI_UINT16(BATT_LEVEL_UUID) };

#if( defined(HID_SUP_CTRL_POINT_CHAR) && HID_SUP_CTRL_POINT_CHAR)
// HID Control Point characteristic
static uint8 hidControlPointProps = GATT_PROP_WRITE_NO_RSP;
static uint8 hidControlPoint;
#endif

// HID Protocol Mode characteristic
static uint8 hidProtocolModeProps = GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;
uint8 hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

#if( defined(EN_KEYBOARD_INPUT) && EN_KEYBOARD_INPUT )
// HID Report characteristic, key input
static uint8 hidReportKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportKeyIn;
static uint16 hidReportKeyInClientCharCfg = GATT_CFG_NO_OPERATION;

// HID Report Reference characteristic descriptor, key input
static uint8 hidReportRefKeyIn[HID_REPORT_REF_LEN] = { HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT };

// HID Boot Keyboard Input Report
static uint8 hidReportBootKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportBootKeyIn;
static uint16 hidReportBootKeyInClientCharCfg  = GATT_CFG_NO_OPERATION;

#endif

#if( defined(EN_KEYBOARD_OUTPUT) && EN_KEYBOARD_OUTPUT )
// HID Report characteristic, LED output
static uint8 hidReportLedOutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportLedOut;
// HID Report Reference characteristic descriptor, LED output
static uint8 hidReportRefLedOut[HID_REPORT_REF_LEN] = { HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT };

// HID Boot Keyboard Output Report
static uint8 hidReportBootKeyOutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportBootKeyOut;
#endif

#if( defined(EN_MOUSE_REPORT) && EN_MOUSE_REPORT )
// HID Boot Mouse Input Report
static uint8 hidReportBootMouseInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportBootMouseIn;
static uint16 hidReportBootMouseInClientCharCfg = GATT_CFG_NO_OPERATION;

// HID Report Reference characteristic descriptor, mouse input
static uint8 hidReportRefMouseIn[HID_REPORT_REF_LEN] = { HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT };

static uint8 hidReportMouseInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint16 hidReportMouseInClientCharCfg = GATT_CFG_NO_OPERATION;
static uint8 hidReportMouseIn;
#endif

#if( defined(EN_FEATURE_REPORT) && EN_FEATURE_REPORT )
// Feature Report
static uint8 hidReportFeatureProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 hidReportFeature;

// HID Report Reference characteristic descriptor, Feature
static uint8 hidReportRefFeature[HID_REPORT_REF_LEN] = { HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE };
#endif

#if( defined(EN_CONSUMER_MODE) && EN_CONSUMER_MODE )
// HID Report Reference characteristic descriptor, consumer control input
static uint8 hidReportRefCCIn[HID_REPORT_REF_LEN] = { HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT };

// HID Report characteristic, consumer control input
static uint8 hidReportCCInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportCCIn;
uint16 hidReportCCInClientCharCfg = GATT_CFG_NO_OPERATION;

#endif



/*********************************************************************
    Profile Attributes - Table
*/

gattAttribute_t hidAttrTbl[] =
{
    // HID Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)& hidService                      /* pValue */
    },

    // HID Information characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidInfoProps
    },

    // HID Information characteristic
    {
        { ATT_BT_UUID_SIZE, hidInfoUUID },
        GATT_PERMIT_READ,
        0,
        (uint8*) hidInfo
    },
    
#if( defined(HID_SUP_CTRL_POINT_CHAR) && HID_SUP_CTRL_POINT_CHAR)
    // HID Control Point characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidControlPointProps
    },

    // HID Control Point characteristic
    {
        { ATT_BT_UUID_SIZE, hidControlPointUUID },
        GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidControlPoint
    },
#endif

    // HID Protocol Mode characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidProtocolModeProps
    },

    // HID Protocol Mode characteristic
    {
        { ATT_BT_UUID_SIZE, hidProtocolModeUUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidProtocolMode
    },


    // HID Report Map characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportMapProps
    },

    // HID Report Map characteristic
    {
        { ATT_BT_UUID_SIZE, hidReportMapUUID },
        GATT_PERMIT_READ,
        0,
        (uint8*) hidReportMap
    },

//    // HID External Report Reference Descriptor
//    {
//        { ATT_BT_UUID_SIZE, extReportRefUUID },
//        GATT_PERMIT_READ,
//        0,
//        hidExtReportRefDesc
//    },
    
    #if( defined(EN_MOUSE_REPORT) && EN_MOUSE_REPORT )
// HID Report characteristic, mouse input declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportMouseInProps
    },

    // HID Report characteristic, mouse input
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportMouseIn
    },

    // HID Report characteristic client characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8*)& hidReportMouseInClientCharCfg
    },

    // HID Report Reference characteristic descriptor, mouse input
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefMouseIn
    },
    // HID Boot Mouse Input Report declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportBootMouseInProps
    },

    // HID Boot Mouse Input Report
    {
        { ATT_BT_UUID_SIZE, hidBootMouseInputUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportBootMouseIn
    },

    // HID Boot Mouse Input Report characteristic client characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8*)& hidReportBootMouseInClientCharCfg
    },
    #endif

	#if( defined(EN_KEYBOARD_INPUT) && EN_KEYBOARD_INPUT )
    // HID Report characteristic, key input declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportKeyInProps
    },

    // HID Report characteristic, key input
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportKeyIn
    },

    // HID Report characteristic client characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8*)& hidReportKeyInClientCharCfg
    },

    // HID Report Reference characteristic descriptor, key input
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefKeyIn
    },
    // HID Boot Keyboard Input Report declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportBootKeyInProps
    },

    // HID Boot Keyboard Input Report
    {
        { ATT_BT_UUID_SIZE, hidBootKeyInputUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportBootKeyIn
    },

    // HID Boot Keyboard Input Report characteristic client characteristic configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        (uint8*)& hidReportBootKeyInClientCharCfg
    },
	#endif

	#if( defined(EN_KEYBOARD_OUTPUT) && EN_KEYBOARD_OUTPUT )
    // HID Report characteristic, LED output declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportLedOutProps
    },

    // HID Report characteristic, LED output
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidReportLedOut
    },

    // HID Report Reference characteristic descriptor, LED output
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefLedOut
    },
    // HID Boot Keyboard Output Report declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportBootKeyOutProps
    },

    // HID Boot Keyboard Output Report
    {
        { ATT_BT_UUID_SIZE, hidBootKeyOutputUUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidReportBootKeyOut
    },
	#endif
    
	#if( defined(EN_CONSUMER_MODE) && EN_CONSUMER_MODE )
    // HID Report characteristic declaration, consumer control
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportCCInProps
    },

    // HID Report characteristic, consumer control
    {
        { ATT_BT_UUID_SIZE, hidReportUUID },
        GATT_PERMIT_ENCRYPT_READ,
        0,
        &hidReportCCIn
    },

    // HID Report characteristic client characteristic configuration, consumer control
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8*)& hidReportCCInClientCharCfg
    },

    // HID Report Reference characteristic descriptor, consumer control
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefCCIn
    },
    #endif

	#if( defined(EN_FEATURE_REPORT) && EN_FEATURE_REPORT )
    // Feature Report declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &hidReportFeatureProps
    },

    // Feature Report
    {
        { ATT_BT_UUID_SIZE,  hidReportUUID},
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_ENCRYPT_WRITE,
        0,
        &hidReportFeature
    },

    // HID Report Reference characteristic descriptor, feature
    {
        { ATT_BT_UUID_SIZE, reportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidReportRefFeature
    },
    #endif
};

/*********************************************************************
    LOCAL FUNCTIONS
*/

/*********************************************************************
    PROFILE CALLBACKS
*/

// Service Callbacks
CONST gattServiceCBs_t hidKbdCBs =
{
    HidDev_ReadAttrCB,  // Read callback function pointer
    HidDev_WriteAttrCB, // Write callback function pointer
    NULL                // Authorization callback function pointer
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      HidKbd_AddService

    @brief   Initializes the HID Service by registering
            GATT attributes with the GATT server.

    @return  Success or Failure
*/
bStatus_t HidKbd_AddService( void )
{
    uint8 status = SUCCESS;
	
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( hidAttrTbl, GATT_NUM_ATTRS( hidAttrTbl ), &hidKbdCBs );
    return ( status );
}



/*********************************************************************
*********************************************************************/
