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

// HID Boot Keyboard Input Report characteristic
CONST uint8 hidBootKeyInputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_KEY_INPUT_UUID), HI_UINT16(BOOT_KEY_INPUT_UUID)
};

// HID Boot Mouse Input Report characteristic
CONST uint8 hidBootMouseInputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_MOUSE_INPUT_UUID), HI_UINT16(BOOT_MOUSE_INPUT_UUID)
};

// HID Boot Keyboard Output Report characteristic
CONST uint8 hidBootKeyOutputUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(BOOT_KEY_OUTPUT_UUID), HI_UINT16(BOOT_KEY_OUTPUT_UUID)
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

// HID Control Point characteristic
CONST uint8 hidControlPointUUID[ATT_BT_UUID_SIZE] =
{
    LO_UINT16(HID_CTRL_PT_UUID), HI_UINT16(HID_CTRL_PT_UUID)
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

    0x05, 0x0C,                     // Usage Page (Consumer)  ---- 0x0c
    0x09, 0x01,                     // Usage (Consumer Control)
    0xA1, 0x01,                     // Collection (Application)
    //  0x85, 0x01,                     //     Report Id (2)
    //  0x15, 0x00,                     //     Logical minimum (0)
    //  0x25, 0x01,                     //     Logical maximum (1)
    //  0x75, 0x01,                     //     Report Size (1)
    //  0x95, 0x01,                     //     Report Count (1)

    0x09, 0xCD,                     //     Usage (Play/Pause)
    0x09, 0x30,                     //
    0x09, 0xB5,                     //     Usage (Scan Next Track)
    0x09, 0xB6,                     //     Usage (Scan Previous Track)
    0x09, 0xEA,                     //     Usage (Volume Down)
    0x09, 0xE9,                     //     Usage (Volume Up)
    0x09, 0x30,                     //     Power
    0x09, 0x30,

    0x15, 0x00,                     //     Logical minimum (0)
    0x25, 0x01,                     //     Logical maximum (1)
    0x75, 0x01,                     //     Report Size (1)
    0x95, 0x08,                     //     Report Count (1)
    0x81, 0x02,                     //
    0xC0

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

// Include attribute (Battery service)
static uint16 include = GATT_INVALID_HANDLE;

// HID Information characteristic
static uint8 hidInfoProps = GATT_PROP_READ;

// HID Report Map characteristic
static uint8 hidReportMapProps = GATT_PROP_READ;

// HID External Report Reference Descriptor
static uint8 hidExtReportRefDesc[ATT_BT_UUID_SIZE] =
{ LO_UINT16(BATT_LEVEL_UUID), HI_UINT16(BATT_LEVEL_UUID) };

// HID Control Point characteristic
static uint8 hidControlPointProps = GATT_PROP_WRITE_NO_RSP;
static uint8 hidControlPoint;

// HID Protocol Mode characteristic
static uint8 hidProtocolModeProps = GATT_PROP_READ | GATT_PROP_WRITE_NO_RSP;
uint8 hidProtocolMode = HID_PROTOCOL_MODE_REPORT;

// HID Report characteristic, key input
static uint8 hidReportKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportKeyIn;
static gattCharCfg_t hidReportKeyInClientCharCfg[GATT_MAX_NUM_CONN];

// HID Report Reference characteristic descriptor, key input
static uint8 hidReportRefKeyIn[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_KEY_IN, HID_REPORT_TYPE_INPUT };

// HID Report characteristic, LED output
static uint8 hidReportLedOutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportLedOut;

// HID Report Reference characteristic descriptor, LED output
static uint8 hidReportRefLedOut[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_LED_OUT, HID_REPORT_TYPE_OUTPUT };

// HID Boot Keyboard Input Report
//static uint8 hidReportBootKeyInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
//static uint8 hidReportBootKeyIn;
//static gattCharCfg_t hidReportBootKeyInClientCharCfg[GATT_MAX_NUM_CONN];

// HID Boot Keyboard Output Report
//static uint8 hidReportBootKeyOutProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;
static uint8 hidReportBootKeyOut;

// HID Boot Mouse Input Report
//static uint8 hidReportBootMouseInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
//static uint8 hidReportBootMouseIn;
//static gattCharCfg_t hidReportBootMouseInClientCharCfg[GATT_MAX_NUM_CONN];

// Feature Report
//static uint8 hidReportFeatureProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 hidReportFeature;

// HID Report Reference characteristic descriptor, Feature
//static uint8 hidReportRefFeature[HID_REPORT_REF_LEN] =
//{ HID_RPT_ID_FEATURE, HID_REPORT_TYPE_FEATURE };

#if EN_MOUSE_REPORT

// HID Report Reference characteristic descriptor, mouse input
static uint8 hidReportRefMouseIn[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT };

static uint8 hidReportMouseInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportMouseIn;
static gattCharCfg_t hidReportMouseInClientCharCfg[GATT_MAX_NUM_CONN];

#endif
#if EN_CONSUMER_MODE
// HID Report Reference characteristic descriptor, consumer control input
static uint8 hidReportRefCCIn[HID_REPORT_REF_LEN] =
{ HID_RPT_ID_CC_IN, HID_REPORT_TYPE_INPUT };

// HID Report characteristic, consumer control input
static uint8 hidReportCCInProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 hidReportCCIn;
static gattCharCfg_t hidReportCCInClientCharCfg[GATT_MAX_NUM_CONN];

#endif



/*********************************************************************
    Profile Attributes - Table
*/

static gattAttribute_t hidAttrTbl[] =
{
    // HID Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)& hidService                      /* pValue */
    },

    // Included service (battery)
    {
        { ATT_BT_UUID_SIZE, includeUUID },
        GATT_PERMIT_READ,
        0,
        (uint8*)& include
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
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8*) hidInfo
    },

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
        GATT_PERMIT_ENCRYPT_READ,
        0,
        (uint8*) hidReportMap
    },

    // HID External Report Reference Descriptor
    {
        { ATT_BT_UUID_SIZE, extReportRefUUID },
        GATT_PERMIT_READ,
        0,
        hidExtReportRefDesc
    },


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
    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, hidReportKeyInClientCharCfg );
    //GATTServApp_InitCharCfg( INVALID_CONNHANDLE, hidReportBootKeyInClientCharCfg );
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( hidAttrTbl, GATT_NUM_ATTRS( hidAttrTbl ), &hidKbdCBs );
    // Set up included service
    Batt_GetParameter( BATT_PARAM_SERVICE_HANDLE,
                       &GATT_INCLUDED_HANDLE( hidAttrTbl, HID_INCLUDED_SERVICE_IDX ) );
    // Construct map of reports to characteristic handles
    // Each report is uniquely identified via its ID and type
    // Key input report
    hidRptMap[0].id = hidReportRefKeyIn[0];
    hidRptMap[0].type = hidReportRefKeyIn[1];
    hidRptMap[0].handle = hidAttrTbl[HID_REPORT_KEY_IN_IDX].handle;
    hidRptMap[0].cccdHandle = hidAttrTbl[HID_REPORT_KEY_IN_CCCD_IDX].handle;
    hidRptMap[0].mode = HID_PROTOCOL_MODE_REPORT;
    // Battery level input report
    Batt_GetParameter( BATT_PARAM_BATT_LEVEL_IN_REPORT, &(hidRptMap[1]) );
    // Setup report ID map
    HidDev_RegisterReports( HID_NUM_REPORTS, hidRptMap );
    return ( status );
}

/*********************************************************************
    @fn      HidKbd_SetParameter

    @brief   Set a HID Kbd parameter.

    @param   id - HID report ID.
    @param   type - HID report type.
    @param   uuid - attribute uuid.
    @param   len - length of data to right.
    @param   pValue - pointer to data to write.  This is dependent on
            the input parameters and WILL be cast to the appropriate
            data type (example: data type of uint16 will be cast to
            uint16 pointer).

    @return  GATT status code.
*/
uint8 HidKbd_SetParameter( uint8 id, uint8 type, uint16 uuid, uint16 len, void* pValue )
{
    bStatus_t ret = SUCCESS;

    switch ( uuid )
    {
    case REPORT_UUID:
        if ( type ==  HID_REPORT_TYPE_OUTPUT )
        {
            if ( len == 1 )
            {
                hidReportLedOut = *((uint8*)pValue);
            }
            else
            {
                ret = ATT_ERR_INVALID_VALUE_SIZE;
            }
        }
        else if ( type == HID_REPORT_TYPE_FEATURE )
        {
            if ( len == 1 )
            {
                hidReportFeature = *((uint8*)pValue);
            }
            else
            {
                ret = ATT_ERR_INVALID_VALUE_SIZE;
            }
        }
        else
        {
            ret = ATT_ERR_ATTR_NOT_FOUND;
        }

        break;

    case BOOT_KEY_OUTPUT_UUID:
        if ( len == 1 )
        {
            hidReportBootKeyOut = *((uint8*)pValue);
        }
        else
        {
            ret = ATT_ERR_INVALID_VALUE_SIZE;
        }

        break;

    default:
        // ignore the request
        break;
    }

    return ( ret );
}

/*********************************************************************
    @fn      HidKbd_GetParameter

    @brief   Get a HID Kbd parameter.

    @param   id - HID report ID.
    @param   type - HID report type.
    @param   uuid - attribute uuid.
    @param   pLen - length of data to be read
    @param   pValue - pointer to data to get.  This is dependent on
            the input parameters and WILL be cast to the appropriate
            data type (example: data type of uint16 will be cast to
            uint16 pointer).

    @return  GATT status code.
*/
uint8 HidKbd_GetParameter( uint8 id, uint8 type, uint16 uuid, uint16* pLen, void* pValue )
{
    switch ( uuid )
    {
    case REPORT_UUID:
        if ( type ==  HID_REPORT_TYPE_OUTPUT )
        {
            *((uint8*)pValue) = hidReportLedOut;
            *pLen = 1;
        }
        else if ( type == HID_REPORT_TYPE_FEATURE )
        {
            *((uint8*)pValue) = hidReportFeature;
            *pLen = 1;
        }
        else
        {
            *pLen = 0;
        }

        break;

    case BOOT_KEY_OUTPUT_UUID:
        *((uint8*)pValue) = hidReportBootKeyOut;
        *pLen = 1;
        break;

    default:
        *pLen = 0;
        break;
    }

    return ( SUCCESS );
}

/*********************************************************************
*********************************************************************/
