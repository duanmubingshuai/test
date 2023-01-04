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



/*********************************************************************
    INCLUDES
*/
#include "rom_sym_def.h"
#include "OSAL.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_uuid.h"
#include "gatt_profile_uuid.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "devinfoservice.h"
#include "battservice.h"
#include "hiddev.h"
#include "hidkbd.h"
#include "rom_sym_def.h"
#include "global_config.h"
#include "ll_def.h"
#include "hidkbdservice.h"
#include "log.h"

/*********************************************************************
    MACROS
*/
// Task Events
#define START_DEVICE_EVT                      0x0001

/*********************************************************************
    CONSTANTS
*/

#define HID_DEV_DATA_LEN                      8

#ifdef HID_DEV_RPT_QUEUE_LEN
    #define HID_DEV_REPORT_Q_SIZE               (HID_DEV_RPT_QUEUE_LEN+1)
#else
    #define HID_DEV_REPORT_Q_SIZE               (10+1)
#endif

/*********************************************************************
    TYPEDEFS
*/
typedef struct
{
    uint8 id;
    uint8 type;
    uint8 len;
    uint8 data[HID_DEV_DATA_LEN];
} hidDevReport_t;

/*********************************************************************
    GLOBAL VARIABLES
*/
// Task ID
uint8 hidDevTaskId;

/*********************************************************************
    EXTERNAL VARIABLES
*/
extern uint16 gapRole_ConnectionHandle;
// HID report mapping table
extern hidRptMap_t	hidRptMap[];
extern uint16 hidReportCCInClientCharCfg;
extern gattAttribute_t hidAttrTbl[];

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/
/*********************************************************************
    LOCAL FUNCTIONS
*/
static void hidDev_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void hidDevProcessGattMsg( gattMsgEvent_t* pMsg );

/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      HidDev_Init

    @brief   Initialization function for the Hid Dev Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void HidDev_Init( uint8 task_id )
{
    hidDevTaskId = task_id;
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
    // Set up services
    #if( defined( SERV_GGS ) && SERV_GGS )
	    GGS_AddService( );            // GAP
	#endif
	#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
	    GATTServApp_AddService( );    // GATT attributes
    #endif
	#if( defined( SERV_DEV_INFO ) && SERV_DEV_INFO )
	    DevInfo_AddService();                           // Device Information Service
    #endif
	#if( defined( SERV_BATT_SERV ) && SERV_BATT_SERV )
	    Batt_AddService();
    #endif
	
    // Setup a delayed profile startup
    osal_set_event( hidDevTaskId, START_DEVICE_EVT );
}

/*********************************************************************
    @fn      HidDev_ProcessEvent

    @brief   Hid Dev Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 HidDev_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    #ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d,events 0x%X\n",__FILE__,__func__,__LINE__,events);
	#endif

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( hidDevTaskId )) != NULL )
        {
            hidDev_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & START_DEVICE_EVT )
    {
        // Start the Device
        GAPRole_StartDevice( );
		
        // Register with bond manager after starting device
        GAPBondMgr_Register( NULL );

        return ( events ^ START_DEVICE_EVT );
    }
	
    return 0;
}

/*********************************************************************
    @fn      HidDev_Report

    @brief   Send a HID report.

    @param   id - HID report ID.
    @param   type - HID report type.
    @param   len - Length of report.
    @param   pData - Report data.

    @return  None.
*/
void HidDev_Report( uint8 id, uint8 type, uint8 len, uint8* pData )
{
	#if( defined(EN_CONSUMER_MODE) && EN_CONSUMER_MODE)
		if( (gapRole_ConnectionHandle != INVALID_CONNHANDLE ) && \
			( HID_RPT_ID_CC_IN == id ) && (hidReportCCInClientCharCfg & GATT_CLIENT_CFG_NOTIFY ) )
		{
			attHandleValueNoti_t Notify;
			Notify.handle = hidAttrTbl[HID_CONSUMER_TABLE_IDX].handle;
			Notify.len = len;
			osal_memcpy(Notify.value, pData, len);
			GATT_Notification( gapRole_ConnectionHandle, &Notify, FALSE );
		}
	#endif
}

/*********************************************************************
    @fn      HidDev_Close

    @brief   Close the connection or stop advertising.

    @return  None.
*/
void HidDev_Close( void )
{
    #ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
}

/*********************************************************************
    @fn          HidDev_ReadAttrCB

    @brief       HID Dev attribute read callback.

    @param       connHandle - connection message was received on
    @param       pAttr - pointer to attribute
    @param       pValue - pointer to data to be read
    @param       pLen - length of data to be read
    @param       offset - offset of the first octet to be read
    @param       maxLen - maximum length of data to be read
    @param       method - type of read message

    @return      SUCCESS, blePending or Failure
*/
uint8 HidDev_ReadAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                         uint8* pValue, uint8* pLen, uint16 offset, uint8 maxLen )
{
    bStatus_t   status = SUCCESS;
//    hidRptMap_t* pRpt;
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d,uuid 0x%X\n",__FILE__,__func__,__LINE__,uuid);
	#endif
    // Only report map is long
    if ( offset > 0 && uuid != REPORT_MAP_UUID )
    {
        return ( ATT_ERR_ATTR_NOT_LONG );
    }
    if ( uuid == REPORT_UUID ||
            uuid == BOOT_KEY_INPUT_UUID ||
            uuid == BOOT_KEY_OUTPUT_UUID ||
            uuid == BOOT_MOUSE_INPUT_UUID )
    {
		// TODO 
        // find report ID in table
//        if ( (pRpt = hidDevRptByHandle(pAttr->handle)) != NULL )
//        {
//            // execute report callback
////            status  = (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
////                                              HID_DEV_OPER_READ, pLen, pValue );
//        }
//        else
//        {
//            *pLen = 0;
//        }
    }
    else if ( uuid == REPORT_MAP_UUID )
    {
        // verify offset
        if ( offset >= hidReportMapLen )
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else
        {
            // determine read length
            *pLen = MIN( maxLen, (hidReportMapLen - offset) );
            // copy data
            osal_memcpy( pValue, pAttr->pValue + offset, *pLen );
        }
    }
    else if ( uuid == HID_INFORMATION_UUID )
    {
        *pLen = HID_INFORMATION_LEN;
        osal_memcpy( pValue, pAttr->pValue, HID_INFORMATION_LEN );
    }
    else if ( uuid == GATT_REPORT_REF_UUID )
    {
        *pLen = HID_REPORT_REF_LEN;
        osal_memcpy( pValue, pAttr->pValue, HID_REPORT_REF_LEN );
    }
    else if ( uuid == PROTOCOL_MODE_UUID )
    {
        *pLen = HID_PROTOCOL_MODE_LEN;
        pValue[0] = pAttr->pValue[0];
    }
    else if ( uuid == GATT_EXT_REPORT_REF_UUID )
    {
        *pLen = HID_EXT_REPORT_REF_LEN;
        osal_memcpy( pValue, pAttr->pValue, HID_EXT_REPORT_REF_LEN );
    }
    return ( status );
}

/*********************************************************************
    @fn      HidDev_WriteAttrCB

    @brief   HID Dev attribute read callback.

    @param   connHandle - connection message was received on
    @param   pAttr - pointer to attribute
    @param   pValue - pointer to data to be written
    @param   len - length of data
    @param   offset - offset of the first octet to be written

    @return  Success or Failure
*/
bStatus_t HidDev_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                              uint8* pValue, uint8 len, uint16 offset )
{
    bStatus_t status = SUCCESS;
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d,offset %d\n",__FILE__,__func__,__LINE__,offset);
	#endif

    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
	#ifdef _PHY_DEBUG 
		LOG("	UUID 0x%X\n",uuid);
	#endif
    if ( uuid == REPORT_UUID || uuid == BOOT_KEY_OUTPUT_UUID )
    {
        // find report ID in table
//        if ((pRpt = hidDevRptByHandle(pAttr->handle)) != NULL)
//        {
//            // execute report callback
////            status  = (*pHidDevCB->reportCB)( pRpt->id, pRpt->type, uuid,
////                                              HID_DEV_OPER_WRITE, &len, pValue );
//        }
    }
	#if( defined( HID_SUP_CTRL_POINT_CHAR ) && HID_SUP_CTRL_POINT_CHAR)
    else if ( uuid == HID_CTRL_PT_UUID )
    {
    	#ifdef _PHY_DEBUG 
			LOG("	HID_CTRL_PT_UUID:ENTER OR EXIT SUSPEND\n");
		#endif
    }
	#endif
	
    else if ( uuid == GATT_CLIENT_CHAR_CFG_UUID )
    {
    	#ifdef _PHY_DEBUG 
			uint16 charCfg = BUILD_UINT16( pValue[0], pValue[1] );
			LOG("	GATT_CLIENT_CHAR_CFG_UUID:status %d,charCfg %d\n",status,charCfg);
		#endif

        if ( status == SUCCESS )
        {
			osal_memcpy(pAttr->pValue,pValue,2);
			#ifdef _PHY_DEBUG 
				LOG("	hidReportCCInClientCharCfg %d\n",hidReportCCInClientCharCfg);
			#endif
        }
    }
    else if ( uuid == PROTOCOL_MODE_UUID )
    {
        if ( len == HID_PROTOCOL_MODE_LEN )
        {
            if ( pValue[0] == HID_PROTOCOL_MODE_BOOT ||
                    pValue[0] == HID_PROTOCOL_MODE_REPORT )
            {
                pAttr->pValue[0] = pValue[0];
                // execute HID app event callback
//                (*pHidDevCB->evtCB)( (pValue[0] == HID_PROTOCOL_MODE_BOOT) ?
//                                     HID_DEV_SET_BOOT_EVT : HID_DEV_SET_REPORT_EVT );
            }
            else
            {
                status = ATT_ERR_INVALID_VALUE;
            }
        }
        else
        {
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
    }
    return ( status );
}

/*********************************************************************
    @fn      hidDev_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void hidDev_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case GATT_MSG_EVENT:
        hidDevProcessGattMsg( (gattMsgEvent_t*) pMsg );
        break;

    default:
        break;
    }
}

/*********************************************************************
    @fn      hidDevProcessGattMsg

    @brief   Process GATT messages

    @return  none
*/
static void hidDevProcessGattMsg( gattMsgEvent_t* pMsg )
{
}


/*********************************************************************
*********************************************************************/
