/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree tvoid SM_Init0( uint8 task_id )
o abide by the terms of these agreements. 
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

/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        
  Revision:       

  Description:    This file contains the Simple BLE Peripheral sample application
                  

**************************************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "bcomdef.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"
#include "ll.h"
#include "ll_def.h"
#include "hci_tl.h"
/*********************************************************************
 * MACROS
 */
//#define LOG(...)  
/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
//#define SBP_PERIODIC_EVT_PERIOD                   5000

//#define DEVINFO_SYSTEM_ID_LEN             8
//#define DEVINFO_SYSTEM_ID                 0
 

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     24//32//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800//48//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500//1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                    0xFFFF

// Default passcode
#define DEFAULT_PASSCODE                      0

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

//#define RESOLVING_LIST_ENTRY_NUM              10

//#define gpio_toggle

/*********************************************************************
 * TYPEDEFS
 */

/*********************************a************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */
//volatile uint8_t g_current_advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;

//extern wtnrTest_t wtnrTest;
//extern l2capSARDbugCnt_t g_sarDbgCnt;
//extern uint32 g_osal_mem_allo_cnt;
//extern uint32 g_osal_mem_free_cnt;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 application_TaskID;   // Task ID for internal task/event processing

//static gaprole_States_t gapProfileState = GAPROLE_INIT;
//static  uint8_t notifyBuf[256];
//static uint16 notifyInterval = 0;
//static uint8 notifyPktNum = 0;
//static uint8 connEvtEndNotify =0;
//static uint16 notifyCnt = 0;

//static uint16 disLatInterval = 0;
//static uint8 disLatTxNum = 0;
//static uint16 disLatCnt = 0;


// GAP - SCAN RSP data (max size = 31 bytes)


// GAP GATT Attributes
//static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "JACK-T- -FFFFFF ";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
//static void peripheralStateNotificationCB( gaprole_States_t newState );
//static void simpleProfileChangeCB( uint8 paramID );
//static void updateAdvData(void);
//static void peripheralStateReadRssiCB( int8 rssi  );

//static void initResolvingList(void);


//void check_PerStatsProcess(void);

//char *bdAddr2Str( uint8 *pAddr );
//static uint8_t simpleBLEPeripheral_ScanRequestFilterCBack(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
//static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
//{
//    peripheralStateNotificationCB,  // Profile State Change Callbacks
//    peripheralStateReadRssiCB       // When a valid RSSI is read from controller (not used by application)
//};

// GAP Bond Manager Callbacks, add 2017-11-15
//static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
//{
//  NULL,                     // Passcode callback (not used by application)
//  NULL                      // Pairing / Bonding state Callback (not used by application)
//};

// Simple GATT Profile Callbacks
//static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
//{
//    simpleProfileChangeCB    // Charactersitic value change callback
//};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
	application_TaskID = task_id;
	#if( defined( SERV_GGS ) && SERV_GGS )
	    GGS_AddService( );            // GAP
	#endif
	#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
	    GATTServApp_AddService( );    // GATT attributes
    #endif
	#if( defined( SERV_DEV_INFO ) && SERV_DEV_INFO )
	    DevInfo_AddService();                           // Device Information Service
    #endif
	simpleGATTProfile_AddService();					// simple GATT Profile Service

    // Setup a delayed profile startup
    osal_set_event( application_TaskID, SBP_START_DEVICE_EVT );
    // for receive HCI complete message
    GAP_RegisterForHCIMsgs(application_TaskID);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( application_TaskID )) != NULL )
        {
            simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        GAPRole_StartDevice( );

        // Start Bond Manager, 2017-11-15
        GAPBondMgr_Register( NULL );
		
        return ( events ^ SBP_START_DEVICE_EVT );
    }
    return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    hciEvt_CmdComplete_t *pHciMsg;
    #ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
    switch ( pMsg->event )
    {  
    	#ifdef _PHY_DEBUG 
			case GAP_MSG_EVENT:
				LOG("	Peripheral Bypass Message to Application\n");
				LOG("	Message Opcode 0x%X\n",((gapEventHdr_t *)pMsg)->opcode);
				if( GAP_DEVICE_INIT_DONE_EVENT == ((gapEventHdr_t *)pMsg)->opcode )
				{
					LOG("	Device Initiliaztion Done\n");
					LOG("	Please Start Application Logic...\n");
				}
			break;
		#endif
        case HCI_GAP_EVENT_EVENT:
        {
            switch( pMsg->status )
            {
                case HCI_COMMAND_COMPLETE_EVENT_CODE:
                    pHciMsg = (hciEvt_CmdComplete_t *)pMsg;
                
//                    LOG("==> HCI_COMMAND_COMPLETE_EVENT_CODE: %x\n", pHciMsg->cmdOpcode);
                    //safeToDealloc = gapProcessHCICmdCompleteEvt( (hciEvt_CmdComplete_t *)pMsg );
                break;


                default:
                    //safeToDealloc = FALSE;  // Send to app
                break;
            }
        }
      }

}
/*********************************************************************
 * @fn      peripheralStateReadRssiCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
//static void peripheralStateReadRssiCB( int8  rssi )
//{
////    notifyBuf[15]++;
////    notifyBuf[16]=rssi;
////    notifyBuf[17]=HI_UINT16(g_conn_param_foff);
////    notifyBuf[18]=LO_UINT16(g_conn_param_foff);;
////    notifyBuf[19]=g_conn_param_carrSens;
//}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
//static void peripheralStateNotificationCB( gaprole_States_t newState )
//{
//    switch ( newState )
//    {
//        case GAPROLE_STARTED:
//        {
//            uint8 ownAddress[B_ADDR_LEN];
//            uint8 str_addr[14]={0}; 
//            uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
//            uint8 initial_advertising_enable = FALSE;//true
//        
//            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
//        
//            // use 6 bytes of device address for 8 bytes of system ID value
//            systemId[0] = ownAddress[0];
//            systemId[1] = ownAddress[1];
//            systemId[2] = ownAddress[2];
//        
//            // set middle bytes to zero
//            systemId[4] = 0x00;
//            systemId[3] = 0x00;
//        
//            // shift three bytes up
//            systemId[7] = ownAddress[5];
//            systemId[6] = ownAddress[4];
//            systemId[5] = ownAddress[3];
//        
////            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
//
//
//            osal_memcpy(&str_addr[0],bdAddr2Str(ownAddress),14);
//            osal_memcpy(&scanRspData[11],&str_addr[6],8);
//            osal_memcpy(&attDeviceName[9],&str_addr[6],8);
//        
//
//            GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
//            // Set the GAP Characteristics
////            GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
//
//            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
//
//			//osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT, 500);    
//			osal_set_event(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT);    
//                                    
//        }
//            break;
//        
//        case GAPROLE_ADVERTISING:
//        {
//            osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
//            notifyCnt=0;
//            notifyInterval = 0;
//
//            osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_DISABLE_LATENCY_TEST_EVT);
//            disLatInterval = 0;
//            disLatCnt = 0;
//            disLatTxNum=0;
//            
//         }   
//            break;
//        
//       case GAPROLE_CONNECTED:
// #if (PHY_MCU_TYPE == MCU_BUMBEE_M0)
//            HCI_PPLUS_ConnEventDoneNoticeCmd(simpleBLEPeripheral_TaskID, NULL);
//            
// #endif /*#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)*/
//           break;
//        
//        case GAPROLE_CONNECTED_ADV:
//            break;      
//        case GAPROLE_WAITING:
//            break;
//        
//        case GAPROLE_WAITING_AFTER_TIMEOUT:
//            break;
//        
//        case GAPROLE_ERROR:
//            break;
//        
//        default:
//            break;        
//    }  
//    gapProfileState = newState;
//
//    LOG("[GAP ROLE %d]\n",newState);
//     
//    VOID gapProfileState;     
//}


/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
//static void simpleProfileChangeCB( uint8 paramID )
//{
//  	uint8 newValue[250];
//    
////	switch( paramID )
////	{
////		case SIMPLEPROFILE_CHAR5:
////			SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, newValue );
////			LOG("[WRT_ATT] %02x \n",newValue[0]);
////			if( newValue[0] == 0x01 )
////				osal_start_reload_timer( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, 1000 );
////		break;
////	}
//}


/*********************************************************************
 * @fn      updateAdvData
 *
 * @brief   update adv data and change the adv type
 *
 * @param   none
 *
 * @return  none
 */
//static void updateAdvData(void)
//{
//    uint8  new_uuid[IBEACON_UUID_LEN];
//    uint16  major;
//    uint16  minor;
//    uint8   power;
//    
//    // 1. get the new setting from GATT attributes
//    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, new_uuid );
//    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2, &major );
//    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &minor );
//    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR4, &power );	  
//    
//    // 2. update adv data storage
//    //set UUID
//    VOID osal_memcpy(&advertData[9], new_uuid, IBEACON_UUID_LEN);
//    // set major
//    advertData[25] = LO_UINT16( major );
//    advertData[26] = HI_UINT16( major );	  
//    // set minor	  
//    advertData[27] = LO_UINT16( minor );
//    advertData[28] = HI_UINT16( minor );	
//    // set power
//    advertData[29] = power;
//	
//    // 3. disconnect all connection
//    GAPRole_TerminateConnection();
//		
//    // 4. close advert
//    uint8 initial_advertising_enable = FALSE;		
//    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );		
//        
//    // 5. update adv data
//    // 5.1 update adv type
//    uint8 advType = g_current_advType;    
//    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );	  
//
////    uint16 advInt = otaAdvIntv<<4;
////    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
////    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
////    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
////    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
//
//    // 5.2 update advert broadcast
//    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );	
//
//    // 5.3 set TxPower
//    g_rfPhyTxPower = power;
//    rf_phy_set_txPower(power);
//
//    // 6. set reset advertisement event, note that GAP/LL will process close adv event in advance
//    osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT,5000);    
//}

/*********************************************************************
* @fn      bdAddr2Str
*
* @brief   Convert Bluetooth address to string. Only needed when
*          LCD display is used.
*
* @return  none
*/
//char *bdAddr2Str( uint8 *pAddr )
//{
//  uint8       i;
//  char        hex[] = "0123456789ABCDEF";
//  static char str[B_ADDR_STR_LEN];
//  char        *pStr = str;
//  
//  *pStr++ = '0';
//  *pStr++ = 'x';
//  
//  // Start from end of addr
//  pAddr += B_ADDR_LEN;
//  
//  for ( i = B_ADDR_LEN; i > 0; i-- )
//  {
//    *pStr++ = hex[*--pAddr >> 4];
//    *pStr++ = hex[*pAddr & 0x0F];
//  }
//  
//  *pStr = 0;
//  
//  return str;
//}



/*********************************************************************
*********************************************************************/
