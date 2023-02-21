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
#include "simplePrivatePeripheral.h"
#include "simpleGATTprofile.h"
#include "hci_tl.h"
#include "osal_bufmgr.h"
/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************a************************************
 * GLOBAL VARIABLES
 */
uint32 tx_failure_cnt = 0;
uint32 tx_malloc_failure_cnt = 0;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern llStatus_t LL_PLUS_DisableSlaveLatency(uint8 connId);
extern llStatus_t LL_PLUS_EnableSlaveLatency(uint8 connId);

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 application_TaskID;   // Task ID for internal task/event processing

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simplePrivatePeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );

/*********************************************************************
 * PROFILE CALLBACKS
 */

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
void SimplePrivatePeripheral_Init( uint8 task_id )
{
	application_TaskID = task_id;
	
	LOG("SimpleBLEPeripheral_Init\n");

	HCI_GAPTaskRegister( task_id );
	HCI_L2CAPTaskRegister(task_id);
//	#if( defined( SERV_GGS ) && SERV_GGS )
//	    GGS_AddService( );            // GAP
//	#endif
//	#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
//	    GATTServApp_AddService( );    // GATT attributes
//    #endif
//	#if( defined( SERV_DEV_INFO ) && SERV_DEV_INFO )
//	    DevInfo_AddService();                           // Device Information Service
//    #endif
//	simpleGATTProfile_AddService();					// simple GATT Profile Service
//	
//	uint16 advInt = 32;//2400;//1600;//1600;//800;//1600;   // actual time = advInt * 625us
//	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
//	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
//	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
//	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
		

    // Setup a delayed profile startup
    osal_set_event( application_TaskID, SBP_START_DEVICE_EVT );
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
 
 uint8 hidMouseReport_test()
{
	uint8 ret = FALSE;
//    if( app_mode == APP_2P4G_MODE)
    {
        uint8  rslt;
        uint8 *data = (uint8 *)HCI_bm_alloc(20);
        if ( data ) {
            osal_memset(data, 0, 10);
            data[0] = 0;
            data[1] = 0;
			data[2] = 2;
			data[3] = 0;
            data[4] = 0;//hidMouse_data.button;
            data[5] = 0;//hidMouse_data.x;// x;
            data[6] = 0;//hidMouse_data.y;//y;
            data[7] = 0;//hidMouse_data.wheel;
            // data[8] = 0x0;
//
//            if ( hidMouse_data.wheel != 0x00 ) {
//                hidMouse_data.wheel = 0;
//                data[5] = 0;// move wheel clear xy
//                data[6] = 0;//
//            }

            rslt = HCI_SendDataPkt(0,0,10,(uint8 *)data);
            if ( rslt != SUCCESS ) {
                // HCI_SendDataPkt(0,0,10,(int8 *)data);
                osal_bm_free(data);
                // LOG_ERROR("HCI_SendDataPkt, rslt:%d\n",rslt);
            }else{
				ret = TRUE;
			}
        } else {
//            LOG_ERROR("!! HCI_bm_alloc, rslt:0 !!\n");
        }
    }
	return ret;
}

//static int s_tmp_cnt;
uint32 app_tx_start = 0;

uint16 SimplePrivatePeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( application_TaskID )) != NULL )
        {
            simplePrivatePeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
            // Release the OSAL message
            /*uint8 ret = */osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
//		LOG("SBP_START_DEVICE_EVT\n");
        // Start the Device
		HCI_LE_SetAdvParamCmd( 0x80, 0x80, 0,0,0,NULL,0x07,0);

		uint8 adv_data[]={0x02,0x01,0x06};
		uint8 scanRspData[] ={	0x0C, 0x09,'P','I','C','O','-','s','p','p','0','0','0'};
		HCI_LE_SetAdvDataCmd( sizeof(adv_data), adv_data );
		HCI_LE_SetScanRspDataCmd( sizeof(scanRspData),scanRspData);
		HCI_LE_SetAdvEnableCmd(TRUE);

//		osal_start_reload_timer(application_TaskID,SBP_PERIODIC_EVT,1000);
        return ( events ^ SBP_START_DEVICE_EVT );
    }
    if( events & SBP_TEST_DLL )
	{		
       uint8 ret = LL_PLUS_DisableSlaveLatency(0);
		if( ret == 0 )
		{
//			LOG_ERROR("Dis\n");
			app_tx_start = read_current_fine_time();

			osal_start_reload_timer(application_TaskID, SBP_PERIODIC_EVT, 2);
		}
		else
		{
//			LOG_ERROR("ret %X\n",ret);
			LL_PLUS_EnableSlaveLatency(0);

			osal_start_timerEx(application_TaskID, SBP_TEST_DLL, 5);
		}

		return ( events ^ SBP_TEST_DLL );
	}
    
	if( events & SBP_STATISTICS_EVT )
	{		
//		LOG("ntfEt %d,txmf %d\n",tx_failure_cnt,tx_malloc_failure_cnt);
//		HCI_DisconnectCmd(0,0x16);
		extern uint32 pIrqCnt ;
		extern uint32 pRtoCnt ;
		extern uint32 pCerrCnt ;
		extern uint32 pCokCnt ;
		extern uint32 linkCnt;
		LOG_ERROR("linkCnt-%d,irqCnt - %d,to-%d,err-%d,ok-%d,OKR-%d/%d\n",linkCnt,pIrqCnt,pRtoCnt,pCerrCnt,pCokCnt,pCokCnt,pIrqCnt);
		pIrqCnt = pRtoCnt = pCerrCnt = pCokCnt = 0;
          
		return ( events ^ SBP_STATISTICS_EVT );
	}
	if( events & SBP_PERIODIC_EVT )
	{
		static uint16 txPktCnt = 0;
		while(1)
		{
			if( TRUE == hidMouseReport_test() )
			{
				txPktCnt++;
				if( txPktCnt > 1000 )
				{
					txPktCnt = 0;
					osal_stop_timerEx(application_TaskID, SBP_PERIODIC_EVT);
					osal_start_timerEx(application_TaskID,SBP_ENABLE_LATENCY_EVT,5);
				}
			}else{
//				gpio_write(P13,1);
//				gpio_write(P13,0);
				break;
			}
		}
		return (  events ^ SBP_PERIODIC_EVT);
	}
	if( events & SBP_ENABLE_LATENCY_EVT )
    {
		/*static uint8 cnt = 0;*/
		/*uint8 ret = */LL_PLUS_EnableSlaveLatency(0);
//		LOG_ERROR("En-%d\n",ret);
		LOG_ERROR("sT-%d\n",LL_TIME_DELTA( app_tx_start ,read_current_fine_time()));
		osal_start_timerEx(application_TaskID,SBP_TEST_DLL,2000);	
        return (  events ^ SBP_ENABLE_LATENCY_EVT);
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
static void simplePrivatePeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
//	LOG_DEBUG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
//	LOG_DEBUG("pMsg->event 0x%X\n",pMsg->event);
    switch ( pMsg->event )
    {  
		case HCI_GAP_EVENT_EVENT:
		{
			if( pMsg->status == HCI_COMMAND_COMPLETE_EVENT_CODE)
			{
			}
			else if( pMsg->status == HCI_LE_EVENT_CODE)
			{
				hciEvt_BLEConnComplete_t *pkt = (hciEvt_BLEConnComplete_t *)pMsg;
				if( pkt->BLEEventCode == HCI_BLE_CONNECTION_COMPLETE_EVENT )
				{
					tx_failure_cnt = 0;
					tx_malloc_failure_cnt = 0;
					osal_start_timerEx(application_TaskID, SBP_TEST_DLL, 2000);
					osal_start_reload_timer(application_TaskID, SBP_STATISTICS_EVT, 10000);
					
					static uint32 re_connect_cnt = 0;
					extern uint32 linkCnt;
					linkCnt++;
					LOG_DEBUG("Established...rCnt %d\n",++re_connect_cnt);
				}
			}
			else if( pMsg->status == HCI_DISCONNECTION_COMPLETE_EVENT_CODE)
			{
				hciEvt_DisconnComplete_t *pkt = (hciEvt_DisconnComplete_t *)pMsg;
				if( pkt->hdr.status == HCI_DISCONNECTION_COMPLETE_EVENT_CODE )
				{
					LOG_DEBUG("Disconnect...\n")
					osal_stop_timerEx(application_TaskID, SBP_TEST_DLL);
					osal_stop_timerEx(application_TaskID, SBP_PERIODIC_EVT);
					osal_stop_timerEx(application_TaskID, SBP_STATISTICS_EVT);
					HCI_LE_SetAdvEnableCmd(TRUE);
				}
			}
			else
			{
				LOG_DEBUG("unKnown pMsg->status 0x%X\n",pMsg->status);
			}
		}
		break; 
		case HCI_DATA_EVENT:
		{
			hciDataEvent_t *msg  = (hciDataEvent_t *)pMsg;
//			LOG_DUMP_BYTE(msg->pData,msg->len);
			osal_bm_free(msg->pData);
		}
		break;
      }
}

//hciEvt_BLEConnComplete_t pkt->hdr.event	  = HCI_GAP_EVENT_EVENT;
//pkt->hdr.status   = HCI_LE_EVENT_CODE;
//pkt->BLEEventCode = HCI_BLE_CONNECTION_COMPLETE_EVENT;
//
//hciEvt_DisconnComplete_t *pkt;
//pkt = (hciEvt_DisconnComplete_t *)osal_msg_allocate( sizeof ( hciEvt_DisconnComplete_t ) );
//if ( pkt )
//{
//pkt->hdr.event  = HCI_GAP_EVENT_EVENT;
//pkt->hdr.status = HCI_DISCONNECTION_COMPLETE_EVENT_CODE;
//pkt->status	   = HCI_SUCCESS;
//pkt->connHandle = connHandle;
//pkt->reason	   = reasonCode;


/*********************************************************************
*********************************************************************/
