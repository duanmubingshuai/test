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
#include "osal_bufmgr.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "simplePrivateCentral.h"
#include "simpleGATTprofile.h"
#include "hci_tl.h"

#include "usb_hal_pcd.h"
#include "usb_pcd.h"
#include "usb_usr_config.h"
#include "hid.h"
/*********************************************************************
 * MACROS
 */
 #define DEFAULT_CONN_INTV_MIN  2
 #define DEFAULT_CONN_LATENCY   256
 #define DEFAULT_CONN_TIMEOUT   251
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 bt_peer_addr[6]={0};
uint8 bt_peer_addr0[6]={0x49,0x49,0x49,0x49,0x70,0x70};
uint8 bt_peer_addr1[6]={0x49,0x49,0x49,0x49,0x71,0x71};

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 g_advRptCnt;
extern uint8 g_advRptNum;
extern usb_hal_pcd_t pcd_handle;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void LL_ExtClearBER_Buffer();
extern uint8 LL_ExtGet_RecommandChanMap(uint16 connHandle , uint8 *rstChanMap,\
										uint32 statis_TimeMs ,uint8 BER_Percent );
extern uint8 ll_ext_chanMap_procedure( uint16 connHandle,uint8 *chanMap );
extern hciStatus_t HCI_LE_CreateConnCmd( uint16 scanInterval,
                                  uint16 scanWindow,
                                  uint8  initFilterPolicy,
                                  uint8  addrTypePeer,
                                  uint8  *peerAddr,
                                  uint8  ownAddrType,
                                  uint16 connIntervalMin,
                                  uint16 connIntervalMax,
                                  uint16 connLatency,
                                  uint16 connTimeout,
                                  uint16 minLen,
                                  uint16 maxLen );
/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 application_TaskID;   // Task ID for internal task/event processing
uint8 app_linkCnt = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simplePrivateCentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );

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

#if 0
int test_key_cnt = 0;
uint32_t mouse_intv = 20;
signed char const test_key_value[4][2] = {
  {50,0},
  {0,50},
  {-50,0},
  {0, -50},
};
#endif

 static void usb_irq_evt(void)
{
    osal_set_event(application_TaskID, usbDeviceTx_EVT);
}

void SimplePrivateCentral_Init( uint8 task_id )
{
    application_TaskID = task_id;

    HCI_GAPTaskRegister( task_id );
    HCI_L2CAPTaskRegister(task_id);

#if 0
    uint32_t mask = 0;
    uint32_t intv_table[4] = {10, 8, 5, 1};
    gpio_dir(P9, GPIO_INPUT);
    gpio_dir(P11, GPIO_INPUT);
    gpio_pull_set(P9, GPIO_PULL_UP);
    gpio_pull_set(P11, GPIO_PULL_UP);

    if(gpio_read(P9) == 0) mask |= BIT(0);
    if(gpio_read(P11) == 0) mask |= BIT(1);
    LOG("mask is %x\n", mask);
    mouse_intv = intv_table[mask];
#endif

#ifdef DEF_USB_HID
    pcd_init(usb_irq_evt);
#endif
    // Setup a delayed profile startup
    osal_set_event( application_TaskID, SPC_START_DEVICE_EVT );
    //osal_set_event( application_TaskID, usbDeviceTask_EVT );
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
//static int s_tmp_cnt;
uint16 SimplePrivateCentral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( application_TaskID )) != NULL )
        {
            simplePrivateCentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SPC_START_DEVICE_EVT )
    {
//    	LOG_ERROR("scan...\n");
        // Start the Device
        g_advRptNum = 1;
        HCI_LE_SetScanParamCmd( FALSE, 240, 24,0,0 );
        HCI_LE_SetScanEnableCmd(TRUE,0);
        return ( events ^ SPC_START_DEVICE_EVT );
    }
    if( events & SPC_START_ESTABLISH_EVT )
    {
	    
    	{
			/*uint8 ret = */HCI_LE_CreateConnCmd( 24, 24, 0,0,bt_peer_addr,0,DEFAULT_CONN_INTV_MIN,\
																		DEFAULT_CONN_INTV_MIN,\
																		DEFAULT_CONN_LATENCY,\
																		DEFAULT_CONN_TIMEOUT,0, 0);
			LOG_DEBUG("start establish...,ret = %d\n",ret);
    	}
//		else
//			LOG_DEBUG("already max conn num\n");
        return (  events ^ SPC_START_ESTABLISH_EVT);
    }
#ifdef DEF_USB_HID
#if 0
    if( events & usbDeviceTask_EVT )
    {
        uint32_t mouse_intv = 20;
//      LOG("int1 = %x\n", (* (volatile int *) 0xe000e100));
//      LOG("int2 = %x\n", (* (volatile int *) 0xe000e104));
        struct hid_mouse_report  mouse_report;
        osal_start_timerEx(application_TaskID, usbDeviceTask_EVT, mouse_intv);
        //mouse_report.report_id      = 2;
        //mouse_report.ac_pan         = 0;
        mouse_report.button         = 0;
        mouse_report.ctl.b.X        = test_key_value[test_key_cnt%4][0];
        mouse_report.ctl.b.Y        = test_key_value[test_key_cnt%4][1];
        mouse_report.ctl.b.Wheel    = 0;
        test_key_cnt++;

        hal_pcd_send_data(&pcd_handle, 1, (uint8_t *)&mouse_report, 8);
        return (events ^ usbDeviceTask_EVT);
    }
#endif
	if( events &  SPC_GHANGE_CHANMAP_EVT )
	{
//		uint8 chanMap[5];
//		uint8 rst = LL_ExtGet_RecommandChanMap(0 , chanMap,10000 ,20 );
//		if( rst )
//		{
//			rst = ll_ext_chanMap_procedure(0,chanMap );
////			LOG_ERROR("rst %d,chanMap:%02X,%02X,%02X,%02X,%02X\n",rst,chanMap[0],chanMap[1],chanMap[2],chanMap[3],chanMap[4]);			
//		}
		return (events ^ SPC_GHANGE_CHANMAP_EVT);
	}
	if ( events & usbDeviceTx_EVT )
    {

//        usb_hid_tx_send(&pcd_handle);
//        return (events ^ usbDeviceTx_EVT);
	}
#endif

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
static void simplePrivateCentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
//  LOG_DEBUG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
//  LOG_DEBUG("pMsg->event 0x%X\n",pMsg->event);
    switch ( pMsg->event )
    {  
        case HCI_GAP_EVENT_EVENT:
        {
            if( pMsg->status == HCI_COMMAND_COMPLETE_EVENT_CODE)
            {
//              LOG_DEBUG("cmd complement...\n");
            }
            else if( pMsg->status == HCI_LE_EVENT_CODE)
            {
                hciEvt_BLEConnComplete_t *pkt = (hciEvt_BLEConnComplete_t *)pMsg;
                if( pkt->BLEEventCode == HCI_BLE_CONNECTION_COMPLETE_EVENT )
                {
                	app_linkCnt++;
//                    LOG_ERROR("Established...\n");
					LL_ExtClearBER_Buffer();
					osal_start_reload_timer(application_TaskID, SPC_PERIODIC_EVT, 1000);
					if( app_linkCnt < APP_CFG_MAX_CONN_NUM )
                    	osal_start_timerEx(application_TaskID, SPC_START_DEVICE_EVT, 100);
//					osal_start_reload_timer(application_TaskID, SPC_GHANGE_CHANMAP_EVT, 10000);
					
                }
                else if( pkt->BLEEventCode == HCI_BLE_ADV_REPORT_EVENT  )
                {
                    hciEvt_BLEAdvPktReport_t *advPkt = (hciEvt_BLEAdvPktReport_t *)pMsg;
                    if( g_advRptCnt > 0 )
                        g_advRptCnt--;
                    if(( TRUE == osal_memcmp(bt_peer_addr0, advPkt->devInfo->addr, B_ADDR_LEN) ) || \
						( TRUE == osal_memcmp(bt_peer_addr1, advPkt->devInfo->addr, B_ADDR_LEN) ))
                    {
						LOG_DUMP_BYTE(advPkt->devInfo->addr,B_ADDR_LEN);
                    	osal_memcpy(bt_peer_addr, advPkt->devInfo->addr, B_ADDR_LEN );
                        HCI_LE_SetScanEnableCmd(FALSE,0);
                        osal_start_timerEx(application_TaskID, SPC_START_ESTABLISH_EVT, 100);
                    }
                }
            }
            else if( pMsg->status == HCI_DISCONNECTION_COMPLETE_EVENT_CODE)
            {
                hciEvt_DisconnComplete_t *pkt = (hciEvt_DisconnComplete_t *)pMsg;
                if( pkt->hdr.status == HCI_DISCONNECTION_COMPLETE_EVENT_CODE )
                {
                	app_linkCnt--;
					osal_stop_timerEx(application_TaskID, SPC_PERIODIC_EVT);
//					osal_stop_timerEx(application_TaskID, SPC_GHANGE_CHANMAP_EVT);
					HCI_LE_SetScanEnableCmd(FALSE,0);
					if( app_linkCnt < APP_CFG_MAX_CONN_NUM )
                    	osal_start_timerEx(application_TaskID, SPC_START_DEVICE_EVT, 100);
                    LOG_ERROR("Disconnect...\n");
                    
                }
            }
            else
            {
//              LOG_DEBUG("unKnown pMsg->status 0x%X\n",pMsg->status);
            }
        }
        break; 
        case HCI_DATA_EVENT:
        {
            hciDataEvent_t *msg  = (hciDataEvent_t *)pMsg;
#ifdef DEF_USB_HID
            struct hid_mouse_report  mouse_report;
            usb_hal_pcd_t *hpcd = &pcd_handle;

            // LOG_DUMP_BYTE(msg->pData,msg->len);
            // log_printf("\r");

            //mouse_report.report_id      = msg->pData[2];
            //mouse_report.ac_pan         = msg->pData[3];
            mouse_report.button         = msg->pData[4];
            mouse_report.ctl.b.X        = (int8)msg->pData[5];
            mouse_report.ctl.b.Y        = (int8)msg->pData[6];
            mouse_report.ctl.b.Wheel    = msg->pData[7];

            if(mouse_report.button && hid_remote_wakeup_rdy == HID_REMOTE_WAKEUP_READY)
            {
                hid_remote_wakeup_rdy = HID_REMOTE_WAKEUP_GOING;
				extern void hal_remote_wakeup(usb_hal_pcd_t *hpcd);
                hal_remote_wakeup(&pcd_handle);
            }
            
            hal_pcd_send_data(hpcd, 1, (uint8_t *)&mouse_report, 8);
#endif
			osal_bm_free(msg->pData);
        }
        break;
      }
}

//hciEvt_BLEConnComplete_t pkt->hdr.event     = HCI_GAP_EVENT_EVENT;
//pkt->hdr.status   = HCI_LE_EVENT_CODE;
//pkt->BLEEventCode = HCI_BLE_CONNECTION_COMPLETE_EVENT;
//
//hciEvt_DisconnComplete_t *pkt;
//pkt = (hciEvt_DisconnComplete_t *)osal_msg_allocate( sizeof ( hciEvt_DisconnComplete_t ) );
//if ( pkt )
//{
//pkt->hdr.event  = HCI_GAP_EVENT_EVENT;
//pkt->hdr.status = HCI_DISCONNECTION_COMPLETE_EVENT_CODE;
//pkt->status      = HCI_SUCCESS;
//pkt->connHandle = connHandle;
//pkt->reason      = reasonCode;


/*********************************************************************
*********************************************************************/
