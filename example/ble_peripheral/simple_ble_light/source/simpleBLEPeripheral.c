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
#include "simpleBLELight.h"
#include "hci_tl.h"
#include "pwm_light.h"
#include "phy_plus_phy.h"

/*********************************************************************
 * MACROS
 */
#define SNRF_PREPARED_ACKPDU_NUM    8

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************a************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 application_TaskID;   // Task ID for internal task/event processing
phy_comm_evt_t s_prepared_ackpdu[SNRF_PREPARED_ACKPDU_NUM];
uint8_t test_prepared_ackpdu[32] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );

uint8_t Smart_nRF_data_process(phy_comm_evt_t *pdata);
uint8_t Smart_nRF_generate_ackpdu(phy_comm_evt_t *packbuf);

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
void SimpleBLEPeripheral_Init( uint8 task_id )
{
	application_TaskID = task_id;

    /*register phy_plus_phy CB func*/
    phy_cbfunc_regist(PHY_DATA_CB,Smart_nRF_data_process);
	#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)    
    phy_cbfunc_regist(PHY_OPCODE_CB,Smart_nRF_generate_ackpdu);
	
	phy_comm_evt_t a =
	{
		.type = 1,
		.len = 31,
		.data = test_prepared_ackpdu,
		.rssi = 0,
	};
	s_prepared_ackpdu[0] = a;
    #endif

	LOG("SimpleBLEPeripheral_Init\n");
	#if( defined( SERV_GGS ) && SERV_GGS )
	    GGS_AddService( );            // GAP
	#endif
	#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
	    GATTServApp_AddService( );    // GATT attributes
    #endif
	#if( defined( SERV_DEV_INFO ) && SERV_DEV_INFO )
	    DevInfo_AddService();                           // Device Information Service
    #endif
	simpleLight_AddService();					// simple GATT Profile Service
	
	uint16 advInt = 400;//2400;//1600;//1600;//800;//1600;   // actual time = advInt * 625us
	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
	GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
	GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
		

    // Setup a delayed profile startup
    osal_set_event( application_TaskID, SBP_START_DEVICE_EVT );
}
static uint32_t cnt=0;
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
		LOG("SBP_START_DEVICE_EVT\n");
        // Start the Device
        GAPRole_StartDevice( );
        // Start Bond Manager
        GAPBondMgr_Register( NULL );
        //osal_set_event(application_TaskID,SBP_PERIODIC_EVT);
        return ( events ^ SBP_START_DEVICE_EVT );
    }
#if 0
    if(events & SBP_PERIODIC_EVT)
    {
        LOG("cnt %d rtcCnt %08x\n",cnt++,rtc_get_counter());
        if(cnt&0x01)
            gpio_pull_set(P9,GPIO_PULL_DOWN);
        else
            gpio_pull_set(P9,GPIO_PULL_UP);
        

        osal_start_timerEx(application_TaskID,SBP_PERIODIC_EVT,1000);
        return (events ^ SBP_PERIODIC_EVT);
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
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
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

      }
}

/*********************************************************************
*********************************************************************/

uint8_t Smart_nRF_data_process(phy_comm_evt_t *pdata)
{
    #if(DEF_PHYPLUS_AUTOACK_SUPPORT==1)

       if(pdata->len==3+6   &&
           pdata->data[6]==0xff &&
           pdata->data[7]==0xf1)
           {
               pwmlight_phy_control(pdata->data[8]);
           }
    if(PHYPLUS_GET_ACK_BIT(pdata->type))
    {
        if(pdata->len != NULL &&
            pdata->data != NULL)
        {
            // LOG_DEBUG("It's CB ack:");
            // my_dump_byte(pdata->data,pdata->len);
        }
    }
    else
    {
        // LOG_DEBUG("It's CB data:");
        // my_dump_byte(pdata->data,pdata->len);
    }
    #endif
	return PPlus_SUCCESS;
}

#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)    
uint8_t Smart_nRF_generate_ackpdu(phy_comm_evt_t *packbuf)
{
    uint8_t opcode = PHYPLUS_GET_OPCODE(packbuf->type);
    if((opcode > SNRF_PREPARED_ACKPDU_NUM) || (s_prepared_ackpdu[opcode-1].len > PHYPLUS_ACK_DATA_MAX_NUM))
    {
        return PPlus_ERR_INVALID_PARAM;
    }
    else if(s_prepared_ackpdu[opcode-1].type == NULL)
    {
        return PPlus_ERR_NULL;
    }
    else
    {
        packbuf->len = s_prepared_ackpdu[opcode-1].len;
        osal_memcpy(packbuf->data, s_prepared_ackpdu[opcode-1].data, s_prepared_ackpdu[opcode-1].len);
    }
		return PPlus_SUCCESS;
}
#endif
