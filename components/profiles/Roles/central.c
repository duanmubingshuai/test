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

/**************************************************************************************************
  Filename:       peripheral.c
  Revised:         
  Revision:        

  Description:    GAP Peripheral Role


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "ll_common.h"
#include "bcomdef.h"
#include "OSAL.h"
#include "hci_tl.h"
#include "l2cap.h"
#include "gap.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "gatt_profile_uuid.h"

/*********************************************************************
 * MACROS
 */
#define GAP_CONFIG_STATIC_ADDR					FALSE

#define GAP_PROFILE_ROLE						( GAP_PROFILE_CENTRAL )

#define GAP_DEFAULT_MSG_BYPASS_ENABLE			FALSE
#if(defined(GAP_DEFAULT_MSG_BYPASS_ENABLE) && GAP_DEFAULT_MSG_BYPASS_ENABLE)
extern uint8 application_TaskID;
#define GAP_DEFAULT_MSG_BYPASS_ID				(application_TaskID)
#else
#define GAP_DEFAULT_MSG_BYPASS_ID				(gapRole_TaskID)
#endif

/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
	minit = 1,
	mlink,
	mlink_err,
	mterm
}GAPCentLinkMode;

/*********************************************************************
 * CONSTANTS
 */
// Profile Events
//#define START_CONN_UPDATE_EVT         0x0004  // Start Connection Update Procedure
//#define CONN_PARAM_TIMEOUT_EVT        0x0008  // Connection Parameters Update Timeout

/*********************************************************************
 * GLOBAL FUNCTIONS
 */
void gapCentProcessAdvRptMsg(hciEvt_BLEAdvPktReport_t* pMsg);


/*********************************************************************
 * GLOBAL VARIABLES
 */
#if(defined(GAP_CONFIG_STATIC_ADDR) && GAP_CONFIG_STATIC_ADDR)
uint8 BD_STATIC_ADDR[B_ADDR_LEN]={0x30,0x31,0x32,0x33,0x34,0xC1};
#endif
static uint8 cLinDevAddr[3]={0};

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 g_advRptNum;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern void gapCentProcessHCICmdEvt( uint16 cmdOpcode, hciEvt_CmdComplete_t *pMsg );
extern bStatus_t gattFindInfo( uint16 connHandle, attFindInfoReq_t *pReq, uint8 taskId );


/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 gapRole_TaskID;   // Task ID for internal task/event processing
static uint8 gapRole_linkCnt = 0;

/*********************************************************************
 * Profile Parameters - reference GAPROLE_PROFILE_PARAMETERS for
 * descriptions
 */
//uint16 gapRole_ConnectionHandle = INVALID_CONNHANDLE;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void gapRole_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void gapRole_ProcessGAPMsg( gapEventHdr_t *pMsg );
static void gapRole_SetupGAP( void );
static void gapRole_SchLinkDev( GAPCentLinkMode mode,uint16 connHandle );

/*********************************************************************
 * @brief   Does the device initialization.
 *
 * Public function defined in peripheral.h.
 */
bStatus_t GAPRole_StartDevice( )
{
    // Start the GAP
    gapRole_SetupGAP();
    return ( SUCCESS );
}

/*********************************************************************
 * @brief   Task Initialization function.
 *
 * Internal function defined in peripheral.h.
 */
void GAPRole_Init( uint8 task_id )
{
	gapRole_TaskID = task_id;
	GATT_InitClient();
	g_advRptNum = 5;
	GAP_RegisterForHCIMsgs(gapRole_TaskID);
}

/*********************************************************************
 * @brief   Task Event Processor function.
 *
 * Internal function defined in peripheral.h.
 */
#define GAP_START_SCAN_EVT			0X0001
uint16 GAPRole_ProcessEvent( uint8 task_id, uint16 events )
{
	VOID task_id; // OSAL required parameter that isn't used in this function

	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;

		if ( (pMsg = osal_msg_receive( gapRole_TaskID )) != NULL )
		{
			gapRole_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

			// Release the OSAL message
			osal_msg_deallocate( pMsg );
		}
		return (events ^ SYS_EVENT_MSG);
	}

	if( GAP_START_SCAN_EVT & events )
	{
		gapDevDiscReq_t params;
		params.taskID = gapRole_TaskID;
		params.mode = DEVDISC_MODE_ALL;
		params.activeScan = TRUE;
		params.whiteList = FALSE;
		uint8 ret = GAP_DeviceDiscoveryRequest(&params);
		PRINT("GAP_DeviceDiscoveryRequest RET 0x%X\n",ret);
		return (events ^ GAP_START_SCAN_EVT);
	}
	return 0;
}

/*********************************************************************
 * @fn      gapRole_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void gapRole_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
//	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
//	#endif
#if(defined(GAP_DEFAULT_MSG_BYPASS_ENABLE) && GAP_DEFAULT_MSG_BYPASS_ENABLE)
	// bypass
#else
	switch ( pMsg->event )
	{
		case GAP_MSG_EVENT:
			gapRole_ProcessGAPMsg( (gapEventHdr_t *)pMsg );
		break;
		case HCI_GAP_EVENT_EVENT:
		{
			switch( pMsg->status )
			{
				case HCI_COMMAND_COMPLETE_EVENT_CODE :
				{
					hciEvt_CmdComplete_t *pPkt = (hciEvt_CmdComplete_t *)pMsg;
					#ifdef _PHY_DEBUG 
						LOG("	HCI_COMMAND_COMPLETE_EVENT_CODE:pPkt->cmdOpcode 0x%X\n",pPkt->cmdOpcode);
					#endif					
//						if ( pPkt->cmdOpcode == HCI_READ_RSSI )
//						{
//							LOG("	Read RSSI %d\n",(int8)pPkt->pReturnParam[3] );
//						}
//						else if(pPkt->cmdOpcode == HCI_LE_SET_DATA_LENGTH )
//						{
//							LOG("	Set DLE Change status %d\n",pPkt->pReturnParam[0]);
//						}
//						
//						else
//							LOG("	Unknown cmdOpcode %d\n",pPkt->cmdOpcode );
					switch(pPkt->cmdOpcode)
					{
						case HCI_LE_SET_SCAN_PARAM:
						case HCI_LE_SET_SCAN_ENABLE:
							gapCentProcessHCICmdEvt( pPkt->cmdOpcode, (hciEvt_CmdComplete_t*)pMsg);
							break;
						default:
						break;
					}
				}
				break;
				case HCI_COMMAND_STATUS_EVENT_CODE:
				{
					#ifdef _PHY_DEBUG 
							hciEvt_CommandStatus_t *pPkt = (hciEvt_CommandStatus_t *)pMsg;
							LOG("	HCI_COMMAND_STATUS_EVENT_CODE\n");
							if ( pPkt->cmdOpcode == HCI_LE_CREATE_CONNECTION )
							{
								LOG("	HCI_LE_CREATE_CONNECTION status 0x%02X\n",pPkt->cmdStatus);
							}
							else
								LOG("	Unknown cmdOpcode %d\n",pPkt->cmdOpcode );
					#endif
				}
				break;
				case HCI_LE_EVENT_CODE:
				{
					hciEvt_BLEEvent_Hdr_t *pPkt = (hciEvt_BLEEvent_Hdr_t *)pMsg;
		            if ( pPkt->BLEEventCode == HCI_BLE_ADV_REPORT_EVENT  )
		            {
		            	gapCentProcessAdvRptMsg((hciEvt_BLEAdvPktReport_t*)pMsg);
		            }
		            else
		            {
		            	#ifdef _PHY_DEBUG
							LOG("	un-known pPkt->BLEEventCode 0x%X\n",pPkt->BLEEventCode);
						#endif
		            }
				}
				break;
			}
		}
		break;
		case GATT_MSG_EVENT:
		{
			#ifdef _PHY_DEBUG
				gattMsgEvent_t *pPkt = (gattMsgEvent_t *)pMsg;
//				LOG( "	GATT_MSG method 0x%X\n",pPkt->method);
				switch( pPkt->method )
				{
					case ATT_ERROR_RSP:
						LOG("Event Done \n");
					break;
					case ATT_FIND_INFO_RSP:
					{
						attFindInfoRsp_t infoRsp = pPkt->msg.findInfoRsp;
//						LOG( "	numInfo pairs 0x%X\n",infoRsp.numInfo);
						// ATT_HANDLE_BT_UUID_TYPE 
//						LOG( "	format uuid len 0x%X\n",infoRsp.format);
						// value : 16bit load btPair , 128bit load pair
						if( infoRsp.format == ATT_HANDLE_BT_UUID_TYPE)
						{
							for( uint8 i=0;i<infoRsp.numInfo;i++)
							{	
								LOG("connHandle %d,handle %d,uuid 0x%04X\n",pPkt->connHandle,\
																			infoRsp.info.btPair[i].handle,BUILD_UINT16(	\
																			infoRsp.info.btPair[i].uuid[0],\
																			infoRsp.info.btPair[i].uuid[1]) );
							}
						}
						else
						{
							for( uint8 i=0;i<infoRsp.numInfo;i++)
							{	
								LOG("connHandle %d,handle %d,uuid:",pPkt->connHandle ,infoRsp.info.pair[i].handle);
								for(uint8 j=0;j<16;j++)
									LOG(" 0x%X ",infoRsp.info.pair[i].uuid[j]);
								LOG("\n");
							}
						}
					}
					break;
				}
			#endif
		}
		break;
		default:
		#ifdef _PHY_DEBUG
			LOG( "	Unknown Msg Event 0x%X\n",pMsg->event);
		#endif
	break;
  }
#endif
}

/*********************************************************************
 * @fn      gapRole_ProcessGAPMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void gapRole_ProcessGAPMsg( gapEventHdr_t *pMsg )
{
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
#if(defined(GAP_DEFAULT_MSG_BYPASS_ENABLE) && GAP_DEFAULT_MSG_BYPASS_ENABLE)

#else
	switch ( pMsg->opcode )
	{
		case GAP_DEVICE_INIT_DONE_EVENT:
		{
			gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;
			PRINT( "Device init done，pPkt->hdr.status 0x%X\n",pPkt->hdr.status );
			if ( pPkt->hdr.status == SUCCESS )
			{
				
				#if( defined(GAP_CONFIG_STATIC_ADDR) && GAP_CONFIG_STATIC_ADDR)
					GAP_ConfigDeviceAddr(ADDRTYPE_STATIC,BD_STATIC_ADDR);
				#endif
				gapDevDiscReq_t params;
			    params.taskID = gapRole_TaskID;
			    params.mode = DEVDISC_MODE_ALL;
			    params.activeScan = TRUE;
			    params.whiteList = FALSE;
			    uint8 ret = GAP_DeviceDiscoveryRequest(&params);
				LOG("GAP_DeviceDiscoveryRequest RET 0x%X\n",ret);
//				gapRole_SchLinkDev( minit,INVALID_CONNHANDLE);
				
			}
		}
		break;
		case GAP_LINK_ESTABLISHED_EVENT:
		{
			gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;
			gapFreeEstLink();
			LOG("%s,pMsg->opcode:0x%04X GAP Link Established,status 0x%X,connIntv %d\n",__func__,pMsg->opcode,pPkt->hdr.status,pPkt->connInterval );
			if( pPkt->hdr.status == SUCCESS && pPkt->connInterval > 0 )
			{
				LOG("	connection handle		%d\n",pPkt->connectionHandle);
				LOG("	connection interval 	%d\n",pPkt->connInterval);
				LOG("	connection latency	%d\n",pPkt->connLatency);
				LOG("	connection timeout	%d\n",pPkt->connTimeout);
				LOG("	connection devAddrType	%d,device Address:0x",pPkt->devAddrType);
				for(uint8 i=0;i<B_ADDR_LEN;i++)
					LOG("%02X",pPkt->devAddr[i]);
				LOG("\n");	
				
				gapRole_linkCnt++;
				cLinDevAddr[pPkt->connectionHandle] = pPkt->devAddr[0];
				// start SDP
				attFindInfoReq_t req;
				req.startHandle = 1;
				req.endHandle = 0xff;
				gattFindInfo( pPkt->connectionHandle, &req, gapRole_TaskID );
				
//				gapRole_SchLinkDev( mlink,pPkt->connectionHandle );
				osal_start_timerEx(gapRole_TaskID, GAP_START_SCAN_EVT, 2000);

			}
			else
			{
				gapRole_SchLinkDev( mlink_err,pPkt->connectionHandle );
			}
		}
		break;
		case GAP_LINK_TERMINATED_EVENT:
		{
			gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;
			LOG("%s,pMsg->opcode:0x%04X GAP Link Terminated,status 0x%X\n",__func__,pMsg->opcode,pPkt->hdr.status );
			if( pPkt->hdr.status == SUCCESS )
			{
				gapRole_linkCnt--;
				cLinDevAddr[pPkt->connectionHandle] = 0;
				LOG("	connection handle	%d\n",pPkt->connectionHandle);
				LOG("	reason				%d\n",pPkt->reason);
				gapRole_SchLinkDev( mterm,pPkt->connectionHandle );
			}
		}
		break;

		default:
			#ifdef _PHY_DEBUG 
				LOG("%s,pMsg->opcode:0x%04X GAP Device Info \n",__func__,pMsg->opcode);
			#endif
		break;
	}
#endif
}

/*********************************************************************
 * @fn      gapRole_SetupGAP
 *
 * @brief   Call the GAP Device Initialization function using the
 *          Profile Parameters.
 *
 * @param   none
 *
 * @return  none
 */
static void gapRole_SetupGAP( void )
{
	uint8 gapRole_IRK[16];
	uint8 gapRole_SRK[16];
	uint32 gapRole_signCounter;

	GAP_DeviceInit( GAP_DEFAULT_MSG_BYPASS_ID,
		GAP_PROFILE_ROLE, 0,
		gapRole_IRK, gapRole_SRK,
		&gapRole_signCounter );
}

void gapCentProcessAdvRptMsg(hciEvt_BLEAdvPktReport_t* pMsg)
{
	extern uint8 g_advRptCnt;
	if( g_advRptCnt > 0 )
		g_advRptCnt--;
	PRINT("g_advRptCnt %d\n",g_advRptCnt);
	#ifdef _PHY_DEBUG 
//		if( ( pMsg->devInfo->addr[0] == 0x78 ) || ( pMsg->devInfo->addr[0] == 0x66 ))
		{
			LOG("\n%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
			LOG("eventType 0x%X\n",pMsg->devInfo->eventType);
			LOG("addrType 0x%X:",pMsg->devInfo->addrType);
			LOG_DUMP_BYTE(pMsg->devInfo->addr,B_ADDR_LEN);
			LOG("dataLen 0x%X:",pMsg->devInfo->dataLen);
			LOG_DUMP_BYTE(pMsg->devInfo->rspData,pMsg->devInfo->dataLen);
			LOG("rssi %d\n",pMsg->devInfo->rssi);
		}
	#endif
	
}

static void gapRole_SchLinkDev( GAPCentLinkMode mode,uint16 connHandle )
{
	// 1st : find un-Established device
	uint8 buf[4] = {0x66,0x67,0x68,0x00};
	uint8 i=0,j=0;
	// 2nd : startlinking
	if( gapRole_linkCnt < 1 )
	{
		gapEstLinkReq_t params;
		params.taskID = gapRole_TaskID;
		params.highDutyCycle = FALSE;
		params.whiteList = 0;
		params.addrTypePeer = 0;
		volatile uint8 peerAddr[B_ADDR_LEN]={0x66,0x65,0x65,0x56,0x56,0x78};

//		LOG_DUMP_BYTE(cLinDevAddr,3);
//					LOG("\n");
		for( i=0;i<3;i++)
		{
			for( j=0;j<3;j++)
			{
				if( buf[i] == cLinDevAddr[j] )
					break;
				else
					continue;
				
			}
			if( j >= 3 )
				break;
		}
		peerAddr[0] = buf[i];
		LOG("%s:",__func__);
//		LOG_DUMP_BYTE(peerAddr,B_ADDR_LEN);
//					LOG("\n");
		osal_memcpy( params.peerAddr, peerAddr, B_ADDR_LEN );
		PRINT("%s,ret = %d\n",__func__,GAP_EstablishLinkReq( &params ));
	}
}

