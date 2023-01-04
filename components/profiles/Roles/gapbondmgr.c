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
  Filename:       gapbondmgr.c
  Revised:         
  Revision:        

  Description:    GAP peripheral profile manages bonded connections


 
**************************************************************************************************/

#if ( HOST_CONFIG & PERIPHERAL_CFG )

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "gap.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "hci.h"
#include "gattservapp.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */ 
#define GAP_BOND_DEFAULT_PAIRING_MODE	GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

#define GAP_BOND_DEFAULT_BONDING		SM_AUTH_STATE_BONDING
#define GAP_BOND_DEFAULT_MITM			FALSE
#if(defined(GAP_BOND_DEFAULT_MITM) && GAP_BOND_DEFAULT_MITM )
#define GAP_BOND_DEFAULT_SLAVE_SECREQ	(GAP_BOND_DEFAULT_BONDING | SM_AUTH_STATE_AUTHENTICATED )
#else
#define GAP_BOND_DEFAULT_SLAVE_SECREQ	GAP_BOND_DEFAULT_BONDING
#endif

#define GAP_BOND_DEFAULT_PASSCODE		0

#define GAP_BOND_DEFAULT_IO_CAP					GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT
#define GAP_BOND_DEFAULT_OOB_ENABLE				FALSE

/*********************************************************************
 * CONSTANTS
 */
 
/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 gapBondMgr_TaskID;   // Task ID for internal task/event processing

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */
 
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
* LOCAL FUNCTIONS
*/
uint8 gapBondMgr_ProcessOSALMsg( osal_event_hdr_t *pMsg );
void gapBondMgrAuthenticate( uint16 connHandle, uint8 addrType,
                          gapPairingReq_t* pPairReq );



/*********************************************************************
 * @brief   Notify the Bond Manager that a connection has been made.
 *
 * Public function defined in gapbondmgr.h.
 */
bStatus_t GAPBondMgr_LinkEst( uint16 connHandle, uint8 role )
{
	linkDBItem_t *pLinkItem; 
	smSecurityInfo_t ltk;
	ltk.keySize=16;
	//	extern bStatus_t GAP_Bond( uint16 connectionHandle, uint8 authenticated,
	//					  smSecurityInfo_t *pParams, uint8 startEncryption );
	//	  bStatus_t ret = GAP_Bond( connHandle,FALSE,&ltk, FALSE );

	pLinkItem = linkDB_Find( connHandle );
	if ( pLinkItem == NULL )
	{
		return ( bleNotConnected );
	}
	// Copy over Bound information
	if ( pLinkItem->pEncParams )
	{
		// If bound information already exists, free it
		osal_mem_free( pLinkItem->pEncParams );
	}
	// Duplicate the params into the link information
	pLinkItem->pEncParams = (encParams_t *)osal_memdup( &ltk, sizeof( encParams_t ) );
	
	if ( role == GAP_PROFILE_PERIPHERAL &&
			GAP_BOND_DEFAULT_PAIRING_MODE == GAPBOND_PAIRING_MODE_INITIATE )
	{
		GAP_SendSlaveSecurityRequest( connHandle, GAP_BOND_DEFAULT_SLAVE_SECREQ );
	}
  return ( SUCCESS );
}

/*********************************************************************
 * @brief   Register callback functions with the bond manager.
 *
 * Public function defined in gapbondmgr.h.
 */
void GAPBondMgr_Register( gapBondCBs_t *pCB )
{
  // Take over the processing of Authentication messages
  GAP_SetParamValue( TGAP_AUTH_TASK_ID, gapBondMgr_TaskID );

  // Register with GATT Server App for event messages
  GATTServApp_RegisterForMsg( gapBondMgr_TaskID );
}

/*********************************************************************
 * @brief   This is a bypass mechanism to allow the bond manager to process
 *              GAP messages.
 *
 * Public function defined in gapbondmgr.h.
 */

uint8 GAPBondMgr_ProcessGAPMsg( gapEventHdr_t *pMsg )
{
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
	switch ( pMsg->opcode )
	{
		case GAP_PASSKEY_NEEDED_EVENT:
		{
			#ifdef _PHY_DEBUG 
				LOG("	GAP_PASSKEY_NEEDED_EVENT\n");
			#endif
			gapPasskeyNeededEvent_t *pPkt = (gapPasskeyNeededEvent_t *)pMsg;
			if ( GAP_PasscodeUpdate( GAP_BOND_DEFAULT_PASSCODE, pPkt->connectionHandle ) != SUCCESS )
			{
				GAP_TerminateAuth( pPkt->connectionHandle, SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED );
			}
		}
		break;

		case GAP_AUTHENTICATION_COMPLETE_EVENT:
		{
			#ifdef _PHY_DEBUG 
				gapAuthCompleteEvent_t *pPkt = (gapAuthCompleteEvent_t *)pMsg;
				LOG("	GAP_AUTHENTICATION_COMPLETE_EVENT status 0x%X\n",pPkt->hdr.status);
			#endif
		}
		break;

		case GAP_BOND_COMPLETE_EVENT:
		{
			// This message is received when the bonding is complete.  If hdr.status is SUCCESS
			// then call app state callback.  If hdr.status is NOT SUCCESS, the connection will be
			// dropped at the LL because of a MIC failure, so again nothing to do.
			// gapBondCompleteEvent_t *pPkt = (gapBondCompleteEvent_t *)pMsg;
			#ifdef _PHY_DEBUG 
				LOG("	GAP_BOND_COMPLETE_EVENT\n");
			#endif
		}
		break;

		case GAP_PAIRING_REQ_EVENT:
		{
			gapPairingReqEvent_t *pPkt = (gapPairingReqEvent_t *)pMsg;

			#ifdef _PHY_DEBUG 
				LOG("	GAP_PAIRING_REQ_EVENT\n");
			#endif
			
			if ( GAP_BOND_DEFAULT_PAIRING_MODE == GAPBOND_PAIRING_MODE_NO_PAIRING)
			{
				// No Pairing - Send error
				GAP_TerminateAuth( pPkt->connectionHandle, SMP_PAIRING_FAILED_NOT_SUPPORTED );
			}
			else
			{
				linkDBItem_t *pLinkItem = linkDB_Find( pPkt->connectionHandle );

				// Send pairing response
				gapBondMgrAuthenticate( pPkt->connectionHandle, pLinkItem->addrType, &(pPkt->pairReq) );
			}
		}
		break;

		default:
			#ifdef _PHY_DEBUG 
				LOG("	GAP Bond Unknown msg opcode 0x%X\n",pMsg->opcode);
			#endif
		break;
	}
  
  return ( TRUE );
}


/*********************************************************************
 * @brief   Task Initialization function.
 *
 * Internal function defined in gapbondmgr.h.
 */
void GAPBondMgr_Init( uint8 task_id )
{
	gapBondMgr_TaskID = task_id;  // Save task ID
}

/*********************************************************************
 * @brief   Task Event Processor function.
 *
 * Internal function defined in gapbondmgr.h.
 */
uint16 GAPBondMgr_ProcessEvent( uint8 task_id, uint16 events )
{
  	VOID task_id; // OSAL required parameter that isn't used in this function
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif

	if ( events & SYS_EVENT_MSG )
	{
		uint8 *pMsg;

		if ( (pMsg = osal_msg_receive( gapBondMgr_TaskID )) != NULL )
		{
			if ( gapBondMgr_ProcessOSALMsg( (osal_event_hdr_t *)pMsg ) )
			{
				osal_msg_deallocate( pMsg );
			}
		}

		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}
	// Discard unknown events
	return 0;
}

/*********************************************************************
 * @fn      gapBondMgr_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
 uint8 gapBondMgr_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
	uint8 safeToDealloc = TRUE;
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
	switch ( pMsg->event )
	{
		case GAP_MSG_EVENT:
			safeToDealloc = GAPBondMgr_ProcessGAPMsg( (gapEventHdr_t *)pMsg );
		break;
		default:
		#ifdef _PHY_DEBUG 
			LOG("	GAP Bond Unknown msg event 0x%X\n",pMsg->event);
		#endif
		break;
	}
	return ( safeToDealloc );
}

/*********************************************************************
 * @fn      gapBondMgrAuthenticate
 *
 * @brief   Initiate authentication
 *
 * @param   connHandle - connection handle
 * @param   addrType - peer address type
 * @param   pPairReq - Enter these parameters if the Pairing Request was already received.
 *          NULL, if waiting for Pairing Request or if initiating.
 *
 * @return  none
 */
void gapBondMgrAuthenticate( uint16 connHandle, uint8 addrType,
                                    gapPairingReq_t *pPairReq )
{
	gapAuthParams_t params;
	osal_memset(&params, 0,sizeof(gapAuthParams_t));
	params.connectionHandle = connHandle;
	params.secReqs.ioCaps	= GAP_BOND_DEFAULT_IO_CAP;
	params.secReqs.oobAvailable = GAP_BOND_DEFAULT_OOB_ENABLE;
	params.secReqs.authReq = GAP_BOND_DEFAULT_SLAVE_SECREQ;
	params.secReqs.keyDist.sEncKey = TRUE;
//	params.secReqs.keyDist.sIdKey = TRUE;
//	params.secReqs.keyDist.mIdKey = TRUE;
	params.secReqs.maxEncKeySize = KEYLEN;

	#if ( defined( GAP_BOND_DEFAULT_OOB_ENABLE ) && GAP_BOND_DEFAULT_OOB_ENABLE)
	// KEYLEN OOB KEY
	uint8 oob[ KEYLEN ] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10};
	osal_memcpy( params.secReqs.oob, oob, KEYLEN );
	#endif
	GAP_Authenticate( &params, pPairReq );
}
#endif

/*********************************************************************
*********************************************************************/
