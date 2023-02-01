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

/*************************************************************************************************
  Filename:       gatt_client.c
  Revised:        
  Revision:      

  Description:    This file contains the Generic Attribute Profile Client.


**************************************************************************************************/


/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"

#include "gatt.h"
#include "gatt_uuid.h"
#include "gatt_internal.h"
#include "log.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
// Structure to keep Client info
typedef struct
{
  // Info maintained for Client that expecting a response back
  uint16 connHandle;          // connection message was sent out
  uint8 method;               // type of response to be received
  gattParseRsp_t pfnParseRsp; // function to parse response to be received
  uint8 timerId;              // response timeout timer id
  uint8 taskId;               // task to be notified of response

  // GATT Request message
  gattMsg_t req;              // request message

  // Info maintained for GATT Response message
  uint8 numRsps;             // number of responses received
} gattClientInfo_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Client Info table (one entry per each physical link)
gattClientInfo_t clientInfoTbl[GATT_MAX_NUM_CONN];

// Task to be notified of Notification and Indication messages
uint8 indTaskId = INVALID_TASK_ID;

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
 * LOCAL FUNCTIONS
 */
static void gattProcessMultiReqs( uint16 connHandle, gattClientInfo_t *pClient,
                                  uint8 method, gattMsg_t *pMsg );
static uint8 gattProcessFindInfo( gattClientInfo_t *pClient,
                                  uint8 method, gattMsg_t *pMsg );
static uint8 gattProcessFindByTypeValue( gattClientInfo_t *pClient,
                                         uint8 method, gattMsg_t *pMsg );
static uint8 gattProcessReadByType( gattClientInfo_t *pClient,
                                    uint8 method, gattMsg_t *pMsg );
static uint8 gattProcessReadLong( gattClientInfo_t *pClient,
                                  uint8 method, gattMsg_t *pMsg );
static uint8 gattProcessReadByGrpType( gattClientInfo_t *pClient,
                                       uint8 method, gattMsg_t *pMsg );
static bStatus_t gattProcessWriteLong( gattClientInfo_t *pClient,
                                       uint8 method, gattMsg_t *pMsg );
static bStatus_t gattProcessReliableWrites( gattClientInfo_t *pClient,
                                            uint8 method, gattMsg_t *pMsg );

static void gattStoreClientInfo( gattClientInfo_t *pClient, gattMsg_t *pReq,
                                 uint8 method, gattParseRsp_t pfnParseRsp, uint8 taskId );
static gattClientInfo_t *gattFindClientInfo( uint16 connHandle );
static bStatus_t gattGetClientStatus( uint16 connHandle, gattClientInfo_t **p2pClient );
static void gattResetClientInfo( gattClientInfo_t *pClient );
static void gattClientStartTimer( uint8 *pData, uint16 timeout, uint8 *pTimerId );

bStatus_t gattFindInfo( uint16 connHandle, attFindInfoReq_t *pReq, uint8 taskId );
static bStatus_t gattFindByTypeValue( uint16 connHandle, attFindByTypeValueReq_t *pReq,
                                      uint8 taskId );
static bStatus_t gattReadByType( uint16 connHandle, attReadByTypeReq_t *pReq,
                                 uint8 discByCharUUID, uint8 taskId );
static bStatus_t gattRead( uint16 connHandle, attReadReq_t *pReq, uint8 taskId );
static bStatus_t gattReadLong( uint16 connHandle, attReadBlobReq_t *pReq, uint8 taskId );
static bStatus_t gattReadByGrpType( uint16 connHandle, attReadByGrpTypeReq_t *pReq, uint8 taskId );
static bStatus_t gattWrite( uint16 connHandle, attWriteReq_t *pReq, uint8 taskId );
static bStatus_t gattWriteLong( uint16 connHandle, gattPrepareWriteReq_t *pReq, uint8 taskId );

// Callback functions
static bStatus_t gattClientProcessMsgCB( uint16 connHandle,  attPacket_t *pPkt );
static void gattClientHandleTimerCB( uint8 *pData );
static void gattClientHandleConnStatusCB( uint16 connectionHandle, uint8 changeType );

/*********************************************************************
 * API FUNCTIONS
 */

/*-------------------------------------------------------------------
 * GATT Client Public APIs
 */

/******************************************************************************
 * @fn      GATT_InitClient
 *
 * @brief   Initialize the Generic Attribute Profile Client.
 *
 * @return  SUCCESS: Client initialized successfully.
 */
bStatus_t GATT_InitClient( void )
{
  uint8 i;

  // Mark all records as unused
  for ( i = 0; i < GATT_MAX_NUM_CONN; i++ )
  {
    gattClientInfo_t *pClient = &clientInfoTbl[i];

    // Initialize connection handle
    if ( i == 0 )
    {
      pClient->connHandle = LOOPBACK_CONNHANDLE;
    }
    else
    {
      pClient->connHandle = INVALID_CONNHANDLE;
    }

    // Initialize response info
    pClient->method = 0;
    pClient->taskId = INVALID_TASK_ID;
    pClient->timerId = INVALID_TIMER_ID;

    // Initialize GATT Response message info
    pClient->numRsps = 0;

    // Initialize request info
    VOID osal_memset( &(pClient->req), 0 , sizeof( gattMsg_t ) );
  }

  // Set up the client's processing function
  gattRegisterClient( gattClientProcessMsgCB );

  // Register with Link DB to receive link status change callback
  linkDB_Register( gattClientHandleConnStatusCB );

  return ( SUCCESS );
}

/******************************************************************************
 * @fn      GATT_RegisterForInd
 *
 * @brief   Register to receive incoming ATT Indications or Notifications
 *          of attribute values.
 *
 * @param   taskId - task to forward indications or notifications to
 *
 * @return  void
 */
void GATT_RegisterForInd( uint8 taskId )
{
  indTaskId = taskId;
}


/*********************************************************************
 * @fn      GATT_WriteNoRsp
 *
 * @brief   This sub-procedure is used to write a Characteristic Value
 *          to a server when the client knows the Characteristic Value
 *          Handle and the client does not need an acknowledgement that
 *          the write was successfully performed. This sub-procedure
 *          only writes the first (ATT_MTU – 3) octets of a Characteristic
 *          Value. This sub-procedure can not be used to write a long
 *          characteristic; instead the Write Long Characteristic Values
 *          sub-procedure should be used.
 *
 *          The ATT Write Command is used for this sub-procedure. The
 *          Attribute Handle parameter shall be set to the Characteristic
 *          Value Handle. The Attribute Value parameter shall be set to
 *          the new Characteristic Value.
 *
 *          No response will be sent to the calling application task for this
 *          sub-procedure. If the Characteristic Value write request is the
 *          wrong size, or has an invalid value as defined by the profile,
 *          then the write will not succeed and no error will be generated
 *          by the server.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to command to be sent
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleTimeout: Previous transaction timed out.
 */
bStatus_t GATT_WriteNoRsp( uint16 connHandle, attWriteReq_t *pReq )
{
  gattClientInfo_t *pClient;
  uint8 status;

  // Make sure we're allowed to send a new request
  status = gattGetClientStatus( connHandle, &pClient );
  if ( status != bleTimeout )
  {
    if ( ( pReq->sig == FALSE ) && ( pReq->cmd == TRUE ) )
    {
      status = ATT_WriteReq( connHandle, pReq );
    }
    else
    {
      status = INVALIDPARAMETER;
    }
  }

  return ( status );
}

/*********************************************************************
 * @fn      GATT_WriteCharValue
 *
 * @brief   This sub-procedure is used to write a characteristic value
 *          to a server when the client knows the characteristic value
 *          handle. This sub-procedure only writes the first (ATT_MTU-3)
 *          octets of a characteristic value. This sub-procedure can not
 *          be used to write a long attribute; instead the Write Long
 *          Characteristic Values sub-procedure should be used.
 *
 *          The ATT Write Request is used in this sub-procedure. The
 *          Attribute Handle parameter shall be set to the Characteristic
 *          Value Handle. The Attribute Value parameter shall be set to
 *          the new characteristic.
 *
 *          If the return status from this function is SUCCESS, the calling
 *          application task will receive an OSAL GATT_MSG_EVENT message.
 *          The type of the message will be either ATT_WRITE_RSP
 *          or ATT_ERROR_RSP (if an error occurred on the server).
 *
 *          Note: This sub-procedure is complete when either ATT_WRITE_RSP
 *                (with SUCCESS or bleTimeout status) or ATT_ERROR_RSP (with
 *                SUCCESS status) is received by the calling application task.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          blePending: A response is pending with this server.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleTimeout: Previous transaction timed out.
 */
bStatus_t GATT_WriteCharValue( uint16 connHandle, attWriteReq_t *pReq, uint8 taskId )
{
  return ( gattWrite( (connHandle), (pReq), (taskId) ) );
}

/*-------------------------------------------------------------------
 * GATT Client Internal Functions
 */

/*********************************************************************
 * @fn      gattFindInfo
 *
 * @brief   The Find Information Request is used to obtain the mapping
 *          of attribute handles with their associated types. This
 *          allows a client to discover the list of attributes and
 *          their types on a server.
 *
 *          Only attributes with attribute handles between and including
 *          the Starting Handle parameter and the Ending Handle parameter
 *          will be returned. To read all attributes, the Starting Handle
 *          parameter shall be set to 0x0001, and the Ending Handle
 *          parameter shall be set to 0xFFFF.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          blePending: A response is pending with this server.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleTimeout: Previous transaction timed out.
 */
bStatus_t gattFindInfo( uint16 connHandle, attFindInfoReq_t *pReq, uint8 taskId )
{
  uint8 status;
  gattClientInfo_t *pClient;

  // Make sure we're allowed to send a new request
  status = gattGetClientStatus( connHandle, &pClient );
  if ( status == SUCCESS )
  {
    // Send the request
    status = ATT_FindInfoReq( connHandle, pReq );
    if ( status == SUCCESS )
    {
      // Store client info
      gattStoreClientInfo( pClient, (gattMsg_t *)pReq, ATT_FIND_INFO_RSP,
                           ATT_ParseFindInfoRsp, taskId );
    }
  }

  return ( status );
}

/*********************************************************************
 * @fn      gattRead
 *
 * @brief   The Read Request is used to request the server to read
 *          the value of an attribute and return its value in a
 *          Read Response.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          blePending: A response is pending with this server.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleTimeout: Previous transaction timed out.
 */
static bStatus_t gattRead( uint16 connHandle, attReadReq_t *pReq, uint8 taskId )
{
  gattClientInfo_t *pClient;
  uint8 status;

  // Make sure we're allowed to send a new request
  status = gattGetClientStatus( connHandle, &pClient );
  if ( status == SUCCESS )
  {
    // Send the request
    status = ATT_ReadReq( connHandle, pReq );
    if ( status == SUCCESS )
    {
      // Store client info
      gattStoreClientInfo( pClient, NULL, ATT_READ_RSP, ATT_ParseReadRsp, taskId );
    }
  }

  return ( status );
}


/*********************************************************************
 * @fn      gattWrite
 *
 * @brief   The Write Request is used to request the server to write
 *          the value of an attribute and acknowledge that this has
 *          been achieved in a Write Response.
 *
 * @param   connHandle - connection to use
 * @param   pReq - pointer to request to be sent
 * @param   taskId - task to be notified of response
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Invalid connection handle or request field.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          blePending: A response is pending with this server.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleTimeout: Previous transaction timed out.
 */
static bStatus_t gattWrite( uint16 connHandle, attWriteReq_t *pReq, uint8 taskId )
{
  uint8 status;

  if ( ( pReq->sig == FALSE ) && ( pReq->cmd == FALSE ) )
  {
    gattClientInfo_t *pClient;

    // Make sure we're allowed to send a new request
    status = gattGetClientStatus( connHandle, &pClient );
    if ( status == SUCCESS )
    {
      // Send the request
      status = ATT_WriteReq( connHandle, pReq );
      if ( status == SUCCESS )
      {
        // Store client info
        gattStoreClientInfo( pClient, NULL, ATT_WRITE_RSP,
                             ATT_ParseWriteRsp, taskId );
      }
    }
  }
  else
  {
    status = INVALIDPARAMETER;
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattProcessClientMsgCB
 *
 * @brief       GATT Client message processing function.
 *
 * @param       connHandle - connection packet was received on
 * @param       pPkt - pointer to received packet
 *
 * @return      SUCCESS: Message processed successfully
 *              ATT_ERR_UNSUPPORTED_REQ: Unsupported message
 *              ATT_ERR_INVALID_PDU: Invalid PDU
 *              bleMemAllocError: Memory allocation error occurred
 */
static bStatus_t gattClientProcessMsgCB( uint16 connHandle, attPacket_t *pPkt )
{
  gattMsg_t msg;
  gattClientInfo_t *pClient;
  uint8 status;
//	LOG("%s\n",__func__);
  // Make sure the incoming message is supported
//  if ( ( pPkt->sig != ATT_SIG_NOT_INCLUDED ) || ( pPkt->cmd == TRUE ) )
//  {
//    // Unsupported message
//    return ( ATT_ERR_UNSUPPORTED_REQ );
//  }

  // See if this is an indication or notification
  if ( ( pPkt->method == ATT_HANDLE_VALUE_NOTI ) || ( pPkt->method == ATT_HANDLE_VALUE_IND ) )
  {
    if ( indTaskId == INVALID_TASK_ID )
    {
      // Application hasn't registered for it!
      return ( ATT_ERR_UNSUPPORTED_REQ );
    }

    // Parse indication
    status = ATT_ParseHandleValueInd( pPkt->sig, pPkt->cmd, pPkt->pParams, pPkt->len, (attMsg_t *)&msg );
    if ( status == SUCCESS )
    {
      // Forward the message to upper layer application
      status = gattNotifyEvent( indTaskId, connHandle, SUCCESS, pPkt->method, &msg );
    }

    // We're done here
    return ( status );
  }

  // Make sure we have the info about the Client that initiated the request
  pClient = gattFindClientInfo( connHandle );
//  LOG("pClient %p\n",pClient);
  if ( pClient == NULL )
  {
    // Request must have timed out
    return ( SUCCESS );
  }
//  LOG("pPkt->method 0x%X\n",pPkt->method);
//  LOG("pClient->method 0x%X\n",pClient->method);

  // Parse the message
  if ( pPkt->method == ATT_ERROR_RSP )
  {
    status = ATT_ParseErrorRsp( pPkt->pParams, pPkt->len, (attMsg_t *)&msg );
  }
  else
  {
    if ( ( pPkt->method == pClient->method ) && ( pClient->pfnParseRsp != NULL ) )
    {
      status = (*pClient->pfnParseRsp)( pPkt->pParams, pPkt->len, (attMsg_t *)&msg );
    }
    else
    {
      // We're not expecting this message!
      status = ATT_ERR_INVALID_PDU;
    }
  }
//LOG("%s,status %d\n",__func__,status);
  // Try to process the response
  if ( status == SUCCESS )
  {
  	if( pClient->method == ATT_FIND_INFO_RSP )
	{
		gattProcessMultiReqs( connHandle, pClient, pPkt->method, &msg );
	}
    // See if this is a response to a GATT sub-procedure with multiple requests
//    if ( ( pClient->method == ATT_FIND_INFO_RSP )          ||
//         ( pClient->method == ATT_FIND_BY_TYPE_VALUE_RSP ) ||
//         ( pClient->method == ATT_READ_BY_TYPE_RSP )       ||
//         ( pClient->method == ATT_READ_BLOB_RSP )          ||
//         ( pClient->method == ATT_READ_BY_GRP_TYPE_RSP )  
////         ( ( pClient->method == ATT_PREPARE_WRITE_RSP )    &&
////           ( pClient->req.gattReliableWritesReq.pReqs != NULL ) ) 
//			) // needed for GATT testing
//    {
//      gattProcessMultiReqs( connHandle, pClient, pPkt->method, &msg );
//    }
//    else if ( ( pClient->method == ATT_EXCHANGE_MTU_RSP )  )       // add 2020-03-18
//    {    
////        ATT_MTU_SIZE_UPDATE(MIN(msg.exchangeMTURsp.serverRxMTU, pClient->req.exchangeMTUReq.clientRxMTU));
//		ATT_UpdateMtuSize(connHandle, MIN(msg.exchangeMTURsp.serverRxMTU, pClient->req.exchangeMTUReq.clientRxMTU));
//
////        LOG("[MTU Rsp]c%d s%d l%d\n", pClient->req.exchangeMTUReq.clientRxMTU, msg.exchangeMTURsp.serverRxMTU, gAttMtuSize[connHandle]);
//
//        // Forward the message to upper layer application
//        status = gattNotifyEvent( pClient->taskId, connHandle, SUCCESS, pPkt->method, &msg );
//
//        // Reset client info
//        gattResetClientInfo( pClient );		
//    }		   
    else
    {
      // Forward the message to upper layer application
      status = gattNotifyEvent( pClient->taskId, connHandle, SUCCESS, pPkt->method, &msg );

      // Reset client info
      gattResetClientInfo( pClient );
    }
  }

  return ( status );
}

/*********************************************************************
 * @fn          gattProcessMultiReqs
 *
 * @brief       Process a GATT sub-procedure that could generate multiple
 *              request messages.
 *
 *              The received response is forwarded up to the application,
 *              or/and the sub-procedure complete is sent to the application
 *              depending on the status returned from the process function:
 *
 *                       Forward Rsp    Send Complete    Reset Client Info
 *                       ===========    =============    =================
 *              Pending      Yes*            No                 No
 *
 *              Failure      Yes             No                 Yes
 *
 *              Success      Yes**           Yes                Yes
 *
 *              *  = Except Prepare Write Response
 *              ** = Except Error Response that indicates end of sub-procedure
 *
 * @param       connHandle - connection event belongs to
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received message
 *
 * @return      none
 */
static void gattProcessMultiReqs( uint16 connHandle, gattClientInfo_t *pClient,
                                  uint8 method, gattMsg_t *pMsg )
{
  uint8 status;

  switch ( pClient->method )
  {
    case ATT_FIND_INFO_RSP:
      status = gattProcessFindInfo( pClient, method, pMsg );
      break;

    default:
      // Should never get here!
      status = FAILURE;
      break;
  }

  // Do not forward an Error Response that indicates the end of a sub-procedure
//  if ( ( status != SUCCESS )       ||
//       ( method != ATT_ERROR_RSP ) ||
//       ( pMsg->errorRsp.errCode != ATT_ERR_ATTR_NOT_FOUND ) )
//  {
//    // Make sure there's data to be forwarded
//    if ( ( method != ATT_READ_BY_TYPE_RSP ) || ( pMsg->readByTypeRsp.numPairs > 0 ) )
//    {
//      // Forward the message to upper layer application
//      VOID gattNotifyEvent( pClient->taskId, connHandle, SUCCESS, method, pMsg );
//    }
//  }
//LOG("%s,status %d\n",__func__,status);
//  if ( status == SUCCESS )
  {
    // Indicate sub-procedure completion to upper layer application
//    gattNotifyEvent( pClient->taskId, connHandle, bleProcedureComplete, pClient->method, pMsg /*NULL*/ );
	gattNotifyEvent( pClient->taskId, connHandle, bleProcedureComplete, /*pClient->method*/method, (method == ATT_ERROR_RSP)?NULL:pMsg );

  }

  if ( status != blePending )
  {
    // Reset client info
    gattResetClientInfo( pClient );
  }
}


/*********************************************************************
 * @fn          gattProcessFindInfo
 *
 * @brief       Process a Find Information message.
 *
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received
 *
 * @return      SUCCESS: Sub-procedure completed
 *              FAILURE: Sub-procedure failed
 *              blePending: Sub-procedure still in progress
 */
static uint8 gattProcessFindInfo( gattClientInfo_t *pClient,
                                  uint8 method, gattMsg_t *pMsg )
{
  // Response to Find Info Request
  if ( method == ATT_FIND_INFO_RSP )
  {
    attFindInfoRsp_t *pRsp = &(pMsg->findInfoRsp); // received response
    attFindInfoReq_t *pReq = &(pClient->req.findInfoReq); // original request
    uint16 endHandle;

    pClient->numRsps++; // Increment number of responses

    // Find out the end handle
    if ( pRsp->format == ATT_HANDLE_BT_UUID_TYPE )
    {
      endHandle = pRsp->info.btPair[pRsp->numInfo-1].handle;
    }
    else // ATT_HANDLE_UUID_TYPE
    {
      endHandle = pRsp->info.pair[pRsp->numInfo-1].handle;
    }

    // The sub-procedure is complete when the Find Information Response has
    // an Attribute Handle that is equal to the Ending Handle of the request.
    if ( endHandle < pReq->endHandle )
    {
      // Update the start handle
      pReq->startHandle = endHandle + 1;

      // Send another Find Info Request
      ATT_FindInfoReq( pClient->connHandle, pReq );

      osal_CbTimerUpdate( pClient->timerId, (ATT_MSG_TIMEOUT * 1000) );

      return ( blePending );
    }
  }
  else // ATT_ERROR_RSP
  {
    attErrorRsp_t *pErrorRsp = &pMsg->errorRsp;

    // See if an error occurred on the server
    if ( ( pErrorRsp->errCode != ATT_ERR_ATTR_NOT_FOUND ) || ( pClient->numRsps == 0 ) )
    {
      // Should never get here!
      return ( FAILURE );
    }

    // The sub-procedure is complete when the Error Response is received and
    // the Error Code is set to Attribute Not Found.
  }

  // No more attributes can be discovered on the server; this sub-procedure is complete.
  return ( SUCCESS );
}

/*********************************************************************
 * @fn          gattProcessFindByTypeValue
 *
 * @brief       Process a Find By Type Value message.
 *
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received
 *
 * @return      SUCCESS: Sub-procedure completed
 *              FAILURE: Sub-procedure failed
 *              blePending: Sub-procedure still in progress
 */


/*********************************************************************
 * @fn          gattProcessReadByType
 *
 * @brief       Process a Read By Type message.
 *
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received
 *
 * @return      SUCCESS: Sub-procedure completed
 *              FAILURE: Sub-procedure failed
 *              blePending: Sub-procedure still in progress
 */


/*********************************************************************
 * @fn          gattProcessReadLong
 *
 * @brief       Process Read Long message.
 *
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received
 *
 * @return      SUCCESS: Sub-procedure completed
 *              FAILURE: Sub-procedure failed
 *              blePending: Sub-procedure still in progress
 */


/*********************************************************************
 * @fn          gattProcessReadByGrpType
 *
 * @brief       Process a Read By Group Type message.
 *
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received
 *
 * @return      SUCCESS: Sub-procedure completed
 *              FAILURE: Sub-procedure failed
 *              blePending: Sub-procedure still in progress
 */


/*********************************************************************
 * @fn          gattProcessWriteLong
 *
 * @brief       Process Write Long message.
 *
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received
 *
 * @return      SUCCESS: Sub-procedure completed
 *              FAILURE: Sub-procedure failed
 *              blePending: Sub-procedure still in progress
 */


/*********************************************************************
 * @fn          gattProcessReliableWrites
 *
 * @brief       Process Relaible Writes message.
 *
 * @param       pClient - pointer to client info
 * @param       method - type of message
 * @param       pMsg - pointer to received
 *
 * @return      SUCCESS: Sub-procedure completed
 *              FAILURE: Sub-procedure failed
 *              blePending: Sub-procedure still in progress
 */


/*********************************************************************
 * @fn      gattStoreClientInfo
 *
 * @brief   Store client info.
 *
 * @param   connHandle – connection to use
 * @param   pMsg – pointer to message to be sent
 * @param   method – response to expect
 * @param   pfnParseRsp – parse function for response
 * @param   taskId – task to be notified of response
 *
 * @return  none
 */
static void gattStoreClientInfo( gattClientInfo_t *pClient, gattMsg_t *pReq,
                                 uint8 method, gattParseRsp_t pfnParseRsp, uint8 taskId )
{
  if ( taskId != INVALID_TASK_ID )
  {
    // Start a timeout timer for the response
    gattClientStartTimer( (uint8 *)pClient, ATT_MSG_TIMEOUT, &pClient->timerId );

    // Store task id to forward the response to
    pClient->taskId = taskId;

    // Store the response method
    pClient->method = method;

    // Store parse function for the response
    pClient->pfnParseRsp = pfnParseRsp;

    // Store info for GATT request
    if ( pReq != NULL )
    {
      osal_memcpy( &(pClient->req), pReq, sizeof( gattMsg_t ) );
    }
    // else pClient->req is already zero'ed out
  }
}

/*********************************************************************
 * @fn          gattGetClientStatus
 *
 * @brief       Get the status for a given client.
 *
 * @param       connHandle - client connection to server
 * @param       p2pClient - pointer to pointer to client info
 *
 * @return      SUCCESS: No response pending
 *              INVALIDPARAMETER: Invalid connection handle
 *              blePending: Response pending
 *              bleTimeout: Previous transaction timed out
 */
static bStatus_t gattGetClientStatus( uint16 connHandle, gattClientInfo_t **p2pClient )
{
  gattClientInfo_t *pClient;

  pClient = gattFindClientInfo( connHandle );
  if ( pClient != NULL )
  {
    if ( p2pClient != NULL )
    {
      *p2pClient = pClient;
    }

    // Make sure there's no response pending or timed out with this server
    return ( TIMER_STATUS( pClient->timerId ) );
  }

  // Connection handle not found
  return ( INVALIDPARAMETER );
}

/*********************************************************************
 * @fn      gattFindClientInfo
 *
 * @brief   Find the client info.  Uses the connection handle to search
 *          the client info table.
 *
 * @param   connHandle - connection handle.
 *
 * @return  a pointer to the found item. NULL, otherwise.
 */
static gattClientInfo_t *gattFindClientInfo( uint16 connHandle )
{
  uint8 i;

  for ( i = 0; i < GATT_MAX_NUM_CONN; i++ )
  {
    if ( clientInfoTbl[i].connHandle == connHandle )
    {
      // Entry found
      return ( &clientInfoTbl[i] );
    }
  }

  return ( (gattClientInfo_t *)NULL );
}

/*********************************************************************
 * @fn      gattResetClientInfo
 *
 * @brief   Reset the client info.
 *
 * @param   pClient - pointer to client info.
 *
 * @return  none
 */
static void gattResetClientInfo( gattClientInfo_t *pClient )
{
  // First cancel the response timer
  gattStopTimer( &pClient->timerId );

  // Free the buffer provided by the application
//  if ( ( pClient->method == ATT_PREPARE_WRITE_RSP ) ||
//       ( pClient->method == ATT_EXECUTE_WRITE_RSP ) )
//  {
//    uint8 *pBuf;
//
//    if ( pClient->req.gattReliableWritesReq.reliable == TRUE )
//    {
//      pBuf = (uint8 *)(pClient->req.gattReliableWritesReq.pReqs);
//    }
//    else
//    {
//      pBuf = pClient->req.gattWriteLongReq.req.pValue;
//    }
//
//    if ( pBuf != NULL )
//    {
//      osal_mem_free( pBuf );
//    }
//  }

  // Reset response info
  pClient->method = 0;
  pClient->taskId = INVALID_TASK_ID;
  pClient->numRsps = 0;

  // Reset request info
  VOID osal_memset( &(pClient->req), 0 , sizeof( gattMsg_t ) );
}

/*********************************************************************
 * @fn      gattClientStartTimer
 *
 * @brief   Start a client timer to expire in n seconds.
 *
 * @param   pData - data to be passed in to callback function
 * @param   timeout - in seconds.
 * @param   pTimerId - will point to new timer Id (if not null)
 *
 * @return  none
 */
static void gattClientStartTimer( uint8 *pData, uint16 timeout, uint8 *pTimerId )
{
  gattStartTimer( gattClientHandleTimerCB, pData, timeout, pTimerId );
}

/*********************************************************************
 * @fn      gattClientHandleTimerCB
 *
 * @brief   Handle a callback for a timer that has just expired.
 *
 * @param   pData - pointer to timer data
 *
 * @return  none
 */
static void gattClientHandleTimerCB( uint8 *pData )
{
  gattClientInfo_t *pClient = (gattClientInfo_t *)pData;

  // Response timer has expired
  if ( ( pClient != NULL ) && ( pClient->timerId != INVALID_TIMER_ID ) )
  {
    // Notify the application about the timeout
    gattNotifyEvent( pClient->taskId, pClient->connHandle, bleTimeout, pClient->method, NULL );

//    if ( pClient->method == ATT_EXECUTE_WRITE_REQ )
//    {
//      attExecuteWriteReq_t req;
//
//      // Cancel all prepared writes
//      req.flags = ATT_CANCEL_PREPARED_WRITES;
//
//      VOID ATT_ExecuteWriteReq( pClient->connHandle, &req );
//    }

    // Timer has expired. If a transaction has not completed before it times
    // out, then this transaction shall be considered to have failed. No more
    // attribute protocol requests, commands, indications or notifications
    // shall be sent to the target device on this ATT Bearer.

//ZQ 20181216 for test
    //pClient->timerId = TIMEOUT_TIMER_ID;

    // Reset client info
    gattResetClientInfo( pClient );
  }
}

/*********************************************************************
 * @fn          gattClientHandleConnStatusCB
 *
 * @brief       GATT link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void gattClientHandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  gattClientInfo_t *pClient = NULL;

  // Check to see if this is loopback connection
  if ( connHandle == LOOPBACK_CONNHANDLE )
  {
    return;
  }

  if ( changeType == LINKDB_STATUS_UPDATE_NEW )
  {
    // A new connection has been made
    pClient = gattFindClientInfo( connHandle );
    if ( pClient == NULL )
    {
      // Entry not found; add it to the server table
      pClient = gattFindClientInfo( INVALID_CONNHANDLE );
      if ( pClient != NULL )
      {
        // Empty entry found
        pClient->connHandle = connHandle;
      }
    }

    // We're done here!
    return;
  }

  if ( changeType == LINKDB_STATUS_UPDATE_REMOVED )
  {
    pClient = gattFindClientInfo( connHandle );
    if ( pClient != NULL )
    {
      // Entry found; remove it from the client table
      pClient->connHandle = INVALID_CONNHANDLE;
    }
  }
  else if ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS )
  {
    // Check to see if the connection has dropped
    if ( !linkDB_Up( connHandle ) )
    {
      pClient = gattFindClientInfo( connHandle );
    }
  }

  // Connection has dropped
  if ( pClient != NULL )
  {
    if ( pClient->timerId != INVALID_TIMER_ID )
    {
      if ( pClient->timerId != TIMEOUT_TIMER_ID )
      {
        // Notify the application about the link disconnect
        VOID gattNotifyEvent( pClient->taskId, connHandle, bleNotConnected,
                              pClient->method, NULL );
      }

      // Reset client info
      gattResetClientInfo( pClient );

      // Just in case if we've timed out waiting for a response
      pClient->timerId = INVALID_TIMER_ID;
    }
  }
}


/****************************************************************************
****************************************************************************/
