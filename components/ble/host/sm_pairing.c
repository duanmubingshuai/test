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
  Filename:       sm_pairing.c
  Revised:        
  Revision:      

  Description:    This file contains the SM Pairing Manager.


**************************************************************************************************/

#include "bcomdef.h"
#include "l2cap.h"
#include "gap_internal.h"
#include "linkdb.h"
#include "sm.h"
#include "sm_internal.h"
#include "smp.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Defines just for the IOCapMatrix table below
#define JUST_WORKS         SM_PAIRING_TYPE_JUST_WORKS
#define INIT_INP           SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS
#define RESP_INP           SM_PAIRING_TYPE_PASSKEY_RESPONDER_INPUTS
#define BOTH_INP           SM_PAIRING_TYPE_PASSKEY_BOTH_INPUTS

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Pairing parameters, NULL while not pairing
smPairingParams_t *pPairingParams = NULL;

// Callback function pointers for Responder
smResponderCBs_t *pfnResponderCBs = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Callback function pointers for Initiator
//static const smInitiatorCBs_t *pfnInitiatorCBs = NULL;
 
// Determine the Pairing passkey requirement (from the spec):  
//       IOCapMatrix[Responder's IOCapability][Initiator's IOCapability];
static CONST uint8 IOCapMatrix[5][5] = 
{
/*        Initiator:    DisplayOnly,  DisplayYesNo, KeyboardOnly, NoInputNoOutput,  KeyboardDisplay */
/* Responder:      */ 
/* DisplayOnly     */ { JUST_WORKS,   JUST_WORKS,   INIT_INP,     JUST_WORKS,       INIT_INP },
/* DisplayYesNo    */ { JUST_WORKS,   JUST_WORKS,   INIT_INP,     JUST_WORKS,       INIT_INP },
/* KeyboardOnly    */ { RESP_INP,     RESP_INP,     BOTH_INP,     JUST_WORKS,       RESP_INP },
/* NoInputNoOutput */ { JUST_WORKS,   JUST_WORKS,   JUST_WORKS,   JUST_WORKS,       JUST_WORKS },
/* KeyboardDisplay */ { RESP_INP,     RESP_INP,     INIT_INP,     JUST_WORKS,       RESP_INP }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 smpProcessIncoming( uint16 connHandle, uint8 cmdID, smpMsgs_t *pParsedMsg );

static bStatus_t smDetermineIOCaps( uint8 initiatorIO, uint8 responderIO );
static void smFreePairingParams( void );
static void smSetPairingReqRsp( smpPairingReq_t *pReq );


/*********************************************************************
 * API FUNCTIONS
 */


/*********************************************************************
 * FUNCTIONS - MASTER API - Only use these in a master device
 */


/*********************************************************************
 * Start the pairing process with a slave device.
 * 
 * Public function defined in sm.h.
 */
bStatus_t SM_StartPairing( uint8 initiator, 
                           uint8 taskID, 
                           uint16 connectionHandle, 
                           smLinkSecurityReq_t *pSecReqs )
{
  bStatus_t ret = SUCCESS;    // Return value
  
  // Only one pairing at a time
  if ( pPairingParams )
  {
    return ( bleAlreadyInRequestedMode );
  }
  
  // Check parameters
  if ( (pSecReqs == NULL) || (pSecReqs->maxEncKeySize < GAP_GetParamValue( TGAP_SM_MIN_KEY_LEN ))
       || (pSecReqs->maxEncKeySize > GAP_GetParamValue( TGAP_SM_MAX_KEY_LEN )) )
  {
    return ( INVALIDPARAMETER );
  }
  
  pPairingParams = ( smPairingParams_t *)osal_mem_alloc( (uint16)sizeof ( smPairingParams_t ) );
  if ( pPairingParams == NULL )
  {
    return ( bleMemAllocError );
  }
  
  osal_memset( pPairingParams, 0, sizeof( smPairingParams_t ) );
  
  // Save parameters
  pPairingParams->state = SM_PAIRING_STATE_INITIALIZE;
  pPairingParams->initiator = initiator;
  pPairingParams->taskID = taskID;
  pPairingParams->connectionHandle = connectionHandle;
  pPairingParams->pSecReqs = pSecReqs;
  pPairingParams->pPairDev = NULL;
  pPairingParams->type = SM_PAIRING_TYPE_INIT;

//  if ( initiator )
//  {
//    // Start the pairing process
//    ret = smGeneratePairingReqRsp();
//  }

  pPairingParams->state = SM_PAIRING_STATE_PAIRING_REQ_SENT;
  
  if ( ret != SUCCESS )
  {
    // Free the mem - not successful.
    smFreePairingParams();
  }
  
  return ( ret );
}

/*********************************************************************
 * Update the passkey for the pairing process.
 *
 * Public function defined in sm.h.
 */
bStatus_t SM_PasskeyUpdate( uint8 *pPasskey, uint16 connectionHandle )
{
  bStatus_t ret = SUCCESS;    // return value
  uint8 sendConfirm = FALSE;  // flag to send pairing confirm message
  
  // Are we pairing?
  if ( pPairingParams == NULL )
  {
    return ( bleIncorrectMode );
  }
  
  // Are we expecting the passkey?
  if ( (pPairingParams->type != SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS)
      && (pPairingParams->type != SM_PAIRING_TYPE_PASSKEY_RESPONDER_INPUTS)
        && (pPairingParams->type != SM_PAIRING_TYPE_PASSKEY_BOTH_INPUTS) )
  {
    return ( bleIncorrectMode );
  }

  // Is this the correct connection?  
  if ( pPairingParams->connectionHandle != connectionHandle )
  {
    return ( INVALIDPARAMETER );
  }
  
  // Copy the passkey (tk)
  osal_memcpy( pPairingParams->tk, pPasskey, KEYLEN );
  
  // Generate Rand (MRand or SRand)
  smGenerateRandBuf( pPairingParams->myComp.rand, SMP_RANDOM_LEN );
  
  // Generate Confirm (MConfirm or SConfirm)
  sm_c1( pPairingParams->tk, 
        pPairingParams->myComp.rand, 
        pPairingParams->myComp.confirm );

  // Set the correct state  
//  if ( (pPairingParams->initiator) 
//      && (pPairingParams->state == SM_PAIRING_STATE_WAIT_PASSKEY) )
//  {
//    // Send Confirm
//    sendConfirm = TRUE;
//    
//    // Next, wait for Responder Confirm
//    pPairingParams->state = SM_PAIRING_STATE_WAIT_CONFIRM;
//  }
//  else 
  if ( (pPairingParams->initiator == 0)
    && (pPairingParams->state == SM_PAIRING_STATE_WAIT_CONFIRM_PASSKEY) )
  {
    pPairingParams->state = SM_PAIRING_STATE_WAIT_CONFIRM;
  }
  else if ( (pPairingParams->initiator == 0)
    && (pPairingParams->state == SM_PAIRING_STATE_WAIT_PASSKEY) )
  {
    // We have both confirm and passkey, send our own confirm
    sendConfirm = TRUE;
      
    // Next, wait for Pairing Random
    pPairingParams->state = SM_PAIRING_STATE_WAIT_RANDOM;
  }
  
  if ( sendConfirm )
  {
#if defined ( TESTMODES )
    if ( GAP_GetParamValue( TGAP_SM_TESTCODE ) == SM_TESTMODE_SEND_BAD_CONFIRM )
    {
      VOID osal_memset( pPairingParams->myComp.confirm, 0, KEYLEN );
    }
#endif // TESTMODE
    
    // Send the Pairing Confirm message    
    ret = smGenerateConfirm();
  }
  
  return ( ret );
}

/*********************************************************************
 * PUBLIC INTERNAL SM FUNCTIONS
 */

/*********************************************************************
 * @fn      smRegisterInitiator
 *
 * @brief   Register Initiator's processing function with SM task.
 *
 * @param   pfnCBs - pointer to Initiator's processing function
 *
 * @return  none
 */
//void smRegisterInitiator( smInitiatorCBs_t *pfnCBs )
//{
//  pfnInitiatorCBs = pfnCBs;
//}

/*********************************************************************
 * @fn      smRegisterResponder
 *
 * @brief   Register Responder's processing function with SM task.
 *
 * @param   pfnCBs - pointer to Responder's processing function
 *
 * @return  none
 */
void smRegisterResponder( smResponderCBs_t *pfnCBs )
{
  pfnResponderCBs = pfnCBs;
}

/*********************************************************************
 * @fn          smLinkCheck
 *
 * @brief       Callback for linkDB to indicate link changes.
 *
 * @param       connectionHandle - link connection handle
 * @param       changeType - link connection handle
 *                              ex. LINKDB_STATUS_UPDATE_REMOVED
 *
 * @return      none
 */
void smLinkCheck( uint16 connectionHandle, uint8 changeType )
{
  if ( (pPairingParams) 
      && (pPairingParams->connectionHandle == connectionHandle) 
        && (changeType == LINKDB_STATUS_UPDATE_REMOVED) )
  {
    // Connection is down, remove the pairing information
    smFreePairingParams();
  }
}

/*********************************************************************
 * @fn          smTimedOut
 *
 * @brief       Something didn't repond quickly enough.
 *
 * @param       none
 *
 * @return      none
 */
void smTimedOut( void )
{
  smEndPairing( bleTimeout );
}

/*********************************************************************
 * @fn          smProcessDataMsg
 *
 * @brief       Process incoming L2CAP messages.
 *
 * @param       pMsg - pointer to message.
 *
 * @return      none
 */
void smProcessDataMsg( l2capDataEvent_t *pMsg )
{
  smpMsgs_t parsedMsg;        // Place to parse the message
  bStatus_t stat = SUCCESS;   // Return value
  smpPairingFailed_t failed;  // Pairing Failed message
  uint8 cmdID;                // Message command ID
  
#if defined ( TESTMODES )
  if ( GAP_GetParamValue( TGAP_SM_TESTCODE ) == SM_TESTMODE_NO_RESPONSE )
  {
    // just ignore the messages
    return;
  }
#endif // TESTMODE
  
  failed.reason = SUCCESS;  // Default to success
  
  // Parse the incoming message
  cmdID = *(pMsg->pkt.pPayload);
  switch ( cmdID )
  {
    case SMP_PAIRING_REQ:
    case SMP_PAIRING_RSP:
      stat = smpParsePairingReq( pMsg->pkt.pPayload, &(parsedMsg.pairingReq) );
      if ( stat == bleIncorrectMode )
      {
        failed.reason = SMP_PAIRING_FAILED_ENC_KEY_SIZE;
      }
      break;
      
    case SMP_PAIRING_CONFIRM:
      stat = smpParsePairingConfirm( pMsg->pkt.pPayload, &(parsedMsg.pairingConfirm) );
      break;
      
    case SMP_PAIRING_RANDOM:
      stat = smpParsePairingRandom( pMsg->pkt.pPayload, &(parsedMsg.pairingRandom) );
      break;
      
    case SMP_PAIRING_FAILED:
      stat = smpParsePairingFailed( pMsg->pkt.pPayload, &(parsedMsg.pairingFailed) );
      break;
      
    case SMP_ENCRYPTION_INFORMATION:
      stat = smpParseEncInfo( pMsg->pkt.pPayload, &(parsedMsg.encInfo) );
      break;
      
    case SMP_MASTER_IDENTIFICATION:
      stat = smpParseMasterID( pMsg->pkt.pPayload, &(parsedMsg.masterID) );
      break;
      
    case SMP_IDENTITY_INFORMATION:
      stat = smpParseIdentityInfo( pMsg->pkt.pPayload, &(parsedMsg.idInfo) );
      break;
      
    case SMP_IDENTITY_ADDR_INFORMATION:
      stat = smpParseIdentityAddrInfo( pMsg->pkt.pPayload, &(parsedMsg.idAddrInfo) );
      break;
      
    case SMP_SIGNING_INFORMATION:
      stat = smpParseSigningInfo( pMsg->pkt.pPayload, &(parsedMsg.signingInfo) );
      break;
      
//    case SMP_SECURITY_REQUEST:
//      stat = smpParseSecurityReq( pMsg->pkt.pPayload, &(parsedMsg.secReq) );
//      break;

    default:
      stat = bleIncorrectMode;
      failed.reason = SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED;
      break;  
  }
  
  // Process the incoming message
  if ( stat == SUCCESS )
  {
    failed.reason = smpProcessIncoming( pMsg->connHandle, cmdID, &parsedMsg );
  }

  // Check for fail indication  
  if ( failed.reason != SUCCESS )
  {
    // Send Pairing Failed message and close the pairing process
    smSendFailAndEnd( pMsg->connHandle, &failed );
  }
  else
  {
    // Reset the timeout
    smStartRspTimer();
  }
}

/*********************************************************************
 * @fn          smSendFailAndEnd
 *
 * @brief       Send the pairing failed message and end existing pairing.
 *
 * @param       connHandle - link ID
 * @param       pFailedMsg - Pairing Failed message 
 *
 * @return      SUCCESS if pairing failed sent
 *              otherwise failure.
 */
bStatus_t smSendFailAndEnd( uint16 connHandle, smpPairingFailed_t *pFailedMsg )
{
  bStatus_t stat;   // return value
  
  // Send Pairing Failed message
  stat = smSendPairingFailed( connHandle, pFailedMsg );
  
  // Clean up after error, end pairing process
  smEndPairing( pFailedMsg->reason );
  
  return ( stat );
}

/*********************************************************************
 * @fn          smProcessEncryptChange
 *
 * @brief       Process the HCI BLE Encryption Change Event.
 *
 * @param       connectionHandle - link ID
 * @param       reason - reason for change
 *
 * @return      TRUE - We are always expecting this message
 */
uint8 smProcessEncryptChange( uint16 connectionHandle, uint8 reason )
{
  uint8 sendBondEnd = FALSE;  // Assume not bonding

  // Check for the correct state and connection 
  if ( (pPairingParams) 
      && (pPairingParams->connectionHandle == connectionHandle) )
  {
    if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_STK )
    {
//      if ( pPairingParams->initiator )
//      {
//        smLinkSecurityReq_t *pSecReq = pPairingParams->pSecReqs;
//        smpPairingReq_t *pPairReq = pPairingParams->pPairDev;
//
//        // Wait for key distribute messages from Responder
//        if ( (pSecReq->keyDist.sEncKey) && (pPairReq->keyDist.sEncKey) )
//        {  
//          pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO;
//        }
//        else if ( (pSecReq->keyDist.sIdKey) && (pPairReq->keyDist.sIdKey) )
//        {
//          pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO;
//        }
//        else if ( (pSecReq->keyDist.sSign) && (pPairReq->keyDist.sSign) )
//        {
//          pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO;
//        }
//        else 
//        {
//          // Send the next key distribution message to the responder
//          if ( pfnInitiatorCBs && pfnInitiatorCBs->pfnSendNextKeyInfo )
//          {
//            pfnInitiatorCBs->pfnSendNextKeyInfo();
//          }
//        }
//      }
//      else
      {
        // Send the next key distribution message
        if ( pfnResponderCBs && pfnResponderCBs->pfnSendNextKeyInfo )
        {
          pfnResponderCBs->pfnSendNextKeyInfo();
        }
      }

      // If we are done, end the pairing process        
      if ( pPairingParams->state == SM_PAIRING_STATE_DONE )
      {
        // Exit pairing
        smEndPairing( SUCCESS );
      }
    }
    else if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_ENCRYPT )
    {
      // We are waiting for the encrypt complete to complete pairing
      if ( reason != SUCCESS )
      {
        // Generic bond error
        reason = bleGAPBondRejected;
      }
      smEndPairing( reason );
    }
    else
    {
      // wasn't expecting
      sendBondEnd = TRUE;
    }
  }
  else
  {
    // loaded key
    sendBondEnd = TRUE;
  }
  
  if ( sendBondEnd == TRUE )
  {
    if ( reason == SUCCESS )
    {
      linkDBItem_t *pLink; // connection information
      
      pLink = linkDB_Find( connectionHandle );
      if ( pLink )
      {
        // Change the connections state to include Encrypted
        pLink->stateFlags |= LINK_ENCRYPTED;
      }
    }

    // Send the bond complete event to the app/profile
    gapSendBondCompleteEvent( reason, connectionHandle );
  }
    
  return ( TRUE );  // We are always expecting this message
}

/*********************************************************************
 * @fn          smNextPairingState
 *
 * @brief       Do the next Pairing key distribution state.
 *
 * @param       none
 *
 * @return      none
 */
void smNextPairingState( void )
{
  if ( pPairingParams ) 
  {
    if ( pPairingParams->initiator == FALSE )
    {
      // Send next responder key distribution message
      if ( pfnResponderCBs && pfnResponderCBs->pfnSendNextKeyInfo )
      {
        pfnResponderCBs->pfnSendNextKeyInfo();
      }
    }
//    else
//    {
//      // Send next initiator key distribution message
//      if ( pfnInitiatorCBs && pfnInitiatorCBs->pfnSendNextKeyInfo )
//      {
//        pfnInitiatorCBs->pfnSendNextKeyInfo();
//      }
//    }
    
    if ( pPairingParams->state == SM_PAIRING_STATE_DONE )
    {
      smEndPairing( SUCCESS );
    }
  }
}

/*********************************************************************
 * @fn          sm_c1
 *
 * @brief       SM Confirm value generation function c1
 * 
 *  NOTE:       This function can only be called during a connection
 *              and during the pairing process.
 *
 * @param       pK - key is 128 bits, LSByte first
 * @param       pR - 128 bit, LSByte first
 * @param       pC1 - pointer to 128 bit results space
 *
 * @return      status
 */
bStatus_t sm_c1( uint8 *pK, uint8 *pR, uint8 *pC1 )
{
  bStatus_t stat;                   // return value
  uint8 pres[SMP_PAIRING_RSP_LEN];  // Pairing Response message (raw)
  uint8 preq[SMP_PAIRING_REQ_LEN];  // Pairing Request message (raw)
  uint8 iat;                        // initiator address type
  uint8 rat;                        // responder address type
  uint8 *pIA;                       // initiator address
  uint8 *pRA;                       // responder address
  smpPairingReq_t pairingStruct;    // Pairing Reqeust struct
  smpPairingReq_t *pPairingReq;      // Pairing Reqeust 
  smpPairingReq_t *pPairingRsp;      // Pairing Response
  linkDBItem_t *pLinkItem;           // Connection information
  
  // Make sure we are in the correct mode
  if ( pPairingParams == NULL )
  {
    return ( bleIncorrectMode );
  }

  // Find the connection  
  pLinkItem = linkDB_Find( pPairingParams->connectionHandle );
  if ( pLinkItem == NULL )
  {
    return ( bleNotConnected );
  }
  
  // Setup pairing request/response parameters
  smSetPairingReqRsp( &pairingStruct );

//  if ( pPairingParams->initiator ) 
//  {
//    // Setup all of the variables for an initiator
//    pPairingReq = &pairingStruct;
//    pPairingRsp = pPairingParams->pPairDev;
//    
//    iat = gapGetDevAddressMode();
//    pIA = gapGetDevAddress( FALSE );
//    rat = pLinkItem->addrType;
//    pRA = pLinkItem->addr;
//  }
//  else
  {
    // Setup all of the variables for a responder
    pPairingReq = pPairingParams->pPairDev;
    pPairingRsp = &pairingStruct;

    rat = gapGetDevAddressMode();
    pRA = gapGetDevAddress( FALSE );
    iat = pLinkItem->addrType;
    pIA = pLinkItem->addr;
  }

  // build the raw data for the Pairing Request and Pairing Response messages  
  smpBuildPairingRsp( pPairingRsp, pres );
  smpBuildPairingReq( pPairingReq, preq );

  // Calculate the cl 
  stat =  sm_c1new( pK, pR, pres, preq, iat, pIA, rat, pRA, pC1 );

  return ( stat );
}

/*********************************************************************
 * @fn          smpProcessIncoming
 *
 * @brief       Process incoming parsed SM message.
 *
 * @param       connHandle - connection Handle
 * @param       cmdID - command ID
 * @param       pParsedMsg - pointer to parsed message 
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_PASSKEY_ENTRY_FAILED
 *              SMP_PAIRING_FAILED_OOB_NOT_AVAIL
 *              SMP_PAIRING_FAILED_AUTH_REQ
 *              SMP_PAIRING_FAILED_CONFIRM_VALUE
 *              SMP_PAIRING_FAILED_NOT_SUPPORTED
 *              SMP_PAIRING_FAILED_ENC_KEY_SIZE
 *              SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 *              SMP_PAIRING_FAILED_REPEATED_ATTEMPTS
 */
static uint8 smpProcessIncoming( uint16 connHandle, uint8 cmdID, smpMsgs_t *pParsedMsg )
{
  linkDBItem_t *pLinkItem; // connection information
  uint8 reason = SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED; // return value
 
  // find the connection
  pLinkItem = linkDB_Find( connHandle );
  if ( pLinkItem == NULL )
  {
    return ( SMP_PAIRING_FAILED_UNSPECIFIED );
  }

//  if ( (gapProfileRole & GAP_PROFILE_CENTRAL) == GAP_PROFILE_CENTRAL )
//  {
//    // Process Initiator pairing messages
//    if ( pfnInitiatorCBs && pfnInitiatorCBs->pfnProcessMsg )
//    {
//      reason = pfnInitiatorCBs->pfnProcessMsg( pLinkItem, cmdID, pParsedMsg );
//    }
//  }
//  else
  {
    // Process Responder pairing messages
    if ( pfnResponderCBs && pfnResponderCBs->pfnProcessMsg )
    {
      reason = pfnResponderCBs->pfnProcessMsg( pLinkItem, cmdID, pParsedMsg );
    }
  }
  
  return ( reason );
}

/*********************************************************************
 * @fn          smProcessPairingReq
 *
 * @brief       Process Pairing Request.
 *
 * @param       pLinkItem - pointer to link item
 * @param       pParsedMsg - pointer to request 
 *
 * @return      void
 */
void smProcessPairingReq( linkDBItem_t *pLinkItem, gapPairingReq_t *pPairReq )
{
  if ( pfnResponderCBs && pfnResponderCBs->pfnProcessMsg )
  {
    uint8 reason;
    smpPairingReq_t pairingReq;
    
    pairingReq.ioCapability = pPairReq->ioCap;
    pairingReq.oobDataFlag = pPairReq->oobDataFlag;
    smUint8ToAuthReq( &(pairingReq.authReq), pPairReq->authReq );
    pairingReq.maxEncKeySize = pPairReq->maxEncKeySize;
    pairingReq.keyDist = pPairReq->keyDist;
    
    reason = pfnResponderCBs->pfnProcessMsg( pLinkItem, SMP_PAIRING_REQ, 
                                             (smpMsgs_t *)&pairingReq );
    if ( reason != SUCCESS )
    {
      smpPairingFailed_t failedMsg;
      failedMsg.reason = reason;

      smSendFailAndEnd( pLinkItem->connectionHandle, &failedMsg );
    }
  }
}

/*********************************************************************
 * @fn          smStartEncryption
 *
 * @brief       Perform Encrypt through HCI
 *
 * @param       connHandle - Connection Handle
 * @param       pLTK - pointer to 16 byte lkt
 * @param       div - div or ediv
 * @param       pRandNum - pointer to 8 byte random number
 * @param       keyLen - length of LTK (bytes)
 *
 * @return      SUCCESS
 *              INVALIDPARAMETER 
 *              other from HCI/LL
 */
//bStatus_t smStartEncryption( uint16 connHandle, uint8 *pLTK, uint16 div,
//                             uint8 *pRandNum, uint8 keyLen )
//{
////  if ( pfnInitiatorCBs && pfnInitiatorCBs->pfnStartEncryption )
////  {
////    return ( pfnInitiatorCBs->pfnStartEncryption( connHandle, pLTK, div, pRandNum, keyLen ) );
////  }
//
//  return ( INVALIDPARAMETER );
//}

/*********************************************************************
 * @fn          smGeneratePairingReqRsp
 *
 * @brief       Generate a pairing req or response
 *
 * @param       none
 *
 * @return      SUCCESS, FAILURE 
 */
bStatus_t smGeneratePairingReqRsp( void )
{
  if ( pPairingParams )
  {
    smpPairingReq_t pairingReq; // Structure to build message
    
    // Setup pairing request/response parameters
    smSetPairingReqRsp( &pairingReq );
    
//    if ( pPairingParams->initiator )
//    {
//      // Send Request
//      return ( smSendPairingReq( pPairingParams->connectionHandle, (smpMsgs_t *)(&pairingReq) ) );
//    }
//    else
    {
      // Send Response
      return ( smSendPairingRsp( pPairingParams->connectionHandle, (smpMsgs_t *)(&pairingReq) ) );
    }
  }
  else
  {
    return ( FAILURE );
  }
}

/*********************************************************************
 * @fn          smGenerateConfirm
 *
 * @brief       Generate a Pairing Confirm
 *
 * @param       none
 *
 * @return      SUCCESS
 */
bStatus_t smGenerateConfirm( void )
{
  smpPairingConfirm_t confirmMsg; // Parameters to build message
  
  // Copy the confirm 
  osal_memcpy( confirmMsg.confirmValue, 
                   pPairingParams->myComp.confirm, SMP_CONFIRM_LEN );

  // Send confirm message
  return ( smSendPairingConfirm( pPairingParams->connectionHandle, &confirmMsg ) );
}

/*********************************************************************
 * @fn          smGenerateRandMsg
 *
 * @brief       Generate a Pairing Random
 *
 * @param       none
 *
 * @return      SUCCESS
 */
bStatus_t smGenerateRandMsg( void )
{
  smpPairingRandom_t randMsg;  // Parameters to build message
  
  // Build rand
  VOID osal_memcpy( randMsg.randomValue, 
                   pPairingParams->myComp.rand, SMP_RANDOM_LEN );

  // Send Random message
  return ( smSendPairingRandom( pPairingParams->connectionHandle, &randMsg ) );
}

/*********************************************************************
 * @fn          smSavePairInfo
 *
 * @brief       Save the Pairing Req or Rsp information
 *
 * @param       pPair - info to save
 *
 * @return      SUCCESS 
 *              bleMemAllocError 
 *              bleInvalidRange - auth reqs don't match
 */
bStatus_t smSavePairInfo( smpPairingReq_t *pPair )
{
  bStatus_t ret = SUCCESS;
  
  // Allocate the pairing information
  pPairingParams->pPairDev = (smpPairingReq_t *)osal_mem_alloc( (uint16)sizeof( smpPairingReq_t ) );
  if ( pPairingParams->pPairDev )
  {
    smLinkSecurityReq_t *pSecReq = pPairingParams->pSecReqs;

    // Copy the pairing information into the pairingParam 
    VOID osal_memcpy( pPairingParams->pPairDev, pPair, (unsigned int)sizeof ( smpPairingReq_t ) );
    
    if ( pPairingParams->initiator == FALSE  )
    {
      keyDist_t *pSecKey = &(pSecReq->keyDist);
      keyDist_t *pPairKey = &(pPairingParams->pPairDev->keyDist);
    
      // Responder: merge our key distribution plans with the initiator's
      pSecKey->sEncKey = ( (pSecKey->sEncKey == pPairKey->sEncKey) && 
                           (pSecKey->sEncKey == TRUE) ) ? TRUE : FALSE;
      pSecKey->sIdKey =  ( (pSecKey->sIdKey == pPairKey->sIdKey) && 
                           (pSecKey->sIdKey == TRUE) ) ? TRUE : FALSE;
      pSecKey->sSign =   ( (pSecKey->sSign == pPairKey->sSign) && 
                           (pSecKey->sSign == TRUE) ) ? TRUE : FALSE;
      pSecKey->mEncKey = ( (pSecKey->mEncKey == pPairKey->mEncKey) && 
                           (pSecKey->mEncKey == TRUE) ) ? TRUE : FALSE;
      pSecKey->mIdKey =  ( (pSecKey->mIdKey == pPairKey->mIdKey) && 
                           (pSecKey->mIdKey == TRUE) ) ? TRUE : FALSE;
      pSecKey->mSign =   ( (pSecKey->mSign == pPairKey->mSign) && 
                           (pSecKey->mSign == TRUE) ) ? TRUE : FALSE;
    }

    // Determine the pairing type
    if ( (pPair->oobDataFlag) && (pSecReq->oobAvailable) )
    {
      pPairingParams->type = SM_PAIRING_TYPE_OOB;
    }  
    else if ( ((pPair->authReq.mitm) == 0) && ((pSecReq->authReq & SMP_AUTHREQ_MITM) == 0) )
    {
      // if either initiator and responder have no MITM
      pPairingParams->type = SM_PAIRING_TYPE_JUST_WORKS;
    }
    else if ( (pPair->authReq.mitm) || (pSecReq->authReq & SMP_AUTHREQ_MITM) )
    {
      uint8 initiatorIO;
      uint8 responderIO;

      if ( pPairingParams->initiator )
      {
        initiatorIO = pSecReq->ioCaps;
        responderIO = pPair->ioCapability;
      }
      else
      {
        responderIO = pSecReq->ioCaps;
        initiatorIO = pPair->ioCapability;
      }
      
      // Update the pairingParam type field
      ret = smDetermineIOCaps( initiatorIO, responderIO );
    }
    else
    {
      ret = bleInvalidRange;
    }
  }
  else
  {
    ret = bleMemAllocError;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          smDetermineIOCaps
 *
 * @brief       Save the Pairing Req or Rsp information
 *
 * @param       initiatorIO - initiator's IO Capabilities
 * @param       responderIO - responder's IO Capabilities
 *
 * NOTE: Also updated is pPairingParams->type
 *
 * @return      SUCCESS 
 *              bleInvalidRange - IO capability out of range
 */
static bStatus_t smDetermineIOCaps( uint8 initiatorIO, uint8 responderIO )
{
  if ( (initiatorIO > SMP_IO_CAP_KEYBOARD_DISPLAY) || (responderIO > SMP_IO_CAP_KEYBOARD_DISPLAY) )
  {
    return ( bleInvalidRange );
  }
  
  // From the Spec matrix determine the pairing type
  pPairingParams->type = IOCapMatrix[responderIO][initiatorIO];
  
  return ( SUCCESS );
}
                                                                       
/*********************************************************************
 * @fn          smPairingSendEncInfo
 *
 * @brief       Send SM Encryption Information message
 *
 * @param       connHandle - connection handle
 * @param       pLTK - pointer to LTK
 *
 * @return      SUCCESS
 *              INVALIDPARAMETER 
 *              other from HCI/LL
 */
void smPairingSendEncInfo( uint16 connHandle, uint8 *pLTK ) 
{
  // The smpEncInfo_t only has one field in it.
  smSendEncInfo( connHandle, (smpEncInfo_t *)pLTK );
}

/*********************************************************************
 * @fn          smPairingSendMasterID
 *
 * @brief       Send SM Master Identification message
 *
 * @param       connHandle - connection handle
 * @param       ediv - enhanced div
 * @param       pRand - pointer to 8 byte random number string
 *
 * @return      SUCCESS
 *              INVALIDPARAMETER 
 *              other from HCI/LL
 */
void smPairingSendMasterID( uint16 connHandle, uint16 ediv, uint8 *pRand ) 
{
  smpMasterID_t masterMsg;  // Message structure
  
  // Fill parameters
  masterMsg.ediv = ediv;
  VOID osal_memcpy( masterMsg.rand, pRand, B_RANDOM_NUM_SIZE );
  
  // Send Master ID message
  VOID smSendMasterID( connHandle, &masterMsg );
}
                                                                        
/*********************************************************************
 * @fn          smPairingSendIdentityInfo
 *
 * @brief       Send SM Identity Information message
 *
 * @param       connHandle - connection handle
 * @param       pIRK - pointer to IRK
 *
 * @return      SUCCESS
 *              INVALIDPARAMETER 
 *              other from HCI/LL
 */
void smPairingSendIdentityInfo( uint16 connHandle, uint8 *pIRK ) 
{
  VOID smSendIdentityInfo( connHandle, (smpIdentityInfo_t *)pIRK );
}
                                                                        
/*********************************************************************
 * @fn          smPairingSendIdentityAddrInfo
 *
 * @brief       Send SM Identity Addr Information message
 *
 * @param       connHandle - connection handle
 * @param       addrType - address type
 * @param       pMACAddr - pointer to BD_ADDR
 *
 * @return      SUCCESS
 *              INVALIDPARAMETER 
 *              other from HCI/LL
 */
void smPairingSendIdentityAddrInfo( uint16 connHandle, uint8 addrType, uint8 *pMACAddr ) 
{
  smpIdentityAddrInfo_t identityAddrMsg;  // Message structure

  // Fill in parameters
  identityAddrMsg.addrType = addrType;
  VOID osal_memcpy( identityAddrMsg.bdAddr, pMACAddr, B_ADDR_LEN );
  
  // Send message
  VOID smSendIdentityAddrInfo( connHandle, &identityAddrMsg );
}
                                                                        
/*********************************************************************
 * @fn          smPairingSendSingingInfo
 *
 * @brief       Send SM Signing Information message
 *
 * @param       connHandle - connection handle
 * @param       pSRK - pointer to SRK
 *
 * @return      SUCCESS
 *              INVALIDPARAMETER 
 *              other from HCI/LL
 */
void smPairingSendSingingInfo( uint16 connHandle, uint8 *pSRK ) 
{
  VOID smSendSigningInfo( connHandle, (smpSigningInfo_t *)pSRK );
}

/*********************************************************************
 * @fn          smFreePairingParams
 *
 * @brief       Free memory used in the Pairing Parameters
 *
 * @param       none
 *
 * @return      none
 */
static void smFreePairingParams( void )
{
#if !defined ( HOLD_PAIRING_PARAMETERS )
  if ( pPairingParams )
  {
    // secReqs is not allocated in SM it's a borrowed struct from GAP.
    // So, no dealloc here.
    
    if ( pPairingParams->pPairDev )
    {
      osal_mem_free( pPairingParams->pPairDev );
    }
    
    if ( pPairingParams->pEncParams )
    {
      osal_mem_free( pPairingParams->pEncParams );
    }
    
    if ( pPairingParams->pDevEncParams )
    {
      osal_mem_free( pPairingParams->pDevEncParams );
    }
    
    if ( pPairingParams->pIdInfo )
    {
      osal_mem_free( pPairingParams->pIdInfo );
    }
    
    if ( pPairingParams->pSigningInfo )
    {
      osal_mem_free( pPairingParams->pSigningInfo );
    }
    
    osal_mem_free( pPairingParams ); 
    pPairingParams = NULL;
  }
#endif
  
  // Clear the SM Timeout
  smStopRspTimer();
}

/*********************************************************************
 * @fn          smEndPairing
 *
 * @brief       Pairing mode has ended.  Yeah. Notify the GAP and free
 *              up the memory used.
 *
 * @param       status - how was the pairing completed
 *
 * @return      none
 */
void smEndPairing( uint8 status )
{
  if ( pPairingParams )
  {
    gapPairingCompleteCB( status, pPairingParams->initiator, 
                          pPairingParams->connectionHandle, 
                          pPairingParams->authState,
                          pPairingParams->pEncParams,
                          pPairingParams->pDevEncParams,
                          pPairingParams->pIdInfo,
                          pPairingParams->pSigningInfo );
    
    // free up the pPairingParams
    smFreePairingParams();
  }      
}

/*********************************************************************
 * @fn          smDetermineKeySize
 *
 * @brief       Determine the maximum encryption key size.
 *
 * @param       none
 *
 * @return      the negotiated key size
 */
uint8 smDetermineKeySize( void )
{
  uint8 keySize = KEYLEN;
  
  if ( pPairingParams )
  {
    if ( pPairingParams->pPairDev && pPairingParams->pSecReqs )
    {
      if ( pPairingParams->pPairDev->maxEncKeySize < pPairingParams->pSecReqs->maxEncKeySize )
      {
        keySize = pPairingParams->pPairDev->maxEncKeySize;
      }
      else
      {
        keySize = pPairingParams->pSecReqs->maxEncKeySize;
      }
    }
  }
  
  return ( keySize );
}

/*********************************************************************
 * @fn          smSetPairingReqRsp
 *
 * @brief       Setup pairing request/response parameters.
 *
 * @param       pReq - Request/Response to be set
 *
 * @return      none
 */
static void smSetPairingReqRsp( smpPairingReq_t *pReq )
{
  if ( pPairingParams && pPairingParams->pSecReqs )
  {
    smLinkSecurityReq_t *pSecReq = pPairingParams->pSecReqs;

    pReq->ioCapability = pSecReq->ioCaps;
    pReq->oobDataFlag = pSecReq->oobAvailable;
    smUint8ToAuthReq( &(pReq->authReq), pSecReq->authReq );
    pReq->maxEncKeySize = pSecReq->maxEncKeySize;
    pReq->keyDist = pSecReq->keyDist;
  }
  else
  {
    VOID osal_memset( pReq, 0, sizeof( smpPairingReq_t ) );
  }
}

/*********************************************************************
*********************************************************************/
