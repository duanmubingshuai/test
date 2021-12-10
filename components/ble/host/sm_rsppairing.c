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
  Filename:       sm_rsppairing.c
  Revised:        
  Revision:       

  Description:    This file contains the SM Responder Pairing Manager.


**************************************************************************************************/


/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "l2cap.h"
#include "gap_internal.h"
#include "linkdb.h"
#include "sm.h"
#include "sm_internal.h"
#include "smp.h"
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

/*********************************************************************
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

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 smpResponderProcessIncoming( linkDBItem_t *pLinkItem, uint8 cmdID, smpMsgs_t *pParsedMsg );
static uint8 smpResponderProcessPairingConfirm( smpPairingConfirm_t *pParsedMsg );
static uint8 smpResponderProcessPairingRandom( smpPairingRandom_t *pParsedMsg );
static uint8 smpResponderProcessEncryptionInformation( smpEncInfo_t *pParsedMsg );
static uint8 smpResponderProcessMasterID( smpMasterID_t *pParsedMsg );
static uint8 smpResponderProcessIdentityInfo( smpIdentityInfo_t *pParsedMsg );
static uint8 smpResponderProcessIdentityAddrInfo( smpIdentityAddrInfo_t *pParsedMsg );
static uint8 smpResponderProcessSigningInfo( smpSigningInfo_t *pParsedMsg );

static void smResponderSendNextKeyInfo( void );
static uint8 smResponderProcessLTKReq( uint16 connectionHandle, uint8 *pRandom, uint16 encDiv );

/*********************************************************************
 * RESPONDER CALLBACKS
 */

// SM Responder Callbacks
static smResponderCBs_t smResponderCBs =
{
  smpResponderProcessIncoming, // Process SMP Message Callback
  smResponderSendNextKeyInfo,  // Send Next Key Message Callback
  smResponderProcessLTKReq     // HCI BLE LTK Request Callback
};

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * FUNCTIONS - SLAVE API - Only use these in a slave device
 */

/*********************************************************************
 * Initialize SM Responder on a slave device.
 *
 * Public function defined in sm.h.
 */
bStatus_t SM_ResponderInit( void )
{
  if ( gapProfileRole & GAP_PROFILE_PERIPHERAL )
  {
    // Set up Responder's processing function
    smRegisterResponder( &smResponderCBs );
  }
  else
  {
    smRegisterResponder( NULL );
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn          smResponderProcessLTKReq
 *
 * @brief       Process the HCI BLE LTK Request Event on Responder.
 *
 * @param       connectionHandle - link ID
 * @param       pRandom - 8 byte random number
 * @param       encDiv - encryption diversifier
 *
 * @return      TRUE - We are always expecting this message
 */
static uint8 smResponderProcessLTKReq( uint16 connectionHandle, uint8 *pRandom, uint16 encDiv )
{
  uint16 connHandle = 0;      // Found connection handle
  uint8  cmdLtk[KEYLEN];      // Place to hold the LTK or STK
  bStatus_t stat = FAILURE;   // Assume failure until verified

  // Clear the LTK
  VOID osal_memset( cmdLtk, 0, KEYLEN );

  // Check the parameters
  if ( (pPairingParams)
        && (pPairingParams->connectionHandle == connectionHandle)
        && (pPairingParams->initiator == FALSE) )
  {
    // Do we want the STK?
    if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_STK )
    {
      uint8 ltk[KEYLEN];  // A place to build the LTK

      // Received Responder, generate STK
      if ( sm_s1( pPairingParams->tk, pPairingParams->myComp.rand,
                 pPairingParams->devComp.rand, ltk ) == SUCCESS )
      {
        // Copy the STK to the LTK space
        VOID osal_memcpy( cmdLtk, ltk, smDetermineKeySize() );
        connHandle = connectionHandle;
        stat = SUCCESS;
      }
    }
    // How about the LTK?
    else if ( (pPairingParams->pEncParams)
             && (pPairingParams->state == SM_PAIRING_STATE_WAIT_ENCRYPT) )
    {
      // Check DIV and Rand first
      if ( (encDiv == pPairingParams->pEncParams->div)
          && (osal_memcmp( pRandom, pPairingParams->pEncParams->rand, B_RANDOM_NUM_SIZE ) == TRUE) )
      {
        // Must be the LTK encryption
        VOID osal_memcpy( cmdLtk, pPairingParams->pEncParams->ltk, smDetermineKeySize() );
        connHandle = connectionHandle;
        stat = SUCCESS;
      }
      else
      {
        // Rejected, end bonding
        smEndPairing( bleGAPBondRejected );
      }
    }
    else
    {
      // Not expecting but, if it's available - load the key
      if ( (pPairingParams->pEncParams)
          && (encDiv == pPairingParams->pEncParams->div)
        && (osal_memcmp( pRandom, pPairingParams->pEncParams->rand, B_RANDOM_NUM_SIZE ) == TRUE) )
      {
        VOID osal_memcpy( cmdLtk, pPairingParams->pEncParams->ltk, pPairingParams->pEncParams->keySize );
        connHandle = connectionHandle;
        stat = SUCCESS;
      }
    }
  }

  // Not directly handled, but make best attempt to load the key
  if ( stat != SUCCESS )
  {
    linkDBItem_t *pLinkItem;  // connection information

    // Find the connection
    pLinkItem = linkDB_Find( connectionHandle );
//    if ( pLinkItem && (pLinkItem->pEncParams)
//        && (encDiv == pLinkItem->pEncParams->div)
//        && (osal_memcmp( pRandom, pLinkItem->pEncParams->rand, B_RANDOM_NUM_SIZE ) == TRUE) )
//    {
//      // Use the key in the connection information
//      osal_memcpy( cmdLtk, pLinkItem->pEncParams->ltk, pLinkItem->pEncParams->keySize );
//      connHandle = connectionHandle;
//      stat = SUCCESS;
//    }
	if( pLinkItem )
	{
		uint8 lkey[16]={0};
		  uint8 public_addr[6] = {0};
		  LL_ReadBDADDR(public_addr);
		  osal_memcpy(lkey, public_addr, 6);
		  sm_d1(lkey, encDiv, cmdLtk);
		  stat = SUCCESS;
	}
  }

  if ( stat == SUCCESS )
  {
    HCI_LE_LtkReqReplyCmd( connHandle, cmdLtk );
  }
  else
  {
    HCI_LE_LtkReqNegReplyCmd( connectionHandle );
  }

  return ( TRUE );  // We are always expecting this message
}

/*********************************************************************
 * @fn          smpResponderProcessIncoming
 *
 * @brief       Process incoming parsed SM Responder message.
 *
 * @param       pLinkItem - connection information
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
static uint8 smpResponderProcessIncoming( linkDBItem_t *pLinkItem, uint8 cmdID, smpMsgs_t *pParsedMsg )
{
  uint8 reason = SUCCESS;   // return value

  // check for pairing mode
  if ( pPairingParams == NULL )
  {
    if ( cmdID == SMP_PAIRING_REQ )
    {
      smpPairingReq_t *pairingReq = (smpPairingReq_t *)pParsedMsg;

      // Notify the app
      gapSendPairingReqEvent( SUCCESS, pLinkItem->connectionHandle,
                              pairingReq->ioCapability,
                              pairingReq->oobDataFlag,
                              smAuthReqToUint8( &(pairingReq->authReq) ),
                              pairingReq->maxEncKeySize,
                              pairingReq->keyDist );
      return ( SUCCESS );
    }
    else
    {
      // Ignore the message, don't respond
      return ( SUCCESS );
    }
  }

  // We can only handle one pairing at a time
  if ( pPairingParams->connectionHandle != pLinkItem->connectionHandle )
  {
    return ( SMP_PAIRING_FAILED_UNSPECIFIED );
  }

  if ( pPairingParams->initiator == TRUE )
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }

  // Process the pairing messages
  switch ( cmdID )
  {
    case SMP_PAIRING_REQ:
      reason = smpResponderProcessPairingReq( (smpPairingReq_t *)pParsedMsg );
      break;

    case SMP_PAIRING_CONFIRM:
      reason = smpResponderProcessPairingConfirm( (smpPairingConfirm_t *)pParsedMsg );
      break;

    case SMP_PAIRING_RANDOM:
      reason = smpResponderProcessPairingRandom( (smpPairingRandom_t *)pParsedMsg );
      break;

    case SMP_ENCRYPTION_INFORMATION:
      reason = smpResponderProcessEncryptionInformation( (smpEncInfo_t *)pParsedMsg );
      break;

    case SMP_MASTER_IDENTIFICATION:
      reason = smpResponderProcessMasterID( (smpMasterID_t *)pParsedMsg );
      break;

    case SMP_IDENTITY_INFORMATION:
      reason = smpResponderProcessIdentityInfo( (smpIdentityInfo_t *)pParsedMsg );
      break;

    case SMP_IDENTITY_ADDR_INFORMATION:
      reason = smpResponderProcessIdentityAddrInfo( (smpIdentityAddrInfo_t *)pParsedMsg );
      break;

    case SMP_SIGNING_INFORMATION:
      reason = smpResponderProcessSigningInfo( (smpSigningInfo_t *)pParsedMsg );
      break;

    case SMP_PAIRING_FAILED:
      smEndPairing( pParsedMsg->pairingFailed.reason );
      break;

    default:
      reason = SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED;
      break;
  }

  return ( reason );
}

/*********************************************************************
 * @fn          smpResponderProcessPairingReq
 *
 * @brief       Process incoming parsed Pairing Request.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 *              SMP_PAIRING_FAILED_AUTH_REQ
 */
uint8 smpResponderProcessPairingReq( smpPairingReq_t *pParsedMsg )
{
  uint8 reason = SUCCESS; // return value
  bStatus_t stat; // status field

  // Save the pairing request information into pPairingParams
  stat = smSavePairInfo( pParsedMsg );
  if ( stat == SUCCESS )
  {
    // Send the response
    smGeneratePairingReqRsp();

    // Check if both sides have bonding
    if ( (pPairingParams->pSecReqs->authReq & SM_AUTH_STATE_BONDING)
        && (pPairingParams->pPairDev->authReq.bonding == SM_AUTH_REQ_BONDING) )
    {
      pPairingParams->authState |= SM_AUTH_STATE_BONDING;
    }

    // Do we need a passkey?
    if ( (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS)
        || (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_RESPONDER_INPUTS)
        || (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_BOTH_INPUTS) )
    {
      uint8 type;

      // Determine the passkey input/output requirements
      if ( pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS )
      {
        type = SM_PASSKEY_TYPE_DISPLAY;
      }
      else
      {
        type = SM_PASSKEY_TYPE_INPUT;
      }

      // Ask the app/profile for passkey
      gapPasskeyNeededCB( pPairingParams->connectionHandle, type );
      pPairingParams->authState |= SM_AUTH_STATE_AUTHENTICATED;

      pPairingParams->state = SM_PAIRING_STATE_WAIT_CONFIRM_PASSKEY;
    }
    else if ( (pPairingParams->type == SM_PAIRING_TYPE_JUST_WORKS)
             || (pPairingParams->type == SM_PAIRING_TYPE_OOB) )
    {
      // Get ready for the Confirm
      pPairingParams->state = SM_PAIRING_STATE_WAIT_CONFIRM;

      // Setup TK
      if ( pPairingParams->type == SM_PAIRING_TYPE_JUST_WORKS )
      {
        // Just Wait
        VOID osal_memset( pPairingParams->tk, 0, KEYLEN );
      }
      else
      {
        // OOB
        VOID osal_memcpy( pPairingParams->tk, pPairingParams->pSecReqs->oob, KEYLEN );
        pPairingParams->authState |= SM_AUTH_STATE_AUTHENTICATED;
      }

      // Generate Rand (SRand)
      smGenerateRandBuf( pPairingParams->myComp.rand, SMP_RANDOM_LEN );
    }
  }
  else if ( stat == bleInvalidRange )
  {
    reason = SMP_PAIRING_FAILED_AUTH_REQ;
  }
  else
  {
    reason = SMP_PAIRING_FAILED_UNSPECIFIED;
  }

  return ( reason );
}

/*********************************************************************
 * @fn          smpResponderProcessPairingConfirm
 *
 * @brief       Process incoming parsed Pairing Confirm.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpResponderProcessPairingConfirm( smpPairingConfirm_t *pParsedMsg )
{
  uint8 reason = SUCCESS;  // return value

  VOID osal_memcpy( pPairingParams->devComp.confirm, pParsedMsg->confirmValue, KEYLEN );

  // Received Initiator Confirm
  if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_CONFIRM_PASSKEY )
  {
    pPairingParams->state = SM_PAIRING_STATE_WAIT_PASSKEY;
  }
  else
  {
    // Generate Confirm (MConfirm)
    sm_c1( pPairingParams->tk,
        pPairingParams->myComp.rand,
        pPairingParams->myComp.confirm );

#if defined ( TESTMODES )
    if ( GAP_GetParamValue( TGAP_SM_TESTCODE ) == SM_TESTMODE_SEND_BAD_CONFIRM )
    {
      VOID osal_memset( pPairingParams->myComp.confirm, 0, KEYLEN );
    }
#endif // TESTMODE

    // Send our own confirm
    if ( smGenerateConfirm() != SUCCESS )
    {
      reason = SMP_PAIRING_FAILED_UNSPECIFIED;
    }
    pPairingParams->state = SM_PAIRING_STATE_WAIT_RANDOM;
  }

  return ( reason );
}

/*********************************************************************
 * @fn          smpResponderProcessPairingRandom
 *
 * @brief       Process incoming parsed Pairing Random.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_CONFIRM_VALUE
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpResponderProcessPairingRandom( smpPairingRandom_t *pParsedMsg )
{
  uint8 reason = SUCCESS;   // return value
  uint8 confirm[KEYLEN];    // working area to calculate a confirm value

  VOID osal_memcpy( pPairingParams->devComp.rand, pParsedMsg->randomValue, SMP_RANDOM_LEN );

  // Check device's Confirm value
  sm_c1( pPairingParams->tk,
        pPairingParams->devComp.rand,
        confirm );

  // Make sure that the calculated confirm matches the confirm from the other device
  if ( osal_memcmp( confirm, pPairingParams->devComp.confirm, KEYLEN ) == TRUE )
  {
    // Received Initiator's Random, so send our own Random
    if ( smGenerateRandMsg() != SUCCESS )
    {
      reason = SMP_PAIRING_FAILED_UNSPECIFIED;
    }
    else
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_STK;
    }
  }
  else
  {
    reason = SMP_PAIRING_FAILED_CONFIRM_VALUE;
  }

  return ( reason );
}

/*********************************************************************
 * @fn          smpResponderProcessEncryptionInformation
 *
 * @brief       Process incoming parsed Encryption Information.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpResponderProcessEncryptionInformation( smpEncInfo_t *pParsedMsg )
{
  if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO )
  {
    // Save off the connected device's encryption information (LTK and key size)
    if ( pPairingParams->pDevEncParams == NULL )
    {
      pPairingParams->pDevEncParams = (smSecurityInfo_t *)osal_mem_alloc( (uint16)sizeof (smSecurityInfo_t ) );
    }

    if ( pPairingParams->pDevEncParams )
    {
      VOID osal_memcpy( pPairingParams->pDevEncParams->ltk, pParsedMsg->ltk, KEYLEN );
      pPairingParams->pDevEncParams->keySize = smDetermineKeySize();

      pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO;

      return ( SUCCESS );
    }
  }

  return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
}

/*********************************************************************
 * @fn          smpResponderProcessMasterID
 *
 * @brief       Process incoming parsed Master Identification.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpResponderProcessMasterID( smpMasterID_t *pParsedMsg )
{
  if ( (pPairingParams->pDevEncParams != NULL)
      && (pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO) )
  {
    // Save off the rest of the connected device's encryption information
    pPairingParams->pDevEncParams->div = pParsedMsg->ediv;
    VOID osal_memcpy( pPairingParams->pDevEncParams->rand, pParsedMsg->rand, B_RANDOM_NUM_SIZE );

    // Setup the next state
    if ( (pPairingParams->pSecReqs->keyDist.mIdKey) && (pPairingParams->pPairDev->keyDist.mIdKey) )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO;
    }
    else if ( (pPairingParams->pSecReqs->keyDist.mSign) && (pPairingParams->pPairDev->keyDist.mSign) )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO;
    }
    else
    {
      // Exit pairing
      smEndPairing( SUCCESS );
    }

    return ( SUCCESS );
  }
  else
  {
    return (  SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          smpResponderProcessIdentityInfo
 *
 * @brief       Process incoming parsed Identity Information.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpResponderProcessIdentityInfo( smpIdentityInfo_t *pParsedMsg )
{
  if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO )
  {
    // Save off the info
    if ( pPairingParams->pIdInfo == NULL )
    {
      pPairingParams->pIdInfo = (smIdentityInfo_t *)osal_mem_alloc( (uint16)sizeof (smIdentityInfo_t ) );
    }
    if ( pPairingParams->pIdInfo )
    {
      VOID osal_memcpy( pPairingParams->pIdInfo->irk, pParsedMsg->irk, KEYLEN );
    }

    // Determine next state
    pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO;

    return ( SUCCESS );
  }
  else
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          smpResponderProcessIdentityAddrInfo
 *
 * @brief       Process incoming parsed Identity Address Information.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpResponderProcessIdentityAddrInfo( smpIdentityAddrInfo_t *pParsedMsg )
{
  if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO )
  {
    VOID osal_memcpy( pPairingParams->pIdInfo->bd_addr, pParsedMsg->bdAddr, B_ADDR_LEN );

    if ( (pPairingParams->pSecReqs->keyDist.mSign) && (pPairingParams->pPairDev->keyDist.mSign) )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO;
    }
    else
    {
      // All done
      smEndPairing( SUCCESS );
    }

    return ( SUCCESS );
  }
  else
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          smpResponderProcessSigningInfo
 *
 * @brief       Process incoming parsed Signing Information.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpResponderProcessSigningInfo( smpSigningInfo_t *pParsedMsg )
{
  if ( pPairingParams )
  {
    if ( pPairingParams->pSigningInfo == NULL )
    {
      pPairingParams->pSigningInfo = (smSigningInfo_t *)osal_mem_alloc( (uint16)sizeof( smSigningInfo_t ) );
      if ( pPairingParams->pSigningInfo == NULL )
      {
        // Only error available for memory error, this will end the pairing process
        return ( SMP_PAIRING_FAILED_UNSPECIFIED );
      }
    }

    // Copy signature information
    if ( pPairingParams->pSigningInfo )
    {
      VOID osal_memcpy( pPairingParams->pSigningInfo->srk, pParsedMsg->signature, KEYLEN );
      pPairingParams->pSigningInfo->signCounter = GAP_INIT_SIGN_COUNTER;
    }

    // All done
    smEndPairing( SUCCESS );

    return ( SUCCESS );
  }
  else
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          smResponderSendNextKeyInfo
 *
 * @brief       Responder role: sends next key message, and sets state
 *              for next event.
 *
 * @param       none
 *
 * @return      none
 */
static void smResponderSendNextKeyInfo( void )
{
  if ( pPairingParams->initiator == FALSE )
  {
    smLinkSecurityReq_t *pSecReq = pPairingParams->pSecReqs;
    smpPairingReq_t *pPairReq = pPairingParams->pPairDev;
    uint8 state = pPairingParams->state;

    // Determine key to send
    if ( state == SM_PAIRING_STATE_WAIT_STK )
    {
      if ( (pSecReq->keyDist.sEncKey) && (pPairReq->keyDist.sEncKey) )
      {
        state = SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO;
      }
      else if ( (pSecReq->keyDist.sIdKey) && (pPairReq->keyDist.sIdKey) )
      {
        state = SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO;
      }
      else if ( (pSecReq->keyDist.sSign) && (pPairReq->keyDist.sSign) )
      {
        state = SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO;
      }
    }

    // Send the correct message
    switch ( state )
    {
      case SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO:
        // STK is setup, so send Encryption Information.
        if ( pPairingParams->pEncParams == NULL )
        {
          pPairingParams->pEncParams = (smSecurityInfo_t *)osal_mem_alloc( (uint16)sizeof (smSecurityInfo_t ) );
        }

        if ( pPairingParams->pEncParams )
        {
          smSecurityInfo_t *pEnc = pPairingParams->pEncParams;

          // For now, temp random the LTK, EDIV and RAND
          pEnc->div = osal_rand();
          osal_memset( pEnc->ltk, 0, KEYLEN );
		  uint8 lkey[16]={0};
		  uint8 public_addr[6] = {0};
		  LL_ReadBDADDR(public_addr);
		  osal_memcpy(lkey, public_addr, 6);
		  sm_d1(lkey, pEnc->div, pEnc->ltk);
//          smGenerateRandBuf( pEnc->ltk, smDetermineKeySize() );
          smGenerateRandBuf( pEnc->rand, B_RANDOM_NUM_SIZE );
          pEnc->keySize = smDetermineKeySize();

          // Send the Encryption Info
          smPairingSendEncInfo( pPairingParams->connectionHandle, pEnc->ltk );
        }
        break;

      case SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO:
        if ( pPairingParams->pEncParams )
        {
          smPairingSendMasterID( pPairingParams->connectionHandle,
                                pPairingParams->pEncParams->div,
                                pPairingParams->pEncParams->rand );
        }
        break;

      case SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO:
        smPairingSendIdentityInfo( pPairingParams->connectionHandle, gapGetIRK() );
        break;

      case SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO:
        {
          uint8 getRealAddr = TRUE;
          uint8 addrType = gapGetDevAddressMode();
          if ( addrType == ADDRTYPE_STATIC )
          {
            getRealAddr = FALSE;
          }
          else
          {
            addrType = ADDRTYPE_PUBLIC;
          }
          smPairingSendIdentityAddrInfo( pPairingParams->connectionHandle,
                      addrType, gapGetDevAddress( getRealAddr ) );
        }
        break;

      case SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO:
        smPairingSendSingingInfo( pPairingParams->connectionHandle, gapGetSRK() );
        break;

      default:
        break;
    }

    // Determine the state
    if ( state == SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO;
    }
    else if ( state == SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO )
    {
      if ( (pSecReq->keyDist.sIdKey) && (pPairReq->keyDist.sIdKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO;
      }
      else if ( (pSecReq->keyDist.sSign) && (pPairReq->keyDist.sSign) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO;
      }
      else if ( (pSecReq->keyDist.mEncKey) && (pPairReq->keyDist.mEncKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO;
      }
      else if ( (pSecReq->keyDist.mIdKey) && (pPairReq->keyDist.mIdKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO;
      }
      else if ( (pSecReq->keyDist.mSign) && (pPairReq->keyDist.mSign) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO;
      }
      else
      {
        pPairingParams->state = SM_PAIRING_STATE_DONE;
      }
    }
    else if ( state == SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO )
    {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO;
    }
    else if ( state == SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO )
    {
      if ( (pSecReq->keyDist.sSign) && (pPairReq->keyDist.sSign)  )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO;
      }
      else if ( (pSecReq->keyDist.mEncKey) && (pPairReq->keyDist.mEncKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO;
      }
      else if ( (pSecReq->keyDist.mIdKey) && (pPairReq->keyDist.mIdKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO;
      }
      else if ( (pSecReq->keyDist.mSign) && (pPairReq->keyDist.mSign) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO;
      }
      else
      {
        pPairingParams->state = SM_PAIRING_STATE_DONE;
      }
    }
    else
    {
      if ( (pSecReq->keyDist.mEncKey) && (pPairReq->keyDist.mEncKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO;
      }
      else if ( (pSecReq->keyDist.mIdKey) && (pPairReq->keyDist.mIdKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO;
      }
      else if ( (pSecReq->keyDist.mSign) && (pPairReq->keyDist.mSign) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO;
      }
      else
      {
        pPairingParams->state = SM_PAIRING_STATE_DONE;
      }
    }

    if ( (pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO) )
    {
      linkDBItem_t *pLinkItem;
      uint32 timeout;

      pLinkItem = linkDB_Find( pPairingParams->connectionHandle );
      if ( pLinkItem != NULL )
      {
        // Make the timeout 1.5 * connInterval (connInterval = 1.25 ms)
        timeout = pLinkItem->connInterval;
        timeout += pLinkItem->connInterval / 2;
      }
      else
      {
        timeout = SM_PAIRING_STATE_WAIT;
      }

      // Set up the next send
      VOID osal_start_timerEx( smTaskID, SM_PAIRING_STATE_EVT, timeout );
    }
  }
}



/*********************************************************************
*********************************************************************/
