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
  Filename:       sm_intpairing.c
  Revised:        
  Revision:       

  Description:    This file contains the SM Initiator Pairing Manager.


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
static uint8 smpInitiatorProcessIncoming( linkDBItem_t *pLinkItem, uint8 cmdID, smpMsgs_t *pParsedMsg );
static uint8 smpInitiatorProcessPairingRsp( smpPairingRsp_t *parsedMsg );
static uint8 smpInitiatorProcessPairingConfirm( smpPairingConfirm_t *pParsedMsg );
static uint8 smpInitiatorProcessPairingRandom( smpPairingRandom_t *pParsedMsg );
static uint8 smpInitiatorProcessEncryptionInformation( smpEncInfo_t *pParsedMsg );
static uint8 smpInitiatorProcessMasterID( smpMasterID_t *pParsedMsg );
static uint8 smpInitiatorProcessIdentityInfo( smpIdentityInfo_t *pParsedMsg );
static uint8 smpInitiatorProcessIdentityAddrInfo( smpIdentityAddrInfo_t *pParsedMsg );
static uint8 smpInitiatorProcessSigningInfo( smpSigningInfo_t *pParsedMsg );

static void setupInitiatorKeys( void );
static void smInitiatorSendNextKeyInfo( void );

/*********************************************************************
 * INITIATOR CALLBACKS
 */

// SM Initiator Callbacks
static smInitiatorCBs_t smInitiatorCBs =
{
  smpInitiatorProcessIncoming, // Process SMP Message Callback
  smInitiatorSendNextKeyInfo,  // Send Next Key Message Callback
  SM_StartEncryption           // Start Encrypt Callback
};

/*********************************************************************
 * API FUNCTIONS
 */


/*********************************************************************
 * FUNCTIONS - MASTER API - Only use these in a master device
 */

/*********************************************************************
 * Initialize SM Initiator on a master device.
 *
 * Public function defined in sm.h.
 */
bStatus_t SM_InitiatorInit( void )
{
  if ( gapProfileRole & GAP_PROFILE_CENTRAL )
  {
    // Set up Initiator's processing function
    smRegisterInitiator( &smInitiatorCBs );
  }
  else
  {
    smRegisterInitiator( NULL );
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn          smEncLTK
 *
 * @brief       Start LKT Encryption on an Initiator.
 *
 * @param       none
 *
 * @return      none
 */
void smEncLTK( void )
{
  // Make sure we are in the right state
  if ( (pPairingParams) && (pPairingParams->initiator)
      && (pPairingParams->state == SM_PAIRING_STATE_WAIT_ENCRYPT)
        && (pPairingParams->pDevEncParams) )
  {
    if ( SM_StartEncryption( pPairingParams->connectionHandle,
                            pPairingParams->pDevEncParams->ltk,
                            pPairingParams->pDevEncParams->div,
                            pPairingParams->pDevEncParams->rand,
                            pPairingParams->pDevEncParams->keySize ) != SUCCESS )
    {
      // Start encryption failed
      smEndPairing( SMP_PAIRING_FAILED_UNSPECIFIED );
    }
  }
}

/*********************************************************************
 * @fn          SM_StartEncryption
 *
 * @brief       Send Start Encrypt through HCI
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
bStatus_t SM_StartEncryption( uint16 connHandle, uint8 *pLTK, uint16 div, uint8 *pRandNum, uint8 keyLen )
{
  uint8 eDiv[2];                    // LTK div
  uint8 key[KEYLEN];                // LTK
  uint8 random[B_RANDOM_NUM_SIZE];  // LTK Random

  // check the parameters
  if ( pLTK == NULL )
  {
    return ( INVALIDPARAMETER );
  }

  // Setup encryption parameters
  eDiv[0] = LO_UINT16( div );
  eDiv[1] = HI_UINT16( div );

  VOID osal_memset( key, 0, KEYLEN );
  VOID osal_memcpy( key, pLTK, keyLen );

  // A null randNum means to build a random number of all zero's
  if ( pRandNum )
  {
    VOID osal_memcpy( random, pRandNum, B_RANDOM_NUM_SIZE );
  }
  else
  {
    VOID osal_memset( random, 0, B_RANDOM_NUM_SIZE );
  }

  return ( HCI_LE_StartEncyptCmd( connHandle, random, eDiv, key  ) );
}

/*********************************************************************
 * @fn          smpInitiatorProcessIncoming
 *
 * @brief       Process incoming parsed SM Initiator message.
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
static uint8 smpInitiatorProcessIncoming( linkDBItem_t *pLinkItem, uint8 cmdID, smpMsgs_t *pParsedMsg )
{
  uint8 reason = SUCCESS;   // return value

  // check for pairing mode
  if ( pPairingParams == NULL )
  {
    if ( cmdID == SMP_SECURITY_REQUEST )
    {
      // Notify app/profile
      gapSendSlaveSecurityReqEvent( pLinkItem->taskID, pLinkItem->connectionHandle,
                                   pLinkItem->addr, smAuthReqToUint8( &(pParsedMsg->secReq.authReq) ) );
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

  if ( pPairingParams->initiator == FALSE )
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }

  // Process the pairing messages
  switch ( cmdID )
  {
    case SMP_PAIRING_RSP:
      reason = smpInitiatorProcessPairingRsp( (smpPairingRsp_t *)pParsedMsg );
      break;

    case SMP_PAIRING_CONFIRM:
      reason = smpInitiatorProcessPairingConfirm( (smpPairingConfirm_t *)pParsedMsg );
      break;

    case SMP_PAIRING_RANDOM:
      reason = smpInitiatorProcessPairingRandom( (smpPairingRandom_t *)pParsedMsg );
      break;

    case SMP_ENCRYPTION_INFORMATION:
      reason = smpInitiatorProcessEncryptionInformation( (smpEncInfo_t *)pParsedMsg );
      break;

    case SMP_MASTER_IDENTIFICATION:
      reason = smpInitiatorProcessMasterID( (smpMasterID_t *)pParsedMsg );
      break;

    case SMP_IDENTITY_INFORMATION:
      reason = smpInitiatorProcessIdentityInfo( (smpIdentityInfo_t *)pParsedMsg );
      break;

    case SMP_IDENTITY_ADDR_INFORMATION:
      reason = smpInitiatorProcessIdentityAddrInfo( (smpIdentityAddrInfo_t *)pParsedMsg );
      break;

    case SMP_SIGNING_INFORMATION:
      reason = smpInitiatorProcessSigningInfo( (smpSigningInfo_t *)pParsedMsg );
      break;

    case SMP_SECURITY_REQUEST:
      if ( pPairingParams )
      {
        // We are currently pairing. Ignore the message, don't respond
        return ( SUCCESS );
      }
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
 * @fn          smpInitiatorProcessPairingRsp
 *
 * @brief       Process incoming parsed Pairing Response.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 *              SMP_PAIRING_FAILED_AUTH_REQ
 */
static uint8 smpInitiatorProcessPairingRsp( smpPairingRsp_t *pParsedMsg )
{
  uint8 reason = SUCCESS; // return value
  bStatus_t stat; // status field

  // Save the response information into pPairingParams
  stat = smSavePairInfo( pParsedMsg );
  if ( stat == SUCCESS )
  {
    // Check for bonding
    if ( (pPairingParams->pSecReqs->authReq & SM_AUTH_STATE_BONDING)
        && (pPairingParams->pPairDev->authReq.bonding == SM_AUTH_REQ_BONDING) )
    {
      pPairingParams->authState |= SM_AUTH_STATE_BONDING;
    }

    if ( (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS)
        || (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_RESPONDER_INPUTS)
        || (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_BOTH_INPUTS) )
    {
      uint8 type;

      // Determine the passkey input/output user requirements
      if ( (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_BOTH_INPUTS)
          || (pPairingParams->type == SM_PAIRING_TYPE_PASSKEY_INITIATOR_INPUTS) )
      {
        type = SM_PASSKEY_TYPE_INPUT;
      }
      else
      {
        type = SM_PASSKEY_TYPE_DISPLAY;
      }

      // Ask the app for passkey
      gapPasskeyNeededCB( pPairingParams->connectionHandle, type );
      pPairingParams->authState |= SM_AUTH_STATE_AUTHENTICATED;

      pPairingParams->state = SM_PAIRING_STATE_WAIT_PASSKEY;
    }
    else if ( (pPairingParams->type == SM_PAIRING_TYPE_JUST_WORKS)
             || (pPairingParams->type == SM_PAIRING_TYPE_OOB) )
    {
      // Get ready for the Confirm
      pPairingParams->state = SM_PAIRING_STATE_WAIT_CONFIRM;

      // Initialize TK space
      VOID osal_memset( pPairingParams->tk, 0, KEYLEN );

      // Setup TK
      if ( pPairingParams->type != SM_PAIRING_TYPE_JUST_WORKS )
      {
        // OOB
        VOID osal_memcpy( pPairingParams->tk, pPairingParams->pSecReqs->oob, KEYLEN );
        pPairingParams->authState |= SM_AUTH_STATE_AUTHENTICATED;
      }

      // Generate Rand (MRand)
      smGenerateRandBuf( pPairingParams->myComp.rand, SMP_RANDOM_LEN );

      // Generate Confirm (MConfirm)
      VOID sm_c1( pPairingParams->tk,
            pPairingParams->myComp.rand,
            pPairingParams->myComp.confirm );

#if defined ( TESTMODES )
      if ( GAP_GetParamValue( TGAP_SM_TESTCODE ) == SM_TESTMODE_SEND_BAD_CONFIRM )
      {
        VOID osal_memset( pPairingParams->myComp.confirm, 0, KEYLEN );
      }
#endif // TESTMODE

      if ( smGenerateConfirm() != SUCCESS )
      {
        reason = SMP_PAIRING_FAILED_UNSPECIFIED;
      }
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
 * @fn          smpInitiatorProcessPairingConfirm
 *
 * @brief       Process incoming parsed Pairing Confirm.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpInitiatorProcessPairingConfirm( smpPairingConfirm_t *pParsedMsg )
{
  uint8 reason = SUCCESS;  // return value

  VOID osal_memcpy( pPairingParams->devComp.confirm, pParsedMsg->confirmValue, KEYLEN );

  // Received Responder Confirm, send Rand message
  if ( smGenerateRandMsg() != SUCCESS )
  {
    reason = SMP_PAIRING_FAILED_UNSPECIFIED;
  }

  pPairingParams->state = SM_PAIRING_STATE_WAIT_RANDOM;

  return ( reason );
}

/*********************************************************************
 * @fn          smpInitiatorProcessPairingRandom
 *
 * @brief       Process incoming parsed Pairing Random.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_CONFIRM_VALUE
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpInitiatorProcessPairingRandom( smpPairingRandom_t *pParsedMsg )
{
  uint8 reason = SUCCESS;   // return value
  uint8 confirm[KEYLEN];    // working area to calculate a confirm value

  VOID osal_memcpy( pPairingParams->devComp.rand, pParsedMsg->randomValue, SMP_RANDOM_LEN );

  // Check device's Confirm value
  VOID sm_c1( pPairingParams->tk,
        pPairingParams->devComp.rand,
        confirm );

  // Make sure that the calculated confirm matches the confirm from the other device
  if ( osal_memcmp( confirm, pPairingParams->devComp.confirm, KEYLEN ) == TRUE )
  {
    uint8 stk[KEYLEN];  // a place to generate the STK

    // Received Responder, generate STK
    if ( sm_s1( pPairingParams->tk, pPairingParams->devComp.rand,
               pPairingParams->myComp.rand, stk ) == SUCCESS )
    {
      // Start Encrypt with STK
      if ( SM_StartEncryption( pPairingParams->connectionHandle,
                                stk, 0, 0, smDetermineKeySize() ) != SUCCESS )
      {
        reason = SMP_PAIRING_FAILED_UNSPECIFIED;
      }
    }
    else
    {
      reason = SMP_PAIRING_FAILED_UNSPECIFIED;
    }

    // Wait for STK to finish
    pPairingParams->state = SM_PAIRING_STATE_WAIT_STK;
  }
  else
  {
    reason = SMP_PAIRING_FAILED_CONFIRM_VALUE;
  }

  return ( reason );
}

/*********************************************************************
 * @fn          smpInitiatorProcessEncryptionInformation
 *
 * @brief       Process incoming parsed Encryption Information.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpInitiatorProcessEncryptionInformation( smpEncInfo_t *pParsedMsg )
{
  if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_ENCRYPTION_INFO )
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

      pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO;

      return ( SUCCESS );
    }
  }

  return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
}

/*********************************************************************
 * @fn          smpInitiatorProcessMasterID
 *
 * @brief       Process incoming parsed Master Identification.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpInitiatorProcessMasterID( smpMasterID_t *pParsedMsg )
{
  if ( (pPairingParams->pDevEncParams != NULL)
      && (pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_MASTER_INFO) )
  {
    // Save off the rest of the connected device's encryption information
    pPairingParams->pDevEncParams->div = pParsedMsg->ediv;
    VOID osal_memcpy( pPairingParams->pDevEncParams->rand, pParsedMsg->rand, B_RANDOM_NUM_SIZE );

    // Setup the next state
    if ( (pPairingParams->pSecReqs->keyDist.sIdKey) && (pPairingParams->pPairDev->keyDist.sIdKey) )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO;
    }
    else if ( (pPairingParams->pSecReqs->keyDist.sSign) && (pPairingParams->pPairDev->keyDist.sSign) )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO;
    }
    else
    {
      setupInitiatorKeys();
    }

    return ( SUCCESS );
  }
  else
  {
    return (  SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          smpInitiatorProcessIdentityInfo
 *
 * @brief       Process incoming parsed Identity Information.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpInitiatorProcessIdentityInfo( smpIdentityInfo_t *pParsedMsg )
{
  if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_INFO )
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
    pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO;

    return ( SUCCESS );
  }
  else
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          smpInitiatorProcessIdentityAddrInfo
 *
 * @brief       Process incoming parsed Identity Address Information.
 *
 * @param       pParsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpInitiatorProcessIdentityAddrInfo( smpIdentityAddrInfo_t *pParsedMsg )
{
  if ( pPairingParams->state == SM_PAIRING_STATE_WAIT_SLAVE_IDENTITY_ADDR_INFO )
  {
    VOID osal_memcpy( pPairingParams->pIdInfo->bd_addr, pParsedMsg->bdAddr, B_ADDR_LEN );

    // Determine the next state
    if ( (pPairingParams->pSecReqs->keyDist.sSign) && (pPairingParams->pPairDev->keyDist.sSign) )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_SLAVE_SIGNING_INFO;
    }
    else
    {
      // Start sending initiator (master) keys
      setupInitiatorKeys();
    }

    return ( SUCCESS );
  }
  else
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          smpInitiatorProcessSigningInfo
 *
 * @brief       Process incoming parsed Signing Information.
 *
 * @param       parsedMsg - pointer to parsed message
 *
 * @return      SUCCESS
 *              SMP_PAIRING_FAILED_UNSPECIFIED
 */
static uint8 smpInitiatorProcessSigningInfo( smpSigningInfo_t *pParsedMsg )
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

    // Send the initiator key messages
    setupInitiatorKeys();

    return ( SUCCESS );
  }
  else
  {
    return ( SMP_PAIRING_FAILED_CMD_NOT_SUPPORTED );
  }
}

/*********************************************************************
 * @fn          setupInitiatorKeys
 *
 * @brief       Setup Initiator Key distribution
 *
 * @param       none
 *
 * @return      none
 */
static void setupInitiatorKeys( void )
{
  if ( ((pPairingParams->pSecReqs->keyDist.mEncKey) && (pPairingParams->pPairDev->keyDist.mEncKey))
      || ((pPairingParams->pSecReqs->keyDist.mIdKey) && (pPairingParams->pPairDev->keyDist.mIdKey))
      || ((pPairingParams->pSecReqs->keyDist.mSign) && (pPairingParams->pPairDev->keyDist.mSign)) )
  {
    // Setup to send initiator key messages
    pPairingParams->state = SM_PAIRING_STATE_WAIT_STK;
    smInitiatorSendNextKeyInfo();
  }
  else
  {
    // No keys to send
    pPairingParams->state = SM_PAIRING_STATE_DONE;
  }

  if ( pPairingParams->state == SM_PAIRING_STATE_DONE )
  {
    smEndPairing( SUCCESS );
  }
}

/*********************************************************************
 * @fn          smInitiatorSendNextKeyInfo
 *
 * @brief       Initiator role: sends next key message, and sets state
 *              for next event.
 *
 * @param       none
 *
 * @return      none
 */
static void smInitiatorSendNextKeyInfo( void )
{
  if ( pPairingParams->initiator == TRUE )
  {
    smLinkSecurityReq_t *pSecReq = pPairingParams->pSecReqs;
    smpPairingReq_t *pPairReq = pPairingParams->pPairDev;
    uint8 state = pPairingParams->state;

    // Determine key to send
    if ( state == SM_PAIRING_STATE_WAIT_STK )
    {
      if ( (pPairReq->keyDist.mEncKey) && (pSecReq->keyDist.mEncKey) )
      {
        state = SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO;
      }
      else if ( (pPairReq->keyDist.mIdKey) && (pSecReq->keyDist.mIdKey) )
      {
        state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO;
      }
      else if ( (pPairReq->keyDist.mSign) && (pSecReq->keyDist.mSign) )
      {
        state = SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO;
      }
    }

    // Send the correct message
    switch ( state )
    {
      case SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO:
        // send Encryption Information.
        if ( pPairingParams->pEncParams == NULL )
        {
          pPairingParams->pEncParams = (smSecurityInfo_t *)osal_mem_alloc( (uint16)sizeof (smSecurityInfo_t ) );
          if ( pPairingParams->pEncParams )
          {
            VOID osal_memset( pPairingParams->pEncParams, 0, sizeof (smSecurityInfo_t ) );
          }
        }

        if ( pPairingParams->pEncParams )
        {
          smSecurityInfo_t *pEnc = pPairingParams->pEncParams;

          // Default the key size to the key size of this encryption session.
          if ( pEnc->keySize == 0 )
          {
            pEnc->keySize = smDetermineKeySize();
          }

          // For now, temp random the LTK, EDIV and RAND
          VOID osal_memset( pEnc->ltk, 0, KEYLEN );
          smGenerateRandBuf( pEnc->ltk, pPairingParams->pEncParams->keySize );
          pEnc->div = osal_rand();
          smGenerateRandBuf( pEnc->rand, B_RANDOM_NUM_SIZE );

          // Send the Encryption Info
          smPairingSendEncInfo( pPairingParams->connectionHandle, pEnc->ltk );
        }
        break;

      case SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO:
        if ( pPairingParams->pEncParams )
        {
          smPairingSendMasterID( pPairingParams->connectionHandle,
                                 pPairingParams->pEncParams->div,
                                 pPairingParams->pEncParams->rand );
        }
        break;

      case SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO:
        smPairingSendIdentityInfo( pPairingParams->connectionHandle, gapGetIRK() );
        break;

      case SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO:
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

      case SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO:
        smPairingSendSingingInfo( pPairingParams->connectionHandle, gapGetSRK() );
        break;

      default:
        break;
    }

    // Determine the next state
    if ( state == SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO )
    {
      pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO;
    }
    else if ( state == SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO )
    {
      if ( (pPairReq->keyDist.mIdKey) && (pSecReq->keyDist.mIdKey) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO;
      }
      else if ((pPairReq->keyDist.mSign) && (pSecReq->keyDist.mSign) )
      {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO;
      }
      else
      {
        pPairingParams->state = SM_PAIRING_STATE_DONE;
      }
    }
    else if ( state == SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO )
    {
        pPairingParams->state = SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO;
    }
    else if ( state == SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO )
    {
      if ( (pPairReq->keyDist.mSign) && (pSecReq->keyDist.mSign) )
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
      pPairingParams->state = SM_PAIRING_STATE_DONE;
    }

    if ( (pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_ENCRYPTION_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_MASTER_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_IDENTITY_ADDR_INFO)
        || (pPairingParams->state == SM_PAIRING_STATE_WAIT_MASTER_SIGNING_INFO) )
    {
      linkDBItem_t *linkItem;
      uint32 timeout;

      linkItem = linkDB_Find( pPairingParams->connectionHandle );
      if ( linkItem != NULL )
      {
        // Make the timeout 1.5 * connInterval (connInterval = 1.25 ms)
        timeout = linkItem->connInterval;
        timeout += linkItem->connInterval / 2;
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
