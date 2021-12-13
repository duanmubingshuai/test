/*************************************************************************************************
  Filename:       HCI_Host.c
  Revised:        $Date: 2010-02-23 10:47:02 -0800 (Tue, 23 Feb 2010) $
  Revision:       $Revision: 21759 $

  Description:    This file contains the Host APIs for the Host Controller 
                  Interface.


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, 
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

#include "hci_tl.h"
#include "hci_host.h"

/*********************************************************************
 * LOCAL FUNCTIONS
 */
Status_t hciSendCommandPkt(uint16 opCode, uint8 paramLen, uint8 *param);
Status_t hciSendCmdSimplePayload( uint16 opCode, uint8 paramLen, uint8 *param );
uint8* hciCmdBufferAlloc( uint8 paramLen );


/*********************************************************************
 * EXTERNAL FUNCTIONS
 * The following external functions are declared in hci.h
 */

/*********************************************************************
 * @fn          HCI_SendDataPkt
 *
 * @brief       Send HCI data packet from the host to the controller.
 *              This function will add the data packet to the HCI
 *              message queue which will be handled by the flow control
 *              manager.
 *
 * @param       connection - Connection handle & PB & BC
 *              len    - Data total length
 *              pData  - Pointer to the message buffer
 *
 * @return      Status_t 
 */
Status_t HCI_SendDataPkt( HCI_Connection_t connection, uint16 len, uint8 *pData )
{
  uint8 *buf;

  /* Adjust the pointer to the beginning of hci header */
  buf = osal_bm_adjust_header( pData, HCI_DATA_HEADER_LEN );

  /* Fill in the HCI data packet header */
  buf[0] = HCI_SCO_DATA_PACKET;
  buf[1] = LO_UINT16( connection.connectionHandle );
  buf[2] = HI_UINT16( connection.connectionHandle & HCI_CONNECTION_HANDLE_MASK ) |
	         ( ( connection.pb & HCI_PB_MASK ) << 4 );  /* BC is always set to 00 */
  buf[3] = LO_UINT16( len );
  buf[4] = HI_UINT16( len );

  /* Add the packet to the message buffer */
  return HCI_AddDataQueue( buf );

  /* Note, buffer will be freed when dequeue the message */			  
}

/*********************************************************************
 * @fn          HCI_ResetCmd
 *
 * @brief       Send Reset Command from the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_ResetCmd( void )
{
  return hciSendCmdSimplePayload( HCI_RESET, 0, NULL );
}

/*********************************************************************
 * @fn          HCI_BLEReadBufSizeCmd
 *
 * @brief       Send BLE Read Buffer Size Command from the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_BLEReadBufSizeCmd( void )
{
  hciSendCmdSimplePayload( HCI_BLE_READ_BUFFER_SIZE, 0, NULL );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_ReadBufSizeCmd
 *
 * @brief       Send Read Buffer Size Command from the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_ReadBufSizeCmd( void )
{
  hciSendCmdSimplePayload( HCI_READ_BUFFER_SIZE, 0, NULL );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_SetControllerToHostFlowCtrlCmd
 *
 * @brief       Send Set Controller To Host Flow Control Command from 
 *              the host to the controller.
 *
 * @param       pEnable - uint8* - Pointer to flow Control Enable
 *
 * @return      Status_t
 */
Status_t HCI_SetControllerToHostFlowCtrlCmd( uint8 *pEnable ) 
{
  hciSendCmdSimplePayload( HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL, 1, pEnable );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_HostBufferSizeCmd
 *
 * @brief       Send Host Buffer Size Command from the host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_HostBufferSizeCmd( HCIParam_HostBufSize_t *param )
{
  uint8 *buf;
  uint8 *pBuf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_HOST_BUF_SIZE )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;
  
  /* Convert the param struct to buffer */
  pBuf = buf;
  *pBuf++ = LO_UINT16( param->hostAclPktLen );
  *pBuf++ = HI_UINT16( param->hostAclPktLen );
  *pBuf++ = param->hostSyncPktLen;
  *pBuf++ = LO_UINT16( param->hostTotalNumAclPkts );
  *pBuf++ = HI_UINT16( param->hostTotalNumAclPkts );
  *pBuf++ = LO_UINT16( param->hostTotalNumSyncPkts );
  *pBuf++ = HI_UINT16( param->hostTotalNumSyncPkts );

  hciSendCommandPkt( HCI_HOST_BUFFER_SIZE, HCI_PARAM_LEN_HOST_BUF_SIZE, buf );

  return SUCCESS;
}


/*********************************************************************
 * @fn          HCI_ReadLocalSupportedFeatureCmd
 *
 * @brief       Send Read Local Support Feature Command from 
 *              the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_ReadLocalSupportedFeatureCmd( void ) 
{
  hciSendCmdSimplePayload( HCI_READ_LOCAL_SUPPORTED_FEATURE, 0, NULL );
  return SUCCESS;
}


/*********************************************************************
 * @fn          HCI_ReadBDADDRCmd
 *
 * @brief       Send BD_ADDR Command from the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_ReadBDADDRCmd( void )
{
  hciSendCmdSimplePayload( HCI_READ_BD_ADDR, 0, NULL );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLESetPrivateAddrCmd
 *
 * @brief       Send BLE Set Private Device Address Command from the 
 *              host to the controller.
 *
 * @param       pAddress - uint8* - pointer to the 48 bit private device address
 *
 * @return      Status_t
 */
Status_t HCI_BLESetPrivateAddrCmd( uint8 *pAddress ) 
{
  hciSendCmdSimplePayload( HCI_BLE_SET_PRIVATE_ADDR, 6, pAddress );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEReadNumEmptyFilterEntriesCmd
 *
 * @brief       Send BLE Read Number of Empty Filter Entries Command 
 *              from the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_BLEReadNumEmptyFilterEntriesCmd( void ) 
{
  hciSendCmdSimplePayload( HCI_BLE_READ_NUM_EMPTY_FILTER_ENTRIES, 0, NULL );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEWriteDefaultPolicyCmd
 *
 * @brief       Send BLE Write Default Filter Policy Command from 
 *              the host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLEWriteDefaultPolicyCmd( HCIParam_WritePolicy_t *param )
{
  uint8 *buf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_WRITE_POLICY )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  buf[0] = param->policyTarget;
  buf[1] = param->policy;

  hciSendCommandPkt( HCI_BLE_WRITE_DEFAULT_POLICY, HCI_PARAM_LEN_WRITE_POLICY, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEAddWhiteListCmd
 *
 * @brief       Send BLE Add Device to Device Address White List 
 *              Command from the host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLEAddWhiteListCmd( HCIParam_AddWhiteList_t *param )
{
  uint8 *buf;
  
  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_ADD_WHITE_LIST )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  buf[0] = param->addrType;
  osal_memcpy( &(buf[1]), param->address, BLE_ADDR_LEN );

  hciSendCommandPkt( HCI_BLE_ADD_WHITE_LIST, HCI_PARAM_LEN_ADD_WHITE_LIST, buf );
  return SUCCESS;
}

 /*********************************************************************
 * @fn          HCI_BLEClearWhiteListCmd
 *
 * @brief       Send BLE Clear Device Address White List Command from 
 *              the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_BLEClearWhiteListCmd( void ) 
{
  hciSendCmdSimplePayload( HCI_BLE_CLEAR_WHITE_LIST, 0, NULL );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEFlushCmd
 *
 * @brief       Send BLE Flush Command from the host to the controller.
 *
 * @param       handle - connection handle
 *
 * @return      Status_t
 */
Status_t HCI_BLEFlushCmd( uint16 handle) 
{
  uint8 *buf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_BLE_FLUSH )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  buf[0] = LO_UINT16( handle );
  buf[1] = HI_UINT16( handle );

  hciSendCommandPkt( HCI_BLE_FLUSH, HCI_PARAM_LEN_BLE_FLUSH, buf );
  return SUCCESS;
}

 /*********************************************************************
 * @fn          HCI_BLEReadLocalFeatureCmd
 *
 * @brief       Send BLE Read Local Feature Support Command from the 
 *              host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_BLEReadLocalFeatureCmd( void ) 
{
  hciSendCmdSimplePayload( HCI_BLE_READ_LOCAL_FEATURE, 0, NULL );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEWriteLocalFeatureCmd
 *
 * @brief       Send BLE Write Local Feature Command from the host to 
 *              the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLEWriteLocalFeatureCmd( HCIParam_WriteLocalFeature_t *param) 
{
  uint8* pBuf;
  uint8 len;

  /* Convert the param struct to buffer */
  len = param->featureSize + 1;
  
  if(( pBuf = hciCmdBufferAlloc( len )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  *pBuf++ = param->featureSize;
  osal_memcpy( pBuf, param->featureSet, param->featureSize );

  hciSendCommandPkt( HCI_BLE_WRITE_LOCAL_FEATURE, len, pBuf );
  return SUCCESS;

}

/*********************************************************************
 * @fn          HCI_BLESetEventMaskCmd
 *
 * @brief       Send BLE Set Event Mask Command from the host to the controller.
 *
 * @param       pMask - uint8* - Pointer to the 8 bytes BLE Event Mask 
 *
 * @return      Status_t
 */
Status_t HCI_BLESetEventMaskCmd( uint8 *pMask ) 
{
  hciSendCmdSimplePayload( HCI_BLE_SET_EVENT_MASK, BLE_EVENT_MASK_LEN, pMask );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_SetEventMaskCmd
 *
 * @brief       Send BLE Set Event Mask Command from the host to the controller.
 *
 * @param       pMask - uint8* - Pointer to the 8 bytes Event Mask 
 *
 * @return      Status_t
 */
Status_t HCI_SetEventMaskCmd( uint8 *pMask ) 
{
  hciSendCmdSimplePayload( HCI_SET_EVENT_MASK, BLE_EVENT_MASK_LEN, pMask );
  return SUCCESS;
}


/*********************************************************************
 * @fn          HCI_BLEWriteAdvModeCmd
 *
 * @brief       Send BLE Write Advertise Mode Command from the host to the controller.
 *
 * @param       pOnOff - uint8* - pointer: Indicate whether advertising is started or stopped
 *
 * @return      Status_t
 */
Status_t HCI_BLEWriteAdvModeCmd( uint8 *pOnOff ) 
{
  hciSendCmdSimplePayload( HCI_BLE_WRITE_ADV_MODE, 1, pOnOff );
  return SUCCESS;
}


/*********************************************************************
 * @fn          HCI_BLESetAdvParamCmd
 *
 * @brief       Send BLE Set Advertising Parameter Command from the 
 *              host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLESetAdvParamCmd( HCIParam_SetAdvParam_t *param) 
{
  uint8 *buf;
  uint8 *pBuf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_SET_ADV_PARAM )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  pBuf = buf;
  *pBuf++ = LO_UINT16( param->advIntervalMin );
  *pBuf++ = HI_UINT16( param->advIntervalMin );
  *pBuf++ = LO_UINT16( param->advIntervalMax );
  *pBuf++ = HI_UINT16( param->advIntervalMax );
  *pBuf++ = param->eventType;  
  *pBuf++ = param->addrTypeOwn;
  *pBuf++ = param->addrTypeInitiator;  
  osal_memcpy( pBuf, param->addrInitiator, BLE_ADDR_LEN );

  hciSendCommandPkt( HCI_BLE_SET_ADV_PARAM, HCI_PARAM_LEN_SET_ADV_PARAM, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLESetAdvChannelCmd
 *
 * @brief       Send BLE Set Advertising Channels Command from the 
 *              host to the controller.
 *
 * @param       pChannelMap - uint8* - Pointer to the 5 bytes BLE Adv 
 *              Channel Map 
 *
 * @return      Status_t
 */
Status_t HCI_BLESetAdvChannelCmd( uint8 *pChannelMap ) 
{
  hciSendCmdSimplePayload( HCI_BLE_SET_ADV_CHANNEL, BLE_CHANNEL_MAP_LEN, pChannelMap );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEWriteLocalNameCmd
 *
 * @brief       Send BLE Write Local Name Command from the host to the controller.
 *
 * @param       pLocalName - uint8* - Pointer to the 248 bytes Local Name 
 *
 * @return      Status_t
 */
Status_t HCI_BLEWriteLocalNameCmd( uint8 *pLocalName ) 
{
  hciSendCmdSimplePayload( HCI_WRITE_LOCAL_NAME, BLE_LOCAL_NAME_LEN, pLocalName );
  return SUCCESS;
}


/*********************************************************************
 * @fn          HCI_BLESetScanRspParamCmd
 *
 * @brief       Send BLE Set Scan Response Parameters Command from 
 *              the host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLESetScanRspParamCmd( HCIParam_SetScanRsp_t *param) 
{
  uint8 *buf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_SET_SCAN_RSP )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  buf[0] = LO_UINT16( param->profileID );
  buf[1] = HI_UINT16( param->profileID );
  buf[2] = param->moreProfiles;  
  buf[3] = param->encryptionRequired;

  hciSendCommandPkt( HCI_SET_SCAN_RSP_PARAM, HCI_PARAM_LEN_SET_SCAN_RSP, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEWriteAdvDataCmd
 *
 * @brief       Send BLE Write Advertising Data Command from the 
 *              host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLEWriteAdvDataCmd( HCIParam_WriteAdvData_t *param) 
{
  uint8* buf;
  uint8 len;

  /* Convert the param struct to buffer */
  len = param->dataLen + 1;
  
  if(( buf = hciCmdBufferAlloc( len )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  buf[0] = param->dataLen;
  osal_memcpy( &(buf[1]), param->data, param->dataLen );

  hciSendCommandPkt( HCI_WRITE_ADV_DATA, len, buf );
  return SUCCESS;
}
                  
/*********************************************************************
 * @fn          HCI_BLEWriteScanModeCmd
 *
 * @brief       Send BLE Write SCAN Mode Command from the host to the controller.
 *
 * @param       pOnOff - uint8* - pointer: Indicate whether scan is started or stopped
 *
 * @return      Status_t
 */
Status_t HCI_BLEWriteScanModeCmd( uint8 *pOnOff ) 
{
  hciSendCmdSimplePayload( HCI_BLE_WRITE_SCAN_MODE, 1, pOnOff );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLESetScanParamCmd
 *
 * @brief       Send BLE Set Scan Parameters Command from the host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLESetScanParamCmd( HCIParam_SetScanParam_t *param) 
{
  uint8 *buf;
  uint8 *pBuf;
  
  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_SET_SCAN_PARAM )) == NULL )
	  return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  pBuf = buf;
  *pBuf++ = param->scanMode;  
  *pBuf++ = param->addrType;
  *pBuf++ = LO_UINT16( param->scanInterval );
  *pBuf++ = HI_UINT16( param->scanInterval );
  *pBuf++ = LO_UINT16( param->scanWindow );
  *pBuf++ = HI_UINT16( param->scanWindow );

  hciSendCommandPkt( HCI_SET_SCAN_PARAM, HCI_PARAM_LEN_SET_SCAN_PARAM, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLECreateLLConnCmd
 *
 * @brief       Send BLE Create Link Layer Connection Command from 
 *              the host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLECreateLLConnCmd( HCIParam_CreateLL_t *param) 
{
  uint8* pBuf;
  uint8* buf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_CREATE_LL )) == NULL )
    return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  pBuf = buf;
  *pBuf++ = LO_UINT16( param->scanInterval );
  *pBuf++ = HI_UINT16( param->scanInterval );
  *pBuf++ = LO_UINT16( param->scanWindow );
  *pBuf++ = HI_UINT16( param->scanWindow );
  *pBuf++ = param->whiteList;  
  *pBuf++ = param->addrTypePeer;
  osal_memcpy( pBuf, param->peerAddr, BLE_ADDR_LEN );
  pBuf += BLE_ADDR_LEN;
  *pBuf++ = param->addrTypeOwn; 
  *pBuf++ = LO_UINT16( param->connIntervalMin );
  *pBuf++ = HI_UINT16( param->connIntervalMin );
  *pBuf++ = LO_UINT16( param->connIntervalMax );
  *pBuf++ = HI_UINT16( param->connIntervalMax );
  *pBuf++ = LO_UINT16( param->connLatency );
  *pBuf++ = HI_UINT16( param->connLatency );
  *pBuf++ = LO_UINT16( param->connTimeout );
  *pBuf++ = HI_UINT16( param->connTimeout );
  *pBuf++ = param->encrypted; 
  *pBuf++ = LO_UINT16( param->minLen );
  *pBuf++ = HI_UINT16( param->minLen );
  *pBuf++ = LO_UINT16( param->maxLen );
  *pBuf++ = HI_UINT16( param->maxLen );

  hciSendCommandPkt( HCI_BLE_CREATE_LL_CONNECTION, HCI_PARAM_LEN_CREATE_LL, buf );
  return SUCCESS;
}


 /*********************************************************************
 * @fn          HCI_BLEStopLLConnCreationCmd
 *
 * @brief       Send BLE Stop LL Connection Creation Command from 
 *              the host to the controller.
 *
 * @param       none
 *
 * @return      Status_t
 */
Status_t HCI_BLEStopLLConnCreationCmd( void )
{ 
  hciSendCmdSimplePayload( HCI_BLE_STOP_LL_CONNECTION_CREATION, 0, NULL );
  return SUCCESS;
}


/*********************************************************************
 * @fn          HCI_BLETerminateLLConnCmd
 *
 * @brief       Send BLE Terminate LL Connection Command from the 
 *              host to the controller.
 *
 * @param       handle - Connection handle
 *
 * @return      Status_t
 */
Status_t HCI_BLETerminateLLConnCmd( uint16 handle ) 
{
  uint8 *buf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_TERMINATE_LL )) == NULL )
    return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  buf[0] = LO_UINT16( handle );
  buf[1] = HI_UINT16( handle );

  hciSendCommandPkt( HCI_BLE_TERMINATE_LL_CONNECTION, HCI_PARAM_LEN_TERMINATE_LL, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEUpdateLLConnCmd
 *
 * @brief       Send BLE Update LL Connection Command from the host 
 *              to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLEUpdateLLConnCmd( HCIParam_UptateLL_t *param ) 
{
  uint8 *buf;
  uint8 *pBuf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_UPDATE_LL )) == NULL )
    return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  pBuf = buf;
  *pBuf++ = LO_UINT16( param->connectionHandle );
  *pBuf++ = HI_UINT16( param->connectionHandle );
  *pBuf++ = LO_UINT16( param->connIntervalMin );
  *pBuf++ = HI_UINT16( param->connIntervalMin );
  *pBuf++ = LO_UINT16( param->connIntervalMax );
  *pBuf++ = HI_UINT16( param->connIntervalMax );
  *pBuf++ = LO_UINT16( param->connLatency );
  *pBuf++ = HI_UINT16( param->connLatency );
  *pBuf++ = LO_UINT16( param->connTimeout );
  *pBuf++ = HI_UINT16( param->connTimeout );
  *pBuf++ = LO_UINT16( param->minLen );
  *pBuf++ = HI_UINT16( param->minLen );
  *pBuf++ = LO_UINT16( param->maxLen );
  *pBuf++ = HI_UINT16( param->maxLen );

  hciSendCommandPkt( HCI_BLE_UPDATE_LL_CONNECTION, HCI_PARAM_LEN_UPDATE_LL, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEUpdateChannelMapCmd
 *
 * @brief       Send BLE Update Channel Map Command from the host 
 *              to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLEUpdateChannelMapCmd( HCIParam_UpdateChannel_t *param ) 
{
  uint8 *buf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_UPDATE_CHANNEL_MAP )) == NULL )
    return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  buf[0] = LO_UINT16( param->connectionHandle );
  buf[1] = HI_UINT16( param->connectionHandle );
  osal_memcpy( &(buf[2]), param->channelMap, BLE_CHANNEL_MAP_LEN); 

  hciSendCommandPkt( HCI_BLE_UPDATE_CHANNEL_MAP, HCI_PARAM_LEN_UPDATE_CHANNEL_MAP, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLEReadRemoteFeatureCmd
 *
 * @brief       Send BLE Read Remote Feature Command from the host to the controller.
 *
 * @param       handle - Connection handle
 *
 * @return      Status_t
 */
Status_t HCI_BLEReadRemoteFeatureCmd( uint16 handle ) 
{
  uint8 *buf;

  if(( buf = hciCmdBufferAlloc( HCI_PARAM_LEN_READ_FEATURE )) == NULL )
    return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  buf[0] = LO_UINT16( handle );
  buf[1] = HI_UINT16( handle );

  hciSendCommandPkt( HCI_BLE_READ_REMOTE_FEATURE_SUPPORT, HCI_PARAM_LEN_READ_FEATURE, buf );
  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_BLENumCompletedPktCmd
 *
 * @brief       Send BLE Number of Completed Packets Command from the host to the controller.
 *
 * @param       param - pointer to the parameter structure
 *
 * @return      Status_t
 */
Status_t HCI_BLENumCompletedPktCmd( HCIParam_NumCompletedPkt_t *param ) 
{
  uint8* pBuf;
  uint8* buf;
  uint8  len;
  uint8  i;

  len = 1 + param->numHandles * 4;

  if(( buf = hciCmdBufferAlloc( len )) == NULL )
    return MSG_BUFFER_NOT_AVAIL;

  /* Convert the param struct to buffer */
  pBuf = buf;
  *pBuf++ = param->numHandles;
  
  for( i = 0; i < param->numHandles; i++ )
  {
    *pBuf++ = LO_UINT16( param->connHandle[i] );
    *pBuf++ = HI_UINT16( param->connHandle[i] );
  }

  for( i = 0; i < param->numHandles; i++ )
  {
    *pBuf++ = LO_UINT16( param->numCompletedPkts[i] );
    *pBuf++ = HI_UINT16( param->numCompletedPkts[i] );
  }

  hciSendCommandPkt( HCI_HOST_NUM_COMPLETED_PACKETS, len, buf );
  return SUCCESS;
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn          hciSendCommandPkt
 *
 * @brief       Send HCI command packet from the host to the controller.
 *              This function will add the command packet to the HCI
 *              message queue which will be sent out later.
 *
 * @param       opCode   - Command opCode
 *              paramLen - Parameter total length
 *              param    - byte array of parameter list
 *
 * @return      Status_t
 */
Status_t hciSendCommandPkt(uint16 opCode, uint8 paramLen, uint8 *param)
{
  uint8 *buf;

  // Adjust the start of the buffer
  buf = (uint8*)( param - HCI_COMMAND_HEADER_LEN );  

  // Fill in the command hearder
  buf[0] = HCI_CMD_PACKET;   /* HCI Packet Type */
  buf[1] = LO_UINT16( opCode );
  buf[2] = HI_UINT16( opCode );
  buf[3] = paramLen;

  /* Add the buffer to the CMD queue */
  return HCI_AddCmdQueue( buf );

  /* Note: The buffer will be freed when dequeue the message */
}

/*********************************************************************
 * @fn          hciSendCmdSimplePayload
 *
 * @brief       Send HCI Command with zero or one parameter.
 *
 * @param       opCode   - Command opCode
 *              paramLen - Parameter total length. Zero means no parameter.
 *              param    - pointer to the parameter. NULL means no parameter.
 *
 * @return      Status_t
 */
Status_t hciSendCmdSimplePayload( uint16 opCode, uint8 paramLen, uint8 *param ) 
{
  uint8 *buf;

  if( ( param == NULL ) && ( paramLen != 0 ) )
	return INVALIDPARAMETER;

  /* Allocate HCI Command buffer */
  if(( buf = hciCmdBufferAlloc( paramLen )) == NULL )
    return MSG_BUFFER_NOT_AVAIL;

  /* Copy the parameter over */
  if( param != NULL )
  {
	osal_memcpy( buf, param, paramLen );
  }

  return hciSendCommandPkt( opCode, paramLen, buf );
}


/*********************************************************************
 * @fn          hciCmdBufferAlloc
 *
 * @brief       Allocate a buffer for HCI command. The length of the buffer
 *              is the sum command header length and parameter list length.
 *
 * @param       paramLen - total length of the parameter list
 *
 * @return      uint8* - pointer to the parameter list
 */
uint8* hciCmdBufferAlloc( uint8 paramLen ) 
{
  uint8 *pBuf;

  if(( pBuf = osal_mem_alloc( HCI_COMMAND_HEADER_LEN + paramLen )) == NULL )
    return NULL;

  return ( pBuf + HCI_COMMAND_HEADER_LEN );
}
