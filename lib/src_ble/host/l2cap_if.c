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
  Filename:       l2cap_if.c
  Revised:        
  Revision:       

  Description:    This file contains the interfaces that L2CAP provides to the
                  upper layer applications.


**************************************************************************************************/


/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"

#include "l2cap_internal.h"

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

// The Controller to Host flow control mode
//static uint8 l2capFlowCtrlMode = FALSE;

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

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      L2CAP_RegisterApp
 *
 * @brief   Register a protocol/application with an L2CAP channel.
 *
 * @param   taskId - task to be registered with channel.
 * @param   CID - channel ID.
 *
 * @return  SUCCESS: Registration was successfull.
 *          INVALIDPARAMETER: Channel ID is invalid.
 */
bStatus_t L2CAP_RegisterApp( uint8 taskId, uint16 CID )
{
  // We only support fixed channels for now
  if ( FIX_CHANNEL( CID ) )
  {
    FIX_CHANNEL_REC( CID ).CID = CID;
    FIX_CHANNEL_REC( CID ).taskId = taskId;

    return ( SUCCESS );
  }

  return ( INVALIDPARAMETER );
}

/*********************************************************************
 * @fn      L2CAP_SendData
 *
 * @brief   Send data packet over an L2CAP channel established over a physical connection.
 *
 *          Note: Packet 'pPayload' must be allocated using L2CAP_bm_alloc().
 *
 * @param   connHandle - connection to be used.
 * @param   pPkt - pointer to packet to be sent.
 *
 * @return  SUCCESS: Data was sent successfully.
 *          INVALIDPARAMETER: Channel ID is invalid.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t L2CAP_SendData( uint16 connHandle, l2capPacket_t *pPkt )
{
  // We only support fixed channels for now
  if ( !FIX_CHANNEL( pPkt->CID ) )
  {
    return ( INVALIDPARAMETER );
  }

  // Make sure the physical connection is up
  if ( !linkDB_Up( connHandle ) )
  { 
    return ( bleNotConnected );
  }

  // Find out destination CID for dynamic channels.
  // Note: The local and remote CIDs are the same for fixed channels but could
  // be different for the dynamic channels (which currently not supported).

  // Encapsulate and send data
  return ( l2capEncapSendData( connHandle, pPkt ) );
}

/*********************************************************************
 * @fn      L2CAP_CmdReject
 *
 * @brief   Send Command Reject.
 *
 * @param   connHandle - connection to use
 * @param   id - identifier of the request packet being rejected
 * @param   pCmdReject - pointer to Command Reject to be sent
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Data can not fit into one packet.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t L2CAP_CmdReject( uint16 connHandle, uint8 id, l2capCmdReject_t *pCmdReject )
{
  return ( l2capSendCmd( connHandle, L2CAP_CMD_REJECT, id,
                         (uint8 *)(pCmdReject), L2CAP_BuildCmdReject ) );
}

/*********************************************************************
 * @fn      L2CAP_EchoReq
 *
 * @brief   Send Ehco Request.
 *
 * @param   connHandle - connection to use
 * @param   pEchoReq - pointer to Echo Request to be sent
 * @param   taskId - task to be notified about result
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Data can not fit into one packet.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleNoResources: No available resource.
 *          bleMemAllocError: Memory allocation error occurred.
 */
//bStatus_t L2CAP_EchoReq( uint16 connHandle, l2capEchoReq_t *pEchoReq, uint8 taskId )
//{
//  if ( ( pEchoReq->len == 0 ) || ( pEchoReq->pData != NULL ) )
//  {
//    return ( l2capSendReq( connHandle, L2CAP_ECHO_REQ, (uint8 *)(pEchoReq),
//                           l2capBuildEchoReq, L2CAP_W4_ECHO_RSP, taskId ) );
//  }
//
//  return ( INVALIDPARAMETER );
//}

/*********************************************************************
 * @fn      L2CAP_InfoReq
 *
 * @brief   Send Information Request.
 *
 * @param   connHandle - connection to use
 * @param   pInfoReq - pointer to Info Request to be sent
 * @param   taskId - task to be notified about result
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Data can not fit into one packet.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleNoResources: No available resource
 *          bleMemAllocError: Memory allocation error occurred.
 */
//bStatus_t L2CAP_InfoReq( uint16 connHandle, l2capInfoReq_t * pInfoReq, uint8 taskId )
//{
//  return ( l2capSendReq( connHandle, L2CAP_INFO_REQ, (uint8 *)(pInfoReq),
//                         l2capBuildInfoReq, L2CAP_W4_INFO_RSP, taskId ) );
//}

/*********************************************************************
 * @fn          L2CAP_ConnParamUpdateReq
 *
 * @brief       Send Connection Parameter Update Request.
 *
 * @param       connHandle - connection to use
 * @param       pUpdateReq - pointer to Update Request to be sent
 * @param       taskId - task to be notified about result
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Data can not fit into one packet.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleMemAllocError: Memory allocation error occurred.
 *          bleNotConnected: Connection is down.
 *          bleNoResources: No available resource
 */
bStatus_t L2CAP_ConnParamUpdateReq( uint16 connHandle, l2capParamUpdateReq_t *pUpdateReq, uint8 taskId )
{
  return ( l2capSendReq( connHandle, L2CAP_PARAM_UPDATE_REQ, (uint8 *)(pUpdateReq),
                         l2capBuildParamUpdateReq, L2CAP_W4_PARAM_UPDATE_RSP, taskId ) );
}

/*********************************************************************
 * @fn      L2CAP_ConnParamUpdateRsp
 *
 * @brief   Send Connection Parameter Update Response.
 *
 * @param   connHandle - connection to use
 * @param   id - identifier received in request
 * @param   pUpdateRsp - pointer to Update Response to be sent
 *
 * @return  SUCCESS: Request was sent successfully.
 *          INVALIDPARAMETER: Data can not fit into one packet.
 *          MSG_BUFFER_NOT_AVAIL: No HCI buffer is available.
 *          bleNotConnected: Connection is down.
 *          bleMemAllocError: Memory allocation error occurred.
 */
//bStatus_t L2CAP_ConnParamUpdateRsp( uint16 connHandle, uint8 id, l2capParamUpdateRsp_t *pUpdateRsp )
//{
//  return ( l2capSendCmd( connHandle, L2CAP_PARAM_UPDATE_RSP, id,
//                         (uint8 *)(pUpdateRsp), L2CAP_BuildParamUpdateRsp ) );
//}

/*********************************************************************
 * @fn      L2CAP_SetControllerToHostFlowCtrl
 *
 * @brief   This API is used by the upper layer to turn flow control on
 *          or off for data packets sent from the Controller to the Host.
 *
 * @param   hostBuffSize - total data buffer available on Host
 * @param   flowCtrlMode - flow control mode: TRUE or FALSE
 *
 * @return  none
 */
//void L2CAP_SetControllerToHostFlowCtrl( uint16 hostBuffSize, uint8 flowCtrlMode )
//{
//  // Save the flow control mode
//  l2capFlowCtrlMode = flowCtrlMode;
//  
//  // See if the flow control is enabled
//  if ( l2capFlowCtrlMode == TRUE )
//  {
//    uint16 hostNumPkts = hostBuffSize / L2CAP_PDU_SIZE;
//      
//    // First turn flow control on for data sent from the Controller to Host
//     HCI_SetControllerToHostFlowCtrlCmd( HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF );
//      
//    // Set the number of ACL packets the Controller can send to the Host
//     HCI_HostBufferSizeCmd( L2CAP_PDU_SIZE, 0, hostNumPkts, 0 );
//  }
//  else
//  { 
//    // Turn flow control off for data sent from the Controller to Host
//     HCI_SetControllerToHostFlowCtrlCmd( HCI_CTRL_TO_HOST_FLOW_CTRL_OFF );
//  }
//}   

/*********************************************************************
 * @fn      L2CAP_HostNumCompletedPkts
 *
 * @brief   This API is used by the upper layer to notify L2CAP of the
 *          number of data packets that have been completed for connection
 *          handle since this API was previously called.
 *
 * @param   connHandle - connection handle
 * @param   numCompletedPkts - number of completed packets
 *
 * @return  none
 */
//void L2CAP_HostNumCompletedPkts( uint16 connHandle, uint16 numCompletedPkts )
//{
//  // Check if Controller to Host flow control is enabled
//  if ( l2capFlowCtrlMode == TRUE )
//  {
//    // Note: It's assumed that the upper layer will only free one buffer at a time.
//     HCI_HostNumCompletedPktCmd( 1, &connHandle, &numCompletedPkts );
//  }
//}


/****************************************************************************
****************************************************************************/