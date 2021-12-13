/*******************************************************************************
  Filename:       hci.c
  Revised:        
  Revision:       

  Description:    This file contains the Host Controller Interface (HCI) API.


*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

//#include "npi.h"
#include "hci_tl.h"
#include "hci_data.h"
#include "hci_event.h"
#include "hci.h"
#include "ll_def.h"
#include "ll.h"
#include "global_config.h"
#include "att.h"

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

// HCI Version and Revision
#define HCI_VERSION                                  0x06    // BT Core Specification V4.0

// Major Version (8 bits) . Minor Version (4 bits) . SubMinor Version (4 bits)
#define HCI_REVISION                                 0x0120  // HCI Version 1.2.0

// Internal Only Status Values
#define HCI_STATUS_WARNING_FLAG_UNCHANGED            LL_STATUS_WARNING_FLAG_UNCHANGED

// Parameter Limits
#define HCI_ADV_NONCONN_INTERVAL_MIN                 LL_ADV_NONCONN_INTERVAL_MIN
#define HCI_ADV_NONCONN_INTERVAL_MAX                 LL_ADV_NONCONN_INTERVAL_MAX
#define HCI_ADV_CONN_INTERVAL_MIN                    LL_ADV_CONN_INTERVAL_MIN
#define HCI_ADV_CONN_INTERVAL_MAX                    LL_ADV_CONN_INTERVAL_MAX
#define HCI_SCAN_INTERVAL_MIN                        LL_SCAN_INTERVAL_MIN
#define HCI_SCAN_INTERVAL_MAX                        LL_SCAN_INTERVAL_MAX
#define HCI_SCAN_WINDOW_MIN                          LL_SCAN_WINDOW_MIN
#define HCI_SCAN_WINDOW_MAX                          LL_SCAN_WINDOW_MAX
#define HCI_CONN_INTERVAL_MIN                        LL_CONN_INTERVAL_MIN
#define HCI_CONN_INTERVAL_MAX                        LL_CONN_INTERVAL_MAX
#define HCI_CONN_TIMEOUT_MIN                         LL_CONN_TIMEOUT_MIN
#define HCI_CONN_TIMEOUT_MAX                         LL_CONN_TIMEOUT_MAX
#define HCI_SLAVE_LATENCY_MIN                        LL_SLAVE_LATENCY_MIN
#define HCI_SLAVE_LATENCY_MAX                        LL_SLAVE_LATENCY_MAX

// Local Supported Feature Set
// Note: Bit 5 in byte 4 is LE supported feature bit.
//       Bit 6 in byte 4 is BR/EDR not supported feature bit.
#define LOCAL_SUPPORTED_FEATURE_SET_BYTE_4           0x60

// Local Supported Commands
#define SUPPORTED_COMMAND_LEN                        64

#define SUPPORTED_COMMAND_BYTE_0                     0x20

#define SUPPORTED_COMMAND_BYTE_2                     0x80
#define SUPPORTED_COMMAND_BYTE_5                     0xC0

#define SUPPORTED_COMMAND_BYTE_10                    0xE4
#define SUPPORTED_COMMAND_BYTE_15                    0x22


#define SUPPORTED_COMMAND_BYTE_14                    0x28

#define SUPPORTED_COMMAND_BYTE_25                    0xF7

#define BYTE_26_ADV                                  0xC2

#define BYTE_26_SCAN                                 0xCD

#define BYTE_27_ADV                                  0x10
#define BYTE_28_ADV                                  0x06

#define BYTE_26_INIT                                 0xF0
#define BYTE_27_INIT                                 0xFF
#define BYTE_28_INIT                                 0x01


#define SUPPORTED_COMMAND_BYTE_26                    (BYTE_26_ADV | BYTE_26_SCAN | BYTE_26_INIT)
#define SUPPORTED_COMMAND_BYTE_27                    (0xC3 | BYTE_27_ADV | BYTE_27_INIT)
#define SUPPORTED_COMMAND_BYTE_28                    (0x78 | BYTE_28_ADV | BYTE_28_INIT)

/*******************************************************************************
 * TYPEDEFS
 */

typedef const uint8 supportedCmdsTable_t;

/*******************************************************************************
 * EXTERNAL VARIABLES
 */
// for HCI ext command: enable/disable notify for connection/adv event
extern  uint8    g_adv_taskID;
extern  uint16   g_adv_taskEvent;
extern  uint8    g_conn_taskID;
extern  uint16   g_conn_taskEvent;
extern  uint8    g_dle_taskID;
extern  uint16   g_dle_taskEvent;
extern  uint8    g_phyChg_taskID;
extern  uint16   g_phyChg_taskEvent;

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Note: There are 64 bytes of supported commands per the Core spec. One
//       additional byte is added as a status when the table is returned
//       as a parameter for event generation.
supportedCmdsTable_t supportedCmdsTable[SUPPORTED_COMMAND_LEN+1] =
{
  // used to hold status when returned as an event parameter
  HCI_SUCCESS,
  // supported commands
  SUPPORTED_COMMAND_BYTE_0,
  0x00,
  SUPPORTED_COMMAND_BYTE_2,
  0x00, 0x00,
  SUPPORTED_COMMAND_BYTE_5,
  0x00, 0x00, 0x00, 0x00,
  SUPPORTED_COMMAND_BYTE_10,
  0x00, 0x00, 0x00,
  SUPPORTED_COMMAND_BYTE_14,
  SUPPORTED_COMMAND_BYTE_15,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  SUPPORTED_COMMAND_BYTE_25,
  SUPPORTED_COMMAND_BYTE_26,
  SUPPORTED_COMMAND_BYTE_27,
  SUPPORTED_COMMAND_BYTE_28,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00
};

/*******************************************************************************
 * GLOBAL VARIABLES
 */
uint8  hciPTMenabled;
uint8  ctrlToHostEnable;
uint16 numHostBufs;
uint8  hciCtrlCmdToken;        // counter of the number of usable command buffers

/*******************************************************************************
 * HCI API
 */

/*
** Buffer Management
*/

/*******************************************************************************
 * This API is used to allocate memory using buffer management.
 *
 * Public function defined in hci.h.
 */
void *HCI_bm_alloc( uint16 size )
{
  return( LL_TX_bm_alloc( size ) );
}


/*******************************************************************************
 * This API is used to check that the connection time parameters are valid.
 *
 * Public function defined in hci.h.
 */
uint8 HCI_ValidConnTimeParams( uint16 connIntervalMin,
                               uint16 connIntervalMax,
                               uint16 connLatency,
                               uint16 connTimeout )
{
  return( !LL_INVALID_CONN_TIME_PARAM( connIntervalMin,
                                       connIntervalMax,
                                       connLatency,
                                       connTimeout )                          &&
          !LL_INVALID_CONN_TIME_PARAM_COMBO( connIntervalMax,
                                             connLatency,
                                             connTimeout ) );
}


/*
** Data
*/

/*******************************************************************************
 * This API is used to send a ACL data packet over a connection.
 *
 * NOTE: L2CAP is affected by this routine's status values as it must remap
 *       them to Host status values. If any additional status values are added
 *       and/or changed in this routine, a TI stack engineer must be notified!
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_SendDataPkt( uint16 connHandle,
                             uint8  pbFlag,
                             uint16 pktLen,
                             uint8  *pData )
{
  hciStatus_t hciStatus;

  // various checks
  if ( hciPTMenabled == TRUE )
  {
    // not allowed command during PTM
    hciStatus = HCI_ERROR_CODE_CONTROLLER_BUSY;
  }
  else if ( (pktLen == 0) || (pData == NULL) || ((connHandle & 0xFF00) != 0) )
  {
    // bad packet length, bad pointer, or bad connection handle
    // Note: The TI LE only supports a eight bit connection handle, so check to
    //       be sure something isn't erroneously mapped to a valid connection
    //       handle (e.g. 0x0100 -> 0x0000).
    hciStatus = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }
  else // packet is okay
  {  
    // attempt to send the packet
    // Note: A return of HCI_SUCCESS from this routine indicates that either
    //       the data was transmitted and freed, or it is still in use
    //       (i.e. queued).
    hciStatus = LL_TxData( connHandle, pData, pktLen, pbFlag );
  }

  return( hciStatus );
}


/*
** Link Control Commands
*/


/*******************************************************************************
 * This BT API is used to terminate a connection.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_DisconnectCmd( uint16 connHandle,
                               uint8  reason )

{
  HCI_CommandStatusEvent( LL_Disconnect(connHandle, reason), HCI_DISCONNECT );

  return( HCI_SUCCESS );
}



/*******************************************************************************
 * This BT API is used to request version information from the the remote
 * device in a connection.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ReadRemoteVersionInfoCmd( uint16 connHandle )
{
  hciStatus_t status;

  HCI_CommandStatusEvent( HCI_SUCCESS, HCI_READ_REMOTE_VERSION_INFO );

  status = LL_ReadRemoteVersionInfo(connHandle);

  // check if something went wrong
  // Note: If success is returned, then Command Complete is handled by Callback.
  if ( status != HCI_SUCCESS )
  {
    HCI_CommandCompleteEvent( HCI_READ_REMOTE_VERSION_INFO, sizeof(status), &status );
  }

  return( HCI_SUCCESS );
}



/*
** Controller and Baseband Commands
*/

/*******************************************************************************
 * This BT API is used to set the HCI event mask, which is used to determine
 * which events are supported.
 *
 * Note: The global pHciEvtMask is used for BT events. A different global is
 *       used for LE events: bleEvtMask.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_SetEventMaskCmd( uint8 *pMask )
{
  hciStatus_t status;

  // check parameters
  if( pMask != NULL )
  {
    (void)osal_memcpy( pHciEvtMask, pMask, B_EVENT_MASK_LEN );

    status = HCI_SUCCESS;
  }
  else // bad parameters
  {
    status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }

  HCI_CommandCompleteEvent( HCI_SET_EVENT_MASK, sizeof(status), &status );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 *
 * This BT API is used to reset the Link Layer.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ResetCmd( void )
{
  hciStatus_t status;

  // reset the Link Layer
  status = LL_Reset();

  // reset the Bluetooth and the BLE event mask bits
  hciInitEventMasks();

  // initialize Controller to Host flow control flag and counter
  ctrlToHostEnable = FALSE;
  numHostBufs      = 0;

  // complete the command
  HCI_CommandCompleteEvent( HCI_RESET, sizeof(status), &status);

  return( HCI_SUCCESS );
}


/*******************************************************************************
 *
 * This BT API is used to read the transmit power level.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ReadTransmitPowerLevelCmd( uint16 connHandle,
                                           uint8  txPwrType )
{
  // 0: Status
  // 1: Connection Handle LSB
  // 2: Connection Handle MSB
  // 3: Transmit Power Level
  uint8 rtnParam[4];

// TODO
//  rtnParam[0] = LL_ReadTxPowerLevel( connHandle,
//                                     txPwrType,
//                                     (int8 *)&(rtnParam[3]) );

  rtnParam[1] = LO_UINT16( connHandle );
  rtnParam[2] = HI_UINT16( connHandle );

  HCI_CommandCompleteEvent( HCI_READ_TRANSMIT_POWER, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}



/*******************************************************************************
 *
 * This BT API is used by the Host to turn flow control on or off for data sent
 * from the Controller to Host.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_SetControllerToHostFlowCtrlCmd( uint8 flowControlEnable )
{
  hciStatus_t status = HCI_SUCCESS;

  // check parameters
  if ( (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_OFF)              ||
       (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF) ||
       (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON) ||
       (flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON) )
  {
    // check the parameter
    if( flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_OFF )
    {
      // disable flow control
      ctrlToHostEnable = FALSE;
    }
    else if ( flowControlEnable == HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF )
    {
      // enable flow control
      ctrlToHostEnable = TRUE;
    }
    else // other two combinations not supported
    {
      // so indidicate
      status = HCI_ERROR_CODE_UNSUPPORTED_FEATURE_PARAM_VALUE;
    }
  }
  else // bad parameters
  {
    status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }

  HCI_CommandCompleteEvent( HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL, sizeof(status), &status);

  return( HCI_SUCCESS );
}



/*******************************************************************************
 *
 * This BT API is used by the Host to notify the Controller of the maximum size
 * ACL buffer size the Controller can send to the Host.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_HostBufferSizeCmd( uint16 hostAclPktLen,
                                   uint8  hostSyncPktLen,
                                   uint16 hostTotalNumAclPkts,
                                   uint16 hostTotalNumSyncPkts )
{
  hciStatus_t status;

  // unused input parameter; PC-Lint error 715.
  (void)hostSyncPktLen;
  (void)hostAclPktLen;
  (void)hostTotalNumSyncPkts;

  // check parameters
  // Note: Only Number of ACL Packets is supported. The rest of the parameters
  //       are ignored for now.
  if ( hostTotalNumAclPkts == 0 )
  {
    status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }
  else // parameter okay
  {
    status = HCI_SUCCESS;

    // so save in a counter
    numHostBufs = hostTotalNumAclPkts;
  }

  HCI_CommandCompleteEvent( HCI_HOST_BUFFER_SIZE, sizeof(status), &status );

  return( HCI_SUCCESS );
}



/*******************************************************************************
 * This BT API is used by the Host to notify the Controller of the number of
 * HCI data packets that have been completed for each connection handle since
 * this command was previously sent to the controller.
 *
 * Note: It is assumed that there will be at most only one handle. Even if more
 *       than one handle is provided, the Controller does not track Host buffers
 *       as a function of connection handles (and isn't required to do so).
 * Note: The connection handle is not used.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_HostNumCompletedPktCmd( uint8   numHandles,
                                        uint16 *connHandles,
                                        uint16 *numCompletedPkts )
{
  // check parameters
  if ( (numHandles != 0) && (connHandles != NULL) &&
       ((numCompletedPkts != NULL) && (*numCompletedPkts != 0)) )
  {
    // check if flow control is enabled
    if ( ctrlToHostEnable == TRUE )
    {
      // check if the number of Host buffers was previously exhausted
      if ( numHostBufs == 0 )
      {
        // yes, so disable LL Rx flow control
        (void)LL_CtrlToHostFlowControl( LL_DISABLE_RX_FLOW_CONTROL );
      }

      for (uint8 i=0; i<numHandles; i++)
      {
        // host is indicating it has freed one or more buffers
        // Note: It is assumed that the Host will only free one buffer at a time,
        //       and in any case, number of Host buffers are not tracked as a
        //       function of connection handles.
        // Note: No checks are made to ensure the specified connection handles
        //       are valid or active.
        numHostBufs += numCompletedPkts[i*2];
      }
    }

    // Note: The specification indicates that no event is normally returned.
  }
  else // bad parameters
  {
    hciStatus_t status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;

    // Note: The specification indicates that no event is normally returned,
    //       except if there are invalid parameters.
    HCI_CommandCompleteEvent( HCI_HOST_NUM_COMPLETED_PACKETS, sizeof(status), &status);
  }

  return( HCI_SUCCESS );
}


/*
** Information Parameters
*/

/*******************************************************************************
 * This BT API is used to read the local version information.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ReadLocalVersionInfoCmd( void )
{
  // 0: Status
  // 1: HCI Version Number
  // 2: HCI Revision Number LSB
  // 3: HCI Revision Number MSB
  // 4: Version Number
  // 5: Connection Handle LSB
  // 6: Connection Handle MSB
  // 7: LL Subversion Number LSB
  // 8: LL Subversion Number MSB
  uint8  rtnParam[9];
  uint8  version;
  uint16 comID;
  uint16 subverNum;

  // status
  rtnParam[0] = LL_ReadLocalVersionInfo( &version,
                                         &comID,
                                         &subverNum );

  // HCI version and revision
  rtnParam[1] = HCI_VERSION;
  rtnParam[2] = LO_UINT16( HCI_REVISION );
  rtnParam[3] = HI_UINT16( HCI_REVISION );

  // LL version, manufacturer name, LL subversion
  rtnParam[4] = version;
  rtnParam[5] = LO_UINT16( comID );
  rtnParam[6] = HI_UINT16( comID );
  rtnParam[7] = LO_UINT16( subverNum );
  rtnParam[8] = HI_UINT16( subverNum );

  HCI_CommandCompleteEvent( HCI_READ_LOCAL_VERSION_INFO, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This BT API is used to read the locally supported commands.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ReadLocalSupportedCommandsCmd( void )
{
  // 0:     Status (HCI_SUCCESS)
  // 1..64: Supported Commands
  HCI_CommandCompleteEvent( HCI_READ_LOCAL_SUPPORTED_COMMANDS,
                            SUPPORTED_COMMAND_LEN+1,
                            (uint8 *)supportedCmdsTable );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This BT API is used to read the locally supported features.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ReadLocalSupportedFeaturesCmd( void )
{
  // 0:    Status
  // 1..8: Supported Features
  uint8 rtnParam[9] = {HCI_SUCCESS, 0, 0, 0, 0, 0, 0, 0, 0};

  // set byte 4 of the feature list, which is the only byte that matters
  rtnParam[5] = LOCAL_SUPPORTED_FEATURE_SET_BYTE_4;

  HCI_CommandCompleteEvent( HCI_READ_LOCAL_SUPPORTED_FEATURES, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This BT API is used to read this device's BLE address (BDADDR).
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ReadBDADDRCmd( void )
{
  // 0:    Status
  // 1..6: BDADDR
  uint8 rtnParam[7];

  // status
  rtnParam[0] = LL_ReadBDADDR( &(rtnParam[1]) );

  HCI_CommandCompleteEvent( HCI_READ_BDADDR, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}

/*
** Status Parameters
*/

/*******************************************************************************
 * This BT API is used to read the RSSI.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_ReadRssiCmd( uint16 connHandle )
{
  // 0: Status
  // 1: Connection Handle LSB
  // 2: Connection Handle MSB
  // 3: RSSI
  uint8 rtnParam[4];

  // status
  rtnParam[0] = LL_ReadRssi( connHandle,
                             (int8*) &(rtnParam[3]) );

  // connection handle
  rtnParam[1] = LO_UINT16( connHandle);
  rtnParam[2] = HI_UINT16( connHandle );

  HCI_CommandCompleteEvent( HCI_READ_RSSI, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}


/*
** HCI Low Energy Commands
*/

/*******************************************************************************
 * This LE API is used to set the HCI LE event mask, which is used to determine
 * which LE events are supported.
 *
 * Note: The global bleEvtMask is used for LE events. A different global is used
 *       for BT events: pHciEvtMask.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetEventMaskCmd( uint8 *pEventMask )
{
  hciStatus_t status;

  // check parameters
  if ( pEventMask != NULL )
  {
    // set the BLE event mask
    // Note: So far, only the first byte is used.
    //bleEvtMask = pEventMask[0];

    // extend le_meta Event masker to 32bit by ZQ 20181031
    // according to Core 5.0 LE Meta EVENT number is 20
    bleEvtMask = BUILD_UINT32(pEventMask[0], pEventMask[1], pEventMask[2], 0x00);

    status = HCI_SUCCESS;
  }
  else // bad parameters
  {
    status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }

  HCI_CommandCompleteEvent( HCI_LE_SET_EVENT_MASK, sizeof(status), &status );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used by the Host to determine the maximum ACL data packet
 * size allowed by the Controller.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadBufSizeCmd( void )
{
  // 0: Status
  // 1: Data Packet Length LSB
  // 2: Data Packet Length MSB
  // 3: Buffer Size
  uint8 rtnParam[4];

  // status
  rtnParam[0] = HCI_SUCCESS;

  // data packet length
  rtnParam[1] = LO_UINT16( HCI_DATA_MAX_DATA_LENGTH );
  rtnParam[2] = HI_UINT16( HCI_DATA_MAX_DATA_LENGTH );

  // number of data packets allowed by Controller
  rtnParam[3] = HCI_MAX_NUM_DATA_BUFFERS;

  HCI_CommandCompleteEvent( HCI_LE_READ_BUFFER_SIZE, sizeof(rtnParam), rtnParam);

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to read the LE locally supported features.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadLocalSupportedFeaturesCmd( void )
{
  // 0:    Status
  // 1..8: Local Supported Features
  uint8 rtnParam[9];

  rtnParam[0] = LL_ReadLocalSupportedFeatures( &(rtnParam[1]) );

  HCI_CommandCompleteEvent( HCI_LE_READ_LOCAL_SUPPORTED_FEATURES, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}



/*******************************************************************************
 * This LE API is used to set this device's Random address.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetRandomAddressCmd( uint8 *pRandAddr )
{
  hciStatus_t status;

  // check parameters
  if ( pRandAddr != NULL )
  {
    status = LL_SetRandomAddress( pRandAddr );
  }
  else // bad parameters
  {
    status = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
  }

  HCI_CommandCompleteEvent( HCI_LE_SET_RANDOM_ADDR, sizeof(status), &status );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to set the Advertising parameters.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetAdvParamCmd( uint16 advIntervalMin,
                                   uint16 advIntervalMax,
                                   uint8  advType,
                                   uint8  ownAddrType,
                                   uint8  directAddrType,
                                   uint8  *directAddr,
                                   uint8  advChannelMap,
                                   uint8  advFilterPolicy )
{
  hciStatus_t status;

  status = LL_SetAdvParam( advIntervalMin,
                           advIntervalMax,
                           advType,
                           ownAddrType,
                           directAddrType,
                           directAddr,
                           advChannelMap,
                           advFilterPolicy );

  HCI_CommandCompleteEvent( HCI_LE_SET_ADV_PARAM, sizeof(status), &status );

  return( HCI_SUCCESS );
}




/*******************************************************************************
 * This LE API is used to set the Advertising data.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetAdvDataCmd( uint8 dataLen,
                                  uint8 *pData )
{
  hciStatus_t status;

  status = LL_SetAdvData( dataLen, pData );

  HCI_CommandCompleteEvent( HCI_LE_SET_ADV_DATA, sizeof(status), &status );

  return( HCI_SUCCESS );
}



/*******************************************************************************
 * This LE API is used to set the Advertising Scan Response data.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetScanRspDataCmd( uint8 dataLen,
                                      uint8 *pData )
{
  hciStatus_t status;

  status = LL_SetScanRspData( dataLen,
                              pData );

  HCI_CommandCompleteEvent( HCI_LE_SET_SCAN_RSP_DATA, sizeof(status), &status );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to turn Advertising on or off.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetAdvEnableCmd( uint8 advEnable )
{
  hciStatus_t status;

  status = LL_SetAdvControl( advEnable );

  HCI_CommandCompleteEvent( HCI_LE_SET_ADV_ENABLE, sizeof(status), &status );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to read transmit power when Advertising.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadAdvChanTxPowerCmd( void )
{
  // 0: Status
  // 1: Advertising Transmit Power
  uint8 rtnParam[2];
// TODO
  // status
//  rtnParam[0] = LL_ReadAdvChanTxPower( (int8*)&(rtnParam[1]) );

  HCI_CommandCompleteEvent( HCI_LE_READ_ADV_CHANNEL_TX_POWER, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to set the Scan parameters.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetScanParamCmd( uint8  scanType,
                                    uint16 scanInterval,
                                    uint16 scanWindow,
                                    uint8  ownAddrType,
                                    uint8  filterPolicy )
{
  hciStatus_t status;

  status = LL_SetScanParam( scanType,
                            scanInterval,
                            scanWindow,
                            ownAddrType,
                            filterPolicy );

  HCI_CommandCompleteEvent( HCI_LE_SET_SCAN_PARAM, sizeof(status), &status );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to turn Scanning on or off.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetScanEnableCmd( uint8 scanEnable,
                                     uint8 filterDuplicates )
{
  hciStatus_t status;

  status = LL_SetScanControl( scanEnable,
                              filterDuplicates );

  HCI_CommandCompleteEvent( HCI_LE_SET_SCAN_ENABLE, sizeof(status), &status );

  return( HCI_SUCCESS );
}

/*******************************************************************************
 * This LE API is used to update the current data channel map.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetHostChanClassificationCmd( uint8 *chanMap )
{
  hciStatus_t status;

//  status = LL_ChanMapUpdate( chanMap );

  HCI_CommandCompleteEvent( HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION, sizeof(status), &status );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to read a connection's data channel map.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadChannelMapCmd( uint16 connHandle )
{
  // 0:    Status
  // 1:    Connection Handle LSB
  // 2:    Connection Handle MSB
  // 3..7: Channel Map (LSB to MSB)
  uint8 rtnParam[8];

  rtnParam[0] = LL_ReadChanMap( connHandle,
                                &(rtnParam[3]) );

  // connection handle
  rtnParam[1] = LO_UINT16( connHandle );
  rtnParam[2] = HI_UINT16( connHandle );

  HCI_CommandCompleteEvent( HCI_LE_READ_CHANNEL_MAP, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to read the remote device's used features.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadRemoteUsedFeaturesCmd( uint16 connHandle )
{
  hciStatus_t status;

  status = LL_ReadRemoteUsedFeatures( connHandle );

  HCI_CommandStatusEvent( status, HCI_LE_READ_REMOTE_USED_FEATURES );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to perform an encryption using AES128.
 *
 * Note: Input parameters are ordered MSB..LSB.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_EncryptCmd( uint8 *key,
                               uint8 *plainText )
{
  // 0:     Status
  // 1..16: Encrypted Data
  uint8 rtnParam[17];

  rtnParam[0] = LL_Encrypt( key,
                            plainText,
                            &rtnParam[1] );

  // check for success
  if ( rtnParam[0] == LL_STATUS_SUCCESS )
  {
    // encryption process reverses the bytes, so restore to MSB..LSB
    HCI_ReverseBytes( &rtnParam[1], KEYLEN );

    HCI_CommandCompleteEvent( HCI_LE_ENCRYPT, sizeof(rtnParam), rtnParam );
  }
  else // error
  {
    HCI_CommandCompleteEvent( HCI_LE_ENCRYPT, sizeof(uint8), rtnParam );
  }

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to generate a random number.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_RandCmd( void )
{
  // 0:    Status
  // 1..8: Random Bytes
  uint8 rtnParam[B_RANDOM_NUM_SIZE+1];

  rtnParam[0] = LL_Rand( &rtnParam[1], B_RANDOM_NUM_SIZE );

  // check if the operation has been completed; if not, then it has been delayed
  // until a current radio operation completes as the radio is needed to
  // generate a true random number, or there was some kind of error
  if ( rtnParam[0] != LL_STATUS_ERROR_DUE_TO_DELAYED_RESOURCES )
  {
    // check if the operation was okay
    if ( rtnParam[0] == LL_STATUS_SUCCESS )
    {
      HCI_CommandCompleteEvent( HCI_LE_RAND, B_RANDOM_NUM_SIZE+1, rtnParam );
    }
    else // an error occurred
    {
      HCI_CommandCompleteEvent( HCI_LE_RAND, sizeof(uint8), rtnParam );
    }
  }

  return( HCI_SUCCESS );
}

/*******************************************************************************
 * This LE API is used by the Host to send to the Controller a positive LTK
 * reply.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_LtkReqReplyCmd( uint16 connHandle,
                                   uint8  *ltk )
{
  // 0: Status
  // 1: Connection Handle (LSB)
  // 2: Connection Handle (MSB)
  uint8 rtnParam[3];

  rtnParam[0] = LL_EncLtkReply( connHandle, ltk );
  rtnParam[1] = LO_UINT16( connHandle );
  rtnParam[2] = HI_UINT16( connHandle );

  HCI_CommandCompleteEvent( HCI_LE_LTK_REQ_REPLY, sizeof(rtnParam), rtnParam );

  return ( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used by the Host to send to the Controller a negative LTK
 * reply.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_LtkReqNegReplyCmd( uint16 connHandle )
{
  // 0: Status
  // 1: Connection Handle (LSB)
  // 2: Connection Handle (MSB)
  uint8 rtnParam[3];

  rtnParam[0] = LL_EncLtkNegReply( connHandle );
  rtnParam[1] = LO_UINT16( connHandle );
  rtnParam[2] = HI_UINT16( connHandle );

  HCI_CommandCompleteEvent( HCI_LE_LTK_REQ_NEG_REPLY, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to read the Controller's supported states.
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadSupportedStatesCmd( void )
{
  // 0:    Status
  // 1..8: Supported States
  uint8 rtnParam[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  rtnParam[0] = LL_ReadSupportedStates( &rtnParam[1] );

  HCI_CommandCompleteEvent( HCI_LE_READ_SUPPORTED_STATES, sizeof(rtnParam), rtnParam );

  return( HCI_SUCCESS );
}

/*******************************************************************************
 * This LE API is used to set data length 
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetDataLengthCmd( uint16 connHandle,
                                       uint16 TxOctets,
                                       uint16 TxTime )
{
  // 0: Status
  // 1: Connection Handle (LSB)
  // 2: Connection Handle (MSB)
  uint8 rtnParam[3];

  rtnParam[0] = LL_SetDataLengh( connHandle,TxOctets,TxTime );
  rtnParam[1] = LO_UINT16( connHandle );
  rtnParam[2] = HI_UINT16( connHandle );

  HCI_CommandCompleteEvent( HCI_LE_SET_DATA_LENGTH, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to read max Data length
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadMaxDataLengthCmd(void)
{

  uint8 rtnParam[9];

  rtnParam[0] = 0x00;//status

  rtnParam[1] = LO_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_TX_OCTECTS);
  rtnParam[2] = HI_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_TX_OCTECTS );

  rtnParam[3] = LO_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_TX_TIME);
  rtnParam[4] = HI_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_TX_TIME );

  rtnParam[5] = LO_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_RX_OCTECTS);
  rtnParam[6] = HI_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_RX_OCTECTS );

  rtnParam[7] = LO_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_RX_TIME);
  rtnParam[8] = HI_UINT16( LL_PDU_LENGTH_SUPPORTED_MAX_RX_TIME );

  HCI_CommandCompleteEvent( HCI_LE_READ_MAXIMUM_DATA_LENGTH, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}


/*******************************************************************************
 * This LE API is used to read Suggested Default max Data length
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadSuggestedDefaultDataLengthCmd(void)
{

  uint8 rtnParam[5];

  rtnParam[0] = 0x00;//status

  rtnParam[1] = LO_UINT16( g_llPduLen.suggested.MaxTxOctets);
  rtnParam[2] = HI_UINT16( g_llPduLen.suggested.MaxTxOctets );

  rtnParam[3] = LO_UINT16( g_llPduLen.suggested.MaxTxTime);
  rtnParam[4] = HI_UINT16( g_llPduLen.suggested.MaxTxTime );

  HCI_CommandCompleteEvent( HCI_LE_READ_SUGGESTED_DEFAULT_DATA_LENGTH, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}

/*******************************************************************************
 * This LE API is used to write Suggested Default Data length
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_WriteSuggestedDefaultDataLengthCmd(uint16 suggestedMaxTxOctets,uint16 suggestedMaxTxTime)
{

  hciStatus_t status;

  status = LL_WriteSuggestedDefaultDataLength(suggestedMaxTxOctets,suggestedMaxTxTime);

  HCI_CommandCompleteEvent( HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH, sizeof(status), &status );
  return( HCI_SUCCESS );
}
/*******************************************************************************
 * This LE API is used to set DefaultPhyMode
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetDefaultPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy)
{

  hciStatus_t status;

  status = LL_SetDefaultPhyMode( connId, allPhy, txPhy,  rxPhy);

  HCI_CommandCompleteEvent( HCI_LE_SET_DEFAULT_PHY, sizeof(status), &status );
  return( HCI_SUCCESS );
}

/*******************************************************************************
 * This LE API is used to Set PHY Mode
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions)
{

  HCI_CommandStatusEvent( LL_SetPhyMode( connId, allPhy, txPhy,  rxPhy,phyOptions), HCI_LE_SET_PHY );
  return( HCI_SUCCESS );
}
/*******************************************************************************
 * This LE API is used to Read PHY Mode
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadPhyMode( uint16 connId)
{

  uint8 rtnParam[5];

  rtnParam[0] = 0x00;//status

  rtnParam[1] = LO_UINT16( connId );
  rtnParam[2] = HI_UINT16( connId );

  rtnParam[3] = conn_param[connId].llPhyModeCtrl.local.txPhy;
  rtnParam[4] = conn_param[connId].llPhyModeCtrl.local.rxPhy;;

  HCI_CommandCompleteEvent( HCI_LE_READ_PHY, sizeof(rtnParam), rtnParam );
  return( HCI_SUCCESS );
}

/*
** LL Callback Functions
*/

/*******************************************************************************
 * This LL command Callback is used by the LL to notify the HCI that the LE
 * RAND command has been completed.
 *
 * Note: The length is always given by B_RANDOM_NUM_SIZE.
 *
 * Public function defined in hci.h.
 */
void LL_RandCback( uint8 *randData )
{
  // 0:    Status
  // 1..8: Random Bytes
  uint8 rtnParam[B_RANDOM_NUM_SIZE+1];

  rtnParam[0] = LL_STATUS_SUCCESS;

  // copy random data block
  (void)osal_memcpy( &rtnParam[1], randData, B_RANDOM_NUM_SIZE );

  HCI_CommandCompleteEvent( HCI_LE_RAND, B_RANDOM_NUM_SIZE+1, rtnParam );

  return;
}

/***************************************************************************************************
 */
 
