/*******************************************************************************
    Filename:       hci.c
    Revised:
    Revision:

    Description:    This file contains the Host Controller Interface (HCI) API.


*******************************************************************************/

/*******************************************************************************
    INCLUDES
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
    MACROS
*/

/*******************************************************************************
    CONSTANTS
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
    TYPEDEFS
*/

typedef const uint8 supportedCmdsTable_t;

/*******************************************************************************
    EXTERNAL VARIABLES
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
    LOCAL VARIABLES
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
    GLOBAL VARIABLES
*/
uint8  hciPTMenabled;
uint8  ctrlToHostEnable;
uint16 numHostBufs;
uint8  hciCtrlCmdToken;        // counter of the number of usable command buffers

/*******************************************************************************
    HCI API
*/

/*
** Buffer Management
*/

/*******************************************************************************
    This API is used to allocate memory using buffer management.

    Public function defined in hci.h.
*/
void* HCI_bm_alloc( uint16 size )
{
    return( LL_TX_bm_alloc( size ) );
}


/*******************************************************************************
    This API is used to check that the connection time parameters are valid.

    Public function defined in hci.h.
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
    This API is used to send a ACL data packet over a connection.

    NOTE: L2CAP is affected by this routine's status values as it must remap
         them to Host status values. If any additional status values are added
         and/or changed in this routine, a TI stack engineer must be notified!

    Public function defined in hci.h.
*/
hciStatus_t HCI_SendDataPkt( uint16 connHandle,
                             uint8  pbFlag,
                             uint16 pktLen,
                             uint8*  pData )
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
    This BT API is used to terminate a connection.

    Public function defined in hci.h.
*/
hciStatus_t HCI_DisconnectCmd( uint16 connHandle,
                               uint8  reason )

{
    HCI_CommandStatusEvent( LL_Disconnect(connHandle, reason), HCI_DISCONNECT );
    return( HCI_SUCCESS );
}



/*******************************************************************************
    This BT API is used to request version information from the the remote
    device in a connection.

    Public function defined in hci.h.
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
    This BT API is used to set the HCI event mask, which is used to determine
    which events are supported.

    Note: The global pHciEvtMask is used for BT events. A different global is
         used for LE events: bleEvtMask.

    Public function defined in hci.h.
*/
hciStatus_t HCI_SetEventMaskCmd( uint8* pMask )
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

    This BT API is used to reset the Link Layer.

    Public function defined in hci.h.
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

    This BT API is used to read the transmit power level.

    Public function defined in hci.h.
*/
hciStatus_t HCI_ReadTransmitPowerLevelCmd( uint16 connHandle,
                                           uint8  txPwrType )
{
    // 0: Status
    // 1: Connection Handle LSB
    // 2: Connection Handle MSB
    // 3: Transmit Power Level
    uint8 rtnParam[4];
    rtnParam[0] = LL_ReadTxPowerLevel( connHandle,
                                       txPwrType,
                                       (int8*)&(rtnParam[3]) );
    rtnParam[1] = LO_UINT16( connHandle );
    rtnParam[2] = HI_UINT16( connHandle );
    HCI_CommandCompleteEvent( HCI_READ_TRANSMIT_POWER, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}



/*******************************************************************************

    This BT API is used by the Host to turn flow control on or off for data sent
    from the Controller to Host.

    Public function defined in hci.h.
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

    This BT API is used by the Host to notify the Controller of the maximum size
    ACL buffer size the Controller can send to the Host.

    Public function defined in hci.h.
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
    This BT API is used by the Host to notify the Controller of the number of
    HCI data packets that have been completed for each connection handle since
    this command was previously sent to the controller.

    Note: It is assumed that there will be at most only one handle. Even if more
         than one handle is provided, the Controller does not track Host buffers
         as a function of connection handles (and isn't required to do so).
    Note: The connection handle is not used.

    Public function defined in hci.h.
*/
hciStatus_t HCI_HostNumCompletedPktCmd( uint8   numHandles,
                                        uint16* connHandles,
                                        uint16* numCompletedPkts )
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
    This BT API is used to read the local version information.

    Public function defined in hci.h.
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
    This BT API is used to read the locally supported commands.

    Public function defined in hci.h.
*/
hciStatus_t HCI_ReadLocalSupportedCommandsCmd( void )
{
    // 0:     Status (HCI_SUCCESS)
    // 1..64: Supported Commands
    HCI_CommandCompleteEvent( HCI_READ_LOCAL_SUPPORTED_COMMANDS,
                              SUPPORTED_COMMAND_LEN+1,
                              (uint8*)supportedCmdsTable );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This BT API is used to read the locally supported features.

    Public function defined in hci.h.
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
    This BT API is used to read this device's BLE address (BDADDR).

    Public function defined in hci.h.
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
    This BT API is used to read the RSSI.

    Public function defined in hci.h.
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
    This LE API is used to set the HCI LE event mask, which is used to determine
    which LE events are supported.

    Note: The global bleEvtMask is used for LE events. A different global is used
         for BT events: pHciEvtMask.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetEventMaskCmd( uint8* pEventMask )
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
    This LE API is used by the Host to determine the maximum ACL data packet
    size allowed by the Controller.

    Public function defined in hci.h.
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
    This LE API is used to read the LE locally supported features.

    Public function defined in hci.h.
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
    This LE API is used to set this device's Random address.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetRandomAddressCmd( uint8* pRandAddr )
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
    This LE API is used to set the Advertising parameters.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetAdvParamCmd( uint16 advIntervalMin,
                                   uint16 advIntervalMax,
                                   uint8  advType,
                                   uint8  ownAddrType,
                                   uint8  directAddrType,
                                   uint8*  directAddr,
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
    This LE API is used to set the Advertising data.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetAdvDataCmd( uint8 dataLen,
                                  uint8* pData )
{
    hciStatus_t status;
    status = LL_SetAdvData( dataLen, pData );
    HCI_CommandCompleteEvent( HCI_LE_SET_ADV_DATA, sizeof(status), &status );
    return( HCI_SUCCESS );
}



/*******************************************************************************
    This LE API is used to set the Advertising Scan Response data.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetScanRspDataCmd( uint8 dataLen,
                                      uint8* pData )
{
    hciStatus_t status;
    status = LL_SetScanRspData( dataLen,
                                pData );
    HCI_CommandCompleteEvent( HCI_LE_SET_SCAN_RSP_DATA, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to turn Advertising on or off.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetAdvEnableCmd( uint8 advEnable )
{
    hciStatus_t status;
    status = LL_SetAdvControl( advEnable );
    HCI_CommandCompleteEvent( HCI_LE_SET_ADV_ENABLE, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to read transmit power when Advertising.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReadAdvChanTxPowerCmd( void )
{
    // 0: Status
    // 1: Advertising Transmit Power
    uint8 rtnParam[2];
    // status
    rtnParam[0] = LL_ReadAdvChanTxPower( (int8*)&(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_ADV_CHANNEL_TX_POWER, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to set the Scan parameters.

    Public function defined in hci.h.
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
    This LE API is used to turn Scanning on or off.

    Public function defined in hci.h.
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
    This LE API is used to create a connection.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_CreateConnCmd( uint16 scanInterval,
                                  uint16 scanWindow,
                                  uint8  initFilterPolicy,
                                  uint8  addrTypePeer,
                                  uint8*  peerAddr,
                                  uint8  ownAddrType,
                                  uint16 connIntervalMin,
                                  uint16 connIntervalMax,
                                  uint16 connLatency,
                                  uint16 connTimeout,
                                  uint16 minLen,
                                  uint16 maxLen )
{
    hciStatus_t status;
    status = LL_CreateConn( scanInterval,
                            scanWindow,
                            initFilterPolicy,
                            addrTypePeer,
                            peerAddr,
                            ownAddrType,
                            connIntervalMin,
                            connIntervalMax,
                            connLatency,
                            connTimeout,
                            minLen,
                            maxLen );
    HCI_CommandStatusEvent( status, HCI_LE_CREATE_CONNECTION );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to cancel a create connection.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_CreateConnCancelCmd( void )
{
    hciStatus_t status;
    status = LL_CreateConnCancel();
    HCI_CommandCompleteEvent( HCI_LE_CREATE_CONNECTION_CANCEL, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to read the total number of white list entries that can
    be stored in the Controller.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReadWhiteListSizeCmd( void )
{
    // 0: Status
    // 1: White List Size
    uint8 rtnParam[2];
    rtnParam[0] = LL_ReadWlSize( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_WHITE_LIST_SIZE, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to clear the white list.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ClearWhiteListCmd( void )
{
    hciStatus_t status;
    status = LL_ClearWhiteList();
    HCI_CommandCompleteEvent( HCI_LE_CLEAR_WHITE_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to add a white list entry.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_AddWhiteListCmd( uint8  addrType,
                                    uint8* devAddr )
{
    hciStatus_t status;
    status = LL_AddWhiteListDevice( devAddr,
                                    addrType );
    HCI_CommandCompleteEvent( HCI_LE_ADD_WHITE_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to remove a white list entry.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_RemoveWhiteListCmd( uint8 addrType,
                                       uint8* devAddr )
{
    hciStatus_t status;
    status = LL_RemoveWhiteListDevice( devAddr,
                                       addrType );
    HCI_CommandCompleteEvent( HCI_LE_REMOVE_WHITE_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to update the connection parameters.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ConnUpdateCmd( uint16 connHandle,
                                  uint16 connIntervalMin,
                                  uint16 connIntervalMax,
                                  uint16 connLatency,
                                  uint16 connTimeout,
                                  uint16 minLen,
                                  uint16 maxLen )
{
    hciStatus_t status;
    status = LL_ConnUpdate( connHandle,
                            connIntervalMin,
                            connIntervalMax,
                            connLatency,
                            connTimeout,
                            minLen,
                            maxLen );
    HCI_CommandStatusEvent( status, HCI_LE_CONNECTION_UPDATE );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to update the current data channel map.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetHostChanClassificationCmd( uint8* chanMap )
{
    hciStatus_t status;
    status = LL_ChanMapUpdate( chanMap );
    HCI_CommandCompleteEvent( HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to read a connection's data channel map.

    Public function defined in hci.h.
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
    This LE API is used to read the remote device's used features.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReadRemoteUsedFeaturesCmd( uint16 connHandle )
{
    hciStatus_t status;
    status = LL_ReadRemoteUsedFeatures( connHandle );
    HCI_CommandStatusEvent( status, HCI_LE_READ_REMOTE_USED_FEATURES );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to perform an encryption using AES128.

    Note: Input parameters are ordered MSB..LSB.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_EncryptCmd( uint8* key,
                               uint8* plainText )
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
    This LE API is used to generate a random number.

    Public function defined in hci.h.
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
    This LE API is used to start encryption in a connection.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_StartEncyptCmd( uint16 connHandle,
                                   uint8*  random,
                                   uint8*  encDiv,
                                   uint8*  ltk )
{
    hciStatus_t status;
    status = LL_StartEncrypt( connHandle,
                              random,
                              encDiv,
                              ltk );
    HCI_CommandStatusEvent( status, HCI_LE_START_ENCRYPTION );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used by the Host to send to the Controller a positive LTK
    reply.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_LtkReqReplyCmd( uint16 connHandle,
                                   uint8*  ltk )
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
    This LE API is used by the Host to send to the Controller a negative LTK
    reply.

    Public function defined in hci.h.
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
    This LE API is used to read the Controller's supported states.

    Public function defined in hci.h.
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
    This LE API is used to start the receiver Direct Test Mode test.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReceiverTestCmd( uint8 rxFreq )
{
    hciStatus_t status;
// status = LL_DirectTestRxTest( rxFreq );
    HCI_CommandCompleteEvent( HCI_LE_RECEIVER_TEST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to start the transmit Direct Test Mode test.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_TransmitterTestCmd( uint8 txFreq,
                                       uint8 dataLen,
                                       uint8 payloadType )
{
    hciStatus_t status;
    status = LL_DirectTestTxTest( txFreq,
                                  dataLen,
                                  payloadType );
    HCI_CommandCompleteEvent( HCI_LE_TRANSMITTER_TEST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to end the Direct Test Mode test.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_TestEndCmd( void )
{
    hciStatus_t status = LL_DirectTestEnd();

    if ( status != HCI_SUCCESS )
    {
        // 0:    Status
        // 1..2: Number of Packets (for Receive DTM only)
        uint8 rtnParam[3];
        rtnParam[0] = HCI_ERROR_CODE_CMD_DISALLOWED;
        // not valid if LL API failed; otherwise values returned by a
        // LL_DirectTestEndDoneCback event
        rtnParam[1] = 0;
        rtnParam[2] = 0;
        HCI_CommandCompleteEvent( HCI_LE_TEST_END, sizeof(rtnParam), rtnParam );
    }

    return( HCI_SUCCESS );
}



/*******************************************************************************
    This LE API is used to set data length

    Public function defined in hci.h.
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
    This LE API is used to read max Data length

    Public function defined in hci.h.
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
    This LE API is used to read Suggested Default max Data length

    Public function defined in hci.h.
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
    This LE API is used to write Suggested Default Data length

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_WriteSuggestedDefaultDataLengthCmd(uint16 suggestedMaxTxOctets,uint16 suggestedMaxTxTime)
{
    hciStatus_t status;
    status = LL_WriteSuggestedDefaultDataLength(suggestedMaxTxOctets,suggestedMaxTxTime);
    HCI_CommandCompleteEvent( HCI_LE_WRITE_SUGGESTED_DEFAULT_DATA_LENGTH, sizeof(status), &status );
    return( HCI_SUCCESS );
}
/*******************************************************************************
    This LE API is used to set DefaultPhyMode

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetDefaultPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy)
{
    hciStatus_t status;
    status = LL_SetDefaultPhyMode( connId, allPhy, txPhy,  rxPhy);
    HCI_CommandCompleteEvent( HCI_LE_SET_DEFAULT_PHY, sizeof(status), &status );
    return( HCI_SUCCESS );
}

/*******************************************************************************
    This LE API is used to Set PHY Mode

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions)
{
    HCI_CommandStatusEvent( LL_SetPhyMode( connId, allPhy, txPhy,  rxPhy,phyOptions), HCI_LE_SET_PHY );
    return( HCI_SUCCESS );
}
/*******************************************************************************
    This LE API is used to Read PHY Mode

    Public function defined in hci.h.
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


// Device Address Type
//#define HCI_PUBLIC_DEVICE_ADDRESS                      LL_DEV_ADDR_TYPE_PUBLIC
//#define HCI_RANDOM_DEVICE_ADDRESS                      LL_DEV_ADDR_TYPE_RANDOM
/*******************************************************************************
    This LE API is used to add a resolving list entry.

          Peer_Identity_Address_Type,
          Peer_Identity_Address,
          Peer_IRK,
          Local_IRK
    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_AddDevToResolvingListCmd( uint8  addrType,
                                             uint8* devAddr,
                                             uint8* peerIrk,
                                             uint8* localIrk)
{
    hciStatus_t status;
    status = LL_AddResolvingListLDevice(addrType,
                                        devAddr,
                                        peerIrk,
                                        localIrk);
    HCI_CommandCompleteEvent( HCI_LE_ADD_DEVICE_TO_RESOLVING_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to remove a resolving list entry.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_RemoveResolvingListCmd( uint8 addrType,
                                           uint8* devAddr )
{
    hciStatus_t status;
    status = LL_RemoveResolvingListDevice(devAddr, addrType);
    HCI_CommandCompleteEvent( HCI_LE_REMOVE_DEVICE_FROM_RESOLVING_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}

/*******************************************************************************
    This LE API is used to clear the resolving list.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ClearResolvingListCmd( void )
{
    hciStatus_t status;
    status = LL_ClearResolvingList();
    HCI_CommandCompleteEvent( HCI_LE_CLEAR_RESOLVING_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to read the total number of resolving list entries that can
    be stored in the Controller.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReadResolvingListSizeCmd( void )
{
    uint8 rtnParam[2];
    rtnParam[0] = LL_ReadResolvingListSize( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_RESOLVING_LIST_SIZE, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


hciStatus_t HCI_LE_ReadPeerResolvableAddressCmd( uint8  peerIdAddrType,
                                                 uint8* peerIdAddr)
{
    uint8 rtnParam[7];
    rtnParam[0] = LL_ReadPeerResolvableAddress( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_PEER_RESOLVABLE_ADDRESS, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_ReadLocalResolvableAddressCmd( uint8  peerIdAddrType,
                                                  uint8* peerIdAddr)
{
    uint8 rtnParam[7];
    rtnParam[0] = LL_ReadLocalResolvableAddress( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_LOCAL_RESOLVABLE_ADDRESS, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

/*******************************************************************************
    This LE API is used to enable/disable address resolution

    0 - disable
    1 - enable
    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetAddressResolutionEnableCmd( uint8 enable )
{
    hciStatus_t status;
    status = LL_SetAddressResolutionEnable(enable);
    HCI_CommandCompleteEvent( HCI_LE_SET_ADDRESS_RESOLUTION_ENABLE, sizeof(status), &status );
    return( HCI_SUCCESS );
}

/*******************************************************************************
    This LE API is used to set resolvable private address timeout


    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_SetResolvablePrivateAddressTimeoutCmd( uint16 rpaTimeout )
{
    hciStatus_t status;

    if (rpaTimeout > 0xA1B8 || rpaTimeout == 0)
        return HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;

    status = LL_SetResolvablePrivateAddressTimeout(rpaTimeout);
    HCI_CommandCompleteEvent( HCI_LE_SET_RESOLVABLE_PRIVATE_ADDRESS_TO, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*
** HCI for Extended Adv
*/
//
hciStatus_t HCI_LE_SetExtAdvSetRandomAddressCmd( uint8 adv_handle,
                                                 uint8* random_address
                                               )
{
    hciStatus_t status;
    status = LL_SetExtAdvSetRandomAddress(adv_handle, random_address);
    HCI_CommandCompleteEvent( HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtAdvParamCmd( uint8 adv_handle,
                                      uint16 adv_event_properties,
                                      uint32 primary_advertising_interval_Min,          // 3 octets
                                      uint32 primary_advertising_interval_Max,          // 3 octets
                                      uint8  primary_advertising_channel_map,
                                      uint8  own_address_type,
                                      uint8  peer_address_type,
                                      uint8* peer_address,
                                      uint8  advertising_filter_policy,
                                      int8   advertising_tx_power,
                                      uint8  primary_advertising_PHY,
                                      uint8  secondary_advertising_max_skip,
                                      uint8  secondary_advertising_PHY,
                                      uint8  advertising_SID,
                                      uint8  scan_request_notification_enable
                                    )
{
    uint8 rtnParam[2];     // octect 0: status, octect 1: Selected_Tx_Power
    int8 selectTxPower;
    rtnParam[0] = LL_SetExtAdvParam( adv_handle,
                                     adv_event_properties,
                                     primary_advertising_interval_Min,          // 3 octets
                                     primary_advertising_interval_Max,          // 3 octets
                                     primary_advertising_channel_map,
                                     own_address_type,
                                     peer_address_type,
                                     peer_address,
                                     advertising_filter_policy,
                                     advertising_tx_power,
                                     primary_advertising_PHY,
                                     secondary_advertising_max_skip,
                                     secondary_advertising_PHY,
                                     advertising_SID,
                                     scan_request_notification_enable, &selectTxPower);
    rtnParam[1] = selectTxPower;
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDER_ADVERTISING_PARAMETERS, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


//
hciStatus_t HCI_LE_SetExtAdvDataCmd( uint8 adv_handle,
                                     uint8 operation,
                                     uint8  fragment_preference,
                                     uint8  advertising_data_length,
                                     uint8* advertising_data
                                   )
{
    hciStatus_t status;
    status = LL_SetExtAdvData(adv_handle,
                              operation,
                              fragment_preference,
                              advertising_data_length,
                              advertising_data
                             );
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDED_ADVERTISING_DATA, sizeof(status), &status );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_SetExtScanRspDataCmd( uint8 adv_handle,
                                         uint8 operation,
                                         uint8  fragment_preference,
                                         uint8  scan_rsp_data_length,
                                         uint8* scan_rsp_data
                                       )
{
    hciStatus_t status;
    status = LL_SetExtScanRspData(adv_handle,
                                  operation,
                                  fragment_preference,
                                  scan_rsp_data_length,
                                  scan_rsp_data
                                 );
    HCI_CommandCompleteEvent( HCI_LE_Set_EXTENDED_SCAN_RESPONSE_DATA, sizeof(status), &status );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_SetExtAdvEnableCmd( uint8  enable,
                                       uint8  number_of_sets,
                                       uint8*  advertising_handle,
                                       uint16* duration,
                                       uint8*  max_extended_advertising_events)
{
    hciStatus_t status;
    status = LL_SetExtAdvEnable(enable, number_of_sets, advertising_handle, duration, max_extended_advertising_events);
    HCI_CommandCompleteEvent( HCI_LE_Set_EXTENDED_ADVERTISING_ENABLE, sizeof(status), &status );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_ReadMaximumAdvDataLengthCmd( void )
{
    uint8  rtnParam[3];
    uint16 len;
    rtnParam[0] = LL_ReadMaximumAdvDataLength( &len);
    rtnParam[1] = LO_UINT16( len);
    rtnParam[2] = HI_UINT16( len );
    HCI_CommandCompleteEvent( HCI_LE_READ_MAXIMUM_ADVERTISING_DATA_LENGTH, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_ReadNumberOfSupportAdvSetCmd( void )
{
    uint8 rtnParam[2];
    rtnParam[0] = LL_ReadNumberOfSupportAdvSet( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_RemoveAdvSetCmd( uint8 adv_handle)
{
    hciStatus_t status;
    status = LL_RemoveAdvSet(adv_handle);
    HCI_CommandCompleteEvent( HCI_LE_REMOVE_ADVERTISING_SET, sizeof(status), &status );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_ClearAdvSetsCmd( void)
{
    hciStatus_t status;
    status = LL_ClearAdvSets();
    HCI_CommandCompleteEvent( HCI_LE_CLEAR_ADVERTISING_SETS, sizeof(status), &status );
    return( HCI_SUCCESS );
}


hciStatus_t HCI_LE_SetExtendedScanParametersCmd(uint8 own_address_type,
                                                uint8 scanning_filter_policy,
                                                uint8 scanning_PHYs,
                                                uint8* scan_sype,
                                                uint16* scan_interval,
                                                uint16* scan_window)
{
    hciStatus_t status;
    status = LL_SetExtendedScanParameters(own_address_type, scanning_filter_policy,
                                          scanning_PHYs, scan_sype, scan_interval, scan_window);
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDED_SCAN_PARAMETERS, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtendedScanEnableCmd(uint8 enable,
                                            uint8 filter_duplicates,
                                            uint16 duration,
                                            uint16 period)
{
    hciStatus_t status;
    status = LL_SetExtendedScanEnable( enable, filter_duplicates, duration, period);
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDED_SCAN_ENABLE, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_ExtendedCreateConnectionCmd(uint8 initiator_filter_policy,
                                               uint8 own_address_type,
                                               uint8 peer_address_type,
                                               uint8* peer_address,
                                               uint8  initiating_PHYs,
                                               uint16* scan_interval,
                                               uint16* scan_window,
                                               uint16* conn_interval_min,
                                               uint16* conn_interval_max,
                                               uint16* conn_latency,
                                               uint16* supervision_timeout,
                                               uint16* minimum_CE_length,
                                               uint16* maximum_CE_length)
{
    hciStatus_t status;
    status = LL_ExtendedCreateConnection(initiator_filter_policy,
                                         own_address_type,
                                         peer_address_type,
                                         peer_address,
                                         initiating_PHYs,
                                         scan_interval,
                                         scan_window,
                                         conn_interval_min,
                                         conn_interval_max,
                                         conn_latency,
                                         supervision_timeout,
                                         minimum_CE_length,
                                         maximum_CE_length);
    HCI_CommandCompleteEvent( HCI_LE_EXTENDED_CREATE_CONNECTION, sizeof(status), &status );
    return( HCI_SUCCESS );
}


// =============== periodic adv
hciStatus_t HCI_LE_SetPeriodicAdvParameterCmd( uint8 adv_handle,
                                               uint16   interval_min,
                                               uint16   interval_max,
                                               uint16   adv_event_properties
                                             )
{
    hciStatus_t status;
    status = LL_SetPeriodicAdvParameter(adv_handle,
                                        interval_min,
                                        interval_max,
                                        adv_event_properties);
    HCI_CommandCompleteEvent( HCI_LE_SET_PERIODIC_ADVERTISING_PARAMETERS, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetPeriodicAdvDataCmd( uint8 adv_handle,
                                          uint8 operation,
                                          uint8  advertising_data_length,
                                          uint8* advertising_data
                                        )
{
    hciStatus_t status;
    status = LL_SetPeriodicAdvData(adv_handle,
                                   operation,
                                   advertising_data_length,
                                   advertising_data
                                  );
    HCI_CommandCompleteEvent( HCI_LE_SET_PERIODIC_ADVERTISING_DATA, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetPeriodicAdvEnableCmd( uint8  enable,
                                            uint8  advertising_handle)
{
    hciStatus_t status;
    status = LL_SetPeriodicAdvEnable(enable, advertising_handle);
    HCI_CommandCompleteEvent( HCI_LE_Set_PERIODIC_ADVERTISING_ENABLE, sizeof(status), &status );
    return( HCI_SUCCESS );
}

// =============

hciStatus_t HCI_LE_PeriodicAdvertisingCreateSyncCmd(uint8 Options,
                                                    uint8 Advertising_SID,
                                                    uint8 Advertiser_Address_Type,
                                                    uint8* Advertiser_Address,
                                                    uint16 Skip,
                                                    uint16 Sync_Timeout,
                                                    uint8 Sync_CTE_Type)
{
    hciStatus_t status;
    status = LL_PeriodicAdvertisingCreateSync( Options, Advertising_SID,
                                               Advertiser_Address_Type, Advertiser_Address, Skip, Sync_Timeout, Sync_CTE_Type);
    HCI_CommandCompleteEvent( HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC, sizeof(status), &status );
    return( HCI_SUCCESS );
}



hciStatus_t HCI_LE_PeriodicAdvertisingCreateSyncCancelCmd(void)
{
    hciStatus_t status;
    status = LL_PeriodicAdvertisingCreateSyncCancel();
    HCI_CommandCompleteEvent( HCI_LE_PERIODIC_ADVERTISING_CREATE_SYNC_CANCEL, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_PeriodicAdvertisingTerminateSyncCmd(            uint16 sync_handle)
{
    hciStatus_t status;
    status = LL_PeriodicAdvertisingTerminateSync(sync_handle);
    HCI_CommandCompleteEvent( HCI_LE_PERIODIC_ADVERTISING_TERMINATE_SYNC, sizeof(status), &status );
    return( HCI_SUCCESS );
}

/*******************************************************************************
    This LE API is used to add a periodic advertiser list entry.


    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_AddDevToPeriodicAdvListCmd( uint8  addrType,
                                               uint8* devAddr,
                                               uint8 sid)
{
    hciStatus_t status;
    status = LL_AddDevToPeriodicAdvList(addrType,
                                        devAddr,
                                        sid);
    HCI_CommandCompleteEvent( HCI_LE_ADD_DEVICE_TO_PERIODIC_ADVERTISER_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to remove a periodic advertiser list entry.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_RemovePeriodicAdvListCmd( uint8  addrType,
                                             uint8* devAddr,
                                             uint8 sid)
{
    hciStatus_t status;
    status = LL_RemovePeriodicAdvListDevice(addrType, devAddr, sid);
    HCI_CommandCompleteEvent( HCI_LE_REMOVE_DEVICE_FROM_PERIODIC_ADVERTISER_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}

/*******************************************************************************
    This LE API is used to clear the periodic advertiser list.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ClearPeriodicAdvListCmd( void )
{
    hciStatus_t status;
    status = LL_ClearPeriodicAdvList();
    HCI_CommandCompleteEvent( HCI_LE_CLEAR_PERIODIC_ADVERTISER_LIST, sizeof(status), &status );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This LE API is used to read the total number of periodic advertiser list entries that can
    be stored in the Controller.

    Public function defined in hci.h.
*/
hciStatus_t HCI_LE_ReadPeriodicAdvListSizeCmd( void )
{
    uint8 rtnParam[2];
    rtnParam[0] = LL_ReadPeriodicAdvListSize( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_PERIODIC_ADVERTISER_LIST_SIZE, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

// ============
hciStatus_t HCI_LE_ConnectionlessCTE_TransmitParamCmd(               uint8 advertising_handle,
                                                                     uint8 len,
                                                                     uint8 type,
                                                                     uint8 count,
                                                                     uint8 Pattern_LEN,
                                                                     uint8* AnaIDs)

{
//  TODO:
    hciStatus_t status;
    status = LL_ConnectionlessCTE_TransmitParam(advertising_handle,len,type,count,Pattern_LEN,AnaIDs);
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_SET_CONNLESS_CTE_TRANS_PARAMETER, sizeof(status), &status );
    return HCI_SUCCESS;
}

hciStatus_t HCI_LE_ConnectionlessCTE_TransmitEnableCmd(              uint8 advertising_handle,
                                                                     uint8 enable)
{
    //  TODO:
    hciStatus_t status;
    status = LL_ConnectionlessCTE_TransmitEnable( advertising_handle,enable);
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_SET_CONNLESS_CTE_TRANS_ENABLE, sizeof(status), &status );
    return HCI_SUCCESS;
}

hciStatus_t HCI_LE_ConnectionlessIQ_SampleEnableCmd(             uint16 sync_handle,
                                                                 uint8 enable,
                                                                 uint8 slot_Duration,
                                                                 uint8 MaxSampledCTEs,
                                                                 uint8 pattern_len,
                                                                 uint8* AnaIDs)
{
    //  TODO:
    hciStatus_t rtnParam[3];
    // rtnParam[0] : return status
    rtnParam[0] = LL_ConnectionlessIQ_SampleEnable( sync_handle,enable,slot_Duration,MaxSampledCTEs,\
                                                    pattern_len,AnaIDs);
    rtnParam[1] = LO_UINT16( sync_handle );
    rtnParam[2] = HI_UINT16( sync_handle );
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_SET_CONNLESS_IQ_SAMPLE_ENABLE, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}


hciStatus_t HCI_LE_Set_ConnectionCTE_ReceiveParamCmd(                uint16 connHandle,
                                                                     uint8 enable,
                                                                     uint8 slot_Duration,
                                                                     uint8 pattern_len,
                                                                     uint8* AnaIDs)
{
    //  TODO:
    hciStatus_t rtnParam[3];
    // rtnParam[0] : return status
    rtnParam[0] = LL_Set_ConnectionCTE_ReceiveParam( connHandle, enable, slot_Duration,pattern_len,AnaIDs);
    rtnParam[1] = LO_UINT16( connHandle );
    rtnParam[2] = HI_UINT16( connHandle );
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_SET_CONNCTE_RECV_PARAMETER, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}



hciStatus_t HCI_LE_Set_ConnectionCTE_TransmitParamCmd(               uint16 connHandle,
                                                                     uint8 type,
                                                                     uint8 pattern_len,
                                                                     uint8* AnaIDs)
{
    //  TODO:
    hciStatus_t rtnParam[3];
    // rtnParam[0] : return status
    rtnParam[0] = LL_Set_ConnectionCTE_TransmitParam(connHandle,type,pattern_len,AnaIDs);
    rtnParam[1] = LO_UINT16( connHandle );
    rtnParam[2] = HI_UINT16( connHandle );
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_SET_CONN_CTE_TRANSMIT_PARAMETER, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}




hciStatus_t HCI_LE_Connection_CTE_Request_EnableCmd(               uint16 connHandle,
                                                                   uint8 enable,
                                                                   uint16 Interval,
                                                                   uint8 len,
                                                                   uint8 type)
{
    //  TODO:
    hciStatus_t rtnParam[3];
    // rtnParam[0] : return status
    rtnParam[0] = LL_Connection_CTE_Request_Enable(connHandle,enable,Interval,len,type);
    rtnParam[1] = LO_UINT16( connHandle );
    rtnParam[2] = HI_UINT16( connHandle );
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_CONN_CTE_REQUEST_ENABLE, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}



hciStatus_t HCI_LE_Connection_CTE_Response_EnableCmd(                uint16 connHandle,
                                                                     uint8 enable)
{
    //  TODO:
    hciStatus_t rtnParam[3];
    // rtnParam[0] : return status
    rtnParam[0] = LL_Connection_CTE_Response_Enable( connHandle,enable);
    rtnParam[1] = LO_UINT16( connHandle );
    rtnParam[2] = HI_UINT16( connHandle );
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_CONN_CTE_RESPONSE_ENABLE, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}


hciStatus_t HCI_LE_READ_Anatenna_InfoCmd(void)
{
    //  TODO:
    hciStatus_t rtnParam[5];
    rtnParam[0] = LL_READ_Anatenna_Info( &rtnParam[1] );
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_READ_ANTENNA_INFO, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}

/* power configure */
hciStatus_t HCI_LE_Read_Transmit_PowerCmd(void)
{
    hciStatus_t rtnParam[3];
    rtnParam[0] = LL_Read_Transmit_Power( &rtnParam[1] );
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_READ_TRANSMIT_POWER, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}

hciStatus_t HCI_LE_Read_Rf_Path_CompensationCmd(void)
{
    hciStatus_t rtnParam[5];
    rtnParam[0] = LL_Read_Rf_Path_Compensation(&rtnParam[1]);
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_READ_RF_PATH_COMPENSATION, sizeof(rtnParam), rtnParam);
    return HCI_SUCCESS;
}

hciStatus_t HCI_LE_Write_Rf_Path_CompensationCmd(int16 tx_compensation, int16 rx_compensation)
{
    hciStatus_t rtnParam[1];
    rtnParam[0] = LL_Write_Rf_Path_Compensation(tx_compensation, rx_compensation);
    // should check event mask bit
    HCI_CommandCompleteEvent( HCI_LE_WRITE_RF_PATH_COMPENSATION, sizeof(rtnParam), rtnParam );
    return HCI_SUCCESS;
}


hciStatus_t HCI_LE_Set_Privacy_ModeCmd(uint8  peerIdType,
                                       uint8* peerIdAddr,
                                       uint8 privacyMode)
{
    hciStatus_t rtnParam[1];
    rtnParam[0] = LL_Set_Privacy_Mode(peerIdType, peerIdAddr, privacyMode);
    HCI_CommandCompleteEvent(HCI_LE_SET_PRIVACY_MODE, sizeof(rtnParam), rtnParam);
    return HCI_SUCCESS;
}

#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)
/*
** HCI Vendor Specific Comamnds: Link Layer Extensions
*/

/*******************************************************************************
    This HCI Extension API is used to set the receiver gain.

    Note: If the LL can not perform the command immediately, the HCI will be
         notified by a corresonding LL callback.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetRxGainCmd( uint8 rxGain )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    uint8 cmdComplete = TRUE;
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_RX_GAIN_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_RX_GAIN_EVENT );
    rtnParam[2] = LL_EXT_SetRxGain( rxGain, &cmdComplete );

    // check if the command was performed, or if it was delayed
    // Note: If delayed, a callback will be generated by the LL.
    if ( cmdComplete == TRUE )
    {
        HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_RX_GAIN, sizeof(rtnParam), rtnParam );
    }

    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set the transmit power.

    Note: If the LL can not perform the command immediately, the HCI will be
         notified by a corresonding LL callback.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetTxPowerCmd( uint8 txPower )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    uint8 cmdComplete = TRUE;
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_TX_POWER_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_TX_POWER_EVENT );
    rtnParam[2] = LL_EXT_SetTxPower( txPower, &cmdComplete );

    // check if the command was performed, or if it was delayed
    // Note: If delayed, a callback will be generated by the LL.
    if ( cmdComplete == TRUE )
    {
        HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_TX_POWER, sizeof(rtnParam), rtnParam );
    }

    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set whether a connection will be limited
    to one packet per event.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_OnePktPerEvtCmd( uint8 control )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_ONE_PKT_PER_EVT_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_ONE_PKT_PER_EVT_EVENT );
    rtnParam[2] = LL_EXT_OnePacketPerEvent( control );

    // check if LL indicates the internal state of this feature is not being
    // changed by this command
    // Note: This is an internal status that only exists between the LL and HCI.
    //       It is being used here to basically suppress unnecessary events,
    //       allowing the application to repeatedly call this API without
    //       resulting in excessive events flooding the system.
    if ( rtnParam[2] != HCI_STATUS_WARNING_FLAG_UNCHANGED )
    {
        // the internal state of this feature has changed, so return event
        HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_ONE_PKT_PER_EVT, sizeof(rtnParam), rtnParam );
    }

    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set whether the system clock will be
    divided when the MCU is halted.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_ClkDivOnHaltCmd( uint8 control )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_CLK_DIVIDE_ON_HALT_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_CLK_DIVIDE_ON_HALT_EVENT );
    rtnParam[2] = LL_EXT_ClkDivOnHalt( control );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_CLK_DIVIDE_ON_HALT, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to indicate to the Controller whether or not
    the Host will be using the NV memory during BLE operations.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_DeclareNvUsageCmd( uint8 mode )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_DECLARE_NV_USAGE_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_DECLARE_NV_USAGE_EVENT );
    rtnParam[2] = LL_EXT_DeclareNvUsage( mode );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_DECLARE_NV_USAGE, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to decrypt encrypted data using AES128.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_DecryptCmd( uint8* key,
                                uint8* encText )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    // 3..18: Plain Text Data
    uint8 rtnParam[19];
    rtnParam[0] = LO_UINT16( HCI_EXT_DECRYPT_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_DECRYPT_EVENT );
    //
    rtnParam[2] = LL_EXT_Decrypt( key,
                                  encText,
                                  &rtnParam[3] );

    // check if okay
    if ( rtnParam[2] == LL_STATUS_SUCCESS )
    {
        // decryption process reverses the bytes, so restore to MSB..LSB
        HCI_ReverseBytes( &rtnParam[3], KEYLEN );
        HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_DECRYPT, sizeof(rtnParam), rtnParam );
    }
    else // bad parameters
    {
        rtnParam[2] = HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS;
        HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_DECRYPT, sizeof(uint8)+2, rtnParam );
    }

    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to write this devie's supported features.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetLocalSupportedFeaturesCmd( uint8* localFeatures )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES_EVENT );
    rtnParam[2] = LL_EXT_SetLocalSupportedFeatures( localFeatures );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set whether transmit data is sent as soon
    as possible even when slave latency is used.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetFastTxResponseTimeCmd( uint8 control )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_FAST_TX_RESP_TIME_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_FAST_TX_RESP_TIME_EVENT );
    rtnParam[2] = LL_EXT_SetFastTxResponseTime( control );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_FAST_TX_RESP_TIME, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}



/*******************************************************************************
    This HCI Extension API is used to enable or disable suspending slave
    latency.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetSlaveLatencyOverrideCmd( uint8 control )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_OVERRIDE_SL_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_OVERRIDE_SL_EVENT );
    rtnParam[2] = LL_EXT_SetSlaveLatencyOverride( control );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_OVERRIDE_SL, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This API is used start a continuous transmitter modem test, using either
    a modulated or unmodulated carrier wave tone, at the frequency that
    corresponds to the specified RF channel. Use HCI_EXT_EndModemTest command
    to end the test.

    Note: A Controller reset will be issued by the HCI_EXT_EndModemTest command!
    Note: The BLE device will transmit at maximum power.
    Note: This API can be used to verify this device meets Japan's TELEC
         regulations.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_ModemTestTxCmd( uint8 cwMode,
                                    uint8 txFreq )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_MODEM_TEST_TX_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_MODEM_TEST_TX_EVENT );
    rtnParam[2] = LL_EXT_ModemTestTx( cwMode, txFreq );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_MODEM_TEST_TX, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This API is used to start a continuous transmitter direct test mode test
    using a modulated carrier wave and transmitting a 37 byte packet of
    Pseudo-Random 9-bit data. A packet is transmitted on a different frequency
    (linearly stepping through all RF channels 0..39) every 625us. Use
    HCI_EXT_EndModemTest to end the test.

    Note: A Controller reset will be issued by the HCI_EXT_EndModemTest command!
    Note: The BLE device will transmit at maximum power.
    Note: This API can be used to verify this device meets Japan's TELEC
         regulations.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_ModemHopTestTxCmd( void )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_MODEM_HOP_TEST_TX_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_MODEM_HOP_TEST_TX_EVENT );
    rtnParam[2] = LL_EXT_ModemHopTestTx();
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_MODEM_HOP_TEST_TX, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This API is used to start a continuous receiver modem test using a modulated
    carrier wave tone, at the frequency that corresponds to the specific RF
    channel. Any received data is discarded. Receiver gain may be adjusted using
    the HCI_EXT_SetRxGain command. RSSI may be read during this test by using the
    HCI_ReadRssi command. Use the HCI_EXT_EndModemTest command to end the test.

    Note: A Controller reset will be issued by LL_EXT_EndModemTest!
    Note: The BLE device will transmit at maximum power.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_ModemTestRxCmd( uint8 rxFreq )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_MODEM_TEST_RX_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_MODEM_TEST_RX_EVENT );
    rtnParam[2] = LL_EXT_ModemTestRx( rxFreq );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_MODEM_TEST_RX, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This API is used to shutdown a modem test. A complete Controller reset will
    take place.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_EndModemTestCmd( void )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_END_MODEM_TEST_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_END_MODEM_TEST_EVENT );
    rtnParam[2] = LL_EXT_EndModemTest();
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_END_MODEM_TEST, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This API is used to set this device's BLE address (BDADDR).

    Note: This command is only allowed when the device's state is Standby.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetBDADDRCmd( uint8* bdAddr )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_BDADDR_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_BDADDR_EVENT );
    rtnParam[2] = LL_EXT_SetBDADDR( bdAddr );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_BDADDR, sizeof(rtnParam), rtnParam );

    // notify the HCI Test Application (if there is one) that BDADDR changed
    if ( hciTestTaskID )
    {
        (void)osal_set_event( hciTestTaskID, HCI_BDADDR_UPDATED_EVENT );
    }

    return( HCI_SUCCESS );
}


/*******************************************************************************
    This API is used to set this device's Sleep Clock Accuracy value.

    Note: For a slave device, this value is directly used, but only
         if power management is enabled. For a master device, this
         value is converted into one of eight ordinal values
         representing a SCA range, as specified in Table 2.2,
         Vol. 6, Part B, Section 2.3.3.1 of the Core specification.

    Note: This command is only allowed when the device is not in a connection.

    Note: The device's SCA value remains unaffected by a HCI_Reset.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetSCACmd( uint16 scaInPPM )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_SCA_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_SCA_EVENT );
    rtnParam[2] = LL_EXT_SetSCA( scaInPPM );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_SCA, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to enable Production Test Mode.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_EnablePTMCmd( void )
{
    // stop everything before entering PTM
    HCI_ResetCmd();
    // set global for runtime check
    hciPTMenabled = TRUE;
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set frequency tuning up or down.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetFreqTuneCmd( uint8 step )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_FREQ_TUNE_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_FREQ_TUNE_EVENT );
    rtnParam[2] = LL_EXT_SetFreqTune( step );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_FREQ_TUNE, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to save the frequency tuning value.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SaveFreqTuneCmd( void )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SAVE_FREQ_TUNE_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SAVE_FREQ_TUNE_EVENT );
    rtnParam[2] = LL_EXT_SaveFreqTune();
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SAVE_FREQ_TUNE, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set the max TX power for Direct Test Mode.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_SetMaxDtmTxPowerCmd( uint8 txPower )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_MAX_DTM_TX_POWER_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_MAX_DTM_TX_POWER_EVENT );
    rtnParam[2] = LL_EXT_SetMaxDtmTxPower( txPower );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_MAX_DTM_TX_POWER, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}



/*******************************************************************************
    This HCI Extension API is used to terminate a connection immediately without
    following normal BLE disconnect control procedure.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_DisconnectImmedCmd( uint16 connHandle )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_DISCONNECT_IMMED_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_DISCONNECT_IMMED_EVENT );
    rtnParam[2] = LL_EXT_DisconnectImmed( connHandle );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_DISCONNECT_IMMED, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to Reset or Read the Packet Error Rate data
    for a connection.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_PacketErrorRateCmd( uint16 connHandle, uint8 command )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    // 3: Command
    uint8 rtnParam[4];
    rtnParam[0] = LO_UINT16( HCI_EXT_PER_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_PER_EVENT );
    rtnParam[2] = LL_EXT_PacketErrorRate( connHandle, command );
    rtnParam[3] = command;

    // check if it is okay to complete this event now or later
    if ( (command == HCI_EXT_PER_RESET) || (rtnParam[2] != LL_STATUS_SUCCESS) )
    {
        HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_PER, sizeof(rtnParam), rtnParam );
    }

    return( HCI_SUCCESS );
}



/*******************************************************************************
    This HCI Extension API is used to start or end Packet Error Rate by Frequency
    counter accumulation for a connection.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_PERbyChanCmd( uint16 connHandle, perByChan_t* perByChan )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_PER_BY_CHAN_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_PER_BY_CHAN_EVENT );
    rtnParam[2] = LL_EXT_PERbyChan( connHandle, perByChan );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_PER_BY_CHAN, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to Extend Rf Range.

    Note: If the LL can not perform the command immediately, the HCI will be
         notified by a corresonding LL callback.

    Public function defined in hci.h.
*/
//hciStatus_t HCI_EXT_ExtendRfRangeCmd( void )
//{
//  // 0: Event Opcode (LSB)
//  // 1: Event Opcode (MSB)
//  // 2: Status
//  uint8 rtnParam[3];
//  uint8 cmdComplete = TRUE;

//  rtnParam[0] = LO_UINT16( HCI_EXT_EXTEND_RF_RANGE_EVENT );
//  rtnParam[1] = HI_UINT16( HCI_EXT_EXTEND_RF_RANGE_EVENT );
//  rtnParam[2] = LL_EXT_ExtendRfRange( &cmdComplete );

//  // check if the command was performed, or if it was delayed
//  // Note: If delayed, a callback will be generated by the LL.
//  if ( cmdComplete == TRUE )
//  {
//    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_EXTEND_RF_RANGE, sizeof(rtnParam), rtnParam );
//  }

//  return( HCI_SUCCESS );
//}


/*******************************************************************************
    This HCI Extension API is used to enable or disable HALT during RF.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_HaltDuringRfCmd( uint8 mode )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_HALT_DURING_RF_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_HALT_DURING_RF_EVENT );
    rtnParam[2] = LL_EXT_HaltDuringRf( mode );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_HALT_DURING_RF, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Adv event ends.
    A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_AdvEventNoticeCmd( uint8 taskID, uint16 taskEvent )
{
    return( LL_EXT_AdvEventNotice( taskID, taskEvent ) );
}


/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Connection event
    ends. A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_ConnEventNoticeCmd( uint8 taskID, uint16 taskEvent )
{
    return( LL_EXT_ConnEventNotice( taskID, taskEvent ) );
}


/*******************************************************************************
    This HCI Extension API is used to set a user revision number or read
    the build revision number (combined user/system build number).

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_BuildRevisionCmd( uint8 mode, uint16 userRevNum )
{
    // check input parameter that doesn't require vendor specific event
    if ( mode == HCI_EXT_SET_USER_REVISION )
    {
        // save the user's revision number
        // Note: No vendor specific event is generated as this is intended to be
        //       called from the target build. Otherwise, an asynchronous event
        //       would unexpectedly be received by the Host processor.
        return( LL_EXT_BuildRevision( mode, userRevNum, NULL ) );
    }
    else // vendor specific event required
    {
        // 0: Event Opcode (LSB)
        // 1: Event Opcode (MSB)
        // 2: Status
        // 3..6: Build Revision (combined user+system)
        uint8 rtnParam[7];
        rtnParam[0] = LO_UINT16( HCI_EXT_BUILD_REVISION_EVENT );
        rtnParam[1] = HI_UINT16( HCI_EXT_BUILD_REVISION_EVENT );
        rtnParam[2] = LL_EXT_BuildRevision( mode, userRevNum, &rtnParam[3] );

        // check for error
        if ( rtnParam[2] != LL_STATUS_SUCCESS )
        {
            // clear build revision
            *((uint32*)&rtnParam[3]) = 0;
        }

        // return event
        HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_BUILD_REVISION, sizeof(rtnParam), rtnParam );
    }

    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set the sleep delay.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_DelaySleepCmd( uint16 delay )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_DELAY_SLEEP_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_DELAY_SLEEP_EVENT );
    rtnParam[2] = LL_EXT_DelaySleep( delay );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_DELAY_SLEEP, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to issue a soft or hard system reset.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_ResetSystemCmd( uint8 mode )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_RESET_SYSTEM_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_RESET_SYSTEM_EVENT );
    rtnParam[2] = LL_EXT_ResetSystem( mode );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_RESET_SYSTEM, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to enable or disable overlapped processing.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_OverlappedProcessingCmd( uint8 mode )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_OVERLAPPED_PROCESSING_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_OVERLAPPED_PROCESSING_EVENT );
    rtnParam[2] = LL_EXT_OverlappedProcessing( mode );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_OVERLAPPED_PROCESSING, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}


/*******************************************************************************
    This HCI Extension API is used to set the minimum number of completed packets
    which must be met before a Number of Completed Packets event is returned. If
    the limit is not reach by the end of the connection event, then a Number of
    Completed Packets event will be returned (if non-zero) based on the
    flushOnEvt flag.

    Public function defined in hci.h.
*/
hciStatus_t HCI_EXT_NumComplPktsLimitCmd( uint8 limit,
                                          uint8 flushOnEvt )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_NUM_COMPLETED_PKTS_LIMIT_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_NUM_COMPLETED_PKTS_LIMIT_EVENT );
    rtnParam[2] = LL_EXT_NumComplPktsLimit( limit, flushOnEvt );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_NUM_COMPLETED_PKTS_LIMIT, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

#endif /*#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)*/

/*
** LL Callback Functions
*/

/*******************************************************************************
    This LL command Callback is used by the LL to notify the HCI that the LE
    RAND command has been completed.

    Note: The length is always given by B_RANDOM_NUM_SIZE.

    Public function defined in hci.h.
*/
void LL_RandCback( uint8* randData )
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

#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)
/*******************************************************************************
    This LL Extension command Callback is used by the LL to notify the HCI that
    the set RX gain command has been completed.

    Public function defined in hci.h.
*/
void LL_EXT_SetRxGainCback( void )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_RX_GAIN_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_RX_GAIN_EVENT );
    rtnParam[2] = HCI_SUCCESS;
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_RX_GAIN, sizeof(rtnParam), rtnParam );
    return;
}


/*******************************************************************************
    This LL Extension command Callback is used by the LL to notify the HCI that
    the set TX power command has been completed.

    Public function defined in hci.h.
*/
void LL_EXT_SetTxPowerCback( void )
{
    // 0: Event Opcode (LSB)
    // 1: Event Opcode (MSB)
    // 2: Status
    uint8 rtnParam[3];
    rtnParam[0] = LO_UINT16( HCI_EXT_SET_TX_POWER_EVENT );
    rtnParam[1] = HI_UINT16( HCI_EXT_SET_TX_POWER_EVENT );
    rtnParam[2] = HCI_SUCCESS;
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_SET_TX_POWER, sizeof(rtnParam), rtnParam );
    return;
}


/*******************************************************************************
    This LL Extension command Callback is used by the LL to notify the HCI that
    the Packet Error Rate Read has been completed.

    Note: The counters are only 16 bits. At the shortest connection
         interval, this provides a bit over 8 minutes of data.

    Public function defined in hci.h.
*/
void LL_EXT_PacketErrorRateCback( uint16 numPkts,
                                  uint16 numCrcErr,
                                  uint16 numEvents,
                                  uint16 numMissedEvts )
{
    // 0:  Event Opcode (LSB)
    // 1:  Event Opcode (MSB)
    // 2:  Status
    // 3:  Command
    // 4:  Number of Packets (LSB)
    // 5:  Number of Packets (MSB)
    // 6:  Number of CRC Errors (LSB)
    // 7:  Number of CRC Errors (MSB)
    // 8:  Number of Events (LSB)
    // 9:  Number of Events (MSB)
    // 10: Number of Missed Events(LSB)
    // 11: Number of Missed Events (MSB)
    uint8 rtnParam[12];
    rtnParam[0]  = LO_UINT16( HCI_EXT_PER_EVENT );
    rtnParam[1]  = HI_UINT16( HCI_EXT_PER_EVENT );
    rtnParam[2]  = HCI_SUCCESS;
    rtnParam[3]  = HCI_EXT_PER_READ;
    rtnParam[4]  = LO_UINT16( numPkts );
    rtnParam[5]  = HI_UINT16( numPkts );
    rtnParam[6]  = LO_UINT16( numCrcErr );
    rtnParam[7]  = HI_UINT16( numCrcErr );
    rtnParam[8]  = LO_UINT16( numEvents );
    rtnParam[9]  = HI_UINT16( numEvents );
    rtnParam[10] = LO_UINT16( numMissedEvts );
    rtnParam[11] = HI_UINT16( numMissedEvts );
    HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_PER, sizeof(rtnParam), rtnParam );
    return;
}

/*******************************************************************************
    This LL Extension command Callback is used by the LL to notify the HCI that
    the Extend Rf Range command has been completed.

    Public function defined in hci.h.
*/
//void LL_EXT_ExtendRfRangeCback( void )
//{
//  // 0: Event Opcode (LSB)
//  // 1: Event Opcode (MSB)
//  // 2: Status
//  uint8 rtnParam[3];

//  rtnParam[0] = LO_UINT16( HCI_EXT_EXTEND_RF_RANGE_EVENT );
//  rtnParam[1] = HI_UINT16( HCI_EXT_EXTEND_RF_RANGE_EVENT );
//  rtnParam[2] = HCI_SUCCESS;

//  HCI_VendorSpecifcCommandCompleteEvent( HCI_EXT_EXTEND_RF_RANGE, sizeof(rtnParam), rtnParam );

//  return;
//}

/////////////////////////////////  PhyPlus Defined HCI

/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Adv event ends.
    A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

*/
hciStatus_t HCI_PPLUS_AdvEventDoneNoticeCmd( uint8 taskID, uint16 taskEvent )
{
    // check that there is only one bit set
    if ( taskEvent & (taskEvent-1) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // save the user's task ID and task event
    g_adv_taskID = taskID;
    g_adv_taskEvent = taskEvent;
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Connection event
    ends. A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

*/
hciStatus_t HCI_PPLUS_ConnEventDoneNoticeCmd( uint8 taskID, uint16 taskEvent )
{
    llConnState_t* connPtr;
    // get current connection
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];

    if ( !connPtr->active )
    {
        return( LL_STATUS_ERROR_INACTIVE_CONNECTION );
    }

    // check that there is only one bit set
    if ( taskEvent & (taskEvent-1) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // save the user's task ID and task event
    g_conn_taskID = taskID;
    g_conn_taskEvent = taskEvent;
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Connection event
    ends. A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

*/
hciStatus_t HCI_PPLUS_DateLengthChangedNoticeCmd( uint8 taskID, uint16 taskEvent )
{
    llConnState_t* connPtr;
    // get current connection
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];

    if ( !connPtr->active )
    {
        return( LL_STATUS_ERROR_INACTIVE_CONNECTION );
    }

    // check that there is only one bit set
    if ( taskEvent & (taskEvent-1) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // save the user's task ID and task event
    g_dle_taskID = taskID;
    g_dle_taskEvent = taskEvent;
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    This HCI Extension API is used to enable or disable a notification to the
    specified task using the specified task event whenever a Connection event
    ends. A non-zero taskEvent value is taken to be "enable", while a zero valued
    taskEvent is taken to be "disable".

*/
hciStatus_t HCI_PPLUS_PhyUpdateNoticeCmd( uint8 taskID, uint16 taskEvent )
{
    llConnState_t* connPtr;
    // get current connection
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];

    if ( !connPtr->active )
    {
        return( LL_STATUS_ERROR_INACTIVE_CONNECTION );
    }

    // check that there is only one bit set
    if ( taskEvent & (taskEvent-1) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // save the user's task ID and task event
    g_phyChg_taskID = taskID;
    g_phyChg_taskEvent = taskEvent;
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    This HCI Extension API is used to enable or disable Tx/Rx packets limit
    per connection event to 8(default is 4).

*/
hciStatus_t HCI_PPLUS_ExtendTRXCmd( uint8 enable )
{
    if (enable)
    {
        pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 8;
        pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 8;
    }
    else
    {
        pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 4;
        pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 4;
    }

    return( HCI_SUCCESS );
}

#endif /*#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)*/
/***************************************************************************************************
*/
