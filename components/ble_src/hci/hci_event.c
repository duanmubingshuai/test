/*******************************************************************************
    Filename:       hci_c_event.c
    Revised:
    Revision:

    Description:    This file send HCI events for the controller. It implements
                  all the LL event callback and HCI events send.


*******************************************************************************/

/*******************************************************************************
    INCLUDES
*/
#include "hci_event.h"
#include "rf_phy_driver.h"

/*******************************************************************************
    MACROS
*/

/*******************************************************************************
    CONSTANTS
*/

// LE Event Lengths
#define HCI_CMD_COMPLETE_EVENT_LEN                  3
#define HCI_CMD_VS_COMPLETE_EVENT_LEN               2
#define HCI_CMD_STATUS_EVENT_LEN                    4
#define HCI_NUM_COMPLETED_PACKET_EVENT_LEN          5
#define HCI_FLUSH_OCCURRED_EVENT_LEN                2
#define HCI_REMOTE_VERSION_INFO_EVENT_LEN           8
#define HCI_CONNECTION_COMPLETE_EVENT_LEN           19
#define HCI_ENH_CONN_COMPLETE_EVENT_LEN             31
#define HCI_DISCONNECTION_COMPLETE_LEN              4
#define HCI_LL_CONN_UPDATE_COMPLETE_LEN             10
#define HCI_ADV_REPORT_EVENT_LEN                    12
#define HCI_READ_REMOTE_FEATURE_COMPLETE_EVENT_LEN  12
#define HCI_LTK_REQUESTED_EVENT_LEN                 13
#define HCI_DATA_BUF_OVERFLOW_EVENT_LEN             1
#define HCI_ENCRYPTION_CHANGE_EVENT_LEN             4
#define HCI_KEY_REFRESH_COMPLETE_EVENT_LEN          3
#define HCI_BUFFER_OVERFLOW_EVENT_LEN               1
#define HCI_LL_DATA_LENGTH_CHANGE_EVENT_LEN         11
#define HCI_LL_PHY_UPDATE_COMPLETE_EVENT_LEN        6

#define HCI_EXT_ADV_REPORT_EVENT_LEN                24   //18
#define HCI_PRD_ADV_SYNC_ESTAB_EVENT_LEN            16
#define HCI_PRD_ADV_REPORT_EVENT_LEN                8
#define HCI_PRD_ADV_SYNC_LOST_EVENT_LEN             2
#define HCI_ADV_SET_TERM_EVENT_LEN                  6
#define HCI_CHN_SEL_ALGO_EVENT_LEN                  4

// LE Event mask - last octet
#define LE_EVT_MASK_CONN_COMPLETE                   0x000001
#define LE_EVT_MASK_ADV_REPORT                      0x000002
#define LE_EVT_MASK_CONN_UPDATE_COMPLETE            0x000004
#define LE_EVT_MASK_READ_REMOTE_FEATURE             0x000008
#define LE_EVT_MASK_LTK_REQUEST                     0x000010

#define LE_EVT_MASK_DATA_LENGTH_CHANGE              0x000040

#define LE_EVT_MASK_PHY_CHANGE                      0x000800

// HCI Event mask - octet index
#define HCI_EVT_INDEX_DISCONN_COMPLETE              0
#define HCI_EVT_INDEX_ENCRYPTION_CHANGE             0
#define HCI_EVT_INDEX_READ_REMOTE_VER               1
#define HCI_EVT_INDEX_HW_ERROR                      1
#define HCI_EVT_INDEX_FLUSH_OCCURRED                2
#define HCI_EVT_INDEX_BUF_OVERFLOW                  3
#define HCI_EVT_INDEX_KEY_REFRESH_COMPLETE          5
#define HCI_EVT_INDEX_LE                            7

// HCI Event mask - octet mask
#define HCI_EVT_MASK_DISCONN_COMPLETE               0x10
#define HCI_EVT_MASK_ENCRYPTION_CHANGE              0x80
#define HCI_EVT_MASK_READ_REMOTE_VER                0x08
#define HCI_EVT_MASK_HW_ERROR                       0x80
#define HCI_EVT_MASK_FLUSH_OCCURRED                 0x01
#define HCI_EVT_MASK_BUF_OVERFLOW                   0x02
#define HCI_EVT_MASK_KEY_REFRESH_COMPLETE           0x80
#define HCI_EVT_MASK_LE                             0x20

// HCI Connection Complete Roles
#define HCI_EVT_MASTER_ROLE                         0x00
#define HCI_EVT_SLAVE_ROLE                          0x01

/*******************************************************************************
    TYPEDEFS
*/

/*******************************************************************************
    LOCAL VARIABLES
*/

//uint8 bleEvtMask;
uint32 bleEvtMask;          // extend le_meta Event masker to 32bit by ZQ 20181031
uint8 pHciEvtMask[B_EVENT_MASK_LEN];

/*******************************************************************************
    GLOBAL VARIABLES
*/
//#if defined(CTRL_CONFIG) && (CTRL_CONFIG & INIT_CFG)
void LL_ReadRemoteUsedFeaturesCompleteCback( hciStatus_t status,
                                             uint16      connHandle,
                                             uint8*       featureSet );
//#endif // CTRL_CONFIG=INIT_CFG

/*******************************************************************************
    EXTERNS
*/
extern uint8 hciPTMenabled;
extern uint8 hciCtrlCmdToken;

/*
** Internal Functions
*/

/*******************************************************************************
    @fn          hciInitEventMasks

    @brief       This routine initializes Bluetooth and BLE event makss to their
                default values.

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
*/
void hciInitEventMasks( void )
{
    // set default Bluetooth event mask bits
    pHciEvtMask[0] = BT_EVT_MASK_BYTE0;
    pHciEvtMask[1] = BT_EVT_MASK_BYTE1;
    pHciEvtMask[2] = BT_EVT_MASK_BYTE2;
    pHciEvtMask[3] = BT_EVT_MASK_BYTE3;
    pHciEvtMask[4] = BT_EVT_MASK_BYTE4;
    pHciEvtMask[5] = BT_EVT_MASK_BYTE5;
    pHciEvtMask[6] = BT_EVT_MASK_BYTE6;
    pHciEvtMask[7] = BT_EVT_MASK_BYTE7;
    // set default BLE event mask bits
    bleEvtMask = LE_EVT_MASK_DEFAULT;
    return;
}


/*
** HCI Events
*/

/*******************************************************************************
    This function sends the Data Buffer Overflow Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_DataBufferOverflowEvent( uint8 linkType )
{
    hciPacket_t* msg;
    uint8       totalLength;
    // OSAL message header + HCI event header + parameters
    totalLength = sizeof (hciPacket_t) +
                  HCI_EVENT_MIN_LENGTH +
                  HCI_BUFFER_OVERFLOW_EVENT_LEN;
    msg = (hciPacket_t*)osal_msg_allocate(totalLength);

    if (msg)
    {
        // create message header
        msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
        msg->hdr.status = 0xFF;
        // create event header
        msg->pData    = (uint8*)(msg+1);
        msg->pData[0] = HCI_EVENT_PACKET;
        msg->pData[1] = HCI_DATA_BUFFER_OVERFLOW_EVENT;
        msg->pData[2] = HCI_BUFFER_OVERFLOW_EVENT_LEN;
        // Link Type
        msg->pData[3] = linkType;
        // send message
        (void)osal_msg_send( hciTaskID, (uint8*)msg );
    }
}


/*******************************************************************************
    This function sends the Number of Completed Packets Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_NumOfCompletedPacketsEvent( uint8  numHandles,
                                     uint16* handles,
                                     uint16* numCompletedPkts )
{
    // check if this is for the Host
    if ( hciL2capTaskID != 0 )
    {
        // In the future, we need to change to an OSAL message
        // and send it to L2CAP or a fragmentation task.
        // For now, drop the event.
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;
        // the data length
        // TODO: CONFUSING. USE 1+(numHandles * HCI_NUM_COMPLETED_PACKET_EVENT_LEN-1)?
        dataLength = HCI_NUM_COMPLETED_PACKET_EVENT_LEN +
                     ((numHandles-1) * (HCI_NUM_COMPLETED_PACKET_EVENT_LEN-1));
        // OSAL message header + HCI event header + data
        totalLength = sizeof(hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        // allocate memory for OSAL hdr + packet
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if ( msg )
        {
            uint8 i;
            // OSAL header
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // build event packet
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;                        // packet type
            msg->pData[1] = HCI_NUM_OF_COMPLETED_PACKETS_EVENT_CODE; // event code
            msg->pData[2] = dataLength;
            msg->pData[3] = numHandles;

            // for each handle, there's a number handle number and a number of
            // completed packets for that handle
            for (i=0; i<numHandles; i++)
            {
                msg->pData[4+(4*i)] = LO_UINT16(handles[i]);
                msg->pData[5+(4*i)] = HI_UINT16(handles[i]);
                msg->pData[6+(4*i)] = LO_UINT16(numCompletedPkts[i]);
                msg->pData[7+(4*i)] = HI_UINT16(numCompletedPkts[i]);
            }

            // send message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    This function sends a Command Complete Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_CommandCompleteEvent( uint16 opcode,
                               uint8  numParam,
                               uint8*  param )
{
    // check if this is for the Host
    if ( ((hciGapTaskID != 0) || (hciSmpTaskID != 0)) && (hciPTMenabled == FALSE) )
    {
        hciEvt_CmdComplete_t* pkt =
            (hciEvt_CmdComplete_t*)osal_msg_allocate( sizeof(hciEvt_CmdComplete_t) +
                                                      numParam );

        if ( pkt )
        {
            uint8 taskID;

            if ( (opcode == HCI_LE_RAND || opcode == HCI_LE_ENCRYPT) && (hciSmpTaskID) )
            {
                taskID         = hciSmpTaskID;
                pkt->hdr.event = HCI_SMP_EVENT_EVENT;
            }
            else
            {
                taskID         = hciGapTaskID;
                pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            }

            pkt->hdr.status   = HCI_COMMAND_COMPLETE_EVENT_CODE;
            pkt->numHciCmdPkt = hciCtrlCmdToken;
            pkt->cmdOpcode    = opcode;
            pkt->pReturnParam = (uint8*)(pkt+1);
            (void)osal_memcpy( pkt->pReturnParam, param, numParam );
            (void)osal_msg_send( taskID, (uint8*)pkt );
        }
    }
    else
    {
        HCI_SendCommandCompleteEvent ( HCI_COMMAND_COMPLETE_EVENT_CODE,
                                       opcode,
                                       numParam,
                                       param );
    }
}


/*******************************************************************************
    This function sends a Vendor Specific Command Complete Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_VendorSpecifcCommandCompleteEvent ( uint16 opcode,
                                             uint8  numParam,
                                             uint8*  param )
{
    HCI_SendCommandCompleteEvent ( HCI_VE_EVENT_CODE,
                                   opcode,
                                   numParam,
                                   param );
}


/*******************************************************************************
    This function sends a Command Status Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_CommandStatusEvent( hciStatus_t status,
                             uint16      opcode )
{
    // check if this is for the Host
    if ( (hciGapTaskID != 0) && (hciPTMenabled == FALSE) )
    {
        hciEvt_CommandStatus_t* pMsg;
        uint8 totalLength;
        totalLength = sizeof(hciEvt_CommandStatus_t);
        pMsg = (hciEvt_CommandStatus_t*)osal_msg_allocate(totalLength);

        if ( pMsg )
        {
            // message type, HCI event type
            pMsg->hdr.event = HCI_GAP_EVENT_EVENT;
            // use the OSAL status field for HCI event code
            pMsg->hdr.status   = HCI_COMMAND_STATUS_EVENT_CODE;
            pMsg->cmdStatus    = status;
            pMsg->numHciCmdPkt = hciCtrlCmdToken;
            pMsg->cmdOpcode    = opcode;
            // send the message
            (void)osal_msg_send( hciGapTaskID, (uint8*)pMsg );
        }
    }
    else
    {
        HCI_SendCommandStatusEvent( HCI_COMMAND_STATUS_EVENT_CODE, status, opcode );
    }
}


/*******************************************************************************
    This function sends a Hardware Error Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_HardwareErrorEvent( uint8 hwErrorCode )
{
    hciPacket_t* msg;
    uint8 dataLength;
    uint8 totalLength;

    // check the event mask
    if( ( HCI_EVT_MASK_HW_ERROR&
            pHciEvtMask[HCI_EVT_INDEX_HW_ERROR]) == 0 )
    {
        /* Event mask is not set for this event, do not send to host */
        return;
    }

    // data length
    dataLength = 1;
    // OSAL message header + HCI event header + data
    totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
    msg = (hciPacket_t*)osal_msg_allocate(totalLength);

    if (msg)
    {
        // message type, length
        msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
        msg->hdr.status = 0xFF;
        // create message
        msg->pData    = (uint8*)(msg+1);
        msg->pData[0] = HCI_EVENT_PACKET;
        msg->pData[1] = HCI_BLE_HARDWARE_ERROR_EVENT_CODE;
        msg->pData[2] = dataLength;
        // error code
        msg->pData[3] = hwErrorCode;
        // send the message
        (void)osal_msg_send( hciTaskID, (uint8*)msg );
    }
}


/*******************************************************************************
    This generic function sends a Command Complete or a Vendor Specific Command
    Complete Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_SendCommandStatusEvent ( uint8  eventCode,
                                  uint16 status,
                                  uint16 opcode )
{
    uint8 data[4];
    data[0] = status;
    data[1] = hciCtrlCmdToken;   // number of HCI packets
    data[2] = LO_UINT16(opcode); // opcode (LSB)
    data[3] = HI_UINT16(opcode); // opcode (MSB)
    HCI_SendControllerToHostEvent( eventCode, 4, data );
}


/*******************************************************************************
    This generic function sends a Command Complete or a Vendor Specific Command
    Complete Event to the Host.

    Public function defined in hci_c_event.h.
*/
void HCI_SendCommandCompleteEvent ( uint8  eventCode,
                                    uint16 opcode,
                                    uint8  numParam,
                                    uint8*  param )
{
    hciPacket_t* msg;
    uint8        totalLength;
    // The initial length will be:
    // OSAL message header(4) - not part of packet sent to HCI Host!
    // Minimum Event Data: Packet Type(1) + Event Code(1) + Length(1)
    // Return Parameters (0..N)
    totalLength = sizeof(hciPacket_t) + HCI_EVENT_MIN_LENGTH + numParam;
    // adjust the size of the event packet based on event code
    // Note: If not a vendor specific event, then the event includes:
    //       Command Complete Data: Number of HCI Commands Allowed(1) + Command Opcode(2)
    // Note: If a vendor specific event, then the event includes:
    //       Vendor Specific Command Complete Data: Vendor Specific Event Opcode(2)
    totalLength += ( (eventCode != HCI_VE_EVENT_CODE)  ?
                     HCI_CMD_COMPLETE_EVENT_LEN        :
                     HCI_CMD_VS_COMPLETE_EVENT_LEN );
    // allocate memory for OSAL hdr + packet
    msg = (hciPacket_t*)osal_msg_allocate(totalLength);

    if ( msg )
    {
        // OSAL message event, status, and pointer to packet
        msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
        msg->hdr.status = 0xFF;
        msg->pData      = (uint8*)(msg+1);
        // fill in Command Complete Event data
        msg->pData[0] = HCI_EVENT_PACKET;
        msg->pData[1] = eventCode;

        // check if this isn't a vendor specific event
        if ( eventCode != HCI_VE_EVENT_CODE )
        {
            msg->pData[2] = numParam + HCI_CMD_COMPLETE_EVENT_LEN;
            msg->pData[3] = hciCtrlCmdToken;     // event parameter 1
            msg->pData[4] = LO_UINT16( opcode ); // event parameter 2
            msg->pData[5] = HI_UINT16( opcode ); // event parameter 2
            // remaining event parameters
            (void)osal_memcpy (&msg->pData[6], param, numParam);
        }
        else // it is a vendor specific event
        {
            // less one byte as number of complete packets not used in vendor specific event
            msg->pData[2] = numParam + HCI_CMD_VS_COMPLETE_EVENT_LEN;
            msg->pData[3] = param[0];            // event parameter 0: event opcode LSB
            msg->pData[4] = param[1];            // event parameter 1: event opcode MSB
            msg->pData[5] = param[2];            // event parameter 2: status
            msg->pData[6] = LO_UINT16( opcode ); // event parameter 3: command opcode LSB
            msg->pData[7] = HI_UINT16( opcode ); // event parameter 3: command opcode MSB
            // remaining event parameters
            // Note: The event opcode and status were already placed in the msg packet.
            (void)osal_memcpy (&msg->pData[8], &param[3], numParam-HCI_EVENT_MIN_LENGTH);
        }

        // send the message
        (void)osal_msg_send( hciTaskID, (uint8*)msg );
    }
}


/*******************************************************************************
    This is a generic function used to send events from the Controller to the
    Host.

    Public function defined in hci_c_event.h.
*/
void HCI_SendControllerToHostEvent( uint8 eventCode,
                                    uint8 dataLen,
                                    uint8* pData )
{
    hciPacket_t* msg;
    uint8 totalLength;
    // OSAL message header + HCI event header + data
    totalLength = sizeof(hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLen;
    // allocate memory for OSAL hdr + packet
    msg = (hciPacket_t*)osal_msg_allocate(totalLength);

    if ( msg )
    {
        // message type, HCI event type
        msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
        msg->hdr.status = 0xFF;
        // packet
        msg->pData    = (uint8*)(msg+1);
        msg->pData[0] = HCI_EVENT_PACKET;
        msg->pData[1] = eventCode;
        msg->pData[2] = dataLen;

        // copy data
        if ( dataLen )
        {
            (void)osal_memcpy( &(msg->pData[3]), pData, dataLen );
        }

        // send message
        (void)osal_msg_send( hciTaskID, (uint8*)msg );
    }
}


/*
** LL Callbacks for LE Meta-Events
*/

/*******************************************************************************
    @fn          LL_AdvReportCback Callback

    @brief       This LL callback is used to generate a Advertisment Report meta
                event when an Advertisment or Scan Response is received by a
                Scanner.

    input parameters

    @param       advEvt      - Advertise event type, or Scan Response event type.
    @param       advAddrType - Public or Random address type.
    @param       advAddr     - Pointer to device address.
    @param       dataLen     - Length of data in bytes.
    @param       advData     - Pointer to data.
    @param       rssi        - The RSSI of received packet.

    output parameters

    @param       None.

    @return      None.
*/
void LL_AdvReportCback( uint8 advEvt,
                        uint8 advAddrType,
                        uint8* advAddr,
                        uint8 dataLen,
                        uint8* advData,
                        int8  rssi )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEAdvPktReport_t* pkt;
        hciEvt_DevInfo_t* devInfo;
        uint8 x;

        if (dataLen > B_MAX_ADV_LEN)      // guard the memory access, 2018-10-15
        {
            return;
        }

        pkt = (hciEvt_BLEAdvPktReport_t*)osal_msg_allocate(
                  sizeof ( hciEvt_BLEAdvPktReport_t ) + sizeof ( hciEvt_DevInfo_t ) );

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_ADV_REPORT_EVENT;
            pkt->numDevices = 1;  // assume one device for now
            pkt->devInfo = devInfo = (hciEvt_DevInfo_t*)(pkt+1);

            for ( x = 0; x < pkt->numDevices; x++, devInfo++ )
            {
                /* Fill in the device info */
                devInfo->eventType = advEvt;
                devInfo->addrType = advAddrType;
                (void)osal_memcpy( devInfo->addr, advAddr, B_ADDR_LEN );
                devInfo->dataLen = dataLen;
                (void)osal_memcpy( devInfo->rspData, advData, dataLen );
                devInfo->rssi = rssi;
            }

            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_ADV_REPORT) == 0 )) )
        {
            // the event mask is not set for this event
            return;
        }

        // data length
        dataLength = HCI_ADV_REPORT_EVENT_LEN + dataLen;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_ADV_REPORT_EVENT;                // event code
            msg->pData[4] = 1;                                       // number of devices; assume 1 for now
            msg->pData[5] = advEvt;                                  // advertisement event type
            msg->pData[6] = advAddrType;                             // address type
            (void)osal_memcpy (&msg->pData[7], advAddr, B_ADDR_LEN); // address
            msg->pData[13] = dataLen;                                // data length
            (void)osal_memcpy (&msg->pData[14], advData, dataLen);   // data
            msg->pData[14 + dataLen] = rssi;                         // RSSI
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}
//#pragma O0
// TO update comments
/*******************************************************************************
    @fn          LL_ExtAdvReportCback

    @brief       This LL callback is used to generate a Advertisment Report meta
                event when an Advertisment or Scan Response is received by a
                Scanner.

    input parameters

    @param       advEvt      - Advertise event type, or Scan Response event type.
    @param       advAddrType - Public or Random address type.
    @param       advAddr     - Pointer to device address.
    @param       dataLen     - Length of data in bytes.
    @param       advData     - Pointer to data.
    @param       rssi        - The RSSI of received packet.

    output parameters

    @param       None.

    @return      None.
*/
void LL_ExtAdvReportCback( uint8 advEvt,
                           uint8   advAddrType,
                           uint8*  advAddr,
                           uint8   primaryPHY,
                           uint8   secondaryPHY,
                           uint8   advertisingSID,
                           uint8   txPower,
                           int8    rssi,
                           uint16  periodicAdvertisingInterval,
                           uint8   directAddrType,
                           uint8*   directAddr,
                           uint8   dataLen,
                           uint8*   rptData)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEExtAdvPktReport_t* pkt;
        hciEvt_ExtAdvRptInfo_t* rptInfo;
        uint8 x;
//    if (dataLen > B_MAX_ADV_LEN)      // guard the memory access, 2018-10-15
//    {
//        return;
//    }
        pkt = (hciEvt_BLEExtAdvPktReport_t*)osal_msg_allocate(
                  sizeof ( hciEvt_BLEExtAdvPktReport_t ) + sizeof ( hciEvt_ExtAdvRptInfo_t ) );

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_EXT_ADV_REPORT_EVENT;
            pkt->numReports = 1;  // assume one report each event now
            pkt->rptInfo = rptInfo = (hciEvt_ExtAdvRptInfo_t*)(pkt+1);

            for ( x = 0; x < pkt->numReports; x++, rptInfo++ )
            {
                /* Fill in the device info */
                rptInfo->eventType = advEvt;
                rptInfo->addrType = advAddrType;
                (void)osal_memcpy( rptInfo->addr, advAddr, B_ADDR_LEN );
                rptInfo->primaryPHY = primaryPHY;

                if (secondaryPHY == PKT_FMT_BLR125K)
                    rptInfo->secondaryPHY = 0x03;           //  convert 4 -> 3
                else
                    rptInfo->secondaryPHY = secondaryPHY;

                rptInfo->advertisingSID = advertisingSID;
                rptInfo->txPower = txPower;
                rptInfo->rssi   = rssi;
                rptInfo->periodicAdvertisingInterval = periodicAdvertisingInterval;
                rptInfo->directAddrType = directAddrType;

                if (advEvt & LE_ADV_PROP_DIRECT_BITMASK)
                {
                    (void)osal_memcpy( rptInfo->directAddr, directAddr, B_ADDR_LEN );
                }

                rptInfo->dataLen = dataLen;
                (void)osal_memcpy( rptInfo->rptData, rptData, dataLen );
                rptInfo->rssi = rssi;
            }

            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_ADV_REPORT) == 0 )) )
        {
            // the event mask is not set for this event
            return;
        }

        // data length
        dataLength = HCI_EXT_ADV_REPORT_EVENT_LEN + dataLen;

        if (advEvt & LE_ADV_PROP_DIRECT_BITMASK)
            dataLength += 6;

        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_EXT_ADV_REPORT_EVENT;                // event code
            msg->pData[4] = 1;                                       // number of reports; assume 1 for now
            msg->pData[5] = advEvt;                                  // advertisement event type
            msg->pData[6] = advAddrType;                             // address type
            (void)osal_memcpy (&msg->pData[7], advAddr, B_ADDR_LEN); // address
            msg->pData[12] = primaryPHY;
            msg->pData[13] = secondaryPHY;
            msg->pData[14] = advertisingSID;
            msg->pData[15] = txPower;
            msg->pData[16] = rssi;
            msg->pData[17]  = LO_UINT16(periodicAdvertisingInterval);
            msg->pData[18]  = HI_UINT16(periodicAdvertisingInterval);
            msg->pData[19] = directAddrType;                             // address type

            if (advEvt & LE_ADV_PROP_DIRECT_BITMASK)
                (void)osal_memcpy (&msg->pData[20], directAddr, B_ADDR_LEN); // address

            msg->pData[26] = dataLen;                                // data length
            (void)osal_memcpy (&msg->pData[27], rptData, dataLen);   // data
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

/*******************************************************************************
    @fn          LL_AdvSetTerminatedCback

    @brief       This LL callback is used to .....



    input parameters

    @param
    @param
    @param
    @param
    @param
    @param

    output parameters

    @param       None.

    @return      None.
*/
void LL_AdvSetTerminatedCback(uint8          status,
                              uint8   adv_handle,
                              uint16  connHandle,
                              uint8   Num_Completed_Extended_Advertising_Events)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_AdvSetTerminated_t* pkt;
        pkt = (hciEvt_AdvSetTerminated_t*)osal_msg_allocate(sizeof(hciEvt_AdvSetTerminated_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_LE_ADVERTISING_SET_TERMINATED;
            pkt->status      = status;
            pkt->adv_handle  = adv_handle;
            pkt->connHandle  = connHandle;
            pkt->Num_Completed_Extended_Advertising_Events = Num_Completed_Extended_Advertising_Events;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;
//    // check if LE Meta-Events are enabled and this event is enabled
//    if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
//        (((bleEvtMask & LE_EVT_MASK_ADV_REPORT) == 0 )) )
//    {
//      // the event mask is not set for this event
//      return;
//    }
        // data length
        dataLength = HCI_ADV_SET_TERM_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT;     // event code
            msg->pData[4] = status;
            msg->pData[5] = adv_handle;
            msg->pData[6] = LO_UINT16(connHandle);
            msg->pData[7] = HI_UINT16(connHandle);
            msg->pData[8] = Num_Completed_Extended_Advertising_Events;
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_ChannelSelectionAlgorithmCback Callback

    @brief       This LL callback is used to .....



    input parameters

    @param
    @param
    @param
    @param
    @param
    @param

    output parameters

    @param       None.

    @return      None.
*/
void LL_ChannelSelectionAlgorithmCback(uint16 connHandle,
                                       uint8  chnSel
                                      )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_ChannelSelAlgo_t* pkt;
        pkt = (hciEvt_ChannelSelAlgo_t*)osal_msg_allocate(sizeof(hciEvt_ChannelSelAlgo_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_LE_CHANNEL_SELECTION_ALGORITHM_EVENT;
            pkt->connHandle  = connHandle;
            pkt->chn_sel     = chnSel;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;
//    // check if LE Meta-Events are enabled and this event is enabled
//    if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
//        (((bleEvtMask & LE_EVT_MASK_ADV_REPORT) == 0 )) )
//    {
//      // the event mask is not set for this event
//      return;
//    }
        // data length
        dataLength = HCI_CHN_SEL_ALGO_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT;     // event code
            msg->pData[4] = LO_UINT16(connHandle);
            msg->pData[5] = HI_UINT16(connHandle);
            msg->pData[6] = chnSel;
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_PrdAdvSyncEstablishedCback Callback

    @brief       This LL callback is used to generate a Advertisment Report meta
                event when an Advertisment or Scan Response is received by a
                Scanner.

    input parameters

    @param       advEvt      - Advertise event type, or Scan Response event type.
    @param       advAddrType - Public or Random address type.
    @param       advAddr     - Pointer to device address.
    @param       dataLen     - Length of data in bytes.
    @param       advData     - Pointer to data.
    @param       rssi        - The RSSI of received packet.

    output parameters

    @param       None.

    @return      None.
*/
void LL_PrdAdvSyncEstablishedCback(uint8        status,
                                   uint16  syncHandle,
                                   uint8   advertisingSID,
                                   uint8   advertiserAddressType,
                                   uint8*   advertiserAddress,
                                   uint8   advertiserPHY,
                                   uint16  periodicAdvertisingInterval,
                                   uint8   advertiserClockAccuracy
                                  )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEPrdAdvSyncEstabPkt_t* pkt;
        pkt = (hciEvt_BLEPrdAdvSyncEstabPkt_t*)osal_msg_allocate(sizeof(hciEvt_BLEPrdAdvSyncEstabPkt_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT;
            pkt->status          = status;
            pkt->syncHandle      = syncHandle;
            pkt->advertisingSID  =  advertisingSID;
            pkt->advertiserAddressType = advertiserAddressType;
            (void)osal_memcpy (&pkt->advertiserAddress[0], advertiserAddress, B_ADDR_LEN); // address
            pkt->advertiserPHY   = advertiserPHY;
            pkt->periodicAdvertisingInterval = periodicAdvertisingInterval;
            pkt->advertiserClockAccuracy = advertiserClockAccuracy;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;
//    // check if LE Meta-Events are enabled and this event is enabled
//    if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
//        (((bleEvtMask & LE_EVT_MASK_ADV_REPORT) == 0 )) )
//    {
//      // the event mask is not set for this event
//      return;
//    }
        // data length
        dataLength = HCI_PRD_ADV_SYNC_ESTAB_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_PERIODIC_ADV_SYNC_ESTABLISHED_EVENT;     // event code
            msg->pData[4] = status;
            msg->pData[5] = LO_UINT16(syncHandle);
            msg->pData[6] = HI_UINT16(syncHandle);
            msg->pData[7] = advertisingSID;
            msg->pData[8] = advertiserAddressType;
            (void)osal_memcpy (&msg->pData[9], advertiserAddress, B_ADDR_LEN); // address
            msg->pData[15] = advertiserPHY;
            msg->pData[16] = LO_UINT16(periodicAdvertisingInterval);
            msg->pData[17] = HI_UINT16(periodicAdvertisingInterval);
            msg->pData[18] = advertiserClockAccuracy;
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_PrdAdvReportCback Callback

    @brief       This LL callback is used to generate a Advertisment Report meta
                event when an Advertisment or Scan Response is received by a
                Scanner.

    input parameters

    @param       advEvt      - Advertise event type, or Scan Response event type.
    @param       advAddrType - Public or Random address type.
    @param       advAddr     - Pointer to device address.
    @param       dataLen     - Length of data in bytes.
    @param       advData     - Pointer to data.
    @param       rssi        - The RSSI of received packet.

    Subevent_Code,
    output parameters

    @param       None.

    @return      None.
*/
void LL_PrdAdvReportCback(uint16 syncHandle,
                          uint8 txPower,
                          uint8 rssi,
                          uint8 cteType,
                          uint8 dataStatus,
                          uint8 dataLength,
                          uint8* data
                         )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEPrdAdvPktReport_t* pkt;
        hciEvt_PrdAdvRptInfo_t* rptInfo;
//    if (dataLen > B_MAX_ADV_LEN)      // guard the memory access, 2018-10-15
//    {
//        return;
//    }
        pkt = (hciEvt_BLEPrdAdvPktReport_t*)osal_msg_allocate(
                  sizeof ( hciEvt_BLEPrdAdvPktReport_t ) + sizeof ( hciEvt_PrdAdvRptInfo_t ) );

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_PERIODIC_ADV_REPORT_EVENT;
            pkt->rptInfo = rptInfo = (hciEvt_PrdAdvRptInfo_t*)(pkt+1);
            /* Fill in the device info */
            rptInfo->syncHandle = syncHandle;
            rptInfo->txPower = txPower;
            rptInfo->cteType = cteType;
            rptInfo->rssi   = rssi;
            rptInfo->dataStatus = dataStatus;
            rptInfo->dataLength = dataLength;
            (void)osal_memcpy( rptInfo->data, data, dataLength );
            rptInfo->rssi = rssi;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 data_length;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_ADV_REPORT) == 0 )) )
        {
            // the event mask is not set for this event
            return;
        }

        // data length
        data_length = HCI_PRD_ADV_REPORT_EVENT_LEN + dataLength;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + data_length;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = data_length;
            // populate event
            msg->pData[3] = HCI_BLE_PERIODIC_ADV_REPORT_EVENT;                // event code
            msg->pData[4] = LO_UINT16(syncHandle);
            msg->pData[5] = HI_UINT16(syncHandle);
            msg->pData[6] = txPower;
            msg->pData[7] = rssi;
            msg->pData[8] = cteType;
            msg->pData[9] = dataStatus;
            msg->pData[10] = dataLength;
            (void)osal_memcpy (&msg->pData[11], data, dataLength);   // data
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_PrdAdvSyncEstablishedCback Callback

    @brief       This LL callback is used to generate a Advertisment Report meta
                event when an Advertisment or Scan Response is received by a
                Scanner.

    input parameters

    @param       advEvt      - Advertise event type, or Scan Response event type.
    @param       advAddrType - Public or Random address type.
    @param       advAddr     - Pointer to device address.
    @param       dataLen     - Length of data in bytes.
    @param       advData     - Pointer to data.
    @param       rssi        - The RSSI of received packet.

    output parameters

    @param       None.

    @return      None.
*/
void LL_PrdAdvSyncLostCback(uint16  syncHandle)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEPrdAdvSyncLostPkt_t* pkt;
        pkt = (hciEvt_BLEPrdAdvSyncLostPkt_t*)osal_msg_allocate(sizeof(hciEvt_BLEPrdAdvSyncLostPkt_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_PERIODIC_ADV_SYNC_LOST_EVENT;
            pkt->syncHandle      = syncHandle;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;
//    // check if LE Meta-Events are enabled and this event is enabled
//    if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
//        (((bleEvtMask & LE_EVT_MASK_ADV_REPORT) == 0 )) )
//    {
//      // the event mask is not set for this event
//      return;
//    }
        // data length
        dataLength = HCI_PRD_ADV_SYNC_LOST_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_PERIODIC_ADV_SYNC_LOST_EVENT;     // event code
            msg->pData[4] = LO_UINT16(syncHandle);
            msg->pData[5] = HI_UINT16(syncHandle);
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


void LL_ConnectionlessIQReportCback(            uint16 syncHandle,
                                                uint8  chan_idx,
                                                int16  rssi,
                                                uint8  rssi_antID,
                                                uint8  cte_type,
                                                uint8  slot_duration,
                                                uint8  packet_status,
                                                uint16 PE_Cnt,
                                                uint8  sampCnt,
                                                uint16*  ISample,
                                                uint16*  QSample)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEConnectionlessIQ_Pkt_t* pkt;
        pkt = (hciEvt_BLEConnectionlessIQ_Pkt_t*)osal_msg_allocate(sizeof(hciEvt_BLEConnectionlessIQ_Pkt_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_LE_CONNECTIONLESS_IQ_REPORT_EVENT;
            pkt->ConnectionlessIQ.Handle    = syncHandle;
            pkt->ConnectionlessIQ.chan_idx  = chan_idx;
            pkt->ConnectionlessIQ.rssi      = rssi;
            pkt->ConnectionlessIQ.rssi_antID    = rssi_antID;
            pkt->ConnectionlessIQ.cte_type  = cte_type;
            pkt->ConnectionlessIQ.slot_duration = slot_duration;
            pkt->ConnectionlessIQ.packet_status = packet_status;
            pkt->ConnectionlessIQ.EventCnt      = PE_Cnt;
            pkt->ConnectionlessIQ.sampCnt   = sampCnt;
            (void)osal_memcpy(&pkt->ConnectionlessIQ.ISample[0],ISample,sampCnt);
            (void)osal_memcpy(&pkt->ConnectionlessIQ.QSample[0],QSample,sampCnt);
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;
        // data length
        dataLength = sizeof(hciEvt_BLEConnectionlessIQ_Pkt_t);
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData  = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3]   = HCI_LE_CONNECTIONLESS_IQ_REPORT_EVENT;    // event code
            msg->pData[4]   = LO_UINT16(syncHandle);
            msg->pData[5]   = HI_UINT16(syncHandle);
            msg->pData[6]   = chan_idx;
            msg->pData[7]   = rssi;
            msg->pData[8]   = rssi_antID;
            msg->pData[9]   = cte_type;
            msg->pData[10]  = slot_duration;
            msg->pData[11]  = packet_status;
            msg->pData[12]  = LO_UINT16(PE_Cnt);
            msg->pData[13]  = HI_UINT16(PE_Cnt);
            msg->pData[14]  = sampCnt;
            (void)osal_memcpy(&msg->pData[15],ISample,sampCnt);
            (void)osal_memcpy(&msg->pData[15 + sampCnt],QSample,sampCnt);
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

void LL_ConnectionIQReportCback(            uint16 connHandle,
                                            uint8  rx_PHY,
                                            uint8  data_chan_idx,
                                            int16  rssi,
                                            uint8  rssi_antID,
                                            uint8  cte_type,
                                            uint8  slot_duration,
                                            uint8  packet_status,
                                            uint16 connEventCounter,
                                            uint8  sampCnt,
                                            uint16*  ISample,
                                            uint16*  QSample)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEConnectionIQ_Pkt_t* pkt;
        pkt = (hciEvt_BLEConnectionIQ_Pkt_t*)osal_msg_allocate(sizeof(hciEvt_BLEConnectionIQ_Pkt_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_LE_CONNECTION_IQ_REPORT_EVENT;
            pkt->RX_PHY = rx_PHY;
            pkt->ConnectionIQ.Handle    = connHandle;
            pkt->ConnectionIQ.chan_idx  = data_chan_idx;
            pkt->ConnectionIQ.rssi      = rssi;
            pkt->ConnectionIQ.rssi_antID    = rssi_antID;
            pkt->ConnectionIQ.cte_type  = cte_type;
            pkt->ConnectionIQ.slot_duration = slot_duration;
            pkt->ConnectionIQ.packet_status = packet_status;
            pkt->ConnectionIQ.EventCnt      = connEventCounter;
            pkt->ConnectionIQ.sampCnt   = sampCnt;
            (void)osal_memcpy(&pkt->ConnectionIQ.ISample[0],ISample,sampCnt);
            (void)osal_memcpy(&pkt->ConnectionIQ.QSample[0],QSample,sampCnt);
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;
        // data length
        dataLength = sizeof(hciEvt_BLEConnectionIQ_Pkt_t);
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData  = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3]   = HCI_LE_CONNECTION_IQ_REPORT_EVENT;    // event code
            msg->pData[4]   = LO_UINT16(connHandle);
            msg->pData[5]   = HI_UINT16(connHandle);
            msg->pData[6]   = rx_PHY;
            msg->pData[7]   = data_chan_idx;
            msg->pData[8]   = rssi;
            msg->pData[9]   = rssi_antID;
            msg->pData[10]  = cte_type;
            msg->pData[11]  = slot_duration;
            msg->pData[12]  = packet_status;
            msg->pData[13]  = LO_UINT16(connEventCounter);
            msg->pData[14]  = HI_UINT16(connEventCounter);
            msg->pData[15]  = sampCnt;
            (void)osal_memcpy(&msg->pData[16],ISample,sampCnt);
            (void)osal_memcpy(&msg->pData[16 + sampCnt],QSample,sampCnt);
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

void LL_CTE_Report_FailedCback( uint8 status,uint16 connHandle)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLE_CTEReport_Pkt_t* pkt;
        pkt = (hciEvt_BLE_CTEReport_Pkt_t*)osal_msg_allocate(sizeof(hciEvt_BLE_CTEReport_Pkt_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_LE_CTE_REQUEST_FAILED_REPORT;
            pkt->status = status;
            pkt->connHandle = connHandle;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;
        // data length
        dataLength = sizeof(hciEvt_BLE_CTEReport_Pkt_t);
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData  = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3]   = HCI_LE_CTE_REQUEST_FAILED_REPORT; // event code
            msg->pData[4]   = status;
            msg->pData[4]   = LO_UINT16(connHandle);
            msg->pData[5]   = HI_UINT16(connHandle);
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_ConnectionCompleteCback Callback

    @brief       This LL callback is used to generate a Connection Complete meta
                event when a connection is established by either an Advertiser
                or an Initiator.

    input parameters

    @param       reasonCode    - Status of connection complete.
    @param       connHandle    - Connection handle.
    @param       role          - Connection formed as Master or Slave.
    @param       peerAddrType  - Peer address as Public or Random.
    @param       peerAddr      - Pointer to peer device address.
    @param       connInterval  - Connection interval.
    @param       slaveLatency  - Slave latency.
    @param       connTimeout   - Connection timeout.
    @param       clockAccuracy - Sleep clock accuracy (from Master only).

    output parameters

    @param       None.

    @return      None.
*/
void LL_ConnectionCompleteCback( uint8  reasonCode,
                                 uint16 connHandle,
                                 uint8  role,
                                 uint8  peerAddrType,
                                 uint8*  peerAddr,
                                 uint16 connInterval,
                                 uint16 slaveLatency,
                                 uint16 connTimeout,
                                 uint8  clockAccuracy )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEConnComplete_t* pkt;
        pkt = (hciEvt_BLEConnComplete_t*)osal_msg_allocate( sizeof ( hciEvt_BLEConnComplete_t ) );

        if ( pkt )
        {
            pkt->hdr.event    = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status   = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_CONNECTION_COMPLETE_EVENT;

            if ( reasonCode == LL_STATUS_SUCCESS )
            {
                pkt->status = HCI_SUCCESS;
                (void)osal_memcpy( pkt->peerAddr, peerAddr, B_ADDR_LEN );
            }
            else
            {
                pkt->status = bleGAPConnNotAcceptable;
                (void)osal_memset( pkt->peerAddr, 0, B_ADDR_LEN );
            }

            pkt->connectionHandle = connHandle;
            pkt->role             = role;
            pkt->peerAddrType     = peerAddrType;
            pkt->connInterval     = connInterval;
            pkt->connLatency      = slaveLatency;
            pkt->connTimeout      = connTimeout;
            pkt->clockAccuracy    = clockAccuracy;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_CONN_COMPLETE) == 0 )) )
        {
            // the event mask is not enabled for this event
            return;
        }

        // data length
        dataLength = HCI_CONNECTION_COMPLETE_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_CONNECTION_COMPLETE_EVENT;          // event code
            msg->pData[4] = reasonCode;                                 // reason code
            msg->pData[5] = LO_UINT16 (connHandle);                     // connection handle (LSB)
            msg->pData[6] = HI_UINT16 (connHandle);                     // connection handle (MSB)
            msg->pData[7] = role;                                       // role (master/slave)
            msg->pData[8] = peerAddrType;                               // address type
            // copy address
            (void)osal_memcpy (&msg->pData[9], peerAddr, B_ADDR_LEN);
            msg->pData[15] = LO_UINT16 (connInterval);                  // connection interval (LSB)
            msg->pData[16] = HI_UINT16 (connInterval);                  // connection interval (MSB)
            msg->pData[17] = LO_UINT16 (slaveLatency);                  // slave latency (LSB)
            msg->pData[18] = HI_UINT16 (slaveLatency);                  // slave latency (LSB)
            msg->pData[19] = LO_UINT16 (connTimeout);                   // connectin timeout (LSB)
            msg->pData[20] = HI_UINT16 (connTimeout);                   // connection timeout (MSB)
            msg->pData[21] = clockAccuracy;                             // clock accuracy
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_EnhConnectionCompleteCback Callback

    @brief       This LL callback is used to generate a Connection Complete meta
                event when a connection is established by either an Advertiser
                or an Initiator.

    input parameters

    @param       reasonCode    - Status of connection complete.
    @param       connHandle    - Connection handle.
    @param       role          - Connection formed as Master or Slave.
    @param       peerAddrType  - Peer address as Public or Random.
    @param       peerAddr      - Pointer to peer device address.
    @param       localRpaAddr  - Pointer to local RPA
    @param       peerRpaAddr   - Pointer to peer RPA
    @param       connInterval  - Connection interval.
    @param       slaveLatency  - Slave latency.
    @param       connTimeout   - Connection timeout.
    @param       clockAccuracy - Sleep clock accuracy (from Master only).

    output parameters

    @param       None.

    @return      None.
*/
void LL_EnhConnectionCompleteCback( uint8  reasonCode,
                                    uint16 connHandle,
                                    uint8  role,
                                    uint8  peerAddrType,
                                    uint8*  peerAddr,
                                    uint8*  localRpaAddr,
                                    uint8*  peerRpaAddr,
                                    uint16 connInterval,
                                    uint16 slaveLatency,
                                    uint16 connTimeout,
                                    uint8  clockAccuracy )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEEnhConnComplete_t* pkt;
        pkt = (hciEvt_BLEEnhConnComplete_t*)osal_msg_allocate( sizeof ( hciEvt_BLEEnhConnComplete_t ) );

        if ( pkt )
        {
            pkt->hdr.event    = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status   = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_ENHANCED_CONNECTION_COMPLETE_EVENT;

            if ( reasonCode == LL_STATUS_SUCCESS )
            {
                pkt->status = HCI_SUCCESS;
                (void)osal_memcpy( pkt->peerAddr, peerAddr, B_ADDR_LEN );
                (void)osal_memcpy( pkt->localRpaAddr, localRpaAddr, B_ADDR_LEN );
                (void)osal_memcpy( pkt->peerRpaAddr, peerRpaAddr, B_ADDR_LEN );
            }
            else
            {
                pkt->status = bleGAPConnNotAcceptable;
                (void)osal_memset( pkt->peerAddr, 0, B_ADDR_LEN );
            }

            pkt->connectionHandle = connHandle;
            pkt->role             = role;
            pkt->peerAddrType     = peerAddrType;
            pkt->connInterval     = connInterval;
            pkt->connLatency      = slaveLatency;
            pkt->connTimeout      = connTimeout;
            pkt->clockAccuracy    = clockAccuracy;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_CONN_COMPLETE) == 0 )) )
        {
            // the event mask is not enabled for this event
            return;
        }

        // data length
        dataLength = HCI_ENH_CONN_COMPLETE_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_CONNECTION_COMPLETE_EVENT;          // event code
            msg->pData[4] = reasonCode;                                 // reason code
            msg->pData[5] = LO_UINT16 (connHandle);                     // connection handle (LSB)
            msg->pData[6] = HI_UINT16 (connHandle);                     // connection handle (MSB)
            msg->pData[7] = role;                                       // role (master/slave)
            msg->pData[8] = peerAddrType;                               // address type

            if ( reasonCode == LL_STATUS_SUCCESS )
            {
//        pkt->status = HCI_SUCCESS;
                (void)osal_memcpy (&msg->pData[9], peerAddr, B_ADDR_LEN);
                (void)osal_memcpy (&msg->pData[15], localRpaAddr, B_ADDR_LEN);
                (void)osal_memcpy (&msg->pData[21], peerRpaAddr, B_ADDR_LEN);
            }
            else
            {
//        pkt->status = bleGAPConnNotAcceptable;
                (void)osal_memset( &msg->pData[9], 0, B_ADDR_LEN );
            }

            // copy address
            (void)osal_memcpy (&msg->pData[9], peerAddr, B_ADDR_LEN);
            msg->pData[27] = LO_UINT16 (connInterval);                  // connection interval (LSB)
            msg->pData[28] = HI_UINT16 (connInterval);                  // connection interval (MSB)
            msg->pData[29] = LO_UINT16 (slaveLatency);                  // slave latency (LSB)
            msg->pData[30] = HI_UINT16 (slaveLatency);                  // slave latency (LSB)
            msg->pData[31] = LO_UINT16 (connTimeout);                   // connectin timeout (LSB)
            msg->pData[32] = HI_UINT16 (connTimeout);                   // connection timeout (MSB)
            msg->pData[33] = clockAccuracy;                             // clock accuracy
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_DirectAdvReportCback Callback

    @brief       This LL callback is used to generate a Advertisment Report meta
                event when an Advertisment or Scan Response is received by a
                Scanner.

    input parameters

    @param       advEvt      - Advertise event type, or Scan Response event type.
    @param       advAddrType - Public or Random address type.
    @param       advAddr     - Pointer to device address.
    @param       directAddrType
    @param       directAddr
    @param       rssi        - The RSSI of received packet.

    output parameters

    @param       None.

    @return      None.
*/
void LL_DirectAdvReportCback( uint8 advEvt,
                              uint8 advAddrType,
                              uint8* advAddr,
                              uint8 directAddrType,
                              uint8* directAddr,
                              int8  rssi )
{
}

/*******************************************************************************
    @fn          LL_DisconnectCback Callback

    @brief       This LL callback is used to generate a Disconnect Complete meta
                event when a connection is disconnected by either a Master or
                a Slave.

    input parameters

    @param       connHandle - Connection handle.
    @param       reasonCode - Status of connection complete.

    output parameters

    @param       None.

    @return      None.
*/
void LL_DisconnectCback( uint16 connHandle,
                         uint8  reasonCode )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_DisconnComplete_t* pkt;
        pkt = (hciEvt_DisconnComplete_t*)osal_msg_allocate( sizeof ( hciEvt_DisconnComplete_t ) );

        if ( pkt )
        {
            pkt->hdr.event  = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_DISCONNECTION_COMPLETE_EVENT_CODE;
            pkt->status     = HCI_SUCCESS;
            pkt->connHandle = connHandle;
            pkt->reason     = reasonCode;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check the event mask
        if( ( HCI_EVT_MASK_DISCONN_COMPLETE&
                pHciEvtMask[HCI_EVT_INDEX_DISCONN_COMPLETE]) == 0 )
        {
            // event mask is not set for this event, do not send to host
            return;
        }

        // data length
        dataLength = HCI_DISCONNECTION_COMPLETE_LEN;
        // the length will be OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_DISCONNECTION_COMPLETE_EVENT_CODE;
            msg->pData[2] = dataLength;
            msg->pData[3] = HCI_SUCCESS;
            msg->pData[4] = LO_UINT16(connHandle);      // connection handle (LSB)
            msg->pData[5] = HI_UINT16(connHandle);      // connection handle (MSB)
            msg->pData[6] = reasonCode;                 // reason code
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_ConnParamUpdateCback Callback

    @brief       This LL callback is used to generate a Connection Update
                Complete meta event when a connection's parameters are updated
                by the Master.

    input parameters

    @param       connHandle   - Connection handle.
    @param       connInterval - Connection interval.
    @param       slaveLatency - Slave latency.
    @param       connTimeout  - Connection timeout.

    output parameters

    @param       None.

    @return      None.
*/
void LL_ConnParamUpdateCback( uint16 connHandle,
                              uint16 connInterval,
                              uint16 connLatency,
                              uint16 connTimeout )
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEConnUpdateComplete_t* msg;
        uint8 totalLength;

        // check the event mask
        if( (LE_EVT_MASK_CONN_UPDATE_COMPLETE & bleEvtMask) == 0 )
        {
            // event mask is not set for this event, do not send to host
            return;
        }

        totalLength = sizeof (hciEvt_BLEConnUpdateComplete_t);
        msg = (hciEvt_BLEConnUpdateComplete_t*)osal_msg_allocate(totalLength);

        if( msg )
        {
            // message header
            msg->hdr.event = HCI_GAP_EVENT_EVENT;
            msg->hdr.status = HCI_LE_EVENT_CODE; // use status field to pass the HCI Event code
            // event packet
            msg->BLEEventCode     = HCI_BLE_CONN_UPDATE_COMPLETE_EVENT;
            msg->status           = HCI_SUCCESS;
            msg->connectionHandle = connHandle;
            msg->connInterval     = connInterval;
            msg->connLatency      = connLatency;
            msg->connTimeout      = connTimeout;
            // send the message
            (void)osal_msg_send( hciGapTaskID, (uint8*)msg );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_CONN_UPDATE_COMPLETE) == 0 )) )
        {
            // the event mask is not enabled for this event
            return;
        }

        // data length
        dataLength = HCI_LL_CONN_UPDATE_COMPLETE_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            uint8* pBuf;
            // message type
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // point to the byte following the hciPacket_t structure
            msg->pData = (uint8*)(msg+1);
            pBuf       = msg->pData;
            *pBuf++ = HCI_EVENT_PACKET;
            *pBuf++ = HCI_LE_EVENT_CODE;
            *pBuf++ = dataLength;
            // populate event
            *pBuf++ = HCI_BLE_CONN_UPDATE_COMPLETE_EVENT;  // event code
            *pBuf++ = HCI_SUCCESS;
            *pBuf++ = LO_UINT16(connHandle);               // connection handle (LSB)
            *pBuf++ = HI_UINT16(connHandle);               // connection handle (MSB)
            *pBuf++ = LO_UINT16(connInterval);             // connection interval (LSB)
            *pBuf++ = HI_UINT16(connInterval);             // connection interval (MSB)
            *pBuf++ = LO_UINT16(connLatency);              // slave latency (LSB)
            *pBuf++ = HI_UINT16(connLatency);              // slave latency (MSB)
            *pBuf++ = LO_UINT16(connTimeout);              // connection timeout (LSB)
            *pBuf++ = HI_UINT16(connTimeout);              // connection timeout (MSB)
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_ReadRemoteUsedFeaturesCompleteCback Callback

    @brief       This LL callback is used to generate a Read Remote Used Features
                Complete meta event when a Master makes this request of a Slave.

    input parameters

    @param       status     - HCI status.
    @param       connHandle - Connection handle.
    @param       featureSet - Pointer to eight byte bit mask of LE features.

    output parameters

    @param       None.

    @return      None.
*/
void LL_ReadRemoteUsedFeaturesCompleteCback( hciStatus_t status,
                                             uint16      connHandle,
                                             uint8*       featureSet )
{
    hciPacket_t* msg;
    uint8 dataLength;
    uint8 totalLength;

    // check if LE Meta-Events are enabled and this event is enabled
    if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
            (((bleEvtMask & LE_EVT_MASK_READ_REMOTE_FEATURE) == 0 )) )
    {
        // the event mask is not enabled for this event
        return;
    }

    // data length
    dataLength = HCI_READ_REMOTE_FEATURE_COMPLETE_EVENT_LEN;
    // OSAL message header + HCI event header + data
    totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
    msg = (hciPacket_t*)osal_msg_allocate(totalLength);

    if (msg)
    {
        // message type, length
        msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
        msg->hdr.status = 0xFF;
        // create message
        msg->pData    = (uint8*)(msg+1);
        msg->pData[0] = HCI_EVENT_PACKET;
        msg->pData[1] = HCI_LE_EVENT_CODE;
        msg->pData[2] = dataLength;
        // event code
        msg->pData[3] = HCI_BLE_READ_REMOTE_FEATURE_COMPLETE_EVENT;
        // Note: The status is always success when this callback is called. When
        //       this action fails, the LL doesn't call this callback. Most likely
        //       the failure is caused by lost of link connection and will be
        //       handled by the GAP as connection termination.
        msg->pData[4] = status;
        msg->pData[5] = LO_UINT16(connHandle);  // connection handle (LSB)
        msg->pData[6] = HI_UINT16(connHandle);  // connection handle (MSB)
        // feature set
        (void)osal_memcpy (&msg->pData[7], featureSet, B_FEATURE_SUPPORT_LENGTH);
        // send the message
        (void)osal_msg_send( hciTaskID, (uint8*)msg );
    }
}


/*******************************************************************************
    @fn          LL_ReadRemoteVersionInfoCback Callback

    @brief       This LL callback is used to generate a Read Remote Version
                Information Complete meta event when a Master makes this request
                of a Slave.

    input parameters

    @param       status     - Status of callback.
    @param       verNum     - Version of the Bluetooth Controller specification.
    @param       connHandle - Company identifier of the manufacturer of the
                             Bluetooth Controller.
    @param       subverNum  - A unique value for each implementation or revision
                             of an implementation of the Bluetooth Controller.

    output parameters

    @param       None.

    @return      None.
*/
void LL_ReadRemoteVersionInfoCback( hciStatus_t status,
                                    uint16 connHandle,
                                    uint8  verNum,
                                    uint16 comId,
                                    uint16 subverNum )
{
    hciPacket_t* msg;
    uint8 dataLength;
    uint8 totalLength;

    // check the event mask
    if( ( HCI_EVT_MASK_READ_REMOTE_VER&
            pHciEvtMask[HCI_EVT_INDEX_READ_REMOTE_VER]) == 0 )
    {
        // event mask is not set for this event, do not send to host
        return;
    }

    // data length
    dataLength = HCI_REMOTE_VERSION_INFO_EVENT_LEN;
    // OSAL message header + HCI event header + data
    totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
    msg = (hciPacket_t*)osal_msg_allocate(totalLength);

    if (msg)
    {
        // message type, length
        msg->hdr.event   = HCI_CTRL_TO_HOST_EVENT;
        msg->hdr.status = 0xFF;
        // create message
        msg->pData     = (uint8*)(msg+1);
        msg->pData[0]  = HCI_EVENT_PACKET;
        msg->pData[1]  = HCI_READ_REMOTE_INFO_COMPLETE_EVENT_CODE;
        msg->pData[2]  = dataLength;
        msg->pData[3]  = status;
        msg->pData[4]  = LO_UINT16( connHandle );
        msg->pData[5]  = HI_UINT16( connHandle );
        msg->pData[6]  = verNum;
        msg->pData[7]  = LO_UINT16( comId );       // company ID (LSB)
        msg->pData[8]  = HI_UINT16( comId );       // company ID (MSB)
        msg->pData[9]  = LO_UINT16( subverNum );
        msg->pData[10] = HI_UINT16( subverNum );
        // send the message
        (void)osal_msg_send( hciTaskID, (uint8*)msg );
    }
}


/*******************************************************************************
    @fn          LL_EncLtkReqCback Callback

    @brief       This LL callback is used to generate a Encryption LTK Request
                meta event to provide to the Host the Master's random number
                and encryption diversifier, and to request the Host's Long Term
                Key (LTK).

    input parameters

    @param       connHandle - The LL connection ID for new connection.
    @param       randNum    - Random vector used in device identification.
    @param       encDiv     - Encrypted diversifier.

    output parameters

    @param       None.

    @return      None.
*/
void LL_EncLtkReqCback( uint16 connHandle,
                        uint8*  randNum,
                        uint8*  encDiv )
{
    // check if this is for the Host
    if ( hciSmpTaskID != 0 )
    {
        hciEvt_BLELTKReq_t* pkt;
        pkt = (hciEvt_BLELTKReq_t*)osal_msg_allocate( sizeof ( hciEvt_BLELTKReq_t ) );

        if ( pkt )
        {
            pkt->hdr.event    = HCI_SMP_EVENT_EVENT;
            pkt->hdr.status   = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_LTK_REQUESTED_EVENT;
            pkt->connHandle   = connHandle;
            (void)osal_memcpy( pkt->random, randNum, B_RANDOM_NUM_SIZE );
            pkt->encryptedDiversifier = BUILD_UINT16( encDiv[0], encDiv[1] );
            (void)osal_msg_send( hciSmpTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_LTK_REQUEST) == 0 )) )
        {
            // the event mask is not set for this event, so reject immediately
            LL_EncLtkNegReply( connHandle );
            return;
        }

        // data length
        dataLength = HCI_LTK_REQUESTED_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            uint8* pBuf;
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData = (uint8*)(msg+1);
            pBuf       = msg->pData;
            *pBuf++ = HCI_EVENT_PACKET;
            *pBuf++ = HCI_LE_EVENT_CODE;
            *pBuf++ = dataLength;
            *pBuf++ = HCI_BLE_LTK_REQUESTED_EVENT;                 // event code
            *pBuf++ = LO_UINT16(connHandle);                       // connection handle (LSB)
            *pBuf++ = HI_UINT16(connHandle);                       // connection handle (MSB)
            (void)osal_memcpy (pBuf, randNum, B_RANDOM_NUM_SIZE);  // random number
            pBuf += B_RANDOM_NUM_SIZE;                             // size of random number
            *pBuf++ = *encDiv++;                                   // encryption diversifier (LSB)
            *pBuf   = *encDiv;                                     // encryption diversifier (MSB)
            /* Send the message */
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}


/*******************************************************************************
    @fn          LL_EncChangeCback Callback

    @brief       This Callback is used by the LL to indicate to the Host that
                an encryption change has taken place. This results when
                the host performs a LL_StartEncrypt when encryption is not
                already enabled.

                Note: If the key request was rejected, then encryption will
                      remain off.

    input parameters

    @param       connHandle - The LL connection ID for new connection.
    @param       reason     - LL_ENC_KEY_REQ_ACCEPTED or LL_ENC_KEY_REQ_REJECTED.
    @param       encEnab    - LL_ENCRYPTION_OFF or LL_ENCRYPTION_ON.

    output parameters

    @param       None.

    @return      None.
*/
void LL_EncChangeCback( uint16 connHandle,
                        uint8  reason,
                        uint8  encEnab )
{
    // check if this is for the Host
    if ( hciSmpTaskID != 0 )
    {
        hciEvt_EncryptChange_t* pkt;
        pkt = (hciEvt_EncryptChange_t*)osal_msg_allocate( sizeof ( hciEvt_EncryptChange_t ) );

        if ( pkt )
        {
            pkt->hdr.event    = HCI_SMP_EVENT_EVENT;
            pkt->hdr.status   = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_ENCRYPTION_CHANGE_EVENT_CODE;
            pkt->connHandle   = connHandle;
            pkt->reason       = reason;
            pkt->encEnable    = encEnab;
            (void)osal_msg_send( hciSmpTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check the event mask
        if( ( HCI_EVT_MASK_ENCRYPTION_CHANGE&
                pHciEvtMask[HCI_EVT_INDEX_ENCRYPTION_CHANGE]) == 0 )
        {
            // event mask is not set for this event, do not send to host
            return;
        }

        // data length
        dataLength = HCI_ENCRYPTION_CHANGE_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            uint8* pBuf;
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData = (uint8*)(msg+1);
            pBuf       = msg->pData;
            // populate event
            *pBuf++    = HCI_EVENT_PACKET;
            *pBuf++    = HCI_ENCRYPTION_CHANGE_EVENT_CODE;
            *pBuf++    = dataLength;
            *pBuf++    = reason;
            *pBuf++    = LO_UINT16(connHandle);
            *pBuf++    = HI_UINT16(connHandle);
            *pBuf      = encEnab;
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

/*******************************************************************************
    @fn          LL_EncKeyRefreshCback Callback

    @brief       This Callback is used by the LL to indicate to the Host that
                an encryption key refresh has taken place. This results when
                the host performs a LL_StartEncrypt when encryption is already
                enabled.

    input parameters

    @param       connHandle - The LL connection ID for new connection.
    @param       reason    - LL_ENC_KEY_REQ_ACCEPTED, LL_CTRL_PKT_TIMEOUT_TERM

    output parameters

    @param       None.

    @return      None.
*/
void LL_EncKeyRefreshCback( uint16 connHandle,
                            uint8  reason )
{
    // check if this is for the Host
    if ( hciSmpTaskID != 0 )
    {
        hciEvt_EncryptChange_t* pkt;
        pkt = (hciEvt_EncryptChange_t*)osal_msg_allocate( sizeof ( hciEvt_EncryptChange_t ) );

        if ( pkt )
        {
            pkt->hdr.event    = HCI_SMP_EVENT_EVENT;
            pkt->hdr.status   = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_ENCRYPTION_CHANGE_EVENT_CODE;
            pkt->connHandle   = connHandle;
            pkt->reason       = reason;
            pkt->encEnable    = TRUE;
            (void)osal_msg_send( hciSmpTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check the event mask
        if( ( HCI_EVT_MASK_KEY_REFRESH_COMPLETE&
                pHciEvtMask[HCI_EVT_INDEX_KEY_REFRESH_COMPLETE]) == 0 )
        {
            /* Event mask is not set for this event, do not send to host */
            return;
        }

        // data length
        dataLength = HCI_KEY_REFRESH_COMPLETE_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            uint8* pBuf;
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData = (uint8*)(msg+1);
            pBuf       = msg->pData;
            // populate event
            *pBuf++    = HCI_EVENT_PACKET;
            *pBuf++    = HCI_KEY_REFRESH_COMPLETE_EVENT_CODE;
            *pBuf++    = dataLength;
            *pBuf++    = reason;
            *pBuf++    = LO_UINT16(connHandle);
            *pBuf++    = HI_UINT16(connHandle);
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

/*******************************************************************************
    @fn          LL_DirectTestEndDone Callback

    @brief       This Callback is used by the LL to notify the HCI that the
                Direct Test End command has completed.


    input parameters

    @param       numPackets - The number of packets received. Zero for transmit.
    @param       mode       - LL_DIRECT_TEST_MODE_TX or LL_DIRECT_TEST_MODE_RX.

    output parameters

    @param       None.

    @return      LL_STATUS_SUCCESS
*/
void LL_DirectTestEndDoneCback( uint16 numPackets,
                                uint8  mode )
{
    uint8 rtnParam[3];
    // unused input parameter; PC-Lint error 715.
    (void)mode;
    rtnParam[0] = HCI_SUCCESS;
    rtnParam[1] = LO_UINT16( numPackets );
    rtnParam[2] = HI_UINT16( numPackets );
    HCI_CommandCompleteEvent( HCI_LE_TEST_END, 3, rtnParam );
}




/*******************************************************************************
    @fn          LL_DataLengthChange Callback

    @brief       This Callback is used by the LL to notify the HCI that the
                DATA LENGTH has been change.


    input parameters

    @param       connHandle - The LL connection ID for new connection.
    @param       MaxTxOctets, MaxTime, MaxRxOctets, MaxRxTime

    output parameters

    @param       None.

    @return      LL_STATUS_SUCCESS
*/
void LL_DataLengthChangeCback(uint16 connHandle,
                              uint16 MaxTxOctets,
                              uint16 MaxTxTime,
                              uint16 MaxRxOctets,
                              uint16 MaxRxTime)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEDataLenChange_t* pkt;
        pkt = (hciEvt_BLEDataLenChange_t*)osal_msg_allocate( sizeof ( hciEvt_BLEDataLenChange_t ) );

        if ( pkt )
        {
            pkt->hdr.event    = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status   = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_DATA_LENGTH_CHANGE_EVENT;
            pkt->connHandle = connHandle;
            pkt->MaxTxOctets= MaxTxOctets;
            pkt->MaxTxTime  = MaxTxTime;
            pkt->MaxRxOctets= MaxRxOctets;
            pkt->MaxRxTime  = MaxRxTime;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_DATA_LENGTH_CHANGE) == 0 )) )
        {
            // the event mask is not enabled for this event
            return;
        }

        // data length
        dataLength = HCI_LL_DATA_LENGTH_CHANGE_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_DATA_LENGTH_CHANGE_EVENT;           // event code
            msg->pData[4] = LO_UINT16 (connHandle);                     // connection handle (LSB)
            msg->pData[5] = HI_UINT16 (connHandle);                     // connection handle (MSB)
            msg->pData[6] = LO_UINT16 (MaxTxOctets);
            msg->pData[7] = HI_UINT16 (MaxTxOctets);
            msg->pData[8] = LO_UINT16 (MaxTxTime);
            msg->pData[9] = HI_UINT16 (MaxTxTime);
            msg->pData[10]= LO_UINT16 (MaxRxOctets);
            msg->pData[11]= HI_UINT16 (MaxRxOctets);
            msg->pData[12]= LO_UINT16 (MaxRxTime);
            msg->pData[13]= HI_UINT16 (MaxRxTime);
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}



/*******************************************************************************
    @fn          LL_PHY_UPDATE_COMPLETE Callback

    @brief       This Callback is used by the LL to notify the HCI that the
                LL_PHY has been change.


    input parameters

    @param       connHandle - The LL connection ID for new connection.
    @param       MaxTxOctets, MaxTime, MaxRxOctets, MaxRxTime

    output parameters

    @param       None.

    @return      LL_STATUS_SUCCESS
*/
void LL_PhyUpdateCompleteCback(uint16 connHandle,
                               uint8 status,
                               uint8 txPhy,
                               uint8 rxPhy)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEPhyUpdateComplete_t* pkt;
        pkt = (hciEvt_BLEPhyUpdateComplete_t*)osal_msg_allocate( sizeof ( hciEvt_BLEPhyUpdateComplete_t ) );

        if ( pkt )
        {
            pkt->hdr.event    = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status   = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_PHY_UPDATE_COMPLETE_EVENT;
            pkt->status     = status;
            pkt->connHandle = connHandle;
            pkt->txPhy      = txPhy;
            pkt->rxPhy      = rxPhy;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 dataLength;
        uint8 totalLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_PHY_CHANGE) == 0 )) )
        {
            // the event mask is not enabled for this event
            return;
        }

        // data length
        dataLength = HCI_LL_PHY_UPDATE_COMPLETE_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_PHY_UPDATE_COMPLETE_EVENT;           // event code
            msg->pData[4] = status;
            msg->pData[5] = LO_UINT16 (connHandle);                     // connection handle (LSB)
            msg->pData[6] = HI_UINT16 (connHandle);                     // connection handle (MSB)
            msg->pData[7] = txPhy;
            msg->pData[8] = rxPhy;
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

/*******************************************************************************
*/

