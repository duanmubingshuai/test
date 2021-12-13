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

/*******************************************************************************
  Filename:       hci.h
  Revised:        
  Revision:        

  Description:    This file contains the Host Controller Interface (HCI) API.
                  It provides the defines, types, and functions for all
                  supported Bluetooth Low Energy (BLE) commands.

                  All Bluetooth and BLE commands are based on:
                  Bluetooth Core Specification, V4.0.0, Vol. 2, Part E.


*******************************************************************************/

#ifndef HCI_H
#define HCI_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "ll.h"
//#include "hal_assert.h"

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*
** HCI Status
**
** Per the Bluetooth Core Specification, V4.0.0, Vol. 2, Part D.
*/
#define HCI_SUCCESS                                                         0x00
#define HCI_ERROR_CODE_UNKNOWN_HCI_CMD                                      0x01
#define HCI_ERROR_CODE_UNKNOWN_CONN_ID                                      0x02
#define HCI_ERROR_CODE_HW_FAILURE                                           0x03
#define HCI_ERROR_CODE_PAGE_TIMEOUT                                         0x04
#define HCI_ERROR_CODE_AUTH_FAILURE                                         0x05
#define HCI_ERROR_CODE_PIN_KEY_MISSING                                      0x06
#define HCI_ERROR_CODE_MEM_CAP_EXCEEDED                                     0x07
#define HCI_ERROR_CODE_CONN_TIMEOUT                                         0x08
#define HCI_ERROR_CODE_CONN_LIMIT_EXCEEDED                                  0x09
#define HCI_ERROR_CODE_SYNCH_CONN_LIMIT_EXCEEDED                            0x0A
#define HCI_ERROR_CODE_ACL_CONN_ALREADY_EXISTS                              0x0B
#define HCI_ERROR_CODE_CMD_DISALLOWED                                       0x0C
#define HCI_ERROR_CODE_CONN_REJ_LIMITED_RESOURCES                           0x0D
#define HCI_ERROR_CODE_CONN_REJECTED_SECURITY_REASONS                       0x0E
#define HCI_ERROR_CODE_CONN_REJECTED_UNACCEPTABLE_BDADDR                    0x0F
#define HCI_ERROR_CODE_CONN_ACCEPT_TIMEOUT_EXCEEDED                         0x10
#define HCI_ERROR_CODE_UNSUPPORTED_FEATURE_PARAM_VALUE                      0x11
#define HCI_ERROR_CODE_INVALID_HCI_CMD_PARAMS                               0x12
#define HCI_ERROR_CODE_REMOTE_USER_TERM_CONN                                0x13
#define HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_LOW_RESOURCES                0x14
#define HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_POWER_OFF                    0x15
#define HCI_ERROR_CODE_CONN_TERM_BY_LOCAL_HOST                              0x16
#define HCI_ERROR_CODE_REPEATED_ATTEMPTS                                    0x17
#define HCI_ERROR_CODE_PAIRING_NOT_ALLOWED                                  0x18
#define HCI_ERROR_CODE_UNKNOWN_LMP_PDU                                      0x19
#define HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE                           0x1A
#define HCI_ERROR_CODE_SCO_OFFSET_REJ                                       0x1B
#define HCI_ERROR_CODE_SCO_INTERVAL_REJ                                     0x1C
#define HCI_ERROR_CODE_SCO_AIR_MODE_REJ                                     0x1D
#define HCI_ERROR_CODE_INVALID_LMP_PARAMS                                   0x1E
#define HCI_ERROR_CODE_UNSPECIFIED_ERROR                                    0x1F
#define HCI_ERROR_CODE_UNSUPPORTED_LMP_PARAM_VAL                            0x20
#define HCI_ERROR_CODE_ROLE_CHANGE_NOT_ALLOWED                              0x21
#define HCI_ERROR_CODE_LMP_LL_RESP_TIMEOUT                                  0x22
#define HCI_ERROR_CODE_LMP_ERR_TRANSACTION_COLLISION                        0x23
#define HCI_ERROR_CODE_LMP_PDU_NOT_ALLOWED                                  0x24
#define HCI_ERROR_CODE_ENCRYPT_MODE_NOT_ACCEPTABLE                          0x25
#define HCI_ERROR_CODE_LINK_KEY_CAN_NOT_BE_CHANGED                          0x26
#define HCI_ERROR_CODE_REQ_QOS_NOT_SUPPORTED                                0x27
#define HCI_ERROR_CODE_INSTANT_PASSED                                       0x28
#define HCI_ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED                  0x29
#define HCI_ERROR_CODE_DIFFERENT_TRANSACTION_COLLISION                      0x2A
#define HCI_ERROR_CODE_RESERVED1                                            0x2B
#define HCI_ERROR_CODE_QOS_UNACCEPTABLE_PARAM                               0x2C
#define HCI_ERROR_CODE_QOS_REJ                                              0x2D
#define HCI_ERROR_CODE_CHAN_ASSESSMENT_NOT_SUPPORTED                        0x2E
#define HCI_ERROR_CODE_INSUFFICIENT_SECURITY                                0x2F
#define HCI_ERROR_CODE_PARAM_OUT_OF_MANDATORY_RANGE                         0x30
#define HCI_ERROR_CODE_RESERVED2                                            0x31
#define HCI_ERROR_CODE_ROLE_SWITCH_PENDING                                  0x32
#define HCI_ERROR_CODE_RESERVED3                                            0x33
#define HCI_ERROR_CODE_RESERVED_SLOT_VIOLATION                              0x34
#define HCI_ERROR_CODE_ROLE_SWITCH_FAILED                                   0x35
#define HCI_ERROR_CODE_EXTENDED_INQUIRY_RESP_TOO_LARGE                      0x36
#define HCI_ERROR_CODE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST                 0x37
#define HCI_ERROR_CODE_HOST_BUSY_PAIRING                                    0x38
#define HCI_ERROR_CODE_CONN_REJ_NO_SUITABLE_CHAN_FOUND                      0x39
#define HCI_ERROR_CODE_CONTROLLER_BUSY                                      0x3A
#define HCI_ERROR_CODE_UNACCEPTABLE_CONN_INTERVAL                           0x3B
#define HCI_ERROR_CODE_DIRECTED_ADV_TIMEOUT                                 0x3C
#define HCI_ERROR_CODE_CONN_TERM_MIC_FAILURE                                0x3D
#define HCI_ERROR_CODE_CONN_FAILED_TO_ESTABLISH                             0x3E
#define HCI_ERROR_CODE_MAC_CONN_FAILED                                      0x3F

/*
** Max Buffers Supported
*/
#define HCI_MAX_NUM_DATA_BUFFERS                       LL_MAX_NUM_DATA_BUFFERS
#define HCI_MAX_NUM_CMD_BUFFERS                        LL_MAX_NUM_CMD_BUFFERS

/*
** HCI Command API Parameters
*/

// Send Data Packet Boundary Flags
#define FIRST_PKT_HOST_TO_CTRL                         LL_DATA_FIRST_PKT_HOST_TO_CTRL
#define CONTINUING_PKT                                 LL_DATA_CONTINUATION_PKT
#define FIRST_PKT_CTRL_TO_HOST                         LL_DATA_FIRST_PKT_CTRL_TO_HOST

// Receive Data Packet
#define HCI_RSSI_NOT_AVAILABLE                         LL_RSSI_NOT_AVAILABLE

// Disconnect Reasons
#define HCI_DISCONNECT_AUTH_FAILURE                    HCI_ERROR_CODE_AUTH_FAILURE
#define HCI_DISCONNECT_REMOTE_USER_TERM                HCI_ERROR_CODE_REMOTE_USER_TERM_CONN
#define HCI_DISCONNECT_REMOTE_DEV_LOW_RESOURCES        HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_LOW_RESOURCES
#define HCI_DISCONNECT_REMOTE_DEV_POWER_OFF            HCI_ERROR_CODE_REMOTE_DEVICE_TERM_CONN_POWER_OFF
#define HCI_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE      HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE
#define HCI_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED       HCI_ERROR_CODE_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED
#define HCI_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL      HCI_ERROR_CODE_UNACCEPTABLE_CONN_INTERVAL

// Tx Power Types
#define HCI_READ_CURRENT_TX_POWER_LEVEL                LL_READ_CURRENT_TX_POWER_LEVEL
#define HCI_READ_MAX_TX_POWER_LEVEL                    LL_READ_MAX_TX_POWER_LEVEL

// Host Flow Control
#define HCI_CTRL_TO_HOST_FLOW_CTRL_OFF                 0
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF    1
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON    2
#define HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON     3

// Device Address Type
#define HCI_PUBLIC_DEVICE_ADDRESS                      LL_DEV_ADDR_TYPE_PUBLIC
#define HCI_RANDOM_DEVICE_ADDRESS                      LL_DEV_ADDR_TYPE_RANDOM

// Advertiser Events
#define HCI_CONNECTABLE_UNDIRECTED_ADV                 LL_ADV_CONNECTABLE_UNDIRECTED_EVT
#define HCI_CONNECTABLE_DIRECTED_HDC_ADV               LL_ADV_CONNECTABLE_DIRECTED_HDC_EVT
#define HCI_SCANNABLE_UNDIRECTED                       LL_ADV_SCANNABLE_UNDIRECTED_EVT
#define HCI_NONCONNECTABLE_UNDIRECTED_ADV              LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT
#define HCI_CONNECTABLE_DIRECTED_LDC_ADV               LL_ADV_CONNECTABLE_DIRECTED_LDC_EVT

// Advertiser Channels
#define HCI_ADV_CHAN_37                                LL_ADV_CHAN_37
#define HCI_ADV_CHAN_38                                LL_ADV_CHAN_38
#define HCI_ADV_CHAN_39                                LL_ADV_CHAN_39
#define HCI_ADV_CHAN_ALL                               (LL_ADV_CHAN_37 | LL_ADV_CHAN_38 | LL_ADV_CHAN_39)

// Advertiser White List Policy
#define HCI_ADV_WL_POLICY_ANY_REQ                      LL_ADV_WL_POLICY_ANY_REQ
#define HCI_ADV_WL_POLICY_WL_SCAN_REQ                  LL_ADV_WL_POLICY_WL_SCAN_REQ
#define HCI_ADV_WL_POLICY_WL_CONNECT_REQ               LL_ADV_WL_POLICY_WL_CONNECT_REQ
#define HCI_ADV_WL_POLICY_WL_ALL_REQ                   LL_ADV_WL_POLICY_WL_ALL_REQ

// Advertiser Commands
#define HCI_ENABLE_ADV                                 LL_ADV_MODE_ON
#define HCI_DISABLE_ADV                                LL_ADV_MODE_OFF

// Scan Types
#define HCI_SCAN_PASSIVE                               LL_SCAN_PASSIVE
#define HCI_SCAN_ACTIVE                                LL_SCAN_ACTIVE

// Scan White List Policy
#define HCI_SCAN_WL_POLICY_ANY_ADV_PKTS                LL_SCAN_WL_POLICY_ANY_ADV_PKTS
#define HCI_SCAN_WL_POLICY_USE_WHITE_LIST              LL_SCAN_WL_POLICY_USE_WHITE_LIST

// Scan Filtering
#define HCI_FILTER_REPORTS_DISABLE                     LL_FILTER_REPORTS_DISABLE
#define HCI_FILTER_REPORTS_ENABLE                      LL_FILTER_REPORTS_ENABLE

// Scan Commands
#define HCI_SCAN_STOP                                  LL_SCAN_STOP
#define HCI_SCAN_START                                 LL_SCAN_START

// Initiator White List Policy
#define HCI_INIT_WL_POLICY_USE_PEER_ADDR               LL_INIT_WL_POLICY_USE_PEER_ADDR
#define HCI_INIT_WL_POLICY_USE_WHITE_LIST              LL_INIT_WL_POLICY_USE_WHITE_LIST

// Encryption Related
#define HCI_ENCRYPTION_OFF                             LL_ENCRYPTION_OFF
#define HCI_ENCRYPTION_ON                              LL_ENCRYPTION_ON

// Direct Test Mode
#define HCI_DTM_NUMBER_RF_CHANS                        LL_DIRECT_TEST_NUM_RF_CHANS
#define HCI_DIRECT_TEST_MAX_PAYLOAD_LEN                LL_DIRECT_TEST_MAX_PAYLOAD_LEN
//
#define HCI_DIRECT_TEST_PAYLOAD_PRBS9                  LL_DIRECT_TEST_PAYLOAD_PRBS9
#define HCI_DIRECT_TEST_PAYLOAD_0x0F                   LL_DIRECT_TEST_PAYLOAD_0x0F
#define HCI_DIRECT_TEST_PAYLOAD_0x55                   LL_DIRECT_TEST_PAYLOAD_0x55
#define HCI_DIRECT_TEST_PAYLOAD_PRBS15                 LL_DIRECT_TEST_PAYLOAD_PRBS15
#define HCI_DIRECT_TEST_PAYLOAD_0xFF                   LL_DIRECT_TEST_PAYLOAD_0xFF
#define HCI_DIRECT_TEST_PAYLOAD_0x00                   LL_DIRECT_TEST_PAYLOAD_0x00
#define HCI_DIRECT_TEST_PAYLOAD_0xF0                   LL_DIRECT_TEST_PAYLOAD_0xF0
#define HCI_DIRECT_TEST_PAYLOAD_0xAA                   LL_DIRECT_TEST_PAYLOAD_0xAA

// Vendor Specific
#define HCI_EXT_RX_GAIN_STD                            LL_EXT_RX_GAIN_STD
#define HCI_EXT_RX_GAIN_HIGH                           LL_EXT_RX_GAIN_HIGH
//
#define HCI_EXT_TX_POWER_MINUS_23_DBM                  LL_EXT_TX_POWER_MINUS_23_DBM
#define HCI_EXT_TX_POWER_MINUS_6_DBM                   LL_EXT_TX_POWER_MINUS_6_DBM
#define HCI_EXT_TX_POWER_0_DBM                         LL_EXT_TX_POWER_0_DBM
#define HCI_EXT_TX_POWER_4_DBM                         LL_EXT_TX_POWER_4_DBM
//
#define HCI_EXT_ENABLE_ONE_PKT_PER_EVT                 LL_EXT_ENABLE_ONE_PKT_PER_EVT
#define HCI_EXT_DISABLE_ONE_PKT_PER_EVT                LL_EXT_DISABLE_ONE_PKT_PER_EVT
//
#define HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT              LL_EXT_ENABLE_CLK_DIVIDE_ON_HALT
#define HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT             LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT
//
#define HCI_EXT_NV_IN_USE                              LL_EXT_NV_IN_USE
#define HCI_EXT_NV_NOT_IN_USE                          LL_EXT_NV_NOT_IN_USE
//
#define HCI_EXT_ENABLE_FAST_TX_RESP_TIME               LL_EXT_ENABLE_FAST_TX_RESP_TIME
#define HCI_EXT_DISABLE_FAST_TX_RESP_TIME              LL_EXT_DISABLE_FAST_TX_RESP_TIME
//
#define HCI_EXT_ENABLE_SL_OVERRIDE                     LL_EXT_ENABLE_SL_OVERRIDE
#define HCI_EXT_DISABLE_SL_OVERRIDE                    LL_EXT_DISABLE_SL_OVERRIDE
//
#define HCI_EXT_TX_MODULATED_CARRIER                   LL_EXT_TX_MODULATED_CARRIER
#define HCI_EXT_TX_UNMODULATED_CARRIER                 LL_EXT_TX_UNMODULATED_CARRIER
//
#define HCI_PTM_SET_FREQ_TUNE_DOWN                     LL_EXT_SET_FREQ_TUNE_DOWN
#define HCI_PTM_SET_FREQ_TUNE_UP                       LL_EXT_SET_FREQ_TUNE_UP
//
//#define HCI_EXT_PM_IO_PORT_P0                          LL_EXT_PM_IO_PORT_P0
//#define HCI_EXT_PM_IO_PORT_P1                          LL_EXT_PM_IO_PORT_P1
//#define HCI_EXT_PM_IO_PORT_P2                          LL_EXT_PM_IO_PORT_P2
//#define HCI_EXT_PM_IO_PORT_NONE                        LL_EXT_PM_IO_PORT_NONE
////
//#define HCI_EXT_PM_IO_PORT_PIN0                        LL_EXT_PM_IO_PORT_PIN0
//#define HCI_EXT_PM_IO_PORT_PIN1                        LL_EXT_PM_IO_PORT_PIN1
//#define HCI_EXT_PM_IO_PORT_PIN2                        LL_EXT_PM_IO_PORT_PIN2
//#define HCI_EXT_PM_IO_PORT_PIN3                        LL_EXT_PM_IO_PORT_PIN3
//#define HCI_EXT_PM_IO_PORT_PIN4                        LL_EXT_PM_IO_PORT_PIN4
//#define HCI_EXT_PM_IO_PORT_PIN5                        LL_EXT_PM_IO_PORT_PIN5
//#define HCI_EXT_PM_IO_PORT_PIN6                        LL_EXT_PM_IO_PORT_PIN6
//#define HCI_EXT_PM_IO_PORT_PIN7                        LL_EXT_PM_IO_PORT_PIN7
//
#define HCI_EXT_PER_RESET                              LL_EXT_PER_RESET
#define HCI_EXT_PER_READ                               LL_EXT_PER_READ
//
#define HCI_EXT_HALT_DURING_RF_DISABLE                 LL_EXT_HALT_DURING_RF_DISABLE
#define HCI_EXT_HALT_DURING_RF_ENABLE                  LL_EXT_HALT_DURING_RF_ENABLE
//
#define HCI_EXT_SET_USER_REVISION                      LL_EXT_SET_USER_REVISION
#define HCI_EXT_READ_BUILD_REVISION                    LL_EXT_READ_BUILD_REVISION
//
#define HCI_EXT_RESET_SYSTEM_HARD                      LL_EXT_RESET_SYSTEM_HARD
#define HCI_EXT_RESET_SYSTEM_SOFT                      LL_EXT_RESET_SYSTEM_SOFT
//
#define HCI_EXT_DISABLE_OVERLAPPED_PROCESSING          LL_EXT_DISABLE_OVERLAPPED_PROCESSING
#define HCI_EXT_ENABLE_OVERLAPPED_PROCESSING           LL_EXT_ENABLE_OVERLAPPED_PROCESSING
//
#define HCI_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT        LL_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT
#define HCI_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT         LL_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT

/*
** HCI Event Parameters
*/

// HCI Link Type for Buffer Overflow
#define HCI_LINK_TYPE_SCO_BUFFER_OVERFLOW              0
#define HCI_LINK_TYPE_ACL_BUFFER_OVERFLOW              1

/*******************************************************************************
 * TYPEDEFS
 */

typedef uint8 hciStatus_t;

/*
** LE Events
*/

// LE Connection Complete Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint8  status;
  uint16 connectionHandle;
  uint8  role;
  uint8  peerAddrType;
  uint8  peerAddr[B_ADDR_LEN];
  uint16 connInterval;
  uint16 connLatency;
  uint16 connTimeout;
  uint8  clockAccuracy;
} hciEvt_BLEConnComplete_t;

//// LE Connection Complete Event
//typedef struct
//{
//  osal_event_hdr_t  hdr;
//  uint8  BLEEventCode;
//  uint8  status;
//  uint16 connectionHandle;
//  uint8  role;
//  uint8  peerAddrType;
//  uint8  peerAddr[B_ADDR_LEN];
//  uint8  localRpaAddr[B_ADDR_LEN];
//  uint8  peerRpaAddr[B_ADDR_LEN];
//  uint16 connInterval;
//  uint16 connLatency;
//  uint16 connTimeout;
//  uint8  clockAccuracy;
//} hciEvt_BLEEnhConnComplete_t;

// LE Advertising Report Event
typedef struct
{
  uint8  eventType;                       // advertisment or scan response event type
  uint8  addrType;                        // public or random address type
  uint8  addr[B_ADDR_LEN];                // device address
  uint8  dataLen;                         // length of report data
  uint8  rspData[B_MAX_ADV_LEN];          // report data given by dataLen
  int8   rssi;                            // report RSSI
} hciEvt_DevInfo_t;

typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint8  numDevices;
  hciEvt_DevInfo_t* devInfo;              // pointer to the array of devInfo
} hciEvt_BLEAdvPktReport_t;

// LE Connection Update Complete Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint8  status;
  uint16 connectionHandle;
  uint16 connInterval;
  uint16 connLatency;
  uint16 connTimeout;
} hciEvt_BLEConnUpdateComplete_t;

// LE Read Remote Used Features Complete Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint8  status;
  uint16 connectionHandle;
  uint8  features[8];
} hciEvt_BLEReadRemoteFeatureComplete_t;

// LE Encryption Change Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint16 connHandle;
  uint8  reason;
  uint8  encEnable;
} hciEvt_EncryptChange_t;

// LE Long Term Key Requested Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint16 connHandle;
  uint8  random[B_RANDOM_NUM_SIZE];
  uint16 encryptedDiversifier;
} hciEvt_BLELTKReq_t;

// LE DATE LENGTH CHANGE Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint16 connHandle;
  uint16 MaxTxOctets;
  uint16 MaxTxTime;
  uint16 MaxRxOctets;
  uint16 MaxRxTime;
} hciEvt_BLEDataLenChange_t;

// LE PHY UPDATE Complete Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
  uint8  status;
  uint16 connHandle;
  uint8  txPhy;
  uint8  rxPhy;
} hciEvt_BLEPhyUpdateComplete_t;

// LE PHY UPDATE Complete Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  BLEEventCode;
} hciEvt_BLEEvent_Hdr_t;

// Number of Completed Packets Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  numHandles;
  uint16 *pConnectionHandle;              // pointer to the connection handle array
  uint16 *pNumCompletedPackets;           // pointer to the number of completed packets array
} hciEvt_NumCompletedPkt_t;

// Command Complete Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  numHciCmdPkt;                    // number of HCI Command Packet
  uint16 cmdOpcode;
  uint8  *pReturnParam;                    // pointer to the return parameter
} hciEvt_CmdComplete_t;

// Command Status Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  cmdStatus;
  uint8  numHciCmdPkt;
  uint16 cmdOpcode;
} hciEvt_CommandStatus_t;

// Hardware Error Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8 hardwareCode;
} hciEvt_HardwareError_t;

// Disconnection Complete Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8  status;
  uint16 connHandle;                      // connection handle
  uint8  reason;
} hciEvt_DisconnComplete_t;

// Data Buffer Overflow Event
typedef struct
{
  osal_event_hdr_t  hdr;
  uint8 linkType;                         // synchronous or asynchronous buffer overflow
} hciEvt_BufferOverflow_t;

// Data structure for HCI Command Complete Event Return Parameter
typedef struct
{
  uint8  status;
  uint16 dataPktLen;
  uint8  numDataPkts;
} hciRetParam_LeReadBufSize_t;

typedef struct
{
  osal_event_hdr_t hdr;
  uint8            *pData;
} hciPacket_t;

typedef struct
{
  osal_event_hdr_t hdr;
  uint8  pktType;
  uint16 connHandle;
  uint8  pbFlag;
  uint16 pktLen;
  uint8  *pData;
} hciDataPacket_t;

// OSAL HCI_DATA_EVENT message format. This message is used to forward incoming
// data messages up to an application
typedef struct
{
  osal_event_hdr_t hdr;                   // OSAL event header
  uint16 connHandle;                      // connection handle
  uint8  pbFlag;                          // data packet boundary flag
  uint16 len;                             // length of data packet
  uint8  *pData;                          // data packet given by len
} hciDataEvent_t;




/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*
** HCI Support Functions
*/

/*******************************************************************************
 * @fn          HCI_bm_alloc API
 *
 * @brief       This API is used to allocate memory using buffer management.
 *
 *              Note: This function should never be called by the application.
 *                    It is only used by HCI and L2CAP_bm_alloc.
 *
 * input parameters
 *
 * @param       size - Number of bytes to allocate from the heap.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Pointer to buffer, or NULL.
 */
extern void *HCI_bm_alloc( uint16 size );


/*******************************************************************************
 * @fn          HCI_ValidConnTimeParams API
 *
 * @brief       This API is used to check that the connection time parameter
 *              ranges are valid, and that the connection time parameter
 *              combination is valid.
 *
 *              Note: Only connIntervalMax is used as part of the time parameter
 *                    combination check.
 *
 * input parameters
 *
 * @param       connIntervalMin - Minimum connection interval.
 * @param       connIntervalMax - Maximum connection interval.
 * @param       connLatency     - Connection slave latency.
 * @param       connTimeout     - Connection supervision timeout.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      TRUE:  Connection time parameter check is valid.
 *              FALSE: Connection time parameter check is invalid.
 */
extern uint8 HCI_ValidConnTimeParams( uint16 connIntervalMin,
                                      uint16 connIntervalMax,
                                      uint16 connLatency,
                                      uint16 connTimeout );


/*******************************************************************************
 * @fn          HCI_TestAppTaskRegister
 *
 * @brief       HCI vendor specific registration for HCI Test Application.
 *
 * input parameters
 *
 * @param       taskID - The HCI Test Application OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_TestAppTaskRegister( uint8 taskID );


/*******************************************************************************
 * @fn          HCI_GAPTaskRegister
 *
 * @brief       HCI vendor specific registration for Host GAP.
 *
 * input parameters
 *
 * @param       taskID - The Host GAP OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_GAPTaskRegister( uint8 taskID );


/*******************************************************************************
 *
 * @fn          HCI_L2CAPTaskRegister
 *
 * @brief       HCI vendor specific registration for Host L2CAP.
 *
 * input parameters
 *
 * @param       taskID - The Host L2CAP OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 *
 */
extern void HCI_L2CAPTaskRegister( uint8 taskID );


/*******************************************************************************
 * @fn          HCI_SMPTaskRegister
 *
 * @brief       HCI vendor specific registration for Host SMP.
 *
 * input parameters
 *
 * @param       taskID - The Host SMP OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_SMPTaskRegister( uint8 taskID );


/*******************************************************************************
 * @fn          HCI_ExtTaskRegister
 *
 * @brief       HCI vendor specific registration for Host extended commands.
 *
 * input parameters
 *
 * @param       taskID - The Host Extended Command OSAL task identifer.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void HCI_ExtTaskRegister( uint8 taskID );


/*******************************************************************************
 * @fn          HCI_SendDataPkt API
 *
 * @brief       This API is used to send a ACL data packet over a connection.
 *
 *              Note: Empty packets are not sent.
 *
 *              Related Events: HCI_NumOfCompletedPacketsEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection ID (handle).
 * @param       pbFlag     - Packet Boundary Flag.
 * @param       pktLen     - Number of bytes of data to transmit.
 * @param       *pData     - Pointer to data buffer to transmit.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_SendDataPkt( uint16 connHandle,
                                    uint8  pbFlag,
                                    uint16 pktLen,
                                    uint8  *pData );



/*
** HCI API
*/

/*******************************************************************************
 * @fn          HCI_DisconnectCmd API
 *
 * @brief       This BT API is used to terminate a connection.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              DisconnectEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       reason     - Reason for disconnection:
 *                           HCI_DISCONNECT_AUTH_FAILURE,
 *                           HCI_DISCONNECT_REMOTE_USER_TERM,
 *                           HCI_DISCONNECT_REMOTE_DEV_POWER_OFF,
 *                           HCI_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE,
 *                           HCI_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED
 *                           HCI_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_DisconnectCmd( uint16 connHandle,
                                      uint8  reason );


/*******************************************************************************
 * @fn          HCI_ReadRemoteVersionInfoCmd API
 *
 * @brief       This BT API is used to request version information from the
 *              the remote device in a connection.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              ReadRemoteVersionInfoEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadRemoteVersionInfoCmd( uint16 connHandle );



/*******************************************************************************
 * @fn          HCI_SetEventMaskCmd API
 *
 * @brief       This BT API is used to set the HCI event mask, which is used to
 *              determine which events are supported.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       pMask - Pointer to an eight byte event mask.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_SetEventMaskCmd( uint8 *pMask );


/*******************************************************************************
 * @fn          HCI_Reset API
 *
 * @brief       This BT API is used to reset the Link Layer.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ResetCmd( void );



/*******************************************************************************
 * @fn          HCI_ReadTransmitPowerLevelCmd API
 *
 * @brief       This BT API is used to read the transmit power level.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       txPwrType  - HCI_READ_CURRENT_TX_POWER_LEVEL,
 *                           HCI_READ_MAXIMUM_TX_POWER_LEVEL
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadTransmitPowerLevelCmd( uint16 connHandle,
                                                  uint8  txPwrType );


/*******************************************************************************
 * @fn          HCI_SetControllerToHostFlowCtrlCmd API
 *
 * @brief       This BT API is used by the Host to turn flow control on or off
 *              for data sent from the Controller to Host.
 *
 *              Note: This command is currently not supported.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       flowControlEnable - HCI_CTRL_TO_HOST_FLOW_CTRL_OFF,
 *                                  HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_OFF,
 *                                  HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_OFF_SYNCH_ON,
 *                                  HCI_CTRL_TO_HOST_FLOW_CTRL_ACL_ON_SYNCH_ON
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_SetControllerToHostFlowCtrlCmd( uint8 flowControlEnable );


/*******************************************************************************
 * @fn          HCI_HostBufferSizeCmd API
 *
 * @brief       This BT API is used by the Host to notify the Controller of the
 *              maximum size ACL buffer size the Controller can send to the
 *              Host.
 *
 *              Note: This command is currently ignored by the Controller. It
 *                    is assumed that the Host can always handle the maximum
 *                    BLE data packet size.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       hostAclPktLen        - Host ACL data packet length.
 * @param       hostSyncPktLen       - Host SCO data packet length .
 * @param       hostTotalNumAclPkts  - Host total number of ACL data packets.
 * @param       hostTotalNumSyncPkts - Host total number of SCO data packets.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_HostBufferSizeCmd( uint16 hostAclPktLen,
                                          uint8  hostSyncPktLen,
                                          uint16 hostTotalNumAclPkts,
                                          uint16 hostTotalNumSyncPkts );


/*******************************************************************************
 * @fn          HCI_HostNumCompletedPktCmd API
 *
 * @brief       This BT API is used by the Host to notify the Controller of the
 *              number of HCI data packets that have been completed for each
 *              connection handle since this command was previously sent to the
 *              controller.
 *
 *              The Host_Number_Of_Conpleted_Packets command is a special
 *              command. No event is normally generated after the command
 *              has completed. The command should only be issued by the
 *              Host if flow control in the direction from controller to
 *              the host is on and there is at least one connection, or
 *              if the controller is in local loopback mode.
 *
 *              Note: It is assumed that there will be at most only one handle.
 *                    Even if more than one handle is provided, the Controller
 *                    does not track Host buffers as a function of connection 
 *                    handles (and isn't required to do so).
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       numHandles       - Number of connection handles.
 * @param       connHandles      - Array of connection handles.
 * @param       numCompletedPkts - Array of number of completed packets.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_HostNumCompletedPktCmd( uint8  numHandles,
                                               uint16 *connHandles,
                                               uint16 *numCompletedPkts );


/*******************************************************************************
 * @fn          HCI_ReadLocalVersionInfoCmd API
 *
 * @brief       This BT API is used to read the local version information.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadLocalVersionInfoCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadLocalSupportedCommandsCmd API
 *
 * @brief       This BT API is used to read the locally supported commands.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadLocalSupportedCommandsCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadLocalSupportedFeaturesCmd API
 *
 * @brief       This BT API is used to read the locally supported features.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadLocalSupportedFeaturesCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadBDADDRCmd API
 *
 * @brief       This BT API is used to read this device's BLE address (BDADDR).
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadBDADDRCmd( void );


/*******************************************************************************
 * @fn          HCI_ReadRssiCmd API
 *
 * @brief       This BT API is used to read the RSSI of the last packet
 *              received on a connection given by the connection handle. If
 *              the Receiver Modem test is running (HCI_EXT_ModemTestRx), then
 *              the RF RSSI for the last received data will be returned. If
 *              there is no RSSI value, then HCI_RSSI_NOT_AVAILABLE will be
 *              returned.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_ReadRssiCmd( uint16 connHandle );

/*
** HCI Low Energy Commands
*/

/*******************************************************************************
 * @fn          HCI_LE_SetEventMaskCmd API
 *
 * @brief       This LE API is used to set the HCI LE event mask, which is used
 *              to determine which LE events are supported.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param        pEventMask - Pointer to LE event mask of 8 bytes.

 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetEventMaskCmd( uint8 *pEventMask );


/*******************************************************************************
 * @fn          HCI_LE_ReadBufSizeCmd API
 *
 * @brief       This LE API is used by the Host to determine the maximum ACL
 *              data packet size allowed by the Controller.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadBufSizeCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_ReadLocalSupportedFeaturesCmd API
 *
 * @brief       This LE API is used to read the LE locally supported features.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadLocalSupportedFeaturesCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_SetRandomAddressCmd API
 *
 * @brief       This LE API is used to set this device's Random address.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       pRandAddr - Pointer to random address.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetRandomAddressCmd( uint8 *pRandAddr );



/*******************************************************************************
 * @fn          HCI_LE_SetAdvParamCmd API
 *
 * @brief       This LE API is used to set the Advertising parameters.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       advIntervalMin  - Minimum allowed advertising interval.
 * @param       advIntervalMax  - Maximum allowed advertising interval.
 * @param       advType         - HCI_CONNECTABLE_UNDIRECTED_ADV,
 *                                HCI_CONNECTABLE_DIRECTED_HDC_ADV,
 *                                HCI_SCANNABLE_UNDIRECTED,
 *                                HCI_NONCONNECTABLE_UNDIRECTED_ADV
 *                                HCI_CONNECTABLE_DIRECTED_LDC_ADV
 * @param       ownAddrType     - HCI_PUBLIC_DEVICE_ADDRESS,
 *                                HCI_RANDOM_DEVICE_ADDRESS
 * @param       directAddrType  - HCI_PUBLIC_DEVICE_ADDRESS,
 *                                HCI_RANDOM_DEVICE_ADDRESS
 * @param       directAddr      - Pointer to address of device when using
 *                                directed advertising.
 * @param       advChannelMap   - HCI_ADV_CHAN_37,
 *                                HCI_ADV_CHAN_38,
 *                                HCI_ADV_CHAN_39,
 *                                HCI_ADV_CHAN_37 | HCI_ADV_CHAN_38,
 *                                HCI_ADV_CHAN_37 | HCI_ADV_CHAN_39,
 *                                HCI_ADV_CHAN_38 | HCI_ADV_CHAN_39,
 *                                HCI_ADV_CHAN_ALL
 * @param       advFilterPolicy - HCI_ADV_WL_POLICY_ANY_REQ,
 *                                HCI_ADV_WL_POLICY_WL_SCAN_REQ,
 *                                HCI_ADV_WL_POLICY_WL_CONNECT_REQ,
 *                                HCI_ADV_WL_POLICY_WL_ALL_REQ
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetAdvParamCmd( uint16 advIntervalMin,
                                          uint16 advIntervalMax,
                                          uint8  advType,
                                          uint8  ownAddrType,
                                          uint8  directAddrType,
                                          uint8  *directAddr,
                                          uint8  advChannelMap,
                                          uint8  advFilterPolicy );


/*******************************************************************************
 * @fn          HCI_LE_SetAdvDataCmd API
 *
 * @brief       This LE API is used to set the Advertising data.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       dataLen - Length of Advertising data.
 * @param       pData   - Pointer to Advertising data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetAdvDataCmd( uint8 dataLen,
                                         uint8 *pData );


/*******************************************************************************
 * @fn          HCI_LE_SetScanRspDataCmd API
 *
 * @brief       This LE API is used to set the Advertising Scan Response data.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       dataLen - Length of Scan Response data.
 * @param       pData   - Pointer to Scan Response data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetScanRspDataCmd( uint8 dataLen,
                                             uint8 *pData );


/*******************************************************************************
 * @fn          HCI_LE_SetAdvEnableCmd API
 *
 * @brief       This LE API is used to turn Advertising on or off.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       advEnable - HCI_ENABLE_ADV, HCI_DISABLE_ADV
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetAdvEnableCmd( uint8 advEnable );


/*******************************************************************************
 * @fn          HCI_LE_ReadAdvChanTxPowerCmd API
 *
 * @brief       This LE API is used to read transmit power when Advertising.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadAdvChanTxPowerCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_SetScanParamCmd API
 *
 * @brief       This LE API is used to set the Scan parameters.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       scanType     - HCI_SCAN_PASSIVE, HCI_SCAN_ACTIVE
 * @param       scanInterval - Time between scan events.
 * @param       scanWindow   - Time of scan before scan event ends.
 *                             Note: When the scanWindow equals the scanInterval
 *                                   then scanning is continuous.
 * @param       ownAddrType  - This device's address.
 * @param       filterPolicy - HCI_SCAN_PASSIVE, HCI_SCAN_ACTIVE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetScanParamCmd( uint8  scanType,
                                           uint16 scanInterval,
                                           uint16 scanWindow,
                                           uint8  ownAddrType,
                                           uint8  filterPolicy );


/*******************************************************************************
 * @fn          HCI_LE_SetScanEnableCmd API
 *
 * @brief       This LE API is used to turn Scanning on or off.
 *
 *              Related Events: HCI_CommandCompleteEvent,
 *                              AdvReportEvent
 *
 * input parameters
 *
 * @param       scanEnable       - HCI_SCAN_START, HCI_SCAN_STOP
 * @param       filterDuplicates - HCI_FILTER_REPORTS_ENABLE,
 *                                 HCI_FILTER_REPORTS_DISABLE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetScanEnableCmd( uint8 scanEnable,
                                            uint8 filterDuplicates );

/*******************************************************************************
 * @fn          HCI_LE_ConnUpdateCmd API
 *
 * @brief       This LE API is used to update the connection parameters.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              ConnectionUpdateCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle       - Time between Init scan events.
 * @param       connIntervalMin  - Minimum allowed connection interval.
 * @param       connIntervalMax  - Maximum allowed connection interval.
 * @param       connLatency      - Number of skipped events (slave latency).
 * @param       connTimeout      - Connection supervision timeout.
 * @param       minLen           - Info parameter about min length of conn.
 * @param       maxLen           - Info parameter about max length of conn.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ConnUpdateCmd( uint16 connHandle,
                                         uint16 connIntervalMin,
                                         uint16 connIntervalMax,
                                         uint16 connLatency,
                                         uint16 connTimeout,
                                         uint16 minLen,
                                         uint16 maxLen );


/*******************************************************************************
 * @fn          HCI_LE_SetHostChanClassificationCmd API
 *
 * @brief       This LE API is used to update the current data channel map.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       chanMap - Pointer to the new channel map.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_SetHostChanClassificationCmd( uint8 *chanMap );


/*******************************************************************************
 * @fn          HCI_LE_ReadChannelMapCmd API
 *
 * @brief       This LE API is used to read a connection's data channel map.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadChannelMapCmd( uint16 connHandle );

/*******************************************************************************
 * @fn          HCI_LE_ReadRemoteUsedFeaturesCmd API
 *
 * @brief       This LE API is used to read the remote device's used features.
 *
 *              Related Events: HCI_CommandStatusEvent,
 *                              ReadRemoteUsedFeaturesCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadRemoteUsedFeaturesCmd( uint16 connHandle );


/*******************************************************************************
 * @fn          HCI_LE_EncryptCmd API
 *
 * @brief       This LE API is used to perform an encryption using AES128.
 *
 *              Note: Input parameters are ordered MSB..LSB.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       key       - Pointer to 16 byte encryption key.
 * @param       plainText - Pointer to 16 byte plaintext data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_EncryptCmd( uint8 *key,
                                      uint8 *plainText );


/*******************************************************************************
 * @fn          HCI_LE_RandCmd API
 *
 * @brief       This LE API is used to generate a random number.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_RandCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_LtkReqReplyCmd API
 *
 * @brief       This LE API is used by the Host to send to the Controller a
 *              positive LTK reply.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connection handle.
 * @param       ltk        - Pointer to 16 byte Long Term Key.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_LtkReqReplyCmd( uint16 connHandle,
                                          uint8  *ltk );

/*******************************************************************************
 * @fn          HCI_LE_LtkReqNegReplyCmd API
 *
 * @brief       This LE API is used by the Host to send to the Controller a
 *              negative LTK reply.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       connHandle - Connectin handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_LtkReqNegReplyCmd( uint16 connHandle );


/*******************************************************************************
 * @fn          HCI_LE_ReadSupportedStatesCmd API
 *
 * @brief       This LE API is used to read the Controller's supported states.
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReadSupportedStatesCmd( void );


/*******************************************************************************
 * @fn          HCI_LE_ReceiverTestCmd API
 *
 * @brief       This LE API is used to start the receiver Direct Test Mode test.
 *
 *              Note: A HCI reset should be issued when done using DTM!
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       rxFreq - Rx RF frequency:
 *                       k=0..HCI_DTM_NUMBER_RF_CHANS-1, where: F=2402+(k*2MHz)
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_ReceiverTestCmd( uint8 rxFreq );


/*******************************************************************************
 * @fn          HCI_LE_TransmitterTestCmd API
 *
 * @brief       This LE API is used to start the transmit Direct Test Mode test.
 *
 *              Note: The BLE device is to transmit at maximum power!
 *
 *              Note: A HCI reset should be issued when done using DTM!
 *
 * input parameters
 *
 * @param       txFreq      - Tx RF frequency:
 *                            k=0..HCI_DTM_NUMBER_RF_CHANS-1, where:
 *                            F=2402+(k*2MHz)
 * @param       dataLen     - Test data length in bytes:
 *                            0..HCI_DIRECT_TEST_MAX_PAYLOAD_LEN
 * @param       payloadType - Type of packet payload, per Direct Test Mode spec:
 *                            HCI_DIRECT_TEST_PAYLOAD_PRBS9,
 *                            HCI_DIRECT_TEST_PAYLOAD_0x0F,
 *                            HCI_DIRECT_TEST_PAYLOAD_0x55,
 *                            HCI_DIRECT_TEST_PAYLOAD_PRBS15,
 *                            HCI_DIRECT_TEST_PAYLOAD_0xFF,
 *                            HCI_DIRECT_TEST_PAYLOAD_0x00,
 *                            HCI_DIRECT_TEST_PAYLOAD_0xF0,
 *                            HCI_DIRECT_TEST_PAYLOAD_0xAA
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_TransmitterTestCmd( uint8 txFreq,
                                              uint8 dataLen,
                                              uint8 pktPayload );


/*******************************************************************************
 * @fn          HCI_LE_TestEndCmd API
 *
 * @brief       This LE API is used to end the Direct Test Mode test.
 *
 *              Note: A HCI reset should be issued when done using DTM!
 *
 *              Related Events: HCI_CommandCompleteEvent
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      hciStatus_t
 */
extern hciStatus_t HCI_LE_TestEndCmd( void );

/*******************************************************************************
*/
hciStatus_t HCI_LE_SetDataLengthCmd( uint16 connHandle,
                                       uint16 TxOctets,
                                       uint16 TxTime );


/*******************************************************************************
*/
hciStatus_t HCI_LE_ReadMaxDataLengthCmd(void);

/*******************************************************************************
 * This LE API is used to read Suggested Default max Data length
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadSuggestedDefaultDataLengthCmd(void);

/*******************************************************************************
 * This LE API is used to write Suggested Default Data length
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_WriteSuggestedDefaultDataLengthCmd(uint16 suggestedMaxTxOctets,uint16 suggestedMaxTxTime);


/*******************************************************************************
 * This LE API is used to set DefaultPhyMode
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetDefaultPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy);


/*******************************************************************************
 * This LE API is used to Set PHY Mode
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_SetPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions);

/*******************************************************************************
 * This LE API is used to Read PHY Mode
 *
 * Public function defined in hci.h.
 */
hciStatus_t HCI_LE_ReadPhyMode( uint16 connId);



#ifdef __cplusplus
}
#endif

#endif /* HCI_H */
