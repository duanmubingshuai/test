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
  Filename:       ll.h
  Revised:         
  Revision:        

  Description:    This file contains the Link Layer (LL) API for the Bluetooth
                  Low Energy (BLE) Controller. It provides the defines, types,
                  and functions for all supported Bluetooth Low Energy (BLE)
                  commands.

                  This API is based on the Bluetooth Core Specification,
                  V4.0.0, Vol. 6.

   
*******************************************************************************/

#ifndef LL_H
#define LL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "timer.h"   
#include "ll_def.h"
/*******************************************************************************
 * MACROS
 */

// check if connection parameter ranges for CI (min/max), SL, and LSTO are valid
#define LL_INVALID_CONN_TIME_PARAM( ciMin, ciMax, sl, lsto )                   \
  (((ciMin) < LL_CONN_INTERVAL_MIN) ||                                         \
   ((ciMin) > LL_CONN_INTERVAL_MAX) ||                                         \
   ((ciMax) < LL_CONN_INTERVAL_MIN) ||                                         \
   ((ciMax) > LL_CONN_INTERVAL_MAX) ||                                         \
   ((ciMax) < (ciMin))              ||                                         \
   ((sl)    > LL_SLAVE_LATENCY_MAX) ||                                         \
   ((lsto)  < LL_CONN_TIMEOUT_MIN)  ||                                         \
   ((lsto)  > LL_CONN_TIMEOUT_MAX))

// check if the CI/SL/LSTO combination is valid
// based on: LSTO > (1 + Slave Latency) * (Connection Interval * 2)
// Note: The CI * 2 requirement based on ESR05 V1.0, Erratum 3904.
// Note: LSTO time is normalized to units of 1.25ms (i.e. 10ms = 8 * 1.25ms).
#define LL_INVALID_CONN_TIME_PARAM_COMBO( ci, sl, lsto )                       \
  ((uint32)((lsto)*8) <= ((uint32)(1+(sl)) * (uint32)((ci)*2)))

#define   LL_TIME_DELTA(T1, T2)   ((T2 >= T1) ? (T2 - T1) : (BASE_TIME_UNITS - T1 + T2))


/*******************************************************************************
 * CONSTANTS
 */

/*
** LL API Status Codes
**
** Note: These status values map directly to the HCI Error Codes.
**       Per the Bluetooth Core Specification, V4.0.0, Vol. 2, Part D.
*/
#define LL_STATUS_SUCCESS                              0x00 // Success
#define LL_STATUS_ERROR_UNKNOWN_CONN_HANDLE            0x02 // Unknown Connection Identifier
#define LL_STATUS_ERROR_INACTIVE_CONNECTION            0x02 // Unknown Connection Identifier for now; may be needed for multiple connections
#define LL_STATUS_ERROR_AUTH_FAILURE                   0x05 // Authentication Failure
#define LL_STATUS_ERROR_PIN_OR_KEY_MISSING             0x06 // Pin or Key Missing
#define LL_STATUS_ERROR_OUT_OF_CONN_RESOURCES          0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_OUT_OF_TX_MEM                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_OUT_OF_RX_MEM                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_OUT_OF_HEAP                    0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_WL_TABLE_FULL                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_RL_TABLE_FULL                  0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_PAL_TABLE_FULL                 0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_TX_DATA_QUEUE_FULL             0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_TX_DATA_QUEUE_EMPTY            0x07 // Memory Capacity Exceeded
#define LL_STATUS_ERROR_CONNECTION_TIMEOUT             0x08 // Connection Timeout
#define LL_STATUS_ERROR_CONNECTION_LIMIT_EXCEEDED      0x09 // Connection Limit Exceeded
#define LL_STATUS_ERROR_COMMAND_DISALLOWED             0x0C // Command Disallowed
#define LL_STATUS_ERROR_DUE_TO_LIMITED_RESOURCES       0x0D // Command Rejected Due To Limited Resources
#define LL_STATUS_ERROR_DUE_TO_DELAYED_RESOURCES       0x0D // Command Delayed Due To Limited Resources
#define LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED          0x11 // Unsupported Feature or Parameter Value
#define LL_STATUS_ERROR_UNEXPECTED_PARAMETER           0x12 // Invalid HCI Command Parameters
#define LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION      0x12 // Invalid HCI Command Parameters
#define LL_STATUS_ERROR_BAD_PARAMETER                  0x12 // Invalid HCI Command Parameters or 0x30: Parameter Out of Mandatory Range?
#define LL_STATUS_ERROR_UNKNOWN_ADV_EVT_TYPE           0x12 // Invalid HCI Command Parameters or 0x30: Parameter Out of Mandatory Range?
#define LL_STATUS_ERROR_PEER_TERM                      0x13 // Remote User Terminated Connection
#define LL_STATUS_ERROR_PEER_DEVICE_TERM_LOW_RESOURCES 0x14 // Remote Device Terminated Connection Due To Low Resources
#define LL_STATUS_ERROR_PEER_DEVICE_TERM_POWER_OFF     0x15 // Remote Device Terminated Connection Due To Power Off
#define LL_STATUS_ERROR_HOST_TERM                      0x16 // Connection Terminated By Local Host
#define LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE     0x1A // Unsupported Remote Feature

// 2020-01-23 add error Code
#define LL_STATUS_ERROR_INVALID_LMP_LL_PARAMETER	   0x1E

#define LL_STATUS_ERROR_WL_ENTRY_NOT_FOUND             0x1F // Unspecified Error
#define LL_STATUS_ERROR_WL_TABLE_EMPTY                 0x1F // Unspecified Error
#define LL_STATUS_ERROR_RL_ENTRY_NOT_FOUND             0x1F // Unspecified Error
#define LL_STATUS_ERROR_RL_TABLE_EMPTY                 0x1F // Unspecified Error
#define LL_STATUS_ERROR_RNG_FAILURE                    0x1F // Unspecified Error
#define LL_STATUS_ERROR_DISCONNECT_IMMEDIATE           0x1F // Unspecified Error
#define LL_STATUS_ERROR_DATA_PACKET_QUEUED             0x1F // Unspecified Error

// 2020-01-23 add error Code
#define LL_STATUS_ERROR_UNSUPPORT_LMP_LL_PARAMETER	   0x20

#define LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE          0x21 // Role Change Not Allowed
#define LL_STATUS_ERROR_LL_TIMEOUT                     0x22 // Link Layer Response Timeout
#define LL_STATUS_ERROR_LL_TIMEOUT_HOST                0x22 // Link Layer Response Timeout
#define LL_STATUS_ERROR_LL_TIMEOUT_PEER                0x22 // Link Layer Response Timeout
#define LL_STATUS_ERROR_LL_PROCEDURE_COLLISION         0x23 // Link Layer Procedure collision


#define LL_STATUS_ERROR_INSTANT_PASSED                 0x28 // Instant Passed
#define LL_STATUS_ERROR_INSTANT_PASSED_HOST            0x28 // Instant Passed
#define LL_STATUS_ERROR_INSTANT_PASSED_PEER            0x28 // Instant Passed
#define LL_STATUS_ERROR_KEY_PAIRING_NOT_SUPPORTED      0x29 // Pairing With Unit Key Not Supported
#define LL_STATUS_ERROR_DIFF_TRANSACTION_COLLISION     0x2A // Link Layer collision in same procedure


#define LL_STATUS_ERROR_NO_ADV_CHAN_FOUND              0x30 // Parameter Out Of Mandatory Range
#define LL_STATUS_ERROR_PARAM_OUT_OF_RANGE             0x30 // Parameter Out Of Mandatory Range
#define LL_STATUS_ERROR_UPDATE_CTRL_PROC_PENDING       0x3A // Controller Busy
#define LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE       0x3A // Controller Busy
#define LL_STATUS_ERROR_VER_INFO_REQ_ALREADY_PENDING   0x3A // Controller Busy
#define LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL     0x3B // Unacceptable Connection Interval
#define LL_STATUS_ERROR_DIRECTED_ADV_TIMEOUT           0x3C // Directed Advertising Timeout
#define LL_STATUS_ERROR_CONN_TERM_DUE_TO_MIC_FAILURE   0x3D // Connection Terminated Due To MIC Failure
#define LL_STATUS_ERROR_CONN_FAILED_TO_BE_ESTABLISHED  0x3E // Connection Failed To Be Established
#define LL_STATUS_ERROR_SYNC_FAILED_TO_BE_ESTABLISHED  0x3E // Connection Failed To Be Established
#define LL_STATUS_ERROR_CONN_TIMING_FAILURE            0x3F // MAC Connection Failed
#define LL_STATUS_ERROR_COARSE_CLK_ADJ_REJECT          0x40 // Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock Dragging
#define LL_STATUS_ERROR_TYPE0_SUBMAP_NOT_DEF           0x41 // Type0 Submap Not Defined
#define LL_STATUS_ERROR_UNKNOWN_ADV_ID                 0x42 // Unknown Advertising Identifier
#define LL_STATUS_ERROR_LIMIT_REACHED                  0x43 // Limit Reached
#define LL_STATUS_ERROR_OP_CANCEL_BY_HOST              0x44 // Operation Cancelled by Host
#define LL_STATUS_ERROR_PACKET_TOO_LONG                0x45 // Packet Too Long

// Internal
#define LL_STATUS_DISABLE_LATENCY_INACTIVE_CONN        0x81
#define LL_STATUS_DISABLE_LATENCY_DISABLED             0x82
#define LL_STATUS_DISABLE_LATENCY_PENDING              0x83
#define LL_STATUS_DISABLE_LATENCY_MISS_EVT             0x84

#define LL_STATUS_DISABLE_LATENCY_FAIL                 0x8F 
#define LL_STATUS_WARNING_WAITING_LLIRQ                0xFE // only used internally, so value doesn't matter

#define LL_STATUS_WARNING_TX_DISABLED                  0xFF // only used internally, so value doesn't matter
#define LL_STATUS_WARNING_FLAG_UNCHANGED               0xFF // only used internally, so value doesn't matter

// Encryption Key Request Reason Codes
#define LL_ENC_KEY_REQ_ACCEPTED                        LL_STATUS_SUCCESS
#define LL_ENC_KEY_REQ_REJECTED                        LL_STATUS_ERROR_PIN_OR_KEY_MISSING
#define LL_ENC_KEY_REQ_UNSUPPORTED_FEATURE             LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE

// Disconnect Reason Codes
#define LL_SUPERVISION_TIMEOUT_TERM                    LL_STATUS_ERROR_CONNECTION_TIMEOUT
#define LL_PEER_REQUESTED_TERM                         LL_STATUS_ERROR_PEER_TERM
#define LL_PEER_REQUESTED_LOW_RESOURCES_TERM           LL_STATUS_ERROR_PEER_DEVICE_TERM_LOW_RESOURCES
#define LL_PEER_REQUESTED_POWER_OFF_TERM               LL_STATUS_ERROR_PEER_DEVICE_TERM_POWER_OFF
#define LL_HOST_REQUESTED_TERM                         LL_STATUS_ERROR_HOST_TERM
#define LL_CTRL_PKT_TIMEOUT_TERM                       LL_STATUS_ERROR_LL_TIMEOUT
#define LL_CTRL_PKT_TIMEOUT_HOST_TERM                  LL_STATUS_ERROR_LL_TIMEOUT_HOST
#define LL_CTRL_PKT_TIMEOUT_PEER_TERM                  LL_STATUS_ERROR_LL_TIMEOUT_PEER
#define LL_CTRL_PKT_INSTANT_PASSED_TERM                LL_STATUS_ERROR_INSTANT_PASSED
#define LL_CTRL_PKT_INSTANT_PASSED_HOST_TERM           LL_STATUS_ERROR_INSTANT_PASSED_HOST
#define LL_CTRL_PKT_INSTANT_PASSED_PEER_TERM           LL_STATUS_ERROR_INSTANT_PASSED_PEER
#define LL_UNACCEPTABLE_CONN_INTERVAL_TERM             LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL
#define LL_MIC_FAILURE_TERM                            LL_STATUS_ERROR_CONN_TERM_DUE_TO_MIC_FAILURE
#define LL_CONN_ESTABLISHMENT_FAILED_TERM              LL_STATUS_ERROR_CONN_FAILED_TO_BE_ESTABLISHED

// Disconnect API Parameter
#define LL_DISCONNECT_AUTH_FAILURE                     LL_STATUS_ERROR_AUTH_FAILURE
#define LL_DISCONNECT_REMOTE_USER_TERM                 LL_STATUS_ERROR_PEER_TERM
#define LL_DISCONNECT_REMOTE_DEV_LOW_RESOURCES         LL_STATUS_ERROR_PEER_DEVICE_TERM_LOW_RESOURCES
#define LL_DISCONNECT_REMOTE_DEV_POWER_OFF             LL_STATUS_ERROR_PEER_DEVICE_TERM_POWER_OFF
#define LL_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE       LL_STATUS_ERROR_UNSUPPORTED_REMOTE_FEATURE
#define LL_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED        LL_STATUS_ERROR_KEY_PAIRING_NOT_SUPPORTED
#define LL_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL       LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL


// LL Advertiser Events
#define LL_ADV_CONNECTABLE_UNDIRECTED_EVT              0
#define LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT            1  // High Duty Cycle
#define LL_ADV_SCANNABLE_UNDIRECTED_EVT                2
#define LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT           3
#define LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT            4  // Low Duty Cycle

// LL Address Type
#define LL_DEV_ADDR_TYPE_PUBLIC                        0
#define LL_DEV_ADDR_TYPE_RANDOM                        1
#define LL_DEV_ADDR_TYPE_RPA_PUBLIC                    2        // Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving list contains no matching entry, use public address
#define LL_DEV_ADDR_TYPE_RPA_RANDOM                    3        // Controller generates Resolvable Private Address based on the local IRK from resolving list. If resolving list contains no matching entry, use random address from LE_Set_Random_Address.



// Advertiser White List Policy
#define LL_ADV_WL_POLICY_ANY_REQ                       0  // any scan request or connect request
#define LL_ADV_WL_POLICY_WL_SCAN_REQ                   1  // any connect request, white list scan request
#define LL_ADV_WL_POLICY_WL_CONNECT_REQ                2  // any scan request, white list connect request
#define LL_ADV_WL_POLICY_WL_ALL_REQ                    3  // white list scan request and connect request

// Scanner White List Policy
#define LL_SCAN_WL_POLICY_ANY_ADV_PKTS                 0
#define LL_SCAN_WL_POLICY_USE_WHITE_LIST               1
#define LL_SCAN_WL_POLICY_BLE42_1                      2
#define LL_SCAN_WL_POLICY_BLE42_2                      3


// Initiator White List Policy
#define LL_INIT_WL_POLICY_USE_PEER_ADDR                0
#define LL_INIT_WL_POLICY_USE_WHITE_LIST               1

// Black List Control
#define LL_SET_BLACKLIST_DISABLE                       0
#define LL_SET_BLACKLIST_ENABLE                        1

// Advertiser Commands
#define LL_ADV_MODE_OFF                                0
#define LL_ADV_MODE_ON                                 1
#define LL_ADV_MODE_RESERVED                           2

// LL Scan Commands
#define LL_SCAN_STOP                                   0
#define LL_SCAN_START                                  1

// LL Scan Filtering
#define LL_FILTER_REPORTS_DISABLE                      0
#define LL_FILTER_REPORTS_ENABLE                       1

// LL Scan Types
#define LL_SCAN_PASSIVE                                0
#define LL_SCAN_ACTIVE                                 1

// LL Tx Power Types
#define LL_READ_CURRENT_TX_POWER_LEVEL                 0
#define LL_READ_MAX_TX_POWER_LEVEL                     1

// Data Fragmentation Flag
#define LL_DATA_FIRST_PKT_HOST_TO_CTRL                 0
#define LL_DATA_CONTINUATION_PKT                       1
#define LL_DATA_FIRST_PKT_CTRL_TO_HOST                 2

// Connection Complete Role
#define LL_LINK_CONNECT_COMPLETE_MASTER                0
#define LL_LINK_CONNECT_COMPLETE_SLAVE                 1

// Encryption Related
#define LL_ENCRYPTION_OFF                              0
#define LL_ENCRYPTION_ON                               1

// Feature Set Related
#define LL_MAX_FEATURE_SET_SIZE                        8  // in bytes
//
#define LL_FEATURE_RFU                                 0  // all bits in a byte
#define LL_FEATURE_ENCRYPTION                          1  // byte 0, bit 0
#define LL_FEATURE_EXT_REJECT_IND                      4  // byte 0, bit 0
#define LL_FEATURE_DATA_LENGTH_EXTENSION            0x20  // byte 0, bit 0

#define LL_FEATURE_2M_PHY                           0x01  // byte 1, bit 0
#define LL_FEATURE_CODED_PHY                        0x08  // byte 1, bit 3

#define LL_FEATURE_CSA2                             0x40  // byte 1, bit 6

// 2020-01-15 add 
// CODE PHY feature
#define LL_FEATURE_CODE_PHY_IDX						1	  // byte 1
#define LL_FEATURE_CODE_PHY							0x08
// extended advertisingCTE feature 
#define LL_FEATURE_EXT_ADV_IDX						1
#define LL_FEATURE_EXT_ADV							0x10
// periodic advertising 
#define LL_FEATURE_PRD_ADV_IDX						1
#define LL_FEATURE_PRD_ADV							0x20
// Channel selection Algorithm #2
#define LL_FEATURE_CSA2_IDX							1
#define LL_FEATURE_CSA2								0x40

// CTE FEATURE
#define LL_CTE_FEATURE_IDX							2
#define LL_CONN_CTE_REQ								0x02
#define LL_CONN_CTE_RSP								0x04
#define LL_CONNLESS_CTE_TRANSMITER					0x08
#define LL_CONNLESS_CTE_RECEIVER					0x10
#define LL_AOD_SUPPORT								0x20
#define LL_AOA_SUPPORT								0x40


// Receive Flow Control
#define LL_DISABLE_RX_FLOW_CONTROL                     0
#define LL_ENABLE_RX_FLOW_CONTROL                      1

// Direct Test Mode
#define LL_DIRECT_TEST_NUM_RF_CHANS                    40   // PHY_NUM_RF_CHANS
#define LL_DIRECT_TEST_MAX_PAYLOAD_LEN                 37
//
#define LL_DIRECT_TEST_PAYLOAD_PRBS9                   0
#define LL_DIRECT_TEST_PAYLOAD_0x0F                    1
#define LL_DIRECT_TEST_PAYLOAD_0x55                    2
#define LL_DIRECT_TEST_PAYLOAD_PRBS15                  3
#define LL_DIRECT_TEST_PAYLOAD_0xFF                    4
#define LL_DIRECT_TEST_PAYLOAD_0x00                    5
#define LL_DIRECT_TEST_PAYLOAD_0xF0                    6
#define LL_DIRECT_TEST_PAYLOAD_0xAA                    7
#define LL_DIRECT_TEST_PAYLOAD_UNDEFINED               0xFF
//
#define LL_DIRECT_TEST_MODE_TX                         0
#define LL_DIRECT_TEST_MODE_RX                         1
//
#define LL_RF_RSSI_UNDEFINED                           PHY_RSSI_VALUE_INVALID

// Vendor Specific
#define LL_EXT_RX_GAIN_STD                             0
#define LL_EXT_RX_GAIN_HIGH                            1
//
#define LL_EXT_TX_POWER_MINUS_23_DBM                   0
#define LL_EXT_TX_POWER_MINUS_6_DBM                    1
#define LL_EXT_TX_POWER_0_DBM                          2
#define LL_EXT_TX_POWER_4_DBM                          3


//
#define LL_EXT_DISABLE_ONE_PKT_PER_EVT                 0
#define LL_EXT_ENABLE_ONE_PKT_PER_EVT                  1
//
#define LL_EXT_DISABLE_CLK_DIVIDE_ON_HALT              0
#define LL_EXT_ENABLE_CLK_DIVIDE_ON_HALT               1
//
#define LL_EXT_NV_NOT_IN_USE                           0
#define LL_EXT_NV_IN_USE                               1
//
#define LL_EXT_DISABLE_FAST_TX_RESP_TIME               0
#define LL_EXT_ENABLE_FAST_TX_RESP_TIME                1
//
#define LL_EXT_DISABLE_SL_OVERRIDE                     0
#define LL_EXT_ENABLE_SL_OVERRIDE                      1
//
#define LL_EXT_TX_MODULATED_CARRIER                    0
#define LL_EXT_TX_UNMODULATED_CARRIER                  1
//
#define LL_EXT_SET_FREQ_TUNE_DOWN                      0
#define LL_EXT_SET_FREQ_TUNE_UP                        1

//
#define LL_EXT_PER_RESET                               0
#define LL_EXT_PER_READ                                1
//
#define LL_EXT_HALT_DURING_RF_DISABLE                  0
#define LL_EXT_HALT_DURING_RF_ENABLE                   1
//
#define LL_EXT_SET_USER_REVISION                       0
#define LL_EXT_READ_BUILD_REVISION                     1
//
#define LL_EXT_RESET_SYSTEM_DELAY                      100 // in ms
#define LL_EXT_RESET_SYSTEM_HARD                       0
#define LL_EXT_RESET_SYSTEM_SOFT                       1
//
#define LL_EXT_DISABLE_OVERLAPPED_PROCESSING           0
#define LL_EXT_ENABLE_OVERLAPPED_PROCESSING            1
//
#define LL_EXT_DISABLE_NUM_COMPL_PKTS_ON_EVENT         0
#define LL_EXT_ENABLE_NUM_COMPL_PKTS_ON_EVENT          1


/*
** Event Parameters
*/

// Advertising Report Data
#define LL_ADV_RPT_ADV_IND                             LL_ADV_CONNECTABLE_UNDIRECTED_EVT
#define LL_ADV_RPT_ADV_DIRECT_IND                      LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT
#define LL_ADV_RPT_ADV_SCANNABLE_IND                   LL_ADV_SCANNABLE_UNDIRECTED_EVT
#define LL_ADV_RPT_ADV_NONCONN_IND                     LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT
#define LL_ADV_RPT_SCAN_RSP                            (LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT + 1)
#define LL_ADV_RPT_INVALID                             0xFF
//
#define LL_RSSI_NOT_AVAILABLE                          127

// Sleep Clock Accuracy (SCA)
#define LL_SCA_500_PPM                                 0
#define LL_SCA_250_PPM                                 1
#define LL_SCA_150_PPM                                 2
#define LL_SCA_100_PPM                                 3
#define LL_SCA_75_PPM                                  4
#define LL_SCA_50_PPM                                  5
#define LL_SCA_30_PPM                                  6
#define LL_SCA_20_PPM                                  7


// Default SCA
//#define LL_SCA_MASTER_DEFAULT         5         // 50ppm (ordinal value)
#define LL_SCA_MASTER_DEFAULT         0          // 500ppm (ordinal value)
#define LL_SCA_SLAVE_DEFAULT          500        // 500ppm

// LL Advertiser Channels
#define LL_ADV_CHAN_37                  1
#define LL_ADV_CHAN_38                  2
#define LL_ADV_CHAN_39                  4
#define LL_ADV_CHAN_ALL                                (LL_ADV_CHAN_37 | LL_ADV_CHAN_38 | LL_ADV_CHAN_39)
#define LL_ADV_CHAN_MAP_DEFAULT       LL_ADV_CHAN_ALL

#define LL_ADV_CHAN_FIRST               37
#define LL_ADV_CHAN_LAST                39

// max future number of events for an update to parameters or data channel
#define LL_MAX_UPDATE_COUNT_RANGE                32767

// Extended Header Flags
#define LE_EXT_HDR_ADVA_PRESENT_BITMASK         0x01
#define LE_EXT_HDR_TARGETA_PRESENT_BITMASK      0x02
#define LE_EXT_HDR_CTE_INFO_PRESENT_BITMASK     0x04
#define LE_EXT_HDR_ADI_PRESENT_BITMASK          0x08
#define LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK      0x10
#define LE_EXT_HDR_SYNC_INFO_PRESENT_BITMASK    0x20            
#define LE_EXT_HDR_TX_PWR_PRESENT_BITMASK       0x40            
#define LE_EXT_HDR_RFU_PRESENT_BITMASK          0x80            



// extended advertisement Macros
#define LE_ADV_PROP_CONN_BITMASK         0x00000001
#define LE_ADV_PROP_SCAN_BITMASK         0x00000002
#define LE_ADV_PROP_DIRECT_BITMASK       0x00000004
#define LE_ADV_PROP_HI_DC_CONN_BITMASK   0x00000008
#define LE_ADV_PROP_LEGACY_BITMASK       0x00000010
#define LE_ADV_PROP_ANON_BITMASK         0x00000020            // applicable to extended adv only
#define LE_ADV_PROP_TX_POWER_BITMASK     0x00000040            // applicable to extended adv & periodic adv

/*  TODO: update according to spec
	0x00 Intermediate fragment of fragmented extended advertising data
0x01 First fragment of fragmented extended advertising data
0x02 Last fragment of fragmented extended advertising data
0x03 Complete extended advertising data
0x04 Unchanged data (just update the Advertising DID) */

#define BLE_EXT_ADV_OP_INTERM_FRAG        0x00
#define BLE_EXT_ADV_OP_FIRST_FRAG         0x01
#define BLE_EXT_ADV_OP_LAST_FRAG          0x02
#define BLE_EXT_ADV_OP_COMPLETE_DATA      0x03
#define BLE_EXT_ADV_OP_UNCHANGED_DATA     0x04

#define BLE_EXT_ADV_FRAG_ENABLED          0x00
#define BLE_EXT_ADV_FRAG_DISABLED         0x01

#define LL_EXT_ADV_MODE_NOCONN_NOSC       0
#define LL_EXT_ADV_MODE_AUX_CONN_RSP      0
#define LL_EXT_ADV_MODE_CONN              1
#define LL_EXT_ADV_MODE_SC                2
#define LL_EXT_ADV_MODE_RFU               3

// AuxPtr
// channel Idx(6bits)  |   CA(1bit)   | offset Unit(1 bit)   | Aux offset(13bits)  | Aux PHY
#define LL_AUX_PTR_CHN_IDX_SHIFT               0
#define LL_AUX_PTR_CA_SHIFT                    6
#define LL_AUX_PTR_OFFSET_UNIT_SHIFT           7
#define LL_AUX_PTR_AUX_OFFSET_SHIFT            8
#define LL_AUX_PTR_AUX_PHY_SHIFT               21

#define LL_AUX_PTR_CHN_IDX_MASK               0x3F
#define LL_AUX_PTR_CA_MASK                    0x1
#define LL_AUX_PTR_OFFSET_UNIT_MASK           0x1
#define LL_AUX_PTR_AUX_OFFSET_MASK            0x1FFF
#define LL_AUX_PTR_AUX_PHY_MASK               0x7


// for Periodic scanner
#define LL_PERIODIC_ADV_CREATE_SYNC_USING_ADV_LIST_BITMASK        0x00000001
#define LL_PERIODIC_ADV_CREATE_SYNC_INIT_RPT_DISABLE_BITMASK      0x00000002

/*
** Miscellaneous
*/
#define   BLE_PKT40_LEN                   42
#define   BLE_PKT51_LEN                   262

#define   BLE_PKT_VERSION_4_0             0
#define   BLE_PKT_VERSION_5_1             1


//====== add after BBB ROM code release
#define LL_EXT_ADV_PROP_ADV_IND		      0x13        //0b00010011
#define LL_EXT_ADV_PROP_ADV_LDC_ADV		  0x15        //0b00010101
#define LL_EXT_ADV_PROP_ADV_HDC_ADV		  0x1d        //0b00011101
#define LL_EXT_ADV_PROP_ADV_SCAN_IND	  0x12        //0b00010010
#define LL_EXT_ADV_PROP_ADV_NOCONN_IND	  0x10        //0b00010000

#define LL_CHN_SEL_ALGORITHM_1            0
#define LL_CHN_SEL_ALGORITHM_2            1

// according to BQB test case HCI/GEV/BV-02-C & HCI/GEV/BV-03-C, 
// mixed legacy advertisement and extended advertisement is disallowed
// mixed legacy scan and extended scan is disallowed
#define LL_MODE_INVALID       0xFF
#define LL_MODE_LEGACY        0x00
#define LL_MODE_EXTENDED      0x01

/*******************************************************************************
 * TYPEDEFS
 */



/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
/*******************************************************************************
 * LL OSAL Functions
 */

/*******************************************************************************
 * @fn          LL_Init
 *
 * @brief       This is the Link Layer task initialization called by OSAL. It
 *              must be called once when the software system is started and
 *              before any other function in the LL API is called.
 *
 * input parameters
 *
 * @param       taskId - Task identifier assigned by OSAL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_Init( uint8 taskId );


/*******************************************************************************
 * @fn          LL_ProcessEvent
 *
 * @brief       This is the Link Layer process event handler called by OSAL.
 *
 * input parameters
 *
 * @param       taskId - Task identifier assigned by OSAL.
 *              events - Event flags to be processed by this task.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Unprocessed event flags.
 */
extern uint16 LL_ProcessEvent( uint8 task_id, uint16 events );


/*******************************************************************************
 * LL API for HCI
 */

/*******************************************************************************
 * @fn          LL_TX_bm_alloc API
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
extern void *LL_TX_bm_alloc( uint16 size );


/*******************************************************************************
 * @fn          LL_RX_bm_alloc API
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
extern void *LL_RX_bm_alloc( uint16 size );


/*******************************************************************************
 * @fn          LL_Reset API
 *
 * @brief       This function is used by the HCI to reset and initialize the
 *              LL Controller.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_Reset( void );


/*******************************************************************************
 * @fn          LL_ReadBDADDR API
 *
 * @brief       This API is called by the HCI to read the controller's
 *              own public device address.
 *
 *              Note: The device's address is stored in NV memory.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       bdAddr  - A pointer to a buffer to hold this device's address.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadBDADDR( uint8 *bdAddr );

/*******************************************************************************
 * @fn          LL_Encrypt API
 *
 * @brief       This API is called by the HCI to request the LL to encrypt the
 *              data in the command using the key given in the command.
 *
 *              Note: The parameters are byte ordered MSO to LSO.
 *
 * input parameters
 *
 * @param       *key           - A 128 bit key to be used to calculate the
 *                               session key.
 * @param       *plaintextData - A 128 bit block that is to be encrypted.
 *
 * output parameters
 *
 * @param       *encryptedData - A 128 bit block that is encrypted.
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_Encrypt( uint8 *key,
                              uint8 *plaintextData,
                              uint8 *encryptedData );


/*******************************************************************************
 * @fn          LL_Rand API
 *
 * @brief       This API is called by the HCI to request the LL Controller to
 *              provide a data block with random content.
 *
 *              Note: If the radio is in use, then this operation has to be
 *                    delayed until the radio finishes.
 *
 * input parameters
 *
 * @param       *randData - Pointer to buffer to place a random block of data.
 * @param        dataLen  - The length of the random data block, from 1-255.
 *
 * output parameters
 *
 * @param       *randData - Pointer to buffer containing a block of true random
 *                          data.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_DUE_TO_LIMITED_RESOURCES,
 *              LL_STATUS_ERROR_COMMAND_DISALLOWED,
 *              LL_STATUS_ERROR_BAD_PARAMETER, LL_STATUS_ERROR_RNG_FAILURE
 */
extern llStatus_t LL_Rand( uint8 *randData,
                           uint8 dataLen );


/*******************************************************************************
 * @fn          LL_ReadSupportedStates API
 *
 * @brief       This function is used to provide the HCI with the Link Layer
 *              supported states and supported state/role combinations.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       *states - Eight byte Bit map of supported states/combos.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadSupportedStates( uint8 *states );


/*******************************************************************************
 * @fn          LL_ReadLocalSupportedFeatures API
 *
 * @brief       This API is called by the HCI to read the controller's
 *              Features Set. The Controller indicates which features it
 *              supports.
 *
 * input parameters
 *
 * @param       featureSet  - A pointer to the Feature Set where each bit:
 *                            0: Feature not supported.
 *                            1: Feature supported by controller.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadLocalSupportedFeatures( uint8 *featureSet );


/*******************************************************************************
 * @fn          LL_ReadLocalVersionInfo API
 *
 * @brief       This API is called by the HCI to read the controller's
 *              Version information.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       verNum    - Version of the Bluetooth Controller specification.
 * @param       comId     - Company identifier of the manufacturer of the
 *                          Bluetooth Controller.
 * @param       subverNum - A unique value for each implementation or revision
 *                          of an implementation of the Bluetooth Controller.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadLocalVersionInfo( uint8  *verNum,
                                           uint16 *comId,
                                           uint16 *subverNum );


/*******************************************************************************
 * @fn          LL_CtrlToHostFlowControl API
 *
 * @brief       This function is used to indicate if the LL enable/disable
 *              receive FIFO processing. This function provides support for
 *              Controller to Host flow control.
 *
 * input parameters
 *
 * @param       mode: LL_ENABLE_RX_FLOW_CONTROL, LL_DISABLE_RX_FLOW_CONTROL
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_CtrlToHostFlowControl( uint8 mode );

/*******************************************************************************
 * @fn          LL_ReadRemoteVersionInfo API
 *
 * @brief       This API is called by the HCI to read the peer controller's
 *              Version Information. If the peer's Version Information has
 *              already been received by its request for our Version
 *              Information, then this data is already cached and can be
 *              directly returned to the Host. If the peer's Version Information
 *              is not already cached, then it will be requested from the peer,
 *              and when received, returned to the Host via the
 *              LL_ReadRemoteVersionInfoCback callback.
 *
 *              Note: Only one Version Indication is allowed for a connection.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_VER_IND_ALREADY_SENT
 */
extern llStatus_t LL_ReadRemoteVersionInfo( uint16 connId );

/*******************************************************************************
 * @fn          LL_ReadTxPowerLevel
 *
 * @brief       This function is used to read a connection's current transmit
 *              power level or the maximum transmit power level.
 *
 * input parameters
 *
 * @param       connId   - The LL connection handle.
 * @param       type     - LL_READ_CURRENT_TX_POWER_LEVEL or
 *                         LL_READ_MAX_TX_POWER_LEVEL
 * @param       *txPower - A signed value from -30..+20, in dBm.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_PARAM_OUT_OF_RANGE,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
llStatus_t LL_ReadTxPowerLevel( uint8 connId,
                                uint8 type,
                                int8  *txPower );

// A1 ROM metal change add
/*******************************************************************************
 * @fn          LL_SetTxPowerLevel
 *
 * @brief       This function is used to set transmit power level
 *
 * input parameters
 *
 * @param       txPower   - The transmit power level to be set
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
llStatus_t LL_SetTxPowerLevel( int8  txPower );

/*******************************************************************************
 * @fn          LL_ReadChanMap API
 *
 * @brief       This API is called by the HCI to read the channel map that the
 *              LL controller is using for the LL connection.
 *
 * input parameters
 *
 * @param       connId  - The LL connection handle.
 *
 * output parameters
 *
 * @param       chanMap - A five byte array containing one bit per data channel
 *                        where a 1 means the channel is "used" and a 0 means
 *                        the channel is "unused".
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_ReadChanMap( uint8 connId,
                                  uint8 *chanMap );



/*******************************************************************************
 * @fn          LL_ReadRssi API
 *
 * @brief       This API is called by the HCI to request RSSI. If there is an
 *              active connection for the given connection ID, then the RSSI of
 *              the last received data packet in the LL will be returned. If a
 *              receiver Modem Test is running, then the RF RSSI for the last
 *              received data will be returned. If no valid RSSI value is
 *              available, then LL_RSSI_NOT_AVAILABLE will be returned.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to read last RSSI.
 *
 * output parameters
 *
 * @param       *lastRssi - The last data RSSI received.
 *                          Range: -127dBm..+20dBm, 127=Not Available.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_ReadRssi( uint16 connId,
                               int8   *lastRssi );
extern llStatus_t LL_ReadFoff( uint16 connId,
                               uint16   *foff );
extern llStatus_t LL_ReadCarrSens( uint16 connId,
                                uint8   *carrSense );

/*******************************************************************************
 * @fn          LL_Disconnect API
 *
 * @brief       This API is called by the HCI to terminate a LL connection.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 * @param       reason - The reason for the Host connection termination.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 *              LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE
 */
extern llStatus_t LL_Disconnect( uint16 connId,
                                 uint8  reason );

/*******************************************************************************
 * @fn          LL_TxData API
 *
 * @brief       This API is called by the HCI to transmit a buffer of data on a
 *              given LL connection. If fragmentation is supported, the HCI must
 *              also indicate whether this is the first Host packet, or a
 *              continuation Host packet. When fragmentation is not supported,
 *              then a start packet should always specified. If the device is in
 *              a connection as a Master and the current connection ID is the
 *              connection for this data, or is in a connection as a Slave, then
 *              the data is written to the TX FIFO (even if the radio is
 *              curerntly active). If this is a Slave connection, and Fast TX is
 *              enabled and Slave Latency is being used, then the amount of time
 *              to the next event is checked. If there's at least a connection
 *              interval plus some overhead, then the next event is re-aligned
 *              to the next event boundary. Otherwise, in all cases, the buffer
 *              pointer will be retained for transmission, and the callback
 *              event LL_TxDataCompleteCback will be generated to the HCI when
 *              the buffer pointer is no longer needed by the LL.
 *
 *              Note: If the return status is LL_STATUS_ERROR_OUT_OF_TX_MEM,
 *                    then the HCI must not release the buffer until it receives
 *                    the LL_TxDataCompleteCback callback, which indicates the
 *                    LL has copied the transmit buffer.
 *
 *              Note: The HCI should not call this routine if a buffer is still
 *                    pending from a previous call. This is fatal!
 *
 *              Note: If the connection should be terminated within the LL
 *                    before the Host knows, attempts by the HCI to send more
 *                    data (after receiving a LL_TxDataCompleteCback) will
 *                    fail (LL_STATUS_ERROR_INACTIVE_CONNECTION).
 *
 * input parameters
 *
 * @param       connId   - The LL connection ID on which to send this data.
 * @param       *pBuf    - A pointer to the data buffer to transmit.
 * @param       pktLen   - The number of bytes to transmit on this connection.
 * @param       fragFlag - LL_DATA_FIRST_PKT_HOST_TO_CTRL:
 *                           Indicates buffer is the start of a
 *                           Host-to-Controller packet.
 *                         LL_DATA_CONTINUATION_PKT:
 *                           Indicates buffer is a continuation of a
 *                           Host-to-Controller packet.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION,
 *              LL_STATUS_ERROR_OUT_OF_TX_MEM,
 *              LL_STATUS_ERROR_UNEXPECTED_PARAMETER
 */
extern llStatus_t LL_TxData( uint16 connId,
                             uint8  *pBuf,
                             uint8  pktLen,
                             uint8  fragFlag );



/*******************************************************************************
 * @fn          LL_SetAdvParam API
 *
 * @brief       This API is called by the HCI to set the Advertiser's
 *              parameters.
 *
 * input parameters
 * @param       advIntervalMin - The minimum Adv interval.
 * @param       advIntervalMax - The maximum Adv interval.
 * @param       advEvtType     - The type of advertisment event.
 * @param       ownAddrType    - The Adv's address type of public or random.
 * @param       directAddrType - Only used for directed advertising.
 * @param       *directAddr    - Only used for directed advertising (NULL otherwise).
 * @param       advChanMap     - A byte containing 1 bit per advertising
 *                               channel. A bit set to 1 means the channel is
 *                               used. The bit positions define the advertising
 *                               channels as follows:
 *                               Bit 0: 37, Bit 1: 38, Bit 2: 39.
 * @param       advWlPolicy    - The Adv white list filter policy.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_NO_ADV_CHAN_FOUND
 */
extern llStatus_t LL_SetAdvParam( uint16 advIntervalMin,
                                  uint16 advIntervalMax,
                                  uint8  advEvtType,
                                  uint8  ownAddrType,
                                  uint8  directAddrType,
                                  uint8  *directAddr,
                                  uint8  advChanMap,
                                  uint8  advWlPolicy );

/*******************************************************************************
 * @fn          LL_SetAdvData API
 *
 * @brief       This API is called by the HCI to set the Advertiser's data.
 *
 *              Note: If the Advertiser is restarted without intervening calls
 *                    to this routine to make updates, then the previously
 *                    defined data will be reused.
 *
 *              Note: If the data happens to be changed while advertising, then
 *                    the new data will be sent on the next advertising event.
 *
 * input parameters
 *
 * @param       advDataLen - The number of scan response bytes: 0..31.
 * @param       advData    - Pointer to the advertiser data, or NULL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_SetAdvData( uint8 advDataLen,
                                 uint8 *advData );

/*******************************************************************************
 * @fn          LL_SetScanRspData API
 *
 * @brief       This API is called by the HCI to set the Advertiser's Scan
 *              Response data.
 *
 *              Note: If the Advertiser is restarted without intervening calls
 *                    to this routine to make updates, then the previously
 *                    defined data will be reused.
 *
 * input parameters
 *
 * @param       scanRspLen   - The number of scan response bytes: 0..31.
 * @param       *scanRspData - Pointer to the scan response data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_SetScanRspData( uint8 scanRspLen,
                                     uint8 *scanRspData );

/*******************************************************************************
 * @fn          LL_SetAdvControl API
 *
 * @brief       This API is called by the HCI to request the Controller to start
 *              or stop advertising.
 *
 * input parameters
 *
 * @param       advMode - LL_ADV_MODE_ON or LL_ADV_MODE_OFF.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_UNEXPECTED_PARAMETER,
 *              LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE,
 *              LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
extern llStatus_t LL_SetAdvControl( uint8 advMode );

/*******************************************************************************
 * @fn          LL_ReadAdvChanTxPower
 *
 * @brief       This function is used to read the transmit power level used
 *              for BLE advertising channel packets. Currently, only two
 *              settings are possible, a standard setting of 0 dBm, and a
 *              maximum setting of 4 dBm.
 *
 * input parameters
 *
 * @param       *txPower - A non-null pointer.
 *
 * output parameters
 *
 * @param       *txPower - A signed value from -20..+10, in dBm.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_PARAM_OUT_OF_RANGE
 */
extern llStatus_t LL_ReadAdvChanTxPower( int8 *txPower );

/*******************************************************************************
 * @fn          LL_SetScanParam API
 *
 * @brief       This API is called by the HCI to set the Scanner's parameters.
 *
 * input parameters
 *
 * @param       scanType     - Passive or Active scan type.
 * @param       scanInterval - Time between scan events.
 * @param       scanWindow   - Duration of a scan. When the same as the scan
 *                             interval, then scan continuously.
 * @param       ownAddrType  - Address type (Public or Random) to use in the
 *                             SCAN_REQ packet.
 * @param       advWlPolicy  - Either allow all Adv packets, or only those that
 *                             are in the white list.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER
 */
extern llStatus_t LL_SetScanParam( uint8  scanType,
                                   uint16 scanInterval,
                                   uint16 scanWindow,
                                   uint8  ownAddrType,
                                   uint8  advWlPolicy );

/*******************************************************************************
 * @fn          LL_SetScanControl API
 *
 * @brief       This API is called by the HCI to start or stop the Scanner. It
 *              also specifies whether the LL will filter duplicate advertising
 *              reports to the Host, or generate a report for each packet
 *              received.
 *
 * input parameters
 *
 * @param       scanMode      - LL_SCAN_START or LL_SCAN_STOP.
 * @param       filterReports - LL_FILTER_REPORTS_DISABLE or
 *                              LL_FILTER_REPORTS_ENABLE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_UNEXPECTED_PARAMETER,
 *              LL_STATUS_ERROR_OUT_OF_TX_MEM,
 *              LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE
 */
extern llStatus_t LL_SetScanControl( uint8 scanMode,
                                     uint8 filterReports );

/*******************************************************************************
 * @fn          LL_EncLtkReply API
 *
 * @brief       This API is called by the HCI to provide the controller with
 *              the Long Term Key (LTK) for encryption. This command is
 *              actually a reply to the link layer's LL_EncLtkReqCback, which
 *              provided the random number and encryption diversifier received
 *              from the Master during an encryption setup.
 *
 *              Note: The key parameter is byte ordered LSO to MSO.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 * @param       *key   - A 128 bit key to be used to calculate the session key.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_EncLtkReply( uint16 connId,
                                  uint8  *key );

/*******************************************************************************
 * @fn          LL_EncLtkNegReply API
 *
 * @brief       This API is called by the HCI to indicate to the controller
 *              that the Long Term Key (LTK) for encryption can not be provided.
 *              This command is actually a reply to the link layer's
 *              LL_EncLtkReqCback, which provided the random number and
 *              encryption diversifier received from the Master during an
 *              encryption setup. How the LL responds to the negative reply
 *              depends on whether this is part of a start encryption or a
 *              re-start encryption after a pause. For the former, an
 *              encryption request rejection is sent to the peer device. For
 *              the latter, the connection is terminated.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_EncLtkNegReply( uint16 connId );

/*******************************************************************************
 * @fn          LL_ConnActive
 *
 * @brief       This API is called by the HCI to check if a connection
 *              given by the connection handle is active.
 *
 * input parameters
 *
 * @param       connId - Connection handle.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 */
extern llStatus_t LL_ConnActive( uint16 connId );

/*******************************************************************************
 * @fn          LL_ConnUpdate API
 *
 * @brief       This API is called by the HCI to update the connection
 *              parameters by initiating a connection update control procedure.
 *
 * input parameters
 *
 * @param       connId          - The connection ID on which to send this data.
 * @param       connIntervalMin - Defines minimum connection interval value.
 * @param       connIntervalMax - Defines maximum connection interval value.
 * @param       connLatency     - The connection's Slave Latency.
 * @param       connTimeout     - The connection's Supervision Timeout.
 * @param       minLength       - Info parameter about min length of connection.
 * @param       maxLength       - Info parameter about max length of connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_INACTIVE_CONNECTION
 *              LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE,
 *              LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION
 */
extern llStatus_t LL_ConnUpdate( uint16 connId,
                                 uint16 connIntervalMin,
                                 uint16 connIntervalMax,
                                 uint16 connLatency,
                                 uint16 connTimeout,
                                 uint16 minLength,
                                 uint16 maxLength );

/*******************************************************************************
 * @fn          LL_ChanMapUpdate API
 *
 * @brief       This API is called by the HCI to update the Host data channels
 *              initiating an Update Data Channel control procedure.
 *
 *              Note: While it isn't specified, it is assumed that the Host
 *                    expects an update channel map on all active connections.
 *
 *              Note: This LL currently only supports one connection.
 *
 * input parameters
 *
 * @param       chanMap - A five byte array containing one bit per data channel
 *                        where a 1 means the channel is "used".
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_BAD_PARAMETER,
 *              LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION
 */
extern llStatus_t LL_ChanMapUpdate( uint8 *chanMap );

/*******************************************************************************
 * @fn          LL_ReadRemoteUsedFeatures API
 *
 * @brief       This API is called by the Master HCI to initiate a feature
 *              setup control process.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID on which to send this data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern llStatus_t LL_ReadRemoteUsedFeatures( uint16 connId );

/*
**  LL Callbacks to HCI
*/

/*******************************************************************************
 * @fn          LL_ConnectionCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              a new connection has been created. For the Slave, this means
 *              a CONNECT_REQ message was received from an Initiator. For the
 *              Master, this means a CONNECT_REQ message was sent in response
 *              to a directed or undirected message addressed to the Initiator.
 *
 * input parameters
 *
 * @param       reasonCode    - LL_STATUS_SUCCESS or ?
 * @param       connId        - The LL connection ID for new connection.
 * @param       role          - LL_LINK_CONNECT_COMPLETE_MASTER or
 *                              LL_LINK_CONNECT_COMPLETE_SLAVE.
 * @param       peerAddrType  - Peer address type (public or random).
 * @param       peerAddr      - Peer address.
 * @param       connInterval  - Connection interval.
 * @param       slaveLatency  - The connection's Slave Latency.
 * @param       connTimeout   - The connection's Supervision Timeout.
 * @param       clockAccuracy - The sleep clock accurracy of the Master. Only
 *                              valid on the Slave. Set to 0x00 for the Master.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ConnectionCompleteCback( uint8  reasonCode,
                                        uint16 connId,
                                        uint8  role,
                                        uint8  peerAddrType,
                                        uint8  *peerAddr,
                                        uint16 connInterval,
                                        uint16 slaveLatency,
                                        uint16 connTimeout,
                                        uint8  clockAccuracy );

/*******************************************************************************
 * @fn          LL_DisconnectCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              the connection has been terminated. The cause is given by the
 *              reason code.
 *
 * input parameters
 *
 * @param       connId - The LL connection ID.
 * @param       reason - The reason the connection was terminated.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_DisconnectCback( uint16 connId,
                                uint8  reason );

/*******************************************************************************
 * @fn          LL_ConnParamUpdateCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              the update parameters control procedure has completed. It is
 *              always made to the Master's Host when the update request has
 *              been sent. It is only made to the Slave's Host when the update
 *              results in a change to the connection interval, and/or the
 *              connection latency, and/or the connection timeout.
 *
 * input parameters
 *
 * @param       connId       - The LL connection ID.
 * @param       connInterval - Connection interval.
 * @param       connLatency  - The connection's Slave Latency.
 * @param       connTimeout  - The connection's Supervision Timeout.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ConnParamUpdateCback( uint16 connId,
                                     uint16 connInterval,
                                     uint16 connLatency,
                                     uint16 connTimeout );

/*******************************************************************************
 * @fn          LL_ReadRemoteVersionInfoCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host the
 *              requested peer's Version information.
 *
 * input parameters
 *
 * @param       status    - Status of callback.
 * @param       connId    - The LL connection ID.
 * @param       verNum    - Version of the Bluetooth Controller specification.
 * @param       comId     - Company identifier of the manufacturer of the
 *                          Bluetooth Controller.
 * @param       subverNum - A unique value for each implementation or revision
 *                          of an implementation of the Bluetooth Controller.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ReadRemoteVersionInfoCback( uint8  status,
                                           uint16 connId,
                                           uint8  verNum,
                                           uint16 comId,
                                           uint16 subverNum );

/*******************************************************************************
 * @fn          LL_EncChangeCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              an encryption change has taken place. This results when
 *              the host performs a LL_StartEncrypt when encryption is not
 *              already enabled.
 *
 *              Note: If the key request was rejected, then encryption will
 *                    remain off.
 *
 * input parameters
 *
 * @param       connId  - The LL connection ID for new connection.
 * @param       reason  - LL_ENC_KEY_REQ_ACCEPTED or LL_ENC_KEY_REQ_REJECTED.
 * @param       encEnab - LL_ENCRYPTION_OFF or LL_ENCRYPTION_ON.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EncChangeCback( uint16 connId,
                               uint8  reason,
                               uint8  encEnab );

/*******************************************************************************
 * @fn          LL_EncKeyRefreshCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              an encryption key change has taken place. This results when
 *              the host performs a LL_StartEncrypt when encryption is already
 *              enabled.
 *
 * input parameters
 *
 * @param       connId  - The LL connection ID for new connection.
 * @param       reason  - LL_ENC_KEY_REQ_ACCEPTED.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EncKeyRefreshCback( uint16 connId,
                                   uint8  reason );

/*******************************************************************************
 * @fn          LL_AdvReportCback Callback
 *
 * @brief       This Callback is used by the LL to provide information about
 *              advertisers from which an advertising packet was received.
 *
 * input parameters
 *
 * @param       eventType   - Type of advertisement packet received by Scanner
 *                            or Initiator, and scan response for Initiator.
 * @param       advAddrType - Advertiser address type (public or random).
 * @param       advAddr     - Advertiser address.
 * @param       dataLen     - Size in bytes of advertisement packet.
 * @param       data        - Advertisement data.
 * @param       rssi        - RSSI value (-127..20dBm), or not available
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_AdvReportCback( uint8 eventType,
                               uint8 advAddrType,
                               uint8 *advAddr,
                               uint8 dataLen,
                               uint8 *data,
                               int8  rssi );

/*******************************************************************************
 * @fn          LL_ReadRemoteUsedFeaturesCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the Host that
 *              the Read Remote Feature Support command as completed.
 *
 * input parameters
 *
 * @param       status      - SUCCESS or control procedure timeout.
 * @param       connId      - The LL connection ID for new connection.
 * @param       featureSet  - A pointer to the Feature Set.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_ReadRemoteUsedFeaturesCompleteCback( uint8  status,
                                                    uint16 connId,
                                                    uint8  *featureSet );



/*******************************************************************************
 * @fn          LL_EncLtkReqCback Callback
 *
 * @brief       This Callback is used by the LL to provide to the Host the
 *              Master's random number and encryption diversifier, and to
 *              request the Host's Long Term Key (LTK).
 *
 * input parameters
 *
 * @param       connId  - The LL connection ID for new connection.
 * @param       randNum - Random vector used in device identification.
 * @param       encDiv  - Encrypted diversifier.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_EncLtkReqCback( uint16 connId,
                               uint8  *randNum,
                               uint8  *encDiv );


/*******************************************************************************
 * @fn          LL_DirectTestEndDone Callback
 *
 * @brief       This Callback is used by the LL to notify the HCI that the
 *              Direct Test End command has completed.
 *
 *
 * input parameters
 *
 * @param       numPackets - The number of packets received. Zero for transmit.
 * @param       mode       - LL_DIRECT_TEST_MODE_TX or LL_DIRECT_TEST_MODE_RX.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
extern void LL_DirectTestEndDoneCback( uint16 numPackets,
                                       uint8  mode );
                                       
/*******************************************************************************
 * @fn          LL_DataLengthChange Callback
 *
 *
 */

extern void LL_DataLengthChangeCback(uint16 connHandle,
                                        uint16 MaxTxOctets,
                                        uint16 MaxTxTime,
                                        uint16 MaxRxOctets,
                                        uint16 MaxRxTime);
                                        

/*******************************************************************************
 * @fn          LL_TxDataCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the HCI that
 *              the HCI's buffer is free for its own use again.
 *
 * input parameters
 *
 * @param       connId   - The LL connection ID on which to send this data.
 * @param       *pBuf    - A pointer to the data buffer to transmit, or NULL.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 ******************************************************************************/
extern void LL_TxDataCompleteCback( uint16 connId,
                                    uint8  *pBuf );

/*******************************************************************************
 * @fn          LL_RxDataCompleteCback Callback
 *
 * @brief       This Callback is used by the LL to indicate to the HCI that
 *              data has been received and placed in the buffer provided by
 *              the HCI.
 *
 * input parameters
 *
 * @param       connId   - The LL connection ID on which data was received.
 * @param       *pBuf    - A pointer to the receive data buffer provided by
 *                           the HCI.
 * @param       len      - The number of bytes received on this connection.
 * @param       fragFlag - LL_DATA_FIRST_PKT indicates buffer is the start of
 *                           a Host packet.
 *                         LL_DATA_CONTINUATION_PKT: Indicates buffer is a
 *                           continuation of a Host packet.
 * @param       rssi     - The RSSI of this received packet as a signed byte.
 *                         Range: -127dBm..+20dBm, 127=Not Available.
 *
 * output parameters
 *
 * @param       **pBuf   - A double pointer updated to the next receive data
 *                         buffer, or NULL if no next buffer is available.
 *
 * @return      None.
 */
extern void LL_RxDataCompleteCback( uint16 connId,
                                    uint8  *ppBuf,
                                    uint8  len,
                                    uint8  fragFlag,
                                    int8   rssi );



/*******************************************************************************
 * @fn          LL_RandCback API
 *
 * @brief       This Callback is used by the LL to notify the HCI that the true
 *              random number command has been completed.
 *
 *              Note: The length is always given by B_RANDOM_NUM_SIZE.
 *
 * input parameters
 *
 * @param       *randData - Pointer to buffer to place a random block of data.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_RandCback( uint8 *randData );


/*******************************************************************************
 * @fn          LL_EXT_ExtendRfRangeCback Callback
 *
 * @brief       This Callback is used by the LL to notify the HCI that the
 *              Extend Rf Range command has been completed.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
//extern void LL_EXT_ExtendRfRangeCback( void );

/*******************************************************************************
 * @fn          LL_PLUS_PerStats_Init
 *
 * @brief       Used to init linklayer per stats
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_PLUS_PerStats_Init(perStatsByChan_t* p_per);
/*******************************************************************************
 * @fn          LL_PLUS_PerStatsReset
 *
 * @brief       Used to reset linklayer per stats
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_PLUS_PerStatsReset(void);


/*******************************************************************************
 * @fn          LL_PLUS_PerStasReadByChn
 *
 * @brief       read per stats by data channel id
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
extern void LL_PLUS_PerStasReadByChn(uint8 chnId,perStats_t * perStats);

extern LL_PLUS_AdvDataFilterCB_t LL_PLUS_AdvDataFilterCBack;
extern void LL_PLUS_SetAdvDataFilterCB(LL_PLUS_AdvDataFilterCB_t AdvDataFilterCBack);
extern uint8_t* LL_PLUS_GetAdvDataExtendData(void);
extern void LL_PLUS_SetScanRequestData(uint8 dLen,uint8* pData);


extern LL_PLUS_ScanRequestFilterCB_t LL_PLUS_ScanRequestFilterCBack;
extern void LL_PLUS_SetScanRequestFilterCB(LL_PLUS_ScanRequestFilterCB_t ScanRequestFilterCBack);
extern uint8 LL_PLUS_GetScanRequestExtendData(uint8* pData);
extern void LL_PLUS_GetScanerAddr(uint8* pData);
extern void LL_PLUS_SetScanRsqData(uint8 dLen,uint8* pData);

extern void LL_PLUS_SetScanRsqDataByIndex(uint8 dIdx,uint8 data);



//DLE
extern llStatus_t LL_SetDataLengh( uint16 connId,uint16 TxOctets,uint16 TxTime );
//extern uint8 LL_PLUS_GetLocalPduDataLength(ll_pdu_length_ctrl_t* pduLen);
extern llStatus_t LL_WriteSuggestedDefaultDataLength(uint16 TxOctets,uint16 TxTime);
extern void LL_DataLengthChangeCback(   uint16 connHandle,
                                                    uint16 MaxTxOctets,
                                                    uint16 MaxTxTime,
                                                    uint16 MaxRxOctets,
                                                    uint16 MaxRxTime);


                                            
//PHY UPDATE

extern llStatus_t LL_SetDefaultPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy);
extern llStatus_t LL_SetPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions);


extern llStatus_t LL_PhyUpdate( uint16 connId );
extern void LL_PhyUpdateCompleteCback(  uint16 connHandle,
                                                    uint8 status,
                                                    uint8 txPhy,
                                                    uint8 rxPhy);

/*******************************************************************************
 * @fn          LL_InitConnectContext
 *
 * @brief       This function initialize the LL connection-orient context
 *
 * input parameters
 *
 * @param       pConnContext   - connection-orient context, the memory is allocated by application
 *              maxConnNum     - the size of connect-orient context
 *              maxPktPerEventTx/Rx - number of packets transmit/receive per connection event
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
llStatus_t LL_InitConnectContext(llConnState_t    *pConnContext,
                                    uint8 *pConnBuffer,
	                                uint8 maxConnNum, 
	                                uint8 maxPktPerEventTx,
	                                uint8 maxPktPerEventRx,
	                                uint8 blePktVersion);
llStatus_t LL_SetRandomAddress( uint8 *devAddr );


#ifdef __cplusplus
}
#endif

#endif /* LL_H */


