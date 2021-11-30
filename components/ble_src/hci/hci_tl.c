/*******************************************************************************
    Filename:       hci_tl.c
    Revised:
    Revision:

    Description:    This file includes implementation for HCI task, event handler,
                  HCI Command, Data, and Event procoessing and sending, for the
                  BLE Transport Layer.


*******************************************************************************/

/*******************************************************************************
    INCLUDES
*/

#include "hci_tl.h"
#include "osal_bufmgr.h"

extern uint32 bleEvtMask;
extern uint8 pHciEvtMask[];
extern uint8 hciPTMenabled;

/*******************************************************************************
    MACROS
*/

/*******************************************************************************
    CONSTANTS
*/

/* States for Command and Data packet parser */
#define HCI_PARSER_STATE_PKT_TYPE          0
//
#define HCI_CMD_PARSER_STATE_OPCODE        1
#define HCI_CMD_PARSER_STATE_LENGTH        2
#define HCI_CMD_PARSER_STATE_DATA          3
//
#define HCI_DATA_PARSER_STATE_HANDLE       4
#define HCI_DATA_PARSER_STATE_LENGTH       5
#define HCI_DATA_PARSER_STATE_DATA         6

// HCI Command Subgroup
#define HCI_OPCODE_CSG_LINK_LAYER          0
#define HCI_OPCODE_CSG_CSG_L2CAP           1
#define HCI_OPCODE_CSG_CSG_ATT             2
#define HCI_OPCODE_CSG_CSG_GATT            3
#define HCI_OPCODE_CSG_CSG_GAP             4
#define HCI_OPCODE_CSG_CSG_SM              5
#define HCI_OPCODE_CSG_CSG_Reserved        6
#define HCI_OPCODE_CSG_CSG_USER_PROFILE    7

// Vendor Specific OGF
#define VENDOR_SPECIFIC_OGF                0x3F

/*******************************************************************************
    TYPEDEFS
*/

typedef hciStatus_t (*hciFunc_t)( uint8* pBuf );

typedef struct
{
    uint16    opCode;
    hciFunc_t hciFunc;
} hciCmdFunc_t;

typedef const hciCmdFunc_t cmdPktTable_t;

/*******************************************************************************
    LOCAL VARIABLES
*/

/*******************************************************************************
    GLOBAL VARIABLES
*/

uint8 hciTaskID;
uint8 hciTestTaskID;
//
uint8 hciGapTaskID;
uint8 hciL2capTaskID;
uint8 hciSmpTaskID;
uint8 hciExtTaskID;

/*
** Controller Prototypes
*/


// Serial Port Related
void hciSerialPacketParser( uint8 port, uint8 event );
void hciProcessHostToCtrlCmd( hciPacket_t* pBuf );
void hciProcessHostToCtrlData( hciDataPacket_t* pMsg );
void hciProcessCtrlToHost( hciPacket_t* pBuf );

// Linker Control Commands
hciStatus_t hciDisconnect                          ( uint8* pBuf );
hciStatus_t hciReadRemoteVersionInfo               ( uint8* pBuf );

// Controller and Baseband Commands
hciStatus_t hciSetEventMask                        ( uint8* pBuf );
hciStatus_t hciReset                               ( uint8* pBuf );

hciStatus_t hciReadTransmitPowerLevel              ( uint8* pBuf );
hciStatus_t hciSetControllerToHostFlowCtrl         ( uint8* pBuf );
hciStatus_t hciHostBufferSize                      ( uint8* pBuf );
hciStatus_t hciHostNumCompletedPkt                 ( uint8* pBuf );


// Information Parameters
hciStatus_t hciReadLocalVersionInfo                ( uint8* pBuf );
hciStatus_t hciReadLocalSupportedCommands          ( uint8* pBuf );
hciStatus_t hciReadLocalSupportedFeatures          ( uint8* pBuf );
hciStatus_t hciReadBDADDR                          ( uint8* pBuf );
hciStatus_t hciReadRssi                            ( uint8* pBuf );

// LE Commands
hciStatus_t hciLESetEventMask                      ( uint8* pBuf );
hciStatus_t hciLEReadBufSize                       ( uint8* pBuf );
hciStatus_t hciLEReadLocalSupportedFeatures        ( uint8* pBuf );
hciStatus_t hciLESetRandomAddr                     ( uint8* pBuf );

hciStatus_t hciLESetAdvParam                       ( uint8* pBuf );
hciStatus_t hciLESetAdvData                        ( uint8* pBuf );
hciStatus_t hciLESetScanRspData                    ( uint8* pBuf );
hciStatus_t hciLESetAdvEnab                        ( uint8* pBuf );
hciStatus_t hciLEReadAdvChanTxPower                ( uint8* pBuf );


hciStatus_t hciLESetScanParam                      ( uint8* pBuf );
hciStatus_t hciLESetScanEnable                     ( uint8* pBuf );


hciStatus_t hciLECreateConn                        ( uint8* pBuf );
hciStatus_t hciLECreateConnCancel                  ( uint8* pBuf );

hciStatus_t hciLEReadWhiteListSize                 ( uint8* pBuf );
hciStatus_t hciLEClearWhiteList                    ( uint8* pBuf );
hciStatus_t hciLEAddWhiteList                      ( uint8* pBuf );
hciStatus_t hciLERemoveWhiteList                   ( uint8* pBuf );

hciStatus_t hciLEConnUpdate                        ( uint8* pBuf );
hciStatus_t hciLESetHostChanClass                  ( uint8* pBuf );
hciStatus_t hciLEReadChanMap                       ( uint8* pBuf );
hciStatus_t hciLEReadRemoteUsedFeatures            ( uint8* pBuf );

hciStatus_t hciLEEncrypt                           ( uint8* pBuf );
hciStatus_t hciLERand                              ( uint8* pBuf );

hciStatus_t hciLEStartEncrypt                      ( uint8* pBuf );


hciStatus_t hciLELtkReqReply                       ( uint8* pBuf );
hciStatus_t hciLELtkReqNegReply                    ( uint8* pBuf );

hciStatus_t hciLEReadSupportedStates               ( uint8* pBuf );
hciStatus_t hciLEReceiverTest                      ( uint8* pBuf );
hciStatus_t hciLETransmitterTest                   ( uint8* pBuf );
hciStatus_t hciLETestEnd                           ( uint8* pBuf );

hciStatus_t hciLESetDataLength                     ( uint8* pBuf );

// Vendor Specific Commands
hciStatus_t hciExtSetRxGain                        ( uint8* pBuf );
hciStatus_t hciExtSetTxPower                       ( uint8* pBuf );
hciStatus_t hciExtExtendRfRange                    ( uint8* pBuf );
hciStatus_t hciExtHaltDuringRf                     ( uint8* pBuf );

hciStatus_t hciExtOnePktPerEvt                     ( uint8* pBuf );

hciStatus_t hciExtClkDivOnHalt                     ( uint8* pBuf );
hciStatus_t hciExtDeclareNvUsage                   ( uint8* pBuf );

hciStatus_t hciExtDelayPostProc                    ( uint8* pBuf );

hciStatus_t hciExtDecrypt                          ( uint8* pBuf );
hciStatus_t hciExtSetLocalSupportedFeatures        ( uint8* pBuf );

hciStatus_t hciExtSetFastTxResponseTime            ( uint8* pBuf );
hciStatus_t hciExtSetSlaveLatencyOverride          ( uint8* pBuf );

hciStatus_t hciExtModemTestTx                      ( uint8* pBuf );
hciStatus_t hciExtModemHopTestTx                   ( uint8* pBuf );
hciStatus_t hciExtModemtestRx                      ( uint8* pBuf );
hciStatus_t hciExtEndModemTest                     ( uint8* pBuf );
hciStatus_t hciExtSetBDADDR                        ( uint8* pBuf );

hciStatus_t hciExtSetSCA                           ( uint8* pBuf );

hciStatus_t hciExtEnablePTM                        ( uint8* pBuf );
hciStatus_t hciExtSetFreqTune                      ( uint8* pBuf );
hciStatus_t hciExtSaveFreqTune                     ( uint8* pBuf );
hciStatus_t hciExtSetMaxDtmTxPower                 ( uint8* pBuf );
//hciStatus_t hciExtMapPmIoPort                      ( uint8 *pBuf );
hciStatus_t hciExtBuildRevision                    ( uint8* pBuf );
hciStatus_t hciExtDelaySleep                       ( uint8* pBuf );
hciStatus_t hciExtResetSystem                      ( uint8* pBuf );

hciStatus_t hciExtDisconnectImmed                  ( uint8* pBuf );
hciStatus_t hciExtPER                              ( uint8* pBuf );
hciStatus_t hciExtOverlappedProcessing             ( uint8* pBuf );
hciStatus_t hciExtNumComplPktsLimit                ( uint8* pBuf );

// handle how the transport layer is built for a source build
#if !defined(HCI_TL_FULL) && !defined(HCI_TL_PTM)  && !defined(HCI_TL_NONE)
    #if defined(HAL_UART) || defined(HAL_UART_SPI)
        #if (HAL_UART == TRUE) || (HAL_UART_SPI != 0)
            #define HCI_TL_FULL
        #else // HAL_UART==FALSE/HAL_UART_SPI==0 or no/other defined value
            #define HCI_TL_NONE
        #endif // HAL_UART==TRUE || HAL_UART_SPI!=0
    #endif // HAL_UART || HAL_UART_SPI
#endif // !HCI_TL_FULL && !HCI_TL_PTM && !HCI_TL_NONE

#if defined(HCI_TL_FULL)
// HCI Packet Opcode Jump Table
cmdPktTable_t hciCmdTable[] =
{
    // Linker Control Commands
    {HCI_DISCONNECT, hciDisconnect                    },
    {HCI_READ_REMOTE_VERSION_INFO, hciReadRemoteVersionInfo         },

    // Controller and Baseband Commands
    {HCI_SET_EVENT_MASK, hciSetEventMask                  },
    {HCI_RESET, hciReset                         },

    {HCI_READ_TRANSMIT_POWER, hciReadTransmitPowerLevel        },
    {HCI_SET_CONTROLLER_TO_HOST_FLOW_CONTROL, hciSetControllerToHostFlowCtrl   },
    {HCI_HOST_BUFFER_SIZE, hciHostBufferSize                },
    {HCI_HOST_NUM_COMPLETED_PACKETS, hciHostNumCompletedPkt           },


// Informational Parameters
    {HCI_READ_LOCAL_VERSION_INFO, hciReadLocalVersionInfo          },
    {HCI_READ_LOCAL_SUPPORTED_COMMANDS, hciReadLocalSupportedCommands    },
    {HCI_READ_LOCAL_SUPPORTED_FEATURES, hciReadLocalSupportedFeatures    },
    {HCI_READ_BDADDR, hciReadBDADDR                    },
    {HCI_READ_RSSI, hciReadRssi                      },

    // LE Commands
    {HCI_LE_SET_EVENT_MASK, hciLESetEventMask                },
    {HCI_LE_READ_BUFFER_SIZE, hciLEReadBufSize                 },
    {HCI_LE_READ_LOCAL_SUPPORTED_FEATURES, hciLEReadLocalSupportedFeatures  },
    {HCI_LE_SET_RANDOM_ADDR, hciLESetRandomAddr               },

    {HCI_LE_SET_ADV_PARAM, hciLESetAdvParam                 },
    {HCI_LE_SET_ADV_DATA, hciLESetAdvData                  },
    {HCI_LE_SET_SCAN_RSP_DATA, hciLESetScanRspData              },
    {HCI_LE_SET_ADV_ENABLE, hciLESetAdvEnab                  },
    {HCI_LE_READ_ADV_CHANNEL_TX_POWER, hciLEReadAdvChanTxPower          },

    {HCI_LE_SET_SCAN_PARAM, hciLESetScanParam                },
    {HCI_LE_SET_SCAN_ENABLE, hciLESetScanEnable               },

    {HCI_LE_CREATE_CONNECTION, hciLECreateConn                  },
    {HCI_LE_CREATE_CONNECTION_CANCEL, hciLECreateConnCancel            },

    {HCI_LE_READ_WHITE_LIST_SIZE, hciLEReadWhiteListSize           },
    {HCI_LE_CLEAR_WHITE_LIST, hciLEClearWhiteList              },
    {HCI_LE_ADD_WHITE_LIST, hciLEAddWhiteList                },
    {HCI_LE_REMOVE_WHITE_LIST, hciLERemoveWhiteList             },

    {HCI_LE_CONNECTION_UPDATE, hciLEConnUpdate                  },
    {HCI_LE_SET_HOST_CHANNEL_CLASSIFICATION, hciLESetHostChanClass            },
    {HCI_LE_READ_CHANNEL_MAP, hciLEReadChanMap                 },
    {HCI_LE_READ_REMOTE_USED_FEATURES, hciLEReadRemoteUsedFeatures      },

    {HCI_LE_ENCRYPT, hciLEEncrypt                     },
    {HCI_LE_RAND, hciLERand                        },

    {HCI_LE_START_ENCRYPTION, hciLEStartEncrypt                },

    {HCI_LE_LTK_REQ_REPLY, hciLELtkReqReply                 },
    {HCI_LE_LTK_REQ_NEG_REPLY, hciLELtkReqNegReply              },

    {HCI_LE_READ_SUPPORTED_STATES, hciLEReadSupportedStates         },
    {HCI_LE_RECEIVER_TEST, hciLEReceiverTest                },
    {HCI_LE_TRANSMITTER_TEST, hciLETransmitterTest             },
    {HCI_LE_TEST_END, hciLETestEnd                     },
    {HCI_LE_SET_DATA_LENGTH, hciLESetDataLength               },
    // Vendor Specific Commands
    {HCI_EXT_SET_RX_GAIN, hciExtSetRxGain                  },
    {HCI_EXT_SET_TX_POWER, hciExtSetTxPower                 },
    {HCI_EXT_EXTEND_RF_RANGE, hciExtExtendRfRange              },
    {HCI_EXT_HALT_DURING_RF, hciExtHaltDuringRf               },

    {HCI_EXT_ONE_PKT_PER_EVT, hciExtOnePktPerEvt               },

    {HCI_EXT_CLK_DIVIDE_ON_HALT, hciExtClkDivOnHalt               },
    {HCI_EXT_DECLARE_NV_USAGE, hciExtDeclareNvUsage             },

    {HCI_EXT_DECRYPT, hciExtDecrypt                    },
    {HCI_EXT_SET_LOCAL_SUPPORTED_FEATURES, hciExtSetLocalSupportedFeatures  },

    {HCI_EXT_SET_FAST_TX_RESP_TIME, hciExtSetFastTxResponseTime      },
    {HCI_EXT_OVERRIDE_SL, hciExtSetSlaveLatencyOverride    },

    {HCI_EXT_MODEM_TEST_TX, hciExtModemTestTx                },
    {HCI_EXT_MODEM_HOP_TEST_TX, hciExtModemHopTestTx             },
    {HCI_EXT_MODEM_TEST_RX, hciExtModemtestRx                },
    {HCI_EXT_END_MODEM_TEST, hciExtEndModemTest               },
    {HCI_EXT_SET_BDADDR, hciExtSetBDADDR                  },

    {HCI_EXT_SET_SCA, hciExtSetSCA                     },

    {HCI_EXT_SET_MAX_DTM_TX_POWER, hciExtSetMaxDtmTxPower           },
//  {HCI_EXT_MAP_PM_IO_PORT                   , hciExtMapPmIoPort                },
    {HCI_EXT_SET_FREQ_TUNE, hciExtSetFreqTune                },
    {HCI_EXT_SAVE_FREQ_TUNE, hciExtSaveFreqTune               },

    {HCI_EXT_DISCONNECT_IMMED, hciExtDisconnectImmed            },
    {HCI_EXT_PER, hciExtPER                        },
    {HCI_EXT_OVERLAPPED_PROCESSING, hciExtOverlappedProcessing       },
    {HCI_EXT_NUM_COMPLETED_PKTS_LIMIT, hciExtNumComplPktsLimit          },

    {HCI_EXT_BUILD_REVISION, hciExtBuildRevision              },
    {HCI_EXT_DELAY_SLEEP, hciExtDelaySleep                 },
    // TEMP: OVERLAPPED PROCESSING HOLDER
    {HCI_EXT_RESET_SYSTEM, hciExtResetSystem                },

    // Last Table Entry Delimiter
    {0xFFFF, NULL                             }
};

#elif defined(HCI_TL_PTM)
// PTM
cmdPktTable_t hciCmdTable[] =
{
    // Controller and Baseband Commands
    {HCI_RESET, hciReset                         },

    // LE Commands - Direct Test Mode
    {HCI_LE_RECEIVER_TEST, hciLEReceiverTest                },
    {HCI_LE_TRANSMITTER_TEST, hciLETransmitterTest             },
    {HCI_LE_TEST_END, hciLETestEnd                     },
    {HCI_EXT_MODEM_TEST_TX, hciExtModemTestTx                },
    {HCI_EXT_MODEM_HOP_TEST_TX, hciExtModemHopTestTx             },
    {HCI_EXT_MODEM_TEST_RX, hciExtModemtestRx                },
    {HCI_EXT_END_MODEM_TEST, hciExtEndModemTest               },

    // LE Commands - General
    {HCI_READ_BDADDR, hciReadBDADDR                    },
    {HCI_EXT_SET_BDADDR, hciExtSetBDADDR                  },
    {HCI_EXT_SET_TX_POWER, hciExtSetTxPower                 },
    {HCI_EXT_SET_MAX_DTM_TX_POWER, hciExtSetMaxDtmTxPower           },
    {HCI_EXT_EXTEND_RF_RANGE, hciExtExtendRfRange              },
    {HCI_EXT_HALT_DURING_RF, hciExtHaltDuringRf               },

    {HCI_READ_TRANSMIT_POWER, hciReadTransmitPowerLevel        },

    {HCI_EXT_BUILD_REVISION, hciExtBuildRevision              },

    // LE Commands - Production Test Mode
    {HCI_EXT_SET_FREQ_TUNE, hciExtSetFreqTune                },
    {HCI_EXT_SAVE_FREQ_TUNE, hciExtSaveFreqTune               },
    {HCI_EXT_RESET_SYSTEM, hciExtResetSystem                },

    // Last Table Entry Delimiter
    {0xFFFF, NULL                             }
};

#else // either HCI_TL_NONE or nothing is defined
// No Transport Layer
cmdPktTable_t hciCmdTable[] =
{
    // Last Table Entry Delimiter
    {0xFFFF, NULL                             }
};

#endif // HCI_TL_FULL

/*
** HCI OSAL API
*/

/*******************************************************************************
    This is the HCI OSAL task initialization routine.

    Public function defined in hci.h.
*/
void HCI_Init0( uint8 taskID )
{
    #if !defined(HCI_TL_NONE)
    // open port
    // Note: If the Transport Layer is built without a serial port, then this
    //       operation will have no effect.
    NPI_InitTransport( hciSerialPacketParser );
    #endif // !HCI_TL_NONE
    // initialize the task for HCI-Controller
    hciTaskID      = taskID;
    hciTestTaskID  = 0;
    hciGapTaskID   = 0;
    hciL2capTaskID = 0;
    hciSmpTaskID   = 0;
    hciExtTaskID   = 0;
    // reset the Bluetooth and the BLE event mask bits
    hciInitEventMasks();
    // disable PTM runtime flag
    hciPTMenabled = FALSE;
    return;
}


/*******************************************************************************
    This is the HCI OSAL task process event handler.

    Public function defined in hci.h.
*/
uint16 HCI_ProcessEvent0( uint8 task_id, uint16 events )
{
    osal_event_hdr_t* pMsg;

    // check for system messages
    if ( events & SYS_EVENT_MSG )
    {
        pMsg = (osal_event_hdr_t*)osal_msg_receive(hciTaskID);

        if ( pMsg )
        {
            #if !defined(HCI_TL_NONE)

            switch( pMsg->event )
            {
            case HCI_HOST_TO_CTRL_DATA_EVENT:
                // process HCI data packet
                hciProcessHostToCtrlData( (hciDataPacket_t*)pMsg );
                break;

            case HCI_HOST_TO_CTRL_CMD_EVENT:
                // process HCI command packet
                hciProcessHostToCtrlCmd( (hciPacket_t*)pMsg );
                break;

            case HCI_CTRL_TO_HOST_EVENT:
                hciProcessCtrlToHost( (hciPacket_t*)pMsg );
                break;

            default:
                break;
            }

            #else // HCI_TL_NONE

            if ( (pMsg->event == HCI_HOST_TO_CTRL_DATA_EVENT) ||
                    (pMsg->event == HCI_CTRL_TO_HOST_EVENT) )
            {
                // deallocate data
                osal_bm_free( ((hciDataPacket_t*)pMsg)->pData );
            }

            // deallocate the message
            (void)osal_msg_deallocate( (uint8*)pMsg );
            #endif // !HCI_TL_NONE
        }

        // return unproccessed events
        return( events ^ SYS_EVENT_MSG );
    }

    return( 0 );
}

/*
** HCI Vendor Specific Handlers for Host
*/

/*******************************************************************************
    Register GAP task ID with HCI.

    Public function defined in hci.h.
*/
void HCI_TestAppTaskRegister( uint8 taskID )
{
    hciTestTaskID = taskID;
}


/*******************************************************************************
    Register GAP task ID with HCI.

    Public function defined in hci.h.
*/
void HCI_GAPTaskRegister( uint8 taskID )
{
    hciGapTaskID = taskID;
}


/*******************************************************************************
    Register L2CAP task ID with HCI.

    Public function defined in hci.h.
*/
void HCI_L2CAPTaskRegister( uint8 taskID )
{
    hciL2capTaskID = taskID;
}


/*******************************************************************************
    Register SMP task ID with HCI.

    Public function defined in hci.h.
*/
void HCI_SMPTaskRegister( uint8 taskID )
{
    hciSmpTaskID = taskID;
}


/*******************************************************************************
    Register task ID to receive HCI Extended Command.

    Public function defined in hci.h.
*/
void HCI_ExtTaskRegister( uint8 taskID )
{
    hciExtTaskID = taskID;
}

#if !defined(HCI_TL_NONE)

/*
** Controller Serial Related API
*/

/*******************************************************************************
    @fn          hciSerialPacketParser

    @brief       This routine is the NPI Transport callback function. It builds
                a BLE command or data packet (depending on the packet type)
                from bytes recieved by the HCI via the interface port. Bytes
                are consumed until either there are none left, or there aren't
                enough to complete a packet, in which case it is left in the
                driver buffer.

                HCI command packet format:
                Packet Type + Command opcode + lengh + command payload
                | 1 octet   |      2         |   1   |      n        |

                HCI data packet format:
                Packet Type +   Conn Handle  + lengh +  data payload
                | 1 octet   |      2         |   2   |      n      |

    input parameters

    @param       port  - UART callback serial port.
    @param       event - UART callback event number.

    output parameters

    @param       None.

    @return      None.
*/
void hciSerialPacketParser( uint8 port,
                            uint8 event )
{
    static uint8  hciPktState = HCI_PARSER_STATE_PKT_TYPE;
    static uint8  pktType;
    static uint16 param1;   // opcode for command, connection handle for data
    static uint16 pktLen;
    //
    uint8  done = FALSE;
    uint16 numBytes;
    // unused input parameter; PC-Lint error 715.
    (void)port;
    (void)event;
    // get the number of available bytes to process
    numBytes = NPI_RxBufLen();

    // check if there's any serial port data to process
    while ( (numBytes > 0) && (!done) )
    {
        // yes, so check if we are at the start of a new packet
        if ( hciPktState == HCI_PARSER_STATE_PKT_TYPE )
        {
            // yes, so read the packet type
            // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
            (void)NPI_ReadTransport(&pktType, 1);
            // decrement the number of available bytes
            numBytes -= 1;

            // set next state based on the type of packet
            // Note: Events are never sent to the target.
            switch( pktType )
            {
            case HCI_CMD_PACKET:
                hciPktState = HCI_CMD_PARSER_STATE_OPCODE;
                break;

            case HCI_ACL_DATA_PACKET:
            case HCI_SCO_DATA_PACKET:
                hciPktState = HCI_DATA_PARSER_STATE_HANDLE;
                break;

            default:
                // illegal packet type
                return;
                // Note: Unreachable statement generates compiler warning!
                //break;
            }
        }

        // process serial port bytes to build the command or data packet
        switch( hciPktState )
        {
        // command opcode
        case HCI_CMD_PARSER_STATE_OPCODE:
            if (numBytes < 2)
            {
                // not enough data to progress, so leave it in driver buffer
                done = TRUE;
                break;
            }

            // read the opcode
            // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
            (void)NPI_ReadTransport((uint8*)&param1, 2);
            // decrement the number of available bytes
            numBytes -= 2;
            hciPktState = HCI_CMD_PARSER_STATE_LENGTH;

        // DROP THROUGH
        //lint -fallthrough

        // command length
        case HCI_CMD_PARSER_STATE_LENGTH:
            if (numBytes < 1)
            {
                // not enough data to progress, so leave it in driver buffer
                done = TRUE;
                break;
            }

            // clear length before setting since only one byte of length is valid
            pktLen = 0;
            // read the length
            // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
            (void)NPI_ReadTransport((uint8*)&pktLen, 1);
            // decrement the number of available bytes
            numBytes -= 1;

            // check if length is invalid
            // Note: Further restrict what we support based on the RX buffer size.
            if ( (pktLen > HCI_MAX_CMD_PKT_SIZE) || (pktLen >= NPI_GetMaxRxBufSize()) )
            {
                // flush the input buffer
                while( NPI_ReadTransport(&pktType, 1) );

                // Max sized command packet
                hciPktState = HCI_PARSER_STATE_PKT_TYPE;
                break;
            }

            hciPktState = HCI_CMD_PARSER_STATE_DATA;

        // DROP THROUGH
        //lint -fallthrough

        // command payload
        case HCI_CMD_PARSER_STATE_DATA:

            // check if there is enough serial port data to finish command packet
            if ( numBytes >= pktLen )
            {
                hciPacket_t* pMsg;
                // there's enough serial data to finish this packet, so allocate memory
                pMsg = (hciPacket_t*)osal_msg_allocate( sizeof (hciPacket_t) +
                                                        HCI_CMD_MIN_LENGTH   +
                                                        pktLen );

                // if we have a block of allocated memory, then fill it
                if ( pMsg )
                {
                    // fill in message data
                    pMsg->pData    = (uint8*)(pMsg+1);
                    pMsg->pData[0] = pktType;
                    pMsg->pData[1] = ((uint8*)&param1)[0];  // opcode (LSB)
                    pMsg->pData[2] = ((uint8*)&param1)[1];  // opcode (MSB)
                    pMsg->pData[3] = ((uint8*)&pktLen)[0];  // one byte of length for cmd
                    // copy serial data into message
                    // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
                    (void)NPI_ReadTransport(&pMsg->pData[4], pktLen);
                    // decrement the number of available bytes
                    numBytes -= pktLen;
                    // set header specific fields
                    pMsg->hdr.status = 0xFF;

                    // check if a Controller Link Layer VS command
                    if (  ((param1 >> 10) == VENDOR_SPECIFIC_OGF) &&
                            (((param1 >> 7) & 0x07) != HCI_OPCODE_CSG_LINK_LAYER) )
                    {
                        // this is a vendor specific command
                        pMsg->hdr.event = HCI_EXT_CMD_EVENT;
                        // so strip the OGF (i.e. the most significant 6 bits of the opcode)
                        pMsg->pData[2] &= 0x03;
                        // and send it to the vendor specific extension handler
                        // Note: The HostTest project has an HCI Extension task with task ID
                        //       HCI_ExtApp_TaskID, which is used in the call to HCI_ExtTaskRegister
                        //       which assigns it to hciExtTaskID. The event HCI_EXT_CMD_EVENT only
                        //       exists in the HostTest project where it is handled by
                        //       HCI_EXT_App_ProcessEvent. Since hciExtTaskID is the same
                        //       as HCI_ExtApp_TaskID, this event is handled directly by
                        //       HCI_EXT_App_ProcessEvent in the HostTest project.
                        (void)osal_msg_send( hciExtTaskID, (uint8*)pMsg );
                    }
                    else // specification specific command
                    {
                        // this is a normal host-to-controller event
                        pMsg->hdr.event = HCI_HOST_TO_CTRL_CMD_EVENT;
                        // so send it to the HCI handler
                        (void)osal_msg_send( hciTaskID, (uint8*)pMsg );
                    }

                    // ready to start the next packet
                    hciPktState = HCI_PARSER_STATE_PKT_TYPE;
                    continue;
                }
            }

            // not enough data to progress, so leave it in driver buffer
            done = TRUE;
            break;

        case HCI_DATA_PARSER_STATE_HANDLE:
            if (numBytes < 2)
            {
                // not enough data to progress, so leave it in driver buffer
                done = TRUE;
                break;
            }

            // read the data connection handle
            // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
            (void)NPI_ReadTransport((uint8*)&param1, 2);
            // decrement the number of available bytes
            numBytes -= 2;
            hciPktState = HCI_DATA_PARSER_STATE_LENGTH;

        // DROP THROUGH
        //lint -fallthrough

        case HCI_DATA_PARSER_STATE_LENGTH:
            if (numBytes < 2)
            {
                // not enough data to progress, so leave it in driver buffer
                done = TRUE;
                break;
            }

            // read the length
            // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
            (void)NPI_ReadTransport((uint8*)&pktLen, 2);
            // decrement the number of available bytes
            numBytes -= 2;

            // check if length is invalid
            // Note: The data packet max length is 65536, but as we don't support
            //       packets of that size, we will limit the length to one byte.
            // Note: Further restrict what we support based on the RX buffer size.
            if ( (pktLen > HCI_MAX_CMD_PKT_SIZE) || (pktLen >= NPI_GetMaxRxBufSize()) )
            {
                // flush the input buffer
                while( NPI_ReadTransport(&pktType, 1) );

                // Max sized data packet
                hciPktState = HCI_PARSER_STATE_PKT_TYPE;
                break;
            }

            hciPktState = HCI_DATA_PARSER_STATE_DATA;

        // DROP THROUGH
        //lint -fallthrough

        case HCI_DATA_PARSER_STATE_DATA:

            // check if there is enough serial port data to finish data packet
            if ( numBytes >= pktLen )
            {
                hciDataPacket_t* pMsg;
                // there's enough serial data to finish this packet; allocate memory
                pMsg = (hciDataPacket_t*)osal_msg_allocate( sizeof(hciDataPacket_t) );

                // if we have a block of allocated memory, then fill it
                if ( pMsg )
                {
                    pMsg->hdr.event  = HCI_HOST_TO_CTRL_DATA_EVENT;
                    pMsg->hdr.status = 0xFF;
                    // fill in message data
                    pMsg->pktType    = pktType;
                    pMsg->connHandle = param1 & 0x0FFF;         // mask out PB and BC flags
                    pMsg->pbFlag     = (param1 & 0x3000) >> 12; // isolate PB flag
                    pMsg->pktLen     = pktLen;
                    pMsg->pData      = HCI_bm_alloc( pktLen );

                    // check if we have a BM buffer for payload
                    if ( pMsg->pData )
                    {
                        // copy serial data into message
                        // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
                        (void)NPI_ReadTransport(pMsg->pData, pktLen);
                        // decrement the number of available bytes
                        numBytes -= pktLen;
                        // send the message
                        (void)osal_msg_send( hciTaskID, (uint8*)pMsg );
                        // ready to start the next packet
                        hciPktState = HCI_PARSER_STATE_PKT_TYPE;
                        continue;
                    }
                    else // no memory available for payload
                    {
                        // so give back memory allocated for message
                        (void)osal_msg_deallocate( (uint8*)pMsg );
                    }
                }
            }

            // not enough data to progress, so leave it in driver buffer
            done = TRUE;
            break;

        default:
            //ASSERT;
            break;
        } // switch
    } // while

    return;
}


/*******************************************************************************
    @fn          hciProcessHostToCtrlCmd

    @brief       This routine handles HCI controller commands received from Host.

    input parameters

    @param       pMsg - Pointer to HCI command packet.

    output parameters

    @param       None.

    @return      None.
*/
void hciProcessHostToCtrlCmd( hciPacket_t* pMsg )
{
    uint16 cmdOpCode;
    uint8  status;
    uint8  i = 0;
    // retrieve opcode
    cmdOpCode = BUILD_UINT16 (pMsg->pData[1], pMsg->pData[2]);

    // lookup corresponding function
    while ((hciCmdTable[i].opCode != 0xFFFF) && (hciCmdTable[i].hciFunc != NULL))
    {
        // there's a valid entry at this index, but check if it's the one we want
        if (hciCmdTable[i].opCode == cmdOpCode)
        {
            // it is, so jump to this function
            (void)(hciCmdTable[i].hciFunc)(&pMsg->pData[4]);
            // done
            break;
        }

        // next...
        i++;
    }

    // check if a matching opcode was found
    if ((hciCmdTable[i].opCode == 0xFFFF) && (hciCmdTable[i].hciFunc == NULL))
    {
        // none found, so return error
        status = HCI_ERROR_CODE_UNKNOWN_HCI_CMD;
        HCI_CommandCompleteEvent ( cmdOpCode, 1, &status);
    }

    // deallocate the message
    (void)osal_msg_deallocate( (uint8*)pMsg );
    return;
}


/*******************************************************************************
    @fn          hciProcessHostToCtrlData

    @brief       This routine handles HCI controller data received from Host.

                HCI Data Packet frame format, and size in bytes:
                | Packet Type (1) | Handle (2) | Length (2) | Data (N) |

    input parameters

    @param       pMsg - Pointer to HCI data packet.

    output parameters

    @param       None.

    @return      None.
*/
void hciProcessHostToCtrlData( hciDataPacket_t* pMsg )
{
    // two types of data possible
    switch( pMsg->pktType )
    {
    case HCI_ACL_DATA_PACKET:

        // check for a problem sending data
        // Note: Success either means the packet was sent and the buffer was
        //       freed, or the packet was queued for a later transmission.
        if ( HCI_SendDataPkt( pMsg->connHandle,
                              pMsg->pbFlag,
                              pMsg->pktLen,
                              pMsg->pData ) != HCI_SUCCESS )
        {
            // packet wasn't sent or queued, so free the user's data
            osal_bm_free( (void*)pMsg->pData );
        }

        break;

    case HCI_SCO_DATA_PACKET:

    // ASSERT
    // DROP THROUGH
    //lint -fallthrough
    default:
        break;
    }

    // deallocate message
    (void)osal_msg_deallocate( (uint8*)pMsg );
    return;
}


/*******************************************************************************
    @fn          hciProcessCtrlToHost

    @brief       This routine handles sending an HCI packet out the serial
                interface.

    input parameters

    @param       pBuf - Pointer to the HCI packet.

    output parameters

    @param       None.

    @return      None.
*/
void hciProcessCtrlToHost( hciPacket_t* pBuf )
{
    switch (pBuf->pData[0])
    {
    case HCI_ACL_DATA_PACKET:
    case HCI_SCO_DATA_PACKET:

        // send through UART - dual solution
        if ( NPI_WriteTransport( pBuf->pData,
                                 HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3],
                                                                    pBuf->pData[4]) ) == 0 )
        {
            // the data was not written. Restore it in the mesage queue.
            (void)osal_msg_push_front( hciTaskID, (uint8*)pBuf );
            return; // we're done here!
        }

        // free the packet buffer
        osal_bm_free( pBuf->pData );
        break;

    case HCI_EVENT_PACKET:

        // send event through UART - dual solution
        if ( NPI_WriteTransport( &pBuf->pData[0],
                                 HCI_EVENT_MIN_LENGTH + pBuf->pData[2] ) == 0 )
        {
            // the data was not written. Restore it in the mesage queue.
            (void)osal_msg_push_front( hciTaskID, (uint8*)pBuf );
            return; // we're done here!
        }

        break;

    default:
        break;
    }

    // deallocate the message if controller only
    (void)osal_msg_deallocate( (uint8*)pBuf );
    return;
}


/*
** Serial Packet Translation Functions for HCI APIs
*/

/*******************************************************************************
    @fn          hciDisconnect

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciDisconnect( uint8* pBuf )
{
    return HCI_DisconnectCmd ( BUILD_UINT16(pBuf[0],
                                            pBuf[1]),
                               pBuf[2] );
}



/*******************************************************************************
    @fn          hciReadRemoteVersionInfo

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReadRemoteVersionInfo( uint8* pBuf )
{
    return HCI_ReadRemoteVersionInfoCmd( BUILD_UINT16(pBuf[0],
                                                      pBuf[1]) );
}


/*******************************************************************************
    @fn          hciSetEventMask

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciSetEventMask( uint8* pBuf )
{
    return HCI_SetEventMaskCmd( pBuf );
}


/*******************************************************************************
    @fn          hciReset

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReset( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_ResetCmd();
}


/*******************************************************************************
    @fn          hciReadTransmitPowerLevel

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReadTransmitPowerLevel( uint8* pBuf )
{
    return HCI_ReadTransmitPowerLevelCmd ( BUILD_UINT16(pBuf[0],
                                                        pBuf[1]),
                                           pBuf[2] );
}


/*******************************************************************************
    @fn          hciSetControllerToHostFlowCtrl

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciSetControllerToHostFlowCtrl( uint8* pBuf )
{
    return HCI_SetControllerToHostFlowCtrlCmd( pBuf[0] );
}



/*******************************************************************************
    @fn          hciHostBufferSize

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       hostAclPktLen        - Host ACL data packet length.
    @param       hostSyncPktLen       - Host SCO data packet length .
    @param       hostTotalNumAclPkts  - Host total number of ACL data packets.
    @param       hostTotalNumSyncPkts - Host total number of SCO data packets.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciHostBufferSize( uint8* pBuf )
{
    return HCI_HostBufferSizeCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                  pBuf[2],
                                  BUILD_UINT16(pBuf[3], pBuf[4]),
                                  BUILD_UINT16(pBuf[5], pBuf[6]) );
}


/*******************************************************************************
    @fn          hciHostNumCompletedPkt

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciHostNumCompletedPkt( uint8* pBuf )
{
    return HCI_HostNumCompletedPktCmd( pBuf[0],
                                       (uint16*)&pBuf[1],
                                       (uint16*)&pBuf[3] );
}


/*******************************************************************************
    @fn          hciReadLocalVersionInfo

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReadLocalVersionInfo( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_ReadLocalVersionInfoCmd();
}


/*******************************************************************************
    @fn          hciReadLocalSupportedCommands

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReadLocalSupportedCommands( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_ReadLocalSupportedCommandsCmd();
}


/*******************************************************************************
    @fn          hciReadLocalSupportedFeatures

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReadLocalSupportedFeatures( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_ReadLocalSupportedFeaturesCmd();
}


/*******************************************************************************
    @fn          hciReadRssi

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReadRssi( uint8* pBuf )
{
    return HCI_ReadRssiCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}


/*******************************************************************************
    @fn          hciLESetEventMask

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetEventMask( uint8* pBuf )
{
    return HCI_LE_SetEventMaskCmd( pBuf );
}


/*******************************************************************************
    @fn          hciLEReadBufSize

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReadBufSize( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_ReadBufSizeCmd();
}


/*******************************************************************************
    @fn          hciLEReadLocalSupportedFeatures

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReadLocalSupportedFeatures( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_ReadLocalSupportedFeaturesCmd();
}


/*******************************************************************************
    @fn          hciLESetRandomAddr

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetRandomAddr( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_SetRandomAddressCmd( pBuf );
}


/*******************************************************************************
    @fn          hciLESetAdvParam

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetAdvParam( uint8* pBuf )
{
    return HCI_LE_SetAdvParamCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                  BUILD_UINT16(pBuf[2], pBuf[3]),
                                  pBuf[4],
                                  pBuf[5],
                                  pBuf[6],
                                  &pBuf[7],
                                  pBuf[13],
                                  pBuf[14] );
}


/*******************************************************************************
    @fn          hciLESetAdvData

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetAdvData( uint8* pBuf )
{
    return HCI_LE_SetAdvDataCmd( pBuf[0],
                                 &pBuf[1] );
}


/*******************************************************************************
    @fn          hciLESetScanRspData

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetScanRspData( uint8* pBuf )
{
    return HCI_LE_SetScanRspDataCmd( pBuf[0],
                                     &pBuf[1] );
}


/*******************************************************************************
    @fn          hciLESetAdvEnab

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetAdvEnab( uint8* pBuf )
{
    return HCI_LE_SetAdvEnableCmd( pBuf[0] );
}

/*******************************************************************************
    @fn          hciLEReadAdvChanTxPower

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReadAdvChanTxPower( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_ReadAdvChanTxPowerCmd();
}

/*******************************************************************************
    @fn          hciLESetScanParam

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetScanParam( uint8* pBuf )
{
    return HCI_LE_SetScanParamCmd( pBuf[0],
                                   BUILD_UINT16(pBuf[1], pBuf[2]),
                                   BUILD_UINT16(pBuf[3], pBuf[4]),
                                   pBuf[5],
                                   pBuf[6] );
}

/*******************************************************************************
    @fn          hciLESetScanEnable

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetScanEnable( uint8* pBuf )
{
    return HCI_LE_SetScanEnableCmd( pBuf[0],
                                    pBuf[1] );
}

/*******************************************************************************
    @fn          hciLECreateConn

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLECreateConn( uint8* pBuf )
{
    return HCI_LE_CreateConnCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                 BUILD_UINT16(pBuf[2], pBuf[3]),
                                 pBuf[4],
                                 pBuf[5],
                                 &pBuf[6],
                                 pBuf[12],
                                 BUILD_UINT16(pBuf[13], pBuf[14]),
                                 BUILD_UINT16(pBuf[15], pBuf[16]),
                                 BUILD_UINT16(pBuf[17], pBuf[18]),
                                 BUILD_UINT16(pBuf[19], pBuf[20]),
                                 BUILD_UINT16(pBuf[21], pBuf[22]),
                                 BUILD_UINT16(pBuf[23], pBuf[24]) );
}



/*******************************************************************************
    @fn          hciLECreateConnCancel

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLECreateConnCancel( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_CreateConnCancelCmd();
}



/*******************************************************************************
    @fn          hciLEReadWhiteListSize

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReadWhiteListSize( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_ReadWhiteListSizeCmd();
}


/*******************************************************************************
    @fn          hciLEClearWhiteList

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEClearWhiteList( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_ClearWhiteListCmd();
}


/*******************************************************************************
    @fn          hciLEAddWhiteList

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEAddWhiteList( uint8* pBuf )
{
    return HCI_LE_AddWhiteListCmd( pBuf[0],
                                   &pBuf[1] );
}


/*******************************************************************************
    @fn          hciLERemoveWhiteList

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLERemoveWhiteList( uint8* pBuf )
{
    return HCI_LE_RemoveWhiteListCmd( pBuf[0],
                                      &pBuf[1] );
}


/*******************************************************************************
    @fn          hciLEConnUpdate

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEConnUpdate( uint8* pBuf )
{
    return HCI_LE_ConnUpdateCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                 BUILD_UINT16(pBuf[2], pBuf[3]),
                                 BUILD_UINT16(pBuf[4], pBuf[5]),
                                 BUILD_UINT16(pBuf[6], pBuf[7]),
                                 BUILD_UINT16(pBuf[8], pBuf[9]),
                                 BUILD_UINT16(pBuf[10], pBuf[11]),
                                 BUILD_UINT16(pBuf[12], pBuf[13]) );
}



/*******************************************************************************
    @fn          hciLESetHostChanClass

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLESetHostChanClass( uint8* pBuf )
{
    return HCI_LE_SetHostChanClassificationCmd( pBuf );
}



/*******************************************************************************
    @fn          hciLEReadChanMap

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReadChanMap( uint8* pBuf )
{
    return HCI_LE_ReadChannelMapCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}



/*******************************************************************************
    @fn          hciLEReadRemoteUsedFeatures

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReadRemoteUsedFeatures( uint8* pBuf )
{
    return HCI_LE_ReadRemoteUsedFeaturesCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}



/*******************************************************************************
    @fn          hciLEEncrypt

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEEncrypt( uint8* pBuf )
{
    // reverse byte order of key (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[0], KEYLEN );
    // reverse byte order of plaintext (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[KEYLEN], KEYLEN );
    return HCI_LE_EncryptCmd( &pBuf[0],
                              &pBuf[KEYLEN] );
}


/*******************************************************************************
    @fn          hciLERand

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLERand( uint8* pBuf )
{
    (void)pBuf;
    return HCI_LE_RandCmd();
}


/*******************************************************************************
    @fn          hciLEStartEncrypt

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEStartEncrypt( uint8* pBuf )
{
    return HCI_LE_StartEncyptCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                  &pBuf[2],
                                  &pBuf[10],
                                  &pBuf[12] );
}


/*******************************************************************************
    @fn          hciLELtkReqReply

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLELtkReqReply( uint8* pBuf )
{
    return HCI_LE_LtkReqReplyCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                  &pBuf[2] );
}


/*******************************************************************************
    @fn          hciLELtkReqNegReply

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLELtkReqNegReply( uint8* pBuf )
{
    return HCI_LE_LtkReqNegReplyCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}


/*******************************************************************************
    @fn          hciLEReadSupportedStates

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReadSupportedStates( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_ReadSupportedStatesCmd();
}


/*******************************************************************************
    @fn          hciLEReceiverTest

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLEReceiverTest( uint8* pBuf )
{
    return HCI_LE_ReceiverTestCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciLETransmitterTest

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLETransmitterTest( uint8* pBuf )
{
    return HCI_LE_TransmitterTestCmd( pBuf[0],
                                      pBuf[1],
                                      pBuf[2] );
}

/*******************************************************************************
    @fn          hciLETestEnd

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciLETestEnd( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_LE_TestEndCmd();
}

hciStatus_t hciLESetDataLength                     ( uint8* pBuf )
{
    return HCI_LE_SetDataLengthCmd( BUILD_UINT16(pBuf[0], pBuf[1]),
                                    BUILD_UINT16(pBuf[2], pBuf[3])
                                    BUILD_UINT16(pBuf[4], pBuf[5]));
}

#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)
/*
** Vendor Specific Commands
*/

/*******************************************************************************
    @fn          hciExtSetRxGain

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetRxGain( uint8* pBuf )
{
    return HCI_EXT_SetRxGainCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtSetTxPower

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetTxPower( uint8* pBuf )
{
    return HCI_EXT_SetTxPowerCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtExtendRfRange

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtExtendRfRange( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_EXT_ExtendRfRangeCmd();
}


/*******************************************************************************
    @fn          hciExtHaltDuringRf

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtHaltDuringRf( uint8* pBuf )
{
    return HCI_EXT_HaltDuringRfCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtSetMaxDtmTxPower

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetMaxDtmTxPower( uint8* pBuf )
{
    return HCI_EXT_SetMaxDtmTxPowerCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtDisconnectImmed

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtDisconnectImmed( uint8* pBuf )
{
    return HCI_EXT_DisconnectImmedCmd ( BUILD_UINT16(pBuf[0],
                                                     pBuf[1]) );
}


/*******************************************************************************
    @fn          hciExtPER

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtPER( uint8* pBuf )
{
    return HCI_EXT_PacketErrorRateCmd ( BUILD_UINT16(pBuf[0],
                                                     pBuf[1]),
                                        pBuf[2] );
}



/*******************************************************************************
    @fn          hciExtOverlappedProcessing

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtOverlappedProcessing( uint8* pBuf )
{
    return HCI_EXT_OverlappedProcessingCmd ( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtNumComplPktsLimit

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtNumComplPktsLimit( uint8* pBuf )
{
    return HCI_EXT_NumComplPktsLimitCmd( pBuf[0],
                                         pBuf[1] );
}

/*******************************************************************************
    @fn          hciExtOnePktPerEvt

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtOnePktPerEvt( uint8* pBuf )
{
    return HCI_EXT_OnePktPerEvtCmd( pBuf[0] );
}



/*******************************************************************************
    @fn          hciExtClkDivOnHalt

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtClkDivOnHalt( uint8* pBuf )
{
    return HCI_EXT_ClkDivOnHaltCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtDeclareNvUsage

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtDeclareNvUsage( uint8* pBuf )
{
    return HCI_EXT_DeclareNvUsageCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtDecrypt

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtDecrypt( uint8* pBuf )
{
    // reverse byte order of key (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[0], KEYLEN );
    // reverse byte order of encText (MSB..LSB required)
    HCI_ReverseBytes( &pBuf[KEYLEN], KEYLEN );
    return HCI_EXT_DecryptCmd( &pBuf[0],
                               &pBuf[KEYLEN] );
}


/*******************************************************************************
    @fn          hciExtSetLocalSupportedFeatures

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetLocalSupportedFeatures( uint8* pBuf )
{
    return HCI_EXT_SetLocalSupportedFeaturesCmd( pBuf );
}


/*******************************************************************************
    @fn          hciExtSetFastTxResponseTime

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetFastTxResponseTime( uint8* pBuf )
{
    return HCI_EXT_SetFastTxResponseTimeCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtSetSlaveLatencyOverride

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetSlaveLatencyOverride( uint8* pBuf )
{
    return HCI_EXT_SetSlaveLatencyOverrideCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtSetSCA

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetSCA( uint8* pBuf )
{
    return HCI_EXT_SetSCACmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}


/*******************************************************************************
    @fn          hciExtBuildRevision

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtBuildRevision( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_EXT_BuildRevisionCmd( pBuf[0], BUILD_UINT16( pBuf[1],
                                                            pBuf[2]) );
}


/*******************************************************************************
    @fn          hciExtDelaySleep

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtDelaySleep( uint8* pBuf )
{
    return HCI_EXT_DelaySleepCmd( BUILD_UINT16(pBuf[0], pBuf[1]) );
}


/*******************************************************************************
    @fn          hciExtResetSystem

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtResetSystem( uint8* pBuf )
{
    return HCI_EXT_ResetSystemCmd( pBuf[0] );
}


/*
** Allowed PTM Commands
*/

/*******************************************************************************
    @fn          hciReadBDADDR

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciReadBDADDR( uint8* pBuf )
{
    // unused input parameter; PC-Lint error 715.
    (void)pBuf;
    return HCI_ReadBDADDRCmd();
}


/*******************************************************************************
    @fn          hciExtModemTestTx

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtModemTestTx( uint8* pBuf )
{
    return HCI_EXT_ModemTestTxCmd( pBuf[0], pBuf[1] );
}


/*******************************************************************************
    @fn          hciExtModemHopTestTxCmd

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtModemHopTestTx( uint8* pBuf )
{
    return HCI_EXT_ModemHopTestTxCmd();
}


/*******************************************************************************
    @fn          hciExtModemtestRxCmd

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtModemtestRx( uint8* pBuf )
{
    return HCI_EXT_ModemTestRxCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtEndModemTest

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtEndModemTest( uint8* pBuf )
{
    return HCI_EXT_EndModemTestCmd();
}


/*******************************************************************************
    @fn          hciExtSetBDADDR

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetBDADDR( uint8* pBuf )
{
    return HCI_EXT_SetBDADDRCmd( pBuf );
}


/*******************************************************************************
    @fn          hciExtEnablePTM

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtEnablePTM( uint8* pBuf )
{
    return HCI_EXT_EnablePTMCmd();
}


/*******************************************************************************
    @fn          hciExtSetFreqTune

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSetFreqTune( uint8* pBuf )
{
    return HCI_EXT_SetFreqTuneCmd( pBuf[0] );
}


/*******************************************************************************
    @fn          hciExtSaveFreqTune

    @brief       Serial interface translation function for HCI API.

    input parameters

    @param       pBuf - Pointer to command parameters and payload.

    output parameters

    @param       None.

    @return      hciStatus_t
*/
hciStatus_t hciExtSaveFreqTune( uint8* pBuf )
{
    return HCI_EXT_SaveFreqTuneCmd();
}
#endif  /*#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)*/

#endif // !HCI_TL_NONE

/*******************************************************************************
*/
