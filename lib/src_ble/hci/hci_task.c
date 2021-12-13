/*************************************************************************************************
  Filename:       HCI_Host.c
  Revised:        $Date: 2010-02-23 10:47:02 -0800 (Tue, 23 Feb 2010) $
  Revision:       $Revision: 21759 $

  Description:    TThis file contains code to process rx/tx and timer events 
                  for HCI.


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
#include "hci.h"
#include "hci_task.h"

/*********************************************************************
 * DEFINES

/* States for Data packet parser */
#define HCI_DATA_PARSER_HANDLER_0_STATE 0x00
#define HCI_DATA_PARSER_HANDLER_1_STATE 0x01
#define HCI_DATA_PARSER_LENGTH_0_STATE  0x02
#define HCI_DATA_PARSER_LENGTH_1_STATE  0x03

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void hciHostPacketHandler( uint8 port, uint8 event );
static void hciHostDataPacketParser( uint8 port, uint8 event );
static void hciHostEvtPacketHandler (uint8 port, uint8 packetType);
static void hciHostEvtPacketParser( hciPacket_t *pkt );
static void hciHostLeEventParser( hciPacket_t *pkt );
static uint8* HCI_LEAdvPktReportEvtParser( uint8 paramLen, uint8 *pParam );
static void hciHostCmdCompleteEvtProcess( hciPacket_t *pkt );
static uint8 *HCI_ReturnParamParser_LEReadBufSize( uint8 len, uint8 *pData );

/*********************************************************************
 * GLOBAL VARIEBLE
 */

uint8 HCI_TaskID;

osal_msg_q_t hciHostCmdQueue;
osal_msg_q_t hciHostDataQueue;

uint8 hciHostNumQueuedCmd = 0;          /* Number of command packets queued */
uint8 hciHostNumQueuedData = 0;         /* Number of command packets queued */

uint8 hciCmdToken = 0;
uint8 hciDataToken = 0;

/* CMD Parser state */
uint8 HCI_CmdParserState;

/* DATA Parser state */
uint8 HCI_DataParserState;

struct 
{
  uint8 gapTaskID;
  uint8 l2capTaskID;
  uint8 smpTaskID;
} HCIOsalTask;

typedef uint8 * (*HCIEvtParserFunc_t) ( uint8 paramLen, uint8 *pBuf );

typedef struct
{
  uint16 opCode;
  HCIEvtParserFunc_t hciFunc;
}HCIBleEvtParser_t;

HCIBleEvtParser_t HCIBleEvtTable[] =
{
  /* Controller setup and configuration */
  {HCI_ULP_ADV_PKT_REPORT_EVENT_CODE         , HCI_LEAdvPktReportEvtParser       },

};

typedef uint8 *(*HCIRtnParamFunc_t) (uint8 len, uint8 *pBuf);

typedef struct
{
  uint16 opCode;
  HCIRtnParamFunc_t hciFunc;
}HCIRtnParamParser_t;

HCIRtnParamParser_t HCIRtnParamTable[] = 
{
  {HCI_BLE_READ_BUFFER_SIZE                 , HCI_ReturnParamParser_LEReadBufSize    },
};



/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * @fn          HCI_Init
 *
 * @brief       Initialization for HCI Host side
 *
 * @param       taskID
 *
 * @return      none
 */
void HCI_Init( uint8 taskID  )
{
	/*
  halUARTCfg_t uartConfig;
*/
  HCI_TaskID = taskID;

  /* UART Configuration */
/*  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = HCI_UART_BR;
  uartConfig.flowControl          = HCI_UART_FC;
  uartConfig.flowControlThreshold = HCI_UART_FC_THRESHOLD;
  uartConfig.rx.maxBufSize        = HCI_UART_RX_BUF_SIZE;
  uartConfig.tx.maxBufSize        = HCI_UART_TX_BUF_SIZE;
  uartConfig.idleTimeout          = HCI_UART_IDLE_TIMEOUT;
  uartConfig.intEnable            = HCI_UART_INT_ENABLE;
  uartConfig.callBackFunc         = hciHostPacketHandler;
*/
  /* Start UART */
 /* HalUARTOpen (HCI_UART_PORT, &uartConfig);*/

  /* Initialize the HCI Message Queue */
  /* Todo */
	
	uart_init (38400,5,4,1);
	
}


/*********************************************************************
 * @fn          HCI_ProcessEvent
 *
 * @brief       HCI Task event processing function.
 *
 * @param       taskID - HCI task ID.
 * @param       events - HCI events.
 *
 * @return      events not processed
 */
uint16 HCI_ProcessEvent( uint8 taskId, uint16 events )
{
  uint8 *pSerialPkt;
  uint8 len;
  uint16 dataLen;
  hciPacket_t * pMsg;

  if ( events & SYS_EVENT_MSG )
  {
    while (pMsg = (hciPacket_t *) osal_msg_receive( taskId ) )
    {
      switch (pMsg->hdr.event)
      {
        case HCI_HOST_PARSE_EVT:
          /* Incoming event packet from Controller, parse it */
          hciHostEvtPacketParser ((hciPacket_t *)pMsg);
          break;

        default:
          break;
      }
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & HCI_EVENT_SEND_CMD )
  {
	  /* Send queued cmd packet from Host to Controller */
    /* Check token availability */
    if ( hciCmdToken > 0 )
	  {
	    /* Send one data packet from the queue */
	    pSerialPkt = osal_msg_dequeue( hciHostCmdQueue );
	    if( pSerialPkt != NULL )
	    {
		    hciHostNumQueuedData--;
     
        /* Calculate the total packet length 
           Header length + parameter length */
        len = HCI_COMMAND_HEADER_LEN + pSerialPkt[HCI_COMMAND_HEADER_LEN - 1];

        HalUARTWrite( HCI_UART_PORT, pSerialPkt, len );

        /* Decrement the token */
	      hciCmdToken--;
	    }
	  }

	  /* If there are more packets in the queue and 
	     token is available, trigger another event */
	  if ( hciCmdToken > 0 ) //|| more queue)
	  {
      osal_set_event( taskId, HCI_EVENT_SEND_CMD );
	  }

    // return unprocessed events
    return (events ^ HCI_EVENT_SEND_CMD);
  }

  if ( events & HCI_EVENT_SEND_DATA )
  {
	  /* Send queued data packet from Host to Controller */
    /* Check token availability */
    if ( hciDataToken > 0 )
	  {
	    /* Send one data packet from the queue */
	    pSerialPkt = osal_msg_dequeue( hciHostDataQueue );

	    if( pSerialPkt != NULL )
	    {
		    hciHostNumQueuedData--;

        /* Calculate the total packet length 
           Header length + parameter length */
        dataLen = HCI_DATA_HEADER_LEN + 
		              BUILD_UINT16( pSerialPkt[HCI_DATA_HEADER_LEN - 2], 
				          pSerialPkt[HCI_DATA_HEADER_LEN - 1] );

        HalUARTWrite( HCI_UART_PORT, pSerialPkt, dataLen );

        /* Decrement the token */
	      hciDataToken--;
	    }
	  }

	  /* If there are more packets in the queue and 
	     token is available, trigger another event */
	  if ( hciDataToken > 0 || hciHostNumQueuedData > 0 )
	  {
      osal_set_event( taskId, HCI_EVENT_SEND_DATA );
	  }

    // return unprocessed events
    return (events ^ HCI_EVENT_SEND_DATA);
  }

  // If reach here, the events are unknown
  // Discard or make more handlers
  return 0;

}

/*********************************************************************
 * @fn          HCI_GAPTaskRegister
 *
 * @brief       Register GAP task ID with HCI
 *
 * @param       taskID - GAP Task ID
 *
 * @return      none
 */
void HCI_GAPTaskRegister( uint8 taskID )
{
  HCIOsalTask.gapTaskID = taskID;
}
/*********************************************************************
 * @fn          HCI_GapTaskRegister
 *
 * @brief       Register L2CAP task ID with HCI
 *
 * @param       taskID - L2CAP Task ID
 *
 * @return      none
 */
void HCI_L2CAPTaskRegister( uint8 taskID )
{
  HCIOsalTask.l2capTaskID = taskID;
}

/*********************************************************************
 * @fn          HCI_SMPTaskRegister
 *
 * @brief       Register SMP task ID with HCI
 *
 * @param       taskID - SMP Task ID
 *
 * @return      none
 */
void HCI_SMPTaskRegister( uint8 taskID )
{
  HCIOsalTask.smpTaskID = taskID;
}


/*********************************************************************
 * Buffer Management
 */

/*********************************************************************
 * @fn          HCI_AddDataQueue
 *
 * @brief       Add a HCI data packet to the data queue.
 *
 * @param       buf - Pointer to the sending buffer.
 *
 * @return      Status_t
 */
Status_t HCI_AddDataQueue( void *buf )
{
  /* Check the size of the queue : TBA */
  if( hciHostNumQueuedData == HCI_HOST_MAX_DATAQUEUE_SIZE )
  {
	// If cmd queue is full, return error
    return MSG_BUFFER_NOT_AVAIL;
  }

  /* Otherwise, add the packet to the message buffer */
  osal_msg_enqueue( hciHostDataQueue, buf );

  /* Increment the counter */
  hciHostNumQueuedData++;

  return SUCCESS;
}

/*********************************************************************
 * @fn          HCI_AddCmdQueue
 *
 * @brief       Add a HCI CMD packet to the data queue.
 *
 * @param       buf - Pointer to the sending buffer.
 *
 * @return      Status_t
 */
Status_t HCI_AddCmdQueue( void *buf )
{
  /* Check the size of the queue : TBA */
  if( hciHostNumQueuedCmd == HCI_HOST_MAX_CMDQUEUE_SIZE )
  {
	// If cmd queue is full, return error
    return MSG_BUFFER_NOT_AVAIL;
  }

  /* Otherwise, add the packet to the message buffer */
  osal_msg_enqueue( hciHostCmdQueue, buf );

  /* Increment the counter */
  hciHostNumQueuedCmd++;

  return SUCCESS;
}


/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn          hciHostPacketHandler
 *
 * @brief       Handle HCI packets sent from Controller to Host.
 *
 * @param       port  - serial port where the event comes from
 *              event - event that causes the callback
 *
 * @return      none
 */
void hciHostPacketHandler( uint8 port, uint8 event )
{
  uint8  ch;

  /* Read the first byte which is packet type */
  HalUARTRead (port, &ch, 1);

  if( ch == HCI_SCO_DATA_PACKET )
  {
    hciHostDataPacketParser( port, ch );
  }
  else if( ch == HCI_EVENT_PACKET )
  {
    hciHostEvtPacketHandler( port, ch);
  }
}

/*********************************************************************
 * @fn          hciHostDataPacketParser
 *
 * @brief       Read and parse HCI Data packets sent from Controller to Host
 *              then send it to HCI task through osal msg.
 *
 * @param       port - port where data comes from
 *
 * @return      none
 */
static void hciHostDataPacketParser( uint8 port, uint8 event )
{
  HCIData_t *msg;
  uint8 handler0 = 0x00;
  uint8 handler1 = 0x00;
  uint8 length0  = 0x00;
  uint8 length1  = 0x00;
  uint16 dataLength = 0x0000;

  if( Hal_UART_RxBufLen( port) >= ( HCI_DATA_HEADER_LEN - 1 ) ) /* 1st byte is packetType */
  {
	  HalUARTRead( port, &handler0, 1 );
    HalUARTRead( port, &handler1, 1 );
    HalUARTRead( port, &length0, 1 );
    HalUARTRead( port, &length1, 1 );
    
    /* Calculate data length */
    dataLength = BUILD_UINT16(length0, length1);

    if( dataLength == 0 )
    {
      /* Zero length data packet, discard */
      return;
    }

    if( Hal_UART_RxBufLen( port) >= dataLength )
    {
      /* Allocate a osal message : structure plus data payload */
      msg = (HCIData_t *)osal_msg_allocate( sizeof ( HCIData_t ) + dataLength );

      if ( msg )
      {
        /* Message type, length */
        msg->hdr.event = HCI_HOST_INCOMING_DATA;
        msg->hdr.status = SUCCESS;

        /* Fill in the message */
        msg->connection.connectionHandle = BUILD_UINT16( handler0, handler1 ) & HCI_CONNECTION_HANDLE_MASK;
        msg->connection.pb = ( handler1 >> 4 ) * HCI_PB_MASK; 
        msg->len = dataLength;
        msg->pData = (uint8*)( msg + 1 );

        /* Read the rest of the packet */
        HalUARTRead (port, msg->pData, dataLength);

        osal_msg_send( HCI_TaskID, (uint8 *)msg );
      }
      else
      {
        return;
      }
    }
    else 
    {
      /* Not enough data for this packet - Discard the packet */
    }
  }
}


/*********************************************************************
 * @fn          hciHostEvtPacketHandler
 *
 * @brief       Handle HCI event packets sent from Controller to Host
 *              then send it to HCI parser through osal msg.
 *
 * @param       port - port where data comes from
 *
 * @return      none
 */
static void hciHostEvtPacketHandler (uint8 port, uint8 packetType)
{
  hciPacket_t *msg;
  uint8 evtCode = 0x00;
  uint8 length = 0x00;

  if( Hal_UART_RxBufLen(port) >= ( HCI_EVT_HEADER_LEN - 1 ) ) /* 1st byte is packetType */
  {
	  HalUARTRead( port, &evtCode, 1 );
    HalUARTRead( port, &length, 1 );
    
    if( Hal_UART_RxBufLen(port) >= length )
    {
      msg = (hciPacket_t *)osal_msg_allocate( sizeof ( hciPacket_t ) + HCI_EVT_HEADER_LEN + length );

      if( msg )
      {
        msg->hdr.event = HCI_HOST_PARSE_EVT;
        msg->hdr.status = 0x00;

        /* Fill in the event header */
        msg->pData[0] = packetType;
        msg->pData[1] = evtCode;
        msg->pData[2] = length;

        /* Read the rest of the packet */
        HalUARTRead( port, &msg->pData[3], length );
        osal_msg_send( HCI_TaskID, (uint8 *)msg );
      }
      else
      {
        return;
      }
    }
    else 
    {
      /* Not enough data for this packet - Discard the packet */
    }
  }
}


/*********************************************************************
 * @fn          hciHostEvtPacketParser
 *
 * @brief       Parse HCI event packets sent from Controller to Host
 *              then send it to HCI parser through osal msg.
 *
 * @param       pkt - HCI Event packet
 *
 * @return      none
 */
static void hciHostEvtPacketParser( hciPacket_t *pkt )
{
  uint8 evtCode;

  evtCode = pkt->pData[1];

  // Parse and process the rest of HCI events
  switch( evtCode )
  {
  case HCI_ULP_EVENT_CODE:
      hciHostLeEventParser( pkt );
      break;

    case HCI_ULP_COMMAND_COMPLETE_EVENT_CODE:
      hciHostCmdCompleteEvtProcess( pkt );
      break;

#if 0 /* Todo */
    case HCI_ULP_COMMAND_STATUS_EVENT_CODE:
      hciHostCmdStatusEvtProcess( pkt );
      break;

    case HCI_ULP_HARDWARE_ERROR_EVENT_CODE:
      hciHostHardwareErrorEvtProcess( pkt );
      break;

    case HCI_ULP_NUM_OF_COMPLETED_PACKETS_EVENT_CODE:
      hciHostNumCompletedPktEvtProcess( pkt );
      break;
#endif

    default:
      break;
  }
}


/*********************************************************************
 * @fn          hciHostLeEventParser
 *
 * @brief       Process HCI ULP event packets sent from Controller to Host
 *              then send it to HCI parser through osal msg.
 *
 * @param       pkt - HCI Event packet
 *
 * @return      none
 */
static void hciHostLeEventParser( hciPacket_t *pkt )
{
  uint8 bleEvtCode;
  uint8 *msg;

  bleEvtCode = pkt->pData[3];   /* BLE event code */

  /* Look up for corresponding event parsing function
     table index is equal to bleEvtCode -1 */

  /* Pass in length of event parameter and pointer to the parameter list */
  msg = HCIBleEvtTable[bleEvtCode - 1].hciFunc( pkt->pData[2], &pkt->pData[3] );

  /* Send the osal message to HCI task to be rerouted to higher layers */
  osal_msg_send( HCI_TaskID, msg );
}


/*********************************************************************
 * @fn          HCI_LEAdvPktReportEvtParser
 *
 * @brief       Allocate the osal message and parse the HCI LE 
 *              Advertising Packet Report event packets 
 *
 * @param       paramLen - length of the parameter list
 *              pParam   - pointer to the event parameter list
 *
 * @return      Pointer to the allocated osal message for the parsed evt
 *              Return NULL means parsing failed
 */
static uint8* HCI_LEAdvPktReportEvtParser( uint8 paramLen, uint8 *pParam )
{
  HCIEvt_BLEAdvPktReport_t *evt;
  uint8 i;
  uint8 num;
  uint16 len;
  uint8 *pBuf;

  /* octet #2 - number of advertisers reported in the event */
  num = pParam[1];
  
  /* The array of dev info is pended following the structure */
  len = sizeof( HCIEvt_BLEAdvPktReport_t ) + num * sizeof( HCIEvt_DevInfo_t );

  if (( evt = osal_mem_alloc( len )) == NULL )
  {
    return NULL;
  }

  /* Fill in the header */
  pBuf = pParam;

  evt->hdr.event = HCI_HOST_INCOMING_EVT;

  /* Reuse header status for the event code */
  evt->hdr.status = HCI_ULP_EVENT_CODE; 
  evt->LEEventCode = *pBuf++;
  evt->numDevices = *pBuf++;
  evt->devInfo = (HCIEvt_DevInfo_t *)(evt + 1); /* Point to the next byte after the structure */

  /* Fill in the array of devInfo */
  for( i = 0; i < num; i++ )
  {
    evt->devInfo[i].eventType = *pBuf++;
    evt->devInfo[i].addrType = *pBuf++;
    osal_memcpy( evt->devInfo[i].addr, pBuf, BLE_ADDR_LEN );
    pBuf += BLE_ADDR_LEN;
    evt->devInfo[i].dataLen = *pBuf++;
    osal_memcpy( evt->devInfo[i].data, pBuf, evt->devInfo[i].dataLen );
    pBuf += evt->devInfo[i].dataLen;
  }
  
  return (uint8*)evt;
}

/*********************************************************************
 * @fn          hciHostCmdCompleteEvtProcess
 *
 * @brief       Parse and process the HCI Command Complete event packets 
 *
 * @param       pkt - HCI Event packet
 *
 * @return      none
 */
static void hciHostCmdCompleteEvtProcess( hciPacket_t *pkt )
{
  HCIEvt_CmdCompleteEvt_t *evt;
  uint8 i = 0;
  uint8 paramLen;
  uint16 cmdOpcode = 0;

  paramLen = pkt->pData[2] - 3; /* length of the return parameter */
  cmdOpcode = pkt->pData[4];

  /* Look up the return parameter parser */
  while ((HCIRtnParamTable[i].opCode != 0xFFFF) && (HCIRtnParamTable[i].hciFunc != NULL))
  {
    if (HCIRtnParamTable[i].opCode == cmdOpcode)
    {
      evt = ( HCIEvt_CmdCompleteEvt_t *)(HCIRtnParamTable[i].hciFunc)( paramLen, &pkt->pData[5] );
      break;
    }

    /* Next */
    i++;
  }

  if( evt == NULL )
  {
    return;
  }

  /* Fill out the common part of the event osal message */
  evt->hdr.event = HCI_HOST_INCOMING_EVT;
  evt->hdr.status = HCI_ULP_COMMAND_COMPLETE_EVENT_CODE;
  evt->cmdOpcode = cmdOpcode;
  evt->pReturnParam = (uint8 *)( evt + 1 ); /* Point to the address following the structure */

  /* Send the osal message to HCI task */
  osal_msg_send( HCI_TaskID, (uint8 *)evt );

  /* Update the command flow control manager */
  /* Todo */
}

/*********************************************************************
 * @fn          HCI_ReturnParamParser_LEReadBufSize
 *
 * @brief       Allocate osal msg and parse the HCI Command Complete 
 *              event packet for LE Read Buffer Size Command
 *
 * @param       len   - Length of the return parameter
 *              pData - Pointer to the return parameter
 *
 * @return      void * - Pointer to the osal message
 */
static uint8 *HCI_ReturnParamParser_LEReadBufSize( uint8 len, uint8 *pData )
{
  HCIEvt_CmdCompleteEvt_t *msg;
  HCIRetParam_LeReadBufSize_t *pParam;

  /* Verify the length of the packet */
  if( len != HCI_RET_PARAM_LEN_READ_BUF_SIZE )
  {
    return NULL;
  }

  msg = osal_mem_alloc( sizeof( HCIEvt_CmdCompleteEvt_t ) + sizeof( HCIRetParam_LeReadBufSize_t ) );

  if( msg != NULL )
  {
    pParam = ( HCIRetParam_LeReadBufSize_t *)( msg + 1 );
    pParam->status = pData[0];
    pParam->dataPktLen = BUILD_UINT16( pData[1], pData[2] );
    pParam->numDataPkts = pData[3];
    
    return (void *)msg;
  }

  return NULL;
}