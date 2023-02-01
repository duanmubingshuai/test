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
  Filename:       ll_MasterEndCauses.c
  Revised:        $
  Revision:       $

  Description:    This file contains the Link Layer (LL) handlers for the
                  various master end causes that result from a PHY task
                  completion. The file also contains the master end causes
                  related to connection termination, as well as common
                  termination routines used by the master.


*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"

#include "ll.h"
#include "ll_common.h"
#include "ll_enc.h"
#include "mcu.h"
#include "bus_dev.h"
#include "jump_function.h"
#include "global_config.h"
#include "ll_hw_drv.h"

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */
 
/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */
int32   connUpdateTimer = 0;                     // adjustment timer for conn update, could be negative if new interval greate than old one. Unit : 625us
/*******************************************************************************
 * GLOBAL VARIABLES
 */

//extern perStatsByChan_t* p_perStatsByChan;
//extern  uint8    g_conn_taskID;
//extern  uint16   g_conn_taskEvent;
extern llConns_t           g_ll_conn_ctx;

/*******************************************************************************
 * Prototypes
 */
uint8 llProcessMasterControlProcedures( llConnState_t *connPtr );
uint8 llSetupNextMasterEvent( void );

/*******************************************************************************
 * Functions
 */

/*******************************************************************************
 * @fn          llMasterEvt_TaskEndOk
 *
 * @brief       This function is used to handle the PHY task done end cause
 *              TASK_ENDOK that can result from one of three causes. First, a
 *              a packet was successfully received with MD=0 (i.e. no more Slave
 *              data) after having transmitted a packet with MD=0. Second, a
 *              received packet did not fit in the RX FIFO after transmitting
 *              a packet with MD=0. Third, a packet was received from the Slave
 *              while BLE_L_CONF.ENDC is true or after Timer 2 Event 2 occurs.
 *
 *              Note: The TASK_ENDOK end cause will also handle the TASK_NOSYNC,
 *                    TASK_RXERR, and TASK_MAXNACK end causes as well.
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
void llMasterEvt_TaskEndOk0( void )
{
    llConnState_t *connPtr;
    uint16         numPkts;
    int        i;
	uint32_t   T2, schedule_time;

  // get connection information
  connPtr = &conn_param[g_ll_conn_ctx.currentConn];

  // advance the connection event count
  connPtr->currentEvent = connPtr->nextEvent;

  // get the total number of received packets
  // Note: Since Auto-Flush is enabled, numRxFifoFull is incremented instead of
  //       numRxOk when there's no room in the FIFO. When Auto-Flush is
  //       disabled and there's no room in the FIFO, only numRxFifoFull is
  //       incremented for any kind of received packet.
  numPkts = ( rfCounters.numRxOk       +
              rfCounters.numRxNotOk    +
              rfCounters.numRxEmpty    +
              rfCounters.numRxIgnored  +
              rfCounters.numRxFifoFull );

  // collect packet error information
  connPtr->perInfo.numPkts   += numPkts;
  connPtr->perInfo.numCrcErr += rfCounters.numRxNotOk;
  //
  connPtr->perInfo.numEvents++;

//  // check if PER by Channel is enabled
//  if ( connPtr->perInfoByChan != NULL )
//  {
//     connPtr->perInfoByChan->numPkts[ PHY_GET_DATA_CHAN() ]   += numPkts;
//     connPtr->perInfoByChan->numCrcErr[ PHY_GET_DATA_CHAN() ] += rfCounters.numRxNotOk;
//  }

  // check if any data has been received
  // Note: numRxOk includes numRxCtrl
  // Note: numRxNotOk removed as 4.5.2 of spec says the timer is reset upon
  //       receipt of a "valid packet", which is taken to mean no CRC error.
  if ( rfCounters.numRxOk    || rfCounters.numRxIgnored ||
       rfCounters.numRxEmpty || rfCounters.numRxFifoFull 
       || connPtr->rx_crcok != 0)     // ever Rx CRC OK packet       
  {
    // yes, so update the supervision expiration count
    connPtr->expirationEvent = connPtr->currentEvent + connPtr->expirationValue;


    // clear flag that indicates we received first packet
    // Note: The first packet only really needs to be signalled when a new
    //       connection is formed. However, there's no harm in resetting it
    //       every time in order to simplify the control logic.
    // Note: True-Low logic is used here to be consistent with nR's language.
    connPtr->firstPacket = 0;

    //20181206 ZQ add phy change nofity
    //receiver ack notifty the host
//    if(connPtr->llPhyModeCtrl.isChanged==TRUE)
//    {
//        connPtr->llPhyModeCtrl.isChanged = FALSE;
//        llPhyModeCtrlUpdateNotify(connPtr,LL_STATUS_SUCCESS);
//    }
    
  }
  else // no data received, or packet received with CRC error
  {
    // check if we received any packets with a CRC error
    if ( rfCounters.numRxNotOk )
    {
      // clear flag that indicates we received first packet
      // Note: The first packet only really needs to be signalled when a new
      //       connection is formed. However, there's no harm in resetting it
      //       every time in order to simplify the control logic.
      // Note: True-Low logic is used here to be consistent with nR's language.
      connPtr->firstPacket = 0;   
    }
    else // no packet was received
    {
      // collect packet error information, 
      connPtr->perInfo.numMissedEvts++;
    }

    // check if we have a Supervision Timeout
    if ( connPtr->expirationEvent == connPtr->currentEvent )     // 2019-7-17, change from '==' to '<='
    {
      // check if the connection has already been established
      if ( connPtr->firstPacket == 0 )
      {
        // yes, so terminate with LSTO
        llConnTerminate( connPtr, LL_SUPERVISION_TIMEOUT_TERM );
      }
      else // no, so this is a failure to establish the connection
      {
        // so terminate immediately with failure to establish connection
        llConnTerminate( connPtr, LL_CONN_ESTABLISHMENT_FAILED_TERM );
      }

//#ifdef MULTI_ROLE   
	  ll_scheduler(LL_INVALID_TIME);           // link is terminated, update scheduler info
//#endif

      return;
    }
  }

  /*
  ** Process RX Data Packets
  */

  // check if there is any data in the Rx FIFO
  uint8_t  buffer_size;
  buffer_size = getRxBufferSize(connPtr);  
  for ( i = 0; i < buffer_size; i ++)     // note: i < getRxBufferSize()  will fail the loop
  {
    // there is, so process it; check if data was processed
    if ( llProcessRxData() == FALSE )
    {
      // it wasn't, so we're done
//        ll_scheduler(LL_INVALID_TIME);      
      break;
    }
  }  

  // check if this connection was terminated
  if ( !connPtr->active )
  {
//#ifdef MULTI_ROLE   
        ll_scheduler(LL_INVALID_TIME);
//#endif

    return;
  }

  /*
  ** Check Control Procedure Processing
  */
  if ( llProcessMasterControlProcedures( connPtr ) == LL_CTRL_PROC_STATUS_TERMINATE )
  {
//#ifdef MULTI_ROLE   
        ll_scheduler(LL_INVALID_TIME);           // link is termainte, update schedle info
//#endif

    return;
  }

  /*
  ** Process TX Data Packets
  */

  // copy any pending data to the TX FIFO
  llProcessTxData( connPtr, LL_TX_DATA_CONTEXT_POST_PROCESSING );

  // if any fragment l2cap pkt, copy to TX FIFO
//  l2capPocessFragmentTxData((uint16)connPtr->connId);

  /*
  ** Setup Next Slave Event Timing
  */

  // update next event, calculate time to next event, calculate timer drift,
  // update anchor points, setup NR T2E1 and T2E2 events
  if ( llSetupNextMasterEvent() == LL_SETUP_NEXT_LINK_STATUS_TERMINATE )            // PHY+ always return success here 
  {
    // this connection is terminated, so nothing to schedule
//#ifdef MULTI_ROLE   
		ll_scheduler(LL_INVALID_TIME);
//#endif

    return;
  }

  /*
  ** Schedule Next Task
  */
//#ifdef MULTI_ROLE  
//  schedule_time = ll_get_next_timer(g_ll_conn_ctx.currentConn);
  schedule_time = (connPtr->curParam.connInterval + connUpdateTimer) * 625;
  
  T2 = read_current_fine_time();

  // TODO: don't know the cause, here need add 32us to gain accurate timing  
  ll_scheduler(schedule_time - LL_TIME_DELTA(g_ll_conn_ctx.timerExpiryTick, T2)/* + 32*/);    // 10us: rough delay from timer expire to timer ISR
  
//#endif

  return;
}




/*******************************************************************************
 * @fn          llSetupNextMasterEvent
 *
 * @brief       This function is used to checks if an Update Parameters and/or an
 *              Update Data Channel has occurred, and sets the next data
 *              channel.
 *
 *              Side Effects:
 *              t2e1
 *              t2e2
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      boolean - Indicates whether a link terminate occurred:
 *                        LL_SETUP_NEXT_LINK_STATUS_TERMINATE: Terminate due to a LSTO.
 *                        LL_SETUP_NEXT_LINK_STATUS_SUCCESS: Do not terminate.
 */
uint8 llSetupNextMasterEvent0( void )
{
    llConnState_t *connPtr;
    //uint32        timeToNextEvt;      // TODO: use this parameter to update timer1 setting

//	connUpdateTimer = 0;                // for connUpdate case, the value is (winoffset + delta interval) , otherwise, it is zero

  // get pointer to connection info
  connPtr = &conn_param[g_ll_conn_ctx.currentConn];

  // update the next connection event count
  connPtr->nextEvent = (uint16)(connPtr->currentEvent + 1);

  /*
  ** Check for a Parameter Update
  */

  // check if there's a connection parameter udpate to the connection, and if
  // so, check if the update event is before or equal to the next active event
//  if ((connPtr->pendingParamUpdate == TRUE)  &&
//       (connPtr->nextEvent == connPtr->paramUpdateEvent))
//  {
//    // convert the new LSTO from time to an expiration connection event count
//    llConvertLstoToEvent( connPtr, &connPtr->paramUpdate );
//
//    // update the LSTO expiration count
//    // Note: This is needed in case the new connection fails to receive a
//    //       packet and a RXTIMEOUT results. The expiration count must
//    //       already be initialized based on the new event values.
//    connPtr->expirationEvent = connPtr->nextEvent + connPtr->expirationValue;
//
//    #if 0
//    // find the number of events between this event and the update parameter
//    // event, based on the original connection interval
//    // Note: The old connection interval must be used!
//    timeToNextEvt = (uint32)connPtr->curParam.connInterval +
//                    (uint32)connPtr->paramUpdate.winOffset;     
//    #endif
//      
////#ifdef MULTI_ROLE
//    // schedule next timer after processing the connection event, no re-schedule required    
//    // for conn update case, schedule time should be (old interval + offset), in llMasterEvt_TaskEndOk, sheduler using new interval
//    connUpdateTimer = connPtr->paramUpdate.winOffset + (connPtr->curParam.connInterval - connPtr->paramUpdate.connInterval);  
////#endif
//
//    // update the current parameters from the new parameters
//    // Note: The new parameter connTimeout is not needed since once it is
//    //       converted to connection events, its value is maintained by
//    //       expirationValue. The new parameter winSize is only needed in case
//    //       there's a RX timeout.
//    connPtr->curParam.connInterval = connPtr->paramUpdate.connInterval;
//    connPtr->curParam.winSize      = connPtr->paramUpdate.winSize;
//    connPtr->curParam.slaveLatency = connPtr->paramUpdate.slaveLatency;
//
//    // convert the Control Procedure timeout into connection event count
//    llConvertCtrlProcTimeoutToEvent( connPtr );
//
//    // clear the pending flag
//    connPtr->pendingParamUpdate = FALSE;
//
//    // notify the Host
//    // Note: The HCI spec says the LE Connection Update Complete event shall
//    //       be generated after the connection parameters have been applied
//    //       by the Controller. This is different from the Slave, which only
//    //       reports this event if the connection interval, slave latency,
//    //       and/or connection timeout are changed.
//    // Note: The values are kept in units of 625us, so they must be
//    //       converted back to their what's used at the interface.
//    LL_ConnParamUpdateCback( (uint16)connPtr->connId,
//                             connPtr->paramUpdate.connInterval >> 1,
//                             connPtr->paramUpdate.slaveLatency,
//                             connPtr->paramUpdate.connTimeout >> 4 );
//
////#ifdef MULTI_ROLE
////#else
////    // next conn event timer start when trigger current conn event, if conn parameter update is scheduled, update the timer    
////    uint32 elapse_time, next_time, calibrate;
////  
////    next_time = timeToNextEvt * 625;
////    calibrate = 10;     // consider re-schedule timer1 cost
////    elapse_time = (CP_TIM1->LoadCount - CP_TIM1->CurrentCount) >> 2;
////  
////    next_time = next_time - elapse_time - calibrate;
////  
////    // re-schedule timer1
////    ll_schedule_next_event(next_time);                 
////#endif    
//  }
//  else // no parameter update, so...
//  {
//    // time to next event continues as usual
//    // timeToNextEvt = connPtr->curParam.connInterval;     // to remove
//  }


  // check for a Data Channel Update and calculate and set next data channel
  // Note: The Data Channel Update must come after the Parameters Update in
  //       case the latter updates the next event count.
  llSetNextDataChan( connPtr );

//  llSetNextPhyMode( connPtr );

  return( LL_SETUP_NEXT_LINK_STATUS_SUCCESS );
}

/*******************************************************************************
 * @fn          llProcessMasterControlPacket
 *
 * @brief       This routine is used to process incoming Control packet.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to BLE LL Connection.
 * @param       pBuf    - Pointer to Control packet payload.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llProcessMasterControlPacket( llConnState_t *connPtr,
                                   uint8         *pBuf )
{
  // unknown data PDU control packet received so save the type
  connPtr->unknownCtrlType = *pBuf++;

  // schedule the output of the control packet
  llEnqueueCtrlPkt( connPtr, LL_CTRL_UNKNOWN_RSP );

  return;
}



/*******************************************************************************
 * @fn          llProcessMasterControlProcedures
 *
 * @brief       This function is used to process any control procedures that
 *              may be active.
 *
 *              Note: There can only be one active control procedure at a time.
 *
 *              Note: It is assumed the NR counters have been updated at the
 *                    end of the task before calling this routine.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      uint8 - Status of control procedure processing, which can be:
 *                      LL_CTRL_PROC_STATUS_SUCCESS: Continue normally.
 *                      LL_CTRL_PROC_STATUS_TERMINATE: We have terminated.
 */
uint8 llProcessMasterControlProcedures( llConnState_t *connPtr )
{
    // check if there are any control packets ready for processing
  while ( connPtr->ctrlPktInfo.ctrlPktCount > 0 )
  {
    // processing based on control packet type at the head of the queue
    switch( connPtr->ctrlPktInfo.ctrlPkts[ 0 ] )
    {
      case LL_CTRL_TERMINATE_IND:
        // check if the control packet procedure is is active
        if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
        {
          // we have already place packet on TX FIFO, so check if its been ACK'ed
          if ( rfCounters.numTxCtrlAck )
          {
            // done with this control packet, so remove from the processing queue
            llDequeueCtrlPkt( connPtr );

            // yes, so process the termination
            // Note: No need to cleanup control packet info as we are done.
            llConnTerminate( connPtr, LL_HOST_REQUESTED_TERM );

            return( LL_CTRL_PROC_STATUS_TERMINATE );
          }
          else // no done yet
          {
            // check if a termination control procedure timeout has occurred
            // Note: No need to cleanup control packet info as we are done.
            if ( --connPtr->ctrlPktInfo.ctrlTimeout == 0 )
            {
              // we're done waiting, so end it all
              // Note: No need to cleanup control packet info as we are done.
              llConnTerminate( connPtr, LL_CTRL_PKT_TIMEOUT_HOST_TERM );

              return( LL_CTRL_PROC_STATUS_TERMINATE );
            }
            else
            {
              //  control packet stays at head of queue, so exit here
              return( LL_CTRL_PROC_STATUS_SUCCESS );
            }
          }
        }
        else // control packet has not been put on the TX FIFO yet
        {
          // so try to put it there; being active depends on a success
          connPtr->ctrlPktInfo.ctrlPktActive = llSetupTermInd( connPtr );

          // Note: Two cases are possible:
          //       a) We successfully placed the packet in the TX FIFO.
          //       b) We did not.
          //
          //       In case (a), it may be possible that a previously just
          //       completed control packet happened to complete based on
          //       rfCounters.numTxCtrlAck. Since the current control
          //       procedure is now active, it could falsely detect
          //       rfCounters.numTxCtrlAck, when in fact this was from the
          //       previous control procedure. Consequently, return.
          //
          //       In case (b), the control packet stays at the head of the
          //       queue, and there's nothing more to do. Consequently, return.
          //
          //       So, in either case, return.
          return( LL_CTRL_PROC_STATUS_SUCCESS );
        }
		case LL_CTRL_UNKNOWN_RSP:
				// try to place control packet in the TX FIFO
				// Note: Since there are no dependencies for this control packet, we
				//		 do not have to bother with the active flag.
				if ( llSetupUnknownRsp( connPtr ) == TRUE )
				{
				  // all we have to do is put this control packet on the TX FIFO, so
				  // remove control packet from the processing queue and drop through
				  llDequeueCtrlPkt( connPtr );
				}
				else // not done yet
				{
				  // control packet stays at head of queue, so exit here
				  return( LL_CTRL_PROC_STATUS_SUCCESS );
				}
				break;

    }
  }

  return( LL_CTRL_PROC_STATUS_SUCCESS );
}

//#pragma O0
// A2 multi-connection
//uint8 ll_processMissMasterEvt(uint8 connId)
//{
//    llConnState_t *connPtr;
//    
////    LOG("-M ");
//
//    connPtr = &conn_param[connId];
//			
//    connPtr->rx_crcok = 0; 
//    connPtr->rx_timeout = 1;
//
////	connPtr->pmCounter.ll_conn_event_cnt ++;
////	connPtr->pmCounter.ll_conn_event_timeout_cnt ++;
////	connPtr->pmCounter.ll_miss_master_evt_cnt ++;
//	
////    if(p_perStatsByChan!=NULL)
////	    p_perStatsByChan->rxToCnt[connPtr->currentChan]++;
//			
//	// Tx done OK counter
//	rfCounters.numTxDone = 0;	
//	
//    // update the numTxCtrlAck counter, add on 2017-11-15
//    rfCounters.numTxCtrlAck = 0;
//
//    // this counter is set in function LL_master_conn_event() used in function llProcessMasterControlProcedures()
//	rfCounters.numTxCtrl = 0;
//	
//    // advance the connection event count
//    connPtr->currentEvent = connPtr->nextEvent;
//
//    connPtr->perInfo.numEvents++;
//	  
//    connPtr->perInfo.numMissedEvts++;
//
//    // check if we have a Supervision Timeout
//    if ( connPtr->expirationEvent <= connPtr->currentEvent )	 // 2019-7-17, change from '==' to '<='
//    {
//        // check if the connection has already been established
//        if ( connPtr->firstPacket == 0 )
//        {
//			// yes, so terminate with LSTO
//			llConnTerminate( connPtr, LL_SUPERVISION_TIMEOUT_TERM );
//        }
//        else // no, so this is a failure to establish the connection
//        {
//			// so terminate immediately with failure to establish connection
//			llConnTerminate( connPtr, LL_CONN_ESTABLISHMENT_FAILED_TERM );
//        }
//	
//        return LL_PROC_LINK_TERMINATE;
//    }
//
//	// no Rx packet
//	
//	/*
//	** Check Control Procedure Processing
//	*/
//	if ( llProcessMasterControlProcedures( connPtr ) == LL_CTRL_PROC_STATUS_TERMINATE )
//	{
//		// this connection is terminated, so nothing to schedule
//		return LL_PROC_LINK_TERMINATE;
//	}
//	
//	/*
//	** Process TX Data Packets, no new data for miss event
//	*/	
//	
//	/*
//	** Setup Next master Event Timing
//	*/
//	
//	// update next event, calculate time to next event, calculate timer drift,
//	// update anchor points, setup NR T2E1 and T2E2 events
//	if ( llSetupNextMasterEvent() == LL_SETUP_NEXT_LINK_STATUS_TERMINATE )  // PHY+ always return success here 
//	{
//		// this connection is terminated, so nothing to schedule
//		return LL_PROC_LINK_TERMINATE;
//	}
//
//	// update scheduler information
//	g_ll_conn_ctx.scheduleInfo[connId].remainder += (connPtr->curParam.connInterval + connUpdateTimer) * 625;
//
//    // connection event notify
////    if (g_conn_taskID != 0)
////        osal_set_event(g_conn_taskID, g_conn_taskEvent);
//
//	return LL_PROC_LINK_KEEP;
//	
//}




/*******************************************************************************
 */
