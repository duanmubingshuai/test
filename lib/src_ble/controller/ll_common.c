/*******************************************************************************
  Filename:       ll_common.c
  Revised:        
  Revision:       

  Description:    This file contains common functions used by the Link Layer
                  Bluetooth Low Energy (BLE) Controller.



  IMPORTANT: 
*******************************************************************************/

#include <string.h>
#include "timer.h"
#include "ll_buf.h"
#include "ll_def.h"
#include "ll.h"
#include "ll_common.h"
#include "hci_event.h"
#include "osal_bufmgr.h"
#include "bus_dev.h"
#include "ll_enc.h"
#include "jump_function.h"
#include "global_config.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "att.h"
#include "osal_memory.h"
#include "log.h"

/*******************************************************************************
 * MACROS
 */

#define ADV_BASE_IDX    37

/*******************************************************************************
 * CONSTANTS
 */
// Master Sleep Clock Accurracy, in PPM
// Note: Worst case range value is assumed.
const uint16 SCA[] = {500, 250, 150, 100, 75, 50, 30, 20};


/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
	
//extern void  *taskEndCauses[];

//extern int current_base_time;     // uint is  s
//extern int current_fine_time;     // uint is  us
extern uint32 llWaitingIrq;
extern uint32_t llScanT1;
extern uint8    g_dle_taskID;
extern uint16   g_dle_taskEvent;
extern uint8    g_phyChg_taskID;
extern uint16   g_phyChg_taskEvent;
extern uint8_t             llSecondaryState;            // secondary state of LL

extern struct buf_tx_desc g_tx_ext_adv_buf;    

/*******************************************************************************
 * Functions
 */


/*******************************************************************************
 * @fn          llMemCopySrc
 *
 * @brief      Generic memory copy. This function copies the source location to
 *             the destination location in bytes, and returns the next source
 *             address.
 *
 * input parameters
 *
 * @param       pDst - Destination address.
 * @param       pSrc - Source address.
 * @param       len  - Number of bytes to copy.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Pointer to next source address.
 */
uint8 *llMemCopySrc( uint8 *pDst, uint8 *pSrc, uint8 len )
{
  while ( len-- )
  {
    *pDst++ = *pSrc++;
  }

  return( pSrc );      // return pSrc
}

/*******************************************************************************
 * @fn          llMemCopyDst
 *
 * @brief      Generic memory copy. This function copies the destination
 *             location to the source location in bytes, and returns the
 *             next destination address.
 *
 * input parameters
 *
 * @param       pDst - Destination address.
 * @param       pSrc - Source address.
 * @param       len  - Number of bytes to copy.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Pointer to next destination address.
 */
uint8 *llMemCopyDst( uint8 *pDst, uint8 *pSrc, uint8 len )
{
  while ( len-- )
  {
    *pDst++ = *pSrc++;
  }

  return( pDst );     // return pDst
}

/*******************************************************************************
 * @fn          llProcessChanMap
 *
 * @brief       This function is used to convert the channel map provided by
 *              the HCI from a bit map format of used data channels, to a table
 *              of consecutively ordered entries. This is needed by the
 *              getNextDataChan algorithm, and should be done whenever the data
 *              channel map is updated.
 *
 *              The following connection globals are side effected:
 *              numUsedChans - Count of the number of used data channels found.
 *              chanMapTable - Table of used data channels in ascending order.
 *
 * input parameters
 *
 * @param       connPtr - A pointer to the current LL connection data.
 * @param       chanMap - A five byte array containing one bit per data channel
 *                        where a 1 means the channel is "used".
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llProcessChanMap( llConnState_t *connPtr,
                       uint8         *chanMap )
{
  uint8_t i, j;

  // channels 37..39 are not data channels and these bits should not be set
  chanMap[LL_NUM_BYTES_FOR_CHAN_MAP - 1] &= 0x1F;

  // clear the count of the number of used channels
  connPtr->numUsedChans = 0;

  // the used channel map uses 1 bit per data channel, or 5 bytes for 37 chans
  for (i = 0; i < LL_NUM_BYTES_FOR_CHAN_MAP; i++)
  {
    // replace the current channel map with the update channel map
    // Note: This needs to be done so that llGetNextDataChan can more
    //       efficiently determine if the next channel is used.
    connPtr->chanMap[i] = chanMap[i];

    // for each channel given by a bit in each of the bytes
    // Note: When i is on the last byte, only 5 bits need to be checked, but
    //       it is easier here to check all 8 with the assumption that the rest
    //       of the reserved bits are zero.
    for (j=0; j<8; j++)
    {
      // check if the channel is used; only interested in used channels
      if ( (chanMap[i] >> j) & 1 )
      {
        // sequence used channels in ascending order
        connPtr->chanMapTable[ connPtr->numUsedChans ] = (i*8U)+j;

        // count it
        connPtr->numUsedChans++;
      }
    }
  }
}
// get next channel

/*******************************************************************************
 * @fn          llGetNextDataChan
 *
 * @brief       This function returns the next data channel for a LL connection
 *              based on the previous data channel, the hop length, and the
 *              number of connection intervals to the next active event. If the
 *              derived channel is "used", then it is returned. If the derived
 *              channel is "unused", then a remapped data channel is returned.
 *
 *              The following connection globals are side effected:
 *              connEvtChan - The calculated connection event channel.
 *
 *              Note: connEvtChan is left updated even if the remapped channel
 *                    is returned.
 *
 *              Note: It is assumed it is safe to check i to see if the entire
 *                    table was searched.
 *
 * input parameters
 *
 * @param       connPtr   - A pointer to the current LL connection data.
 * @param       numEvents - The number of skipped events to base the next
 *                          data channel calculation on.
 * output parameters
 *
 * @param       None.
 *
 * @return      The next data channel to use.
 */
uint8 llGetNextDataChan( llConnState_t *connPtr,
                                   uint16         numEvents )
{
		
	 connPtr->nextChan = ( connPtr->currentChan +
                        (connPtr->hop *numEvents) ) % LL_MAX_NUM_DATA_CHAN;

  if ( connPtr->chanMap[ connPtr->nextChan >> 3 ] & (1 << (connPtr->nextChan % 8)) )
  {
    // used channel
    return( connPtr->nextChan );
  }
  else // unused channel
  {
    return( connPtr->chanMapTable[ connPtr->nextChan % connPtr->numUsedChans ] );
  }
	 
}

#if 0
static uint8_t chan_rev_8(uint8_t b)
{
	b = (((uint32_t)b * 0x0802LU & 0x22110LU) |
	     ((uint32_t)b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;

	return b;
}

static uint16_t chan_perm(uint16_t i)
{
	return (chan_rev_8((i >> 8) & 0xFF) << 8) | chan_rev_8(i & 0xFF);
}

static uint16_t chan_mam(uint16_t a, uint16_t b)
{
	return ((uint32_t)a * 17U + b) & 0xFFFF;
}

static uint16_t chan_prn(uint16_t counter, uint16_t chan_id)
{
	uint8_t iterate;
	uint16_t prn_e;

	prn_e = counter ^ chan_id;

	for (iterate = 0U; iterate < 3; iterate++) {
		prn_e = chan_perm(prn_e);
		prn_e = chan_mam(prn_e, chan_id);
	}

	prn_e ^= chan_id;

	return prn_e;
}
#endif

/*******************************************************************************
 * @fn          llSetNextDataChan
 *
 * @brief       This function finds and sets the data channel for the next
 *              active connection event. This routine also checks if a data
 *              channel update procedure is pending, and if so, whether it
 *              will take place between the current event (exclusive) and the
 *              next active event (inclusive). If so, the next data is adjusted
 *              for the next active event keeping slave latency in effect.
 *
 * input parameters
 *
 * @param       *connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llSetNextDataChan0( llConnState_t *connPtr )
{
  uint16 numEvents;
//    uint8  connId;
//    
//    connId = connPtr->connId;

  /*
  ** Check for a Data Channel Update and Set the Next Data Channel
  **
  ** Note: The Data Channel Update must come after the Parameters Update in
  **       case the latter updates the next event count.
  */

  // before finding the next channel, save it as the current channel
  // Note: nextChan is now called unmappedChannel in the spec.
  // Note: currentChan is now called lastUnmappedChannel in the spec.
  connPtr->currentChan = connPtr->nextChan;

  // find the number of events to the next event
  numEvents = llEventDelta( connPtr->nextEvent, connPtr->currentEvent );

  // check if there's a pending update to the data channel for next event and
  // whether the update event count is prior to the next active event count
  if ( ( connPtr->pendingChanUpdate == TRUE ) &&
       ( llEventInRange( connPtr->currentEvent,
                         connPtr->nextEvent,
                         connPtr->chanMapUpdateEvent ) ) )
  {
    //=================================================================
    //20190803 ZQ:
    // store the old chanMap for restore in manualy disable slave latency
    //20190806 ZQ:
    //store the old chanMap while nextEvent-chanMapUpdateEvt > 3
    if(llEventDelta(connPtr->nextEvent,connPtr->chanMapUpdateEvent )>3)
    {
        connPtr->preChanMapUpdate.chanMapUpdated = TRUE;
        connPtr->preChanMapUpdate.chanMapUpdateEvent = connPtr->chanMapUpdateEvent;
        osal_memcpy(&(connPtr->preChanMapUpdate.chanMap[0]), &(connPtr->chanMap[0]), 5);
        //LOG("[ST preChanMap] c%d n%d u%d\r\n",connPtr->currentEvent,connPtr->nextEvent,connPtr->chanMapUpdateEvent);
    }
     
    //=================================================================
    //update data channel table based on pending data channel update
    //ZQ 20200207 
    //llProcessChanMap(connPtr, chanMapUpdate.chanMap);
    llProcessChanMap(connPtr, connPtr->chanMapUpdate.chanMap);
   
    connPtr->pendingChanUpdate = FALSE;
    
  }

  //20190806 ZQ
  //clear the preChanMapUpdate flg
  //when current_event is runover the chanMapUpdated_evt, clear flg
  if(connPtr->preChanMapUpdate.chanMapUpdated ==TRUE)
  {
    if(llEventDelta(connPtr->preChanMapUpdate.chanMapUpdateEvent, connPtr->currentEvent )>32768)
    {
        connPtr->preChanMapUpdate.chanMapUpdated =FALSE;
        //LOG("[END preChanMap] c%d n%d u%d\r\n",connPtr->currentEvent,connPtr->nextEvent,connPtr->chanMapUpdateEvent);
    }
  }

      connPtr->currentChan = llGetNextDataChan(connPtr,  numEvents );
	
  return;
}

void llSetNextPhyMode0( llConnState_t *connPtr )
{
//  uint16 numEvents;
  uint8 lastPhyMode;

  // find the number of events to the next event
//  numEvents = llEventDelta( connPtr->nextEvent, connPtr->currentEvent );

  // check if there's a pending update to the data channel for next event and
  // whether the update event count is prior to the next active event count
  if ( ( connPtr->pendingPhyModeUpdate == TRUE ) &&
       ( llEventInRange( connPtr->currentEvent,
                         connPtr->nextEvent,
                         connPtr->phyModeUpdateEvent ) ) )
  {
    lastPhyMode = connPtr->llRfPhyPktFmt;
    // update data channel table based on pending data channel update
    if((connPtr->phyUpdateInfo.m2sPhy & LE_1M_PHY) >0)
    {
        connPtr->llRfPhyPktFmt = PKT_FMT_BLE1M;
    }
    else if((connPtr->phyUpdateInfo.m2sPhy & LE_2M_PHY) >0)
    {
        connPtr->llRfPhyPktFmt = PKT_FMT_BLE2M;
    }
//    else if((connPtr->phyUpdateInfo.m2sPhy & LE_CODED_PHY) >0)
//    {
//        connPtr->llRfPhyPktFmt = PKT_FMT_BLR500K;
//    }

    if(lastPhyMode != connPtr->llRfPhyPktFmt)
        connPtr->llPhyModeCtrl.isChanged=TRUE;
    // clear the pending flag
    connPtr->pendingPhyModeUpdate = FALSE;
  }



  return;
}



/*******************************************************************************
 * @fn          llSetupInit
 *
 * @brief       This function
 *
 * input parameters
 *
 * @param       connId - Connection ID that Init will attempt to start.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
//void llSetupInit( uint8 connId )
//{
//
//    // Hold off interrupts.
//    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();	
//
//    // construct CONN REQ message
//    
//    // configure scan 
//    // reset all FIFOs; all data is forfeit
//    ll_hw_rst_tfifo();
//    ll_hw_rst_rfifo();
//  
//	set_crc_seed(ADV_CRC_INIT_VALUE); // crc seed for adv is same for all channels
//
//	set_access_address(ADV_SYNCH_WORD);
//
//	set_channel(initInfo.nextScanChan);
//
//	set_whiten_seed(initInfo.nextScanChan); 
//
//	set_max_length(0xff);  
//    
//    ll_hw_set_rx_timeout(10000);
//    
//    ll_hw_set_srx();
//    
//    ll_hw_go();  
//  
//    llScanT1 = read_current_fine_time();
//  
//    llWaitingIrq = TRUE;  
//            
//    // set LL state
//    llState = LL_STATE_INIT;
//
//    HAL_EXIT_CRITICAL_SECTION(); 
//
//    ll_debug_output(DEBUG_LL_STATE_INIT);
//    
//    return;
//}

/*******************************************************************************
 * @fn          llSetupScan
 *
 * @brief       This function readies the device as a Scanner.
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
void llSetupScan0( uint8 chan )
{
    // Hold off interrupts.
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();	


    // check if the Scanner should be initialized
    if ( scanInfo.initPending == TRUE )
    {
        // setup Scan initalization
        llSetupScanInit();

        // only need to do this once
        scanInfo.initPending = FALSE;
    }

    //support rf phy change
    rf_phy_change_cfg(g_rfPhyPktFmt);

    // reset all FIFOs; all data is forfeit
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
  
	set_crc_seed(ADV_CRC_INIT_VALUE); // crc seed for adv is same for all channels

	set_access_address(ADV_SYNCH_WORD);

	set_channel(chan);

	set_whiten_seed(chan); 

	set_max_length(0xff);  
    
    ll_hw_set_rx_timeout(10000);     // 10000us
    
    ll_hw_set_srx();
    ll_hw_ign_rfifo(LL_HW_IGN_CRC|LL_HW_IGN_EMP);	
    
    ll_hw_go();  
  
    llScanT1 = read_current_fine_time();
  
    llWaitingIrq = TRUE;      

	HAL_EXIT_CRITICAL_SECTION(); 
    
//    ll_debug_output(DEBUG_LL_STATE_SCAN);

    return;
}

/*******************************************************************************
 * @fn          llSetupScanInit
 *
 * @brief       This function performs common initialization for when the device
 *              is being setup as a Scanner.
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
void llSetupScanInit( void )
{
    // set scan backoff upper limit
    scanInfo.scanBackoffUL = 1;

    // set the backoff counter. Note: Initially, this value is set to one, per the spec.
    scanInfo.currentBackoff = 1;
    
    return;
}


/*******************************************************************************
 * This function dequeues a TX data packet from the front of the data queue.
 *
 * Public function defined in hci_c_data.h.
 */
txData_t *llDequeueDataQ( llDataQ_t *pDataQ )
{
  txData_t      *pTxData = NULL;

  _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

  // check if the data queue is not empty
  if ( pDataQ->head != NULL )
  {
    // at least one entry already on the data queue, so remove packet
    pTxData        = pDataQ->head;
    pDataQ->head   = pTxData->pNext;
    pTxData->pNext = NULL;

    // check if queue is now empty
    if ( pDataQ->head == NULL )
    {
      pDataQ->tail = NULL;
    }
  }

  HAL_EXIT_CRITICAL_SECTION();

  return( pTxData );
}

/*******************************************************************************
 * @fn          llAllocConnId
 *
 * @brief       This function is called to get the next free LL connection. If
 *              all available connections are in use, NULL is returned.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      A valid connection pointer, or NULL.
 */
llConnState_t *llAllocConnId( void )
{
	uint8 i;
    // find first free connection
    for (i = 0; i < g_maxConnNum; i++)
    {
        llConnState_t *connPtr = &conn_param[i];
	    if (connPtr->allocConn == FALSE)
        {
			connPtr->allocConn = TRUE;
			llResetConnId(connPtr->connId);
   	        return( connPtr );
	    }
      
    }	
	
    return( NULL );



}

/*******************************************************************************
 * @fn          llReleaseConnId0
 *
 * @brief       This function is called to mark a connection resource as free.
 *
 *              Note: If there are other active connections, the scheduler will
 *                    find the next one to schedule, and will set currentConn.
 *
 * input parameters
 *
 * @param       connPtr - A pointer to the connection resource to free.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
 
void llReleaseConnId0( llConnState_t *connPtr )
{
//	uint8_t  next;
    // check that this connection is active
    // Note: If a cancel is issued, this connection may be released even though
    //       it isn't yet active.
    if ( connPtr->active )
    {
        // mark connection as inactive
        connPtr->active = FALSE;
    }

	if (connPtr->allocConn == TRUE)
    {
        connPtr->allocConn = FALSE;
		// comment for jack
		// only one connection , after terminate connection , next automatically invalid
//        next = ll_get_next_active_conn(connPtr->connId);
//		if (next == LL_INVALID_CONNECTION_ID)
        {                  // no more active connection
            llState = LL_STATE_IDLE;  

            // if secondary state not idle/pending state, transit it to main state
            // pending states are processed in LL_IRQHandler
//            if ((llSecondaryState == LL_SEC_STATE_INIT) || (llSecondaryState == LL_SEC_STATE_INIT_PENDING))
//            {
//			    llState = LL_STATE_INIT;
//				llSecondaryState = LL_SEC_STATE_IDLE;
//            }
//			else 
			if ( (llSecondaryState == LL_SEC_STATE_SCAN) || (llSecondaryState == LL_SEC_STATE_SCAN_PENDING))
            {
			    llState = LL_STATE_SCAN;
			    llSecondaryState = LL_SEC_STATE_IDLE;
            }
		}

    }
	
	
    // add by HZF
    llResetConnId(connPtr->connId);
    connPtr->ctrlDataIsProcess = 0;   // seding a control packet or not
    connPtr->ctrlDataIsPending = 0;   // control packet is pending to be sent

    reset_conn_buf(connPtr->connId);

	memset((uint8_t *)&connPtr->pmCounter , 0, sizeof(llLinkStatistics_t) );    
    return;
}

/*******************************************************************************
 * @fn          llConnCleanup
 *
 * @brief       This function is used to clean up the LL connection data
 *              structures, queued Tx data packet, and the connection task.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llConnCleanup( llConnState_t *connPtr )
{


  // free any pending Tx data queue headers and buffers, if any
  while( connPtr->txDataQ.head != NULL )
  {
    txData_t *pTxData;

    // okay to remove from the front of the data queue
    pTxData = llDequeueDataQ( &connPtr->txDataQ );

    // free the memory allocated in higher layer
    osal_bm_free( pTxData );

    // inc the number completed
    numComplPkts++;
  }

  // release the connection
  // Note: Releasing the connection resets the TX/RX FIFOs.
  llReleaseConnId( connPtr );

  // free the associated task block
  //llFreeTask( &connPtr->llTask );

  return;
}

/*******************************************************************************
 * @fn          llReleaseAllConnId
 *
 * @brief       This function is called to free all connection resources.
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
void llReleaseAllConnId( void )
{

  return;
}

/*******************************************************************************
 * @fn          llConnTerminate0
 *
 * @brief       This function is used to handle the commmon termination
 *              operations for a LL connection.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 * @param       reason  - The reason code for the termination, to send to Host.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llConnTerminate0( llConnState_t *connPtr,
                      uint8          reason )
{   
	// let the application know
	LL_DisconnectCback( (uint16)connPtr->connId, reason );

	// cleanup the connection data structures and task
	llConnCleanup( connPtr );
	
  return;
}			  


/*******************************************************************************
 * @fn          llWriteTxData0
 *
 * @brief       This function is used to place Host/HCI data in the TX FIFO for
 *              transmission. If there is no reason to stall transmission (due
 *              to encryption control procedures, physical flow control, etc.),
 *              then the data is placed in the TX FIFO for transmission.
 *
 * input parameters
 *
 * @param       *connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_OUT_OF_TX_MEM,
 *              LL_STATUS_WARNING_TX_DISABLED
 */
uint8 llWriteTxData0( llConnState_t *connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8         *pBuf )
{
	  uint8_t idx;
	
	  // check if data output is allowed
	  // Note: Data output can be disabled so that the TX FIFO can empty.
	  //       This is needed for encryption pause, for example.
	  if ( connPtr->txDataEnabled == TRUE )
	  {
	      // check if there's enough space in TX FIFO				
	      if(getTxBufferFree(connPtr) > 0)   
	      {
	          // check if packet needs to be encrypted
	          if ( connPtr->encEnabled )
	          {
	              LL_ENC_Encrypt( connPtr,
                        pktHdr,
                        pktLen,
                        pBuf );

	              // add the MIC to length
	              pktLen += LL_PKT_MIC_LEN;
	          }
      					
	          idx = get_tx_write_ptr(connPtr);//find_free_tx_buf();
	          connPtr->ll_buf.tx_conn_desc[idx]->header =  pktLen<<8 | pktHdr;
			
	          memcpy (connPtr->ll_buf.tx_conn_desc[idx]->data, pBuf , pktLen );
			
	          connPtr->ll_buf.tx_conn_desc[idx]->valid =1;
			 
	          //buf.tx_conn_pkts++;
			
	          //MOVE_IDX_ONE (tx_head_idx );
	          update_tx_write_ptr(connPtr);
	          //tx_head_idx = (uint8_t)get_tx_write_ptr();
			
	          return( LL_STATUS_SUCCESS );			
	      }
	      else
	      {
	          return( LL_STATUS_ERROR_OUT_OF_TX_MEM );
	      }
	  }
	  else // TX is disabled
	  {
	      return( LL_STATUS_WARNING_TX_DISABLED );
	  }
}

/*******************************************************************************
 * @fn          llProcessTxData
 *
 * @brief       This function is used to check if HCI TX data that has been
 *              previously stalled from sending (due to  encryption control
 *              procedures, physical flow control, etc.) can now be transmitted.
 *              If so, it attempts to send it, and if successful, notifies the
 *              HCI that the buffer has been transmitted, and the LL is ready
 *              for more data.
 *
 * input parameters
 *
 * @param       *connPtr - Pointer to a connection.
 * @param        context - LL_TX_DATA_CONTEXT_SEND_DATA |
 *                         LL_TX_DATA_CONTEXT_TX_ISR    |
 *                         LL_TX_DATA_CONTEXT_POST_PROCESSING
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llProcessTxData0( llConnState_t *connPtr, uint8 context )
{
    uint8         *pBuf;
//    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

  //TODO
  if(context==LL_TX_DATA_CONTEXT_SEND_DATA)
        return;
    // try to put as many packets into the TX FIFO as possible
    while( connPtr->txDataQ.head != NULL )
    {
        // point to packet header
        pBuf = (uint8 *)(connPtr->txDataQ.head + 1);
        
        // check if TX is enabled and if there is room in TX FIFO
        
        if ( llWriteTxData( connPtr, pBuf[1], pBuf[0], &pBuf[2] ) == LL_STATUS_SUCCESS )
        {
            // remove entry from TX queue and free the buffer
            osal_bm_free( llDequeueDataQ( &connPtr->txDataQ ) );
            
            numComplPkts ++;
            
            // update counter
            connPtr->pmCounter.ll_hci_to_ll_pkt_cnt += numComplPkts;   
//            LOG("TX:%d\n", connPtr->connId);            
            
            continue;
        }
        
        // unable to complete the write, so keep packet on queue
        break;
    }
  
    // check if we completed any packets
    // The Number of Completed Packets event is sent when the number of completed
    // packets is equal to or greater than the user specified limit, which can
    // range from 1 to the LL_MAX_NUM_DATA_BUFFERS (default is one). If the
    // number of completed packets is less than the limit, then this event is only
    // sent if the user indicated that this event to be sent at the end of the
    // connection event.
    // Note: Spec indicates that while the Controller has HCI data packets in its
    //       buffer, it must keep sending the Number Of Completed Packets event
    //       to the Host at least periodically, until it finally reports that all
    //       the pending ACL Data Packets have been transmitted or flushed.
    //       However, this can potentially waste a lot of time sending events
    //       with a number of completed packets set to zero, so for now, this
    //       will not be supported.
    if ( (numComplPkts > 0) &&
         (numComplPkts >= numComplPktsLimit) ) // ||
          //((context == LL_TX_DATA_CONTEXT_POST_PROCESSING) && numComplPktsFlush)) )
    {
        uint16 connId = connPtr->connId;
        uint16 numCompletedPackets = numComplPkts;
                
        // and send credits to the Host
        HCI_NumOfCompletedPacketsEvent( 1,
                                        &connId,
                                        &numCompletedPackets );
        
        // clear count
        numComplPkts = 0;
    }
  
//    HAL_EXIT_CRITICAL_SECTION();
  
    return;
}

/*******************************************************************************
 * @fn          llCtrlAllowInEncProc
 *
 * @brief       This routine check whether the received control PDU type is allowed 
 *              during start encrypt procedure. Ref to 5.1.3.1 Encryption Start Procedure
 *                "After these Data Channel PDUs are acknowledged,
 *                 the Link Layer of the master shall only send Empty PDUs or LL_ENC_REQ,
 *                 LL_START_ENC_RSP, LL_TERMINATE_IND, LL_REJECT_IND or
 *                 LL_REJECT_EXT_IND PDUs."
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      TRUE: Rx packet processed.
 *              FALSE: Unable to process packet.
 */
static uint8 llCtrlAllowInEncProc(uint8 opCode)
{
    if (opCode == LL_CTRL_ENC_REQ       ||             // normally should not received
        opCode == LL_CTRL_START_ENC_RSP ||
        opCode == LL_CTRL_TERMINATE_IND ||
        opCode == LL_CTRL_REJECT_IND)
        return TRUE;
    else
        return FALSE;
    
}

/*******************************************************************************
 * @fn          llProcessRxData0
 *
 * @brief       This routine is called by the RF_NormalIsr when an Rx packet
 *              has been received. It can also be called by the Master or
 *              Slave post-processing software.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      TRUE: Rx packet processed.
 *              FALSE: Unable to process packet.
 */
uint8 llProcessRxData0( void )
{
    llConnState_t *connPtr;
    uint8_t  i;
  	
    // get current connection
    connPtr = &conn_param[0];        

    if(getRxBufferSize(connPtr) > 0)
    {
        uint8    pktLen;
        uint8    pktHdr;
        uint8    *pBuf;
        uint8    *hciRxBuffer = NULL;       // for data packet
        
        // process the data packet
        i = get_rx_read_ptr (connPtr);
        pBuf     = connPtr->ll_buf.rx_conn_desc[i]->data;
        
        // read length first; placed this way by RF hardware for DMA optimization
        pktLen = (connPtr->ll_buf.rx_conn_desc[i]->header & 0xff00) >> 8;
        
        // read header
        pktHdr = connPtr->ll_buf.rx_conn_desc[i]->header & 0x3;
        
        // check if we have received a data packet during an encryption procedure
        // Note: This requirement is based on ESR05 V1.0, Erratum 3565.
        // Note: Technically, an empty packet will never be received since they
        //       are flushed.
        // Note: Also end if the packet type is invalid.
        if (LL_INVALID_LLID(pktHdr)  ||                     // bug fixed 2018-04-08     
           ((LL_DATA_PDU(pktHdr) && (pktLen != 0) && (connPtr->rxDataEnabled == FALSE))))// ||     // receive no-empty Data PDU in start enc procedure
        {
            //non-empty data packet or an invalid packet received ruing encryption
            // procedure, so terminate immediately per
            llConnTerminate( connPtr, LL_MIC_FAILURE_TERM );
            
            connPtr->pmCounter.ll_recv_invalid_pkt_cnt++;
            
            return( FALSE );
        }

        // adjust pointer to buffer based on packet type
        // Note: If a control PDU, then the generic pPkt buffer will be used
        //       whether encryption is enabled or not. For a data PDU, the generic
        //       pPkt buffer will only be used if encryption is enabled.
        if ( LL_DATA_PDU(pktHdr) )
        {        		
            connPtr->pmCounter.ll_to_hci_pkt_cnt++;
            // check if we have a data packet and the receive flow control is enabled
            // Note: This is in support of Controller to Host flow control.
            // yes, so request a Rx buffer from the HCI, sans the MIC bytes if enc is enabled
        		
            hciRxBuffer = LL_RX_bm_alloc( pktLen - ((connPtr->encEnabled)? 4 : 0) );
        
            // check that we have a buffer
            if ( hciRxBuffer == NULL )
            {
                connPtr->pmCounter.ll_hci_buffer_alloc_err_cnt ++;            // A1 ROM metal change add
                // abort processing the Rx PDU, not update Rx FIFO read ptr in this case. It will be processed next connection event
                return( FALSE );
            }
        }

        // check if we have to decrypt the PDU
        if ( connPtr->encEnabled )
        {
            // subtract the MIC size from the packet length, LL_ENC_Decrypt will add back
            pktLen -= LL_ENC_MIC_LEN;
			
            // decrypt PDU with authentication check
            if ( LL_ENC_Decrypt( connPtr,
                               pktHdr,
                               pktLen,
                               pBuf ) == FALSE )
            {
                // free the packet buffer
                osal_bm_free( hciRxBuffer );     

                // authentication failed; terminate immediately
                llConnTerminate( connPtr, LL_MIC_FAILURE_TERM );
                
                return( FALSE );
            }	
        }

        // check if this was a data PDU
        if ( LL_DATA_PDU(pktHdr) )
        {
            connPtr->pmCounter.ll_recv_data_pkt_cnt ++;
            
            // restore decrypted PDU to data buffer	    		
            osal_memcpy(hciRxBuffer, pBuf, pktLen );
	    		
            // call the HCI to process the received data 	
            LL_RxDataCompleteCback( (uint16)conn_param[0].connId,
                                   hciRxBuffer,  //pBuf,  
                                   pktLen,
                                   pktHdr,
                                   connPtr->lastRssi);	
	    		
            connPtr->ll_buf.rx_conn_desc[i]->valid = 0;
            connPtr->ll_buf.rx_conn_desc[i]->header = 0;

            update_rx_read_ptr(connPtr);        
        }
        else // a control PDU
        {
            connPtr->pmCounter.ll_recv_ctrl_pkt_cnt++;
            
            update_rx_read_ptr(connPtr);   //Attention:  correct bug 2017-04-20.   in llProcessSlaveControlPacket, the link may be terminated and reset so this staement should not be put after terminate
	    	
            {   // bug fixed 2018-04-08 , receive no expected Ctrl PDU in start enc procedure
                if (LL_CTRL_PDU(pktHdr) && (!llCtrlAllowInEncProc(*pBuf)) && (connPtr->rxDataEnabled == FALSE)
                    && !(connPtr->encInfo.encRestart))        // bug fixed 2018-4-20, consider pause enc case
                {
                    llConnTerminate( connPtr, LL_MIC_FAILURE_TERM );
            
                    connPtr->pmCounter.ll_recv_invalid_pkt_cnt++;
            
                    return( FALSE );
                }
                llProcessSlaveControlPacket( connPtr, pBuf );
                connPtr->ll_buf.rx_conn_desc[i]->valid = 0;
                connPtr->ll_buf.rx_conn_desc[i]->header = 0;         			
            }
        } // if data or control packet
    }

    return( TRUE );
}
		
/*******************************************************************************
 * @fn          llEnqueueCtrlPkt
 *
 * @brief       This function is used to add a control packet for subsequent
 *              processing. How the control packet is handled depends on
 *              whether it is part of a control procedure, and whether there's
 *              a control packet timeout associated with it.
 *
 *              Note: Duplicate entries are filtered.
 *
 *              Note: Can be called via the HCI interface or from the LL.
 *
 * input parameters
 *
 * @param       connPtr  - Pointer to the current connection.
 * @param       ctrlType - Control packet type.
 *
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llEnqueueCtrlPkt( llConnState_t *connPtr,
                       uint8         ctrlType )
{
  uint8         i;

  _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

  // filter duplicates
  for (i=0; i<connPtr->ctrlPktInfo.ctrlPktCount; i++)
  {
    if ( connPtr->ctrlPktInfo.ctrlPkts[ i ] == ctrlType )
    {
      // already present, so we're done here

      HAL_EXIT_CRITICAL_SECTION();

      return;
    }
  }

  // add to queue
  connPtr->ctrlPktInfo.ctrlPkts[ connPtr->ctrlPktInfo.ctrlPktCount ] = ctrlType;

  // bump the count
  connPtr->ctrlPktInfo.ctrlPktCount++;

  HAL_EXIT_CRITICAL_SECTION();

  return;
}

/*******************************************************************************
 * @fn          llDequeueCtrlPkt
 *
 * @brief       This function is used to remove a control packet that has
 *              completed processing. The control packet at the head of the
 *              queue is "removed" by moving remaining packets up.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llDequeueCtrlPkt( llConnState_t *connPtr )
{
  uint8         i;

if (connPtr->ctrlPktInfo.ctrlPktCount == 0)        // add by HZF
	return;

  _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

  // decrement the number of control packets left
  connPtr->ctrlPktInfo.ctrlPktCount--;

  // shift remaining packets, if any, up one spot
  // ALT: COULD UNROLL LOOP TO OPTIMIZE.
  for (i=0; i<connPtr->ctrlPktInfo.ctrlPktCount; i++)
  {
    connPtr->ctrlPktInfo.ctrlPkts[i] = connPtr->ctrlPktInfo.ctrlPkts[i+1];
  }

  // stuff an undefined packet at the end of the queue
  connPtr->ctrlPktInfo.ctrlPkts[LL_MAX_NUM_CTRL_PROC_PKTS-1] = LL_CTRL_UNDEFINED_PKT;

  // set the control processing to inactive for next packet, if there is one
  connPtr->ctrlPktInfo.ctrlPktActive = FALSE;

  HAL_EXIT_CRITICAL_SECTION();

  return;
}

/*******************************************************************************
 * @fn          llReplaceCtrlPkt
 *
 * @brief       This function is used to replace the current control packet at
 *              the head of the queue with the next control packet for the same
 *              control procedure. This is used to ensure that a new control
 *              procedure enqueued doesn't interleave with a control procedure
 *              that is already in progress.
 *
 * input parameters
 *
 * @param       connPtr  - Pointer to the current connection.
 * @param       ctrlType - Control packet type.
 *
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llReplaceCtrlPkt( llConnState_t *connPtr,
                       uint8          ctrlType )
{

  _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

  // replace control packet at head of queue
  connPtr->ctrlPktInfo.ctrlPkts[0] = ctrlType;

  // set the control processing to inactive
  connPtr->ctrlPktInfo.ctrlPktActive = FALSE;

  HAL_EXIT_CRITICAL_SECTION();

  return;
}


/*******************************************************************************
 * @fn          llConvertCtrlProcTimeoutToEvent
 *
 * @brief       This function is used to convert the control procedure timeout,
 *              which is a constant 40s, into an expiration event count. This
 *              routine is called when a new connection is formed, or when the
 *              connection update parameters have been changed.
 *
 *              Note: It is assumed here that connInterval has already been
 *                    converted into units of 625us.
 *
 *              Note: Based on Core V4.0, the termination control procedure
 *                    timeout is based on the Connection Supervision Timeout
 *                    value.
 *
 *              Side Effects:
 *              ctrlPktInfo.ctrlTimeoutVal - Updated with number of events to
 *                                           expiration.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llConvertCtrlProcTimeoutToEvent( llConnState_t *connPtr )
{
  // find the number of connection intervals in the control procedure timeout
  // Note: 1600 coarse ticks per second
  connPtr->ctrlPktInfo.ctrlTimeoutVal = (uint16)LL_MAX_CTRL_PROC_TIMEOUT /
                                                (connPtr->curParam.connInterval);

  // take the ceiling
  if ( (uint16)LL_MAX_CTRL_PROC_TIMEOUT % (connPtr->curParam.connInterval) )   // 40 s
  {
    // there's overage, so bump the count
    connPtr->ctrlPktInfo.ctrlTimeoutVal++;
  }

  return;
}


// TODO: to decide whether change llTimeCompare to ll24BitTimeCompare or not
#define LL_MAX_24BIT_TIME_IN_625US    0xC800  // 32s in 625us ticks (LSTO limit)
/*******************************************************************************
 * @fn          ll24BitTimeCompare
 *
 * @brief       This function determines if the first time parameter is greater
 *              than the second time parameter, taking timer counter wrap into
 *              account. If so, TRUE is returned. Both values are 24-bit times.
 *
 * input parameters
 *
 * @param       time1 - First 24-bit time.
 * @param       time2 - Second 24-bit time.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      TRUE:  When first parameter is greater than second.
 *              FALSE: When first parmaeter is less than or equal to
 *                     the second.
 */
uint8 ll24BitTimeCompare( uint32 time1,
                                    uint32 time2 )
{
  time1 &= 0x00FFFFFF;
  time2 &= 0x00FFFFFF;

  if ( time1 > time2 )
  {
    // check if time1 is greater than time2 due to wrap; that is, time2 is
    // actually greater than time1
    return ( (uint8)((time1-time2) <= LL_MAX_24BIT_TIME_IN_625US) );
  }
  else // time2 >= time1
  {
    // check if time2 is greater than time1 due to wrap; that is, time1 is
    // actually greater than time2
    return( (uint8)((time2-time1) > LL_MAX_24BIT_TIME_IN_625US) );
  }
}

/*******************************************************************************
 * @fn          llEventDelta
 *
 * @brief       This function returns the difference between two 16 bit event
 *              counters. It is used to find the number of events between some
 *              future event (event A) and the current event (event B).
 *
 * input parameters
 *
 * @param       eventA - The first event count.
 * @param       eventB - The second event count.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      The absolute number of events: | eventA - eventB |
 */
uint16 llEventDelta( uint16 eventA,
                               uint16 eventB )
{
  // first check if we don't need to worry about wrapping
  if ( eventA >= eventB )
  {
    // we don't, so take a straight difference
    return( eventA - eventB );
  }
  else // we have to handle wrap
  {
    // so combine the two ranges
    return( (uint16)((0x10000 - eventB) + eventA) );
  }
}

/*******************************************************************************
 * @fn          llEventInRange
 *
 * @brief       This function determines if a connection update event count is
 *              between the current event count (exclusive) and the next event
 *              count (inclusive), given 16 bit event counters are used. It is
 *              used when the next active event count is beyond the current
 *              event count plus one (possible due to slave latency) but an
 *              update event is pending before that and must be handled.
 *
 * input parameters
 *
 * @param       curEvent    - The current event that just completed.
 * @param       nextEvent   - The next active event based on slave latency.
 * @param       updateEvent - The event when an update needs to take place.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Indicates whether is between current event and next event:
 *              TRUE:  current event < update event <= next event
 *              FALSE: update event > next event
 */
uint8 llEventInRange( uint16 curEvent,
                                uint16 nextEvent,
                                uint16 updateEvent )
{
  // first check if we don't need to worry about wrapping
  if ( nextEvent > curEvent )
  {
    // we don't, so check if the instant is between curEvent and nextEvent
    // Note: Need to check that the instant is greater than the curEvent in
    //       case the instant has wrapped!
    return( (uint8) ((updateEvent <= nextEvent) && (updateEvent > curEvent)) );
  }
  else // we have to handle wrap
  {
    // so compare either the upper range exclusive or the lower range inclusive
    return( (uint8)((updateEvent > curEvent) || (updateEvent <= nextEvent)) );
  }
}

/*******************************************************************************
 * @fn          llConvertLstoToEvent
 *
 * @brief       This function is used to convert the LSTO, which is given
 *              in integer multiples of 10ms from 100ms to 32s, into an
 *              expiration event count.
 *
 *              Note: It is assumed here that connInterval and connTimeout have
 *                    already been converted into units of 625us.
 *
 *              Side Effects:
 *              expirationValue - The connection expiration value is updated.
 *
 * input parameters
 *
 * @param       connPtr    - Pointer to the current connection.
 * @param       connParams - Pointer to the connection parameters.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void llConvertLstoToEvent( llConnState_t *connPtr,
                           connParam_t   *connParams )
{
  // find the number of connection intervals in the LSTO
  connPtr->expirationValue = connParams->connTimeout / connParams->connInterval;

  // take the ceiling
  if ( connParams->connTimeout % connParams->connInterval )
  {
    // there's overage, so bump the count
    connPtr->expirationValue++;
  }

  return;
}
/*******************************************************************************
 * @fn          llSetupDataLenghtReq
 *
 * @brief       This function is used to setup the data length request.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupDataLenghtReq( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;
     	
    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
    {		
        pktLen = LL_LENGTH_REQ_PAYLOAD_LEN;
    
        // write control type as payload
        *pBuf++ = LL_LENGTH_REQ;

        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxRxOctets) , 2 );
        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxRxTime)   , 2 );
        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxTxOctets) , 2 );
        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxTxTime)   , 2 );
      
        // encrypt TX packet in place in the TX FIFO
        if ( connPtr->encEnabled )
        { 
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                          LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                          pktLen,
                          connPtr->ctrlData.data );
    
            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }
    
    
        connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
    
        return( TRUE );
    }

    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupDataLenghtRsp
 *
 * @brief        This function is used to setup the data length response.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupDataLenghtRsp( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;
     	
    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
    {		
        pktLen = LL_LENGTH_RSP_PAYLOAD_LEN;
    
        // write control type as payload
        *pBuf++ = LL_LENGTH_RSP;

        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxRxOctets) , 2 );
        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxRxTime)   , 2 );
        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxTxOctets) , 2 );
        pBuf = llMemCopyDst( pBuf, (uint8 *)& (g_llPduLen.suggested.MaxTxTime)   , 2 );
      
        // encrypt TX packet in place in the TX FIFO
        if ( connPtr->encEnabled )
        { 
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                          LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                          pktLen,
                          connPtr->ctrlData.data );
    
            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }
    
    
        connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
    
        return( TRUE );
    }

    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupPhyReq
 *
 * @brief       This function is used to setup the phy update request.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupPhyReq( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;
     	
    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
    {		
        pktLen = LL_PHY_REQ_PAYLOAD_LEN;
    
        // write control type as payload
        *pBuf++ = LL_PHY_REQ;

        *pBuf++ = connPtr->llPhyModeCtrl.req.txPhy;
        *pBuf++ = connPtr->llPhyModeCtrl.req.rxPhy;;

      
        // encrypt TX packet in place in the TX FIFO
        if ( connPtr->encEnabled )
        { 
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                          LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                          pktLen,
                          connPtr->ctrlData.data );
    
            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }
    
    
        connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
    
        return( TRUE );
    }

    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupPhyReq
 *
 * @brief       This function is used to setup the phy update response.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupPhyRsp( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;
     	
    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
    {		
        pktLen = LL_PHY_RSP_PAYLOAD_LEN;
    
        // write control type as payload
        *pBuf++ = LL_PHY_RSP;

        *pBuf++ = connPtr->llPhyModeCtrl.def.txPhy;
        *pBuf++ = connPtr->llPhyModeCtrl.def.rxPhy;

      
        // encrypt TX packet in place in the TX FIFO
        if ( connPtr->encEnabled )
        { 
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                          LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                          pktLen,
                          connPtr->ctrlData.data );
    
            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }
    
    
        connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
    
        return( TRUE );
    }

    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupPhyUpdateInd
 *
 * @brief       This function is used to setup the phy update indicate.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
//uint8 llSetupPhyUpdateInd( llConnState_t *connPtr )
//{
//    uint8 pktLen;
//    uint8 *pBuf = connPtr->ctrlData.data;
//
//    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
//    {		
//        pktLen = LL_PHY_UPDATE_IND_PAYLOAD_LEN;
//    
//        // write control type as payload
//        *pBuf++ = LL_PHY_UPDATE_IND;
//
//        *pBuf++ = connPtr->phyUpdateInfo.m2sPhy;
//        *pBuf++ = connPtr->phyUpdateInfo.s2mPhy;
//        
//        
//        // we are writing to the FIFO, so convert relative instant number to an
//        // absolute event number
//        connPtr->phyModeUpdateEvent+= connPtr->currentEvent;
//        pBuf = llMemCopyDst( pBuf, (uint8 *)& (connPtr->phyModeUpdateEvent) , 2 );
//
//        // encrypt TX packet in place in the TX FIFO
//        if ( connPtr->encEnabled )
//        { 
//            // encrypt PDU with authentication check
//            LL_ENC_Encrypt( connPtr,
//                          LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
//                          pktLen,
//                          connPtr->ctrlData.data );
//    
//            // increase length by size of MIC
//            pktLen += LL_ENC_MIC_LEN;
//        }
//    
//    
//        connPtr->ctrlDataIsPending  = 1;
//        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
//    
//        return( TRUE );
//    }
//
//    return( FALSE );
//}

/*******************************************************************************
 * @fn          llSetupUpdateParamReq
 *
 * @brief       This function is used to setup the update parameter request.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
//uint8 llSetupUpdateParamReq( llConnState_t *connPtr )
//{
//    uint8 pktLen;
//    uint8 *pBuf = connPtr->ctrlData.data;
//     	
//    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
//    {		
//        pktLen = LL_CONN_UPDATE_REQ_PAYLOAD_LEN;
//    
//        // write control type as payload
//        *pBuf++ = LL_CTRL_CONNECTION_UPDATE_REQ;
//    
//        // write the window size
//        *pBuf++ = connPtr->paramUpdate.winSize >> 1;
//    
//        // write the window offset
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->paramUpdate.winOffset, 2 );
//    
//        // write the connection interval
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->paramUpdate.connInterval, 2 );
//    
//        // write the slave latency
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->paramUpdate.slaveLatency, 2 );
//    
//        // write the connection timeout
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->paramUpdate.connTimeout, 2 );
//    
//        // we are writing to the FIFO, so convert relative instant number to an
//        // absolute event number
//        connPtr->paramUpdateEvent += connPtr->currentEvent;
//    
//        // write the update event count
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->paramUpdateEvent, 2 );
//    
//        // encrypt TX packet in place in the TX FIFO
//        if ( connPtr->encEnabled )
//        { 
//            // encrypt PDU with authentication check
//            LL_ENC_Encrypt( connPtr,
//                          LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
//                          pktLen,
//                          connPtr->ctrlData.data );
//    
//            // increase length by size of MIC
//            pktLen += LL_ENC_MIC_LEN;
//        }
//    
//    
//        connPtr->ctrlDataIsPending  = 1;
//        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
//    
//        return( TRUE );
//    }
//
//    return( FALSE );
//}

/*******************************************************************************
 * @fn          llSetupUpdateChanReq
 *
 * @brief       This function is used to setup the update channel request.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
//uint8 llSetupUpdateChanReq( llConnState_t *connPtr )
//{
//    uint8 pktLen;
//    uint8 *pBuf = connPtr->ctrlData.data;
//     	
//    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
//    {		
//        pktLen = LL_CHAN_MAP_REQ_PAYLOAD_LEN;
//    
//        // write control type as payload
//        *pBuf++ = LL_CTRL_CHANNEL_MAP_REQ;
//    
//        // write the new channel map
//        pBuf = llMemCopyDst( pBuf, chanMapUpdate.chanMap, LL_NUM_BYTES_FOR_CHAN_MAP );
//    
//        // we are writing to the FIFO, so convert relative instant number to an
//        // absolute event number
//        connPtr->chanMapUpdateEvent += connPtr->currentEvent;
//    
//        // write the update event count
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->chanMapUpdateEvent, 2 );
//    
//        // encrypt TX packet in place in the TX FIFO
//        if ( connPtr->encEnabled )
//        {
//            // encrypt PDU with authentication check
//            LL_ENC_Encrypt( connPtr,
//                            LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
//                            pktLen,
//                            connPtr->ctrlData.data );
//    
//            // increase length by size of MIC
//            pktLen += LL_ENC_MIC_LEN;
//        }
//
//        connPtr->ctrlDataIsPending  = 1;
//        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;		
//    
//        return( TRUE );
//    }
//
//    return( FALSE );
//}

/*******************************************************************************
 * @fn          llSetupEncReq
 *
 * @brief       This function is used to setup the start encryption request.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
//uint8 llSetupEncReq( llConnState_t *connPtr )
//{
//    uint8 pktLen;
//    uint8 *pBuf = connPtr->ctrlData.data;
//    
//    // first we need to verify that all pending transmissions have finished and
//    if (getTxBufferSize(connPtr) == 0 
//      && !connPtr->ctrlDataIsPending      // TX FIFO empty, of course, no ctrl data is pending
//      && !connPtr->ctrlDataIsProcess)   // TODO: change to check physical Tx FIFO done, then could setup enc
//    {  
//        pktLen = LL_ENC_REQ_PAYLOAD_LEN;
//    
//        // write control type as payload
//        *pBuf++ = LL_CTRL_ENC_REQ;	
//    
//        // rand
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->encInfo.RAND, LL_ENC_RAND_LEN );
//    	
//        // EDIV
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->encInfo.EDIV, LL_ENC_EDIV_LEN );
//    	
//        // SKDm
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->encInfo.SKD[LL_ENC_SKD_M_OFFSET], LL_ENC_SKD_M_LEN );
//    	
//        // IVm
//        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->encInfo.IV[LL_ENC_IV_M_OFFSET], LL_ENC_IV_M_LEN );
//    
//        // bug fixed 2018-8-6, need reverse the SKDm & IVm byte order
//        // bytes are generated LSO..MSO, but need to be maintained as
//        // MSO..LSO, per FIPS 197 (AES), so reverse the bytes
//        LL_ENC_ReverseBytes( (uint8 *)&connPtr->encInfo.SKD[LL_ENC_SKD_M_OFFSET],
//                          LL_ENC_SKD_M_LEN );
//
//        // bytes are generated LSO..MSO, but need to be maintained as
//        // MSO..LSO, per FIPS 197 (AES), so reverse the bytes
//        // ALT: POSSIBLE TO MAINTAIN THE IV IN LSO..MSO ORDER SINCE THE NONCE IS
//        //      FORMED THAT WAY.
//        LL_ENC_ReverseBytes( (uint8 *)&connPtr->encInfo.IV[LL_ENC_IV_M_OFFSET],
//                          LL_ENC_IV_M_LEN );
//
//        connPtr->ctrlDataIsPending  = 1;
//        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
//    
//        // set the control packet timeout for 40s relative to our present time
//        // Note: This is done in terms of connection events.
//        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
//    
//        return( TRUE );
//    }
//
//    return( FALSE );
//}

/*******************************************************************************
 * @fn          llSetupEncRsp
 *
 * @brief       This function is used to setup the encryption response. This
 *              can only be done when all pending transmissions have first been
 *              completed (i.e. the TX FIFO is empty).
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupEncRsp( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;
    // first we need to verify that all pending transmissions have finished and
    if (getTxBufferSize(connPtr) == 0 
      && !connPtr->ctrlDataIsPending      // TX FIFO empty, of course, no ctrl data is pending
      && !connPtr->ctrlDataIsProcess)   // TODO: change to check physical Tx FIFO done, then could setup enc
    {
        uint8 i;
    		
        // ============== update 2018-1-24	
        // write control type as payload
        *pBuf++ = LL_CTRL_ENC_RSP;	
    
        // SKDs
        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->encInfo.SKD[LL_ENC_SKD_S_OFFSET], LL_ENC_SKD_S_LEN );
    
        // IVs
        pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->encInfo.IV[LL_ENC_IV_S_OFFSET], LL_ENC_IV_S_LEN);		
        // ========= update 2018-1-24 end		
    
        // bytes are generated LSO..MSO, but need to be maintained as
        // MSO..LSO, per FIPS 197 (AES), so reverse the bytes
        LL_ENC_ReverseBytes( (uint8 *)&connPtr->encInfo.SKD[LL_ENC_SKD_S_OFFSET],
                            LL_ENC_SKD_S_LEN );
    
        // bytes are generated LSO..MSO, but need to be maintained as
        // MSO..LSO, per FIPS 197 (AES), so reverse the bytes
        // ALT: POSSIBLE TO MAINTAIN THE IV IN LSO..MSO ORDER SINCE THE NONCE IS
        //      FORMED THAT WAY.
        LL_ENC_ReverseBytes( (uint8 *)&connPtr->encInfo.IV[LL_ENC_IV_S_OFFSET],
                            LL_ENC_IV_S_LEN );
    
        // place the IV into the Nonce to be used for this connection
        // Note: If a Pause Encryption control procedure is started, the
        //       old Nonce value will be used until encryption is disabled.
        // Note: The IV is sequenced LSO..MSO within the Nonce.
        // ALT: POSSIBLE TO MAINTAIN THE IV IN LSO..MSO ORDER SINCE THE NONCE IS
        //      FORMED THAT WAY.
        for (i=0; i<LL_ENC_IV_LEN; i++)
        {
          connPtr->encInfo.nonce[ LL_END_NONCE_IV_OFFSET+i ] =
            connPtr->encInfo.IV[ (LL_ENC_IV_LEN-i)-1 ];
        }
    
        pktLen = LL_ENC_RSP_PAYLOAD_LEN;
    
        connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
        // set the control packet timeout for 40s relative to our present time
        // Note: This is done in terms of connection events.
        // Note: For a Encryption Setup procedure that follows an Encryption Pause,
        //       this is a Restart Timer operation as the timer was already started
        //       when LL_PAUSE_ENC_RSP was enqueued for transmission. For a normal
        //       Encryption Setup procedure, this will have no effect as the timer
        //       was never started before.
        // Note: Upon re-examination of the previous "Note", and given the most
        //       recent changes to the Controller spec (D09R31), it isn't clear
        //       why the timer should be started or re-started when a
        //       LL_PAUSE_ENC_RSP or LL_ENC_RSP packet is sent for a re-start enc.
        //       Or given
        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
    
        return( TRUE );
    }
    
    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupStartEncReq
 *
 * @brief       This function is used to handle placing the Start Encryption
 *              Request into the TX FIFO.
 *
 *              Note: The TX FIFO should already be empty.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupStartEncReq( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;
    // Note: No need to check if there's enough room in the TX FIFO since it was
    //       forced to empty prior to beginning encryption control procedure.

    pktLen = LL_START_ENC_REQ_PAYLOAD_LEN;

    // write control type as payload
    *pBuf++ = LL_CTRL_START_ENC_REQ;	

    connPtr->ctrlDataIsPending  = 1;
    connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
    // set the control packet timeout for 40s relative to our present time
    // Note: This is done in terms of connection events.
    // Note: For a Encryption Setup procedure that follows an Encryption Pause,
    //       this is a Restart Timer operation. For a normal Encryption Setup
    //       procedure, this is a Start Timer operation. Effectively, there is
    //       no difference between the two.
    connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;

    return( TRUE );
}


/*******************************************************************************
 * @fn          llSetupStartEncRsp
 *
 * @brief       This function is used to handle the placement of the Encryption
 *              Response packet in the TX FIFO.
 *
 *              Note: The TX FIFO should already be empty.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupStartEncRsp( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;
 
    // Note: No need to check if there's enough room in the TX FIFO since it was
    //       forced to empty prior to beginning encryption control procedure.

    // write control type as payload
    *pBuf = LL_CTRL_START_ENC_RSP;		
    // encrypt PDU with authentication check
    LL_ENC_Encrypt( connPtr,
                  LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                  LL_START_ENC_RSP_PAYLOAD_LEN,
                  pBuf );	// input no-encrypt data pBuf, output in the same buffer

    pktLen = LL_START_ENC_RSP_PAYLOAD_LEN + LL_ENC_MIC_LEN;

    connPtr->ctrlDataIsPending  = 1;
    connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;	
    // control procedure timeout value only needed for Master after Start Enc Response
    if ( llState == LL_STATE_CONN_MASTER )
    {
        // set the control packet timeout for 40s relative to our present time
        // Note: This is done in terms of connection events.
        // Note: Core Spec V4.0 now indicates that each LL control PDU that is queued
        //       for transmission resets the procedure response timeout timer.
        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
    }


    return( TRUE );
}


/*******************************************************************************
 * @fn          llSetupPauseEncReq
 *
 * @brief       This function is used to handle the placement of the Encryption
 *              Pause Request packet in the TX FIFO.
 *
 *              Note: The TX FIFO should already be empty.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
//uint8 llSetupPauseEncReq( llConnState_t *connPtr )
//{
//    uint8 pktLen;
//    uint8 *pBuf = connPtr->ctrlData.data;
//    // first we need to verify that all pending transmissions have finished and
//    //if ( PHY_TX_FIFO_LEN() == 0 )
//    if (getTxBufferSize(connPtr) == 0 
//      && !connPtr->ctrlDataIsPending      // TX FIFO empty, of course, no ctrl data is pending
//      && !connPtr->ctrlDataIsProcess)   // TODO: change to check physical Tx FIFO done, then could setup enc
//    {
//        // Note: No need to check if there's enough room in the TX FIFO since it was
//        //       forced to empty prior to beginning encryption control procedure.
//
//        // write control type as payload
//        *pBuf = LL_CTRL_PAUSE_ENC_REQ;	
//
//        // encrypt PDU with authentication check
//        LL_ENC_Encrypt( connPtr,
//                    LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
//                    LL_PAUSE_ENC_REQ_PAYLOAD_LEN,
//                    pBuf );
//	
//        pktLen = LL_PAUSE_ENC_REQ_PAYLOAD_LEN + LL_ENC_MIC_LEN;
//	
//        connPtr->ctrlDataIsPending  = 1;
//        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
//
//
//        // set the control packet timeout for 40s relative to our present time
//        // Note: This is done in terms of connection events.
//        // Note: Core Spec V4.0 now indicates that each LL control PDU that is queued
//        //       for transmission resets the procedure response timeout timer.
//        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
//
//        return( TRUE );
//    }
//
//  return( FALSE );
//}


/*******************************************************************************
 * @fn          llSetupPauseEncRsp
 *
 * @brief       This function is used to handle the placement of the Pause
 *              Encryption packet in the TX FIFO. This can only be done when
 *              all pending transmissions have first been completed (i.e.
 *              the TX FIFO is empty).
 *
 *              Note: The TX FIFO should already be empty.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully done:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupPauseEncRsp( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;

    // first we need to verify that all pending transmissions have finished
    // Note: On the Master, the FIFO should already be empty.
    //if ( PHY_TX_FIFO_LEN() == 0 )
    if (getTxBufferSize(connPtr) == 0 
      && !connPtr->ctrlDataIsPending      // TX FIFO empty, of course, no ctrl data is pending
      && !connPtr->ctrlDataIsProcess)   // TODO: change to check physical Tx FIFO done, then could setup enc	
    {
  
        pktLen = LL_PAUSE_ENC_RSP_PAYLOAD_LEN;
    
        // write control type as payload
        *pBuf = LL_CTRL_PAUSE_ENC_RSP;
    
        // only the Slave encrypts the Pause Encryption Response
        if ( llState == LL_STATE_CONN_SLAVE )
        {
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                            LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                            pktLen,
                            pBuf );
      
            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }
    
        connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
        // set the control packet timeout for 40s relative to our present time
        // Note: This is done in terms of connection events.
        // Note: Core Spec V4.0 now indicates that each LL control PDU that is
        //       queued for transmission resets the procedure response timeout
        //       timer.
        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
    
        return( TRUE );
    }
    
    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupRejectInd
 *
 * @brief       This function is used to setup the Reject Indications procedure
 *              which results when encryption has been started, but the LTK
 *              request was negative. Once the rejection indication is sent
 *              and acknowledged, the connection remains unencrypted and TX
 *              is re-enabled.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupRejectInd( llConnState_t *connPtr ,uint8 errCode)
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;

    pktLen = LL_REJECT_IND_PAYLOAD_LEN;

    // write control type as payload
    *pBuf = LL_CTRL_REJECT_IND;  
    //*(pBuf + 1) = connPtr->encInfo.encRejectErrCode;    // TP/SEC/SLA/BV-04-C, reject cause should be filled
	*(pBuf + 1) = errCode;    // TP/SEC/SLA/BV-04-C, reject cause should be filled
    connPtr->ctrlDataIsPending  = 1;
    
    // set the control packet timeout for 40s relative to our present time
    // Note: This is done in terms of connection events.    
    connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
    
    connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
		
    return( TRUE );
}

/*******************************************************************************
 * @fn          llSetupRejectExtInd
 *
 * @brief       This function is used to setup the Reject Indications procedure
 *              which results when encryption has been started, but the LTK
 *              request was negative. Once the rejection indication is sent
 *              and acknowledged, the connection remains unencrypted and TX
 *              is re-enabled.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupRejectExtInd( llConnState_t *connPtr ,uint8 errCode)
{
    uint8 pktLen;
    uint8 *pBuf = connPtr->ctrlData.data;

    pktLen = LL_REJECT_EXT_IND_PAYLOAD_LEN;

    // write control type as payload
    *pBuf = LL_CTRL_REJECT_EXT_IND;  
    *(pBuf + 1) = connPtr->rejectOpCode;    // TP/SEC/SLA/BV-04-C, reject cause should be filled
    *(pBuf + 1) = errCode;    // TP/SEC/SLA/BV-04-C, reject cause should be filled
	
    connPtr->ctrlDataIsPending  = 1;
    
    // set the control packet timeout for 40s relative to our present time
    // Note: This is done in terms of connection events.    
    connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
    
    connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
		
    return( TRUE );
}

/*******************************************************************************
 * @fn          llSetupVersionIndReq
 *
 * @brief       This function is used to setup the version information
 *              indication.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupVersionIndReq( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf;  
   	
    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
    {		
        pBuf= connPtr->ctrlData.data ;

        // write payload length first
        // Note: The Radio expects length to have been set one more than the
        //       actual payload size (to support DMA), which it will decrement by
        //       one before transmitting. Since DMA isn't being used here, the
        //       length is bumped by one to offset this decrement.
        pktLen = LL_VERSION_IND_PAYLOAD_LEN;

        // write control type as payload
        *pBuf++ = LL_CTRL_VERSION_IND;

        // write the version number
        *pBuf++ = verInfo.verNum;

        // write the company ID
        pBuf = llMemCopyDst( pBuf, (uint8 *)&verInfo.comId, 2 );

        // write the subversion number
        pBuf = llMemCopyDst( pBuf, (uint8 *)&verInfo.subverNum, 2 );

        // encrypt TX packet in place in the TX FIFO
        if ( connPtr->encEnabled )
        {
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                      LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                      pktLen,
                      connPtr->ctrlData.data );
            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }

        connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;		
        
        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
		
        return (TRUE);
    }
    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupFeatureSetReq
 *
 * @brief       This function is used to setup the feature set request.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
//uint8 llSetupFeatureSetReq( llConnState_t *connPtr )
//{
//    uint8 pktLen;
//    uint8 *pBuf;  
//   	
//    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )   // new by phyplus, change to ctrl data queue?
//    {		
//        pBuf= connPtr->ctrlData.data ;
//
//        pktLen = LL_FEATURE_REQ_PAYLOAD_LEN;
//
//        // write control type as payload
//        *pBuf++ = LL_CTRL_FEATURE_REQ;
//
//        // write the feature set
//        pBuf = llMemCopyDst( pBuf, connPtr->featureSetInfo.featureSet, LL_MAX_FEATURE_SET_SIZE );
//
//        // encrypt TX packet in place in the TX FIFO
//        if ( connPtr->encEnabled )
//        {
//            // encrypt PDU with authentication check
//            LL_ENC_Encrypt( connPtr,
//                      LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
//                      pktLen,
//                      connPtr->ctrlData.data );
//
//            // increase length by size of MIC
//            pktLen += LL_ENC_MIC_LEN;
//        }
//
//        connPtr->ctrlDataIsPending  =1;
//        connPtr->ctrlData .header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
//        // set the control packet timeout for 40s relative to our present time
//        // Note: This is done in terms of connection events.
//        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->ctrlPktInfo.ctrlTimeoutVal;
//
//        return( TRUE );
//    }
//
//    return( FALSE );
//}

/*******************************************************************************
 * @fn          llSetupFeatureSetRsp
 *
 * @brief       This function is used to setup the Feature Set Response packet.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupFeatureSetRsp( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf;
  
    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )
	{
        pBuf= connPtr->ctrlData.data;

        pktLen = LL_FEATURE_RSP_PAYLOAD_LEN;

        // write control type as payload
        *pBuf++ = LL_CTRL_FEATURE_RSP;

        // write the feature set response payload
        pBuf = llMemCopyDst( pBuf, connPtr->featureSetInfo.featureSet, LL_MAX_FEATURE_SET_SIZE );

        // encrypt TX packet
        if ( connPtr->encEnabled )
        {
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                      LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                      pktLen,
                      connPtr->ctrlData .data );

            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }

        // deactivate slave latency, if it was enabled.   TI setting, it seems useless?
        // Note: Not used by Master.
        connPtr->slaveLatency = 0;
		
		connPtr->ctrlDataIsPending  = 1;
		connPtr->ctrlData .header = pktLen <<8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
        
        return( TRUE );
    }

    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupUnknownRsp
 *
 * @brief       This function is used to setup the Unknown Response packet.
 *
 *              Note: There is no control procedure timeout associated with
 *                    the Unknown Response control packet.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupUnknownRsp( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf;
  
    if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )
	{
        pBuf= connPtr->ctrlData .data;
	
        pktLen = LL_UNKNOWN_RSP_PAYLOAD_LEN;

        // write control type as payload
        *pBuf++ = LL_CTRL_UNKNOWN_RSP;

        // write unknown control type as payload
        *pBuf = connPtr->unknownCtrlType;

        // encrypt TX packet in place in the TX FIFO
        if ( connPtr->encEnabled )
        {
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                      LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                      pktLen,
                      connPtr->ctrlData .data );

            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }

	    connPtr->ctrlDataIsPending  = 1;
        connPtr->ctrlData .header = pktLen <<8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
		
        return( TRUE );
    }

    return( FALSE );
}

/*******************************************************************************
 * @fn          llSetupTermInd
 *
 * @brief       This function is used to setup the Connection Termination
 *              procedure that has been requested by the Host. This function
 *              is called before the start of the next Slave task. The
 *              Connection Termination procedure requires that the Slave send
 *              a TERMINATE_IND packet and receive an ACK from the Master. In
 *              addition, a control packet timeout must be started.
 *
 *              To simplify this procedure, all packets will be removed from
 *              the TX FIFO before writing the TERMINATE_IND packet there. This
 *              simplifies how can determine the terminate packet has been sent
 *              by making the number of expected ACKs deterministic. However,
 *              since it is possible that the NR  may be retransmitting a prior
 *              packet (which we can not remove), we have to first determine
 *              when the terminate packet is sent before waiting for its ACK.
 *              This is the same as waiting for either one or two ACKs,
 *              depending on whether there is a retransmit packet or not.
 *              This can be determined by first checking if the TX FIFO is empty
 *              after removing all pending packets (the retransmit packet, if
 *              present, is unaffected), and setting the number of expected
 *              packets accordingly. When the task ends, the number of ACKS
 *              received can be checked.
 *
 *              Please note that the NR might also be retransmitting a prior
 *              auto-empty packet (please see section 6.5.1 of the NR spec
 *              for more detail), however in this case, the number of expected
 *              ACKs would still be one since the BLE_L_NTXDONE counter is only
 *              incremented when an ACK is received for a packet that is in the
 *              TX FIFO.
 *
 * input parameters
 *
 * @param       connPtr - Pointer to the current connection.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Boolean to indicate whether the setup was successfully
 *              completed:
 *              TRUE:  Success
 *              FALSE: Not Done
 */
uint8 llSetupTermInd( llConnState_t *connPtr )
{
    uint8 pktLen;
    uint8 *pBuf;

	if(!connPtr->ctrlDataIsPending && !connPtr->ctrlDataIsProcess )
	{
        pBuf= connPtr->ctrlData.data;

        pktLen = LL_TERM_IND_PAYLOAD_LEN;

        // write control type as payload
        *pBuf++ = LL_CTRL_TERMINATE_IND;

        // write the reason code
        *pBuf = connPtr->termInfo.reason;

        // encrypt TX packet in place in the TX FIFO
        if ( connPtr->encEnabled )
        {
            // encrypt PDU with authentication check
            LL_ENC_Encrypt( connPtr,
                      LL_DATA_PDU_HDR_LLID_CONTROL_PKT,
                      pktLen,
                      connPtr->ctrlData .data );

            // increase length by size of MIC
            pktLen += LL_ENC_MIC_LEN;
        }


        // disable the update parameter and update data channel pending flags
        // just in case they happen to be set
        // Note: Technically, these flags should not be set as only one active
        //       control procedure is permitted at a time.
        connPtr->pendingParamUpdate = FALSE;
        connPtr->pendingChanUpdate  = FALSE;
        
        connPtr->pendingPhyModeUpdate = FALSE;

        // deactivate slave latency, if it was enabled
        // Note: Not used by Master.
        connPtr->slaveLatency = 0;

        // Per Core V4.0 spec change, the termination timeout is the LSTO
        connPtr->ctrlPktInfo.ctrlTimeout = connPtr->expirationValue;

        connPtr->ctrlDataIsPending  =1;
        connPtr->ctrlData.header = pktLen <<8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;
		
        return( TRUE );
    }

    return( FALSE );
}

/*******************************************************************************
 * This function enqueues a TX data packet to the back of the data queue.
 *
 * Public function defined in hci_c_data.h.
 */
uint8 llEnqueueDataQ( llDataQ_t *pDataQ, txData_t *pTxData )
{
   

  _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

  // check if the data queue is not empty
  if ( pDataQ->head != NULL )
  {
    // at least one entry already on the data queue, so append packet
    pDataQ->tail->pNext = pTxData;
  }
  else // data queue is empty
  {
    // so add first packet
    pDataQ->head = pTxData;
  }

  // either way, the tail gets this packet, and its next pointer is NULL
  pDataQ->tail   = pTxData;
  pTxData->pNext = NULL;

  HAL_EXIT_CRITICAL_SECTION();

  return( LL_STATUS_SUCCESS );
}



/*******************************************************************************
 * @fn          llResetConnId
 *
 * @brief       reset the global conn_param[0] , refer to llAllocConnId
 *            
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      none
 */
void llResetConnId(  uint8 connId )
{
	int i;
	llConnState_t *connPtr;

	connPtr = &conn_param[connId];       

	//connPtr->allocConn                  = TRUE;
//	connPtr->connId                     = 0;
	connPtr->txDataEnabled              = TRUE;
	connPtr->rxDataEnabled              = TRUE;
	connPtr->currentEvent               = 0;
    connPtr->nextEvent                  = 0;
    connPtr->rx_timeout                 = 0;
    connPtr->firstPacket                = 1;
    connPtr->currentChan                = 0;
    connPtr->lastCurrentChan            = 0;
    connPtr->lastTimeToNextEvt          = 0;
    connPtr->slaveLatencyAllowed        = FALSE;
    connPtr->slaveLatency               = 0;
    connPtr->pendingChanUpdate          = FALSE;
    connPtr->pendingParamUpdate         = FALSE;
    connPtr->pendingPhyModeUpdate       = FALSE;
    connPtr->isCollision                = FALSE;
    
    //connPtr->lastRssi                   = LL_RF_RSSI_UNDEFINED;
    connPtr->ctrlPktInfo.ctrlPktActive  = FALSE;
    connPtr->ctrlPktInfo.ctrlPktCount   = 0;
    connPtr->verExchange.peerInfoValid  = FALSE;
    connPtr->verExchange.hostRequest    = FALSE;
    connPtr->verExchange.verInfoSent    = FALSE;
    connPtr->verInfo.verNum             = 0;
    connPtr->verInfo.comId              = 0;
    connPtr->verInfo.subverNum          = 0;
    connPtr->termInfo.termIndRcvd       = FALSE;
    connPtr->encEnabled                 = FALSE;
    connPtr->encInfo.SKValid            = FALSE;
    connPtr->encInfo.LTKValid           = FALSE;
    connPtr->encInfo.txPktCount         = 0;
    connPtr->encInfo.rxPktCount         = 0;
    connPtr->encInfo.startEncRspRcved   = FALSE;
    connPtr->encInfo.encReqRcved        = FALSE;
    connPtr->encInfo.encRestart         = FALSE;
    
    connPtr->encInfo.startEncReqRcved   = FALSE;
    connPtr->encInfo.rejectIndRcved     = FALSE;
    
    connPtr->perInfo.numPkts            = 0;
    connPtr->perInfo.numCrcErr          = 0;
    connPtr->perInfo.numEvents          = 0;
    connPtr->perInfo.numMissedEvts      = 0;

    // PHY mode ctrl
    connPtr->llPhyModeCtrl.local.txPhy=LE_1M_PHY;
    connPtr->llPhyModeCtrl.local.rxPhy=LE_1M_PHY;
		
    connPtr->llPhyModeCtrl.isProcessingReq = FALSE;
    connPtr->llPhyModeCtrl.isWatingRsp 	= FALSE;
    connPtr->llPhyModeCtrl.isChanged		= FALSE;		

    // PDU len
    connPtr->llPduLen.local.MaxRxOctets=LL_PDU_LENGTH_INITIAL_MAX_RX_OCTECTS;
    connPtr->llPduLen.local.MaxTxOctets=LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS;
    connPtr->llPduLen.local.MaxRxTime  =LL_PDU_LENGTH_INITIAL_MAX_RX_TIME;
    connPtr->llPduLen.local.MaxTxTime  =LL_PDU_LENGTH_INITIAL_MAX_TX_TIME;

    connPtr->llPduLen.remote.MaxRxOctets=LL_PDU_LENGTH_INITIAL_MAX_RX_OCTECTS;
    connPtr->llPduLen.remote.MaxTxOctets=LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS;
    connPtr->llPduLen.remote.MaxRxTime  =LL_PDU_LENGTH_INITIAL_MAX_RX_TIME;
    connPtr->llPduLen.remote.MaxTxTime  =LL_PDU_LENGTH_INITIAL_MAX_TX_TIME;    
    connPtr->llPduLen.isProcessingReq=FALSE;
    connPtr->llPduLen.isWatingRsp	  =FALSE;
    connPtr->llPduLen.isChanged	  =FALSE;

	// add 2020-03-17
	connPtr->llRfPhyPktFmt  = LE_1M_PHY;

    //add by ZQ for preChannMapUpdate
    connPtr->preChanMapUpdate.chanMapUpdated = FALSE;
	
    // set packet queue to undefined values
    // Note: This is used for debugging purposes.
    for (i = 0; i < LL_MAX_NUM_CTRL_PROC_PKTS; i++)
    {
        connPtr->ctrlPktInfo.ctrlPkts[i] = LL_CTRL_UNDEFINED_PKT;
    }

    // set the connection Feature Set based on this device's default
    for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
    {
        connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
    }
}

/*******************************************************************************
 * @fn          llPendingUpdateParam
 *
 * @brief       This function is used to check if any active connections have
 *              an Update Parameter control procedure pending.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None
 *
 * @return      TRUE:  There is an update control procedure for at least one
 *                     active connection.
 *              FALSE: There is no update control procedure on any active
 *                     connection.
 */
uint8 llPendingUpdateParam( void )
{
  uint8 i;

  // check if an update parameter control procedure is active on any connections
  for (i = 0; i < g_maxConnNum; i++)
  {
    llConnState_t *connPtr = &conn_param[i];        // only 1 connection now, HZF

    // check if this connection is active
    if ( connPtr->active )
    {
      // check if an Update Parameter control procedure is pending
      if ( (connPtr->ctrlPktInfo.ctrlPktCount > 0) &&
           (connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_CONNECTION_UPDATE_REQ) )
       {
         return( TRUE );
       }
    }
  }

  return( FALSE );
}

	
/*******************************************************************************
 * @fn          llInitFeatureSet
 *
 * @brief       This function initializes this device's Feature Set.
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
void llInitFeatureSet( void )
{
  uint8 i;

  // clear Feature Set data for this device
  for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
  {
    // set all feature set bits to "unused"
    deviceFeatureSet.featureSet[i] = LL_FEATURE_RFU;
  }

  // set those features supported by this controller
  // ALT: COULD DO THIS IN TERMS OF BYTE/BIT NUMBER.
  deviceFeatureSet.featureSet[0] = (uint8)(   LL_FEATURE_ENCRYPTION  );

  return;
}

void llInitFeatureSetDLE(uint8 enable)
{
    if(enable)
    {
        deviceFeatureSet.featureSet[0]|=(uint8)LL_FEATURE_DATA_LENGTH_EXTENSION;
    }
    else
    {
        deviceFeatureSet.featureSet[0]&=(uint8)(~LL_FEATURE_DATA_LENGTH_EXTENSION);
    }
}

void llInitFeatureSet2MPHY(uint8 enable)
{
    if(enable)
    {
        deviceFeatureSet.featureSet[1]|=(uint8)LL_FEATURE_2M_PHY;
        deviceFeatureSet.featureSet[0]|=(uint8)(LL_FEATURE_EXT_REJECT_IND  );
    }
    else
    {
        deviceFeatureSet.featureSet[1] &=(uint8)(~LL_FEATURE_2M_PHY);
        deviceFeatureSet.featureSet[0] &=(uint8)(~LL_FEATURE_EXT_REJECT_IND  );
    }
}

// this function is invoke by LL_ChanMapUpdate which is not used now
/*******************************************************************************
 * @fn          llAtLeastTwoChans
 *
 * @brief       This function determines if at least two data channels are
 *              being used, as required by a change in requirements per BT
 *              Core V4.0.0, Volume 6. The channel map is provided as an array
 *              of five bytes, which bits 0 to 36 represent the corresponding
 *              data channel. When set, the data channel is used. This routine
 *              first checks if more than one byte is not zero. If only one
 *              byte is zero, it checks if more than one bit in that byte is
 *              non-zero.
 *
 * input parameters
 *
 * @param       chanMap  - A five byte array containing one bit per data channel
 *                         where a 1 means the channel is "used".
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      Indicates whether the channel map is valid:
 *              TRUE:  Two or more data channels are being used.
 *              FALSE: Less than two data channels are being used.
 */
uint8 llAtLeastTwoChans( uint8 *chanMap )
{
  uint8 i;
  uint8 x            = 0; // for lint
  uint8 nonZeroBytes = 0;

  // channels 37..39 are not data channels and these bits should not be set
  chanMap[LL_NUM_BYTES_FOR_CHAN_MAP-1] &= 0x1F;

  // check each byte to see if more than one has a bit set
  for (i = 0; i < LL_NUM_BYTES_FOR_CHAN_MAP; i++)
  {
    // check/count if a byte is non-zero; we can leave once greater than one
    if ( (nonZeroBytes += ((chanMap[i] != 0)?1:0)) > 1 )
    {
      return( TRUE );
    }

    // save if any channels are set in case only one byte is non-zero
    if ( chanMap[i] )
    {
      x = chanMap[i];
    }
  }

  // check if no channels are used
  // Note: Value is either zero or one as we would have exited if greater
  //       than one.
  if ( nonZeroBytes != 0 )
  {
    // exactly one byte is non-zero; check if not a power of two
    return( (x & (x-1)) != 0);                 // 
  }

  // no channel bits are set
  return( FALSE );
}


/*******************************************************************************
 * @fn          llResetRfCounters
 *
 * @brief       This routine is used to reset global "rfCounters"
 *              these are per connection event counters and should be reset when start of event
 *              
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS, LL_STATUS_ERROR_COMMAND_DISALLOWED
 */
void llResetRfCounters(void)
{
    osal_memset(&rfCounters, 0, sizeof(rfCounters));
}


/*******************************************************************************
 * @fn          llCalcScaFactor
 *
 * @brief       This function is used when a connection is formed to calculate
 *              the timer drift divisor (called a timer drift factor) based on
 *              the combined SCA of the Master (as received in the CONNECT_REQ
 *              packet) and the Slave (based on either the default value of
 *              40ppm or the value set by HCI_EXT_SetSCA, from 0..500).
 *
 *              Note: In order to limit the number of build configurations,
 *                    POWER_SAVINGS is always defined, so this define can no
 *                    longer be used to determine if the Slave's SCA should be
 *                    included in timer drift calculations. Instead, the global
 *                    pwrmgr_device will be used instead.
 *              Note: When PM is not used, the Slave's SCA is zero and the
 *                    active clock accuracy is included in the overhead.
 *
 * input parameters
 *
 * @param       masterSCA - An ordinal value from 0..7 that corresponds to a
 *                          SCA range per Vol. 6, Part B, Section 2.3.3.1,
 *                          Table 2.2.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      The timer drift factor.
 */
uint16 llCalcScaFactor( uint8 masterSCA )
{
  uint16 sca = 0;
  uint32 tdFactor;

  // include the Slave's SCA in timer drift correction
  sca = adv_param.scaValue;

  // convert master's SCA to PPM and combine with slave
  sca += SCA[ masterSCA ];

  // calculate the Slave SCA factor
  //tdFactor = llDivide31By16To16( 1000000, sca );
  tdFactor = 1000000 / sca ;  

  return( (uint16)tdFactor );
}


// add by HZF
/*******************************************************************************
 * @fn          llGetNextAdvChn
 *
 * @brief       This function is used to get the next advertisement channel ascend. If current
 *              advertisement channel is not set, it will return the lowest advertisement
 *              channel according to advertisement channel map setting
 *
 *              
 *
 * input parameters
 *
 * @param       cur_chn - current adv channel No.
 *
 * output parameters
 *
 * @param       None
 *
 * @return      next advertise channel number.
 */
uint8 llGetNextAdvChn(uint8 cur_chn)
{
    uint8 temp, next_chn;
    uint8 i;
    
    // sanity check, should not be here
    if ((adv_param.advChanMap & LL_ADV_CHAN_ALL ) == 0)
        return 0xff;
    
    // if cur_chn adv channel number invalid, return the 1st vaild adv channel No.
    if (cur_chn > 39 || cur_chn < 37)
    {
        i = 0;
        while (!(adv_param.advChanMap & (1 << i)))   i ++;
        
        return  (ADV_BASE_IDX + i);
    }    

    // duplicate adv channel map
    temp = ((adv_param.advChanMap & LL_ADV_CHAN_ALL ) << 3) | (adv_param.advChanMap & LL_ADV_CHAN_ALL );
    
    // shift from next possible channel
    i = cur_chn - 36;
    
    while (!(temp & (1 << i)) && i < 6)
        i ++ ;
    
    i = (i >= 3)? (i - 3) : i;
    
    next_chn = ADV_BASE_IDX + i;
    
    return next_chn;   
}    

/*******************************************************************************
 * @fn          llGetNextAdvChn
 *
 *           
 *
 * input parameters
 *
 * @param       index - index of conn context
 *
 * output parameters
 *
 * @param       None
 *
 * @return      None
 */
void reset_conn_buf(uint8 index)
{
    int i;
	llConnState_t *connPtr;

    connPtr = &conn_param[index]; 
    
    for(i = 0; i < g_maxPktPerEventTx; i++)
	{
        connPtr->ll_buf.tx_conn_desc[i]->valid = 0;
        connPtr->ll_buf.tx_conn_desc[i]->header = 0;
    }

    for(i = 0; i < g_maxPktPerEventRx; i++)
	{
        connPtr->ll_buf.rx_conn_desc[i]->valid = 0;
        connPtr->ll_buf.rx_conn_desc[i]->header = 0;		
    }
        
    connPtr->ll_buf.tx_write = 0;
    connPtr->ll_buf.tx_read = 0;
    connPtr->ll_buf.tx_loop = 0;

    connPtr->ll_buf.rx_write = 0;
    connPtr->ll_buf.rx_read = 0;   
    connPtr->ll_buf.rx_loop = 0;
    
    connPtr->ll_buf.ntrm_cnt = 0;
    connPtr->ll_buf.tx_not_ack_pkt->valid = 0;      // bug fix 2018-05-21. In slave initiate conn term case, android mobile may not send ACK, and this status will be kept
}

// Tx loop buffer
void update_tx_write_ptr(llConnState_t *connPtr)
{
    connPtr->ll_buf.tx_write ++;
    if (connPtr->ll_buf.tx_write >= g_maxPktPerEventTx)
    {
        connPtr->ll_buf.tx_loop = 1;       // exceed the bank 0, then enter bank 1
        connPtr->ll_buf.tx_write = 0;
    }
}

void update_tx_read_ptr(llConnState_t *connPtr)
{
    connPtr->ll_buf.tx_read ++;
    
    if (connPtr->ll_buf.tx_read >= g_maxPktPerEventTx)
    {
        if ( connPtr->ll_buf.tx_loop == 1)
        {
            connPtr->ll_buf.tx_read = 0;      // now read ptr & write ptr in the same bank, clear flag
            connPtr->ll_buf.tx_loop = 0;
        }
        else
        {
            // error, should not be here. Don't increase read ptr
            connPtr->ll_buf.tx_read --;            
        }
    }
}

uint8_t getTxBufferSize(llConnState_t *connPtr)
{
    if (connPtr->ll_buf.tx_loop)
        return g_maxPktPerEventTx + connPtr->ll_buf.tx_write - connPtr->ll_buf.tx_read;
    else
        return connPtr->ll_buf.tx_write - connPtr->ll_buf.tx_read;

}

uint8_t getTxBufferFree(llConnState_t *connPtr)
{
    return (g_maxPktPerEventTx - getTxBufferSize(connPtr) );
}

uint8_t get_tx_read_ptr(llConnState_t *connPtr)
{
    return connPtr->ll_buf.tx_read;
}

uint8_t get_tx_write_ptr(llConnState_t *connPtr)
{
    return connPtr->ll_buf.tx_write;
}

// Rx loop buffer
void update_rx_write_ptr(llConnState_t *connPtr)
{
    connPtr->ll_buf.rx_write ++;
    if (connPtr->ll_buf.rx_write >= g_maxPktPerEventRx)
    {
        connPtr->ll_buf.rx_loop = 1;       // exceed the bank 0, then enter bank 1
        connPtr->ll_buf.rx_write = 0;
    }
}

void update_rx_read_ptr(llConnState_t *connPtr)
{
    connPtr->ll_buf.rx_read ++;
    
    if (connPtr->ll_buf.rx_read >= g_maxPktPerEventRx )
    {
        if (connPtr->ll_buf.rx_loop == 1)
        {
            connPtr->ll_buf.rx_read = 0;      // now read ptr & write ptr in the same bank, clear flag
            connPtr->ll_buf.rx_loop = 0;            
        }
        else
        {
            // error, should not be here. Don't increase read 
            connPtr->ll_buf.rx_read --;            
        }
    }
}

// get Rx buffer packet number
uint8_t getRxBufferSize(llConnState_t *connPtr)
{
    if (connPtr->ll_buf.rx_loop)
        return g_maxPktPerEventRx + connPtr->ll_buf.rx_write - connPtr->ll_buf.rx_read;
    else
        return connPtr->ll_buf.rx_write - connPtr->ll_buf.rx_read;
}

// get Rx buffer free packet number
uint8_t getRxBufferFree(llConnState_t *connPtr)
{
    return (g_maxPktPerEventRx - getRxBufferSize(connPtr) );
}

uint8_t get_rx_read_ptr(llConnState_t *connPtr)
{
    return connPtr->ll_buf.rx_read;
}

uint8_t get_rx_write_ptr(llConnState_t *connPtr)
{
    return connPtr->ll_buf.rx_write;
}

/*******************************************************************************
 * @fn          llSetupConn
 *
 * @brief       This function is used to determine the proper start time for the
 *              new master connection, and from this and the start time of the
 *              initiator, determine the initial window offset that will be
 *              adjusted dynamically each system tick until the CONNECT_REQ is
 *              transmitted.
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
//void llSetupConn( void )
//{
//
//}

/*******************************************************************************
 * @fn          llGenerateCRC
 *
 * @brief       This function is used to generate an initial 24-bit CRC value.

 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      A 24-bit CRC value.
 */
//uint32 llGenerateCRC( void )
//{
//  uint32 crcInit = 0;
//
//  // generate 24 bit CRC init value
//  // Note: Generating true random numbers requires the use of the radio.
//  ((uint8 *)&crcInit)[0] = LL_ENC_GeneratePseudoRandNum();
//  ((uint8 *)&crcInit)[1] = LL_ENC_GeneratePseudoRandNum();
//  ((uint8 *)&crcInit)[2] = LL_ENC_GeneratePseudoRandNum();
//
//  return( crcInit );
//}
//
//uint8 llValidAccessAddr( uint32 accessAddr );
//uint8 llGtSixConsecZerosOrOnes( uint32 accessAddr);
//uint8 llEqSynchWord( uint32 accessAddr);
//uint8 llOneBitSynchWordDiffer( uint32 accessAddr);
//uint8 llEqualBytes( uint32 accessAddr);
//uint8 llGtTwentyFourTransitions( uint32 accessAddr);
//uint8 llLtTwoChangesInLastSixBits( uint32 accessAddr);
//uint8 llEqAlreadyValidAddr( uint32 accessAddr ); 
//
//
///*******************************************************************************
// * @fn          llValidAccessAddr
// *
// * @brief       This function is called to check if an access address is a
// *              valid access address.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indication of valid access address:
// *              TRUE: Access address is valid.
// *              FALSE: Access address is invalid.
// */
//uint8 llValidAccessAddr( uint32 accessAddr )
//{
//  // check if this access address has already been chosen before
//  if ( !llEqAlreadyValidAddr( accessAddr ) )
//  {
//    // test if the access address meets all the requirements
//    if ( !llGtSixConsecZerosOrOnes(accessAddr)    &&  // test if there are greater than six consecutive zeros
//         !llEqSynchWord(accessAddr)               &&  // test if equal to advertising packet sync word
//         !llOneBitSynchWordDiffer(accessAddr)     &&  // test if it differs only in one bit from the advertising packet sync word
//         !llEqualBytes(accessAddr)                &&  // test if all four bytes are equal
//         !llGtTwentyFourTransitions(accessAddr)   &&  // test that there aren't more than 24 transitions
//         !llLtTwoChangesInLastSixBits(accessAddr) )   // test there's a minimum of two changes in the last six bit portion
//    {
//      return( TRUE );
//    }
//  }
//
//  return( FALSE );
//}
//
//
///*******************************************************************************
// * @fn          llGenerateValidAccessAddr
// *
// * @brief       This function is called to randomly generate a valid access
// *              address.
// *
// * input parameters
// *
// * @param       None.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      A valid 32-bit access address.
// */
//uint32 llGenerateValidAccessAddr( void )
//{
//  uint32 accessAddr;
//
//  // generate a valid access address
//  do
//  {
//    // generate a true random 32 bit number
//    // Note: Generating true random numbers requires the use of the radio.
//    ((uint8 *)&accessAddr)[0] = LL_ENC_GeneratePseudoRandNum();
//    ((uint8 *)&accessAddr)[1] = LL_ENC_GeneratePseudoRandNum();
//    ((uint8 *)&accessAddr)[2] = LL_ENC_GeneratePseudoRandNum();
//    ((uint8 *)&accessAddr)[3] = LL_ENC_GeneratePseudoRandNum();
//
//    // verify if it is valid
//  } while(  !llValidAccessAddr( accessAddr ) );
//
//  return( accessAddr );
//}
//
//
///*******************************************************************************
// * @fn          llGtSixConsecZerosOrOnes
// *
// * @brief       This function is called to check if there are more than six
// *              consecutive zeros or ones in the access address.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indicates if access address meets criteria:
// *              TRUE: Access address meets criteria.
// *              FALSE: Access address does not meet criteria.
// */
//uint8 llGtSixConsecZerosOrOnes( uint32 accessAddr )
//{
//  uint8 prevBit, nextBit, count, i;
//
//  // init count
//  count = 1;
//
//  // init first bit
//  prevBit = (uint8)(accessAddr & 1);
//
//  // for each subsequent bit
//  for (i=1; i<32; i++)
//  {
//    // get the next bit
//    nextBit = (uint8)( (accessAddr >> i) & 1 );
//
//    // check if it is the same as previous bit
//    if (nextBit == prevBit)
//    {
//      // same, so increment count and check if more than six
//      if (++count > 6)
//      {
//        // yep, more than six ones or zeros in a row
//        return( TRUE );
//      }
//    }
//    else // different from prev
//    {
//      // so replace prev with this bit, reset count, and keep checking
//      prevBit = nextBit;
//      count = 1;
//    }
//  }
//
//  return( FALSE );
//}
//
//
///*******************************************************************************
// * @fn          llEqSynchWord
// *
// * @brief       This function is called to check if the access address is equal
// *              to the pre-defined Advertiser synch word.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indicates if access address meets criteria:
// *              TRUE: Access address meets criteria.
// *              FALSE: Access address does not meet criteria.
// */
//uint8 llEqSynchWord( uint32 accessAddr )
//{
//  return ( (uint8)(accessAddr == (uint32)ADV_SYNCH_WORD) );
//}
//
//
///*******************************************************************************
// * @fn          llOneBitSynchWordDiffer
// *
// * @brief       This function is called to check if the access address differs
// *              from the pre-defined Advertiser synch word by only one bit.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indicates if access address meets criteria:
// *              TRUE: Access address meets criteria.
// *              FALSE: Access address does not meet criteria.
// */
//uint8 llOneBitSynchWordDiffer( uint32 accessAddr )
//{
//  uint32 numOnes;
//
//  // exclusive or
//  numOnes = (accessAddr ^ (uint32)ADV_SYNCH_WORD);
//
//  // check if a power of two (i.e. only one bit is set)
//  // Note: If accessAddr is ADV_SYNCH_WORD, the this routine will return TRUE
//  //       even though there are no bits that differ. But note that the routine
//  //       llEqSynchWord will catch this case, so we don't really need to
//  //       check for it here. However, to be thorough, the explict check for
//  //       no bits differing is made here.
//  return( (numOnes != 0) && ((numOnes & (numOnes-1)) == 0) );
//}
//
//
///*******************************************************************************
// * @fn          llEqualBytes
// *
// * @brief       This function is called to check if the access address's four
// *              bytes are equal.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indicates if access address meets criteria:
// *              TRUE: Access address meets criteria.
// *              FALSE: Access address does not meet criteria.
// */
//uint8 llEqualBytes( uint32 accessAddr)
//{
//  uint8 b0, b1, b2, b3;
//
//  b0 = ((uint8 *)&accessAddr)[0];  //((accessAddr >>  0) & 0xFF);
//  b1 = ((uint8 *)&accessAddr)[1];  //((accessAddr >>  8) & 0xFF);
//  b2 = ((uint8 *)&accessAddr)[2];  //((accessAddr >> 16) & 0xFF);
//  b3 = ((uint8 *)&accessAddr)[3];  //((accessAddr >> 24) & 0xFF);
//
//  return( (uint8)((b0 == b1) && (b0 == b2) && (b0 == b3)) );
//}
//
//
///*******************************************************************************
// * @fn          llGtTwentyFourTransitions
// *
// * @brief       This function is called to check if the access address has more
// *              than 24 bit transitions.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indicates if access address meets criteria:
// *              TRUE: Access address meets criteria.
// *              FALSE: Access address does not meet criteria.
// */
//uint8 llGtTwentyFourTransitions( uint32 accessAddr )
//{
//  uint8 prevBit, nextBit, count, i;
//
//  // init count
//  count = 0;
//
//  // init first bit
//  prevBit = (uint8)(accessAddr & 1);
//
//  // for each subsequent bit
//  for (i=1; i<32; i++)
//  {
//    // check if next bit is same as previous bit
//    nextBit = (uint8)((accessAddr >> i) & 1);
//
//    // check if we have a transition or not
//    if (nextBit != prevBit)
//    {
//      // transition, so increment count and check if more than 24
//      if (++count > 24)
//      {
//        // yep, more than 24 transitions
//        return( TRUE );
//      }
//
//      // replace prev with this bit
//      prevBit = nextBit;
//    }
//  }
//
//  return( FALSE );
//}
//
//
///*******************************************************************************
// * @fn          llLtTwoChangesInLastSixBits
// *
// * @brief       This function is called to check if the access address has a
// *              minimum of two changes in the last six bits.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indicates if access address meets criteria:
// *              TRUE: Access address meets criteria.
// *              FALSE: Access address does not meet criteria.
// */
//uint8 llLtTwoChangesInLastSixBits( uint32 accessAddr )
//{
//  uint8 prevBit, nextBit, count, i;
//
//  // init count
//  count = 0;
//
//  // init first bit, which is bit 27 as we want only the last six bits
//  prevBit = (uint8)((accessAddr >> 26) & 1);
//
//  // for each subsequent bit
//  for (i=27; i<32; i++)
//  {
//    // check if next bit is same as previous bit
//    nextBit = (uint8)((accessAddr >> i) & 1);
//
//    // check if we have a transition or not
//    if (nextBit != prevBit)
//    {
//      // transition, so increment count and check if equal to 2
//      if (++count == 2)
//      {
//        // yep, at least 2 transitions in last six bits
//        return( FALSE );
//      }
//
//      // replace prev with this bit
//      prevBit = nextBit;
//    }
//  }
//
//  // less than 2 transitions in last six bits
//  return( TRUE );
//}
//
//
///*******************************************************************************
// * @fn          llEqAlreadyValidAddr
// *
// * @brief       This function is called to check if the access address is the
// *              the same as any already existing valid access address.
// *
// *              Note: It is assumed that a LL connection data structure has not
// *                    yet been allocated as this is a potential connection. The
// *                    the check of already existing identical access addresses
// *                    is restricted to only already existing connections.
// *
// * input parameters
// *
// * @param       accessAddr - Connection synchronization word.
// *
// * output parameters
// *
// * @param       None.
// *
// * @return      Indicates if access address meets criteria:
// *              TRUE: Access address meets criteria.
// *              FALSE: Access address does not meet criteria.
// */
//uint8 llEqAlreadyValidAddr( uint32 accessAddr )
//{
//    #if 0
//  uint8 i;
//
//  // check any already existing connections don't have the same access address
//  for (i=0; i<llConns.numLLConns; i++)
//  {
//    if ( accessAddr == llConns.llConnection[ i ].accessAddr )
//    {
//      return( TRUE );
//    }
//  }
//  #endif
//  
//  return( FALSE );
//}

// our 24bit HW timer could support up to 4.096second
// adjust slave latency value if (slaveLatency + 1) * connInteval > 4s(not use 4.096 to ease the caculation)
#define LL_MAX_ALLOW_TIMER                             4000000
void llAdjSlaveLatencyValue( llConnState_t *connPtr )
{
    while ((connPtr->slaveLatencyValue + 1) * connPtr->curParam.connInterval > (LL_MAX_ALLOW_TIMER / 625)
           && (connPtr->slaveLatencyValue > 0))
    {
        connPtr->slaveLatencyValue --;
    }
    
}
//add by ZQ 20181030 for DLE feature
void llPduLengthManagmentReset(void)
{
    g_llPduLen.suggested.MaxRxOctets=LL_PDU_LENGTH_SUPPORTED_MAX_RX_OCTECTS;
    g_llPduLen.suggested.MaxTxOctets=LL_PDU_LENGTH_SUPPORTED_MAX_TX_OCTECTS;
    g_llPduLen.suggested.MaxRxTime  =LL_PDU_LENGTH_SUPPORTED_MAX_RX_TIME;
    g_llPduLen.suggested.MaxTxTime  =LL_PDU_LENGTH_SUPPORTED_MAX_TX_TIME;

	for (int i = 0; i < g_maxConnNum; i++)
    {
        conn_param[i].llPduLen.local.MaxRxOctets=LL_PDU_LENGTH_INITIAL_MAX_RX_OCTECTS;
        conn_param[i].llPduLen.local.MaxTxOctets=LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS;
        conn_param[i].llPduLen.local.MaxRxTime  =LL_PDU_LENGTH_INITIAL_MAX_RX_TIME;
        conn_param[i].llPduLen.local.MaxTxTime  =LL_PDU_LENGTH_INITIAL_MAX_TX_TIME;

        conn_param[i].llPduLen.remote.MaxRxOctets=LL_PDU_LENGTH_INITIAL_MAX_RX_OCTECTS;
        conn_param[i].llPduLen.remote.MaxTxOctets=LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS;
        conn_param[i].llPduLen.remote.MaxRxTime  =LL_PDU_LENGTH_INITIAL_MAX_RX_TIME;
        conn_param[i].llPduLen.remote.MaxTxTime  =LL_PDU_LENGTH_INITIAL_MAX_TX_TIME;    
		conn_param[i].llPduLen.isProcessingReq=FALSE;
		conn_param[i].llPduLen.isWatingRsp	  =FALSE;
		conn_param[i].llPduLen.isChanged	  =FALSE;

    }

//    llTrxNumAdaptiveConfig();

    return;
}

//add by ZQ 20181030 for DLE feature
void llPduLengthUpdate0(uint16 connHandle)
{
    llConnState_t *connPtr;

	connPtr = &conn_param[connHandle];
    connPtr->llPduLen.isChanged      =FALSE;

    if(     connPtr->llPduLen.remote.MaxRxOctets   <   LL_PDU_LENGTH_INITIAL_MAX_RX_OCTECTS
        ||  connPtr->llPduLen.remote.MaxRxTime     <   LL_PDU_LENGTH_INITIAL_MAX_RX_TIME
        ||  connPtr->llPduLen.remote.MaxTxOctets   <   LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS
        ||  connPtr->llPduLen.remote.MaxTxTime     <   LL_PDU_LENGTH_INITIAL_MAX_TX_TIME       )
    {
            //should not be here
          
    }
    else
    {   
        //compare remote Tx with suggested Rx
        if( connPtr->llPduLen.local.MaxRxOctets 
            !=MIN(  g_llPduLen.suggested.MaxRxOctets,connPtr->llPduLen.remote.MaxTxOctets))
        {
             connPtr->llPduLen.local.MaxRxOctets =    MIN(  g_llPduLen.suggested.MaxRxOctets, connPtr->llPduLen.remote.MaxTxOctets);
             connPtr->llPduLen.isChanged =TRUE;
        }

        if( connPtr->llPduLen.local.MaxRxTime 
            !=MIN(  g_llPduLen.suggested.MaxRxTime,connPtr->llPduLen.remote.MaxTxTime))
        {
             connPtr->llPduLen.local.MaxRxTime =    MIN(  g_llPduLen.suggested.MaxRxTime, connPtr->llPduLen.remote.MaxTxTime);
             connPtr->llPduLen.isChanged =TRUE;
        }

        //compare remote Rx with suggested Tx
        if(connPtr->llPduLen.local.MaxTxOctets 
            !=MIN(  g_llPduLen.suggested.MaxTxOctets, connPtr->llPduLen.remote.MaxRxOctets))
        {
             connPtr->llPduLen.local.MaxTxOctets = MIN(g_llPduLen.suggested.MaxTxOctets, connPtr->llPduLen.remote.MaxRxOctets);
             connPtr->llPduLen.isChanged =TRUE;
        }

        if(connPtr->llPduLen.local.MaxTxTime 
            !=MIN(  g_llPduLen.suggested.MaxTxTime, connPtr->llPduLen.remote.MaxRxTime))
        {
             connPtr->llPduLen.local.MaxTxTime =    MIN(  g_llPduLen.suggested.MaxTxTime, connPtr->llPduLen.remote.MaxRxTime);
             connPtr->llPduLen.isChanged =TRUE;
        }

    }

//    llTrxNumAdaptiveConfig();
    //------------------------------------------------------------
    //nofify upper layer
    if(connPtr->llPduLen.isChanged)
    {
        LL_DataLengthChangeCback(connHandle, 
                connPtr->llPduLen.local.MaxTxOctets,
                connPtr->llPduLen.local.MaxTxTime, 
                connPtr->llPduLen.local.MaxRxOctets,
                connPtr->llPduLen.local.MaxRxTime);
    }
    
    if(g_dle_taskID!=0)
        osal_set_event(g_dle_taskID,g_dle_taskEvent);

    return;
}

//add by ZQ 20181106 for PHY MODE Update feature
void llPhyModeCtrlReset(void)
{

    // For symmetric connection, 
    // it should make both fields the same, only specifiying a single PHY
    for (int i = 0; i < g_maxConnNum; i ++)
    {
		conn_param[i].llPhyModeCtrl.local.txPhy=LE_1M_PHY;
		conn_param[i].llPhyModeCtrl.local.rxPhy=LE_1M_PHY;
		
		conn_param[i].llPhyModeCtrl.isProcessingReq = FALSE;
		conn_param[i].llPhyModeCtrl.isWatingRsp 	= FALSE;
		conn_param[i].llPhyModeCtrl.isChanged		= FALSE;	

    }
	g_rfPhyPktFmt = PKT_FMT_BLE1M;      // to consider whether should move to connection context
	
    return;
}

void llPhyModeCtrlUpdateNotify(llConnState_t *connPtr, uint8 status)
{

    if(status==LL_STATUS_SUCCESS)
    {


        connPtr->llPhyModeCtrl.local.txPhy = connPtr->phyUpdateInfo.s2mPhy;
        connPtr->llPhyModeCtrl.local.rxPhy = connPtr->phyUpdateInfo.m2sPhy;
    }

    LL_PhyUpdateCompleteCback((uint16)(connPtr->connId),
                                  status,
                                  connPtr->phyUpdateInfo.s2mPhy,
                                  connPtr->phyUpdateInfo.m2sPhy);
 

    

    if(g_phyChg_taskID!=0)
        osal_set_event(g_phyChg_taskID,g_phyChg_taskEvent);

    return;
}

/////////////////  end of file //////////////////////////////////

