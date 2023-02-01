
/*******************************************************************************
  Filename:       ll.c, updated base on ll.h
  Revised:        
  Revision:       

  Description:    This file contains the Link Layer (LL) API for the Bluetooth
                  Low Energy (BLE) Controller. It provides the defines, types,
                  and functions for all supported Bluetooth Low Energy (BLE)
                  commands.

                  This API is based on the Bluetooth Core Specification,
                  V4.0.0, Vol. 6.


*******************************************************************************/
//#define DEBUG_LL

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "ll_buf.h"
#include "ll.h"
#include "ll_def.h"
#include "ll_common.h"
#include "ll_hw_drv.h"
#include "osal.h"
#include "OSAL_PwrMgr.h"
#include "osal_bufmgr.h"
#include "bus_dev.h"
#include "jump_function.h"
#include "global_config.h"
#include "ll_debug.h"
#include "ll_enc.h"
#include "rf_phy_driver.h"
#include "time.h"
#include "log.h"
// =============== compile flag, comment out if not required below feature
//#define  EXT_ADV_ENABLE

//#define OWN_PUBLIC_ADDR_POS      0x11004000

extern uint32_t get_timer_count(AP_TIM_TypeDef *TIMx);
/*******************************************************************************
 * MACROS
 */
 #define LL_COPY_DEV_ADDR_LE( dstPtr, srcPtr )        {                          \
  (dstPtr)[0] = (srcPtr)[0];                                                   \
  (dstPtr)[1] = (srcPtr)[1];                                                   \
  (dstPtr)[2] = (srcPtr)[2];                                                   \
  (dstPtr)[3] = (srcPtr)[3];                                                   \
  (dstPtr)[4] = (srcPtr)[4];                                                   \
  (dstPtr)[5] = (srcPtr)[5];}
// ALT: COULD USE OSAL COPY.
// osal_memcpy( (dstPtr), (srcPtr), LL_DEVICE_ADDR_LEN );

//#define LL_COPY_DEV_ADDR_BE( dstPtr, srcPtr )         {                         \
//  (dstPtr)[0] = (srcPtr)[5];                                                   \
//  (dstPtr)[1] = (srcPtr)[4];                                                   \
//  (dstPtr)[2] = (srcPtr)[3];                                                   \
//  (dstPtr)[3] = (srcPtr)[2];                                                   \
//  (dstPtr)[4] = (srcPtr)[1];                                                   \
//  (dstPtr)[5] = (srcPtr)[0];}
// ALT: COULD USE OSAL COPY.
// osal_memcpy( (dstPtr), (srcPtr), LL_DEVICE_ADDR_LEN );

//#define BDADDR_VALID( bdAddr )                                                 \
//  ( !(                                                                         \
//       ((bdAddr)[0] == 0xFF) &&                                                \
//       ((bdAddr)[1] == 0xFF) &&                                                \
//       ((bdAddr)[2] == 0xFF) &&                                                \
//       ((bdAddr)[3] == 0xFF) &&                                                \
//       ((bdAddr)[4] == 0xFF) &&                                                \
//       ((bdAddr)[5] == 0xFF)                                                   \
//     )                                                                         \
//  )

// See "Supported States Related" below.
//#define LL_SET_SUPPORTED_STATES( state )                                       \
//  states[ (state)>>4 ] |= (1<<((state) & 0x0F));

/*******************************************************************************
 * CONSTANTS
 */
// Bluetooth Version Information
//#define LL_VERSION_NUM                0x06  	// BT Core Specification V5.0.0, refer to https://www.bluetooth.com/specifications/assigned-numbers/host-controller-interface
//#define LL_COMPANY_ID                 0x0504    // Phyplus
//
//// Major Version (8 bits) . Minor Version (4 bits) . SubMinor Version (4 bits)
//#define LL_SUBVERSION_NUM             0x0208    //  Controller v1.0.0. Definition not found in BLE spec

// Connection Window Information
//#define LL_WINDOW_SIZE                2         // 2.5ms in 1.25ms ticks
//#define LL_WINDOW_OFFSET              0         // 1.25ms + 0

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*
** Own Device Address
**
** Note that the device may have a permanently assigned BLE address located in
** the Information Page. However, this address could be overridden with either
** the build flag BDADDR_FROM_FLASH, or by the vendor specific command
** LL_EXT_SET_BDADDR.
*/
// Own Device Public Address
uint8 ownPublicAddr[ LL_DEVICE_ADDR_LEN ];     // index 0..5 is LSO..MSB

// Own Device Random Address
uint8 ownRandomAddr[ LL_DEVICE_ADDR_LEN ];     // index 0..5 is LSO..MSB

//uint8 g_prdAdvRptEnable = FALSE;

// Delay Sleep
static uint16       sleepDelay;          // delay sleep for XOSC stabilization upon reset

extern uint32 llWaitingIrq;
extern struct buf_rx_desc g_rx_adv_buf;

//extern uint8    g_currentLocalRpa[LL_DEVICE_ADDR_LEN];
//extern uint8    g_currentPeerRpa[LL_DEVICE_ADDR_LEN];
extern uint8    g_currentPeerAddrType;
extern uint8    g_currentLocalAddrType;

//extern uint32_t llScanDuration;
/*******************************************************************************
 * GLOBAL VARIABLES
 */

advInfo_t           adv_param;       // Advertiser info
// add for scanner
scanInfo_t          scanInfo;        // Scanner info
// add for initiator 2018-05-09
initInfo_t          initInfo;        // Initiator info

// BBB new: the memory of LL connect context is allocated by APP
llConnState_t       *conn_param      = NULL;
uint8               g_maxConnNum     = 0;
uint8               g_maxPktPerEventTx = TYP_CONN_BUF_LEN;
uint8               g_maxPktPerEventRx = TYP_CONN_BUF_LEN;
//uint8               g_blePktVersion  = BLE_PKT_VERSION_4_0;

//move to llConnStat_t
//preChanMapUpdate_t preChanMapUpdate[MAX_NUM_LL_CONN];    

// A2 multi-connection
llConns_t           g_ll_conn_ctx;


uint8               LL_TaskID;          // OSAL LL task ID
uint8_t             llState;            // state of LL                            ==> to move to per connection context ???
peerInfo_t          peerInfo;           // peer device's address and address type   ==> to move to per connection context???
chanMap_t       	chanMapUpdate;      // channel map for updates
//featureSet_t   		deviceFeatureSet;   // feature set for this device
//verInfo_t           verInfo;            // own version information
//llConns_t         llConns;            // LL connections table
uint8              numComplPkts;        // number of completed Tx buffers, use global beacuse we report HCI event when numComplPkts >= numComplPktsLimit
uint8              numComplPktsLimit;   // minimum number of completed Tx buffers before event
rfCounters_t       rfCounters;          // counters for LL RX/TX atomic operation in one connection event

//uint8              fastTxRespTime;      // flag indicates if fast TX response time feature is enabled/disabled


// ============== A1 ROM metal change add 
uint32_t       g_llHdcDirAdvTime;      // for HDC direct adv
//==

//uint8         numComplPktsLimit;    // minimum number of completed Tx buffers before event
//uint8         numComplPktsFlush;    // flag to indicate send number of completed buffers at end of event
//uint8         fastTxRespTime;     // flag indicates if fast TX response time feature is enabled/disabled
//==

// RX Flow Control
//uint8        rxFifoFlowCtrl;        

//llLinkBuf_t  ll_buf;

//llGlobalStatistics_t g_pmCounters;             // TODO: to divide into per connection counters & global counters

// =====   A2 metal change add
//llPduLenManagment_t  g_llPduLen;    //for dle feature       ==> to move to per connection context
//llPhyModeManagment_t g_llPhyModeCtrl;// for phy update      ==> to move to per connection context
uint8_t             llSecondaryState;            // secondary state of LL
// ===== A2 add End
//extern l2capSARDbugCnt_t g_sarDbgCnt;

extern struct buf_tx_desc g_tx_adv_buf;
//extern struct buf_tx_desc g_tx_ext_adv_buf;

extern struct buf_tx_desc tx_scanRsp_desc;

//
uint8          g_currentTimerTask;          // scan or adv
uint32         g_timerExpiryTick;           // us

// 2020-02-15 add for connectionless IQ Sample buffer
//uint16 *g_pLLcteISample=NULL;
//uint16 *g_pLLcteQSample=NULL;


//uint8  g_llScanMode = LL_MODE_INVALID;
//uint8  g_llAdvMode =  LL_MODE_INVALID;


// RF path compensation
//extern int16  g_rfTxPathCompensation, g_rfRxPathCompensation;

/*******************************************************************************
 * @fn          LL_Init0
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
void LL_Init( uint8 taskId )
{
    LL_TaskID = taskId;
     
    // =========== calculate whiten seed
//	calculate_whiten_seed(); // must set!!!     


    // read flash driectly becasue HW has do the address mapping for read Flash operation
    uint8 *p = NULL;
    p = (uint8 *)pGlobal_config[MAC_ADDRESS_LOC];
    ownPublicAddr[3] = *(p++);
    ownPublicAddr[2] = *(p++);
    ownPublicAddr[1] = *(p++);
    ownPublicAddr[0] = *(p++);
    ownPublicAddr[5] = *(p++);
    ownPublicAddr[4] = *(p);
	 
    // set own random address as invalid until one is provided
    osal_memset(ownRandomAddr, 0xFF, B_ADDR_LEN);

	osal_memset(&adv_param, 0xFF, sizeof(advInfo_t));

//	for (int i = 0; i < g_maxConnNum; i++)
//    {
//        conn_param[i].connId    = i;
//		conn_param[i].active    = FALSE;
//		conn_param[i].allocConn = FALSE;
//    }


    // set default Scan values
    osal_memset(&scanInfo, 0xFF, sizeof(scanInfo_t));

    // set default Init values
    osal_memset(&initInfo, 0xFF, sizeof(initInfo_t));
  
    // reset the Link Layer
    LL_Reset();
    
    // generate true random number for AES-CCM
    LL_ENC_GenerateTrueRandNum( cachedTRNGdata, LL_ENC_TRUE_RAND_BUF_SIZE );

    numComplPkts         = 0;
    numComplPktsLimit    = 1;
      
    llState = LL_STATE_IDLE;
    
    // add in A2
    llSecondaryState = LL_SEC_STATE_IDLE;
  
    osal_pwrmgr_task_state( LL_TaskID, PWRMGR_CONSERVE );
    
    // default sleep delay
    sleepDelay = pGlobal_config[MIN_TIME_TO_STABLE_32KHZ_XOSC];
    
    // delay sleep to allow the 32kHz crystal to stablize
    osal_set_event( LL_TaskID, LL_EVT_START_32KHZ_XOSC_DELAY );
 }


/*******************************************************************************
 * @fn          LL_ProcessEvent0
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
 
uint16 LL_ProcessEvent( uint8 task_id, uint16 events )
{
    /*
    ** Directed Advertising results in a Master Connection
    */
    if ( events & LL_EVT_MASTER_CONN_CREATED )
    {
        llConnState_t *connPtr;

        connPtr = &conn_param[initInfo.connId];     

		{    		
            LL_ConnectionCompleteCback( LL_STATUS_SUCCESS,                       // reasonCode
                                        (uint16)connPtr->connId,                 // connection handle
                                        LL_LINK_CONNECT_COMPLETE_MASTER,          // role
                                        peerInfo.peerAddrType,                   // peer's address type
                                        peerInfo.peerAddr,                       // peer's address
                                        connPtr->curParam.connInterval >> 1,     // connection interval, back to 1.25ms units
                                        connPtr->curParam.slaveLatency,              // slave latency
                                        connPtr->curParam.connTimeout >> 4,      // connection timeout, back to 10ms units
                                        0 );             // sleep clock accurracy
		}
	   return (events ^ LL_EVT_MASTER_CONN_CREATED );
    }

    /*
    ** Create Connection Cancel
    */
    if ( events & LL_EVT_MASTER_CONN_CANCELLED )
    {
        // notify Host
        LL_ConnectionCompleteCback( LL_STATUS_ERROR_UNKNOWN_CONN_HANDLE,     // reasonCode
                                (uint16)0,                               // connection handle
                                LL_LINK_CONNECT_COMPLETE_MASTER,         // role
                                peerInfo.peerAddrType,                   // peer's address type
                                peerInfo.peerAddr,                       // peer's address
                                0,                                       // connection interval, back to 1.25ms units
                                0,                                       // slave latency
                                0,                                       // connection timeout, back to 10ms units
                                0 );                                     // sleep clock accurracy not valid for master

        return (events ^ LL_EVT_MASTER_CONN_CANCELLED );
    }


    /*
    ** Directed Advertising failed to connect
    */
    if ( events & LL_EVT_DIRECTED_ADV_FAILED )
    {
        // notify Host
        LL_ConnectionCompleteCback( LL_STATUS_ERROR_DIRECTED_ADV_TIMEOUT,    // reasonCode
                                    (uint16)0,                               // connection handle
                                    LL_LINK_CONNECT_COMPLETE_SLAVE,          // role
                                    peerInfo.peerAddrType,                   // peer's address type
                                    peerInfo.peerAddr,                       // peer's address
                                    0,                                       // connection interval, back to 1.25ms units
                                    0,                                       // slave latency
                                    0,                                       // connection timeout, back to 10ms units
                                    0 );                                     // sleep clock accurracy
    
        return (events ^ LL_EVT_DIRECTED_ADV_FAILED );
    }
	
    /*
    ** Directed Advertising results in a Slave Connection
    */
    if ( events & LL_EVT_SLAVE_CONN_CREATED )
    {
        llConnState_t *connPtr;
    
        connPtr = &conn_param[0];        
		{    		
            LL_ConnectionCompleteCback( LL_STATUS_SUCCESS,                       // reasonCode
                                        (uint16)connPtr->connId,                 // connection handle
                                        LL_LINK_CONNECT_COMPLETE_SLAVE,          // role
                                        peerInfo.peerAddrType,                   // peer's address type
                                        peerInfo.peerAddr,                       // peer's address
                                        connPtr->curParam.connInterval >> 1,     // connection interval, back to 1.25ms units
                                        connPtr->curParam.slaveLatency,              // slave latency
                                        connPtr->curParam.connTimeout >> 4,      // connection timeout, back to 10ms units
                                        connPtr->sleepClkAccuracy );             // sleep clock accurracy
		}
        return (events ^ LL_EVT_SLAVE_CONN_CREATED );
    }

    /*
    ** Advertising results in a Slave Connection with an Unacceptable
    ** Connection Interval
    */
//    if ( events & LL_EVT_SLAVE_CONN_CREATED_BAD_PARAM )
//    {
//        // notify Host
//        LL_ConnectionCompleteCback( LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL, // reasonCode
//                                    (uint16)0,                                  // connection handle
//                                    LL_LINK_CONNECT_COMPLETE_SLAVE,             // role
//                                    peerInfo.peerAddrType,                      // peer's address type
//                                    peerInfo.peerAddr,                          // peer's address
//                                    0,                                          // connection interval, back to 1.25ms units
//                                    0,                                          // slave latency
//                                    0,                                          // connection timeout, back to 10ms units
//                                    0 );                                        // sleep clock accurracy
//    
//        return (events ^ LL_EVT_SLAVE_CONN_CREATED_BAD_PARAM );
//    }

    /*
    ** Ensure the 32kHz crystal is stable after POR/External Reset/PM3
    ** before allowing sleep.
    ** Note: There is nothing in hardware to indicate this to software.
    */
    if ( events & LL_EVT_START_32KHZ_XOSC_DELAY )
    {
        // check if the delay hasn't been disabled
        if ( sleepDelay )
        {
            //osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
            (void)osal_pwrmgr_task_state( LL_TaskID, PWRMGR_HOLD );

            osal_start_timerEx( LL_TaskID, LL_EVT_32KHZ_XOSC_DELAY, sleepDelay );
        }

        return (events ^ LL_EVT_START_32KHZ_XOSC_DELAY );
    }

    /*
    ** The minimum stabilization delay for the 32kHz crystal has expired.
    */
    if ( events & LL_EVT_32KHZ_XOSC_DELAY )
    {
        // delay over, so allow sleep
        //osal_pwrmgr_device( PWRMGR_BATTERY );
        (void)osal_pwrmgr_task_state( LL_TaskID, PWRMGR_CONSERVE );
    
        return (events ^ LL_EVT_32KHZ_XOSC_DELAY );
    }
    // ===== scan
    if (events & LL_EVT_SECONDARY_SCAN)
    {
        if (llSecondaryState == LL_SEC_STATE_IDLE || scanInfo.scanMode == LL_SCAN_STOP)	  // scan may be cancel during waiting period, do nothing in this case
            llSecondaryState = LL_SEC_STATE_IDLE;
        else
        {
            llSetupSecScan(scanInfo.nextScanChan);
        }
		  
        return (events ^ LL_EVT_SECONDARY_SCAN );
    }

    // ======= add for A2 multi-conn, init
    if (events & LL_EVT_SECONDARY_INIT)
    {
        if (llSecondaryState == LL_SEC_STATE_IDLE || initInfo.scanMode == LL_SCAN_STOP)	  // scan may be cancel during waiting period, do nothing in this case
            llSecondaryState = LL_SEC_STATE_IDLE;
        else
        {
            llSetupSecInit(initInfo.nextScanChan);
              // TODO
        }
		  
        return (events ^ LL_EVT_SECONDARY_INIT );
    }

    
    return 0;		 
 }


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
void *LL_TX_bm_alloc( uint16 size )
{
    uint8 *pBuf;

    size =  (size+LL_ENC_BLOCK_LEN-1)&0xfff0;//align to 16Byte
    pBuf = osal_bm_alloc( size             +
                          sizeof(txData_t) +
                          LL_PKT_HDR_LEN   +
                          LL_PKT_MIC_LEN );
  
    if ( pBuf != NULL )
    {
        // return pointer to user payload
        return( pBuf + ((uint16)sizeof(txData_t)+LL_PKT_HDR_LEN) );
    }
  
    return( (void *)NULL );
}


/*******************************************************************************
 * This API is used to allocate memory using buffer management.
 *
 * Public function defined in ll.h.
 */
void *LL_RX_bm_alloc( uint16 size )
{
    uint8 *pBuf;
  
    // Note: This is the lowest call for RX buffer management allocation.
  
    pBuf = osal_bm_alloc( size + HCI_RX_PKT_HDR_SIZE );
  
    if ( pBuf != NULL )
    {
        // return pointer to user payload
        return( pBuf + HCI_RX_PKT_HDR_SIZE );
    }
  
    return( (void *)NULL );
}

/*******************************************************************************
 * This function is used by the HCI to reset and initialize the LL Controller.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_Reset0( void )
{
    // enter critical section
    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
	
    	
    // set the peer's device address type, as allowed for connections by Host
    osal_memset(&peerInfo, 0x00, sizeof(peerInfo_t));
    // initialize default channel map
    osal_memset(&chanMapUpdate, 0xFF, sizeof(chanMap_t));
    // set state/role
    llState = LL_STATE_IDLE;
//    ll_debug_output(DEBUG_LL_STATE_IDLE);
    
    // add in A2
    llSecondaryState = LL_SEC_STATE_IDLE;    

    numComplPkts         = 0;
    numComplPktsLimit    = 1;

	for (int i = 0; i < g_maxConnNum; i ++)
    {
        llResetConnId(i);
	    reset_conn_buf(i);
    }
    
	// HZF: add for multi-connection
	g_ll_conn_ctx.currentConn = LL_INVALID_CONNECTION_ID;
	g_ll_conn_ctx.numLLConns = 0;
	g_ll_conn_ctx.numLLMasterConns = 0;

    // exit critical section
    HAL_EXIT_CRITICAL_SECTION();
  
    return( LL_STATUS_SUCCESS );
}



/*******************************************************************************
 * @fn          LL_Disconnect0 API
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
llStatus_t LL_Disconnect0( uint16 connId,
                                 uint8  reason )
{
    llStatus_t    status;
    llConnState_t *connPtr;
  	  		
    // make sure connection ID is valid
    if ( (status = LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }
  
    // get connection info
    connPtr = &conn_param[connId];
  
    // check if any control procedure is already pending
    if ( connPtr->ctrlPktInfo.ctrlPktActive == TRUE )
    {
        // check if a terminate control procedure is already what's pending
        if ( connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_TERMINATE_IND )
        {
            return( LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE );
        }
        else // spec now says a terminate can happen any time
        {
            // indicate the peer requested this termination
            connPtr->termInfo.reason = reason;
      
            // de-activate slave latency to expedite termination
            connPtr->slaveLatency = 0;
      
            // override any control procedure that may be in progress
            llReplaceCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
        }
    }
    else // no control procedure currently active, so set this one up
    {
        // indicate the peer requested this termination
        connPtr->termInfo.reason = reason;
    
        // de-activate slave latency to expedite termination
        connPtr->slaveLatency = 0;
    
        // queue control packet for processing
        llEnqueueCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
    }
  
    return( LL_STATUS_SUCCESS );
}	

/*******************************************************************************
 * @fn          LL_TxData0 API
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
llStatus_t LL_TxData0( uint16 connId,
                      uint8 *pBuf,
                      uint8  pktLen,
                      uint8  fragFlag )
{
    llConnState_t *connPtr;
    txData_t      *pTxData;

    // get the connection info based on the connection ID
    connPtr = &conn_param[connId];   

    // sanity check input parameters
    if ( (pBuf == NULL) /*|| (pktLen > connPtr->llPduLen.local.MaxTxOctetsLL_MAX_LINK_DATA_LEN) */||
         ((fragFlag != LL_DATA_FIRST_PKT_HOST_TO_CTRL) &&
          (fragFlag != LL_DATA_CONTINUATION_PKT)) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // make sure connection ID is valid
    if ( !conn_param [connId].active )
    {
        //return( status );
        return( LL_STATUS_ERROR_INACTIVE_CONNECTION );
    }
    
  
    // check if the number of available data buffers has been exceeded
    if ( getTxBufferFree(connPtr) == 0)
    {
        return( LL_STATUS_ERROR_OUT_OF_TX_MEM );
    }

    // adjust pointer to start of packet (i.e. at the header)
    pBuf -= LL_PKT_HDR_LEN;

    // set the packet length field
    // Note: The LLID and Length fields are swapped for the nR (for DMA).
    pBuf[0] = pktLen;

    // set LLID fragmentation flag in header
    // Note: NESN=SN=MD=0 and is handled by RF.
    pBuf[1] = (fragFlag==LL_DATA_FIRST_PKT_HOST_TO_CTRL) ?
             LL_DATA_PDU_HDR_LLID_DATA_PKT_FIRST       : // first pkt
             LL_DATA_PDU_HDR_LLID_DATA_PKT_NEXT;         // continuation pkt

    // ALT: Place check if packet needs to be encrypted and encryption here,
    //      but be careful about control packets and getting them out of order!

    // point to Tx data entry
    pTxData = (txData_t *)(pBuf - sizeof(txData_t));

    // it does, so queue up this data
    llEnqueueDataQ( &connPtr->txDataQ, pTxData );

    // check that we are either the master or slave, and if so, that the current
    // connection is connId
    if ( !(((llState == LL_STATE_CONN_MASTER) || (llState == LL_STATE_CONN_SLAVE))))
        //(connId == g_ll_conn_ctx.currentConn)) )
    {
        // either we are not the master or the slave, or the we are but the connId
        // is not the current connection, so just return success as the data is
        // queued on the connection
        return( LL_STATUS_SUCCESS );
    }

    // copy any pending data to the TX FIFO
    llProcessTxData( connPtr, LL_TX_DATA_CONTEXT_SEND_DATA );

    // indicate the packet is sent of buffered
    return( LL_STATUS_SUCCESS );		
}

/*******************************************************************************
 * @fn          LL_SetAdvParam0 API
 *
 * @brief       This API is called by the HCI to set the Advertiser's
 *              parameters.
 *
 * input parameters
 * @param       advIntervalMin - The minimum Adv interval.
 * @param       advIntervalMax - The maximum Adv interval.
 * @param       advEvtType     - The type of advertisment event.
 * @param       ownAddrType    - The Adv's address type of public or random.
 * @param       peerAddrType   - BLE4.0: Only used for directed advertising.  BLE4.2: Peer address type
 * @param       *peerAddr      - BLE4.0: Only used for directed advertising (NULL otherwise). BLE4.2: Peer address
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
llStatus_t LL_SetAdvParam0( uint16 advIntervalMin,
                           uint16 advIntervalMax,
                           uint8  advEvtType,
                           uint8  ownAddrType,
                           uint8  peerAddrType,
                           uint8  *peerAddr,
                           uint8  advChanMap,
                           uint8  advWlPolicy )
{
  uint8 pduType;
  
  // check if advertising is active
  if ( adv_param.advMode == LL_ADV_MODE_ON )
  {
    // yes, so not allowed per the spec
    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
  }

  adv_param.advEvtType = advEvtType;
  adv_param.ownAddrType = ownAddrType;

  // save off the advertiser channel map
  adv_param.advChanMap = advChanMap & 0x07;

  // make sure there's at least one advertising channel that can be used
  if ( !adv_param.advChanMap )
  {
    // force all to be usable in case the error message is ignored
    adv_param.advChanMap = LL_ADV_CHAN_ALL;

    return( LL_STATUS_ERROR_NO_ADV_CHAN_FOUND );
  }

  // set the advertiser address based on the HCI's address type preference
  if ( ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC )
  {
    // get our address and address type
    g_currentLocalAddrType = LL_DEV_ADDR_TYPE_PUBLIC;
    LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownPublicAddr );
  }
  else if ( ownAddrType == LL_DEV_ADDR_TYPE_RANDOM )
  {
    // get our address and address type
    g_currentLocalAddrType  = LL_DEV_ADDR_TYPE_RANDOM;
    LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownRandomAddr );
  }
  
  // save peer address info, to consider whether we need it
  if (peerAddr != NULL)
      LL_COPY_DEV_ADDR_LE( peerInfo.peerAddr, peerAddr );
  peerInfo.peerAddrType = peerAddrType;


  // a Connectable Directed Adv event requires Init address info
  if ( advEvtType == LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT )
  {
    // get the Init's address and address type as well
    LL_COPY_DEV_ADDR_LE( peerInfo.peerAddr, peerAddr );

    // the advertising interval and delay are not used
    adv_param.advInterval = 2;//0;    // set by HZF, 2 means 1.25ms, so 3 adv channel = 3.75ms, spec require < 3.75ms
  }
  else if ( advEvtType == LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT )
  {
    // get the Init's address and address type as well
    LL_COPY_DEV_ADDR_LE( peerInfo.peerAddr, peerAddr );

    // calculate the advertising interface based on the max/min values
    // ALT: COULD UPDATE WITH ALGO IF NEED BE.
    adv_param.advInterval = advIntervalMin;
  }
  else // undirected, discoverable, or non-connectable
  {		
    // calculate the advertising interface based on the max/min values
    // ALT: COULD UPDATE WITH ALGO IF NEED BE.
    adv_param.advInterval = advIntervalMin;
  }
  // mapping from adv event type to packet header 
  switch (adv_param.advEvtType)
  {
    case LL_ADV_CONNECTABLE_UNDIRECTED_EVT:
		pduType = ADV_IND;
             break;
    case LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT:
    case LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT:	
             pduType = ADV_DIRECT_IND;
             break;	
    case LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT:
             pduType = ADV_NONCONN_IND;
             break;
//    case LL_ADV_SCANNABLE_UNDIRECTED_EVT:
//             pduType = ADV_SCAN_IND;
//             break;
    default:
             // should not come here, sanity check in the function start. set default value to suppress warning
             pduType = ADV_IND;
             break;		
  }	
  SET_BITS(g_tx_adv_buf.txheader, pduType, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
  SET_BITS(g_tx_adv_buf.txheader, (g_currentLocalAddrType & 0x01), TX_ADD_SHIFT, TX_ADD_MASK);
//  SET_BITS(g_tx_adv_buf.txheader, peerInfo.peerAddrType, RX_ADD_SHIFT, RX_ADD_MASK);   // RxAdd need't set
  
  osal_memcpy( g_tx_adv_buf.data,  adv_param.ownAddr, 6);

  SET_BITS(tx_scanRsp_desc.txheader, ADV_SCAN_RSP, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
  SET_BITS(tx_scanRsp_desc.txheader, g_currentLocalAddrType, TX_ADD_SHIFT, TX_ADD_MASK);  
  osal_memcpy( tx_scanRsp_desc.data, adv_param.ownAddr, 6);	

   // adv length should be set for not direct adv type, 2018-04-05
   SET_BITS(g_tx_adv_buf.txheader, (adv_param.advDataLen + 6), LENGTH_SHIFT, LENGTH_MASK);

  // for direct adv, copy the peer address to PDU
	if(pduType == ADV_DIRECT_IND )
	{
		SET_BITS(g_tx_adv_buf.txheader, 12, LENGTH_SHIFT, LENGTH_MASK);
		osal_memcpy((uint8_t*) &(g_tx_adv_buf.data[6]), peerInfo.peerAddr, 6);
		SET_BITS(g_tx_adv_buf.txheader, peerInfo.peerAddrType, RX_ADD_SHIFT, RX_ADD_MASK); 
	}
						
  return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * @fn          LL_SetAdvData0 API
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
llStatus_t LL_SetAdvData0( uint8  advDataLen,
                          uint8 *advData )
{
  // check that data length isn't greater than the max allowed size
  if ( advDataLen > LL_MAX_ADV_DATA_LEN )
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }

  // save advertiser data length
  adv_param.advDataLen = advDataLen;
	
  // check if there's supposed to be data
  if ( advDataLen > 0 )
  {
    // yes, so make sure we have a valid pointer
    if ( advData == NULL )
    {
      return( LL_STATUS_ERROR_BAD_PARAMETER );
    }
    else // okay to go
    {
      // save advertiser data
      //osal_memcpy( (uint8_t *)adv_param.advData, advData, adv_param.advDataLen );                   // save adv data
      osal_memcpy( (uint8_t *) &(g_tx_adv_buf.data[6]), advData, adv_param.advDataLen );    // write adv to tx buffer, change it ?? ... HZF
    }
  }

// set tx buffer, to be changed
//   SET_BITS(g_tx_adv_buf.txheader, peerInfo .peerAddrType, RX_ADD_SHIFT, RX_ADD_MASK);

  // osal_memcpy(g_tx_adv_buf.data,  adv_param.ownAddr, 6);

   SET_BITS(g_tx_adv_buf.txheader, (adv_param.advDataLen+6), LENGTH_SHIFT, LENGTH_MASK);

  return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * This API is called by the HCI to request the Controller to start or stop
 * advertising.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_SetAdvControl0( uint8 advMode )
{
    _HAL_CS_ALLOC_(); 
	#ifdef _PHY_DEBUG 
		  LOG("%s,%s,Line %d,advMode %d,adv_param.advMode %d,llState %d\n",__FILE__,__func__,__LINE__,advMode,adv_param.advMode,llState);
	#endif
    // check if we should begin advertising
    switch( advMode )
    {
        // Advertisment Mode is On
        case LL_ADV_MODE_ON:
            // check if command makes sense
            if ( adv_param.advMode == LL_ADV_MODE_ON )
            {
              // this is unexpected; something is wrong
              return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
            }

            if (llState == LL_STATE_IDLE)
            {
                switch(adv_param .advEvtType)
                {
                    case LL_ADV_CONNECTABLE_UNDIRECTED_EVT:
                        llState=LL_STATE_ADV_UNDIRECTED;
		    	 
                        break;
                    case LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT:
                    case LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT:
                        llState=LL_STATE_ADV_DIRECTED;		     
                        break;
		    	 
                    case LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT:
                        llState=LL_STATE_ADV_NONCONN;	   
                        break;
		    	 
                    default:
                        llState=LL_STATE_IDLE;	     
                        break;
                }
            }
                        
            // llState changed when configure adv parameters
            if (llState == LL_STATE_ADV_UNDIRECTED 
			 || llState == LL_STATE_ADV_DIRECTED 
			 || llState == LL_STATE_ADV_NONCONN  )     // TODO: check this setting
            {
                g_llHdcDirAdvTime = 0;    // for HDC direct adv
                adv_param.advNextChan = LL_ADV_CHAN_LAST + 1;       // set adv channel invalid
                if ( llSetupAdv() != LL_STATUS_SUCCESS )
                {           
                    // indicate advertising is no longer active
                    adv_param.advMode = LL_ADV_MODE_OFF;
            
                    return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
                }
            }
			// add in A2, simultaneous conn event & scan/adv event
            else             
                return (LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE);
            
            // indicate advertising is no longer active
            adv_param.advMode = LL_ADV_MODE_ON;

            break;
    
        case LL_ADV_MODE_OFF:			
            // check if command makes sense
            if ( adv_param.advMode == LL_ADV_MODE_OFF )
            {
                // this is unexpected; something is wrong
                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
            }
            
            HAL_ENTER_CRITICAL_SECTION();
            
            // free the associated task block
            //llFreeTask( &advInfo.llTask );
            
            // indicate we are no longer actively advertising
            adv_param.advMode = LL_ADV_MODE_OFF;
			llState = LL_STATE_IDLE;	 // if not in connect state, set idle to disable advertise
			clear_timer(AP_TIM1);
            HAL_EXIT_CRITICAL_SECTION();
            break;
    
        default:
            // we have an invalid value for advertisement mode
            return( LL_STATUS_ERROR_BAD_PARAMETER );
    }
  
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * This API is called by the LL or HCI to check if a connection given by the
 * connection handle is active.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_ConnActive( uint16 connId )
{
  // check if the connection handle is valid
  if (connId >= g_maxConnNum ) 
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }
  
  // check if the connection is active
  if ( conn_param[connId].active == FALSE )
  {
    return( LL_STATUS_ERROR_INACTIVE_CONNECTION );
  }

  return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * This API is called by the HCI to request the LL Controller to provide a data
 * block with random content.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_Rand( uint8 *randData,
                    uint8 dataLen )
{
    uint16 temp_rand;
    uint8 *pData = randData;
	  uint32 sysTick;

    if (dataLen == 0)
    {
      return (LL_STATUS_ERROR_BAD_PARAMETER);
    }
				
    // now use timer3 counter
    sysTick = get_timer_count(AP_TIM3);

    srand(sysTick);
    //srand((unsigned)time(&t));
    while (dataLen > 1)
    {
      temp_rand = (uint16)(rand() & 0xffff);
      *(pData ++) = (uint8)((temp_rand & 0xff00) >> 8);
      *(pData ++) = (uint8)(temp_rand & 0xff) ;
      dataLen -= 2;      
    }

    if (dataLen == 1)
    {
      temp_rand = (uint16)(rand() & 0xffff);
      *(pData) = (uint8)((temp_rand & 0xff00) >> 8);
    }
    
    return( LL_STATUS_SUCCESS );

}

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
llStatus_t LL_ReadRssi0( uint16 connId,
                         int8  *lastRssi )
{
    *lastRssi = conn_param[connId].lastRssi ;
    return( LL_STATUS_SUCCESS );    
}

llStatus_t LL_ReadFoff( uint16 connId,
                         uint16  *foff )
{
    *foff = conn_param[connId].foff ;
    return( LL_STATUS_SUCCESS );    
}

llStatus_t LL_ReadCarrSens( uint16 connId,
                         uint8  *carrSens )
{
    *carrSens = conn_param[connId].carrSens ;
    return( LL_STATUS_SUCCESS );    
}
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
llStatus_t LL_SetScanParam0( uint8  scanType,
                            uint16 scanInterval,
                            uint16 scanWindow,
                            uint8  ownAddrType,
                            uint8  scanWlPolicy )
{


  // check if scan is active
  if ( scanInfo.scanMode == LL_SCAN_START )
  {
    // yes, so not allowed per the spec
    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
  }

  // set the scan type
  scanInfo.scanType = scanType;

  scanInfo.ownAddrType = ownAddrType;

  // set the scanner's address based on the HCI's address type preference
  if ( ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC )
  {
    // get our address and address type
    LL_COPY_DEV_ADDR_LE( scanInfo.ownAddr, ownPublicAddr );
  }                            
  else if ( ownAddrType == LL_DEV_ADDR_TYPE_RANDOM )// LL_DEV_ADDR_TYPE_RANDOM
  {
    // get our address and address type
    LL_COPY_DEV_ADDR_LE( scanInfo.ownAddr, ownRandomAddr );
  }

  // set the scan interval
  scanInfo.scanInterval = scanInterval;

  // set the scan window
  scanInfo.scanWindow = scanWindow;

  return( LL_STATUS_SUCCESS );

}	

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
llStatus_t LL_SetScanControl0( uint8 scanMode,
                              uint8 filterReports )
{
    _HAL_CS_ALLOC_(); 

    // check if we should begin scanning
    switch( scanMode )
    {
        // Scanning Mode is On
        case LL_SCAN_START:
            // check if command makes sense
            if ( scanInfo.scanMode == LL_SCAN_START )
            {
                // this is unexpected; something is wrong
                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
            }

            // check if no other tasks are currently active
            if ( llState == LL_STATE_IDLE )
            {
                // indicate Scan has not already been initalized
                scanInfo.initPending = TRUE;

                // save the scan filtering flag
                scanInfo.filterReports = filterReports;      

                // add by HZF
                scanInfo.nextScanChan  = LL_SCAN_ADV_CHAN_37;
                // set LL state
                llState = LL_STATE_SCAN;          
          
                // Note: llState has been changed.
                LL_evt_schedule();
            }
            else if (llState == LL_STATE_CONN_MASTER)
            {
                if (llSecondaryState != LL_SEC_STATE_IDLE)
                    return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );  

                scanInfo.nextScanChan  = LL_SCAN_ADV_CHAN_37;
                llSecondaryState = LL_SEC_STATE_SCAN;
                osal_set_event(LL_TaskID, LL_EVT_SECONDARY_SCAN);
            }
            else
                return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );  

            // indicate we are actively scanning
            scanInfo.scanMode = LL_SCAN_START;			
            break;

        case LL_SCAN_STOP:

            HAL_ENTER_CRITICAL_SECTION();

    	    if (llState == LL_STATE_SCAN)      // no conn + scan case
    	    {
                llState = LL_STATE_IDLE;     // if not in connect state, set idle to disable scan

                //ZQ 20190912
                //stop ll timer when idle, considering the scan-adv interleve case
                clear_timer(AP_TIM1);
                
//                ll_debug_output(DEBUG_LL_STATE_IDLE);
    	    }
			else if (llState == LL_STATE_CONN_MASTER)                      // conn + scan case
			{
                llSecondaryState = LL_SEC_STATE_IDLE;
				osal_stop_timerEx(LL_TaskID, LL_EVT_SECONDARY_SCAN);
			}	

            // indicate we are no longer actively scanning
            scanInfo.scanMode = LL_SCAN_STOP;

            // A2 multiconn, should we consider current LL state to avoid change master/slave configuration
            // now LL slave/master event use same parameter 88
			ll_hw_set_rx_timeout(88); 
      
            // HZF: should we stop scan task immediately, or wait scan IRQ then stop? Now use option 2.
            HAL_EXIT_CRITICAL_SECTION();
			
			// for gcc : strict type check
			uint32 llwaitIrqAddr = (uint32)&llWaitingIrq;
	  		while(read_reg(llwaitIrqAddr) == TRUE);


      break;

        default:
      // we have an invalid value for advertisement mode
      return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    return( LL_STATUS_SUCCESS );
}


/*******************************************************************************
 * This API is called by the HCI to set the Advertiser's Scan Response data.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_SetScanRspData( uint8  scanRspLen,
                              uint8 *scanRspData )
{
	
  // check that data length isn't greater than the max allowed size
  if ( scanRspLen > LL_MAX_SCAN_DATA_LEN )
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }

  // save scan response data length
  adv_param.scanRspLen = scanRspLen;

  // check if there is any scan response data
  if ( scanRspLen > 0 )
  {
    // yes, so make sure we have a valid pointer
    if ( scanRspData == NULL )
    {
      return( LL_STATUS_ERROR_BAD_PARAMETER );
    }
    else // okay to go
    {
      // save scan response data
      osal_memcpy( (uint8_t *) &(tx_scanRsp_desc.data[6]), scanRspData, adv_param.scanRspLen );
    }
  }

	SET_BITS(tx_scanRsp_desc.txheader,  (adv_param .scanRspLen+6), LENGTH_SHIFT, LENGTH_MASK);
		
  return( LL_STATUS_SUCCESS );
}

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
llStatus_t LL_EncLtkReply( uint16 connId,
                                  uint8  *key )
{
  uint8         i;
  llStatus_t    status;
  llConnState_t *connPtr;

  // make sure we're in Master role
  if ( llState != LL_STATE_CONN_SLAVE )
  {
    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
  }

  // check parameters
  if ( key == NULL )
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }

  // make sure connection ID is valid
  if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
  {
    return( status );
  }

  // get connection info
  connPtr = &conn_param[ connId ];

  // ALT: COULD MAKE THIS PER CONNECTION.

  // save LTK
  for (i=0; i<LL_ENC_LTK_LEN; i++)
  {
    // store LTK in MSO..LSO byte order, per FIPS 197 (AES)
    connPtr->encInfo.LTK[(LL_ENC_LTK_LEN-i)-1] = key[i];
  }

  // indicate the host has provided the key
  connPtr->encInfo.LTKValid = TRUE;

  // got the LTK, so schedule the start of encryption
  // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
  //      THE QUEUE.
  llEnqueueCtrlPkt( connPtr, LL_CTRL_START_ENC_REQ );

  return( LL_STATUS_SUCCESS );

}	

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
//llStatus_t LL_EncLtkNegReply( uint16 connId )
//{
//  llStatus_t    status;
//  llConnState_t *connPtr;
//
//  // make sure we're in Master role
//  if ( llState != LL_STATE_CONN_SLAVE )
//  {
//    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
//  }
//
//  // make sure connection ID is valid
//  if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
//  {
//    return( status );
//  }
//
//  // get connection info
//  connPtr = &conn_param[ connId ];
//
//  // check if this is during a start or a re-start encryption procedure
//  if ( connPtr->encInfo.encRestart == TRUE )
//  {
//    // indicate the peer requested this termination
//    connPtr->termInfo.reason = LL_ENC_KEY_REQ_REJECTED;
//
//    // queue control packet for processing
//    // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
//    //      THE QUEUE.
//    //llReplaceCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
//    llEnqueueCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
//  }
//  else // during a start encryption
//  {
//    // set the encryption rejection error code
//    connPtr->encInfo.encRejectErrCode = LL_STATUS_ERROR_PIN_OR_KEY_MISSING; // same as LL_ENC_KEY_REQ_REJECTED
//
//    // and reject the encryption request
//    // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
//    //      THE QUEUE.
//    //llReplaceCtrlPkt( connPtr, LL_CTRL_REJECT_IND );
//    llEnqueueCtrlPkt( connPtr, LL_CTRL_REJECT_IND );
//  }
//
//  return( LL_STATUS_SUCCESS );
//
//}

/*******************************************************************************
 * This API is called by the HCI to read the controller's own public device
 * address.
 *
 * Note: The device's address is stored in NV memory.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_ReadBDADDR( uint8 *bdAddr )
{
  // return own public device address LSO..MSO
  bdAddr[0] = ownPublicAddr[0];
  bdAddr[1] = ownPublicAddr[1];
  bdAddr[2] = ownPublicAddr[2];
  bdAddr[3] = ownPublicAddr[3];
  bdAddr[4] = ownPublicAddr[4];
  bdAddr[5] = ownPublicAddr[5];

  return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * This function is used to save this device's random address. It is provided by
 * the Host for devices that are unable to store an IEEE assigned public address
 * in NV memory.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_SetRandomAddress( uint8 *devAddr )
{
    // store our random address LSO..MSO
    ownRandomAddr[0] = devAddr[0];
    ownRandomAddr[1] = devAddr[1];  
    ownRandomAddr[2] = devAddr[2];
    ownRandomAddr[3] = devAddr[3];
    ownRandomAddr[4] = devAddr[4];
    ownRandomAddr[5] = devAddr[5];

    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * This API is called by the HCI to create a connection.
 *
 * Public function defined in ll.h.
 */
// TODO: check the usage of new enum value of  ownAddrType/peerAddrType
llStatus_t LL_CreateConn0( uint16 scanInterval,
                          uint16 scanWindow,
                          uint8  initWlPolicy,
                          uint8  peerAddrType,
                          uint8  *peerAddr,
                          uint8  ownAddrType,
                          uint16 connIntervalMin,
                          uint16 connIntervalMax,
                          uint16 connLatency,
                          uint16 connTimeout,
                          uint16 minLength,        //  minimum length of connection needed for this LE conn, no use now
                          uint16 maxLength )       //  maximum length of connection needed for this LE conn, no use now
{

    uint8          i;
    llConnState_t *connPtr;
    uint16         txHeader = 0x2205;           // header for CONNECT REQ message, length: 0x22, PDU type: 0x5, TxAdd & RxAdd to be set below

    // multi-connction limits check
	if (g_ll_conn_ctx.numLLConns >= g_maxConnNum)
	{
	    return( LL_STATUS_ERROR_OUT_OF_CONN_RESOURCES );
	}

	  // if there is at least one connection, make sure this connection interval
	  // is a multiple/divisor of all other active connection intervals; also make
	  // sure that this connection's interval is not less than the allowed maximum
	  // connection interval as determined by the maximum number of allowed
	  // connections times the number of slots per connection.
//	  if ( g_ll_conn_ctx.numLLMasterConns > 0 ) 	   //	if ( g_ll_conn_ctx.numLLConns > 0 )
//	  {
//		 
//		uint16 connInterval = (connIntervalMax << 1);	   // convert to 625us ticks
//		uint16 minCI		= g_ll_conn_ctx.connInterval;
//	
//	
//	//	  // does the CI need to be checked as a multiple of the minCI?
//		if ( connInterval >= minCI )
//		{
//		  // check if this connection's CI is valid (i.e. a multiple of minCI)
//		  if ( connInterval % minCI )
//		  {
//			return( LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL );
//		  }
//		}
//		else
//			return( LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL );
//	  }
//	  else
//	  {   // TODO: should we consider if there is only slave connection, using the interval of slave as the standard interval of master?
//	      // how could application know the slave interval?
//	  }
    if (llState == LL_STATE_CONN_MASTER )
   	{
      if (llSecondaryState == LL_SEC_STATE_IDLE)
	  	  llSecondaryState = LL_SEC_STATE_INIT;
	  else
	  	  return LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE;
   	}
//		LOG("%s,state %d-%d\n",__func__,llState,llSecondaryState);

	// add for pico
//	if( llState == LL_STATE_INIT  )
//		return LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE;
	
    // allocate a connection and assure it is valid
    if ( (connPtr = llAllocConnId()) == NULL )
    {
        llSecondaryState = LL_SEC_STATE_IDLE;      // recover llSecondaryState   
        // exceeded the number of available connection structures
        return( LL_STATUS_ERROR_CONNECTION_LIMIT_EXCEEDED );
    }

    g_ll_conn_ctx.numLLMasterConns ++;
	
    
//    connId = 0;
//    connPtr = &conn_param[connId];
    // reset connection parameters
	LL_set_default_conn_params(connPtr);
    
    // clear the connection buffer
    reset_conn_buf(connPtr->connId);        

    // save the connection ID with Init
    initInfo.connId = connPtr->connId;

    // set the scan interval
    initInfo.scanInterval = scanInterval;

    // set the scan window
    initInfo.scanWindow = scanWindow;

    // set the Init white list policy
//    initInfo.wlPolicy = initWlPolicy;

    // set the connection channel map
    for (i = 0; i < LL_NUM_BYTES_FOR_CHAN_MAP; i++)
    {
        connPtr->chanMap[i] = chanMapUpdate.chanMap[i];
    }

    // process connection channel map into the data channel table
    llProcessChanMap( connPtr, connPtr->chanMap );
    
    // Core spec V4.0 requires that a minimum of two data channels be used
//    if ( connPtr->numUsedChans < 2 )
//    {
//        // it isn't, so release the resource
//        llReleaseConnId( connPtr );
//		g_ll_conn_ctx.numLLMasterConns --;
//
//        return( LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION );
//    }

    // save the peer address
    if (peerAddr != NULL)              // bug fixed
        LL_COPY_DEV_ADDR_LE( peerInfo.peerAddr, peerAddr );
	


    // save our address type
    initInfo.ownAddrType = ownAddrType;

    // check the type of own address
//    if ( ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC )
    {
        // save our address
        LL_COPY_DEV_ADDR_LE( initInfo.ownAddr, ownPublicAddr );
    }
//    else // LL_DEV_ADDR_TYPE_RANDOM
//    {
//        // save our address
//        LL_COPY_DEV_ADDR_LE( initInfo.ownAddr, ownRandomAddr );
//    }

    // save the peer address type
    peerInfo.peerAddrType = peerAddrType;	
    
    txHeader |=  (ownAddrType << TX_ADD_SHIFT & TX_ADD_MASK);
    txHeader |=  (peerAddrType << RX_ADD_SHIFT & RX_ADD_MASK);  

    // ramdomly generate a valid 24 bit CRC value
    connPtr->initCRC = llGenerateCRC();

    // randomly generate a valid, previously unused, 32-bit access address
    connPtr->accessAddr = llGenerateValidAccessAddr();

//	g_ll_conn_ctx.scheduleInfo[connPtr->allocConn].linkRole = LL_ROLE_MASTER;    // will change the role in move_to_master_function

    if (g_ll_conn_ctx.numLLMasterConns == 1)    // A2 multi-connection, 1st connection, save the connection parameters
    {
        // determine the connection interval based on min and max values
        // Note: Range not used, so assume max value.
        // Note: minLength and maxLength are informational.
        connPtr->curParam.connInterval = connIntervalMax;

        // set the connection timeout
        // Note: The spec says this begins at the end of the CONNECT_REQ, but the
        //       LSTO will be converted into events.
        connPtr->curParam.connTimeout = connTimeout;

        // set the slave latency
        connPtr->curParam.slaveLatency = connLatency;  

		// save connection parameter as global 
		g_ll_conn_ctx.connInterval = connPtr->curParam.connInterval;                              // unit: 1.25ms
		g_ll_conn_ctx.slaveLatency = connPtr->curParam.slaveLatency;
		g_ll_conn_ctx.connTimeout  = connPtr->curParam.connTimeout;	

		g_ll_conn_ctx.per_slot_time = pGlobal_config[LL_MULTICONN_MASTER_SLOT];//connPtr->curParam.connInterval * 2 / g_maxConnNum;       // unit: 625us
    }
	else
    {
        // determine the connection interval based on min and max values
        // Note: Range not used, so assume max value.
        // Note: minLength and maxLength are informational.
        connPtr->curParam.connInterval = g_ll_conn_ctx.connInterval;

        // set the connection timeout
        // Note: The spec says this begins at the end of the CONNECT_REQ, but the
        //       LSTO will be converted into events.
        connPtr->curParam.connTimeout = g_ll_conn_ctx.connTimeout;

        // set the slave latency
        connPtr->curParam.slaveLatency = g_ll_conn_ctx.slaveLatency;    		
    }


    // set the master's SCA
    extern const uint16 SCA[] ;
    connPtr->sleepClkAccuracy = 0;//initInfo.scaValue;

    // set the window size (units of 1.25ms)
    // Note: Must be the lesser of 10ms and the connection interval - 1.25ms.
    connPtr->curParam.winSize = pGlobal_config[LL_CONN_REQ_WIN_SIZE];//5;//LL_WINDOW_SIZE;

    // set the window offset (units of 1.25ms). TO change if we support multiple connections
    // Note: Normally, the window offset is managed dynamically so that precise
    //       connection start times can be achieved (necessary for multiple
    //       connnections). However, sometimes it is useful to force the window
    //       offset to something specific for testing. This can be done by here
    //       when the project is built with the above define.
    // Note: This define should only be used for testing one connection and will
    //       NOT work when multiple connections are attempted!
    connPtr->curParam.winOffset = pGlobal_config[LL_CONN_REQ_WIN_OFFSET];//2;//LL_WINDOW_OFFSET;

    // set the channel map hop length (5..16)
    // Note: 0..255 % 12 = 0..11 + 5 = 5..16.
    connPtr->hop = (uint8)( (LL_ENC_GeneratePseudoRandNum() % 12) + 5);

    // track advertising channel for advancement at each scan window
    initInfo.nextScanChan = LL_SCAN_ADV_CHAN_37;

    // enable Init scan
    initInfo.scanMode = LL_SCAN_START;
    
    // check if this is the only task
    // Note: If there are one or more master connections already running, then
    //       the Init task will be scheduled when the next connection ends.
    
    {
        // ============= construct CONN REQ message payload in g_tx_adv_buf buffer. Using byte copy to avoid
        // hardfault cause by no word align reading
        uint8 offset = 0;
        
        g_tx_adv_buf.txheader = txHeader;    
        // setup CONN REQ in connPtr->ll_buf
        LL_ReadBDADDR(&g_tx_adv_buf.data[offset]);                    // initA, Byte 0 ~ 5
        offset += 6;
		if (peerAddr != NULL)
            LL_COPY_DEV_ADDR_LE(&g_tx_adv_buf.data[offset], peerAddr)     // AdvA,  Byte 6 ~ 11
        offset += 6;
        // Access Address, Byte 12 ~ 15
        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->accessAddr, 4);
        offset += 4;
        // CRC init, Byte 16 ~ 18
        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->initCRC, 3);
        offset += 3;
        
        // WinSize, Byte 19
        g_tx_adv_buf.data[offset] = connPtr->curParam.winSize;
        offset += 1;
        // WinOffset, Byte 20 ~ 21
        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->curParam.winOffset, 2);
        offset += 2;
        // Interval, Byte 22 ~ 23
        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->curParam.connInterval, 2); 
        offset += 2;        
        // Latency, Byte 24 ~ 25
        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->curParam.slaveLatency, 2);           
        offset += 2;        
        // Timeout, Byte 26 ~ 27
        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->curParam.connTimeout, 2);        
        offset += 2;        
        // Channel Map, Byte 28 ~ 32
        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->chanMap[0], 5);
        offset += 5;
        
        // Hop(5bit) + SCA(3bit), Byte 33
        g_tx_adv_buf.data[offset] = (connPtr->hop & 0x1f) | ((connPtr->sleepClkAccuracy & 0x7) << 5);

        // go ahead and start Init immediately
//        llSetupInit( connPtr->connId );

        // TODO: calc new connection start time and set dynamic window offset
        //llSetupConn();

    }

	if ( llState == LL_STATE_IDLE )
        // go ahead and start Init immediately
        llSetupInit( connPtr->connId );		
	else
        osal_set_event(LL_TaskID, LL_EVT_SECONDARY_INIT);
	
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * This API is called by the HCI to cancel a previously given LL connection
 * creation command that is still pending. This command should only be used
 * after the LL_CreateConn command as been issued, but before the
 * LL_ConnComplete callback.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_CreateConnCancel0( void )
{
  uint8 cancel_now = FALSE;
  uint8 connId;
  
  // ensure Init is active
  if ( initInfo.scanMode == LL_SCAN_STOP)
  {
	  // no create connection in progress
	  return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
  }

  // if scan has not trigger, release conn ID and report. Note: we assume only 1 type of scan could be enabled at the same time
  if (initInfo.scanMode == LL_SCAN_START)
	  connId = initInfo.connId;
  else
	  return LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE;

  if (llState == LL_STATE_INIT && !llWaitingIrq)	  // primary LL state is init case
  {
	  cancel_now = TRUE;
	  llState = LL_STATE_IDLE;
  }
  else if( llSecondaryState == LL_SEC_STATE_INIT && !llWaitingIrq )
  {
	  cancel_now = TRUE;
	  llSecondaryState = LL_SEC_STATE_IDLE;
  }
  else if (llSecondaryState == LL_SEC_STATE_INIT_PENDING)  // secondary LL state is init case
  {
	  cancel_now = TRUE;
	  llSecondaryState = LL_SEC_STATE_IDLE;
  }

  // indicate we are no longer actively scanning
  initInfo.scanMode = LL_SCAN_STOP;
  
  // if the scan is not ongoing, release conn ID
  if (cancel_now == TRUE)
  {
	  llReleaseConnId(&conn_param[connId]); 
	  g_ll_conn_ctx.numLLMasterConns --;
	  (void)osal_set_event( LL_TaskID, LL_EVT_MASTER_CONN_CANCELLED );		   // inform high layer   
  }

  return( LL_STATUS_SUCCESS );
}


/*******************************************************************************
 * This API is called by the Master HCI to setup encryption and to update
 * encryption keys in the LL connection. If the connection is already in
 * encryption mode, then this command will first pause the encryption before
 * subsequently running the encryption setup.
 *
 * Public function defined in ll.h.
 */
//llStatus_t LL_StartEncrypt0( uint16 connId,
//                            uint8  *rand,
//                            uint8  *eDiv,
//                            uint8  *ltk )
//{
//  uint8         i;
//  llStatus_t    status;
//  llConnState_t *connPtr;
//
//  // make sure we're in Master role
//  if ( llState != LL_STATE_CONN_MASTER )
//  {
//    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
//  }
//
//  // check parameters
//  if ( (rand == NULL) || (eDiv == NULL) || (ltk == NULL) )
//  {
//    return( LL_STATUS_ERROR_BAD_PARAMETER );
//  }
//
//  // make sure connection ID is valid
//  if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
//  {
//    return( status );
//  }
//
//  // get connection info
//  connPtr = &conn_param[connId]; 
//
//  // check if a feature response control procedure has taken place
//  if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
//  {
//    // it hasn't so re-load this device's local Feature Set to the
//    // connection as it may have been changed by the Host with HCI
//    // extenstion Set Local Feature Set command
////    for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
////    {
////      connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
////    }
//  }
//
//  // check if encryption is a supported feature set item
//  if ( (connPtr->featureSetInfo.featureSet[0] & LL_FEATURE_ENCRYPTION) != LL_FEATURE_ENCRYPTION )
//  {
//    return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
//  }
//
//  // cache the master's random vector
//  // Note: The RAND will be left in LSO..MSO order as this is assumed to be the
//  //       order of the bytes that will be returned to the Host.
//  for (i=0; i<LL_ENC_RAND_LEN; i++)
//  {
//    connPtr->encInfo.RAND[i] = rand[i];
//  }
//
//  // cache the master's encryption diversifier
//  // Note: The EDIV will be left in LSO..MSO order as this is assumed to be the
//  //       order of the bytes that will be returned to the Host.
//  connPtr->encInfo.EDIV[0] = eDiv[0];
//  connPtr->encInfo.EDIV[1] = eDiv[1];
//
//  // cache the master's long term key
//  // Note: The order of the bytes will be maintained as MSO..LSO
//  //       per FIPS 197 (AES).
//  for (i=0; i<LL_ENC_LTK_LEN; i++)
//  {
//    connPtr->encInfo.LTK[(LL_ENC_LTK_LEN-i)-1] = ltk[i];
//  }
//
//  // generate SKDm
//  // Note: The SKDm LSO is the LSO of the SKD.
//  // Note: Placement of result forms concatenation of SKDm and SKDs.
//  // Note: The order of the bytes will be maintained as MSO..LSO
//  //       per FIPS 197 (AES).
//  LL_ENC_GenDeviceSKD( &connPtr->encInfo.SKD[ LL_ENC_SKD_M_OFFSET ] );
//
//  // generate IVm
//  // Note: The IVm LSO is the LSO of the IV.
//  // Note: Placement of result forms concatenation of IVm and IVs.
//  // Note: The order of the bytes will be maintained as MSO..LSO
//  //       per FIPS 197 (AES).
//  LL_ENC_GenDeviceIV( &connPtr->encInfo.IV[ LL_ENC_IV_M_OFFSET ] );
//
//  // schedule a cache update of FIPS TRNG values for next SKD/IV usage
////  postRfOperations |= LL_POST_RADIO_CACHE_RANDOM_NUM;
//  
//  (void)LL_ENC_GenerateTrueRandNum( cachedTRNGdata, LL_ENC_TRUE_RAND_BUF_SIZE );
//
//  // set flag to stop all outgoing transmissions
//  connPtr->txDataEnabled = FALSE;
//
//  // invalidate the existing session key, if any
//  connPtr->encInfo.SKValid = FALSE;
//
//  // indicate the LTK is not valid
//  connPtr->encInfo.LTKValid = FALSE;
//
//  // check if we are already in encryption mode
//  if ( connPtr->encEnabled == TRUE )
//  {
//    // set a flag to indicate this is a restart (i.e. pause-then-start)
//    connPtr->encInfo.encRestart = TRUE;
//
//    // setup a pause encryption control procedure
//    llEnqueueCtrlPkt( connPtr, LL_CTRL_PAUSE_ENC_REQ );
//  }
//  else // no, so...
//  {
//    // clear flag to indicate this is an encryption setup
//    connPtr->encInfo.encRestart = FALSE;
//
//    // setup an encryption control procedure
//    llEnqueueCtrlPkt( connPtr, LL_CTRL_ENC_REQ );
//  }
//
//  return( LL_STATUS_SUCCESS );
//
//}

#define DBG_DISABLE_LATENCY 0
llStatus_t LL_PLUS_DisableSlaveLatency0(uint8 connId)
{
    llConnState_t *connPtr;
    uint16  next_event, elapse_event,chanMapDltEvt;
    uint32  remain_time1, remain_time2;
    uint32  old_timerDrift;
    uint32  time_advance;  

    // get connection information
    connPtr = &conn_param[connId];        // only 1 connection now, HZF    

    // check whether it should recover from latency
    if (FALSE == connPtr->active
      || llState != LL_STATE_CONN_SLAVE
      || llSecondaryState != LL_SEC_STATE_IDLE)
    {
        //LOG("==1==\n\r");  
        return(LL_STATUS_DISABLE_LATENCY_INACTIVE_CONN);
    }

    // TODO: if latency already disable , return
    pGlobal_config[LL_SWITCH] &=  ~SLAVE_LATENCY_ALLOW;         
    connPtr->slaveLatencyAllowed = FALSE;
    
    if ((AP_TIM1->ControlReg & 0x1) == 0     // timer1 not running
        || llWaitingIrq
        || connPtr->slaveLatency == 0)
    {
        //LOG("==2==%d,%d,%d\n\r",(CP_TIM1->ControlReg & 0x1),llWaitingIrq,connPtr->slaveLatency);
        return (LL_STATUS_DISABLE_LATENCY_DISABLED);
    }
        
    remain_time1 = read_LL_remainder_time();
    if (remain_time1 <=connPtr->lastTimeToNextEvt*625* 3)        // the timer will expiry soon, so not adjust it
    {
        //LOG("==3==\n\r");
        return(LL_STATUS_DISABLE_LATENCY_PENDING);
    }
    
	
    uint16 remain_event;

    remain_event=(remain_time1+connPtr->timerDrift)/(connPtr->lastTimeToNextEvt * 625);

    //20190805 ZQ:
    // considering the event wrap case
    elapse_event= llEventDelta(connPtr->nextEvent,connPtr->currentEvent)-remain_event-(uint8)1;

    //	elapse_time=elapse_event*(connPtr->lastTimeToNextEvt * 625);

    next_event= connPtr->currentEvent + elapse_event + (uint8)2;   // additional 2 connect event for some margin 

    old_timerDrift = connPtr->timerDrift;
		
    llCalcTimerDrift(connPtr->lastTimeToNextEvt, 
                     elapse_event + 1,
                     connPtr->sleepClkAccuracy,
                     (uint32 *)&(connPtr->timerDrift));   

    // timer should expiry early, consider: 1. less timer drift  2. less latency event
    //time_advance = connPtr->lastTimeToNextEvt * (connPtr->nextEvent - next_event) * 625 - (old_timerDrift - connPtr->timerDrift)+550;
    time_advance = connPtr->lastTimeToNextEvt * (llEventDelta(connPtr->nextEvent,next_event) ) * 625 - (old_timerDrift - connPtr->timerDrift)+550;
    
    // apply the timing advance
		
    remain_time2 = AP_TIM1->CurrentCount >> 2;
		
	if(remain_time2<time_advance)
    {
       //LOG("==4==\n\r");
//       pGlobal_config[LL_SWITCH] |=  SLAVE_LATENCY_ALLOW;
//       connPtr->slaveLatencyAllowed = TRUE;
        return (LL_STATUS_DISABLE_LATENCY_MISS_EVT);
    }
		
    set_timer(AP_TIM1,remain_time2 - time_advance);   
//    LOG("currentEvent = %d\r\n", connPtr->currentEvent);
//    LOG("old next_event = %d\r\n", connPtr->nextEvent);
//    LOG("new next_event = %d\r\n", next_event);    
    
    // update connection context
    connPtr->currentEvent += elapse_event;
    //connPtr->currentChan = old_chn;

    //ZQ 20191209 : should use unmap channel
    connPtr->currentChan = connPtr->lastCurrentChan;


    //ZQ 20190805
    // Max latency is 500, preChanMapUdate restore should be trigged within 500 evt
    if(connPtr->preChanMapUpdate.chanMapUpdated==TRUE)
        
    {
        chanMapDltEvt =  llEventDelta( connPtr->preChanMapUpdate.chanMapUpdateEvent, (connPtr->currentEvent + (uint8) 2));

        //20190806 ZQ:
        //only process the revert chanMap when chanMap Delta event with in (0 500)
        if(chanMapDltEvt>0 && chanMapDltEvt<500)
        {

            connPtr->pendingChanUpdate = TRUE;
            osal_memcpy(&(connPtr->chanMap[0]),&(connPtr->preChanMapUpdate.chanMap[0]) ,5);
            llProcessChanMap(connPtr, connPtr->chanMap );   // 16MHz clk, cost 116us!
            #if (DBG_DISABLE_LATENCY)
            LOG("[Hit preChnMap] %d\n\r",connPtr->preChanMapUpdate.chanMapUpdateEvent);
            #endif
        }

    }

//    if (connPtr->channel_selection == LL_CHN_SEL_ALGORITHM_1)
	    connPtr->currentChan = llGetNextDataChan(connPtr,  elapse_event + 2 );
//	else
//	{    // channel selection algorithm 2
//		connPtr->currentChan = llGetNextDataChanCSA2(next_event,
//													(( ( connPtr->accessAddr & 0xFFFF0000 )>> 16 ) ^ ( connPtr->accessAddr  & 0x0000FFFF)),
//													connPtr->chanMap,
//													connPtr->chanMapTable,
//													connPtr->numUsedChans);	
//	}
    
    //connPtr->slaveLatency -= (connPtr->nextEvent - next_event);    
    connPtr->slaveLatency -= llEventDelta(connPtr->nextEvent , next_event);
    connPtr->nextEvent = next_event;
		
//		
//	LOG("==5==\n\r");
//
//    
#if (DBG_DISABLE_LATENCY)
    LOG("new timer = %d\r\n", remain_time2 - time_advance);

    LOG("old Drift = %d\,new Drift = %d \r\n", old_timerDrift,connPtr->timerDrift);
    LOG("cur_event=%d,elapse_event=%d, next_event=%d,\r\n",connPtr->currentEvent, elapse_event,next_event);   
    LOG("RFCHN n=%d o=%d\r\n",connPtr->currentChan,connPtr->lastCurrentChan);

#endif

    return(LL_STATUS_SUCCESS);
}


llStatus_t LL_PLUS_EnableSlaveLatency0(uint8 connId)
{
    llConnState_t *connPtr;
     
    // get connection information
    connPtr = &conn_param[connId];        // only 1 connection now. 

    // check whether it should enable from disable_latency
    if (FALSE == connPtr->active
      || llState != LL_STATE_CONN_SLAVE
      || llSecondaryState != LL_SEC_STATE_IDLE)
    {
        //LOG("==e1==\n\r");
        return(LL_STATUS_DISABLE_LATENCY_INACTIVE_CONN);
    }

//    // check whether it should enable from disable_latency
//    if (   connPtr->pendingChanUpdate == TRUE  
//        || connPtr->pendingParamUpdate == TRUE
//        || connPtr->pendingPhyModeUpdate == TRUE)
//    {
//        LOG("==e2==\n\r");
//        return;
//    }
    pGlobal_config[LL_SWITCH] |=  SLAVE_LATENCY_ALLOW;
    connPtr->slaveLatencyAllowed = TRUE;

    return(LL_STATUS_SUCCESS); 

}

/*******************************************************************************
 * @fn          LL_InitConnectContext
 *
 * @brief       This function initialize the LL connection-orient context
 *
 * input parameters
 *
 * @param       pConnContext   - connection-orient context, the memory is allocated by application
 *              maxConnNum     - the size of connect-orient context
 *              maxPktPerEvent - number of packets transmit/receive per connection event
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
llStatus_t LL_InitConnectContext(llConnState_t    *pConnContext,llScheduleInfo_t *pSchInfoContext,
                                    uint8 *pConnBuffer,
	                                uint8 maxConnNum, 
	                                uint8 maxPktPerEventTx,
	                                uint8 maxPktPerEventRx,
	                                uint8 blePktVersion)
{
    int i, j;
	int pktLen=BLE_PKT40_LEN;
	uint8 *p;

    int total = 0;
	
    if (maxConnNum > MAX_NUM_LL_CONN)
		return LL_STATUS_ERROR_BAD_PARAMETER;

	if ((pConnContext == NULL) || (pSchInfoContext == NULL ) )
		return LL_STATUS_ERROR_BAD_PARAMETER;

	if (pConnBuffer == NULL)
		return LL_STATUS_ERROR_BAD_PARAMETER;	

	if (blePktVersion == BLE_PKT_VERSION_4_0)         // BLE4.0
	    pktLen = BLE_PKT40_LEN;


	pktLen += 6;             // header
	
    g_maxConnNum = maxConnNum;
	conn_param   = pConnContext;
	g_ll_conn_ctx.scheduleInfo = pSchInfoContext;
	g_maxPktPerEventTx = maxPktPerEventTx;
	g_maxPktPerEventRx = maxPktPerEventRx;
//	g_blePktVersion = blePktVersion;

//LOG("conn_param %p\n",conn_param);
    p  = pConnBuffer;
	for (i = 0; i < maxConnNum; i++)
	{
	    osal_memset(&conn_param[i], 0, sizeof(llConnState_t));
	    for (j = 0; j < maxPktPerEventTx; j++)
	    {
	        conn_param[i].ll_buf.tx_conn_desc[j] = (struct ll_pkt_desc *)p;
			p += pktLen;
			total += pktLen;
//			LOG("tx %p\n",conn_param[i].ll_buf.tx_conn_desc[j]);
	    }

	    for (j = 0; j < maxPktPerEventRx; j++)
	    {
	        conn_param[i].ll_buf.rx_conn_desc[j] = (struct ll_pkt_desc *)p;
			p += pktLen;
			total += pktLen;
//			LOG("rx %p\n",conn_param[i].ll_buf.rx_conn_desc[j]);
	    }		

        conn_param[i].ll_buf.tx_not_ack_pkt = (struct ll_pkt_desc *)p;
		p += pktLen;
		total += pktLen;		

	    for (j = 0; j < maxPktPerEventTx; j++)
	    {
	        conn_param[i].ll_buf.tx_ntrm_pkts[j] = (struct ll_pkt_desc *)p;
			p += pktLen;
			total += pktLen;
	    }		
		
	}

//	LOG("total = %d\n", total);
	    
    return( LL_STATUS_SUCCESS );
}


