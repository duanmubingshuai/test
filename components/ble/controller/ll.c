
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

// =============== compile flag, comment out if not required below feature

#define OWN_PUBLIC_ADDR_POS      0x11004000

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

#define LL_COPY_DEV_ADDR_BE( dstPtr, srcPtr )         {                         \
  (dstPtr)[0] = (srcPtr)[5];                                                   \
  (dstPtr)[1] = (srcPtr)[4];                                                   \
  (dstPtr)[2] = (srcPtr)[3];                                                   \
  (dstPtr)[3] = (srcPtr)[2];                                                   \
  (dstPtr)[4] = (srcPtr)[1];                                                   \
  (dstPtr)[5] = (srcPtr)[0];}
// ALT: COULD USE OSAL COPY.
// osal_memcpy( (dstPtr), (srcPtr), LL_DEVICE_ADDR_LEN );

#define BDADDR_VALID( bdAddr )                                                 \
  ( !(                                                                         \
       ((bdAddr)[0] == 0xFF) &&                                                \
       ((bdAddr)[1] == 0xFF) &&                                                \
       ((bdAddr)[2] == 0xFF) &&                                                \
       ((bdAddr)[3] == 0xFF) &&                                                \
       ((bdAddr)[4] == 0xFF) &&                                                \
       ((bdAddr)[5] == 0xFF)                                                   \
     )                                                                         \
  )

// See "Supported States Related" below.
#define LL_SET_SUPPORTED_STATES( state )                                       \
  states[ (state)>>4 ] |= (1<<((state) & 0x0F));

/*******************************************************************************
 * CONSTANTS
 */
// Bluetooth Version Information
#define LL_VERSION_NUM                0x09  	// BT Core Specification V5.0.0, refer to https://www.bluetooth.com/specifications/assigned-numbers/host-controller-interface
#define LL_COMPANY_ID                 0x0504    // Phyplus

// Major Version (8 bits) . Minor Version (4 bits) . SubMinor Version (4 bits)
#define LL_SUBVERSION_NUM             0x0208    //  Controller v1.0.0. Definition not found in BLE spec

// Connection Window Information
#define LL_WINDOW_SIZE                2         // 2.5ms in 1.25ms ticks
#define LL_WINDOW_OFFSET              0         // 1.25ms + 0

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
//
//uint8 g_prdAdvRptEnable = FALSE;

// Delay Sleep
static uint16       sleepDelay;          // delay sleep for XOSC stabilization upon reset

extern uint32 llWaitingIrq;
extern struct buf_rx_desc g_rx_adv_buf;
extern uint32_t llScanDuration;
/*******************************************************************************
 * GLOBAL VARIABLES
 */

advInfo_t           adv_param;       // Advertiser info
// add for scanner
scanInfo_t          scanInfo;        // Scanner info

// BBB new: the memory of LL connect context is allocated by APP
llConnState_t       *conn_param      = NULL;
uint8               g_maxConnNum     = 0;
uint8               g_maxPktPerEventTx = TYP_CONN_BUF_LEN;
uint8               g_maxPktPerEventRx = TYP_CONN_BUF_LEN;
uint8               g_blePktVersion  = BLE_PKT_VERSION_4_0;

uint8               LL_TaskID;          // OSAL LL task ID
uint8_t             llState;            // state of LL                            ==> to move to per connection context ???
peerInfo_t          peerInfo;           // peer device's address and address type   ==> to move to per connection context???
chanMap_t       	chanMapUpdate;      // channel map for updates
featureSet_t   		deviceFeatureSet;   // feature set for this device
verInfo_t           verInfo;            // own version information

uint8              numComplPkts;        // number of completed Tx buffers, use global beacuse we report HCI event when numComplPkts >= numComplPktsLimit
uint8              numComplPktsLimit;   // minimum number of completed Tx buffers before event
rfCounters_t       rfCounters;          // counters for LL RX/TX atomic operation in one connection event

uint8              fastTxRespTime;      // flag indicates if fast TX response time feature is enabled/disabled

// ============== A1 ROM metal change add 
uint32_t       g_llHdcDirAdvTime;      // for HDC direct adv

// RX Flow Control
uint8        rxFifoFlowCtrl;        

llGlobalStatistics_t g_pmCounters;             // TODO: to divide into per connection counters & global counters

// =====   A2 metal change add
llPduLenManagment_t  g_llPduLen;    //for dle feature       ==> to move to per connection context
//llPhyModeManagment_t g_llPhyModeCtrl;// for phy update      ==> to move to per connection context
uint8_t             llSecondaryState;            // secondary state of LL
// ===== A2 add End
extern l2capSARDbugCnt_t g_sarDbgCnt;

extern struct buf_tx_desc g_tx_adv_buf;
extern struct buf_tx_desc tx_scanRsp_desc;

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
void LL_Init0( uint8 taskId )
{
	uint8 *p;
    LL_TaskID = taskId;

    // set BLE version information
    verInfo.verNum    = LL_VERSION_NUM;
    verInfo.comId     = LL_COMPANY_ID;
    verInfo.subverNum = LL_SUBVERSION_NUM;
     
    // =========== calculate whiten seed
	calculate_whiten_seed(); // must set!!!     

    // read flash driectly becasue HW has do the address mapping for read Flash operation
    p = (uint8 *)pGlobal_config[MAC_ADDRESS_LOC];
    ownPublicAddr[3] = *(p++);
    ownPublicAddr[2] = *(p++);
    ownPublicAddr[1] = *(p++);
    ownPublicAddr[0] = *(p++);
    
    ownPublicAddr[5] = *(p++);
    ownPublicAddr[4] = *(p);


    adv_param.ownAddr[0]     = 0xFF;
    adv_param.ownAddr[1]     = 0xFF;
    adv_param.ownAddr[2]     = 0xFF;
    adv_param.ownAddr[3]     = 0xFF;
    adv_param.ownAddr[4]     = 0xFF;
    adv_param.ownAddr[5]     = 0xFF;
    adv_param.ownAddrType    = LL_DEV_ADDR_TYPE_PUBLIC;
    adv_param.advMode        = LL_ADV_MODE_OFF;
    adv_param.advInterval    = LL_ADV_INTERVAL_DEFAULT;
    adv_param.advEvtType     = LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;
    adv_param.advChanMap     = LL_ADV_CHAN_MAP_DEFAULT;            // correct by HZF, 2-23
    adv_param.wlPolicy       = LL_ADV_WL_POLICY_ANY_REQ;
    adv_param.scaValue       = LL_SCA_SLAVE_DEFAULT;
    adv_param.advDataLen     = 0;
    adv_param.scanRspLen     = 0;

	for (int i = 0; i < g_maxConnNum; i++)
    {
        conn_param[i].connId    = i;
		conn_param[i].active    = FALSE;
		conn_param[i].allocConn = FALSE;
		memset((uint8_t *)&conn_param[i].pmCounter , 0, sizeof(llLinkStatistics_t) );
    }


    // set default Scan values
    scanInfo.ownAddr[0]    = 0xFF;
    scanInfo.ownAddr[1]    = 0xFF;
    scanInfo.ownAddr[2]    = 0xFF;
    scanInfo.ownAddr[3]    = 0xFF;
    scanInfo.ownAddr[4]    = 0xFF;
    scanInfo.ownAddr[5]    = 0xFF;
    scanInfo.ownAddrType   = LL_DEV_ADDR_TYPE_PUBLIC;
    scanInfo.initPending   = TRUE;
    scanInfo.scanMode      = LL_SCAN_STOP;
    scanInfo.scanType      = LL_SCAN_PASSIVE;
    scanInfo.scanInterval  = LL_SCAN_INTERVAL_DEFAULT;
    scanInfo.scanWindow    = LL_SCAN_INTERVAL_DEFAULT;
    scanInfo.wlPolicy      = LL_SCAN_WL_POLICY_ANY_ADV_PKTS;
    scanInfo.filterReports = TRUE;
    scanInfo.scanBackoffUL = 1;
    scanInfo.nextScanChan  = LL_SCAN_ADV_CHAN_37;
    scanInfo.numSuccess    = 0;
    scanInfo.numFailure    = 0;
  
    // reset the Link Layer
    (void)LL_Reset();
    
    // generate true random number for AES-CCM
    (void)LL_ENC_GenerateTrueRandNum( cachedTRNGdata, LL_ENC_TRUE_RAND_BUF_SIZE );

    numComplPkts         = 0;
    numComplPktsLimit    = 1;
  
    // add by HZF, init globals
    fastTxRespTime = LL_EXT_DISABLE_FAST_TX_RESP_TIME;   // what is fast TX feature???
    llState = LL_STATE_IDLE;
    
    // add in A2
    llSecondaryState = LL_SEC_STATE_IDLE;
  	
  	// initialize this devices Feature Set
    // has been called in LL_Reset() , so comended by ZQ
    //llInitFeatureSet();
  
    (void)osal_pwrmgr_task_state( LL_TaskID, PWRMGR_CONSERVE );
    
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
 
uint16 LL_ProcessEvent0( uint8 task_id, uint16 events )
{
//    if ( events & LL_EVT_NEXT_INTERVAL )
//    {  
//        ll_debug_output(DEBUG_LL_TIMER_EXPIRY_ENTRY);
//        
//        LL_evt_schedule();
//        
//        ll_debug_output(DEBUG_LL_TIMER_EXPIRY_EXIT);
//		 
//        return (events ^ LL_EVT_NEXT_INTERVAL );
//    }
    
    
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
    
        connPtr = &conn_param[adv_param.connId];        


            LL_ConnectionCompleteCback( LL_STATUS_SUCCESS,                       // reasonCode
                                        (uint16)connPtr->connId,                 // connection handle
                                        LL_LINK_CONNECT_COMPLETE_SLAVE,          // role
                                        peerInfo.peerAddrType,                   // peer's address type
                                        peerInfo.peerAddr,                       // peer's address
                                        connPtr->curParam.connInterval >> 1,     // connection interval, back to 1.25ms units
                                        connPtr->curParam.slaveLatency,              // slave latency
                                        connPtr->curParam.connTimeout >> 4,      // connection timeout, back to 10ms units
                                        connPtr->sleepClkAccuracy );             // sleep clock accurracy

                                              
        return (events ^ LL_EVT_SLAVE_CONN_CREATED );
    }

    /*
    ** Advertising results in a Slave Connection with an Unacceptable
    ** Connection Interval
    */
    if ( events & LL_EVT_SLAVE_CONN_CREATED_BAD_PARAM )
    {
        // notify Host
        LL_ConnectionCompleteCback( LL_STATUS_ERROR_UNACCEPTABLE_CONN_INTERVAL, // reasonCode
                                    (uint16)0,                                  // connection handle
                                    LL_LINK_CONNECT_COMPLETE_SLAVE,             // role
                                    peerInfo.peerAddrType,                      // peer's address type
                                    peerInfo.peerAddr,                          // peer's address
                                    0,                                          // connection interval, back to 1.25ms units
                                    0,                                          // slave latency
                                    0,                                          // connection timeout, back to 10ms units
                                    0 );                                        // sleep clock accurracy
    
        return (events ^ LL_EVT_SLAVE_CONN_CREATED_BAD_PARAM );
    }

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
    /*
    ** Issue Hard System Reset
    */
//    if ( events & LL_EVT_RESET_SYSTEM_HARD )
//    {
//        // Note: This will break communication with USB dongle.
//        // Note: No return here after reset.
//        //SystemReset();
//    
//        // Note: Unreachable statement generates compiler warning!
//        //return (events ^ LL_EVT_RESET_SYSTEM_HARD );
//    }
  
    /*
    ** Issue Soft System Reset
    */
//    if ( events & LL_EVT_RESET_SYSTEM_SOFT )
//    {
//        // Note: This will not break communication with USB dongle.
//        // Note: No return here after reset.
//        //SystemResetSoft();
//      
//        return (events ^ LL_EVT_RESET_SYSTEM_SOFT );
//    }
    
    //====== add in A2, for simultaneous connect & adv/scan
    if (events & LL_EVT_SECONDARY_ADV)
    {
        if (llSecondaryState == LL_SEC_STATE_IDLE
 		 || llSecondaryState == LL_SEC_STATE_IDLE_PENDING)    // adv may be cancel during waiting period, do nothing in this case
			;
		else
		{
            // advertise allow decision
            if (llSecAdvAllow())
            {
                llSetupSecAdvEvt();
            }
//            else
//            {
//                llSecondaryState = LL_SEC_STATE_ADV_PENDING;
//            }
		}
      
        return (events ^ LL_EVT_SECONDARY_ADV );
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
    peerInfo.peerAddrType = LL_DEV_ADDR_TYPE_PUBLIC;
  
    // clear the peer's address
    peerInfo.peerAddr[0] = 0x00;
    peerInfo.peerAddr[1] = 0x00;
    peerInfo.peerAddr[2] = 0x00;
    peerInfo.peerAddr[3] = 0x00;
    peerInfo.peerAddr[4] = 0x00;
    peerInfo.peerAddr[5] = 0x00;

    // init the Adv parameters to their default values
    adv_param.advMode = LL_ADV_MODE_OFF;
  
    // clear the advertising interval
    adv_param.advInterval = LL_ADV_INTERVAL_DEFAULT;//  Ti set this parameter = 0;
  
    // Note that only the first three bits are used, and note that despite the
    // fact that the Host HCI call passes 5 bytes to specify which advertising
    // channels are to be used, only the first three bits of the first byte are
    // actually used. We'll save four bytes then, and default to all used.
    adv_param.advChanMap = LL_ADV_CHAN_ALL;
  
    // set a default type of advertising
    adv_param.advEvtType = LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;
  
    // payload length, including AdvA
    adv_param.advDataLen = 0;

    // default to no scan response data
    adv_param.scanRspLen = 0;
    
    // ===================== for scan/init parameters 
    // disable scanning
    scanInfo.scanMode = LL_SCAN_STOP;

    // initialize this devices Feature Set
    llInitFeatureSet();

    // initialize default channel map
    chanMapUpdate.chanMap[0] = 0xFF;
    chanMapUpdate.chanMap[1] = 0xFF;
    chanMapUpdate.chanMap[2] = 0xFF;
    chanMapUpdate.chanMap[3] = 0xFF;
    chanMapUpdate.chanMap[4] = 0x1F;

    // set state/role
    llState = LL_STATE_IDLE;
//    ll_debug_output(DEBUG_LL_STATE_IDLE);
    
    // add in A2
    llSecondaryState = LL_SEC_STATE_IDLE;    

    numComplPkts         = 0;
    numComplPktsLimit    = 1;

    fastTxRespTime = LL_EXT_DISABLE_FAST_TX_RESP_TIME;      // TI default enable it, hzf

    rxFifoFlowCtrl   = LL_RX_FLOW_CONTROL_DISABLED;

    llPduLengthManagmentReset();
    llPhyModeCtrlReset();

	for (int i = 0; i < g_maxConnNum; i ++)
    {
        conn_param[i].llPhyModeCtrl.def.txPhy = LE_1M_PHY | LE_2M_PHY;		  
        conn_param[i].llPhyModeCtrl.def.rxPhy = LE_1M_PHY | LE_2M_PHY;
        conn_param[i].llPhyModeCtrl.def.allPhy=0;

        llResetConnId(i);
	    reset_conn_buf(i);
    }
    // exit critical section
    HAL_EXIT_CRITICAL_SECTION();
    
    // reset all statistics counters
    osal_memset(&g_pmCounters, 0, sizeof(g_pmCounters));    
  
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
  	
    // check if the reason code is valid
    if ( (reason != LL_DISCONNECT_AUTH_FAILURE)                &&
         (reason != LL_DISCONNECT_REMOTE_USER_TERM)            &&
         (reason != LL_DISCONNECT_REMOTE_DEV_LOW_RESOURCES)    &&
         (reason != LL_DISCONNECT_REMOTE_DEV_POWER_OFF)        &&
         (reason != LL_DISCONNECT_UNSUPPORTED_REMOTE_FEATURE)  &&
         (reason != LL_DISCONNECT_KEY_PAIRING_NOT_SUPPORTED)   &&
         (reason != LL_DISCONNECT_UNACCEPTABLE_CONN_INTERVAL) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }
  		
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
    if ( (pBuf == NULL) || (pktLen > connPtr->llPduLen.local.MaxTxOctets/*LL_MAX_LINK_DATA_LEN*/) ||
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
    {
        // either we are not the master or the slave, or the we are but the connId
        // is not the current connection, so just return success as the data is
        // queued on the connection
        return( LL_STATUS_SUCCESS );
    }

    // copy any pending data to the TX FIFO
    llProcessTxData( connPtr, LL_TX_DATA_CONTEXT_SEND_DATA );

    // check if TX FIFO has anything in it
    if (getTxBufferSize(connPtr) > 0 )
    {
        // check if we are a master or slave (i.e. in a connection), and the current
        // current connection is connId; check if fast TX is enabled; check if
        // slave latency is in use
       if ( (fastTxRespTime == LL_EXT_ENABLE_FAST_TX_RESP_TIME) && connPtr->slaveLatency )
       {     
    }  // else delta correction active so queue packet

    // M/S, curConn==connID, fast Tx enabled, and SL in use
    } // TX FIFO empty
  

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
	uint8    g_currentLocalAddrType = 0 ;
  
  // sanity check of parameters
  if ( ( (advEvtType != LL_ADV_CONNECTABLE_UNDIRECTED_EVT)     &&
         (advEvtType != LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)   &&
         (advEvtType != LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)  &&
         (advEvtType != LL_ADV_SCANNABLE_UNDIRECTED_EVT)       &&
         (advEvtType != LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT) )      ||

       ( ((advEvtType == LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)  ||
          (advEvtType == LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT)) &&
         ((peerAddr == NULL)                                 ||
          ((peerAddrType != LL_DEV_ADDR_TYPE_PUBLIC)         &&
           (peerAddrType != LL_DEV_ADDR_TYPE_RANDOM))) )          ||

       ( ((advEvtType == LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT) ||
          (advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT))     &&
          // the minimum interval for nonconnectable Adv is 100ms
          ((advIntervalMin < LL_ADV_CONN_INTERVAL_MIN)      ||    // should use LL_ADV_NONCONN_INTERVAL_MIN after update it to 20ms
         (advIntervalMin > LL_ADV_NONCONN_INTERVAL_MAX)     ||
         (advIntervalMax < LL_ADV_CONN_INTERVAL_MIN)        ||    // should use LL_ADV_NONCONN_INTERVAL_MIN after update it to 20ms
         (advIntervalMax > LL_ADV_NONCONN_INTERVAL_MAX)) )        ||

       ( (advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT)     &&
          // the minimum interval for connectable undirected Adv is 20ms
          ((advIntervalMin < LL_ADV_CONN_INTERVAL_MIN)         ||
           (advIntervalMin > LL_ADV_CONN_INTERVAL_MAX)         ||
           (advIntervalMax < LL_ADV_CONN_INTERVAL_MIN)         ||
           (advIntervalMax > LL_ADV_CONN_INTERVAL_MAX)) )      ||
           ( advIntervalMax < advIntervalMin )                 ||
        ( (ownAddrType != LL_DEV_ADDR_TYPE_PUBLIC)              &&
         (ownAddrType != LL_DEV_ADDR_TYPE_RANDOM)              &&
         (ownAddrType != LL_DEV_ADDR_TYPE_RPA_PUBLIC)          &&            // BLE 4.2
         (ownAddrType != LL_DEV_ADDR_TYPE_RPA_RANDOM))         ||            // BLE 4.2
         ( ((ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC)   ||
           (ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))   &&
         (peerAddr == NULL))                                   ||
        ( (advWlPolicy != LL_ADV_WL_POLICY_ANY_REQ)        &&
         (advWlPolicy != LL_ADV_WL_POLICY_WL_SCAN_REQ)     &&
         (advWlPolicy != LL_ADV_WL_POLICY_WL_CONNECT_REQ)  &&
         (advWlPolicy != LL_ADV_WL_POLICY_WL_ALL_REQ) )         ||

        ( ((advChanMap & LL_ADV_CHAN_ALL) == 0) ) )
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }

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
  // save the white list policy
  adv_param.wlPolicy = advWlPolicy;

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
  // TODO
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
    case LL_ADV_SCANNABLE_UNDIRECTED_EVT:
             pduType = ADV_SCAN_IND;
             break;
    default:
             // should not come here, sanity check in the function start. set default value to suppress warning
             pduType = ADV_IND;
             break;
  }	
  SET_BITS(g_tx_adv_buf.txheader, pduType, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
  SET_BITS(g_tx_adv_buf.txheader, (g_currentLocalAddrType & 0x01), TX_ADD_SHIFT, TX_ADD_MASK);
//  SET_BITS(g_tx_adv_buf.txheader, peerInfo.peerAddrType, RX_ADD_SHIFT, RX_ADD_MASK);   // RxAdd need't set
  if ((advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT
  	 || advEvtType == LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT
  	 || advEvtType == LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT)
  	 && pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
      SET_BITS(g_tx_adv_buf.txheader, 1, CHSEL_SHIFT, CHSEL_MASK);
  
  osal_memcpy( g_tx_adv_buf.data,  adv_param.ownAddr, 6);

	
  SET_BITS(tx_scanRsp_desc.txheader, ADV_SCAN_RSP, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
  SET_BITS(tx_scanRsp_desc.txheader, g_currentLocalAddrType, TX_ADD_SHIFT, TX_ADD_MASK);  
  osal_memcpy( tx_scanRsp_desc.data, adv_param.ownAddr, 6);	
//  SET_BITS(tx_scanRsp_desc.txheader, 0, TX_ADD_SHIFT, TX_ADD_MASK);

   // adv length should be set for not direct adv type, 2018-04-05
   SET_BITS(g_tx_adv_buf.txheader, (adv_param.advDataLen + 6), LENGTH_SHIFT, LENGTH_MASK);

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
//    _HAL_CS_ALLOC_(); 
	
    //if random address isn't defined,can't set ownaddresstype to random
    if ((advMode)&&(((adv_param.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM)    ||
                     (adv_param.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))  &&
                    (  (ownRandomAddr[0] == 0xFF) &&
                       (ownRandomAddr[1] == 0xFF) &&
                       (ownRandomAddr[2] == 0xFF) &&
                       (ownRandomAddr[3] == 0xFF) &&
                       (ownRandomAddr[4] == 0xFF) &&
                       (ownRandomAddr[5] == 0xFF)  )))
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // check if a direct test mode or modem test is in progress
    if ( (llState == LL_STATE_DIRECT_TEST_MODE_TX) ||
         (llState == LL_STATE_DIRECT_TEST_MODE_RX) ||
         (llState == LL_STATE_MODEM_TEST_TX)       ||
         (llState == LL_STATE_MODEM_TEST_RX)       ||
         (llState == LL_STATE_MODEM_TEST_TX_FREQ_HOPPING) )
    {
        return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
    }
  
    // 2021-4-19, check init/scan state should not enable/disable adv
    if ( (llState == LL_STATE_SCAN) ||
            (llState == LL_STATE_INIT) )
    {
        return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
    }

    // sanity checks again to be sure we don't start with bad parameters
    if ( ( (adv_param.advEvtType != LL_ADV_CONNECTABLE_UNDIRECTED_EVT)     &&
           (adv_param.advEvtType != LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)   &&
           (adv_param.advEvtType != LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)  &&
           (adv_param.advEvtType != LL_ADV_SCANNABLE_UNDIRECTED_EVT)       &&
           (adv_param.advEvtType != LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT) )        ||
         ( (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_PUBLIC)              &&
           (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_RANDOM)              &&
           (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_RPA_PUBLIC)          &&
           (adv_param.ownAddrType != LL_DEV_ADDR_TYPE_RPA_RANDOM))                ||
         ( ((adv_param.advEvtType == LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)        ||
            (adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT))     &&
           (adv_param.advInterval < LL_ADV_CONN_INTERVAL_MIN) ) )     // should use LL_ADV_NONCONN_INTERVAL_MIN after update it to 20ms
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

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
//                        ll_debug_output(DEBUG_LL_STATE_ADV_UNDIRECTED);
		    	 
                        break;
                    case LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT:
                    case LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT:
                        llState=LL_STATE_ADV_DIRECTED;		
//                        ll_debug_output(DEBUG_LL_STATE_ADV_DIRECTED);        
                        break;
		    	 
                    case LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT:
                        llState=LL_STATE_ADV_NONCONN;	
//                        ll_debug_output(DEBUG_LL_STATE_ADV_NONCONN);        
                        break;
		    	 
                    case LL_ADV_SCANNABLE_UNDIRECTED_EVT:
                        llState=LL_STATE_ADV_SCAN;
//                        ll_debug_output(DEBUG_LL_STATE_ADV_SCAN);	 
                        break;
		    	 
                    default:
                        llState=LL_STATE_IDLE;	
//                        ll_debug_output(DEBUG_LL_STATE_IDLE);        
                        break;
                }
            }
                        
            // llState changed when configure adv parameters
            if (llState == LL_STATE_ADV_UNDIRECTED 
			 || llState == LL_STATE_ADV_DIRECTED 
			 || llState == LL_STATE_ADV_NONCONN 			 
			 || llState == LL_STATE_ADV_SCAN )     // TODO: check this setting
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
            else if((llState == LL_STATE_CONN_SLAVE 
                 || llState == LL_STATE_CONN_MASTER)
                 && (pGlobal_config[LL_SWITCH] & SIMUL_CONN_ADV_ALLOW))
            {
                if (llSecondaryState != LL_SEC_STATE_IDLE)
					return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );

				// adv event check
				if (adv_param.advEvtType  != LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT
				  && adv_param.advEvtType != LL_ADV_SCANNABLE_UNDIRECTED_EVT
				  && adv_param.advEvtType != LL_ADV_CONNECTABLE_UNDIRECTED_EVT)
				    return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );

			    // Note: we may need maximum slave number check here. If number of slave reach ceil,
			    //       only no-connectable adv is allowed. The checking could be don't in host
                
                llSecondaryState = LL_SEC_STATE_ADV;
				adv_param.advNextChan = LL_ADV_CHAN_LAST + 1;        // set adv channel invalid
                osal_stop_timerEx( LL_TaskID, LL_EVT_SECONDARY_ADV );
                osal_set_event(LL_TaskID, LL_EVT_SECONDARY_ADV);     // set adv event
            }
            else           // other state
                return (LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE);
            
            // indicate advertising is no longer active
            adv_param.advMode = LL_ADV_MODE_ON;


            break;
    
        case LL_ADV_MODE_OFF:
        	{
            _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();
            
            // free the associated task block
            //llFreeTask( &advInfo.llTask );
            
            // indicate we are no longer actively advertising
            adv_param.advMode = LL_ADV_MODE_OFF;
            
    	    if (llState != LL_STATE_CONN_SLAVE && 
    	    	llState != LL_STATE_CONN_MASTER)      // no conn + adv case
    	    {
                llState = LL_STATE_IDLE;     // if not in connect state, set idle to disable advertise

                //ZQ 20190912
                //stop ll timer when idle, considering the scan-adv interleve case
                clear_timer(AP_TIM1);
                
//                ll_debug_output(DEBUG_LL_STATE_IDLE);
            
    	    }
			else                       // conn + adv case
			{
			// TODO
//                uint8 i;
//
//                i = 0;
//                while (!(adv_param.advChanMap & (1 << i)))   i ++;    // get the 1st adv channel in the adv channel map                
//                
//			    if ((llSecondaryState == LL_SEC_STATE_ADV)
//                    && (adv_param.advNextChan != (LL_ADV_CHAN_FIRST + i)))      // last adv event is not finished
//			        llSecondaryState = LL_SEC_STATE_IDLE_PENDING;
//				else
                {
                    llSecondaryState = LL_SEC_STATE_IDLE;
                    osal_stop_timerEx( LL_TaskID, LL_EVT_SECONDARY_ADV );    // stop timer
                }
			}
            
            HAL_EXIT_CRITICAL_SECTION();
        	}
	
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
 * This API is called by the HCI to read the peer controller's Version
 * Information. If the peer's Version Information has already been received by
 * its request for our Version Information, then this data is already cached and
 * can be directly returned to the Host. If the peer's Version Information is
 * not already cached, then it will be requested from the peer, and when
 * received, returned to the Host via the LL_ReadRemoteVersionInfoCback
 * callback.
 *
 * Note: Only one Version Indication is allowed for a connection.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_ReadRemoteVersionInfo( uint16 connId )
{
  llStatus_t    status;
  llConnState_t *connPtr;

  // make sure connection ID is valid
  if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
  {
    return( status );
  }

  // get connection info
  connPtr = &conn_param[connId];       

  // first, make sure the connection is still active
  if ( !connPtr->active )
  {
    return( LL_STATUS_ERROR_INACTIVE_CONNECTION );
  }

  // check if the peer's version information has already been obtained
  if ( connPtr->verExchange.peerInfoValid == TRUE )
  {
    // yes it has, so provide it to the host
    LL_ReadRemoteVersionInfoCback( LL_STATUS_SUCCESS,
                                   connId,
                                   connPtr->verInfo.verNum,
                                   connPtr->verInfo.comId,
                                   connPtr->verInfo.subverNum );
  }
  else // no it hasn't, so...
  {
    // ...check if the host has already requested this information
    if ( connPtr->verExchange.hostRequest == FALSE )
    {	
      // no, so request it by queueing the control packet for processing
      llEnqueueCtrlPkt( connPtr, LL_CTRL_VERSION_IND );

      // set the flag to indicate the host has requested this information
      connPtr->verExchange.hostRequest = TRUE;
    }
    else // previously requested
    {
      return( LL_STATUS_ERROR_VER_INFO_REQ_ALREADY_PENDING );
    }
  }

  return( LL_STATUS_SUCCESS );
}

llStatus_t LL_PhyUpdate0( uint16 connId )
{
    llStatus_t    status;
    llConnState_t *connPtr;
    uint8 phyMode;


    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // get connection info
    connPtr = &conn_param[connId ];

    // check if an update control procedure is already pending
    if ( ((connPtr->ctrlPktInfo.ctrlPktCount > 0) &&
        (connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_PHY_UPDATE_IND)) ||
      (connPtr->pendingPhyModeUpdate == TRUE) )
    {
        return( LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE );
    }

    // we only support symmetric connection
    // tx rx phy should be same
    if(connPtr->llPhyModeCtrl.req.allPhy==0)
    {
        phyMode = connPtr->llPhyModeCtrl.req.txPhy & connPtr->llPhyModeCtrl.rsp.txPhy;
    
        phyMode|= connPtr->llPhyModeCtrl.req.rxPhy & connPtr->llPhyModeCtrl.rsp.rxPhy;
    }
    else if(connPtr->llPhyModeCtrl.req.allPhy==1)
    {
        phyMode = connPtr->llPhyModeCtrl.req.rxPhy & connPtr->llPhyModeCtrl.rsp.rxPhy;
    }
    else if(connPtr->llPhyModeCtrl.req.allPhy==2)
    {
        phyMode = connPtr->llPhyModeCtrl.req.txPhy & connPtr->llPhyModeCtrl.rsp.txPhy;
    }
    else
    {
        phyMode=0;
    }


    
    if(phyMode==0)
    {
        //no change case
        connPtr->phyUpdateInfo.m2sPhy = 0;
        connPtr->phyUpdateInfo.s2mPhy = 0;
    }
    else if(phyMode&LE_2M_PHY)
    {
        connPtr->phyUpdateInfo.m2sPhy = LE_2M_PHY;
        connPtr->phyUpdateInfo.s2mPhy = LE_2M_PHY;
    }
    else if(phyMode&LE_CODED_PHY)
    {
        connPtr->phyUpdateInfo.m2sPhy = LE_CODED_PHY;
        connPtr->phyUpdateInfo.s2mPhy = LE_CODED_PHY;
    }
    else
    {
        //no perferce can not support the tx/rx same time
        connPtr->phyUpdateInfo.m2sPhy = LE_1M_PHY;
        connPtr->phyUpdateInfo.s2mPhy = LE_1M_PHY;
    }

    if(phyMode==0)
    {
        connPtr->phyModeUpdateEvent = 0;

        connPtr->phyUpdateInfo.instant =   connPtr->phyModeUpdateEvent;       
    }
    else
    {
        connPtr->phyModeUpdateEvent = (connPtr->curParam.slaveLatency+1) +
                                              LL_INSTANT_NUMBER_MIN;

        connPtr->phyUpdateInfo.instant =   connPtr->phyModeUpdateEvent;       

    }
    // queue control packet for processing
    llEnqueueCtrlPkt( connPtr, LL_CTRL_PHY_UPDATE_IND );

    return( LL_STATUS_SUCCESS );

}


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
llStatus_t LL_ReadRemoteUsedFeatures0( uint16 connId )
{
  llStatus_t    status;
  llConnState_t *connPtr;

//  // make sure we're in Master role            // in BLE4.2, it could be send in both master & slave
//  if ( llState != LL_STATE_CONN_MASTER )
//  {
//    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
//  }

  // make sure connection ID is valid
  if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
  {
    return( status );
  }

  // get connection info
  connPtr = &conn_param[connId ];

  // initiate a Feature Set control procedure
  llEnqueueCtrlPkt( connPtr, LL_CTRL_FEATURE_REQ );

  return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * @fn          LL_NumEmptyWlEntries API
 *
 * @brief       This API is called by the HCI to get the number of White List
 *              entries that are empty.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       *numEmptyEntries - number of empty entries in the White List.
 *
 * @return      LL_STATUS_SUCCESS
 */
//extern llStatus_t LL_NumEmptyWlEntries( uint8 *numEmptyEntries );  // Not used by TI code


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
llStatus_t LL_Encrypt0( uint8 *key,
                       uint8 *plaintextData,
                       uint8 *encryptedData )
{
  // check parameters
  if ( (key == NULL )          ||
       (plaintextData == NULL) ||
       (encryptedData == NULL) )
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }

  // encrypt on behalf of the host
  LL_ENC_AES128_Encrypt( key, plaintextData, encryptedData );

  return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
 * This API is called by the HCI to request the LL Controller to provide a data
 * block with random content.
 *
 * Note: we use different scheme to TI. Below is from TI notes:
 *       The HCI spec indicates that the random number
 *       generation should adhere to one of those specified in FIPS
 *       PUB 140-2. The Core spec refers specifically to the
 *       algorithm specified in FIPS PUB 186-2, Appendix 3.1.
 *       Note that this software only uses the RF hardware to
 *       generate true random numbers. What's more, if the RF is
 *       already in use (i.e. overlapped execution), then the use
 *       of radio to generate true random numbers is prohibited.
 *       In this case, a pseudo-random blocks of numbers will be
 *       returned instead.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_Rand( uint8 *randData,
                    uint8 dataLen )
{
    uint16 temp_rand;
    uint8 *pData = randData;
	  uint32 sysTick;

    // check if a DTM or Modem operation is in progress
    if ( (llState == LL_STATE_DIRECT_TEST_MODE_TX) ||
       (llState == LL_STATE_DIRECT_TEST_MODE_RX) ||
       (llState == LL_STATE_MODEM_TEST_TX)       ||
       (llState == LL_STATE_MODEM_TEST_RX)       ||
       (llState == LL_STATE_MODEM_TEST_TX_FREQ_HOPPING) )
    {
      // yes, so sorry, no true random number generation allowed as the radio
      // is in continuous use
      return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
    }

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
 * This API is a generic interface to get a block of pseudo-random numbers.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_PseudoRand( uint8 *randData,
                          uint8 dataLen )
{ 
 /* uint8 i;

  if ( (randData == NULL) || (dataLen == 0) )
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }

  // can't be sure the radio isn't in use, so provide pseudo random numbers
  for (i=0; i<dataLen; i++)
  {
    randData[i] = LL_ENC_GeneratePseudoRandNum();
  }
*/
  return( LL_STATUS_SUCCESS );
}



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
// the defination of LL_SET_SUPPORTED_STATES implicly using input parameter "states", not good, but it is TI's
llStatus_t LL_ReadSupportedStates( uint8 *states )
{
  // provide supported state/role combinations
  // Note: Upper nibble is byte offset, lower nibble is bit offset.

  LL_SET_SUPPORTED_STATES( LL_SCAN_PASSIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_SCAN_ACTIVE_STATE );

  LL_SET_SUPPORTED_STATES( LL_ADV_NONCONN_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_DISCOV_STATE );

  LL_SET_SUPPORTED_STATES( LL_ADV_NONCONN_SCAN_PASSIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_DISCOV_SCAN_PASSIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_NONCONN_SCAN_ACTIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_DISCOV_SCAN_ACTIVE_STATE );

  LL_SET_SUPPORTED_STATES( LL_INIT_STATE );
  LL_SET_SUPPORTED_STATES( LL_INIT_MASTER_STATE );

  LL_SET_SUPPORTED_STATES( LL_SCAN_PASSIVE_INIT_STATE );
  LL_SET_SUPPORTED_STATES( LL_SCAN_ACTIVE_INIT_STATE );
  LL_SET_SUPPORTED_STATES( LL_SCAN_PASSIVE_MASTER_STATE );
  LL_SET_SUPPORTED_STATES( LL_SCAN_ACTIVE_MASTER_STATE );

  LL_SET_SUPPORTED_STATES( LL_ADV_NONCONN_INIT_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_DISCOV_INIT_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_NONCONN_MASTER_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_DISCOV_MASTER_STATE );

  LL_SET_SUPPORTED_STATES( LL_ADV_UNDIRECT_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_HDC_DIRECT_STATE );
  LL_SET_SUPPORTED_STATES( LL_SLAVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_LDC_DIRECT_STATE );

  LL_SET_SUPPORTED_STATES( LL_ADV_UNDIRECT_SCAN_PASSIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_HDC_DIRECT_SCAN_PASSIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_UNDIRECT_SCAN_ACTIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_HDC_DIRECT_SCAN_ACTIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_SCAN_PASSIVE_SLAVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_SCAN_ACTIVE_SLAVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_LDC_DIRECT_SCAN_PASSIVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_LDC_DIRECT_SCAN_ACTIVE_STATE );

  LL_SET_SUPPORTED_STATES( LL_ADV_NONCONN_SLAVE_STATE );
  LL_SET_SUPPORTED_STATES( LL_ADV_DISCOV_SLAVE_STATE );

  return( LL_STATUS_SUCCESS );

}

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
llStatus_t LL_ReadLocalVersionInfo( uint8  *verNum,
                                           uint16 *comId,
                                           uint16 *subverNum )
{
  // get the version of this BLE controller
  *verNum = verInfo.verNum;

  // get the company ID of this BLE controller
  *comId = verInfo.comId;

  // get the subversion of this BLE controller
  *subverNum = verInfo.subverNum;

  return( LL_STATUS_SUCCESS );
}


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
llStatus_t LL_CtrlToHostFlowControl( uint8 mode )
{
  if ( mode == LL_ENABLE_RX_FLOW_CONTROL )
  {
    // set flag to indicate flow control is enabled
    rxFifoFlowCtrl = LL_RX_FLOW_CONTROL_ENABLED;
  }
  else if ( mode == LL_DISABLE_RX_FLOW_CONTROL )
  {
    // set flag to indicate flow control is disabled
    rxFifoFlowCtrl = LL_RX_FLOW_CONTROL_DISABLED;
  }
  else // error
  {
    return( LL_STATUS_ERROR_UNEXPECTED_PARAMETER );
  }

  return( LL_STATUS_SUCCESS );
}

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
llStatus_t LL_ReadTxPowerLevel0( uint8 connId,
                                uint8 type,
                                int8  *txPower )
{
    uint8_t txPowerIdx=0;
    int8 txPowerMaping[18]={
                            RF_PHY_TX_POWER_EXTRA_MAX,  10,
                            RF_PHY_TX_POWER_MAX,         7,
                            RF_PHY_TX_POWER_5DBM,        5,
                            RF_PHY_TX_POWER_0DBM,        0,
                            RF_PHY_TX_POWER_N5DBM,      -5,
                            RF_PHY_TX_POWER_N10DBM,     -10,
                            RF_PHY_TX_POWER_N15DBM,     -15,
                            RF_PHY_TX_POWER_N20DBM,     -20,
                            RF_PHY_TX_POWER_MIN,        -40,//[lastIdx-1] should be zero to end mapping search
    
    };
    
    if ( txPower == NULL )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    // determine which type of TX power level is required
    switch( type )
    {
        case LL_READ_CURRENT_TX_POWER_LEVEL:
        
           // search the mapping table
           while(   txPowerMaping[txPowerIdx]>0
               &&   txPowerMaping[txPowerIdx]>g_rfPhyTxPower)
           {
               txPowerIdx=txPowerIdx+2;
           }
          
           // return the TX power level based on current setting
           *txPower =txPowerMaping[txPowerIdx+1];        // assume when g_rfPhyTxPower = 0x1f, tx power = 10dBm, 
                    
//      // check if Tx output power is valid
//      if ( *txPower == LL_TX_POWER_INVALID )
//      {
//        return( LL_STATUS_ERROR_PARAM_OUT_OF_RANGE );
//      }
        break;

        case LL_READ_MAX_TX_POWER_LEVEL:
            // return max data channel TX power level
            *txPower = RF_PHY_TX_POWER_MAX;
        break;

        default:
            return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

    return( LL_STATUS_SUCCESS );
}	

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
llStatus_t LL_SetTxPowerLevel0( int8  txPower )
{
    // TODO: add tx power range check
    // TODO: add tx power mapping    
    g_rfPhyTxPower = txPower;
    rf_phy_set_txPower(g_rfPhyTxPower);    
    
    return LL_STATUS_SUCCESS;
}	

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
llStatus_t LL_ReadChanMap( uint8 connId,
                                  uint8 *chanMap )
{
  llStatus_t    status;
  llConnState_t *connPtr;

  // make sure connection ID is valid
  if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
  {
    return( status );
  }

  // get connection info
  connPtr = &conn_param[ connId ];

  // copy current channel map
  chanMap[0] = connPtr->chanMap[0];
  chanMap[1] = connPtr->chanMap[1];
  chanMap[2] = connPtr->chanMap[2];
  chanMap[3] = connPtr->chanMap[3];
  chanMap[4] = connPtr->chanMap[4];

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
	
  // sanity check of parameters
  if ( ( (scanType != LL_SCAN_PASSIVE)    &&
         (scanType != LL_SCAN_ACTIVE) )                         ||
       ( (ownAddrType != LL_DEV_ADDR_TYPE_PUBLIC)      &&
         (ownAddrType != LL_DEV_ADDR_TYPE_RANDOM)      &&
         (ownAddrType != LL_DEV_ADDR_TYPE_RPA_PUBLIC)  &&
         (ownAddrType != LL_DEV_ADDR_TYPE_RPA_RANDOM) )             ||
       ( (scanInterval < LL_SCAN_WINDOW_MIN)                    ||
         (scanInterval > LL_SCAN_WINDOW_MAX) )                  ||
       ( (scanWindow < LL_SCAN_WINDOW_MIN)                      ||
         (scanWindow > LL_SCAN_WINDOW_MAX) )                    ||
       ( (scanWindow > scanInterval) )                          ||
       ( (scanWlPolicy != LL_SCAN_WL_POLICY_ANY_ADV_PKTS)   &&
         (scanWlPolicy != LL_SCAN_WL_POLICY_USE_WHITE_LIST) ) )
  {
    return( LL_STATUS_ERROR_BAD_PARAMETER );
  }

  // check if scan is active
  if ( scanInfo.scanMode == LL_SCAN_START )
  {
    // yes, so not allowed per the spec
    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
  }

  // set the scan type
  scanInfo.scanType = scanType;

  // save white list policy
  scanInfo.wlPolicy = scanWlPolicy;

  scanInfo.ownAddrType = ownAddrType;

	// TODO
  // set the scanner's address based on the HCI's address type preference
  if ( ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC )
  {
    // get our address and address type
    LL_COPY_DEV_ADDR_LE( scanInfo.ownAddr, ownPublicAddr );
  }   
  else
  {    // for RPAs, scan control not indicate using which RPA list entry, no copy scanA here
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
//    _HAL_CS_ALLOC_(); 
	
    // check if a direct test mode or modem test is in progress
    if ( (llState == LL_STATE_DIRECT_TEST_MODE_TX) ||
       (llState == LL_STATE_DIRECT_TEST_MODE_RX) ||
       (llState == LL_STATE_MODEM_TEST_TX)       ||
       (llState == LL_STATE_MODEM_TEST_RX)       ||
       (llState == LL_STATE_MODEM_TEST_TX_FREQ_HOPPING) )
    {
        return( LL_STATUS_ERROR_UNEXPECTED_STATE_ROLE );
    }

    // sanity checks again to be sure we don't start with bad parameters
    if ( ( (scanInfo.scanType != LL_SCAN_PASSIVE)   &&
         (scanInfo.scanType != LL_SCAN_ACTIVE))              ||
       ( (scanInfo.ownAddrType != LL_DEV_ADDR_TYPE_PUBLIC) &&
         (scanInfo.ownAddrType != LL_DEV_ADDR_TYPE_RANDOM))  ||
       ( (scanInfo.scanInterval < LL_SCAN_WINDOW_MIN)        ||
         (scanInfo.scanInterval > LL_SCAN_WINDOW_MAX))       ||
       ( (scanInfo.scanWindow < LL_SCAN_WINDOW_MIN)          ||
         (scanInfo.scanWindow > LL_SCAN_WINDOW_MAX))         ||
       ( (scanInfo.scanWindow > scanInfo.scanInterval) )     ||
       ( (filterReports != LL_FILTER_REPORTS_DISABLE)  &&
         (filterReports != LL_FILTER_REPORTS_ENABLE)) )
    {
        return( LL_STATUS_ERROR_BAD_PARAMETER );
    }

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

            // get a task block for this BLE state/role
            // Note: There will always be a valid pointer, so no NULL check required.
//          scanInfo.llTask = llAllocTask( LL_TASK_ID_SCANNER );

            if (((scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM)    ||
                    (scanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))  &&
                    (  (ownRandomAddr[0] == 0xFF) &&
                      (ownRandomAddr[1] == 0xFF) &&
                      (ownRandomAddr[2] == 0xFF) &&
                      (ownRandomAddr[3] == 0xFF) &&
                      (ownRandomAddr[4] == 0xFF) &&
                      (ownRandomAddr[5] == 0xFF)  ))
            {
                return( LL_STATUS_ERROR_BAD_PARAMETER );
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
            else if ((llState == LL_STATE_CONN_SLAVE 
                  || llState == LL_STATE_CONN_MASTER)     // HZF: if we should support adv + scan, add more state here
                  && (pGlobal_config[LL_SWITCH] & SIMUL_CONN_SCAN_ALLOW))
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
        	{
			_HAL_CS_ALLOC_();

            HAL_ENTER_CRITICAL_SECTION();

    	    if (llState == LL_STATE_SCAN)      // no conn + scan case
    	    {
                llState = LL_STATE_IDLE;     // if not in connect state, set idle to disable scan

                //ZQ 20190912
                //stop ll timer when idle, considering the scan-adv interleve case
                clear_timer(AP_TIM1);
                
//                ll_debug_output(DEBUG_LL_STATE_IDLE);
    	    }
			else if (llState == LL_STATE_CONN_SLAVE
				|| llState == LL_STATE_CONN_MASTER)                      // conn + scan case
			{
                llSecondaryState = LL_SEC_STATE_IDLE;
			}	

            // indicate we are no longer actively scanning
            scanInfo.scanMode = LL_SCAN_STOP;

            // A2 multiconn, should we consider current LL state to avoid change master/slave configuration
            // now LL slave/master event use same parameter 88
			ll_hw_set_rx_timeout(88); 
      
            // HZF: should we stop scan task immediately, or wait scan IRQ then stop? Now use option 2.
            HAL_EXIT_CRITICAL_SECTION();

        	}
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
llStatus_t LL_EncLtkNegReply( uint16 connId )
{
  llStatus_t    status;
  llConnState_t *connPtr;

  // make sure we're in Master role
  if ( llState != LL_STATE_CONN_SLAVE )
  {
    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
  }

  // make sure connection ID is valid
  if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
  {
    return( status );
  }

  // get connection info
  connPtr = &conn_param[ connId ];

  // check if this is during a start or a re-start encryption procedure
  if ( connPtr->encInfo.encRestart == TRUE )
  {
    // indicate the peer requested this termination
    connPtr->termInfo.reason = LL_ENC_KEY_REQ_REJECTED;

    // queue control packet for processing
    // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
    //      THE QUEUE.
    //llReplaceCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
    llEnqueueCtrlPkt( connPtr, LL_CTRL_TERMINATE_IND );
  }
  else // during a start encryption
  {
    // set the encryption rejection error code
    connPtr->encInfo.encRejectErrCode = LL_STATUS_ERROR_PIN_OR_KEY_MISSING; // same as LL_ENC_KEY_REQ_REJECTED

    // and reject the encryption request
    // ALT: COULD MAKE THIS A REPLACE IF A DUMMY IS SITTING AT THE HEAD OF
    //      THE QUEUE.
    //llReplaceCtrlPkt( connPtr, LL_CTRL_REJECT_IND );
    llEnqueueCtrlPkt( connPtr, LL_CTRL_REJECT_IND );
  }

  return( LL_STATUS_SUCCESS );

}

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
 * This API is called by the HCI to read the controller's Feature Set. The
 * Controller indicates which features it supports.
 *
 * Public function defined in ll.h.
 */
llStatus_t LL_ReadLocalSupportedFeatures( uint8 *featureSet )
{
  uint8 i;

  // copy Feature Set
  for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
  {
    featureSet[i] = deviceFeatureSet.featureSet[i];
  }

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
//    // add for BQB test 2018-9-20, LL/SEC/ADV/BV-01-C
//  // check if advertising is active
//  if ( adv_param.advMode == LL_ADV_MODE_ON )
//  {
//    // yes, so not allowed per the spec
//    return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
//  }

    if ( (llState == LL_STATE_ADV_UNDIRECTED) ||
         (llState == LL_STATE_ADV_DIRECTED)   ||
         (llState == LL_STATE_ADV_SCAN)       ||
         (llState == LL_STATE_ADV_NONCONN)    ||
         (llState == LL_STATE_SCAN)           ||
         (llState == LL_STATE_INIT)           ||
         (llSecondaryState == LL_SEC_STATE_ADV)          ||
         (llSecondaryState == LL_SEC_STATE_ADV_PENDING)  ||
         (llSecondaryState == LL_SEC_STATE_SCAN)         ||
         (llSecondaryState == LL_SEC_STATE_SCAN_PENDING) ||         
         (llSecondaryState == LL_SEC_STATE_INIT)         ||
         (llSecondaryState == LL_SEC_STATE_INIT_PENDING))
    {
        return( LL_STATUS_ERROR_COMMAND_DISALLOWED );
    }	  
  
    // store our random address LSO..MSO
    ownRandomAddr[0] = devAddr[0];
    ownRandomAddr[1] = devAddr[1];  
    ownRandomAddr[2] = devAddr[2];
    ownRandomAddr[3] = devAddr[3];
    ownRandomAddr[4] = devAddr[4];
    ownRandomAddr[5] = devAddr[5];

    return( LL_STATUS_SUCCESS );
}


llStatus_t LL_WriteSuggestedDefaultDataLength(uint16 TxOctets,uint16 TxTime)
{
    if(TxOctets >   LL_PDU_LENGTH_SUPPORTED_MAX_TX_OCTECTS
    || TxTime   >   LL_PDU_LENGTH_SUPPORTED_MAX_TX_TIME
    || TxOctets <   LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS
    || TxTime   <   LL_PDU_LENGTH_INITIAL_MAX_TX_TIME)
    {
        return(LL_STATUS_ERROR_PARAM_OUT_OF_RANGE);
    }
    else
    {
        g_llPduLen.suggested.MaxTxOctets= TxOctets;
        g_llPduLen.suggested.MaxRxTime  = TxTime;
        return(LL_STATUS_SUCCESS);
    }
}

llStatus_t LL_SetDataLengh0( uint16 connId,uint16 TxOctets,uint16 TxTime )
{
    uint8         i;
    llStatus_t    status;
    llConnState_t *connPtr;

    if(TxOctets >   LL_PDU_LENGTH_SUPPORTED_MAX_TX_OCTECTS
    || TxTime   >   LL_PDU_LENGTH_SUPPORTED_MAX_TX_TIME
    || TxOctets <   LL_PDU_LENGTH_INITIAL_MAX_TX_OCTECTS
    || TxTime   <   LL_PDU_LENGTH_INITIAL_MAX_TX_TIME)
    {
        return(LL_STATUS_ERROR_PARAM_OUT_OF_RANGE);
    }

  
    // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // get connection info
    connPtr = &conn_param[connId]; 

    // check if a feature response control procedure has taken place
    if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
    {
        // it hasn't so re-load this device's local Feature Set to the
        // connection as it may have been changed by the Host with HCI
        // extenstion Set Local Feature Set command
        for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
        {
          connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
        }
    }

    // check if dle is a supported feature set item
    if ( (connPtr->featureSetInfo.featureSet[0] & LL_FEATURE_DATA_LENGTH_EXTENSION) != LL_FEATURE_DATA_LENGTH_EXTENSION )
    {
        return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
    }

    // check if an updated parameters control procedure is already what's pending
    if ( ((connPtr->ctrlPktInfo.ctrlPktCount > 0) &&
        (connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_LENGTH_REQ)) ||
        (connPtr->llPduLen.isProcessingReq == TRUE) || (connPtr->llPduLen.isWatingRsp == TRUE) )
    {
        return( LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE );
    }

   g_llPduLen.suggested.MaxTxOctets = TxOctets;          // remove by HZF, suggested value is from host, should not change in controller
   g_llPduLen.suggested.MaxTxTime   = TxTime;

    
    // setup an LL_LENGTH_REQ
    llEnqueueCtrlPkt( connPtr, LL_CTRL_LENGTH_REQ );
    
    return(LL_STATUS_SUCCESS);
}


llStatus_t LL_SetDefaultPhyMode( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy)
{

    conn_param[connId].llPhyModeCtrl.def.allPhy = allPhy;
    conn_param[connId].llPhyModeCtrl.def.txPhy  = txPhy;
    conn_param[connId].llPhyModeCtrl.def.rxPhy  = rxPhy;


    return( LL_STATUS_SUCCESS );
}


llStatus_t LL_SetPhyMode0( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions)
{
    uint8         i;
    llStatus_t    status;
    llConnState_t *connPtr;

     // make sure connection ID is valid
    if ( (status=LL_ConnActive(connId)) != LL_STATUS_SUCCESS )
    {
        return( status );
    }

    // get connection info
    connPtr = &conn_param[connId]; 

    // check if a feature response control procedure has taken place
    if ( connPtr->featureSetInfo.featureRspRcved == FALSE )
    {
        // it hasn't so re-load this device's local Feature Set to the
        // connection as it may have been changed by the Host with HCI
        // extenstion Set Local Feature Set command
        for (i=0; i<LL_MAX_FEATURE_SET_SIZE; i++)
        {
          connPtr->featureSetInfo.featureSet[i] = deviceFeatureSet.featureSet[i];
        }
    }

    // check if dle is a supported feature set item
    if(     ( (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_2M_PHY) != LL_FEATURE_2M_PHY )
        &&  ( (connPtr->featureSetInfo.featureSet[1] & LL_FEATURE_CODED_PHY) != LL_FEATURE_CODED_PHY ) )
    {
        return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
    }


    // check if an updated parameters control procedure is already what's pending
    if ( ((connPtr->ctrlPktInfo.ctrlPktCount > 0) &&
        (connPtr->ctrlPktInfo.ctrlPkts[0] == LL_CTRL_PHY_REQ)) ||
        (connPtr->pendingPhyModeUpdate== TRUE) || 
        (connPtr->llPhyModeCtrl.isWatingRsp == TRUE) || (connPtr->llPhyModeCtrl.isProcessingReq == TRUE) )
    {
        return( LL_STATUS_ERROR_CTRL_PROC_ALREADY_ACTIVE );
    }

    //support Symmetric Only
    if(allPhy==0 &&(txPhy!=rxPhy))
    {
        return( LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED );
    }
    

    // how to check the required param?
    //LL_TS_5.0.3 Table 4.43: PDU payload contents for each case variation for LE 2M PHY
    connPtr->llPhyModeCtrl.req.allPhy = allPhy;

    if(connPtr->llPhyModeCtrl.req.allPhy==0)
    {
        connPtr->llPhyModeCtrl.req.txPhy  = txPhy;
        connPtr->llPhyModeCtrl.req.rxPhy  = txPhy;
    }
    else if(connPtr->llPhyModeCtrl.req.allPhy==1)
    {
        connPtr->llPhyModeCtrl.req.txPhy  = 0;
        connPtr->llPhyModeCtrl.req.rxPhy  = txPhy;

    }
    else if(connPtr->llPhyModeCtrl.req.allPhy==2)
    {
        connPtr->llPhyModeCtrl.req.txPhy  = txPhy;
        connPtr->llPhyModeCtrl.req.rxPhy  = 0;

    }
    else
    {
        //no prefer on both phy
        connPtr->llPhyModeCtrl.req.txPhy  = 0;
        connPtr->llPhyModeCtrl.req.rxPhy  = 0;
    }
    connPtr->llPhyModeCtrl.phyOptions = phyOptions;
    
   
    // setup an LL_LENGTH_REQ
    llEnqueueCtrlPkt( connPtr, LL_CTRL_PHY_REQ );
    
    return(LL_STATUS_SUCCESS);
}


/*******************************************************************************
 * @fn          LL_ReadMaximumAdvDataLength
 *
 * @brief       This function is used to read the maximum adv set data length support by controller
 *
 * input parameters
 *
 * @param       adv_handle - advertisement set handler
 *              
 *
 * output parameters
 *
 * @param       length  -  pointer to the variable of maximum data length support
 *
 * @return      LL_STATUS_SUCCESS, error code(TBD)
 */
//llStatus_t LL_ReadMaximumAdvDataLength( uint16 *length )
//{
//    *length = LL_MAX_ADVERTISER_SET_LENGTH;            // TBD. 
//    return( LL_STATUS_SUCCESS );
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

	    connPtr->currentChan = llGetNextDataChan(connPtr,  elapse_event + 2 );
    //connPtr->slaveLatency -= (connPtr->nextEvent - next_event);    
    connPtr->slaveLatency -= llEventDelta(connPtr->nextEvent , next_event);
    connPtr->nextEvent = next_event;
		
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
llStatus_t LL_InitConnectContext(llConnState_t    *pConnContext,
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

	if (pConnContext == NULL)
		return LL_STATUS_ERROR_BAD_PARAMETER;

	if (pConnBuffer == NULL)
		return LL_STATUS_ERROR_BAD_PARAMETER;	

	if (blePktVersion == BLE_PKT_VERSION_4_0)         // BLE4.0
	    pktLen = BLE_PKT40_LEN;
	else if (blePktVersion == BLE_PKT_VERSION_5_1)         // BLE5.1
		pktLen = BLE_PKT51_LEN;

	pktLen += 6;             // header
	
    g_maxConnNum = maxConnNum;
	conn_param   = pConnContext;
	g_maxPktPerEventTx = maxPktPerEventTx;
	g_maxPktPerEventRx = maxPktPerEventRx;
	g_blePktVersion = blePktVersion;

    p  = pConnBuffer;
	for (i = 0; i < maxConnNum; i++)
	{
	    memset(&conn_param[i], 0, sizeof(llConnState_t));
	    for (j = 0; j < maxPktPerEventTx; j++)
	    {
	        conn_param[i].ll_buf.tx_conn_desc[j] = (struct ll_pkt_desc *)p;
			p += pktLen;
			total += pktLen;
	    }

	    for (j = 0; j < maxPktPerEventRx; j++)
	    {
	        conn_param[i].ll_buf.rx_conn_desc[j] = (struct ll_pkt_desc *)p;
			p += pktLen;
			total += pktLen;
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


