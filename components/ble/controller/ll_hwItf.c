
// this file is temporary, most new chip code will be wrote in this file
// after stable, the functions should be move to other LL files
/*******************************************************************************
  Filename:       ll_hwItf.c
  Revised:        
  Revision:       

  Description:    Interface functions to LL HW.

  Copyright 2017-2020 Phyplus  Incorporated. All rights reserved.

  IMPORTANT: 
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ll_hw_drv.h"

#include "timer.h"
#include "ll_buf.h"
#include "ll_def.h"
#include "ll.h"
#include "ll_common.h"
#include "hci_event.h"
#include "osal_bufmgr.h"
#include "bus_dev.h"
#include "ll_enc.h"
#include "rf_phy_driver.h"
#include "jump_function.h"
#include "global_config.h"
#include "ll_debug.h"
#include "log.h"

// =============== compile flag, comment out if not required below feature
//#define  PRD_ADV_ENABLE
//#define  EXT_SCAN_ENABLE

// ==============

/*******************************************************************************
 * MACROS
 */

// add by HZF, merge with ll_hw_drv.h later
#define LL_HW_MODE_STX           0x00
#define LL_HW_MODE_SRX           0x01
#define LL_HW_MODE_TRX           0x02
#define LL_HW_MODE_RTX           0x03
#define LL_HW_MODE_TRLP          0x04
#define LL_HW_MODE_RTLP          0x05

// =========== A1 ROM metal change add
#define MAX_HDC_DIRECT_ADV_TIME          1280000              // 1.28s in unit us

#define LL_CALC_NEXT_SCAN_CHN(chan)     { chan ++; \
    chan = (chan > LL_SCAN_ADV_CHAN_39) ? LL_SCAN_ADV_CHAN_37 : chan;}

/*******************************************************************************
 * CONSTANTS
 */
// Master Sleep Clock Accurracy, in PPM
// Note: Worst case range value is assumed.
extern const uint16 SCA[] ;//= {500, 250, 150, 100, 75, 50, 30, 20};
extern uint8 ownPublicAddr[];     // index 0..5 is LSO..MSB
//extern uint8 ownRandomAddr[];
/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

///*******************************************************************************
// * GLOBAL VARIABLES
// */

uint32_t g_t_llhwgo = 0;
uint32 llWaitingIrq = FALSE;
uint32 ISR_entry_time = 0;
uint32_t g_TIM1_IRQ_timing = 250;
int32 slave_conn_event_recv_delay = 0;
// for HCI ext command: enable/disable notify for connection/adv event
uint8    g_adv_taskID = 0;
uint16   g_adv_taskEvent = 0;
uint8    g_conn_taskID = 0;
uint16   g_conn_taskEvent = 0;
uint8    g_dle_taskID = 0;
uint16   g_dle_taskEvent = 0;
uint8    g_phyChg_taskID = 0;
uint16   g_phyChg_taskEvent = 0;


// =========== A2 metal change add, to move to llConnState_t(per connection parameters)
uint32_t  g_smartWindowLater          = 0;
uint32_t  g_smartWindowSize           = 0;
uint32_t  g_smartWindowSizeNew        = 0;           // to omit
bool      g_smartWindowActive         = 0;
uint32_t  g_smartWindowActiveCnt      = 0;
uint32_t  g_smartWindowPreAnchPoint   = 0;
uint8_t   g_smartWindowRTOCnt         = 0;


volatile uint32  g_getPn23_cnt  = 0;
volatile uint32  g_getPn23_seed = 0x12345678;

// =======  A2 multi-connection ========================
struct buf_tx_desc g_tx_adv_buf;
struct buf_tx_desc tx_scanRsp_desc;
struct buf_rx_desc g_rx_adv_buf;

//20180523 by ZQ
//path for tx_rx_offset issue in scan_rsq
volatile uint8_t g_same_rf_channel_flag     = FALSE;            //path for tx_rx_offset config in scan_rsp

// for scan
uint32_t llScanT1;
uint32_t llScanTime = 0;
uint32_t llCurrentScanChn;

uint32_t llScanDuration = 0;

//===================== external
//extern ctrl_packet_buf   ctrlData;

extern uint32            osal_sys_tick;
extern uint32            hclk_per_us,   hclk_per_us_shift;
// RX Flow Control
extern uint8             rxFifoFlowCtrl;

// ============== A1 ROM metal change add 
extern uint32_t g_llHdcDirAdvTime;      // for HDC direct adv

// =====   A2 metal change add
extern uint8_t             llSecondaryState;            // secondary state of LL
perStatsByChan_t*          p_perStatsByChan = NULL;

/*******************************************************************************
 * Functions
 */
uint32_t ll_hw_get_loop_time(void);

int ll_hw_get_rfifo_depth(void);
uint16_t ll_hw_get_tfifo_wrptr(void);

// A2 metal change add
uint32_t getPN23RandNumber(void);
void ll_adptive_smart_window(uint32_t irq_status,uint32_t anchor_point);
//////////////  For Master
//void move_to_master_function(void);

//void LL_master_conn_event(void);

//void ll_hw_read_tfifo_trlp(void);

///////////// For avtive scan
static void llAdjBoffUpperLimitSuccess( void );

static void llAdjBoffUpperLimitFailure( void );

static void llGenerateNextBackoffCount( void );


LL_PLUS_AdvDataFilterCB_t LL_PLUS_AdvDataFilterCBack=NULL;
LL_PLUS_ScanRequestFilterCB_t LL_PLUS_ScanRequestFilterCBack=NULL;

void LL_PLUS_PerStats_Init(perStatsByChan_t* p_per);
void LL_PLUS_PerStatsReset(void);

void LL_PLUS_PerStasReadByChn(uint8 chnId,perStats_t * perStats);

void ll_hw_tx2rx_timing_config(uint8 pkt);
void ll_hw_trx_settle_config(uint8 pkt);
// externel
extern uint32_t get_timer_count(AP_TIM_TypeDef *TIMx);

// local function
uint32  read_ll_adv_remainder_time(void);

// add by HZF, to move to ll_hw_drv.c
/**************************************************************************************
 * @fn          ll_hw_get_tr_mode
 *
 * @brief       This function get the current LL HW engine TR mode.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      mode  -  STX(0), SRX(1), TRX(2), RTX(3), TRLP(4), RTLP(5).
 */
uint8 ll_hw_get_tr_mode(void)
{
    uint8 mode;
	
	mode = (*(volatile uint32_t *)(LL_HW_BASE+ 0x04)) & 0x0f;	//[3:0]	RW	4'b0	Mode
    
    return mode;
}

/*******************************************************************************
 * @fn          LL_IRQHandler
 *
 * @brief      Interrupt Request Handler for Link Layer
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None
 */
 uint8 ll_processBasicIRQ(uint32_t      irq_status);
void LL_IRQHandler(void)
{
    uint32         irq_status;  
	//int8 ret;    

    ISR_entry_time = read_current_fine_time();    

//    ll_debug_output(DEBUG_ISR_ENTRY);
    
    irq_status = ll_hw_get_irq_status();
    
    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        return;
    }
    llWaitingIrq = FALSE;

    ll_processBasicIRQ(irq_status);

	// ================ Post ISR process: secondary pending state process
    // conn-adv case 2: other ISR, there is pending secondary advertise event, make it happen
    if (llSecondaryState == LL_SEC_STATE_ADV_PENDING)
   	{
	    if (llSecAdvAllow())    // for multi-connection case, it is possible still no enough time for adv
	    {
			llSetupSecAdvEvt();
		 
//			llSecondaryState = LL_SEC_STATE_ADV;
	    }
   	}
	// there is pending scan event, make it happen, note that it may stay pending if there is no enough idle time
	else if (llSecondaryState == LL_SEC_STATE_SCAN_PENDING)
   	{      // trigger scan
	    llSetupSecScan(scanInfo.nextScanChan);
   	}
//    ll_debug_output(DEBUG_ISR_EXIT);
}

/*
 this function is implemented per interval,  it include multi loops of M->S and S->M within one interval

*/
// to check whether we need it later
void LL_set_default_conn_params0(llConnState_t *connPtr)
{
	connPtr->sn_nesn = 0;
    connPtr->rx_timeout   = 0;
    
//	connPtr->connected = 0;
    connPtr->firstPacket  = 1;
    
    connPtr->txDataEnabled = TRUE;    
    connPtr->rxDataEnabled = TRUE;      // bug fixed 218-04-08
    
    connPtr->lastTimeToNextEvt  = 0;
    connPtr->lastSlaveLatency   = 0;
    
    connPtr->encEnabled  = FALSE;
    
    connPtr->lastRssi = 0;          // A1 ROM metal change add
		
	connPtr->expirationEvent = LL_LINK_SETUP_TIMEOUT;		    
}

/*******************************************************************************
 * @fn          move_to_slave_function0
 *
 * @brief       This function is used to process CONN_REQ and move the llState to slave 
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
 * @return      None,
 *             
 */
void move_to_slave_function0(void)
{
    llConnState_t *connPtr;
    uint8_t       *pBuf;
    uint8_t       tempByte;
	uint8_t       chnSel;
    volatile uint32_t      calibra_time, T2;	
//LOG("%s\n",__func__);
//	hal_gpio_write(GPIO_P15, 1);

	if ( (connPtr = llAllocConnId()) == NULL )
    {
		return;
    }
	adv_param.connId = connPtr->connId;       

	pBuf = g_rx_adv_buf.data;	     
    
    // reset connection parameters
	LL_set_default_conn_params(connPtr);
    
    // clear the connection buffer
    reset_conn_buf(connPtr->connId);    
    
    adv_param.advMode = LL_ADV_MODE_OFF;
	
    pBuf += 12;      // skip initA and AdvA
    pBuf = llMemCopySrc( (uint8 *)&connPtr->accessAddr,             pBuf, 4 );
    pBuf = llMemCopySrc( (uint8 *)&connPtr->initCRC,                pBuf, 3 );
    pBuf = llMemCopySrc( (uint8 *)&connPtr->curParam.winSize,       pBuf, 1 );
    pBuf = llMemCopySrc( (uint8 *)&connPtr->curParam.winOffset,     pBuf, 2 );
    pBuf = llMemCopySrc( (uint8 *)&connPtr->curParam.connInterval,  pBuf, 2 );
    pBuf = llMemCopySrc( (uint8 *)&connPtr->curParam.slaveLatency,  pBuf, 2 );
    pBuf = llMemCopySrc( (uint8 *)&connPtr->curParam.connTimeout,   pBuf, 2 );

    // TI style: convert to 625us tick
	connPtr->curParam.winSize      <<= 1;
	connPtr->curParam.winOffset    <<= 1;
	connPtr->curParam.connInterval <<= 1;          
	connPtr->curParam.connTimeout  <<= 4;

    llConvertLstoToEvent( connPtr, &(connPtr->curParam) );     // 16MHz CLK, need 56.5us
    
    // bug fixed 2018-4-4, calculate control procedure timeout value when connection setup
    // convert the Control Procedure timeout into connection event count
    llConvertCtrlProcTimeoutToEvent(connPtr);

	if (((connPtr->curParam.connTimeout <= ((connPtr->curParam.slaveLatency ) * connPtr->curParam.connInterval << 1))) 
		    || (connPtr->curParam.connInterval == 0) )
    {
        // schedule LL Event to notify the Host a connection was formed with
        // a bad parameter
        // Note: This event doesn't take parameters, so it is assumed there that
        //       the reason code was due to an unacceptable connection interval.
        (void)osal_set_event( LL_TaskID, LL_EVT_SLAVE_CONN_CREATED_BAD_PARAM );

        return;
    }

    pBuf = llMemCopySrc( (uint8 *)connPtr->chanMap,   pBuf, 5 );
    pBuf = llMemCopySrc( &tempByte,   pBuf, 1 );
	
	connPtr->hop  = tempByte & 0x1F;

	connPtr->sleepClkAccuracy  = (tempByte >> 5) & 0x07;
 	
	// calculate channel for data
	llProcessChanMap(connPtr, connPtr->chanMap );   // 16MHz clk, cost 116us!
			
	connPtr->slaveLatency = 0;        //correct 05-09, no latency before connect
	connPtr->slaveLatencyValue = connPtr->curParam.slaveLatency;
    connPtr->accuTimerDrift = 0;
    llAdjSlaveLatencyValue(connPtr);

	// combine slave SCA with master's SCA and calculate timer drift factor
	//connPtr->scaFactor = llCalcScaFactor( connPtr->sleepClkAccuracy );
	//connPtr->currentChan = llGetNextDataChan(1);
		
	llState = LL_STATE_CONN_SLAVE;
//    ll_debug_output(DEBUG_LL_STATE_CONN_SLAVE);
	
	connPtr->active = TRUE;
    connPtr->sn_nesn = 0;                  // 1st rtlp, init sn/nesn as 0
    connPtr->llMode = LL_HW_RTLP_1ST;     // set as RTLP_1ST for the 1st connection event

    // calculate the 1st channel
    connPtr->currentChan = 0;	
//hal_gpio_write(GPIO_P15, 1);
//	BM_SET(reg_gpio_ioe_porta, BIT(GPIO_P15));
//    BM_SET(reg_gpio_swporta_dr, BIT(GPIO_P15));

	    connPtr->currentChan = llGetNextDataChan(connPtr, 1);
	
//    hal_gpio_write(GPIO_P15, 0);
//	BM_SET(reg_gpio_ioe_porta, BIT(GPIO_P15));
//    BM_CLR(reg_gpio_swporta_dr, BIT(GPIO_P15));

    // calculate timer drift
    llCalcTimerDrift(connPtr->curParam.winOffset + 2,        // 1250us + win offset, in 625us tick
                           connPtr->slaveLatency,
                           connPtr->sleepClkAccuracy,
                           (uint32 *)&(connPtr->timerDrift));    
        
    T2 = read_current_fine_time();
    // calculate the SW delay from ISR to here
    calibra_time = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
    // other delay: conn req tail -> ISR: 32us, timing advance: 50us, HW engine startup: 60us
    // start slave event SW process time: 50us
    // soft parameter: pGlobal_config[CONN_REQ_TO_SLAVE_DELAY]
    calibra_time += pGlobal_config[CONN_REQ_TO_SLAVE_DELAY];     //(32 + 50 + 60 + 50 + pGlobal_config[CONN_REQ_TO_SLAVE_DELAY]);     
    
    ll_schedule_next_event(1250 + connPtr->curParam.winOffset * 625 - calibra_time - connPtr->timerDrift);
    
    (void)osal_set_event( LL_TaskID, LL_EVT_SLAVE_CONN_CREATED);	
//		    LOG("calibra_time %d\n",calibra_time);
    g_pmCounters.ll_conn_succ_cnt ++;     // move to anchor point catch ?

//    hal_gpio_write(GPIO_P15, 0);	

}


/*******************************************************************************
 * @fn          LL_slave_conn_event
 *
 * @brief       This function process slave connection event
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
 * @return      None,
 *             
 */
 // TODO 
void LL_slave_conn_event0(void)            // TODO: update connection context select
{
    uint16_t ll_rdCntIni;
    uint32_t      tx_num, rx_num;
    llConnState_t *connPtr;

//	g_ll_conn_ctx.timerExpiryTick = read_current_fine_time();                     // A2 multiconnection	
	
//	hal_gpio_write(GPIO_P14, 1);

	connPtr = &conn_param[0];

    // time critical process, disable interrupt
    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    
    tx_num = pGlobal_config[LL_TX_PKTS_PER_CONN_EVT];
    rx_num = pGlobal_config[LL_RX_PKTS_PER_CONN_EVT];
    
    if (tx_num > g_maxPktPerEventTx || tx_num == 0)    tx_num = g_maxPktPerEventTx;
    if (rx_num > g_maxPktPerEventRx || rx_num == 0)    rx_num = g_maxPktPerEventRx;
    
    connPtr->pmCounter.ll_conn_event_cnt ++;
    if(p_perStatsByChan!=NULL)
        p_perStatsByChan->connEvtCnt[connPtr->currentChan]++;

    //ZQ 20191209
    //restore the currentChan for disable slavelatency 
    //ZQ20200207 should use nextChan
    connPtr->lastCurrentChan = connPtr->nextChan;
    // counter for one connection event
    llResetRfCounters();

    //support rf phy change
    rf_phy_change_cfg(connPtr->llRfPhyPktFmt);
    ll_hw_tx2rx_timing_config(connPtr->llRfPhyPktFmt);
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();      

    // channel physical configuration   
    set_crc_seed(connPtr->initCRC );            // crc seed for data PDU is from CONNECT_REQ
    
    set_access_address(connPtr->accessAddr);     // access address
    
    set_channel(connPtr->currentChan );        // set channel    
    
    set_whiten_seed(connPtr->currentChan);     // set whiten seed

    // A2-multiconn
	ll_hw_set_rx_timeout(88);

	set_max_length(0xff);                  // add 2020-03-10
    // win size for 1st packet
    if (connPtr->firstPacket)     // not received the 1st packet, CRC error or correct
    {
        //ll_hw_set_rx_timeout_1st(conn_param[connId].curParam.winSize * 625 + conn_param[connId].timerDrift * 2 );  

        
        //20180412 enlarge the connectInd or connect_update timing tolerence
//        uint32_t first_window_timout=pGlobal_config[LL_SMART_WINDOW_FIRST_WINDOW] + connPtr->curParam.winSize * 625 + connPtr->timerDrift * 2 ;
        uint32_t first_window_timout=pGlobal_config[LL_FIRST_WINDOW] + connPtr->curParam.winSize * 625 + connPtr->timerDrift * 2 ;


        
        //The transmitWindowOffset shall be a multiple of 1.25 ms in the range of 0 ms
        //to connInterval. The transmitWindowSize shall be a multiple of 1.25 ms in the
        //range of 1.25 ms to the lesser of 10 ms and (connInterval - 1.25 ms). 
        //ZQ 20200208 
        uint32_t winSizeLimt = MIN(10000 , (connPtr->curParam.connInterval * 625 - 1250) );

        if(winSizeLimt<first_window_timout)
            first_window_timout = winSizeLimt;
    
        ll_hw_set_rx_timeout_1st(first_window_timout);  
    }
    else                  // A1 ROM metal change , 2018 - 1 - 3
    {
//        if (connPtr->rx_timeout)    // timeout case
//            ll_hw_set_rx_timeout_1st(pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] + pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] * 2 + (connPtr->timerDrift + connPtr->accuTimerDrift) * 2 ); 
//        else
//            ll_hw_set_rx_timeout_1st(pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] + pGlobal_config[SLAVE_CONN_DELAY] * 2 + connPtr->timerDrift * 2 ); 

ll_hw_set_rx_timeout_1st(pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] + connPtr->accuTimerDrift*2 ); 

    }
    // configure loop timeout
    // considering the dle case
    uint32_t temp = connPtr->curParam.connInterval * 625 - connPtr->llPduLen.local.MaxRxTime- pGlobal_config[LL_HW_RTLP_TO_GAP];       // 500us: margin for timer1 IRQ
    ll_hw_set_loop_timeout(temp > pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] ?
                    pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] : temp);         // 2018-6-20, global config for the parameter
                                              // now we support 4 RT in one RTLP, if PDU size is 256Byte, need (256*8 + 150) * 8 = 17684us, 
                                               // not consider Rx packet size


    ll_hw_trx_settle_config(connPtr->llRfPhyPktFmt);

    // retransmit count limit
    ll_hw_set_loop_nack_num( 4 );
    //set the rfifo ign control
	ll_hw_ign_rfifo(LL_HW_IGN_ALL);	
    
    // write packets to Tx FIFO
    tx_num = ll_generateTxBuffer(tx_num, &ll_rdCntIni);  

    // TODO: consider Rx flow control here
//    if (LL_RX_FLOW_CONTROL_ENABLED == rxFifoFlowCtrl)
//    {
//        // configure LL HW to keep NESN 
//    }
                       
    ll_hw_config( LL_HW_RTLP,  //connPtr->llMode, 
                  connPtr->sn_nesn,   // sn,nesn init
                  tx_num,                    // ll_txNum      
                  rx_num,                    // ll_rxNum
                  1,                         // ll_mdRx
                  ll_rdCntIni);              // rdCntIni        

	uint8 temp_rf_fmt = g_rfPhyPktFmt;
	g_rfPhyPktFmt = connPtr->llRfPhyPktFmt;
				  
    // start LL HW engine
    ll_hw_go();
    llWaitingIrq = TRUE;

	g_rfPhyPktFmt = temp_rf_fmt;
    
    HAL_EXIT_CRITICAL_SECTION();
//    ll_debug_output(DEBUG_LL_HW_SET_RTLP);
}  

/*******************************************************************************
 * @fn          LL_evt_schedule0
 *
 * @brief       Link layer event process entry
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
 * @return      None,
 *             
 */
void LL_evt_schedule0(void)
{  
//LOG("%s'n",__func__);
    if (llWaitingIrq == TRUE)
    {            // normally should not be here
        //LOG("event collision detect\n");
        g_pmCounters.ll_evt_shc_err++;
    }
    switch(llState)
    {
        // adv envent entry
        case LL_STATE_ADV_UNDIRECTED:              
        case LL_STATE_ADV_DIRECTED:              
        case LL_STATE_ADV_NONCONN:              
        case LL_STATE_ADV_SCAN:    	
  	        llSetupAdv();
            break;
        				
        // slave connect event entry
        case LL_STATE_CONN_SLAVE:    	// TO update
        // TODO
          //  set_default_conn_data();
//          if(g_ll_conn_ctx.currentConn != LL_INVALID_CONNECTION_ID && conn_param [g_ll_conn_ctx.currentConn].active)
          {
            LL_slave_conn_event();
          }
//          else
//          {    // add by HZF on 2017-12-12, if connection is not active, release 
//              llReleaseConnId(&conn_param[g_ll_conn_ctx.currentConn]); 
//          }
          break;
          
        case LL_STATE_SCAN:               
            llScanTime = 0;
            llSetupScan(scanInfo.nextScanChan);        
          break;    
         
        case LL_STATE_IDLE:    	  
        default:    	
//            ll_schedule_next_event(20000);     // add by HZF, period trigger LL task when IDLE and other not process ll state
                                               // it may useful when some application change the ll state but not trigger ll loop
          break;  
    }  
}  

/*******************************************************************************
 * @fn          llSetupAdv0
 *
 * @brief       This routine is used to setup the Controller for Advertising
 *              based on the Advertising event type.
 *
 * input parameters
 *
 * @param       None.
 *
 * output== parameters
 *
 * @param       None.
 *
 * @return      LL_STATUS_SUCCESS
 */
llStatus_t llSetupAdv0( void )
{	
    if ((llState == LL_STATE_ADV_DIRECTED && (adv_param.advEvtType != LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)  && (adv_param.advEvtType != LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT)) 
    	|| (llState == LL_STATE_ADV_UNDIRECTED && (adv_param.advEvtType !=  LL_ADV_CONNECTABLE_UNDIRECTED_EVT) )
    	|| (llState == LL_STATE_ADV_NONCONN && (adv_param.advEvtType !=  LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT) )
    	|| (llState == LL_STATE_ADV_SCAN && (adv_param.advEvtType != LL_ADV_SCANNABLE_UNDIRECTED_EVT) ))
    {
        // sanity check failure
    }

    g_rfPhyPktFmt = LE_1M_PHY;          
  	//support rf phy change
    rf_phy_change_cfg(g_rfPhyPktFmt);
    ll_hw_ign_rfifo(LL_HW_IGN_CRC|LL_HW_IGN_EMP);	

	ll_hw_set_rx_timeout(188); 
    
    // reset all FIFOs; all data is forfeit
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();
    
    // setup and start the advertising task based on the Adv event
    switch( adv_param.advEvtType )
    {
      // can only get a CONNECT_REQ
      case LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT:
      case LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT:
        // setup a Directed Advertising Event
        llSetupDirectedAdvEvt();
  
      break;
  
      // can get a SCAN_REQ or a CONNECT_REQ
      case LL_ADV_CONNECTABLE_UNDIRECTED_EVT:
  
        // setup a Undirected Advertising Event
        llSetupUndirectedAdvEvt();
      break;
  
      // neither a SCAN_REQ nor a CONNECT_REQ can be received
      case LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT:
  
        // setup a Undirected Advertising Event
        llSetupNonConnectableAdvEvt();
      break;
  
      // can only get a SCAN_REQ
      case LL_ADV_SCANNABLE_UNDIRECTED_EVT:
        // setup a Discoverable Undirected Advertising Event
        llSetupScannableAdvEvt();
      break;
      
      default:
        // Note: this should not ever happen as the params are checked by
        //       LL_SetAdvParam()
        return( LL_STATUS_ERROR_UNKNOWN_ADV_EVT_TYPE );
    }
    
    // notify upper layer if required
    if (g_adv_taskID != 0)
    {
        uint8_t firstAdvChan = (adv_param.advChanMap & LL_ADV_CHAN_37) != 0 ? 37 :
                                 (adv_param.advChanMap & LL_ADV_CHAN_38) != 0 ? 38 : 39;
                                   
        if(adv_param.advNextChan == firstAdvChan)
        {
            osal_set_event(g_adv_taskID, g_adv_taskEvent);                                                    
        }        
    }
  
    return( LL_STATUS_SUCCESS );
}
/*******************************************************************************
 * @fn          llSetupUndirectedAdvEvt0
 *
 * @brief       This function process for Undirected Advertising.
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
void  llSetupUndirectedAdvEvt0(void)
{
    uint32_t  ch_idx;
    int i;
    // next adv channel invalid, get 1st adv chn
    if (adv_param.advNextChan > LL_ADV_CHAN_LAST || adv_param.advNextChan < LL_ADV_CHAN_FIRST)
        adv_param.advNextChan =  llGetNextAdvChn(0);     // get 1st adv chn
        
    ch_idx = adv_param.advNextChan;      
    adv_param.advNextChan = llGetNextAdvChn(adv_param.advNextChan);
    // schedule next adv time
    if (ch_idx >= adv_param.advNextChan)
    {          // next adv event
        int random_delay = get_timer_count(AP_TIM3) & 0x3ff;     // random adv event delay to avoid collision in the air, 0 - 1023us
        
        i = (adv_param.advChanMap & 0x01)
          + ((adv_param.advChanMap & 0x02) >> 1)
          + ((adv_param.advChanMap & 0x04) >> 2)
          - 1;
		
        ll_schedule_next_event(adv_param.advInterval * 625 - (i * pGlobal_config[ADV_CHANNEL_INTERVAL]) + (getPN23RandNumber()>>11)+random_delay);              
    }
    else      
    {
        ll_schedule_next_event(pGlobal_config[ADV_CHANNEL_INTERVAL]);     
    }

    
    // set proposed state
    llState = LL_STATE_ADV_UNDIRECTED;
//    ll_debug_output(DEBUG_LL_STATE_ADV_UNDIRECTED);
         
    //============== configure and trigger LL HW engine, LL HW work in Tx - Rx mode  ==================
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    
    set_access_address(ADV_SYNCH_WORD);   // access address
    
    set_channel(ch_idx);             // channel
    
    set_whiten_seed(ch_idx);         // whiten seed
    
    set_max_length(50);            // rx PDU max length, may receive SCAN_REQ/CONN_REQ
    
    ll_hw_set_trx_settle  (pGlobal_config[LL_HW_BB_DELAY_ADV], 
                           pGlobal_config[LL_HW_AFE_DELAY_ADV], 
                           pGlobal_config[LL_HW_PLL_DELAY_ADV]);		//TxBB,RxAFE,PLL
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();    
        
    ll_hw_set_trx();                      // set LL HW as Tx - Rx mode  
    ll_hw_ign_rfifo(LL_HW_IGN_EMP | LL_HW_IGN_CRC);		//set the rfifo ign control
    
    //write Tx FIFO
    ll_hw_write_tfifo((uint8 *)&(g_tx_adv_buf.txheader), ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   
    
    ll_hw_go();     
    llWaitingIrq = TRUE;
    
    g_pmCounters.ll_send_undirect_adv_cnt ++;
    
//    ll_debug_output(DEBUG_LL_HW_SET_TRX);
} 

/*******************************************************************************
 * @fn          llSetupNonConnectableAdvEvt0
 *
 * @brief       This function process for Nonconnectable Advertising.
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
void llSetupNonConnectableAdvEvt0( void )
{   
    uint32_t  ch_idx;
    int i;   
    
    // next adv channel invalid, get 1st adv chn
    if (adv_param.advNextChan > LL_ADV_CHAN_LAST || adv_param.advNextChan < LL_ADV_CHAN_FIRST)
        adv_param.advNextChan =  llGetNextAdvChn(0);     // get 1st adv chn
        
    ch_idx = adv_param.advNextChan;      
    adv_param.advNextChan = llGetNextAdvChn(adv_param.advNextChan);
    // schedule next adv time
    if (ch_idx >= adv_param.advNextChan)
    {          // next adv event
        int random_delay = get_timer_count(AP_TIM3) & 0x3ff;;     // random adv event delay to avoid collision in the air
        
        i = (adv_param.advChanMap & 0x01)
          + ((adv_param.advChanMap & 0x02) >> 1)
          + ((adv_param.advChanMap & 0x04) >> 2)
          - 1;
        ll_schedule_next_event(adv_param.advInterval * 625 - (i * pGlobal_config[NON_ADV_CHANNEL_INTERVAL]) + random_delay);
    }
    else      
        ll_schedule_next_event(pGlobal_config[NON_ADV_CHANNEL_INTERVAL]);    

    // set proposed state
    llState = LL_STATE_ADV_NONCONN;
//    ll_debug_output(DEBUG_LL_STATE_ADV_NONCONN);
         
    //============== configure and trigger LL HW engine, LL HW work in Single Tx mode  ==================
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    
    set_access_address(ADV_SYNCH_WORD);   // access address
    
    set_channel(ch_idx);             // channel
    
    set_whiten_seed(ch_idx);         // whiten seed
    
    set_max_length(0xff);            // rx PDU max length
    
    ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_ADV], 
                           pGlobal_config[LL_HW_AFE_DELAY_ADV], 
                           pGlobal_config[LL_HW_PLL_DELAY_ADV]);		//TxBB,RxAFE,PLL
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();    
        
    ll_hw_set_stx();                      // set LL HW as Tx - Rx mode  
    ll_hw_ign_rfifo(LL_HW_IGN_ALL);		//set the rfifo ign control
    
    //write Tx FIFO
    ll_hw_write_tfifo((uint8 *)&(g_tx_adv_buf.txheader), ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   
    
    ll_hw_go();     
    llWaitingIrq = TRUE;
    
    g_pmCounters.ll_send_nonconn_adv_cnt ++;
    
//    ll_debug_output(DEBUG_LL_HW_SET_STX);
    
}  /*  end of function  */

/*******************************************************************************
 * @fn          llSetupScannableAdvEvt0
 *
 * @brief       This function process Discoverable Undirected
 *              Advertising.
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
void llSetupScannableAdvEvt0( void )
{
    uint32_t  ch_idx;
    int i;

    // next adv channel invalid, get 1st adv chn
    if (adv_param.advNextChan > LL_ADV_CHAN_LAST || adv_param.advNextChan < LL_ADV_CHAN_FIRST)
        adv_param.advNextChan =  llGetNextAdvChn(0);     // get 1st adv chn
        
    ch_idx = adv_param.advNextChan;      
    adv_param.advNextChan = llGetNextAdvChn(adv_param.advNextChan);
    // schedule next adv time
    if (ch_idx >= adv_param.advNextChan)
    {          // next adv event
        int random_delay = get_timer_count(AP_TIM3) & 0x3ff;;     // random adv event delay to avoid collision in the air
        
        i = (adv_param.advChanMap & 0x01)
          + ((adv_param.advChanMap & 0x02) >> 1)
          + ((adv_param.advChanMap & 0x04) >> 2)
          - 1;
        ll_schedule_next_event(adv_param.advInterval * 625 - (i * pGlobal_config[ADV_CHANNEL_INTERVAL]) + random_delay);
    }
    else      
        ll_schedule_next_event(pGlobal_config[ADV_CHANNEL_INTERVAL]);    
    
    // set proposed state
    llState = LL_STATE_ADV_SCAN;
//    ll_debug_output(DEBUG_LL_STATE_SCAN);
         
    //============== configure and trigger LL HW engine, LL HW work in Tx - Rx mode  ==================
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    
    set_access_address(ADV_SYNCH_WORD);   // access address
    
    set_channel(ch_idx);             // channel
    
    set_whiten_seed(ch_idx);         // whiten seed
    
    set_max_length(50);            // rx PDU max length, may receive SCAN_REQ
    
    ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_ADV], 
                           pGlobal_config[LL_HW_AFE_DELAY_ADV], 
                           pGlobal_config[LL_HW_PLL_DELAY_ADV]);		//TxBB,RxAFE,PLL
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();    
        
    ll_hw_set_trx();                      // set LL HW as Tx - Rx mode  
    ll_hw_ign_rfifo(LL_HW_IGN_EMP | LL_HW_IGN_CRC);		//set the rfifo ign control
    
    //write Tx FIFO
    ll_hw_write_tfifo((uint8 *)&(g_tx_adv_buf.txheader), ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   
    
    ll_hw_go();     
    llWaitingIrq = TRUE;    
    
    g_pmCounters.ll_send_scan_adv_cnt ++;
    
//    ll_debug_output(DEBUG_LL_HW_SET_TRX);
} 


/*******************************************************************************
 * @fn          llSetupDirectedAdvEvt0
 *
 * @brief       This function process for Directed Advertising.
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
// for high duty cycle, should send pdu in all adv channels in 3.75ms
void llSetupDirectedAdvEvt0( void )
{    
    uint32  ch_idx, interval;
    int i;
  
    if (adv_param.advEvtType == LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)
        interval = pGlobal_config[HDC_DIRECT_ADV_INTERVAL];    //   adv event < 3.75ms
    else
        interval = pGlobal_config[LDC_DIRECT_ADV_INTERVAL];    // should < 10ms according to spec

// 2020-5-7, g_tx_adv_buf should be filled in function LL_SetAdvParam0. To test
#if 0
    // construct transmit PDU
    osal_memset(g_tx_adv_buf.data, 0, sizeof(g_tx_adv_buf.data));
    SET_BITS(g_tx_adv_buf.txheader, ADV_DIRECT_IND, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
    SET_BITS(g_tx_adv_buf.txheader, adv_param.ownAddrType, TX_ADD_SHIFT, TX_ADD_MASK);
    SET_BITS(g_tx_adv_buf.txheader, peerInfo.peerAddrType, RX_ADD_SHIFT, RX_ADD_MASK);
    SET_BITS(g_tx_adv_buf.txheader, 12, LENGTH_SHIFT, LENGTH_MASK);
    osal_memcpy(g_tx_adv_buf.data,  adv_param.ownAddr, 6);
    osal_memcpy((uint8_t *) &(g_tx_adv_buf.data[6]), peerInfo.peerAddr, 6);    
#endif	

    // next adv channel invalid, get 1st adv chn
    if (adv_param.advNextChan > LL_ADV_CHAN_LAST || adv_param.advNextChan < LL_ADV_CHAN_FIRST)
        adv_param.advNextChan =  llGetNextAdvChn(0);     // get 1st adv chn
      
    ch_idx = adv_param.advNextChan;      
    adv_param.advNextChan = llGetNextAdvChn(adv_param.advNextChan);
    // schedule next adv time
    if (adv_param.advEvtType == LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)
    {
        g_llHdcDirAdvTime += interval + pGlobal_config[DIR_ADV_DELAY] ;
        if (g_llHdcDirAdvTime >= MAX_HDC_DIRECT_ADV_TIME)     // for HDC direct adv, should not adv more than 1.28s
        {
            llState = LL_STATE_IDLE;                       // back to idle

            adv_param.advMode = LL_ADV_MODE_OFF;
            (void)osal_set_event( LL_TaskID, LL_EVT_DIRECTED_ADV_FAILED );
            return;
        }
        ll_schedule_next_event(interval);   
        
    }
    else if (ch_idx >= adv_param.advNextChan)
    {          // next adv event    
        i = (adv_param.advChanMap & 0x01)
          + ((adv_param.advChanMap & 0x02) >> 1)
          + ((adv_param.advChanMap & 0x04) >> 2)
          - 1;
        ll_schedule_next_event(adv_param.advInterval * 625 - i * interval);
    }
    else      
        ll_schedule_next_event(interval);      
  
    // set proposed state
    llState = LL_STATE_ADV_DIRECTED;
//    ll_debug_output(DEBUG_LL_STATE_ADV_DIRECTED);    
  
    //============== configure and trigger LL HW engine, LL HW work in Tx - Rx mode  ==================
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels

    set_access_address(ADV_SYNCH_WORD);   // access address

    set_channel(ch_idx);             // channel

    set_whiten_seed(ch_idx);         // whiten seed

    set_max_length(0xff);            // rx PDU max length

    ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_ADV], 
                         pGlobal_config[LL_HW_AFE_DELAY_ADV], 
                         pGlobal_config[LL_HW_PLL_DELAY_ADV]);		//TxBB,RxAFE,PLL
  
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();    
      
    ll_hw_set_trx();                      // set LL HW as Tx - Rx mode  
    ll_hw_ign_rfifo(LL_HW_IGN_EMP | LL_HW_IGN_CRC);		//set the rfifo ign control
  
    //write Tx FIFO
    ll_hw_write_tfifo((uint8 *)&(g_tx_adv_buf.txheader), ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   
  
    ll_hw_go();     
    llWaitingIrq = TRUE;   
    
    if (adv_param.advEvtType == LL_ADV_CONNECTABLE_HDC_DIRECTED_EVT)
        g_pmCounters.ll_send_hdc_dir_adv_cnt ++;
    else
        g_pmCounters.ll_send_ldc_dir_adv_cnt ++;
    
////    ll_debug_output(DEBUG_LL_HW_SET_TRX);
	 
} /*  end of function */

/*******************************************************************************
 * @fn          llCalcTimerDrift0
 *
 * @brief       This function is used to calculate the timer drift based on a
 *              given time interval and a Sleep Clock Accuracy (SCA) for the
 *              connection.
 *
 *              Note: This routine assumes that the scaValue index is valid.
 *
 *              
 *
 * input parameters
 *
 * @param       connInterval - The connection interval in 625us ticks.
 * @param       slaveLatency - Number of skipped events.
 * @param       sleepClkAccuracy - SCA of peer
 * @param       timerDrift   - Pointer for storing the timer drift adjustment.
 *
 * output parameters
 *
 * @param       timerDrift   - The timer drift adjustment (coarse/fine ticks).
 *
 * @return      None.
 */
void llCalcTimerDrift0( uint32    connInterval,
                        uint16   slaveLatency,
                        uint8    sleepClkAccuracy,
                        uint32   *timerDrift )      // HZF, it seems we need't corse & fine time, change to uint32
{
    uint32 time;
    uint16 sca = 0;
    
    // adjust the connection interval by the slave latency
    time =  (uint32)connInterval * (uint32)(slaveLatency + 1);
      
    // include the Slave's SCA in timer drift correction
    sca = adv_param.scaValue;
    
    // convert master's SCA to PPM and combine with slave
    sca += SCA[sleepClkAccuracy ];   
    
    time *= sca;
    
    time = time / 1600;      // time * 625 / 1000 000 => time / 1600
    
    *timerDrift = time;
    
    return;
}

/*******************************************************************************
 * @fn          ll_generateTxBuffer0
 *
 * @brief       This function generate Tx data and find in Tx FIFO
 *              there are 4 kinds of data:
 *                 1. control data
 *                 2. last no-ack data
 *                 3. last no-transmit data
 *                 4. new data
 *               in the new RTLP buffer, the data should be in the below sequence:
 *                   2 --> 1 --> 3 --> 4             
 *              
 * input parameters
 *
 * @param       txFifo_vacancy - allow max tx packet number.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      the pointer of 1st not transmit packet/new packet.
 *             
 */
uint16 ll_generateTxBuffer0(int txFifo_vacancy, uint16 *pSave_ptr)
{
    int i, new_pkts_num, tx_num = 0;
	llConnState_t *connPtr;

    connPtr = &conn_param[0];        
        
    // 0. write empty packet 
    if(connPtr->llMode == LL_HW_RTLP_EMPT
      || connPtr->llMode == LL_HW_TRLP_EMPT)     //  TRLP case, to be confirmed/test
    {
        LL_HW_WRT_EMPTY_PKT;
        
        connPtr->ll_buf.tx_not_ack_pkt->valid = 0;                    // empty mode, tx_not_ack buffer null or empty packet
        tx_num ++;        
    }
    // 1. write last not-ACK packet  
    else if (connPtr->ll_buf.tx_not_ack_pkt->valid != 0)            // TODO: if the valid field could omit, move the not-ACK flag to buf.    
    {   
        ll_hw_write_tfifo((uint8 *)&(connPtr->ll_buf.tx_not_ack_pkt->header), ((connPtr->ll_buf.tx_not_ack_pkt->header & 0xff00) >> 8) + 2);  
        //txFifo_vacancy --;     

        tx_num ++;        
                           
        connPtr->ll_buf.tx_not_ack_pkt->valid = 0; 
    }   
    // 1st RTLP event, no porcess 0/1, it should be 0 because we have reset the TFIFO
    // other case, it is 1st not transmit packet/new packet
    *pSave_ptr = ll_hw_get_tfifo_wrptr();   
    
	// 3. write last not transmit packets		 
	if (connPtr->ll_buf.ntrm_cnt > 0
		&& txFifo_vacancy >= connPtr->ll_buf.ntrm_cnt)	 
	{
		for (i = 0; i < connPtr->ll_buf.ntrm_cnt ; i++)
		{
			ll_hw_write_tfifo((uint8 *)&(connPtr->ll_buf.tx_ntrm_pkts[i]->header), ((connPtr->ll_buf.tx_ntrm_pkts[i]->header & 0xff00) >> 8) + 2);						
		}			
		txFifo_vacancy -= connPtr->ll_buf.ntrm_cnt;  
		tx_num += connPtr->ll_buf.ntrm_cnt; 	   
		
		connPtr->ll_buf.ntrm_cnt = 0;
	}
    rfCounters.numTxCtrl = 0;    // add on 2017-11-15, set tx control packet number 0
        
    // 2. write control packet   
    if ((connPtr->ll_buf.tx_not_ack_pkt->valid == 0 ||                 // no tx not_ack packet, add on 2017-11-15
        (connPtr->ll_buf.tx_not_ack_pkt->header & 0x3) != LL_DATA_PDU_HDR_LLID_CONTROL_PKT)    // last nack packet is not a control packet
         && connPtr->ctrlDataIsPending                                               // we only support 1 control procedure per connection
         && !connPtr->ctrlDataIsProcess
         && txFifo_vacancy > connPtr->ll_buf.ntrm_cnt)    // tricky here:  if the Tx FIFO is full and nothing is sent in last event, then it can't fill new packet(include ctrl pkt) in new event
    {   // not in a control procedure, and there is control packet pending
        // fill ctrl packet
        ll_hw_write_tfifo((uint8 *)&(connPtr->ctrlData .header), ((connPtr->ctrlData .header & 0xff00) >> 8) + 2);   
        txFifo_vacancy --;

        tx_num ++;
        
        // put Ctrl packet in TFIFO, change the control procedure status
        connPtr->ctrlDataIsPending = 0;
        connPtr->ctrlDataIsProcess = 1;
                
        rfCounters.numTxCtrl = 1;     // add 2017-11-15, if put new ctrl packet in FIFO, add the counter
    }
    

    if (connPtr->ll_buf.ntrm_cnt != 0)
    {     // should not be here, new packets should not be sent if there is not-transmit packets
        return tx_num;
    }
      
    // 4. write  new data packets to FIFO
    new_pkts_num = getTxBufferSize(connPtr);
    if ((new_pkts_num > 0)     
        && txFifo_vacancy > 0)       
    {   // fill the data packet to Tx FIFO
        for (i = 0; i < new_pkts_num && i < txFifo_vacancy; i++)
        {
            uint8_t idx = get_tx_read_ptr(connPtr);
            ll_hw_write_tfifo((uint8 *)&(connPtr->ll_buf.tx_conn_desc[idx]->header), ((connPtr->ll_buf.tx_conn_desc[idx]->header & 0xff00) >> 8) + 2); 
            
            update_tx_read_ptr(connPtr);  
            tx_num++;            
            
            // update PM counter, add A1 ROM metal change
            connPtr->pmCounter.ll_send_data_pkt_cnt ++;
        }        
    }   
	
    return tx_num;
}


/*******************************************************************************
 * @fn          ll_read_rxfifo0
 *
 * @brief       This function read HW Rx FIFO to internal buffer
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
void ll_read_rxfifo0(void)
{            
    uint8_t packet_len, idx;            
    uint16_t pktLen;            
    uint32_t pktFoot0, pktFoot1;  
    int depth;

	llConnState_t *connPtr;

    connPtr = &conn_param[0];        // TODO   
    
    depth = ll_hw_get_rfifo_depth();      
            
    // read packet
    while (getRxBufferFree(connPtr) > 0      // SW rx buffer has vacancy
          && depth > 0)               // something to read from HW Rx FIFO
    {
        idx = get_rx_write_ptr(connPtr); 
            
        packet_len = ll_hw_read_rfifo((uint8_t*)(&(connPtr->ll_buf.rx_conn_desc[idx]->header)), 
                                           &pktLen, 
                                           &pktFoot0, 
                                           &pktFoot1); 
                
        if (packet_len == 0)
        {
            break;
        }
        connPtr->ll_buf.rx_conn_desc[idx]->valid = 1;
        update_rx_write_ptr(connPtr);     // increment write pointer
        depth -= (packet_len + 2) ;     
    }     
        
    connPtr->lastRssi =   pktFoot1 >> 24;           // RSSI , -dBm
    
    
    // TODO: get other information from pktFoot0/pktFoot1

    if (depth > 0)
    {
        // warning: some data in Rx FIFO is not read, normal this should not happen
        connPtr->pmCounter.ll_recv_abnormal_cnt ++;
    }        
}


/*******************************************************************************
 * @fn          ll_hw_read_tfifo_rtlp0
 *
 * @brief       This function read not-ack packet and untransmit packets in RT Loop
 *              This function is also used in TR Loop mode, note that for TRLP, 
 *              Tx_fifo_rd_addr_last == Tx_fifo_rd_addr, no not-ACK packet
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
void ll_hw_read_tfifo_rtlp0(void)
{
    uint32_t  tmp, irq_status; 
    uint32_t last_rd_addr, current_wr_addr;
    uint32_t current_rd_addr;
    int depth, len;
 	llConnState_t *connPtr;

    connPtr = &conn_param[0];       
    // reset the not-ACK buf & not-transmit buf status
    connPtr->ll_buf.tx_not_ack_pkt->valid = 0;
    connPtr->ll_buf.ntrm_cnt = 0;    
    
    // read registers for LL HW status
    last_rd_addr =  (*(volatile uint32_t *)(LL_HW_BASE + 0x4c)) & 0x7ff;
    
    current_wr_addr = 0x07ff & ((*(volatile uint32_t *)(LL_HW_BASE + 0x50)) >> 16);
    current_rd_addr = 0x07ff & ((*(volatile uint32_t *)(LL_HW_BASE + 0x50)));
    
    // revert HW read pointer
    tmp = *(volatile uint32_t *)(LL_HW_BASE + 0x5C);
    *(volatile uint32_t *)(LL_HW_BASE + 0x5C) = (tmp & 0x7ff0000) | last_rd_addr;   // set rd_cnt_ini as Tx_fifo_rd_addr_last
    
    // calculate depth in Tx FIFO, include not-ack + not-transmit packets, note that the depth in 4bytes unit
    depth = current_wr_addr - last_rd_addr;     // Tx_fifo_wr_addr - Tx_fifo_rd_addr_last
        
    //read not ack packet
    if (depth > 0)
    {
        if (current_rd_addr != last_rd_addr)
        {
            len = ll_hw_read_tfifo_packet((uint8 *)&(connPtr->ll_buf.tx_not_ack_pkt->header));
            depth -= len;
            connPtr->ll_buf.tx_not_ack_pkt->valid = 1;                // set not ack packet buffer valid                
        }
        else          // current_rd_addr == last_rd_addr
        {    
            irq_status = ll_hw_get_irq_status();
            if (irq_status & LIRQ_CERR2)                 // double CRC error, HW will not send packet when 2nd CRC error
            {
                len = ll_hw_read_tfifo_packet((uint8 *)&(connPtr->ll_buf.tx_not_ack_pkt->header));
                depth -= len;
                connPtr->ll_buf.tx_not_ack_pkt->valid = 1;                // set not ack packet buffer valid            
            }
            else
            {            // should not be here
            }
        }            
    }
    
    // read not transmit packets
    while (depth > 0        
        && connPtr->ll_buf.ntrm_cnt < g_maxPktPerEventTx)     // A1 ROM metal change: change to MAX_LL_BUF_LEN // 05-19: change size from (MAX_CONN_BUF - 1) to MAX_CONN_BUF. not ack packet is not part of tx num
    {
        len = ll_hw_read_tfifo_packet((uint8 *)&(connPtr->ll_buf.tx_ntrm_pkts[connPtr->ll_buf.ntrm_cnt]->header));   // read a packet from Tx FIFO
        
        depth -= len;
        connPtr->ll_buf.ntrm_cnt ++;
    }    
}

/*******************************************************************************
 * @fn          ll_hw_read_tfifo_packet0
 *
 * @brief       This function read a packet form HW Tx FIFO
 *
 * input parameters
 *
 * @param       pkt    - packet buffer pointer
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      length of pkt in 4bytes unit.
 */
int ll_hw_read_tfifo_packet0(uint8 *pkt)
{
    int j , len;

    
    *((uint32_t *)&pkt[0]) = *(volatile uint32_t *)(LL_HW_TFIFO);

    uint8_t sp =BLE_HEAD_WITH_CTE(pkt[0]);
        
    len = 	(pkt[1] + 2 + 3 +sp ) & 0x1fc;                 //+2 for Header, +3 to get ceil 	

    for(j = 4; j < len; j += 4){
        *((uint32_t *)&pkt[j]) = *(volatile uint32_t *)(LL_HW_TFIFO);
    }
    
    return (len >> 2);				
}			


/*******************************************************************************
 * @fn          ll_hw_process_RTO0
 *
 * @brief       This function will update the TFIFO last read pointer in receive time out case
 *
 * input parameters
 *
 * @param       ack_num 
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 * @Note        if receive timeout in 1st connection event and there is data in TFIFO, there should be no
 *              Not-ACK packet. This function not consider this because we will not send data packet in 1st event
 */
void ll_hw_process_RTO0(uint32 ack_num)
{
    uint32_t  tmp; 
    uint32_t last_rd_addr; 
    uint32_t current_rd_addr;
    int len, i;

    // read registers for LL HW status
    last_rd_addr = 0;
    //current_wr_addr = 0x07ff & ((*(volatile uint32_t *)(LL_HW_BASE + 0x50)) >> 16);
    //current_rd_addr = 0x07ff & ((*(volatile uint32_t *)(LL_HW_BASE + 0x50)));       //  equal last_rd_addr in RTO case
        
    //*(volatile uint32_t *)0x40030068 = current_rd_addr;
    
    // 1. get last rd addr
    for (i = 0; i < ack_num; i ++)
    {
        tmp = *(volatile uint32_t *)(LL_HW_BASE + 0x5C);
        *(volatile uint32_t *)(LL_HW_BASE + 0x5C) = (tmp & 0x7ff0000) | last_rd_addr;   // set rd_cnt_ini as Tx_fifo_rd_addr_last     
        
        tmp	= *(volatile uint32 *)(LL_HW_TFIFO);
        len = (tmp >> 8) & 0xff;                            // length of the packet
        len = ((len + 2 + 3 ) >> 2) ;                   // +2 for Header, +3 to get ceil
       
        last_rd_addr += len ;
    }
    
    // 2. get current rd addr
    tmp = *(volatile uint32_t *)(LL_HW_BASE + 0x5C);
    *(volatile uint32_t *)(LL_HW_BASE + 0x5C) = (tmp & 0x7ff0000) | last_rd_addr;   // set rd_cnt_ini as Tx_fifo_rd_addr_last
    tmp	= *(volatile uint32 *)(LL_HW_TFIFO);
    len = (tmp >> 8) & 0xff;                            // length of the packet
    len = ((len + 2 + 3 ) >> 2) ;                   // +2 for Header, +3 to get ceil    
    current_rd_addr = last_rd_addr + len;
    
    
    // set HW read pointer
    *(volatile uint32_t *)(LL_HW_BASE + 0x5C) =  (last_rd_addr << 16) | current_rd_addr;   
}

/**************************************************************************************
 * @fn          ll_hw_get_tfifo_wrptr
 *
 * @brief       This function process for HW LL getting tx fifo information
 *
 * input parameters
 *
 * @param       
 *
 * output parameters
 *
 * @param       rdPtr  : read pointer.
 *				wrPtr  : write pointer
 *				wrDepth: fifo depth can be writen
 *
 * @return      None.
 */
uint16_t ll_hw_get_tfifo_wrptr(void)
{
	uint16_t wrPtr;

	wrPtr    =  0x07ff & ((*(volatile uint32_t *)(LL_HW_BASE + 0x50)) >>16);
    
    return wrPtr;
}

/**************************************************************************************
 * @fn          ll_hw_get_loop_time
 *
 * @brief       This function get the loop timeout timer counter 
 *
 * input parameters
 *
 * @param        None.
 *
 * output parameters
 *
 * @param        None.
 *				
 *				
 *
 * @return      loop timer counter.
 */
uint32_t ll_hw_get_loop_time(void)    
{
	return (*(volatile uint32_t *)(LL_HW_BASE + 0x70));  
}

/**************************************************************************************
 * @fn          ll_hw_get_rfifo_depth
 *
 * @brief       This function get the HW LL rx fifo depth
 *
 * input parameters
 *
 * @param       
 *
 * output parameters
 *
 * @param       None.
 *				
 *				
 *
 * @return      Rx FIFO depth(i.e. words available) in 4bytes unit
 */
int ll_hw_get_rfifo_depth(void)
{
    uint16_t    rdPtr, wrPtr;
	uint32_t tmp = *(volatile uint32_t *)(LL_HW_BASE + 0x54);

	rdPtr    =  0x07ff & tmp;
	wrPtr    =  0x07ff & (tmp>>16);
	return ( wrPtr - rdPtr);	
}



/*******************************************************************************
 * @fn          ll_adptive_smart_window
 *
 * @brief      will adptive modified following var to adjust the rx window in RTLP
 *              1.slave_conn_event_recv_delay
 *              2.pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT
 *
 * input parameters
 * @param       irq_status   
 * @param       anchor_point   
 *
 * output parameters
 *
 * @param       None   
 *
 * @return      None.
 */
//void ll_adptive_smart_window0(uint32 irq_status,uint32 anchor_point)
//{
//    if(irq_status & LIRQ_TD )
//    {
//        //tracking anchpoint
//        if((        irq_status & LIRQ_COK) 
//            &&  (0== conn_param[0].firstPacket) )            
//        {
//            //==========================================================
//            // when the anchor point change litter, active smartwindow
//            uint32_t anchPre_anchCurrent;
//            anchPre_anchCurrent =   (g_smartWindowPreAnchPoint>anchor_point)
//                                  ? g_smartWindowPreAnchPoint-anchor_point
//                                  : anchor_point-g_smartWindowPreAnchPoint;
//                                  
//            if(anchPre_anchCurrent < pGlobal_config[LL_SMART_WINDOW_ACTIVE_RANGE])
//            {
//                g_smartWindowActiveCnt = (g_smartWindowActiveCnt > pGlobal_config[LL_SMART_WINDOW_ACTIVE_THD])
//                                         ? g_smartWindowActiveCnt : g_smartWindowActiveCnt+1;                   
//            }
//        }
//        else
//        {
//            g_smartWindowLater      =   0;
//            g_smartWindowActiveCnt  =   0;                                                       
//        }
//    }
//    else
//    {
//        g_smartWindowLater      =   0;
//        g_smartWindowActiveCnt  =   0;        
//    }
//
//    //record the pre anchor point
//    if(irq_status& LIRQ_TD && irq_status & LIRQ_COK)
//    {
//        g_smartWindowPreAnchPoint = anchor_point;
//    }
//
//    if(g_smartWindowActiveCnt > pGlobal_config[LL_SMART_WINDOW_ACTIVE_THD])
//    {            
//        int dlt_anchPoint = anchor_point - pGlobal_config[LL_SMART_WINDOW_TARGET];
//        g_smartWindowLater = g_smartWindowLater + (dlt_anchPoint >> pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]);
//            
//        slave_conn_event_recv_delay -= g_smartWindowLater;
//    }
//    
//    //-------------------------------------------------------------------------------
//    //enlarge the RX WINDOW when rxtimeout
//    if(irq_status & LIRQ_TD )
//    {
//        g_smartWindowRTOCnt = 0;
//    }
//    else
//    {
//        g_smartWindowRTOCnt = (g_smartWindowRTOCnt > 5) ? g_smartWindowRTOCnt : (g_smartWindowRTOCnt + 1);
//    }
//}

uint32_t getPN23RandNumber(void)
{
    g_getPn23_cnt++;
    
    uint32_t feedback = ((g_getPn23_seed & (1<<23))>>23) + ((g_getPn23_seed&(1<<18))>>18) & 0x01;
    g_getPn23_seed = ((0x007fffff & g_getPn23_seed)<<1) + feedback;

    return (g_getPn23_seed);
}

//////////////////////////
void ll_schedule_next_event(int time)
{
	set_timer(AP_TIM1, time);
	
}

/**************************************************************************************
 * @fn          ll_debug_output
 *
 * @brief       update the debug state
 *
 * input parameters
 *
 * @param       state  -  the LL status, the SRAM project will interpret the status and dirve GPIO, UART, memory print, ...
 *
 * output parameters
 *
 * @param       None
 *				
 *				
 * @return      None
 */
//void ll_debug_output(uint32 state)
//{
//    if (pGlobal_config[LL_SWITCH] & LL_DEBUG_ALLOW)
//    {
//        debug_print(state);
//    }
//}

                 
void llWaitUs(uint32_t wtTime);

// ======================== scan back off process function
/*******************************************************************************
 * @fn          llAdjBoffUpperLimitSuccess
 *
 * @brief       This function is used to handle the calculation of the Scan
 *              backoff counter upper limit based on the number of successfully
 *              received Scan Responses to our Scan Requests. When two
 *              consecutive Scan Responses are successfully received, the upper
 *              limit of the Scan backoff counter is halfed (but no less than
 *              one).
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
static void llAdjBoffUpperLimitSuccess( void )
{
    // first, since this is a success, clear the number of consecutive failures
    scanInfo.numFailure = 0;
  
    // check if we received two successful in a row
    if ( ++scanInfo.numSuccess == 2 )
    {
        // yes, so half backoff upper limit
        scanInfo.scanBackoffUL >>= 1;
    
        // however, the minimum is 1
        if ( scanInfo.scanBackoffUL == 0 )
        {
            scanInfo.scanBackoffUL = 1;
        }
    
        // reset consecutive count
        scanInfo.numSuccess = 0;
    }
  
    return;
}


/*******************************************************************************
 * @fn          llAdjBoffUpperLimitFailure
 *
 * @brief       This function is used to handle the calculation of the Scan
 *              backoff counter upper limit based on the number of failured to
 *              receive Scan Responses to our Scan Requests. When two
 *              consecutive Scan Responses fail to be received, the upper
 *              limit of the Scan backoff counter is doubled (but no more than
 *              256).
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
static void llAdjBoffUpperLimitFailure( void )
{
    // first, since this was a failure, clear the number of consecutive successes
    scanInfo.numSuccess = 0;
  
    // check if we received two failures in a row
    if ( ++scanInfo.numFailure == 2 )
    {
        // yes, so double backoff upper limit
        scanInfo.scanBackoffUL <<= 1;
    
        // maximum is 256
        if ( scanInfo.scanBackoffUL > 256 )
        {
            scanInfo.scanBackoffUL = 256;
        }
    
        // reset consecutive count
        scanInfo.numFailure = 0;
    }
  g_pmCounters.ll_tbd_cnt4++;
    return;
}


/*******************************************************************************
 * @fn          llGenerateNextBackoffCount
 *
 * @brief       This function is used to find the next Scan backoff
 *              count which determines when the next Scan Request will be sent
 *              when the appropriate Adv packet is correctly received. The
 *              backoff count is just to limit collisions. The backoff count
 *              is randomly generated, but must be between 1 and the backoff
 *              count upper limit (max 256). The upper limit changes based on
 *              the number of consecutive successful or failed Scan Responses.
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
static void llGenerateNextBackoffCount( void )
{
    // determine the new backoff count constrained by upper limit
    // Note: Backoff and Upper Limit can be 1..256.
    if ( scanInfo.scanBackoffUL == 1 )
    {
        scanInfo.currentBackoff = 1;
    }
    else // backoff count is a random number from 1..UL
    {
        scanInfo.currentBackoff = ((uint16)LL_ENC_GeneratePseudoRandNum() % scanInfo.scanBackoffUL) + 1;
    }
    return;
}

// ===============================

void llWaitUs(uint32_t wtTime)
{    
    uint32_t T1, T2, deltTick;
    
    if (wtTime == 0)
        return;
    
    T1 = read_current_fine_time();
    
    while(1){
        T2 = read_current_fine_time();
        deltTick = (T2 >= T1) ? (T2 - T1) : (BASE_TIME_UNITS - T1 + T2);
        if(deltTick > wtTime)
            break;
    }

}

//uint8_t* LL_PLUS_GetAdvDataExtendData(void)
//{    
//    return((uint8_t*)&g_rx_adv_buf.data[0]);
//}
//
//void LL_PLUS_SetAdvDataFilterCB(LL_PLUS_AdvDataFilterCB_t AdvDataFilterCBack)
//{
//    LL_PLUS_AdvDataFilterCBack=AdvDataFilterCBack;
//}
//
//void LL_PLUS_SetScanRequestData(uint8 dLen, uint8 * pData)
//{    
//    for(int i=0;i<dLen;i++)
//        g_tx_adv_buf.data[12+i]=pData[i];
//    g_tx_adv_buf.txheader=((dLen+12)<<8)|(g_tx_adv_buf.txheader&0x00ff);
//}
//
//void LL_PLUS_SetScanRequestFilterCB(LL_PLUS_ScanRequestFilterCB_t ScanRequestFilterCBack)
//{
//    LL_PLUS_ScanRequestFilterCBack=ScanRequestFilterCBack;
//}
//
//uint8 LL_PLUS_GetScanRequestExtendData(uint8* pData)
//{    
//    uint8 eLen=((g_rx_adv_buf.rxheader&0xff00)>>8)-12;
//
//    for(uint8 i=0;i<eLen;i++)
//        pData[i] = g_rx_adv_buf.data[12+i];
//
//    return eLen;
//}
//
//void LL_PLUS_GetScanerAddr(uint8* pData)
//{     
//    for(uint8 i=0;i<6;i++)
//        pData[i] = g_rx_adv_buf.data[0+i];
//
//    return;
//}
//
//void LL_PLUS_SetScanRsqData(uint8 dLen,uint8* pData)
//{       
//    for(uint8 i=0;i<dLen;i++)
//    {
//        g_tx_adv_buf.data[6+i]=pData[i];
//    }
//    g_tx_adv_buf.txheader=((dLen+6)<<8)|(g_tx_adv_buf.txheader&0x00ff);
//    
//}
//
//void LL_PLUS_SetScanRsqDataByIndex(uint8 dIdx,uint8 data)
//{    
//    tx_scanRsp_desc.data[6+dIdx]=data;
//}
//void LL_PLUS_PerStats_Init(perStatsByChan_t* p_per)
//{
//    p_perStatsByChan = p_per;
//    LL_PLUS_PerStatsReset();
//}
//
//void LL_PLUS_PerStatsReset(void)
//{
//    if(p_perStatsByChan!=NULL)
//        osal_memset(p_perStatsByChan, 0,sizeof(perStatsByChan_t));
//
//}
//
//void LL_PLUS_PerStasReadByChn(uint8 chnId,perStats_t * perStats)
//{
//    if( (p_perStatsByChan!=NULL) && (perStats!=NULL) )
//    {
//        perStats->rxNumPkts     =p_perStatsByChan->rxNumPkts[chnId];
//        perStats->rxNumCrcErr   =p_perStatsByChan->rxNumCrcErr[chnId];
//        perStats->txNumRetry    =p_perStatsByChan->txNumRetry[chnId];
//        perStats->TxNumAck      =p_perStatsByChan->TxNumAck[chnId];
//        perStats->rxToCnt       =p_perStatsByChan->rxToCnt[chnId];
//        perStats->connEvtCnt    =p_perStatsByChan->connEvtCnt[chnId];
//    }
//}

void ll_hw_tx2rx_timing_config(uint8 pkt)
{
    if(pkt==PKT_FMT_BLE1M)
    {
        ll_hw_set_rx_tx_interval(    pGlobal_config[LL_HW_Rx_TO_TX_INTV]);		//T_IFS=150us for BLE 1M
	    ll_hw_set_tx_rx_interval(    pGlobal_config[LL_HW_Tx_TO_RX_INTV]);		//T_IFS=150us for BLE 1M
    }
    else if(pkt==PKT_FMT_BLE2M)
    {
        ll_hw_set_rx_tx_interval(    pGlobal_config[LL_HW_Rx_TO_TX_INTV_2MPHY]);		//T_IFS=150us for BLE 1M
	    ll_hw_set_tx_rx_interval(    pGlobal_config[LL_HW_Tx_TO_RX_INTV_2MPHY]);	
    }
   }

void ll_hw_trx_settle_config(uint8 pkt)
{
   if(pkt==PKT_FMT_BLE1M)
    {
        ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY], 
                             pGlobal_config[LL_HW_AFE_DELAY], 
                             pGlobal_config[LL_HW_PLL_DELAY]);		// TxBB, RxAFE, PLL
    }
    else if(pkt==PKT_FMT_BLE2M)
    {
        ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_2MPHY], 
                             pGlobal_config[LL_HW_AFE_DELAY_2MPHY], 
                             pGlobal_config[LL_HW_PLL_DELAY_2MPHY]);		// TxBB, RxAFE, PLL
    }

}


// multi-connection
/*******************************************************************************
 * @fn          llSetupSecAdvEvt0
 *
 * @brief       This function process for Nonconnectable Advertising.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      uint8 - TRUE(send success), FALSE(send fail).
 */
uint8 llSetupSecAdvEvt0( void )
{ 
    uint8 ret = FALSE;


    if (llState == LL_STATE_IDLE)
    {
        if (adv_param.advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT)
	        llState = LL_STATE_ADV_UNDIRECTED;
	    else if (adv_param.advEvtType == LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)
		    llState = LL_STATE_ADV_NONCONN;
	    else if (adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT)
		    llState = LL_STATE_ADV_SCAN;

		llSetupAdv();

		llSecondaryState = LL_SEC_STATE_IDLE;

		return TRUE;
    }	

	uint8 temp_rf_fmt = g_rfPhyPktFmt;
	g_rfPhyPktFmt = LE_1M_PHY;
	rf_phy_change_cfg(g_rfPhyPktFmt);
    if (adv_param.advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT)
		ret = llSetupSecConnectableAdvEvt();
	else if (adv_param.advEvtType == LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT)
		ret = llSetupSecNonConnectableAdvEvt();
	else if (adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT)
		ret = llSetupSecScannableAdvEvt();
	else 
		ret = FALSE;          // other type adv should not here

	llSecondaryState = LL_SEC_STATE_ADV;
	g_rfPhyPktFmt = temp_rf_fmt;

	return ret;
}

/*******************************************************************************
 * @fn          llSetupSecConnectableAdvEvt
 *
 * @brief       This function process for set up sec undirect Advertising.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      uint8 - TRUE(send success), FALSE(send fail).
 */
uint8 llSetupSecConnectableAdvEvt0( void )
{   
    uint32_t  ch_idx;

    // Hold off interrupts.
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();		
	if (llWaitingIrq)
    {
		g_pmCounters.ll_tbd_cnt1++;
        HAL_EXIT_CRITICAL_SECTION(  ); 
		return FALSE;
    }
    
    // next adv channel invalid, get 1st adv chn
    if (adv_param.advNextChan > LL_ADV_CHAN_LAST || adv_param.advNextChan < LL_ADV_CHAN_FIRST)
        adv_param.advNextChan =  llGetNextAdvChn(0);     // get 1st adv chn
        
    ch_idx = adv_param.advNextChan;      
    adv_param.advNextChan = llGetNextAdvChn(adv_param.advNextChan);
         
    //============== configure and trigger LL HW engine, LL HW work in Single Tx mode  ==================
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    
    set_access_address(ADV_SYNCH_WORD);   // access address
    
    set_channel(ch_idx);             // channel
    
    set_whiten_seed(ch_idx);         // whiten seed
    
    set_max_length(0xff);            // rx PDU max length
    
    ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_ADV], 
                           pGlobal_config[LL_HW_AFE_DELAY_ADV], 
                           pGlobal_config[LL_HW_PLL_DELAY_ADV]);		//TxBB,RxAFE,PLL
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();    
        
    ll_hw_set_trx();					  // set LL HW as Tx - Rx mode	
    ll_hw_ign_rfifo(LL_HW_IGN_EMP | LL_HW_IGN_CRC); 	//set the rfifo ign control
    
    //write Tx FIFO
    ll_hw_write_tfifo((uint8 *)&(g_tx_adv_buf.txheader), ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   
    
    ll_hw_go();     
    llWaitingIrq = TRUE;  
    HAL_EXIT_CRITICAL_SECTION(  ); 
	g_pmCounters.ll_send_conn_adv_cnt ++;           // adv in conn state counter
	g_pmCounters.ll_send_undirect_adv_cnt ++;
	
#ifdef DEBUG_LL
	LOG("c%d ", ch_idx);
#endif
    return TRUE;
	
}  /*  end of function  */


/*******************************************************************************
 * @fn          llSetupSecScannableAdvEvt0
 *
 * @brief       This function process for setup scannable Advertising.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      uint8 - TRUE(send success), FALSE(send fail).
 */
uint8 llSetupSecScannableAdvEvt0( void )
{   
    uint32_t  ch_idx;

    // Hold off interrupts.
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();		
	if (llWaitingIrq)
    {
		g_pmCounters.ll_tbd_cnt1++;           // to update the conuter name
        HAL_EXIT_CRITICAL_SECTION(  ); 
		return FALSE;
    }
    
    // next adv channel invalid, get 1st adv chn
    if (adv_param.advNextChan > LL_ADV_CHAN_LAST || adv_param.advNextChan < LL_ADV_CHAN_FIRST)
        adv_param.advNextChan =  llGetNextAdvChn(0);     // get 1st adv chn
        
    ch_idx = adv_param.advNextChan;      
    adv_param.advNextChan = llGetNextAdvChn(adv_param.advNextChan);
         
    //============== configure and trigger LL HW engine, LL HW work in Single Tx mode  ==================
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    
    set_access_address(ADV_SYNCH_WORD);   // access address
    
    set_channel(ch_idx);             // channel
    
    set_whiten_seed(ch_idx);         // whiten seed
    
    set_max_length(0xff);            // rx PDU max length
    
    ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_ADV], 
                           pGlobal_config[LL_HW_AFE_DELAY_ADV], 
                           pGlobal_config[LL_HW_PLL_DELAY_ADV]);		//TxBB,RxAFE,PLL
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();    
        
    ll_hw_set_trx();					  // set LL HW as Tx - Rx mode	
    ll_hw_ign_rfifo(LL_HW_IGN_EMP | LL_HW_IGN_CRC); 	//set the rfifo ign control
    
    //write Tx FIFO
    ll_hw_write_tfifo((uint8 *)&(g_tx_adv_buf.txheader), ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   
    
    ll_hw_go();     
    llWaitingIrq = TRUE;  
    HAL_EXIT_CRITICAL_SECTION(  ); 
	g_pmCounters.ll_send_conn_adv_cnt ++;           // adv in conn state counter
	g_pmCounters.ll_send_scan_adv_cnt ++;
	
#ifdef DEBUG_LL
	LOG("c%d ", ch_idx);
#endif
    return TRUE;
	
}  /*  end of function  */


// add in A2
/*******************************************************************************
 * @fn          llSetupSecNonConnectableAdvEvt
 *
 * @brief       This function process for Nonconnectable Advertising.
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      uint8 - TRUE(send success), FALSE(send fail).
 */
uint8 llSetupSecNonConnectableAdvEvt0( void )
{   
    uint32_t  ch_idx;
    // Hold off interrupts.
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();		
	if (llWaitingIrq)
    {
		g_pmCounters.ll_tbd_cnt1++;
        HAL_EXIT_CRITICAL_SECTION(); 
		return FALSE;
    }
    
    // next adv channel invalid, get 1st adv chn
    if (adv_param.advNextChan > LL_ADV_CHAN_LAST || adv_param.advNextChan < LL_ADV_CHAN_FIRST)
        adv_param.advNextChan =  llGetNextAdvChn(0);     // get 1st adv chn
        
    ch_idx = adv_param.advNextChan;      
    adv_param.advNextChan = llGetNextAdvChn(adv_param.advNextChan);
         
    //============== configure and trigger LL HW engine, LL HW work in Single Tx mode  ==================
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    
    set_access_address(ADV_SYNCH_WORD);   // access address
    
    set_channel(ch_idx);             // channel
    
    set_whiten_seed(ch_idx);         // whiten seed
    
    set_max_length(0xff);            // rx PDU max length
    
    ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_ADV], 
                           pGlobal_config[LL_HW_AFE_DELAY_ADV], 
                           pGlobal_config[LL_HW_PLL_DELAY_ADV]);		//TxBB,RxAFE,PLL
    
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();    
        
    ll_hw_set_stx();                      // set LL HW as Tx - Rx mode  
    ll_hw_ign_rfifo(LL_HW_IGN_ALL);		//set the rfifo ign control
    
    //write Tx FIFO
    ll_hw_write_tfifo((uint8 *)&(g_tx_adv_buf.txheader), ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   
    
    ll_hw_go();     
    llWaitingIrq = TRUE;  
	
    HAL_EXIT_CRITICAL_SECTION(); 
	g_pmCounters.ll_send_conn_adv_cnt ++;             // adv in conn state counter
	g_pmCounters.ll_send_nonconn_adv_cnt ++;

#ifdef DEBUG_LL
	LOG("nc%d ", ch_idx);
#endif
    return TRUE;
	
}  /*  end of function  */

/*******************************************************************************
 * @fn          llSecAdvAllow
 *
 * @brief       Decision the remain time to next LL conn interval is enough 
 *              for no_conn adv, consider the time for no conn advertisement 
 *              in channel 37/38/39. Plus some margin.
 *              
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
 * @return      TRUE:  Secondary advertisement allow
 *              FALSE: Secondary advertisement NOT allow
 */
uint8 llSecAdvAllow0(void)
{
    uint32 advTime, margin;
	uint32 remainTime;
	uint8 ret = FALSE;

    // Hold off interrupts.
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();	

	// read global config to get advTime and margin
    advTime = pGlobal_config[LL_NOCONN_ADV_EST_TIME];
	margin = pGlobal_config[LL_NOCONN_ADV_MARGIN];

    // remain time before trigger LL HW
	remainTime = read_LL_remainder_time();

	if(( llState == LL_STATE_IDLE ) || ((remainTime > advTime + margin) && !llWaitingIrq))
        ret = TRUE;
	else
	{
		llSecondaryState = LL_SEC_STATE_ADV_PENDING;
		g_pmCounters.ll_conn_adv_pending_cnt ++;
	}

    HAL_EXIT_CRITICAL_SECTION(); 

	return ret;
}

/*******************************************************************************
 * @fn          llCalcMaxScanTime
 *
 * @brief       Decide the maximum scan time, consider remain time and 
 *              the margin              
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
 * @return      allow maximum scan time in us
 *              
 */
uint32 llCalcMaxScanTime0(void)
{
    uint32 margin, scanTime;
	uint32 remainTime;

	margin = pGlobal_config[LL_SEC_SCAN_MARGIN];
	
    // Hold off interrupts.
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();	

    // remain time before trigger LL HW
	remainTime = read_LL_remainder_time();	

	scanTime = 0;

	if (remainTime > margin + pGlobal_config[LL_MIN_SCAN_TIME]
		&& !llWaitingIrq)
		scanTime = remainTime - margin;
    HAL_EXIT_CRITICAL_SECTION(); 

    return (scanTime);
}

/*******************************************************************************
 * @fn          llSetupSecScan
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
void llSetupSecScan0( uint8 chan )
{
    uint32 scanTime;
	
    // Hold off interrupts.
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();		
	scanTime = scanInfo.scanWindow * 625;

//	if(llWaitingIrq)
//	{
//	    LOG("==== error, mode: %d\n", scanInfo.scanMode);		
//	}

    if (llState == LL_STATE_IDLE)
    {
		llState = LL_STATE_SCAN;
		llSecondaryState = LL_SEC_STATE_IDLE;
    }
	else
	{
        // calculate scan time
        scanTime = llCalcMaxScanTime();
        if (scanTime)		// trigger scan   
        {
            llSecondaryState = LL_SEC_STATE_SCAN;
        }
        else                // no enough time to scan, pending
        {
            llSecondaryState = LL_SEC_STATE_SCAN_PENDING;
		    g_pmCounters.ll_conn_scan_pending_cnt ++;
            HAL_EXIT_CRITICAL_SECTION(  ); 	
	        return;
        }
	}
    
    if (scanTime > scanInfo.scanWindow * 625)
        scanTime = scanInfo.scanWindow * 625;

	
    // reset all FIFOs; all data is forfeit
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();

    uint8 temp_rf_fmt = g_rfPhyPktFmt;
	g_rfPhyPktFmt = LE_1M_PHY;
	rf_phy_change_cfg(g_rfPhyPktFmt);

  
	set_crc_seed(ADV_CRC_INIT_VALUE); // crc seed for adv is same for all channels

	set_access_address(ADV_SYNCH_WORD);

	set_channel(chan);

	set_whiten_seed(chan); 

	set_max_length(0xff);  
		
    
    ll_hw_set_rx_timeout(scanTime);   // maximum scan time, note that actual scan time may exceed the limit if timer expiry when LL engine receiving a report
    
    ll_hw_set_srx();

	ll_hw_ign_rfifo(LL_HW_IGN_CRC|LL_HW_IGN_EMP);
    
    ll_hw_go();  
    g_rfPhyPktFmt = temp_rf_fmt;
    llScanT1 = read_current_fine_time();
  
    llWaitingIrq = TRUE;      
    HAL_EXIT_CRITICAL_SECTION(); 	
//    uint32 remainTime = read_LL_remainder_time();
//	LOG("<%d %d>", scanTime, remainTime);

    return;
}


/*******************************************************************************
 * @fn          llSetupSecInit
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
//void llSetupSecInit( uint8 chan )
//{
//    uint32 scanTime;
//
//    // Hold off interrupts.
//    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();	
//	scanTime = scanInfo.scanWindow * 625;
//
//    if (llState == LL_STATE_IDLE)
//    {
//		llState = LL_STATE_INIT;
//		llSecondaryState = LL_SEC_STATE_IDLE;
//    }
//	else
//	{
//        // calculate scan time
//        scanTime = llCalcMaxScanTime();
//        if (scanTime)		// trigger scan   
//        {
//            llSecondaryState = LL_SEC_STATE_INIT;
//        }
//        else                // no enough time to scan, pending
//        {
//            llSecondaryState = LL_SEC_STATE_INIT_PENDING;
////		    g_pmCounters.ll_conn_scan_pending_cnt ++;
//            HAL_EXIT_CRITICAL_SECTION(  ); 	
//		    return;
//        }
//	}
//    
//    if (scanTime > initInfo.scanWindow * 625)
//        scanTime = initInfo.scanWindow * 625;
//
//	
//    // reset all FIFOs; all data is forfeit
//    ll_hw_rst_tfifo();
//    ll_hw_rst_rfifo();
//  
//	set_crc_seed(ADV_CRC_INIT_VALUE); // crc seed for adv is same for all channels
//
//	set_access_address(ADV_SYNCH_WORD);
//
//	set_channel(chan);
//
//	set_whiten_seed(chan); 
//
//	set_max_length(0xff);  
//
//    // A2 multi-conn
//	ll_hw_set_rx_timeout(88);
////	ll_hw_set_rx_timeout_1st(88);	
//    
//    ll_hw_set_rx_timeout(scanTime);   // maximum scan time, note that actual scan time may exceed the limit if timer expiry when LL engine receiving a report
//    
//    ll_hw_set_srx();
//
//	ll_hw_ign_rfifo(LL_HW_IGN_CRC|LL_HW_IGN_EMP);
//    
//    ll_hw_go();  
//  
//    llScanT1 = read_current_fine_time();
////hal_gpio_write(GPIO_P18, 1);  
//    llWaitingIrq = TRUE;      
//    HAL_EXIT_CRITICAL_SECTION(  ); 	
//
//    return;
//}

//uint32  ll_get_next_timer(uint8 current_conn_id)
//{
//    uint8  next;
//	uint32 timer;
//
//	next = ll_get_next_active_conn(current_conn_id);
//	if (next == LL_INVALID_CONNECTION_ID)
//		return 0;
//
//	timer = (next > current_conn_id) ? 
//	      ((next - current_conn_id) * g_ll_conn_ctx.per_slot_time * 625) : 
//	      (((g_ll_conn_ctx.connInterval << 1) - (current_conn_id - next) * g_ll_conn_ctx.per_slot_time) * 625);
//
//
//    return timer;
//
//}
//#pragma O0


/*******************************************************************************
 * @fn          ll_scheduler
 *
 * @brief       schedule next task, if current connection will be free, input 
 *              parameter should be LL_INVALID_TIME. The function is invoked 
 *              after old connection task end, it will not add new task but may 
 *              delete exist task
 *
 * input parameters
 *
 * @param       time - schedule time for current connection
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
 // TODO
//void ll_scheduler0(uint32 time)
//{
//    uint32  T1, T2, delta, min, prio_adj;
//	uint8   i, next, temp;
//
//	T1 = read_current_fine_time();
//
//    // timer1 is running, normally it should not occur
//    if (isTimer1Running())
//    {
////        LOG("=== ASSERT FAIL, timer1 running when invoke ll_scheduler ===\n");
//        g_pmCounters.ll_evt_shc_err++;
//        return;
//    }
//
//	// if timer1 is not running, calculate the time elapse since last timer expiry
//	delta = g_ll_conn_ctx.current_timer + LL_TIME_DELTA(g_ll_conn_ctx.timerExpiryTick, T1) + pGlobal_config[TIMER_ISR_ENTRY_TIME];
//	
//    // update current context
//	g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].remainder = time;      // if current conn terminal, the parameter "time" shall be LL_INVALID_TIME
//
//	min = time;
//	if (time == LL_INVALID_TIME)
//    {
//        ll_deleteTask(g_ll_conn_ctx.currentConn);
//		g_ll_conn_ctx.currentConn = LL_INVALID_CONNECTION_ID;
//    }
//
//	next = g_ll_conn_ctx.currentConn;
//
//    if (next != LL_INVALID_CONNECTION_ID)
//    {
//        // if we want master or slave connection has higher schedule priority, set LL_MASTER_PREEMPHASIS/LL_SLAVE_PREEMPHASIS
//        if (g_ll_conn_ctx.scheduleInfo[next].linkRole == LL_ROLE_MASTER)
//            min = (time > pGlobal_config[LL_MULTICONN_MASTER_PREEMP]) ? (time - pGlobal_config[LL_MULTICONN_MASTER_PREEMP]) : 0;
//        if (g_ll_conn_ctx.scheduleInfo[next].linkRole == LL_ROLE_SLAVE)
//            min = (time > pGlobal_config[LL_MULTICONN_SLAVE_PREEMP]) ? (time - pGlobal_config[LL_MULTICONN_SLAVE_PREEMP]) : 0;
//
//    }
//
//	// update schedule task list and get the earliest task
//	for (i = 0; i < g_maxConnNum; i++)
//    {
//        if ((i != g_ll_conn_ctx.currentConn) && conn_param[i].active)
//       	{
//       	    // task conflict process
//       	    // if there is no enough time for new task, invoke relate slave/master conn event process function
////       	    if (g_ll_conn_ctx.scheduleInfo[i].remainder < delta + g_ll_conn_ctx.scheduleInfo[i].task_duration)    
//            if (g_ll_conn_ctx.scheduleInfo[i].remainder < delta + 40)     // 40 : margin for process delay, unit: us
//       	    {                              // no enough time to process the event, regard the event as missed and update the conn context and timer
//       	        uint8  ret = LL_PROC_LINK_KEEP;
//				
//       	        if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_MASTER)
//       	        {
//       	            // temporary update g_ll_conn_ctx.currentConn to current connection ID because 
//       	            // ll_processMissMasterEvt will invoke function using global variable g_ll_conn_ctx.currentConn
//       	            temp = g_ll_conn_ctx.currentConn;
//					g_ll_conn_ctx.currentConn = i;
//					ret = ll_processMissMasterEvt(i);
//					g_ll_conn_ctx.currentConn = temp;
//       	        }
//				else if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_SLAVE)
//       	        {
//       	            // TODO: add by Zhangzhufei for Huapu, to check
//       	            // temporary update g_ll_conn_ctx.currentConn to current connection ID because 
//       	            // ll_processMissSlaveEvt will invoke function using global variable g_ll_conn_ctx.currentConn
//       	            temp = g_ll_conn_ctx.currentConn;
//					g_ll_conn_ctx.currentConn = i;	
//					if( delta > g_ll_conn_ctx.scheduleInfo[i].remainder )
//					{
//						llConnState_t *connPtr = &conn_param[i];
//						uint8 missCE = (( delta - g_ll_conn_ctx.scheduleInfo[i].remainder ) / connPtr->curParam.connInterval ) + 1;
//						for( uint8 misI = 0; misI < missCE ; misI++)
//							ll_processMissSlaveEvt(i);
//					}
//					else
//					{
//						ret = ll_processMissSlaveEvt(i);
//					}
//       	            g_ll_conn_ctx.currentConn = temp;
//       	        }
//
//				if (ret == LL_PROC_LINK_TERMINATE)    // the connection is terminated, update shcedule information. 
//				{
//				    ll_deleteTask(i);
//					continue;                  // continue next link
//				}
//
//				// increase the task priority
//				g_ll_conn_ctx.scheduleInfo[i].priority ++;
//				if (g_ll_conn_ctx.scheduleInfo[i].priority == LL_SCH_PRIO_LAST)  
//					g_ll_conn_ctx.scheduleInfo[i].priority = LL_SCH_PRIO_IMMED;
//       	    }
//
//            prio_adj =0;
//            if (min != LL_INVALID_TIME)
//            {
//                // consider the task prioriy
//                switch (g_ll_conn_ctx.scheduleInfo[i].priority)
//                {
//                case LL_SCH_PRIO_LOW:
//					prio_adj = 0;
//					break;
//                case LL_SCH_PRIO_MED:
//					prio_adj = MAX(LL_TASK_MASTER_DURATION, LL_TASK_SLAVE_DURATION) + 2000;
//					break;
//                case LL_SCH_PRIO_HIGH:
//					prio_adj = MAX(LL_TASK_MASTER_DURATION, LL_TASK_SLAVE_DURATION) + 10 + 2000;
//					break;
//                case LL_SCH_PRIO_IMMED:
//					prio_adj = MAX(LL_TASK_MASTER_DURATION, LL_TASK_SLAVE_DURATION) + 20 + 2000;
//					break;
//                default:
//					prio_adj = 0;
//					break;				
//                }
//
//                if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_MASTER)
//                    prio_adj += pGlobal_config[LL_MULTICONN_MASTER_PREEMP];
//                if (g_ll_conn_ctx.scheduleInfo[i].linkRole == LL_ROLE_SLAVE)
//                    prio_adj += pGlobal_config[LL_MULTICONN_SLAVE_PREEMP];				
//            }                       
//
//			// update remainder time 
//       	    g_ll_conn_ctx.scheduleInfo[i].remainder -= delta;
//			
//			if (g_ll_conn_ctx.scheduleInfo[i].remainder < min + prio_adj)
//       	    {
//       	        next = i;
//				min = (g_ll_conn_ctx.scheduleInfo[i].remainder > prio_adj) ? (g_ll_conn_ctx.scheduleInfo[i].remainder - prio_adj) : 0;     
//       	    }
//       	}
//    }
//
//	if (min == LL_INVALID_TIME)     // all task may be delete, not start timer 
//	{
////	    LOG("=== all task delete ====\n");
//		return;
//	}
//
//	T2 = read_current_fine_time();
//
//	// calculate the time elapse since enter this function. 
//	delta = LL_TIME_DELTA(T1, T2);
//
//	if (g_ll_conn_ctx.scheduleInfo[next].remainder < delta)          // TODO: should not go here, if this issue detected, root cause should be invest
//	{
////		LOG("delta = %d\n", delta);	
//		set_timer(AP_TIM1,20);            
//        g_ll_conn_ctx.current_timer = 20;                        // add 2020-03-30
//	}
//	else
//	{
//        set_timer(AP_TIM1,g_ll_conn_ctx.scheduleInfo[next].remainder - delta);
//
//	    // update connection context & schedule info
//        g_ll_conn_ctx.current_timer = g_ll_conn_ctx.scheduleInfo[next].remainder - delta;
//	}
//
//	g_ll_conn_ctx.currentConn = next;
//
//    // set ll state according to current connection LL state
//	if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_SLAVE)
//		llState = LL_STATE_CONN_SLAVE;
//	else if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_MASTER)
//		llState = LL_STATE_CONN_MASTER;
//
//    // the task is scheduled, set the priority as low
//	g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].priority = LL_SCH_PRIO_LOW;
//
//    // take into account the time between start timer1 and T1
//	for (i = 0; i < g_maxConnNum; i++)
//    {
//        if (conn_param[i].active)
//            g_ll_conn_ctx.scheduleInfo[i].remainder -= delta;
//    }	
//
////    LOG("<%x, %x> ", g_ll_conn_ctx.scheduleInfo[0].remainder, g_ll_conn_ctx.scheduleInfo[1].remainder);
////    LOG("<%x, %x, %x, %x> ", g_ll_conn_ctx.scheduleInfo[0].remainder, g_ll_conn_ctx.scheduleInfo[1].remainder,
////                             g_ll_conn_ctx.scheduleInfo[2].remainder, g_ll_conn_ctx.scheduleInfo[3].remainder);
//}


// get remainder adv time
uint32  read_ll_adv_remainder_time(void)
{
    uint32 currentCount;

	currentCount = AP_TIM4->CurrentCount;
	  
    currentCount = currentCount >> 2;    // convert to us
    
    return currentCount;
}

/*******************************************************************************
 * @fn          LL_processBasicIRQ
 *
 * @brief      Interrupt Request Handler for Link Layer
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None
 */
uint8 ll_processBasicIRQ(uint32_t      irq_status)
{
    uint8         mode;     
    uint32_t      T2, delay;
	llConnState_t *connPtr;
    connPtr = &conn_param[0];        // To update  
    
    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();   
            
    mode = ll_hw_get_tr_mode();   
//	LOG("%s,mode %d,llState %d\n",__func__,mode,llState);
    // ===================   mode TRX process 1
    if (mode == LL_HW_MODE_TRX  &&
          (llState == LL_STATE_ADV_UNDIRECTED ||
           llState == LL_STATE_ADV_SCAN       ||
           llState == LL_STATE_ADV_DIRECTED)
       )
    {  
        uint8_t  packet_len, pdu_type, txAdd;
		uint8_t  *peerAddr;
//		uint8_t  bWlRlCheckOk = TRUE;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;    
        int      calibra_time;                 // this parameter will be provided by global_config
        
//        ll_debug_output(DEBUG_LL_HW_TRX);
        
        // read packet
        if (irq_status & LIRQ_COK)
            packet_len = ll_hw_read_rfifo((uint8_t*)(&(g_rx_adv_buf.rxheader)), 
                                       &pktLen, 
                                       &pktFoot0, 
                                       &pktFoot1);
		else
			packet_len = 0;
		
        if(ll_hw_get_rfifo_depth()>0)
        {
            g_pmCounters.ll_rfifo_read_err++;
            packet_len=0;
            pktLen=0;
        }
        
        // check receive pdu type
        pdu_type = g_rx_adv_buf.rxheader & PDU_TYPE_MASK;
        txAdd    = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
        
        if (packet_len > 0                       // any better checking rule for rx anything?
          && pdu_type == ADV_SCAN_REQ
          && (llState == LL_STATE_ADV_UNDIRECTED 
              || llState == LL_STATE_ADV_SCAN))
        {                  
            // 1. scan req
            g_pmCounters.ll_recv_scan_req_cnt ++;
				
            // check AdvA
            if (g_rx_adv_buf.data[6]  != adv_param.ownAddr[0]
             || g_rx_adv_buf.data[7]  != adv_param.ownAddr[1] 
             || g_rx_adv_buf.data[8]  != adv_param.ownAddr[2]
             || g_rx_adv_buf.data[9]  != adv_param.ownAddr[3]
             || g_rx_adv_buf.data[10] != adv_param.ownAddr[4] 
             || g_rx_adv_buf.data[11] != adv_param.ownAddr[5])
            {              
            }
            else
            {
                peerAddr = &g_rx_adv_buf.data[0];      // ScanA
				{						
                    g_pmCounters.ll_rx_peer_cnt++;

                    uint8 retScanRspFilter=1;
                    if(LL_PLUS_ScanRequestFilterCBack)
                    {
                        retScanRspFilter = LL_PLUS_ScanRequestFilterCBack();
                    }

                    if(retScanRspFilter)
                    {
                        // send scan rsp
                        ll_hw_set_stx();             // set LL HW as single Tx mode  

                        g_same_rf_channel_flag = TRUE;
                        // calculate the delay
                        T2 = read_current_fine_time();
                    
                        delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                        calibra_time = pGlobal_config[SCAN_RSP_DELAY];            // consider rx_done to ISR time, SW delay after read_current_fine_time(), func read_current_fine_time() delay ...
                        delay = 118 - delay - calibra_time;                       // IFS = 150us, Tx tail -> Rx done time: about 32us
                    
                        ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK                  
                                         pGlobal_config[LL_HW_AFE_DELAY], 
                                         pGlobal_config[LL_HW_PLL_DELAY]);		  //RxAFE,PLL    
	
                        ll_hw_go();
                        llWaitingIrq = TRUE;
                        g_same_rf_channel_flag = FALSE;                        
                        
                        // reset Rx/Tx FIFO
                        ll_hw_rst_rfifo();
                        ll_hw_rst_tfifo(); 
// TODO
                        //write Tx FIFO
                        ll_hw_write_tfifo((uint8 *)&(tx_scanRsp_desc.txheader), 
                                       ((tx_scanRsp_desc.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
//                        ll_debug_output(DEBUG_LL_HW_SET_STX);
                        g_pmCounters.ll_send_scan_rsp_cnt ++;
                    }
                }
            }                    
        }
        else if ((packet_len > 0) && (pdu_type == ADV_CONN_REQ)
                && (llState == LL_STATE_ADV_UNDIRECTED 
                   || llState == LL_STATE_ADV_DIRECTED))
        {           // 2. connect req 
        // TODO
//            g_currentPeerAddrType = txAdd;

            
            g_pmCounters.ll_recv_conn_req_cnt ++;
            // check AdvA
            if (g_rx_adv_buf.data[6]  != adv_param.ownAddr[0]
             || g_rx_adv_buf.data[7]  != adv_param.ownAddr[1] 
             || g_rx_adv_buf.data[8]  != adv_param.ownAddr[2]
             || g_rx_adv_buf.data[9]  != adv_param.ownAddr[3]
             || g_rx_adv_buf.data[10] != adv_param.ownAddr[4] 
             || g_rx_adv_buf.data[11] != adv_param.ownAddr[5])
            {
                // nothing to do       
            }
            else
            {
//           		uint8_t  rpaListIndex = LL_RESOLVINGLIST_ENTRY_NUM;
                peerAddr = &g_rx_adv_buf.data[0];        // initA
							 
                uint8_t bWlRlCheckOk = TRUE;
                // fixed bug 2018-09-25, LL/CON/ADV/BV-04-C, for direct adv, initA should equal peer Addr
                if (llState == LL_STATE_ADV_DIRECTED)
                {
                    if (//txAdd         != peerInfo.peerAddrType    // for (extended) set adv param, peer addr type could only be 0x0 or 0x01
                      peerAddr[0]     != peerInfo.peerAddr[0]
                      || peerAddr[1]  != peerInfo.peerAddr[1] 
                      || peerAddr[2]  != peerInfo.peerAddr[2]
                      || peerAddr[3]  != peerInfo.peerAddr[3]
                      || peerAddr[4]  != peerInfo.peerAddr[4] 
                      || peerAddr[5]  != peerInfo.peerAddr[5])
                    {     // not match, check next
                        bWlRlCheckOk = FALSE;     
                    }                     
                }
                
                if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
                {
                    g_pmCounters.ll_filter_conn_req_cnt ++;
                }
                else
                {
                    // increment statistics counter
                    g_pmCounters.ll_rx_peer_cnt++;
                
	                // bug fixed 2018-01-23, peerAddrType should read TxAdd
                    peerInfo.peerAddrType = txAdd;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
			        osal_memcpy(peerInfo.peerAddr, &peerAddr[0], 6);

					move_to_slave_function();    // move to slave role for connection state 	
                }
            }
        }
        //test for fast adv
        else if(llState == LL_STATE_ADV_UNDIRECTED)
        {
            uint8_t firstAdvChan = (adv_param.advChanMap&LL_ADV_CHAN_37) !=0 ? 37 :
                                   (adv_param.advChanMap&LL_ADV_CHAN_38) !=0 ? 38 : 39;
                                   
            if(adv_param.advNextChan>firstAdvChan)
            {
                //llSetupUndirectedAdvEvt1();
                ll_schedule_next_event(50);         //20180623 modified by ZQ
                                                    // reset the timer1 instead of llSetupUndirectedAdvEvt1
                                                    // reduced the process time in LL_IRQ
                                                    // llSetupUndirectedAdvEvt1 will cost about 120us
                                                    
            }
        }   	
    }

    // =================== mode TRX process 2, for scanner, active scan case
    else if (mode == LL_HW_MODE_TRX  &&
             (llState == LL_STATE_SCAN))
    {
        // check whether receives SCAN RSP
//        ll_debug_output(DEBUG_LL_HW_TRX);

        llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

        // check whether receives SCAN RSP
        if (irq_status & LIRQ_COK)                        // bug correct 2018-10-15 
        {   // rx done
            uint8_t packet_len, pdu_type;
            uint16_t pktLen;
            uint32_t pktFoot0, pktFoot1;   

            // read packet
            packet_len = ll_hw_read_rfifo((uint8_t*)(&(g_rx_adv_buf.rxheader)), 
                                           &pktLen, 
                                           &pktFoot0, 
                                           &pktFoot1);
                        
            // check receive pdu type
            pdu_type = g_rx_adv_buf.rxheader & 0x0f;

            if(ll_hw_get_rfifo_depth()>0)
            {
                g_pmCounters.ll_rfifo_read_err++;
                packet_len=0;
                pktLen=0;
            }  
            
            if (packet_len > 0 && pdu_type == ADV_SCAN_RSP)
            {         // receives SCAN_RSP   
                uint8  advEventType;
				uint8  rpaListIndex;
				uint8 *peerAddr;
                uint8  addrType = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;
                uint8  dataLen  = pktLen - 8;
                int8   rssi     =  -(pktFoot1 >> 24);        
				uint8  bCheckOk = TRUE;

				peerAddr = &g_rx_adv_buf.data[0];

				//===
				// AdvA of SCAN_RSP should also be checked here. Refer to 4.4.3.2 Active Scanning
				// After sending a scan request PDU the Link Layer listens for a scan response
                //PDU from that advertiser. If the scan response PDU was not received from that
                //advertiser, it is considered a failure; otherwise it is considered a success. 
				
				// check AdvA in Scan Rsp is identical to Scan Req
				if (g_rx_adv_buf.data[0] != g_tx_adv_buf.data[6]  ||
				    g_rx_adv_buf.data[1] != g_tx_adv_buf.data[7]  ||
				    g_rx_adv_buf.data[2] != g_tx_adv_buf.data[8]  ||
				    g_rx_adv_buf.data[3] != g_tx_adv_buf.data[9]  ||
				    g_rx_adv_buf.data[4] != g_tx_adv_buf.data[10] ||
				    g_rx_adv_buf.data[5] != g_tx_adv_buf.data[11] 
					)
				    bCheckOk = FALSE;

				//===

                if (bCheckOk == TRUE)
                {
                    advEventType = LL_ADV_RPT_SCAN_RSP;
                    // below function cost 51us/66us(measure with GPIO)                        
                    LL_AdvReportCback( advEventType,                   // event type
                                 addrType,                             // Adv address type (TxAdd)
                                 peerAddr,                             // Adv address (AdvA)
                                 dataLen,                              // length of rest of the payload
                                 &g_rx_adv_buf.data[6],                // rest of payload
                                 rssi );                               // RSSI     
                    g_pmCounters.ll_recv_scan_rsp_cnt ++;
                
                    llAdjBoffUpperLimitSuccess();
                }

            }
            else
                llAdjBoffUpperLimitFailure();
        }        
        else
            llAdjBoffUpperLimitFailure();
        
        // update back off value according to new backoff upperLimit
        llGenerateNextBackoffCount();
        
        if (llScanTime >= scanInfo.scanWindow * 625)
        {
            // calculate next scan channel
            LL_CALC_NEXT_SCAN_CHN(scanInfo.nextScanChan);
                
            // schedule next scan event
            if (scanInfo.scanWindow == scanInfo.scanInterval)      // scanWindow == scanInterval, trigger immediately
                LL_evt_schedule();
            else
//                set_timer4((scanInfo.scanInterval - scanInfo.scanWindow) * 625);
			    ll_schedule_next_event((scanInfo.scanInterval - scanInfo.scanWindow) * 625);
                
            // reset scan total time
            llScanTime = 0;
        }
        else
            llSetupScan(scanInfo.nextScanChan);        
    }
    // =================== mode SRX process, for scan/init
    else if (mode == LL_HW_MODE_SRX
		 && (llState == LL_STATE_SCAN))
    {
//        ll_debug_output(DEBUG_LL_HW_SRX);
//		uint8_t  rpaListIndex = LL_RESOLVINGLIST_ENTRY_NUM;	
		uint8_t  bWlRlCheckOk = TRUE;
		uint8_t  *peerAddr;
        struct buf_tx_desc tx_scanreq_buf;
        
        // ============= scan case
        if (llState == LL_STATE_SCAN)
        {     
            uint8   bSendingScanReq = FALSE;            

            // check status
            if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))       // bug correct 2018-10-15     
            {   // rx done
                uint8_t  packet_len, pdu_type;
                uint16_t pktLen;
                uint32_t pktFoot0, pktFoot1;  				
                
                // read packet
                // cost 21-26us(measure with GPIO), depneds on the length of ADV
                packet_len = ll_hw_read_rfifo((uint8_t*)(&(g_rx_adv_buf.rxheader)), 
                                           &pktLen, 
                                           &pktFoot0, 
                                           &pktFoot1);            
        
                // check receive pdu type
                pdu_type = g_rx_adv_buf.rxheader & 0x0f;

                if(ll_hw_get_rfifo_depth()>0)
                {
                    g_pmCounters.ll_rfifo_read_err++;
                    packet_len=0;
                    pktLen=0;
                }                 
                
                if (packet_len   != 0 
                    && ((pdu_type == ADV_IND)
                    || (pdu_type  == ADV_NONCONN_IND)
                    || (pdu_type  == ADV_SCAN_IND)
                    || (pdu_type == ADV_DIRECT_IND)))
                {                                
					uint8   addrType;                  // peer address type
                    uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random

					peerAddr = &g_rx_adv_buf.data[0];		 // AdvA
					addrType = txAdd;
					
					// if(pdu_type == ADV_DIRECT_IND)    // direct adv only report addr & addr type match the whitelist
					// 	bWlRlCheckOk = FALSE;
                    
                    // if valid, trigger osal event to report adv 
                    if (bWlRlCheckOk == TRUE)   
                    {
                        uint8  advEventType;
                        int8   rssi;
                        
                        llCurrentScanChn = scanInfo.nextScanChan;

                        // active scan scenario, send scan req
                        if (scanInfo.scanType == LL_SCAN_ACTIVE
                            && (pdu_type== ADV_IND
                            || pdu_type == ADV_SCAN_IND ))     
                        {      
                            // back off process
                            scanInfo.currentBackoff = (scanInfo.currentBackoff > 0) ? (scanInfo.currentBackoff - 1) : 0;
                            if (scanInfo.currentBackoff == 0)      // back off value = 0, send scan req
                            {           
                                tx_scanreq_buf.txheader = 0xC03; 

                                //ZQ 20181012: add AdvFilterCB
                                uint8_t retAdvFilter = 1;
                                if(LL_PLUS_AdvDataFilterCBack)
                                {
                                    retAdvFilter = LL_PLUS_AdvDataFilterCBack();
                                }

                                if(retAdvFilter)
                                {
                                    g_same_rf_channel_flag = TRUE;
//                                    ll_hw_set_tx_rx_interval(10);
                                    ll_hw_set_rx_timeout(158);
									set_max_length(0xFF);                    // add 2020-03-10
                                    T2 = read_current_fine_time();
                                    delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                                    delay = 118 - delay - pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY];         
                                    ll_hw_set_trx();             // set LL HW as single TRx mode  
                                    ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK                  
                                         pGlobal_config[LL_HW_AFE_DELAY], 
                                         pGlobal_config[LL_HW_PLL_DELAY]);		  //RxAFE,PLL    

                                    
                                    ll_hw_go();   
                                    g_pmCounters.ll_send_scan_req_cnt++;
                                    llWaitingIrq = TRUE;
                                    
                                            
                                    // reset Rx/Tx FIFO
                                    ll_hw_rst_rfifo();
                                    ll_hw_rst_tfifo();
                                    
                                    ll_hw_ign_rfifo(LL_HW_IGN_CRC | LL_HW_IGN_EMP);
                                    
                                    // construct SCAN REQ packet
                                    //tx_scanreq_buf.txheader = 0xCC3;

                                    //20181012 ZQ: change the txheader according to the adtype
                                    tx_scanreq_buf.txheader |=(((g_rx_adv_buf.rxheader&0x40)<<1) 
                                                             | (scanInfo.ownAddrType<< TX_ADD_SHIFT & TX_ADD_MASK));
                                        //LL_ReadBDADDR(&tx_scanreq_buf.data[0]);
										memcpy((uint8 *)&tx_scanreq_buf.data[0], &scanInfo.ownAddr[0], 6);	
						
                                    // AdvA, for SCAN REQ, it should identical to the ADV_IND/ADV_SCAN_IND
                                    tx_scanreq_buf.data[6]  = peerAddr[0];
                                    tx_scanreq_buf.data[7]  = peerAddr[1];
                                    tx_scanreq_buf.data[8]  = peerAddr[2];
                                    tx_scanreq_buf.data[9]  = peerAddr[3];
                                    tx_scanreq_buf.data[10] = peerAddr[4];
                                    tx_scanreq_buf.data[11] = peerAddr[5];
                                    
                                    //write Tx FIFO
                                    ll_hw_write_tfifo((uint8 *)&(tx_scanreq_buf.txheader), 
                                               ((tx_scanreq_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                                    
                                    bSendingScanReq = TRUE;
                                    g_same_rf_channel_flag = FALSE;

                                }

                            }
                        }
  
                        // convert pdu type to GAP enum
                        switch (pdu_type)
                        {
                            case ADV_IND:
                                advEventType = LL_ADV_RPT_ADV_IND;
                                break;
                            case ADV_SCAN_IND:
                                advEventType = LL_ADV_RPT_ADV_SCANNABLE_IND;
                                break;
                            case ADV_DIRECT_IND:
                                advEventType = LL_ADV_RPT_ADV_DIRECT_IND;
                                break;            
                            case ADV_NONCONN_IND:
                                advEventType = LL_ADV_RPT_ADV_NONCONN_IND;
                                break;                                     
                            case ADV_SCAN_RSP:
                                advEventType = LL_ADV_RPT_INVALID;
                                break;
                            default:
                                advEventType = LL_ADV_RPT_ADV_IND;
                                break;
                        }              
                        rssi  =  -(pktFoot1 >> 24);                  


                        // below function cost 51us/66us(measure with GPIO)                        
                        LL_AdvReportCback( advEventType,                         // event type
                                           addrType,                                // Adv address type (TxAdd)
                                           &peerAddr[0],                         // Adv address (AdvA)
                                           pktLen - 8,                           // length of rest of the payload, 2 - header, 6 - advA
                                           &g_rx_adv_buf.data[6],                // rest of payload
                                           rssi );                               // RSSI          
                        g_pmCounters.ll_recv_adv_pkt_cnt ++;
                    }
                }
                else
                {
                    // invalid ADV PDU type
//                    llSetupScan();
                }
            }        

            // if not waiting for scan rsp, schedule next scan
            if (!bSendingScanReq)           
            {
                // not sending SCAN REQ, update scan time
                llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));
        
                if (llScanTime >= scanInfo.scanWindow * 625)
                {
                    
                    
                    // calculate next scan channel
                    LL_CALC_NEXT_SCAN_CHN(scanInfo.nextScanChan);
                
                    // schedule next scan event
                    if (scanInfo.scanWindow == scanInfo.scanInterval)      // scanWindow == scanInterval, trigger immediately
                        LL_evt_schedule();
                    else
//                        set_timer4((scanInfo.scanInterval - scanInfo.scanWindow) * 625);
					    ll_schedule_next_event((scanInfo.scanInterval - scanInfo.scanWindow) * 625);
                
                    // reset scan total time
                    llScanTime = 0;
                }
                else
                {
//                    AT_LOG("%03x %x %d %d %d %d\n",irq_status,*(volatile uint32_t *)(0x40031054),ll_hw_get_anchor(),
//                                                        g_rfifo_rst_cnt,(uint32_t)ISR_entry_time,read_current_fine_time());
              
                    llSetupScan(scanInfo.nextScanChan);                    
                }                  
            }                        
        }
        // ===========  initiator case
        
    }
    // =================== mode RTLP process
    else if (mode == LL_HW_MODE_RTLP 
              && llState == LL_STATE_CONN_SLAVE)
    {   // slave 
        uint8_t  ack_num, tx_num;          
//        uint32_t anchor_point, loop_time;
        
	    uint8 temp_sn_nesn;                        // 2018-2-28 add, for BQB     
		

		connPtr = &conn_param[0];
        
//        ll_debug_output(DEBUG_LL_HW_RTLP);
                      
        connPtr->rx_crcok = 0; 
        connPtr->rx_timeout = 0;
        
        // read loop timeout counter, system clock may be 16MHz, 32MHz, 64MHz and 48MHz, 96MHz
//        if (hclk_per_us_shift != 0)
//            loop_time = ll_hw_get_loop_cycle() >> hclk_per_us_shift;      // convert to us
//        else
//            loop_time = ll_hw_get_loop_cycle() / hclk_per_us;             // convert to us
        
        // read anchor point
        if (irq_status & LIRQ_TD)   // sync OK, then anchor point is OK, whether received something or CRC error => something will be Tx and done
        {
//            if (hclk_per_us_shift != 0)
//                anchor_point = ll_hw_get_anchor() >> hclk_per_us_shift;      // convert to us
//            else
//                anchor_point = ll_hw_get_anchor() / hclk_per_us;      // convert to us
            
            if (connPtr->firstPacket)                  // anchor point catched, change state
            {
                connPtr->firstPacket = 0;
            }            

            //20180715 by ZQ
            //get the rssi form rf_phy register
            rf_phy_get_pktFoot(&connPtr->lastRssi, &connPtr->foff, &connPtr->carrSens);            

            // global_config[SLAVE_CONN_DELAY]: soft parameter could be set after release
//            slave_conn_event_recv_delay = loop_time - anchor_point + pGlobal_config[SLAVE_CONN_DELAY];     
            
            if (irq_status & LIRQ_COK)     // Rx CRC OK, empty of data packet
            {
                connPtr->rx_crcok = 1;
            }           
        }
        else 
        {    // timeout case            
            connPtr->rx_timeout = 1;
            
            connPtr->pmCounter.ll_conn_event_timeout_cnt ++;
            // pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC]: soft parameter could be set after release
//            slave_conn_event_recv_delay = loop_time - 270 + pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC];   // refer to llSlaveEvt_TaskEndOk(), we use the same formula for timeout & sync case

            //20181014 ZQ: perStats
            if(p_perStatsByChan!=NULL)
                p_perStatsByChan->rxToCnt[connPtr->currentChan]++;
        }

        //====== 20180324 modified by ZQ
//        ll_adptive_smart_window(irq_status,anchor_point);      // comment-out by HZF
                
        ack_num = ll_hw_get_txAck();   
        rfCounters.numTxDone = ack_num;    
        // below 2 counters are update in  ll_hw_update()       
        //rfCounters.numTxAck = ack_num;         // not align to TI, ACK for empty packet is not considered
        //rfCounters.numRxOk = ll_hw_get_rxPkt_num(); 
        
        // TODO: update more Rf counters
        rfCounters.numRxNotOk    = 0;
        rfCounters.numRxIgnored  = 0;
        rfCounters.numRxEmpty    = 0;
        rfCounters.numRxFifoFull = 0;
        
        // A1 ROM metal change add
        if ((irq_status & LIRQ_CERR2)
         || (irq_status & LIRQ_CERR ))
        {
            connPtr->pmCounter.ll_recv_crcerr_event_cnt ++;
            rfCounters.numRxNotOk = 1;                  // CRC error, add 2018-6-12
        }
         
        //rfCounters.numTxRetrans = 0;             
        //rfCounters.numTx = 0;                                            
        //rfCounters.numRxCtrl = 0;   
        
        temp_sn_nesn = connPtr->sn_nesn;     // 2018-2-28, BQB
       
        // update the LL HW engine mode and save the sn, nesn
        connPtr->llMode = ll_hw_update(connPtr->llMode,         // Attention: this mode is not real HW mode 
                                            &(rfCounters.numTxAck), 
                                            &(rfCounters.numRxOk), 
                                            &(connPtr->sn_nesn));     
        
	    // add 2018-2-28, for slave latency scenario in BQB test
		if (connPtr->firstPacket == 0      // anchor point catched
			 && (temp_sn_nesn & 0x02) != (connPtr->sn_nesn & 0x02)  // local sn has changed
		     && rfCounters.numTxAck == 0)
		{
		   rfCounters.numTxAck = 1;
		}        
                                            
        // ==== check HW Tx FIFO, read packets which not transmit or transit but not receive ACK
        tx_num = (*(volatile uint32_t *)(LL_HW_BASE + 0x04) >> 8) & 0xff;
				
        if ( connPtr->encEnabled )
            g_pmCounters.ll_tbd_cnt2 += ack_num;


        
        //20200128 ZQ: perStats
        if(p_perStatsByChan!=NULL)
        {
            p_perStatsByChan->TxNumAck[connPtr->currentChan]+=ack_num;
            p_perStatsByChan->txNumRetry[connPtr->currentChan]+=ll_hw_get_nAck();

            uint8_t crcErrNum,rxTotalNum,rxPktNum;
            ll_hw_get_rxPkt_stats(&crcErrNum,&rxTotalNum,&rxPktNum);
            p_perStatsByChan->rxNumPkts[connPtr->currentChan]+=rxTotalNum;
            p_perStatsByChan->rxNumCrcErr[connPtr->currentChan]+=crcErrNum; 
        }

 
        
 
        if (irq_status & LIRQ_RTO
            && ack_num < tx_num)   // receive time out, there are bugs in LL HW process TFIFO pointer, recover the pointer by SW
        {
            ll_hw_process_RTO(ack_num);
        }            
        ll_hw_read_tfifo_rtlp();
        
        // update the numTxCtrlAck counter, add on 2017-11-15
        rfCounters.numTxCtrlAck = 0;
        if (connPtr->ctrlDataIsProcess == 1              // control packet in this RTLP event TFIFO
          && ack_num > 0                        // get ACK in this event
          && (connPtr->ll_buf.tx_not_ack_pkt->valid == 0  // no not_ack packet
               || (connPtr->ll_buf.tx_not_ack_pkt->header & 0x3) != LL_DATA_PDU_HDR_LLID_CONTROL_PKT))  // not_ack packet is not ctrl packet
        {
            rfCounters.numTxCtrlAck = 1;
            connPtr->ctrlDataIsProcess = 0;
        }            
        
        // if receive some packets, read them
        if (rfCounters.numRxOk)  
        {
            // read HW Rx FIFO to internal buffer
            ll_read_rxfifo();
        }      
   
        // call llSlaveEvt_TaskEndOk  
        llSlaveEvt_TaskEndOk();
        
        // connection event notify
        if (g_conn_taskID != 0)
            osal_set_event(g_conn_taskID, g_conn_taskEvent);	

    }
    // =================== mode TRLP process
    
    // =================== other mode(STX, RTX), no process(send SCAN RSP done) or no used
    // === A2 add for simultaneous connect event & adv event
    // conn-adv case 1: STX ISR, continue broadcast left sec adv channels
    else if ((llSecondaryState == LL_SEC_STATE_ADV || llSecondaryState == LL_SEC_STATE_IDLE_PENDING)
		  && (mode == LL_HW_MODE_STX ))
    {    // secondary adv state
        uint8 i;
        i = 0;
        while (!(adv_param.advChanMap & (1 << i)))   i ++;    // get the 1st adv channel

        // adv_param.advNextChan stores the next adv channel, when adv the last adv channel, advNextChan should equal 1st adv channel
        if (adv_param.advNextChan != (LL_ADV_CHAN_FIRST + i))           // not finish adv the last channel, continue adv 
        {
	        llSetupSecAdvEvt();
        }
		else
		{
			if (llSecondaryState == LL_SEC_STATE_IDLE_PENDING)         // advertise last channel and transiting to IDLE
  			    llSecondaryState = LL_SEC_STATE_IDLE;
			else                                                       // otherwise, schedule next adv
				osal_start_timerEx(LL_TaskID, LL_EVT_SECONDARY_ADV, (adv_param.advInterval * 5) >> 3);   // * 625 / 1000
		}
//        ll_debug_output(DEBUG_LL_HW_STX);		
    }
    // multi-connection, support connectable/scannable adv
    else if ((llSecondaryState == LL_SEC_STATE_ADV || llSecondaryState == LL_SEC_STATE_IDLE_PENDING)
		  && (mode == LL_HW_MODE_TRX )
		  && (adv_param.advEvtType == LL_ADV_CONNECTABLE_UNDIRECTED_EVT || adv_param.advEvtType == LL_ADV_SCANNABLE_UNDIRECTED_EVT))
    {     // secondary adv state, connectable adv or scannable adv
        uint8_t  packet_len, pdu_type;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;    
        int      calibra_time;                 // this parameter will be provided by global_config
        
////        ll_debug_output(DEBUG_LL_HW_TRX);
        
        // read packet
        packet_len = ll_hw_read_rfifo((uint8_t*)(&(g_rx_adv_buf.rxheader)), 
                                       &pktLen, 
                                       &pktFoot0, 
                                       &pktFoot1);
        if(ll_hw_get_rfifo_depth() > 0)
        {
            g_pmCounters.ll_rfifo_read_err++;
            packet_len=0;
            pktLen=0;
        }
        
        // check receive pdu type
        pdu_type = g_rx_adv_buf.rxheader & PDU_TYPE_MASK;
//        txAdd    = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
        
        if (packet_len > 0                       // any better checking rule for rx anything?
  		  && (irq_status & LIRQ_COK) 
          && pdu_type == ADV_SCAN_REQ)
//          && (llState == LL_STATE_ADV_UNDIRECTED 
//              || llState == LL_STATE_ADV_SCAN))
        {                   
            // 1. scan req
            g_pmCounters.ll_recv_scan_req_cnt ++;
            // check AdvA
//            if (g_rx_adv_buf.data[6]  != adv_param.ownAddr[0]
//             || g_rx_adv_buf.data[7]  != adv_param.ownAddr[1] 
//             || g_rx_adv_buf.data[8]  != adv_param.ownAddr[2]
//             || g_rx_adv_buf.data[9]  != adv_param.ownAddr[3]
//             || g_rx_adv_buf.data[10] != adv_param.ownAddr[4] 
//             || g_rx_adv_buf.data[11] != adv_param.ownAddr[5])
//            {        
            	// TODO : check addr not fetch case ?
//            }
//            else
            {
//===
//           		uint8_t  rpaListIndex, bWlRlCheckOk;
                uint8_t *peerAddr = &g_rx_adv_buf.data[0];      // ScanA
				                

//                if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
//                {
//                    g_pmCounters.ll_filter_scan_req_cnt ++;
//                }
//                else
                {								
                    g_pmCounters.ll_rx_peer_cnt++;

                    uint8 retScanRspFilter=1;
                    if(LL_PLUS_ScanRequestFilterCBack)
                    {
                        retScanRspFilter = LL_PLUS_ScanRequestFilterCBack();
                    }

                    if(retScanRspFilter)
                    {
                        // send scan rsp
                        ll_hw_set_stx();             // set LL HW as single Tx mode  

                        g_same_rf_channel_flag = TRUE;
                        // calculate the delay
                        T2 = read_current_fine_time();
                    
                        delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                        calibra_time = pGlobal_config[SCAN_RSP_DELAY];            // consider rx_done to ISR time, SW delay after read_current_fine_time(), func read_current_fine_time() delay ...
                        delay = 118 - delay - calibra_time;                       // IFS = 150us, Tx tail -> Rx done time: about 32us
                    
                        ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK                  
                                         pGlobal_config[LL_HW_AFE_DELAY], 
                                         pGlobal_config[LL_HW_PLL_DELAY]);		  //RxAFE,PLL    

                        ll_hw_go();
                        llWaitingIrq = TRUE;
                        g_same_rf_channel_flag = FALSE;                        
                        
                        // reset Rx/Tx FIFO
                        ll_hw_rst_rfifo();
                        ll_hw_rst_tfifo(); 

                        //write Tx FIFO
                        ll_hw_write_tfifo((uint8 *)&(tx_scanRsp_desc.txheader), 
                                       ((tx_scanRsp_desc.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
//                        ll_debug_output(DEBUG_LL_HW_SET_STX);
                        g_pmCounters.ll_send_scan_rsp_cnt ++;
                    }
                }
            }                    				
        }
        //test for fast adv
        else //if(llState == LL_STATE_ADV_UNDIRECTED)
        {     // adv in next channel, or schedule next adv event
            uint8 i = 0;
					
            while (!(adv_param.advChanMap & (1 << i)))	 i ++;	  // get the 1st adv channel
			
            // adv_param.advNextChan stores the next adv channel, when adv the last adv channel, advNextChan should equal 1st adv channel
            if (adv_param.advNextChan != (LL_ADV_CHAN_FIRST + i))			// not finish adv the last channel, continue adv 
            {
                llSetupSecAdvEvt();
            }
            else
            {
                if (llSecondaryState == LL_SEC_STATE_IDLE_PENDING)		   // advertise last channel and transiting to IDLE
                    llSecondaryState = LL_SEC_STATE_IDLE;
                else													   // otherwise, schedule next adv
                    osal_start_timerEx(LL_TaskID, LL_EVT_SECONDARY_ADV, (adv_param.advInterval * 5) >> 3);	 // * 625 / 1000
            }			
						        }   	

	
    }
    // TODO: add RPA list checking, identical to single role case
    else if (mode == LL_HW_MODE_SRX
          && llSecondaryState == LL_SEC_STATE_SCAN)
    {     
        // check status
        if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))       // bug correct 2018-10-15     
        {   // rx done
            uint8_t packet_len, pdu_type;
            uint16_t pktLen;
            uint32_t pktFoot0, pktFoot1;  
            // read packet
            // cost 21-26us(measure with GPIO), depneds on the length of ADV
            packet_len = ll_hw_read_rfifo((uint8_t*)(&(g_rx_adv_buf.rxheader)), 
                                           &pktLen, 
                                           &pktFoot0, 
                                           &pktFoot1);            
        
            // check receive pdu type
            pdu_type = g_rx_adv_buf.rxheader & 0x0f;
            
			if (packet_len	 != 0 
				&& ((pdu_type == ADV_IND)
				|| (pdu_type  == ADV_NONCONN_IND)
				|| (pdu_type  == ADV_SCAN_IND)))
            {                                
                int     i = 0;
                uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
        
                    
                // if valid, trigger osal event to report adv 
//                if (i < LL_WHITELIST_ENTRY_NUM)   
                {
                    uint8  advEventType;
                    int8   rssi;
                        
                    llCurrentScanChn = scanInfo.nextScanChan;

                    // no active scan scenario
  
					// convert pdu type to GAP enum
					switch (pdu_type)
					{
						case ADV_IND:
							advEventType = LL_ADV_RPT_ADV_IND;
							break;
						case ADV_SCAN_IND:
							advEventType = LL_ADV_RPT_ADV_SCANNABLE_IND;
							break;
						case ADV_DIRECT_IND:
							advEventType = LL_ADV_RPT_ADV_DIRECT_IND;
							break;			  
						case ADV_NONCONN_IND:
							advEventType = LL_ADV_RPT_ADV_NONCONN_IND;
							break;									   
						case ADV_SCAN_RSP:
							advEventType = LL_ADV_RPT_INVALID;
							break;
						default:
							advEventType = LL_ADV_RPT_ADV_IND;
							break;
					}		

						
                    rssi  =  -(pktFoot1 >> 24);                         
                    // below function cost 51us/66us(measure with GPIO)                        
                    LL_AdvReportCback( advEventType,                         // event type
                                       txAdd,                                // Adv address type (TxAdd)
                                       &g_rx_adv_buf.data[0],       // Adv address (AdvA)
                                       pktLen - 8,                           // length of rest of the payload, 2 - header, 6 - advA
                                       &g_rx_adv_buf.data[6],       // rest of payload
                                       rssi );                               // RSSI       
                    g_pmCounters.ll_recv_adv_pkt_cnt ++;
                }
            }
        }        

        //  update scan time
        llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));
        if (llScanTime >= scanInfo.scanWindow * 625)
        {   // switch scan channel, set event instead of trigger immediately
            // calculate next scan channel
            LL_CALC_NEXT_SCAN_CHN(scanInfo.nextScanChan);
                
            // schedule next scan event
            if (scanInfo.scanWindow == scanInfo.scanInterval)      // scanWindow == scanInterval, trigger immediately
                osal_set_event(LL_TaskID, LL_EVT_SECONDARY_SCAN);
            else
                osal_start_timerEx(LL_TaskID, LL_EVT_SECONDARY_SCAN, ((scanInfo.scanInterval - scanInfo.scanWindow) * 5) >> 3 );
                
            // reset scan total time
            llScanTime = 0;
        }
        else if (llSecondaryState == LL_SEC_STATE_SCAN)
            llSetupSecScan(scanInfo.nextScanChan);
   	}
    // ======= A2 multi-connection
    // TODO: add RPA list checking, identical to single role case
   
		  
    // post ISR process   
    if (!llWaitingIrq)                      // bug fixed 2018-05-04, only clear IRQ status when no config new one
        ll_hw_clr_irq();
    
    HAL_EXIT_CRITICAL_SECTION();
    
	return TRUE;
}


//============



