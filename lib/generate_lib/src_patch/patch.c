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

/**************************************************************************************************
  Filename:       patch.c
  Revised:         
  Revision:        
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "clock.h"
#include "bus_dev.h"
#include "OSAL_Tasks.h"
#include "rom_sym_def.h"
#include "global_config.h"
#include "jump_function.h"
#include "ll_hw_drv.h"

/*********************************************************************
 * MACROS
 */
#define PATCH_DEBUG				TRUE

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32 global_config[];
extern uint32_t  __initial_sp;
extern uint32_t  g_smartWindowSize	;
extern volatile sysclk_t       g_system_clk;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint32* pGlobal_config;
#if( defined( PATCH_DEBUG ) && PATCH_DEBUG )
//ab8967452301
const uint32 s_macaddr[2];
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void __wdt_init(void)
{
    typedef void (*my_function)(void );
    my_function pFunc = NULL;
    pFunc = (my_function)(JUMP_FUNCTION_GET(HAL_WATCHDOG_INIT));

    if (pFunc != ((my_function)SRAM_BASE_ADDR))
        pFunc();
}

void wakeup_init1()
{
    __wdt_init();
    wakeup_init0();
}

void ll_hw_go1(void)
{
	ll_hw_go0();
	gpio_write(P23, 1);
	gpio_write(P23, 0);
}

#define LL_HW_FIFO_MARGIN_SMALL (10)
uint8_t  ll_hw_write_tfifo1(uint8_t* txPkt, uint16_t pktLen)
{
	int rdPtr,wrPtr,wrDepth,wlen;
	uint32_t *p_txPkt=(uint32_t *)txPkt;

	ll_hw_get_tfifo_info(&rdPtr,&wrPtr,&wrDepth);
	uint16_t tfifoSpace  = 0x07ff & (*(uint32_t *) (LL_HW_BASE + 0x74)); 

    //LL_FIFO_SPACE is 512B for tx, need adjust the LL_HW_FIFO_MARGIN
	if((pktLen>0) && (wrDepth+(pktLen>>2)<tfifoSpace-LL_HW_FIFO_MARGIN_SMALL)){	// make sure write the longest pkt will not overflow
		
		wlen = 1+((pktLen-1)>>2);						// calc the write tfifo count 

		//--------------------------------------------------------------
		//write tfifo wlen-1 firstly
		while(p_txPkt<((uint32_t *)txPkt+wlen-1)){
			*(volatile uint32_t *)(LL_HW_TFIFO) = *p_txPkt++;
		}

		//--------------------------------------------------------------
		//calc the residue txPkt length
		//write tfifo last time 
		int rduLen = pktLen&0x03;//pktLen%4
		
		if(			rduLen==3){
			*(volatile uint32_t *)(LL_HW_TFIFO) = *(uint16_t *)(txPkt+pktLen-3) | (txPkt[pktLen-1]<<16) ;
		}else if(	rduLen==2){
			*(volatile uint32_t *)(LL_HW_TFIFO) = *(uint16_t *)(txPkt+pktLen-2);
		}else if(	rduLen==1){
			*(volatile uint32_t *)(LL_HW_TFIFO) = *(txPkt+pktLen-1);
		}else{
			*(volatile uint32_t *)(LL_HW_TFIFO) = *p_txPkt;	
		}
		
		
		return wlen;

	}else{

		return 0;
	}
}

uint16 ll_generateTxBuffer1(int txFifo_vacancy, uint16 *pSave_ptr)
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
        ll_hw_write_tfifo1((uint8 *)&(connPtr->ll_buf.tx_not_ack_pkt->header), ((connPtr->ll_buf.tx_not_ack_pkt->header & 0xff00) >> 8) + 2);  
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
			ll_hw_write_tfifo1((uint8 *)&(connPtr->ll_buf.tx_ntrm_pkts[i]->header), ((connPtr->ll_buf.tx_ntrm_pkts[i]->header & 0xff00) >> 8) + 2);						
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
        ll_hw_write_tfifo1((uint8 *)&(connPtr->ctrlData .header), ((connPtr->ctrlData .header & 0xff00) >> 8) + 2);   
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
            ll_hw_write_tfifo1((uint8 *)&(connPtr->ll_buf.tx_conn_desc[idx]->header), ((connPtr->ll_buf.tx_conn_desc[idx]->header & 0xff00) >> 8) + 2); 
            
            update_tx_read_ptr(connPtr);  
            tx_num++;            
            
            // update PM counter, add A1 ROM metal change
            connPtr->pmCounter.ll_send_data_pkt_cnt ++;
        }        
    }   
	
    return tx_num;
}





/*********************************************************************
 * @brief   initiliaztion configuration 
 *
 * for osal scheduler and Bluetooth Controller
 */
void init_config(void)
{
	int i;

	pGlobal_config = global_config;

	for (i = 0; i < SOFT_PARAMETER_NUM; i ++)
		pGlobal_config[i] = 0;

	//save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
	pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;

	// LL switch setting
	pGlobal_config[LL_SWITCH] =  LL_DEBUG_ALLOW | SLAVE_LATENCY_ALLOW | RC32_TRACKINK_ALLOW
												| SIMUL_CONN_ADV_ALLOW | SIMUL_CONN_SCAN_ALLOW 
												| GAP_DUP_RPT_FILTER_DISALLOW;



	// sleep delay
	pGlobal_config[MIN_TIME_TO_STABLE_32KHZ_XOSC] = 10;      // 10ms, temporary set

	// system clock setting
	pGlobal_config[CLOCK_SETTING] = g_system_clk;//CLOCK_32MHZ;

	//------------------------------------------------------------------------
	// wakeup time cose
	// t1. HW_Wakeup->MCU relase 62.5us
	// t2. wakeup_process in waitRTCCounter 30.5us*[WAKEUP_DELAY] about 500us
	// t3. dll_en -> hclk_sel in hal_system_ini 100us in run as RC32M
	// t4. sw prepare cal sleep tick initial rf_ini about 300us @16M this part depends on HCLK
	// WAKEUP_ADVANCE should be larger than t1+t2+t3+t4 
	//------------------------------------------------------------------------
	// wakeup advance time, in us

	pGlobal_config[WAKEUP_ADVANCE] = 1850;//650;//600;//310;

	if(g_system_clk==SYS_CLK_XTAL_16M)
	{
		pGlobal_config[WAKEUP_DELAY] = 16;
	}
	else if(g_system_clk==SYS_CLK_DBL_32M)
	{
		pGlobal_config[WAKEUP_DELAY] = 18;
	}
	else if(g_system_clk==SYS_CLK_DLL_48M)
	{
		pGlobal_config[WAKEUP_DELAY] = 20;
	}
	else if(g_system_clk==SYS_CLK_DLL_64M)
	{
		pGlobal_config[WAKEUP_DELAY] = 24;
	}

	// sleep time, in us
	pGlobal_config[MAX_SLEEP_TIME] = 1500000;    
	pGlobal_config[MIN_SLEEP_TIME] = 1500;

	pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K] = 60;// 30.5 per tick

	// LL engine settle time
	pGlobal_config[LL_HW_BB_DELAY] = 54;//54-8;
	pGlobal_config[LL_HW_AFE_DELAY] = 8;
	pGlobal_config[LL_HW_PLL_DELAY] = 40;

	// Tx2Rx and Rx2Tx interval
	//Tx2Rx could be advanced a little
	//Rx2Tx should be ensure T_IFS within150us+-2us
	pGlobal_config[LL_HW_Rx_TO_TX_INTV] = 58;//57;			// 2019/3/20 A2: 57 --> 58
	pGlobal_config[LL_HW_Tx_TO_RX_INTV] = 49;//50; //65		// 2019/3/20 A2: 50 --> 49

	//------------------------------------------------2MPHY
	// LL engine settle time 
	pGlobal_config[LL_HW_BB_DELAY_2MPHY] = 59;
	pGlobal_config[LL_HW_AFE_DELAY_2MPHY] = 8;
	pGlobal_config[LL_HW_PLL_DELAY_2MPHY] = 40;
	// Tx2Rx and Rx2Tx interval
	//Tx2Rx could be advanced a little
	//Rx2Tx should be ensure T_IFS within150us+-2us
	pGlobal_config[LL_HW_Rx_TO_TX_INTV_2MPHY] = 70;//72
	pGlobal_config[LL_HW_Tx_TO_RX_INTV_2MPHY] = 57;//72   


	// LL engine settle time, for advertisement
	pGlobal_config[LL_HW_BB_DELAY_ADV] = 90;
	pGlobal_config[LL_HW_AFE_DELAY_ADV] = 8;
	pGlobal_config[LL_HW_PLL_DELAY_ADV] = 60;    

	// adv channel interval
	pGlobal_config[ADV_CHANNEL_INTERVAL] = 1400;//6250;
	pGlobal_config[NON_ADV_CHANNEL_INTERVAL] = 1400;

	if(g_system_clk==SYS_CLK_XTAL_16M)
	{
		// scan req -> scan rsp timing
		pGlobal_config[SCAN_RSP_DELAY] = 16;//23;        //  2019/3/19 A2: 23 --> 16
	}
	else if(g_system_clk==SYS_CLK_DBL_32M)
	{
		// scan req -> scan rsp timing
		pGlobal_config[SCAN_RSP_DELAY] = 12;        // 12	//  2019/3/19 A2: 12 --> 9
	} 
	else if(g_system_clk==SYS_CLK_DLL_48M)
	{
		// scan req -> scan rsp timing
		pGlobal_config[SCAN_RSP_DELAY] = 9;        // 12	//  2019/3/19 A2: 12 --> 9
	}                                      
	else if(g_system_clk == SYS_CLK_DLL_64M)		//  2019/3/26 add
	{
		pGlobal_config[SCAN_RSP_DELAY] = 8;
	}

	// conn_req -> slave connection event calibration time, will advance the receive window
	pGlobal_config[CONN_REQ_TO_SLAVE_DELAY] = 300;//192;//500;//192;

	// calibration time for 2 connection event, will advance the next conn event receive window
	// SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
	pGlobal_config[SLAVE_CONN_DELAY] = 143;//0;//1500;//0;//3000;//0;          ---> update 11-20
	pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 160;

	// RTLP timeout
	pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] = 50000;
	pGlobal_config[LL_HW_RTLP_TO_GAP]       = 1000;

	pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]  = 2000;
	pGlobal_config[LL_FIRST_WINDOW]         = 5000; //jack rom add for slave first connet evt


	// direct adv interval configuration
	pGlobal_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
	pGlobal_config[LDC_DIRECT_ADV_INTERVAL] = 6250;

	// A1 ROM metal change for HDC direct adv, 
	pGlobal_config[DIR_ADV_DELAY] = 115;   // in us, consider both direct adv broadcast time & SW delay, ... etc.

	// A1 ROM metal change
	pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 6;//8;
	pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 6;//8;    

	pGlobal_config[LL_TRX_NUM_ADAPTIVE_CONFIG] = 8;     //0:        disable adaptive 
	//other:    adaptive max limitation
	g_smartWindowSize = pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] ;    

	//====== A2 metal change add, for scanner & initiator
	if(g_system_clk==SYS_CLK_XTAL_16M)
	{
		pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY] = 18;//20;		//  2019/3/19 A2: 20 --> 18
	}
	else if(g_system_clk==SYS_CLK_DLL_48M)
	{
		pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY] = 10;//12;		//  2019/3/19 A2: 12 --> 10
	}
	else if(g_system_clk==SYS_CLK_DLL_64M)
	{
		pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY] = 8;				//  2019/3/26 add
	}

	// TRLP timeout
	pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM] = 56;  // 0.125us

    pGlobal_config[MAC_ADDRESS_LOC] = 0x11004000;
	// for simultaneous conn & adv/scan
	pGlobal_config[LL_NOCONN_ADV_EST_TIME] = 1400*3;
	pGlobal_config[LL_NOCONN_ADV_MARGIN] = 600;

	pGlobal_config[LL_SEC_SCAN_MARGIN] = 1400;
	pGlobal_config[LL_MIN_SCAN_TIME] = 2000;

	pGlobal_config[TIMER_ISR_ENTRY_TIME] = 30;
	pGlobal_config[NEXT_TIMER1_CONSIDER_DELAY] = 300;
	pGlobal_config[LL_WRITE_RE_TX_FIFO] = 50;

	extern void osalInitTasks( void );
	extern pTaskEventHandlerFn tasksArr[];
	extern uint16 tasksCnt;
	extern uint16 *tasksEvents;
	JUMP_FUNCTION_SET(OSAL_INIT_TASKS,(uint32_t*)osalInitTasks);
	JUMP_FUNCTION_SET(TASKS_ARRAY,(uint32_t*)tasksArr);
	JUMP_FUNCTION_SET(TASK_COUNT ,(uint32_t)&tasksCnt);
	JUMP_FUNCTION_SET(TASK_EVENTS,(uint32_t)&tasksEvents);

	//JUMP_FUNCTION_SET(LL_HW_GO , (uint32_t)&ll_hw_go1);
	extern void rf_phy_ini1(void);
    	extern void rf_phy_change_cfg1(uint8_t pktFmt);
    	extern uint16 ll_generateTxBuffer1(int txFifo_vacancy, uint16 *pSave_ptr);
	JUMP_FUNCTION_SET(RF_INIT , (uint32_t)&rf_phy_ini1);
	JUMP_FUNCTION_SET(RF_PHY_CHANGE , (uint32_t)&rf_phy_change_cfg1);
	JUMP_FUNCTION_SET(LL_GENERATE_TX_BUFFER , (uint32_t)&ll_generateTxBuffer1);

	JUMP_FUNCTION_SET(LL_GENERATE_TX_BUFFER , (uint32_t)&ll_generateTxBuffer1);

    	JUMP_FUNCTION_SET(WAKEUP_INIT , (uint32_t)&wakeup_init1);

	//__set_MSP(pGlobal_config[INITIAL_STACK_PTR]);
	
}

void ll_set_ble_mac_addr(uint32_t macAddr)
{
    pGlobal_config[MAC_ADDRESS_LOC] = macAddr;
}

extern void ble_main(void);
void hal_rom_boot_init(void )
{
	ble_main();

}

