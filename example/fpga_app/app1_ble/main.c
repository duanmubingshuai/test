#include "rom_sym_def.h"

//#include <stdio.h>
#include "bus_dev.h"
#include "uart.h"

#include "gpio.h"
#include "clock.h"
#include "timer.h"
#include "ll.h"
#include "simpleBLEperipheral.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"

#include "ll_sleep.h"
#include "ll.h"
#include "ll_buf.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "pwrmgr.h"
#include "mcu.h"
#include "OSAL_Tasks.h"
#include "gpio.h"
#include "log.h"

#define WFI()   __WFI();

typedef void (*my_function);

//uint32 *pGlobal_config = (uint32 *)(CONFIG_BASE_ADDR);
int  main(void);
void  ble_main(void);

extern void osalInitTasks(void);
extern int app_main(void);

extern void disableSleep(void);
extern void enableSleep(void);


extern uint32_t get_timer3_count(void);	

void wakeup_init(void);
void boot_init(void);
void rf_calibrate1(void);
///////////////   from ROM
//uint32 *pGlobal_config = NULL;
extern uint32 hclk_per_us;
extern uint32 hclk_per_us_shift;

/////////////////////////

extern uint32 sleep_flag;
extern uint32 osal_sys_tick;
extern uint32 ll_remain_time;

extern uint32 llWaitingIrq;
extern uint32 ISR_entry_time;

extern uint32 counter_tracking;

extern uint32_t  __initial_sp;
extern struct buf_rx_desc g_rx_adv_buf;
//extern uint8 simpleBLEPeripheral_TaskID; 


//uint8   g_phyRssi       = 0;        
//uint8   g_phyGidx       = 0;        
//uint8   g_phyCarrSens   = 0;        
//uint16  g_phyFoff       = 0;

//uint16  g_phyCrcErrNum  = 0;
//uint16  g_phyRxTONum    = 0;
//uint16  g_phyRxPktNum   = 0;

//volatile uint32  g_llRxPkt   = 0;
//volatile uint32  g_llIrq     = 0;
//volatile uint8   g_rxTo      = 0;
//volatile uint16  g_anchPoint = 0;
//volatile uint8   g_phyRxPkt  = 0; 
//volatile uint8   g_phyCrcErr = 0;
//volatile uint32  g_pktFoot0  = 0;
//volatile uint32  g_pktFoot1  = 0;

volatile uint8 g_clk32K_config;

extern unsigned char urx_buf[];

extern volatile unsigned int uart_rx_wIdx;
/////////////////////////////

// ===================== connection context relate definition

#define   BLE_MAX_ALLOW_CONNECTION              3
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_TX        1
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_RX        1

#define   BLE_PKT_VERSION                       BLE_PKT_VERSION_4_0 //BLE_PKT_VERSION_5_1 //BLE_PKT_VERSION_5_1     

#define   BLE_PKT_BUF_SIZE                  (((BLE_PKT_VERSION == BLE_PKT_VERSION_4_0) ? 1 : 0) * BLE_PKT40_LEN \
	                                        + (sizeof(struct ll_pkt_desc) - 2))

/* BLE_MAX_ALLOW_PER_CONNECTION 
{
    ...
    struct ll_pkt_desc *tx_conn_desc[MAX_LL_BUF_LEN];     // new Tx data buffer
    struct ll_pkt_desc *rx_conn_desc[MAX_LL_BUF_LEN];

    struct ll_pkt_desc *tx_not_ack_pkt;
    struct ll_pkt_desc *tx_ntrm_pkts[MAX_LL_BUF_LEN];  
    ...
 }   
 tx_conn_desc[] + tx_ntrm_pkts[]    --> BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2
 rx_conn_desc[]             --> BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE
 tx_not_ack_pkt             --> 1*BLE_PKT_BUF_SIZE
 
*/
#define   BLE_MAX_ALLOW_PER_CONNECTION          ( (BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2) \
                                                 +(BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE)   \
                                                 + BLE_PKT_BUF_SIZE )
                                                 
#define   BLE_CONN_BUF_SIZE                 (BLE_MAX_ALLOW_CONNECTION * BLE_MAX_ALLOW_PER_CONNECTION)
                                                                                        

uint8     g_pConnectionBuffer[BLE_CONN_BUF_SIZE] __attribute__ ((aligned(4)));
llConnState_t               pConnContext[BLE_MAX_ALLOW_CONNECTION];


//#define BLE_SUPPORT_CTE_IQ_SAMPLE TRUE
#ifdef BLE_SUPPORT_CTE_IQ_SAMPLE 
uint16 g_llCteSampleI[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];
uint16 g_llCteSampleQ[LL_CTE_MAX_SUPP_LEN * LL_CTE_SUPP_LEN_UNIT];
#endif

// ======================
#define DBGIO_LL_TRIG       P23
#define DBGIO_LL_IRQ        P24
#define DBGIO_APP_WAKEUP    P23
#define DBGIO_APP_SLEEP     P0
#define DBGIO_DIS_IRQ       P11
#define DBGIO_EN_IRQ        P34


////---------------------------------------------------------------------------------------////
// rom_main_var and defined

/////////////////////////
int   int_state;

int         hclk,  pclk;
#define     LARGE_HEAP_SIZE  (3*1024)
uint8       largeHeap[LARGE_HEAP_SIZE];


#define DEFAULT_UART_BAUD   115200

//----------------------------------------------------------------------------------------------
//patch
extern volatile uint8_t g_same_rf_channel_flag;

//void ll_hw_go1(void)
//{		
////LOG("%s\n",__func__);
//    //20190115 ZQ recorded ll re-trigger 
//    if(llWaitingIrq==TRUE)
//    {
//        g_pmCounters.ll_trigger_err++;
//    }
//    
//	*(volatile uint32_t *)(LL_HW_BASE+ 0x14) = LL_HW_IRQ_MASK;	//clr  irq status
//	*(volatile uint32_t *)(LL_HW_BASE+ 0x0c) = 0x0001;			//mask irq :only use mode done
//	*(volatile uint32_t *)(LL_HW_BASE+ 0x00) = 0x0001;			//trig 
//
//    //2018-05-23 ZQ
//    //fix negative rfPhyFreqOff bug, when in scan_rsq case, ll_hw_go will be  excuted before set_channel()
//    //so do not change the tx_rx_foff
//    //next metal change could modified the set_channel() to deal with the tx_rx_foff
//    uint8_t rfChnIdx = PHY_REG_RD(0x400300b4)&0xff;
//    if(!g_same_rf_channel_flag)
//    {
//        if(g_rfPhyFreqOffSet>=0)
//            PHY_REG_WT(0x400300b4, (g_rfPhyFreqOffSet<<16)+(g_rfPhyFreqOffSet<<8)+rfChnIdx);
//        else
//            PHY_REG_WT(0x400300b4, ((255+g_rfPhyFreqOffSet)<<16)+((255+g_rfPhyFreqOffSet)<<8)+(rfChnIdx-1) );
//    }
//    //2018-02-09 ZQ
//    //considering the ll_trigger timing, Trigger first, then set the tp_cal cap
//    
//    if(rfChnIdx<2) 
//    {
//        rfChnIdx=2;
//    }
//    else if(rfChnIdx>80)
//    {
//        rfChnIdx=80;
//    }
//
////    if(g_rfPhyPktFmt==PKT_FMT_BLE2M)
////        subWriteReg(0x40030094,7,0,g_rfPhyTpCalCapArry_2Mbps[(rfChnIdx-2)>>1]);
////    else
////        subWriteReg(0x40030094,7,0,g_rfPhyTpCalCapArry[(rfChnIdx-2)>>1]);
//
//    if(g_rfPhyPktFmt==PKT_FMT_BLE2M)
//        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0_2Mbps,g_rfPhyTpCal1_2Mbps,(rfChnIdx-2)>>1));
//    else
//        subWriteReg(0x40030094,7,0,RF_PHY_TPCAL_CALC(g_rfPhyTpCal0,g_rfPhyTpCal1,(rfChnIdx-2)>>1));
//        
//
//}


extern uint8_t             llSecondaryState;            // secondary state of LL
extern initInfo_t          initInfo;
void LL_IRQHandler1(void)
{
   uint32         irq_status;
    int8 ret;
    ISR_entry_time = read_current_fine_time();
    //*(volatile uint32_t *)0x4000f0b8 = 1;  // pclk_clk_gate_en
//    ll_debug_output(DEBUG_ISR_ENTRY);
    irq_status = ll_hw_get_irq_status();
	
//	gpio_write(P23,0);


	llWaitingIrq = FALSE;
    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
//		LOG(" NOT mode done\n");
//		gpio_write(P24,1);gpio_write(P24,0);

        return;
    }
	
//	if ((irq_status & LIRQ_COK)) 
//	{
//		gpio_write(P24,1);gpio_write(P24,0);
//	}
//	if ((irq_status & LIRQ_RTO)) 
//	{
//		gpio_write(P25,1);gpio_write(P25,0);
//	}

	ret = ll_processBasicIRQ(irq_status);
	// ================ Post ISR process: secondary pending state process
    // conn-adv case 2: other ISR, there is pending secondary advertise event, make it happen
//    if (llSecondaryState == LL_SEC_STATE_ADV_PENDING)
//   	{
//	    if (llSecAdvAllow())    // for multi-connection case, it is possible still no enough time for adv
//	    {
//			llSetupSecAdvEvt();
//		 
//			llSecondaryState = LL_SEC_STATE_ADV;
//	    }
//   	}
//	// there is pending scan event, make it happen, note that it may stay pending if there is no enough idle time
//	else 
	if (llSecondaryState == LL_SEC_STATE_SCAN_PENDING)
   	{      // trigger scan
	    llSetupSecScan(scanInfo.nextScanChan);
   	}
	else if (llSecondaryState == LL_SEC_STATE_INIT_PENDING)
    {
        // trigger scan
        llSetupSecInit(initInfo.nextScanChan);
    }
//    ll_debug_output(DEBUG_ISR_EXIT);
}
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_PendingTick;
extern uint32_t g_TIM1_IRQ_timing;
//uint32 pirq =0;
uint32_t t1_IRQDelayCnt = 0;

void TIM1_IRQHandler1(void)
{
//	LOG("%s,AP_TIM1->status 0x%x\n",__func__,AP_TIM1->status);
//	gpio_write(P25,0);
	uint32_t TIM1_IRQ_timing = read_current_fine_time();
	t1_IRQDelayCnt = AP_TIM1->LoadCount - AP_TIM1->CurrentCount ; 
	
	_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    if(AP_TIM1->status&0x1)   
    {  
        clear_timer_int(AP_TIM1);
        clear_timer(AP_TIM1);
		LL_evt_schedule();
    }
	g_TIM1_IRQ_timing = LL_TIME_DELTA(TIM1_IRQ_timing,read_current_fine_time() )   ;
//	if( pirq & 0x00100000 )
//	{
		g_TIM1_IRQ_timing += ( t1_IRQDelayCnt >> 2);
//	}
	HAL_EXIT_CRITICAL_SECTION();
}


void TIM2_IRQHandler1(void)
{
//	pirq = NVIC_GetPendingIRQs();
	_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
//	LOG("pirq 0X%X\n",pirq);
    if(AP_TIM2->status&0x1)   
    {  
        g_TIM2_IRQ_TIM3_CurrCount = AP_TIM3->CurrentCount;
        g_TIM2_IRQ_PendingTick = AP_TIM2->CurrentCount;
        clear_timer_int(AP_TIM2);
        osal_sys_tick ++; 
    }
	HAL_EXIT_CRITICAL_SECTION();
}

void TIM3_IRQHandler1(void)
{
//	pirq = NVIC_GetPendingIRQs();

	_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    if(AP_TIM3->status&0x1)
    {     
        clear_timer_int(AP_TIM3);
        //current_base_time++;      
    }
	HAL_EXIT_CRITICAL_SECTION();
}


void app_sleep_process1(void);
void app_wakeup_process1(void);
void __ck802_cpu_wakeup_init(void)
{
    __set_VBR(0);

    /* Clear active and pending IRQ */
    VIC->IABR[0] = 0x0;
    VIC->ICPR[0] = 0xFFFFFFFF;

    __enable_excp_irq();
    subWriteReg(0x40003030,4,4,1);//ck802 WFI enable
}
void hal_wakeup_irq_config(void)
{
    subWriteReg(0x40003030,4,4,1);//ck802 WFI enable
    NVIC_SetWakeupIRQ(TIM1_IRQn);
    NVIC_SetWakeupIRQ(TIM2_IRQn);
    //NVIC_SetWakeupIRQ(TIM3_IRQn);
    NVIC_SetWakeupIRQ(TIM4_IRQn);
    NVIC_SetWakeupIRQ(BB_IRQn);

}
// global configuration in SRAM, it could be change by application
// TODO: when integrate, the global_config should be set by APP project
void init_config(void)
{
    int i;
    
    for (i = 0; i < SOFT_PARAMETER_NUM; i ++)
        pGlobal_config[i] = 0;
    
    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;
    
    // LL switch setting
//    pGlobal_config[LL_SWITCH] =  LL_DEBUG_ALLOW | SLAVE_LATENCY_ALLOW | LL_WHITELIST_ALLOW
//                                   | SIMUL_CONN_ADV_ALLOW | SIMUL_CONN_SCAN_ALLOW; //RC32_TRACKINK_ALLOW
    
    if(g_clk32K_config==CLK_32K_XTAL)
        pGlobal_config[LL_SWITCH] &= 0xffffffee;
    else
        pGlobal_config[LL_SWITCH] |= RC32_TRACKINK_ALLOW | LL_RC32K_SEL;
    
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

    pGlobal_config[WAKEUP_ADVANCE] = 2350;//650;//600;//310;
       
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[WAKEUP_DELAY] = 16;
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[WAKEUP_DELAY] = 20;
    }

    // sleep time, in us
    pGlobal_config[MAX_SLEEP_TIME] = 1500000;    
    pGlobal_config[MIN_SLEEP_TIME] = 1500;
    
    pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K] = 60;// 30.5 per tick    

    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    // LL engine settle time
    pGlobal_config[LL_HW_BB_DELAY] = 54;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY] = 52;
        // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV] = 62-RF_PHY_EXT_PREAMBLE_US;
    pGlobal_config[LL_HW_Tx_TO_RX_INTV] = 50;//65   
    
    //------------------------------------------------2MPHY
    // LL engine settle time 
    pGlobal_config[LL_HW_BB_DELAY_2MPHY] = 59;
    pGlobal_config[LL_HW_AFE_DELAY_2MPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_2MPHY] = 52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_2MPHY] = 70-RF_PHY_EXT_PREAMBLE_US;//72
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_2MPHY] = 57;//72   

    
    // LL engine settle time, for advertisement
    pGlobal_config[LL_HW_BB_DELAY_ADV] = 90;
    pGlobal_config[LL_HW_AFE_DELAY_ADV] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_ADV] = 60;    
        
    // adv channel interval
    pGlobal_config[ADV_CHANNEL_INTERVAL] = 1400;//6250;
    
    pGlobal_config[NON_ADV_CHANNEL_INTERVAL] = 666;//6250;
    
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 13+RF_PHY_EXT_PREAMBLE_US;//23;       
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 4+RF_PHY_EXT_PREAMBLE_US;        // 12	//  2019/3/19 A2: 12 --> 9
    }                                      
	else if(g_system_clk == SYS_CLK_DLL_64M)		//  2019/3/26 add
	{
		pGlobal_config[SCAN_RSP_DELAY] = 3+RF_PHY_EXT_PREAMBLE_US;
	}
				
    // conn_req -> slave connection event calibration time, will advance the receive window
    pGlobal_config[CONN_REQ_TO_SLAVE_DELAY] = 100;	// 1300
		
    // calibration time for 2 connection event, will advance the next conn event receive window
    // SLAVE_CONN_DELAY for sync catch, SLAVE_CONN_DELAY_BEFORE_SYNC for sync not catch
    pGlobal_config[SLAVE_CONN_DELAY] = 143;//0;//1500;//0;//3000;//0;          ---> update 11-20
    pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 160;
		
    // RTLP timeout
//    pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] = 50000;
//    pGlobal_config[LL_HW_RTLP_TO_GAP]       = 1000;
    
//    pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]  = 4000 + pGlobal_config[SLAVE_CONN_DELAY] * 2;//500;
	pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]	= 1000 ;

		   
    // direct adv interval configuration
    pGlobal_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
    pGlobal_config[LDC_DIRECT_ADV_INTERVAL] = 6250;
    
    // A1 ROM metal change for HDC direct adv, 
    pGlobal_config[DIR_ADV_DELAY] = 115;   // in us, consider both direct adv broadcast time & SW delay, ... etc.
    
    // A1 ROM metal change
//    pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 6;//8;
//    pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 6;//8;    

//    pGlobal_config[LL_TRX_NUM_ADAPTIVE_CONFIG] = 8;     //0:        disable adaptive 
                                                        //other:    adaptive max limitation

    
//    pGlobal_config[LL_TX_PWR_TO_REG_BIAS]   = 0x15;   // assume when g_rfPhyTxPower = 0x1f, tx power = 10dBm
    
    //smart window configuration
//    pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]      = 2;
//    pGlobal_config[LL_SMART_WINDOW_TARGET]          = 600;
//    pGlobal_config[LL_SMART_WINDOW_INCREMENT]       = 9;
//    pGlobal_config[LL_SMART_WINDOW_LIMIT]           = 20000;
//    pGlobal_config[LL_SMART_WINDOW_ACTIVE_THD]      = 8;
//    pGlobal_config[LL_SMART_WINDOW_ACTIVE_RANGE]    = 0;//300
//
//
//    pGlobal_config[LL_SMART_WINDOW_FIRST_WINDOW]    = 5000;
//	pGlobal_config[LL_FIRST_WINDOW]    = 5000;
    
    //====== A2 metal change add, for scanner & initiator
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 18+RF_PHY_EXT_PREAMBLE_US;//20;		//  2019/3/19 A2: 20 --> 18
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 25+RF_PHY_EXT_PREAMBLE_US;//27;		//  2019/3/19 A2: 27 --> 25
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 10+RF_PHY_EXT_PREAMBLE_US;//12;		//  2019/3/19 A2: 12 --> 10
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 40+RF_PHY_EXT_PREAMBLE_US;
    }
	else if(g_system_clk==SYS_CLK_DLL_64M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 8+RF_PHY_EXT_PREAMBLE_US;				//  2019/3/26 add
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 10+RF_PHY_EXT_PREAMBLE_US;
    }

    
    // TRLP timeout
//    pGlobal_config[LL_HW_TRLP_LOOP_TIMEOUT] = 50000;    // enough for 8Tx + 8Rx : (41 * 8 + 150) * 16 - 150 = 7498us
//    pGlobal_config[LL_HW_TRLP_TO_GAP]       = 1000;
    pGlobal_config[LL_MOVE_TO_MASTER_DELAY] = 500;  
    
    pGlobal_config[LL_CONN_REQ_WIN_SIZE] = 1;  
    pGlobal_config[LL_CONN_REQ_WIN_OFFSET] = 1;  
    
    pGlobal_config[LL_MASTER_PROCESS_TARGET] = 200;   // reserve time for preparing master conn event, delay should be insert if needn't so long time
//    pGlobal_config[LL_MASTER_TIRQ_DELAY] = 0;         // timer IRQ -> timer ISR delay


    pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM] = 56;  // 0.125us
    
    pGlobal_config[MAC_ADDRESS_LOC] = 0x11004000;
    
    // for simultaneous conn & adv/scan
//    pGlobal_config[LL_NOCONN_ADV_EST_TIME] = 1400*3;
//    pGlobal_config[LL_NOCONN_ADV_MARGIN] = 600;
    
    pGlobal_config[LL_SEC_SCAN_MARGIN] = 1250;//1400;  to avoid mesh proxy llTrigErr 0x15
    pGlobal_config[LL_MIN_SCAN_TIME] = 200;
    
    //  BBB new
    pGlobal_config[TIMER_SET_ENTRY_TIME] = 60;
    pGlobal_config[LL_MULTICONN_MASTER_PREEMP] = 0;
	pGlobal_config[LL_MULTICONN_MASTER_SLOT] = 4;
//    pGlobal_config[LL_MULTICONN_SLAVE_PREEMP] = 0;
//
//    pGlobal_config[LL_EXT_ADV_TASK_DURATION] = 20000;	
//    pGlobal_config[LL_PRD_ADV_TASK_DURATION] = 20000;	
    pGlobal_config[LL_CONN_TASK_DURATION] = 900;			
//    pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT] = 5000;	
//	pGlobal_config[LL_EXT_ADV_INTER_SEC_CHN_INT] = 5000;	
//	pGlobal_config[LL_EXT_ADV_PRI_2_SEC_CHN_INT] = 1500;	
//
//	pGlobal_config[LL_EXT_ADV_RSC_PERIOD] = 1000000;	
//	pGlobal_config[LL_EXT_ADV_RSC_SLOT_DURATION] = 10000;	
//
//	pGlobal_config[LL_PRD_ADV_RSC_PERIOD] = 1000000;	
//	pGlobal_config[LL_PRD_ADV_RSC_SLOT_DURATION] = 10000;	
//
//    pGlobal_config[LL_EXT_ADV_PROCESS_TARGET] = 500;		
//    pGlobal_config[LL_PRD_ADV_PROCESS_TARGET] = 500;		
//	
	
    osal_mem_set_heap((osalMemHdr_t *)largeHeap, LARGE_HEAP_SIZE);
    
//    LOG("sizeof(struct ll_pkt_desc) = %d, buf size = %d\n", sizeof(struct ll_pkt_desc), BLE_CONN_BUF_SIZE);

//    LL_InitConnectContext(pConnContext, 
//                        g_pConnectionBuffer, 
//                        BLE_MAX_ALLOW_CONNECTION, 
//                        BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
//                        BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
//                        BLE_PKT_VERSION);


	extern void osalInitTasks( void );
	extern pTaskEventHandlerFn tasksArr[];
	extern uint16 tasksCnt;
	extern uint16 *tasksEvents;
	JUMP_FUNCTION_SET(OSAL_INIT_TASKS,(uint32_t)&osalInitTasks);
	JUMP_FUNCTION_SET(TASKS_ARRAY,(uint32_t)&tasksArr);
	JUMP_FUNCTION_SET(TASK_COUNT ,(uint32_t)&tasksCnt);
	JUMP_FUNCTION_SET(TASK_EVENTS,(uint32_t)&tasksEvents);

    
    //-------------------------------------------------------------------
    // patch function register
    //--------------------------------------------------------------------

	JUMP_FUNCTION(228)                         =   (uint32_t)&LL_IRQHandler1;
	JUMP_FUNCTION(244)                         =   (uint32_t)&TIM1_IRQHandler1;
	
	JUMP_FUNCTION_SET(V4_IRQ_HANDLER,(uint32_t)&LL_IRQHandler1);
	JUMP_FUNCTION_SET(APP_SLEEP_PROCESS,(uint32_t)&app_sleep_process1);
	JUMP_FUNCTION_SET(APP_WAKEUP_PROCESS,(uint32_t)&app_wakeup_process1);
	JUMP_FUNCTION_SET(V20_IRQ_HANDLER,(uint32_t)&TIM1_IRQHandler1);
	JUMP_FUNCTION_SET(V21_IRQ_HANDLER,(uint32_t)&TIM2_IRQHandler1);
	JUMP_FUNCTION_SET(V22_IRQ_HANDLER,(uint32_t)&TIM3_IRQHandler1);
	JUMP_FUNCTION_SET(RF_CALIBRATTE,(uint32_t)&rf_calibrate1);
	JUMP_FUNCTION_SET(RF_PHY_CHANGE,(uint32_t)&rf_phy_change_cfg0);
// ====================
	

    disableSleep();
    //setSleepMode(MCU_SLEEP_MODE);//SYSTEM_SLEEP_MODE
//    enableSleep();
    setSleepMode(SYSTEM_SLEEP_MODE);
}




static void hal_low_power_io_init(void)
{
   //========= pull all io to gnd by default
    ioinit_cfg_t ioInit[GPIO_NUM]= {
       {GPIO_P00,   GPIO_PULL_DOWN  },
       {GPIO_P01,   GPIO_PULL_DOWN  },
       {GPIO_P02,   GPIO_FLOATING   },/*SWD*/
       {GPIO_P03,   GPIO_FLOATING   },/*SWD*/
       {GPIO_P07,   GPIO_PULL_DOWN  },
       {GPIO_P09,   GPIO_PULL_UP    },/*UART TX*/
       {GPIO_P10,   GPIO_PULL_UP    },/*UART RX*/
       {GPIO_P11,   GPIO_PULL_DOWN  },
       {GPIO_P14,   GPIO_PULL_DOWN  },
       {GPIO_P15,   GPIO_PULL_DOWN  },
       {GPIO_P16,   GPIO_FLOATING   },/*32k xtal*/
       {GPIO_P17,   GPIO_FLOATING   },/*32k xtal*/
       {GPIO_P18,   GPIO_PULL_DOWN  },
       {GPIO_P20,   GPIO_PULL_DOWN  },
       {GPIO_P23,   GPIO_PULL_DOWN  },
       {GPIO_P24,   GPIO_PULL_DOWN  },
       {GPIO_P25,   GPIO_PULL_DOWN  },
       {GPIO_P26,   GPIO_PULL_DOWN  },
       {GPIO_P27,   GPIO_PULL_DOWN  },
       {GPIO_P31,   GPIO_PULL_DOWN  },
       {GPIO_P32,   GPIO_PULL_DOWN  },
       {GPIO_P33,   GPIO_PULL_DOWN  },
       {GPIO_P34,   GPIO_PULL_DOWN  },
    };
    for(uint8_t i=0;i<GPIO_NUM;i++)
        hal_gpio_pull_set(ioInit[i].pin,ioInit[i].type);

    DCDC_CONFIG_SETTING(0x0f);
    DIG_LDO_CURRENT_SETTING(0x01);
    hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2);
    //hal_pwrmgr_RAM_retention(RET_SRAM0);
    hal_pwrmgr_RAM_retention_set(); 
    hal_pwrmgr_LowCurrentLdo_enable();
    //========= low power module clk gate
    #if(PHY_MCU_TYPE==MCU_BUMBEE_CK802)

    *(volatile uint32_t *)0x40000008 = 0x001961f1;  //
    *(volatile uint32_t *)0x40000014 = 0x01e00278;  //
    #else

    *(volatile uint32_t *)0x40000008 = 0x001961f0;  //
    *(volatile uint32_t *)0x40000014 = 0x01e00279;  //
    #endif
 
}

void wakeup_init1()
{
    uint8_t pktFmt = 1;    // packet format 1: BLE 1M
    uint32  temp;
    //int int_state;
		
    // =========== clk gate for low power
	//*(volatile uint32_t *) 0x40000008 = 0x01e92190; 
    
    // enable rng analog block. RNG analog need > 200us before stable, and it consume few current, so open it at wakeup
    //*(volatile uint32_t *) 0x4000f048 |= 1 << 23;           
	

    // =========== config PCRM 
//	*(volatile uint32_t *) 0x4000f040 = 0x501fb000; //enable xtal out
//	*(volatile uint32_t *) 0x4000f044 = 0x01ade8b0; //switch rf,adc to doubler,32M
	
//---by ZQ  2017-10-17
    //*(volatile uint32_t *) 0x4000f040 = 0x501fb820;  // enable xtal out
                                                     // set the xtal cap to zero for faster settle
                                                     // set [16] manually enable ac strigger f 20180613 by ZQ
    //*(volatile uint32_t *) 0x4000f044 = 0x01bdf8b0;//0x01bef830;  // switch rf,adc to doubler, dll_off, dll_ldo on
                                                     // dll will be turn on in rf_ini after xtal settle

    //*(volatile uint32_t *) 0x4000f044 = 0x00be0830;  //[26:22] 0x02,[21:18]0x0f,[16:12]0x00,[7:4]0x03
                                                     //< 22>:sel_rf_clk_16M;  
                                                     //< 23>:sel_rf_dbl_clk_32M;
                                                     //< 24>:sel_rxadc_dbl_clk_32M;
                                                     //< 25>:sel_rxadc_dbl_clk_32M_polarity;
                                                     //< 26>:sel_rf_dbl_clk_32M_polarity
                                                     
                                                    // < 18>:en_rf_clk;
                                                    // < 19>:en_rxadc_clk_32M;
                                                    // < 20>:sel_cp_clk_32M;
                                                    // < 21>:sel_dig_dble_clk_32M;

                                                    // < 12>:en_cp_dll_clk;
                                                    // < 13>:en_dig_clk_32M;
                                                    // < 14>:en_dig_clk_48M;
                                                    // < 15>:en_dig_clk_64M;
                                                    // < 16>:en_dig_clk_96M; 




    //each rtc count is about 30.5us
    //after 15count , xtal will be feedout to dll and doubler
    WaitRTCCount(pGlobal_config[WAKEUP_DELAY]);
    
    // ============ config BB Top
	*(volatile uint32_t *) 0x40030000 = 0x3d068001; // set tx pkt =2
	*(volatile uint32_t *) 0x400300bc = 0x834;      //[7:0] pll_tm [11:8] rxafe settle
	*(volatile uint32_t *) 0x400300a4 = 0x140;      //[6] for tpm_en

	clk_init(g_system_clk);

	
	  // ================= clock selection
    // hclk_sel	select hclk source. 0---rc 32m    1----dll 32m  2---xtal 16m   3---dll 48m  4----dll 64m   5----dll 96m
//    switch (pGlobal_config[CLOCK_SETTING])
//    {
//        case SYS_CLK_XTAL_16M:
////            *(int *) 0x4000f03C = 0x18001;                  // clock selection
//            *(int *) 0x4000f03C = 0x10002;                    // clock selection
//        break;
//        case SYS_CLK_DBL_32M:
//        case SYS_CLK_DLL_32M:
//            *(int *) 0x4000f03C = 0x10001;                    // clock selection
//        break;
//        case SYS_CLK_DLL_48M:
//            *(int *) 0x4000f03C = 0x10003;                    // clock selection
//        break;
//        case SYS_CLK_DLL_64M:
//            *(int *) 0x4000f03C = 0x10004;                    // clock selection
//        break;
//        case SYS_CLK_DLL_96M:
//            *(int *) 0x4000f03C = 0x10005;                    // clock selection
//        break;
//        default:
//            *(int *) 0x4000f03C = 0x10002;                    // clock selection
//        break;        
//    }

    // ========== init timers
    set_timer(AP_TIM2, 625);      // OSAL 625us tick     
    set_timer(AP_TIM3, BASE_TIME_UNITS);   // 1s timer       

    // =========== open interrupt mask
	//int_state = 0x14;
	//set_int(int_state);
    //should use NVIC_EnableIRQn()
    NVIC_EnableIRQ(BB_IRQn);
    NVIC_EnableIRQ(TIM1_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM4_IRQn);
	#if(PHY_MCU_TYPE==MCU_BUMBEE_CK802)
		SystemInit();
		subWriteReg(0x40003030,4,4,1);//ck802 WFI enable
	#endif  
    // =========== ll HW setting
    set_max_length(0xff);

    ll_hw_set_empty_head(0x0001);
	
    //time related setting
    ll_hw_set_rx_timeout_1st(   500);
    ll_hw_set_rx_timeout(        88);       //ZQ 20180606, reduce rx timeout for power saving
                                            //preamble + syncword=40us, sync process = 8us
                                            //timeout should be larger then 48us, 
											
    //ll_hw_set_rx_timeout(       268);		//for ble shoulde be larger than 80+128. if sync, the timeout timer stop. 
                                      // (80 + 128) - BLE 5.0 preamble + access time, 60 for HW process delay
                                      // this time doesn't consider HW startup time, it is set in other regs

    ll_hw_set_loop_timeout(   30000);
    
//		ll_hw_set_tx_rx_release	(10,   	 1);
//		ll_hw_set_rx_tx_interval(    	57);		//T_IFS=150us for BLE 1M
//		ll_hw_set_tx_rx_interval(    	65);		//T_IFS=150us for BLE 1M
//		ll_hw_set_trx_settle	(57, 8, 52);		//TxBB,RxAFE,PLL    
    ll_hw_set_timing(pktFmt);
    
	
    ll_hw_ign_rfifo(LL_HW_IGN_SSN | LL_HW_IGN_CRC | LL_HW_IGN_EMP);
    
    // ======== enable tracking 32KHz RC timer with 16MHz crystal clock
    temp = *(volatile uint32_t *)0x4000f05C;
    *(volatile uint32_t *)0x4000f05C = (temp & 0xfffefe00) | 0x0108; //[16] 16M [8:4] cnt [3] track_en_rc32k
}


void  ble_main1(void)  
{  
	
//    hal_uart_init(115200, 32000000, P9, P10);
//    hal_uart_tx("ble_main");

    //pGlobal_config = (uint32 *)(CONFIG_BASE_ADDR);
    
    switch (pGlobal_config[CLOCK_SETTING])
    {
        case SYS_CLK_XTAL_16M:
            hclk_per_us = 16;
            hclk_per_us_shift = 4;
        break;
        case SYS_CLK_RC_32M:
        case SYS_CLK_DLL_32M:
            hclk_per_us = 32;
            hclk_per_us_shift = 5;
        break;
				
#if (PHY_MCU_TYPE == MCU_BUMBEE_M0)
				case SYS_CLK_4M:
					break;
				case SYS_CLK_8M:
					break;
#elif ((PHY_MCU_TYPE == MCU_PRIME_A1) ||(PHY_MCU_TYPE == MCU_PRIME_A2))
				case SYS_CLK_DBL_32M:
					  hclk_per_us = 32;
            hclk_per_us_shift = 5;
        break;
#endif
				
        case SYS_CLK_DLL_48M:
            hclk_per_us = 48;
            hclk_per_us_shift = 0;
        break;
        case SYS_CLK_DLL_64M:
            hclk_per_us = 64;
            hclk_per_us_shift = 6;
        break;
        case SYS_CLK_DLL_96M:
            hclk_per_us = 96;
            hclk_per_us_shift = 0;
        break;
        default:
            hclk_per_us = 16;
            hclk_per_us_shift = 4;
        break;        
    }
    boot_init();	
    LOG("1-");
    wakeup_init1();
    // rf initial entry, will be set in app
    rf_init();    
    
    rf_calibrate();
       
    //app_main();
}


static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_N40KHZ;
    //============config xtal 16M cap
    XTAL16M_CAP_SETTING(0x09);
//	XTAL16M_CURRENT_SETTING(0x01);
    ble_main1();

    NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV

//    hal_pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);

}
#if 0
void hal_rtc_clock_config(uint8_t clk32Mode)
{
    if(clk32Mode == CLK_32K_RCOSC)
    {
        subWriteReg(0x4000f014,31,27,0x05);
        subWriteReg(0x4000f01c,16,7,0x3fb);   //software control 32k_clk
        subWriteReg(0x4000f01c,6,6 ,0x01);    //enable software control
        
        pGlobal_config[LL_SWITCH] |= RC32_TRACKINK_ALLOW | LL_RC32K_SEL;

//        //disable smart windwo for stable conectivity not higher power consumption
//        pGlobal_config[LL_SMART_WINDOW_COEF_ALPHA]=0;
    }
    else if(clk32Mode == CLK_32K_XTAL)
    {
        // P16 P17 for 32K XTAL input
        hal_gpio_pull_set(P16,GPIO_FLOATING);
        hal_gpio_pull_set(P17,GPIO_FLOATING);

        subWriteReg(0x4000f01c,9,8,0x03);   //software control 32k_clk
        subWriteReg(0x4000f01c,6,6,0x00);   //disable software control

        subWriteReg(0x4000f014,31,27,0x16);
        pGlobal_config[LL_SWITCH] &= 0xffffffee;
    }

}
#endif

static void hal_cache_init(void)
{
    volatile int dly=100;
    //clock gate
    hal_clk_gate_enable(MOD_HCLK_CACHE);
    hal_clk_gate_enable(MOD_PCLK_CACHE);

    //cache rst
    AP_PCR->CACHE_RST=0x00;
    while(dly--){};
    AP_PCR->CACHE_RST=0x03;
    //cache flush tag
    AP_CACHE->CTRL0 = 0x01;
    //cache disable
    AP_PCR->CACHE_BYPASS = 1;
}
static void hal_mpu_config(void)
{
	mpu_region_attr_t attr0 = {
		.nx = 0,
		.ap = AP_BOTH_RW,
		.s = 0,
	};
	mpu_region_attr_t attr1 = {
		.nx = 0,
		.ap = AP_BOTH_RW,
		.s = 1,
	};

	csi_mpu_config_region(0, 0,          REGION_SIZE_4GB, attr0, 1);
	csi_mpu_config_region(1, 0x11000000, REGION_SIZE_512KB, attr1, 1);
	csi_mpu_enable();

}

static void hal_init(void)
{
    hal_low_power_io_init();
    clk_init(g_system_clk); //system init
    hal_rtc_clock_config(g_clk32K_config);
    
    clk_spif_ref_clk(SYS_CLK_DLL_64M);
    AP_SPIF->read_instr = 0x801003b;
    hal_cache_init();
    hal_mpu_config();
    hal_pwrmgr_init();
    hal_wakeup_irq_config();

    hal_gpio_init();
				
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
int  main(void)  
{

//#if(DBG_ROM_MAIN==1)
//	rom_main();
//#endif

    g_system_clk = SYS_CLK_DLL_48M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
    g_clk32K_config = CLK_32K_XTAL;//CLK_32K_XTAL,CLK_32K_RCOSC    
    //pGlobal_config = (uint32 *)(CONFIG_BASE_ADDR);   
    
    drv_irq_init();

    init_config();
    
    hal_rfphy_init();
    
    hal_init();

    //ble_main();
	
//set_int(0x814);
//	uart_init0(115200, 16000000, 9, 10, 1);


//    hal_uart_init(115200, P9, P10, NULL);
    LOG_INIT();

	LL_InitConnectContext(pConnContext, 
                        g_pConnectionBuffer, 
                        BLE_MAX_ALLOW_CONNECTION, 
                        BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
                        BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
                        BLE_PKT_VERSION);
	
	LOG("===DEMO V2.2.1===\n");
	LOG("0x%p,0x%p\n",&main,(uint16)&main);
    LOG("rfClk %d rcClk %d sysClk %d tpCap[%02x %02x]\n",g_rfPhyClkSel,g_clk32K_config,g_system_clk,g_rfPhyTpCal0,g_rfPhyTpCal1);

    LOG("sizeof(struct ll_pkt_desc) = %d, buf size = %d\n", sizeof(struct ll_pkt_desc), BLE_CONN_BUF_SIZE);
    LOG("g_pConnectionBuffer %p,sizeof(g_pConnectionBuffer) = %d, sizeof(pConnContext) = %d,\n", g_pConnectionBuffer,sizeof(g_pConnectionBuffer), sizeof(pConnContext));  
	rf_phy_ini();

//	for(uint8 i=0;i<BLE_CONN_BUF_SIZE;i++)
//		LOG("0x%02X ",g_pConnectionBuffer[i]);
//	LOG("\n");
    //     // ======== enable tracking 32KHz RC timer with 16MHz crystal clock
    // uint32_t temp = *(volatile uint32_t *)0x4000f05C;
    // *(volatile uint32_t *)0x4000f05C = (temp & 0xfffefe00) | 0x0108; //[16] 16M [8:4] cnt [3] track_en_rc32k

    // while(1)
    // {
    //     LOG("%d\n",0x1ffff&(*(volatile uint32_t *)0x4000f064));
    //     WaitMs(50);
    // }

    app_main();	

}

void app_wakeup_process1(void)
{
    //========= low power module clk gate
    #if(PHY_MCU_TYPE==MCU_BUMBEE_CK802)

    *(volatile uint32_t *)0x40000008 = 0x001961f1;  //
    *(volatile uint32_t *)0x40000014 = 0x01e00278;  //
    #else

    *(volatile uint32_t *)0x40000008 = 0x001961f0;  //
    *(volatile uint32_t *)0x40000014 = 0x01e00279;  //
    #endif

    NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV
    hal_cache_init();
    hal_pwrmgr_wakeup_process();
    hal_wakeup_irq_config();
}

void app_sleep_process1(void)
{
    //LOG("[S] %8x %d %d \n",PHY_REG_RD(0x4000f024),g_ram_sleep_tick,PHY_REG_RD(0x4000f02c));
    
}



/////////////////////////////////////  end  ///////////////////////////////////////









	









