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



#include "rom_sym_def.h"
#include "bus_dev.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "ll_sleep.h"

#include "uart.h"
#include "adc.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "string.h"
#include "clock.h"
#include "log.h"

#include "OSAL_Tasks.h"

extern void  ble_main(void);

extern void hal_rom_code_ini(void);
extern int app_main(void);
extern void init_config(void);
extern void drv_irq_init(void);

extern  uint32_t pclk;


#define   BLE_MAX_ALLOW_CONNECTION              1
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_TX        2
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_RX        2

#define   BLE_PKT_VERSION                       BLE_PKT_VERSION_4_0 //BLE_PKT_VERSION_5_1 //BLE_PKT_VERSION_5_1     

#define   BLE_PKT_BUF_SIZE                  (((BLE_PKT_VERSION == BLE_PKT_VERSION_4_0) ? 1 : 0) * BLE_PKT40_LEN \
	                                        + (sizeof(struct ll_pkt_desc) - 2))

#define   BLE_MAX_ALLOW_PER_CONNECTION          ( (BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2) \
                                                 +(BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE)   \
                                                 + BLE_PKT_BUF_SIZE )
                                                 
#define   BLE_CONN_BUF_SIZE                 (BLE_MAX_ALLOW_CONNECTION * BLE_MAX_ALLOW_PER_CONNECTION)
                                                                                        

uint8     g_pConnectionBuffer[BLE_CONN_BUF_SIZE] __attribute__ ((aligned(4)));
llConnState_t               pConnContext[BLE_MAX_ALLOW_CONNECTION];


#define     LARGE_HEAP_SIZE  2*1024
uint8       g_largeHeap[LARGE_HEAP_SIZE];

static void rf_wakeup_handler(void){
  NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
}

void boot_init(void);

void wakeup_init(void);

void hal_rom_code_ini(void)
{
    boot_init();    
    wakeup_init();

}


void hal_init(void)
{
    //clk_init(g_system_clk);
	 
	//hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2|RET_SRAM3|RET_SRAM4);
	pwrmgr_RAM_retention_set();
	
    LOG_INIT();
    
    //hal_gpio_init();
	
    //hal_adc_init();
      
    PRINT("all driver init OK!\n");
}
static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;

    //hal_rom_code_ini();
    ble_main();
    
    //Quick Boot setting and 
     *(volatile uint32_t *) 0x4000f01c = 0x0000004;       //  3'b1xx: 62.5us.  control bits for main digital part reset wait time after power up charge pump. 

    //========= low currernt setting IO init
    //========= pull all io to gnd by default
    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down

    //========= UART RX Pull up
    //hal_gpio_pull_set(P10,WEAK_PULL_UP);    


    NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);

    pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);

}

void HAL_wakeup_process(void)
{
	 LOG_INIT();
}

extern uint32 global_config[];
extern uint32_t  __initial_sp;
extern uint32_t  g_smartWindowSize  ;

uint32* pGlobal_config;

//ab8967452301
const uint32 s_macaddr[2];

void init_config(void)
{
    int i;

 
    pGlobal_config = global_config;

    //Config the ROM_CODE global_config pointer 2018-3-20
       
    
    for (i = 0; i < SOFT_PARAMETER_NUM; i ++)
        pGlobal_config[i] = 0;

    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;
    
    // LL switch setting
    pGlobal_config[LL_SWITCH] =  LL_DEBUG_ALLOW | SLAVE_LATENCY_ALLOW | RC32_TRACKINK_ALLOW
                                | SIMUL_CONN_ADV_ALLOW | SIMUL_CONN_SCAN_ALLOW | GAP_DUP_RPT_FILTER_DISALLOW; //
    
 
    
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
        pGlobal_config[WAKEUP_DELAY] = 3;
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[WAKEUP_DELAY] = 3;
    }
	else if(g_system_clk==SYS_CLK_DLL_64M)
	{
		pGlobal_config[WAKEUP_DELAY] = 3;
	}

    // set 0 to bpyass CLK_TRACKING
    pGlobal_config[CLK_TRACKING_CONFIG_DLL ] = 0x0001f414;// [19:8] target_cnt [7:0] settle_thd 
    pGlobal_config[CLK_TRACKING_CONFIG_XTAL] = 0x0000fa0a;// [19:8] target_cnt [7:0] settle_thd

    pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG] = 0x00640501;//[31:16]avg init cnt  [15:8] alpha [7:0] beta
    
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

    
    //ownPublicAddr[5] = *(p++);4
    //ownPublicAddr[4] = *(p);  5  
    //ownPublicAddr[3] = *(p++);0
    //ownPublicAddr[2] = *(p++);1
    //ownPublicAddr[1] = *(p++);2
    //ownPublicAddr[0] = *(p++);3
    uint8_t* p8_macaddr = (uint8_t*)(&s_macaddr);
    p8_macaddr[4] = 0x01;
    p8_macaddr[5] = 0x23;
    p8_macaddr[0] = 0x45;
    p8_macaddr[1] = 0x67;
    p8_macaddr[2] = 0x89;
    p8_macaddr[3] = 0xab;
    pGlobal_config[MAC_ADDRESS_LOC] = (uint32_t)s_macaddr;
    
    // for simultaneous conn & adv/scan
    pGlobal_config[LL_NOCONN_ADV_EST_TIME] = 1400*3;
    pGlobal_config[LL_NOCONN_ADV_MARGIN] = 600;
    
    pGlobal_config[LL_SEC_SCAN_MARGIN] = 1400;
    pGlobal_config[LL_MIN_SCAN_TIME] = 2000;
	
	pGlobal_config[TIMER_ISR_ENTRY_TIME] = 30;
	pGlobal_config[NEXT_TIMER1_CONSIDER_DELAY] = 300;
	pGlobal_config[LL_WRITE_RE_TX_FIFO] = 50;
    
#if 0
    //-------------------------------------------------------------------
    // patch function register
    //--------------------------------------------------------------------
    JUMP_FUNCTION(LL_HW_GO)                         =   (uint32_t)&ll_hw_go1;
    JUMP_FUNCTION(RF_CALIBRATTE)                    =   (uint32_t)&rf_calibrate1;
		
    JUMP_FUNCTION(LL_SET_SCAN_CTRL)                 = (uint32_t)&LL_SetScanControl1;
    JUMP_FUNCTION(LL_SETUP_ADV)                     = (uint32_t)&llSetupAdv1;

    //JUMP_FUNCTION(V4_IRQ_HANDLER)                         =   (uint32_t)&LL_IRQHandler1;
    
#if(__BUILD_FOR_OPTIMUS_DEBUG__)
    JUMP_FUNCTION(DEBUG_PRINT)                      =   (uint32_t)&output_optimus0; 

#endif

    //check chip ID
    check_chip_id();
    check_chip_mAddr();
#endif
	extern void osalInitTasks( void );
	extern pTaskEventHandlerFn tasksArr[];
	extern uint16 tasksCnt;
	extern uint16 *tasksEvents;
	JUMP_FUNCTION_SET(OSAL_INIT_TASKS,(uint32_t*)osalInitTasks);
	JUMP_FUNCTION_SET(TASKS_ARRAY,(uint32_t*)tasksArr);
	JUMP_FUNCTION_SET(TASK_COUNT ,(uint32_t)&tasksCnt);
	JUMP_FUNCTION_SET(TASK_EVENTS,(uint32_t)&tasksEvents);

	JUMP_FUNCTION_SET(APP_WAKEUP_PROCESS,(uint32_t)&HAL_wakeup_process);

disableSleep();
    setSleepMode(SYSTEM_SLEEP_MODE);
	
osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
LL_InitConnectContext(pConnContext, 
                        g_pConnectionBuffer, 
                        BLE_MAX_ALLOW_CONNECTION, 
                        BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
                        BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
                        BLE_PKT_VERSION);


//(const uint32_t*)0, //writeLog,                  // 0. write Log
//	(const uint32_t*)osalInitTasks,             // 1. init entry of app
//	(const uint32_t*)tasksArr,                  // 2. task list
//	(const uint32_t*)&tasksCnt,                 // 3. task count
//	(const uint32_t*)&tasksEvents,              // 4. task events

}
int  main(void)  
{     
    // init global configuration of SOC
    //g_system_clk = SYS_CLK_RC_32M;
	//*((unsigned int*)0x40000028) = 1;	//disable cache
    g_system_clk = SYS_CLK_DLL_48M;
    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
	
    drv_irq_init();
    
    init_config();
    pwrmgr_config(PWR_MODE_SLEEP);
    pwrmgr_init();
    
    hal_rfphy_init();  

    hal_init();
	
    app_main();	
}


///////////////////////////////////  end  ///////////////////////////////////////

