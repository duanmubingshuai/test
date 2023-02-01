#include "rom_sym_def.h"

//#include <stdio.h>
#include "bus_dev.h"
#include "uart.h"

#include "gpio.h"
#include "clock.h"
#include "timer.h"
#include "ll.h"
#include "ll_def.h"
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
#include "linkdb.h"

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
extern void linkDB_InitContext( linkDBItem_t *linkDBs,pfnLinkDBCB_t *linkCB,uint8 cb_maxs);

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

//volatile uint8 g_clk32K_config;

extern unsigned char urx_buf[];

extern volatile unsigned int uart_rx_wIdx;
/////////////////////////////

// ===================== connection context relate definition

#define   BLE_MAX_ALLOW_CONNECTION              1
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_TX        2
#define   BLE_MAX_ALLOW_PKT_PER_EVENT_RX        2

#define   BLE_PKT_VERSION                       BLE_PKT_VERSION_4_0 

#define   BLE_PKT_BUF_SIZE                  (((BLE_PKT_VERSION == BLE_PKT_VERSION_4_0) ? 1 : 0) * BLE_PKT40_LEN \
	                                        + (sizeof(struct ll_pkt_desc) - 2))

#define   BLE_MAX_ALLOW_PER_CONNECTION          ( (BLE_MAX_ALLOW_PKT_PER_EVENT_TX * BLE_PKT_BUF_SIZE*2) \
                                                 +(BLE_MAX_ALLOW_PKT_PER_EVENT_RX * BLE_PKT_BUF_SIZE)   \
                                                  +BLE_PKT_BUF_SIZE)
                                                 
#define   BLE_CONN_BUF_SIZE                 (BLE_MAX_ALLOW_CONNECTION * BLE_MAX_ALLOW_PER_CONNECTION)
                                                                                        

uint8     g_pConnectionBuffer[BLE_CONN_BUF_SIZE] __attribute__ ((aligned(4)));
llConnState_t               pConnContext[BLE_MAX_ALLOW_CONNECTION];

#define LINKDB_CBS	5
linkDBItem_t plinkDB[BLE_MAX_ALLOW_CONNECTION];
pfnLinkDBCB_t plinkCBs[LINKDB_CBS];

llScheduleInfo_t         scheduleInfo[BLE_MAX_ALLOW_CONNECTION];

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
#define     LARGE_HEAP_SIZE  (1*1024)
uint8       largeHeap[LARGE_HEAP_SIZE];


#define DEFAULT_UART_BAUD   115200

//----------------------------------------------------------------------------------------------
//patch
extern volatile uint8_t g_same_rf_channel_flag;

//void app_sleep_process1(void);
//void app_wakeup_process1(void);
//void __ck802_cpu_wakeup_init(void)
//{
//    __set_VBR(0);
//
//    /* Clear active and pending IRQ */
//    VIC->IABR[0] = 0x0;
//    VIC->ICPR[0] = 0xFFFFFFFF;
//
//    __enable_excp_irq();
//    subWriteReg(0x40003030,4,4,1);//ck802 WFI enable
//}
//void hal_wakeup_irq_config(void)
//{
//    subWriteReg(0x40003030,4,4,1);//ck802 WFI enable
//    NVIC_SetWakeupIRQ(TIM1_IRQn);
//    NVIC_SetWakeupIRQ(TIM2_IRQn);
//    //NVIC_SetWakeupIRQ(TIM3_IRQn);
//    NVIC_SetWakeupIRQ(TIM4_IRQn);
//    NVIC_SetWakeupIRQ(BB_IRQn);
//
//}

static void rf_wakeup_handler(void){
  NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
}



static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;
    //============config xtal 16M cap
//    XTAL16M_CAP_SETTING(0x09);
////	XTAL16M_CURRENT_SETTING(0x01);
//    ble_main();
	hal_rom_code_ini();

#if 0
    //Quick Boot setting and 
     *(volatile uint32_t *) 0x4000f01c = 0x0000004;

	//========= low currernt setting IO init
    //========= pull all io to gnd by default
    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    *(volatile uint32_t *) 0x4000f010 = 0x36db6db6;//P20 - P29 pull down
    *(volatile uint32_t *) 0x4000f014 = 0xb0c3edb6;//P30 - P34 pull donw 

	DCDC_CONFIG_SETTING(0x0d);
	
    NVIC_SetPriority((IRQn_Type)BB_IRQn,    IRQ_PRIO_REALTIME);
    NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV

    pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);
#else
	//========= low currernt setting IO init
	//========= pull all io to gnd by default
	*(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
	*(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down

	//========= UART RX Pull up
	//hal_gpio_pull_set(P10,WEAK_PULL_UP);    

	NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
	NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
	NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
	NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV

	pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);
#endif

}

void hal_mpu_config(void)
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
    csi_mpu_config_region(1, 0x1fff8000, REGION_SIZE_16KB, attr1, 1);
    csi_mpu_enable();
}

void hal_gpio_IRQ(void)
{
	_symrom_GPIO_IRQHandler();
}
void hal_rom_code_ini(void)
{
#if 0
    boot_init();  
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
    //wakeup_init();
#else
	ble_main();
#endif

}
static void hal_init(void)
{
#if 0
   	volatile int ret;
    clk_init(g_system_clk);
	 
	//hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2|RET_SRAM3|RET_SRAM4);
	pwrmgr_RAM_retention_set();
  
    LOG_INIT();
    
	JUMP_FUNCTION_SET(GPIO_IRQ_HANDLER,(uint32_t)&hal_gpio_IRQ);	
	ret = gpio_init();
	if(ret != PPlus_SUCCESS)
	{
		LOG("error,stop,line:%d\n",__LINE__);
		while(1);
	}
	
    //hal_gpio_init();
	cache_init();
	//subWriteReg(&(AP_PCR->FLH_BUS_SEL),0,0,1); 
	hal_mpu_config();
    //hal_adc_init();
//	flash_sector_erase(0x3000);
//	uint32_t val = 0xfffffff0;
//	otp_prog_data_polling(0x1fff80f0,(uint32_t*)(&val),1);
#else
	pwrmgr_RAM_retention_set();
    otp_cache_init();
    LOG_INIT();
#endif
    LOG("all driver init OK!\n");
				
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
int  main(void)  
{
    g_system_clk = SYS_CLK_XTAL_16M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
    osal_mem_set_heap((osalMemHdr_t *)largeHeap, LARGE_HEAP_SIZE);
	LOG_INIT();
	two_stage_print();
	two_stage_app_load();
	
	
//    drv_irq_init();
//    init_config();
//	hal_pwrmgr_init();
//    hal_rfphy_init();
//	
////    hal_rom_code_ini();
//    hal_init();
//
//	gpio_write(P15,1);
////    LOG_INIT();
//
//	LL_InitConnectContext(pConnContext, scheduleInfo,
//                        g_pConnectionBuffer, 
//                        BLE_MAX_ALLOW_CONNECTION, 
//                        BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
//                        BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
//                        BLE_PKT_VERSION);
//	linkDB_InitContext( plinkDB,plinkCBs,LINKDB_CBS);
//	
//	LOG("===DEMO V2.2.1===\n");
//	LOG("0x%p,0x%p\n",&main,(uint16)&main);
//    LOG("rfClk %d sysClk %d tpCap[%02x %02x]\n",g_rfPhyClkSel,g_system_clk,g_rfPhyTpCal0,g_rfPhyTpCal1);
//
//    LOG("sizeof(struct ll_pkt_desc) = %d, buf size = %d\n", sizeof(struct ll_pkt_desc), BLE_CONN_BUF_SIZE);
//    LOG("g_pConnectionBuffer %p,sizeof(g_pConnectionBuffer) = %d, sizeof(pConnContext) = %d,\n", g_pConnectionBuffer,sizeof(g_pConnectionBuffer), sizeof(pConnContext));  
////	rf_phy_ini();
//
//    app_main();	

}


/////////////////////////////////////  end  ///////////////////////////////////////









	









