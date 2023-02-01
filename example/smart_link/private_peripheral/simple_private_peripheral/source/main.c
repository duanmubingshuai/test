#include "rom_sym_def.h"

//#include <stdio.h>
#include "bus_dev.h"
#include "uart.h"
#include "otp.h"
#include "gpio.h"
#include "clock.h"
#include "timer.h"
#include "ll.h"
#include "ll_def.h"
#include "simplePrivateperipheral.h"
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
#include "version.h"
#include "watchdog.h"

extern int app_main(void);
extern void linkDB_InitContext( linkDBItem_t *linkDBs,pfnLinkDBCB_t *linkCB,uint8 cb_maxs);


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

//#define LINKDB_CBS	5
//linkDBItem_t plinkDB[BLE_MAX_ALLOW_CONNECTION];
//pfnLinkDBCB_t plinkCBs[LINKDB_CBS];

llScheduleInfo_t         scheduleInfo[BLE_MAX_ALLOW_CONNECTION];


/*********************************************************************
    OSAL LARGE HEAP CONFIG
*/
#define     LARGE_HEAP_SIZE  (512)
uint8      g_largeHeap[LARGE_HEAP_SIZE] __attribute__ ((aligned(4)));



static void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;
    //============config xtal 16M cap
    XTAL16M_CAP_SETTING(0x09);
    XTAL16M_CURRENT_SETTING(0x03);

	extern void hal_rom_code_ini(void);
	hal_rom_code_ini();

	NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
	NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
	NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
}



static void hal_init(void)
{
    
	//========= low currernt setting IO init
	//========= pull all io to gnd by default, except P07,P15(keep floating)
    aon_ioctl0_pack(3,0,3,0,/*p7*/0,0,3,0,3,0,3,0,3,0,3,0,3,0,3,0); /*P0~P9*/
    aon_ioctl1_pack(3,0,3,0,3,0,3,0,/*p15*/0,0,3,0,3,0,3,0,3,0,3,0);/*P10~P19*/
        
    PMU_HIGH_LDO_ENABLE(0);
    DIG_LDO_CURRENT_SETTING(0x01);
	pwrmgr_RAM_retention_set();
    pwrmgr_LowCurrentLdo_enable();
    
	hal_pwrmgr_init();
    otp_cache_init();
    LOG_INIT();
				
}

static void hal_mem_init_config(void)
{
    osal_mem_set_heap((osalMemHdr_t*)g_largeHeap, LARGE_HEAP_SIZE);
    
	LL_InitConnectContext(pConnContext, scheduleInfo,
                        g_pConnectionBuffer, 
                        BLE_MAX_ALLOW_CONNECTION, 
                        BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
                        BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
                        BLE_PKT_VERSION);
//	linkDB_InitContext( plinkDB,plinkCBs,LINKDB_CBS);
    
}


//uint8_t bt_addr[6]={0x99,0x66,0x03,0x0c,0x62,0x30};
uint8_t bt_addr[6]={0x49,0x49,0x49,0x49,0x71,0x71};
extern void ll_set_ble_mac_addr(uint32_t macAddr);
extern void patch_slave(void);
extern void init_config(void);
/////////////////////////////////////////////////////////////////////////////////////////////////////////
int  main(void)  
{
//    hal_watchdog_config(WDG_2S);
    g_system_clk = SYS_CLK_DLL_48M;//SYS_CLK_XTAL_16M;//SYS_CLK_DLL_48M;
	
    hal_mem_init_config();
    
    drv_irq_init();
    init_config();
	patch_slave();
    hal_rfphy_init();
    hal_init();
    
#if(_DEF_DTM_BUILD_)
    rf_phy_dtm_ate();
#endif
    
    ll_set_ble_mac_addr((uint32)&bt_addr);
	
//	LOG_DEBUG("SDK Version ID %08x \n",SDK_VER_RELEASE_ID);
    LOG_DEBUG("rfClk %d sysClk %d tpCap[%02x %02x] rcCal[%02x]\n",g_rfPhyClkSel,g_system_clk,g_rfPhyTpCal0,g_rfPhyTpCal1,0x7f&read_reg(0x4000f018));
//
//    LOG_DEBUG("sizeof(struct ll_pkt_desc) = %d, buf size = %d\n", sizeof(struct ll_pkt_desc), BLE_CONN_BUF_SIZE);
//    LOG_DEBUG("g_pConnectionBuffer %p,sizeof(g_pConnectionBuffer) = %d, sizeof(pConnContext) = %d,\n",
//    g_pConnectionBuffer,sizeof(g_pConnectionBuffer), sizeof(pConnContext));  
    LOG_DEBUG("[REST CAUSE] %d\n ",g_system_reset_cause);
    app_main();	

}


/////////////////////////////////////  end  ///////////////////////////////////////









	









