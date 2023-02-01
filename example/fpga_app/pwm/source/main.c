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
extern void set_timer(AP_TIM_TypeDef *TIMx, int time);
extern  uint32_t pclk;



#define     LARGE_HEAP_SIZE  256
uint8       g_largeHeap[LARGE_HEAP_SIZE];

static void rf_wakeup_handler(void){
  NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
}

void boot_init(void);

void wakeup_init(void);

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
//void hal_mpu_config(void)
//{
//    mpu_region_attr_t attr0 = {
//        .nx = 0,
//        .ap = AP_BOTH_RW,
//        .s = 0,
//    };
//    mpu_region_attr_t attr1 = {
//        .nx = 0,
//        .ap = AP_BOTH_RW,
//        .s = 1,
//    };
//
//    csi_mpu_config_region(0, 0,          REGION_SIZE_4GB, attr0, 1);
//    csi_mpu_config_region(1, 0x1fff8000, REGION_SIZE_16KB, attr1, 1);
//    csi_mpu_enable();
//}

void hal_gpio_IRQ(void)
{
	_symrom_GPIO_IRQHandler();
}

void hal_init(void)
{
	volatile int ret;
    //clk_init(g_system_clk);
	 
	//hal_pwrmgr_RAM_retention(RET_SRAM0|RET_SRAM1|RET_SRAM2|RET_SRAM3|RET_SRAM4);
	pwrmgr_RAM_retention_set();
    otp_cache_init();
    LOG_INIT();
    
	JUMP_FUNCTION_SET(GPIO_IRQ_HANDLER,(uint32_t)&hal_gpio_IRQ);	
	ret = gpio_init();
	if(ret != PPlus_SUCCESS)
	{
		LOG("error,stop,line:%d\n",__LINE__);
		while(1);
	}
	
    //hal_gpio_init();
//	hal_mpu_config();
    //hal_adc_init();
//	flash_sector_erase(0x3000);
//	uint32_t val = 0xfffffff0;
//	otp_prog_data_polling(0x1fff80f0,(uint32_t*)(&val),1);
	
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

    hal_rom_code_ini();
    
    //Quick Boot setting and 
     *(volatile uint32_t *) 0x4000f01c = 0x0000004;       //  3'b1xx: 62.5us.  control bits for main digital part reset wait time after power up charge pump. 

    //========= low currernt setting IO init
    //========= pull all io to gnd by default
    *(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
    *(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    *(volatile uint32_t *) 0x4000f010 = 0x36db6db6;//P20 - P29 pull down
    *(volatile uint32_t *) 0x4000f014 = 0xb0c3edb6;//P30 - P34 pull donw

    //========= UART RX Pull up
    //hal_gpio_pull_set(P10,WEAK_PULL_UP);    

    DCDC_CONFIG_SETTING(0x0d);


    NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
	NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
    NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
    NVIC_SetPriority((IRQn_Type)TIM4_IRQn,  IRQ_PRIO_HIGH);     //LL_EXA_ADV

    pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);

}

void HAL_wakeup_process(void)
{
	 LOG_INIT();
}
//extern uint32 global_config[];
extern uint32_t  __initial_sp;
/* extern */ uint32_t  g_smartWindowSize;

//uint32* pGlobal_config;

void init_config(void)
{
    int i;

 
//   pGlobal_config = global_config;

    //Config the ROM_CODE global_config pointer 2018-3-20
       
    
    for (i = 0; i < SOFT_PARAMETER_NUM; i ++)
        pGlobal_config[i] = 0;

    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;
    
    // LL switch setting
    pGlobal_config[LL_SWITCH] =  SLAVE_LATENCY_ALLOW | RC32_TRACKINK_ALLOW
                                | LL_RC32K_SEL | SIMUL_CONN_SCAN_ALLOW |RC32_CAP_ADAPTIVE_TRIM; //
    
 
    
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

    pGlobal_config[WAKEUP_ADVANCE] = 1350;//650;//600;//310;
       
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[WAKEUP_DELAY] = 16;
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
    pGlobal_config[LL_HW_PLL_DELAY] = 52;

    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV] = 58;//57;			// 2019/3/20 A2: 57 --> 58
    pGlobal_config[LL_HW_Tx_TO_RX_INTV] = 49;//50; //65		// 2019/3/20 A2: 50 --> 49

    //------------------------------------------------2MPHY
    // LL engine settle time 
    pGlobal_config[LL_HW_BB_DELAY_2MPHY] = 59;
    pGlobal_config[LL_HW_AFE_DELAY_2MPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_2MPHY] = 52;
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
    pGlobal_config[SLAVE_CONN_DELAY] = 600;//0;//1500;//0;//3000;//0;          ---> update 11-20
    pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 800;
		
    // RTLP timeout
    pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] = 50000;
    //pGlobal_config[LL_HW_RTLP_TO_GAP]       = 1000;
    
    pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]  = 2500 + pGlobal_config[SLAVE_CONN_DELAY] * 2;//500;


    // direct adv interval configuration
    pGlobal_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
    pGlobal_config[LDC_DIRECT_ADV_INTERVAL] = 6250;

    // A1 ROM metal change for HDC direct adv, 
    pGlobal_config[DIR_ADV_DELAY] = 115;   // in us, consider both direct adv broadcast time & SW delay, ... etc.
    
    // A1 ROM metal change
    //pGlobal_config[LL_TX_PKTS_PER_CONN_EVT] = 6;//8;
    //pGlobal_config[LL_RX_PKTS_PER_CONN_EVT] = 6;//8;    

    //pGlobal_config[LL_TRX_NUM_ADAPTIVE_CONFIG] = 8;     //0:        disable adaptive 
                                                        //other:    adaptive max limitation
    g_smartWindowSize = pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT];

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
    
    pGlobal_config[MAC_ADDRESS_LOC] = 0x1fff8100;//0x11004000;
    
    // for simultaneous conn & adv/scan
    //pGlobal_config[LL_NOCONN_ADV_EST_TIME] = 1400;
    //pGlobal_config[LL_NOCONN_ADV_MARGIN] = 600;
    
    pGlobal_config[LL_SEC_SCAN_MARGIN] = 1400;
    pGlobal_config[LL_MIN_SCAN_TIME] = 2000;
    
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

	//JUMP_FUNCTION_SET(APP_WAKEUP_PROCESS,(uint32_t)&HAL_wakeup_process);


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
	
    g_system_clk = SYS_CLK_XTAL_16M; //SYS_CLK_DLL_48M;//SYS_CLK_RC_32M; //SYS_CLK_DLL_64M; //
    osal_mem_set_heap((osalMemHdr_t *)g_largeHeap, LARGE_HEAP_SIZE);
	
    drv_irq_init();
    
    init_config();
    //pwrmgr_config(PWR_MODE_SLEEP);
    hal_pwrmgr_init();
	//pwrmgr_init();
//	pwrmgr_register(MOD_USR0, NULL, rf_wakeup_handler);

//    hal_rfphy_init();  
	hal_rom_code_ini();
    hal_init();
	//LOG("RTC Clock started\n");
	LOG_DEBUG("start!\n");
	LOG("rtc cnt %d\n",rtc_get_counter());

    app_main();	
}

///////////////////////////////////  end  ///////////////////////////////////////

