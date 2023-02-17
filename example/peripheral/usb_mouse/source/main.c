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
#include "otp.h"
#include "usb_pcd.h"

#include "OSAL_Tasks.h"

extern void  ble_main(void);

extern void hal_rom_code_ini(void);
extern int app_main(void);
extern void init_config(void);
extern void drv_irq_init(void);
extern void set_timer(AP_TIM_TypeDef *TIMx, int time);
extern  uint32_t pclk;
extern void trap_c(uint32_t *regs);



#define     LARGE_HEAP_SIZE  128
uint8       g_largeHeap[LARGE_HEAP_SIZE];
uint32_t trap_stack[76/4];




void init_config(void)
{
    extern uint32_t     __initial_sp;
    int i;
    
    for (i = 0; i < SOFT_PARAMETER_NUM; i ++)
        pGlobal_config[i] = 0;
    
    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;
    

	extern void osalInitTasks( void );
	extern pTaskEventHandlerFn tasksArr[];
	extern uint16 tasksCnt;
	extern uint16 *tasksEvents;
	JUMP_FUNCTION_SET(OSAL_INIT_TASKS,(uint32_t)&osalInitTasks);
	JUMP_FUNCTION_SET(TASKS_ARRAY,(uint32_t)&tasksArr);
	JUMP_FUNCTION_SET(TASK_COUNT ,(uint32_t)&tasksCnt);
	JUMP_FUNCTION_SET(TASK_EVENTS,(uint32_t)&tasksEvents);

}



void hal_rfphy_init(void)
{
    //============config the txPower
    g_rfPhyTxPower  = RF_PHY_TX_POWER_0DBM ;
    //============config BLE_PHY TYPE
    g_rfPhyPktFmt   = PKT_FMT_BLE1M;
    //============config RF Frequency Offset
    g_rfPhyFreqOffSet   =RF_PHY_FREQ_FOFF_00KHZ;
    XTAL16M_CURRENT_SETTING(0x01);
	
	//clk_init(SYS_CLK_XTAL_16M);
	//wait_hclk_cycle_us(1000);

	ble_main();//hal_rom_code_ini();

	NVIC_SetPriority(BB_IRQn, IRQ_PRIO_REALTIME);
	NVIC_SetPriority((IRQn_Type)TIM1_IRQn,  IRQ_PRIO_HIGH);     //ll_EVT
	NVIC_SetPriority((IRQn_Type)TIM2_IRQn,  IRQ_PRIO_HIGH);     //OSAL_TICK
}

void hid_send_key(char data)
{
	if(data == 'n'){
		LOG_ERROR("send:\n");
	}
	else{
		LOG_ERROR("part:\n");
		
	}
}

//#define hid_send_key(a)
void kbd_uart_cbk(comm_evt_t* pev)
{
	gpio_write(P8, 1);
	gpio_write(P8, 0);
	
	if(pev->type == UART_EVT_TYPE_RX_DATA){
		for(int i = 0; i< pev->len; i++)
			hid_send_key(pev->data[i]);
	}
}


void hal_init(void)
{
    
	//========= low currernt setting IO init
	//========= pull all io to gnd by default
	//*(volatile uint32_t *) 0x4000f008 = 0x36db6db6;//P00 - P09 pull down
	//*(volatile uint32_t *) 0x4000f00c = 0x36db6db6;//P10 - P19 pull down
    extern void clk_set_usb();
    clk_set_usb();
	
	JUMP_FUNCTION_SET(V26_IRQ_HANDLER,(uint32_t)&USB_IRQHandler);

    PMU_HIGH_LDO_ENABLE(1);
    DIG_LDO_CURRENT_SETTING(0x01);
	pwrmgr_RAM_retention_set();
    pwrmgr_LowCurrentLdo_enable();
    
	hal_pwrmgr_init();
	
	gpio_init();
	
    otp_cache_init();
    //LOG_INIT();
    //swu_uart_init(115200,P4,P5,kbd_uart_cbk);
				
}

void hal_mem_init_config(void)
{
    osal_mem_set_heap((osalMemHdr_t*)g_largeHeap, LARGE_HEAP_SIZE);
    
	//LL_InitConnectContext(pConnContext, scheduleInfo,
    //                    g_pConnectionBuffer, 
    //                    BLE_MAX_ALLOW_CONNECTION, 
    //                    BLE_MAX_ALLOW_PKT_PER_EVENT_TX,
    //                    BLE_MAX_ALLOW_PKT_PER_EVENT_RX,
    //                    BLE_PKT_VERSION);
	//linkDB_InitContext( plinkDB,plinkCBs,LINKDB_CBS);
    
}



int  main(void)  
{     

    // init global configuration of SOC 
	clk_init(SYS_CLK_XTAL_16M);
    g_system_clk = SYS_CLK_DLL_48M; //SYS_CLK_XTAL_16M; //SYS_CLK_RC_32M; //SYS_CLK_DLL_64M; //
	
    hal_mem_init_config();
    
	trap_set_stack(trap_stack, sizeof(trap_stack));
    JUMP_FUNCTION_SET(CK802_TRAP_C, (uint32_t)trap_c);
	
    drv_irq_init();
    extern void init_config(void);
    init_config();
    hal_rfphy_init();
    hal_init();

	//LOG_ERROR("[REST CAUSE] %d\n ",g_system_reset_cause);

	app_main();	
}

///////////////////////////////////  end  ///////////////////////////////////////

