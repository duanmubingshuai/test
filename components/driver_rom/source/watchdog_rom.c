#include "jump_function.h"
#include "watchdog.h"
#include "error.h"
#include "pwrmgr.h"
#include "clock.h"
#include "OSAL_Timers.h"

uint8 watchdog_TaskID;
#define WATCHDOG_1000MS_EVENT    0x0001

WDG_CYCLE_Type_e r_wdg_cycle = WDG_2S;
bool r_wdg_int_mode = FALSE;
uint32_t wdg_ms_cycle = 1000;//unit:ms

#if 1
void __attribute__((used))  WDT_IRQHandler(void)
{
    //volatile uint32_t a;
    AP_WDT->EOI;
    AP_WDT->CRR = 0x76;
	
}

#else

//extern bool feed_watchdog_in_cycle;
//extern uint8_t feed_wdg_int;
//void __attribute__((used))  WDT_IRQHandler(void)
//{
//	if(feed_watchdog_in_cycle == TRUE)
//	{
//		volatile uint32_t a;
//		a = AP_WDT->EOI;
//		AP_WDT->CRR = 0x76;	
//		
//		feed_wdg_int = 1;
//	}
//}

#endif

void watchdog_feed(void)
{
    AP_WDT->CRR = 0x76;
}

bool watchdog_init(WDG_CYCLE_Type_e cycle,bool int_mode)
{
    //volatile uint32_t a;
	uint8_t delay;	
	
	clk_gate_enable(MOD_WDT);        
	if((AP_PCR->SW_RESET0 & 0x04)==0)
	{
		AP_PCR->SW_RESET0 |= 0x04;
		delay = 20;
		while(delay-->0);
	}

	if((AP_PCR->SW_RESET2 & 0x04)==0)
	{    
		AP_PCR->SW_RESET2 |= 0x04;
		delay=20;
		while(delay-->0);
	}
  
	AP_PCR->SW_RESET2 &= ~0x20;
	delay=20;
	while(delay-->0);
  
	AP_PCR->SW_RESET2 |= 0x20;
	delay=20;
	while(delay-->0);

    AP_WDT->EOI; 
	AP_WDT->TORR = cycle;

	if(TRUE == int_mode)
	{
//		JUMP_FUNCTION(WDT_IRQ_HANDLER)                  =   (uint32_t)&WDT_IRQHandler;
//		JUMP_FUNCTION_SET(WDT_IRQ_HANDLER,(uint32_t)&WDT_IRQHandler);
//		JUMP_FUNCTION_SET(WDT_IRQ_HANDLER,0);


		AP_WDT->CR = 0x1F;//use int
		NVIC_SetPriority((IRQn_Type)WDT_IRQn, IRQ_PRIO_HAL);
		NVIC_EnableIRQ((IRQn_Type)WDT_IRQn);
	}
	else  
	{
//		JUMP_FUNCTION_SET(WDT_IRQ_HANDLER,0);
		AP_WDT->CR = 0x1D;//no use int
		NVIC_DisableIRQ((IRQn_Type)WDT_IRQn);
	}

    AP_WDT->CRR = 0x76;
#ifdef FPGA_ROM_DRIVER_TEST	
#else
    osal_start_reload_timer(watchdog_TaskID, WATCHDOG_1000MS_EVENT, wdg_ms_cycle);
#endif	

	return TRUE;

}

void watchdog_wakeup_handler(void)
{
    watchdog_init(r_wdg_cycle,r_wdg_int_mode);
}

void watchdog_sleep_handler(void)
{
    osal_stop_timerEx(watchdog_TaskID, WATCHDOG_1000MS_EVENT);
}

void Watchdog_Init(uint8 task_id)
{
    watchdog_TaskID = task_id;
    watchdog_init(r_wdg_cycle,r_wdg_int_mode);
    pwrmgr_register(MOD_WDT, watchdog_sleep_handler, watchdog_wakeup_handler);
}

uint16 Watchdog_ProcessEvent(uint8 task_id, uint16 events)
{
    if(events & WATCHDOG_1000MS_EVENT)
    {
        watchdog_feed();
        return (events ^ WATCHDOG_1000MS_EVENT);
    }

    return 0;
}
