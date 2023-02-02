#include "jump_function.h"
#include "watchdog.h"
#include "error.h"
#include "pwrmgr.h"
#include "clock.h"
#include "OSAL_Timers.h"

#define WATCHDOG_1000MS_EVENT    0x0001


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

	return TRUE;
}
//extern uint32_t s_config_swClk1;
//void hal_watchdog_init(void)
//{
//
//	watchdog_init(g_wdt_cycle,/*int_mode*/0);//wdt polling_mode
//	s_config_swClk1|=_CLK_WDT;
//}
//
//int hal_watchdog_config(WDG_CYCLE_Type_e cycle)
//{
//    if(cycle > WDG_256S)
//        return PPlus_ERR_INVALID_PARAM;
//    else
//        g_wdt_cycle = cycle;
//
//    hal_watchdog_init();
//    JUMP_FUNCTION_SET(HAL_WATCHDOG_INIT, (uint32_t)&hal_watchdog_init);
//	return PPlus_SUCCESS;
//    
//}


