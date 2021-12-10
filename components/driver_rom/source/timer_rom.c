/*******************************************************************************
* @file   timer.c
* @brief  Contains all functions support for timer driver
* @version  0.0
* @date   27. Nov. 2017
* @author qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
/*
*rom use
*/
#include "jump_function.h"
#include "bus_dev.h"
#include "timer.h"
#include "clock.h"
#include "pwrmgr.h"

#ifndef FPGA_PERIPHERAL_TEST
#include "ll.h"
#include "ll_def.h"
#include "ll_common.h"
#endif
#include "mcu.h"

uint32_t get_current_time(AP_TIM_TypeDef *TIMx);
void  mask_timer_int(AP_TIM_TypeDef *TIMx, uint8_t en);
int   clear_timer_int(AP_TIM_TypeDef *TIMx);

// add by HZF
uint32 osal_sys_tick = 0;    
uint32 g_timer4_irq_pending_time;            // timer4 IRQ pending time

extern uint32_t g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t g_TIM2_IRQ_PendingTick;
extern uint32_t g_TIM1_IRQ_timing;
extern uint32 counter_tracking;
extern uint32_t g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32_t g_osal_tick_trim;
extern uint32_t g_TIM2_wakeup_delay;

void clear_timer(AP_TIM_TypeDef *TIMx)
{
	TIMx->ControlReg = 0;
	TIMx->LoadCount = 0;   //0x0
	TIMx->CurrentCount = 0;//0x4
}

//prime:cp timer(1~4) phy use,ap timer(1~4) customer use
//bumbee:ap timer(1~4) phy use,ap timer(5~6) customer use
//jack:total timer(1-4)
void TIM1_IRQHandler(void)
{
	// for JACK Strictly link layer slave  trig timing
	uint32_t TIM1_IRQ_timing = read_current_fine_time();
	uint32_t t1_IRQDelayCnt = AP_TIM1->LoadCount - AP_TIM1->CurrentCount ; 
	_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    if(AP_TIM1->status&0x1)   
    {  
        clear_timer_int(AP_TIM1);
        clear_timer(AP_TIM1);
		
#ifndef FPGA_PERIPHERAL_TEST
	LL_evt_schedule();
#endif
    }
	
	g_TIM1_IRQ_timing = LL_TIME_DELTA(TIM1_IRQ_timing,read_current_fine_time() )   ;
	g_TIM1_IRQ_timing += ( t1_IRQDelayCnt >> 2);
	
	HAL_EXIT_CRITICAL_SECTION();	
}

void TIM2_IRQHandler(void)
{
	_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    if(AP_TIM2->status&0x1)   
    {  
#ifndef FPGA_PERIPHERAL_TEST
        g_TIM2_IRQ_TIM3_CurrCount = AP_TIM3->CurrentCount;
        g_TIM2_IRQ_PendingTick = AP_TIM2->CurrentCount;
#endif
		
	clear_timer_int(AP_TIM2);
        
#ifndef FPGA_PERIPHERAL_TEST
	osal_sys_tick ++; 
#endif
    }
	HAL_EXIT_CRITICAL_SECTION();
}

void TIM3_IRQHandler(void)
{
	_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    if(AP_TIM3->status&0x1)
    {     
        clear_timer_int(AP_TIM3);   
    }
	HAL_EXIT_CRITICAL_SECTION();
}

//void TIM4_IRQHandler(void)
//{
//	_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
//    if(AP_TIM4->status&0x1)
//    {     
//    	g_timer4_irq_pending_time = AP_TIM4->CurrentCount - AP_TIM4->LoadCount;    // TODO: check the formula
//		clear_timer_int(AP_TIM4);
//		clear_timer(AP_TIM4);
//#ifndef FPGA_PERIPHERAL_TEST
//    	if (g_currentTimerTask == LL_TASK_EXTENDED_ADV)
//		     LL_extAdvTimerExpProcess();		
//        else if (g_currentTimerTask == LL_TASK_PERIODIC_ADV)
//		     LL_prdAdvTimerExpProcess();			  
//		else if (g_currentTimerTask == LL_TASK_EXTENDED_SCAN)
//			 LL_extScanTimerExpProcess();		  
//        else if (g_currentTimerTask == LL_TASK_EXTENDED_INIT)
//		     LL_extInitTimerExpProcess();		  
//		else if (g_currentTimerTask == LL_TASK_PERIODIC_SCAN)
//		     LL_prdScanTimerExpProcess();		  	     
//#endif
//    }
//	HAL_EXIT_CRITICAL_SECTION();
//}

/////////////////////  timer1~4  ///////////////////////////////
void set_timer(AP_TIM_TypeDef *TIMx, int time)
{
    if(time > 0)
    {
        TIMx->ControlReg=0x0;
        TIMx->ControlReg=0x2;
        TIMx->LoadCount = time * 4;           // 4MHz system timer, * 4 to convert to 1MHz timer
        TIMx->ControlReg=0x3;
        switch((uint32_t)TIMx){
        case AP_TIM1_BASE:
            NVIC_EnableIRQ(TIM1_IRQn);
            break;
        case AP_TIM2_BASE:
            NVIC_EnableIRQ(TIM2_IRQn);
            break;
        case AP_TIM3_BASE:
            NVIC_EnableIRQ(TIM3_IRQn);
            break;
//        case AP_TIM4_BASE:
//            NVIC_EnableIRQ(TIM4_IRQn);
            break;
        }
    }
    else
    {
        switch((uint32_t)TIMx){
        case AP_TIM1_BASE:
            NVIC_DisableIRQ(TIM1_IRQn);
            break;
        case AP_TIM2_BASE:
            NVIC_DisableIRQ(TIM2_IRQn);
            break;
        case AP_TIM3_BASE:
            NVIC_DisableIRQ(TIM3_IRQn);
            break;
//        case AP_TIM4_BASE:
//            NVIC_DisableIRQ(TIM4_IRQn);
//            break;
        }
        TIMx->ControlReg=0x0;
    }
}

uint32_t get_timer_count(AP_TIM_TypeDef *TIMx)
{
    return(TIMx->CurrentCount);    
}

int clear_timer_int(AP_TIM_TypeDef *TIMx)
{
    return TIMx->EOI;
}

int get_timer_int(AP_TIM_TypeDef *TIMx)
{
    return (TIMx->status &0x1);
}

uint8 isTimer1Running(void)
{
    if (AP_TIM1->ControlReg & 0x01)
		return TRUE;
    else
		return FALSE;
}

uint8 isTimer4Running(void)//this timer will release to user
{
    if (AP_TIM4->ControlReg & 0x01)
		return TRUE;
    else
		return FALSE;
}

uint32_t read_current_fine_time(void)
{
    return (BASE_TIME_UNITS - (get_timer_count(AP_TIM3) >> 2) + 2);
}

// get remainder LL counter
uint32  read_LL_remainder_time(void)
{
    uint32 currentCount;
    uint32 g_tim1_pass = read_current_fine_time();
    currentCount = AP_TIM1->CurrentCount;

    if((currentCount < 6) || NVIC_GetPendingIRQ(TIM1_IRQn))
        return 0;
    else
        return (currentCount >> 2);

}

// add by HZF
uint32 getMcuPrecisionCount(void)
{
    return osal_sys_tick;
}

/*
*sdk use
*/
#include "error.h"
ap_tm_hdl_t s_ap_callback = NULL;

void __attribute__((used)) hal_TIMER4_IRQHandler(void)
{
    if(AP_TIM4->status & 0x1)
    {        
		AP_TIM4->EOI;//hal_timer_clear_int(AP_TIM4);
        if(s_ap_callback)
            s_ap_callback(HAL_EVT_TIMER_4);
    }
}
void hal_timer_wakeup_handler(void)
{
    if(s_ap_callback)
        s_ap_callback(HAL_EVT_WAKEUP);
}

void hal_timer_sleep_handler(void)
{
    if(s_ap_callback)
        s_ap_callback(HAL_EVT_SLEEP);
}

int hal_timer_mask_int(User_Timer_e timeId, bool en)
{
	if(AP_TIMER_ID_4 == timeId)
	{
		if(en)
			AP_TIM4->ControlReg |= (1 << 2);
		else
		AP_TIM4->ControlReg &= ~(1 << 2);

		return PPlus_SUCCESS;
	}
	return PPlus_ERR_INVALID_PARAM;
}

int hal_timer_set(User_Timer_e timeId, uint32_t us)
{
    switch(timeId)
    {
		case AP_TIMER_ID_4:
			//JUMP_FUNCTION_SET(TIM4_IRQ_HANDLER,(uint32_t)&hal_TIMER4_IRQHandler);
			NVIC_EnableIRQ((IRQn_Type)TIM4_IRQn);
			NVIC_SetPriority((IRQn_Type)TIM4_IRQn, IRQ_PRIO_HAL);
	
			if(us>0)//hal_timer_set_loadtimer(AP_TIM4, us);
			{
				AP_TIM4->ControlReg = 0x0;
				AP_TIM4->ControlReg = 0x2;
				AP_TIM4->LoadCount = 4*us;// 4MHz system timer, * 4 to convert to 1MHz timer
				AP_TIM4->ControlReg = 0x3;
			}
			else
			{
				AP_TIM4->ControlReg = 0x0;
			}
			clk_gate_enable(MOD_TIMER4);
			break;

		default:
			return PPlus_ERR_INVALID_PARAM;
    }

    return PPlus_SUCCESS;
}

int hal_timer_stop(User_Timer_e timeId)
{
	switch(timeId)
		{
		case AP_TIMER_ID_4:
			//JUMP_FUNCTION_SET(TIM4_IRQ_HANDLER,0);
			
			AP_TIM4->ControlReg = 0;//hal_timer_stop_counter(AP_TIM4);
			AP_TIM4->LoadCount = 0;   //0x0
			AP_TIM4->CurrentCount = 0;//0x4
	
			NVIC_DisableIRQ((IRQn_Type)TIM4_IRQn);
			clk_gate_disable(MOD_TIMER4);
			break;

		default:
			return PPlus_ERR_INVALID_PARAM;
		}

    return PPlus_SUCCESS;
}

int hal_timer_init(ap_tm_hdl_t callback)
{
    s_ap_callback = callback;
    hal_timer_stop(AP_TIMER_ID_4);
    return pwrmgr_register(MOD_TIMER, hal_timer_sleep_handler, hal_timer_wakeup_handler);
}

int hal_timer_deinit(void)
{
    s_ap_callback = NULL;
    return pwrmgr_unregister(MOD_TIMER);
}
