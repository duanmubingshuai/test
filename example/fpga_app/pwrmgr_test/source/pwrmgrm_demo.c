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
  Filename:     
  Revised:       
  Revision:      


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "OSAL.h"
#include "pwrmgrm_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"

//#define LOG(...)  log_printf(__VA_ARGS__)

uint8 pwrmgr_TaskID;

extern uint32 t_tmp, p_tmp;
uint32 t_tmp1=0, p_tmp1=0;
uint32 t_tmp2=0, p_tmp2=0;
extern uint32 s_num;

static void gpio_wakeup_handler(void){
	t_tmp1++;
	NVIC_SetPriority(GPIO_IRQn, IRQ_PRIO_REALTIME);
//	NVIC_EnableIRQ(GPIO_IRQn);
}

static void wdt_wakeup_handler(void){
	t_tmp2++;
	NVIC_SetPriority(WDT_IRQn, IRQ_PRIO_REALTIME);
}

void test1(void)
{
	p_tmp1++;
}

void test2(void)
{
	p_tmp2++;
}


void pwrmgrm_Init(uint8 task_id)
{	
//	volatile int ret;	
	gpio_pupd_e type = GPIO_PULL_DOWN;
	
	pwrmgr_TaskID = task_id;
	LOG("pwrmgr_demo\n");
	
	pwrmgr_register(MOD_USR1, test1, gpio_wakeup_handler);
	pwrmgr_register(MOD_USR2, test2, wdt_wakeup_handler);
	
//	if(ret != PPlus_SUCCESS)
//	{
//		LOG("error:%d,stop,line:%d\n",ret,__LINE__);
//		while(1);
//	}
	if(s_num < 5)
	{
		LOG("error:%d,stop_line:%d\n",s_num,__LINE__);
		while(1);
	}
		
	osal_set_event(pwrmgr_TaskID,OSAL_SET_EVENT_EVT);
	LOG("set event\n");
	osal_start_timerEx(pwrmgr_TaskID, OSAL_ONCE_TIMER_EVT, 1000);
	osal_start_reload_timer(pwrmgr_TaskID, OSAL_RELOAY_TIMER_EVT, 1500);
}


uint16 pwrmgrm_ProcessEvent( uint8 task_id, uint16 events )
{
	static uint32_t once_timer_counter = 0;
//	static uint32_t cycle_timer_counter = 0;

	if(task_id != pwrmgr_TaskID){
		return 0;
	}

	if ( events & OSAL_SET_EVENT_EVT )
	{
		LOG("osal_set_event runs ok\n");
		return ( events ^ OSAL_SET_EVENT_EVT );
	}
//	LOG("start osal once timer\n");
	if ( events & OSAL_ONCE_TIMER_EVT )
	{
		LOG("once_timer_counter:%d\n",once_timer_counter++);
		LOG("s_num:%d\n",s_num);
		LOG("t_tmp:%d\n",t_tmp);
		LOG("t_tmp1:%d\n",t_tmp1);
		LOG("t_tmp2:%d\n",t_tmp2);
		LOG("p_tmp:%d\n",p_tmp);
		LOG("p_tmp1:%d\n",p_tmp1);
		LOG("p_tmp2:%d\n",p_tmp2);
		
		osal_start_timerEx(pwrmgr_TaskID, OSAL_ONCE_TIMER_EVT, 1000);

		if(once_timer_counter%2)
		{
			;
		}
		else
		{
			;
		}
		return ( events ^ OSAL_ONCE_TIMER_EVT );
	}
	
  return 0;
}
