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
#include "gpio_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "jump_function.h"

//#define LOG(...)  log_printf(__VA_ARGS__)
#define GPIO_TEST_NUM 				4

uint8 application_TaskID;

gpioin_t pin_test[GPIO_TEST_NUM];
gpio_pin_e pin_o[2] = {P0,P1};
gpio_pin_e pin_i[2] = {P14,P15};

volatile unsigned int wakeup_cnt = 0;
volatile unsigned int int_cnt = 0;
extern uint32_t s_gpio_wakeup_src;



void pos_cb(gpio_pin_e pin,gpio_polarity_e type)
{
	uint8_t pin_idx = pin;
  _HAL_CS_ALLOC_();
  HAL_ENTER_CRITICAL_SECTION();
	
	if(s_gpio_wakeup_src)
	{
    LOG("WWWWWWWWWWWWWWWWWWW: pin:%d (pos) 0x%x, %d\n",pin,s_gpio_wakeup_src,wakeup_cnt);
		wakeup_cnt++;
	}
	else
	{
		LOG("IIIIIIIIIIIIIIIIIII: pin:%d (pos)       %d\n",pin,int_cnt);
		int_cnt++;
	}
	s_gpio_wakeup_src = 0;
	HAL_EXIT_CRITICAL_SECTION();
}

void neg_cb(gpio_pin_e pin,gpio_polarity_e type)
{
	uint8_t pin_idx = pin;
  _HAL_CS_ALLOC_();
	HAL_ENTER_CRITICAL_SECTION();

	if(s_gpio_wakeup_src)
	{
    LOG("WWWWWWWWWWWWWWWWWWW: pin:%d (neg) 0x%x, %d\n",pin,s_gpio_wakeup_src,wakeup_cnt);
		wakeup_cnt++;
	}
	else
	{
		LOG("IIIIIIIIIIIIIIIIIII: pin:%d (neg)       %d\n",pin,int_cnt);
		int_cnt++;
	}
	s_gpio_wakeup_src = 0;
	HAL_EXIT_CRITICAL_SECTION();
}

void gpio_Init(uint8 task_id)
{	
	volatile int ret;	
	gpio_pupd_e type = GPIO_PULL_DOWN;
	
	application_TaskID = task_id;
	LOG("GPIO_demo\n");
	
	
	
	

	ret = gpioin_init(pin_test,sizeof(pin_test)/sizeof(pin_test[0]));
	if(ret != PPlus_SUCCESS)
	{
		LOG("error:%d,stop,line:%d\n",ret,__LINE__);
		while(1);
	}
	
	for(int i=0;i<2;i++)
	{
		gpio_pull_set(pin_i[i],type);
		ret = gpioin_register(pin_i[i],pos_cb,neg_cb);
		if(PPlus_SUCCESS != ret)
		{
			LOG("error detect,please check:%d\n",__LINE__);
		}
	}

	for(int i=0;i<2;i++)
	{
		gpio_dir(pin_o[i],GPIO_OUTPUT);
		gpio_retention(pin_o[i],TRUE);
	}
	osal_set_event(application_TaskID,OSAL_SET_EVENT_EVT);
	//osal_start_timerEx(application_TaskID, OSAL_ONCE_TIMER_EVT, 1000);
	//osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 1500);
}

uint32 t_tmp;
uint32 p_tmp;
uint16 gpio_ProcessEvent( uint8 task_id, uint16 events )
{
	static uint32_t once_timer_counter = 0;
	static uint32_t cycle_timer_counter = 0;
	LOG("gpio_ProcessEvent %x\n", events);
	if(task_id != application_TaskID){
		return 0;
	}

	if ( events & OSAL_SET_EVENT_EVT )
	{
		LOG("osal_set_event runs ok\n");
		return ( events ^ OSAL_SET_EVENT_EVT );
	}

	if ( events & OSAL_ONCE_TIMER_EVT )
	{
		LOG("\nonce_timer_counter:%d\n",once_timer_counter++);
		LOG("t_tmp:%d\n",t_tmp);
		LOG("p_tmp:%d\n",p_tmp);
		LOG("int_cnt:%d\n",int_cnt);
		LOG("wakeup_cnt:%d\n",wakeup_cnt);
		
		osal_start_timerEx(application_TaskID, OSAL_ONCE_TIMER_EVT, 1000);

		if(once_timer_counter%2)
		{
			gpio_write(pin_o[0],1);
		}
		else
		{
			gpio_write(pin_o[0],0);
		}
		return ( events ^ OSAL_ONCE_TIMER_EVT );
	}

	if ( events & OSAL_RELOAY_TIMER_EVT )
	{
		LOG("cycle_timer_counter:%d\n",cycle_timer_counter++);

		if(cycle_timer_counter%2)
		{
			gpio_write(pin_o[1],1);
		}
		else
		{
			gpio_write(pin_o[1],0);
		}
		return ( events ^ OSAL_RELOAY_TIMER_EVT );
	}
	
  // Discard unknown events
  return 0;
}
