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
  Filename:       gpio_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "OSAL.h"
#include "pwm.h"
#include "pwm_demo.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"

//#define LOG(...)                    {log_printf(__VA_ARGS__);}

uint8 application_TaskID;

#define MIN_DEADZONE_LENTH 50  //2%~96% The unit of this figure is %
#define DEADZONE_CHANGE_TIME 100  // The unit of this figure is ms

#define CNT 2
#define CNT_TOP_VAL (CNT * 100)    //pwm output freq is 16MHz/DIV/CNT_TOP_VAL = 80kHz

extern void WaitUs(uint32_t);

pwm_cfg_t pwm[2]={
	{

		.pin = P12,
		.div = PWM_CLK_NO_DIV,
		.mode = PWM_CNT_UP,
		.polarity = PWM_POLARITY_RISING,
		.cmp_val = 30,
		.top_val = 200,
		.ch = 2,

	},
	{

		.pin = P13,
		.div = PWM_CLK_NO_DIV,
		.mode = PWM_CNT_UP,
		.polarity = PWM_POLARITY_FALLING,
		.cmp_val = 20,
		.top_val = 200,
		.ch = 3,

	}
};
/*
	duty=0%~100%~0%~100%...
*/

void pwm_Init(uint8 task_id)
{		
	application_TaskID = task_id;
	LOG("pwm demo deadband\n");

	pwm_ch_start(&pwm[0]);
	pwm_ch_start(&pwm[1]);
	

	PWM_DISABLE() ;
	
	WaitUs(100);
	
	PWM_ENABLE_CH_23()  ;


	osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, DEADZONE_CHANGE_TIME);
}

uint16 pwm_ProcessEvent( uint8 task_id, uint16 events )
{
	if(task_id != application_TaskID){
		return 0;
	}
	
	if ( events & OSAL_RELOAY_TIMER_EVT )
	{
		static int count = 0;
		static int i = CNT;
		

		if (count <= CNT)
		{
			i = CNT;
		}
		else if (count >= (CNT_TOP_VAL - CNT * MIN_DEADZONE_LENTH)/2)
		{
			i = -CNT;
		}
		count += i;
		
		LOG("count = %d\n",count);
		
		PWM_SET_CMP_VAL(2,CNT_TOP_VAL - count - 1);
		PWM_SET_CMP_VAL(3,0 + count);
		
		return ( events ^ OSAL_RELOAY_TIMER_EVT );
	}
	

  return 0;
}
