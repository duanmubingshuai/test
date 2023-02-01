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

pwm_cfg_t pwm0 = 
{

	.pin = P18,
	.div = PWM_CLK_DIV_4,
	.mode = PWM_CNT_UP,
	.polarity = PWM_POLARITY_RISING,
	.cmp_val = 9,
	.top_val = 199,
};
pwm_cfg_t pwm[3]={
	{

		.pin = P12,
		.div = PWM_CLK_NO_DIV,
		.mode = PWM_CNT_UP,
		.polarity = PWM_POLARITY_RISING,
		.cmp_val = 30,
		.top_val = 200,

	},
	{

		.pin = P13,
		.div = PWM_CLK_NO_DIV,
		.mode = PWM_CNT_UP,
		.polarity = PWM_POLARITY_FALLING,
		.cmp_val = 20,
		.top_val = 200,

	},
	{

		.pin = P14,
		.div = PWM_CLK_NO_DIV,
		.mode = PWM_CNT_UP,
		.polarity = PWM_POLARITY_RISING,
		.cmp_val = 10,
		.top_val = 200,

	}
};
/*
	duty=0%~100%~0%~100%...
*/
void pwm_deadband_Init1(uint8 task_id)
{
	application_TaskID = task_id;
	LOG("pwm demo deadband\n");
	pwm_ch_start(&pwm[0]);
	pwm_ch_start(&pwm[1]);
	PWM_DISABLE();
	WaitMs(50);

	PWM_ENABLE_CH_23();
	osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 100);
}
void pwm_Init(uint8 task_id)
{		
	application_TaskID = task_id;
	LOG("pwm demo\n");
	
	pwm0.pin = P18;		
	pwm0.mode = PWM_CNT_UP_AND_DOWN;
	pwm0.polarity = PWM_POLARITY_FALLING;
	pwm0.cmp_val = 9;
	pwm0.top_val = 99;
//	pwm0.pwm_start(&pwm0);
	pwm_ch_start(&pwm0);
	osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 100);
}

uint16 pwm_ProcessEvent( uint8 task_id, uint16 events )
{
	static uint32_t i = 0;
	static bool add_flag = TRUE;
//	int ret;
	const uint8 cmp_val[11] = {0,10-1,20-1,30-1,40-1,50-1,60-1,70-1,80-1,90-1,100-1};
	LOG("in processevent\n");
	if(task_id != application_TaskID){
		return 0;
	}
	
	if ( events & OSAL_RELOAY_TIMER_EVT )
	{
#if 0
//		if(add_flag == TRUE)
//		{
//			if(i < 10)
//			{
//				i++;
//			}
//			else
//			{	i = 10 - 1;
//				add_flag = FALSE;
//			}
//		}
//		else if(add_flag == FALSE)
//		{
//			if(i > 0)
//			{
//				i--;
//			}
//			else
//			{	i = 0 + 1;
//				add_flag = TRUE;
//			}
//		}
//		pwm0.cmp_val = cmp_val[i];
//		LOG("cmp+1:%d top+1:%d\n",(pwm0.cmp_val+1),(pwm0.top_val+1));
//		gpio_write(P12,1);
//		pwm_change_parameter(&pwm0);
//		gpio_write(P12,0);
#endif
		pwm_test1();
		LOG("pwm_cmp = %d\n pwm_cntTop = %d\n",pwm0.cmp_val,pwm0.top_val);
		return ( events ^ OSAL_RELOAY_TIMER_EVT );
	}
	
  // Discard unknown events
  return 0;
}
