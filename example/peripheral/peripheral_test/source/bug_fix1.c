/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree tvoid SM_Init0( uint8 task_id )
o abide by the terms of these agreements. 
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
  Filename:       timer_demo.c
  Revised:        
  Revision:       

  Description:  
                  

**************************************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "jump_function.h"
#include "timer.h"
#include "timer_demo.h"
#include  "log.h"
/*********************************************************************
 * MACROS
 */
//#define LOG(...)  
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************a************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint8 _symrom_osal_start_reload_timer( uint8 taskID, uint16 event_id, uint32 timeout_value );

extern int _symrom_hal_timer_init(ap_tm_hdl_t callback);
extern int _symrom_hal_timer_deinit(void);
extern int _symrom_hal_timer_set(User_Timer_e timeId, uint32_t us);
extern int _symrom_hal_timer_mask_int(User_Timer_e timeId, bool en);
extern int _symrom_hal_timer_stop(User_Timer_e timeId);


/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 timer_TaskID;
static uint8_t s_testCase = 0;
static void timer_test(uint8_t testCase);

void timer_int_process(uint8_t evt)
{
    switch(evt)
    {
    case HAL_EVT_TIMER_4:
	
        LOG("t4\n");
        break;

    case HAL_EVT_WAKEUP:
        LOG("wakeup\n");
        //LOG("timer will disable when sleep,so if you want it work please init it when wakeup\n");
        break;

    case HAL_EVT_SLEEP:
        LOG("sleep\n");
        break;

    default:
        LOG("err ");
        break;
    }
}

uint32_t t_flag = 0;
void __attribute__((used)) hal_TIMER4_IRQ(void)
{
    if(AP_TIM4->status & 0x1)
    {        
		AP_TIM4->EOI;//hal_timer_clear_int(AP_TIM4);
		if(AP_TIM4->status > 0)		
		{
			while(1);;
		}
		
		if(t_flag == 0)
		{
			t_flag = 1;
			*(volatile int*)0x40008000 = 0x01;//p0=1
		}
		else
		{
			t_flag = 0;
			*(volatile int*)0x40008000 = 0x00;//p0=0
		}
    }
}

void timer_Init( uint8 task_id )
{
    timer_TaskID = task_id;
    LOG("timer demo\n");
	
	*(volatile int*)0x40008004 |= 0x01;//p0=output
	_symrom_hal_timer_init(timer_int_process);
	_symrom_hal_timer_set(AP_TIMER_ID_4,100);
	JUMP_FUNCTION_SET(TIM4_IRQ_HANDLER,(uint32_t)&hal_TIMER4_IRQ);			
}


uint16 timer_ProcessEvent( uint8 task_id, uint16 events )
{
    static uint8_t min_count = 4;

    if (events & TIMER_1000_MS_EVT )
    {

        return (events ^ TIMER_1000_MS_EVT);
    }

    return 0;
}
