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
#include "gpio_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"


uint8 key_TaskID;
#define START_DEVICE_EVT    1
#define KEY_DEMO_ONCE_TIMER 2
volatile int s_tmp = 1234;
void Key_Demo_Init(uint8 task_id)
{
	uint8_t i = 0;
	
	key_TaskID = task_id;
	PRINT("gpio key demo start...\n");
	osal_set_event(key_TaskID, START_DEVICE_EVT);
}
static int s_tmp_cnt;
uint16 Key_ProcessEvent( uint8 task_id, uint16 events )
{
	if(task_id != key_TaskID){
		return 0;
	}
	if( events & START_DEVICE_EVT){
        s_tmp ++;
//		LOG_INIT();
		PRINT("START_DEVICE_EVT\n");
		osal_start_timerEx(key_TaskID, KEY_DEMO_ONCE_TIMER, 50);
		return (events ^ START_DEVICE_EVT);
	}
	
	if( events & KEY_DEMO_ONCE_TIMER){		
//		LOG_INIT();
        PRINT("%d,rtc cnt %d\n", s_tmp_cnt++,rtc_get_counter());
		osal_start_timerEx( key_TaskID, KEY_DEMO_ONCE_TIMER , 500);
		return (events ^ KEY_DEMO_ONCE_TIMER);
	}
	
  // Discard unknown events
  return 0;
}

/*********************************************************************
*********************************************************************/
