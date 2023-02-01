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
#include "sleep.h"
#include "bsp_button_task.h"

uint8 SleepTaskId;


void SleepTask_Init(  uint8 task_id  )
{
	SleepTaskId = task_id;

	// !system enterl power off mode after ten seconds
	if( g_system_reset_cause != 2 )
	{
		osal_start_timerEx(SleepTaskId, POWER_OFF_EVT, 60 * 1000);
	}
	
}

uint16 SleepTask_ProcessEvent( uint8 task_id, uint16 events )
{
	VOID task_id; // OSAL required parameter that isn't used in this function
	
	if( events & POWER_OFF_EVT )
	{
		enter_poweroff_mode();
		return ( events ^ POWER_OFF_EVT );
	}


	if( events & SLEEP_EVT )
	{
		return ( events ^ SLEEP_EVT);
	}
	return 0;
}


void enter_poweroff_mode( void )
{
	// !kscan gpio config ready to power off mode
	kscan_enter_power_off_gpio_option();

	LOG("power off mode\r\n");

	hal_pwrmgr_poweroff(NULL, 0);

}


void start_enter_power_off_mode( uint32 timeout_value )
{
	if( !timeout_value )
	{
		osal_set_event(SleepTaskId, POWER_OFF_EVT);
	}
	else
	{
		osal_start_timerEx(SleepTaskId, POWER_OFF_EVT, timeout_value);
	}
}

void stop_enter_power_off_mode( void )
{
	if( osal_get_timeoutEx( SleepTaskId, POWER_OFF_EVT ) != 0 )
	{
		osal_stop_timerEx(SleepTaskId, POWER_OFF_EVT);
	}
}
