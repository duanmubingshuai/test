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

#include <string.h>
#include "OSAL.h"
#include "rom_sym_def.h"
#include "pwrmgr.h"
#include "pin_map.h"
#include "gpio.h"
#include "bsp_button.h"
#include "OSAL_Timers.h"
#include "bsp_button_task.h"
#include "hal_keyboard_matrix.h"
#include "log.h"
//#include "hidkbd.h"
//#include "voice_task.h"
//#include "single_carrier_task.h"
//#include "sleep.h"
//#include "ir_task.h"
//#include "hiddev.h"
//#include "peripheral.h"
//#include "ll.h"
//#include "sleep.h"
//#include "kscan.h"
//#include "gapbondmgr.h"
/*********************************************************************
    TYPEDEFS
*/



uint8 matrix_key_detected_flag = 0;
uint8 ir_sending_over_flag = 0;
uint16 ir_sending_timeout_count = 0;
/*********************************************************************
    GLOBAL VARIABLES
*/
uint8 halKeyboardMatrix_TaskID;   // Task ID for internal task/event processing

bool key_press = false;
/*********************************************************************
    EXTERNAL VARIABLES
*/





/*********************************************************************
    FUNCTIONS
*/



/*********************************************************************
    LOCAL VARIABLES
*/

#ifdef PHY6230_REMOTE_CONTROL_VERSION
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KSCAN_MAP_GPIO_ROW0,KSCAN_MAP_GPIO_ROW1,KSCAN_MAP_GPIO_ROW2,KSCAN_MAP_GPIO_ROW3};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KSCAN_MAP_GPIO_COL0,KSCAN_MAP_GPIO_COL1,KSCAN_MAP_GPIO_COL2,KSCAN_MAP_GPIO_COL3};
gpio_pin_e row_io[NUM_KEY_ROWS] = {KSCAN_ROW_0_GPIO, KSCAN_ROW_1_GPIO, KSCAN_ROW_2_GPIO, KSCAN_ROW_3_GPIO };
gpio_pin_e col_io[NUM_KEY_COLS] = {KSCAN_COL_0_GPIO, KSCAN_COL_1_GPIO, KSCAN_COL_2_GPIO, KSCAN_COL_3_GPIO};
#endif

#ifdef PHY6252_REMOTE_CONTROL_VERSION
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KSCAN_MAP_GPIO_ROW0,KSCAN_MAP_GPIO_ROW1,KSCAN_MAP_GPIO_ROW2,KSCAN_MAP_GPIO_ROW3};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KSCAN_MAP_GPIO_COL0,KSCAN_MAP_GPIO_COL1,KSCAN_MAP_GPIO_COL2,KSCAN_MAP_GPIO_COL3};
gpio_pin_e row_io[NUM_KEY_ROWS] = {KSCAN_ROW_0_GPIO, KSCAN_ROW_1_GPIO, KSCAN_ROW_2_GPIO, KSCAN_ROW_3_GPIO };
gpio_pin_e col_io[NUM_KEY_COLS] = {KSCAN_COL_0_GPIO, KSCAN_COL_1_GPIO, KSCAN_COL_2_GPIO, KSCAN_COL_3_GPIO};
#endif

#ifdef PHY6222_REMOTE_CONTROL_VERSION
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KSCAN_MAP_GPIO_ROW0,KSCAN_MAP_GPIO_ROW1,KSCAN_MAP_GPIO_ROW2,KSCAN_MAP_GPIO_ROW3,KSCAN_MAP_GPIO_ROW4};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KSCAN_MAP_GPIO_COL0,KSCAN_MAP_GPIO_COL1,KSCAN_MAP_GPIO_COL2,KSCAN_MAP_GPIO_COL3,KSCAN_MAP_GPIO_COL4, KSCAN_MAP_GPIO_COL5};
gpio_pin_e row_io[NUM_KEY_ROWS] = {KSCAN_ROW_0_GPIO, KSCAN_ROW_1_GPIO, KSCAN_ROW_2_GPIO, KSCAN_ROW_3_GPIO,  KSCAN_ROW_4_GPIO};
gpio_pin_e col_io[NUM_KEY_COLS] = {KSCAN_COL_0_GPIO, KSCAN_COL_1_GPIO, KSCAN_COL_2_GPIO, KSCAN_COL_3_GPIO, KSCAN_COL_4_GPIO, KSCAN_COL_5_GPIO};
#endif


#if (BSP_COMBINE_BTN_NUM > 0)
uint32_t usr_combine_btn_array[BSP_COMBINE_BTN_NUM] =
{
    (BIT(14)|BIT(15)),// ! FOR ENABLE ADVERTISING
	(BIT(14)|BIT(12)),// ! FOR SENDING CARRIER
	(BIT(10)|BIT(11)|BIT(9)),  // ! FOR ENTER PRODUCT 0
};
#endif


void hal_bsp_btn_callback(uint8_t evt)
{
	uint8 key_index = BSP_BTN_INDEX(evt);
	static uint8 ir_learn_long_press_flag = 0;
	
	// !start power off event
    start_enter_power_off_mode(60 * 1000);
    
    switch( BSP_BTN_TYPE(evt) )
    {
		// ! press
		case BSP_BTN_PD_TYPE:
			KEY_LOG("[key]:%02d pre\n", key_index);
			key_press = true;
		
			break;
        // ! release
		case BSP_BTN_UP_TYPE:
			KEY_LOG("[key]:%02d rel\n", key_index);
			key_press = false;
			break;
        // !long press
		case BSP_BTN_LPS_TYPE:
			KEY_LOG("[key]:%02d longpress\n", key_index);
			// ! combine key long press to enable advetising or disable advertising
			break;
        // ! long press keep
		case BSP_BTN_LPK_TYPE:
			LOG("long press keep\r\n");
			break;

		default:
			LOG("unexpected ");
			break;
    }

}


void hal_kscan_btn_check(bsp_btn_callback_t cb)
{
    if((NUM_KEY_ROWS != sizeof(rows)/sizeof(rows[0])) || (NUM_KEY_COLS != sizeof(cols)/sizeof(cols[0])))
    {
        return;
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    if(BSP_COMBINE_BTN_NUM != sizeof(usr_combine_btn_array)/sizeof(usr_combine_btn_array[0]))
    {
        return;
    }

    #endif
    if(cb == NULL)
    {
        return;
    }

    #if (BSP_COMBINE_BTN_NUM > 0)

    for(int i = 0; i < BSP_COMBINE_BTN_NUM; i++)
    {
        if(usr_combine_btn_array[i] == 0x00)
        {
			LOG("EEEE\r\n");
            return;
        }
    }

    #endif
    bsp_btn_cb = cb;
    bsp_btn_kscan_flag = TRUE;
}
//#endif






/*********************************************************************
    @fn      hal_keyboard_matrix_task_init


    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/

void hal_keyboard_matrix_task_init( uint8 task_id )
{
    halKeyboardMatrix_TaskID = task_id;
	
	// !register kscan application callback
	hal_kscan_btn_check(hal_bsp_btn_callback);
	
	if(bsp_btn_kscan_flag != TRUE)
	{
		KEY_LOG("hal_kscan_btn_check error:%d\n",__LINE__);
	}

}




/*********************************************************************
    @fn      hal_keyboard_matrix_task_ProcessEvent

    @brief   hal_keyboard_matrix Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 hal_keyboard_matrix_task_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
	
	// ! key release status event
	if( events & HAL_KEY_SOFT_SCAN_EVT )
	{
		// !key press always
		if( kscan_soft_polling_read() == 1 )
		{
			osal_start_timerEx(halKeyboardMatrix_TaskID, HAL_KEY_SOFT_SCAN_EVT, 50);
		}
		// !key release
		else
		{
			KEY_LOG("[poweroff long key] release\r\n");
			if( osal_get_timeoutEx(halKeyboardMatrix_TaskID, HAL_KEY_SOFT_SCAN_EVT) != 0 )
			{
				osal_stop_timerEx(halKeyboardMatrix_TaskID, HAL_KEY_SOFT_SCAN_EVT);
			}
			
			// !stop ir sending because no key continue press
			//ir_right_sending_stop_handle(0);
			
			// !bsp_button_init_config_handle
			bsp_button_init_config_handle();
			
			// !enter power off mode after 30 seconds
			start_enter_power_off_mode(60*1000);
			
		}
		return ( events ^ HAL_KEY_SOFT_SCAN_EVT);
	}

    return 0;
}


/*********************************************************************
    @fn      kscan_soft_read_pin_init

    @brief   kscan soft row&col scan gpio init

    @param   none

    @return  none
*/
void kscan_soft_read_pin_init(void)
{
	uint8_t i = 0;


	for(i = 0; i < sizeof(col_io); i++)
	{
		gpio_fmux_set(col_io[i], FMUX_GPIO);
		gpio_dir(col_io[i], GPIO_OUTPUT);
		gpio_pull_set(col_io[i],PULL_DOWN);
	}

	for(i = 0; i < sizeof(row_io); i++)
	{
		gpio_fmux_set(row_io[i], FMUX_GPIO);
		gpio_dir(row_io[i], GPIO_INPUT);
		gpio_pull_set(row_io[i],PULL_DOWN);
	}
}


/*********************************************************************
    @fn      kscan_soft_polling_read

    @brief   kscan soft row&col scan gpio read once

    @param   none

    @return  none
*/
uint8 kscan_soft_polling_read(void)
{
	uint8 read_bits = 0;
	uint8 read_bit_arr[4] = { 0x00 };
	static uint8 current_matrix_data[16];
    osal_memset(current_matrix_data, 0xff, 16);

	kscan_soft_read_pin_init();
	
	
	for( uint8 column = 0; column < 4; column++ )
	{
		// !column output high level
		gpio_write(col_io[column], 1);
		
		// !read row gpio level and save gpio level in read_bit_arr
		for( uint8 row_index = 0; row_index < 4; row_index++ )
		{
			read_bits = gpio_read( row_io[ row_index ] );
			read_bit_arr[column] |= (read_bits ? 1:0) << row_index;
		}
		
		// !key data verify
		if( read_bit_arr[column] != 0 )
		{
			// !key press
			for( uint8 row_offset = 0; row_offset < 4; row_offset ++ )
			{
				if( read_bit_arr[column] & (1 << row_offset) )
				{
//					LOG("row col =%d %d\r\n", row_offset, column);
                    current_matrix_data[column * 4 + row_offset] = column * 4 + row_offset;
					
				}
			}
		}
		
		// !column output low level
		gpio_write(col_io[column], 0);
	}

	//whether kscan key wake system from power off mode
    for( uint8 i = 0; i < 16; i++ )
    {
		if( current_matrix_data[i] < 16 )
		{
			return 1;
		}
    }

    return 0;
}




bool get_key_press_status( void )
{
	return key_press;
}





