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

#include "keyboard.h"
#include "pin_map.h"
#include "pwrmgr.h"
#include "OSAL_Timers.h"
//#include "hiddev.h"
//#include "hidkbdservice.h"
#include "OSAL.h"
  

static keyboard_cfg keyboard_param;

static const GPIO_Pin_e matrix_col_to_pin_map[MATRIX_KEYBOARD_COL]= 
{
    COL_GPIO_00,
    COL_GPIO_01,
    COL_GPIO_02,
    COL_GPIO_03,
};

static const GPIO_Pin_e matrix_row_to_pin_map[MATRIX_KEYBOARD_ROW]=
{
    ROW_GPIO_00,
    ROW_GPIO_01,
    ROW_GPIO_02,           
    ROW_GPIO_03,
};

static uint8 matrix_pin_state[ 8 ]={0x00};

#if ( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_0 )
static void keyboard_matrix_pin_init(uint8 choice_mode);
#endif

static void keyboard_to_sleep_gpio_config(  void )
{
	#if ( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_0 )
	gpio_pull_set(ROW_GPIO_00, PULL_DOWN);
	gpio_pull_set(ROW_GPIO_01, PULL_DOWN);
	gpio_pull_set(ROW_GPIO_02, PULL_DOWN);
	gpio_pull_set(ROW_GPIO_03, PULL_DOWN);
	
	gpio_pull_set(COL_GPIO_00, PULL_DOWN);
	gpio_pull_set(COL_GPIO_00, PULL_DOWN);
	gpio_pull_set(COL_GPIO_00, PULL_DOWN);
	gpio_pull_set(COL_GPIO_00, PULL_DOWN);
	
	for(uint8_t i=0;i<MATRIX_KEYBOARD_COL;i++)
	{	
		GPIO_Pin_e col_pin = (GPIO_Pin_e)matrix_col_to_pin_map[i];
		gpio_dir(col_pin, OEN);
		gpio_fast_write(col_pin, 1);
		gpio_retention(col_pin, 1);
	}

	for(uint8_t i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		gpio_fast_write(row_pin, 0);
	}
	
	#elif ( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_1 )
	gpio_pull_set(ROW_GPIO_00, GPIO_PULL_UP);
	gpio_pull_set(ROW_GPIO_01, GPIO_PULL_UP);
	gpio_pull_set(ROW_GPIO_02, GPIO_PULL_UP);
	gpio_pull_set(ROW_GPIO_03, GPIO_PULL_UP);
	#endif
}

uint8_t compare_data_arr_is_null(uint8_t *data, uint8_t data_len)
{
    if( data == NULL )
	{
		return 1;
	}
	for(uint8 i = 0;i < data_len; i++)
	{
		if( data[i] != 0 )
		{
			return 0;
		}
	}

	return 1;
}

__ATTR_SECTION_SRAM__ void keyboard_sleep_handler(void)
{	
	IO_Wakeup_Pol_e pol;
	KEYBOARD_LOG("keyboard sleep\n");

	
	#if ( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_0 )
	for(uint8 i=0;i<MATRIX_KEYBOARD_COL;i++)
	{	
		GPIO_Pin_e col_pin = (GPIO_Pin_e)matrix_col_to_pin_map[i];
		gpio_dir(col_pin, OEN);
		gpio_fast_write(col_pin, 1);
	}
	
	for(uint8 i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		gpio_fast_write(row_pin, 0);
		
		pol = gpio_read(row_pin) ? NEGEDGE:POSEDGE;

		gpio_wakeup_set(row_pin, pol);
		matrix_pin_state[i] = pol;
	}

	#elif( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_1 )
	for(uint8 i=0;i<MATRIX_KEYBOARD_COL;i++)
	{	
		GPIO_Pin_e col_pin = (GPIO_Pin_e)matrix_col_to_pin_map[i];
		gpio_dir(col_pin, OEN);
		gpio_fast_write(col_pin, 0);
	}
	
	for(uint8 i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		gpio_pull_set( (GPIO_Pin_e)matrix_row_to_pin_map[i], WEAK_PULL_UP );
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		
		pol = gpio_read(row_pin) ? NEGEDGE:POSEDGE;

		gpio_wakeup_set(row_pin, pol);
		matrix_pin_state[i] = pol;
	}
	#endif
}


__ATTR_SECTION_SRAM__ static void keyboard_wakeup_handler(void)
{
	IO_Wakeup_Pol_e pol;

	#if ( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_0 )
	for(uint8_t i=0;i<MATRIX_KEYBOARD_COL;i++)
	{
		GPIO_Pin_e col_pin = (GPIO_Pin_e)matrix_col_to_pin_map[i];
		gpio_dir(col_pin, OEN);
		gpio_fast_write(col_pin, 1);
	}
	
	for(uint8_t i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		gpio_fast_write(row_pin, 0);
		gpio_pull_set(row_pin, PULL_DOWN);

	}
	
	for(uint8_t i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		
		pol = gpio_read(row_pin) ? POSEDGE:NEGEDGE;
		
		if(pol == matrix_pin_state[i])
		{
			break;
		}
		else if( pol == 0 && matrix_pin_state[i] == 1 )
		{
			break;
		}
		else if( i == MATRIX_KEYBOARD_ROW - 1 )
		{
			return;
		}
	}
	
	#elif( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_1 )
	for(uint8_t i=0;i<MATRIX_KEYBOARD_COL;i++)
	{
		GPIO_Pin_e col_pin = (GPIO_Pin_e)matrix_col_to_pin_map[i];
		gpio_dir(col_pin, OEN);
		gpio_fast_write(col_pin, 0);
	}
	
	for(uint8_t i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		gpio_pull_set(row_pin, WEAK_PULL_UP);
	}
	
	for(uint8_t i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		
		pol = gpio_read(row_pin) ? POSEDGE:NEGEDGE;
		
		if(pol == matrix_pin_state[i] )
		{
			break;
		}
		else if( i == MATRIX_KEYBOARD_ROW - 1 )
		{
			return;
		}
	}
	#endif
	

	
	  KEYBOARD_LOG("keyboard wake\n");
    pwrmgr_lock( KEYBOARD_PWRMGR_MOD );
    osal_set_event(keyboard_param.keyboard_taskId, keyboard_param.keyboard_event);	 
}

#if( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_0 )
static void keyboard_matrix_pin_init(uint8 choice_mode)
{
    uint8_t i = 0;

    switch( choice_mode )
    {
        case 0:
            for(i = 0; i < sizeof(matrix_col_to_pin_map); i++)
            {
				        gpio_pull_set((GPIO_Pin_e)matrix_col_to_pin_map[i], PULL_DOWN );
                gpio_dir((GPIO_Pin_e)matrix_col_to_pin_map[i],OEN);
                gpio_fast_write((GPIO_Pin_e)matrix_col_to_pin_map[i], 0);
            }

            for(i = 0; i < sizeof(matrix_row_to_pin_map); i++)
            {
				        gpio_pull_set((GPIO_Pin_e)matrix_row_to_pin_map[i], PULL_DOWN );
                gpio_dir((GPIO_Pin_e)matrix_row_to_pin_map[i],IE);
            }
            break;

       case 1:
            for(i = 0; i < sizeof(matrix_col_to_pin_map); i++)
            {
				        gpio_pull_set((GPIO_Pin_e)matrix_col_to_pin_map[i], PULL_DOWN );
                gpio_dir((GPIO_Pin_e)matrix_col_to_pin_map[i],IE);
            }

            for(i = 0; i < sizeof(matrix_row_to_pin_map); i++)
            {
				        gpio_pull_set((GPIO_Pin_e)matrix_row_to_pin_map[i], PULL_DOWN );
                gpio_dir((GPIO_Pin_e)matrix_row_to_pin_map[i],OEN);
                gpio_fast_write((GPIO_Pin_e)matrix_row_to_pin_map[i],0);
            }
            break;
		
    }

}


void keyboard_io_read( void )
{
    uint8 i = 0;
	uint8 row = 0;
	uint8 column = 0;
	uint32 r = 0;
	uint8 push_row = 0, push_col = 0;
	uint8 push_key_state = 0;
	uint8 key_status_change = 0;
    uint8 row_state_arr[MATRIX_KEYBOARD_COL] = { 0x00 };	
    uint8 column_state_arr[MATRIX_KEYBOARD_ROW] = { 0x00 };
	
    static uint8 previous_matrix_data[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];
    static uint8 current_matrix_data[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];
    static uint8 current_matrix_data_column[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];
	static uint8 current_matrix_data_row[MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW];

	keyboard_matrix_pin_init(0);
	
	for( column = 0; column < MATRIX_KEYBOARD_COL; column++ )
	{
		gpio_write((GPIO_Pin_e)matrix_col_to_pin_map[column], 1);
		for( i = 0; i < MATRIX_KEYBOARD_ROW; i++ )
		{
			r = gpio_read((GPIO_Pin_e)matrix_row_to_pin_map[i]);
			row_state_arr[column] |= ((uint8_t)((r ? 1 : 0) << i));
		}
		
		if( row_state_arr[column] != 0 )
		{ 
			for( row = 0; row < MATRIX_KEYBOARD_ROW; row++)
			{
                if( row_state_arr[column] & (1U << row) )
                {					
					current_matrix_data_column[ column * MATRIX_KEYBOARD_ROW + row ] =column * MATRIX_KEYBOARD_ROW + row + 1;	
				}
				else
				{
					current_matrix_data_column[ column * MATRIX_KEYBOARD_ROW + row ] = 0;
				}
			}
		}
		else
		{
		    for( row = 0; row < MATRIX_KEYBOARD_ROW; row++ )
		    {
				current_matrix_data_column[ column * MATRIX_KEYBOARD_ROW + row ] = 0;
			}
		}

		gpio_write((GPIO_Pin_e)matrix_col_to_pin_map[column], 0);
		osal_memset( row_state_arr, 0x00, sizeof(row_state_arr) );		
	}

	keyboard_matrix_pin_init(1);
	
	for( row = 0; row < MATRIX_KEYBOARD_ROW; row++ )
	{
		gpio_fast_write((GPIO_Pin_e)matrix_row_to_pin_map[row], 1);

		for( i = 0; i < MATRIX_KEYBOARD_COL; i++ )
		{
			r = gpio_read((GPIO_Pin_e)matrix_col_to_pin_map[i]);
			column_state_arr[row] |= ((r ? 1 : 0) << i);
		}
		
		if( column_state_arr[row] != 0 )
		{ 
			for( column = 0; column < MATRIX_KEYBOARD_COL; column++)
			{
				if( column_state_arr[row] & (1U << column) )
				{	
					current_matrix_data_row[ column * MATRIX_KEYBOARD_ROW + row ] = column * MATRIX_KEYBOARD_ROW + row + 1;
				}
				else
				{
					current_matrix_data_row[ column * MATRIX_KEYBOARD_ROW + row ] = 0;
				}
			}
		}
		else
		{
			for( column = 0; column< MATRIX_KEYBOARD_COL; column++ )
			{
				current_matrix_data_row[ column * MATRIX_KEYBOARD_ROW + row ] = 0;
			}
		}

        gpio_fast_write((GPIO_Pin_e)matrix_row_to_pin_map[row], 0);
		osal_memset(column_state_arr, 0x00, sizeof( column_state_arr ));
	}


	for( i = 0; i < MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW; i++ )
	{
		if( current_matrix_data_row[i] == current_matrix_data_column[i] )
		{
			current_matrix_data[i] = current_matrix_data_row[i];
		}
		else if( current_matrix_data_row[i] == 0 && current_matrix_data_column[i] != 0 )
		{
			current_matrix_data[i] = current_matrix_data_column[i];
		}
		else if( current_matrix_data_row[i] != 0 && current_matrix_data_column[i] == 0 )
		{
			current_matrix_data[i] = current_matrix_data_row[i];
		}

		
		key_status_change = current_matrix_data[i] ^ previous_matrix_data[i];
		if( key_status_change != 0 )
		{
			push_key_state = !!current_matrix_data[i];

			push_row = i % MATRIX_KEYBOARD_ROW;
			push_col = i / MATRIX_KEYBOARD_ROW;
			
			KEYBOARD_LOG("[%d %d %d]\n", push_row, push_col, push_key_state);

			if( keyboard_param.keyboard_cb != NULL )
			{
				keyboard_param.keyboard_cb(MATRIX_KEYBOARD_COL, push_row, push_col, push_key_state);
			}
		}
		previous_matrix_data[i] = current_matrix_data[i];

		current_matrix_data[i] = 0;
	}

    if( 1 == compare_data_arr_is_null(previous_matrix_data, MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW ) )
    {
		osal_memset(previous_matrix_data, 0x00, MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW);
		osal_memset(current_matrix_data, 0x00, MATRIX_KEYBOARD_COL * MATRIX_KEYBOARD_ROW);
		osal_clear_event( keyboard_param.keyboard_taskId, keyboard_param.keyboard_event );
		
		keyboard_to_sleep_gpio_config();

		pwrmgr_unlock( KEYBOARD_PWRMGR_MOD );
			
    }
    else
	{
		osal_start_timerEx(keyboard_param.keyboard_taskId, keyboard_param.keyboard_event, KEYBOARD_POLLING_INTERVAL);
	}
}
#endif



#if ( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_1 )
void keyboard_io_read(void)
{
	uint32 matrix_io_status = 0x00;
	uint8 key_index[ KEYBOARD_BUFF_LEN ] = {0x00};
	uint8 key_state[ KEYBOARD_BUFF_LEN ] = {0x00};
	uint8 key_change = 0;
	uint8 push_key_state=0XFF;
	uint8 push_row = 0, push_col = 0;
	static uint8 preious_key_state[ KEYBOARD_BUFF_LEN ] = {0x00};
	
	for( uint8 a = 0 ; a < KEYBOARD_BUFF_LEN; a++ )
	{
		key_index[a] = a+1;
	}
	/* for  key extend to gnd */
    // ! row io init
    for( uint8 n = 0; n < sizeof( matrix_row_to_pin_map ); n++ )
	{
		gpio_dir(matrix_row_to_pin_map[n], IE);
		gpio_pull_set(matrix_row_to_pin_map[n], WEAK_PULL_UP);
	}
	
	// ! col io init
	for( uint8 n = 0; n < sizeof( matrix_col_to_pin_map ); n++ )
	{
		gpio_write(matrix_col_to_pin_map[n] ,1);
	}
	
	// ! read row io status when key extend to gnd
	for( uint8 m = 0; m < KEYBOARD_EXTEND_KEY_NUM; m++ )
	{
		matrix_io_status |= ( ( !gpio_read(matrix_row_to_pin_map[m] ) ? 1 : 0 ) << m );
	}
	
	/* for  key extend to gpio and no connect gnd*/
	// ! row io init
	for( uint8 n = 0; n < sizeof( matrix_row_to_pin_map ); n++ )
	{
		gpio_dir(matrix_row_to_pin_map[n], IE);
		gpio_pull_set(matrix_row_to_pin_map[n], WEAK_PULL_UP);
	}
	// ! col io init
	for( uint8 n = 0; n < sizeof( matrix_col_to_pin_map ); n++ )
	{
		gpio_write(matrix_col_to_pin_map[n] ,1);
	}
	
	for( uint8 col = 0; col < sizeof( matrix_col_to_pin_map ); col++ )
	{
		// ! col io init
		
		gpio_write(matrix_col_to_pin_map[ col ] ,0);
		
		// ! read row io status when key no extend to gnd
		for( uint8 m = 0; m < sizeof( matrix_row_to_pin_map ); m++ )
		{
			if( (matrix_io_status & (1 << m)) == 0 )
			{
				matrix_io_status |= ( ( !gpio_read(matrix_row_to_pin_map[m] ) ? 1 : 0 ) << (col * 4 + m + KEYBOARD_EXTEND_KEY_NUM) );
			}
		}
		gpio_write(matrix_col_to_pin_map[ col ] ,1);
	}
	  
	for( uint8 j = 0; j < KEYBOARD_EXTEND_KEY_NUM; j++ )
	{
		if( (matrix_io_status & BIT(j)) != 0 )
		{
			key_state[j] =key_index[j]; 
		}
		else
		{
			key_state[j] = 0; 
		}
	}
		
	for( uint8 j = KEYBOARD_EXTEND_KEY_NUM; j < KEYBOARD_BUFF_LEN; j++ )
	{
		if( (matrix_io_status & BIT(j)) != 0 )
		{
			key_state[j] =key_index[j]; 
		}
		else
		{
			key_state[j] =0;
		}
    }
	
	// !key data analyse
	for( uint8 l = 0; l < KEYBOARD_BUFF_LEN; l++ )
	{
		key_change  = preious_key_state[l] ^ key_state[l];
		if( key_change != 0 )
		{
			push_key_state = !!key_state[l];
			
			push_row = l % MATRIX_KEYBOARD_ROW;
			push_col = l / MATRIX_KEYBOARD_ROW;
			
			KEYBOARD_LOG("[%d %d %d]\n", push_row, push_col, push_key_state);

			if( keyboard_param.keyboard_cb != NULL )
			{
				keyboard_param.keyboard_cb(MATRIX_KEYBOARD_COL+1, push_row, push_col, push_key_state);
			}
			
	    }	
	    preious_key_state[l] = key_state[l];
	}

	if( 1 == compare_data_arr_is_null(preious_key_state, KEYBOARD_BUFF_LEN ) )
    {
		osal_memset(preious_key_state, 0x00, sizeof(preious_key_state));
		osal_memset(key_state, 0x00, sizeof(key_state));
		osal_clear_event( keyboard_param.keyboard_taskId, keyboard_param.keyboard_event );
		
		keyboard_to_sleep_gpio_config();

		hal_pwrmgr_unlock( KEYBOARD_PWRMGR_MOD );
			
    }
    else
	{
		osal_start_timerEx( keyboard_param.keyboard_taskId, keyboard_param.keyboard_event, KEYBOARD_POLLING_INTERVAL);
	}
}


#endif


void keyboard_register( keyboard_event_t cb, uint8 task_Id, uint16 event )
{
	if( cb == NULL )
	{
		return;
	}
	
	keyboard_param.keyboard_cb = cb;
	keyboard_param.keyboard_taskId = task_Id;
	keyboard_param.keyboard_event = event;
	
	keyboard_to_sleep_gpio_config();
  pwrmgr_register( KEYBOARD_PWRMGR_MOD, keyboard_sleep_handler, keyboard_wakeup_handler);
	
}

void keyboard_deep_sleep_handler( void )
{
	for(uint8 i=0;i<MATRIX_KEYBOARD_COL;i++)
	{
		GPIO_Pin_e col_pin = (GPIO_Pin_e)matrix_col_to_pin_map[i];
		gpio_dir(col_pin, OEN);
    gpio_pull_set(col_pin, PULL_DOWN);
	}
	
	for(uint8 i=0;i<MATRIX_KEYBOARD_ROW;i++)
	{
		GPIO_Pin_e row_pin = (GPIO_Pin_e)matrix_row_to_pin_map[i];
		gpio_dir(row_pin, IE);
		gpio_pull_set(row_pin, WEAK_PULL_UP);
		
		gpio_wakeup_set(row_pin, 0);
	}
}
