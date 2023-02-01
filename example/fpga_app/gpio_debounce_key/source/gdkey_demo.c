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
#include "gdkey_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "gpio_debounce_key.h"


uint8 application_TaskID;



#define DEMO_KEY_NUM 2

gdkey_t key_cfg[DEMO_KEY_NUM] = {

    {P12, 0, 0, GPIO_PULL_UP},
    {P13, 0, 0, GPIO_PULL_DOWN}
};


void key_cb(gpio_pin_e pin, gdkey_evt_e evt)
{
    uint8_t i,bit;
    for(i = 0; i< DEMO_KEY_NUM; i++){
		if(pin == key_cfg[i].pin){
			bit = i*2 + evt;
			LOG("key_cb %x\n", evt);
			osal_set_event(application_TaskID, BIT(bit));
			return;
		}
    }
}

gpioin_t s_gpio_in[2];


void key_Init(uint8 task_id)
{	
	int ret;	
	gpio_pupd_e type = GPIO_PULL_DOWN;
	
	application_TaskID = task_id;
	LOG("key_demo\n");
    gpioin_init(s_gpio_in, 2);
    ret = gdkey_register(key_cfg, 2, key_cb);
    LOG("gdkey_register %d\n", ret);
	//osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 1500);
}

extern uint32 t_tmp;
extern uint32 p_tmp;
uint16 key_ProcessEvent( uint8 task_id, uint16 events )
{
    uint8_t i;	
	if(task_id != application_TaskID){
		return 0;
	}
	LOG("evt %d, %x\n",task_id,events);


    for(i = 0; i< DEMO_KEY_NUM; i++)
    {
    	if ( events & BIT(i*2))
    	{
    		LOG("Key press %d\n", key_cfg[i].pin);
    		return ( events ^ BIT(i*2));
    	}
    	if ( events & BIT(i*2+1))
    	{
    		LOG("Key release %d\n", key_cfg[i].pin);
    		return ( events ^ BIT(i*2+1));
    	}

    }
	
  // Discard unknown events
  return 0;
}
