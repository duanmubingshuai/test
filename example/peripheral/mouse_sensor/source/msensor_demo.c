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
#include "msensor_demo.h"
#include "log.h"

#include "gpio.h"
#include "spi_lite.h"
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




void gpio_Init(uint8 task_id)
{	
	volatile int ret;	
	gpio_pupd_e type = GPIO_PULL_DOWN;
	
	application_TaskID = task_id;
	gpio_init_patch();

	//osal_start_timerEx(application_TaskID, OSAL_ONCE_TIMER_EVT, 1000);
	//osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 1500);
}

#if 0
spilite_cfg_t s_spi_cfg = {

	.sclk_pin = {P16,3},
	.ssn_pin = {GPIO_DUMMY, 0},
	.MOSI = {P17, 3},
    .MISO = {P18, 3},


	.baudrate = 100000,
	.spi_scmod = SPI_MODE3,
	
};

uint8_t ms_cmd[2];
uint8_t ms_data[10];

void msensor_test(void)
{
  uint8_t* p = ms_data;
  spi_bus_init(&s_spi_cfg);
  gpio_pull_set(s_spi_cfg.MOSI.pin, GPIO_PULL_UP);

  //read ID
  ms_cmd[0] = 0x01;
  spi_transmit(ms_cmd, NULL, 1);

  gpio_fmux_set(s_spi_cfg.MOSI.pin, 0);//spimap[pin_flag[2]][1]);
  //gpio_fmux_set(rx_pin, spimap[pin_flag[3]][1]);
  spi_transmit(NULL, p++, 1);

  ms_cmd[0] = 0x00;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 1);
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 0);//spimap[pin_flag[2]][1]);
  spi_transmit(NULL, p++, 1);
  
  ms_cmd[0] = 0x05;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 1);
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 0);//spimap[pin_flag[2]][1]);
  spi_transmit(NULL, p++, 1);

  ms_cmd[0] = 0x85;
  ms_cmd[1] = 0xa0;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 2);

  ms_cmd[0] = 0x05;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 1);
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 0);//spimap[pin_flag[2]][1]);
  spi_transmit(NULL, p++, 1);

  ms_cmd[0] = 0x0D;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 1);
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 0);//spimap[pin_flag[2]][1]);
  spi_transmit(NULL, p++, 1);


  ms_cmd[0] = 0x8D;
  ms_cmd[1] = 0x93;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 2);

  ms_cmd[0] = 0x0D;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 1);
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 0);//spimap[pin_flag[2]][1]);
  spi_transmit(NULL, p++, 1);

#if 0
  ms_cmd[0] = 0x00;
  gpio_fmux_set(s_spi_cfg.MOSI.pin, 3);//spimap[pin_flag[2]][1]);
  spi_transmit(ms_cmd, NULL, 1);
  gpio_fmux_set(s_spi_cfg.MOSI, 0);//spimap[pin_flag[2]][1]);
  spi_transmit(NULL, p++, 1);
#endif
}
#endif


uint16 gpio_ProcessEvent( uint8 task_id, uint16 events )
{
	static uint32_t once_timer_counter = 0;
	static uint32_t cycle_timer_counter = 0;
	if(task_id != application_TaskID){
		return 0;
	}

	
  // Discard unknown events
  return 0;
}
