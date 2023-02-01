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
#include "spi_demo.h"
#include "log.h"
#include "spi.h"
#include "spiflash.h"
#include "clock.h"
#include "pwrmgr.h"
#include "error.h"
#include "pico_reg.h"
//#define LOG(...)                    {log_printf(__VA_ARGS__);}

uint8 application_TaskID;

hal_spi_t spi ={
	.spi_index = SPI0,
};
spi_Cfg_t spi_cfg = {			
	.sclk_pin = P16,
	.ssn_pin = P19,
	.MOSI = P17,
	.MISO = P18,

//	.sclk_pin = P14,
//	.ssn_pin = P13,
//	.MOSI = P15,
//	.MISO = P12,

//	.sclk_pin = P4,
//	.ssn_pin = P3,
//	.MOSI = P5,
//	.MISO = P2,

	.baudrate = 1000000,
	.spi_tmod = SPI_TRXD,
	.spi_scmod = SPI_MODE0,
	
	.int_mode = false,
	.force_cs = true,
	.evt_handler = NULL,
};

void spi_Init(uint8 task_id)
{	
//	volatile int ret;	
//	gpio_pupd_e type = GPIO_PULL_DOWN;

	application_TaskID = task_id;
	LOG("spi demo\n");

	if(spi_bus_init(spi_cfg) != PPlus_SUCCESS)
	{
		LOG("spi init error\n");
		while(1);
	}
	osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 50);
}
void spi_baud_initt(uint16_t baud_num)
{
	volatile int aabb = 1;
//	spi0_ssienr_set(0);
	*(uint32_t *)0x40006008 = 0x00000000;
	WaitUs(200);
	spi0_baudr_set(baud_num);
//	while(aabb);
	*(uint32_t *)0x40006008 = 0x00000001;
//	spi0_ssienr_set(1);


}
uint16 spi_ProcessEvent( uint8 task_id, uint16 events)
{
	volatile uint32_t flash_id = 0;

	if(task_id != application_TaskID){
		return 0;
	}

	if ( events & OSAL_RELOAY_TIMER_EVT )
	{
		static uint32_t spi_clk_divider = 0x2;
//		spi_cfg.baudrate = clk_get_pclk() / spi_clk_divider;

		spi_baud_initt(spi_clk_divider);
		
		LOG("divider = %d\n",spi_clk_divider);
		LOG("0x40006014 = 0x%x,baud = %d\n",*(uint32_t *)0x40006014,spi_cfg.baudrate);

		flash_id = spiflash_read_identification();
		
		LOG("flash_id:0x%x\n",flash_id);//read flash id or see wave
		
		if (spi_clk_divider > 65599)
		{
			LOG("divider = %d\n",spi_clk_divider);
			while(1);
		}
		spi_clk_divider += 24;

#if 0
		uint8_t spi_send_data[4] = {0x55,0xcc,0xff,0xf0};
		uint8_t spi_rx_data[4];
		spiflash_deep_powerdown();
		spiflash_release_from_powerdown();

		spiflash_write_enable();
		spiflash_read_status_register(0);
//		spiflash_block_erase_32KB(000000);
		spiflash_write(0x000000  ,spi_send_data,4);
		spiflash_read(0x000000,spi_rx_data,4);
		LOG("rx1 = %x,rx2 = %x,rx3 = %x,rx4 = %x\n",*(spi_rx_data),*(spi_rx_data+1),*(spi_rx_data+2),*(spi_rx_data+3));
#endif
#if 0
		uint8_t spi_send_data[4] = {0x55,0xcc,0xff,0xf0};
		uint8_t spi_rx_data[4];
		GD25_init();
		GD25_erase(0,32);
		GD25_write(0,spi_send_data,4);
		GD25_read(0,spi_rx_data,4);
#endif
		return ( events ^ OSAL_RELOAY_TIMER_EVT );
	}

  // Discard unknown events
  return 0;
}




