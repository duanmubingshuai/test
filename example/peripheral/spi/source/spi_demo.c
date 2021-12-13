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
  Filename:       spi_demo.c
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
#include "spi.h"
#include "spiflash.h"
#include "spi_demo.h"
#include "log.h"
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
extern void _symrom_WaitMs(uint32_t msecond);


/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 timer_TaskID;

hal_spi_t spi ={
	.spi_index = SPI0,
};

spi_Cfg_t spi_cfg = {	
	
	.sclk_pin = P36,
	.ssn_pin = P37,
	.MOSI = P35,
	.MISO = P38,
	
	.baudrate = 1000000,
	.spi_tmod = SPI_TRXD,
	.spi_scmod = SPI_MODE0,
	
	.int_mode = false,
	.force_cs = true,
	.evt_handler = NULL,
};


extern int spi_bus_init(hal_spi_t* spi_ptr,spi_Cfg_t cfg);
void spi_test(void)
{
	volatile uint32_t flash_id = 0;
	int ret;
	
	_symrom_gpio_fmux_control(P35,Bit_DISABLE);
	_symrom_gpio_fmux_control(P36,Bit_DISABLE); 
	_symrom_gpio_fmux_control(P37,Bit_DISABLE); 
	_symrom_gpio_fmux_control(P38,Bit_DISABLE);
	
	//for(;;)
	{		
		ret = spi_bus_init(&spi,spi_cfg);
		if(ret != PPlus_SUCCESS)
		{
			LOG("error:%d %d \n",__LINE__,ret);
			while(1);
		}
							
		flash_id = spiflash_read_identification();
		LOG("flash_id:0x%x\n",flash_id);
		
		if((flash_id == 0) || (flash_id == 0xffffffff))
		{
			LOG("error:%d %d \n",__LINE__,ret);
			while(1);	
		}									
		spi_bus_deinit(&spi);
			
		//_symrom_WaitMs(1000);
	}
}

void spi_Init( uint8 task_id )
{
    timer_TaskID = task_id;
    LOG("spi demo\n");
					
	spi_test();
    _symrom_osal_start_reload_timer( timer_TaskID, TIMER_1000_MS_EVT, 1000);
}


uint16 spi_ProcessEvent( uint8 task_id, uint16 events )
{
    if (events & TIMER_1000_MS_EVT )
    {
		spi_test();
        return (events ^ TIMER_1000_MS_EVT);
    }

    return 0;
}
