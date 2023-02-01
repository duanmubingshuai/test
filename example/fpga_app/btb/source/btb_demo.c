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
#include "i2c_demo.h"
#include "log.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"

//#define LOG(...)                    {log_printf(__VA_ARGS__);}

uint8 application_TaskID;

struct reg_combination
{
	uint32_t reg_address;
	char * name;
};
#define REG_COMBI_LENTH 10
const struct reg_combination reg_add[REG_COMBI_LENTH] = {
	{0x40005000,"i2c     "},
	{0x4000f00c,"AON     "},
	{0x4000f07c,"PCRM_ADC"},
	{0x40001000,"Timer   "},
	{0x40006000,"spi     "},
	{0x4000e028,"pwm     "},
	{0x40000030,"pcr     "},
	{0x1404420c,"usb_int "},
	{0x40003828,"iomux   "},
	{0x40008034,"gpio    "}
	
};
	

void i2c_Init(uint8 task_id)
{		
	application_TaskID = task_id;
	LOG("btb demo\n");
	osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 50);
}

extern void i2c_test(void);
uint16 i2c_ProcessEvent( uint8 task_id, uint16 events )
{	
	static uint8_t reg_sign = 0;
	static uint16_t count = 0;
//	LOG("reg_sign = %d\n count = %d\n",reg_sign,count);
	if(reg_sign >= REG_COMBI_LENTH)
	{
		return 0;
	}
	if (count > 100)
	{
		reg_sign ++;
		count = 0;
	}

	if(task_id != application_TaskID){
		return 0;
	}
	
	if ( events & OSAL_RELOAY_TIMER_EVT )
	{
//		i2c_test();
		reg_test_function(reg_add[reg_sign].reg_address,reg_add[reg_sign].name);
		count++;
		return ( events ^ OSAL_RELOAY_TIMER_EVT );
	}
	
  return 0;
}
