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
#include "watchdog.h"
#include "error.h"
#include "clock.h"
#include "jump_function.h"

extern volatile uint8 g_clk32K_config;
extern uint32_t s_config_swClk1;
#if(CFG_WDT_ENABLE==1)
WDG_CYCLE_Type_e g_wdt_cycle = 0xFF;//valid value:0~7.0xFF:watchdog disable.
#endif


#if(CFG_WDT_ENABLE==1)
void hal_watchdog_init(void)
{
	watchdog_init(g_wdt_cycle,/*int_mode*/0);//wdt polling_mode
	s_config_swClk1|=_CLK_WDT;
}
#endif

int hal_watchdog_config(WDG_CYCLE_Type_e cycle)
{
	
#if(CFG_WDT_ENABLE==1)
    if(cycle > WDG_256S)
        return PPlus_ERR_INVALID_PARAM;
    else
        g_wdt_cycle = cycle;

    hal_watchdog_init();
    JUMP_FUNCTION_SET(HAL_WATCHDOG_INIT, (uint32_t)&hal_watchdog_init);
	return PPlus_SUCCESS;
#else
	(void) cycle;
	return PPlus_ERR_UNINITIALIZED;
#endif	
    
}
