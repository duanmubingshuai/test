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
#include "pwrmgr.h"
#include "clock.h"
#include "OSAL_Timers.h"
#include "jump_function.h"

static uint8 watchdog_TaskID;
#define WATCHDOG_1000MS_EVENT    0x0001
#define WATCHDOG_1000MS_CYCLE    1000

#if(HAL_WDG_CFG_MODE == WDG_USE_INT_MODE)
void __attribute__((used))  hal_WATCHDOG_IRQHandler(void)
{
    volatile uint32_t a;
    a = AP_WDT->EOI;
    AP_WDT->CRR = 0x76;
}
#endif
static void hal_watchdog_feed(void)
{
    AP_WDT->CRR = 0x76;
}

static bool watchdog_init(WDG_CYCLE_Type_e cycle)
{
    volatile uint32_t a;
    uint8_t delay;
    hal_clk_gate_enable(MOD_WDT);

    if((AP_PCR->SW_RESET0 & 0x04)==0)
    {
        AP_PCR->SW_RESET0 |= 0x04;
        delay = 20;

        while(delay-->0);
    }

    if((AP_PCR->SW_RESET2 & 0x04)==0)
    {
        AP_PCR->SW_RESET2 |= 0x04;
        delay=20;

        while(delay-->0);
    }

    AP_PCR->SW_RESET2 &= ~0x20;
    delay=20;

    while(delay-->0);

    AP_PCR->SW_RESET2 |= 0x20;
    delay=20;

    while(delay-->0);

    a = AP_WDT->EOI;
    AP_WDT->TORR = cycle;
    #if(HAL_WDG_CFG_MODE == WDG_USE_INT_MODE)
    JUMP_FUNCTION(WDT_IRQ_HANDLER)                  =   (uint32_t)&hal_WATCHDOG_IRQHandler;
    AP_WDT->CR = 0x1F;//use int
    NVIC_SetPriority((IRQn_Type)WDT_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ((IRQn_Type)WDT_IRQn);
    #else
    JUMP_FUNCTION(WDT_IRQ_HANDLER)                  =   0;
    AP_WDT->CR = 0x1D;//no use int
    NVIC_DisableIRQ((IRQn_Type)WDT_IRQn);
    #endif
    AP_WDT->CRR = 0x76;
    osal_start_reload_timer(watchdog_TaskID, WATCHDOG_1000MS_EVENT, WATCHDOG_1000MS_CYCLE);
    return PPlus_SUCCESS;
}

static void hal_watchdog_wakeup_handler(void)
{
    watchdog_init(HAL_WDG_CFG_CYCLE);
}

static void hal_watchdog_sleep_handler(void)
{
    osal_stop_timerEx(watchdog_TaskID, WATCHDOG_1000MS_EVENT);
}

void Watchdog_Init(uint8 task_id)
{
    watchdog_TaskID = task_id;
    watchdog_init(HAL_WDG_CFG_CYCLE);
    hal_pwrmgr_register(MOD_WDT, hal_watchdog_sleep_handler, hal_watchdog_wakeup_handler);
}

uint16 Watchdog_ProcessEvent(uint8 task_id, uint16 events)
{
    if(events & WATCHDOG_1000MS_EVENT)
    {
        hal_watchdog_feed();
        return (events ^ WATCHDOG_1000MS_EVENT);
    }

    return 0;
}

