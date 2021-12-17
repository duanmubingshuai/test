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
    Filename:       gpio_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "pwm_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "pwm.h"
#include "led_light.h"

/*
    board and feature config
*/
//#define DEF_BOARD_6252_DONGLE
#define DEF_BOARD_6222_EVBKIT

#ifdef DEF_EVB_BOARD_6252_DONGLE

    #define GPIO_GREEN    P7
    #define GPIO_BLUE     P3
    #define GPIO_RED      P2

#endif

#ifdef DEF_BOARD_6222_EVBKIT

    #define GPIO_GREEN    P32
    #define GPIO_BLUE     P31
    #define GPIO_RED      P23

#endif

#define PIN_ALWAYS_FULLMUX_PWM       0x01
#define PIN_FULLMUX_PWM_AND_OTHER    0x02
#define LIGHT_BLINK_DEMO             0x03

#define PWM_DEMO_TYPE               LIGHT_BLINK_DEMO
/*********************************************************************
*/
static uint8 pwm_TaskID;
static uint32_t timer_cycle;

static gpio_pin_e led_pins[3] = {GPIO_GREEN,GPIO_BLUE,GPIO_RED};

void pwm_Init( uint8 task_id )
{
    pwm_TaskID = task_id;
    LOG("pwm demo start...\n");
    light_init(led_pins,3);
    #if (PWM_DEMO_TYPE == LIGHT_BLINK_DEMO)
    light_blink_evt_cfg(pwm_TaskID,PWM_LIGHT_PRCESS_EVT);
    light_blink_set(LIGHT_RED, 2, 20);
    #endif
    osal_start_reload_timer(pwm_TaskID,PWM_LIGHT_CONTROL,500);
}

uint16 pwm_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != pwm_TaskID)
    {
        return 0;
    }

    if(events & PWM_LIGHT_PRCESS_EVT)
    {
        #if (PWM_DEMO_TYPE == LIGHT_BLINK_DEMO)
        light_blink_porcess_evt();
        #endif
        return (events ^ PWM_LIGHT_PRCESS_EVT);
    }

    if(events & PWM_LIGHT_CONTROL)
    {
        #if (PWM_DEMO_TYPE == PIN_ALWAYS_FULLMUX_PWM)

        if((timer_cycle&0x03) == 0)
        {
            LIGHT_ONLY_RED_ON;
        }
        else if((timer_cycle&0x03) == 1)
        {
            LIGHT_ONLY_GREEN_ON;
        }
        else if((timer_cycle&0x03) == 2)
        {
            LIGHT_ONLY_BLUE_ON;
        }
        else
        {
            LIGHT_ON_OFF(0,0,0);
        }

        #elif (PWM_DEMO_TYPE == PIN_FULLMUX_PWM_AND_OTHER)

        switch((timer_cycle&0x07))
        {
        case 0:
            LIGHT_ONLY_RED_ON;
            break;

        case 1:
            LIGHT_ONLY_GREEN_ON;
            break;

        case 2:
            LIGHT_ONLY_BLUE_ON;
            break;

        case 3:
            LIGHT_ON_OFF(0,0,0);
            break;

        case 4:
            light_pwm_deinit();
            break;

        case 5:
            light_pwm_init();
            break;

        case 7:
        default:
            break;
        }

        #endif
        LOG("timer cycle %d\n",timer_cycle);
        timer_cycle++;
        return (events ^ PWM_LIGHT_CONTROL);
    }

    return 0;
}
/*********************************************************************
*********************************************************************/
