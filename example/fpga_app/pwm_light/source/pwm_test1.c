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
    Filename:       pwm_test.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "pwm_test.h"
#include "pwm.h"
#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "log.h"

#include "pwrmgr.h"

uint8 pwmDemo_TaskID;   // Task ID for internal task/event processing

const static uint16_t   gradual_cntTopval = 999;
static uint8_t          gradual_ratio = 100;
static bool             gradual_setp_up;


static void pwm_dead_init(uint8_t dead_ratio)
{
    const static uint16_t cmpval = 299;
    const static uint16_t cntTopval = 499;
    PWM_DISABLE_CH_23;
    hal_pwm_init(PWM_CH2, PWM_CLK_NO_DIV, PWM_CNT_UP_AND_DOWN, PWM_POLARITY_FALLING);
    hal_pwm_set_count_val(PWM_CH2, cmpval, cntTopval);
    hal_pwm_open_channel(PWM_CH2, P34);
    hal_pwm_init(PWM_CH3, PWM_CLK_NO_DIV, PWM_CNT_UP_AND_DOWN, PWM_POLARITY_RISING);
    hal_pwm_set_count_val(PWM_CH3, cmpval+((cntTopval+1)/dead_ratio), cntTopval);
    hal_pwm_open_channel(PWM_CH3, P33);
    PWM_ENABLE_CH_23;
}


static void pwm_complement_init(void)
{
    const static uint16_t cmpval = 499;
    const static uint16_t cntTopval = 999;
    PWM_DISABLE_CH_01;
    hal_pwm_init(PWM_CH0, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_FALLING);
    hal_pwm_set_count_val(PWM_CH0, cmpval, cntTopval);
    hal_pwm_open_channel(PWM_CH0, P31);
    hal_pwm_init(PWM_CH1, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_RISING);
    hal_pwm_set_count_val(PWM_CH1, cmpval, cntTopval);
    hal_pwm_open_channel(PWM_CH1, P32);
    PWM_ENABLE_CH_01;
}


static void pwm_16k_dpi1000_init(void)
{
    const static uint16_t cmpval = 499;
    const static uint16_t cntTopval = 999;
    AP_PWM->pwmen &= ~BIT(16);
    hal_pwm_init(PWM_CH4, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_FALLING);
    hal_pwm_set_count_val(PWM_CH4, cmpval, cntTopval);
    hal_pwm_open_channel(PWM_CH4, P0);
    AP_PWM->pwmen |= BIT(16);
}


static void pwm_gradual_init(void)
{
    AP_PWM->pwmen &= ~BIT(17);
    hal_pwm_init(PWM_CH5, PWM_CLK_NO_DIV, PWM_CNT_UP, PWM_POLARITY_FALLING);
    hal_pwm_set_count_val(PWM_CH5, gradual_cntTopval, gradual_cntTopval);
    hal_pwm_open_channel(PWM_CH5, P11);
    AP_PWM->pwmen |= BIT(17);
}


static void pwm_gradual_set_cmpVal(uint16_t cmp_val)
{
    hal_pwm_set_count_val(PWM_CH5, cmp_val, gradual_cntTopval);
    PWM_SET_DIV(PWM_CH5, PWM_CLK_NO_DIV);
}


void pwm_Init( uint8 task_id )
{
    pwmDemo_TaskID =task_id;

    hal_pwm_module_init();
    pwm_complement_init();
    pwm_dead_init(10);
    pwm_16k_dpi1000_init();
    pwm_gradual_init();
    hal_pwrmgr_lock(MOD_PWM);
    osal_start_timerEx(pwmDemo_TaskID, BLEMESH_PWM_EVT,100);
}

	
uint16 pwm_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != pwmDemo_TaskID)
    {
        return 0;
    }

    if ( events & BLEMESH_PWM_EVT )
    {
        if(gradual_setp_up)
        {
            if(gradual_ratio != 100)
            {
                gradual_ratio += 2;
            }
            else 
            {
                gradual_setp_up = false;
                gradual_ratio -= 2;
            }
            pwm_gradual_set_cmpVal(gradual_ratio*(gradual_cntTopval+1)/100-1);
        }
        else
        {
            if(gradual_ratio != 0)
            {
                gradual_ratio -= 2;
            }
            else 
            {
                gradual_setp_up = true;
                gradual_ratio += 2;
            }
            if(gradual_ratio == 0)
            {
                pwm_gradual_set_cmpVal(0);
            }
            else
            {
                pwm_gradual_set_cmpVal(gradual_ratio*(gradual_cntTopval+1)/100-1);
            }
        }

        if(gradual_ratio == 0 || gradual_ratio == 100)
        {
            osal_start_timerEx(pwmDemo_TaskID, BLEMESH_PWM_EVT, 200);	
        }
        else
        {
            osal_start_timerEx(pwmDemo_TaskID, BLEMESH_PWM_EVT, 10);	
        }
        	
        return (events ^ BLEMESH_PWM_EVT);
    }

    return 0;
}




