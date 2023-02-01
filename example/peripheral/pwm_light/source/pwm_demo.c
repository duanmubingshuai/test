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
  Filename:       pwm_light.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "types.h"
#include "OSAL.h"
#include "pwm.h"
#include "pwm_demo.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"

//#define LOG(...)                    {log_printf(__VA_ARGS__);}

uint8 pwmlight_TaskID = 0;
#define LIGHT_R  P13
#define LIGHT_G  P14
#define LIGHT_B  P1

#define LIGHT_TOP_VAL_MAX  999

#define PWMLIGHT_FADE_INTV 50


pwm_cfg_t g_pwm_chcfg[3];


void uart_cbk(comm_evt_t* pev)
{
	if(pev->data[0] == 'a')
	{
		osal_set_event(pwmlight_TaskID, PWMLIGHT_EVT_FADE_IN);
	}
	else if(pev->data[0] == 'b')
	{
		osal_set_event(pwmlight_TaskID, PWMLIGHT_EVT_FADE_OUT);
	}
	else if(pev->data[0] == 'x')
	{
		osal_set_event(pwmlight_TaskID, PWMLIGHT_EVT_500MS);
	}
}

void pwmlight_onoff(pwm_cfg_t* pcfg, bool onoff)
{
  pcfg->cmp_val = LIGHT_TOP_VAL_MAX;
  if(onoff)
    pwm_ch_start(pcfg);
  else
    pwm_ch_stop(pcfg);
}

void pwmlight_onoff_all(bool onoff)
{
  g_pwm_chcfg[0].cmp_val = LIGHT_TOP_VAL_MAX;
  g_pwm_chcfg[1].cmp_val = LIGHT_TOP_VAL_MAX;
  g_pwm_chcfg[2].cmp_val = LIGHT_TOP_VAL_MAX;

  if(onoff)
  {
    pwm_ch_start(&(g_pwm_chcfg[0]));
    pwm_ch_start(&(g_pwm_chcfg[1]));
    pwm_ch_start(&(g_pwm_chcfg[2]));
  }
  else
  {
    pwm_ch_stop(&(g_pwm_chcfg[0]));
    pwm_ch_stop(&(g_pwm_chcfg[1]));
    pwm_ch_stop(&(g_pwm_chcfg[2]));
  }
  

}


void pwmlight_setting(uint32_t cmp_val)
{
  g_pwm_chcfg[0].cmp_val = cmp_val;
  g_pwm_chcfg[1].cmp_val = cmp_val;
  g_pwm_chcfg[2].cmp_val = cmp_val;

  pwm_update(&(g_pwm_chcfg[0]));
  pwm_update(&(g_pwm_chcfg[1]));
  pwm_update(&(g_pwm_chcfg[2]));
}


void pwmlight_Init(uint8 task_id)
{
  int i;
	pwmlight_TaskID = task_id;

  g_pwm_chcfg[0].pin = LIGHT_R;
  g_pwm_chcfg[1].pin = LIGHT_G;
  g_pwm_chcfg[2].pin = LIGHT_B;

  for(i = 0; i< 3; i++)
  {
    g_pwm_chcfg[i].ch    = pwm_get_channel(g_pwm_chcfg[i].pin);
    g_pwm_chcfg[i].div   = PWM_CLK_DIV_16;
    g_pwm_chcfg[i].mode  = PWM_CNT_UP;
    g_pwm_chcfg[i].polarity =  PWM_POLARITY_RISING;
    g_pwm_chcfg[i].cmp_val = 0;
    g_pwm_chcfg[i].top_val = LIGHT_TOP_VAL_MAX;

  }

  pwm_init();
  
}


uint16 pwmlight_ProcessEvent( uint8 task_id, uint16 events )
{
  static bool r = false, g = false, b = false, rgb = false;
  static uint32_t fade_setting = 0; //change from 0~ 999
  uint32_t fade_step = 10;
  LOG("in processevent %d\n",events);
  if(task_id != pwmlight_TaskID){
    return 0;
  }
	
  if ( events & PWMLIGHT_EVT_R_ONOFF)
  {
    r = !r;
    pwmlight_onoff(&(g_pwm_chcfg[0]), r);
    return ( events ^ PWMLIGHT_EVT_R_ONOFF);
  }

  if ( events & PWMLIGHT_EVT_G_ONOFF)
  {
    g = !g;
    pwmlight_onoff(&(g_pwm_chcfg[1]), g);
    return ( events ^ PWMLIGHT_EVT_G_ONOFF);
  }

  if ( events & PWMLIGHT_EVT_B_ONOFF)
  {
    b = !b;
    pwmlight_onoff(&(g_pwm_chcfg[2]), b);
    return ( events ^ PWMLIGHT_EVT_B_ONOFF);
  }

  if ( events & PWMLIGHT_EVT_FADE_IN)
  {
    fade_setting += fade_step;
    if(fade_setting >= LIGHT_TOP_VAL_MAX)
      fade_setting = LIGHT_TOP_VAL_MAX;
    else
      osal_start_timerEx(pwmlight_TaskID, PWMLIGHT_EVT_FADE_IN, PWMLIGHT_FADE_INTV);
    LOG("A");
    pwmlight_setting(fade_setting);
    return ( events ^ PWMLIGHT_EVT_FADE_IN);
  }

  if ( events & PWMLIGHT_EVT_FADE_OUT)
  {
    fade_setting = (fade_setting > fade_step)? fade_setting - fade_step: 0;
    if(fade_setting > 0)
      osal_start_timerEx(pwmlight_TaskID, PWMLIGHT_EVT_FADE_OUT, PWMLIGHT_FADE_INTV);
    LOG("B");
    pwmlight_setting(fade_setting);
    return ( events ^ PWMLIGHT_EVT_FADE_OUT);
  }

  if ( events & PWMLIGHT_EVT_HALF)
  {
    pwmlight_setting(LIGHT_TOP_VAL_MAX/2);
    return ( events ^ PWMLIGHT_EVT_HALF);
  }
  
  if ( events & PWMLIGHT_EVT_ONOFF)
  {
    rgb = !rgb;
    pwmlight_onoff_all(rgb);
    return ( events ^ PWMLIGHT_EVT_ONOFF);
  }
  if ( events & PWMLIGHT_EVT_500MS)
  {
    LOG("~");
    osal_start_timerEx(pwmlight_TaskID, PWMLIGHT_EVT_500MS, 500);
    return ( events ^ PWMLIGHT_EVT_500MS);
  }
  // Discard unknown events
  return 0;
}
