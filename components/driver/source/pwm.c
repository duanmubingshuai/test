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

/*******************************************************************************
* @file		pwm.c
* @brief	Contains all functions support for pwm driver
* @version	0.0
* @date		30. Oct. 2017
* @author	Ding
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "rom_sym_def.h"
#include "gpio.h"
#include "clock.h"
#include "pwm.h"
#include "pwrmgr.h"
//#include "pico_reg_pwm.h"

/*simple pwm,for fpga test*/
#define PWM_PIN_NUM 14
const uint8_t pwm_pin_map[PWM_PIN_NUM][3]=
{
	{(uint8_t)P0, (uint8_t)FMUX_P0_PWM0,  0},
	{(uint8_t)P1, (uint8_t)FMUX_P1_PWM1,  1},
	{(uint8_t)P2, (uint8_t)FMUX_P2_PWM0,  0},
	{(uint8_t)P3, (uint8_t)FMUX_P3_PWM1,  1},
	{(uint8_t)P6, (uint8_t)FMUX_P6_PWM2,  2},
	{(uint8_t)P7, (uint8_t)FMUX_P7_PWM3,  3},
	{(uint8_t)P8, (uint8_t)FMUX_P8_PWM4,  4},
	{(uint8_t)P11,(uint8_t)FMUX_P11_PWM5, 5},
	{(uint8_t)P12,(uint8_t)FMUX_P12_PWM2, 2},
	{(uint8_t)P13,(uint8_t)FMUX_P13_PWM3, 3},
	{(uint8_t)P14,(uint8_t)FMUX_P14_PWM4, 4},
	{(uint8_t)P15,(uint8_t)FMUX_P15_PWM5, 5},
	{(uint8_t)P18,(uint8_t)FMUX_P18_PWM0, 0},
	{(uint8_t)P19,(uint8_t)FMUX_P19_PWM1, 1},
};


int pwm_init(void)
{
  clk_gate_enable(MOD_PWM);
  clk_reset(MOD_PWM);
  clk_gate_disable(MOD_PWM);
  pwrmgr_register(MOD_PWM, NULL, NULL);
  return PPlus_SUCCESS;
}
void pwm_deinit(void)
{
  clk_gate_disable(MOD_PWM);
  pwrmgr_unregister(MOD_PWM);

}


uint8_t pwm_get_channel(gpio_pin_e pin)
{
	uint8_t ch = 0xFF,i;

	for(i = 0; i< PWM_PIN_NUM; i++)
	{
	  if((uint8_t)pin == pwm_pin_map[i][0])
		return pwm_pin_map[i][2];
	}
	return 0xff;
}
gpio_fmux_e pwm_get_fmux(gpio_pin_e pin)
{
	uint8_t ch = 0xFF,i;

	for(i = 0; i< PWM_PIN_NUM; i++)
	{
	  if((uint8_t)pin == pwm_pin_map[i][0])
	     return (gpio_fmux_e)pwm_pin_map[i][1];
	}
	return FMUX_UNKNOW;
}


int pwm_ch_start(pwm_cfg_t* pcfg)
{
	int i=0;

	gpio_fmux_e fmux =  pwm_get_fmux(pcfg->pin);
	if(fmux == FMUX_UNKNOW)
	  return PPlus_ERR_INVALID_PARAM;
		
  clk_gate_enable(MOD_PWM);
	
	////////////////

  PWM_DISABLE_CH(pcfg->ch);
  PWM_SET_DIV(pcfg->ch, pcfg->div);
  PWM_SET_MODE(pcfg->ch, pcfg->mode);
  PWM_SET_POL(pcfg->ch, pcfg->polarity);
  PWM_INSTANT_LOAD_CH(pcfg->ch);

  PWM_NO_LOAD_CH(pcfg->ch);
  PWM_SET_CMP_VAL(pcfg->ch, pcfg->cmp_val);
  PWM_SET_TOP_VAL(pcfg->ch, pcfg->top_val);
  PWM_LOAD_CH(pcfg->ch);

  gpio_fmux_set(pcfg->pin, fmux);

  PWM_ENABLE_CH(pcfg->ch);

  pwrmgr_lock(MOD_PWM);
  PWM_ENABLE_ALL();

  return PPlus_SUCCESS;
}

int pwm_ch_stop(pwm_cfg_t* pcfg)
{
  int i;

  pcfg->cmp_val = 0;
  
	PWM_DISABLE_CH(pcfg->ch);
	PWM_NO_LOAD_CH(pcfg->ch);
	PWM_NO_INSTANT_LOAD_CH(pcfg->ch);
	PWM_SET_DIV(pcfg->ch, 0);
	PWM_SET_MODE(pcfg->ch, 0);
	PWM_SET_POL(pcfg->ch, 0);
	PWM_SET_TOP_VAL(pcfg->ch, 0);
	PWM_SET_CMP_VAL(pcfg->ch, 0);

	if((pcfg->pin != P2) && (pcfg->pin != P3))
	{
		gpio_fmux_set(pcfg->pin,FMUX_GPIO);
	}
	else
	{
		gpio_fmux_set(pcfg->pin,FMUX_P2_GPIO);
	}
	
	PWM_DISABLE_CH(pcfg->ch);

	for(i = 0; i< PWM_CH_NUM; i++)
	  if(read_reg(PWM_CTL0_ADDR(i)) & BIT(1))
	    return PPlus_SUCCESS;
		
	pwrmgr_unlock(MOD_PWM);  
	PWM_DISABLE();
	clk_gate_disable(MOD_PWM);
  return PPlus_SUCCESS;
}



int pwm_update(pwm_cfg_t* pcfg)
{

  if((read_reg(PWM_CTL0_ADDR(pcfg->ch)) & BIT(1)) == 0){
    pwm_ch_start(pcfg);
    return PPlus_SUCCESS;
  }

  PWM_NO_LOAD_CH(pcfg->ch);
  PWM_SET_CMP_VAL(pcfg->ch, pcfg->cmp_val);
  PWM_SET_TOP_VAL(pcfg->ch, pcfg->top_val);
  PWM_LOAD_CH(pcfg->ch);

  PWM_SET_DIV(pcfg->ch,pcfg->div);
	
	return PPlus_SUCCESS;
}


