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
    @file   adc.c
    @brief  Contains all functions support for adc driver
    @version  0.0
    @date   18. Oct. 2017
    @author qing.han

*******************************************************************************/
#include "rom_sym_def.h"
#include <string.h>
#include "error.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "clock.h"
#include "adc.h"
#include "log.h"
#include "jump_function.h"
#include "version.h"
#include "pico_reg.h"

#define ADCCLKSEL 2

const uint8_t adc_samptime[4] = {1, 2, 40, 80};			//00: 250k,01:125k,10:6250,11:3125

const uint8_t s_pinmap[ADC_CH_NUM] = {GPIO_DUMMY, GPIO_DUMMY, P4, P5, P6, P9, P10, P11, P12, P13};

void gpio_cfg_analog_io(adc_ch_t ch, bit_action_e value)
{
    if(s_pinmap[ch] == GPIO_DUMMY)
        return;

    if(value)
    {
		gpio_pull_set(s_pinmap[ch], GPIO_FLOATING);

		gpio_fmux_set(s_pinmap[ch], 3);
    }
    else
    {
		gpio_fmux_control(s_pinmap[ch], Bit_DISABLE);
    }

}

int set_sampling_resolution(adc_ch_t ch, adc_resol_t resolution)
{

	if(ch == ADC_VBAT)
	{
		pcrm_ana_ctl1_vbat_det_en_setf(1);
	}
	
	if(ch == ADC_3V3)
	{
		pcrm_ana_ctl1_vbat_det_en_setf(1);
	}	
	
	if(ch >= 2 && ch < ADC_CH_NUM)
	{
		uint8_t aio = 2*ch + 8;
		
		subWriteReg(&AP_PCRM->ANA_CTL1, aio+1, aio, resolution);
	
	}
	
	return PPlus_SUCCESS;
}

void hal_adc_init(void)
{
    pwrmgr_register(MOD_ADCC, NULL, NULL);
}

uint32_t hal_adc_sample(adc_ch_t ch, adc_resol_t resolution, int samples)
{
	
    int i;
    uint32_t adc_val = 0;

	pcrm_ana_ctl1_set(0x00);        //ANA_CTL1
	
	pcrm_ana_ctl_adc12b_en_setf(0x0);
	pcrm_ana_ctl_ana_ldo_en_setf(0x0);

	clk_gate_disable(MOD_ADCC);
	clk_reset(MOD_ADCC);

	if(pcr_sw_clk_clkg_adcc_getf() == 0)
	{
		clk_gate_enable(MOD_ADCC);
	}

	//    CLK_1P28M_ENABLE;
	pcrm_clksel_clk_1p28m_en_setf(1);

	//    ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
	pcrm_clkhf_ctl0_xtal_clk_dig_en_setf(1);

	pcrm_clkhf_ctl1_pack(3, 0, 0, 1, 1, 0, 0, 1, 1, 2, 0, 1, 3, 1); //ENABLE_DLL; en_dig_clk_32m; ENABLE_ADC
	
	pcrm_adc_ctl4_pack(0x700, 0x500, 1, 1, 0, 0x2, 1);	//ADC_CTL4 cmp_hth	cmp_lth

	if(set_sampling_resolution(ch, resolution))
		return 0xffffffff;

	if(ch%2)
	{
		pcrm_ana_ctl_pack(0, 1, 0, 0, 0, 0, 0, 0, 0, 1, (ch/2), 1, 1, 1);	//channel select;Power up analog LDO;Power up ADC;

	}
	else
	{
		pcrm_ana_ctl_pack(0, 1, 0, 0, 0, 0, 0, 1, 0, 0, (ch/2), 1, 1, 1);

	}
	
	pcrm_adc_ctl4_adc_clk_sel_setf(ADCCLKSEL);	//adc_clock_sel
	
	
	for(i = 0; i< samples; i++)
	{
		WaitUs(adc_samptime[ADCCLKSEL]*4);
		adc_val += pcrm_adc_sync_data_out_adc_sync_data_out_getf();
	}
	
	gpio_cfg_analog_io(ch, Bit_DISABLE);

	clk_reset(MOD_ADCC);
	clk_gate_disable(MOD_ADCC);
	
	pcrm_ana_ctl1_set(0x00);        //ANA_CTL1
	
	return adc_val/samples;
}
