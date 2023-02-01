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
    @file     adc.h
    @brief    Contains all functions support for adc driver
    @version  0.0
    @date     18. Oct. 2017
    @author   qing.han

*******************************************************************************/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "bus_dev.h"
#include "gpio.h"


typedef enum{
    ADC_RESOL_BYPASS = 1,
    ADC_RESOL_DIV5   = 2,
    ADC_RESOL_DIV3   = 3
}adc_resol_t; //resolution


typedef enum
{
	ADC_VBAT = 0,  MIN_ADC_CH = 0,
	ADC_3V3  = 1,
    ADC_CH0  = 2,  ADC_CH0N_P4  = 2,
    ADC_CH1  = 3,  ADC_CH1P_P5  = 3,
    ADC_CH2  = 4,  ADC_CH2N_P6  = 4,
    ADC_CH3  = 5,  ADC_CH3P_P9  = 5,
	ADC_CH4  = 6,  ADC_CH4N_P10  = 6,
	ADC_CH5  = 7,  ADC_CH5P_P11  = 7,
    ADC_CH6  = 8,  ADC_CH6N_P12 = 8,
    ADC_CH7  = 9,  ADC_CH7P_P13 = 9, MAX_ADC_CH = 9,

    ADC_CH_NUM = 10,
} adc_ch_t;


typedef enum
{
    ADC_CLOCK_250K = 0,
    ADC_CLOCK_125K = 1,
    ADC_CLOCK_6250 = 2,
    ADC_CLOCK_3152 = 3,
} adc_CLOCK_SEL_t;



void hal_adc_init(void);

uint32_t hal_adc_sample(adc_ch_t ch, adc_resol_t resolution, int samples);


#ifdef __cplusplus
}
#endif

#endif
