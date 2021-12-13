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
  Filename:       adc_demo.c
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
#include "log.h"
#include "adc_demo.h"
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


/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 adcDemo_TaskID;   // Task ID for internal task/event processing
/*
    channel:
    is_differential_mode:
    is_high_resolution:
    [bit7~bit2]=[p20,p15~p11],ignore[bit1,bit0]
    when measure adc(not battery),we'd better use high_resolution.
    when measure battery,we'd better use no high_resolution and keep the gpio alone.

    differential_mode is rarely used,
    if use please config channel as one of [ADC_CH3DIFF,ADC_CH2DIFF,ADC_CH1DIFF],
    and is_high_resolution as one of [0x80,0x20,0x08],
    then the pair of [P20~P15,P14~P13,P12~P11] will work.
    other adc channel cannot work.
*/
adc_Cfg_t adc_cfg =
{
	.channel = ADC_BIT(ADC_CH3P_P20)|ADC_BIT(ADC_CH3N_P15)|ADC_BIT(ADC_CH2P_P14)|ADC_BIT(ADC_CH2N_P24)|ADC_BIT(ADC_CH1P_P23)|ADC_BIT(ADC_CH1N_P11),
	//.channel = ADC_BIT(ADC_CH3P_P20)|ADC_BIT(ADC_CH2P_P14)|ADC_BIT(ADC_CH3N_P15),
	//.channel = ADC_BIT(ADC_CH2P_P14),
	
    .is_continue_mode = FALSE,
    .is_differential_mode = 0x00,
    .is_high_resolution = 0x7f,
};

/*********************************************************************
    LOCAL FUNCTIONS
*/
static void adc_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void adcMeasureTask( void );

/*********************************************************************
    PROFILE CALLBACKS
*/

/*********************************************************************
    PUBLIC FUNCTIONS
*/

void adc_Init( uint8 task_id )
{
	//unsigned int buf[6]={4000000,4000000,4000000,4000000,4000000,4000000};
	
    adcDemo_TaskID = task_id;	
	//_symrom_hal_adc_set_lambda(buf);
    adcMeasureTask();
}

uint16 adc_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
    //LOG("adc_ProcessEvent: 0x%x\n",events);

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( adcDemo_TaskID )) != NULL )
        {
            adc_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & 0x20 )
    {
        return (events ^ 0x20);
    }

    if ( events & adcMeasureTask_EVT )
    {
        adcMeasureTask();
        return (events ^ adcMeasureTask_EVT);
    }

    // Discard unknown events
    return 0;
}

static void adc_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
}

static void adc_evt(adc_Evt_t* pev)
{
	float value = 0;
	bool is_high_resolution = FALSE;
	bool is_differential_mode = FALSE;
	uint8_t ch = 0;
	static uint8_t coutner = 0;
	
	is_high_resolution = (adc_cfg.is_high_resolution & BIT(pev->ch))?TRUE:FALSE;
	is_differential_mode = (adc_cfg.is_differential_mode & BIT(pev->ch))?TRUE:FALSE;
	value = hal_adc_value_cal_x(pev->ch,pev->data, pev->size, is_high_resolution,is_differential_mode);

	switch(pev->ch)
	{
		case ADC_CH1N_P11:
		ch=11;
		break;

		case ADC_CH1P_P23:
		ch=23;
		break;

		case ADC_CH2N_P24:
		ch=24;
		break;

		case ADC_CH2P_P14:
		ch=14;
		break;

		case ADC_CH3N_P15:
		ch=15;
		break;

		case ADC_CH3P_P20:
		ch=20;
		break;

		default:
		break;
	}

	if(ch!=0)
	{
		LOG("P%d %d mv ",ch,(int)(value*1000));
	}
	else
	{
		LOG("invalid channel\n");
	}
	coutner++;
	if(coutner>=6)//adc channel enable number
	{
		coutner = 0;
		LOG("\n");
	}
	//LOG(" mode:%d \n",adc_cfg.is_continue_mode);
}

static void adcMeasureTask( void )
{
	int ret;
	bool batt_mode = TRUE;//FALSE;//TRUE;
	uint8_t batt_ch = ADC_CH3P_P20;
	GPIO_Pin_e pin;

	LOG("\nadcMeasureTask\n");
	if(FALSE == batt_mode)
	{
		ret = hal_adc_config_channel_x(adc_cfg, adc_evt);
	}
	else
	{
		if(((BIT(batt_ch) & adc_cfg.channel) == 0) || adc_cfg.is_differential_mode)
		{
			LOG("Error config parameter!\n");
			return;
		}
		
		pin = s_pinmap[batt_ch];
		_symrom_gpio_cfg_analog_io(pin,Bit_DISABLE);
		_symrom_gpio_write(pin, 1);
		ret = hal_adc_config_channel_x(adc_cfg, adc_evt);
		_symrom_gpio_cfg_analog_io(pin,Bit_DISABLE);
	}

	if(ret != PPlus_SUCCESS)
	{
		LOG("ret = %d\n",ret);
		return;
	}
	
	//_symrom_hal_adc_start();
	hal_adc_start_x();
	if(adc_cfg.is_continue_mode == FALSE)
	{
		osal_start_timerEx(adcDemo_TaskID, adcMeasureTask_EVT,1000);
	}
}
