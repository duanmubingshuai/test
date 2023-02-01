/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree tvoid SM_Init0( uint8 task_id )
  abide by the terms of these agreements. 
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
#include "gpio.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "jump_function.h"
#include "log.h"
#include "adc.h"
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


/*********************************************************************
    LOCAL FUNCTIONS
*/
static void adcMeasureTask( void );

/*********************************************************************
    PROFILE CALLBACKS
*/

/*********************************************************************
    PUBLIC FUNCTIONS
*/

void adc_Init( uint8 task_id )
{
	LOG("adc_init\n");
    adcDemo_TaskID = task_id;
    osal_start_timerEx(adcDemo_TaskID, adcMeasureTask_EVT ,1);
}

uint16 adc_ProcessEvent( uint8 task_id, uint16 events)
{
	if(task_id != adcDemo_TaskID){
		return 0;
	}

    if ( events & adcMeasureTask_EVT )
    {
        adcMeasureTask();
        osal_start_timerEx(adcDemo_TaskID, adcMeasureTask_EVT ,500);
        return (events ^ adcMeasureTask_EVT);
    }

    return 0;
}



static void adcMeasureTask( void )
{
	uint32_t adc_val;
	
	adc_val = hal_adc_sample(ADC_CH3P_P9, ADC_RESOL_BYPASS, 10);
//	adc_val = hal_adc_sample(ADC_CH3P_P9, ADC_RESOL_DIV5, 10);
//	adc_val = hal_adc_sample(ADC_CH3P_P9, ADC_RESOL_DIV3, 10);
	
    LOG("adcVal %d\n", adc_val);
}

