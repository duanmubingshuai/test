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
  Filename:       peripheral_test.c
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
#include "timer.h"
#include "pwm.h"
#include "peripheral_test.h"
#include  "log.h"

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

static uint8 peripheral_TaskID;
static uint8_t s_testCase = 0;


static void toggle_gpio(void)
{
	gpio_write(P1,1);
	gpio_write(P1,0);
}

static void delay_ms(void)
{
	WaitMs(50);
}

void up_output(uint8_t testcase)
{
	pwm_ch_t ch,ch_mul[6];
	int count,i;
	unsigned int counter = 0;
	
	pwm_module_init();	
	
	ch.pwmN = PWM_CH0;

	ch.pwmPin = P0;
	ch.pwmDiv = PWM_CLK_NO_DIV;
	ch.pwmMode = PWM_CNT_UP;
	ch.pwmPolarity = PWM_POLARITY_FALLING;//PWM_POLARITY_RISING;

	if(testcase == 0)//16000000/(799+1)=20Khz (199+1)/(799+1)=25%
	{
		ch.cmpVal = 199;
		ch.cntTopVal = 799;	
		toggle_gpio();
		pwm_ch_start(ch);
		toggle_gpio();
		
		delay_ms();	
		pwm_ch_stop(ch);
		delay_ms();
		while(1);;
	}
	else if(testcase == 1)//0 2/8 3/8 4/8 5/8 6/8 7/8 1[0,25,37.5,50,62.5,75,87.5,100]
	{
		LOG("->(+)\n");
		
		toggle_gpio();
		for(count=0;count<=7;count++)
		{
			if((count != 0) && (count != 1) && (count != 6) && (count != 7))
			{
				continue;
			}
			
			ch.cmpVal = count;
			ch.cntTopVal=7;			
			LOG("val:%d top:%d\n",ch.cmpVal,ch.cntTopVal);//[0,25,37.5,50,62.5,75,87.5,100]
			pwm_ch_start(ch);
			delay_ms();
		}
		
		toggle_gpio();
		LOG("->(-)\n");
		for(count=7;count>=0;count--)//[100 87.5 75 62.5 50 37.5 25 0]
		{
			if((count != 0) && (count != 1) && (count != 6) && (count != 7))
			{
				continue;
			}
			
			ch.cmpVal = count;
			ch.cntTopVal=7;			
			LOG("val:%d top:%d\n",ch.cmpVal,ch.cntTopVal);
			pwm_ch_start(ch);	
			delay_ms();
		}
		LOG("finish\n");		
		toggle_gpio();

		//clk_reset(MOD_PWM);
		while(1);
	}
	else if(testcase == 2)//20K 10K 5K 2.5K 1.25K 0.625K 0.3125K 0.15625K
	{
		ch.cmpVal = 199;
		ch.cntTopVal = 799;
		ch.pwmDiv=PWM_CLK_NO_DIV;
		
		LOG("->(+)\n");		
		ch.pwmDiv=PWM_CLK_NO_DIV;
		
		toggle_gpio();
		for(count=0;count<=7;count++)
		{			
			LOG("ch.pwmDiv:%d\n",ch.pwmDiv);
			pwm_ch_start(ch);//20K 10K 5K 2.5K 1.25K 0.625K 0.3125K 0.15625K
			LOG("run_pwm\n");
			ch.pwmDiv++;
			delay_ms();
		}
		
		toggle_gpio();		
		LOG("->(-)\n");
		ch.pwmDiv=PWM_CLK_DIV_128;
		for(count=0;count<=7;count++)
		{			
			LOG("ch.pwmDiv:%d\n",ch.pwmDiv);
			pwm_ch_start(ch);//0.15625K 0.3125K  0.625K 1.25K  2.5K 5K 10K 20K 
			LOG("run_pwm\n");
			ch.pwmDiv--;
			delay_ms();
		}
		toggle_gpio();
		LOG("finish\n");
		//clk_reset(MOD_PWM);	
		while(1);
	}
	else if(testcase == 3)
	{		
		ch.cmpVal = 199;
		ch.cntTopVal = 799;
		
		toggle_gpio();		
		for(ch.pwmN = PWM_CH0;ch.pwmN<=PWM_CH5;ch.pwmN++)
		{		
			pwm_ch_start(ch);
			LOG("pwm ch:%d\n",ch.pwmN);
			delay_ms();	
			pwm_ch_stop(ch);
			delay_ms();
		}
		LOG("finish\n");
		toggle_gpio();
		//clk_reset(MOD_PWM);	
		while(1);
	}
	else if(testcase == 4)
	{
		ch.pwmN = PWM_CH0;
		ch.pwmPin = P0;
		ch.pwmDiv = PWM_CLK_NO_DIV;
		ch.pwmMode = PWM_CNT_UP;
		ch.cmpVal = 199;
		ch.cntTopVal = 799;	
		
		toggle_gpio();	
		ch.pwmPolarity = PWM_POLARITY_FALLING;
		pwm_ch_start(ch);
		delay_ms();
		
		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();
				
		toggle_gpio();		
		ch.pwmPolarity = PWM_POLARITY_RISING;
		pwm_ch_start(ch);
		delay_ms();		

		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();
		
		toggle_gpio();
		ch.pwmPolarity = PWM_POLARITY_FALLING;
		pwm_ch_start(ch);
		delay_ms();
		
		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();
	
		toggle_gpio();	
		ch.pwmPolarity = PWM_POLARITY_RISING;
		pwm_ch_start(ch);
		delay_ms();
		
		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();
		
		//clk_reset(MOD_PWM);
		LOG("finish\n");		
		while(1);
	}	
	pwm_module_deinit();
}

void up_and_down_output(uint8_t testcase)
{
	pwm_ch_t ch,ch_mul[6];
	int count,i;
	unsigned int value_start = 0;
	
	pwm_module_init();	
	ch.pwmN = PWM_CH0;
	ch.pwmPin = P0;
	ch.pwmDiv = PWM_CLK_NO_DIV;
	ch.pwmMode = PWM_CNT_UP_AND_DOWN;
	ch.pwmPolarity = PWM_POLARITY_FALLING;//PWM_POLARITY_RISING;

	if(testcase == 10)//8000000Hz/400=20Khz 300/400=75%
	{
		ch.cmpVal = 300;//400
		ch.cntTopVal = 400;		
		toggle_gpio();
		pwm_ch_start(ch);			
		toggle_gpio();
		
		LOG("run_pwm\n");
		
		delay_ms();	
		pwm_ch_stop(ch);
		delay_ms();
		
		//clk_reset(MOD_PWM);
		while(1);
	}
	else if(testcase == 11)//0 1/8=12.5% 2/8 3/8=37.5% 4/8 5/8=62.5% 6/8 7/8=87.5%
	{			
		value_start = 0;//0 or 1
		LOG("value_start:%d\n",value_start);
		
		LOG("->(+)\n");
		toggle_gpio();
		for(count=value_start;count<=7;count++)
		{
			ch.cmpVal = count;
			ch.cntTopVal=8;
			pwm_ch_start(ch);//[12.5 25 37.5 50 62.5 75 87.5]
		
			LOG("val:%d top:%d\n",ch.cmpVal,ch.cntTopVal);
			LOG("\n0x4000E000:0x%x\n",*(volatile int*)0x4000E000);
			LOG("0x4000E004:0x%x\n",*(volatile int*)0x4000E004);
			LOG("0x4000E008:0x%x\n\n",*(volatile int*)0x4000E008);
			
			delay_ms();		
		}	
		
		LOG("->(-)\n");
		
		toggle_gpio();
		for(count=7;count>=value_start;count--)
		{		
			ch.cmpVal = count;
			ch.cntTopVal=8;
			pwm_ch_start(ch);//[87.5 75 62.5 50 37.5 25 12.5]
		
			LOG("val:%d top:%d\n",ch.cmpVal,ch.cntTopVal);
			LOG("\n0x4000E000:0x%x\n",*(volatile int*)0x4000E000);
			LOG("0x4000E004:0x%x\n",*(volatile int*)0x4000E004);
			LOG("0x4000E008:0x%x\n\n",*(volatile int*)0x4000E008);
						
			if(count < 0)
				break;
			delay_ms();	
		}
		LOG("finish\n");
		toggle_gpio();
		clk_reset(MOD_PWM);
		while(1);
	}	
	else if(testcase == 12)//20K 10K 5K 2.5K 1.25K 0.625K 0.3125K 0.15625K
	{
		ch.cmpVal = 100;
		ch.cntTopVal = 400;	
		ch.pwmDiv=PWM_CLK_NO_DIV;
		ch.pwmPolarity = PWM_POLARITY_FALLING;
		
		LOG("->(+)\n");
		toggle_gpio();
		for(ch.pwmDiv=PWM_CLK_NO_DIV;ch.pwmDiv<=PWM_CLK_DIV_128;ch.pwmDiv++)
		{					
			pwm_ch_start(ch);//[20K 10K 5K 2.5K 1.25K 0.625K 0.3125K 0.15625K]
			LOG("ch.pwmDiv:%d\n",ch.pwmDiv);
			delay_ms();		
			if(ch.pwmDiv>=PWM_CLK_DIV_128)
			{
				break;			
			}
		}
		
		LOG("->(-)\n");		
		toggle_gpio();
		ch.pwmDiv=PWM_CLK_DIV_128;
		for(ch.pwmDiv=PWM_CLK_DIV_128;ch.pwmDiv>=PWM_CLK_NO_DIV;ch.pwmDiv--)
		{				
			pwm_ch_start(ch);//[0.15625K 0.3125K  0.625K 1.25K  2.5K 5K 10K 20K ]
			LOG("ch.pwmDiv:%d\n",ch.pwmDiv);
			delay_ms();				
			if(ch.pwmDiv<=PWM_CLK_NO_DIV)
			{
				break;			
			}
		}
		LOG("finish\n");
		
		toggle_gpio();	
		//clk_reset(MOD_PWM);
		while(1);
	}
	else if(testcase == 13)
	{
		ch.cmpVal = 100;
		ch.cntTopVal = 400;
		toggle_gpio();
		for(ch.pwmN = PWM_CH0;ch.pwmN<=PWM_CH5;ch.pwmN++)
		{
			pwm_ch_start(ch);
			delay_ms();
			
			LOG("pwm ch:%d\n",ch.pwmN);			
			pwm_ch_stop(ch);
			delay_ms();	
		}
		LOG("finish\n");
		toggle_gpio();
		//clk_reset(MOD_PWM);	
		while(1);
	}
	else if(testcase == 14)
	{
		ch.pwmN = PWM_CH0;
		ch.pwmPin = P0;
		ch.pwmDiv = PWM_CLK_NO_DIV;
		ch.pwmMode = PWM_CNT_UP_AND_DOWN;
		ch.cmpVal = 100;
		ch.cntTopVal = 400;	
		
		toggle_gpio();
		ch.pwmPolarity = PWM_POLARITY_FALLING;
		pwm_ch_start(ch);
		delay_ms();
				
		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();
	
		toggle_gpio();
		ch.pwmPolarity = PWM_POLARITY_RISING;		
		pwm_ch_start(ch);
		delay_ms();
		
		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();
		
		toggle_gpio();
		ch.pwmPolarity = PWM_POLARITY_FALLING;
		pwm_ch_start(ch);
		delay_ms();
		
		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();

		toggle_gpio();
		ch.pwmPolarity = PWM_POLARITY_RISING;		
		pwm_ch_start(ch);
		delay_ms();
		
		toggle_gpio();
		pwm_ch_stop(ch);
		delay_ms();
		
		//clk_reset(MOD_PWM);
		while(1);			
	}
	
	_symrom_pwm_module_deinit();
}


void temp_test_up(void)
{
	pwm_ch_t ch;

	pwm_module_init();	

	ch.pwmN = PWM_CH0;
	ch.pwmPin = P0;
	ch.pwmDiv = PWM_CLK_NO_DIV;
	ch.pwmMode = PWM_CNT_UP;
	//ch.pwmPolarity = PWM_POLARITY_FALLING;
	ch.pwmPolarity = PWM_POLARITY_RISING;
	
	ch.cmpVal = 799;
	ch.cntTopVal = 799;	
	toggle_gpio();
	pwm_ch_start(ch);
	delay_ms();

	ch.cmpVal = 0;
	ch.cntTopVal = 799;	
	toggle_gpio();
	pwm_ch_start(ch);
	delay_ms();

	ch.cmpVal = 799;
	ch.cntTopVal = 799;	
	toggle_gpio();
	pwm_ch_start(ch);
	delay_ms();

	ch.cmpVal = 0;
	ch.cntTopVal = 799;	
	toggle_gpio();
	pwm_ch_start(ch);
	delay_ms();
	
	while(1);;;	
}

void temp_test_up_and_down(void)
{
	pwm_ch_t ch;

	pwm_module_init();	

	ch.pwmN = PWM_CH0;
	ch.pwmPin = P0;
	ch.pwmDiv = PWM_CLK_NO_DIV;
	ch.pwmMode = PWM_CNT_UP_AND_DOWN;
	ch.pwmPolarity = PWM_POLARITY_FALLING;//0:ok.100%:error
	//ch.pwmPolarity = PWM_POLARITY_RISING;//100%:ok.0,error
	
	ch.cmpVal = 400;
	ch.cntTopVal = 400;	
	toggle_gpio();
	pwm_ch_start(ch);
	delay_ms();

//	ch.cmpVal = 0;
//	ch.cntTopVal = 400;	
//	toggle_gpio();
//	pwm_ch_start(ch);
//	delay_ms();
//	
	while(1);;;	
}

void Peripheral_Init( uint8 task_id )
{
	uint8_t testcase = 11;
		
    peripheral_TaskID = task_id;
    LOG("peripheral demo\n");
		
	switch(testcase)
	{			
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
			up_output(testcase);
			break;
			
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
			up_and_down_output(testcase);
			break;
			
		case 20:
			temp_test_up();
			break;
		case 21:
			temp_test_up_and_down();
			break;
		default:
			break;
	}
				
    //_symrom_osal_start_reload_timer( timer_TaskID, TIMER_1000_MS_EVT, 1000);
}



uint16 Peripheral_ProcessEvent( uint8 task_id, uint16 events )
{

    if (events & TIMER_1000_MS_EVT )
    {
        return (events ^ TIMER_1000_MS_EVT);
    }

    return 0;
}
