#include "rom_sym_def.h"
#include "OSAL.h"
#include "pwm.h"
#include "pwm_demo.h"
#include "log.h"
#include "pwrmgr.h"
#include "error.h"
extern pwm_cfg_t pwm0;

void pwm_test1(void)
{
	volatile int ret;
	volatile int i= 0;
	int testcase = 1;
	const uint8_t cmp_val[11] = {0,10-1,20-1,30-1,40-1,50-1,60-1,70-1,80-1,90-1,100-1};
	const uint8 pin[] = {P0,P1,P2,P3,P6,P7,P8,P11,P12,P13,P14,P15,P18,P19};	
	LOG("pwm_test_in\n");
	//basic test
	while(0)
	{
		pwm_ch_start(&pwm0);
		ret =pwm_state(&pwm0);
		LOG("ret:%d\n",ret);
		WaitMs(1000);
		
		pwm_ch_stop(&pwm0);
		ret =pwm_state(&pwm0);
		LOG("ret:%d\n",ret);
		WaitMs(1000);
	}
	
	//feature test
	switch(testcase)
	{
		case 1://iomux for pwm 
#if 1	
			//test pwm pin expect P2P3
			for(i=0;i<sizeof(pin)/sizeof(pin[0]);i++)
			{
				if((pin[i] == P2) || (pin[i] == P3))
				{
					continue;
				}				
				pwm0.pin = pin[i];
				pwm0.mode = PWM_CNT_UP_AND_DOWN;
				pwm0.polarity = PWM_POLARITY_FALLING;
				pwm0.cmp_val = 9;
				pwm0.top_val = 99;
				pwm_ch_start(&pwm0);
				
				WaitMs(50);
				
				pwm_ch_stop(&pwm0);
				WaitMs(50);
			}
#else			
			//test P2 and P3
			pwm0.mode = PWM_CNT_UP_AND_DOWN;
			pwm0.polarity = PWM_POLARITY_FALLING;
			pwm0.cmp_val = 9;
			pwm0.top_val = 99;
			while(1)
			{
				pwm0.pin = P2;
				pwm_ch_start(&pwm0);
				WaitMs(500);
				
				pwm_ch_stop(&pwm0);
				WaitMs(1500);
				
				pwm0.pin = P3;
				pwm_ch_start(&pwm0);
				WaitMs(1500);
				
				pwm_ch_stop(&pwm0);
				WaitMs(500);				
			}
#endif			
			break;
		
		case 2://pwm no_div~128_div
			while(1)
			{
				LOG("testcase 2\n");
				pwm0.pin = P18;
				pwm0.mode = PWM_CNT_UP;
				pwm0.cmp_val = 10-1;
				pwm0.top_val = 200-1;
				pwm0.div = PWM_CLK_NO_DIV;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
				
				pwm0.div = PWM_CLK_DIV_2;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
							
				pwm0.div = PWM_CLK_DIV_4;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
							
				pwm0.div = PWM_CLK_DIV_8;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
							
				pwm0.div = PWM_CLK_DIV_16;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
			
				pwm0.div = PWM_CLK_DIV_32;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
				
				pwm0.div = PWM_CLK_DIV_64;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
				
				pwm0.div = PWM_CLK_DIV_128;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);		
				WaitMs(100);
			}
			break;	
			
		case 3://POLARITY,rising or failing
			while(1)
			{
				pwm0.pin = P18;
				//pwm0.mode = PWM_CNT_UP;
				pwm0.mode = PWM_CNT_UP_AND_DOWN;
				if(pwm0.mode == PWM_CNT_UP)
				{
					pwm0.cmp_val = 10-1;
					pwm0.top_val = 200-1;
				}
				else
				{
					pwm0.cmp_val = 10;
					pwm0.top_val = 200;
				}

				pwm0.polarity = PWM_POLARITY_RISING;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
				gpio_write(P12,1);
				gpio_write(P12,0);
			
				pwm0.polarity = PWM_POLARITY_FALLING;
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);		
				WaitMs(100);
			}
			break;
			
		case 4://Mode and cmp_val TopVal
			while(1)
			{
				pwm0.pin = P18;
				//pwm0.mode = PWM_CNT_UP;
				pwm0.mode = PWM_CNT_UP_AND_DOWN;
				if(pwm0.mode == PWM_CNT_UP)
				{
					pwm0.cmp_val = 100-1;
					pwm0.top_val = 200-1;
				}
				else
				{
					pwm0.cmp_val = 100;
					pwm0.top_val = 200;
				}
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
				
				pwm0.pin = P0;
				if(pwm0.mode == PWM_CNT_UP)
				{
					pwm0.cmp_val = 100-1;
					pwm0.top_val = 400-1;
				}
				else
				{
					pwm0.cmp_val = 100;
					pwm0.top_val = 400;
				}
				pwm_ch_start(&pwm0);
				WaitMs(100);
				pwm_ch_stop(&pwm0);
				WaitMs(100);
			}
			break;
			
		case 5:
			pwm0.pin = P18;
			pwm0.mode = PWM_CNT_UP;
			pwm0.polarity = PWM_POLARITY_FALLING;
			pwm0.cmp_val = 9;
			pwm0.top_val = 99;
			pwm_ch_start(&pwm0);
			while(1)
			{
				LOG("parameter change test:\n");//duty={0%~100%,+10%}
				for(i=0;i<11;i++)
				{
					pwm0.cmp_val = cmp_val[i];
					LOG("cmp+1:%d top+1:%d\n",(pwm0.cmp_val+1),(pwm0.top_val+1));
					gpio_write(P1,0);
					gpio_write(P1,1);
					ret = pwm_change_parameter(&pwm0);
					WaitMs(50);
				}					
			}
			break;
	}
	LOG("pwm test end:%d\n",testcase);
	while(1);
}