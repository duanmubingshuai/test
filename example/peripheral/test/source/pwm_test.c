#include "test_demo.h"

#if (TEST_CASE == PWM_TEST)
#include "OSAL.h"
#include "gpio.h"
#include "clock.h"
#include "pwm.h"
#include "log.h"

void gpio_debug_init(gpio_pin_e pin)
{
    hal_gpio_pin_init(pin,GPIO_OUTPUT);
    hal_gpio_fast_write(pin,0);
}

void gpio_debug_toggle(gpio_pin_e pin,uint8_t n)
{
    for(int i=0; i<n; i++)
    {
        hal_gpio_fast_write(pin,1);
        hal_gpio_fast_write(pin,0);
    }
}

void pwm_debug_print_reg(void)
{
    LOG("AP_PWM->pwmen:0x%x\n",AP_PWM->pwmen);

    for(int n=0; n<6; n++)
    {
        LOG("channel:%d ctrl0:0x%x ctrl1:0x%x\n",n,AP_PWM_CTRL(n)->ctrl0,AP_PWM_CTRL(n)->ctrl1);
    }
}

/*
    pwm channel:0~5
    pwm clk_div:1 2 4 8...128

    up mode(16/N/(TOP_VAL+1))
    PWM_POLARITY_FALLING:start is high
    ((CMP_VAL+1)/(TOP_VAL+1))
    one cycle:-|_

    PWM_POLARITY_RISING:start is low
    1-((CMP_VAL+1)/(TOP_VAL+1))

    up and down mode(8/N/TOP_VAL)
    PWM_POLARITY_FALLING:start is high
    (CMP_VAL/TOP_VAL)
    one cycle:-|__|-

    PWM_POLARITY_RISING:start is low
    1-(CMP_VAL/TOP_VAL)

*/

#define PWM_TOP_VALUE      61538//250

#define PWM_DEBUG_PIN P24
void pwm_test(void)
{
    uint8_t testcase = 0;
    bool up_flag = FALSE;
    pwm_ch_t pwm_ch[3];
    uint32_t y = 0,duty=0;
    uint32_t cmp_value,top_value;
    gpio_debug_init(PWM_DEBUG_PIN);

    switch(testcase)
    {
    /*
        basic test
    */
    case 0:
        hal_pwm_module_init();
        pwm_ch[0].pwmN = PWM_CH0;
        pwm_ch[0].pwmPin = P23;
        pwm_ch[0].pwmDiv = PWM_CLK_NO_DIV;
        pwm_ch[0].pwmMode = PWM_CNT_UP_AND_DOWN;//PWM_CNT_UP;
        pwm_ch[0].pwmPolarity = PWM_POLARITY_FALLING;//PWM_POLARITY_FALLING;//PWM_POLARITY_RISING;
        pwm_ch[0].cmpVal = 18000;
        pwm_ch[0].cntTopVal = 20000;
        hal_pwm_ch_start(pwm_ch[0]);
        gpio_debug_toggle(PWM_DEBUG_PIN,1);
        WaitMs(100);
        gpio_debug_toggle(PWM_DEBUG_PIN,1);
        hal_pwm_ch_stop(pwm_ch[0]);
        hal_pwm_module_deinit();

        while(1);

        break;

    /*
        cmp=0  1~98 99
        top=99
        duty=0 2%~99% 100%
        duty=(cmp+1)/(top+1)
    */
    case 1://up:[0%,100%]
        cmp_value = 0;
        top_value = 99;
        hal_pwm_module_init();
        pwm_ch[0].pwmN = PWM_CH0;
        pwm_ch[0].pwmPin = P23;
        pwm_ch[0].pwmDiv = PWM_CLK_NO_DIV;
        pwm_ch[0].pwmMode = PWM_CNT_UP;
        pwm_ch[0].pwmPolarity = PWM_POLARITY_FALLING;
        pwm_ch[0].cmpVal = top_value;
        pwm_ch[0].cntTopVal = top_value;
        up_flag = FALSE;
        hal_pwm_ch_start(pwm_ch[0]);

        while(1)
        {
            WaitMs(5);

            if(up_flag == FALSE)
            {
                if(pwm_ch[0].cmpVal > cmp_value)
                {
                    pwm_ch[0].cmpVal--;
                }

                hal_pwm_ch_start(pwm_ch[0]);

                if(pwm_ch[0].cmpVal == cmp_value)
                {
                    up_flag = TRUE;
                }
            }
            else
            {
                if(pwm_ch[0].cmpVal < top_value)
                {
                    pwm_ch[0].cmpVal++;
                }

                hal_pwm_ch_start(pwm_ch[0]);

                if(pwm_ch[0].cmpVal == top_value)
                {
                    up_flag = FALSE;
                }
            }

            gpio_debug_toggle(PWM_DEBUG_PIN,1);
        }

        break;

    /*
        cmp=1~99
        top=100
        duty=1%~99%
        duty=cmp/top
    */
    case 2://up and dowm:[1%,99%]
        cmp_value = 1;
        top_value = 100;
        hal_pwm_module_init();
        pwm_ch[0].pwmN = PWM_CH0;
        pwm_ch[0].pwmPin = P23;
        pwm_ch[0].pwmDiv = PWM_CLK_NO_DIV;
        pwm_ch[0].pwmMode = PWM_CNT_UP_AND_DOWN;
        pwm_ch[0].pwmPolarity = PWM_POLARITY_FALLING;
        pwm_ch[0].cmpVal = top_value-1;
        pwm_ch[0].cntTopVal = top_value;
        up_flag = FALSE;
        hal_pwm_ch_start(pwm_ch[0]);

        while(1)
        {
            WaitMs(5);

            if(up_flag == FALSE)
            {
                if(pwm_ch[0].cmpVal > 1)
                {
                    pwm_ch[0].cmpVal--;
                }

                hal_pwm_ch_start(pwm_ch[0]);

                if(pwm_ch[0].cmpVal == 1)
                {
                    up_flag = TRUE;
                }
            }
            else
            {
                if(pwm_ch[0].cmpVal < (top_value-1))
                {
                    pwm_ch[0].cmpVal++;
                }

                hal_pwm_ch_start(pwm_ch[0]);

                if(pwm_ch[0].cmpVal == (top_value-1))
                {
                    up_flag = FALSE;
                }
            }

            gpio_debug_toggle(PWM_DEBUG_PIN,1);
        }

        break;

    /*
        top:[1,65535]
        tmp:[0,1~top-1,top]
        (1+1)/(65535+1)
    */
    case 3:
        //cmp_value = 0;
        //top_value = 1;// 16/2=8Mhz duty=0,100
        top_value = 2;// 16/3=5.33Mhz duty=0 2/3 100
        //top_value = 65535;
        hal_pwm_module_init();
        pwm_ch[0].pwmN = PWM_CH0;
        pwm_ch[0].pwmPin = P23;
        pwm_ch[0].pwmDiv = PWM_CLK_NO_DIV;
        pwm_ch[0].pwmMode = PWM_CNT_UP;
        pwm_ch[0].pwmPolarity = PWM_POLARITY_FALLING;
        pwm_ch[0].cmpVal = 1;//top_value;
        pwm_ch[0].cntTopVal = top_value;
        up_flag = FALSE;
        hal_pwm_ch_start(pwm_ch[0]);

        //pwm_debug_print_reg();
        while(1);;

    /*
        top:[2,65535]
        tmp:[1,top-1]
        1/65535
    */
    case 4:
        top_value = 2;
        //top_value = 65535;
        hal_pwm_module_init();
        pwm_ch[0].pwmN = PWM_CH0;
        pwm_ch[0].pwmPin = P23;
        pwm_ch[0].pwmDiv = PWM_CLK_NO_DIV;
        pwm_ch[0].pwmMode = PWM_CNT_UP_AND_DOWN;
        pwm_ch[0].pwmPolarity = PWM_POLARITY_FALLING;
        pwm_ch[0].cmpVal = 1;//top_value;
        pwm_ch[0].cntTopVal = top_value;
        up_flag = FALSE;
        hal_pwm_ch_start(pwm_ch[0]);

        //pwm_debug_print_reg();
        while(1);;

        break;

    /*
        customer support for wanhu
    */
    case 13:
        hal_pwm_module_init();
        pwm_ch[0].pwmN = PWM_CH0;
        pwm_ch[0].pwmPin = P23;
        pwm_ch[0].pwmDiv = PWM_CLK_NO_DIV;
        pwm_ch[0].pwmMode = PWM_CNT_UP;
        pwm_ch[0].pwmPolarity = PWM_POLARITY_RISING;
        pwm_ch[0].cmpVal = duty;
        pwm_ch[0].cntTopVal = PWM_TOP_VALUE;
        pwm_ch[1].pwmN = PWM_CH1;
        pwm_ch[1].pwmPin = P24;
        pwm_ch[1].pwmDiv = PWM_CLK_NO_DIV;
        pwm_ch[1].pwmMode = PWM_CNT_UP;
        pwm_ch[1].pwmPolarity = PWM_POLARITY_RISING;
        pwm_ch[1].cmpVal = duty;
        pwm_ch[1].cntTopVal = PWM_TOP_VALUE;

        while(1)
        {
            y++;

            if(y <= 50)
            {
                duty=y*615;
                pwm_ch[0].cmpVal = duty;
                pwm_ch[1].cmpVal = duty;
                hal_pwm_ch_start(pwm_ch[0]);
                WaitUs(1900);
                hal_pwm_ch_start(pwm_ch[1]);
                LOG("y==%d\tduty==%d\t\n",y,duty);
            }
            else
            {
                y=0;
                LOG("PWM stop11111\n");
            }

            WaitMs(10);
        }

        break;

    default:
        break;
    }
}

#endif
