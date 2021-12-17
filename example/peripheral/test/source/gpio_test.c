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
#include "test_demo.h"

#if (TEST_CASE == GPIO_TEST)
#include "error.h"
#include "mcu.h"
#include "clock.h"
#include "gpio.h"
#include "uart.h"

#include "bus_dev.h"
#include "log.h"
#include "gpio.h"
#include "jump_function.h"

static void gpio_in_out(gpio_pin_e j,gpio_pin_e k)
{
    static volatile int i,r;
    static volatile uint8_t data_wr[2]= {0,1};
    LOG("\n");

    for(i=0; i<2; i++)
    {
        hal_gpio_write(j,data_wr[i]);//gpio_write(j,data_wr[i]);
        hal_gpio_pin_init(k,GPIO_INPUT);//gpio_dir_r(k,GPIO_INPUT);
        {
            r = hal_gpio_read(k);//r = gpio_read(k);

            if(r!=data_wr[i])
            {
                LOG("error[line:%d] [output pin:%d output value:%d] [input pin:%d input value:%d]\n",j,data_wr[i],k,r);

                while(1);
            }
            else
            {
                LOG("ok  [output pin:%d output value:%d] [input pin:%d input value:%d]\n",j,data_wr[i],k,r);
            }
        }
    }

    //goto L1;
    for(i=0; i<2; i++)
    {
        hal_gpio_write(k,data_wr[i]);//gpio_write(k,data_wr[i]);
        hal_gpio_pin_init(j,GPIO_INPUT);//gpio_dir_r(j,GPIO_INPUT);
        {
            r = hal_gpio_read(j);//r = gpio_read(j);

            if(r!=data_wr[i])
            {
                LOG("error[line:%d] [output pin:%d output value:%d] [input pin:%d input value:%d]\n",k,data_wr[i],j,r);

                while(1);
            }
            else
            {
                LOG("ok  [output pin:%d output value:%d] [input pin:%d input value:%d]\n",k,data_wr[i],j,r);
            }
        }
    }
}




int int_flag = 0;
int counter = 0;
int p0p1_int_en = 0;//0:enable 1:disable 2:set
void pos_cb(gpio_pin_e pin,gpio_polarity_e type)
{
    LOG("pin:%d int(pos)\n",pin);

    if(pin == P7)
    {
        p0p1_int_en = 0;
    }
}

void neg_cb(gpio_pin_e pin,gpio_polarity_e type)
{
    LOG("pin:%d int(neg)\n",pin);

    if(pin == P7)
    {
        p0p1_int_en = 1;
    }
}

void gpio_int_test(gpio_pin_e pin)
{
    NVIC_SetPriority((IRQn_Type)GPIO_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ((IRQn_Type)GPIO_IRQn);

    if((pin == P16) || (pin == P17))
    {
        *(volatile int*)0x40003800 = 0;
    }

    while(1)
    {
        int_flag = 0;
        hal_gpioin_register(pin,pos_cb,neg_cb);//gpioin_register(pin,pos_cb,neg_cb);

        while(int_flag == 0);

        hal_gpioin_unregister(pin);//gpioin_unregister(pin);
    }
}

void gpio_wakeup_off_test(uint8_t mode,gpio_pin_e pin)
{
    //uint32_t counter=0;
    gpio_polarity_e pol;
    bool gpio_rd;

    if((pin == P16)||(pin == P17))
    {
        *(volatile int*)0x40003800 = 0;
    }

    //gpio_dir_r(pin,GPIO_INPUT);
    hal_gpio_pin_init(pin,GPIO_INPUT);
    //gpio_rd = gpio_read((gpio_pin_e)pin);
    gpio_rd = hal_gpio_read((gpio_pin_e)pin);
    LOG("pin and cur state:%d %d\n",pin,gpio_rd);
    pol = gpio_rd ? POL_FALLING : POL_RISING;

    if(POL_FALLING == pol)
    {
        LOG("[FALLING]");
    }
    else
    {
        LOG("[RISING]");
    }

    //gpio_wakeup_set(pin,pol);
    hal_gpio_wakeup_set(pin,pol);

    if(mode == 0)
    {
        *(volatile int*)0x4000F0A8 |= (1<<0);
        LOG("->sleep:\n\n\n");
        AP_AON->PWRSLP = 0xA5A55A5A;
    }
    else
    {
        *(volatile int*)0x4000F0A8 |= (1<<1);
        LOG("->off:\n\n\n");
        AP_AON->PWROFF = 0x5A5AA5A5;
    }

    LOG("error2_____\n");

    while(1)
    {
        int_flag++;
    }
}

void gpio_pull_test(gpio_pin_e pin,gpio_pupd_e type)
{
    hal_gpio_pull_set(pin,type);
    hal_gpio_pin_init(pin,GPIO_INPUT);
}

void gpio_test(void)
{
    uint8_t testcase = 4;

    switch(testcase)
    {
    case 0://gpio in output test
        gpio_in_out(P2,P1);
        break;

    case 1://gpio int test
        gpio_int_test(P1);
        break;

    case 2://gpio sleep and wakeup test
        gpio_wakeup_off_test(0,P1);
        break;

    case 3://gpio off and wakeup test
        gpio_wakeup_off_test(1,P1);
        break;

    case 4:
        gpio_pull_test(P1,GPIO_PULL_UP_S);
        LOG("GPIO_PULL_UP_S\n");
//          gpio_pull_test(P1,GPIO_PULL_UP);
//          LOG("GPIO_PULL_UP\n");
//          gpio_pull_test(P1,GPIO_PULL_DOWN);
//          LOG("GPIO_PULL_DOWN\n");
        WaitMs(60000);
        break;

    case 5:
        //gpio rentation
        break;

    case 6:
        //gpio fullmux test,different pin
        break;

    case 7:
        //gpio fullmux test,different function
        break;

    default:
        break;
    }
}
#endif
