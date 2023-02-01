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
    Filename:       test_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "test_demo.h"
#include "log.h"
#include "gpio.h"
#include "clock.h"
#include "pwrmgr.h"


static volatile uint8_t test_demo_TaskID;
#define DEMO_1000MS_CYCLE           1000
#define DEMO_1000MS_EVENT           0x0001
#define TEST_EVENT_FOR_UART         0x0002

extern void gpio_test(void);
extern void uart_test(void);
extern void i2c_test(void);
extern void pwm_test(void);

#if (TEST_CASE == UART_TEST)
#define DBG(...)  dbg_printf(__VA_ARGS__)

#define UART_RX_BUFFER_LEN 100
uint8_t  __attribute__((weak)) uart_rx_buffer[UART_RX_BUFFER_LEN];
uint8_t  __attribute__((weak)) uart_rx_len;

void uartrx_timeout_timer_start(void)
{
    osal_start_timerEx(test_demo_TaskID, TEST_EVENT_FOR_UART, 10);
}

void uartrx_timeout_timer_stop(void)
{
    osal_stop_timerEx(test_demo_TaskID, TEST_EVENT_FOR_UART);
}

void uarttx_timeout_timer_start(void)
{
    osal_start_reload_timer(test_demo_TaskID, TEST_EVENT_FOR_UART, 1000);
}
extern int uart_get_testcase(void);
#endif

void Test_Demo_Init( uint8 task_id )
{
    uint8_t testcase = TEST_CASE;
    test_demo_TaskID = task_id;
    LOG("\n\n\ntest demo start:\n");

    switch(testcase)
    {
    case GPIO_TEST:
        LOG("gpio test\n");
        gpio_test();
        break;

    case UART_TEST:
        LOG("uart test\n");
        uart_test();
        break;

    case I2C_TEST:
        LOG("i2c test\n");
        i2c_test();
        break;

    case PWM_TEST:
        LOG("pwm test\n");
        pwm_test();

    default:
        break;
    }

    osal_start_reload_timer(test_demo_TaskID, DEMO_1000MS_EVENT, DEMO_1000MS_CYCLE);
}

//extern uint32_t wakeup_counter;
uint16 Test_Demo_ProcessEvent( uint8 task_id, uint16 events )
{
    static uint8_t counter = 0;

    if(events & DEMO_1000MS_EVENT)
    {
        #if (TEST_CASE == GPIO_TEST)
        gpio_test();
        #endif
        #if (TEST_CASE == I2C_TEST)
        i2c_test();
        #endif
//      LOG("wakeup_counter:%d\n",wakeup_counter);
        LOG("counter:%d\n",counter++);
        return (events ^ DEMO_1000MS_EVENT);
    }

    if(events & TEST_EVENT_FOR_UART)
    {
        #if (TEST_CASE == UART_TEST)

        if(uart_rx_len != 0)
        {
            if((uart_get_testcase()%2) == 0)
            {
                hal_uart_send_buff(UART0,uart_rx_buffer,uart_rx_len);
            }
            else
            {
                hal_uart_send_buff(UART1,uart_rx_buffer,uart_rx_len);
            }

            uart_rx_len = 0;
        }
        else
        {
            uint8_t test_ptr[] = {"ÄãºÃ123abc\n"};

            if((uart_get_testcase()%2) == 0)
            {
                hal_uart_send_buff(UART0,test_ptr,sizeof(test_ptr));
            }
            else
            {
                hal_uart_send_buff(UART1,test_ptr,sizeof(test_ptr));
            }
        }

        #endif
        return (events ^ TEST_EVENT_FOR_UART);
    }

    return 0;
}
