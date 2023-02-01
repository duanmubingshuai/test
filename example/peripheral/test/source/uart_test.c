#include "test_demo.h"

#if (TEST_CASE == UART_TEST)
#include "rom_sym_def.h"
#include "types.h"
#include <stdarg.h>
#include <string.h>
#include "uart.h"
#include "log.h"

#define UART_RX_BUFFER_LEN 100
extern uint8_t uart_rx_buffer[UART_RX_BUFFER_LEN];
extern uint8_t uart_rx_len;

extern uint8_t uartrx_timeout_timer_start(void);
extern void uartrx_timeout_timer_stop(void);
extern void uarttx_timeout_timer_start(void);

#define DBG(...)  dbg_printf(__VA_ARGS__)

void uart_test_evt_hdl(uart_Evt_t* pev)
{
    switch(pev->type)
    {
    case  UART_EVT_TYPE_RX_DATA:
    case  UART_EVT_TYPE_RX_DATA_TO:
        if((uart_rx_len + pev->len)>=UART_RX_BUFFER_LEN)
            break;

        uartrx_timeout_timer_stop();
        uartrx_timeout_timer_start();
        memcpy(uart_rx_buffer + uart_rx_len, pev->data, pev->len);
        uart_rx_len += pev->len;
        break;

    case  UART_EVT_TYPE_TX_COMPLETED:
        //osal_set_event(bleuart_TaskID, BUP_OSAL_EVT_UART_TX_COMPLETE);
        break;

    default:
        break;
    }
}

static uint8_t testcase = 0;
void uart_test(void)
{
    uint8_t uart_0[]= {"[uart0]"};
    uint8_t uart_1[]= {"[uart1]"};
    uint8_t sleep_en[]= {"[with sleep]"};
    uint8_t sleep_dis[]= {"[no sleep]"};
    uart_Cfg_t cfg =
    {
        .tx_pin = P9,
        .rx_pin = P10,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = uart_test_evt_hdl,
    };

    switch(testcase)
    {
    case 0:
        hal_uart_init(cfg, UART0);
        hal_uart_send_buff(UART0,uart_0,sizeof(uart_0));
        hal_uart_send_buff(UART0,sleep_dis,sizeof(sleep_dis));
        memset(uart_rx_buffer,0x00,UART_RX_BUFFER_LEN);
        uart_rx_len = 0;
        break;

    case 1:
        hal_uart_init(cfg, UART1);
        hal_uart_send_buff(UART1,uart_1,sizeof(uart_1));
        hal_uart_send_buff(UART1,sleep_dis,sizeof(sleep_dis));
        memset(uart_rx_buffer,0x00,UART_RX_BUFFER_LEN);
        uart_rx_len = 0;
        break;

    case 2:
        cfg.evt_handler = NULL;
        hal_uart_init(cfg, UART0);
        hal_uart_send_buff(UART0,uart_0,sizeof(uart_0));
        hal_uart_send_buff(UART0,sleep_en,sizeof(sleep_en));
        uart_rx_len = 0;
        uarttx_timeout_timer_start();
        break;

    case 3:
        cfg.evt_handler = NULL;
        hal_uart_init(cfg, UART1);
        hal_uart_send_buff(UART1,uart_1,sizeof(uart_1));
        hal_uart_send_buff(UART1,sleep_en,sizeof(sleep_en));
        uart_rx_len = 0;
        uarttx_timeout_timer_start();
        break;

    default:
        break;
    }
}

int uart_get_testcase(void)
{
    return testcase;
}
#endif
