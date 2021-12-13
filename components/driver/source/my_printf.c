
#include "rom_sym_def.h"
#include "jump_function.h"
#include "types.h"
#include <stdarg.h>
#include <string.h>
#include "uart.h"
#include "log.h"

static void _uart_putc(char* data, uint16_t size)
{
    hal_uart_send_buff(UART0, (uint8_t*)data, size);
}

void dbg_printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    log_vsprintf(_uart_putc, format, args);
    va_end(args);
}

void dbg_UART0_IRQHandler(void)
{
    hal_UART0_IRQHandler();
}

void dbg_printf_init(void)
{
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
        .evt_handler = NULL,
    };
    JUMP_FUNCTION_SET(UART0_IRQ_HANDLER,(uint32_t)&dbg_UART0_IRQHandler);
    hal_uart_init(cfg, UART0);//uart init
}

void my_dump_byte(uint8_t* pData, int dlen)
{
    for(int i=0; i<dlen; i++)
    {
        dbg_printf("%02x ",pData[i]);
    }

    dbg_printf("\n");
}


