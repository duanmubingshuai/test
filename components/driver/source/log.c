
#include "rom_sym_def.h"
#include "types.h"
#include "uart.h"
#include "log.h"
#include "pwrmgr.h"
#include "jump_function.h"

#ifndef _LOG_PRINTF_SWU_
#define _LOG_PRINTF_SWU_ 1
#endif


extern void hal_UART0_IRQHandler(void);
void dbg_uart_irq_handler(void)
{
    hal_UART0_IRQHandler();
}
void uart_cbk(comm_evt_t* pev);

#if _LOG_PRINTF_SWU_
void dbg_printf_swu_init(void)
{
    swu_uart_init(115200,P4,P5,NULL);
}
#endif
void dbg_printf_init(void)
{
#if( _LOG_PRINTF_SWU_==0)
    uart_Cfg_t cfg =
    {
        .tx_pin = P4,
        .rx_pin = P5,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .fcr        = NULL,
        .evt_handler = NULL,
    };
    JUMP_FUNCTION_SET(UART0_IRQ_HANDLER,dbg_uart_irq_handler);
    hal_uart_init(cfg);//uart init
#else
    dbg_printf_swu_init();
    pwrmgr_register(MOD_USR8, NULL, dbg_printf_swu_init);
#endif
}

void my_dump_byte(uint8_t* pData, int dlen)
{
    for(int i=0; i<dlen; i++)
    {
        log_printf("%02x ",pData[i]);
    }

    log_printf("\n");
}



