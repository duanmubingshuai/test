#ifndef __PHY_LOG_ROM_H__
#define __PHY_LOG_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"
#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

#define LOG_LEVEL_NONE  0 //no log output*/
#define LOG_LEVEL_ERROR 1 //only log error*/
#define LOG_LEVEL_DEBUG 2 //output debug info and error info*/
#define LOG_LEVEL_LOG   3 //output all infomation*/

#define LOG_INIT()                  {swu_uart_init(115200,P9,P10,NULL);}

#if 0//DEBUG_FPGA
#define LOG(...) do{;}while(0); 
#else

//conditional output
#define LOG(...)                   log_printf(__VA_ARGS__)//{if(s_rom_debug_level == LOG_LEVEL_LOG) log_printf(__VA_ARGS__);}
#define LOG_DEBUG(...)             // {if(s_rom_debug_level >= LOG_LEVEL_DEBUG) log_printf(__VA_ARGS__);}
#define LOG_ERROR(...)             log_printf(__VA_ARGS__)// {if(s_rom_debug_level >= LOG_LEVEL_ERROR) log_printf(__VA_ARGS__);}

//tx data anyway
#define PRINT(...)                  {SWU_TX(); log_printf(__VA_ARGS__);}

#endif

extern volatile uint32_t s_rom_debug_level;

typedef void(*std_putc)(char* data, int size);

void log_vsprintf(std_putc putc, const char *fmt, va_list args);
void log_printf(const char *format, ...);
void log_set_putc(std_putc putc);
void log_clr_putc(std_putc putc);
int log_debug_level(uint8_t level);
uint32_t log_get_debug_level(void);

#ifdef __cplusplus
}
#endif


#endif
