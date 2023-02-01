#ifndef __PHY_LOG_ROM_H__
#define __PHY_LOG_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"
#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

#ifndef _DEF_DEBUG_INFO_
#define _DEF_DEBUG_INFO_ 3
#endif


#define LOG_LEVEL_NONE  0 //no log output*/ only for rom PRINT
#define LOG_LEVEL_ERROR 1 //only log error*/
#define LOG_LEVEL_DEBUG 2 //output debug info and error info*/
#define LOG_LEVEL_LOG   3 //output all infomation*/


#if(_DEF_DEBUG_INFO_==LOG_LEVEL_NONE)
#define LOG_INIT()                  //{swu_uart_init(115200,P4,P5,NULL);}
#define LOG(...)                    //{if(s_rom_debug_level == LOG_LEVEL_LOG) log_printf(__VA_ARGS__);}
#define LOG_DEBUG(...)              // {if(s_rom_debug_level >= LOG_LEVEL_DEBUG) log_printf(__VA_ARGS__);}
#define LOG_ERROR(...)              // {if(s_rom_debug_level >= LOG_LEVEL_ERROR) log_printf(__VA_ARGS__);}
#define LOG_DUMP_BYTE(a,b)
//tx data anyway
#define PRINT(...)                  {SWU_TX(); log_printf(__VA_ARGS__);}

#elif(_DEF_DEBUG_INFO_==LOG_LEVEL_ERROR)

#define LOG_INIT()                  {dbg_printf_init();}
#define LOG(...)                    //{log_printf(__VA_ARGS__);}
#define LOG_DEBUG(...)              //{log_printf("[DBG]");log_printf(__VA_ARGS__);}
#define LOG_ERROR(...)              {log_printf("[ERR]");log_printf(__VA_ARGS__);}
#define LOG_DUMP_BYTE(a,b)          my_dump_byte(a,b)
//tx data anyway
#define PRINT(...)                  {log_printf(__VA_ARGS__);}

#elif(_DEF_DEBUG_INFO_==LOG_LEVEL_DEBUG)
#define LOG_INIT()                  {dbg_printf_init();}
#define LOG(...)                    //{log_printf(__VA_ARGS__);}
#define LOG_DEBUG(...)              {log_printf("[DBG]");log_printf(__VA_ARGS__);}
#define LOG_ERROR(...)              //{log_printf("[ERR]");log_printf(__VA_ARGS__);}
#define LOG_DUMP_BYTE(a,b)          my_dump_byte(a,b)
//tx data anyway
#define PRINT(...)                  {log_printf(__VA_ARGS__);}

#elif(_DEF_DEBUG_INFO_==LOG_LEVEL_LOG)

#define LOG_INIT()                  {dbg_printf_init();}
#define LOG(...)                    {log_printf(__VA_ARGS__);}
#define LOG_DEBUG(...)              {log_printf("[DBG]");log_printf(__VA_ARGS__);}
#define LOG_ERROR(...)              {log_printf("[ERR]");log_printf(__VA_ARGS__);}
#define LOG_DUMP_BYTE(a,b)          my_dump_byte(a,b)
//tx data anyway
#define PRINT(...)                  {log_printf(__VA_ARGS__);}
#endif

extern volatile uint32_t s_rom_debug_level;

typedef void(*std_putc)(char* data, int size);

void log_vsprintf(std_putc putc, const char *fmt, va_list args);
void log_printf(const char *format, ...);
void log_set_putc(std_putc putc);
void log_clr_putc(std_putc putc);
void my_dump_byte(uint8_t* pData, int dlen);
void dbg_printf_init(void);
#ifdef __cplusplus
}
#endif


#endif
