
#ifndef _TWO_STAGE_APP_H
#define _TWO_STAGE_APP_H

#include "bus_dev.h"
#include "otp.h"

#define OTP_2STAGE_NUM_MAX        5 //max is 5, 1st boot at least 0x20, total 0x70

#define OTP_2STAGE_PREAMBLE       "OAP2"
#define P_OTP_2STAGE_PREAMBLE(n)  ((volatile uint8_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR + (n+1)*0x10)) 
#define OTP_2STAGE_ENTRY(n)       (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR + (n+1)*0x10 + 4))
#define OTP_2STAGE_SIZE(n)        (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR + (n+1)*0x10 + 8))
#define OTP_2STAGE_CRC(n)         (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR + (n+1)*0x10 + 12))

#define UARTRUN_2S_TAG  0x35db
#define UARTRUN_2S_ENTRY(addr) ((addr &0xffff)|(UARTRUN_2S_TAG<<16))

void two_stage_app_uartrun(uint32_t uartrun_entry);
void two_stage_app_load(void);
void two_stage_print(void);
#endif

