#ifndef _PICO_REG_H_
#define _PICO_REG_H_

#define _PICO_REG_WR(addr, data) (*(volatile unsigned int *)(addr)=(unsigned int)(data))
#define _PICO_REG_RD(addr)       (*(volatile unsigned int *)(addr))


#include "pico_reg_aes.h"

#include "pico_reg_aon.h"

#include "pico_reg_bb_top.h"

#include "pico_reg_cache.h"

#include "pico_reg_com.h"

#include "pico_reg_cp_timer.h"

#include "pico_reg_gpio.h"

#include "pico_reg_i2c0.h"

#include "pico_reg_iomux.h"

#include "pico_reg_ll.h"

#include "pico_reg_otp.h"

#include "pico_reg_pcr.h"

#include "pico_reg_pcrm.h"

#include "pico_reg_pwm.h"

#include "pico_reg_spi0.h"

#include "pico_reg_uart.h"

#endif // _PICO_REG_H_

