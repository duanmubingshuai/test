#ifndef __CLOCK_ROM_H__
#define __CLOCK_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"
#include "bus_dev.h"

#define TIME_BASE               (0x003fffff) // 24bit count shift 2 bit as 1us/bit
#define TIME_DELTA(x,y)         ( (x>=y) ? x-y : TIME_BASE-y+x )

enum  LOWCLK_SEL{
    RC_32K,
    XTAL_32K
};

typedef enum{
    NO_AP_NO_CP = 0,HCLK = 0,
    PCLK = 1,
} pclk_Type_t;


typedef enum  _SYSCLK_SEL {
    SYS_CLK_RC_32M      = 0,
    SYS_CLK_DLL_32M     = 1,
    SYS_CLK_XTAL_16M    = 2,
    SYS_CLK_DLL_48M     = 3,
    SYS_CLK_DLL_64M     = 4,
    SYS_CLK_DLL_96M     = 5,
    SYS_CLK_8M          = 6,
    SYS_CLK_4M          = 7,
    SYS_CLK_NUM         = 8,
    SYS_CLK_NONE        = 0xff,
}sysclk_t;

extern volatile uint32_t  g_pclk,g_hclk;

void rtc_start(void);
void rtc_stop(void);
void rtc_clear(void);
bool rtc_config_prescale(uint32_t pre);
uint32_t rtc_get_counter(void);

void WaitRTCCount(uint32_t rtcDelyCnt);
void WaitUs(uint32_t wtTime);
void WaitMs(uint32_t msecond);

int clk_spif_ref_clk(sysclk_t spif_ref_clk);
int clk_init(sysclk_t h_system_clk_sel);
void clk_set_pclk_div(uint8_t div);
uint32_t clk_get_pclk(void);

void clk_gate_enable(MODULE_e module);
void clk_gate_disable(MODULE_e module);
int clk_gate_get(MODULE_e module);
void clk_reset(MODULE_e module);


uint32_t get_systick(void);
uint32_t get_ms_intv(uint32_t tick);
void system_soft_reset(void);

#ifdef __cplusplus
}
#endif

#endif

