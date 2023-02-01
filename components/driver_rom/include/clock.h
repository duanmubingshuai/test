#ifndef __CLOCK_ROM_H__
#define __CLOCK_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"
#include "bus_dev.h"
#define RTC_OVERFLOW_VAL        (0x000fffff) //20bit rtc counter
#define TIME_BASE               (0x003fffff) // 24bit count shift 2 bit as 1us/bit
#define TIME_DELTA(x,y)         ( (x>=y) ? x-y : TIME_BASE-y+x )

#define XTAL_TRACKING_REG                 (0x4000F188)
#define DLL_TRACKING_REG                  (0x4000F18c)

#define CLK_TRACKING_DLL                  (0)
#define CLK_TRACKING_XTAL                 (1)

#define CLK_TRACKING_MOD_IRQ              (0)
#define CLK_TRACKING_MOD_POLLING          (1)


#define _CLK_TRACKING_ENABLE(a,x)         subWriteReg(a, 0, 0, x)
#define _CLK_TRACKING_IRQ_STATUS(a)       (0x1 & (read_reg(a)>>2))
#define _CLK_TRACKING_SETTLE(a)           (0x1 & (read_reg(a)>>1))
#define _CLK_TRACKING_IRQ_CLR(a)          {subWriteReg(a, 25, 25, 1);subWriteReg(a, 25, 25, 0);}
#define _CLK_TRACKING_IRQ_MASK(a,x)       subWriteReg(a, 24, 24, x)

#define _CLK_TRACKING_CNT_TARGET(a,x)     subWriteReg(a, 23, 12, 0x0fff&(x))
#define _CLK_TRACKING_THD_SETTLE(a,x)     subWriteReg(a, 11,  4, 0x00ff&(x))
#define _CLK_TRACKING_SET_CONFIG(a,m,c)   subWriteReg(a, 24,  4, ((m)<<20)|(0xfffff&(c)))

#define AON_RC32M_DIV_SET(x)              subWriteReg(0x4000f0bc, 10, 8, 0x07&(x))
#define AON_RC32M_DIV_GET                 (0x07&(read_reg(0x4000f0bc)>>8))

enum  LOWCLK_SEL{
    RC_32K,
    XTAL_32K
};

typedef enum CLK_RC32M_DIV{
    DIV0_TO_32M=0,
    DIV1_TO_16M,
    DIV2_TO_8M,
    DIV3_TO_4M,
    DIV4_TO_2M,
    DIV5_TO_1M
}clk_rc32m_div_t;

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
//bool rtc_config_prescale(uint32_t pre);
uint32_t rtc_get_counter(void);

void WaitRTCCount(uint32_t rtcDelyCnt);
void WaitUs(uint32_t wtTime);
void WaitMs(uint32_t msecond);
void wait_hclk_cycle_us(uint32_t dlyT);

int clk_spif_ref_clk(sysclk_t spif_ref_clk);
int clk_init(sysclk_t h_system_clk_sel);
void clk_set_pclk_div(uint8_t div);
uint32_t clk_get_pclk(void);
int clk_set_rc32M_div(clk_rc32m_div_t div);
void clk_gate_enable(MODULE_e module);
void clk_gate_disable(MODULE_e module);
int clk_gate_get(MODULE_e module);
void clk_reset(MODULE_e module);
void _clk_dll_enable(void);

uint32_t get_systick(void);
uint32_t get_ms_intv(uint32_t tick);
void system_soft_reset(void);
void XTAL_Tracking_IRQHandler(void);
void DLL_Tracking_IRQHandler(void);
void check_clk_settle(uint8_t mod );
void clk_tracking_init(uint8_t mod ,uint32_t config,uint8 irqMask);
void _clk_apply_setting(uint32_t hclk_sel, uint32_t digi_clk_en, uint32_t digi32_sel,uint8_t wfi_sel);

#ifdef __cplusplus
}
#endif

#endif

