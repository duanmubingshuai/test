#ifndef _PICO_REG_PWM_H_
#define _PICO_REG_PWM_H_

#include <stdint.h>

#define PWM_COUNT 13

#define PWM_BASE_ADDR 0x4000E000

#define PWM_SIZE 0x00000044


 /**
 * @brief PWMEN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     17          pwm_load_45   0
 *     16            pwm_en_45   0
 *     15          pwm_load_23   0
 *     14            pwm_en_23   0
 *     13          pwm_load_01   0
 *     12            pwm_en_01   0
 *     11         pwm_load_345   0
 *     10           pwm_en_345   0
 *     09         pwm_load_012   0
 *     08           pwm_en_012   0
 *     04         pwm_load_all   0
 *     00           pwm_en_all   0
 * </pre>
 */
#define PWM_PWMEN_OFFSET 0x00000000


__INLINE uint32_t pwm_pwmen_get(void)
{
    return _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwmen_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWMEN_PWM_LOAD_45_BIT                       ((uint32_t)0x00020000)
#define PWM_PWMEN_PWM_LOAD_45_POS                       17
#define PWM_PWMEN_PWM_EN_45_BIT                         ((uint32_t)0x00010000)
#define PWM_PWMEN_PWM_EN_45_POS                         16
#define PWM_PWMEN_PWM_LOAD_23_BIT                       ((uint32_t)0x00008000)
#define PWM_PWMEN_PWM_LOAD_23_POS                       15
#define PWM_PWMEN_PWM_EN_23_BIT                         ((uint32_t)0x00004000)
#define PWM_PWMEN_PWM_EN_23_POS                         14
#define PWM_PWMEN_PWM_LOAD_01_BIT                       ((uint32_t)0x00002000)
#define PWM_PWMEN_PWM_LOAD_01_POS                       13
#define PWM_PWMEN_PWM_EN_01_BIT                         ((uint32_t)0x00001000)
#define PWM_PWMEN_PWM_EN_01_POS                         12
#define PWM_PWMEN_PWM_LOAD_345_BIT                      ((uint32_t)0x00000800)
#define PWM_PWMEN_PWM_LOAD_345_POS                      11
#define PWM_PWMEN_PWM_EN_345_BIT                        ((uint32_t)0x00000400)
#define PWM_PWMEN_PWM_EN_345_POS                        10
#define PWM_PWMEN_PWM_LOAD_012_BIT                      ((uint32_t)0x00000200)
#define PWM_PWMEN_PWM_LOAD_012_POS                      9
#define PWM_PWMEN_PWM_EN_012_BIT                        ((uint32_t)0x00000100)
#define PWM_PWMEN_PWM_EN_012_POS                        8
#define PWM_PWMEN_PWM_LOAD_ALL_BIT                      ((uint32_t)0x00000010)
#define PWM_PWMEN_PWM_LOAD_ALL_POS                      4
#define PWM_PWMEN_PWM_EN_ALL_BIT                        ((uint32_t)0x00000001)
#define PWM_PWMEN_PWM_EN_ALL_POS                        0

#define PWM_PWMEN_PWM_LOAD_45_RST                       0x0
#define PWM_PWMEN_PWM_EN_45_RST                         0x0
#define PWM_PWMEN_PWM_LOAD_23_RST                       0x0
#define PWM_PWMEN_PWM_EN_23_RST                         0x0
#define PWM_PWMEN_PWM_LOAD_01_RST                       0x0
#define PWM_PWMEN_PWM_EN_01_RST                         0x0
#define PWM_PWMEN_PWM_LOAD_345_RST                      0x0
#define PWM_PWMEN_PWM_EN_345_RST                        0x0
#define PWM_PWMEN_PWM_LOAD_012_RST                      0x0
#define PWM_PWMEN_PWM_EN_012_RST                        0x0
#define PWM_PWMEN_PWM_LOAD_ALL_RST                      0x0
#define PWM_PWMEN_PWM_EN_ALL_RST                        0x0

__INLINE void pwm_pwmen_pack(uint8_t pwmload45, uint8_t pwmen45, uint8_t pwmload23, uint8_t pwmen23, uint8_t pwmload01, uint8_t pwmen01, uint8_t pwmload345, uint8_t pwmen345, uint8_t pwmload012, uint8_t pwmen012, uint8_t pwmloadall, uint8_t pwmenall)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwmload45 << 17) | ((uint32_t)pwmen45 << 16) | ((uint32_t)pwmload23 << 15) | ((uint32_t)pwmen23 << 14) | ((uint32_t)pwmload01 << 13) | ((uint32_t)pwmen01 << 12) | ((uint32_t)pwmload345 << 11) | ((uint32_t)pwmen345 << 10) | ((uint32_t)pwmload012 << 9) | ((uint32_t)pwmen012 << 8) | ((uint32_t)pwmloadall << 4) | ((uint32_t)pwmenall << 0));
}

__INLINE void pwm_pwmen_unpack(uint8_t* pwmload45, uint8_t* pwmen45, uint8_t* pwmload23, uint8_t* pwmen23, uint8_t* pwmload01, uint8_t* pwmen01, uint8_t* pwmload345, uint8_t* pwmen345, uint8_t* pwmload012, uint8_t* pwmen012, uint8_t* pwmloadall, uint8_t* pwmenall)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);

    *pwmload45 = (localVal & ((uint32_t)0x00020000)) >> 17;
    *pwmen45 = (localVal & ((uint32_t)0x00010000)) >> 16;
    *pwmload23 = (localVal & ((uint32_t)0x00008000)) >> 15;
    *pwmen23 = (localVal & ((uint32_t)0x00004000)) >> 14;
    *pwmload01 = (localVal & ((uint32_t)0x00002000)) >> 13;
    *pwmen01 = (localVal & ((uint32_t)0x00001000)) >> 12;
    *pwmload345 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *pwmen345 = (localVal & ((uint32_t)0x00000400)) >> 10;
    *pwmload012 = (localVal & ((uint32_t)0x00000200)) >> 9;
    *pwmen012 = (localVal & ((uint32_t)0x00000100)) >> 8;
    *pwmloadall = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pwmenall = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pwm_pwmen_pwm_load_45_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void pwm_pwmen_pwm_load_45_setf(uint8_t pwmload45)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)pwmload45 << 17));
}

__INLINE uint8_t pwm_pwmen_pwm_en_45_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pwm_pwmen_pwm_en_45_setf(uint8_t pwmen45)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pwmen45 << 16));
}

__INLINE uint8_t pwm_pwmen_pwm_load_23_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void pwm_pwmen_pwm_load_23_setf(uint8_t pwmload23)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)pwmload23 << 15));
}

__INLINE uint8_t pwm_pwmen_pwm_en_23_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void pwm_pwmen_pwm_en_23_setf(uint8_t pwmen23)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)pwmen23 << 14));
}

__INLINE uint8_t pwm_pwmen_pwm_load_01_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void pwm_pwmen_pwm_load_01_setf(uint8_t pwmload01)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)pwmload01 << 13));
}

__INLINE uint8_t pwm_pwmen_pwm_en_01_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void pwm_pwmen_pwm_en_01_setf(uint8_t pwmen01)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)pwmen01 << 12));
}

__INLINE uint8_t pwm_pwmen_pwm_load_345_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void pwm_pwmen_pwm_load_345_setf(uint8_t pwmload345)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)pwmload345 << 11));
}

__INLINE uint8_t pwm_pwmen_pwm_en_345_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void pwm_pwmen_pwm_en_345_setf(uint8_t pwmen345)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)pwmen345 << 10));
}

__INLINE uint8_t pwm_pwmen_pwm_load_012_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void pwm_pwmen_pwm_load_012_setf(uint8_t pwmload012)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)pwmload012 << 9));
}

__INLINE uint8_t pwm_pwmen_pwm_en_012_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pwm_pwmen_pwm_en_012_setf(uint8_t pwmen012)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pwmen012 << 8));
}

__INLINE uint8_t pwm_pwmen_pwm_load_all_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pwm_pwmen_pwm_load_all_setf(uint8_t pwmloadall)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)pwmloadall << 4));
}

__INLINE uint8_t pwm_pwmen_pwm_en_all_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pwm_pwmen_pwm_en_all_setf(uint8_t pwmenall)
{
    _PICO_REG_WR(PWM_PWMEN_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWMEN_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pwmenall << 0));
}

 /**
 * @brief PWM0CTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    pwm0_load_instant   0
 *     16            pwm0_load   0
 *  14:12         pwm0_clk_div   0b0
 *     08        pwm0_cnt_mode   0
 *     04        pwm0_polarity   0
 *     00              pwm0_en   0
 * </pre>
 */
#define PWM_PWM0CTL0_OFFSET 0x00000004


__INLINE uint32_t pwm_pwm0ctl0_get(void)
{
    return _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm0ctl0_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM0CTL0_PWM0_LOAD_INSTANT_BIT                 ((uint32_t)0x80000000)
#define PWM_PWM0CTL0_PWM0_LOAD_INSTANT_POS                 31
#define PWM_PWM0CTL0_PWM0_LOAD_BIT                         ((uint32_t)0x00010000)
#define PWM_PWM0CTL0_PWM0_LOAD_POS                         16
#define PWM_PWM0CTL0_PWM0_CLK_DIV_MASK                     ((uint32_t)0x00007000)
#define PWM_PWM0CTL0_PWM0_CLK_DIV_LSB                      12
#define PWM_PWM0CTL0_PWM0_CLK_DIV_WIDTH                    ((uint32_t)0x00000003)
#define PWM_PWM0CTL0_PWM0_CNT_MODE_BIT                     ((uint32_t)0x00000100)
#define PWM_PWM0CTL0_PWM0_CNT_MODE_POS                     8
#define PWM_PWM0CTL0_PWM0_POLARITY_BIT                     ((uint32_t)0x00000010)
#define PWM_PWM0CTL0_PWM0_POLARITY_POS                     4
#define PWM_PWM0CTL0_PWM0_EN_BIT                           ((uint32_t)0x00000001)
#define PWM_PWM0CTL0_PWM0_EN_POS                           0

#define PWM_PWM0CTL0_PWM0_LOAD_INSTANT_RST                 0x0
#define PWM_PWM0CTL0_PWM0_LOAD_RST                         0x0
#define PWM_PWM0CTL0_PWM0_CLK_DIV_RST                      0x0
#define PWM_PWM0CTL0_PWM0_CNT_MODE_RST                     0x0
#define PWM_PWM0CTL0_PWM0_POLARITY_RST                     0x0
#define PWM_PWM0CTL0_PWM0_EN_RST                           0x0

__INLINE void pwm_pwm0ctl0_pack(uint8_t pwm0loadinstant, uint8_t pwm0load, uint8_t pwm0clkdiv, uint8_t pwm0cntmode, uint8_t pwm0polarity, uint8_t pwm0en)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm0loadinstant << 31) | ((uint32_t)pwm0load << 16) | ((uint32_t)pwm0clkdiv << 12) | ((uint32_t)pwm0cntmode << 8) | ((uint32_t)pwm0polarity << 4) | ((uint32_t)pwm0en << 0));
}

__INLINE void pwm_pwm0ctl0_unpack(uint8_t* pwm0loadinstant, uint8_t* pwm0load, uint8_t* pwm0clkdiv, uint8_t* pwm0cntmode, uint8_t* pwm0polarity, uint8_t* pwm0en)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);

    *pwm0loadinstant = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pwm0load = (localVal & ((uint32_t)0x00010000)) >> 16;
    *pwm0clkdiv = (localVal & ((uint32_t)0x00007000)) >> 12;
    *pwm0cntmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *pwm0polarity = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pwm0en = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pwm_pwm0ctl0_pwm0_load_instant_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pwm_pwm0ctl0_pwm0_load_instant_setf(uint8_t pwm0loadinstant)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)pwm0loadinstant << 31));
}

__INLINE uint8_t pwm_pwm0ctl0_pwm0_load_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pwm_pwm0ctl0_pwm0_load_setf(uint8_t pwm0load)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pwm0load << 16));
}

__INLINE uint8_t pwm_pwm0ctl0_pwm0_clk_div_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void pwm_pwm0ctl0_pwm0_clk_div_setf(uint8_t pwm0clkdiv)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)pwm0clkdiv << 12));
}

__INLINE uint8_t pwm_pwm0ctl0_pwm0_cnt_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pwm_pwm0ctl0_pwm0_cnt_mode_setf(uint8_t pwm0cntmode)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pwm0cntmode << 8));
}

__INLINE uint8_t pwm_pwm0ctl0_pwm0_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pwm_pwm0ctl0_pwm0_polarity_setf(uint8_t pwm0polarity)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)pwm0polarity << 4));
}

__INLINE uint8_t pwm_pwm0ctl0_pwm0_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pwm_pwm0ctl0_pwm0_en_setf(uint8_t pwm0en)
{
    _PICO_REG_WR(PWM_PWM0CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pwm0en << 0));
}

 /**
 * @brief PWM0CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16         pwm0_cmp_val   0b0
 *  15:00         pwm0_cnt_top   0b0
 * </pre>
 */
#define PWM_PWM0CTL1_OFFSET 0x00000008


__INLINE uint32_t pwm_pwm0ctl1_get(void)
{
    return _PICO_REG_RD(PWM_PWM0CTL1_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm0ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM0CTL1_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM0CTL1_PWM0_CMP_VAL_MASK                     ((uint32_t)0xFFFF0000)
#define PWM_PWM0CTL1_PWM0_CMP_VAL_LSB                      16
#define PWM_PWM0CTL1_PWM0_CMP_VAL_WIDTH                    ((uint32_t)0x00000010)
#define PWM_PWM0CTL1_PWM0_CNT_TOP_MASK                     ((uint32_t)0x0000FFFF)
#define PWM_PWM0CTL1_PWM0_CNT_TOP_LSB                      0
#define PWM_PWM0CTL1_PWM0_CNT_TOP_WIDTH                    ((uint32_t)0x00000010)

#define PWM_PWM0CTL1_PWM0_CMP_VAL_RST                      0x0
#define PWM_PWM0CTL1_PWM0_CNT_TOP_RST                      0x0

__INLINE void pwm_pwm0ctl1_pack(uint16_t pwm0cmpval, uint16_t pwm0cnttop)
{
    _PICO_REG_WR(PWM_PWM0CTL1_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm0cmpval << 16) | ((uint32_t)pwm0cnttop << 0));
}

__INLINE void pwm_pwm0ctl1_unpack(uint8_t* pwm0cmpval, uint8_t* pwm0cnttop)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL1_OFFSET + PWM_BASE_ADDR);

    *pwm0cmpval = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *pwm0cnttop = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t pwm_pwm0ctl1_pwm0_cmp_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void pwm_pwm0ctl1_pwm0_cmp_val_setf(uint16_t pwm0cmpval)
{
    _PICO_REG_WR(PWM_PWM0CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)pwm0cmpval << 16));
}

__INLINE uint16_t pwm_pwm0ctl1_pwm0_cnt_top_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM0CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void pwm_pwm0ctl1_pwm0_cnt_top_setf(uint16_t pwm0cnttop)
{
    _PICO_REG_WR(PWM_PWM0CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM0CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)pwm0cnttop << 0));
}

 /**
 * @brief PWM1CTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    pwm1_load_instant   0
 *     16            pwm1_load   0
 *  14:12         pwm1_clk_div   0b0
 *     08        pwm1_cnt_mode   0
 *     04        pwm1_polarity   0
 *     00              pwm1_en   0
 * </pre>
 */
#define PWM_PWM1CTL0_OFFSET 0x00000010


__INLINE uint32_t pwm_pwm1ctl0_get(void)
{
    return _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm1ctl0_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM1CTL0_PWM1_LOAD_INSTANT_BIT                 ((uint32_t)0x80000000)
#define PWM_PWM1CTL0_PWM1_LOAD_INSTANT_POS                 31
#define PWM_PWM1CTL0_PWM1_LOAD_BIT                         ((uint32_t)0x00010000)
#define PWM_PWM1CTL0_PWM1_LOAD_POS                         16
#define PWM_PWM1CTL0_PWM1_CLK_DIV_MASK                     ((uint32_t)0x00007000)
#define PWM_PWM1CTL0_PWM1_CLK_DIV_LSB                      12
#define PWM_PWM1CTL0_PWM1_CLK_DIV_WIDTH                    ((uint32_t)0x00000003)
#define PWM_PWM1CTL0_PWM1_CNT_MODE_BIT                     ((uint32_t)0x00000100)
#define PWM_PWM1CTL0_PWM1_CNT_MODE_POS                     8
#define PWM_PWM1CTL0_PWM1_POLARITY_BIT                     ((uint32_t)0x00000010)
#define PWM_PWM1CTL0_PWM1_POLARITY_POS                     4
#define PWM_PWM1CTL0_PWM1_EN_BIT                           ((uint32_t)0x00000001)
#define PWM_PWM1CTL0_PWM1_EN_POS                           0

#define PWM_PWM1CTL0_PWM1_LOAD_INSTANT_RST                 0x0
#define PWM_PWM1CTL0_PWM1_LOAD_RST                         0x0
#define PWM_PWM1CTL0_PWM1_CLK_DIV_RST                      0x0
#define PWM_PWM1CTL0_PWM1_CNT_MODE_RST                     0x0
#define PWM_PWM1CTL0_PWM1_POLARITY_RST                     0x0
#define PWM_PWM1CTL0_PWM1_EN_RST                           0x0

__INLINE void pwm_pwm1ctl0_pack(uint8_t pwm1loadinstant, uint8_t pwm1load, uint8_t pwm1clkdiv, uint8_t pwm1cntmode, uint8_t pwm1polarity, uint8_t pwm1en)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm1loadinstant << 31) | ((uint32_t)pwm1load << 16) | ((uint32_t)pwm1clkdiv << 12) | ((uint32_t)pwm1cntmode << 8) | ((uint32_t)pwm1polarity << 4) | ((uint32_t)pwm1en << 0));
}

__INLINE void pwm_pwm1ctl0_unpack(uint8_t* pwm1loadinstant, uint8_t* pwm1load, uint8_t* pwm1clkdiv, uint8_t* pwm1cntmode, uint8_t* pwm1polarity, uint8_t* pwm1en)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);

    *pwm1loadinstant = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pwm1load = (localVal & ((uint32_t)0x00010000)) >> 16;
    *pwm1clkdiv = (localVal & ((uint32_t)0x00007000)) >> 12;
    *pwm1cntmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *pwm1polarity = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pwm1en = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pwm_pwm1ctl0_pwm1_load_instant_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pwm_pwm1ctl0_pwm1_load_instant_setf(uint8_t pwm1loadinstant)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)pwm1loadinstant << 31));
}

__INLINE uint8_t pwm_pwm1ctl0_pwm1_load_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pwm_pwm1ctl0_pwm1_load_setf(uint8_t pwm1load)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pwm1load << 16));
}

__INLINE uint8_t pwm_pwm1ctl0_pwm1_clk_div_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void pwm_pwm1ctl0_pwm1_clk_div_setf(uint8_t pwm1clkdiv)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)pwm1clkdiv << 12));
}

__INLINE uint8_t pwm_pwm1ctl0_pwm1_cnt_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pwm_pwm1ctl0_pwm1_cnt_mode_setf(uint8_t pwm1cntmode)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pwm1cntmode << 8));
}

__INLINE uint8_t pwm_pwm1ctl0_pwm1_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pwm_pwm1ctl0_pwm1_polarity_setf(uint8_t pwm1polarity)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)pwm1polarity << 4));
}

__INLINE uint8_t pwm_pwm1ctl0_pwm1_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pwm_pwm1ctl0_pwm1_en_setf(uint8_t pwm1en)
{
    _PICO_REG_WR(PWM_PWM1CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pwm1en << 0));
}

 /**
 * @brief PWM1CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16         pwm1_cmp_val   0b0
 *  15:00         pwm1_cnt_top   0b0
 * </pre>
 */
#define PWM_PWM1CTL1_OFFSET 0x00000014


__INLINE uint32_t pwm_pwm1ctl1_get(void)
{
    return _PICO_REG_RD(PWM_PWM1CTL1_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm1ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM1CTL1_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM1CTL1_PWM1_CMP_VAL_MASK                     ((uint32_t)0xFFFF0000)
#define PWM_PWM1CTL1_PWM1_CMP_VAL_LSB                      16
#define PWM_PWM1CTL1_PWM1_CMP_VAL_WIDTH                    ((uint32_t)0x00000010)
#define PWM_PWM1CTL1_PWM1_CNT_TOP_MASK                     ((uint32_t)0x0000FFFF)
#define PWM_PWM1CTL1_PWM1_CNT_TOP_LSB                      0
#define PWM_PWM1CTL1_PWM1_CNT_TOP_WIDTH                    ((uint32_t)0x00000010)

#define PWM_PWM1CTL1_PWM1_CMP_VAL_RST                      0x0
#define PWM_PWM1CTL1_PWM1_CNT_TOP_RST                      0x0

__INLINE void pwm_pwm1ctl1_pack(uint16_t pwm1cmpval, uint16_t pwm1cnttop)
{
    _PICO_REG_WR(PWM_PWM1CTL1_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm1cmpval << 16) | ((uint32_t)pwm1cnttop << 0));
}

__INLINE void pwm_pwm1ctl1_unpack(uint8_t* pwm1cmpval, uint8_t* pwm1cnttop)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL1_OFFSET + PWM_BASE_ADDR);

    *pwm1cmpval = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *pwm1cnttop = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t pwm_pwm1ctl1_pwm1_cmp_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void pwm_pwm1ctl1_pwm1_cmp_val_setf(uint16_t pwm1cmpval)
{
    _PICO_REG_WR(PWM_PWM1CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)pwm1cmpval << 16));
}

__INLINE uint16_t pwm_pwm1ctl1_pwm1_cnt_top_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM1CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void pwm_pwm1ctl1_pwm1_cnt_top_setf(uint16_t pwm1cnttop)
{
    _PICO_REG_WR(PWM_PWM1CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM1CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)pwm1cnttop << 0));
}

 /**
 * @brief PWM2CTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    pwm2_load_instant   0
 *     16            pwm2_load   0
 *  14:12         pwm2_clk_div   0b0
 *     08        pwm2_cnt_mode   0
 *     04        pwm2_polarity   0
 *     00              pwm2_en   0
 * </pre>
 */
#define PWM_PWM2CTL0_OFFSET 0x0000001C


__INLINE uint32_t pwm_pwm2ctl0_get(void)
{
    return _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm2ctl0_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM2CTL0_PWM2_LOAD_INSTANT_BIT                 ((uint32_t)0x80000000)
#define PWM_PWM2CTL0_PWM2_LOAD_INSTANT_POS                 31
#define PWM_PWM2CTL0_PWM2_LOAD_BIT                         ((uint32_t)0x00010000)
#define PWM_PWM2CTL0_PWM2_LOAD_POS                         16
#define PWM_PWM2CTL0_PWM2_CLK_DIV_MASK                     ((uint32_t)0x00007000)
#define PWM_PWM2CTL0_PWM2_CLK_DIV_LSB                      12
#define PWM_PWM2CTL0_PWM2_CLK_DIV_WIDTH                    ((uint32_t)0x00000003)
#define PWM_PWM2CTL0_PWM2_CNT_MODE_BIT                     ((uint32_t)0x00000100)
#define PWM_PWM2CTL0_PWM2_CNT_MODE_POS                     8
#define PWM_PWM2CTL0_PWM2_POLARITY_BIT                     ((uint32_t)0x00000010)
#define PWM_PWM2CTL0_PWM2_POLARITY_POS                     4
#define PWM_PWM2CTL0_PWM2_EN_BIT                           ((uint32_t)0x00000001)
#define PWM_PWM2CTL0_PWM2_EN_POS                           0

#define PWM_PWM2CTL0_PWM2_LOAD_INSTANT_RST                 0x0
#define PWM_PWM2CTL0_PWM2_LOAD_RST                         0x0
#define PWM_PWM2CTL0_PWM2_CLK_DIV_RST                      0x0
#define PWM_PWM2CTL0_PWM2_CNT_MODE_RST                     0x0
#define PWM_PWM2CTL0_PWM2_POLARITY_RST                     0x0
#define PWM_PWM2CTL0_PWM2_EN_RST                           0x0

__INLINE void pwm_pwm2ctl0_pack(uint8_t pwm2loadinstant, uint8_t pwm2load, uint8_t pwm2clkdiv, uint8_t pwm2cntmode, uint8_t pwm2polarity, uint8_t pwm2en)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm2loadinstant << 31) | ((uint32_t)pwm2load << 16) | ((uint32_t)pwm2clkdiv << 12) | ((uint32_t)pwm2cntmode << 8) | ((uint32_t)pwm2polarity << 4) | ((uint32_t)pwm2en << 0));
}

__INLINE void pwm_pwm2ctl0_unpack(uint8_t* pwm2loadinstant, uint8_t* pwm2load, uint8_t* pwm2clkdiv, uint8_t* pwm2cntmode, uint8_t* pwm2polarity, uint8_t* pwm2en)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);

    *pwm2loadinstant = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pwm2load = (localVal & ((uint32_t)0x00010000)) >> 16;
    *pwm2clkdiv = (localVal & ((uint32_t)0x00007000)) >> 12;
    *pwm2cntmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *pwm2polarity = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pwm2en = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pwm_pwm2ctl0_pwm2_load_instant_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pwm_pwm2ctl0_pwm2_load_instant_setf(uint8_t pwm2loadinstant)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)pwm2loadinstant << 31));
}

__INLINE uint8_t pwm_pwm2ctl0_pwm2_load_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pwm_pwm2ctl0_pwm2_load_setf(uint8_t pwm2load)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pwm2load << 16));
}

__INLINE uint8_t pwm_pwm2ctl0_pwm2_clk_div_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void pwm_pwm2ctl0_pwm2_clk_div_setf(uint8_t pwm2clkdiv)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)pwm2clkdiv << 12));
}

__INLINE uint8_t pwm_pwm2ctl0_pwm2_cnt_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pwm_pwm2ctl0_pwm2_cnt_mode_setf(uint8_t pwm2cntmode)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pwm2cntmode << 8));
}

__INLINE uint8_t pwm_pwm2ctl0_pwm2_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pwm_pwm2ctl0_pwm2_polarity_setf(uint8_t pwm2polarity)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)pwm2polarity << 4));
}

__INLINE uint8_t pwm_pwm2ctl0_pwm2_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pwm_pwm2ctl0_pwm2_en_setf(uint8_t pwm2en)
{
    _PICO_REG_WR(PWM_PWM2CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pwm2en << 0));
}

 /**
 * @brief PWM2CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16         pwm2_cmp_val   0b0
 *  15:00         pwm2_cnt_top   0b0
 * </pre>
 */
#define PWM_PWM2CTL1_OFFSET 0x00000020


__INLINE uint32_t pwm_pwm2ctl1_get(void)
{
    return _PICO_REG_RD(PWM_PWM2CTL1_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm2ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM2CTL1_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM2CTL1_PWM2_CMP_VAL_MASK                     ((uint32_t)0xFFFF0000)
#define PWM_PWM2CTL1_PWM2_CMP_VAL_LSB                      16
#define PWM_PWM2CTL1_PWM2_CMP_VAL_WIDTH                    ((uint32_t)0x00000010)
#define PWM_PWM2CTL1_PWM2_CNT_TOP_MASK                     ((uint32_t)0x0000FFFF)
#define PWM_PWM2CTL1_PWM2_CNT_TOP_LSB                      0
#define PWM_PWM2CTL1_PWM2_CNT_TOP_WIDTH                    ((uint32_t)0x00000010)

#define PWM_PWM2CTL1_PWM2_CMP_VAL_RST                      0x0
#define PWM_PWM2CTL1_PWM2_CNT_TOP_RST                      0x0

__INLINE void pwm_pwm2ctl1_pack(uint16_t pwm2cmpval, uint16_t pwm2cnttop)
{
    _PICO_REG_WR(PWM_PWM2CTL1_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm2cmpval << 16) | ((uint32_t)pwm2cnttop << 0));
}

__INLINE void pwm_pwm2ctl1_unpack(uint8_t* pwm2cmpval, uint8_t* pwm2cnttop)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL1_OFFSET + PWM_BASE_ADDR);

    *pwm2cmpval = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *pwm2cnttop = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t pwm_pwm2ctl1_pwm2_cmp_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void pwm_pwm2ctl1_pwm2_cmp_val_setf(uint16_t pwm2cmpval)
{
    _PICO_REG_WR(PWM_PWM2CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)pwm2cmpval << 16));
}

__INLINE uint16_t pwm_pwm2ctl1_pwm2_cnt_top_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM2CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void pwm_pwm2ctl1_pwm2_cnt_top_setf(uint16_t pwm2cnttop)
{
    _PICO_REG_WR(PWM_PWM2CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM2CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)pwm2cnttop << 0));
}

 /**
 * @brief PWM3CTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    pwm3_load_instant   0
 *     16            pwm3_load   0
 *  14:12         pwm3_clk_div   0b0
 *     08        pwm3_cnt_mode   0
 *     04        pwm3_polarity   0
 *     00              pwm3_en   0
 * </pre>
 */
#define PWM_PWM3CTL0_OFFSET 0x00000028


__INLINE uint32_t pwm_pwm3ctl0_get(void)
{
    return _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm3ctl0_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM3CTL0_PWM3_LOAD_INSTANT_BIT                 ((uint32_t)0x80000000)
#define PWM_PWM3CTL0_PWM3_LOAD_INSTANT_POS                 31
#define PWM_PWM3CTL0_PWM3_LOAD_BIT                         ((uint32_t)0x00010000)
#define PWM_PWM3CTL0_PWM3_LOAD_POS                         16
#define PWM_PWM3CTL0_PWM3_CLK_DIV_MASK                     ((uint32_t)0x00007000)
#define PWM_PWM3CTL0_PWM3_CLK_DIV_LSB                      12
#define PWM_PWM3CTL0_PWM3_CLK_DIV_WIDTH                    ((uint32_t)0x00000003)
#define PWM_PWM3CTL0_PWM3_CNT_MODE_BIT                     ((uint32_t)0x00000100)
#define PWM_PWM3CTL0_PWM3_CNT_MODE_POS                     8
#define PWM_PWM3CTL0_PWM3_POLARITY_BIT                     ((uint32_t)0x00000010)
#define PWM_PWM3CTL0_PWM3_POLARITY_POS                     4
#define PWM_PWM3CTL0_PWM3_EN_BIT                           ((uint32_t)0x00000001)
#define PWM_PWM3CTL0_PWM3_EN_POS                           0

#define PWM_PWM3CTL0_PWM3_LOAD_INSTANT_RST                 0x0
#define PWM_PWM3CTL0_PWM3_LOAD_RST                         0x0
#define PWM_PWM3CTL0_PWM3_CLK_DIV_RST                      0x0
#define PWM_PWM3CTL0_PWM3_CNT_MODE_RST                     0x0
#define PWM_PWM3CTL0_PWM3_POLARITY_RST                     0x0
#define PWM_PWM3CTL0_PWM3_EN_RST                           0x0

__INLINE void pwm_pwm3ctl0_pack(uint8_t pwm3loadinstant, uint8_t pwm3load, uint8_t pwm3clkdiv, uint8_t pwm3cntmode, uint8_t pwm3polarity, uint8_t pwm3en)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm3loadinstant << 31) | ((uint32_t)pwm3load << 16) | ((uint32_t)pwm3clkdiv << 12) | ((uint32_t)pwm3cntmode << 8) | ((uint32_t)pwm3polarity << 4) | ((uint32_t)pwm3en << 0));
}

__INLINE void pwm_pwm3ctl0_unpack(uint8_t* pwm3loadinstant, uint8_t* pwm3load, uint8_t* pwm3clkdiv, uint8_t* pwm3cntmode, uint8_t* pwm3polarity, uint8_t* pwm3en)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);

    *pwm3loadinstant = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pwm3load = (localVal & ((uint32_t)0x00010000)) >> 16;
    *pwm3clkdiv = (localVal & ((uint32_t)0x00007000)) >> 12;
    *pwm3cntmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *pwm3polarity = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pwm3en = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pwm_pwm3ctl0_pwm3_load_instant_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pwm_pwm3ctl0_pwm3_load_instant_setf(uint8_t pwm3loadinstant)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)pwm3loadinstant << 31));
}

__INLINE uint8_t pwm_pwm3ctl0_pwm3_load_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pwm_pwm3ctl0_pwm3_load_setf(uint8_t pwm3load)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pwm3load << 16));
}

__INLINE uint8_t pwm_pwm3ctl0_pwm3_clk_div_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void pwm_pwm3ctl0_pwm3_clk_div_setf(uint8_t pwm3clkdiv)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)pwm3clkdiv << 12));
}

__INLINE uint8_t pwm_pwm3ctl0_pwm3_cnt_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pwm_pwm3ctl0_pwm3_cnt_mode_setf(uint8_t pwm3cntmode)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pwm3cntmode << 8));
}

__INLINE uint8_t pwm_pwm3ctl0_pwm3_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pwm_pwm3ctl0_pwm3_polarity_setf(uint8_t pwm3polarity)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)pwm3polarity << 4));
}

__INLINE uint8_t pwm_pwm3ctl0_pwm3_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pwm_pwm3ctl0_pwm3_en_setf(uint8_t pwm3en)
{
    _PICO_REG_WR(PWM_PWM3CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pwm3en << 0));
}

 /**
 * @brief PWM3CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16         pwm3_cmp_val   0b0
 *  15:00         pwm3_cnt_top   0b0
 * </pre>
 */
#define PWM_PWM3CTL1_OFFSET 0x0000002C


__INLINE uint32_t pwm_pwm3ctl1_get(void)
{
    return _PICO_REG_RD(PWM_PWM3CTL1_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm3ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM3CTL1_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM3CTL1_PWM3_CMP_VAL_MASK                     ((uint32_t)0xFFFF0000)
#define PWM_PWM3CTL1_PWM3_CMP_VAL_LSB                      16
#define PWM_PWM3CTL1_PWM3_CMP_VAL_WIDTH                    ((uint32_t)0x00000010)
#define PWM_PWM3CTL1_PWM3_CNT_TOP_MASK                     ((uint32_t)0x0000FFFF)
#define PWM_PWM3CTL1_PWM3_CNT_TOP_LSB                      0
#define PWM_PWM3CTL1_PWM3_CNT_TOP_WIDTH                    ((uint32_t)0x00000010)

#define PWM_PWM3CTL1_PWM3_CMP_VAL_RST                      0x0
#define PWM_PWM3CTL1_PWM3_CNT_TOP_RST                      0x0

__INLINE void pwm_pwm3ctl1_pack(uint16_t pwm3cmpval, uint16_t pwm3cnttop)
{
    _PICO_REG_WR(PWM_PWM3CTL1_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm3cmpval << 16) | ((uint32_t)pwm3cnttop << 0));
}

__INLINE void pwm_pwm3ctl1_unpack(uint8_t* pwm3cmpval, uint8_t* pwm3cnttop)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL1_OFFSET + PWM_BASE_ADDR);

    *pwm3cmpval = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *pwm3cnttop = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t pwm_pwm3ctl1_pwm3_cmp_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void pwm_pwm3ctl1_pwm3_cmp_val_setf(uint16_t pwm3cmpval)
{
    _PICO_REG_WR(PWM_PWM3CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)pwm3cmpval << 16));
}

__INLINE uint16_t pwm_pwm3ctl1_pwm3_cnt_top_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM3CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void pwm_pwm3ctl1_pwm3_cnt_top_setf(uint16_t pwm3cnttop)
{
    _PICO_REG_WR(PWM_PWM3CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM3CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)pwm3cnttop << 0));
}

 /**
 * @brief PWM4CTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    pwm4_load_instant   0
 *     16            pwm4_load   0
 *  14:12         pwm4_clk_div   0b0
 *     08        pwm4_cnt_mode   0
 *     04        pwm4_polarity   0
 *     00              pwm4_en   0
 * </pre>
 */
#define PWM_PWM4CTL0_OFFSET 0x00000034


__INLINE uint32_t pwm_pwm4ctl0_get(void)
{
    return _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm4ctl0_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM4CTL0_PWM4_LOAD_INSTANT_BIT                 ((uint32_t)0x80000000)
#define PWM_PWM4CTL0_PWM4_LOAD_INSTANT_POS                 31
#define PWM_PWM4CTL0_PWM4_LOAD_BIT                         ((uint32_t)0x00010000)
#define PWM_PWM4CTL0_PWM4_LOAD_POS                         16
#define PWM_PWM4CTL0_PWM4_CLK_DIV_MASK                     ((uint32_t)0x00007000)
#define PWM_PWM4CTL0_PWM4_CLK_DIV_LSB                      12
#define PWM_PWM4CTL0_PWM4_CLK_DIV_WIDTH                    ((uint32_t)0x00000003)
#define PWM_PWM4CTL0_PWM4_CNT_MODE_BIT                     ((uint32_t)0x00000100)
#define PWM_PWM4CTL0_PWM4_CNT_MODE_POS                     8
#define PWM_PWM4CTL0_PWM4_POLARITY_BIT                     ((uint32_t)0x00000010)
#define PWM_PWM4CTL0_PWM4_POLARITY_POS                     4
#define PWM_PWM4CTL0_PWM4_EN_BIT                           ((uint32_t)0x00000001)
#define PWM_PWM4CTL0_PWM4_EN_POS                           0

#define PWM_PWM4CTL0_PWM4_LOAD_INSTANT_RST                 0x0
#define PWM_PWM4CTL0_PWM4_LOAD_RST                         0x0
#define PWM_PWM4CTL0_PWM4_CLK_DIV_RST                      0x0
#define PWM_PWM4CTL0_PWM4_CNT_MODE_RST                     0x0
#define PWM_PWM4CTL0_PWM4_POLARITY_RST                     0x0
#define PWM_PWM4CTL0_PWM4_EN_RST                           0x0

__INLINE void pwm_pwm4ctl0_pack(uint8_t pwm4loadinstant, uint8_t pwm4load, uint8_t pwm4clkdiv, uint8_t pwm4cntmode, uint8_t pwm4polarity, uint8_t pwm4en)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm4loadinstant << 31) | ((uint32_t)pwm4load << 16) | ((uint32_t)pwm4clkdiv << 12) | ((uint32_t)pwm4cntmode << 8) | ((uint32_t)pwm4polarity << 4) | ((uint32_t)pwm4en << 0));
}

__INLINE void pwm_pwm4ctl0_unpack(uint8_t* pwm4loadinstant, uint8_t* pwm4load, uint8_t* pwm4clkdiv, uint8_t* pwm4cntmode, uint8_t* pwm4polarity, uint8_t* pwm4en)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);

    *pwm4loadinstant = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pwm4load = (localVal & ((uint32_t)0x00010000)) >> 16;
    *pwm4clkdiv = (localVal & ((uint32_t)0x00007000)) >> 12;
    *pwm4cntmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *pwm4polarity = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pwm4en = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pwm_pwm4ctl0_pwm4_load_instant_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pwm_pwm4ctl0_pwm4_load_instant_setf(uint8_t pwm4loadinstant)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)pwm4loadinstant << 31));
}

__INLINE uint8_t pwm_pwm4ctl0_pwm4_load_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pwm_pwm4ctl0_pwm4_load_setf(uint8_t pwm4load)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pwm4load << 16));
}

__INLINE uint8_t pwm_pwm4ctl0_pwm4_clk_div_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void pwm_pwm4ctl0_pwm4_clk_div_setf(uint8_t pwm4clkdiv)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)pwm4clkdiv << 12));
}

__INLINE uint8_t pwm_pwm4ctl0_pwm4_cnt_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pwm_pwm4ctl0_pwm4_cnt_mode_setf(uint8_t pwm4cntmode)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pwm4cntmode << 8));
}

__INLINE uint8_t pwm_pwm4ctl0_pwm4_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pwm_pwm4ctl0_pwm4_polarity_setf(uint8_t pwm4polarity)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)pwm4polarity << 4));
}

__INLINE uint8_t pwm_pwm4ctl0_pwm4_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pwm_pwm4ctl0_pwm4_en_setf(uint8_t pwm4en)
{
    _PICO_REG_WR(PWM_PWM4CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pwm4en << 0));
}

 /**
 * @brief PWM4CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16         pwm4_cmp_val   0b0
 *  15:00         pwm4_cnt_top   0b0
 * </pre>
 */
#define PWM_PWM4CTL1_OFFSET 0x00000038


__INLINE uint32_t pwm_pwm4ctl1_get(void)
{
    return _PICO_REG_RD(PWM_PWM4CTL1_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm4ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM4CTL1_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM4CTL1_PWM4_CMP_VAL_MASK                     ((uint32_t)0xFFFF0000)
#define PWM_PWM4CTL1_PWM4_CMP_VAL_LSB                      16
#define PWM_PWM4CTL1_PWM4_CMP_VAL_WIDTH                    ((uint32_t)0x00000010)
#define PWM_PWM4CTL1_PWM4_CNT_TOP_MASK                     ((uint32_t)0x0000FFFF)
#define PWM_PWM4CTL1_PWM4_CNT_TOP_LSB                      0
#define PWM_PWM4CTL1_PWM4_CNT_TOP_WIDTH                    ((uint32_t)0x00000010)

#define PWM_PWM4CTL1_PWM4_CMP_VAL_RST                      0x0
#define PWM_PWM4CTL1_PWM4_CNT_TOP_RST                      0x0

__INLINE void pwm_pwm4ctl1_pack(uint16_t pwm4cmpval, uint16_t pwm4cnttop)
{
    _PICO_REG_WR(PWM_PWM4CTL1_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm4cmpval << 16) | ((uint32_t)pwm4cnttop << 0));
}

__INLINE void pwm_pwm4ctl1_unpack(uint8_t* pwm4cmpval, uint8_t* pwm4cnttop)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL1_OFFSET + PWM_BASE_ADDR);

    *pwm4cmpval = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *pwm4cnttop = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t pwm_pwm4ctl1_pwm4_cmp_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void pwm_pwm4ctl1_pwm4_cmp_val_setf(uint16_t pwm4cmpval)
{
    _PICO_REG_WR(PWM_PWM4CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)pwm4cmpval << 16));
}

__INLINE uint16_t pwm_pwm4ctl1_pwm4_cnt_top_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM4CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void pwm_pwm4ctl1_pwm4_cnt_top_setf(uint16_t pwm4cnttop)
{
    _PICO_REG_WR(PWM_PWM4CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM4CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)pwm4cnttop << 0));
}

 /**
 * @brief PWM5CTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    pwm5_load_instant   0
 *     16            pwm5_load   0
 *  14:12         pwm5_clk_div   0b0
 *     08        pwm5_cnt_mode   0
 *     04        pwm5_polarity   0
 *     00              pwm5_en   0
 * </pre>
 */
#define PWM_PWM5CTL0_OFFSET 0x00000040


__INLINE uint32_t pwm_pwm5ctl0_get(void)
{
    return _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm5ctl0_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM5CTL0_PWM5_LOAD_INSTANT_BIT                 ((uint32_t)0x80000000)
#define PWM_PWM5CTL0_PWM5_LOAD_INSTANT_POS                 31
#define PWM_PWM5CTL0_PWM5_LOAD_BIT                         ((uint32_t)0x00010000)
#define PWM_PWM5CTL0_PWM5_LOAD_POS                         16
#define PWM_PWM5CTL0_PWM5_CLK_DIV_MASK                     ((uint32_t)0x00007000)
#define PWM_PWM5CTL0_PWM5_CLK_DIV_LSB                      12
#define PWM_PWM5CTL0_PWM5_CLK_DIV_WIDTH                    ((uint32_t)0x00000003)
#define PWM_PWM5CTL0_PWM5_CNT_MODE_BIT                     ((uint32_t)0x00000100)
#define PWM_PWM5CTL0_PWM5_CNT_MODE_POS                     8
#define PWM_PWM5CTL0_PWM5_POLARITY_BIT                     ((uint32_t)0x00000010)
#define PWM_PWM5CTL0_PWM5_POLARITY_POS                     4
#define PWM_PWM5CTL0_PWM5_EN_BIT                           ((uint32_t)0x00000001)
#define PWM_PWM5CTL0_PWM5_EN_POS                           0

#define PWM_PWM5CTL0_PWM5_LOAD_INSTANT_RST                 0x0
#define PWM_PWM5CTL0_PWM5_LOAD_RST                         0x0
#define PWM_PWM5CTL0_PWM5_CLK_DIV_RST                      0x0
#define PWM_PWM5CTL0_PWM5_CNT_MODE_RST                     0x0
#define PWM_PWM5CTL0_PWM5_POLARITY_RST                     0x0
#define PWM_PWM5CTL0_PWM5_EN_RST                           0x0

__INLINE void pwm_pwm5ctl0_pack(uint8_t pwm5loadinstant, uint8_t pwm5load, uint8_t pwm5clkdiv, uint8_t pwm5cntmode, uint8_t pwm5polarity, uint8_t pwm5en)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm5loadinstant << 31) | ((uint32_t)pwm5load << 16) | ((uint32_t)pwm5clkdiv << 12) | ((uint32_t)pwm5cntmode << 8) | ((uint32_t)pwm5polarity << 4) | ((uint32_t)pwm5en << 0));
}

__INLINE void pwm_pwm5ctl0_unpack(uint8_t* pwm5loadinstant, uint8_t* pwm5load, uint8_t* pwm5clkdiv, uint8_t* pwm5cntmode, uint8_t* pwm5polarity, uint8_t* pwm5en)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);

    *pwm5loadinstant = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pwm5load = (localVal & ((uint32_t)0x00010000)) >> 16;
    *pwm5clkdiv = (localVal & ((uint32_t)0x00007000)) >> 12;
    *pwm5cntmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *pwm5polarity = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pwm5en = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pwm_pwm5ctl0_pwm5_load_instant_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pwm_pwm5ctl0_pwm5_load_instant_setf(uint8_t pwm5loadinstant)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)pwm5loadinstant << 31));
}

__INLINE uint8_t pwm_pwm5ctl0_pwm5_load_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pwm_pwm5ctl0_pwm5_load_setf(uint8_t pwm5load)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pwm5load << 16));
}

__INLINE uint8_t pwm_pwm5ctl0_pwm5_clk_div_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007000)) >> 12);
}

__INLINE void pwm_pwm5ctl0_pwm5_clk_div_setf(uint8_t pwm5clkdiv)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00007000)) | ((uint32_t)pwm5clkdiv << 12));
}

__INLINE uint8_t pwm_pwm5ctl0_pwm5_cnt_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pwm_pwm5ctl0_pwm5_cnt_mode_setf(uint8_t pwm5cntmode)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pwm5cntmode << 8));
}

__INLINE uint8_t pwm_pwm5ctl0_pwm5_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pwm_pwm5ctl0_pwm5_polarity_setf(uint8_t pwm5polarity)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)pwm5polarity << 4));
}

__INLINE uint8_t pwm_pwm5ctl0_pwm5_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pwm_pwm5ctl0_pwm5_en_setf(uint8_t pwm5en)
{
    _PICO_REG_WR(PWM_PWM5CTL0_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL0_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pwm5en << 0));
}

 /**
 * @brief PWM5CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16         pwm5_cmp_val   0b0
 *  15:00         pwm5_cnt_top   0b0
 * </pre>
 */
#define PWM_PWM5CTL1_OFFSET 0x00000044


__INLINE uint32_t pwm_pwm5ctl1_get(void)
{
    return _PICO_REG_RD(PWM_PWM5CTL1_OFFSET + PWM_BASE_ADDR);
}

__INLINE void pwm_pwm5ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PWM_PWM5CTL1_OFFSET+ PWM_BASE_ADDR, value);
}

// field definitions
#define PWM_PWM5CTL1_PWM5_CMP_VAL_MASK                     ((uint32_t)0xFFFF0000)
#define PWM_PWM5CTL1_PWM5_CMP_VAL_LSB                      16
#define PWM_PWM5CTL1_PWM5_CMP_VAL_WIDTH                    ((uint32_t)0x00000010)
#define PWM_PWM5CTL1_PWM5_CNT_TOP_MASK                     ((uint32_t)0x0000FFFF)
#define PWM_PWM5CTL1_PWM5_CNT_TOP_LSB                      0
#define PWM_PWM5CTL1_PWM5_CNT_TOP_WIDTH                    ((uint32_t)0x00000010)

#define PWM_PWM5CTL1_PWM5_CMP_VAL_RST                      0x0
#define PWM_PWM5CTL1_PWM5_CNT_TOP_RST                      0x0

__INLINE void pwm_pwm5ctl1_pack(uint16_t pwm5cmpval, uint16_t pwm5cnttop)
{
    _PICO_REG_WR(PWM_PWM5CTL1_OFFSET+ PWM_BASE_ADDR,  ((uint32_t)pwm5cmpval << 16) | ((uint32_t)pwm5cnttop << 0));
}

__INLINE void pwm_pwm5ctl1_unpack(uint8_t* pwm5cmpval, uint8_t* pwm5cnttop)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL1_OFFSET + PWM_BASE_ADDR);

    *pwm5cmpval = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *pwm5cnttop = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t pwm_pwm5ctl1_pwm5_cmp_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void pwm_pwm5ctl1_pwm5_cmp_val_setf(uint16_t pwm5cmpval)
{
    _PICO_REG_WR(PWM_PWM5CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)pwm5cmpval << 16));
}

__INLINE uint16_t pwm_pwm5ctl1_pwm5_cnt_top_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PWM_PWM5CTL1_OFFSET + PWM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void pwm_pwm5ctl1_pwm5_cnt_top_setf(uint16_t pwm5cnttop)
{
    _PICO_REG_WR(PWM_PWM5CTL1_OFFSET+ PWM_BASE_ADDR, (_PICO_REG_RD(PWM_PWM5CTL1_OFFSET + PWM_BASE_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)pwm5cnttop << 0));
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :14;
      __IO uint32_t pwm_load_45:1;
      __IO uint32_t pwm_en_45:1;
      __IO uint32_t pwm_load_23:1;
      __IO uint32_t pwm_en_23:1;
      __IO uint32_t pwm_load_01:1;
      __IO uint32_t pwm_en_01:1;
      __IO uint32_t pwm_load_345:1;
      __IO uint32_t pwm_en_345:1;
      __IO uint32_t pwm_load_012:1;
      __IO uint32_t pwm_en_012:1;
      __IO uint32_t :3;
      __IO uint32_t pwm_load_all:1;
      __IO uint32_t :3;
      __IO uint32_t pwm_en_all:1;
    }PWMEN_fld;
    __IO uint32_t PWMEN;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t pwm0_load_instant:1;
      __IO uint32_t :14;
      __IO uint32_t pwm0_load:1;
      __IO uint32_t :1;
      __IO uint32_t pwm0_clk_div:3;
      __IO uint32_t :3;
      __IO uint32_t pwm0_cnt_mode:1;
      __IO uint32_t :3;
      __IO uint32_t pwm0_polarity:1;
      __IO uint32_t :3;
      __IO uint32_t pwm0_en:1;
    }PWM0CTL0_fld;
    __IO uint32_t PWM0CTL0;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t pwm0_cmp_val:16;
      __IO uint32_t pwm0_cnt_top:16;
    }PWM0CTL1_fld;
    __IO uint32_t PWM0CTL1;
  };


  union{ //offset addr 0x0010
    struct{
      __IO uint32_t pwm1_load_instant:1;
      __IO uint32_t :14;
      __IO uint32_t pwm1_load:1;
      __IO uint32_t :1;
      __IO uint32_t pwm1_clk_div:3;
      __IO uint32_t :3;
      __IO uint32_t pwm1_cnt_mode:1;
      __IO uint32_t :3;
      __IO uint32_t pwm1_polarity:1;
      __IO uint32_t :3;
      __IO uint32_t pwm1_en:1;
    }PWM1CTL0_fld;
    __IO uint32_t PWM1CTL0;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t pwm1_cmp_val:16;
      __IO uint32_t pwm1_cnt_top:16;
    }PWM1CTL1_fld;
    __IO uint32_t PWM1CTL1;
  };


  union{ //offset addr 0x001c
    struct{
      __IO uint32_t pwm2_load_instant:1;
      __IO uint32_t :14;
      __IO uint32_t pwm2_load:1;
      __IO uint32_t :1;
      __IO uint32_t pwm2_clk_div:3;
      __IO uint32_t :3;
      __IO uint32_t pwm2_cnt_mode:1;
      __IO uint32_t :3;
      __IO uint32_t pwm2_polarity:1;
      __IO uint32_t :3;
      __IO uint32_t pwm2_en:1;
    }PWM2CTL0_fld;
    __IO uint32_t PWM2CTL0;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t pwm2_cmp_val:16;
      __IO uint32_t pwm2_cnt_top:16;
    }PWM2CTL1_fld;
    __IO uint32_t PWM2CTL1;
  };


  union{ //offset addr 0x0028
    struct{
      __IO uint32_t pwm3_load_instant:1;
      __IO uint32_t :14;
      __IO uint32_t pwm3_load:1;
      __IO uint32_t :1;
      __IO uint32_t pwm3_clk_div:3;
      __IO uint32_t :3;
      __IO uint32_t pwm3_cnt_mode:1;
      __IO uint32_t :3;
      __IO uint32_t pwm3_polarity:1;
      __IO uint32_t :3;
      __IO uint32_t pwm3_en:1;
    }PWM3CTL0_fld;
    __IO uint32_t PWM3CTL0;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t pwm3_cmp_val:16;
      __IO uint32_t pwm3_cnt_top:16;
    }PWM3CTL1_fld;
    __IO uint32_t PWM3CTL1;
  };


  union{ //offset addr 0x0034
    struct{
      __IO uint32_t pwm4_load_instant:1;
      __IO uint32_t :14;
      __IO uint32_t pwm4_load:1;
      __IO uint32_t :1;
      __IO uint32_t pwm4_clk_div:3;
      __IO uint32_t :3;
      __IO uint32_t pwm4_cnt_mode:1;
      __IO uint32_t :3;
      __IO uint32_t pwm4_polarity:1;
      __IO uint32_t :3;
      __IO uint32_t pwm4_en:1;
    }PWM4CTL0_fld;
    __IO uint32_t PWM4CTL0;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t pwm4_cmp_val:16;
      __IO uint32_t pwm4_cnt_top:16;
    }PWM4CTL1_fld;
    __IO uint32_t PWM4CTL1;
  };


  union{ //offset addr 0x0040
    struct{
      __IO uint32_t pwm5_load_instant:1;
      __IO uint32_t :14;
      __IO uint32_t pwm5_load:1;
      __IO uint32_t :1;
      __IO uint32_t pwm5_clk_div:3;
      __IO uint32_t :3;
      __IO uint32_t pwm5_cnt_mode:1;
      __IO uint32_t :3;
      __IO uint32_t pwm5_polarity:1;
      __IO uint32_t :3;
      __IO uint32_t pwm5_en:1;
    }PWM5CTL0_fld;
    __IO uint32_t PWM5CTL0;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t pwm5_cmp_val:16;
      __IO uint32_t pwm5_cnt_top:16;
    }PWM5CTL1_fld;
    __IO uint32_t PWM5CTL1;
  };

} PICO_REG_PWM_TypeDef;

#define PICO_REG_PWM PICO_REG_PWM_TypeDef *0x4000E000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_PWM_H_

