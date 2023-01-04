#ifndef _PICO_REG_OTP_H_
#define _PICO_REG_OTP_H_

#include <stdint.h>

#define OTP_COUNT 9

#define OTP_BASE_ADDR 0x1FFFC000

#define OTP_SIZE 0x00000020


 /**
 * @brief MODE_SETTING register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     08            soft_mode   0
 *  07:04            test_mode   0b0
 *     03     progream_mode_en   0
 *     02   deep_sleep_mode_en   0
 *     01              read_en   0
 *     00             power_on   0
 * </pre>
 */
#define OTP_MODE_SETTING_OFFSET 0x00000000


__INLINE uint32_t otp_mode_setting_get(void)
{
    return _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_mode_setting_set(uint32_t value)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_MODE_SETTING_SOFT_MODE_BIT                         ((uint32_t)0x00000100)
#define OTP_MODE_SETTING_SOFT_MODE_POS                         8
#define OTP_MODE_SETTING_TEST_MODE_MASK                        ((uint32_t)0x000000F0)
#define OTP_MODE_SETTING_TEST_MODE_LSB                         4
#define OTP_MODE_SETTING_TEST_MODE_WIDTH                       ((uint32_t)0x00000004)
#define OTP_MODE_SETTING_PROGREAM_MODE_EN_BIT                  ((uint32_t)0x00000008)
#define OTP_MODE_SETTING_PROGREAM_MODE_EN_POS                  3
#define OTP_MODE_SETTING_DEEP_SLEEP_MODE_EN_BIT                ((uint32_t)0x00000004)
#define OTP_MODE_SETTING_DEEP_SLEEP_MODE_EN_POS                2
#define OTP_MODE_SETTING_READ_EN_BIT                           ((uint32_t)0x00000002)
#define OTP_MODE_SETTING_READ_EN_POS                           1
#define OTP_MODE_SETTING_POWER_ON_BIT                          ((uint32_t)0x00000001)
#define OTP_MODE_SETTING_POWER_ON_POS                          0

#define OTP_MODE_SETTING_SOFT_MODE_RST                         0x0
#define OTP_MODE_SETTING_TEST_MODE_RST                         0x0
#define OTP_MODE_SETTING_PROGREAM_MODE_EN_RST                  0x0
#define OTP_MODE_SETTING_DEEP_SLEEP_MODE_EN_RST                0x0
#define OTP_MODE_SETTING_READ_EN_RST                           0x0
#define OTP_MODE_SETTING_POWER_ON_RST                          0x0

__INLINE void otp_mode_setting_pack(uint8_t softmode, uint8_t testmode, uint8_t progreammodeen, uint8_t deepsleepmodeen, uint8_t readen, uint8_t poweron)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)softmode << 8) | ((uint32_t)testmode << 4) | ((uint32_t)progreammodeen << 3) | ((uint32_t)deepsleepmodeen << 2) | ((uint32_t)readen << 1) | ((uint32_t)poweron << 0));
}

__INLINE void otp_mode_setting_unpack(uint8_t* softmode, uint8_t* testmode, uint8_t* progreammodeen, uint8_t* deepsleepmodeen, uint8_t* readen, uint8_t* poweron)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);

    *softmode = (localVal & ((uint32_t)0x00000100)) >> 8;
    *testmode = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *progreammodeen = (localVal & ((uint32_t)0x00000008)) >> 3;
    *deepsleepmodeen = (localVal & ((uint32_t)0x00000004)) >> 2;
    *readen = (localVal & ((uint32_t)0x00000002)) >> 1;
    *poweron = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t otp_mode_setting_soft_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void otp_mode_setting_soft_mode_setf(uint8_t softmode)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)softmode << 8));
}

__INLINE uint8_t otp_mode_setting_test_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void otp_mode_setting_test_mode_setf(uint8_t testmode)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)testmode << 4));
}

__INLINE uint8_t otp_mode_setting_progream_mode_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void otp_mode_setting_progream_mode_en_setf(uint8_t progreammodeen)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)progreammodeen << 3));
}

__INLINE uint8_t otp_mode_setting_deep_sleep_mode_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void otp_mode_setting_deep_sleep_mode_en_setf(uint8_t deepsleepmodeen)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)deepsleepmodeen << 2));
}

__INLINE uint8_t otp_mode_setting_read_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void otp_mode_setting_read_en_setf(uint8_t readen)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)readen << 1));
}

__INLINE uint8_t otp_mode_setting_power_on_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void otp_mode_setting_power_on_setf(uint8_t poweron)
{
    _PICO_REG_WR(OTP_MODE_SETTING_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_MODE_SETTING_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)poweron << 0));
}

 /**
 * @brief POWER_TIMING_CONTROL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:28                tvddh   0b1
 *  27:24                tpenh   0b1
 *  23:20                 tplh   0b1
 *  19:16                 tash   0b12
 *  15:12                 tsas   0b3
 *  11:08                 tpls   0b12
 *  07:04                tpens   0b2
 *  03:00                 tvds   0b2
 * </pre>
 */
#define OTP_POWER_TIMING_CONTROL_OFFSET 0x00000004


__INLINE uint32_t otp_power_timing_control_get(void)
{
    return _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_power_timing_control_set(uint32_t value)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_POWER_TIMING_CONTROL_TVDDH_MASK                            ((uint32_t)0xF0000000)
#define OTP_POWER_TIMING_CONTROL_TVDDH_LSB                             28
#define OTP_POWER_TIMING_CONTROL_TVDDH_WIDTH                           ((uint32_t)0x00000004)
#define OTP_POWER_TIMING_CONTROL_TPENH_MASK                            ((uint32_t)0x0F000000)
#define OTP_POWER_TIMING_CONTROL_TPENH_LSB                             24
#define OTP_POWER_TIMING_CONTROL_TPENH_WIDTH                           ((uint32_t)0x00000004)
#define OTP_POWER_TIMING_CONTROL_TPLH_MASK                             ((uint32_t)0x00F00000)
#define OTP_POWER_TIMING_CONTROL_TPLH_LSB                              20
#define OTP_POWER_TIMING_CONTROL_TPLH_WIDTH                            ((uint32_t)0x00000004)
#define OTP_POWER_TIMING_CONTROL_TASH_MASK                             ((uint32_t)0x000F0000)
#define OTP_POWER_TIMING_CONTROL_TASH_LSB                              16
#define OTP_POWER_TIMING_CONTROL_TASH_WIDTH                            ((uint32_t)0x00000004)
#define OTP_POWER_TIMING_CONTROL_TSAS_MASK                             ((uint32_t)0x0000F000)
#define OTP_POWER_TIMING_CONTROL_TSAS_LSB                              12
#define OTP_POWER_TIMING_CONTROL_TSAS_WIDTH                            ((uint32_t)0x00000004)
#define OTP_POWER_TIMING_CONTROL_TPLS_MASK                             ((uint32_t)0x00000F00)
#define OTP_POWER_TIMING_CONTROL_TPLS_LSB                              8
#define OTP_POWER_TIMING_CONTROL_TPLS_WIDTH                            ((uint32_t)0x00000004)
#define OTP_POWER_TIMING_CONTROL_TPENS_MASK                            ((uint32_t)0x000000F0)
#define OTP_POWER_TIMING_CONTROL_TPENS_LSB                             4
#define OTP_POWER_TIMING_CONTROL_TPENS_WIDTH                           ((uint32_t)0x00000004)
#define OTP_POWER_TIMING_CONTROL_TVDS_MASK                             ((uint32_t)0x0000000F)
#define OTP_POWER_TIMING_CONTROL_TVDS_LSB                              0
#define OTP_POWER_TIMING_CONTROL_TVDS_WIDTH                            ((uint32_t)0x00000004)

#define OTP_POWER_TIMING_CONTROL_TVDDH_RST                             0x1
#define OTP_POWER_TIMING_CONTROL_TPENH_RST                             0x1
#define OTP_POWER_TIMING_CONTROL_TPLH_RST                              0x1
#define OTP_POWER_TIMING_CONTROL_TASH_RST                              0x12
#define OTP_POWER_TIMING_CONTROL_TSAS_RST                              0x3
#define OTP_POWER_TIMING_CONTROL_TPLS_RST                              0x12
#define OTP_POWER_TIMING_CONTROL_TPENS_RST                             0x2
#define OTP_POWER_TIMING_CONTROL_TVDS_RST                              0x2

__INLINE void otp_power_timing_control_pack(uint8_t tvddh, uint8_t tpenh, uint8_t tplh, uint8_t tash, uint8_t tsas, uint8_t tpls, uint8_t tpens, uint8_t tvds)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)tvddh << 28) | ((uint32_t)tpenh << 24) | ((uint32_t)tplh << 20) | ((uint32_t)tash << 16) | ((uint32_t)tsas << 12) | ((uint32_t)tpls << 8) | ((uint32_t)tpens << 4) | ((uint32_t)tvds << 0));
}

__INLINE void otp_power_timing_control_unpack(uint8_t* tvddh, uint8_t* tpenh, uint8_t* tplh, uint8_t* tash, uint8_t* tsas, uint8_t* tpls, uint8_t* tpens, uint8_t* tvds)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);

    *tvddh = (localVal & ((uint32_t)0xF0000000)) >> 28;
    *tpenh = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *tplh = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *tash = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *tsas = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *tpls = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *tpens = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *tvds = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t otp_power_timing_control_tvddh_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xF0000000)) >> 28);
}

__INLINE void otp_power_timing_control_tvddh_setf(uint8_t tvddh)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0xF0000000)) | ((uint32_t)tvddh << 28));
}

__INLINE uint8_t otp_power_timing_control_tpenh_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void otp_power_timing_control_tpenh_setf(uint8_t tpenh)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)tpenh << 24));
}

__INLINE uint8_t otp_power_timing_control_tplh_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void otp_power_timing_control_tplh_setf(uint8_t tplh)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)tplh << 20));
}

__INLINE uint8_t otp_power_timing_control_tash_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void otp_power_timing_control_tash_setf(uint8_t tash)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)tash << 16));
}

__INLINE uint8_t otp_power_timing_control_tsas_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void otp_power_timing_control_tsas_setf(uint8_t tsas)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)tsas << 12));
}

__INLINE uint8_t otp_power_timing_control_tpls_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void otp_power_timing_control_tpls_setf(uint8_t tpls)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)tpls << 8));
}

__INLINE uint8_t otp_power_timing_control_tpens_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void otp_power_timing_control_tpens_setf(uint8_t tpens)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)tpens << 4));
}

__INLINE uint8_t otp_power_timing_control_tvds_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void otp_power_timing_control_tvds_setf(uint8_t tvds)
{
    _PICO_REG_WR(OTP_POWER_TIMING_CONTROL_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_POWER_TIMING_CONTROL_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)tvds << 0));
}

 /**
 * @brief SETTING1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:21                 tpwi   0b2
 *  20:16                  tpw   0b12
 *  15:11                 tpps   0b6
 *  10:04                 tcps   0b12
 *  03:00                  tms   0b2
 * </pre>
 */
#define OTP_SETTING1_OFFSET 0x00000008


__INLINE uint32_t otp_setting1_get(void)
{
    return _PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_setting1_set(uint32_t value)
{
    _PICO_REG_WR(OTP_SETTING1_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_SETTING1_TPWI_MASK                             ((uint32_t)0x00E00000)
#define OTP_SETTING1_TPWI_LSB                              21
#define OTP_SETTING1_TPWI_WIDTH                            ((uint32_t)0x00000003)
#define OTP_SETTING1_TPW_MASK                              ((uint32_t)0x001F0000)
#define OTP_SETTING1_TPW_LSB                               16
#define OTP_SETTING1_TPW_WIDTH                             ((uint32_t)0x00000005)
#define OTP_SETTING1_TPPS_MASK                             ((uint32_t)0x0000F800)
#define OTP_SETTING1_TPPS_LSB                              11
#define OTP_SETTING1_TPPS_WIDTH                            ((uint32_t)0x00000005)
#define OTP_SETTING1_TCPS_MASK                             ((uint32_t)0x000007F0)
#define OTP_SETTING1_TCPS_LSB                              4
#define OTP_SETTING1_TCPS_WIDTH                            ((uint32_t)0x00000007)
#define OTP_SETTING1_TMS_MASK                              ((uint32_t)0x0000000F)
#define OTP_SETTING1_TMS_LSB                               0
#define OTP_SETTING1_TMS_WIDTH                             ((uint32_t)0x00000004)

#define OTP_SETTING1_TPWI_RST                              0x2
#define OTP_SETTING1_TPW_RST                               0x12
#define OTP_SETTING1_TPPS_RST                              0x6
#define OTP_SETTING1_TCPS_RST                              0x12
#define OTP_SETTING1_TMS_RST                               0x2

__INLINE void otp_setting1_pack(uint8_t tpwi, uint8_t tpw, uint8_t tpps, uint8_t tcps, uint8_t tms)
{
    _PICO_REG_WR(OTP_SETTING1_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)tpwi << 21) | ((uint32_t)tpw << 16) | ((uint32_t)tpps << 11) | ((uint32_t)tcps << 4) | ((uint32_t)tms << 0));
}

__INLINE void otp_setting1_unpack(uint8_t* tpwi, uint8_t* tpw, uint8_t* tpps, uint8_t* tcps, uint8_t* tms)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR);

    *tpwi = (localVal & ((uint32_t)0x00E00000)) >> 21;
    *tpw = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *tpps = (localVal & ((uint32_t)0x0000F800)) >> 11;
    *tcps = (localVal & ((uint32_t)0x000007F0)) >> 4;
    *tms = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t otp_setting1_tpwi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00E00000)) >> 21);
}

__INLINE void otp_setting1_tpwi_setf(uint8_t tpwi)
{
    _PICO_REG_WR(OTP_SETTING1_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00E00000)) | ((uint32_t)tpwi << 21));
}

__INLINE uint8_t otp_setting1_tpw_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE void otp_setting1_tpw_setf(uint8_t tpw)
{
    _PICO_REG_WR(OTP_SETTING1_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x001F0000)) | ((uint32_t)tpw << 16));
}

__INLINE uint8_t otp_setting1_tpps_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000F800)) >> 11);
}

__INLINE void otp_setting1_tpps_setf(uint8_t tpps)
{
    _PICO_REG_WR(OTP_SETTING1_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x0000F800)) | ((uint32_t)tpps << 11));
}

__INLINE uint8_t otp_setting1_tcps_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000007F0)) >> 4);
}

__INLINE void otp_setting1_tcps_setf(uint8_t tcps)
{
    _PICO_REG_WR(OTP_SETTING1_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x000007F0)) | ((uint32_t)tcps << 4));
}

__INLINE uint8_t otp_setting1_tms_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void otp_setting1_tms_setf(uint8_t tms)
{
    _PICO_REG_WR(OTP_SETTING1_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING1_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)tms << 0));
}

 /**
 * @brief SETTING2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:16                  tmh   0b1
 *  14:08                 tppr   0b12
 *  04:00                 tpph   0b6
 * </pre>
 */
#define OTP_SETTING2_OFFSET 0x0000000C


__INLINE uint32_t otp_setting2_get(void)
{
    return _PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_setting2_set(uint32_t value)
{
    _PICO_REG_WR(OTP_SETTING2_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_SETTING2_TMH_MASK                              ((uint32_t)0x000F0000)
#define OTP_SETTING2_TMH_LSB                               16
#define OTP_SETTING2_TMH_WIDTH                             ((uint32_t)0x00000004)
#define OTP_SETTING2_TPPR_MASK                             ((uint32_t)0x00007F00)
#define OTP_SETTING2_TPPR_LSB                              8
#define OTP_SETTING2_TPPR_WIDTH                            ((uint32_t)0x00000007)
#define OTP_SETTING2_TPPH_MASK                             ((uint32_t)0x0000001F)
#define OTP_SETTING2_TPPH_LSB                              0
#define OTP_SETTING2_TPPH_WIDTH                            ((uint32_t)0x00000005)

#define OTP_SETTING2_TMH_RST                               0x1
#define OTP_SETTING2_TPPR_RST                              0x12
#define OTP_SETTING2_TPPH_RST                              0x6

__INLINE void otp_setting2_pack(uint8_t tmh, uint8_t tppr, uint8_t tpph)
{
    _PICO_REG_WR(OTP_SETTING2_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)tmh << 16) | ((uint32_t)tppr << 8) | ((uint32_t)tpph << 0));
}

__INLINE void otp_setting2_unpack(uint8_t* tmh, uint8_t* tppr, uint8_t* tpph)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR);

    *tmh = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *tppr = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *tpph = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t otp_setting2_tmh_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void otp_setting2_tmh_setf(uint8_t tmh)
{
    _PICO_REG_WR(OTP_SETTING2_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)tmh << 16));
}

__INLINE uint8_t otp_setting2_tppr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void otp_setting2_tppr_setf(uint8_t tppr)
{
    _PICO_REG_WR(OTP_SETTING2_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)tppr << 8));
}

__INLINE uint8_t otp_setting2_tpph_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void otp_setting2_tpph_setf(uint8_t tpph)
{
    _PICO_REG_WR(OTP_SETTING2_OFFSET+ OTP_BASE_ADDR, (_PICO_REG_RD(OTP_SETTING2_OFFSET + OTP_BASE_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)tpph << 0));
}

 /**
 * @brief TEST_CTRL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00                  tcs   0b0
 * </pre>
 */
#define OTP_TEST_CTRL_OFFSET 0x00000010


__INLINE uint32_t otp_test_ctrl_get(void)
{
    return _PICO_REG_RD(OTP_TEST_CTRL_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_test_ctrl_set(uint32_t value)
{
    _PICO_REG_WR(OTP_TEST_CTRL_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_TEST_CTRL_TCS_MASK                              ((uint32_t)0x0000000F)
#define OTP_TEST_CTRL_TCS_LSB                               0
#define OTP_TEST_CTRL_TCS_WIDTH                             ((uint32_t)0x00000004)

#define OTP_TEST_CTRL_TCS_RST                               0x0

__INLINE void otp_test_ctrl_pack(uint8_t tcs)
{
    _PICO_REG_WR(OTP_TEST_CTRL_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)tcs << 0));
}

__INLINE void otp_test_ctrl_unpack(uint8_t* tcs)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_TEST_CTRL_OFFSET + OTP_BASE_ADDR);

    *tcs = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t otp_test_ctrl_tcs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_TEST_CTRL_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void otp_test_ctrl_tcs_setf(uint8_t tcs)
{
    _PICO_REG_WR(OTP_TEST_CTRL_OFFSET+ OTP_BASE_ADDR, (uint32_t)tcs << 0);
}

 /**
 * @brief STATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04                 busy   0
 *  03:00                state   0b0
 * </pre>
 */
#define OTP_STATE_OFFSET 0x00000014


__INLINE uint32_t otp_state_get(void)
{
    return _PICO_REG_RD(OTP_STATE_OFFSET + OTP_BASE_ADDR);
}

// field definitions
#define OTP_STATE_BUSY_BIT                              ((uint32_t)0x00000010)
#define OTP_STATE_BUSY_POS                              4
#define OTP_STATE_STATE_MASK                            ((uint32_t)0x0000000F)
#define OTP_STATE_STATE_LSB                             0
#define OTP_STATE_STATE_WIDTH                           ((uint32_t)0x00000004)

#define OTP_STATE_BUSY_RST                              0x0
#define OTP_STATE_STATE_RST                             0x0

__INLINE void otp_state_unpack(uint8_t* busy, uint8_t* state)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_STATE_OFFSET + OTP_BASE_ADDR);

    *busy = (localVal & ((uint32_t)0x00000010)) >> 4;
    *state = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t otp_state_busy_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_STATE_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t otp_state_state_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_STATE_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief PROG_NUM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:00       program_number   0b0
 * </pre>
 */
#define OTP_PROG_NUM_OFFSET 0x00000018


__INLINE uint32_t otp_prog_num_get(void)
{
    return _PICO_REG_RD(OTP_PROG_NUM_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_prog_num_set(uint32_t value)
{
    _PICO_REG_WR(OTP_PROG_NUM_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_PROG_NUM_PROGRAM_NUMBER_MASK                   ((uint32_t)0x00000FFF)
#define OTP_PROG_NUM_PROGRAM_NUMBER_LSB                    0
#define OTP_PROG_NUM_PROGRAM_NUMBER_WIDTH                  ((uint32_t)0x0000000C)

#define OTP_PROG_NUM_PROGRAM_NUMBER_RST                    0x0

__INLINE void otp_prog_num_pack(uint16_t programnumber)
{
    _PICO_REG_WR(OTP_PROG_NUM_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)programnumber << 0));
}

__INLINE void otp_prog_num_unpack(uint8_t* programnumber)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_PROG_NUM_OFFSET + OTP_BASE_ADDR);

    *programnumber = (localVal & ((uint32_t)0x00000FFF)) >> 0;
}

__INLINE uint16_t otp_prog_num_program_number_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_PROG_NUM_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000FFF)) >> 0);
}

__INLINE void otp_prog_num_program_number_setf(uint16_t programnumber)
{
    _PICO_REG_WR(OTP_PROG_NUM_OFFSET+ OTP_BASE_ADDR, (uint32_t)programnumber << 0);
}

 /**
 * @brief PROG_ADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:00      program_address   0b0
 * </pre>
 */
#define OTP_PROG_ADDR_OFFSET 0x0000001C


__INLINE uint32_t otp_prog_addr_get(void)
{
    return _PICO_REG_RD(OTP_PROG_ADDR_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_prog_addr_set(uint32_t value)
{
    _PICO_REG_WR(OTP_PROG_ADDR_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_PROG_ADDR_PROGRAM_ADDRESS_MASK                  ((uint32_t)0x00000FFF)
#define OTP_PROG_ADDR_PROGRAM_ADDRESS_LSB                   0
#define OTP_PROG_ADDR_PROGRAM_ADDRESS_WIDTH                 ((uint32_t)0x0000000C)

#define OTP_PROG_ADDR_PROGRAM_ADDRESS_RST                   0x0

__INLINE void otp_prog_addr_pack(uint16_t programaddress)
{
    _PICO_REG_WR(OTP_PROG_ADDR_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)programaddress << 0));
}

__INLINE void otp_prog_addr_unpack(uint8_t* programaddress)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_PROG_ADDR_OFFSET + OTP_BASE_ADDR);

    *programaddress = (localVal & ((uint32_t)0x00000FFF)) >> 0;
}

__INLINE uint16_t otp_prog_addr_program_address_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_PROG_ADDR_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000FFF)) >> 0);
}

__INLINE void otp_prog_addr_program_address_setf(uint16_t programaddress)
{
    _PICO_REG_WR(OTP_PROG_ADDR_OFFSET+ OTP_BASE_ADDR, (uint32_t)programaddress << 0);
}

 /**
 * @brief PROG_DATA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00      program_data_in   0b0
 * </pre>
 */
#define OTP_PROG_DATA_OFFSET 0x00000020


__INLINE uint32_t otp_prog_data_get(void)
{
    return _PICO_REG_RD(OTP_PROG_DATA_OFFSET + OTP_BASE_ADDR);
}

__INLINE void otp_prog_data_set(uint32_t value)
{
    _PICO_REG_WR(OTP_PROG_DATA_OFFSET+ OTP_BASE_ADDR, value);
}

// field definitions
#define OTP_PROG_DATA_PROGRAM_DATA_IN_MASK                  ((uint32_t)0xFFFFFFFF)
#define OTP_PROG_DATA_PROGRAM_DATA_IN_LSB                   0
#define OTP_PROG_DATA_PROGRAM_DATA_IN_WIDTH                 ((uint32_t)0x00000020)

#define OTP_PROG_DATA_PROGRAM_DATA_IN_RST                   0x0

__INLINE void otp_prog_data_pack(uint32_t programdatain)
{
    _PICO_REG_WR(OTP_PROG_DATA_OFFSET+ OTP_BASE_ADDR,  ((uint32_t)programdatain << 0));
}

__INLINE void otp_prog_data_unpack(uint8_t* programdatain)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_PROG_DATA_OFFSET + OTP_BASE_ADDR);

    *programdatain = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t otp_prog_data_program_data_in_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(OTP_PROG_DATA_OFFSET + OTP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void otp_prog_data_program_data_in_setf(uint32_t programdatain)
{
    _PICO_REG_WR(OTP_PROG_DATA_OFFSET+ OTP_BASE_ADDR, (uint32_t)programdatain << 0);
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :23;
      __IO uint32_t soft_mode:1;
      __IO uint32_t test_mode:4;
      __IO uint32_t progream_mode_en:1;
      __IO uint32_t deep_sleep_mode_en:1;
      __IO uint32_t read_en:1;
      __IO uint32_t power_on:1;
    }mode_setting_fld;
    __IO uint32_t mode_setting;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t tvddh:4;
      __IO uint32_t tpenh:4;
      __IO uint32_t tplh:4;
      __IO uint32_t tash:4;
      __IO uint32_t tsas:4;
      __IO uint32_t tpls:4;
      __IO uint32_t tpens:4;
      __IO uint32_t tvds:4;
    }power_timing_control_fld;
    __IO uint32_t power_timing_control;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :8;
      __IO uint32_t tpwi:3;
      __IO uint32_t tpw:5;
      __IO uint32_t tpps:5;
      __IO uint32_t tcps:7;
      __IO uint32_t tms:4;
    }setting1_fld;
    __IO uint32_t setting1;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :12;
      __IO uint32_t tmh:4;
      __IO uint32_t :1;
      __IO uint32_t tppr:7;
      __IO uint32_t :3;
      __IO uint32_t tpph:5;
    }setting2_fld;
    __IO uint32_t setting2;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :28;
      __IO uint32_t tcs:4;
    }test_ctrl_fld;
    __IO uint32_t test_ctrl;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :27;
      __IO uint32_t busy:1;
      __IO uint32_t state:4;
    }state_fld;
    __IO uint32_t state;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :20;
      __IO uint32_t program_number:12;
    }prog_num_fld;
    __IO uint32_t prog_num;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :20;
      __IO uint32_t program_address:12;
    }prog_addr_fld;
    __IO uint32_t prog_addr;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t program_data_in:32;
    }prog_data_fld;
    __IO uint32_t prog_data;
  };

} PICO_REG_OTP_TypeDef;

#define PICO_REG_OTP PICO_REG_OTP_TypeDef *0x1FFFC000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_OTP_H_

