#ifndef _PICO_REG_PCR_H_
#define _PICO_REG_PCR_H_

#include <stdint.h>

#define PCR_COUNT 13

#define PCR_BASE_ADDR 0x40000000

#define PCR_SIZE 0x00000030


 /**
 * @brief SW_RESET0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     25         srst_uart2_n   1
 *     19          srst_spif_n   1
 *     18           srst_pwm_n   1
 *     17          srst_adcc_n   1
 *     15          srst_qdec_n   1
 *     13          srst_gpio_n   1
 *     12          srst_spi1_n   1
 *     11          srst_spi0_n   1
 *     10          srst_i2c1_n   1
 *     09          srst_i2c0_n   1
 *     08         srst_uart1_n   1
 *     07         srst_iomux_n   1
 *     05         srst_timer_n   1
 *     04           srst_aes_n   1
 *     03           srst_dma_n   1
 *     02         wdt_reset_en   0
 *     01   m0_lockup_reset_en   0
 *     00     ck802_soft_reset   1
 * </pre>
 */
#define PCR_SW_RESET0_OFFSET 0x00000000


__INLINE uint32_t pcr_sw_reset0_get(void)
{
    return _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_sw_reset0_set(uint32_t value)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_SW_RESET0_SRST_UART2_N_BIT                      ((uint32_t)0x02000000)
#define PCR_SW_RESET0_SRST_UART2_N_POS                      25
#define PCR_SW_RESET0_SRST_SPIF_N_BIT                       ((uint32_t)0x00080000)
#define PCR_SW_RESET0_SRST_SPIF_N_POS                       19
#define PCR_SW_RESET0_SRST_PWM_N_BIT                        ((uint32_t)0x00040000)
#define PCR_SW_RESET0_SRST_PWM_N_POS                        18
#define PCR_SW_RESET0_SRST_ADCC_N_BIT                       ((uint32_t)0x00020000)
#define PCR_SW_RESET0_SRST_ADCC_N_POS                       17
#define PCR_SW_RESET0_SRST_QDEC_N_BIT                       ((uint32_t)0x00008000)
#define PCR_SW_RESET0_SRST_QDEC_N_POS                       15
#define PCR_SW_RESET0_SRST_GPIO_N_BIT                       ((uint32_t)0x00002000)
#define PCR_SW_RESET0_SRST_GPIO_N_POS                       13
#define PCR_SW_RESET0_SRST_SPI1_N_BIT                       ((uint32_t)0x00001000)
#define PCR_SW_RESET0_SRST_SPI1_N_POS                       12
#define PCR_SW_RESET0_SRST_SPI0_N_BIT                       ((uint32_t)0x00000800)
#define PCR_SW_RESET0_SRST_SPI0_N_POS                       11
#define PCR_SW_RESET0_SRST_I2C1_N_BIT                       ((uint32_t)0x00000400)
#define PCR_SW_RESET0_SRST_I2C1_N_POS                       10
#define PCR_SW_RESET0_SRST_I2C0_N_BIT                       ((uint32_t)0x00000200)
#define PCR_SW_RESET0_SRST_I2C0_N_POS                       9
#define PCR_SW_RESET0_SRST_UART1_N_BIT                      ((uint32_t)0x00000100)
#define PCR_SW_RESET0_SRST_UART1_N_POS                      8
#define PCR_SW_RESET0_SRST_IOMUX_N_BIT                      ((uint32_t)0x00000080)
#define PCR_SW_RESET0_SRST_IOMUX_N_POS                      7
#define PCR_SW_RESET0_SRST_TIMER_N_BIT                      ((uint32_t)0x00000020)
#define PCR_SW_RESET0_SRST_TIMER_N_POS                      5
#define PCR_SW_RESET0_SRST_AES_N_BIT                        ((uint32_t)0x00000010)
#define PCR_SW_RESET0_SRST_AES_N_POS                        4
#define PCR_SW_RESET0_SRST_DMA_N_BIT                        ((uint32_t)0x00000008)
#define PCR_SW_RESET0_SRST_DMA_N_POS                        3
#define PCR_SW_RESET0_WDT_RESET_EN_BIT                      ((uint32_t)0x00000004)
#define PCR_SW_RESET0_WDT_RESET_EN_POS                      2
#define PCR_SW_RESET0_M0_LOCKUP_RESET_EN_BIT                ((uint32_t)0x00000002)
#define PCR_SW_RESET0_M0_LOCKUP_RESET_EN_POS                1
#define PCR_SW_RESET0_CK802_SOFT_RESET_BIT                  ((uint32_t)0x00000001)
#define PCR_SW_RESET0_CK802_SOFT_RESET_POS                  0

#define PCR_SW_RESET0_SRST_UART2_N_RST                      0x1
#define PCR_SW_RESET0_SRST_SPIF_N_RST                       0x1
#define PCR_SW_RESET0_SRST_PWM_N_RST                        0x1
#define PCR_SW_RESET0_SRST_ADCC_N_RST                       0x1
#define PCR_SW_RESET0_SRST_QDEC_N_RST                       0x1
#define PCR_SW_RESET0_SRST_GPIO_N_RST                       0x1
#define PCR_SW_RESET0_SRST_SPI1_N_RST                       0x1
#define PCR_SW_RESET0_SRST_SPI0_N_RST                       0x1
#define PCR_SW_RESET0_SRST_I2C1_N_RST                       0x1
#define PCR_SW_RESET0_SRST_I2C0_N_RST                       0x1
#define PCR_SW_RESET0_SRST_UART1_N_RST                      0x1
#define PCR_SW_RESET0_SRST_IOMUX_N_RST                      0x1
#define PCR_SW_RESET0_SRST_TIMER_N_RST                      0x1
#define PCR_SW_RESET0_SRST_AES_N_RST                        0x1
#define PCR_SW_RESET0_SRST_DMA_N_RST                        0x1
#define PCR_SW_RESET0_WDT_RESET_EN_RST                      0x0
#define PCR_SW_RESET0_M0_LOCKUP_RESET_EN_RST                0x0
#define PCR_SW_RESET0_CK802_SOFT_RESET_RST                  0x1

__INLINE void pcr_sw_reset0_pack(uint8_t srstuart2n, uint8_t srstspifn, uint8_t srstpwmn, uint8_t srstadccn, uint8_t srstqdecn, uint8_t srstgpion, uint8_t srstspi1n, uint8_t srstspi0n, uint8_t srsti2c1n, uint8_t srsti2c0n, uint8_t srstuart1n, uint8_t srstiomuxn, uint8_t srsttimern, uint8_t srstaesn, uint8_t srstdman, uint8_t wdtreseten, uint8_t m0lockupreseten, uint8_t ck802softreset)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)srstuart2n << 25) | ((uint32_t)srstspifn << 19) | ((uint32_t)srstpwmn << 18) | ((uint32_t)srstadccn << 17) | ((uint32_t)srstqdecn << 15) | ((uint32_t)srstgpion << 13) | ((uint32_t)srstspi1n << 12) | ((uint32_t)srstspi0n << 11) | ((uint32_t)srsti2c1n << 10) | ((uint32_t)srsti2c0n << 9) | ((uint32_t)srstuart1n << 8) | ((uint32_t)srstiomuxn << 7) | ((uint32_t)srsttimern << 5) | ((uint32_t)srstaesn << 4) | ((uint32_t)srstdman << 3) | ((uint32_t)wdtreseten << 2) | ((uint32_t)m0lockupreseten << 1) | ((uint32_t)ck802softreset << 0));
}

__INLINE void pcr_sw_reset0_unpack(uint8_t* srstuart2n, uint8_t* srstspifn, uint8_t* srstpwmn, uint8_t* srstadccn, uint8_t* srstqdecn, uint8_t* srstgpion, uint8_t* srstspi1n, uint8_t* srstspi0n, uint8_t* srsti2c1n, uint8_t* srsti2c0n, uint8_t* srstuart1n, uint8_t* srstiomuxn, uint8_t* srsttimern, uint8_t* srstaesn, uint8_t* srstdman, uint8_t* wdtreseten, uint8_t* m0lockupreseten, uint8_t* ck802softreset)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);

    *srstuart2n = (localVal & ((uint32_t)0x02000000)) >> 25;
    *srstspifn = (localVal & ((uint32_t)0x00080000)) >> 19;
    *srstpwmn = (localVal & ((uint32_t)0x00040000)) >> 18;
    *srstadccn = (localVal & ((uint32_t)0x00020000)) >> 17;
    *srstqdecn = (localVal & ((uint32_t)0x00008000)) >> 15;
    *srstgpion = (localVal & ((uint32_t)0x00002000)) >> 13;
    *srstspi1n = (localVal & ((uint32_t)0x00001000)) >> 12;
    *srstspi0n = (localVal & ((uint32_t)0x00000800)) >> 11;
    *srsti2c1n = (localVal & ((uint32_t)0x00000400)) >> 10;
    *srsti2c0n = (localVal & ((uint32_t)0x00000200)) >> 9;
    *srstuart1n = (localVal & ((uint32_t)0x00000100)) >> 8;
    *srstiomuxn = (localVal & ((uint32_t)0x00000080)) >> 7;
    *srsttimern = (localVal & ((uint32_t)0x00000020)) >> 5;
    *srstaesn = (localVal & ((uint32_t)0x00000010)) >> 4;
    *srstdman = (localVal & ((uint32_t)0x00000008)) >> 3;
    *wdtreseten = (localVal & ((uint32_t)0x00000004)) >> 2;
    *m0lockupreseten = (localVal & ((uint32_t)0x00000002)) >> 1;
    *ck802softreset = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_sw_reset0_srst_uart2_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void pcr_sw_reset0_srst_uart2_n_setf(uint8_t srstuart2n)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)srstuart2n << 25));
}

__INLINE uint8_t pcr_sw_reset0_srst_spif_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void pcr_sw_reset0_srst_spif_n_setf(uint8_t srstspifn)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)srstspifn << 19));
}

__INLINE uint8_t pcr_sw_reset0_srst_pwm_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void pcr_sw_reset0_srst_pwm_n_setf(uint8_t srstpwmn)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)srstpwmn << 18));
}

__INLINE uint8_t pcr_sw_reset0_srst_adcc_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void pcr_sw_reset0_srst_adcc_n_setf(uint8_t srstadccn)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)srstadccn << 17));
}

__INLINE uint8_t pcr_sw_reset0_srst_qdec_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void pcr_sw_reset0_srst_qdec_n_setf(uint8_t srstqdecn)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)srstqdecn << 15));
}

__INLINE uint8_t pcr_sw_reset0_srst_gpio_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void pcr_sw_reset0_srst_gpio_n_setf(uint8_t srstgpion)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)srstgpion << 13));
}

__INLINE uint8_t pcr_sw_reset0_srst_spi1_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void pcr_sw_reset0_srst_spi1_n_setf(uint8_t srstspi1n)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)srstspi1n << 12));
}

__INLINE uint8_t pcr_sw_reset0_srst_spi0_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void pcr_sw_reset0_srst_spi0_n_setf(uint8_t srstspi0n)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)srstspi0n << 11));
}

__INLINE uint8_t pcr_sw_reset0_srst_i2c1_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void pcr_sw_reset0_srst_i2c1_n_setf(uint8_t srsti2c1n)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)srsti2c1n << 10));
}

__INLINE uint8_t pcr_sw_reset0_srst_i2c0_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void pcr_sw_reset0_srst_i2c0_n_setf(uint8_t srsti2c0n)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)srsti2c0n << 9));
}

__INLINE uint8_t pcr_sw_reset0_srst_uart1_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcr_sw_reset0_srst_uart1_n_setf(uint8_t srstuart1n)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)srstuart1n << 8));
}

__INLINE uint8_t pcr_sw_reset0_srst_iomux_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void pcr_sw_reset0_srst_iomux_n_setf(uint8_t srstiomuxn)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)srstiomuxn << 7));
}

__INLINE uint8_t pcr_sw_reset0_srst_timer_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void pcr_sw_reset0_srst_timer_n_setf(uint8_t srsttimern)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)srsttimern << 5));
}

__INLINE uint8_t pcr_sw_reset0_srst_aes_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcr_sw_reset0_srst_aes_n_setf(uint8_t srstaesn)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)srstaesn << 4));
}

__INLINE uint8_t pcr_sw_reset0_srst_dma_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcr_sw_reset0_srst_dma_n_setf(uint8_t srstdman)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)srstdman << 3));
}

__INLINE uint8_t pcr_sw_reset0_wdt_reset_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void pcr_sw_reset0_wdt_reset_en_setf(uint8_t wdtreseten)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)wdtreseten << 2));
}

__INLINE uint8_t pcr_sw_reset0_m0_lockup_reset_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_sw_reset0_m0_lockup_reset_en_setf(uint8_t m0lockupreseten)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)m0lockupreseten << 1));
}

__INLINE uint8_t pcr_sw_reset0_ck802_soft_reset_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_sw_reset0_ck802_soft_reset_setf(uint8_t ck802softreset)
{
    _PICO_REG_WR(PCR_SW_RESET0_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET0_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)ck802softreset << 0));
}

 /**
 * @brief SW_RESET1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02       all_srst_n_pls   1
 *     01       cpu_srst_n_pls   1
 *     00       sys_srst_n_pls   1
 * </pre>
 */
#define PCR_SW_RESET1_OFFSET 0x00000004


__INLINE uint32_t pcr_sw_reset1_get(void)
{
    return _PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_sw_reset1_set(uint32_t value)
{
    _PICO_REG_WR(PCR_SW_RESET1_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_SW_RESET1_ALL_SRST_N_PLS_BIT                    ((uint32_t)0x00000004)
#define PCR_SW_RESET1_ALL_SRST_N_PLS_POS                    2
#define PCR_SW_RESET1_CPU_SRST_N_PLS_BIT                    ((uint32_t)0x00000002)
#define PCR_SW_RESET1_CPU_SRST_N_PLS_POS                    1
#define PCR_SW_RESET1_SYS_SRST_N_PLS_BIT                    ((uint32_t)0x00000001)
#define PCR_SW_RESET1_SYS_SRST_N_PLS_POS                    0

#define PCR_SW_RESET1_ALL_SRST_N_PLS_RST                    0x1
#define PCR_SW_RESET1_CPU_SRST_N_PLS_RST                    0x1
#define PCR_SW_RESET1_SYS_SRST_N_PLS_RST                    0x1

__INLINE void pcr_sw_reset1_pack(uint8_t allsrstnpls, uint8_t cpusrstnpls, uint8_t syssrstnpls)
{
    _PICO_REG_WR(PCR_SW_RESET1_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)allsrstnpls << 2) | ((uint32_t)cpusrstnpls << 1) | ((uint32_t)syssrstnpls << 0));
}

__INLINE void pcr_sw_reset1_unpack(uint8_t* allsrstnpls, uint8_t* cpusrstnpls, uint8_t* syssrstnpls)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR);

    *allsrstnpls = (localVal & ((uint32_t)0x00000004)) >> 2;
    *cpusrstnpls = (localVal & ((uint32_t)0x00000002)) >> 1;
    *syssrstnpls = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_sw_reset1_all_srst_n_pls_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void pcr_sw_reset1_all_srst_n_pls_setf(uint8_t allsrstnpls)
{
    _PICO_REG_WR(PCR_SW_RESET1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)allsrstnpls << 2));
}

__INLINE uint8_t pcr_sw_reset1_cpu_srst_n_pls_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_sw_reset1_cpu_srst_n_pls_setf(uint8_t cpusrstnpls)
{
    _PICO_REG_WR(PCR_SW_RESET1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)cpusrstnpls << 1));
}

__INLINE uint8_t pcr_sw_reset1_sys_srst_n_pls_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_sw_reset1_sys_srst_n_pls_setf(uint8_t syssrstnpls)
{
    _PICO_REG_WR(PCR_SW_RESET1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)syssrstnpls << 0));
}

 /**
 * @brief SW_CLK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     25           clkg_uart2   1
 *     22          clkg_timer6   1
 *     21          clkg_timer5   1
 *     19            clkg_spif   1
 *     18             clkg_pwm   1
 *     17            clkg_adcc   1
 *     15            clkg_qdec   1
 *     13            clkg_gpio   1
 *     12            clkg_spi1   1
 *     11            clkg_spi0   1
 *     10            clkg_i2c1   1
 *     09            clkg_i2c0   1
 *     08            clkg_uart   1
 *     07           clkg_iomux   1
 *     04             clkg_aes   1
 *     03             clkg_dma   1
 *     00       ck802_clkg_cpu   1
 * </pre>
 */
#define PCR_SW_CLK_OFFSET 0x00000008


__INLINE uint32_t pcr_sw_clk_get(void)
{
    return _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_sw_clk_set(uint32_t value)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_SW_CLK_CLKG_UART2_BIT                        ((uint32_t)0x02000000)
#define PCR_SW_CLK_CLKG_UART2_POS                        25
#define PCR_SW_CLK_CLKG_TIMER6_BIT                       ((uint32_t)0x00400000)
#define PCR_SW_CLK_CLKG_TIMER6_POS                       22
#define PCR_SW_CLK_CLKG_TIMER5_BIT                       ((uint32_t)0x00200000)
#define PCR_SW_CLK_CLKG_TIMER5_POS                       21
#define PCR_SW_CLK_CLKG_SPIF_BIT                         ((uint32_t)0x00080000)
#define PCR_SW_CLK_CLKG_SPIF_POS                         19
#define PCR_SW_CLK_CLKG_PWM_BIT                          ((uint32_t)0x00040000)
#define PCR_SW_CLK_CLKG_PWM_POS                          18
#define PCR_SW_CLK_CLKG_ADCC_BIT                         ((uint32_t)0x00020000)
#define PCR_SW_CLK_CLKG_ADCC_POS                         17
#define PCR_SW_CLK_CLKG_QDEC_BIT                         ((uint32_t)0x00008000)
#define PCR_SW_CLK_CLKG_QDEC_POS                         15
#define PCR_SW_CLK_CLKG_GPIO_BIT                         ((uint32_t)0x00002000)
#define PCR_SW_CLK_CLKG_GPIO_POS                         13
#define PCR_SW_CLK_CLKG_SPI1_BIT                         ((uint32_t)0x00001000)
#define PCR_SW_CLK_CLKG_SPI1_POS                         12
#define PCR_SW_CLK_CLKG_SPI0_BIT                         ((uint32_t)0x00000800)
#define PCR_SW_CLK_CLKG_SPI0_POS                         11
#define PCR_SW_CLK_CLKG_I2C1_BIT                         ((uint32_t)0x00000400)
#define PCR_SW_CLK_CLKG_I2C1_POS                         10
#define PCR_SW_CLK_CLKG_I2C0_BIT                         ((uint32_t)0x00000200)
#define PCR_SW_CLK_CLKG_I2C0_POS                         9
#define PCR_SW_CLK_CLKG_UART_BIT                         ((uint32_t)0x00000100)
#define PCR_SW_CLK_CLKG_UART_POS                         8
#define PCR_SW_CLK_CLKG_IOMUX_BIT                        ((uint32_t)0x00000080)
#define PCR_SW_CLK_CLKG_IOMUX_POS                        7
#define PCR_SW_CLK_CLKG_AES_BIT                          ((uint32_t)0x00000010)
#define PCR_SW_CLK_CLKG_AES_POS                          4
#define PCR_SW_CLK_CLKG_DMA_BIT                          ((uint32_t)0x00000008)
#define PCR_SW_CLK_CLKG_DMA_POS                          3
#define PCR_SW_CLK_CK802_CLKG_CPU_BIT                    ((uint32_t)0x00000001)
#define PCR_SW_CLK_CK802_CLKG_CPU_POS                    0

#define PCR_SW_CLK_CLKG_UART2_RST                        0x1
#define PCR_SW_CLK_CLKG_TIMER6_RST                       0x1
#define PCR_SW_CLK_CLKG_TIMER5_RST                       0x1
#define PCR_SW_CLK_CLKG_SPIF_RST                         0x1
#define PCR_SW_CLK_CLKG_PWM_RST                          0x1
#define PCR_SW_CLK_CLKG_ADCC_RST                         0x1
#define PCR_SW_CLK_CLKG_QDEC_RST                         0x1
#define PCR_SW_CLK_CLKG_GPIO_RST                         0x1
#define PCR_SW_CLK_CLKG_SPI1_RST                         0x1
#define PCR_SW_CLK_CLKG_SPI0_RST                         0x1
#define PCR_SW_CLK_CLKG_I2C1_RST                         0x1
#define PCR_SW_CLK_CLKG_I2C0_RST                         0x1
#define PCR_SW_CLK_CLKG_UART_RST                         0x1
#define PCR_SW_CLK_CLKG_IOMUX_RST                        0x1
#define PCR_SW_CLK_CLKG_AES_RST                          0x1
#define PCR_SW_CLK_CLKG_DMA_RST                          0x1
#define PCR_SW_CLK_CK802_CLKG_CPU_RST                    0x1

__INLINE void pcr_sw_clk_pack(uint8_t clkguart2, uint8_t clkgtimer6, uint8_t clkgtimer5, uint8_t clkgspif, uint8_t clkgpwm, uint8_t clkgadcc, uint8_t clkgqdec, uint8_t clkggpio, uint8_t clkgspi1, uint8_t clkgspi0, uint8_t clkgi2c1, uint8_t clkgi2c0, uint8_t clkguart, uint8_t clkgiomux, uint8_t clkgaes, uint8_t clkgdma, uint8_t ck802clkgcpu)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)clkguart2 << 25) | ((uint32_t)clkgtimer6 << 22) | ((uint32_t)clkgtimer5 << 21) | ((uint32_t)clkgspif << 19) | ((uint32_t)clkgpwm << 18) | ((uint32_t)clkgadcc << 17) | ((uint32_t)clkgqdec << 15) | ((uint32_t)clkggpio << 13) | ((uint32_t)clkgspi1 << 12) | ((uint32_t)clkgspi0 << 11) | ((uint32_t)clkgi2c1 << 10) | ((uint32_t)clkgi2c0 << 9) | ((uint32_t)clkguart << 8) | ((uint32_t)clkgiomux << 7) | ((uint32_t)clkgaes << 4) | ((uint32_t)clkgdma << 3) | ((uint32_t)ck802clkgcpu << 0));
}

__INLINE void pcr_sw_clk_unpack(uint8_t* clkguart2, uint8_t* clkgtimer6, uint8_t* clkgtimer5, uint8_t* clkgspif, uint8_t* clkgpwm, uint8_t* clkgadcc, uint8_t* clkgqdec, uint8_t* clkggpio, uint8_t* clkgspi1, uint8_t* clkgspi0, uint8_t* clkgi2c1, uint8_t* clkgi2c0, uint8_t* clkguart, uint8_t* clkgiomux, uint8_t* clkgaes, uint8_t* clkgdma, uint8_t* ck802clkgcpu)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);

    *clkguart2 = (localVal & ((uint32_t)0x02000000)) >> 25;
    *clkgtimer6 = (localVal & ((uint32_t)0x00400000)) >> 22;
    *clkgtimer5 = (localVal & ((uint32_t)0x00200000)) >> 21;
    *clkgspif = (localVal & ((uint32_t)0x00080000)) >> 19;
    *clkgpwm = (localVal & ((uint32_t)0x00040000)) >> 18;
    *clkgadcc = (localVal & ((uint32_t)0x00020000)) >> 17;
    *clkgqdec = (localVal & ((uint32_t)0x00008000)) >> 15;
    *clkggpio = (localVal & ((uint32_t)0x00002000)) >> 13;
    *clkgspi1 = (localVal & ((uint32_t)0x00001000)) >> 12;
    *clkgspi0 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *clkgi2c1 = (localVal & ((uint32_t)0x00000400)) >> 10;
    *clkgi2c0 = (localVal & ((uint32_t)0x00000200)) >> 9;
    *clkguart = (localVal & ((uint32_t)0x00000100)) >> 8;
    *clkgiomux = (localVal & ((uint32_t)0x00000080)) >> 7;
    *clkgaes = (localVal & ((uint32_t)0x00000010)) >> 4;
    *clkgdma = (localVal & ((uint32_t)0x00000008)) >> 3;
    *ck802clkgcpu = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_sw_clk_clkg_uart2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void pcr_sw_clk_clkg_uart2_setf(uint8_t clkguart2)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)clkguart2 << 25));
}

__INLINE uint8_t pcr_sw_clk_clkg_timer6_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void pcr_sw_clk_clkg_timer6_setf(uint8_t clkgtimer6)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)clkgtimer6 << 22));
}

__INLINE uint8_t pcr_sw_clk_clkg_timer5_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void pcr_sw_clk_clkg_timer5_setf(uint8_t clkgtimer5)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)clkgtimer5 << 21));
}

__INLINE uint8_t pcr_sw_clk_clkg_spif_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void pcr_sw_clk_clkg_spif_setf(uint8_t clkgspif)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)clkgspif << 19));
}

__INLINE uint8_t pcr_sw_clk_clkg_pwm_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void pcr_sw_clk_clkg_pwm_setf(uint8_t clkgpwm)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)clkgpwm << 18));
}

__INLINE uint8_t pcr_sw_clk_clkg_adcc_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void pcr_sw_clk_clkg_adcc_setf(uint8_t clkgadcc)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)clkgadcc << 17));
}

__INLINE uint8_t pcr_sw_clk_clkg_qdec_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void pcr_sw_clk_clkg_qdec_setf(uint8_t clkgqdec)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)clkgqdec << 15));
}

__INLINE uint8_t pcr_sw_clk_clkg_gpio_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void pcr_sw_clk_clkg_gpio_setf(uint8_t clkggpio)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)clkggpio << 13));
}

__INLINE uint8_t pcr_sw_clk_clkg_spi1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void pcr_sw_clk_clkg_spi1_setf(uint8_t clkgspi1)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)clkgspi1 << 12));
}

__INLINE uint8_t pcr_sw_clk_clkg_spi0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void pcr_sw_clk_clkg_spi0_setf(uint8_t clkgspi0)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)clkgspi0 << 11));
}

__INLINE uint8_t pcr_sw_clk_clkg_i2c1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void pcr_sw_clk_clkg_i2c1_setf(uint8_t clkgi2c1)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)clkgi2c1 << 10));
}

__INLINE uint8_t pcr_sw_clk_clkg_i2c0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void pcr_sw_clk_clkg_i2c0_setf(uint8_t clkgi2c0)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)clkgi2c0 << 9));
}

__INLINE uint8_t pcr_sw_clk_clkg_uart_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcr_sw_clk_clkg_uart_setf(uint8_t clkguart)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)clkguart << 8));
}

__INLINE uint8_t pcr_sw_clk_clkg_iomux_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void pcr_sw_clk_clkg_iomux_setf(uint8_t clkgiomux)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)clkgiomux << 7));
}

__INLINE uint8_t pcr_sw_clk_clkg_aes_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcr_sw_clk_clkg_aes_setf(uint8_t clkgaes)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)clkgaes << 4));
}

__INLINE uint8_t pcr_sw_clk_clkg_dma_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcr_sw_clk_clkg_dma_setf(uint8_t clkgdma)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)clkgdma << 3));
}

__INLINE uint8_t pcr_sw_clk_ck802_clkg_cpu_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_sw_clk_ck802_clkg_cpu_setf(uint8_t ck802clkgcpu)
{
    _PICO_REG_WR(PCR_SW_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)ck802clkgcpu << 0));
}

 /**
 * @brief SW_RESET2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     13         srst_bbmix_n   1
 *     12          srst_bbrx_n   1
 *     11          srst_bbtx_n   1
 *     10          srst_bbll_n   1
 *     09         srst_bbreg_n   1
 *     08     m0_enabled_by_m4   0
 *     07            srst_ks_n   1
 *     06           srst_com_n   1
 *     05           srst_wdt_n   1
 *     04         srst_timer_n   1
 *     03            srst_bb_n   1
 *     02         wdt_reset_en   0
 *     01   m0_lockup_reset_en   0
 *     00        m0_soft_reset   1
 * </pre>
 */
#define PCR_SW_RESET2_OFFSET 0x0000000C


__INLINE uint32_t pcr_sw_reset2_get(void)
{
    return _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_sw_reset2_set(uint32_t value)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_SW_RESET2_SRST_BBMIX_N_BIT                      ((uint32_t)0x00002000)
#define PCR_SW_RESET2_SRST_BBMIX_N_POS                      13
#define PCR_SW_RESET2_SRST_BBRX_N_BIT                       ((uint32_t)0x00001000)
#define PCR_SW_RESET2_SRST_BBRX_N_POS                       12
#define PCR_SW_RESET2_SRST_BBTX_N_BIT                       ((uint32_t)0x00000800)
#define PCR_SW_RESET2_SRST_BBTX_N_POS                       11
#define PCR_SW_RESET2_SRST_BBLL_N_BIT                       ((uint32_t)0x00000400)
#define PCR_SW_RESET2_SRST_BBLL_N_POS                       10
#define PCR_SW_RESET2_SRST_BBREG_N_BIT                      ((uint32_t)0x00000200)
#define PCR_SW_RESET2_SRST_BBREG_N_POS                      9
#define PCR_SW_RESET2_M0_ENABLED_BY_M4_BIT                  ((uint32_t)0x00000100)
#define PCR_SW_RESET2_M0_ENABLED_BY_M4_POS                  8
#define PCR_SW_RESET2_SRST_KS_N_BIT                         ((uint32_t)0x00000080)
#define PCR_SW_RESET2_SRST_KS_N_POS                         7
#define PCR_SW_RESET2_SRST_COM_N_BIT                        ((uint32_t)0x00000040)
#define PCR_SW_RESET2_SRST_COM_N_POS                        6
#define PCR_SW_RESET2_SRST_WDT_N_BIT                        ((uint32_t)0x00000020)
#define PCR_SW_RESET2_SRST_WDT_N_POS                        5
#define PCR_SW_RESET2_SRST_TIMER_N_BIT                      ((uint32_t)0x00000010)
#define PCR_SW_RESET2_SRST_TIMER_N_POS                      4
#define PCR_SW_RESET2_SRST_BB_N_BIT                         ((uint32_t)0x00000008)
#define PCR_SW_RESET2_SRST_BB_N_POS                         3
#define PCR_SW_RESET2_WDT_RESET_EN_BIT                      ((uint32_t)0x00000004)
#define PCR_SW_RESET2_WDT_RESET_EN_POS                      2
#define PCR_SW_RESET2_M0_LOCKUP_RESET_EN_BIT                ((uint32_t)0x00000002)
#define PCR_SW_RESET2_M0_LOCKUP_RESET_EN_POS                1
#define PCR_SW_RESET2_M0_SOFT_RESET_BIT                     ((uint32_t)0x00000001)
#define PCR_SW_RESET2_M0_SOFT_RESET_POS                     0

#define PCR_SW_RESET2_SRST_BBMIX_N_RST                      0x1
#define PCR_SW_RESET2_SRST_BBRX_N_RST                       0x1
#define PCR_SW_RESET2_SRST_BBTX_N_RST                       0x1
#define PCR_SW_RESET2_SRST_BBLL_N_RST                       0x1
#define PCR_SW_RESET2_SRST_BBREG_N_RST                      0x1
#define PCR_SW_RESET2_M0_ENABLED_BY_M4_RST                  0x0
#define PCR_SW_RESET2_SRST_KS_N_RST                         0x1
#define PCR_SW_RESET2_SRST_COM_N_RST                        0x1
#define PCR_SW_RESET2_SRST_WDT_N_RST                        0x1
#define PCR_SW_RESET2_SRST_TIMER_N_RST                      0x1
#define PCR_SW_RESET2_SRST_BB_N_RST                         0x1
#define PCR_SW_RESET2_WDT_RESET_EN_RST                      0x0
#define PCR_SW_RESET2_M0_LOCKUP_RESET_EN_RST                0x0
#define PCR_SW_RESET2_M0_SOFT_RESET_RST                     0x1

__INLINE void pcr_sw_reset2_pack(uint8_t srstbbmixn, uint8_t srstbbrxn, uint8_t srstbbtxn, uint8_t srstbblln, uint8_t srstbbregn, uint8_t m0enabledbym4, uint8_t srstksn, uint8_t srstcomn, uint8_t srstwdtn, uint8_t srsttimern, uint8_t srstbbn, uint8_t wdtreseten, uint8_t m0lockupreseten, uint8_t m0softreset)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)srstbbmixn << 13) | ((uint32_t)srstbbrxn << 12) | ((uint32_t)srstbbtxn << 11) | ((uint32_t)srstbblln << 10) | ((uint32_t)srstbbregn << 9) | ((uint32_t)m0enabledbym4 << 8) | ((uint32_t)srstksn << 7) | ((uint32_t)srstcomn << 6) | ((uint32_t)srstwdtn << 5) | ((uint32_t)srsttimern << 4) | ((uint32_t)srstbbn << 3) | ((uint32_t)wdtreseten << 2) | ((uint32_t)m0lockupreseten << 1) | ((uint32_t)m0softreset << 0));
}

__INLINE void pcr_sw_reset2_unpack(uint8_t* srstbbmixn, uint8_t* srstbbrxn, uint8_t* srstbbtxn, uint8_t* srstbblln, uint8_t* srstbbregn, uint8_t* m0enabledbym4, uint8_t* srstksn, uint8_t* srstcomn, uint8_t* srstwdtn, uint8_t* srsttimern, uint8_t* srstbbn, uint8_t* wdtreseten, uint8_t* m0lockupreseten, uint8_t* m0softreset)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);

    *srstbbmixn = (localVal & ((uint32_t)0x00002000)) >> 13;
    *srstbbrxn = (localVal & ((uint32_t)0x00001000)) >> 12;
    *srstbbtxn = (localVal & ((uint32_t)0x00000800)) >> 11;
    *srstbblln = (localVal & ((uint32_t)0x00000400)) >> 10;
    *srstbbregn = (localVal & ((uint32_t)0x00000200)) >> 9;
    *m0enabledbym4 = (localVal & ((uint32_t)0x00000100)) >> 8;
    *srstksn = (localVal & ((uint32_t)0x00000080)) >> 7;
    *srstcomn = (localVal & ((uint32_t)0x00000040)) >> 6;
    *srstwdtn = (localVal & ((uint32_t)0x00000020)) >> 5;
    *srsttimern = (localVal & ((uint32_t)0x00000010)) >> 4;
    *srstbbn = (localVal & ((uint32_t)0x00000008)) >> 3;
    *wdtreseten = (localVal & ((uint32_t)0x00000004)) >> 2;
    *m0lockupreseten = (localVal & ((uint32_t)0x00000002)) >> 1;
    *m0softreset = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_sw_reset2_srst_bbmix_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void pcr_sw_reset2_srst_bbmix_n_setf(uint8_t srstbbmixn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)srstbbmixn << 13));
}

__INLINE uint8_t pcr_sw_reset2_srst_bbrx_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void pcr_sw_reset2_srst_bbrx_n_setf(uint8_t srstbbrxn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)srstbbrxn << 12));
}

__INLINE uint8_t pcr_sw_reset2_srst_bbtx_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void pcr_sw_reset2_srst_bbtx_n_setf(uint8_t srstbbtxn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)srstbbtxn << 11));
}

__INLINE uint8_t pcr_sw_reset2_srst_bbll_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void pcr_sw_reset2_srst_bbll_n_setf(uint8_t srstbblln)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)srstbblln << 10));
}

__INLINE uint8_t pcr_sw_reset2_srst_bbreg_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void pcr_sw_reset2_srst_bbreg_n_setf(uint8_t srstbbregn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)srstbbregn << 9));
}

__INLINE uint8_t pcr_sw_reset2_m0_enabled_by_m4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcr_sw_reset2_m0_enabled_by_m4_setf(uint8_t m0enabledbym4)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)m0enabledbym4 << 8));
}

__INLINE uint8_t pcr_sw_reset2_srst_ks_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void pcr_sw_reset2_srst_ks_n_setf(uint8_t srstksn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)srstksn << 7));
}

__INLINE uint8_t pcr_sw_reset2_srst_com_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void pcr_sw_reset2_srst_com_n_setf(uint8_t srstcomn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)srstcomn << 6));
}

__INLINE uint8_t pcr_sw_reset2_srst_wdt_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void pcr_sw_reset2_srst_wdt_n_setf(uint8_t srstwdtn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)srstwdtn << 5));
}

__INLINE uint8_t pcr_sw_reset2_srst_timer_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcr_sw_reset2_srst_timer_n_setf(uint8_t srsttimern)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)srsttimern << 4));
}

__INLINE uint8_t pcr_sw_reset2_srst_bb_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcr_sw_reset2_srst_bb_n_setf(uint8_t srstbbn)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)srstbbn << 3));
}

__INLINE uint8_t pcr_sw_reset2_wdt_reset_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void pcr_sw_reset2_wdt_reset_en_setf(uint8_t wdtreseten)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)wdtreseten << 2));
}

__INLINE uint8_t pcr_sw_reset2_m0_lockup_reset_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_sw_reset2_m0_lockup_reset_en_setf(uint8_t m0lockupreseten)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)m0lockupreseten << 1));
}

__INLINE uint8_t pcr_sw_reset2_m0_soft_reset_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_sw_reset2_m0_soft_reset_setf(uint8_t m0softreset)
{
    _PICO_REG_WR(PCR_SW_RESET2_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET2_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)m0softreset << 0));
}

 /**
 * @brief SW_RESET3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01       cpu_srst_n_pls   1
 *     00       sys_srst_n_pls   1
 * </pre>
 */
#define PCR_SW_RESET3_OFFSET 0x00000010


__INLINE uint32_t pcr_sw_reset3_get(void)
{
    return _PICO_REG_RD(PCR_SW_RESET3_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_sw_reset3_set(uint32_t value)
{
    _PICO_REG_WR(PCR_SW_RESET3_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_SW_RESET3_CPU_SRST_N_PLS_BIT                    ((uint32_t)0x00000002)
#define PCR_SW_RESET3_CPU_SRST_N_PLS_POS                    1
#define PCR_SW_RESET3_SYS_SRST_N_PLS_BIT                    ((uint32_t)0x00000001)
#define PCR_SW_RESET3_SYS_SRST_N_PLS_POS                    0

#define PCR_SW_RESET3_CPU_SRST_N_PLS_RST                    0x1
#define PCR_SW_RESET3_SYS_SRST_N_PLS_RST                    0x1

__INLINE void pcr_sw_reset3_pack(uint8_t cpusrstnpls, uint8_t syssrstnpls)
{
    _PICO_REG_WR(PCR_SW_RESET3_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)cpusrstnpls << 1) | ((uint32_t)syssrstnpls << 0));
}

__INLINE void pcr_sw_reset3_unpack(uint8_t* cpusrstnpls, uint8_t* syssrstnpls)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET3_OFFSET + PCR_BASE_ADDR);

    *cpusrstnpls = (localVal & ((uint32_t)0x00000002)) >> 1;
    *syssrstnpls = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_sw_reset3_cpu_srst_n_pls_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET3_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_sw_reset3_cpu_srst_n_pls_setf(uint8_t cpusrstnpls)
{
    _PICO_REG_WR(PCR_SW_RESET3_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET3_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)cpusrstnpls << 1));
}

__INLINE uint8_t pcr_sw_reset3_sys_srst_n_pls_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_RESET3_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_sw_reset3_sys_srst_n_pls_setf(uint8_t syssrstnpls)
{
    _PICO_REG_WR(PCR_SW_RESET3_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_RESET3_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)syssrstnpls << 0));
}

 /**
 * @brief SW_CLK1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     24          clkg_timer4   1
 *     23          clkg_timer3   1
 *     22          clkg_timer2   1
 *     21          clkg_timer1   1
 *     09           clkg_bbreg   1
 *     07              clkg_ks   1
 *     06             clkg_com   1
 *     05             clkg_wdt   1
 *     04           clkg_timer   1
 *     03              clkg_bb   1
 *     00          m0_clkg_cpu   1
 * </pre>
 */
#define PCR_SW_CLK1_OFFSET 0x00000014


__INLINE uint32_t pcr_sw_clk1_get(void)
{
    return _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_sw_clk1_set(uint32_t value)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_SW_CLK1_CLKG_TIMER4_BIT                       ((uint32_t)0x01000000)
#define PCR_SW_CLK1_CLKG_TIMER4_POS                       24
#define PCR_SW_CLK1_CLKG_TIMER3_BIT                       ((uint32_t)0x00800000)
#define PCR_SW_CLK1_CLKG_TIMER3_POS                       23
#define PCR_SW_CLK1_CLKG_TIMER2_BIT                       ((uint32_t)0x00400000)
#define PCR_SW_CLK1_CLKG_TIMER2_POS                       22
#define PCR_SW_CLK1_CLKG_TIMER1_BIT                       ((uint32_t)0x00200000)
#define PCR_SW_CLK1_CLKG_TIMER1_POS                       21
#define PCR_SW_CLK1_CLKG_BBREG_BIT                        ((uint32_t)0x00000200)
#define PCR_SW_CLK1_CLKG_BBREG_POS                        9
#define PCR_SW_CLK1_CLKG_KS_BIT                           ((uint32_t)0x00000080)
#define PCR_SW_CLK1_CLKG_KS_POS                           7
#define PCR_SW_CLK1_CLKG_COM_BIT                          ((uint32_t)0x00000040)
#define PCR_SW_CLK1_CLKG_COM_POS                          6
#define PCR_SW_CLK1_CLKG_WDT_BIT                          ((uint32_t)0x00000020)
#define PCR_SW_CLK1_CLKG_WDT_POS                          5
#define PCR_SW_CLK1_CLKG_TIMER_BIT                        ((uint32_t)0x00000010)
#define PCR_SW_CLK1_CLKG_TIMER_POS                        4
#define PCR_SW_CLK1_CLKG_BB_BIT                           ((uint32_t)0x00000008)
#define PCR_SW_CLK1_CLKG_BB_POS                           3
#define PCR_SW_CLK1_M0_CLKG_CPU_BIT                       ((uint32_t)0x00000001)
#define PCR_SW_CLK1_M0_CLKG_CPU_POS                       0

#define PCR_SW_CLK1_CLKG_TIMER4_RST                       0x1
#define PCR_SW_CLK1_CLKG_TIMER3_RST                       0x1
#define PCR_SW_CLK1_CLKG_TIMER2_RST                       0x1
#define PCR_SW_CLK1_CLKG_TIMER1_RST                       0x1
#define PCR_SW_CLK1_CLKG_BBREG_RST                        0x1
#define PCR_SW_CLK1_CLKG_KS_RST                           0x1
#define PCR_SW_CLK1_CLKG_COM_RST                          0x1
#define PCR_SW_CLK1_CLKG_WDT_RST                          0x1
#define PCR_SW_CLK1_CLKG_TIMER_RST                        0x1
#define PCR_SW_CLK1_CLKG_BB_RST                           0x1
#define PCR_SW_CLK1_M0_CLKG_CPU_RST                       0x1

__INLINE void pcr_sw_clk1_pack(uint8_t clkgtimer4, uint8_t clkgtimer3, uint8_t clkgtimer2, uint8_t clkgtimer1, uint8_t clkgbbreg, uint8_t clkgks, uint8_t clkgcom, uint8_t clkgwdt, uint8_t clkgtimer, uint8_t clkgbb, uint8_t m0clkgcpu)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)clkgtimer4 << 24) | ((uint32_t)clkgtimer3 << 23) | ((uint32_t)clkgtimer2 << 22) | ((uint32_t)clkgtimer1 << 21) | ((uint32_t)clkgbbreg << 9) | ((uint32_t)clkgks << 7) | ((uint32_t)clkgcom << 6) | ((uint32_t)clkgwdt << 5) | ((uint32_t)clkgtimer << 4) | ((uint32_t)clkgbb << 3) | ((uint32_t)m0clkgcpu << 0));
}

__INLINE void pcr_sw_clk1_unpack(uint8_t* clkgtimer4, uint8_t* clkgtimer3, uint8_t* clkgtimer2, uint8_t* clkgtimer1, uint8_t* clkgbbreg, uint8_t* clkgks, uint8_t* clkgcom, uint8_t* clkgwdt, uint8_t* clkgtimer, uint8_t* clkgbb, uint8_t* m0clkgcpu)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);

    *clkgtimer4 = (localVal & ((uint32_t)0x01000000)) >> 24;
    *clkgtimer3 = (localVal & ((uint32_t)0x00800000)) >> 23;
    *clkgtimer2 = (localVal & ((uint32_t)0x00400000)) >> 22;
    *clkgtimer1 = (localVal & ((uint32_t)0x00200000)) >> 21;
    *clkgbbreg = (localVal & ((uint32_t)0x00000200)) >> 9;
    *clkgks = (localVal & ((uint32_t)0x00000080)) >> 7;
    *clkgcom = (localVal & ((uint32_t)0x00000040)) >> 6;
    *clkgwdt = (localVal & ((uint32_t)0x00000020)) >> 5;
    *clkgtimer = (localVal & ((uint32_t)0x00000010)) >> 4;
    *clkgbb = (localVal & ((uint32_t)0x00000008)) >> 3;
    *m0clkgcpu = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_sw_clk1_clkg_timer4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void pcr_sw_clk1_clkg_timer4_setf(uint8_t clkgtimer4)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)clkgtimer4 << 24));
}

__INLINE uint8_t pcr_sw_clk1_clkg_timer3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void pcr_sw_clk1_clkg_timer3_setf(uint8_t clkgtimer3)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)clkgtimer3 << 23));
}

__INLINE uint8_t pcr_sw_clk1_clkg_timer2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void pcr_sw_clk1_clkg_timer2_setf(uint8_t clkgtimer2)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)clkgtimer2 << 22));
}

__INLINE uint8_t pcr_sw_clk1_clkg_timer1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void pcr_sw_clk1_clkg_timer1_setf(uint8_t clkgtimer1)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)clkgtimer1 << 21));
}

__INLINE uint8_t pcr_sw_clk1_clkg_bbreg_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void pcr_sw_clk1_clkg_bbreg_setf(uint8_t clkgbbreg)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)clkgbbreg << 9));
}

__INLINE uint8_t pcr_sw_clk1_clkg_ks_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void pcr_sw_clk1_clkg_ks_setf(uint8_t clkgks)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)clkgks << 7));
}

__INLINE uint8_t pcr_sw_clk1_clkg_com_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void pcr_sw_clk1_clkg_com_setf(uint8_t clkgcom)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)clkgcom << 6));
}

__INLINE uint8_t pcr_sw_clk1_clkg_wdt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void pcr_sw_clk1_clkg_wdt_setf(uint8_t clkgwdt)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)clkgwdt << 5));
}

__INLINE uint8_t pcr_sw_clk1_clkg_timer_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcr_sw_clk1_clkg_timer_setf(uint8_t clkgtimer)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)clkgtimer << 4));
}

__INLINE uint8_t pcr_sw_clk1_clkg_bb_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcr_sw_clk1_clkg_bb_setf(uint8_t clkgbb)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)clkgbb << 3));
}

__INLINE uint8_t pcr_sw_clk1_m0_clkg_cpu_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_sw_clk1_m0_clkg_cpu_setf(uint8_t m0clkgcpu)
{
    _PICO_REG_WR(PCR_SW_CLK1_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_SW_CLK1_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)m0clkgcpu << 0));
}

 /**
 * @brief APB_CLK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:04          pclk_div_m0   0b0
 *  03:00             reserved   0b0
 * </pre>
 */
#define PCR_APB_CLK_OFFSET 0x00000018


__INLINE uint32_t pcr_apb_clk_get(void)
{
    return _PICO_REG_RD(PCR_APB_CLK_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_apb_clk_set(uint32_t value)
{
    _PICO_REG_WR(PCR_APB_CLK_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_APB_CLK_PCLK_DIV_M0_MASK                      ((uint32_t)0x000000F0)
#define PCR_APB_CLK_PCLK_DIV_M0_LSB                       4
#define PCR_APB_CLK_PCLK_DIV_M0_WIDTH                     ((uint32_t)0x00000004)
#define PCR_APB_CLK_RESERVED_MASK                         ((uint32_t)0x0000000F)
#define PCR_APB_CLK_RESERVED_LSB                          0
#define PCR_APB_CLK_RESERVED_WIDTH                        ((uint32_t)0x00000004)

#define PCR_APB_CLK_PCLK_DIV_M0_RST                       0x0
#define PCR_APB_CLK_RESERVED_RST                          0x0

__INLINE void pcr_apb_clk_pack(uint8_t pclkdivm0, uint8_t reserved)
{
    _PICO_REG_WR(PCR_APB_CLK_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)pclkdivm0 << 4) | ((uint32_t)reserved << 0));
}

__INLINE void pcr_apb_clk_unpack(uint8_t* pclkdivm0, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_APB_CLK_OFFSET + PCR_BASE_ADDR);

    *pclkdivm0 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *reserved = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t pcr_apb_clk_pclk_div_m0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_APB_CLK_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void pcr_apb_clk_pclk_div_m0_setf(uint8_t pclkdivm0)
{
    _PICO_REG_WR(PCR_APB_CLK_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_APB_CLK_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)pclkdivm0 << 4));
}

 /**
 * @brief APB_CLK_UPDATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01        pclk_update_1   0
 *     00        pclk_update_0   0
 * </pre>
 */
#define PCR_APB_CLK_UPDATE_OFFSET 0x0000001C


__INLINE uint32_t pcr_apb_clk_update_get(void)
{
    return _PICO_REG_RD(PCR_APB_CLK_UPDATE_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_apb_clk_update_set(uint32_t value)
{
    _PICO_REG_WR(PCR_APB_CLK_UPDATE_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_APB_CLK_UPDATE_PCLK_UPDATE_1_BIT                     ((uint32_t)0x00000002)
#define PCR_APB_CLK_UPDATE_PCLK_UPDATE_1_POS                     1
#define PCR_APB_CLK_UPDATE_PCLK_UPDATE_0_BIT                     ((uint32_t)0x00000001)
#define PCR_APB_CLK_UPDATE_PCLK_UPDATE_0_POS                     0

#define PCR_APB_CLK_UPDATE_PCLK_UPDATE_1_RST                     0x0
#define PCR_APB_CLK_UPDATE_PCLK_UPDATE_0_RST                     0x0

__INLINE void pcr_apb_clk_update_pack(uint8_t pclkupdate1, uint8_t pclkupdate0)
{
    _PICO_REG_WR(PCR_APB_CLK_UPDATE_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)pclkupdate1 << 1) | ((uint32_t)pclkupdate0 << 0));
}

__INLINE void pcr_apb_clk_update_unpack(uint8_t* pclkupdate1, uint8_t* pclkupdate0)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_APB_CLK_UPDATE_OFFSET + PCR_BASE_ADDR);

    *pclkupdate1 = (localVal & ((uint32_t)0x00000002)) >> 1;
    *pclkupdate0 = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_apb_clk_update_pclk_update_1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_APB_CLK_UPDATE_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_apb_clk_update_pclk_update_1_setf(uint8_t pclkupdate1)
{
    _PICO_REG_WR(PCR_APB_CLK_UPDATE_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_APB_CLK_UPDATE_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)pclkupdate1 << 1));
}

__INLINE uint8_t pcr_apb_clk_update_pclk_update_0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_APB_CLK_UPDATE_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_apb_clk_update_pclk_update_0_setf(uint8_t pclkupdate0)
{
    _PICO_REG_WR(PCR_APB_CLK_UPDATE_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_APB_CLK_UPDATE_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)pclkupdate0 << 0));
}

 /**
 * @brief CACHE_CLOCK_GATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01          pclkg_cache   1
 *     00          hclkg_cache   1
 * </pre>
 */
#define PCR_CACHE_CLOCK_GATE_OFFSET 0x00000020


__INLINE uint32_t pcr_cache_clock_gate_get(void)
{
    return _PICO_REG_RD(PCR_CACHE_CLOCK_GATE_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_cache_clock_gate_set(uint32_t value)
{
    _PICO_REG_WR(PCR_CACHE_CLOCK_GATE_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_CACHE_CLOCK_GATE_PCLKG_CACHE_BIT                       ((uint32_t)0x00000002)
#define PCR_CACHE_CLOCK_GATE_PCLKG_CACHE_POS                       1
#define PCR_CACHE_CLOCK_GATE_HCLKG_CACHE_BIT                       ((uint32_t)0x00000001)
#define PCR_CACHE_CLOCK_GATE_HCLKG_CACHE_POS                       0

#define PCR_CACHE_CLOCK_GATE_PCLKG_CACHE_RST                       0x1
#define PCR_CACHE_CLOCK_GATE_HCLKG_CACHE_RST                       0x1

__INLINE void pcr_cache_clock_gate_pack(uint8_t pclkgcache, uint8_t hclkgcache)
{
    _PICO_REG_WR(PCR_CACHE_CLOCK_GATE_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)pclkgcache << 1) | ((uint32_t)hclkgcache << 0));
}

__INLINE void pcr_cache_clock_gate_unpack(uint8_t* pclkgcache, uint8_t* hclkgcache)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_CACHE_CLOCK_GATE_OFFSET + PCR_BASE_ADDR);

    *pclkgcache = (localVal & ((uint32_t)0x00000002)) >> 1;
    *hclkgcache = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_cache_clock_gate_pclkg_cache_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_CACHE_CLOCK_GATE_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_cache_clock_gate_pclkg_cache_setf(uint8_t pclkgcache)
{
    _PICO_REG_WR(PCR_CACHE_CLOCK_GATE_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_CACHE_CLOCK_GATE_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)pclkgcache << 1));
}

__INLINE uint8_t pcr_cache_clock_gate_hclkg_cache_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_CACHE_CLOCK_GATE_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_cache_clock_gate_hclkg_cache_setf(uint8_t hclkgcache)
{
    _PICO_REG_WR(PCR_CACHE_CLOCK_GATE_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_CACHE_CLOCK_GATE_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)hclkgcache << 0));
}

 /**
 * @brief CACHE_RST register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01          hsrst_cache   1
 *     00          psrst_cache   1
 * </pre>
 */
#define PCR_CACHE_RST_OFFSET 0x00000024


__INLINE uint32_t pcr_cache_rst_get(void)
{
    return _PICO_REG_RD(PCR_CACHE_RST_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_cache_rst_set(uint32_t value)
{
    _PICO_REG_WR(PCR_CACHE_RST_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_CACHE_RST_HSRST_CACHE_BIT                       ((uint32_t)0x00000002)
#define PCR_CACHE_RST_HSRST_CACHE_POS                       1
#define PCR_CACHE_RST_PSRST_CACHE_BIT                       ((uint32_t)0x00000001)
#define PCR_CACHE_RST_PSRST_CACHE_POS                       0

#define PCR_CACHE_RST_HSRST_CACHE_RST                       0x1
#define PCR_CACHE_RST_PSRST_CACHE_RST                       0x1

__INLINE void pcr_cache_rst_pack(uint8_t hsrstcache, uint8_t psrstcache)
{
    _PICO_REG_WR(PCR_CACHE_RST_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)hsrstcache << 1) | ((uint32_t)psrstcache << 0));
}

__INLINE void pcr_cache_rst_unpack(uint8_t* hsrstcache, uint8_t* psrstcache)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_CACHE_RST_OFFSET + PCR_BASE_ADDR);

    *hsrstcache = (localVal & ((uint32_t)0x00000002)) >> 1;
    *psrstcache = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_cache_rst_hsrst_cache_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_CACHE_RST_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_cache_rst_hsrst_cache_setf(uint8_t hsrstcache)
{
    _PICO_REG_WR(PCR_CACHE_RST_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_CACHE_RST_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)hsrstcache << 1));
}

__INLINE uint8_t pcr_cache_rst_psrst_cache_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_CACHE_RST_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_cache_rst_psrst_cache_setf(uint8_t psrstcache)
{
    _PICO_REG_WR(PCR_CACHE_RST_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_CACHE_RST_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)psrstcache << 0));
}

 /**
 * @brief FLH_BUS_SEL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     12     otp_core_pclk_en   0
 *     08          hsrst_otp_n   1
 *     04               bypass   0
 *     00          flh_bus_sel   0
 * </pre>
 */
#define PCR_FLH_BUS_SEL_OFFSET 0x00000028


__INLINE uint32_t pcr_flh_bus_sel_get(void)
{
    return _PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_flh_bus_sel_set(uint32_t value)
{
    _PICO_REG_WR(PCR_FLH_BUS_SEL_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_FLH_BUS_SEL_OTP_CORE_PCLK_EN_BIT                  ((uint32_t)0x00001000)
#define PCR_FLH_BUS_SEL_OTP_CORE_PCLK_EN_POS                  12
#define PCR_FLH_BUS_SEL_HSRST_OTP_N_BIT                       ((uint32_t)0x00000100)
#define PCR_FLH_BUS_SEL_HSRST_OTP_N_POS                       8
#define PCR_FLH_BUS_SEL_BYPASS_BIT                            ((uint32_t)0x00000010)
#define PCR_FLH_BUS_SEL_BYPASS_POS                            4
#define PCR_FLH_BUS_SEL_FLH_BUS_SEL_BIT                       ((uint32_t)0x00000001)
#define PCR_FLH_BUS_SEL_FLH_BUS_SEL_POS                       0

#define PCR_FLH_BUS_SEL_OTP_CORE_PCLK_EN_RST                  0x0
#define PCR_FLH_BUS_SEL_HSRST_OTP_N_RST                       0x1
#define PCR_FLH_BUS_SEL_BYPASS_RST                            0x0
#define PCR_FLH_BUS_SEL_FLH_BUS_SEL_RST                       0x0

__INLINE void pcr_flh_bus_sel_pack(uint8_t otpcorepclken, uint8_t hsrstotpn, uint8_t bypass, uint8_t flhbussel)
{
    _PICO_REG_WR(PCR_FLH_BUS_SEL_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)otpcorepclken << 12) | ((uint32_t)hsrstotpn << 8) | ((uint32_t)bypass << 4) | ((uint32_t)flhbussel << 0));
}

__INLINE void pcr_flh_bus_sel_unpack(uint8_t* otpcorepclken, uint8_t* hsrstotpn, uint8_t* bypass, uint8_t* flhbussel)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR);

    *otpcorepclken = (localVal & ((uint32_t)0x00001000)) >> 12;
    *hsrstotpn = (localVal & ((uint32_t)0x00000100)) >> 8;
    *bypass = (localVal & ((uint32_t)0x00000010)) >> 4;
    *flhbussel = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_flh_bus_sel_otp_core_pclk_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void pcr_flh_bus_sel_otp_core_pclk_en_setf(uint8_t otpcorepclken)
{
    _PICO_REG_WR(PCR_FLH_BUS_SEL_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)otpcorepclken << 12));
}

__INLINE uint8_t pcr_flh_bus_sel_hsrst_otp_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcr_flh_bus_sel_hsrst_otp_n_setf(uint8_t hsrstotpn)
{
    _PICO_REG_WR(PCR_FLH_BUS_SEL_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)hsrstotpn << 8));
}

__INLINE uint8_t pcr_flh_bus_sel_bypass_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcr_flh_bus_sel_bypass_setf(uint8_t bypass)
{
    _PICO_REG_WR(PCR_FLH_BUS_SEL_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)bypass << 4));
}

__INLINE uint8_t pcr_flh_bus_sel_flh_bus_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_flh_bus_sel_flh_bus_sel_setf(uint8_t flhbussel)
{
    _PICO_REG_WR(PCR_FLH_BUS_SEL_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_FLH_BUS_SEL_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)flhbussel << 0));
}

 /**
 * @brief HCLKG_USB register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00            hclkg_usb   0
 * </pre>
 */
#define PCR_HCLKG_USB_OFFSET 0x0000002C


__INLINE uint32_t pcr_hclkg_usb_get(void)
{
    return _PICO_REG_RD(PCR_HCLKG_USB_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_hclkg_usb_set(uint32_t value)
{
    _PICO_REG_WR(PCR_HCLKG_USB_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_HCLKG_USB_HCLKG_USB_BIT                         ((uint32_t)0x00000001)
#define PCR_HCLKG_USB_HCLKG_USB_POS                         0

#define PCR_HCLKG_USB_HCLKG_USB_RST                         0x0

__INLINE void pcr_hclkg_usb_pack(uint8_t hclkgusb)
{
    _PICO_REG_WR(PCR_HCLKG_USB_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)hclkgusb << 0));
}

__INLINE void pcr_hclkg_usb_unpack(uint8_t* hclkgusb)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_HCLKG_USB_OFFSET + PCR_BASE_ADDR);

    *hclkgusb = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_hclkg_usb_hclkg_usb_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_HCLKG_USB_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_hclkg_usb_hclkg_usb_setf(uint8_t hclkgusb)
{
    _PICO_REG_WR(PCR_HCLKG_USB_OFFSET+ PCR_BASE_ADDR, (uint32_t)hclkgusb << 0);
}

 /**
 * @brief HSRTS_USB register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01            rts_usb_n   0
 *     00          hsrts_usb_n   0
 * </pre>
 */
#define PCR_HSRTS_USB_OFFSET 0x00000030


__INLINE uint32_t pcr_hsrts_usb_get(void)
{
    return _PICO_REG_RD(PCR_HSRTS_USB_OFFSET + PCR_BASE_ADDR);
}

__INLINE void pcr_hsrts_usb_set(uint32_t value)
{
    _PICO_REG_WR(PCR_HSRTS_USB_OFFSET+ PCR_BASE_ADDR, value);
}

// field definitions
#define PCR_HSRTS_USB_RTS_USB_N_BIT                         ((uint32_t)0x00000002)
#define PCR_HSRTS_USB_RTS_USB_N_POS                         1
#define PCR_HSRTS_USB_HSRTS_USB_N_BIT                       ((uint32_t)0x00000001)
#define PCR_HSRTS_USB_HSRTS_USB_N_POS                       0

#define PCR_HSRTS_USB_RTS_USB_N_RST                         0x0
#define PCR_HSRTS_USB_HSRTS_USB_N_RST                       0x0

__INLINE void pcr_hsrts_usb_pack(uint8_t rtsusbn, uint8_t hsrtsusbn)
{
    _PICO_REG_WR(PCR_HSRTS_USB_OFFSET+ PCR_BASE_ADDR,  ((uint32_t)rtsusbn << 1) | ((uint32_t)hsrtsusbn << 0));
}

__INLINE void pcr_hsrts_usb_unpack(uint8_t* rtsusbn, uint8_t* hsrtsusbn)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_HSRTS_USB_OFFSET + PCR_BASE_ADDR);

    *rtsusbn = (localVal & ((uint32_t)0x00000002)) >> 1;
    *hsrtsusbn = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcr_hsrts_usb_rts_usb_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_HSRTS_USB_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcr_hsrts_usb_rts_usb_n_setf(uint8_t rtsusbn)
{
    _PICO_REG_WR(PCR_HSRTS_USB_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_HSRTS_USB_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rtsusbn << 1));
}

__INLINE uint8_t pcr_hsrts_usb_hsrts_usb_n_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCR_HSRTS_USB_OFFSET + PCR_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcr_hsrts_usb_hsrts_usb_n_setf(uint8_t hsrtsusbn)
{
    _PICO_REG_WR(PCR_HSRTS_USB_OFFSET+ PCR_BASE_ADDR, (_PICO_REG_RD(PCR_HSRTS_USB_OFFSET + PCR_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)hsrtsusbn << 0));
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :6;
      __IO uint32_t srst_uart2_n:1;
      __IO uint32_t :5;
      __IO uint32_t srst_spif_n:1;
      __IO uint32_t srst_pwm_n:1;
      __IO uint32_t srst_adcc_n:1;
      __IO uint32_t :1;
      __IO uint32_t srst_qdec_n:1;
      __IO uint32_t :1;
      __IO uint32_t srst_gpio_n:1;
      __IO uint32_t srst_spi1_n:1;
      __IO uint32_t srst_spi0_n:1;
      __IO uint32_t srst_i2c1_n:1;
      __IO uint32_t srst_i2c0_n:1;
      __IO uint32_t srst_uart1_n:1;
      __IO uint32_t srst_iomux_n:1;
      __IO uint32_t :1;
      __IO uint32_t srst_timer_n:1;
      __IO uint32_t srst_aes_n:1;
      __IO uint32_t srst_dma_n:1;
      __IO uint32_t wdt_reset_en:1;
      __IO uint32_t m0_lockup_reset_en:1;
      __IO uint32_t ck802_soft_reset:1;
    }sw_reset0_fld;
    __IO uint32_t sw_reset0;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :29;
      __IO uint32_t all_srst_n_pls:1;
      __IO uint32_t cpu_srst_n_pls:1;
      __IO uint32_t sys_srst_n_pls:1;
    }sw_reset1_fld;
    __IO uint32_t sw_reset1;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :6;
      __IO uint32_t clkg_uart2:1;
      __IO uint32_t :2;
      __IO uint32_t clkg_timer6:1;
      __IO uint32_t clkg_timer5:1;
      __IO uint32_t :1;
      __IO uint32_t clkg_spif:1;
      __IO uint32_t clkg_pwm:1;
      __IO uint32_t clkg_adcc:1;
      __IO uint32_t :1;
      __IO uint32_t clkg_qdec:1;
      __IO uint32_t :1;
      __IO uint32_t clkg_gpio:1;
      __IO uint32_t clkg_spi1:1;
      __IO uint32_t clkg_spi0:1;
      __IO uint32_t clkg_i2c1:1;
      __IO uint32_t clkg_i2c0:1;
      __IO uint32_t clkg_uart:1;
      __IO uint32_t clkg_iomux:1;
      __IO uint32_t :2;
      __IO uint32_t clkg_aes:1;
      __IO uint32_t clkg_dma:1;
      __IO uint32_t :2;
      __IO uint32_t ck802_clkg_cpu:1;
    }sw_clk_fld;
    __IO uint32_t sw_clk;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :18;
      __IO uint32_t srst_bbmix_n:1;
      __IO uint32_t srst_bbrx_n:1;
      __IO uint32_t srst_bbtx_n:1;
      __IO uint32_t srst_bbll_n:1;
      __IO uint32_t srst_bbreg_n:1;
      __IO uint32_t m0_enabled_by_m4:1;
      __IO uint32_t srst_ks_n:1;
      __IO uint32_t srst_com_n:1;
      __IO uint32_t srst_wdt_n:1;
      __IO uint32_t srst_timer_n:1;
      __IO uint32_t srst_bb_n:1;
      __IO uint32_t wdt_reset_en:1;
      __IO uint32_t m0_lockup_reset_en:1;
      __IO uint32_t m0_soft_reset:1;
    }sw_reset2_fld;
    __IO uint32_t sw_reset2;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :30;
      __IO uint32_t cpu_srst_n_pls:1;
      __IO uint32_t sys_srst_n_pls:1;
    }sw_reset3_fld;
    __IO uint32_t sw_reset3;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :7;
      __IO uint32_t clkg_timer4:1;
      __IO uint32_t clkg_timer3:1;
      __IO uint32_t clkg_timer2:1;
      __IO uint32_t clkg_timer1:1;
      __IO uint32_t :11;
      __IO uint32_t clkg_bbreg:1;
      __IO uint32_t :1;
      __IO uint32_t clkg_ks:1;
      __IO uint32_t clkg_com:1;
      __IO uint32_t clkg_wdt:1;
      __IO uint32_t clkg_timer:1;
      __IO uint32_t clkg_bb:1;
      __IO uint32_t :2;
      __IO uint32_t m0_clkg_cpu:1;
    }sw_clk1_fld;
    __IO uint32_t sw_clk1;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :24;
      __IO uint32_t pclk_div_m0:4;
      __IO uint32_t :4;
    }apb_clk_fld;
    __IO uint32_t apb_clk;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :30;
      __IO uint32_t pclk_update_1:1;
      __IO uint32_t pclk_update_0:1;
    }apb_clk_update_fld;
    __IO uint32_t apb_clk_update;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t :30;
      __IO uint32_t pclkg_cache:1;
      __IO uint32_t hclkg_cache:1;
    }cache_clock_gate_fld;
    __IO uint32_t cache_clock_gate;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :30;
      __IO uint32_t hsrst_cache:1;
      __IO uint32_t psrst_cache:1;
    }cache_rst_fld;
    __IO uint32_t cache_rst;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t :19;
      __IO uint32_t otp_core_pclk_en:1;
      __IO uint32_t :3;
      __IO uint32_t hsrst_otp_n:1;
      __IO uint32_t :3;
      __IO uint32_t bypass:1;
      __IO uint32_t :3;
      __IO uint32_t flh_bus_sel:1;
    }flh_bus_sel_fld;
    __IO uint32_t flh_bus_sel;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t :31;
      __IO uint32_t hclkg_usb:1;
    }hclkg_usb_fld;
    __IO uint32_t hclkg_usb;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :30;
      __IO uint32_t rts_usb_n:1;
      __IO uint32_t hsrts_usb_n:1;
    }hsrts_usb_fld;
    __IO uint32_t hsrts_usb;
  };

} PICO_REG_PCR_TypeDef;

#define PICO_REG_PCR PICO_REG_PCR_TypeDef *0x40000000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_PCR_H_

