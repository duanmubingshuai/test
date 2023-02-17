#ifndef _PICO_REG_AON_H_
#define _PICO_REG_AON_H_

#include <stdint.h>

#define AON_COUNT 27

#define AON_BASE_ADDR 0x4000F000

#define AON_SIZE 0x000000CC


 /**
 * @brief PWROFF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               PWROFF   0b0
 * </pre>
 */
#define AON_PWROFF_OFFSET 0x00000000


__INLINE uint32_t aon_pwroff_get(void)
{
    return _PICO_REG_RD(AON_PWROFF_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_pwroff_set(uint32_t value)
{
    _PICO_REG_WR(AON_PWROFF_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_PWROFF_PWROFF_MASK                           ((uint32_t)0xFFFFFFFF)
#define AON_PWROFF_PWROFF_LSB                            0
#define AON_PWROFF_PWROFF_WIDTH                          ((uint32_t)0x00000020)

#define AON_PWROFF_PWROFF_RST                            0x0

__INLINE void aon_pwroff_pack(uint32_t pwroff)
{
    _PICO_REG_WR(AON_PWROFF_OFFSET+ AON_BASE_ADDR,  ((uint32_t)pwroff << 0));
}

__INLINE void aon_pwroff_unpack(uint8_t* pwroff)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PWROFF_OFFSET + AON_BASE_ADDR);

    *pwroff = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aon_pwroff_pwroff_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PWROFF_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aon_pwroff_pwroff_setf(uint32_t pwroff)
{
    _PICO_REG_WR(AON_PWROFF_OFFSET+ AON_BASE_ADDR, (uint32_t)pwroff << 0);
}

 /**
 * @brief PWRSLP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               PWRSLP   0b0
 * </pre>
 */
#define AON_PWRSLP_OFFSET 0x00000004


__INLINE uint32_t aon_pwrslp_get(void)
{
    return _PICO_REG_RD(AON_PWRSLP_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_pwrslp_set(uint32_t value)
{
    _PICO_REG_WR(AON_PWRSLP_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_PWRSLP_PWRSLP_MASK                           ((uint32_t)0xFFFFFFFF)
#define AON_PWRSLP_PWRSLP_LSB                            0
#define AON_PWRSLP_PWRSLP_WIDTH                          ((uint32_t)0x00000020)

#define AON_PWRSLP_PWRSLP_RST                            0x0

__INLINE void aon_pwrslp_pack(uint32_t pwrslp)
{
    _PICO_REG_WR(AON_PWRSLP_OFFSET+ AON_BASE_ADDR,  ((uint32_t)pwrslp << 0));
}

__INLINE void aon_pwrslp_unpack(uint8_t* pwrslp)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PWRSLP_OFFSET + AON_BASE_ADDR);

    *pwrslp = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aon_pwrslp_pwrslp_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PWRSLP_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aon_pwrslp_pwrslp_setf(uint32_t pwrslp)
{
    _PICO_REG_WR(AON_PWRSLP_OFFSET+ AON_BASE_ADDR, (uint32_t)pwrslp << 0);
}

 /**
 * @brief IOCTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:28              p09_pul   0b0
 *     27             p09_wu_s   0
 *  26:25              p08_pul   0b11
 *     24             p08_wu_s   0
 *  23:22              p07_pul   0b0
 *     21             p07_wu_s   0
 *  20:19              p06_pul   0b0
 *     18             p06_wu_s   0
 *  17:16              p05_pul   0b0
 *     15             p05_wu_s   0
 *  14:13              p04_pul   0b0
 *     12             p04_wu_s   0
 *  11:10              p03_pul   0b11
 *     09             p03_wu_s   0
 *  08:07              p02_pul   0b0
 *     06             p02_wu_s   0
 *  05:04              p01_pul   0b0
 *     03             p01_wu_s   0
 *  02:01              p00_pul   0b0
 *     00             p00_wu_s   0
 * </pre>
 */
#define AON_IOCTL0_OFFSET 0x00000008


__INLINE uint32_t aon_ioctl0_get(void)
{
    return _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_ioctl0_set(uint32_t value)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_IOCTL0_P09_PUL_MASK                          ((uint32_t)0x30000000)
#define AON_IOCTL0_P09_PUL_LSB                           28
#define AON_IOCTL0_P09_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P09_WU_S_BIT                          ((uint32_t)0x08000000)
#define AON_IOCTL0_P09_WU_S_POS                          27
#define AON_IOCTL0_P08_PUL_MASK                          ((uint32_t)0x06000000)
#define AON_IOCTL0_P08_PUL_LSB                           25
#define AON_IOCTL0_P08_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P08_WU_S_BIT                          ((uint32_t)0x01000000)
#define AON_IOCTL0_P08_WU_S_POS                          24
#define AON_IOCTL0_P07_PUL_MASK                          ((uint32_t)0x00C00000)
#define AON_IOCTL0_P07_PUL_LSB                           22
#define AON_IOCTL0_P07_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P07_WU_S_BIT                          ((uint32_t)0x00200000)
#define AON_IOCTL0_P07_WU_S_POS                          21
#define AON_IOCTL0_P06_PUL_MASK                          ((uint32_t)0x00180000)
#define AON_IOCTL0_P06_PUL_LSB                           19
#define AON_IOCTL0_P06_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P06_WU_S_BIT                          ((uint32_t)0x00040000)
#define AON_IOCTL0_P06_WU_S_POS                          18
#define AON_IOCTL0_P05_PUL_MASK                          ((uint32_t)0x00030000)
#define AON_IOCTL0_P05_PUL_LSB                           16
#define AON_IOCTL0_P05_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P05_WU_S_BIT                          ((uint32_t)0x00008000)
#define AON_IOCTL0_P05_WU_S_POS                          15
#define AON_IOCTL0_P04_PUL_MASK                          ((uint32_t)0x00006000)
#define AON_IOCTL0_P04_PUL_LSB                           13
#define AON_IOCTL0_P04_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P04_WU_S_BIT                          ((uint32_t)0x00001000)
#define AON_IOCTL0_P04_WU_S_POS                          12
#define AON_IOCTL0_P03_PUL_MASK                          ((uint32_t)0x00000C00)
#define AON_IOCTL0_P03_PUL_LSB                           10
#define AON_IOCTL0_P03_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P03_WU_S_BIT                          ((uint32_t)0x00000200)
#define AON_IOCTL0_P03_WU_S_POS                          9
#define AON_IOCTL0_P02_PUL_MASK                          ((uint32_t)0x00000180)
#define AON_IOCTL0_P02_PUL_LSB                           7
#define AON_IOCTL0_P02_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P02_WU_S_BIT                          ((uint32_t)0x00000040)
#define AON_IOCTL0_P02_WU_S_POS                          6
#define AON_IOCTL0_P01_PUL_MASK                          ((uint32_t)0x00000030)
#define AON_IOCTL0_P01_PUL_LSB                           4
#define AON_IOCTL0_P01_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P01_WU_S_BIT                          ((uint32_t)0x00000008)
#define AON_IOCTL0_P01_WU_S_POS                          3
#define AON_IOCTL0_P00_PUL_MASK                          ((uint32_t)0x00000006)
#define AON_IOCTL0_P00_PUL_LSB                           1
#define AON_IOCTL0_P00_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL0_P00_WU_S_BIT                          ((uint32_t)0x00000001)
#define AON_IOCTL0_P00_WU_S_POS                          0

#define AON_IOCTL0_P09_PUL_RST                           0x0
#define AON_IOCTL0_P09_WU_S_RST                          0x0
#define AON_IOCTL0_P08_PUL_RST                           0x11
#define AON_IOCTL0_P08_WU_S_RST                          0x0
#define AON_IOCTL0_P07_PUL_RST                           0x0
#define AON_IOCTL0_P07_WU_S_RST                          0x0
#define AON_IOCTL0_P06_PUL_RST                           0x0
#define AON_IOCTL0_P06_WU_S_RST                          0x0
#define AON_IOCTL0_P05_PUL_RST                           0x0
#define AON_IOCTL0_P05_WU_S_RST                          0x0
#define AON_IOCTL0_P04_PUL_RST                           0x0
#define AON_IOCTL0_P04_WU_S_RST                          0x0
#define AON_IOCTL0_P03_PUL_RST                           0x11
#define AON_IOCTL0_P03_WU_S_RST                          0x0
#define AON_IOCTL0_P02_PUL_RST                           0x0
#define AON_IOCTL0_P02_WU_S_RST                          0x0
#define AON_IOCTL0_P01_PUL_RST                           0x0
#define AON_IOCTL0_P01_WU_S_RST                          0x0
#define AON_IOCTL0_P00_PUL_RST                           0x0
#define AON_IOCTL0_P00_WU_S_RST                          0x0

__INLINE void aon_ioctl0_pack(uint8_t p09pul, uint8_t p09wus, uint8_t p08pul, uint8_t p08wus, uint8_t p07pul, uint8_t p07wus, uint8_t p06pul, uint8_t p06wus, uint8_t p05pul, uint8_t p05wus, uint8_t p04pul, uint8_t p04wus, uint8_t p03pul, uint8_t p03wus, uint8_t p02pul, uint8_t p02wus, uint8_t p01pul, uint8_t p01wus, uint8_t p00pul, uint8_t p00wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR,  ((uint32_t)p09pul << 28) | ((uint32_t)p09wus << 27) | ((uint32_t)p08pul << 25) | ((uint32_t)p08wus << 24) | ((uint32_t)p07pul << 22) | ((uint32_t)p07wus << 21) | ((uint32_t)p06pul << 19) | ((uint32_t)p06wus << 18) | ((uint32_t)p05pul << 16) | ((uint32_t)p05wus << 15) | ((uint32_t)p04pul << 13) | ((uint32_t)p04wus << 12) | ((uint32_t)p03pul << 10) | ((uint32_t)p03wus << 9) | ((uint32_t)p02pul << 7) | ((uint32_t)p02wus << 6) | ((uint32_t)p01pul << 4) | ((uint32_t)p01wus << 3) | ((uint32_t)p00pul << 1) | ((uint32_t)p00wus << 0));
}

__INLINE void aon_ioctl0_unpack(uint8_t* p09pul, uint8_t* p09wus, uint8_t* p08pul, uint8_t* p08wus, uint8_t* p07pul, uint8_t* p07wus, uint8_t* p06pul, uint8_t* p06wus, uint8_t* p05pul, uint8_t* p05wus, uint8_t* p04pul, uint8_t* p04wus, uint8_t* p03pul, uint8_t* p03wus, uint8_t* p02pul, uint8_t* p02wus, uint8_t* p01pul, uint8_t* p01wus, uint8_t* p00pul, uint8_t* p00wus)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);

    *p09pul = (localVal & ((uint32_t)0x30000000)) >> 28;
    *p09wus = (localVal & ((uint32_t)0x08000000)) >> 27;
    *p08pul = (localVal & ((uint32_t)0x06000000)) >> 25;
    *p08wus = (localVal & ((uint32_t)0x01000000)) >> 24;
    *p07pul = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *p07wus = (localVal & ((uint32_t)0x00200000)) >> 21;
    *p06pul = (localVal & ((uint32_t)0x00180000)) >> 19;
    *p06wus = (localVal & ((uint32_t)0x00040000)) >> 18;
    *p05pul = (localVal & ((uint32_t)0x00030000)) >> 16;
    *p05wus = (localVal & ((uint32_t)0x00008000)) >> 15;
    *p04pul = (localVal & ((uint32_t)0x00006000)) >> 13;
    *p04wus = (localVal & ((uint32_t)0x00001000)) >> 12;
    *p03pul = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *p03wus = (localVal & ((uint32_t)0x00000200)) >> 9;
    *p02pul = (localVal & ((uint32_t)0x00000180)) >> 7;
    *p02wus = (localVal & ((uint32_t)0x00000040)) >> 6;
    *p01pul = (localVal & ((uint32_t)0x00000030)) >> 4;
    *p01wus = (localVal & ((uint32_t)0x00000008)) >> 3;
    *p00pul = (localVal & ((uint32_t)0x00000006)) >> 1;
    *p00wus = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t aon_ioctl0_p09_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void aon_ioctl0_p09_pul_setf(uint8_t p09pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)p09pul << 28));
}

__INLINE uint8_t aon_ioctl0_p09_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void aon_ioctl0_p09_wu_s_setf(uint8_t p09wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)p09wus << 27));
}

__INLINE uint8_t aon_ioctl0_p08_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x06000000)) >> 25);
}

__INLINE void aon_ioctl0_p08_pul_setf(uint8_t p08pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x06000000)) | ((uint32_t)p08pul << 25));
}

__INLINE uint8_t aon_ioctl0_p08_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void aon_ioctl0_p08_wu_s_setf(uint8_t p08wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)p08wus << 24));
}

__INLINE uint8_t aon_ioctl0_p07_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE void aon_ioctl0_p07_pul_setf(uint8_t p07pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00C00000)) | ((uint32_t)p07pul << 22));
}

__INLINE uint8_t aon_ioctl0_p07_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void aon_ioctl0_p07_wu_s_setf(uint8_t p07wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)p07wus << 21));
}

__INLINE uint8_t aon_ioctl0_p06_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00180000)) >> 19);
}

__INLINE void aon_ioctl0_p06_pul_setf(uint8_t p06pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00180000)) | ((uint32_t)p06pul << 19));
}

__INLINE uint8_t aon_ioctl0_p06_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void aon_ioctl0_p06_wu_s_setf(uint8_t p06wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)p06wus << 18));
}

__INLINE uint8_t aon_ioctl0_p05_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void aon_ioctl0_p05_pul_setf(uint8_t p05pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)p05pul << 16));
}

__INLINE uint8_t aon_ioctl0_p05_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void aon_ioctl0_p05_wu_s_setf(uint8_t p05wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)p05wus << 15));
}

__INLINE uint8_t aon_ioctl0_p04_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00006000)) >> 13);
}

__INLINE void aon_ioctl0_p04_pul_setf(uint8_t p04pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00006000)) | ((uint32_t)p04pul << 13));
}

__INLINE uint8_t aon_ioctl0_p04_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void aon_ioctl0_p04_wu_s_setf(uint8_t p04wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)p04wus << 12));
}

__INLINE uint8_t aon_ioctl0_p03_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void aon_ioctl0_p03_pul_setf(uint8_t p03pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)p03pul << 10));
}

__INLINE uint8_t aon_ioctl0_p03_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void aon_ioctl0_p03_wu_s_setf(uint8_t p03wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)p03wus << 9));
}

__INLINE uint8_t aon_ioctl0_p02_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000180)) >> 7);
}

__INLINE void aon_ioctl0_p02_pul_setf(uint8_t p02pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000180)) | ((uint32_t)p02pul << 7));
}

__INLINE uint8_t aon_ioctl0_p02_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void aon_ioctl0_p02_wu_s_setf(uint8_t p02wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)p02wus << 6));
}

__INLINE uint8_t aon_ioctl0_p01_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void aon_ioctl0_p01_pul_setf(uint8_t p01pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)p01pul << 4));
}

__INLINE uint8_t aon_ioctl0_p01_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void aon_ioctl0_p01_wu_s_setf(uint8_t p01wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)p01wus << 3));
}

__INLINE uint8_t aon_ioctl0_p00_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000006)) >> 1);
}

__INLINE void aon_ioctl0_p00_pul_setf(uint8_t p00pul)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000006)) | ((uint32_t)p00pul << 1));
}

__INLINE uint8_t aon_ioctl0_p00_wu_s_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void aon_ioctl0_p00_wu_s_setf(uint8_t p00wus)
{
    _PICO_REG_WR(AON_IOCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)p00wus << 0));
}

 /**
 * @brief IOCTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:28              p19_pul   0b0
 *     27          p19_wake_up   0
 *  26:25              p18_pul   0b0
 *     24          p18_wake_up   0
 *  23:22              p17_pul   0b0
 *     21          p17_wake_up   0
 *  20:19              p16_pul   0b0
 *     18          p16_wake_up   0
 *  17:16              p15_pul   0b0
 *     15          p15_wake_up   0
 *  14:13              p14_pul   0b0
 *     12          p14_wake_up   0
 *  11:10              p13_pul   0b0
 *     09          p13_wake_up   0
 *  08:07              p12_pul   0b0
 *     06          p12_wake_up   0
 *  05:04              p11_pul   0b0
 *     03          p11_wake_up   0
 *  02:01              p10_pul   0b0
 *     00          p10_wake_up   0
 * </pre>
 */
#define AON_IOCTL1_OFFSET 0x0000000C


__INLINE uint32_t aon_ioctl1_get(void)
{
    return _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_ioctl1_set(uint32_t value)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_IOCTL1_P19_PUL_MASK                          ((uint32_t)0x30000000)
#define AON_IOCTL1_P19_PUL_LSB                           28
#define AON_IOCTL1_P19_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P19_WAKE_UP_BIT                       ((uint32_t)0x08000000)
#define AON_IOCTL1_P19_WAKE_UP_POS                       27
#define AON_IOCTL1_P18_PUL_MASK                          ((uint32_t)0x06000000)
#define AON_IOCTL1_P18_PUL_LSB                           25
#define AON_IOCTL1_P18_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P18_WAKE_UP_BIT                       ((uint32_t)0x01000000)
#define AON_IOCTL1_P18_WAKE_UP_POS                       24
#define AON_IOCTL1_P17_PUL_MASK                          ((uint32_t)0x00C00000)
#define AON_IOCTL1_P17_PUL_LSB                           22
#define AON_IOCTL1_P17_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P17_WAKE_UP_BIT                       ((uint32_t)0x00200000)
#define AON_IOCTL1_P17_WAKE_UP_POS                       21
#define AON_IOCTL1_P16_PUL_MASK                          ((uint32_t)0x00180000)
#define AON_IOCTL1_P16_PUL_LSB                           19
#define AON_IOCTL1_P16_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P16_WAKE_UP_BIT                       ((uint32_t)0x00040000)
#define AON_IOCTL1_P16_WAKE_UP_POS                       18
#define AON_IOCTL1_P15_PUL_MASK                          ((uint32_t)0x00030000)
#define AON_IOCTL1_P15_PUL_LSB                           16
#define AON_IOCTL1_P15_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P15_WAKE_UP_BIT                       ((uint32_t)0x00008000)
#define AON_IOCTL1_P15_WAKE_UP_POS                       15
#define AON_IOCTL1_P14_PUL_MASK                          ((uint32_t)0x00006000)
#define AON_IOCTL1_P14_PUL_LSB                           13
#define AON_IOCTL1_P14_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P14_WAKE_UP_BIT                       ((uint32_t)0x00001000)
#define AON_IOCTL1_P14_WAKE_UP_POS                       12
#define AON_IOCTL1_P13_PUL_MASK                          ((uint32_t)0x00000C00)
#define AON_IOCTL1_P13_PUL_LSB                           10
#define AON_IOCTL1_P13_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P13_WAKE_UP_BIT                       ((uint32_t)0x00000200)
#define AON_IOCTL1_P13_WAKE_UP_POS                       9
#define AON_IOCTL1_P12_PUL_MASK                          ((uint32_t)0x00000180)
#define AON_IOCTL1_P12_PUL_LSB                           7
#define AON_IOCTL1_P12_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P12_WAKE_UP_BIT                       ((uint32_t)0x00000040)
#define AON_IOCTL1_P12_WAKE_UP_POS                       6
#define AON_IOCTL1_P11_PUL_MASK                          ((uint32_t)0x00000030)
#define AON_IOCTL1_P11_PUL_LSB                           4
#define AON_IOCTL1_P11_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P11_WAKE_UP_BIT                       ((uint32_t)0x00000008)
#define AON_IOCTL1_P11_WAKE_UP_POS                       3
#define AON_IOCTL1_P10_PUL_MASK                          ((uint32_t)0x00000006)
#define AON_IOCTL1_P10_PUL_LSB                           1
#define AON_IOCTL1_P10_PUL_WIDTH                         ((uint32_t)0x00000002)
#define AON_IOCTL1_P10_WAKE_UP_BIT                       ((uint32_t)0x00000001)
#define AON_IOCTL1_P10_WAKE_UP_POS                       0

#define AON_IOCTL1_P19_PUL_RST                           0x0
#define AON_IOCTL1_P19_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P18_PUL_RST                           0x0
#define AON_IOCTL1_P18_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P17_PUL_RST                           0x0
#define AON_IOCTL1_P17_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P16_PUL_RST                           0x0
#define AON_IOCTL1_P16_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P15_PUL_RST                           0x0
#define AON_IOCTL1_P15_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P14_PUL_RST                           0x0
#define AON_IOCTL1_P14_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P13_PUL_RST                           0x0
#define AON_IOCTL1_P13_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P12_PUL_RST                           0x0
#define AON_IOCTL1_P12_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P11_PUL_RST                           0x0
#define AON_IOCTL1_P11_WAKE_UP_RST                       0x0
#define AON_IOCTL1_P10_PUL_RST                           0x0
#define AON_IOCTL1_P10_WAKE_UP_RST                       0x0

__INLINE void aon_ioctl1_pack(uint8_t p19pul, uint8_t p19wakeup, uint8_t p18pul, uint8_t p18wakeup, uint8_t p17pul, uint8_t p17wakeup, uint8_t p16pul, uint8_t p16wakeup, uint8_t p15pul, uint8_t p15wakeup, uint8_t p14pul, uint8_t p14wakeup, uint8_t p13pul, uint8_t p13wakeup, uint8_t p12pul, uint8_t p12wakeup, uint8_t p11pul, uint8_t p11wakeup, uint8_t p10pul, uint8_t p10wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR,  ((uint32_t)p19pul << 28) | ((uint32_t)p19wakeup << 27) | ((uint32_t)p18pul << 25) | ((uint32_t)p18wakeup << 24) | ((uint32_t)p17pul << 22) | ((uint32_t)p17wakeup << 21) | ((uint32_t)p16pul << 19) | ((uint32_t)p16wakeup << 18) | ((uint32_t)p15pul << 16) | ((uint32_t)p15wakeup << 15) | ((uint32_t)p14pul << 13) | ((uint32_t)p14wakeup << 12) | ((uint32_t)p13pul << 10) | ((uint32_t)p13wakeup << 9) | ((uint32_t)p12pul << 7) | ((uint32_t)p12wakeup << 6) | ((uint32_t)p11pul << 4) | ((uint32_t)p11wakeup << 3) | ((uint32_t)p10pul << 1) | ((uint32_t)p10wakeup << 0));
}

__INLINE void aon_ioctl1_unpack(uint8_t* p19pul, uint8_t* p19wakeup, uint8_t* p18pul, uint8_t* p18wakeup, uint8_t* p17pul, uint8_t* p17wakeup, uint8_t* p16pul, uint8_t* p16wakeup, uint8_t* p15pul, uint8_t* p15wakeup, uint8_t* p14pul, uint8_t* p14wakeup, uint8_t* p13pul, uint8_t* p13wakeup, uint8_t* p12pul, uint8_t* p12wakeup, uint8_t* p11pul, uint8_t* p11wakeup, uint8_t* p10pul, uint8_t* p10wakeup)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);

    *p19pul = (localVal & ((uint32_t)0x30000000)) >> 28;
    *p19wakeup = (localVal & ((uint32_t)0x08000000)) >> 27;
    *p18pul = (localVal & ((uint32_t)0x06000000)) >> 25;
    *p18wakeup = (localVal & ((uint32_t)0x01000000)) >> 24;
    *p17pul = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *p17wakeup = (localVal & ((uint32_t)0x00200000)) >> 21;
    *p16pul = (localVal & ((uint32_t)0x00180000)) >> 19;
    *p16wakeup = (localVal & ((uint32_t)0x00040000)) >> 18;
    *p15pul = (localVal & ((uint32_t)0x00030000)) >> 16;
    *p15wakeup = (localVal & ((uint32_t)0x00008000)) >> 15;
    *p14pul = (localVal & ((uint32_t)0x00006000)) >> 13;
    *p14wakeup = (localVal & ((uint32_t)0x00001000)) >> 12;
    *p13pul = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *p13wakeup = (localVal & ((uint32_t)0x00000200)) >> 9;
    *p12pul = (localVal & ((uint32_t)0x00000180)) >> 7;
    *p12wakeup = (localVal & ((uint32_t)0x00000040)) >> 6;
    *p11pul = (localVal & ((uint32_t)0x00000030)) >> 4;
    *p11wakeup = (localVal & ((uint32_t)0x00000008)) >> 3;
    *p10pul = (localVal & ((uint32_t)0x00000006)) >> 1;
    *p10wakeup = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t aon_ioctl1_p19_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x30000000)) >> 28);
}

__INLINE void aon_ioctl1_p19_pul_setf(uint8_t p19pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x30000000)) | ((uint32_t)p19pul << 28));
}

__INLINE uint8_t aon_ioctl1_p19_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void aon_ioctl1_p19_wake_up_setf(uint8_t p19wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)p19wakeup << 27));
}

__INLINE uint8_t aon_ioctl1_p18_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x06000000)) >> 25);
}

__INLINE void aon_ioctl1_p18_pul_setf(uint8_t p18pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x06000000)) | ((uint32_t)p18pul << 25));
}

__INLINE uint8_t aon_ioctl1_p18_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void aon_ioctl1_p18_wake_up_setf(uint8_t p18wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)p18wakeup << 24));
}

__INLINE uint8_t aon_ioctl1_p17_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE void aon_ioctl1_p17_pul_setf(uint8_t p17pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00C00000)) | ((uint32_t)p17pul << 22));
}

__INLINE uint8_t aon_ioctl1_p17_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void aon_ioctl1_p17_wake_up_setf(uint8_t p17wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)p17wakeup << 21));
}

__INLINE uint8_t aon_ioctl1_p16_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00180000)) >> 19);
}

__INLINE void aon_ioctl1_p16_pul_setf(uint8_t p16pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00180000)) | ((uint32_t)p16pul << 19));
}

__INLINE uint8_t aon_ioctl1_p16_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void aon_ioctl1_p16_wake_up_setf(uint8_t p16wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)p16wakeup << 18));
}

__INLINE uint8_t aon_ioctl1_p15_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void aon_ioctl1_p15_pul_setf(uint8_t p15pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)p15pul << 16));
}

__INLINE uint8_t aon_ioctl1_p15_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void aon_ioctl1_p15_wake_up_setf(uint8_t p15wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)p15wakeup << 15));
}

__INLINE uint8_t aon_ioctl1_p14_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00006000)) >> 13);
}

__INLINE void aon_ioctl1_p14_pul_setf(uint8_t p14pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00006000)) | ((uint32_t)p14pul << 13));
}

__INLINE uint8_t aon_ioctl1_p14_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void aon_ioctl1_p14_wake_up_setf(uint8_t p14wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)p14wakeup << 12));
}

__INLINE uint8_t aon_ioctl1_p13_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void aon_ioctl1_p13_pul_setf(uint8_t p13pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)p13pul << 10));
}

__INLINE uint8_t aon_ioctl1_p13_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void aon_ioctl1_p13_wake_up_setf(uint8_t p13wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)p13wakeup << 9));
}

__INLINE uint8_t aon_ioctl1_p12_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000180)) >> 7);
}

__INLINE void aon_ioctl1_p12_pul_setf(uint8_t p12pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000180)) | ((uint32_t)p12pul << 7));
}

__INLINE uint8_t aon_ioctl1_p12_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void aon_ioctl1_p12_wake_up_setf(uint8_t p12wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)p12wakeup << 6));
}

__INLINE uint8_t aon_ioctl1_p11_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void aon_ioctl1_p11_pul_setf(uint8_t p11pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)p11pul << 4));
}

__INLINE uint8_t aon_ioctl1_p11_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void aon_ioctl1_p11_wake_up_setf(uint8_t p11wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)p11wakeup << 3));
}

__INLINE uint8_t aon_ioctl1_p10_pul_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000006)) >> 1);
}

__INLINE void aon_ioctl1_p10_pul_setf(uint8_t p10pul)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000006)) | ((uint32_t)p10pul << 1));
}

__INLINE uint8_t aon_ioctl1_p10_wake_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void aon_ioctl1_p10_wake_up_setf(uint8_t p10wakeup)
{
    _PICO_REG_WR(AON_IOCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_IOCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)p10wakeup << 0));
}

 /**
 * @brief IOCTL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00            io_ret_en   0b0
 * </pre>
 */
#define AON_IOCTL2_OFFSET 0x00000010


__INLINE uint32_t aon_ioctl2_get(void)
{
    return _PICO_REG_RD(AON_IOCTL2_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_ioctl2_set(uint32_t value)
{
    _PICO_REG_WR(AON_IOCTL2_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_IOCTL2_IO_RET_EN_MASK                        ((uint32_t)0x000FFFFF)
#define AON_IOCTL2_IO_RET_EN_LSB                         0
#define AON_IOCTL2_IO_RET_EN_WIDTH                       ((uint32_t)0x00000014)

#define AON_IOCTL2_IO_RET_EN_RST                         0x0

__INLINE void aon_ioctl2_pack(uint32_t ioreten)
{
    _PICO_REG_WR(AON_IOCTL2_OFFSET+ AON_BASE_ADDR,  ((uint32_t)ioreten << 0));
}

__INLINE void aon_ioctl2_unpack(uint8_t* ioreten)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL2_OFFSET + AON_BASE_ADDR);

    *ioreten = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t aon_ioctl2_io_ret_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IOCTL2_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void aon_ioctl2_io_ret_en_setf(uint32_t ioreten)
{
    _PICO_REG_WR(AON_IOCTL2_OFFSET+ AON_BASE_ADDR, (uint32_t)ioreten << 0);
}

 /**
 * @brief PMCTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31          rtc_clk_sel   0
 *     30         slp_en_lpcmp   0
 *     29           slp_en_rtc   0
 *     28         slp_en_xt32k   0
 *     27         slp_en_rc32k   0
 *     26       lcldo_out_trim   0
 *  22:21         dig_ldo_trim   0b1
 *  20:00             reserved   0b0
 * </pre>
 */
#define AON_PMCTL0_OFFSET 0x00000014


__INLINE uint32_t aon_pmctl0_get(void)
{
    return _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_pmctl0_set(uint32_t value)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_PMCTL0_RTC_CLK_SEL_BIT                       ((uint32_t)0x80000000)
#define AON_PMCTL0_RTC_CLK_SEL_POS                       31
#define AON_PMCTL0_SLP_EN_LPCMP_BIT                      ((uint32_t)0x40000000)
#define AON_PMCTL0_SLP_EN_LPCMP_POS                      30
#define AON_PMCTL0_SLP_EN_RTC_BIT                        ((uint32_t)0x20000000)
#define AON_PMCTL0_SLP_EN_RTC_POS                        29
#define AON_PMCTL0_SLP_EN_XT32K_BIT                      ((uint32_t)0x10000000)
#define AON_PMCTL0_SLP_EN_XT32K_POS                      28
#define AON_PMCTL0_SLP_EN_RC32K_BIT                      ((uint32_t)0x08000000)
#define AON_PMCTL0_SLP_EN_RC32K_POS                      27
#define AON_PMCTL0_LCLDO_OUT_TRIM_BIT                    ((uint32_t)0x04000000)
#define AON_PMCTL0_LCLDO_OUT_TRIM_POS                    26
#define AON_PMCTL0_DIG_LDO_TRIM_MASK                     ((uint32_t)0x00600000)
#define AON_PMCTL0_DIG_LDO_TRIM_LSB                      21
#define AON_PMCTL0_DIG_LDO_TRIM_WIDTH                    ((uint32_t)0x00000002)
#define AON_PMCTL0_RESERVED_MASK                         ((uint32_t)0x001FFFFF)
#define AON_PMCTL0_RESERVED_LSB                          0
#define AON_PMCTL0_RESERVED_WIDTH                        ((uint32_t)0x00000015)

#define AON_PMCTL0_RTC_CLK_SEL_RST                       0x0
#define AON_PMCTL0_SLP_EN_LPCMP_RST                      0x0
#define AON_PMCTL0_SLP_EN_RTC_RST                        0x0
#define AON_PMCTL0_SLP_EN_XT32K_RST                      0x0
#define AON_PMCTL0_SLP_EN_RC32K_RST                      0x0
#define AON_PMCTL0_LCLDO_OUT_TRIM_RST                    0x0
#define AON_PMCTL0_DIG_LDO_TRIM_RST                      0x1
#define AON_PMCTL0_RESERVED_RST                          0x0

__INLINE void aon_pmctl0_pack(uint8_t rtcclksel, uint8_t slpenlpcmp, uint8_t slpenrtc, uint8_t slpenxt32k, uint8_t slpenrc32k, uint8_t lcldoouttrim, uint8_t digldotrim)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR,  ((uint32_t)rtcclksel << 31) | ((uint32_t)slpenlpcmp << 30) | ((uint32_t)slpenrtc << 29) | ((uint32_t)slpenxt32k << 28) | ((uint32_t)slpenrc32k << 27) | ((uint32_t)lcldoouttrim << 26) | ((uint32_t)digldotrim << 21));
}

__INLINE void aon_pmctl0_unpack(uint8_t* rtcclksel, uint8_t* slpenlpcmp, uint8_t* slpenrtc, uint8_t* slpenxt32k, uint8_t* slpenrc32k, uint8_t* lcldoouttrim, uint8_t* digldotrim, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);

    *rtcclksel = (localVal & ((uint32_t)0x80000000)) >> 31;
    *slpenlpcmp = (localVal & ((uint32_t)0x40000000)) >> 30;
    *slpenrtc = (localVal & ((uint32_t)0x20000000)) >> 29;
    *slpenxt32k = (localVal & ((uint32_t)0x10000000)) >> 28;
    *slpenrc32k = (localVal & ((uint32_t)0x08000000)) >> 27;
    *lcldoouttrim = (localVal & ((uint32_t)0x04000000)) >> 26;
    *digldotrim = (localVal & ((uint32_t)0x00600000)) >> 21;
    *reserved = (localVal & ((uint32_t)0x001FFFFF)) >> 0;
}

__INLINE uint8_t aon_pmctl0_rtc_clk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void aon_pmctl0_rtc_clk_sel_setf(uint8_t rtcclksel)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)rtcclksel << 31));
}

__INLINE uint8_t aon_pmctl0_slp_en_lpcmp_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void aon_pmctl0_slp_en_lpcmp_setf(uint8_t slpenlpcmp)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)slpenlpcmp << 30));
}

__INLINE uint8_t aon_pmctl0_slp_en_rtc_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void aon_pmctl0_slp_en_rtc_setf(uint8_t slpenrtc)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)slpenrtc << 29));
}

__INLINE uint8_t aon_pmctl0_slp_en_xt32k_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void aon_pmctl0_slp_en_xt32k_setf(uint8_t slpenxt32k)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)slpenxt32k << 28));
}

__INLINE uint8_t aon_pmctl0_slp_en_rc32k_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void aon_pmctl0_slp_en_rc32k_setf(uint8_t slpenrc32k)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)slpenrc32k << 27));
}

__INLINE uint8_t aon_pmctl0_lcldo_out_trim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void aon_pmctl0_lcldo_out_trim_setf(uint8_t lcldoouttrim)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)lcldoouttrim << 26));
}

__INLINE uint8_t aon_pmctl0_dig_ldo_trim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00600000)) >> 21);
}

__INLINE void aon_pmctl0_dig_ldo_trim_setf(uint8_t digldotrim)
{
    _PICO_REG_WR(AON_PMCTL0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00600000)) | ((uint32_t)digldotrim << 21));
}

 /**
 * @brief PMCTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  18:17          xt32k_b_ctl   0b11
 *  14:09         rc32m_c_trim   0b20
 *     08        rc32m_cap_mux   0
 *     07        rc32k_cap_mux   0
 *  06:01         rc32k_c_trim   0b20
 *     00             reserved   0
 * </pre>
 */
#define AON_PMCTL1_OFFSET 0x00000018


__INLINE uint32_t aon_pmctl1_get(void)
{
    return _PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_pmctl1_set(uint32_t value)
{
    _PICO_REG_WR(AON_PMCTL1_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_PMCTL1_XT32K_B_CTL_MASK                      ((uint32_t)0x00060000)
#define AON_PMCTL1_XT32K_B_CTL_LSB                       17
#define AON_PMCTL1_XT32K_B_CTL_WIDTH                     ((uint32_t)0x00000002)
#define AON_PMCTL1_RC32M_C_TRIM_MASK                     ((uint32_t)0x00007E00)
#define AON_PMCTL1_RC32M_C_TRIM_LSB                      9
#define AON_PMCTL1_RC32M_C_TRIM_WIDTH                    ((uint32_t)0x00000006)
#define AON_PMCTL1_RC32M_CAP_MUX_BIT                     ((uint32_t)0x00000100)
#define AON_PMCTL1_RC32M_CAP_MUX_POS                     8
#define AON_PMCTL1_RC32K_CAP_MUX_BIT                     ((uint32_t)0x00000080)
#define AON_PMCTL1_RC32K_CAP_MUX_POS                     7
#define AON_PMCTL1_RC32K_C_TRIM_MASK                     ((uint32_t)0x0000007E)
#define AON_PMCTL1_RC32K_C_TRIM_LSB                      1
#define AON_PMCTL1_RC32K_C_TRIM_WIDTH                    ((uint32_t)0x00000006)
#define AON_PMCTL1_RESERVED_BIT                          ((uint32_t)0x00000001)
#define AON_PMCTL1_RESERVED_POS                          0

#define AON_PMCTL1_XT32K_B_CTL_RST                       0x11
#define AON_PMCTL1_RC32M_C_TRIM_RST                      0x20
#define AON_PMCTL1_RC32M_CAP_MUX_RST                     0x0
#define AON_PMCTL1_RC32K_CAP_MUX_RST                     0x0
#define AON_PMCTL1_RC32K_C_TRIM_RST                      0x20
#define AON_PMCTL1_RESERVED_RST                          0x0

__INLINE void aon_pmctl1_pack(uint8_t xt32kbctl, uint8_t rc32mctrim, uint8_t rc32mcapmux, uint8_t rc32kcapmux, uint8_t rc32kctrim)
{
    _PICO_REG_WR(AON_PMCTL1_OFFSET+ AON_BASE_ADDR,  ((uint32_t)xt32kbctl << 17) | ((uint32_t)rc32mctrim << 9) | ((uint32_t)rc32mcapmux << 8) | ((uint32_t)rc32kcapmux << 7) | ((uint32_t)rc32kctrim << 1));
}

__INLINE void aon_pmctl1_unpack(uint8_t* xt32kbctl, uint8_t* rc32mctrim, uint8_t* rc32mcapmux, uint8_t* rc32kcapmux, uint8_t* rc32kctrim, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR);

    *xt32kbctl = (localVal & ((uint32_t)0x00060000)) >> 17;
    *rc32mctrim = (localVal & ((uint32_t)0x00007E00)) >> 9;
    *rc32mcapmux = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rc32kcapmux = (localVal & ((uint32_t)0x00000080)) >> 7;
    *rc32kctrim = (localVal & ((uint32_t)0x0000007E)) >> 1;
    *reserved = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t aon_pmctl1_xt32k_b_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00060000)) >> 17);
}

__INLINE void aon_pmctl1_xt32k_b_ctl_setf(uint8_t xt32kbctl)
{
    _PICO_REG_WR(AON_PMCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00060000)) | ((uint32_t)xt32kbctl << 17));
}

__INLINE uint8_t aon_pmctl1_rc32m_c_trim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007E00)) >> 9);
}

__INLINE void aon_pmctl1_rc32m_c_trim_setf(uint8_t rc32mctrim)
{
    _PICO_REG_WR(AON_PMCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00007E00)) | ((uint32_t)rc32mctrim << 9));
}

__INLINE uint8_t aon_pmctl1_rc32m_cap_mux_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void aon_pmctl1_rc32m_cap_mux_setf(uint8_t rc32mcapmux)
{
    _PICO_REG_WR(AON_PMCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rc32mcapmux << 8));
}

__INLINE uint8_t aon_pmctl1_rc32k_cap_mux_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void aon_pmctl1_rc32k_cap_mux_setf(uint8_t rc32kcapmux)
{
    _PICO_REG_WR(AON_PMCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rc32kcapmux << 7));
}

__INLINE uint8_t aon_pmctl1_rc32k_c_trim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000007E)) >> 1);
}

__INLINE void aon_pmctl1_rc32k_c_trim_setf(uint8_t rc32kctrim)
{
    _PICO_REG_WR(AON_PMCTL1_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL1_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x0000007E)) | ((uint32_t)rc32kctrim << 1));
}

 /**
 * @brief PMCTL2_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     21           retram4_en   0
 *     20                   NA   0
 *     19           retram2_en   0
 *     18           retram1_en   0
 *     17           retram0_en   0
 *     16         lpcmp_en_ctl   1
 *     15           rtc_en_ctl   1
 *     14         lcldo_en_ctl   1
 *     13          chgp_en_ctl   1
 *     12        digldo_en_ctl   1
 *     11          dcdc_en_ctl   1
 *     10            bg_en_ctl   1
 *     09         xt32k_en_ctl   1
 *     08         rc32k_en_ctl   1
 *     07         rc32m_en_ctl   1
 *     06         pwr_ctl_mode   0
 *  02:00        chgp_wait_sel   0b110
 * </pre>
 */
#define AON_PMCTL2_0_OFFSET 0x0000001C


__INLINE uint32_t aon_pmctl2_0_get(void)
{
    return _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_pmctl2_0_set(uint32_t value)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_PMCTL2_0_RETRAM4_EN_BIT                        ((uint32_t)0x00200000)
#define AON_PMCTL2_0_RETRAM4_EN_POS                        21
#define AON_PMCTL2_0_NA_BIT                                ((uint32_t)0x00100000)
#define AON_PMCTL2_0_NA_POS                                20
#define AON_PMCTL2_0_RETRAM2_EN_BIT                        ((uint32_t)0x00080000)
#define AON_PMCTL2_0_RETRAM2_EN_POS                        19
#define AON_PMCTL2_0_RETRAM1_EN_BIT                        ((uint32_t)0x00040000)
#define AON_PMCTL2_0_RETRAM1_EN_POS                        18
#define AON_PMCTL2_0_RETRAM0_EN_BIT                        ((uint32_t)0x00020000)
#define AON_PMCTL2_0_RETRAM0_EN_POS                        17
#define AON_PMCTL2_0_LPCMP_EN_CTL_BIT                      ((uint32_t)0x00010000)
#define AON_PMCTL2_0_LPCMP_EN_CTL_POS                      16
#define AON_PMCTL2_0_RTC_EN_CTL_BIT                        ((uint32_t)0x00008000)
#define AON_PMCTL2_0_RTC_EN_CTL_POS                        15
#define AON_PMCTL2_0_LCLDO_EN_CTL_BIT                      ((uint32_t)0x00004000)
#define AON_PMCTL2_0_LCLDO_EN_CTL_POS                      14
#define AON_PMCTL2_0_CHGP_EN_CTL_BIT                       ((uint32_t)0x00002000)
#define AON_PMCTL2_0_CHGP_EN_CTL_POS                       13
#define AON_PMCTL2_0_DIGLDO_EN_CTL_BIT                     ((uint32_t)0x00001000)
#define AON_PMCTL2_0_DIGLDO_EN_CTL_POS                     12
#define AON_PMCTL2_0_DCDC_EN_CTL_BIT                       ((uint32_t)0x00000800)
#define AON_PMCTL2_0_DCDC_EN_CTL_POS                       11
#define AON_PMCTL2_0_BG_EN_CTL_BIT                         ((uint32_t)0x00000400)
#define AON_PMCTL2_0_BG_EN_CTL_POS                         10
#define AON_PMCTL2_0_XT32K_EN_CTL_BIT                      ((uint32_t)0x00000200)
#define AON_PMCTL2_0_XT32K_EN_CTL_POS                      9
#define AON_PMCTL2_0_RC32K_EN_CTL_BIT                      ((uint32_t)0x00000100)
#define AON_PMCTL2_0_RC32K_EN_CTL_POS                      8
#define AON_PMCTL2_0_RC32M_EN_CTL_BIT                      ((uint32_t)0x00000080)
#define AON_PMCTL2_0_RC32M_EN_CTL_POS                      7
#define AON_PMCTL2_0_PWR_CTL_MODE_BIT                      ((uint32_t)0x00000040)
#define AON_PMCTL2_0_PWR_CTL_MODE_POS                      6
#define AON_PMCTL2_0_CHGP_WAIT_SEL_MASK                    ((uint32_t)0x00000007)
#define AON_PMCTL2_0_CHGP_WAIT_SEL_LSB                     0
#define AON_PMCTL2_0_CHGP_WAIT_SEL_WIDTH                   ((uint32_t)0x00000003)

#define AON_PMCTL2_0_RETRAM4_EN_RST                        0x0
#define AON_PMCTL2_0_NA_RST                                0x0
#define AON_PMCTL2_0_RETRAM2_EN_RST                        0x0
#define AON_PMCTL2_0_RETRAM1_EN_RST                        0x0
#define AON_PMCTL2_0_RETRAM0_EN_RST                        0x0
#define AON_PMCTL2_0_LPCMP_EN_CTL_RST                      0x1
#define AON_PMCTL2_0_RTC_EN_CTL_RST                        0x1
#define AON_PMCTL2_0_LCLDO_EN_CTL_RST                      0x1
#define AON_PMCTL2_0_CHGP_EN_CTL_RST                       0x1
#define AON_PMCTL2_0_DIGLDO_EN_CTL_RST                     0x1
#define AON_PMCTL2_0_DCDC_EN_CTL_RST                       0x1
#define AON_PMCTL2_0_BG_EN_CTL_RST                         0x1
#define AON_PMCTL2_0_XT32K_EN_CTL_RST                      0x1
#define AON_PMCTL2_0_RC32K_EN_CTL_RST                      0x1
#define AON_PMCTL2_0_RC32M_EN_CTL_RST                      0x1
#define AON_PMCTL2_0_PWR_CTL_MODE_RST                      0x0
#define AON_PMCTL2_0_CHGP_WAIT_SEL_RST                     0x110

__INLINE void aon_pmctl2_0_pack(uint8_t retram4en, uint8_t na, uint8_t retram2en, uint8_t retram1en, uint8_t retram0en, uint8_t lpcmpenctl, uint8_t rtcenctl, uint8_t lcldoenctl, uint8_t chgpenctl, uint8_t digldoenctl, uint8_t dcdcenctl, uint8_t bgenctl, uint8_t xt32kenctl, uint8_t rc32kenctl, uint8_t rc32menctl, uint8_t pwrctlmode, uint8_t chgpwaitsel)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR,  ((uint32_t)retram4en << 21) | ((uint32_t)na << 20) | ((uint32_t)retram2en << 19) | ((uint32_t)retram1en << 18) | ((uint32_t)retram0en << 17) | ((uint32_t)lpcmpenctl << 16) | ((uint32_t)rtcenctl << 15) | ((uint32_t)lcldoenctl << 14) | ((uint32_t)chgpenctl << 13) | ((uint32_t)digldoenctl << 12) | ((uint32_t)dcdcenctl << 11) | ((uint32_t)bgenctl << 10) | ((uint32_t)xt32kenctl << 9) | ((uint32_t)rc32kenctl << 8) | ((uint32_t)rc32menctl << 7) | ((uint32_t)pwrctlmode << 6) | ((uint32_t)chgpwaitsel << 0));
}

__INLINE void aon_pmctl2_0_unpack(uint8_t* retram4en, uint8_t* na, uint8_t* retram2en, uint8_t* retram1en, uint8_t* retram0en, uint8_t* lpcmpenctl, uint8_t* rtcenctl, uint8_t* lcldoenctl, uint8_t* chgpenctl, uint8_t* digldoenctl, uint8_t* dcdcenctl, uint8_t* bgenctl, uint8_t* xt32kenctl, uint8_t* rc32kenctl, uint8_t* rc32menctl, uint8_t* pwrctlmode, uint8_t* chgpwaitsel)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);

    *retram4en = (localVal & ((uint32_t)0x00200000)) >> 21;
    *na = (localVal & ((uint32_t)0x00100000)) >> 20;
    *retram2en = (localVal & ((uint32_t)0x00080000)) >> 19;
    *retram1en = (localVal & ((uint32_t)0x00040000)) >> 18;
    *retram0en = (localVal & ((uint32_t)0x00020000)) >> 17;
    *lpcmpenctl = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rtcenctl = (localVal & ((uint32_t)0x00008000)) >> 15;
    *lcldoenctl = (localVal & ((uint32_t)0x00004000)) >> 14;
    *chgpenctl = (localVal & ((uint32_t)0x00002000)) >> 13;
    *digldoenctl = (localVal & ((uint32_t)0x00001000)) >> 12;
    *dcdcenctl = (localVal & ((uint32_t)0x00000800)) >> 11;
    *bgenctl = (localVal & ((uint32_t)0x00000400)) >> 10;
    *xt32kenctl = (localVal & ((uint32_t)0x00000200)) >> 9;
    *rc32kenctl = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rc32menctl = (localVal & ((uint32_t)0x00000080)) >> 7;
    *pwrctlmode = (localVal & ((uint32_t)0x00000040)) >> 6;
    *chgpwaitsel = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t aon_pmctl2_0_retram4_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void aon_pmctl2_0_retram4_en_setf(uint8_t retram4en)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)retram4en << 21));
}

__INLINE uint8_t aon_pmctl2_0_na_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void aon_pmctl2_0_na_setf(uint8_t na)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)na << 20));
}

__INLINE uint8_t aon_pmctl2_0_retram2_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void aon_pmctl2_0_retram2_en_setf(uint8_t retram2en)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)retram2en << 19));
}

__INLINE uint8_t aon_pmctl2_0_retram1_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void aon_pmctl2_0_retram1_en_setf(uint8_t retram1en)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)retram1en << 18));
}

__INLINE uint8_t aon_pmctl2_0_retram0_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void aon_pmctl2_0_retram0_en_setf(uint8_t retram0en)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)retram0en << 17));
}

__INLINE uint8_t aon_pmctl2_0_lpcmp_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void aon_pmctl2_0_lpcmp_en_ctl_setf(uint8_t lpcmpenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)lpcmpenctl << 16));
}

__INLINE uint8_t aon_pmctl2_0_rtc_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void aon_pmctl2_0_rtc_en_ctl_setf(uint8_t rtcenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)rtcenctl << 15));
}

__INLINE uint8_t aon_pmctl2_0_lcldo_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void aon_pmctl2_0_lcldo_en_ctl_setf(uint8_t lcldoenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)lcldoenctl << 14));
}

__INLINE uint8_t aon_pmctl2_0_chgp_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void aon_pmctl2_0_chgp_en_ctl_setf(uint8_t chgpenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)chgpenctl << 13));
}

__INLINE uint8_t aon_pmctl2_0_digldo_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void aon_pmctl2_0_digldo_en_ctl_setf(uint8_t digldoenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)digldoenctl << 12));
}

__INLINE uint8_t aon_pmctl2_0_dcdc_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void aon_pmctl2_0_dcdc_en_ctl_setf(uint8_t dcdcenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)dcdcenctl << 11));
}

__INLINE uint8_t aon_pmctl2_0_bg_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void aon_pmctl2_0_bg_en_ctl_setf(uint8_t bgenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)bgenctl << 10));
}

__INLINE uint8_t aon_pmctl2_0_xt32k_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void aon_pmctl2_0_xt32k_en_ctl_setf(uint8_t xt32kenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)xt32kenctl << 9));
}

__INLINE uint8_t aon_pmctl2_0_rc32k_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void aon_pmctl2_0_rc32k_en_ctl_setf(uint8_t rc32kenctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rc32kenctl << 8));
}

__INLINE uint8_t aon_pmctl2_0_rc32m_en_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void aon_pmctl2_0_rc32m_en_ctl_setf(uint8_t rc32menctl)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rc32menctl << 7));
}

__INLINE uint8_t aon_pmctl2_0_pwr_ctl_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void aon_pmctl2_0_pwr_ctl_mode_setf(uint8_t pwrctlmode)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)pwrctlmode << 6));
}

__INLINE uint8_t aon_pmctl2_0_chgp_wait_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void aon_pmctl2_0_chgp_wait_sel_setf(uint8_t chgpwaitsel)
{
    _PICO_REG_WR(AON_PMCTL2_0_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_PMCTL2_0_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)chgpwaitsel << 0));
}

 /**
 * @brief PMCTL2_1 register definition
 */
#define AON_PMCTL2_1_OFFSET 0x00000020


__INLINE uint32_t aon_pmctl2_1_get(void)
{
    return _PICO_REG_RD(AON_PMCTL2_1_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_PMCTL2_1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_PMCTL2_1_RESERVED_LSB                          0
#define AON_PMCTL2_1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AON_PMCTL2_1_RESERVED_RST                          0x0

__INLINE void aon_pmctl2_1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PMCTL2_1_OFFSET + AON_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief RTCCTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     20        cmp0_event_en   0
 *     01          rtc_clr_bit   0
 *     00           rtc_rs_ctl   0
 * </pre>
 */
#define AON_RTCCTL_OFFSET 0x00000024


__INLINE uint32_t aon_rtcctl_get(void)
{
    return _PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_rtcctl_set(uint32_t value)
{
    _PICO_REG_WR(AON_RTCCTL_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_RTCCTL_CMP0_EVENT_EN_BIT                     ((uint32_t)0x00100000)
#define AON_RTCCTL_CMP0_EVENT_EN_POS                     20
#define AON_RTCCTL_RTC_CLR_BIT_BIT                       ((uint32_t)0x00000002)
#define AON_RTCCTL_RTC_CLR_BIT_POS                       1
#define AON_RTCCTL_RTC_RS_CTL_BIT                        ((uint32_t)0x00000001)
#define AON_RTCCTL_RTC_RS_CTL_POS                        0

#define AON_RTCCTL_CMP0_EVENT_EN_RST                     0x0
#define AON_RTCCTL_RTC_CLR_BIT_RST                       0x0
#define AON_RTCCTL_RTC_RS_CTL_RST                        0x0

__INLINE void aon_rtcctl_pack(uint8_t cmp0eventen, uint8_t rtcclrbit, uint8_t rtcrsctl)
{
    _PICO_REG_WR(AON_RTCCTL_OFFSET+ AON_BASE_ADDR,  ((uint32_t)cmp0eventen << 20) | ((uint32_t)rtcclrbit << 1) | ((uint32_t)rtcrsctl << 0));
}

__INLINE void aon_rtcctl_unpack(uint8_t* cmp0eventen, uint8_t* rtcclrbit, uint8_t* rtcrsctl)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR);

    *cmp0eventen = (localVal & ((uint32_t)0x00100000)) >> 20;
    *rtcclrbit = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rtcrsctl = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t aon_rtcctl_cmp0_event_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void aon_rtcctl_cmp0_event_en_setf(uint8_t cmp0eventen)
{
    _PICO_REG_WR(AON_RTCCTL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)cmp0eventen << 20));
}

__INLINE uint8_t aon_rtcctl_rtc_clr_bit_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void aon_rtcctl_rtc_clr_bit_setf(uint8_t rtcclrbit)
{
    _PICO_REG_WR(AON_RTCCTL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rtcclrbit << 1));
}

__INLINE uint8_t aon_rtcctl_rtc_rs_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void aon_rtcctl_rtc_rs_ctl_setf(uint8_t rtcrsctl)
{
    _PICO_REG_WR(AON_RTCCTL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_RTCCTL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rtcrsctl << 0));
}

 /**
 * @brief RTCCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00           rd_rtc_val   0b0
 * </pre>
 */
#define AON_RTCCNT_OFFSET 0x00000028


__INLINE uint32_t aon_rtccnt_get(void)
{
    return _PICO_REG_RD(AON_RTCCNT_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_RTCCNT_RD_RTC_VAL_MASK                       ((uint32_t)0x00FFFFFF)
#define AON_RTCCNT_RD_RTC_VAL_LSB                        0
#define AON_RTCCNT_RD_RTC_VAL_WIDTH                      ((uint32_t)0x00000018)

#define AON_RTCCNT_RD_RTC_VAL_RST                        0x0

__INLINE void aon_rtccnt_unpack(uint8_t* rdrtcval)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCNT_OFFSET + AON_BASE_ADDR);

    *rdrtcval = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t aon_rtccnt_rd_rtc_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCNT_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

 /**
 * @brief RTCCC0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00             cmp0_val   0b0
 * </pre>
 */
#define AON_RTCCC0_OFFSET 0x0000002C


__INLINE uint32_t aon_rtccc0_get(void)
{
    return _PICO_REG_RD(AON_RTCCC0_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_rtccc0_set(uint32_t value)
{
    _PICO_REG_WR(AON_RTCCC0_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_RTCCC0_CMP0_VAL_MASK                         ((uint32_t)0x000FFFFF)
#define AON_RTCCC0_CMP0_VAL_LSB                          0
#define AON_RTCCC0_CMP0_VAL_WIDTH                        ((uint32_t)0x00000014)

#define AON_RTCCC0_CMP0_VAL_RST                          0x0

__INLINE void aon_rtccc0_pack(uint32_t cmp0val)
{
    _PICO_REG_WR(AON_RTCCC0_OFFSET+ AON_BASE_ADDR,  ((uint32_t)cmp0val << 0));
}

__INLINE void aon_rtccc0_unpack(uint8_t* cmp0val)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCC0_OFFSET + AON_BASE_ADDR);

    *cmp0val = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t aon_rtccc0_cmp0_val_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCC0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void aon_rtccc0_cmp0_val_setf(uint32_t cmp0val)
{
    _PICO_REG_WR(AON_RTCCC0_OFFSET+ AON_BASE_ADDR, (uint32_t)cmp0val << 0);
}

 /**
 * @brief RTCCC1 register definition
 */
#define AON_RTCCC1_OFFSET 0x00000030


__INLINE uint32_t aon_rtccc1_get(void)
{
    return _PICO_REG_RD(AON_RTCCC1_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_RTCCC1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_RTCCC1_RESERVED_LSB                          0
#define AON_RTCCC1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AON_RTCCC1_RESERVED_RST                          0x0

__INLINE void aon_rtccc1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCC1_OFFSET + AON_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief RTCCC2 register definition
 */
#define AON_RTCCC2_OFFSET 0x00000034


__INLINE uint32_t aon_rtccc2_get(void)
{
    return _PICO_REG_RD(AON_RTCCC2_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_RTCCC2_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_RTCCC2_RESERVED_LSB                          0
#define AON_RTCCC2_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AON_RTCCC2_RESERVED_RST                          0x0

__INLINE void aon_rtccc2_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCCC2_OFFSET + AON_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief RTCFLAG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00            cmp0_flag   0
 * </pre>
 */
#define AON_RTCFLAG_OFFSET 0x00000038


__INLINE uint32_t aon_rtcflag_get(void)
{
    return _PICO_REG_RD(AON_RTCFLAG_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_RTCFLAG_CMP0_FLAG_BIT                         ((uint32_t)0x00000001)
#define AON_RTCFLAG_CMP0_FLAG_POS                         0

#define AON_RTCFLAG_CMP0_FLAG_RST                         0x0

__INLINE void aon_rtcflag_unpack(uint8_t* cmp0flag)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCFLAG_OFFSET + AON_BASE_ADDR);

    *cmp0flag = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t aon_rtcflag_cmp0_flag_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RTCFLAG_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief REG_S9 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00           io_wu_mask   0b0
 * </pre>
 */
#define AON_REG_S9_OFFSET 0x000000A0


__INLINE uint32_t aon_reg_s9_get(void)
{
    return _PICO_REG_RD(AON_REG_S9_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_reg_s9_set(uint32_t value)
{
    _PICO_REG_WR(AON_REG_S9_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_REG_S9_IO_WU_MASK_MASK                       ((uint32_t)0x000FFFFF)
#define AON_REG_S9_IO_WU_MASK_LSB                        0
#define AON_REG_S9_IO_WU_MASK_WIDTH                      ((uint32_t)0x00000014)

#define AON_REG_S9_IO_WU_MASK_RST                        0x0

__INLINE void aon_reg_s9_pack(uint32_t iowumask)
{
    _PICO_REG_WR(AON_REG_S9_OFFSET+ AON_BASE_ADDR,  ((uint32_t)iowumask << 0));
}

__INLINE void aon_reg_s9_unpack(uint8_t* iowumask)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_REG_S9_OFFSET + AON_BASE_ADDR);

    *iowumask = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t aon_reg_s9_io_wu_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_REG_S9_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void aon_reg_s9_io_wu_mask_setf(uint32_t iowumask)
{
    _PICO_REG_WR(AON_REG_S9_OFFSET+ AON_BASE_ADDR, (uint32_t)iowumask << 0);
}

 /**
 * @brief REG_S10 register definition
 */
#define AON_REG_S10_OFFSET 0x000000A4


__INLINE uint32_t aon_reg_s10_get(void)
{
    return _PICO_REG_RD(AON_REG_S10_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_REG_S10_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_REG_S10_RESERVED_LSB                          0
#define AON_REG_S10_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AON_REG_S10_RESERVED_RST                          0x0

__INLINE void aon_reg_s10_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_REG_S10_OFFSET + AON_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief REG_S11 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:07             vtor_lat   0b0
 *  02:01            remap_lat   0b0
 *     00          wakeup_flag   0
 * </pre>
 */
#define AON_REG_S11_OFFSET 0x000000A8


__INLINE uint32_t aon_reg_s11_get(void)
{
    return _PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_reg_s11_set(uint32_t value)
{
    _PICO_REG_WR(AON_REG_S11_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_REG_S11_VTOR_LAT_MASK                         ((uint32_t)0xFFFFFF80)
#define AON_REG_S11_VTOR_LAT_LSB                          7
#define AON_REG_S11_VTOR_LAT_WIDTH                        ((uint32_t)0x00000019)
#define AON_REG_S11_REMAP_LAT_MASK                        ((uint32_t)0x00000006)
#define AON_REG_S11_REMAP_LAT_LSB                         1
#define AON_REG_S11_REMAP_LAT_WIDTH                       ((uint32_t)0x00000002)
#define AON_REG_S11_WAKEUP_FLAG_BIT                       ((uint32_t)0x00000001)
#define AON_REG_S11_WAKEUP_FLAG_POS                       0

#define AON_REG_S11_VTOR_LAT_RST                          0x0
#define AON_REG_S11_REMAP_LAT_RST                         0x0
#define AON_REG_S11_WAKEUP_FLAG_RST                       0x0

__INLINE void aon_reg_s11_pack(uint32_t vtorlat, uint8_t remaplat, uint8_t wakeupflag)
{
    _PICO_REG_WR(AON_REG_S11_OFFSET+ AON_BASE_ADDR,  ((uint32_t)vtorlat << 7) | ((uint32_t)remaplat << 1) | ((uint32_t)wakeupflag << 0));
}

__INLINE void aon_reg_s11_unpack(uint8_t* vtorlat, uint8_t* remaplat, uint8_t* wakeupflag)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR);

    *vtorlat = (localVal & ((uint32_t)0xFFFFFF80)) >> 7;
    *remaplat = (localVal & ((uint32_t)0x00000006)) >> 1;
    *wakeupflag = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint32_t aon_reg_s11_vtor_lat_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFF80)) >> 7);
}

__INLINE void aon_reg_s11_vtor_lat_setf(uint32_t vtorlat)
{
    _PICO_REG_WR(AON_REG_S11_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0xFFFFFF80)) | ((uint32_t)vtorlat << 7));
}

__INLINE uint8_t aon_reg_s11_remap_lat_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000006)) >> 1);
}

__INLINE void aon_reg_s11_remap_lat_setf(uint8_t remaplat)
{
    _PICO_REG_WR(AON_REG_S11_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000006)) | ((uint32_t)remaplat << 1));
}

__INLINE uint8_t aon_reg_s11_wakeup_flag_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void aon_reg_s11_wakeup_flag_setf(uint8_t wakeupflag)
{
    _PICO_REG_WR(AON_REG_S11_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_REG_S11_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)wakeupflag << 0));
}

 /**
 * @brief IDLE_REG register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00           r_idle_reg   0b0
 * </pre>
 */
#define AON_IDLE_REG_OFFSET 0x000000AC


__INLINE uint32_t aon_idle_reg_get(void)
{
    return _PICO_REG_RD(AON_IDLE_REG_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_IDLE_REG_R_IDLE_REG_MASK                       ((uint32_t)0xFFFFFFFF)
#define AON_IDLE_REG_R_IDLE_REG_LSB                        0
#define AON_IDLE_REG_R_IDLE_REG_WIDTH                      ((uint32_t)0x00000020)

#define AON_IDLE_REG_R_IDLE_REG_RST                        0x0

__INLINE void aon_idle_reg_unpack(uint8_t* ridlereg)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IDLE_REG_OFFSET + AON_BASE_ADDR);

    *ridlereg = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aon_idle_reg_r_idle_reg_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_IDLE_REG_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GPIO_WU_SRC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00          gpio_wu_src   0b0
 * </pre>
 */
#define AON_GPIO_WU_SRC_OFFSET 0x000000B0


__INLINE uint32_t aon_gpio_wu_src_get(void)
{
    return _PICO_REG_RD(AON_GPIO_WU_SRC_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_GPIO_WU_SRC_GPIO_WU_SRC_MASK                      ((uint32_t)0x000FFFFF)
#define AON_GPIO_WU_SRC_GPIO_WU_SRC_LSB                       0
#define AON_GPIO_WU_SRC_GPIO_WU_SRC_WIDTH                     ((uint32_t)0x00000014)

#define AON_GPIO_WU_SRC_GPIO_WU_SRC_RST                       0x0

__INLINE void aon_gpio_wu_src_unpack(uint8_t* gpiowusrc)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_GPIO_WU_SRC_OFFSET + AON_BASE_ADDR);

    *gpiowusrc = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t aon_gpio_wu_src_gpio_wu_src_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_GPIO_WU_SRC_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

 /**
 * @brief RESERVED register definition
 */
#define AON_RESERVED_OFFSET 0x000000B4


__INLINE uint32_t aon_reserved_get(void)
{
    return _PICO_REG_RD(AON_RESERVED_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_RESERVED_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_RESERVED_RESERVED_LSB                          0
#define AON_RESERVED_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AON_RESERVED_RESERVED_RST                          0x0

__INLINE void aon_reserved_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RESERVED_OFFSET + AON_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief PCLK_CLK_GATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00     pclk_clk_gate_en   1
 * </pre>
 */
#define AON_PCLK_CLK_GATE_OFFSET 0x000000B8


__INLINE uint32_t aon_pclk_clk_gate_get(void)
{
    return _PICO_REG_RD(AON_PCLK_CLK_GATE_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_pclk_clk_gate_set(uint32_t value)
{
    _PICO_REG_WR(AON_PCLK_CLK_GATE_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_PCLK_CLK_GATE_PCLK_CLK_GATE_EN_BIT                  ((uint32_t)0x00000001)
#define AON_PCLK_CLK_GATE_PCLK_CLK_GATE_EN_POS                  0

#define AON_PCLK_CLK_GATE_PCLK_CLK_GATE_EN_RST                  0x1

__INLINE void aon_pclk_clk_gate_pack(uint8_t pclkclkgateen)
{
    _PICO_REG_WR(AON_PCLK_CLK_GATE_OFFSET+ AON_BASE_ADDR,  ((uint32_t)pclkclkgateen << 0));
}

__INLINE void aon_pclk_clk_gate_unpack(uint8_t* pclkclkgateen)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PCLK_CLK_GATE_OFFSET + AON_BASE_ADDR);

    *pclkclkgateen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t aon_pclk_clk_gate_pclk_clk_gate_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_PCLK_CLK_GATE_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void aon_pclk_clk_gate_pclk_clk_gate_en_setf(uint8_t pclkclkgateen)
{
    _PICO_REG_WR(AON_PCLK_CLK_GATE_OFFSET+ AON_BASE_ADDR, (uint32_t)pclkclkgateen << 0);
}

 /**
 * @brief XTAL_16M_CTRL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31           xtal16M_en   1
 *  30:29        xtal_ldo_ctrl   0b11
 *     28                 rsv0   0
 *  27:26   xtal_dither_i_tune   0b0
 *  25:16    pmu_ldoH_out_ctrl   0b0
 *     12      pmu_ldoH_out_en   1
 *  10:08         boot_clk_sel   0b0
 *  06:00        xtal_16m_ctrl   0b0
 * </pre>
 */
#define AON_XTAL_16M_CTRL_OFFSET 0x000000BC


__INLINE uint32_t aon_xtal_16m_ctrl_get(void)
{
    return _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_xtal_16m_ctrl_set(uint32_t value)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_XTAL_16M_CTRL_XTAL16M_EN_BIT                        ((uint32_t)0x80000000)
#define AON_XTAL_16M_CTRL_XTAL16M_EN_POS                        31
#define AON_XTAL_16M_CTRL_XTAL_LDO_CTRL_MASK                    ((uint32_t)0x60000000)
#define AON_XTAL_16M_CTRL_XTAL_LDO_CTRL_LSB                     29
#define AON_XTAL_16M_CTRL_XTAL_LDO_CTRL_WIDTH                   ((uint32_t)0x00000002)
#define AON_XTAL_16M_CTRL_RSV0_BIT                              ((uint32_t)0x10000000)
#define AON_XTAL_16M_CTRL_RSV0_POS                              28
#define AON_XTAL_16M_CTRL_XTAL_DITHER_I_TUNE_MASK               ((uint32_t)0x0C000000)
#define AON_XTAL_16M_CTRL_XTAL_DITHER_I_TUNE_LSB                26
#define AON_XTAL_16M_CTRL_XTAL_DITHER_I_TUNE_WIDTH              ((uint32_t)0x00000002)
#define AON_XTAL_16M_CTRL_PMU_LDO_H_OUT_CTRL_MASK               ((uint32_t)0x03FF0000)
#define AON_XTAL_16M_CTRL_PMU_LDO_H_OUT_CTRL_LSB                16
#define AON_XTAL_16M_CTRL_PMU_LDO_H_OUT_CTRL_WIDTH              ((uint32_t)0x0000000A)
#define AON_XTAL_16M_CTRL_PMU_LDO_H_OUT_EN_BIT                  ((uint32_t)0x00001000)
#define AON_XTAL_16M_CTRL_PMU_LDO_H_OUT_EN_POS                  12
#define AON_XTAL_16M_CTRL_BOOT_CLK_SEL_MASK                     ((uint32_t)0x00000700)
#define AON_XTAL_16M_CTRL_BOOT_CLK_SEL_LSB                      8
#define AON_XTAL_16M_CTRL_BOOT_CLK_SEL_WIDTH                    ((uint32_t)0x00000003)
#define AON_XTAL_16M_CTRL_XTAL_16M_CTRL_MASK                    ((uint32_t)0x0000007F)
#define AON_XTAL_16M_CTRL_XTAL_16M_CTRL_LSB                     0
#define AON_XTAL_16M_CTRL_XTAL_16M_CTRL_WIDTH                   ((uint32_t)0x00000007)

#define AON_XTAL_16M_CTRL_XTAL16M_EN_RST                        0x1
#define AON_XTAL_16M_CTRL_XTAL_LDO_CTRL_RST                     0x11
#define AON_XTAL_16M_CTRL_RSV0_RST                              0x0
#define AON_XTAL_16M_CTRL_XTAL_DITHER_I_TUNE_RST                0x0
#define AON_XTAL_16M_CTRL_PMU_LDO_H_OUT_CTRL_RST                0x0
#define AON_XTAL_16M_CTRL_PMU_LDO_H_OUT_EN_RST                  0x1
#define AON_XTAL_16M_CTRL_BOOT_CLK_SEL_RST                      0x0
#define AON_XTAL_16M_CTRL_XTAL_16M_CTRL_RST                     0x0

__INLINE void aon_xtal_16m_ctrl_pack(uint8_t xtal16men, uint8_t xtalldoctrl, uint8_t rsv0, uint8_t xtalditheritune, uint16_t pmuldohoutctrl, uint8_t pmuldohouten, uint8_t bootclksel, uint8_t xtal16mctrl)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR,  ((uint32_t)xtal16men << 31) | ((uint32_t)xtalldoctrl << 29) | ((uint32_t)rsv0 << 28) | ((uint32_t)xtalditheritune << 26) | ((uint32_t)pmuldohoutctrl << 16) | ((uint32_t)pmuldohouten << 12) | ((uint32_t)bootclksel << 8) | ((uint32_t)xtal16mctrl << 0));
}

__INLINE void aon_xtal_16m_ctrl_unpack(uint8_t* xtal16men, uint8_t* xtalldoctrl, uint8_t* rsv0, uint8_t* xtalditheritune, uint8_t* pmuldohoutctrl, uint8_t* pmuldohouten, uint8_t* bootclksel, uint8_t* xtal16mctrl)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);

    *xtal16men = (localVal & ((uint32_t)0x80000000)) >> 31;
    *xtalldoctrl = (localVal & ((uint32_t)0x60000000)) >> 29;
    *rsv0 = (localVal & ((uint32_t)0x10000000)) >> 28;
    *xtalditheritune = (localVal & ((uint32_t)0x0C000000)) >> 26;
    *pmuldohoutctrl = (localVal & ((uint32_t)0x03FF0000)) >> 16;
    *pmuldohouten = (localVal & ((uint32_t)0x00001000)) >> 12;
    *bootclksel = (localVal & ((uint32_t)0x00000700)) >> 8;
    *xtal16mctrl = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t aon_xtal_16m_ctrl_xtal16m_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void aon_xtal_16m_ctrl_xtal16m_en_setf(uint8_t xtal16men)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)xtal16men << 31));
}

__INLINE uint8_t aon_xtal_16m_ctrl_xtal_ldo_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x60000000)) >> 29);
}

__INLINE void aon_xtal_16m_ctrl_xtal_ldo_ctrl_setf(uint8_t xtalldoctrl)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x60000000)) | ((uint32_t)xtalldoctrl << 29));
}

__INLINE uint8_t aon_xtal_16m_ctrl_rsv0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void aon_xtal_16m_ctrl_rsv0_setf(uint8_t rsv0)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)rsv0 << 28));
}

__INLINE uint8_t aon_xtal_16m_ctrl_xtal_dither_i_tune_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0C000000)) >> 26);
}

__INLINE void aon_xtal_16m_ctrl_xtal_dither_i_tune_setf(uint8_t xtalditheritune)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x0C000000)) | ((uint32_t)xtalditheritune << 26));
}

__INLINE uint16_t aon_xtal_16m_ctrl_pmu_ldo_h_out_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03FF0000)) >> 16);
}

__INLINE void aon_xtal_16m_ctrl_pmu_ldo_h_out_ctrl_setf(uint16_t pmuldohoutctrl)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x03FF0000)) | ((uint32_t)pmuldohoutctrl << 16));
}

__INLINE uint8_t aon_xtal_16m_ctrl_pmu_ldo_h_out_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void aon_xtal_16m_ctrl_pmu_ldo_h_out_en_setf(uint8_t pmuldohouten)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)pmuldohouten << 12));
}

__INLINE uint8_t aon_xtal_16m_ctrl_boot_clk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void aon_xtal_16m_ctrl_boot_clk_sel_setf(uint8_t bootclksel)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)bootclksel << 8));
}

__INLINE uint8_t aon_xtal_16m_ctrl_xtal_16m_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void aon_xtal_16m_ctrl_xtal_16m_ctrl_setf(uint8_t xtal16mctrl)
{
    _PICO_REG_WR(AON_XTAL_16M_CTRL_OFFSET+ AON_BASE_ADDR, (_PICO_REG_RD(AON_XTAL_16M_CTRL_OFFSET + AON_BASE_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)xtal16mctrl << 0));
}

 /**
 * @brief SLEEP_R0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             sleep_r0   0b0
 * </pre>
 */
#define AON_SLEEP_R0_OFFSET 0x000000C0


__INLINE uint32_t aon_sleep_r0_get(void)
{
    return _PICO_REG_RD(AON_SLEEP_R0_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_sleep_r0_set(uint32_t value)
{
    _PICO_REG_WR(AON_SLEEP_R0_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_SLEEP_R0_SLEEP_R0_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_SLEEP_R0_SLEEP_R0_LSB                          0
#define AON_SLEEP_R0_SLEEP_R0_WIDTH                        ((uint32_t)0x00000020)

#define AON_SLEEP_R0_SLEEP_R0_RST                          0x0

__INLINE void aon_sleep_r0_pack(uint32_t sleepr0)
{
    _PICO_REG_WR(AON_SLEEP_R0_OFFSET+ AON_BASE_ADDR,  ((uint32_t)sleepr0 << 0));
}

__INLINE void aon_sleep_r0_unpack(uint8_t* sleepr0)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_SLEEP_R0_OFFSET + AON_BASE_ADDR);

    *sleepr0 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aon_sleep_r0_sleep_r0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_SLEEP_R0_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aon_sleep_r0_sleep_r0_setf(uint32_t sleepr0)
{
    _PICO_REG_WR(AON_SLEEP_R0_OFFSET+ AON_BASE_ADDR, (uint32_t)sleepr0 << 0);
}

 /**
 * @brief SLEEP_R1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             sleep_r1   0b0
 * </pre>
 */
#define AON_SLEEP_R1_OFFSET 0x000000C4


__INLINE uint32_t aon_sleep_r1_get(void)
{
    return _PICO_REG_RD(AON_SLEEP_R1_OFFSET + AON_BASE_ADDR);
}

__INLINE void aon_sleep_r1_set(uint32_t value)
{
    _PICO_REG_WR(AON_SLEEP_R1_OFFSET+ AON_BASE_ADDR, value);
}

// field definitions
#define AON_SLEEP_R1_SLEEP_R1_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_SLEEP_R1_SLEEP_R1_LSB                          0
#define AON_SLEEP_R1_SLEEP_R1_WIDTH                        ((uint32_t)0x00000020)

#define AON_SLEEP_R1_SLEEP_R1_RST                          0x0

__INLINE void aon_sleep_r1_pack(uint32_t sleepr1)
{
    _PICO_REG_WR(AON_SLEEP_R1_OFFSET+ AON_BASE_ADDR,  ((uint32_t)sleepr1 << 0));
}

__INLINE void aon_sleep_r1_unpack(uint8_t* sleepr1)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_SLEEP_R1_OFFSET + AON_BASE_ADDR);

    *sleepr1 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aon_sleep_r1_sleep_r1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_SLEEP_R1_OFFSET + AON_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aon_sleep_r1_sleep_r1_setf(uint32_t sleepr1)
{
    _PICO_REG_WR(AON_SLEEP_R1_OFFSET+ AON_BASE_ADDR, (uint32_t)sleepr1 << 0);
}

 /**
 * @brief RESERVED1 register definition
 */
#define AON_RESERVED1_OFFSET 0x000000C8


__INLINE uint32_t aon_reserved1_get(void)
{
    return _PICO_REG_RD(AON_RESERVED1_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_RESERVED1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_RESERVED1_RESERVED_LSB                          0
#define AON_RESERVED1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AON_RESERVED1_RESERVED_RST                          0x0

__INLINE void aon_reserved1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RESERVED1_OFFSET + AON_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief RESERVED2 register definition
 */
#define AON_RESERVED2_OFFSET 0x000000CC


__INLINE uint32_t aon_reserved2_get(void)
{
    return _PICO_REG_RD(AON_RESERVED2_OFFSET + AON_BASE_ADDR);
}

// field definitions
#define AON_RESERVED2_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AON_RESERVED2_RESERVED_LSB                          0
#define AON_RESERVED2_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AON_RESERVED2_RESERVED_RST                          0x0

__INLINE void aon_reserved2_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AON_RESERVED2_OFFSET + AON_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t pwroff:32;
    }PWROFF_fld;
    __IO uint32_t PWROFF;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t pwrslp:32;
    }PWRSLP_fld;
    __IO uint32_t PWRSLP;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :2;
      __IO uint32_t p09_pul:2;
      __IO uint32_t p09_wu_s:1;
      __IO uint32_t p08_pul:2;
      __IO uint32_t p08_wu_s:1;
      __IO uint32_t p07_pul:2;
      __IO uint32_t p07_wu_s:1;
      __IO uint32_t p06_pul:2;
      __IO uint32_t p06_wu_s:1;
      __IO uint32_t p05_pul:2;
      __IO uint32_t p05_wu_s:1;
      __IO uint32_t p04_pul:2;
      __IO uint32_t p04_wu_s:1;
      __IO uint32_t p03_pul:2;
      __IO uint32_t p03_wu_s:1;
      __IO uint32_t p02_pul:2;
      __IO uint32_t p02_wu_s:1;
      __IO uint32_t p01_pul:2;
      __IO uint32_t p01_wu_s:1;
      __IO uint32_t p00_pul:2;
      __IO uint32_t p00_wu_s:1;
    }IOCTL0_fld;
    __IO uint32_t IOCTL0;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :2;
      __IO uint32_t p19_pul:2;
      __IO uint32_t p19_wake_up:1;
      __IO uint32_t p18_pul:2;
      __IO uint32_t p18_wake_up:1;
      __IO uint32_t p17_pul:2;
      __IO uint32_t p17_wake_up:1;
      __IO uint32_t p16_pul:2;
      __IO uint32_t p16_wake_up:1;
      __IO uint32_t p15_pul:2;
      __IO uint32_t p15_wake_up:1;
      __IO uint32_t p14_pul:2;
      __IO uint32_t p14_wake_up:1;
      __IO uint32_t p13_pul:2;
      __IO uint32_t p13_wake_up:1;
      __IO uint32_t p12_pul:2;
      __IO uint32_t p12_wake_up:1;
      __IO uint32_t p11_pul:2;
      __IO uint32_t p11_wake_up:1;
      __IO uint32_t p10_pul:2;
      __IO uint32_t p10_wake_up:1;
    }IOCTL1_fld;
    __IO uint32_t IOCTL1;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :12;
      __IO uint32_t io_ret_en:20;
    }IOCTL2_fld;
    __IO uint32_t IOCTL2;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t rtc_clk_sel:1;
      __IO uint32_t slp_en_lpcmp:1;
      __IO uint32_t slp_en_rtc:1;
      __IO uint32_t slp_en_xt32k:1;
      __IO uint32_t slp_en_rc32k:1;
      __IO uint32_t lcldo_out_trim:1;
      __IO uint32_t :3;
      __IO uint32_t dig_ldo_trim:2;
      __IO uint32_t :21;
    }PMCTL0_fld;
    __IO uint32_t PMCTL0;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :13;
      __IO uint32_t xt32k_b_ctl:2;
      __IO uint32_t :2;
      __IO uint32_t rc32m_c_trim:6;
      __IO uint32_t rc32m_cap_mux:1;
      __IO uint32_t rc32k_cap_mux:1;
      __IO uint32_t rc32k_c_trim:6;
      __IO uint32_t :1;
    }PMCTL1_fld;
    __IO uint32_t PMCTL1;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :10;
      __IO uint32_t retram4_en:1;
      __IO uint32_t na:1;
      __IO uint32_t retram2_en:1;
      __IO uint32_t retram1_en:1;
      __IO uint32_t retram0_en:1;
      __IO uint32_t lpcmp_en_ctl:1;
      __IO uint32_t rtc_en_ctl:1;
      __IO uint32_t lcldo_en_ctl:1;
      __IO uint32_t chgp_en_ctl:1;
      __IO uint32_t digldo_en_ctl:1;
      __IO uint32_t dcdc_en_ctl:1;
      __IO uint32_t bg_en_ctl:1;
      __IO uint32_t xt32k_en_ctl:1;
      __IO uint32_t rc32k_en_ctl:1;
      __IO uint32_t rc32m_en_ctl:1;
      __IO uint32_t pwr_ctl_mode:1;
      __IO uint32_t :3;
      __IO uint32_t chgp_wait_sel:3;
    }PMCTL2_0_fld;
    __IO uint32_t PMCTL2_0;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t :32;
    }PMCTL2_1_fld;
    __IO uint32_t PMCTL2_1;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :11;
      __IO uint32_t cmp0_event_en:1;
      __IO uint32_t :18;
      __IO uint32_t rtc_clr_bit:1;
      __IO uint32_t rtc_rs_ctl:1;
    }RTCCTL_fld;
    __IO uint32_t RTCCTL;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t :8;
      __IO uint32_t rd_rtc_val:24;
    }RTCCNT_fld;
    __IO uint32_t RTCCNT;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t :12;
      __IO uint32_t cmp0_val:20;
    }RTCCC0_fld;
    __IO uint32_t RTCCC0;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :32;
    }RTCCC1_fld;
    __IO uint32_t RTCCC1;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t :32;
    }RTCCC2_fld;
    __IO uint32_t RTCCC2;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t :31;
      __IO uint32_t cmp0_flag:1;
    }RTCFLAG_fld;
    __IO uint32_t RTCFLAG;
  };


  union{ //offset addr 0x00a0
    struct{
      __IO uint32_t :12;
      __IO uint32_t io_wu_mask:20;
    }reg_s9_fld;
    __IO uint32_t reg_s9;
  };

  union{ //offset addr 0x00a4
    struct{
      __IO uint32_t :32;
    }reg_s10_fld;
    __IO uint32_t reg_s10;
  };

  union{ //offset addr 0x00a8
    struct{
      __IO uint32_t vtor_lat:25;
      __IO uint32_t :4;
      __IO uint32_t remap_lat:2;
      __IO uint32_t wakeup_flag:1;
    }reg_s11_fld;
    __IO uint32_t reg_s11;
  };

  union{ //offset addr 0x00ac
    struct{
      __IO uint32_t r_idle_reg:32;
    }idle_reg_fld;
    __IO uint32_t idle_reg;
  };

  union{ //offset addr 0x00b0
    struct{
      __IO uint32_t :12;
      __IO uint32_t gpio_wu_src:20;
    }gpio_wu_src_fld;
    __IO uint32_t gpio_wu_src;
  };

  union{ //offset addr 0x00b4
    struct{
      __IO uint32_t :32;
    }reserved_fld;
    __IO uint32_t reserved;
  };

  union{ //offset addr 0x00b8
    struct{
      __IO uint32_t :31;
      __IO uint32_t pclk_clk_gate_en:1;
    }pclk_clk_gate_fld;
    __IO uint32_t pclk_clk_gate;
  };

  union{ //offset addr 0x00bc
    struct{
      __IO uint32_t xtal16m_en:1;
      __IO uint32_t xtal_ldo_ctrl:2;
      __IO uint32_t rsv0:1;
      __IO uint32_t xtal_dither_i_tune:2;
      __IO uint32_t pmu_ldo_h_out_ctrl:10;
      __IO uint32_t :3;
      __IO uint32_t pmu_ldo_h_out_en:1;
      __IO uint32_t :1;
      __IO uint32_t boot_clk_sel:3;
      __IO uint32_t :1;
      __IO uint32_t xtal_16m_ctrl:7;
    }xtal_16m_ctrl_fld;
    __IO uint32_t xtal_16m_ctrl;
  };

  union{ //offset addr 0x00c0
    struct{
      __IO uint32_t sleep_r0:32;
    }sleep_r0_fld;
    __IO uint32_t sleep_r0;
  };

  union{ //offset addr 0x00c4
    struct{
      __IO uint32_t sleep_r1:32;
    }sleep_r1_fld;
    __IO uint32_t sleep_r1;
  };

  union{ //offset addr 0x00c8
    struct{
      __IO uint32_t :32;
    }reserved1_fld;
    __IO uint32_t reserved1;
  };

  union{ //offset addr 0x00cc
    struct{
      __IO uint32_t :32;
    }reserved2_fld;
    __IO uint32_t reserved2;
  };

} PICO_REG_AON_TypeDef;

#define PICO_REG_AON PICO_REG_AON_TypeDef *0x4000F000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_AON_H_

