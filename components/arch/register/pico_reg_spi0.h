#ifndef _PICO_REG_SPI0_H_
#define _PICO_REG_SPI0_H_

#include <stdint.h>

#define SPI0_COUNT 28

#define SPI0_BASE_ADDR 0x40006000

#define SPI0_SIZE 0x000000FC


 /**
 * @brief CTRLR0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:12                  CFS   0b0
 *     11                  SRL   0
 *     10               SLV_OE   0
 *  09:08                 TMOD   0b0
 *     07                SCPOL   0
 *     06                 SCPH   0
 *  05:04                  FRF   0b0
 *  03:00                  DFS   0b7
 * </pre>
 */
#define SPI0_CTRLR0_OFFSET 0x00000000


__INLINE uint32_t spi0_ctrlr0_get(void)
{
    return _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_ctrlr0_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_CTRLR0_CFS_MASK                              ((uint32_t)0x0000F000)
#define SPI0_CTRLR0_CFS_LSB                               12
#define SPI0_CTRLR0_CFS_WIDTH                             ((uint32_t)0x00000004)
#define SPI0_CTRLR0_SRL_BIT                               ((uint32_t)0x00000800)
#define SPI0_CTRLR0_SRL_POS                               11
#define SPI0_CTRLR0_SLV_OE_BIT                            ((uint32_t)0x00000400)
#define SPI0_CTRLR0_SLV_OE_POS                            10
#define SPI0_CTRLR0_TMOD_MASK                             ((uint32_t)0x00000300)
#define SPI0_CTRLR0_TMOD_LSB                              8
#define SPI0_CTRLR0_TMOD_WIDTH                            ((uint32_t)0x00000002)
#define SPI0_CTRLR0_SCPOL_BIT                             ((uint32_t)0x00000080)
#define SPI0_CTRLR0_SCPOL_POS                             7
#define SPI0_CTRLR0_SCPH_BIT                              ((uint32_t)0x00000040)
#define SPI0_CTRLR0_SCPH_POS                              6
#define SPI0_CTRLR0_FRF_MASK                              ((uint32_t)0x00000030)
#define SPI0_CTRLR0_FRF_LSB                               4
#define SPI0_CTRLR0_FRF_WIDTH                             ((uint32_t)0x00000002)
#define SPI0_CTRLR0_DFS_MASK                              ((uint32_t)0x0000000F)
#define SPI0_CTRLR0_DFS_LSB                               0
#define SPI0_CTRLR0_DFS_WIDTH                             ((uint32_t)0x00000004)

#define SPI0_CTRLR0_CFS_RST                               0x0
#define SPI0_CTRLR0_SRL_RST                               0x0
#define SPI0_CTRLR0_SLV_OE_RST                            0x0
#define SPI0_CTRLR0_TMOD_RST                              0x0
#define SPI0_CTRLR0_SCPOL_RST                             0x0
#define SPI0_CTRLR0_SCPH_RST                              0x0
#define SPI0_CTRLR0_FRF_RST                               0x0
#define SPI0_CTRLR0_DFS_RST                               0x7

__INLINE void spi0_ctrlr0_pack(uint8_t cfs, uint8_t srl, uint8_t slvoe, uint8_t tmod, uint8_t scpol, uint8_t scph, uint8_t frf, uint8_t dfs)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)cfs << 12) | ((uint32_t)srl << 11) | ((uint32_t)slvoe << 10) | ((uint32_t)tmod << 8) | ((uint32_t)scpol << 7) | ((uint32_t)scph << 6) | ((uint32_t)frf << 4) | ((uint32_t)dfs << 0));
}

__INLINE void spi0_ctrlr0_unpack(uint8_t* cfs, uint8_t* srl, uint8_t* slvoe, uint8_t* tmod, uint8_t* scpol, uint8_t* scph, uint8_t* frf, uint8_t* dfs)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);

    *cfs = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *srl = (localVal & ((uint32_t)0x00000800)) >> 11;
    *slvoe = (localVal & ((uint32_t)0x00000400)) >> 10;
    *tmod = (localVal & ((uint32_t)0x00000300)) >> 8;
    *scpol = (localVal & ((uint32_t)0x00000080)) >> 7;
    *scph = (localVal & ((uint32_t)0x00000040)) >> 6;
    *frf = (localVal & ((uint32_t)0x00000030)) >> 4;
    *dfs = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t spi0_ctrlr0_cfs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void spi0_ctrlr0_cfs_setf(uint8_t cfs)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)cfs << 12));
}

__INLINE uint8_t spi0_ctrlr0_srl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void spi0_ctrlr0_srl_setf(uint8_t srl)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)srl << 11));
}

__INLINE uint8_t spi0_ctrlr0_slv_oe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void spi0_ctrlr0_slv_oe_setf(uint8_t slvoe)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)slvoe << 10));
}

__INLINE uint8_t spi0_ctrlr0_tmod_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void spi0_ctrlr0_tmod_setf(uint8_t tmod)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)tmod << 8));
}

__INLINE uint8_t spi0_ctrlr0_scpol_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void spi0_ctrlr0_scpol_setf(uint8_t scpol)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)scpol << 7));
}

__INLINE uint8_t spi0_ctrlr0_scph_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void spi0_ctrlr0_scph_setf(uint8_t scph)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)scph << 6));
}

__INLINE uint8_t spi0_ctrlr0_frf_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000030)) >> 4);
}

__INLINE void spi0_ctrlr0_frf_setf(uint8_t frf)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)frf << 4));
}

__INLINE uint8_t spi0_ctrlr0_dfs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void spi0_ctrlr0_dfs_setf(uint8_t dfs)
{
    _PICO_REG_WR(SPI0_CTRLR0_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_CTRLR0_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)dfs << 0));
}

 /**
 * @brief CTRLR1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                  NDF   0b0
 * </pre>
 */
#define SPI0_CTRLR1_OFFSET 0x00000004


__INLINE uint32_t spi0_ctrlr1_get(void)
{
    return _PICO_REG_RD(SPI0_CTRLR1_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_ctrlr1_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_CTRLR1_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_CTRLR1_NDF_MASK                              ((uint32_t)0x0000FFFF)
#define SPI0_CTRLR1_NDF_LSB                               0
#define SPI0_CTRLR1_NDF_WIDTH                             ((uint32_t)0x00000010)

#define SPI0_CTRLR1_NDF_RST                               0x0

__INLINE void spi0_ctrlr1_pack(uint16_t ndf)
{
    _PICO_REG_WR(SPI0_CTRLR1_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)ndf << 0));
}

__INLINE void spi0_ctrlr1_unpack(uint8_t* ndf)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR1_OFFSET + SPI0_BASE_ADDR);

    *ndf = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t spi0_ctrlr1_ndf_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_CTRLR1_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void spi0_ctrlr1_ndf_setf(uint16_t ndf)
{
    _PICO_REG_WR(SPI0_CTRLR1_OFFSET+ SPI0_BASE_ADDR, (uint32_t)ndf << 0);
}

 /**
 * @brief SSIENR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00               SSI_EN   0
 * </pre>
 */
#define SPI0_SSIENR_OFFSET 0x00000008


__INLINE uint32_t spi0_ssienr_get(void)
{
    return _PICO_REG_RD(SPI0_SSIENR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_ssienr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_SSIENR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_SSIENR_SSI_EN_BIT                            ((uint32_t)0x00000001)
#define SPI0_SSIENR_SSI_EN_POS                            0

#define SPI0_SSIENR_SSI_EN_RST                            0x0

__INLINE void spi0_ssienr_pack(uint8_t ssien)
{
    _PICO_REG_WR(SPI0_SSIENR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)ssien << 0));
}

__INLINE void spi0_ssienr_unpack(uint8_t* ssien)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SSIENR_OFFSET + SPI0_BASE_ADDR);

    *ssien = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_ssienr_ssi_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SSIENR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void spi0_ssienr_ssi_en_setf(uint8_t ssien)
{
    _PICO_REG_WR(SPI0_SSIENR_OFFSET+ SPI0_BASE_ADDR, (uint32_t)ssien << 0);
}

 /**
 * @brief MWCR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02                  MHS   0
 *     01                  MDD   0
 *     00                MWMOD   0
 * </pre>
 */
#define SPI0_MWCR_OFFSET 0x0000000C


__INLINE uint32_t spi0_mwcr_get(void)
{
    return _PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_mwcr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_MWCR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_MWCR_MHS_BIT                               ((uint32_t)0x00000004)
#define SPI0_MWCR_MHS_POS                               2
#define SPI0_MWCR_MDD_BIT                               ((uint32_t)0x00000002)
#define SPI0_MWCR_MDD_POS                               1
#define SPI0_MWCR_MWMOD_BIT                             ((uint32_t)0x00000001)
#define SPI0_MWCR_MWMOD_POS                             0

#define SPI0_MWCR_MHS_RST                               0x0
#define SPI0_MWCR_MDD_RST                               0x0
#define SPI0_MWCR_MWMOD_RST                             0x0

__INLINE void spi0_mwcr_pack(uint8_t mhs, uint8_t mdd, uint8_t mwmod)
{
    _PICO_REG_WR(SPI0_MWCR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)mhs << 2) | ((uint32_t)mdd << 1) | ((uint32_t)mwmod << 0));
}

__INLINE void spi0_mwcr_unpack(uint8_t* mhs, uint8_t* mdd, uint8_t* mwmod)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR);

    *mhs = (localVal & ((uint32_t)0x00000004)) >> 2;
    *mdd = (localVal & ((uint32_t)0x00000002)) >> 1;
    *mwmod = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_mwcr_mhs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void spi0_mwcr_mhs_setf(uint8_t mhs)
{
    _PICO_REG_WR(SPI0_MWCR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)mhs << 2));
}

__INLINE uint8_t spi0_mwcr_mdd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void spi0_mwcr_mdd_setf(uint8_t mdd)
{
    _PICO_REG_WR(SPI0_MWCR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)mdd << 1));
}

__INLINE uint8_t spi0_mwcr_mwmod_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void spi0_mwcr_mwmod_setf(uint8_t mwmod)
{
    _PICO_REG_WR(SPI0_MWCR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_MWCR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)mwmod << 0));
}

 /**
 * @brief SER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00                  SER   0
 * </pre>
 */
#define SPI0_SER_OFFSET 0x00000010


__INLINE uint32_t spi0_ser_get(void)
{
    return _PICO_REG_RD(SPI0_SER_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_ser_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_SER_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_SER_SER_BIT                               ((uint32_t)0x00000001)
#define SPI0_SER_SER_POS                               0

#define SPI0_SER_SER_RST                               0x0

__INLINE void spi0_ser_pack(uint8_t ser)
{
    _PICO_REG_WR(SPI0_SER_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)ser << 0));
}

__INLINE void spi0_ser_unpack(uint8_t* ser)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SER_OFFSET + SPI0_BASE_ADDR);

    *ser = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_ser_ser_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SER_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void spi0_ser_ser_setf(uint8_t ser)
{
    _PICO_REG_WR(SPI0_SER_OFFSET+ SPI0_BASE_ADDR, (uint32_t)ser << 0);
}

 /**
 * @brief BAUDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                SCKDV   0b0
 * </pre>
 */
#define SPI0_BAUDR_OFFSET 0x00000014


__INLINE uint32_t spi0_baudr_get(void)
{
    return _PICO_REG_RD(SPI0_BAUDR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_baudr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_BAUDR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_BAUDR_SCKDV_MASK                            ((uint32_t)0x0000FFFF)
#define SPI0_BAUDR_SCKDV_LSB                             0
#define SPI0_BAUDR_SCKDV_WIDTH                           ((uint32_t)0x00000010)

#define SPI0_BAUDR_SCKDV_RST                             0x0

__INLINE void spi0_baudr_pack(uint16_t sckdv)
{
    _PICO_REG_WR(SPI0_BAUDR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)sckdv << 0));
}

__INLINE void spi0_baudr_unpack(uint8_t* sckdv)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_BAUDR_OFFSET + SPI0_BASE_ADDR);

    *sckdv = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t spi0_baudr_sckdv_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_BAUDR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void spi0_baudr_sckdv_setf(uint16_t sckdv)
{
    _PICO_REG_WR(SPI0_BAUDR_OFFSET+ SPI0_BASE_ADDR, (uint32_t)sckdv << 0);
}

 /**
 * @brief TXFTLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00                  TFT   0b0
 * </pre>
 */
#define SPI0_TXFTLR_OFFSET 0x00000018


__INLINE uint32_t spi0_txftlr_get(void)
{
    return _PICO_REG_RD(SPI0_TXFTLR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_txftlr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_TXFTLR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_TXFTLR_TFT_MASK                              ((uint32_t)0x00000007)
#define SPI0_TXFTLR_TFT_LSB                               0
#define SPI0_TXFTLR_TFT_WIDTH                             ((uint32_t)0x00000003)

#define SPI0_TXFTLR_TFT_RST                               0x0

__INLINE void spi0_txftlr_pack(uint8_t tft)
{
    _PICO_REG_WR(SPI0_TXFTLR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)tft << 0));
}

__INLINE void spi0_txftlr_unpack(uint8_t* tft)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_TXFTLR_OFFSET + SPI0_BASE_ADDR);

    *tft = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t spi0_txftlr_tft_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_TXFTLR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void spi0_txftlr_tft_setf(uint8_t tft)
{
    _PICO_REG_WR(SPI0_TXFTLR_OFFSET+ SPI0_BASE_ADDR, (uint32_t)tft << 0);
}

 /**
 * @brief RXFTLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00                  RFT   0b0
 * </pre>
 */
#define SPI0_RXFTLR_OFFSET 0x0000001C


__INLINE uint32_t spi0_rxftlr_get(void)
{
    return _PICO_REG_RD(SPI0_RXFTLR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_rxftlr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_RXFTLR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_RXFTLR_RFT_MASK                              ((uint32_t)0x00000007)
#define SPI0_RXFTLR_RFT_LSB                               0
#define SPI0_RXFTLR_RFT_WIDTH                             ((uint32_t)0x00000003)

#define SPI0_RXFTLR_RFT_RST                               0x0

__INLINE void spi0_rxftlr_pack(uint8_t rft)
{
    _PICO_REG_WR(SPI0_RXFTLR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)rft << 0));
}

__INLINE void spi0_rxftlr_unpack(uint8_t* rft)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXFTLR_OFFSET + SPI0_BASE_ADDR);

    *rft = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t spi0_rxftlr_rft_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXFTLR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void spi0_rxftlr_rft_setf(uint8_t rft)
{
    _PICO_REG_WR(SPI0_RXFTLR_OFFSET+ SPI0_BASE_ADDR, (uint32_t)rft << 0);
}

 /**
 * @brief TXFLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00                TXTFL   0b0
 * </pre>
 */
#define SPI0_TXFLR_OFFSET 0x00000020


__INLINE uint32_t spi0_txflr_get(void)
{
    return _PICO_REG_RD(SPI0_TXFLR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_TXFLR_TXTFL_MASK                            ((uint32_t)0x0000000F)
#define SPI0_TXFLR_TXTFL_LSB                             0
#define SPI0_TXFLR_TXTFL_WIDTH                           ((uint32_t)0x00000004)

#define SPI0_TXFLR_TXTFL_RST                             0x0

__INLINE void spi0_txflr_unpack(uint8_t* txtfl)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_TXFLR_OFFSET + SPI0_BASE_ADDR);

    *txtfl = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t spi0_txflr_txtfl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_TXFLR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief RXFLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00                RXTFL   0b0
 * </pre>
 */
#define SPI0_RXFLR_OFFSET 0x00000024


__INLINE uint32_t spi0_rxflr_get(void)
{
    return _PICO_REG_RD(SPI0_RXFLR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_RXFLR_RXTFL_MASK                            ((uint32_t)0x0000000F)
#define SPI0_RXFLR_RXTFL_LSB                             0
#define SPI0_RXFLR_RXTFL_WIDTH                           ((uint32_t)0x00000004)

#define SPI0_RXFLR_RXTFL_RST                             0x0

__INLINE void spi0_rxflr_unpack(uint8_t* rxtfl)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXFLR_OFFSET + SPI0_BASE_ADDR);

    *rxtfl = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t spi0_rxflr_rxtfl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXFLR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief SR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06                 DCOL   0
 *     05                  TXE   0
 *     04                  RFF   0
 *     03                 RFNE   0
 *     02                  TFE   1
 *     01                 TFNF   1
 *     00                 BUSY   0
 * </pre>
 */
#define SPI0_SR_OFFSET 0x00000028


__INLINE uint32_t spi0_sr_get(void)
{
    return _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_SR_DCOL_BIT                              ((uint32_t)0x00000040)
#define SPI0_SR_DCOL_POS                              6
#define SPI0_SR_TXE_BIT                               ((uint32_t)0x00000020)
#define SPI0_SR_TXE_POS                               5
#define SPI0_SR_RFF_BIT                               ((uint32_t)0x00000010)
#define SPI0_SR_RFF_POS                               4
#define SPI0_SR_RFNE_BIT                              ((uint32_t)0x00000008)
#define SPI0_SR_RFNE_POS                              3
#define SPI0_SR_TFE_BIT                               ((uint32_t)0x00000004)
#define SPI0_SR_TFE_POS                               2
#define SPI0_SR_TFNF_BIT                              ((uint32_t)0x00000002)
#define SPI0_SR_TFNF_POS                              1
#define SPI0_SR_BUSY_BIT                              ((uint32_t)0x00000001)
#define SPI0_SR_BUSY_POS                              0

#define SPI0_SR_DCOL_RST                              0x0
#define SPI0_SR_TXE_RST                               0x0
#define SPI0_SR_RFF_RST                               0x0
#define SPI0_SR_RFNE_RST                              0x0
#define SPI0_SR_TFE_RST                               0x1
#define SPI0_SR_TFNF_RST                              0x1
#define SPI0_SR_BUSY_RST                              0x0

__INLINE void spi0_sr_unpack(uint8_t* dcol, uint8_t* txe, uint8_t* rff, uint8_t* rfne, uint8_t* tfe, uint8_t* tfnf, uint8_t* busy)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);

    *dcol = (localVal & ((uint32_t)0x00000040)) >> 6;
    *txe = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rff = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rfne = (localVal & ((uint32_t)0x00000008)) >> 3;
    *tfe = (localVal & ((uint32_t)0x00000004)) >> 2;
    *tfnf = (localVal & ((uint32_t)0x00000002)) >> 1;
    *busy = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_sr_dcol_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t spi0_sr_txe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t spi0_sr_rff_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t spi0_sr_rfne_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t spi0_sr_tfe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t spi0_sr_tfnf_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t spi0_sr_busy_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IMR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     05                MSTIM   1
 *     04                RXFIM   1
 *     03                RXOIM   1
 *     02                RXUIM   1
 *     01                TXOIM   1
 *     00                TXEIM   1
 * </pre>
 */
#define SPI0_IMR_OFFSET 0x0000002C


__INLINE uint32_t spi0_imr_get(void)
{
    return _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_imr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_IMR_MSTIM_BIT                             ((uint32_t)0x00000020)
#define SPI0_IMR_MSTIM_POS                             5
#define SPI0_IMR_RXFIM_BIT                             ((uint32_t)0x00000010)
#define SPI0_IMR_RXFIM_POS                             4
#define SPI0_IMR_RXOIM_BIT                             ((uint32_t)0x00000008)
#define SPI0_IMR_RXOIM_POS                             3
#define SPI0_IMR_RXUIM_BIT                             ((uint32_t)0x00000004)
#define SPI0_IMR_RXUIM_POS                             2
#define SPI0_IMR_TXOIM_BIT                             ((uint32_t)0x00000002)
#define SPI0_IMR_TXOIM_POS                             1
#define SPI0_IMR_TXEIM_BIT                             ((uint32_t)0x00000001)
#define SPI0_IMR_TXEIM_POS                             0

#define SPI0_IMR_MSTIM_RST                             0x1
#define SPI0_IMR_RXFIM_RST                             0x1
#define SPI0_IMR_RXOIM_RST                             0x1
#define SPI0_IMR_RXUIM_RST                             0x1
#define SPI0_IMR_TXOIM_RST                             0x1
#define SPI0_IMR_TXEIM_RST                             0x1

__INLINE void spi0_imr_pack(uint8_t mstim, uint8_t rxfim, uint8_t rxoim, uint8_t rxuim, uint8_t txoim, uint8_t txeim)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)mstim << 5) | ((uint32_t)rxfim << 4) | ((uint32_t)rxoim << 3) | ((uint32_t)rxuim << 2) | ((uint32_t)txoim << 1) | ((uint32_t)txeim << 0));
}

__INLINE void spi0_imr_unpack(uint8_t* mstim, uint8_t* rxfim, uint8_t* rxoim, uint8_t* rxuim, uint8_t* txoim, uint8_t* txeim)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);

    *mstim = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxfim = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxoim = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxuim = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txoim = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txeim = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_imr_mstim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void spi0_imr_mstim_setf(uint8_t mstim)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)mstim << 5));
}

__INLINE uint8_t spi0_imr_rxfim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void spi0_imr_rxfim_setf(uint8_t rxfim)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxfim << 4));
}

__INLINE uint8_t spi0_imr_rxoim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void spi0_imr_rxoim_setf(uint8_t rxoim)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rxoim << 3));
}

__INLINE uint8_t spi0_imr_rxuim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void spi0_imr_rxuim_setf(uint8_t rxuim)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rxuim << 2));
}

__INLINE uint8_t spi0_imr_txoim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void spi0_imr_txoim_setf(uint8_t txoim)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)txoim << 1));
}

__INLINE uint8_t spi0_imr_txeim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void spi0_imr_txeim_setf(uint8_t txeim)
{
    _PICO_REG_WR(SPI0_IMR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_IMR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txeim << 0));
}

 /**
 * @brief ISR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     05                MSTIS   0
 *     04                RXFIS   0
 *     03                RXOIS   0
 *     02                RXUIS   0
 *     01                TXOIS   0
 *     00                TXEIS   0
 * </pre>
 */
#define SPI0_ISR_OFFSET 0x00000030


__INLINE uint32_t spi0_isr_get(void)
{
    return _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_ISR_MSTIS_BIT                             ((uint32_t)0x00000020)
#define SPI0_ISR_MSTIS_POS                             5
#define SPI0_ISR_RXFIS_BIT                             ((uint32_t)0x00000010)
#define SPI0_ISR_RXFIS_POS                             4
#define SPI0_ISR_RXOIS_BIT                             ((uint32_t)0x00000008)
#define SPI0_ISR_RXOIS_POS                             3
#define SPI0_ISR_RXUIS_BIT                             ((uint32_t)0x00000004)
#define SPI0_ISR_RXUIS_POS                             2
#define SPI0_ISR_TXOIS_BIT                             ((uint32_t)0x00000002)
#define SPI0_ISR_TXOIS_POS                             1
#define SPI0_ISR_TXEIS_BIT                             ((uint32_t)0x00000001)
#define SPI0_ISR_TXEIS_POS                             0

#define SPI0_ISR_MSTIS_RST                             0x0
#define SPI0_ISR_RXFIS_RST                             0x0
#define SPI0_ISR_RXOIS_RST                             0x0
#define SPI0_ISR_RXUIS_RST                             0x0
#define SPI0_ISR_TXOIS_RST                             0x0
#define SPI0_ISR_TXEIS_RST                             0x0

__INLINE void spi0_isr_unpack(uint8_t* mstis, uint8_t* rxfis, uint8_t* rxois, uint8_t* rxuis, uint8_t* txois, uint8_t* txeis)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);

    *mstis = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxfis = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxois = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxuis = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txois = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txeis = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_isr_mstis_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t spi0_isr_rxfis_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t spi0_isr_rxois_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t spi0_isr_rxuis_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t spi0_isr_txois_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t spi0_isr_txeis_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief RISR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     05                MSTIR   0
 *     04                RXFIR   0
 *     03                RXOIR   0
 *     02                RXUIR   0
 *     01                TXOIR   0
 *     00                TXEIR   0
 * </pre>
 */
#define SPI0_RISR_OFFSET 0x00000034


__INLINE uint32_t spi0_risr_get(void)
{
    return _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_RISR_MSTIR_BIT                             ((uint32_t)0x00000020)
#define SPI0_RISR_MSTIR_POS                             5
#define SPI0_RISR_RXFIR_BIT                             ((uint32_t)0x00000010)
#define SPI0_RISR_RXFIR_POS                             4
#define SPI0_RISR_RXOIR_BIT                             ((uint32_t)0x00000008)
#define SPI0_RISR_RXOIR_POS                             3
#define SPI0_RISR_RXUIR_BIT                             ((uint32_t)0x00000004)
#define SPI0_RISR_RXUIR_POS                             2
#define SPI0_RISR_TXOIR_BIT                             ((uint32_t)0x00000002)
#define SPI0_RISR_TXOIR_POS                             1
#define SPI0_RISR_TXEIR_BIT                             ((uint32_t)0x00000001)
#define SPI0_RISR_TXEIR_POS                             0

#define SPI0_RISR_MSTIR_RST                             0x0
#define SPI0_RISR_RXFIR_RST                             0x0
#define SPI0_RISR_RXOIR_RST                             0x0
#define SPI0_RISR_RXUIR_RST                             0x0
#define SPI0_RISR_TXOIR_RST                             0x0
#define SPI0_RISR_TXEIR_RST                             0x0

__INLINE void spi0_risr_unpack(uint8_t* mstir, uint8_t* rxfir, uint8_t* rxoir, uint8_t* rxuir, uint8_t* txoir, uint8_t* txeir)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);

    *mstir = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxfir = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxoir = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxuir = (localVal & ((uint32_t)0x00000004)) >> 2;
    *txoir = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txeir = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_risr_mstir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t spi0_risr_rxfir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t spi0_risr_rxoir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t spi0_risr_rxuir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t spi0_risr_txoir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t spi0_risr_txeir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RISR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TXOICR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00               TXOICR   0
 * </pre>
 */
#define SPI0_TXOICR_OFFSET 0x00000038


__INLINE uint32_t spi0_txoicr_get(void)
{
    return _PICO_REG_RD(SPI0_TXOICR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_TXOICR_TXOICR_BIT                            ((uint32_t)0x00000001)
#define SPI0_TXOICR_TXOICR_POS                            0

#define SPI0_TXOICR_TXOICR_RST                            0x0

__INLINE void spi0_txoicr_unpack(uint8_t* txoicr)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_TXOICR_OFFSET + SPI0_BASE_ADDR);

    *txoicr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_txoicr_txoicr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_TXOICR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief RXOICR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00               RXOICR   0
 * </pre>
 */
#define SPI0_RXOICR_OFFSET 0x0000003C


__INLINE uint32_t spi0_rxoicr_get(void)
{
    return _PICO_REG_RD(SPI0_RXOICR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_RXOICR_RXOICR_BIT                            ((uint32_t)0x00000001)
#define SPI0_RXOICR_RXOICR_POS                            0

#define SPI0_RXOICR_RXOICR_RST                            0x0

__INLINE void spi0_rxoicr_unpack(uint8_t* rxoicr)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXOICR_OFFSET + SPI0_BASE_ADDR);

    *rxoicr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_rxoicr_rxoicr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXOICR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief RXUICR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00               RXUICR   0
 * </pre>
 */
#define SPI0_RXUICR_OFFSET 0x00000040


__INLINE uint32_t spi0_rxuicr_get(void)
{
    return _PICO_REG_RD(SPI0_RXUICR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_RXUICR_RXUICR_BIT                            ((uint32_t)0x00000001)
#define SPI0_RXUICR_RXUICR_POS                            0

#define SPI0_RXUICR_RXUICR_RST                            0x0

__INLINE void spi0_rxuicr_unpack(uint8_t* rxuicr)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXUICR_OFFSET + SPI0_BASE_ADDR);

    *rxuicr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_rxuicr_rxuicr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RXUICR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief MSTICR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00               MSTICR   0
 * </pre>
 */
#define SPI0_MSTICR_OFFSET 0x00000044


__INLINE uint32_t spi0_msticr_get(void)
{
    return _PICO_REG_RD(SPI0_MSTICR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_MSTICR_MSTICR_BIT                            ((uint32_t)0x00000001)
#define SPI0_MSTICR_MSTICR_POS                            0

#define SPI0_MSTICR_MSTICR_RST                            0x0

__INLINE void spi0_msticr_unpack(uint8_t* msticr)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_MSTICR_OFFSET + SPI0_BASE_ADDR);

    *msticr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_msticr_msticr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_MSTICR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief ICR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00                  ICR   0
 * </pre>
 */
#define SPI0_ICR_OFFSET 0x00000048


__INLINE uint32_t spi0_icr_get(void)
{
    return _PICO_REG_RD(SPI0_ICR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_ICR_ICR_BIT                               ((uint32_t)0x00000001)
#define SPI0_ICR_ICR_POS                               0

#define SPI0_ICR_ICR_RST                               0x0

__INLINE void spi0_icr_unpack(uint8_t* icr)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ICR_OFFSET + SPI0_BASE_ADDR);

    *icr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_icr_icr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_ICR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief DMACR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01                TDMAE   0
 *     00                RDMAE   0
 * </pre>
 */
#define SPI0_DMACR_OFFSET 0x0000004C


__INLINE uint32_t spi0_dmacr_get(void)
{
    return _PICO_REG_RD(SPI0_DMACR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_dmacr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_DMACR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_DMACR_TDMAE_BIT                             ((uint32_t)0x00000002)
#define SPI0_DMACR_TDMAE_POS                             1
#define SPI0_DMACR_RDMAE_BIT                             ((uint32_t)0x00000001)
#define SPI0_DMACR_RDMAE_POS                             0

#define SPI0_DMACR_TDMAE_RST                             0x0
#define SPI0_DMACR_RDMAE_RST                             0x0

__INLINE void spi0_dmacr_pack(uint8_t tdmae, uint8_t rdmae)
{
    _PICO_REG_WR(SPI0_DMACR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)tdmae << 1) | ((uint32_t)rdmae << 0));
}

__INLINE void spi0_dmacr_unpack(uint8_t* tdmae, uint8_t* rdmae)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DMACR_OFFSET + SPI0_BASE_ADDR);

    *tdmae = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rdmae = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t spi0_dmacr_tdmae_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DMACR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void spi0_dmacr_tdmae_setf(uint8_t tdmae)
{
    _PICO_REG_WR(SPI0_DMACR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_DMACR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)tdmae << 1));
}

__INLINE uint8_t spi0_dmacr_rdmae_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DMACR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void spi0_dmacr_rdmae_setf(uint8_t rdmae)
{
    _PICO_REG_WR(SPI0_DMACR_OFFSET+ SPI0_BASE_ADDR, (_PICO_REG_RD(SPI0_DMACR_OFFSET + SPI0_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rdmae << 0));
}

 /**
 * @brief DMATDLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00               DMATDL   0b0
 * </pre>
 */
#define SPI0_DMATDLR_OFFSET 0x00000050


__INLINE uint32_t spi0_dmatdlr_get(void)
{
    return _PICO_REG_RD(SPI0_DMATDLR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_dmatdlr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_DMATDLR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_DMATDLR_DMATDL_MASK                           ((uint32_t)0x00000007)
#define SPI0_DMATDLR_DMATDL_LSB                            0
#define SPI0_DMATDLR_DMATDL_WIDTH                          ((uint32_t)0x00000003)

#define SPI0_DMATDLR_DMATDL_RST                            0x0

__INLINE void spi0_dmatdlr_pack(uint8_t dmatdl)
{
    _PICO_REG_WR(SPI0_DMATDLR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)dmatdl << 0));
}

__INLINE void spi0_dmatdlr_unpack(uint8_t* dmatdl)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DMATDLR_OFFSET + SPI0_BASE_ADDR);

    *dmatdl = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t spi0_dmatdlr_dmatdl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DMATDLR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void spi0_dmatdlr_dmatdl_setf(uint8_t dmatdl)
{
    _PICO_REG_WR(SPI0_DMATDLR_OFFSET+ SPI0_BASE_ADDR, (uint32_t)dmatdl << 0);
}

 /**
 * @brief DMARDLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00               DMARDL   0b0
 * </pre>
 */
#define SPI0_DMARDLR_OFFSET 0x00000054


__INLINE uint32_t spi0_dmardlr_get(void)
{
    return _PICO_REG_RD(SPI0_DMARDLR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_dmardlr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_DMARDLR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_DMARDLR_DMARDL_MASK                           ((uint32_t)0x00000007)
#define SPI0_DMARDLR_DMARDL_LSB                            0
#define SPI0_DMARDLR_DMARDL_WIDTH                          ((uint32_t)0x00000003)

#define SPI0_DMARDLR_DMARDL_RST                            0x0

__INLINE void spi0_dmardlr_pack(uint8_t dmardl)
{
    _PICO_REG_WR(SPI0_DMARDLR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)dmardl << 0));
}

__INLINE void spi0_dmardlr_unpack(uint8_t* dmardl)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DMARDLR_OFFSET + SPI0_BASE_ADDR);

    *dmardl = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t spi0_dmardlr_dmardl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DMARDLR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void spi0_dmardlr_dmardl_setf(uint8_t dmardl)
{
    _PICO_REG_WR(SPI0_DMARDLR_OFFSET+ SPI0_BASE_ADDR, (uint32_t)dmardl << 0);
}

 /**
 * @brief IDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               IDCODE   0b0
 * </pre>
 */
#define SPI0_IDR_OFFSET 0x00000058


__INLINE uint32_t spi0_idr_get(void)
{
    return _PICO_REG_RD(SPI0_IDR_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_IDR_IDCODE_MASK                           ((uint32_t)0xFFFFFFFF)
#define SPI0_IDR_IDCODE_LSB                            0
#define SPI0_IDR_IDCODE_WIDTH                          ((uint32_t)0x00000020)

#define SPI0_IDR_IDCODE_RST                            0x0

__INLINE void spi0_idr_unpack(uint8_t* idcode)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IDR_OFFSET + SPI0_BASE_ADDR);

    *idcode = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t spi0_idr_idcode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_IDR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief SSI_COMP_VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00     SSI_COMP_VERSION   0b0
 * </pre>
 */
#define SPI0_SSI_COMP_VERSION_OFFSET 0x0000005C


__INLINE uint32_t spi0_ssi_comp_version_get(void)
{
    return _PICO_REG_RD(SPI0_SSI_COMP_VERSION_OFFSET + SPI0_BASE_ADDR);
}

// field definitions
#define SPI0_SSI_COMP_VERSION_SSI_COMP_VERSION_MASK                 ((uint32_t)0xFFFFFFFF)
#define SPI0_SSI_COMP_VERSION_SSI_COMP_VERSION_LSB                  0
#define SPI0_SSI_COMP_VERSION_SSI_COMP_VERSION_WIDTH                ((uint32_t)0x00000020)

#define SPI0_SSI_COMP_VERSION_SSI_COMP_VERSION_RST                  0x0

__INLINE void spi0_ssi_comp_version_unpack(uint8_t* ssicompversion)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SSI_COMP_VERSION_OFFSET + SPI0_BASE_ADDR);

    *ssicompversion = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t spi0_ssi_comp_version_ssi_comp_version_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_SSI_COMP_VERSION_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief DR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                   DR   0b0
 * </pre>
 */
#define SPI0_DR_OFFSET 0x00000060


__INLINE uint32_t spi0_dr_get(void)
{
    return _PICO_REG_RD(SPI0_DR_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_dr_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_DR_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_DR_DR_MASK                               ((uint32_t)0x0000FFFF)
#define SPI0_DR_DR_LSB                                0
#define SPI0_DR_DR_WIDTH                              ((uint32_t)0x00000010)

#define SPI0_DR_DR_RST                                0x0

__INLINE void spi0_dr_pack(uint16_t dr)
{
    _PICO_REG_WR(SPI0_DR_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)dr << 0));
}

__INLINE void spi0_dr_unpack(uint8_t* dr)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DR_OFFSET + SPI0_BASE_ADDR);

    *dr = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t spi0_dr_dr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_DR_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void spi0_dr_dr_setf(uint16_t dr)
{
    _PICO_REG_WR(SPI0_DR_OFFSET+ SPI0_BASE_ADDR, (uint32_t)dr << 0);
}

 /**
 * @brief RSVD0 register definition
 */
#define SPI0_RSVD0_OFFSET 0x000000F4


__INLINE uint32_t spi0_rsvd0_get(void)
{
    return _PICO_REG_RD(SPI0_RSVD0_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_rsvd0_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_RSVD0_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_RSVD0_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define SPI0_RSVD0_RESERVED_LSB                          0
#define SPI0_RSVD0_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define SPI0_RSVD0_RESERVED_RST                          0x0

__INLINE void spi0_rsvd0_pack(uint32_t reserved)
{
    _PICO_REG_WR(SPI0_RSVD0_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)reserved << 0));
}

__INLINE void spi0_rsvd0_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RSVD0_OFFSET + SPI0_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief RSVD1 register definition
 */
#define SPI0_RSVD1_OFFSET 0x000000F8


__INLINE uint32_t spi0_rsvd1_get(void)
{
    return _PICO_REG_RD(SPI0_RSVD1_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_rsvd1_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_RSVD1_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_RSVD1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define SPI0_RSVD1_RESERVED_LSB                          0
#define SPI0_RSVD1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define SPI0_RSVD1_RESERVED_RST                          0x0

__INLINE void spi0_rsvd1_pack(uint32_t reserved)
{
    _PICO_REG_WR(SPI0_RSVD1_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)reserved << 0));
}

__INLINE void spi0_rsvd1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RSVD1_OFFSET + SPI0_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief RX_SAMPLE_DLY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                  RSD   0b0
 * </pre>
 */
#define SPI0_RX_SAMPLE_DLY_OFFSET 0x000000FC


__INLINE uint32_t spi0_rx_sample_dly_get(void)
{
    return _PICO_REG_RD(SPI0_RX_SAMPLE_DLY_OFFSET + SPI0_BASE_ADDR);
}

__INLINE void spi0_rx_sample_dly_set(uint32_t value)
{
    _PICO_REG_WR(SPI0_RX_SAMPLE_DLY_OFFSET+ SPI0_BASE_ADDR, value);
}

// field definitions
#define SPI0_RX_SAMPLE_DLY_RSD_MASK                              ((uint32_t)0x000000FF)
#define SPI0_RX_SAMPLE_DLY_RSD_LSB                               0
#define SPI0_RX_SAMPLE_DLY_RSD_WIDTH                             ((uint32_t)0x00000008)

#define SPI0_RX_SAMPLE_DLY_RSD_RST                               0x0

__INLINE void spi0_rx_sample_dly_pack(uint8_t rsd)
{
    _PICO_REG_WR(SPI0_RX_SAMPLE_DLY_OFFSET+ SPI0_BASE_ADDR,  ((uint32_t)rsd << 0));
}

__INLINE void spi0_rx_sample_dly_unpack(uint8_t* rsd)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RX_SAMPLE_DLY_OFFSET + SPI0_BASE_ADDR);

    *rsd = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t spi0_rx_sample_dly_rsd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(SPI0_RX_SAMPLE_DLY_OFFSET + SPI0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void spi0_rx_sample_dly_rsd_setf(uint8_t rsd)
{
    _PICO_REG_WR(SPI0_RX_SAMPLE_DLY_OFFSET+ SPI0_BASE_ADDR, (uint32_t)rsd << 0);
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :16;
      __IO uint32_t cfs:4;
      __IO uint32_t srl:1;
      __IO uint32_t slv_oe:1;
      __IO uint32_t tmod:2;
      __IO uint32_t scpol:1;
      __IO uint32_t scph:1;
      __IO uint32_t frf:2;
      __IO uint32_t dfs:4;
    }CTRLR0_fld;
    __IO uint32_t CTRLR0;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :16;
      __IO uint32_t ndf:16;
    }CTRLR1_fld;
    __IO uint32_t CTRLR1;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :31;
      __IO uint32_t ssi_en:1;
    }SSIENR_fld;
    __IO uint32_t SSIENR;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :29;
      __IO uint32_t mhs:1;
      __IO uint32_t mdd:1;
      __IO uint32_t mwmod:1;
    }MWCR_fld;
    __IO uint32_t MWCR;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :31;
      __IO uint32_t ser:1;
    }SER_fld;
    __IO uint32_t SER;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :16;
      __IO uint32_t sckdv:16;
    }BAUDR_fld;
    __IO uint32_t BAUDR;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :29;
      __IO uint32_t tft:3;
    }TXFTLR_fld;
    __IO uint32_t TXFTLR;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :29;
      __IO uint32_t rft:3;
    }RXFTLR_fld;
    __IO uint32_t RXFTLR;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t :28;
      __IO uint32_t txtfl:4;
    }TXFLR_fld;
    __IO uint32_t TXFLR;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :28;
      __IO uint32_t rxtfl:4;
    }RXFLR_fld;
    __IO uint32_t RXFLR;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t :25;
      __IO uint32_t dcol:1;
      __IO uint32_t txe:1;
      __IO uint32_t rff:1;
      __IO uint32_t rfne:1;
      __IO uint32_t tfe:1;
      __IO uint32_t tfnf:1;
      __IO uint32_t busy:1;
    }SR_fld;
    __IO uint32_t SR;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t :26;
      __IO uint32_t mstim:1;
      __IO uint32_t rxfim:1;
      __IO uint32_t rxoim:1;
      __IO uint32_t rxuim:1;
      __IO uint32_t txoim:1;
      __IO uint32_t txeim:1;
    }IMR_fld;
    __IO uint32_t IMR;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :26;
      __IO uint32_t mstis:1;
      __IO uint32_t rxfis:1;
      __IO uint32_t rxois:1;
      __IO uint32_t rxuis:1;
      __IO uint32_t txois:1;
      __IO uint32_t txeis:1;
    }ISR_fld;
    __IO uint32_t ISR;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t :26;
      __IO uint32_t mstir:1;
      __IO uint32_t rxfir:1;
      __IO uint32_t rxoir:1;
      __IO uint32_t rxuir:1;
      __IO uint32_t txoir:1;
      __IO uint32_t txeir:1;
    }RISR_fld;
    __IO uint32_t RISR;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t :31;
      __IO uint32_t txoicr:1;
    }TXOICR_fld;
    __IO uint32_t TXOICR;
  };

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t :31;
      __IO uint32_t rxoicr:1;
    }RXOICR_fld;
    __IO uint32_t RXOICR;
  };

  union{ //offset addr 0x0040
    struct{
      __IO uint32_t :31;
      __IO uint32_t rxuicr:1;
    }RXUICR_fld;
    __IO uint32_t RXUICR;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t :31;
      __IO uint32_t msticr:1;
    }MSTICR_fld;
    __IO uint32_t MSTICR;
  };

  union{ //offset addr 0x0048
    struct{
      __IO uint32_t :31;
      __IO uint32_t icr:1;
    }ICR_fld;
    __IO uint32_t ICR;
  };

  union{ //offset addr 0x004c
    struct{
      __IO uint32_t :30;
      __IO uint32_t tdmae:1;
      __IO uint32_t rdmae:1;
    }DMACR_fld;
    __IO uint32_t DMACR;
  };

  union{ //offset addr 0x0050
    struct{
      __IO uint32_t :29;
      __IO uint32_t dmatdl:3;
    }DMATDLR_fld;
    __IO uint32_t DMATDLR;
  };

  union{ //offset addr 0x0054
    struct{
      __IO uint32_t :29;
      __IO uint32_t dmardl:3;
    }DMARDLR_fld;
    __IO uint32_t DMARDLR;
  };

  union{ //offset addr 0x0058
    struct{
      __IO uint32_t idcode:32;
    }IDR_fld;
    __IO uint32_t IDR;
  };

  union{ //offset addr 0x005c
    struct{
      __IO uint32_t ssi_comp_version:32;
    }SSI_COMP_VERSION_fld;
    __IO uint32_t SSI_COMP_VERSION;
  };

  union{ //offset addr 0x0060
    struct{
      __IO uint32_t :16;
      __IO uint32_t dr:16;
    }DR_fld;
    __IO uint32_t DR;
  };


  union{ //offset addr 0x00f4
    struct{
      __IO uint32_t :32;
    }RSVD0_fld;
    __IO uint32_t RSVD0;
  };

  union{ //offset addr 0x00f8
    struct{
      __IO uint32_t :32;
    }RSVD1_fld;
    __IO uint32_t RSVD1;
  };

  union{ //offset addr 0x00fc
    struct{
      __IO uint32_t :24;
      __IO uint32_t rsd:8;
    }RX_SAMPLE_DLY_fld;
    __IO uint32_t RX_SAMPLE_DLY;
  };

} PICO_REG_SPI0_TypeDef;

#define PICO_REG_SPI0 PICO_REG_SPI0_TypeDef *0x40006000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_SPI0_H_

