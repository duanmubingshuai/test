#ifndef _PICO_REG_PCRM_H_
#define _PICO_REG_PCRM_H_

#include <stdint.h>

#define PCRM_COUNT 23

#define PCRM_BASE_ADDR 0x4000F000

#define PCRM_SIZE 0x00000154


 /**
 * @brief CLKSEL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24     spif_ref_clk_sel   0b0
 *     17         aon_pclk_inv   0
 *     16           lowclk_sel   0
 *     08              sel_16M   0
 *     06         clk_1p28m_en   1
 *     04   hclk_mux_done_override   1
 *     03   hclk_sel_en_override   1
 *  02:00             hclk_sel   0b0
 * </pre>
 */
#define PCRM_CLKSEL_OFFSET 0x0000003C


__INLINE uint32_t pcrm_clksel_get(void)
{
    return _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_clksel_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_CLKSEL_SPIF_REF_CLK_SEL_MASK                 ((uint32_t)0x07000000)
#define PCRM_CLKSEL_SPIF_REF_CLK_SEL_LSB                  24
#define PCRM_CLKSEL_SPIF_REF_CLK_SEL_WIDTH                ((uint32_t)0x00000003)
#define PCRM_CLKSEL_AON_PCLK_INV_BIT                      ((uint32_t)0x00020000)
#define PCRM_CLKSEL_AON_PCLK_INV_POS                      17
#define PCRM_CLKSEL_LOWCLK_SEL_BIT                        ((uint32_t)0x00010000)
#define PCRM_CLKSEL_LOWCLK_SEL_POS                        16
#define PCRM_CLKSEL_SEL_16M_BIT                           ((uint32_t)0x00000100)
#define PCRM_CLKSEL_SEL_16M_POS                           8
#define PCRM_CLKSEL_CLK_1P28M_EN_BIT                      ((uint32_t)0x00000040)
#define PCRM_CLKSEL_CLK_1P28M_EN_POS                      6
#define PCRM_CLKSEL_HCLK_MUX_DONE_OVERRIDE_BIT            ((uint32_t)0x00000010)
#define PCRM_CLKSEL_HCLK_MUX_DONE_OVERRIDE_POS            4
#define PCRM_CLKSEL_HCLK_SEL_EN_OVERRIDE_BIT              ((uint32_t)0x00000008)
#define PCRM_CLKSEL_HCLK_SEL_EN_OVERRIDE_POS              3
#define PCRM_CLKSEL_HCLK_SEL_MASK                         ((uint32_t)0x00000007)
#define PCRM_CLKSEL_HCLK_SEL_LSB                          0
#define PCRM_CLKSEL_HCLK_SEL_WIDTH                        ((uint32_t)0x00000003)

#define PCRM_CLKSEL_SPIF_REF_CLK_SEL_RST                  0x0
#define PCRM_CLKSEL_AON_PCLK_INV_RST                      0x0
#define PCRM_CLKSEL_LOWCLK_SEL_RST                        0x0
#define PCRM_CLKSEL_SEL_16M_RST                           0x0
#define PCRM_CLKSEL_CLK_1P28M_EN_RST                      0x1
#define PCRM_CLKSEL_HCLK_MUX_DONE_OVERRIDE_RST            0x1
#define PCRM_CLKSEL_HCLK_SEL_EN_OVERRIDE_RST              0x1
#define PCRM_CLKSEL_HCLK_SEL_RST                          0x0

__INLINE void pcrm_clksel_pack(uint8_t spifrefclksel, uint8_t aonpclkinv, uint8_t lowclksel, uint8_t sel16m, uint8_t clk1p28men, uint8_t hclkmuxdoneoverride, uint8_t hclkselenoverride, uint8_t hclksel)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)spifrefclksel << 24) | ((uint32_t)aonpclkinv << 17) | ((uint32_t)lowclksel << 16) | ((uint32_t)sel16m << 8) | ((uint32_t)clk1p28men << 6) | ((uint32_t)hclkmuxdoneoverride << 4) | ((uint32_t)hclkselenoverride << 3) | ((uint32_t)hclksel << 0));
}

__INLINE void pcrm_clksel_unpack(uint8_t* spifrefclksel, uint8_t* aonpclkinv, uint8_t* lowclksel, uint8_t* sel16m, uint8_t* clk1p28men, uint8_t* hclkmuxdoneoverride, uint8_t* hclkselenoverride, uint8_t* hclksel)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);

    *spifrefclksel = (localVal & ((uint32_t)0x07000000)) >> 24;
    *aonpclkinv = (localVal & ((uint32_t)0x00020000)) >> 17;
    *lowclksel = (localVal & ((uint32_t)0x00010000)) >> 16;
    *sel16m = (localVal & ((uint32_t)0x00000100)) >> 8;
    *clk1p28men = (localVal & ((uint32_t)0x00000040)) >> 6;
    *hclkmuxdoneoverride = (localVal & ((uint32_t)0x00000010)) >> 4;
    *hclkselenoverride = (localVal & ((uint32_t)0x00000008)) >> 3;
    *hclksel = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t pcrm_clksel_spif_ref_clk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void pcrm_clksel_spif_ref_clk_sel_setf(uint8_t spifrefclksel)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)spifrefclksel << 24));
}

__INLINE uint8_t pcrm_clksel_aon_pclk_inv_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void pcrm_clksel_aon_pclk_inv_setf(uint8_t aonpclkinv)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)aonpclkinv << 17));
}

__INLINE uint8_t pcrm_clksel_lowclk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pcrm_clksel_lowclk_sel_setf(uint8_t lowclksel)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)lowclksel << 16));
}

__INLINE uint8_t pcrm_clksel_sel_16m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcrm_clksel_sel_16m_setf(uint8_t sel16m)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)sel16m << 8));
}

__INLINE uint8_t pcrm_clksel_clk_1p28m_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void pcrm_clksel_clk_1p28m_en_setf(uint8_t clk1p28men)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)clk1p28men << 6));
}

__INLINE uint8_t pcrm_clksel_hclk_mux_done_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcrm_clksel_hclk_mux_done_override_setf(uint8_t hclkmuxdoneoverride)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)hclkmuxdoneoverride << 4));
}

__INLINE uint8_t pcrm_clksel_hclk_sel_en_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcrm_clksel_hclk_sel_en_override_setf(uint8_t hclkselenoverride)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)hclkselenoverride << 3));
}

__INLINE uint8_t pcrm_clksel_hclk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void pcrm_clksel_hclk_sel_setf(uint8_t hclksel)
{
    _PICO_REG_WR(PCRM_CLKSEL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKSEL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)hclksel << 0));
}

 /**
 * @brief CLKHF_CTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     18      xtal_clk_dig_en   0
 *  05:00             reserved   0b0
 * </pre>
 */
#define PCRM_CLKHF_CTL0_OFFSET 0x00000040


__INLINE uint32_t pcrm_clkhf_ctl0_get(void)
{
    return _PICO_REG_RD(PCRM_CLKHF_CTL0_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_clkhf_ctl0_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL0_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_CLKHF_CTL0_XTAL_CLK_DIG_EN_BIT                   ((uint32_t)0x00040000)
#define PCRM_CLKHF_CTL0_XTAL_CLK_DIG_EN_POS                   18
#define PCRM_CLKHF_CTL0_RESERVED_MASK                         ((uint32_t)0x0000003F)
#define PCRM_CLKHF_CTL0_RESERVED_LSB                          0
#define PCRM_CLKHF_CTL0_RESERVED_WIDTH                        ((uint32_t)0x00000006)

#define PCRM_CLKHF_CTL0_XTAL_CLK_DIG_EN_RST                   0x0
#define PCRM_CLKHF_CTL0_RESERVED_RST                          0x0

__INLINE void pcrm_clkhf_ctl0_pack(uint8_t xtalclkdigen)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL0_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)xtalclkdigen << 18));
}

__INLINE void pcrm_clkhf_ctl0_unpack(uint8_t* xtalclkdigen, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL0_OFFSET + PCRM_BASE_ADDR);

    *xtalclkdigen = (localVal & ((uint32_t)0x00040000)) >> 18;
    *reserved = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t pcrm_clkhf_ctl0_xtal_clk_dig_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL0_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void pcrm_clkhf_ctl0_xtal_clk_dig_en_setf(uint8_t xtalclkdigen)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL0_OFFSET+ PCRM_BASE_ADDR, (uint32_t)xtalclkdigen << 18);
}

 /**
 * @brief CLKHF_CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:24        rxadc_clk_sel   0b10
 *  23:22           rf_clk_sel   0b10
 *  21:20      dig_clk_32M_sel   0b10
 *     19     en_rxadc_clk_32M   0
 *     18            en_rf_clk   0
 *     16       en_dig_clk_96M   0
 *     15       en_dig_clk_64M   0
 *     14       en_dig_clk_48M   0
 *     13       en_dig_clk_32M   0
 *  10:09         dbl_cap_tune   0b10
 *     08               dbl_en   0
 *     07               dll_en   0
 *  06:05        dll_ldo_vctrl   0b1
 *     04           dll_ldo_pu   0
 * </pre>
 */
#define PCRM_CLKHF_CTL1_OFFSET 0x00000044


__INLINE uint32_t pcrm_clkhf_ctl1_get(void)
{
    return _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_clkhf_ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_CLKHF_CTL1_RXADC_CLK_SEL_MASK                    ((uint32_t)0x03000000)
#define PCRM_CLKHF_CTL1_RXADC_CLK_SEL_LSB                     24
#define PCRM_CLKHF_CTL1_RXADC_CLK_SEL_WIDTH                   ((uint32_t)0x00000002)
#define PCRM_CLKHF_CTL1_RF_CLK_SEL_MASK                       ((uint32_t)0x00C00000)
#define PCRM_CLKHF_CTL1_RF_CLK_SEL_LSB                        22
#define PCRM_CLKHF_CTL1_RF_CLK_SEL_WIDTH                      ((uint32_t)0x00000002)
#define PCRM_CLKHF_CTL1_DIG_CLK_32M_SEL_MASK                  ((uint32_t)0x00300000)
#define PCRM_CLKHF_CTL1_DIG_CLK_32M_SEL_LSB                   20
#define PCRM_CLKHF_CTL1_DIG_CLK_32M_SEL_WIDTH                 ((uint32_t)0x00000002)
#define PCRM_CLKHF_CTL1_EN_RXADC_CLK_32M_BIT                  ((uint32_t)0x00080000)
#define PCRM_CLKHF_CTL1_EN_RXADC_CLK_32M_POS                  19
#define PCRM_CLKHF_CTL1_EN_RF_CLK_BIT                         ((uint32_t)0x00040000)
#define PCRM_CLKHF_CTL1_EN_RF_CLK_POS                         18
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_96M_BIT                    ((uint32_t)0x00010000)
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_96M_POS                    16
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_64M_BIT                    ((uint32_t)0x00008000)
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_64M_POS                    15
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_48M_BIT                    ((uint32_t)0x00004000)
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_48M_POS                    14
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_32M_BIT                    ((uint32_t)0x00002000)
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_32M_POS                    13
#define PCRM_CLKHF_CTL1_DBL_CAP_TUNE_MASK                     ((uint32_t)0x00000600)
#define PCRM_CLKHF_CTL1_DBL_CAP_TUNE_LSB                      9
#define PCRM_CLKHF_CTL1_DBL_CAP_TUNE_WIDTH                    ((uint32_t)0x00000002)
#define PCRM_CLKHF_CTL1_DBL_EN_BIT                            ((uint32_t)0x00000100)
#define PCRM_CLKHF_CTL1_DBL_EN_POS                            8
#define PCRM_CLKHF_CTL1_DLL_EN_BIT                            ((uint32_t)0x00000080)
#define PCRM_CLKHF_CTL1_DLL_EN_POS                            7
#define PCRM_CLKHF_CTL1_DLL_LDO_VCTRL_MASK                    ((uint32_t)0x00000060)
#define PCRM_CLKHF_CTL1_DLL_LDO_VCTRL_LSB                     5
#define PCRM_CLKHF_CTL1_DLL_LDO_VCTRL_WIDTH                   ((uint32_t)0x00000002)
#define PCRM_CLKHF_CTL1_DLL_LDO_PU_BIT                        ((uint32_t)0x00000010)
#define PCRM_CLKHF_CTL1_DLL_LDO_PU_POS                        4

#define PCRM_CLKHF_CTL1_RXADC_CLK_SEL_RST                     0x10
#define PCRM_CLKHF_CTL1_RF_CLK_SEL_RST                        0x10
#define PCRM_CLKHF_CTL1_DIG_CLK_32M_SEL_RST                   0x10
#define PCRM_CLKHF_CTL1_EN_RXADC_CLK_32M_RST                  0x0
#define PCRM_CLKHF_CTL1_EN_RF_CLK_RST                         0x0
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_96M_RST                    0x0
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_64M_RST                    0x0
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_48M_RST                    0x0
#define PCRM_CLKHF_CTL1_EN_DIG_CLK_32M_RST                    0x0
#define PCRM_CLKHF_CTL1_DBL_CAP_TUNE_RST                      0x10
#define PCRM_CLKHF_CTL1_DBL_EN_RST                            0x0
#define PCRM_CLKHF_CTL1_DLL_EN_RST                            0x0
#define PCRM_CLKHF_CTL1_DLL_LDO_VCTRL_RST                     0x1
#define PCRM_CLKHF_CTL1_DLL_LDO_PU_RST                        0x0

__INLINE void pcrm_clkhf_ctl1_pack(uint8_t rxadcclksel, uint8_t rfclksel, uint8_t digclk32msel, uint8_t enrxadcclk32m, uint8_t enrfclk, uint8_t endigclk96m, uint8_t endigclk64m, uint8_t endigclk48m, uint8_t endigclk32m, uint8_t dblcaptune, uint8_t dblen, uint8_t dllen, uint8_t dllldovctrl, uint8_t dllldopu)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)rxadcclksel << 24) | ((uint32_t)rfclksel << 22) | ((uint32_t)digclk32msel << 20) | ((uint32_t)enrxadcclk32m << 19) | ((uint32_t)enrfclk << 18) | ((uint32_t)endigclk96m << 16) | ((uint32_t)endigclk64m << 15) | ((uint32_t)endigclk48m << 14) | ((uint32_t)endigclk32m << 13) | ((uint32_t)dblcaptune << 9) | ((uint32_t)dblen << 8) | ((uint32_t)dllen << 7) | ((uint32_t)dllldovctrl << 5) | ((uint32_t)dllldopu << 4));
}

__INLINE void pcrm_clkhf_ctl1_unpack(uint8_t* rxadcclksel, uint8_t* rfclksel, uint8_t* digclk32msel, uint8_t* enrxadcclk32m, uint8_t* enrfclk, uint8_t* endigclk96m, uint8_t* endigclk64m, uint8_t* endigclk48m, uint8_t* endigclk32m, uint8_t* dblcaptune, uint8_t* dblen, uint8_t* dllen, uint8_t* dllldovctrl, uint8_t* dllldopu)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);

    *rxadcclksel = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rfclksel = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *digclk32msel = (localVal & ((uint32_t)0x00300000)) >> 20;
    *enrxadcclk32m = (localVal & ((uint32_t)0x00080000)) >> 19;
    *enrfclk = (localVal & ((uint32_t)0x00040000)) >> 18;
    *endigclk96m = (localVal & ((uint32_t)0x00010000)) >> 16;
    *endigclk64m = (localVal & ((uint32_t)0x00008000)) >> 15;
    *endigclk48m = (localVal & ((uint32_t)0x00004000)) >> 14;
    *endigclk32m = (localVal & ((uint32_t)0x00002000)) >> 13;
    *dblcaptune = (localVal & ((uint32_t)0x00000600)) >> 9;
    *dblen = (localVal & ((uint32_t)0x00000100)) >> 8;
    *dllen = (localVal & ((uint32_t)0x00000080)) >> 7;
    *dllldovctrl = (localVal & ((uint32_t)0x00000060)) >> 5;
    *dllldopu = (localVal & ((uint32_t)0x00000010)) >> 4;
}

__INLINE uint8_t pcrm_clkhf_ctl1_rxadc_clk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void pcrm_clkhf_ctl1_rxadc_clk_sel_setf(uint8_t rxadcclksel)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rxadcclksel << 24));
}

__INLINE uint8_t pcrm_clkhf_ctl1_rf_clk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE void pcrm_clkhf_ctl1_rf_clk_sel_setf(uint8_t rfclksel)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00C00000)) | ((uint32_t)rfclksel << 22));
}

__INLINE uint8_t pcrm_clkhf_ctl1_dig_clk_32m_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void pcrm_clkhf_ctl1_dig_clk_32m_sel_setf(uint8_t digclk32msel)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)digclk32msel << 20));
}

__INLINE uint8_t pcrm_clkhf_ctl1_en_rxadc_clk_32m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void pcrm_clkhf_ctl1_en_rxadc_clk_32m_setf(uint8_t enrxadcclk32m)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)enrxadcclk32m << 19));
}

__INLINE uint8_t pcrm_clkhf_ctl1_en_rf_clk_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void pcrm_clkhf_ctl1_en_rf_clk_setf(uint8_t enrfclk)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)enrfclk << 18));
}

__INLINE uint8_t pcrm_clkhf_ctl1_en_dig_clk_96m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pcrm_clkhf_ctl1_en_dig_clk_96m_setf(uint8_t endigclk96m)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)endigclk96m << 16));
}

__INLINE uint8_t pcrm_clkhf_ctl1_en_dig_clk_64m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void pcrm_clkhf_ctl1_en_dig_clk_64m_setf(uint8_t endigclk64m)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)endigclk64m << 15));
}

__INLINE uint8_t pcrm_clkhf_ctl1_en_dig_clk_48m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void pcrm_clkhf_ctl1_en_dig_clk_48m_setf(uint8_t endigclk48m)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)endigclk48m << 14));
}

__INLINE uint8_t pcrm_clkhf_ctl1_en_dig_clk_32m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void pcrm_clkhf_ctl1_en_dig_clk_32m_setf(uint8_t endigclk32m)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)endigclk32m << 13));
}

__INLINE uint8_t pcrm_clkhf_ctl1_dbl_cap_tune_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000600)) >> 9);
}

__INLINE void pcrm_clkhf_ctl1_dbl_cap_tune_setf(uint8_t dblcaptune)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000600)) | ((uint32_t)dblcaptune << 9));
}

__INLINE uint8_t pcrm_clkhf_ctl1_dbl_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcrm_clkhf_ctl1_dbl_en_setf(uint8_t dblen)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)dblen << 8));
}

__INLINE uint8_t pcrm_clkhf_ctl1_dll_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void pcrm_clkhf_ctl1_dll_en_setf(uint8_t dllen)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)dllen << 7));
}

__INLINE uint8_t pcrm_clkhf_ctl1_dll_ldo_vctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000060)) >> 5);
}

__INLINE void pcrm_clkhf_ctl1_dll_ldo_vctrl_setf(uint8_t dllldovctrl)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000060)) | ((uint32_t)dllldovctrl << 5));
}

__INLINE uint8_t pcrm_clkhf_ctl1_dll_ldo_pu_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcrm_clkhf_ctl1_dll_ldo_pu_setf(uint8_t dllldopu)
{
    _PICO_REG_WR(PCRM_CLKHF_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CLKHF_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)dllldopu << 4));
}

 /**
 * @brief ANA_CTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     26   micbias_vref_fil_en   0
 *  25:24     micbias_out_ctrl   0b1
 *     23           micbias_en   0
 *     22         pga_1st_gain   0
 *  21:19         pga_2nd_gain   0b0
 *  18:17      PGA_LDO_outctrl   0b1
 *     16               pga_en   0
 *     11    adc12b_semode_enm   0
 *  10:09          adc_dly_ctl   0b0
 *     08    adc12b_semode_enp   0
 *  07:05       adc12b_chn_sel   0b0
 *     03            adc12b_en   0
 *  02:01        ana_ldo_vctrl   0b1
 *     00           ana_ldo_en   0
 * </pre>
 */
#define PCRM_ANA_CTL_OFFSET 0x00000048


__INLINE uint32_t pcrm_ana_ctl_get(void)
{
    return _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_ana_ctl_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_ANA_CTL_MICBIAS_VREF_FIL_EN_BIT               ((uint32_t)0x04000000)
#define PCRM_ANA_CTL_MICBIAS_VREF_FIL_EN_POS               26
#define PCRM_ANA_CTL_MICBIAS_OUT_CTRL_MASK                 ((uint32_t)0x03000000)
#define PCRM_ANA_CTL_MICBIAS_OUT_CTRL_LSB                  24
#define PCRM_ANA_CTL_MICBIAS_OUT_CTRL_WIDTH                ((uint32_t)0x00000002)
#define PCRM_ANA_CTL_MICBIAS_EN_BIT                        ((uint32_t)0x00800000)
#define PCRM_ANA_CTL_MICBIAS_EN_POS                        23
#define PCRM_ANA_CTL_PGA_1ST_GAIN_BIT                      ((uint32_t)0x00400000)
#define PCRM_ANA_CTL_PGA_1ST_GAIN_POS                      22
#define PCRM_ANA_CTL_PGA_2ND_GAIN_MASK                     ((uint32_t)0x00380000)
#define PCRM_ANA_CTL_PGA_2ND_GAIN_LSB                      19
#define PCRM_ANA_CTL_PGA_2ND_GAIN_WIDTH                    ((uint32_t)0x00000003)
#define PCRM_ANA_CTL_PGA_LDO_OUTCTRL_MASK                  ((uint32_t)0x00060000)
#define PCRM_ANA_CTL_PGA_LDO_OUTCTRL_LSB                   17
#define PCRM_ANA_CTL_PGA_LDO_OUTCTRL_WIDTH                 ((uint32_t)0x00000002)
#define PCRM_ANA_CTL_PGA_EN_BIT                            ((uint32_t)0x00010000)
#define PCRM_ANA_CTL_PGA_EN_POS                            16
#define PCRM_ANA_CTL_ADC12B_SEMODE_ENM_BIT                 ((uint32_t)0x00000800)
#define PCRM_ANA_CTL_ADC12B_SEMODE_ENM_POS                 11
#define PCRM_ANA_CTL_ADC_DLY_CTL_MASK                      ((uint32_t)0x00000600)
#define PCRM_ANA_CTL_ADC_DLY_CTL_LSB                       9
#define PCRM_ANA_CTL_ADC_DLY_CTL_WIDTH                     ((uint32_t)0x00000002)
#define PCRM_ANA_CTL_ADC12B_SEMODE_ENP_BIT                 ((uint32_t)0x00000100)
#define PCRM_ANA_CTL_ADC12B_SEMODE_ENP_POS                 8
#define PCRM_ANA_CTL_ADC12B_CHN_SEL_MASK                   ((uint32_t)0x000000E0)
#define PCRM_ANA_CTL_ADC12B_CHN_SEL_LSB                    5
#define PCRM_ANA_CTL_ADC12B_CHN_SEL_WIDTH                  ((uint32_t)0x00000003)
#define PCRM_ANA_CTL_ADC12B_EN_BIT                         ((uint32_t)0x00000008)
#define PCRM_ANA_CTL_ADC12B_EN_POS                         3
#define PCRM_ANA_CTL_ANA_LDO_VCTRL_MASK                    ((uint32_t)0x00000006)
#define PCRM_ANA_CTL_ANA_LDO_VCTRL_LSB                     1
#define PCRM_ANA_CTL_ANA_LDO_VCTRL_WIDTH                   ((uint32_t)0x00000002)
#define PCRM_ANA_CTL_ANA_LDO_EN_BIT                        ((uint32_t)0x00000001)
#define PCRM_ANA_CTL_ANA_LDO_EN_POS                        0

#define PCRM_ANA_CTL_MICBIAS_VREF_FIL_EN_RST               0x0
#define PCRM_ANA_CTL_MICBIAS_OUT_CTRL_RST                  0x1
#define PCRM_ANA_CTL_MICBIAS_EN_RST                        0x0
#define PCRM_ANA_CTL_PGA_1ST_GAIN_RST                      0x0
#define PCRM_ANA_CTL_PGA_2ND_GAIN_RST                      0x0
#define PCRM_ANA_CTL_PGA_LDO_OUTCTRL_RST                   0x1
#define PCRM_ANA_CTL_PGA_EN_RST                            0x0
#define PCRM_ANA_CTL_ADC12B_SEMODE_ENM_RST                 0x0
#define PCRM_ANA_CTL_ADC_DLY_CTL_RST                       0x0
#define PCRM_ANA_CTL_ADC12B_SEMODE_ENP_RST                 0x0
#define PCRM_ANA_CTL_ADC12B_CHN_SEL_RST                    0x0
#define PCRM_ANA_CTL_ADC12B_EN_RST                         0x0
#define PCRM_ANA_CTL_ANA_LDO_VCTRL_RST                     0x1
#define PCRM_ANA_CTL_ANA_LDO_EN_RST                        0x0

__INLINE void pcrm_ana_ctl_pack(uint8_t micbiasvreffilen, uint8_t micbiasoutctrl, uint8_t micbiasen, uint8_t pga1stgain, uint8_t pga2ndgain, uint8_t pgaldooutctrl, uint8_t pgaen, uint8_t adc12bsemodeenm, uint8_t adcdlyctl, uint8_t adc12bsemodeenp, uint8_t adc12bchnsel, uint8_t adc12ben, uint8_t analdovctrl, uint8_t analdoen)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)micbiasvreffilen << 26) | ((uint32_t)micbiasoutctrl << 24) | ((uint32_t)micbiasen << 23) | ((uint32_t)pga1stgain << 22) | ((uint32_t)pga2ndgain << 19) | ((uint32_t)pgaldooutctrl << 17) | ((uint32_t)pgaen << 16) | ((uint32_t)adc12bsemodeenm << 11) | ((uint32_t)adcdlyctl << 9) | ((uint32_t)adc12bsemodeenp << 8) | ((uint32_t)adc12bchnsel << 5) | ((uint32_t)adc12ben << 3) | ((uint32_t)analdovctrl << 1) | ((uint32_t)analdoen << 0));
}

__INLINE void pcrm_ana_ctl_unpack(uint8_t* micbiasvreffilen, uint8_t* micbiasoutctrl, uint8_t* micbiasen, uint8_t* pga1stgain, uint8_t* pga2ndgain, uint8_t* pgaldooutctrl, uint8_t* pgaen, uint8_t* adc12bsemodeenm, uint8_t* adcdlyctl, uint8_t* adc12bsemodeenp, uint8_t* adc12bchnsel, uint8_t* adc12ben, uint8_t* analdovctrl, uint8_t* analdoen)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);

    *micbiasvreffilen = (localVal & ((uint32_t)0x04000000)) >> 26;
    *micbiasoutctrl = (localVal & ((uint32_t)0x03000000)) >> 24;
    *micbiasen = (localVal & ((uint32_t)0x00800000)) >> 23;
    *pga1stgain = (localVal & ((uint32_t)0x00400000)) >> 22;
    *pga2ndgain = (localVal & ((uint32_t)0x00380000)) >> 19;
    *pgaldooutctrl = (localVal & ((uint32_t)0x00060000)) >> 17;
    *pgaen = (localVal & ((uint32_t)0x00010000)) >> 16;
    *adc12bsemodeenm = (localVal & ((uint32_t)0x00000800)) >> 11;
    *adcdlyctl = (localVal & ((uint32_t)0x00000600)) >> 9;
    *adc12bsemodeenp = (localVal & ((uint32_t)0x00000100)) >> 8;
    *adc12bchnsel = (localVal & ((uint32_t)0x000000E0)) >> 5;
    *adc12ben = (localVal & ((uint32_t)0x00000008)) >> 3;
    *analdovctrl = (localVal & ((uint32_t)0x00000006)) >> 1;
    *analdoen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcrm_ana_ctl_micbias_vref_fil_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x04000000)) >> 26);
}

__INLINE void pcrm_ana_ctl_micbias_vref_fil_en_setf(uint8_t micbiasvreffilen)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x04000000)) | ((uint32_t)micbiasvreffilen << 26));
}

__INLINE uint8_t pcrm_ana_ctl_micbias_out_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void pcrm_ana_ctl_micbias_out_ctrl_setf(uint8_t micbiasoutctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)micbiasoutctrl << 24));
}

__INLINE uint8_t pcrm_ana_ctl_micbias_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void pcrm_ana_ctl_micbias_en_setf(uint8_t micbiasen)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)micbiasen << 23));
}

__INLINE uint8_t pcrm_ana_ctl_pga_1st_gain_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void pcrm_ana_ctl_pga_1st_gain_setf(uint8_t pga1stgain)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)pga1stgain << 22));
}

__INLINE uint8_t pcrm_ana_ctl_pga_2nd_gain_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00380000)) >> 19);
}

__INLINE void pcrm_ana_ctl_pga_2nd_gain_setf(uint8_t pga2ndgain)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00380000)) | ((uint32_t)pga2ndgain << 19));
}

__INLINE uint8_t pcrm_ana_ctl_pga_ldo_outctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00060000)) >> 17);
}

__INLINE void pcrm_ana_ctl_pga_ldo_outctrl_setf(uint8_t pgaldooutctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00060000)) | ((uint32_t)pgaldooutctrl << 17));
}

__INLINE uint8_t pcrm_ana_ctl_pga_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pcrm_ana_ctl_pga_en_setf(uint8_t pgaen)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)pgaen << 16));
}

__INLINE uint8_t pcrm_ana_ctl_adc12b_semode_enm_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void pcrm_ana_ctl_adc12b_semode_enm_setf(uint8_t adc12bsemodeenm)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)adc12bsemodeenm << 11));
}

__INLINE uint8_t pcrm_ana_ctl_adc_dly_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000600)) >> 9);
}

__INLINE void pcrm_ana_ctl_adc_dly_ctl_setf(uint8_t adcdlyctl)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000600)) | ((uint32_t)adcdlyctl << 9));
}

__INLINE uint8_t pcrm_ana_ctl_adc12b_semode_enp_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void pcrm_ana_ctl_adc12b_semode_enp_setf(uint8_t adc12bsemodeenp)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)adc12bsemodeenp << 8));
}

__INLINE uint8_t pcrm_ana_ctl_adc12b_chn_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000E0)) >> 5);
}

__INLINE void pcrm_ana_ctl_adc12b_chn_sel_setf(uint8_t adc12bchnsel)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x000000E0)) | ((uint32_t)adc12bchnsel << 5));
}

__INLINE uint8_t pcrm_ana_ctl_adc12b_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcrm_ana_ctl_adc12b_en_setf(uint8_t adc12ben)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)adc12ben << 3));
}

__INLINE uint8_t pcrm_ana_ctl_ana_ldo_vctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000006)) >> 1);
}

__INLINE void pcrm_ana_ctl_ana_ldo_vctrl_setf(uint8_t analdovctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000006)) | ((uint32_t)analdovctrl << 1));
}

__INLINE uint8_t pcrm_ana_ctl_ana_ldo_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcrm_ana_ctl_ana_ldo_en_setf(uint8_t analdoen)
{
    _PICO_REG_WR(PCRM_ANA_CTL_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)analdoen << 0));
}

 /**
 * @brief MEM_0_1_DVS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31            ram1_dvse   0
 *  30:28             ram1_dvs   0b0
 *     27            ram0_dvse   0
 *  26:24             ram0_dvs   0b0
 *     04            rom0_dvse   0
 *  03:00             rom0_dvs   0b0
 * </pre>
 */
#define PCRM_MEM_0_1_DVS_OFFSET 0x0000004C


__INLINE uint32_t pcrm_mem_0_1_dvs_get(void)
{
    return _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_mem_0_1_dvs_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_MEM_0_1_DVS_RAM1_DVSE_BIT                         ((uint32_t)0x80000000)
#define PCRM_MEM_0_1_DVS_RAM1_DVSE_POS                         31
#define PCRM_MEM_0_1_DVS_RAM1_DVS_MASK                         ((uint32_t)0x70000000)
#define PCRM_MEM_0_1_DVS_RAM1_DVS_LSB                          28
#define PCRM_MEM_0_1_DVS_RAM1_DVS_WIDTH                        ((uint32_t)0x00000003)
#define PCRM_MEM_0_1_DVS_RAM0_DVSE_BIT                         ((uint32_t)0x08000000)
#define PCRM_MEM_0_1_DVS_RAM0_DVSE_POS                         27
#define PCRM_MEM_0_1_DVS_RAM0_DVS_MASK                         ((uint32_t)0x07000000)
#define PCRM_MEM_0_1_DVS_RAM0_DVS_LSB                          24
#define PCRM_MEM_0_1_DVS_RAM0_DVS_WIDTH                        ((uint32_t)0x00000003)
#define PCRM_MEM_0_1_DVS_ROM0_DVSE_BIT                         ((uint32_t)0x00000010)
#define PCRM_MEM_0_1_DVS_ROM0_DVSE_POS                         4
#define PCRM_MEM_0_1_DVS_ROM0_DVS_MASK                         ((uint32_t)0x0000000F)
#define PCRM_MEM_0_1_DVS_ROM0_DVS_LSB                          0
#define PCRM_MEM_0_1_DVS_ROM0_DVS_WIDTH                        ((uint32_t)0x00000004)

#define PCRM_MEM_0_1_DVS_RAM1_DVSE_RST                         0x0
#define PCRM_MEM_0_1_DVS_RAM1_DVS_RST                          0x0
#define PCRM_MEM_0_1_DVS_RAM0_DVSE_RST                         0x0
#define PCRM_MEM_0_1_DVS_RAM0_DVS_RST                          0x0
#define PCRM_MEM_0_1_DVS_ROM0_DVSE_RST                         0x0
#define PCRM_MEM_0_1_DVS_ROM0_DVS_RST                          0x0

__INLINE void pcrm_mem_0_1_dvs_pack(uint8_t ram1dvse, uint8_t ram1dvs, uint8_t ram0dvse, uint8_t ram0dvs, uint8_t rom0dvse, uint8_t rom0dvs)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)ram1dvse << 31) | ((uint32_t)ram1dvs << 28) | ((uint32_t)ram0dvse << 27) | ((uint32_t)ram0dvs << 24) | ((uint32_t)rom0dvse << 4) | ((uint32_t)rom0dvs << 0));
}

__INLINE void pcrm_mem_0_1_dvs_unpack(uint8_t* ram1dvse, uint8_t* ram1dvs, uint8_t* ram0dvse, uint8_t* ram0dvs, uint8_t* rom0dvse, uint8_t* rom0dvs)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);

    *ram1dvse = (localVal & ((uint32_t)0x80000000)) >> 31;
    *ram1dvs = (localVal & ((uint32_t)0x70000000)) >> 28;
    *ram0dvse = (localVal & ((uint32_t)0x08000000)) >> 27;
    *ram0dvs = (localVal & ((uint32_t)0x07000000)) >> 24;
    *rom0dvse = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rom0dvs = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t pcrm_mem_0_1_dvs_ram1_dvse_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pcrm_mem_0_1_dvs_ram1_dvse_setf(uint8_t ram1dvse)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)ram1dvse << 31));
}

__INLINE uint8_t pcrm_mem_0_1_dvs_ram1_dvs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x70000000)) >> 28);
}

__INLINE void pcrm_mem_0_1_dvs_ram1_dvs_setf(uint8_t ram1dvs)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x70000000)) | ((uint32_t)ram1dvs << 28));
}

__INLINE uint8_t pcrm_mem_0_1_dvs_ram0_dvse_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x08000000)) >> 27);
}

__INLINE void pcrm_mem_0_1_dvs_ram0_dvse_setf(uint8_t ram0dvse)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x08000000)) | ((uint32_t)ram0dvse << 27));
}

__INLINE uint8_t pcrm_mem_0_1_dvs_ram0_dvs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void pcrm_mem_0_1_dvs_ram0_dvs_setf(uint8_t ram0dvs)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)ram0dvs << 24));
}

__INLINE uint8_t pcrm_mem_0_1_dvs_rom0_dvse_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcrm_mem_0_1_dvs_rom0_dvse_setf(uint8_t rom0dvse)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rom0dvse << 4));
}

__INLINE uint8_t pcrm_mem_0_1_dvs_rom0_dvs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void pcrm_mem_0_1_dvs_rom0_dvs_setf(uint8_t rom0dvs)
{
    _PICO_REG_WR(PCRM_MEM_0_1_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_0_1_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)rom0dvs << 0));
}

 /**
 * @brief MEM_2_3_4_DVS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     28          bb_ram_dvse   0
 *  27:24           bb_ram_dvs   0b0
 *  23:00             reserved   0b0
 * </pre>
 */
#define PCRM_MEM_2_3_4_DVS_OFFSET 0x00000050


__INLINE uint32_t pcrm_mem_2_3_4_dvs_get(void)
{
    return _PICO_REG_RD(PCRM_MEM_2_3_4_DVS_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_mem_2_3_4_dvs_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_MEM_2_3_4_DVS_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_MEM_2_3_4_DVS_BB_RAM_DVSE_BIT                       ((uint32_t)0x10000000)
#define PCRM_MEM_2_3_4_DVS_BB_RAM_DVSE_POS                       28
#define PCRM_MEM_2_3_4_DVS_BB_RAM_DVS_MASK                       ((uint32_t)0x0F000000)
#define PCRM_MEM_2_3_4_DVS_BB_RAM_DVS_LSB                        24
#define PCRM_MEM_2_3_4_DVS_BB_RAM_DVS_WIDTH                      ((uint32_t)0x00000004)
#define PCRM_MEM_2_3_4_DVS_RESERVED_MASK                         ((uint32_t)0x00FFFFFF)
#define PCRM_MEM_2_3_4_DVS_RESERVED_LSB                          0
#define PCRM_MEM_2_3_4_DVS_RESERVED_WIDTH                        ((uint32_t)0x00000018)

#define PCRM_MEM_2_3_4_DVS_BB_RAM_DVSE_RST                       0x0
#define PCRM_MEM_2_3_4_DVS_BB_RAM_DVS_RST                        0x0
#define PCRM_MEM_2_3_4_DVS_RESERVED_RST                          0x0

__INLINE void pcrm_mem_2_3_4_dvs_pack(uint8_t bbramdvse, uint8_t bbramdvs)
{
    _PICO_REG_WR(PCRM_MEM_2_3_4_DVS_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)bbramdvse << 28) | ((uint32_t)bbramdvs << 24));
}

__INLINE void pcrm_mem_2_3_4_dvs_unpack(uint8_t* bbramdvse, uint8_t* bbramdvs, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_2_3_4_DVS_OFFSET + PCRM_BASE_ADDR);

    *bbramdvse = (localVal & ((uint32_t)0x10000000)) >> 28;
    *bbramdvs = (localVal & ((uint32_t)0x0F000000)) >> 24;
    *reserved = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint8_t pcrm_mem_2_3_4_dvs_bb_ram_dvse_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_2_3_4_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void pcrm_mem_2_3_4_dvs_bb_ram_dvse_setf(uint8_t bbramdvse)
{
    _PICO_REG_WR(PCRM_MEM_2_3_4_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_2_3_4_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)bbramdvse << 28));
}

__INLINE uint8_t pcrm_mem_2_3_4_dvs_bb_ram_dvs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_MEM_2_3_4_DVS_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0F000000)) >> 24);
}

__INLINE void pcrm_mem_2_3_4_dvs_bb_ram_dvs_setf(uint8_t bbramdvs)
{
    _PICO_REG_WR(PCRM_MEM_2_3_4_DVS_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_MEM_2_3_4_DVS_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x0F000000)) | ((uint32_t)bbramdvs << 24));
}

 /**
 * @brief CAL_RW register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16        rc32k_clk_sel   0
 *  08:04      num_track_cycle   0b11111
 *     03       track_en_rc32k   0
 *     02             rccal_en   0
 *     01           cal_en_32M   1
 *     00           cal_en_32k   0
 * </pre>
 */
#define PCRM_CAL_RW_OFFSET 0x0000005C


__INLINE uint32_t pcrm_cal_rw_get(void)
{
    return _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_cal_rw_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_CAL_RW_RC32K_CLK_SEL_BIT                     ((uint32_t)0x00010000)
#define PCRM_CAL_RW_RC32K_CLK_SEL_POS                     16
#define PCRM_CAL_RW_NUM_TRACK_CYCLE_MASK                  ((uint32_t)0x000001F0)
#define PCRM_CAL_RW_NUM_TRACK_CYCLE_LSB                   4
#define PCRM_CAL_RW_NUM_TRACK_CYCLE_WIDTH                 ((uint32_t)0x00000005)
#define PCRM_CAL_RW_TRACK_EN_RC32K_BIT                    ((uint32_t)0x00000008)
#define PCRM_CAL_RW_TRACK_EN_RC32K_POS                    3
#define PCRM_CAL_RW_RCCAL_EN_BIT                          ((uint32_t)0x00000004)
#define PCRM_CAL_RW_RCCAL_EN_POS                          2
#define PCRM_CAL_RW_CAL_EN_32M_BIT                        ((uint32_t)0x00000002)
#define PCRM_CAL_RW_CAL_EN_32M_POS                        1
#define PCRM_CAL_RW_CAL_EN_32K_BIT                        ((uint32_t)0x00000001)
#define PCRM_CAL_RW_CAL_EN_32K_POS                        0

#define PCRM_CAL_RW_RC32K_CLK_SEL_RST                     0x0
#define PCRM_CAL_RW_NUM_TRACK_CYCLE_RST                   0x11111
#define PCRM_CAL_RW_TRACK_EN_RC32K_RST                    0x0
#define PCRM_CAL_RW_RCCAL_EN_RST                          0x0
#define PCRM_CAL_RW_CAL_EN_32M_RST                        0x1
#define PCRM_CAL_RW_CAL_EN_32K_RST                        0x0

__INLINE void pcrm_cal_rw_pack(uint8_t rc32kclksel, uint8_t numtrackcycle, uint8_t trackenrc32k, uint8_t rccalen, uint8_t calen32m, uint8_t calen32k)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)rc32kclksel << 16) | ((uint32_t)numtrackcycle << 4) | ((uint32_t)trackenrc32k << 3) | ((uint32_t)rccalen << 2) | ((uint32_t)calen32m << 1) | ((uint32_t)calen32k << 0));
}

__INLINE void pcrm_cal_rw_unpack(uint8_t* rc32kclksel, uint8_t* numtrackcycle, uint8_t* trackenrc32k, uint8_t* rccalen, uint8_t* calen32m, uint8_t* calen32k)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);

    *rc32kclksel = (localVal & ((uint32_t)0x00010000)) >> 16;
    *numtrackcycle = (localVal & ((uint32_t)0x000001F0)) >> 4;
    *trackenrc32k = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rccalen = (localVal & ((uint32_t)0x00000004)) >> 2;
    *calen32m = (localVal & ((uint32_t)0x00000002)) >> 1;
    *calen32k = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcrm_cal_rw_rc32k_clk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void pcrm_cal_rw_rc32k_clk_sel_setf(uint8_t rc32kclksel)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)rc32kclksel << 16));
}

__INLINE uint8_t pcrm_cal_rw_num_track_cycle_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000001F0)) >> 4);
}

__INLINE void pcrm_cal_rw_num_track_cycle_setf(uint8_t numtrackcycle)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x000001F0)) | ((uint32_t)numtrackcycle << 4));
}

__INLINE uint8_t pcrm_cal_rw_track_en_rc32k_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcrm_cal_rw_track_en_rc32k_setf(uint8_t trackenrc32k)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)trackenrc32k << 3));
}

__INLINE uint8_t pcrm_cal_rw_rccal_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void pcrm_cal_rw_rccal_en_setf(uint8_t rccalen)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rccalen << 2));
}

__INLINE uint8_t pcrm_cal_rw_cal_en_32m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void pcrm_cal_rw_cal_en_32m_setf(uint8_t calen32m)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)calen32m << 1));
}

__INLINE uint8_t pcrm_cal_rw_cal_en_32k_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcrm_cal_rw_cal_en_32k_setf(uint8_t calen32k)
{
    _PICO_REG_WR(PCRM_CAL_RW_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_CAL_RW_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)calen32k << 0));
}

 /**
 * @brief CAL_RO0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22    actu_cnt_fina_32k   0b0
 *  21:16    cal_word_fina_32k   0b0
 *  15:06    actu_cnt_fina_32M   0b0
 *  05:00    cal_word_fina_32M   0b0
 * </pre>
 */
#define PCRM_CAL_RO0_OFFSET 0x00000060


__INLINE uint32_t pcrm_cal_ro0_get(void)
{
    return _PICO_REG_RD(PCRM_CAL_RO0_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_CAL_RO0_ACTU_CNT_FINA_32K_MASK                ((uint32_t)0xFFC00000)
#define PCRM_CAL_RO0_ACTU_CNT_FINA_32K_LSB                 22
#define PCRM_CAL_RO0_ACTU_CNT_FINA_32K_WIDTH               ((uint32_t)0x0000000A)
#define PCRM_CAL_RO0_CAL_WORD_FINA_32K_MASK                ((uint32_t)0x003F0000)
#define PCRM_CAL_RO0_CAL_WORD_FINA_32K_LSB                 16
#define PCRM_CAL_RO0_CAL_WORD_FINA_32K_WIDTH               ((uint32_t)0x00000006)
#define PCRM_CAL_RO0_ACTU_CNT_FINA_32M_MASK                ((uint32_t)0x0000FFC0)
#define PCRM_CAL_RO0_ACTU_CNT_FINA_32M_LSB                 6
#define PCRM_CAL_RO0_ACTU_CNT_FINA_32M_WIDTH               ((uint32_t)0x0000000A)
#define PCRM_CAL_RO0_CAL_WORD_FINA_32M_MASK                ((uint32_t)0x0000003F)
#define PCRM_CAL_RO0_CAL_WORD_FINA_32M_LSB                 0
#define PCRM_CAL_RO0_CAL_WORD_FINA_32M_WIDTH               ((uint32_t)0x00000006)

#define PCRM_CAL_RO0_ACTU_CNT_FINA_32K_RST                 0x0
#define PCRM_CAL_RO0_CAL_WORD_FINA_32K_RST                 0x0
#define PCRM_CAL_RO0_ACTU_CNT_FINA_32M_RST                 0x0
#define PCRM_CAL_RO0_CAL_WORD_FINA_32M_RST                 0x0

__INLINE void pcrm_cal_ro0_unpack(uint8_t* actucntfina32k, uint8_t* calwordfina32k, uint8_t* actucntfina32m, uint8_t* calwordfina32m)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO0_OFFSET + PCRM_BASE_ADDR);

    *actucntfina32k = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *calwordfina32k = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *actucntfina32m = (localVal & ((uint32_t)0x0000FFC0)) >> 6;
    *calwordfina32m = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint16_t pcrm_cal_ro0_actu_cnt_fina_32k_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO0_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE uint8_t pcrm_cal_ro0_cal_word_fina_32k_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO0_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE uint16_t pcrm_cal_ro0_actu_cnt_fina_32m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO0_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFC0)) >> 6);
}

__INLINE uint8_t pcrm_cal_ro0_cal_word_fina_32m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO0_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

 /**
 * @brief CAL_RO1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:19    actu_cnt_fina_32k   0b0
 *  16:00   cnt_track_16M_fina   0b0
 * </pre>
 */
#define PCRM_CAL_RO1_OFFSET 0x00000064


__INLINE uint32_t pcrm_cal_ro1_get(void)
{
    return _PICO_REG_RD(PCRM_CAL_RO1_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_CAL_RO1_ACTU_CNT_FINA_32K_MASK                ((uint32_t)0xFFF80000)
#define PCRM_CAL_RO1_ACTU_CNT_FINA_32K_LSB                 19
#define PCRM_CAL_RO1_ACTU_CNT_FINA_32K_WIDTH               ((uint32_t)0x0000000D)
#define PCRM_CAL_RO1_CNT_TRACK_16M_FINA_MASK               ((uint32_t)0x0001FFFF)
#define PCRM_CAL_RO1_CNT_TRACK_16M_FINA_LSB                0
#define PCRM_CAL_RO1_CNT_TRACK_16M_FINA_WIDTH              ((uint32_t)0x00000011)

#define PCRM_CAL_RO1_ACTU_CNT_FINA_32K_RST                 0x0
#define PCRM_CAL_RO1_CNT_TRACK_16M_FINA_RST                0x0

__INLINE void pcrm_cal_ro1_unpack(uint8_t* actucntfina32k, uint8_t* cnttrack16mfina)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO1_OFFSET + PCRM_BASE_ADDR);

    *actucntfina32k = (localVal & ((uint32_t)0xFFF80000)) >> 19;
    *cnttrack16mfina = (localVal & ((uint32_t)0x0001FFFF)) >> 0;
}

__INLINE uint16_t pcrm_cal_ro1_actu_cnt_fina_32k_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFF80000)) >> 19);
}

__INLINE uint32_t pcrm_cal_ro1_cnt_track_16m_fina_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0001FFFF)) >> 0);
}

 /**
 * @brief CAL_RO2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     10           rccal_done   0
 *     09          rc32k_ready   0
 *     08         cal_done_32M   0
 *  04:00      rccal_word_fina   0b0
 * </pre>
 */
#define PCRM_CAL_RO2_OFFSET 0x00000068


__INLINE uint32_t pcrm_cal_ro2_get(void)
{
    return _PICO_REG_RD(PCRM_CAL_RO2_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_CAL_RO2_RCCAL_DONE_BIT                        ((uint32_t)0x00000400)
#define PCRM_CAL_RO2_RCCAL_DONE_POS                        10
#define PCRM_CAL_RO2_RC32K_READY_BIT                       ((uint32_t)0x00000200)
#define PCRM_CAL_RO2_RC32K_READY_POS                       9
#define PCRM_CAL_RO2_CAL_DONE_32M_BIT                      ((uint32_t)0x00000100)
#define PCRM_CAL_RO2_CAL_DONE_32M_POS                      8
#define PCRM_CAL_RO2_RCCAL_WORD_FINA_MASK                  ((uint32_t)0x0000001F)
#define PCRM_CAL_RO2_RCCAL_WORD_FINA_LSB                   0
#define PCRM_CAL_RO2_RCCAL_WORD_FINA_WIDTH                 ((uint32_t)0x00000005)

#define PCRM_CAL_RO2_RCCAL_DONE_RST                        0x0
#define PCRM_CAL_RO2_RC32K_READY_RST                       0x0
#define PCRM_CAL_RO2_CAL_DONE_32M_RST                      0x0
#define PCRM_CAL_RO2_RCCAL_WORD_FINA_RST                   0x0

__INLINE void pcrm_cal_ro2_unpack(uint8_t* rccaldone, uint8_t* rc32kready, uint8_t* caldone32m, uint8_t* rccalwordfina)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO2_OFFSET + PCRM_BASE_ADDR);

    *rccaldone = (localVal & ((uint32_t)0x00000400)) >> 10;
    *rc32kready = (localVal & ((uint32_t)0x00000200)) >> 9;
    *caldone32m = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rccalwordfina = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t pcrm_cal_ro2_rccal_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t pcrm_cal_ro2_rc32k_ready_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t pcrm_cal_ro2_cal_done_32m_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t pcrm_cal_ro2_rccal_word_fina_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CAL_RO2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

 /**
 * @brief ADC_CTL0 register definition
 */
#define PCRM_ADC_CTL0_OFFSET 0x0000006C


__INLINE uint32_t pcrm_adc_ctl0_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_CTL0_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_ADC_CTL0_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define PCRM_ADC_CTL0_RESERVED_LSB                          0
#define PCRM_ADC_CTL0_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define PCRM_ADC_CTL0_RESERVED_RST                          0x0

__INLINE void pcrm_adc_ctl0_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL0_OFFSET + PCRM_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief ADC_CTL1 register definition
 */
#define PCRM_ADC_CTL1_OFFSET 0x00000070


__INLINE uint32_t pcrm_adc_ctl1_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_CTL1_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_ADC_CTL1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define PCRM_ADC_CTL1_RESERVED_LSB                          0
#define PCRM_ADC_CTL1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define PCRM_ADC_CTL1_RESERVED_RST                          0x0

__INLINE void pcrm_adc_ctl1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL1_OFFSET + PCRM_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief ADC_CTL2 register definition
 */
#define PCRM_ADC_CTL2_OFFSET 0x00000074


__INLINE uint32_t pcrm_adc_ctl2_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_CTL2_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_ADC_CTL2_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define PCRM_ADC_CTL2_RESERVED_LSB                          0
#define PCRM_ADC_CTL2_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define PCRM_ADC_CTL2_RESERVED_RST                          0x0

__INLINE void pcrm_adc_ctl2_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL2_OFFSET + PCRM_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief ADC_CTL3 register definition
 */
#define PCRM_ADC_CTL3_OFFSET 0x00000078


__INLINE uint32_t pcrm_adc_ctl3_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_CTL3_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_ADC_CTL3_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define PCRM_ADC_CTL3_RESERVED_LSB                          0
#define PCRM_ADC_CTL3_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define PCRM_ADC_CTL3_RESERVED_RST                          0x0

__INLINE void pcrm_adc_ctl3_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL3_OFFSET + PCRM_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief ADC_CTL4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:20          adc_cmp_hth   0b1
 *  19:08          adc_cmp_lth   0b1
 *     05    adc_cmp_direction   1
 *     04           adc_cmp_en   1
 *     03        adc_tconv_sel   0
 *  02:01          adc_clk_sel   0b10
 *     00               adc_en   1
 * </pre>
 */
#define PCRM_ADC_CTL4_OFFSET 0x0000007C


__INLINE uint32_t pcrm_adc_ctl4_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_adc_ctl4_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_ADC_CTL4_ADC_CMP_HTH_MASK                      ((uint32_t)0xFFF00000)
#define PCRM_ADC_CTL4_ADC_CMP_HTH_LSB                       20
#define PCRM_ADC_CTL4_ADC_CMP_HTH_WIDTH                     ((uint32_t)0x0000000C)
#define PCRM_ADC_CTL4_ADC_CMP_LTH_MASK                      ((uint32_t)0x000FFF00)
#define PCRM_ADC_CTL4_ADC_CMP_LTH_LSB                       8
#define PCRM_ADC_CTL4_ADC_CMP_LTH_WIDTH                     ((uint32_t)0x0000000C)
#define PCRM_ADC_CTL4_ADC_CMP_DIRECTION_BIT                 ((uint32_t)0x00000020)
#define PCRM_ADC_CTL4_ADC_CMP_DIRECTION_POS                 5
#define PCRM_ADC_CTL4_ADC_CMP_EN_BIT                        ((uint32_t)0x00000010)
#define PCRM_ADC_CTL4_ADC_CMP_EN_POS                        4
#define PCRM_ADC_CTL4_ADC_TCONV_SEL_BIT                     ((uint32_t)0x00000008)
#define PCRM_ADC_CTL4_ADC_TCONV_SEL_POS                     3
#define PCRM_ADC_CTL4_ADC_CLK_SEL_MASK                      ((uint32_t)0x00000006)
#define PCRM_ADC_CTL4_ADC_CLK_SEL_LSB                       1
#define PCRM_ADC_CTL4_ADC_CLK_SEL_WIDTH                     ((uint32_t)0x00000002)
#define PCRM_ADC_CTL4_ADC_EN_BIT                            ((uint32_t)0x00000001)
#define PCRM_ADC_CTL4_ADC_EN_POS                            0

#define PCRM_ADC_CTL4_ADC_CMP_HTH_RST                       0x1
#define PCRM_ADC_CTL4_ADC_CMP_LTH_RST                       0x1
#define PCRM_ADC_CTL4_ADC_CMP_DIRECTION_RST                 0x1
#define PCRM_ADC_CTL4_ADC_CMP_EN_RST                        0x1
#define PCRM_ADC_CTL4_ADC_TCONV_SEL_RST                     0x0
#define PCRM_ADC_CTL4_ADC_CLK_SEL_RST                       0x10
#define PCRM_ADC_CTL4_ADC_EN_RST                            0x1

__INLINE void pcrm_adc_ctl4_pack(uint16_t adccmphth, uint16_t adccmplth, uint8_t adccmpdirection, uint8_t adccmpen, uint8_t adctconvsel, uint8_t adcclksel, uint8_t adcen)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)adccmphth << 20) | ((uint32_t)adccmplth << 8) | ((uint32_t)adccmpdirection << 5) | ((uint32_t)adccmpen << 4) | ((uint32_t)adctconvsel << 3) | ((uint32_t)adcclksel << 1) | ((uint32_t)adcen << 0));
}

__INLINE void pcrm_adc_ctl4_unpack(uint8_t* adccmphth, uint8_t* adccmplth, uint8_t* adccmpdirection, uint8_t* adccmpen, uint8_t* adctconvsel, uint8_t* adcclksel, uint8_t* adcen)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);

    *adccmphth = (localVal & ((uint32_t)0xFFF00000)) >> 20;
    *adccmplth = (localVal & ((uint32_t)0x000FFF00)) >> 8;
    *adccmpdirection = (localVal & ((uint32_t)0x00000020)) >> 5;
    *adccmpen = (localVal & ((uint32_t)0x00000010)) >> 4;
    *adctconvsel = (localVal & ((uint32_t)0x00000008)) >> 3;
    *adcclksel = (localVal & ((uint32_t)0x00000006)) >> 1;
    *adcen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t pcrm_adc_ctl4_adc_cmp_hth_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFF00000)) >> 20);
}

__INLINE void pcrm_adc_ctl4_adc_cmp_hth_setf(uint16_t adccmphth)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0xFFF00000)) | ((uint32_t)adccmphth << 20));
}

__INLINE uint16_t pcrm_adc_ctl4_adc_cmp_lth_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFF00)) >> 8);
}

__INLINE void pcrm_adc_ctl4_adc_cmp_lth_setf(uint16_t adccmplth)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x000FFF00)) | ((uint32_t)adccmplth << 8));
}

__INLINE uint8_t pcrm_adc_ctl4_adc_cmp_direction_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void pcrm_adc_ctl4_adc_cmp_direction_setf(uint8_t adccmpdirection)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)adccmpdirection << 5));
}

__INLINE uint8_t pcrm_adc_ctl4_adc_cmp_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void pcrm_adc_ctl4_adc_cmp_en_setf(uint8_t adccmpen)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)adccmpen << 4));
}

__INLINE uint8_t pcrm_adc_ctl4_adc_tconv_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void pcrm_adc_ctl4_adc_tconv_sel_setf(uint8_t adctconvsel)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)adctconvsel << 3));
}

__INLINE uint8_t pcrm_adc_ctl4_adc_clk_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000006)) >> 1);
}

__INLINE void pcrm_adc_ctl4_adc_clk_sel_setf(uint8_t adcclksel)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000006)) | ((uint32_t)adcclksel << 1));
}

__INLINE uint8_t pcrm_adc_ctl4_adc_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcrm_adc_ctl4_adc_en_setf(uint8_t adcen)
{
    _PICO_REG_WR(PCRM_ADC_CTL4_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ADC_CTL4_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)adcen << 0));
}

 /**
 * @brief ADC_INT_CLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00          adc_int_clr   0b0
 * </pre>
 */
#define PCRM_ADC_INT_CLR_OFFSET 0x00000080


__INLINE uint32_t pcrm_adc_int_clr_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_INT_CLR_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_adc_int_clr_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_ADC_INT_CLR_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_ADC_INT_CLR_ADC_INT_CLR_MASK                      ((uint32_t)0x00000003)
#define PCRM_ADC_INT_CLR_ADC_INT_CLR_LSB                       0
#define PCRM_ADC_INT_CLR_ADC_INT_CLR_WIDTH                     ((uint32_t)0x00000002)

#define PCRM_ADC_INT_CLR_ADC_INT_CLR_RST                       0x0

__INLINE void pcrm_adc_int_clr_pack(uint8_t adcintclr)
{
    _PICO_REG_WR(PCRM_ADC_INT_CLR_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)adcintclr << 0));
}

__INLINE void pcrm_adc_int_clr_unpack(uint8_t* adcintclr)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_INT_CLR_OFFSET + PCRM_BASE_ADDR);

    *adcintclr = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t pcrm_adc_int_clr_adc_int_clr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_INT_CLR_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void pcrm_adc_int_clr_adc_int_clr_setf(uint8_t adcintclr)
{
    _PICO_REG_WR(PCRM_ADC_INT_CLR_OFFSET+ PCRM_BASE_ADDR, (uint32_t)adcintclr << 0);
}

 /**
 * @brief ADC_INT_MASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00         adc_int_mask   0b0
 * </pre>
 */
#define PCRM_ADC_INT_MASK_OFFSET 0x00000084


__INLINE uint32_t pcrm_adc_int_mask_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_INT_MASK_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_adc_int_mask_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_ADC_INT_MASK_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_ADC_INT_MASK_ADC_INT_MASK_MASK                     ((uint32_t)0x00000003)
#define PCRM_ADC_INT_MASK_ADC_INT_MASK_LSB                      0
#define PCRM_ADC_INT_MASK_ADC_INT_MASK_WIDTH                    ((uint32_t)0x00000002)

#define PCRM_ADC_INT_MASK_ADC_INT_MASK_RST                      0x0

__INLINE void pcrm_adc_int_mask_pack(uint8_t adcintmask)
{
    _PICO_REG_WR(PCRM_ADC_INT_MASK_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)adcintmask << 0));
}

__INLINE void pcrm_adc_int_mask_unpack(uint8_t* adcintmask)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_INT_MASK_OFFSET + PCRM_BASE_ADDR);

    *adcintmask = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t pcrm_adc_int_mask_adc_int_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_INT_MASK_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void pcrm_adc_int_mask_adc_int_mask_setf(uint8_t adcintmask)
{
    _PICO_REG_WR(PCRM_ADC_INT_MASK_OFFSET+ PCRM_BASE_ADDR, (uint32_t)adcintmask << 0);
}

 /**
 * @brief ADC_INT_SRC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00          adc_int_src   0b0
 * </pre>
 */
#define PCRM_ADC_INT_SRC_OFFSET 0x00000088


__INLINE uint32_t pcrm_adc_int_src_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_INT_SRC_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_ADC_INT_SRC_ADC_INT_SRC_MASK                      ((uint32_t)0x00000003)
#define PCRM_ADC_INT_SRC_ADC_INT_SRC_LSB                       0
#define PCRM_ADC_INT_SRC_ADC_INT_SRC_WIDTH                     ((uint32_t)0x00000002)

#define PCRM_ADC_INT_SRC_ADC_INT_SRC_RST                       0x0

__INLINE void pcrm_adc_int_src_unpack(uint8_t* adcintsrc)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_INT_SRC_OFFSET + PCRM_BASE_ADDR);

    *adcintsrc = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t pcrm_adc_int_src_adc_int_src_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_INT_SRC_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

 /**
 * @brief ADC_SYNC_DATA_OUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:00    adc_sync_data_out   0b0
 * </pre>
 */
#define PCRM_ADC_SYNC_DATA_OUT_OFFSET 0x0000008C


__INLINE uint32_t pcrm_adc_sync_data_out_get(void)
{
    return _PICO_REG_RD(PCRM_ADC_SYNC_DATA_OUT_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_ADC_SYNC_DATA_OUT_ADC_SYNC_DATA_OUT_MASK                ((uint32_t)0x00000FFF)
#define PCRM_ADC_SYNC_DATA_OUT_ADC_SYNC_DATA_OUT_LSB                 0
#define PCRM_ADC_SYNC_DATA_OUT_ADC_SYNC_DATA_OUT_WIDTH               ((uint32_t)0x0000000C)

#define PCRM_ADC_SYNC_DATA_OUT_ADC_SYNC_DATA_OUT_RST                 0x0

__INLINE void pcrm_adc_sync_data_out_unpack(uint8_t* adcsyncdataout)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_SYNC_DATA_OUT_OFFSET + PCRM_BASE_ADDR);

    *adcsyncdataout = (localVal & ((uint32_t)0x00000FFF)) >> 0;
}

__INLINE uint16_t pcrm_adc_sync_data_out_adc_sync_data_out_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ADC_SYNC_DATA_OUT_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000FFF)) >> 0);
}

 /**
 * @brief CHANGE_STATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01          change_done   0
 *     00            change_on   0
 * </pre>
 */
#define PCRM_CHANGE_STATE_OFFSET 0x00000184


__INLINE uint32_t pcrm_change_state_get(void)
{
    return _PICO_REG_RD(PCRM_CHANGE_STATE_OFFSET + PCRM_BASE_ADDR);
}

// field definitions
#define PCRM_CHANGE_STATE_CHANGE_DONE_BIT                       ((uint32_t)0x00000002)
#define PCRM_CHANGE_STATE_CHANGE_DONE_POS                       1
#define PCRM_CHANGE_STATE_CHANGE_ON_BIT                         ((uint32_t)0x00000001)
#define PCRM_CHANGE_STATE_CHANGE_ON_POS                         0

#define PCRM_CHANGE_STATE_CHANGE_DONE_RST                       0x0
#define PCRM_CHANGE_STATE_CHANGE_ON_RST                         0x0

__INLINE void pcrm_change_state_unpack(uint8_t* changedone, uint8_t* changeon)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CHANGE_STATE_OFFSET + PCRM_BASE_ADDR);

    *changedone = (localVal & ((uint32_t)0x00000002)) >> 1;
    *changeon = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcrm_change_state_change_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CHANGE_STATE_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t pcrm_change_state_change_on_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_CHANGE_STATE_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief I_CLKHF_CTRL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31              oe_xc_o   0
 *     30              od_xc_o   0
 *     29              oe_xc_i   0
 *     28              od_xc_i   0
 *     25         xtal_irq_eoi   0
 *     24        xtal_irq_mask   0
 *  23:12   xtal_track_cnt_target   0b250
 *  11:04   xtal_track_settle_th   0b10
 *     02         xtal_irq_raw   0
 *     01          xtal_settle   0
 *     00        en_xtal_track   0
 * </pre>
 */
#define PCRM_I_CLKHF_CTRL2_OFFSET 0x00000188


__INLINE uint32_t pcrm_i_clkhf_ctrl2_get(void)
{
    return _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_i_clkhf_ctrl2_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_I_CLKHF_CTRL2_OE_XC_O_BIT                           ((uint32_t)0x80000000)
#define PCRM_I_CLKHF_CTRL2_OE_XC_O_POS                           31
#define PCRM_I_CLKHF_CTRL2_OD_XC_O_BIT                           ((uint32_t)0x40000000)
#define PCRM_I_CLKHF_CTRL2_OD_XC_O_POS                           30
#define PCRM_I_CLKHF_CTRL2_OE_XC_I_BIT                           ((uint32_t)0x20000000)
#define PCRM_I_CLKHF_CTRL2_OE_XC_I_POS                           29
#define PCRM_I_CLKHF_CTRL2_OD_XC_I_BIT                           ((uint32_t)0x10000000)
#define PCRM_I_CLKHF_CTRL2_OD_XC_I_POS                           28
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_EOI_BIT                      ((uint32_t)0x02000000)
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_EOI_POS                      25
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_MASK_BIT                     ((uint32_t)0x01000000)
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_MASK_POS                     24
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_CNT_TARGET_MASK            ((uint32_t)0x00FFF000)
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_CNT_TARGET_LSB             12
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_CNT_TARGET_WIDTH           ((uint32_t)0x0000000C)
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_SETTLE_TH_MASK             ((uint32_t)0x00000FF0)
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_SETTLE_TH_LSB              4
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_SETTLE_TH_WIDTH            ((uint32_t)0x00000008)
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_RAW_BIT                      ((uint32_t)0x00000004)
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_RAW_POS                      2
#define PCRM_I_CLKHF_CTRL2_XTAL_SETTLE_BIT                       ((uint32_t)0x00000002)
#define PCRM_I_CLKHF_CTRL2_XTAL_SETTLE_POS                       1
#define PCRM_I_CLKHF_CTRL2_EN_XTAL_TRACK_BIT                     ((uint32_t)0x00000001)
#define PCRM_I_CLKHF_CTRL2_EN_XTAL_TRACK_POS                     0

#define PCRM_I_CLKHF_CTRL2_OE_XC_O_RST                           0x0
#define PCRM_I_CLKHF_CTRL2_OD_XC_O_RST                           0x0
#define PCRM_I_CLKHF_CTRL2_OE_XC_I_RST                           0x0
#define PCRM_I_CLKHF_CTRL2_OD_XC_I_RST                           0x0
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_EOI_RST                      0x0
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_MASK_RST                     0x0
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_CNT_TARGET_RST             0x250
#define PCRM_I_CLKHF_CTRL2_XTAL_TRACK_SETTLE_TH_RST              0x10
#define PCRM_I_CLKHF_CTRL2_XTAL_IRQ_RAW_RST                      0x0
#define PCRM_I_CLKHF_CTRL2_XTAL_SETTLE_RST                       0x0
#define PCRM_I_CLKHF_CTRL2_EN_XTAL_TRACK_RST                     0x0

__INLINE void pcrm_i_clkhf_ctrl2_pack(uint8_t oexco, uint8_t odxco, uint8_t oexci, uint8_t odxci, uint8_t xtalirqeoi, uint8_t xtalirqmask, uint16_t xtaltrackcnttarget, uint8_t xtaltracksettleth, uint8_t enxtaltrack)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)oexco << 31) | ((uint32_t)odxco << 30) | ((uint32_t)oexci << 29) | ((uint32_t)odxci << 28) | ((uint32_t)xtalirqeoi << 25) | ((uint32_t)xtalirqmask << 24) | ((uint32_t)xtaltrackcnttarget << 12) | ((uint32_t)xtaltracksettleth << 4) | ((uint32_t)enxtaltrack << 0));
}

__INLINE void pcrm_i_clkhf_ctrl2_unpack(uint8_t* oexco, uint8_t* odxco, uint8_t* oexci, uint8_t* odxci, uint8_t* xtalirqeoi, uint8_t* xtalirqmask, uint8_t* xtaltrackcnttarget, uint8_t* xtaltracksettleth, uint8_t* xtalirqraw, uint8_t* xtalsettle, uint8_t* enxtaltrack)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);

    *oexco = (localVal & ((uint32_t)0x80000000)) >> 31;
    *odxco = (localVal & ((uint32_t)0x40000000)) >> 30;
    *oexci = (localVal & ((uint32_t)0x20000000)) >> 29;
    *odxci = (localVal & ((uint32_t)0x10000000)) >> 28;
    *xtalirqeoi = (localVal & ((uint32_t)0x02000000)) >> 25;
    *xtalirqmask = (localVal & ((uint32_t)0x01000000)) >> 24;
    *xtaltrackcnttarget = (localVal & ((uint32_t)0x00FFF000)) >> 12;
    *xtaltracksettleth = (localVal & ((uint32_t)0x00000FF0)) >> 4;
    *xtalirqraw = (localVal & ((uint32_t)0x00000004)) >> 2;
    *xtalsettle = (localVal & ((uint32_t)0x00000002)) >> 1;
    *enxtaltrack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_oe_xc_o_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void pcrm_i_clkhf_ctrl2_oe_xc_o_setf(uint8_t oexco)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)oexco << 31));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_od_xc_o_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x40000000)) >> 30);
}

__INLINE void pcrm_i_clkhf_ctrl2_od_xc_o_setf(uint8_t odxco)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x40000000)) | ((uint32_t)odxco << 30));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_oe_xc_i_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void pcrm_i_clkhf_ctrl2_oe_xc_i_setf(uint8_t oexci)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)oexci << 29));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_od_xc_i_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void pcrm_i_clkhf_ctrl2_od_xc_i_setf(uint8_t odxci)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)odxci << 28));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_xtal_irq_eoi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void pcrm_i_clkhf_ctrl2_xtal_irq_eoi_setf(uint8_t xtalirqeoi)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)xtalirqeoi << 25));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_xtal_irq_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void pcrm_i_clkhf_ctrl2_xtal_irq_mask_setf(uint8_t xtalirqmask)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)xtalirqmask << 24));
}

__INLINE uint16_t pcrm_i_clkhf_ctrl2_xtal_track_cnt_target_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFF000)) >> 12);
}

__INLINE void pcrm_i_clkhf_ctrl2_xtal_track_cnt_target_setf(uint16_t xtaltrackcnttarget)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00FFF000)) | ((uint32_t)xtaltrackcnttarget << 12));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_xtal_track_settle_th_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000FF0)) >> 4);
}

__INLINE void pcrm_i_clkhf_ctrl2_xtal_track_settle_th_setf(uint8_t xtaltracksettleth)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000FF0)) | ((uint32_t)xtaltracksettleth << 4));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_xtal_irq_raw_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_xtal_settle_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t pcrm_i_clkhf_ctrl2_en_xtal_track_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcrm_i_clkhf_ctrl2_en_xtal_track_setf(uint8_t enxtaltrack)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL2_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL2_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)enxtaltrack << 0));
}

 /**
 * @brief I_CLKHF_CTRL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     25          dll_irq_eoi   0
 *     24         dll_irq_mask   0
 *  23:12   dll_track_cnt_target   0b500
 *  11:04   dll_track_settle_th   0b20
 *     02          dll_irq_raw   0
 *     01           dll_settle   0
 *     00         en_dll_track   0
 * </pre>
 */
#define PCRM_I_CLKHF_CTRL3_OFFSET 0x0000018C


__INLINE uint32_t pcrm_i_clkhf_ctrl3_get(void)
{
    return _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_i_clkhf_ctrl3_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL3_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_EOI_BIT                       ((uint32_t)0x02000000)
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_EOI_POS                       25
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_MASK_BIT                      ((uint32_t)0x01000000)
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_MASK_POS                      24
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_CNT_TARGET_MASK             ((uint32_t)0x00FFF000)
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_CNT_TARGET_LSB              12
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_CNT_TARGET_WIDTH            ((uint32_t)0x0000000C)
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_SETTLE_TH_MASK              ((uint32_t)0x00000FF0)
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_SETTLE_TH_LSB               4
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_SETTLE_TH_WIDTH             ((uint32_t)0x00000008)
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_RAW_BIT                       ((uint32_t)0x00000004)
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_RAW_POS                       2
#define PCRM_I_CLKHF_CTRL3_DLL_SETTLE_BIT                        ((uint32_t)0x00000002)
#define PCRM_I_CLKHF_CTRL3_DLL_SETTLE_POS                        1
#define PCRM_I_CLKHF_CTRL3_EN_DLL_TRACK_BIT                      ((uint32_t)0x00000001)
#define PCRM_I_CLKHF_CTRL3_EN_DLL_TRACK_POS                      0

#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_EOI_RST                       0x0
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_MASK_RST                      0x0
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_CNT_TARGET_RST              0x500
#define PCRM_I_CLKHF_CTRL3_DLL_TRACK_SETTLE_TH_RST               0x20
#define PCRM_I_CLKHF_CTRL3_DLL_IRQ_RAW_RST                       0x0
#define PCRM_I_CLKHF_CTRL3_DLL_SETTLE_RST                        0x0
#define PCRM_I_CLKHF_CTRL3_EN_DLL_TRACK_RST                      0x0

__INLINE void pcrm_i_clkhf_ctrl3_pack(uint8_t dllirqeoi, uint8_t dllirqmask, uint16_t dlltrackcnttarget, uint8_t dlltracksettleth, uint8_t endlltrack)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL3_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)dllirqeoi << 25) | ((uint32_t)dllirqmask << 24) | ((uint32_t)dlltrackcnttarget << 12) | ((uint32_t)dlltracksettleth << 4) | ((uint32_t)endlltrack << 0));
}

__INLINE void pcrm_i_clkhf_ctrl3_unpack(uint8_t* dllirqeoi, uint8_t* dllirqmask, uint8_t* dlltrackcnttarget, uint8_t* dlltracksettleth, uint8_t* dllirqraw, uint8_t* dllsettle, uint8_t* endlltrack)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);

    *dllirqeoi = (localVal & ((uint32_t)0x02000000)) >> 25;
    *dllirqmask = (localVal & ((uint32_t)0x01000000)) >> 24;
    *dlltrackcnttarget = (localVal & ((uint32_t)0x00FFF000)) >> 12;
    *dlltracksettleth = (localVal & ((uint32_t)0x00000FF0)) >> 4;
    *dllirqraw = (localVal & ((uint32_t)0x00000004)) >> 2;
    *dllsettle = (localVal & ((uint32_t)0x00000002)) >> 1;
    *endlltrack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t pcrm_i_clkhf_ctrl3_dll_irq_eoi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void pcrm_i_clkhf_ctrl3_dll_irq_eoi_setf(uint8_t dllirqeoi)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL3_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)dllirqeoi << 25));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl3_dll_irq_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void pcrm_i_clkhf_ctrl3_dll_irq_mask_setf(uint8_t dllirqmask)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL3_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)dllirqmask << 24));
}

__INLINE uint16_t pcrm_i_clkhf_ctrl3_dll_track_cnt_target_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFF000)) >> 12);
}

__INLINE void pcrm_i_clkhf_ctrl3_dll_track_cnt_target_setf(uint16_t dlltrackcnttarget)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL3_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00FFF000)) | ((uint32_t)dlltrackcnttarget << 12));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl3_dll_track_settle_th_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000FF0)) >> 4);
}

__INLINE void pcrm_i_clkhf_ctrl3_dll_track_settle_th_setf(uint8_t dlltracksettleth)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL3_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000FF0)) | ((uint32_t)dlltracksettleth << 4));
}

__INLINE uint8_t pcrm_i_clkhf_ctrl3_dll_irq_raw_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t pcrm_i_clkhf_ctrl3_dll_settle_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t pcrm_i_clkhf_ctrl3_en_dll_track_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void pcrm_i_clkhf_ctrl3_en_dll_track_setf(uint8_t endlltrack)
{
    _PICO_REG_WR(PCRM_I_CLKHF_CTRL3_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_I_CLKHF_CTRL3_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)endlltrack << 0));
}

 /**
 * @brief ANA_CTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     29          vbat_det_en   0
 *     28          vdd3_det_en   0
 *  27:26            aio7_ctrl   0b0
 *  25:24            aio6_ctrl   0b0
 *  23:22            aio5_ctrl   0b0
 *  21:20            aio4_ctrl   0b0
 *  19:18            aio3_ctrl   0b0
 *  17:16            aio2_ctrl   0b0
 *  15:14            aio1_ctrl   0b0
 *  13:12            aio0_ctrl   0b0
 *  11:00             reserved   0b0
 * </pre>
 */
#define PCRM_ANA_CTL1_OFFSET 0x00000190


__INLINE uint32_t pcrm_ana_ctl1_get(void)
{
    return _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
}

__INLINE void pcrm_ana_ctl1_set(uint32_t value)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, value);
}

// field definitions
#define PCRM_ANA_CTL1_VBAT_DET_EN_BIT                       ((uint32_t)0x20000000)
#define PCRM_ANA_CTL1_VBAT_DET_EN_POS                       29
#define PCRM_ANA_CTL1_VDD3_DET_EN_BIT                       ((uint32_t)0x10000000)
#define PCRM_ANA_CTL1_VDD3_DET_EN_POS                       28
#define PCRM_ANA_CTL1_AIO7_CTRL_MASK                        ((uint32_t)0x0C000000)
#define PCRM_ANA_CTL1_AIO7_CTRL_LSB                         26
#define PCRM_ANA_CTL1_AIO7_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_AIO6_CTRL_MASK                        ((uint32_t)0x03000000)
#define PCRM_ANA_CTL1_AIO6_CTRL_LSB                         24
#define PCRM_ANA_CTL1_AIO6_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_AIO5_CTRL_MASK                        ((uint32_t)0x00C00000)
#define PCRM_ANA_CTL1_AIO5_CTRL_LSB                         22
#define PCRM_ANA_CTL1_AIO5_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_AIO4_CTRL_MASK                        ((uint32_t)0x00300000)
#define PCRM_ANA_CTL1_AIO4_CTRL_LSB                         20
#define PCRM_ANA_CTL1_AIO4_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_AIO3_CTRL_MASK                        ((uint32_t)0x000C0000)
#define PCRM_ANA_CTL1_AIO3_CTRL_LSB                         18
#define PCRM_ANA_CTL1_AIO3_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_AIO2_CTRL_MASK                        ((uint32_t)0x00030000)
#define PCRM_ANA_CTL1_AIO2_CTRL_LSB                         16
#define PCRM_ANA_CTL1_AIO2_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_AIO1_CTRL_MASK                        ((uint32_t)0x0000C000)
#define PCRM_ANA_CTL1_AIO1_CTRL_LSB                         14
#define PCRM_ANA_CTL1_AIO1_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_AIO0_CTRL_MASK                        ((uint32_t)0x00003000)
#define PCRM_ANA_CTL1_AIO0_CTRL_LSB                         12
#define PCRM_ANA_CTL1_AIO0_CTRL_WIDTH                       ((uint32_t)0x00000002)
#define PCRM_ANA_CTL1_RESERVED_MASK                         ((uint32_t)0x00000FFF)
#define PCRM_ANA_CTL1_RESERVED_LSB                          0
#define PCRM_ANA_CTL1_RESERVED_WIDTH                        ((uint32_t)0x0000000C)

#define PCRM_ANA_CTL1_VBAT_DET_EN_RST                       0x0
#define PCRM_ANA_CTL1_VDD3_DET_EN_RST                       0x0
#define PCRM_ANA_CTL1_AIO7_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_AIO6_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_AIO5_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_AIO4_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_AIO3_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_AIO2_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_AIO1_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_AIO0_CTRL_RST                         0x0
#define PCRM_ANA_CTL1_RESERVED_RST                          0x0

__INLINE void pcrm_ana_ctl1_pack(uint8_t vbatdeten, uint8_t vdd3deten, uint8_t aio7ctrl, uint8_t aio6ctrl, uint8_t aio5ctrl, uint8_t aio4ctrl, uint8_t aio3ctrl, uint8_t aio2ctrl, uint8_t aio1ctrl, uint8_t aio0ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR,  ((uint32_t)vbatdeten << 29) | ((uint32_t)vdd3deten << 28) | ((uint32_t)aio7ctrl << 26) | ((uint32_t)aio6ctrl << 24) | ((uint32_t)aio5ctrl << 22) | ((uint32_t)aio4ctrl << 20) | ((uint32_t)aio3ctrl << 18) | ((uint32_t)aio2ctrl << 16) | ((uint32_t)aio1ctrl << 14) | ((uint32_t)aio0ctrl << 12));
}

__INLINE void pcrm_ana_ctl1_unpack(uint8_t* vbatdeten, uint8_t* vdd3deten, uint8_t* aio7ctrl, uint8_t* aio6ctrl, uint8_t* aio5ctrl, uint8_t* aio4ctrl, uint8_t* aio3ctrl, uint8_t* aio2ctrl, uint8_t* aio1ctrl, uint8_t* aio0ctrl, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);

    *vbatdeten = (localVal & ((uint32_t)0x20000000)) >> 29;
    *vdd3deten = (localVal & ((uint32_t)0x10000000)) >> 28;
    *aio7ctrl = (localVal & ((uint32_t)0x0C000000)) >> 26;
    *aio6ctrl = (localVal & ((uint32_t)0x03000000)) >> 24;
    *aio5ctrl = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *aio4ctrl = (localVal & ((uint32_t)0x00300000)) >> 20;
    *aio3ctrl = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *aio2ctrl = (localVal & ((uint32_t)0x00030000)) >> 16;
    *aio1ctrl = (localVal & ((uint32_t)0x0000C000)) >> 14;
    *aio0ctrl = (localVal & ((uint32_t)0x00003000)) >> 12;
    *reserved = (localVal & ((uint32_t)0x00000FFF)) >> 0;
}

__INLINE uint8_t pcrm_ana_ctl1_vbat_det_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x20000000)) >> 29);
}

__INLINE void pcrm_ana_ctl1_vbat_det_en_setf(uint8_t vbatdeten)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x20000000)) | ((uint32_t)vbatdeten << 29));
}

__INLINE uint8_t pcrm_ana_ctl1_vdd3_det_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x10000000)) >> 28);
}

__INLINE void pcrm_ana_ctl1_vdd3_det_en_setf(uint8_t vdd3deten)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x10000000)) | ((uint32_t)vdd3deten << 28));
}

__INLINE uint8_t pcrm_ana_ctl1_aio7_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0C000000)) >> 26);
}

__INLINE void pcrm_ana_ctl1_aio7_ctrl_setf(uint8_t aio7ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x0C000000)) | ((uint32_t)aio7ctrl << 26));
}

__INLINE uint8_t pcrm_ana_ctl1_aio6_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void pcrm_ana_ctl1_aio6_ctrl_setf(uint8_t aio6ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)aio6ctrl << 24));
}

__INLINE uint8_t pcrm_ana_ctl1_aio5_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE void pcrm_ana_ctl1_aio5_ctrl_setf(uint8_t aio5ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00C00000)) | ((uint32_t)aio5ctrl << 22));
}

__INLINE uint8_t pcrm_ana_ctl1_aio4_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00300000)) >> 20);
}

__INLINE void pcrm_ana_ctl1_aio4_ctrl_setf(uint8_t aio4ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00300000)) | ((uint32_t)aio4ctrl << 20));
}

__INLINE uint8_t pcrm_ana_ctl1_aio3_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void pcrm_ana_ctl1_aio3_ctrl_setf(uint8_t aio3ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)aio3ctrl << 18));
}

__INLINE uint8_t pcrm_ana_ctl1_aio2_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void pcrm_ana_ctl1_aio2_ctrl_setf(uint8_t aio2ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)aio2ctrl << 16));
}

__INLINE uint8_t pcrm_ana_ctl1_aio1_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void pcrm_ana_ctl1_aio1_ctrl_setf(uint8_t aio1ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)aio1ctrl << 14));
}

__INLINE uint8_t pcrm_ana_ctl1_aio0_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void pcrm_ana_ctl1_aio0_ctrl_setf(uint8_t aio0ctrl)
{
    _PICO_REG_WR(PCRM_ANA_CTL1_OFFSET+ PCRM_BASE_ADDR, (_PICO_REG_RD(PCRM_ANA_CTL1_OFFSET + PCRM_BASE_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)aio0ctrl << 12));
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t :5;
      __IO uint32_t spif_ref_clk_sel:3;
      __IO uint32_t :6;
      __IO uint32_t aon_pclk_inv:1;
      __IO uint32_t lowclk_sel:1;
      __IO uint32_t :7;
      __IO uint32_t sel_16m:1;
      __IO uint32_t :1;
      __IO uint32_t clk_1p28m_en:1;
      __IO uint32_t :1;
      __IO uint32_t hclk_mux_done_override:1;
      __IO uint32_t hclk_sel_en_override:1;
      __IO uint32_t hclk_sel:3;
    }CLKSEL_fld;
    __IO uint32_t CLKSEL;
  };

  union{ //offset addr 0x0040
    struct{
      __IO uint32_t :13;
      __IO uint32_t xtal_clk_dig_en:1;
      __IO uint32_t :18;
    }CLKHF_CTL0_fld;
    __IO uint32_t CLKHF_CTL0;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t :6;
      __IO uint32_t rxadc_clk_sel:2;
      __IO uint32_t rf_clk_sel:2;
      __IO uint32_t dig_clk_32m_sel:2;
      __IO uint32_t en_rxadc_clk_32m:1;
      __IO uint32_t en_rf_clk:1;
      __IO uint32_t :1;
      __IO uint32_t en_dig_clk_96m:1;
      __IO uint32_t en_dig_clk_64m:1;
      __IO uint32_t en_dig_clk_48m:1;
      __IO uint32_t en_dig_clk_32m:1;
      __IO uint32_t :2;
      __IO uint32_t dbl_cap_tune:2;
      __IO uint32_t dbl_en:1;
      __IO uint32_t dll_en:1;
      __IO uint32_t dll_ldo_vctrl:2;
      __IO uint32_t dll_ldo_pu:1;
      __IO uint32_t :4;
    }CLKHF_CTL1_fld;
    __IO uint32_t CLKHF_CTL1;
  };

  union{ //offset addr 0x0048
    struct{
      __IO uint32_t :5;
      __IO uint32_t micbias_vref_fil_en:1;
      __IO uint32_t micbias_out_ctrl:2;
      __IO uint32_t micbias_en:1;
      __IO uint32_t pga_1st_gain:1;
      __IO uint32_t pga_2nd_gain:3;
      __IO uint32_t pga_ldo_outctrl:2;
      __IO uint32_t pga_en:1;
      __IO uint32_t :4;
      __IO uint32_t adc12b_semode_enm:1;
      __IO uint32_t adc_dly_ctl:2;
      __IO uint32_t adc12b_semode_enp:1;
      __IO uint32_t adc12b_chn_sel:3;
      __IO uint32_t :1;
      __IO uint32_t adc12b_en:1;
      __IO uint32_t ana_ldo_vctrl:2;
      __IO uint32_t ana_ldo_en:1;
    }ANA_CTL_fld;
    __IO uint32_t ANA_CTL;
  };

  union{ //offset addr 0x004c
    struct{
      __IO uint32_t ram1_dvse:1;
      __IO uint32_t ram1_dvs:3;
      __IO uint32_t ram0_dvse:1;
      __IO uint32_t ram0_dvs:3;
      __IO uint32_t :19;
      __IO uint32_t rom0_dvse:1;
      __IO uint32_t rom0_dvs:4;
    }mem_0_1_dvs_fld;
    __IO uint32_t mem_0_1_dvs;
  };

  union{ //offset addr 0x0050
    struct{
      __IO uint32_t :3;
      __IO uint32_t bb_ram_dvse:1;
      __IO uint32_t bb_ram_dvs:4;
      __IO uint32_t :24;
    }mem_2_3_4_dvs_fld;
    __IO uint32_t mem_2_3_4_dvs;
  };


  union{ //offset addr 0x005c
    struct{
      __IO uint32_t :15;
      __IO uint32_t rc32k_clk_sel:1;
      __IO uint32_t :7;
      __IO uint32_t num_track_cycle:5;
      __IO uint32_t track_en_rc32k:1;
      __IO uint32_t rccal_en:1;
      __IO uint32_t cal_en_32m:1;
      __IO uint32_t cal_en_32k:1;
    }cal_rw_fld;
    __IO uint32_t cal_rw;
  };

  union{ //offset addr 0x0060
    struct{
      __IO uint32_t actu_cnt_fina_32k:10;
      __IO uint32_t cal_word_fina_32k:6;
      __IO uint32_t actu_cnt_fina_32m:10;
      __IO uint32_t cal_word_fina_32m:6;
    }cal_ro0_fld;
    __IO uint32_t cal_ro0;
  };

  union{ //offset addr 0x0064
    struct{
      __IO uint32_t actu_cnt_fina_32k:13;
      __IO uint32_t :2;
      __IO uint32_t cnt_track_16m_fina:17;
    }cal_ro1_fld;
    __IO uint32_t cal_ro1;
  };

  union{ //offset addr 0x0068
    struct{
      __IO uint32_t :21;
      __IO uint32_t rccal_done:1;
      __IO uint32_t rc32k_ready:1;
      __IO uint32_t cal_done_32m:1;
      __IO uint32_t :3;
      __IO uint32_t rccal_word_fina:5;
    }cal_ro2_fld;
    __IO uint32_t cal_ro2;
  };

  union{ //offset addr 0x006c
    struct{
      __IO uint32_t :32;
    }ADC_CTL0_fld;
    __IO uint32_t ADC_CTL0;
  };

  union{ //offset addr 0x0070
    struct{
      __IO uint32_t :32;
    }ADC_CTL1_fld;
    __IO uint32_t ADC_CTL1;
  };

  union{ //offset addr 0x0074
    struct{
      __IO uint32_t :32;
    }ADC_CTL2_fld;
    __IO uint32_t ADC_CTL2;
  };

  union{ //offset addr 0x0078
    struct{
      __IO uint32_t :32;
    }ADC_CTL3_fld;
    __IO uint32_t ADC_CTL3;
  };

  union{ //offset addr 0x007c
    struct{
      __IO uint32_t adc_cmp_hth:12;
      __IO uint32_t adc_cmp_lth:12;
      __IO uint32_t :2;
      __IO uint32_t adc_cmp_direction:1;
      __IO uint32_t adc_cmp_en:1;
      __IO uint32_t adc_tconv_sel:1;
      __IO uint32_t adc_clk_sel:2;
      __IO uint32_t adc_en:1;
    }ADC_CTL4_fld;
    __IO uint32_t ADC_CTL4;
  };

  union{ //offset addr 0x0080
    struct{
      __IO uint32_t :30;
      __IO uint32_t adc_int_clr:2;
    }ADC_INT_CLR_fld;
    __IO uint32_t ADC_INT_CLR;
  };

  union{ //offset addr 0x0084
    struct{
      __IO uint32_t :30;
      __IO uint32_t adc_int_mask:2;
    }adc_int_mask_fld;
    __IO uint32_t adc_int_mask;
  };

  union{ //offset addr 0x0088
    struct{
      __IO uint32_t :30;
      __IO uint32_t adc_int_src:2;
    }adc_int_src_fld;
    __IO uint32_t adc_int_src;
  };

  union{ //offset addr 0x008c
    struct{
      __IO uint32_t :20;
      __IO uint32_t adc_sync_data_out:12;
    }adc_sync_data_out_fld;
    __IO uint32_t adc_sync_data_out;
  };


  union{ //offset addr 0x0184
    struct{
      __IO uint32_t :30;
      __IO uint32_t change_done:1;
      __IO uint32_t change_on:1;
    }change_state_fld;
    __IO uint32_t change_state;
  };

  union{ //offset addr 0x0188
    struct{
      __IO uint32_t oe_xc_o:1;
      __IO uint32_t od_xc_o:1;
      __IO uint32_t oe_xc_i:1;
      __IO uint32_t od_xc_i:1;
      __IO uint32_t :2;
      __IO uint32_t xtal_irq_eoi:1;
      __IO uint32_t xtal_irq_mask:1;
      __IO uint32_t xtal_track_cnt_target:12;
      __IO uint32_t xtal_track_settle_th:8;
      __IO uint32_t :1;
      __IO uint32_t xtal_irq_raw:1;
      __IO uint32_t xtal_settle:1;
      __IO uint32_t en_xtal_track:1;
    }i_clkhf_ctrl2_fld;
    __IO uint32_t i_clkhf_ctrl2;
  };

  union{ //offset addr 0x018c
    struct{
      __IO uint32_t :6;
      __IO uint32_t dll_irq_eoi:1;
      __IO uint32_t dll_irq_mask:1;
      __IO uint32_t dll_track_cnt_target:12;
      __IO uint32_t dll_track_settle_th:8;
      __IO uint32_t :1;
      __IO uint32_t dll_irq_raw:1;
      __IO uint32_t dll_settle:1;
      __IO uint32_t en_dll_track:1;
    }i_clkhf_ctrl3_fld;
    __IO uint32_t i_clkhf_ctrl3;
  };

  union{ //offset addr 0x0190
    struct{
      __IO uint32_t :2;
      __IO uint32_t vbat_det_en:1;
      __IO uint32_t vdd3_det_en:1;
      __IO uint32_t aio7_ctrl:2;
      __IO uint32_t aio6_ctrl:2;
      __IO uint32_t aio5_ctrl:2;
      __IO uint32_t aio4_ctrl:2;
      __IO uint32_t aio3_ctrl:2;
      __IO uint32_t aio2_ctrl:2;
      __IO uint32_t aio1_ctrl:2;
      __IO uint32_t aio0_ctrl:2;
      __IO uint32_t :12;
    }ANA_CTL1_fld;
    __IO uint32_t ANA_CTL1;
  };

} PICO_REG_PCRM_TypeDef;

#define PICO_REG_PCRM PICO_REG_PCRM_TypeDef *0x4000F000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_PCRM_H_

