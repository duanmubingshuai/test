#ifndef _PICO_REG_BB_TOP_H_
#define _PICO_REG_BB_TOP_H_

#include <stdint.h>

#define BB_TOP_COUNT 56

#define BB_TOP_BASE_ADDR 0x40030000

#define BB_TOP_SIZE 0x0000010C


 /**
 * @brief I_DIG_BB_CTRL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24          rx_sync_thd   0b55
 *  23:21          blr_acc_err   0b0
 *     20           zb_sfd_err   0
 *  19:18         sync_sch_dly   0b1
 *  17:16             sync_len   0b2
 *  15:08           rx_iq_fsel   0b80
 *     05      rx_32m_clk_gate   0
 *     04      tx_32m_clk_gate   0
 *     03          clk32m_rise   0
 *  02:00              pkt_fmt   0b1
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL0_OFFSET 0x00000000


__INLINE uint32_t bb_top_i_dig_bb_ctrl0_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl0_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL0_RX_SYNC_THD_MASK                      ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL0_RX_SYNC_THD_LSB                       24
#define BB_TOP_I_DIG_BB_CTRL0_RX_SYNC_THD_WIDTH                     ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL0_BLR_ACC_ERR_MASK                      ((uint32_t)0x00E00000)
#define BB_TOP_I_DIG_BB_CTRL0_BLR_ACC_ERR_LSB                       21
#define BB_TOP_I_DIG_BB_CTRL0_BLR_ACC_ERR_WIDTH                     ((uint32_t)0x00000003)
#define BB_TOP_I_DIG_BB_CTRL0_ZB_SFD_ERR_BIT                        ((uint32_t)0x00100000)
#define BB_TOP_I_DIG_BB_CTRL0_ZB_SFD_ERR_POS                        20
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_SCH_DLY_MASK                     ((uint32_t)0x000C0000)
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_SCH_DLY_LSB                      18
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_SCH_DLY_WIDTH                    ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_LEN_MASK                         ((uint32_t)0x00030000)
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_LEN_LSB                          16
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_LEN_WIDTH                        ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL0_RX_IQ_FSEL_MASK                       ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL0_RX_IQ_FSEL_LSB                        8
#define BB_TOP_I_DIG_BB_CTRL0_RX_IQ_FSEL_WIDTH                      ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL0_RX_32M_CLK_GATE_BIT                   ((uint32_t)0x00000020)
#define BB_TOP_I_DIG_BB_CTRL0_RX_32M_CLK_GATE_POS                   5
#define BB_TOP_I_DIG_BB_CTRL0_TX_32M_CLK_GATE_BIT                   ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL0_TX_32M_CLK_GATE_POS                   4
#define BB_TOP_I_DIG_BB_CTRL0_CLK32M_RISE_BIT                       ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL0_CLK32M_RISE_POS                       3
#define BB_TOP_I_DIG_BB_CTRL0_PKT_FMT_MASK                          ((uint32_t)0x00000007)
#define BB_TOP_I_DIG_BB_CTRL0_PKT_FMT_LSB                           0
#define BB_TOP_I_DIG_BB_CTRL0_PKT_FMT_WIDTH                         ((uint32_t)0x00000003)

#define BB_TOP_I_DIG_BB_CTRL0_RX_SYNC_THD_RST                       0x55
#define BB_TOP_I_DIG_BB_CTRL0_BLR_ACC_ERR_RST                       0x0
#define BB_TOP_I_DIG_BB_CTRL0_ZB_SFD_ERR_RST                        0x0
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_SCH_DLY_RST                      0x1
#define BB_TOP_I_DIG_BB_CTRL0_SYNC_LEN_RST                          0x2
#define BB_TOP_I_DIG_BB_CTRL0_RX_IQ_FSEL_RST                        0x80
#define BB_TOP_I_DIG_BB_CTRL0_RX_32M_CLK_GATE_RST                   0x0
#define BB_TOP_I_DIG_BB_CTRL0_TX_32M_CLK_GATE_RST                   0x0
#define BB_TOP_I_DIG_BB_CTRL0_CLK32M_RISE_RST                       0x0
#define BB_TOP_I_DIG_BB_CTRL0_PKT_FMT_RST                           0x1

__INLINE void bb_top_i_dig_bb_ctrl0_pack(uint8_t rxsyncthd, uint8_t blraccerr, uint8_t zbsfderr, uint8_t syncschdly, uint8_t synclen, uint8_t rxiqfsel, uint8_t rx32mclkgate, uint8_t tx32mclkgate, uint8_t clk32mrise, uint8_t pktfmt)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)rxsyncthd << 24) | ((uint32_t)blraccerr << 21) | ((uint32_t)zbsfderr << 20) | ((uint32_t)syncschdly << 18) | ((uint32_t)synclen << 16) | ((uint32_t)rxiqfsel << 8) | ((uint32_t)rx32mclkgate << 5) | ((uint32_t)tx32mclkgate << 4) | ((uint32_t)clk32mrise << 3) | ((uint32_t)pktfmt << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl0_unpack(uint8_t* rxsyncthd, uint8_t* blraccerr, uint8_t* zbsfderr, uint8_t* syncschdly, uint8_t* synclen, uint8_t* rxiqfsel, uint8_t* rx32mclkgate, uint8_t* tx32mclkgate, uint8_t* clk32mrise, uint8_t* pktfmt)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);

    *rxsyncthd = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *blraccerr = (localVal & ((uint32_t)0x00E00000)) >> 21;
    *zbsfderr = (localVal & ((uint32_t)0x00100000)) >> 20;
    *syncschdly = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *synclen = (localVal & ((uint32_t)0x00030000)) >> 16;
    *rxiqfsel = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rx32mclkgate = (localVal & ((uint32_t)0x00000020)) >> 5;
    *tx32mclkgate = (localVal & ((uint32_t)0x00000010)) >> 4;
    *clk32mrise = (localVal & ((uint32_t)0x00000008)) >> 3;
    *pktfmt = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_rx_sync_thd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl0_rx_sync_thd_setf(uint8_t rxsyncthd)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)rxsyncthd << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_blr_acc_err_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00E00000)) >> 21);
}

__INLINE void bb_top_i_dig_bb_ctrl0_blr_acc_err_setf(uint8_t blraccerr)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00E00000)) | ((uint32_t)blraccerr << 21));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_zb_sfd_err_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void bb_top_i_dig_bb_ctrl0_zb_sfd_err_setf(uint8_t zbsfderr)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)zbsfderr << 20));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_sync_sch_dly_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void bb_top_i_dig_bb_ctrl0_sync_sch_dly_setf(uint8_t syncschdly)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)syncschdly << 18));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_sync_len_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl0_sync_len_setf(uint8_t synclen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)synclen << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_rx_iq_fsel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl0_rx_iq_fsel_setf(uint8_t rxiqfsel)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)rxiqfsel << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_rx_32m_clk_gate_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bb_top_i_dig_bb_ctrl0_rx_32m_clk_gate_setf(uint8_t rx32mclkgate)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rx32mclkgate << 5));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_tx_32m_clk_gate_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl0_tx_32m_clk_gate_setf(uint8_t tx32mclkgate)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)tx32mclkgate << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_clk32m_rise_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bb_top_i_dig_bb_ctrl0_clk32m_rise_setf(uint8_t clk32mrise)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)clk32mrise << 3));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl0_pkt_fmt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl0_pkt_fmt_setf(uint8_t pktfmt)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)pktfmt << 0));
}

 /**
 * @brief I_DIG_BB_CTRL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        sync_rssi_thd   0b90
 *  23:16       rx_sync_thd_hi   0b150
 *  14:08       diff_demod_thd   0b90
 *  07:04            tr_dc_sft   0b5
 *  03:00          tr_tedm_sft   0b4
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL1_OFFSET 0x00000004


__INLINE uint32_t bb_top_i_dig_bb_ctrl1_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl1_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL1_SYNC_RSSI_THD_MASK                    ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL1_SYNC_RSSI_THD_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL1_SYNC_RSSI_THD_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL1_RX_SYNC_THD_HI_MASK                   ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL1_RX_SYNC_THD_HI_LSB                    16
#define BB_TOP_I_DIG_BB_CTRL1_RX_SYNC_THD_HI_WIDTH                  ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL1_DIFF_DEMOD_THD_MASK                   ((uint32_t)0x00007F00)
#define BB_TOP_I_DIG_BB_CTRL1_DIFF_DEMOD_THD_LSB                    8
#define BB_TOP_I_DIG_BB_CTRL1_DIFF_DEMOD_THD_WIDTH                  ((uint32_t)0x00000007)
#define BB_TOP_I_DIG_BB_CTRL1_TR_DC_SFT_MASK                        ((uint32_t)0x000000F0)
#define BB_TOP_I_DIG_BB_CTRL1_TR_DC_SFT_LSB                         4
#define BB_TOP_I_DIG_BB_CTRL1_TR_DC_SFT_WIDTH                       ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL1_TR_TEDM_SFT_MASK                      ((uint32_t)0x0000000F)
#define BB_TOP_I_DIG_BB_CTRL1_TR_TEDM_SFT_LSB                       0
#define BB_TOP_I_DIG_BB_CTRL1_TR_TEDM_SFT_WIDTH                     ((uint32_t)0x00000004)

#define BB_TOP_I_DIG_BB_CTRL1_SYNC_RSSI_THD_RST                     0x90
#define BB_TOP_I_DIG_BB_CTRL1_RX_SYNC_THD_HI_RST                    0x150
#define BB_TOP_I_DIG_BB_CTRL1_DIFF_DEMOD_THD_RST                    0x90
#define BB_TOP_I_DIG_BB_CTRL1_TR_DC_SFT_RST                         0x5
#define BB_TOP_I_DIG_BB_CTRL1_TR_TEDM_SFT_RST                       0x4

__INLINE void bb_top_i_dig_bb_ctrl1_pack(uint8_t syncrssithd, uint8_t rxsyncthdhi, uint8_t diffdemodthd, uint8_t trdcsft, uint8_t trtedmsft)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL1_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)syncrssithd << 24) | ((uint32_t)rxsyncthdhi << 16) | ((uint32_t)diffdemodthd << 8) | ((uint32_t)trdcsft << 4) | ((uint32_t)trtedmsft << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl1_unpack(uint8_t* syncrssithd, uint8_t* rxsyncthdhi, uint8_t* diffdemodthd, uint8_t* trdcsft, uint8_t* trtedmsft)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);

    *syncrssithd = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *rxsyncthdhi = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *diffdemodthd = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *trdcsft = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *trtedmsft = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl1_sync_rssi_thd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl1_sync_rssi_thd_setf(uint8_t syncrssithd)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)syncrssithd << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl1_rx_sync_thd_hi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl1_rx_sync_thd_hi_setf(uint8_t rxsyncthdhi)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxsyncthdhi << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl1_diff_demod_thd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl1_diff_demod_thd_setf(uint8_t diffdemodthd)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)diffdemodthd << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl1_tr_dc_sft_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl1_tr_dc_sft_setf(uint8_t trdcsft)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)trdcsft << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl1_tr_tedm_sft_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl1_tr_tedm_sft_setf(uint8_t trtedmsft)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)trtedmsft << 0));
}

 /**
 * @brief I_DIG_BB_CTRL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24          ahdfe_ki_cr   0b32
 *  23:16          ahdfe_kp_cr   0b96
 *     08             ahdfe_en   1
 *  07:04          ahdfe_hidx1   0b9
 *  03:00          ahdfe_hidx0   0b2
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL2_OFFSET 0x00000008


__INLINE uint32_t bb_top_i_dig_bb_ctrl2_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl2_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KI_CR_MASK                      ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KI_CR_LSB                       24
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KI_CR_WIDTH                     ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KP_CR_MASK                      ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KP_CR_LSB                       16
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KP_CR_WIDTH                     ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_EN_BIT                          ((uint32_t)0x00000100)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_EN_POS                          8
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX1_MASK                      ((uint32_t)0x000000F0)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX1_LSB                       4
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX1_WIDTH                     ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX0_MASK                      ((uint32_t)0x0000000F)
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX0_LSB                       0
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX0_WIDTH                     ((uint32_t)0x00000004)

#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KI_CR_RST                       0x32
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_KP_CR_RST                       0x96
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_EN_RST                          0x1
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX1_RST                       0x9
#define BB_TOP_I_DIG_BB_CTRL2_AHDFE_HIDX0_RST                       0x2

__INLINE void bb_top_i_dig_bb_ctrl2_pack(uint8_t ahdfekicr, uint8_t ahdfekpcr, uint8_t ahdfeen, uint8_t ahdfehidx1, uint8_t ahdfehidx0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL2_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)ahdfekicr << 24) | ((uint32_t)ahdfekpcr << 16) | ((uint32_t)ahdfeen << 8) | ((uint32_t)ahdfehidx1 << 4) | ((uint32_t)ahdfehidx0 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl2_unpack(uint8_t* ahdfekicr, uint8_t* ahdfekpcr, uint8_t* ahdfeen, uint8_t* ahdfehidx1, uint8_t* ahdfehidx0)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);

    *ahdfekicr = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *ahdfekpcr = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *ahdfeen = (localVal & ((uint32_t)0x00000100)) >> 8;
    *ahdfehidx1 = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *ahdfehidx0 = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl2_ahdfe_ki_cr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl2_ahdfe_ki_cr_setf(uint8_t ahdfekicr)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)ahdfekicr << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl2_ahdfe_kp_cr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl2_ahdfe_kp_cr_setf(uint8_t ahdfekpcr)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)ahdfekpcr << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl2_ahdfe_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl2_ahdfe_en_setf(uint8_t ahdfeen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)ahdfeen << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl2_ahdfe_hidx1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl2_ahdfe_hidx1_setf(uint8_t ahdfehidx1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)ahdfehidx1 << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl2_ahdfe_hidx0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl2_ahdfe_hidx0_setf(uint8_t ahdfehidx0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)ahdfehidx0 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16          rx_time_out   0b280
 *  15:08         rx_supp_samp   0b0
 *  07:00    rx_max_packet_len   0b128
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL3_OFFSET 0x0000000C


__INLINE uint32_t bb_top_i_dig_bb_ctrl3_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl3_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL3_RX_TIME_OUT_MASK                      ((uint32_t)0xFFFF0000)
#define BB_TOP_I_DIG_BB_CTRL3_RX_TIME_OUT_LSB                       16
#define BB_TOP_I_DIG_BB_CTRL3_RX_TIME_OUT_WIDTH                     ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL3_RX_SUPP_SAMP_MASK                     ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL3_RX_SUPP_SAMP_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL3_RX_SUPP_SAMP_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL3_RX_MAX_PACKET_LEN_MASK                ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL3_RX_MAX_PACKET_LEN_LSB                 0
#define BB_TOP_I_DIG_BB_CTRL3_RX_MAX_PACKET_LEN_WIDTH               ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL3_RX_TIME_OUT_RST                       0x280
#define BB_TOP_I_DIG_BB_CTRL3_RX_SUPP_SAMP_RST                      0x0
#define BB_TOP_I_DIG_BB_CTRL3_RX_MAX_PACKET_LEN_RST                 0x128

__INLINE void bb_top_i_dig_bb_ctrl3_pack(uint16_t rxtimeout, uint8_t rxsuppsamp, uint8_t rxmaxpacketlen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL3_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)rxtimeout << 16) | ((uint32_t)rxsuppsamp << 8) | ((uint32_t)rxmaxpacketlen << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl3_unpack(uint8_t* rxtimeout, uint8_t* rxsuppsamp, uint8_t* rxmaxpacketlen)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);

    *rxtimeout = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *rxsuppsamp = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rxmaxpacketlen = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint16_t bb_top_i_dig_bb_ctrl3_rx_time_out_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl3_rx_time_out_setf(uint16_t rxtimeout)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)rxtimeout << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl3_rx_supp_samp_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl3_rx_supp_samp_setf(uint8_t rxsuppsamp)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)rxsuppsamp << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl3_rx_max_packet_len_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl3_rx_max_packet_len_setf(uint8_t rxmaxpacketlen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rxmaxpacketlen << 0));
}

 /**
 * @brief I_DIG_BB_CTRL4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:24        dc_compq_flt3   0b32
 *  21:16        dc_compi_flt3   0b32
 *  13:08        dc_compq_flt4   0b32
 *  05:00        dc_compi_flt4   0b32
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL4_OFFSET 0x00000010


__INLINE uint32_t bb_top_i_dig_bb_ctrl4_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl4_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL4_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT3_MASK                    ((uint32_t)0x3F000000)
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT3_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT3_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT3_MASK                    ((uint32_t)0x003F0000)
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT3_LSB                     16
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT3_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT4_MASK                    ((uint32_t)0x00003F00)
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT4_LSB                     8
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT4_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT4_MASK                    ((uint32_t)0x0000003F)
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT4_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT4_WIDTH                   ((uint32_t)0x00000006)

#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT3_RST                     0x32
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT3_RST                     0x32
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPQ_FLT4_RST                     0x32
#define BB_TOP_I_DIG_BB_CTRL4_DC_COMPI_FLT4_RST                     0x32

__INLINE void bb_top_i_dig_bb_ctrl4_pack(uint8_t dccompqflt3, uint8_t dccompiflt3, uint8_t dccompqflt4, uint8_t dccompiflt4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL4_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)dccompqflt3 << 24) | ((uint32_t)dccompiflt3 << 16) | ((uint32_t)dccompqflt4 << 8) | ((uint32_t)dccompiflt4 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl4_unpack(uint8_t* dccompqflt3, uint8_t* dccompiflt3, uint8_t* dccompqflt4, uint8_t* dccompiflt4)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);

    *dccompqflt3 = (localVal & ((uint32_t)0x3F000000)) >> 24;
    *dccompiflt3 = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *dccompqflt4 = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *dccompiflt4 = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl4_dc_compq_flt3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x3F000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl4_dc_compq_flt3_setf(uint8_t dccompqflt3)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL4_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x3F000000)) | ((uint32_t)dccompqflt3 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl4_dc_compi_flt3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl4_dc_compi_flt3_setf(uint8_t dccompiflt3)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL4_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x003F0000)) | ((uint32_t)dccompiflt3 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl4_dc_compq_flt4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl4_dc_compq_flt4_setf(uint8_t dccompqflt4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL4_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00003F00)) | ((uint32_t)dccompqflt4 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl4_dc_compi_flt4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl4_dc_compi_flt4_setf(uint8_t dccompiflt4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL4_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000003F)) | ((uint32_t)dccompiflt4 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL16 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24         tx_supp_tail   0b0
 *  19:16           tx_iq_fsel   0b3
 *  15:08         tx_pld_nbyte   0b40
 *  07:05       tx_pre_len_ext   0b0
 *     04          tx_mode_sel   0
 *  03:00              tx_pldt   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL16_OFFSET 0x00000040


__INLINE uint32_t bb_top_i_dig_bb_ctrl16_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl16_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL16_TX_SUPP_TAIL_MASK                     ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL16_TX_SUPP_TAIL_LSB                      24
#define BB_TOP_I_DIG_BB_CTRL16_TX_SUPP_TAIL_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL16_TX_IQ_FSEL_MASK                       ((uint32_t)0x000F0000)
#define BB_TOP_I_DIG_BB_CTRL16_TX_IQ_FSEL_LSB                        16
#define BB_TOP_I_DIG_BB_CTRL16_TX_IQ_FSEL_WIDTH                      ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLD_NBYTE_MASK                     ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLD_NBYTE_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLD_NBYTE_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL16_TX_PRE_LEN_EXT_MASK                   ((uint32_t)0x000000E0)
#define BB_TOP_I_DIG_BB_CTRL16_TX_PRE_LEN_EXT_LSB                    5
#define BB_TOP_I_DIG_BB_CTRL16_TX_PRE_LEN_EXT_WIDTH                  ((uint32_t)0x00000003)
#define BB_TOP_I_DIG_BB_CTRL16_TX_MODE_SEL_BIT                       ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL16_TX_MODE_SEL_POS                       4
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLDT_MASK                          ((uint32_t)0x0000000F)
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLDT_LSB                           0
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLDT_WIDTH                         ((uint32_t)0x00000004)

#define BB_TOP_I_DIG_BB_CTRL16_TX_SUPP_TAIL_RST                      0x0
#define BB_TOP_I_DIG_BB_CTRL16_TX_IQ_FSEL_RST                        0x3
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLD_NBYTE_RST                      0x40
#define BB_TOP_I_DIG_BB_CTRL16_TX_PRE_LEN_EXT_RST                    0x0
#define BB_TOP_I_DIG_BB_CTRL16_TX_MODE_SEL_RST                       0x0
#define BB_TOP_I_DIG_BB_CTRL16_TX_PLDT_RST                           0x0

__INLINE void bb_top_i_dig_bb_ctrl16_pack(uint8_t txsupptail, uint8_t txiqfsel, uint8_t txpldnbyte, uint8_t txprelenext, uint8_t txmodesel, uint8_t txpldt)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)txsupptail << 24) | ((uint32_t)txiqfsel << 16) | ((uint32_t)txpldnbyte << 8) | ((uint32_t)txprelenext << 5) | ((uint32_t)txmodesel << 4) | ((uint32_t)txpldt << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl16_unpack(uint8_t* txsupptail, uint8_t* txiqfsel, uint8_t* txpldnbyte, uint8_t* txprelenext, uint8_t* txmodesel, uint8_t* txpldt)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);

    *txsupptail = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *txiqfsel = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *txpldnbyte = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *txprelenext = (localVal & ((uint32_t)0x000000E0)) >> 5;
    *txmodesel = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txpldt = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl16_tx_supp_tail_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl16_tx_supp_tail_setf(uint8_t txsupptail)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)txsupptail << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl16_tx_iq_fsel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl16_tx_iq_fsel_setf(uint8_t txiqfsel)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)txiqfsel << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl16_tx_pld_nbyte_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl16_tx_pld_nbyte_setf(uint8_t txpldnbyte)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)txpldnbyte << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl16_tx_pre_len_ext_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000E0)) >> 5);
}

__INLINE void bb_top_i_dig_bb_ctrl16_tx_pre_len_ext_setf(uint8_t txprelenext)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000E0)) | ((uint32_t)txprelenext << 5));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl16_tx_mode_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl16_tx_mode_sel_setf(uint8_t txmodesel)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)txmodesel << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl16_tx_pldt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl16_tx_pldt_setf(uint8_t txpldt)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL16_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL16_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)txpldt << 0));
}

 /**
 * @brief I_DIG_BB_CTRL17 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         tx_prbs_seed   0b1F
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL17_OFFSET 0x00000044


__INLINE uint32_t bb_top_i_dig_bb_ctrl17_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL17_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl17_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL17_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL17_TX_PRBS_SEED_MASK                     ((uint32_t)0xFFFFFFFF)
#define BB_TOP_I_DIG_BB_CTRL17_TX_PRBS_SEED_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL17_TX_PRBS_SEED_WIDTH                    ((uint32_t)0x00000020)

#define BB_TOP_I_DIG_BB_CTRL17_TX_PRBS_SEED_RST                      0x1F

__INLINE void bb_top_i_dig_bb_ctrl17_pack(uint32_t txprbsseed)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL17_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)txprbsseed << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl17_unpack(uint8_t* txprbsseed)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL17_OFFSET + BB_TOP_BASE_ADDR);

    *txprbsseed = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t bb_top_i_dig_bb_ctrl17_tx_prbs_seed_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL17_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl17_tx_prbs_seed_setf(uint32_t txprbsseed)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL17_OFFSET+ BB_TOP_BASE_ADDR, (uint32_t)txprbsseed << 0);
}

 /**
 * @brief I_DIG_BB_CTRL18 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  30:24           tx_wt_seed   0b37
 *  23:00            tx_crc_24   0b555555
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL18_OFFSET 0x00000048


__INLINE uint32_t bb_top_i_dig_bb_ctrl18_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL18_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl18_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL18_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL18_TX_WT_SEED_MASK                       ((uint32_t)0x7F000000)
#define BB_TOP_I_DIG_BB_CTRL18_TX_WT_SEED_LSB                        24
#define BB_TOP_I_DIG_BB_CTRL18_TX_WT_SEED_WIDTH                      ((uint32_t)0x00000007)
#define BB_TOP_I_DIG_BB_CTRL18_TX_CRC_24_MASK                        ((uint32_t)0x00FFFFFF)
#define BB_TOP_I_DIG_BB_CTRL18_TX_CRC_24_LSB                         0
#define BB_TOP_I_DIG_BB_CTRL18_TX_CRC_24_WIDTH                       ((uint32_t)0x00000018)

#define BB_TOP_I_DIG_BB_CTRL18_TX_WT_SEED_RST                        0x37
#define BB_TOP_I_DIG_BB_CTRL18_TX_CRC_24_RST                         0x555555

__INLINE void bb_top_i_dig_bb_ctrl18_pack(uint8_t txwtseed, uint32_t txcrc24)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL18_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)txwtseed << 24) | ((uint32_t)txcrc24 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl18_unpack(uint8_t* txwtseed, uint8_t* txcrc24)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL18_OFFSET + BB_TOP_BASE_ADDR);

    *txwtseed = (localVal & ((uint32_t)0x7F000000)) >> 24;
    *txcrc24 = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl18_tx_wt_seed_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL18_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x7F000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl18_tx_wt_seed_setf(uint8_t txwtseed)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL18_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL18_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x7F000000)) | ((uint32_t)txwtseed << 24));
}

__INLINE uint32_t bb_top_i_dig_bb_ctrl18_tx_crc_24_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL18_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl18_tx_crc_24_setf(uint32_t txcrc24)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL18_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL18_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FFFFFF)) | ((uint32_t)txcrc24 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL19 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00       tx_ble_acc_adr   0b8E89BEB6
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL19_OFFSET 0x0000004C


__INLINE uint32_t bb_top_i_dig_bb_ctrl19_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL19_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl19_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL19_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL19_TX_BLE_ACC_ADR_MASK                   ((uint32_t)0xFFFFFFFF)
#define BB_TOP_I_DIG_BB_CTRL19_TX_BLE_ACC_ADR_LSB                    0
#define BB_TOP_I_DIG_BB_CTRL19_TX_BLE_ACC_ADR_WIDTH                  ((uint32_t)0x00000020)

#define BB_TOP_I_DIG_BB_CTRL19_TX_BLE_ACC_ADR_RST                    0x8E89BEB6

__INLINE void bb_top_i_dig_bb_ctrl19_pack(uint32_t txbleaccadr)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL19_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)txbleaccadr << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl19_unpack(uint8_t* txbleaccadr)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL19_OFFSET + BB_TOP_BASE_ADDR);

    *txbleaccadr = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t bb_top_i_dig_bb_ctrl19_tx_ble_acc_adr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL19_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl19_tx_ble_acc_adr_setf(uint32_t txbleaccadr)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL19_OFFSET+ BB_TOP_BASE_ADDR, (uint32_t)txbleaccadr << 0);
}

 /**
 * @brief I_DIG_BB_CTRL20 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24          rssi_offset   0b32
 *  23:20         agc_time_out   0b0
 *  19:16           agc_settle   0b8
 *  15:12             rx_pwr_t   0b5
 *  11:08             rx_abs_t   0b5
 *     07               agc_en   1
 *     04         agc_fix_gain   0
 *  03:00             agc_gain   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL20_OFFSET 0x00000050


__INLINE uint32_t bb_top_i_dig_bb_ctrl20_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl20_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL20_RSSI_OFFSET_MASK                      ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL20_RSSI_OFFSET_LSB                       24
#define BB_TOP_I_DIG_BB_CTRL20_RSSI_OFFSET_WIDTH                     ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_TIME_OUT_MASK                     ((uint32_t)0x00F00000)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_TIME_OUT_LSB                      20
#define BB_TOP_I_DIG_BB_CTRL20_AGC_TIME_OUT_WIDTH                    ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_SETTLE_MASK                       ((uint32_t)0x000F0000)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_SETTLE_LSB                        16
#define BB_TOP_I_DIG_BB_CTRL20_AGC_SETTLE_WIDTH                      ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL20_RX_PWR_T_MASK                         ((uint32_t)0x0000F000)
#define BB_TOP_I_DIG_BB_CTRL20_RX_PWR_T_LSB                          12
#define BB_TOP_I_DIG_BB_CTRL20_RX_PWR_T_WIDTH                        ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL20_RX_ABS_T_MASK                         ((uint32_t)0x00000F00)
#define BB_TOP_I_DIG_BB_CTRL20_RX_ABS_T_LSB                          8
#define BB_TOP_I_DIG_BB_CTRL20_RX_ABS_T_WIDTH                        ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_EN_BIT                            ((uint32_t)0x00000080)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_EN_POS                            7
#define BB_TOP_I_DIG_BB_CTRL20_AGC_FIX_GAIN_BIT                      ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_FIX_GAIN_POS                      4
#define BB_TOP_I_DIG_BB_CTRL20_AGC_GAIN_MASK                         ((uint32_t)0x0000000F)
#define BB_TOP_I_DIG_BB_CTRL20_AGC_GAIN_LSB                          0
#define BB_TOP_I_DIG_BB_CTRL20_AGC_GAIN_WIDTH                        ((uint32_t)0x00000004)

#define BB_TOP_I_DIG_BB_CTRL20_RSSI_OFFSET_RST                       0x32
#define BB_TOP_I_DIG_BB_CTRL20_AGC_TIME_OUT_RST                      0x0
#define BB_TOP_I_DIG_BB_CTRL20_AGC_SETTLE_RST                        0x8
#define BB_TOP_I_DIG_BB_CTRL20_RX_PWR_T_RST                          0x5
#define BB_TOP_I_DIG_BB_CTRL20_RX_ABS_T_RST                          0x5
#define BB_TOP_I_DIG_BB_CTRL20_AGC_EN_RST                            0x1
#define BB_TOP_I_DIG_BB_CTRL20_AGC_FIX_GAIN_RST                      0x0
#define BB_TOP_I_DIG_BB_CTRL20_AGC_GAIN_RST                          0x0

__INLINE void bb_top_i_dig_bb_ctrl20_pack(uint8_t rssioffset, uint8_t agctimeout, uint8_t agcsettle, uint8_t rxpwrt, uint8_t rxabst, uint8_t agcen, uint8_t agcfixgain, uint8_t agcgain)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)rssioffset << 24) | ((uint32_t)agctimeout << 20) | ((uint32_t)agcsettle << 16) | ((uint32_t)rxpwrt << 12) | ((uint32_t)rxabst << 8) | ((uint32_t)agcen << 7) | ((uint32_t)agcfixgain << 4) | ((uint32_t)agcgain << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl20_unpack(uint8_t* rssioffset, uint8_t* agctimeout, uint8_t* agcsettle, uint8_t* rxpwrt, uint8_t* rxabst, uint8_t* agcen, uint8_t* agcfixgain, uint8_t* agcgain)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);

    *rssioffset = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctimeout = (localVal & ((uint32_t)0x00F00000)) >> 20;
    *agcsettle = (localVal & ((uint32_t)0x000F0000)) >> 16;
    *rxpwrt = (localVal & ((uint32_t)0x0000F000)) >> 12;
    *rxabst = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *agcen = (localVal & ((uint32_t)0x00000080)) >> 7;
    *agcfixgain = (localVal & ((uint32_t)0x00000010)) >> 4;
    *agcgain = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_rssi_offset_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl20_rssi_offset_setf(uint8_t rssioffset)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)rssioffset << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_agc_time_out_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00F00000)) >> 20);
}

__INLINE void bb_top_i_dig_bb_ctrl20_agc_time_out_setf(uint8_t agctimeout)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00F00000)) | ((uint32_t)agctimeout << 20));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_agc_settle_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000F0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl20_agc_settle_setf(uint8_t agcsettle)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000F0000)) | ((uint32_t)agcsettle << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_rx_pwr_t_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000F000)) >> 12);
}

__INLINE void bb_top_i_dig_bb_ctrl20_rx_pwr_t_setf(uint8_t rxpwrt)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000F000)) | ((uint32_t)rxpwrt << 12));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_rx_abs_t_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl20_rx_abs_t_setf(uint8_t rxabst)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)rxabst << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_agc_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bb_top_i_dig_bb_ctrl20_agc_en_setf(uint8_t agcen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)agcen << 7));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_agc_fix_gain_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl20_agc_fix_gain_setf(uint8_t agcfixgain)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)agcfixgain << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl20_agc_gain_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl20_agc_gain_setf(uint8_t agcgain)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL20_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL20_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)agcgain << 0));
}

 /**
 * @brief I_DIG_BB_CTRL21 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        agc_tab_gset3   0b11100
 *  23:16        agc_tab_gset2   0b1011100
 *  15:08        agc_tab_gset1   0b10011100
 *  07:00        agc_tab_gset0   0b10100100
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL21_OFFSET 0x00000054


__INLINE uint32_t bb_top_i_dig_bb_ctrl21_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl21_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL21_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET3_MASK                    ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET3_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET3_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET2_MASK                    ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET2_LSB                     16
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET2_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET1_MASK                    ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET1_LSB                     8
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET1_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET0_MASK                    ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET0_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET0_WIDTH                   ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET3_RST                     0x11100
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET2_RST                     0x1011100
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET1_RST                     0x10011100
#define BB_TOP_I_DIG_BB_CTRL21_AGC_TAB_GSET0_RST                     0x10100100

__INLINE void bb_top_i_dig_bb_ctrl21_pack(uint8_t agctabgset3, uint8_t agctabgset2, uint8_t agctabgset1, uint8_t agctabgset0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL21_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabgset3 << 24) | ((uint32_t)agctabgset2 << 16) | ((uint32_t)agctabgset1 << 8) | ((uint32_t)agctabgset0 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl21_unpack(uint8_t* agctabgset3, uint8_t* agctabgset2, uint8_t* agctabgset1, uint8_t* agctabgset0)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR);

    *agctabgset3 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabgset2 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabgset1 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabgset0 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl21_agc_tab_gset3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl21_agc_tab_gset3_setf(uint8_t agctabgset3)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL21_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabgset3 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl21_agc_tab_gset2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl21_agc_tab_gset2_setf(uint8_t agctabgset2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL21_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabgset2 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl21_agc_tab_gset1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl21_agc_tab_gset1_setf(uint8_t agctabgset1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL21_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabgset1 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl21_agc_tab_gset0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl21_agc_tab_gset0_setf(uint8_t agctabgset0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL21_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL21_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabgset0 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL22 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        agc_tab_gset7   0b10
 *  23:16        agc_tab_gset6   0b100
 *  15:08        agc_tab_gset5   0b1100
 *  07:00        agc_tab_gset4   0b10100
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL22_OFFSET 0x00000058


__INLINE uint32_t bb_top_i_dig_bb_ctrl22_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl22_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL22_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET7_MASK                    ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET7_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET7_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET6_MASK                    ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET6_LSB                     16
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET6_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET5_MASK                    ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET5_LSB                     8
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET5_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET4_MASK                    ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET4_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET4_WIDTH                   ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET7_RST                     0x10
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET6_RST                     0x100
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET5_RST                     0x1100
#define BB_TOP_I_DIG_BB_CTRL22_AGC_TAB_GSET4_RST                     0x10100

__INLINE void bb_top_i_dig_bb_ctrl22_pack(uint8_t agctabgset7, uint8_t agctabgset6, uint8_t agctabgset5, uint8_t agctabgset4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL22_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabgset7 << 24) | ((uint32_t)agctabgset6 << 16) | ((uint32_t)agctabgset5 << 8) | ((uint32_t)agctabgset4 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl22_unpack(uint8_t* agctabgset7, uint8_t* agctabgset6, uint8_t* agctabgset5, uint8_t* agctabgset4)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR);

    *agctabgset7 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabgset6 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabgset5 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabgset4 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl22_agc_tab_gset7_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl22_agc_tab_gset7_setf(uint8_t agctabgset7)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL22_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabgset7 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl22_agc_tab_gset6_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl22_agc_tab_gset6_setf(uint8_t agctabgset6)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL22_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabgset6 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl22_agc_tab_gset5_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl22_agc_tab_gset5_setf(uint8_t agctabgset5)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL22_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabgset5 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl22_agc_tab_gset4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl22_agc_tab_gset4_setf(uint8_t agctabgset4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL22_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL22_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabgset4 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL23 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24         agc_tab_dto2   0b78
 *  23:16         agc_tab_dto1   0b84
 *  15:08         agc_tab_dto0   0b90
 *  07:00        agc_tab_gset8   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL23_OFFSET 0x0000005C


__INLINE uint32_t bb_top_i_dig_bb_ctrl23_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl23_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL23_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO2_MASK                     ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO2_LSB                      24
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO2_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO1_MASK                     ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO1_LSB                      16
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO1_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO0_MASK                     ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO0_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO0_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_GSET8_MASK                    ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_GSET8_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_GSET8_WIDTH                   ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO2_RST                      0x78
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO1_RST                      0x84
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_DTO0_RST                      0x90
#define BB_TOP_I_DIG_BB_CTRL23_AGC_TAB_GSET8_RST                     0x0

__INLINE void bb_top_i_dig_bb_ctrl23_pack(uint8_t agctabdto2, uint8_t agctabdto1, uint8_t agctabdto0, uint8_t agctabgset8)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL23_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabdto2 << 24) | ((uint32_t)agctabdto1 << 16) | ((uint32_t)agctabdto0 << 8) | ((uint32_t)agctabgset8 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl23_unpack(uint8_t* agctabdto2, uint8_t* agctabdto1, uint8_t* agctabdto0, uint8_t* agctabgset8)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR);

    *agctabdto2 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabdto1 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabdto0 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabgset8 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl23_agc_tab_dto2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl23_agc_tab_dto2_setf(uint8_t agctabdto2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL23_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabdto2 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl23_agc_tab_dto1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl23_agc_tab_dto1_setf(uint8_t agctabdto1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL23_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabdto1 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl23_agc_tab_dto0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl23_agc_tab_dto0_setf(uint8_t agctabdto0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL23_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabdto0 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl23_agc_tab_gset8_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl23_agc_tab_gset8_setf(uint8_t agctabgset8)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL23_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL23_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabgset8 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL24 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24         agc_tab_dto6   0b54
 *  23:16         agc_tab_dto5   0b60
 *  15:08         agc_tab_dto4   0b66
 *  07:00         agc_tab_dto3   0b72
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL24_OFFSET 0x00000060


__INLINE uint32_t bb_top_i_dig_bb_ctrl24_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl24_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL24_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO6_MASK                     ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO6_LSB                      24
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO6_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO5_MASK                     ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO5_LSB                      16
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO5_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO4_MASK                     ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO4_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO4_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO3_MASK                     ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO3_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO3_WIDTH                    ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO6_RST                      0x54
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO5_RST                      0x60
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO4_RST                      0x66
#define BB_TOP_I_DIG_BB_CTRL24_AGC_TAB_DTO3_RST                      0x72

__INLINE void bb_top_i_dig_bb_ctrl24_pack(uint8_t agctabdto6, uint8_t agctabdto5, uint8_t agctabdto4, uint8_t agctabdto3)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL24_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabdto6 << 24) | ((uint32_t)agctabdto5 << 16) | ((uint32_t)agctabdto4 << 8) | ((uint32_t)agctabdto3 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl24_unpack(uint8_t* agctabdto6, uint8_t* agctabdto5, uint8_t* agctabdto4, uint8_t* agctabdto3)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR);

    *agctabdto6 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabdto5 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabdto4 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabdto3 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl24_agc_tab_dto6_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl24_agc_tab_dto6_setf(uint8_t agctabdto6)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL24_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabdto6 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl24_agc_tab_dto5_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl24_agc_tab_dto5_setf(uint8_t agctabdto5)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL24_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabdto5 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl24_agc_tab_dto4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl24_agc_tab_dto4_setf(uint8_t agctabdto4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL24_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabdto4 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl24_agc_tab_dto3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl24_agc_tab_dto3_setf(uint8_t agctabdto3)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL24_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL24_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabdto3 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL25 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24         agc_tab_ato1   0b93
 *  23:16         agc_tab_ato0   0b127
 *  15:08         agc_tab_dto8   0b0
 *  07:00         agc_tab_dto7   0b48
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL25_OFFSET 0x00000064


__INLINE uint32_t bb_top_i_dig_bb_ctrl25_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl25_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL25_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO1_MASK                     ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO1_LSB                      24
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO1_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO0_MASK                     ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO0_LSB                      16
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO0_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO8_MASK                     ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO8_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO8_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO7_MASK                     ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO7_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO7_WIDTH                    ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO1_RST                      0x93
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_ATO0_RST                      0x127
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO8_RST                      0x0
#define BB_TOP_I_DIG_BB_CTRL25_AGC_TAB_DTO7_RST                      0x48

__INLINE void bb_top_i_dig_bb_ctrl25_pack(uint8_t agctabato1, uint8_t agctabato0, uint8_t agctabdto8, uint8_t agctabdto7)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL25_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabato1 << 24) | ((uint32_t)agctabato0 << 16) | ((uint32_t)agctabdto8 << 8) | ((uint32_t)agctabdto7 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl25_unpack(uint8_t* agctabato1, uint8_t* agctabato0, uint8_t* agctabdto8, uint8_t* agctabdto7)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR);

    *agctabato1 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabato0 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabdto8 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabdto7 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl25_agc_tab_ato1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl25_agc_tab_ato1_setf(uint8_t agctabato1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL25_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabato1 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl25_agc_tab_ato0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl25_agc_tab_ato0_setf(uint8_t agctabato0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL25_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabato0 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl25_agc_tab_dto8_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl25_agc_tab_dto8_setf(uint8_t agctabdto8)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL25_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabdto8 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl25_agc_tab_dto7_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl25_agc_tab_dto7_setf(uint8_t agctabdto7)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL25_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL25_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabdto7 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL26 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24         agc_tab_ato5   0b69
 *  23:16         agc_tab_ato4   0b75
 *  15:08         agc_tab_ato3   0b81
 *  07:00         agc_tab_ato2   0b87
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL26_OFFSET 0x00000068


__INLINE uint32_t bb_top_i_dig_bb_ctrl26_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl26_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL26_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO5_MASK                     ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO5_LSB                      24
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO5_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO4_MASK                     ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO4_LSB                      16
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO4_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO3_MASK                     ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO3_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO3_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO2_MASK                     ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO2_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO2_WIDTH                    ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO5_RST                      0x69
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO4_RST                      0x75
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO3_RST                      0x81
#define BB_TOP_I_DIG_BB_CTRL26_AGC_TAB_ATO2_RST                      0x87

__INLINE void bb_top_i_dig_bb_ctrl26_pack(uint8_t agctabato5, uint8_t agctabato4, uint8_t agctabato3, uint8_t agctabato2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL26_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabato5 << 24) | ((uint32_t)agctabato4 << 16) | ((uint32_t)agctabato3 << 8) | ((uint32_t)agctabato2 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl26_unpack(uint8_t* agctabato5, uint8_t* agctabato4, uint8_t* agctabato3, uint8_t* agctabato2)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR);

    *agctabato5 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabato4 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabato3 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabato2 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl26_agc_tab_ato5_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl26_agc_tab_ato5_setf(uint8_t agctabato5)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL26_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabato5 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl26_agc_tab_ato4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl26_agc_tab_ato4_setf(uint8_t agctabato4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL26_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabato4 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl26_agc_tab_ato3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl26_agc_tab_ato3_setf(uint8_t agctabato3)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL26_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabato3 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl26_agc_tab_ato2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl26_agc_tab_ato2_setf(uint8_t agctabato2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL26_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL26_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabato2 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL27 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        agc_tab_gain0   0b86
 *  23:16         agc_tab_ato8   0b51
 *  15:08         agc_tab_ato7   0b57
 *  07:00         agc_tab_ato6   0b63
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL27_OFFSET 0x0000006C


__INLINE uint32_t bb_top_i_dig_bb_ctrl27_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl27_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL27_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_GAIN0_MASK                    ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_GAIN0_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_GAIN0_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO8_MASK                     ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO8_LSB                      16
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO8_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO7_MASK                     ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO7_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO7_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO6_MASK                     ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO6_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO6_WIDTH                    ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_GAIN0_RST                     0x86
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO8_RST                      0x51
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO7_RST                      0x57
#define BB_TOP_I_DIG_BB_CTRL27_AGC_TAB_ATO6_RST                      0x63

__INLINE void bb_top_i_dig_bb_ctrl27_pack(uint8_t agctabgain0, uint8_t agctabato8, uint8_t agctabato7, uint8_t agctabato6)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL27_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabgain0 << 24) | ((uint32_t)agctabato8 << 16) | ((uint32_t)agctabato7 << 8) | ((uint32_t)agctabato6 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl27_unpack(uint8_t* agctabgain0, uint8_t* agctabato8, uint8_t* agctabato7, uint8_t* agctabato6)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR);

    *agctabgain0 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabato8 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabato7 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabato6 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl27_agc_tab_gain0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl27_agc_tab_gain0_setf(uint8_t agctabgain0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL27_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabgain0 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl27_agc_tab_ato8_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl27_agc_tab_ato8_setf(uint8_t agctabato8)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL27_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabato8 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl27_agc_tab_ato7_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl27_agc_tab_ato7_setf(uint8_t agctabato7)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL27_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabato7 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl27_agc_tab_ato6_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl27_agc_tab_ato6_setf(uint8_t agctabato6)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL27_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL27_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabato6 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL28 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        agc_tab_gain4   0b62
 *  23:16        agc_tab_gain3   0b68
 *  15:08        agc_tab_gain2   0b74
 *  07:00        agc_tab_gain1   0b80
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL28_OFFSET 0x00000070


__INLINE uint32_t bb_top_i_dig_bb_ctrl28_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl28_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL28_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN4_MASK                    ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN4_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN4_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN3_MASK                    ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN3_LSB                     16
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN3_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN2_MASK                    ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN2_LSB                     8
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN2_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN1_MASK                    ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN1_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN1_WIDTH                   ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN4_RST                     0x62
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN3_RST                     0x68
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN2_RST                     0x74
#define BB_TOP_I_DIG_BB_CTRL28_AGC_TAB_GAIN1_RST                     0x80

__INLINE void bb_top_i_dig_bb_ctrl28_pack(uint8_t agctabgain4, uint8_t agctabgain3, uint8_t agctabgain2, uint8_t agctabgain1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL28_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabgain4 << 24) | ((uint32_t)agctabgain3 << 16) | ((uint32_t)agctabgain2 << 8) | ((uint32_t)agctabgain1 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl28_unpack(uint8_t* agctabgain4, uint8_t* agctabgain3, uint8_t* agctabgain2, uint8_t* agctabgain1)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR);

    *agctabgain4 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabgain3 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabgain2 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabgain1 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl28_agc_tab_gain4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl28_agc_tab_gain4_setf(uint8_t agctabgain4)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL28_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabgain4 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl28_agc_tab_gain3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl28_agc_tab_gain3_setf(uint8_t agctabgain3)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL28_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabgain3 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl28_agc_tab_gain2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl28_agc_tab_gain2_setf(uint8_t agctabgain2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL28_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabgain2 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl28_agc_tab_gain1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl28_agc_tab_gain1_setf(uint8_t agctabgain1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL28_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL28_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabgain1 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL29 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24        agc_tab_gain8   0b38
 *  23:16        agc_tab_gain7   0b44
 *  15:08        agc_tab_gain6   0b50
 *  07:00        agc_tab_gain5   0b56
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL29_OFFSET 0x00000074


__INLINE uint32_t bb_top_i_dig_bb_ctrl29_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl29_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL29_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN8_MASK                    ((uint32_t)0xFF000000)
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN8_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN8_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN7_MASK                    ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN7_LSB                     16
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN7_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN6_MASK                    ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN6_LSB                     8
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN6_WIDTH                   ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN5_MASK                    ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN5_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN5_WIDTH                   ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN8_RST                     0x38
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN7_RST                     0x44
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN6_RST                     0x50
#define BB_TOP_I_DIG_BB_CTRL29_AGC_TAB_GAIN5_RST                     0x56

__INLINE void bb_top_i_dig_bb_ctrl29_pack(uint8_t agctabgain8, uint8_t agctabgain7, uint8_t agctabgain6, uint8_t agctabgain5)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL29_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agctabgain8 << 24) | ((uint32_t)agctabgain7 << 16) | ((uint32_t)agctabgain6 << 8) | ((uint32_t)agctabgain5 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl29_unpack(uint8_t* agctabgain8, uint8_t* agctabgain7, uint8_t* agctabgain6, uint8_t* agctabgain5)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR);

    *agctabgain8 = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *agctabgain7 = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *agctabgain6 = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *agctabgain5 = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl29_agc_tab_gain8_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl29_agc_tab_gain8_setf(uint8_t agctabgain8)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL29_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFF000000)) | ((uint32_t)agctabgain8 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl29_agc_tab_gain7_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl29_agc_tab_gain7_setf(uint8_t agctabgain7)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL29_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)agctabgain7 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl29_agc_tab_gain6_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl29_agc_tab_gain6_setf(uint8_t agctabgain6)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL29_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)agctabgain6 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl29_agc_tab_gain5_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl29_agc_tab_gain5_setf(uint8_t agctabgain5)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL29_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL29_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agctabgain5 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL30 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:22         dcoc_flt_phy   0b845
 *  21:18        dcoc_amp_step   0b8
 *  17:12         dcoc_amp_ini   0b16
 *  11:08       dcoc_resdc_thd   0b5
 *  07:02         dcoc_phy_thd   0b25
 *     01         dcoc_cal_mod   0
 *     00              dcoc_en   0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL30_OFFSET 0x00000078


__INLINE uint32_t bb_top_i_dig_bb_ctrl30_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl30_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_FLT_PHY_MASK                     ((uint32_t)0xFFC00000)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_FLT_PHY_LSB                      22
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_FLT_PHY_WIDTH                    ((uint32_t)0x0000000A)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_STEP_MASK                    ((uint32_t)0x003C0000)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_STEP_LSB                     18
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_STEP_WIDTH                   ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_INI_MASK                     ((uint32_t)0x0003F000)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_INI_LSB                      12
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_INI_WIDTH                    ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_RESDC_THD_MASK                   ((uint32_t)0x00000F00)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_RESDC_THD_LSB                    8
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_RESDC_THD_WIDTH                  ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_PHY_THD_MASK                     ((uint32_t)0x000000FC)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_PHY_THD_LSB                      2
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_PHY_THD_WIDTH                    ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_CAL_MOD_BIT                      ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_CAL_MOD_POS                      1
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_EN_BIT                           ((uint32_t)0x00000001)
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_EN_POS                           0

#define BB_TOP_I_DIG_BB_CTRL30_DCOC_FLT_PHY_RST                      0x845
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_STEP_RST                     0x8
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_AMP_INI_RST                      0x16
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_RESDC_THD_RST                    0x5
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_PHY_THD_RST                      0x25
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_CAL_MOD_RST                      0x0
#define BB_TOP_I_DIG_BB_CTRL30_DCOC_EN_RST                           0x0

__INLINE void bb_top_i_dig_bb_ctrl30_pack(uint16_t dcocfltphy, uint8_t dcocampstep, uint8_t dcocampini, uint8_t dcocresdcthd, uint8_t dcocphythd, uint8_t dcoccalmod, uint8_t dcocen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)dcocfltphy << 22) | ((uint32_t)dcocampstep << 18) | ((uint32_t)dcocampini << 12) | ((uint32_t)dcocresdcthd << 8) | ((uint32_t)dcocphythd << 2) | ((uint32_t)dcoccalmod << 1) | ((uint32_t)dcocen << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl30_unpack(uint8_t* dcocfltphy, uint8_t* dcocampstep, uint8_t* dcocampini, uint8_t* dcocresdcthd, uint8_t* dcocphythd, uint8_t* dcoccalmod, uint8_t* dcocen)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);

    *dcocfltphy = (localVal & ((uint32_t)0xFFC00000)) >> 22;
    *dcocampstep = (localVal & ((uint32_t)0x003C0000)) >> 18;
    *dcocampini = (localVal & ((uint32_t)0x0003F000)) >> 12;
    *dcocresdcthd = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *dcocphythd = (localVal & ((uint32_t)0x000000FC)) >> 2;
    *dcoccalmod = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dcocen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint16_t bb_top_i_dig_bb_ctrl30_dcoc_flt_phy_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFC00000)) >> 22);
}

__INLINE void bb_top_i_dig_bb_ctrl30_dcoc_flt_phy_setf(uint16_t dcocfltphy)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0xFFC00000)) | ((uint32_t)dcocfltphy << 22));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl30_dcoc_amp_step_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x003C0000)) >> 18);
}

__INLINE void bb_top_i_dig_bb_ctrl30_dcoc_amp_step_setf(uint8_t dcocampstep)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x003C0000)) | ((uint32_t)dcocampstep << 18));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl30_dcoc_amp_ini_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0003F000)) >> 12);
}

__INLINE void bb_top_i_dig_bb_ctrl30_dcoc_amp_ini_setf(uint8_t dcocampini)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0003F000)) | ((uint32_t)dcocampini << 12));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl30_dcoc_resdc_thd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl30_dcoc_resdc_thd_setf(uint8_t dcocresdcthd)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)dcocresdcthd << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl30_dcoc_phy_thd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FC)) >> 2);
}

__INLINE void bb_top_i_dig_bb_ctrl30_dcoc_phy_thd_setf(uint8_t dcocphythd)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FC)) | ((uint32_t)dcocphythd << 2));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl30_dcoc_cal_mod_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bb_top_i_dig_bb_ctrl30_dcoc_cal_mod_setf(uint8_t dcoccalmod)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)dcoccalmod << 1));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl30_dcoc_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl30_dcoc_en_setf(uint8_t dcocen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL30_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL30_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)dcocen << 0));
}

 /**
 * @brief I_DIG_BB_CTRL31 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:20            dcoc_phy2   0b683
 *  19:10            dcoc_phy1   0b341
 *  09:00            dcoc_phy0   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL31_OFFSET 0x0000007C


__INLINE uint32_t bb_top_i_dig_bb_ctrl31_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl31_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL31_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY2_MASK                        ((uint32_t)0x3FF00000)
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY2_LSB                         20
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY2_WIDTH                       ((uint32_t)0x0000000A)
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY1_MASK                        ((uint32_t)0x000FFC00)
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY1_LSB                         10
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY1_WIDTH                       ((uint32_t)0x0000000A)
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY0_MASK                        ((uint32_t)0x000003FF)
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY0_LSB                         0
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY0_WIDTH                       ((uint32_t)0x0000000A)

#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY2_RST                         0x683
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY1_RST                         0x341
#define BB_TOP_I_DIG_BB_CTRL31_DCOC_PHY0_RST                         0x0

__INLINE void bb_top_i_dig_bb_ctrl31_pack(uint16_t dcocphy2, uint16_t dcocphy1, uint16_t dcocphy0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL31_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)dcocphy2 << 20) | ((uint32_t)dcocphy1 << 10) | ((uint32_t)dcocphy0 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl31_unpack(uint8_t* dcocphy2, uint8_t* dcocphy1, uint8_t* dcocphy0)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR);

    *dcocphy2 = (localVal & ((uint32_t)0x3FF00000)) >> 20;
    *dcocphy1 = (localVal & ((uint32_t)0x000FFC00)) >> 10;
    *dcocphy0 = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t bb_top_i_dig_bb_ctrl31_dcoc_phy2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x3FF00000)) >> 20);
}

__INLINE void bb_top_i_dig_bb_ctrl31_dcoc_phy2_setf(uint16_t dcocphy2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL31_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x3FF00000)) | ((uint32_t)dcocphy2 << 20));
}

__INLINE uint16_t bb_top_i_dig_bb_ctrl31_dcoc_phy1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFC00)) >> 10);
}

__INLINE void bb_top_i_dig_bb_ctrl31_dcoc_phy1_setf(uint16_t dcocphy1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL31_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000FFC00)) | ((uint32_t)dcocphy1 << 10));
}

__INLINE uint16_t bb_top_i_dig_bb_ctrl31_dcoc_phy0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl31_dcoc_phy0_setf(uint16_t dcocphy0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL31_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL31_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)dcocphy0 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL32 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     14                ld_en   0
 *  13:12              ld_time   0b10
 *  11:10     len_slot_fr_spir   0b10
 *  09:07       len_ct_fr_spir   0b0
 *  06:04       len_fc_fr_spir   0b10
 *  03:01      len_rst_fr_spir   0b10
 *     00     clk_flag_fr_spir   0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL32_OFFSET 0x00000080


__INLINE uint32_t bb_top_i_dig_bb_ctrl32_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl32_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL32_LD_EN_BIT                             ((uint32_t)0x00004000)
#define BB_TOP_I_DIG_BB_CTRL32_LD_EN_POS                             14
#define BB_TOP_I_DIG_BB_CTRL32_LD_TIME_MASK                          ((uint32_t)0x00003000)
#define BB_TOP_I_DIG_BB_CTRL32_LD_TIME_LSB                           12
#define BB_TOP_I_DIG_BB_CTRL32_LD_TIME_WIDTH                         ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_SLOT_FR_SPIR_MASK                 ((uint32_t)0x00000C00)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_SLOT_FR_SPIR_LSB                  10
#define BB_TOP_I_DIG_BB_CTRL32_LEN_SLOT_FR_SPIR_WIDTH                ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_CT_FR_SPIR_MASK                   ((uint32_t)0x00000380)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_CT_FR_SPIR_LSB                    7
#define BB_TOP_I_DIG_BB_CTRL32_LEN_CT_FR_SPIR_WIDTH                  ((uint32_t)0x00000003)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_FC_FR_SPIR_MASK                   ((uint32_t)0x00000070)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_FC_FR_SPIR_LSB                    4
#define BB_TOP_I_DIG_BB_CTRL32_LEN_FC_FR_SPIR_WIDTH                  ((uint32_t)0x00000003)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_RST_FR_SPIR_MASK                  ((uint32_t)0x0000000E)
#define BB_TOP_I_DIG_BB_CTRL32_LEN_RST_FR_SPIR_LSB                   1
#define BB_TOP_I_DIG_BB_CTRL32_LEN_RST_FR_SPIR_WIDTH                 ((uint32_t)0x00000003)
#define BB_TOP_I_DIG_BB_CTRL32_CLK_FLAG_FR_SPIR_BIT                  ((uint32_t)0x00000001)
#define BB_TOP_I_DIG_BB_CTRL32_CLK_FLAG_FR_SPIR_POS                  0

#define BB_TOP_I_DIG_BB_CTRL32_LD_EN_RST                             0x0
#define BB_TOP_I_DIG_BB_CTRL32_LD_TIME_RST                           0x10
#define BB_TOP_I_DIG_BB_CTRL32_LEN_SLOT_FR_SPIR_RST                  0x10
#define BB_TOP_I_DIG_BB_CTRL32_LEN_CT_FR_SPIR_RST                    0x0
#define BB_TOP_I_DIG_BB_CTRL32_LEN_FC_FR_SPIR_RST                    0x10
#define BB_TOP_I_DIG_BB_CTRL32_LEN_RST_FR_SPIR_RST                   0x10
#define BB_TOP_I_DIG_BB_CTRL32_CLK_FLAG_FR_SPIR_RST                  0x0

__INLINE void bb_top_i_dig_bb_ctrl32_pack(uint8_t lden, uint8_t ldtime, uint8_t lenslotfrspir, uint8_t lenctfrspir, uint8_t lenfcfrspir, uint8_t lenrstfrspir, uint8_t clkflagfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)lden << 14) | ((uint32_t)ldtime << 12) | ((uint32_t)lenslotfrspir << 10) | ((uint32_t)lenctfrspir << 7) | ((uint32_t)lenfcfrspir << 4) | ((uint32_t)lenrstfrspir << 1) | ((uint32_t)clkflagfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl32_unpack(uint8_t* lden, uint8_t* ldtime, uint8_t* lenslotfrspir, uint8_t* lenctfrspir, uint8_t* lenfcfrspir, uint8_t* lenrstfrspir, uint8_t* clkflagfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);

    *lden = (localVal & ((uint32_t)0x00004000)) >> 14;
    *ldtime = (localVal & ((uint32_t)0x00003000)) >> 12;
    *lenslotfrspir = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *lenctfrspir = (localVal & ((uint32_t)0x00000380)) >> 7;
    *lenfcfrspir = (localVal & ((uint32_t)0x00000070)) >> 4;
    *lenrstfrspir = (localVal & ((uint32_t)0x0000000E)) >> 1;
    *clkflagfrspir = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl32_ld_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bb_top_i_dig_bb_ctrl32_ld_en_setf(uint8_t lden)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)lden << 14));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl32_ld_time_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void bb_top_i_dig_bb_ctrl32_ld_time_setf(uint8_t ldtime)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)ldtime << 12));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl32_len_slot_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void bb_top_i_dig_bb_ctrl32_len_slot_fr_spir_setf(uint8_t lenslotfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)lenslotfrspir << 10));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl32_len_ct_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000380)) >> 7);
}

__INLINE void bb_top_i_dig_bb_ctrl32_len_ct_fr_spir_setf(uint8_t lenctfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000380)) | ((uint32_t)lenctfrspir << 7));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl32_len_fc_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl32_len_fc_fr_spir_setf(uint8_t lenfcfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)lenfcfrspir << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl32_len_rst_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000E)) >> 1);
}

__INLINE void bb_top_i_dig_bb_ctrl32_len_rst_fr_spir_setf(uint8_t lenrstfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000000E)) | ((uint32_t)lenrstfrspir << 1));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl32_clk_flag_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl32_clk_flag_fr_spir_setf(uint8_t clkflagfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL32_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL32_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)clkflagfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL33 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31   ct_override_fr_spir   0
 *  08:00      ct_word_fr_spir   0b100
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL33_OFFSET 0x00000084


__INLINE uint32_t bb_top_i_dig_bb_ctrl33_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL33_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl33_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL33_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL33_CT_OVERRIDE_FR_SPIR_BIT               ((uint32_t)0x80000000)
#define BB_TOP_I_DIG_BB_CTRL33_CT_OVERRIDE_FR_SPIR_POS               31
#define BB_TOP_I_DIG_BB_CTRL33_CT_WORD_FR_SPIR_MASK                  ((uint32_t)0x000001FF)
#define BB_TOP_I_DIG_BB_CTRL33_CT_WORD_FR_SPIR_LSB                   0
#define BB_TOP_I_DIG_BB_CTRL33_CT_WORD_FR_SPIR_WIDTH                 ((uint32_t)0x00000009)

#define BB_TOP_I_DIG_BB_CTRL33_CT_OVERRIDE_FR_SPIR_RST               0x0
#define BB_TOP_I_DIG_BB_CTRL33_CT_WORD_FR_SPIR_RST                   0x100

__INLINE void bb_top_i_dig_bb_ctrl33_pack(uint8_t ctoverridefrspir, uint16_t ctwordfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL33_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)ctoverridefrspir << 31) | ((uint32_t)ctwordfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl33_unpack(uint8_t* ctoverridefrspir, uint8_t* ctwordfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL33_OFFSET + BB_TOP_BASE_ADDR);

    *ctoverridefrspir = (localVal & ((uint32_t)0x80000000)) >> 31;
    *ctwordfrspir = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl33_ct_override_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL33_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bb_top_i_dig_bb_ctrl33_ct_override_fr_spir_setf(uint8_t ctoverridefrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL33_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL33_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)ctoverridefrspir << 31));
}

__INLINE uint16_t bb_top_i_dig_bb_ctrl33_ct_word_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL33_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl33_ct_word_fr_spir_setf(uint16_t ctwordfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL33_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL33_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)ctwordfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL34 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31    targ_cnt_override   0
 *  19:00     targ_cnt_fr_spir   0b2440
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL34_OFFSET 0x00000088


__INLINE uint32_t bb_top_i_dig_bb_ctrl34_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL34_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl34_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL34_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL34_TARG_CNT_OVERRIDE_BIT                 ((uint32_t)0x80000000)
#define BB_TOP_I_DIG_BB_CTRL34_TARG_CNT_OVERRIDE_POS                 31
#define BB_TOP_I_DIG_BB_CTRL34_TARG_CNT_FR_SPIR_MASK                 ((uint32_t)0x000FFFFF)
#define BB_TOP_I_DIG_BB_CTRL34_TARG_CNT_FR_SPIR_LSB                  0
#define BB_TOP_I_DIG_BB_CTRL34_TARG_CNT_FR_SPIR_WIDTH                ((uint32_t)0x00000014)

#define BB_TOP_I_DIG_BB_CTRL34_TARG_CNT_OVERRIDE_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL34_TARG_CNT_FR_SPIR_RST                  0x2440

__INLINE void bb_top_i_dig_bb_ctrl34_pack(uint8_t targcntoverride, uint32_t targcntfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL34_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)targcntoverride << 31) | ((uint32_t)targcntfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl34_unpack(uint8_t* targcntoverride, uint8_t* targcntfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL34_OFFSET + BB_TOP_BASE_ADDR);

    *targcntoverride = (localVal & ((uint32_t)0x80000000)) >> 31;
    *targcntfrspir = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl34_targ_cnt_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL34_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bb_top_i_dig_bb_ctrl34_targ_cnt_override_setf(uint8_t targcntoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL34_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL34_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)targcntoverride << 31));
}

__INLINE uint32_t bb_top_i_dig_bb_ctrl34_targ_cnt_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL34_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl34_targ_cnt_fr_spir_setf(uint32_t targcntfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL34_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL34_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000FFFFF)) | ((uint32_t)targcntfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL35 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     24        cnt_manual_en   0
 *     23    clk_edge_polarity   0
 *     22    tp_cal_en_fr_spir   0
 *     21    delta_ftx_fr_spir   0
 *     20      mod_sel_fr_spir   1
 *  19:18     leng_tm1_fr_spir   0b0
 *  17:16     leng_tm0_fr_spir   0b0
 *  14:08      kdac_fi_fr_spir   0b40
 *  06:00      kdac_co_fr_spir   0b40
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL35_OFFSET 0x0000008C


__INLINE uint32_t bb_top_i_dig_bb_ctrl35_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl35_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL35_CNT_MANUAL_EN_BIT                     ((uint32_t)0x01000000)
#define BB_TOP_I_DIG_BB_CTRL35_CNT_MANUAL_EN_POS                     24
#define BB_TOP_I_DIG_BB_CTRL35_CLK_EDGE_POLARITY_BIT                 ((uint32_t)0x00800000)
#define BB_TOP_I_DIG_BB_CTRL35_CLK_EDGE_POLARITY_POS                 23
#define BB_TOP_I_DIG_BB_CTRL35_TP_CAL_EN_FR_SPIR_BIT                 ((uint32_t)0x00400000)
#define BB_TOP_I_DIG_BB_CTRL35_TP_CAL_EN_FR_SPIR_POS                 22
#define BB_TOP_I_DIG_BB_CTRL35_DELTA_FTX_FR_SPIR_BIT                 ((uint32_t)0x00200000)
#define BB_TOP_I_DIG_BB_CTRL35_DELTA_FTX_FR_SPIR_POS                 21
#define BB_TOP_I_DIG_BB_CTRL35_MOD_SEL_FR_SPIR_BIT                   ((uint32_t)0x00100000)
#define BB_TOP_I_DIG_BB_CTRL35_MOD_SEL_FR_SPIR_POS                   20
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM1_FR_SPIR_MASK                 ((uint32_t)0x000C0000)
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM1_FR_SPIR_LSB                  18
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM1_FR_SPIR_WIDTH                ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM0_FR_SPIR_MASK                 ((uint32_t)0x00030000)
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM0_FR_SPIR_LSB                  16
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM0_FR_SPIR_WIDTH                ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_FI_FR_SPIR_MASK                  ((uint32_t)0x00007F00)
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_FI_FR_SPIR_LSB                   8
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_FI_FR_SPIR_WIDTH                 ((uint32_t)0x00000007)
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_CO_FR_SPIR_MASK                  ((uint32_t)0x0000007F)
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_CO_FR_SPIR_LSB                   0
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_CO_FR_SPIR_WIDTH                 ((uint32_t)0x00000007)

#define BB_TOP_I_DIG_BB_CTRL35_CNT_MANUAL_EN_RST                     0x0
#define BB_TOP_I_DIG_BB_CTRL35_CLK_EDGE_POLARITY_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL35_TP_CAL_EN_FR_SPIR_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL35_DELTA_FTX_FR_SPIR_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL35_MOD_SEL_FR_SPIR_RST                   0x1
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM1_FR_SPIR_RST                  0x0
#define BB_TOP_I_DIG_BB_CTRL35_LENG_TM0_FR_SPIR_RST                  0x0
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_FI_FR_SPIR_RST                   0x40
#define BB_TOP_I_DIG_BB_CTRL35_KDAC_CO_FR_SPIR_RST                   0x40

__INLINE void bb_top_i_dig_bb_ctrl35_pack(uint8_t cntmanualen, uint8_t clkedgepolarity, uint8_t tpcalenfrspir, uint8_t deltaftxfrspir, uint8_t modselfrspir, uint8_t lengtm1frspir, uint8_t lengtm0frspir, uint8_t kdacfifrspir, uint8_t kdaccofrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)cntmanualen << 24) | ((uint32_t)clkedgepolarity << 23) | ((uint32_t)tpcalenfrspir << 22) | ((uint32_t)deltaftxfrspir << 21) | ((uint32_t)modselfrspir << 20) | ((uint32_t)lengtm1frspir << 18) | ((uint32_t)lengtm0frspir << 16) | ((uint32_t)kdacfifrspir << 8) | ((uint32_t)kdaccofrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl35_unpack(uint8_t* cntmanualen, uint8_t* clkedgepolarity, uint8_t* tpcalenfrspir, uint8_t* deltaftxfrspir, uint8_t* modselfrspir, uint8_t* lengtm1frspir, uint8_t* lengtm0frspir, uint8_t* kdacfifrspir, uint8_t* kdaccofrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);

    *cntmanualen = (localVal & ((uint32_t)0x01000000)) >> 24;
    *clkedgepolarity = (localVal & ((uint32_t)0x00800000)) >> 23;
    *tpcalenfrspir = (localVal & ((uint32_t)0x00400000)) >> 22;
    *deltaftxfrspir = (localVal & ((uint32_t)0x00200000)) >> 21;
    *modselfrspir = (localVal & ((uint32_t)0x00100000)) >> 20;
    *lengtm1frspir = (localVal & ((uint32_t)0x000C0000)) >> 18;
    *lengtm0frspir = (localVal & ((uint32_t)0x00030000)) >> 16;
    *kdacfifrspir = (localVal & ((uint32_t)0x00007F00)) >> 8;
    *kdaccofrspir = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_cnt_manual_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl35_cnt_manual_en_setf(uint8_t cntmanualen)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)cntmanualen << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_clk_edge_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bb_top_i_dig_bb_ctrl35_clk_edge_polarity_setf(uint8_t clkedgepolarity)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)clkedgepolarity << 23));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_tp_cal_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bb_top_i_dig_bb_ctrl35_tp_cal_en_fr_spir_setf(uint8_t tpcalenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)tpcalenfrspir << 22));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_delta_ftx_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00200000)) >> 21);
}

__INLINE void bb_top_i_dig_bb_ctrl35_delta_ftx_fr_spir_setf(uint8_t deltaftxfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00200000)) | ((uint32_t)deltaftxfrspir << 21));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_mod_sel_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00100000)) >> 20);
}

__INLINE void bb_top_i_dig_bb_ctrl35_mod_sel_fr_spir_setf(uint8_t modselfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00100000)) | ((uint32_t)modselfrspir << 20));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_leng_tm1_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000C0000)) >> 18);
}

__INLINE void bb_top_i_dig_bb_ctrl35_leng_tm1_fr_spir_setf(uint8_t lengtm1frspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000C0000)) | ((uint32_t)lengtm1frspir << 18));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_leng_tm0_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl35_leng_tm0_fr_spir_setf(uint8_t lengtm0frspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)lengtm0frspir << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_kdac_fi_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00007F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl35_kdac_fi_fr_spir_setf(uint8_t kdacfifrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00007F00)) | ((uint32_t)kdacfifrspir << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl35_kdac_co_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl35_kdac_co_fr_spir_setf(uint8_t kdaccofrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL35_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL35_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)kdaccofrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL36 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00       reg_dc_fr_spir   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL36_OFFSET 0x00000090


__INLINE uint32_t bb_top_i_dig_bb_ctrl36_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL36_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl36_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL36_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL36_REG_DC_FR_SPIR_MASK                   ((uint32_t)0x000FFFFF)
#define BB_TOP_I_DIG_BB_CTRL36_REG_DC_FR_SPIR_LSB                    0
#define BB_TOP_I_DIG_BB_CTRL36_REG_DC_FR_SPIR_WIDTH                  ((uint32_t)0x00000014)

#define BB_TOP_I_DIG_BB_CTRL36_REG_DC_FR_SPIR_RST                    0x0

__INLINE void bb_top_i_dig_bb_ctrl36_pack(uint32_t regdcfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL36_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)regdcfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl36_unpack(uint8_t* regdcfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL36_OFFSET + BB_TOP_BASE_ADDR);

    *regdcfrspir = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t bb_top_i_dig_bb_ctrl36_reg_dc_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL36_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl36_reg_dc_fr_spir_setf(uint32_t regdcfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL36_OFFSET+ BB_TOP_BASE_ADDR, (uint32_t)regdcfrspir << 0);
}

 /**
 * @brief I_DIG_BB_CTRL37 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16      tp_cal_override   0
 *  12:08         rcal_fr_spir   0b10
 *  06:00         kcal_fr_spir   0b40
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL37_OFFSET 0x00000094


__INLINE uint32_t bb_top_i_dig_bb_ctrl37_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl37_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL37_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL37_TP_CAL_OVERRIDE_BIT                   ((uint32_t)0x00010000)
#define BB_TOP_I_DIG_BB_CTRL37_TP_CAL_OVERRIDE_POS                   16
#define BB_TOP_I_DIG_BB_CTRL37_RCAL_FR_SPIR_MASK                     ((uint32_t)0x00001F00)
#define BB_TOP_I_DIG_BB_CTRL37_RCAL_FR_SPIR_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL37_RCAL_FR_SPIR_WIDTH                    ((uint32_t)0x00000005)
#define BB_TOP_I_DIG_BB_CTRL37_KCAL_FR_SPIR_MASK                     ((uint32_t)0x0000007F)
#define BB_TOP_I_DIG_BB_CTRL37_KCAL_FR_SPIR_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL37_KCAL_FR_SPIR_WIDTH                    ((uint32_t)0x00000007)

#define BB_TOP_I_DIG_BB_CTRL37_TP_CAL_OVERRIDE_RST                   0x0
#define BB_TOP_I_DIG_BB_CTRL37_RCAL_FR_SPIR_RST                      0x10
#define BB_TOP_I_DIG_BB_CTRL37_KCAL_FR_SPIR_RST                      0x40

__INLINE void bb_top_i_dig_bb_ctrl37_pack(uint8_t tpcaloverride, uint8_t rcalfrspir, uint8_t kcalfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL37_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)tpcaloverride << 16) | ((uint32_t)rcalfrspir << 8) | ((uint32_t)kcalfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl37_unpack(uint8_t* tpcaloverride, uint8_t* rcalfrspir, uint8_t* kcalfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR);

    *tpcaloverride = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rcalfrspir = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *kcalfrspir = (localVal & ((uint32_t)0x0000007F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl37_tp_cal_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl37_tp_cal_override_setf(uint8_t tpcaloverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL37_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)tpcaloverride << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl37_rcal_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl37_rcal_fr_spir_setf(uint8_t rcalfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL37_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)rcalfrspir << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl37_kcal_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000007F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl37_kcal_fr_spir_setf(uint8_t kcalfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL37_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL37_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000007F)) | ((uint32_t)kcalfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL38 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16   intg_frac_override   0
 *  08:00         intg_fr_spir   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL38_OFFSET 0x00000098


__INLINE uint32_t bb_top_i_dig_bb_ctrl38_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL38_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl38_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL38_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL38_INTG_FRAC_OVERRIDE_BIT                ((uint32_t)0x00010000)
#define BB_TOP_I_DIG_BB_CTRL38_INTG_FRAC_OVERRIDE_POS                16
#define BB_TOP_I_DIG_BB_CTRL38_INTG_FR_SPIR_MASK                     ((uint32_t)0x000001FF)
#define BB_TOP_I_DIG_BB_CTRL38_INTG_FR_SPIR_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL38_INTG_FR_SPIR_WIDTH                    ((uint32_t)0x00000009)

#define BB_TOP_I_DIG_BB_CTRL38_INTG_FRAC_OVERRIDE_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL38_INTG_FR_SPIR_RST                      0x0

__INLINE void bb_top_i_dig_bb_ctrl38_pack(uint8_t intgfracoverride, uint16_t intgfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL38_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)intgfracoverride << 16) | ((uint32_t)intgfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl38_unpack(uint8_t* intgfracoverride, uint8_t* intgfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL38_OFFSET + BB_TOP_BASE_ADDR);

    *intgfracoverride = (localVal & ((uint32_t)0x00010000)) >> 16;
    *intgfrspir = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl38_intg_frac_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL38_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl38_intg_frac_override_setf(uint8_t intgfracoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL38_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL38_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)intgfracoverride << 16));
}

__INLINE uint16_t bb_top_i_dig_bb_ctrl38_intg_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL38_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl38_intg_fr_spir_setf(uint16_t intgfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL38_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL38_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000001FF)) | ((uint32_t)intgfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL39 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     24         dsm_override   0
 *  23:00         frac_fr_spir   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL39_OFFSET 0x0000009C


__INLINE uint32_t bb_top_i_dig_bb_ctrl39_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL39_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl39_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL39_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL39_DSM_OVERRIDE_BIT                      ((uint32_t)0x01000000)
#define BB_TOP_I_DIG_BB_CTRL39_DSM_OVERRIDE_POS                      24
#define BB_TOP_I_DIG_BB_CTRL39_FRAC_FR_SPIR_MASK                     ((uint32_t)0x00FFFFFF)
#define BB_TOP_I_DIG_BB_CTRL39_FRAC_FR_SPIR_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL39_FRAC_FR_SPIR_WIDTH                    ((uint32_t)0x00000018)

#define BB_TOP_I_DIG_BB_CTRL39_DSM_OVERRIDE_RST                      0x0
#define BB_TOP_I_DIG_BB_CTRL39_FRAC_FR_SPIR_RST                      0x0

__INLINE void bb_top_i_dig_bb_ctrl39_pack(uint8_t dsmoverride, uint32_t fracfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL39_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)dsmoverride << 24) | ((uint32_t)fracfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl39_unpack(uint8_t* dsmoverride, uint8_t* fracfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL39_OFFSET + BB_TOP_BASE_ADDR);

    *dsmoverride = (localVal & ((uint32_t)0x01000000)) >> 24;
    *fracfrspir = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl39_dsm_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL39_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl39_dsm_override_setf(uint8_t dsmoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL39_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL39_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)dsmoverride << 24));
}

__INLINE uint32_t bb_top_i_dig_bb_ctrl39_frac_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL39_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl39_frac_fr_spir_setf(uint32_t fracfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL39_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL39_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FFFFFF)) | ((uint32_t)fracfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL40 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04    pll_test_override   0
 *     03    rx_bb_en_override   0
 *     02    tx_bb_en_override   0
 *     01       rx_en_override   0
 *     00    tx_pa_en_override   0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL40_OFFSET 0x000000A0


__INLINE uint32_t bb_top_i_dig_bb_ctrl40_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl40_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL40_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL40_PLL_TEST_OVERRIDE_BIT                 ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL40_PLL_TEST_OVERRIDE_POS                 4
#define BB_TOP_I_DIG_BB_CTRL40_RX_BB_EN_OVERRIDE_BIT                 ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL40_RX_BB_EN_OVERRIDE_POS                 3
#define BB_TOP_I_DIG_BB_CTRL40_TX_BB_EN_OVERRIDE_BIT                 ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL40_TX_BB_EN_OVERRIDE_POS                 2
#define BB_TOP_I_DIG_BB_CTRL40_RX_EN_OVERRIDE_BIT                    ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL40_RX_EN_OVERRIDE_POS                    1
#define BB_TOP_I_DIG_BB_CTRL40_TX_PA_EN_OVERRIDE_BIT                 ((uint32_t)0x00000001)
#define BB_TOP_I_DIG_BB_CTRL40_TX_PA_EN_OVERRIDE_POS                 0

#define BB_TOP_I_DIG_BB_CTRL40_PLL_TEST_OVERRIDE_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL40_RX_BB_EN_OVERRIDE_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL40_TX_BB_EN_OVERRIDE_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL40_RX_EN_OVERRIDE_RST                    0x0
#define BB_TOP_I_DIG_BB_CTRL40_TX_PA_EN_OVERRIDE_RST                 0x0

__INLINE void bb_top_i_dig_bb_ctrl40_pack(uint8_t plltestoverride, uint8_t rxbbenoverride, uint8_t txbbenoverride, uint8_t rxenoverride, uint8_t txpaenoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL40_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)plltestoverride << 4) | ((uint32_t)rxbbenoverride << 3) | ((uint32_t)txbbenoverride << 2) | ((uint32_t)rxenoverride << 1) | ((uint32_t)txpaenoverride << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl40_unpack(uint8_t* plltestoverride, uint8_t* rxbbenoverride, uint8_t* txbbenoverride, uint8_t* rxenoverride, uint8_t* txpaenoverride)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR);

    *plltestoverride = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxbbenoverride = (localVal & ((uint32_t)0x00000008)) >> 3;
    *txbbenoverride = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxenoverride = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txpaenoverride = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl40_pll_test_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl40_pll_test_override_setf(uint8_t plltestoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL40_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)plltestoverride << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl40_rx_bb_en_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bb_top_i_dig_bb_ctrl40_rx_bb_en_override_setf(uint8_t rxbbenoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL40_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rxbbenoverride << 3));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl40_tx_bb_en_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bb_top_i_dig_bb_ctrl40_tx_bb_en_override_setf(uint8_t txbbenoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL40_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)txbbenoverride << 2));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl40_rx_en_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bb_top_i_dig_bb_ctrl40_rx_en_override_setf(uint8_t rxenoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL40_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rxenoverride << 1));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl40_tx_pa_en_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl40_tx_pa_en_override_setf(uint8_t txpaenoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL40_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL40_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txpaenoverride << 0));
}

 /**
 * @brief I_DIG_BB_CTRL41 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     08      pa_setb_fr_spir   1
 *     07          rst_fr_spir   0
 *     06   tpm_mod_en_fr_spir   0
 *     05   rx_at_ctrl0_fr_spir   0
 *     04   tx_at_ctrl0_fr_spir   0
 *     03     rx_bb_en_fr_spir   0
 *     02     tx_bb_en_fr_spir   0
 *     01        rx_en_fr_spir   0
 *     00     tx_pa_en_fr_spir   0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL41_OFFSET 0x000000A4


__INLINE uint32_t bb_top_i_dig_bb_ctrl41_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl41_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL41_PA_SETB_FR_SPIR_BIT                   ((uint32_t)0x00000100)
#define BB_TOP_I_DIG_BB_CTRL41_PA_SETB_FR_SPIR_POS                   8
#define BB_TOP_I_DIG_BB_CTRL41_RST_FR_SPIR_BIT                       ((uint32_t)0x00000080)
#define BB_TOP_I_DIG_BB_CTRL41_RST_FR_SPIR_POS                       7
#define BB_TOP_I_DIG_BB_CTRL41_TPM_MOD_EN_FR_SPIR_BIT                ((uint32_t)0x00000040)
#define BB_TOP_I_DIG_BB_CTRL41_TPM_MOD_EN_FR_SPIR_POS                6
#define BB_TOP_I_DIG_BB_CTRL41_RX_AT_CTRL0_FR_SPIR_BIT               ((uint32_t)0x00000020)
#define BB_TOP_I_DIG_BB_CTRL41_RX_AT_CTRL0_FR_SPIR_POS               5
#define BB_TOP_I_DIG_BB_CTRL41_TX_AT_CTRL0_FR_SPIR_BIT               ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL41_TX_AT_CTRL0_FR_SPIR_POS               4
#define BB_TOP_I_DIG_BB_CTRL41_RX_BB_EN_FR_SPIR_BIT                  ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL41_RX_BB_EN_FR_SPIR_POS                  3
#define BB_TOP_I_DIG_BB_CTRL41_TX_BB_EN_FR_SPIR_BIT                  ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL41_TX_BB_EN_FR_SPIR_POS                  2
#define BB_TOP_I_DIG_BB_CTRL41_RX_EN_FR_SPIR_BIT                     ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL41_RX_EN_FR_SPIR_POS                     1
#define BB_TOP_I_DIG_BB_CTRL41_TX_PA_EN_FR_SPIR_BIT                  ((uint32_t)0x00000001)
#define BB_TOP_I_DIG_BB_CTRL41_TX_PA_EN_FR_SPIR_POS                  0

#define BB_TOP_I_DIG_BB_CTRL41_PA_SETB_FR_SPIR_RST                   0x1
#define BB_TOP_I_DIG_BB_CTRL41_RST_FR_SPIR_RST                       0x0
#define BB_TOP_I_DIG_BB_CTRL41_TPM_MOD_EN_FR_SPIR_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL41_RX_AT_CTRL0_FR_SPIR_RST               0x0
#define BB_TOP_I_DIG_BB_CTRL41_TX_AT_CTRL0_FR_SPIR_RST               0x0
#define BB_TOP_I_DIG_BB_CTRL41_RX_BB_EN_FR_SPIR_RST                  0x0
#define BB_TOP_I_DIG_BB_CTRL41_TX_BB_EN_FR_SPIR_RST                  0x0
#define BB_TOP_I_DIG_BB_CTRL41_RX_EN_FR_SPIR_RST                     0x0
#define BB_TOP_I_DIG_BB_CTRL41_TX_PA_EN_FR_SPIR_RST                  0x0

__INLINE void bb_top_i_dig_bb_ctrl41_pack(uint8_t pasetbfrspir, uint8_t rstfrspir, uint8_t tpmmodenfrspir, uint8_t rxatctrl0frspir, uint8_t txatctrl0frspir, uint8_t rxbbenfrspir, uint8_t txbbenfrspir, uint8_t rxenfrspir, uint8_t txpaenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)pasetbfrspir << 8) | ((uint32_t)rstfrspir << 7) | ((uint32_t)tpmmodenfrspir << 6) | ((uint32_t)rxatctrl0frspir << 5) | ((uint32_t)txatctrl0frspir << 4) | ((uint32_t)rxbbenfrspir << 3) | ((uint32_t)txbbenfrspir << 2) | ((uint32_t)rxenfrspir << 1) | ((uint32_t)txpaenfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl41_unpack(uint8_t* pasetbfrspir, uint8_t* rstfrspir, uint8_t* tpmmodenfrspir, uint8_t* rxatctrl0frspir, uint8_t* txatctrl0frspir, uint8_t* rxbbenfrspir, uint8_t* txbbenfrspir, uint8_t* rxenfrspir, uint8_t* txpaenfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);

    *pasetbfrspir = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rstfrspir = (localVal & ((uint32_t)0x00000080)) >> 7;
    *tpmmodenfrspir = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rxatctrl0frspir = (localVal & ((uint32_t)0x00000020)) >> 5;
    *txatctrl0frspir = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxbbenfrspir = (localVal & ((uint32_t)0x00000008)) >> 3;
    *txbbenfrspir = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxenfrspir = (localVal & ((uint32_t)0x00000002)) >> 1;
    *txpaenfrspir = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_pa_setb_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl41_pa_setb_fr_spir_setf(uint8_t pasetbfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)pasetbfrspir << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_rst_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bb_top_i_dig_bb_ctrl41_rst_fr_spir_setf(uint8_t rstfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rstfrspir << 7));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_tpm_mod_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bb_top_i_dig_bb_ctrl41_tpm_mod_en_fr_spir_setf(uint8_t tpmmodenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)tpmmodenfrspir << 6));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_rx_at_ctrl0_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bb_top_i_dig_bb_ctrl41_rx_at_ctrl0_fr_spir_setf(uint8_t rxatctrl0frspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rxatctrl0frspir << 5));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_tx_at_ctrl0_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl41_tx_at_ctrl0_fr_spir_setf(uint8_t txatctrl0frspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)txatctrl0frspir << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_rx_bb_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bb_top_i_dig_bb_ctrl41_rx_bb_en_fr_spir_setf(uint8_t rxbbenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rxbbenfrspir << 3));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_tx_bb_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bb_top_i_dig_bb_ctrl41_tx_bb_en_fr_spir_setf(uint8_t txbbenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)txbbenfrspir << 2));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_rx_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bb_top_i_dig_bb_ctrl41_rx_en_fr_spir_setf(uint8_t rxenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rxenfrspir << 1));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl41_tx_pa_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl41_tx_pa_en_fr_spir_setf(uint8_t txpaenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL41_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL41_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txpaenfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL42 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06    synth_lo_override   0
 *     05   synth_tpm_en_override   0
 *     04   synth_vmid_override   0
 *     03   synth_bias_en_override   0
 *     02   synth_pfd_override   0
 *     01   synth_fch_override   0
 *     00   synth_ldo_override   0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL42_OFFSET 0x000000A8


__INLINE uint32_t bb_top_i_dig_bb_ctrl42_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl42_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_LO_OVERRIDE_BIT                 ((uint32_t)0x00000040)
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_LO_OVERRIDE_POS                 6
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_TPM_EN_OVERRIDE_BIT             ((uint32_t)0x00000020)
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_TPM_EN_OVERRIDE_POS             5
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_VMID_OVERRIDE_BIT               ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_VMID_OVERRIDE_POS               4
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_BIAS_EN_OVERRIDE_BIT            ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_BIAS_EN_OVERRIDE_POS            3
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_PFD_OVERRIDE_BIT                ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_PFD_OVERRIDE_POS                2
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_FCH_OVERRIDE_BIT                ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_FCH_OVERRIDE_POS                1
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_LDO_OVERRIDE_BIT                ((uint32_t)0x00000001)
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_LDO_OVERRIDE_POS                0

#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_LO_OVERRIDE_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_TPM_EN_OVERRIDE_RST             0x0
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_VMID_OVERRIDE_RST               0x0
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_BIAS_EN_OVERRIDE_RST            0x0
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_PFD_OVERRIDE_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_FCH_OVERRIDE_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL42_SYNTH_LDO_OVERRIDE_RST                0x0

__INLINE void bb_top_i_dig_bb_ctrl42_pack(uint8_t synthlooverride, uint8_t synthtpmenoverride, uint8_t synthvmidoverride, uint8_t synthbiasenoverride, uint8_t synthpfdoverride, uint8_t synthfchoverride, uint8_t synthldooverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)synthlooverride << 6) | ((uint32_t)synthtpmenoverride << 5) | ((uint32_t)synthvmidoverride << 4) | ((uint32_t)synthbiasenoverride << 3) | ((uint32_t)synthpfdoverride << 2) | ((uint32_t)synthfchoverride << 1) | ((uint32_t)synthldooverride << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl42_unpack(uint8_t* synthlooverride, uint8_t* synthtpmenoverride, uint8_t* synthvmidoverride, uint8_t* synthbiasenoverride, uint8_t* synthpfdoverride, uint8_t* synthfchoverride, uint8_t* synthldooverride)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);

    *synthlooverride = (localVal & ((uint32_t)0x00000040)) >> 6;
    *synthtpmenoverride = (localVal & ((uint32_t)0x00000020)) >> 5;
    *synthvmidoverride = (localVal & ((uint32_t)0x00000010)) >> 4;
    *synthbiasenoverride = (localVal & ((uint32_t)0x00000008)) >> 3;
    *synthpfdoverride = (localVal & ((uint32_t)0x00000004)) >> 2;
    *synthfchoverride = (localVal & ((uint32_t)0x00000002)) >> 1;
    *synthldooverride = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl42_synth_lo_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bb_top_i_dig_bb_ctrl42_synth_lo_override_setf(uint8_t synthlooverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)synthlooverride << 6));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl42_synth_tpm_en_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bb_top_i_dig_bb_ctrl42_synth_tpm_en_override_setf(uint8_t synthtpmenoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)synthtpmenoverride << 5));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl42_synth_vmid_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl42_synth_vmid_override_setf(uint8_t synthvmidoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)synthvmidoverride << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl42_synth_bias_en_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bb_top_i_dig_bb_ctrl42_synth_bias_en_override_setf(uint8_t synthbiasenoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)synthbiasenoverride << 3));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl42_synth_pfd_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bb_top_i_dig_bb_ctrl42_synth_pfd_override_setf(uint8_t synthpfdoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)synthpfdoverride << 2));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl42_synth_fch_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bb_top_i_dig_bb_ctrl42_synth_fch_override_setf(uint8_t synthfchoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)synthfchoverride << 1));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl42_synth_ldo_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl42_synth_ldo_override_setf(uint8_t synthldooverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL42_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL42_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)synthldooverride << 0));
}

 /**
 * @brief I_DIG_BB_CTRL43 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     19   tpm_dac_reset_fr_spir   0
 *     18        ct_en_fr_spir   0
 *     17      vmid_en_fr_spir   0
 *     16       tpm_en_fr_spir   0
 *     15   en_tpm_var_fr_spir   0
 *     14   reset_ndiv_fr_spir   0
 *     13       pfd_en_fr_spir   0
 *     12   fastcharge_vref_vco_fr_spir   0
 *     11   fch_vco_vos_fr_spir   0
 *     10   fch_vco_pbgen_fr_spir   0
 *     09   cp_bias_en_fr_spir   0
 *     08   en_vco_bias_fr_spir   0
 *     07   en_pll_imir_fr_spir   0
 *     06    en_pllbuf_fr_spir   0
 *     05      en_lotx_fr_spir   0
 *     04      en_lorx_fr_spir   0
 *     03    pu_ldo_lo_fr_spir   0
 *     02   pu_ldo_pll_fr_spir   0
 *     01   pu_ldo_vco_fr_spir   0
 *     00   pu_ldo_top_fr_spir   0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL43_OFFSET 0x000000AC


__INLINE uint32_t bb_top_i_dig_bb_ctrl43_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl43_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL43_TPM_DAC_RESET_FR_SPIR_BIT             ((uint32_t)0x00080000)
#define BB_TOP_I_DIG_BB_CTRL43_TPM_DAC_RESET_FR_SPIR_POS             19
#define BB_TOP_I_DIG_BB_CTRL43_CT_EN_FR_SPIR_BIT                     ((uint32_t)0x00040000)
#define BB_TOP_I_DIG_BB_CTRL43_CT_EN_FR_SPIR_POS                     18
#define BB_TOP_I_DIG_BB_CTRL43_VMID_EN_FR_SPIR_BIT                   ((uint32_t)0x00020000)
#define BB_TOP_I_DIG_BB_CTRL43_VMID_EN_FR_SPIR_POS                   17
#define BB_TOP_I_DIG_BB_CTRL43_TPM_EN_FR_SPIR_BIT                    ((uint32_t)0x00010000)
#define BB_TOP_I_DIG_BB_CTRL43_TPM_EN_FR_SPIR_POS                    16
#define BB_TOP_I_DIG_BB_CTRL43_EN_TPM_VAR_FR_SPIR_BIT                ((uint32_t)0x00008000)
#define BB_TOP_I_DIG_BB_CTRL43_EN_TPM_VAR_FR_SPIR_POS                15
#define BB_TOP_I_DIG_BB_CTRL43_RESET_NDIV_FR_SPIR_BIT                ((uint32_t)0x00004000)
#define BB_TOP_I_DIG_BB_CTRL43_RESET_NDIV_FR_SPIR_POS                14
#define BB_TOP_I_DIG_BB_CTRL43_PFD_EN_FR_SPIR_BIT                    ((uint32_t)0x00002000)
#define BB_TOP_I_DIG_BB_CTRL43_PFD_EN_FR_SPIR_POS                    13
#define BB_TOP_I_DIG_BB_CTRL43_FASTCHARGE_VREF_VCO_FR_SPIR_BIT       ((uint32_t)0x00001000)
#define BB_TOP_I_DIG_BB_CTRL43_FASTCHARGE_VREF_VCO_FR_SPIR_POS       12
#define BB_TOP_I_DIG_BB_CTRL43_FCH_VCO_VOS_FR_SPIR_BIT               ((uint32_t)0x00000800)
#define BB_TOP_I_DIG_BB_CTRL43_FCH_VCO_VOS_FR_SPIR_POS               11
#define BB_TOP_I_DIG_BB_CTRL43_FCH_VCO_PBGEN_FR_SPIR_BIT             ((uint32_t)0x00000400)
#define BB_TOP_I_DIG_BB_CTRL43_FCH_VCO_PBGEN_FR_SPIR_POS             10
#define BB_TOP_I_DIG_BB_CTRL43_CP_BIAS_EN_FR_SPIR_BIT                ((uint32_t)0x00000200)
#define BB_TOP_I_DIG_BB_CTRL43_CP_BIAS_EN_FR_SPIR_POS                9
#define BB_TOP_I_DIG_BB_CTRL43_EN_VCO_BIAS_FR_SPIR_BIT               ((uint32_t)0x00000100)
#define BB_TOP_I_DIG_BB_CTRL43_EN_VCO_BIAS_FR_SPIR_POS               8
#define BB_TOP_I_DIG_BB_CTRL43_EN_PLL_IMIR_FR_SPIR_BIT               ((uint32_t)0x00000080)
#define BB_TOP_I_DIG_BB_CTRL43_EN_PLL_IMIR_FR_SPIR_POS               7
#define BB_TOP_I_DIG_BB_CTRL43_EN_PLLBUF_FR_SPIR_BIT                 ((uint32_t)0x00000040)
#define BB_TOP_I_DIG_BB_CTRL43_EN_PLLBUF_FR_SPIR_POS                 6
#define BB_TOP_I_DIG_BB_CTRL43_EN_LOTX_FR_SPIR_BIT                   ((uint32_t)0x00000020)
#define BB_TOP_I_DIG_BB_CTRL43_EN_LOTX_FR_SPIR_POS                   5
#define BB_TOP_I_DIG_BB_CTRL43_EN_LORX_FR_SPIR_BIT                   ((uint32_t)0x00000010)
#define BB_TOP_I_DIG_BB_CTRL43_EN_LORX_FR_SPIR_POS                   4
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_LO_FR_SPIR_BIT                 ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_LO_FR_SPIR_POS                 3
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_PLL_FR_SPIR_BIT                ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_PLL_FR_SPIR_POS                2
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_VCO_FR_SPIR_BIT                ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_VCO_FR_SPIR_POS                1
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_TOP_FR_SPIR_BIT                ((uint32_t)0x00000001)
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_TOP_FR_SPIR_POS                0

#define BB_TOP_I_DIG_BB_CTRL43_TPM_DAC_RESET_FR_SPIR_RST             0x0
#define BB_TOP_I_DIG_BB_CTRL43_CT_EN_FR_SPIR_RST                     0x0
#define BB_TOP_I_DIG_BB_CTRL43_VMID_EN_FR_SPIR_RST                   0x0
#define BB_TOP_I_DIG_BB_CTRL43_TPM_EN_FR_SPIR_RST                    0x0
#define BB_TOP_I_DIG_BB_CTRL43_EN_TPM_VAR_FR_SPIR_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL43_RESET_NDIV_FR_SPIR_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL43_PFD_EN_FR_SPIR_RST                    0x0
#define BB_TOP_I_DIG_BB_CTRL43_FASTCHARGE_VREF_VCO_FR_SPIR_RST       0x0
#define BB_TOP_I_DIG_BB_CTRL43_FCH_VCO_VOS_FR_SPIR_RST               0x0
#define BB_TOP_I_DIG_BB_CTRL43_FCH_VCO_PBGEN_FR_SPIR_RST             0x0
#define BB_TOP_I_DIG_BB_CTRL43_CP_BIAS_EN_FR_SPIR_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL43_EN_VCO_BIAS_FR_SPIR_RST               0x0
#define BB_TOP_I_DIG_BB_CTRL43_EN_PLL_IMIR_FR_SPIR_RST               0x0
#define BB_TOP_I_DIG_BB_CTRL43_EN_PLLBUF_FR_SPIR_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL43_EN_LOTX_FR_SPIR_RST                   0x0
#define BB_TOP_I_DIG_BB_CTRL43_EN_LORX_FR_SPIR_RST                   0x0
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_LO_FR_SPIR_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_PLL_FR_SPIR_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_VCO_FR_SPIR_RST                0x0
#define BB_TOP_I_DIG_BB_CTRL43_PU_LDO_TOP_FR_SPIR_RST                0x0

__INLINE void bb_top_i_dig_bb_ctrl43_pack(uint8_t tpmdacresetfrspir, uint8_t ctenfrspir, uint8_t vmidenfrspir, uint8_t tpmenfrspir, uint8_t entpmvarfrspir, uint8_t resetndivfrspir, uint8_t pfdenfrspir, uint8_t fastchargevrefvcofrspir, uint8_t fchvcovosfrspir, uint8_t fchvcopbgenfrspir, uint8_t cpbiasenfrspir, uint8_t envcobiasfrspir, uint8_t enpllimirfrspir, uint8_t enpllbuffrspir, uint8_t enlotxfrspir, uint8_t enlorxfrspir, uint8_t puldolofrspir, uint8_t puldopllfrspir, uint8_t puldovcofrspir, uint8_t puldotopfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)tpmdacresetfrspir << 19) | ((uint32_t)ctenfrspir << 18) | ((uint32_t)vmidenfrspir << 17) | ((uint32_t)tpmenfrspir << 16) | ((uint32_t)entpmvarfrspir << 15) | ((uint32_t)resetndivfrspir << 14) | ((uint32_t)pfdenfrspir << 13) | ((uint32_t)fastchargevrefvcofrspir << 12) | ((uint32_t)fchvcovosfrspir << 11) | ((uint32_t)fchvcopbgenfrspir << 10) | ((uint32_t)cpbiasenfrspir << 9) | ((uint32_t)envcobiasfrspir << 8) | ((uint32_t)enpllimirfrspir << 7) | ((uint32_t)enpllbuffrspir << 6) | ((uint32_t)enlotxfrspir << 5) | ((uint32_t)enlorxfrspir << 4) | ((uint32_t)puldolofrspir << 3) | ((uint32_t)puldopllfrspir << 2) | ((uint32_t)puldovcofrspir << 1) | ((uint32_t)puldotopfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl43_unpack(uint8_t* tpmdacresetfrspir, uint8_t* ctenfrspir, uint8_t* vmidenfrspir, uint8_t* tpmenfrspir, uint8_t* entpmvarfrspir, uint8_t* resetndivfrspir, uint8_t* pfdenfrspir, uint8_t* fastchargevrefvcofrspir, uint8_t* fchvcovosfrspir, uint8_t* fchvcopbgenfrspir, uint8_t* cpbiasenfrspir, uint8_t* envcobiasfrspir, uint8_t* enpllimirfrspir, uint8_t* enpllbuffrspir, uint8_t* enlotxfrspir, uint8_t* enlorxfrspir, uint8_t* puldolofrspir, uint8_t* puldopllfrspir, uint8_t* puldovcofrspir, uint8_t* puldotopfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);

    *tpmdacresetfrspir = (localVal & ((uint32_t)0x00080000)) >> 19;
    *ctenfrspir = (localVal & ((uint32_t)0x00040000)) >> 18;
    *vmidenfrspir = (localVal & ((uint32_t)0x00020000)) >> 17;
    *tpmenfrspir = (localVal & ((uint32_t)0x00010000)) >> 16;
    *entpmvarfrspir = (localVal & ((uint32_t)0x00008000)) >> 15;
    *resetndivfrspir = (localVal & ((uint32_t)0x00004000)) >> 14;
    *pfdenfrspir = (localVal & ((uint32_t)0x00002000)) >> 13;
    *fastchargevrefvcofrspir = (localVal & ((uint32_t)0x00001000)) >> 12;
    *fchvcovosfrspir = (localVal & ((uint32_t)0x00000800)) >> 11;
    *fchvcopbgenfrspir = (localVal & ((uint32_t)0x00000400)) >> 10;
    *cpbiasenfrspir = (localVal & ((uint32_t)0x00000200)) >> 9;
    *envcobiasfrspir = (localVal & ((uint32_t)0x00000100)) >> 8;
    *enpllimirfrspir = (localVal & ((uint32_t)0x00000080)) >> 7;
    *enpllbuffrspir = (localVal & ((uint32_t)0x00000040)) >> 6;
    *enlotxfrspir = (localVal & ((uint32_t)0x00000020)) >> 5;
    *enlorxfrspir = (localVal & ((uint32_t)0x00000010)) >> 4;
    *puldolofrspir = (localVal & ((uint32_t)0x00000008)) >> 3;
    *puldopllfrspir = (localVal & ((uint32_t)0x00000004)) >> 2;
    *puldovcofrspir = (localVal & ((uint32_t)0x00000002)) >> 1;
    *puldotopfrspir = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_tpm_dac_reset_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00080000)) >> 19);
}

__INLINE void bb_top_i_dig_bb_ctrl43_tpm_dac_reset_fr_spir_setf(uint8_t tpmdacresetfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00080000)) | ((uint32_t)tpmdacresetfrspir << 19));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_ct_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void bb_top_i_dig_bb_ctrl43_ct_en_fr_spir_setf(uint8_t ctenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)ctenfrspir << 18));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_vmid_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void bb_top_i_dig_bb_ctrl43_vmid_en_fr_spir_setf(uint8_t vmidenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)vmidenfrspir << 17));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_tpm_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl43_tpm_en_fr_spir_setf(uint8_t tpmenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)tpmenfrspir << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_en_tpm_var_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bb_top_i_dig_bb_ctrl43_en_tpm_var_fr_spir_setf(uint8_t entpmvarfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)entpmvarfrspir << 15));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_reset_ndiv_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bb_top_i_dig_bb_ctrl43_reset_ndiv_fr_spir_setf(uint8_t resetndivfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)resetndivfrspir << 14));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_pfd_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bb_top_i_dig_bb_ctrl43_pfd_en_fr_spir_setf(uint8_t pfdenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)pfdenfrspir << 13));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_fastcharge_vref_vco_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bb_top_i_dig_bb_ctrl43_fastcharge_vref_vco_fr_spir_setf(uint8_t fastchargevrefvcofrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)fastchargevrefvcofrspir << 12));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_fch_vco_vos_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void bb_top_i_dig_bb_ctrl43_fch_vco_vos_fr_spir_setf(uint8_t fchvcovosfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)fchvcovosfrspir << 11));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_fch_vco_pbgen_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bb_top_i_dig_bb_ctrl43_fch_vco_pbgen_fr_spir_setf(uint8_t fchvcopbgenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)fchvcopbgenfrspir << 10));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_cp_bias_en_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void bb_top_i_dig_bb_ctrl43_cp_bias_en_fr_spir_setf(uint8_t cpbiasenfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)cpbiasenfrspir << 9));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_en_vco_bias_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl43_en_vco_bias_fr_spir_setf(uint8_t envcobiasfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)envcobiasfrspir << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_en_pll_imir_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void bb_top_i_dig_bb_ctrl43_en_pll_imir_fr_spir_setf(uint8_t enpllimirfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)enpllimirfrspir << 7));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_en_pllbuf_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void bb_top_i_dig_bb_ctrl43_en_pllbuf_fr_spir_setf(uint8_t enpllbuffrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)enpllbuffrspir << 6));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_en_lotx_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void bb_top_i_dig_bb_ctrl43_en_lotx_fr_spir_setf(uint8_t enlotxfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)enlotxfrspir << 5));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_en_lorx_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl43_en_lorx_fr_spir_setf(uint8_t enlorxfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)enlorxfrspir << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_pu_ldo_lo_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void bb_top_i_dig_bb_ctrl43_pu_ldo_lo_fr_spir_setf(uint8_t puldolofrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)puldolofrspir << 3));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_pu_ldo_pll_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bb_top_i_dig_bb_ctrl43_pu_ldo_pll_fr_spir_setf(uint8_t puldopllfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)puldopllfrspir << 2));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_pu_ldo_vco_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bb_top_i_dig_bb_ctrl43_pu_ldo_vco_fr_spir_setf(uint8_t puldovcofrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)puldovcofrspir << 1));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl43_pu_ldo_top_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl43_pu_ldo_top_fr_spir_setf(uint8_t puldotopfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL43_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL43_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)puldotopfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL44 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31     tpd_plry_fr_spir   0
 *  26:24      pi_dsft_fr_spir   0b1
 *  17:16     intg_dly_fr_spir   0b0
 *  12:08           tx_sdm_dly   0b0
 *  04:00           tx_dac_dly   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL44_OFFSET 0x000000B0


__INLINE uint32_t bb_top_i_dig_bb_ctrl44_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl44_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL44_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL44_TPD_PLRY_FR_SPIR_BIT                  ((uint32_t)0x80000000)
#define BB_TOP_I_DIG_BB_CTRL44_TPD_PLRY_FR_SPIR_POS                  31
#define BB_TOP_I_DIG_BB_CTRL44_PI_DSFT_FR_SPIR_MASK                  ((uint32_t)0x07000000)
#define BB_TOP_I_DIG_BB_CTRL44_PI_DSFT_FR_SPIR_LSB                   24
#define BB_TOP_I_DIG_BB_CTRL44_PI_DSFT_FR_SPIR_WIDTH                 ((uint32_t)0x00000003)
#define BB_TOP_I_DIG_BB_CTRL44_INTG_DLY_FR_SPIR_MASK                 ((uint32_t)0x00030000)
#define BB_TOP_I_DIG_BB_CTRL44_INTG_DLY_FR_SPIR_LSB                  16
#define BB_TOP_I_DIG_BB_CTRL44_INTG_DLY_FR_SPIR_WIDTH                ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL44_TX_SDM_DLY_MASK                       ((uint32_t)0x00001F00)
#define BB_TOP_I_DIG_BB_CTRL44_TX_SDM_DLY_LSB                        8
#define BB_TOP_I_DIG_BB_CTRL44_TX_SDM_DLY_WIDTH                      ((uint32_t)0x00000005)
#define BB_TOP_I_DIG_BB_CTRL44_TX_DAC_DLY_MASK                       ((uint32_t)0x0000001F)
#define BB_TOP_I_DIG_BB_CTRL44_TX_DAC_DLY_LSB                        0
#define BB_TOP_I_DIG_BB_CTRL44_TX_DAC_DLY_WIDTH                      ((uint32_t)0x00000005)

#define BB_TOP_I_DIG_BB_CTRL44_TPD_PLRY_FR_SPIR_RST                  0x0
#define BB_TOP_I_DIG_BB_CTRL44_PI_DSFT_FR_SPIR_RST                   0x1
#define BB_TOP_I_DIG_BB_CTRL44_INTG_DLY_FR_SPIR_RST                  0x0
#define BB_TOP_I_DIG_BB_CTRL44_TX_SDM_DLY_RST                        0x0
#define BB_TOP_I_DIG_BB_CTRL44_TX_DAC_DLY_RST                        0x0

__INLINE void bb_top_i_dig_bb_ctrl44_pack(uint8_t tpdplryfrspir, uint8_t pidsftfrspir, uint8_t intgdlyfrspir, uint8_t txsdmdly, uint8_t txdacdly)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL44_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)tpdplryfrspir << 31) | ((uint32_t)pidsftfrspir << 24) | ((uint32_t)intgdlyfrspir << 16) | ((uint32_t)txsdmdly << 8) | ((uint32_t)txdacdly << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl44_unpack(uint8_t* tpdplryfrspir, uint8_t* pidsftfrspir, uint8_t* intgdlyfrspir, uint8_t* txsdmdly, uint8_t* txdacdly)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR);

    *tpdplryfrspir = (localVal & ((uint32_t)0x80000000)) >> 31;
    *pidsftfrspir = (localVal & ((uint32_t)0x07000000)) >> 24;
    *intgdlyfrspir = (localVal & ((uint32_t)0x00030000)) >> 16;
    *txsdmdly = (localVal & ((uint32_t)0x00001F00)) >> 8;
    *txdacdly = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl44_tpd_plry_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE void bb_top_i_dig_bb_ctrl44_tpd_plry_fr_spir_setf(uint8_t tpdplryfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL44_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x80000000)) | ((uint32_t)tpdplryfrspir << 31));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl44_pi_dsft_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl44_pi_dsft_fr_spir_setf(uint8_t pidsftfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL44_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)pidsftfrspir << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl44_intg_dly_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl44_intg_dly_fr_spir_setf(uint8_t intgdlyfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL44_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)intgdlyfrspir << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl44_tx_sdm_dly_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl44_tx_sdm_dly_setf(uint8_t txsdmdly)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL44_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00001F00)) | ((uint32_t)txsdmdly << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl44_tx_dac_dly_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl44_tx_dac_dly_setf(uint8_t txdacdly)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL44_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL44_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000001F)) | ((uint32_t)txdacdly << 0));
}

 /**
 * @brief I_DIG_BB_CTRL45 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     24              if_freq   0
 *  23:16          rx_freq_off   0b0
 *  15:08          tx_freq_off   0b0
 *  07:00               rf_chn   0b37
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL45_OFFSET 0x000000B4


__INLINE uint32_t bb_top_i_dig_bb_ctrl45_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl45_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL45_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL45_IF_FREQ_BIT                           ((uint32_t)0x01000000)
#define BB_TOP_I_DIG_BB_CTRL45_IF_FREQ_POS                           24
#define BB_TOP_I_DIG_BB_CTRL45_RX_FREQ_OFF_MASK                      ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL45_RX_FREQ_OFF_LSB                       16
#define BB_TOP_I_DIG_BB_CTRL45_RX_FREQ_OFF_WIDTH                     ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL45_TX_FREQ_OFF_MASK                      ((uint32_t)0x0000FF00)
#define BB_TOP_I_DIG_BB_CTRL45_TX_FREQ_OFF_LSB                       8
#define BB_TOP_I_DIG_BB_CTRL45_TX_FREQ_OFF_WIDTH                     ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL45_RF_CHN_MASK                           ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL45_RF_CHN_LSB                            0
#define BB_TOP_I_DIG_BB_CTRL45_RF_CHN_WIDTH                          ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL45_IF_FREQ_RST                           0x0
#define BB_TOP_I_DIG_BB_CTRL45_RX_FREQ_OFF_RST                       0x0
#define BB_TOP_I_DIG_BB_CTRL45_TX_FREQ_OFF_RST                       0x0
#define BB_TOP_I_DIG_BB_CTRL45_RF_CHN_RST                            0x37

__INLINE void bb_top_i_dig_bb_ctrl45_pack(uint8_t iffreq, uint8_t rxfreqoff, uint8_t txfreqoff, uint8_t rfchn)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL45_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)iffreq << 24) | ((uint32_t)rxfreqoff << 16) | ((uint32_t)txfreqoff << 8) | ((uint32_t)rfchn << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl45_unpack(uint8_t* iffreq, uint8_t* rxfreqoff, uint8_t* txfreqoff, uint8_t* rfchn)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR);

    *iffreq = (localVal & ((uint32_t)0x01000000)) >> 24;
    *rxfreqoff = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txfreqoff = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rfchn = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl45_if_freq_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl45_if_freq_setf(uint8_t iffreq)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL45_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)iffreq << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl45_rx_freq_off_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl45_rx_freq_off_setf(uint8_t rxfreqoff)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL45_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxfreqoff << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl45_tx_freq_off_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl45_tx_freq_off_setf(uint8_t txfreqoff)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL45_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)txfreqoff << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl45_rf_chn_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl45_rf_chn_setf(uint8_t rfchn)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL45_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL45_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rfchn << 0));
}

 /**
 * @brief I_DIG_BB_CTRL46 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  16:12        trx_pa_target   0b0
 *  11:08         tm_pa_settle   0b8
 *  06:04     trx_paramp_speed   0b1
 *  02:00   len_dly_ramp_fr_spir   0b1
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL46_OFFSET 0x000000B8


__INLINE uint32_t bb_top_i_dig_bb_ctrl46_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl46_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL46_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL46_TRX_PA_TARGET_MASK                    ((uint32_t)0x0001F000)
#define BB_TOP_I_DIG_BB_CTRL46_TRX_PA_TARGET_LSB                     12
#define BB_TOP_I_DIG_BB_CTRL46_TRX_PA_TARGET_WIDTH                   ((uint32_t)0x00000005)
#define BB_TOP_I_DIG_BB_CTRL46_TM_PA_SETTLE_MASK                     ((uint32_t)0x00000F00)
#define BB_TOP_I_DIG_BB_CTRL46_TM_PA_SETTLE_LSB                      8
#define BB_TOP_I_DIG_BB_CTRL46_TM_PA_SETTLE_WIDTH                    ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL46_TRX_PARAMP_SPEED_MASK                 ((uint32_t)0x00000070)
#define BB_TOP_I_DIG_BB_CTRL46_TRX_PARAMP_SPEED_LSB                  4
#define BB_TOP_I_DIG_BB_CTRL46_TRX_PARAMP_SPEED_WIDTH                ((uint32_t)0x00000003)
#define BB_TOP_I_DIG_BB_CTRL46_LEN_DLY_RAMP_FR_SPIR_MASK             ((uint32_t)0x00000007)
#define BB_TOP_I_DIG_BB_CTRL46_LEN_DLY_RAMP_FR_SPIR_LSB              0
#define BB_TOP_I_DIG_BB_CTRL46_LEN_DLY_RAMP_FR_SPIR_WIDTH            ((uint32_t)0x00000003)

#define BB_TOP_I_DIG_BB_CTRL46_TRX_PA_TARGET_RST                     0x0
#define BB_TOP_I_DIG_BB_CTRL46_TM_PA_SETTLE_RST                      0x8
#define BB_TOP_I_DIG_BB_CTRL46_TRX_PARAMP_SPEED_RST                  0x1
#define BB_TOP_I_DIG_BB_CTRL46_LEN_DLY_RAMP_FR_SPIR_RST              0x1

__INLINE void bb_top_i_dig_bb_ctrl46_pack(uint8_t trxpatarget, uint8_t tmpasettle, uint8_t trxparampspeed, uint8_t lendlyrampfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL46_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)trxpatarget << 12) | ((uint32_t)tmpasettle << 8) | ((uint32_t)trxparampspeed << 4) | ((uint32_t)lendlyrampfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl46_unpack(uint8_t* trxpatarget, uint8_t* tmpasettle, uint8_t* trxparampspeed, uint8_t* lendlyrampfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR);

    *trxpatarget = (localVal & ((uint32_t)0x0001F000)) >> 12;
    *tmpasettle = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *trxparampspeed = (localVal & ((uint32_t)0x00000070)) >> 4;
    *lendlyrampfrspir = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl46_trx_pa_target_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0001F000)) >> 12);
}

__INLINE void bb_top_i_dig_bb_ctrl46_trx_pa_target_setf(uint8_t trxpatarget)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL46_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0001F000)) | ((uint32_t)trxpatarget << 12));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl46_tm_pa_settle_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl46_tm_pa_settle_setf(uint8_t tmpasettle)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL46_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)tmpasettle << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl46_trx_paramp_speed_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000070)) >> 4);
}

__INLINE void bb_top_i_dig_bb_ctrl46_trx_paramp_speed_setf(uint8_t trxparampspeed)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL46_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000070)) | ((uint32_t)trxparampspeed << 4));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl46_len_dly_ramp_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl46_len_dly_ramp_fr_spir_setf(uint8_t lendlyrampfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL46_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL46_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)lendlyrampfrspir << 0));
}

 /**
 * @brief I_DIG_BB_CTRL47 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16         tx_bb_en_dly   0b108
 *  11:08      tm_rxafe_settle   0b8
 *  07:00         tm_pll_ftune   0b100
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL47_OFFSET 0x000000BC


__INLINE uint32_t bb_top_i_dig_bb_ctrl47_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl47_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL47_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL47_TX_BB_EN_DLY_MASK                     ((uint32_t)0x00FF0000)
#define BB_TOP_I_DIG_BB_CTRL47_TX_BB_EN_DLY_LSB                      16
#define BB_TOP_I_DIG_BB_CTRL47_TX_BB_EN_DLY_WIDTH                    ((uint32_t)0x00000008)
#define BB_TOP_I_DIG_BB_CTRL47_TM_RXAFE_SETTLE_MASK                  ((uint32_t)0x00000F00)
#define BB_TOP_I_DIG_BB_CTRL47_TM_RXAFE_SETTLE_LSB                   8
#define BB_TOP_I_DIG_BB_CTRL47_TM_RXAFE_SETTLE_WIDTH                 ((uint32_t)0x00000004)
#define BB_TOP_I_DIG_BB_CTRL47_TM_PLL_FTUNE_MASK                     ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL47_TM_PLL_FTUNE_LSB                      0
#define BB_TOP_I_DIG_BB_CTRL47_TM_PLL_FTUNE_WIDTH                    ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL47_TX_BB_EN_DLY_RST                      0x108
#define BB_TOP_I_DIG_BB_CTRL47_TM_RXAFE_SETTLE_RST                   0x8
#define BB_TOP_I_DIG_BB_CTRL47_TM_PLL_FTUNE_RST                      0x100

__INLINE void bb_top_i_dig_bb_ctrl47_pack(uint8_t txbbendly, uint8_t tmrxafesettle, uint8_t tmpllftune)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL47_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)txbbendly << 16) | ((uint32_t)tmrxafesettle << 8) | ((uint32_t)tmpllftune << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl47_unpack(uint8_t* txbbendly, uint8_t* tmrxafesettle, uint8_t* tmpllftune)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR);

    *txbbendly = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *tmrxafesettle = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *tmpllftune = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl47_tx_bb_en_dly_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl47_tx_bb_en_dly_setf(uint8_t txbbendly)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL47_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)txbbendly << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl47_tm_rxafe_settle_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl47_tm_rxafe_settle_setf(uint8_t tmrxafesettle)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL47_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)tmrxafesettle << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl47_tm_pll_ftune_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl47_tm_pll_ftune_setf(uint8_t tmpllftune)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL47_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL47_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)tmpllftune << 0));
}

 /**
 * @brief I_DIG_BB_CTRL48 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  29:24        dc_compq_flt1   0b32
 *  21:16        dc_compi_flt1   0b32
 *  13:08        dc_compq_flt2   0b32
 *  05:00        dc_compi_flt2   0b32
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL48_OFFSET 0x000000C0


__INLINE uint32_t bb_top_i_dig_bb_ctrl48_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl48_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL48_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT1_MASK                    ((uint32_t)0x3F000000)
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT1_LSB                     24
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT1_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT1_MASK                    ((uint32_t)0x003F0000)
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT1_LSB                     16
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT1_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT2_MASK                    ((uint32_t)0x00003F00)
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT2_LSB                     8
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT2_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT2_MASK                    ((uint32_t)0x0000003F)
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT2_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT2_WIDTH                   ((uint32_t)0x00000006)

#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT1_RST                     0x32
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT1_RST                     0x32
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPQ_FLT2_RST                     0x32
#define BB_TOP_I_DIG_BB_CTRL48_DC_COMPI_FLT2_RST                     0x32

__INLINE void bb_top_i_dig_bb_ctrl48_pack(uint8_t dccompqflt1, uint8_t dccompiflt1, uint8_t dccompqflt2, uint8_t dccompiflt2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL48_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)dccompqflt1 << 24) | ((uint32_t)dccompiflt1 << 16) | ((uint32_t)dccompqflt2 << 8) | ((uint32_t)dccompiflt2 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl48_unpack(uint8_t* dccompqflt1, uint8_t* dccompiflt1, uint8_t* dccompqflt2, uint8_t* dccompiflt2)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR);

    *dccompqflt1 = (localVal & ((uint32_t)0x3F000000)) >> 24;
    *dccompiflt1 = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *dccompqflt2 = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *dccompiflt2 = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl48_dc_compq_flt1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x3F000000)) >> 24);
}

__INLINE void bb_top_i_dig_bb_ctrl48_dc_compq_flt1_setf(uint8_t dccompqflt1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL48_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x3F000000)) | ((uint32_t)dccompqflt1 << 24));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl48_dc_compi_flt1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl48_dc_compi_flt1_setf(uint8_t dccompiflt1)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL48_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x003F0000)) | ((uint32_t)dccompiflt1 << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl48_dc_compq_flt2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl48_dc_compq_flt2_setf(uint8_t dccompqflt2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL48_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00003F00)) | ((uint32_t)dccompqflt2 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl48_dc_compi_flt2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl48_dc_compi_flt2_setf(uint8_t dccompiflt2)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL48_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL48_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000003F)) | ((uint32_t)dccompiflt2 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL49 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  17:16          dc_comp_sel   0b0
 *  13:08        dc_compq_flt0   0b32
 *  05:00        dc_compi_flt0   0b32
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL49_OFFSET 0x000000C4


__INLINE uint32_t bb_top_i_dig_bb_ctrl49_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl49_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL49_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMP_SEL_MASK                      ((uint32_t)0x00030000)
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMP_SEL_LSB                       16
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMP_SEL_WIDTH                     ((uint32_t)0x00000002)
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPQ_FLT0_MASK                    ((uint32_t)0x00003F00)
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPQ_FLT0_LSB                     8
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPQ_FLT0_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPI_FLT0_MASK                    ((uint32_t)0x0000003F)
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPI_FLT0_LSB                     0
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPI_FLT0_WIDTH                   ((uint32_t)0x00000006)

#define BB_TOP_I_DIG_BB_CTRL49_DC_COMP_SEL_RST                       0x0
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPQ_FLT0_RST                     0x32
#define BB_TOP_I_DIG_BB_CTRL49_DC_COMPI_FLT0_RST                     0x32

__INLINE void bb_top_i_dig_bb_ctrl49_pack(uint8_t dccompsel, uint8_t dccompqflt0, uint8_t dccompiflt0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL49_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)dccompsel << 16) | ((uint32_t)dccompqflt0 << 8) | ((uint32_t)dccompiflt0 << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl49_unpack(uint8_t* dccompsel, uint8_t* dccompqflt0, uint8_t* dccompiflt0)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR);

    *dccompsel = (localVal & ((uint32_t)0x00030000)) >> 16;
    *dccompqflt0 = (localVal & ((uint32_t)0x00003F00)) >> 8;
    *dccompiflt0 = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl49_dc_comp_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bb_top_i_dig_bb_ctrl49_dc_comp_sel_setf(uint8_t dccompsel)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL49_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)dccompsel << 16));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl49_dc_compq_flt0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00003F00)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl49_dc_compq_flt0_setf(uint8_t dccompqflt0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL49_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00003F00)) | ((uint32_t)dccompqflt0 << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl49_dc_compi_flt0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000003F)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl49_dc_compi_flt0_setf(uint8_t dccompiflt0)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL49_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL49_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000003F)) | ((uint32_t)dccompiflt0 << 0));
}

 /**
 * @brief I_DIG_BB_CTRL50 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     08    agc_gain_override   0
 *  07:00     agc_gain_fr_spir   0b0
 * </pre>
 */
#define BB_TOP_I_DIG_BB_CTRL50_OFFSET 0x000000C8


__INLINE uint32_t bb_top_i_dig_bb_ctrl50_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL50_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_dig_bb_ctrl50_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL50_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_DIG_BB_CTRL50_AGC_GAIN_OVERRIDE_BIT                 ((uint32_t)0x00000100)
#define BB_TOP_I_DIG_BB_CTRL50_AGC_GAIN_OVERRIDE_POS                 8
#define BB_TOP_I_DIG_BB_CTRL50_AGC_GAIN_FR_SPIR_MASK                 ((uint32_t)0x000000FF)
#define BB_TOP_I_DIG_BB_CTRL50_AGC_GAIN_FR_SPIR_LSB                  0
#define BB_TOP_I_DIG_BB_CTRL50_AGC_GAIN_FR_SPIR_WIDTH                ((uint32_t)0x00000008)

#define BB_TOP_I_DIG_BB_CTRL50_AGC_GAIN_OVERRIDE_RST                 0x0
#define BB_TOP_I_DIG_BB_CTRL50_AGC_GAIN_FR_SPIR_RST                  0x0

__INLINE void bb_top_i_dig_bb_ctrl50_pack(uint8_t agcgainoverride, uint8_t agcgainfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL50_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)agcgainoverride << 8) | ((uint32_t)agcgainfrspir << 0));
}

__INLINE void bb_top_i_dig_bb_ctrl50_unpack(uint8_t* agcgainoverride, uint8_t* agcgainfrspir)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL50_OFFSET + BB_TOP_BASE_ADDR);

    *agcgainoverride = (localVal & ((uint32_t)0x00000100)) >> 8;
    *agcgainfrspir = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl50_agc_gain_override_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL50_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void bb_top_i_dig_bb_ctrl50_agc_gain_override_setf(uint8_t agcgainoverride)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL50_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL50_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)agcgainoverride << 8));
}

__INLINE uint8_t bb_top_i_dig_bb_ctrl50_agc_gain_fr_spir_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL50_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void bb_top_i_dig_bb_ctrl50_agc_gain_fr_spir_setf(uint8_t agcgainfrspir)
{
    _PICO_REG_WR(BB_TOP_I_DIG_BB_CTRL50_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_DIG_BB_CTRL50_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)agcgainfrspir << 0));
}

 /**
 * @brief I_PLL_CTRL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:10     vout_ctrl_ldo_lo   0b1
 *  09:08    vout_ctrl_ldo_pll   0b1
 *  07:06    vout_ctrl_ldo_vco   0b1
 *  05:00             Reserved   0b0
 * </pre>
 */
#define BB_TOP_I_PLL_CTRL0_OFFSET 0x000000CC


__INLINE uint32_t bb_top_i_pll_ctrl0_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_pll_ctrl0_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_LO_MASK                 ((uint32_t)0x00000C00)
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_LO_LSB                  10
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_LO_WIDTH                ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_PLL_MASK                ((uint32_t)0x00000300)
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_PLL_LSB                 8
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_PLL_WIDTH               ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_VCO_MASK                ((uint32_t)0x000000C0)
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_VCO_LSB                 6
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_VCO_WIDTH               ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL0_RESERVED_MASK                         ((uint32_t)0x0000003F)
#define BB_TOP_I_PLL_CTRL0_RESERVED_LSB                          0
#define BB_TOP_I_PLL_CTRL0_RESERVED_WIDTH                        ((uint32_t)0x00000006)

#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_LO_RST                  0x1
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_PLL_RST                 0x1
#define BB_TOP_I_PLL_CTRL0_VOUT_CTRL_LDO_VCO_RST                 0x1
#define BB_TOP_I_PLL_CTRL0_RESERVED_RST                          0x0

__INLINE void bb_top_i_pll_ctrl0_pack(uint8_t voutctrlldolo, uint8_t voutctrlldopll, uint8_t voutctrlldovco)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL0_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)voutctrlldolo << 10) | ((uint32_t)voutctrlldopll << 8) | ((uint32_t)voutctrlldovco << 6));
}

__INLINE void bb_top_i_pll_ctrl0_unpack(uint8_t* voutctrlldolo, uint8_t* voutctrlldopll, uint8_t* voutctrlldovco, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR);

    *voutctrlldolo = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *voutctrlldopll = (localVal & ((uint32_t)0x00000300)) >> 8;
    *voutctrlldovco = (localVal & ((uint32_t)0x000000C0)) >> 6;
    *reserved = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t bb_top_i_pll_ctrl0_vout_ctrl_ldo_lo_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void bb_top_i_pll_ctrl0_vout_ctrl_ldo_lo_setf(uint8_t voutctrlldolo)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)voutctrlldolo << 10));
}

__INLINE uint8_t bb_top_i_pll_ctrl0_vout_ctrl_ldo_pll_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void bb_top_i_pll_ctrl0_vout_ctrl_ldo_pll_setf(uint8_t voutctrlldopll)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)voutctrlldopll << 8));
}

__INLINE uint8_t bb_top_i_pll_ctrl0_vout_ctrl_ldo_vco_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000C0)) >> 6);
}

__INLINE void bb_top_i_pll_ctrl0_vout_ctrl_ldo_vco_setf(uint8_t voutctrlldovco)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000C0)) | ((uint32_t)voutctrlldovco << 6));
}

 /**
 * @brief I_PLL_CTRL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24             delay_up   0b0
 *  23:21             delay_dn   0b0
 *  08:05              cp_prog   0b1000
 *     00      en_pllbuf_ctcnt   0
 * </pre>
 */
#define BB_TOP_I_PLL_CTRL1_OFFSET 0x000000D0


__INLINE uint32_t bb_top_i_pll_ctrl1_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_pll_ctrl1_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_PLL_CTRL1_DELAY_UP_MASK                         ((uint32_t)0x07000000)
#define BB_TOP_I_PLL_CTRL1_DELAY_UP_LSB                          24
#define BB_TOP_I_PLL_CTRL1_DELAY_UP_WIDTH                        ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL1_DELAY_DN_MASK                         ((uint32_t)0x00E00000)
#define BB_TOP_I_PLL_CTRL1_DELAY_DN_LSB                          21
#define BB_TOP_I_PLL_CTRL1_DELAY_DN_WIDTH                        ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL1_CP_PROG_MASK                          ((uint32_t)0x000001E0)
#define BB_TOP_I_PLL_CTRL1_CP_PROG_LSB                           5
#define BB_TOP_I_PLL_CTRL1_CP_PROG_WIDTH                         ((uint32_t)0x00000004)
#define BB_TOP_I_PLL_CTRL1_EN_PLLBUF_CTCNT_BIT                   ((uint32_t)0x00000001)
#define BB_TOP_I_PLL_CTRL1_EN_PLLBUF_CTCNT_POS                   0

#define BB_TOP_I_PLL_CTRL1_DELAY_UP_RST                          0x0
#define BB_TOP_I_PLL_CTRL1_DELAY_DN_RST                          0x0
#define BB_TOP_I_PLL_CTRL1_CP_PROG_RST                           0x1000
#define BB_TOP_I_PLL_CTRL1_EN_PLLBUF_CTCNT_RST                   0x0

__INLINE void bb_top_i_pll_ctrl1_pack(uint8_t delayup, uint8_t delaydn, uint8_t cpprog, uint8_t enpllbufctcnt)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL1_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)delayup << 24) | ((uint32_t)delaydn << 21) | ((uint32_t)cpprog << 5) | ((uint32_t)enpllbufctcnt << 0));
}

__INLINE void bb_top_i_pll_ctrl1_unpack(uint8_t* delayup, uint8_t* delaydn, uint8_t* cpprog, uint8_t* enpllbufctcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR);

    *delayup = (localVal & ((uint32_t)0x07000000)) >> 24;
    *delaydn = (localVal & ((uint32_t)0x00E00000)) >> 21;
    *cpprog = (localVal & ((uint32_t)0x000001E0)) >> 5;
    *enpllbufctcnt = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_pll_ctrl1_delay_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void bb_top_i_pll_ctrl1_delay_up_setf(uint8_t delayup)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)delayup << 24));
}

__INLINE uint8_t bb_top_i_pll_ctrl1_delay_dn_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00E00000)) >> 21);
}

__INLINE void bb_top_i_pll_ctrl1_delay_dn_setf(uint8_t delaydn)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00E00000)) | ((uint32_t)delaydn << 21));
}

__INLINE uint8_t bb_top_i_pll_ctrl1_cp_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000001E0)) >> 5);
}

__INLINE void bb_top_i_pll_ctrl1_cp_prog_setf(uint8_t cpprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000001E0)) | ((uint32_t)cpprog << 5));
}

__INLINE uint8_t bb_top_i_pll_ctrl1_en_pllbuf_ctcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_pll_ctrl1_en_pllbuf_ctcnt_setf(uint8_t enpllbufctcnt)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)enpllbufctcnt << 0));
}

 /**
 * @brief I_PLL_CTRL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:21               lpf_rz   0b10
 *  20:18          ib_tpm_prog   0b10
 *  17:16               lpf_r2   0b0
 *  15:14               lpf_r1   0b0
 *  13:12               lpf_c3   0b0
 *  11:10               lpf_c2   0b0
 *  09:08               lpf_c1   0b0
 *  07:04          ib_vco_prog   0b1000
 *  03:00             reserved   0b1010
 * </pre>
 */
#define BB_TOP_I_PLL_CTRL2_OFFSET 0x000000D4


__INLINE uint32_t bb_top_i_pll_ctrl2_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_pll_ctrl2_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_PLL_CTRL2_LPF_RZ_MASK                           ((uint32_t)0x00E00000)
#define BB_TOP_I_PLL_CTRL2_LPF_RZ_LSB                            21
#define BB_TOP_I_PLL_CTRL2_LPF_RZ_WIDTH                          ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL2_IB_TPM_PROG_MASK                      ((uint32_t)0x001C0000)
#define BB_TOP_I_PLL_CTRL2_IB_TPM_PROG_LSB                       18
#define BB_TOP_I_PLL_CTRL2_IB_TPM_PROG_WIDTH                     ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL2_LPF_R2_MASK                           ((uint32_t)0x00030000)
#define BB_TOP_I_PLL_CTRL2_LPF_R2_LSB                            16
#define BB_TOP_I_PLL_CTRL2_LPF_R2_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL2_LPF_R1_MASK                           ((uint32_t)0x0000C000)
#define BB_TOP_I_PLL_CTRL2_LPF_R1_LSB                            14
#define BB_TOP_I_PLL_CTRL2_LPF_R1_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL2_LPF_C3_MASK                           ((uint32_t)0x00003000)
#define BB_TOP_I_PLL_CTRL2_LPF_C3_LSB                            12
#define BB_TOP_I_PLL_CTRL2_LPF_C3_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL2_LPF_C2_MASK                           ((uint32_t)0x00000C00)
#define BB_TOP_I_PLL_CTRL2_LPF_C2_LSB                            10
#define BB_TOP_I_PLL_CTRL2_LPF_C2_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL2_LPF_C1_MASK                           ((uint32_t)0x00000300)
#define BB_TOP_I_PLL_CTRL2_LPF_C1_LSB                            8
#define BB_TOP_I_PLL_CTRL2_LPF_C1_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL2_IB_VCO_PROG_MASK                      ((uint32_t)0x000000F0)
#define BB_TOP_I_PLL_CTRL2_IB_VCO_PROG_LSB                       4
#define BB_TOP_I_PLL_CTRL2_IB_VCO_PROG_WIDTH                     ((uint32_t)0x00000004)
#define BB_TOP_I_PLL_CTRL2_RESERVED_MASK                         ((uint32_t)0x0000000F)
#define BB_TOP_I_PLL_CTRL2_RESERVED_LSB                          0
#define BB_TOP_I_PLL_CTRL2_RESERVED_WIDTH                        ((uint32_t)0x00000004)

#define BB_TOP_I_PLL_CTRL2_LPF_RZ_RST                            0x10
#define BB_TOP_I_PLL_CTRL2_IB_TPM_PROG_RST                       0x10
#define BB_TOP_I_PLL_CTRL2_LPF_R2_RST                            0x0
#define BB_TOP_I_PLL_CTRL2_LPF_R1_RST                            0x0
#define BB_TOP_I_PLL_CTRL2_LPF_C3_RST                            0x0
#define BB_TOP_I_PLL_CTRL2_LPF_C2_RST                            0x0
#define BB_TOP_I_PLL_CTRL2_LPF_C1_RST                            0x0
#define BB_TOP_I_PLL_CTRL2_IB_VCO_PROG_RST                       0x1000
#define BB_TOP_I_PLL_CTRL2_RESERVED_RST                          0x1010

__INLINE void bb_top_i_pll_ctrl2_pack(uint8_t lpfrz, uint8_t ibtpmprog, uint8_t lpfr2, uint8_t lpfr1, uint8_t lpfc3, uint8_t lpfc2, uint8_t lpfc1, uint8_t ibvcoprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)lpfrz << 21) | ((uint32_t)ibtpmprog << 18) | ((uint32_t)lpfr2 << 16) | ((uint32_t)lpfr1 << 14) | ((uint32_t)lpfc3 << 12) | ((uint32_t)lpfc2 << 10) | ((uint32_t)lpfc1 << 8) | ((uint32_t)ibvcoprog << 4));
}

__INLINE void bb_top_i_pll_ctrl2_unpack(uint8_t* lpfrz, uint8_t* ibtpmprog, uint8_t* lpfr2, uint8_t* lpfr1, uint8_t* lpfc3, uint8_t* lpfc2, uint8_t* lpfc1, uint8_t* ibvcoprog, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);

    *lpfrz = (localVal & ((uint32_t)0x00E00000)) >> 21;
    *ibtpmprog = (localVal & ((uint32_t)0x001C0000)) >> 18;
    *lpfr2 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *lpfr1 = (localVal & ((uint32_t)0x0000C000)) >> 14;
    *lpfc3 = (localVal & ((uint32_t)0x00003000)) >> 12;
    *lpfc2 = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *lpfc1 = (localVal & ((uint32_t)0x00000300)) >> 8;
    *ibvcoprog = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *reserved = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bb_top_i_pll_ctrl2_lpf_rz_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00E00000)) >> 21);
}

__INLINE void bb_top_i_pll_ctrl2_lpf_rz_setf(uint8_t lpfrz)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00E00000)) | ((uint32_t)lpfrz << 21));
}

__INLINE uint8_t bb_top_i_pll_ctrl2_ib_tpm_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x001C0000)) >> 18);
}

__INLINE void bb_top_i_pll_ctrl2_ib_tpm_prog_setf(uint8_t ibtpmprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x001C0000)) | ((uint32_t)ibtpmprog << 18));
}

__INLINE uint8_t bb_top_i_pll_ctrl2_lpf_r2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bb_top_i_pll_ctrl2_lpf_r2_setf(uint8_t lpfr2)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)lpfr2 << 16));
}

__INLINE uint8_t bb_top_i_pll_ctrl2_lpf_r1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void bb_top_i_pll_ctrl2_lpf_r1_setf(uint8_t lpfr1)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)lpfr1 << 14));
}

__INLINE uint8_t bb_top_i_pll_ctrl2_lpf_c3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void bb_top_i_pll_ctrl2_lpf_c3_setf(uint8_t lpfc3)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)lpfc3 << 12));
}

__INLINE uint8_t bb_top_i_pll_ctrl2_lpf_c2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void bb_top_i_pll_ctrl2_lpf_c2_setf(uint8_t lpfc2)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)lpfc2 << 10));
}

__INLINE uint8_t bb_top_i_pll_ctrl2_lpf_c1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void bb_top_i_pll_ctrl2_lpf_c1_setf(uint8_t lpfc1)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)lpfc1 << 8));
}

__INLINE uint8_t bb_top_i_pll_ctrl2_ib_vco_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bb_top_i_pll_ctrl2_ib_vco_prog_setf(uint8_t ibvcoprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL2_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL2_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)ibvcoprog << 4));
}

 /**
 * @brief I_PLL_CTRL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24            vmid_prog   0b100
 *  20:18     vco_var_tpm_prog   0b100
 *  17:15         vco_var_prog   0b100
 *     10              pfd_pol   0
 *  09:00             Reserved   0b0
 * </pre>
 */
#define BB_TOP_I_PLL_CTRL3_OFFSET 0x000000D8


__INLINE uint32_t bb_top_i_pll_ctrl3_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_pll_ctrl3_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_PLL_CTRL3_VMID_PROG_MASK                        ((uint32_t)0x07000000)
#define BB_TOP_I_PLL_CTRL3_VMID_PROG_LSB                         24
#define BB_TOP_I_PLL_CTRL3_VMID_PROG_WIDTH                       ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_TPM_PROG_MASK                 ((uint32_t)0x001C0000)
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_TPM_PROG_LSB                  18
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_TPM_PROG_WIDTH                ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_PROG_MASK                     ((uint32_t)0x00038000)
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_PROG_LSB                      15
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_PROG_WIDTH                    ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL3_PFD_POL_BIT                           ((uint32_t)0x00000400)
#define BB_TOP_I_PLL_CTRL3_PFD_POL_POS                           10
#define BB_TOP_I_PLL_CTRL3_RESERVED_MASK                         ((uint32_t)0x000003FF)
#define BB_TOP_I_PLL_CTRL3_RESERVED_LSB                          0
#define BB_TOP_I_PLL_CTRL3_RESERVED_WIDTH                        ((uint32_t)0x0000000A)

#define BB_TOP_I_PLL_CTRL3_VMID_PROG_RST                         0x100
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_TPM_PROG_RST                  0x100
#define BB_TOP_I_PLL_CTRL3_VCO_VAR_PROG_RST                      0x100
#define BB_TOP_I_PLL_CTRL3_PFD_POL_RST                           0x0
#define BB_TOP_I_PLL_CTRL3_RESERVED_RST                          0x0

__INLINE void bb_top_i_pll_ctrl3_pack(uint8_t vmidprog, uint8_t vcovartpmprog, uint8_t vcovarprog, uint8_t pfdpol)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL3_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)vmidprog << 24) | ((uint32_t)vcovartpmprog << 18) | ((uint32_t)vcovarprog << 15) | ((uint32_t)pfdpol << 10));
}

__INLINE void bb_top_i_pll_ctrl3_unpack(uint8_t* vmidprog, uint8_t* vcovartpmprog, uint8_t* vcovarprog, uint8_t* pfdpol, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR);

    *vmidprog = (localVal & ((uint32_t)0x07000000)) >> 24;
    *vcovartpmprog = (localVal & ((uint32_t)0x001C0000)) >> 18;
    *vcovarprog = (localVal & ((uint32_t)0x00038000)) >> 15;
    *pfdpol = (localVal & ((uint32_t)0x00000400)) >> 10;
    *reserved = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint8_t bb_top_i_pll_ctrl3_vmid_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void bb_top_i_pll_ctrl3_vmid_prog_setf(uint8_t vmidprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)vmidprog << 24));
}

__INLINE uint8_t bb_top_i_pll_ctrl3_vco_var_tpm_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x001C0000)) >> 18);
}

__INLINE void bb_top_i_pll_ctrl3_vco_var_tpm_prog_setf(uint8_t vcovartpmprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x001C0000)) | ((uint32_t)vcovartpmprog << 18));
}

__INLINE uint8_t bb_top_i_pll_ctrl3_vco_var_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00038000)) >> 15);
}

__INLINE void bb_top_i_pll_ctrl3_vco_var_prog_setf(uint8_t vcovarprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00038000)) | ((uint32_t)vcovarprog << 15));
}

__INLINE uint8_t bb_top_i_pll_ctrl3_pfd_pol_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bb_top_i_pll_ctrl3_pfd_pol_setf(uint8_t pfdpol)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL3_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL3_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)pfdpol << 10));
}

 /**
 * @brief I_RX_CTRL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     22   en_rx_test_buffer_1p2v_1   0
 *  21:19      rx_lna_cap_trim   0b100
 *     18               TIA_en   1
 *     17      rx_en_dcoc_1p2v   1
 *     16   en_rx_test_buffer_1p2v_0   0
 *     15        rx_tia_i_ctrl   0
 *     14           en_pa_1p2v   1
 *     13        en_mixer_1p2v   1
 *     12          en_lna_1p2v   1
 *     11       en_filter_1p2v   1
 *     10       en_rx_adc_1p2v   0
 *  06:05   ldo_lna_out_tune_1p2v   0b1
 *  04:03   ldo_ana_bb_out_tune_1p2v   0b1
 *     02       rx_bb_vcm_ctrl   1
 *     01   en_ldo_lna_pa_1p2v   1
 *     00   en_ldo_ana_bb_1p2v   1
 * </pre>
 */
#define BB_TOP_I_RX_CTRL0_OFFSET 0x000000DC


__INLINE uint32_t bb_top_i_rx_ctrl0_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_rx_ctrl0_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_RX_CTRL0_EN_RX_TEST_BUFFER_1P2V_1_BIT          ((uint32_t)0x00400000)
#define BB_TOP_I_RX_CTRL0_EN_RX_TEST_BUFFER_1P2V_1_POS          22
#define BB_TOP_I_RX_CTRL0_RX_LNA_CAP_TRIM_MASK                  ((uint32_t)0x00380000)
#define BB_TOP_I_RX_CTRL0_RX_LNA_CAP_TRIM_LSB                   19
#define BB_TOP_I_RX_CTRL0_RX_LNA_CAP_TRIM_WIDTH                 ((uint32_t)0x00000003)
#define BB_TOP_I_RX_CTRL0_TIA_EN_BIT                            ((uint32_t)0x00040000)
#define BB_TOP_I_RX_CTRL0_TIA_EN_POS                            18
#define BB_TOP_I_RX_CTRL0_RX_EN_DCOC_1P2V_BIT                   ((uint32_t)0x00020000)
#define BB_TOP_I_RX_CTRL0_RX_EN_DCOC_1P2V_POS                   17
#define BB_TOP_I_RX_CTRL0_EN_RX_TEST_BUFFER_1P2V_0_BIT          ((uint32_t)0x00010000)
#define BB_TOP_I_RX_CTRL0_EN_RX_TEST_BUFFER_1P2V_0_POS          16
#define BB_TOP_I_RX_CTRL0_RX_TIA_I_CTRL_BIT                     ((uint32_t)0x00008000)
#define BB_TOP_I_RX_CTRL0_RX_TIA_I_CTRL_POS                     15
#define BB_TOP_I_RX_CTRL0_EN_PA_1P2V_BIT                        ((uint32_t)0x00004000)
#define BB_TOP_I_RX_CTRL0_EN_PA_1P2V_POS                        14
#define BB_TOP_I_RX_CTRL0_EN_MIXER_1P2V_BIT                     ((uint32_t)0x00002000)
#define BB_TOP_I_RX_CTRL0_EN_MIXER_1P2V_POS                     13
#define BB_TOP_I_RX_CTRL0_EN_LNA_1P2V_BIT                       ((uint32_t)0x00001000)
#define BB_TOP_I_RX_CTRL0_EN_LNA_1P2V_POS                       12
#define BB_TOP_I_RX_CTRL0_EN_FILTER_1P2V_BIT                    ((uint32_t)0x00000800)
#define BB_TOP_I_RX_CTRL0_EN_FILTER_1P2V_POS                    11
#define BB_TOP_I_RX_CTRL0_EN_RX_ADC_1P2V_BIT                    ((uint32_t)0x00000400)
#define BB_TOP_I_RX_CTRL0_EN_RX_ADC_1P2V_POS                    10
#define BB_TOP_I_RX_CTRL0_LDO_LNA_OUT_TUNE_1P2V_MASK            ((uint32_t)0x00000060)
#define BB_TOP_I_RX_CTRL0_LDO_LNA_OUT_TUNE_1P2V_LSB             5
#define BB_TOP_I_RX_CTRL0_LDO_LNA_OUT_TUNE_1P2V_WIDTH           ((uint32_t)0x00000002)
#define BB_TOP_I_RX_CTRL0_LDO_ANA_BB_OUT_TUNE_1P2V_MASK         ((uint32_t)0x00000018)
#define BB_TOP_I_RX_CTRL0_LDO_ANA_BB_OUT_TUNE_1P2V_LSB          3
#define BB_TOP_I_RX_CTRL0_LDO_ANA_BB_OUT_TUNE_1P2V_WIDTH        ((uint32_t)0x00000002)
#define BB_TOP_I_RX_CTRL0_RX_BB_VCM_CTRL_BIT                    ((uint32_t)0x00000004)
#define BB_TOP_I_RX_CTRL0_RX_BB_VCM_CTRL_POS                    2
#define BB_TOP_I_RX_CTRL0_EN_LDO_LNA_PA_1P2V_BIT                ((uint32_t)0x00000002)
#define BB_TOP_I_RX_CTRL0_EN_LDO_LNA_PA_1P2V_POS                1
#define BB_TOP_I_RX_CTRL0_EN_LDO_ANA_BB_1P2V_BIT                ((uint32_t)0x00000001)
#define BB_TOP_I_RX_CTRL0_EN_LDO_ANA_BB_1P2V_POS                0

#define BB_TOP_I_RX_CTRL0_EN_RX_TEST_BUFFER_1P2V_1_RST          0x0
#define BB_TOP_I_RX_CTRL0_RX_LNA_CAP_TRIM_RST                   0x100
#define BB_TOP_I_RX_CTRL0_TIA_EN_RST                            0x1
#define BB_TOP_I_RX_CTRL0_RX_EN_DCOC_1P2V_RST                   0x1
#define BB_TOP_I_RX_CTRL0_EN_RX_TEST_BUFFER_1P2V_0_RST          0x0
#define BB_TOP_I_RX_CTRL0_RX_TIA_I_CTRL_RST                     0x0
#define BB_TOP_I_RX_CTRL0_EN_PA_1P2V_RST                        0x1
#define BB_TOP_I_RX_CTRL0_EN_MIXER_1P2V_RST                     0x1
#define BB_TOP_I_RX_CTRL0_EN_LNA_1P2V_RST                       0x1
#define BB_TOP_I_RX_CTRL0_EN_FILTER_1P2V_RST                    0x1
#define BB_TOP_I_RX_CTRL0_EN_RX_ADC_1P2V_RST                    0x0
#define BB_TOP_I_RX_CTRL0_LDO_LNA_OUT_TUNE_1P2V_RST             0x1
#define BB_TOP_I_RX_CTRL0_LDO_ANA_BB_OUT_TUNE_1P2V_RST          0x1
#define BB_TOP_I_RX_CTRL0_RX_BB_VCM_CTRL_RST                    0x1
#define BB_TOP_I_RX_CTRL0_EN_LDO_LNA_PA_1P2V_RST                0x1
#define BB_TOP_I_RX_CTRL0_EN_LDO_ANA_BB_1P2V_RST                0x1

__INLINE void bb_top_i_rx_ctrl0_pack(uint8_t enrxtestbuffer1p2v1, uint8_t rxlnacaptrim, uint8_t tiaen, uint8_t rxendcoc1p2v, uint8_t enrxtestbuffer1p2v0, uint8_t rxtiaictrl, uint8_t enpa1p2v, uint8_t enmixer1p2v, uint8_t enlna1p2v, uint8_t enfilter1p2v, uint8_t enrxadc1p2v, uint8_t ldolnaouttune1p2v, uint8_t ldoanabbouttune1p2v, uint8_t rxbbvcmctrl, uint8_t enldolnapa1p2v, uint8_t enldoanabb1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)enrxtestbuffer1p2v1 << 22) | ((uint32_t)rxlnacaptrim << 19) | ((uint32_t)tiaen << 18) | ((uint32_t)rxendcoc1p2v << 17) | ((uint32_t)enrxtestbuffer1p2v0 << 16) | ((uint32_t)rxtiaictrl << 15) | ((uint32_t)enpa1p2v << 14) | ((uint32_t)enmixer1p2v << 13) | ((uint32_t)enlna1p2v << 12) | ((uint32_t)enfilter1p2v << 11) | ((uint32_t)enrxadc1p2v << 10) | ((uint32_t)ldolnaouttune1p2v << 5) | ((uint32_t)ldoanabbouttune1p2v << 3) | ((uint32_t)rxbbvcmctrl << 2) | ((uint32_t)enldolnapa1p2v << 1) | ((uint32_t)enldoanabb1p2v << 0));
}

__INLINE void bb_top_i_rx_ctrl0_unpack(uint8_t* enrxtestbuffer1p2v1, uint8_t* rxlnacaptrim, uint8_t* tiaen, uint8_t* rxendcoc1p2v, uint8_t* enrxtestbuffer1p2v0, uint8_t* rxtiaictrl, uint8_t* enpa1p2v, uint8_t* enmixer1p2v, uint8_t* enlna1p2v, uint8_t* enfilter1p2v, uint8_t* enrxadc1p2v, uint8_t* ldolnaouttune1p2v, uint8_t* ldoanabbouttune1p2v, uint8_t* rxbbvcmctrl, uint8_t* enldolnapa1p2v, uint8_t* enldoanabb1p2v)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);

    *enrxtestbuffer1p2v1 = (localVal & ((uint32_t)0x00400000)) >> 22;
    *rxlnacaptrim = (localVal & ((uint32_t)0x00380000)) >> 19;
    *tiaen = (localVal & ((uint32_t)0x00040000)) >> 18;
    *rxendcoc1p2v = (localVal & ((uint32_t)0x00020000)) >> 17;
    *enrxtestbuffer1p2v0 = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rxtiaictrl = (localVal & ((uint32_t)0x00008000)) >> 15;
    *enpa1p2v = (localVal & ((uint32_t)0x00004000)) >> 14;
    *enmixer1p2v = (localVal & ((uint32_t)0x00002000)) >> 13;
    *enlna1p2v = (localVal & ((uint32_t)0x00001000)) >> 12;
    *enfilter1p2v = (localVal & ((uint32_t)0x00000800)) >> 11;
    *enrxadc1p2v = (localVal & ((uint32_t)0x00000400)) >> 10;
    *ldolnaouttune1p2v = (localVal & ((uint32_t)0x00000060)) >> 5;
    *ldoanabbouttune1p2v = (localVal & ((uint32_t)0x00000018)) >> 3;
    *rxbbvcmctrl = (localVal & ((uint32_t)0x00000004)) >> 2;
    *enldolnapa1p2v = (localVal & ((uint32_t)0x00000002)) >> 1;
    *enldoanabb1p2v = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_rx_test_buffer_1p2v_1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bb_top_i_rx_ctrl0_en_rx_test_buffer_1p2v_1_setf(uint8_t enrxtestbuffer1p2v1)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)enrxtestbuffer1p2v1 << 22));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_rx_lna_cap_trim_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00380000)) >> 19);
}

__INLINE void bb_top_i_rx_ctrl0_rx_lna_cap_trim_setf(uint8_t rxlnacaptrim)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00380000)) | ((uint32_t)rxlnacaptrim << 19));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_tia_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00040000)) >> 18);
}

__INLINE void bb_top_i_rx_ctrl0_tia_en_setf(uint8_t tiaen)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00040000)) | ((uint32_t)tiaen << 18));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_rx_en_dcoc_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00020000)) >> 17);
}

__INLINE void bb_top_i_rx_ctrl0_rx_en_dcoc_1p2v_setf(uint8_t rxendcoc1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00020000)) | ((uint32_t)rxendcoc1p2v << 17));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_rx_test_buffer_1p2v_0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bb_top_i_rx_ctrl0_en_rx_test_buffer_1p2v_0_setf(uint8_t enrxtestbuffer1p2v0)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)enrxtestbuffer1p2v0 << 16));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_rx_tia_i_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE void bb_top_i_rx_ctrl0_rx_tia_i_ctrl_setf(uint8_t rxtiaictrl)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00008000)) | ((uint32_t)rxtiaictrl << 15));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_pa_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE void bb_top_i_rx_ctrl0_en_pa_1p2v_setf(uint8_t enpa1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00004000)) | ((uint32_t)enpa1p2v << 14));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_mixer_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bb_top_i_rx_ctrl0_en_mixer_1p2v_setf(uint8_t enmixer1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)enmixer1p2v << 13));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_lna_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bb_top_i_rx_ctrl0_en_lna_1p2v_setf(uint8_t enlna1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)enlna1p2v << 12));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_filter_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void bb_top_i_rx_ctrl0_en_filter_1p2v_setf(uint8_t enfilter1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)enfilter1p2v << 11));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_rx_adc_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bb_top_i_rx_ctrl0_en_rx_adc_1p2v_setf(uint8_t enrxadc1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)enrxadc1p2v << 10));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_ldo_lna_out_tune_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000060)) >> 5);
}

__INLINE void bb_top_i_rx_ctrl0_ldo_lna_out_tune_1p2v_setf(uint8_t ldolnaouttune1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000060)) | ((uint32_t)ldolnaouttune1p2v << 5));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_ldo_ana_bb_out_tune_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000018)) >> 3);
}

__INLINE void bb_top_i_rx_ctrl0_ldo_ana_bb_out_tune_1p2v_setf(uint8_t ldoanabbouttune1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000018)) | ((uint32_t)ldoanabbouttune1p2v << 3));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_rx_bb_vcm_ctrl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void bb_top_i_rx_ctrl0_rx_bb_vcm_ctrl_setf(uint8_t rxbbvcmctrl)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rxbbvcmctrl << 2));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_ldo_lna_pa_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void bb_top_i_rx_ctrl0_en_ldo_lna_pa_1p2v_setf(uint8_t enldolnapa1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)enldolnapa1p2v << 1));
}

__INLINE uint8_t bb_top_i_rx_ctrl0_en_ldo_ana_bb_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_rx_ctrl0_en_ldo_ana_bb_1p2v_setf(uint8_t enldoanabb1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL0_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL0_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)enldoanabb1p2v << 0));
}

 /**
 * @brief I_RX_CTRL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     25    rccal_discharge_c   0
 *     24        rccal_dcoc_en   0
 *     23   rccal_comparator_clk   0
 *     22       rccal_charge_c   0
 *  21:17         rccal_cap_sw   0b0
 *     16          rx_rccal_en   0
 *     13   rx_filter_swap_iq_1p2v   0
 *     12   rx_filter_bw_sel_1p2v   0
 *  09:06   filter_cap_sw_1p2v   0b1000
 *  05:00             reserved   0b0
 * </pre>
 */
#define BB_TOP_I_RX_CTRL1_OFFSET 0x000000E0


__INLINE uint32_t bb_top_i_rx_ctrl1_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_rx_ctrl1_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_RX_CTRL1_RCCAL_DISCHARGE_C_BIT                 ((uint32_t)0x02000000)
#define BB_TOP_I_RX_CTRL1_RCCAL_DISCHARGE_C_POS                 25
#define BB_TOP_I_RX_CTRL1_RCCAL_DCOC_EN_BIT                     ((uint32_t)0x01000000)
#define BB_TOP_I_RX_CTRL1_RCCAL_DCOC_EN_POS                     24
#define BB_TOP_I_RX_CTRL1_RCCAL_COMPARATOR_CLK_BIT              ((uint32_t)0x00800000)
#define BB_TOP_I_RX_CTRL1_RCCAL_COMPARATOR_CLK_POS              23
#define BB_TOP_I_RX_CTRL1_RCCAL_CHARGE_C_BIT                    ((uint32_t)0x00400000)
#define BB_TOP_I_RX_CTRL1_RCCAL_CHARGE_C_POS                    22
#define BB_TOP_I_RX_CTRL1_RCCAL_CAP_SW_MASK                     ((uint32_t)0x003E0000)
#define BB_TOP_I_RX_CTRL1_RCCAL_CAP_SW_LSB                      17
#define BB_TOP_I_RX_CTRL1_RCCAL_CAP_SW_WIDTH                    ((uint32_t)0x00000005)
#define BB_TOP_I_RX_CTRL1_RX_RCCAL_EN_BIT                       ((uint32_t)0x00010000)
#define BB_TOP_I_RX_CTRL1_RX_RCCAL_EN_POS                       16
#define BB_TOP_I_RX_CTRL1_RX_FILTER_SWAP_IQ_1P2V_BIT            ((uint32_t)0x00002000)
#define BB_TOP_I_RX_CTRL1_RX_FILTER_SWAP_IQ_1P2V_POS            13
#define BB_TOP_I_RX_CTRL1_RX_FILTER_BW_SEL_1P2V_BIT             ((uint32_t)0x00001000)
#define BB_TOP_I_RX_CTRL1_RX_FILTER_BW_SEL_1P2V_POS             12
#define BB_TOP_I_RX_CTRL1_FILTER_CAP_SW_1P2V_MASK               ((uint32_t)0x000003C0)
#define BB_TOP_I_RX_CTRL1_FILTER_CAP_SW_1P2V_LSB                6
#define BB_TOP_I_RX_CTRL1_FILTER_CAP_SW_1P2V_WIDTH              ((uint32_t)0x00000004)
#define BB_TOP_I_RX_CTRL1_RESERVED_MASK                         ((uint32_t)0x0000003F)
#define BB_TOP_I_RX_CTRL1_RESERVED_LSB                          0
#define BB_TOP_I_RX_CTRL1_RESERVED_WIDTH                        ((uint32_t)0x00000006)

#define BB_TOP_I_RX_CTRL1_RCCAL_DISCHARGE_C_RST                 0x0
#define BB_TOP_I_RX_CTRL1_RCCAL_DCOC_EN_RST                     0x0
#define BB_TOP_I_RX_CTRL1_RCCAL_COMPARATOR_CLK_RST              0x0
#define BB_TOP_I_RX_CTRL1_RCCAL_CHARGE_C_RST                    0x0
#define BB_TOP_I_RX_CTRL1_RCCAL_CAP_SW_RST                      0x0
#define BB_TOP_I_RX_CTRL1_RX_RCCAL_EN_RST                       0x0
#define BB_TOP_I_RX_CTRL1_RX_FILTER_SWAP_IQ_1P2V_RST            0x0
#define BB_TOP_I_RX_CTRL1_RX_FILTER_BW_SEL_1P2V_RST             0x0
#define BB_TOP_I_RX_CTRL1_FILTER_CAP_SW_1P2V_RST                0x1000
#define BB_TOP_I_RX_CTRL1_RESERVED_RST                          0x0

__INLINE void bb_top_i_rx_ctrl1_pack(uint8_t rccaldischargec, uint8_t rccaldcocen, uint8_t rccalcomparatorclk, uint8_t rccalchargec, uint8_t rccalcapsw, uint8_t rxrccalen, uint8_t rxfilterswapiq1p2v, uint8_t rxfilterbwsel1p2v, uint8_t filtercapsw1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)rccaldischargec << 25) | ((uint32_t)rccaldcocen << 24) | ((uint32_t)rccalcomparatorclk << 23) | ((uint32_t)rccalchargec << 22) | ((uint32_t)rccalcapsw << 17) | ((uint32_t)rxrccalen << 16) | ((uint32_t)rxfilterswapiq1p2v << 13) | ((uint32_t)rxfilterbwsel1p2v << 12) | ((uint32_t)filtercapsw1p2v << 6));
}

__INLINE void bb_top_i_rx_ctrl1_unpack(uint8_t* rccaldischargec, uint8_t* rccaldcocen, uint8_t* rccalcomparatorclk, uint8_t* rccalchargec, uint8_t* rccalcapsw, uint8_t* rxrccalen, uint8_t* rxfilterswapiq1p2v, uint8_t* rxfilterbwsel1p2v, uint8_t* filtercapsw1p2v, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);

    *rccaldischargec = (localVal & ((uint32_t)0x02000000)) >> 25;
    *rccaldcocen = (localVal & ((uint32_t)0x01000000)) >> 24;
    *rccalcomparatorclk = (localVal & ((uint32_t)0x00800000)) >> 23;
    *rccalchargec = (localVal & ((uint32_t)0x00400000)) >> 22;
    *rccalcapsw = (localVal & ((uint32_t)0x003E0000)) >> 17;
    *rxrccalen = (localVal & ((uint32_t)0x00010000)) >> 16;
    *rxfilterswapiq1p2v = (localVal & ((uint32_t)0x00002000)) >> 13;
    *rxfilterbwsel1p2v = (localVal & ((uint32_t)0x00001000)) >> 12;
    *filtercapsw1p2v = (localVal & ((uint32_t)0x000003C0)) >> 6;
    *reserved = (localVal & ((uint32_t)0x0000003F)) >> 0;
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rccal_discharge_c_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x02000000)) >> 25);
}

__INLINE void bb_top_i_rx_ctrl1_rccal_discharge_c_setf(uint8_t rccaldischargec)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x02000000)) | ((uint32_t)rccaldischargec << 25));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rccal_dcoc_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x01000000)) >> 24);
}

__INLINE void bb_top_i_rx_ctrl1_rccal_dcoc_en_setf(uint8_t rccaldcocen)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x01000000)) | ((uint32_t)rccaldcocen << 24));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rccal_comparator_clk_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00800000)) >> 23);
}

__INLINE void bb_top_i_rx_ctrl1_rccal_comparator_clk_setf(uint8_t rccalcomparatorclk)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00800000)) | ((uint32_t)rccalcomparatorclk << 23));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rccal_charge_c_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE void bb_top_i_rx_ctrl1_rccal_charge_c_setf(uint8_t rccalchargec)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00400000)) | ((uint32_t)rccalchargec << 22));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rccal_cap_sw_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x003E0000)) >> 17);
}

__INLINE void bb_top_i_rx_ctrl1_rccal_cap_sw_setf(uint8_t rccalcapsw)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x003E0000)) | ((uint32_t)rccalcapsw << 17));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rx_rccal_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void bb_top_i_rx_ctrl1_rx_rccal_en_setf(uint8_t rxrccalen)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)rxrccalen << 16));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rx_filter_swap_iq_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void bb_top_i_rx_ctrl1_rx_filter_swap_iq_1p2v_setf(uint8_t rxfilterswapiq1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)rxfilterswapiq1p2v << 13));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_rx_filter_bw_sel_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void bb_top_i_rx_ctrl1_rx_filter_bw_sel_1p2v_setf(uint8_t rxfilterbwsel1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)rxfilterbwsel1p2v << 12));
}

__INLINE uint8_t bb_top_i_rx_ctrl1_filter_cap_sw_1p2v_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000003C0)) >> 6);
}

__INLINE void bb_top_i_rx_ctrl1_filter_cap_sw_1p2v_setf(uint8_t filtercapsw1p2v)
{
    _PICO_REG_WR(BB_TOP_I_RX_CTRL1_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_RX_CTRL1_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000003C0)) | ((uint32_t)filtercapsw1p2v << 6));
}

 /**
 * @brief O_DIG_BB_CTRL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24      sync_astate_out   0b0
 *  23:22                   ri   0b0
 *  20:16       rx_demod_check   0b0
 *  09:00         freq_est_raw   0b0
 * </pre>
 */
#define BB_TOP_O_DIG_BB_CTRL0_OFFSET 0x000000E4


__INLINE uint32_t bb_top_o_dig_bb_ctrl0_get(void)
{
    return _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
}

// field definitions
#define BB_TOP_O_DIG_BB_CTRL0_SYNC_ASTATE_OUT_MASK                  ((uint32_t)0xFF000000)
#define BB_TOP_O_DIG_BB_CTRL0_SYNC_ASTATE_OUT_LSB                   24
#define BB_TOP_O_DIG_BB_CTRL0_SYNC_ASTATE_OUT_WIDTH                 ((uint32_t)0x00000008)
#define BB_TOP_O_DIG_BB_CTRL0_RI_MASK                               ((uint32_t)0x00C00000)
#define BB_TOP_O_DIG_BB_CTRL0_RI_LSB                                22
#define BB_TOP_O_DIG_BB_CTRL0_RI_WIDTH                              ((uint32_t)0x00000002)
#define BB_TOP_O_DIG_BB_CTRL0_RX_DEMOD_CHECK_MASK                   ((uint32_t)0x001F0000)
#define BB_TOP_O_DIG_BB_CTRL0_RX_DEMOD_CHECK_LSB                    16
#define BB_TOP_O_DIG_BB_CTRL0_RX_DEMOD_CHECK_WIDTH                  ((uint32_t)0x00000005)
#define BB_TOP_O_DIG_BB_CTRL0_FREQ_EST_RAW_MASK                     ((uint32_t)0x000003FF)
#define BB_TOP_O_DIG_BB_CTRL0_FREQ_EST_RAW_LSB                      0
#define BB_TOP_O_DIG_BB_CTRL0_FREQ_EST_RAW_WIDTH                    ((uint32_t)0x0000000A)

#define BB_TOP_O_DIG_BB_CTRL0_SYNC_ASTATE_OUT_RST                   0x0
#define BB_TOP_O_DIG_BB_CTRL0_RI_RST                                0x0
#define BB_TOP_O_DIG_BB_CTRL0_RX_DEMOD_CHECK_RST                    0x0
#define BB_TOP_O_DIG_BB_CTRL0_FREQ_EST_RAW_RST                      0x0

__INLINE void bb_top_o_dig_bb_ctrl0_unpack(uint8_t* syncastateout, uint8_t* ri, uint8_t* rxdemodcheck, uint8_t* freqestraw)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);

    *syncastateout = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *ri = (localVal & ((uint32_t)0x00C00000)) >> 22;
    *rxdemodcheck = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *freqestraw = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl0_sync_astate_out_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl0_ri_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00C00000)) >> 22);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl0_rx_demod_check_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE uint16_t bb_top_o_dig_bb_ctrl0_freq_est_raw_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL0_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

 /**
 * @brief O_DIG_BB_CTRL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24                nrssi   0b0
 *  23:16                arssi   0b0
 *  15:08                rxabs   0b0
 *  07:04       rx_demod_check   0b0
 *  03:00             agc_gIdx   0b0
 * </pre>
 */
#define BB_TOP_O_DIG_BB_CTRL1_OFFSET 0x000000E8


__INLINE uint32_t bb_top_o_dig_bb_ctrl1_get(void)
{
    return _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
}

// field definitions
#define BB_TOP_O_DIG_BB_CTRL1_NRSSI_MASK                            ((uint32_t)0xFF000000)
#define BB_TOP_O_DIG_BB_CTRL1_NRSSI_LSB                             24
#define BB_TOP_O_DIG_BB_CTRL1_NRSSI_WIDTH                           ((uint32_t)0x00000008)
#define BB_TOP_O_DIG_BB_CTRL1_ARSSI_MASK                            ((uint32_t)0x00FF0000)
#define BB_TOP_O_DIG_BB_CTRL1_ARSSI_LSB                             16
#define BB_TOP_O_DIG_BB_CTRL1_ARSSI_WIDTH                           ((uint32_t)0x00000008)
#define BB_TOP_O_DIG_BB_CTRL1_RXABS_MASK                            ((uint32_t)0x0000FF00)
#define BB_TOP_O_DIG_BB_CTRL1_RXABS_LSB                             8
#define BB_TOP_O_DIG_BB_CTRL1_RXABS_WIDTH                           ((uint32_t)0x00000008)
#define BB_TOP_O_DIG_BB_CTRL1_RX_DEMOD_CHECK_MASK                   ((uint32_t)0x000000F0)
#define BB_TOP_O_DIG_BB_CTRL1_RX_DEMOD_CHECK_LSB                    4
#define BB_TOP_O_DIG_BB_CTRL1_RX_DEMOD_CHECK_WIDTH                  ((uint32_t)0x00000004)
#define BB_TOP_O_DIG_BB_CTRL1_AGC_G_IDX_MASK                        ((uint32_t)0x0000000F)
#define BB_TOP_O_DIG_BB_CTRL1_AGC_G_IDX_LSB                         0
#define BB_TOP_O_DIG_BB_CTRL1_AGC_G_IDX_WIDTH                       ((uint32_t)0x00000004)

#define BB_TOP_O_DIG_BB_CTRL1_NRSSI_RST                             0x0
#define BB_TOP_O_DIG_BB_CTRL1_ARSSI_RST                             0x0
#define BB_TOP_O_DIG_BB_CTRL1_RXABS_RST                             0x0
#define BB_TOP_O_DIG_BB_CTRL1_RX_DEMOD_CHECK_RST                    0x0
#define BB_TOP_O_DIG_BB_CTRL1_AGC_G_IDX_RST                         0x0

__INLINE void bb_top_o_dig_bb_ctrl1_unpack(uint8_t* nrssi, uint8_t* arssi, uint8_t* rxabs, uint8_t* rxdemodcheck, uint8_t* agcgidx)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);

    *nrssi = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *arssi = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *rxabs = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rxdemodcheck = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *agcgidx = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl1_nrssi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl1_arssi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl1_rxabs_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl1_rx_demod_check_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl1_agc_g_idx_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL1_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief O_DIG_BB_CTRL2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:30         dc_cal_state   0b0
 *  29:24        dc_comp_dac_q   0b20
 *  21:16        dc_comp_dac_i   0b20
 *  15:08               dcqVec   0b0
 *  07:00               dciVec   0b0
 * </pre>
 */
#define BB_TOP_O_DIG_BB_CTRL2_OFFSET 0x000000EC


__INLINE uint32_t bb_top_o_dig_bb_ctrl2_get(void)
{
    return _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
}

// field definitions
#define BB_TOP_O_DIG_BB_CTRL2_DC_CAL_STATE_MASK                     ((uint32_t)0xC0000000)
#define BB_TOP_O_DIG_BB_CTRL2_DC_CAL_STATE_LSB                      30
#define BB_TOP_O_DIG_BB_CTRL2_DC_CAL_STATE_WIDTH                    ((uint32_t)0x00000002)
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_Q_MASK                    ((uint32_t)0x3F000000)
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_Q_LSB                     24
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_Q_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_I_MASK                    ((uint32_t)0x003F0000)
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_I_LSB                     16
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_I_WIDTH                   ((uint32_t)0x00000006)
#define BB_TOP_O_DIG_BB_CTRL2_DCQ_VEC_MASK                          ((uint32_t)0x0000FF00)
#define BB_TOP_O_DIG_BB_CTRL2_DCQ_VEC_LSB                           8
#define BB_TOP_O_DIG_BB_CTRL2_DCQ_VEC_WIDTH                         ((uint32_t)0x00000008)
#define BB_TOP_O_DIG_BB_CTRL2_DCI_VEC_MASK                          ((uint32_t)0x000000FF)
#define BB_TOP_O_DIG_BB_CTRL2_DCI_VEC_LSB                           0
#define BB_TOP_O_DIG_BB_CTRL2_DCI_VEC_WIDTH                         ((uint32_t)0x00000008)

#define BB_TOP_O_DIG_BB_CTRL2_DC_CAL_STATE_RST                      0x0
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_Q_RST                     0x20
#define BB_TOP_O_DIG_BB_CTRL2_DC_COMP_DAC_I_RST                     0x20
#define BB_TOP_O_DIG_BB_CTRL2_DCQ_VEC_RST                           0x0
#define BB_TOP_O_DIG_BB_CTRL2_DCI_VEC_RST                           0x0

__INLINE void bb_top_o_dig_bb_ctrl2_unpack(uint8_t* dccalstate, uint8_t* dccompdacq, uint8_t* dccompdaci, uint8_t* dcqvec, uint8_t* dcivec)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);

    *dccalstate = (localVal & ((uint32_t)0xC0000000)) >> 30;
    *dccompdacq = (localVal & ((uint32_t)0x3F000000)) >> 24;
    *dccompdaci = (localVal & ((uint32_t)0x003F0000)) >> 16;
    *dcqvec = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *dcivec = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl2_dc_cal_state_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xC0000000)) >> 30);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl2_dc_comp_dac_q_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x3F000000)) >> 24);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl2_dc_comp_dac_i_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x003F0000)) >> 16);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl2_dcq_vec_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl2_dci_vec_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL2_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

 /**
 * @brief O_DIG_BB_CTRL3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     05      zb_sfd_sync_err   0
 *     04      rx_pdu_type_ind   0
 *  03:01   rx_add_filter_ind1   0b0
 *     00   rx_add_filter_ind0   0
 * </pre>
 */
#define BB_TOP_O_DIG_BB_CTRL3_OFFSET 0x000000F0


__INLINE uint32_t bb_top_o_dig_bb_ctrl3_get(void)
{
    return _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
}

// field definitions
#define BB_TOP_O_DIG_BB_CTRL3_ZB_SFD_SYNC_ERR_BIT                   ((uint32_t)0x00000020)
#define BB_TOP_O_DIG_BB_CTRL3_ZB_SFD_SYNC_ERR_POS                   5
#define BB_TOP_O_DIG_BB_CTRL3_RX_PDU_TYPE_IND_BIT                   ((uint32_t)0x00000010)
#define BB_TOP_O_DIG_BB_CTRL3_RX_PDU_TYPE_IND_POS                   4
#define BB_TOP_O_DIG_BB_CTRL3_RX_ADD_FILTER_IND1_MASK               ((uint32_t)0x0000000E)
#define BB_TOP_O_DIG_BB_CTRL3_RX_ADD_FILTER_IND1_LSB                1
#define BB_TOP_O_DIG_BB_CTRL3_RX_ADD_FILTER_IND1_WIDTH              ((uint32_t)0x00000003)
#define BB_TOP_O_DIG_BB_CTRL3_RX_ADD_FILTER_IND0_BIT                ((uint32_t)0x00000001)
#define BB_TOP_O_DIG_BB_CTRL3_RX_ADD_FILTER_IND0_POS                0

#define BB_TOP_O_DIG_BB_CTRL3_ZB_SFD_SYNC_ERR_RST                   0x0
#define BB_TOP_O_DIG_BB_CTRL3_RX_PDU_TYPE_IND_RST                   0x0
#define BB_TOP_O_DIG_BB_CTRL3_RX_ADD_FILTER_IND1_RST                0x0
#define BB_TOP_O_DIG_BB_CTRL3_RX_ADD_FILTER_IND0_RST                0x0

__INLINE void bb_top_o_dig_bb_ctrl3_unpack(uint8_t* zbsfdsyncerr, uint8_t* rxpdutypeind, uint8_t* rxaddfilterind1, uint8_t* rxaddfilterind0)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);

    *zbsfdsyncerr = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rxpdutypeind = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxaddfilterind1 = (localVal & ((uint32_t)0x0000000E)) >> 1;
    *rxaddfilterind0 = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl3_zb_sfd_sync_err_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl3_rx_pdu_type_ind_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl3_rx_add_filter_ind1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000E)) >> 1);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl3_rx_add_filter_ind0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL3_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief O_DIG_BB_CTRL4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:24            rcal_fina   0b0
 *  22:16            kcal_fina   0b0
 *  08:00         ct_word_fina   0b0
 * </pre>
 */
#define BB_TOP_O_DIG_BB_CTRL4_OFFSET 0x000000F4


__INLINE uint32_t bb_top_o_dig_bb_ctrl4_get(void)
{
    return _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
}

// field definitions
#define BB_TOP_O_DIG_BB_CTRL4_RCAL_FINA_MASK                        ((uint32_t)0x1F000000)
#define BB_TOP_O_DIG_BB_CTRL4_RCAL_FINA_LSB                         24
#define BB_TOP_O_DIG_BB_CTRL4_RCAL_FINA_WIDTH                       ((uint32_t)0x00000005)
#define BB_TOP_O_DIG_BB_CTRL4_KCAL_FINA_MASK                        ((uint32_t)0x007F0000)
#define BB_TOP_O_DIG_BB_CTRL4_KCAL_FINA_LSB                         16
#define BB_TOP_O_DIG_BB_CTRL4_KCAL_FINA_WIDTH                       ((uint32_t)0x00000007)
#define BB_TOP_O_DIG_BB_CTRL4_CT_WORD_FINA_MASK                     ((uint32_t)0x000001FF)
#define BB_TOP_O_DIG_BB_CTRL4_CT_WORD_FINA_LSB                      0
#define BB_TOP_O_DIG_BB_CTRL4_CT_WORD_FINA_WIDTH                    ((uint32_t)0x00000009)

#define BB_TOP_O_DIG_BB_CTRL4_RCAL_FINA_RST                         0x0
#define BB_TOP_O_DIG_BB_CTRL4_KCAL_FINA_RST                         0x0
#define BB_TOP_O_DIG_BB_CTRL4_CT_WORD_FINA_RST                      0x0

__INLINE void bb_top_o_dig_bb_ctrl4_unpack(uint8_t* rcalfina, uint8_t* kcalfina, uint8_t* ctwordfina)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);

    *rcalfina = (localVal & ((uint32_t)0x1F000000)) >> 24;
    *kcalfina = (localVal & ((uint32_t)0x007F0000)) >> 16;
    *ctwordfina = (localVal & ((uint32_t)0x000001FF)) >> 0;
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl4_rcal_fina_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1F000000)) >> 24);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl4_kcal_fina_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x007F0000)) >> 16);
}

__INLINE uint16_t bb_top_o_dig_bb_ctrl4_ct_word_fina_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL4_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000001FF)) >> 0);
}

 /**
 * @brief O_DIG_BB_CTRL5 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     31   rx_rccal_comparator_out   0
 *     22           lock_ready   0
 *  19:00        actu_cnt_fina   0b0
 * </pre>
 */
#define BB_TOP_O_DIG_BB_CTRL5_OFFSET 0x000000F8


__INLINE uint32_t bb_top_o_dig_bb_ctrl5_get(void)
{
    return _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
}

// field definitions
#define BB_TOP_O_DIG_BB_CTRL5_RX_RCCAL_COMPARATOR_OUT_BIT           ((uint32_t)0x80000000)
#define BB_TOP_O_DIG_BB_CTRL5_RX_RCCAL_COMPARATOR_OUT_POS           31
#define BB_TOP_O_DIG_BB_CTRL5_LOCK_READY_BIT                        ((uint32_t)0x00400000)
#define BB_TOP_O_DIG_BB_CTRL5_LOCK_READY_POS                        22
#define BB_TOP_O_DIG_BB_CTRL5_ACTU_CNT_FINA_MASK                    ((uint32_t)0x000FFFFF)
#define BB_TOP_O_DIG_BB_CTRL5_ACTU_CNT_FINA_LSB                     0
#define BB_TOP_O_DIG_BB_CTRL5_ACTU_CNT_FINA_WIDTH                   ((uint32_t)0x00000014)

#define BB_TOP_O_DIG_BB_CTRL5_RX_RCCAL_COMPARATOR_OUT_RST           0x0
#define BB_TOP_O_DIG_BB_CTRL5_LOCK_READY_RST                        0x0
#define BB_TOP_O_DIG_BB_CTRL5_ACTU_CNT_FINA_RST                     0x0

__INLINE void bb_top_o_dig_bb_ctrl5_unpack(uint8_t* rxrccalcomparatorout, uint8_t* lockready, uint8_t* actucntfina)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL5_OFFSET + BB_TOP_BASE_ADDR);

    *rxrccalcomparatorout = (localVal & ((uint32_t)0x80000000)) >> 31;
    *lockready = (localVal & ((uint32_t)0x00400000)) >> 22;
    *actucntfina = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl5_rx_rccal_comparator_out_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x80000000)) >> 31);
}

__INLINE uint8_t bb_top_o_dig_bb_ctrl5_lock_ready_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00400000)) >> 22);
}

__INLINE uint32_t bb_top_o_dig_bb_ctrl5_actu_cnt_fina_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

 /**
 * @brief O_DIG_BB_CTRL6 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:20        supp_samp_cnt   0b0
 *  19:10          supp_samp_q   0b0
 *  09:00          supp_samp_i   0b0
 * </pre>
 */
#define BB_TOP_O_DIG_BB_CTRL6_OFFSET 0x000000FC


__INLINE uint32_t bb_top_o_dig_bb_ctrl6_get(void)
{
    return _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
}

// field definitions
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_CNT_MASK                    ((uint32_t)0xFFF00000)
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_CNT_LSB                     20
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_CNT_WIDTH                   ((uint32_t)0x0000000C)
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_Q_MASK                      ((uint32_t)0x000FFC00)
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_Q_LSB                       10
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_Q_WIDTH                     ((uint32_t)0x0000000A)
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_I_MASK                      ((uint32_t)0x000003FF)
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_I_LSB                       0
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_I_WIDTH                     ((uint32_t)0x0000000A)

#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_CNT_RST                     0x0
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_Q_RST                       0x0
#define BB_TOP_O_DIG_BB_CTRL6_SUPP_SAMP_I_RST                       0x0

__INLINE void bb_top_o_dig_bb_ctrl6_unpack(uint8_t* suppsampcnt, uint8_t* suppsampq, uint8_t* suppsampi)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL6_OFFSET + BB_TOP_BASE_ADDR);

    *suppsampcnt = (localVal & ((uint32_t)0xFFF00000)) >> 20;
    *suppsampq = (localVal & ((uint32_t)0x000FFC00)) >> 10;
    *suppsampi = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t bb_top_o_dig_bb_ctrl6_supp_samp_cnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFF00000)) >> 20);
}

__INLINE uint16_t bb_top_o_dig_bb_ctrl6_supp_samp_q_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFC00)) >> 10);
}

__INLINE uint16_t bb_top_o_dig_bb_ctrl6_supp_samp_i_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_O_DIG_BB_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

 /**
 * @brief I_PLL_CTRL5 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24             delay_up   0b0
 *  23:21             delay_dn   0b0
 *  08:05              cp_prog   0b0
 *     00      en_pllbuf_ctcnt   0
 * </pre>
 */
#define BB_TOP_I_PLL_CTRL5_OFFSET 0x00000104


__INLINE uint32_t bb_top_i_pll_ctrl5_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_pll_ctrl5_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL5_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_PLL_CTRL5_DELAY_UP_MASK                         ((uint32_t)0x07000000)
#define BB_TOP_I_PLL_CTRL5_DELAY_UP_LSB                          24
#define BB_TOP_I_PLL_CTRL5_DELAY_UP_WIDTH                        ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL5_DELAY_DN_MASK                         ((uint32_t)0x00E00000)
#define BB_TOP_I_PLL_CTRL5_DELAY_DN_LSB                          21
#define BB_TOP_I_PLL_CTRL5_DELAY_DN_WIDTH                        ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL5_CP_PROG_MASK                          ((uint32_t)0x000001E0)
#define BB_TOP_I_PLL_CTRL5_CP_PROG_LSB                           5
#define BB_TOP_I_PLL_CTRL5_CP_PROG_WIDTH                         ((uint32_t)0x00000004)
#define BB_TOP_I_PLL_CTRL5_EN_PLLBUF_CTCNT_BIT                   ((uint32_t)0x00000001)
#define BB_TOP_I_PLL_CTRL5_EN_PLLBUF_CTCNT_POS                   0

#define BB_TOP_I_PLL_CTRL5_DELAY_UP_RST                          0x0
#define BB_TOP_I_PLL_CTRL5_DELAY_DN_RST                          0x0
#define BB_TOP_I_PLL_CTRL5_CP_PROG_RST                           0x0
#define BB_TOP_I_PLL_CTRL5_EN_PLLBUF_CTCNT_RST                   0x0

__INLINE void bb_top_i_pll_ctrl5_pack(uint8_t delayup, uint8_t delaydn, uint8_t cpprog, uint8_t enpllbufctcnt)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL5_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)delayup << 24) | ((uint32_t)delaydn << 21) | ((uint32_t)cpprog << 5) | ((uint32_t)enpllbufctcnt << 0));
}

__INLINE void bb_top_i_pll_ctrl5_unpack(uint8_t* delayup, uint8_t* delaydn, uint8_t* cpprog, uint8_t* enpllbufctcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR);

    *delayup = (localVal & ((uint32_t)0x07000000)) >> 24;
    *delaydn = (localVal & ((uint32_t)0x00E00000)) >> 21;
    *cpprog = (localVal & ((uint32_t)0x000001E0)) >> 5;
    *enpllbufctcnt = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t bb_top_i_pll_ctrl5_delay_up_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void bb_top_i_pll_ctrl5_delay_up_setf(uint8_t delayup)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL5_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)delayup << 24));
}

__INLINE uint8_t bb_top_i_pll_ctrl5_delay_dn_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00E00000)) >> 21);
}

__INLINE void bb_top_i_pll_ctrl5_delay_dn_setf(uint8_t delaydn)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL5_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00E00000)) | ((uint32_t)delaydn << 21));
}

__INLINE uint8_t bb_top_i_pll_ctrl5_cp_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000001E0)) >> 5);
}

__INLINE void bb_top_i_pll_ctrl5_cp_prog_setf(uint8_t cpprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL5_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000001E0)) | ((uint32_t)cpprog << 5));
}

__INLINE uint8_t bb_top_i_pll_ctrl5_en_pllbuf_ctcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void bb_top_i_pll_ctrl5_en_pllbuf_ctcnt_setf(uint8_t enpllbufctcnt)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL5_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL5_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)enpllbufctcnt << 0));
}

 /**
 * @brief I_PLL_CTRL6 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:21               lpf_rz   0b0
 *  20:18          ib_tpm_prog   0b10
 *  17:16               lpf_r2   0b0
 *  15:14               lpf_r1   0b0
 *  13:12               lpf_c3   0b0
 *  11:10               lpf_c2   0b0
 *  09:08               lpf_c1   0b0
 *  07:04          ib_vco_prog   0b1000
 *  03:00           ib_cp_prog   0b1010
 * </pre>
 */
#define BB_TOP_I_PLL_CTRL6_OFFSET 0x00000108


__INLINE uint32_t bb_top_i_pll_ctrl6_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_pll_ctrl6_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_PLL_CTRL6_LPF_RZ_MASK                           ((uint32_t)0x00E00000)
#define BB_TOP_I_PLL_CTRL6_LPF_RZ_LSB                            21
#define BB_TOP_I_PLL_CTRL6_LPF_RZ_WIDTH                          ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL6_IB_TPM_PROG_MASK                      ((uint32_t)0x001C0000)
#define BB_TOP_I_PLL_CTRL6_IB_TPM_PROG_LSB                       18
#define BB_TOP_I_PLL_CTRL6_IB_TPM_PROG_WIDTH                     ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL6_LPF_R2_MASK                           ((uint32_t)0x00030000)
#define BB_TOP_I_PLL_CTRL6_LPF_R2_LSB                            16
#define BB_TOP_I_PLL_CTRL6_LPF_R2_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL6_LPF_R1_MASK                           ((uint32_t)0x0000C000)
#define BB_TOP_I_PLL_CTRL6_LPF_R1_LSB                            14
#define BB_TOP_I_PLL_CTRL6_LPF_R1_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL6_LPF_C3_MASK                           ((uint32_t)0x00003000)
#define BB_TOP_I_PLL_CTRL6_LPF_C3_LSB                            12
#define BB_TOP_I_PLL_CTRL6_LPF_C3_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL6_LPF_C2_MASK                           ((uint32_t)0x00000C00)
#define BB_TOP_I_PLL_CTRL6_LPF_C2_LSB                            10
#define BB_TOP_I_PLL_CTRL6_LPF_C2_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL6_LPF_C1_MASK                           ((uint32_t)0x00000300)
#define BB_TOP_I_PLL_CTRL6_LPF_C1_LSB                            8
#define BB_TOP_I_PLL_CTRL6_LPF_C1_WIDTH                          ((uint32_t)0x00000002)
#define BB_TOP_I_PLL_CTRL6_IB_VCO_PROG_MASK                      ((uint32_t)0x000000F0)
#define BB_TOP_I_PLL_CTRL6_IB_VCO_PROG_LSB                       4
#define BB_TOP_I_PLL_CTRL6_IB_VCO_PROG_WIDTH                     ((uint32_t)0x00000004)
#define BB_TOP_I_PLL_CTRL6_IB_CP_PROG_MASK                       ((uint32_t)0x0000000F)
#define BB_TOP_I_PLL_CTRL6_IB_CP_PROG_LSB                        0
#define BB_TOP_I_PLL_CTRL6_IB_CP_PROG_WIDTH                      ((uint32_t)0x00000004)

#define BB_TOP_I_PLL_CTRL6_LPF_RZ_RST                            0x0
#define BB_TOP_I_PLL_CTRL6_IB_TPM_PROG_RST                       0x10
#define BB_TOP_I_PLL_CTRL6_LPF_R2_RST                            0x0
#define BB_TOP_I_PLL_CTRL6_LPF_R1_RST                            0x0
#define BB_TOP_I_PLL_CTRL6_LPF_C3_RST                            0x0
#define BB_TOP_I_PLL_CTRL6_LPF_C2_RST                            0x0
#define BB_TOP_I_PLL_CTRL6_LPF_C1_RST                            0x0
#define BB_TOP_I_PLL_CTRL6_IB_VCO_PROG_RST                       0x1000
#define BB_TOP_I_PLL_CTRL6_IB_CP_PROG_RST                        0x1010

__INLINE void bb_top_i_pll_ctrl6_pack(uint8_t lpfrz, uint8_t ibtpmprog, uint8_t lpfr2, uint8_t lpfr1, uint8_t lpfc3, uint8_t lpfc2, uint8_t lpfc1, uint8_t ibvcoprog, uint8_t ibcpprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)lpfrz << 21) | ((uint32_t)ibtpmprog << 18) | ((uint32_t)lpfr2 << 16) | ((uint32_t)lpfr1 << 14) | ((uint32_t)lpfc3 << 12) | ((uint32_t)lpfc2 << 10) | ((uint32_t)lpfc1 << 8) | ((uint32_t)ibvcoprog << 4) | ((uint32_t)ibcpprog << 0));
}

__INLINE void bb_top_i_pll_ctrl6_unpack(uint8_t* lpfrz, uint8_t* ibtpmprog, uint8_t* lpfr2, uint8_t* lpfr1, uint8_t* lpfc3, uint8_t* lpfc2, uint8_t* lpfc1, uint8_t* ibvcoprog, uint8_t* ibcpprog)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);

    *lpfrz = (localVal & ((uint32_t)0x00E00000)) >> 21;
    *ibtpmprog = (localVal & ((uint32_t)0x001C0000)) >> 18;
    *lpfr2 = (localVal & ((uint32_t)0x00030000)) >> 16;
    *lpfr1 = (localVal & ((uint32_t)0x0000C000)) >> 14;
    *lpfc3 = (localVal & ((uint32_t)0x00003000)) >> 12;
    *lpfc2 = (localVal & ((uint32_t)0x00000C00)) >> 10;
    *lpfc1 = (localVal & ((uint32_t)0x00000300)) >> 8;
    *ibvcoprog = (localVal & ((uint32_t)0x000000F0)) >> 4;
    *ibcpprog = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t bb_top_i_pll_ctrl6_lpf_rz_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00E00000)) >> 21);
}

__INLINE void bb_top_i_pll_ctrl6_lpf_rz_setf(uint8_t lpfrz)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00E00000)) | ((uint32_t)lpfrz << 21));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_ib_tpm_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x001C0000)) >> 18);
}

__INLINE void bb_top_i_pll_ctrl6_ib_tpm_prog_setf(uint8_t ibtpmprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x001C0000)) | ((uint32_t)ibtpmprog << 18));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_lpf_r2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void bb_top_i_pll_ctrl6_lpf_r2_setf(uint8_t lpfr2)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)lpfr2 << 16));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_lpf_r1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000C000)) >> 14);
}

__INLINE void bb_top_i_pll_ctrl6_lpf_r1_setf(uint8_t lpfr1)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000C000)) | ((uint32_t)lpfr1 << 14));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_lpf_c3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00003000)) >> 12);
}

__INLINE void bb_top_i_pll_ctrl6_lpf_c3_setf(uint8_t lpfc3)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00003000)) | ((uint32_t)lpfc3 << 12));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_lpf_c2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000C00)) >> 10);
}

__INLINE void bb_top_i_pll_ctrl6_lpf_c2_setf(uint8_t lpfc2)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000C00)) | ((uint32_t)lpfc2 << 10));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_lpf_c1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void bb_top_i_pll_ctrl6_lpf_c1_setf(uint8_t lpfc1)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)lpfc1 << 8));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_ib_vco_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000F0)) >> 4);
}

__INLINE void bb_top_i_pll_ctrl6_ib_vco_prog_setf(uint8_t ibvcoprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x000000F0)) | ((uint32_t)ibvcoprog << 4));
}

__INLINE uint8_t bb_top_i_pll_ctrl6_ib_cp_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void bb_top_i_pll_ctrl6_ib_cp_prog_setf(uint8_t ibcpprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL6_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL6_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)ibcpprog << 0));
}

 /**
 * @brief I_PLL_CTRL7 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24            vmid_prog   0b100
 *  20:18     vco_var_tpm_prog   0b100
 *  17:15         vco_var_prog   0b100
 *     10              pfd_pol   0
 *  09:00             Reserved   0b0
 * </pre>
 */
#define BB_TOP_I_PLL_CTRL7_OFFSET 0x0000010C


__INLINE uint32_t bb_top_i_pll_ctrl7_get(void)
{
    return _PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR);
}

__INLINE void bb_top_i_pll_ctrl7_set(uint32_t value)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL7_OFFSET+ BB_TOP_BASE_ADDR, value);
}

// field definitions
#define BB_TOP_I_PLL_CTRL7_VMID_PROG_MASK                        ((uint32_t)0x07000000)
#define BB_TOP_I_PLL_CTRL7_VMID_PROG_LSB                         24
#define BB_TOP_I_PLL_CTRL7_VMID_PROG_WIDTH                       ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_TPM_PROG_MASK                 ((uint32_t)0x001C0000)
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_TPM_PROG_LSB                  18
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_TPM_PROG_WIDTH                ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_PROG_MASK                     ((uint32_t)0x00038000)
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_PROG_LSB                      15
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_PROG_WIDTH                    ((uint32_t)0x00000003)
#define BB_TOP_I_PLL_CTRL7_PFD_POL_BIT                           ((uint32_t)0x00000400)
#define BB_TOP_I_PLL_CTRL7_PFD_POL_POS                           10
#define BB_TOP_I_PLL_CTRL7_RESERVED_MASK                         ((uint32_t)0x000003FF)
#define BB_TOP_I_PLL_CTRL7_RESERVED_LSB                          0
#define BB_TOP_I_PLL_CTRL7_RESERVED_WIDTH                        ((uint32_t)0x0000000A)

#define BB_TOP_I_PLL_CTRL7_VMID_PROG_RST                         0x100
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_TPM_PROG_RST                  0x100
#define BB_TOP_I_PLL_CTRL7_VCO_VAR_PROG_RST                      0x100
#define BB_TOP_I_PLL_CTRL7_PFD_POL_RST                           0x0
#define BB_TOP_I_PLL_CTRL7_RESERVED_RST                          0x0

__INLINE void bb_top_i_pll_ctrl7_pack(uint8_t vmidprog, uint8_t vcovartpmprog, uint8_t vcovarprog, uint8_t pfdpol)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL7_OFFSET+ BB_TOP_BASE_ADDR,  ((uint32_t)vmidprog << 24) | ((uint32_t)vcovartpmprog << 18) | ((uint32_t)vcovarprog << 15) | ((uint32_t)pfdpol << 10));
}

__INLINE void bb_top_i_pll_ctrl7_unpack(uint8_t* vmidprog, uint8_t* vcovartpmprog, uint8_t* vcovarprog, uint8_t* pfdpol, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR);

    *vmidprog = (localVal & ((uint32_t)0x07000000)) >> 24;
    *vcovartpmprog = (localVal & ((uint32_t)0x001C0000)) >> 18;
    *vcovarprog = (localVal & ((uint32_t)0x00038000)) >> 15;
    *pfdpol = (localVal & ((uint32_t)0x00000400)) >> 10;
    *reserved = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint8_t bb_top_i_pll_ctrl7_vmid_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void bb_top_i_pll_ctrl7_vmid_prog_setf(uint8_t vmidprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL7_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)vmidprog << 24));
}

__INLINE uint8_t bb_top_i_pll_ctrl7_vco_var_tpm_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x001C0000)) >> 18);
}

__INLINE void bb_top_i_pll_ctrl7_vco_var_tpm_prog_setf(uint8_t vcovartpmprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL7_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x001C0000)) | ((uint32_t)vcovartpmprog << 18));
}

__INLINE uint8_t bb_top_i_pll_ctrl7_vco_var_prog_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00038000)) >> 15);
}

__INLINE void bb_top_i_pll_ctrl7_vco_var_prog_setf(uint8_t vcovarprog)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL7_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00038000)) | ((uint32_t)vcovarprog << 15));
}

__INLINE uint8_t bb_top_i_pll_ctrl7_pfd_pol_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void bb_top_i_pll_ctrl7_pfd_pol_setf(uint8_t pfdpol)
{
    _PICO_REG_WR(BB_TOP_I_PLL_CTRL7_OFFSET+ BB_TOP_BASE_ADDR, (_PICO_REG_RD(BB_TOP_I_PLL_CTRL7_OFFSET + BB_TOP_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)pfdpol << 10));
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t rx_sync_thd:8;
      __IO uint32_t blr_acc_err:3;
      __IO uint32_t zb_sfd_err:1;
      __IO uint32_t sync_sch_dly:2;
      __IO uint32_t sync_len:2;
      __IO uint32_t rx_iq_fsel:8;
      __IO uint32_t :2;
      __IO uint32_t rx_32m_clk_gate:1;
      __IO uint32_t tx_32m_clk_gate:1;
      __IO uint32_t clk32m_rise:1;
      __IO uint32_t pkt_fmt:3;
    }i_dig_bb_ctrl0_fld;
    __IO uint32_t i_dig_bb_ctrl0;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t sync_rssi_thd:8;
      __IO uint32_t rx_sync_thd_hi:8;
      __IO uint32_t :1;
      __IO uint32_t diff_demod_thd:7;
      __IO uint32_t tr_dc_sft:4;
      __IO uint32_t tr_tedm_sft:4;
    }i_dig_bb_ctrl1_fld;
    __IO uint32_t i_dig_bb_ctrl1;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t ahdfe_ki_cr:8;
      __IO uint32_t ahdfe_kp_cr:8;
      __IO uint32_t :7;
      __IO uint32_t ahdfe_en:1;
      __IO uint32_t ahdfe_hidx1:4;
      __IO uint32_t ahdfe_hidx0:4;
    }i_dig_bb_ctrl2_fld;
    __IO uint32_t i_dig_bb_ctrl2;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t rx_time_out:16;
      __IO uint32_t rx_supp_samp:8;
      __IO uint32_t rx_max_packet_len:8;
    }i_dig_bb_ctrl3_fld;
    __IO uint32_t i_dig_bb_ctrl3;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :2;
      __IO uint32_t dc_compq_flt3:6;
      __IO uint32_t :2;
      __IO uint32_t dc_compi_flt3:6;
      __IO uint32_t :2;
      __IO uint32_t dc_compq_flt4:6;
      __IO uint32_t :2;
      __IO uint32_t dc_compi_flt4:6;
    }i_dig_bb_ctrl4_fld;
    __IO uint32_t i_dig_bb_ctrl4;
  };


  union{ //offset addr 0x0040
    struct{
      __IO uint32_t tx_supp_tail:8;
      __IO uint32_t :4;
      __IO uint32_t tx_iq_fsel:4;
      __IO uint32_t tx_pld_nbyte:8;
      __IO uint32_t tx_pre_len_ext:3;
      __IO uint32_t tx_mode_sel:1;
      __IO uint32_t tx_pldt:4;
    }i_dig_bb_ctrl16_fld;
    __IO uint32_t i_dig_bb_ctrl16;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t tx_prbs_seed:32;
    }i_dig_bb_ctrl17_fld;
    __IO uint32_t i_dig_bb_ctrl17;
  };

  union{ //offset addr 0x0048
    struct{
      __IO uint32_t :1;
      __IO uint32_t tx_wt_seed:7;
      __IO uint32_t tx_crc_24:24;
    }i_dig_bb_ctrl18_fld;
    __IO uint32_t i_dig_bb_ctrl18;
  };

  union{ //offset addr 0x004c
    struct{
      __IO uint32_t tx_ble_acc_adr:32;
    }i_dig_bb_ctrl19_fld;
    __IO uint32_t i_dig_bb_ctrl19;
  };

  union{ //offset addr 0x0050
    struct{
      __IO uint32_t rssi_offset:8;
      __IO uint32_t agc_time_out:4;
      __IO uint32_t agc_settle:4;
      __IO uint32_t rx_pwr_t:4;
      __IO uint32_t rx_abs_t:4;
      __IO uint32_t agc_en:1;
      __IO uint32_t :2;
      __IO uint32_t agc_fix_gain:1;
      __IO uint32_t agc_gain:4;
    }i_dig_bb_ctrl20_fld;
    __IO uint32_t i_dig_bb_ctrl20;
  };

  union{ //offset addr 0x0054
    struct{
      __IO uint32_t agc_tab_gset3:8;
      __IO uint32_t agc_tab_gset2:8;
      __IO uint32_t agc_tab_gset1:8;
      __IO uint32_t agc_tab_gset0:8;
    }i_dig_bb_ctrl21_fld;
    __IO uint32_t i_dig_bb_ctrl21;
  };

  union{ //offset addr 0x0058
    struct{
      __IO uint32_t agc_tab_gset7:8;
      __IO uint32_t agc_tab_gset6:8;
      __IO uint32_t agc_tab_gset5:8;
      __IO uint32_t agc_tab_gset4:8;
    }i_dig_bb_ctrl22_fld;
    __IO uint32_t i_dig_bb_ctrl22;
  };

  union{ //offset addr 0x005c
    struct{
      __IO uint32_t agc_tab_dto2:8;
      __IO uint32_t agc_tab_dto1:8;
      __IO uint32_t agc_tab_dto0:8;
      __IO uint32_t agc_tab_gset8:8;
    }i_dig_bb_ctrl23_fld;
    __IO uint32_t i_dig_bb_ctrl23;
  };

  union{ //offset addr 0x0060
    struct{
      __IO uint32_t agc_tab_dto6:8;
      __IO uint32_t agc_tab_dto5:8;
      __IO uint32_t agc_tab_dto4:8;
      __IO uint32_t agc_tab_dto3:8;
    }i_dig_bb_ctrl24_fld;
    __IO uint32_t i_dig_bb_ctrl24;
  };

  union{ //offset addr 0x0064
    struct{
      __IO uint32_t agc_tab_ato1:8;
      __IO uint32_t agc_tab_ato0:8;
      __IO uint32_t agc_tab_dto8:8;
      __IO uint32_t agc_tab_dto7:8;
    }i_dig_bb_ctrl25_fld;
    __IO uint32_t i_dig_bb_ctrl25;
  };

  union{ //offset addr 0x0068
    struct{
      __IO uint32_t agc_tab_ato5:8;
      __IO uint32_t agc_tab_ato4:8;
      __IO uint32_t agc_tab_ato3:8;
      __IO uint32_t agc_tab_ato2:8;
    }i_dig_bb_ctrl26_fld;
    __IO uint32_t i_dig_bb_ctrl26;
  };

  union{ //offset addr 0x006c
    struct{
      __IO uint32_t agc_tab_gain0:8;
      __IO uint32_t agc_tab_ato8:8;
      __IO uint32_t agc_tab_ato7:8;
      __IO uint32_t agc_tab_ato6:8;
    }i_dig_bb_ctrl27_fld;
    __IO uint32_t i_dig_bb_ctrl27;
  };

  union{ //offset addr 0x0070
    struct{
      __IO uint32_t agc_tab_gain4:8;
      __IO uint32_t agc_tab_gain3:8;
      __IO uint32_t agc_tab_gain2:8;
      __IO uint32_t agc_tab_gain1:8;
    }i_dig_bb_ctrl28_fld;
    __IO uint32_t i_dig_bb_ctrl28;
  };

  union{ //offset addr 0x0074
    struct{
      __IO uint32_t agc_tab_gain8:8;
      __IO uint32_t agc_tab_gain7:8;
      __IO uint32_t agc_tab_gain6:8;
      __IO uint32_t agc_tab_gain5:8;
    }i_dig_bb_ctrl29_fld;
    __IO uint32_t i_dig_bb_ctrl29;
  };

  union{ //offset addr 0x0078
    struct{
      __IO uint32_t dcoc_flt_phy:10;
      __IO uint32_t dcoc_amp_step:4;
      __IO uint32_t dcoc_amp_ini:6;
      __IO uint32_t dcoc_resdc_thd:4;
      __IO uint32_t dcoc_phy_thd:6;
      __IO uint32_t dcoc_cal_mod:1;
      __IO uint32_t dcoc_en:1;
    }i_dig_bb_ctrl30_fld;
    __IO uint32_t i_dig_bb_ctrl30;
  };

  union{ //offset addr 0x007c
    struct{
      __IO uint32_t :2;
      __IO uint32_t dcoc_phy2:10;
      __IO uint32_t dcoc_phy1:10;
      __IO uint32_t dcoc_phy0:10;
    }i_dig_bb_ctrl31_fld;
    __IO uint32_t i_dig_bb_ctrl31;
  };

  union{ //offset addr 0x0080
    struct{
      __IO uint32_t :17;
      __IO uint32_t ld_en:1;
      __IO uint32_t ld_time:2;
      __IO uint32_t len_slot_fr_spir:2;
      __IO uint32_t len_ct_fr_spir:3;
      __IO uint32_t len_fc_fr_spir:3;
      __IO uint32_t len_rst_fr_spir:3;
      __IO uint32_t clk_flag_fr_spir:1;
    }i_dig_bb_ctrl32_fld;
    __IO uint32_t i_dig_bb_ctrl32;
  };

  union{ //offset addr 0x0084
    struct{
      __IO uint32_t ct_override_fr_spir:1;
      __IO uint32_t :22;
      __IO uint32_t ct_word_fr_spir:9;
    }i_dig_bb_ctrl33_fld;
    __IO uint32_t i_dig_bb_ctrl33;
  };

  union{ //offset addr 0x0088
    struct{
      __IO uint32_t targ_cnt_override:1;
      __IO uint32_t :11;
      __IO uint32_t targ_cnt_fr_spir:20;
    }i_dig_bb_ctrl34_fld;
    __IO uint32_t i_dig_bb_ctrl34;
  };

  union{ //offset addr 0x008c
    struct{
      __IO uint32_t :7;
      __IO uint32_t cnt_manual_en:1;
      __IO uint32_t clk_edge_polarity:1;
      __IO uint32_t tp_cal_en_fr_spir:1;
      __IO uint32_t delta_ftx_fr_spir:1;
      __IO uint32_t mod_sel_fr_spir:1;
      __IO uint32_t leng_tm1_fr_spir:2;
      __IO uint32_t leng_tm0_fr_spir:2;
      __IO uint32_t :1;
      __IO uint32_t kdac_fi_fr_spir:7;
      __IO uint32_t :1;
      __IO uint32_t kdac_co_fr_spir:7;
    }i_dig_bb_ctrl35_fld;
    __IO uint32_t i_dig_bb_ctrl35;
  };

  union{ //offset addr 0x0090
    struct{
      __IO uint32_t :12;
      __IO uint32_t reg_dc_fr_spir:20;
    }i_dig_bb_ctrl36_fld;
    __IO uint32_t i_dig_bb_ctrl36;
  };

  union{ //offset addr 0x0094
    struct{
      __IO uint32_t :15;
      __IO uint32_t tp_cal_override:1;
      __IO uint32_t :3;
      __IO uint32_t rcal_fr_spir:5;
      __IO uint32_t :1;
      __IO uint32_t kcal_fr_spir:7;
    }i_dig_bb_ctrl37_fld;
    __IO uint32_t i_dig_bb_ctrl37;
  };

  union{ //offset addr 0x0098
    struct{
      __IO uint32_t :15;
      __IO uint32_t intg_frac_override:1;
      __IO uint32_t :7;
      __IO uint32_t intg_fr_spir:9;
    }i_dig_bb_ctrl38_fld;
    __IO uint32_t i_dig_bb_ctrl38;
  };

  union{ //offset addr 0x009c
    struct{
      __IO uint32_t :7;
      __IO uint32_t dsm_override:1;
      __IO uint32_t frac_fr_spir:24;
    }i_dig_bb_ctrl39_fld;
    __IO uint32_t i_dig_bb_ctrl39;
  };

  union{ //offset addr 0x00a0
    struct{
      __IO uint32_t :27;
      __IO uint32_t pll_test_override:1;
      __IO uint32_t rx_bb_en_override:1;
      __IO uint32_t tx_bb_en_override:1;
      __IO uint32_t rx_en_override:1;
      __IO uint32_t tx_pa_en_override:1;
    }i_dig_bb_ctrl40_fld;
    __IO uint32_t i_dig_bb_ctrl40;
  };

  union{ //offset addr 0x00a4
    struct{
      __IO uint32_t :23;
      __IO uint32_t pa_setb_fr_spir:1;
      __IO uint32_t rst_fr_spir:1;
      __IO uint32_t tpm_mod_en_fr_spir:1;
      __IO uint32_t rx_at_ctrl0_fr_spir:1;
      __IO uint32_t tx_at_ctrl0_fr_spir:1;
      __IO uint32_t rx_bb_en_fr_spir:1;
      __IO uint32_t tx_bb_en_fr_spir:1;
      __IO uint32_t rx_en_fr_spir:1;
      __IO uint32_t tx_pa_en_fr_spir:1;
    }i_dig_bb_ctrl41_fld;
    __IO uint32_t i_dig_bb_ctrl41;
  };

  union{ //offset addr 0x00a8
    struct{
      __IO uint32_t :25;
      __IO uint32_t synth_lo_override:1;
      __IO uint32_t synth_tpm_en_override:1;
      __IO uint32_t synth_vmid_override:1;
      __IO uint32_t synth_bias_en_override:1;
      __IO uint32_t synth_pfd_override:1;
      __IO uint32_t synth_fch_override:1;
      __IO uint32_t synth_ldo_override:1;
    }i_dig_bb_ctrl42_fld;
    __IO uint32_t i_dig_bb_ctrl42;
  };

  union{ //offset addr 0x00ac
    struct{
      __IO uint32_t :12;
      __IO uint32_t tpm_dac_reset_fr_spir:1;
      __IO uint32_t ct_en_fr_spir:1;
      __IO uint32_t vmid_en_fr_spir:1;
      __IO uint32_t tpm_en_fr_spir:1;
      __IO uint32_t en_tpm_var_fr_spir:1;
      __IO uint32_t reset_ndiv_fr_spir:1;
      __IO uint32_t pfd_en_fr_spir:1;
      __IO uint32_t fastcharge_vref_vco_fr_spir:1;
      __IO uint32_t fch_vco_vos_fr_spir:1;
      __IO uint32_t fch_vco_pbgen_fr_spir:1;
      __IO uint32_t cp_bias_en_fr_spir:1;
      __IO uint32_t en_vco_bias_fr_spir:1;
      __IO uint32_t en_pll_imir_fr_spir:1;
      __IO uint32_t en_pllbuf_fr_spir:1;
      __IO uint32_t en_lotx_fr_spir:1;
      __IO uint32_t en_lorx_fr_spir:1;
      __IO uint32_t pu_ldo_lo_fr_spir:1;
      __IO uint32_t pu_ldo_pll_fr_spir:1;
      __IO uint32_t pu_ldo_vco_fr_spir:1;
      __IO uint32_t pu_ldo_top_fr_spir:1;
    }i_dig_bb_ctrl43_fld;
    __IO uint32_t i_dig_bb_ctrl43;
  };

  union{ //offset addr 0x00b0
    struct{
      __IO uint32_t tpd_plry_fr_spir:1;
      __IO uint32_t :4;
      __IO uint32_t pi_dsft_fr_spir:3;
      __IO uint32_t :6;
      __IO uint32_t intg_dly_fr_spir:2;
      __IO uint32_t :3;
      __IO uint32_t tx_sdm_dly:5;
      __IO uint32_t :3;
      __IO uint32_t tx_dac_dly:5;
    }i_dig_bb_ctrl44_fld;
    __IO uint32_t i_dig_bb_ctrl44;
  };

  union{ //offset addr 0x00b4
    struct{
      __IO uint32_t :7;
      __IO uint32_t if_freq:1;
      __IO uint32_t rx_freq_off:8;
      __IO uint32_t tx_freq_off:8;
      __IO uint32_t rf_chn:8;
    }i_dig_bb_ctrl45_fld;
    __IO uint32_t i_dig_bb_ctrl45;
  };

  union{ //offset addr 0x00b8
    struct{
      __IO uint32_t :15;
      __IO uint32_t trx_pa_target:5;
      __IO uint32_t tm_pa_settle:4;
      __IO uint32_t :1;
      __IO uint32_t trx_paramp_speed:3;
      __IO uint32_t :1;
      __IO uint32_t len_dly_ramp_fr_spir:3;
    }i_dig_bb_ctrl46_fld;
    __IO uint32_t i_dig_bb_ctrl46;
  };

  union{ //offset addr 0x00bc
    struct{
      __IO uint32_t :8;
      __IO uint32_t tx_bb_en_dly:8;
      __IO uint32_t :4;
      __IO uint32_t tm_rxafe_settle:4;
      __IO uint32_t tm_pll_ftune:8;
    }i_dig_bb_ctrl47_fld;
    __IO uint32_t i_dig_bb_ctrl47;
  };

  union{ //offset addr 0x00c0
    struct{
      __IO uint32_t :2;
      __IO uint32_t dc_compq_flt1:6;
      __IO uint32_t :2;
      __IO uint32_t dc_compi_flt1:6;
      __IO uint32_t :2;
      __IO uint32_t dc_compq_flt2:6;
      __IO uint32_t :2;
      __IO uint32_t dc_compi_flt2:6;
    }i_dig_bb_ctrl48_fld;
    __IO uint32_t i_dig_bb_ctrl48;
  };

  union{ //offset addr 0x00c4
    struct{
      __IO uint32_t :14;
      __IO uint32_t dc_comp_sel:2;
      __IO uint32_t :2;
      __IO uint32_t dc_compq_flt0:6;
      __IO uint32_t :2;
      __IO uint32_t dc_compi_flt0:6;
    }i_dig_bb_ctrl49_fld;
    __IO uint32_t i_dig_bb_ctrl49;
  };

  union{ //offset addr 0x00c8
    struct{
      __IO uint32_t :23;
      __IO uint32_t agc_gain_override:1;
      __IO uint32_t agc_gain_fr_spir:8;
    }i_dig_bb_ctrl50_fld;
    __IO uint32_t i_dig_bb_ctrl50;
  };

  union{ //offset addr 0x00cc
    struct{
      __IO uint32_t :20;
      __IO uint32_t vout_ctrl_ldo_lo:2;
      __IO uint32_t vout_ctrl_ldo_pll:2;
      __IO uint32_t vout_ctrl_ldo_vco:2;
      __IO uint32_t :6;
    }i_pll_ctrl0_fld;
    __IO uint32_t i_pll_ctrl0;
  };

  union{ //offset addr 0x00d0
    struct{
      __IO uint32_t :5;
      __IO uint32_t delay_up:3;
      __IO uint32_t delay_dn:3;
      __IO uint32_t :12;
      __IO uint32_t cp_prog:4;
      __IO uint32_t :4;
      __IO uint32_t en_pllbuf_ctcnt:1;
    }i_pll_ctrl1_fld;
    __IO uint32_t i_pll_ctrl1;
  };

  union{ //offset addr 0x00d4
    struct{
      __IO uint32_t :8;
      __IO uint32_t lpf_rz:3;
      __IO uint32_t ib_tpm_prog:3;
      __IO uint32_t lpf_r2:2;
      __IO uint32_t lpf_r1:2;
      __IO uint32_t lpf_c3:2;
      __IO uint32_t lpf_c2:2;
      __IO uint32_t lpf_c1:2;
      __IO uint32_t ib_vco_prog:4;
      __IO uint32_t :4;
    }i_pll_ctrl2_fld;
    __IO uint32_t i_pll_ctrl2;
  };

  union{ //offset addr 0x00d8
    struct{
      __IO uint32_t :5;
      __IO uint32_t vmid_prog:3;
      __IO uint32_t :3;
      __IO uint32_t vco_var_tpm_prog:3;
      __IO uint32_t vco_var_prog:3;
      __IO uint32_t :4;
      __IO uint32_t pfd_pol:1;
      __IO uint32_t :10;
    }i_pll_ctrl3_fld;
    __IO uint32_t i_pll_ctrl3;
  };

  union{ //offset addr 0x00dc
    struct{
      __IO uint32_t :9;
      __IO uint32_t en_rx_test_buffer_1p2v_1:1;
      __IO uint32_t rx_lna_cap_trim:3;
      __IO uint32_t tia_en:1;
      __IO uint32_t rx_en_dcoc_1p2v:1;
      __IO uint32_t en_rx_test_buffer_1p2v_0:1;
      __IO uint32_t rx_tia_i_ctrl:1;
      __IO uint32_t en_pa_1p2v:1;
      __IO uint32_t en_mixer_1p2v:1;
      __IO uint32_t en_lna_1p2v:1;
      __IO uint32_t en_filter_1p2v:1;
      __IO uint32_t en_rx_adc_1p2v:1;
      __IO uint32_t :3;
      __IO uint32_t ldo_lna_out_tune_1p2v:2;
      __IO uint32_t ldo_ana_bb_out_tune_1p2v:2;
      __IO uint32_t rx_bb_vcm_ctrl:1;
      __IO uint32_t en_ldo_lna_pa_1p2v:1;
      __IO uint32_t en_ldo_ana_bb_1p2v:1;
    }i_rx_ctrl0_fld;
    __IO uint32_t i_rx_ctrl0;
  };

  union{ //offset addr 0x00e0
    struct{
      __IO uint32_t :6;
      __IO uint32_t rccal_discharge_c:1;
      __IO uint32_t rccal_dcoc_en:1;
      __IO uint32_t rccal_comparator_clk:1;
      __IO uint32_t rccal_charge_c:1;
      __IO uint32_t rccal_cap_sw:5;
      __IO uint32_t rx_rccal_en:1;
      __IO uint32_t :2;
      __IO uint32_t rx_filter_swap_iq_1p2v:1;
      __IO uint32_t rx_filter_bw_sel_1p2v:1;
      __IO uint32_t :2;
      __IO uint32_t filter_cap_sw_1p2v:4;
      __IO uint32_t :6;
    }i_rx_ctrl1_fld;
    __IO uint32_t i_rx_ctrl1;
  };

  union{ //offset addr 0x00e4
    struct{
      __IO uint32_t sync_astate_out:8;
      __IO uint32_t ri:2;
      __IO uint32_t :1;
      __IO uint32_t rx_demod_check:5;
      __IO uint32_t :6;
      __IO uint32_t freq_est_raw:10;
    }o_dig_bb_ctrl0_fld;
    __IO uint32_t o_dig_bb_ctrl0;
  };

  union{ //offset addr 0x00e8
    struct{
      __IO uint32_t nrssi:8;
      __IO uint32_t arssi:8;
      __IO uint32_t rxabs:8;
      __IO uint32_t rx_demod_check:4;
      __IO uint32_t agc_g_idx:4;
    }o_dig_bb_ctrl1_fld;
    __IO uint32_t o_dig_bb_ctrl1;
  };

  union{ //offset addr 0x00ec
    struct{
      __IO uint32_t dc_cal_state:2;
      __IO uint32_t dc_comp_dac_q:6;
      __IO uint32_t :2;
      __IO uint32_t dc_comp_dac_i:6;
      __IO uint32_t dcq_vec:8;
      __IO uint32_t dci_vec:8;
    }o_dig_bb_ctrl2_fld;
    __IO uint32_t o_dig_bb_ctrl2;
  };

  union{ //offset addr 0x00f0
    struct{
      __IO uint32_t :26;
      __IO uint32_t zb_sfd_sync_err:1;
      __IO uint32_t rx_pdu_type_ind:1;
      __IO uint32_t rx_add_filter_ind1:3;
      __IO uint32_t rx_add_filter_ind0:1;
    }o_dig_bb_ctrl3_fld;
    __IO uint32_t o_dig_bb_ctrl3;
  };

  union{ //offset addr 0x00f4
    struct{
      __IO uint32_t :3;
      __IO uint32_t rcal_fina:5;
      __IO uint32_t :1;
      __IO uint32_t kcal_fina:7;
      __IO uint32_t :7;
      __IO uint32_t ct_word_fina:9;
    }o_dig_bb_ctrl4_fld;
    __IO uint32_t o_dig_bb_ctrl4;
  };

  union{ //offset addr 0x00f8
    struct{
      __IO uint32_t rx_rccal_comparator_out:1;
      __IO uint32_t :8;
      __IO uint32_t lock_ready:1;
      __IO uint32_t :2;
      __IO uint32_t actu_cnt_fina:20;
    }o_dig_bb_ctrl5_fld;
    __IO uint32_t o_dig_bb_ctrl5;
  };

  union{ //offset addr 0x00fc
    struct{
      __IO uint32_t supp_samp_cnt:12;
      __IO uint32_t supp_samp_q:10;
      __IO uint32_t supp_samp_i:10;
    }o_dig_bb_ctrl6_fld;
    __IO uint32_t o_dig_bb_ctrl6;
  };


  union{ //offset addr 0x0104
    struct{
      __IO uint32_t :5;
      __IO uint32_t delay_up:3;
      __IO uint32_t delay_dn:3;
      __IO uint32_t :12;
      __IO uint32_t cp_prog:4;
      __IO uint32_t :4;
      __IO uint32_t en_pllbuf_ctcnt:1;
    }i_pll_ctrl5_fld;
    __IO uint32_t i_pll_ctrl5;
  };

  union{ //offset addr 0x0108
    struct{
      __IO uint32_t :8;
      __IO uint32_t lpf_rz:3;
      __IO uint32_t ib_tpm_prog:3;
      __IO uint32_t lpf_r2:2;
      __IO uint32_t lpf_r1:2;
      __IO uint32_t lpf_c3:2;
      __IO uint32_t lpf_c2:2;
      __IO uint32_t lpf_c1:2;
      __IO uint32_t ib_vco_prog:4;
      __IO uint32_t ib_cp_prog:4;
    }i_pll_ctrl6_fld;
    __IO uint32_t i_pll_ctrl6;
  };

  union{ //offset addr 0x010c
    struct{
      __IO uint32_t :5;
      __IO uint32_t vmid_prog:3;
      __IO uint32_t :3;
      __IO uint32_t vco_var_tpm_prog:3;
      __IO uint32_t vco_var_prog:3;
      __IO uint32_t :4;
      __IO uint32_t pfd_pol:1;
      __IO uint32_t :10;
    }i_pll_ctrl7_fld;
    __IO uint32_t i_pll_ctrl7;
  };

} PICO_REG_BB_TOP_TypeDef;

#define PICO_REG_BB_TOP PICO_REG_BB_TOP_TypeDef *0x40030000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_BB_TOP_H_

