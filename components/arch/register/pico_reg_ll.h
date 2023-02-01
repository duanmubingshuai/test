#ifndef _PICO_REG_LL_H_
#define _PICO_REG_LL_H_

#include <stdint.h>

#define LL_COUNT 47

#define LL_BASE_ADDR 0x40031000

#define LL_SIZE 0x00000C00


 /**
 * @brief ENABLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00               Enable   0
 * </pre>
 */
#define LL_ENABLE_OFFSET 0x00000000


__INLINE uint32_t ll_enable_get(void)
{
    return _PICO_REG_RD(LL_ENABLE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_enable_set(uint32_t value)
{
    _PICO_REG_WR(LL_ENABLE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_ENABLE_ENABLE_BIT                            ((uint32_t)0x00000001)
#define LL_ENABLE_ENABLE_POS                            0

#define LL_ENABLE_ENABLE_RST                            0x0

__INLINE void ll_enable_pack(uint8_t enable)
{
    _PICO_REG_WR(LL_ENABLE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)enable << 0));
}

__INLINE void ll_enable_unpack(uint8_t* enable)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_ENABLE_OFFSET + LL_BASE_ADDR);

    *enable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_enable_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_ENABLE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ll_enable_enable_setf(uint8_t enable)
{
    _PICO_REG_WR(LL_ENABLE_OFFSET+ LL_BASE_ADDR, (uint32_t)enable << 0);
}

 /**
 * @brief MODE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16     RX_packet_number   0b0
 *  15:08     Tx_Packet_number   0b0
 *  03:00                 Mode   0b0
 * </pre>
 */
#define LL_MODE_OFFSET 0x00000004


__INLINE uint32_t ll_mode_get(void)
{
    return _PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_mode_set(uint32_t value)
{
    _PICO_REG_WR(LL_MODE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_MODE_RX_PACKET_NUMBER_MASK                 ((uint32_t)0x00FF0000)
#define LL_MODE_RX_PACKET_NUMBER_LSB                  16
#define LL_MODE_RX_PACKET_NUMBER_WIDTH                ((uint32_t)0x00000008)
#define LL_MODE_TX_PACKET_NUMBER_MASK                 ((uint32_t)0x0000FF00)
#define LL_MODE_TX_PACKET_NUMBER_LSB                  8
#define LL_MODE_TX_PACKET_NUMBER_WIDTH                ((uint32_t)0x00000008)
#define LL_MODE_MODE_MASK                             ((uint32_t)0x0000000F)
#define LL_MODE_MODE_LSB                              0
#define LL_MODE_MODE_WIDTH                            ((uint32_t)0x00000004)

#define LL_MODE_RX_PACKET_NUMBER_RST                  0x0
#define LL_MODE_TX_PACKET_NUMBER_RST                  0x0
#define LL_MODE_MODE_RST                              0x0

__INLINE void ll_mode_pack(uint8_t rxpacketnumber, uint8_t txpacketnumber, uint8_t mode)
{
    _PICO_REG_WR(LL_MODE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rxpacketnumber << 16) | ((uint32_t)txpacketnumber << 8) | ((uint32_t)mode << 0));
}

__INLINE void ll_mode_unpack(uint8_t* rxpacketnumber, uint8_t* txpacketnumber, uint8_t* mode)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR);

    *rxpacketnumber = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *txpacketnumber = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *mode = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ll_mode_rx_packet_number_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE void ll_mode_rx_packet_number_setf(uint8_t rxpacketnumber)
{
    _PICO_REG_WR(LL_MODE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00FF0000)) | ((uint32_t)rxpacketnumber << 16));
}

__INLINE uint8_t ll_mode_tx_packet_number_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void ll_mode_tx_packet_number_setf(uint8_t txpacketnumber)
{
    _PICO_REG_WR(LL_MODE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)txpacketnumber << 8));
}

__INLINE uint8_t ll_mode_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ll_mode_mode_setf(uint8_t mode)
{
    _PICO_REG_WR(LL_MODE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x0000000F)) | ((uint32_t)mode << 0));
}

 /**
 * @brief STATE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00                state   0b0
 * </pre>
 */
#define LL_STATE_OFFSET 0x00000008


__INLINE uint32_t ll_state_get(void)
{
    return _PICO_REG_RD(LL_STATE_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_STATE_STATE_MASK                            ((uint32_t)0x0000000F)
#define LL_STATE_STATE_LSB                             0
#define LL_STATE_STATE_WIDTH                           ((uint32_t)0x00000004)

#define LL_STATE_STATE_RST                             0x0

__INLINE void ll_state_unpack(uint8_t* state)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_STATE_OFFSET + LL_BASE_ADDR);

    *state = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ll_state_state_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_STATE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief MODE1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     13             nack_int   0
 *     12         loop_timeout   0
 *     11            Crc_err_2   0
 *     10               CRC_OK   0
 *     09              Rx_done   0
 *     08              Tx_done   0
 *     04    Rx_fifo_over_half   0
 *     03         rx_fifo_full   0
 *     02           Rx_timeout   0
 *     01         RX_CRC_error   0
 *     00            Mode_done   0
 * </pre>
 */
#define LL_MODE1_OFFSET 0x0000000C


__INLINE uint32_t ll_mode1_get(void)
{
    return _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_mode1_set(uint32_t value)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_MODE1_NACK_INT_BIT                          ((uint32_t)0x00002000)
#define LL_MODE1_NACK_INT_POS                          13
#define LL_MODE1_LOOP_TIMEOUT_BIT                      ((uint32_t)0x00001000)
#define LL_MODE1_LOOP_TIMEOUT_POS                      12
#define LL_MODE1_CRC_ERR_2_BIT                         ((uint32_t)0x00000800)
#define LL_MODE1_CRC_ERR_2_POS                         11
#define LL_MODE1_CRC_OK_BIT                            ((uint32_t)0x00000400)
#define LL_MODE1_CRC_OK_POS                            10
#define LL_MODE1_RX_DONE_BIT                           ((uint32_t)0x00000200)
#define LL_MODE1_RX_DONE_POS                           9
#define LL_MODE1_TX_DONE_BIT                           ((uint32_t)0x00000100)
#define LL_MODE1_TX_DONE_POS                           8
#define LL_MODE1_RX_FIFO_OVER_HALF_BIT                 ((uint32_t)0x00000010)
#define LL_MODE1_RX_FIFO_OVER_HALF_POS                 4
#define LL_MODE1_RX_FIFO_FULL_BIT                      ((uint32_t)0x00000008)
#define LL_MODE1_RX_FIFO_FULL_POS                      3
#define LL_MODE1_RX_TIMEOUT_BIT                        ((uint32_t)0x00000004)
#define LL_MODE1_RX_TIMEOUT_POS                        2
#define LL_MODE1_RX_CRC_ERROR_BIT                      ((uint32_t)0x00000002)
#define LL_MODE1_RX_CRC_ERROR_POS                      1
#define LL_MODE1_MODE_DONE_BIT                         ((uint32_t)0x00000001)
#define LL_MODE1_MODE_DONE_POS                         0

#define LL_MODE1_NACK_INT_RST                          0x0
#define LL_MODE1_LOOP_TIMEOUT_RST                      0x0
#define LL_MODE1_CRC_ERR_2_RST                         0x0
#define LL_MODE1_CRC_OK_RST                            0x0
#define LL_MODE1_RX_DONE_RST                           0x0
#define LL_MODE1_TX_DONE_RST                           0x0
#define LL_MODE1_RX_FIFO_OVER_HALF_RST                 0x0
#define LL_MODE1_RX_FIFO_FULL_RST                      0x0
#define LL_MODE1_RX_TIMEOUT_RST                        0x0
#define LL_MODE1_RX_CRC_ERROR_RST                      0x0
#define LL_MODE1_MODE_DONE_RST                         0x0

__INLINE void ll_mode1_pack(uint8_t nackint, uint8_t looptimeout, uint8_t crcerr2, uint8_t crcok, uint8_t rxdone, uint8_t txdone, uint8_t rxfifooverhalf, uint8_t rxfifofull, uint8_t rxtimeout, uint8_t rxcrcerror, uint8_t modedone)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR,  ((uint32_t)nackint << 13) | ((uint32_t)looptimeout << 12) | ((uint32_t)crcerr2 << 11) | ((uint32_t)crcok << 10) | ((uint32_t)rxdone << 9) | ((uint32_t)txdone << 8) | ((uint32_t)rxfifooverhalf << 4) | ((uint32_t)rxfifofull << 3) | ((uint32_t)rxtimeout << 2) | ((uint32_t)rxcrcerror << 1) | ((uint32_t)modedone << 0));
}

__INLINE void ll_mode1_unpack(uint8_t* nackint, uint8_t* looptimeout, uint8_t* crcerr2, uint8_t* crcok, uint8_t* rxdone, uint8_t* txdone, uint8_t* rxfifooverhalf, uint8_t* rxfifofull, uint8_t* rxtimeout, uint8_t* rxcrcerror, uint8_t* modedone)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);

    *nackint = (localVal & ((uint32_t)0x00002000)) >> 13;
    *looptimeout = (localVal & ((uint32_t)0x00001000)) >> 12;
    *crcerr2 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *crcok = (localVal & ((uint32_t)0x00000400)) >> 10;
    *rxdone = (localVal & ((uint32_t)0x00000200)) >> 9;
    *txdone = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxfifooverhalf = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxfifofull = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxtimeout = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxcrcerror = (localVal & ((uint32_t)0x00000002)) >> 1;
    *modedone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_mode1_nack_int_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ll_mode1_nack_int_setf(uint8_t nackint)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)nackint << 13));
}

__INLINE uint8_t ll_mode1_loop_timeout_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ll_mode1_loop_timeout_setf(uint8_t looptimeout)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)looptimeout << 12));
}

__INLINE uint8_t ll_mode1_crc_err_2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ll_mode1_crc_err_2_setf(uint8_t crcerr2)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)crcerr2 << 11));
}

__INLINE uint8_t ll_mode1_crc_ok_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ll_mode1_crc_ok_setf(uint8_t crcok)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)crcok << 10));
}

__INLINE uint8_t ll_mode1_rx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ll_mode1_rx_done_setf(uint8_t rxdone)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)rxdone << 9));
}

__INLINE uint8_t ll_mode1_tx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ll_mode1_tx_done_setf(uint8_t txdone)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)txdone << 8));
}

__INLINE uint8_t ll_mode1_rx_fifo_over_half_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ll_mode1_rx_fifo_over_half_setf(uint8_t rxfifooverhalf)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxfifooverhalf << 4));
}

__INLINE uint8_t ll_mode1_rx_fifo_full_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ll_mode1_rx_fifo_full_setf(uint8_t rxfifofull)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rxfifofull << 3));
}

__INLINE uint8_t ll_mode1_rx_timeout_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ll_mode1_rx_timeout_setf(uint8_t rxtimeout)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rxtimeout << 2));
}

__INLINE uint8_t ll_mode1_rx_crc_error_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ll_mode1_rx_crc_error_setf(uint8_t rxcrcerror)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rxcrcerror << 1));
}

__INLINE uint8_t ll_mode1_mode_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ll_mode1_mode_done_setf(uint8_t modedone)
{
    _PICO_REG_WR(LL_MODE1_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE1_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)modedone << 0));
}

 /**
 * @brief MODE2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     13             nack_int   0
 *     12         loop_timeout   0
 *     11            Crc_err_2   0
 *     10               CRC_OK   0
 *     09              Rx_done   0
 *     08              Tx_done   0
 *     04    Rx_fifo_over_half   0
 *     03         rx_fifo_full   0
 *     02           Rx_timeout   0
 *     01         RX_CRC_error   0
 *     00            Mode_done   0
 * </pre>
 */
#define LL_MODE2_OFFSET 0x00000010


__INLINE uint32_t ll_mode2_get(void)
{
    return _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_MODE2_NACK_INT_BIT                          ((uint32_t)0x00002000)
#define LL_MODE2_NACK_INT_POS                          13
#define LL_MODE2_LOOP_TIMEOUT_BIT                      ((uint32_t)0x00001000)
#define LL_MODE2_LOOP_TIMEOUT_POS                      12
#define LL_MODE2_CRC_ERR_2_BIT                         ((uint32_t)0x00000800)
#define LL_MODE2_CRC_ERR_2_POS                         11
#define LL_MODE2_CRC_OK_BIT                            ((uint32_t)0x00000400)
#define LL_MODE2_CRC_OK_POS                            10
#define LL_MODE2_RX_DONE_BIT                           ((uint32_t)0x00000200)
#define LL_MODE2_RX_DONE_POS                           9
#define LL_MODE2_TX_DONE_BIT                           ((uint32_t)0x00000100)
#define LL_MODE2_TX_DONE_POS                           8
#define LL_MODE2_RX_FIFO_OVER_HALF_BIT                 ((uint32_t)0x00000010)
#define LL_MODE2_RX_FIFO_OVER_HALF_POS                 4
#define LL_MODE2_RX_FIFO_FULL_BIT                      ((uint32_t)0x00000008)
#define LL_MODE2_RX_FIFO_FULL_POS                      3
#define LL_MODE2_RX_TIMEOUT_BIT                        ((uint32_t)0x00000004)
#define LL_MODE2_RX_TIMEOUT_POS                        2
#define LL_MODE2_RX_CRC_ERROR_BIT                      ((uint32_t)0x00000002)
#define LL_MODE2_RX_CRC_ERROR_POS                      1
#define LL_MODE2_MODE_DONE_BIT                         ((uint32_t)0x00000001)
#define LL_MODE2_MODE_DONE_POS                         0

#define LL_MODE2_NACK_INT_RST                          0x0
#define LL_MODE2_LOOP_TIMEOUT_RST                      0x0
#define LL_MODE2_CRC_ERR_2_RST                         0x0
#define LL_MODE2_CRC_OK_RST                            0x0
#define LL_MODE2_RX_DONE_RST                           0x0
#define LL_MODE2_TX_DONE_RST                           0x0
#define LL_MODE2_RX_FIFO_OVER_HALF_RST                 0x0
#define LL_MODE2_RX_FIFO_FULL_RST                      0x0
#define LL_MODE2_RX_TIMEOUT_RST                        0x0
#define LL_MODE2_RX_CRC_ERROR_RST                      0x0
#define LL_MODE2_MODE_DONE_RST                         0x0

__INLINE void ll_mode2_unpack(uint8_t* nackint, uint8_t* looptimeout, uint8_t* crcerr2, uint8_t* crcok, uint8_t* rxdone, uint8_t* txdone, uint8_t* rxfifooverhalf, uint8_t* rxfifofull, uint8_t* rxtimeout, uint8_t* rxcrcerror, uint8_t* modedone)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);

    *nackint = (localVal & ((uint32_t)0x00002000)) >> 13;
    *looptimeout = (localVal & ((uint32_t)0x00001000)) >> 12;
    *crcerr2 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *crcok = (localVal & ((uint32_t)0x00000400)) >> 10;
    *rxdone = (localVal & ((uint32_t)0x00000200)) >> 9;
    *txdone = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxfifooverhalf = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxfifofull = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxtimeout = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxcrcerror = (localVal & ((uint32_t)0x00000002)) >> 1;
    *modedone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_mode2_nack_int_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t ll_mode2_loop_timeout_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t ll_mode2_crc_err_2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t ll_mode2_crc_ok_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t ll_mode2_rx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t ll_mode2_tx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t ll_mode2_rx_fifo_over_half_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t ll_mode2_rx_fifo_full_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t ll_mode2_rx_timeout_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ll_mode2_rx_crc_error_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ll_mode2_mode_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief MODE3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     13             nack_int   0
 *     12         loop_timeout   0
 *     11            Crc_err_2   0
 *     10               CRC_OK   0
 *     09              Rx_done   0
 *     08              Tx_done   0
 *     04   Rx_fifo_over_half_   0
 *     03         rx_fifo_full   0
 *     02           Rx_timeout   0
 *     01         RX_CRC_error   0
 *     00            Mode_done   0
 * </pre>
 */
#define LL_MODE3_OFFSET 0x00000014


__INLINE uint32_t ll_mode3_get(void)
{
    return _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_mode3_set(uint32_t value)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_MODE3_NACK_INT_BIT                          ((uint32_t)0x00002000)
#define LL_MODE3_NACK_INT_POS                          13
#define LL_MODE3_LOOP_TIMEOUT_BIT                      ((uint32_t)0x00001000)
#define LL_MODE3_LOOP_TIMEOUT_POS                      12
#define LL_MODE3_CRC_ERR_2_BIT                         ((uint32_t)0x00000800)
#define LL_MODE3_CRC_ERR_2_POS                         11
#define LL_MODE3_CRC_OK_BIT                            ((uint32_t)0x00000400)
#define LL_MODE3_CRC_OK_POS                            10
#define LL_MODE3_RX_DONE_BIT                           ((uint32_t)0x00000200)
#define LL_MODE3_RX_DONE_POS                           9
#define LL_MODE3_TX_DONE_BIT                           ((uint32_t)0x00000100)
#define LL_MODE3_TX_DONE_POS                           8
#define LL_MODE3_RX_FIFO_OVER_HALF__BIT                ((uint32_t)0x00000010)
#define LL_MODE3_RX_FIFO_OVER_HALF__POS                4
#define LL_MODE3_RX_FIFO_FULL_BIT                      ((uint32_t)0x00000008)
#define LL_MODE3_RX_FIFO_FULL_POS                      3
#define LL_MODE3_RX_TIMEOUT_BIT                        ((uint32_t)0x00000004)
#define LL_MODE3_RX_TIMEOUT_POS                        2
#define LL_MODE3_RX_CRC_ERROR_BIT                      ((uint32_t)0x00000002)
#define LL_MODE3_RX_CRC_ERROR_POS                      1
#define LL_MODE3_MODE_DONE_BIT                         ((uint32_t)0x00000001)
#define LL_MODE3_MODE_DONE_POS                         0

#define LL_MODE3_NACK_INT_RST                          0x0
#define LL_MODE3_LOOP_TIMEOUT_RST                      0x0
#define LL_MODE3_CRC_ERR_2_RST                         0x0
#define LL_MODE3_CRC_OK_RST                            0x0
#define LL_MODE3_RX_DONE_RST                           0x0
#define LL_MODE3_TX_DONE_RST                           0x0
#define LL_MODE3_RX_FIFO_OVER_HALF__RST                0x0
#define LL_MODE3_RX_FIFO_FULL_RST                      0x0
#define LL_MODE3_RX_TIMEOUT_RST                        0x0
#define LL_MODE3_RX_CRC_ERROR_RST                      0x0
#define LL_MODE3_MODE_DONE_RST                         0x0

__INLINE void ll_mode3_pack(uint8_t nackint, uint8_t looptimeout, uint8_t crcerr2, uint8_t crcok, uint8_t rxdone, uint8_t txdone, uint8_t rxfifooverhalf, uint8_t rxfifofull, uint8_t rxtimeout, uint8_t rxcrcerror, uint8_t modedone)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR,  ((uint32_t)nackint << 13) | ((uint32_t)looptimeout << 12) | ((uint32_t)crcerr2 << 11) | ((uint32_t)crcok << 10) | ((uint32_t)rxdone << 9) | ((uint32_t)txdone << 8) | ((uint32_t)rxfifooverhalf << 4) | ((uint32_t)rxfifofull << 3) | ((uint32_t)rxtimeout << 2) | ((uint32_t)rxcrcerror << 1) | ((uint32_t)modedone << 0));
}

__INLINE void ll_mode3_unpack(uint8_t* nackint, uint8_t* looptimeout, uint8_t* crcerr2, uint8_t* crcok, uint8_t* rxdone, uint8_t* txdone, uint8_t* rxfifooverhalf, uint8_t* rxfifofull, uint8_t* rxtimeout, uint8_t* rxcrcerror, uint8_t* modedone)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);

    *nackint = (localVal & ((uint32_t)0x00002000)) >> 13;
    *looptimeout = (localVal & ((uint32_t)0x00001000)) >> 12;
    *crcerr2 = (localVal & ((uint32_t)0x00000800)) >> 11;
    *crcok = (localVal & ((uint32_t)0x00000400)) >> 10;
    *rxdone = (localVal & ((uint32_t)0x00000200)) >> 9;
    *txdone = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxfifooverhalf = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rxfifofull = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxtimeout = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxcrcerror = (localVal & ((uint32_t)0x00000002)) >> 1;
    *modedone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_mode3_nack_int_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE void ll_mode3_nack_int_setf(uint8_t nackint)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00002000)) | ((uint32_t)nackint << 13));
}

__INLINE uint8_t ll_mode3_loop_timeout_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void ll_mode3_loop_timeout_setf(uint8_t looptimeout)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)looptimeout << 12));
}

__INLINE uint8_t ll_mode3_crc_err_2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void ll_mode3_crc_err_2_setf(uint8_t crcerr2)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)crcerr2 << 11));
}

__INLINE uint8_t ll_mode3_crc_ok_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ll_mode3_crc_ok_setf(uint8_t crcok)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)crcok << 10));
}

__INLINE uint8_t ll_mode3_rx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ll_mode3_rx_done_setf(uint8_t rxdone)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)rxdone << 9));
}

__INLINE uint8_t ll_mode3_tx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ll_mode3_tx_done_setf(uint8_t txdone)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)txdone << 8));
}

__INLINE uint8_t ll_mode3_rx_fifo_over_half__getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void ll_mode3_rx_fifo_over_half__setf(uint8_t rxfifooverhalf)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rxfifooverhalf << 4));
}

__INLINE uint8_t ll_mode3_rx_fifo_full_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void ll_mode3_rx_fifo_full_setf(uint8_t rxfifofull)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rxfifofull << 3));
}

__INLINE uint8_t ll_mode3_rx_timeout_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ll_mode3_rx_timeout_setf(uint8_t rxtimeout)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rxtimeout << 2));
}

__INLINE uint8_t ll_mode3_rx_crc_error_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ll_mode3_rx_crc_error_setf(uint8_t rxcrcerror)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rxcrcerror << 1));
}

__INLINE uint8_t ll_mode3_mode_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ll_mode3_mode_done_setf(uint8_t modedone)
{
    _PICO_REG_WR(LL_MODE3_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_MODE3_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)modedone << 0));
}

 /**
 * @brief TX_TO_RX_INTERVAL_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00   TX_to_RX_interval_value   0b0
 * </pre>
 */
#define LL_TX_TO_RX_INTERVAL_VALUE_OFFSET 0x00000018


__INLINE uint32_t ll_tx_to_rx_interval_value_get(void)
{
    return _PICO_REG_RD(LL_TX_TO_RX_INTERVAL_VALUE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_tx_to_rx_interval_value_set(uint32_t value)
{
    _PICO_REG_WR(LL_TX_TO_RX_INTERVAL_VALUE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_TX_TO_RX_INTERVAL_VALUE_TX_TO_RX_INTERVAL_VALUE_MASK          ((uint32_t)0xFFFFFFFF)
#define LL_TX_TO_RX_INTERVAL_VALUE_TX_TO_RX_INTERVAL_VALUE_LSB           0
#define LL_TX_TO_RX_INTERVAL_VALUE_TX_TO_RX_INTERVAL_VALUE_WIDTH         ((uint32_t)0x00000020)

#define LL_TX_TO_RX_INTERVAL_VALUE_TX_TO_RX_INTERVAL_VALUE_RST           0x0

__INLINE void ll_tx_to_rx_interval_value_pack(uint32_t txtorxintervalvalue)
{
    _PICO_REG_WR(LL_TX_TO_RX_INTERVAL_VALUE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)txtorxintervalvalue << 0));
}

__INLINE void ll_tx_to_rx_interval_value_unpack(uint8_t* txtorxintervalvalue)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_TO_RX_INTERVAL_VALUE_OFFSET + LL_BASE_ADDR);

    *txtorxintervalvalue = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_tx_to_rx_interval_value_tx_to_rx_interval_value_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_TO_RX_INTERVAL_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void ll_tx_to_rx_interval_value_tx_to_rx_interval_value_setf(uint32_t txtorxintervalvalue)
{
    _PICO_REG_WR(LL_TX_TO_RX_INTERVAL_VALUE_OFFSET+ LL_BASE_ADDR, (uint32_t)txtorxintervalvalue << 0);
}

 /**
 * @brief TX_TO_RX_INTERVAL_VALUE1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00   RX_to_TX_interval_value1   0b0
 * </pre>
 */
#define LL_TX_TO_RX_INTERVAL_VALUE1_OFFSET 0x0000001C


__INLINE uint32_t ll_tx_to_rx_interval_value1_get(void)
{
    return _PICO_REG_RD(LL_TX_TO_RX_INTERVAL_VALUE1_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_tx_to_rx_interval_value1_set(uint32_t value)
{
    _PICO_REG_WR(LL_TX_TO_RX_INTERVAL_VALUE1_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_TX_TO_RX_INTERVAL_VALUE1_RX_TO_TX_INTERVAL_VALUE1_MASK         ((uint32_t)0xFFFFFFFF)
#define LL_TX_TO_RX_INTERVAL_VALUE1_RX_TO_TX_INTERVAL_VALUE1_LSB          0
#define LL_TX_TO_RX_INTERVAL_VALUE1_RX_TO_TX_INTERVAL_VALUE1_WIDTH        ((uint32_t)0x00000020)

#define LL_TX_TO_RX_INTERVAL_VALUE1_RX_TO_TX_INTERVAL_VALUE1_RST          0x0

__INLINE void ll_tx_to_rx_interval_value1_pack(uint32_t rxtotxintervalvalue1)
{
    _PICO_REG_WR(LL_TX_TO_RX_INTERVAL_VALUE1_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rxtotxintervalvalue1 << 0));
}

__INLINE void ll_tx_to_rx_interval_value1_unpack(uint8_t* rxtotxintervalvalue1)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_TO_RX_INTERVAL_VALUE1_OFFSET + LL_BASE_ADDR);

    *rxtotxintervalvalue1 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_tx_to_rx_interval_value1_rx_to_tx_interval_value1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_TO_RX_INTERVAL_VALUE1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void ll_tx_to_rx_interval_value1_rx_to_tx_interval_value1_setf(uint32_t rxtotxintervalvalue1)
{
    _PICO_REG_WR(LL_TX_TO_RX_INTERVAL_VALUE1_OFFSET+ LL_BASE_ADDR, (uint32_t)rxtotxintervalvalue1 << 0);
}

 /**
 * @brief TX_AND_RX_EN_RELEASE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16    PHY_TX_en_release   0b0
 *  15:00    PHY_RX_en_release   0b0
 * </pre>
 */
#define LL_TX_AND_RX_EN_RELEASE_OFFSET 0x00000020


__INLINE uint32_t ll_tx_and_rx_en_release_get(void)
{
    return _PICO_REG_RD(LL_TX_AND_RX_EN_RELEASE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_tx_and_rx_en_release_set(uint32_t value)
{
    _PICO_REG_WR(LL_TX_AND_RX_EN_RELEASE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_TX_AND_RX_EN_RELEASE_PHY_TX_EN_RELEASE_MASK                ((uint32_t)0xFFFF0000)
#define LL_TX_AND_RX_EN_RELEASE_PHY_TX_EN_RELEASE_LSB                 16
#define LL_TX_AND_RX_EN_RELEASE_PHY_TX_EN_RELEASE_WIDTH               ((uint32_t)0x00000010)
#define LL_TX_AND_RX_EN_RELEASE_PHY_RX_EN_RELEASE_MASK                ((uint32_t)0x0000FFFF)
#define LL_TX_AND_RX_EN_RELEASE_PHY_RX_EN_RELEASE_LSB                 0
#define LL_TX_AND_RX_EN_RELEASE_PHY_RX_EN_RELEASE_WIDTH               ((uint32_t)0x00000010)

#define LL_TX_AND_RX_EN_RELEASE_PHY_TX_EN_RELEASE_RST                 0x0
#define LL_TX_AND_RX_EN_RELEASE_PHY_RX_EN_RELEASE_RST                 0x0

__INLINE void ll_tx_and_rx_en_release_pack(uint16_t phytxenrelease, uint16_t phyrxenrelease)
{
    _PICO_REG_WR(LL_TX_AND_RX_EN_RELEASE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)phytxenrelease << 16) | ((uint32_t)phyrxenrelease << 0));
}

__INLINE void ll_tx_and_rx_en_release_unpack(uint8_t* phytxenrelease, uint8_t* phyrxenrelease)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_AND_RX_EN_RELEASE_OFFSET + LL_BASE_ADDR);

    *phytxenrelease = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *phyrxenrelease = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ll_tx_and_rx_en_release_phy_tx_en_release_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_AND_RX_EN_RELEASE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ll_tx_and_rx_en_release_phy_tx_en_release_setf(uint16_t phytxenrelease)
{
    _PICO_REG_WR(LL_TX_AND_RX_EN_RELEASE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_TX_AND_RX_EN_RELEASE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0xFFFF0000)) | ((uint32_t)phytxenrelease << 16));
}

__INLINE uint16_t ll_tx_and_rx_en_release_phy_rx_en_release_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_AND_RX_EN_RELEASE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ll_tx_and_rx_en_release_phy_rx_en_release_setf(uint16_t phyrxenrelease)
{
    _PICO_REG_WR(LL_TX_AND_RX_EN_RELEASE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_TX_AND_RX_EN_RELEASE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x0000FFFF)) | ((uint32_t)phyrxenrelease << 0));
}

 /**
 * @brief RX_TIME_OUT_1ST register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00      rx_time_out_1st   0b280
 * </pre>
 */
#define LL_RX_TIME_OUT_1ST_OFFSET 0x00000024


__INLINE uint32_t ll_rx_time_out_1st_get(void)
{
    return _PICO_REG_RD(LL_RX_TIME_OUT_1ST_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_rx_time_out_1st_set(uint32_t value)
{
    _PICO_REG_WR(LL_RX_TIME_OUT_1ST_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_RX_TIME_OUT_1ST_RX_TIME_OUT_1ST_MASK                  ((uint32_t)0x0000FFFF)
#define LL_RX_TIME_OUT_1ST_RX_TIME_OUT_1ST_LSB                   0
#define LL_RX_TIME_OUT_1ST_RX_TIME_OUT_1ST_WIDTH                 ((uint32_t)0x00000010)

#define LL_RX_TIME_OUT_1ST_RX_TIME_OUT_1ST_RST                   0x280

__INLINE void ll_rx_time_out_1st_pack(uint16_t rxtimeout1st)
{
    _PICO_REG_WR(LL_RX_TIME_OUT_1ST_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rxtimeout1st << 0));
}

__INLINE void ll_rx_time_out_1st_unpack(uint8_t* rxtimeout1st)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_TIME_OUT_1ST_OFFSET + LL_BASE_ADDR);

    *rxtimeout1st = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ll_rx_time_out_1st_rx_time_out_1st_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_TIME_OUT_1ST_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ll_rx_time_out_1st_rx_time_out_1st_setf(uint16_t rxtimeout1st)
{
    _PICO_REG_WR(LL_RX_TIME_OUT_1ST_OFFSET+ LL_BASE_ADDR, (uint32_t)rxtimeout1st << 0);
}

 /**
 * @brief RX_TIME_OUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          rx_time_out   0b280
 * </pre>
 */
#define LL_RX_TIME_OUT_OFFSET 0x00000028


__INLINE uint32_t ll_rx_time_out_get(void)
{
    return _PICO_REG_RD(LL_RX_TIME_OUT_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_rx_time_out_set(uint32_t value)
{
    _PICO_REG_WR(LL_RX_TIME_OUT_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_RX_TIME_OUT_RX_TIME_OUT_MASK                      ((uint32_t)0x0000FFFF)
#define LL_RX_TIME_OUT_RX_TIME_OUT_LSB                       0
#define LL_RX_TIME_OUT_RX_TIME_OUT_WIDTH                     ((uint32_t)0x00000010)

#define LL_RX_TIME_OUT_RX_TIME_OUT_RST                       0x280

__INLINE void ll_rx_time_out_pack(uint16_t rxtimeout)
{
    _PICO_REG_WR(LL_RX_TIME_OUT_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rxtimeout << 0));
}

__INLINE void ll_rx_time_out_unpack(uint8_t* rxtimeout)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_TIME_OUT_OFFSET + LL_BASE_ADDR);

    *rxtimeout = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ll_rx_time_out_rx_time_out_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_TIME_OUT_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void ll_rx_time_out_rx_time_out_setf(uint16_t rxtimeout)
{
    _PICO_REG_WR(LL_RX_TIME_OUT_OFFSET+ LL_BASE_ADDR, (uint32_t)rxtimeout << 0);
}

 /**
 * @brief RX_PKT_HEADER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:16   TX_empty_packet_header   0b0
 *  15:00        rx_pkt_header   0b0
 * </pre>
 */
#define LL_RX_PKT_HEADER_OFFSET 0x0000002C


__INLINE uint32_t ll_rx_pkt_header_get(void)
{
    return _PICO_REG_RD(LL_RX_PKT_HEADER_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_rx_pkt_header_set(uint32_t value)
{
    _PICO_REG_WR(LL_RX_PKT_HEADER_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_RX_PKT_HEADER_TX_EMPTY_PACKET_HEADER_MASK           ((uint32_t)0xFFFF0000)
#define LL_RX_PKT_HEADER_TX_EMPTY_PACKET_HEADER_LSB            16
#define LL_RX_PKT_HEADER_TX_EMPTY_PACKET_HEADER_WIDTH          ((uint32_t)0x00000010)
#define LL_RX_PKT_HEADER_RX_PKT_HEADER_MASK                    ((uint32_t)0x0000FFFF)
#define LL_RX_PKT_HEADER_RX_PKT_HEADER_LSB                     0
#define LL_RX_PKT_HEADER_RX_PKT_HEADER_WIDTH                   ((uint32_t)0x00000010)

#define LL_RX_PKT_HEADER_TX_EMPTY_PACKET_HEADER_RST            0x0
#define LL_RX_PKT_HEADER_RX_PKT_HEADER_RST                     0x0

__INLINE void ll_rx_pkt_header_pack(uint16_t txemptypacketheader)
{
    _PICO_REG_WR(LL_RX_PKT_HEADER_OFFSET+ LL_BASE_ADDR,  ((uint32_t)txemptypacketheader << 16));
}

__INLINE void ll_rx_pkt_header_unpack(uint8_t* txemptypacketheader, uint8_t* rxpktheader)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PKT_HEADER_OFFSET + LL_BASE_ADDR);

    *txemptypacketheader = (localVal & ((uint32_t)0xFFFF0000)) >> 16;
    *rxpktheader = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ll_rx_pkt_header_tx_empty_packet_header_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PKT_HEADER_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFF0000)) >> 16);
}

__INLINE void ll_rx_pkt_header_tx_empty_packet_header_setf(uint16_t txemptypacketheader)
{
    _PICO_REG_WR(LL_RX_PKT_HEADER_OFFSET+ LL_BASE_ADDR, (uint32_t)txemptypacketheader << 16);
}

__INLINE uint16_t ll_rx_pkt_header_rx_pkt_header_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PKT_HEADER_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

 /**
 * @brief RX_PACKET_NUMBER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16   Rx_crc_error_count   0b0
 *  15:08         Rx_total_cnt   0b0
 *  07:00     RX_packet_number   0b0
 * </pre>
 */
#define LL_RX_PACKET_NUMBER_OFFSET 0x00000030


__INLINE uint32_t ll_rx_packet_number_get(void)
{
    return _PICO_REG_RD(LL_RX_PACKET_NUMBER_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_RX_PACKET_NUMBER_RX_CRC_ERROR_COUNT_MASK               ((uint32_t)0x00FF0000)
#define LL_RX_PACKET_NUMBER_RX_CRC_ERROR_COUNT_LSB                16
#define LL_RX_PACKET_NUMBER_RX_CRC_ERROR_COUNT_WIDTH              ((uint32_t)0x00000008)
#define LL_RX_PACKET_NUMBER_RX_TOTAL_CNT_MASK                     ((uint32_t)0x0000FF00)
#define LL_RX_PACKET_NUMBER_RX_TOTAL_CNT_LSB                      8
#define LL_RX_PACKET_NUMBER_RX_TOTAL_CNT_WIDTH                    ((uint32_t)0x00000008)
#define LL_RX_PACKET_NUMBER_RX_PACKET_NUMBER_MASK                 ((uint32_t)0x000000FF)
#define LL_RX_PACKET_NUMBER_RX_PACKET_NUMBER_LSB                  0
#define LL_RX_PACKET_NUMBER_RX_PACKET_NUMBER_WIDTH                ((uint32_t)0x00000008)

#define LL_RX_PACKET_NUMBER_RX_CRC_ERROR_COUNT_RST                0x0
#define LL_RX_PACKET_NUMBER_RX_TOTAL_CNT_RST                      0x0
#define LL_RX_PACKET_NUMBER_RX_PACKET_NUMBER_RST                  0x0

__INLINE void ll_rx_packet_number_unpack(uint8_t* rxcrcerrorcount, uint8_t* rxtotalcnt, uint8_t* rxpacketnumber)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PACKET_NUMBER_OFFSET + LL_BASE_ADDR);

    *rxcrcerrorcount = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *rxtotalcnt = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *rxpacketnumber = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ll_rx_packet_number_rx_crc_error_count_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PACKET_NUMBER_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t ll_rx_packet_number_rx_total_cnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PACKET_NUMBER_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t ll_rx_packet_number_rx_packet_number_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PACKET_NUMBER_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

 /**
 * @brief TX_ACK_COUNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:08        TX_NACK_count   0b0
 *  07:00         TX_ACK_count   0b0
 * </pre>
 */
#define LL_TX_ACK_COUNT_OFFSET 0x00000034


__INLINE uint32_t ll_tx_ack_count_get(void)
{
    return _PICO_REG_RD(LL_TX_ACK_COUNT_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_TX_ACK_COUNT_TX_NACK_COUNT_MASK                    ((uint32_t)0x00000F00)
#define LL_TX_ACK_COUNT_TX_NACK_COUNT_LSB                     8
#define LL_TX_ACK_COUNT_TX_NACK_COUNT_WIDTH                   ((uint32_t)0x00000004)
#define LL_TX_ACK_COUNT_TX_ACK_COUNT_MASK                     ((uint32_t)0x000000FF)
#define LL_TX_ACK_COUNT_TX_ACK_COUNT_LSB                      0
#define LL_TX_ACK_COUNT_TX_ACK_COUNT_WIDTH                    ((uint32_t)0x00000008)

#define LL_TX_ACK_COUNT_TX_NACK_COUNT_RST                     0x0
#define LL_TX_ACK_COUNT_TX_ACK_COUNT_RST                      0x0

__INLINE void ll_tx_ack_count_unpack(uint8_t* txnackcount, uint8_t* txackcount)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_ACK_COUNT_OFFSET + LL_BASE_ADDR);

    *txnackcount = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *txackcount = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ll_tx_ack_count_tx_nack_count_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_ACK_COUNT_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE uint8_t ll_tx_ack_count_tx_ack_count_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_ACK_COUNT_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

 /**
 * @brief BYPASS_SOFT_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02            md_bypass   0
 *     01            sn_bypass   0
 *     00          nesn_bypass   0
 * </pre>
 */
#define LL_BYPASS_SOFT_VALUE_OFFSET 0x00000038


__INLINE uint32_t ll_bypass_soft_value_get(void)
{
    return _PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_bypass_soft_value_set(uint32_t value)
{
    _PICO_REG_WR(LL_BYPASS_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_BYPASS_SOFT_VALUE_MD_BYPASS_BIT                         ((uint32_t)0x00000004)
#define LL_BYPASS_SOFT_VALUE_MD_BYPASS_POS                         2
#define LL_BYPASS_SOFT_VALUE_SN_BYPASS_BIT                         ((uint32_t)0x00000002)
#define LL_BYPASS_SOFT_VALUE_SN_BYPASS_POS                         1
#define LL_BYPASS_SOFT_VALUE_NESN_BYPASS_BIT                       ((uint32_t)0x00000001)
#define LL_BYPASS_SOFT_VALUE_NESN_BYPASS_POS                       0

#define LL_BYPASS_SOFT_VALUE_MD_BYPASS_RST                         0x0
#define LL_BYPASS_SOFT_VALUE_SN_BYPASS_RST                         0x0
#define LL_BYPASS_SOFT_VALUE_NESN_BYPASS_RST                       0x0

__INLINE void ll_bypass_soft_value_pack(uint8_t mdbypass, uint8_t snbypass, uint8_t nesnbypass)
{
    _PICO_REG_WR(LL_BYPASS_SOFT_VALUE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)mdbypass << 2) | ((uint32_t)snbypass << 1) | ((uint32_t)nesnbypass << 0));
}

__INLINE void ll_bypass_soft_value_unpack(uint8_t* mdbypass, uint8_t* snbypass, uint8_t* nesnbypass)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR);

    *mdbypass = (localVal & ((uint32_t)0x00000004)) >> 2;
    *snbypass = (localVal & ((uint32_t)0x00000002)) >> 1;
    *nesnbypass = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_bypass_soft_value_md_bypass_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ll_bypass_soft_value_md_bypass_setf(uint8_t mdbypass)
{
    _PICO_REG_WR(LL_BYPASS_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)mdbypass << 2));
}

__INLINE uint8_t ll_bypass_soft_value_sn_bypass_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ll_bypass_soft_value_sn_bypass_setf(uint8_t snbypass)
{
    _PICO_REG_WR(LL_BYPASS_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)snbypass << 1));
}

__INLINE uint8_t ll_bypass_soft_value_nesn_bypass_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ll_bypass_soft_value_nesn_bypass_setf(uint8_t nesnbypass)
{
    _PICO_REG_WR(LL_BYPASS_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_BYPASS_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)nesnbypass << 0));
}

 /**
 * @brief INI_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02               md_ini   0
 *     01               sn_ini   0
 *     00             Nesn_ini   0
 * </pre>
 */
#define LL_INI_VALUE_OFFSET 0x0000003C


__INLINE uint32_t ll_ini_value_get(void)
{
    return _PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_ini_value_set(uint32_t value)
{
    _PICO_REG_WR(LL_INI_VALUE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_INI_VALUE_MD_INI_BIT                            ((uint32_t)0x00000004)
#define LL_INI_VALUE_MD_INI_POS                            2
#define LL_INI_VALUE_SN_INI_BIT                            ((uint32_t)0x00000002)
#define LL_INI_VALUE_SN_INI_POS                            1
#define LL_INI_VALUE_NESN_INI_BIT                          ((uint32_t)0x00000001)
#define LL_INI_VALUE_NESN_INI_POS                          0

#define LL_INI_VALUE_MD_INI_RST                            0x0
#define LL_INI_VALUE_SN_INI_RST                            0x0
#define LL_INI_VALUE_NESN_INI_RST                          0x0

__INLINE void ll_ini_value_pack(uint8_t mdini, uint8_t snini, uint8_t nesnini)
{
    _PICO_REG_WR(LL_INI_VALUE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)mdini << 2) | ((uint32_t)snini << 1) | ((uint32_t)nesnini << 0));
}

__INLINE void ll_ini_value_unpack(uint8_t* mdini, uint8_t* snini, uint8_t* nesnini)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR);

    *mdini = (localVal & ((uint32_t)0x00000004)) >> 2;
    *snini = (localVal & ((uint32_t)0x00000002)) >> 1;
    *nesnini = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_ini_value_md_ini_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ll_ini_value_md_ini_setf(uint8_t mdini)
{
    _PICO_REG_WR(LL_INI_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)mdini << 2));
}

__INLINE uint8_t ll_ini_value_sn_ini_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ll_ini_value_sn_ini_setf(uint8_t snini)
{
    _PICO_REG_WR(LL_INI_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)snini << 1));
}

__INLINE uint8_t ll_ini_value_nesn_ini_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ll_ini_value_nesn_ini_setf(uint8_t nesnini)
{
    _PICO_REG_WR(LL_INI_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_INI_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)nesnini << 0));
}

 /**
 * @brief SOFT_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     10          md_soft_sel   0
 *     09          sn_soft_sel   0
 *     08        nesn_soft_sel   0
 *     02              md_soft   0
 *     01              sn_soft   0
 *     00            nesn_soft   0
 * </pre>
 */
#define LL_SOFT_VALUE_OFFSET 0x00000040


__INLINE uint32_t ll_soft_value_get(void)
{
    return _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_soft_value_set(uint32_t value)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_SOFT_VALUE_MD_SOFT_SEL_BIT                       ((uint32_t)0x00000400)
#define LL_SOFT_VALUE_MD_SOFT_SEL_POS                       10
#define LL_SOFT_VALUE_SN_SOFT_SEL_BIT                       ((uint32_t)0x00000200)
#define LL_SOFT_VALUE_SN_SOFT_SEL_POS                       9
#define LL_SOFT_VALUE_NESN_SOFT_SEL_BIT                     ((uint32_t)0x00000100)
#define LL_SOFT_VALUE_NESN_SOFT_SEL_POS                     8
#define LL_SOFT_VALUE_MD_SOFT_BIT                           ((uint32_t)0x00000004)
#define LL_SOFT_VALUE_MD_SOFT_POS                           2
#define LL_SOFT_VALUE_SN_SOFT_BIT                           ((uint32_t)0x00000002)
#define LL_SOFT_VALUE_SN_SOFT_POS                           1
#define LL_SOFT_VALUE_NESN_SOFT_BIT                         ((uint32_t)0x00000001)
#define LL_SOFT_VALUE_NESN_SOFT_POS                         0

#define LL_SOFT_VALUE_MD_SOFT_SEL_RST                       0x0
#define LL_SOFT_VALUE_SN_SOFT_SEL_RST                       0x0
#define LL_SOFT_VALUE_NESN_SOFT_SEL_RST                     0x0
#define LL_SOFT_VALUE_MD_SOFT_RST                           0x0
#define LL_SOFT_VALUE_SN_SOFT_RST                           0x0
#define LL_SOFT_VALUE_NESN_SOFT_RST                         0x0

__INLINE void ll_soft_value_pack(uint8_t mdsoftsel, uint8_t snsoftsel, uint8_t nesnsoftsel, uint8_t mdsoft, uint8_t snsoft, uint8_t nesnsoft)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)mdsoftsel << 10) | ((uint32_t)snsoftsel << 9) | ((uint32_t)nesnsoftsel << 8) | ((uint32_t)mdsoft << 2) | ((uint32_t)snsoft << 1) | ((uint32_t)nesnsoft << 0));
}

__INLINE void ll_soft_value_unpack(uint8_t* mdsoftsel, uint8_t* snsoftsel, uint8_t* nesnsoftsel, uint8_t* mdsoft, uint8_t* snsoft, uint8_t* nesnsoft)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);

    *mdsoftsel = (localVal & ((uint32_t)0x00000400)) >> 10;
    *snsoftsel = (localVal & ((uint32_t)0x00000200)) >> 9;
    *nesnsoftsel = (localVal & ((uint32_t)0x00000100)) >> 8;
    *mdsoft = (localVal & ((uint32_t)0x00000004)) >> 2;
    *snsoft = (localVal & ((uint32_t)0x00000002)) >> 1;
    *nesnsoft = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_soft_value_md_soft_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void ll_soft_value_md_soft_sel_setf(uint8_t mdsoftsel)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)mdsoftsel << 10));
}

__INLINE uint8_t ll_soft_value_sn_soft_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void ll_soft_value_sn_soft_sel_setf(uint8_t snsoftsel)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)snsoftsel << 9));
}

__INLINE uint8_t ll_soft_value_nesn_soft_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ll_soft_value_nesn_soft_sel_setf(uint8_t nesnsoftsel)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)nesnsoftsel << 8));
}

__INLINE uint8_t ll_soft_value_md_soft_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void ll_soft_value_md_soft_setf(uint8_t mdsoft)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)mdsoft << 2));
}

__INLINE uint8_t ll_soft_value_sn_soft_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void ll_soft_value_sn_soft_setf(uint8_t snsoft)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)snsoft << 1));
}

__INLINE uint8_t ll_soft_value_nesn_soft_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ll_soft_value_nesn_soft_setf(uint8_t nesnsoft)
{
    _PICO_REG_WR(LL_SOFT_VALUE_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_SOFT_VALUE_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)nesnsoft << 0));
}

 /**
 * @brief LOCAL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     10                md_rx   0
 *     09                sn_rx   0
 *     08              nesn_rx   0
 *     02             md_local   0
 *     01             sn_local   0
 *     00           Nesn_local   0
 * </pre>
 */
#define LL_LOCAL_OFFSET 0x00000044


__INLINE uint32_t ll_local_get(void)
{
    return _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_LOCAL_MD_RX_BIT                             ((uint32_t)0x00000400)
#define LL_LOCAL_MD_RX_POS                             10
#define LL_LOCAL_SN_RX_BIT                             ((uint32_t)0x00000200)
#define LL_LOCAL_SN_RX_POS                             9
#define LL_LOCAL_NESN_RX_BIT                           ((uint32_t)0x00000100)
#define LL_LOCAL_NESN_RX_POS                           8
#define LL_LOCAL_MD_LOCAL_BIT                          ((uint32_t)0x00000004)
#define LL_LOCAL_MD_LOCAL_POS                          2
#define LL_LOCAL_SN_LOCAL_BIT                          ((uint32_t)0x00000002)
#define LL_LOCAL_SN_LOCAL_POS                          1
#define LL_LOCAL_NESN_LOCAL_BIT                        ((uint32_t)0x00000001)
#define LL_LOCAL_NESN_LOCAL_POS                        0

#define LL_LOCAL_MD_RX_RST                             0x0
#define LL_LOCAL_SN_RX_RST                             0x0
#define LL_LOCAL_NESN_RX_RST                           0x0
#define LL_LOCAL_MD_LOCAL_RST                          0x0
#define LL_LOCAL_SN_LOCAL_RST                          0x0
#define LL_LOCAL_NESN_LOCAL_RST                        0x0

__INLINE void ll_local_unpack(uint8_t* mdrx, uint8_t* snrx, uint8_t* nesnrx, uint8_t* mdlocal, uint8_t* snlocal, uint8_t* nesnlocal)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);

    *mdrx = (localVal & ((uint32_t)0x00000400)) >> 10;
    *snrx = (localVal & ((uint32_t)0x00000200)) >> 9;
    *nesnrx = (localVal & ((uint32_t)0x00000100)) >> 8;
    *mdlocal = (localVal & ((uint32_t)0x00000004)) >> 2;
    *snlocal = (localVal & ((uint32_t)0x00000002)) >> 1;
    *nesnlocal = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_local_md_rx_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t ll_local_sn_rx_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t ll_local_nesn_rx_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t ll_local_md_local_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t ll_local_sn_local_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t ll_local_nesn_local_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOCAL_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief RX_OR_TX_FIFO_ADDR_LAST register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16   Rx_fifo_wr_addr_last   0b0
 *  10:00   Tx_fifo_rd_addr_last   0b0
 * </pre>
 */
#define LL_RX_OR_TX_FIFO_ADDR_LAST_OFFSET 0x0000004C


__INLINE uint32_t ll_rx_or_tx_fifo_addr_last_get(void)
{
    return _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_LAST_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_RX_OR_TX_FIFO_ADDR_LAST_RX_FIFO_WR_ADDR_LAST_MASK             ((uint32_t)0x07FF0000)
#define LL_RX_OR_TX_FIFO_ADDR_LAST_RX_FIFO_WR_ADDR_LAST_LSB              16
#define LL_RX_OR_TX_FIFO_ADDR_LAST_RX_FIFO_WR_ADDR_LAST_WIDTH            ((uint32_t)0x0000000B)
#define LL_RX_OR_TX_FIFO_ADDR_LAST_TX_FIFO_RD_ADDR_LAST_MASK             ((uint32_t)0x000007FF)
#define LL_RX_OR_TX_FIFO_ADDR_LAST_TX_FIFO_RD_ADDR_LAST_LSB              0
#define LL_RX_OR_TX_FIFO_ADDR_LAST_TX_FIFO_RD_ADDR_LAST_WIDTH            ((uint32_t)0x0000000B)

#define LL_RX_OR_TX_FIFO_ADDR_LAST_RX_FIFO_WR_ADDR_LAST_RST              0x0
#define LL_RX_OR_TX_FIFO_ADDR_LAST_TX_FIFO_RD_ADDR_LAST_RST              0x0

__INLINE void ll_rx_or_tx_fifo_addr_last_unpack(uint8_t* rxfifowraddrlast, uint8_t* txfifordaddrlast)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_LAST_OFFSET + LL_BASE_ADDR);

    *rxfifowraddrlast = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *txfifordaddrlast = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint16_t ll_rx_or_tx_fifo_addr_last_rx_fifo_wr_addr_last_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_LAST_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE uint16_t ll_rx_or_tx_fifo_addr_last_tx_fifo_rd_addr_last_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_LAST_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

 /**
 * @brief RX_OR_TX_FIFO_ADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16      Tx_fifo_wr_addr   0b0
 *  10:00      Tx_fifo_rd_addr   0b0
 * </pre>
 */
#define LL_RX_OR_TX_FIFO_ADDR_OFFSET 0x00000050


__INLINE uint32_t ll_rx_or_tx_fifo_addr_get(void)
{
    return _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_WR_ADDR_MASK                  ((uint32_t)0x07FF0000)
#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_WR_ADDR_LSB                   16
#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_WR_ADDR_WIDTH                 ((uint32_t)0x0000000B)
#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_RD_ADDR_MASK                  ((uint32_t)0x000007FF)
#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_RD_ADDR_LSB                   0
#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_RD_ADDR_WIDTH                 ((uint32_t)0x0000000B)

#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_WR_ADDR_RST                   0x0
#define LL_RX_OR_TX_FIFO_ADDR_TX_FIFO_RD_ADDR_RST                   0x0

__INLINE void ll_rx_or_tx_fifo_addr_unpack(uint8_t* txfifowraddr, uint8_t* txfifordaddr)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_OFFSET + LL_BASE_ADDR);

    *txfifowraddr = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *txfifordaddr = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint16_t ll_rx_or_tx_fifo_addr_tx_fifo_wr_addr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE uint16_t ll_rx_or_tx_fifo_addr_tx_fifo_rd_addr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

 /**
 * @brief RX_OR_TX_FIFO_ADDR1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16      rx_fifo_wr_addr   0b0
 *  10:00      rx_fifo_rd_addr   0b0
 * </pre>
 */
#define LL_RX_OR_TX_FIFO_ADDR1_OFFSET 0x00000054


__INLINE uint32_t ll_rx_or_tx_fifo_addr1_get(void)
{
    return _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR1_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_WR_ADDR_MASK                  ((uint32_t)0x07FF0000)
#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_WR_ADDR_LSB                   16
#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_WR_ADDR_WIDTH                 ((uint32_t)0x0000000B)
#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_RD_ADDR_MASK                  ((uint32_t)0x000007FF)
#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_RD_ADDR_LSB                   0
#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_RD_ADDR_WIDTH                 ((uint32_t)0x0000000B)

#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_WR_ADDR_RST                   0x0
#define LL_RX_OR_TX_FIFO_ADDR1_RX_FIFO_RD_ADDR_RST                   0x0

__INLINE void ll_rx_or_tx_fifo_addr1_unpack(uint8_t* rxfifowraddr, uint8_t* rxfifordaddr)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR1_OFFSET + LL_BASE_ADDR);

    *rxfifowraddr = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *rxfifordaddr = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint16_t ll_rx_or_tx_fifo_addr1_rx_fifo_wr_addr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE uint16_t ll_rx_or_tx_fifo_addr1_rx_fifo_rd_addr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_ADDR1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

 /**
 * @brief RX_OR_TX_FIFO_RESET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     08        RX_fifo_reset   0
 *     00        TX_fifo_reset   0
 * </pre>
 */
#define LL_RX_OR_TX_FIFO_RESET_OFFSET 0x00000058


__INLINE uint32_t ll_rx_or_tx_fifo_reset_get(void)
{
    return _PICO_REG_RD(LL_RX_OR_TX_FIFO_RESET_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_rx_or_tx_fifo_reset_set(uint32_t value)
{
    _PICO_REG_WR(LL_RX_OR_TX_FIFO_RESET_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_RX_OR_TX_FIFO_RESET_RX_FIFO_RESET_BIT                     ((uint32_t)0x00000100)
#define LL_RX_OR_TX_FIFO_RESET_RX_FIFO_RESET_POS                     8
#define LL_RX_OR_TX_FIFO_RESET_TX_FIFO_RESET_BIT                     ((uint32_t)0x00000001)
#define LL_RX_OR_TX_FIFO_RESET_TX_FIFO_RESET_POS                     0

#define LL_RX_OR_TX_FIFO_RESET_RX_FIFO_RESET_RST                     0x0
#define LL_RX_OR_TX_FIFO_RESET_TX_FIFO_RESET_RST                     0x0

__INLINE void ll_rx_or_tx_fifo_reset_pack(uint8_t rxfiforeset, uint8_t txfiforeset)
{
    _PICO_REG_WR(LL_RX_OR_TX_FIFO_RESET_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rxfiforeset << 8) | ((uint32_t)txfiforeset << 0));
}

__INLINE void ll_rx_or_tx_fifo_reset_unpack(uint8_t* rxfiforeset, uint8_t* txfiforeset)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_RESET_OFFSET + LL_BASE_ADDR);

    *rxfiforeset = (localVal & ((uint32_t)0x00000100)) >> 8;
    *txfiforeset = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t ll_rx_or_tx_fifo_reset_rx_fifo_reset_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_RESET_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void ll_rx_or_tx_fifo_reset_rx_fifo_reset_setf(uint8_t rxfiforeset)
{
    _PICO_REG_WR(LL_RX_OR_TX_FIFO_RESET_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_RX_OR_TX_FIFO_RESET_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rxfiforeset << 8));
}

__INLINE uint8_t ll_rx_or_tx_fifo_reset_tx_fifo_reset_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_OR_TX_FIFO_RESET_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void ll_rx_or_tx_fifo_reset_tx_fifo_reset_setf(uint8_t txfiforeset)
{
    _PICO_REG_WR(LL_RX_OR_TX_FIFO_RESET_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_RX_OR_TX_FIFO_RESET_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)txfiforeset << 0));
}

 /**
 * @brief RD_CNT_INI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:16      rd_cnt_last_ini   0b0
 *  10:00           rd_cnt_ini   0b0
 * </pre>
 */
#define LL_RD_CNT_INI_OFFSET 0x0000005C


__INLINE uint32_t ll_rd_cnt_ini_get(void)
{
    return _PICO_REG_RD(LL_RD_CNT_INI_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_rd_cnt_ini_set(uint32_t value)
{
    _PICO_REG_WR(LL_RD_CNT_INI_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_RD_CNT_INI_RD_CNT_LAST_INI_MASK                  ((uint32_t)0x07FF0000)
#define LL_RD_CNT_INI_RD_CNT_LAST_INI_LSB                   16
#define LL_RD_CNT_INI_RD_CNT_LAST_INI_WIDTH                 ((uint32_t)0x0000000B)
#define LL_RD_CNT_INI_RD_CNT_INI_MASK                       ((uint32_t)0x000007FF)
#define LL_RD_CNT_INI_RD_CNT_INI_LSB                        0
#define LL_RD_CNT_INI_RD_CNT_INI_WIDTH                      ((uint32_t)0x0000000B)

#define LL_RD_CNT_INI_RD_CNT_LAST_INI_RST                   0x0
#define LL_RD_CNT_INI_RD_CNT_INI_RST                        0x0

__INLINE void ll_rd_cnt_ini_pack(uint16_t rdcntlastini, uint16_t rdcntini)
{
    _PICO_REG_WR(LL_RD_CNT_INI_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rdcntlastini << 16) | ((uint32_t)rdcntini << 0));
}

__INLINE void ll_rd_cnt_ini_unpack(uint8_t* rdcntlastini, uint8_t* rdcntini)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RD_CNT_INI_OFFSET + LL_BASE_ADDR);

    *rdcntlastini = (localVal & ((uint32_t)0x07FF0000)) >> 16;
    *rdcntini = (localVal & ((uint32_t)0x000007FF)) >> 0;
}

__INLINE uint16_t ll_rd_cnt_ini_rd_cnt_last_ini_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RD_CNT_INI_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07FF0000)) >> 16);
}

__INLINE void ll_rd_cnt_ini_rd_cnt_last_ini_setf(uint16_t rdcntlastini)
{
    _PICO_REG_WR(LL_RD_CNT_INI_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_RD_CNT_INI_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x07FF0000)) | ((uint32_t)rdcntlastini << 16));
}

__INLINE uint16_t ll_rd_cnt_ini_rd_cnt_ini_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RD_CNT_INI_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000007FF)) >> 0);
}

__INLINE void ll_rd_cnt_ini_rd_cnt_ini_setf(uint16_t rdcntini)
{
    _PICO_REG_WR(LL_RD_CNT_INI_OFFSET+ LL_BASE_ADDR, (_PICO_REG_RD(LL_RD_CNT_INI_OFFSET + LL_BASE_ADDR) & ~((uint32_t)0x000007FF)) | ((uint32_t)rdcntini << 0));
}

 /**
 * @brief LOOP_TIMEOUT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         loop_timeout   0b0
 * </pre>
 */
#define LL_LOOP_TIMEOUT_OFFSET 0x00000060


__INLINE uint32_t ll_loop_timeout_get(void)
{
    return _PICO_REG_RD(LL_LOOP_TIMEOUT_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_loop_timeout_set(uint32_t value)
{
    _PICO_REG_WR(LL_LOOP_TIMEOUT_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_LOOP_TIMEOUT_LOOP_TIMEOUT_MASK                     ((uint32_t)0xFFFFFFFF)
#define LL_LOOP_TIMEOUT_LOOP_TIMEOUT_LSB                      0
#define LL_LOOP_TIMEOUT_LOOP_TIMEOUT_WIDTH                    ((uint32_t)0x00000020)

#define LL_LOOP_TIMEOUT_LOOP_TIMEOUT_RST                      0x0

__INLINE void ll_loop_timeout_pack(uint32_t looptimeout)
{
    _PICO_REG_WR(LL_LOOP_TIMEOUT_OFFSET+ LL_BASE_ADDR,  ((uint32_t)looptimeout << 0));
}

__INLINE void ll_loop_timeout_unpack(uint8_t* looptimeout)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOOP_TIMEOUT_OFFSET + LL_BASE_ADDR);

    *looptimeout = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_loop_timeout_loop_timeout_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_LOOP_TIMEOUT_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void ll_loop_timeout_loop_timeout_setf(uint32_t looptimeout)
{
    _PICO_REG_WR(LL_LOOP_TIMEOUT_OFFSET+ LL_BASE_ADDR, (uint32_t)looptimeout << 0);
}

 /**
 * @brief NACK_NUM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00             Nack_num   0b0
 * </pre>
 */
#define LL_NACK_NUM_OFFSET 0x00000064


__INLINE uint32_t ll_nack_num_get(void)
{
    return _PICO_REG_RD(LL_NACK_NUM_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_nack_num_set(uint32_t value)
{
    _PICO_REG_WR(LL_NACK_NUM_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_NACK_NUM_NACK_NUM_MASK                         ((uint32_t)0x0000000F)
#define LL_NACK_NUM_NACK_NUM_LSB                          0
#define LL_NACK_NUM_NACK_NUM_WIDTH                        ((uint32_t)0x00000004)

#define LL_NACK_NUM_NACK_NUM_RST                          0x0

__INLINE void ll_nack_num_pack(uint8_t nacknum)
{
    _PICO_REG_WR(LL_NACK_NUM_OFFSET+ LL_BASE_ADDR,  ((uint32_t)nacknum << 0));
}

__INLINE void ll_nack_num_unpack(uint8_t* nacknum)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_NACK_NUM_OFFSET + LL_BASE_ADDR);

    *nacknum = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t ll_nack_num_nack_num_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_NACK_NUM_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void ll_nack_num_nack_num_setf(uint8_t nacknum)
{
    _PICO_REG_WR(LL_NACK_NUM_OFFSET+ LL_BASE_ADDR, (uint32_t)nacknum << 0);
}

 /**
 * @brief NACK_NUM1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00             Nack_num   0b0
 * </pre>
 */
#define LL_NACK_NUM1_OFFSET 0x00000068


__INLINE uint32_t ll_nack_num1_get(void)
{
    return _PICO_REG_RD(LL_NACK_NUM1_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_nack_num1_set(uint32_t value)
{
    _PICO_REG_WR(LL_NACK_NUM1_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_NACK_NUM1_NACK_NUM_MASK                         ((uint32_t)0x000000FF)
#define LL_NACK_NUM1_NACK_NUM_LSB                          0
#define LL_NACK_NUM1_NACK_NUM_WIDTH                        ((uint32_t)0x00000008)

#define LL_NACK_NUM1_NACK_NUM_RST                          0x0

__INLINE void ll_nack_num1_pack(uint8_t nacknum)
{
    _PICO_REG_WR(LL_NACK_NUM1_OFFSET+ LL_BASE_ADDR,  ((uint32_t)nacknum << 0));
}

__INLINE void ll_nack_num1_unpack(uint8_t* nacknum)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_NACK_NUM1_OFFSET + LL_BASE_ADDR);

    *nacknum = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t ll_nack_num1_nack_num_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_NACK_NUM1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void ll_nack_num1_nack_num_setf(uint8_t nacknum)
{
    _PICO_REG_WR(LL_NACK_NUM1_OFFSET+ LL_BASE_ADDR, (uint32_t)nacknum << 0);
}

 /**
 * @brief ANCHOR_POINT_CNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00     Anchor_point_cnt   0b0
 * </pre>
 */
#define LL_ANCHOR_POINT_CNT_OFFSET 0x0000006C


__INLINE uint32_t ll_anchor_point_cnt_get(void)
{
    return _PICO_REG_RD(LL_ANCHOR_POINT_CNT_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_ANCHOR_POINT_CNT_ANCHOR_POINT_CNT_MASK                 ((uint32_t)0x0000FFFF)
#define LL_ANCHOR_POINT_CNT_ANCHOR_POINT_CNT_LSB                  0
#define LL_ANCHOR_POINT_CNT_ANCHOR_POINT_CNT_WIDTH                ((uint32_t)0x00000010)

#define LL_ANCHOR_POINT_CNT_ANCHOR_POINT_CNT_RST                  0x0

__INLINE void ll_anchor_point_cnt_unpack(uint8_t* anchorpointcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_ANCHOR_POINT_CNT_OFFSET + LL_BASE_ADDR);

    *anchorpointcnt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t ll_anchor_point_cnt_anchor_point_cnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_ANCHOR_POINT_CNT_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

 /**
 * @brief RX_TEST_MODE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00   RX_test_mode_total_time_window   0b0
 * </pre>
 */
#define LL_RX_TEST_MODE_OFFSET 0x00000080


__INLINE uint32_t ll_rx_test_mode_get(void)
{
    return _PICO_REG_RD(LL_RX_TEST_MODE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_rx_test_mode_set(uint32_t value)
{
    _PICO_REG_WR(LL_RX_TEST_MODE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_RX_TEST_MODE_RX_TEST_MODE_TOTAL_TIME_WINDOW_MASK   ((uint32_t)0xFFFFFFFF)
#define LL_RX_TEST_MODE_RX_TEST_MODE_TOTAL_TIME_WINDOW_LSB    0
#define LL_RX_TEST_MODE_RX_TEST_MODE_TOTAL_TIME_WINDOW_WIDTH  ((uint32_t)0x00000020)

#define LL_RX_TEST_MODE_RX_TEST_MODE_TOTAL_TIME_WINDOW_RST    0x0

__INLINE void ll_rx_test_mode_pack(uint32_t rxtestmodetotaltimewindow)
{
    _PICO_REG_WR(LL_RX_TEST_MODE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rxtestmodetotaltimewindow << 0));
}

__INLINE void ll_rx_test_mode_unpack(uint8_t* rxtestmodetotaltimewindow)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_TEST_MODE_OFFSET + LL_BASE_ADDR);

    *rxtestmodetotaltimewindow = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_rx_test_mode_rx_test_mode_total_time_window_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_TEST_MODE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void ll_rx_test_mode_rx_test_mode_total_time_window_setf(uint32_t rxtestmodetotaltimewindow)
{
    _PICO_REG_WR(LL_RX_TEST_MODE_OFFSET+ LL_BASE_ADDR, (uint32_t)rxtestmodetotaltimewindow << 0);
}

 /**
 * @brief RX_PKT_NUMBER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00        rx_pkt_number   0b0
 * </pre>
 */
#define LL_RX_PKT_NUMBER_OFFSET 0x00000084


__INLINE uint32_t ll_rx_pkt_number_get(void)
{
    return _PICO_REG_RD(LL_RX_PKT_NUMBER_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_RX_PKT_NUMBER_RX_PKT_NUMBER_MASK                    ((uint32_t)0xFFFFFFFF)
#define LL_RX_PKT_NUMBER_RX_PKT_NUMBER_LSB                     0
#define LL_RX_PKT_NUMBER_RX_PKT_NUMBER_WIDTH                   ((uint32_t)0x00000020)

#define LL_RX_PKT_NUMBER_RX_PKT_NUMBER_RST                     0x0

__INLINE void ll_rx_pkt_number_unpack(uint8_t* rxpktnumber)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PKT_NUMBER_OFFSET + LL_BASE_ADDR);

    *rxpktnumber = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_rx_pkt_number_rx_pkt_number_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_PKT_NUMBER_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief CRC_ERROR_NUMBER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00     Crc_error_number   0b0
 * </pre>
 */
#define LL_CRC_ERROR_NUMBER_OFFSET 0x00000088


__INLINE uint32_t ll_crc_error_number_get(void)
{
    return _PICO_REG_RD(LL_CRC_ERROR_NUMBER_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_CRC_ERROR_NUMBER_CRC_ERROR_NUMBER_MASK                 ((uint32_t)0xFFFFFFFF)
#define LL_CRC_ERROR_NUMBER_CRC_ERROR_NUMBER_LSB                  0
#define LL_CRC_ERROR_NUMBER_CRC_ERROR_NUMBER_WIDTH                ((uint32_t)0x00000020)

#define LL_CRC_ERROR_NUMBER_CRC_ERROR_NUMBER_RST                  0x0

__INLINE void ll_crc_error_number_unpack(uint8_t* crcerrornumber)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_CRC_ERROR_NUMBER_OFFSET + LL_BASE_ADDR);

    *crcerrornumber = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_crc_error_number_crc_error_number_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_CRC_ERROR_NUMBER_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief SFD_ERROR_SUM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00        sfd_error_sum   0b0
 * </pre>
 */
#define LL_SFD_ERROR_SUM_OFFSET 0x0000008C


__INLINE uint32_t ll_sfd_error_sum_get(void)
{
    return _PICO_REG_RD(LL_SFD_ERROR_SUM_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_SFD_ERROR_SUM_SFD_ERROR_SUM_MASK                    ((uint32_t)0xFFFFFFFF)
#define LL_SFD_ERROR_SUM_SFD_ERROR_SUM_LSB                     0
#define LL_SFD_ERROR_SUM_SFD_ERROR_SUM_WIDTH                   ((uint32_t)0x00000020)

#define LL_SFD_ERROR_SUM_SFD_ERROR_SUM_RST                     0x0

__INLINE void ll_sfd_error_sum_unpack(uint8_t* sfderrorsum)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SFD_ERROR_SUM_OFFSET + LL_BASE_ADDR);

    *sfderrorsum = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_sfd_error_sum_sfd_error_sum_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SFD_ERROR_SUM_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief SYNC_ASATE_SUM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00       Sync_asate_sum   0b0
 * </pre>
 */
#define LL_SYNC_ASATE_SUM_OFFSET 0x00000090


__INLINE uint32_t ll_sync_asate_sum_get(void)
{
    return _PICO_REG_RD(LL_SYNC_ASATE_SUM_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_SYNC_ASATE_SUM_SYNC_ASATE_SUM_MASK                   ((uint32_t)0xFFFFFFFF)
#define LL_SYNC_ASATE_SUM_SYNC_ASATE_SUM_LSB                    0
#define LL_SYNC_ASATE_SUM_SYNC_ASATE_SUM_WIDTH                  ((uint32_t)0x00000020)

#define LL_SYNC_ASATE_SUM_SYNC_ASATE_SUM_RST                    0x0

__INLINE void ll_sync_asate_sum_unpack(uint8_t* syncasatesum)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SYNC_ASATE_SUM_OFFSET + LL_BASE_ADDR);

    *syncasatesum = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_sync_asate_sum_sync_asate_sum_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_SYNC_ASATE_SUM_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief FREQ_OFFSET_SUM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00      Freq_offset_sum   0b0
 * </pre>
 */
#define LL_FREQ_OFFSET_SUM_OFFSET 0x00000094


__INLINE uint32_t ll_freq_offset_sum_get(void)
{
    return _PICO_REG_RD(LL_FREQ_OFFSET_SUM_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_FREQ_OFFSET_SUM_FREQ_OFFSET_SUM_MASK                  ((uint32_t)0xFFFFFFFF)
#define LL_FREQ_OFFSET_SUM_FREQ_OFFSET_SUM_LSB                   0
#define LL_FREQ_OFFSET_SUM_FREQ_OFFSET_SUM_WIDTH                 ((uint32_t)0x00000020)

#define LL_FREQ_OFFSET_SUM_FREQ_OFFSET_SUM_RST                   0x0

__INLINE void ll_freq_offset_sum_unpack(uint8_t* freqoffsetsum)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_FREQ_OFFSET_SUM_OFFSET + LL_BASE_ADDR);

    *freqoffsetsum = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_freq_offset_sum_freq_offset_sum_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_FREQ_OFFSET_SUM_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief NRSSI_SUM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            Nrssi_sum   0b0
 * </pre>
 */
#define LL_NRSSI_SUM_OFFSET 0x00000098


__INLINE uint32_t ll_nrssi_sum_get(void)
{
    return _PICO_REG_RD(LL_NRSSI_SUM_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_NRSSI_SUM_NRSSI_SUM_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_NRSSI_SUM_NRSSI_SUM_LSB                         0
#define LL_NRSSI_SUM_NRSSI_SUM_WIDTH                       ((uint32_t)0x00000020)

#define LL_NRSSI_SUM_NRSSI_SUM_RST                         0x0

__INLINE void ll_nrssi_sum_unpack(uint8_t* nrssisum)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_NRSSI_SUM_OFFSET + LL_BASE_ADDR);

    *nrssisum = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_nrssi_sum_nrssi_sum_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_NRSSI_SUM_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief ARSSI_UM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00             Arssi_um   0b0
 * </pre>
 */
#define LL_ARSSI_UM_OFFSET 0x0000009C


__INLINE uint32_t ll_arssi_um_get(void)
{
    return _PICO_REG_RD(LL_ARSSI_UM_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_ARSSI_UM_ARSSI_UM_MASK                         ((uint32_t)0xFFFFFFFF)
#define LL_ARSSI_UM_ARSSI_UM_LSB                          0
#define LL_ARSSI_UM_ARSSI_UM_WIDTH                        ((uint32_t)0x00000020)

#define LL_ARSSI_UM_ARSSI_UM_RST                          0x0

__INLINE void ll_arssi_um_unpack(uint8_t* arssium)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_ARSSI_UM_OFFSET + LL_BASE_ADDR);

    *arssium = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_arssi_um_arssi_um_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_ARSSI_UM_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum0   0b0
 * </pre>
 */
#define LL_GAIN_SUM0_OFFSET 0x000000A0


__INLINE uint32_t ll_gain_sum0_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM0_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM0_GAIN_SUM0_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM0_GAIN_SUM0_LSB                         0
#define LL_GAIN_SUM0_GAIN_SUM0_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM0_GAIN_SUM0_RST                         0x0

__INLINE void ll_gain_sum0_unpack(uint8_t* gainsum0)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM0_OFFSET + LL_BASE_ADDR);

    *gainsum0 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum0_gain_sum0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM0_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum1   0b0
 * </pre>
 */
#define LL_GAIN_SUM1_OFFSET 0x000000A4


__INLINE uint32_t ll_gain_sum1_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM1_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM1_GAIN_SUM1_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM1_GAIN_SUM1_LSB                         0
#define LL_GAIN_SUM1_GAIN_SUM1_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM1_GAIN_SUM1_RST                         0x0

__INLINE void ll_gain_sum1_unpack(uint8_t* gainsum1)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM1_OFFSET + LL_BASE_ADDR);

    *gainsum1 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum1_gain_sum1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM1_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum2   0b0
 * </pre>
 */
#define LL_GAIN_SUM2_OFFSET 0x000000A8


__INLINE uint32_t ll_gain_sum2_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM2_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM2_GAIN_SUM2_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM2_GAIN_SUM2_LSB                         0
#define LL_GAIN_SUM2_GAIN_SUM2_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM2_GAIN_SUM2_RST                         0x0

__INLINE void ll_gain_sum2_unpack(uint8_t* gainsum2)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM2_OFFSET + LL_BASE_ADDR);

    *gainsum2 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum2_gain_sum2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM2_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum3   0b0
 * </pre>
 */
#define LL_GAIN_SUM3_OFFSET 0x000000AC


__INLINE uint32_t ll_gain_sum3_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM3_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM3_GAIN_SUM3_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM3_GAIN_SUM3_LSB                         0
#define LL_GAIN_SUM3_GAIN_SUM3_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM3_GAIN_SUM3_RST                         0x0

__INLINE void ll_gain_sum3_unpack(uint8_t* gainsum3)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM3_OFFSET + LL_BASE_ADDR);

    *gainsum3 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum3_gain_sum3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM3_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum4   0b0
 * </pre>
 */
#define LL_GAIN_SUM4_OFFSET 0x000000B0


__INLINE uint32_t ll_gain_sum4_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM4_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM4_GAIN_SUM4_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM4_GAIN_SUM4_LSB                         0
#define LL_GAIN_SUM4_GAIN_SUM4_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM4_GAIN_SUM4_RST                         0x0

__INLINE void ll_gain_sum4_unpack(uint8_t* gainsum4)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM4_OFFSET + LL_BASE_ADDR);

    *gainsum4 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum4_gain_sum4_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM4_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM5 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum5   0b0
 * </pre>
 */
#define LL_GAIN_SUM5_OFFSET 0x000000B4


__INLINE uint32_t ll_gain_sum5_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM5_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM5_GAIN_SUM5_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM5_GAIN_SUM5_LSB                         0
#define LL_GAIN_SUM5_GAIN_SUM5_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM5_GAIN_SUM5_RST                         0x0

__INLINE void ll_gain_sum5_unpack(uint8_t* gainsum5)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM5_OFFSET + LL_BASE_ADDR);

    *gainsum5 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum5_gain_sum5_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM5_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM6 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum6   0b0
 * </pre>
 */
#define LL_GAIN_SUM6_OFFSET 0x000000B8


__INLINE uint32_t ll_gain_sum6_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM6_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM6_GAIN_SUM6_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM6_GAIN_SUM6_LSB                         0
#define LL_GAIN_SUM6_GAIN_SUM6_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM6_GAIN_SUM6_RST                         0x0

__INLINE void ll_gain_sum6_unpack(uint8_t* gainsum6)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM6_OFFSET + LL_BASE_ADDR);

    *gainsum6 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum6_gain_sum6_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM6_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM7 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum7   0b0
 * </pre>
 */
#define LL_GAIN_SUM7_OFFSET 0x000000BC


__INLINE uint32_t ll_gain_sum7_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM7_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM7_GAIN_SUM7_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM7_GAIN_SUM7_LSB                         0
#define LL_GAIN_SUM7_GAIN_SUM7_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM7_GAIN_SUM7_RST                         0x0

__INLINE void ll_gain_sum7_unpack(uint8_t* gainsum7)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM7_OFFSET + LL_BASE_ADDR);

    *gainsum7 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum7_gain_sum7_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM7_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief GAIN_SUM8 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00            gain_sum8   0b0
 * </pre>
 */
#define LL_GAIN_SUM8_OFFSET 0x000000C0


__INLINE uint32_t ll_gain_sum8_get(void)
{
    return _PICO_REG_RD(LL_GAIN_SUM8_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_GAIN_SUM8_GAIN_SUM8_MASK                        ((uint32_t)0xFFFFFFFF)
#define LL_GAIN_SUM8_GAIN_SUM8_LSB                         0
#define LL_GAIN_SUM8_GAIN_SUM8_WIDTH                       ((uint32_t)0x00000020)

#define LL_GAIN_SUM8_GAIN_SUM8_RST                         0x0

__INLINE void ll_gain_sum8_unpack(uint8_t* gainsum8)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM8_OFFSET + LL_BASE_ADDR);

    *gainsum8 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_gain_sum8_gain_sum8_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_GAIN_SUM8_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief AOD_DATA_READ register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00        AOD_data_read   0b0
 * </pre>
 */
#define LL_AOD_DATA_READ_OFFSET 0x00000200


__INLINE uint32_t ll_aod_data_read_get(void)
{
    return _PICO_REG_RD(LL_AOD_DATA_READ_OFFSET + LL_BASE_ADDR);
}

// field definitions
#define LL_AOD_DATA_READ_AOD_DATA_READ_MASK                    ((uint32_t)0xFFFFFFFF)
#define LL_AOD_DATA_READ_AOD_DATA_READ_LSB                     0
#define LL_AOD_DATA_READ_AOD_DATA_READ_WIDTH                   ((uint32_t)0x00000020)

#define LL_AOD_DATA_READ_AOD_DATA_READ_RST                     0x0

__INLINE void ll_aod_data_read_unpack(uint8_t* aoddataread)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_AOD_DATA_READ_OFFSET + LL_BASE_ADDR);

    *aoddataread = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_aod_data_read_aod_data_read_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_AOD_DATA_READ_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief TX_FIFO_WRITE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00        TX_fifo_write   0b0
 * </pre>
 */
#define LL_TX_FIFO_WRITE_OFFSET 0x00000400


__INLINE uint32_t ll_tx_fifo_write_get(void)
{
    return _PICO_REG_RD(LL_TX_FIFO_WRITE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_tx_fifo_write_set(uint32_t value)
{
    _PICO_REG_WR(LL_TX_FIFO_WRITE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_TX_FIFO_WRITE_TX_FIFO_WRITE_MASK                    ((uint32_t)0xFFFFFFFF)
#define LL_TX_FIFO_WRITE_TX_FIFO_WRITE_LSB                     0
#define LL_TX_FIFO_WRITE_TX_FIFO_WRITE_WIDTH                   ((uint32_t)0x00000020)

#define LL_TX_FIFO_WRITE_TX_FIFO_WRITE_RST                     0x0

__INLINE void ll_tx_fifo_write_pack(uint32_t txfifowrite)
{
    _PICO_REG_WR(LL_TX_FIFO_WRITE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)txfifowrite << 0));
}

__INLINE void ll_tx_fifo_write_unpack(uint8_t* txfifowrite)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_FIFO_WRITE_OFFSET + LL_BASE_ADDR);

    *txfifowrite = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_tx_fifo_write_tx_fifo_write_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_TX_FIFO_WRITE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void ll_tx_fifo_write_tx_fifo_write_setf(uint32_t txfifowrite)
{
    _PICO_REG_WR(LL_TX_FIFO_WRITE_OFFSET+ LL_BASE_ADDR, (uint32_t)txfifowrite << 0);
}

 /**
 * @brief RX_FIFO_WRITE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00        RX_fifo_write   0b0
 * </pre>
 */
#define LL_RX_FIFO_WRITE_OFFSET 0x00000C00


__INLINE uint32_t ll_rx_fifo_write_get(void)
{
    return _PICO_REG_RD(LL_RX_FIFO_WRITE_OFFSET + LL_BASE_ADDR);
}

__INLINE void ll_rx_fifo_write_set(uint32_t value)
{
    _PICO_REG_WR(LL_RX_FIFO_WRITE_OFFSET+ LL_BASE_ADDR, value);
}

// field definitions
#define LL_RX_FIFO_WRITE_RX_FIFO_WRITE_MASK                    ((uint32_t)0xFFFFFFFF)
#define LL_RX_FIFO_WRITE_RX_FIFO_WRITE_LSB                     0
#define LL_RX_FIFO_WRITE_RX_FIFO_WRITE_WIDTH                   ((uint32_t)0x00000020)

#define LL_RX_FIFO_WRITE_RX_FIFO_WRITE_RST                     0x0

__INLINE void ll_rx_fifo_write_pack(uint32_t rxfifowrite)
{
    _PICO_REG_WR(LL_RX_FIFO_WRITE_OFFSET+ LL_BASE_ADDR,  ((uint32_t)rxfifowrite << 0));
}

__INLINE void ll_rx_fifo_write_unpack(uint8_t* rxfifowrite)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_FIFO_WRITE_OFFSET + LL_BASE_ADDR);

    *rxfifowrite = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t ll_rx_fifo_write_rx_fifo_write_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(LL_RX_FIFO_WRITE_OFFSET + LL_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void ll_rx_fifo_write_rx_fifo_write_setf(uint32_t rxfifowrite)
{
    _PICO_REG_WR(LL_RX_FIFO_WRITE_OFFSET+ LL_BASE_ADDR, (uint32_t)rxfifowrite << 0);
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :31;
      __IO uint32_t enable:1;
    }Enable_fld;
    __IO uint32_t Enable;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :8;
      __IO uint32_t rx_packet_number:8;
      __IO uint32_t tx_packet_number:8;
      __IO uint32_t :4;
      __IO uint32_t mode:4;
    }Mode_fld;
    __IO uint32_t Mode;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :28;
      __IO uint32_t state:4;
    }state_fld;
    __IO uint32_t state;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :18;
      __IO uint32_t nack_int:1;
      __IO uint32_t loop_timeout:1;
      __IO uint32_t crc_err_2:1;
      __IO uint32_t crc_ok:1;
      __IO uint32_t rx_done:1;
      __IO uint32_t tx_done:1;
      __IO uint32_t :3;
      __IO uint32_t rx_fifo_over_half:1;
      __IO uint32_t rx_fifo_full:1;
      __IO uint32_t rx_timeout:1;
      __IO uint32_t rx_crc_error:1;
      __IO uint32_t mode_done:1;
    }Mode1_fld;
    __IO uint32_t Mode1;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :18;
      __IO uint32_t nack_int:1;
      __IO uint32_t loop_timeout:1;
      __IO uint32_t crc_err_2:1;
      __IO uint32_t crc_ok:1;
      __IO uint32_t rx_done:1;
      __IO uint32_t tx_done:1;
      __IO uint32_t :3;
      __IO uint32_t rx_fifo_over_half:1;
      __IO uint32_t rx_fifo_full:1;
      __IO uint32_t rx_timeout:1;
      __IO uint32_t rx_crc_error:1;
      __IO uint32_t mode_done:1;
    }Mode2_fld;
    __IO uint32_t Mode2;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :18;
      __IO uint32_t nack_int:1;
      __IO uint32_t loop_timeout:1;
      __IO uint32_t crc_err_2:1;
      __IO uint32_t crc_ok:1;
      __IO uint32_t rx_done:1;
      __IO uint32_t tx_done:1;
      __IO uint32_t :3;
      __IO uint32_t rx_fifo_over_half_:1;
      __IO uint32_t rx_fifo_full:1;
      __IO uint32_t rx_timeout:1;
      __IO uint32_t rx_crc_error:1;
      __IO uint32_t mode_done:1;
    }Mode3_fld;
    __IO uint32_t Mode3;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t tx_to_rx_interval_value:32;
    }TX_to_RX_interval_value_fld;
    __IO uint32_t TX_to_RX_interval_value;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t rx_to_tx_interval_value1:32;
    }TX_to_RX_interval_value1_fld;
    __IO uint32_t TX_to_RX_interval_value1;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t phy_tx_en_release:16;
      __IO uint32_t phy_rx_en_release:16;
    }TX_and_RX_en_release_fld;
    __IO uint32_t TX_and_RX_en_release;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :16;
      __IO uint32_t rx_time_out_1st:16;
    }rx_time_out_1st_fld;
    __IO uint32_t rx_time_out_1st;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t :16;
      __IO uint32_t rx_time_out:16;
    }rx_time_out_fld;
    __IO uint32_t rx_time_out;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t tx_empty_packet_header:16;
      __IO uint32_t rx_pkt_header:16;
    }rx_pkt_header_fld;
    __IO uint32_t rx_pkt_header;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :8;
      __IO uint32_t rx_crc_error_count:8;
      __IO uint32_t rx_total_cnt:8;
      __IO uint32_t rx_packet_number:8;
    }RX_packet_number_fld;
    __IO uint32_t RX_packet_number;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t :20;
      __IO uint32_t tx_nack_count:4;
      __IO uint32_t tx_ack_count:8;
    }TX_ACK_count_fld;
    __IO uint32_t TX_ACK_count;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t :29;
      __IO uint32_t md_bypass:1;
      __IO uint32_t sn_bypass:1;
      __IO uint32_t nesn_bypass:1;
    }bypass_soft_value_fld;
    __IO uint32_t bypass_soft_value;
  };

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t :29;
      __IO uint32_t md_ini:1;
      __IO uint32_t sn_ini:1;
      __IO uint32_t nesn_ini:1;
    }ini_value_fld;
    __IO uint32_t ini_value;
  };

  union{ //offset addr 0x0040
    struct{
      __IO uint32_t :21;
      __IO uint32_t md_soft_sel:1;
      __IO uint32_t sn_soft_sel:1;
      __IO uint32_t nesn_soft_sel:1;
      __IO uint32_t :5;
      __IO uint32_t md_soft:1;
      __IO uint32_t sn_soft:1;
      __IO uint32_t nesn_soft:1;
    }soft_value_fld;
    __IO uint32_t soft_value;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t :21;
      __IO uint32_t md_rx:1;
      __IO uint32_t sn_rx:1;
      __IO uint32_t nesn_rx:1;
      __IO uint32_t :5;
      __IO uint32_t md_local:1;
      __IO uint32_t sn_local:1;
      __IO uint32_t nesn_local:1;
    }Local_fld;
    __IO uint32_t Local;
  };


  union{ //offset addr 0x004c
    struct{
      __IO uint32_t :5;
      __IO uint32_t rx_fifo_wr_addr_last:11;
      __IO uint32_t :5;
      __IO uint32_t tx_fifo_rd_addr_last:11;
    }RX_or_TX_fifo_addr_last_fld;
    __IO uint32_t RX_or_TX_fifo_addr_last;
  };

  union{ //offset addr 0x0050
    struct{
      __IO uint32_t :5;
      __IO uint32_t tx_fifo_wr_addr:11;
      __IO uint32_t :5;
      __IO uint32_t tx_fifo_rd_addr:11;
    }RX_or_TX_fifo_addr_fld;
    __IO uint32_t RX_or_TX_fifo_addr;
  };

  union{ //offset addr 0x0054
    struct{
      __IO uint32_t :5;
      __IO uint32_t rx_fifo_wr_addr:11;
      __IO uint32_t :5;
      __IO uint32_t rx_fifo_rd_addr:11;
    }RX_or_TX_fifo_addr1_fld;
    __IO uint32_t RX_or_TX_fifo_addr1;
  };

  union{ //offset addr 0x0058
    struct{
      __IO uint32_t :23;
      __IO uint32_t rx_fifo_reset:1;
      __IO uint32_t :7;
      __IO uint32_t tx_fifo_reset:1;
    }RX_or_TX_fifo_reset_fld;
    __IO uint32_t RX_or_TX_fifo_reset;
  };

  union{ //offset addr 0x005c
    struct{
      __IO uint32_t :5;
      __IO uint32_t rd_cnt_last_ini:11;
      __IO uint32_t :5;
      __IO uint32_t rd_cnt_ini:11;
    }rd_cnt_ini_fld;
    __IO uint32_t rd_cnt_ini;
  };

  union{ //offset addr 0x0060
    struct{
      __IO uint32_t loop_timeout:32;
    }loop_timeout_fld;
    __IO uint32_t loop_timeout;
  };

  union{ //offset addr 0x0064
    struct{
      __IO uint32_t :28;
      __IO uint32_t nack_num:4;
    }Nack_num_fld;
    __IO uint32_t Nack_num;
  };

  union{ //offset addr 0x0068
    struct{
      __IO uint32_t :24;
      __IO uint32_t nack_num:8;
    }Nack_num1_fld;
    __IO uint32_t Nack_num1;
  };

  union{ //offset addr 0x006c
    struct{
      __IO uint32_t :16;
      __IO uint32_t anchor_point_cnt:16;
    }Anchor_point_cnt_fld;
    __IO uint32_t Anchor_point_cnt;
  };


  union{ //offset addr 0x0080
    struct{
      __IO uint32_t rx_test_mode_total_time_window:32;
    }RX_test_mode_fld;
    __IO uint32_t RX_test_mode;
  };

  union{ //offset addr 0x0084
    struct{
      __IO uint32_t rx_pkt_number:32;
    }rx_pkt_number_fld;
    __IO uint32_t rx_pkt_number;
  };

  union{ //offset addr 0x0088
    struct{
      __IO uint32_t crc_error_number:32;
    }Crc_error_number_fld;
    __IO uint32_t Crc_error_number;
  };

  union{ //offset addr 0x008c
    struct{
      __IO uint32_t sfd_error_sum:32;
    }sfd_error_sum_fld;
    __IO uint32_t sfd_error_sum;
  };

  union{ //offset addr 0x0090
    struct{
      __IO uint32_t sync_asate_sum:32;
    }Sync_asate_sum_fld;
    __IO uint32_t Sync_asate_sum;
  };

  union{ //offset addr 0x0094
    struct{
      __IO uint32_t freq_offset_sum:32;
    }Freq_offset_sum_fld;
    __IO uint32_t Freq_offset_sum;
  };

  union{ //offset addr 0x0098
    struct{
      __IO uint32_t nrssi_sum:32;
    }Nrssi_sum_fld;
    __IO uint32_t Nrssi_sum;
  };

  union{ //offset addr 0x009c
    struct{
      __IO uint32_t arssi_um:32;
    }Arssi_um_fld;
    __IO uint32_t Arssi_um;
  };

  union{ //offset addr 0x00a0
    struct{
      __IO uint32_t gain_sum0:32;
    }gain_sum0_fld;
    __IO uint32_t gain_sum0;
  };

  union{ //offset addr 0x00a4
    struct{
      __IO uint32_t gain_sum1:32;
    }gain_sum1_fld;
    __IO uint32_t gain_sum1;
  };

  union{ //offset addr 0x00a8
    struct{
      __IO uint32_t gain_sum2:32;
    }gain_sum2_fld;
    __IO uint32_t gain_sum2;
  };

  union{ //offset addr 0x00ac
    struct{
      __IO uint32_t gain_sum3:32;
    }gain_sum3_fld;
    __IO uint32_t gain_sum3;
  };

  union{ //offset addr 0x00b0
    struct{
      __IO uint32_t gain_sum4:32;
    }gain_sum4_fld;
    __IO uint32_t gain_sum4;
  };

  union{ //offset addr 0x00b4
    struct{
      __IO uint32_t gain_sum5:32;
    }gain_sum5_fld;
    __IO uint32_t gain_sum5;
  };

  union{ //offset addr 0x00b8
    struct{
      __IO uint32_t gain_sum6:32;
    }gain_sum6_fld;
    __IO uint32_t gain_sum6;
  };

  union{ //offset addr 0x00bc
    struct{
      __IO uint32_t gain_sum7:32;
    }gain_sum7_fld;
    __IO uint32_t gain_sum7;
  };

  union{ //offset addr 0x00c0
    struct{
      __IO uint32_t gain_sum8:32;
    }gain_sum8_fld;
    __IO uint32_t gain_sum8;
  };


  union{ //offset addr 0x0200
    struct{
      __IO uint32_t aod_data_read:32;
    }AOD_data_read_fld;
    __IO uint32_t AOD_data_read;
  };


  union{ //offset addr 0x0400
    struct{
      __IO uint32_t tx_fifo_write:32;
    }TX_fifo_write_fld;
    __IO uint32_t TX_fifo_write;
  };


  union{ //offset addr 0x0c00
    struct{
      __IO uint32_t rx_fifo_write:32;
    }RX_fifo_write_fld;
    __IO uint32_t RX_fifo_write;
  };

} PICO_REG_LL_TypeDef;

#define PICO_REG_LL PICO_REG_LL_TypeDef *0x40031000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_LL_H_

