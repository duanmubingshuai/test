#ifndef _PICO_REG_UART_H_
#define _PICO_REG_UART_H_

#include <stdint.h>

#define UART_COUNT 34

#define UART_BASE_ADDR 0x40004000

#define UART_SIZE 0x000000FC


 /**
 * @brief RBR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00       Receive_Buffer   0b0
 * </pre>
 */
#define UART_RBR_OFFSET 0x00000000


__INLINE uint32_t uart_rbr_get(void)
{
    return _PICO_REG_RD(UART_RBR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_RBR_RECEIVE_BUFFER_MASK                   ((uint32_t)0x000000FF)
#define UART_RBR_RECEIVE_BUFFER_LSB                    0
#define UART_RBR_RECEIVE_BUFFER_WIDTH                  ((uint32_t)0x00000008)

#define UART_RBR_RECEIVE_BUFFER_RST                    0x0

__INLINE void uart_rbr_unpack(uint8_t* receivebuffer)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_RBR_OFFSET + UART_BASE_ADDR);

    *receivebuffer = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_rbr_receive_buffer_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_RBR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

 /**
 * @brief THR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00     Transmit_Holding   0b0
 * </pre>
 */
#define UART_THR_OFFSET 0x00000000


__INLINE uint32_t uart_thr_get(void)
{
    return _PICO_REG_RD(UART_THR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_thr_set(uint32_t value)
{
    _PICO_REG_WR(UART_THR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_THR_TRANSMIT_HOLDING_MASK                 ((uint32_t)0x000000FF)
#define UART_THR_TRANSMIT_HOLDING_LSB                  0
#define UART_THR_TRANSMIT_HOLDING_WIDTH                ((uint32_t)0x00000008)

#define UART_THR_TRANSMIT_HOLDING_RST                  0x0

__INLINE void uart_thr_pack(uint8_t transmitholding)
{
    _PICO_REG_WR(UART_THR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)transmitholding << 0));
}

__INLINE void uart_thr_unpack(uint8_t* transmitholding)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_THR_OFFSET + UART_BASE_ADDR);

    *transmitholding = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE void uart_thr_transmit_holding_setf(uint8_t transmitholding)
{
    _PICO_REG_WR(UART_THR_OFFSET+ UART_BASE_ADDR, (uint32_t)transmitholding << 0);
}

 /**
 * @brief DLL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00    Divisor_Latch_low   0b0
 * </pre>
 */
#define UART_DLL_OFFSET 0x00000000


__INLINE uint32_t uart_dll_get(void)
{
    return _PICO_REG_RD(UART_DLL_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_dll_set(uint32_t value)
{
    _PICO_REG_WR(UART_DLL_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_DLL_DIVISOR_LATCH_LOW_MASK                ((uint32_t)0x000000FF)
#define UART_DLL_DIVISOR_LATCH_LOW_LSB                 0
#define UART_DLL_DIVISOR_LATCH_LOW_WIDTH               ((uint32_t)0x00000008)

#define UART_DLL_DIVISOR_LATCH_LOW_RST                 0x0

__INLINE void uart_dll_pack(uint8_t divisorlatchlow)
{
    _PICO_REG_WR(UART_DLL_OFFSET+ UART_BASE_ADDR,  ((uint32_t)divisorlatchlow << 0));
}

__INLINE void uart_dll_unpack(uint8_t* divisorlatchlow)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_DLL_OFFSET + UART_BASE_ADDR);

    *divisorlatchlow = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_dll_divisor_latch_low_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_DLL_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void uart_dll_divisor_latch_low_setf(uint8_t divisorlatchlow)
{
    _PICO_REG_WR(UART_DLL_OFFSET+ UART_BASE_ADDR, (uint32_t)divisorlatchlow << 0);
}

 /**
 * @brief DLH register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00   Divisor_Latch_high   0b0
 * </pre>
 */
#define UART_DLH_OFFSET 0x00000004


__INLINE uint32_t uart_dlh_get(void)
{
    return _PICO_REG_RD(UART_DLH_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_dlh_set(uint32_t value)
{
    _PICO_REG_WR(UART_DLH_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_DLH_DIVISOR_LATCH_HIGH_MASK               ((uint32_t)0x000000FF)
#define UART_DLH_DIVISOR_LATCH_HIGH_LSB                0
#define UART_DLH_DIVISOR_LATCH_HIGH_WIDTH              ((uint32_t)0x00000008)

#define UART_DLH_DIVISOR_LATCH_HIGH_RST                0x0

__INLINE void uart_dlh_pack(uint8_t divisorlatchhigh)
{
    _PICO_REG_WR(UART_DLH_OFFSET+ UART_BASE_ADDR,  ((uint32_t)divisorlatchhigh << 0));
}

__INLINE void uart_dlh_unpack(uint8_t* divisorlatchhigh)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_DLH_OFFSET + UART_BASE_ADDR);

    *divisorlatchhigh = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_dlh_divisor_latch_high_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_DLH_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void uart_dlh_divisor_latch_high_setf(uint8_t divisorlatchhigh)
{
    _PICO_REG_WR(UART_DLH_OFFSET+ UART_BASE_ADDR, (uint32_t)divisorlatchhigh << 0);
}

 /**
 * @brief IER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07                PTIME   0
 *     03                EDSSI   0
 *     02                 ELSI   0
 *     01                ETBEI   0
 *     00                ERBFI   0
 * </pre>
 */
#define UART_IER_OFFSET 0x00000004


__INLINE uint32_t uart_ier_get(void)
{
    return _PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_ier_set(uint32_t value)
{
    _PICO_REG_WR(UART_IER_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_IER_PTIME_BIT                             ((uint32_t)0x00000080)
#define UART_IER_PTIME_POS                             7
#define UART_IER_EDSSI_BIT                             ((uint32_t)0x00000008)
#define UART_IER_EDSSI_POS                             3
#define UART_IER_ELSI_BIT                              ((uint32_t)0x00000004)
#define UART_IER_ELSI_POS                              2
#define UART_IER_ETBEI_BIT                             ((uint32_t)0x00000002)
#define UART_IER_ETBEI_POS                             1
#define UART_IER_ERBFI_BIT                             ((uint32_t)0x00000001)
#define UART_IER_ERBFI_POS                             0

#define UART_IER_PTIME_RST                             0x0
#define UART_IER_EDSSI_RST                             0x0
#define UART_IER_ELSI_RST                              0x0
#define UART_IER_ETBEI_RST                             0x0
#define UART_IER_ERBFI_RST                             0x0

__INLINE void uart_ier_pack(uint8_t ptime, uint8_t edssi, uint8_t elsi, uint8_t etbei, uint8_t erbfi)
{
    _PICO_REG_WR(UART_IER_OFFSET+ UART_BASE_ADDR,  ((uint32_t)ptime << 7) | ((uint32_t)edssi << 3) | ((uint32_t)elsi << 2) | ((uint32_t)etbei << 1) | ((uint32_t)erbfi << 0));
}

__INLINE void uart_ier_unpack(uint8_t* ptime, uint8_t* edssi, uint8_t* elsi, uint8_t* etbei, uint8_t* erbfi)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR);

    *ptime = (localVal & ((uint32_t)0x00000080)) >> 7;
    *edssi = (localVal & ((uint32_t)0x00000008)) >> 3;
    *elsi = (localVal & ((uint32_t)0x00000004)) >> 2;
    *etbei = (localVal & ((uint32_t)0x00000002)) >> 1;
    *erbfi = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_ier_ptime_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void uart_ier_ptime_setf(uint8_t ptime)
{
    _PICO_REG_WR(UART_IER_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)ptime << 7));
}

__INLINE uint8_t uart_ier_edssi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void uart_ier_edssi_setf(uint8_t edssi)
{
    _PICO_REG_WR(UART_IER_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)edssi << 3));
}

__INLINE uint8_t uart_ier_elsi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void uart_ier_elsi_setf(uint8_t elsi)
{
    _PICO_REG_WR(UART_IER_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)elsi << 2));
}

__INLINE uint8_t uart_ier_etbei_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void uart_ier_etbei_setf(uint8_t etbei)
{
    _PICO_REG_WR(UART_IER_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)etbei << 1));
}

__INLINE uint8_t uart_ier_erbfi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_ier_erbfi_setf(uint8_t erbfi)
{
    _PICO_REG_WR(UART_IER_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_IER_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)erbfi << 0));
}

 /**
 * @brief IIR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:06               FIFOSE   0b0
 *  03:00                  IID   0b1
 * </pre>
 */
#define UART_IIR_OFFSET 0x00000008


__INLINE uint32_t uart_iir_get(void)
{
    return _PICO_REG_RD(UART_IIR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_IIR_FIFOSE_MASK                           ((uint32_t)0x000000C0)
#define UART_IIR_FIFOSE_LSB                            6
#define UART_IIR_FIFOSE_WIDTH                          ((uint32_t)0x00000002)
#define UART_IIR_IID_MASK                              ((uint32_t)0x0000000F)
#define UART_IIR_IID_LSB                               0
#define UART_IIR_IID_WIDTH                             ((uint32_t)0x00000004)

#define UART_IIR_FIFOSE_RST                            0x0
#define UART_IIR_IID_RST                               0x1

__INLINE void uart_iir_unpack(uint8_t* fifose, uint8_t* iid)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IIR_OFFSET + UART_BASE_ADDR);

    *fifose = (localVal & ((uint32_t)0x000000C0)) >> 6;
    *iid = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t uart_iir_fifose_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IIR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000C0)) >> 6);
}

__INLINE uint8_t uart_iir_iid_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_IIR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief FCR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:06                   RT   0b0
 *  05:04                  TET   0b0
 *     03                 DMAM   0
 *     02               XFIFOR   0
 *     01               RFIFOR   0
 *     00                FIFOE   0
 * </pre>
 */
#define UART_FCR_OFFSET 0x00000008


__INLINE uint32_t uart_fcr_get(void)
{
    return _PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_fcr_set(uint32_t value)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_FCR_RT_MASK                               ((uint32_t)0x000000C0)
#define UART_FCR_RT_LSB                                6
#define UART_FCR_RT_WIDTH                              ((uint32_t)0x00000002)
#define UART_FCR_TET_MASK                              ((uint32_t)0x00000030)
#define UART_FCR_TET_LSB                               4
#define UART_FCR_TET_WIDTH                             ((uint32_t)0x00000002)
#define UART_FCR_DMAM_BIT                              ((uint32_t)0x00000008)
#define UART_FCR_DMAM_POS                              3
#define UART_FCR_XFIFOR_BIT                            ((uint32_t)0x00000004)
#define UART_FCR_XFIFOR_POS                            2
#define UART_FCR_RFIFOR_BIT                            ((uint32_t)0x00000002)
#define UART_FCR_RFIFOR_POS                            1
#define UART_FCR_FIFOE_BIT                             ((uint32_t)0x00000001)
#define UART_FCR_FIFOE_POS                             0

#define UART_FCR_RT_RST                                0x0
#define UART_FCR_TET_RST                               0x0
#define UART_FCR_DMAM_RST                              0x0
#define UART_FCR_XFIFOR_RST                            0x0
#define UART_FCR_RFIFOR_RST                            0x0
#define UART_FCR_FIFOE_RST                             0x0

__INLINE void uart_fcr_pack(uint8_t rt, uint8_t tet, uint8_t dmam, uint8_t xfifor, uint8_t rfifor, uint8_t fifoe)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)rt << 6) | ((uint32_t)tet << 4) | ((uint32_t)dmam << 3) | ((uint32_t)xfifor << 2) | ((uint32_t)rfifor << 1) | ((uint32_t)fifoe << 0));
}

__INLINE void uart_fcr_unpack(uint8_t* rt, uint8_t* tet, uint8_t* dmam, uint8_t* xfifor, uint8_t* rfifor, uint8_t* fifoe)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR);

    *rt = (localVal & ((uint32_t)0x000000C0)) >> 6;
    *tet = (localVal & ((uint32_t)0x00000030)) >> 4;
    *dmam = (localVal & ((uint32_t)0x00000008)) >> 3;
    *xfifor = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rfifor = (localVal & ((uint32_t)0x00000002)) >> 1;
    *fifoe = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE void uart_fcr_rt_setf(uint8_t rt)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x000000C0)) | ((uint32_t)rt << 6));
}

__INLINE void uart_fcr_tet_setf(uint8_t tet)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000030)) | ((uint32_t)tet << 4));
}

__INLINE void uart_fcr_dmam_setf(uint8_t dmam)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)dmam << 3));
}

__INLINE void uart_fcr_xfifor_setf(uint8_t xfifor)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)xfifor << 2));
}

__INLINE void uart_fcr_rfifor_setf(uint8_t rfifor)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rfifor << 1));
}

__INLINE void uart_fcr_fifoe_setf(uint8_t fifoe)
{
    _PICO_REG_WR(UART_FCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_FCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)fifoe << 0));
}

 /**
 * @brief LCR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07                 DLAB   0
 *     06            Break_con   0
 *     04                  EPS   0
 *     03                  PEN   0
 *     02                 STOP   0
 *  01:00                  DLS   0b0
 * </pre>
 */
#define UART_LCR_OFFSET 0x0000000C


__INLINE uint32_t uart_lcr_get(void)
{
    return _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_lcr_set(uint32_t value)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_LCR_DLAB_BIT                              ((uint32_t)0x00000080)
#define UART_LCR_DLAB_POS                              7
#define UART_LCR_BREAK_CON_BIT                         ((uint32_t)0x00000040)
#define UART_LCR_BREAK_CON_POS                         6
#define UART_LCR_EPS_BIT                               ((uint32_t)0x00000010)
#define UART_LCR_EPS_POS                               4
#define UART_LCR_PEN_BIT                               ((uint32_t)0x00000008)
#define UART_LCR_PEN_POS                               3
#define UART_LCR_STOP_BIT                              ((uint32_t)0x00000004)
#define UART_LCR_STOP_POS                              2
#define UART_LCR_DLS_MASK                              ((uint32_t)0x00000003)
#define UART_LCR_DLS_LSB                               0
#define UART_LCR_DLS_WIDTH                             ((uint32_t)0x00000002)

#define UART_LCR_DLAB_RST                              0x0
#define UART_LCR_BREAK_CON_RST                         0x0
#define UART_LCR_EPS_RST                               0x0
#define UART_LCR_PEN_RST                               0x0
#define UART_LCR_STOP_RST                              0x0
#define UART_LCR_DLS_RST                               0x0

__INLINE void uart_lcr_pack(uint8_t dlab, uint8_t breakcon, uint8_t eps, uint8_t pen, uint8_t stop, uint8_t dls)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)dlab << 7) | ((uint32_t)breakcon << 6) | ((uint32_t)eps << 4) | ((uint32_t)pen << 3) | ((uint32_t)stop << 2) | ((uint32_t)dls << 0));
}

__INLINE void uart_lcr_unpack(uint8_t* dlab, uint8_t* breakcon, uint8_t* eps, uint8_t* pen, uint8_t* stop, uint8_t* dls)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);

    *dlab = (localVal & ((uint32_t)0x00000080)) >> 7;
    *breakcon = (localVal & ((uint32_t)0x00000040)) >> 6;
    *eps = (localVal & ((uint32_t)0x00000010)) >> 4;
    *pen = (localVal & ((uint32_t)0x00000008)) >> 3;
    *stop = (localVal & ((uint32_t)0x00000004)) >> 2;
    *dls = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t uart_lcr_dlab_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void uart_lcr_dlab_setf(uint8_t dlab)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)dlab << 7));
}

__INLINE uint8_t uart_lcr_break_con_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void uart_lcr_break_con_setf(uint8_t breakcon)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)breakcon << 6));
}

__INLINE uint8_t uart_lcr_eps_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void uart_lcr_eps_setf(uint8_t eps)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)eps << 4));
}

__INLINE uint8_t uart_lcr_pen_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void uart_lcr_pen_setf(uint8_t pen)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)pen << 3));
}

__INLINE uint8_t uart_lcr_stop_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void uart_lcr_stop_setf(uint8_t stop)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)stop << 2));
}

__INLINE uint8_t uart_lcr_dls_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void uart_lcr_dls_setf(uint8_t dls)
{
    _PICO_REG_WR(UART_LCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_LCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)dls << 0));
}

 /**
 * @brief MCR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06                 SIRE   0
 *     05                 AFCE   0
 *     04             LoopBack   0
 *     03                 OUT2   0
 *     02                 OUT1   0
 *     01                  RTS   0
 *     00                  DTR   0
 * </pre>
 */
#define UART_MCR_OFFSET 0x00000010


__INLINE uint32_t uart_mcr_get(void)
{
    return _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_mcr_set(uint32_t value)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_MCR_SIRE_BIT                              ((uint32_t)0x00000040)
#define UART_MCR_SIRE_POS                              6
#define UART_MCR_AFCE_BIT                              ((uint32_t)0x00000020)
#define UART_MCR_AFCE_POS                              5
#define UART_MCR_LOOP_BACK_BIT                         ((uint32_t)0x00000010)
#define UART_MCR_LOOP_BACK_POS                         4
#define UART_MCR_OUT2_BIT                              ((uint32_t)0x00000008)
#define UART_MCR_OUT2_POS                              3
#define UART_MCR_OUT1_BIT                              ((uint32_t)0x00000004)
#define UART_MCR_OUT1_POS                              2
#define UART_MCR_RTS_BIT                               ((uint32_t)0x00000002)
#define UART_MCR_RTS_POS                               1
#define UART_MCR_DTR_BIT                               ((uint32_t)0x00000001)
#define UART_MCR_DTR_POS                               0

#define UART_MCR_SIRE_RST                              0x0
#define UART_MCR_AFCE_RST                              0x0
#define UART_MCR_LOOP_BACK_RST                         0x0
#define UART_MCR_OUT2_RST                              0x0
#define UART_MCR_OUT1_RST                              0x0
#define UART_MCR_RTS_RST                               0x0
#define UART_MCR_DTR_RST                               0x0

__INLINE void uart_mcr_pack(uint8_t sire, uint8_t afce, uint8_t loopback, uint8_t out2, uint8_t out1, uint8_t rts, uint8_t dtr)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)sire << 6) | ((uint32_t)afce << 5) | ((uint32_t)loopback << 4) | ((uint32_t)out2 << 3) | ((uint32_t)out1 << 2) | ((uint32_t)rts << 1) | ((uint32_t)dtr << 0));
}

__INLINE void uart_mcr_unpack(uint8_t* sire, uint8_t* afce, uint8_t* loopback, uint8_t* out2, uint8_t* out1, uint8_t* rts, uint8_t* dtr)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);

    *sire = (localVal & ((uint32_t)0x00000040)) >> 6;
    *afce = (localVal & ((uint32_t)0x00000020)) >> 5;
    *loopback = (localVal & ((uint32_t)0x00000010)) >> 4;
    *out2 = (localVal & ((uint32_t)0x00000008)) >> 3;
    *out1 = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rts = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dtr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_mcr_sire_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void uart_mcr_sire_setf(uint8_t sire)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)sire << 6));
}

__INLINE uint8_t uart_mcr_afce_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void uart_mcr_afce_setf(uint8_t afce)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)afce << 5));
}

__INLINE uint8_t uart_mcr_loop_back_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void uart_mcr_loop_back_setf(uint8_t loopback)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)loopback << 4));
}

__INLINE uint8_t uart_mcr_out2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void uart_mcr_out2_setf(uint8_t out2)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)out2 << 3));
}

__INLINE uint8_t uart_mcr_out1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void uart_mcr_out1_setf(uint8_t out1)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)out1 << 2));
}

__INLINE uint8_t uart_mcr_rts_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void uart_mcr_rts_setf(uint8_t rts)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rts << 1));
}

__INLINE uint8_t uart_mcr_dtr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_mcr_dtr_setf(uint8_t dtr)
{
    _PICO_REG_WR(UART_MCR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_MCR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)dtr << 0));
}

 /**
 * @brief LSR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07                  RFE   0
 *     06                 TEMT   1
 *     05                 THRE   1
 *     04                   BI   0
 *     03                   FE   0
 *     02                   PE   0
 *     01                   OE   0
 *     00                   DR   0
 * </pre>
 */
#define UART_LSR_OFFSET 0x00000014


__INLINE uint32_t uart_lsr_get(void)
{
    return _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_LSR_RFE_BIT                               ((uint32_t)0x00000080)
#define UART_LSR_RFE_POS                               7
#define UART_LSR_TEMT_BIT                              ((uint32_t)0x00000040)
#define UART_LSR_TEMT_POS                              6
#define UART_LSR_THRE_BIT                              ((uint32_t)0x00000020)
#define UART_LSR_THRE_POS                              5
#define UART_LSR_BI_BIT                                ((uint32_t)0x00000010)
#define UART_LSR_BI_POS                                4
#define UART_LSR_FE_BIT                                ((uint32_t)0x00000008)
#define UART_LSR_FE_POS                                3
#define UART_LSR_PE_BIT                                ((uint32_t)0x00000004)
#define UART_LSR_PE_POS                                2
#define UART_LSR_OE_BIT                                ((uint32_t)0x00000002)
#define UART_LSR_OE_POS                                1
#define UART_LSR_DR_BIT                                ((uint32_t)0x00000001)
#define UART_LSR_DR_POS                                0

#define UART_LSR_RFE_RST                               0x0
#define UART_LSR_TEMT_RST                              0x1
#define UART_LSR_THRE_RST                              0x1
#define UART_LSR_BI_RST                                0x0
#define UART_LSR_FE_RST                                0x0
#define UART_LSR_PE_RST                                0x0
#define UART_LSR_OE_RST                                0x0
#define UART_LSR_DR_RST                                0x0

__INLINE void uart_lsr_unpack(uint8_t* rfe, uint8_t* temt, uint8_t* thre, uint8_t* bi, uint8_t* fe, uint8_t* pe, uint8_t* oe, uint8_t* dr)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);

    *rfe = (localVal & ((uint32_t)0x00000080)) >> 7;
    *temt = (localVal & ((uint32_t)0x00000040)) >> 6;
    *thre = (localVal & ((uint32_t)0x00000020)) >> 5;
    *bi = (localVal & ((uint32_t)0x00000010)) >> 4;
    *fe = (localVal & ((uint32_t)0x00000008)) >> 3;
    *pe = (localVal & ((uint32_t)0x00000004)) >> 2;
    *oe = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_lsr_rfe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t uart_lsr_temt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t uart_lsr_thre_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t uart_lsr_bi_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_lsr_fe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t uart_lsr_pe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t uart_lsr_oe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t uart_lsr_dr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief MSR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07                  DCD   0
 *     06                   RI   0
 *     05                  DSR   0
 *     04                  CTS   0
 *     03                 DDCD   0
 *     02                 TERI   0
 *     01                 DDSR   0
 *     00                 DCTS   0
 * </pre>
 */
#define UART_MSR_OFFSET 0x00000018


__INLINE uint32_t uart_msr_get(void)
{
    return _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_MSR_DCD_BIT                               ((uint32_t)0x00000080)
#define UART_MSR_DCD_POS                               7
#define UART_MSR_RI_BIT                                ((uint32_t)0x00000040)
#define UART_MSR_RI_POS                                6
#define UART_MSR_DSR_BIT                               ((uint32_t)0x00000020)
#define UART_MSR_DSR_POS                               5
#define UART_MSR_CTS_BIT                               ((uint32_t)0x00000010)
#define UART_MSR_CTS_POS                               4
#define UART_MSR_DDCD_BIT                              ((uint32_t)0x00000008)
#define UART_MSR_DDCD_POS                              3
#define UART_MSR_TERI_BIT                              ((uint32_t)0x00000004)
#define UART_MSR_TERI_POS                              2
#define UART_MSR_DDSR_BIT                              ((uint32_t)0x00000002)
#define UART_MSR_DDSR_POS                              1
#define UART_MSR_DCTS_BIT                              ((uint32_t)0x00000001)
#define UART_MSR_DCTS_POS                              0

#define UART_MSR_DCD_RST                               0x0
#define UART_MSR_RI_RST                                0x0
#define UART_MSR_DSR_RST                               0x0
#define UART_MSR_CTS_RST                               0x0
#define UART_MSR_DDCD_RST                              0x0
#define UART_MSR_TERI_RST                              0x0
#define UART_MSR_DDSR_RST                              0x0
#define UART_MSR_DCTS_RST                              0x0

__INLINE void uart_msr_unpack(uint8_t* dcd, uint8_t* ri, uint8_t* dsr, uint8_t* cts, uint8_t* ddcd, uint8_t* teri, uint8_t* ddsr, uint8_t* dcts)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);

    *dcd = (localVal & ((uint32_t)0x00000080)) >> 7;
    *ri = (localVal & ((uint32_t)0x00000040)) >> 6;
    *dsr = (localVal & ((uint32_t)0x00000020)) >> 5;
    *cts = (localVal & ((uint32_t)0x00000010)) >> 4;
    *ddcd = (localVal & ((uint32_t)0x00000008)) >> 3;
    *teri = (localVal & ((uint32_t)0x00000004)) >> 2;
    *ddsr = (localVal & ((uint32_t)0x00000002)) >> 1;
    *dcts = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_msr_dcd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t uart_msr_ri_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t uart_msr_dsr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t uart_msr_cts_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_msr_ddcd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t uart_msr_teri_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t uart_msr_ddsr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t uart_msr_dcts_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_MSR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief SCR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00           Scratchpad   0b0
 * </pre>
 */
#define UART_SCR_OFFSET 0x0000001C


__INLINE uint32_t uart_scr_get(void)
{
    return _PICO_REG_RD(UART_SCR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_scr_set(uint32_t value)
{
    _PICO_REG_WR(UART_SCR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_SCR_SCRATCHPAD_MASK                       ((uint32_t)0x000000FF)
#define UART_SCR_SCRATCHPAD_LSB                        0
#define UART_SCR_SCRATCHPAD_WIDTH                      ((uint32_t)0x00000008)

#define UART_SCR_SCRATCHPAD_RST                        0x0

__INLINE void uart_scr_pack(uint8_t scratchpad)
{
    _PICO_REG_WR(UART_SCR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)scratchpad << 0));
}

__INLINE void uart_scr_unpack(uint8_t* scratchpad)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SCR_OFFSET + UART_BASE_ADDR);

    *scratchpad = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_scr_scratchpad_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void uart_scr_scratchpad_setf(uint8_t scratchpad)
{
    _PICO_REG_WR(UART_SCR_OFFSET+ UART_BASE_ADDR, (uint32_t)scratchpad << 0);
}

 /**
 * @brief LPDLL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                LPDLL   0b0
 * </pre>
 */
#define UART_LPDLL_OFFSET 0x00000020


__INLINE uint32_t uart_lpdll_get(void)
{
    return _PICO_REG_RD(UART_LPDLL_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_lpdll_set(uint32_t value)
{
    _PICO_REG_WR(UART_LPDLL_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_LPDLL_LPDLL_MASK                            ((uint32_t)0x000000FF)
#define UART_LPDLL_LPDLL_LSB                             0
#define UART_LPDLL_LPDLL_WIDTH                           ((uint32_t)0x00000008)

#define UART_LPDLL_LPDLL_RST                             0x0

__INLINE void uart_lpdll_pack(uint8_t lpdll)
{
    _PICO_REG_WR(UART_LPDLL_OFFSET+ UART_BASE_ADDR,  ((uint32_t)lpdll << 0));
}

__INLINE void uart_lpdll_unpack(uint8_t* lpdll)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LPDLL_OFFSET + UART_BASE_ADDR);

    *lpdll = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_lpdll_lpdll_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LPDLL_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void uart_lpdll_lpdll_setf(uint8_t lpdll)
{
    _PICO_REG_WR(UART_LPDLL_OFFSET+ UART_BASE_ADDR, (uint32_t)lpdll << 0);
}

 /**
 * @brief LPDLH register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                LPDLH   0b0
 * </pre>
 */
#define UART_LPDLH_OFFSET 0x00000024


__INLINE uint32_t uart_lpdlh_get(void)
{
    return _PICO_REG_RD(UART_LPDLH_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_lpdlh_set(uint32_t value)
{
    _PICO_REG_WR(UART_LPDLH_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_LPDLH_LPDLH_MASK                            ((uint32_t)0x000000FF)
#define UART_LPDLH_LPDLH_LSB                             0
#define UART_LPDLH_LPDLH_WIDTH                           ((uint32_t)0x00000008)

#define UART_LPDLH_LPDLH_RST                             0x0

__INLINE void uart_lpdlh_pack(uint8_t lpdlh)
{
    _PICO_REG_WR(UART_LPDLH_OFFSET+ UART_BASE_ADDR,  ((uint32_t)lpdlh << 0));
}

__INLINE void uart_lpdlh_unpack(uint8_t* lpdlh)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LPDLH_OFFSET + UART_BASE_ADDR);

    *lpdlh = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_lpdlh_lpdlh_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_LPDLH_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void uart_lpdlh_lpdlh_setf(uint8_t lpdlh)
{
    _PICO_REG_WR(UART_LPDLH_OFFSET+ UART_BASE_ADDR, (uint32_t)lpdlh << 0);
}

 /**
 * @brief SRBR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00   Shadow_Receive_Buffer   0b0
 * </pre>
 */
#define UART_SRBR_OFFSET 0x00000030


__INLINE uint32_t uart_srbr_get(void)
{
    return _PICO_REG_RD(UART_SRBR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_SRBR_SHADOW_RECEIVE_BUFFER_MASK            ((uint32_t)0x000000FF)
#define UART_SRBR_SHADOW_RECEIVE_BUFFER_LSB             0
#define UART_SRBR_SHADOW_RECEIVE_BUFFER_WIDTH           ((uint32_t)0x00000008)

#define UART_SRBR_SHADOW_RECEIVE_BUFFER_RST             0x0

__INLINE void uart_srbr_unpack(uint8_t* shadowreceivebuffer)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SRBR_OFFSET + UART_BASE_ADDR);

    *shadowreceivebuffer = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_srbr_shadow_receive_buffer_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SRBR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

 /**
 * @brief STHR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00   Shadow_Transmit_Holding   0b0
 * </pre>
 */
#define UART_STHR_OFFSET 0x00000030


__INLINE uint32_t uart_sthr_get(void)
{
    return _PICO_REG_RD(UART_STHR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_sthr_set(uint32_t value)
{
    _PICO_REG_WR(UART_STHR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_STHR_SHADOW_TRANSMIT_HOLDING_MASK          ((uint32_t)0x000000FF)
#define UART_STHR_SHADOW_TRANSMIT_HOLDING_LSB           0
#define UART_STHR_SHADOW_TRANSMIT_HOLDING_WIDTH         ((uint32_t)0x00000008)

#define UART_STHR_SHADOW_TRANSMIT_HOLDING_RST           0x0

__INLINE void uart_sthr_pack(uint8_t shadowtransmitholding)
{
    _PICO_REG_WR(UART_STHR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)shadowtransmitholding << 0));
}

__INLINE void uart_sthr_unpack(uint8_t* shadowtransmitholding)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_STHR_OFFSET + UART_BASE_ADDR);

    *shadowtransmitholding = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE void uart_sthr_shadow_transmit_holding_setf(uint8_t shadowtransmitholding)
{
    _PICO_REG_WR(UART_STHR_OFFSET+ UART_BASE_ADDR, (uint32_t)shadowtransmitholding << 0);
}

 /**
 * @brief FAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00          FIFO_Access   0
 * </pre>
 */
#define UART_FAR_OFFSET 0x00000070


__INLINE uint32_t uart_far_get(void)
{
    return _PICO_REG_RD(UART_FAR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_far_set(uint32_t value)
{
    _PICO_REG_WR(UART_FAR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_FAR_FIFO_ACCESS_BIT                       ((uint32_t)0x00000001)
#define UART_FAR_FIFO_ACCESS_POS                       0

#define UART_FAR_FIFO_ACCESS_RST                       0x0

__INLINE void uart_far_pack(uint8_t fifoaccess)
{
    _PICO_REG_WR(UART_FAR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)fifoaccess << 0));
}

__INLINE void uart_far_unpack(uint8_t* fifoaccess)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_FAR_OFFSET + UART_BASE_ADDR);

    *fifoaccess = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_far_fifo_access_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_FAR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_far_fifo_access_setf(uint8_t fifoaccess)
{
    _PICO_REG_WR(UART_FAR_OFFSET+ UART_BASE_ADDR, (uint32_t)fifoaccess << 0);
}

 /**
 * @brief TFR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00   Transmit_FIFO_Read   0b0
 * </pre>
 */
#define UART_TFR_OFFSET 0x00000074


__INLINE uint32_t uart_tfr_get(void)
{
    return _PICO_REG_RD(UART_TFR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_TFR_TRANSMIT_FIFO_READ_MASK               ((uint32_t)0x000000FF)
#define UART_TFR_TRANSMIT_FIFO_READ_LSB                0
#define UART_TFR_TRANSMIT_FIFO_READ_WIDTH              ((uint32_t)0x00000008)

#define UART_TFR_TRANSMIT_FIFO_READ_RST                0x0

__INLINE void uart_tfr_unpack(uint8_t* transmitfiforead)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_TFR_OFFSET + UART_BASE_ADDR);

    *transmitfiforead = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t uart_tfr_transmit_fifo_read_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_TFR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

 /**
 * @brief RFW register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     09                 RFFE   0
 *     08                 RFPE   0
 *  07:00                 RFWD   0b0
 * </pre>
 */
#define UART_RFW_OFFSET 0x00000078


__INLINE uint32_t uart_rfw_get(void)
{
    return _PICO_REG_RD(UART_RFW_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_rfw_set(uint32_t value)
{
    _PICO_REG_WR(UART_RFW_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_RFW_RFFE_BIT                              ((uint32_t)0x00000200)
#define UART_RFW_RFFE_POS                              9
#define UART_RFW_RFPE_BIT                              ((uint32_t)0x00000100)
#define UART_RFW_RFPE_POS                              8
#define UART_RFW_RFWD_MASK                             ((uint32_t)0x000000FF)
#define UART_RFW_RFWD_LSB                              0
#define UART_RFW_RFWD_WIDTH                            ((uint32_t)0x00000008)

#define UART_RFW_RFFE_RST                              0x0
#define UART_RFW_RFPE_RST                              0x0
#define UART_RFW_RFWD_RST                              0x0

__INLINE void uart_rfw_pack(uint8_t rffe, uint8_t rfpe, uint8_t rfwd)
{
    _PICO_REG_WR(UART_RFW_OFFSET+ UART_BASE_ADDR,  ((uint32_t)rffe << 9) | ((uint32_t)rfpe << 8) | ((uint32_t)rfwd << 0));
}

__INLINE void uart_rfw_unpack(uint8_t* rffe, uint8_t* rfpe, uint8_t* rfwd)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_RFW_OFFSET + UART_BASE_ADDR);

    *rffe = (localVal & ((uint32_t)0x00000200)) >> 9;
    *rfpe = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rfwd = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE void uart_rfw_rffe_setf(uint8_t rffe)
{
    _PICO_REG_WR(UART_RFW_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_RFW_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)rffe << 9));
}

__INLINE void uart_rfw_rfpe_setf(uint8_t rfpe)
{
    _PICO_REG_WR(UART_RFW_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_RFW_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)rfpe << 8));
}

__INLINE void uart_rfw_rfwd_setf(uint8_t rfwd)
{
    _PICO_REG_WR(UART_RFW_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_RFW_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)rfwd << 0));
}

 /**
 * @brief USR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     04                  RFF   0
 *     03                 RFNE   0
 *     02                  TFE   1
 *     01                 TFNF   1
 *     00                 BUSY   0
 * </pre>
 */
#define UART_USR_OFFSET 0x0000007C


__INLINE uint32_t uart_usr_get(void)
{
    return _PICO_REG_RD(UART_USR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_USR_RFF_BIT                               ((uint32_t)0x00000010)
#define UART_USR_RFF_POS                               4
#define UART_USR_RFNE_BIT                              ((uint32_t)0x00000008)
#define UART_USR_RFNE_POS                              3
#define UART_USR_TFE_BIT                               ((uint32_t)0x00000004)
#define UART_USR_TFE_POS                               2
#define UART_USR_TFNF_BIT                              ((uint32_t)0x00000002)
#define UART_USR_TFNF_POS                              1
#define UART_USR_BUSY_BIT                              ((uint32_t)0x00000001)
#define UART_USR_BUSY_POS                              0

#define UART_USR_RFF_RST                               0x0
#define UART_USR_RFNE_RST                              0x0
#define UART_USR_TFE_RST                               0x1
#define UART_USR_TFNF_RST                              0x1
#define UART_USR_BUSY_RST                              0x0

__INLINE void uart_usr_unpack(uint8_t* rff, uint8_t* rfne, uint8_t* tfe, uint8_t* tfnf, uint8_t* busy)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_USR_OFFSET + UART_BASE_ADDR);

    *rff = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rfne = (localVal & ((uint32_t)0x00000008)) >> 3;
    *tfe = (localVal & ((uint32_t)0x00000004)) >> 2;
    *tfnf = (localVal & ((uint32_t)0x00000002)) >> 1;
    *busy = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_usr_rff_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_USR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_usr_rfne_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_USR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t uart_usr_tfe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_USR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t uart_usr_tfnf_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_USR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t uart_usr_busy_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_USR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TFL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  04:00   Transmit_FIFO_Level   0b0
 * </pre>
 */
#define UART_TFL_OFFSET 0x00000080


__INLINE uint32_t uart_tfl_get(void)
{
    return _PICO_REG_RD(UART_TFL_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_TFL_TRANSMIT_FIFO_LEVEL_MASK              ((uint32_t)0x0000001F)
#define UART_TFL_TRANSMIT_FIFO_LEVEL_LSB               0
#define UART_TFL_TRANSMIT_FIFO_LEVEL_WIDTH             ((uint32_t)0x00000005)

#define UART_TFL_TRANSMIT_FIFO_LEVEL_RST               0x0

__INLINE void uart_tfl_unpack(uint8_t* transmitfifolevel)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_TFL_OFFSET + UART_BASE_ADDR);

    *transmitfifolevel = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t uart_tfl_transmit_fifo_level_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_TFL_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

 /**
 * @brief RFL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  04:00   Receive_FIFO_Level   0b0
 * </pre>
 */
#define UART_RFL_OFFSET 0x00000084


__INLINE uint32_t uart_rfl_get(void)
{
    return _PICO_REG_RD(UART_RFL_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_RFL_RECEIVE_FIFO_LEVEL_MASK               ((uint32_t)0x0000001F)
#define UART_RFL_RECEIVE_FIFO_LEVEL_LSB                0
#define UART_RFL_RECEIVE_FIFO_LEVEL_WIDTH              ((uint32_t)0x00000005)

#define UART_RFL_RECEIVE_FIFO_LEVEL_RST                0x0

__INLINE void uart_rfl_unpack(uint8_t* receivefifolevel)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_RFL_OFFSET + UART_BASE_ADDR);

    *receivefifolevel = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t uart_rfl_receive_fifo_level_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_RFL_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

 /**
 * @brief SRR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02                  XFR   0
 *     01                  RFR   0
 *     00                   UR   0
 * </pre>
 */
#define UART_SRR_OFFSET 0x00000088


__INLINE uint32_t uart_srr_get(void)
{
    return _PICO_REG_RD(UART_SRR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_srr_set(uint32_t value)
{
    _PICO_REG_WR(UART_SRR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_SRR_XFR_BIT                               ((uint32_t)0x00000004)
#define UART_SRR_XFR_POS                               2
#define UART_SRR_RFR_BIT                               ((uint32_t)0x00000002)
#define UART_SRR_RFR_POS                               1
#define UART_SRR_UR_BIT                                ((uint32_t)0x00000001)
#define UART_SRR_UR_POS                                0

#define UART_SRR_XFR_RST                               0x0
#define UART_SRR_RFR_RST                               0x0
#define UART_SRR_UR_RST                                0x0

__INLINE void uart_srr_pack(uint8_t xfr, uint8_t rfr, uint8_t ur)
{
    _PICO_REG_WR(UART_SRR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)xfr << 2) | ((uint32_t)rfr << 1) | ((uint32_t)ur << 0));
}

__INLINE void uart_srr_unpack(uint8_t* xfr, uint8_t* rfr, uint8_t* ur)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SRR_OFFSET + UART_BASE_ADDR);

    *xfr = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rfr = (localVal & ((uint32_t)0x00000002)) >> 1;
    *ur = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE void uart_srr_xfr_setf(uint8_t xfr)
{
    _PICO_REG_WR(UART_SRR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_SRR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)xfr << 2));
}

__INLINE void uart_srr_rfr_setf(uint8_t rfr)
{
    _PICO_REG_WR(UART_SRR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_SRR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rfr << 1));
}

__INLINE void uart_srr_ur_setf(uint8_t ur)
{
    _PICO_REG_WR(UART_SRR_OFFSET+ UART_BASE_ADDR, (_PICO_REG_RD(UART_SRR_OFFSET + UART_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)ur << 0));
}

 /**
 * @brief SRTS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Shadow_Request_to_Send   0
 * </pre>
 */
#define UART_SRTS_OFFSET 0x0000008C


__INLINE uint32_t uart_srts_get(void)
{
    return _PICO_REG_RD(UART_SRTS_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_srts_set(uint32_t value)
{
    _PICO_REG_WR(UART_SRTS_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_SRTS_SHADOW_REQUEST_TO_SEND_BIT            ((uint32_t)0x00000001)
#define UART_SRTS_SHADOW_REQUEST_TO_SEND_POS            0

#define UART_SRTS_SHADOW_REQUEST_TO_SEND_RST            0x0

__INLINE void uart_srts_pack(uint8_t shadowrequesttosend)
{
    _PICO_REG_WR(UART_SRTS_OFFSET+ UART_BASE_ADDR,  ((uint32_t)shadowrequesttosend << 0));
}

__INLINE void uart_srts_unpack(uint8_t* shadowrequesttosend)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SRTS_OFFSET + UART_BASE_ADDR);

    *shadowrequesttosend = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_srts_shadow_request_to_send_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SRTS_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_srts_shadow_request_to_send_setf(uint8_t shadowrequesttosend)
{
    _PICO_REG_WR(UART_SRTS_OFFSET+ UART_BASE_ADDR, (uint32_t)shadowrequesttosend << 0);
}

 /**
 * @brief SBCR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Shadow_Break_Control   0
 * </pre>
 */
#define UART_SBCR_OFFSET 0x00000090


__INLINE uint32_t uart_sbcr_get(void)
{
    return _PICO_REG_RD(UART_SBCR_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_sbcr_set(uint32_t value)
{
    _PICO_REG_WR(UART_SBCR_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_SBCR_SHADOW_BREAK_CONTROL_BIT              ((uint32_t)0x00000001)
#define UART_SBCR_SHADOW_BREAK_CONTROL_POS              0

#define UART_SBCR_SHADOW_BREAK_CONTROL_RST              0x0

__INLINE void uart_sbcr_pack(uint8_t shadowbreakcontrol)
{
    _PICO_REG_WR(UART_SBCR_OFFSET+ UART_BASE_ADDR,  ((uint32_t)shadowbreakcontrol << 0));
}

__INLINE void uart_sbcr_unpack(uint8_t* shadowbreakcontrol)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SBCR_OFFSET + UART_BASE_ADDR);

    *shadowbreakcontrol = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_sbcr_shadow_break_control_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SBCR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_sbcr_shadow_break_control_setf(uint8_t shadowbreakcontrol)
{
    _PICO_REG_WR(UART_SBCR_OFFSET+ UART_BASE_ADDR, (uint32_t)shadowbreakcontrol << 0);
}

 /**
 * @brief SDMAM register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00      Shadow_DMA_Mode   0
 * </pre>
 */
#define UART_SDMAM_OFFSET 0x00000094


__INLINE uint32_t uart_sdmam_get(void)
{
    return _PICO_REG_RD(UART_SDMAM_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_sdmam_set(uint32_t value)
{
    _PICO_REG_WR(UART_SDMAM_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_SDMAM_SHADOW_DMA_MODE_BIT                   ((uint32_t)0x00000001)
#define UART_SDMAM_SHADOW_DMA_MODE_POS                   0

#define UART_SDMAM_SHADOW_DMA_MODE_RST                   0x0

__INLINE void uart_sdmam_pack(uint8_t shadowdmamode)
{
    _PICO_REG_WR(UART_SDMAM_OFFSET+ UART_BASE_ADDR,  ((uint32_t)shadowdmamode << 0));
}

__INLINE void uart_sdmam_unpack(uint8_t* shadowdmamode)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SDMAM_OFFSET + UART_BASE_ADDR);

    *shadowdmamode = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_sdmam_shadow_dma_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SDMAM_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_sdmam_shadow_dma_mode_setf(uint8_t shadowdmamode)
{
    _PICO_REG_WR(UART_SDMAM_OFFSET+ UART_BASE_ADDR, (uint32_t)shadowdmamode << 0);
}

 /**
 * @brief SFE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Shadow_FIFO_Enable   0
 * </pre>
 */
#define UART_SFE_OFFSET 0x00000098


__INLINE uint32_t uart_sfe_get(void)
{
    return _PICO_REG_RD(UART_SFE_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_sfe_set(uint32_t value)
{
    _PICO_REG_WR(UART_SFE_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_SFE_SHADOW_FIFO_ENABLE_BIT                ((uint32_t)0x00000001)
#define UART_SFE_SHADOW_FIFO_ENABLE_POS                0

#define UART_SFE_SHADOW_FIFO_ENABLE_RST                0x0

__INLINE void uart_sfe_pack(uint8_t shadowfifoenable)
{
    _PICO_REG_WR(UART_SFE_OFFSET+ UART_BASE_ADDR,  ((uint32_t)shadowfifoenable << 0));
}

__INLINE void uart_sfe_unpack(uint8_t* shadowfifoenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SFE_OFFSET + UART_BASE_ADDR);

    *shadowfifoenable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_sfe_shadow_fifo_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SFE_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_sfe_shadow_fifo_enable_setf(uint8_t shadowfifoenable)
{
    _PICO_REG_WR(UART_SFE_OFFSET+ UART_BASE_ADDR, (uint32_t)shadowfifoenable << 0);
}

 /**
 * @brief SRT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00   Shadow_RCVR_Trigger   0b0
 * </pre>
 */
#define UART_SRT_OFFSET 0x0000009C


__INLINE uint32_t uart_srt_get(void)
{
    return _PICO_REG_RD(UART_SRT_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_srt_set(uint32_t value)
{
    _PICO_REG_WR(UART_SRT_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_SRT_SHADOW_RCVR_TRIGGER_MASK              ((uint32_t)0x00000003)
#define UART_SRT_SHADOW_RCVR_TRIGGER_LSB               0
#define UART_SRT_SHADOW_RCVR_TRIGGER_WIDTH             ((uint32_t)0x00000002)

#define UART_SRT_SHADOW_RCVR_TRIGGER_RST               0x0

__INLINE void uart_srt_pack(uint8_t shadowrcvrtrigger)
{
    _PICO_REG_WR(UART_SRT_OFFSET+ UART_BASE_ADDR,  ((uint32_t)shadowrcvrtrigger << 0));
}

__INLINE void uart_srt_unpack(uint8_t* shadowrcvrtrigger)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SRT_OFFSET + UART_BASE_ADDR);

    *shadowrcvrtrigger = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t uart_srt_shadow_rcvr_trigger_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_SRT_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void uart_srt_shadow_rcvr_trigger_setf(uint8_t shadowrcvrtrigger)
{
    _PICO_REG_WR(UART_SRT_OFFSET+ UART_BASE_ADDR, (uint32_t)shadowrcvrtrigger << 0);
}

 /**
 * @brief STET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00   Shadow_TX_Empty_Trigger   0b0
 * </pre>
 */
#define UART_STET_OFFSET 0x000000A0


__INLINE uint32_t uart_stet_get(void)
{
    return _PICO_REG_RD(UART_STET_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_stet_set(uint32_t value)
{
    _PICO_REG_WR(UART_STET_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_STET_SHADOW_TX_EMPTY_TRIGGER_MASK          ((uint32_t)0x00000003)
#define UART_STET_SHADOW_TX_EMPTY_TRIGGER_LSB           0
#define UART_STET_SHADOW_TX_EMPTY_TRIGGER_WIDTH         ((uint32_t)0x00000002)

#define UART_STET_SHADOW_TX_EMPTY_TRIGGER_RST           0x0

__INLINE void uart_stet_pack(uint8_t shadowtxemptytrigger)
{
    _PICO_REG_WR(UART_STET_OFFSET+ UART_BASE_ADDR,  ((uint32_t)shadowtxemptytrigger << 0));
}

__INLINE void uart_stet_unpack(uint8_t* shadowtxemptytrigger)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_STET_OFFSET + UART_BASE_ADDR);

    *shadowtxemptytrigger = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t uart_stet_shadow_tx_empty_trigger_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_STET_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void uart_stet_shadow_tx_empty_trigger_setf(uint8_t shadowtxemptytrigger)
{
    _PICO_REG_WR(UART_STET_OFFSET+ UART_BASE_ADDR, (uint32_t)shadowtxemptytrigger << 0);
}

 /**
 * @brief HTX register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00              Halt_TX   0
 * </pre>
 */
#define UART_HTX_OFFSET 0x000000A4


__INLINE uint32_t uart_htx_get(void)
{
    return _PICO_REG_RD(UART_HTX_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_htx_set(uint32_t value)
{
    _PICO_REG_WR(UART_HTX_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_HTX_HALT_TX_BIT                           ((uint32_t)0x00000001)
#define UART_HTX_HALT_TX_POS                           0

#define UART_HTX_HALT_TX_RST                           0x0

__INLINE void uart_htx_pack(uint8_t halttx)
{
    _PICO_REG_WR(UART_HTX_OFFSET+ UART_BASE_ADDR,  ((uint32_t)halttx << 0));
}

__INLINE void uart_htx_unpack(uint8_t* halttx)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_HTX_OFFSET + UART_BASE_ADDR);

    *halttx = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t uart_htx_halt_tx_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_HTX_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void uart_htx_halt_tx_setf(uint8_t halttx)
{
    _PICO_REG_WR(UART_HTX_OFFSET+ UART_BASE_ADDR, (uint32_t)halttx << 0);
}

 /**
 * @brief DMASA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   DMA_Software_Acknowledge   0
 * </pre>
 */
#define UART_DMASA_OFFSET 0x000000A8


__INLINE uint32_t uart_dmasa_get(void)
{
    return _PICO_REG_RD(UART_DMASA_OFFSET + UART_BASE_ADDR);
}

__INLINE void uart_dmasa_set(uint32_t value)
{
    _PICO_REG_WR(UART_DMASA_OFFSET+ UART_BASE_ADDR, value);
}

// field definitions
#define UART_DMASA_DMA_SOFTWARE_ACKNOWLEDGE_BIT          ((uint32_t)0x00000001)
#define UART_DMASA_DMA_SOFTWARE_ACKNOWLEDGE_POS          0

#define UART_DMASA_DMA_SOFTWARE_ACKNOWLEDGE_RST          0x0

__INLINE void uart_dmasa_pack(uint8_t dmasoftwareacknowledge)
{
    _PICO_REG_WR(UART_DMASA_OFFSET+ UART_BASE_ADDR,  ((uint32_t)dmasoftwareacknowledge << 0));
}

__INLINE void uart_dmasa_unpack(uint8_t* dmasoftwareacknowledge)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_DMASA_OFFSET + UART_BASE_ADDR);

    *dmasoftwareacknowledge = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE void uart_dmasa_dma_software_acknowledge_setf(uint8_t dmasoftwareacknowledge)
{
    _PICO_REG_WR(UART_DMASA_OFFSET+ UART_BASE_ADDR, (uint32_t)dmasoftwareacknowledge << 0);
}

 /**
 * @brief CPR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16            FIFO_MODE   0b0
 *     13            DMA_EXTRA   0
 *     12   UART_ADD_ENCODED_PARAMS   0
 *     11               SHADOW   0
 *     10            FIFO_STAT   0
 *     09          FIFO_ACCESS   0
 *     08      ADDITIONAL_FEAT   0
 *     07          SIR_LP_MODE   0
 *     06             SIR_MODE   0
 *     05            THRE_MODE   0
 *     04            AFCE_MODE   0
 *  01:00       APB_DATA_WIDTH   0b0
 * </pre>
 */
#define UART_CPR_OFFSET 0x000000F4


__INLINE uint32_t uart_cpr_get(void)
{
    return _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_CPR_FIFO_MODE_MASK                        ((uint32_t)0x00FF0000)
#define UART_CPR_FIFO_MODE_LSB                         16
#define UART_CPR_FIFO_MODE_WIDTH                       ((uint32_t)0x00000008)
#define UART_CPR_DMA_EXTRA_BIT                         ((uint32_t)0x00002000)
#define UART_CPR_DMA_EXTRA_POS                         13
#define UART_CPR_UART_ADD_ENCODED_PARAMS_BIT           ((uint32_t)0x00001000)
#define UART_CPR_UART_ADD_ENCODED_PARAMS_POS           12
#define UART_CPR_SHADOW_BIT                            ((uint32_t)0x00000800)
#define UART_CPR_SHADOW_POS                            11
#define UART_CPR_FIFO_STAT_BIT                         ((uint32_t)0x00000400)
#define UART_CPR_FIFO_STAT_POS                         10
#define UART_CPR_FIFO_ACCESS_BIT                       ((uint32_t)0x00000200)
#define UART_CPR_FIFO_ACCESS_POS                       9
#define UART_CPR_ADDITIONAL_FEAT_BIT                   ((uint32_t)0x00000100)
#define UART_CPR_ADDITIONAL_FEAT_POS                   8
#define UART_CPR_SIR_LP_MODE_BIT                       ((uint32_t)0x00000080)
#define UART_CPR_SIR_LP_MODE_POS                       7
#define UART_CPR_SIR_MODE_BIT                          ((uint32_t)0x00000040)
#define UART_CPR_SIR_MODE_POS                          6
#define UART_CPR_THRE_MODE_BIT                         ((uint32_t)0x00000020)
#define UART_CPR_THRE_MODE_POS                         5
#define UART_CPR_AFCE_MODE_BIT                         ((uint32_t)0x00000010)
#define UART_CPR_AFCE_MODE_POS                         4
#define UART_CPR_APB_DATA_WIDTH_MASK                   ((uint32_t)0x00000003)
#define UART_CPR_APB_DATA_WIDTH_LSB                    0
#define UART_CPR_APB_DATA_WIDTH_WIDTH                  ((uint32_t)0x00000002)

#define UART_CPR_FIFO_MODE_RST                         0x0
#define UART_CPR_DMA_EXTRA_RST                         0x0
#define UART_CPR_UART_ADD_ENCODED_PARAMS_RST           0x0
#define UART_CPR_SHADOW_RST                            0x0
#define UART_CPR_FIFO_STAT_RST                         0x0
#define UART_CPR_FIFO_ACCESS_RST                       0x0
#define UART_CPR_ADDITIONAL_FEAT_RST                   0x0
#define UART_CPR_SIR_LP_MODE_RST                       0x0
#define UART_CPR_SIR_MODE_RST                          0x0
#define UART_CPR_THRE_MODE_RST                         0x0
#define UART_CPR_AFCE_MODE_RST                         0x0
#define UART_CPR_APB_DATA_WIDTH_RST                    0x0

__INLINE void uart_cpr_unpack(uint8_t* fifomode, uint8_t* dmaextra, uint8_t* uartaddencodedparams, uint8_t* shadow, uint8_t* fifostat, uint8_t* fifoaccess, uint8_t* additionalfeat, uint8_t* sirlpmode, uint8_t* sirmode, uint8_t* thremode, uint8_t* afcemode, uint8_t* apbdatawidth)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);

    *fifomode = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *dmaextra = (localVal & ((uint32_t)0x00002000)) >> 13;
    *uartaddencodedparams = (localVal & ((uint32_t)0x00001000)) >> 12;
    *shadow = (localVal & ((uint32_t)0x00000800)) >> 11;
    *fifostat = (localVal & ((uint32_t)0x00000400)) >> 10;
    *fifoaccess = (localVal & ((uint32_t)0x00000200)) >> 9;
    *additionalfeat = (localVal & ((uint32_t)0x00000100)) >> 8;
    *sirlpmode = (localVal & ((uint32_t)0x00000080)) >> 7;
    *sirmode = (localVal & ((uint32_t)0x00000040)) >> 6;
    *thremode = (localVal & ((uint32_t)0x00000020)) >> 5;
    *afcemode = (localVal & ((uint32_t)0x00000010)) >> 4;
    *apbdatawidth = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t uart_cpr_fifo_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t uart_cpr_dma_extra_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t uart_cpr_uart_add_encoded_params_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t uart_cpr_shadow_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t uart_cpr_fifo_stat_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t uart_cpr_fifo_access_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t uart_cpr_additional_feat_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t uart_cpr_sir_lp_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t uart_cpr_sir_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t uart_cpr_thre_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t uart_cpr_afce_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t uart_cpr_apb_data_width_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CPR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

 /**
 * @brief UCV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00   UART_Component_Version   0b0
 * </pre>
 */
#define UART_UCV_OFFSET 0x000000F8


__INLINE uint32_t uart_ucv_get(void)
{
    return _PICO_REG_RD(UART_UCV_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_UCV_UART_COMPONENT_VERSION_MASK           ((uint32_t)0xFFFFFFFF)
#define UART_UCV_UART_COMPONENT_VERSION_LSB            0
#define UART_UCV_UART_COMPONENT_VERSION_WIDTH          ((uint32_t)0x00000020)

#define UART_UCV_UART_COMPONENT_VERSION_RST            0x0

__INLINE void uart_ucv_unpack(uint8_t* uartcomponentversion)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_UCV_OFFSET + UART_BASE_ADDR);

    *uartcomponentversion = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t uart_ucv_uart_component_version_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_UCV_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief CTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00        Peripheral_ID   0b44570110
 * </pre>
 */
#define UART_CTR_OFFSET 0x000000FC


__INLINE uint32_t uart_ctr_get(void)
{
    return _PICO_REG_RD(UART_CTR_OFFSET + UART_BASE_ADDR);
}

// field definitions
#define UART_CTR_PERIPHERAL_ID_MASK                    ((uint32_t)0xFFFFFFFF)
#define UART_CTR_PERIPHERAL_ID_LSB                     0
#define UART_CTR_PERIPHERAL_ID_WIDTH                   ((uint32_t)0x00000020)

#define UART_CTR_PERIPHERAL_ID_RST                     0x44570110

__INLINE void uart_ctr_unpack(uint8_t* peripheralid)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CTR_OFFSET + UART_BASE_ADDR);

    *peripheralid = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t uart_ctr_peripheral_id_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(UART_CTR_OFFSET + UART_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :24;
      __IO uint32_t receive_buffer:8;
    }RBR_fld;
    __IO uint32_t RBR;
  };


  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :24;
      __IO uint32_t transmit_holding:8;
    }THR_fld;
    __IO uint32_t THR;
  };


  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :24;
      __IO uint32_t divisor_latch_low:8;
    }DLL_fld;
    __IO uint32_t DLL;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :24;
      __IO uint32_t divisor_latch_high:8;
    }DLH_fld;
    __IO uint32_t DLH;
  };


  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :24;
      __IO uint32_t ptime:1;
      __IO uint32_t :3;
      __IO uint32_t edssi:1;
      __IO uint32_t elsi:1;
      __IO uint32_t etbei:1;
      __IO uint32_t erbfi:1;
    }IER_fld;
    __IO uint32_t IER;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :24;
      __IO uint32_t fifose:2;
      __IO uint32_t :2;
      __IO uint32_t iid:4;
    }IIR_fld;
    __IO uint32_t IIR;
  };


  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :24;
      __IO uint32_t rt:2;
      __IO uint32_t tet:2;
      __IO uint32_t dmam:1;
      __IO uint32_t xfifor:1;
      __IO uint32_t rfifor:1;
      __IO uint32_t fifoe:1;
    }FCR_fld;
    __IO uint32_t FCR;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :24;
      __IO uint32_t dlab:1;
      __IO uint32_t break_con:1;
      __IO uint32_t :1;
      __IO uint32_t eps:1;
      __IO uint32_t pen:1;
      __IO uint32_t stop:1;
      __IO uint32_t dls:2;
    }LCR_fld;
    __IO uint32_t LCR;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :25;
      __IO uint32_t sire:1;
      __IO uint32_t afce:1;
      __IO uint32_t loop_back:1;
      __IO uint32_t out2:1;
      __IO uint32_t out1:1;
      __IO uint32_t rts:1;
      __IO uint32_t dtr:1;
    }MCR_fld;
    __IO uint32_t MCR;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :24;
      __IO uint32_t rfe:1;
      __IO uint32_t temt:1;
      __IO uint32_t thre:1;
      __IO uint32_t bi:1;
      __IO uint32_t fe:1;
      __IO uint32_t pe:1;
      __IO uint32_t oe:1;
      __IO uint32_t dr:1;
    }LSR_fld;
    __IO uint32_t LSR;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :24;
      __IO uint32_t dcd:1;
      __IO uint32_t ri:1;
      __IO uint32_t dsr:1;
      __IO uint32_t cts:1;
      __IO uint32_t ddcd:1;
      __IO uint32_t teri:1;
      __IO uint32_t ddsr:1;
      __IO uint32_t dcts:1;
    }MSR_fld;
    __IO uint32_t MSR;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :24;
      __IO uint32_t scratchpad:8;
    }SCR_fld;
    __IO uint32_t SCR;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t :24;
      __IO uint32_t lpdll:8;
    }LPDLL_fld;
    __IO uint32_t LPDLL;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :24;
      __IO uint32_t lpdlh:8;
    }LPDLH_fld;
    __IO uint32_t LPDLH;
  };


  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :24;
      __IO uint32_t shadow_receive_buffer:8;
    }SRBR_fld;
    __IO uint32_t SRBR;
  };


  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :24;
      __IO uint32_t shadow_transmit_holding:8;
    }STHR_fld;
    __IO uint32_t STHR;
  };


  union{ //offset addr 0x0070
    struct{
      __IO uint32_t :31;
      __IO uint32_t fifo_access:1;
    }FAR_fld;
    __IO uint32_t FAR;
  };

  union{ //offset addr 0x0074
    struct{
      __IO uint32_t :24;
      __IO uint32_t transmit_fifo_read:8;
    }TFR_fld;
    __IO uint32_t TFR;
  };

  union{ //offset addr 0x0078
    struct{
      __IO uint32_t :22;
      __IO uint32_t rffe:1;
      __IO uint32_t rfpe:1;
      __IO uint32_t rfwd:8;
    }RFW_fld;
    __IO uint32_t RFW;
  };

  union{ //offset addr 0x007c
    struct{
      __IO uint32_t :27;
      __IO uint32_t rff:1;
      __IO uint32_t rfne:1;
      __IO uint32_t tfe:1;
      __IO uint32_t tfnf:1;
      __IO uint32_t busy:1;
    }USR_fld;
    __IO uint32_t USR;
  };

  union{ //offset addr 0x0080
    struct{
      __IO uint32_t :27;
      __IO uint32_t transmit_fifo_level:5;
    }TFL_fld;
    __IO uint32_t TFL;
  };

  union{ //offset addr 0x0084
    struct{
      __IO uint32_t :27;
      __IO uint32_t receive_fifo_level:5;
    }RFL_fld;
    __IO uint32_t RFL;
  };

  union{ //offset addr 0x0088
    struct{
      __IO uint32_t :29;
      __IO uint32_t xfr:1;
      __IO uint32_t rfr:1;
      __IO uint32_t ur:1;
    }SRR_fld;
    __IO uint32_t SRR;
  };

  union{ //offset addr 0x008c
    struct{
      __IO uint32_t :31;
      __IO uint32_t shadow_request_to_send:1;
    }SRTS_fld;
    __IO uint32_t SRTS;
  };

  union{ //offset addr 0x0090
    struct{
      __IO uint32_t :31;
      __IO uint32_t shadow_break_control:1;
    }SBCR_fld;
    __IO uint32_t SBCR;
  };

  union{ //offset addr 0x0094
    struct{
      __IO uint32_t :31;
      __IO uint32_t shadow_dma_mode:1;
    }SDMAM_fld;
    __IO uint32_t SDMAM;
  };

  union{ //offset addr 0x0098
    struct{
      __IO uint32_t :31;
      __IO uint32_t shadow_fifo_enable:1;
    }SFE_fld;
    __IO uint32_t SFE;
  };

  union{ //offset addr 0x009c
    struct{
      __IO uint32_t :30;
      __IO uint32_t shadow_rcvr_trigger:2;
    }SRT_fld;
    __IO uint32_t SRT;
  };

  union{ //offset addr 0x00a0
    struct{
      __IO uint32_t :30;
      __IO uint32_t shadow_tx_empty_trigger:2;
    }STET_fld;
    __IO uint32_t STET;
  };

  union{ //offset addr 0x00a4
    struct{
      __IO uint32_t :31;
      __IO uint32_t halt_tx:1;
    }HTX_fld;
    __IO uint32_t HTX;
  };

  union{ //offset addr 0x00a8
    struct{
      __IO uint32_t :31;
      __IO uint32_t dma_software_acknowledge:1;
    }DMASA_fld;
    __IO uint32_t DMASA;
  };


  union{ //offset addr 0x00f4
    struct{
      __IO uint32_t :8;
      __IO uint32_t fifo_mode:8;
      __IO uint32_t :2;
      __IO uint32_t dma_extra:1;
      __IO uint32_t uart_add_encoded_params:1;
      __IO uint32_t shadow:1;
      __IO uint32_t fifo_stat:1;
      __IO uint32_t fifo_access:1;
      __IO uint32_t additional_feat:1;
      __IO uint32_t sir_lp_mode:1;
      __IO uint32_t sir_mode:1;
      __IO uint32_t thre_mode:1;
      __IO uint32_t afce_mode:1;
      __IO uint32_t :2;
      __IO uint32_t apb_data_width:2;
    }CPR_fld;
    __IO uint32_t CPR;
  };

  union{ //offset addr 0x00f8
    struct{
      __IO uint32_t uart_component_version:32;
    }UCV_fld;
    __IO uint32_t UCV;
  };

  union{ //offset addr 0x00fc
    struct{
      __IO uint32_t peripheral_id:32;
    }CTR_fld;
    __IO uint32_t CTR;
  };

} PICO_REG_UART_TypeDef;

#define PICO_REG_UART PICO_REG_UART_TypeDef *0x40004000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_UART_H_

