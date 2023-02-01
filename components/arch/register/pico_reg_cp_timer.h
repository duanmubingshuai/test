#ifndef _PICO_REG_CP_TIMER_H_
#define _PICO_REG_CP_TIMER_H_

#include <stdint.h>

#define CP_TIMER_COUNT 24

#define CP_TIMER_BASE_ADDR 0x40001000

#define CP_TIMER_SIZE 0x000000AC


 /**
 * @brief TIMER1_LOAD_COUNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00    Timer1_Load_Count   0b0
 * </pre>
 */
#define CP_TIMER_TIMER1_LOAD_COUNT_OFFSET 0x00000000


__INLINE uint32_t cp_timer_timer1loadcount_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER1_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer1loadcount_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER1LOADCOUNT_TIMER1_LOAD_COUNT_MASK                ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER1LOADCOUNT_TIMER1_LOAD_COUNT_LSB                 0
#define CP_TIMER_TIMER1LOADCOUNT_TIMER1_LOAD_COUNT_WIDTH               ((uint32_t)0x00000018)

#define CP_TIMER_TIMER1LOADCOUNT_TIMER1_LOAD_COUNT_RST                 0x0

__INLINE void cp_timer_timer1loadcount_pack(uint32_t timer1loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timer1loadcount << 0));
}

__INLINE void cp_timer_timer1loadcount_unpack(uint8_t* timer1loadcount)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);

    *timer1loadcount = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer1loadcount_timer1_load_count_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

__INLINE void cp_timer_timer1loadcount_timer1_load_count_setf(uint32_t timer1loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, (uint32_t)timer1loadcount << 0);
}

 /**
 * @brief TIMER1_CURRENT_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00   Timer1_Current_Value   0b0
 * </pre>
 */
#define CP_TIMER_TIMER1_CURRENT_VALUE_OFFSET 0x00000004


__INLINE uint32_t cp_timer_timer1currentvalue_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER1_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER1CURRENTVALUE_TIMER1_CURRENT_VALUE_MASK             ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER1CURRENTVALUE_TIMER1_CURRENT_VALUE_LSB              0
#define CP_TIMER_TIMER1CURRENTVALUE_TIMER1_CURRENT_VALUE_WIDTH            ((uint32_t)0x00000018)

#define CP_TIMER_TIMER1CURRENTVALUE_TIMER1_CURRENT_VALUE_RST              0x0

__INLINE void cp_timer_timer1currentvalue_unpack(uint8_t* timer1currentvalue)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);

    *timer1currentvalue = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer1currentvalue_timer1_current_value_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

 /**
 * @brief TIMER1_CONTROL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02   Timer_Interrupt_Mask   0
 *     01           Timer_Mode   0
 *     00         Timer_Enable   0
 * </pre>
 */
#define CP_TIMER_TIMER1_CONTROL_OFFSET 0x00000008


__INLINE uint32_t cp_timer_timer1controlreg_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer1controlreg_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER1CONTROLREG_TIMER_INTERRUPT_MASK_BIT              ((uint32_t)0x00000004)
#define CP_TIMER_TIMER1CONTROLREG_TIMER_INTERRUPT_MASK_POS              2
#define CP_TIMER_TIMER1CONTROLREG_TIMER_MODE_BIT                        ((uint32_t)0x00000002)
#define CP_TIMER_TIMER1CONTROLREG_TIMER_MODE_POS                        1
#define CP_TIMER_TIMER1CONTROLREG_TIMER_ENABLE_BIT                      ((uint32_t)0x00000001)
#define CP_TIMER_TIMER1CONTROLREG_TIMER_ENABLE_POS                      0

#define CP_TIMER_TIMER1CONTROLREG_TIMER_INTERRUPT_MASK_RST              0x0
#define CP_TIMER_TIMER1CONTROLREG_TIMER_MODE_RST                        0x0
#define CP_TIMER_TIMER1CONTROLREG_TIMER_ENABLE_RST                      0x0

__INLINE void cp_timer_timer1controlreg_pack(uint8_t timerinterruptmask, uint8_t timermode, uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timerinterruptmask << 2) | ((uint32_t)timermode << 1) | ((uint32_t)timerenable << 0));
}

__INLINE void cp_timer_timer1controlreg_unpack(uint8_t* timerinterruptmask, uint8_t* timermode, uint8_t* timerenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);

    *timerinterruptmask = (localVal & ((uint32_t)0x00000004)) >> 2;
    *timermode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *timerenable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer1controlreg_timer_interrupt_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void cp_timer_timer1controlreg_timer_interrupt_mask_setf(uint8_t timerinterruptmask)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)timerinterruptmask << 2));
}

__INLINE uint8_t cp_timer_timer1controlreg_timer_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void cp_timer_timer1controlreg_timer_mode_setf(uint8_t timermode)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)timermode << 1));
}

__INLINE uint8_t cp_timer_timer1controlreg_timer_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void cp_timer_timer1controlreg_timer_enable_setf(uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER1_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER1_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)timerenable << 0));
}

 /**
 * @brief TIMER1EOI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer1_End_of_Interrupt   0
 * </pre>
 */
#define CP_TIMER_TIMER1EOI_OFFSET 0x0000000C


__INLINE uint32_t cp_timer_timer1eoi_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER1EOI_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER1EOI_TIMER1_END_OF_INTERRUPT_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER1EOI_TIMER1_END_OF_INTERRUPT_POS           0

#define CP_TIMER_TIMER1EOI_TIMER1_END_OF_INTERRUPT_RST           0x0

__INLINE void cp_timer_timer1eoi_unpack(uint8_t* timer1endofinterrupt)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1EOI_OFFSET + CP_TIMER_BASE_ADDR);

    *timer1endofinterrupt = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer1eoi_timer1_end_of_interrupt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1EOI_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMER1_INT_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer1_Interrupt_Status   0
 * </pre>
 */
#define CP_TIMER_TIMER1_INT_STATUS_OFFSET 0x00000010


__INLINE uint32_t cp_timer_timer1intstatus_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER1_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER1INTSTATUS_TIMER1_INTERRUPT_STATUS_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER1INTSTATUS_TIMER1_INTERRUPT_STATUS_POS           0

#define CP_TIMER_TIMER1INTSTATUS_TIMER1_INTERRUPT_STATUS_RST           0x0

__INLINE void cp_timer_timer1intstatus_unpack(uint8_t* timer1interruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);

    *timer1interruptstatus = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer1intstatus_timer1_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER1_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMER2_LOAD_COUNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00    Timer2_Load_Count   0b0
 * </pre>
 */
#define CP_TIMER_TIMER2_LOAD_COUNT_OFFSET 0x00000014


__INLINE uint32_t cp_timer_timer2loadcount_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER2_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer2loadcount_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER2LOADCOUNT_TIMER2_LOAD_COUNT_MASK                ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER2LOADCOUNT_TIMER2_LOAD_COUNT_LSB                 0
#define CP_TIMER_TIMER2LOADCOUNT_TIMER2_LOAD_COUNT_WIDTH               ((uint32_t)0x00000018)

#define CP_TIMER_TIMER2LOADCOUNT_TIMER2_LOAD_COUNT_RST                 0x0

__INLINE void cp_timer_timer2loadcount_pack(uint32_t timer2loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timer2loadcount << 0));
}

__INLINE void cp_timer_timer2loadcount_unpack(uint8_t* timer2loadcount)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);

    *timer2loadcount = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer2loadcount_timer2_load_count_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

__INLINE void cp_timer_timer2loadcount_timer2_load_count_setf(uint32_t timer2loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, (uint32_t)timer2loadcount << 0);
}

 /**
 * @brief TIMER2_CURRENT_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00   Timer2_Current_Value   0b0
 * </pre>
 */
#define CP_TIMER_TIMER2_CURRENT_VALUE_OFFSET 0x00000018


__INLINE uint32_t cp_timer_timer2currentvalue_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER2_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER2CURRENTVALUE_TIMER2_CURRENT_VALUE_MASK             ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER2CURRENTVALUE_TIMER2_CURRENT_VALUE_LSB              0
#define CP_TIMER_TIMER2CURRENTVALUE_TIMER2_CURRENT_VALUE_WIDTH            ((uint32_t)0x00000018)

#define CP_TIMER_TIMER2CURRENTVALUE_TIMER2_CURRENT_VALUE_RST              0x0

__INLINE void cp_timer_timer2currentvalue_unpack(uint8_t* timer2currentvalue)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);

    *timer2currentvalue = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer2currentvalue_timer2_current_value_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

 /**
 * @brief TIMER2_CONTROL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02   Timer_Interrupt_Mask   0
 *     01           Timer_Mode   0
 *     00         Timer_Enable   0
 * </pre>
 */
#define CP_TIMER_TIMER2_CONTROL_OFFSET 0x0000001C


__INLINE uint32_t cp_timer_timer2controlreg_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer2controlreg_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER2CONTROLREG_TIMER_INTERRUPT_MASK_BIT              ((uint32_t)0x00000004)
#define CP_TIMER_TIMER2CONTROLREG_TIMER_INTERRUPT_MASK_POS              2
#define CP_TIMER_TIMER2CONTROLREG_TIMER_MODE_BIT                        ((uint32_t)0x00000002)
#define CP_TIMER_TIMER2CONTROLREG_TIMER_MODE_POS                        1
#define CP_TIMER_TIMER2CONTROLREG_TIMER_ENABLE_BIT                      ((uint32_t)0x00000001)
#define CP_TIMER_TIMER2CONTROLREG_TIMER_ENABLE_POS                      0

#define CP_TIMER_TIMER2CONTROLREG_TIMER_INTERRUPT_MASK_RST              0x0
#define CP_TIMER_TIMER2CONTROLREG_TIMER_MODE_RST                        0x0
#define CP_TIMER_TIMER2CONTROLREG_TIMER_ENABLE_RST                      0x0

__INLINE void cp_timer_timer2controlreg_pack(uint8_t timerinterruptmask, uint8_t timermode, uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timerinterruptmask << 2) | ((uint32_t)timermode << 1) | ((uint32_t)timerenable << 0));
}

__INLINE void cp_timer_timer2controlreg_unpack(uint8_t* timerinterruptmask, uint8_t* timermode, uint8_t* timerenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);

    *timerinterruptmask = (localVal & ((uint32_t)0x00000004)) >> 2;
    *timermode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *timerenable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer2controlreg_timer_interrupt_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void cp_timer_timer2controlreg_timer_interrupt_mask_setf(uint8_t timerinterruptmask)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)timerinterruptmask << 2));
}

__INLINE uint8_t cp_timer_timer2controlreg_timer_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void cp_timer_timer2controlreg_timer_mode_setf(uint8_t timermode)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)timermode << 1));
}

__INLINE uint8_t cp_timer_timer2controlreg_timer_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void cp_timer_timer2controlreg_timer_enable_setf(uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER2_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER2_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)timerenable << 0));
}

 /**
 * @brief TIMER2EOI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer2_End_of_Interrupt   0
 * </pre>
 */
#define CP_TIMER_TIMER2EOI_OFFSET 0x00000020


__INLINE uint32_t cp_timer_timer2eoi_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER2EOI_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER2EOI_TIMER2_END_OF_INTERRUPT_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER2EOI_TIMER2_END_OF_INTERRUPT_POS           0

#define CP_TIMER_TIMER2EOI_TIMER2_END_OF_INTERRUPT_RST           0x0

__INLINE void cp_timer_timer2eoi_unpack(uint8_t* timer2endofinterrupt)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2EOI_OFFSET + CP_TIMER_BASE_ADDR);

    *timer2endofinterrupt = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer2eoi_timer2_end_of_interrupt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2EOI_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMER2_INT_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer2_Interrupt_Status   0
 * </pre>
 */
#define CP_TIMER_TIMER2_INT_STATUS_OFFSET 0x00000024


__INLINE uint32_t cp_timer_timer2intstatus_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER2_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER2INTSTATUS_TIMER2_INTERRUPT_STATUS_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER2INTSTATUS_TIMER2_INTERRUPT_STATUS_POS           0

#define CP_TIMER_TIMER2INTSTATUS_TIMER2_INTERRUPT_STATUS_RST           0x0

__INLINE void cp_timer_timer2intstatus_unpack(uint8_t* timer2interruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);

    *timer2interruptstatus = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer2intstatus_timer2_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER2_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMER3_LOAD_COUNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00    Timer3_Load_Count   0b0
 * </pre>
 */
#define CP_TIMER_TIMER3_LOAD_COUNT_OFFSET 0x00000028


__INLINE uint32_t cp_timer_timer3loadcount_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER3_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer3loadcount_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER3LOADCOUNT_TIMER3_LOAD_COUNT_MASK                ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER3LOADCOUNT_TIMER3_LOAD_COUNT_LSB                 0
#define CP_TIMER_TIMER3LOADCOUNT_TIMER3_LOAD_COUNT_WIDTH               ((uint32_t)0x00000018)

#define CP_TIMER_TIMER3LOADCOUNT_TIMER3_LOAD_COUNT_RST                 0x0

__INLINE void cp_timer_timer3loadcount_pack(uint32_t timer3loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timer3loadcount << 0));
}

__INLINE void cp_timer_timer3loadcount_unpack(uint8_t* timer3loadcount)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);

    *timer3loadcount = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer3loadcount_timer3_load_count_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

__INLINE void cp_timer_timer3loadcount_timer3_load_count_setf(uint32_t timer3loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, (uint32_t)timer3loadcount << 0);
}

 /**
 * @brief TIMER3_CURRENT_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00   Timer3_Current_Value   0b0
 * </pre>
 */
#define CP_TIMER_TIMER3_CURRENT_VALUE_OFFSET 0x0000002C


__INLINE uint32_t cp_timer_timer3currentvalue_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER3_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER3CURRENTVALUE_TIMER3_CURRENT_VALUE_MASK             ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER3CURRENTVALUE_TIMER3_CURRENT_VALUE_LSB              0
#define CP_TIMER_TIMER3CURRENTVALUE_TIMER3_CURRENT_VALUE_WIDTH            ((uint32_t)0x00000018)

#define CP_TIMER_TIMER3CURRENTVALUE_TIMER3_CURRENT_VALUE_RST              0x0

__INLINE void cp_timer_timer3currentvalue_unpack(uint8_t* timer3currentvalue)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);

    *timer3currentvalue = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer3currentvalue_timer3_current_value_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

 /**
 * @brief TIMER3_CONTROL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02   Timer_Interrupt_Mask   0
 *     01           Timer_Mode   0
 *     00         Timer_Enable   0
 * </pre>
 */
#define CP_TIMER_TIMER3_CONTROL_OFFSET 0x00000030


__INLINE uint32_t cp_timer_timer3controlreg_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer3controlreg_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER3CONTROLREG_TIMER_INTERRUPT_MASK_BIT              ((uint32_t)0x00000004)
#define CP_TIMER_TIMER3CONTROLREG_TIMER_INTERRUPT_MASK_POS              2
#define CP_TIMER_TIMER3CONTROLREG_TIMER_MODE_BIT                        ((uint32_t)0x00000002)
#define CP_TIMER_TIMER3CONTROLREG_TIMER_MODE_POS                        1
#define CP_TIMER_TIMER3CONTROLREG_TIMER_ENABLE_BIT                      ((uint32_t)0x00000001)
#define CP_TIMER_TIMER3CONTROLREG_TIMER_ENABLE_POS                      0

#define CP_TIMER_TIMER3CONTROLREG_TIMER_INTERRUPT_MASK_RST              0x0
#define CP_TIMER_TIMER3CONTROLREG_TIMER_MODE_RST                        0x0
#define CP_TIMER_TIMER3CONTROLREG_TIMER_ENABLE_RST                      0x0

__INLINE void cp_timer_timer3controlreg_pack(uint8_t timerinterruptmask, uint8_t timermode, uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timerinterruptmask << 2) | ((uint32_t)timermode << 1) | ((uint32_t)timerenable << 0));
}

__INLINE void cp_timer_timer3controlreg_unpack(uint8_t* timerinterruptmask, uint8_t* timermode, uint8_t* timerenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);

    *timerinterruptmask = (localVal & ((uint32_t)0x00000004)) >> 2;
    *timermode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *timerenable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer3controlreg_timer_interrupt_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void cp_timer_timer3controlreg_timer_interrupt_mask_setf(uint8_t timerinterruptmask)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)timerinterruptmask << 2));
}

__INLINE uint8_t cp_timer_timer3controlreg_timer_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void cp_timer_timer3controlreg_timer_mode_setf(uint8_t timermode)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)timermode << 1));
}

__INLINE uint8_t cp_timer_timer3controlreg_timer_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void cp_timer_timer3controlreg_timer_enable_setf(uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER3_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER3_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)timerenable << 0));
}

 /**
 * @brief TIMER3EOI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer3_End_of_Interrupt   0
 * </pre>
 */
#define CP_TIMER_TIMER3EOI_OFFSET 0x00000034


__INLINE uint32_t cp_timer_timer3eoi_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER3EOI_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER3EOI_TIMER3_END_OF_INTERRUPT_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER3EOI_TIMER3_END_OF_INTERRUPT_POS           0

#define CP_TIMER_TIMER3EOI_TIMER3_END_OF_INTERRUPT_RST           0x0

__INLINE void cp_timer_timer3eoi_unpack(uint8_t* timer3endofinterrupt)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3EOI_OFFSET + CP_TIMER_BASE_ADDR);

    *timer3endofinterrupt = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer3eoi_timer3_end_of_interrupt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3EOI_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMER3_INT_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer3_Interrupt_Status   0
 * </pre>
 */
#define CP_TIMER_TIMER3_INT_STATUS_OFFSET 0x00000038


__INLINE uint32_t cp_timer_timer3intstatus_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER3_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER3INTSTATUS_TIMER3_INTERRUPT_STATUS_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER3INTSTATUS_TIMER3_INTERRUPT_STATUS_POS           0

#define CP_TIMER_TIMER3INTSTATUS_TIMER3_INTERRUPT_STATUS_RST           0x0

__INLINE void cp_timer_timer3intstatus_unpack(uint8_t* timer3interruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);

    *timer3interruptstatus = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer3intstatus_timer3_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER3_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMER4_LOAD_COUNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00    Timer4_Load_Count   0b0
 * </pre>
 */
#define CP_TIMER_TIMER4_LOAD_COUNT_OFFSET 0x0000003C


__INLINE uint32_t cp_timer_timer4loadcount_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER4_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer4loadcount_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER4LOADCOUNT_TIMER4_LOAD_COUNT_MASK                ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER4LOADCOUNT_TIMER4_LOAD_COUNT_LSB                 0
#define CP_TIMER_TIMER4LOADCOUNT_TIMER4_LOAD_COUNT_WIDTH               ((uint32_t)0x00000018)

#define CP_TIMER_TIMER4LOADCOUNT_TIMER4_LOAD_COUNT_RST                 0x0

__INLINE void cp_timer_timer4loadcount_pack(uint32_t timer4loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timer4loadcount << 0));
}

__INLINE void cp_timer_timer4loadcount_unpack(uint8_t* timer4loadcount)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);

    *timer4loadcount = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer4loadcount_timer4_load_count_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_LOAD_COUNT_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

__INLINE void cp_timer_timer4loadcount_timer4_load_count_setf(uint32_t timer4loadcount)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_LOAD_COUNT_OFFSET+ CP_TIMER_BASE_ADDR, (uint32_t)timer4loadcount << 0);
}

 /**
 * @brief TIMER4_CURRENT_VALUE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:00   Timer4_Current_Value   0b0
 * </pre>
 */
#define CP_TIMER_TIMER4_CURRENT_VALUE_OFFSET 0x00000040


__INLINE uint32_t cp_timer_timer4currentvalue_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER4_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER4CURRENTVALUE_TIMER4_CURRENT_VALUE_MASK             ((uint32_t)0x00FFFFFF)
#define CP_TIMER_TIMER4CURRENTVALUE_TIMER4_CURRENT_VALUE_LSB              0
#define CP_TIMER_TIMER4CURRENTVALUE_TIMER4_CURRENT_VALUE_WIDTH            ((uint32_t)0x00000018)

#define CP_TIMER_TIMER4CURRENTVALUE_TIMER4_CURRENT_VALUE_RST              0x0

__INLINE void cp_timer_timer4currentvalue_unpack(uint8_t* timer4currentvalue)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);

    *timer4currentvalue = (localVal & ((uint32_t)0x00FFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timer4currentvalue_timer4_current_value_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_CURRENT_VALUE_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FFFFFF)) >> 0);
}

 /**
 * @brief TIMER4_CONTROL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02   Timer_Interrupt_Mask   0
 *     01           Timer_Mode   0
 *     00         Timer_Enable   0
 * </pre>
 */
#define CP_TIMER_TIMER4_CONTROL_OFFSET 0x00000044


__INLINE uint32_t cp_timer_timer4controlreg_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
}

__INLINE void cp_timer_timer4controlreg_set(uint32_t value)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, value);
}

// field definitions
#define CP_TIMER_TIMER4CONTROLREG_TIMER_INTERRUPT_MASK_BIT              ((uint32_t)0x00000004)
#define CP_TIMER_TIMER4CONTROLREG_TIMER_INTERRUPT_MASK_POS              2
#define CP_TIMER_TIMER4CONTROLREG_TIMER_MODE_BIT                        ((uint32_t)0x00000002)
#define CP_TIMER_TIMER4CONTROLREG_TIMER_MODE_POS                        1
#define CP_TIMER_TIMER4CONTROLREG_TIMER_ENABLE_BIT                      ((uint32_t)0x00000001)
#define CP_TIMER_TIMER4CONTROLREG_TIMER_ENABLE_POS                      0

#define CP_TIMER_TIMER4CONTROLREG_TIMER_INTERRUPT_MASK_RST              0x0
#define CP_TIMER_TIMER4CONTROLREG_TIMER_MODE_RST                        0x0
#define CP_TIMER_TIMER4CONTROLREG_TIMER_ENABLE_RST                      0x0

__INLINE void cp_timer_timer4controlreg_pack(uint8_t timerinterruptmask, uint8_t timermode, uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR,  ((uint32_t)timerinterruptmask << 2) | ((uint32_t)timermode << 1) | ((uint32_t)timerenable << 0));
}

__INLINE void cp_timer_timer4controlreg_unpack(uint8_t* timerinterruptmask, uint8_t* timermode, uint8_t* timerenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);

    *timerinterruptmask = (localVal & ((uint32_t)0x00000004)) >> 2;
    *timermode = (localVal & ((uint32_t)0x00000002)) >> 1;
    *timerenable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer4controlreg_timer_interrupt_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void cp_timer_timer4controlreg_timer_interrupt_mask_setf(uint8_t timerinterruptmask)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)timerinterruptmask << 2));
}

__INLINE uint8_t cp_timer_timer4controlreg_timer_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void cp_timer_timer4controlreg_timer_mode_setf(uint8_t timermode)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)timermode << 1));
}

__INLINE uint8_t cp_timer_timer4controlreg_timer_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void cp_timer_timer4controlreg_timer_enable_setf(uint8_t timerenable)
{
    _PICO_REG_WR(CP_TIMER_TIMER4_CONTROL_OFFSET+ CP_TIMER_BASE_ADDR, (_PICO_REG_RD(CP_TIMER_TIMER4_CONTROL_OFFSET + CP_TIMER_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)timerenable << 0));
}

 /**
 * @brief TIMER4EOI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer4_End_of_Interrupt   0
 * </pre>
 */
#define CP_TIMER_TIMER4EOI_OFFSET 0x00000048


__INLINE uint32_t cp_timer_timer4eoi_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER4EOI_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER4EOI_TIMER4_END_OF_INTERRUPT_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER4EOI_TIMER4_END_OF_INTERRUPT_POS           0

#define CP_TIMER_TIMER4EOI_TIMER4_END_OF_INTERRUPT_RST           0x0

__INLINE void cp_timer_timer4eoi_unpack(uint8_t* timer4endofinterrupt)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4EOI_OFFSET + CP_TIMER_BASE_ADDR);

    *timer4endofinterrupt = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer4eoi_timer4_end_of_interrupt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4EOI_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMER4_INT_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Timer4_Interrupt_Status   0
 * </pre>
 */
#define CP_TIMER_TIMER4_INT_STATUS_OFFSET 0x0000004C


__INLINE uint32_t cp_timer_timer4intstatus_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMER4_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMER4INTSTATUS_TIMER4_INTERRUPT_STATUS_BIT           ((uint32_t)0x00000001)
#define CP_TIMER_TIMER4INTSTATUS_TIMER4_INTERRUPT_STATUS_POS           0

#define CP_TIMER_TIMER4INTSTATUS_TIMER4_INTERRUPT_STATUS_RST           0x0

__INLINE void cp_timer_timer4intstatus_unpack(uint8_t* timer4interruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);

    *timer4interruptstatus = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cp_timer_timer4intstatus_timer4_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMER4_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief TIMERS_INT_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00   Timers_Interrupt_Status   0b0
 * </pre>
 */
#define CP_TIMER_TIMERS_INT_STATUS_OFFSET 0x000000A0


__INLINE uint32_t cp_timer_timersintstatus_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMERS_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMERSINTSTATUS_TIMERS_INTERRUPT_STATUS_MASK          ((uint32_t)0x0000000F)
#define CP_TIMER_TIMERSINTSTATUS_TIMERS_INTERRUPT_STATUS_LSB           0
#define CP_TIMER_TIMERSINTSTATUS_TIMERS_INTERRUPT_STATUS_WIDTH         ((uint32_t)0x00000004)

#define CP_TIMER_TIMERSINTSTATUS_TIMERS_INTERRUPT_STATUS_RST           0x0

__INLINE void cp_timer_timersintstatus_unpack(uint8_t* timersinterruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);

    *timersinterruptstatus = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t cp_timer_timersintstatus_timers_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_INT_STATUS_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief TIMERS_EOI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00   Timers_End_of_Interrupt   0b0
 * </pre>
 */
#define CP_TIMER_TIMERS_EOI_OFFSET 0x000000A4


__INLINE uint32_t cp_timer_timerseoi_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMERS_EOI_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMERSEOI_TIMERS_END_OF_INTERRUPT_MASK          ((uint32_t)0x0000000F)
#define CP_TIMER_TIMERSEOI_TIMERS_END_OF_INTERRUPT_LSB           0
#define CP_TIMER_TIMERSEOI_TIMERS_END_OF_INTERRUPT_WIDTH         ((uint32_t)0x00000004)

#define CP_TIMER_TIMERSEOI_TIMERS_END_OF_INTERRUPT_RST           0x0

__INLINE void cp_timer_timerseoi_unpack(uint8_t* timersendofinterrupt)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_EOI_OFFSET + CP_TIMER_BASE_ADDR);

    *timersendofinterrupt = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t cp_timer_timerseoi_timers_end_of_interrupt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_EOI_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief TIMERS_RAW_INT_STATUS_0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00   Timers_Raw_Interrupt_Status   0b0
 * </pre>
 */
#define CP_TIMER_TIMERS_RAW_INT_STATUS_0_OFFSET 0x000000A8


__INLINE uint32_t cp_timer_timersrawintstatus_0_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMERS_RAW_INT_STATUS_0_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMERSRAWINTSTATUS_0_TIMERS_RAW_INTERRUPT_STATUS_MASK      ((uint32_t)0x0000000F)
#define CP_TIMER_TIMERSRAWINTSTATUS_0_TIMERS_RAW_INTERRUPT_STATUS_LSB       0
#define CP_TIMER_TIMERSRAWINTSTATUS_0_TIMERS_RAW_INTERRUPT_STATUS_WIDTH     ((uint32_t)0x00000004)

#define CP_TIMER_TIMERSRAWINTSTATUS_0_TIMERS_RAW_INTERRUPT_STATUS_RST       0x0

__INLINE void cp_timer_timersrawintstatus_0_unpack(uint8_t* timersrawinterruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_RAW_INT_STATUS_0_OFFSET + CP_TIMER_BASE_ADDR);

    *timersrawinterruptstatus = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t cp_timer_timersrawintstatus_0_timers_raw_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_RAW_INT_STATUS_0_OFFSET + CP_TIMER_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief TIMERS_RAW_INT_STATUS_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00   Timers_Component_Version   0b0
 * </pre>
 */
#define CP_TIMER_TIMERS_RAW_INT_STATUS_1_OFFSET 0x000000AC


__INLINE uint32_t cp_timer_timersrawintstatus_1_get(void)
{
    return _PICO_REG_RD(CP_TIMER_TIMERS_RAW_INT_STATUS_1_OFFSET + CP_TIMER_BASE_ADDR);
}

// field definitions
#define CP_TIMER_TIMERSRAWINTSTATUS_1_TIMERS_COMPONENT_VERSION_MASK         ((uint32_t)0xFFFFFFFF)
#define CP_TIMER_TIMERSRAWINTSTATUS_1_TIMERS_COMPONENT_VERSION_LSB          0
#define CP_TIMER_TIMERSRAWINTSTATUS_1_TIMERS_COMPONENT_VERSION_WIDTH        ((uint32_t)0x00000020)

#define CP_TIMER_TIMERSRAWINTSTATUS_1_TIMERS_COMPONENT_VERSION_RST          0x0

__INLINE void cp_timer_timersrawintstatus_1_unpack(uint8_t* timerscomponentversion)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_RAW_INT_STATUS_1_OFFSET + CP_TIMER_BASE_ADDR);

    *timerscomponentversion = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t cp_timer_timersrawintstatus_1_timers_component_version_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CP_TIMER_TIMERS_RAW_INT_STATUS_1_OFFSET + CP_TIMER_BASE_ADDR);
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
      __IO uint32_t :8;
      __IO uint32_t timer1_load_count:24;
    }Timer1LoadCount_fld;
    __IO uint32_t Timer1LoadCount;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :8;
      __IO uint32_t timer1_current_value:24;
    }Timer1CurrentValue_fld;
    __IO uint32_t Timer1CurrentValue;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :29;
      __IO uint32_t timer_interrupt_mask:1;
      __IO uint32_t timer_mode:1;
      __IO uint32_t timer_enable:1;
    }Timer1ControlReg_fld;
    __IO uint32_t Timer1ControlReg;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer1_end_of_interrupt:1;
    }Timer1EOI_fld;
    __IO uint32_t Timer1EOI;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer1_interrupt_status:1;
    }Timer1IntStatus_fld;
    __IO uint32_t Timer1IntStatus;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :8;
      __IO uint32_t timer2_load_count:24;
    }Timer2LoadCount_fld;
    __IO uint32_t Timer2LoadCount;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :8;
      __IO uint32_t timer2_current_value:24;
    }Timer2CurrentValue_fld;
    __IO uint32_t Timer2CurrentValue;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :29;
      __IO uint32_t timer_interrupt_mask:1;
      __IO uint32_t timer_mode:1;
      __IO uint32_t timer_enable:1;
    }Timer2ControlReg_fld;
    __IO uint32_t Timer2ControlReg;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer2_end_of_interrupt:1;
    }Timer2EOI_fld;
    __IO uint32_t Timer2EOI;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer2_interrupt_status:1;
    }Timer2IntStatus_fld;
    __IO uint32_t Timer2IntStatus;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t :8;
      __IO uint32_t timer3_load_count:24;
    }Timer3LoadCount_fld;
    __IO uint32_t Timer3LoadCount;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t :8;
      __IO uint32_t timer3_current_value:24;
    }Timer3CurrentValue_fld;
    __IO uint32_t Timer3CurrentValue;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :29;
      __IO uint32_t timer_interrupt_mask:1;
      __IO uint32_t timer_mode:1;
      __IO uint32_t timer_enable:1;
    }Timer3ControlReg_fld;
    __IO uint32_t Timer3ControlReg;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer3_end_of_interrupt:1;
    }Timer3EOI_fld;
    __IO uint32_t Timer3EOI;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer3_interrupt_status:1;
    }Timer3IntStatus_fld;
    __IO uint32_t Timer3IntStatus;
  };

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t :8;
      __IO uint32_t timer4_load_count:24;
    }Timer4LoadCount_fld;
    __IO uint32_t Timer4LoadCount;
  };

  union{ //offset addr 0x0040
    struct{
      __IO uint32_t :8;
      __IO uint32_t timer4_current_value:24;
    }Timer4CurrentValue_fld;
    __IO uint32_t Timer4CurrentValue;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t :29;
      __IO uint32_t timer_interrupt_mask:1;
      __IO uint32_t timer_mode:1;
      __IO uint32_t timer_enable:1;
    }Timer4ControlReg_fld;
    __IO uint32_t Timer4ControlReg;
  };

  union{ //offset addr 0x0048
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer4_end_of_interrupt:1;
    }Timer4EOI_fld;
    __IO uint32_t Timer4EOI;
  };

  union{ //offset addr 0x004c
    struct{
      __IO uint32_t :31;
      __IO uint32_t timer4_interrupt_status:1;
    }Timer4IntStatus_fld;
    __IO uint32_t Timer4IntStatus;
  };


  union{ //offset addr 0x00a0
    struct{
      __IO uint32_t :28;
      __IO uint32_t timers_interrupt_status:4;
    }TimersIntStatus_fld;
    __IO uint32_t TimersIntStatus;
  };

  union{ //offset addr 0x00a4
    struct{
      __IO uint32_t :28;
      __IO uint32_t timers_end_of_interrupt:4;
    }TimersEOI_fld;
    __IO uint32_t TimersEOI;
  };

  union{ //offset addr 0x00a8
    struct{
      __IO uint32_t :28;
      __IO uint32_t timers_raw_interrupt_status:4;
    }TimersRawIntStatus_0_fld;
    __IO uint32_t TimersRawIntStatus_0;
  };

  union{ //offset addr 0x00ac
    struct{
      __IO uint32_t timers_component_version:32;
    }TimersRawIntStatus_1_fld;
    __IO uint32_t TimersRawIntStatus_1;
  };

} PICO_REG_CP_TIMER_TypeDef;

#define PICO_REG_CP_TIMER PICO_REG_CP_TIMER_TypeDef *0x40001000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_CP_TIMER_H_

