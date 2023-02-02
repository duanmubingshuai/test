#ifndef _PICO_REG_GPIO_H_
#define _PICO_REG_GPIO_H_

#include <stdint.h>

#define GPIO_COUNT 17

#define GPIO_BASE_ADDR 0x40008000

#define GPIO_SIZE 0x00000074


 /**
 * @brief SWPORTA_DR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00   Port_A_Data_Register   0b0
 * </pre>
 */
#define GPIO_SWPORTA_DR_OFFSET 0x00000000


__INLINE uint32_t gpio_swporta_dr_get(void)
{
    return _PICO_REG_RD(GPIO_SWPORTA_DR_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_swporta_dr_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_SWPORTA_DR_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_SWPORTA_DR_PORT_A_DATA_REGISTER_MASK             ((uint32_t)0x1FFFFFFF)
#define GPIO_SWPORTA_DR_PORT_A_DATA_REGISTER_LSB              0
#define GPIO_SWPORTA_DR_PORT_A_DATA_REGISTER_WIDTH            ((uint32_t)0x0000001D)

#define GPIO_SWPORTA_DR_PORT_A_DATA_REGISTER_RST              0x0

__INLINE void gpio_swporta_dr_pack(uint32_t portadataregister)
{
    _PICO_REG_WR(GPIO_SWPORTA_DR_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)portadataregister << 0));
}

__INLINE void gpio_swporta_dr_unpack(uint8_t* portadataregister)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_SWPORTA_DR_OFFSET + GPIO_BASE_ADDR);

    *portadataregister = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_swporta_dr_port_a_data_register_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_SWPORTA_DR_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_swporta_dr_port_a_data_register_setf(uint32_t portadataregister)
{
    _PICO_REG_WR(GPIO_SWPORTA_DR_OFFSET+ GPIO_BASE_ADDR, (uint32_t)portadataregister << 0);
}

 /**
 * @brief SWPORTA_DDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00   Port_A_Data_Direction_Register   0b0
 * </pre>
 */
#define GPIO_SWPORTA_DDR_OFFSET 0x00000004


__INLINE uint32_t gpio_swporta_ddr_get(void)
{
    return _PICO_REG_RD(GPIO_SWPORTA_DDR_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_swporta_ddr_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_SWPORTA_DDR_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_SWPORTA_DDR_PORT_A_DATA_DIRECTION_REGISTER_MASK   ((uint32_t)0x1FFFFFFF)
#define GPIO_SWPORTA_DDR_PORT_A_DATA_DIRECTION_REGISTER_LSB    0
#define GPIO_SWPORTA_DDR_PORT_A_DATA_DIRECTION_REGISTER_WIDTH  ((uint32_t)0x0000001D)

#define GPIO_SWPORTA_DDR_PORT_A_DATA_DIRECTION_REGISTER_RST    0x0

__INLINE void gpio_swporta_ddr_pack(uint32_t portadatadirectionregister)
{
    _PICO_REG_WR(GPIO_SWPORTA_DDR_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)portadatadirectionregister << 0));
}

__INLINE void gpio_swporta_ddr_unpack(uint8_t* portadatadirectionregister)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_SWPORTA_DDR_OFFSET + GPIO_BASE_ADDR);

    *portadatadirectionregister = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_swporta_ddr_port_a_data_direction_register_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_SWPORTA_DDR_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_swporta_ddr_port_a_data_direction_register_setf(uint32_t portadatadirectionregister)
{
    _PICO_REG_WR(GPIO_SWPORTA_DDR_OFFSET+ GPIO_BASE_ADDR, (uint32_t)portadatadirectionregister << 0);
}

 /**
 * @brief SWPORTA_CTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Port_A_Data_Source   0
 * </pre>
 */
#define GPIO_SWPORTA_CTL_OFFSET 0x00000008


__INLINE uint32_t gpio_swporta_ctl_get(void)
{
    return _PICO_REG_RD(GPIO_SWPORTA_CTL_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_swporta_ctl_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_SWPORTA_CTL_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_SWPORTA_CTL_PORT_A_DATA_SOURCE_BIT                ((uint32_t)0x00000001)
#define GPIO_SWPORTA_CTL_PORT_A_DATA_SOURCE_POS                0

#define GPIO_SWPORTA_CTL_PORT_A_DATA_SOURCE_RST                0x0

__INLINE void gpio_swporta_ctl_pack(uint8_t portadatasource)
{
    _PICO_REG_WR(GPIO_SWPORTA_CTL_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)portadatasource << 0));
}

__INLINE void gpio_swporta_ctl_unpack(uint8_t* portadatasource)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_SWPORTA_CTL_OFFSET + GPIO_BASE_ADDR);

    *portadatasource = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t gpio_swporta_ctl_port_a_data_source_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_SWPORTA_CTL_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void gpio_swporta_ctl_port_a_data_source_setf(uint8_t portadatasource)
{
    _PICO_REG_WR(GPIO_SWPORTA_CTL_OFFSET+ GPIO_BASE_ADDR, (uint32_t)portadatasource << 0);
}

 /**
 * @brief INTEN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00     Interrupt_enable   0b0
 * </pre>
 */
#define GPIO_INTEN_OFFSET 0x00000030


__INLINE uint32_t gpio_inten_get(void)
{
    return _PICO_REG_RD(GPIO_INTEN_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_inten_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_INTEN_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_INTEN_INTERRUPT_ENABLE_MASK                 ((uint32_t)0x1FFFFFFF)
#define GPIO_INTEN_INTERRUPT_ENABLE_LSB                  0
#define GPIO_INTEN_INTERRUPT_ENABLE_WIDTH                ((uint32_t)0x0000001D)

#define GPIO_INTEN_INTERRUPT_ENABLE_RST                  0x0

__INLINE void gpio_inten_pack(uint32_t interruptenable)
{
    _PICO_REG_WR(GPIO_INTEN_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)interruptenable << 0));
}

__INLINE void gpio_inten_unpack(uint8_t* interruptenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTEN_OFFSET + GPIO_BASE_ADDR);

    *interruptenable = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_inten_interrupt_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTEN_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_inten_interrupt_enable_setf(uint32_t interruptenable)
{
    _PICO_REG_WR(GPIO_INTEN_OFFSET+ GPIO_BASE_ADDR, (uint32_t)interruptenable << 0);
}

 /**
 * @brief INTMASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00       Interrupt_mask   0b0
 * </pre>
 */
#define GPIO_INTMASK_OFFSET 0x00000034


__INLINE uint32_t gpio_intmask_get(void)
{
    return _PICO_REG_RD(GPIO_INTMASK_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_intmask_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_INTMASK_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_INTMASK_INTERRUPT_MASK_MASK                   ((uint32_t)0x1FFFFFFF)
#define GPIO_INTMASK_INTERRUPT_MASK_LSB                    0
#define GPIO_INTMASK_INTERRUPT_MASK_WIDTH                  ((uint32_t)0x0000001D)

#define GPIO_INTMASK_INTERRUPT_MASK_RST                    0x0

__INLINE void gpio_intmask_pack(uint32_t interruptmask)
{
    _PICO_REG_WR(GPIO_INTMASK_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)interruptmask << 0));
}

__INLINE void gpio_intmask_unpack(uint8_t* interruptmask)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTMASK_OFFSET + GPIO_BASE_ADDR);

    *interruptmask = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_intmask_interrupt_mask_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTMASK_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_intmask_interrupt_mask_setf(uint32_t interruptmask)
{
    _PICO_REG_WR(GPIO_INTMASK_OFFSET+ GPIO_BASE_ADDR, (uint32_t)interruptmask << 0);
}

 /**
 * @brief INTTYPE_LEVEL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00      Interrupt_level   0b0
 * </pre>
 */
#define GPIO_INTTYPE_LEVEL_OFFSET 0x00000038


__INLINE uint32_t gpio_inttype_level_get(void)
{
    return _PICO_REG_RD(GPIO_INTTYPE_LEVEL_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_inttype_level_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_INTTYPE_LEVEL_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_INTTYPE_LEVEL_INTERRUPT_LEVEL_MASK                  ((uint32_t)0x1FFFFFFF)
#define GPIO_INTTYPE_LEVEL_INTERRUPT_LEVEL_LSB                   0
#define GPIO_INTTYPE_LEVEL_INTERRUPT_LEVEL_WIDTH                 ((uint32_t)0x0000001D)

#define GPIO_INTTYPE_LEVEL_INTERRUPT_LEVEL_RST                   0x0

__INLINE void gpio_inttype_level_pack(uint32_t interruptlevel)
{
    _PICO_REG_WR(GPIO_INTTYPE_LEVEL_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)interruptlevel << 0));
}

__INLINE void gpio_inttype_level_unpack(uint8_t* interruptlevel)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTTYPE_LEVEL_OFFSET + GPIO_BASE_ADDR);

    *interruptlevel = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_inttype_level_interrupt_level_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTTYPE_LEVEL_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_inttype_level_interrupt_level_setf(uint32_t interruptlevel)
{
    _PICO_REG_WR(GPIO_INTTYPE_LEVEL_OFFSET+ GPIO_BASE_ADDR, (uint32_t)interruptlevel << 0);
}

 /**
 * @brief INT_POLARITY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00   Interrupt_polarity   0b0
 * </pre>
 */
#define GPIO_INT_POLARITY_OFFSET 0x0000003C


__INLINE uint32_t gpio_int_polarity_get(void)
{
    return _PICO_REG_RD(GPIO_INT_POLARITY_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_int_polarity_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_INT_POLARITY_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_INT_POLARITY_INTERRUPT_POLARITY_MASK               ((uint32_t)0x1FFFFFFF)
#define GPIO_INT_POLARITY_INTERRUPT_POLARITY_LSB                0
#define GPIO_INT_POLARITY_INTERRUPT_POLARITY_WIDTH              ((uint32_t)0x0000001D)

#define GPIO_INT_POLARITY_INTERRUPT_POLARITY_RST                0x0

__INLINE void gpio_int_polarity_pack(uint32_t interruptpolarity)
{
    _PICO_REG_WR(GPIO_INT_POLARITY_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)interruptpolarity << 0));
}

__INLINE void gpio_int_polarity_unpack(uint8_t* interruptpolarity)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INT_POLARITY_OFFSET + GPIO_BASE_ADDR);

    *interruptpolarity = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_int_polarity_interrupt_polarity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INT_POLARITY_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_int_polarity_interrupt_polarity_setf(uint32_t interruptpolarity)
{
    _PICO_REG_WR(GPIO_INT_POLARITY_OFFSET+ GPIO_BASE_ADDR, (uint32_t)interruptpolarity << 0);
}

 /**
 * @brief INTSTATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00     Interrupt_status   0b0
 * </pre>
 */
#define GPIO_INTSTATUS_OFFSET 0x00000040


__INLINE uint32_t gpio_intstatus_get(void)
{
    return _PICO_REG_RD(GPIO_INTSTATUS_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_intstatus_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_INTSTATUS_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_INTSTATUS_INTERRUPT_STATUS_MASK                 ((uint32_t)0x1FFFFFFF)
#define GPIO_INTSTATUS_INTERRUPT_STATUS_LSB                  0
#define GPIO_INTSTATUS_INTERRUPT_STATUS_WIDTH                ((uint32_t)0x0000001D)

#define GPIO_INTSTATUS_INTERRUPT_STATUS_RST                  0x0

__INLINE void gpio_intstatus_pack(uint32_t interruptstatus)
{
    _PICO_REG_WR(GPIO_INTSTATUS_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)interruptstatus << 0));
}

__INLINE void gpio_intstatus_unpack(uint8_t* interruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTSTATUS_OFFSET + GPIO_BASE_ADDR);

    *interruptstatus = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_intstatus_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_INTSTATUS_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_intstatus_interrupt_status_setf(uint32_t interruptstatus)
{
    _PICO_REG_WR(GPIO_INTSTATUS_OFFSET+ GPIO_BASE_ADDR, (uint32_t)interruptstatus << 0);
}

 /**
 * @brief RAW_INTSTATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00   Raw_interrupt_status   0b0
 * </pre>
 */
#define GPIO_RAW_INTSTATUS_OFFSET 0x00000044


__INLINE uint32_t gpio_raw_intstatus_get(void)
{
    return _PICO_REG_RD(GPIO_RAW_INTSTATUS_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_raw_intstatus_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_RAW_INTSTATUS_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_RAW_INTSTATUS_RAW_INTERRUPT_STATUS_MASK             ((uint32_t)0x1FFFFFFF)
#define GPIO_RAW_INTSTATUS_RAW_INTERRUPT_STATUS_LSB              0
#define GPIO_RAW_INTSTATUS_RAW_INTERRUPT_STATUS_WIDTH            ((uint32_t)0x0000001D)

#define GPIO_RAW_INTSTATUS_RAW_INTERRUPT_STATUS_RST              0x0

__INLINE void gpio_raw_intstatus_pack(uint32_t rawinterruptstatus)
{
    _PICO_REG_WR(GPIO_RAW_INTSTATUS_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)rawinterruptstatus << 0));
}

__INLINE void gpio_raw_intstatus_unpack(uint8_t* rawinterruptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_RAW_INTSTATUS_OFFSET + GPIO_BASE_ADDR);

    *rawinterruptstatus = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_raw_intstatus_raw_interrupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_RAW_INTSTATUS_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_raw_intstatus_raw_interrupt_status_setf(uint32_t rawinterruptstatus)
{
    _PICO_REG_WR(GPIO_RAW_INTSTATUS_OFFSET+ GPIO_BASE_ADDR, (uint32_t)rawinterruptstatus << 0);
}

 /**
 * @brief DEBOUNCE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00      Debounce_enable   0b0
 * </pre>
 */
#define GPIO_DEBOUNCE_OFFSET 0x00000048


__INLINE uint32_t gpio_debounce_get(void)
{
    return _PICO_REG_RD(GPIO_DEBOUNCE_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_debounce_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_DEBOUNCE_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_DEBOUNCE_DEBOUNCE_ENABLE_MASK                  ((uint32_t)0x1FFFFFFF)
#define GPIO_DEBOUNCE_DEBOUNCE_ENABLE_LSB                   0
#define GPIO_DEBOUNCE_DEBOUNCE_ENABLE_WIDTH                 ((uint32_t)0x0000001D)

#define GPIO_DEBOUNCE_DEBOUNCE_ENABLE_RST                   0x0

__INLINE void gpio_debounce_pack(uint32_t debounceenable)
{
    _PICO_REG_WR(GPIO_DEBOUNCE_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)debounceenable << 0));
}

__INLINE void gpio_debounce_unpack(uint8_t* debounceenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_DEBOUNCE_OFFSET + GPIO_BASE_ADDR);

    *debounceenable = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_debounce_debounce_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_DEBOUNCE_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_debounce_debounce_enable_setf(uint32_t debounceenable)
{
    _PICO_REG_WR(GPIO_DEBOUNCE_OFFSET+ GPIO_BASE_ADDR, (uint32_t)debounceenable << 0);
}

 /**
 * @brief PORTA_EOI register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00      Clear_interrupt   0b0
 * </pre>
 */
#define GPIO_PORTA_EOI_OFFSET 0x0000004C


__INLINE uint32_t gpio_porta_eoi_get(void)
{
    return _PICO_REG_RD(GPIO_PORTA_EOI_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_porta_eoi_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_PORTA_EOI_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_PORTA_EOI_CLEAR_INTERRUPT_MASK                  ((uint32_t)0x1FFFFFFF)
#define GPIO_PORTA_EOI_CLEAR_INTERRUPT_LSB                   0
#define GPIO_PORTA_EOI_CLEAR_INTERRUPT_WIDTH                 ((uint32_t)0x0000001D)

#define GPIO_PORTA_EOI_CLEAR_INTERRUPT_RST                   0x0

__INLINE void gpio_porta_eoi_pack(uint32_t clearinterrupt)
{
    _PICO_REG_WR(GPIO_PORTA_EOI_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)clearinterrupt << 0));
}

__INLINE void gpio_porta_eoi_unpack(uint8_t* clearinterrupt)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_PORTA_EOI_OFFSET + GPIO_BASE_ADDR);

    *clearinterrupt = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_porta_eoi_clear_interrupt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_PORTA_EOI_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_porta_eoi_clear_interrupt_setf(uint32_t clearinterrupt)
{
    _PICO_REG_WR(GPIO_PORTA_EOI_OFFSET+ GPIO_BASE_ADDR, (uint32_t)clearinterrupt << 0);
}

 /**
 * @brief EXT_PORTA register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  28:00      External_Port_A   0b0
 * </pre>
 */
#define GPIO_EXT_PORTA_OFFSET 0x00000050


__INLINE uint32_t gpio_ext_porta_get(void)
{
    return _PICO_REG_RD(GPIO_EXT_PORTA_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_ext_porta_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_EXT_PORTA_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_EXT_PORTA_EXTERNAL_PORT_A_MASK                  ((uint32_t)0x1FFFFFFF)
#define GPIO_EXT_PORTA_EXTERNAL_PORT_A_LSB                   0
#define GPIO_EXT_PORTA_EXTERNAL_PORT_A_WIDTH                 ((uint32_t)0x0000001D)

#define GPIO_EXT_PORTA_EXTERNAL_PORT_A_RST                   0x0

__INLINE void gpio_ext_porta_pack(uint32_t externalporta)
{
    _PICO_REG_WR(GPIO_EXT_PORTA_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)externalporta << 0));
}

__INLINE void gpio_ext_porta_unpack(uint8_t* externalporta)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_EXT_PORTA_OFFSET + GPIO_BASE_ADDR);

    *externalporta = (localVal & ((uint32_t)0x1FFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_ext_porta_external_port_a_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_EXT_PORTA_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x1FFFFFFF)) >> 0);
}

__INLINE void gpio_ext_porta_external_port_a_setf(uint32_t externalporta)
{
    _PICO_REG_WR(GPIO_EXT_PORTA_OFFSET+ GPIO_BASE_ADDR, (uint32_t)externalporta << 0);
}

 /**
 * @brief LS_SYNC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00   Synchronization_level   0
 * </pre>
 */
#define GPIO_LS_SYNC_OFFSET 0x00000060


__INLINE uint32_t gpio_ls_sync_get(void)
{
    return _PICO_REG_RD(GPIO_LS_SYNC_OFFSET + GPIO_BASE_ADDR);
}

__INLINE void gpio_ls_sync_set(uint32_t value)
{
    _PICO_REG_WR(GPIO_LS_SYNC_OFFSET+ GPIO_BASE_ADDR, value);
}

// field definitions
#define GPIO_LS_SYNC_SYNCHRONIZATION_LEVEL_BIT             ((uint32_t)0x00000001)
#define GPIO_LS_SYNC_SYNCHRONIZATION_LEVEL_POS             0

#define GPIO_LS_SYNC_SYNCHRONIZATION_LEVEL_RST             0x0

__INLINE void gpio_ls_sync_pack(uint8_t synchronizationlevel)
{
    _PICO_REG_WR(GPIO_LS_SYNC_OFFSET+ GPIO_BASE_ADDR,  ((uint32_t)synchronizationlevel << 0));
}

__INLINE void gpio_ls_sync_unpack(uint8_t* synchronizationlevel)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_LS_SYNC_OFFSET + GPIO_BASE_ADDR);

    *synchronizationlevel = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t gpio_ls_sync_synchronization_level_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_LS_SYNC_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void gpio_ls_sync_synchronization_level_setf(uint8_t synchronizationlevel)
{
    _PICO_REG_WR(GPIO_LS_SYNC_OFFSET+ GPIO_BASE_ADDR, (uint32_t)synchronizationlevel << 0);
}

 /**
 * @brief ID_CODE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              ID_code   0b0
 * </pre>
 */
#define GPIO_ID_CODE_OFFSET 0x00000064


__INLINE uint32_t gpio_id_code_get(void)
{
    return _PICO_REG_RD(GPIO_ID_CODE_OFFSET + GPIO_BASE_ADDR);
}

// field definitions
#define GPIO_ID_CODE_ID_CODE_MASK                          ((uint32_t)0x0000FFFF)
#define GPIO_ID_CODE_ID_CODE_LSB                           0
#define GPIO_ID_CODE_ID_CODE_WIDTH                         ((uint32_t)0x00000010)

#define GPIO_ID_CODE_ID_CODE_RST                           0x0

__INLINE void gpio_id_code_unpack(uint8_t* idcode)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_ID_CODE_OFFSET + GPIO_BASE_ADDR);

    *idcode = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t gpio_id_code_id_code_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_ID_CODE_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

 /**
 * @brief VER_ID_CODE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00    Component_Version   0b0
 * </pre>
 */
#define GPIO_VER_ID_CODE_OFFSET 0x0000006C


__INLINE uint32_t gpio_ver_id_code_get(void)
{
    return _PICO_REG_RD(GPIO_VER_ID_CODE_OFFSET + GPIO_BASE_ADDR);
}

// field definitions
#define GPIO_VER_ID_CODE_COMPONENT_VERSION_MASK                ((uint32_t)0xFFFFFFFF)
#define GPIO_VER_ID_CODE_COMPONENT_VERSION_LSB                 0
#define GPIO_VER_ID_CODE_COMPONENT_VERSION_WIDTH               ((uint32_t)0x00000020)

#define GPIO_VER_ID_CODE_COMPONENT_VERSION_RST                 0x0

__INLINE void gpio_ver_id_code_unpack(uint8_t* componentversion)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_VER_ID_CODE_OFFSET + GPIO_BASE_ADDR);

    *componentversion = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t gpio_ver_id_code_component_version_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_VER_ID_CODE_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief CONFIG_REG1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  20:16     ENCODED_ID_WIDTH   0bF
 *     15                   ID   0
 *     14   ADD_ENCODED_PARAMS   0
 *     13             DEBOUNCE   0
 *     12           PORTA_INTR   0
 *     09             HW_PORTB   0
 *     08             HW_PORTA   0
 *     05     PORTB_SINGLE_CTL   0
 *     04     PORTA_SINGLE_CTL   0
 *  03:02            NUM_PORTS   0b2
 *  01:00       APB_DATA_WIDTH   0b2
 * </pre>
 */
#define GPIO_CONFIG_REG1_OFFSET 0x00000074


__INLINE uint32_t gpio_config_reg1_get(void)
{
    return _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
}

// field definitions
#define GPIO_CONFIG_REG1_ENCODED_ID_WIDTH_MASK                 ((uint32_t)0x001F0000)
#define GPIO_CONFIG_REG1_ENCODED_ID_WIDTH_LSB                  16
#define GPIO_CONFIG_REG1_ENCODED_ID_WIDTH_WIDTH                ((uint32_t)0x00000005)
#define GPIO_CONFIG_REG1_ID_BIT                                ((uint32_t)0x00008000)
#define GPIO_CONFIG_REG1_ID_POS                                15
#define GPIO_CONFIG_REG1_ADD_ENCODED_PARAMS_BIT                ((uint32_t)0x00004000)
#define GPIO_CONFIG_REG1_ADD_ENCODED_PARAMS_POS                14
#define GPIO_CONFIG_REG1_DEBOUNCE_BIT                          ((uint32_t)0x00002000)
#define GPIO_CONFIG_REG1_DEBOUNCE_POS                          13
#define GPIO_CONFIG_REG1_PORTA_INTR_BIT                        ((uint32_t)0x00001000)
#define GPIO_CONFIG_REG1_PORTA_INTR_POS                        12
#define GPIO_CONFIG_REG1_HW_PORTB_BIT                          ((uint32_t)0x00000200)
#define GPIO_CONFIG_REG1_HW_PORTB_POS                          9
#define GPIO_CONFIG_REG1_HW_PORTA_BIT                          ((uint32_t)0x00000100)
#define GPIO_CONFIG_REG1_HW_PORTA_POS                          8
#define GPIO_CONFIG_REG1_PORTB_SINGLE_CTL_BIT                  ((uint32_t)0x00000020)
#define GPIO_CONFIG_REG1_PORTB_SINGLE_CTL_POS                  5
#define GPIO_CONFIG_REG1_PORTA_SINGLE_CTL_BIT                  ((uint32_t)0x00000010)
#define GPIO_CONFIG_REG1_PORTA_SINGLE_CTL_POS                  4
#define GPIO_CONFIG_REG1_NUM_PORTS_MASK                        ((uint32_t)0x0000000C)
#define GPIO_CONFIG_REG1_NUM_PORTS_LSB                         2
#define GPIO_CONFIG_REG1_NUM_PORTS_WIDTH                       ((uint32_t)0x00000002)
#define GPIO_CONFIG_REG1_APB_DATA_WIDTH_MASK                   ((uint32_t)0x00000003)
#define GPIO_CONFIG_REG1_APB_DATA_WIDTH_LSB                    0
#define GPIO_CONFIG_REG1_APB_DATA_WIDTH_WIDTH                  ((uint32_t)0x00000002)

#define GPIO_CONFIG_REG1_ENCODED_ID_WIDTH_RST                  0xF
#define GPIO_CONFIG_REG1_ID_RST                                0x0
#define GPIO_CONFIG_REG1_ADD_ENCODED_PARAMS_RST                0x0
#define GPIO_CONFIG_REG1_DEBOUNCE_RST                          0x0
#define GPIO_CONFIG_REG1_PORTA_INTR_RST                        0x0
#define GPIO_CONFIG_REG1_HW_PORTB_RST                          0x0
#define GPIO_CONFIG_REG1_HW_PORTA_RST                          0x0
#define GPIO_CONFIG_REG1_PORTB_SINGLE_CTL_RST                  0x0
#define GPIO_CONFIG_REG1_PORTA_SINGLE_CTL_RST                  0x0
#define GPIO_CONFIG_REG1_NUM_PORTS_RST                         0x2
#define GPIO_CONFIG_REG1_APB_DATA_WIDTH_RST                    0x2

__INLINE void gpio_config_reg1_unpack(uint8_t* encodedidwidth, uint8_t* id, uint8_t* addencodedparams, uint8_t* debounce, uint8_t* portaintr, uint8_t* hwportb, uint8_t* hwporta, uint8_t* portbsinglectl, uint8_t* portasinglectl, uint8_t* numports, uint8_t* apbdatawidth)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);

    *encodedidwidth = (localVal & ((uint32_t)0x001F0000)) >> 16;
    *id = (localVal & ((uint32_t)0x00008000)) >> 15;
    *addencodedparams = (localVal & ((uint32_t)0x00004000)) >> 14;
    *debounce = (localVal & ((uint32_t)0x00002000)) >> 13;
    *portaintr = (localVal & ((uint32_t)0x00001000)) >> 12;
    *hwportb = (localVal & ((uint32_t)0x00000200)) >> 9;
    *hwporta = (localVal & ((uint32_t)0x00000100)) >> 8;
    *portbsinglectl = (localVal & ((uint32_t)0x00000020)) >> 5;
    *portasinglectl = (localVal & ((uint32_t)0x00000010)) >> 4;
    *numports = (localVal & ((uint32_t)0x0000000C)) >> 2;
    *apbdatawidth = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t gpio_config_reg1_encoded_id_width_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x001F0000)) >> 16);
}

__INLINE uint8_t gpio_config_reg1_id_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t gpio_config_reg1_add_encoded_params_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t gpio_config_reg1_debounce_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t gpio_config_reg1_porta_intr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t gpio_config_reg1_hw_portb_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t gpio_config_reg1_hw_porta_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t gpio_config_reg1_portb_single_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t gpio_config_reg1_porta_single_ctl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t gpio_config_reg1_num_ports_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000C)) >> 2);
}

__INLINE uint8_t gpio_config_reg1_apb_data_width_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG1_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

 /**
 * @brief CONFIG_REG2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:05   ENCODED_ID_PWIDTH_B   0bF
 *  04:00   ENCODED_ID_PWIDTH_A   0b11
 * </pre>
 */
#define GPIO_CONFIG_REG2_OFFSET 0x00000070


__INLINE uint32_t gpio_config_reg2_get(void)
{
    return _PICO_REG_RD(GPIO_CONFIG_REG2_OFFSET + GPIO_BASE_ADDR);
}

// field definitions
#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_B_MASK              ((uint32_t)0x000003E0)
#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_B_LSB               5
#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_B_WIDTH             ((uint32_t)0x00000005)
#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_A_MASK              ((uint32_t)0x0000001F)
#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_A_LSB               0
#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_A_WIDTH             ((uint32_t)0x00000005)

#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_B_RST               0xF
#define GPIO_CONFIG_REG2_ENCODED_ID_PWIDTH_A_RST               0x11

__INLINE void gpio_config_reg2_unpack(uint8_t* encodedidpwidthb, uint8_t* encodedidpwidtha)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG2_OFFSET + GPIO_BASE_ADDR);

    *encodedidpwidthb = (localVal & ((uint32_t)0x000003E0)) >> 5;
    *encodedidpwidtha = (localVal & ((uint32_t)0x0000001F)) >> 0;
}

__INLINE uint8_t gpio_config_reg2_encoded_id_pwidth_b_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG2_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000003E0)) >> 5);
}

__INLINE uint8_t gpio_config_reg2_encoded_id_pwidth_a_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(GPIO_CONFIG_REG2_OFFSET + GPIO_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000001F)) >> 0);
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :3;
      __IO uint32_t port_a_data_register:29;
    }swporta_dr_fld;
    __IO uint32_t swporta_dr;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :3;
      __IO uint32_t port_a_data_direction_register:29;
    }swporta_ddr_fld;
    __IO uint32_t swporta_ddr;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :31;
      __IO uint32_t port_a_data_source:1;
    }swporta_ctl_fld;
    __IO uint32_t swporta_ctl;
  };


  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :3;
      __IO uint32_t interrupt_enable:29;
    }inten_fld;
    __IO uint32_t inten;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t :3;
      __IO uint32_t interrupt_mask:29;
    }intmask_fld;
    __IO uint32_t intmask;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t :3;
      __IO uint32_t interrupt_level:29;
    }inttype_level_fld;
    __IO uint32_t inttype_level;
  };

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t :3;
      __IO uint32_t interrupt_polarity:29;
    }int_polarity_fld;
    __IO uint32_t int_polarity;
  };

  union{ //offset addr 0x0040
    struct{
      __IO uint32_t :3;
      __IO uint32_t interrupt_status:29;
    }intstatus_fld;
    __IO uint32_t intstatus;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t :3;
      __IO uint32_t raw_interrupt_status:29;
    }raw_intstatus_fld;
    __IO uint32_t raw_intstatus;
  };

  union{ //offset addr 0x0048
    struct{
      __IO uint32_t :3;
      __IO uint32_t debounce_enable:29;
    }debounce_fld;
    __IO uint32_t debounce;
  };

  union{ //offset addr 0x004c
    struct{
      __IO uint32_t :3;
      __IO uint32_t clear_interrupt:29;
    }porta_eoi_fld;
    __IO uint32_t porta_eoi;
  };

  union{ //offset addr 0x0050
    struct{
      __IO uint32_t :3;
      __IO uint32_t external_port_a:29;
    }ext_porta_fld;
    __IO uint32_t ext_porta;
  };


  union{ //offset addr 0x0060
    struct{
      __IO uint32_t :31;
      __IO uint32_t synchronization_level:1;
    }ls_sync_fld;
    __IO uint32_t ls_sync;
  };

  union{ //offset addr 0x0064
    struct{
      __IO uint32_t :16;
      __IO uint32_t id_code:16;
    }id_code_fld;
    __IO uint32_t id_code;
  };


  union{ //offset addr 0x006c
    struct{
      __IO uint32_t component_version:32;
    }ver_id_code_fld;
    __IO uint32_t ver_id_code;
  };


  union{ //offset addr 0x0074
    struct{
      __IO uint32_t :11;
      __IO uint32_t encoded_id_width:5;
      __IO uint32_t id:1;
      __IO uint32_t add_encoded_params:1;
      __IO uint32_t debounce:1;
      __IO uint32_t porta_intr:1;
      __IO uint32_t :2;
      __IO uint32_t hw_portb:1;
      __IO uint32_t hw_porta:1;
      __IO uint32_t :2;
      __IO uint32_t portb_single_ctl:1;
      __IO uint32_t porta_single_ctl:1;
      __IO uint32_t num_ports:2;
      __IO uint32_t apb_data_width:2;
    }config_reg1_fld;
    __IO uint32_t config_reg1;
  };


  union{ //offset addr 0x0070
    struct{
      __IO uint32_t :22;
      __IO uint32_t encoded_id_pwidth_b:5;
      __IO uint32_t encoded_id_pwidth_a:5;
    }config_reg2_fld;
    __IO uint32_t config_reg2;
  };

} PICO_REG_GPIO_TypeDef;

#define PICO_REG_GPIO PICO_REG_GPIO_TypeDef *0x40008000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_GPIO_H_

