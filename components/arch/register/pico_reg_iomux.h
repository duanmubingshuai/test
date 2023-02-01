#ifndef _PICO_REG_IOMUX_H_
#define _PICO_REG_IOMUX_H_

#include <stdint.h>

#define IOMUX_COUNT 22

#define IOMUX_BASE_ADDR 0x40003800

#define IOMUX_SIZE 0x00000080


 /**
 * @brief R_ANALOG_IO register definition
 */
#define IOMUX_R_ANALOG_IO_OFFSET 0x00000000


__INLINE uint32_t iomux_r_analog_io_get(void)
{
    return _PICO_REG_RD(IOMUX_R_ANALOG_IO_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_R_ANALOG_IO_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_R_ANALOG_IO_RESERVED_LSB                          0
#define IOMUX_R_ANALOG_IO_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_R_ANALOG_IO_RESERVED_RST                          0x0

__INLINE void iomux_r_analog_io_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_R_ANALOG_IO_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief SPI_DEBUG register definition
 */
#define IOMUX_SPI_DEBUG_OFFSET 0x00000004


__INLINE uint32_t iomux_spi_debug_get(void)
{
    return _PICO_REG_RD(IOMUX_SPI_DEBUG_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_SPI_DEBUG_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_SPI_DEBUG_RESERVED_LSB                          0
#define IOMUX_SPI_DEBUG_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_SPI_DEBUG_RESERVED_RST                          0x0

__INLINE void iomux_spi_debug_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_SPI_DEBUG_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief DEBUG_MUX register definition
 */
#define IOMUX_DEBUG_MUX_OFFSET 0x00000008


__INLINE uint32_t iomux_debug_mux_get(void)
{
    return _PICO_REG_RD(IOMUX_DEBUG_MUX_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_DEBUG_MUX_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_DEBUG_MUX_RESERVED_LSB                          0
#define IOMUX_DEBUG_MUX_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_DEBUG_MUX_RESERVED_RST                          0x0

__INLINE void iomux_debug_mux_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_DEBUG_MUX_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief FULL_MUX0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00         r_func_io_en   0b0
 * </pre>
 */
#define IOMUX_FULL_MUX0_OFFSET 0x0000000C


__INLINE uint32_t iomux_full_mux0_get(void)
{
    return _PICO_REG_RD(IOMUX_FULL_MUX0_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_full_mux0_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_FULL_MUX0_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_FULL_MUX0_R_FUNC_IO_EN_MASK                     ((uint32_t)0x000FFFFF)
#define IOMUX_FULL_MUX0_R_FUNC_IO_EN_LSB                      0
#define IOMUX_FULL_MUX0_R_FUNC_IO_EN_WIDTH                    ((uint32_t)0x00000014)

#define IOMUX_FULL_MUX0_R_FUNC_IO_EN_RST                      0x0

__INLINE void iomux_full_mux0_pack(uint32_t rfuncioen)
{
    _PICO_REG_WR(IOMUX_FULL_MUX0_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)rfuncioen << 0));
}

__INLINE void iomux_full_mux0_unpack(uint8_t* rfuncioen)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FULL_MUX0_OFFSET + IOMUX_BASE_ADDR);

    *rfuncioen = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t iomux_full_mux0_r_func_io_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FULL_MUX0_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void iomux_full_mux0_r_func_io_en_setf(uint32_t rfuncioen)
{
    _PICO_REG_WR(IOMUX_FULL_MUX0_OFFSET+ IOMUX_BASE_ADDR, (uint32_t)rfuncioen << 0);
}

 /**
 * @brief FULL_MUX1 register definition
 */
#define IOMUX_FULL_MUX1_OFFSET 0x00000010


__INLINE uint32_t iomux_full_mux1_get(void)
{
    return _PICO_REG_RD(IOMUX_FULL_MUX1_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_FULL_MUX1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_FULL_MUX1_RESERVED_LSB                          0
#define IOMUX_FULL_MUX1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_FULL_MUX1_RESERVED_RST                          0x0

__INLINE void iomux_full_mux1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FULL_MUX1_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief GPIO_PAPB register definition
 */
#define IOMUX_GPIO_PAPB_OFFSET 0x00000014


__INLINE uint32_t iomux_gpio_papb_get(void)
{
    return _PICO_REG_RD(IOMUX_GPIO_PAPB_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_GPIO_PAPB_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_GPIO_PAPB_RESERVED_LSB                          0
#define IOMUX_GPIO_PAPB_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_GPIO_PAPB_RESERVED_RST                          0x0

__INLINE void iomux_gpio_papb_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_GPIO_PAPB_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief FUNC_IO0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  26:24      r_func_io03_sel   0b0
 *  18:16      r_func_io02_sel   0b0
 *  09:08      r_func_io01_sel   0b0
 *  01:00      r_func_io00_sel   0b0
 * </pre>
 */
#define IOMUX_FUNC_IO0_OFFSET 0x00000018


__INLINE uint32_t iomux_func_io0_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_func_io0_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_FUNC_IO0_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_FUNC_IO0_R_FUNC_IO03_SEL_MASK                  ((uint32_t)0x07000000)
#define IOMUX_FUNC_IO0_R_FUNC_IO03_SEL_LSB                   24
#define IOMUX_FUNC_IO0_R_FUNC_IO03_SEL_WIDTH                 ((uint32_t)0x00000003)
#define IOMUX_FUNC_IO0_R_FUNC_IO02_SEL_MASK                  ((uint32_t)0x00070000)
#define IOMUX_FUNC_IO0_R_FUNC_IO02_SEL_LSB                   16
#define IOMUX_FUNC_IO0_R_FUNC_IO02_SEL_WIDTH                 ((uint32_t)0x00000003)
#define IOMUX_FUNC_IO0_R_FUNC_IO01_SEL_MASK                  ((uint32_t)0x00000300)
#define IOMUX_FUNC_IO0_R_FUNC_IO01_SEL_LSB                   8
#define IOMUX_FUNC_IO0_R_FUNC_IO01_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO0_R_FUNC_IO00_SEL_MASK                  ((uint32_t)0x00000003)
#define IOMUX_FUNC_IO0_R_FUNC_IO00_SEL_LSB                   0
#define IOMUX_FUNC_IO0_R_FUNC_IO00_SEL_WIDTH                 ((uint32_t)0x00000002)

#define IOMUX_FUNC_IO0_R_FUNC_IO03_SEL_RST                   0x0
#define IOMUX_FUNC_IO0_R_FUNC_IO02_SEL_RST                   0x0
#define IOMUX_FUNC_IO0_R_FUNC_IO01_SEL_RST                   0x0
#define IOMUX_FUNC_IO0_R_FUNC_IO00_SEL_RST                   0x0

__INLINE void iomux_func_io0_pack(uint8_t rfuncio03sel, uint8_t rfuncio02sel, uint8_t rfuncio01sel, uint8_t rfuncio00sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO0_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)rfuncio03sel << 24) | ((uint32_t)rfuncio02sel << 16) | ((uint32_t)rfuncio01sel << 8) | ((uint32_t)rfuncio00sel << 0));
}

__INLINE void iomux_func_io0_unpack(uint8_t* rfuncio03sel, uint8_t* rfuncio02sel, uint8_t* rfuncio01sel, uint8_t* rfuncio00sel)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR);

    *rfuncio03sel = (localVal & ((uint32_t)0x07000000)) >> 24;
    *rfuncio02sel = (localVal & ((uint32_t)0x00070000)) >> 16;
    *rfuncio01sel = (localVal & ((uint32_t)0x00000300)) >> 8;
    *rfuncio00sel = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t iomux_func_io0_r_func_io03_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x07000000)) >> 24);
}

__INLINE void iomux_func_io0_r_func_io03_sel_setf(uint8_t rfuncio03sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO0_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x07000000)) | ((uint32_t)rfuncio03sel << 24));
}

__INLINE uint8_t iomux_func_io0_r_func_io02_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00070000)) >> 16);
}

__INLINE void iomux_func_io0_r_func_io02_sel_setf(uint8_t rfuncio02sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO0_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00070000)) | ((uint32_t)rfuncio02sel << 16));
}

__INLINE uint8_t iomux_func_io0_r_func_io01_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void iomux_func_io0_r_func_io01_sel_setf(uint8_t rfuncio01sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO0_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)rfuncio01sel << 8));
}

__INLINE uint8_t iomux_func_io0_r_func_io00_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void iomux_func_io0_r_func_io00_sel_setf(uint8_t rfuncio00sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO0_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO0_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)rfuncio00sel << 0));
}

 /**
 * @brief FUNC_IO1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:24      r_func_io07_sel   0b0
 *  17:16      r_func_io06_sel   0b0
 *  10:08      r_func_io05_sel   0b0
 *  02:00      r_func_io04_sel   0b0
 * </pre>
 */
#define IOMUX_FUNC_IO1_OFFSET 0x0000001C


__INLINE uint32_t iomux_func_io1_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_func_io1_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_FUNC_IO1_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_FUNC_IO1_R_FUNC_IO07_SEL_MASK                  ((uint32_t)0x03000000)
#define IOMUX_FUNC_IO1_R_FUNC_IO07_SEL_LSB                   24
#define IOMUX_FUNC_IO1_R_FUNC_IO07_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO1_R_FUNC_IO06_SEL_MASK                  ((uint32_t)0x00030000)
#define IOMUX_FUNC_IO1_R_FUNC_IO06_SEL_LSB                   16
#define IOMUX_FUNC_IO1_R_FUNC_IO06_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO1_R_FUNC_IO05_SEL_MASK                  ((uint32_t)0x00000700)
#define IOMUX_FUNC_IO1_R_FUNC_IO05_SEL_LSB                   8
#define IOMUX_FUNC_IO1_R_FUNC_IO05_SEL_WIDTH                 ((uint32_t)0x00000003)
#define IOMUX_FUNC_IO1_R_FUNC_IO04_SEL_MASK                  ((uint32_t)0x00000007)
#define IOMUX_FUNC_IO1_R_FUNC_IO04_SEL_LSB                   0
#define IOMUX_FUNC_IO1_R_FUNC_IO04_SEL_WIDTH                 ((uint32_t)0x00000003)

#define IOMUX_FUNC_IO1_R_FUNC_IO07_SEL_RST                   0x0
#define IOMUX_FUNC_IO1_R_FUNC_IO06_SEL_RST                   0x0
#define IOMUX_FUNC_IO1_R_FUNC_IO05_SEL_RST                   0x0
#define IOMUX_FUNC_IO1_R_FUNC_IO04_SEL_RST                   0x0

__INLINE void iomux_func_io1_pack(uint8_t rfuncio07sel, uint8_t rfuncio06sel, uint8_t rfuncio05sel, uint8_t rfuncio04sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO1_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)rfuncio07sel << 24) | ((uint32_t)rfuncio06sel << 16) | ((uint32_t)rfuncio05sel << 8) | ((uint32_t)rfuncio04sel << 0));
}

__INLINE void iomux_func_io1_unpack(uint8_t* rfuncio07sel, uint8_t* rfuncio06sel, uint8_t* rfuncio05sel, uint8_t* rfuncio04sel)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR);

    *rfuncio07sel = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rfuncio06sel = (localVal & ((uint32_t)0x00030000)) >> 16;
    *rfuncio05sel = (localVal & ((uint32_t)0x00000700)) >> 8;
    *rfuncio04sel = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t iomux_func_io1_r_func_io07_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void iomux_func_io1_r_func_io07_sel_setf(uint8_t rfuncio07sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO1_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rfuncio07sel << 24));
}

__INLINE uint8_t iomux_func_io1_r_func_io06_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void iomux_func_io1_r_func_io06_sel_setf(uint8_t rfuncio06sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO1_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)rfuncio06sel << 16));
}

__INLINE uint8_t iomux_func_io1_r_func_io05_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000700)) >> 8);
}

__INLINE void iomux_func_io1_r_func_io05_sel_setf(uint8_t rfuncio05sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO1_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000700)) | ((uint32_t)rfuncio05sel << 8));
}

__INLINE uint8_t iomux_func_io1_r_func_io04_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void iomux_func_io1_r_func_io04_sel_setf(uint8_t rfuncio04sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO1_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO1_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000007)) | ((uint32_t)rfuncio04sel << 0));
}

 /**
 * @brief FUNC_IO2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:24      r_func_io11_sel   0b0
 *  17:16      r_func_io10_sel   0b0
 *  09:08      r_func_io09_sel   0b0
 *  01:00      r_func_io08_sel   0b0
 * </pre>
 */
#define IOMUX_FUNC_IO2_OFFSET 0x00000020


__INLINE uint32_t iomux_func_io2_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_func_io2_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_FUNC_IO2_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_FUNC_IO2_R_FUNC_IO11_SEL_MASK                  ((uint32_t)0x03000000)
#define IOMUX_FUNC_IO2_R_FUNC_IO11_SEL_LSB                   24
#define IOMUX_FUNC_IO2_R_FUNC_IO11_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO2_R_FUNC_IO10_SEL_MASK                  ((uint32_t)0x00030000)
#define IOMUX_FUNC_IO2_R_FUNC_IO10_SEL_LSB                   16
#define IOMUX_FUNC_IO2_R_FUNC_IO10_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO2_R_FUNC_IO09_SEL_MASK                  ((uint32_t)0x00000300)
#define IOMUX_FUNC_IO2_R_FUNC_IO09_SEL_LSB                   8
#define IOMUX_FUNC_IO2_R_FUNC_IO09_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO2_R_FUNC_IO08_SEL_MASK                  ((uint32_t)0x00000003)
#define IOMUX_FUNC_IO2_R_FUNC_IO08_SEL_LSB                   0
#define IOMUX_FUNC_IO2_R_FUNC_IO08_SEL_WIDTH                 ((uint32_t)0x00000002)

#define IOMUX_FUNC_IO2_R_FUNC_IO11_SEL_RST                   0x0
#define IOMUX_FUNC_IO2_R_FUNC_IO10_SEL_RST                   0x0
#define IOMUX_FUNC_IO2_R_FUNC_IO09_SEL_RST                   0x0
#define IOMUX_FUNC_IO2_R_FUNC_IO08_SEL_RST                   0x0

__INLINE void iomux_func_io2_pack(uint8_t rfuncio11sel, uint8_t rfuncio10sel, uint8_t rfuncio09sel, uint8_t rfuncio08sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO2_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)rfuncio11sel << 24) | ((uint32_t)rfuncio10sel << 16) | ((uint32_t)rfuncio09sel << 8) | ((uint32_t)rfuncio08sel << 0));
}

__INLINE void iomux_func_io2_unpack(uint8_t* rfuncio11sel, uint8_t* rfuncio10sel, uint8_t* rfuncio09sel, uint8_t* rfuncio08sel)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR);

    *rfuncio11sel = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rfuncio10sel = (localVal & ((uint32_t)0x00030000)) >> 16;
    *rfuncio09sel = (localVal & ((uint32_t)0x00000300)) >> 8;
    *rfuncio08sel = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t iomux_func_io2_r_func_io11_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void iomux_func_io2_r_func_io11_sel_setf(uint8_t rfuncio11sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO2_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rfuncio11sel << 24));
}

__INLINE uint8_t iomux_func_io2_r_func_io10_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void iomux_func_io2_r_func_io10_sel_setf(uint8_t rfuncio10sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO2_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)rfuncio10sel << 16));
}

__INLINE uint8_t iomux_func_io2_r_func_io09_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void iomux_func_io2_r_func_io09_sel_setf(uint8_t rfuncio09sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO2_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)rfuncio09sel << 8));
}

__INLINE uint8_t iomux_func_io2_r_func_io08_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void iomux_func_io2_r_func_io08_sel_setf(uint8_t rfuncio08sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO2_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO2_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)rfuncio08sel << 0));
}

 /**
 * @brief FUNC_IO3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:24      r_func_io15_sel   0b0
 *  17:16      r_func_io14_sel   0b0
 *  09:08      r_func_io13_sel   0b0
 *  01:00      r_func_io12_sel   0b0
 * </pre>
 */
#define IOMUX_FUNC_IO3_OFFSET 0x00000024


__INLINE uint32_t iomux_func_io3_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_func_io3_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_FUNC_IO3_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_FUNC_IO3_R_FUNC_IO15_SEL_MASK                  ((uint32_t)0x03000000)
#define IOMUX_FUNC_IO3_R_FUNC_IO15_SEL_LSB                   24
#define IOMUX_FUNC_IO3_R_FUNC_IO15_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO3_R_FUNC_IO14_SEL_MASK                  ((uint32_t)0x00030000)
#define IOMUX_FUNC_IO3_R_FUNC_IO14_SEL_LSB                   16
#define IOMUX_FUNC_IO3_R_FUNC_IO14_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO3_R_FUNC_IO13_SEL_MASK                  ((uint32_t)0x00000300)
#define IOMUX_FUNC_IO3_R_FUNC_IO13_SEL_LSB                   8
#define IOMUX_FUNC_IO3_R_FUNC_IO13_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO3_R_FUNC_IO12_SEL_MASK                  ((uint32_t)0x00000003)
#define IOMUX_FUNC_IO3_R_FUNC_IO12_SEL_LSB                   0
#define IOMUX_FUNC_IO3_R_FUNC_IO12_SEL_WIDTH                 ((uint32_t)0x00000002)

#define IOMUX_FUNC_IO3_R_FUNC_IO15_SEL_RST                   0x0
#define IOMUX_FUNC_IO3_R_FUNC_IO14_SEL_RST                   0x0
#define IOMUX_FUNC_IO3_R_FUNC_IO13_SEL_RST                   0x0
#define IOMUX_FUNC_IO3_R_FUNC_IO12_SEL_RST                   0x0

__INLINE void iomux_func_io3_pack(uint8_t rfuncio15sel, uint8_t rfuncio14sel, uint8_t rfuncio13sel, uint8_t rfuncio12sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO3_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)rfuncio15sel << 24) | ((uint32_t)rfuncio14sel << 16) | ((uint32_t)rfuncio13sel << 8) | ((uint32_t)rfuncio12sel << 0));
}

__INLINE void iomux_func_io3_unpack(uint8_t* rfuncio15sel, uint8_t* rfuncio14sel, uint8_t* rfuncio13sel, uint8_t* rfuncio12sel)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR);

    *rfuncio15sel = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rfuncio14sel = (localVal & ((uint32_t)0x00030000)) >> 16;
    *rfuncio13sel = (localVal & ((uint32_t)0x00000300)) >> 8;
    *rfuncio12sel = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t iomux_func_io3_r_func_io15_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void iomux_func_io3_r_func_io15_sel_setf(uint8_t rfuncio15sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO3_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rfuncio15sel << 24));
}

__INLINE uint8_t iomux_func_io3_r_func_io14_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void iomux_func_io3_r_func_io14_sel_setf(uint8_t rfuncio14sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO3_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)rfuncio14sel << 16));
}

__INLINE uint8_t iomux_func_io3_r_func_io13_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void iomux_func_io3_r_func_io13_sel_setf(uint8_t rfuncio13sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO3_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)rfuncio13sel << 8));
}

__INLINE uint8_t iomux_func_io3_r_func_io12_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void iomux_func_io3_r_func_io12_sel_setf(uint8_t rfuncio12sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO3_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO3_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)rfuncio12sel << 0));
}

 /**
 * @brief FUNC_IO4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  25:24      r_func_io19_sel   0b0
 *  17:16      r_func_io18_sel   0b0
 *  09:08      r_func_io17_sel   0b0
 *  01:00      r_func_io16_sel   0b0
 * </pre>
 */
#define IOMUX_FUNC_IO4_OFFSET 0x00000028


__INLINE uint32_t iomux_func_io4_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_func_io4_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_FUNC_IO4_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_FUNC_IO4_R_FUNC_IO19_SEL_MASK                  ((uint32_t)0x03000000)
#define IOMUX_FUNC_IO4_R_FUNC_IO19_SEL_LSB                   24
#define IOMUX_FUNC_IO4_R_FUNC_IO19_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO4_R_FUNC_IO18_SEL_MASK                  ((uint32_t)0x00030000)
#define IOMUX_FUNC_IO4_R_FUNC_IO18_SEL_LSB                   16
#define IOMUX_FUNC_IO4_R_FUNC_IO18_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO4_R_FUNC_IO17_SEL_MASK                  ((uint32_t)0x00000300)
#define IOMUX_FUNC_IO4_R_FUNC_IO17_SEL_LSB                   8
#define IOMUX_FUNC_IO4_R_FUNC_IO17_SEL_WIDTH                 ((uint32_t)0x00000002)
#define IOMUX_FUNC_IO4_R_FUNC_IO16_SEL_MASK                  ((uint32_t)0x00000003)
#define IOMUX_FUNC_IO4_R_FUNC_IO16_SEL_LSB                   0
#define IOMUX_FUNC_IO4_R_FUNC_IO16_SEL_WIDTH                 ((uint32_t)0x00000002)

#define IOMUX_FUNC_IO4_R_FUNC_IO19_SEL_RST                   0x0
#define IOMUX_FUNC_IO4_R_FUNC_IO18_SEL_RST                   0x0
#define IOMUX_FUNC_IO4_R_FUNC_IO17_SEL_RST                   0x0
#define IOMUX_FUNC_IO4_R_FUNC_IO16_SEL_RST                   0x0

__INLINE void iomux_func_io4_pack(uint8_t rfuncio19sel, uint8_t rfuncio18sel, uint8_t rfuncio17sel, uint8_t rfuncio16sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO4_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)rfuncio19sel << 24) | ((uint32_t)rfuncio18sel << 16) | ((uint32_t)rfuncio17sel << 8) | ((uint32_t)rfuncio16sel << 0));
}

__INLINE void iomux_func_io4_unpack(uint8_t* rfuncio19sel, uint8_t* rfuncio18sel, uint8_t* rfuncio17sel, uint8_t* rfuncio16sel)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR);

    *rfuncio19sel = (localVal & ((uint32_t)0x03000000)) >> 24;
    *rfuncio18sel = (localVal & ((uint32_t)0x00030000)) >> 16;
    *rfuncio17sel = (localVal & ((uint32_t)0x00000300)) >> 8;
    *rfuncio16sel = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t iomux_func_io4_r_func_io19_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x03000000)) >> 24);
}

__INLINE void iomux_func_io4_r_func_io19_sel_setf(uint8_t rfuncio19sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO4_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x03000000)) | ((uint32_t)rfuncio19sel << 24));
}

__INLINE uint8_t iomux_func_io4_r_func_io18_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00030000)) >> 16);
}

__INLINE void iomux_func_io4_r_func_io18_sel_setf(uint8_t rfuncio18sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO4_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00030000)) | ((uint32_t)rfuncio18sel << 16));
}

__INLINE uint8_t iomux_func_io4_r_func_io17_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000300)) >> 8);
}

__INLINE void iomux_func_io4_r_func_io17_sel_setf(uint8_t rfuncio17sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO4_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000300)) | ((uint32_t)rfuncio17sel << 8));
}

__INLINE uint8_t iomux_func_io4_r_func_io16_sel_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

__INLINE void iomux_func_io4_r_func_io16_sel_setf(uint8_t rfuncio16sel)
{
    _PICO_REG_WR(IOMUX_FUNC_IO4_OFFSET+ IOMUX_BASE_ADDR, (_PICO_REG_RD(IOMUX_FUNC_IO4_OFFSET + IOMUX_BASE_ADDR) & ~((uint32_t)0x00000003)) | ((uint32_t)rfuncio16sel << 0));
}

 /**
 * @brief FUNC_IO5 register definition
 */
#define IOMUX_FUNC_IO5_OFFSET 0x0000002C


__INLINE uint32_t iomux_func_io5_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO5_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_FUNC_IO5_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_FUNC_IO5_RESERVED_LSB                          0
#define IOMUX_FUNC_IO5_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_FUNC_IO5_RESERVED_RST                          0x0

__INLINE void iomux_func_io5_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO5_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief FUNC_IO6 register definition
 */
#define IOMUX_FUNC_IO6_OFFSET 0x00000030


__INLINE uint32_t iomux_func_io6_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO6_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_FUNC_IO6_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_FUNC_IO6_RESERVED_LSB                          0
#define IOMUX_FUNC_IO6_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_FUNC_IO6_RESERVED_RST                          0x0

__INLINE void iomux_func_io6_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO6_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief FUNC_IO7 register definition
 */
#define IOMUX_FUNC_IO7_OFFSET 0x00000034


__INLINE uint32_t iomux_func_io7_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO7_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_FUNC_IO7_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_FUNC_IO7_RESERVED_LSB                          0
#define IOMUX_FUNC_IO7_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_FUNC_IO7_RESERVED_RST                          0x0

__INLINE void iomux_func_io7_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO7_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief FUNC_IO8 register definition
 */
#define IOMUX_FUNC_IO8_OFFSET 0x00000038


__INLINE uint32_t iomux_func_io8_get(void)
{
    return _PICO_REG_RD(IOMUX_FUNC_IO8_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_FUNC_IO8_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_FUNC_IO8_RESERVED_LSB                          0
#define IOMUX_FUNC_IO8_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_FUNC_IO8_RESERVED_RST                          0x0

__INLINE void iomux_func_io8_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_FUNC_IO8_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief PAD_PE0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00              r_io_pe   0b0
 * </pre>
 */
#define IOMUX_PAD_PE0_OFFSET 0x0000003C


__INLINE uint32_t iomux_pad_pe0_get(void)
{
    return _PICO_REG_RD(IOMUX_PAD_PE0_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_pad_pe0_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_PAD_PE0_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_PAD_PE0_R_IO_PE_MASK                          ((uint32_t)0x000FFFFF)
#define IOMUX_PAD_PE0_R_IO_PE_LSB                           0
#define IOMUX_PAD_PE0_R_IO_PE_WIDTH                         ((uint32_t)0x00000014)

#define IOMUX_PAD_PE0_R_IO_PE_RST                           0x0

__INLINE void iomux_pad_pe0_pack(uint32_t riope)
{
    _PICO_REG_WR(IOMUX_PAD_PE0_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)riope << 0));
}

__INLINE void iomux_pad_pe0_unpack(uint8_t* riope)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_PAD_PE0_OFFSET + IOMUX_BASE_ADDR);

    *riope = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t iomux_pad_pe0_r_io_pe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_PAD_PE0_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void iomux_pad_pe0_r_io_pe_setf(uint32_t riope)
{
    _PICO_REG_WR(IOMUX_PAD_PE0_OFFSET+ IOMUX_BASE_ADDR, (uint32_t)riope << 0);
}

 /**
 * @brief PAD_PE1 register definition
 */
#define IOMUX_PAD_PE1_OFFSET 0x00000040


__INLINE uint32_t iomux_pad_pe1_get(void)
{
    return _PICO_REG_RD(IOMUX_PAD_PE1_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_PAD_PE1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_PAD_PE1_RESERVED_LSB                          0
#define IOMUX_PAD_PE1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_PAD_PE1_RESERVED_RST                          0x0

__INLINE void iomux_pad_pe1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_PAD_PE1_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief PAD_PS0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  19:00              r_io_ds   0b0
 * </pre>
 */
#define IOMUX_PAD_PS0_OFFSET 0x00000044


__INLINE uint32_t iomux_pad_ps0_get(void)
{
    return _PICO_REG_RD(IOMUX_PAD_PS0_OFFSET + IOMUX_BASE_ADDR);
}

__INLINE void iomux_pad_ps0_set(uint32_t value)
{
    _PICO_REG_WR(IOMUX_PAD_PS0_OFFSET+ IOMUX_BASE_ADDR, value);
}

// field definitions
#define IOMUX_PAD_PS0_R_IO_DS_MASK                          ((uint32_t)0x000FFFFF)
#define IOMUX_PAD_PS0_R_IO_DS_LSB                           0
#define IOMUX_PAD_PS0_R_IO_DS_WIDTH                         ((uint32_t)0x00000014)

#define IOMUX_PAD_PS0_R_IO_DS_RST                           0x0

__INLINE void iomux_pad_ps0_pack(uint32_t riods)
{
    _PICO_REG_WR(IOMUX_PAD_PS0_OFFSET+ IOMUX_BASE_ADDR,  ((uint32_t)riods << 0));
}

__INLINE void iomux_pad_ps0_unpack(uint8_t* riods)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_PAD_PS0_OFFSET + IOMUX_BASE_ADDR);

    *riods = (localVal & ((uint32_t)0x000FFFFF)) >> 0;
}

__INLINE uint32_t iomux_pad_ps0_r_io_ds_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_PAD_PS0_OFFSET + IOMUX_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000FFFFF)) >> 0);
}

__INLINE void iomux_pad_ps0_r_io_ds_setf(uint32_t riods)
{
    _PICO_REG_WR(IOMUX_PAD_PS0_OFFSET+ IOMUX_BASE_ADDR, (uint32_t)riods << 0);
}

 /**
 * @brief PAD_PS1 register definition
 */
#define IOMUX_PAD_PS1_OFFSET 0x00000048


__INLINE uint32_t iomux_pad_ps1_get(void)
{
    return _PICO_REG_RD(IOMUX_PAD_PS1_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_PAD_PS1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_PAD_PS1_RESERVED_LSB                          0
#define IOMUX_PAD_PS1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_PAD_PS1_RESERVED_RST                          0x0

__INLINE void iomux_pad_ps1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_PAD_PS1_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief KEY_SCAN_IN_EN register definition
 */
#define IOMUX_KEY_SCAN_IN_EN_OFFSET 0x0000004C


__INLINE uint32_t iomux_key_scan_in_en_get(void)
{
    return _PICO_REG_RD(IOMUX_KEY_SCAN_IN_EN_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_KEY_SCAN_IN_EN_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_KEY_SCAN_IN_EN_RESERVED_LSB                          0
#define IOMUX_KEY_SCAN_IN_EN_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_KEY_SCAN_IN_EN_RESERVED_RST                          0x0

__INLINE void iomux_key_scan_in_en_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_KEY_SCAN_IN_EN_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief KEY_SCAN_OUT_EN register definition
 */
#define IOMUX_KEY_SCAN_OUT_EN_OFFSET 0x00000050


__INLINE uint32_t iomux_key_scan_out_en_get(void)
{
    return _PICO_REG_RD(IOMUX_KEY_SCAN_OUT_EN_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_KEY_SCAN_OUT_EN_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_KEY_SCAN_OUT_EN_RESERVED_LSB                          0
#define IOMUX_KEY_SCAN_OUT_EN_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_KEY_SCAN_OUT_EN_RESERVED_RST                          0x0

__INLINE void iomux_key_scan_out_en_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_KEY_SCAN_OUT_EN_OFFSET + IOMUX_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief PTM_DS_PE register definition
 */
#define IOMUX_PTM_DS_PE_OFFSET 0x00000080


__INLINE uint32_t iomux_ptm_ds_pe_get(void)
{
    return _PICO_REG_RD(IOMUX_PTM_DS_PE_OFFSET + IOMUX_BASE_ADDR);
}

// field definitions
#define IOMUX_PTM_DS_PE_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define IOMUX_PTM_DS_PE_RESERVED_LSB                          0
#define IOMUX_PTM_DS_PE_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define IOMUX_PTM_DS_PE_RESERVED_RST                          0x0

__INLINE void iomux_ptm_ds_pe_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(IOMUX_PTM_DS_PE_OFFSET + IOMUX_BASE_ADDR);

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
      __IO uint32_t :32;
    }r_analog_io_fld;
    __IO uint32_t r_analog_io;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :32;
    }spi_debug_fld;
    __IO uint32_t spi_debug;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :32;
    }debug_mux_fld;
    __IO uint32_t debug_mux;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :12;
      __IO uint32_t r_func_io_en:20;
    }full_mux0_fld;
    __IO uint32_t full_mux0;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :32;
    }full_mux1_fld;
    __IO uint32_t full_mux1;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :32;
    }gpio_papb_fld;
    __IO uint32_t gpio_papb;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :5;
      __IO uint32_t r_func_io03_sel:3;
      __IO uint32_t :5;
      __IO uint32_t r_func_io02_sel:3;
      __IO uint32_t :6;
      __IO uint32_t r_func_io01_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io00_sel:2;
    }func_io0_fld;
    __IO uint32_t func_io0;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :6;
      __IO uint32_t r_func_io07_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io06_sel:2;
      __IO uint32_t :5;
      __IO uint32_t r_func_io05_sel:3;
      __IO uint32_t :5;
      __IO uint32_t r_func_io04_sel:3;
    }func_io1_fld;
    __IO uint32_t func_io1;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t :6;
      __IO uint32_t r_func_io11_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io10_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io09_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io08_sel:2;
    }func_io2_fld;
    __IO uint32_t func_io2;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :6;
      __IO uint32_t r_func_io15_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io14_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io13_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io12_sel:2;
    }func_io3_fld;
    __IO uint32_t func_io3;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t :6;
      __IO uint32_t r_func_io19_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io18_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io17_sel:2;
      __IO uint32_t :6;
      __IO uint32_t r_func_io16_sel:2;
    }func_io4_fld;
    __IO uint32_t func_io4;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t :32;
    }func_io5_fld;
    __IO uint32_t func_io5;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :32;
    }func_io6_fld;
    __IO uint32_t func_io6;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t :32;
    }func_io7_fld;
    __IO uint32_t func_io7;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t :32;
    }func_io8_fld;
    __IO uint32_t func_io8;
  };

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t :12;
      __IO uint32_t r_io_pe:20;
    }pad_pe0_fld;
    __IO uint32_t pad_pe0;
  };

  union{ //offset addr 0x0040
    struct{
      __IO uint32_t :32;
    }pad_pe1_fld;
    __IO uint32_t pad_pe1;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t :12;
      __IO uint32_t r_io_ds:20;
    }pad_ps0_fld;
    __IO uint32_t pad_ps0;
  };

  union{ //offset addr 0x0048
    struct{
      __IO uint32_t :32;
    }pad_ps1_fld;
    __IO uint32_t pad_ps1;
  };

  union{ //offset addr 0x004c
    struct{
      __IO uint32_t :32;
    }key_scan_in_en_fld;
    __IO uint32_t key_scan_in_en;
  };

  union{ //offset addr 0x0050
    struct{
      __IO uint32_t :32;
    }key_scan_out_en_fld;
    __IO uint32_t key_scan_out_en;
  };


  union{ //offset addr 0x0080
    struct{
      __IO uint32_t :32;
    }ptm_ds_pe_fld;
    __IO uint32_t ptm_ds_pe;
  };

} PICO_REG_IOMUX_TypeDef;

#define PICO_REG_IOMUX PICO_REG_IOMUX_TypeDef *0x40003800


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_IOMUX_H_

