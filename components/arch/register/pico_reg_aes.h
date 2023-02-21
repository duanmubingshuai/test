#ifndef _PICO_REG_AES_H_
#define _PICO_REG_AES_H_

#include <stdint.h>

#define AES_COUNT 21

#define AES_BASE_ADDR 0x40040000

#define AES_SIZE 0x00000100


 /**
 * @brief LAYER_ENABLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00               Enable   0
 * </pre>
 */
#define AES_LAYER_ENABLE_OFFSET 0x00000000


__INLINE uint32_t aes_layer_enable_get(void)
{
    return _PICO_REG_RD(AES_LAYER_ENABLE_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_layer_enable_set(uint32_t value)
{
    _PICO_REG_WR(AES_LAYER_ENABLE_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_LAYER_ENABLE_ENABLE_BIT                            ((uint32_t)0x00000001)
#define AES_LAYER_ENABLE_ENABLE_POS                            0

#define AES_LAYER_ENABLE_ENABLE_RST                            0x0

__INLINE void aes_layer_enable_pack(uint8_t enable)
{
    _PICO_REG_WR(AES_LAYER_ENABLE_OFFSET+ AES_BASE_ADDR,  ((uint32_t)enable << 0));
}

__INLINE void aes_layer_enable_unpack(uint8_t* enable)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_LAYER_ENABLE_OFFSET + AES_BASE_ADDR);

    *enable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t aes_layer_enable_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_LAYER_ENABLE_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void aes_layer_enable_enable_setf(uint8_t enable)
{
    _PICO_REG_WR(AES_LAYER_ENABLE_OFFSET+ AES_BASE_ADDR, (uint32_t)enable << 0);
}

 /**
 * @brief LAYER_CONTROL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     16          FIFO_IO_PDU   0
 *  11:08       Enginne_revert   0b0
 *     04          single_mode   0
 *     03            Code_mode   0
 *  02:00             reserved   0b0
 * </pre>
 */
#define AES_LAYER_CONTROL_OFFSET 0x00000004


__INLINE uint32_t aes_layer_control_get(void)
{
    return _PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_layer_control_set(uint32_t value)
{
    _PICO_REG_WR(AES_LAYER_CONTROL_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_LAYER_CONTROL_FIFO_IO_PDU_BIT                       ((uint32_t)0x00010000)
#define AES_LAYER_CONTROL_FIFO_IO_PDU_POS                       16
#define AES_LAYER_CONTROL_ENGINNE_REVERT_MASK                   ((uint32_t)0x00000F00)
#define AES_LAYER_CONTROL_ENGINNE_REVERT_LSB                    8
#define AES_LAYER_CONTROL_ENGINNE_REVERT_WIDTH                  ((uint32_t)0x00000004)
#define AES_LAYER_CONTROL_SINGLE_MODE_BIT                       ((uint32_t)0x00000010)
#define AES_LAYER_CONTROL_SINGLE_MODE_POS                       4
#define AES_LAYER_CONTROL_CODE_MODE_BIT                         ((uint32_t)0x00000008)
#define AES_LAYER_CONTROL_CODE_MODE_POS                         3
#define AES_LAYER_CONTROL_RESERVED_MASK                         ((uint32_t)0x00000007)
#define AES_LAYER_CONTROL_RESERVED_LSB                          0
#define AES_LAYER_CONTROL_RESERVED_WIDTH                        ((uint32_t)0x00000003)

#define AES_LAYER_CONTROL_FIFO_IO_PDU_RST                       0x0
#define AES_LAYER_CONTROL_ENGINNE_REVERT_RST                    0x0
#define AES_LAYER_CONTROL_SINGLE_MODE_RST                       0x0
#define AES_LAYER_CONTROL_CODE_MODE_RST                         0x0
#define AES_LAYER_CONTROL_RESERVED_RST                          0x0

__INLINE void aes_layer_control_pack(uint8_t fifoiopdu, uint8_t enginnerevert, uint8_t singlemode, uint8_t codemode)
{
    _PICO_REG_WR(AES_LAYER_CONTROL_OFFSET+ AES_BASE_ADDR,  ((uint32_t)fifoiopdu << 16) | ((uint32_t)enginnerevert << 8) | ((uint32_t)singlemode << 4) | ((uint32_t)codemode << 3));
}

__INLINE void aes_layer_control_unpack(uint8_t* fifoiopdu, uint8_t* enginnerevert, uint8_t* singlemode, uint8_t* codemode, uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR);

    *fifoiopdu = (localVal & ((uint32_t)0x00010000)) >> 16;
    *enginnerevert = (localVal & ((uint32_t)0x00000F00)) >> 8;
    *singlemode = (localVal & ((uint32_t)0x00000010)) >> 4;
    *codemode = (localVal & ((uint32_t)0x00000008)) >> 3;
    *reserved = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t aes_layer_control_fifo_io_pdu_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE void aes_layer_control_fifo_io_pdu_setf(uint8_t fifoiopdu)
{
    _PICO_REG_WR(AES_LAYER_CONTROL_OFFSET+ AES_BASE_ADDR, (_PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR) & ~((uint32_t)0x00010000)) | ((uint32_t)fifoiopdu << 16));
}

__INLINE uint8_t aes_layer_control_enginne_revert_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000F00)) >> 8);
}

__INLINE void aes_layer_control_enginne_revert_setf(uint8_t enginnerevert)
{
    _PICO_REG_WR(AES_LAYER_CONTROL_OFFSET+ AES_BASE_ADDR, (_PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR) & ~((uint32_t)0x00000F00)) | ((uint32_t)enginnerevert << 8));
}

__INLINE uint8_t aes_layer_control_single_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void aes_layer_control_single_mode_setf(uint8_t singlemode)
{
    _PICO_REG_WR(AES_LAYER_CONTROL_OFFSET+ AES_BASE_ADDR, (_PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)singlemode << 4));
}

__INLINE uint8_t aes_layer_control_code_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void aes_layer_control_code_mode_setf(uint8_t codemode)
{
    _PICO_REG_WR(AES_LAYER_CONTROL_OFFSET+ AES_BASE_ADDR, (_PICO_REG_RD(AES_LAYER_CONTROL_OFFSET + AES_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)codemode << 3));
}

 /**
 * @brief RESERVED0 register definition
 */
#define AES_RESERVED0_OFFSET 0x00000008


__INLINE uint32_t aes_reserved0_get(void)
{
    return _PICO_REG_RD(AES_RESERVED0_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_RESERVED0_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AES_RESERVED0_RESERVED_LSB                          0
#define AES_RESERVED0_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AES_RESERVED0_RESERVED_RST                          0x0

__INLINE void aes_reserved0_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_RESERVED0_OFFSET + AES_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief PLEN_AND_AAD register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08                 plen   0b0
 *  07:00                  aad   0b0
 * </pre>
 */
#define AES_PLEN_AND_AAD_OFFSET 0x0000000C


__INLINE uint32_t aes_plen_and_aad_get(void)
{
    return _PICO_REG_RD(AES_PLEN_AND_AAD_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_plen_and_aad_set(uint32_t value)
{
    _PICO_REG_WR(AES_PLEN_AND_AAD_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_PLEN_AND_AAD_PLEN_MASK                             ((uint32_t)0x0000FF00)
#define AES_PLEN_AND_AAD_PLEN_LSB                              8
#define AES_PLEN_AND_AAD_PLEN_WIDTH                            ((uint32_t)0x00000008)
#define AES_PLEN_AND_AAD_AAD_MASK                              ((uint32_t)0x000000FF)
#define AES_PLEN_AND_AAD_AAD_LSB                               0
#define AES_PLEN_AND_AAD_AAD_WIDTH                             ((uint32_t)0x00000008)

#define AES_PLEN_AND_AAD_PLEN_RST                              0x0
#define AES_PLEN_AND_AAD_AAD_RST                               0x0

__INLINE void aes_plen_and_aad_pack(uint8_t plen, uint8_t aad)
{
    _PICO_REG_WR(AES_PLEN_AND_AAD_OFFSET+ AES_BASE_ADDR,  ((uint32_t)plen << 8) | ((uint32_t)aad << 0));
}

__INLINE void aes_plen_and_aad_unpack(uint8_t* plen, uint8_t* aad)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_PLEN_AND_AAD_OFFSET + AES_BASE_ADDR);

    *plen = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *aad = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t aes_plen_and_aad_plen_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_PLEN_AND_AAD_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE void aes_plen_and_aad_plen_setf(uint8_t plen)
{
    _PICO_REG_WR(AES_PLEN_AND_AAD_OFFSET+ AES_BASE_ADDR, (_PICO_REG_RD(AES_PLEN_AND_AAD_OFFSET + AES_BASE_ADDR) & ~((uint32_t)0x0000FF00)) | ((uint32_t)plen << 8));
}

__INLINE uint8_t aes_plen_and_aad_aad_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_PLEN_AND_AAD_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void aes_plen_and_aad_aad_setf(uint8_t aad)
{
    _PICO_REG_WR(AES_PLEN_AND_AAD_OFFSET+ AES_BASE_ADDR, (_PICO_REG_RD(AES_PLEN_AND_AAD_OFFSET + AES_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)aad << 0));
}

 /**
 * @brief INTERRUPT_MASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00     interupt_enable_   0b0
 * </pre>
 */
#define AES_INTERRUPT_MASK_OFFSET 0x00000010


__INLINE uint32_t aes_interrupt_mask_get(void)
{
    return _PICO_REG_RD(AES_INTERRUPT_MASK_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_interrupt_mask_set(uint32_t value)
{
    _PICO_REG_WR(AES_INTERRUPT_MASK_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_INTERRUPT_MASK_INTERUPT_ENABLE__MASK                 ((uint32_t)0x0000000F)
#define AES_INTERRUPT_MASK_INTERUPT_ENABLE__LSB                  0
#define AES_INTERRUPT_MASK_INTERUPT_ENABLE__WIDTH                ((uint32_t)0x00000004)

#define AES_INTERRUPT_MASK_INTERUPT_ENABLE__RST                  0x0

__INLINE void aes_interrupt_mask_pack(uint8_t interuptenable)
{
    _PICO_REG_WR(AES_INTERRUPT_MASK_OFFSET+ AES_BASE_ADDR,  ((uint32_t)interuptenable << 0));
}

__INLINE void aes_interrupt_mask_unpack(uint8_t* interuptenable)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_INTERRUPT_MASK_OFFSET + AES_BASE_ADDR);

    *interuptenable = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t aes_interrupt_mask_interupt_enable__getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_INTERRUPT_MASK_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

__INLINE void aes_interrupt_mask_interupt_enable__setf(uint8_t interuptenable)
{
    _PICO_REG_WR(AES_INTERRUPT_MASK_OFFSET+ AES_BASE_ADDR, (uint32_t)interuptenable << 0);
}

 /**
 * @brief INTERRUPT_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00      interupt_status   0b0
 * </pre>
 */
#define AES_INTERRUPT_STATUS_OFFSET 0x00000014


__INLINE uint32_t aes_interrupt_status_get(void)
{
    return _PICO_REG_RD(AES_INTERRUPT_STATUS_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_INTERRUPT_STATUS_INTERUPT_STATUS_MASK                  ((uint32_t)0x0000000F)
#define AES_INTERRUPT_STATUS_INTERUPT_STATUS_LSB                   0
#define AES_INTERRUPT_STATUS_INTERUPT_STATUS_WIDTH                 ((uint32_t)0x00000004)

#define AES_INTERRUPT_STATUS_INTERUPT_STATUS_RST                   0x0

__INLINE void aes_interrupt_status_unpack(uint8_t* interuptstatus)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_INTERRUPT_STATUS_OFFSET + AES_BASE_ADDR);

    *interuptstatus = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t aes_interrupt_status_interupt_status_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_INTERRUPT_STATUS_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief RESERVED1 register definition
 */
#define AES_RESERVED1_OFFSET 0x00000018


__INLINE uint32_t aes_reserved1_get(void)
{
    return _PICO_REG_RD(AES_RESERVED1_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_RESERVED1_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AES_RESERVED1_RESERVED_LSB                          0
#define AES_RESERVED1_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AES_RESERVED1_RESERVED_RST                          0x0

__INLINE void aes_reserved1_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_RESERVED1_OFFSET + AES_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief RESERVED2 register definition
 */
#define AES_RESERVED2_OFFSET 0x0000001C


__INLINE uint32_t aes_reserved2_get(void)
{
    return _PICO_REG_RD(AES_RESERVED2_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_RESERVED2_RESERVED_MASK                         ((uint32_t)0xFFFFFFFF)
#define AES_RESERVED2_RESERVED_LSB                          0
#define AES_RESERVED2_RESERVED_WIDTH                        ((uint32_t)0x00000020)

#define AES_RESERVED2_RESERVED_RST                          0x0

__INLINE void aes_reserved2_unpack(uint8_t* reserved)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_RESERVED2_OFFSET + AES_BASE_ADDR);

    *reserved = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

 /**
 * @brief KEY0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                 Key0   0b0
 * </pre>
 */
#define AES_KEY0_OFFSET 0x00000020


__INLINE uint32_t aes_key0_get(void)
{
    return _PICO_REG_RD(AES_KEY0_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_key0_set(uint32_t value)
{
    _PICO_REG_WR(AES_KEY0_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_KEY0_KEY0_MASK                             ((uint32_t)0xFFFFFFFF)
#define AES_KEY0_KEY0_LSB                              0
#define AES_KEY0_KEY0_WIDTH                            ((uint32_t)0x00000020)

#define AES_KEY0_KEY0_RST                              0x0

__INLINE void aes_key0_pack(uint32_t key0)
{
    _PICO_REG_WR(AES_KEY0_OFFSET+ AES_BASE_ADDR,  ((uint32_t)key0 << 0));
}

__INLINE void aes_key0_unpack(uint8_t* key0)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY0_OFFSET + AES_BASE_ADDR);

    *key0 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_key0_key0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY0_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_key0_key0_setf(uint32_t key0)
{
    _PICO_REG_WR(AES_KEY0_OFFSET+ AES_BASE_ADDR, (uint32_t)key0 << 0);
}

 /**
 * @brief KEY1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                 Key1   0b0
 * </pre>
 */
#define AES_KEY1_OFFSET 0x00000024


__INLINE uint32_t aes_key1_get(void)
{
    return _PICO_REG_RD(AES_KEY1_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_key1_set(uint32_t value)
{
    _PICO_REG_WR(AES_KEY1_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_KEY1_KEY1_MASK                             ((uint32_t)0xFFFFFFFF)
#define AES_KEY1_KEY1_LSB                              0
#define AES_KEY1_KEY1_WIDTH                            ((uint32_t)0x00000020)

#define AES_KEY1_KEY1_RST                              0x0

__INLINE void aes_key1_pack(uint32_t key1)
{
    _PICO_REG_WR(AES_KEY1_OFFSET+ AES_BASE_ADDR,  ((uint32_t)key1 << 0));
}

__INLINE void aes_key1_unpack(uint8_t* key1)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY1_OFFSET + AES_BASE_ADDR);

    *key1 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_key1_key1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY1_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_key1_key1_setf(uint32_t key1)
{
    _PICO_REG_WR(AES_KEY1_OFFSET+ AES_BASE_ADDR, (uint32_t)key1 << 0);
}

 /**
 * @brief KEY2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                 Key2   0b0
 * </pre>
 */
#define AES_KEY2_OFFSET 0x00000028


__INLINE uint32_t aes_key2_get(void)
{
    return _PICO_REG_RD(AES_KEY2_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_key2_set(uint32_t value)
{
    _PICO_REG_WR(AES_KEY2_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_KEY2_KEY2_MASK                             ((uint32_t)0xFFFFFFFF)
#define AES_KEY2_KEY2_LSB                              0
#define AES_KEY2_KEY2_WIDTH                            ((uint32_t)0x00000020)

#define AES_KEY2_KEY2_RST                              0x0

__INLINE void aes_key2_pack(uint32_t key2)
{
    _PICO_REG_WR(AES_KEY2_OFFSET+ AES_BASE_ADDR,  ((uint32_t)key2 << 0));
}

__INLINE void aes_key2_unpack(uint8_t* key2)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY2_OFFSET + AES_BASE_ADDR);

    *key2 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_key2_key2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY2_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_key2_key2_setf(uint32_t key2)
{
    _PICO_REG_WR(AES_KEY2_OFFSET+ AES_BASE_ADDR, (uint32_t)key2 << 0);
}

 /**
 * @brief KEY3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00                 Key3   0b0
 * </pre>
 */
#define AES_KEY3_OFFSET 0x0000002C


__INLINE uint32_t aes_key3_get(void)
{
    return _PICO_REG_RD(AES_KEY3_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_key3_set(uint32_t value)
{
    _PICO_REG_WR(AES_KEY3_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_KEY3_KEY3_MASK                             ((uint32_t)0xFFFFFFFF)
#define AES_KEY3_KEY3_LSB                              0
#define AES_KEY3_KEY3_WIDTH                            ((uint32_t)0x00000020)

#define AES_KEY3_KEY3_RST                              0x0

__INLINE void aes_key3_pack(uint32_t key3)
{
    _PICO_REG_WR(AES_KEY3_OFFSET+ AES_BASE_ADDR,  ((uint32_t)key3 << 0));
}

__INLINE void aes_key3_unpack(uint8_t* key3)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY3_OFFSET + AES_BASE_ADDR);

    *key3 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_key3_key3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_KEY3_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_key3_key3_setf(uint32_t key3)
{
    _PICO_REG_WR(AES_KEY3_OFFSET+ AES_BASE_ADDR, (uint32_t)key3 << 0);
}

 /**
 * @brief NONCE0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               Nonce0   0b0
 * </pre>
 */
#define AES_NONCE0_OFFSET 0x00000030


__INLINE uint32_t aes_nonce0_get(void)
{
    return _PICO_REG_RD(AES_NONCE0_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_nonce0_set(uint32_t value)
{
    _PICO_REG_WR(AES_NONCE0_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_NONCE0_NONCE0_MASK                           ((uint32_t)0xFFFFFFFF)
#define AES_NONCE0_NONCE0_LSB                            0
#define AES_NONCE0_NONCE0_WIDTH                          ((uint32_t)0x00000020)

#define AES_NONCE0_NONCE0_RST                            0x0

__INLINE void aes_nonce0_pack(uint32_t nonce0)
{
    _PICO_REG_WR(AES_NONCE0_OFFSET+ AES_BASE_ADDR,  ((uint32_t)nonce0 << 0));
}

__INLINE void aes_nonce0_unpack(uint8_t* nonce0)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE0_OFFSET + AES_BASE_ADDR);

    *nonce0 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_nonce0_nonce0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE0_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_nonce0_nonce0_setf(uint32_t nonce0)
{
    _PICO_REG_WR(AES_NONCE0_OFFSET+ AES_BASE_ADDR, (uint32_t)nonce0 << 0);
}

 /**
 * @brief NONCE1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               Nonce1   0b0
 * </pre>
 */
#define AES_NONCE1_OFFSET 0x00000034


__INLINE uint32_t aes_nonce1_get(void)
{
    return _PICO_REG_RD(AES_NONCE1_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_nonce1_set(uint32_t value)
{
    _PICO_REG_WR(AES_NONCE1_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_NONCE1_NONCE1_MASK                           ((uint32_t)0xFFFFFFFF)
#define AES_NONCE1_NONCE1_LSB                            0
#define AES_NONCE1_NONCE1_WIDTH                          ((uint32_t)0x00000020)

#define AES_NONCE1_NONCE1_RST                            0x0

__INLINE void aes_nonce1_pack(uint32_t nonce1)
{
    _PICO_REG_WR(AES_NONCE1_OFFSET+ AES_BASE_ADDR,  ((uint32_t)nonce1 << 0));
}

__INLINE void aes_nonce1_unpack(uint8_t* nonce1)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE1_OFFSET + AES_BASE_ADDR);

    *nonce1 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_nonce1_nonce1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE1_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_nonce1_nonce1_setf(uint32_t nonce1)
{
    _PICO_REG_WR(AES_NONCE1_OFFSET+ AES_BASE_ADDR, (uint32_t)nonce1 << 0);
}

 /**
 * @brief NONCE2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               Nonce2   0b0
 * </pre>
 */
#define AES_NONCE2_OFFSET 0x00000038


__INLINE uint32_t aes_nonce2_get(void)
{
    return _PICO_REG_RD(AES_NONCE2_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_nonce2_set(uint32_t value)
{
    _PICO_REG_WR(AES_NONCE2_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_NONCE2_NONCE2_MASK                           ((uint32_t)0xFFFFFFFF)
#define AES_NONCE2_NONCE2_LSB                            0
#define AES_NONCE2_NONCE2_WIDTH                          ((uint32_t)0x00000020)

#define AES_NONCE2_NONCE2_RST                            0x0

__INLINE void aes_nonce2_pack(uint32_t nonce2)
{
    _PICO_REG_WR(AES_NONCE2_OFFSET+ AES_BASE_ADDR,  ((uint32_t)nonce2 << 0));
}

__INLINE void aes_nonce2_unpack(uint8_t* nonce2)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE2_OFFSET + AES_BASE_ADDR);

    *nonce2 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_nonce2_nonce2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE2_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_nonce2_nonce2_setf(uint32_t nonce2)
{
    _PICO_REG_WR(AES_NONCE2_OFFSET+ AES_BASE_ADDR, (uint32_t)nonce2 << 0);
}

 /**
 * @brief NONCE3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00               Nonce3   0b0
 * </pre>
 */
#define AES_NONCE3_OFFSET 0x0000003C


__INLINE uint32_t aes_nonce3_get(void)
{
    return _PICO_REG_RD(AES_NONCE3_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_nonce3_set(uint32_t value)
{
    _PICO_REG_WR(AES_NONCE3_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_NONCE3_NONCE3_MASK                           ((uint32_t)0xFFFFFFFF)
#define AES_NONCE3_NONCE3_LSB                            0
#define AES_NONCE3_NONCE3_WIDTH                          ((uint32_t)0x00000020)

#define AES_NONCE3_NONCE3_RST                            0x0

__INLINE void aes_nonce3_pack(uint32_t nonce3)
{
    _PICO_REG_WR(AES_NONCE3_OFFSET+ AES_BASE_ADDR,  ((uint32_t)nonce3 << 0));
}

__INLINE void aes_nonce3_unpack(uint8_t* nonce3)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE3_OFFSET + AES_BASE_ADDR);

    *nonce3 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_nonce3_nonce3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_NONCE3_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_nonce3_nonce3_setf(uint32_t nonce3)
{
    _PICO_REG_WR(AES_NONCE3_OFFSET+ AES_BASE_ADDR, (uint32_t)nonce3 << 0);
}

 /**
 * @brief DATA_OUT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00              Data_o0   0b0
 * </pre>
 */
#define AES_DATA_OUT0_OFFSET 0x00000050


__INLINE uint32_t aes_data_out0_get(void)
{
    return _PICO_REG_RD(AES_DATA_OUT0_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_DATA_OUT0_DATA_O0_MASK                          ((uint32_t)0xFFFFFFFF)
#define AES_DATA_OUT0_DATA_O0_LSB                           0
#define AES_DATA_OUT0_DATA_O0_WIDTH                         ((uint32_t)0x00000020)

#define AES_DATA_OUT0_DATA_O0_RST                           0x0

__INLINE void aes_data_out0_unpack(uint8_t* datao0)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT0_OFFSET + AES_BASE_ADDR);

    *datao0 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_data_out0_data_o0_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT0_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief DATA_OUT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00              Data_o1   0b0
 * </pre>
 */
#define AES_DATA_OUT1_OFFSET 0x00000054


__INLINE uint32_t aes_data_out1_get(void)
{
    return _PICO_REG_RD(AES_DATA_OUT1_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_DATA_OUT1_DATA_O1_MASK                          ((uint32_t)0xFFFFFFFF)
#define AES_DATA_OUT1_DATA_O1_LSB                           0
#define AES_DATA_OUT1_DATA_O1_WIDTH                         ((uint32_t)0x00000020)

#define AES_DATA_OUT1_DATA_O1_RST                           0x0

__INLINE void aes_data_out1_unpack(uint8_t* datao1)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT1_OFFSET + AES_BASE_ADDR);

    *datao1 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_data_out1_data_o1_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT1_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief DATA_OUT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00              Data_o2   0b0
 * </pre>
 */
#define AES_DATA_OUT2_OFFSET 0x00000058


__INLINE uint32_t aes_data_out2_get(void)
{
    return _PICO_REG_RD(AES_DATA_OUT2_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_DATA_OUT2_DATA_O2_MASK                          ((uint32_t)0xFFFFFFFF)
#define AES_DATA_OUT2_DATA_O2_LSB                           0
#define AES_DATA_OUT2_DATA_O2_WIDTH                         ((uint32_t)0x00000020)

#define AES_DATA_OUT2_DATA_O2_RST                           0x0

__INLINE void aes_data_out2_unpack(uint8_t* datao2)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT2_OFFSET + AES_BASE_ADDR);

    *datao2 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_data_out2_data_o2_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT2_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief DATA_OUT3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00              Data_o3   0b0
 * </pre>
 */
#define AES_DATA_OUT3_OFFSET 0x0000005C


__INLINE uint32_t aes_data_out3_get(void)
{
    return _PICO_REG_RD(AES_DATA_OUT3_OFFSET + AES_BASE_ADDR);
}

// field definitions
#define AES_DATA_OUT3_DATA_O3_MASK                          ((uint32_t)0xFFFFFFFF)
#define AES_DATA_OUT3_DATA_O3_LSB                           0
#define AES_DATA_OUT3_DATA_O3_WIDTH                         ((uint32_t)0x00000020)

#define AES_DATA_OUT3_DATA_O3_RST                           0x0

__INLINE void aes_data_out3_unpack(uint8_t* datao3)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT3_OFFSET + AES_BASE_ADDR);

    *datao3 = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_data_out3_data_o3_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_DATA_OUT3_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief MEMORY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         memory_write   0b0
 * </pre>
 */
#define AES_MEMORY_OFFSET 0x00000100


__INLINE uint32_t aes_memory_get(void)
{
    return _PICO_REG_RD(AES_MEMORY_OFFSET + AES_BASE_ADDR);
}

__INLINE void aes_memory_set(uint32_t value)
{
    _PICO_REG_WR(AES_MEMORY_OFFSET+ AES_BASE_ADDR, value);
}

// field definitions
#define AES_MEMORY_MEMORY_WRITE_MASK                     ((uint32_t)0xFFFFFFFF)
#define AES_MEMORY_MEMORY_WRITE_LSB                      0
#define AES_MEMORY_MEMORY_WRITE_WIDTH                    ((uint32_t)0x00000020)

#define AES_MEMORY_MEMORY_WRITE_RST                      0x0

__INLINE void aes_memory_pack(uint32_t memorywrite)
{
    _PICO_REG_WR(AES_MEMORY_OFFSET+ AES_BASE_ADDR,  ((uint32_t)memorywrite << 0));
}

__INLINE void aes_memory_unpack(uint8_t* memorywrite)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_MEMORY_OFFSET + AES_BASE_ADDR);

    *memorywrite = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t aes_memory_memory_write_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(AES_MEMORY_OFFSET + AES_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

__INLINE void aes_memory_memory_write_setf(uint32_t memorywrite)
{
    _PICO_REG_WR(AES_MEMORY_OFFSET+ AES_BASE_ADDR, (uint32_t)memorywrite << 0);
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
    }layer_enable_fld;
    __IO uint32_t layer_enable;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :15;
      __IO uint32_t fifo_io_pdu:1;
      __IO uint32_t :4;
      __IO uint32_t enginne_revert:4;
      __IO uint32_t :3;
      __IO uint32_t single_mode:1;
      __IO uint32_t code_mode:1;
      __IO uint32_t :3;
    }layer_control_fld;
    __IO uint32_t layer_control;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :32;
    }reserved0_fld;
    __IO uint32_t reserved0;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :16;
      __IO uint32_t plen:8;
      __IO uint32_t aad:8;
    }plen_and_aad_fld;
    __IO uint32_t plen_and_aad;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :28;
      __IO uint32_t interupt_enable_:4;
    }interrupt_mask_fld;
    __IO uint32_t interrupt_mask;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :28;
      __IO uint32_t interupt_status:4;
    }interrupt_status_fld;
    __IO uint32_t interrupt_status;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :32;
    }reserved1_fld;
    __IO uint32_t reserved1;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :32;
    }reserved2_fld;
    __IO uint32_t reserved2;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t key0:32;
    }key0_fld;
    __IO uint32_t key0;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t key1:32;
    }key1_fld;
    __IO uint32_t key1;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t key2:32;
    }key2_fld;
    __IO uint32_t key2;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t key3:32;
    }key3_fld;
    __IO uint32_t key3;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t nonce0:32;
    }nonce0_fld;
    __IO uint32_t nonce0;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t nonce1:32;
    }nonce1_fld;
    __IO uint32_t nonce1;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t nonce2:32;
    }nonce2_fld;
    __IO uint32_t nonce2;
  };

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t nonce3:32;
    }nonce3_fld;
    __IO uint32_t nonce3;
  };


  union{ //offset addr 0x0050
    struct{
      __IO uint32_t data_o0:32;
    }data_out0_fld;
    __IO uint32_t data_out0;
  };

  union{ //offset addr 0x0054
    struct{
      __IO uint32_t data_o1:32;
    }data_out1_fld;
    __IO uint32_t data_out1;
  };

  union{ //offset addr 0x0058
    struct{
      __IO uint32_t data_o2:32;
    }data_out2_fld;
    __IO uint32_t data_out2;
  };

  union{ //offset addr 0x005c
    struct{
      __IO uint32_t data_o3:32;
    }data_out3_fld;
    __IO uint32_t data_out3;
  };


  union{ //offset addr 0x0100
    struct{
      __IO uint32_t memory_write:32;
    }memory_fld;
    __IO uint32_t memory;
  };

} PICO_REG_AES_TypeDef;

#define PICO_REG_AES PICO_REG_AES_TypeDef *0x40040000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_AES_H_

