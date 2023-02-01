#ifndef _PICO_REG_CACHE_H_
#define _PICO_REG_CACHE_H_

#include <stdint.h>

#define CACHE_COUNT 3

#define CACHE_BASE_ADDR 0x4000C000

#define CACHE_SIZE 0x00000002


 /**
 * @brief SETTING register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01               bypass   0
 *     00                flush   0
 * </pre>
 */
#define CACHE_SETTING_OFFSET 0x00000000


__INLINE uint32_t cache_setting_get(void)
{
    return _PICO_REG_RD(CACHE_SETTING_OFFSET + CACHE_BASE_ADDR);
}

__INLINE void cache_setting_set(uint32_t value)
{
    _PICO_REG_WR(CACHE_SETTING_OFFSET+ CACHE_BASE_ADDR, value);
}

// field definitions
#define CACHE_SETTING_BYPASS_BIT                            ((uint32_t)0x00000002)
#define CACHE_SETTING_BYPASS_POS                            1
#define CACHE_SETTING_FLUSH_BIT                             ((uint32_t)0x00000001)
#define CACHE_SETTING_FLUSH_POS                             0

#define CACHE_SETTING_BYPASS_RST                            0x0
#define CACHE_SETTING_FLUSH_RST                             0x0

__INLINE void cache_setting_pack(uint8_t bypass, uint8_t flush)
{
    _PICO_REG_WR(CACHE_SETTING_OFFSET+ CACHE_BASE_ADDR,  ((uint32_t)bypass << 1) | ((uint32_t)flush << 0));
}

__INLINE void cache_setting_unpack(uint8_t* bypass, uint8_t* flush)
{
    volatile uint32_t localVal = _PICO_REG_RD(CACHE_SETTING_OFFSET + CACHE_BASE_ADDR);

    *bypass = (localVal & ((uint32_t)0x00000002)) >> 1;
    *flush = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t cache_setting_bypass_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CACHE_SETTING_OFFSET + CACHE_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void cache_setting_bypass_setf(uint8_t bypass)
{
    _PICO_REG_WR(CACHE_SETTING_OFFSET+ CACHE_BASE_ADDR, (_PICO_REG_RD(CACHE_SETTING_OFFSET + CACHE_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)bypass << 1));
}

__INLINE uint8_t cache_setting_flush_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CACHE_SETTING_OFFSET + CACHE_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void cache_setting_flush_setf(uint8_t flush)
{
    _PICO_REG_WR(CACHE_SETTING_OFFSET+ CACHE_BASE_ADDR, (_PICO_REG_RD(CACHE_SETTING_OFFSET + CACHE_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)flush << 0));
}

 /**
 * @brief CACHELINE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00            cacheline   0b0
 * </pre>
 */
#define CACHE_CACHELINE_OFFSET 0x00000001


__INLINE uint32_t cache_cacheline_get(void)
{
    return _PICO_REG_RD(CACHE_CACHELINE_OFFSET + CACHE_BASE_ADDR);
}

// field definitions
#define CACHE_CACHELINE_CACHELINE_MASK                        ((uint32_t)0x00000003)
#define CACHE_CACHELINE_CACHELINE_LSB                         0
#define CACHE_CACHELINE_CACHELINE_WIDTH                       ((uint32_t)0x00000002)

#define CACHE_CACHELINE_CACHELINE_RST                         0x0

__INLINE void cache_cacheline_unpack(uint8_t* cacheline)
{
    volatile uint32_t localVal = _PICO_REG_RD(CACHE_CACHELINE_OFFSET + CACHE_BASE_ADDR);

    *cacheline = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t cache_cacheline_cacheline_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CACHE_CACHELINE_OFFSET + CACHE_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

 /**
 * @brief ASSOCIATIVY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  01:00          associativy   0b0
 * </pre>
 */
#define CACHE_ASSOCIATIVY_OFFSET 0x00000002


__INLINE uint32_t cache_associativy_get(void)
{
    return _PICO_REG_RD(CACHE_ASSOCIATIVY_OFFSET + CACHE_BASE_ADDR);
}

// field definitions
#define CACHE_ASSOCIATIVY_ASSOCIATIVY_MASK                      ((uint32_t)0x00000003)
#define CACHE_ASSOCIATIVY_ASSOCIATIVY_LSB                       0
#define CACHE_ASSOCIATIVY_ASSOCIATIVY_WIDTH                     ((uint32_t)0x00000002)

#define CACHE_ASSOCIATIVY_ASSOCIATIVY_RST                       0x0

__INLINE void cache_associativy_unpack(uint8_t* associativy)
{
    volatile uint32_t localVal = _PICO_REG_RD(CACHE_ASSOCIATIVY_OFFSET + CACHE_BASE_ADDR);

    *associativy = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t cache_associativy_associativy_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(CACHE_ASSOCIATIVY_OFFSET + CACHE_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

#if defined(__CC_ARM)
#pragma push
#pragma anon_unions
#endif

typedef struct
{
  union{ //offset addr 0x0000
    struct{
      __IO uint32_t :30;
      __IO uint32_t bypass:1;
      __IO uint32_t flush:1;
    }setting_fld;
    __IO uint32_t setting;
  };


  union{ //offset addr 0x0001
    struct{
      __IO uint32_t :30;
      __IO uint32_t cacheline:2;
    }cacheline_fld;
    __IO uint32_t cacheline;
  };


  union{ //offset addr 0x0002
    struct{
      __IO uint32_t :30;
      __IO uint32_t associativy:2;
    }associativy_fld;
    __IO uint32_t associativy;
  };

} PICO_REG_CACHE_TypeDef;

#define PICO_REG_CACHE PICO_REG_CACHE_TypeDef *0x4000C000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_CACHE_H_

