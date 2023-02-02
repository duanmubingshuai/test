#ifndef _PICO_REG_I2C0_H_
#define _PICO_REG_I2C0_H_

#include <stdint.h>

#define I2C0_COUNT 45

#define I2C0_BASE_ADDR 0x40005000

#define I2C0_SIZE 0x000000FC


 /**
 * @brief IC_CON register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06     IC_SLAVE_DISABLE   0
 *     05        IC_RESTART_EN   1
 *     04   IC_10BITADDR_MASTER   1
 *     03   IC_10BITADDR_SLAVE   1
 *  02:01                SPEED   0b11
 *     00          MASTER_MODE   0
 * </pre>
 */
#define I2C0_IC_CON_OFFSET 0x00000000


__INLINE uint32_t i2c0_ic_con_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_con_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_CON_IC_SLAVE_DISABLE_BIT                  ((uint32_t)0x00000040)
#define I2C0_IC_CON_IC_SLAVE_DISABLE_POS                  6
#define I2C0_IC_CON_IC_RESTART_EN_BIT                     ((uint32_t)0x00000020)
#define I2C0_IC_CON_IC_RESTART_EN_POS                     5
#define I2C0_IC_CON_IC_10BITADDR_MASTER_BIT               ((uint32_t)0x00000010)
#define I2C0_IC_CON_IC_10BITADDR_MASTER_POS               4
#define I2C0_IC_CON_IC_10BITADDR_SLAVE_BIT                ((uint32_t)0x00000008)
#define I2C0_IC_CON_IC_10BITADDR_SLAVE_POS                3
#define I2C0_IC_CON_SPEED_MASK                            ((uint32_t)0x00000006)
#define I2C0_IC_CON_SPEED_LSB                             1
#define I2C0_IC_CON_SPEED_WIDTH                           ((uint32_t)0x00000002)
#define I2C0_IC_CON_MASTER_MODE_BIT                       ((uint32_t)0x00000001)
#define I2C0_IC_CON_MASTER_MODE_POS                       0

#define I2C0_IC_CON_IC_SLAVE_DISABLE_RST                  0x0
#define I2C0_IC_CON_IC_RESTART_EN_RST                     0x1
#define I2C0_IC_CON_IC_10BITADDR_MASTER_RST               0x1
#define I2C0_IC_CON_IC_10BITADDR_SLAVE_RST                0x1
#define I2C0_IC_CON_SPEED_RST                             0x11
#define I2C0_IC_CON_MASTER_MODE_RST                       0x0

__INLINE void i2c0_ic_con_pack(uint8_t icslavedisable, uint8_t icrestarten, uint8_t ic10bitaddrmaster, uint8_t ic10bitaddrslave, uint8_t speed, uint8_t mastermode)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icslavedisable << 6) | ((uint32_t)icrestarten << 5) | ((uint32_t)ic10bitaddrmaster << 4) | ((uint32_t)ic10bitaddrslave << 3) | ((uint32_t)speed << 1) | ((uint32_t)mastermode << 0));
}

__INLINE void i2c0_ic_con_unpack(uint8_t* icslavedisable, uint8_t* icrestarten, uint8_t* ic10bitaddrmaster, uint8_t* ic10bitaddrslave, uint8_t* speed, uint8_t* mastermode)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);

    *icslavedisable = (localVal & ((uint32_t)0x00000040)) >> 6;
    *icrestarten = (localVal & ((uint32_t)0x00000020)) >> 5;
    *ic10bitaddrmaster = (localVal & ((uint32_t)0x00000010)) >> 4;
    *ic10bitaddrslave = (localVal & ((uint32_t)0x00000008)) >> 3;
    *speed = (localVal & ((uint32_t)0x00000006)) >> 1;
    *mastermode = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_con_ic_slave_disable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void i2c0_ic_con_ic_slave_disable_setf(uint8_t icslavedisable)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)icslavedisable << 6));
}

__INLINE uint8_t i2c0_ic_con_ic_restart_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void i2c0_ic_con_ic_restart_en_setf(uint8_t icrestarten)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)icrestarten << 5));
}

__INLINE uint8_t i2c0_ic_con_ic_10bitaddr_master_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void i2c0_ic_con_ic_10bitaddr_master_setf(uint8_t ic10bitaddrmaster)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)ic10bitaddrmaster << 4));
}

__INLINE uint8_t i2c0_ic_con_ic_10bitaddr_slave_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void i2c0_ic_con_ic_10bitaddr_slave_setf(uint8_t ic10bitaddrslave)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)ic10bitaddrslave << 3));
}

__INLINE uint8_t i2c0_ic_con_speed_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000006)) >> 1);
}

__INLINE void i2c0_ic_con_speed_setf(uint8_t speed)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000006)) | ((uint32_t)speed << 1));
}

__INLINE uint8_t i2c0_ic_con_master_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void i2c0_ic_con_master_mode_setf(uint8_t mastermode)
{
    _PICO_REG_WR(I2C0_IC_CON_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_CON_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)mastermode << 0));
}

 /**
 * @brief IC_TAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     12   IC_10BITADDR_MASTER   1
 *     11              SPECIAL   0
 *     10          GC_OR_START   0
 *  09:00               IC_TAR   0b55
 * </pre>
 */
#define I2C0_IC_TAR_OFFSET 0x00000004


__INLINE uint32_t i2c0_ic_tar_get(void)
{
    return _PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_tar_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_TAR_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_TAR_IC_10BITADDR_MASTER_BIT               ((uint32_t)0x00001000)
#define I2C0_IC_TAR_IC_10BITADDR_MASTER_POS               12
#define I2C0_IC_TAR_SPECIAL_BIT                           ((uint32_t)0x00000800)
#define I2C0_IC_TAR_SPECIAL_POS                           11
#define I2C0_IC_TAR_GC_OR_START_BIT                       ((uint32_t)0x00000400)
#define I2C0_IC_TAR_GC_OR_START_POS                       10
#define I2C0_IC_TAR_IC_TAR_MASK                           ((uint32_t)0x000003FF)
#define I2C0_IC_TAR_IC_TAR_LSB                            0
#define I2C0_IC_TAR_IC_TAR_WIDTH                          ((uint32_t)0x0000000A)

#define I2C0_IC_TAR_IC_10BITADDR_MASTER_RST               0x1
#define I2C0_IC_TAR_SPECIAL_RST                           0x0
#define I2C0_IC_TAR_GC_OR_START_RST                       0x0
#define I2C0_IC_TAR_IC_TAR_RST                            0x55

__INLINE void i2c0_ic_tar_pack(uint8_t ic10bitaddrmaster, uint8_t special, uint8_t gcorstart, uint16_t ictar)
{
    _PICO_REG_WR(I2C0_IC_TAR_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)ic10bitaddrmaster << 12) | ((uint32_t)special << 11) | ((uint32_t)gcorstart << 10) | ((uint32_t)ictar << 0));
}

__INLINE void i2c0_ic_tar_unpack(uint8_t* ic10bitaddrmaster, uint8_t* special, uint8_t* gcorstart, uint8_t* ictar)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR);

    *ic10bitaddrmaster = (localVal & ((uint32_t)0x00001000)) >> 12;
    *special = (localVal & ((uint32_t)0x00000800)) >> 11;
    *gcorstart = (localVal & ((uint32_t)0x00000400)) >> 10;
    *ictar = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint8_t i2c0_ic_tar_ic_10bitaddr_master_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE void i2c0_ic_tar_ic_10bitaddr_master_setf(uint8_t ic10bitaddrmaster)
{
    _PICO_REG_WR(I2C0_IC_TAR_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00001000)) | ((uint32_t)ic10bitaddrmaster << 12));
}

__INLINE uint8_t i2c0_ic_tar_special_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void i2c0_ic_tar_special_setf(uint8_t special)
{
    _PICO_REG_WR(I2C0_IC_TAR_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)special << 11));
}

__INLINE uint8_t i2c0_ic_tar_gc_or_start_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void i2c0_ic_tar_gc_or_start_setf(uint8_t gcorstart)
{
    _PICO_REG_WR(I2C0_IC_TAR_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)gcorstart << 10));
}

__INLINE uint16_t i2c0_ic_tar_ic_tar_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void i2c0_ic_tar_ic_tar_setf(uint16_t ictar)
{
    _PICO_REG_WR(I2C0_IC_TAR_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_TAR_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x000003FF)) | ((uint32_t)ictar << 0));
}

 /**
 * @brief IC_SAR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  09:00               IC_SAR   0b55
 * </pre>
 */
#define I2C0_IC_SAR_OFFSET 0x00000008


__INLINE uint32_t i2c0_ic_sar_get(void)
{
    return _PICO_REG_RD(I2C0_IC_SAR_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_sar_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_SAR_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_SAR_IC_SAR_MASK                           ((uint32_t)0x000003FF)
#define I2C0_IC_SAR_IC_SAR_LSB                            0
#define I2C0_IC_SAR_IC_SAR_WIDTH                          ((uint32_t)0x0000000A)

#define I2C0_IC_SAR_IC_SAR_RST                            0x55

__INLINE void i2c0_ic_sar_pack(uint16_t icsar)
{
    _PICO_REG_WR(I2C0_IC_SAR_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icsar << 0));
}

__INLINE void i2c0_ic_sar_unpack(uint8_t* icsar)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SAR_OFFSET + I2C0_BASE_ADDR);

    *icsar = (localVal & ((uint32_t)0x000003FF)) >> 0;
}

__INLINE uint16_t i2c0_ic_sar_ic_sar_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SAR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000003FF)) >> 0);
}

__INLINE void i2c0_ic_sar_ic_sar_setf(uint16_t icsar)
{
    _PICO_REG_WR(I2C0_IC_SAR_OFFSET+ I2C0_BASE_ADDR, (uint32_t)icsar << 0);
}

 /**
 * @brief IC_HS_MADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00            IC_HS_MAR   0b1
 * </pre>
 */
#define I2C0_IC_HS_MADDR_OFFSET 0x0000000C


__INLINE uint32_t i2c0_ic_hs_maddr_get(void)
{
    return _PICO_REG_RD(I2C0_IC_HS_MADDR_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_hs_maddr_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_HS_MADDR_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_HS_MADDR_IC_HS_MAR_MASK                        ((uint32_t)0x00000007)
#define I2C0_IC_HS_MADDR_IC_HS_MAR_LSB                         0
#define I2C0_IC_HS_MADDR_IC_HS_MAR_WIDTH                       ((uint32_t)0x00000003)

#define I2C0_IC_HS_MADDR_IC_HS_MAR_RST                         0x1

__INLINE void i2c0_ic_hs_maddr_pack(uint8_t ichsmar)
{
    _PICO_REG_WR(I2C0_IC_HS_MADDR_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)ichsmar << 0));
}

__INLINE void i2c0_ic_hs_maddr_unpack(uint8_t* ichsmar)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_MADDR_OFFSET + I2C0_BASE_ADDR);

    *ichsmar = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t i2c0_ic_hs_maddr_ic_hs_mar_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_MADDR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void i2c0_ic_hs_maddr_ic_hs_mar_setf(uint8_t ichsmar)
{
    _PICO_REG_WR(I2C0_IC_HS_MADDR_OFFSET+ I2C0_BASE_ADDR, (uint32_t)ichsmar << 0);
}

 /**
 * @brief IC_DATA_CMD register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     08                  CMD   0
 *  07:00                  DAT   0b0
 * </pre>
 */
#define I2C0_IC_DATA_CMD_OFFSET 0x00000010


__INLINE uint32_t i2c0_ic_data_cmd_get(void)
{
    return _PICO_REG_RD(I2C0_IC_DATA_CMD_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_data_cmd_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_DATA_CMD_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_DATA_CMD_CMD_BIT                               ((uint32_t)0x00000100)
#define I2C0_IC_DATA_CMD_CMD_POS                               8
#define I2C0_IC_DATA_CMD_DAT_MASK                              ((uint32_t)0x000000FF)
#define I2C0_IC_DATA_CMD_DAT_LSB                               0
#define I2C0_IC_DATA_CMD_DAT_WIDTH                             ((uint32_t)0x00000008)

#define I2C0_IC_DATA_CMD_CMD_RST                               0x0
#define I2C0_IC_DATA_CMD_DAT_RST                               0x0

__INLINE void i2c0_ic_data_cmd_pack(uint8_t cmd, uint8_t dat)
{
    _PICO_REG_WR(I2C0_IC_DATA_CMD_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)cmd << 8) | ((uint32_t)dat << 0));
}

__INLINE void i2c0_ic_data_cmd_unpack(uint8_t* cmd, uint8_t* dat)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DATA_CMD_OFFSET + I2C0_BASE_ADDR);

    *cmd = (localVal & ((uint32_t)0x00000100)) >> 8;
    *dat = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t i2c0_ic_data_cmd_cmd_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DATA_CMD_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void i2c0_ic_data_cmd_cmd_setf(uint8_t cmd)
{
    _PICO_REG_WR(I2C0_IC_DATA_CMD_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_DATA_CMD_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)cmd << 8));
}

__INLINE uint8_t i2c0_ic_data_cmd_dat_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DATA_CMD_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void i2c0_ic_data_cmd_dat_setf(uint8_t dat)
{
    _PICO_REG_WR(I2C0_IC_DATA_CMD_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_DATA_CMD_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x000000FF)) | ((uint32_t)dat << 0));
}

 /**
 * @brief IC_SS_SCL_HCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       IC_SS_SCL_HCNT   0b0
 * </pre>
 */
#define I2C0_IC_SS_SCL_HCNT_OFFSET 0x00000014


__INLINE uint32_t i2c0_ic_ss_scl_hcnt_get(void)
{
    return _PICO_REG_RD(I2C0_IC_SS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_ss_scl_hcnt_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_SS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_MASK                   ((uint32_t)0x0000FFFF)
#define I2C0_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_LSB                    0
#define I2C0_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_WIDTH                  ((uint32_t)0x00000010)

#define I2C0_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_RST                    0x0

__INLINE void i2c0_ic_ss_scl_hcnt_pack(uint16_t icsssclhcnt)
{
    _PICO_REG_WR(I2C0_IC_SS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icsssclhcnt << 0));
}

__INLINE void i2c0_ic_ss_scl_hcnt_unpack(uint8_t* icsssclhcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);

    *icsssclhcnt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t i2c0_ic_ss_scl_hcnt_ic_ss_scl_hcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void i2c0_ic_ss_scl_hcnt_ic_ss_scl_hcnt_setf(uint16_t icsssclhcnt)
{
    _PICO_REG_WR(I2C0_IC_SS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR, (uint32_t)icsssclhcnt << 0);
}

 /**
 * @brief IC_SS_SCL_LCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       IC_SS_SCL_LCNT   0b0
 * </pre>
 */
#define I2C0_IC_SS_SCL_LCNT_OFFSET 0x00000018


__INLINE uint32_t i2c0_ic_ss_scl_lcnt_get(void)
{
    return _PICO_REG_RD(I2C0_IC_SS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_ss_scl_lcnt_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_SS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_MASK                   ((uint32_t)0x0000FFFF)
#define I2C0_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_LSB                    0
#define I2C0_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_WIDTH                  ((uint32_t)0x00000010)

#define I2C0_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_RST                    0x0

__INLINE void i2c0_ic_ss_scl_lcnt_pack(uint16_t icssscllcnt)
{
    _PICO_REG_WR(I2C0_IC_SS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icssscllcnt << 0));
}

__INLINE void i2c0_ic_ss_scl_lcnt_unpack(uint8_t* icssscllcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);

    *icssscllcnt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t i2c0_ic_ss_scl_lcnt_ic_ss_scl_lcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void i2c0_ic_ss_scl_lcnt_ic_ss_scl_lcnt_setf(uint16_t icssscllcnt)
{
    _PICO_REG_WR(I2C0_IC_SS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR, (uint32_t)icssscllcnt << 0);
}

 /**
 * @brief IC_FS_SCL_HCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       IC_FS_SCL_HCNT   0b0
 * </pre>
 */
#define I2C0_IC_FS_SCL_HCNT_OFFSET 0x0000001C


__INLINE uint32_t i2c0_ic_fs_scl_hcnt_get(void)
{
    return _PICO_REG_RD(I2C0_IC_FS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_fs_scl_hcnt_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_FS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_MASK                   ((uint32_t)0x0000FFFF)
#define I2C0_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_LSB                    0
#define I2C0_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_WIDTH                  ((uint32_t)0x00000010)

#define I2C0_IC_FS_SCL_HCNT_IC_FS_SCL_HCNT_RST                    0x0

__INLINE void i2c0_ic_fs_scl_hcnt_pack(uint16_t icfssclhcnt)
{
    _PICO_REG_WR(I2C0_IC_FS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icfssclhcnt << 0));
}

__INLINE void i2c0_ic_fs_scl_hcnt_unpack(uint8_t* icfssclhcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_FS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);

    *icfssclhcnt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t i2c0_ic_fs_scl_hcnt_ic_fs_scl_hcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_FS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void i2c0_ic_fs_scl_hcnt_ic_fs_scl_hcnt_setf(uint16_t icfssclhcnt)
{
    _PICO_REG_WR(I2C0_IC_FS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR, (uint32_t)icfssclhcnt << 0);
}

 /**
 * @brief IC_FS_SCL_LCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       IC_FS_SCL_LCNT   0b0
 * </pre>
 */
#define I2C0_IC_FS_SCL_LCNT_OFFSET 0x00000020


__INLINE uint32_t i2c0_ic_fs_scl_lcnt_get(void)
{
    return _PICO_REG_RD(I2C0_IC_FS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_fs_scl_lcnt_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_FS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_MASK                   ((uint32_t)0x0000FFFF)
#define I2C0_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_LSB                    0
#define I2C0_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_WIDTH                  ((uint32_t)0x00000010)

#define I2C0_IC_FS_SCL_LCNT_IC_FS_SCL_LCNT_RST                    0x0

__INLINE void i2c0_ic_fs_scl_lcnt_pack(uint16_t icfsscllcnt)
{
    _PICO_REG_WR(I2C0_IC_FS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icfsscllcnt << 0));
}

__INLINE void i2c0_ic_fs_scl_lcnt_unpack(uint8_t* icfsscllcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_FS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);

    *icfsscllcnt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t i2c0_ic_fs_scl_lcnt_ic_fs_scl_lcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_FS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void i2c0_ic_fs_scl_lcnt_ic_fs_scl_lcnt_setf(uint16_t icfsscllcnt)
{
    _PICO_REG_WR(I2C0_IC_FS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR, (uint32_t)icfsscllcnt << 0);
}

 /**
 * @brief IC_HS_SCL_HCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       IC_HS_SCL_HCNT   0b0
 * </pre>
 */
#define I2C0_IC_HS_SCL_HCNT_OFFSET 0x00000024


__INLINE uint32_t i2c0_ic_hs_scl_hcnt_get(void)
{
    return _PICO_REG_RD(I2C0_IC_HS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_hs_scl_hcnt_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_HS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_HS_SCL_HCNT_IC_HS_SCL_HCNT_MASK                   ((uint32_t)0x0000FFFF)
#define I2C0_IC_HS_SCL_HCNT_IC_HS_SCL_HCNT_LSB                    0
#define I2C0_IC_HS_SCL_HCNT_IC_HS_SCL_HCNT_WIDTH                  ((uint32_t)0x00000010)

#define I2C0_IC_HS_SCL_HCNT_IC_HS_SCL_HCNT_RST                    0x0

__INLINE void i2c0_ic_hs_scl_hcnt_pack(uint16_t ichssclhcnt)
{
    _PICO_REG_WR(I2C0_IC_HS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)ichssclhcnt << 0));
}

__INLINE void i2c0_ic_hs_scl_hcnt_unpack(uint8_t* ichssclhcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);

    *ichssclhcnt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t i2c0_ic_hs_scl_hcnt_ic_hs_scl_hcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_SCL_HCNT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void i2c0_ic_hs_scl_hcnt_ic_hs_scl_hcnt_setf(uint16_t ichssclhcnt)
{
    _PICO_REG_WR(I2C0_IC_HS_SCL_HCNT_OFFSET+ I2C0_BASE_ADDR, (uint32_t)ichssclhcnt << 0);
}

 /**
 * @brief IC_HS_SCL_LCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       IC_HS_SCL_LCNT   0b0
 * </pre>
 */
#define I2C0_IC_HS_SCL_LCNT_OFFSET 0x00000028


__INLINE uint32_t i2c0_ic_hs_scl_lcnt_get(void)
{
    return _PICO_REG_RD(I2C0_IC_HS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_hs_scl_lcnt_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_HS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_HS_SCL_LCNT_IC_HS_SCL_LCNT_MASK                   ((uint32_t)0x0000FFFF)
#define I2C0_IC_HS_SCL_LCNT_IC_HS_SCL_LCNT_LSB                    0
#define I2C0_IC_HS_SCL_LCNT_IC_HS_SCL_LCNT_WIDTH                  ((uint32_t)0x00000010)

#define I2C0_IC_HS_SCL_LCNT_IC_HS_SCL_LCNT_RST                    0x0

__INLINE void i2c0_ic_hs_scl_lcnt_pack(uint16_t ichsscllcnt)
{
    _PICO_REG_WR(I2C0_IC_HS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)ichsscllcnt << 0));
}

__INLINE void i2c0_ic_hs_scl_lcnt_unpack(uint8_t* ichsscllcnt)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);

    *ichsscllcnt = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t i2c0_ic_hs_scl_lcnt_ic_hs_scl_lcnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_SCL_LCNT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void i2c0_ic_hs_scl_lcnt_ic_hs_scl_lcnt_setf(uint16_t ichsscllcnt)
{
    _PICO_REG_WR(I2C0_IC_HS_SCL_LCNT_OFFSET+ I2C0_BASE_ADDR, (uint32_t)ichsscllcnt << 0);
}

 /**
 * @brief IC_INTR_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     11           R_GEN_CALL   0
 *     10          R_START_DET   0
 *     09           R_STOP_DET   0
 *     08           R_ACTIVITY   0
 *     07            R_RX_DONE   0
 *     06            R_TX_ABRT   0
 *     05             R_RD_REQ   0
 *     04           R_TX_EMPTY   0
 *     03            R_TX_OVER   0
 *     02            R_RX_FULL   0
 *     01            R_RX_OVER   0
 *     00           R_RX_UNDER   0
 * </pre>
 */
#define I2C0_IC_INTR_STAT_OFFSET 0x0000002C


__INLINE uint32_t i2c0_ic_intr_stat_get(void)
{
    return _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_INTR_STAT_R_GEN_CALL_BIT                        ((uint32_t)0x00000800)
#define I2C0_IC_INTR_STAT_R_GEN_CALL_POS                        11
#define I2C0_IC_INTR_STAT_R_START_DET_BIT                       ((uint32_t)0x00000400)
#define I2C0_IC_INTR_STAT_R_START_DET_POS                       10
#define I2C0_IC_INTR_STAT_R_STOP_DET_BIT                        ((uint32_t)0x00000200)
#define I2C0_IC_INTR_STAT_R_STOP_DET_POS                        9
#define I2C0_IC_INTR_STAT_R_ACTIVITY_BIT                        ((uint32_t)0x00000100)
#define I2C0_IC_INTR_STAT_R_ACTIVITY_POS                        8
#define I2C0_IC_INTR_STAT_R_RX_DONE_BIT                         ((uint32_t)0x00000080)
#define I2C0_IC_INTR_STAT_R_RX_DONE_POS                         7
#define I2C0_IC_INTR_STAT_R_TX_ABRT_BIT                         ((uint32_t)0x00000040)
#define I2C0_IC_INTR_STAT_R_TX_ABRT_POS                         6
#define I2C0_IC_INTR_STAT_R_RD_REQ_BIT                          ((uint32_t)0x00000020)
#define I2C0_IC_INTR_STAT_R_RD_REQ_POS                          5
#define I2C0_IC_INTR_STAT_R_TX_EMPTY_BIT                        ((uint32_t)0x00000010)
#define I2C0_IC_INTR_STAT_R_TX_EMPTY_POS                        4
#define I2C0_IC_INTR_STAT_R_TX_OVER_BIT                         ((uint32_t)0x00000008)
#define I2C0_IC_INTR_STAT_R_TX_OVER_POS                         3
#define I2C0_IC_INTR_STAT_R_RX_FULL_BIT                         ((uint32_t)0x00000004)
#define I2C0_IC_INTR_STAT_R_RX_FULL_POS                         2
#define I2C0_IC_INTR_STAT_R_RX_OVER_BIT                         ((uint32_t)0x00000002)
#define I2C0_IC_INTR_STAT_R_RX_OVER_POS                         1
#define I2C0_IC_INTR_STAT_R_RX_UNDER_BIT                        ((uint32_t)0x00000001)
#define I2C0_IC_INTR_STAT_R_RX_UNDER_POS                        0

#define I2C0_IC_INTR_STAT_R_GEN_CALL_RST                        0x0
#define I2C0_IC_INTR_STAT_R_START_DET_RST                       0x0
#define I2C0_IC_INTR_STAT_R_STOP_DET_RST                        0x0
#define I2C0_IC_INTR_STAT_R_ACTIVITY_RST                        0x0
#define I2C0_IC_INTR_STAT_R_RX_DONE_RST                         0x0
#define I2C0_IC_INTR_STAT_R_TX_ABRT_RST                         0x0
#define I2C0_IC_INTR_STAT_R_RD_REQ_RST                          0x0
#define I2C0_IC_INTR_STAT_R_TX_EMPTY_RST                        0x0
#define I2C0_IC_INTR_STAT_R_TX_OVER_RST                         0x0
#define I2C0_IC_INTR_STAT_R_RX_FULL_RST                         0x0
#define I2C0_IC_INTR_STAT_R_RX_OVER_RST                         0x0
#define I2C0_IC_INTR_STAT_R_RX_UNDER_RST                        0x0

__INLINE void i2c0_ic_intr_stat_unpack(uint8_t* rgencall, uint8_t* rstartdet, uint8_t* rstopdet, uint8_t* ractivity, uint8_t* rrxdone, uint8_t* rtxabrt, uint8_t* rrdreq, uint8_t* rtxempty, uint8_t* rtxover, uint8_t* rrxfull, uint8_t* rrxover, uint8_t* rrxunder)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);

    *rgencall = (localVal & ((uint32_t)0x00000800)) >> 11;
    *rstartdet = (localVal & ((uint32_t)0x00000400)) >> 10;
    *rstopdet = (localVal & ((uint32_t)0x00000200)) >> 9;
    *ractivity = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rrxdone = (localVal & ((uint32_t)0x00000080)) >> 7;
    *rtxabrt = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rrdreq = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rtxempty = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rtxover = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rrxfull = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rrxover = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rrxunder = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_intr_stat_r_gen_call_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_start_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_stop_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_activity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_rx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_tx_abrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_rd_req_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_tx_empty_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_tx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_rx_full_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_rx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t i2c0_ic_intr_stat_r_rx_under_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_INTR_MASK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     11           R_GEN_CALL   1
 *     10          R_START_DET   0
 *     09           R_STOP_DET   0
 *     08           R_ACTIVITY   0
 *     07            R_RX_DONE   1
 *     06            R_TX_ABRT   1
 *     05             R_RD_REQ   1
 *     04           R_TX_EMPTY   1
 *     03            R_TX_OVER   1
 *     02            R_RX_FULL   1
 *     01            R_RX_OVER   1
 *     00           R_RX_UNDER   1
 * </pre>
 */
#define I2C0_IC_INTR_MASK_OFFSET 0x00000030


__INLINE uint32_t i2c0_ic_intr_mask_get(void)
{
    return _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_intr_mask_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_INTR_MASK_R_GEN_CALL_BIT                        ((uint32_t)0x00000800)
#define I2C0_IC_INTR_MASK_R_GEN_CALL_POS                        11
#define I2C0_IC_INTR_MASK_R_START_DET_BIT                       ((uint32_t)0x00000400)
#define I2C0_IC_INTR_MASK_R_START_DET_POS                       10
#define I2C0_IC_INTR_MASK_R_STOP_DET_BIT                        ((uint32_t)0x00000200)
#define I2C0_IC_INTR_MASK_R_STOP_DET_POS                        9
#define I2C0_IC_INTR_MASK_R_ACTIVITY_BIT                        ((uint32_t)0x00000100)
#define I2C0_IC_INTR_MASK_R_ACTIVITY_POS                        8
#define I2C0_IC_INTR_MASK_R_RX_DONE_BIT                         ((uint32_t)0x00000080)
#define I2C0_IC_INTR_MASK_R_RX_DONE_POS                         7
#define I2C0_IC_INTR_MASK_R_TX_ABRT_BIT                         ((uint32_t)0x00000040)
#define I2C0_IC_INTR_MASK_R_TX_ABRT_POS                         6
#define I2C0_IC_INTR_MASK_R_RD_REQ_BIT                          ((uint32_t)0x00000020)
#define I2C0_IC_INTR_MASK_R_RD_REQ_POS                          5
#define I2C0_IC_INTR_MASK_R_TX_EMPTY_BIT                        ((uint32_t)0x00000010)
#define I2C0_IC_INTR_MASK_R_TX_EMPTY_POS                        4
#define I2C0_IC_INTR_MASK_R_TX_OVER_BIT                         ((uint32_t)0x00000008)
#define I2C0_IC_INTR_MASK_R_TX_OVER_POS                         3
#define I2C0_IC_INTR_MASK_R_RX_FULL_BIT                         ((uint32_t)0x00000004)
#define I2C0_IC_INTR_MASK_R_RX_FULL_POS                         2
#define I2C0_IC_INTR_MASK_R_RX_OVER_BIT                         ((uint32_t)0x00000002)
#define I2C0_IC_INTR_MASK_R_RX_OVER_POS                         1
#define I2C0_IC_INTR_MASK_R_RX_UNDER_BIT                        ((uint32_t)0x00000001)
#define I2C0_IC_INTR_MASK_R_RX_UNDER_POS                        0

#define I2C0_IC_INTR_MASK_R_GEN_CALL_RST                        0x1
#define I2C0_IC_INTR_MASK_R_START_DET_RST                       0x0
#define I2C0_IC_INTR_MASK_R_STOP_DET_RST                        0x0
#define I2C0_IC_INTR_MASK_R_ACTIVITY_RST                        0x0
#define I2C0_IC_INTR_MASK_R_RX_DONE_RST                         0x1
#define I2C0_IC_INTR_MASK_R_TX_ABRT_RST                         0x1
#define I2C0_IC_INTR_MASK_R_RD_REQ_RST                          0x1
#define I2C0_IC_INTR_MASK_R_TX_EMPTY_RST                        0x1
#define I2C0_IC_INTR_MASK_R_TX_OVER_RST                         0x1
#define I2C0_IC_INTR_MASK_R_RX_FULL_RST                         0x1
#define I2C0_IC_INTR_MASK_R_RX_OVER_RST                         0x1
#define I2C0_IC_INTR_MASK_R_RX_UNDER_RST                        0x1

__INLINE void i2c0_ic_intr_mask_pack(uint8_t rgencall, uint8_t rstartdet, uint8_t rstopdet, uint8_t ractivity, uint8_t rrxdone, uint8_t rtxabrt, uint8_t rrdreq, uint8_t rtxempty, uint8_t rtxover, uint8_t rrxfull, uint8_t rrxover, uint8_t rrxunder)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)rgencall << 11) | ((uint32_t)rstartdet << 10) | ((uint32_t)rstopdet << 9) | ((uint32_t)ractivity << 8) | ((uint32_t)rrxdone << 7) | ((uint32_t)rtxabrt << 6) | ((uint32_t)rrdreq << 5) | ((uint32_t)rtxempty << 4) | ((uint32_t)rtxover << 3) | ((uint32_t)rrxfull << 2) | ((uint32_t)rrxover << 1) | ((uint32_t)rrxunder << 0));
}

__INLINE void i2c0_ic_intr_mask_unpack(uint8_t* rgencall, uint8_t* rstartdet, uint8_t* rstopdet, uint8_t* ractivity, uint8_t* rrxdone, uint8_t* rtxabrt, uint8_t* rrdreq, uint8_t* rtxempty, uint8_t* rtxover, uint8_t* rrxfull, uint8_t* rrxover, uint8_t* rrxunder)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);

    *rgencall = (localVal & ((uint32_t)0x00000800)) >> 11;
    *rstartdet = (localVal & ((uint32_t)0x00000400)) >> 10;
    *rstopdet = (localVal & ((uint32_t)0x00000200)) >> 9;
    *ractivity = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rrxdone = (localVal & ((uint32_t)0x00000080)) >> 7;
    *rtxabrt = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rrdreq = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rtxempty = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rtxover = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rrxfull = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rrxover = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rrxunder = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_intr_mask_r_gen_call_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE void i2c0_ic_intr_mask_r_gen_call_setf(uint8_t rgencall)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000800)) | ((uint32_t)rgencall << 11));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_start_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE void i2c0_ic_intr_mask_r_start_det_setf(uint8_t rstartdet)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000400)) | ((uint32_t)rstartdet << 10));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_stop_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE void i2c0_ic_intr_mask_r_stop_det_setf(uint8_t rstopdet)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000200)) | ((uint32_t)rstopdet << 9));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_activity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE void i2c0_ic_intr_mask_r_activity_setf(uint8_t ractivity)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000100)) | ((uint32_t)ractivity << 8));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_rx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE void i2c0_ic_intr_mask_r_rx_done_setf(uint8_t rrxdone)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000080)) | ((uint32_t)rrxdone << 7));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_tx_abrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE void i2c0_ic_intr_mask_r_tx_abrt_setf(uint8_t rtxabrt)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000040)) | ((uint32_t)rtxabrt << 6));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_rd_req_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE void i2c0_ic_intr_mask_r_rd_req_setf(uint8_t rrdreq)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000020)) | ((uint32_t)rrdreq << 5));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_tx_empty_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE void i2c0_ic_intr_mask_r_tx_empty_setf(uint8_t rtxempty)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000010)) | ((uint32_t)rtxempty << 4));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_tx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE void i2c0_ic_intr_mask_r_tx_over_setf(uint8_t rtxover)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000008)) | ((uint32_t)rtxover << 3));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_rx_full_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE void i2c0_ic_intr_mask_r_rx_full_setf(uint8_t rrxfull)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000004)) | ((uint32_t)rrxfull << 2));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_rx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void i2c0_ic_intr_mask_r_rx_over_setf(uint8_t rrxover)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)rrxover << 1));
}

__INLINE uint8_t i2c0_ic_intr_mask_r_rx_under_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void i2c0_ic_intr_mask_r_rx_under_setf(uint8_t rrxunder)
{
    _PICO_REG_WR(I2C0_IC_INTR_MASK_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_INTR_MASK_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rrxunder << 0));
}

 /**
 * @brief IC_RAW_INTR_STAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     11             GEN_CALL   0
 *     10            START_DET   0
 *     09             STOP_DET   0
 *     08             ACTIVITY   0
 *     07              RX_DONE   0
 *     06              TX_ABRT   0
 *     05               RD_REQ   0
 *     04             TX_EMPTY   0
 *     03              TX_OVER   0
 *     02              RX_FULL   0
 *     01              RX_OVER   0
 *     00             RX_UNDER   0
 * </pre>
 */
#define I2C0_IC_RAW_INTR_STAT_OFFSET 0x00000034


__INLINE uint32_t i2c0_ic_raw_intr_stat_get(void)
{
    return _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_RAW_INTR_STAT_GEN_CALL_BIT                          ((uint32_t)0x00000800)
#define I2C0_IC_RAW_INTR_STAT_GEN_CALL_POS                          11
#define I2C0_IC_RAW_INTR_STAT_START_DET_BIT                         ((uint32_t)0x00000400)
#define I2C0_IC_RAW_INTR_STAT_START_DET_POS                         10
#define I2C0_IC_RAW_INTR_STAT_STOP_DET_BIT                          ((uint32_t)0x00000200)
#define I2C0_IC_RAW_INTR_STAT_STOP_DET_POS                          9
#define I2C0_IC_RAW_INTR_STAT_ACTIVITY_BIT                          ((uint32_t)0x00000100)
#define I2C0_IC_RAW_INTR_STAT_ACTIVITY_POS                          8
#define I2C0_IC_RAW_INTR_STAT_RX_DONE_BIT                           ((uint32_t)0x00000080)
#define I2C0_IC_RAW_INTR_STAT_RX_DONE_POS                           7
#define I2C0_IC_RAW_INTR_STAT_TX_ABRT_BIT                           ((uint32_t)0x00000040)
#define I2C0_IC_RAW_INTR_STAT_TX_ABRT_POS                           6
#define I2C0_IC_RAW_INTR_STAT_RD_REQ_BIT                            ((uint32_t)0x00000020)
#define I2C0_IC_RAW_INTR_STAT_RD_REQ_POS                            5
#define I2C0_IC_RAW_INTR_STAT_TX_EMPTY_BIT                          ((uint32_t)0x00000010)
#define I2C0_IC_RAW_INTR_STAT_TX_EMPTY_POS                          4
#define I2C0_IC_RAW_INTR_STAT_TX_OVER_BIT                           ((uint32_t)0x00000008)
#define I2C0_IC_RAW_INTR_STAT_TX_OVER_POS                           3
#define I2C0_IC_RAW_INTR_STAT_RX_FULL_BIT                           ((uint32_t)0x00000004)
#define I2C0_IC_RAW_INTR_STAT_RX_FULL_POS                           2
#define I2C0_IC_RAW_INTR_STAT_RX_OVER_BIT                           ((uint32_t)0x00000002)
#define I2C0_IC_RAW_INTR_STAT_RX_OVER_POS                           1
#define I2C0_IC_RAW_INTR_STAT_RX_UNDER_BIT                          ((uint32_t)0x00000001)
#define I2C0_IC_RAW_INTR_STAT_RX_UNDER_POS                          0

#define I2C0_IC_RAW_INTR_STAT_GEN_CALL_RST                          0x0
#define I2C0_IC_RAW_INTR_STAT_START_DET_RST                         0x0
#define I2C0_IC_RAW_INTR_STAT_STOP_DET_RST                          0x0
#define I2C0_IC_RAW_INTR_STAT_ACTIVITY_RST                          0x0
#define I2C0_IC_RAW_INTR_STAT_RX_DONE_RST                           0x0
#define I2C0_IC_RAW_INTR_STAT_TX_ABRT_RST                           0x0
#define I2C0_IC_RAW_INTR_STAT_RD_REQ_RST                            0x0
#define I2C0_IC_RAW_INTR_STAT_TX_EMPTY_RST                          0x0
#define I2C0_IC_RAW_INTR_STAT_TX_OVER_RST                           0x0
#define I2C0_IC_RAW_INTR_STAT_RX_FULL_RST                           0x0
#define I2C0_IC_RAW_INTR_STAT_RX_OVER_RST                           0x0
#define I2C0_IC_RAW_INTR_STAT_RX_UNDER_RST                          0x0

__INLINE void i2c0_ic_raw_intr_stat_unpack(uint8_t* gencall, uint8_t* startdet, uint8_t* stopdet, uint8_t* activity, uint8_t* rxdone, uint8_t* txabrt, uint8_t* rdreq, uint8_t* txempty, uint8_t* txover, uint8_t* rxfull, uint8_t* rxover, uint8_t* rxunder)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);

    *gencall = (localVal & ((uint32_t)0x00000800)) >> 11;
    *startdet = (localVal & ((uint32_t)0x00000400)) >> 10;
    *stopdet = (localVal & ((uint32_t)0x00000200)) >> 9;
    *activity = (localVal & ((uint32_t)0x00000100)) >> 8;
    *rxdone = (localVal & ((uint32_t)0x00000080)) >> 7;
    *txabrt = (localVal & ((uint32_t)0x00000040)) >> 6;
    *rdreq = (localVal & ((uint32_t)0x00000020)) >> 5;
    *txempty = (localVal & ((uint32_t)0x00000010)) >> 4;
    *txover = (localVal & ((uint32_t)0x00000008)) >> 3;
    *rxfull = (localVal & ((uint32_t)0x00000004)) >> 2;
    *rxover = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rxunder = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_gen_call_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_start_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_stop_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_activity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_rx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_tx_abrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_rd_req_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_tx_empty_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_tx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_rx_full_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_rx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t i2c0_ic_raw_intr_stat_rx_under_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RAW_INTR_STAT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_RX_TL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                RX_TL   0b0
 * </pre>
 */
#define I2C0_IC_RX_TL_OFFSET 0x00000038


__INLINE uint32_t i2c0_ic_rx_tl_get(void)
{
    return _PICO_REG_RD(I2C0_IC_RX_TL_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_rx_tl_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_RX_TL_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_RX_TL_RX_TL_MASK                            ((uint32_t)0x000000FF)
#define I2C0_IC_RX_TL_RX_TL_LSB                             0
#define I2C0_IC_RX_TL_RX_TL_WIDTH                           ((uint32_t)0x00000008)

#define I2C0_IC_RX_TL_RX_TL_RST                             0x0

__INLINE void i2c0_ic_rx_tl_pack(uint8_t rxtl)
{
    _PICO_REG_WR(I2C0_IC_RX_TL_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)rxtl << 0));
}

__INLINE void i2c0_ic_rx_tl_unpack(uint8_t* rxtl)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RX_TL_OFFSET + I2C0_BASE_ADDR);

    *rxtl = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t i2c0_ic_rx_tl_rx_tl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RX_TL_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void i2c0_ic_rx_tl_rx_tl_setf(uint8_t rxtl)
{
    _PICO_REG_WR(I2C0_IC_RX_TL_OFFSET+ I2C0_BASE_ADDR, (uint32_t)rxtl << 0);
}

 /**
 * @brief IC_TX_TL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                TX_TL   0b0
 * </pre>
 */
#define I2C0_IC_TX_TL_OFFSET 0x0000003C


__INLINE uint32_t i2c0_ic_tx_tl_get(void)
{
    return _PICO_REG_RD(I2C0_IC_TX_TL_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_tx_tl_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_TX_TL_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_TX_TL_TX_TL_MASK                            ((uint32_t)0x000000FF)
#define I2C0_IC_TX_TL_TX_TL_LSB                             0
#define I2C0_IC_TX_TL_TX_TL_WIDTH                           ((uint32_t)0x00000008)

#define I2C0_IC_TX_TL_TX_TL_RST                             0x0

__INLINE void i2c0_ic_tx_tl_pack(uint8_t txtl)
{
    _PICO_REG_WR(I2C0_IC_TX_TL_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)txtl << 0));
}

__INLINE void i2c0_ic_tx_tl_unpack(uint8_t* txtl)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_TL_OFFSET + I2C0_BASE_ADDR);

    *txtl = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t i2c0_ic_tx_tl_tx_tl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_TL_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void i2c0_ic_tx_tl_tx_tl_setf(uint8_t txtl)
{
    _PICO_REG_WR(I2C0_IC_TX_TL_OFFSET+ I2C0_BASE_ADDR, (uint32_t)txtl << 0);
}

 /**
 * @brief IC_CLR_INTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00             CLR_INTR   0
 * </pre>
 */
#define I2C0_IC_CLR_INTR_OFFSET 0x00000040


__INLINE uint32_t i2c0_ic_clr_intr_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_INTR_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_INTR_CLR_INTR_BIT                          ((uint32_t)0x00000001)
#define I2C0_IC_CLR_INTR_CLR_INTR_POS                          0

#define I2C0_IC_CLR_INTR_CLR_INTR_RST                          0x0

__INLINE void i2c0_ic_clr_intr_unpack(uint8_t* clrintr)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_INTR_OFFSET + I2C0_BASE_ADDR);

    *clrintr = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_intr_clr_intr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_INTR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_RX_UNDER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00         CLR_RX_UNDER   0
 * </pre>
 */
#define I2C0_IC_CLR_RX_UNDER_OFFSET 0x00000044


__INLINE uint32_t i2c0_ic_clr_rx_under_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_RX_UNDER_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_RX_UNDER_CLR_RX_UNDER_BIT                      ((uint32_t)0x00000001)
#define I2C0_IC_CLR_RX_UNDER_CLR_RX_UNDER_POS                      0

#define I2C0_IC_CLR_RX_UNDER_CLR_RX_UNDER_RST                      0x0

__INLINE void i2c0_ic_clr_rx_under_unpack(uint8_t* clrrxunder)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RX_UNDER_OFFSET + I2C0_BASE_ADDR);

    *clrrxunder = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_rx_under_clr_rx_under_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RX_UNDER_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_RX_OVER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00          CLR_RX_OVER   0
 * </pre>
 */
#define I2C0_IC_CLR_RX_OVER_OFFSET 0x00000048


__INLINE uint32_t i2c0_ic_clr_rx_over_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_RX_OVER_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_RX_OVER_CLR_RX_OVER_BIT                       ((uint32_t)0x00000001)
#define I2C0_IC_CLR_RX_OVER_CLR_RX_OVER_POS                       0

#define I2C0_IC_CLR_RX_OVER_CLR_RX_OVER_RST                       0x0

__INLINE void i2c0_ic_clr_rx_over_unpack(uint8_t* clrrxover)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RX_OVER_OFFSET + I2C0_BASE_ADDR);

    *clrrxover = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_rx_over_clr_rx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RX_OVER_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_TX_OVER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00          CLR_TX_OVER   0
 * </pre>
 */
#define I2C0_IC_CLR_TX_OVER_OFFSET 0x0000004C


__INLINE uint32_t i2c0_ic_clr_tx_over_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_TX_OVER_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_TX_OVER_CLR_TX_OVER_BIT                       ((uint32_t)0x00000001)
#define I2C0_IC_CLR_TX_OVER_CLR_TX_OVER_POS                       0

#define I2C0_IC_CLR_TX_OVER_CLR_TX_OVER_RST                       0x0

__INLINE void i2c0_ic_clr_tx_over_unpack(uint8_t* clrtxover)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_TX_OVER_OFFSET + I2C0_BASE_ADDR);

    *clrtxover = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_tx_over_clr_tx_over_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_TX_OVER_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_RD_REQ register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00           CLR_RD_REQ   0
 * </pre>
 */
#define I2C0_IC_CLR_RD_REQ_OFFSET 0x00000050


__INLINE uint32_t i2c0_ic_clr_rd_req_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_RD_REQ_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_RD_REQ_CLR_RD_REQ_BIT                        ((uint32_t)0x00000001)
#define I2C0_IC_CLR_RD_REQ_CLR_RD_REQ_POS                        0

#define I2C0_IC_CLR_RD_REQ_CLR_RD_REQ_RST                        0x0

__INLINE void i2c0_ic_clr_rd_req_unpack(uint8_t* clrrdreq)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RD_REQ_OFFSET + I2C0_BASE_ADDR);

    *clrrdreq = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_rd_req_clr_rd_req_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RD_REQ_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_TX_ABRT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00          CLR_TX_ABRT   0
 * </pre>
 */
#define I2C0_IC_CLR_TX_ABRT_OFFSET 0x00000054


__INLINE uint32_t i2c0_ic_clr_tx_abrt_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_TX_ABRT_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_TX_ABRT_CLR_TX_ABRT_BIT                       ((uint32_t)0x00000001)
#define I2C0_IC_CLR_TX_ABRT_CLR_TX_ABRT_POS                       0

#define I2C0_IC_CLR_TX_ABRT_CLR_TX_ABRT_RST                       0x0

__INLINE void i2c0_ic_clr_tx_abrt_unpack(uint8_t* clrtxabrt)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_TX_ABRT_OFFSET + I2C0_BASE_ADDR);

    *clrtxabrt = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_tx_abrt_clr_tx_abrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_TX_ABRT_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_RX_DONE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00          CLR_RX_DONE   0
 * </pre>
 */
#define I2C0_IC_CLR_RX_DONE_OFFSET 0x00000058


__INLINE uint32_t i2c0_ic_clr_rx_done_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_RX_DONE_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_RX_DONE_CLR_RX_DONE_BIT                       ((uint32_t)0x00000001)
#define I2C0_IC_CLR_RX_DONE_CLR_RX_DONE_POS                       0

#define I2C0_IC_CLR_RX_DONE_CLR_RX_DONE_RST                       0x0

__INLINE void i2c0_ic_clr_rx_done_unpack(uint8_t* clrrxdone)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RX_DONE_OFFSET + I2C0_BASE_ADDR);

    *clrrxdone = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_rx_done_clr_rx_done_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_RX_DONE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_ACTIVITY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00         CLR_ACTIVITY   0
 * </pre>
 */
#define I2C0_IC_CLR_ACTIVITY_OFFSET 0x0000005C


__INLINE uint32_t i2c0_ic_clr_activity_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_ACTIVITY_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_ACTIVITY_CLR_ACTIVITY_BIT                      ((uint32_t)0x00000001)
#define I2C0_IC_CLR_ACTIVITY_CLR_ACTIVITY_POS                      0

#define I2C0_IC_CLR_ACTIVITY_CLR_ACTIVITY_RST                      0x0

__INLINE void i2c0_ic_clr_activity_unpack(uint8_t* clractivity)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_ACTIVITY_OFFSET + I2C0_BASE_ADDR);

    *clractivity = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_activity_clr_activity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_ACTIVITY_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_STOP_DET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00         CLR_STOP_DET   0
 * </pre>
 */
#define I2C0_IC_CLR_STOP_DET_OFFSET 0x00000060


__INLINE uint32_t i2c0_ic_clr_stop_det_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_STOP_DET_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_STOP_DET_CLR_STOP_DET_BIT                      ((uint32_t)0x00000001)
#define I2C0_IC_CLR_STOP_DET_CLR_STOP_DET_POS                      0

#define I2C0_IC_CLR_STOP_DET_CLR_STOP_DET_RST                      0x0

__INLINE void i2c0_ic_clr_stop_det_unpack(uint8_t* clrstopdet)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_STOP_DET_OFFSET + I2C0_BASE_ADDR);

    *clrstopdet = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_stop_det_clr_stop_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_STOP_DET_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_START_DET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00        CLR_START_DET   0
 * </pre>
 */
#define I2C0_IC_CLR_START_DET_OFFSET 0x00000064


__INLINE uint32_t i2c0_ic_clr_start_det_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_START_DET_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_START_DET_CLR_START_DET_BIT                     ((uint32_t)0x00000001)
#define I2C0_IC_CLR_START_DET_CLR_START_DET_POS                     0

#define I2C0_IC_CLR_START_DET_CLR_START_DET_RST                     0x0

__INLINE void i2c0_ic_clr_start_det_unpack(uint8_t* clrstartdet)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_START_DET_OFFSET + I2C0_BASE_ADDR);

    *clrstartdet = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_start_det_clr_start_det_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_START_DET_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_CLR_GEN_CALL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00         CLR_GEN_CALL   0
 * </pre>
 */
#define I2C0_IC_CLR_GEN_CALL_OFFSET 0x00000068


__INLINE uint32_t i2c0_ic_clr_gen_call_get(void)
{
    return _PICO_REG_RD(I2C0_IC_CLR_GEN_CALL_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_CLR_GEN_CALL_CLR_GEN_CALL_BIT                      ((uint32_t)0x00000001)
#define I2C0_IC_CLR_GEN_CALL_CLR_GEN_CALL_POS                      0

#define I2C0_IC_CLR_GEN_CALL_CLR_GEN_CALL_RST                      0x0

__INLINE void i2c0_ic_clr_gen_call_unpack(uint8_t* clrgencall)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_GEN_CALL_OFFSET + I2C0_BASE_ADDR);

    *clrgencall = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_clr_gen_call_clr_gen_call_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_CLR_GEN_CALL_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_ENABLE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01                ABORT   0
 *     00               ENABLE   0
 * </pre>
 */
#define I2C0_IC_ENABLE_OFFSET 0x0000006C


__INLINE uint32_t i2c0_ic_enable_get(void)
{
    return _PICO_REG_RD(I2C0_IC_ENABLE_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_enable_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_ENABLE_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_ENABLE_ABORT_BIT                             ((uint32_t)0x00000002)
#define I2C0_IC_ENABLE_ABORT_POS                             1
#define I2C0_IC_ENABLE_ENABLE_BIT                            ((uint32_t)0x00000001)
#define I2C0_IC_ENABLE_ENABLE_POS                            0

#define I2C0_IC_ENABLE_ABORT_RST                             0x0
#define I2C0_IC_ENABLE_ENABLE_RST                            0x0

__INLINE void i2c0_ic_enable_pack(uint8_t abort, uint8_t enable)
{
    _PICO_REG_WR(I2C0_IC_ENABLE_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)abort << 1) | ((uint32_t)enable << 0));
}

__INLINE void i2c0_ic_enable_unpack(uint8_t* abort, uint8_t* enable)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ENABLE_OFFSET + I2C0_BASE_ADDR);

    *abort = (localVal & ((uint32_t)0x00000002)) >> 1;
    *enable = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_enable_abort_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ENABLE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void i2c0_ic_enable_abort_setf(uint8_t abort)
{
    _PICO_REG_WR(I2C0_IC_ENABLE_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_ENABLE_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)abort << 1));
}

__INLINE uint8_t i2c0_ic_enable_enable_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ENABLE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void i2c0_ic_enable_enable_setf(uint8_t enable)
{
    _PICO_REG_WR(I2C0_IC_ENABLE_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_ENABLE_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)enable << 0));
}

 /**
 * @brief IC_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     06         SLV_ACTIVITY   0
 *     05        MST_ACTIVITY_   0
 *     04                  RFF   0
 *     03                 RFNE   0
 *     02                  TFE   1
 *     01                 TFNF   1
 *     00             ACTIVITY   0
 * </pre>
 */
#define I2C0_IC_STATUS_OFFSET 0x00000070


__INLINE uint32_t i2c0_ic_status_get(void)
{
    return _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_STATUS_SLV_ACTIVITY_BIT                      ((uint32_t)0x00000040)
#define I2C0_IC_STATUS_SLV_ACTIVITY_POS                      6
#define I2C0_IC_STATUS_MST_ACTIVITY__BIT                     ((uint32_t)0x00000020)
#define I2C0_IC_STATUS_MST_ACTIVITY__POS                     5
#define I2C0_IC_STATUS_RFF_BIT                               ((uint32_t)0x00000010)
#define I2C0_IC_STATUS_RFF_POS                               4
#define I2C0_IC_STATUS_RFNE_BIT                              ((uint32_t)0x00000008)
#define I2C0_IC_STATUS_RFNE_POS                              3
#define I2C0_IC_STATUS_TFE_BIT                               ((uint32_t)0x00000004)
#define I2C0_IC_STATUS_TFE_POS                               2
#define I2C0_IC_STATUS_TFNF_BIT                              ((uint32_t)0x00000002)
#define I2C0_IC_STATUS_TFNF_POS                              1
#define I2C0_IC_STATUS_ACTIVITY_BIT                          ((uint32_t)0x00000001)
#define I2C0_IC_STATUS_ACTIVITY_POS                          0

#define I2C0_IC_STATUS_SLV_ACTIVITY_RST                      0x0
#define I2C0_IC_STATUS_MST_ACTIVITY__RST                     0x0
#define I2C0_IC_STATUS_RFF_RST                               0x0
#define I2C0_IC_STATUS_RFNE_RST                              0x0
#define I2C0_IC_STATUS_TFE_RST                               0x1
#define I2C0_IC_STATUS_TFNF_RST                              0x1
#define I2C0_IC_STATUS_ACTIVITY_RST                          0x0

__INLINE void i2c0_ic_status_unpack(uint8_t* slvactivity, uint8_t* mstactivity, uint8_t* rff, uint8_t* rfne, uint8_t* tfe, uint8_t* tfnf, uint8_t* activity)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);

    *slvactivity = (localVal & ((uint32_t)0x00000040)) >> 6;
    *mstactivity = (localVal & ((uint32_t)0x00000020)) >> 5;
    *rff = (localVal & ((uint32_t)0x00000010)) >> 4;
    *rfne = (localVal & ((uint32_t)0x00000008)) >> 3;
    *tfe = (localVal & ((uint32_t)0x00000004)) >> 2;
    *tfnf = (localVal & ((uint32_t)0x00000002)) >> 1;
    *activity = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_status_slv_activity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t i2c0_ic_status_mst_activity__getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t i2c0_ic_status_rff_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t i2c0_ic_status_rfne_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t i2c0_ic_status_tfe_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t i2c0_ic_status_tfnf_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t i2c0_ic_status_activity_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_TXFLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00                TXFLR   0b0
 * </pre>
 */
#define I2C0_IC_TXFLR_OFFSET 0x00000074


__INLINE uint32_t i2c0_ic_txflr_get(void)
{
    return _PICO_REG_RD(I2C0_IC_TXFLR_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_TXFLR_TXFLR_MASK                            ((uint32_t)0x0000000F)
#define I2C0_IC_TXFLR_TXFLR_LSB                             0
#define I2C0_IC_TXFLR_TXFLR_WIDTH                           ((uint32_t)0x00000004)

#define I2C0_IC_TXFLR_TXFLR_RST                             0x0

__INLINE void i2c0_ic_txflr_unpack(uint8_t* txflr)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TXFLR_OFFSET + I2C0_BASE_ADDR);

    *txflr = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t i2c0_ic_txflr_txflr_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TXFLR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief IC_RXFLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  03:00               RXFLR_   0b0
 * </pre>
 */
#define I2C0_IC_RXFLR_OFFSET 0x00000078


__INLINE uint32_t i2c0_ic_rxflr_get(void)
{
    return _PICO_REG_RD(I2C0_IC_RXFLR_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_RXFLR_RXFLR__MASK                           ((uint32_t)0x0000000F)
#define I2C0_IC_RXFLR_RXFLR__LSB                            0
#define I2C0_IC_RXFLR_RXFLR__WIDTH                          ((uint32_t)0x00000004)

#define I2C0_IC_RXFLR_RXFLR__RST                            0x0

__INLINE void i2c0_ic_rxflr_unpack(uint8_t* rxflr)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RXFLR_OFFSET + I2C0_BASE_ADDR);

    *rxflr = (localVal & ((uint32_t)0x0000000F)) >> 0;
}

__INLINE uint8_t i2c0_ic_rxflr_rxflr__getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_RXFLR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000F)) >> 0);
}

 /**
 * @brief IC_SDA_HOLD register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          IC_SDA_HOLD   0b1
 * </pre>
 */
#define I2C0_IC_SDA_HOLD_OFFSET 0x0000007C


__INLINE uint32_t i2c0_ic_sda_hold_get(void)
{
    return _PICO_REG_RD(I2C0_IC_SDA_HOLD_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_sda_hold_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_SDA_HOLD_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_SDA_HOLD_IC_SDA_HOLD_MASK                      ((uint32_t)0x0000FFFF)
#define I2C0_IC_SDA_HOLD_IC_SDA_HOLD_LSB                       0
#define I2C0_IC_SDA_HOLD_IC_SDA_HOLD_WIDTH                     ((uint32_t)0x00000010)

#define I2C0_IC_SDA_HOLD_IC_SDA_HOLD_RST                       0x1

__INLINE void i2c0_ic_sda_hold_pack(uint16_t icsdahold)
{
    _PICO_REG_WR(I2C0_IC_SDA_HOLD_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icsdahold << 0));
}

__INLINE void i2c0_ic_sda_hold_unpack(uint8_t* icsdahold)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SDA_HOLD_OFFSET + I2C0_BASE_ADDR);

    *icsdahold = (localVal & ((uint32_t)0x0000FFFF)) >> 0;
}

__INLINE uint16_t i2c0_ic_sda_hold_ic_sda_hold_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SDA_HOLD_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FFFF)) >> 0);
}

__INLINE void i2c0_ic_sda_hold_ic_sda_hold_setf(uint16_t icsdahold)
{
    _PICO_REG_WR(I2C0_IC_SDA_HOLD_OFFSET+ I2C0_BASE_ADDR, (uint32_t)icsdahold << 0);
}

 /**
 * @brief IC_TX_ABRT_SOURCE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:24         TX_FLUSH_CNT   0b0
 *     16       ABRT_USER_ABRT   0
 *     15      ABRT_SLVRD_INTX   0
 *     14     ABRT_SLV_ARBLOST   0
 *     13   ABRT_SLVFLUSH_TXFIFO   0
 *     12             ARB_LOST   0
 *     11      ABRT_MASTER_DIS   0
 *     10   ABRT_10B_RD_NORSTRT   0
 *     09   ABRT_SBYTE_NORSTRT   0
 *     08      ABRT_HS_NORSTRT   0
 *     07    ABRT_SBYTE_ACKDET   0
 *     06       ABRT_HS_ACKDET   0
 *     05      ABRT_GCALL_READ   0
 *     04     ABRT_GCALL_NOACK   0
 *     03    ABRT_TXDATA_NOACK   0
 *     02   ABRT_10ADDR2_NOACK   0
 *     01   ABRT_10ADDR1_NOACK   0
 *     00   ABRT_7B_ADDR_NOACK_   0
 * </pre>
 */
#define I2C0_IC_TX_ABRT_SOURCE_OFFSET 0x00000080


__INLINE uint32_t i2c0_ic_tx_abrt_source_get(void)
{
    return _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_MASK                     ((uint32_t)0xFF000000)
#define I2C0_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_LSB                      24
#define I2C0_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_WIDTH                    ((uint32_t)0x00000008)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_BIT                    ((uint32_t)0x00010000)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_POS                    16
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_BIT                   ((uint32_t)0x00008000)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_POS                   15
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_BIT                  ((uint32_t)0x00004000)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_POS                  14
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_BIT              ((uint32_t)0x00002000)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_POS              13
#define I2C0_IC_TX_ABRT_SOURCE_ARB_LOST_BIT                          ((uint32_t)0x00001000)
#define I2C0_IC_TX_ABRT_SOURCE_ARB_LOST_POS                          12
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_BIT                   ((uint32_t)0x00000800)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_POS                   11
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT_BIT               ((uint32_t)0x00000400)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT_POS               10
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT_BIT                ((uint32_t)0x00000200)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT_POS                9
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT_BIT                   ((uint32_t)0x00000100)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT_POS                   8
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_BIT                 ((uint32_t)0x00000080)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_POS                 7
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_BIT                    ((uint32_t)0x00000040)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_POS                    6
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_BIT                   ((uint32_t)0x00000020)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_POS                   5
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_BIT                  ((uint32_t)0x00000010)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_POS                  4
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BIT                 ((uint32_t)0x00000008)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_POS                 3
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_BIT                ((uint32_t)0x00000004)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_POS                2
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_BIT                ((uint32_t)0x00000002)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_POS                1
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK__BIT               ((uint32_t)0x00000001)
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK__POS               0

#define I2C0_IC_TX_ABRT_SOURCE_TX_FLUSH_CNT_RST                      0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_USER_ABRT_RST                    0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLVRD_INTX_RST                   0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLV_ARBLOST_RST                  0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SLVFLUSH_TXFIFO_RST              0x0
#define I2C0_IC_TX_ABRT_SOURCE_ARB_LOST_RST                          0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_MASTER_DIS_RST                   0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10B_RD_NORSTRT_RST               0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SBYTE_NORSTRT_RST                0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_HS_NORSTRT_RST                   0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_SBYTE_ACKDET_RST                 0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_HS_ACKDET_RST                    0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_GCALL_READ_RST                   0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_GCALL_NOACK_RST                  0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_RST                 0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10ADDR2_NOACK_RST                0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_10ADDR1_NOACK_RST                0x0
#define I2C0_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK__RST               0x0

__INLINE void i2c0_ic_tx_abrt_source_unpack(uint8_t* txflushcnt, uint8_t* abrtuserabrt, uint8_t* abrtslvrdintx, uint8_t* abrtslvarblost, uint8_t* abrtslvflushtxfifo, uint8_t* arblost, uint8_t* abrtmasterdis, uint8_t* abrt10brdnorstrt, uint8_t* abrtsbytenorstrt, uint8_t* abrthsnorstrt, uint8_t* abrtsbyteackdet, uint8_t* abrthsackdet, uint8_t* abrtgcallread, uint8_t* abrtgcallnoack, uint8_t* abrttxdatanoack, uint8_t* abrt10addr2noack, uint8_t* abrt10addr1noack, uint8_t* abrt7baddrnoack)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);

    *txflushcnt = (localVal & ((uint32_t)0xFF000000)) >> 24;
    *abrtuserabrt = (localVal & ((uint32_t)0x00010000)) >> 16;
    *abrtslvrdintx = (localVal & ((uint32_t)0x00008000)) >> 15;
    *abrtslvarblost = (localVal & ((uint32_t)0x00004000)) >> 14;
    *abrtslvflushtxfifo = (localVal & ((uint32_t)0x00002000)) >> 13;
    *arblost = (localVal & ((uint32_t)0x00001000)) >> 12;
    *abrtmasterdis = (localVal & ((uint32_t)0x00000800)) >> 11;
    *abrt10brdnorstrt = (localVal & ((uint32_t)0x00000400)) >> 10;
    *abrtsbytenorstrt = (localVal & ((uint32_t)0x00000200)) >> 9;
    *abrthsnorstrt = (localVal & ((uint32_t)0x00000100)) >> 8;
    *abrtsbyteackdet = (localVal & ((uint32_t)0x00000080)) >> 7;
    *abrthsackdet = (localVal & ((uint32_t)0x00000040)) >> 6;
    *abrtgcallread = (localVal & ((uint32_t)0x00000020)) >> 5;
    *abrtgcallnoack = (localVal & ((uint32_t)0x00000010)) >> 4;
    *abrttxdatanoack = (localVal & ((uint32_t)0x00000008)) >> 3;
    *abrt10addr2noack = (localVal & ((uint32_t)0x00000004)) >> 2;
    *abrt10addr1noack = (localVal & ((uint32_t)0x00000002)) >> 1;
    *abrt7baddrnoack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_tx_flush_cnt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFF000000)) >> 24);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_user_abrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00010000)) >> 16);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_slvrd_intx_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00008000)) >> 15);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_slv_arblost_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00004000)) >> 14);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_slvflush_txfifo_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00002000)) >> 13);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_arb_lost_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00001000)) >> 12);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_master_dis_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000800)) >> 11);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_10b_rd_norstrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000400)) >> 10);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_sbyte_norstrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000200)) >> 9);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_hs_norstrt_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000100)) >> 8);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_sbyte_ackdet_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_hs_ackdet_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_gcall_read_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_gcall_noack_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_txdata_noack_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000008)) >> 3);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_10addr2_noack_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_10addr1_noack_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t i2c0_ic_tx_abrt_source_abrt_7b_addr_noack__getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_TX_ABRT_SOURCE_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_SLV_DATA_NACK_ONLY register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00                 NACK   0
 * </pre>
 */
#define I2C0_IC_SLV_DATA_NACK_ONLY_OFFSET 0x00000084


__INLINE uint32_t i2c0_ic_slv_data_nack_only_get(void)
{
    return _PICO_REG_RD(I2C0_IC_SLV_DATA_NACK_ONLY_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_slv_data_nack_only_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_SLV_DATA_NACK_ONLY_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_SLV_DATA_NACK_ONLY_NACK_BIT                              ((uint32_t)0x00000001)
#define I2C0_IC_SLV_DATA_NACK_ONLY_NACK_POS                              0

#define I2C0_IC_SLV_DATA_NACK_ONLY_NACK_RST                              0x0

__INLINE void i2c0_ic_slv_data_nack_only_pack(uint8_t nack)
{
    _PICO_REG_WR(I2C0_IC_SLV_DATA_NACK_ONLY_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)nack << 0));
}

__INLINE void i2c0_ic_slv_data_nack_only_unpack(uint8_t* nack)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SLV_DATA_NACK_ONLY_OFFSET + I2C0_BASE_ADDR);

    *nack = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_slv_data_nack_only_nack_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SLV_DATA_NACK_ONLY_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void i2c0_ic_slv_data_nack_only_nack_setf(uint8_t nack)
{
    _PICO_REG_WR(I2C0_IC_SLV_DATA_NACK_ONLY_OFFSET+ I2C0_BASE_ADDR, (uint32_t)nack << 0);
}

 /**
 * @brief IC_DMA_CR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     01                TDMAE   0
 *     00                RDMAE   0
 * </pre>
 */
#define I2C0_IC_DMA_CR_OFFSET 0x00000088


__INLINE uint32_t i2c0_ic_dma_cr_get(void)
{
    return _PICO_REG_RD(I2C0_IC_DMA_CR_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_dma_cr_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_DMA_CR_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_DMA_CR_TDMAE_BIT                             ((uint32_t)0x00000002)
#define I2C0_IC_DMA_CR_TDMAE_POS                             1
#define I2C0_IC_DMA_CR_RDMAE_BIT                             ((uint32_t)0x00000001)
#define I2C0_IC_DMA_CR_RDMAE_POS                             0

#define I2C0_IC_DMA_CR_TDMAE_RST                             0x0
#define I2C0_IC_DMA_CR_RDMAE_RST                             0x0

__INLINE void i2c0_ic_dma_cr_pack(uint8_t tdmae, uint8_t rdmae)
{
    _PICO_REG_WR(I2C0_IC_DMA_CR_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)tdmae << 1) | ((uint32_t)rdmae << 0));
}

__INLINE void i2c0_ic_dma_cr_unpack(uint8_t* tdmae, uint8_t* rdmae)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DMA_CR_OFFSET + I2C0_BASE_ADDR);

    *tdmae = (localVal & ((uint32_t)0x00000002)) >> 1;
    *rdmae = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_dma_cr_tdmae_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DMA_CR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE void i2c0_ic_dma_cr_tdmae_setf(uint8_t tdmae)
{
    _PICO_REG_WR(I2C0_IC_DMA_CR_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_DMA_CR_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000002)) | ((uint32_t)tdmae << 1));
}

__INLINE uint8_t i2c0_ic_dma_cr_rdmae_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DMA_CR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void i2c0_ic_dma_cr_rdmae_setf(uint8_t rdmae)
{
    _PICO_REG_WR(I2C0_IC_DMA_CR_OFFSET+ I2C0_BASE_ADDR, (_PICO_REG_RD(I2C0_IC_DMA_CR_OFFSET + I2C0_BASE_ADDR) & ~((uint32_t)0x00000001)) | ((uint32_t)rdmae << 0));
}

 /**
 * @brief IC_DMA_TDLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00               DMATDL   0b0
 * </pre>
 */
#define I2C0_IC_DMA_TDLR_OFFSET 0x0000008C


__INLINE uint32_t i2c0_ic_dma_tdlr_get(void)
{
    return _PICO_REG_RD(I2C0_IC_DMA_TDLR_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_dma_tdlr_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_DMA_TDLR_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_DMA_TDLR_DMATDL_MASK                           ((uint32_t)0x00000007)
#define I2C0_IC_DMA_TDLR_DMATDL_LSB                            0
#define I2C0_IC_DMA_TDLR_DMATDL_WIDTH                          ((uint32_t)0x00000003)

#define I2C0_IC_DMA_TDLR_DMATDL_RST                            0x0

__INLINE void i2c0_ic_dma_tdlr_pack(uint8_t dmatdl)
{
    _PICO_REG_WR(I2C0_IC_DMA_TDLR_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)dmatdl << 0));
}

__INLINE void i2c0_ic_dma_tdlr_unpack(uint8_t* dmatdl)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DMA_TDLR_OFFSET + I2C0_BASE_ADDR);

    *dmatdl = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t i2c0_ic_dma_tdlr_dmatdl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DMA_TDLR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void i2c0_ic_dma_tdlr_dmatdl_setf(uint8_t dmatdl)
{
    _PICO_REG_WR(I2C0_IC_DMA_TDLR_OFFSET+ I2C0_BASE_ADDR, (uint32_t)dmatdl << 0);
}

 /**
 * @brief IC_DMA_RDLR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  02:00               DMARDL   0b0
 * </pre>
 */
#define I2C0_IC_DMA_RDLR_OFFSET 0x00000090


__INLINE uint32_t i2c0_ic_dma_rdlr_get(void)
{
    return _PICO_REG_RD(I2C0_IC_DMA_RDLR_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_dma_rdlr_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_DMA_RDLR_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_DMA_RDLR_DMARDL_MASK                           ((uint32_t)0x00000007)
#define I2C0_IC_DMA_RDLR_DMARDL_LSB                            0
#define I2C0_IC_DMA_RDLR_DMARDL_WIDTH                          ((uint32_t)0x00000003)

#define I2C0_IC_DMA_RDLR_DMARDL_RST                            0x0

__INLINE void i2c0_ic_dma_rdlr_pack(uint8_t dmardl)
{
    _PICO_REG_WR(I2C0_IC_DMA_RDLR_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)dmardl << 0));
}

__INLINE void i2c0_ic_dma_rdlr_unpack(uint8_t* dmardl)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DMA_RDLR_OFFSET + I2C0_BASE_ADDR);

    *dmardl = (localVal & ((uint32_t)0x00000007)) >> 0;
}

__INLINE uint8_t i2c0_ic_dma_rdlr_dmardl_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_DMA_RDLR_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000007)) >> 0);
}

__INLINE void i2c0_ic_dma_rdlr_dmardl_setf(uint8_t dmardl)
{
    _PICO_REG_WR(I2C0_IC_DMA_RDLR_OFFSET+ I2C0_BASE_ADDR, (uint32_t)dmardl << 0);
}

 /**
 * @brief IC_SDA_SETUP register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00            SDA_SETUP   0b64
 * </pre>
 */
#define I2C0_IC_SDA_SETUP_OFFSET 0x00000094


__INLINE uint32_t i2c0_ic_sda_setup_get(void)
{
    return _PICO_REG_RD(I2C0_IC_SDA_SETUP_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_sda_setup_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_SDA_SETUP_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_SDA_SETUP_SDA_SETUP_MASK                        ((uint32_t)0x000000FF)
#define I2C0_IC_SDA_SETUP_SDA_SETUP_LSB                         0
#define I2C0_IC_SDA_SETUP_SDA_SETUP_WIDTH                       ((uint32_t)0x00000008)

#define I2C0_IC_SDA_SETUP_SDA_SETUP_RST                         0x64

__INLINE void i2c0_ic_sda_setup_pack(uint8_t sdasetup)
{
    _PICO_REG_WR(I2C0_IC_SDA_SETUP_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)sdasetup << 0));
}

__INLINE void i2c0_ic_sda_setup_unpack(uint8_t* sdasetup)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SDA_SETUP_OFFSET + I2C0_BASE_ADDR);

    *sdasetup = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t i2c0_ic_sda_setup_sda_setup_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_SDA_SETUP_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void i2c0_ic_sda_setup_sda_setup_setf(uint8_t sdasetup)
{
    _PICO_REG_WR(I2C0_IC_SDA_SETUP_OFFSET+ I2C0_BASE_ADDR, (uint32_t)sdasetup << 0);
}

 /**
 * @brief IC_ACK_GENERAL_CALL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00         ACK_GEN_CALL   1
 * </pre>
 */
#define I2C0_IC_ACK_GENERAL_CALL_OFFSET 0x00000098


__INLINE uint32_t i2c0_ic_ack_general_call_get(void)
{
    return _PICO_REG_RD(I2C0_IC_ACK_GENERAL_CALL_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_ack_general_call_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_ACK_GENERAL_CALL_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_BIT                      ((uint32_t)0x00000001)
#define I2C0_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_POS                      0

#define I2C0_IC_ACK_GENERAL_CALL_ACK_GEN_CALL_RST                      0x1

__INLINE void i2c0_ic_ack_general_call_pack(uint8_t ackgencall)
{
    _PICO_REG_WR(I2C0_IC_ACK_GENERAL_CALL_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)ackgencall << 0));
}

__INLINE void i2c0_ic_ack_general_call_unpack(uint8_t* ackgencall)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ACK_GENERAL_CALL_OFFSET + I2C0_BASE_ADDR);

    *ackgencall = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_ack_general_call_ack_gen_call_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ACK_GENERAL_CALL_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

__INLINE void i2c0_ic_ack_general_call_ack_gen_call_setf(uint8_t ackgencall)
{
    _PICO_REG_WR(I2C0_IC_ACK_GENERAL_CALL_OFFSET+ I2C0_BASE_ADDR, (uint32_t)ackgencall << 0);
}

 /**
 * @brief IC_ENABLE_STATUS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     02     SLV_RX_DATA_LOST   0
 *     01   SLV_DISABLED_WHILE_BUSY   0
 *     00                IC_EN   0
 * </pre>
 */
#define I2C0_IC_ENABLE_STATUS_OFFSET 0x0000009C


__INLINE uint32_t i2c0_ic_enable_status_get(void)
{
    return _PICO_REG_RD(I2C0_IC_ENABLE_STATUS_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_BIT                  ((uint32_t)0x00000004)
#define I2C0_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_POS                  2
#define I2C0_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_BIT           ((uint32_t)0x00000002)
#define I2C0_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_POS           1
#define I2C0_IC_ENABLE_STATUS_IC_EN_BIT                             ((uint32_t)0x00000001)
#define I2C0_IC_ENABLE_STATUS_IC_EN_POS                             0

#define I2C0_IC_ENABLE_STATUS_SLV_RX_DATA_LOST_RST                  0x0
#define I2C0_IC_ENABLE_STATUS_SLV_DISABLED_WHILE_BUSY_RST           0x0
#define I2C0_IC_ENABLE_STATUS_IC_EN_RST                             0x0

__INLINE void i2c0_ic_enable_status_unpack(uint8_t* slvrxdatalost, uint8_t* slvdisabledwhilebusy, uint8_t* icen)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ENABLE_STATUS_OFFSET + I2C0_BASE_ADDR);

    *slvrxdatalost = (localVal & ((uint32_t)0x00000004)) >> 2;
    *slvdisabledwhilebusy = (localVal & ((uint32_t)0x00000002)) >> 1;
    *icen = (localVal & ((uint32_t)0x00000001)) >> 0;
}

__INLINE uint8_t i2c0_ic_enable_status_slv_rx_data_lost_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ENABLE_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000004)) >> 2);
}

__INLINE uint8_t i2c0_ic_enable_status_slv_disabled_while_busy_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ENABLE_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000002)) >> 1);
}

__INLINE uint8_t i2c0_ic_enable_status_ic_en_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_ENABLE_STATUS_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000001)) >> 0);
}

 /**
 * @brief IC_FS_SPKLEN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00         IC_FS_SPKLEN   0bFF
 * </pre>
 */
#define I2C0_IC_FS_SPKLEN_OFFSET 0x000000A0


__INLINE uint32_t i2c0_ic_fs_spklen_get(void)
{
    return _PICO_REG_RD(I2C0_IC_FS_SPKLEN_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_fs_spklen_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_FS_SPKLEN_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_FS_SPKLEN_IC_FS_SPKLEN_MASK                     ((uint32_t)0x000000FF)
#define I2C0_IC_FS_SPKLEN_IC_FS_SPKLEN_LSB                      0
#define I2C0_IC_FS_SPKLEN_IC_FS_SPKLEN_WIDTH                    ((uint32_t)0x00000008)

#define I2C0_IC_FS_SPKLEN_IC_FS_SPKLEN_RST                      0xFF

__INLINE void i2c0_ic_fs_spklen_pack(uint8_t icfsspklen)
{
    _PICO_REG_WR(I2C0_IC_FS_SPKLEN_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)icfsspklen << 0));
}

__INLINE void i2c0_ic_fs_spklen_unpack(uint8_t* icfsspklen)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_FS_SPKLEN_OFFSET + I2C0_BASE_ADDR);

    *icfsspklen = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t i2c0_ic_fs_spklen_ic_fs_spklen_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_FS_SPKLEN_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void i2c0_ic_fs_spklen_ic_fs_spklen_setf(uint8_t icfsspklen)
{
    _PICO_REG_WR(I2C0_IC_FS_SPKLEN_OFFSET+ I2C0_BASE_ADDR, (uint32_t)icfsspklen << 0);
}

 /**
 * @brief IC_HS_SPKLEN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00         IC_HS_SPKLEN   0bFF
 * </pre>
 */
#define I2C0_IC_HS_SPKLEN_OFFSET 0x000000A4


__INLINE uint32_t i2c0_ic_hs_spklen_get(void)
{
    return _PICO_REG_RD(I2C0_IC_HS_SPKLEN_OFFSET + I2C0_BASE_ADDR);
}

__INLINE void i2c0_ic_hs_spklen_set(uint32_t value)
{
    _PICO_REG_WR(I2C0_IC_HS_SPKLEN_OFFSET+ I2C0_BASE_ADDR, value);
}

// field definitions
#define I2C0_IC_HS_SPKLEN_IC_HS_SPKLEN_MASK                     ((uint32_t)0x000000FF)
#define I2C0_IC_HS_SPKLEN_IC_HS_SPKLEN_LSB                      0
#define I2C0_IC_HS_SPKLEN_IC_HS_SPKLEN_WIDTH                    ((uint32_t)0x00000008)

#define I2C0_IC_HS_SPKLEN_IC_HS_SPKLEN_RST                      0xFF

__INLINE void i2c0_ic_hs_spklen_pack(uint8_t ichsspklen)
{
    _PICO_REG_WR(I2C0_IC_HS_SPKLEN_OFFSET+ I2C0_BASE_ADDR,  ((uint32_t)ichsspklen << 0));
}

__INLINE void i2c0_ic_hs_spklen_unpack(uint8_t* ichsspklen)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_SPKLEN_OFFSET + I2C0_BASE_ADDR);

    *ichsspklen = (localVal & ((uint32_t)0x000000FF)) >> 0;
}

__INLINE uint8_t i2c0_ic_hs_spklen_ic_hs_spklen_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_HS_SPKLEN_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x000000FF)) >> 0);
}

__INLINE void i2c0_ic_hs_spklen_ic_hs_spklen_setf(uint8_t ichsspklen)
{
    _PICO_REG_WR(I2C0_IC_HS_SPKLEN_OFFSET+ I2C0_BASE_ADDR, (uint32_t)ichsspklen << 0);
}

 /**
 * @brief IC_COMP_PARAM_1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  23:16      TX_BUFFER_DEPTH   0b0
 *  15:08      RX_BUFFER_DEPTH   0b0
 *     07   ADD_ENCODED_PARAMS   0
 *     06              HAS_DMA   0
 *     05              INTR_IO   0
 *     04      HC_COUNT_VALUES   0
 *  03:02       MAX_SPEED_MODE   0b0
 *  01:00       APB_DATA_WIDTH   0b0
 * </pre>
 */
#define I2C0_IC_COMP_PARAM_1_OFFSET 0x000000F4


__INLINE uint32_t i2c0_ic_comp_param_1_get(void)
{
    return _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_MASK                  ((uint32_t)0x00FF0000)
#define I2C0_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_LSB                   16
#define I2C0_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_WIDTH                 ((uint32_t)0x00000008)
#define I2C0_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_MASK                  ((uint32_t)0x0000FF00)
#define I2C0_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_LSB                   8
#define I2C0_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_WIDTH                 ((uint32_t)0x00000008)
#define I2C0_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS_BIT                ((uint32_t)0x00000080)
#define I2C0_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS_POS                7
#define I2C0_IC_COMP_PARAM_1_HAS_DMA_BIT                           ((uint32_t)0x00000040)
#define I2C0_IC_COMP_PARAM_1_HAS_DMA_POS                           6
#define I2C0_IC_COMP_PARAM_1_INTR_IO_BIT                           ((uint32_t)0x00000020)
#define I2C0_IC_COMP_PARAM_1_INTR_IO_POS                           5
#define I2C0_IC_COMP_PARAM_1_HC_COUNT_VALUES_BIT                   ((uint32_t)0x00000010)
#define I2C0_IC_COMP_PARAM_1_HC_COUNT_VALUES_POS                   4
#define I2C0_IC_COMP_PARAM_1_MAX_SPEED_MODE_MASK                   ((uint32_t)0x0000000C)
#define I2C0_IC_COMP_PARAM_1_MAX_SPEED_MODE_LSB                    2
#define I2C0_IC_COMP_PARAM_1_MAX_SPEED_MODE_WIDTH                  ((uint32_t)0x00000002)
#define I2C0_IC_COMP_PARAM_1_APB_DATA_WIDTH_MASK                   ((uint32_t)0x00000003)
#define I2C0_IC_COMP_PARAM_1_APB_DATA_WIDTH_LSB                    0
#define I2C0_IC_COMP_PARAM_1_APB_DATA_WIDTH_WIDTH                  ((uint32_t)0x00000002)

#define I2C0_IC_COMP_PARAM_1_TX_BUFFER_DEPTH_RST                   0x0
#define I2C0_IC_COMP_PARAM_1_RX_BUFFER_DEPTH_RST                   0x0
#define I2C0_IC_COMP_PARAM_1_ADD_ENCODED_PARAMS_RST                0x0
#define I2C0_IC_COMP_PARAM_1_HAS_DMA_RST                           0x0
#define I2C0_IC_COMP_PARAM_1_INTR_IO_RST                           0x0
#define I2C0_IC_COMP_PARAM_1_HC_COUNT_VALUES_RST                   0x0
#define I2C0_IC_COMP_PARAM_1_MAX_SPEED_MODE_RST                    0x0
#define I2C0_IC_COMP_PARAM_1_APB_DATA_WIDTH_RST                    0x0

__INLINE void i2c0_ic_comp_param_1_unpack(uint8_t* txbufferdepth, uint8_t* rxbufferdepth, uint8_t* addencodedparams, uint8_t* hasdma, uint8_t* intrio, uint8_t* hccountvalues, uint8_t* maxspeedmode, uint8_t* apbdatawidth)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);

    *txbufferdepth = (localVal & ((uint32_t)0x00FF0000)) >> 16;
    *rxbufferdepth = (localVal & ((uint32_t)0x0000FF00)) >> 8;
    *addencodedparams = (localVal & ((uint32_t)0x00000080)) >> 7;
    *hasdma = (localVal & ((uint32_t)0x00000040)) >> 6;
    *intrio = (localVal & ((uint32_t)0x00000020)) >> 5;
    *hccountvalues = (localVal & ((uint32_t)0x00000010)) >> 4;
    *maxspeedmode = (localVal & ((uint32_t)0x0000000C)) >> 2;
    *apbdatawidth = (localVal & ((uint32_t)0x00000003)) >> 0;
}

__INLINE uint8_t i2c0_ic_comp_param_1_tx_buffer_depth_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00FF0000)) >> 16);
}

__INLINE uint8_t i2c0_ic_comp_param_1_rx_buffer_depth_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t i2c0_ic_comp_param_1_add_encoded_params_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000080)) >> 7);
}

__INLINE uint8_t i2c0_ic_comp_param_1_has_dma_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000040)) >> 6);
}

__INLINE uint8_t i2c0_ic_comp_param_1_intr_io_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000020)) >> 5);
}

__INLINE uint8_t i2c0_ic_comp_param_1_hc_count_values_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000010)) >> 4);
}

__INLINE uint8_t i2c0_ic_comp_param_1_max_speed_mode_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x0000000C)) >> 2);
}

__INLINE uint8_t i2c0_ic_comp_param_1_apb_data_width_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_PARAM_1_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0x00000003)) >> 0);
}

 /**
 * @brief IC_COMP_VERSION register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00      IC_COMP_VERSION   0b0
 * </pre>
 */
#define I2C0_IC_COMP_VERSION_OFFSET 0x000000F8


__INLINE uint32_t i2c0_ic_comp_version_get(void)
{
    return _PICO_REG_RD(I2C0_IC_COMP_VERSION_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_COMP_VERSION_IC_COMP_VERSION_MASK                  ((uint32_t)0xFFFFFFFF)
#define I2C0_IC_COMP_VERSION_IC_COMP_VERSION_LSB                   0
#define I2C0_IC_COMP_VERSION_IC_COMP_VERSION_WIDTH                 ((uint32_t)0x00000020)

#define I2C0_IC_COMP_VERSION_IC_COMP_VERSION_RST                   0x0

__INLINE void i2c0_ic_comp_version_unpack(uint8_t* iccompversion)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_VERSION_OFFSET + I2C0_BASE_ADDR);

    *iccompversion = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t i2c0_ic_comp_version_ic_comp_version_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_VERSION_OFFSET + I2C0_BASE_ADDR);
    return ((localVal & ((uint32_t)0xFFFFFFFF)) >> 0);
}

 /**
 * @brief IC_COMP_TYPE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  31:00         IC_COMP_TYPE   0b0
 * </pre>
 */
#define I2C0_IC_COMP_TYPE_OFFSET 0x000000FC


__INLINE uint32_t i2c0_ic_comp_type_get(void)
{
    return _PICO_REG_RD(I2C0_IC_COMP_TYPE_OFFSET + I2C0_BASE_ADDR);
}

// field definitions
#define I2C0_IC_COMP_TYPE_IC_COMP_TYPE_MASK                     ((uint32_t)0xFFFFFFFF)
#define I2C0_IC_COMP_TYPE_IC_COMP_TYPE_LSB                      0
#define I2C0_IC_COMP_TYPE_IC_COMP_TYPE_WIDTH                    ((uint32_t)0x00000020)

#define I2C0_IC_COMP_TYPE_IC_COMP_TYPE_RST                      0x0

__INLINE void i2c0_ic_comp_type_unpack(uint8_t* iccomptype)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_TYPE_OFFSET + I2C0_BASE_ADDR);

    *iccomptype = (localVal & ((uint32_t)0xFFFFFFFF)) >> 0;
}

__INLINE uint32_t i2c0_ic_comp_type_ic_comp_type_getf(void)
{
    volatile uint32_t localVal = _PICO_REG_RD(I2C0_IC_COMP_TYPE_OFFSET + I2C0_BASE_ADDR);
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
      __IO uint32_t :25;
      __IO uint32_t ic_slave_disable:1;
      __IO uint32_t ic_restart_en:1;
      __IO uint32_t ic_10bitaddr_master:1;
      __IO uint32_t ic_10bitaddr_slave:1;
      __IO uint32_t speed:2;
      __IO uint32_t master_mode:1;
    }IC_CON_fld;
    __IO uint32_t IC_CON;
  };

  union{ //offset addr 0x0004
    struct{
      __IO uint32_t :19;
      __IO uint32_t ic_10bitaddr_master:1;
      __IO uint32_t special:1;
      __IO uint32_t gc_or_start:1;
      __IO uint32_t ic_tar:10;
    }IC_TAR_fld;
    __IO uint32_t IC_TAR;
  };

  union{ //offset addr 0x0008
    struct{
      __IO uint32_t :22;
      __IO uint32_t ic_sar:10;
    }IC_SAR_fld;
    __IO uint32_t IC_SAR;
  };

  union{ //offset addr 0x000c
    struct{
      __IO uint32_t :29;
      __IO uint32_t ic_hs_mar:3;
    }IC_HS_MADDR_fld;
    __IO uint32_t IC_HS_MADDR;
  };

  union{ //offset addr 0x0010
    struct{
      __IO uint32_t :23;
      __IO uint32_t cmd:1;
      __IO uint32_t dat:8;
    }IC_DATA_CMD_fld;
    __IO uint32_t IC_DATA_CMD;
  };

  union{ //offset addr 0x0014
    struct{
      __IO uint32_t :16;
      __IO uint32_t ic_ss_scl_hcnt:16;
    }IC_SS_SCL_HCNT_fld;
    __IO uint32_t IC_SS_SCL_HCNT;
  };

  union{ //offset addr 0x0018
    struct{
      __IO uint32_t :16;
      __IO uint32_t ic_ss_scl_lcnt:16;
    }IC_SS_SCL_LCNT_fld;
    __IO uint32_t IC_SS_SCL_LCNT;
  };

  union{ //offset addr 0x001c
    struct{
      __IO uint32_t :16;
      __IO uint32_t ic_fs_scl_hcnt:16;
    }IC_FS_SCL_HCNT_fld;
    __IO uint32_t IC_FS_SCL_HCNT;
  };

  union{ //offset addr 0x0020
    struct{
      __IO uint32_t :16;
      __IO uint32_t ic_fs_scl_lcnt:16;
    }IC_FS_SCL_LCNT_fld;
    __IO uint32_t IC_FS_SCL_LCNT;
  };

  union{ //offset addr 0x0024
    struct{
      __IO uint32_t :16;
      __IO uint32_t ic_hs_scl_hcnt:16;
    }IC_HS_SCL_HCNT_fld;
    __IO uint32_t IC_HS_SCL_HCNT;
  };

  union{ //offset addr 0x0028
    struct{
      __IO uint32_t :16;
      __IO uint32_t ic_hs_scl_lcnt:16;
    }IC_HS_SCL_LCNT_fld;
    __IO uint32_t IC_HS_SCL_LCNT;
  };

  union{ //offset addr 0x002c
    struct{
      __IO uint32_t :20;
      __IO uint32_t r_gen_call:1;
      __IO uint32_t r_start_det:1;
      __IO uint32_t r_stop_det:1;
      __IO uint32_t r_activity:1;
      __IO uint32_t r_rx_done:1;
      __IO uint32_t r_tx_abrt:1;
      __IO uint32_t r_rd_req:1;
      __IO uint32_t r_tx_empty:1;
      __IO uint32_t r_tx_over:1;
      __IO uint32_t r_rx_full:1;
      __IO uint32_t r_rx_over:1;
      __IO uint32_t r_rx_under:1;
    }IC_INTR_STAT_fld;
    __IO uint32_t IC_INTR_STAT;
  };

  union{ //offset addr 0x0030
    struct{
      __IO uint32_t :20;
      __IO uint32_t r_gen_call:1;
      __IO uint32_t r_start_det:1;
      __IO uint32_t r_stop_det:1;
      __IO uint32_t r_activity:1;
      __IO uint32_t r_rx_done:1;
      __IO uint32_t r_tx_abrt:1;
      __IO uint32_t r_rd_req:1;
      __IO uint32_t r_tx_empty:1;
      __IO uint32_t r_tx_over:1;
      __IO uint32_t r_rx_full:1;
      __IO uint32_t r_rx_over:1;
      __IO uint32_t r_rx_under:1;
    }IC_INTR_MASK_fld;
    __IO uint32_t IC_INTR_MASK;
  };

  union{ //offset addr 0x0034
    struct{
      __IO uint32_t :20;
      __IO uint32_t gen_call:1;
      __IO uint32_t start_det:1;
      __IO uint32_t stop_det:1;
      __IO uint32_t activity:1;
      __IO uint32_t rx_done:1;
      __IO uint32_t tx_abrt:1;
      __IO uint32_t rd_req:1;
      __IO uint32_t tx_empty:1;
      __IO uint32_t tx_over:1;
      __IO uint32_t rx_full:1;
      __IO uint32_t rx_over:1;
      __IO uint32_t rx_under:1;
    }IC_RAW_INTR_STAT_fld;
    __IO uint32_t IC_RAW_INTR_STAT;
  };

  union{ //offset addr 0x0038
    struct{
      __IO uint32_t :24;
      __IO uint32_t rx_tl:8;
    }IC_RX_TL_fld;
    __IO uint32_t IC_RX_TL;
  };

  union{ //offset addr 0x003c
    struct{
      __IO uint32_t :24;
      __IO uint32_t tx_tl:8;
    }IC_TX_TL_fld;
    __IO uint32_t IC_TX_TL;
  };

  union{ //offset addr 0x0040
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_intr:1;
    }IC_CLR_INTR_fld;
    __IO uint32_t IC_CLR_INTR;
  };

  union{ //offset addr 0x0044
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_rx_under:1;
    }IC_CLR_RX_UNDER_fld;
    __IO uint32_t IC_CLR_RX_UNDER;
  };

  union{ //offset addr 0x0048
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_rx_over:1;
    }IC_CLR_RX_OVER_fld;
    __IO uint32_t IC_CLR_RX_OVER;
  };

  union{ //offset addr 0x004c
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_tx_over:1;
    }IC_CLR_TX_OVER_fld;
    __IO uint32_t IC_CLR_TX_OVER;
  };

  union{ //offset addr 0x0050
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_rd_req:1;
    }IC_CLR_RD_REQ_fld;
    __IO uint32_t IC_CLR_RD_REQ;
  };

  union{ //offset addr 0x0054
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_tx_abrt:1;
    }IC_CLR_TX_ABRT_fld;
    __IO uint32_t IC_CLR_TX_ABRT;
  };

  union{ //offset addr 0x0058
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_rx_done:1;
    }IC_CLR_RX_DONE_fld;
    __IO uint32_t IC_CLR_RX_DONE;
  };

  union{ //offset addr 0x005c
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_activity:1;
    }IC_CLR_ACTIVITY_fld;
    __IO uint32_t IC_CLR_ACTIVITY;
  };

  union{ //offset addr 0x0060
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_stop_det:1;
    }IC_CLR_STOP_DET_fld;
    __IO uint32_t IC_CLR_STOP_DET;
  };

  union{ //offset addr 0x0064
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_start_det:1;
    }IC_CLR_START_DET_fld;
    __IO uint32_t IC_CLR_START_DET;
  };

  union{ //offset addr 0x0068
    struct{
      __IO uint32_t :31;
      __IO uint32_t clr_gen_call:1;
    }IC_CLR_GEN_CALL_fld;
    __IO uint32_t IC_CLR_GEN_CALL;
  };

  union{ //offset addr 0x006c
    struct{
      __IO uint32_t :30;
      __IO uint32_t abort:1;
      __IO uint32_t enable:1;
    }IC_ENABLE_fld;
    __IO uint32_t IC_ENABLE;
  };

  union{ //offset addr 0x0070
    struct{
      __IO uint32_t :25;
      __IO uint32_t slv_activity:1;
      __IO uint32_t mst_activity_:1;
      __IO uint32_t rff:1;
      __IO uint32_t rfne:1;
      __IO uint32_t tfe:1;
      __IO uint32_t tfnf:1;
      __IO uint32_t activity:1;
    }IC_STATUS_fld;
    __IO uint32_t IC_STATUS;
  };

  union{ //offset addr 0x0074
    struct{
      __IO uint32_t :28;
      __IO uint32_t txflr:4;
    }IC_TXFLR_fld;
    __IO uint32_t IC_TXFLR;
  };

  union{ //offset addr 0x0078
    struct{
      __IO uint32_t :28;
      __IO uint32_t rxflr_:4;
    }IC_RXFLR_fld;
    __IO uint32_t IC_RXFLR;
  };

  union{ //offset addr 0x007c
    struct{
      __IO uint32_t :16;
      __IO uint32_t ic_sda_hold:16;
    }IC_SDA_HOLD_fld;
    __IO uint32_t IC_SDA_HOLD;
  };

  union{ //offset addr 0x0080
    struct{
      __IO uint32_t tx_flush_cnt:8;
      __IO uint32_t :7;
      __IO uint32_t abrt_user_abrt:1;
      __IO uint32_t abrt_slvrd_intx:1;
      __IO uint32_t abrt_slv_arblost:1;
      __IO uint32_t abrt_slvflush_txfifo:1;
      __IO uint32_t arb_lost:1;
      __IO uint32_t abrt_master_dis:1;
      __IO uint32_t abrt_10b_rd_norstrt:1;
      __IO uint32_t abrt_sbyte_norstrt:1;
      __IO uint32_t abrt_hs_norstrt:1;
      __IO uint32_t abrt_sbyte_ackdet:1;
      __IO uint32_t abrt_hs_ackdet:1;
      __IO uint32_t abrt_gcall_read:1;
      __IO uint32_t abrt_gcall_noack:1;
      __IO uint32_t abrt_txdata_noack:1;
      __IO uint32_t abrt_10addr2_noack:1;
      __IO uint32_t abrt_10addr1_noack:1;
      __IO uint32_t abrt_7b_addr_noack_:1;
    }IC_TX_ABRT_SOURCE_fld;
    __IO uint32_t IC_TX_ABRT_SOURCE;
  };

  union{ //offset addr 0x0084
    struct{
      __IO uint32_t :31;
      __IO uint32_t nack:1;
    }IC_SLV_DATA_NACK_ONLY_fld;
    __IO uint32_t IC_SLV_DATA_NACK_ONLY;
  };

  union{ //offset addr 0x0088
    struct{
      __IO uint32_t :30;
      __IO uint32_t tdmae:1;
      __IO uint32_t rdmae:1;
    }IC_DMA_CR_fld;
    __IO uint32_t IC_DMA_CR;
  };

  union{ //offset addr 0x008c
    struct{
      __IO uint32_t :29;
      __IO uint32_t dmatdl:3;
    }IC_DMA_TDLR_fld;
    __IO uint32_t IC_DMA_TDLR;
  };

  union{ //offset addr 0x0090
    struct{
      __IO uint32_t :29;
      __IO uint32_t dmardl:3;
    }IC_DMA_RDLR_fld;
    __IO uint32_t IC_DMA_RDLR;
  };

  union{ //offset addr 0x0094
    struct{
      __IO uint32_t :24;
      __IO uint32_t sda_setup:8;
    }IC_SDA_SETUP_fld;
    __IO uint32_t IC_SDA_SETUP;
  };

  union{ //offset addr 0x0098
    struct{
      __IO uint32_t :31;
      __IO uint32_t ack_gen_call:1;
    }IC_ACK_GENERAL_CALL_fld;
    __IO uint32_t IC_ACK_GENERAL_CALL;
  };

  union{ //offset addr 0x009c
    struct{
      __IO uint32_t :29;
      __IO uint32_t slv_rx_data_lost:1;
      __IO uint32_t slv_disabled_while_busy:1;
      __IO uint32_t ic_en:1;
    }IC_ENABLE_STATUS_fld;
    __IO uint32_t IC_ENABLE_STATUS;
  };

  union{ //offset addr 0x00a0
    struct{
      __IO uint32_t :24;
      __IO uint32_t ic_fs_spklen:8;
    }IC_FS_SPKLEN_fld;
    __IO uint32_t IC_FS_SPKLEN;
  };

  union{ //offset addr 0x00a4
    struct{
      __IO uint32_t :24;
      __IO uint32_t ic_hs_spklen:8;
    }IC_HS_SPKLEN_fld;
    __IO uint32_t IC_HS_SPKLEN;
  };


  union{ //offset addr 0x00f4
    struct{
      __IO uint32_t :8;
      __IO uint32_t tx_buffer_depth:8;
      __IO uint32_t rx_buffer_depth:8;
      __IO uint32_t add_encoded_params:1;
      __IO uint32_t has_dma:1;
      __IO uint32_t intr_io:1;
      __IO uint32_t hc_count_values:1;
      __IO uint32_t max_speed_mode:2;
      __IO uint32_t apb_data_width:2;
    }IC_COMP_PARAM_1_fld;
    __IO uint32_t IC_COMP_PARAM_1;
  };

  union{ //offset addr 0x00f8
    struct{
      __IO uint32_t ic_comp_version:32;
    }IC_COMP_VERSION_fld;
    __IO uint32_t IC_COMP_VERSION;
  };

  union{ //offset addr 0x00fc
    struct{
      __IO uint32_t ic_comp_type:32;
    }IC_COMP_TYPE_fld;
    __IO uint32_t IC_COMP_TYPE;
  };

} PICO_REG_I2C0_TypeDef;

#define PICO_REG_I2C0 PICO_REG_I2C0_TypeDef *0x40005000


#if defined(__CC_ARM)
#pragma pop
#endif


#endif // _PICO_REG_I2C0_H_

