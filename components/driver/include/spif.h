/*******************************************************************************
* @file		flash.h
* @brief	Contains all functions support for flash driver
* @version	0.0
* @date		27. Nov. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __FLASH_ROM_H__
#define __FLASH_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "types.h"
#include "gpio.h"

//define flash ucds
#define FLASH_UCDS_ADDR_BASE   0x11005000

#define SPIF_BASE_ADDR          (0x11000000)

#define SPIF_USER_SPACE         (0x2000)

/*boot sector*/
#define SPIF_BOOT_AREA_ADDR             (SPIF_USER_SPACE)
//boot secure mic include all image info
#define SPIF_BOOT_SEC_ADDR              ((uint8_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x10))
#define SPIF_BOOT_BIN_NUM               (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR))
#define SPIF_BOOT_MIC                   (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR + 4))
#define SPIF_BOOT_RUN_ADDR              (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR + 8))
#define SPIF_BOOT_PART_FADDR(n)         (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x10+(n)*0x10))
#define SPIF_BOOT_PART_SIZE(n)          ((*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x14+(n)*0x10))&0xffffff)
#define SPIF_BOOT_PART_SEC(n)           ((*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x14+(n)*0x10))&0x80000000)
#define SPIF_BOOT_PART_AUTH(n)          ((*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x14+(n)*0x10))&0x40000000)
#define SPIF_BOOT_PART_RADDR(n)         (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x18+(n)*0x10))
#define SPIF_BOOT_PART_SEC_MIC(n)       (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x1c+(n)*0x10))
#define SPIF_BOOT_PART_CRC(n)           (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_BOOT_AREA_ADDR+0x1c+(n)*0x10))

/*secure sector*/
#define SPIF_SECURE_AREA_ADDR           (0x2800)
#define SPIF_SECURE_AUTH_WORD           (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR)) //FCT mode
//#define SPIF_SECURE_USE_EFUSE           (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0x4))//FFFF_FFFF or 0: only flash key; 1: use efuse key
//efuse data replace by flash key
//#define SPIF_SECURE_KEY_0               (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0x10))
//#define SPIF_SECURE_KEY_1               (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0x14))
#define SPIF_SECURE_KEY_L               (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0x8))
#define SPIF_SECURE_KEY_H               (*(volatile uint32_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0xc))
//#define pSPIF_SECURE_PLAINTEXT          ((volatile uint8_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0x20))
#define pSPIF_SECURE_MIC                ((volatile uint8_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0x10))
#define pSPIF_SECURE_IV                 ((volatile uint8_t*)(SPIF_BASE_ADDR + SPIF_SECURE_AREA_ADDR + 0x20))//128 bit, secure IV ffff_ffff --> 0000_0000

/*app*/
uint32_t spif_flash_size(void);
void spif_set_flash_size(uint32_t fsize);
int  spif_read_id(uint32_t* pid);
int flash_erase_all(void);
int spif_program_write(uint32_t addr, uint8_t* data, uint32_t size);

/*driver*/
void init_spif(void);
void close_spif(void); 

void enable_spif_flash_write(void);
void disable_spif_flash_write(void);

int check_internal_bus_busy(void);
int read_status_register(void);
void write_status_register(int value);

void  read_deviceID(void);

int flash_sector_erase(unsigned int addr);
void flash_block32_erase(unsigned int addr);
int flash_block64_erase(unsigned int addr);
int flash_chip_erase(void);

void set_flash_deep_sleep(void);
void release_flash_deep_sleep(void);

//int ProgramWordShort(unsigned long offset, const unsigned char *buf,uint8_t size) ;
int ProgramPage4 (unsigned long offset, uint32 data);
int ProgramPage64 (unsigned long offset, const unsigned char *buf,  int size);
int ProgramWord (unsigned long offset, const unsigned char *buf,uint8_t size);

int WriteFlash(unsigned int offset, uint32_t  value);
int WriteFlashShort(unsigned int offset, uint16_t  value);

uint8_t ReadFlash (unsigned int addr);
uint32_t ReadFlashWord (unsigned int addr);

#ifdef __cplusplus
}
#endif

#endif
