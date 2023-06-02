/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/
/*******************************************************************************
    # File: lib_efuse.c
    # Hist:
     2022.7.7  YU File Creation
     2022.8.11 YU Reduce memory usage
 *******************************************************************************/
#include <string.h>
#include "lib_efuse3.h"
#include "mcu.h"
struct Efuse_inf
{
    unsigned int mft:2; //厂家
    unsigned int zigbee_enab:1;
    unsigned int prog_ver:4; //测试程序版本编号
    unsigned int chip_ver:4; //芯片型号
    unsigned int ble_enab:1;
    unsigned int lotnum:10;
    unsigned int site:4;
    unsigned int mesh_enab:1;
    unsigned int multirole_enab:1;
    unsigned int pass_flg:2;
    unsigned int time_stamp:31;//yyyymmddhhmmss
};

typedef enum
{
    EFUSE_BLOCK_0 = 0,
    EFUSE_BLOCK_1 = 1,
    EFUSE_BLOCK_2 = 2,
    EFUSE_BLOCK_3 = 3,

} EFUSE_block_t;

static struct Efuse_inf efuse_inf;

unsigned int lib_efuse_mft(void)
{
    return efuse_inf.mft;
}

unsigned int lib_efuse_zigbee(void)
{
    return efuse_inf.zigbee_enab;
}

unsigned int lib_efuse_prog_ver(void)
{
    return efuse_inf.prog_ver;
}

unsigned int lib_efuse_chip_ver(void)
{
    return efuse_inf.chip_ver;
}

unsigned int lib_efuse_ble(void)
{
    return efuse_inf.ble_enab;
}

unsigned int lib_efuse_lotnum(void)
{
    return efuse_inf.lotnum;
}

unsigned int lib_efuse_site(void)
{
    return efuse_inf.site;
}

unsigned int lib_efuse_time_stamp()
{
    return efuse_inf.time_stamp;
}

unsigned int lib_efuse_mesh(void)
{
    return efuse_inf.mesh_enab;
}

unsigned int lib_efuse_multirole(void)
{
    return efuse_inf.multirole_enab;
}

unsigned int lib_efuse_pass_flg(void)
{
    return efuse_inf.pass_flg;
}

void lib_read_hw_version(uint32_t* buff)
{
    subWriteReg(0x4000f054,19,19,0x01);
    extern int efuse_read(EFUSE_block_t block,uint32_t* buf);
    efuse_read(EFUSE_BLOCK_3,buff);
    lib_efuse_load((uint32_t*)buff);
}

void lib_efuse_load(uint32_t* efuse_data)
{
    uint32_t data[2];
    memcpy(data,efuse_data,sizeof(data));
    //mft
    efuse_inf.mft = (data[0] >> 1) & 0x03;
    //zigbee_enab
    efuse_inf.zigbee_enab = ((data[0] >> 3) & 0x01) == 0 ? true : false;
    //prog_ver
    efuse_inf.prog_ver = (data[0] >> 4) & 0x0f;
    //chip_ver
    efuse_inf.chip_ver = (data[0] >> 8) & 0x0f;
    //ble_enab
    efuse_inf.ble_enab = ((data[0] >> 12) & 0x01) == 0 ? true : false;
    //lotnum
    efuse_inf.lotnum = (data[0] >> 13) & 0x03ff;
    //site
    efuse_inf.site = (data[0] >> 23) & 0x0f;
    //time stamp
    efuse_inf.time_stamp = (data[0] >> 27) & 0x1f;
    efuse_inf.time_stamp |= ((data[1]) & 0x3ffffff) << 5;
    //mesh
    efuse_inf.mesh_enab = ((data[1] >> 26) & 0x01) == 0 ? true : false;
    //multirole
    efuse_inf.multirole_enab = ((data[1] >> 27) & 0x01) == 0 ? true : false;
    //pass_flg
    efuse_inf.pass_flg = (data[1] >> 28) & 0x03;
    return;
}
