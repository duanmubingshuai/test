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

#ifndef __BSP_BUTTON_H__
#define __BSP_BUTTON_H__

#include "types.h"
// !使能按键应用驱动长按逻辑
#define BSP_BTN_LONG_PRESS_ENABLE

#define BTN_SYS_TICK                            4  //unit:ms
#define BTN_FILTER_TICK_COUNT                   5   //(BTN_SYS_TICK*BTN_FILTER_TICK_COUNT)            ms
#define BTN_LONG_PRESS_START_TICK_COUNT         400 //(BTN_SYS_TICK*BTN_LONG_PRESS_START_TICK_COUNT)  ms
#define BTN_LONG_PRESS_KEEP_TICK_COUNT          100 //(BTN_SYS_TICK*BTN_LONG_PRESS_KEEP_TICK_COUNT)   ms

#define BTN_NUMBER                              32  //valid:[0,0x2F].reserved[0x30,0x3F]
#define BTN_NONE                                0xFF

#if (BTN_NUMBER > 48)
    #error "error bsp button config,please check"
#endif

/*
    bit0:press down
    bit1:press up
    bit2:long press start
    bit3:long press keep
    bit4:combine or not
*/
#define BSP_BTN_PD_CFG                         0x01
#define BSP_BTN_UP_CFG                         0x02
#define BSP_BTN_LPS_CFG                        0x04
#define BSP_BTN_LPK_CFG                        0x08
#define BSP_BTN_CM_CFG                         0x10

#define BSP_BTN_PD_TYPE                        (0U<<6)
#define BSP_BTN_UP_TYPE                        (1U<<6)
#define BSP_BTN_LPS_TYPE                       (2U<<6)
#define BSP_BTN_LPK_TYPE                       (3U<<6)

#define BSP_BTN_PD_BASE                       BSP_BTN_PD_TYPE
#define BSP_BTN_UP_BASE                       BSP_BTN_UP_TYPE
#define BSP_BTN_LPS_BASE                      BSP_BTN_LPS_TYPE
#define BSP_BTN_LPK_BASE                      BSP_BTN_LPK_TYPE

#define BSP_BTN_TYPE(key)                    (key & 0xC0)
#define BSP_BTN_INDEX(key)                   (key & 0x3F)

typedef uint32_t BTN_COMBINE_T;

typedef struct
{
    uint8_t KeyConfig;
    uint8_t State;
    uint8_t Count;
    uint8_t FilterTime;

    #ifdef BSP_BTN_LONG_PRESS_ENABLE
    uint16_t LongCount;
    uint16_t LongTime;
    uint16_t RepeatSpeed;
    uint16_t RepeatCount;
    #endif
} BTN_T;


#define KEY_FIFO_SIZE   20
typedef struct
{
    uint8_t Buf[KEY_FIFO_SIZE];
    uint8_t Read;
    uint8_t Write;
} KEY_FIFO_T;

bool bsp_InitBtn(BTN_T* sum_btn_array,uint8_t sum_btn_num,uint8_t combine_btn_start,BTN_COMBINE_T* combine_btn_array);

uint8_t bsp_KeyPro(void);
uint8_t bsp_GetKey(void);
bool bsp_set_key_value_by_row_col(uint8_t cols_num,uint8_t row,uint8_t col,bool value);
void bsp_set_key_value_by_index(uint8_t index,bool value);
bool bsp_KeyEmpty(void);

#endif
