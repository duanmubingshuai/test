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

#ifndef __KEYBOARD_TASK_H__
#define __KEYBOARD_TASK_H__

#include "bcomdef.h"
#include "bsp_button_task.h"

//#define KEYBOARD_DEBUG
#define __ATTR_SECTION_SRAM__   
#define __ATTR_SECTION_XIP__  

#ifdef KEYBOARD_DEBUG
#define KEYBOARD_LOG(...)    LOG(__VA_ARGS__) 
#else
#define KEYBOARD_LOG(...)
#endif

typedef void (*keyboard_event_t)( uint8 col_nums, uint8 row, uint8 col, uint8 key_state );

typedef struct keyboard_cfg_t
{
    keyboard_event_t keyboard_cb;
	uint8  keyboard_taskId;
	uint16 keyboard_event;
}keyboard_cfg;

#define GPIO_ROW_COL_0                    0 // ! soft row col scan
#define GPIO_ROW_COL_1                    1 // ! soft row col scan & row to extend gnd

#define KEYBOARD_HARDWARE_MODE            GPIO_ROW_COL_0

#define KEYBOARD_POLLING_INTERVAL         10
/*only gpio for col */
#define MATRIX_KEYBOARD_COL               4
/*only gpio for row */
#define MATRIX_KEYBOARD_ROW               4
/*pwrmgr mod define*/
#define KEYBOARD_PWRMGR_MOD               MOD_USR4
                  

#if( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_1 )
#define KEYBOARD_EXTEND_KEY_NUM           4
#define KEYBOARD_BUFF_LEN                 MATRIX_KEYBOARD_ROW*MATRIX_KEYBOARD_COL+KEYBOARD_EXTEND_KEY_NUM
#elif( KEYBOARD_HARDWARE_MODE == GPIO_ROW_COL_0 )
#define KEYBOARD_EXTEND_KEY_NUM           0
#define KEYBOARD_BUFF_LEN                 MATRIX_KEYBOARD_ROW*MATRIX_KEYBOARD_COL+KEYBOARD_EXTEND_KEY_NUM
#endif

#define ROW_GPIO_00                       KSCAN_ROW_0_GPIO
#define ROW_GPIO_01                       KSCAN_ROW_1_GPIO
#define ROW_GPIO_02                       KSCAN_ROW_2_GPIO           
#define ROW_GPIO_03                       KSCAN_ROW_3_GPIO           

#define COL_GPIO_00                       KSCAN_COL_0_GPIO
#define COL_GPIO_01                       KSCAN_COL_1_GPIO
#define COL_GPIO_02                       KSCAN_COL_2_GPIO
#define COL_GPIO_03                       KSCAN_COL_3_GPIO


void keyboard_register( keyboard_event_t cb, uint8 task_Id, uint16 event );

void keyboard_io_read( void );

void keyboard_deep_sleep_handler( void );

#endif
