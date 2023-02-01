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
/**************************************************************


    Module Name: hal_keyboard_matrix
    File name:   hal_keyboard_matrix.h
    Brief description:
      key driver module
    Author:  Teddy.Deng
    Data:    2018-12-20
    Revision:V0.01
****************************************************************/

#ifndef _HAL_KEYBOARD_MATRIX_H_
#define _HAL_KEYBOARD_MATRIX_H_
#include "types.h"
//#include "kscan.h"
#include "bsp_button.h"
#include "log.h"
// !sleep debug 
#define KEY_DEBUG_LOG
#ifdef KEY_DEBUG_LOG
#define KEY_LOG(...)	LOG(__VA_ARGS__)
#else
#define KEY_LOG(...)
#endif

/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/
#define HAL_KEY_SOFT_SCAN_EVT  	0X0001


extern uint8 matrix_key_detected_flag;
extern uint8 ir_sending_over_flag;
extern uint16 ir_sending_timeout_count;
//extern KSCAN_ROWS_e rows[NUM_KEY_ROWS];
//extern KSCAN_COLS_e cols[NUM_KEY_COLS];

extern uint8 halKeyboardMatrix_TaskID;   // Task ID for internal task/event processing
extern void hal_keyboard_matrix_task_init( uint8 task_id );

extern uint16 hal_keyboard_matrix_task_ProcessEvent( uint8 task_id, uint16 events );
void kscan_soft_read_pin_init(void);
uint8 kscan_soft_polling_read(void);
void kscan_soft_read(void);
bool get_key_press_status( void );
#endif


