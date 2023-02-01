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

/**************************************************************************************************
    Filename:       bsp_button_task.h
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/
#ifndef __BSP_BUTTON_TASK_H__
#define __BSP_BUTTON_TASK_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "bus_dev.h"
#include "bsp_gpio.h"
#include "keyboard.h"
#include "bsp_button.h"
#include "log.h"


/*********************************************************************
    INCLUDES
*/

/*********************************************************************
    CONSTANTS
*/
#define BSP_BTN_JUST_GPIO                                   (0x01)
#define BSP_BTN_JUST_KSCAN                                  (0x02)
#define BSP_BTN_GPIO_AND_KSCAN                              (0x03)
#define BSP_BTN_SOFT_SCAN                                   (0X04)
#define BSP_BTN_HARDWARE_CONFIG                             BSP_BTN_SOFT_SCAN

#if (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_GPIO)

#define BSP_SINGLE_BTN_NUM                                  (GPIO_SINGLE_BTN_NUM)
#define BSP_COMBINE_BTN_NUM                                 (2)
#define BSP_TOTAL_BTN_NUM                                   (BSP_SINGLE_BTN_NUM + BSP_COMBINE_BTN_NUM)

#elif  (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_JUST_KSCAN)

#define BSP_SINGLE_BTN_NUM                                  (NUM_KEY_ROWS * NUM_KEY_COLS)
#define BSP_COMBINE_BTN_NUM                                 (3)
#define BSP_TOTAL_BTN_NUM                                   (BSP_SINGLE_BTN_NUM + BSP_COMBINE_BTN_NUM)
/* key id for enable keyid */
#define BSP_COMBINE_KEY_ID_0                                BSP_SINGLE_BTN_NUM+0
/* key id for enable singlecarrier */
#define BSP_COMBINE_KEY_ID_1                                BSP_SINGLE_BTN_NUM+1
/* key id for enable product mode */
#define BSP_COMBINE_KEY_ID_2                                BSP_SINGLE_BTN_NUM+2

/* key id for enable ir learn */
#define BSP_IR_LEARN_KEY_ID                                 13

#include "kscan.h"

#elif  (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_GPIO_AND_KSCAN)

#define BSP_KSCAN_SINGLE_BTN_NUM                            (NUM_KEY_ROWS * NUM_KEY_COLS)
#define BSP_GPIO_SINGLE_BTN_NUM                             (GPIO_SINGLE_BTN_NUM)

#define BSP_KSCAN_COMBINE_BTN_NUM                           (1)
#define BSP_GPIO_COMBINE_BTN_NUM                            (1)

#define BSP_SINGLE_BTN_NUM                                  (BSP_KSCAN_SINGLE_BTN_NUM + BSP_GPIO_SINGLE_BTN_NUM)
#define BSP_COMBINE_BTN_NUM                                 (BSP_KSCAN_COMBINE_BTN_NUM + BSP_GPIO_COMBINE_BTN_NUM)
#define BSP_TOTAL_BTN_NUM                                   (BSP_SINGLE_BTN_NUM + BSP_COMBINE_BTN_NUM)
#elif  (BSP_BTN_HARDWARE_CONFIG == BSP_BTN_SOFT_SCAN)
#define BSP_SINGLE_BTN_NUM                                  KEYBOARD_BUFF_LEN
#define BSP_COMBINE_BTN_NUM                                 (2)
#define BSP_TOTAL_BTN_NUM                                   (BSP_SINGLE_BTN_NUM + BSP_COMBINE_BTN_NUM)
/* key id for enable keyid */
#define BSP_COMBINE_KEY_ID_0                                BSP_SINGLE_BTN_NUM+0
/* key id for enable singlecarrier */
#define BSP_COMBINE_KEY_ID_1                                BSP_SINGLE_BTN_NUM+1
/* key id for enable ir learn */
#define BSP_IR_LEARN_KEY_ID                                 13

//#include "kscan.h"

#else

#error "error bsp button config,please check"

#endif


#if (BSP_COMBINE_BTN_NUM > 0)
extern uint32_t usr_combine_btn_array[BSP_COMBINE_BTN_NUM];
#endif

extern bool bsp_btn_gpio_flag;
extern bool bsp_btn_kscan_flag;

typedef void (*bsp_btn_callback_t)(uint8_t evt);
extern bsp_btn_callback_t bsp_btn_cb;

#define BSP_BTN_EVT_SYSTICK                                 0x0010
#define KSCAN_WAKEUP_TIMEOUT_EVT                            0x0020
#define BSP_BTN_EVT_DBG                                     0x0040
#define BSP_BTN_KSCAN_INIT_CONFIG_EVT                       0X0080
#define BSP_SOFT_POLLING_EVT                                0X0001

extern uint8 Bsp_Btn_TaskID;
uint16 Bsp_Btn_ProcessEvent( uint8 task_id, uint16 events);
void   Bsp_Btn_Init( uint8 task_id );

//extern KSCAN_ROWS_e rows[NUM_KEY_ROWS];
//extern KSCAN_COLS_e cols[NUM_KEY_COLS];

void btp_button_init(void);

void clean_latency_flag(void);

void bsp_button_init_config_handle(void);

uint8 get_latency_flag(void);


void kscan_enter_power_off_gpio_option( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BSP_BUTTON_TASK_H */
