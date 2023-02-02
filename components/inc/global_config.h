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

/**
 ****************************************************************************************
 *
 * @file global_config.h
 *
 * @brief This file contains the definitions of index of global configuration which 
 *         will be configured in APP project.
 *
 *
 * $Rev:  $
 *
 ****************************************************************************************
 */


#ifndef _GLOBAL_CONFIG_H_
#define _GLOBAL_CONFIG_H_

#include "types.h"

/*******************************************************************************
 *  software configuration parameters definition
 */
 
#define CONFIG_BASE_ADDR 0x1fff0100 

#define   SOFT_PARAMETER_NUM               64

// parameter index of configuration array
#define   ADV_CHANNEL_INTERVAL             0                // interval between adv channel in the same adv event
#define   SCAN_RSP_DELAY                   1                // to adjust scan req -> scan rsp delay
#define   CONN_REQ_TO_SLAVE_DELAY          2                // to calibrate the delay between conn req & 1st slave conn event
#define   SLAVE_CONN_DELAY                 3                // to adjust the delay between 2 slave connection events
#define   SLAVE_CONN_DELAY_BEFORE_SYNC     4                // to adjust the delay between 2 slave connection events before 1st anchor is acquired
#define   MAX_SLEEP_TIME                   5                // maximum sleep time in us
#define   MIN_SLEEP_TIME                   6                // minimum sleep time in us
#define   WAKEUP_ADVANCE                   7                // wakeup advance time, to cover HW delay, crystal settle time, sw delay, ... etc.
#define   WAKEUP_DELAY                     8                // cycles of SW delay to wait crystal settle

#define   HDC_DIRECT_ADV_INTERVAL          9
#define   LDC_DIRECT_ADV_INTERVAL          10


#define   LL_SWITCH                        11               // Link Layer switch, 1 enable, 0 disable
#define   NON_ADV_CHANNEL_INTERVAL         12               // interval between non-adv channel in the same adv event

#define   DIR_ADV_DELAY                    13

#define   CLOCK_SETTING                    14               // HCLK
#define   LL_HW_BB_DELAY                   15
#define   LL_HW_AFE_DELAY                  16
#define   LL_HW_PLL_DELAY                  17

#define   LL_HW_RTLP_LOOP_TIMEOUT          18
#define   LL_HW_RTLP_1ST_TIMEOUT           19

#define   MIN_TIME_TO_STABLE_32KHZ_XOSC    20

//#define   LL_TX_PKTS_PER_CONN_EVT          21
//#define   LL_RX_PKTS_PER_CONN_EVT          22


//  ============= A1 ROM metal change add



//#define   LL_TX_PWR_TO_REG_BIAS            24


//#define   LL_SMART_WINDOW_COEF_ALPHA       25
//#define   LL_SMART_WINDOW_TARGET           26
//#define   LL_SMART_WINDOW_INCREMENT        27
//#define   LL_SMART_WINDOW_LIMIT            28
//#define   LL_SMART_WINDOW_ACTIVE_THD       29
//#define   LL_SMART_WINDOW_ACTIVE_RANGE     30

//#define   LL_SMART_WINDOW_FIRST_WINDOW     31
//#define   LL_FIRST_WINDOW	LL_SMART_WINDOW_FIRST_WINDOW 

#define   LL_HW_Tx_TO_RX_INTV              21
#define   LL_HW_Rx_TO_TX_INTV              22

#define   INITIAL_STACK_PTR                23
#define   ALLOW_TO_SLEEP_TICK_RC32K        24

#define   LL_HW_BB_DELAY_ADV               25
#define   LL_HW_AFE_DELAY_ADV              26
#define   LL_HW_PLL_DELAY_ADV              27

// For scan & master, add 2018-6-15
#define   LL_ADV_TO_SCAN_REQ_DELAY         28
#define   LL_ADV_TO_CONN_REQ_DELAY         29

#define   LL_MOVE_TO_MASTER_DELAY          30

//#define   LL_HW_TRLP_LOOP_TIMEOUT          42

#define   LL_CONN_REQ_WIN_SIZE             31
#define   LL_CONN_REQ_WIN_OFFSET           32

#define   LL_MASTER_PROCESS_TARGET         33
//#define   LL_MASTER_TIRQ_DELAY             46

//for PHY updated add 2018-11-07
#define   LL_HW_BB_DELAY_2MPHY             34
#define   LL_HW_AFE_DELAY_2MPHY            35
#define   LL_HW_PLL_DELAY_2MPHY            36

#define   LL_HW_Tx_TO_RX_INTV_2MPHY        37
#define   LL_HW_Rx_TO_TX_INTV_2MPHY        38


#define   OSAL_SYS_TICK_WAKEUP_TRIM        39

#define   LL_SEC_SCAN_MARGIN               40
#define   LL_MIN_SCAN_TIME                 41
// Bumblebee ROM code
//#define   LL_CONN_ADV_EST_TIME             74
//#define   LL_SCANABLE_ADV_EST_TIME         75


#define   MAC_ADDRESS_LOC                  42



// ==== For Extended Adv & Periodic adv
//#define   LL_EXT_ADV_INTER_PRI_CHN_INT     81
//#define   LL_EXT_ADV_INTER_AUX_CHN_INT     82
//#define   LL_EXT_ADV_RSC_POOL_PERIOD       83
//#define   LL_EXT_ADV_RSC_POOL_UNIT         84
//
//#define   LL_EXT_ADV_TASK_DURATION         86
//#define   LL_PRD_ADV_TASK_DURATION         87
#define   LL_CONN_TASK_DURATION            43
//
#define   TIMER_SET_ENTRY_TIME             44       // time from HW timer expiry to ISR entry, unit: us
#define   LL_MULTICONN_MASTER_PREEMP       45
#define   LL_MULTICONN_MASTER_SLOT		   46
//#define   LL_MULTICONN_SLAVE_PREEMP        92
//
//#define   LL_EXT_ADV_INTER_SEC_CHN_INT     93
//#define   LL_EXT_ADV_PRI_2_SEC_CHN_INT     94
//
//#define   LL_EXT_ADV_RSC_PERIOD            95
//#define   LL_EXT_ADV_RSC_SLOT_DURATION     96            
//
//#define   LL_PRD_ADV_RSC_PERIOD            97
//#define   LL_PRD_ADV_RSC_SLOT_DURATION     98   
//
//#define   LL_EXT_ADV_PROCESS_TARGET        99
//#define   LL_PRD_ADV_PROCESS_TARGET        100


#define   CLK_TRACKING_CONFIG_DLL		   47
#define   CLK_TRACKING_CONFIG_XTAL		   48
#define   RC32_CNT_TRACKING_AVG_CONFIG     49


#define   UARTRUN_2S_ENTRY_CONFIG         50    //for 2 stage uartrun entry setting




// ==============

#define   RC32_TRACKINK_ALLOW              0x00000001       // enable tracking RC 32KHz clock with 16MHz hclk
#define   SLAVE_LATENCY_ALLOW              0x00000002       // slave latency allow switch
//#define   LL_DEBUG_ALLOW                   0x00000004       // enable invoke RAM project debug output fucntion
//#define   LL_WHITELIST_ALLOW               0x00000008       // enable whitelist filter
#define   LL_RC32K_SEL                     0x00000010       // select RC32K RTC, otherwise select crystal 32K RTC
//#define   SIMUL_CONN_ADV_ALLOW             0x00000020       // allow send adv in connect state
#define   SIMUL_CONN_SCAN_ALLOW            0x00000040       // allow scan in connect state

//#define   CONN_CSA2_ALLOW                  0x00000080       // allow using CSA2 in connection state

//#define   GAP_DUP_RPT_FILTER_DISALLOW      0x00000100       // duplicate report filter in GAP layer, allow default

//#define   ENH_CONN_CMP_EVENT_ALLOW         0x00000200       // allow LL to send enhanced connection complete event. We may 
                                                            // disable it if host not support this message, althought NOT 
                                                            // align to the spec
#define   RC32_CAP_ADAPTIVE_TRIM             0x00000400  
// delete 2018-7-17, should use enum  H_SYSCLK_SEL
//enum
//{
//    CLOCK_16MHZ = 0,
//    CLOCK_32MHZ = 1,
//    CLOCK_48MHZ = 2,
//    CLOCK_64MHZ = 3,
//    CLOCK_96MHZ = 4,
//    CLOCK_32MHZ_DBL=5
//};

//extern uint32 global_config[SOFT_PARAMETER_NUM];
//extern uint32 * pGlobal_config;           // note: app project needn't this variable
#define pGlobal_config ((uint32_t *)(CONFIG_BASE_ADDR))

#endif // _GLOBAL_CONFIG_H_
