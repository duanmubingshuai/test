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
#define   TIMER_ISR_ENTRY_TIME             13

#define   CLOCK_SETTING                    14               // HCLK
#define   LL_HW_BB_DELAY                   15
#define   LL_HW_AFE_DELAY                  16
#define   LL_HW_PLL_DELAY                  17

#define   LL_HW_RTLP_LOOP_TIMEOUT          18
#define   LL_HW_RTLP_1ST_TIMEOUT           19

#define   MIN_TIME_TO_STABLE_32KHZ_XOSC    20

#define   LL_TX_PKTS_PER_CONN_EVT          21
#define   LL_RX_PKTS_PER_CONN_EVT          22


//  ============= A1 ROM metal change add
#define   DIR_ADV_DELAY                    23


#define   LL_HW_Tx_TO_RX_INTV              24
#define   LL_HW_Rx_TO_TX_INTV              25

#define   INITIAL_STACK_PTR                26
#define   ALLOW_TO_SLEEP_TICK_RC32K        27

#define   LL_HW_BB_DELAY_ADV               28
#define   LL_HW_AFE_DELAY_ADV              29
#define   LL_HW_PLL_DELAY_ADV              30

// For scan & master, add 2018-6-15
#define   LL_ADV_TO_SCAN_REQ_DELAY         31

//for PHY updated add 2018-11-07
#define   LL_HW_BB_DELAY_2MPHY             32
#define   LL_HW_AFE_DELAY_2MPHY            33
#define   LL_HW_PLL_DELAY_2MPHY            34

#define   LL_HW_Tx_TO_RX_INTV_2MPHY        35
#define   LL_HW_Rx_TO_TX_INTV_2MPHY        36

#define   LL_HW_RTLP_TO_GAP                37

#define   LL_TRX_NUM_ADAPTIVE_CONFIG       38
#define   OSAL_SYS_TICK_WAKEUP_TRIM        39

// ==== A2 add, for secondary adv/scan
#define   LL_NOCONN_ADV_EST_TIME           40
#define   LL_NOCONN_ADV_MARGIN             41
#define   LL_SEC_SCAN_MARGIN               42
#define   LL_MIN_SCAN_TIME                 43
#define   MAC_ADDRESS_LOC                  44
#define	  LL_FIRST_WINDOW				   45
#define	  NEXT_TIMER1_CONSIDER_DELAY	   46
#define	  LL_WRITE_RE_TX_FIFO			   47



// ==============

#define   RC32_TRACKINK_ALLOW              0x00000001       // enable tracking RC 32KHz clock with 16MHz hclk
#define   SLAVE_LATENCY_ALLOW              0x00000002       // slave latency allow switch
#define   LL_DEBUG_ALLOW                   0x00000004       // enable invoke RAM project debug output fucntion
#define   LL_WHITELIST_ALLOW               0x00000008       // enable whitelist filter
#define   LL_RC32K_SEL                     0x00000010       // select RC32K RTC, otherwise select crystal 32K RTC
#define   SIMUL_CONN_ADV_ALLOW             0x00000020       // allow send adv in connect state
#define   SIMUL_CONN_SCAN_ALLOW            0x00000040       // allow scan in connect state

#define   CONN_CSA2_ALLOW                  0x00000080       // allow using CSA2 in connection state

#define   GAP_DUP_RPT_FILTER_DISALLOW      0x00000100       // duplicate report filter in GAP layer, allow default

#define   ENH_CONN_CMP_EVENT_ALLOW         0x00000200       // allow LL to send enhanced connection complete event. We may 
                                                            // disable it if host not support this message, althought NOT 
                                                            // align to the spec

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
extern uint32 *pGlobal_config;           // note: app project needn't this variable

#endif // _GLOBAL_CONFIG_H_
