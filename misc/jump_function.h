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
 * @file jump_fucntion.h
 *
 * @brief This file contains the definitions of the macros and functions that are
 * architecture dependent.  The implementation of those is implemented in the
 * appropriate architecture directory.
 *
 *
 * $Rev:  $
 *
 ****************************************************************************************
 */
#ifndef _JUMP_FUNC_H_
#define _JUMP_FUNC_H_
#include <stdint.h>
#include "types.h"

//#ifndef FPGA_ROM_DRIVER_TEST 
#include "ll_def.h"
#include "ll_sleep.h"
#include "hci.h"
#include "l2cap.h"
//#endif

// =====================  MACROS =======================
#define JUMP_BASE_ADDR 0x1fff0000
#define JUMP_FUNC_NUM  128
void JUMP_FUNCTION_SET(uint8 jidx,uint32 val);
#define JUMP_FUNCTION_GET(x)	((uint32_t)((*(uint16 *)(JUMP_BASE_ADDR + (x << 1))) + (SRAM_BASE_ADDR&0xffff0000)))

//
//#define JUMP_FUNCTION(x)    (*(uint32 *)(JUMP_BASE_ADDR + (x << 2)))

// ROM function entries

// 0 - 10 for common         
#define     OSAL_INIT_TASKS                      1
#define     TASKS_ARRAY                          2
#define     TASK_COUNT                           3
#define     TASK_EVENTS                          4    
#define     OSAL_MEM_INIT                        5
#define     LL_RESET                             6
#define     LL_TXDATA                            7
#define     LL_DISCONNECT                        8
#define     LL_SET_ADV_PARAM                     9
#define     LL_SET_ADV_DATA                      10
#define     LL_SET_ADV_CONTROL                   11
#define     LL_SLAVE_EVT_ENDOK                   12
#define     LL_SETUP_NEXT_SLAVE_EVT              13
#define     LL_PROCESS_SLAVE_CTRL_PROC           14
#define     LL_PROCESS_SLAVE_CTRL_PKT            15
#define     LL_PROCESS_RX_DATA                   16
#define     LL_PROCESS_TX_DATA                   17
#define     LL_CONN_TERMINATE                    18
#define     LL_WRITE_TX_DATA                     19
#define     LL_EVT_SCHEDULE                      20
#define     LL_MOVE_TO_SLAVE_FUNCTION            21
#define     LL_SLAVE_CONN_EVENT                  22
#define     LL_SETUP_ADV                         23
#define     LL_SETUP_UNDIRECT_ADV                24
#define     LL_SETUP_NOCONN_ADV                  25
#define     LL_SETUP_DIRECT_ADV                  26
#define     LL_CALC_TIMER_DRIFT                  27
#define     LL_GENERATE_TX_BUFFER                28
#define     LL_READ_RX_FIFO                      29
#define     LL_READ_TX_FIFO_RTLP                 30
#define     LL_READ_TX_FIFO_PKT                  31
#define     LL_HW_PROCESS_RTO                    32
#define     LL_RELEASE_CONN_ID                   33
#define     OSAL_POWER_CONSERVE                  34
#define     ENTER_SLEEP_PROCESS                  35
#define     WAKEUP_PROCESS                       36
#define     CONFIG_RTC                           37
#define     ENTER_SLEEP_OFF_MODE                 38
#define     LL_HW_GO                             39
#define     L2CAP_PARSE_PACKET                   40
#define     L2CAP_ENCAP_PACKET                   41
#define     GAP_LINK_MGR_PROCESS_CONNECT_EVT     42
#define     GAP_LINK_MGR_PROCESS_DISCONNECT_EVT  43
#define     APP_SLEEP_PROCESS                    44
#define     APP_WAKEUP_PROCESS                   45
#define     RF_INIT                              46
#define     WAKEUP_INIT                          47
#define     BOOT_INIT                            48
#define     RF_CALIBRATTE                        49
#define     RF_PHY_CHANGE                        50
#define     LL_MASTER_EVT_ENDOK                  51    
#define     LL_SETUP_NEXT_MASTER_EVT             52
#define     LL_MOVE_TO_MASTER_FUNCTION           53
#define     LL_MASTER_CONN_EVENT                 54
#define     LL_SET_SCAN_CTRL                     55
#define     LL_SET_SCAN_PARAM                    56
#define     LL_CREATE_CONN                       57
#define     LL_CREATE_CONN_CANCEL                58
#define     LL_SETUP_SCAN                        59
#define     LL_SETUP_SEC_SCAN                    60
#define     LL_CALC_MAX_SCAN_TIME                61
#define     LL_ADP_ADJ_NEXT_TIME                 62
#define     LL_SET_NEXT_DATA_CHN                 63
#define     LL_PLUS_DISABLE_LATENCY              64
#define     LL_PLUS_ENABLE_LATENCY               65
#define     LL_SCHEDULER                         66
#define     LL_ADD_TASK                          67
#define     LL_DEL_TASK                          68
#define     OSAL_SET_EVENT                       69
#define     OSAL_MSG_SEND                        70
#define     HAL_DRV_IRQ_INIT                     71
#define     HAL_DRV_IRQ_ENABLE                   72
#define     HAL_DRV_IRQ_DISABLE                  73
#define     NMI_HANDLER                          74
#define     HARDFAULT_HANDLER                    75
#define 	CK802_TRAP_C	HARDFAULT_HANDLER
#define     SVC_HANDLER                          76
#define     PENDSV_HANDLER                       77
#define     SYSTICK_HANDLER                      78
#define     V0_IRQ_HANDLER                       79
#define     V1_IRQ_HANDLER                       80
#define     V2_IRQ_HANDLER                       81
#define     V3_IRQ_HANDLER                       82
#define     V4_IRQ_HANDLER                       83
#define     V5_IRQ_HANDLER                       84
#define     V6_IRQ_HANDLER                       85
#define     V7_IRQ_HANDLER                       86
#define     V8_IRQ_HANDLER                       87
#define     V9_IRQ_HANDLER                       88
#define     V10_IRQ_HANDLER                      89
#define     V11_IRQ_HANDLER                      90
#define     V12_IRQ_HANDLER                      91
#define     V13_IRQ_HANDLER                      92
#define     V14_IRQ_HANDLER                      93
#define     V15_IRQ_HANDLER                      94
#define     V16_IRQ_HANDLER                      95
#define     V17_IRQ_HANDLER                      96
#define     V18_IRQ_HANDLER                      97
#define     V19_IRQ_HANDLER                      98
#define     V20_IRQ_HANDLER                      99
#define     V21_IRQ_HANDLER                      100
#define     V22_IRQ_HANDLER                      101
#define     V23_IRQ_HANDLER                      102
#define     V24_IRQ_HANDLER                      103
#define     V25_IRQ_HANDLER                      104
#define     V26_IRQ_HANDLER                      105
#define     V27_IRQ_HANDLER                      106
#define     V28_IRQ_HANDLER                      107
#define     V29_IRQ_HANDLER                      108
#define     V30_IRQ_HANDLER                      109
#define     V31_IRQ_HANDLER                      110
#define     LL_SET_CHANNEL                       111
#define     LL_SET_WHITEN_SEED                   112
#define     CLK_APPLY_SETTING                    113

#define     HAL_WATCHDOG_INIT                    127

#define     BB_IRQ_HANDLER              V4_IRQ_HANDLER
//#define     KSCAN_IRQ_HANDLER           V5_IRQ_HANDLER
//#define     RTC_IRQ_HANDLER             V6_IRQ_HANDLER
#define     CP_COM_IRQ_HANDLER          V7_IRQ_HANDLER
#define     AP_COM_IRQ_HANDLER          V8_IRQ_HANDLER
#define     WDT_IRQ_HANDLER             V10_IRQ_HANDLER
#define     UART0_IRQ_HANDLER           V11_IRQ_HANDLER
#define     I2C0_IRQ_HANDLER            V12_IRQ_HANDLER
//#define     I2C1_IRQ_HANDLER            V13_IRQ_HANDLER
#define     SPI0_IRQ_HANDLER            V14_IRQ_HANDLER
//#define     SPI1_IRQ_HANDLER            V15_IRQ_HANDLER
#define     GPIO_IRQ_HANDLER            V16_IRQ_HANDLER
//#define     UART1_IRQ_HANDLER           V17_IRQ_HANDLER
//#define     SPIF_IRQ_HANDLER            V18_IRQ_HANDLER
//#define     DMAC_IRQ_HANDLER            V19_IRQ_HANDLER
#define     TIM1_IRQ_HANDLER            V20_IRQ_HANDLER
#define     TIM2_IRQ_HANDLER            V21_IRQ_HANDLER
#define     TIM3_IRQ_HANDLER            V22_IRQ_HANDLER
#define     TIM4_IRQ_HANDLER            V23_IRQ_HANDLER
//#define     TIM5_IRQ_HANDLER            V24_IRQ_HANDLER
//#define     TIM6_IRQ_HANDLER            V25_IRQ_HANDLER
#define     AES_IRQ_HANDLER             V28_IRQ_HANDLER
#define     ADCC_IRQ_HANDLER            V29_IRQ_HANDLER
#define     QDEC_IRQ_HANDLER            V30_IRQ_HANDLER



// ================== FUNCTIONS  ==================================
void move_to_slave_function0(void);
void LL_slave_conn_event0(void);
llStatus_t llSetupAdv0(void);
void llSetupUndirectedAdvEvt0(void);
void llSetupNonConnectableAdvEvt0( void );
//void llSetupScannableAdvEvt0( void );
void llSetupDirectedAdvEvt0( void );
void LL_evt_schedule0(void);

void llCalcTimerDrift0( uint32    connInterval,
                        uint16   slaveLatency,
                        uint8    sleepClkAccuracy,
                        uint32   *timerDrift ) ;

uint16 ll_generateTxBuffer0(int txFifo_vacancy, uint16 *pSave_ptr);

void ll_hw_read_tfifo_rtlp0(void);

void ll_read_rxfifo0(void);

int ll_hw_read_tfifo_packet0(uint8 *pkt);

void ll_hw_process_RTO0(uint32 ack_num);

void LL_set_default_conn_params0(llConnState_t *connPtr);

// =====
void enterSleepProcess0(uint32 time);

void wakeupProcess0(void);

void config_RTC0(uint32 time);

void enter_sleep_off_mode0(Sleep_Mode mode);

void llSlaveEvt_TaskEndOk0( void );

uint8 llSetupNextSlaveEvent0( void );

uint8 llCheckForLstoDuringSL0( llConnState_t *connPtr );

uint8 llProcessSlaveControlProcedures0( llConnState_t *connPtr );

void llProcessSlaveControlPacket0( llConnState_t *connPtr,
                                  uint8         *pBuf );

void llSlaveEvt_TaskAbort0(void );
void llMasterEvt_TaskEndOk0( void );
void llProcessMasterControlPacket0( llConnState_t *connPtr,
                                   uint8         *pBuf );
uint8 llProcessMasterControlProcedures0( llConnState_t *connPtr );
uint8 llSetupNextMasterEvent0( void );

void move_to_master_function0(void);
void LL_master_conn_event0(void);

llStatus_t LL_SetScanControl0( uint8 scanMode,
                              uint8 filterReports );
llStatus_t LL_SetScanParam0( uint8  scanType,
                            uint16 scanInterval,
                            uint16 scanWindow,
                            uint8  ownAddrType,
                            uint8  scanWlPolicy );

llStatus_t LL_CreateConn0( uint16 scanInterval,
                          uint16 scanWindow,
                          uint8  initWlPolicy,
                          uint8  peerAddrType,
                          uint8  *peerAddr,
                          uint8  ownAddrType,
                          uint16 connIntervalMin,
                          uint16 connIntervalMax,
                          uint16 connLatency,
                          uint16 connTimeout,
                          uint16 minLength,
                          uint16 maxLength );
llStatus_t LL_CreateConnCancel0( void );    
                            
void llSetupScan0( uint8 chan );                            

//  ================== ll.c
void LL_Init0( uint8 taskId );
uint16 LL_ProcessEvent0( uint8 task_id, uint16 events );
llStatus_t LL_Reset0( void );
llStatus_t LL_TxData0( uint16 connId, uint8 *pBuf, uint8  pktLen, uint8  fragFlag );
llStatus_t LL_Disconnect0( uint16 connId, uint8  reason );
llStatus_t LL_SetAdvParam0( uint16 advIntervalMin,
                           uint16 advIntervalMax,
                           uint8  advEvtType,
                           uint8  ownAddrType,
                           uint8  directAddrType,
                           uint8  *directAddr,
                           uint8  advChanMap,
                           uint8  advWlPolicy );
llStatus_t LL_SetAdvData0( uint8  advDataLen, uint8 *advData );
llStatus_t LL_SetAdvControl0( uint8 advMode );

llStatus_t LL_EXT_SetTxPower0( uint8 txPower, uint8 *cmdComplete );

llStatus_t LL_ReadTxPowerLevel0( uint8 connId, uint8 type, int8  *txPower );
llStatus_t LL_SetTxPowerLevel0( int8  txPower );
llStatus_t LL_ReadAdvChanTxPower0( int8 *txPower );
llStatus_t LL_ReadRssi0( uint16 connId, int8   *lastRssi );
//llStatus_t LL_ReadRemoteUsedFeatures0( uint16 connId );
llStatus_t LL_Encrypt0( uint8 *key, uint8 *plaintextData, uint8 *encryptedData );


// ================ ll_common.c
void llProcessTxData0( llConnState_t *connPtr, uint8 context );
uint8 llProcessRxData0( void );
uint8 llWriteTxData0( llConnState_t *connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8         *pBuf );
void llConnTerminate0( llConnState_t *connPtr, uint8 reason );
void llReleaseConnId0( llConnState_t *connPtr );

// ================ ll_enc.c
void LL_ENC_AES128_Encrypt( uint8 *key,
                            uint8 *plaintext,
                            uint8 *ciphertext );
//uint8 LL_ENC_GenerateTrueRandNum0( uint8 *buf,
//                                  uint8 len );
void LL_ENC_GenDeviceSKD0( uint8 *SKD );
void LL_ENC_GenDeviceIV0( uint8 *IV );
void LL_ENC_GenerateNonce0( uint32 pktCnt,
                           uint8  direction,
                           uint8  *nonce );
void LL_ENC_Encrypt0( llConnState_t *connPtr,
                     uint8          pktHdr,
                     uint8          pktLen,
                     uint8         *pBuf );
uint8 LL_ENC_Decrypt0( llConnState_t *connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8         *pBuf );

// =================== osal
void osal_pwrmgr_powerconserve0( void ) ;

// =================== ll_hw_drv.c
void ll_hw_set_timing0(uint8 pktFmt);
void ll_hw_go0(void);
void ll_hw_trigger0(void);

// ================== SMP functions
void SM_Init0( uint8 task_id );
uint16 SM_ProcessEvent0( uint8 task_id, uint16 events );

// ================== HCI_TL functions
void HCI_Init0( uint8 task_id );
uint16 HCI_ProcessEvent0( uint8 task_id, uint16 events );


// ======= OSAL memory 
void osal_mem_init0(void);

// =========== ROM -> APP function
void app_sleep_process(void);

void app_wakeup_process(void);

void rf_init(void);

void boot_init0(void);

void wakeup_init0(void);

void debug_print(uint32 state);

void rf_calibrate0(void);

void rf_phy_change_cfg(uint8 pktFmt);

// ========== A2, for conn-adv, conn-scan
uint8 llSetupSecNonConnectableAdvEvt0( void );
uint8 llSecAdvAllow0(void);
uint32 llCalcMaxScanTime0(void);
void llSetupSecScan0( uint8 chan );

uint8 llSetupSecAdvEvt0( void );
uint8 llSetupSecConnectableAdvEvt0( void );
uint8 llSetupSecScannableAdvEvt0( void );

//=============== gap_linkmgr.c
void gapProcessDisconnectCompleteEvt0( hciEvt_DisconnComplete_t *pPkt );
void gapProcessConnectionCompleteEvt0( hciEvt_BLEConnComplete_t *pPkt );


//=============== l2cap_util.c
uint8 l2capParsePacket0( l2capPacket_t *pPkt, hciDataEvent_t *pHciMsg );
uint8 l2capEncapSendData0( uint16 connHandle, l2capPacket_t *pPkt );
uint8 l2capPktToSegmentBuff0(uint16 connHandle, l2capSegmentBuff_t* pSegBuf, uint8 blen,uint8* pBuf);
uint8 l2capSegmentBuffToLinkLayer0(uint16 connHandle, l2capSegmentBuff_t* pSegBuf);
void l2capPocessFragmentTxData0(uint16 connHandle);


//=============== DLE
llStatus_t LL_SetDataLengh0( uint16 connId,uint16 TxOctets,uint16 TxTime );
void llPduLengthUpdate0(uint16 connHandle);
void llTrxNumAdaptiveConfig0(void);

//===============LL ADJ WINDOW
void ll_adptive_adj_next_time0(uint32 nextTime);
void ll_adptive_smart_window0(uint32 irq_status,uint32 anchor_point);
void llSetNextDataChan0( llConnState_t *connPtr );

//=============== PHY UPDATE
llStatus_t LL_SetPhyMode0( uint16 connId,uint8 allPhy,uint8 txPhy, uint8 rxPhy,uint16 phyOptions);
llStatus_t LL_PhyUpdate0( uint16 connId );
//void llSetNextPhyMode0( llConnState_t *connPtr );

llStatus_t LL_PLUS_DisableSlaveLatency0(uint8 connId);
llStatus_t LL_PLUS_EnableSlaveLatency0(uint8 connId);

// ================= BBB
void ll_scheduler0(uint32 time);
void ll_addTask0(uint8 connId, uint32 time);
void ll_deleteTask0(uint8 connId);

void ll_scheduler0(uint32 time);


//=============== OSAL
uint8 osal_set_event0( uint8 task_id, uint16 event_flag );
uint8 osal_msg_send0( uint8 destination_task, uint8 *msg_ptr );

//=============== _HAL_IRQ_
void drv_irq_init0(void);
int drv_enable_irq0(int cs);
int drv_disable_irq0(void);

void set_channel0(uint32_t  channel);
void set_whiten_seed0(uint32_t channel);



#endif // _JUMP_FUNC_H_
