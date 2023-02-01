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

#include <stdlib.h>
#include <string.h>

//#include "common.h"
//#include "uart.h"
//#include "dma.h"
//#include "flash.h"
//#include "gpio_rom.h"
//#include "i2c.h"
//#include "i2s.h"
//#include "spi.h"
//#include "timer.h"

#include "ll.h"
#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "uart.h"
#include "ll_sleep.h"
#include "ll_debug.h"
#include "ll.h"
#include "bus_dev.h"
#include "ll_hw_drv.h"
#include "gpio.h"
#include "ll_enc.h"
#include "OSAL_Clock.h"
#include "osal_bufmgr.h"
#include "OSAL_Memory.h"
#include "log.h"
#include "hci.h"
#include "hci_tl.h"
#include "version.h"
#include "flash.h"
#include "gatt.h"
#include "att.h"
#include "error.h"
#include "clock.h"
//========================================================
// build config
#define WFI()   __WFI()

#define LL_MODE_INVALID       0xFF
#define LL_MODE_LEGACY        0x00
#define LL_MODE_EXTENDED      0x01

#define LL_HW_MODE_STX           0x00
#define LL_HW_MODE_SRX           0x01
#define LL_HW_MODE_TRX           0x02
#define LL_HW_MODE_RTX           0x03
#define LL_HW_MODE_TRLP          0x04
#define LL_HW_MODE_RTLP          0x05

#define LL_COPY_DEV_ADDR_LE( dstPtr, srcPtr )        {                          \
        (dstPtr)[0] = (srcPtr)[0];                                                   \
        (dstPtr)[1] = (srcPtr)[1];                                                   \
        (dstPtr)[2] = (srcPtr)[2];                                                   \
        (dstPtr)[3] = (srcPtr)[3];                                                   \
        (dstPtr)[4] = (srcPtr)[4];                                                   \
        (dstPtr)[5] = (srcPtr)[5];}

#define LL_WINDOW_SIZE                2         // 2.5ms in 1.25ms ticks

#define LL_CALC_NEXT_SCAN_CHN(chan)     { chan ++; \
        chan = (chan > LL_SCAN_ADV_CHAN_39) ? LL_SCAN_ADV_CHAN_37 : chan;}

#define   CONN_CSA2_ALLOW                  0x00000080

// LE Event Lengths
#define HCI_CMD_COMPLETE_EVENT_LEN                  3
#define HCI_CMD_VS_COMPLETE_EVENT_LEN               2
#define HCI_CMD_STATUS_EVENT_LEN                    4
#define HCI_NUM_COMPLETED_PACKET_EVENT_LEN          5
#define HCI_FLUSH_OCCURRED_EVENT_LEN                2
#define HCI_REMOTE_VERSION_INFO_EVENT_LEN           8
#define HCI_CONNECTION_COMPLETE_EVENT_LEN           19
#define HCI_ENH_CONN_COMPLETE_EVENT_LEN             31
#define HCI_DISCONNECTION_COMPLETE_LEN              4
#define HCI_LL_CONN_UPDATE_COMPLETE_LEN             10
#define HCI_ADV_REPORT_EVENT_LEN                    12
#define HCI_READ_REMOTE_FEATURE_COMPLETE_EVENT_LEN  12
#define HCI_LTK_REQUESTED_EVENT_LEN                 13
#define HCI_DATA_BUF_OVERFLOW_EVENT_LEN             1
#define HCI_ENCRYPTION_CHANGE_EVENT_LEN             4
#define HCI_KEY_REFRESH_COMPLETE_EVENT_LEN          3
#define HCI_BUFFER_OVERFLOW_EVENT_LEN               1
#define HCI_LL_DATA_LENGTH_CHANGE_EVENT_LEN         11
#define HCI_LL_PHY_UPDATE_COMPLETE_EVENT_LEN        6

#define HCI_EXT_ADV_REPORT_EVENT_LEN                26   //18
#define HCI_PRD_ADV_SYNC_ESTAB_EVENT_LEN            16
#define HCI_PRD_ADV_REPORT_EVENT_LEN                8
#define HCI_PRD_ADV_SYNC_LOST_EVENT_LEN             2
#define HCI_ADV_SET_TERM_EVENT_LEN                  6
#define HCI_SCAN_REQ_RECV_EVENT_LEN                 7
#define HCI_CHN_SEL_ALGO_EVENT_LEN                  4

#define HCI_SCAN_TIMEOUT_EVENT_LEN                  1

// LE Event mask - 1st octet
#define LE_EVT_MASK_CONN_COMPLETE                   0x00000001
#define LE_EVT_MASK_ADV_REPORT                      0x00000002
#define LE_EVT_MASK_CONN_UPDATE_COMPLETE            0x00000004
#define LE_EVT_MASK_READ_REMOTE_FEATURE             0x00000008

#define LE_EVT_MASK_LTK_REQUEST                     0x00000010
#define LE_EVT_MASK_REMOTE_CONN_PARAM_REQ           0x00000020
#define LE_EVT_MASK_DATA_LENGTH_CHANGE              0x00000040
#define LE_EVT_MASK_LOCAL_P256_PUB_KEY_CMP          0x00000080

// LE Event mask - 2nd octet
#define LE_EVT_MASK_GEN_DHKEY_CMP                   0x00000100
#define LE_EVT_MASK_ENH_CONN_CMP                    0x00000200
#define LE_EVT_MASK_DIRECT_ADV_RPT                  0x00000400
#define LE_EVT_MASK_PHY_CHANGE                      0x00000800

#define LE_EVT_MASK_EXT_ADV_RPT                     0x00001000
#define LE_EVT_MASK_PRD_ADV_SYNC_EST                0x00002000
#define LE_EVT_MASK_PRD_ADV_RPT                     0x00004000
#define LE_EVT_MASK_PRD_ADV_SYNC_LOST               0x00008000

// LE Event mask - 3rd octet
#define LE_EVT_MASK_SCAN_TO                         0x00010000
#define LE_EVT_MASK_ADV_SET_TERM                    0x00020000
#define LE_EVT_MASK_SCAN_REQ_RECV                   0x00040000
#define LE_EVT_MASK_CHN_SEL_ALGO                    0x00080000

#define LE_EVT_MASK_CONNECTIONLESS_IQ_RPT           0x00100000
#define LE_EVT_MASK_CONNECTION_IQ_RPT               0x00200000
#define LE_EVT_MASK_CTE_REQ_FAIL                    0x00400000
#define LE_EVT_MASK_PRD_ADV_SYNC_TRANSFER_RECV      0x00800000

#define HCI_EVT_INDEX_LE                            7
#define HCI_EVT_MASK_LE                             0x20

typedef struct
{
    osal_event_hdr_t  hdr;
    uint8  BLEEventCode;
} hciEvt_ScanTimeout_t;

#define BDADDR_VALID( bdAddr )                                                 \
    ( !(                                                                         \
                                                                                 ((bdAddr)[0] == 0xFF) &&                                                \
                                                                                 ((bdAddr)[1] == 0xFF) &&                                                \
                                                                                 ((bdAddr)[2] == 0xFF) &&                                                \
                                                                                 ((bdAddr)[3] == 0xFF) &&                                                \
                                                                                 ((bdAddr)[4] == 0xFF) &&                                                \
                                                                                 ((bdAddr)[5] == 0xFF)                                                   \
       )                                                                         \
    )

#define HCI_BLE_SCAN_TIMEOUT_EVENT                     0x11
#define LE_ADV_PROP_SCAN_RSP_BITMASK        0x08
#define LE_ADV_PROP_MORE_DATA_BITMASK       0x20
#define LE_ADV_PROP_DATA_TRUNCATED_BITMASK  0x40
//------------------------------------------------------------------------------------
//extern rom function
//
//extern int gpio_write(gpio_pin_e pin, bit_action_e en);
extern uint8   ll_processExtAdvIRQ(uint32_t      irq_status);
extern uint8   ll_processPrdAdvIRQ(uint32_t      irq_status);
extern uint8   ll_processExtScanIRQ(uint32_t      irq_status);
extern uint8   ll_processExtInitIRQ(uint32_t      irq_status);
extern uint8   ll_processPrdScanIRQ(uint32_t      irq_status);
extern uint8   ll_processBasicIRQ(uint32_t      irq_status);
extern uint32  read_ll_adv_remainder_time(void);

extern int clear_timer_int(AP_TIM_TypeDef* TIMx);
extern uint8 isTimer1Running(void);
extern uint8 isTimer4Running(void);
extern void clear_timer(AP_TIM_TypeDef* TIMx);

extern uint8 ll_processMissMasterEvt(uint8 connId);
extern uint8 ll_processMissSlaveEvt(uint8 connId);
extern int gpio_write(GPIO_Pin_e pin, uint8_t en);
extern void ll_hw_tx2rx_timing_config(uint8 pkt);
extern void wakeup_init0(void);
extern void enter_sleep_off_mode0(Sleep_Mode mode);
extern void spif_release_deep_sleep(void);
extern void spif_set_deep_sleep(void);

extern uint8 ll_hw_get_tr_mode(void);
extern int ll_hw_get_rfifo_depth(void);
//extern void move_to_master_function(void);

extern void llWaitUs(uint32_t wtTime);

extern struct buf_tx_desc g_tx_adv_buf;
extern struct buf_tx_desc g_tx_ext_adv_buf;
extern struct buf_tx_desc tx_scanRsp_desc;

extern struct buf_rx_desc g_rx_adv_buf;

extern uint8_t  ll_hw_read_rfifo1(uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1);
extern void llPrdAdvDecideNextChn(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
extern void llSetupSyncInfo(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
//-----------------------------------------------------------------------------------
//extern rom  variable
//
extern uint32* pGlobal_config;


extern uint32   g_interAuxPduDuration;

extern uint32 hclk_per_us;
extern uint32 hclk_per_us_shift;
extern volatile uint8 g_clk32K_config;
/////////////////////////

extern uint32 sleep_flag;
extern uint32 osal_sys_tick;
extern uint32 ll_remain_time;

extern uint32 llWaitingIrq;
extern uint32 ISR_entry_time;

extern uint32 counter_tracking;

extern uint32_t  __initial_sp;
extern uint32_t  g_smartWindowSize;
extern volatile uint8_t g_same_rf_channel_flag;
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32_t  g_TIM2_IRQ_PendingTick;
extern uint32_t  g_osal_tick_trim;
extern uint32_t  g_osalTickTrim_mod;
extern uint32_t  g_TIM2_wakeup_delay;
extern uint32_t  rtc_mod_value;
extern uint32_t  g_counter_traking_cnt;
extern uint32_t  sleep_tick;
extern uint32_t  g_wakeup_rtc_tick;
extern int slave_conn_event_recv_delay;
extern uint8  g_llScanMode;
extern uint8 g_currentPeerAddrType;
extern uint8 g_currentPeerRpa[LL_DEVICE_ADDR_LEN];
extern uint8 ownRandomAddr[];
extern uint16_t ll_hw_get_tfifo_wrptr(void);
extern uint32_t llCurrentScanChn;
extern uint8 ownPublicAddr[];
extern uint32_t llScanTime;
extern uint32_t llScanT1;
extern uint8    isPeerRpaStore;
extern uint8    currentPeerRpa[LL_DEVICE_ADDR_LEN];
extern uint8    storeRpaListIndex;
extern uint8    g_currentLocalAddrType;
extern uint8    g_currentLocalRpa[LL_DEVICE_ADDR_LEN];
//extern llPduLenManagment_t g_llPduLen;

extern int16  g_rfTxPathCompensation, g_rfRxPathCompensation;
extern uint8 ownPublicAddr[];

extern uint8_t   llSecondaryState;            // secondary state of LL
extern uint8     g_llAdvMode;
extern scannerSyncInfo_t    scanSyncInfo;
extern periodicAdvertiserListInfo_t g_llPeriodicAdvlist[];
extern uint8          g_llPrdAdvDeviceNum;                     // current periodic advertiser device number
extern syncInfo_t   syncInfo;
extern uint32 g_new_master_delta;
//----------------------------------------------------------------------------------------------
// patch local function
uint8 llSetupExtAdvLegacyEvent(extAdvInfo_t*  pAdvInfo);
void ll_hw_trx_settle_bb(uint8 pkt, uint8 bb_delay);
void ll_hw_trx_settle_config(uint8 pkt);
void LL_IRQHandler2(void);
void llSetupAdvExtIndPDU0(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
void llSetupAuxConnectRspPDU0(extAdvInfo_t*  pAdvInfo);
void LL_slave_conn_event1(void);
void LL_ExtAdvReportCback0( uint16 advEvt,
                            uint8   advAddrType,
                            uint8*  advAddr,
                            uint8   primaryPHY,
                            uint8   secondaryPHY,
                            uint8   advertisingSID,
                            uint8   txPower,
                            int8    rssi,
                            uint16  periodicAdvertisingInterval,
                            uint8   directAddrType,
                            uint8*   directAddr,
                            uint8   dataLen,
                            uint8*   rptData);
void LL_ScanTimeoutCback(void);
void llSetupExtScan( uint8 chan );
void LL_IRQHandler3(void);
void llSetupAuxAdvIndPDU1(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
void move_to_master_function1(void);
void llSetupExtInit(void);
void llSetupAuxChainIndPDU1(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv);
void llSetupAuxScanRspPDU1(extAdvInfo_t*  pAdvInfo);

//xip patch
void __ATTR_FUNC_XIP__(init_extadv_config)(void);
void __ATTR_FUNC_XIP__(init_extscan_config)(void);
llStatus_t __ATTR_FUNC_XIP__(LL_SetExtAdvSetRandomAddress)( uint8 adv_handle,
                                                            uint8* random_address
                                                          );
llStatus_t __ATTR_FUNC_XIP__(LL_SetExtAdvParam)( uint8 adv_handle,
                                                 uint16 adv_event_properties,
                                                 uint32 primary_advertising_interval_Min,          // 3 octets
                                                 uint32 primary_advertising_interval_Max,          // 3 octets
                                                 uint8  primary_advertising_channel_map,
                                                 uint8  own_address_type,
                                                 uint8  peer_address_type,
                                                 uint8* peer_address,
                                                 uint8  advertising_filter_policy,
                                                 int8   advertising_tx_power,
                                                 uint8  primary_advertising_PHY,
                                                 uint8  secondary_advertising_max_skip,
                                                 uint8  secondary_advertising_PHY,
                                                 uint8  advertising_SID,
                                                 uint8  scan_request_notification_enable,
                                                 int8*  selectTxPwr
                                               );
llStatus_t __ATTR_FUNC_XIP__(LL_SetExtAdvData)( uint8  adv_handle,
                                                uint8  operation,
                                                uint8  fragment_preference,
                                                uint8  advertising_data_length,
                                                uint8* advertising_data
                                              );
llStatus_t __ATTR_FUNC_XIP__(LL_SetExtScanRspData)( uint8 adv_handle,
                                                    uint8 operation,
                                                    uint8  fragment_preference,
                                                    uint8  scan_rsp_data_length,
                                                    uint8* scan_rsp_data
                                                  );
llStatus_t __ATTR_FUNC_XIP__(LL_SetExtAdvEnable)( uint8  enable,
                                                  uint8  number_of_sets,
                                                  uint8*  advertising_handle,
                                                  uint16* duration,
                                                  uint8*  max_extended_advertising_events);
llStatus_t __ATTR_FUNC_XIP__(LL_ReadMaximumAdvDataLength)( uint16* length );
llStatus_t __ATTR_FUNC_XIP__(LL_ReadNumberOfSupportAdvSet)( uint8* number );
llStatus_t __ATTR_FUNC_XIP__(LL_RemoveAdvSet)( uint8 adv_handle);
llStatus_t __ATTR_FUNC_XIP__(LL_ClearAdvSets)(void);
llStatus_t __ATTR_FUNC_XIP__(LL_SetExtendedScanParameters)(uint8 own_address_type,
                                                           uint8 scanning_filter_policy,
                                                           uint8 scanning_PHYs,
                                                           uint8* scan_type,
                                                           uint16* scan_interval,
                                                           uint16* scan_window);
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_SetExtAdvSetRandomAddressCmd)( uint8 adv_handle,
                                                                    uint8* random_address
                                                                  );
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_SetExtAdvParamCmd)( uint8 adv_handle,
                                                         uint16 adv_event_properties,
                                                         uint32 primary_advertising_interval_Min,          // 3 octets
                                                         uint32 primary_advertising_interval_Max,          // 3 octets
                                                         uint8  primary_advertising_channel_map,
                                                         uint8  own_address_type,
                                                         uint8  peer_address_type,
                                                         uint8* peer_address,
                                                         uint8  advertising_filter_policy,
                                                         int8   advertising_tx_power,
                                                         uint8  primary_advertising_PHY,
                                                         uint8  secondary_advertising_max_skip,
                                                         uint8  secondary_advertising_PHY,
                                                         uint8  advertising_SID,
                                                         uint8  scan_request_notification_enable
                                                       );
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_SetExtAdvDataCmd)( uint8 adv_handle,
                                                        uint8 operation,
                                                        uint8  fragment_preference,
                                                        uint8  advertising_data_length,
                                                        uint8* advertising_data
                                                      );
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_SetExtScanRspDataCmd)( uint8 adv_handle,
                                                            uint8 operation,
                                                            uint8  fragment_preference,
                                                            uint8  scan_rsp_data_length,
                                                            uint8* scan_rsp_data
                                                          );
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_SetExtAdvEnableCmd)( uint8  enable,
                                                          uint8  number_of_sets,
                                                          uint8*  advertising_handle,
                                                          uint16* duration,
                                                          uint8*  max_extended_advertising_events);
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_ReadMaximumAdvDataLengthCmd)( void );
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_ReadNumberOfSupportAdvSetCmd)( void );
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_RemoveAdvSetCmd)( uint8 adv_handle);
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_ClearAdvSetsCmd)( void);
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_SetExtendedScanParametersCmd)(uint8 own_address_type,
                                                                   uint8 scanning_filter_policy,
                                                                   uint8 scanning_PHYs,
                                                                   uint8* scan_sype,
                                                                   uint16* scan_interval,
                                                                   uint16* scan_window);
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_SetExtendedScanEnableCmd)(uint8 enable,
                                                               uint8 filter_duplicates,
                                                               uint16 duration,
                                                               uint16 period);
hciStatus_t __ATTR_FUNC_XIP__(HCI_LE_ExtendedCreateConnectionCmd)(uint8 initiator_filter_policy,
                                                                  uint8 own_address_type,
                                                                  uint8 peer_address_type,
                                                                  uint8* peer_address,
                                                                  uint8  initiating_PHYs,
                                                                  uint16* scan_interval,
                                                                  uint16* scan_window,
                                                                  uint16* conn_interval_min,
                                                                  uint16* conn_interval_max,
                                                                  uint16* conn_latency,
                                                                  uint16* supervision_timeout,
                                                                  uint16* minimum_CE_length,
                                                                  uint16* maximum_CE_length);
llStatus_t __ATTR_FUNC_XIP__(LL_InitialExtendedAdv)( extAdvInfo_t* extAdvInfo,
                                                     uint8         extAdvSetNumber,
                                                     uint16        advSetMaxLen);

//----------------------------------------------------------------------------------------------
//patch
uint32 g_auxconnreq_ISR_entry_time;
uint8  activeScanAdi;               // for active scan, the ADI field may not present in aux_scan_rsp, keep the info in AUX_ADV_IND
uint32_t llScanDuration = 0;
uint16 extscanrsp_offset = 0;

//#ifdef __BUILD_RF_LIB_EXT_ADV__
void TIM4_IRQHandler(void);
void init_extadv_config(void)
{
    pGlobal_config = (uint32*)(CONFIG_BASE_ADDR);
    //-------------------------------------------------------------------
    // patch function register
    //--------------------------------------------------------------------
    JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&LL_IRQHandler2;
    JUMP_FUNCTION(V23_IRQ_HANDLER)                   =   (uint32_t)&TIM4_IRQHandler;
    JUMP_FUNCTION(LL_ADV_SCHEDULER)                  =   (uint32_t)&ll_adv_scheduler0;
    JUMP_FUNCTION(LL_ADV_ADD_TASK)                  =   (uint32_t)&ll_add_adv_task0;
    JUMP_FUNCTION(LL_ADV_DEL_TASK)                  =   (uint32_t)&ll_delete_adv_task0;
    JUMP_FUNCTION(LL_SETUP_EXT_ADV_EVENT)           =   (uint32_t)&llSetupExtAdvEvent0;
    JUMP_FUNCTION(LL_SETUP_ADV_EXT_IND_PDU)           =   (uint32_t)&llSetupAdvExtIndPDU0;
    JUMP_FUNCTION(LL_SLAVE_CONN_EVENT)           =   (uint32_t)&LL_slave_conn_event1;
    JUMP_FUNCTION(LL_SETUP_AUX_ADV_IND_PDU)           =   (uint32_t)&llSetupAuxAdvIndPDU1;
    JUMP_FUNCTION(LL_SETUP_AUX_CHAIN_IND_PDU)       = (uint32_t)&llSetupAuxChainIndPDU1;
    JUMP_FUNCTION(LL_SETUP_AUX_SCAN_RSP_PDU)        = (uint32_t)&llSetupAuxScanRspPDU1;
    pGlobal_config[LL_SWITCH] |= CONN_CSA2_ALLOW;
    pGlobal_config[LL_EXT_ADV_TASK_DURATION] = 17000;//20000;
    pGlobal_config[LL_PRD_ADV_TASK_DURATION] = 20000;
    pGlobal_config[LL_CONN_TASK_DURATION] = 5000;
    pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT] = 1500;//5000;
    pGlobal_config[LL_EXT_ADV_INTER_SEC_CHN_INT] = 2500;//5000;//20000;//5000;
    pGlobal_config[LL_EXT_ADV_INTER_SEC_CHN_INT_2MPHY] = 1400;
    pGlobal_config[LL_EXT_ADV_PRI_2_SEC_CHN_INT] = 1500;
    pGlobal_config[LL_EXT_ADV_RSC_PERIOD] = 400000;//1000000;
    pGlobal_config[LL_EXT_ADV_RSC_SLOT_DURATION] = 10000;
    pGlobal_config[LL_PRD_ADV_RSC_PERIOD] = 1000000;
    pGlobal_config[LL_PRD_ADV_RSC_SLOT_DURATION] = 10000;
    pGlobal_config[LL_EXT_ADV_PROCESS_TARGET] = 150;//1000;//500;
    pGlobal_config[LL_PRD_ADV_PROCESS_TARGET] = 150;//500;

//    ownPublicAddr[0] = 0x22;//0x55;
//    ownPublicAddr[1] = 0x22;//0x55;
//    ownPublicAddr[2] = 0x44;
//    ownPublicAddr[3] = 0x44;
//    ownPublicAddr[4] = 0x55;//0x22;
//    ownPublicAddr[5] = 0x45;//0x22;

    //config aux scanrsp delay,aux connrsp delay
    if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[EXT_ADV_AUXSCANRSP_DELAY_1MPHY] = 7+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXCONNRSP_DELAY_1MPHY] = 7+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXSCANRSP_DELAY_2MPHY] = 7+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXCONNRSP_DELAY_2MPHY] = 7+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXSCANRSP_DELAY_125KPHY] = 63;
        pGlobal_config[EXT_ADV_AUXCONNRSP_DELAY_125KPHY] = 63;
    }
}

void init_extscan_config(void)
{
    pGlobal_config = (uint32*)(CONFIG_BASE_ADDR);
    //-------------------------------------------------------------------
    // patch function register
    //--------------------------------------------------------------------
    JUMP_FUNCTION(V4_IRQ_HANDLER)                   =   (uint32_t)&LL_IRQHandler3;
    JUMP_FUNCTION(V23_IRQ_HANDLER)                   =   (uint32_t)&TIM4_IRQHandler;
    pGlobal_config[LL_SWITCH] |= CONN_CSA2_ALLOW;

    if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[EXT_ADV_AUXSCANREQ_DELAY_1MPHY] = 8+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXCONNREQ_DELAY_1MPHY] = 8+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXSCANREQ_DELAY_2MPHY] = 8+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXCONNREQ_DELAY_2MPHY] = 8+RF_PHY_EXT_PREAMBLE_US;
        pGlobal_config[EXT_ADV_AUXSCANREQ_DELAY_125KPHY] =64;
        pGlobal_config[EXT_ADV_AUXCONNREQ_DELAY_125KPHY] =0;
    }
}

// =============================================================
//       extended adv procedure patch
#define LL_ADV_TIMING_COMPENSATE          5                 // rough compensation for the time not consider in calculation

uint8 LL_extAdvTimerExpProcess(void)
{
    extAdvInfo_t*  pAdvInfo;
    uint8  current_chn;
    //uint16 current_offset;
    // TODO: if the timer IRQ is pending, should advanced expiry tick
    g_timerExpiryTick = read_current_fine_time();
    pAdvInfo = g_pAdvSchInfo[g_currentExtAdv].pAdvInfo;
    current_chn = pAdvInfo->currentChn;
    //current_offset = pAdvInfo->currentAdvOffset;
    // update scheduler task list
    ll_updateExtAdvRemainderTime(g_currentAdvTimer + LL_ADV_TIMING_COMPENSATE);

    // check timer1, if no enough margin, start timer
    if (llWaitingIrq ||
            (isTimer1Running() && read_LL_remainder_time() < pGlobal_config[LL_EXT_ADV_TASK_DURATION]))
    {
        ll_ext_adv_schedule_next_event(pGlobal_config[LL_EXT_ADV_TASK_DURATION]);
        return TRUE;
    }
    else
    {
        if (ll_isLegacyAdv(pAdvInfo))
            llSetupExtAdvLegacyEvent(pAdvInfo);       // send legacy ADV PDU
        else
            llSetupExtAdvEvent(pAdvInfo);       // send extended advertisement
    }

    // for 1st pri adv channel , update remainder time in scheduler
    if (ll_isFirstAdvChn(pAdvInfo->parameter.priAdvChnMap, current_chn))
    {
//      g_pAdvSchInfo[g_currentExtAdv].nextEventRemainder += pAdvInfo->primary_advertising_interval;       // comment out 2020-0909, the interval should have beed added in function ll_updateExtAdvRemainderTime
//      g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->adv_event_duration += pAdvInfo->primary_advertising_interval;
        g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->adv_event_counter ++;

        // event expiry decision, update 2020-04-07
        if (g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->duration != 0 &&
                g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->adv_event_duration > g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->duration)
        {
            g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->active = FALSE;          // mark as inactive
            LL_AdvSetTerminatedCback(LL_STATUS_ERROR_DIRECTED_ADV_TIMEOUT,
                                     g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->advHandle,
                                     0,
                                     g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->adv_event_counter - 1);
        }
        else if (g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->maxExtAdvEvents != 0
                 && g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->adv_event_counter >= g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->maxExtAdvEvents)
        {
            g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->active = FALSE;          // mark as inactive
            LL_AdvSetTerminatedCback(LL_STATUS_ERROR_LIMIT_REACHED,
                                     g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->advHandle,
                                     0,
                                     g_pAdvSchInfo[g_currentExtAdv].pAdvInfo->adv_event_counter - 1);
        }
    }

    return TRUE;
}


void LL_extScanTimerExpProcess(void)
{
    if (extScanInfo.enable == TRUE)
        llSetupExtScan(extScanInfo.current_chn);
}

void LL_extInitTimerExpProcess(void)
{
    llSetupExtInit();
}

extern uint32 g_timer4_irq_pending_time;
void TIM4_IRQHandler(void)
{
    HAL_ENTER_CRITICAL_SECTION();

    if(AP_TIM4->status&0x1)
    {
        g_timer4_irq_pending_time = AP_TIM4->CurrentCount - AP_TIM4->LoadCount;    // TODO: check the formula
        clear_timer_int(AP_TIM4);
        clear_timer(AP_TIM4);

        if (g_currentTimerTask == LL_TASK_EXTENDED_ADV)
            LL_extAdvTimerExpProcess();
        else if (g_currentTimerTask == LL_TASK_PERIODIC_ADV)
            LL_prdAdvTimerExpProcess();
        else if (g_currentTimerTask == LL_TASK_EXTENDED_SCAN)
            LL_extScanTimerExpProcess();
        else if (g_currentTimerTask == LL_TASK_EXTENDED_INIT)
            LL_extInitTimerExpProcess();
        else if (g_currentTimerTask == LL_TASK_PERIODIC_SCAN)
            LL_prdScanTimerExpProcess();
    }

    HAL_EXIT_CRITICAL_SECTION();
}

uint8 llSetupExtAdvLegacyEvent(extAdvInfo_t*  pAdvInfo)
{
    uint8 ch_idx, pktFmt;
    int i;
    uint8 pduType;
    ch_idx = pAdvInfo->currentChn;

    //LOG("L:%d",ch_idx);
    if (ch_idx < LL_ADV_CHAN_FIRST || ch_idx > LL_ADV_CHAN_LAST )
        return FALSE;

    // fill advertisement PDU
    g_tx_ext_adv_buf.txheader = 0;

    // AdvA
    if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)
        memcpy(&g_tx_ext_adv_buf.data[0], pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
    else    // public address
        memcpy(&g_tx_ext_adv_buf.data[0], ownPublicAddr, LL_DEVICE_ADDR_LEN);

    switch (pAdvInfo->parameter.advEventProperties)
    {
    case LL_EXT_ADV_PROP_ADV_IND:
        pduType = ADV_IND;
        // Length
        SET_BITS(g_tx_ext_adv_buf.txheader, (6 + pAdvInfo->data.advertisingDataLength), LENGTH_SHIFT, LENGTH_MASK);
        // AdvData
        osal_memcpy((uint8_t*)&(g_tx_ext_adv_buf.data[6]), &pAdvInfo->data.advertisingData[0], pAdvInfo->data.advertisingDataLength);
        break;

    case LL_EXT_ADV_PROP_ADV_SCAN_IND:
        pduType = ADV_SCAN_IND;
        // Length
        SET_BITS(g_tx_ext_adv_buf.txheader, (6 + pAdvInfo->data.advertisingDataLength), LENGTH_SHIFT, LENGTH_MASK);
        // AdvData
        osal_memcpy((uint8_t*)&(g_tx_ext_adv_buf.data[6]), pAdvInfo->data.advertisingData, pAdvInfo->data.advertisingDataLength);
        break;

    case LL_EXT_ADV_PROP_ADV_NOCONN_IND:
        pduType = ADV_NONCONN_IND;
        // Length
        SET_BITS(g_tx_ext_adv_buf.txheader, (6 + pAdvInfo->data.advertisingDataLength), LENGTH_SHIFT, LENGTH_MASK);
        // AdvData
        osal_memcpy((uint8_t*)&(g_tx_ext_adv_buf.data[6]), pAdvInfo->data.advertisingData, pAdvInfo->data.advertisingDataLength);
        break;

    case LL_EXT_ADV_PROP_ADV_LDC_ADV:
    case LL_EXT_ADV_PROP_ADV_HDC_ADV:
        pduType = ADV_DIRECT_IND;
        // Length
        SET_BITS(g_tx_ext_adv_buf.txheader, 12, LENGTH_SHIFT, LENGTH_MASK);
        SET_BITS(g_tx_ext_adv_buf.txheader, pAdvInfo->parameter.peerAddrType, RX_ADD_SHIFT, RX_ADD_MASK);
        // initA
        osal_memcpy((uint8_t*)&(g_tx_ext_adv_buf.data[6]), &pAdvInfo->parameter.peerAddress[0], 6);
        break;

    default:
        break;
    }

    // PDU type, 4 bits
    SET_BITS(g_tx_ext_adv_buf.txheader, pduType, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
    // RFU, ChSel, TxAdd, RxAdd
    SET_BITS(g_tx_ext_adv_buf.txheader, pAdvInfo->parameter.ownAddrType, TX_ADD_SHIFT, TX_ADD_MASK);

    if ((pduType == ADV_IND
            || pduType == ADV_DIRECT_IND)
            && pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
        SET_BITS(g_tx_ext_adv_buf.txheader, 1, CHSEL_SHIFT, CHSEL_MASK);

    // decide next adv channel
    i = ch_idx - LL_ADV_CHAN_FIRST + 1;

    while ((i < 3) && !(pAdvInfo->parameter.priAdvChnMap & (1 << i))) i ++;       // search channel map for next adv channel number

    if (i == 3)   // finish primary adv channel broadcast
    {
        pAdvInfo->currentChn = ll_getFirstAdvChn(pAdvInfo->parameter.priAdvChnMap);
    }
    else
        pAdvInfo->currentChn = LL_ADV_CHAN_FIRST + i;

    // Legacy Adv always in 1M PHY
    pktFmt = LE_1M_PHY;
    HAL_ENTER_CRITICAL_SECTION();

    // if there is ongoing LL HW task, skip this task
    if (llWaitingIrq)
    {
        HAL_EXIT_CRITICAL_SECTION();
        return FALSE;
    }

    //============== configure and trigger LL HW engine, LL HW work in Single Tx mode  ==================
    rf_phy_change_cfg0(pktFmt);
    ll_hw_tx2rx_timing_config(pktFmt);
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    set_access_address(ADV_SYNCH_WORD);   // access address
    set_channel(ch_idx);             // channel
    set_whiten_seed(ch_idx);         // whiten seed
    set_max_length(50);            // rx PDU max length
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();

    // for AUX_ADV_IND, connectable/scannable case, should configure TRX
    if ((pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_CONN_BITMASK) ||
            (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK))
    {
        ll_hw_set_trx_settle(pGlobal_config[LL_HW_BB_DELAY_ADV],
                             pGlobal_config[LL_HW_AFE_DELAY_ADV],
                             pGlobal_config[LL_HW_PLL_DELAY_ADV]);        //TxBB,RxAFE,PLL
        ll_hw_set_trx();                      // set LL HW as Tx - Rx mode
    }
    else
    {
        ll_hw_set_stx();                      // set LL HW as Tx - Rx mode
    }

    //ll_hw_ign_rfifo(LL_HW_IGN_ALL);         //set the rfifo ign control
    ll_hw_ign_rfifo(LL_HW_IGN_EMP | LL_HW_IGN_CRC);
    //write Tx FIFO
    ll_hw_write_tfifo((uint8*)&(g_tx_ext_adv_buf.txheader), ((g_tx_ext_adv_buf.txheader & 0xff00) >> 8) + 2);
    ll_hw_go();
    llWaitingIrq = TRUE;
    llTaskState = LL_TASK_EXTENDED_ADV;
    HAL_EXIT_CRITICAL_SECTION();

    // 2020-12-2, for ADV_IND/ADV_SCAN_IND, need config scan rsp data
    if (pAdvInfo->parameter.advEventProperties == LL_EXT_ADV_PROP_ADV_IND
            || pAdvInfo->parameter.advEventProperties == LL_EXT_ADV_PROP_ADV_SCAN_IND)
    {
        SET_BITS(tx_scanRsp_desc.txheader, ADV_SCAN_RSP, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
        SET_BITS(tx_scanRsp_desc.txheader, (g_currentLocalAddrType == 0 ? 0: 1), TX_ADD_SHIFT, TX_ADD_MASK);

        if (g_currentLocalAddrType  == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)  // per adv set random address may be updated after generate adv_param.ownAddr
            memcpy(tx_scanRsp_desc.data, pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
        else
            memcpy(tx_scanRsp_desc.data, adv_param.ownAddr, LL_DEVICE_ADDR_LEN);

        osal_memcpy( (uint8_t*) &(tx_scanRsp_desc.data[6]), pAdvInfo->scanRspData, pAdvInfo->scanRspMaxLength);
        SET_BITS(tx_scanRsp_desc.txheader, (pAdvInfo->scanRspMaxLength + 6), LENGTH_SHIFT, LENGTH_MASK);
    }

    return TRUE;
}

extern void debug_probe(void);

/*******************************************************************************
    @fn          llSetupExtAdvEvent

    @brief       This function will setup ext adv event
                1. fill ext adv pdu(EXT_ADV_IND or AUX_XXX_IND)
                2. update timer info for next chn EXT_ADV_IND or AUX_XXX_IND

    input parameters

    @param       None.

    output== parameters

    @param       None.

    @return      LL_STATUS_SUCCESS
*/
uint8 llSetupExtAdvEvent0(extAdvInfo_t*  pAdvInfo)
{
    uint8 ch_idx, pktFmt, auxPduIndFlag = FALSE;
    int i;
    uint32 T2, T1, delta, temp;
    ch_idx = pAdvInfo->currentChn;
    T1 = read_current_fine_time();

    //LOG("E:%d ", pAdvInfo->currentChn);

    if (ch_idx >= LL_ADV_CHAN_FIRST && ch_idx <= LL_ADV_CHAN_LAST )   // advertise at primary channel case
    {
        llSetupAdvExtIndPDU(pAdvInfo, NULL);
        // decide next adv channel
        i = ch_idx - LL_ADV_CHAN_FIRST + 1;

        while ((i < 3) && !(pAdvInfo->parameter.priAdvChnMap & (1 << i))) i ++;       // search channel map for next adv channel number

        if (i == 3)   // finish primary adv channel broadcast
        {
            if (g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder != LL_INVALID_TIME
                    && g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder > g_pAdvSchInfo[g_currentExtAdv].nextEventRemainder)
                pAdvInfo->currentChn = ll_getFirstAdvChn(pAdvInfo->parameter.priAdvChnMap);
            else
                pAdvInfo->currentChn = pAdvInfo->auxChn;
        }
        else
            pAdvInfo->currentChn = LL_ADV_CHAN_FIRST + i;

        // config primary PHY
        if (pAdvInfo->parameter.primaryAdvPHY == LL_PHY_CODE)
            pktFmt = PKT_FMT_BLR125K;
        else
            pktFmt = pAdvInfo->parameter.primaryAdvPHY;
    }
    else  // advertise at primary channel case
    {
        // Note: if the Ext adv has no aux pdu, llSetupExtAdv should not be invoked
        if ((extscanrsp_offset != 0) && (extscanrsp_offset < pAdvInfo->scanRspMaxLength))
            llSetupAuxChainIndPDU(pAdvInfo, NULL);
        else if(pAdvInfo->currentAdvOffset == 0)    // 1st AUX PDU. AUX_ADV_IND should include advData
        {
            llSetupAuxAdvIndPDU(pAdvInfo, NULL);
            auxPduIndFlag = TRUE;
        }
        else
            llSetupAuxChainIndPDU(pAdvInfo, NULL);

        // config secondary PHY
        if (pAdvInfo->parameter.secondaryAdvPHY == LL_SECOND_ADV_PHY_CODE)           // coded PHY
            pktFmt = PKT_FMT_BLR125K;          //
        else
            pktFmt = pAdvInfo->parameter.secondaryAdvPHY;
    }

    HAL_ENTER_CRITICAL_SECTION();

    // if there is ongoing LL HW task, skip this task
    if (llWaitingIrq)
    {
//      g_pmCounters.ll_tbd_cnt1++;
        HAL_EXIT_CRITICAL_SECTION();
        return FALSE;
    }

    //============== configure and trigger LL HW engine, LL HW work in Single Tx mode  ==================
    g_rfPhyPktFmt = pktFmt;
//=====================
//  *(volatile uint32_t *) 0x40030000 = 0x3d068001; // set tx pkt =2
//  *(volatile uint32_t *) 0x400300bc = 0x834;      //[7:0] pll_tm [11:8] rxafe settle
//  *(volatile uint32_t *) 0x400300a4 = 0x140;      //[6] for tpm_en
////
//    set_max_length(0xff);
//    ll_hw_set_empty_head(0x0001);
//    ll_hw_set_rx_timeout_1st(   500);
//    ll_hw_set_rx_timeout(        88);       //ZQ 20180606, reduce rx timeout for power saving
//                                            //preamble + syncword=40us, sync process = 8us
//                                            //timeout should be larger then 48us,
////
//    ll_hw_set_loop_timeout(   30000);
////
//ll_hw_set_timing(pktFmt);
//ll_hw_ign_rfifo(LL_HW_IGN_SSN | LL_HW_IGN_CRC | LL_HW_IGN_EMP);
//     ll_hw_tx2rx_timing_config(g_rfPhyPktFmt);
//
//    rf_phy_ini();
// =========================
    rf_phy_change_cfg0(pktFmt);
    ll_hw_tx2rx_timing_config(pktFmt);
    set_crc_seed(ADV_CRC_INIT_VALUE);     // crc seed for adv is same for all channels
    set_access_address(ADV_SYNCH_WORD);   // access address
    set_channel(ch_idx);             // channel
    set_whiten_seed(ch_idx);         // whiten seed
    set_max_length(50);            // rx PDU max length
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();

    // for AUX_ADV_IND, connectable/scannable case, should configure TRX
    if ((auxPduIndFlag == TRUE)                      &&                     // AUX_ADV_IND
            ((pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_CONN_BITMASK) ||
             (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK)))
    {
        ll_hw_trx_settle_config(g_rfPhyPktFmt);
        ll_hw_set_trx();                      // set LL HW as Tx - Rx mode

        if (g_rfPhyPktFmt ==  PKT_FMT_BLE1M || g_rfPhyPktFmt ==  PKT_FMT_BLE2M)     // 1M/2M PHY
            ll_hw_set_rx_timeout(500);//(300);
        else                   // coded PHY
            ll_hw_set_rx_timeout(3000);

//  ll_hw_set_rx_timeout_1st(500);
//        debug_probe();
    }
    else
    {
        ll_hw_set_stx();                      // set LL HW as Tx - Rx mode
    }

    //ll_hw_ign_rfifo(LL_HW_IGN_ALL);         //set the rfifo ign control
    ll_hw_ign_rfifo(LL_HW_IGN_EMP | LL_HW_IGN_CRC);
    //write Tx FIFO
    ll_hw_write_tfifo((uint8*)&(g_tx_ext_adv_buf.txheader), ((g_tx_ext_adv_buf.txheader & 0xff00) >> 8) + 2);
//LOG("%d-%d ",ch_idx, pktFmt);
    T2 = read_current_fine_time();
    delta = LL_TIME_DELTA(T1, T2);
    temp = ( pGlobal_config[LL_EXT_ADV_PROCESS_TARGET] > delta) ? (pGlobal_config[LL_EXT_ADV_PROCESS_TARGET] - delta) : 0;
    llWaitUs(temp);             // insert delay to make process time equal PROCESS_TARGET
    ll_hw_go();
    llWaitingIrq = TRUE;
    llTaskState = LL_TASK_EXTENDED_ADV;
    HAL_EXIT_CRITICAL_SECTION();
//LOG("%d-%d",pAdvInfo->currentChn, pktFmt);
//LOG("%d ", pktFmt);
//    hal_gpio_write(GPIO_P14, 0);
//LOG("<%d>", delta);
    return TRUE;
}
/*******************************************************************************
    @fn          move_to_slave_function0

    @brief       This function is used to process CONN_REQ and move the llState to slave


    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None,

*/
void move_to_slave_function3(void)
{
    llConnState_t* connPtr;
    uint8_t*       pBuf;
    uint8_t       tempByte;
    uint8_t       chnSel;
    uint32_t      calibra_time, T2;

    if ( (connPtr = llAllocConnId()) == NULL )
    {
        return;
    }

    adv_param.connId = connPtr->connId;
    chnSel   = (g_rx_adv_buf.rxheader & CHSEL_MASK) >> CHSEL_SHIFT;

    if (pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
        connPtr->channel_selection = chnSel;
    else
        connPtr->channel_selection = LL_CHN_SEL_ALGORITHM_1;

    pBuf = g_rx_adv_buf.data;
    // reset connection parameters
    LL_set_default_conn_params(connPtr);
    // clear the connection buffer
    reset_conn_buf(connPtr->connId);

    // switch off the adv, will be switch on after link terminate by GAP
    if (llTaskState == LL_TASK_EXTENDED_ADV)
    {
        // TODO: ext advertiser process
    }
    else
        adv_param.advMode = LL_ADV_MODE_OFF;

    pBuf += 12;      // skip initA and AdvA
    pBuf = llMemCopySrc( (uint8*)&connPtr->accessAddr,             pBuf, 4 );
    pBuf = llMemCopySrc( (uint8*)&connPtr->initCRC,                pBuf, 3 );
    pBuf = llMemCopySrc( (uint8*)&connPtr->curParam.winSize,       pBuf, 1 );
    pBuf = llMemCopySrc( (uint8*)&connPtr->curParam.winOffset,     pBuf, 2 );
    pBuf = llMemCopySrc( (uint8*)&connPtr->curParam.connInterval,  pBuf, 2 );
    pBuf = llMemCopySrc( (uint8*)&connPtr->curParam.slaveLatency,  pBuf, 2 );
    pBuf = llMemCopySrc( (uint8*)&connPtr->curParam.connTimeout,   pBuf, 2 );
    // TI style: convert to 625us tick
    connPtr->curParam.winSize      <<= 1;
    connPtr->curParam.winOffset    <<= 1;
    connPtr->curParam.connInterval <<= 1;
    connPtr->curParam.connTimeout  <<= 4;
    llConvertLstoToEvent( connPtr, &(connPtr->curParam) );     // 16MHz CLK, need 56.5us
    // bug fixed 2018-4-4, calculate control procedure timeout value when connection setup
    // convert the Control Procedure timeout into connection event count
    llConvertCtrlProcTimeoutToEvent(connPtr);

    if (((connPtr->curParam.connTimeout <= ((connPtr->curParam.slaveLatency ) * connPtr->curParam.connInterval << 1)))
            || (connPtr->curParam.connInterval == 0) )
    {
        // schedule LL Event to notify the Host a connection was formed with
        // a bad parameter
        // Note: This event doesn't take parameters, so it is assumed there that
        //       the reason code was due to an unacceptable connection interval.
        (void)osal_set_event( LL_TaskID, LL_EVT_SLAVE_CONN_CREATED_BAD_PARAM );
        return;
    }

    pBuf = llMemCopySrc( (uint8*)connPtr->chanMap,   pBuf, 5 );
    pBuf = llMemCopySrc( &tempByte,   pBuf, 1 );
    connPtr->hop  = tempByte & 0x1F;
    connPtr->sleepClkAccuracy  = (tempByte >> 5) & 0x07;
    // calculate channel for data
    llProcessChanMap(connPtr, connPtr->chanMap );   // 16MHz clk, cost 116us!
    connPtr->slaveLatency = 0;        //correct 05-09, no latency before connect
    connPtr->slaveLatencyValue = connPtr->curParam.slaveLatency;
    connPtr->accuTimerDrift = 0;
    llAdjSlaveLatencyValue(connPtr);
    // combine slave SCA with master's SCA and calculate timer drift factor
    //connPtr->scaFactor = llCalcScaFactor( connPtr->sleepClkAccuracy );
    //connPtr->currentChan = llGetNextDataChan(1);
    // add 2020-7-25
    connPtr->llRfPhyPktFmt = g_rfPhyPktFmt;
    llState = LL_STATE_CONN_SLAVE;
    ll_debug_output(DEBUG_LL_STATE_CONN_SLAVE);
    connPtr->active = TRUE;
    connPtr->sn_nesn = 0;                  // 1st rtlp, init sn/nesn as 0
    connPtr->llMode = LL_HW_RTLP_1ST;     // set as RTLP_1ST for the 1st connection event
    // calculate the 1st channel
    connPtr->currentChan = 0;

    if (connPtr->channel_selection == LL_CHN_SEL_ALGORITHM_1)
        connPtr->currentChan = llGetNextDataChan(connPtr, 1);
    else
    {
        // channel selection algorithm 2
        connPtr->currentChan = llGetNextDataChanCSA2(0,
                                                     (( connPtr->accessAddr & 0xFFFF0000 )>> 16 ) ^ ( connPtr->accessAddr  & 0x0000FFFF),
                                                     connPtr->chanMap,
                                                     connPtr->chanMapTable,
                                                     connPtr->numUsedChans);
    }

    // calculate timer drift
    llCalcTimerDrift(connPtr->curParam.winOffset + 2,        // 1250us + win offset, in 625us tick
                     connPtr->slaveLatency,
                     connPtr->sleepClkAccuracy,
                     (uint32*)&(connPtr->timerDrift));
    T2 = read_current_fine_time();
    // calculate the SW delay from ISR to here
    calibra_time = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
    // other delay: conn req tail -> ISR: 32us, timing advance: 50us, HW engine startup: 60us
    // start slave event SW process time: 50us
    // soft parameter: pGlobal_config[CONN_REQ_TO_SLAVE_DELAY]
    calibra_time += pGlobal_config[CONN_REQ_TO_SLAVE_DELAY];     //(32 + 50 + 60 + 50 + pGlobal_config[CONN_REQ_TO_SLAVE_DELAY]);
    // TODO: need consider the case: 0ms window offset, sometimes timer offset will < 0,
//    timer1 = 1250 + conn_param[connId].curParam.winOffset * 625 - calibra_time - conn_param[connId].timerDrift;
//    if (timer1 < 0)
//        while(1);
//
//    ll_schedule_next_event(timer1);
    uint32_t  temp, tranWinDelay = 1250;

    if (g_llAdvMode == LL_MODE_EXTENDED)
    {
        extAdvInfo_t*  pAdvInfo;
        pAdvInfo = g_pAdvSchInfo[g_currentExtAdv].pAdvInfo;

        if (pAdvInfo != NULL && !ll_isLegacyAdv(pAdvInfo))
        {
            tranWinDelay = 2500;              // using AUX_CONN_REQ, delay shoule be 2.5ms

            if (connPtr->llRfPhyPktFmt == PKT_FMT_BLR500K || connPtr->llRfPhyPktFmt == PKT_FMT_BLR125K)
                tranWinDelay = 3750;

            //
            calibra_time = (T2 > g_auxconnreq_ISR_entry_time) ? (T2 - g_auxconnreq_ISR_entry_time) : (BASE_TIME_UNITS - g_auxconnreq_ISR_entry_time + T2);
            calibra_time += pGlobal_config[CONN_REQ_TO_SLAVE_DELAY];
        }
    }

    temp = tranWinDelay + connPtr->curParam.winOffset * 625 - calibra_time - connPtr->timerDrift;

    if (g_ll_conn_ctx.numLLConns == 1)     // 1st connection, time1 is for adv event
        clear_timer(AP_TIM1);                // stop the timer between different adv channel

    ll_addTask(connPtr->connId, temp  );
//    ll_addTask(connPtr->connId, 1250 + connPtr->curParam.winOffset * 625 - calibra_time - connPtr->timerDrift);
    g_ll_conn_ctx.scheduleInfo[connPtr->connId].task_duration = 3000;     // slave task duration: 150 + 80 + 150 + 2120 + window

    // current link id may be updated in ll_addTask, update the ll state
    if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_MASTER)
        llState = LL_STATE_CONN_MASTER;
    else if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_SLAVE)
        llState = LL_STATE_CONN_SLAVE;

    llSecondaryState = LL_SEC_STATE_IDLE;
    g_ll_conn_ctx.scheduleInfo[connPtr->connId].linkRole = LL_ROLE_SLAVE;
    (void)osal_set_event( LL_TaskID, LL_EVT_SLAVE_CONN_CREATED);
    g_pmCounters.ll_conn_succ_cnt ++;     // move to anchor point catch ?
}

uint8 ll_processExtAdvIRQ1(uint32_t      irq_status)
{
    // gpio_write(P32,1);
    // gpio_write(P32,0);
    uint8         mode;
    extAdvInfo_t*  pAdvInfo;
    uint32_t      T2, delay;
    pAdvInfo = g_pAdvSchInfo[g_currentExtAdv].pAdvInfo;

    if (pAdvInfo == NULL)
        return FALSE;

//    gpio_write(GPIO_P34,1);
//    gpio_write(GPIO_P34,0);
    HAL_ENTER_CRITICAL_SECTION();
    mode = ll_hw_get_tr_mode();

    if (ll_isLegacyAdv(pAdvInfo))
    {
        // process legacy Adv
        uint8_t  packet_len, pdu_type, txAdd;
        uint8_t*  peerAddr, *ownAddr;
        uint8_t  bWlRlCheckOk = TRUE;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        int      calibra_time;                 // this parameter will be provided by global_config
        ll_debug_output(DEBUG_LL_HW_TRX);
        // read packet
        packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                       &pktLen,
                                       &pktFoot0,
                                       &pktFoot1);

        if(ll_hw_get_rfifo_depth()>0)
        {
            g_pmCounters.ll_rfifo_read_err++;
            packet_len=0;
            pktLen=0;
        }

        // check receive pdu type
        pdu_type = g_rx_adv_buf.rxheader & PDU_TYPE_MASK;
        txAdd    = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random

        if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)
            ownAddr = pAdvInfo->parameter.ownRandomAddress;
        else
            ownAddr = ownPublicAddr;

        if (packet_len > 0                       // any better checking rule for rx anything?
                && pdu_type == ADV_SCAN_REQ
                && (pAdvInfo->parameter.advEventProperties == LL_EXT_ADV_PROP_ADV_IND
                    || pAdvInfo->parameter.advEventProperties == LL_EXT_ADV_PROP_ADV_SCAN_IND))
        {
            // 1. scan req
            g_pmCounters.ll_recv_scan_req_cnt ++;

            // check AdvA
            if (g_rx_adv_buf.data[6]  != ownAddr[0]
                    || g_rx_adv_buf.data[7]  != ownAddr[1]
                    || g_rx_adv_buf.data[8]  != ownAddr[2]
                    || g_rx_adv_buf.data[9]  != ownAddr[3]
                    || g_rx_adv_buf.data[10] != ownAddr[4]
                    || g_rx_adv_buf.data[11] != ownAddr[5])
            {
            }
            else
            {
                uint8_t  rpaListIndex;
                uint8_t  peerType = txAdd;
                peerAddr = &g_rx_adv_buf.data[0];      // ScanA

                // Resolving list checking
                if (g_llRlEnable == TRUE          &&
                        txAdd == LL_DEV_ADDR_TYPE_RANDOM &&
                        (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                {
                    rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                    {
                        peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                        peerType = g_llResolvinglist[rpaListIndex].peerAddrType;
                    }
                    else
                        bWlRlCheckOk = FALSE;
                }

                // check white list
                if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                        && (pAdvInfo->parameter.wlPolicy  == LL_ADV_WL_POLICY_WL_SCAN_REQ
                            || pAdvInfo->parameter.wlPolicy  == LL_ADV_WL_POLICY_WL_ALL_REQ)
                        && (bWlRlCheckOk == TRUE))
                {
                    // check white list
                    bWlRlCheckOk = ll_isAddrInWhiteList(peerType, peerAddr);
                }

                if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
                {
                    g_pmCounters.ll_filter_scan_req_cnt ++;
                }
                else
                {
                    g_pmCounters.ll_rx_peer_cnt++;
                    uint8 retScanRspFilter = 1;

                    if(LL_PLUS_ScanRequestFilterCBack)
                    {
                        retScanRspFilter = 1;//LL_PLUS_ScanRequestFilterCBack();
                    }

                    if(retScanRspFilter)
                    {
                        // send scan rsp
                        ll_hw_set_stx();             // set LL HW as single Tx mode
                        g_same_rf_channel_flag = TRUE;
                        // calculate the delay
                        T2 = read_current_fine_time();
                        delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                        calibra_time = pGlobal_config[SCAN_RSP_DELAY];            // consider rx_done to ISR time, SW delay after read_current_fine_time(), func read_current_fine_time() delay ...
                        delay = 118 - delay - calibra_time;                       // IFS = 150us, Tx tail -> Rx done time: about 32us
                        ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
                                             pGlobal_config[LL_HW_AFE_DELAY],
                                             pGlobal_config[LL_HW_PLL_DELAY]);        //RxAFE,PLL
                        ll_hw_go();
                        llWaitingIrq = TRUE;
                        g_same_rf_channel_flag = FALSE;
                        // reset Rx/Tx FIFO
                        ll_hw_rst_rfifo();
                        ll_hw_rst_tfifo();
                        //write Tx FIFO
                        ll_hw_write_tfifo((uint8*)&(tx_scanRsp_desc.txheader),
                                          ((tx_scanRsp_desc.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                        ll_debug_output(DEBUG_LL_HW_SET_STX);
                        g_pmCounters.ll_send_scan_rsp_cnt ++;
                    }
                }
            }
        }
        else if (pdu_type == ADV_CONN_REQ
                 && (pAdvInfo->parameter.advEventProperties == LL_EXT_ADV_PROP_ADV_IND
                     || pAdvInfo->parameter.advEventProperties == LL_EXT_ADV_PROP_ADV_LDC_ADV
                     || pAdvInfo->parameter.advEventProperties == LL_EXT_ADV_PROP_ADV_HDC_ADV))
        {
            // 2. connect req
            g_pmCounters.ll_recv_conn_req_cnt ++;

            // check AdvA
            if (g_rx_adv_buf.data[6]  != ownAddr[0]
                    || g_rx_adv_buf.data[7]  != ownAddr[1]
                    || g_rx_adv_buf.data[8]  != ownAddr[2]
                    || g_rx_adv_buf.data[9]  != ownAddr[3]
                    || g_rx_adv_buf.data[10] != ownAddr[4]
                    || g_rx_adv_buf.data[11] != ownAddr[5])
            {
                // nothing to do
            }
            else
            {
                uint8_t  rpaListIndex;
                uint8_t  peerType = txAdd;
                peerAddr = &g_rx_adv_buf.data[0];        // initA

                // Resolving list checking
                if (g_llRlEnable == TRUE             &&
                        txAdd == LL_DEV_ADDR_TYPE_RANDOM   &&
                        (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                {
                    rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                    {
                        peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                        peerType = g_llResolvinglist[rpaListIndex].peerAddrType;
                    }
                    else
                        bWlRlCheckOk = FALSE;
                }

                // check white list
                if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                        && (llState == LL_STATE_ADV_UNDIRECTED)
                        && (pAdvInfo->parameter.wlPolicy   == LL_ADV_WL_POLICY_WL_CONNECT_REQ
                            || pAdvInfo->parameter.wlPolicy  == LL_ADV_WL_POLICY_WL_ALL_REQ)
                        && (bWlRlCheckOk == TRUE))
                {
                    // check white list
                    bWlRlCheckOk = ll_isAddrInWhiteList(peerType, peerAddr);
                }

                // fixed bug 2018-09-25, LL/CON/ADV/BV-04-C, for direct adv, initA should equal peer Addr
                if (llState == LL_STATE_ADV_DIRECTED)
                {
                    if (txAdd         != peerInfo.peerAddrType
                            || peerAddr[0]  != peerInfo.peerAddr[0]
                            || peerAddr[1]  != peerInfo.peerAddr[1]
                            || peerAddr[2]  != peerInfo.peerAddr[2]
                            || peerAddr[3]  != peerInfo.peerAddr[3]
                            || peerAddr[4]  != peerInfo.peerAddr[4]
                            || peerAddr[5]  != peerInfo.peerAddr[5])
                    {
                        // not match, check next
                        bWlRlCheckOk = FALSE;
                    }
                }

                if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
                {
                    g_pmCounters.ll_filter_conn_req_cnt ++;
                }
                else
                {
                    // increment statistics counter
                    g_pmCounters.ll_rx_peer_cnt++;
                    // bug fixed 2018-01-23, peerAddrType should read TxAdd
                    peerInfo.peerAddrType = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                    osal_memcpy( peerInfo.peerAddr, g_rx_adv_buf.data, 6);
                    move_to_slave_function();    // move to slave role for connection state
                    // add 04-01, set adv inactive, and it will be remove from the scheduler when invoke ll_adv_scheduler()
                    pAdvInfo->active = FALSE;
                    LL_AdvSetTerminatedCback(LL_STATUS_SUCCESS,
                                             pAdvInfo->advHandle,
                                             adv_param.connId,
                                             pAdvInfo->adv_event_counter);
                }
            }
        }
    }
    else if (mode == LL_HW_MODE_TRX  &&
             (irq_status & LIRQ_COK))
    {
        // TRX mode, receives AUX_SCAN_REQ or AUX_CONNECT_REQ
        uint8_t  packet_len, pdu_type, txAdd;
        uint8_t*  peerAddr;
        uint8_t  bWlRlCheckOk = TRUE;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        int      calibra_time;                 // this parameter will be provided by global_config
        uint8*    ownAddr;
        ll_debug_output(DEBUG_LL_HW_TRX);
        // read packet
        packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                       &pktLen,
                                       &pktFoot0,
                                       &pktFoot1);

        if(ll_hw_get_rfifo_depth() > 0)
        {
            g_pmCounters.ll_rfifo_read_err++;
            packet_len=0;
            pktLen=0;
        }

        // check receive pdu type
        pdu_type = g_rx_adv_buf.rxheader & PDU_TYPE_MASK;
        txAdd    = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random

        if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)
            ownAddr = pAdvInfo->parameter.ownRandomAddress;
        else if (g_currentLocalAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
            ownAddr = g_currentLocalRpa;
        else
            ownAddr = ownPublicAddr;

        if (packet_len > 0
                && pdu_type == ADV_AUX_SCAN_REQ
                && (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK))
        {
            // 1. scan req
            g_pmCounters.ll_recv_scan_req_cnt ++;

            // check AdvA
            if (g_rx_adv_buf.data[6]  != ownAddr[0]
                    || g_rx_adv_buf.data[7]  != ownAddr[1]
                    || g_rx_adv_buf.data[8]  != ownAddr[2]
                    || g_rx_adv_buf.data[9]  != ownAddr[3]
                    || g_rx_adv_buf.data[10] != ownAddr[4]
                    || g_rx_adv_buf.data[11] != ownAddr[5])
            {
            }
            else
            {
                uint8_t  rpaListIndex;
                uint8_t  peerType = txAdd;
                peerAddr = &g_rx_adv_buf.data[0];      // ScanA

                // Resolving list checking
                if (g_llRlEnable == TRUE          &&
                        txAdd == LL_DEV_ADDR_TYPE_RANDOM &&
                        (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                {
                    rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                    {
                        peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                        peerType = g_llResolvinglist[rpaListIndex].peerAddrType;
                    }
                    else
                        bWlRlCheckOk = FALSE;
                }

                // check white list
                if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                        && (pAdvInfo->parameter.wlPolicy  == LL_ADV_WL_POLICY_WL_SCAN_REQ
                            || pAdvInfo->parameter.wlPolicy  == LL_ADV_WL_POLICY_WL_ALL_REQ)
                        && (bWlRlCheckOk == TRUE))
                {
                    // check white list
                    bWlRlCheckOk = ll_isAddrInWhiteList(peerType, peerAddr);
                }

                if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
                {
                    g_pmCounters.ll_filter_scan_req_cnt ++;
                }
                else
                {
                    g_pmCounters.ll_rx_peer_cnt++;
                    g_tx_ext_adv_buf.txheader = 0;
//                    // BQB check RxAdd but it seems not required by spec
//                    SET_BITS(g_tx_ext_adv_buf.txheader, txAdd, RX_ADD_SHIFT, RX_ADD_MASK);
                    g_timerExpiryTick = read_current_fine_time();
                    llSetupAuxScanRspPDU(pAdvInfo);
                    // send scan rsp
                    ll_hw_set_stx();             // set LL HW as single Tx mode
                    g_same_rf_channel_flag = TRUE;
                    // calculate the delay
                    T2 = read_current_fine_time();
                    delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);

                    //calibra_time = pGlobal_config[SCAN_RSP_DELAY];            // consider rx_done to ISR time, SW delay after read_current_fine_time(), func read_current_fine_time() delay ...
                    if(g_rfPhyPktFmt == PKT_FMT_BLE1M)
                    {
                        calibra_time = pGlobal_config[EXT_ADV_AUXSCANRSP_DELAY_1MPHY];
                        delay = 118 - delay - calibra_time;
                    }
                    else if(g_rfPhyPktFmt == PKT_FMT_BLE2M)
                    {
                        calibra_time = pGlobal_config[EXT_ADV_AUXSCANRSP_DELAY_2MPHY];
                        delay = 118+14 - delay - calibra_time;
                    }
                    else
                    {
                        calibra_time = pGlobal_config[EXT_ADV_AUXSCANRSP_DELAY_125KPHY];
                        delay = 118 - delay - calibra_time;
                    }

                    //delay = 118 - delay - calibra_time;                       // IFS = 150us, Tx tail -> Rx done time: about 32us
                    //LOG("delay:%d\n",delay);
//                        ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
//                                         pGlobal_config[LL_HW_AFE_DELAY],
//                                         pGlobal_config[LL_HW_PLL_DELAY]);          //RxAFE,PLL
//                    // temporary fixed for BQB test
//                    if (g_rfPhyPktFmt == PKT_FMT_BLE2M)
//                        delay += 15;       // 2Mbps
//                    else if (g_rfPhyPktFmt == PKT_FMT_BLR125K)
//                        delay = 0;   // coded PHY
                    //LOG("%d\n",delay);
                    ll_hw_trx_settle_bb(g_rfPhyPktFmt, delay);
                    ll_hw_go();
                    llWaitingIrq = TRUE;
                    g_same_rf_channel_flag = FALSE;
                    // reset Rx/Tx FIFO
                    ll_hw_rst_rfifo();
                    ll_hw_rst_tfifo();
                    //write Tx FIFO
                    // =================== TODO: change the buffer to ext adv set
                    ll_hw_write_tfifo((uint8*)&(g_tx_ext_adv_buf.txheader),
                                      ((g_tx_ext_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                    ll_debug_output(DEBUG_LL_HW_SET_STX);
                    g_pmCounters.ll_send_scan_rsp_cnt ++;
                    //LOG("RSP");
                }
            }
        }
        else if (pdu_type == ADV_AUX_CONN_REQ
                 && (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_CONN_BITMASK))
        {
            g_auxconnreq_ISR_entry_time = ISR_entry_time;
            // 2. connect req
            g_pmCounters.ll_recv_conn_req_cnt ++;

            // check AdvA
            if (g_rx_adv_buf.data[6]  != ownAddr[0]
                    || g_rx_adv_buf.data[7]  != ownAddr[1]
                    || g_rx_adv_buf.data[8]  != ownAddr[2]
                    || g_rx_adv_buf.data[9]  != ownAddr[3]
                    || g_rx_adv_buf.data[10] != ownAddr[4]
                    || g_rx_adv_buf.data[11] != ownAddr[5])
            {
                // nothing to do
            }
            else
            {
                uint8_t  rpaListIndex;
                uint8_t  peerType = txAdd;
                peerAddr = &g_rx_adv_buf.data[0];        // initA

//================= cheater for BQB test
                // if (g_rfPhyPktFmt == PKT_FMT_BLR125K)
                // {
                //     g_same_rf_channel_flag = TRUE;
                //  ll_hw_set_stx();
                //     ll_hw_go();
                //  llWaitingIrq = TRUE;

                //     ll_hw_set_trx_settle(10,//30,//delay,                             // set BB delay, about 80us in 16MHz HCLK
                //                      pGlobal_config[LL_HW_AFE_DELAY_125KPHY],
                //                      pGlobal_config[LL_HW_PLL_DELAY_125KPHY]);         //RxAFE,PLL

                // }
// =====================

                // Resolving list checking
                if (g_llRlEnable == TRUE             &&
                        txAdd == LL_DEV_ADDR_TYPE_RANDOM   &&
                        (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                {
                    rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                    {
                        peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                        peerType = g_llResolvinglist[rpaListIndex].peerAddrType;
                    }
                    else
                        bWlRlCheckOk = FALSE;
                }

                // check white list
                if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
//                    && (llState == LL_STATE_ADV_UNDIRECTED)                       // 11-18
                        && (pAdvInfo->parameter.wlPolicy   == LL_ADV_WL_POLICY_WL_CONNECT_REQ
                            || pAdvInfo->parameter.wlPolicy  == LL_ADV_WL_POLICY_WL_ALL_REQ)
                        && (bWlRlCheckOk == TRUE))
                {
                    // check white list
                    bWlRlCheckOk = ll_isAddrInWhiteList(peerType, peerAddr);
                }

                if (bWlRlCheckOk == FALSE)   // if not in white list, do nothing
                {
                    g_pmCounters.ll_filter_conn_req_cnt ++;
                    //LOG("- ");
                }
                else
                {
                    // increment statistics counter
                    g_pmCounters.ll_rx_peer_cnt++;
//==============
                    llSetupAuxConnectRspPDU0(pAdvInfo);

                    if (!llWaitingIrq)
                    {
                        // send scan rsp
                        ll_hw_set_stx();             // set LL HW as single Tx mode
                        g_same_rf_channel_flag = TRUE;
                        // calculate the delay
                        T2 = read_current_fine_time();
                        delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);

                        //calibra_time = pGlobal_config[SCAN_RSP_DELAY];            // consider rx_done to ISR time, SW delay after read_current_fine_time(), func read_current_fine_time() delay ...
                        if(g_rfPhyPktFmt == PKT_FMT_BLE1M)
                        {
                            calibra_time = pGlobal_config[EXT_ADV_AUXCONNRSP_DELAY_1MPHY];
                            delay = 118 - delay - calibra_time;
                        }
                        else if(g_rfPhyPktFmt == PKT_FMT_BLE2M)
                        {
                            calibra_time = pGlobal_config[EXT_ADV_AUXCONNRSP_DELAY_2MPHY];
                            delay = 118+14 - delay - calibra_time;
                        }
                        else
                        {
                            calibra_time = pGlobal_config[EXT_ADV_AUXCONNRSP_DELAY_125KPHY];
                            delay = 118 - delay - calibra_time;
                        }

                        //delay = 118 - delay - calibra_time;                       // IFS = 150us, Tx tail -> Rx done time: about 32us
//                        ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
//                                         pGlobal_config[LL_HW_AFE_DELAY],
//                                         pGlobal_config[LL_HW_PLL_DELAY]);          //RxAFE,PLL
//                        // temporary fixed for BQB test
//                        if (g_rfPhyPktFmt == PKT_FMT_BLE2M)
//                            delay += 15;       // 2Mbps
//                        else if (g_rfPhyPktFmt == PKT_FMT_BLR125K)
//                            delay = 0;   // coded PHY
                        ll_hw_trx_settle_bb(g_rfPhyPktFmt, delay);
                        ll_hw_go();
//                    hal_gpio_write(GPIO_P14, 1);
//                    hal_gpio_write(GPIO_P14, 0);
                        llWaitingIrq = TRUE;
                    }

                    g_same_rf_channel_flag = FALSE;
//      hal_gpio_write(GPIO_P15, 1);
//        hal_gpio_write(GPIO_P15, 0);
                    // reset Rx/Tx FIFO
                    ll_hw_rst_rfifo();
                    ll_hw_rst_tfifo();
                    //write Tx FIFO
                    ll_hw_write_tfifo((uint8*)&(g_tx_adv_buf.txheader),
                                      ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                    ll_debug_output(DEBUG_LL_HW_SET_STX);
                    g_pmCounters.ll_send_conn_rsp_cnt ++;
//==============
                    // bug fixed 2018-01-23, peerAddrType should read TxAdd
                    peerInfo.peerAddrType = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                    osal_memcpy( peerInfo.peerAddr, g_rx_adv_buf.data, 6);
                    // 2021-1-7, walkaround to force extenede adv connection using CSA2. To be modified next ROM revision
                    SET_BITS(g_rx_adv_buf.rxheader, 1, CHSEL_SHIFT, CHSEL_MASK);
                    move_to_slave_function3();    // move to slave role for connection state
                    // add 04-01, set adv inactive, and it will be remove from the scheduler when invoke ll_adv_scheduler()
                    pAdvInfo->active = FALSE;
//                    LOG("<%d>", delay);
                    LL_AdvSetTerminatedCback(LL_STATUS_SUCCESS,
                                             pAdvInfo->advHandle,
                                             adv_param.connId,
                                             pAdvInfo->adv_event_counter);
                }
            }
        }
    }
//    else if (mode == LL_HW_MODE_TRX)
//    {
//        LOG("%x ", irq_status);
//    }
    else if (mode == LL_HW_MODE_STX )
    {
        //LOG("COK:%d\n",irq_status & LIRQ_COK);
    }

//  // update scheduler list
//  ll_adv_scheduler();

    if (!llWaitingIrq)
    {
        // update scheduler list  // update 04-01, consider sending aux_scan_rsp/aux_conn_rsp case, will invoke scheduler after STX IRQ
        ll_adv_scheduler();
        ll_hw_clr_irq();
        llTaskState = LL_TASK_OTHERS;
    }

    HAL_EXIT_CRITICAL_SECTION();
    return TRUE;
}

static uint8   scanningScanInd = FALSE;
uint8  tempAdvA[6];
uint8  tempTargetA[6];
static uint8 scanningpeer_addrtype;
static uint8 scanningpeer_addr[6];
static uint8 scanningauxchain=FALSE;
uint8 ll_processExtScanIRQ1(uint32_t      irq_status)
{
    //LOG("1\n");
    uint8         ll_mode, adv_mode, ext_hdr_len;
    uint32_t      T2, delay;
    HAL_ENTER_CRITICAL_SECTION();
    ll_mode = ll_hw_get_tr_mode();

    if (ll_mode == LL_HW_MODE_SRX)      // passive scan
    {
        uint8_t  rpaListIndex = LL_RESOLVINGLIST_ENTRY_NUM;
        uint8_t  bWlRlCheckOk = TRUE;
        uint8_t*  peerAddr = NULL;//&ext_adv_hdr.advA[0];
        uint8   peerAddrType;                  // peer address type
        uint8_t  packet_len, pdu_type;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        ll_debug_output(DEBUG_LL_HW_SRX);

        // ============= scan case
        if (llTaskState == LL_TASK_EXTENDED_SCAN)
        {
            uint8   bSendingScanReq = FALSE;
            memset(&ext_adv_hdr, 0, sizeof(ext_adv_hdr));

            // check status
            if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))
            {
                // rx done
                uint8    isTargetAChecked = FALSE;     // flag for whether the targetA RPA has been resolved in last event
                uint8_t  pktErrFlg = 0;
                // read packet
                packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                               &pktLen,
                                               &pktFoot0,
                                               &pktFoot1);
                // check receive pdu type
                pdu_type = g_rx_adv_buf.rxheader & 0x0f;

                if(ll_hw_get_rfifo_depth() > 0)
                {
                    g_pmCounters.ll_rfifo_read_err++;
                    packet_len = 0;
                    pktLen = 0;
                    pktErrFlg = 1;
                }

                // case 1: extended adv type
                if (pktErrFlg== 0                       //if (packet_len   != 0
                        && (pdu_type == ADV_EXT_TYPE))
                {
                    uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                    uint8   payload_len = (g_rx_adv_buf.rxheader & 0xFF00) >> LENGTH_SHIFT;
                    uint8   adv_data_len;
                    peerAddrType = txAdd;
                    adv_mode    = (g_rx_adv_buf.data[0] & 0xc0) >> 6;
                    ext_hdr_len =  g_rx_adv_buf.data[0] & 0x3f;
                    ll_parseExtHeader(&g_rx_adv_buf.data[1], ext_hdr_len);//payload_len - 1);   // update length 20201012

                    if (ext_adv_hdr.header & LE_EXT_HDR_ADVA_PRESENT_BITMASK)
                    {
                        memcpy(&tempAdvA[0], &ext_adv_hdr.advA[0], 6);
                        peerAddr = &ext_adv_hdr.advA[0];
                    }

                    // Resolving list checking
                    if (g_llRlEnable == TRUE
                            && (adv_mode == LL_EXT_ADV_MODE_SC )   // BQB test only required checking RL for scanable ADV
                            && !(ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)      // no more aux PDU
                            && (ext_adv_hdr.header & LE_EXT_HDR_ADVA_PRESENT_BITMASK))         // AdvA field present
                    {
                        // if ScanA is resolvable private address
                        if ((ext_adv_hdr.advA[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                        {
                            bWlRlCheckOk = FALSE;

                            // if the advA has been checked, skip RPA list check
                            if  (isPeerRpaStore == TRUE
                                    && currentPeerRpa[0] == ext_adv_hdr.advA[0]
                                    && currentPeerRpa[1] == ext_adv_hdr.advA[1]
                                    && currentPeerRpa[2] == ext_adv_hdr.advA[2]
                                    && currentPeerRpa[3] == ext_adv_hdr.advA[3]
                                    && currentPeerRpa[4] == ext_adv_hdr.advA[4]
                                    && currentPeerRpa[5] == ext_adv_hdr.advA[5])
                            {
                                rpaListIndex = storeRpaListIndex;
                                isTargetAChecked = TRUE;
                            }
                            else
                                rpaListIndex = ll_getRPAListEntry(&ext_adv_hdr.advA[0]);

                            if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                            {
                                peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                                peerAddrType = g_llResolvinglist[rpaListIndex].peerAddrType;
                                bWlRlCheckOk = TRUE;
                            }
                        }
                        else   // ScanA is device Identity, if the device ID in the RPA list, check whether RPA should be used
                        {
                            bWlRlCheckOk = TRUE;

                            for (int i = 0; i < LL_RESOLVINGLIST_ENTRY_NUM; i++)
                            {
                                if (ext_adv_hdr.advA[0] == g_llResolvinglist[i].peerAddr[0]
                                        && ext_adv_hdr.advA[1] == g_llResolvinglist[i].peerAddr[1]
                                        && ext_adv_hdr.advA[2] == g_llResolvinglist[i].peerAddr[2]
                                        && ext_adv_hdr.advA[3] == g_llResolvinglist[i].peerAddr[3]
                                        && ext_adv_hdr.advA[4] == g_llResolvinglist[i].peerAddr[4]
                                        && ext_adv_hdr.advA[5] == g_llResolvinglist[i].peerAddr[5])
                                {
                                    if (g_llResolvinglist[i].privacyMode == NETWORK_PRIVACY_MODE &&
                                            !ll_isIrkAllZero(g_llResolvinglist[i].peerIrk))
                                        bWlRlCheckOk = FALSE;
                                    else
                                        rpaListIndex = i;

                                    break;
                                }
                            }
                        }
                    }

                    // check white list
                    if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                            && (extScanInfo.wlPolicy  == LL_SCAN_WL_POLICY_USE_WHITE_LIST)
                            && (bWlRlCheckOk == TRUE)
                            && isTargetAChecked == FALSE)
                    {
                        // check white list
                        bWlRlCheckOk = ll_isAddrInWhiteList(peerAddrType, peerAddr);
                    }

                    //save scanning peer
                    if(ext_adv_hdr.header & LE_EXT_HDR_ADVA_PRESENT_BITMASK)
                    {
                        scanningpeer_addrtype = peerAddrType;
                        osal_memcpy(&scanningpeer_addr[0],&peerAddr[0],6);
                    }

                    // 2020-11-27, for periodic adv create sync, if AdvA is present, check periodic adv list/peer addr
                    if ((scanSyncInfo.valid == TRUE)                               // createSync is ongoing
                            && (ext_adv_hdr.header & LE_EXT_HDR_ADVA_PRESENT_BITMASK)     // advA is present
                            && (bWlRlCheckOk == TRUE))
                    {
                        if (scanSyncInfo.options & LL_PERIODIC_ADV_CREATE_SYNC_USING_ADV_LIST_BITMASK)
                        {
                            // option1, check the periodic advertise list
                            bWlRlCheckOk = FALSE;

                            if (g_llPrdAdvDeviceNum == 0)
                                ;
                            else
                            {
                                for (int i = 0; i < LL_PRD_ADV_ENTRY_NUM; i++)
                                {
                                    if (peerAddrType         != g_llPeriodicAdvlist[i].addrType
                                            || ext_adv_hdr.advA[0]  != g_llPeriodicAdvlist[i].addr[0]
                                            || ext_adv_hdr.advA[1]  != g_llPeriodicAdvlist[i].addr[1]
                                            || ext_adv_hdr.advA[2]  != g_llPeriodicAdvlist[i].addr[2]
                                            || ext_adv_hdr.advA[3]  != g_llPeriodicAdvlist[i].addr[3]
                                            || ext_adv_hdr.advA[4]  != g_llPeriodicAdvlist[i].addr[4]
                                            || ext_adv_hdr.advA[5]  != g_llPeriodicAdvlist[i].addr[5]
                                            || ((ext_adv_hdr.adi >> 12) & 0xf) != g_llPeriodicAdvlist[i].sid)
                                    {
                                        // not match, check next
                                        continue;
                                    }
                                    else
                                    {
                                        // check OK
                                        bWlRlCheckOk = TRUE;
                                        break;
                                    }
                                }
                            }
                        }
                        else
                        {
                            // option 2, check addr + addr type + sid
//                          if (((ext_adv_hdr.adi >> 12) & 0x0F)  != scanSyncInfo.advertising_SID       // ADI is mandatory for periodic adv(ADV_EXT_IND, AUX_ADV_IND)
//                              || peerAddrType != scanSyncInfo.advertiser_Address_Type
//                              || memcmp(&peerAddr[0], &scanSyncInfo.advertiser_Address[0], LL_DEVICE_ADDR_LEN) != 0)
                            if (peerAddrType != scanSyncInfo.advertiser_Address_Type
                                    || memcmp(&peerAddr[0], &scanSyncInfo.advertiser_Address[0], LL_DEVICE_ADDR_LEN) != 0)
                            {
                                // find the periodic sync info, sync with the period adv
                                bWlRlCheckOk = FALSE;
                            }
                        }
                    }

                    if (ext_adv_hdr.header & LE_EXT_HDR_TARGETA_PRESENT_BITMASK
                            && (bWlRlCheckOk == TRUE))
                    {
                        // for direct adv, check targetA == own Addr
                        if (isTargetAChecked == TRUE)
                            bWlRlCheckOk = TRUE;
                        else if ((ext_adv_hdr.targetA[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                        {
                            if (rpaListIndex >= LL_RESOLVINGLIST_ENTRY_NUM
                                    || (ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk)    // all-0 local IRK
                                        || ll_ResolveRandomAddrs(g_llResolvinglist[rpaListIndex].localIrk, &ext_adv_hdr.targetA[0]) != SUCCESS))
                                bWlRlCheckOk = FALSE;
                        }
                        else
                        {
                            uint8* localAddr;
//                          uint8_t rxAdd = (g_rx_adv_buf.rxheader & RX_ADD_MASK) >> RX_ADD_SHIFT;

                            // should not use device ID case
                            if ((extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC || extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM )
                                    && (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM
                                        && !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk)))
                            {
                                bWlRlCheckOk = FALSE;
                            }

                            if (extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM || extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM)
                                localAddr = ownRandomAddr;
                            else
                                localAddr = ownPublicAddr;

                            if (ext_adv_hdr.targetA[0] != localAddr[0]
                                    || ext_adv_hdr.targetA[1] != localAddr[1]
                                    || ext_adv_hdr.targetA[2] != localAddr[2]
                                    || ext_adv_hdr.targetA[3] != localAddr[3]
                                    || ext_adv_hdr.targetA[4] != localAddr[4]
                                    || ext_adv_hdr.targetA[5] != localAddr[5])        // not check rxAdd, to add if required
                                bWlRlCheckOk = FALSE;
                        }
                    }

                    // if valid, trigger osal event to report adv
                    if (bWlRlCheckOk == FALSE)
                        ext_adv_hdr.header = 0;       // 11-19, ignore current adv event, the following PDUs of this event will not be received
                    else
                    {
                        uint16  advEventType = adv_mode;
                        int8   rssi;
                        uint8  adi = 0xFF, txPwr = 0x7F;

                        // active scan scenario, send scan req
                        if (extScanInfo.scanType[extScanInfo.current_index] == LL_SCAN_ACTIVE
                                && adv_mode == LL_EXT_ADV_MODE_SC       // only scannable adv accept AUX_SCAN_REQ
                                && !(ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK))
                        {
                            g_tx_adv_buf.txheader = 0xC03;

                            // ========================== active scan path   ===================
                            // fill scanA, using RPA or device ID address
                            if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM &&
                                    !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk))
                            {
                                // for resolving private address case, calculate the scanA with Local IRK
                                //ll_CalcRandomAddr(g_llResolvinglist[rpaListIndex].localIrk, &g_tx_adv_buf.data[0]);
                                osal_memcpy(&g_tx_adv_buf.data[0], &(ext_adv_hdr.targetA[0] ), 6);
                                g_tx_adv_buf.txheader |=(((g_rx_adv_buf.rxheader & TX_ADD_MASK) << 1)
                                                         | (LL_DEV_ADDR_TYPE_RANDOM << TX_ADD_SHIFT & TX_ADD_MASK));
                            }
                            else
                            {
                                memcpy((uint8*)&g_tx_adv_buf.data[0], &extScanInfo.ownAddr[0], 6);
                                g_tx_adv_buf.txheader |=(((g_rx_adv_buf.rxheader & TX_ADD_MASK) << 1)
                                                         | (extScanInfo.ownAddrType << TX_ADD_SHIFT & TX_ADD_MASK));
                            }

                            g_same_rf_channel_flag = TRUE;
                            ll_hw_set_tx_rx_interval(10);
                            ll_hw_set_rx_timeout(500);
                            set_max_length(0xFF);               // add 2020-03-10
                            T2 = read_current_fine_time();
                            delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);

                            if ((delay > 140 - pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY] - pGlobal_config[LL_HW_PLL_DELAY])
                                    && (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM))
                            {
                                // not enough time to send scan req, store the RPA
                                isPeerRpaStore = TRUE;
                                storeRpaListIndex = rpaListIndex;
                                osal_memcpy(&currentPeerRpa[0], &ext_adv_hdr.advA[0], 6);
                                g_same_rf_channel_flag = FALSE;
                            }
                            else
                            {
                                if(g_rfPhyPktFmt == PKT_FMT_BLE1M)
                                {
                                    delay = 118 - delay - pGlobal_config[EXT_ADV_AUXSCANREQ_DELAY_1MPHY];
                                }
                                else if(g_rfPhyPktFmt == PKT_FMT_BLE2M)
                                {
                                    delay = 118+15 - delay - pGlobal_config[EXT_ADV_AUXSCANREQ_DELAY_2MPHY];
                                }
                                else if(g_rfPhyPktFmt == PKT_FMT_BLR125K)
                                {
                                    delay = 118 - delay - pGlobal_config[EXT_ADV_AUXSCANREQ_DELAY_125KPHY];
                                }

//                                // temporary fixed for BQB test
//                                if (g_rfPhyPktFmt == PKT_FMT_BLE2M)
//                                    delay += 15;       // 2Mbps
//                                else if (g_rfPhyPktFmt == PKT_FMT_BLR125K)
//                                    delay = 0;   // coded PHY
                                ll_hw_set_trx();             // set LL HW as single TRx mode
//                                ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
//                                         pGlobal_config[LL_HW_AFE_DELAY],
//                                         pGlobal_config[LL_HW_PLL_DELAY]);          //RxAFE,PLL
                                ll_hw_trx_settle_bb(g_rfPhyPktFmt, delay);
                                ll_hw_go();
                                g_pmCounters.ll_send_scan_req_cnt++;
                                llWaitingIrq = TRUE;
                                // reset Rx/Tx FIFO
                                ll_hw_rst_rfifo();
                                ll_hw_rst_tfifo();
                                ll_hw_ign_rfifo(LL_HW_IGN_CRC | LL_HW_IGN_EMP);
                                // AdvA, for SCAN REQ, it should identical to the ADV_IND/ADV_SCAN_IND
                                g_tx_adv_buf.data[6]  = ext_adv_hdr.advA[0];
                                g_tx_adv_buf.data[7]  = ext_adv_hdr.advA[1];
                                g_tx_adv_buf.data[8]  = ext_adv_hdr.advA[2];
                                g_tx_adv_buf.data[9]  = ext_adv_hdr.advA[3];
                                g_tx_adv_buf.data[10] = ext_adv_hdr.advA[4];
                                g_tx_adv_buf.data[11] = ext_adv_hdr.advA[5];
                                //write Tx FIFO
                                ll_hw_write_tfifo((uint8*)&(g_tx_adv_buf.txheader),
                                                  ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                                bSendingScanReq = TRUE;
                                g_same_rf_channel_flag = FALSE;
                                // TODO: whether we should reset the flag
                                isPeerRpaStore = FALSE;
                                //LOG("D1 %d\n",delay);
                            }

                            // ========================== active scan path end  ===================
                        }

                        adv_data_len = payload_len - ext_hdr_len - 1;
                        rssi  =  -(pktFoot1 >> 24);

                        if (ext_adv_hdr.header & LE_EXT_HDR_ADI_PRESENT_BITMASK)
                            adi = (ext_adv_hdr.adi & 0xF000) >> 12;

                        if (ext_adv_hdr.header & LE_EXT_HDR_TX_PWR_PRESENT_BITMASK)
                            txPwr = ext_adv_hdr.txPower;

                        activeScanAdi = adi;

                        if (ext_adv_hdr.header & LE_EXT_HDR_SYNC_INFO_PRESENT_BITMASK)     // periodic adv report sync info
                        {
                            uint8 pri_phy, sec_phy;

                            if (peerAddr == NULL ||
                                    (peerAddr[0] == 0 &&
                                     peerAddr[1] == 0 &&
                                     peerAddr[2] == 0 &&
                                     peerAddr[3] == 0 &&
                                     peerAddr[4] == 0 &&
                                     peerAddr[5] == 0 ))
                            {
                                peerAddr = &tempAdvA[0];
                            }

                            pri_phy = extScanInfo.scanPHYs[extScanInfo.current_index];
                            sec_phy = extScanInfo.current_scan_PHY;

                            if (pri_phy == PKT_FMT_BLR125K) pri_phy = 3;    // coded PHY

                            if (sec_phy == PKT_FMT_BLR125K) sec_phy = 3;    // coded PHY

                            LL_ExtAdvReportCback0( 0,                            // event type
                                                   peerAddrType,                                // Adv address type (TxAdd)
                                                   &peerAddr[0],                         // Adv address (AdvA)
                                                   pri_phy,                              // primary PHY
                                                   sec_phy,                              // secondary PHY
                                                   adi,
                                                   txPwr,
                                                   rssi,
                                                   syncInfo.interval,                                     // periodicAdvertisingInterval, TO update
                                                   0,
                                                   NULL,                                 // NULL for undirect adv report
                                                   adv_data_len,                                    // for periodic syncInfo, no adv data
                                                   &g_rx_adv_buf.data[ext_hdr_len + 1]);                // rest of payload
                        }
                        else if (!bSendingScanReq //(adv_data_len > 0 && !bSendingScanReq)     // for scannable adv, no adv data. should we report the adv report?
                                 && (!(ext_adv_hdr.header & LE_EXT_HDR_TARGETA_PRESENT_BITMASK)))  // 2021-1-14, not report when no advData & with aux packet
                            //&& (adv_data_len >= 0 || !(ext_adv_hdr.header & LE_EXT_HDR_TARGETA_PRESENT_BITMASK)))  // 2021-1-14, not report when no advData & with aux packet
                        {
                            uint8* directA = NULL;
                            uint8  directAddrType = 0;

                            // targetA field present
                            if (ext_adv_hdr.header & LE_EXT_HDR_TARGETA_PRESENT_BITMASK)
                            {
                                advEventType |= LE_ADV_PROP_DIRECT_BITMASK;
                                directA = extScanInfo.ownAddr;//ext_adv_hdr.targetA;

                                if (ext_adv_hdr.targetA[5] & RANDOM_ADDR_HDR)            // TODO: decide the direct addr type
                                    directAddrType = LL_DEV_ADDR_TYPE_RANDOM;
                                else
                                    directAddrType = LL_DEV_ADDR_TYPE_PUBLIC;

                                // TODO: check the setting
//                              directAddrType = 0x01;//BQBTEST: SCN_BV_63
//                              directA = tempTargetA;//ownPublicAddr;//ownRandomAddr;
                            }

                            osal_memcpy(&tempTargetA[0], &(ext_adv_hdr.targetA[0]),6);

                            if (ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)   // more data
                                advEventType |= LE_ADV_PROP_MORE_DATA_BITMASK;

                            uint8 pri_phy, sec_phy;
                            pri_phy = extScanInfo.scanPHYs[extScanInfo.current_index];
                            sec_phy = extScanInfo.current_scan_PHY;

                            if (pri_phy == PKT_FMT_BLR125K) pri_phy = 3;    // coded PHY

                            if (sec_phy == PKT_FMT_BLR125K) sec_phy = 3;    // coded PHY

                            // copy the adv_data
                            LL_ExtAdvReportCback0( advEventType,                         // event type
                                                   scanningpeer_addrtype,                                // Adv address type (TxAdd)
                                                   &scanningpeer_addr[0],                         // Adv address (AdvA)
                                                   pri_phy,                              // primary PHY
                                                   sec_phy,                              // secondary PHY
                                                   adi,
                                                   txPwr,
                                                   rssi,
                                                   0,                                     // periodicAdvertisingInterval, reserved
                                                   directAddrType,
                                                   directA,                                 // NULL for undirect adv report
                                                   adv_data_len,
                                                   &g_rx_adv_buf.data[ext_hdr_len + 1]);                // rest of payload
                            g_pmCounters.ll_recv_adv_pkt_cnt ++;
                        }
                    }
                }
                // case 2: legacy adv type, only applicable to 1Mbps PHY
                else if (pktErrFlg == 0    //else if (packet_len   != 0
                         && ((pdu_type == ADV_IND)
                             || (pdu_type  == ADV_NONCONN_IND)
                             || (pdu_type  == ADV_SCAN_IND)
                             || (pdu_type == ADV_DIRECT_IND)))
                {
                    uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                    uint8* localAddr;

                    if (extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM || extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM)
                        localAddr = ownRandomAddr;
                    else
                        localAddr = ownPublicAddr;

                    peerAddr = &g_rx_adv_buf.data[0];        // AdvA
                    peerAddrType = txAdd;

                    // check white list
                    if ((pGlobal_config[LL_SWITCH] & LL_WHITELIST_ALLOW)
                            && (extScanInfo.wlPolicy  == LL_SCAN_WL_POLICY_USE_WHITE_LIST)
                            && (bWlRlCheckOk == TRUE))
                    {
                        // check white list
                        bWlRlCheckOk = ll_isAddrInWhiteList(txAdd, peerAddr);
                    }

                    if(pdu_type == ADV_DIRECT_IND)    // direct adv only report addr & addr type match itself
                    {
                        // compare direct adv targetA & own addr
                        if (g_rx_adv_buf.data[6] != localAddr[0] ||
                                g_rx_adv_buf.data[7] != localAddr[1] ||
                                g_rx_adv_buf.data[8] != localAddr[2] ||
                                g_rx_adv_buf.data[9] != localAddr[3] ||
                                g_rx_adv_buf.data[10] != localAddr[4] ||
                                g_rx_adv_buf.data[11] != localAddr[5] )   // TODO: add check RxAdd ?
                            bWlRlCheckOk = FALSE;
                        else
                            bWlRlCheckOk = TRUE;
                    }

                    // if valid, trigger osal event to report adv
                    if (bWlRlCheckOk == TRUE)
                    {
                        uint16  advEventType;
                        int8   rssi;

//                        llCurrentScanChn = extScanInfo.nextScanChan;            // TO check

                        // active scan scenario, send scan req
                        if (extScanInfo.scanType[0] == LL_SCAN_ACTIVE
                                && (pdu_type == ADV_IND
                                    || pdu_type == ADV_SCAN_IND ))
                        {
                            {
                                g_tx_adv_buf.txheader = 0xC03;
                                {
                                    g_same_rf_channel_flag = TRUE;
                                    ll_hw_set_tx_rx_interval(10);
//                                    ll_hw_set_rx_timeout(158);
                                    ll_hw_set_rx_timeout(300);
                                    set_max_length(0xFF);                    // add 2020-03-10
                                    T2 = read_current_fine_time();
                                    delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);
                                    delay = 118 - delay - pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY];
                                    ll_hw_set_trx();             // set LL HW as single TRx mode
//                                    ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
//                                         pGlobal_config[LL_HW_AFE_DELAY],
//                                         pGlobal_config[LL_HW_PLL_DELAY]);          //RxAFE,PLL
                                    ll_hw_trx_settle_bb(g_rfPhyPktFmt, delay);
                                    ll_hw_go();
                                    g_pmCounters.ll_send_scan_req_cnt++;
                                    llWaitingIrq = TRUE;
                                    // reset Rx/Tx FIFO
                                    ll_hw_rst_rfifo();
                                    ll_hw_rst_tfifo();
                                    ll_hw_ign_rfifo(LL_HW_IGN_CRC | LL_HW_IGN_EMP);
                                    //20181012 ZQ: change the txheader according to the adtype
                                    g_tx_adv_buf.txheader |=(((g_rx_adv_buf.rxheader&0x40)<<1)
                                                             | (extScanInfo.ownAddrType<< TX_ADD_SHIFT & TX_ADD_MASK));
                                    // fill scanA, using RPA or device ID address   // TODO: move below code before ll_hw_go?
//                                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM &&
//                                      !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk))
//                                    {    // for resolving private address case, calculate the scanA with Local IRK
//                                        ll_CalcRandomAddr(g_llResolvinglist[rpaListIndex].localIrk, &g_tx_adv_buf.data[0]);
////                                        osal_memcpy( &g_currentLocalRpa[0],  &g_tx_adv_buf.data[0], 6);
//                                    }
//                                  else
                                    {
                                        //LL_ReadBDADDR(&g_tx_adv_buf.data[0]);
                                        memcpy((uint8*)&g_tx_adv_buf.data[0], &extScanInfo.ownAddr[0], 6);
                                    }
                                    // AdvA, for SCAN REQ, it should identical to the ADV_IND/ADV_SCAN_IND
                                    g_tx_adv_buf.data[6]  = peerAddr[0];
                                    g_tx_adv_buf.data[7]  = peerAddr[1];
                                    g_tx_adv_buf.data[8]  = peerAddr[2];
                                    g_tx_adv_buf.data[9]  = peerAddr[3];
                                    g_tx_adv_buf.data[10] = peerAddr[4];
                                    g_tx_adv_buf.data[11] = peerAddr[5];
                                    //write Tx FIFO
                                    ll_hw_write_tfifo((uint8*)&(g_tx_adv_buf.txheader),
                                                      ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
                                    bSendingScanReq = TRUE;
                                    g_same_rf_channel_flag = FALSE;

                                    //LOG("D2 %d\n",delay);
                                    if (pdu_type == ADV_SCAN_IND)
                                        scanningScanInd = TRUE;
                                }
                            }
                        }

                        // convert pdu type to GAP enum
                        switch (pdu_type)
                        {
                        case ADV_IND:
                            advEventType = LL_EXT_ADV_PROP_ADV_IND;
                            break;

                        case ADV_SCAN_IND:
                            advEventType = LL_EXT_ADV_PROP_ADV_SCAN_IND;
                            break;

                        case ADV_DIRECT_IND:
                            advEventType = LL_EXT_ADV_PROP_ADV_LDC_ADV;
                            break;

                        case ADV_NONCONN_IND:
                            advEventType = LL_EXT_ADV_PROP_ADV_NOCONN_IND;
                            break;

//                            case ADV_SCAN_RSP:               // TODO: fill advEventType
//                                advEventType = 0x1a;         // SCAN_RSP to an ADV_IND: 0x1b, SCAN_RSP to an ADV_SCAN_IND: 0x1a
//                                break;
                        default:
                            advEventType = LL_ADV_RPT_ADV_IND;
                            break;
                        }

                        rssi  =  -(pktFoot1 >> 24);
                        rssi  =  -(pktFoot1 >> 24);

                        if (pktLen >= 8)
                        {
                            // copy the adv_data
                            LL_ExtAdvReportCback0( advEventType,                         // event type
                                                   peerAddrType,                                // Adv address type (TxAdd)
                                                   &peerAddr[0],                         // Adv address (AdvA)
                                                   0x01,                                 // primary PHY, 1Mbps PHY only
                                                   0x0,                                  // secondary PHY, no applicable
                                                   0xFF,                                  // no ADI
                                                   0x7F,                                  // no Tx power
                                                   rssi,
                                                   0,                                     // periodicAdvertisingInterval, reserved
                                                   0,
                                                   NULL,                                 // NULL for undirect adv report
                                                   pktLen - 8,                           // length of rest of the payload, 2 - header, 6 - advA
                                                   &g_rx_adv_buf.data[6]);                // rest of payload
                            g_pmCounters.ll_recv_adv_pkt_cnt ++;
                        }
                    }
                }
                else
                {
                    // invalid ADV PDU type
//                    llSetupScan();
                }
            }
            else if(scanningauxchain)
            {
                //send data_truncated advrpt when recv auxchain rxTO
                LL_ExtAdvReportCback0( LE_ADV_PROP_DATA_TRUNCATED_BITMASK,                         // event type
                                       scanningpeer_addrtype,                                // Adv address type (TxAdd)
                                       &scanningpeer_addr[0],                         // Adv address (AdvA)
                                       0x01,                                 // primary PHY, 1Mbps PHY only
                                       0x0,                                  // secondary PHY, no applicable
                                       0xFF,                                  // no ADI
                                       0x7F,                                  // no Tx power
                                       0x7F,
                                       0,                                     // periodicAdvertisingInterval, reserved
                                       0,
                                       NULL,                                 // NULL for undirect adv report
                                       0,                           // length of rest of the payload, 2 - header, 6 - advA
                                       NULL);                // rest of payload
                scanningauxchain=FALSE;
            }

            // if not waiting for scan rsp, schedule next scan
            if (!bSendingScanReq)
            {
                uint8   bStartPeriodScan = FALSE;

                // =========== scanning periodic adv case
                if ((ext_adv_hdr.header & LE_EXT_HDR_SYNC_INFO_PRESENT_BITMASK)
                        && (scanSyncInfo.valid == TRUE))
                {
                    uint16    sync_handler;
                    uint32    schedule_time;
                    uint32    accessAddress;
                    uint32     pkt_time;

                    // 2021-1-14, consider AUX_ADV_IND PDU time when calculate offset to AUX_SYNC_IND
                    if (extScanInfo.current_scan_PHY == PKT_FMT_BLE1M)
                        pkt_time = packet_len * 8 * 4;
                    else if (extScanInfo.current_scan_PHY == PKT_FMT_BLE2M)
                        pkt_time = packet_len * 8 * 2;           // no such case?
                    else               // coded PHY, S = 8
                        pkt_time = packet_len * 8 * 4 * 8;

                    llPeriodicScannerInfo_t*  pPrdScanInfo;
                    sync_handler = llAllocateSyncHandle();
                    pPrdScanInfo = &g_llPeriodAdvSyncInfo[sync_handler];     // only 1 period scanner
                    // get elapse time since the AUX_ADV_IND start point
                    pPrdScanInfo->syncHandler  = sync_handler;
                    pPrdScanInfo->eventCounter = syncInfo.event_counter;
                    pPrdScanInfo->advInterval  = syncInfo.interval * 1250;
                    memcpy(&pPrdScanInfo->chnMap[0], &syncInfo.chn_map[0], 4);
                    pPrdScanInfo->chnMap[4]    = syncInfo.chn_map4.chn_map;
                    pPrdScanInfo->sca          = syncInfo.chn_map4.sca;
                    memcpy(&pPrdScanInfo->accessAddress[0], &syncInfo.AA[0], 4);
                    memcpy(&pPrdScanInfo->crcInit[0], &syncInfo.crcInit[0], 3);  // TO check bit order
                    accessAddress = (pPrdScanInfo->accessAddress[3] << 24)
                                    | (pPrdScanInfo->accessAddress[2] << 16)
                                    | (pPrdScanInfo->accessAddress[1] << 8)
                                    |  pPrdScanInfo->accessAddress[0];
                    pPrdScanInfo->advPhy = extScanInfo.current_scan_PHY;
                    pPrdScanInfo->syncTimeout = scanSyncInfo.sync_Timeout * 10000;
                    pPrdScanInfo->syncCteType = scanSyncInfo.sync_CTE_Type;
                    pPrdScanInfo->skip        = scanSyncInfo.skip;
                    // calculate next event timer, need consider the start point of AUX_ADV_IND
                    pPrdScanInfo->nextEventRemainder = ((syncInfo.offset.offsetUnit == 1) ? 300 : 30) * syncInfo.offset.syncPacketOffset;
                    pPrdScanInfo->nextEventRemainder -= pkt_time;
                    pPrdScanInfo->syncEstOk = FALSE;
                    pPrdScanInfo->event1stFlag = TRUE;        // receiving the 1st PDU of a periodic event
                    // TODO
                    pPrdScanInfo->channelIdentifier = (( accessAddress & 0xFFFF0000 )>> 16 ) ^ ( accessAddress & 0x0000FFFF);

                    if(g_llPeriodAdvSyncInfo[sync_handler].nextEventRemainder>2200)
                        schedule_time = g_llPeriodAdvSyncInfo[sync_handler].nextEventRemainder - 2000;  // 1000: timing advance
                    else
                        schedule_time = 200;

                    for (int i = 0; i < LL_NUM_BYTES_FOR_CHAN_MAP; i++)
                    {
                        // for each channel given by a bit in each of the bytes
                        // Note: When i is on the last byte, only 5 bits need to be checked, but
                        //       it is easier here to check all 8 with the assumption that the rest
                        //       of the reserved bits are zero.
                        for (uint8 j = 0; j < 8; j++)
                        {
                            // check if the channel is used; only interested in used channels
                            if ( (pPrdScanInfo->chnMap[i] >> j) & 1 )
                            {
                                // sequence used channels in ascending order
                                pPrdScanInfo->chanMapTable[pPrdScanInfo->numUsedChans] = (i * 8U) + j;
                                // count it
                                pPrdScanInfo->numUsedChans++;
                            }
                        }
                    }

                    pPrdScanInfo->currentEventChannel = llGetNextDataChanCSA2(pPrdScanInfo->eventCounter,
                                                                              pPrdScanInfo->channelIdentifier,
                                                                              pPrdScanInfo->chnMap,
                                                                              pPrdScanInfo->chanMapTable,
                                                                              pPrdScanInfo->numUsedChans);
                    g_llPeriodAdvSyncInfo[sync_handler].current_channel = pPrdScanInfo->currentEventChannel;
                    // start timer
                    ll_prd_scan_schedule_next_event(schedule_time);
                    bStartPeriodScan = TRUE;
                }

                // ============== scanning extended adv case

                if (bStartPeriodScan == FALSE)
                {
                    llScanDuration += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

                    if (extScanInfo.duration != 0 && extScanInfo.period == 0 && llScanDuration >= extScanInfo.duration * 10000)  // scan timeout
                    {
                        llTaskState = LL_TASK_INVALID;
                        LL_ScanTimeoutCback();                 // 2020-11-26
                    }
                    else
                    {
                        if(ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK &&
                                extScanInfo.current_chn<LL_SCAN_ADV_CHAN_37)
                            scanningauxchain=TRUE;
                        else
                            scanningauxchain=FALSE;

                        // case 1: receives auxPtr, update PHY/chn according to AuxPtr, start timer according to aux offset
                        if (ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)
                        {
                            uint32    wait_time, pkt_time;

                            // 2021-1-7, take out PDU time from AuxPtr offset
                            if (extScanInfo.current_scan_PHY == PKT_FMT_BLE1M)
                                pkt_time = packet_len * 8 * 4;
                            else if (extScanInfo.current_scan_PHY == PKT_FMT_BLE2M)
                                pkt_time = packet_len * 8 * 2;           // no such case?
                            else               // coded PHY, S = 8
                                pkt_time = packet_len * 8 * 4 * 8;

                            extScanInfo.current_chn = ext_adv_hdr.auxPtr.chn_idx;
                            extScanInfo.current_scan_PHY = ext_adv_hdr.auxPtr.aux_phy;
                            wait_time = ext_adv_hdr.auxPtr.aux_offset * ((ext_adv_hdr.auxPtr.offset_unit == 1) ? 300 : 30);
                            wait_time -= pkt_time;

                            if (wait_time <= 1000)
                                wait_time = 10;
                            else
                                wait_time -= 990;     // scan advance

                            llScanTime = 0;
                            ll_ext_scan_schedule_next_event(wait_time);
                        }
                        // case 2: scanning aux channel, no more aux PDU, continue to scan in primary channel
                        else if (extScanInfo.current_chn < LL_SCAN_ADV_CHAN_37)
                        {
                            extScanInfo.current_chn = LL_SCAN_ADV_CHAN_37;

                            if (extScanInfo.numOfScanPHY > 1)
                            {
                                extScanInfo.current_index = (extScanInfo.current_index + 1) & 0x01;
                            }

                            extScanInfo.current_scan_PHY = extScanInfo.scanPHYs[extScanInfo.current_index];
                            llSetupExtScan(extScanInfo.current_chn);
                            llScanTime = 0;
                        }
                        // case 3: update scan time, only when scan primary channel and not receive AuxPtr
                        else
                        {
                            // not sending SCAN REQ, update scan time
                            llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

                            if (llScanTime >= extScanInfo.scanWindow[extScanInfo.current_index] * 625)
                            {
                                if (extScanInfo.numOfScanPHY > 1 && extScanInfo.current_chn == LL_SCAN_ADV_CHAN_39)
                                {
                                    extScanInfo.current_index = (extScanInfo.current_index + 1) & 0x01;
                                    extScanInfo.current_scan_PHY = extScanInfo.scanPHYs[extScanInfo.current_index];
                                }

                                LL_CALC_NEXT_SCAN_CHN(extScanInfo.current_chn);

                                // schedule next scan event
                                if (extScanInfo.scanWindow[extScanInfo.current_index] == extScanInfo.scanInterval[extScanInfo.current_index])      // scanWindow == scanInterval, trigger immediately
                                    llSetupExtScan(extScanInfo.current_chn);
                                else
                                {
                                    ll_ext_scan_schedule_next_event((extScanInfo.scanInterval[extScanInfo.current_index]
                                                                     - extScanInfo.scanWindow[extScanInfo.current_index]) * 625);
                                    llScanDuration += (extScanInfo.scanInterval[extScanInfo.current_index]
                                                       - extScanInfo.scanWindow[extScanInfo.current_index]) * 625;
                                }

                                // reset scan total time
                                llScanTime = 0;
                            }
                            else
                            {
                                llSetupExtScan(extScanInfo.current_chn);
                            }
                        }
                    }
                }
            }
        }
    }
    else if (ll_mode == LL_HW_MODE_TRX && llTaskState == LL_TASK_EXTENDED_SCAN)            // active scan
    {
        uint8_t*  peerAddr = &ext_adv_hdr.advA[0];
        uint8    peerAddrType;                  // peer address type
        uint8_t  packet_len, pdu_type;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        uint8_t  pktErrFlg = 0;
        memset(&ext_adv_hdr, 0, sizeof(ext_adv_hdr));

        // receice CRC OK, process SCAN_RSP or AUX_SCAN_RSP
        if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))
        {
            // rx done
            // read packet
            packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                           &pktLen,
                                           &pktFoot0,
                                           &pktFoot1);
            // check receive pdu type
            pdu_type = g_rx_adv_buf.rxheader & 0x0f;

            if(ll_hw_get_rfifo_depth() > 0)
            {
                g_pmCounters.ll_rfifo_read_err++;
                packet_len = 0;
                pktLen = 0;
                pktErrFlg = 1;
            }

            // receive legacy SCAN_RSP PDU
            if (pktErrFlg == 0 /*packet_len > 0*/ && pdu_type == ADV_SCAN_RSP)
            {
                // receives SCAN_RSP
                uint8  rpaListIndex;
                uint8  peerAddrType = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;
                int8   rssi     =  -(pktFoot1 >> 24);
                uint8  bCheckOk = TRUE;
                peerAddr = &g_rx_adv_buf.data[0];

                //===
                // AdvA of SCAN_RSP should also be checked here. Refer to 4.4.3.2 Active Scanning
                // After sending a scan request PDU the Link Layer listens for a scan response
                //PDU from that advertiser. If the scan response PDU was not received from that
                //advertiser, it is considered a failure; otherwise it is considered a success.

                // check AdvA in Scan Rsp is identical to Scan Req
                if (g_rx_adv_buf.data[0] != g_tx_adv_buf.data[6]  ||
                        g_rx_adv_buf.data[1] != g_tx_adv_buf.data[7]  ||
                        g_rx_adv_buf.data[2] != g_tx_adv_buf.data[8]  ||
                        g_rx_adv_buf.data[3] != g_tx_adv_buf.data[9]  ||
                        g_rx_adv_buf.data[4] != g_tx_adv_buf.data[10] ||
                        g_rx_adv_buf.data[5] != g_tx_adv_buf.data[11]
                   )
                    bCheckOk = FALSE;

                // RPA checking. Note that we do not check whether it is the same RPA index
                if (bCheckOk == TRUE && peerAddrType == LL_DEV_ADDR_TYPE_RANDOM  &&
                        (g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                {
                    if (g_llRlEnable == TRUE)
                    {
                        rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);

                        if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                        {
                            peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                            // refer to HCI LE Advertising Report Event, RPA address type should be
                            // 0x02: Public Identity Address (Corresponds to Resolved Private Address)
                            // 0x03: Random (static) Identity Address (Corresponds to Resolved Private Address)
                            peerAddrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                            bCheckOk = TRUE;
                        }
                        else
                            bCheckOk = FALSE;
                    }
                }

                //===
//                                advEventType = 0x1a;         // SCAN_RSP to an ADV_IND: 0x1b, SCAN_RSP to an ADV_SCAN_IND: 0x1a

                if (bCheckOk == TRUE)
                {
                    uint8 advEvt = 0x1B;

                    if (scanningScanInd == TRUE)
                        advEvt = 0x1A;

                    LL_ExtAdvReportCback0(advEvt,                  // event type
                                          peerAddrType,                         // Adv address type (TxAdd)
                                          peerAddr,                             // Adv address (AdvA)
                                          0x01,                                 // primary PHY, 1Mbps PHY only
                                          0x0,                                  // secondary PHY, no applicable
                                          0xFF,                                  // no ADI
                                          0x7F,                                  // no Tx power
                                          rssi,
                                          0,                                     // periodicAdvertisingInterval, reserved
                                          0,
                                          NULL,                                 // NULL for undirect adv report
                                          pktLen - 8,                           // length of rest of the payload, 2 - header, 6 - advA
                                          &g_rx_adv_buf.data[6]);                // rest of payload
                    g_pmCounters.ll_recv_scan_rsp_cnt ++;
                }
            }
            // receive AUX_SCAN_RSP PDU
            else if (pktErrFlg == 0 /*packet_len > 0*/ && pdu_type == ADV_EXT_TYPE)
            {
                uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                uint8   payload_len = (g_rx_adv_buf.rxheader & 0xFF00) >> LENGTH_SHIFT;
                uint8   adv_data_len;
                uint8   txPwr = 0x7F, rssi, adi;
                uint16   advEventType = LE_ADV_PROP_SCAN_BITMASK | LE_ADV_PROP_SCAN_RSP_BITMASK;  // LE_ADV_PROP_DIRECT_BITMASK: temp set for BQB LL/DDI/SCN/BV-63-C
                peerAddrType = txAdd;
                adv_mode    = (g_rx_adv_buf.data[0] & 0xc0) >> 6;
                ext_hdr_len =  g_rx_adv_buf.data[0] & 0x3f;
                ll_parseExtHeader(&g_rx_adv_buf.data[1], ext_hdr_len);//payload_len - 1);   // update length 20201012
                peerAddr = &ext_adv_hdr.advA[0];
                uint8_t tIdx = ll_getRPAListEntry(&ext_adv_hdr.advA[0]);

                if (tIdx < LL_RESOLVINGLIST_ENTRY_NUM)
                {
                    peerAddr = &g_llResolvinglist[tIdx].peerAddr[0];
                    peerAddrType = g_llResolvinglist[tIdx].peerAddrType;
                }

                adv_data_len = payload_len - ext_hdr_len - 1;
                rssi  =  -(pktFoot1 >> 24);

                if (ext_adv_hdr.header & LE_EXT_HDR_ADI_PRESENT_BITMASK)
                    adi = (ext_adv_hdr.adi & 0xF000) >> 12;
                else
                    adi = activeScanAdi;

                if (ext_adv_hdr.header & LE_EXT_HDR_TX_PWR_PRESENT_BITMASK)
                    txPwr = ext_adv_hdr.txPower;

                //if (adv_data_len > 0)                                            // receive advData
                {
                    if (ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)   // more data
                        advEventType |= LE_ADV_PROP_MORE_DATA_BITMASK;

                    uint8 extscan_prim_phy = extScanInfo.scanPHYs[extScanInfo.current_index];

                    if(extscan_prim_phy==0x04)
                    {
                        extscan_prim_phy=0x03;
                    }

                    // copy the adv_data
                    LL_ExtAdvReportCback0( advEventType,                         // event type
                                           peerAddrType,                         // Adv address type (TxAdd)
                                           &peerAddr[0],                         // Adv address (AdvA)
                                           extscan_prim_phy,                             // primary PHY
                                           extScanInfo.current_scan_PHY,          // secondary PHY
                                           adi,
                                           txPwr,
                                           rssi,
                                           0,                                     // periodicAdvertisingInterval, reserved
                                           0,
                                           NULL,                                 // NULL for undirect adv report
                                           adv_data_len,
                                           &g_rx_adv_buf.data[ext_hdr_len + 1]);                // rest of payload
                    g_pmCounters.ll_recv_adv_pkt_cnt ++;
                }
            }
        }

        // trigger next scan
        llScanDuration += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

        if (extScanInfo.duration != 0 && extScanInfo.period == 0 && llScanDuration >= extScanInfo.duration * 10000)  // scan timeout
        {
            llTaskState = LL_TASK_INVALID;
            LL_ScanTimeoutCback();                 // 2020-11-26
        }
        else
        {
            if(ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK &&
                    extScanInfo.current_chn<LL_SCAN_ADV_CHAN_37)
                scanningauxchain=TRUE;
            else
                scanningauxchain=FALSE;

            // case 1: receives auxPtr, update PHY/chn according to AuxPtr, start timer according to aux offset
            if (ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)
            {
                uint32    wait_time, pkt_time;

                // 2021-1-7, take out PDU time from AuxPtr offset
                if (extScanInfo.current_scan_PHY == PKT_FMT_BLE1M)
                    pkt_time = packet_len * 8 * 4;
                else if (extScanInfo.current_scan_PHY == PKT_FMT_BLE2M)
                    pkt_time = packet_len * 8 * 2;           // no such case?
                else               // coded PHY, S = 8
                    pkt_time = packet_len * 8 * 4 * 8;

                extScanInfo.current_chn = ext_adv_hdr.auxPtr.chn_idx;
                extScanInfo.current_scan_PHY = ext_adv_hdr.auxPtr.aux_phy;
                wait_time = ext_adv_hdr.auxPtr.aux_offset * ((ext_adv_hdr.auxPtr.offset_unit == 1) ? 300 : 30);
                wait_time -= pkt_time;

                if (wait_time <= 1000)
                    wait_time = 10;
                else
                    wait_time -= 990;     // scan advance

                llScanTime = 0;
                ll_ext_scan_schedule_next_event(wait_time);
                memcpy(&tempAdvA[0], &peerAddr[0], 6);      // LL/DDI/SCN/BV-25-C
            }
            // case 2: scanning aux channel, no more aux PDU, continue to scan in primary channel
            else if (extScanInfo.current_chn < LL_SCAN_ADV_CHAN_37)
            {
                extScanInfo.current_chn = LL_SCAN_ADV_CHAN_37;

                if (extScanInfo.numOfScanPHY > 1)
                {
                    extScanInfo.current_index = (extScanInfo.current_index + 1) & 0x01;
                }

                extScanInfo.current_scan_PHY = extScanInfo.scanPHYs[extScanInfo.current_index];
                llSetupExtScan(extScanInfo.current_chn);
                llScanTime = 0;
            }
            // case 3: update scan time, only when scan primary channel and not receive AuxPtr
            else
            {
                // not sending SCAN REQ, update scan time
                llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

                if (llScanTime >= extScanInfo.scanWindow[extScanInfo.current_index] * 625)
                {
                    if (extScanInfo.numOfScanPHY > 1 && extScanInfo.current_chn == LL_SCAN_ADV_CHAN_39)
                    {
                        extScanInfo.current_index = (extScanInfo.current_index + 1) & 0x01;
                        extScanInfo.current_scan_PHY = extScanInfo.scanPHYs[extScanInfo.current_index];
                    }

                    LL_CALC_NEXT_SCAN_CHN(extScanInfo.current_chn);

                    // schedule next scan event
                    if (extScanInfo.scanWindow[extScanInfo.current_index] == extScanInfo.scanInterval[extScanInfo.current_index])      // scanWindow == scanInterval, trigger immediately
                        llSetupExtScan(extScanInfo.current_chn);
                    else
                    {
                        ll_ext_scan_schedule_next_event((extScanInfo.scanInterval[extScanInfo.current_index]
                                                         - extScanInfo.scanWindow[extScanInfo.current_index]) * 625);
                        llScanDuration += (extScanInfo.scanInterval[extScanInfo.current_index]
                                           - extScanInfo.scanWindow[extScanInfo.current_index]) * 625;
                    }

                    // reset scan total time
                    llScanTime = 0;
                }
                else
                {
                    llSetupExtScan(extScanInfo.current_chn);
                }
            }
        }
    }

    if (!llWaitingIrq)
    {
        ll_hw_clr_irq();
        llTaskState = LL_TASK_OTHERS;
    }

    HAL_EXIT_CRITICAL_SECTION();
    return TRUE;
}

//uint32   T_hw_go;
uint8 ll_processExtInitIRQ1(uint32_t      irq_status)
{
    //uint8          ll_mode, adv_mode, ext_hdr_len;
    uint8          ll_mode, adv_mode;
    llConnState_t*  connPtr;
    uint32_t      T2, delay;
    HAL_ENTER_CRITICAL_SECTION();
    ll_mode = ll_hw_get_tr_mode();

    if (ll_mode == LL_HW_MODE_SRX)
    {
        uint8_t  bWlRlCheckOk = TRUE;
        uint8 bConnecting = FALSE;
        uint8_t packet_len, pdu_type;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        connPtr = &conn_param[extInitInfo.connId];           // connId is allocated when create conn
        memset(&ext_adv_hdr, 0, sizeof(ext_adv_hdr));        // 2021-1-4

        // check status
        if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))
        {
            // rx done
            if (1/*adv_mode == LL_EXT_ADV_MODE_CONN*/
                    && g_rfPhyPktFmt == PKT_FMT_BLR125K
                    && extInitInfo.current_chn < 37)
            {
                g_same_rf_channel_flag = TRUE;
                ll_hw_set_trx();
                ll_hw_go();
                llWaitingIrq = TRUE;
                ll_hw_set_trx_settle(43,//30,//delay,                             // set BB delay, about 80us in 16MHz HCLK
                                     pGlobal_config[LL_HW_AFE_DELAY_125KPHY],
                                     pGlobal_config[LL_HW_PLL_DELAY_125KPHY]);        //RxAFE,PLL
            }

            // read packet
            packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                           &pktLen,
                                           &pktFoot0,
                                           &pktFoot1);
            // check receive pdu type
            pdu_type = g_rx_adv_buf.rxheader & 0x0f;

            if(ll_hw_get_rfifo_depth() > 0)
            {
                g_pmCounters.ll_rfifo_read_err++;
                packet_len = 0;
                pktLen = 0;
            }

            if (packet_len   != 0
                    && (pdu_type == ADV_EXT_TYPE))
            {
                // never used (addrType, txAdd)
                //uint8   addrType;                  // peer address type
                //uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
//                uint8_t chSel = (g_rx_adv_buf.rxheader & CHSEL_MASK) >> CHSEL_SHIFT;
//                uint8   payload_len = (g_rx_adv_buf.rxheader & 0xFF00) >> LENGTH_SHIFT;
                //addrType = txAdd;          // TO check  //never used
                adv_mode    = (g_rx_adv_buf.data[0] & 0xc0) >> 6;
                uint16 ext_hdr_len =  g_rx_adv_buf.data[0] & 0x3f; //never used
                ll_parseExtHeader(&g_rx_adv_buf.data[1], ext_hdr_len);//payload_len - 1);   // update length 20201012

                //===== TODO: RPA & whitelist checking
//              uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;
//              uint8   rpaListIndex;
//              bWlRlCheckOk = ll_isPeerAddrRpaCheckOK(txAdd, ext_adv_hdr.advA, &rpaListIndex);
//              // too add whitelist checking
                //check
                if(!osal_memcmp(ext_adv_hdr.advA,peerInfo.peerAddr,6))
                {
                    bWlRlCheckOk = FALSE;
                }

                if (bWlRlCheckOk == TRUE
                        && (adv_mode== LL_EXT_ADV_MODE_CONN)             // connectable adv
                        && !(ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK))    // AUX_ADV_IND. auxPtr is mandatory in connectable ADV_EXT_IND
                {
                    // gpio_write(P32,1);
                    // gpio_write(P32,0);
                    uint32 temp = llWaitingIrq;

                    if (!llWaitingIrq)
                    {
                        g_same_rf_channel_flag = TRUE;
                        ll_hw_set_trx();
                        ll_hw_go();
                        llWaitingIrq = TRUE;
                    }

                    // channel selection algorithm decision
//                    if ((pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
//                        && chSel == LL_CHN_SEL_ALGORITHM_2)
//                    {
//                          conn_param[initInfo.connId].channel_selection = LL_CHN_SEL_ALGORITHM_2;
//                          SET_BITS(g_tx_adv_buf.txheader, LL_CHN_SEL_ALGORITHM_2, CHSEL_SHIFT, CHSEL_MASK);
//                    }
//                    else
//                    {
//                            conn_param[initInfo.connId].channel_selection = LL_CHN_SEL_ALGORITHM_1;
//                          SET_BITS(g_tx_adv_buf.txheader, LL_CHN_SEL_ALGORITHM_1, CHSEL_SHIFT, CHSEL_MASK);
//                    }
                    // 2021-1-7, for extended ini case, always use CSA2
                    conn_param[extInitInfo.connId].channel_selection = LL_CHN_SEL_ALGORITHM_2;
                    llSetupAuxConnectReqPDU();
                    rf_phy_change_cfg0(g_rfPhyPktFmt);//extInitInfo.current_scan_PHY);
                    ll_hw_tx2rx_timing_config(g_rfPhyPktFmt);
                    // send conn req
                    // send conn req
                    T2 = read_current_fine_time();
                    delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);

                    //delay = 118 - delay - pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY] + 2;     // 11-19, 1M + 2, 2M + 17
                    if(g_rfPhyPktFmt == PKT_FMT_BLE1M)
                    {
                        delay = 118 - delay - pGlobal_config[EXT_ADV_AUXCONNREQ_DELAY_1MPHY]+17;
                    }
                    else if(g_rfPhyPktFmt == PKT_FMT_BLE2M)
                    {
                        delay = 118+15 - delay - pGlobal_config[EXT_ADV_AUXCONNREQ_DELAY_2MPHY]+17;
                    }

                    // else if(g_rfPhyPktFmt == PKT_FMT_BLR125K)
                    // {
                    //     delay=18;
                    // }
                    // temporary fixed for BQB test
                    // if (g_rfPhyPktFmt == PKT_FMT_BLE2M)
                    //     delay += 15;       // 2Mbps
                    // else if (g_rfPhyPktFmt == PKT_FMT_BLR125K)
                    //     delay = 0;   // coded PHY

                    if (!temp)//!llWaitingIrq)
                        ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
                                             0,//pGlobal_config[LL_HW_AFE_DELAY],
                                             pGlobal_config[LL_HW_PLL_DELAY]);        //RxAFE,PLL

                    // reset Rx/Tx FIFO
                    ll_hw_rst_rfifo();
                    ll_hw_rst_tfifo();
                    ll_hw_ign_rfifo(LL_HW_IGN_CRC | LL_HW_IGN_EMP);
                    // AdvA, offset 6
                    memcpy((uint8*)&g_tx_adv_buf.data[6], &ext_adv_hdr.advA[0], 6);
                    //write Tx FIFO
                    ll_hw_write_tfifo((uint8*)&(g_tx_adv_buf.txheader),
                                      ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)
//                    ll_hw_set_tx_rx_interval(10);
                    ll_hw_set_rx_timeout(700);
                    set_max_length(0xFF);
                    connPtr->llRfPhyPktFmt = g_rfPhyPktFmt;
                    // ========== bug fixed 0907
                    connPtr->curParam.winOffset = pGlobal_config[LL_CONN_REQ_WIN_OFFSET];
                    move_to_master_function1();
                    bConnecting = TRUE;
                    g_same_rf_channel_flag = FALSE;
                    //T_hw_go = read_current_fine_time();
                }
            }
            else if (packet_len   != 0
                     && ((pdu_type == ADV_IND) || pdu_type == ADV_DIRECT_IND))
            {
                uint8_t txAdd = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random
                uint8_t chSel = (g_rx_adv_buf.rxheader & CHSEL_MASK) >> CHSEL_SHIFT;
                uint8_t rpaListIndex = LL_RESOLVINGLIST_ENTRY_NUM;
                uint8_t* peerAddr = &g_rx_adv_buf.data[0];        // AdvA
                g_currentPeerAddrType = txAdd;
                uint8_t bMatchAdv = FALSE;

                // ================= Resolving list checking
                // case 1: receive InitA using RPA
                if (txAdd == LL_DEV_ADDR_TYPE_RANDOM  &&
                        ((g_rx_adv_buf.data[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR))
                {
                    bWlRlCheckOk = FALSE;

                    if (g_llRlEnable == TRUE)
                    {
                        // if the RPA checking is done in previous scan, compare
                        if (isPeerRpaStore  == TRUE  &&
                                currentPeerRpa[0] == g_rx_adv_buf.data[0]
                                && currentPeerRpa[1] == g_rx_adv_buf.data[1]
                                && currentPeerRpa[2] == g_rx_adv_buf.data[2]
                                && currentPeerRpa[3] == g_rx_adv_buf.data[3]
                                && currentPeerRpa[4] == g_rx_adv_buf.data[4]
                                && currentPeerRpa[5] == g_rx_adv_buf.data[5])
                        {
                            rpaListIndex = storeRpaListIndex;
                            peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                            g_currentPeerAddrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                            bWlRlCheckOk = TRUE;
                            bMatchAdv = TRUE;
                        }
                        else     // resolve the address
                        {
                            rpaListIndex = ll_getRPAListEntry(&g_rx_adv_buf.data[0]);    // spend 30us(48MHz) when the 1st item match

                            if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM)
                            {
                                peerAddr = &g_llResolvinglist[rpaListIndex].peerAddr[0];
                                g_currentPeerAddrType = g_llResolvinglist[rpaListIndex].peerAddrType + 2;
                                bWlRlCheckOk = TRUE;
                            }
                        }
                    }
                }
                // case 2: receive InitA using device ID, or init device not using RPA
                else
                {
                    for (int i = 0; i < LL_RESOLVINGLIST_ENTRY_NUM; i++)
                    {
                        if (g_llResolvinglist[i].peerAddr[0] == g_rx_adv_buf.data[0]
                                && g_llResolvinglist[i].peerAddr[1] == g_rx_adv_buf.data[1]
                                && g_llResolvinglist[i].peerAddr[2] == g_rx_adv_buf.data[2]
                                && g_llResolvinglist[i].peerAddr[3] == g_rx_adv_buf.data[3]
                                && g_llResolvinglist[i].peerAddr[4] == g_rx_adv_buf.data[4]
                                && g_llResolvinglist[i].peerAddr[5] == g_rx_adv_buf.data[5])
                        {
                            // the device ID in the RPA list
                            if (g_llResolvinglist[i].privacyMode == NETWORK_PRIVACY_MODE &&
                                    !ll_isIrkAllZero(g_llResolvinglist[i].peerIrk))
                            {
                                bWlRlCheckOk = FALSE;
                            }
                            else
                            {
                                rpaListIndex = i;
                            }
                        }
                    }
                }

                // ====== for direct adv, also check initA == own addr
                if (pdu_type == ADV_DIRECT_IND && bWlRlCheckOk == TRUE && bMatchAdv != TRUE)
                {
                    // initA is resolvable address case
                    if ((g_rx_adv_buf.data[11] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
                    {
                        // should not use RPA case
                        if (initInfo.ownAddrType != LL_DEV_ADDR_TYPE_RPA_PUBLIC && initInfo.ownAddrType != LL_DEV_ADDR_TYPE_RPA_RANDOM)
                        {
                            bWlRlCheckOk = FALSE;
                        }

                        if (rpaListIndex >= LL_RESOLVINGLIST_ENTRY_NUM
                                || (ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk))    // all-0 local IRK
                                || (ll_ResolveRandomAddrs(g_llResolvinglist[rpaListIndex].localIrk, &g_rx_adv_buf.data[6]) != SUCCESS))   // resolve failed
                        {
                            bWlRlCheckOk = FALSE;
                        }
                    }
                    else
                    {
                        uint8* localAddr;
                        uint8_t rxAdd = (g_rx_adv_buf.rxheader & RX_ADD_MASK) >> RX_ADD_SHIFT;

                        // should not use device ID case
                        if ((initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC || initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
                                && (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM
                                    && !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk)))
                        {
                            bWlRlCheckOk = FALSE;
                        }

                        if (rxAdd == LL_DEV_ADDR_TYPE_RANDOM)
                        {
                            localAddr = ownRandomAddr;
                        }
                        else
                        {
                            localAddr = ownPublicAddr;
                        }

                        if (g_rx_adv_buf.data[6]  != localAddr[0]
                                || g_rx_adv_buf.data[7]  != localAddr[1]
                                || g_rx_adv_buf.data[8]  != localAddr[2]
                                || g_rx_adv_buf.data[9]  != localAddr[3]
                                || g_rx_adv_buf.data[10] != localAddr[4]
                                || g_rx_adv_buf.data[11] != localAddr[5])
                        {
                            bWlRlCheckOk = FALSE;
                        }
                    }
                }

                // initiator, 2 types of filter process: 1. connect to peer address set by host   2. connect to  address in whitelist only
                // 1. connect to peer address set by host
                if (initInfo.wlPolicy == LL_INIT_WL_POLICY_USE_PEER_ADDR
                        && bWlRlCheckOk == TRUE)
                {
                    if (peerAddr[0]  != peerInfo.peerAddr[0]
                            || peerAddr[1]  != peerInfo.peerAddr[1]
                            || peerAddr[2]  != peerInfo.peerAddr[2]
                            || peerAddr[3]  != peerInfo.peerAddr[3]
                            || peerAddr[4]  != peerInfo.peerAddr[4]
                            || peerAddr[5]  != peerInfo.peerAddr[5])
                    {
                        // not match, not init connect
                        bWlRlCheckOk = FALSE;
                    }
                }
                // 2. connect to  address in whitelist only
                else if (initInfo.wlPolicy == LL_INIT_WL_POLICY_USE_WHITE_LIST &&
                         bWlRlCheckOk == TRUE)
                {
                    // if advA in whitelist list, connect
                    // check white list
                    bWlRlCheckOk = ll_isAddrInWhiteList(txAdd, peerAddr);
                }

                if (bWlRlCheckOk == TRUE)
                {
                    g_same_rf_channel_flag = TRUE;
                    // ===============  TODO: construct AUX_CONN_REQ PDU
                    // fill connect interval latency timeout
                    llSetupAuxConnectReqPDU();

                    // channel selection algorithm decision
                    if ((pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
                            && chSel == LL_CHN_SEL_ALGORITHM_2)
                    {
                        conn_param[initInfo.connId].channel_selection = LL_CHN_SEL_ALGORITHM_2;
                        SET_BITS(g_tx_adv_buf.txheader, LL_CHN_SEL_ALGORITHM_2, CHSEL_SHIFT, CHSEL_MASK);
                    }
                    else
                    {
                        conn_param[initInfo.connId].channel_selection = LL_CHN_SEL_ALGORITHM_1;
                        SET_BITS(g_tx_adv_buf.txheader, LL_CHN_SEL_ALGORITHM_1, CHSEL_SHIFT, CHSEL_MASK);
                    }

                    // calculate initA if using RPA list, otherwise copy the address stored in initInfo
                    if (rpaListIndex < LL_RESOLVINGLIST_ENTRY_NUM &&
                            !ll_isIrkAllZero(g_llResolvinglist[rpaListIndex].localIrk) &&
                            (initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC || initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM))
                    {
                        // for resolving private address case, calculate the scanA with Local IRK
                        ll_CalcRandomAddr(g_llResolvinglist[rpaListIndex].localIrk, &g_tx_adv_buf.data[0]);
                        SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
//                           osal_memcpy( &g_currentLocalRpa[0],  &g_tx_adv_buf.data[0], 6);
                        g_currentLocalAddrType = LL_DEV_ADDR_TYPE_RPA_RANDOM;        // not accute local type, for branch selection in enh conn complete event
                    }
                    else
                    {
                        if (initInfo.ownAddrType == LL_DEV_ADDR_TYPE_PUBLIC || initInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC)
                        {
                            osal_memcpy((uint8*)&g_tx_adv_buf.data[0], &ownPublicAddr[0], 6);
                            SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_PUBLIC, TX_ADD_SHIFT, TX_ADD_MASK);
                        }
                        else
                        {
                            osal_memcpy((uint8*)&g_tx_adv_buf.data[0], &ownRandomAddr[0], 6);
                            SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
                        }

                        g_currentLocalAddrType = LL_DEV_ADDR_TYPE_RANDOM;             // not accute local type, for branch selection in enh conn complete event
                    }

                    // send conn req
                    T2 = read_current_fine_time();
                    delay = (T2 > ISR_entry_time) ? (T2 - ISR_entry_time) : (BASE_TIME_UNITS - ISR_entry_time + T2);

                    if (delay > 118 - pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY] - pGlobal_config[LL_HW_PLL_DELAY])   // not enough time
                    {
                        // not enough time to send conn req, store the RPA
                        isPeerRpaStore = TRUE;
                        storeRpaListIndex = rpaListIndex;
                        osal_memcpy(&currentPeerRpa[0], &g_rx_adv_buf.data[0], 6);
//                          LOG("store %d\n", storeRpaListIndex);
                        g_same_rf_channel_flag = FALSE;
                        //LOG("<%d>", delay);
                    }
                    else
                    {
                        delay = 118 - delay - pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY];
                        ll_hw_set_trx_settle(delay,                               // set BB delay, about 80us in 16MHz HCLK
                                             pGlobal_config[LL_HW_AFE_DELAY],
                                             pGlobal_config[LL_HW_PLL_DELAY]);        //RxAFE,PLL
                        // reset Rx/Tx FIFO
                        ll_hw_rst_rfifo();
                        ll_hw_rst_tfifo();
                        // send conn req
                        ll_hw_set_stx();             // set LL HW as single Tx mode
                        ll_hw_go();
                        llWaitingIrq = TRUE;
                        // AdvA, offset 6
                        osal_memcpy((uint8*)&g_tx_adv_buf.data[6], &g_rx_adv_buf.data[0], 6);
                        //write Tx FIFO
                        ll_hw_write_tfifo((uint8*) & (g_tx_adv_buf.txheader),
                                          ((g_tx_adv_buf.txheader & 0xff00) >> 8) + 2);   // payload length + header length(2)

                        if (g_currentPeerAddrType >= 0x02)
                        {
                            osal_memcpy(&g_currentPeerRpa[0], &g_rx_adv_buf.data[0], 6);
                        }

                        if (g_currentLocalAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
                        {
                            osal_memcpy(&g_currentLocalRpa[0],  &g_tx_adv_buf.data[0], 6);
                        }

                        move_to_master_function1();
                        isPeerRpaStore = FALSE;
                        bConnecting = TRUE;
                        g_same_rf_channel_flag = FALSE;
                        extInitInfo.scanMode = LL_SCAN_STOP;
                        (void)osal_set_event(LL_TaskID, LL_EVT_MASTER_CONN_CREATED);
                    }
                }
            }
            else
            {
            }
        }

        // scan again if not start connect
        if (!bConnecting)           // if not waiting for scan rsp, schedule next scan
        {
            if (extInitInfo.scanMode == LL_SCAN_STOP)
            {
                // scan has been stopped
                llState = LL_STATE_IDLE;                                                 // for single connection case, set the LL state idle
                //  release the associated allocated connection
                llReleaseConnId(connPtr);                                                // new for multi-connection
                g_ll_conn_ctx.numLLMasterConns --;
                (void)osal_set_event( LL_TaskID, LL_EVT_MASTER_CONN_CANCELLED );         // inform high layer
            }
            else
            {
                // not sending SCAN REQ, update scan time
                llScanTime += ((ISR_entry_time > llScanT1) ? (ISR_entry_time - llScanT1) : (BASE_TIME_UNITS - llScanT1 + ISR_entry_time));

                //if advmode=0x01 ,with auxptr ,at primarychannel ：retrive auxptr
                //else scan at primary channel
                if((adv_mode== LL_EXT_ADV_MODE_CONN) &&
                        (ext_adv_hdr.header & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)&&
                        (extInitInfo.current_chn>=LL_SCAN_ADV_CHAN_37))
                {
                    uint32    wait_time, pkt_time;

                    // 2021-1-7, take out PDU time from AuxPtr offset
                    if (extInitInfo.current_scan_PHY == PKT_FMT_BLE1M)
                        pkt_time = packet_len * 8 * 4;
                    else if (extInitInfo.current_scan_PHY == PKT_FMT_BLE2M)
                        pkt_time = packet_len * 8 * 2;           // no this case?
                    else // coded PHY, S = 8
                        pkt_time = packet_len * 8 * 4 * 8;

                    extInitInfo.current_chn = ext_adv_hdr.auxPtr.chn_idx;
                    extInitInfo.current_scan_PHY = ext_adv_hdr.auxPtr.aux_phy;
                    wait_time = ext_adv_hdr.auxPtr.aux_offset * ((ext_adv_hdr.auxPtr.offset_unit == 1) ? 300 : 30);
                    wait_time -= pkt_time;
//                      wait_time -= (ext_adv_hdr.auxPtr.ca == 0) ? 500 : 499;     // temporary setting, consider clock accuracy

                    if (wait_time <= 1000)
                        wait_time = 10;
                    else
                        wait_time -= 990;     // scan advance

                    llScanTime = 0;
                    ll_ext_init_schedule_next_event(wait_time);
                }
                // case 2: scanning aux channel, no more aux PDU, continue to scan in primary channel
                else if (extInitInfo.current_chn < LL_ADV_CHAN_FIRST)
                {
                    extInitInfo.current_chn = LL_SCAN_ADV_CHAN_37;

                    if (extInitInfo.numOfScanPHY > 1)
                    {
                        extInitInfo.current_index = (extInitInfo.current_index + 1) & 0x01;
                    }

                    extInitInfo.current_scan_PHY = extInitInfo.initPHYs[extInitInfo.current_index];
                    llSetupExtInit();
                    llScanTime = 0;
                }
                // case 3: update scan time, only when scan primary channel and not receive AuxPtr
                else
                {
                    if (llScanTime >= extInitInfo.scanWindow[extInitInfo.current_index] * 625)
                    {
                        if (extInitInfo.numOfScanPHY > 1 && extInitInfo.current_chn == LL_SCAN_ADV_CHAN_39)
                        {
                            extInitInfo.current_index = (extInitInfo.current_index + 1) & 0x01;
                            extInitInfo.current_scan_PHY = extInitInfo.initPHYs[extInitInfo.current_index];
                        }

                        LL_CALC_NEXT_SCAN_CHN(extInitInfo.current_chn);

                        // schedule next scan event
                        if (extInitInfo.scanWindow[extInitInfo.current_index] == extInitInfo.scanInterval[extInitInfo.current_index])      // scanWindow == scanInterval, trigger immediately
                            llSetupExtInit();
                        else
                            ll_ext_init_schedule_next_event((extInitInfo.scanInterval[extInitInfo.current_index]
                                                             - extInitInfo.scanWindow[extInitInfo.current_index]) * 625);

                        // reset scan total time
                        llScanTime = 0;
                    }
                    else
                    {
//                        extInitInfo.current_scan_PHY = extInitInfo.initPHYs[extInitInfo.current_index];
                        llSetupExtInit();
                    }
                }
            }
        }
    }
    else if (ll_mode == LL_HW_MODE_TRX)            // init case, waiting for AUX_CONNECT_RSP
    {
//=============
        uint8_t  packet_len, pdu_type;//, txAdd;
        uint16_t pktLen;
        uint32_t pktFoot0, pktFoot1;
        uint8    cancelConnect = FALSE;
        ll_debug_output(DEBUG_LL_HW_TRX);

        if ((irq_status & LIRQ_RD) && (irq_status & LIRQ_COK))
        {
            // read packet
            packet_len = ll_hw_read_rfifo1((uint8_t*)(&(g_rx_adv_buf.rxheader)),
                                           &pktLen,
                                           &pktFoot0,
                                           &pktFoot1);

            if(ll_hw_get_rfifo_depth() > 0)
            {
                g_pmCounters.ll_rfifo_read_err++;
                packet_len=0;
                pktLen=0;
            }

            // check receive pdu type
            pdu_type = g_rx_adv_buf.rxheader & PDU_TYPE_MASK;
//        txAdd    = (g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT;    // adv PDU header, bit 6: TxAdd, 0 - public, 1 - random

            if (packet_len > 0                       // any better checking rule for rx anything?
                    && pdu_type == ADV_AUX_CONN_RSP)
            {
//            g_pmCounters.ll_recv_scan_req_cnt ++;

                // check AdvA
                if (g_rx_adv_buf.data[2]  != ext_adv_hdr.advA[0]      // note: ext_adv_hdr info is parsed in previous AUX_ADV_IND(AdvA is 'M' field) process ISR
                        || g_rx_adv_buf.data[3] != ext_adv_hdr.advA[1]
                        || g_rx_adv_buf.data[4] != ext_adv_hdr.advA[2]
                        || g_rx_adv_buf.data[5] != ext_adv_hdr.advA[3]
                        || g_rx_adv_buf.data[6] != ext_adv_hdr.advA[4]
                        || g_rx_adv_buf.data[7] != ext_adv_hdr.advA[5])
                {
                    // receive err response, cancel the connection
                    cancelConnect = TRUE;
                }
            }
        }
        else
        {
            cancelConnect = TRUE;
        }

        if (cancelConnect == TRUE)
        {
            // TODO
            // receive error connect rsp or timeout, cancel connection
            if (extInitInfo.scanMode == LL_SCAN_STOP)
            {
                // scan has been stopped
                llState = LL_STATE_IDLE;                                                 // for single connection case, set the LL state idle
                //  release the associated allocated connection
                ll_deleteTask(extInitInfo.connId);
                g_ll_conn_ctx.currentConn = LL_INVALID_CONNECTION_ID;
                llReleaseConnId(connPtr);                                                // new for multi-connection
                g_ll_conn_ctx.numLLMasterConns --;
                (void)osal_set_event( LL_TaskID, LL_EVT_MASTER_CONN_CANCELLED );         // inform high layer
            }
            else
            {
                llState = LL_STATE_IDLE;                                                 // for single connection case, set the LL state idle
                ll_deleteTask(extInitInfo.connId);
                g_ll_conn_ctx.currentConn = LL_INVALID_CONNECTION_ID;
                extInitInfo.current_chn = LL_SCAN_ADV_CHAN_37;

                if (extInitInfo.numOfScanPHY > 1)
                {
                    extInitInfo.current_index = (extInitInfo.current_index + 1) & 0x01;
                }

                extInitInfo.current_scan_PHY = extInitInfo.initPHYs[extInitInfo.current_index];
                llSetupExtInit();
                llScanTime = 0;
            }
        }
        else     // received aux_conn_rsp, report connect created event
        {
            extInitInfo.scanMode = LL_SCAN_STOP;
            // LOG("CONN\n");
            (void)osal_set_event( LL_TaskID, LL_EVT_MASTER_CONN_CREATED );
        }

//===========
    }

    if (!llWaitingIrq)
    {
        ll_hw_clr_irq();
        llTaskState = LL_TASK_OTHERS;
    }

    HAL_EXIT_CRITICAL_SECTION();
    return TRUE;
}

extern uint8_t             llSecondaryState;
void LL_IRQHandler2(void)
{
    uint32         irq_status;
    int8 ret;
    ISR_entry_time = read_current_fine_time();
    //*(volatile uint32_t *)0x4000f0b8 = 1;  // pclk_clk_gate_en
    ll_debug_output(DEBUG_ISR_ENTRY);
    irq_status = ll_hw_get_irq_status();

    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        return;
    }

    llWaitingIrq = FALSE;

    if (llTaskState == LL_TASK_EXTENDED_ADV)
    {
        ret = ll_processExtAdvIRQ1(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_EXTENDED_SCAN)
    {
        ret = ll_processExtScanIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_EXTENDED_INIT)
    {
        ret = ll_processExtInitIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_PERIODIC_ADV)
    {
        ret = ll_processPrdAdvIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_PERIODIC_SCAN)
    {
        ret = ll_processPrdScanIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else
    {
        ret = ll_processBasicIRQ(irq_status);
    }

    // ================ Post ISR process: secondary pending state process
    // conn-adv case 2: other ISR, there is pending secondary advertise event, make it happen
    if (llSecondaryState == LL_SEC_STATE_ADV_PENDING)
    {
        if (llSecAdvAllow())    // for multi-connection case, it is possible still no enough time for adv
        {
            llSetupSecAdvEvt();
            llSecondaryState = LL_SEC_STATE_ADV;
        }
    }
    // there is pending scan event, make it happen, note that it may stay pending if there is no enough idle time
    else if (llSecondaryState == LL_SEC_STATE_SCAN_PENDING)
    {
        // trigger scan
        llSetupSecScan(scanInfo.nextScanChan);
    }
    // there is pending init event, make it happen, note that it may stay pending if there is no enough idle time
    else if (llSecondaryState == LL_SEC_STATE_INIT_PENDING)
    {
        // trigger scan
        llSetupSecInit(initInfo.nextScanChan);
    }

    ll_debug_output(DEBUG_ISR_EXIT);
}

void LL_IRQHandler3(void)
{
    uint32         irq_status;
    int8 ret;
    ISR_entry_time = read_current_fine_time();
    //*(volatile uint32_t *)0x4000f0b8 = 1;  // pclk_clk_gate_en
    ll_debug_output(DEBUG_ISR_ENTRY);
    irq_status = ll_hw_get_irq_status();

    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        return;
    }

    llWaitingIrq = FALSE;

    if (llTaskState == LL_TASK_EXTENDED_ADV)
    {
        //gpio_write(P32,1);
        ret = ll_processExtAdvIRQ(irq_status);

        //gpio_write(P32,0);
        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_EXTENDED_SCAN)
    {
        // gpio_write(P32,1);
        ret = ll_processExtScanIRQ1(irq_status);

        // gpio_write(P32,0);
        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_EXTENDED_INIT)
    {
        //gpio_write(P33,1);
        ret = ll_processExtInitIRQ1(irq_status);

        //gpio_write(P33,0);
        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_PERIODIC_ADV)
    {
        ret = ll_processPrdAdvIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else if (llTaskState == LL_TASK_PERIODIC_SCAN)
    {
        ret = ll_processPrdScanIRQ(irq_status);

        // TODO: consider whether need process secondary adv/scan here
        if (ret == TRUE)
            return;
    }
    else
    {
        ret = ll_processBasicIRQ(irq_status);
    }

    // ================ Post ISR process: secondary pending state process
    // conn-adv case 2: other ISR, there is pending secondary advertise event, make it happen
    if (llSecondaryState == LL_SEC_STATE_ADV_PENDING)
    {
        if (llSecAdvAllow())    // for multi-connection case, it is possible still no enough time for adv
        {
            llSetupSecAdvEvt();
            llSecondaryState = LL_SEC_STATE_ADV;
        }
    }
    // there is pending scan event, make it happen, note that it may stay pending if there is no enough idle time
    else if (llSecondaryState == LL_SEC_STATE_SCAN_PENDING)
    {
        // trigger scan
        llSetupSecScan(scanInfo.nextScanChan);
    }
    // there is pending init event, make it happen, note that it may stay pending if there is no enough idle time
    else if (llSecondaryState == LL_SEC_STATE_INIT_PENDING)
    {
        // trigger scan
        llSetupSecInit(initInfo.nextScanChan);
    }

    ll_debug_output(DEBUG_ISR_EXIT);
}

// adv scheduler - conn scheduler interaction functions
void ll_adv_scheduler0(void)
{
    uint32  T2, T3, delta;
    uint32  minAuxPduTime, minPriPduTime;
    uint8   minIndexAux, minIndexPri;
    uint8   done = FALSE;
    int i;
    llAdvScheduleInfo_t* p_scheduler = NULL;
    extAdvInfo_t*  pAdvInfo = NULL;
    // calculate elapse time since last timer trigger
    T2 = read_current_fine_time();
    delta = LL_TIME_DELTA(g_timerExpiryTick, T2);

    // 12-15, if the ext adv has been disable, not schedule any more
    if (g_currentExtAdv == LL_INVALID_ADV_SET_HANDLE)
        return;

    p_scheduler = &g_pAdvSchInfo[g_currentExtAdv];
    pAdvInfo = p_scheduler->pAdvInfo;

    // 12-15, check adv active status first
    // if current adv is not active, delete it from task list
    if (pAdvInfo->active == FALSE)
    {
        ll_delete_adv_task(g_currentExtAdv);
        return;
    }

    // the advertisement scheduler will schedule next task according to current adv's next advertise channel
    // case 1: not finish primary ADV channel or AUX_XXX_IND chain, continue advertise current adv task
    if (pAdvInfo->currentChn > LL_ADV_CHAN_FIRST &&   // next channel in primary adv channel
            pAdvInfo->active == TRUE                 &&   // add 04-01, legacy adv may stop when receive conn_req
            !ll_isFirstAdvChn(pAdvInfo->parameter.priAdvChnMap, pAdvInfo->currentChn))       // not the 1st primary ADV channel
    {
        ll_ext_adv_schedule_next_event(pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT] - delta);
        done = TRUE;
    }
    else if (pAdvInfo->currentChn < LL_ADV_CHAN_FIRST    // broadcast in aux adv chn
             &&  pAdvInfo->currentAdvOffset > 0)              // offset in adv data set greater than 0 means next PDU is AUX_CHAIN_IND PDU
    {
        // g_interAuxPduDuration is the time between the edge of AUX_ADV_IND & AUX_CHAIN_IND,
        // delta consider the time between timer expiry time and new timer trigger point,
        // the time from TO to ll_hw_trigger is not consider, this time denote as alpha below.
        // If alpha for AUX_ADV_IND and AUX_CHAIN_IND is the same, then no compensation is required
        // but if the alpha for AUX_ADV_IND and AUX_CHAIN_IND is different, using pGlobal_config[LL_EXT_ADV_INTER_SEC_COMP]
        // to compensate it
        if (extscanrsp_offset ==100)//first auxchain of auxscanrsp
            ll_ext_adv_schedule_next_event(g_interAuxPduDuration - delta - 125);//125:compensate for TIM4IRQ->setup auxchain
        else
            ll_ext_adv_schedule_next_event(g_interAuxPduDuration - delta);

        done = TRUE;
    }

    // update scheduler task list
    ll_updateExtAdvRemainderTime(delta);

    if (done)               // next task schedule done
        return;

    // case 2: finish broadcast in primary adv channel/aux adv channel. Check ext adv scheduler to get the earliest task
    // search the nearest expiry task and PDU type
    minAuxPduTime = g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder;
    minPriPduTime = g_pAdvSchInfo[g_currentExtAdv].nextEventRemainder;
    minIndexAux = g_currentExtAdv;
    minIndexPri = g_currentExtAdv;

    for (i = 0; i < g_extAdvNumber; i++)
    {
        if (g_pAdvSchInfo[i].adv_handler != LL_INVALID_ADV_SET_HANDLE)
        {
            if (g_pAdvSchInfo[i].auxPduRemainder < minAuxPduTime)
            {
                minIndexAux = i;
                minAuxPduTime = g_pAdvSchInfo[i].auxPduRemainder;
            }

            if (g_pAdvSchInfo[i].nextEventRemainder < minPriPduTime)
            {
                minIndexPri = i;
                minPriPduTime = g_pAdvSchInfo[i].nextEventRemainder;
            }
        }
    }

    T3 = read_current_fine_time();
    delta = LL_TIME_DELTA(T2, T3);
    // compare the minimum AUX channel remainder time & minimum Primary channel PDU remainder time,
    // AUX channel PDU could add some pre-emphesis here
    uint32  auxPduEmphesis = pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT] * 3;                           // add 03-31

    if (minAuxPduTime != LL_INVALID_TIME
            && (minAuxPduTime < minPriPduTime + auxPduEmphesis))   // next schedule task is aux PDU
    {
        ll_ext_adv_schedule_next_event(minAuxPduTime - delta);
        g_currentExtAdv   = minIndexAux;
    }
    else   // next schedule task is pri PDU
    {
        ll_ext_adv_schedule_next_event(minPriPduTime - delta);
        g_currentExtAdv   = minIndexPri;
    }

    // update scheduler task list
    ll_updateExtAdvRemainderTime(delta);
    //LOG("<%d>\n", g_pAdvSchInfo[0].nextEventRemainder - g_pAdvSchInfo[0].auxPduRemainder);
}

void ll_add_adv_task0(extAdvInfo_t* pExtAdv)
{
    int i, spare;
    llAdvScheduleInfo_t* p_scheduler = NULL, *p_current_scheduler;
    uint32  T1 = 0, T2, delta, remainder, elapse_time = 0;
    uint8   chanNumber, temp;
    uint8   isLegacy = ll_isLegacyAdv(pExtAdv);
    temp = pExtAdv->parameter.priAdvChnMap;
    chanNumber = (temp & 0x01) + ((temp & 0x02) >> 1) + ((temp & 0x04) >> 2);
    // init new Ext adv event context
    pExtAdv->currentChn = ((temp & ((~temp) + 1)) >> 1) + 37;       // calculate 1st adv channel
    pExtAdv->adv_event_counter = 0;
    pExtAdv->currentAdvOffset = 0;
    pExtAdv->adv_event_duration = 0;

    // ======== case 1: the 1st adv task
    if (g_currentExtAdv == LL_INVALID_ADV_SET_HANDLE)     //    if (!isTimer4Running())
    {
        g_schExtAdvNum = 1;
        g_currentExtAdv = 0;          // scheduler index = 0
        p_current_scheduler = &g_pAdvSchInfo[g_currentExtAdv];

        // add 03-31
        if (isLegacy
                || (pExtAdv->data.advertisingDataLength == 0
                    && !(pExtAdv->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK)
                    && !(pExtAdv->parameter.advEventProperties & LE_ADV_PROP_CONN_BITMASK)))    // adv data length == 0, no aux PDU is required. to test
        {
            g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder = LL_INVALID_TIME;
        }
        else
            ll_allocAuxAdvTimeSlot(g_currentExtAdv);

//      LOG("[%d]  ", g_pAdvSchInfo[i].auxPduRemainder);

        // extended adv & connect task could co-exist
        if (llWaitingIrq)    // 1. conn IRQ ongoing
        {
            g_currentAdvTimer = pGlobal_config[LL_CONN_TASK_DURATION];
        }
        else if (isTimer1Running()   // 2. conn task scheduled
                 && ((remainder = read_LL_remainder_time()) < pGlobal_config[LL_EXT_ADV_TASK_DURATION]))     // timer1 for connection or legacy adv
        {
            g_currentAdvTimer =  remainder + pGlobal_config[LL_CONN_TASK_DURATION];   // no enough time for adv case
        }
        else                         // 3. no conn task
        {
            if (chanNumber > 1)
                g_currentAdvTimer = pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT];
            else
                g_currentAdvTimer = (p_current_scheduler->auxPduRemainder < p_current_scheduler->pAdvInfo->primary_advertising_interval) ?
                                    p_current_scheduler->auxPduRemainder : p_current_scheduler->pAdvInfo->primary_advertising_interval;

            g_timerExpiryTick = read_current_fine_time();                 // fake timer expiry tick
            // invoke set up ext adv function
            llSetupExtAdvEvent(pExtAdv);
        }

// 2021-3-12
//      ll_ext_adv_schedule_next_event(g_currentAdvTimer);
//      g_timerExpiryTick = read_current_fine_time();                 // fake timer expiry tick
        p_current_scheduler->pAdvInfo = pExtAdv;
        p_current_scheduler->adv_handler = pExtAdv->advHandle;
        p_current_scheduler->nextEventRemainder = p_current_scheduler->pAdvInfo->primary_advertising_interval;  // add some random delay between 0-10ms?
        pExtAdv->active = TRUE;
        return;
    }

    // ===== case 2: there are ongoing adv
    p_current_scheduler = &g_pAdvSchInfo[g_currentExtAdv];

    // get the 1st spare scheduler slot
    for (i = 0; i < g_extAdvNumber; i++)    // bug fixed 04-07
    {
        if (g_pAdvSchInfo[i].adv_handler == LL_INVALID_ADV_SET_HANDLE)
        {
            p_scheduler = &g_pAdvSchInfo[i];
            spare = i;
            break;
        }
    }

    // no empty scheduler slot, return
    if (p_scheduler == NULL)
        return;

    g_schExtAdvNum ++;
    pExtAdv->active = TRUE;

    // arrange the timing of AUX_XXX_IND, it is independent to EXT_ADV_IND
    // add 03-31
    if (isLegacy || pExtAdv->data.advertisingDataLength == 0)    // adv data length == 0, no aux PDU is required. to test
    {
        g_pAdvSchInfo[spare].auxPduRemainder = LL_INVALID_TIME;
    }
    else
        ll_allocAuxAdvTimeSlot(spare);

    if (isTimer4Running())
    {
        remainder = read_ll_adv_remainder_time();
        T1   = read_current_fine_time();
        // calculate the elapse time since current adv timer trigger
        elapse_time = g_currentAdvTimer - remainder;
    }
    else
        remainder = 0;

    if (isTimer1Running())
    {
        uint32 temp = read_LL_remainder_time();
        remainder = (temp < remainder) ? temp : remainder;
    }

    if (llWaitingIrq)           // there is ongoing LL IRQ, not setup adv
        remainder = 0;

    // case 2.1: there is enough time for EXT_ADV_IND, setup new adv first
    if (0)//remainder > pGlobal_config[LL_EXT_ADV_TASK_DURATION])
    {
        g_currentExtAdv = spare;

        if (chanNumber > 1)
            g_currentAdvTimer = pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT];

        ll_ext_adv_schedule_next_event(g_currentAdvTimer);
        // setup ext adv event
        llSetupExtAdvEvent(pExtAdv);
        T2 = read_current_fine_time();
        delta = LL_TIME_DELTA(T1, T2);
        // A BUG here??? should be: p_scheduler->nextEventRemainder = ....
//      p_current_scheduler->nextEventRemainder = p_current_scheduler->pAdvInfo->primary_advertising_interval;  // add some random delay between 0-10ms?
        p_scheduler->nextEventRemainder = p_scheduler->pAdvInfo->primary_advertising_interval;

        // update the timer in the scheduler info list
        for (i = 0; i < g_extAdvNumber; i++)
        {
            if (g_pAdvSchInfo[i].adv_handler != LL_INVALID_ADV_SET_HANDLE &&
                    i != g_currentExtAdv)
            {
                if (g_pAdvSchInfo[i].nextEventRemainder < (elapse_time + delta))
                {
                    g_pAdvSchInfo[i].nextEventRemainder += g_pAdvSchInfo[i].pAdvInfo->primary_advertising_interval;
                    g_pAdvSchInfo[i].pAdvInfo->adv_event_duration += g_pAdvSchInfo[i].pAdvInfo->primary_advertising_interval;
                    g_pAdvSchInfo[i].pAdvInfo->adv_event_counter ++;
                }

                if (g_pAdvSchInfo[i].auxPduRemainder < (elapse_time + delta))           // normally this case should not occur
                    g_pAdvSchInfo[i].auxPduRemainder += g_advSlotPeriodic;

                g_pAdvSchInfo[i].nextEventRemainder -= (elapse_time + delta);

                if (g_pAdvSchInfo[i].auxPduRemainder != LL_INVALID_TIME)
                    g_pAdvSchInfo[i].auxPduRemainder    -= (elapse_time + delta);
            }
        }
    }
    // case 2.2: no enough time, start new adv after current one
    else
    {
        // add new adv to adv scheduler list, not change current adv task
//      p_scheduler->nextEventRemainder = p_current_scheduler->nextEventRemainder
//                                        + (spare > g_currentExtAdv ? (spare - g_currentExtAdv) : (g_extAdvNumber + spare - g_currentExtAdv)) * pGlobal_config[LL_EXT_ADV_TASK_DURATION];
        p_scheduler->nextEventRemainder = p_current_scheduler->nextEventRemainder
                                          + (spare > g_currentExtAdv ? (spare - g_currentExtAdv) : (g_extAdvNumber + spare - g_currentExtAdv)) * (g_advSlotPeriodic >> 2);
    }

    p_scheduler->adv_handler = pExtAdv->advHandle;
    p_scheduler->pAdvInfo    = pExtAdv;
}

// TODO: function split between ll_adv_scheduler() & ll_delete_adv_task
void ll_delete_adv_task0(uint8 index)
{
    uint32  T1, T2, delta, remainder, elapse_time;
    uint32  minAuxPduTime, minPriPduTime;
    uint8   minIndexAux, minIndexPri;
    int i;
    g_pAdvSchInfo[index].adv_handler        = LL_INVALID_ADV_SET_HANDLE;
    g_pAdvSchInfo[index].pAdvInfo           = NULL;
    g_pAdvSchInfo[index].auxPduRemainder    = LL_INVALID_TIME;
    g_pAdvSchInfo[index].nextEventRemainder = LL_INVALID_TIME;
    g_schExtAdvNum --;

    // only 1 task case, clear scheduler info and stop timer
    if (g_schExtAdvNum == 0)
    {
        g_currentExtAdv = LL_INVALID_ADV_SET_HANDLE;
        clear_timer(AP_TIM4);
        return;
    }

    // current awaiting adv is disable, and there are more than 1 task
    if (index == g_currentExtAdv
            && isTimer4Running())   // 2020-7-7
    {
        remainder = read_ll_adv_remainder_time();
        T1  = read_current_fine_time();
        elapse_time = g_currentAdvTimer - remainder;
        // find the earliest task
        minAuxPduTime = g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder;      // LL_INVALID_TIME
        minPriPduTime = g_pAdvSchInfo[g_currentExtAdv].nextEventRemainder;
        minIndexAux = g_currentExtAdv;
        minIndexPri = g_currentExtAdv;

        for (i = 0; i < g_extAdvNumber; i++)
        {
            if (g_pAdvSchInfo[i].adv_handler != LL_INVALID_ADV_SET_HANDLE)
            {
                if (g_pAdvSchInfo[i].auxPduRemainder < minAuxPduTime)
                {
                    minIndexAux = i;
                    minAuxPduTime = g_pAdvSchInfo[i].auxPduRemainder;
                }

                if (g_pAdvSchInfo[i].nextEventRemainder < minPriPduTime)
                {
                    minIndexPri = i;
                    minPriPduTime = g_pAdvSchInfo[i].nextEventRemainder;
                }
            }
        }

        // start new timer
        T2  = read_current_fine_time();
        delta = LL_TIME_DELTA(T1, T2);

        if (minAuxPduTime < minPriPduTime)   // next schedule task is aux PDU
        {
            ll_ext_adv_schedule_next_event(minAuxPduTime - elapse_time - delta);
//          g_currentAdvTimer = minAuxPduTime - elapse_time - delta;
            g_currentExtAdv   = minIndexAux;
        }
        else   // next schedule task is pri PDU
        {
            ll_ext_adv_schedule_next_event(minPriPduTime - elapse_time - delta);
//          g_currentAdvTimer = minPriPduTime - elapse_time - delta;
            g_currentExtAdv   = minIndexPri;
        }

        // update the scheduler list
        ll_updateExtAdvRemainderTime(elapse_time + delta);
    }
}

void llSetupExtScan( uint8 chan )
{
    // gpio_write(P33,1);
    // gpio_write(P33,0);
    //LOG("S %d ", chan);
    // Hold off interrupts.
    HAL_ENTER_CRITICAL_SECTION( );
    //support rf phy change
    g_rfPhyPktFmt = extScanInfo.current_scan_PHY;
    rf_phy_change_cfg0(extScanInfo.current_scan_PHY);
    // reset all FIFOs; all data is forfeit
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    set_crc_seed(ADV_CRC_INIT_VALUE); // crc seed for adv is same for all channels
    set_access_address(ADV_SYNCH_WORD);
    set_channel(chan);
    set_whiten_seed(chan);
    set_max_length(0xff);
    ll_hw_set_rx_timeout(50000);     // us
    ll_hw_set_srx();
    ll_hw_ign_rfifo(LL_HW_IGN_CRC|LL_HW_IGN_EMP);
    ll_hw_go();
    llScanT1 = read_current_fine_time();
    llWaitingIrq = TRUE;
    llTaskState = LL_TASK_EXTENDED_SCAN;
    HAL_EXIT_CRITICAL_SECTION();
    return;
}

// ===========================================================
//     extended adv LL HCI command process functions' patch

/*******************************************************************************
    @fn          LL_SetExtAdvSetRandomAddress

    @brief       This function is used to set random address for a advertisement set

    input parameters

    @param       adv_handle - advertisement set handler
                random_address -  random address

    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_SetExtAdvSetRandomAddress( uint8 adv_handle,
                                         uint8* random_address
                                       )
{
    int i;

    // extended advertiser memory is allocate by application, return fail if not
    if (g_pExtendedAdvInfo == NULL)
        return LL_STATUS_ERROR_OUT_OF_HEAP;

    // search advertisement handler list
    for (i = 0; i < g_extAdvNumber; i ++)
    {
        if (g_pExtendedAdvInfo[i].advHandle == adv_handle)
            break;
    }

    // if not found, then adv parameter has not been set
    if (i == g_extAdvNumber)
        return LL_STATUS_ERROR_UNKNOWN_ADV_ID;

    g_pExtendedAdvInfo[i].parameter.isOwnRandomAddressSet = TRUE;
    memcpy(&g_pExtendedAdvInfo[i].parameter.ownRandomAddress[0], random_address, LL_DEVICE_ADDR_LEN);
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_SetExtAdvParam

    @brief       This function is used to set extend advertiser parameters

    input parameters

    @param       adv_handle - advertisement set handler


    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_SetExtAdvParam( uint8 adv_handle,
                              uint16 adv_event_properties,
                              uint32 primary_advertising_interval_Min,          // 3 octets
                              uint32 primary_advertising_interval_Max,          // 3 octets
                              uint8  primary_advertising_channel_map,
                              uint8  own_address_type,
                              uint8  peer_address_type,
                              uint8* peer_address,
                              uint8  advertising_filter_policy,
                              int8   advertising_tx_power,
                              uint8  primary_advertising_PHY,
                              uint8  secondary_advertising_max_skip,
                              uint8  secondary_advertising_PHY,
                              uint8  advertising_SID,
                              uint8  scan_request_notification_enable,
                              int8*  selectTxPwr
                            )
{
    int   i;

    // TODO: add parameters range checking
    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;

    if ((adv_event_properties & LE_ADV_PROP_LEGACY_BITMASK)           // Legacy Adv
            && (adv_event_properties != LL_EXT_ADV_PROP_ADV_IND)
            && (adv_event_properties != LL_EXT_ADV_PROP_ADV_LDC_ADV)
            && (adv_event_properties != LL_EXT_ADV_PROP_ADV_HDC_ADV)
            && (adv_event_properties != LL_EXT_ADV_PROP_ADV_SCAN_IND)
            && (adv_event_properties != LL_EXT_ADV_PROP_ADV_NOCONN_IND))
        return LL_STATUS_ERROR_BAD_PARAMETER;

    if (primary_advertising_interval_Max < primary_advertising_interval_Min
            || primary_advertising_interval_Min < 0x20
            || primary_advertising_interval_Max < 0x20)
        return LL_STATUS_ERROR_BAD_PARAMETER;

    if (!(adv_event_properties & LE_ADV_PROP_LEGACY_BITMASK)
            && (adv_event_properties & LE_ADV_PROP_SCAN_BITMASK)
            && (adv_event_properties & LE_ADV_PROP_CONN_BITMASK))
        return LL_STATUS_ERROR_BAD_PARAMETER;

    // comment out 2020-9-17
    // check whether aux adv pdu resource period is OK for max skip & adv interval setting
//    if (!(adv_event_properties & LE_ADV_PROP_LEGACY_BITMASK)                    // not legacy Adv
//        && primary_advertising_interval_Max * (secondary_advertising_max_skip + 1) * 625 < g_advSlotPeriodic)
//      return LL_STATUS_ERROR_BAD_PARAMETER;

    // extended advertiser memory is allocate by application, return fail if not
    if (g_pExtendedAdvInfo == NULL)
        return LL_STATUS_ERROR_OUT_OF_HEAP;

    // search advertisement handler list
    for (i = 0; i < g_extAdvNumber; i ++)
    {
        if (g_pExtendedAdvInfo[i].advHandle == adv_handle)
            break;
    }

    // if not found, search the 1st available slot
    if (i == g_extAdvNumber)
    {
        for (i = 0; i < g_extAdvNumber; i ++)
        {
            if (g_pExtendedAdvInfo[i].advHandle == LL_INVALID_ADV_SET_HANDLE)
            {
//                uint8 *scanRspData = g_pExtendedAdvInfo[i].scanRspData;
//              uint8 *advData     = g_pExtendedAdvInfo[i].data.advertisingData;
//
//                memset(&g_pExtendedAdvInfo[i], 0, sizeof(extAdvInfo_t));
//              g_pExtendedAdvInfo[i].scanRspData = scanRspData;
//              g_pExtendedAdvInfo[i].data.advertisingData = advData;
                g_pExtendedAdvInfo[i].advHandle = LL_INVALID_ADV_SET_HANDLE;
                g_pExtendedAdvInfo[i].parameter.isOwnRandomAddressSet = FALSE;
                g_pExtendedAdvInfo[i].adv_event_counter = 0;
                g_pExtendedAdvInfo[i].data.fragmentPreference = 0xFF;
                g_pExtendedAdvInfo[i].data.advertisingDataLength = 0;        // no adv data
                g_pExtendedAdvInfo[i].data.dataComplete = TRUE;
                g_pExtendedAdvInfo[i].scanRspMaxLength = 0;                  // no scan rsp data
                g_pExtendedAdvInfo[i].isPeriodic = FALSE;
                break;
            }
        }
    }

    if (i == g_extAdvNumber)
        return LL_STATUS_ERROR_OUT_OF_HEAP;

    // save the advertisement parameters
    g_pExtendedAdvInfo[i].advHandle = adv_handle;
    g_pExtendedAdvInfo[i].parameter.advEventProperties = adv_event_properties;
    g_pExtendedAdvInfo[i].parameter.priAdvIntMin       = primary_advertising_interval_Min;
    g_pExtendedAdvInfo[i].parameter.priAdvgIntMax      = primary_advertising_interval_Max;
    g_pExtendedAdvInfo[i].parameter.priAdvChnMap       = primary_advertising_channel_map;
    g_pExtendedAdvInfo[i].parameter.ownAddrType        = own_address_type;
    g_pExtendedAdvInfo[i].parameter.peerAddrType       = peer_address_type;
    memcpy(g_pExtendedAdvInfo[i].parameter.peerAddress, peer_address, LL_DEVICE_ADDR_LEN);
    g_pExtendedAdvInfo[i].parameter.wlPolicy           = advertising_filter_policy;
    g_pExtendedAdvInfo[i].parameter.advTxPower         = advertising_tx_power;
    g_pExtendedAdvInfo[i].parameter.primaryAdvPHY      = primary_advertising_PHY;
    g_pExtendedAdvInfo[i].parameter.secondaryAdvPHY    = secondary_advertising_PHY;
    g_pExtendedAdvInfo[i].parameter.secondaryAdvMaxSkip= secondary_advertising_max_skip;
    g_pExtendedAdvInfo[i].parameter.advertisingSID     = advertising_SID;
    g_pExtendedAdvInfo[i].parameter.scanReqNotificationEnable  = scan_request_notification_enable;
    memset(&g_currentPeerRpa[0],  0, 6);
    adv_param.ownAddrType = own_address_type;           // for RPA timeout process

    if (adv_event_properties & LE_ADV_PROP_DIRECT_BITMASK)  // for RPA timeout process
        adv_param.advEvtType = LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;

    // if Resloved list exist, calculate own RPA address
    if (g_llRlDeviceNum > 0)
    {
        uint8 resolve_address[6];
        uint8* localIrk, *peerIrk;

        if ( own_address_type == LL_DEV_ADDR_TYPE_PUBLIC )
        {
            // get our address and address type
            g_currentLocalAddrType = LL_DEV_ADDR_TYPE_PUBLIC;
            LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownPublicAddr );
        }
        else if ( own_address_type == LL_DEV_ADDR_TYPE_RANDOM )
        {
            // get our address and address type
            g_currentLocalAddrType  = LL_DEV_ADDR_TYPE_RANDOM;
            LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownRandomAddr );
        }
        else if ( own_address_type == LL_DEV_ADDR_TYPE_RPA_PUBLIC ||
                  own_address_type == LL_DEV_ADDR_TYPE_RPA_RANDOM)
        {
            uint8 found = FALSE;

            // search the resolving list to get local IRK
            if (ll_readLocalIRK(&localIrk, peer_address, peer_address_type) == TRUE)
            {
                if (!ll_isIrkAllZero(localIrk))            // for all-zero local IRK, not RPA used
                {
                    if (ll_CalcRandomAddr(localIrk, resolve_address) == SUCCESS)
                    {
                        LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, resolve_address );
                        osal_memcpy( &g_currentLocalRpa[0],  resolve_address, 6);
                        found = TRUE;
                        g_currentLocalAddrType = LL_DEV_ADDR_TYPE_RPA_RANDOM;
                    }
                }
            }

            if (found == FALSE)
            {
                if (own_address_type == LL_DEV_ADDR_TYPE_RPA_PUBLIC)
                {
                    LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownPublicAddr );
                    g_currentLocalAddrType = LL_DEV_ADDR_TYPE_PUBLIC;
                }
                else
                {
                    LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownRandomAddr );
                    g_currentLocalAddrType  = LL_DEV_ADDR_TYPE_RANDOM;
                }
            }

            LL_COPY_DEV_ADDR_LE( peerInfo.peerAddr, peer_address );
            peerInfo.peerAddrType = peer_address_type;
        }

        // calculate target Address for direct Adv
        // TO consider how to save and used
        if(adv_event_properties & LE_ADV_PROP_DIRECT_BITMASK)
        {
            //          uint8 useRpa = FALSE;

            // search the resolving list to get local IRK
            if (ll_readPeerIRK(&peerIrk, peer_address, peer_address_type) == TRUE)
            {
                if (!ll_isIrkAllZero(peerIrk))            // for all-zero local IRK, not RPA used
                {
                    if (ll_CalcRandomAddr(peerIrk, resolve_address) == SUCCESS)
                    {
                        //                    useRpa = TRUE;
                        osal_memcpy( &g_currentPeerRpa[0],  resolve_address, 6);
                    }
                }
            }

//    if (useRpa == FALSE)
//          osal_memcpy((uint8_t *) &(g_tx_adv_buf.data[6]), peerInfo.peerAddr, 6);
        }
    }
    else
    {
        if (own_address_type == LL_DEV_ADDR_TYPE_RPA_PUBLIC
                || own_address_type == LL_DEV_ADDR_TYPE_PUBLIC)
        {
            LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownPublicAddr );
            g_currentLocalAddrType = LL_DEV_ADDR_TYPE_PUBLIC;
        }
        else
        {
            LL_COPY_DEV_ADDR_LE( adv_param.ownAddr, ownRandomAddr );
            g_currentLocalAddrType  = LL_DEV_ADDR_TYPE_RANDOM;
        }
    }

    // =========== controller select parameters, TBD
    // decide primary advertising interval. The aux PDU period is decided by global_config, the primary adv interval
    // should be set considering the maximum skip AUX PDU requirement
    if (primary_advertising_interval_Min * (secondary_advertising_max_skip + 1) * 625 > g_advSlotPeriodic)
        g_pExtendedAdvInfo[i].primary_advertising_interval = primary_advertising_interval_Min * 625;    // bug fixed 04-08, * 1250 -> * 625
    else
        g_pExtendedAdvInfo[i].primary_advertising_interval = primary_advertising_interval_Max * 625;

    if(secondary_advertising_PHY==LL_PHY_2M)
        g_interAuxPduDuration = pGlobal_config[LL_EXT_ADV_INTER_SEC_CHN_INT_2MPHY];
    else
        g_interAuxPduDuration = pGlobal_config[LL_EXT_ADV_INTER_SEC_CHN_INT];

    // select Tx power and return
    g_pExtendedAdvInfo[i].tx_power = advertising_tx_power;
    *selectTxPwr = advertising_tx_power;
    g_pExtendedAdvInfo[i].isPeriodic = FALSE;
    return( LL_STATUS_SUCCESS );
}


/*******************************************************************************
    @fn          LL_SetExtAdvData

    @brief       This function is used to set extend advertiser set data

    input parameters

    @param       adv_handle - advertisement set handler


    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_SetExtAdvData( uint8  adv_handle,
                             uint8  operation,
                             uint8  fragment_preference,
                             uint8  advertising_data_length,
                             uint8* advertising_data
                           )
{
    int   i;

    // TODO: add parameters range checking
    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;

    if (operation > BLE_EXT_ADV_OP_UNCHANGED_DATA)
        return LL_STATUS_ERROR_BAD_PARAMETER;

    if (advertising_data_length > 251)
        return LL_STATUS_ERROR_BAD_PARAMETER;

    // extended advertiser memory is allocate by application, return fail if not
    if (g_pExtendedAdvInfo == NULL)
        return LL_STATUS_ERROR_OUT_OF_HEAP;

    // search advertisement handler list
    for (i = 0; i < g_extAdvNumber; i ++)
    {
        if (g_pExtendedAdvInfo[i].advHandle == adv_handle)
            break;
    }

    // if not found, then adv parameter has not been set
    if (i == g_extAdvNumber)
        return LL_STATUS_ERROR_UNKNOWN_ADV_ID;

    // check legacy ADV data length should <= 31
    if (ll_isLegacyAdv(&g_pExtendedAdvInfo[i])
            && (operation != BLE_EXT_ADV_OP_COMPLETE_DATA
                || advertising_data_length > 31))
        return LL_STATUS_ERROR_BAD_PARAMETER;

    if (operation == BLE_EXT_ADV_OP_FIRST_FRAG ||
            operation == BLE_EXT_ADV_OP_COMPLETE_DATA)
    {
        g_pExtendedAdvInfo[i].data.advertisingDataLength = 0;
        g_pExtendedAdvInfo[i].data.dataComplete = FALSE;
    }

    if (g_pExtendedAdvInfo[i].data.advertisingDataLength + advertising_data_length > g_advSetMaximumLen)
    {
        g_pExtendedAdvInfo[i].data.advertisingDataLength = 0;
        return LL_STATUS_ERROR_OUT_OF_HEAP;
    }

    // fill advertising set
    g_pExtendedAdvInfo[i].advHandle          = adv_handle;
    g_pExtendedAdvInfo[i].data.fragmentPreference = fragment_preference;
    memcpy(&g_pExtendedAdvInfo[i].data.advertisingData[g_pExtendedAdvInfo[i].data.advertisingDataLength],
           advertising_data, advertising_data_length);
    g_pExtendedAdvInfo[i].data.advertisingDataLength += advertising_data_length;

    // last fragment or 1 segment adv data
    if (operation == BLE_EXT_ADV_OP_LAST_FRAG     ||
            operation == BLE_EXT_ADV_OP_COMPLETE_DATA )
        g_pExtendedAdvInfo[i].data.dataComplete = TRUE;

    if (operation == BLE_EXT_ADV_OP_LAST_FRAG     ||
            operation == BLE_EXT_ADV_OP_COMPLETE_DATA ||
            operation == BLE_EXT_ADV_OP_UNCHANGED_DATA )    // unchange data, just update DID
    {
        // update DID
        g_pExtendedAdvInfo[i].data.DIDInfo = ll_generateExtAdvDid(g_pExtendedAdvInfo[i].data.DIDInfo);
    }

    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_SetExtScanRspData

    @brief       This function is used to set extend scan response data

    input parameters

    @param       adv_handle - advertisement set handler


    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_SetExtScanRspData( uint8 adv_handle,
                                 uint8 operation,
                                 uint8  fragment_preference,
                                 uint8  scan_rsp_data_length,
                                 uint8* scan_rsp_data
                               )
{
    int   i;

    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;

    if ((operation != BLE_EXT_ADV_OP_INTERM_FRAG)
            && (operation != BLE_EXT_ADV_OP_FIRST_FRAG)
            && (operation != BLE_EXT_ADV_OP_LAST_FRAG)
            && (operation != BLE_EXT_ADV_OP_COMPLETE_DATA))
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    // extended advertiser memory is allocate by application, return fail if not
    if (g_pExtendedAdvInfo == NULL)
        return LL_STATUS_ERROR_OUT_OF_HEAP;

    // search advertisement handler list
    for (i = 0; i < g_extAdvNumber; i ++)
    {
        if (g_pExtendedAdvInfo[i].advHandle == adv_handle)
            break;
    }

    // if not found, then adv parameter has not been set
    if (i == g_extAdvNumber)
        return LL_STATUS_ERROR_UNKNOWN_ADV_ID;

    // check legacy ADV scan rsp data length should <= 31
    if (ll_isLegacyAdv(&g_pExtendedAdvInfo[i])
            && (operation != BLE_EXT_ADV_OP_COMPLETE_DATA
                || scan_rsp_data_length > 31))
        return LL_STATUS_ERROR_BAD_PARAMETER;

//  // if not found, search the 1st available slot
//  if (i == g_extAdvNumber)
//    {
//        for (i = 0; i < g_extAdvNumber; i ++)
//        {
//            if (g_pExtendedAdvInfo[i].advHandle == LL_INVALID_ADV_SET_HANDLE)
//              break;
//        }
//        if (i == g_extAdvNumber)
//          return LL_STATUS_ERROR_OUT_OF_HEAP;
//    }

    if ((operation == BLE_EXT_ADV_OP_FIRST_FRAG) && (g_pExtendedAdvInfo[i].scanRspMaxLength != 0))
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    if (g_pExtendedAdvInfo[i].scanRspData == NULL || g_advSetMaximumLen < g_pExtendedAdvInfo[i].scanRspMaxLength + scan_rsp_data_length)
        return LL_STATUS_ERROR_OUT_OF_HEAP;

    if (ll_isLegacyAdv(&g_pExtendedAdvInfo[i]))
        g_pExtendedAdvInfo[i].scanRspMaxLength = 0;

    osal_memcpy(&g_pExtendedAdvInfo[i].scanRspData[g_pExtendedAdvInfo[i].scanRspMaxLength], scan_rsp_data, scan_rsp_data_length);
    //osal_memcpy(&g_pExtendedAdvInfo[i].scanRspData[0], scan_rsp_data, scan_rsp_data_length);
    g_pExtendedAdvInfo[i].scanRspMaxLength += scan_rsp_data_length;
    extscanrsp_offset = 0;
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_SetExtAdvEnable

    @brief       This function is used to enable/disable extend advertise

    input parameters

    @param       enable - enable/disable


    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_SetExtAdvEnable( uint8  enable,
                               uint8  number_of_sets,
                               uint8*  advertising_handle,
                               uint16* duration,
                               uint8*  max_extended_advertising_events)
{
    int i, j;
    extAdvInfo_t*  pExtAdv[64];
    periodicAdvInfo_t* pPrdAdv[64];

    //LOG("==>LL_SetExtAdvEnable\n");
    // TODO: add more sanity checking to align to the spec
    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;

    if (number_of_sets == 0 && enable == TRUE)
        return LL_STATUS_ERROR_UNEXPECTED_PARAMETER;

    //  the adv parameter should be present for the adv_handler, otherwise, it will return Unknown Advertising Identifier(0x42)
    for (i = 0; i < number_of_sets; i++)
    {
        pExtAdv[i] = NULL;
        pPrdAdv[i] = NULL;

        for (j = 0; j < g_extAdvNumber; j++)
        {
            if (g_pExtendedAdvInfo[j].advHandle == advertising_handle[i])
            {
                pExtAdv[i] = &g_pExtendedAdvInfo[j];
                break;
            }
        }

        if (pExtAdv[i] == NULL)     // adv handle not found
            return LL_STATUS_ERROR_UNKNOWN_ADV_ID;

        if (ll_isLegacyAdv(pExtAdv[i]))
        {
            if (pExtAdv[i]->data.advertisingDataLength > 31)
                return LL_STATUS_ERROR_ILLEGAL_PARAM_COMBINATION;
        }

        if ((pExtAdv[i]->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM
                || pExtAdv[i]->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
                && pExtAdv[i]->parameter.isOwnRandomAddressSet == FALSE)
            return LL_STATUS_ERROR_BAD_PARAMETER;

        // TODO: If the advertising set's Own_Address_Type parameter is set to 0x03, the
        // controller's resolving list did not contain a matching entry, and the random
        // address for the advertising set has not been initialized, the Controller shall
        // return the error code Invalid HCI Command Parameters (0x12).

//        pExtAdv[i]->isPeriodic = FALSE;
//        // if periodic adv is support, check whether the adv set is periodic adv
//        if (g_pPeriodicAdvInfo != NULL)
//        {
//            for (j = 0; j < g_perioAdvNumber; j ++)
//            {
//                if (g_pPeriodicAdvInfo[j].advHandle == pExtAdv[i]->advHandle)
//                {
//                    pExtAdv[i]->isPeriodic = TRUE;
//                  index = j;
//                    break;
//                }
//            }
//        }

        // for periodic adv, search the period adv context
        if (pExtAdv[i]->isPeriodic == TRUE)
        {
            for (j = 0; j < g_perioAdvNumber; j ++)
            {
                if (g_pPeriodicAdvInfo[j].advHandle == pExtAdv[i]->advHandle)
                {
                    pPrdAdv[i] = &g_pPeriodicAdvInfo[j];
                    break;
                }
            }

            if (pPrdAdv[i] == NULL)
                return LL_STATUS_ERROR_UNKNOWN_ADV_ID;
        }

        // for enable adv case, the adv set data should be complete,
        //   otherwise, it shall return Command Disallowed(0x0C)
        if (enable == TRUE)
        {
            // check scan rsp data
            if ((!ll_isLegacyAdv(pExtAdv[i]))          // no legacy adv(legacy adv, scan rsp with len = 0 scan data is allowed)
                    && (pExtAdv[i]->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK)   // scannable adv
                    && (pExtAdv[i]->scanRspData == NULL || pExtAdv[i]->scanRspMaxLength == 0))     // no scan rsp data
                return  LL_STATUS_ERROR_COMMAND_DISALLOWED;

            // check adv data
            if ((pExtAdv[i]->isPeriodic == FALSE
                    && pExtAdv[i]->data.dataComplete == FALSE
                    && !(pExtAdv[i]->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK))   // no adv data for scan adv
                    || (pExtAdv[i]->isPeriodic == TRUE && pPrdAdv[i]->data.dataComplete == FALSE))
                return LL_STATUS_ERROR_COMMAND_DISALLOWED;
        }
    }

    // check OK, save the enable info
    for (i = 0; i < number_of_sets; i++)
    {
        pExtAdv[i]->active = enable;

        if (enable == TRUE)
        {
            // only save parameters in enable case
            pExtAdv[i]->duration = duration[i] * 10000;     // 10000: unit 10ms, convert to us
            pExtAdv[i]->maxExtAdvEvents = max_extended_advertising_events[i];
        }
    }

    // ====== update extend advertiser scheduler to apply new parameters
    extscanrsp_offset = 0;
    HAL_ENTER_CRITICAL_SECTION();

    if (enable == FALSE)
    {
        // case 1: number of set is 0, disable all adv case. Remove the adv from adv schduler
        if (number_of_sets == 0)
        {
            for (i = 0; i < g_extAdvNumber; i ++)      // bug fixed 04-07
            {
                if (g_pAdvSchInfo[i].adv_handler == LL_INVALID_ADV_SET_HANDLE)
                    continue;

                ll_delete_adv_task(i);
            }

            //  periodic adv case, disable all EXT_ADV_IND + AUX_ADV_IND of periodic adv. It should be OK
            //  by set the active flag to FALSE. TO BE TEST.
            for (i = 0; i < g_perioAdvNumber; i ++)
            {
                if (g_pAdvSchInfo_periodic[i].adv_handler == LL_INVALID_ADV_SET_HANDLE)
                    continue;

                g_pAdvSchInfo_periodic[i].pAdvInfo->active = FALSE;

                if (g_pAdvSchInfo_periodic[i].pAdvInfo_prd->active == FALSE)
                    ll_delete_adv_task_periodic(i);
            }
        }
        // case 2: disable the adv in the adv set list
        else
        {
            for (i = 0; i < number_of_sets; i ++)
            {
                // 2021-1-11, extended adv case
                if (pExtAdv[i]->isPeriodic == FALSE)
                {
                    // search the adv in the scheduler list and disable it
                    for (j = 0; j < g_schExtAdvNum; j++)
                    {
                        if (g_pAdvSchInfo[j].adv_handler == pExtAdv[i]->advHandle)
                        {
                            ll_delete_adv_task(j);
                            break;
                        }
                    }
                }
                else // periodic adv case
                {
                    pExtAdv[i]->active = FALSE;

                    for (j = 0; j < g_schExtAdvNum_periodic; j ++)
                    {
                        if (g_pAdvSchInfo_periodic[j].adv_handler == pExtAdv[i]->advHandle)
                        {
                            g_pAdvSchInfo_periodic[j].nextEventRemainder = LL_INVALID_TIME;

                            if (pPrdAdv[i]->active == FALSE)
                                ll_delete_adv_task_periodic(j);

                            break;
                        }
                    }
                }
            }
        }

        // stop RPA refresh timer, not consider multiple ext adv enable case
        osal_stop_timerEx(LL_TaskID, LL_EVT_RPA_TIMEOUT);
    }
    else     // enable case
    {
        // LOG("len:%d\n",g_pExtendedAdvInfo[0].data.advertisingDataLength);
        // for(int y=0;y<g_pExtendedAdvInfo[0].data.advertisingDataLength;y++)
        // {
        //     LOG("%02x ",g_pExtendedAdvInfo[0].data.advertisingData[y]);
        // }
        for (i = 0; i < number_of_sets; i++)
        {
            for (j = 0; j < g_schExtAdvNum; j++)
            {
                // check whether the adv already started
                if (g_pAdvSchInfo[j].adv_handler == pExtAdv[i]->advHandle)    // the adv in the scheduler list
                {
                    // TODO: adv already enable, it should:
                    // If the HCI_LE_Set_Extended_Advertising_Enable command is sent again for
                    // an advertising set while that set is enabled, the timer used for the duration and
                    // the number of events counter are reset and any change to the random address
                    // shall take effect
                    break;
                }
            }

            // new extended adv case
            if (j == g_schExtAdvNum && pExtAdv[i]->isPeriodic == FALSE)
            {
                // for the 1st extended adv, set global aux PDU resource pool period as the primary adv channel interval
                if (/*!ll_isLegacyAdv(pExtAdv[i]) &&*/ g_schExtAdvNum == 0)   // TODO: check legacy/aux PDU required
                {
                    g_advSlotPeriodic = pExtAdv[i]->primary_advertising_interval;

                    if (g_extAdvNumber > 4)
                        g_advPerSlotTick = g_advSlotPeriodic >> 3;
                    else
                        g_advPerSlotTick = g_advSlotPeriodic >> 2;
                }

                ll_add_adv_task(pExtAdv[i]);      // TODO: if add adv task failed, return failure to host
            }

            // new periodic adv case
            if (j == g_schExtAdvNum && pExtAdv[i]->isPeriodic == TRUE)
            {
                // check whether the corresponding periodic adv is enable
                // 1. enable, start periodic adv
                // 2. disable, do nothing
                if (pPrdAdv[i]->active == TRUE)
                    ll_add_adv_task_periodic(pPrdAdv[i], pExtAdv[i]);     // TODO: if add adv task failed, return failure to host
            }
        }

        // start RPA refresh timer, not consider multiple adv enable case now
        if (g_llRlDeviceNum > 0)
            osal_start_timerEx( LL_TaskID, LL_EVT_RPA_TIMEOUT, g_llRlTimeout * 1000 );
    }

    HAL_EXIT_CRITICAL_SECTION();
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_ReadMaximumAdvDataLength

    @brief       This function is used to read the maximum adv set data length support by controller

    input parameters

    @param       adv_handle - advertisement set handler


    output parameters

    @param       length  -  pointer to the variable of maximum data length support

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_ReadMaximumAdvDataLength( uint16* length )
{
//    *length = LL_MAX_ADVERTISER_SET_LENGTH;            // TBD.
    *length = g_advSetMaximumLen;

    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_ReadNumberOfSupportAdvSet

    @brief       This function is used to read number of adv set supported by controller

    input parameters

    @param       adv_handle - advertisement set handler


    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_ReadNumberOfSupportAdvSet( uint8* number )
{
    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;
    *number = 240;              // TBD
    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_RemoveAdvSet

    @brief       This function is used to remove advertisement set

    input parameters

    @param       adv_handle - advertisement set handler


    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_RemoveAdvSet( uint8 adv_handle)
{
    uint8 i;
    uint8 extIndex = 0xFF, prdIndex = 0xFF;

    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;

    // search extendedadvertisement handler list
    for (i = 0; i < g_extAdvNumber; i ++)
    {
        if (g_pExtendedAdvInfo[i].advHandle == adv_handle)
        {
            extIndex = i;
            break;
        }
    }

    // search advertisement handler list
    for (i = 0; i < g_perioAdvNumber; i ++)
    {
        if (g_pPeriodicAdvInfo[i].advHandle == adv_handle)
        {
            prdIndex = i;
            break;
        }
    }

    if (extIndex == 0xFF && prdIndex == 0xFF)
        return LL_STATUS_ERROR_UNKNOWN_ADV_ID;

    if ((extIndex != 0xFF && g_pExtendedAdvInfo[extIndex].active == TRUE)
            || (prdIndex != 0xFF && g_pPeriodicAdvInfo[prdIndex].active == TRUE))
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    if (extIndex != 0xFF)
    {
        g_pExtendedAdvInfo[i].advHandle = LL_INVALID_ADV_SET_HANDLE;
        g_pExtendedAdvInfo[i].parameter.isOwnRandomAddressSet = FALSE;
        g_pExtendedAdvInfo[i].adv_event_counter = 0;
        g_pExtendedAdvInfo[i].data.fragmentPreference = 0xFF;
        g_pExtendedAdvInfo[i].data.advertisingDataLength = 0;        // no adv data
        g_pExtendedAdvInfo[i].data.dataComplete = TRUE;
        g_pExtendedAdvInfo[i].scanRspMaxLength = 0;                  // no scan rsp data
        g_pExtendedAdvInfo[i].isPeriodic = FALSE;
    }

    if (prdIndex != 0xFF)
    {
        g_pPeriodicAdvInfo[i].advHandle                       = LL_INVALID_ADV_SET_HANDLE;
        g_pPeriodicAdvInfo[i].active            = FALSE;
        g_pPeriodicAdvInfo[i].periodic_adv_event_counter = 0;
        g_pPeriodicAdvInfo[i].currentAdvOffset  = 0;
        g_pPeriodicAdvInfo[i].data.advertisingDataLength = 0;        // no data
        g_pPeriodicAdvInfo[i].data.dataComplete = TRUE;
        g_pPeriodicAdvInfo[i].PrdCTEInfo.enable = FALSE;
    }

    return( LL_STATUS_SUCCESS );
}

/*******************************************************************************
    @fn          LL_ClearAdvSets

    @brief       This function is used to clear the stored advertisement sets in controller

    input parameters

    @param       none


    output parameters

    @param       none

    @return      LL_STATUS_SUCCESS, error code(TBD)
*/
llStatus_t LL_ClearAdvSets(void)
{
    uint8 i;

    if (g_llAdvMode == LL_MODE_LEGACY)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llAdvMode = LL_MODE_EXTENDED;

    // check extendedadvertisement handler list
    for (i = 0; i < g_extAdvNumber; i ++)
    {
        if (g_pExtendedAdvInfo[i].advHandle != LL_INVALID_ADV_SET_HANDLE
                && g_pExtendedAdvInfo[i].active == TRUE)
            return LL_STATUS_ERROR_COMMAND_DISALLOWED;
    }

    // check periodic advertisement handler list
    for (i = 0; i < g_perioAdvNumber; i ++)
    {
        if (g_pPeriodicAdvInfo[i].advHandle != LL_INVALID_ADV_SET_HANDLE
                && g_pPeriodicAdvInfo[i].active == TRUE)
            return LL_STATUS_ERROR_COMMAND_DISALLOWED;
    }

    // clear adv set (extended adv part)
    for (i = 0; i < g_extAdvNumber; i ++)
    {
        g_pExtendedAdvInfo[i].advHandle = LL_INVALID_ADV_SET_HANDLE;
        g_pExtendedAdvInfo[i].parameter.isOwnRandomAddressSet = FALSE;
        g_pExtendedAdvInfo[i].adv_event_counter = 0;
        g_pExtendedAdvInfo[i].data.fragmentPreference = 0xFF;
        g_pExtendedAdvInfo[i].data.advertisingDataLength = 0;        // no adv data
        g_pExtendedAdvInfo[i].data.dataComplete = TRUE;
        g_pExtendedAdvInfo[i].scanRspMaxLength = 0;                  // no scan rsp data
        g_pExtendedAdvInfo[i].isPeriodic = FALSE;
    }

    // clear adv set (periodic adv part)
    for (i = 0; i < g_perioAdvNumber; i ++)
    {
        g_pPeriodicAdvInfo[i].advHandle                       = LL_INVALID_ADV_SET_HANDLE;
        g_pPeriodicAdvInfo[i].active            = FALSE;
        g_pPeriodicAdvInfo[i].periodic_adv_event_counter = 0;
        g_pPeriodicAdvInfo[i].currentAdvOffset  = 0;
        g_pPeriodicAdvInfo[i].data.advertisingDataLength = 0;        // no data
        g_pPeriodicAdvInfo[i].data.dataComplete = TRUE;
        g_pPeriodicAdvInfo[i].PrdCTEInfo.enable = FALSE;
    }

    return( LL_STATUS_SUCCESS );
}

llStatus_t LL_SetExtendedScanParameters(uint8 own_address_type,
                                        uint8 scanning_filter_policy,
                                        uint8 scanning_PHYs,
                                        uint8* scan_type,
                                        uint16* scan_interval,
                                        uint16* scan_window)
{
    //LOG("LL_SetExtendedScanParameters\n");
    uint8 number_phys, i;

    // TODO: sanity checking
    if (g_llScanMode == LL_MODE_LEGACY )
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llScanMode = LL_MODE_EXTENDED;

    // If the Host specifies a PHY that is not supported by the Controller, including a bit that is reserved for future use,
    // it should return the error code Unsupported Feature or Parameter Value(0x11).
    if ((scanning_PHYs & (~(LL_SCAN_PHY_1M_BITMASK | LL_SCAN_PHY_CODED_BITMASK))) != 0)
        return LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED;

    extScanInfo.wlPolicy    = scanning_filter_policy;
    number_phys = 0;

    if (scanning_PHYs & LL_SCAN_PHY_1M_BITMASK)
    {
        extScanInfo.scanPHYs[number_phys] = PKT_FMT_BLE1M;
        number_phys ++;
    }

    if (scanning_PHYs & LL_SCAN_PHY_CODED_BITMASK)
    {
        extScanInfo.scanPHYs[number_phys] = PKT_FMT_BLR125K;
        number_phys ++;
    }

    for (i = 0; i < number_phys; i ++)
    {
        extScanInfo.scanType[i] = scan_type[i];
        extScanInfo.scanInterval[i] = scan_interval[i];
        extScanInfo.scanWindow[i] = scan_window[i];
    }

    extScanInfo.numOfScanPHY = number_phys;
    extScanInfo.ownAddrType = own_address_type;

    // set the scanner's address based on the HCI's address type preference
    if ( own_address_type == LL_DEV_ADDR_TYPE_PUBLIC )
    {
        // get our address and address type
        LL_COPY_DEV_ADDR_LE( extScanInfo.ownAddr, ownPublicAddr );
    }
    else if ( own_address_type == LL_DEV_ADDR_TYPE_RANDOM )
    {
        // get our address and address type
        LL_COPY_DEV_ADDR_LE( extScanInfo.ownAddr, ownRandomAddr );
    }
    else
    {
        // for RPAs, scan control not indicate using which RPA list entry, no copy scanA here
    }

    return LL_STATUS_SUCCESS;
}

llStatus_t LL_SetExtendedScanEnable(uint8 enable,
                                    uint8 filter_duplicates,
                                    uint16 duration,
                                    uint16 period)
{
    //LOG("LL_SetExtendedScanEnable\n");
    // TODO: sanity checking
    if (g_llScanMode == LL_MODE_LEGACY )
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llScanMode = LL_MODE_EXTENDED;

    // comment out because BQB test set duration == period > 0
//  if (duration != 0 && period != 0 && duration >= period)
//      return LL_STATUS_ERROR_BAD_PARAMETER;

    if ((enable == TRUE)
            && (extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM              // own addr type is random
                || extScanInfo.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
            && (BDADDR_VALID(ownRandomAddr) == FALSE))                        // own random address is not set
        return LL_STATUS_ERROR_BAD_PARAMETER;

    if (extScanInfo.numOfScanPHY == 0)
        return LL_STATUS_ERROR_BAD_PARAMETER;

    extScanInfo.enable = enable;
    extScanInfo.filterDuplicate = filter_duplicates;
    extScanInfo.duration = duration;
    extScanInfo.period   = period;

    // trigger scan task
    if (enable == TRUE)
    {
        extScanInfo.current_index = 0;
        extScanInfo.current_chn = LL_ADV_CHAN_FIRST;
        extScanInfo.current_scan_PHY = extScanInfo.scanPHYs[extScanInfo.current_index];
        extScanInfo.adv_data_offset = 0;
        llScanDuration = 0;
        llSetupExtScan(extScanInfo.current_chn);
    }
    else
    {
        if (llTaskState == LL_TASK_EXTENDED_SCAN || llTaskState == LL_TASK_EXTENDED_INIT)
            llTaskState = LL_TASK_INVALID;                  // if ext scanning is ongoing, the IRQ will not process

        // otherwise, when scheduler timer expiry, it will check extScanInfo.enable
    }

    return LL_STATUS_SUCCESS;
}

llStatus_t LL_ExtendedCreateConnection(uint8 initiator_filter_policy,
                                       uint8 own_address_type,
                                       uint8 peer_address_type,
                                       uint8* peer_address,
                                       uint8 initiating_PHYs,
                                       uint16* scan_interval,
                                       uint16* scan_window,
                                       uint16* conn_interval_min,
                                       uint16* conn_interval_max,
                                       uint16* conn_latency,
                                       uint16* supervision_timeout,
                                       uint16* minimum_CE_length,
                                       uint16* maximum_CE_length)
{
    //LOG("LL_ExtendedCreateConnection\n");
    uint8 number_phys, i;
    llConnState_t* connPtr;
    uint16         txHeader = 0x2205;

    if (g_llScanMode == LL_MODE_LEGACY )
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    g_llScanMode = LL_MODE_EXTENDED;

    if (extInitInfo.scanMode == LL_SCAN_START)
        return LL_STATUS_ERROR_COMMAND_DISALLOWED;

    // TODO: more sanity checking
    if ((initiating_PHYs & (~(LL_SCAN_PHY_1M_BITMASK | LL_CONN_PHY_2M_BITMASK | LL_SCAN_PHY_CODED_BITMASK))) != 0)
        return LL_STATUS_ERROR_FEATURE_NOT_SUPPORTED;

    // =====  allocate a connection and assure it is valid
    if ( (connPtr = llAllocConnId()) == NULL )
    {
        llSecondaryState = LL_SEC_STATE_IDLE;      // recover llSecondaryState
        // exceeded the number of available connection structures
        return( LL_STATUS_ERROR_CONNECTION_LIMIT_EXCEEDED );
    }

    // ==================  save init parameters
    extInitInfo.ownAddrType = own_address_type;
    extInitInfo.wlPolicy    = initiator_filter_policy;
    number_phys = 0;
    i = 0;

    if (initiating_PHYs & LL_SCAN_PHY_1M_BITMASK)
    {
        extInitInfo.initPHYs[number_phys] = PKT_FMT_BLE1M;
        extInitInfo.scanInterval[number_phys] = scan_interval[i];
        extInitInfo.scanWindow[number_phys] = scan_window[i];
        extInitInfo.conn_latency[number_phys] = conn_latency[i];
        extInitInfo.supervision_timeout[number_phys] = supervision_timeout[i];
        extInitInfo.conn_interval_min[number_phys] = conn_interval_min[i];
        extInitInfo.conn_interval_max[number_phys] = conn_interval_max[i];
        extInitInfo.minimum_CE_length[number_phys] = minimum_CE_length[i];
        extInitInfo.maximum_CE_length[number_phys] = maximum_CE_length[i];
        number_phys ++;
        i ++;
    }

    if (initiating_PHYs & LL_CONN_PHY_2M_BITMASK)
    {
        // if no 1M PHY scan parameters, using 2M PHY scan parameters. Note that it may conflict with
        // spec "The Scan_Interval[i] and Scan_Window[i] parameters are ...  If bit 1 is
        // set in Initiating_PHYs, the values for the LE 2M PHY shall be ignored."
//      if (number_phys == 0)
//      {
//          extInitInfo.initPHYs[number_phys] = PKT_FMT_BLE1M;
//          extInitInfo.scanInterval[number_phys] = scan_interval[i];
//          extInitInfo.scanWindow[number_phys] = scan_window[i];
//          number_phys ++;
//      }
        extInitInfo.is_2M_parameter_present = TRUE;
        // no scan parameters for 2M PHY, the HCI parameters are skipped
        extInitInfo.conn_latency_2Mbps = conn_latency[i];
        extInitInfo.supervision_timeout_2Mbps = supervision_timeout[i];
        extInitInfo.conn_interval_min_2Mbps = conn_interval_min[i];
        extInitInfo.conn_interval_max_2Mbps = conn_interval_max[i];
        extInitInfo.minimum_CE_length_2Mbps = minimum_CE_length[i];
        extInitInfo.maximum_CE_length_2Mbps = maximum_CE_length[i];
        i ++;
    }

    if (initiating_PHYs & LL_SCAN_PHY_CODED_BITMASK)
    {
        extInitInfo.initPHYs[number_phys] = PKT_FMT_BLR125K;
        extInitInfo.scanInterval[number_phys] = scan_interval[i];
        extInitInfo.scanWindow[number_phys] = scan_window[i];
        extInitInfo.conn_latency[number_phys] = conn_latency[i];
        extInitInfo.supervision_timeout[number_phys] = supervision_timeout[i];
        extInitInfo.conn_interval_min[number_phys] = conn_interval_min[i];
        extInitInfo.conn_interval_max[number_phys] = conn_interval_max[i];
        extInitInfo.minimum_CE_length[number_phys] = minimum_CE_length[i];
        extInitInfo.maximum_CE_length[number_phys] = maximum_CE_length[i];
        number_phys ++;
    }

    extInitInfo.numOfScanPHY = number_phys;

    // if 2Mbps PHY connection parameters are not present, we will use 1Mbps PHY connection parameter for 2M PHY
    // note that if 1M PHY connection parameters are not present, it will not create connection in 2M PHY also
    if (((initiating_PHYs & LL_CONN_PHY_2M_BITMASK) == 0)
            && (initiating_PHYs & LL_SCAN_PHY_1M_BITMASK))
    {
        extInitInfo.is_2M_parameter_present = TRUE;
        // no scan parameters for 2M PHY, the HCI parameters are skipped
        extInitInfo.conn_latency_2Mbps = conn_latency[0];
        extInitInfo.supervision_timeout_2Mbps = supervision_timeout[0];
        extInitInfo.conn_interval_min_2Mbps = conn_interval_min[0];
        extInitInfo.conn_interval_max_2Mbps = conn_interval_max[0];
        extInitInfo.minimum_CE_length_2Mbps = minimum_CE_length[0];
        extInitInfo.maximum_CE_length_2Mbps = maximum_CE_length[0];
    }

    // save the peer address type
    peerInfo.peerAddrType = peer_address_type;

    // save the peer address
    if (peer_address != NULL)              // bug fixed
        LL_COPY_DEV_ADDR_LE( peerInfo.peerAddr, peer_address );

    // save our address type
    extInitInfo.ownAddrType = own_address_type;

    // check the type of own address
    if ( own_address_type == LL_DEV_ADDR_TYPE_PUBLIC )
    {
        // save our address
        LL_COPY_DEV_ADDR_LE( extInitInfo.ownAddr, ownPublicAddr );
    }
    else // LL_DEV_ADDR_TYPE_RANDOM
    {
        // save our address
        LL_COPY_DEV_ADDR_LE( extInitInfo.ownAddr, ownRandomAddr );
    }

    // ========== update connection context
    g_ll_conn_ctx.numLLMasterConns ++;
    // reset connection parameters
    LL_set_default_conn_params(connPtr);

    // clear the connection buffer
    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.tx_conn_desc[j] = connPtr->ll_buf.tx_not_ack_pkt;

    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.rx_conn_desc[j] = connPtr->ll_buf.tx_not_ack_pkt;

    reset_conn_buf(connPtr->connId);

    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.tx_conn_desc[j] = NULL;

    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.rx_conn_desc[j] = NULL;

    // save the connection ID with Init
    extInitInfo.connId = connPtr->connId;

    // set the connection channel map
    for (i = 0; i < LL_NUM_BYTES_FOR_CHAN_MAP; i++)
    {
        connPtr->chanMap[i] = chanMapUpdate.chanMap[i];
    }

    // process connection channel map into the data channel table
    llProcessChanMap( connPtr, connPtr->chanMap );
    // ramdomly generate a valid 24 bit CRC value
    connPtr->initCRC = llGenerateCRC();
    // randomly generate a valid, previously unused, 32-bit access address
    connPtr->accessAddr = llGenerateValidAccessAddr();
//  ============= below connection context depends on PHY index, will be selected when scan OK
    #if 0

    if (g_ll_conn_ctx.numLLMasterConns == 1)    // A2 multi-connection, 1st connection, save the connection parameters
    {
        // determine the connection interval based on min and max values
        // Note: Range not used, so assume max value.
        // Note: minLength and maxLength are informational.
        connPtr->curParam.connInterval = conn_interval_max;
        // set the connection timeout
        // Note: The spec says this begins at the end of the CONNECT_REQ, but the
        //       LSTO will be converted into events.
        connPtr->curParam.connTimeout = supervision_timeout;
        // set the slave latency
        connPtr->curParam.slaveLatency = conn_latency;
        // save connection parameter as global
        g_ll_conn_ctx.connInterval = connPtr->curParam.connInterval;                              // unit: 1.25ms
        g_ll_conn_ctx.slaveLatency = connPtr->curParam.slaveLatency;
        g_ll_conn_ctx.connTimeout  = connPtr->curParam.connTimeout;
        g_ll_conn_ctx.per_slot_time = connPtr->curParam.connInterval * 2 / g_maxConnNum;       // unit: 625us
    }
    else
    {
        // determine the connection interval based on min and max values
        // Note: Range not used, so assume max value.
        // Note: minLength and maxLength are informational.
        connPtr->curParam.connInterval = g_ll_conn_ctx.connInterval;
        // set the connection timeout
        // Note: The spec says this begins at the end of the CONNECT_REQ, but the
        //       LSTO will be converted into events.
        connPtr->curParam.connTimeout = g_ll_conn_ctx.connTimeout;
        // set the slave latency
        connPtr->curParam.slaveLatency = g_ll_conn_ctx.slaveLatency;
    }

    #endif
    // set the master's SCA
    connPtr->sleepClkAccuracy = extInitInfo.scaValue;
    // set the window size (units of 1.25ms)
    // Note: Must be the lesser of 10ms and the connection interval - 1.25ms.
    connPtr->curParam.winSize = pGlobal_config[LL_CONN_REQ_WIN_SIZE];
    // set the window offset (units of 1.25ms). TO change if we support multiple connections
    // Note: Normally, the window offset is managed dynamically so that precise
    //       connection start times can be achieved (necessary for multiple
    //       connnections). However, sometimes it is useful to force the window
    //       offset to something specific for testing. This can be done by here
    //       when the project is built with the above define.
    // Note: This define should only be used for testing one connection and will
    //       NOT work when multiple connections are attempted!
    connPtr->curParam.winOffset = pGlobal_config[LL_CONN_REQ_WIN_OFFSET];//2;//LL_WINDOW_OFFSET;
    // set the channel map hop length (5..16)
    // Note: 0..255 % 12 = 0..11 + 5 = 5..16.
    connPtr->hop = (uint8)( (LL_ENC_GeneratePseudoRandNum() % 12) + 5);
    // ===============  fill AUX_CONNECT_REQ PDU, some connection parameters depend on init PHY
    {
        uint8 offset = 0;

        // initA, Byte 0 ~ 5
        // check the type of own address. Note: RPA will be considered in ll_processExtInitIRQ, TO be added(2020-12-3)
        if ( own_address_type == LL_DEV_ADDR_TYPE_PUBLIC )
        {
            // save our address
            LL_COPY_DEV_ADDR_LE( &g_tx_adv_buf.data[offset], ownPublicAddr );
        }
        else // LL_DEV_ADDR_TYPE_RANDOM
        {
            // save our address
            LL_COPY_DEV_ADDR_LE( &g_tx_adv_buf.data[offset], ownRandomAddr );
        }

        offset += 6;

        if (peer_address != NULL)
            LL_COPY_DEV_ADDR_LE(&g_tx_adv_buf.data[offset], peer_address)     // AdvA,  Byte 6 ~ 11
            offset += 6;

        txHeader |=  (own_address_type << TX_ADD_SHIFT & TX_ADD_MASK);
        txHeader |=  (peer_address_type << RX_ADD_SHIFT & RX_ADD_MASK);     // TODO: not consider RPA type
        g_tx_adv_buf.txheader = txHeader;
        // Access Address, Byte 12 ~ 15
        memcpy((uint8*)&g_tx_adv_buf.data[offset], (uint8*)&connPtr->accessAddr, 4);
        offset += 4;
        // CRC init, Byte 16 ~ 18
        memcpy((uint8*)&g_tx_adv_buf.data[offset], (uint8*)&connPtr->initCRC, 3);
        offset += 3;
        // WinSize, Byte 19
        g_tx_adv_buf.data[offset] = connPtr->curParam.winSize;
        offset += 1;
        // WinOffset, Byte 20 ~ 21
        memcpy((uint8*)&g_tx_adv_buf.data[offset], (uint8*)&connPtr->curParam.winOffset, 2);
        offset += 2;
        // Interval, Byte 22 ~ 23
//        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->curParam.connInterval, 2);
        offset += 2;
        // Latency, Byte 24 ~ 25
//        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->curParam.slaveLatency, 2);
        offset += 2;
        // Timeout, Byte 26 ~ 27
//        memcpy((uint8 *)&g_tx_adv_buf.data[offset], (uint8 *)&connPtr->curParam.connTimeout, 2);
        offset += 2;
        // Channel Map, Byte 28 ~ 32
        memcpy((uint8*)&g_tx_adv_buf.data[offset], (uint8*)&connPtr->chanMap[0], 5);
        offset += 5;
        // Hop(5bit) + SCA(3bit), Byte 33
        g_tx_adv_buf.data[offset] = (connPtr->hop & 0x1f) | ((connPtr->sleepClkAccuracy & 0x7) << 5);
    }
    // =============== init scan
//  if ( llState == LL_STATE_IDLE )
//        // go ahead and start Init immediately
    extInitInfo.current_index = 0;
    extInitInfo.current_chn = LL_ADV_CHAN_FIRST;
    extInitInfo.current_scan_PHY = extInitInfo.initPHYs[extInitInfo.current_index];
    // enable Init scan
    extInitInfo.scanMode = LL_SCAN_START;
    llSetupExtInit();
//  else
//        osal_set_event(LL_TaskID, LL_EVT_SECONDARY_INIT);
    return LL_STATUS_SUCCESS;
}

// ===========================================================
//     extended adv HCI functions' patch
hciStatus_t HCI_LE_SetExtAdvSetRandomAddressCmd( uint8 adv_handle,
                                                 uint8* random_address
                                               )
{
    hciStatus_t status;
    status = LL_SetExtAdvSetRandomAddress(adv_handle, random_address);
    HCI_CommandCompleteEvent( HCI_LE_SET_ADVERTISING_SET_RANDOM_ADDRESS, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtAdvParamCmd( uint8 adv_handle,
                                      uint16 adv_event_properties,
                                      uint32 primary_advertising_interval_Min,          // 3 octets
                                      uint32 primary_advertising_interval_Max,          // 3 octets
                                      uint8  primary_advertising_channel_map,
                                      uint8  own_address_type,
                                      uint8  peer_address_type,
                                      uint8* peer_address,
                                      uint8  advertising_filter_policy,
                                      int8   advertising_tx_power,
                                      uint8  primary_advertising_PHY,
                                      uint8  secondary_advertising_max_skip,
                                      uint8  secondary_advertising_PHY,
                                      uint8  advertising_SID,
                                      uint8  scan_request_notification_enable
                                    )
{
    uint8 rtnParam[2];     // octect 0: status, octect 1: Selected_Tx_Power
    int8 selectTxPower;
    rtnParam[0] = LL_SetExtAdvParam( adv_handle,
                                     adv_event_properties,
                                     primary_advertising_interval_Min,          // 3 octets
                                     primary_advertising_interval_Max,          // 3 octets
                                     primary_advertising_channel_map,
                                     own_address_type,
                                     peer_address_type,
                                     peer_address,
                                     advertising_filter_policy,
                                     advertising_tx_power,
                                     primary_advertising_PHY,
                                     secondary_advertising_max_skip,
                                     secondary_advertising_PHY,
                                     advertising_SID,
                                     scan_request_notification_enable, &selectTxPower);
    rtnParam[1] = selectTxPower;
    //LOG("LL_SetExtAdvParam adv_handle:%d,ret:%d\n",adv_handle,rtnParam[0]);
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDER_ADVERTISING_PARAMETERS, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtAdvDataCmd( uint8 adv_handle,
                                     uint8 operation,
                                     uint8  fragment_preference,
                                     uint8  advertising_data_length,
                                     uint8* advertising_data
                                   )
{
    hciStatus_t status;
    status = LL_SetExtAdvData(adv_handle,
                              operation,
                              fragment_preference,
                              advertising_data_length,
                              advertising_data
                             );
    //LOG("LL_SetExtAdvData adv_handle:%d,ret:%d\n",adv_handle,status);
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDED_ADVERTISING_DATA, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtScanRspDataCmd( uint8 adv_handle,
                                         uint8 operation,
                                         uint8  fragment_preference,
                                         uint8  scan_rsp_data_length,
                                         uint8* scan_rsp_data
                                       )
{
    hciStatus_t status;
    status = LL_SetExtScanRspData(adv_handle,
                                  operation,
                                  fragment_preference,
                                  scan_rsp_data_length,
                                  scan_rsp_data
                                 );
    //LOG("LL_SetExtScanRspData adv_handle:%d,ret:%d\n",adv_handle,status);
    HCI_CommandCompleteEvent( HCI_LE_Set_EXTENDED_SCAN_RESPONSE_DATA, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtAdvEnableCmd( uint8  enable,
                                       uint8  number_of_sets,
                                       uint8*  advertising_handle,
                                       uint16* duration,
                                       uint8*  max_extended_advertising_events)
{
    hciStatus_t status;
    status = LL_SetExtAdvEnable(enable, number_of_sets, advertising_handle, duration, max_extended_advertising_events);
    //LOG("LL_SetExtAdvEnable ret:%d\n",status);
    HCI_CommandCompleteEvent( HCI_LE_Set_EXTENDED_ADVERTISING_ENABLE, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_ReadMaximumAdvDataLengthCmd( void )
{
    uint8  rtnParam[3];
    uint16 len;
    rtnParam[0] = LL_ReadMaximumAdvDataLength( &len);
    rtnParam[1] = LO_UINT16( len);
    rtnParam[2] = HI_UINT16( len );
    HCI_CommandCompleteEvent( HCI_LE_READ_MAXIMUM_ADVERTISING_DATA_LENGTH, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_ReadNumberOfSupportAdvSetCmd( void )
{
    uint8 rtnParam[2];
    rtnParam[0] = LL_ReadNumberOfSupportAdvSet( &(rtnParam[1]) );
    HCI_CommandCompleteEvent( HCI_LE_READ_NUMBER_OF_SUPPORTED_ADVERTISING_SETS, sizeof(rtnParam), rtnParam );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_RemoveAdvSetCmd( uint8 adv_handle)
{
    hciStatus_t status;
    status = LL_RemoveAdvSet(adv_handle);
    HCI_CommandCompleteEvent( HCI_LE_REMOVE_ADVERTISING_SET, sizeof(status), &status );
    return( HCI_SUCCESS );
}

//
hciStatus_t HCI_LE_ClearAdvSetsCmd( void)
{
    hciStatus_t status;
    status = LL_ClearAdvSets();
    HCI_CommandCompleteEvent( HCI_LE_CLEAR_ADVERTISING_SETS, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtendedScanParametersCmd(uint8 own_address_type,
                                                uint8 scanning_filter_policy,
                                                uint8 scanning_PHYs,
                                                uint8* scan_sype,
                                                uint16* scan_interval,
                                                uint16* scan_window)
{
    hciStatus_t status;
    status = LL_SetExtendedScanParameters(own_address_type, scanning_filter_policy,
                                          scanning_PHYs, scan_sype, scan_interval, scan_window);
    //LOG("LL_SetExtendedScanParameters status:%d\n",status);
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDED_SCAN_PARAMETERS, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_SetExtendedScanEnableCmd(uint8 enable,
                                            uint8 filter_duplicates,
                                            uint16 duration,
                                            uint16 period)
{
    hciStatus_t status;
    status = LL_SetExtendedScanEnable( enable, filter_duplicates, duration, period);
    //LOG("LL_SetExtendedScanEnable status:%d\n",status);
    HCI_CommandCompleteEvent( HCI_LE_SET_EXTENDED_SCAN_ENABLE, sizeof(status), &status );
    return( HCI_SUCCESS );
}

hciStatus_t HCI_LE_ExtendedCreateConnectionCmd(uint8 initiator_filter_policy,
                                               uint8 own_address_type,
                                               uint8 peer_address_type,
                                               uint8* peer_address,
                                               uint8  initiating_PHYs,
                                               uint16* scan_interval,
                                               uint16* scan_window,
                                               uint16* conn_interval_min,
                                               uint16* conn_interval_max,
                                               uint16* conn_latency,
                                               uint16* supervision_timeout,
                                               uint16* minimum_CE_length,
                                               uint16* maximum_CE_length)
{
    hciStatus_t status;
    status = LL_ExtendedCreateConnection(initiator_filter_policy,
                                         own_address_type,
                                         peer_address_type,
                                         peer_address,
                                         initiating_PHYs,
                                         scan_interval,
                                         scan_window,
                                         conn_interval_min,
                                         conn_interval_max,
                                         conn_latency,
                                         supervision_timeout,
                                         minimum_CE_length,
                                         maximum_CE_length);
    HCI_CommandStatusEvent(status, HCI_LE_EXTENDED_CREATE_CONNECTION);
    return( HCI_SUCCESS );
}

// ===========================================================
//     extended adv HCI event functions' patch
/*******************************************************************************
    @fn          LL_ExtAdvReportCback

    @brief       This LL callback is used to



    input parameters

    @param       advEvt      - Advertise event type, or Scan Response event type.
    @param       advAddrType - Public or Random address type.
    @param       advAddr     - Pointer to device address.
    @param       dataLen     - Length of data in bytes.
    @param       advData     - Pointer to data.
    @param       rssi        - The RSSI of received packet.

    output parameters

    @param       None.

    @return      None.
*/
void LL_ExtAdvReportCback0( uint16 advEvt,
                            uint8   advAddrType,
                            uint8*  advAddr,
                            uint8   primaryPHY,
                            uint8   secondaryPHY,
                            uint8   advertisingSID,
                            uint8   txPower,
                            int8    rssi,
                            uint16  periodicAdvertisingInterval,
                            uint8   directAddrType,
                            uint8*   directAddr,
                            uint8   dataLen,
                            uint8*   rptData)
{
    #if(1)

    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_BLEExtAdvPktReport_t* pkt;
        hciEvt_ExtAdvRptInfo_t* rptInfo;
        uint8 x;
        uint8 rpt_index=0;
        uint8 datalen_fragment = 0;

        if(advEvt==LE_ADV_PROP_DATA_TRUNCATED_BITMASK && dataLen==0)
        {
            pkt = (hciEvt_BLEExtAdvPktReport_t*)osal_msg_allocate(
                      sizeof ( hciEvt_BLEExtAdvPktReport_t ) + sizeof ( hciEvt_ExtAdvRptInfo_t ) );

            if ( pkt )
            {
                pkt->hdr.event = HCI_GAP_EVENT_EVENT;
                pkt->hdr.status = HCI_LE_EVENT_CODE;
                pkt->BLEEventCode = HCI_BLE_EXT_ADV_REPORT_EVENT;
                pkt->numReports = 1;  // assume one report each event now
                pkt->rptInfo = rptInfo = (hciEvt_ExtAdvRptInfo_t*)(pkt+1);

                for ( x = 0; x < pkt->numReports; x++, rptInfo++ )
                {
                    /* Fill in the device info */
                    rptInfo->eventType = advEvt;
                    rptInfo->addrType = advAddrType;
                    (void)osal_memcpy( rptInfo->addr, advAddr, B_ADDR_LEN );
                    rptInfo->primaryPHY = primaryPHY;

                    if (secondaryPHY == PKT_FMT_BLR125K)
                        rptInfo->secondaryPHY = 0x03;           //  convert 4 -> 3
                    else
                        rptInfo->secondaryPHY = secondaryPHY;

                    rptInfo->advertisingSID = advertisingSID;
                    rptInfo->txPower = txPower;
                    rptInfo->rssi   = rssi;
                    rptInfo->periodicAdvertisingInterval = periodicAdvertisingInterval;
                    rptInfo->directAddrType = directAddrType;
                    rptInfo->dataLen = dataLen;
                    rptInfo->rssi = rssi;
                }

                (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
            }
        }

        while(rpt_index<dataLen)
        {
            if(dataLen-rpt_index>B_MAX_EXT_ADV_LEN)
            {
                datalen_fragment = B_MAX_EXT_ADV_LEN;
            }
            else
            {
                datalen_fragment = dataLen-rpt_index;
            }

            pkt = (hciEvt_BLEExtAdvPktReport_t*)osal_msg_allocate(
                      sizeof ( hciEvt_BLEExtAdvPktReport_t ) + sizeof ( hciEvt_ExtAdvRptInfo_t ) );

            if ( pkt )
            {
                pkt->hdr.event = HCI_GAP_EVENT_EVENT;
                pkt->hdr.status = HCI_LE_EVENT_CODE;
                pkt->BLEEventCode = HCI_BLE_EXT_ADV_REPORT_EVENT;
                pkt->numReports = 1;  // assume one report each event now
                pkt->rptInfo = rptInfo = (hciEvt_ExtAdvRptInfo_t*)(pkt+1);

                for ( x = 0; x < pkt->numReports; x++, rptInfo++ )
                {
                    /* Fill in the device info */
                    rptInfo->eventType = advEvt;

                    if(dataLen-rpt_index>B_MAX_EXT_ADV_LEN)
                    {
                        rptInfo->eventType |= LE_ADV_PROP_MORE_DATA_BITMASK;
                    }

                    rptInfo->addrType = advAddrType;
                    (void)osal_memcpy( rptInfo->addr, advAddr, B_ADDR_LEN );
                    rptInfo->primaryPHY = primaryPHY;

                    if (secondaryPHY == PKT_FMT_BLR125K)
                        rptInfo->secondaryPHY = 0x03;           //  convert 4 -> 3
                    else
                        rptInfo->secondaryPHY = secondaryPHY;

                    rptInfo->advertisingSID = advertisingSID;
                    rptInfo->txPower = txPower;
                    rptInfo->rssi   = rssi;
                    rptInfo->periodicAdvertisingInterval = periodicAdvertisingInterval;
                    rptInfo->directAddrType = directAddrType;

                    if (advEvt & LE_ADV_PROP_DIRECT_BITMASK)
                    {
                        (void)osal_memcpy( rptInfo->directAddr, directAddr, B_ADDR_LEN );
                    }

                    rptInfo->dataLen = datalen_fragment;
                    (void)osal_memcpy( rptInfo->rptData, &rptData[rpt_index], datalen_fragment );
                    rptInfo->rssi = rssi;
                }

                (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
            }

            rpt_index +=datalen_fragment;
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_EXT_ADV_RPT) == 0 )) )
        {
            // the event mask is not set for this event
            return;
        }

        // data length
        dataLength = HCI_EXT_ADV_REPORT_EVENT_LEN + dataLen;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_EXT_ADV_REPORT_EVENT;                // event code
            msg->pData[4] = 1;                                       // number of reports; assume 1 for now
            msg->pData[5] = LO_UINT16(advEvt);                                  // advertisement event type
            msg->pData[6] = HI_UINT16(advEvt);
            msg->pData[7] = advAddrType;                             // address type
            (void)osal_memcpy (&msg->pData[8], advAddr, B_ADDR_LEN); // address
            msg->pData[14] = primaryPHY;
            msg->pData[15] = secondaryPHY;
            msg->pData[16] = advertisingSID;
            msg->pData[17] = txPower;
            msg->pData[18] = rssi;
            msg->pData[19]  = LO_UINT16(periodicAdvertisingInterval);
            msg->pData[20]  = HI_UINT16(periodicAdvertisingInterval);
            msg->pData[21] = directAddrType;                             // address type

            if (advEvt & LE_ADV_PROP_DIRECT_BITMASK)
                (void)osal_memcpy (&msg->pData[22], directAddr, B_ADDR_LEN); // address
            else
            {
                osal_memset(&msg->pData[22], 0, B_ADDR_LEN);                // set zero
            }

            msg->pData[28] = dataLen;                                // data length
            (void)osal_memcpy (&msg->pData[29], rptData, dataLen);   // data
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }

    #endif
    #if 0

//  if( advEvt & 0x10 )
//      return;
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
//    hciEvt_BLEExtAdvPktReport_t *pkt;
//    hciEvt_ExtAdvRptInfo_t *rptInfo;
//    uint8 x;
//
//    pkt = (hciEvt_BLEExtAdvPktReport_t *)osal_msg_allocate(
//                    sizeof ( hciEvt_BLEExtAdvPktReport_t ) + sizeof ( hciEvt_ExtAdvRptInfo_t ) );
//
//    if ( pkt )
//    {
//      pkt->hdr.event = HCI_GAP_EVENT_EVENT;
//      pkt->hdr.status = HCI_LE_EVENT_CODE;
//      pkt->BLEEventCode = HCI_BLE_EXT_ADV_REPORT_EVENT;
//      pkt->numReports = 1;  // assume one report each event now
//      pkt->rptInfo = rptInfo = (hciEvt_ExtAdvRptInfo_t *)(pkt+1);
//
//      for ( x = 0; x < pkt->numReports; x++, rptInfo++ )
//      {
//        /* Fill in the device info */
//        rptInfo->eventType = advEvt;
//        rptInfo->addrType = advAddrType;
//        (void)osal_memcpy( rptInfo->addr, advAddr, B_ADDR_LEN );
//
//      rptInfo->primaryPHY = primaryPHY;
//      if (secondaryPHY == PKT_FMT_BLR125K)
//          rptInfo->secondaryPHY = 0x03;           //  convert 4 -> 3
//      else
//          rptInfo->secondaryPHY = secondaryPHY;
//      rptInfo->advertisingSID = advertisingSID;
//      rptInfo->txPower = txPower;
//      rptInfo->rssi   = rssi;
//      rptInfo->periodicAdvertisingInterval = periodicAdvertisingInterval;
//      rptInfo->directAddrType = directAddrType;
//      if (advEvt & LE_ADV_PROP_DIRECT_BITMASK)
//      {
//          (void)osal_memcpy( rptInfo->directAddr, directAddr, B_ADDR_LEN );
//      }
//
//        rptInfo->dataLen = dataLen;
//        (void)osal_memcpy( rptInfo->rptData, rptData, dataLen );
//        rptInfo->rssi = rssi;
//      }
//
//      (void)osal_msg_send( hciGapTaskID, (uint8 *)pkt );
//    }
    }
    else
    {
        hciPacket_t* msg;
        uint16 totalLength;
        uint16 dataLength;
        uint8 rptIdx = 0;
        // check if LE Meta-Events are enabled and this event is enabled
//    if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
//        (((bleEvtMask & LE_EVT_MASK_EXT_ADV_RPT) == 0 )) )
//    {
//      // the event mask is not set for this event
//      return;
//    }
        uint8 t_dataLen = dataLen;

        if( t_dataLen > 229 )
            t_dataLen = 229;

//  gpio_write(P24,1);
        // data length
extadvRpt:
        dataLength = 26 + t_dataLen;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
//    gpio_write(P25,1);gpio_write(P25,0);
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_EXT_ADV_REPORT_EVENT;                // event code
            msg->pData[4] = 1;                                       // number of reports; assume 1 for now
            msg->pData[5] = LO_UINT16(advEvt);                                  // advertisement event type
            msg->pData[6] = HI_UINT16(advEvt);
            msg->pData[7] = advAddrType;                             // address type
            (void)osal_memcpy (&msg->pData[8], advAddr, B_ADDR_LEN); // address
            msg->pData[14] = primaryPHY;
            msg->pData[15] = secondaryPHY;
            msg->pData[16] = advertisingSID;
            msg->pData[17] = txPower;
            msg->pData[18] = rssi;
            msg->pData[19]  = LO_UINT16(periodicAdvertisingInterval);
            msg->pData[20]  = HI_UINT16(periodicAdvertisingInterval);
            msg->pData[21] = directAddrType;                             // address type

            if (advEvt & LE_ADV_PROP_DIRECT_BITMASK)
                (void)osal_memcpy (&msg->pData[22], directAddr, B_ADDR_LEN); // address
            else
            {
                osal_memset(&msg->pData[22], 0, B_ADDR_LEN);                // set zero
            }

            msg->pData[28] = t_dataLen;                                // data length
            (void)osal_memcpy (&msg->pData[29], &rptData[rptIdx], t_dataLen);   // data
//printf("%d-\n",dataLen);
            // send the message
//      gpio_write(P25,1);
//    gpio_write(P25,0);
            (void)osal_msg_send1( hciTaskID, (uint8*)msg );

            if( dataLen > t_dataLen )
            {
                // only once
                dataLen = t_dataLen = dataLen - t_dataLen;
                rptIdx = 229;
                // TODO : double check, advEvt Data status shall not change
                goto extadvRpt;
            }
        }

//  else
//  {
//      gpio_write(GPIO_P34,1);
//      gpio_write(GPIO_P34,0);
//  }
//  gpio_write(P24,0);
        // DEBUG
//  else
//  {
//      gpio_write(P25,1);gpio_write(P25,0);
//      gpio_write(P25,1);gpio_write(P25,0);
//  }
    }

    #endif
}


void llSetupAdvExtIndPDU0(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv)
{
    uint8 advMode, extHeaderFlag, length, extHdrLength;
    uint8 offset = 0;

    // set AdvMode
    if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_CONN_BITMASK)
        advMode = LL_EXT_ADV_MODE_CONN;
    else if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK)
        advMode = LL_EXT_ADV_MODE_SC;
    else
        advMode = LL_EXT_ADV_MODE_NOCONN_NOSC;

    length = 0;
    // adv PDU header
    g_tx_ext_adv_buf.txheader = 0;
    // PDU type, 4 bits
    SET_BITS(g_tx_ext_adv_buf.txheader, ADV_EXT_TYPE, PDU_TYPE_SHIFT, PDU_TYPE_MASK);

    // RFU, ChSel, TxAdd, RxAdd
    if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC
            ||  pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
    {
        if ((adv_param.ownAddr[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
            SET_BITS(g_tx_ext_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
        else
            SET_BITS(g_tx_ext_adv_buf.txheader, (pAdvInfo->parameter.ownAddrType & 0x01), TX_ADD_SHIFT, TX_ADD_MASK);
    }
    else
        SET_BITS(g_tx_ext_adv_buf.txheader, pAdvInfo->parameter.ownAddrType, TX_ADD_SHIFT, TX_ADD_MASK);

    if (pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
        SET_BITS(g_tx_ext_adv_buf.txheader, 1, CHSEL_SHIFT, CHSEL_MASK);

    // === step 1. decide the extended header fields
    extHdrLength = 0;
    // extended header
    extHeaderFlag = 0;
    extHdrLength ++;       // flag

    if (pAdvInfo->isPeriodic == FALSE
            && (pAdvInfo->data.dataComplete == TRUE && pAdvInfo->data.advertisingDataLength == 0)    // no aux PDU case
            && !(pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_CONN_BITMASK)
            && !(pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK))
    {
        extHeaderFlag |= LE_EXT_HDR_ADVA_PRESENT_BITMASK;
        extHdrLength += 6;

        if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_DIRECT_BITMASK)
        {
            extHeaderFlag |= LE_EXT_HDR_TARGETA_PRESENT_BITMASK;
            extHdrLength += 6;
        }
    }
    else         // with auxilary PDU case
    {
        extHeaderFlag |= LE_EXT_HDR_ADI_PRESENT_BITMASK | LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK;
        extHdrLength += 5;
    }

    if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_TX_POWER_BITMASK)
    {
        extHeaderFlag |= LE_EXT_HDR_TX_PWR_PRESENT_BITMASK;
        extHdrLength ++;
    }

    length = 1 + extHdrLength;    // 1: extended header len(6bits) + advMode(2bit)
    // Length
    SET_BITS(g_tx_ext_adv_buf.txheader, length, LENGTH_SHIFT, LENGTH_MASK);
    // === step 2. fill extended header
    offset = 0;
    // Extended header length + AdvMode(1 octet)
    g_tx_ext_adv_buf.data[offset] = ((advMode & 0x3) << 6) | (extHdrLength & 0x3F);
    offset ++;
    g_tx_ext_adv_buf.data[offset] = extHeaderFlag;
    offset ++;

    // AdvA (6 octets)
    if (extHeaderFlag & LE_EXT_HDR_ADVA_PRESENT_BITMASK)
    {
        // 2020-11-24, the adv_param.ownAddr has been set as lastest own address, refer to LL_ProcessEvent0 & LL_SetExtAdvParam
        if (g_currentLocalAddrType  == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)  // per adv set random address may be updated after generate adv_param.ownAddr
            memcpy(&g_tx_ext_adv_buf.data[offset], pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
        else
            memcpy(&g_tx_ext_adv_buf.data[offset], adv_param.ownAddr, LL_DEVICE_ADDR_LEN);

        offset += LL_DEVICE_ADDR_LEN;
    }

    // TargetA(6 octets)
    if (extHeaderFlag & LE_EXT_HDR_TARGETA_PRESENT_BITMASK)
    {
        // TODO: peer addr type process check
        if (g_currentPeerRpa[5] != 0)      // TODO: also check g_llRlEnable???
            memcpy(&g_tx_ext_adv_buf.data[offset], &g_currentPeerRpa[0], LL_DEVICE_ADDR_LEN);
        else
            memcpy(&g_tx_ext_adv_buf.data[offset], pAdvInfo->parameter.peerAddress, LL_DEVICE_ADDR_LEN);

        offset += LL_DEVICE_ADDR_LEN;
    }

    // AdvDataInfo(ADI)(2 octets)
    if (extHeaderFlag & LE_EXT_HDR_ADI_PRESENT_BITMASK)
    {
        uint16 adi;
        adi = ((pAdvInfo->parameter.advertisingSID & 0x0F) << 12) | (pAdvInfo->data.DIDInfo & 0x0FFF);
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&adi, 2);
        offset += 2;
    }

    // AuxPtr(3 octets)
    if (extHeaderFlag & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)
    {
        uint8   chn_idx, ca, offset_unit, aux_phy;
        uint16  aux_offset;
        uint32  temp = 0;
        //20210916 channel hop setting
        chn_idx = 3 + (pAdvInfo->advHandle & 0x0F);        // temp set
        ca      = 0;                                       // 50-500ppm
        aux_phy = pAdvInfo->parameter.secondaryAdvPHY - 1;     // HCI & LL using different enum

        // for extenede adv case, the offset is calculated by auxPduRemainder
        if (pAdvInfo->isPeriodic == FALSE)
        {
            if (g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder >= 245700)
                offset_unit = 1;                                   // 300us, for aux offset >= 245700us
            else
                offset_unit = 0;                                   // 30us, for aux offset < 245700us

            //aux_offset = (g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder - elapse_time) / ((offset_unit == 1) ? 300 : 30);
            uint16 offset_adj = 15;        // temp set for aux ptr offset compensation, to move to global config
            aux_offset = g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder / ((offset_unit == 1) ? 300 : 30);
            aux_offset = (g_pAdvSchInfo[g_currentExtAdv].auxPduRemainder + offset_adj) / ((offset_unit == 1) ? 300 : 30);
        }
        // for periodic adv case, the offset is fixed 1500us + 5000 * adv channel left
        else
        {
            uint8_t temp, number;
            temp = 1 << (pPrdAdv->currentChn - LL_ADV_CHAN_FIRST);     // current bit mask
            temp = ~(temp | (temp - 1));
            temp = pAdvInfo->parameter.priAdvChnMap & temp;            // channel in the chan map to be broadcast
            number = (temp & 0x0001) +
                     ((temp & 0x0002) >> 1) +
                     ((temp & 0x0004) >> 2);
            // the interval between chan 37<->38, 38<->39 is 5000us, primary adv -> aux adv chn is 1500us
            offset_unit = 0;    // 30us, for aux offset < 245700us
            aux_offset = (number * pGlobal_config[LL_EXT_ADV_INTER_PRI_CHN_INT] + g_interAuxPduDuration /*LL_EXT_ADV_PRI_2_SEC_CHN_INT]*/) / 30;
        }

        temp |= (chn_idx & LL_AUX_PTR_CHN_IDX_MASK) << LL_AUX_PTR_CHN_IDX_SHIFT;
        temp |= (ca & LL_AUX_PTR_CA_MASK) << LL_AUX_PTR_CA_SHIFT;
        temp |= (offset_unit & LL_AUX_PTR_OFFSET_UNIT_MASK) << LL_AUX_PTR_OFFSET_UNIT_SHIFT;
        temp |= (aux_offset & LL_AUX_PTR_AUX_OFFSET_MASK) << LL_AUX_PTR_AUX_OFFSET_SHIFT;
        temp |= (aux_phy & LL_AUX_PTR_AUX_PHY_MASK) << LL_AUX_PTR_AUX_PHY_SHIFT;
        temp &= 0x00FFFFFF;
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&temp, 3);
        pAdvInfo->auxChn = chn_idx;        // save secondary channel index
        pAdvInfo->currentAdvOffset = 0;    // reset offset in adv data set
        offset += 3;
    }

    // TxPower(1 octets)
    if (extHeaderFlag & LE_EXT_HDR_TX_PWR_PRESENT_BITMASK)    // Tx power is optional, could we only filled it in AUX_ADV_IND?
    {
        int16  radio_pwr;
        radio_pwr = pAdvInfo->tx_power * 10 + g_rfTxPathCompensation;

        if (radio_pwr > 1270)   radio_pwr = 1270;

        if (radio_pwr < -1270)  radio_pwr = -1270;

        g_tx_ext_adv_buf.data[offset] = (uint8)(radio_pwr / 10);
        offset += 1;
    }

    // init adv data offset
    pAdvInfo->currentAdvOffset = 0;
}

void llSetupAuxConnectRspPDU0(extAdvInfo_t*  pAdvInfo)
{
    uint8 advMode, extHeaderFlag, length, extHdrLength;
    uint8 offset = 0;
    length = 14;
    // adv PDU header
    g_tx_adv_buf.txheader = 0;
    // PDU type, 4 bits
    SET_BITS(g_tx_adv_buf.txheader, ADV_AUX_CONN_RSP, PDU_TYPE_SHIFT, PDU_TYPE_MASK);

    // RFU, ChSel, TxAdd, RxAdd
    if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC
            ||  pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
    {
        if ((adv_param.ownAddr[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
            SET_BITS(g_tx_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
        else
            SET_BITS(g_tx_adv_buf.txheader, (pAdvInfo->parameter.ownAddrType & 0x01), TX_ADD_SHIFT, TX_ADD_MASK);
    }
    else
        SET_BITS(g_tx_adv_buf.txheader, pAdvInfo->parameter.ownAddrType, TX_ADD_SHIFT, TX_ADD_MASK);

    // Length
    SET_BITS(g_tx_adv_buf.txheader, length, LENGTH_SHIFT, LENGTH_MASK);
    offset = 0;
    extHdrLength = 13;    // ext header flag(1byte) + advA(6 octets) + targetA(6octets)
    // set AdvMode
    advMode = LL_EXT_ADV_MODE_AUX_CONN_RSP;
    g_tx_adv_buf.data[offset] = ((advMode & 0x3) << 6) | (extHdrLength & 0x3F);
    offset ++;
    // extended header
    extHeaderFlag = LE_EXT_HDR_ADVA_PRESENT_BITMASK | LE_EXT_HDR_TARGETA_PRESENT_BITMASK;
    g_tx_adv_buf.data[offset] = extHeaderFlag;
    offset ++;

//    if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)
//        memcpy(&g_tx_adv_buf.data[offset], pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
//    else    // public address
//      memcpy(&g_tx_adv_buf.data[offset], ownPublicAddr, LL_DEVICE_ADDR_LEN);

    // 2020-11-24, the adv_param.ownAddr has been set as lastest own address, refer to LL_ProcessEvent0 & LL_SetExtAdvParam
    if (g_currentLocalAddrType  == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)  // per adv set random address may be updated after generate adv_param.ownAddr
        memcpy(&g_tx_adv_buf.data[offset], pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
    else
        memcpy(&g_tx_adv_buf.data[offset], adv_param.ownAddr, LL_DEVICE_ADDR_LEN);

    offset += LL_DEVICE_ADDR_LEN;
    // TargetA(6 octets)
    osal_memcpy(&g_tx_adv_buf.data[offset], g_rx_adv_buf.data, 6);
    offset += LL_DEVICE_ADDR_LEN;
    // 2021-1-7, set RxAdd field
    SET_BITS(g_tx_adv_buf.txheader, ((g_rx_adv_buf.rxheader & TX_ADD_MASK) >> TX_ADD_SHIFT), RX_ADD_SHIFT, RX_ADD_MASK);
}



/*******************************************************************************
    @fn          LL_AdvSetTerminatedCback

    @brief       This LL callback is used to .....

    input parameters

    @param
    @param
    @param
    @param
    @param
    @param

    output parameters

    @param       None.

    @return      None.
*/
void LL_AdvSetTerminatedCback(uint8          status,
                              uint8   adv_handle,
                              uint16  connHandle,
                              uint8   Num_Completed_Extended_Advertising_Events)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_AdvSetTerminated_t* pkt;
        pkt = (hciEvt_AdvSetTerminated_t*)osal_msg_allocate(sizeof(hciEvt_AdvSetTerminated_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_LE_ADVERTISING_SET_TERMINATED;
            pkt->status      = status;
            pkt->adv_handle  = adv_handle;
            pkt->connHandle  = connHandle;
            pkt->Num_Completed_Extended_Advertising_Events = Num_Completed_Extended_Advertising_Events;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_ADV_SET_TERM) == 0 )) )
        {
            // the event mask is not set for this event
            return;
        }

        // data length
        dataLength = HCI_ADV_SET_TERM_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_LE_ADVERTISING_SET_TERMINATED;     // event code
            msg->pData[4] = status;
            msg->pData[5] = adv_handle;
            msg->pData[6] = LO_UINT16(connHandle);
            msg->pData[7] = HI_UINT16(connHandle);
            msg->pData[8] = Num_Completed_Extended_Advertising_Events;
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

// ============================================================
//    extended adv LL initial function
/*******************************************************************************
    @fn          LL_InitExtendedAdv

    @brief       The capability of extend advertiser is decided by application.
                Application should allocate the memory for extend adv and init
                LL ext advertiser



    input parameters

    @param       extAdvInfo   - pointer to memory block for extended adv info
    @param       extAdvNumber - size of the memory block for extended adv info
    @param       advSetMaxLen   - maximum adv data set length

    output parameters

    @param       None.

    @return      None.
*/
llStatus_t LL_InitialExtendedAdv( extAdvInfo_t* extAdvInfo,
                                  uint8         extAdvSetNumber,
                                  uint16        advSetMaxLen)
{
    int i;
    LL_InitExtendedAdv(extAdvInfo, extAdvSetNumber, advSetMaxLen);
    g_pExtendedAdvInfo  = extAdvInfo;
    g_extAdvNumber      = extAdvSetNumber;
    g_advSetMaximumLen  = advSetMaxLen;

    for (i = 0; i < g_extAdvNumber; i ++)
    {
        g_pExtendedAdvInfo[i].data.dataComplete = TRUE;
    }

    return( LL_STATUS_SUCCESS );
}

// ============================================================
//    hw configuration function patchs
void ll_hw_trx_settle_bb(uint8 pkt, uint8 bb_delay)
{
    if(pkt==PKT_FMT_BLE1M)
    {
        ll_hw_set_trx_settle(bb_delay,
                             pGlobal_config[LL_HW_AFE_DELAY],
                             pGlobal_config[LL_HW_PLL_DELAY]);      // TxBB, RxAFE, PLL
    }
    else if(pkt==PKT_FMT_BLE2M)
    {
        ll_hw_set_trx_settle(bb_delay,
                             pGlobal_config[LL_HW_AFE_DELAY_2MPHY],
                             pGlobal_config[LL_HW_PLL_DELAY_2MPHY]);        // TxBB, RxAFE, PLL
    }
    else if(pkt==PKT_FMT_BLR500K)
    {
        ll_hw_set_trx_settle(bb_delay,
                             pGlobal_config[LL_HW_AFE_DELAY_500KPHY],
                             pGlobal_config[LL_HW_PLL_DELAY_500KPHY]);      // TxBB, RxAFE, PLL
    }
    else
    {
        ll_hw_set_trx_settle(bb_delay,
                             pGlobal_config[LL_HW_AFE_DELAY_125KPHY],
                             pGlobal_config[LL_HW_PLL_DELAY_125KPHY]);      // TxBB, RxAFE, PLL
    }
}

///=================== for debug
void LL_slave_conn_event1(void)            // TODO: update connection context select
{
    uint16_t ll_rdCntIni;
    uint32_t      tx_num, rx_num;
    llConnState_t* connPtr;
    g_ll_conn_ctx.timerExpiryTick = read_current_fine_time();                     // A2 multiconnection
    connPtr = &conn_param[g_ll_conn_ctx.currentConn];
    // time critical process, disable interrupt
    HAL_ENTER_CRITICAL_SECTION();
    tx_num = pGlobal_config[LL_TX_PKTS_PER_CONN_EVT];
    rx_num = pGlobal_config[LL_RX_PKTS_PER_CONN_EVT];

    if (tx_num > g_maxPktPerEventTx || tx_num == 0)    tx_num = g_maxPktPerEventTx;

    if (rx_num > g_maxPktPerEventRx || rx_num == 0)    rx_num = g_maxPktPerEventRx;

    connPtr->pmCounter.ll_conn_event_cnt ++;
//    if(p_perStatsByChan!=NULL)
//        p_perStatsByChan->connEvtCnt[connPtr->currentChan]++;
    //ZQ 20191209
    //restore the currentChan for disable slavelatency
    //ZQ20200207 should use nextChan
    connPtr->lastCurrentChan = connPtr->nextChan;
    // counter for one connection event
    llResetRfCounters();
    //support rf phy change
    rf_phy_change_cfg0(connPtr->llRfPhyPktFmt);
    ll_hw_tx2rx_timing_config(connPtr->llRfPhyPktFmt);
    // reset Rx/Tx FIFO
    ll_hw_rst_rfifo();
    ll_hw_rst_tfifo();
    // channel physical configuration
    set_crc_seed(connPtr->initCRC );            // crc seed for data PDU is from CONNECT_REQ
    set_access_address(connPtr->accessAddr);     // access address
    set_channel(connPtr->currentChan );        // set channel
    set_whiten_seed(connPtr->currentChan);     // set whiten seed
    // A2-multiconn
    ll_hw_set_rx_timeout(88);

    if (connPtr->llRfPhyPktFmt == PKT_FMT_BLR125K || connPtr->llRfPhyPktFmt == PKT_FMT_BLR500K)
    {
        ll_hw_set_rx_timeout(350);
    }
    else
    {
        ll_hw_set_rx_timeout(88);
    }

    set_max_length(0xff);                  // add 2020-03-10

    // win size for 1st packet
    if (connPtr->firstPacket)     // not received the 1st packet, CRC error or correct
    {
        //ll_hw_set_rx_timeout_1st(conn_param[connId].curParam.winSize * 625 + conn_param[connId].timerDrift * 2 );
        //20180412 enlarge the connectInd or connect_update timing tolerence
        uint32_t first_window_timout=pGlobal_config[LL_SMART_WINDOW_FIRST_WINDOW] + connPtr->curParam.winSize * 625 + connPtr->timerDrift * 2 ;
        //The transmitWindowOffset shall be a multiple of 1.25 ms in the range of 0 ms
        //to connInterval. The transmitWindowSize shall be a multiple of 1.25 ms in the
        //range of 1.25 ms to the lesser of 10 ms and (connInterval - 1.25 ms).
        //ZQ 20200208
        uint32_t winSizeLimt = MIN(10000, (connPtr->curParam.connInterval * 625 - 1250) );

        if(winSizeLimt<first_window_timout)
            first_window_timout = winSizeLimt;

        ll_hw_set_rx_timeout_1st(first_window_timout);
    }
    else                  // A1 ROM metal change , 2018 - 1 - 3
    {
        if (connPtr->rx_timeout)    // timeout case
            ll_hw_set_rx_timeout_1st(pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] + pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] * 2 + (connPtr->timerDrift + connPtr->accuTimerDrift) * 2 );
        else
            ll_hw_set_rx_timeout_1st(pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT] + pGlobal_config[SLAVE_CONN_DELAY] * 2 + connPtr->timerDrift * 2 );
    }

    // configure loop timeout
    // considering the dle case
    uint32_t temp = connPtr->curParam.connInterval * 625 - connPtr->llPduLen.local.MaxRxTime- pGlobal_config[LL_HW_RTLP_TO_GAP];       // 500us: margin for timer1 IRQ
    ll_hw_set_loop_timeout(temp > pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] ?
                           pGlobal_config[LL_HW_RTLP_LOOP_TIMEOUT] : temp);         // 2018-6-20, global config for the parameter
    // now we support 4 RT in one RTLP, if PDU size is 256Byte, need (256*8 + 150) * 8 = 17684us,
    // not consider Rx packet size
    ll_hw_trx_settle_config(connPtr->llRfPhyPktFmt);
    // retransmit count limit
    ll_hw_set_loop_nack_num( 4 );
    //set the rfifo ign control
    ll_hw_ign_rfifo(LL_HW_IGN_ALL);
    // write packets to Tx FIFO
    tx_num = ll_generateTxBuffer(tx_num, &ll_rdCntIni);
    // TODO: consider Rx flow control here
//    if (LL_RX_FLOW_CONTROL_ENABLED == rxFifoFlowCtrl)
//    {
//        // configure LL HW to keep NESN
//    }
    ll_hw_config( LL_HW_RTLP,  //connPtr->llMode,
                  connPtr->sn_nesn,   // sn,nesn init
                  tx_num,                    // ll_txNum
                  rx_num,                    // ll_rxNum
                  1,                         // ll_mdRx
                  ll_rdCntIni);              // rdCntIni
    uint8 temp_rf_fmt = g_rfPhyPktFmt;
    g_rfPhyPktFmt = connPtr->llRfPhyPktFmt;
    // start LL HW engine
    ll_hw_go();
//    hal_gpio_write(GPIO_P14, 1);
//    hal_gpio_write(GPIO_P14, 0);
    llWaitingIrq = TRUE;
    g_rfPhyPktFmt = temp_rf_fmt;
    HAL_EXIT_CRITICAL_SECTION();
//    LOG("%d-%d ", g_ll_conn_ctx.numLLConns, g_ll_conn_ctx.currentConn);
    //LOG("%d ", connPtr->currentChan);
    ll_debug_output(DEBUG_LL_HW_SET_RTLP);
}

/*******************************************************************************
    @fn          LL_ScanTimeoutCback Callback

    @brief       This LL callback is used to generate a Advertisment Report meta
                event when an Advertisment or Scan Response is received by a
                Scanner.

    input parameters

    @param
                None.

    output parameters

    @param       None.

    @return      None.
*/
void LL_ScanTimeoutCback(void)
{
    // check if this is for the Host
    if ( hciGapTaskID != 0 )
    {
        hciEvt_ScanTimeout_t* pkt;
        pkt = (hciEvt_ScanTimeout_t*)osal_msg_allocate(sizeof(hciEvt_ScanTimeout_t ));

        if ( pkt )
        {
            pkt->hdr.event = HCI_GAP_EVENT_EVENT;
            pkt->hdr.status = HCI_LE_EVENT_CODE;
            pkt->BLEEventCode = HCI_BLE_SCAN_TIMEOUT_EVENT;
            (void)osal_msg_send( hciGapTaskID, (uint8*)pkt );
        }
    }
    else
    {
        hciPacket_t* msg;
        uint8 totalLength;
        uint8 dataLength;

        // check if LE Meta-Events are enabled and this event is enabled
        if ( ((pHciEvtMask[HCI_EVT_INDEX_LE] & HCI_EVT_MASK_LE) == 0) ||
                (((bleEvtMask & LE_EVT_MASK_SCAN_TO) == 0 )) )
        {
            // the event mask is not set for this event
            return;
        }

        // data length
        dataLength = HCI_SCAN_TIMEOUT_EVENT_LEN;
        // OSAL message header + HCI event header + data
        totalLength = sizeof (hciPacket_t) + HCI_EVENT_MIN_LENGTH + dataLength;
        msg = (hciPacket_t*)osal_msg_allocate(totalLength);

        if (msg)
        {
            // message type, length
            msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
            msg->hdr.status = 0xFF;
            // create message
            msg->pData    = (uint8*)(msg+1);
            msg->pData[0] = HCI_EVENT_PACKET;
            msg->pData[1] = HCI_LE_EVENT_CODE;
            msg->pData[2] = dataLength;
            // populate event
            msg->pData[3] = HCI_BLE_SCAN_TIMEOUT_EVENT;     // Subevent_Code
            // send the message
            (void)osal_msg_send( hciTaskID, (uint8*)msg );
        }
    }
}

void llSetupAuxAdvIndPDU1(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv)
{
    uint8 advMode, extHeaderFlag, length, extHdrLength, advDataLen;
    uint8 offset = 0;
//  uint32 T2, elapse_time;

    // set AdvMode
    if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_CONN_BITMASK)
        advMode = LL_EXT_ADV_MODE_CONN;
    else if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_SCAN_BITMASK)
        advMode = LL_EXT_ADV_MODE_SC;
    else
        advMode = LL_EXT_ADV_MODE_NOCONN_NOSC;

    length = 0;
    // adv PDU header
    g_tx_ext_adv_buf.txheader = 0;
    // PDU type, 4 bits
    SET_BITS(g_tx_ext_adv_buf.txheader, ADV_EXT_TYPE, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
    // RFU, ChSel, TxAdd, RxAdd
    SET_BITS(g_tx_ext_adv_buf.txheader, pAdvInfo->parameter.ownAddrType, TX_ADD_SHIFT, TX_ADD_MASK);

    if (pGlobal_config[LL_SWITCH] & CONN_CSA2_ALLOW)
        SET_BITS(g_tx_adv_buf.txheader, 1, CHSEL_SHIFT, CHSEL_MASK);

    extHdrLength = 0;
    // == step 1. decide what fields should be present in extended header
    // extended header
    extHeaderFlag = 0;
    extHdrLength ++;
    extHeaderFlag |= LE_EXT_HDR_ADI_PRESENT_BITMASK;
    extHdrLength += 2;

    if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_DIRECT_BITMASK)
    {
        extHeaderFlag |= LE_EXT_HDR_TARGETA_PRESENT_BITMASK;
        extHdrLength += 6;
    }

//  if (advMode != LL_EXT_ADV_MODE_NOCONN_NOSC)           // This field is C4 for LL_EXT_ADV_MODE_NOCONN_NOSC, we will not send AdvA in EXT_ADV_IND, so it is mandatory here
//  {
    extHeaderFlag |= LE_EXT_HDR_ADVA_PRESENT_BITMASK;
    extHdrLength += 6;
//  }

    if (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_TX_POWER_BITMASK)
    {
        extHeaderFlag |= LE_EXT_HDR_TX_PWR_PRESENT_BITMASK;
        extHdrLength ++;
    }

    if (pAdvInfo->isPeriodic == TRUE)
    {
        extHeaderFlag |= LE_EXT_HDR_SYNC_INFO_PRESENT_BITMASK;
        extHdrLength += 18;
    }

    if (pAdvInfo->data.dataComplete == TRUE)
    {
        if (advMode == LL_EXT_ADV_MODE_NOCONN_NOSC
                && (pAdvInfo->isPeriodic == FALSE)
                && (pAdvInfo->data.advertisingDataLength > 255 - 1 - extHdrLength))
        {
            extHeaderFlag |= LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK;
            extHdrLength += 3;
            // maximum payload length = 255: header len (= 1), ext header len(extHdrLength), adv data(advDataLen)
            advDataLen = 255 - 1 - extHdrLength;       // adv data length. TODO: check spec
        }
        else if(pAdvInfo->data.advertisingDataLength > 255 - 1 - extHdrLength)
        {
            advDataLen = 255 - 1 - extHdrLength;  // put all adv data in field "Adv Data"
        }
        else
        {
            advDataLen = pAdvInfo->data.advertisingDataLength;  // put all adv data in field "Adv Data"
        }
    }
    else         // update 04-13, consider update adv data case
        advDataLen = 0;

    length = 1 + extHdrLength + advDataLen;    // 1: extended header len(6bits) + advMode(2bit)
    // Length
    SET_BITS(g_tx_ext_adv_buf.txheader, length, LENGTH_SHIFT, LENGTH_MASK);
    // === step 2.  fill AUX_ADV_IND PDU
    // Extended header length + AdvMode(1 octet)
    g_tx_ext_adv_buf.data[offset] = ((advMode & 0x3) << 6) | (extHdrLength & 0x3F);
    offset ++;
    g_tx_ext_adv_buf.data[offset] = extHeaderFlag;
    offset ++;

    // AdvA (6 octets)
    if (extHeaderFlag & LE_EXT_HDR_ADVA_PRESENT_BITMASK)
    {
        if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)
            memcpy(&g_tx_ext_adv_buf.data[offset], pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
        else    // public address
            memcpy(&g_tx_ext_adv_buf.data[offset], ownPublicAddr, LL_DEVICE_ADDR_LEN);

        offset += LL_DEVICE_ADDR_LEN;
    }

    // TargetA(6 octets)
    if (extHeaderFlag & LE_EXT_HDR_TARGETA_PRESENT_BITMASK)
    {
        memcpy(&g_tx_ext_adv_buf.data[offset], pAdvInfo->parameter.peerAddress, LL_DEVICE_ADDR_LEN);
        offset += LL_DEVICE_ADDR_LEN;
    }

    // CTEInfo(1 octets), not present for AUX_ADV_IND
    if (extHeaderFlag & LE_EXT_HDR_CTE_INFO_PRESENT_BITMASK)
    {
        // offset += 1;
    }

    // AdvDataInfo(ADI)(2 octets)
    if (extHeaderFlag & LE_EXT_HDR_ADI_PRESENT_BITMASK)
    {
        uint16 adi;
        adi = ((pAdvInfo->parameter.advertisingSID & 0x0F) << 12) | (pAdvInfo->data.DIDInfo & 0x0FFF);
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&adi, 2);
        offset += 2;
    }

    // AuxPtr(3 octets)
    if (extHeaderFlag & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)
    {
        uint8   chn_idx, ca, offset_unit, aux_phy;
        uint16  aux_offset;
        uint32  temp = 0;
//        if (pAdvInfo->isPeriodic == FALSE)   // no periodic adv case
//        {
        chn_idx = llGetNextAuxAdvChn(pAdvInfo->currentChn);
        ca      = 0;        // 50-500ppm
        offset_unit = 0;    // 30us, for aux offset < 245700us
        aux_phy = pAdvInfo->parameter.secondaryAdvPHY - 1;             // HCI & LL using different enum
        aux_offset = g_interAuxPduDuration / 30;
        pAdvInfo->currentChn = chn_idx;
//        }
//      else     // AUX_PTR field is not required for periodic adv AUX_ADV_IND
//      {
//      }
        temp |= (chn_idx & LL_AUX_PTR_CHN_IDX_MASK) << LL_AUX_PTR_CHN_IDX_SHIFT;
        temp |= (ca & LL_AUX_PTR_CA_MASK) << LL_AUX_PTR_CA_SHIFT;
        temp |= (offset_unit & LL_AUX_PTR_OFFSET_UNIT_MASK) << LL_AUX_PTR_OFFSET_UNIT_SHIFT;
        temp |= (aux_offset & LL_AUX_PTR_AUX_OFFSET_MASK) << LL_AUX_PTR_AUX_OFFSET_SHIFT;
        temp |= (aux_phy & LL_AUX_PTR_AUX_PHY_MASK) << LL_AUX_PTR_AUX_PHY_SHIFT;
        temp &= 0x00FFFFFF;
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&temp, 3);
        offset += 3;
    }
    else if (pAdvInfo->isPeriodic == FALSE)   // only applicable to extended adv case
    {
        // no more aux PDU, update next channel number
        int i = 0;

        while ((i < 3) && !(pAdvInfo->parameter.priAdvChnMap & (1 << i))) i ++;

        pAdvInfo->currentChn = LL_ADV_CHAN_FIRST + i;
    }
    else    // periodic adv case
    {
        llPrdAdvDecideNextChn(pAdvInfo, pPrdAdv);
    }

    // SyncInfo(18 octets)
    if (extHeaderFlag & LE_EXT_HDR_SYNC_INFO_PRESENT_BITMASK)
    {
        // TODO
        llSetupSyncInfo(pAdvInfo, pPrdAdv);
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&syncInfo, 18);
        offset += 18;
    }

    // TxPower(1 octets)
    if (extHeaderFlag & LE_EXT_HDR_TX_PWR_PRESENT_BITMASK)    // Tx power is optional, could we only filled it in AUX_ADV_IND?
    {
        int16  radia_pwr;
        radia_pwr = pAdvInfo->tx_power * 10 + g_rfTxPathCompensation;

        if (radia_pwr > 1270)   radia_pwr = 1270;

        if (radia_pwr < -1270)  radia_pwr = -1270;

        g_tx_ext_adv_buf.data[offset] = (uint8)(radia_pwr / 10);
        offset += 1;
    }

    // ACAD(varies), not present

    // copy adv data
    if (pAdvInfo->isPeriodic == FALSE)
    {
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&pAdvInfo->data.advertisingData[pAdvInfo->currentAdvOffset], advDataLen);
        pAdvInfo->currentAdvOffset += advDataLen;
    }
}

void llSetupAuxChainIndPDU1(extAdvInfo_t*  pAdvInfo, periodicAdvInfo_t* pPrdAdv)
{
    uint8 advMode, extHeaderFlag, length, extHdrLength, advDataLen;
    uint8 offset = 0;
    // set AdvMode
    advMode = 0;
    length = 0;
    // adv PDU header
    g_tx_ext_adv_buf.txheader = 0;
    // PDU type, 4 bits
    SET_BITS(g_tx_ext_adv_buf.txheader, ADV_EXT_TYPE, PDU_TYPE_SHIFT, PDU_TYPE_MASK);
    // RFU, ChSel, TxAdd, RxAdd
    SET_BITS(g_tx_ext_adv_buf.txheader, pAdvInfo->parameter.ownAddrType, TX_ADD_SHIFT, TX_ADD_MASK);
    extHdrLength = 0;
    // extended header
    extHeaderFlag = 0;
    extHdrLength ++;

    // 2020-02-10 add for Connectionless CTE Info
    // CTEInfo field is C5: Optional
    if( pPrdAdv->PrdCTEInfo.enable == LL_CTE_ENABLE )
    {
        if( pPrdAdv->PrdCTEInfo.CTE_Count > pPrdAdv->PrdCTEInfo.CTE_Count_Idx )
        {
            extHeaderFlag |= LE_EXT_HDR_CTE_INFO_PRESENT_BITMASK;
            extHdrLength ++;
        }
    }

    // ADI field is C3, it is present in our implementation
    if (pAdvInfo->isPeriodic == FALSE)
    {
        extHeaderFlag |= LE_EXT_HDR_ADI_PRESENT_BITMASK;
        extHdrLength += 2;
    }

    // comment out because for periodic adv, tx pwr field is in AUX_SYNC_IND. for extended adv, txPwr field in AUX_ADV_IND
//  if (((pAdvInfo->isPeriodic == FALSE) && (pAdvInfo->parameter.advEventProperties & LE_ADV_PROP_TX_POWER_BITMASK))
//    || ((pAdvInfo->isPeriodic == TRUE) && (pPrdAdv->adv_event_properties & LE_ADV_PROP_TX_POWER_BITMASK)))
//  {
//      extHeaderFlag |= LE_EXT_HDR_TX_PWR_PRESENT_BITMASK;
//      extHdrLength ++;
//  }

    // if Adv Data could not be sent completely in this PDU, need AuxPtr
    if (pAdvInfo->isPeriodic == FALSE)
    {
        if ((extscanrsp_offset != 0) && (extscanrsp_offset < pAdvInfo->scanRspMaxLength))
        {
            if (pAdvInfo->scanRspMaxLength - extscanrsp_offset < 255 - 1 - extHdrLength)
                advDataLen = pAdvInfo->scanRspMaxLength - extscanrsp_offset;
            else
            {
                extHeaderFlag |= LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK;
                extHdrLength += 3;
                advDataLen = 255 - 1 - extHdrLength;
            }
        }
        else if (pAdvInfo->data.dataComplete == TRUE)
        {
            if (pAdvInfo->data.advertisingDataLength - pAdvInfo->currentAdvOffset > 255 - 1 - extHdrLength)
            {
                extHeaderFlag |= LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK;
                extHdrLength += 3;
                // maximum payload length = 255, header len = 1, ADI len = 2, AUX_PTR len = 2
                advDataLen = 255 - 1 - extHdrLength;
            }
            else
                advDataLen = pAdvInfo->data.advertisingDataLength - pAdvInfo->currentAdvOffset;  // put all remain adv data in field "Adv Data"
        }
        else      // update 04-13, adv data may be reconfigured during advertising, include no data in such case
            advDataLen = 0;
    }
    else
    {
        if (pPrdAdv->data.dataComplete == TRUE)
        {
            if (pPrdAdv->data.advertisingDataLength - pPrdAdv->currentAdvOffset > 255 - 1 - extHdrLength)
            {
                extHeaderFlag |= LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK;
                extHdrLength += 3;
                // maximum payload length = 255, header len = 1, ADI len = 2, AUX_PTR len = 2
                advDataLen = 255 - 1 - extHdrLength;;
            }
            else
                advDataLen = pPrdAdv->data.advertisingDataLength - pPrdAdv->currentAdvOffset;  // put all remain adv data in field "Adv Data"
        }
        else      // update 04-13, adv data may be reconfigured during advertising, include no data in such case
            advDataLen = 0;
    }

    length = 1 + extHdrLength + advDataLen;   // 1: extended header len(6bits) + advMode(2bit)
    // Length
    SET_BITS(g_tx_ext_adv_buf.txheader, length, LENGTH_SHIFT, LENGTH_MASK);
    // fill extended header
    offset = 0;
    // Extended header length + AdvMode(1 octet)
    g_tx_ext_adv_buf.data[offset] = ((advMode & 0x3) << 6) | (extHdrLength & 0x3F);
    offset ++;
    g_tx_ext_adv_buf.data[offset] = extHeaderFlag;
    offset ++;

    // CTEInfo(1 octets), not present for AUX_ADV_IND
    if (extHeaderFlag & LE_EXT_HDR_CTE_INFO_PRESENT_BITMASK)
    {
        g_tx_ext_adv_buf.data[offset] = (   ( pPrdAdv->PrdCTEInfo.CTE_Type << 6 ) | \
                                            ( pPrdAdv->PrdCTEInfo.CTE_Length));
        pPrdAdv->PrdCTEInfo.CTE_Count_Idx ++;
        offset += 1;
    }

    // AdvDataInfo(ADI)(2 octets)
    if (extHeaderFlag & LE_EXT_HDR_ADI_PRESENT_BITMASK)
    {
        uint16 adi;
        adi = ((pAdvInfo->parameter.advertisingSID & 0x0F) << 12) | (pAdvInfo->data.DIDInfo & 0x0FFF);
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&adi, 2);
        offset += 2;
    }

    // AuxPtr(3 octets)
    if (extHeaderFlag & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)
    {
        uint8   chn_idx, ca, offset_unit, aux_phy;
        uint16  aux_offset;
        uint32  temp = 0;
        chn_idx = llGetNextAuxAdvChn(pAdvInfo->currentChn);
        ca      = 0;        // 50-500ppm
        offset_unit = 0;    // 30us, for aux offset < 245700us
        aux_phy = pAdvInfo->parameter.secondaryAdvPHY - 1;     // HCI & LL using different enum
        aux_offset = g_interAuxPduDuration / 30;
        temp |= (chn_idx & LL_AUX_PTR_CHN_IDX_MASK) << LL_AUX_PTR_CHN_IDX_SHIFT;
        temp |= (ca & LL_AUX_PTR_CA_MASK) << LL_AUX_PTR_CA_SHIFT;
        temp |= (offset_unit & LL_AUX_PTR_OFFSET_UNIT_MASK) << LL_AUX_PTR_OFFSET_UNIT_SHIFT;
        temp |= (aux_offset & LL_AUX_PTR_AUX_OFFSET_MASK) << LL_AUX_PTR_AUX_OFFSET_SHIFT;
        temp |= (aux_phy & LL_AUX_PTR_AUX_PHY_MASK) << LL_AUX_PTR_AUX_PHY_SHIFT;
        temp &= 0x00FFFFFF;
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&temp, 3);
        pAdvInfo->currentChn = chn_idx;        // save secondary channel index
        pPrdAdv->currentChn = chn_idx;
        offset += 3;
    }
    else
    {
        // no more aux PDU, update next channel number
        if (pAdvInfo->isPeriodic == FALSE)
        {
            int i = 0;

            while ((i < 3) && !(pAdvInfo->parameter.priAdvChnMap & (1 << i))) i ++;

            pAdvInfo->currentChn = LL_ADV_CHAN_FIRST + i;
        }
        else    // periodic adv case
        {
            llPrdAdvDecideNextChn(pAdvInfo, pPrdAdv);
        }
    }

//    // TxPower(1 octets)
//    if (extHeaderFlag & LE_EXT_HDR_TX_PWR_PRESENT_BITMASK)
//    {
//        // TODO
//        offset += 1;
//    }

    // ACAD(varies), not present

    // copy adv data
    if (pAdvInfo->isPeriodic == FALSE)
    {
        if ((extscanrsp_offset != 0) && (extscanrsp_offset < pAdvInfo->scanRspMaxLength))
        {
            osal_memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&pAdvInfo->scanRspData[extscanrsp_offset], advDataLen);
            extscanrsp_offset += advDataLen;

            if (extscanrsp_offset == pAdvInfo->scanRspMaxLength)
                extscanrsp_offset = 0;
        }
        else
        {
            memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&pAdvInfo->data.advertisingData[pAdvInfo->currentAdvOffset], advDataLen);
            pAdvInfo->currentAdvOffset += advDataLen;
        }
    }
    else    // periodic adv case
    {
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&pPrdAdv->data.advertisingData[pPrdAdv->currentAdvOffset], advDataLen);
        pPrdAdv->currentAdvOffset += advDataLen;

        // if finish broad current periodic adv event, revert the read pointer of adv data
        if (pPrdAdv->currentAdvOffset == pPrdAdv->data.advertisingDataLength)
        {
            // 2020-02-10 add logic for CTE info
            if( pPrdAdv->PrdCTEInfo.enable == LL_CTE_ENABLE )
            {
                if( pPrdAdv->PrdCTEInfo.CTE_Count_Idx >= pPrdAdv->PrdCTEInfo.CTE_Count )
                {
                    // already send all CTE info, then reset pPrdAdv->currentAdvOffset
                    pPrdAdv->currentAdvOffset = 0;
                    pPrdAdv->PrdCTEInfo.CTE_Count_Idx = 0;
                    // length set to zero means stop CTE
                    ll_hw_set_cte_txSupp( CTE_SUPP_LEN_SET | 0x0 );
                }
            }
            else
            {
                // 2020-02-21 bug fix: If CTE is not enabled, then
                // subsequent packets will not be sent
                pPrdAdv->currentAdvOffset = 0;
            }
        }
    }
}

void llSetupAuxScanRspPDU1(extAdvInfo_t*  pAdvInfo)
{
    uint8 advMode, extHeaderFlag, length, extHdrLength, advDataLen;
    uint8 offset = 0;
    // set AdvMode
    advMode = 0;
    length = 0;
    // adv PDU header
    // g_tx_ext_adv_buf.txheader = 0;
    // PDU type, 4 bits
    SET_BITS(g_tx_ext_adv_buf.txheader, ADV_EXT_TYPE, PDU_TYPE_SHIFT, PDU_TYPE_MASK);

    // RFU, ChSel, TxAdd, RxAdd
//    SET_BITS(g_tx_ext_adv_buf.txheader, 0, TX_ADD_SHIFT, TX_ADD_MASK);    // it seems TxAdd is ignored in spec
    if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RPA_PUBLIC
            ||  pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RPA_RANDOM)
    {
        if ((adv_param.ownAddr[5] & RANDOM_ADDR_HDR) == PRIVATE_RESOLVE_ADDR_HDR)
            SET_BITS(g_tx_ext_adv_buf.txheader, LL_DEV_ADDR_TYPE_RANDOM, TX_ADD_SHIFT, TX_ADD_MASK);
        else
            SET_BITS(g_tx_ext_adv_buf.txheader, (pAdvInfo->parameter.ownAddrType & 0x01), TX_ADD_SHIFT, TX_ADD_MASK);
    }
    else
        SET_BITS(g_tx_ext_adv_buf.txheader, pAdvInfo->parameter.ownAddrType, TX_ADD_SHIFT, TX_ADD_MASK);

    extHdrLength = 0;
    // extended header
    extHeaderFlag = 0;          // for AUX_SYNC_IND PDU: CTE info, AuxPtr, Tx power, ACAD, Adv Data are optional, other fields are absent
    extHdrLength ++;
//  extHeaderFlag |= LE_EXT_HDR_ADVA_PRESENT_BITMASK;
    extHeaderFlag |= LE_EXT_HDR_ADVA_PRESENT_BITMASK | LE_EXT_HDR_ADI_PRESENT_BITMASK;
    extHdrLength += (6 + 2);

    if (pAdvInfo->scanRspMaxLength - extscanrsp_offset <= 100)
        advDataLen = pAdvInfo->scanRspMaxLength - extscanrsp_offset;
    else
    {
        extHeaderFlag |= LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK ;
        extHdrLength += 3;
        advDataLen = 100;
    }

    length = 1 + extHdrLength + advDataLen;   // 1: extended header len(6bits) + advMode(2bit)
    // Length
    SET_BITS(g_tx_ext_adv_buf.txheader, length, LENGTH_SHIFT, LENGTH_MASK);
    // fill extended header
    offset = 0;
    // Extended header length + AdvMode(1 octet)
    g_tx_ext_adv_buf.data[offset] = ((advMode & 0x3) << 6) | (extHdrLength & 0x3F);
    offset ++;
    g_tx_ext_adv_buf.data[offset] = extHeaderFlag;
    offset ++;

    // AdvA (6 octets)
    if (extHeaderFlag & LE_EXT_HDR_ADVA_PRESENT_BITMASK)
    {
//        if (pAdvInfo->parameter.ownAddrType == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)
//            memcpy(&g_tx_ext_adv_buf.data[offset], pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
//      else    // public address
//          memcpy(&g_tx_ext_adv_buf.data[offset], ownPublicAddr, LL_DEVICE_ADDR_LEN);

        // 2020-11-24, the adv_param.ownAddr has been set as lastest own address, refer to LL_ProcessEvent0 & LL_SetExtAdvParam
        if (g_currentLocalAddrType  == LL_DEV_ADDR_TYPE_RANDOM && pAdvInfo->parameter.isOwnRandomAddressSet == TRUE)  // per adv set random address may be updated after generate adv_param.ownAddr
            memcpy(&g_tx_ext_adv_buf.data[offset], pAdvInfo->parameter.ownRandomAddress, LL_DEVICE_ADDR_LEN);
        else
            memcpy(&g_tx_ext_adv_buf.data[offset], adv_param.ownAddr, LL_DEVICE_ADDR_LEN);

        offset += LL_DEVICE_ADDR_LEN;
    }

    // AdvDataInfo(ADI)(2 octets)
    if (extHeaderFlag & LE_EXT_HDR_ADI_PRESENT_BITMASK)
    {
        uint16 adi;
        adi = ((pAdvInfo->parameter.advertisingSID & 0x0F) << 12) | (pAdvInfo->data.DIDInfo & 0x0FFF);
        memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&adi, 2);
        offset += 2;
    }

    // AuxPtr
    if (extHeaderFlag & LE_EXT_HDR_AUX_PTR_PRESENT_BITMASK)
    {
        uint8   chn_idx, ca, offset_unit, aux_phy;
        uint16  aux_offset;
        uint32  temp = 0;
//      if (pAdvInfo->isPeriodic == FALSE)   // no periodic adv case
//      {
        chn_idx = llGetNextAuxAdvChn(pAdvInfo->currentChn);
        ca      = 0;        // 50-500ppm
        offset_unit = 0;    // 30us, for aux offset < 245700us
        aux_phy = pAdvInfo->parameter.secondaryAdvPHY - 1;             // HCI & LL using different enum
        aux_offset = g_interAuxPduDuration / 30;
        pAdvInfo->currentChn = chn_idx;
//      }
//    else     // AUX_PTR field is not required for periodic adv AUX_ADV_IND
//    {
//    }
        temp |= (chn_idx & LL_AUX_PTR_CHN_IDX_MASK) << LL_AUX_PTR_CHN_IDX_SHIFT;
        temp |= (ca & LL_AUX_PTR_CA_MASK) << LL_AUX_PTR_CA_SHIFT;
        temp |= (offset_unit & LL_AUX_PTR_OFFSET_UNIT_MASK) << LL_AUX_PTR_OFFSET_UNIT_SHIFT;
        temp |= (aux_offset & LL_AUX_PTR_AUX_OFFSET_MASK) << LL_AUX_PTR_AUX_OFFSET_SHIFT;
        temp |= (aux_phy & LL_AUX_PTR_AUX_PHY_MASK) << LL_AUX_PTR_AUX_PHY_SHIFT;
        temp &= 0x00FFFFFF;
        osal_memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&temp, 3);
        offset += 3;
    }

    // TxPwr
    // copy adv data
    osal_memcpy(&g_tx_ext_adv_buf.data[offset], (uint8*)&pAdvInfo->scanRspData[extscanrsp_offset], advDataLen);
    extscanrsp_offset += advDataLen;

    if (extscanrsp_offset == pAdvInfo->scanRspMaxLength)
        extscanrsp_offset = 0;
}

void move_to_master_function1(void)
{
    int  calibrate;
    llConnState_t* connPtr;

    if (g_llScanMode == LL_MODE_EXTENDED)
        connPtr = &conn_param[extInitInfo.connId];
    else
        connPtr = &conn_param[initInfo.connId];

    // set connection parameters
    LL_set_default_conn_params(connPtr);

    // clear the connection buffer
    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.tx_conn_desc[j] = connPtr->ll_buf.tx_not_ack_pkt;

    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.rx_conn_desc[j] = connPtr->ll_buf.tx_not_ack_pkt;

    reset_conn_buf(connPtr->connId);

    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.tx_conn_desc[j] = NULL;

    for(int j = 0; j < MAX_LL_BUF_LEN; j++)
        connPtr->ll_buf.rx_conn_desc[j] = NULL;

    initInfo.scanMode = LL_SCAN_STOP;
//  extInitInfo.scanMode = LL_SCAN_STOP;            // comment out 2020-6-12, for extended init, will be stop when receive aux_conn_rsp
    // adjust time units to 625us
    connPtr->curParam.winSize      <<= 1; // in 1.25ms units so convert to 625us
    connPtr->curParam.winOffset    <<= 1; // in 1.25ms units so convert to 625us
    connPtr->curParam.connInterval <<= 1; // in 1.25ms units so convert to 625us
    connPtr->curParam.connTimeout  <<= 4; // in 10ms units so convert to 625us
    // convert the LSTO from time to an expiration connection event count
    llConvertLstoToEvent( connPtr, &connPtr->curParam );
    // set the expiration connection event count to a specified limited number
    // Note: This is required in case the Master never sends that first packet.
    connPtr->expirationEvent = LL_LINK_SETUP_TIMEOUT;
    // convert the Control Procedure timeout into connection event count
    llConvertCtrlProcTimeoutToEvent( connPtr );
//  // calculate channel for data
//  llProcessChanMap(&conn_param[connId], conn_param[connId].chanMap );
    // need?
//  connPtr->slaveLatency = connPtr->curParam.slaveLatency ;
//  connPtr->slaveLatencyValue = connPtr->curParam.slaveLatency;
    connPtr->currentEvent               = 0;
    connPtr->nextEvent                  = 0;
    connPtr->active = 1;
    connPtr->sn_nesn = 0;                  // 1st rtlp, init sn/nesn as 0
    connPtr->llMode = LL_HW_TRLP;         // set as RTLP_1ST for the 1st connection event
    connPtr->currentChan = 0;

    if (connPtr->channel_selection == LL_CHN_SEL_ALGORITHM_1)
        connPtr->currentChan = llGetNextDataChan(connPtr, 1);
    else
    {
        // channel selection algorithm 2
        connPtr->currentChan = llGetNextDataChanCSA2(0,
                                                     (( connPtr->accessAddr & 0xFFFF0000 )>> 16 ) ^ ( connPtr->accessAddr  & 0x0000FFFF),
                                                     connPtr->chanMap,
                                                     connPtr->chanMapTable,
                                                     connPtr->numUsedChans);
    }

    llState = LL_STATE_CONN_MASTER;
    llSecondaryState = LL_SEC_STATE_IDLE;                 // add for multi-connection
    // wait to the next event start
    calibrate = pGlobal_config[LL_MOVE_TO_MASTER_DELAY];
    uint32  schedule_time;

//  margin = 2500;
    // get current allocate ID delta time to ID 0
    if (g_ll_conn_ctx.currentConn != LL_INVALID_CONNECTION_ID)
    {
        schedule_time = g_new_master_delta;// * 625;     // delta timing to previous connection slot
        ll_addTask(connPtr->connId, schedule_time);         // shcedule new connection
        g_ll_conn_ctx.scheduleInfo[connPtr->connId].task_duration = 2700;   // master task duration

        // current link id may be updated in ll_addTask, update the ll state
        if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_MASTER)
            llState = LL_STATE_CONN_MASTER;
        else if (g_ll_conn_ctx.scheduleInfo[g_ll_conn_ctx.currentConn].linkRole == LL_ROLE_SLAVE)
            llState = LL_STATE_CONN_SLAVE;
    }
    else   //  1st connection
    {
        uint32 tranWinDelay = 1250;
        uint32 connReqLen   = 352;   // legacy connect req length

        if (g_llScanMode == LL_MODE_EXTENDED)
        {
            tranWinDelay = 2500;              // using AUX_CONN_REQ, delay shoule be 2.5ms

            if (connPtr->llRfPhyPktFmt == PKT_FMT_BLR500K || connPtr->llRfPhyPktFmt == PKT_FMT_BLR125K)
            {
                tranWinDelay = 3750;
                connReqLen  = 2900;     // rough estimate for 125Kbps PHY case
            }
        }

        schedule_time = tranWinDelay + connPtr->curParam.winOffset * 625 + connReqLen + calibrate;// 352(us): CONN_REQ duration
        ll_addTask(connPtr->connId, schedule_time);
        g_ll_conn_ctx.scheduleInfo[connPtr->connId].task_duration = 2700;   // master task duration
        llState = LL_STATE_CONN_MASTER;
    }

    g_ll_conn_ctx.scheduleInfo[connPtr->connId].linkRole = LL_ROLE_MASTER;

    if (g_llScanMode != LL_MODE_EXTENDED)
        (void)osal_set_event( LL_TaskID, LL_EVT_MASTER_CONN_CREATED );

    g_pmCounters.ll_conn_succ_cnt ++;
}

void llSetupExtInit(void)    // TODO: the parameter channel idx needed?
{
    // TODO: if there are multi connections, we may need add guard code here
//    LOG("S %d ", extInitInfo.current_chn);
    // Hold off interrupts.
    HAL_ENTER_CRITICAL_SECTION( );
    g_rfPhyPktFmt = extInitInfo.current_scan_PHY;
    rf_phy_change_cfg0(extInitInfo.current_scan_PHY);
    // reset all FIFOs; all data is forfeit
    ll_hw_rst_tfifo();
    ll_hw_rst_rfifo();
    set_crc_seed(ADV_CRC_INIT_VALUE); // crc seed for adv is same for all channels
    set_access_address(ADV_SYNCH_WORD);
    set_channel(extInitInfo.current_chn);
    set_whiten_seed(extInitInfo.current_chn);
    set_max_length(0xff);
    ll_hw_set_rx_timeout(50000);
    ll_hw_set_srx();
    ll_hw_ign_rfifo(LL_HW_IGN_CRC|LL_HW_IGN_EMP);
    ll_hw_go();
    llScanT1 = read_current_fine_time();
    llWaitingIrq = TRUE;
    llTaskState = LL_TASK_EXTENDED_INIT;
    HAL_EXIT_CRITICAL_SECTION();
//    ll_debug_output(DEBUG_LL_STATE_INIT);
    return;
}


