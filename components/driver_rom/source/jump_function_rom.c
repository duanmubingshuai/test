
// this file define the call proxy of ROM function:
// 

/*******************************************************************************
 * INCLUDES
 */
#include "bus_dev.h"

#include "otp.h"
//#include "efuse.h"
#include "OSAL_PwrMgr.h"
#include "ll_sleep.h"

#include "ll_def.h"
#include "timer.h"
#include "ll_common.h"
#include "ll.h"
#include "hci_tl.h"
#include "pwrmgr.h"
#include "jump_function.h"
#include "rf_phy_driver.h"
/*******************************************************************************
 * MACROS
 */



/*******************************************************************************
 * CONSTANTS
 */



/*******************************************************************************
 * Prototypes
 */


/*******************************************************************************
 * LOCAL VARIABLES
 */


/*********************************************************************
 * EXTERNAL VARIABLES
 
 */



 #ifndef FPGA_ROM_DRIVER_TEST
/*******************************************************************************
 * Functions
 */
 
// ================= ll_hwItf.c
void move_to_slave_function(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
        
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_MOVE_TO_SLAVE_FUNCTION));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        move_to_slave_function0();
}

void LL_slave_conn_event(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SLAVE_CONN_EVENT));
    
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        LL_slave_conn_event0();
}
llStatus_t llSetupAdv(void)
{
    typedef llStatus_t (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_ADV));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();
    else
        return llSetupAdv0();     
}

void  llSetupUndirectedAdvEvt(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_UNDIRECT_ADV));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        llSetupUndirectedAdvEvt0();    
}

void llSetupNonConnectableAdvEvt( void )
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_NOCONN_ADV));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        llSetupNonConnectableAdvEvt0();    
}

void LL_evt_schedule(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_EVT_SCHEDULE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        LL_evt_schedule0();     
}

void llCalcTimerDrift( uint32    connInterval,
                        uint16   slaveLatency,
                        uint8    sleepClkAccuracy,
                        uint32   *timerDrift ) 
{
    typedef void (*my_function)(uint32, uint16, uint8, uint32 *);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_CALC_TIMER_DRIFT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connInterval, slaveLatency, sleepClkAccuracy, timerDrift);
    else
        llCalcTimerDrift0(connInterval, slaveLatency, sleepClkAccuracy, timerDrift);      
}

uint16 ll_generateTxBuffer(int txFifo_vacancy, uint16 *pSave_ptr)
{
    typedef uint16 (*my_function)(int, uint16 *);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_GENERATE_TX_BUFFER));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(txFifo_vacancy, pSave_ptr);
    else
        return ll_generateTxBuffer0(txFifo_vacancy, pSave_ptr);    
}

void ll_read_rxfifo(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_READ_RX_FIFO));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        ll_read_rxfifo0();       
}

void ll_hw_read_tfifo_rtlp(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_READ_TX_FIFO_RTLP));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        ll_hw_read_tfifo_rtlp0();     
}

int ll_hw_read_tfifo_packet(uint8 *pkt)
{
    typedef int (*my_function)(uint8 *);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_READ_TX_FIFO_PKT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(pkt);
    else
        return ll_hw_read_tfifo_packet0(pkt);      
}

void ll_hw_process_RTO(uint32 ack_num)
{
    typedef void (*my_function)(uint32);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_HW_PROCESS_RTO));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(ack_num);
    else
        ll_hw_process_RTO0(ack_num);        
}

//void LL_set_default_conn_params(llConnState_t *connPtr)
//{
//    typedef void (*my_function)(llConnState_t *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_SET_DEFAULT_CONN_PARAM));
//    
//	if (pFunc != NULL)
//		pFunc(connPtr);
//    else
//        LL_set_default_conn_params0(connPtr);       
//}

//  ================== ll_sleep.c
void enterSleepProcess(uint32 time)
{
    typedef void (*my_function)(uint32);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(ENTER_SLEEP_PROCESS));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(time);
    else
        enterSleepProcess0(time);        
}


void wakeupProcess(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
//    extern int phy_sec_otp_lock(void);
//    phy_sec_otp_lock();
    __disable_cache();//disable cache when wakeup
    pFunc = (my_function)(JUMP_FUNCTION_GET(WAKEUP_PROCESS));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        wakeupProcess0();        
}

void config_RTC(uint32 time)
{
    typedef void (*my_function)(uint32);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(CONFIG_RTC));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(time);
    else
        config_RTC0(time);        
}   

void enter_sleep_off_mode(Sleep_Mode mode)
{
    typedef void (*my_function)(Sleep_Mode);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(ENTER_SLEEP_OFF_MODE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(mode);
    else
        enter_sleep_off_mode0(mode);      
}

// ======================= ll_SlaveEndCauses.c
void llSlaveEvt_TaskEndOk( void )
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SLAVE_EVT_ENDOK));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        llSlaveEvt_TaskEndOk0();  
}

uint8 llSetupNextSlaveEvent( void )
{
    typedef uint8 (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_NEXT_SLAVE_EVT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();
    else
        return llSetupNextSlaveEvent0();  
}

//uint8 llCheckForLstoDuringSL( llConnState_t *connPtr )
//{
//    typedef uint8 (*my_function)(llConnState_t *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_CHK_LSTO_DURING_SL));
//    
//	if (pFunc != NULL)
//		return pFunc(connPtr);
//    else
//        return llCheckForLstoDuringSL0(connPtr);      
//}

uint8 llProcessSlaveControlProcedures( llConnState_t *connPtr )
{
    typedef uint8 (*my_function)(llConnState_t *);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_PROCESS_SLAVE_CTRL_PROC));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(connPtr);
    else
        return llProcessSlaveControlProcedures0(connPtr);        
}

void llProcessSlaveControlPacket( llConnState_t *connPtr,
                                  uint8         *pBuf )
{
    typedef void (*my_function)(llConnState_t *, uint8 *);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_PROCESS_SLAVE_CTRL_PKT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connPtr, pBuf);
    else
        llProcessSlaveControlPacket0(connPtr, pBuf);      
}

//void llSlaveEvt_TaskAbort( void )
//{
//    typedef void (*my_function)(void);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_SLAVE_EVT_ABORT));
//    
//	if (pFunc != NULL)
//		pFunc();
//    else
//        llSlaveEvt_TaskAbort0();      
//}

//void llSetupScannableAdvEvt( void )
//{
//    typedef void (*my_function)(void);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_SETUP_SCAN_ADV));
//    
//	if (pFunc != NULL)
//		pFunc();
//    else
//        llSetupScannableAdvEvt0();     
//}

void llSetupDirectedAdvEvt( void )
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_DIRECT_ADV));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        llSetupDirectedAdvEvt0();      
}

// ===================== ll_MasterEndCauses.c
void llMasterEvt_TaskEndOk( void )
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_MASTER_EVT_ENDOK));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        llMasterEvt_TaskEndOk0();      
}

//void llProcessMasterControlPacket( llConnState_t *connPtr,
//                                   uint8         *pBuf )
//{
//    typedef void (*my_function)(llConnState_t *, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_PROCESS_MASTER_CTRL_PKT));
//    
//	if (pFunc != NULL)
//		pFunc(connPtr, pBuf);
//    else
//        llProcessMasterControlPacket0(connPtr, pBuf);      
//}

//uint8 llProcessMasterControlProcedures( llConnState_t *connPtr )
//{
//    typedef uint8 (*my_function)(llConnState_t *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_PROCESS_MASTER_CTRL_PROC));
//    
//	if (pFunc != NULL)
//		return pFunc(connPtr);
//    else
//        return llProcessMasterControlProcedures0(connPtr);        
//}

uint8 llSetupNextMasterEvent( void )
{
    typedef uint8 (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_NEXT_MASTER_EVT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();
    else
        return llSetupNextMasterEvent0();      
}    

void move_to_master_function( void )
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_MOVE_TO_MASTER_FUNCTION));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        move_to_master_function0();      
}   

void LL_master_conn_event( void )
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_MASTER_CONN_EVENT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        LL_master_conn_event0();      
}   

llStatus_t LL_SetScanControl( uint8 scanMode,
                              uint8 filterReports )
{
    typedef llStatus_t (*my_function)(uint8, uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_SCAN_CTRL));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(scanMode, filterReports);
    else
        return LL_SetScanControl0(scanMode, filterReports);       
}

llStatus_t LL_SetScanParam( uint8  scanType,
                            uint16 scanInterval,
                            uint16 scanWindow,
                            uint8  ownAddrType,
                            uint8  scanWlPolicy )
{
    typedef llStatus_t (*my_function)(uint8, uint16, uint16, uint8, uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_SCAN_PARAM));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(scanType, scanInterval, scanWindow, ownAddrType, scanWlPolicy);
    else
        return LL_SetScanParam0(scanType, scanInterval, scanWindow, ownAddrType, scanWlPolicy);       
}

llStatus_t LL_CreateConn( uint16 scanInterval,
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
                          uint16 maxLength )
{
    typedef llStatus_t (*my_function)(uint16, uint16, uint8, uint8, uint8 *, uint8, uint16, uint16, uint16, uint16, uint16, uint16);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_CREATE_CONN));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(scanInterval, scanWindow, initWlPolicy, peerAddrType, peerAddr, ownAddrType, connIntervalMin, connIntervalMax, connLatency, connTimeout, minLength, maxLength);
    else
        return LL_CreateConn0(scanInterval, scanWindow, initWlPolicy, peerAddrType, peerAddr, ownAddrType, connIntervalMin, connIntervalMax, connLatency, connTimeout, minLength, maxLength);      
}

llStatus_t LL_CreateConnCancel( void )
{
    typedef llStatus_t (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_CREATE_CONN_CANCEL));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();
    else
        return LL_CreateConnCancel0();       
}

//llStatus_t LL_StartEncrypt( uint16 connId,
//                            uint8  *rand,
//                            uint8  *eDiv,
//                            uint8  *ltk )
//{
//    typedef llStatus_t (*my_function)(uint16, uint8 *, uint8 *, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_START_ENCRYPT));
//    
//	if (pFunc != NULL)
//		return pFunc(connId, rand, eDiv, ltk);
//    else
//        return LL_StartEncrypt0(connId, rand, eDiv, ltk);
//}

void llSetupScan( uint8 chan )
{
    typedef void (*my_function)(uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_SCAN));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(chan);
    else
        llSetupScan0(chan);      
}   

// ============================== ll.c
//void LL_Init( uint8 taskId )
//{
//    typedef void (*my_function)(uint8);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_INIT));
//    
//	if (pFunc != NULL)
//		pFunc(taskId);
//    else
//        LL_Init0(taskId);         
//}

//uint16 LL_ProcessEvent( uint8 task_id, uint16 events )
//{
//    typedef uint16 (*my_function)(uint8, uint16);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_PROCESS_EVENT));
//    
//	if (pFunc != NULL)
//		return pFunc(task_id, events);
//    else
//        return LL_ProcessEvent0(task_id, events);   
//}

llStatus_t LL_Reset( void )
{
    typedef llStatus_t (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_RESET));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();
    else
        return LL_Reset0();     
}

llStatus_t LL_TxData( uint16 connId,
                      uint8 *pBuf,
                      uint8  pktLen,
                      uint8  fragFlag )
{
    typedef llStatus_t (*my_function)(uint16 ,
                      uint8 *,
                      uint8  ,
                      uint8  );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_TXDATA));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(connId, pBuf, pktLen, fragFlag);
    else
        return LL_TxData0(connId, pBuf, pktLen, fragFlag);      
}

llStatus_t LL_Disconnect( uint16 connId, uint8  reason )
{
    typedef llStatus_t (*my_function)(uint16 , uint8  );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_DISCONNECT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(connId, reason);
    else
        return LL_Disconnect0(connId, reason);     
}

llStatus_t LL_SetAdvParam( uint16 advIntervalMin,
                           uint16 advIntervalMax,
                           uint8  advEvtType,
                           uint8  ownAddrType,
                           uint8  directAddrType,
                           uint8  *directAddr,
                           uint8  advChanMap,
                           uint8  advWlPolicy )
{
    typedef llStatus_t (*my_function)(uint16, uint16, uint8, uint8, uint8, uint8*, uint8, uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_ADV_PARAM));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(advIntervalMin, advIntervalMax, advEvtType, ownAddrType, directAddrType, directAddr, advChanMap, advWlPolicy);
    else
        return LL_SetAdvParam0(advIntervalMin, advIntervalMax, advEvtType, ownAddrType, directAddrType, directAddr, advChanMap, advWlPolicy);        
}

llStatus_t LL_SetAdvData( uint8  advDataLen, uint8 *advData )
{
    typedef llStatus_t (*my_function)(uint8 , uint8 *  );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_ADV_DATA));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(advDataLen, advData);
    else
        return LL_SetAdvData0(advDataLen, advData);     
}

llStatus_t LL_SetAdvControl( uint8 advMode )
{
    typedef llStatus_t (*my_function)(uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_ADV_CONTROL));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(advMode);
    else
        return LL_SetAdvControl0(advMode);     
}

//llStatus_t LL_EXT_SetTxPower( uint8 txPower, uint8 *cmdComplete )
//{
//    typedef llStatus_t (*my_function)(uint8, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_EXT_SET_TX_POWER));
//    
//	if (pFunc != NULL)
//		return pFunc(txPower, cmdComplete);
//    else
//        return LL_EXT_SetTxPower0(txPower, cmdComplete);      
//}


//llStatus_t LL_ReadAdvChanTxPower( int8 *txPower )
//{
//    typedef llStatus_t (*my_function)(int8  *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_READ_ADV_TX_PWR_LVL));
//    
//	if (pFunc != NULL)
//		return pFunc(txPower);
//    else
//        return LL_ReadAdvChanTxPower0(txPower);          
//}

//llStatus_t LL_ReadTxPowerLevel( uint8 connId,
//                                uint8 type,
//                                int8  *txPower )
//{
//    typedef llStatus_t (*my_function)(uint8, uint8, int8  *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_READ_TX_PWR_LVL));
//    
//	if (pFunc != NULL)
//		return pFunc(connId, type, txPower);
//    else
//        return LL_ReadTxPowerLevel0(connId, type, txPower);      
//}

//llStatus_t LL_SetTxPowerLevel(int8  txPower )
//{
//    typedef llStatus_t (*my_function)(int8  txPower);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_SET_TX_PWR_LVL));
//    
//	if (pFunc != NULL)
//		return pFunc(txPower);
//    else
//        return LL_SetTxPowerLevel0(txPower);      
//}

//llStatus_t LL_ReadRssi( uint16 connId,
//                        int8   *lastRssi )
//{
//    typedef llStatus_t (*my_function)(uint16, int8  *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_READ_RSSI));
//    
//	if (pFunc != NULL)
//		return pFunc(connId, lastRssi);
//    else
//        return LL_ReadRssi0(connId, lastRssi);       
//}

//llStatus_t LL_ReadRemoteUsedFeatures( uint16 connId )
//{
//    typedef llStatus_t (*my_function)(uint16);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_READ_REMOTE_USE_FEATURES));
//    
//	if (pFunc != NULL)
//		return pFunc(connId);
//    else
//        return LL_ReadRemoteUsedFeatures0(connId);       
//}

//llStatus_t LL_Encrypt( uint8 *key,
//                       uint8 *plaintextData,
//                       uint8 *encryptedData )
//{
//    typedef llStatus_t (*my_function)(uint8 *, uint8 *, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_ENCRYPT));
//    
//	if (pFunc != NULL)
//		return pFunc(key, plaintextData, encryptedData);
//    else
//        return LL_Encrypt0(key, plaintextData, encryptedData);       
//}

// ============= ll_common.c
void llProcessTxData( llConnState_t *connPtr, uint8 context )
{
    typedef void (*my_function)(llConnState_t *, uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_PROCESS_TX_DATA));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connPtr, context);
    else
        llProcessTxData0(connPtr, context);        
}

uint8 llProcessRxData( void )
{
    typedef uint8 (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_PROCESS_RX_DATA));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();
    else
        return llProcessRxData0();       
}

void llConnTerminate( llConnState_t *connPtr,
                      uint8          reason )
{
//    uart_tx0(" term ");
    typedef void (*my_function)(llConnState_t *, uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_CONN_TERMINATE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connPtr, reason);
    else
        llConnTerminate0(connPtr, reason);       
}

uint8 llWriteTxData ( llConnState_t *connPtr,
                      uint8          pktHdr,
                      uint8          pktLen,
                      uint8         *pBuf )
{
    typedef uint8 (*my_function)(llConnState_t * ,
                      uint8 ,
                      uint8 ,
                      uint8 * );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_WRITE_TX_DATA));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(connPtr, pktHdr, pktLen, pBuf);
    else
        return llWriteTxData0(connPtr, pktHdr, pktLen, pBuf);       
}

void llReleaseConnId( llConnState_t *connPtr )
{
    typedef void (*my_function)(llConnState_t *);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_RELEASE_CONN_ID));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connPtr);
    else
        llReleaseConnId0(connPtr);      
}

// ======================== ll_enc.c
//void LL_ENC_AES128_Encrypt( uint8 *key,
//                            uint8 *plaintext,
//                            uint8 *ciphertext )
//{
//    typedef void (*my_function)(uint8 *, uint8 *, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_AES128_ENCRYPT));
//    
//	if (pFunc != NULL)
//		pFunc(key, plaintext, ciphertext);
//    else
//        LL_ENC_AES128_Encrypt0(key, plaintext, ciphertext);       
//}

//uint8 LL_ENC_GenerateTrueRandNum( uint8 *buf,
//                                  uint8 len )
//{
//    typedef uint8 (*my_function)(uint8 *, uint8);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_GEN_TRUE_RANDOM));
//    
//	if (pFunc != NULL)
//		return pFunc(buf, len);
//    else
//        return LL_ENC_GenerateTrueRandNum0(buf, len);       
//}

//void LL_ENC_GenDeviceSKD( uint8 *SKD )
//{
//    typedef void (*my_function)(uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_GEN_DEVICE_SKD));
//    
//	if (pFunc != NULL)
//		pFunc(SKD);
//    else
//        LL_ENC_GenDeviceSKD0(SKD);       
//}

//void LL_ENC_GenDeviceIV( uint8 *IV )
//{
//    typedef void (*my_function)(uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_GEN_DEVICE_IV));
//    
//	if (pFunc != NULL)
//		pFunc(IV);
//    else
//        LL_ENC_GenDeviceIV0(IV);       
//}
//
//void LL_ENC_GenerateNonce( uint32 pktCnt,
//                           uint8  direction,
//                           uint8  *nonce )
//{
//    typedef void (*my_function)(uint32, uint8, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_GENERATE_NOUNCE));
//    
//	if (pFunc != NULL)
//		pFunc(pktCnt, direction, nonce);
//    else
//        LL_ENC_GenerateNonce0(pktCnt, direction, nonce);      
//}
//
//void LL_ENC_Encrypt( llConnState_t *connPtr,
//                     uint8          pktHdr,
//                     uint8          pktLen,
//                     uint8         *pBuf )
//{
//    typedef void (*my_function)(llConnState_t *, uint8, uint8, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_ENC_ENCRYPT));
//    
//	if (pFunc != NULL)
//		pFunc(connPtr, pktHdr, pktLen, pBuf);
//    else
//        LL_ENC_Encrypt0(connPtr, pktHdr, pktLen, pBuf);      
//}
//
//uint8 LL_ENC_Decrypt( llConnState_t *connPtr,
//                      uint8          pktHdr,
//                      uint8          pktLen,
//                      uint8         *pBuf )
//{
//    typedef uint8 (*my_function)(llConnState_t *, uint8, uint8, uint8 *);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_ENC_DECRYPT));
//    
//	if (pFunc != NULL)
//		return pFunc(connPtr, pktHdr, pktLen, pBuf);
//    else
//        return LL_ENC_Decrypt0(connPtr, pktHdr, pktLen, pBuf);      
//}


void osal_pwrmgr_powerconserve( void ) 
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(OSAL_POWER_CONSERVE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        osal_pwrmgr_powerconserve0();      
}

void osal_mem_init(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(OSAL_MEM_INIT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        osal_mem_init0();      
    
}

// =============== ll_hw_drv.c
//void ll_hw_set_timing(uint8 pktFmt)
//{
//    typedef void (*my_function)(uint8);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_HW_SET_TIMING));
//    
//	if (pFunc != NULL)
//		pFunc(pktFmt);
//    else
//        ll_hw_set_timing0(pktFmt);     
//}

void ll_hw_go(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_HW_GO));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();  
    else
        ll_hw_go0();        
}

//void ll_hw_trigger(void)
//{
//    typedef void (*my_function)(void);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(LL_HW_TRIGGER));
//    
//	if (pFunc != NULL)
//		pFunc();  
//    else
//        ll_hw_trigger0();        
//}

// ========================= call APP function
void app_sleep_process(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(APP_SLEEP_PROCESS));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        pwrmgr_sleep_process();

}

void app_wakeup_process(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(APP_WAKEUP_PROCESS));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        pwrmgr_wakeup_process();
}

void rf_init(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(RF_INIT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(); 
    else
        rf_phy_ini();     
}

void rf_phy_change_cfg(uint8 pktFmt)
{
    typedef void (*my_function)(uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(RF_PHY_CHANGE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(pktFmt);    
    else
        rf_phy_change_cfg0(pktFmt);  
}

void boot_init(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(BOOT_INIT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();    
    else
        boot_init0();
}

void wakeup_init(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(WAKEUP_INIT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();  
    else
        wakeup_init0();       
}

void rf_calibrate(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(RF_CALIBRATTE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();  
    else
        rf_calibrate0();       
}

// application project may trigger GPIO, uart, ..., etc. base on the variable "state"
//void debug_print(uint32 state)
//{
//    typedef void (*my_function)(uint32);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(DEBUG_PRINT));
//    
//	if (pFunc != NULL)
//		pFunc(state);  
// 
//}
#if __BUILD_INCLUDE_HOST__ 
// ==================== SMP functions
//void SM_Init( uint8 task_id )
//{
//    typedef void (*my_function)(uint8);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(SMP_INIT));
//    
//	if (pFunc != NULL)
//		pFunc(task_id);   
//    else
//        SM_Init0(task_id);       
//}

//uint16 SM_ProcessEvent( uint8 task_id, uint16 events )
//{
//    typedef uint16 (*my_function)(uint8, uint16);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(SMP_PROCESS_EVENT));
//    
//	if (pFunc != NULL)
//		return pFunc(task_id, events);
//    else
//        return SM_ProcessEvent0(task_id, events);     
//}
#endif

// ==================== HCI_TL functions
//void HCI_Init( uint8 task_id )
//{
//    typedef void (*my_function)(uint8);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(HCI_INIT));
//    
//	if (pFunc != NULL)
//		pFunc(task_id);   
//    else
//        HCI_Init0(task_id);       
//}

//uint16 HCI_ProcessEvent( uint8 task_id, uint16 events )
//{
//    typedef uint16 (*my_function)(uint8, uint16);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(HCI_PROCESS_EVENT));
//    
//	if (pFunc != NULL)
//		return pFunc(task_id, events);
//    else
//        return HCI_ProcessEvent0(task_id, events);     
//}

uint32 llCalcMaxScanTime(void)
{
    typedef uint32 (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_CALC_MAX_SCAN_TIME));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();   
    else
        return llCalcMaxScanTime0();       
}

void llSetupSecScan( uint8 chan )
{
    typedef void (*my_function)(uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SETUP_SEC_SCAN));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(chan);   
    else
        llSetupSecScan0(chan);       
}    
//#if __BUILD_INCLUDE_HOST__ 
//=============== gap_linkmgr.c
void gapProcessDisconnectCompleteEvt( hciEvt_DisconnComplete_t *pPkt )
{
    typedef void (*my_function)(hciEvt_DisconnComplete_t * );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(GAP_LINK_MGR_PROCESS_DISCONNECT_EVT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(pPkt);
    else
        gapProcessDisconnectCompleteEvt0(pPkt); 
}

void gapProcessConnectionCompleteEvt( hciEvt_BLEConnComplete_t *pPkt )
{
    typedef void (*my_function)(hciEvt_BLEConnComplete_t * );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(GAP_LINK_MGR_PROCESS_CONNECT_EVT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(pPkt);
    else
        gapProcessConnectionCompleteEvt0(pPkt); 
}

//=============== l2cap_util.c
uint8 l2capParsePacket( l2capPacket_t *pPkt, hciDataEvent_t *pHciMsg )
{
    typedef uint8 (*my_function)(l2capPacket_t* , hciDataEvent_t* );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(L2CAP_PARSE_PACKET));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(pPkt, pHciMsg);
    else
        return l2capParsePacket0(pPkt, pHciMsg);    

}


bStatus_t l2capEncapSendData( uint16 connHandle, l2capPacket_t *pPkt )
{
    typedef bStatus_t (*my_function)(uint16 , l2capPacket_t* );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(L2CAP_ENCAP_PACKET));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(connHandle, pPkt);
    else
        return l2capEncapSendData0(connHandle, pPkt);    

}

//uint8 l2capPktToSegmentBuff(uint16 connHandle,l2capSegmentBuff_t* pSegBuf, uint8 blen,uint8* pBuf)
//{
//    typedef uint8 (*my_function)(uint16 ,l2capSegmentBuff_t* , uint8,uint8*);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(L2CAP_PKT_TO_SEGBUFF));
//    
//	if (pFunc != NULL)
//		return pFunc(connHandle,pSegBuf,blen,pBuf);
//    else
//        return l2capPktToSegmentBuff0(connHandle,pSegBuf,blen,pBuf);   
//
//}

//uint8 l2capSegmentBuffToLinkLayer(uint16 connHandle, l2capSegmentBuff_t* pSegBuf)
//{
//    typedef uint8 (*my_function)(uint16 , l2capSegmentBuff_t*);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(L2CAP_SEGBUFF_TO_LINKLAYER));
//    
//	if (pFunc != NULL)
//		return pFunc(connHandle,pSegBuf);
//    else
//        return l2capSegmentBuffToLinkLayer0(connHandle,pSegBuf);   
//
//}


//void l2capPocessFragmentTxData(uint16 connHandle)
//{
//    typedef void (*my_function)(uint16 );
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(L2CAP_PROCESS_FREGMENT_TX_DATA));
//    
//	if (pFunc != NULL)
//		pFunc(connHandle);
//    else
//        l2capPocessFragmentTxData0(connHandle); 
//    
//}
//#else
//void l2capPocessFragmentTxData(uint16 connHandle)
//{
//    typedef void (*my_function)(uint16 );
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(L2CAP_PROCESS_FREGMENT_TX_DATA));
//    
//	if (pFunc != NULL)
//		pFunc(connHandle);    
//    
//}
//#endif


void ll_adptive_adj_next_time(uint32 nextTime)
{
    typedef void (*my_function)(uint32 );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_ADP_ADJ_NEXT_TIME));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(nextTime);
    else
        ll_adptive_adj_next_time0(nextTime); 
}


void llSetNextDataChan( llConnState_t *connPtr )
{
    typedef void (*my_function)(llConnState_t * );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_NEXT_DATA_CHN));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connPtr);
    else
        llSetNextDataChan0(connPtr);
}

llStatus_t LL_PLUS_DisableSlaveLatency(uint8 connId)
{
    typedef llStatus_t (*my_function)(uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_PLUS_DISABLE_LATENCY));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return(pFunc(connId));
    else
        return(LL_PLUS_DisableSlaveLatency0(connId));
}

llStatus_t LL_PLUS_EnableSlaveLatency(uint8 connId)
{
    typedef llStatus_t (*my_function)(uint8);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_PLUS_ENABLE_LATENCY));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
        return (pFunc(connId));
    else
        return(LL_PLUS_EnableSlaveLatency0(connId));
}

void ll_scheduler(uint32 time)
{
    typedef void (*my_function)(uint32 time);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SCHEDULER));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(time);
    else
        ll_scheduler0(time);
}

void ll_addTask(uint8 connId, uint32 time)
{
    typedef void (*my_function)(uint8 connId, uint32 time);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_ADD_TASK));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connId, time);
    else
        ll_addTask0(connId, time);
}

void ll_deleteTask(uint8 connId)
{
    typedef void (*my_function)(uint8 connId);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_DEL_TASK));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc(connId);
    else
        ll_deleteTask0(connId);

}


//=============== OSAL
uint8 osal_set_event( uint8 task_id, uint16 event_flag )
{
    typedef uint8 (*my_function)(uint8,uint16);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(OSAL_SET_EVENT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(task_id,event_flag);
    else
        return osal_set_event0(task_id,event_flag); 
}

uint8 osal_msg_send( uint8 destination_task, uint8 *msg_ptr )
{
    typedef uint8 (*my_function)(uint8,uint8*);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(OSAL_MSG_SEND));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(destination_task,msg_ptr);
    else
        return osal_msg_send0(destination_task,msg_ptr); 
}
#endif

//=============== _HAL_IRQ_
void drv_irq_init(void)
{
    typedef void (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(HAL_DRV_IRQ_INIT));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc();
    else
        drv_irq_init0();
}

int drv_enable_irq(int cs)
{
    typedef int (*my_function)(int);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(HAL_DRV_IRQ_ENABLE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc(cs);
    else
       return drv_enable_irq0(cs);
}

int drv_disable_irq(void)
{
    typedef int (*my_function)(void);
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(HAL_DRV_IRQ_DISABLE));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		return pFunc();
    else
        return drv_disable_irq0();
}

void __wdt_init(void)
{
    typedef void (*my_function)(void );
    my_function pFunc = NULL;
    pFunc = (my_function)(JUMP_FUNCTION_GET(HAL_WATCHDOG_INIT));

    if (pFunc != ((my_function)SRAM_BASE_ADDR))
        pFunc();
}

void set_channel(uint32_t  channel)
{
    typedef void (*my_function)(uint32_t );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_CHANNEL));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc( channel );
    else
        set_channel0( channel );
}

void set_whiten_seed(uint32_t channel)
{
    typedef void (*my_function)(uint32_t );
    my_function pFunc = NULL;
    
    pFunc = (my_function)(JUMP_FUNCTION_GET(LL_SET_WHITEN_SEED));
    
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc( channel );
    else
        set_whiten_seed0( channel );
}

void _clk_apply_setting(uint32_t hclk_sel, uint32_t digi_clk_en, uint32_t digi32_sel,uint8_t wfi_sel)
{
    typedef void (*my_function)(uint32_t,uint32_t,uint32_t,uint8_t );
    my_function pFunc = NULL;
    pFunc = (my_function)(JUMP_FUNCTION_GET(CLK_APPLY_SETTING));
	if (pFunc != ((my_function)SRAM_BASE_ADDR))
		pFunc( hclk_sel,digi_clk_en,digi32_sel,wfi_sel);
    else
        _clk_apply_setting0( hclk_sel,digi_clk_en,digi32_sel,wfi_sel );
}

void JUMP_FUNCTION_SET(uint8 jidx,uint32 val)
{
	uint8 inValid = TRUE;
	if( jidx < JUMP_FUNC_NUM)
	{
		if( ( 0 == val ) || (JUMP_BASE_ADDR == (val & JUMP_BASE_ADDR)) )
		{
			inValid = FALSE;
			*(uint16 *)(JUMP_BASE_ADDR + ( jidx << 1 ) ) = (uint16)(val & 0xffff);
		}			
	}

	if( inValid )
		while(1);
	
}

//#if(PHY_MCU_TYPE==MCU_BUMBEE_CK802)
//void krhino_intrpt_enter_hook(int irq)
//{
//    typedef void (*my_function)(int);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(RHINO_INTRPT_ENTER_HOOK));
//    
//	if (pFunc != NULL)
//	    pFunc(irq);
//    
//} 
//void krhino_intrpt_exit_hook(int irq)
//{
//    typedef void (*my_function)(int);
//    my_function pFunc = NULL;
//    
//    pFunc = (my_function)(JUMP_FUNCTION(RHINO_INTRPT_EXIT_HOOK));
//    
//	if (pFunc != NULL)
//	    pFunc(irq);
//    
//}
//#endif

