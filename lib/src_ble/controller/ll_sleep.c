

/*******************************************************************************
 * INCLUDES
 */
#include "bus_dev.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_Clock.h"
#include "ll_sleep.h"

#include "ll_def.h"
#include "timer.h"
#include "ll_common.h"
#include "jump_function.h"
#include "global_config.h"
#include "ll_sleep.h"
#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "rf_phy_driver.h"
#include "ate.h"

/*******************************************************************************
 * MACROS
 */
#define      SRAM0_ADDRESS        0x1fff0f00
#define      SRAM1_ADDRESS        0x1fff9000

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * Prototypes
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */
uint32 sleep_flag = 0;                   // when sleep, set this value to SLEEP_MAGIC. when wakeup, set this value to 0
uint32_t  g_wakeup_rtc_tick = 0;
uint32_t  g_counter_traking_avg       = 7812;//defualt 16 rtc cnt

//used for sleep timer sync 
uint32_t  g_TIM2_IRQ_TIM3_CurrCount  =   0;
uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick=0;
uint32_t  g_TIM2_IRQ_PendingTick=0;
uint32_t  g_osal_tick_trim=0;
uint32_t  g_osalTickTrim_mod=0;
uint32_t  g_TIM2_wakeup_delay=0;
uint32_t  rtc_mod_value = 0;
uint16_t  g_counter_traking_cnt = 0;
uint32_t  sleep_tick;
uint32_t  counter_tracking=7812;                // 24bit tracking counter, read from 0x4000f064
uint8_t s_rom_wakeup_flg = 0;
/*******************************************************************************
 * LOCAL VARIABLES
 */
static Sleep_Mode sleepMode = SYSTEM_SLEEP_MODE;// MCU_SLEEP_MODE;
static uint8 bSleepAllow = TRUE;


volatile uint32_t forever_write;


/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32 ll_remain_time;
extern uint32 osal_sys_tick;
extern uint8  llState;

 
/*******************************************************************************
 * Functions
 */
 
extern void wakeup_init(void);
extern void set_flash_deep_sleep(void);

extern void ll_hw_tx2rx_timing_config(uint8 pkt);
extern void ll_hw_trx_settle_config(uint8 pkt);

// =========================== sleep mode configuration functions
// is sleep allow
uint8 isSleepAllow(void)
{
    return bSleepAllow;
}

// enable sleep
void enableSleep(void)
{
    bSleepAllow = TRUE;
}

// disable sleep
void disableSleep(void)
{
    bSleepAllow = FALSE;
}

// set sleep mode
void setSleepMode(Sleep_Mode mode)
{
    sleepMode = mode;
}

// get sleep mode configuration
Sleep_Mode getSleepMode(void)
{
    return sleepMode;
}
void rc32k_cap_adptive_trim(uint32_t temp)
{
    uint32_t rccap= (*(volatile uint32_t *) 0x4000f018 & 0x7f)>>1;
    if((temp > CRY32_16_CYCLE_16MHZ_CYCLE_MAX) && (rccap> 0))
    {
         rccap -= 1;
         subWriteReg(0x4000f018,6,1,rccap);
         //WaitRTCCount(3);
    }    
    else if((temp < CRY32_16_CYCLE_16MHZ_CYCLE_MIN) && (rccap<0x3f) )
    {
         rccap += 1;
         subWriteReg(0x4000f018,6,1,rccap);
         //WaitRTCCount(3);
    }
}
////////////////////////////
// process of enter system sleep mode

/*******************************************************************************
 * @fn          enterSleepProcess0
 *
 * @brief       enter system sleep process function.
 *               
 *
 * input parameters
 *
 * @param       time  - sleep RTC ticks
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void enterSleepProcess0(uint32 time)
{        
    uint32 delta, total, step, temp;

    // if allow RC 32KHz tracking, adjust the time according to the bias    
    if (pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)      
    {
        // 1. read RC 32KHz tracking counter, calculate 16MHz ticks number per RC32KHz cycle

        temp = *(volatile uint32_t *)0x4000f064 & 0x1ffff;

        if (pGlobal_config[LL_SWITCH] & RC32_CAP_ADAPTIVE_TRIM)  
        {
            rc32k_cap_adptive_trim(temp);
        }
        uint8  initValFlg = 0x01 & ((pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG])>>31);
        uint16 initCnt    = 0x7fff & (pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG]>>16);
        uint8 alpha       = 0xff&((pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG])>>8);
        uint8 beta        = 0xff&(pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG]);
        //check for the abnormal temp value

        g_counter_traking_avg = (((1<<alpha)-beta) * g_counter_traking_avg + beta*temp + (1<<(alpha-1)))>>alpha;
        if(g_counter_traking_cnt<initCnt)
        {
            g_counter_traking_cnt++;
            counter_tracking = initValFlg==0 ? temp : STD_CRY32_16_CYCLE_16MHZ_CYCLE ;
        }
        else
        {
            counter_tracking = g_counter_traking_avg;
        }

        // 2.  adjust the time according to the bias

        step = (counter_tracking) >> 3;           // accurate step = 500 for 32768Hz timer   


        if (counter_tracking > STD_CRY32_16_CYCLE_16MHZ_CYCLE)           // RTC is slower, should sleep less RTC tick
        {
            delta = counter_tracking - STD_CRY32_16_CYCLE_16MHZ_CYCLE;   // delta 16MHz tick in 8 32KHz ticks
            
            total = (time * delta) >> 3;                               // total timer bias in 16MHz tick 
            
            while (total > step)        
            {
                total -= step;
                time --;
            }
        }
        else    // RTC is faster, should sleep more RTC tick
        {
            delta = STD_CRY32_16_CYCLE_16MHZ_CYCLE - counter_tracking;   // delta 16MHz tick in 8 32KHz ticks
            total = (time * delta) >> 3;                               // total timer bias in 16MHz tick 
            while (total > step)
            {
                total -= step;
                time ++;
            }        
        }
    }
            
    // backup registers         ------   none now
    
    // backup timers            ------   none now

	//#warning "need check -- byZQ"
    //===20180417 added by ZQ
    //   for P16,P17
    subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x00);   //disable software control
    // 3. config wakeup timer
    config_RTC(time);
  
    // 4. app could add operation before sleep
    app_sleep_process();
    //====== set sram retention
    //    hal_pwrmgr_RAM_retention_set();           // IMPORTANT: application should set retention in app_sleep_process
       
//    ll_debug_output(DEBUG_ENTER_SYSTEM_SLEEP);
         
    // 5. set sleep flag(AON reg & retention SRAM variable)
    set_sleep_flag(1);

    // 6. trigger system sleep
    enter_sleep_off_mode(SYSTEM_SLEEP_MODE);
}


/*******************************************************************************
 * @fn          config_RTC API
 *
 * @brief       This function will configure SRAM retention & comparator
*             Regs:
 *             0x4000f024 : RTCCTL
 *             0x4000f028 : current RTC counter
 *
 * input parameters
 * @param       time   - sleep time in RC32K ticks, may be adjust
 *
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None
 *              
 */
void config_RTC0(uint32 time)
{
//    *((volatile uint32_t *)(0xe000e100)) |= INT_BIT_RTC;   // remove, we don't use RTC interrupt
	//align to rtc clock edge
	WaitRTCCount(1);

    // comparator configuration
    sleep_tick = *(volatile uint32_t *) 0x4000f028;         // read current RTC counter

    g_TIM2_IRQ_to_Sleep_DeltTick = (g_TIM2_IRQ_TIM3_CurrCount>(AP_TIM3->CurrentCount))
                                    ? (g_TIM2_IRQ_TIM3_CurrCount-(AP_TIM3->CurrentCount)): 0;

    AP_AON->RTCCC0 = sleep_tick + time;  //set RTC comparatr0 value
    
//	*(volatile uint32_t *) 0x4000f024 |= 1 << 20;           //enable comparator0 envent
//	*(volatile uint32_t *) 0x4000f024 |= 1 << 18;           //counter overflow interrupt
//	*(volatile uint32_t *) 0x4000f024 |= 1 << 15;           //enable comparator0 inerrupt
    
    //*(volatile uint32_t *) 0x4000f024 |= 0x148000;          // combine above 3 statement to save MCU time
    AP_AON->RTCCTL |= BIT(15)|BIT(18)|BIT(20);
}

/*******************************************************************************
 * @fn          wakeupProcess0
 *
 * @brief       wakeup from system sleep process function.
 *               
 *
 * input parameters
 *
 * @param       None
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void wakeupProcess0(void)
{
    uint32 current_RTC_tick;
    uint32 wakeup_time, wakeup_time0, next_time;
    uint32 sleep_total;
    uint32 dlt_tick;
    
    //restore initial_sp according to the app_initial_sp : 20180706 ZQ
    __set_MSP(pGlobal_config[INITIAL_STACK_PTR]);

	if(s_rom_wakeup_flg != 0){
		ate_wakeup_process();
	}

	HAL_CRITICAL_SECTION_INIT();
 
 	//====  20180416 commented by ZQ
	//      to enable flash access after wakeup
    //      current consumption has been checked. No big different
    //rom_set_flash_deep_sleep();

    //=======fix sram_rent issue 20180323
    //hal_pwrmgr_RAM_retention_clr();            
    //subWriteReg(0x4000f01c,21,17,0);
    
    if (sleep_flag != SLEEP_MAGIC)
    {                 // enter this branch not in sleep/wakeup scenario
        set_sleep_flag(0);

        // software reset
        *(volatile uint32 *)0x40000010 &= ~0x2;    // bit 1: M0 cpu reset pulse, bit 0: M0 system reset pulse.
    }
    /*
    clear sleep flag before wakeup init
    __wdt_init is recoved in wakeup init
    if check_clk_tracking NG, wdt will reset system. should not enter wakeup_process
    goto normal_boot
    */
    set_sleep_flag(0);                    
    // restore HW registers
    wakeup_init();		

    //===20180417 added by ZQ
    //  could be move into wakeup_init
    //  add the patch entry for tx2rx/rx2tx interval config

    //2018-11-10 by ZQ
    //config the tx2rx timing according to the g_rfPhyPktFmt
    ll_hw_tx2rx_timing_config(g_rfPhyPktFmt);
    
//    if (pGlobal_config[LL_SWITCH] & LL_RC32K_SEL)
//    {
//        subWriteReg(0x4000f01c,16,7,0x3fb);   //software control 32k_clk
//        subWriteReg(0x4000f01c,6,6 ,0x01);    //enable software control
//
//    }
//    else
//    {
//        subWriteReg(0x4000f01c,9,8,0x03);   //software control 32k_clk
//        subWriteReg(0x4000f01c,6,6,0x00);   //disable software control
//    }
//		
    //20181201 by ZQ
    //restart the TIM2 to align the RTC
    //----------------------------------------------------------
    //stop the 625 timer
    AP_TIM2->ControlReg=0x0;
	AP_TIM2->ControlReg=0x2;
	AP_TIM2->LoadCount = 2500;
	//----------------------------------------------------------
    //wait rtc cnt change
	WaitRTCCount(1);
	//----------------------------------------------------------
	//restart the 625 timer
    AP_TIM2->ControlReg=0x3;
    current_RTC_tick = rtc_get_counter();
    //g_TIM2_wakeup_delay= (AP_TIM2->CurrentCount)+12; //12 is used to align the rtc_tick
    
 
    wakeup_time0 = read_current_fine_time();


    g_wakeup_rtc_tick = rtc_get_counter();
    // rf initial entry, will be set in app
    rf_init();        

	if(current_RTC_tick>sleep_tick)
	{
	    dlt_tick = current_RTC_tick - sleep_tick;
    }
	else
	{
	    //dlt_tick = current_RTC_tick+0x00ffffff - sleep_tick;
	    dlt_tick = (RTC_OVERFLOW_VAL - sleep_tick)+current_RTC_tick;

    }
    
      
    //if (pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)
    {
        
        //sleep_total = ((current_RTC_tick - sleep_tick) * counter_tracking) >> 7; // shift 4 for 16MHz -> 1MHz, shift 3 for we count 8 RTC tick
        
        // sleep_total =   ((((dlt_tick &0xffff0000)>>16)*counter_tracking)<<9) 
        //                 + (((dlt_tick &0xffff)*counter_tracking)>>7);
        
        //counter_tracking default 16 cycle
        sleep_total =   ((((dlt_tick &0xffff0000)>>16)*counter_tracking)<<8) 
                        + (((dlt_tick &0xffff)*counter_tracking)>>8);
    }


    // restore systick
    g_osal_tick_trim = (pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM]+g_TIM2_IRQ_to_Sleep_DeltTick+2500-g_TIM2_IRQ_PendingTick)>>2;        //16 is used to compensate the cal delay
    g_osalTickTrim_mod+=(pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM]+g_TIM2_IRQ_to_Sleep_DeltTick+2500-g_TIM2_IRQ_PendingTick)&0x03;     //16 is used to compensate the cal delay
    if(g_osalTickTrim_mod>4)
    {
        g_osal_tick_trim+=1;
        g_osalTickTrim_mod = g_osalTickTrim_mod%4;
    }
    
    // restore systick
    osal_sys_tick += (sleep_total+g_osal_tick_trim) / 625;      // convert to 625us systick 
    rtc_mod_value += ((sleep_total+g_osal_tick_trim)%625);

    if(rtc_mod_value > 625){
      osal_sys_tick += 1;
      rtc_mod_value = rtc_mod_value%625;
    }
    
    osalTimeUpdate();
    
    // osal time update, not required. It will be updated when osal_run_system() is called after wakeup
    
    // TODO: should we consider widen the time drift window  ????
    
    //20190117 ZQ 
    if(llState != LL_STATE_IDLE)
    {
        // SW delay
        wakeup_time = read_current_fine_time() - wakeup_time0;

        next_time = 0;

        if (ll_remain_time > sleep_total + wakeup_time)
        {
            next_time = ll_remain_time - sleep_total - wakeup_time;    
            // restore LL timer
			set_timer(AP_TIM1, next_time);
        }
        else
        {
            // should not be here
			set_timer(AP_TIM1, 1000);
        }
    }

//	if (g_llSleepContext.isTimer4RecoverRequired)
//	{
//        // SW delay
//        wakeup_time = read_current_fine_time() - wakeup_time0;
//
//        next_time = 0;
//
//        if (g_llSleepContext.timer4Remainder > sleep_total + wakeup_time)
//        {
//            next_time = g_llSleepContext.timer4Remainder - sleep_total - wakeup_time;    
//            // restore LL timer
//            set_timer(AP_TIM4, next_time);
//        }
//        else
//        {
//            // should not be here
//            set_timer(AP_TIM4, 1500);
//    		//	next_time = 0xffff;
//        }	
//
//		g_llSleepContext.isTimer4RecoverRequired = FALSE;
//	}
    
    // app could add operation after wakeup
    app_wakeup_process();
//    uart_tx0(" 111 ");

//    ll_debug_output(DEBUG_WAKEUP);

    //set_sleep_flag(0);    
    
    // ==== measure value, from RTC counter meet comparator 0 -> here : 260us ~ 270us
    // start task loop
    osal_start_system();
}

/**************************************************************************************
 * @fn          set_sleep_flag
 *
 * @brief       This function set/clear sleep flag in 2 ways: 1. AON reg 0x4000f0a8 
 *                         2. global variable "sleep_flag"
 *
 * input parameters
 *
 * @param       flag   -  0 clear the sleep flag, 1 set the sleep flag.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 */
void set_sleep_flag(int flag)
{
	if(flag)
    {
		*(volatile uint32_t *) 0x4000f0a8 |= 1 ;
        sleep_flag = SLEEP_MAGIC ;
    }
	else
    {
		*(volatile uint32_t *) 0x4000f0a8 &= ~1;
        sleep_flag = 0 ;
    }
}

/**************************************************************************************
 * @fn          get_sleep_flag
 *
 * @brief       This function read the  AON reg 0x4000f0a8 
 *                        
 *
 * input parameters
 *
 * @param       None.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      value of AON reg 0x4000f0a8.
 */
unsigned int get_sleep_flag(void)
{
	return (*(volatile uint32_t *) 0x4000f0a8);
}

/**************************************************************************************
 * @fn          enter_sleep_off_mode
 *
 * @brief       This function will trigger
 *                         
 *
 * input parameters
 *
 * @param       mode     - sleep mode, could be SYSTEM_SLEEP_MODE and SYSTEM_OFF_MODE.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None
 */
void enter_sleep_off_mode0(Sleep_Mode mode)
{
    //uint32 sram0, sram1;
    
    // read data from sram0 - sram4
    //sram0 = *(uint32 *)(SRAM0_ADDRESS);
    //sram1 = *(uint32 *)(SRAM1_ADDRESS);
    
	if(mode == SYSTEM_SLEEP_MODE)
	{
		*(volatile uint32_t *) 0x4000f004 = 0xa5a55a5a; //enter system sleep mode
	}
	else if(mode == SYSTEM_OFF_MODE)
	{
		*(volatile uint32_t *) 0x4000f000 = 0x5a5aa5a5; //enter system off mode
	}
    
    // write back sram0 - sram4, HW need read sram clock so that the sram could enter retention mode,
    // otherwise it will enter standby mode which will consume more power
    //*(volatile uint32 *)(SRAM0_ADDRESS) = sram0;
    //*(volatile uint32 *)(SRAM1_ADDRESS) = sram1;
	
    //same as Prime
    while (1)                   // from config reg to sleep cost about 10us, halt the process 
    {   
	    forever_write = 0x12345678;
    }	    
		
}




