/**************************************************************************************************
  Filename:       OSAL_pwrmgr.c
  Revised:        
  Revision:       

  Description:    This file contains the OSAL Power Management API.


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "comdef.h"
#include "bus_dev.h"
#include "mcu.h"
#include "clock.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_Timers.h"
#include "OSAL_PwrMgr.h"

#include "timer.h"
#include "ll_sleep.h"
#include "jump_function.h"
#include "global_config.h"

/*********************************************************************
 * MACROS
 */

/** @brief Invoke the wait for interrupt procedure of the processor.
 *
 * @warning It is suggested that this macro is called while the interrupts are disabled
 * to have performed the checks necessary to decide to move to sleep mode.
 *
 */

#define WFI()   __WFI();

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/* This global variable stores the power management attributes.
 */
pwrmgr_attribute_t pwrmgr_attribute;
uint32 ll_remain_time;
//llSleepContext    g_llSleepContext;
	
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32 llWaitingIrq;
extern uint16 *tasksEvents;
extern uint32_t  g_wakeup_rtc_tick;

extern uint32 counter_tracking;
extern uint32_t  g_counter_traking_avg;

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
extern uint32  read_ll_adv_remainder_time(void);
/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osal_pwrmgr_init
 *
 * @brief   Initialize the power management system.
 *
 * @param   none.
 *
 * @return  none.
 */
void osal_pwrmgr_init( void )
{
  pwrmgr_attribute.pwrmgr_device = PWRMGR_ALWAYS_ON; // Default to no power conservation.
  pwrmgr_attribute.pwrmgr_task_state = 0;            // Cleared.  All set to conserve
}

/*********************************************************************
 * @fn      osal_pwrmgr_device
 *
 * @brief   Sets the device power characteristic.
 *
 * @param   pwrmgr_device - type of power devices. With PWRMGR_ALWAYS_ON
 *          selection, there is no power savings and the device is most
 *          likely on mains power. The PWRMGR_BATTERY selection allows the
 *          HAL sleep manager to enter sleep.
 *
 * @return  none
 */
void osal_pwrmgr_device( uint8 pwrmgr_device )
{
  pwrmgr_attribute.pwrmgr_device = pwrmgr_device;
}

/*********************************************************************
 * @fn      osal_pwrmgr_task_state
 *
 * @brief   This function is called by each task to state whether or
 *          not this task wants to conserve power.
 *
 * @param   task_id - calling task ID.
 *          state - whether the calling task wants to
 *          conserve power or not.
 *
 * @return  SUCCESS if task complete
 */
uint8 osal_pwrmgr_task_state( uint8 task_id, uint8 state )
{
//	uint16 task_cnt = *(uint16 *)(JUMP_BASE_ADDR + (TASK_COUNT << 1));
  uint16 task_cnt = *((uint16 *)(JUMP_FUNCTION_GET(TASK_COUNT)));

  if ( task_id >= (uint8)task_cnt ) //tasksCnt )
    return ( INVALID_TASK );

  if ( state == PWRMGR_CONSERVE )
  {
    // Clear the task state flag
    pwrmgr_attribute.pwrmgr_task_state &= ~(1 << task_id );
  }
  else
  {
    // Set the task state flag
    pwrmgr_attribute.pwrmgr_task_state |= (1 << task_id);
  }

  return ( SUCCESS );
}


/*********************************************************************
 * @fn      osal_pwrmgr_powerconserve0
 *
 * @brief   This function is called from the main OSAL loop when there are
 *          no events scheduled and shouldn't be called from anywhere else.
 *
 * @param   none.
 *
 * @return  none.
 */
void osal_pwrmgr_powerconserve0( void )  
{                                        
  uint32        next;
  //uint32        rtc_counter;
  uint32 temp1, temp2;

  // Hold off interrupts.
  _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

  if (isSleepAllow())
  {    
  
        
    if ( (pwrmgr_attribute.pwrmgr_device != PWRMGR_ALWAYS_ON) )
    {
        if (llWaitingIrq)       // bug correct 2017-7-5, osal sleep function may be interrupted by LL irq and LL timer could be changed
        {                       // Don't triggered System sleep in such case
            HAL_EXIT_CRITICAL_SECTION(); 
            WFI();
            return;
        }
        
        //===============================================================================================================
        // ZQ: add 20180712. For 32K RC Tracking
        // Ensure wakup enough time before enter sleep
        // RTC counter_tracking is fatched at enter_sleep_process, 
        // Need to reserve enough time for 16M Xtal settling, then fatch the RTC counter_traking.
        // When wakeup time is not enough, turn on the IRQ and _WFI,waiting for the timer_irq or peripheral_irq
        // 
        if(pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)
        {
            uint32 cnt1 = rtc_get_counter();
        
            uint32 delt = (cnt1>=g_wakeup_rtc_tick) ? cnt1-g_wakeup_rtc_tick : (0xffffffff-g_wakeup_rtc_tick+cnt1);

            if(delt<(pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K]) )
            {
                HAL_EXIT_CRITICAL_SECTION();
                WFI();
                return;
            }

        }
        //===============================================================================================================
        // Hold off interrupts.
//        HAL_ENTER_CRITICAL_SECTION( intState );
        
        // the comparator of RTC not consider loop case, it will interrupt immediately if comparator value smaller then threshold
        // walkaround: sleep time will be decreased to avoid RTC counter overflow when sleep

        //20200603 remove rtc overflow protection
        //rtc can wakeup when overflow
        #if 0
        //rtc_counter = *(volatile uint32_t *)0x4000f028 & 0x00ffffff;     // read current RTC counter
        rtc_counter = rtc_get_counter();
        rtc_counter = 0xffffffff - rtc_counter;

        if(pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)
        {
            //when next>rtc_counter, use rc32k_rtc_counter 20180324
            //becase this time will be convert to rc32k in enterSleepProcess once more
//            rc32k_rtc_counter = (((rtc_counter << 7) - (rtc_counter << 2) - (rtc_counter << 1)) >>2 )   /* rtc_counter * (128-4-2)/4 */
//                    +(((rtc_counter << 3)+ rtc_counter ) >> 9 ) ; /* rtc_counter *9/512 */ 
//           
//            //check for the abnormal counter_tracking value
            counter_tracking =  (counter_tracking>CRY32_8_CYCLE_16MHZ_CYCLE_MAX || counter_tracking<CRY32_8_CYCLE_16MHZ_CYCLE_MIN)
                            ? g_counter_traking_avg : counter_tracking;

            rtc_counter =   ((((rtc_counter&0xffff0000)>>16)*counter_tracking)<<9) 
                            + (((rtc_counter&0xffff)*counter_tracking)>>7);

//            //rtc_counter = (rtc_counter << 5) - (rtc_counter << 1) + (rtc_counter >> 1);     // convert RTC tick to us, multiply 30.5
//            rtc_counter = (((rtc_counter << 7) - (rtc_counter << 2) - (rtc_counter << 1)) >>2 )   /* rtc_counter * (128-4-2)/4 */
//                    +(((rtc_counter << 3)+ rtc_counter ) >> 9 ) ; /* rtc_counter *9/512 */ 
        }
        else
        {

            //rtc_counter = (rtc_counter << 5) - (rtc_counter << 1) + (rtc_counter >> 1);     // convert RTC tick to us, multiply 30.5
            rtc_counter = (((rtc_counter << 7) - (rtc_counter << 2) - (rtc_counter << 1)) >>2 )   /* rtc_counter * (128-4-2)/4 */
                    +(((rtc_counter << 3)+ rtc_counter ) >> 9 ) ; /* rtc_counter *9/512 */ 
        }


        // Get next time-out
        next = osal_next_timeout() * 1000;         // convert from ms to us
        
        if (next == 0)                     // no OSAL timer running, only consider rtc_counter & ll timer
            next = rtc_counter;
        else
            next = (next > rtc_counter) ? rtc_counter : next;     
        #endif 
        
        // Get next time-out
        next = osal_next_timeout() * 1000;         // convert from ms to us
        
        // if LL timer is not kick off, set remain time as max sleep time              
        ll_remain_time = read_LL_remainder_time();    
        // ZQ: add 20180514. When timer1 overrun happend, turn on the IRQ and return. 
        // TIM1_IRQ_Handle will process the TIM1_IRQ evernt

        if((AP_TIM1->status &0x1)==1)
        {
            HAL_EXIT_CRITICAL_SECTION();
            return;
        }
        if (llState == LL_STATE_IDLE)   // (ll_remain_time == 0 || llState == LL_STATE_IDLE)  
            ll_remain_time = pGlobal_config[MAX_SLEEP_TIME];       
				
        // remove below decision 2018-04-04. In LL_STATE_IDLE, below statement will fail the sleep process
        // correct 17-09-11, timer IRQ during sleep process will cause read fault ll_remain_time value
        // walkaround here, it is better to modify timer driver to clear all register in ISR
//        if (AP_TIM1->CurrentCount > AP_TIM1->LoadCount)// && (AP_TIM1->ControlReg & 0x1) != 0)
//        {
//            HAL_EXIT_CRITICAL_SECTION();
//            return;					
//        }

        // ===============  BBB ROM code add, timer4 save/recover process
//        if (isTimer4Running())
//        {
//            g_llSleepContext.isTimer4RecoverRequired = TRUE;
//			g_llSleepContext.timer4Remainder         = read_ll_adv_remainder_time();
//        }
//		else
//			g_llSleepContext.isTimer4RecoverRequired = FALSE;
        
        //next = (next > ll_remain_time) ? ll_remain_time : next;   
        //20200616 fix next=0 issue, 
        //when next==0, no OSAL timer running,  
        //let next= ll_remain_time, the max sleep should be configed by pGolbal_config[MAX_SLEEP_TIME]
        next = (next==0) ? ll_remain_time :( (next > ll_remain_time) ? ll_remain_time : next );   
//		if (g_llSleepContext.isTimer4RecoverRequired)
//		    next = (next > g_llSleepContext.timer4Remainder) ? g_llSleepContext.timer4Remainder : next; 
		
        if (getSleepMode() == MCU_SLEEP_MODE
             || llWaitingIrq       // system sleep will not trigger when ll HW processing
             || next < (pGlobal_config[MIN_SLEEP_TIME] + pGlobal_config[WAKEUP_ADVANCE])   // update 2018-09-06
             || pwrmgr_attribute.pwrmgr_task_state != 0)    // Are all tasks in agreement to conserve   
	        {    // MCU sleep mode	
            //LOG("[E]%d %d %d %d\n",getSleepMode(),llWaitingIrq,next,pwrmgr_attribute.pwrmgr_task_state);
            HAL_EXIT_CRITICAL_SECTION();                 
            WFI();
			
            return;
        }
        else
        {                             
            next = next - pGlobal_config[WAKEUP_ADVANCE];    // wakeup advance: consider HW delay, SW re-init time, ..., etc.
            if (next > pGlobal_config[MAX_SLEEP_TIME]) // consider slave latency, we may sleep up to 16s between 2 connection events
                next = pGlobal_config[MAX_SLEEP_TIME];
            
            // covert time to RTC ticks
            // RTC timer clock is 32768HZ, about 30.5us per tick
            // time / 30.5 = time * ( 1/32 + 1 / 512 - 1 / 2048 + 1/16384 - 1/65536 + 1/128K - 1/512K...)
            //             = time * ( 1/32 + 1 / 512 - 1 / 2048 + 1/16384 - 1/128K - 1/512K...) = 32766.3 * 10e-6
            temp1 = next >> 9;
            temp2 = next >> 11;
            next = next >> 5;     
            next = next + temp1 - temp2 + (temp2 >> 3) - (temp2 >> 6) - (temp2 >> 8);
            //next = next + temp1 - temp2 + (temp2 >> 4);
		
            enterSleepProcess(next);           
        }
    }
  }
  // Re-enable interrupts.
  HAL_EXIT_CRITICAL_SECTION();      
}

/*********************************************************************
*********************************************************************/
