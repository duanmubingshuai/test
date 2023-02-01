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
  Filename:       patch.c
  Revised:         
  Revision:        
**************************************************************************************************/

#include "rom_sym_def.h"
#include "bus_dev.h"
#include "global_config.h"
#include "clock.h"
#include "log.h"
#include "rf_phy_driver.h"
#include "OSAL_Tasks.h"

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint32 		global_config[];
extern uint32_t  	__initial_sp;
extern perStatsByChan_t*          p_perStatsByChan;
extern uint32_t g_TIM1_IRQ_timing;
extern void ll_hw_go0(void); 
void ll_hw_go1(void)
{
//	if( *(volatile uint32_t *)(LL_HW_BASE+ 0x24) > 1000)
//		*(volatile uint32_t *)(LL_HW_BASE+ 0x24) = 1000;
	ll_hw_set_loop_nack_num(2);

	ll_hw_go0();
	
//	gpio_write(P13,1);
	
//	llConnState_t *connPtr;
//	connPtr = &conn_param[0];
//	if( connPtr->active )
//	{
//		dbg_printf_init();
//		log_printf("E-%d\n",connPtr->currentEvent);
////	connPtr->sleepClkAccuracy = 7;
//
//	}
	
}
extern uint8   ll_processBasicIRQ(uint32_t      irq_status);
extern void __wdt_init(void);
void hal_wakeup_irq_config(void);
extern void  ble_main(void);

extern uint8 rc32k_calibration(void);
extern void rf_tpCal_gen_cap_arrary(void);
extern uint32 ISR_entry_time;
extern uint32 llWaitingIrq;
extern uint8_t	llSecondaryState;
extern initInfo_t          initInfo;

uint32 pIrqCnt = 0;
uint32 pRtoCnt = 0;
uint32 pCerrCnt = 0;
uint32 pCokCnt = 0;
uint32 linkCnt = 0;

extern void rf_phy_change_cfg1(uint8 pktFmt);
extern void rf_phy_init1(void);
extern void rf_calibrate1(void);

void LL_IRQHandler1(void)
{
    uint32         irq_status;  

    ISR_entry_time = read_current_fine_time();    

	gpio_write(P13,0);
    
    irq_status = ll_hw_get_irq_status();
//    LOG("irq_status %x\n",irq_status);
    if (!(irq_status & LIRQ_MD))          // only process IRQ of MODE DONE
    {
        ll_hw_clr_irq();                  // clear irq status
        return;
    }
	uint32 llhwMode = *(volatile uint32_t *)(LL_HW_BASE+ 0x04);
	if( (  llhwMode & 0x0f ) == 5 )
		{
			pIrqCnt++;
			if(irq_status &LIRQ_RTO)
			{
				pRtoCnt++;
				gpio_write(P13,1);gpio_write(P13,0);
			}
			if((irq_status &LIRQ_CERR)
				|| (irq_status &LIRQ_CERR2))
			{
				pCerrCnt++;
				gpio_write(P13,1);gpio_write(P13,0);gpio_write(P13,1);gpio_write(P13,0);
			}
			if(irq_status &LIRQ_COK)
			{
				pCokCnt++;
			}
		}
		else
		{
			pIrqCnt = 0;
			pRtoCnt = 0;
			pCerrCnt = 0;
			pCokCnt = 0;
		}

//	if(irq_status &LIRQ_RTO)
//	{
//		gpio_write(P13,1);
//		gpio_write(P13,0);
//	}
//	if((irq_status &LIRQ_CERR)
//	|| (irq_status &LIRQ_CERR2))
//	{
//		gpio_write(P14,1);
//		gpio_write(P14,0);
//		gpio_write(P14,1);
//		gpio_write(P14,0);
//	}
    llWaitingIrq = FALSE;

    ll_processBasicIRQ(irq_status);

	if (llSecondaryState == LL_SEC_STATE_SCAN_PENDING)
   	{      // trigger scan
	    llSetupSecScan(scanInfo.nextScanChan);
   	}
	// there is pending init event, make it happen, note that it may stay pending if there is no enough idle time
	else if (llSecondaryState == LL_SEC_STATE_INIT_PENDING)
   	{      // trigger scan
	    llSetupSecInit(initInfo.nextScanChan);
   	}	
}

void move_to_slave_function1(void)
{
	gpio_write(P13,1);
	gpio_write(P13,0);
	move_to_slave_function0();
}
void move_to_master_function1( void )
{
	gpio_write(P13,1);
	gpio_write(P13,0);
	move_to_master_function0();
}
uint8 llProcessRxData1( void )
{
	gpio_write(P14,1);
	gpio_write(P14,0);
	return llProcessRxData0(); 
}
void wakeup_init1()
{
    gpio_write(P13,1);
	wakeup_init0();
	gpio_write(P13,0);
}

//void HAL_wakeup_process(void)
//{
//	pwrmgr_wakeup_process();
//	// LOG_INIT();
//	gpio_write(P17,1);
//}
//
//void HAL_sleep_process(void)
//{
//	gpio_write(P17,0);
//	pwrmgr_sleep_process();
//}
#if _DEF_A0_PATCH_
extern volatile uint32_t  g_hclk;
extern uint32_t dwc_rc32m_frequency(void);
static uint32_t _clk_32m_sel(uint32_t digi_32m_en)
{
    uint32_t digi_32m_sel = 0;  //bit0 32m dll, bit1, 32m dbl
    uint32_t tmp_sel = 0;
    //rxadc
    if(AP_PCRM->CLKHF_CTL1 & BIT(19)){
        tmp_sel =((AP_PCRM->CLKHF_CTL1 >>24) &3);
        if(tmp_sel == 3)    //32m dll
            digi_32m_sel |= BIT(0);
    }
    //rf clk
    if(AP_PCRM->CLKHF_CTL1 & BIT(18)){
        tmp_sel =((AP_PCRM->CLKHF_CTL1 >>22) &3);
        if(tmp_sel == 3)    //32m dll
            digi_32m_sel |= BIT(0);
    }
    //digi 32m
    if(digi_32m_en & BIT(0)){
        digi_32m_sel |= BIT(0); // only support 32M_DLL for pico
    }
    return digi_32m_sel;
}

static uint32 _clk_sel_calc(sysclk_t sel)
{
    uint32_t digi_gate = 0;
    if( sel == SYS_CLK_DLL_32M)
	{
		digi_gate = BIT(0);  //digi gate 32M
	}
	else if( sel == SYS_CLK_DLL_48M)
	{
		digi_gate = BIT(1); //digi gate 48M
	}
	else if( sel == SYS_CLK_DLL_64M)
	{
		digi_gate = BIT(2); //digi gate 64M
	}
	else if( sel == SYS_CLK_DLL_96M)
	{
		digi_gate = BIT(3); //digi gate 96M
	}
    return digi_gate;
}


#define  DELAY_4_NOP_DELAY  {  __asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");}
#define  DELAY_8_NOP_DELAY  {DELAY_4_NOP_DELAY;DELAY_4_NOP_DELAY;}
#define  DELAY_16_NOP_DELAY {DELAY_8_NOP_DELAY;DELAY_8_NOP_DELAY;}
#define  DELAY_32_NOP_DELAY {DELAY_16_NOP_DELAY;DELAY_16_NOP_DELAY;}
void _clk_apply_setting1(uint32_t hclk_sel, uint32_t digi_clk_en, uint32_t digi32_sel)
{   
    uint32_t ispr, iser , clk, clk1;  
    
    //disable clock for bus dev
    clk = AP_PCR->SW_CLK;
    clk1 = AP_PCR->SW_CLK1;
    	
	iser = NVIC_GetEnableIRQs() & 0x7fffffff;
	NVIC_DisableIRQs(0xffffffff);
//
	ispr= NVIC_GetPendingIRQs() & 0x7fffffff; //don't not restore #31 interrupt
	NVIC_ClearPendingIRQs(0xffffffff);

	AP_PCR->SW_CLK = BIT(0)|BIT(19); //need keep MCU and SPIF clock
	AP_PCR->SW_CLK1 = BIT(0)|BIT(6)|(BIT(5)); //need keep MCU and watch dog clock, WFI need com clk

    NVIC_EnableIRQ(RNG_IRQn);
    NVIC_SetWakeupIRQ(RNG_IRQn);

    if(hclk_sel==SYS_CLK_DLL_32M)
        subWriteReg(&(AP_PCRM->CLKHF_CTL1),21,20,3);


    if(digi32_sel & BIT(0)){ //enable dll 32m
        AP_PCRM->CLKHF_CTL1 |= BIT(4); //turn on dll ldo for pico
        wait_hclk_cycle_us(16);
        AP_PCRM->CLKHF_CTL1 |= BIT(7);
    }
    
//    if(digi32_sel & BIT(1)){ //enable doubler 32m
//        AP_PCRM->CLKHF_CTL1 |= BIT(8);
//    }

    if(digi_clk_en){
        if(digi_clk_en & 0xe)//bit 1,2,3 enable, dll enable
        {
            digi32_sel |= 1; //dll bit
            AP_PCRM->CLKHF_CTL1 |= BIT(4); //turn on dll ldo for pico
            wait_hclk_cycle_us(16);
            AP_PCRM->CLKHF_CTL1 |= BIT(7);
        }
        wait_hclk_cycle_us(32);// add delay between dll_en and dig_clk_sel
        AP_PCRM->CLKHF_CTL1 |= ((digi_clk_en&0xf) << 13); //enable digi clock in relative bit of ctl1 register
    }


	//wait_write_finish = 1000;
    if(hclk_sel != (sysclk_t)(AP_PCRM->CLKSEL &0x7))
    {

        if(hclk_sel == SYS_CLK_DLL_32M || hclk_sel == SYS_CLK_DLL_48M ||
            hclk_sel == SYS_CLK_DLL_64M || hclk_sel == SYS_CLK_DLL_96M)
        {
            //wait for dll settle before clock switch
            wait_hclk_cycle_us(32);
        }
        //DELAY_32_NOP_DELAY;
        subWriteReg(0x40003030,4,4,1);
        subWriteReg(&(AP_PCRM->CLKSEL),2,0,hclk_sel); 
        __WFI();
    }
    //clear eneffective clock setting
    subWriteReg(&(AP_PCRM->CLKHF_CTL1), 7,7, digi32_sel); // only clr dll en
    subWriteReg(&(AP_PCRM->CLKHF_CTL1), 16,13, digi_clk_en);
 
    //restore clock for bus dev
    AP_PCR->SW_CLK = clk|BIT(0)|BIT(19);
    AP_PCR->SW_CLK1 = clk1|BIT(0);
   
    NVIC_ClearWakeupIRQ(RNG_IRQn);
    NVIC_DisableIRQ(RNG_IRQn);
    
    NVIC_SetPendingIRQs(ispr);
    NVIC_EnableIRQs(iser);
}

int clk_init_wfi(sysclk_t hclk_sel)
{
    uint32_t digi_clk_en = 0;
    //sysclk_t spif_ref_sel;
    uint32_t digi_32m_sel = 0;  //bit0 32m dll, bit1, 32m dbl

    if((AP_PCRM->CLKHF_CTL0 & BIT(18)) == 0)//xtal 16Mhz never disabled
        AP_PCRM->CLKHF_CTL0 |= BIT(18);

    //if((AP_PCRM->CLKSEL >> 3 & 3) != 0)
    //subWriteReg(&(AP_PCRM->CLKSEL),4,3,3);//override:default is 1, here set 0
	subWriteReg(&(AP_PCRM->CLKSEL),4,3,0);//override:default is 1, here set 0
    
    //spif_ref_sel = (sysclk_t)((AP_PCRM->CLKSEL >> 24) &0x7);
    //digi_clk_en = _clk_sel_calc(spif_ref_sel);
    digi_clk_en |= _clk_sel_calc(hclk_sel);

    digi_32m_sel = _clk_32m_sel(digi_clk_en & BIT(0));
    
    extern uint32_t g_hclk_table[SYS_CLK_NUM] ;
    g_hclk = g_hclk_table[hclk_sel];
    
    if(hclk_sel == SYS_CLK_RC_32M && dwc_rc32m_frequency())
        g_hclk = dwc_rc32m_frequency();
       

    //apply clock sel
    //gpio_write(P9,1);
	_clk_apply_setting1(hclk_sel, digi_clk_en, digi_32m_sel);
	//gpio_write(P9,0);
//BASE_ADDR 0x4000_0000
//0x28 [4]
//OTP AHB sync down bridge bypas
//1: bypass
//0: use bridge
//	if(hclk_sel == SYS_CLK_XTAL_16M)//otp use bypass
//	{
//		*(volatile int*)0x40000028 |= 0x10;
//	}
//	else//otp use bridage
//	{
//		*(volatile int*)0x40000028 &= 0xFFFFFFEF;	
//	}

	*(volatile int*)0x40000028 &= 0xFFFFFFEF;	
	
    return PPlus_SUCCESS;
}
uint8_t g_dllErr;

#define DLL_TRACKING_96M_2CYC_LIMT_L (4000) // 2*(1/32768)*96000000 = 5859
#define DLL_TRACKING_96M_2CYC_LIMT_DLT (10) // 2*(1/32768)*96000000 = 5859
void check_dll_by_rcTracking(void)
{
    
    WaitRTCCount(3);
    uint32_t temp,temp1,dlt;
    // //enable digclk 96M
    temp = *(volatile uint32_t*)0x4000f044;
    *(volatile uint32_t*)0x4000f044 = temp | BIT(16);

    temp = *(volatile uint32_t*)0x4000f05C;
    *(volatile uint32_t*)0x4000f05C = (temp & 0xfffefe00) | 0x0028 | BIT(16);

    //subWriteReg(0x4000f05C,3,3,1);

    for(uint8 index=0; index<100; index++)
    {

        WaitRTCCount(3);
        temp1 = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
        WaitRTCCount(3);
        temp  = (*(volatile uint32_t*)0x4000f064 & 0x1ffff);
        dlt = (temp1>temp) ? temp1-temp : temp-temp1;

        if(     temp1 >DLL_TRACKING_96M_2CYC_LIMT_L 
            && temp >DLL_TRACKING_96M_2CYC_LIMT_L 
            && dlt<DLL_TRACKING_96M_2CYC_LIMT_DLT && (dlt > 0) )
        {
            //disable 96M
            subWriteReg(0x4000f05C,16,16,0);
            subWriteReg(0x4000f044,16,16,0);
            g_dllErr=index;
            return;
        }
        if(((index&0x07)==0 && index>0 ) || (( temp1 == temp ) && ( temp == 0)) )
        {
            subWriteReg(0x4000f044,7,7,0);
            WaitRTCCount(3);
            subWriteReg(0x4000f044,7,7,1);
        }
    }
    AP_AON->SLEEP_R[0]=0x10;
    system_soft_reset();
}
static uint32_t s_last_check_dll_t0=0;
void runtime_check_dll(void)
{
    uint32_t t0,dlt;
    t0 = rtc_get_counter();
    dlt = (t0>=s_last_check_dll_t0) ? t0-s_last_check_dll_t0 : 0xffffff-s_last_check_dll_t0+t0;
    if(dlt>30000)
    {
        check_dll_by_rcTracking();
        s_last_check_dll_t0 = rtc_get_counter();
        // ======== enable tracking 32KHz RC timer with 16MHz crystal clock
         uint32_t temp = *(volatile uint32_t *)0x4000f05C;
        *(volatile uint32_t *)0x4000f05C = (temp & 0xfffefe00) | 0x0108; //[16] 16M [8:4] cnt [3] track_en_rc32k
        WaitRTCCount(16);
    }
    
}
void wakeup_init2()
{
    __wdt_init();
    uint8_t pktFmt = 1;    // packet format 1: BLE 1M
    uint32  temp;

#if (DBG_BUILD_LL_TIMING)
    //====== for timing debug============
    gpio_write(TMPIN2, 1);
    gpio_write(TMPIN2, 0);
    #warning "only use for debug, please disable this code"

    //PHY_REG_WT(AP_IOMUX_BASE+8,1);//en debugMux[0]
#endif  
    WaitRTCCount(pGlobal_config[WAKEUP_DELAY]);
//    clk_tracking_init(CLK_TRACKING_XTAL, pGlobal_config[CLK_TRACKING_CONFIG_XTAL], CLK_TRACKING_MOD_POLLING);
//        
//    if(pGlobal_config[CLK_TRACKING_CONFIG_XTAL])
//        check_clk_settle(CLK_TRACKING_XTAL);

    /*
        DLL Tracking Tips
        While dll tracking is enable ,the initial settle status is fail. 
        SHOULD NOT config the hlk as dll , before the dll tracking is finished.
        When HCLK=DLL, enable dll tracking, will due to hclk swith to 32M RC by HW.
        Recommend sequence is :
        1. turn on dll
        2. do dll tracking
        3. wait till dll tracking finihsed
        4. call clk_init sequenc.
    */
    if( g_system_clk == SYS_CLK_DLL_32M ||g_system_clk == SYS_CLK_DLL_48M
        ||g_system_clk == SYS_CLK_DLL_64M|| g_system_clk == SYS_CLK_DLL_96M
        ||(g_rxAdcClkSel == RX_ADC_CLK_SEL_32M_DLL)
        ||(g_rfPhyClkSel == RF_PHY_CLK_SEL_32M_DLL)
      )
    {
        AP_PCRM->CLKHF_CTL0 |= BIT(18);
        subWriteReg(0x4000f044, 25, 22, (g_rxAdcClkSel<<3)|g_rfPhyClkSel );

        _clk_dll_enable();
        
        check_dll_by_rcTracking();
//        clk_tracking_init(CLK_TRACKING_DLL, pGlobal_config[CLK_TRACKING_CONFIG_DLL], CLK_TRACKING_MOD_IRQ);
//
//        if(pGlobal_config[CLK_TRACKING_CONFIG_DLL])
//        {   
//            /*
//                1.disable dll irq, check dll settling by polling mode 
//                2. check dll settling for  the dll initial phase
//                3. after dll is settled, enable dll irq, 
//            */
//
//            NVIC_DisableIRQ(DLL_IRQn);
//            check_clk_settle(CLK_TRACKING_DLL); 
//            NVIC_EnableIRQ(DLL_IRQn);
//        }

    }

	clk_init_wfi(g_system_clk);

	uint32 delta_tick;
	uint32 compens_af = 0;
	uint32 current_RTC_tick = rtc_get_counter();       
	extern uint32 sleep_tick;
	extern uint32 ll_remain_time;
	if(current_RTC_tick>sleep_tick)
	{
	    delta_tick = current_RTC_tick - sleep_tick;
    }
	else
	{
	    //dlt_tick = current_RTC_tick+0x00ffffff - sleep_tick;
	    delta_tick = (RTC_OVERFLOW_VAL - sleep_tick)+current_RTC_tick;

    }
	compens_af = (delta_tick >> 3) + (delta_tick >> 6);
//	if( ll_remain_time > delta_tick )
		ll_remain_time += compens_af ;
    // ========== init timers
    set_timer(AP_TIM2, 625);      // OSAL 625us tick     
    set_timer(AP_TIM3, BASE_TIME_UNITS);   // 1s timer       

    // =========== open interrupt mask
	//int_state = 0x14;
	//set_int(int_state);
    //should use NVIC_EnableIRQn()
    NVIC_EnableIRQ(BB_IRQn);
    NVIC_EnableIRQ(TIM1_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM4_IRQn);
	#if(PHY_MCU_TYPE==MCU_BUMBEE_CK802)
//		SystemInit();
        extern void _symrom_SystemInit(void);
		_symrom_SystemInit();
//		subWriteReg(0x40003030,4,4,1);//ck802 WFI enable
		hal_wakeup_irq_config();
	#endif  
    // ============ config BB Top
	*(volatile uint32_t *) 0x40030000 = 0x3d068001; // set tx pkt =2
	*(volatile uint32_t *) 0x400300bc = 0x834;      //[7:0] pll_tm [11:8] rxafe settle
	*(volatile uint32_t *) 0x400300a4 = 0x140;      //[6] for tpm_en	
    // =========== ll HW setting
    set_max_length(0xff);

    ll_hw_set_empty_head(0x0001);
	
    //time related setting
    ll_hw_set_rx_timeout_1st(   500);
    ll_hw_set_rx_timeout(        88);       //ZQ 20180606, reduce rx timeout for power saving
                                            //preamble + syncword=40us, sync process = 8us
                                            //timeout should be larger then 48us, 
											
    //ll_hw_set_rx_timeout(       268);		//for ble shoulde be larger than 80+128. if sync, the timeout timer stop. 
                                      // (80 + 128) - BLE 5.0 preamble + access time, 60 for HW process delay
                                      // this time doesn't consider HW startup time, it is set in other regs

    ll_hw_set_loop_timeout(   30000);
    
//		ll_hw_set_tx_rx_release	(10,   	 1);
//		ll_hw_set_rx_tx_interval(    	57);		//T_IFS=150us for BLE 1M
//		ll_hw_set_tx_rx_interval(    	65);		//T_IFS=150us for BLE 1M
//		ll_hw_set_trx_settle	(57, 8, 52);		//TxBB,RxAFE,PLL    
    ll_hw_set_timing(pktFmt);
    
	
    ll_hw_ign_rfifo(LL_HW_IGN_SSN | LL_HW_IGN_CRC | LL_HW_IGN_EMP);
    
    // ======== enable tracking 32KHz RC timer with 16MHz crystal clock
    temp = *(volatile uint32_t *)0x4000f05C;
    *(volatile uint32_t *)0x4000f05C = (temp & 0xfffefe00) | 0x0108; //[16] 16M [8:4] cnt [3] track_en_rc32k
    #if 0
    //get wakeup tracking counter
    if (pGlobal_config[LL_SWITCH] & RC32_TRACKINK_ALLOW)
    {
        WaitRTCCount(17);
        uint32_t  counter_tracking_wakeup = *(volatile uint32_t *)0x4000f064 & 0x1ffff;
        counter_tracking = (counter_tracking_wakeup + counter_tracking)>>1;
        //compensate for delay
        // pGlobal_config[WAKEUP_ADVANCE] += 510;
    }
    #endif
}



#endif
#if 0
void wakeup_init3()
{
	gpio_write(P14,1);
	wakeup_init0();
	gpio_write(P14,0);
	gpio_write(P14,1);
	gpio_write(P14,0);
}
extern uint8_t s_rom_wakeup_flg;
extern uint32 sleep_flag;
extern uint32_t  g_wakeup_rtc_tick;
extern uint32 sleep_tick;
extern uint32 ll_remain_time;
extern uint32 counter_tracking;
extern uint32_t  g_osal_tick_trim;
extern uint32_t  g_osalTickTrim_mod;
extern uint32_t  rtc_mod_value;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32_t  g_TIM2_IRQ_PendingTick;
extern uint32 osal_sys_tick;
extern void ate_wakeup_process(void);

void wakeupProcess1(void)
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
			gpio_write(P13,1);
			gpio_write(P13,0);
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
#endif
extern uint32_t g_TIM1_IRQ_timing ;
void llSlaveEvt_TaskEndOk1( void )
{
	llConnState_t *connPtr;
	connPtr = &conn_param[0];
	if( connPtr->rx_timeout ) 
	{
		g_TIM1_IRQ_timing = 0;
		if( connPtr->llTbd1++ == 1 )
			pGlobal_config[TIMER_SET_ENTRY_TIME] = 600;
		else
			pGlobal_config[TIMER_SET_ENTRY_TIME] = 0;
	}
	else
	{	
		connPtr->llTbd1 = 0 ;
		pGlobal_config[TIMER_SET_ENTRY_TIME] = 200;
	
    }
	llSlaveEvt_TaskEndOk0();

}

extern int slave_conn_event_recv_delay;
void ll_adptive_adj_next_time1(uint32 nextTime)
{
	ll_adptive_adj_next_time0( nextTime );
	slave_conn_event_recv_delay -= 63;
}



extern void ll_scheduler0(uint32 time);
extern llConns_t           g_ll_conn_ctx;
void ll_scheduler1(uint32 time)
{
	llConnState_t *connPtr;
	
	if( time != LL_INVALID_TIME )
	{
		ll_scheduler0(time-11);
	}
	else
	{
		ll_scheduler0(time);
	}
	if( g_ll_conn_ctx.currentConn != LL_INVALID_CONNECTION_ID  )
	{
		connPtr = &conn_param[ g_ll_conn_ctx.currentConn ];
		if( connPtr->active )
		{
			if( get_timer_count(AP_TIM1) == 0 )
			{
				set_timer(AP_TIM1,20);
				g_ll_conn_ctx.current_timer = 20;
			}
		}
	}

}


void debug_wakeup_process(void)
{
    gpio_write(P11,1);
    gpio_write(P11,0);
    
    wakeupProcess0();
}
uint8_t g_dllErr;
void DLL_Tracking_IRQHandler1(void)
{
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();

    _CLK_TRACKING_IRQ_CLR(DLL_TRACKING_REG);
    uint32_t cnt0,cnt1,delt;
    cnt0 = rtc_get_counter();
    while(0==_CLK_TRACKING_SETTLE(DLL_TRACKING_REG))
    {
        cnt1 = rtc_get_counter();
        delt = (cnt1>=cnt0) ? (cnt1-cnt0) : (RTC_OVERFLOW_VAL-cnt0+cnt1);
        if(delt > 30)
        {
            //reset dll for 
            subWriteReg(0x4000f044,7,7,0);
            WaitRTCCount(3);
            subWriteReg(0x4000f044,7,7,1);
            cnt0 = rtc_get_counter();
            g_dllErr++;
        }
    }
    HAL_EXIT_CRITICAL_SECTION();
    
}
extern void _clk_apply_setting0(uint32_t hclk_sel, uint32_t digi_clk_en, uint32_t digi32_sel,uint8_t wfi_sel);
void _clk_apply_setting1(uint32_t hclk_sel, uint32_t digi_clk_en, uint32_t digi32_sel,uint8_t wfi_sel)
{
    _clk_apply_setting0( hclk_sel, digi_clk_en, 1,wfi_sel);
}

void init_config(void)
{
    int i;
    
    for (i = 0; i < SOFT_PARAMETER_NUM; i ++)
        pGlobal_config[i] = 0;
    
    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;
    
    pGlobal_config[LL_SWITCH] =  SLAVE_LATENCY_ALLOW | RC32_TRACKINK_ALLOW | LL_RC32K_SEL |
								SIMUL_CONN_SCAN_ALLOW ;
//    if(g_clk32K_config==CLK_32K_XTAL)
//        pGlobal_config[LL_SWITCH] &= 0xffffffee;
//    else
//        pGlobal_config[LL_SWITCH] |= RC32_TRACKINK_ALLOW | LL_RC32K_SEL;
    
    // sleep delay
    pGlobal_config[MIN_TIME_TO_STABLE_32KHZ_XOSC] = 10;      // 10ms, temporary set
    
    // system clock setting
    pGlobal_config[CLOCK_SETTING] = g_system_clk;//CLOCK_32MHZ;
    
    //------------------------------------------------------------------------
    // wakeup time cose
    // t1. HW_Wakeup->MCU relase 62.5us
    // t2. wakeup_process in waitRTCCounter 30.5us*[WAKEUP_DELAY] about 500us
    // t3. dll_en -> hclk_sel in hal_system_ini 100us in run as RC32M
    // t4. sw prepare cal sleep tick initial rf_ini about 300us @16M this part depends on HCLK
    // WAKEUP_ADVANCE should be larger than t1+t2+t3+t4 
    //------------------------------------------------------------------------
    // wakeup advance time, in us


       
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[WAKEUP_ADVANCE] = 2100;//650;//600;//310;
        pGlobal_config[WAKEUP_DELAY] = 10;
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[WAKEUP_ADVANCE] = 2250;//650;//600;//310;
        pGlobal_config[WAKEUP_DELAY] = 10;
    }
    
    pGlobal_config[CLK_TRACKING_CONFIG_DLL ] = 0x0001f414;//0x0001f414;// [19:8] target_cnt [7:0] settle_thd 
    pGlobal_config[CLK_TRACKING_CONFIG_XTAL] = 0x0000fa0a;// [19:8] target_cnt [7:0] settle_thd

    pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG] = 0x00640508;//[31:16]avg init cnt  [15:8] alpha [7:0] beta

	// sleep time, in us
    pGlobal_config[MAX_SLEEP_TIME] = 1500000;    
    pGlobal_config[MIN_SLEEP_TIME] = 1500;
    
    pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K] = 60;// 30.5 per tick    

    //-------------------------------------------------------------------------
    //-------------------------------------------------------------------------
    // LL engine settle time
    pGlobal_config[LL_HW_BB_DELAY] = 54;//54-8;
    pGlobal_config[LL_HW_AFE_DELAY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY] = 52;
        // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV] = 65-RF_PHY_EXT_PREAMBLE_US;
    pGlobal_config[LL_HW_Tx_TO_RX_INTV] = 50;//65   
    
    //------------------------------------------------2MPHY
    // LL engine settle time 
    pGlobal_config[LL_HW_BB_DELAY_2MPHY] = 59;
    pGlobal_config[LL_HW_AFE_DELAY_2MPHY] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_2MPHY] = 52;
    // Tx2Rx and Rx2Tx interval
    //Tx2Rx could be advanced a little
    //Rx2Tx should be ensure T_IFS within150us+-2us
    pGlobal_config[LL_HW_Rx_TO_TX_INTV_2MPHY] = 20-RF_PHY_EXT_PREAMBLE_US;//72
    pGlobal_config[LL_HW_Tx_TO_RX_INTV_2MPHY] = 7;//72   

    
    // LL engine settle time, for advertisement
    pGlobal_config[LL_HW_BB_DELAY_ADV] = 90;
    pGlobal_config[LL_HW_AFE_DELAY_ADV] = 8;
    pGlobal_config[LL_HW_PLL_DELAY_ADV] = 60;    
        
    // adv channel interval
    pGlobal_config[ADV_CHANNEL_INTERVAL] = 1400;//6250;
    
    pGlobal_config[NON_ADV_CHANNEL_INTERVAL] = 666;//6250;
    
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 9+RF_PHY_EXT_PREAMBLE_US;//23;       
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        // scan req -> scan rsp timing
        pGlobal_config[SCAN_RSP_DELAY] = 5+RF_PHY_EXT_PREAMBLE_US;        // 12	//  2019/3/19 A2: 12 --> 9
    }                                      
	else if(g_system_clk == SYS_CLK_DLL_64M)		//  2019/3/26 add
	{
		pGlobal_config[SCAN_RSP_DELAY] = 3+RF_PHY_EXT_PREAMBLE_US;
	}
				
    // conn_req -> slave connection event calibration time, will advance the receive window
    pGlobal_config[CONN_REQ_TO_SLAVE_DELAY] = 100;	// 1300
		
    pGlobal_config[SLAVE_CONN_DELAY] = 143;//0;//1500;//0;//3000;//0;          ---> update 11-20
    pGlobal_config[SLAVE_CONN_DELAY_BEFORE_SYNC] = 160;
		
    // RTLP timeout
	pGlobal_config[LL_HW_RTLP_1ST_TIMEOUT]	= 1000 ;

		   
    // direct adv interval configuration
    pGlobal_config[HDC_DIRECT_ADV_INTERVAL] = 1000;
    pGlobal_config[LDC_DIRECT_ADV_INTERVAL] = 6250;
    
    // A1 ROM metal change for HDC direct adv, 
    pGlobal_config[DIR_ADV_DELAY] = 115;   // in us, consider both direct adv broadcast time & SW delay, ... etc.
    
    //====== A2 metal change add, for scanner & initiator
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 18+RF_PHY_EXT_PREAMBLE_US;//20;		//  2019/3/19 A2: 20 --> 18
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 25+RF_PHY_EXT_PREAMBLE_US;//27;		//  2019/3/19 A2: 27 --> 25
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 10+RF_PHY_EXT_PREAMBLE_US;//12;		//  2019/3/19 A2: 12 --> 10
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 40+RF_PHY_EXT_PREAMBLE_US;
    }
	else if(g_system_clk==SYS_CLK_DLL_64M)
    {
        pGlobal_config[LL_ADV_TO_SCAN_REQ_DELAY]    = 8+RF_PHY_EXT_PREAMBLE_US;				//  2019/3/26 add
        pGlobal_config[LL_ADV_TO_CONN_REQ_DELAY]    = 10+RF_PHY_EXT_PREAMBLE_US;
    }

    
    // TRLP timeout
    pGlobal_config[LL_MOVE_TO_MASTER_DELAY] = 500;  
    
    pGlobal_config[LL_CONN_REQ_WIN_SIZE] = 1;  
    pGlobal_config[LL_CONN_REQ_WIN_OFFSET] = 1;  
    
    pGlobal_config[LL_MASTER_PROCESS_TARGET] = 200;   // reserve time for preparing master conn event, delay should be insert if needn't so long time


    pGlobal_config[OSAL_SYS_TICK_WAKEUP_TRIM] = 56;  // 0.125us
    
    pGlobal_config[MAC_ADDRESS_LOC] = 0x1FFF00C0;
    
    
    pGlobal_config[LL_SEC_SCAN_MARGIN] = 1250;//1400;  to avoid mesh proxy llTrigErr 0x15
    pGlobal_config[LL_MIN_SCAN_TIME] = 200;
    
    //  BBB new
    pGlobal_config[TIMER_SET_ENTRY_TIME] = 160;
    pGlobal_config[LL_MULTICONN_MASTER_PREEMP] = 0;
	pGlobal_config[LL_MULTICONN_MASTER_SLOT] = 4;
	
    pGlobal_config[LL_CONN_TASK_DURATION] = 1300;			
    

	extern void osalInitTasks( void );
	extern pTaskEventHandlerFn tasksArr[];
	extern uint16 tasksCnt;
	extern uint16 *tasksEvents;
	JUMP_FUNCTION_SET(OSAL_INIT_TASKS,(uint32_t)&osalInitTasks);
	JUMP_FUNCTION_SET(TASKS_ARRAY,(uint32_t)&tasksArr);
	JUMP_FUNCTION_SET(TASK_COUNT ,(uint32_t)&tasksCnt);
	JUMP_FUNCTION_SET(TASK_EVENTS,(uint32_t)&tasksEvents);
	  
	JUMP_FUNCTION_SET(RF_INIT,(uint32_t)&rf_phy_init1);
	JUMP_FUNCTION_SET(V18_IRQ_HANDLER,(uint32_t)&DLL_Tracking_IRQHandler1);
	JUMP_FUNCTION_SET(CLK_APPLY_SETTING,(uint32_t)&_clk_apply_setting1);
	JUMP_FUNCTION_SET(RF_CALIBRATTE,(uint32_t)&rf_calibrate1);
    JUMP_FUNCTION_SET(RF_PHY_CHANGE,(uint32_t)&rf_phy_change_cfg1);
    //JUMP_FUNCTION_SET(WAKEUP_PROCESS,(uint32_t)&debug_wakeup_process);
//    JUMP_FUNCTION_SET(LL_PROCESS_RX_DATA,(uint32_t)&llProcessRxData1);
    //-------------------------------------------------------------------
    // patch function register
    //--------------------------------------------------------------------

	//JUMP_FUNCTION_SET(APP_WAKEUP_PROCESS,(uint32_t)&HAL_wakeup_process);
	
	//JUMP_FUNCTION_SET(APP_SLEEP_PROCESS,(uint32_t)&HAL_sleep_process);
	
//	JUMP_FUNCTION(228)                         =   (uint32_t)&LL_IRQHandler1;
//	JUMP_FUNCTION(244)                         =   (uint32_t)&TIM1_IRQHandler1;
	
//	JUMP_FUNCTION_SET(V4_IRQ_HANDLER,(uint32_t)&LL_IRQHandler1);
//	JUMP_FUNCTION_SET(APP_SLEEP_PROCESS,(uint32_t)&app_sleep_process1);
//	JUMP_FUNCTION_SET(APP_WAKEUP_PROCESS,(uint32_t)&app_wakeup_process1);
//	JUMP_FUNCTION_SET(V20_IRQ_HANDLER,(uint32_t)&TIM1_IRQHandler1);
//	JUMP_FUNCTION_SET(V21_IRQ_HANDLER,(uint32_t)&TIM2_IRQHandler1);
//	JUMP_FUNCTION_SET(V22_IRQ_HANDLER,(uint32_t)&TIM3_IRQHandler1);

//	JUMP_FUNCTION_SET(RF_PHY_CHANGE,(uint32_t)&rf_phy_change_cfg0);
// ====================
	

//    disableSleep();
//    //setSleepMode(MCU_SLEEP_MODE);//SYSTEM_SLEEP_MODE
//    enableSleep();
//    setSleepMode(SYSTEM_SLEEP_MODE);

}
void patch_slave(void)
{
	JUMP_FUNCTION_SET(LL_HW_GO,(uint32_t)&ll_hw_go1);
//	JUMP_FUNCTION_SET(V4_IRQ_HANDLER,(uint32_t)&LL_IRQHandler1);
	JUMP_FUNCTION_SET(LL_SLAVE_EVT_ENDOK,(uint32_t)&llSlaveEvt_TaskEndOk1);
//	JUMP_FUNCTION_SET(WAKEUP_INIT,(uint32_t)&wakeup_init1);
//	JUMP_FUNCTION_SET(WAKEUP_PROCESS,(uint32_t)&wakeupProcess1);
	//	JUMP_FUNCTION_SET(LL_MOVE_TO_SLAVE_FUNCTION,(uint32_t)&move_to_slave_function1);
//	JUMP_FUNCTION_SET(LL_ADP_ADJ_NEXT_TIME,(uint32_t)&ll_adptive_adj_next_time1);
}
void patch_master(void)
{
//	JUMP_FUNCTION_SET(V4_IRQ_HANDLER,(uint32_t)&LL_IRQHandler1);
	JUMP_FUNCTION_SET(LL_SCHEDULER,(uint32_t)&ll_scheduler1);
	//	JUMP_FUNCTION_SET(LL_MOVE_TO_MASTER_FUNCTION,(uint32_t)&move_to_master_function1);
}

void init_config_snrf(void)
{
    int i;
    
    for (i = 0; i < SOFT_PARAMETER_NUM; i ++)
        pGlobal_config[i] = 0;
    
    //save the app initial_sp  which will be used in wakeupProcess 20180706 by ZQ
    pGlobal_config[INITIAL_STACK_PTR] = (uint32_t)&__initial_sp;
    
    pGlobal_config[LL_SWITCH] =  SLAVE_LATENCY_ALLOW | RC32_TRACKINK_ALLOW | LL_RC32K_SEL |
								SIMUL_CONN_SCAN_ALLOW ;
//    if(g_clk32K_config==CLK_32K_XTAL)
//        pGlobal_config[LL_SWITCH] &= 0xffffffee;
//    else
//        pGlobal_config[LL_SWITCH] |= RC32_TRACKINK_ALLOW | LL_RC32K_SEL;
    
    // sleep delay
    pGlobal_config[MIN_TIME_TO_STABLE_32KHZ_XOSC] = 10;      // 10ms, temporary set
    
    // system clock setting
    pGlobal_config[CLOCK_SETTING] = g_system_clk;//CLOCK_32MHZ;
    
    //------------------------------------------------------------------------
    // wakeup time cose
    // t1. HW_Wakeup->MCU relase 62.5us
    // t2. wakeup_process in waitRTCCounter 30.5us*[WAKEUP_DELAY] about 500us
    // t3. dll_en -> hclk_sel in hal_system_ini 100us in run as RC32M
    // t4. sw prepare cal sleep tick initial rf_ini about 300us @16M this part depends on HCLK
    // WAKEUP_ADVANCE should be larger than t1+t2+t3+t4 
    //------------------------------------------------------------------------
    // wakeup advance time, in us


       
    if(g_system_clk==SYS_CLK_XTAL_16M)
    {
        pGlobal_config[WAKEUP_ADVANCE] = 1850;//650;//600;//310;
        pGlobal_config[WAKEUP_DELAY] = 10;
    }
    else if(g_system_clk==SYS_CLK_DLL_48M)
    {
        pGlobal_config[WAKEUP_ADVANCE] = 1850;//650;//600;//310;
        pGlobal_config[WAKEUP_DELAY] = 10;
    }
    
    pGlobal_config[CLK_TRACKING_CONFIG_DLL ] = 0x0001f414;// [19:8] target_cnt [7:0] settle_thd 
    pGlobal_config[CLK_TRACKING_CONFIG_XTAL] = 0x0000fa0a;// [19:8] target_cnt [7:0] settle_thd

    pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG] = 0x00640508;//[31:16]avg init cnt  [15:8] alpha [7:0] beta

	// pGlobal_config[RC32_CNT_TRACKING_AVG_CONFIG] = 0x00640501;//[31:16]avg init cnt  [15:8] alpha [7:0] beta

	// sleep time, in us
    pGlobal_config[MAX_SLEEP_TIME] = 1500000;    
    pGlobal_config[MIN_SLEEP_TIME] = 1500;
    
    pGlobal_config[ALLOW_TO_SLEEP_TICK_RC32K] = 60;// 30.5 per tick    


	extern void osalInitTasks( void );
	extern pTaskEventHandlerFn tasksArr[];
	extern uint16 tasksCnt;
	extern uint16 *tasksEvents;
	JUMP_FUNCTION_SET(OSAL_INIT_TASKS,(uint32_t)&osalInitTasks);
	JUMP_FUNCTION_SET(TASKS_ARRAY,(uint32_t)&tasksArr);
	JUMP_FUNCTION_SET(TASK_COUNT ,(uint32_t)&tasksCnt);
	JUMP_FUNCTION_SET(TASK_EVENTS,(uint32_t)&tasksEvents);
	// JUMP_FUNCTION_SET(LL_HW_GO,(uint32_t)&ll_hw_go1);
	// JUMP_FUNCTION_SET(V4_IRQ_HANDLER,(uint32_t)&LL_IRQHandler1);
    //JUMP_FUNCTION_SET(WAKEUP_INIT,(uint32_t)&wakeup_init1);
//	JUMP_FUNCTION_SET(LL_MOVE_TO_SLAVE_FUNCTION,(uint32_t)&move_to_slave_function1);
//	JUMP_FUNCTION_SET(LL_MOVE_TO_MASTER_FUNCTION,(uint32_t)&move_to_master_function1);
//    JUMP_FUNCTION_SET(LL_PROCESS_RX_DATA,(uint32_t)&llProcessRxData1);
	JUMP_FUNCTION_SET(RF_INIT,(uint32_t)&rf_phy_init1);
	JUMP_FUNCTION_SET(V18_IRQ_HANDLER,(uint32_t)&DLL_Tracking_IRQHandler1);
	JUMP_FUNCTION_SET(CLK_APPLY_SETTING,(uint32_t)&_clk_apply_setting1);
	JUMP_FUNCTION_SET(RF_CALIBRATTE,(uint32_t)&rf_calibrate1);
    JUMP_FUNCTION_SET(RF_PHY_CHANGE,(uint32_t)&rf_phy_change_cfg1);
	
}

void ll_set_ble_mac_addr(uint32_t macAddr)
{
	pGlobal_config[MAC_ADDRESS_LOC] = macAddr;
}

void hal_rom_code_ini(void)
{
    rc32k_calibration();
	ble_main();
}

extern uint8 g_largeHeap[];
uint32  osal_memory_statics(void)
{
    osalMemHdr_t* header, *current;
    void* ptr;
    uint32  sum_alloc = 0;
    uint32  sum_free = 0;
    uint32  max_block = 0;
//    halIntState_t intState;
    ptr = (void*)g_largeHeap;
    header = (osalMemHdr_t*)ptr;
    current = (osalMemHdr_t*)ptr;
_HAL_CS_ALLOC_();
//    HAL_ENTER_CRITICAL_SECTION1( intState );  // Hold off interrupts.
    HAL_ENTER_CRITICAL_SECTION();
    do
    {
        if ((uint32)ptr > (uint32)header + 4096)
        {
//            dbg_printf("==========error: memory audit failed===============\r\n");
            break;
        }

        // seek to the last block, return
        if ( current->val == 0 )       /// val = 0, so len = 0
        {
            break;
        }

        if (current->hdr.inUse)
            sum_alloc += current->hdr.len;
        else
        {
            sum_free += current->hdr.len;

            if (current->hdr.len > max_block && (void*)(&current->hdr) > (void*)(header ))
                max_block = current->hdr.len;
        }

        current = (osalMemHdr_t*)((uint8*)current + current->hdr.len);
    }
    while (1);
    HAL_EXIT_CRITICAL_SECTION();

//    HAL_EXIT_CRITICAL_SECTION1( intState );  // Re-enable interrupts.
//    printf("sum_alloc = %d, sum_free = %d, max_free_block = %d\r\n", sum_alloc, sum_free, max_block);
    LOG_ERROR("sum_alloc = %d, sum_free = %d,max_free_block = %d \n", sum_alloc,sum_free, max_block);
    return sum_alloc;
}

void app_stack_monitor()
{
	extern uint32_t g_intstackbase;
	uint32_t *g_intstackbase_addr = &g_intstackbase;
	uint32_t *initial_sp_addr = &__initial_sp;
	LOG_ERROR("Stack total : %d Bytes,",((initial_sp_addr - g_intstackbase_addr)<<2));
	for(uint32_t *addr=g_intstackbase_addr;addr<initial_sp_addr;addr++)
	{
		if(*addr != 0 )
		{
			LOG_ERROR("used : %d Bytes\n",((initial_sp_addr - addr) << 2));
			break;
		}
	}
}

void LL_ExtInit_ChanMap( perStatsByChan_t *patch_StatsByChan,uint16 *rxNumCrcErr, uint16 *txNumRetry,uint16 *rxToCnt)
{ 
	p_perStatsByChan = patch_StatsByChan;
	p_perStatsByChan->rxNumCrcErr = rxNumCrcErr;
	p_perStatsByChan->txNumRetry = txNumRetry;
	p_perStatsByChan->rxToCnt = rxToCnt;
//	LOG_ERROR("%p,%p,%p,%p\n",p_perStatsByChan,p_perStatsByChan->rxNumCrcErr,p_perStatsByChan->txNumRetry,p_perStatsByChan->rxToCnt);
}

void LL_ExtClearBER_Buffer()
{
	if( p_perStatsByChan != NULL )
	{
		osal_memset(p_perStatsByChan->rxNumCrcErr, 0, 74) ;
		osal_memset(p_perStatsByChan->txNumRetry, 0, 74)	;
		osal_memset(p_perStatsByChan->rxToCnt, 0, 74) ;		
	}
}
extern uint16 rxToCnt[ LL_MAX_NUM_DATA_CHAN ];
uint8 LL_ExtGet_RecommandChanMap(uint16 connHandle , uint8 *rstChanMap,uint32 statis_TimeMs ,uint8 BER_Percent )
{
	llConnState_t *connPtr;
	connPtr = &conn_param[ connHandle ];
	uint8 rst = FALSE;
	if( ( connPtr->active )  && ( p_perStatsByChan != NULL ))
	{
		uint32 statis_total_EvtCnt = statis_TimeMs / ( connPtr->curParam.connInterval  ) ;
		uint32 threshold_BER = statis_total_EvtCnt / connPtr->numUsedChans * ( BER_Percent  ) ; // BER_Percent/128 ~ BER_Percent/100
		threshold_BER = (threshold_BER >> 6 ) ;
		int8 usedChan = connPtr->numUsedChans;
//		LOG_ERROR("threshold_BER %d,%d\n",threshold_BER,statis_total_EvtCnt);
		llMemCopyDst( chanMapUpdate.chanMap, connPtr->chanMap ,LL_NUM_BYTES_FOR_CHAN_MAP );
		// step 1 : caculate the high BER channel
		for( uint8 i=0;i< LL_MAX_NUM_DATA_CHAN; i++)
		{
//			LOG_ERROR("%2d,Trto-%d,ERR-%d,Ret-%d,rto-%d\n",i,rxToCnt[i],p_perStatsByChan->rxNumCrcErr[i],p_perStatsByChan->txNumRetry[i],p_perStatsByChan->rxToCnt[i]);
			if( (p_perStatsByChan->rxNumCrcErr[i] + p_perStatsByChan->txNumRetry[i] ) > threshold_BER )
			{
				// unused channel shall not invode
				chanMapUpdate.chanMap[ i / 8 ] &=  ( ~((uint8)(0x1 << ( i % 8 ))) );
				usedChan--;
			}
		}
		// round : reset stat buffer
		LL_ExtClearBER_Buffer();
		// setp 2 : appropriate recover some unused channel , which is the high BER channel cacluate before
		{
			// algorithm
			if( usedChan < 2 ) 
			{
				osal_memset(chanMapUpdate.chanMap,0xFF,5);
			}
		}
		chanMapUpdate.chanMap[4] &= 0x1F;
		
		llMemCopyDst( rstChanMap, chanMapUpdate.chanMap, LL_NUM_BYTES_FOR_CHAN_MAP );
		rst = TRUE;
	}
	return rst;
}
uint8 ll_ext_chanMap_procedure( uint16 connHandle,uint8 *chanMap )
{
	llConnState_t *connPtr;
	connPtr = &conn_param[ connHandle ];
	uint8 rst = FALSE;
	if( !connPtr->pendingChanUpdate  )
	{
		uint8 *pBuf = connPtr->ctrlData.data;
		uint8 pktLen = LL_CHAN_MAP_REQ_PAYLOAD_LEN;

		// write control type as payload
		*pBuf++ = LL_CTRL_CHANNEL_MAP_REQ;

		// write the new channel map
		pBuf = llMemCopyDst( pBuf, chanMap, LL_NUM_BYTES_FOR_CHAN_MAP );
		llMemCopyDst( connPtr->chanMapUpdate.chanMap, chanMap, LL_NUM_BYTES_FOR_CHAN_MAP );

		// we are writing to the FIFO, so convert relative instant number to an
		// absolute event number
		connPtr->chanMapUpdateEvent = connPtr->currentEvent + ( connPtr->curParam.slaveLatency << 1 )  ;

		// write the update event count
		pBuf = llMemCopyDst( pBuf, (uint8 *)&connPtr->chanMapUpdateEvent, 2 );
		connPtr->ctrlData.header = pktLen << 8 | LL_DATA_PDU_HDR_LLID_CONTROL_PKT;		
		connPtr->ctrlDataIsPending = 1;
		connPtr->ctrlDataIsProcess = 0;
		connPtr->pendingChanUpdate = TRUE;
		rst = TRUE;
	}
	return rst;
}

#define _TOSTRING(s)	#s
#define TOSTRING(s)	_TOSTRING(s)

uint32 get_commit_id(void)
{
	uint32 ret;
	
//	ret = (uint32)COMMIT_ID;
	ret = (uint32)TOSTRING(COMMIT_ID);

	return ret;
}

uint32 get_date_time(void)
{
	uint32 rd;
	
//	rd = (uint32)COMMIT_DATE;
	rd = (uint32)TOSTRING(COMMIT_DATE);

	return rd;
}

//void patch_test()
//{
//	for (uint8 i = 0; i < g_maxConnNum; i++)
//	{
//		if (conn_param[i].active)
//		{
//}
