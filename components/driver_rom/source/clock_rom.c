#include "mcu.h"
#include "bus_dev.h"
#include "clock.h"
#include "digi_wav_capture.h"
#include "error.h"
#include "gpio.h"
#define  DELAY_4_NOP_DELAY  {  __asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");}
#define  DELAY_8_NOP_DELAY  {DELAY_4_NOP_DELAY;DELAY_4_NOP_DELAY;}
#define  DELAY_16_NOP_DELAY {DELAY_8_NOP_DELAY;DELAY_8_NOP_DELAY;}
#define  DELAY_32_NOP_DELAY {DELAY_16_NOP_DELAY;DELAY_16_NOP_DELAY;}

void clk_gate_enable(MODULE_e module)
{	
	if(module < MOD_CP_CPU)
	{
		AP_PCR->SW_CLK |= BIT(module);	
	}
	else if(module < MOD_PCLK_CACHE)
	{
		AP_PCR->SW_CLK1 |= BIT(module-MOD_CP_CPU);
	}
}

void clk_gate_disable(MODULE_e module)
{
	if(module < MOD_CP_CPU)
	{
		AP_PCR->SW_CLK &= ~(BIT(module));
	}
	else if(module < MOD_PCLK_CACHE)
	{
		AP_PCR->SW_CLK1 &= ~(BIT(module-MOD_CP_CPU));
	}
}

int clk_gate_get(MODULE_e module)
{
	if(module < MOD_CP_CPU)
	{
		return (AP_PCR->SW_CLK & BIT(module));
	}
	else if(module < MOD_PCLK_CACHE)
	{
		return (AP_PCR->SW_CLK1 & BIT(module-MOD_CP_CPU));
	}
    //else if(module < MOD_USR0)//jack no cache,remove compile warnning
    {
        return (AP_PCR->CACHE_CLOCK_GATE & BIT(module-MOD_PCLK_CACHE));
    }
}
	
void clk_reset(MODULE_e module)
{
	if(module < MOD_CP_CPU)
	{
		if((module >= MOD_TIMER5) &&(module <= MOD_TIMER6))
		{
			AP_PCR->SW_RESET0 &= ~BIT(5);
			AP_PCR->SW_RESET0 |= BIT(5);
		}
		else
		{
			AP_PCR->SW_RESET0 &= ~BIT(module);
			AP_PCR->SW_RESET0 |= BIT(module);
		}
	}
	else if(module < MOD_PCLK_CACHE)
	{		
		if((module >= MOD_TIMER1) &&(module <= MOD_TIMER4))
		{
			AP_PCR->SW_RESET2 &= ~BIT(4);
			AP_PCR->SW_RESET2 |= BIT(4);
		}
		else
		{
			AP_PCR->SW_RESET2 &= ~BIT(module-MOD_CP_CPU);
			AP_PCR->SW_RESET2 |= BIT(module-MOD_CP_CPU);
		}
	}
}

const uint32_t g_hclk_table[SYS_CLK_NUM] = {
    32000000, //SYS_CLK_RC_32M      = 0,
    32000000, //SYS_CLK_DLL_32M     = 1,
    16000000, //SYS_CLK_XTAL_16M    = 2,
    48000000, //SYS_CLK_DLL_48M     = 3,
    64000000, //SYS_CLK_DLL_64M     = 4,
    96000000, //SYS_CLK_DLL_96M     = 5,
    8000000, //SYS_CLK_8M          = 6,
    4000000, //SYS_CLK_4M          = 7,
};

volatile uint32_t g_hclk = 32000000;

void rtc_start(void)
{
    AP_AON->RTCCTL |= BIT(0);
}

//void rtc_stop(void)
//{
//    AP_AON->RTCCTL &= ~BIT(0);
//}
//
//void rtc_clear(void)
//{
//    int i = 100;
//    AP_AON->RTCCTL |= BIT(1);
//    while(i-->0);
//}

//bool rtc_config_prescale(uint32_t pre)
//{
//    uint8_t divider;
//    
//    if((pre == 0) || (pre > 0xFFF))	
//        return FALSE;
//	else
//        divider = pre -1;
//    
//    if(((AP_AON->RTCCTL>>2)& 0xFFF) != divider)
//    {
//        rtc_stop();
//        subWriteReg(&(AP_AON->RTCCTL),13,2,divider);
//        rtc_start();
//    }
//    return TRUE;
//}

uint32_t rtc_get_counter(void)
{
    uint32_t cnt0,cnt1;
   
    while(1)
    {
        cnt0 = (AP_AON->RTCCNT);
        cnt1 = (AP_AON->RTCCNT);
        
        if(cnt1==cnt0)
            break;
    }
    return cnt1;   
}

static int read_current_time(void)
{
    return(TIME_BASE - ((AP_TIM3->CurrentCount) >> 2));
}

void WaitUs(uint32_t wtTime)
{    
    uint32_t T0,currTick,deltTick;
    T0 = read_current_time();

    while(1)
    {
        currTick = read_current_time();
        deltTick = TIME_DELTA(currTick,T0);
        
        if(deltTick>wtTime)
            break;
    }
}
void wait_hclk_cycle_us(uint32_t dlyT)
{
   volatile uint32 delay;
   uint8_t curr_hclk = AP_PCRM->CLKSEL &0x7;
   delay = g_hclk_table[curr_hclk];//32M
   delay = (delay +(1<<22))>>23;//+0.5 for round

   delay = dlyT*delay;
   if((sysclk_t)curr_hclk == SYS_CLK_RC_32M)
   {
     delay = delay>>(AON_RC32M_DIV_GET);
     
   }
   delay+=1;// add 1 to keep delay>0 for lower clock 

   while(delay--){};
   
}

void WaitRTCCount(uint32_t rtcDelyCnt)
{
    volatile uint32 cnt0,cnt1;
    uint32 delt =0;
    AP_AON->RTCCTL |= BIT(0);//RUN_RTC;
    cnt0 = rtc_get_counter();

    while(delt<rtcDelyCnt)
    { 
        cnt1 = rtc_get_counter();
        delt = (cnt1>=cnt0) ? (cnt1-cnt0) : (RTC_OVERFLOW_VAL-cnt0+cnt1);
    }    
}

void WaitMs(uint32_t msecond)
{
	WaitRTCCount((msecond<<15)/1000);
}

void clk_set_pclk_div(uint8_t div)
{
    volatile int i = 100;
    AP_PCR->APB_CLK = ((div <<4) & 0xF0);
    AP_PCR->APB_CLK_UPDATE = 2;
    while(i--){
        ;
    }
    AP_PCR->APB_CLK_UPDATE = 2;
}
int clk_set_rc32M_div(clk_rc32m_div_t div)
{
    if(AP_PCRM->CLKSEL &0x7)
    {
        AON_RC32M_DIV_SET(div);
        return PPlus_SUCCESS;
    }
    return PPlus_ERR_BUSY;
}
uint32_t clk_get_hclk(void)
{
    return g_hclk_table[AP_PCRM->CLKSEL & 0x07];
}

uint32_t clk_get_pclk(void)
{
    volatile uint32_t pclk;	
    pclk = (g_hclk_table[AP_PCRM->CLKSEL & 0x07] / (((AP_PCR->APB_CLK & 0xF0) >> 4) + 1));
    return pclk;
}
void _clk_dll_enable(void)
{
    AP_PCRM->CLKHF_CTL1 |= BIT(4); //turn on dll ldo for pico
    wait_hclk_cycle_us(16);
    AP_PCRM->CLKHF_CTL1 |= BIT(7);
    wait_hclk_cycle_us(32);
}
/*
disable interrupt
switch clock
MCU Enter low power mode(M0 _WFI, 802 WAIT)
*/
void _clk_apply_setting0(uint32_t hclk_sel, uint32_t digi_clk_en, uint32_t digi32_sel,uint8_t wfi_sel)
{   
    uint32_t ispr, iser , clk, clk1;  
    
    //disable clock for bus dev
    clk = AP_PCR->SW_CLK;
    clk1 = AP_PCR->SW_CLK1;
    	
	iser = NVIC_GetEnableIRQs() & 0x7fffffff;
	NVIC_DisableIRQs(0xffffffff);

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
        DELAY_16_NOP_DELAY;
        if(wfi_sel)
        {
            subWriteReg(&(AP_PCRM->CLKSEL),4,3,0);//override:default is 1, here set 0
            subWriteReg(0x40003030, 4, 4, 1); //ck802 WFI enable
        }
        else
            subWriteReg(&(AP_PCRM->CLKSEL),4,3,3);//override:default is 1, here set 0
        subWriteReg(&(AP_PCRM->CLKSEL),2,0,hclk_sel); 
        //DELAY_32_NOP_DELAY;
        //while(((AP_PCRM->CLKSEL &0x7) != hclk_sel) && (wait_write_finish--));
        if(wfi_sel)
            __WFI();
        else
        {
            DELAY_16_NOP_DELAY;
        }
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


/*
return value:
0: no 32m digi
1: 32m dll enable
2: 32m doubler enabler : not support on pico
3: both up two : not support on pico
*/
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
    switch(sel){
    case SYS_CLK_RC_32M:
        break;
    case SYS_CLK_DLL_32M:
        digi_gate = BIT(0);  //digi gate 32M
        break;
    case SYS_CLK_XTAL_16M:
    case SYS_CLK_8M:
    case SYS_CLK_4M:
        //do nothing
        break;
    case SYS_CLK_DLL_48M:
        digi_gate = BIT(1); //digi gate 48M
        break;
    case SYS_CLK_DLL_64M:
        digi_gate = BIT(2); //digi gate 64M
        break;
    case SYS_CLK_DLL_96M:
        digi_gate = BIT(3); //digi gate 96M
        break;
    default:
        break;
    }
    return digi_gate;
}

int clk_init_wfi_sel(sysclk_t hclk_sel,uint8_t wfi_sel)
{
    uint32_t digi_clk_en = 0;
    //sysclk_t spif_ref_sel;
    uint32_t digi_32m_sel = 0;  //bit0 32m dll, bit1, 32m dbl

    if((AP_PCRM->CLKHF_CTL0 & BIT(18)) == 0)//xtal 16Mhz never disabled
        AP_PCRM->CLKHF_CTL0 |= BIT(18);

    //if((AP_PCRM->CLKSEL >> 3 & 3) != 0)
    //subWriteReg(&(AP_PCRM->CLKSEL),4,3,3);//override:default is 1, here set 0
	
    //spif_ref_sel = (sysclk_t)((AP_PCRM->CLKSEL >> 24) &0x7);
    //digi_clk_en = _clk_sel_calc(spif_ref_sel);
    digi_clk_en |= _clk_sel_calc(hclk_sel);

    digi_32m_sel = _clk_32m_sel(digi_clk_en & BIT(0));
    

    g_hclk = g_hclk_table[hclk_sel];
    
    if(hclk_sel == SYS_CLK_RC_32M && dwc_rc32m_frequency)
        g_hclk = dwc_rc32m_frequency();
       

    //apply clock sel
    //_clk_apply_setting(spif_ref_sel, hclk_sel, digi_clk_en, digi_32m_sel);
	_clk_apply_setting(hclk_sel, digi_clk_en, digi_32m_sel,wfi_sel);
	
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
int clk_init_no_wfi(sysclk_t hclk_sel)
{
    return clk_init_wfi_sel( hclk_sel,0);
}

int clk_init(sysclk_t hclk_sel)
{
    return clk_init_wfi_sel( hclk_sel,1);
}
extern uint32_t osal_sys_tick;

uint32_t get_systick(void)
{
	return osal_sys_tick;
}

uint32_t get_ms_intv(uint32_t tick)
{
    uint32_t diff = 0;

    if(osal_sys_tick < tick)
    {
        diff = 0xffffffff- tick;
        diff = osal_sys_tick + diff;
    }
    else
    {
        diff = osal_sys_tick - tick;
    }

    return diff*625/1000;
}

void system_soft_reset(void)
{
    //_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
	drv_disable_irq();
    AP_PCR->SW_RESET1 = 0;

    while(1);
}

void XTAL_Tracking_IRQHandler(void)
{
    _CLK_TRACKING_IRQ_CLR(XTAL_TRACKING_REG);
    if(0==_CLK_TRACKING_SETTLE(XTAL_TRACKING_REG))
    {
        AP_AON->SLEEP_R[0]=8;
        system_soft_reset();
    }
}

void DLL_Tracking_IRQHandler(void)
{

    _CLK_TRACKING_IRQ_CLR(DLL_TRACKING_REG);
    if(0==_CLK_TRACKING_SETTLE(DLL_TRACKING_REG))
    {
        AP_AON->SLEEP_R[0]=0x10;
        system_soft_reset();
    }
}

void clk_tracking_init(uint8_t mod ,uint32_t config,uint8 irqMask)
{
    if(config==0)
        return;

    /*
        20220417 add register for cnt_tracking +- thd sync
        the register is in the dll power domain, need turn on dll ldo 
    */
    AP_PCRM->CLKHF_CTL1 |= BIT(4); //turn on dll ldo for pico
    wait_hclk_cycle_us(16);
    
    uint32_t addr = (mod==CLK_TRACKING_DLL)?DLL_TRACKING_REG:XTAL_TRACKING_REG;
    _HAL_CS_ALLOC_();HAL_ENTER_CRITICAL_SECTION();
    _CLK_TRACKING_ENABLE(addr,0);
    wait_hclk_cycle_us(66);
    _CLK_TRACKING_IRQ_CLR(addr);


    
    if(0==irqMask)
    {
        //enable irq
        if(mod==CLK_TRACKING_DLL)
        {
            NVIC_EnableIRQ(DLL_IRQn);
            NVIC_SetWakeupIRQ(DLL_IRQn);
        }
        else
        {
            NVIC_EnableIRQ(XTAL_IRQn);
            NVIC_SetWakeupIRQ(XTAL_IRQn);
        }
    }
//    _CLK_TRACKING_CNT_TARGET(addr,config&0x0fff);
//    _CLK_TRACKING_THD_SETTLE(addr,(config>>16)&0x00ff);
//    _CLK_TRACKING_IRQ_MASK(addr,irqMask);
    _CLK_TRACKING_SET_CONFIG(addr,irqMask,config);

    _CLK_TRACKING_ENABLE(addr,1);

    HAL_EXIT_CRITICAL_SECTION();
}


void check_clk_settle(uint8_t mod )
{
    uint32_t addr = (mod==CLK_TRACKING_DLL)?DLL_TRACKING_REG:XTAL_TRACKING_REG;
    uint8_t rstCnt = 0;
    while(1)
    {
        WaitRTCCount(3);
        if(_CLK_TRACKING_IRQ_STATUS(addr))
        {

            _CLK_TRACKING_IRQ_CLR(addr);
            if(_CLK_TRACKING_SETTLE(addr))
            {
                break;
            }
        }
        rstCnt++;
        /* 
            restart clk tracking when no irq_raw or not settling
            irq_raw will not be clr when disable clk tracking
            need clr irq_raw manually after one 32k clk cyc

            1st settle is generated after 13 rtc clk, 2nd and later settle is updated every 4 rtc clk
            we set the restart limt as 64*3 192 rtc tick, if not irq status, restart the clk_tracking
        */
        if(rstCnt==0x40)
        {
            rstCnt=0;
            _CLK_TRACKING_ENABLE(addr,0);
            WaitRTCCount(3);
            _CLK_TRACKING_IRQ_CLR(addr);
            _CLK_TRACKING_ENABLE(addr,1);
        }
        
    }
}
