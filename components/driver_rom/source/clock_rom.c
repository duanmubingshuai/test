#include "mcu.h"
#include "bus_dev.h"
#include "clock.h"
#include "digi_wav_capture.h"
#include "error.h"

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

void rtc_stop(void)
{
    AP_AON->RTCCTL &= ~BIT(0);
}

void rtc_clear(void)
{
    int i = 100;
    AP_AON->RTCCTL |= BIT(1);
    while(i-->0);
}

bool rtc_config_prescale(uint32_t pre)
{
    uint8_t divider;
    
    if((pre == 0) || (pre > 0xFFF))	
        return FALSE;
	else
        divider = pre -1;
    
    if(((AP_AON->RTCCTL>>2)& 0xFFF) != divider)
    {
        rtc_stop();
        subWriteReg(&(AP_AON->RTCCTL),13,2,divider);
        rtc_start();
    }
    return TRUE;
}

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

void WaitRTCCount(uint32_t rtcDelyCnt)
{
    volatile uint32 cnt0,cnt1;
    uint32 delt =0;
    AP_AON->RTCCTL |= BIT(0);//RUN_RTC;
    cnt0 = rtc_get_counter();

    while(delt<rtcDelyCnt)
    { 
        cnt1 = rtc_get_counter();
        delt = (cnt1>=cnt0) ? (cnt1-cnt0) : (0xffffffff-cnt0+cnt1);
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

/*
disable interrupt
switch clock
MCU Enter low power mode(M0 _WFI, 802 WAIT)
*/
static void _clk_apply_setting(uint32_t hclk_sel, uint32_t digi_clk_en, uint32_t digi32_sel)
{   
    uint32_t ispr, iser , clk, clk1;
    //volatile int wait_write_finish = 1000;

    //disable clock for bus dev
    clk = AP_PCR->SW_CLK;
    clk1 = AP_PCR->SW_CLK1;
    	
	iser = NVIC_GetEnableIRQs() & 0x7fffffff;
	NVIC_DisableIRQs(0xffffffff);

	ispr= NVIC_GetPendingIRQs() & 0x7fffffff; //don't not restore #31 interrupt
	NVIC_ClearPendingIRQs(0xffffffff);

	AP_PCR->SW_CLK = BIT(0)|BIT(19); //need keep MCU and SPIF clock
	AP_PCR->SW_CLK1 = BIT(0); //need keep MCU and SPIF clock

    NVIC_EnableIRQ(RNG_IRQn);
    NVIC_SetWakeupIRQ(RNG_IRQn);

    if(digi32_sel & BIT(0)){ //enable dll 32m
        AP_PCRM->CLKHF_CTL1 |= BIT(7);
    }
    
    if(digi32_sel & BIT(1)){ //enable doubler 32m
        AP_PCRM->CLKHF_CTL1 |= BIT(8);
    }

    if(digi_clk_en){
        if(digi_clk_en & 0xe)//bit 1,2,3 enable, dll enable
        {
            digi32_sel |= 1; //dll bit
            AP_PCRM->CLKHF_CTL1 |= BIT(7);
        }
        AP_PCRM->CLKHF_CTL1 |= ((digi_clk_en&0xf) << 13); //enable digi clock in relative bit of ctl1 register
    }

	//wait_write_finish = 1000;
    if(hclk_sel != (sysclk_t)(AP_PCRM->CLKSEL &0x7))
    {
        subWriteReg(&(AP_PCRM->CLKSEL),2,0,hclk_sel); 
        //while(((AP_PCRM->CLKSEL &0x7) != hclk_sel) && (wait_write_finish--));
        //__WFI();
    }

    //clear eneffective clock setting
    subWriteReg(&(AP_PCRM->CLKHF_CTL1), 8,7, digi32_sel);
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
2: 32m doubler enabler
3: both up two
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
        else if(tmp_sel > 0) //32m dbl
            digi_32m_sel |= BIT(1); 
    }
    //rf clk
    if(AP_PCRM->CLKHF_CTL1 & BIT(18)){
        tmp_sel =((AP_PCRM->CLKHF_CTL1 >>22) &3);
        if(tmp_sel == 3)    //32m dll
            digi_32m_sel |= BIT(0);
        else if(tmp_sel > 0) //32m dbl
            digi_32m_sel |= BIT(1); 
    }
    //digi 32m
    if(digi_32m_en & BIT(0)){
        tmp_sel = ((AP_PCRM->CLKHF_CTL1 >>20) &3);
        if(tmp_sel == 3)    //32m dll
            digi_32m_sel |= BIT(0);
        else //32m dbl
            digi_32m_sel |= BIT(1); 
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

int clk_init(sysclk_t hclk_sel)
{
    uint32_t digi_clk_en = 0;
    //sysclk_t spif_ref_sel;
    uint32_t digi_32m_sel = 0;  //bit0 32m dll, bit1, 32m dbl
    volatile uint32_t delay;
    
    //delay = g_hclk>>15;
    delay = g_hclk_table[AP_PCRM->CLKSEL &0x7]>>15;//32M 
    if((AP_PCRM->CLKHF_CTL0 & BIT(18)) == 0)//xtal 16Mhz never disabled
        AP_PCRM->CLKHF_CTL0 |= BIT(18);

    //if((AP_PCRM->CLKSEL >> 3 & 3) != 0)
    subWriteReg(&(AP_PCRM->CLKSEL),4,3,3);//override:default is 1, here set 0
	
    //spif_ref_sel = (sysclk_t)((AP_PCRM->CLKSEL >> 24) &0x7);
    //digi_clk_en = _clk_sel_calc(spif_ref_sel);
    digi_clk_en |= _clk_sel_calc(hclk_sel);

    digi_32m_sel = _clk_32m_sel(digi_clk_en & BIT(0));
    

    g_hclk = g_hclk_table[hclk_sel];
    
    if(hclk_sel == SYS_CLK_RC_32M && dwc_rc32m_frequency)
        g_hclk = dwc_rc32m_frequency();
       
    while (delay-- > 0)
	{
		;
	}

    //apply clock sel
    //_clk_apply_setting(spif_ref_sel, hclk_sel, digi_clk_en, digi_32m_sel);
	_clk_apply_setting(hclk_sel, digi_clk_en, digi_32m_sel);
	
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

#ifdef FPGA_ROM_DRIVER_TEST
uint32_t osal_sys_tick;
#endif
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
