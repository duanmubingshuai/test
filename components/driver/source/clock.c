#include "rom_sym_def.h"
#include "mcu.h"
#include "bus_dev.h"
#include "clock.h"
#include "error.h"
#include "gpio.h"


void hal_rtc_clock_config(CLK32K_e clk32Mode)
{
    if(clk32Mode == CLK_32K_RCOSC)
    {
        subWriteReg(&(AP_AON->PMCTL0),31,27,0x05);
        subWriteReg(&(AP_AON->PMCTL2_0),16,7,0x3fb);
        subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x01);
        //pGlobal_config[LL_SWITCH]|=RC32_TRACKINK_ALLOW|LL_RC32K_SEL;
    }
    else if(clk32Mode == CLK_32K_XTAL)
    {
        // P16 P17 for 32K XTAL input
        gpio_pull_set(P16,FLOATING);
        gpio_pull_set(P17,FLOATING);
        subWriteReg(&(AP_AON->PMCTL2_0),9,8,0x03);   //software control 32k_clk
        subWriteReg(&(AP_AON->PMCTL2_0),6,6,0x00);   //disable software control
        subWriteReg(&(AP_AON->PMCTL0),31,27,0x16);
        //pGlobal_config[LL_SWITCH]&=0xffffffee;
    }

    //ZQ 20200812 for rc32k wakeup
    subWriteReg(&(AP_AON->PMCTL0),28,28,0x1);//turn on 32kxtal
    subWriteReg(&(AP_AON->PMCTL1),18,17,0x0);// reduce 32kxtl bias current
}


void hal_system_soft_reset(void)
{
    //_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    /**
        config reset casue as RSTC_WARM_NDWC
        reset path walkaround dwc
    */
    AP_AON->SLEEP_R[0]=4;
	drv_disable_irq();
    AP_PCR->SW_RESET1 = 0;

    while(1);
}
