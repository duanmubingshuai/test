/*
this file is used for all extern interrupt routine, it need other project to fill the jump table 
to realize interrupt function defined by user.
*/

#include "bus_dev.h"
#include "timer.h"
#include "jump_function.h"
#include "uart.h"

#ifndef FPGA_PERIPHERAL_TEST
#include "ll_common.h"
#endif

#include "gpio.h"

int   hardFault;
typedef void (*irq_function)(void);

//V3 otp
//extern void OTP_IRQHandler(void);

//V4 bb
extern void LL_IRQHandler(void);

//V5 kscan
extern void __attribute__((used)) hal_KSCAN_IRQHandler(void);

//V6 rtc
//V7 cpcom_ap_ipc_irq
//V8 apcom_ap_ipc_irq

//V10 wdt_irq
extern void __attribute__((used)) WDT_IRQHandler(void);

//V11 uart0
extern void SWU_UART0_IRQHandler(void);
extern void __attribute__((used)) hal_UART0_IRQHandler(void);

//V12 i2c0_irq

//V14 spi0
extern void __attribute__((used)) SPI0_IRQHandler(void);

//V15 spi1
extern void __attribute__((used)) SPI1_IRQHandler(void);

//V16 gpio
extern void GPIO_IRQHandler_R(void);    

//V17 uart1                
extern void __attribute__((used)) hal_UART1_IRQHandler(void);

//V18 spif_irq
//V19

//V20 timer1
extern void TIM1_IRQHandler(void);

//V21 timer2
extern void TIM2_IRQHandler(void);

//V22 timer3
extern void TIM3_IRQHandler(void);

//V23 timer4
extern void __attribute__((used)) hal_TIMER4_IRQHandler(void);

//V24
//V25
//V26
//V27
//V28 aes

//V29 adc
extern void __attribute__((used)) hal_ADC_IRQHandler(void);

//V30 qdec
extern void __attribute__((used)) QDEC_IRQHandler();      

// ====== extern function
// ==========================
void HardFault_IRQHandler(void)
{
    hardFault++;
    //gpio_write(P1, Bit_ENABLE);
    while(1)
    {
    }
}

void NMI_Handler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(NMI_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
}

void HardFault_Handler()
{
    irq_function pFunc = NULL;
     
    
  pFunc = (irq_function)(JUMP_FUNCTION_GET(HARDFAULT_HANDLER));
    
    if ( pFunc != 0)
         pFunc();
    else
         HardFault_IRQHandler();
}


void SVC_Handler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(SVC_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
    
}

void PendSV_Handler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(PENDSV_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
    
}

ATTRIBUTE_ISR void SysTick_Handler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(SYSTICK_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
    
}

//None
ATTRIBUTE_ISR void V0_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V0_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
}

ATTRIBUTE_ISR void V1_IRQHandler(void)//SysTick_Handler
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V1_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
}

//None
ATTRIBUTE_ISR void V2_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V2_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
}

//OTP
ATTRIBUTE_ISR void V3_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V3_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
}

// LL_IRQHandler BB_IRQ
ATTRIBUTE_ISR void V4_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V4_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
#ifndef FPGA_PERIPHERAL_TEST		
    else
        LL_IRQHandler();
#endif		
}

// KSCAN
ATTRIBUTE_ISR void V5_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V5_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
//	else
//		hal_KSCAN_IRQHandler();

}

//RTC
ATTRIBUTE_ISR void V6_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V6_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();

}


//None
ATTRIBUTE_ISR void V7_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V7_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();

}

//None
ATTRIBUTE_ISR void V8_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V8_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();

}

ATTRIBUTE_ISR void V9_IRQHandler(void)
{
	
}

//Watchdog
ATTRIBUTE_ISR void V10_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V10_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
	else
		WDT_IRQHandler();		
}

//uart0
ATTRIBUTE_ISR void V11_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V11_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
    else
        SWU_UART0_IRQHandler();

}

//iic 0
ATTRIBUTE_ISR void V12_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V12_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
}

ATTRIBUTE_ISR void V13_IRQHandler(void)
{
	
}

//spi 0
ATTRIBUTE_ISR void V14_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V14_IRQ_HANDLER));
    
	if (pFunc != ((irq_function)SRAM_BASE_ADDR))
		pFunc();
//	else
//		SPI0_IRQHandler();		
}

//spi 1
ATTRIBUTE_ISR void V15_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V15_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
//	else
//		SPI1_IRQHandler();
}

//gpio
ATTRIBUTE_ISR void V16_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V16_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
    else
        GPIO_IRQHandler_r();

}

//uart 1
ATTRIBUTE_ISR void V17_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V17_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
	else
		hal_UART1_IRQHandler();

}

//spif
ATTRIBUTE_ISR void V18_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V18_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
//    AP_SPIF->int_status = 0xffffffff;

}
ATTRIBUTE_ISR void V19_IRQHandler(void)
{
	
}

//timer 1
ATTRIBUTE_ISR void V20_IRQHandler(void)//rom use
{
	irq_function pFunc = NULL;

	pFunc = (irq_function)(JUMP_FUNCTION_GET(V20_IRQ_HANDLER));

	if (pFunc != ((irq_function)SRAM_BASE_ADDR))
		pFunc();
	else
		TIM1_IRQHandler();
}

//timer 2
ATTRIBUTE_ISR void V21_IRQHandler(void)//rom use
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V21_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
    else
		TIM2_IRQHandler();

}

//timer 3
ATTRIBUTE_ISR void V22_IRQHandler(void)//rom use
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V22_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
    else
		TIM3_IRQHandler();
}

//timer 4
ATTRIBUTE_ISR void V23_IRQHandler(void)//sdk use
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V23_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
	else
		hal_TIMER4_IRQHandler();
}

ATTRIBUTE_ISR void V24_IRQHandler(void)
{
	
}
ATTRIBUTE_ISR void V25_IRQHandler(void)
{
	
}
ATTRIBUTE_ISR void V26_IRQHandler(void)
{
	
}
ATTRIBUTE_ISR void V27_IRQHandler(void)
{
	
}
//aes
ATTRIBUTE_ISR void V28_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V28_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
}

//adcc
ATTRIBUTE_ISR void V29_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V29_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc(); 
	else
		hal_ADC_IRQHandler();
}

//qdec
ATTRIBUTE_ISR void V30_IRQHandler(void)
{
    irq_function pFunc = NULL;
        
    pFunc = (irq_function)(JUMP_FUNCTION_GET(V30_IRQ_HANDLER));
    
    if (pFunc != ((irq_function)SRAM_BASE_ADDR))
        pFunc();
//	else
//		QDEC_IRQHandler();
}


ATTRIBUTE_ISR void V31_IRQHandler(void)
{
	
}