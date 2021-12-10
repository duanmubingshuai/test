/*
this file is used for all extern interrupt routine, it need other project to fill the jump table 
tpo realize interrupt function defined by user

*/

#include "bus_dev.h"
#include "timer.h"
#include "uart.h"
#include "gpio.h"

#define JUMP_FUNCTION(a) 0

int   hardFault;
typedef void (*irq_function)(void);

// ====== extern function
extern int TIM0_IRQHandler(void);


// ==========================
void HardFault_IRQHandler(void)
{
	  hardFault++;
	  //gpio_write(P1, 1);
}


void NMI_Handler(void)
{
}

void HardFault_Handler()
{
		 HardFault_IRQHandler();

}


void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

ATTRIBUTE_ISR void SysTick_Handler(void)
{
}

//None
ATTRIBUTE_ISR void V0_IRQHandler(void)
{
}

//None
ATTRIBUTE_ISR void V1_IRQHandler(void)
{
}

//None
ATTRIBUTE_ISR void V2_IRQHandler(void)
{
}

//None
ATTRIBUTE_ISR void V3_IRQHandler(void)
{
}

// LL_IRQHandler BB_IRQ
ATTRIBUTE_ISR void V4_IRQHandler(void)
{
}

// KSCAN
ATTRIBUTE_ISR void V5_IRQHandler(void)
{
}

//RTC
ATTRIBUTE_ISR void V6_IRQHandler(void)
{
}


//None
ATTRIBUTE_ISR void V7_IRQHandler(void)
{
}

//None
ATTRIBUTE_ISR void V8_IRQHandler(void)
{
}

//None
ATTRIBUTE_ISR void V9_IRQHandler(void)
{
}

//Watch dog
ATTRIBUTE_ISR void V10_IRQHandler(void)
{
}

//uart0
ATTRIBUTE_ISR void V11_IRQHandler(void)
{
		UART0_IRQHandler();

}

//iic 0
ATTRIBUTE_ISR void V12_IRQHandler(void)
{
}

//iic 1
ATTRIBUTE_ISR void V13_IRQHandler(void)
{
}

//spi 0
ATTRIBUTE_ISR void V14_IRQHandler(void)
{
}

//spi 1
ATTRIBUTE_ISR void V15_IRQHandler(void)
{
}

////gpio
ATTRIBUTE_ISR void V16_IRQHandler(void)
{
}

//uart 1
ATTRIBUTE_ISR void V17_IRQHandler(void)
{
}

//spif
ATTRIBUTE_ISR void V18_IRQHandler(void)
{
}

//dmac
ATTRIBUTE_ISR void V19_IRQHandler(void)
{
}

//timer 0
ATTRIBUTE_ISR void V20_IRQHandler(void)
{
    TIM0_IRQHandler();
}

//timer 1
ATTRIBUTE_ISR void V21_IRQHandler(void)
{
}

//timer 2
ATTRIBUTE_ISR void V22_IRQHandler(void)
{
}

//timer 3
ATTRIBUTE_ISR void V23_IRQHandler(void)
{
}

//timer 4
ATTRIBUTE_ISR void V24_IRQHandler(void)
{
}

//timer 5
ATTRIBUTE_ISR void V25_IRQHandler(void)
{
}


//None
ATTRIBUTE_ISR void V26_IRQHandler(void)
{
}

//None
ATTRIBUTE_ISR void V27_IRQHandler(void)
{
}

//aes
ATTRIBUTE_ISR void V28_IRQHandler(void)
{
}

//adcc
ATTRIBUTE_ISR void V29_IRQHandler(void)
{
}

//qdec
ATTRIBUTE_ISR void V30_IRQHandler(void)
{
}

//None
ATTRIBUTE_ISR void V31_IRQHandler(void)
{
}


