;/**************************************************************************//**
; * @file     startup_ARMCM4.s
; * @brief    CMSIS Core Device Startup File for
; *           ARMCM4 Device Series
; * @version  V5.00
; * @date     02. March 2016
; ******************************************************************************/
;/*
; * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
; *
; * SPDX-License-Identifier: Apache-2.0
; *
; * Licensed under the Apache License, Version 2.0 (the License); you may
; * not use this file except in compliance with the License.
; * You may obtain a copy of the License at
; *
; * http://www.apache.org/licenses/LICENSE-2.0
; *
; * Unless required by applicable law or agreed to in writing, software
; * distributed under the License is distributed on an AS IS BASIS, WITHOUT
; * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; * See the License for the specific language governing permissions and
; * limitations under the License.
; */

;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000800

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size
				;IMPORT	TIM0_IRQHandler

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     CPAPIPC_IRQHandler        ; 0:cpcom_ap_ipc_irq
                DCD     APAPIPC_IRQHandler        ; 1:apcom_ap_ipc_irq
                DCD     AP_TIM0_IRQHandler        ; 2:timer_irq
                DCD     WDC_IRQHandler            ; 3:wdt_irq
                DCD     UART0_IRQHandler          ; 4:uart_irq
                DCD     I2C0_IRQHandler           ; 5:i2c0_irq
                DCD     I2C1_IRQHandler           ; 6:i2c1_irq
                DCD     SPI0_IRQHandler           ; 7:spi0_irq
                DCD     SPI1_IRQHandler           ; 8:spi1_irq
                DCD     GPIO_IRQHandler           ; 9:gpio_irq
                DCD     I2S_IRQHandler            ; 10:i2s_irq
                DCD     SPIF_IRQHandler           ; 11:spif_irq
                DCD     DMACINTR_IRQHandler       ; 12:dmac_intr
                DCD     DMACINTTC_IRQHandler      ; 13:dmac_inttc
                DCD     DMACINTERR_IRQHandler     ; 14:dmac_interr
                DCD     FPIDC_IRQHandler          ; 15:fpidc
                DCD     FPDZC_IRQHandler          ; 16:fpdzc
                DCD     FPIOC_IRQHandler          ; 17:fpioc
                DCD     FPUFC_IRQHandler          ; 18:fpufc
                DCD     FPOFC_IRQHandler          ; 19:fpofc
                DCD     FPIXC_IRQHandler          ; 20:fpixc
                DCD     AES_IRQHandler            ; 21:aes_irq
                DCD     ADCC_IRQHandler           ; 22:adcc_irq
                DCD     QDEC_IRQHandler           ; 23:qdec_irq
                DCD     RNG_IRQHandler            ; 24:rng_irq
                DCD     APCPIPC_IRQHandler        ; 25:apcom_cp_ipc_irq
                DCD     CPCPIPC_IRQHandler        ; 26:cpcom_cp_ipc_irq
                ;DCD     TIM0_IRQHandler           ; 27:cp_timer_irq
                DCD     CPWDT_IRQHandler          ; 28:cp_wdt_irq
                DCD     LL_IRQHandler             ; 29:bb_irq
                DCD     KSCAN_IRQHandler          ; 30:kscan_irq
                DCD     RTC_IRQHandler            ; 31:rtc_irq
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT CPAPIPC_IRQHandler        [WEAK]; 0:cpcom_ap_ipc_irq
                EXPORT APAPIPC_IRQHandler        [WEAK]; 1:apcom_ap_ipc_irq
                EXPORT AP_TIM0_IRQHandler        [WEAK]; 2:timer_irq
                EXPORT WDC_IRQHandler            [WEAK]; 3:wdt_irq
                EXPORT UART0_IRQHandler          [WEAK]; 4:uart_irq
                EXPORT I2C0_IRQHandler           [WEAK]; 5:i2c0_irq
                EXPORT I2C1_IRQHandler           [WEAK]; 6:i2c1_irq
                EXPORT SPI0_IRQHandler           [WEAK]; 7:spi0_irq
                EXPORT SPI1_IRQHandler           [WEAK]; 8:spi1_irq
                EXPORT GPIO_IRQHandler           [WEAK]; 9:gpio_irq
                EXPORT I2S_IRQHandler            [WEAK]; 10:i2s_irq
                EXPORT SPIF_IRQHandler           [WEAK]; 11:spif_irq
                EXPORT DMACINTR_IRQHandler       [WEAK]; 12:dmac_intr
                EXPORT DMACINTTC_IRQHandler      [WEAK]; 13:dmac_inttc
                EXPORT DMACINTERR_IRQHandler     [WEAK]; 14:dmac_interr
                EXPORT FPIDC_IRQHandler          [WEAK]; 15:fpidc
                EXPORT FPDZC_IRQHandler          [WEAK]; 16:fpdzc
                EXPORT FPIOC_IRQHandler          [WEAK]; 17:fpioc
                EXPORT FPUFC_IRQHandler          [WEAK]; 18:fpufc
                EXPORT FPOFC_IRQHandler          [WEAK]; 19:fpofc
                EXPORT FPIXC_IRQHandler          [WEAK]; 20:fpixc
                EXPORT AES_IRQHandler            [WEAK]; 21:aes_irq
                EXPORT ADCC_IRQHandler           [WEAK]; 22:adcc_irq
                EXPORT QDEC_IRQHandler           [WEAK]; 23:qdec_irq
                EXPORT RNG_IRQHandler            [WEAK]; 24:rng_irq
                EXPORT APCPIPC_IRQHandler        [WEAK]; 25:apcom_cp_ipc_irq
                EXPORT CPCPIPC_IRQHandler        [WEAK]; 26:cpcom_cp_ipc_irq
                ;EXPORT CPTIM_IRQHandler          [WEAK]; 27:cp_timer_irq
                EXPORT CPWDT_IRQHandler          [WEAK]; 28:cp_wdt_irq
                EXPORT LL_IRQHandler             [WEAK]; 29:bb_irq
                EXPORT KSCAN_IRQHandler          [WEAK]; 30:kscan_irq
                EXPORT RTC_IRQHandler            [WEAK]; 31:rtc_irq

               
CPAPIPC_IRQHandler    
APAPIPC_IRQHandler    
AP_TIM0_IRQHandler       
WDC_IRQHandler        
UART0_IRQHandler      
I2C0_IRQHandler       
I2C1_IRQHandler       
SPI0_IRQHandler       
SPI1_IRQHandler       
GPIO_IRQHandler       
I2S_IRQHandler        
SPIF_IRQHandler       
DMACINTR_IRQHandler   
DMACINTTC_IRQHandler  
DMACINTERR_IRQHandler 
FPIDC_IRQHandler      
FPDZC_IRQHandler      
FPIOC_IRQHandler      
FPUFC_IRQHandler      
FPOFC_IRQHandler      
FPIXC_IRQHandler      
AES_IRQHandler        
ADCC_IRQHandler       
QDEC_IRQHandler       
RNG_IRQHandler        
APCPIPC_IRQHandler    
CPCPIPC_IRQHandler    
;CPTIM_IRQHandler      
CPWDT_IRQHandler      
LL_IRQHandler         
KSCAN_IRQHandler      
RTC_IRQHandler        

                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
