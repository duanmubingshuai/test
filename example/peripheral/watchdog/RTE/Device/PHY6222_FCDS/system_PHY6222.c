/**************************************************************************//**
    @file     system_PHY6222.c
    @brief    CMSIS Device System Source File for
             PHY6222 Device Series
    @version  V1.0
    @date     20. Sept 2021

    @note

 ******************************************************************************/
/*	Phyplus Microelectronics Limited confidential and proprietary. 
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

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
    ---------------------------------------------------------------------------*/


#include "PHY62xx.h"

/*  ----------------------------------------------------------------------------
    Define clocks
    ----------------------------------------------------------------------------*/
#define __HSI             ( 8000000UL)
#define __XTAL            ( 5000000UL)    /* Oscillator frequency             */

#define __SYSTEM_CLOCK    (5*__XTAL)


/*  ----------------------------------------------------------------------------
    System Core Clock Variable
    ----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __SYSTEM_CLOCK;/* System Core Clock Frequency      */


/**
    Update SystemCoreClock variable

    @param  none
    @return none

    @brief  Updates the SystemCoreClock with current core Clock
           retrieved from cpu registers.
*/
void SystemCoreClockUpdate (void)
{
    SystemCoreClock = __SYSTEM_CLOCK;
}

/**
    Initialize the system

    @param  none
    @return none

    @brief  Setup the microcontroller system.
           Initialize the System.
*/
void SystemInit (void)
{
    SystemCoreClock = __SYSTEM_CLOCK;
}
