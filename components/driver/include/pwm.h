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

/*******************************************************************************
* @file		pwm.h
* @brief	Contains all functions support for pwm driver
* @version	0.0
* @date		30. Oct. 2017
* @author	Ding
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __PWM_ROM_H__
#define __PWM_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"
#include "bus_dev.h"
#include "gpio.h"


#define   PWM_CH_NUM    6


#define    PWM_CH_BASE              (AP_PWM_BASE + 4)
#define    PWM_CTL0_ADDR(n)        	(PWM_CH_BASE + n*12)
#define    PWM_CTL1_ADDR(n)        	(PWM_CH_BASE + n*12 + 4)

#define    PWM_ENABLE_ALL()          	(AP_PWM->PWMEN |= (BIT(0)|BIT(4)))

#define    PWM_ENABLE_CH_012()      	(AP_PWM->PWMEN |= (BIT(8)|BIT(9)))
#define    PWM_ENABLE_CH_345()    	  (AP_PWM->PWMEN |= (BIT(10)|BIT(11)))

#define    PWM_ENABLE_CH_01()        	(AP_PWM->PWMEN |= (BIT(12)|BIT(13)))
#define    PWM_ENABLE_CH_23()        	(AP_PWM->PWMEN |= (BIT(14)|BIT(15)))
#define    PWM_ENABLE_CH_45()        	(AP_PWM->PWMEN |= (BIT(16)|BIT(17)))

#define    PWM_DISABLE() 			        (AP_PWM->PWMEN = 0)

//#define    PWM_DISABLE_ALL 			    (AP_PWM->PWMEN &=  ~(BIT(0)|BIT(4)))
//#define    PWM_DISABLE_CH_012       (AP_PWM->PWMEN &=  ~(BIT(8)|BIT(9)))
//#define    PWM_DISABLE_CH_345      	(AP_PWM->PWMEN &=  ~(BIT(10)|BIT(11)))
//#define    PWM_DISABLE_CH_01       	(AP_PWM->PWMEN &=  ~(BIT(12)|BIT(13)))
//#define    PWM_DISABLE_CH_23       	(AP_PWM->PWMEN &=  ~(BIT(14)|BIT(15)))
//#define    PWM_DISABLE_CH_45    	  (AP_PWM->PWMEN &=  ~(BIT(16)|BIT(17)))

						
#define    PWM_INSTANT_LOAD_CH(n)  	  subWriteReg(PWM_CTL0_ADDR(n),31,31,1)
#define    PWM_NO_INSTANT_LOAD_CH(n)  subWriteReg(PWM_CTL0_ADDR(n),31,31,0)
#define    PWM_LOAD_CH(n) 			      subWriteReg(PWM_CTL0_ADDR(n),16,16,1)
#define    PWM_NO_LOAD_CH(n) 		      subWriteReg(PWM_CTL0_ADDR(n),16,16,0)																 
#define    PWM_SET_DIV(n,v) 		      subWriteReg(PWM_CTL0_ADDR(n),14,12,v)
#define    PWM_SET_MODE(n,v) 	   	    subWriteReg(PWM_CTL0_ADDR(n),8,8,v)
#define    PWM_SET_POL(n,v) 		      subWriteReg(PWM_CTL0_ADDR(n),4,4,v)
#define    PWM_ENABLE_CH(n)        	  subWriteReg(PWM_CTL0_ADDR(n),0,0,1)
#define    PWM_DISABLE_CH(n)       	  subWriteReg(PWM_CTL0_ADDR(n),0,0,0)
									
#define    PWM_SET_CMP_VAL(n,v) 	    subWriteReg(PWM_CTL1_ADDR(n),31,16,v)					
#define    PWM_SET_TOP_VAL(n,v) 	    subWriteReg(PWM_CTL1_ADDR(n),15,0,v)
#define    PWM_GET_CMP_VAL(n) 	   	  ((read_reg(PWM_CTL1_ADDR(n)) & 0xFFFF0000) >> 8)
#define    PWM_GET_TOP_VAL(n) 	   	  (read_reg(PWM_CTL1_ADDR(n)) & 0x0000FFFF)


/*************************************************************
*	@brief		enum variable, the number of PWM channels supported
*
*/

/*************************************************************
*	@brief		enum variable used for PWM clock prescaler
*
*/
typedef enum
{
    PWM_CLK_NO_DIV = 0,
    PWM_CLK_DIV_2 = 1,
    PWM_CLK_DIV_4 = 2,
    PWM_CLK_DIV_8 = 3,
    PWM_CLK_DIV_16 = 4,
    PWM_CLK_DIV_32 = 5,
    PWM_CLK_DIV_64 = 6,
    PWM_CLK_DIV_128 = 7
} PWM_CLK_DIV_e;

/*************************************************************
*	@brief		enum variable used for PWM work mode setting
*
*/
typedef enum
{
    PWM_CNT_UP = 0,
    PWM_CNT_UP_AND_DOWN = 1
} PWM_CNT_MODE_e;

/*************************************************************
*	@brief		enum variable used for PWM output polarity setting
*
*/
typedef enum
{
    PWM_POLARITY_RISING = 0,
    PWM_POLARITY_FALLING = 1
} PWM_POLARITY_e;

typedef struct
{	
	gpio_pin_e  pin;
	uint8_t     ch;         //channel number
  uint8_t     div;        //3bit,0(no div)~7(128 div)
  uint8_t     mode;       //1bit,PWM_CNT_UP=0,PWM_CNT_UP_AND_DOWN=1
  uint8_t     polarity;   //1bit,PWM_POLARITY_RISING=0,PWM_POLARITY_FALLING=1	
  uint16_t    cmp_val;    //compare value
  uint16_t    top_val;    //counter top value of PWM channe;
} pwm_cfg_t;

uint8_t pwm_get_channel(gpio_pin_e pin);
int pwm_init(void);
void pwm_deinit(void);
int pwm_ch_start(pwm_cfg_t* pcfg);
int pwm_ch_stop(pwm_cfg_t* pcfg);
int pwm_update(pwm_cfg_t* pcfg);


#ifdef __cplusplus
}
#endif

#endif
