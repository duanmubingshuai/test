/*******************************************************************************
* @file     gpio.h
* @brief    Contains all functions support for gpio driver
* @version  0.0
* @date     27. Nov. 2017
* @author   qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __GPIO_ROM_H__
#define __GPIO_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <types.h>
//#include "bus_dev.h"
#include "error.h"

//special gpio:P35~P40,no pull,no retention,no wakeup
//#define SPECIAL_GPIO_MIN P35
//#define SPECIAL_GPIO_NUM 6

typedef enum{
    GPIO_P00   =   0,    P0  =  GPIO_P00,
    GPIO_P01   =   1,    P1  =  GPIO_P01,
    GPIO_P02   =   2,    P2  =  GPIO_P02,
    GPIO_P03   =   3,    P3  =  GPIO_P03,
    GPIO_P04   =   4,    P4  =  GPIO_P04, Analog_IO_0 =  GPIO_P04,
    GPIO_P05   =   5,    P5  =  GPIO_P05, Analog_IO_1 =  GPIO_P05,
    GPIO_P06   =   6,    P6  =  GPIO_P06, Analog_IO_2 =  GPIO_P06,
    GPIO_P07   =   7,    P7  =  GPIO_P07, Analog_IO_3 =  GPIO_P07,
    GPIO_P08   =   8,    P8  =  GPIO_P08, Analog_IO_4 =  GPIO_P08,
    GPIO_P09   =   9,    P9  =  GPIO_P09, Analog_IO_5 =  GPIO_P09,
    GPIO_P10   =   10,   P10  =  GPIO_P10,Analog_IO_6 =  GPIO_P10,
    GPIO_P11   =   11,   P11  =  GPIO_P11,Analog_IO_7 =  GPIO_P11,
    GPIO_P12   =   12,   P12  =  GPIO_P12,
    GPIO_P13   =   13,   P13  =  GPIO_P13,
    GPIO_P14   =   14,   P14  =  GPIO_P14,
    GPIO_P15   =   15,   P15  =  GPIO_P15,
    GPIO_P16   =   16,   P16  =  GPIO_P16,
    GPIO_P17   =   17,   P17  =  GPIO_P17,
    GPIO_P18   =   18,   P18  =  GPIO_P18,
    GPIO_P19   =   19,   P19  =  GPIO_P19,	
    GPIO_NUM   =   20,
    GPIO_DUMMY =  0xff,
}gpio_pin_e;

typedef enum {  
    FMUX_GPIO = 0,

    FMUX_P0_GPIO = FMUX_GPIO,
    FMUX_P0_USBDP = 1,
    FMUX_P0_PWM0 = 2,
    FMUX_P0_UART_TX = 3,
    
    FMUX_P1_GPIO = FMUX_GPIO,
    FMUX_P1_USBDM = 1,
    FMUX_P1_PWM1 = 2,
    FMUX_P1_UART_RX = 3,

    FMUX_P2_SW_DIO = 0,
    FMUX_P2_GPIO = 1,   
    FMUX_P2_SPI_RX = 2,
    FMUX_P2_PWM0 = 3,
	FMUX_P2_UART_TX = 4,
		
    FMUX_P3_SW_CLK = 0,
    FMUX_P3_GPIO = 1,   
    FMUX_P3_SPI_SSN = 2,
    FMUX_P3_PWM1 = 3,
	FMUX_P3_UART_RX = 4,
	
    FMUX_P4_GPIO = FMUX_GPIO,
    FMUX_P4_IIC_SCL = 1,
    FMUX_P4_SPI_SCK = 2,
    FMUX_P4_ANA0 = 3,
	FMUX_P4_UART_TX = 4,
		
    FMUX_P5_GPIO = FMUX_GPIO,
    FMUX_P5_IIC_SDA = 1,
    FMUX_P5_UART_TX = 2,
    FMUX_P5_ANA1 = 3,
	FMUX_P5_UART_RX = 4,
		
    FMUX_P6_GPIO = FMUX_GPIO,
    FMUX_P6_PWM2 = 1,
    FMUX_P6_USBDP = 2,
    FMUX_P6_ANA2 = 3,

    FMUX_P7_GPIO = FMUX_GPIO,
    FMUX_P7_PWM3 = 1,
    FMUX_P7_USBDM = 2,
    FMUX_P7_ANA3 = 3,

    FMUX_P8_GPIO = FMUX_GPIO,
    FMUX_P8_PWM4 = 1,
    FMUX_P8_RF_RX_EN = 2,
    FMUX_P8_ANA4 = 3,

    FMUX_P9_GPIO = FMUX_GPIO,
    FMUX_P9_UART_TX = 1,
    FMUX_P9_IIC_SCL = 2,
    FMUX_P9_ANA3 = 3,

    FMUX_P10_GPIO = FMUX_GPIO,
    FMUX_P10_UART_RX = 1,
    FMUX_P10_IIC_SDA = 2,    
    FMUX_P10_ANA4 = 3,

    FMUX_P11_GPIO = FMUX_GPIO,
    FMUX_P11_PWM5 = 1,
    FMUX_P11_RF_TX_EN = 2,
    FMUX_P11_ANA5 = 3,

    FMUX_P12_GPIO = FMUX_GPIO,
    FMUX_P12_SPI_RX = 1,
    FMUX_P12_PWM2 = 2,
//    FMUX_P12_DBG_MUX0 = 3,
    FMUX_P12_ANA6 = 3,

    FMUX_P13_GPIO = FMUX_GPIO,
    FMUX_P13_SPI_SSN = 1,
    FMUX_P13_PWM3 = 2,
//    FMUX_P13_DBG_MUX1 = 3,
    FMUX_P13_ANA7 = 3,

    FMUX_P14_GPIO = FMUX_GPIO,
    FMUX_P14_SPI_CLK = 1,
    FMUX_P14_PWM4 = 2,
    FMUX_P14_IIC_SCL = 3,

    FMUX_P15_GPIO = FMUX_GPIO,
    FMUX_P15_SPI_TX = 1,
    FMUX_P15_PWM5 = 2,
    FMUX_P15_IIC_SDA = 3,

    FMUX_P16_GPIO = FMUX_GPIO,
    FMUX_P16_UART_TX = 1,
    FMUX_P16_RF_TX_EN = 2,
    FMUX_P16_SPI_CLK = 3,

    FMUX_P17_GPIO = FMUX_GPIO,
    FMUX_P17_UART_RX = 1,
    FMUX_P17_RF_RX_EN = 2,
    FMUX_P17_SPI_TX = 3,

    FMUX_P18_GPIO = FMUX_GPIO,
    FMUX_P18_PWM0 = 1,
    FMUX_P18_IIC_SCL = 2,
    FMUX_P18_SPI_RX = 3,

    FMUX_P19_GPIO = FMUX_GPIO,
    FMUX_P19_PWM1 = 1,
    FMUX_P19_IIC_SDA = 2,
    FMUX_P19_SPI_SSN = 3,
    FMUX_UNKNOW = 0xff
}gpio_fmux_e;
 
typedef enum{
    GPIO_INPUT  = 0,IE = 0,
    GPIO_OUTPUT = 1,OEN = 1,
}gpio_dir_t;


typedef enum{
    POL_FALLING = 0, POL_ACT_LOW  = 0,NEGEDGE = 0,
    POL_RISING  = 1, POL_ACT_HIGH = 1,POSEDGE = 1,
}gpio_polarity_e;


typedef enum
{
    GPIO_FLOATING   = 0x00, FLOATING = 0x00,       //no pull
    GPIO_PULL_UP    = 0x01, WEAK_PULL_UP = 0x01,   //150K
    GPIO_PULL_DOWN  = 0x03, PULL_DOWN = 0x03,      //150K
} gpio_pupd_e;
	

typedef enum{
    Bit_DISABLE = 0,
    Bit_ENABLE = 1,
}bit_action_e;

enum{
    GPIO_EVT_EDGE_RISING = 1,
    GPIO_EVT_EDGE_FALLING
};

#define IO_Wakeup_Pol_e     gpio_polarity_e
#define GPIO_Pin_e          gpio_pin_e
#define Fmux_Type_e         gpio_fmux_e
#define GPIO_Wakeup_Pol_e   gpio_polarity_e
#define BitAction_e         bit_action_e

//gpio pin callback,pos or neg
typedef void (*gpioin_Hdl_t)(gpio_pin_e pin,gpio_polarity_e type);

typedef struct 
{	
	bool          enable;
	gpio_pin_e    pin;
	int           pin_state;
	gpioin_Hdl_t  posedgeHdl;
	gpioin_Hdl_t  negedgeHdl;
    
}gpioin_t;


typedef struct 
{
  int             state;
  uint32_t        retention_map;
  int             gpioin_nums;
  gpioin_t*  	  gpioin_ctx;
}gpio_Ctx_t;

/*
 * rom use:
 * *_r
 *
*/
void gpio_init_r(void);
void gpio_dir_r(gpio_pin_e pin, gpio_dir_t dir);
int gpio_in_trigger_r(gpio_pin_e pin, comm_cb_t in_cb);
void gpio_interrupt_set_r(gpio_pin_e pin, bit_action_e en);
void GPIO_IRQHandler_r(void);

/*
 * 
 * */
int gpio_init(void);        //this function exist a problem of losing wakeup 

int gpio_init_patch(void);  //This function is used to replace "gpio_init" to solve the wakeup loss problem

void gpio_dir(gpio_pin_e pin, gpio_dir_t dir);
void gpio_write(gpio_pin_e pin, bit_action_e en);
void gpio_fast_write(gpio_pin_e pin, uint8_t en);
bool gpio_read(gpio_pin_e pin);

void gpio_pull_set(gpio_pin_e pin, gpio_pupd_e type);
void gpio_wakeup_set(gpio_pin_e pin, gpio_polarity_e type);
void gpio_ds_control(gpio_pin_e pin, bit_action_e value);
void gpio_fmux_control(gpio_pin_e pin, bit_action_e value);
void gpio_fmux_set(gpio_pin_e pin,gpio_fmux_e type);

int gpioin_init(gpioin_t* gpioin, int pin_nums);
int gpioin_disable(gpioin_t* p_gpioin_ctx);
int gpioin_enable(gpioin_t* p_gpioin_ctx);
int gpioin_register(gpio_pin_e pin, gpioin_Hdl_t posedgeHdl, gpioin_Hdl_t negedgeHdl);
int gpioin_unregister(gpio_pin_e pin);
void gpio_retention(gpio_pin_e pin, bool en);

gpio_Ctx_t* gpio_get_handler(void);

void GPIO_IRQHandler(void);


void hal_gpio_IRQ(void);

#ifdef __cplusplus
}
#endif

#endif
