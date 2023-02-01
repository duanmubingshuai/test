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


//#include "ap_cp.h"
#include "gpio.h"
//#include "hal_mcu.h"
#include "clock.h"
#include "i2c_s.h"
#include "i2c.h"
#include "error.h"
#include "log.h"
#include "rom_sym_def.h"
#include "jump_function.h"
#include "pico_reg.h"
typedef struct{
  uint8_t           id;  //0: uninit, 1: i2c0, 2:i2c1
  uint8_t           mode; //(1)I2CS_MODE_REG_8BIT,  (2)I2CS_MODE_REG_16BIT,(3)I2CS_MODE_RAW
  uint8_t           saddr;
  GPIO_Pin_e        cs;  //cs pin, used to  wakeup or release
  GPIO_Pin_e        sda;
  GPIO_Pin_e        scl;
  AP_I2C_TypeDef*   dev;
  i2cs_hdl_t        evt_handler;
  uint8_t           rxoffset;
  uint8_t           rxbuf[I2CS_RX_MAX_SIZE];
  uint8_t           txoffset;
  uint8_t           txbuf[I2CS_TX_MAX_SIZE];
}i2cs_ctx_t;

const int iics_pmux_config[4][4] = {
{P4,P5 , FMUX_P4_IIC_SCL, FMUX_P5_IIC_SDA },
{P9,P10, FMUX_P9_IIC_SCL, FMUX_P10_IIC_SDA},
{P14,P15, FMUX_P14_IIC_SCL,FMUX_P15_IIC_SDA},
{P18,P19, FMUX_P18_IIC_SCL,FMUX_P19_IIC_SDA},
};

typedef enum
{
	IICS_P4SCL_P5SDA   = 0,
	IICS_P9SCL_P10SDA  = 1,
	IICS_P14SCL_P15SDA = 2,
	IICS_P18SCL_P19SDA = 3,
}iics_pin_e;

static uint8_t s_i2cs_state = I2CSST_IDLE;

static i2cs_ctx_t s_i2cs_ctx;

int i2cs_pin_init(iic_pin_e pin_mod)
{
	const int* iic_pin_mod = iics_pmux_config[pin_mod];

	gpio_fmux_set(iic_pin_mod[IIC_PIN_SCL],iic_pin_mod[FMUX_IIC_SCL]);
	gpio_fmux_set(iic_pin_mod[IIC_PIN_SDA],iic_pin_mod[FMUX_IIC_SDA]);
	gpio_pull_set(iic_pin_mod[IIC_PIN_SCL],GPIO_PULL_UP);
	gpio_pull_set(iic_pin_mod[IIC_PIN_SDA],GPIO_PULL_UP);
	clk_gate_enable(MOD_I2C0);
	return PPlus_SUCCESS;
}
static uint8_t i2cs_irq_rx_handler(AP_I2C_TypeDef* pdev)
{
	uint32_t val;
	
	while(1){
		if((pdev->IC_STATUS & BV(3)) == 0)              //rx fifo empty
			break;
			

		val = i2c0_ic_data_cmd_dat_getf();


	}
	return val;
}
static uint32_t tx_dummy = 2; 
void i2cs_irq_tx_handler(AP_I2C_TypeDef* pdev)
{

	for(uint8_t send_count = 2; send_count > 0; send_count--)
	{
		
		i2c0_ic_data_cmd_set(tx_dummy &0xff);
		tx_dummy ++;
	}

}
static void i2cs_irq_handler(AP_I2C_TypeDef* pdev)
{

	volatile uint32_t int_status = i2c0_ic_intr_stat_get();
	volatile uint32_t clr = i2c0_ic_clr_intr_clr_intr_getf();

	if(int_status & I2C_MASK_TX_ABRT)
	{
		LOG("ABRT interr!!\n");
		volatile uint32_t clr_abrt = *(uint32_t *)(0x40005054);
	}

	if(int_status & I2C_MASK_RX_FULL)               //rx fifo full 
	{
		i2cs_irq_rx_handler(pdev);
	}
	if(int_status & I2C_MASK_RD_REQ)                //master want to read the slave
	{
		i2cs_irq_tx_handler(pdev);
	}

	volatile uint32_t wait_count = 1,bus_state;


}

void hal_I2C0_IRQHandler(void)
{
    i2cs_irq_handler(AP_I2C0);
}

int i2cs_init(uint8_t saddr)
{
	i2cs_ctx_t* pctx = &s_i2cs_ctx;

	AP_I2C_TypeDef* pdev = AP_I2C0;;

	clk_gate_enable(MOD_I2C0);


	i2c0_ic_enable_enable_setf(0);
	i2c0_ic_con_set(0x00000000);                 //set slave enable,
	i2c0_ic_con_speed_setf(SPEED_FAST);
	i2c0_ic_sar_ic_sar_setf(saddr);
	i2c0_ic_rx_tl_set(0);
	i2c0_ic_tx_tl_set(1);
	i2c0_ic_intr_mask_set(0x864);//mask I2C_txfifo not empty
	i2c0_ic_enable_enable_setf(1);

	NVIC_EnableIRQ((IRQn_Type)I2C0_IRQn);
		
	NVIC_SetPriority(I2C0_IRQn, IRQ_PRIO_HAL);

	JUMP_FUNCTION_SET(V12_IRQ_HANDLER,hal_I2C0_IRQHandler);

	return PPlus_SUCCESS;
}

//
int i2cs_deinit(void)
{
	i2cs_ctx_t* pctx = &s_i2cs_ctx;
  
	return PPlus_SUCCESS;
}

