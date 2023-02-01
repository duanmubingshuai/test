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
* @file		i2c.c
* @brief	Contains all functions support for i2c driver
* @version	0.0
* @date		25. Oct. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

/*******************************************************************************
*@ Module    		:  pre-compiler
*@ Description    :  NULL
*******************************************************************************/
#define _I2C_CMD_

/*******************************************************************************
*@ Module    			:  Includes
*@ Description    :  None
*******************************************************************************/
#include "rom_sym_def.h"
#include "types.h"
#include "gpio.h"
#include "i2c.h"
#include "clock.h"
#include "log.h"
#include "error.h"
#include "OSAL.h"
#include "pwrmgr.h"
#include "pico_reg.h"

#define i2c_timeout_en  FALSE

#define i2c_op_timeout 100

#define EEPROM_SELECT K24C04_512BYTE


const int ss_hlcnt_config[2][7] = {
	{NULL,70,148,230,307,NULL,460},
	{NULL,76,154,236,320,NULL,470},

};
const int fs_hlcnt_config[2][7] = {
	{NULL,10,30 ,48 ,67 ,NULL,105},
	{NULL,17,35 ,54 ,75 ,NULL,103},
};
const int iic_pmux_config[4][4] = {
{P4,P5 , FMUX_P4_IIC_SCL, FMUX_P5_IIC_SDA },
{P9,P10, FMUX_P9_IIC_SCL, FMUX_P10_IIC_SDA},
{P14,P15, FMUX_P14_IIC_SCL,FMUX_P15_IIC_SDA},
{P18,P19, FMUX_P18_IIC_SCL,FMUX_P19_IIC_SDA},
};
typedef enum{
	hcnt_mod = 0,
	lcnt_mod = 1,
}hlcnt_mod;

int I2C_CHECK_TOUT(int timeout, char* loginfo)
{
	uint32_t i2c_to;
	if(i2c_timeout_en == TRUE)
	{
		i2c_to = get_systick();	
		
		if(get_ms_intv(i2c_to) > timeout)
		{
			LOG(loginfo);
			return PPlus_ERR_TIMEOUT;
		}
	}
	return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          master_send_read_cmd
 *
 * @brief       This function process for master send read command;It's vaild when the chip act as master
 *
 * input parameters
 *
 * @param       uint8_t len: read length
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void master_send_read_cmd(uint8_t len)
{
	uint8_t i;
	AP_I2C_TypeDef * pi2cdev = AP_I2C0;
	
	for(i=0;i<len;i++)
	{
		WaitUs(1);                     //The transmitter response speed is not fast enough, so need to wait for 1us
		i2c0_ic_data_cmd_set(0x100);
	}
}

static void i2c_send_byte(uint8_t data)
{
	i2c0_ic_data_cmd_dat_setf(data);
}
 
/**************************************************************************************
 * @fn          i2c_send
 *
 * @brief       This function process for send a serial(programe length) data by i2c interface
 *
 * input parameters
 *
 * @param       unsigned char* str: send data(string)
 *              uint32_t len: send length
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int i2c_send(uint8_t* str,uint8_t len)
{
	uint8_t i;

	for(i=0;i<len;i++)
	{		
		i2c_send_byte(str[i]);
	}
	return PPlus_SUCCESS;
}

int i2c_wait_tx_completed(void)
{
	int cnt = 0;
	AP_I2C_TypeDef * pi2cdev = AP_I2C0;



	while(1)
	{
		cnt++;
//		if(pi2cdev->IC_RAW_INTR_STAT&0x200)//check tx empty
		if(i2c0_ic_raw_intr_stat_get()&0x200)
			break;
		if(PPlus_ERR_TIMEOUT == I2C_CHECK_TOUT(i2c_op_timeout, "i2c_wait_tx_completed TO\n"))
			return PPlus_ERR_TIMEOUT;
	}
	return PPlus_SUCCESS;
}


void i2c_init(I2C_CLOCK_e i2c_clock_rate)
{
	int pclk_mod = clk_get_pclk()/16000000;
	AP_I2C_TypeDef * pi2cdev = AP_I2C0;

	clk_gate_enable(MOD_I2C0);
	i2c0_ic_enable_enable_setf(0);
	i2c0_ic_con_set(0x61);
	if (i2c_clock_rate == I2C_CLOCK_100K)
	{
		i2c0_ic_ss_scl_hcnt_set(ss_hlcnt_config[hcnt_mod][pclk_mod]);
		i2c0_ic_ss_scl_lcnt_set(ss_hlcnt_config[lcnt_mod][pclk_mod]);
	}
	else
	{
		i2c0_ic_fs_scl_hcnt_set(fs_hlcnt_config[hcnt_mod][pclk_mod]);
		i2c0_ic_fs_scl_lcnt_set(fs_hlcnt_config[lcnt_mod][pclk_mod]);
	}
	i2c0_ic_tar_ic_tar_setf(I2C_MASTER_ADDR_DEF);
	i2c0_ic_intr_mask_set(0);
	i2c0_ic_rx_tl_rx_tl_setf(0);
	i2c0_ic_tx_tl_tx_tl_setf(1);
	i2c0_ic_enable_enable_setf(0);
}

int i2c_deinit(void)
{
	AP_I2C_TypeDef * pi2cdev = AP_I2C0;
	i2c0_ic_enable_enable_setf(0);

	clk_gate_disable(MOD_I2C0);


  return PPlus_SUCCESS;
}


/**************************************************************************************
 * @fn          i2c_pin_init
 *
 * @brief       This function process for i2c pin initial(2 lines);You can use two i2c,i2c0 and i2c1,should programe by USE_AP_I2CX 
 *
 * input parameters
 *
 * @param       gpio_pin_e pin_sda: define sda_pin
 *              gpio_pin_e pin_clk: define clk_pin
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/

int i2c_pin_init(iic_pin_e pin_mod)
{
	const int* iic_pin_mod = iic_pmux_config[pin_mod];

	gpio_fmux_set(iic_pin_mod[IIC_PIN_SCL],iic_pin_mod[FMUX_IIC_SCL]);
	gpio_fmux_set(iic_pin_mod[IIC_PIN_SDA],iic_pin_mod[FMUX_IIC_SDA]);
	gpio_pull_set(iic_pin_mod[IIC_PIN_SCL],GPIO_PULL_UP);
	gpio_pull_set(iic_pin_mod[IIC_PIN_SDA],GPIO_PULL_UP);
	clk_gate_enable(MOD_I2C0);
	return PPlus_SUCCESS;
}

int i2c_tx_start(void)
{
	AP_I2C_TypeDef * pi2cdev = AP_I2C0;


	i2c0_ic_enable_enable_setf(1);
	return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          i2c_addr_update
 *
 * @brief       This function process for tar update 
 *
 * input parameters
 *
 * @param       uint8_t addr: address
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int i2c_addr_update(uint8_t addr)
{
	AP_I2C_TypeDef * pi2cdev = AP_I2C0;

	i2c0_ic_enable_enable_setf(0);
	WaitUs(1);
	i2c0_ic_tar_ic_tar_setf(addr);

	i2c0_ic_enable_enable_setf(0);
	return PPlus_SUCCESS;
}

int i2c_read(
  uint16_t slave_addr,
  uint16_t reg,
  uint8_t* data,
  uint8_t size
  )
{
	uint8_t cnt,cnt2;
	uint32_t wait_cnt = 0;
	int ret = PPlus_SUCCESS;

	AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)AP_I2C0;

	while(size)
	{
		cnt = (size >8) ? 8 : size;
		size -= cnt;
		cnt2 = cnt;
		i2c_addr_update(slave_addr);
		_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
		i2c_tx_start();
		
		i2c_send_byte(reg);

		HAL_EXIT_CRITICAL_SECTION();
		master_send_read_cmd(cnt2);
		
		while(!I2C_RX_FIFO_NOT_EMPTY(pi2cdev))
		{
			if(wait_cnt++>20000)
			{
				LOG("\nreceive anything\n");
				wait_cnt = 0;
				return PPlus_ERR_TIMEOUT;
			}
		}
		
		for(; cnt2 > 0; cnt2--)
		{

			WaitUs(10);
			if(I2C_RX_FIFO_NOT_EMPTY(pi2cdev))
			{
				*data = (pi2cdev->IC_DATA_CMD&0xff);
				data++;
			}
			if(PPlus_ERR_TIMEOUT == I2C_CHECK_TOUT(i2c_op_timeout*size, "I2C RD TO\n"))
				return PPlus_ERR_TIMEOUT;
		}
		
	}
	return ret;
}
int i2c_write(
  uint16_t slave_addr,
  uint16_t reg,
  uint8_t* data,
  uint8_t size)
{
	uint8_t cnt = 0,cnt2 = 0;
	int ret = PPlus_SUCCESS;

	AP_I2C_TypeDef * pi2cdev = (AP_I2C_TypeDef *)AP_I2C0;

	while(size)
	{
		cnt = (size >8) ? 8 : size;
		size -= cnt;

		i2c_addr_update(slave_addr);
		_HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
		i2c_tx_start();

		i2c_send_byte(reg);

		for(; cnt2 > 0; cnt2--)
		{
			i2c_send_byte(*data);

			data++;
		}	
		HAL_EXIT_CRITICAL_SECTION();		
		if(PPlus_ERR_TIMEOUT == I2C_CHECK_TOUT(i2c_op_timeout*size, "I2C RD TO\n"))
				return PPlus_ERR_TIMEOUT;
		WaitUs(500);
	}
	return ret;
}