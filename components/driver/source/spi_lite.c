

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
* @file   spi_lite.c
* @brief  Contains all functions support for spi driver
* @version  0.0
* @date   18. Oct. 2022
* @author LYF
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "rom_sym_def.h"
#include "bus_dev.h"
#include "spi_lite.h"
#include "gpio.h"
#include "error.h"
#include <string.h>
#include "pwrmgr.h"
#include "clock.h"
#include "log.h"
#include "OSAL.h"
#include "pico_reg.h"

spilite_cfg_t m_spicfg;



static void spi_master_init(uint32_t baud,SPI_SCMOD_e scmod,SPI_TMOD_e tmod) 
{
	uint8_t shift = 0;
	//AP_SSI_TypeDef *Ssix = AP_SPI0; 
	AP_COM_TypeDef *apcom = AP_COM;
	uint16_t baud_temp;
	int pclk = clk_get_pclk();
	
	spi0_ssienr_set(0x00); 

	apcom->PERI_MASTER_SELECT |= (BIT(shift)|BIT(shift+4));

	spi0_ctrlr0_set((spi0_ctrlr0_cfs_getf() & 0xfffffc3f)|(scmod<<6)|(tmod<<8));

	baud_temp = (pclk + (baud>>1)) / baud;
	if(baud_temp<2)
	{
		baud_temp = 2;
	}
	else if(baud_temp>65534)
	{
		baud_temp =65534;
	} 
	
	spi0_baudr_set(baud_temp);
	spi0_txftlr_set(4);
	spi0_rxftlr_set(1);
	spi0_imr_set(0x00);
	spi0_ser_set(1);
	spi0_ssienr_set(1);
	
}



int spi_transmit(uint8_t* tx_buf,uint8_t* rx_buf,uint16_t len)
{
  uint32_t rx_size = len, tx_size = len;
  uint32_t tmp_len,i;
  //AP_SSI_TypeDef *Ssix = AP_SPI0; 
  
	while(1){
		if(spi0_sr_tfnf_getf() && tx_size)
		{
			tmp_len = 8-spi0_txflr_get();
			if(tmp_len > tx_size)
				tmp_len = tx_size;
			for(i = 0; i< tmp_len; i++){
			if(tx_buf)
				spi0_dr_set(*tx_buf++);
			else
				spi0_dr_set(0);
			}
		tx_size -= tmp_len;
		}
		if(spi0_rxflr_get())
	    {
    		tmp_len = spi0_rxflr_get();
    		
    		for(i = 0; i< tmp_len; i++){
          		if(rx_buf){
        	  	    *rx_buf++ = (uint8_t) spi0_dr_get();
                }
                else
                {
                    spi0_dr_get();
                }
    		}
    		rx_size -= tmp_len;
		}
		if(rx_size == 0)
			break;
	}
	while(spi0_sr_busy_getf())
	{
	  ;
	}

	return PPlus_SUCCESS;
}

/*shot transmit, len < fifo size*/
int spi_transmit_s(uint8_t* tx_buf,uint8_t* rx_buf,uint8_t len)
{
  uint8_t i;
  //AP_SSI_TypeDef *Ssix = AP_SPI0; 

	for(i = 0; i< len; i++){
    if(tx_buf)
			spi0_dr_set(*tx_buf++);
		else
			spi0_dr_set(0);
	}

  while(spi0_rxflr_get() < len)
  {;}
  
	for(i = 0; i< len; i++){
	  if(rx_buf)
          *rx_buf++ = (uint8_t)spi0_dr_get();
      else
          spi0_dr_get();
	}

	return PPlus_SUCCESS;
}


static void spi_wakeup_handler(void)
{
  spi_bus_init(NULL);
}



int spi_bus_init(spilite_cfg_t* pcfg)
{
  spilite_cfg_t* pctx = NULL;
  if(pcfg)
    m_spicfg = *pcfg;
    
  pctx = &m_spicfg;
  
  clk_gate_enable((MODULE_e)MOD_SPI0);

  //LOG("pctx->sclk_pin.pin %x\n",pctx->sclk_pin.pin);
  
  spi_master_init(pctx->baudrate, pctx->spi_scmod, SPI_TRXD);
  gpio_fmux_set(pctx->sclk_pin.pin , pctx->sclk_pin.pmux);
  if(pctx->MOSI.pin != GPIO_DUMMY)
    gpio_fmux_set(pctx->MOSI.pin , pctx->MOSI.pmux);
  if(pctx->MISO.pin != GPIO_DUMMY)
    gpio_fmux_set(pctx->MISO.pin , pctx->MISO.pmux);
  if(pctx->ssn_pin.pin != GPIO_DUMMY)
    gpio_fmux_set(pctx->ssn_pin.pin , pctx->ssn_pin.pmux);
  
  return PPlus_SUCCESS;
}




/**************************************************************************************
 * @fn          hal_spi_init
 *
 * @brief       it is used to init spi module.
 *
 * input parameters
 * @param       None
 *
 * output parameters
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
int spi_init(void)
{
  int ret = PPlus_SUCCESS;

  ret = pwrmgr_register(MOD_SPI0,NULL, spi_wakeup_handler);
  if(ret == PPlus_SUCCESS)
    osal_memset(&m_spicfg,0,sizeof(m_spicfg));

  return ret;
}

