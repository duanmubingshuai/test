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
* @file   spi.c
* @brief  Contains all functions support for spi driver
* @version  0.0
* @date   18. Oct. 2017
* @author qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "rom_sym_def.h"
#include "bus_dev.h"
#include "spi.h"
#include "gpio.h"
#include "error.h"
#include <string.h>
#include "pwrmgr.h"
#include "clock.h"
#include "log.h"
#include "pico_reg.h"
typedef struct _spi_Context{
  spi_Cfg_t   cfg;
  hal_spi_t*  spi_info;
  bool        is_slave_mode;
  spi_xmit_t  transmit;
}spi_Ctx_t;

static spi_Ctx_t m_spiCtx;

uint8_t pin_flag[4]={0xff,0xff,0xff,0xff};
//extern  uint32_t pclk;

#define SPI_HDL_VALIDATE(hdl)   {if(hdl == NULL)\
                                  return PPlus_ERR_INVALID_PARAM;\
                                if((hdl != m_spiCtx.spi_info) && (hdl != m_spiCtx.spi_info))\
                                  return PPlus_ERR_NOT_REGISTED;}


////////////////// SPI  /////////////////////////////////////////
//this function is used just when you want to use two spi master module
static void spi_write_fifo(uint8_t len,uint8_t* tx_rx_ptr)
{
  uint8_t i=0;
  _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
  while(i<len)
  {
	spi0_dr_dr_setf(*(tx_rx_ptr+i));
    i++;
  }
  HAL_EXIT_CRITICAL_SECTION();
}


void spi_int_enable(uint32_t mask)
{

  spi0_imr_set(mask & 0x11);
  
  NVIC_EnableIRQ((IRQn_Type)SPI0_IRQn);
  NVIC_SetPriority((IRQn_Type)SPI0_IRQn, IRQ_PRIO_HAL);

}

static void spi_int_disable(void)
{
  AP_SSI_TypeDef *Ssix = AP_SPI0; 

  NVIC_DisableIRQ((IRQn_Type)(SPI0_IRQn));
  spi0_imr_set(0x00);
}

static void spi_int_handle(uint8_t id, spi_Ctx_t* pctx, AP_SSI_TypeDef * Ssix)
{ 
  volatile uint8_t spi_irs_status;
  spi_evt_t evt;
  uint8_t i, cnt;
  spi_xmit_t* trans_ptr;
  

  trans_ptr = &pctx->transmit;

  if(spi0_isr_txeis_getf())
  {
    cnt = 8 - Ssix->TXFLR;
    for(i = 0; i< cnt; i++){
      if(trans_ptr->tx_buf){
		spi0_dr_dr_setf(trans_ptr->tx_buf[trans_ptr->tx_offset ++]);

      }
      else
      {
        trans_ptr->tx_offset++;
		spi0_dr_dr_setf(0);

      }
      if(trans_ptr->tx_offset == trans_ptr->xmit_len){
		spi0_imr_set(0x10);

        break;
      }
      
    }
  }
  if(spi0_isr_rxfis_getf())
  {
    cnt = Ssix->RXFTLR;
    for(i = 0; i< cnt; i++)
    {
      trans_ptr->rx_buf[trans_ptr->rx_offset++] = Ssix->DataReg;
    }
        
    if(trans_ptr->rx_offset == trans_ptr->xmit_len)
    {
      if(pctx->cfg.force_cs == true)
        gpio_fmux_control(pctx->cfg.ssn_pin, Bit_ENABLE);

      trans_ptr->busy = false;
      trans_ptr->rx_buf = NULL;
      trans_ptr->rx_offset = 0;
      evt.id = id;
      evt.evt = SPI_RX_COMPLETED;
      pctx->cfg.evt_handler(&evt);
      evt.evt = SPI_TX_COMPLETED;
      pctx->cfg.evt_handler(&evt);
    }
  }
  
}

static void spis_int_handle(uint8_t id, spi_Ctx_t* pctx, AP_SSI_TypeDef * Ssix)
{ 
  volatile uint8_t spi_irs_status;
  spi_xmit_t* trans_ptr;
  spi_evt_t evt;
  uint16_t i, cnt;
  
  
  trans_ptr = &(pctx->transmit);

  if(spi0_isr_txeis_getf())
  {
	cnt = 8 - spi0_txflr_get();
    for(i = 0; i< cnt; i++){
      if(trans_ptr->tx_offset == trans_ptr->xmit_len){
		spi0_imr_set(0x10);
        break;
      }
      if(trans_ptr->tx_buf){
        Ssix->DataReg = trans_ptr->tx_buf[trans_ptr->tx_offset ++];
      }
      else
      {
        trans_ptr->tx_offset ++;
		spi0_dr_set(0x00);
      }
    }
  }
  if(spi0_isr_rxfis_getf())
  {
    volatile uint32_t garbage;
	cnt = spi0_rxflr_get();

    if(trans_ptr->rx_buf){
      for(i = 0; i< cnt; i++)
      {
        if(trans_ptr->xmit_len > trans_ptr->rx_offset)
		  trans_ptr->rx_buf[trans_ptr->rx_offset++] = spi0_dr_get();
        else
		  garbage = spi0_dr_get();

      }
    }
    else
    {
      uint8_t rxbuf[16];
      if(trans_ptr->busy)
        trans_ptr->rx_offset += cnt;
      for(i = 0; i< cnt; i++)
      {
		*(rxbuf+i) = spi0_dr_get();
      }
      evt.id = id;
      evt.evt = SPI_RX_DATA_S;
  		evt.data = rxbuf;
  		evt.len = cnt;
      pctx->cfg.evt_handler(&evt);
      
    }
    
    if(trans_ptr->busy && trans_ptr->rx_offset >= trans_ptr->xmit_len){
      memset(trans_ptr, 0, sizeof(spi_xmit_t));
      evt.id = id;
      evt.evt = SPI_RX_COMPLETED;
  		evt.data = NULL;
  		evt.len = cnt;
      pctx->cfg.evt_handler(&evt);
      evt.evt = SPI_TX_COMPLETED;
      pctx->cfg.evt_handler(&evt);
    }
  }
  
}

/**************************************************************************************
 * @fn          hal_SPI0_IRQHandler
 *
 * @brief       This function process for spi0 interrupt,when use int please consummate its callbackfunction
 *
 * input parameters
 *
 * @param       None.      
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void __attribute__((used)) SPI0_IRQHandler(void)
{
  spi_Ctx_t* pctx = &m_spiCtx;
  if(pctx->spi_info == NULL)
    return;
  if(pctx->is_slave_mode)
    spis_int_handle(0, pctx, AP_SPI0);
  else
    spi_int_handle(0, pctx, AP_SPI0);
}

/**************************************************************************************
 * @fn          hal_spi_pin_init
 *
 * @brief       This function process for spi pin initial(4 lines);You can use two spi,spi0 and spi1,should programe by USE_AP_SPIX 
 *
 * input parameters
 *
 * @param       gpio_pin_e sck_pin: define sclk pin
 *              gpio_pin_e ssn_pin: define ssn pin
 *              gpio_pin_e tx_pin: define transmit pin;when use as master,it's mosi pin;corresponding,use as slave,it's miso
 *              gpio_pin_e rx_pin: define receive pin;when use as master,it's miso pin;corresponding,use as slave,it's mosi
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
const uint8_t spimap[14][2]=
{
	{P2,FMUX_P2_SPI_RX},
	{P3,FMUX_P3_SPI_SSN},
	{P4,FMUX_P4_SPI_SCK},
	{P5,NULL},
	
	{P12,FMUX_P12_SPI_RX},
	{P13,FMUX_P13_SPI_SSN},
	{P14,FMUX_P14_SPI_CLK},
	{P15,FMUX_P15_SPI_TX},
	
	{P16,FMUX_P16_SPI_CLK},
	{P17,FMUX_P17_SPI_TX},
	{P18,FMUX_P18_SPI_RX},
	{P19,FMUX_P19_SPI_SSN},
};

//static int spi_pin_init(hal_spi_t* spi_ptr,gpio_pin_e sck_pin,gpio_pin_e ssn_pin,gpio_pin_e tx_pin,gpio_pin_e rx_pin)
static int spi_pin_init(gpio_pin_e sck_pin,gpio_pin_e ssn_pin,gpio_pin_e tx_pin,gpio_pin_e rx_pin)
{  
	for(int i=0;i<14;i++)
	{
		if(sck_pin == spimap[i][0])
		{
			pin_flag[0] = i;
		}
		
		else if(ssn_pin == spimap[i][0])
		{
			pin_flag[1] = i;
		}
		
		else if(tx_pin == spimap[i][0])
		{
			pin_flag[2] = i;
		}		
		else if(rx_pin == spimap[i][0])
		{
			pin_flag[3] = i;
		}
	}
	
	if((pin_flag[0]!=0xFF) && (pin_flag[0]!=0xFF) && (pin_flag[0]!=0xFF) && (pin_flag[0]!=0xFF))
	{
		gpio_fmux_set(sck_pin, spimap[pin_flag[0]][1]);
		gpio_fmux_set(ssn_pin, spimap[pin_flag[1]][1]);
		gpio_fmux_set(tx_pin, spimap[pin_flag[2]][1]);
		gpio_fmux_set(rx_pin, spimap[pin_flag[3]][1]);
		return PPlus_SUCCESS;
	}
//  }
	return PPlus_ERR_INVALID_PARAM;
}

static void spi_pin_deinit(gpio_pin_e sck_pin,gpio_pin_e ssn_pin,gpio_pin_e tx_pin,gpio_pin_e rx_pin)
{
	gpio_fmux_control(sck_pin, Bit_DISABLE);
	gpio_fmux_control(ssn_pin, Bit_DISABLE); 
	gpio_fmux_control(tx_pin, Bit_DISABLE); 
	gpio_fmux_control(rx_pin, Bit_DISABLE); 
}

/**************************************************************************************
 * @fn          hal_spi_master_init
 *
 * @brief       This function process for spi master initial
 *
 * input parameters
 *
 * @param       uint32_t baud: baudrate select
 *              SPI_SCMOD_e scmod: Serial Clock Polarity and Phase select;  SPI_MODE0,        //SCPOL=0,SCPH=0(default)
 *                                                                          SPI_MODE1,        //SCPOL=0,SCPH=1
 *                                                                          SPI_MODE2,        //SCPOL=1,SCPH=0
 *                                                                          SPI_MODE3,        //SCPOL=1,SCPH=1
 *              SPI_TMOD_e tmod: Transfer Mode                              SPI_TRXD,        //Transmit & Receive(default)
 *                                                                          SPI_TXD,         //Transmit Only
 *                                                                          SPI_RXD,         //Receive Only
 *                                                                          SPI_EEPROM,      //EEPROM Read  
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/

static void spi_master_init(uint32_t baud,SPI_SCMOD_e scmod,SPI_TMOD_e tmod) 
{
	uint8_t shift = 0;
	AP_SSI_TypeDef *Ssix = AP_SPI0; 
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

/**************************************************************************************
 * @fn          hal_spi_slave_init
 *
 * @brief       This function process for spi slave initial
 *
 * input parameters
 *
 * @param       uint32_t baud: baudrate select
 *              SPI_SCMOD_e scmod: Serial Clock Polarity and Phase select;  SPI_MODE0,        //SCPOL=0,SCPH=0(default)
 *                                                                          SPI_MODE1,        //SCPOL=0,SCPH=1
 *                                                                          SPI_MODE2,        //SCPOL=1,SCPH=0
 *                                                                          SPI_MODE3,        //SCPOL=1,SCPH=1
 *              SPI_TMOD_e tmod: Transfer Mode                              SPI_TRXD,        //Transmit & Receive(default)
 *                                                                          SPI_TXD,         //Transmit Only
 *                                                                          SPI_RXD,         //Receive Only
 *                                                                          SPI_EEPROM,      //EEPROM Read  
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
/*static*/ void spi_slave_init(uint32_t baud,SPI_SCMOD_e scmod,SPI_TMOD_e tmod)
{
	uint8_t shift = 0;
	AP_SSI_TypeDef *Ssix = AP_SPI0;
	AP_COM_TypeDef *apcom = AP_COM;
	uint16_t baud_temp; 
	int pclk = clk_get_pclk();
    spi0_ssienr_set(0x00);

	apcom->PERI_MASTER_SELECT &= ~(BIT(shift)); //master select
	
	spi0_ctrlr0_set((spi0_ctrlr0_cfs_getf() & 0xfffffc3f)|(scmod<<6)|(tmod<<8)|0x400);

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
	spi0_imr_set(0x11);

	spi0_ssienr_set(1);

}



static int spi_xmit_polling(uint8_t* tx_buf,uint8_t* rx_buf,uint32_t len)
{
  uint32_t rx_size = len, tx_size = len;
  uint32_t tmp_len,i;
  AP_SSI_TypeDef *Ssix = AP_SPI0; 
  
	SPI_INIT_TOUT(to);
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
		if(spi0_rxflr_get()){
		tmp_len = spi0_rxflr_get();
		
		for(i = 0; i< tmp_len; i++){
			*rx_buf++ = spi0_dr_get();
		}
		rx_size -= tmp_len;
		}
		if(rx_size == 0)
			break;
		SPI_CHECK_TOUT(to, SPI_OP_TIMEOUT, "hal_spi_xmit_polling TO\n");
	}
	while(spi0_sr_busy_getf())
	{
		SPI_CHECK_TOUT(to, SPI_OP_TIMEOUT, "hal_spi_xmit_polling TO\n");
	}

	return PPlus_SUCCESS;
}

static void spi0_sleep_handler(void)
{
  if(m_spiCtx.spi_info != NULL)
    spi_bus_deinit();   
}


static void spi0_wakeup_handler(void)
{
  NVIC_SetPriority((IRQn_Type)SPI0_IRQn, IRQ_PRIO_HAL);
}

int spi_transmit(uint8_t* tx_buf,uint8_t* rx_buf,uint16_t len)
{
	int ret;
	spi_Ctx_t* pctx;
	AP_SSI_TypeDef *Ssix = AP_SPI0; 
	spi_xmit_t* trans_ptr;

	pctx = &m_spiCtx;
	trans_ptr = &(pctx->transmit);

	if(len == 0)
		return PPlus_ERR_INVALID_PARAM;

	if(pctx->transmit.busy == true)
		return PPlus_ERR_BUSY;
  
	if(pctx->cfg.force_cs == true && pctx->is_slave_mode == FALSE)
	{
		gpio_fmux_control(pctx->cfg.ssn_pin,Bit_DISABLE);
		gpio_write(pctx->cfg.ssn_pin,0);
	}  

  
  if(pctx->cfg.int_mode == false)
  {
    ret = spi_xmit_polling(tx_buf, rx_buf, len);
      
    if(pctx->cfg.force_cs == true  && pctx->is_slave_mode == FALSE)
		gpio_fmux_set(pctx->cfg.ssn_pin,spimap[pin_flag[1]][1]);
		if(ret)
			return PPlus_ERR_TIMEOUT;
  }
  else
  {
	spi_int_disable();
	if(trans_ptr->buf_len < len)
		return PPlus_ERR_NO_MEM;
	
	if(tx_buf){
		if(!trans_ptr->tx_buf)
		return PPlus_ERR_NO_MEM;
	}
	
	trans_ptr->tx_offset = 0;
	memcpy(trans_ptr->tx_buf,tx_buf,len);
	if(spi0_sr_tfnf_getf())
	{
		uint16_t tx_len;
		uint8_t dummy[8];

		if(rx_buf){
		trans_ptr->rx_buf = rx_buf;
	}

	tx_len = (len >= 8)?8:len;

	if(trans_ptr->tx_buf){
		trans_ptr->tx_offset += tx_len;
		spi_write_fifo(tx_len,tx_buf);
	}
	else{
		spi_write_fifo(tx_len,dummy);
	}
	
	trans_ptr->xmit_len = len;
	}
	pctx->transmit.busy = true;
	spi_int_enable(0x11);
  }
  
  return PPlus_SUCCESS;
}

int spi_set_tx_buffer(uint8_t* tx_buf,uint16_t len)
{

  if((tx_buf == NULL) || (len == 0))
    return PPlus_ERR_INVALID_PARAM;
    
  m_spiCtx.transmit.tx_buf = tx_buf;//used when tx int
  m_spiCtx.transmit.buf_len = len;

  return PPlus_SUCCESS;
}

int spi_set_int_mode(bool en)
{ 
  m_spiCtx.cfg.int_mode = en;
  
  if(en)
  {
      m_spiCtx.cfg.int_mode = true;
      spi_int_enable(0x10);
  }
  else
  {
      m_spiCtx.cfg.int_mode = false;
      spi_int_disable();
  }
  
  return PPlus_SUCCESS;
}

int spi_set_force_cs(bool en)
{

  m_spiCtx.cfg.force_cs = en;
  return PPlus_SUCCESS;
}

bool spi_get_transmit_bus_state(void)
{
  return m_spiCtx.transmit.busy;
}


int spi_TxComplete(void)
{
  AP_SSI_TypeDef *Ssix = AP_SPI0; 
  
  
  SPI_INIT_TOUT(to);
  while(spi0_sr_busy_getf())
  {
    SPI_CHECK_TOUT(to, SPI_OP_TIMEOUT, "hal_spi_TxComplete TO\n");
  }
  return PPlus_SUCCESS;
}

int spi_send_byte(uint8_t data)
{
  AP_SSI_TypeDef *Ssix = AP_SPI0; 
  
  if(spi0_sr_tfnf_getf()){
	spi0_dr_set(data & 0xff);
    SPI_INIT_TOUT(to);
	while(spi0_sr_get() & SPI_BUSY)
    {
      SPI_CHECK_TOUT(to, SPI_OP_TIMEOUT,"hal_spi_send_byte TO\n");
    }      
  }
	return PPlus_SUCCESS;
}


int spi_bus_init(spi_Cfg_t cfg)
{
  spi_Ctx_t* pctx = NULL;
  int ret;
  
  pctx = &m_spiCtx;
  
  if(pctx->spi_info != NULL)
    return PPlus_ERR_BUSY;

  clk_gate_enable((MODULE_e)MOD_SPI0);
  
  ret = spi_pin_init(cfg.sclk_pin,cfg.ssn_pin,cfg.MOSI,cfg.MISO);
	if(ret != PPlus_SUCCESS)
	{
		 clk_gate_disable((MODULE_e)MOD_SPI0);
		return PPlus_ERR_INVALID_PARAM;
	}
  spi_master_init(cfg.baudrate, cfg.spi_scmod, cfg.spi_tmod);
  
  pctx->cfg = cfg;
  pctx->transmit.busy = false;
  pctx->spi_info = SPI0;
  
  if(cfg.int_mode)
    spi_int_enable(0x10);
  else
    spi_int_disable();

  pctx->is_slave_mode = false;
  
  return PPlus_SUCCESS;
}

//#define SPI_INIT_TOUT(to) int to = hal_systick()
//#define PSI_CHECK_TOUT(to, timeout, loginfo) {if(hal_ms_intv(to) > timeout){LOG(loginfo);return PPlus_ERR_TIMEOUT;}}

int spis_clear_rx(void)
{
  AP_SSI_TypeDef *Ssix = AP_SPI0;
  volatile uint8_t rx;
  while(spi0_rxflr_get()){
	rx = spi0_dr_get();
  }
  return (int)rx;
}

uint32_t spis_rx_len(void)
{
  AP_SSI_TypeDef *Ssix = AP_SPI0; 
  return spi0_rxflr_get();
}

int spis_read_rxn(uint8_t* pbuf, uint16_t len)
{
  AP_SSI_TypeDef *Ssix = AP_SPI0; 
  
  while(len){
	*pbuf = spi0_dr_get();
    pbuf ++;
    len --;
  }
  return PPlus_SUCCESS;
}

int spis_bus_init(spi_Cfg_t cfg)
{
	int ret;
  spi_Ctx_t* pctx = NULL;
  
  pctx = &m_spiCtx;

  if(pctx->spi_info != NULL)
    return PPlus_ERR_BUSY;

  clk_gate_enable((MODULE_e)MOD_SPI0);
  
  ret = spi_pin_init(cfg.sclk_pin,cfg.ssn_pin,cfg.MOSI,cfg.MISO);
	if(ret != PPlus_SUCCESS)
	{
		 clk_gate_disable((MODULE_e)MOD_SPI0);
		return PPlus_ERR_INVALID_PARAM;
	}
  spi_slave_init(cfg.baudrate, cfg.spi_scmod, cfg.spi_tmod);
  
  pctx->cfg = cfg;

  memset(&(pctx->transmit), 0, sizeof(spi_xmit_t));
  pctx->spi_info = SPI0;
  pctx->is_slave_mode = true;
  if(cfg.int_mode)
    spi_int_enable(0x10);
  else
    spi_int_disable();
  


  
  return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          hal_spi_deinit
 *
 * @brief       This function will deinit the spi you select.
 *
 * input parameters
 *
 * @param         hal_spi_t* spi_ptr: spi module handle.

 *
 * output parameters
 *
 * @param       None.
 *
 * @return      
 *              PPlus_SUCCESS
 *              PPlus_ERR_INVALID_PARAM
 **************************************************************************************/
int spi_bus_deinit(void)
{
  clk_gate_disable((MODULE_e)MOD_SPI0);

  spi_pin_deinit(m_spiCtx.cfg.sclk_pin,m_spiCtx.cfg.ssn_pin,m_spiCtx.cfg.MOSI,m_spiCtx.cfg.MISO);
  memset(&m_spiCtx,0,sizeof(spi_Ctx_t));
  
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

  ret = pwrmgr_register(MOD_SPI0,spi0_sleep_handler, spi0_wakeup_handler);
  if(ret == PPlus_SUCCESS)
    memset(&m_spiCtx,0,sizeof(spi_Ctx_t));

  return ret;
}
