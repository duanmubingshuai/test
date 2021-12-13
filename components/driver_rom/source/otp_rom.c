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
#include "otp.h"
#include "clock.h"
#include <string.h>
#include "error.h"
//#include "log.h"

#define DELAY_COUNT 10000
volatile uint32_t otp_internal_delay;
volatile uint32_t otp_mode_switch_timeout_delay = DELAY_COUNT;

#define OTP_NORMAL_START	0x1FFF8000
#define OTP_NORMAL_END		0x1FFFC000
#define OTP_NORMAL_SIZE     (16*1024)
#define OTP_DEFAULT_VALUE   0xFFFFFFFF

void otp_init(void)
{
	AP_OTP->power_timing_control = OTP_TVDDH(2)   |\
									OTP_TPENH(2)  |\
									OTP_TPLH(2)   |\
									OTP_TASH(15)  |\
									OTP_TSAS(4)   |\
									OTP_TPLS(15)  |\
									OTP_TPENS(3)  |\
									OTP_TVDS(3);
  
	AP_OTP->setting1 = OTP_TPWI(3)  |\
						OTP_TPW(15) |\
						OTP_TPPS(8) |\
						OTP_TCPS(15)|\
						OTP_TMS(3);
	
	AP_OTP->setting2 =  OTP_TMH(2)   |\
						 OTP_TPPR(15)|\
						 OTP_TPPH(8);

	AP_OTP->test_ctrl = OTP_TCS(1);	

	AP_OTP->int_clr = 0xffffffff; ////clear all interrupt
	AP_OTP->int_mask = 0;//mask all interrupt,note!!! 0 is mask, 1 for enable interrupt

	NVIC_DisableIRQ((IRQn_Type)OTP_IRQn); //disable interrupt
}

void otp_delay_nop(void)
{
	volatile uint32_t i = 16;
	while(i--)
	{
		__ASM volatile("nop");
	}
}

uint8_t get_otp_state(void)
{
	return (AP_OTP->state & 0x1F);
}

bool otp_go_standby(void)
{
	otp_internal_delay = otp_mode_switch_timeout_delay;
	AP_OTP->otp_mode_setting = 1;
	otp_delay_nop();
	while((get_otp_state() != OTP_W_S_STANDBY) && otp_internal_delay--)
	{
		otp_delay_nop();
	}
		
	if(otp_internal_delay > 0)
	{
		return TRUE;
	}
	return FALSE;	
}

bool otp_go_test(uint8_t test_mode)
{	
	uint32_t temp;
	uint32_t next_mode;
	
	switch(test_mode)
	{
		case OTP_W_S_TEST_MODE0:
			temp = (BIT(4)|0x01);
			next_mode = OTP_W_S_TEST_MODE0;
			break;	
		case OTP_W_S_TEST_MODE1:
			temp = (BIT(6)|0x01);
			next_mode = OTP_W_S_TEST_MODE1;
			break;
		case OTP_W_S_TEST_MODE2:
			temp = ((9ul<<4)|1);
			next_mode = OTP_W_S_TEST_MODE2;
			break;
		case OTP_W_S_TEST_MODE3:
			temp = ((12ul<<4)|1);
			next_mode = OTP_W_S_TEST_MODE3;
			break;
		default:
			return FALSE;	
	}
		
	otp_internal_delay = otp_mode_switch_timeout_delay;
	AP_OTP->otp_mode_setting = temp;	
	otp_delay_nop();
	while((get_otp_state() != next_mode) && otp_internal_delay--)
	{
		otp_delay_nop();
	}
	
	if(otp_internal_delay > 0)
	{
		return TRUE;
	}
	
	return FALSE;	
}

bool otp_go_read(void)
{	
	otp_internal_delay = otp_mode_switch_timeout_delay;
	AP_OTP->otp_mode_setting = (BIT(1)|0x01);
	otp_delay_nop();
	while((get_otp_state() != OTP_W_S_READ) && otp_internal_delay--)
	{
		otp_delay_nop();
	}

	if(otp_internal_delay > 0)
	{
		return TRUE;
	}
	
	return FALSE;	
}

bool otp_go_prog(void)
{	
	otp_internal_delay = otp_mode_switch_timeout_delay;
	AP_OTP->otp_mode_setting =  (BIT(3)|0x01);
	otp_delay_nop();
	while((get_otp_state() != OTP_W_S_PROG_ACCESS) && otp_internal_delay--)
	{
		otp_delay_nop();
	}
	
	if(otp_internal_delay > 0)
	{
		return TRUE;
	}	
	return FALSE;	
}

bool otp_go_deepsleep(void)
{
	otp_internal_delay = otp_mode_switch_timeout_delay;
	AP_OTP->otp_mode_setting = (BIT(2)|1);	
	otp_delay_nop();
	while((get_otp_state() != OTP_W_S_DEEPLEEP_IN) && otp_internal_delay--)
	{
		otp_delay_nop();
	}
	
	if(otp_internal_delay > 0)
	{
		return TRUE;
	}
	
	return FALSE;	
}

bool otp_go_powerdown(void)
{	
	otp_internal_delay = otp_mode_switch_timeout_delay;
	AP_OTP->otp_mode_setting = 0;	
	otp_delay_nop();
	while((get_otp_state() != OTP_W_S_IDLE) && otp_internal_delay--)
	{
		otp_delay_nop();
	}
	
	if(otp_internal_delay > 0)
	{
		return TRUE;
	}
	
	return FALSE;	
}

static int otp_read_mode_switch(OTP_READ_Mode_e mode)
{	
	if(mode == OTP_USER_READ_MODE)
	{
		if(FALSE == otp_go_read())
		{
			return PPlus_ERR_TIMEOUT;
		}		
	}
	else if(mode <=  OTP_PGM_MARGIN_READ_HIGH_TEMP)
	{
		if(FALSE == otp_go_test(mode + OTP_W_S_TEST_MODE0 - OTP_INIT_MARGIN_READ_ROOM_TEMP))
		{
			return PPlus_ERR_TIMEOUT;
		}
	}
	else
	{		
		return PPlus_ERR_INVALID_PARAM;
	}	
	return PPlus_SUCCESS;
}

int otp_read_data(uint32_t read_addr, uint32_t* data_buf, uint32_t data_len,OTP_READ_Mode_e mode)
{
	int ret;
	
	if((read_addr < OTP_NORMAL_START) || (read_addr >= OTP_NORMAL_END) || (read_addr & 0x03) || (data_buf == NULL) || (data_len > 4096))
	{
		return PPlus_ERR_INVALID_PARAM;
	}
		
	if((OTP_NORMAL_END - read_addr) < (data_len * 4))
	{
		return PPlus_ERR_INVALID_PARAM;
	}
		
	
	if(mode == OTP_NO_READ_CHECK)
	{
		return PPlus_ERR_INVALID_PARAM;
	}
	
	otp_control_software_mode(FALSE);

	ret = otp_read_mode_switch(mode);
	if(ret != PPlus_SUCCESS)
	{
		return ret;
	}
	
	otp_delay_nop();
	memcpy(data_buf, (uint32_t*)read_addr, (sizeof(uint32_t)*data_len));
	
	return PPlus_SUCCESS;
}

int otp_read_data_byte(uint32_t read_addr, uint8_t* data_buf, uint32_t data_len,OTP_READ_Mode_e mode)
{
	int ret;
	
	if((read_addr < OTP_NORMAL_START) || (read_addr >= OTP_NORMAL_END) || (data_buf == NULL) || (data_len > 4096*4) || (mode == OTP_NO_READ_CHECK))
	{
		return PPlus_ERR_INVALID_PARAM;
	}
	
	if((OTP_NORMAL_END - read_addr) < data_len)
	{
		return PPlus_ERR_INVALID_PARAM;
	}

	if(mode == OTP_NO_READ_CHECK)
	{
		return PPlus_ERR_INVALID_PARAM;
	}
	
	otp_control_software_mode(FALSE);

	
	ret = otp_read_mode_switch(mode);
	if(ret != PPlus_SUCCESS)
	{
		return ret;
	}
	
	otp_delay_nop();//16 NOP  16 32 48
	memcpy(data_buf, (uint8_t*)read_addr, (sizeof(uint8_t)*data_len));

	return PPlus_SUCCESS;
}

int otp_prog_data_polling(uint32_t prog_addr, uint32_t* data_buf, uint32_t data_len,OTP_READ_Mode_e mode)
{
	uint32_t addr,i;
	int ret;
	
	if((prog_addr < OTP_NORMAL_START) || (prog_addr >= OTP_NORMAL_END) || (prog_addr & 0x03) || (data_buf == NULL) || (data_len > 4096))
	{
		return PPlus_ERR_INVALID_PARAM;
	}
	
	if((OTP_NORMAL_END - prog_addr) < (data_len * 4))
	{
		return PPlus_ERR_INVALID_PARAM;
	}
	
	
	otp_control_software_mode(FALSE);
	AP_OTP->int_mask = 0x00;
	AP_OTP->int_clr = AP_OTP->int_source;
	
	AP_OTP->prog_num = 1;
	addr = prog_addr;

  //goto deep sleep mode before program
  otp_go_deepsleep();
	
	for(i = 0;i < data_len;i++)
	{		
		AP_OTP->prog_addr = addr;
		AP_OTP->prog_data = *(data_buf+i);
				
		if(FALSE == otp_go_prog())
		{
			return PPlus_ERR_TIMEOUT;
		}	
		while(get_otp_state() != OTP_W_S_STANDBY){
		  ;
		}

		addr += 4;
	}
	
	if(mode == OTP_NO_READ_CHECK)
	{
		return PPlus_SUCCESS;
	}
	
	ret = otp_read_mode_switch(mode);
	if(ret != PPlus_SUCCESS)
	{
		return ret;
	}
			
	otp_delay_nop();//16 NOP
	addr = prog_addr;
	for(i = 0; i < data_len;i++)
	{
		if(*(volatile int*)addr != *(data_buf+i))
		{
			return PPlus_ERR_INTERNAL;
		}
		addr+=4;
	}

	return PPlus_SUCCESS;
}


void otp_control_software_mode(bool flag)
{
	if(flag == TRUE)
	{
		OTP_SOFTWARE_CONTROL;
	}
	else
	{
		OTP_HARDWARE_CONTROL;//default mode
	}
}

int otp_default_value_check(uint32_t default_value,OTP_READ_Mode_e mode)
{
	int ret;
	uint32_t otp_i = 0;
	
	if(mode == OTP_NO_READ_CHECK)
	{
		return PPlus_ERR_INVALID_PARAM;
	}
	
	otp_control_software_mode(FALSE);
	ret = otp_read_mode_switch(mode);
	if(ret != PPlus_SUCCESS)
	{
		return ret;
	}
	
	//WaitRTCCount(1);//
	otp_delay_nop();//16 NOP

	for(otp_i=0;otp_i<4096;otp_i++)
	{
		if(default_value != *(volatile int*)(OTP_NORMAL_START+(otp_i<<2)))
		{
			return PPlus_ERR_FATAL;
		}
	}
	
	return PPlus_SUCCESS;
}

//simulation
uint16_t otp_ate_test_read(uint32_t default_val,uint8_t main_read_mode)
{
	int ret;
	ret = otp_default_value_check(default_val,main_read_mode);
	return ret;
}

uint16_t otp_ate_test_row_prog(uint16_t prog_num,uint32_t prog_data)
{
	uint16_t otp_result = 0;
	int ret;
	
	OTP_TEST_ROW_ENABLE;
	
	for(uint8_t i=0;i<16;i++){
		if(prog_num & BIT(i)){
			ret = otp_prog_data_polling(OTP_BASE_ADDR +i*4,(uint32_t*)(&prog_data), 1, OTP_NO_READ_CHECK);
			if(ret == 0){
				otp_result |= BIT(i);
			}
		}			
	}
	otp_go_powerdown();
	otp_go_standby();
	OTP_TEST_ROW_DISABLE;
	return otp_result;
}

uint16_t otp_ate_test_row_read(uint16_t read_num,uint32_t read_data,uint8_t read_mode)
{
	uint16_t otp_result = 0;
	uint32_t temp;
	int ret;
	
	OTP_TEST_ROW_ENABLE;

	for(uint8_t i=0;i<16;i++){						     		
		if(read_num & BIT(i)){
			ret = otp_read_data(OTP_BASE_ADDR +i*4,(uint32_t*)(&temp),1,read_mode);
				if((PPlus_SUCCESS == ret) && (temp == read_data)){
					otp_result |= BIT(i);
			}
		}
	}	
	otp_go_powerdown();
	otp_go_standby();
	OTP_TEST_ROW_DISABLE;	
	return otp_result;
}
