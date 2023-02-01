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

//typedef enum EEPROM_MOD{
//	K24C02_256BYTE = 0,
//	K24C04_512BYTE = 1,
//	K24C08_1024BYTE = 2,
//	K24C016_2048BYTE = 3
//};
int eeprom_init(void)
{
	i2c_init(I2C_CLOCK_400K);
	
	i2c_pin_init(IIC_P9SCL_P10SDA);   //there is no P19 and P15 is vBat
	
	return PPlus_SUCCESS;
}
//uint8_t* eeprom_current_read(uint8_t address)
//{
//	uint8_t *data_buf = 0x00;
//	
//	int ret = i2c_read(address,NULL,data_buf,3); //need modify
//	
//	if (ret == PPlus_SUCCESS)
//		return data_buf;
//}

int eeprom_random_read(
	uint16_t device_address,
	uint16_t word_address,
	uint8_t *data_buf,
	uint16_t size)
{
	uint16_t cnt,error_cnt = 0;
	int ret = PPlus_SUCCESS;
	
	cnt = size > 8 ? 8 : size;
	
	while(size)
	{
		LOG("address = 0x%x\n",device_address|(word_address>>8));
		ret = i2c_read(device_address|(word_address>>8),word_address,data_buf,size);

		if (ret == PPlus_SUCCESS)
		{
			error_cnt = 0;       //clear error_cnt
			
			size -= cnt;
			
			for(uint16_t i = cnt;i > 0;i--)
			{
				word_address++;
				data_buf++;
			}	
		}
		else if(error_cnt > 5)
		{
			LOG("five times read failed\n");
			return PPlus_ERR_IO_FAIL; 
		}
		else
		{
			error_cnt++;
		}
		WaitUs(10);

	}
	return ret;
}


int eeprom_byte_write(
	uint8_t word_address,
	uint8_t device_address,
	uint8_t *data)
{
	
	int ret = i2c_write(device_address|(word_address>>8),word_address,data,1);
	
	return ret;
}

int eeprom_page_write(
	uint16_t device_address,
	uint16_t word_address,
	uint8_t *data,
	uint16_t size)
{
	uint16_t cnt = 0,error_cnt = 0;
	int ret = PPlus_SUCCESS;
	
	
	while(size>0)
	{	
		cnt = size > 8 ? 8 : size;

		ret = i2c_write(device_address|(word_address>>8),word_address,data,size);

		if (ret == PPlus_SUCCESS)
		{
			error_cnt = 0;       //clear error_cnt
			
			size -= cnt;
			
			for(uint16_t i = cnt;i > 0;i--)
			{
				word_address++;
				data++;
			}	
		}
		else if(error_cnt > 5)
		{	
			LOG("five times receive failed\n");
			return PPlus_ERR_IO_FAIL; 
		}
		else
		{
			error_cnt++;
		}
		WaitUs(10);
	}
	

	return ret;
}