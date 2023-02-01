#include "rom_sym_def.h"
#include "i2c.h"

#include "log.h"


void i2c_test2(void)
{
	uint8_t data[8] = {0x01,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	uint16_t reg_adress = 0;
	int ret = PPlus_SUCCESS;
	

	for (uint32_t i = 30;i < 64;i++)
	{
		LOG("%dth write\n",i);

		eeprom_page_write(0x50,reg_adress+i*8,data,8);    //Write eight bytes at a time

		LOG("\n");
		WaitMs(500);
	}
	
	for (uint32_t i = 30;i <64;i++)
	{
		LOG("%dth read \n",i);

		ret = eeprom_random_read(0x50,reg_adress+i*8,data,8);   //Read eight bytes at a time
		
		if(ret == PPlus_SUCCESS)
		{
			for(uint8_t j = 0;j < 8;j++)
			{
				LOG("i=%d,j=%d,data=%x ",i,j,*(data+j));  
			}
		}
		LOG("\n");
		WaitMs(500);
	}
}