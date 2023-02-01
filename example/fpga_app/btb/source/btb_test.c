#include "rom_sym_def.h"
#include "i2c.h"

#include "log.h"
#include "pico_reg.h"

#define   WRITE_REG(addr,high,low,value)    {write_reg(addr,(read_reg(addr)&\
                                              (~((((unsigned int)1<<((high)-(low)+1))-1)<<(low))))|\
                                                  ((unsigned int)(value)<<(low)));}
												  
void reg_test_function(uint32_t re_address,char* reg_name)
{
	static uint32_t no_nop_num,nop_num;
	
	_PICO_REG_WR(re_address,0x0);
	_PICO_REG_WR(re_address,0xf);
	nop_num = _PICO_REG_RD(re_address);
	
	
	_PICO_REG_WR(re_address,0x0);  //clear
	_PICO_REG_WR(re_address,0xf);
	no_nop_num = _PICO_REG_RD(re_address);
	
	
	if (no_nop_num != nop_num)
	{
		LOG(" need_nop ");
	}
	else
	{
		LOG(" no_need_nop ");
	}
	
	LOG("name = %s address = 0x%x no_nop = 0x%x nop =  0x%x\n",reg_name,re_address,no_nop_num,nop_num);
	
}