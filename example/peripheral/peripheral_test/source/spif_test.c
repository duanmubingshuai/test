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

/**************************************************************************************************
  Filename:       gpio_demo.c
  Revised:        $Date $
  Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "rom_sym_def.h"
#include "OSAL.h"
#include "gpio_demo.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"


uint8 key_TaskID;
#define START_DEVICE_EVT    1
#define KEY_DEMO_ONCE_TIMER 2
volatile int s_tmp = 1234;



//extern void spif_test(void);
#include "otp.h"
volatile uint32_t test[256];

//void (*otp_init_p)(void);
//int (*otp_read_data_p)(uint32_t read_addr, uint32_t* data_buf, uint32_t data_len,OTP_READ_Mode_e mode);
 
void get_otp_data(void)
{
	uint32_t addr,len;
	int ret;
    uint32_t err_flag=0;
	
	//otp_init_p = 0x00005eb0;
	//otp_read_data_p = 0x000060aa;

	//(*otp_init_p)();	
	_symrom_otp_init();
	LOG("read otp data start\n");

	//ret = _symrom_otp_read_data(0x1FFF8000,test,2,OTP_USER_READ_MODE);//OTP_USER_READ_MODE
	//LOG("ret:%d 0x%x 0x%x\n",ret,test[0],test[1]);
	
	//LOG("l:%d\n",__LINE__);

	err_flag=0;
	for(int sector_index = 0;sector_index < 16;sector_index++)
	{				
		addr = 0x1FFF8000 + sector_index*1024;
		len=256;
		ret = _symrom_otp_read_data(addr,test,len,OTP_USER_READ_MODE);


		//1fff80090 55555555 
		//1fff80094 aaaa_aaaa,
		//1fff80f0 aaaa_aaaa		
//		if(addr == 0x1FFF8000)
//		{
//			if((test[36] != 0x555555) || \ 
//				(test[37] != 0xaaaaaa)|| \
//				(test[60] != 0xaaaaaa))
//			{
//				err_flag = 1;
//				LOG("1fff8090:0x%x\n",test[36]);
//				LOG("1fff8094:0x%x\n",test[37]);
//				LOG("1fff80f0:0x%x\n",test[60]);
//			}		
//		}
//		else
		{
			for(int i=0;i<256;i++)
			{
				if((i%2)==0)
				{
					if(test[i] != 0x55555555)
					{
						err_flag = 1;
						break;
					}
				}
				else
				{
					if(test[i] != 0xaaaaaaaa)
					{
						err_flag = 1;
						break;
					}	
				}
			}
		}

		
		if(err_flag == 1)
		{
			LOG("addr:0x%x len:%d ret:0x%x\n",addr,len,ret);
			//LOG("l:%d\n",__LINE__);	
			for(int i=0;i<256;i++)
			{
				if((i%4)==0)
				{
					LOG("\n");
				}
				LOG("0x%x ",test[i]);
			}
			LOG("\n");
		}
	}
	
	if(err_flag == 1)
	{
		LOG("otp read error,stop\n");
		while(1);
	}
	
	LOG("read otp data end\n");
}




unsigned int flash_addr_start = 0x11000000;
unsigned int flash_add_end = 0x11080000;
//unsigned int flash_add_end = 0x11000000+16;


unsigned char data_wr_buf[64];
unsigned int data_word_wr = 0x12345678;
unsigned short data_hf_wr = 0x1234; 
unsigned char data_byte_wr = 0x55;

unsigned char data_rd_buf[64];
unsigned int data_word_rd;
unsigned char data_byte_rd;


unsigned char gpio_p0_toggle_test[32] = {
	0x66, 0x10, 0x01, 0x32, 0x41, 0xB3, 0x66, 0x10, 0x00, 0x32, 0x53, 0xB3, 0x63, 0x10, 0x00, 0x32, 
	0x40, 0xB3, 0x01, 0x32, 0x40, 0xB3, 0xFB, 0x07, 0x00, 0x80, 0x00, 0x40, 0x80, 0xF0, 0x00, 0x40
};
	
void bx_to_application(unsigned int address)
{
    __ASM volatile("ld.w r1, (%0,0)\n\t"
				"jmp r1"
				:
				:"r"(address));
}

void run_in_flash_jump(void)
{
	volatile unsigned int flash_data;
	volatile unsigned int flash_addr = 0x11000000;
	int ret;
	int i;
#if 1

	ret = _symrom_flash_chip_erase();//erase chip	
	_symrom_WriteFlash(0x11000000,0x11000004);
	LOG("addr:0x%x value:0x%x\n",0x11000000,0x11000004);
	
	flash_addr = 0x11000004;	
	for(i=0;i<(sizeof(gpio_p0_toggle_test)/sizeof(gpio_p0_toggle_test[0]));i=i+4)//copy bin to flash
	{
		flash_data = gpio_p0_toggle_test[i]|(gpio_p0_toggle_test[i+1]<<8)|(gpio_p0_toggle_test[i+2]<<16)|(gpio_p0_toggle_test[i+3]<<24);
		ret = _symrom_WriteFlash(flash_addr,flash_data);
		//LOG("addr:0x%x value:0x%x ret:%d\n",flash_addr,flash_data,ret);//ret=0=error
		//LOG(".");
		
		if(ret != PPlus_SUCCESS)
		{
			LOG("addr:0x%x value:0x%x ret:%d\n",flash_addr,flash_data,ret);//ret=0=error
			while(1);
		}
		flash_addr += 4;
		//delay_temp();
	}
	LOG("\n\n\n");
#endif		
#if 0
	for(int j=0;j<1;j++)
	{
		data_word_rd = ReadFlashWord(0x11000000);
		LOG("addr:0x%x value:0x%x\n",0x11000000,data_word_rd);
		
		flash_addr = 0x11000004;		
		for(i=0;i<(sizeof(gpio_p0_toggle_test)/sizeof(gpio_p0_toggle_test[0]));i=i+4)//copy bin to flash
		{
			i_counter = 0;
			//L1:
			flash_data = gpio_p0_toggle_test[i]|(gpio_p0_toggle_test[i+1]<<8)|(gpio_p0_toggle_test[i+2]<<16)|(gpio_p0_toggle_test[i+3]<<24);
			data_word_rd = _symrom_ReadFlashWord(flash_addr);
			//LOG("addr:0x%x 0x%x 0x%x",flash_addr,flash_data,data_word_rd);
			//LOG(".");
			
			if(flash_data != data_word_rd)
			{
				LOG("addr:0x%x 0x%x 0x%x",flash_addr,flash_data,data_word_rd);
				LOG(" *(error)\n");
				while(1);
				//if(i_counter++<10)					
					//goto L1;
			}
			else
			{
				LOG("\n");
			}
			
			flash_addr += 4;
			//delay_temp();	
		}
		LOG("\n\n\n");
	}
#endif	
	//*(volatile int*)0x4000F0CC = 0x44;
	LOG("hello");
	bx_to_application(0x11000000);
	while(1);;;;
}


void flash_test(void)
{
	uint32_t testcase = 9;
	uint32_t addr;
	uint32_t ret;
	uint8_t id[3];

	//_symrom_init_spif();
	LOG("testcase:%d\n",testcase);
	switch(testcase)
	{
			case 0://0x1364c8=512KB
			while(1)
			{
				ret = _symrom_spif_read_id(id);
				LOG("flashid:0x%x 0x%x 0x%x\n",id[0],id[1],id[2]);
				_symrom_WaitMs(1000);
			}
			break;

			case 1://_symrom_flash_chip_erase _symrom_ReadFlashWord _symrom_ReadFlash ok
			case 2://_symrom_flash_block64_erase  _symrom_ReadFlashWord _symrom_ReadFlash ok
			case 3://_symrom_flash_block32_erase _symrom_ReadFlashWord _symrom_ReadFlash ok
			case 4://_symrom_flash_sector_erase  _symrom_ReadFlashWord _symrom_ReadFlash ok
			data_word_wr = 0xffffffff;
			data_byte_wr = 0xff;
			if(testcase == 1)
			{	
				LOG("chip erase\n");
				ret = _symrom_flash_chip_erase();//erase chip
			}
			else if(testcase == 2)
			{
				LOG("block64 erase\n");
				for(addr = flash_addr_start; addr < flash_add_end;addr += 0x10000)//64KB
				{					
					_symrom_flash_block64_erase(addr);
				}
			}
			else if(testcase == 3)
			{
				LOG("block32 erase\n");
				for(addr = flash_addr_start; addr < flash_add_end;addr += 0x8000)//32KB
				{
					_symrom_flash_block32_erase(addr);
				}
				flash_addr_start = 0x11000000+32*0x1000;
			}
			else if(testcase == 4)
			{
				LOG("sector erase\n");
				for(addr = flash_addr_start; addr < flash_add_end;addr += 0x1000)//1KB
				{
					_symrom_flash_sector_erase(addr);
				}				
			}
			LOG("erase finish\n");

			
			for(addr = flash_addr_start; addr < flash_add_end;addr += 4)
			{
				data_word_rd = _symrom_ReadFlashWord(addr);
				if(data_word_wr != data_word_rd)
				{
					while(1)
					{
						LOG("addr:0x%x data(r):0x%x data(w):0x%x\n",addr,data_word_rd,data_word_wr);
						LOG("error stop:%d\n",__LINE__);
						_symrom_WaitMs(5000);
					}					
				}
			}
			LOG("erase ok(rd word)\n");
						
			for(addr = flash_addr_start; addr < flash_add_end;addr += 1)
			{
				data_byte_rd = _symrom_ReadFlash(addr);
				if(data_byte_wr != data_byte_rd)
				{
					while(1)
					{
						LOG("addr:0x%x data(r):0x%x data(w):0x%x\n",addr,data_byte_rd,data_byte_wr);
						LOG("error stop:%d\n",__LINE__);
						_symrom_WaitMs(5000);
					}
				}
			}
			LOG("erase ok(rd byte)\n");
			break;

			case 5://_symrom_WriteFlash _symrom_ReadFlashWord ok
			case 6://_symrom_ProgramPage4 _symrom_ReadFlashWord ok
			case 7://_symrom_WriteFlashShort _symrom_ReadFlashWord ok
			case 8://ProgramPage64 _symrom_ReadFlashWord ok
										
				ret = _symrom_flash_chip_erase();//erase chip
				if((testcase == 5)||(testcase == 6))
				{
					data_word_wr = 0x12345678;	
					for(addr = flash_addr_start; addr < flash_add_end;addr += 4)
					{
						if(testcase == 5)
						{
							ret = _symrom_WriteFlash(addr,data_word_wr);
						}
						else if(testcase == 6)
						{					
							ret = _symrom_ProgramPage4(addr,data_word_wr);
						}
						if(ret != PPlus_SUCCESS)
						{
							while(1)
							{								
								LOG("error stop:%d\n",__LINE__);
								_symrom_WaitMs(5000);
							}
						}

					}
				}
				else if(testcase == 7)
				{
					data_hf_wr = 0x1234;
					data_word_wr = 0x12341234;
					for(addr = flash_addr_start; addr < flash_add_end;addr += 2)
					{				
						ret = _symrom_WriteFlashShort(addr,data_hf_wr);						
						if(ret != PPlus_SUCCESS)
						{
							while(1)
							{								
								LOG("error stop:%d\n",__LINE__);
								_symrom_WaitMs(5000);
							}
						}
					}
				}
				else if(testcase == 8)
				{
					for(int i=0;i<64;i++)
					{
						data_wr_buf[i] = 0;
					}
					data_word_wr = 0x00000000;
					
					for(addr = flash_addr_start; addr < flash_add_end;addr += 64)
					{
						ret = _symrom_ProgramPage64(addr,data_wr_buf,64);
						if(ret == 0)
						{
							while(1)
							{								
								LOG("error stop:%d\n",__LINE__);
								_symrom_WaitMs(5000);
							}
						}
					}
				}
				_symrom_WaitMs(1000);
				LOG("write flash finish\n");
				
				for(addr = flash_addr_start; addr < flash_add_end;addr += 4)
				{
					data_word_rd = _symrom_ReadFlashWord(addr);
					if(data_word_wr != data_word_rd)
					{
						while(1)
						{
							LOG("addr:0x%x data(r):0x%x data(w):0x%x\n",addr,data_word_rd,data_word_wr);
							LOG("error stop:%d\n",__LINE__);
							_symrom_WaitMs(5000);
						}						
					}
				}			
				break;

			case 9:
				run_in_flash_jump();
				break;
			
			default:
				break;
	}


		
	while(1)
	{
		LOG("test finish no problem,testcase:%d:%d\n",testcase,__LINE__);
		_symrom_WaitMs(1000);
	}	
}

void Key_Demo_Init(uint8 task_id)
{
	uint8_t i = 0;
	uint8_t testcae =0;
	uint32_t n=0;
	key_TaskID = task_id;
	
	LOG("gpio key demo start...\n");
	//spif_test();

	LOG("------->otp test:%d\n",n);
	while(1)
	{
#if 0	
		switch(testcae)
		{
			case 0:			
				clk_init(SYS_CLK_DLL_48M);
				LOG_INIT();
				LOG("48M:0x%x\n",*(volatile int*)0x4000F03c & 0x07);
				break;
			
			case 1:
				clk_init(SYS_CLK_XTAL_16M);
				LOG_INIT();
				LOG("16M:0x%x\n",*(volatile int*)0x4000F03c & 0x07);
				break;

			case 2:
				clk_init(SYS_CLK_DLL_32M);
				LOG_INIT();
				LOG("dll32M:0x%x\n",*(volatile int*)0x4000F03c & 0x07);
				break;
			case 3:
				write_reg(0x4000f018,(read_reg(0x4000f018))|0x100);
				subWriteReg(0x4000f05c, 1, 1, 0);
				_symrom_WaitUs(100);
				write_reg(0x4000f05c,(read_reg(0x4000f05c))|0x2);			
				_symrom_WaitUs(1);
				clk_init(SYS_CLK_RC_32M);
				LOG_INIT();
				LOG("rc32M:0x%x\n",*(volatile int*)0x4000F03c & 0x07);
				break;
		}
		if(testcae++>=4)
		{
			testcae=0;
		}
		
		get_otp_data();
#endif		
		flash_test();
		
		_symrom_WaitMs(1000*2);
		while(1);
	}

// g_system_clk = SYS_CLK_DLL_48M;
//	get_otp_data()
//	while(1);
//	
//	osal_set_event(key_TaskID, START_DEVICE_EVT);
}
static int s_tmp_cnt;
uint16 Key_ProcessEvent( uint8 task_id, uint16 events )
{
	if(task_id != key_TaskID){
		return 0;
	}
	if( events & START_DEVICE_EVT){
        s_tmp ++;
//		LOG_INIT();
		PRINT("START_DEVICE_EVT\n");
		osal_start_timerEx(key_TaskID, KEY_DEMO_ONCE_TIMER, 50);
		return (events ^ START_DEVICE_EVT);
	}
	
	if( events & KEY_DEMO_ONCE_TIMER){		
//		LOG_INIT();
        PRINT("%d,rtc cnt %d\n", s_tmp_cnt++,rtc_get_counter());
		osal_start_timerEx( key_TaskID, KEY_DEMO_ONCE_TIMER , 500);
		return (events ^ KEY_DEMO_ONCE_TIMER);
	}
	
  // Discard unknown events
  return 0;
}

/*********************************************************************
*********************************************************************/
