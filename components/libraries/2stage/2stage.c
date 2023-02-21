
#include "rom_sym_def.h"
#include "bus_dev.h"
#include "2stage.h"
#include "error.h"
#include "global_config.h"
#include "log.h"

extern void bx_to_application(unsigned int address);


void two_stage_app_run(uint32_t entry)
{
  bx_to_application(entry);
}

void two_stage_app_uartrun(uint32_t uartrun_entry)
{
  uint32_t check_entry = pGlobal_config[UARTRUN_2S_ENTRY_CONFIG];

  if(check_entry == uartrun_entry){
    pGlobal_config[UARTRUN_2S_ENTRY_CONFIG] = 0;
    return;
  }
  pGlobal_config[UARTRUN_2S_ENTRY_CONFIG] = UARTRUN_2S_ENTRY(uartrun_entry);
  bx_to_application(0);
}

int two_stage_app_verification(uint32_t entry, uint32_t size , uint32_t crc)
{
  uint16_t crc_value = 0;
  if(crc == 0xffffffff)
    return PPlus_SUCCESS;

  crc_value = crc16(0, (const volatile void *)entry, size);
  if(crc_value == (uint16_t)crc)
    return PPlus_SUCCESS;
  return PPlus_ERR_INVALID_DATA;
}

void two_stage_print(void)
{
	//LOG_ERROR("in 2stage print\n");
	//PRINT("putc1 =  %x\n",read_reg(0x1fff03ac));
	//LOG_INIT();
	//PRINT("putc2 =  %x\n",read_reg(0x1fff03ac));
	//PRINT("in 2stage print\n");
	if(read_reg(0x1fff03ac)){
		gpio_write(GPIO_P15,1);
		gpio_write(GPIO_P15,0);
	}
	PRINT("2\n");
	gpio_write(GPIO_P14,1);
	gpio_write(GPIO_P14,0);

}
void two_stage_app_load(void)
{
  uint32_t i;
  uint32_t num = OTP_BOOT_BIN_NUM;
  uint32_t entry = 0, size = 0, crc = 0;

  //two stage uartrun
  uint32_t uartrun_entry = pGlobal_config[UARTRUN_2S_ENTRY_CONFIG];
  if((uint32_t)(uartrun_entry >> 16) == UARTRUN_2S_TAG){
    uartrun_entry = (uartrun_entry & 0xffff) | 0x1fff0000;
    if(uartrun_entry < OTP_BASE_ADDR)
      pGlobal_config[UARTRUN_2S_ENTRY_CONFIG] = uartrun_entry;
      two_stage_app_run(uartrun_entry);
      return;
  }
  pGlobal_config[UARTRUN_2S_ENTRY_CONFIG] = 0;

  for(i = 0; i < OTP_2STAGE_NUM_MAX; i++){
	char * realValue = P_OTP_2STAGE_PREAMBLE(num);
    if(memcmp(P_OTP_2STAGE_PREAMBLE(num), OTP_2STAGE_PREAMBLE,4) == 0)
    {
		if(memcmp(P_OTP_2STAGE_PREAMBLE(num+1), OTP_2STAGE_PREAMBLE,4) != 0)
		{
			entry = OTP_2STAGE_ENTRY(num);
			size  = OTP_2STAGE_SIZE(num);
			crc   = OTP_2STAGE_CRC(num);
			break;
		}
      
    }
    
    num = num + 1;
	//break;
  }

  if((entry & OTP_BASE_ADDR)== OTP_BASE_ADDR)
  {
    if(two_stage_app_verification(entry, size, crc) == PPlus_SUCCESS)
		two_stage_app_run(entry);
  }
}



