#include "rom_sym_def.h"
#include "i2c.h"
#include "log.h"
void i2cs_initial(void)
{
	i2cs_pin_init(IIC_P9SCL_P10SDA);
	i2cs_init(0x7F);
}
void i2cs_test(void)
{

	LOG("i2cs_test\n");
	WaitMs(2000);
}