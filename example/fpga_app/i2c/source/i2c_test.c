#include "rom_sym_def.h"
#include "i2c.h"
#include "lcd_allvision.h"
#include "log.h"

typedef struct{
	int16_t x;
	int16_t y;
	int16_t w;
	int16_t h;
}disp_rect_t;

void lcd_update_rect(disp_rect_t rect, const uint8_t* data)
{
	uint16_t page_s = rect.y/8, page_e = (rect.y+rect.h-1)/8;
	lcd_draw(page_s, rect.x, page_e, rect.w, data);
}

disp_rect_t rect;
#define SCN_WIDTH	  		128
#define SCN_HEIGHT			64
#define FRAME_BUF_SIZE		(SCN_WIDTH*SCN_HEIGHT/8)

#define DISP_RECT_FULL(rect) 	{	rect.x = 0;\
									rect.y = 0;\
									rect.w = SCN_WIDTH;\
									rect.h = SCN_HEIGHT;}
									
									
#define DISP_RECT_FULL(rect)    {   rect.x = 0;\
        rect.y = 0;\
        rect.w = SCN_WIDTH;\
        rect.h = SCN_HEIGHT;}
uint8_t fb[FRAME_BUF_SIZE];
								
void i2c_test(void)
{
	int i;
	uint8_t testcase = 0;
	uint8_t fb[FRAME_BUF_SIZE];	
			
	switch(testcase)
	{
		case 0:			
			lcd_init();
			lcd_on();
			DISP_RECT_FULL(rect);
			while(1)
			{
				for(i=0;i<FRAME_BUF_SIZE;i++)
					fb[i]= 0xFF;
				lcd_update_rect(rect, fb);
				WaitMs(1000);

				for(i=0;i<FRAME_BUF_SIZE;i++)
					fb[i]= 0x01;
				lcd_update_rect(rect, fb);	
				WaitMs(1000);
				
				for(i=0;i<FRAME_BUF_SIZE;i++)
					fb[i]= 0xFE;
				lcd_update_rect(rect, fb);	
				WaitMs(1000);
				
				for(i=0;i<FRAME_BUF_SIZE;i++)
					fb[i]= 0x0F;
				lcd_update_rect(rect, fb);	
				WaitMs(1000);
				
				for(i=0;i<FRAME_BUF_SIZE;i++)
					fb[i]= 0xF0;
				lcd_update_rect(rect, fb);
				WaitMs(1000);
				
				for(i=0;i<FRAME_BUF_SIZE;i++)
					fb[i]= 0x00;
				lcd_update_rect(rect, fb);
				WaitMs(1000);
			}
			break;

		default:
			break;
	}	
}
void i2c_test2(void)
{
	uint8_t abb[3] = {0x11,0x22,0x33};


	i2c_init(I2C_CLOCK_400K);
	i2c_pin_init(IIC_P18SCL_P19SDA);
	for (uint32_t i = 0;i < 256;i++)
	{
		LOG_DEBUG("send%d\n",i);
		LOG("addr = 0x%x\n",*(uint32_t*)0x40005004);
		i2c_read(0x78,0x55,abb,3);
		WaitMs(200);
	}

}