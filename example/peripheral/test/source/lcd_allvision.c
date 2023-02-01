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
/**************************************************************
    Module Name: lcd driver
    File name:   lcd_allvision.c
    Brief description:
      lcd driver
    Author:  Eagle.Lao
    Revision:V0.01
****************************************************************/
#if(CFG_DISP == DISP_OLED)

#include <stdint.h>
#include "lcd_allvision.h"
#include "i2c.h"
#include "types.h"
#include "bus_dev.h"
#include "pwrmgr.h"
#include "log.h"

#include "test_demo.h"

#define STATE_MAX 0xFF
#define STATE_MIN 0x00
#define STATE_55 0x55
#define STATE_AA 0xAA
#define START_PAGE 0xB0
#define PAGE_TOTAL 4
#define START_HIGH_BIT 0x10
#define START_LOW_BIT 0x00
#define FRAME_HIGH_ROW 0x01
#define FRAME_LOW_ROW 0x80

#if (TEST_CASE == I2C_TEST)

static void* I2CInit(void)
{
    hal_i2c_pin_init(I2C_1, P23, P24);
    return hal_i2c_init(I2C_1, I2C_CLOCK_100K);
}
int I2CDeinit(void* pi2c)
{
    int ret = hal_i2c_deinit(pi2c);
    hal_gpio_pin_init(P32,IE);
    hal_gpio_pin_init(P33,IE);
    return ret;
}
static int I2CWrite(void* pi2c, uint8 reg, uint8 val)
{
    uint8 data[2];
    data[0] = reg;
    data[1] = val;
    hal_i2c_addr_update(pi2c, 0x78/2);
    {
        HAL_ENTER_CRITICAL_SECTION();
        hal_i2c_tx_start(pi2c);
        hal_i2c_send(pi2c, data, 2);
        HAL_EXIT_CRITICAL_SECTION();
    }
    return hal_i2c_wait_tx_completed(pi2c);
}

static void WriteCmd(void* pi2c, uint8_t cmd)
{
    I2CWrite(pi2c, 0x00, cmd);
}

extern int hal_i2c_send_x(void* pi2c, uint8_t* str,uint8_t len);
static int WriteCmd_x(void* pi2c, uint8_t* buf,uint8_t len)
{
    buf[0] = 0x00;
    hal_i2c_addr_update(pi2c, 0x78/2);
    {
        HAL_ENTER_CRITICAL_SECTION();
        hal_i2c_tx_start(pi2c);
        hal_i2c_send_x(pi2c, buf, len);
        HAL_EXIT_CRITICAL_SECTION();
    }
    return hal_i2c_wait_tx_completed(pi2c);
}

static void WriteData(void* pi2c, uint8_t data)
{
    I2CWrite(pi2c, 0x40, data);
}

int lcd_bus_init(void)
{
    return 0;
}

/*
    WriteCmd(pi2c, 0xAE);
    Setup Write to 0x78(0111 1000)+ACK 0x00+ACK 0xAE+ACK Stop
    addr=0x78/2
*/
void lcd_init(void)
{
    void* pi2c = I2CInit();
    WriteCmd(pi2c, 0xAE); //display off
    WriteCmd(pi2c, 0x20);   //Set Memory Addressing Mode
    WriteCmd(pi2c, 0x10);   //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    WriteCmd(pi2c, 0xb0);   //Set Page Start Address for Page Addressing Mode,0-7
    WriteCmd(pi2c, 0xc8);   //Set COM Output Scan Direction
    WriteCmd(pi2c, 0x00); //---set low column address
    WriteCmd(pi2c, 0x10); //---set high column address
    WriteCmd(pi2c, 0x40); //--set start line address
    WriteCmd(pi2c, 0x81); //--set contrast control register
    WriteCmd(pi2c, 0xff); //���ȵ��� 0x00~0xff
    WriteCmd(pi2c, 0xa1); //--set segment re-map 0 to 127
    WriteCmd(pi2c, 0xa6); //--set normal display
    WriteCmd(pi2c, 0xa8); //--set multiplex ratio(1 to 64)
    WriteCmd(pi2c, 0x3F); //
    WriteCmd(pi2c, 0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    WriteCmd(pi2c, 0xd3); //-set display offset
    WriteCmd(pi2c, 0x00); //-not offset
    WriteCmd(pi2c, 0xd5); //--set display clock divide ratio/oscillator frequency
    WriteCmd(pi2c, 0xf0); //--set divide ratio
    WriteCmd(pi2c, 0xd9); //--set pre-charge period
    WriteCmd(pi2c, 0x22); //
    WriteCmd(pi2c, 0xda); //--set com pins hardware configuration
    WriteCmd(pi2c, 0x12);
    WriteCmd(pi2c, 0xdb); //--set vcomh
    WriteCmd(pi2c, 0x20); //0x20,0.77xVcc
    WriteCmd(pi2c, 0x8d); //--set DC-DC enable
    WriteCmd(pi2c, 0x14); //
    WriteCmd(pi2c, 0xae); //--turn on oled panel
    I2CDeinit(pi2c);
}

uint8_t lcd_init_cmd_buf[]=
{
    0xAE,
    0x20,
    0x10,
    0xb0,
    0xc8,
    0x00,
    0x10,
    0x40,
    0x81,
    0xff,
    0xa1,
    0xa6,
    0xa8,
    0x3F,
    0xa4,
    0xd3,
    0x00,
    0xd5,
    0xf0,
    0xd9,
    0x22,
    0xda,
    0x12,
    0xdb,
    0x20,
    0x8d,
    0x14,
    0xae
};

void lcd_init_x(void)
{
    void* pi2c = I2CInit();
    WriteCmd_x(pi2c, lcd_init_cmd_buf,sizeof(lcd_init_cmd_buf)/sizeof(lcd_init_cmd_buf[0]));
    I2CDeinit(pi2c);
}

void lcd_on(void)
{
    void* pi2c = I2CInit();
    WriteCmd(pi2c, 0X8D);
    WriteCmd(pi2c, 0X14);
    WriteCmd(pi2c, 0XAF);
    I2CDeinit(pi2c);
}

void lcd_off(void)
{
    void* pi2c = I2CInit();
    WriteCmd(pi2c, 0X8D);
    WriteCmd(pi2c, 0X10);
    WriteCmd(pi2c, 0XAE);
    I2CDeinit(pi2c);
}

/*
    cmd
    0x78 0x00 0xB0
    0x78 0x00 0x10
    0x78 0x00 0x01

    data
    0x78 0x40 0xFF
*/
int lcd_draw(uint8_t page_s, uint8_t x, uint8_t page_e, uint8_t width, const uint8_t* data)
{
    uint8_t page_number = page_s,column_number = 0;
    void* pi2c = I2CInit();

    for(page_number = page_s; page_number <= page_e; page_number++)
    {
        WriteCmd(pi2c, START_PAGE + page_number);//0xB0+0~7
        WriteCmd(pi2c, x & 0x0F);//low 4bit
        WriteCmd(pi2c,  0x10 | ((x&0xF0)>>4));//high 3bit

        for(column_number=0; column_number<width; column_number++)
        {
            WriteData(pi2c, data[page_number*SCN_WIDTH + x+column_number]);
        }
    }

    I2CDeinit(pi2c);
    return 0;
}

#define I2C_USE_DMA
#ifdef I2C_USE_DMA

#include "dma.h"

static DMA_CH_CFG_t dma_ch_cfg;

typedef struct
{
    void*           pi2c;
    DMA_CH_t        ch;
    DMA_Hdl_t       cb;
    volatile bool   busy;
    volatile bool   dma_init_flag;

} I2C_DMA_CTRL_t;

extern void dma_cb(DMA_CH_t ch);

I2C_DMA_CTRL_t i2c_dma_ctl =
{
    .pi2c = NULL,
    .ch = DMA_CH_0,
    .cb = dma_cb,
    .busy = 0,
    .dma_init_flag = 0
};

void dma_cb(DMA_CH_t ch)
{
    i2c_dma_ctl.busy = 0;
    //LOG("\ndma done!!!:%d\n",i2c_dma_ctl.busy);
}

static void lcd_dma_init(void)
{
    hal_dma_init();
    HAL_DMA_t ch_cfg;
    ch_cfg.dma_channel = i2c_dma_ctl.ch;
    ch_cfg.evt_handler = i2c_dma_ctl.cb;
    hal_dma_init_channel(ch_cfg);
}

void ssd1315_bus_init(void)
{
    if(i2c_dma_ctl.pi2c == NULL)
    {
        i2c_dma_ctl.pi2c = I2CInit();
    }

    if(i2c_dma_ctl.dma_init_flag == 0)//dma init
    {
        lcd_dma_init();
        i2c_dma_ctl.dma_init_flag = 1;
    }
}

void ssd1315_bus_deinit(void)
{
    if(i2c_dma_ctl.pi2c != NULL)
    {
        I2CDeinit(i2c_dma_ctl.pi2c);
        hal_dma_stop_channel(i2c_dma_ctl.ch);
        i2c_dma_ctl.pi2c = NULL;
        i2c_dma_ctl.busy = 0;
    }
}

int ssd1315_bus_data_trans(uint8_t* buf_ptr,uint8_t buf_len)
{
    int retval = PPlus_ERR_INVALID_PARAM;

    if(i2c_dma_ctl.pi2c == NULL)
    {
        return retval;
    }

    //LOG("dma demo start...\n");
    //hal_dma_stop_channel(i2c_dma_ctl.ch);
    dma_ch_cfg.transf_size = buf_len;
    dma_ch_cfg.sinc = DMA_INC_INC;
    dma_ch_cfg.src_tr_width = DMA_WIDTH_BYTE;
    dma_ch_cfg.src_msize = DMA_BSIZE_1;
    dma_ch_cfg.src_addr = (uint32_t)buf_ptr;
    dma_ch_cfg.dinc = DMA_INC_NCHG;
    dma_ch_cfg.dst_tr_width = DMA_WIDTH_BYTE;
    dma_ch_cfg.dst_msize = DMA_BSIZE_1;
    dma_ch_cfg.dst_addr = (uint32_t)(&(((AP_I2C_TypeDef*)(i2c_dma_ctl.pi2c))->IC_DATA_CMD));
    dma_ch_cfg.enable_int = true;
    hal_i2c_addr_update(i2c_dma_ctl.pi2c, 0x78/2);
    ((AP_I2C_TypeDef*)(i2c_dma_ctl.pi2c))->IC_DMA_CR = 0x02;
    retval = hal_dma_config_channel(i2c_dma_ctl.ch,&dma_ch_cfg);

    if(retval == PPlus_SUCCESS)
    {
        hal_dma_start_channel(i2c_dma_ctl.ch);
        i2c_dma_ctl.busy = 1;

        if(dma_ch_cfg.enable_int == false)
        {
            hal_dma_wait_channel_complete(i2c_dma_ctl.ch);
            LOG("dma success\n");
        }
    }
    else
    {
        LOG("[DMAC]Config channel Failed,Error code is %d\n",retval);
    }

    return retval;
}

bool ssd1315_bus_state(void)
{
    return i2c_dma_ctl.busy;
}

extern void i2c_show_pic(void);
void i2c_dma_test(void)
{
    uint8 lcd_on_cmd_buf[] = {0x00,0X8D,0X14,0XAF};
    /*
        use dma:
        1.init i2c and dma,
        2.transmit data(oled cmd),
        3.uninit i2c and dma
    */
    ssd1315_bus_init();
    ssd1315_bus_data_trans(lcd_init_cmd_buf,sizeof(lcd_init_cmd_buf)/sizeof(lcd_init_cmd_buf[0]));

    while(ssd1315_bus_state() == 1);

    ssd1315_bus_data_trans(lcd_on_cmd_buf,sizeof(lcd_on_cmd_buf)/sizeof(lcd_on_cmd_buf[0]));

    while(ssd1315_bus_state() == 1);

    ssd1315_bus_deinit();
    /*
        not use dma
        init i2c,transmit data(oled data,show a picture and clean oled),uninit i2c
    */
    i2c_show_pic();
}

#endif

#endif//I2C_TEST

#endif/*CFG_DISP==DISP_OLED*/
