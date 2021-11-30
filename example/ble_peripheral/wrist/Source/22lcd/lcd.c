//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//����Ӳ������Ƭ��STM32F103RCT6,����ԭ��MiniSTM32������,��Ƶ72MHZ������12MHZ
//QDtech-TFTҺ������ for STM32 IOģ��
//xiao��@ShenZhen QDtech co.,LTD
//��˾��վ:www.qdtft.com
//�Ա���վ��http://qdtech.taobao.com
//wiki������վ��http://www.lcdwiki.com
//��˾�ṩ����֧�֣��κμ������⻶ӭ��ʱ����ѧϰ
//�̻�(����) :+86 0755-23594567
//�ֻ�:15989313508���빤��
//����:lcdwiki01@gmail.com    support@lcdwiki.com    goodtft@163.com
//����֧��QQ:3002773612  3002778157
//��������QQȺ:324828016
//��������:2018/08/09
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������ȫ�����Ӽ������޹�˾ 2018-2028
//All rights reserved
/****************************************************************************************************
    //=========================================��Դ����================================================//
    //     LCDģ��                STM32��Ƭ��
    //      VCC          ��        DC5V/3.3V      //��Դ
    //      GND          ��          GND          //��Դ��
    //=======================================Һ���������߽���==========================================//
    //��ģ��Ĭ��������������ΪSPI����
    //     LCDģ��                STM32��Ƭ��
    //    SDI(MOSI)      ��          PB15         //Һ����SPI��������д�ź�
    //    SDO(MISO)      ��          PB14         //Һ����SPI�������ݶ��źţ��������Ҫ�������Բ�����
    //=======================================Һ���������߽���==========================================//
    //     LCDģ��                        STM32��Ƭ��
    //       LED         ��          PB9          //Һ������������źţ��������Ҫ���ƣ���5V��3.3V
    //       SCK         ��          PB13         //Һ����SPI����ʱ���ź�
    //      DC/RS        ��          PB10         //Һ��������/��������ź�
    //       RST         ��          PB12         //Һ������λ�����ź�
    //       CS          ��          PB11         //Һ����Ƭѡ�����ź�
    //=========================================������������=========================================//
    //���ģ�鲻���������ܻ��ߴ��д������ܣ����ǲ���Ҫ�������ܣ�����Ҫ���д���������
    //     LCDģ��                STM32��Ƭ��
    //      T_IRQ        ��          PC10         //�����������ж��ź�
    //      T_DO         ��          PC2          //������SPI���߶��ź�
    //      T_DIN        ��          PC3          //������SPI����д�ź�
    //      T_CS         ��          PC13         //������Ƭѡ�����ź�
    //      T_CLK        ��          PC0          //������SPI����ʱ���ź�
**************************************************************************************************/
/*  @attention

    THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
    WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
    TIME. AS A RESULT, QD electronic SHALL NOT BE HELD LIABLE FOR ANY
    DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
    FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
    CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
**************************************************************************************************/
#include "lcd.h"
#include "stdlib.h"
#include "spi.h"
#include "log.h"
#if DMAC_USE
    #include "dma.h"
#endif


//����LCD��Ҫ����
//Ĭ��Ϊ����

hal_spi_t lcd_spi =
{
    .spi_index = SPI0,
};

spi_Cfg_t lcd_spi_cfg =
{
    .sclk_pin = GPIO_P33,
    .ssn_pin = GPIO_P25,
    .MOSI = GPIO_P32,
    .MISO = GPIO_DUMMY,

    .baudrate = 32000000,
    .spi_tmod = SPI_TRXD,
    .spi_scmod = SPI_MODE0,
    .spi_dfsmod = SPI_1BYTE,

    #if DMAC_USE
    .dma_tx_enable = false,
    .dma_rx_enable = false,
    #endif

    .int_mode = false,
    .force_cs = true,
    .evt_handler = NULL,
};

#if DMAC_USE
HAL_DMA_t lcd_dma_cfg =
{
    .dma_channel = DMA_CH_0,
    .evt_handler = NULL,
};

#endif

_lcd_dev lcddev;

//������ɫ,������ɫ
u16 POINT_COLOR = 0x0000,BACK_COLOR = 0xFFFF;
u16 DeviceCode;

/*****************************************************************************
    @name       :void LCD_WR_REG(u8 data)
    @date       :2018-08-09
    @function   :Write an 8-bit command to the LCD screen
    @parameters :data:Command value to be written
    @retvalue   :None
******************************************************************************/
void LCD_WR_REG(u8 data)
{
    uint8_t buf_send[2];
    LCD_RS_CLR;
    buf_send[0] = data;
    hal_spi_transmit(&lcd_spi,SPI_TXD,buf_send,NULL,1,0);
}

/*****************************************************************************
    @name       :void LCD_WR_DATA(u8 data)
    @date       :2018-08-09
    @function   :Write an 8-bit data to the LCD screen
    @parameters :data:data value to be written
    @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(u8 data)
{
    uint8_t buf_send[2];
    LCD_RS_SET;
    buf_send[0] = data;
    hal_spi_transmit(&lcd_spi,SPI_TXD,buf_send,NULL,1,0);
}

/*****************************************************************************
    @name       :void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
    @date       :2018-08-09
    @function   :Write data into registers
    @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
    @retvalue   :None
******************************************************************************/
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue)
{
    LCD_WR_REG(LCD_Reg);
    LCD_WR_DATA(LCD_RegValue);
}

/*****************************************************************************
    @name       :void LCD_WriteRAM_Prepare(void)
    @date       :2018-08-09
    @function   :Write GRAM
    @parameters :None
    @retvalue   :None
******************************************************************************/
void LCD_WriteRAM_Prepare(void)
{
    LCD_WR_REG(lcddev.wramcmd);
}

void Lcd_WriteData_Continuous(u16 Data,u32 Len)
{
    uint8_t buf_send[2];
    u32 tx_len=Len;
    u32 tmp_len;
    hal_spi_dfs_set(&lcd_spi,SPI_2BYTE);
    LCD_RS_SET;
    buf_send[0] = Data&0xff;
    buf_send[1] = Data>>8;
    #if DMAC_USE
    hal_spi_dma_set(&lcd_spi,1,0);
    u16 max_block_len = DMA_GET_MAX_TRANSPORT_SIZE(lcd_dma_cfg.dma_channel);

//    LOG("%08x %08x\n",max_block_len,tx_len);
    while(1)
    {
        tmp_len = (tx_len>max_block_len) ? max_block_len : tx_len;
        hal_spi_transmit(&lcd_spi,SPI_TXD,buf_send,NULL,tmp_len,0);
        tx_len -= tmp_len;

        if(tx_len == 0)
            break;
    }

    #endif
    hal_spi_dfs_set(&lcd_spi,SPI_1BYTE);
}


/*****************************************************************************
    @name       :void Lcd_WriteData_16Bit(u16 Data)
    @date       :2018-08-09
    @function   :Write an 16-bit command to the LCD screen
    @parameters :Data:Data to be written
    @retvalue   :None
******************************************************************************/
void Lcd_WriteData_16Bit(u16 Data)
{
    uint8_t buf_send[2];
    hal_spi_dfs_set(&lcd_spi,SPI_2BYTE);
    LCD_RS_SET;
    buf_send[0] = Data&0xff;
    buf_send[1] = Data>>8;
    hal_spi_transmit(&lcd_spi,SPI_TXD,buf_send,NULL,1,0);
    hal_spi_dfs_set(&lcd_spi,SPI_1BYTE);
}

/*****************************************************************************
    @name       :void LCD_DrawPoint(u16 x,u16 y)
    @date       :2018-08-09
    @function   :Write a pixel data at a specified location
    @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
    @retvalue   :None
******************************************************************************/
void LCD_DrawPoint(u16 x,u16 y)
{
    LCD_SetCursor(x,y);//���ù��λ��
    Lcd_WriteData_16Bit(POINT_COLOR);
}

/*****************************************************************************
    @name       :void LCD_Clear(u16 Color)
    @date       :2018-08-09
    @function   :Full screen filled LCD screen
    @parameters :color:Filled color
    @retvalue   :None
******************************************************************************/
void LCD_Clear(u16 Color)
{
    LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
    LCD_RS_SET;
//    LOG("%08x %08x",lcddev.width,lcddev.height);
    Lcd_WriteData_Continuous(Color,lcddev.height * lcddev.width);
//    LOG("%08x %08x",lcddev.width,lcddev.height);
//  for(i=0;i<lcddev.height;i++)
//  {
//        for(m=0;m<lcddev.width;m++)
//        {
//            Lcd_WriteData_16Bit(Color);
//      }
//  }
}

/*****************************************************************************
    @name       :void LCD_RESET(void)
    @date       :2018-08-09
    @function   :Reset LCD screen
    @parameters :None
    @retvalue   :None
******************************************************************************/
void LCD_RESET(void)
{
    LCD_RST_CLR;
    WaitMs(100);
    LCD_RST_SET;
    WaitMs(50);
}

/*****************************************************************************
    @name       :void LCD_RESET(void)
    @date       :2018-08-09
    @function   :Initialization LCD screen
    @parameters :None
    @retvalue   :None
******************************************************************************/
void LCD_Init(void)
{
    uint8_t retval;
    retval = hal_spi_bus_init(&lcd_spi,lcd_spi_cfg);

    if(retval != PPlus_SUCCESS)
    {
        LOG("spi init err!please check it!\n");
    } //Ӳ��SPI2��ʼ��

    LCD_RESET(); //LCD ��λ
    #if DMAC_USE
    hal_dma_init();
    hal_dma_stop_channel(lcd_dma_cfg.dma_channel);
    hal_dma_init_channel(lcd_dma_cfg);
    #endif
//*************2.2inch ILI9341��ʼ��**********//
    LCD_WR_REG(0xCF);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0xD9); //C1
    LCD_WR_DATA(0X30);
    LCD_WR_REG(0xED);
    LCD_WR_DATA(0x64);
    LCD_WR_DATA(0x03);
    LCD_WR_DATA(0X12);
    LCD_WR_DATA(0X81);
    LCD_WR_REG(0xE8);
    LCD_WR_DATA(0x85);
    LCD_WR_DATA(0x10);
    LCD_WR_DATA(0x7A);
    LCD_WR_REG(0xCB);
    LCD_WR_DATA(0x39);
    LCD_WR_DATA(0x2C);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x34);
    LCD_WR_DATA(0x02);
    LCD_WR_REG(0xF7);
    LCD_WR_DATA(0x20);
    LCD_WR_REG(0xEA);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0xC0);    //Power control
    LCD_WR_DATA(0x21);   //VRH[5:0]  //1B
    LCD_WR_REG(0xC1);    //Power control
    LCD_WR_DATA(0x12);   //SAP[2:0];BT[3:0] //01
    LCD_WR_REG(0xC5);    //VCM control
    LCD_WR_DATA(0x39);   //3F
    LCD_WR_DATA(0x37);   //3C
    LCD_WR_REG(0xC7);    //VCM control2
    LCD_WR_DATA(0XAB);   //B0
    LCD_WR_REG(0x36);    // Memory Access Control
    LCD_WR_DATA(0x48);
    LCD_WR_REG(0x3A);
    LCD_WR_DATA(0x55);
    LCD_WR_REG(0xB1);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x1B);  //1A
    LCD_WR_REG(0xB6);    // Display Function Control
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0xA2);
    LCD_WR_REG(0xF2);    // 3Gamma Function Disable
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0x26);    //Gamma curve selected
    LCD_WR_DATA(0x01);
    LCD_WR_REG(0xE0); //Set Gamma
    LCD_WR_DATA(0x0F);
    LCD_WR_DATA(0x23);
    LCD_WR_DATA(0x1F);
    LCD_WR_DATA(0x0B);
    LCD_WR_DATA(0x0E);
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x4B);
    LCD_WR_DATA(0XA8);
    LCD_WR_DATA(0x3B);
    LCD_WR_DATA(0x0A);
    LCD_WR_DATA(0x14);
    LCD_WR_DATA(0x06);
    LCD_WR_DATA(0x10);
    LCD_WR_DATA(0x09);
    LCD_WR_DATA(0x00);
    LCD_WR_REG(0XE1); //Set Gamma
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x1C);
    LCD_WR_DATA(0x20);
    LCD_WR_DATA(0x04);
    LCD_WR_DATA(0x10);
    LCD_WR_DATA(0x08);
    LCD_WR_DATA(0x34);
    LCD_WR_DATA(0x47);
    LCD_WR_DATA(0x44);
    LCD_WR_DATA(0x05);
    LCD_WR_DATA(0x0B);
    LCD_WR_DATA(0x09);
    LCD_WR_DATA(0x2F);
    LCD_WR_DATA(0x36);
    LCD_WR_DATA(0x0F);
    LCD_WR_REG(0x2B);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x01);
    LCD_WR_DATA(0x3f);
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0x00);
    LCD_WR_DATA(0xef);
    LCD_WR_REG(0x11); //Exit Sleep
    WaitMs(120);
    LCD_WR_REG(0x29); //display on
    LCD_direction(USE_HORIZONTAL);//����LCD��ʾ����
    LCD_LED;//��������
//  LCD_Clear(WHITE);//��ȫ����ɫ
}

/*****************************************************************************
    @name       :void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
    @date       :2018-08-09
    @function   :Setting LCD display window
    @parameters :xStar:the bebinning x coordinate of the LCD display window
                                yStar:the bebinning y coordinate of the LCD display window
                                xEnd:the endning x coordinate of the LCD display window
                                yEnd:the endning y coordinate of the LCD display window
    @retvalue   :None
******************************************************************************/
void LCD_SetWindows(u16 xStar, u16 yStar,u16 xEnd,u16 yEnd)
{
    LCD_WR_REG(lcddev.setxcmd);
    LCD_WR_DATA(xStar>>8);
    LCD_WR_DATA(0x00FF&xStar);
    LCD_WR_DATA(xEnd>>8);
    LCD_WR_DATA(0x00FF&xEnd);
    LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA(yStar>>8);
    LCD_WR_DATA(0x00FF&yStar);
    LCD_WR_DATA(yEnd>>8);
    LCD_WR_DATA(0x00FF&yEnd);
    LCD_WriteRAM_Prepare(); //��ʼд��GRAM
}

/*****************************************************************************
    @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
    @date       :2018-08-09
    @function   :Set coordinate value
    @parameters :Xpos:the  x coordinate of the pixel
                                Ypos:the  y coordinate of the pixel
    @retvalue   :None
******************************************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
    LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);
}

/*****************************************************************************
    @name       :void LCD_direction(u8 direction)
    @date       :2018-08-09
    @function   :Setting the display direction of LCD screen
    @parameters :direction:0-0 degree
                          1-90 degree
                                                    2-180 degree
                                                    3-270 degree
    @retvalue   :None
******************************************************************************/
void LCD_direction(u8 direction)
{
    lcddev.setxcmd=0x2A;
    lcddev.setycmd=0x2B;
    lcddev.wramcmd=0x2C;

    switch(direction)
    {
    case 0:
        lcddev.width=LCD_W;
        lcddev.height=LCD_H;
        LCD_WriteReg(0x36,(1<<3)|(0<<6)|(0<<7));//BGR==1,MY==0,MX==0,MV==0
        break;

    case 1:
        lcddev.width=LCD_H;
        lcddev.height=LCD_W;
        LCD_WriteReg(0x36,(1<<3)|(0<<7)|(1<<6)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
        break;

    case 2:
        lcddev.width=LCD_W;
        lcddev.height=LCD_H;
        LCD_WriteReg(0x36,(1<<3)|(1<<6)|(1<<7));//BGR==1,MY==0,MX==0,MV==0
        break;

    case 3:
        lcddev.width=LCD_H;
        lcddev.height=LCD_W;
        LCD_WriteReg(0x36,(1<<3)|(1<<7)|(1<<5));//BGR==1,MY==1,MX==0,MV==1
        break;

    default:
        break;
    }
}
