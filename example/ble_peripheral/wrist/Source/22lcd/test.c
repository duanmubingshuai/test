//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
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
#include "gui.h"
#include "test.h"
#include "key.h"
#include "pic.h"

//========================variable==========================//
u16 ColorTab[5]= {RED,GREEN,BLUE,YELLOW,BRED}; //������ɫ����
//=====================end of variable======================//

/*****************************************************************************
    @name       :void DrawTestPage(u8 *str)
    @date       :2018-08-09
    @function   :Drawing test interface
    @parameters :str:the start address of the Chinese and English strings
    @retvalue   :None
******************************************************************************/
void DrawTestPage(u8* str)
{
//���ƹ̶���up
    LCD_Clear(WHITE);
//    WaitMs(100);
//    LCD_Clear(BLUE);
//    WaitMs(100);
//    LCD_Clear(BRED);
//    WaitMs(100);
//    LCD_Clear(GRED);
//    WaitMs(100);
//    LCD_Clear(RED);
//    WaitMs(100);
//    LCD_Clear(MAGENTA);
//    WaitMs(100);
//    LCD_Clear(YELLOW);
//    WaitMs(100);
    LCD_Fill(0,0,lcddev.width,20,BLUE);
    //���ƹ̶���down
    LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
    POINT_COLOR=WHITE;
    Gui_StrCenter(0,2,WHITE,BLUE,str,16,1);//������ʾ
    Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"http://www.lcdwiki.com",16,1);//������ʾ
//    ���Ʋ�������
    LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
}

/*****************************************************************************
    @name       :void main_test(void)
    @date       :2018-08-09
    @function   :Drawing the main Interface of the Comprehensive Test Program
    @parameters :None
    @retvalue   :None
******************************************************************************/
void main_test(void)
{
    DrawTestPage("ȫ�������ۺϲ��Գ���");
    Gui_StrCenter(0,30,RED,BLUE,"ȫ������",16,1);//������ʾ
    Gui_StrCenter(0,60,RED,BLUE,"�ۺϲ��Գ���",16,1);//������ʾ
    Gui_StrCenter(0,90,GREEN,BLUE,"2.2\" ILI9341 240X320",16,1);//������ʾ
    Gui_StrCenter(0,120,BLUE,BLUE,"xiaoFeng@QDtech 2018-08-20",16,1);//������ʾ
    WaitMs(1500);
    WaitMs(1500);
}

/*****************************************************************************
    @name       :void Test_Color(void)
    @date       :2018-08-09
    @function   :Color fill test(white,black,red,green,blue)
    @parameters :None
    @retvalue   :None
******************************************************************************/
void Test_Color(void)
{
    //DrawTestPage("����1:��ɫ������");
    LCD_Fill(0,0,lcddev.width,lcddev.height,WHITE);
    Show_Str(20,30,BLUE,YELLOW,"BL Test",16,1);
    WaitMs(800);
    LCD_Fill(0,0,lcddev.width,lcddev.height,RED);
    Show_Str(20,30,BLUE,YELLOW,"RED ",16,1);
    WaitMs(800);
    LCD_Fill(0,0,lcddev.width,lcddev.height,GREEN);
    Show_Str(20,30,BLUE,YELLOW,"GREEN ",16,1);
    WaitMs(800);
    LCD_Fill(0,0,lcddev.width,lcddev.height,BLUE);
    Show_Str(20,30,RED,YELLOW,"BLUE ",16,1);
    WaitMs(800);
}

/*****************************************************************************
    @name       :void Test_FillRec(void)
    @date       :2018-08-09
    @function   :Rectangular display and fill test
                                Display red,green,blue,yellow,pink rectangular boxes in turn,
                                1500 milliseconds later,
                                Fill the rectangle in red,green,blue,yellow and pink in turn
    @parameters :None
    @retvalue   :None
******************************************************************************/
void Test_FillRec(void)
{
    u8 i=0;
    DrawTestPage("����2:GUI����������");
    LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);

    for (i=0; i<5; i++)
    {
        POINT_COLOR=ColorTab[i];
        LCD_DrawRectangle(lcddev.width/2-80+(i*15),lcddev.height/2-80+(i*15),lcddev.width/2-80+(i*15)+60,lcddev.height/2-80+(i*15)+60);
    }

    WaitMs(1500);
    LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);

    for (i=0; i<5; i++)
    {
        POINT_COLOR=ColorTab[i];
        LCD_DrawFillRectangle(lcddev.width/2-80+(i*15),lcddev.height/2-80+(i*15),lcddev.width/2-80+(i*15)+60,lcddev.height/2-80+(i*15)+60);
    }

    WaitMs(1500);
}

/*****************************************************************************
    @name       :void Test_Circle(void)
    @date       :2018-08-09
    @function   :circular display and fill test
                                Display red,green,blue,yellow,pink circular boxes in turn,
                                1500 milliseconds later,
                                Fill the circular in red,green,blue,yellow and pink in turn
    @parameters :None
    @retvalue   :None
******************************************************************************/
void Test_Circle(void)
{
    u8 i=0;
    DrawTestPage("����3:GUI��Բ������");
    LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);

    for (i=0; i<5; i++)
        gui_circle(lcddev.width/2-80+(i*25),lcddev.height/2-50+(i*25),ColorTab[i],30,0);

    WaitMs(1500);
    LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);

    for (i=0; i<5; i++)
        gui_circle(lcddev.width/2-80+(i*25),lcddev.height/2-50+(i*25),ColorTab[i],30,1);

    WaitMs(1500);
}

/*****************************************************************************
    @name       :void English_Font_test(void)
    @date       :2018-08-09
    @function   :English display test
    @parameters :None
    @retvalue   :None
******************************************************************************/
void English_Font_test(void)
{
    DrawTestPage("����5:Ӣ����ʾ����");
    Show_Str(10,30,BLUE,YELLOW,"6X12:abcdefghijklmnopqrstuvwxyz0123456789",12,0);
    Show_Str(10,45,BLUE,YELLOW,"6X12:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789",12,1);
    Show_Str(10,60,BLUE,YELLOW,"6X12:~!@#$%^&*()_+{}:<>?/|-+.",12,0);
    Show_Str(10,80,BLUE,YELLOW,"8X16:abcdefghijklmnopqrstuvwxyz0123456789",16,0);
    Show_Str(10,100,BLUE,YELLOW,"8X16:ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789",16,1);
    Show_Str(10,120,BLUE,YELLOW,"8X16:~!@#$%^&*()_+{}:<>?/|-+.",16,0);
    WaitMs(1200);
}

/*****************************************************************************
    @name       :void Test_Triangle(void)
    @date       :2018-08-09
    @function   :triangle display and fill test
                                Display red,green,blue,yellow,pink triangle boxes in turn,
                                1500 milliseconds later,
                                Fill the triangle in red,green,blue,yellow and pink in turn
    @parameters :None
    @retvalue   :None
******************************************************************************/
void Test_Triangle(void)
{
    u8 i=0;
    DrawTestPage("����4:GUI Triangle������");
    LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);

    for(i=0; i<5; i++)
    {
        POINT_COLOR=ColorTab[i];
        Draw_Triangel(lcddev.width/2-80+(i*20),lcddev.height/2-20+(i*15),lcddev.width/2-50-1+(i*20),lcddev.height/2-20-52-1+(i*15),lcddev.width/2-20-1+(i*20),lcddev.height/2-20+(i*15));
    }

    WaitMs(1500);
    LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);

    for(i=0; i<5; i++)
    {
        POINT_COLOR=ColorTab[i];
        Fill_Triangel(lcddev.width/2-80+(i*20),lcddev.height/2-20+(i*15),lcddev.width/2-50-1+(i*20),lcddev.height/2-20-52-1+(i*15),lcddev.width/2-20-1+(i*20),lcddev.height/2-20+(i*15));
    }

    WaitMs(1500);
}

/*****************************************************************************
    @name       :void Chinese_Font_test(void)
    @date       :2018-08-09
    @function   :chinese display test
    @parameters :None
    @retvalue   :None
******************************************************************************/
void Chinese_Font_test(void)
{
    DrawTestPage("����6:������ʾ����");
    Show_Str(10,30,BLUE,YELLOW,"16X16:ȫ�����Ӽ������޹�˾��ӭ��",16,0);
    Show_Str(10,50,BLUE,YELLOW,"16X16:Welcomeȫ������",16,0);
    Show_Str(10,70,BLUE,YELLOW,"24X24:���������Ĳ���",24,1);
    Show_Str(10,100,BLUE,YELLOW,"32X32:�������",32,1);
    WaitMs(1200);
}

/*****************************************************************************
    @name       :void Pic_test(void)
    @date       :2018-08-09
    @function   :picture display test
    @parameters :None
    @retvalue   :None
******************************************************************************/
void Pic_test(void)
{
    DrawTestPage("����7:ͼƬ��ʾ����");
    //LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
    Gui_Drawbmp16(30,30,gImage_qq);
    Show_Str(30+12,75,BLUE,YELLOW,"QQ",16,1);
    Gui_Drawbmp16(90,30,gImage_qq);
    Show_Str(90+12,75,BLUE,YELLOW,"QQ",16,1);
    Gui_Drawbmp16(150,30,gImage_qq);
    Show_Str(150+12,75,BLUE,YELLOW,"QQ",16,1);
    WaitMs(1200);
}

/*****************************************************************************
    @name       :void Rotate_Test(void)
    @date       :2018-08-09
    @function   :rotate test
    @parameters :None
    @retvalue   :None
******************************************************************************/
void Rotate_Test(void)
{
    u8 i=0;
    u8* Direction[4]= {"Rotation:0","Rotation:90","Rotation:180","Rotation:270"};

    for(i=0; i<4; i++)
    {
        LCD_direction(i);
        DrawTestPage("����8:��Ļ��ת����");
        Show_Str(20,30,BLUE,YELLOW,Direction[i],16,1);
        Gui_Drawbmp16(30,50,gImage_qq);
        WaitMs(1000);
        WaitMs(1000);
    }

    LCD_direction(USE_HORIZONTAL);
}




