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
#ifndef __OTP_ROM_H__
#define __OTP_ROM_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "bus_dev.h"

/*test row*/
#define OTP_TEST_ROW_ENABLE   AP_COM->new_add_reg1 |= BIT(0)


#define OTP_TEST_ROW_DISABLE  AP_COM->new_add_reg1 &= ~BIT(0)


//0x00
#define OTP_HARDWARE_CONTROL 		AP_OTP->otp_mode_setting &= ~BIT(8)
#define OTP_SOFTWARE_CONTROL 		AP_OTP->otp_mode_setting |= BIT(8)

#define    OTP_OUT_TEST_MODE	        AP_OTP->otp_mode_setting &= ~(15ul<<4)
#define    OTP_IN_TEST_MODE_1        	do{\
											OTP_OUT_TEST_MODE;\
											AP_OTP->otp_mode_setting |= (1ul<<4);\
											}while(0)

#define    OTP_IN_TEST_MODE_2        	do{\
											OTP_OUT_TEST_MODE;\
											AP_OTP->otp_mode_setting |= (4ul<<4);\
											}while(0)

#define    OTP_IN_TEST_MODE_3        	do{\
										OTP_OUT_TEST_MODE;\
										AP_OTP->otp_mode_setting |= (9ul<<4);\
										}while(0)

#define    OTP_IN_TEST_MODE_4        	do{\
										OTP_OUT_TEST_MODE;\
										AP_OTP->otp_mode_setting |= (12ul<<4);\
										}while(0)
										
#define OTP_PROG_MODE_EN			AP_OTP->otp_mode_setting |= BIT(3)
#define OTP_PROG_MODE_DIS			AP_OTP->otp_mode_setting &= ~BIT(3)

#define OTP_DEEPSLEEP_MODE_EN		AP_OTP->otp_mode_setting |= BIT(2)	
#define OTP_DEEPSLEEP_MODE_DIS		AP_OTP->otp_mode_setting &= ~BIT(2)	

#define OTP_READ_EN				AP_OTP->otp_mode_setting |= BIT(1)
#define OTP_READ_DIS			AP_OTP->otp_mode_setting &= ~BIT(1)
										
#define OTP_POWERON			AP_OTP->otp_mode_setting |= BIT(0)
#define OTP_POWERDOWM		AP_OTP->otp_mode_setting &= ~BIT(0)
				
//0x44
#define OTP_PTM_SET(n)             subWriteReg(&(AP_OTP->ptm),3,0,n)    
										
//0x50
#define OTP_PCLK_EN		AP_OTP->pclk_clockgate_en |= BIT(0)						
#define OTP_PCLK_DIS		AP_OTP->pclk_clockgate_en &= ~BIT(0)

//0x54
#define OTP_PCE_SET		AP_OTP->pce |= BIT(0)
#define OTP_PCE_CLR		AP_OTP->pce &= ~BIT(0)


#define OTP_W_S_IDLE         (0)
//#define OTP_W_S_POWER_UP     (1)
#define OTP_W_S_STANDBY      (2)
#define OTP_W_S_READ         (3)
#define OTP_W_S_DEEPLEEP_IN  (4) 
//#define OTP_W_S_DEEPLEEP_OUT (5)
//#define OTP_W_S_PROG_ENTRY   (6)
#define OTP_W_S_PROG_ACCESS  (7)
#define OTP_W_S_TEST_MODE0   (8)
#define OTP_W_S_TEST_MODE1   (9)
#define OTP_W_S_TEST_MODE2   (10)
#define OTP_W_S_TEST_MODE3   (11)
//#define OTP_W_S_POWER_DOWN   (15)


typedef enum
{	
	OTP_USER_READ_MODE = 0,
	OTP_INIT_MARGIN_READ_ROOM_TEMP = 1,
	OTP_PGM_MARGIN_READ_ROOM_TEMP = 2,
	OTP_INIT_MARGIN_READ_HIGH_TEMP = 3,  
	OTP_PGM_MARGIN_READ_HIGH_TEMP = 4,  
	OTP_NO_READ_CHECK = 5,
	
}OTP_READ_Mode_e;

//otp mapping
#define OTP_BASE_ADDR          (0x1fff8000)
//offset:0x00~0x1f otp config
#define OTP_USER_SPACE             (0x20)

/*boot info*/
#define OTP_BOOT_AREA_ADDR             (OTP_USER_SPACE)
/*secure info*/
#define OTP_SECURE_AREA_ADDR           (0x90)

#if 1
//boot secure mic include all image info
#define OTP_BOOT_SEC_ADDR              ((uint8_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x10))
#define OTP_BOOT_BIN_NUM               (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR))
#define OTP_BOOT_MIC                   (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR + 4))
#define OTP_BOOT_RUN_ADDR              (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR + 8))

#define OTP_BOOT_PART_FADDR(n)         (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x10+(n)*0x10))
#define OTP_BOOT_PART_SIZE(n)          ((*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x14+(n)*0x10))&0xffffff)
#define OTP_BOOT_PART_SEC(n)           ((*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x14+(n)*0x10))&0x80000000)
#define OTP_BOOT_PART_AUTH(n)          ((*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x14+(n)*0x10))&0x40000000)
#define OTP_BOOT_PART_RADDR(n)         (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x18+(n)*0x10))
#define OTP_BOOT_PART_SEC_MIC(n)       (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x1c+(n)*0x10))
#define OTP_BOOT_PART_CRC(n)           (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_BOOT_AREA_ADDR+0x1c+(n)*0x10))


#define OTP_SECURE_AUTH_WORD           (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR)) //fct mode flag
#define OTP_SECURE_USE_SELECT           (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x4))
#define OTP_SECURE_KEY_L               (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x8)) //replace efuse data[0]
#define OTP_SECURE_KEY_H               (*(volatile uint32_t*)(OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0xc)) //replace efuse data[1]
//#define pOTP_SECURE_PLAINTEXT          ((volatile uint8_t*)(OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x10)) //default value
#define pOTP_SECURE_MIC                ((volatile uint8_t*)(OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x10))
#define pOTP_SECURE_IV                 ((volatile uint8_t*)(OTP_BASE_ADDR + OTP_SECURE_AREA_ADDR + 0x20))//128 bit, secure IV ffff_ffff --> 0000_0000

#endif

#define OTP_TVDDH(val)  ((((uint32_t) val) & 0xf)<<28) //bit 28~31
#define OTP_TPENH(val)  ((((uint32_t) val) & 0xf)<<24) //bit 24~27
#define OTP_TPLH(val)   ((((uint32_t) val) & 0xf)<<20) //bit 20~23
#define OTP_TASH(val)   ((((uint32_t) val) & 0xf)<<16) //bit 16~19
#define OTP_TSAS(val)   ((((uint32_t) val) & 0xf)<<12) //bit 12~15
#define OTP_TPLS(val)   ((((uint32_t) val) & 0xf)<<8)  //bit 8~11
#define OTP_TPENS(val)  ((((uint32_t) val) & 0xf)<<4)  //bit 4~7
#define OTP_TVDS(val)   ((((uint32_t) val) & 0xf)<<0)  //bit 0~3

#define OTP_TPWI(val)  ((((uint32_t) val) & 0x7)<<21)  //bit 21~23
#define OTP_TPW(val)   ((((uint32_t) val) & 0x1f)<<16) //bit 16~20
#define OTP_TPPS(val)  ((((uint32_t) val) & 0x1f)<<11) //bit 11~15
#define OTP_TCPS(val)  ((((uint32_t) val) & 0x7f)<<4)  //bit 4~10
#define OTP_TMS(val)   ((((uint32_t) val) & 0xf)<<0)   //bit 0~3

#define OTP_TMH(val)   ((((uint32_t) val) & 0xf)<<16)   //bit 16~19
#define OTP_TPPR(val)  ((((uint32_t) val) & 0x7f)<<8)  //bit 8~14
#define OTP_TPPH(val)  ((((uint32_t) val) & 0x1f)<<0)  //bit 0~4

#define OTP_TCS(val)   ((((uint32_t) val) & 0xf)<<0)          //bit 0~4
void otp_init(void);

//otp driver using hardmode(default mode)
void otp_control_software_mode(bool flag);
int otp_read_data_byte(uint32_t read_addr, uint8_t* data_buf, uint32_t data_len,OTP_READ_Mode_e mode);

int otp_read_data(uint32_t read_addr, uint32_t* data_buf, uint32_t data_len,OTP_READ_Mode_e mode);
int otp_prog_data_polling(uint32_t prog_addr, uint32_t* data_buf, uint32_t data_len,OTP_READ_Mode_e mode);

#if 0
//prog_addr:valid scope [0x1FFF8000,0x1FFFC000)
//data_buf:data unit is word
//data_len:word data number to prog
//timeout_us:0 means not timeout just wait otp finish int.
int otp_prog_data_int(uint32_t prog_addr, uint32_t* data_buf, uint32_t data_len,uint32_t timeout_us);
#endif

int otp_default_value_check(uint32_t default_value,OTP_READ_Mode_e mode);


//ate used api
uint16_t otp_ate_test_read(uint32_t default_val,uint8_t main_read_mode);
uint16_t otp_ate_test_row_prog(uint16_t prog_num,uint32_t prog_data);
uint16_t otp_ate_test_row_read(uint16_t read_num,uint32_t read_data,uint8_t read_mode);


#ifdef __cplusplus
}
#endif



#endif
