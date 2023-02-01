
#ifndef __SPI_LITE_H
#define __SPI_LITE_H


#include "types.h"
#include "gpio.h"
#include "clock.h"
#include "bus_dev.h"


typedef enum{
	SPI_MODE0=0,      //SCPOL=0,SCPH=0
	SPI_MODE1,        //SCPOL=0,SCPH=1
	SPI_MODE2,        //SCPOL=1,SCPH=0
	SPI_MODE3,        //SCPOL=1,SCPH=1
}SPI_SCMOD_e;

typedef enum{
	 SPI_TRXD=0,        //Transmit & Receive
	 SPI_TXD,         //Transmit Only
	 SPI_RXD,         //Receive Only
   SPI_EEPROM,      //EEPROM Read	
}SPI_TMOD_e;


typedef struct{
  uint8_t pin;
  uint8_t pmux;   
} spi_pin_t;



typedef struct {
	spi_pin_t    sclk_pin;
	spi_pin_t    ssn_pin;
	spi_pin_t    MOSI;
	spi_pin_t    MISO;
	uint32_t      baudrate;
	SPI_SCMOD_e   spi_scmod;
}spilite_cfg_t;

int spi_init(void);
int spi_bus_init(spilite_cfg_t* cfg);
int spi_transmit(uint8_t* tx_buf,uint8_t* rx_buf,uint16_t len);
int spi_transmit_s(uint8_t* tx_buf,uint8_t* rx_buf,uint8_t len);


#endif


