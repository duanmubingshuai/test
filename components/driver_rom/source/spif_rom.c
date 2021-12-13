/*******************************************************************************
* @file		flash.c
* @brief	Contains all functions support for flash driver
* @version	0.0
* @date		27. Nov. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "spif.h"
#include "mcu.h"
#include "bus_dev.h"
#include "error.h"

// spif config register
#define SPIF_CONFIG_BASE   	0x11080000
#define SPIF_RLEN       	(SPIF_CONFIG_BASE+0x88)  
#define RFIFO_DEPTH     	(SPIF_CONFIG_BASE+0x84)

// commands defines
#define PAGE_PROGRAM_CMD      0x2
#define FAST_PROGRAM_CMD      0xF2

#define WRITE_ENABLE_CMD      0x6
#define WRITE_DISABLE_CMD     0x4

#define READ_STATUS_CMD       0x5
#define WRITE_STATUS_CMD      0x1

#define SECT_ERASE_CMD        0x20
#define BLOCK32_ERASE_CMD     0x52
#define BLOCK64_ERASE_CMD     0xD8
#define CHIP_ERASE_CMD        0xC7

#define DEEP_DOWN_CMD         0xB9
#define RELEASE_DOWN_CMD      0xAB
#define MANUF_ID_CMD          0x90
#define READ_ID_CMD           0x9F

#define FAST_READ_CMD         0x0B
#define READ_DATA_CMD         0x3

#define WAIT_FOR_SPIF()   {while(!( (*(volatile uint32_t *)(SPIF_CONFIG_BASE+0x90)) & 0x4)) ;}


typedef struct{
 //uint8_t state;
 uint32_t size;
 
 //bool w_protect;
 
 //uint8_t clk_div;
 
 //uint32_t qspi_en;
 //uint32_t rd_instr;
 //uint32_t wr_instr;
 //uint32_t remap;
 
 /*device id*/
 uint8_t mid;
 uint8_t mtype; //mempry type
 uint8_t mcapc; //memory capcity
 
}spif_ctx_t;

spif_ctx_t s_spif_ctx = {
  .size = 512*1024,
  
  .mid = 0,
  .mtype = 0,
  .mcapc = 0,
};

#ifdef FPGA_SPIF_TEST
uint8_t MID, JID8, JID0;
extern  uint32_t ReadWord (unsigned int addr);
#endif


uint32_t spif_flash_size(void)
{
    if(s_spif_ctx.size == 0)
        return 512*1024;
    return s_spif_ctx.size;
}

void spif_set_flash_size(uint32_t fsize)
{
    s_spif_ctx.size = fsize;
}

int  spif_read_id(uint32_t* pid)
{
    uint8_t id[3];
    uint8_t* p = (uint8_t*)pid;
    //spif_cmd(FCMD_RDID, 0, 3, 0, 0, 0);
	read_deviceID();
    //spif_rddata(id, 3);

    id[0] =  s_spif_ctx.mid;
	id[1] =  s_spif_ctx.mtype;
	id[2] =  s_spif_ctx.mcapc;

    if(s_spif_ctx.mcapc >= 0x11){
       s_spif_ctx.size = (1ul << (s_spif_ctx.mcapc));
    }
    else
    {
        s_spif_ctx.size = 512*1024;
    }
    
    if(p){
        p[0] = id[0];
        p[1] = id[1];
        p[2] = id[2];
        p[3] = 0 ;
    }
    return PPlus_SUCCESS;
}


int flash_erase_all(void)
{   
    uint32_t addr = SPIF_USER_SPACE;
    for(;addr < 0x10000; addr += 0x1000){
        flash_sector_erase(addr);
    }
    
    for(; addr < s_spif_ctx.size; addr += 0x10000){
        flash_block64_erase(addr);
    }
    
	return PPlus_SUCCESS;
}

int spif_program_write(uint32_t addr, uint8_t* data, uint32_t size)
{
	uint32_t spif_ad = ((addr&0x7ffff)|SPIF_BASE_ADDR);
	uint32_t ram_ad = (uint32_t)data;
	uint32_t len = 0;	
	while(size)
    {
        len = size > 64 ? 64 :size;
        if(ProgramPage64 (spif_ad, (const unsigned char *)ram_ad ,len)==0)
		{
			return PPlus_ERR_INVALID_PARAM;
		}
        size -= len;
        ram_ad += len;
        spif_ad += len;
    }
	return PPlus_SUCCESS;
}

/////////////////////////////////   spif  flash   /////////////////////////////////////
void init_spif(void)//rom
{	
	*(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x88) = 0;
	
	*(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x8c)= 0;
		
	*(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x8c) = 0x32;
	
	*(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x90) = 0x3;
	
	//*(volatile unsigned int *)(SPIF_CONFIG_BASE+0x8c) = 0x7;   //enable spif	 
	*(volatile unsigned int *)(SPIF_CONFIG_BASE+0x8c) = 0x3;   //enable spif	  fpga test
}

// disable spif interface
void close_spif(void)//rom,patch
{
	*(volatile unsigned int *)(SPIF_CONFIG_BASE + 0x8c) = 0;   //enable spif	
}

//enable flash write 
void enable_spif_flash_write(void)//rom,patch
{
	//HAL_DISABLE_INTERRUPTS();
	WAIT_FOR_SPIF();
	
	*(volatile unsigned int *)SPIF_RLEN = 0;
	*(volatile uint8_t *)(SPIF_CONFIG_BASE) = WRITE_ENABLE_CMD;
	
	WAIT_FOR_SPIF();
	//HAL_ENABLE_INTERRUPTS();		
}

// disable spif write
void disable_spif_flash_write(void)//rom,patch
{
    //HAL_DISABLE_INTERRUPTS();
    
	WAIT_FOR_SPIF();
	
	*(volatile unsigned int *) SPIF_RLEN = 0;
	*(volatile uint8_t *)(SPIF_CONFIG_BASE) = WRITE_DISABLE_CMD;
	
	WAIT_FOR_SPIF();

    //HAL_ENABLE_INTERRUPTS();	
}

// check flash internal bus busy or notfor erase and program operation
int check_internal_bus_busy(void)//rom,patch
{
	int  state1=0;

    //HAL_DISABLE_INTERRUPTS();
	
	WAIT_FOR_SPIF();
	
	*(volatile unsigned int *) SPIF_RLEN =1;
	
	*(volatile uint8_t *)(SPIF_CONFIG_BASE) = READ_STATUS_CMD;
	
	WAIT_FOR_SPIF();   //不能去掉
	
	state1= *(volatile  uint8_t *)(SPIF_CONFIG_BASE);	

    //HAL_ENABLE_INTERRUPTS();
	
	return (( state1 & 0x1));
	
}

// read flash internal status regiser
int read_status_register(void)//rom
{
	 unsigned int  state;

     //HAL_DISABLE_INTERRUPTS();
	
	 WAIT_FOR_SPIF();
	
	 *(volatile unsigned int*)SPIF_RLEN =1;
	
	 *(volatile uint8_t *)(SPIF_CONFIG_BASE) = READ_STATUS_CMD;
	
	 WAIT_FOR_SPIF();
	
	 state = *(volatile uint8_t *)(SPIF_CONFIG_BASE);

     //HAL_ENABLE_INTERRUPTS();

	 return (state & 0xff);
}

// write flash internal status register
void write_status_register(int value)//rom
{
    //HAL_DISABLE_INTERRUPTS();
	while(check_internal_bus_busy());
	
	enable_spif_flash_write();
			
	*(volatile unsigned int *)SPIF_RLEN = 0;
	
	*(volatile unsigned short *)(SPIF_CONFIG_BASE) = (WRITE_STATUS_CMD | (value << 8));

	while(check_internal_bus_busy());
		
	disable_spif_flash_write();

    //HAL_ENABLE_INTERRUPTS();		
}

// read device ID of flash
void  read_deviceID(void)//rom
{	
	 WAIT_FOR_SPIF();
	
	 *(volatile unsigned int *) SPIF_RLEN = 3;  //set rlen
	
	 *(volatile uint8_t  *) (SPIF_CONFIG_BASE) = READ_ID_CMD;  // command
	
	 WAIT_FOR_SPIF();   //不能去掉
		
	 s_spif_ctx.mid	= (*(volatile uint8_t  *) (SPIF_CONFIG_BASE));  // read data
	 s_spif_ctx.mtype 	= (*(volatile uint8_t  *) (SPIF_CONFIG_BASE));  // read data
	 s_spif_ctx.mcapc 	= (*(volatile uint8_t  *) (SPIF_CONFIG_BASE));  // read data
	 
#ifdef FPGA_SPIF_TEST
	MID	= s_spif_ctx.mid;
	JID8 = s_spif_ctx.mtype;
	JID0 = s_spif_ctx.mcapc; 
#endif

// read result should be 51 40 13
}


//phyplusinc
// erase a sector of flash, size is 4KB
int flash_sector_erase(unsigned int addr)//rom
{   	
	int little_addr;

    //HAL_DISABLE_INTERRUPTS();
	
	while(check_internal_bus_busy());
	
	enable_spif_flash_write();   // enable write
	
	*(volatile unsigned int *) SPIF_RLEN =0;
	
	little_addr = (((addr & 0xff) << 16) | ((addr & 0xff0000) >> 16) | (addr & 0xff00));
	
	*(volatile unsigned int *) (SPIF_CONFIG_BASE) = SECT_ERASE_CMD | (little_addr << 8);
	
	while(check_internal_bus_busy());		

	disable_spif_flash_write();   // diasble write
		
    //HAL_ENABLE_INTERRUPTS();
	return PPlus_SUCCESS;
}


//phyplusinc
#ifndef __BUILD_FOR_8KROM0__
//form prime patch
// erase a block whose size is 32KB
void flash_block32_erase(unsigned int addr)//rom
{   	
	int little_addr;

#ifndef FPGA_SPIF_TEST
	if ((addr&0xffffff)<0x2000)
    {
        return;
	}
#endif
	
    //HAL_DISABLE_INTERRUPTS();
	
	while( check_internal_bus_busy() ) ;
	
	enable_spif_flash_write();   // enable write
	
	*(volatile unsigned int *) SPIF_RLEN =0;
	
	little_addr=(addr & 0xff)<<16 | (addr &0xff0000)>>16 | (addr & 0xff00);
	
	*(volatile unsigned int  *) (SPIF_CONFIG_BASE) = BLOCK32_ERASE_CMD | (little_addr <<8);
		
	while(check_internal_bus_busy());	
			
	disable_spif_flash_write();   // diasble write
		
	//HAL_ENABLE_INTERRUPTS();	
}
#endif

//plyplusinc
//form prime patch
// erase a block whose size is 64KB
int flash_block64_erase(unsigned int addr)//rom
{   
	int little_addr;

	while( check_internal_bus_busy() ) ;
	
	enable_spif_flash_write();   // enable write
	
	*(volatile unsigned int *) SPIF_RLEN =0;
	
	little_addr=(addr & 0xff)<<16 | (addr &0xff0000)>>16 | (addr & 0xff00);
	
	*(volatile uint32_t  *) (SPIF_CONFIG_BASE) = BLOCK64_ERASE_CMD | (little_addr <<8);
		
	while(check_internal_bus_busy());		
		
	disable_spif_flash_write();   // diasble write
		
	//HAL_ENABLE_INTERRUPTS();	
	return PPlus_SUCCESS;
}


// erase total chip of flash
int flash_chip_erase(void)//rom
{   
    //HAL_DISABLE_INTERRUPTS();
    
	while(check_internal_bus_busy());

    //HAL_DISABLE_INTERRUPTS();
	
	enable_spif_flash_write();   // enable write
	
	*(volatile unsigned int *) SPIF_RLEN =0;
	
	*(volatile uint8_t  *) (SPIF_CONFIG_BASE) = CHIP_ERASE_CMD;
		
	while(check_internal_bus_busy());		
	
	disable_spif_flash_write();   // diasble write	

    //HAL_ENABLE_INTERRUPTS();
	return PPlus_SUCCESS;
}


// set flash to deep sleep mode
void set_flash_deep_sleep(void)//rom
{
    //HAL_DISABLE_INTERRUPTS();
	WAIT_FOR_SPIF();
	
	*(volatile unsigned int *) SPIF_RLEN =0;  //set rlen
	
	*(volatile uint8_t  *) (SPIF_CONFIG_BASE) = DEEP_DOWN_CMD;  // command
	
	WAIT_FOR_SPIF();   //不能去掉 
	//HAL_ENABLE_INTERRUPTS();	
}

// wake up flash from sleep state
void release_flash_deep_sleep(void)//rom
{	
    //HAL_DISABLE_INTERRUPTS();
	WAIT_FOR_SPIF();
	
	*(volatile unsigned int *) SPIF_RLEN =0;  //set rlen
	
	*(volatile uint8_t  *) (SPIF_CONFIG_BASE) = RELEASE_DOWN_CMD;  // command
	
	WAIT_FOR_SPIF();   //不能去掉	
	//HAL_ENABLE_INTERRUPTS();
}

/*static*/ int ProgramWordShort(unsigned long offset, const unsigned char *buf,uint8_t size)   
{
	int    little_addr;
    
	while(check_internal_bus_busy());
	
    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

	enable_spif_flash_write();   // enable write
	
	little_addr = (((offset & 0xff) << 16) | ((offset & 0xff0000) >> 16) | (offset & 0xff00));
		
    *(volatile unsigned int *) SPIF_RLEN = 0;    // write  0x0 to RLEN	

    *(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));
    	
    *(volatile unsigned char *)(SPIF_CONFIG_BASE) = buf[0];
    *(volatile unsigned char *)(SPIF_CONFIG_BASE) = buf[1];
  	
	while( check_internal_bus_busy());

	HAL_EXIT_CRITICAL_SECTION();
	
	disable_spif_flash_write();   // disable  write
		
	return PPlus_SUCCESS;
}

int ProgramPage4 (unsigned long offset, uint32 data)
{
	int little_addr;
		
	while( check_internal_bus_busy());
	
	 _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
		
	enable_spif_flash_write();   // enable write
	
	little_addr = (((offset & 0xff) << 16) | ((offset & 0xff0000) >> 16) | (offset & 0xff00));		
	
	*(volatile unsigned int *) SPIF_RLEN =0;    // write  0x0 to RLEN	
		
	*(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));
       
	*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data;					 
	
	HAL_EXIT_CRITICAL_SECTION();
	
	while( check_internal_bus_busy());
	
	disable_spif_flash_write();   // disable  write
		
	return PPlus_SUCCESS;
}


// program 64 bytes into flash
int ProgramPage64 (unsigned long offset, const unsigned char *buf,  int size)   // size must be <=64
{
	int    i=0, j;
	int    little_addr;
    int    data[16];
	int    baseAddr;
	
	if( size> 64)  return 0;
	if(size ==0 )  return 0;
	 
	for(i=0; i<16; i++)
	{
	    data[i]=0;
	}	
 	i=0;
	while(i<size)
	{
		data[i>>2] |= buf[i]<< ((i&0x03)<<3);
		i++;
	}
	
	j= (size & 0x03)? ((size>>2)+1): (size>>2);
	
	i=0;
		
	while( check_internal_bus_busy());
	
	 _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
		
	enable_spif_flash_write();   // enable write
	
	little_addr=(offset & 0xff)<<16 | (offset &0xff0000)>>16 | (offset & 0xff00);
		
#if 1	
	if(size<=4)
	{
		*(volatile unsigned int *) SPIF_RLEN = 0;    // write  0x0 to RLEN	
		
		*(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));
		 	
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[0];
	}
	else if(size<64)
	{
		*(volatile unsigned int *) SPIF_RLEN = 0;    // write  0x0 to RLEN	
		
		*(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));
		
	 	while(i<j)
		{			 
		    *(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[i++];			
		}

	}
	else if(size == 64)
	{
		*(volatile unsigned int *)SPIF_RLEN = 0;    // write  0x0 to RLEN	
		
		*(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));
					 
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[0];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[1];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[2];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[3];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[4];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[5];
			
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[6];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[7];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[8];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[9];
		 
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[10];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[11];
		 
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[12];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[13];
		 
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[14];
		
		*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data[15];			
	}	
#endif
	
	HAL_EXIT_CRITICAL_SECTION();
	
	while( check_internal_bus_busy());
	
	disable_spif_flash_write();   // disable  write

	// verify program contents
	i=0;
	baseAddr = 0x11000000+ offset;
	while(i<j)
	{
        if((baseAddr + i*4) < 0x11080000)
        {
            if( *(volatile int *) (baseAddr + i*4)!= data[i])  return 0;
        }
        else
        {
            if(ReadFlashWord(baseAddr + i*4) != data[i]) return 0;
        }
        i++;		 		
	}
		
	return size;
}


int ProgramWord (unsigned long offset, const unsigned char *buf,uint8_t size)
{
	int    i=0;
	int    little_addr;
	int    data=0;

	while(i<4)
	{
		data |= buf[i]<< ((i & 0x03) << 3);
		i++;
	}
	while(check_internal_bus_busy());

	enable_spif_flash_write();   // enable write

	little_addr=(offset & 0xff)<<16 | (offset &0xff0000)>>16 | (offset & 0xff00);

	 _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
	*(volatile unsigned int *) SPIF_RLEN =0;    // write  0x0 to RLEN	

	*(volatile unsigned int *)SPIF_CONFIG_BASE = (PAGE_PROGRAM_CMD | (little_addr << 8));
	*(volatile unsigned int *)(SPIF_CONFIG_BASE) = data;
	HAL_EXIT_CRITICAL_SECTION();

	while( check_internal_bus_busy());

	disable_spif_flash_write();   // disable  write

	//HAL_ENABLE_INTERRUPTS();
	if(ReadFlashWord(0x11000000 + offset) != data)
		return PPlus_ERR_FATAL;

	return PPlus_SUCCESS;
}


// write a Word to flash
int WriteFlash(unsigned int offset, uint32_t  value)
{
	uint32_t temp = value;
    offset &= 0x00ffffff;
	    
    if(ProgramWord(offset, (uint8_t *) &temp, 4) == PPlus_SUCCESS)  
		return PPlus_SUCCESS;
	
	return PPlus_ERR_FATAL; 	
}

// write a Word to flash
int WriteFlashShort(unsigned int offset, uint16_t  value)
{
	uint16_t temp = value;
	offset &= 0x00ffffff;
	    
	if(ProgramWordShort(offset, (uint8_t *) &temp, 2) == PPlus_SUCCESS)  
		return PPlus_SUCCESS;
	
	return PPlus_ERR_FATAL; 	
}

// read one byte from flash
uint8_t ReadFlash (unsigned int addr)//rom
{
	uint8_t value;
	uint32_t little_addr;
    
	WAIT_FOR_SPIF();
    
    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();

	*(volatile unsigned int *) SPIF_RLEN = 1;	
	
	little_addr = (((addr & 0xff) << 16) | ((addr & 0xff0000) >> 16) | (addr & 0xff00));
	
	*(volatile unsigned int *)SPIF_CONFIG_BASE = READ_DATA_CMD | (little_addr << 8);
      
	WAIT_FOR_SPIF();   //不能去掉

    HAL_EXIT_CRITICAL_SECTION();
	
	value = *(volatile uint8_t *)(SPIF_CONFIG_BASE);
	
	return value;
}

//patch
uint32_t ReadFlashWord (unsigned int addr)
{
	uint32_t value;
	uint32_t little_addr;
    
	WAIT_FOR_SPIF();
    
     _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
	*(volatile unsigned int *) SPIF_RLEN = 4;
	
	little_addr = (((addr & 0xff) << 16) | ((addr & 0xff0000) >> 16) | (addr & 0xff00));
	
	*(volatile unsigned int *)SPIF_CONFIG_BASE = READ_DATA_CMD | (little_addr << 8);
      
	WAIT_FOR_SPIF(); 

    HAL_EXIT_CRITICAL_SECTION();
	
	value= *(volatile uint32_t *)(SPIF_CONFIG_BASE);
	
	return value;
}
