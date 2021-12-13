/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree tvoid SM_Init0( uint8 task_id )
o abide by the terms of these agreements. 
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
  Filename:       simpleBLEPeripheral.c
  Revised:        
  Revision:       

  Description:    This file contains the Simple BLE Peripheral sample application
                  

**************************************************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "simpleBLEPeripheral.h"
#include "jump_function.h"
#include  "log.h"
/*********************************************************************
 * MACROS
 */
//#define LOG(...)  
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************a************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 application_TaskID; 

void gpio_pull_up_down_test(void)
{
	gpio_pupd_e type = GPIO_PULL_DOWN;//GPIO_PULL_DOWN GPIO_PULL_UP GPIO_PULL_UP_S
	gpio_pin_e pin;

	LOG("pull type:%d\n",type);	
	
	_symrom_WaitMs(10);
	_symrom_gpio_fmux_control(P9,Bit_DISABLE);
	_symrom_gpio_fmux_control(P10,Bit_DISABLE); 
	for(pin=P0;pin<GPIO_NUM;pin++)
	{
		_symrom_gpio_pull_set(pin,type);			
	}	
	
	while(1);;;
}

static void gpio_in_out(gpio_pin_e j,gpio_pin_e k)
{
    volatile int i,r;
    volatile uint8_t data_wr[2]={0,1};
	//L1:
	for(i=0;i<2;i++)
	{		
		gpio_write(j,data_wr[i]);//_symrom_
		_symrom_gpio_dir_r(k,GPIO_INPUT);
		{
			r = gpio_read(k);
			if(r!=data_wr[i])
			{									
				LOG("err [output pin:%d output value:%d] [input pin:%d input value:%d]\n",j,data_wr[i],k,r);
				//while(1);
			}
			else
			{
				LOG("ok  [output pin:%d output value:%d] [input pin:%d input value:%d]\n",j,data_wr[i],k,r);
			}			
		}	
	}	
	//goto L1;		
	for(i=0;i<2;i++)
	{
		gpio_write(k,data_wr[i]);
		_symrom_gpio_dir_r(j,GPIO_INPUT);
		{
			r = gpio_read(j);
			if(r!=data_wr[i])
			{									
				LOG("err [output pin:%d output value:%d] [input pin:%d input value:%d]\n",k,data_wr[i],j,r);
				//while(1);
			}
			else
			{
				LOG("ok  [output pin:%d output value:%d] [input pin:%d input value:%d]\n",k,data_wr[i],j,r);
			}			
		}	
	}	
	LOG("\n");
}

void gpio_input_output_test(void)
{
#if 0
	gpio_in_out(P0,P1);	
	gpio_in_out(P2,P3);
	gpio_in_out(P7,P11);
#endif	

#if 0
	// gpio_in_out(P34,P33);//ok
	// gpio_in_out(P32,P31);//ok
	// gpio_in_out(P26,P25);//ok

	// gpio_in_out(P24,P23);
	// gpio_in_out(P14,P15);
	// gpio_in_out(P18,P20);
#endif	
	gpio_in_out(P9,P10);
	while(1);;	
}

uint32_t t1 = 0,t2 = 0;
void usr0_sleep_handler(void)
{
	t1++;
}

void usr0_wakeup_handler(void)
{
	t2++;
}

volatile unsigned int int_flag = 0;
uint8_t pin_map[GPIO_NUM] = 
{
	0,    //p0
	1,    //p1
	2,    //p2
	3,    //p3
	7,    //p7
	9,    //p9
	10,   //p10
	11,   //p11
	14,   //p14
	15,   //p15
	16,   //p16
	17,   //p17
	18,   //p18
	20,   //p20
	23,   //p23
	24,   //p24
	25,   //p25
	26,   //p26
	27,   //p27
	31,   //p31
	32,   //p32
	33,   //p33
	34,   //p34
};

extern uint32_t s_gpio_wakeup_src[2];
void pos_cb(gpio_pin_e pin,gpio_polarity_e type)
{
	uint8_t pin_idx = pin_map[pin];
	uint32_t triggered = s_gpio_wakeup_src[pin_idx/32] & BIT(pin_idx%32);
	
	LOG("pin:%d (pos) 0x%x 0x%x ",pin,s_gpio_wakeup_src[0],s_gpio_wakeup_src[1]);
	int_flag++;

	if(triggered)
	{
		LOG("wakeup:%d\n",int_flag);
	}
	else
	{
		LOG("int:%d\n",int_flag);
	}
}

void neg_cb(gpio_pin_e pin,gpio_polarity_e type)
{
	uint8_t pin_idx = pin_map[pin];
	uint32_t triggered = s_gpio_wakeup_src[pin_idx/32] & BIT(pin_idx%32);

	LOG("pin:%d (neg) 0x%x 0x%x ",pin,s_gpio_wakeup_src[0],s_gpio_wakeup_src[1]);
	int_flag++;

	if(triggered)
	{
		LOG("wakeup:%d\n",int_flag);
	}
	else
	{
		LOG("int:%d\n",int_flag);
	}
}

void gpio_int_test(gpio_pin_e pin)//P0~P34 falling-edge or rising-edge
{
	int ret;

	NVIC_SetPriority((IRQn_Type)GPIO_IRQn, IRQ_PRIO_HAL);
	NVIC_EnableIRQ((IRQn_Type)GPIO_IRQn);	
		
	while(1)
	{
		int_flag = 0;
		LOG("start:\n");
		ret = _symrom_gpioin_register(pin,pos_cb,neg_cb);
		if(ret != PPlus_SUCCESS)
		{
			LOG("ret:%d %d\n",ret,__LINE__);
			while(1);
		}

		while(int_flag< 10);	

		LOG("stop:\n");	
		ret =_symrom_gpioin_unregister(pin);
		if(ret != PPlus_SUCCESS)
		{
			LOG("ret:%d %d\n",ret,__LINE__);
			while(1);
		}
		LOG("ret:%d %d\n",ret,__LINE__);
	}
}

void gpio_int_test_many(void)
{
	int ret;
	gpio_pin_e pins[]={P0,P1,P2,P3,P7,P11,P14,P15,P18,P20,P23,P24,P25,P26,P31,P32,P33,P34,P35};
	//gpio_pin_e pins[]={P9,P10};
	uint32_t pin;
	uint8_t j=0;

	NVIC_SetPriority((IRQn_Type)GPIO_IRQn, IRQ_PRIO_HAL);
	NVIC_EnableIRQ((IRQn_Type)GPIO_IRQn);	
			
	pin = pins[j];
	while(1)
	{		
		int_flag = 0;
		LOG("start:pin:%d\n",pin);
		ret = _symrom_gpioin_register(pin,pos_cb,neg_cb);
		if(ret != PPlus_SUCCESS)
		{
			LOG("ret:%d %d\n",ret,__LINE__);
			while(1);
		}

		while(int_flag< 10);	

		LOG("stop:\n");	
		ret =_symrom_gpioin_unregister(pin);
		if(ret != PPlus_SUCCESS)
		{
			LOG("ret:%d %d\n",ret,__LINE__);
			while(1);
		}

		if(++j < sizeof(pins)/sizeof(pins[0]))
		{
			pin = pins[j];
		}
		else
		{
			break;
		}		
	}
	
	LOG("test end,no problem\n");
	while(1);;
}

void gpio_wakeup_test(gpio_pin_e pin)
{
	int ret;
	gpio_pupd_e type = GPIO_PULL_DOWN;//GPIO_PULL_DOWN GPIO_PULL_UP GPIO_PULL_UP_S

	_symrom_gpio_pull_set(pin,type);	
	ret = _symrom_gpioin_register(pin,pos_cb,neg_cb);
	if(ret != PPlus_SUCCESS)
	{
		LOG("ret:%d %d\n",ret,__LINE__);
		while(1);
	}
	LOG("pin:%d\n",pin);
}

void gpio_wakeup_test_many(void)
{
	gpio_pupd_e type = GPIO_PULL_DOWN;//GPIO_PULL_DOWN GPIO_PULL_UP GPIO_PULL_UP_S
	gpio_pin_e pins[]={P0,P1,P2,P3,P7,P11,P14,P15,P18,P20,P23,P24,P25,P26,P31,P32,P33,P34,P35};
	//gpio_pin_e pins[]={P9,P10};

	uint32_t pin;
	uint8_t j = 0;
	int ret;
		
	while(1)
	{
		pin = pins[j];
		_symrom_gpio_pull_set(pin,type);
		ret = _symrom_gpioin_register(pin,pos_cb,neg_cb);
		if(ret != PPlus_SUCCESS)
		{
			LOG("ret:%d %d\n",ret,__LINE__);
			while(1);
		}
		LOG("pin:%d\n",pin);
		if(++j >= sizeof(pins)/sizeof(pins[0]))
		{
			break;
		}
	}	
}

extern void uart_test(void);
gpioin_t pin_test[GPIO_NUM];

#if 0
void GPIO_Init( uint8 task_id )
{
	int ret;
	int testcase = 3;

	application_TaskID = task_id;

	_symrom_gpio_fmux_control(P35,Bit_DISABLE);
	_symrom_gpio_fmux_control(P36,Bit_DISABLE); 
	_symrom_gpio_fmux_control(P37,Bit_DISABLE); 
	_symrom_gpio_fmux_control(P38,Bit_DISABLE);
	*(volatile int*)0x40003814 = 3;

	LOG("GPIO_Init\n");
	LOG("fullmux reg:0x%x\n",*(volatile int*)0x4000380c);
	LOG("sw reg:0x%x\n",*(volatile int*)0x40003814);

	switch(testcase)
	{
		case 0:
			LOG("gpio pull up down test\n");
			gpio_pull_up_down_test();
			break;

		case 1:
			LOG("gpio input and output test\n");
			gpio_input_output_test();
			break;
		case 2:
			LOG("gpio int test\n");
			ret = gpioin_init(&(pin_test[0]),sizeof(pin_test)/sizeof(pin_test[0]));
			if(ret != PPlus_SUCCESS)
			{	
				LOG("ret:%d %d\n",ret,__LINE__);
				while(1);
			}

			gpio_int_test(P34);
			//gpio_int_test_many();
			while(1);
			break;

		case 3:
			ret = gpioin_init(pin_test,sizeof(pin_test)/sizeof(pin_test[0]));
			if(ret != PPlus_SUCCESS)
			{
				LOG("ret:%d %d\n",ret,__LINE__);
				while(1);
			}
			//gpio_wakeup_test(P33);
			//gpio_wakeup_test(P34);
			gpio_wakeup_test_many();
			break;
		case 4:
			 uart_test();
			 break;
		default:
			break;
	}

	pwrmgr_register(MOD_USR0, usr0_sleep_handler, usr0_wakeup_handler);
	osal_start_timerEx(application_TaskID, OSAL_ONCE_TIMER_EVT, 5000);
	_symrom_osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 5000);
}
#endif

#if 1
void watchdog_test(void)
{
	LOG("watchdog test\n");
	
#if(CFG_WDT_ENABLE==1)
	 if(AP_WDT_ENABLE_STATE)
	 {
		 LOG("watchdog enable\n");
	 }
	 else
	 {
		 LOG("watchdog disable1\n");
	 }	 
#else 
	 {
		  LOG("watchdog disable2\n");
	 }	 
#endif
	LOG("wait 300s\n");
	for(int i=1;i<300;i++)
	{
			_symrom_WaitMs(1000);
			LOG("%ds ",i);
			AP_WDT_FEED;
	}
	LOG("system run\n");
}

void GPIO_Init( uint8 task_id )
{
	int ret;

	application_TaskID = task_id;
	//watchdog_test();

	pwrmgr_register(MOD_USR0, usr0_sleep_handler, usr0_wakeup_handler);
	osal_start_timerEx(application_TaskID, OSAL_ONCE_TIMER_EVT, 500);
	_symrom_osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 500);
}
#endif

uint16 GPIO_ProcessEvent( uint8 task_id, uint16 events )
{
	VOID task_id;
	static uint8_t test_flag = 0;

	if ( events & OSAL_ONCE_TIMER_EVT )
	{
		LOG("t1:%d\n",t1);
		osal_start_timerEx(application_TaskID, OSAL_ONCE_TIMER_EVT, 500);
		return ( events ^ OSAL_ONCE_TIMER_EVT );
	}

	if ( events & OSAL_RELOAY_TIMER_EVT )
	{
		LOG("t2:%d\n",t2);
		if(test_flag == 0)
		{
			test_flag = 1;
			watchdog_test();
		}
		return ( events ^ OSAL_RELOAY_TIMER_EVT );
	}

	return 0;
}

#define IO_FULLMUX_UART
#ifdef IO_FULLMUX_UART
uint8_t tx_finish = 1;
uint16_t i;
volatile uint8_t test_buf_4[100];
volatile uint8_t rx_buf[100];
volatile uint8_t rx_i=0;
volatile uint32_t rx_c=0;
volatile uint8_t rx_flag =0;

uint8_t hello[]="uart test";
void uart_evt_hdl(uart_Evt_t* pev)
{
	switch(pev->type)
	{
		case  UART_EVT_TYPE_TX_COMPLETED:
            tx_finish = 1;
            break;
		
		case  UART_EVT_TYPE_RX_DATA:
		case  UART_EVT_TYPE_RX_DATA_TO:
			for(i=0;i<pev->len;i++)
			{
				rx_buf[rx_i] = *(pev->data+i);
				rx_i++;
			}
			rx_c = 1000;
			rx_flag = 1;
		break;
		
		default:
		break;
	}
}

void hal_UART0_IRQ(void)
{
	_symrom_hal_UART0_IRQHandler();
}

void hal_UART1_IRQ(void)
{
	_symrom_hal_UART1_IRQHandler();
}

uart_Cfg_t uart_cfg =
{
	.tx_pin = P33,
	.rx_pin = P34,
	.rts_pin = GPIO_DUMMY,
	.cts_pin = GPIO_DUMMY,
	.baudrate = 115200,
	.use_fifo = TRUE,
	.hw_fwctrl = FALSE,
	.use_tx_buf = FALSE,
	.parity     = FALSE,
	.fcr = FCR_TX_FIFO_RESET|FCR_RX_FIFO_RESET|FCR_FIFO_ENABLE|UART_FIFO_RX_TRIGGER|UART_FIFO_TX_TRIGGER,
	.evt_handler = uart_evt_hdl,
};
	
volatile UART_INDEX_e uart_sel=UART0;

void uart_rx_tx(bool p_use_tx_buf,bool p_use_fifo)
{	
	int ret;
	static int first_flag = 0;
	
	rx_i = 0;
	rx_c = 1;
	rx_flag = 0;
	memset(test_buf_4,0x00,sizeof(test_buf_4));
	memset(rx_buf,0x00,sizeof(rx_buf));
		
	_symrom_hal_uart_deinit(uart_sel);
		
	if(uart_sel== UART0)
    {
        JUMP_FUNCTION_SET(UART0_IRQ_HANDLER,(uint32_t)&hal_UART0_IRQ);
    }
    else
    {
         JUMP_FUNCTION_SET(UART1_IRQ_HANDLER,(uint32_t)&hal_UART1_IRQ);
    }

	if(p_use_fifo == TRUE)
	{
		uart_cfg.fcr = FCR_TX_FIFO_RESET|FCR_RX_FIFO_RESET|FCR_FIFO_ENABLE|UART_FIFO_RX_TRIGGER|UART_FIFO_TX_TRIGGER;
	}
	else
	{
		uart_cfg.fcr = 0;
	}
	
	if((p_use_tx_buf == true) && (p_use_fifo==true))
	{
		_symrom_hal_uart_init(uart_cfg,uart_sel);	
		_symrom_hal_uart_set_tx_buf(uart_sel,test_buf_4,sizeof(test_buf_4) / sizeof(test_buf_4[0]));			
	}
	else
	{
		_symrom_hal_uart_init(uart_cfg,uart_sel);
	}	

	if(first_flag == 0)
	{
		first_flag = 1;
		_symrom_hal_uart_send_buff(uart_sel,hello,sizeof(hello)/sizeof(uint8));
	}
	
	while(1)
	{				
		if(rx_flag == 0)
		continue;

		if(rx_c > 5)
		{
			rx_c--;
		}
		else if(rx_c == 5)
		{					
			if((p_use_tx_buf == true) && (p_use_fifo==true))
			{
				tx_finish = 0;
				_symrom_hal_uart_send_buff(uart_sel,&(rx_buf[0]),rx_i);
				_symrom_WaitMs(10);
				while(1)
				{
					if(tx_finish == 1)
					break;
				}
			}
			else
			{
				_symrom_hal_uart_send_buff(uart_sel,rx_buf,rx_i);
			}
			
			rx_i = 0;
			rx_c = 1;
			rx_flag = 0;
			
			_symrom_hal_uart_deinit(uart_sel); 
			break;
		}
	}	
}

void uart_test(void)
{
	volatile uint8_t testcase = 1;
    uint8_t testcycle=0,i=0;
				
	switch(testcase)
	{			
		case 1:
			for(uart_sel=UART0;uart_sel<=UART1;uart_sel++)
			{	
				uart_cfg.use_tx_buf = false;
				uart_cfg.use_fifo = true;
				testcycle=2;
				while(testcycle--)	
				{					
					uart_rx_tx(uart_cfg.use_tx_buf,uart_cfg.use_fifo);				
				}  	
											
				uart_cfg.use_tx_buf = true;		    			
				uart_cfg.use_fifo = true;						
				testcycle=2;
				while(testcycle--)	
				{					
					uart_rx_tx(uart_cfg.use_tx_buf,uart_cfg.use_fifo);				
				}
				
				uart_cfg.use_tx_buf = false;		    
				uart_cfg.use_fifo = false;		
				testcycle=2;
				while(testcycle--)
				{					
					uart_rx_tx(uart_cfg.use_tx_buf,uart_cfg.use_fifo);				
				}  								
			}
			
			break;		
		
		default:
			break;
	}
}
#endif
