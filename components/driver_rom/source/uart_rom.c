/*******************************************************************************
* @file     uart.c
* @brief    Contains all functions support for uart driver
* @version  0.0
* @date     19. Oct. 2017
* @author   qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include <string.h>
#include "jump_function.h"
#include "pwrmgr.h"
#include "uart.h"
#include "gpio.h"
#include "error.h"
#include "clock.h"

static comm_cb_t s_uart_cb = NULL;

enum{
    SWU_MODE_NONE = 0,
    SWU_MODE_RX   = 1,
    SWU_MODE_TX   = 2
};
int         s_uart_baudrate = 0;
gpio_pin_e  s_tx_pin = GPIO_DUMMY;
gpio_pin_e  s_rx_pin = GPIO_DUMMY;
gpio_pin_e  s_swu_pin = GPIO_DUMMY;
uint8_t     s_swu_mode = SWU_MODE_NONE;

/*
 * rom use:
 * SWU_*
 * swu_*
*/
void SWU_UART0_IRQHandler(void)
{
    uint8_t data[UART_RX_FIFO_SIZE];
    uint8_t i, len;  
    switch (AP_UART0->IIR & 0xf)
    {
    case RDA_IRQ:
    case TIMEOUT_IRQ:
        len = AP_UART0->RFL;
        for(i = 0; i< len; i++){
            data[i] = (uint8_t)(AP_UART0->RBR & 0xff);
        }
        if(s_uart_cb){
            comm_evt_t ev;
            ev.data = data;
            ev.type = UART_EVT_TYPE_RX_DATA;
            ev.len = (uint32_t)len;
            s_uart_cb(&ev);
        }
        break;
    case BUSY_IRQ:
        (void) AP_UART0 -> USR;
        break;                
    }
}

int swu_uart_init(int baud, gpio_pin_e tx_pin, gpio_pin_e rx_pin, comm_cb_t cb)
{
    int  dll;

    gpio_pull_set(tx_pin, WEAK_PULL_UP);
    gpio_pull_set(rx_pin, WEAK_PULL_UP);
    if(tx_pin == SWU_DUMMY_PIN)
    {
        s_swu_pin = rx_pin;
        s_swu_mode = SWU_MODE_RX;
    }
    else if(rx_pin == SWU_DUMMY_PIN)
    {
        s_swu_pin = tx_pin;
        s_swu_mode = SWU_MODE_TX;
    }
    s_uart_baudrate = baud;
    AP_UART0->LCR =0;
    dll=((clk_get_pclk()>>4)+(baud>>1))/baud;
	if((tx_pin == P16) || (rx_pin == P16))
	{
			AP_IOMUX->Analog_IO_en &= ~BIT(3); 
	}

	if((tx_pin == P17) || (rx_pin == P17))
	{
			AP_IOMUX->Analog_IO_en &= ~BIT(4); 
	}
	
    gpio_fmux_set(tx_pin,FMUX_UART_TX);
    gpio_fmux_set(rx_pin,FMUX_UART_RX);

    //reset clock gate
    AP_PCR->SW_RESET0 &= ~(BIT(MOD_UART0));
    AP_PCR->SW_RESET0 |= BIT(MOD_UART0);

    s_tx_pin = tx_pin;
    s_rx_pin = rx_pin;
    
    AP_UART0->MCR=0x0;
    AP_UART0->LCR=0x80;
	AP_UART0->DLM=(dll&0xFF00)>>8;
    AP_UART0->DLL=(dll&0xFF);  
    //AP_UART0->IER=0x0;
    
    //enable tx FIFO mode(empty trigger), rx FIFO mode(1/2 trigger)
    AP_UART0->FCR= FCR_TX_FIFO_RESET|FCR_RX_FIFO_RESET|FCR_FIFO_ENABLE|UART_FIFO_RX_TRIGGER|UART_FIFO_TX_TRIGGER;

    AP_UART0->LCR = 0x3; //8bit, 1 stop no parity
    AP_UART0->IER=0x0;
    
    //enable Received Data Available Interrupt
    AP_UART0->IER = IER_ERBFI;

//    AP_UART0->IER = IER_ERBFI|IER_PTIME;
    s_uart_cb = cb;
    //enable uart irq
    NVIC_EnableIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, IRQ_PRIO_HAL);

    return PPlus_SUCCESS;
}

int swu_uart_deinit(void)
{
    //disable uart irq
    NVIC_DisableIRQ(UART0_IRQn);

    if(s_tx_pin != GPIO_DUMMY)
        gpio_fmux_control(s_tx_pin,Bit_DISABLE);
    if(s_rx_pin != GPIO_DUMMY)
        gpio_fmux_control(s_rx_pin,Bit_DISABLE);    
    s_tx_pin = GPIO_DUMMY;
    s_rx_pin = GPIO_DUMMY;

    return PPlus_SUCCESS;
}

int swu_uart_tx(const char *data)
{
    int len = strlen(data);
    return swu_uart_send_buff((uint8_t *)data, len);
}

int swu_uart_send_buff(uint8_t *data, int len)
{   
    HAL_WAIT_CONDITION_TIMEOUT(!(AP_UART0->USR & USR_BUSY), 100000);

    while(len--)
    {
        HAL_WAIT_CONDITION_TIMEOUT((AP_UART0->LSR & LSR_THRE), 100000);

        AP_UART0->THR = *data++;
    }
    //wait shift register empty
    HAL_WAIT_CONDITION_TIMEOUT((AP_UART0->LSR & LSR_TEMT), 100000);
    return PPlus_SUCCESS;;
}

int swu_uart_set_baudrate(uint32_t baud)
{
    if(baud)
        s_uart_baudrate= baud;
    uint32_t dll = ((clk_get_pclk()>>4)+(s_uart_baudrate>>1))/s_uart_baudrate;
    AP_UART0->LCR = 0x80;
    AP_UART0->DLL = dll; 
    AP_UART0->LCR = 0x3;
    return PPlus_SUCCESS;
}

void swu_tx_mode(int baud)
{
    //only when now is rx mode, need switch to tx mode
    if(s_swu_mode != SWU_MODE_RX) 
        return;
    s_swu_mode = SWU_MODE_TX;
    swu_uart_deinit();
    if(baud ==0) baud = s_uart_baudrate;
    swu_uart_init(baud, s_swu_pin, SWU_DUMMY_PIN, s_uart_cb);
}

void swu_rx_mode(int baud)
{
    //only when now is tx mode, need switch to rx mode
    if(s_swu_mode != SWU_MODE_TX)
        return;
    s_swu_mode = SWU_MODE_RX;
    swu_uart_deinit();
    if(baud ==0) baud = s_uart_baudrate;
    swu_uart_init(baud, SWU_DUMMY_PIN, s_swu_pin, s_uart_cb);
}

/*
    AP_UART0->SRT & 0x3;  //RCVR Trigger      9c
    AP_UART0->STET & 0x3; //TX enpty trigger  a0
    AP_UART0->SDMAM & 0x1; //DMA MODE         94
    AP_UART0->SFE &0x1;  //enable             98
*/ 
int swu_uart_rxtrigger(uint32_t rxfifo_num)
{
    rxfifo_num = rxfifo_num & 0x3;           
    AP_UART0->FCR = (rxfifo_num<<6)+((AP_UART0->STET&0x3)<<4)+((AP_UART0->SDMAM&0x1)<<3)+FCR_FIFO_ENABLE;
    return PPlus_SUCCESS;
}
int swu_uart_txtrigger(uint32_t txfifo_num)
{
    txfifo_num = txfifo_num & 0x3;           
    AP_UART0->FCR = ((AP_UART0->SRT&0x3)<<6)+(txfifo_num<<4)+((AP_UART0->SDMAM&0x1)<<3)+FCR_FIFO_ENABLE;
    return PPlus_SUCCESS;
}

/*
 * sdk use
 * */
typedef struct _uart_Context
{
    bool          enable;

    uint8_t       tx_state;
    uart_Tx_Buf_t tx_buf;
    uart_Cfg_t    cfg;
} uart_Ctx_t;

static uart_Ctx_t m_uartCtx[2] =
{
    {.enable = FALSE,},
    {.enable = FALSE,},
};

static int txmit_buf_use_tx_buf(UART_INDEX_e uart_index,uint8_t* buf,uint16_t len)
{
    uart_Tx_Buf_t* p_txbuf = &(m_uartCtx[uart_index].tx_buf);
    uint8_t* p_data;
    AP_UART_TypeDef* cur_uart = (AP_UART_TypeDef*) AP_UART0_BASE;

    if(len == 0 || buf == NULL)
        return PPlus_ERR_INVALID_PARAM;

    if(p_txbuf->tx_state == TX_STATE_UNINIT)
        return PPlus_ERR_NO_MEM;

    if(p_txbuf->tx_buf_size < len)
        return PPlus_ERR_NO_MEM;

    if(p_txbuf->tx_state != TX_STATE_IDLE)
    {
        if(p_txbuf->tx_data_size + len > p_txbuf->tx_buf_size)
            return PPlus_ERR_NO_MEM;

        _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
        memcpy(p_txbuf->tx_buf + p_txbuf->tx_data_size, buf, len);
        p_txbuf->tx_data_size += len;
        HAL_EXIT_CRITICAL_SECTION();
        return PPlus_SUCCESS;
    }

    memcpy(p_txbuf->tx_buf, buf, len);
    p_txbuf->tx_data_size = len;
    p_txbuf->tx_data_offset = 0;
    p_txbuf->tx_state = TX_STATE_TX;
    p_data = p_txbuf->tx_buf;
//    len = p_txbuf->tx_data_size;
    len = len > UART_TX_FIFO_SIZE ? UART_TX_FIFO_SIZE : len;

    if(uart_index == UART1)
        cur_uart =   (AP_UART_TypeDef*) AP_UART1_BASE;

    cur_uart->IER &= ~(IER_ETBEI);

    while(len--)
    {
        cur_uart->THR = p_data[p_txbuf->tx_data_offset++];
    }

    if(uart_index == UART0)
        pwrmgr_lock(MOD_UART0);
    else
        pwrmgr_lock(MOD_UART1);

    cur_uart->IER |= IER_ETBEI;
    return PPlus_SUCCESS;
}

static int txmit_buf_polling(UART_INDEX_e uart_index,uint8_t* buf,uint16_t len)
{
    AP_UART_TypeDef* cur_uart = (AP_UART_TypeDef*) AP_UART0_BASE;

    if(uart_index == UART1)
        cur_uart = (AP_UART_TypeDef*) AP_UART1_BASE;

    HAL_WAIT_CONDITION_TIMEOUT(!(cur_uart->USR & USR_BUSY), 100000);

    while(len--)
    {
        HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_THRE), 100000);
        cur_uart->THR = *buf++;
    }

    //wait shift register empty
    HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_TEMT), 100000);
    return PPlus_SUCCESS;;
}

static void irq_rx_handler(UART_INDEX_e uart_index,uint8_t flg)
{
    int i;
    uint8_t data[UART_RX_FIFO_SIZE];
    uint8_t len;
    AP_UART_TypeDef* cur_uart = (AP_UART_TypeDef*)AP_UART0_BASE;

    if(uart_index == UART1)
    {
        cur_uart = (AP_UART_TypeDef*) AP_UART1_BASE;
    }

    if(m_uartCtx[uart_index].cfg.use_fifo)
    {
        len = cur_uart->RFL;

        for(i = 0; i< len; i++)
            data[i] = (uint8_t)(cur_uart->RBR & 0xff);
    }
    else
    {
        len = 1;
        cur_uart->LSR;  //clear interrupt
        data[0] = (uint8_t)(cur_uart->RBR & 0xff);
    }

    if(m_uartCtx[uart_index].cfg.evt_handler)
    {
        uart_Evt_t evt;
        evt.type = flg;
        evt.data = data;
        evt.len = len;
        m_uartCtx[uart_index].cfg.evt_handler(&evt);
    }
}

static void irq_tx_empty_handler(UART_INDEX_e uart_index)
{
    uart_Tx_Buf_t* p_txbuf = &(m_uartCtx[uart_index].tx_buf);
    uint8_t* p_data;
    uint16_t len;
    AP_UART_TypeDef* cur_uart = (AP_UART_TypeDef*)AP_UART0_BASE;

    if(m_uartCtx[uart_index].enable == FALSE)
        return;

    if(m_uartCtx[uart_index].cfg.use_fifo == FALSE)
        return;

    if(m_uartCtx[uart_index].cfg.use_tx_buf == FALSE)
        return;

    if(p_txbuf->tx_state != TX_STATE_TX)
        return;

    p_data = p_txbuf->tx_buf;
    len = p_txbuf->tx_data_size - p_txbuf->tx_data_offset;
    len = len > UART_TX_FIFO_SIZE ? UART_TX_FIFO_SIZE : len;

    if(len == 0)
    {
        p_txbuf->tx_state = TX_STATE_IDLE;
        p_txbuf->tx_data_offset = 0;
        p_txbuf->tx_data_size = 0;

        if(m_uartCtx[uart_index].cfg.evt_handler)
        {
            uart_Evt_t evt =
            {
                .type = UART_EVT_TYPE_TX_COMPLETED,
                .data = NULL,
                .len = 0,
            };
            m_uartCtx[uart_index].cfg.evt_handler(&evt);
        }

        if(UART0 == uart_index)
            pwrmgr_unlock(MOD_UART0);
        else
            pwrmgr_unlock(MOD_UART1);

        return;
    }

    if(uart_index == UART1)
        cur_uart =   (AP_UART_TypeDef*) AP_UART1_BASE;

    while(len--)
    {
        cur_uart->THR = p_data[p_txbuf->tx_data_offset++];
    }
}

extern int uart_hw_deinit(UART_INDEX_e uart_index);
static int uart_hw_init(UART_INDEX_e uart_index)
{
    uart_Cfg_t* pcfg;
    int pclk = clk_get_pclk();
    uint32_t dll;
    
    AP_UART_TypeDef * cur_uart = AP_UART0;
    MODULE_e mod        = MOD_UART0;
    IRQn_Type irq_type  = UART0_IRQn;
    gpio_fmux_e fmux_tx    = FMUX_UART0_TX,     fmux_rx = FMUX_UART0_RX;

	uart_hw_deinit(uart_index);
    if(uart_index== UART1)
	{
        cur_uart    = AP_UART1;
        mod         = MOD_UART1;
        irq_type    = UART1_IRQn;
        fmux_tx     = FMUX_UART1_TX;
        fmux_rx     = FMUX_UART1_RX;
    }
    
    if((m_uartCtx[uart_index].cfg.tx_pin == GPIO_DUMMY) && (m_uartCtx[uart_index].cfg.rx_pin == GPIO_DUMMY))
        return PPlus_ERR_INVALID_PARAM;

    pcfg = &(m_uartCtx[uart_index].cfg);

    clk_gate_enable(mod);
    clk_reset(mod);
  
    gpio_pull_set(pcfg->tx_pin, WEAK_PULL_UP);
    gpio_pull_set(pcfg->rx_pin, WEAK_PULL_UP);
    
    gpio_fmux_set(pcfg->tx_pin, fmux_tx);
    gpio_fmux_set(pcfg->rx_pin, fmux_rx);

    cur_uart->LCR =0;  
    dll = ((pclk>>4)+(pcfg->baudrate>>1))/pcfg->baudrate;
    cur_uart->MCR=0x0;
    cur_uart->LCR=0x80; 
    cur_uart->DLM=(dll & 0xFF00) >> 8;   
    cur_uart->DLL=(dll & 0xFF);   

    if(pcfg->parity)    
        cur_uart->LCR = 0x1b; //8bit, 1 stop even parity    
    else    
        cur_uart->LCR = 0x3;  //8bit, 1 stop no parity    
    
    if(pcfg->use_fifo)//set fifo, enable tx FIFO mode(empty trigger), rx FIFO mode(1/2 trigger)
        //cur_uart->FCR= FCR_TX_FIFO_RESET|FCR_RX_FIFO_RESET|FCR_FIFO_ENABLE|UART_FIFO_RX_TRIGGER|UART_FIFO_TX_TRIGGER;    
        cur_uart->FCR= pcfg->fcr;
    else     
        cur_uart->FCR=0;
        
    //enable Received Data Available Interrupt
    cur_uart->IER = IER_ERBFI;

	if(pcfg->use_fifo)
		cur_uart->IER |= IER_PTIME;
	
    if(pcfg->use_tx_buf)    
        cur_uart->IER |= IER_ETBEI;    
   
    //if(uart_index== UART0)
    //{
    //    JUMP_FUNCTION_SET(UART0_IRQ_HANDLER,(uint32_t)&hal_UART0_IRQHandler) ;
    //}
    //else
    //{
    //     JUMP_FUNCTION_SET(UART1_IRQ_HANDLER,(uint32_t)&hal_UART1_IRQHandler) ;
    //}

    NVIC_SetPriority(irq_type, IRQ_PRIO_HAL);
    NVIC_EnableIRQ(irq_type);
    
    return PPlus_SUCCESS;
}

int uart_hw_deinit(UART_INDEX_e uart_index)
{
	MODULE_e mod = MOD_UART0;
	IRQn_Type irq_type = UART0_IRQn;
	AP_UART_TypeDef * cur_uart = AP_UART0;
	
	if(uart_index== UART1)
	{
		mod         = MOD_UART1;
		irq_type    = UART1_IRQn;
		cur_uart    = AP_UART1;
	}

	NVIC_DisableIRQ(irq_type);
	gpio_fmux_control(m_uartCtx[uart_index].cfg.tx_pin,Bit_DISABLE);
	gpio_fmux_control(m_uartCtx[uart_index].cfg.rx_pin,Bit_DISABLE);
	cur_uart->LCR=0x80; 
	cur_uart->DLM=0;   
	cur_uart->DLL=0;   
	cur_uart->LCR =0;  

	cur_uart->FCR=0;       
	cur_uart->IER = 0;
	
	clk_reset(mod);
	clk_gate_disable(mod);

    //if(uart_index== UART0)
    //{
    //    JUMP_FUNCTION_SET(UART0_IRQ_HANDLER,0);
    //}
    //else
    //{
    //    JUMP_FUNCTION_SET(UART1_IRQ_HANDLER,0);
    //}

	return PPlus_SUCCESS;
}

/**************************************************************************************
    @fn          hal_UART0_IRQHandler

    @brief       This function process for uart interrupt

    input parameters

    @param       None.

    output parameters

    @param       None.

    @return      None.
 **************************************************************************************/
void __attribute__((used)) hal_UART0_IRQHandler(void)
{
    uint8_t IRQ_ID= (AP_UART0->IIR & 0x0f);

    switch(IRQ_ID)
    {
    case TIMEOUT_IRQ:
        irq_rx_handler(UART0,UART_EVT_TYPE_RX_DATA_TO);
        break;

    case RDA_IRQ:
        irq_rx_handler(UART0,UART_EVT_TYPE_RX_DATA);
        break;

    case THR_EMPTY:
        irq_tx_empty_handler(UART0);
        break;

    case RLS_IRQ:
        break;

    case BUSY_IRQ:
        (void)AP_UART0->USR;
        break;

    default:
        break;
    }
}

void __attribute__((used)) hal_UART1_IRQHandler(void)
{
    uint8_t IRQ_ID= (AP_UART1->IIR & 0x0f);

    switch(IRQ_ID)
    {
    case TIMEOUT_IRQ:
        irq_rx_handler(UART1,UART_EVT_TYPE_RX_DATA_TO);
        break;

    case RDA_IRQ:
        irq_rx_handler(UART1,UART_EVT_TYPE_RX_DATA);
        break;

    case THR_EMPTY:
        irq_tx_empty_handler(UART1);
        break;

    case RLS_IRQ:
        break;

    case BUSY_IRQ:
        (void)AP_UART1->USR;
        break;

    default:
        break;
    }
}

static void uart_wakeup_process_0(void)
{
    uart_hw_init(UART0);
}

static void uart_wakeup_process_1(void)
{
    uart_hw_init(UART1);
}

int hal_uart_init(uart_Cfg_t cfg,UART_INDEX_e uart_index)
{
    if(m_uartCtx[uart_index].enable)
        return PPlus_ERR_BUSY;
    
    if(uart_index == UART0 && (JUMP_FUNCTION_GET(UART0_IRQ_HANDLER) == SRAM_BASE_ADDR))
      return PPlus_ERR_NOT_REGISTED;
    if(uart_index == UART1 && (JUMP_FUNCTION_GET(UART1_IRQ_HANDLER) == SRAM_BASE_ADDR))
      return PPlus_ERR_NOT_REGISTED;
    //if(cfg.hw_fwctrl || cfg.parity)
    // return PPlus_ERR_NOT_SUPPORTED;
    if(cfg.hw_fwctrl)
        return PPlus_ERR_NOT_SUPPORTED;

    memset(&(m_uartCtx[uart_index]), 0, sizeof(uart_Ctx_t));
    memcpy(&(m_uartCtx[uart_index].cfg), &cfg, sizeof(uart_Cfg_t));
    uart_hw_init(uart_index);
    m_uartCtx[uart_index].enable = TRUE;

    if(uart_index == UART0)
        pwrmgr_register(MOD_UART0, NULL,  uart_wakeup_process_0);
    else
        pwrmgr_register(MOD_UART1, NULL,  uart_wakeup_process_1);

    return PPlus_SUCCESS;
}

int hal_uart_deinit(UART_INDEX_e uart_index)
{
    uart_hw_deinit(uart_index);
    memset(&(m_uartCtx[uart_index]), 0, sizeof(uart_Ctx_t));
    m_uartCtx[uart_index].enable = FALSE;

    if(uart_index == UART0)
        pwrmgr_unregister(MOD_UART0);
    else
        pwrmgr_unregister(MOD_UART1);

    return PPlus_SUCCESS;
}

int hal_uart_set_tx_buf(UART_INDEX_e uart_index,uint8_t* buf, uint16_t size)
{
    uart_Tx_Buf_t* p_txbuf = &(m_uartCtx[uart_index].tx_buf);

    if(m_uartCtx[uart_index].enable == FALSE)
        return PPlus_ERR_INVALID_STATE;

    if(m_uartCtx[uart_index].cfg.use_tx_buf == FALSE)
        return PPlus_ERR_NOT_SUPPORTED;

    if(p_txbuf->tx_state != TX_STATE_UNINIT)
        return PPlus_ERR_INVALID_STATE;

    _HAL_CS_ALLOC_(); HAL_ENTER_CRITICAL_SECTION();
    p_txbuf->tx_buf = buf;
    p_txbuf->tx_buf_size = size;
    p_txbuf->tx_data_offset = 0;
    p_txbuf->tx_data_size= 0;
    p_txbuf->tx_state = TX_STATE_IDLE;
    HAL_EXIT_CRITICAL_SECTION();
	
    return PPlus_SUCCESS;
}

int hal_uart_get_tx_ready(UART_INDEX_e uart_index)
{
    if(m_uartCtx[uart_index].cfg.use_tx_buf == FALSE)
        return PPlus_SUCCESS;

    if(m_uartCtx[uart_index].tx_buf.tx_state == TX_STATE_IDLE)
        return PPlus_SUCCESS;

    return PPlus_ERR_BUSY;
}

int hal_uart_send_buff(UART_INDEX_e uart_index,uint8_t* buff,uint16_t len)
{
    if(m_uartCtx[uart_index].cfg.use_tx_buf)
    {
        return txmit_buf_use_tx_buf(uart_index,buff,len);
    }

    return txmit_buf_polling(uart_index,buff,len);
}

int hal_uart_send_byte(UART_INDEX_e uart_index,unsigned char data)
{
    AP_UART_TypeDef* cur_uart = (AP_UART_TypeDef*) AP_UART0_BASE;

    if(uart_index == UART1)
        cur_uart =   (AP_UART_TypeDef*) AP_UART1_BASE;

    HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_THRE), 10000);
    cur_uart->THR=data;
    HAL_WAIT_CONDITION_TIMEOUT((cur_uart->LSR & LSR_TEMT), 10000);
	
    return PPlus_SUCCESS;
}
