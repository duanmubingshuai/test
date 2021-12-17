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

/**************************************************************************************************
    Filename:       gpio_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "rom_sym_def.h"
#include "OSAL.h"
#include "cliPeripheral.h"
#include "log.h"

#include "gpio.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "global_config.h"
#include "cliface.h"
#include "pwm.h"
#include "kscan.h"
#include "dma.h"
#include "spi.h"
#include "flash.h"
#include "spiflash.h"

#define BUF_LEN 512
const char* s_str_xmit = "this string is for test spi master and slave xmit, loop count =  ";
uint8_t s_spi_tx_buf[BUF_LEN];
uint8_t s_spi_rx_buf[BUF_LEN];
uint32_t s_spi_rx_offset=0;


uint8 key_TaskID;


uint8_t cmdstr[64];
uint8_t cmdlen;

uint8_t g_dma_src_buffer_u8[0x400*2];
uint8_t g_dma_dst_buffer_u8[0x400*2];

hal_spi_t s_spi =
{
    .spi_index = SPI0,
};
spi_Cfg_t s_spim_cfg =
{
    .sclk_pin = GPIO_P34,
    .ssn_pin = GPIO_P31,
    .MOSI = GPIO_P32,
    .MISO = GPIO_P33,

    .baudrate = 500000,//1M
    .spi_tmod = SPI_TRXD,
    .spi_scmod = SPI_MODE3,

    .int_mode = false,
    .force_cs = false,
    .evt_handler = NULL,
};


volatile int s_tmp = 1234;

uint16_t cli_demo_help(uint32_t argc, uint8_t* argv[]);
uint16_t cli_demo_pwm_test(uint32_t argc, uint8_t* argv[]);
uint16_t cli_demo_kscan_test(uint32_t argc, uint8_t* argv[]);
uint16_t cli_demo_dma_test(uint32_t argc, uint8_t* argv[]);
uint16_t cli_demo_flash_test(uint32_t argc, uint8_t* argv[]);
uint16_t cli_demo_spif_flash_test(uint32_t argc, uint8_t* argv[]);



const CLI_COMMAND cli_cmd_list[] =
{
    /* Help */
    { "help", "Help on this CLI Demo Menu", cli_demo_help },

    /* PWM Test */
    { "pwm", "Start pwm control test", cli_demo_pwm_test },

    /* KSCAN Test */
    { "kscan", "Start kscan control test", cli_demo_kscan_test },

    /* DMA Test */
    { "dma", "Start dma test", cli_demo_dma_test },

    /* flash Test */
    { "flash", "Start flash test", cli_demo_flash_test },

    /* spif flash(extern flash) Test */
    { "spiflash", "Start spif flash test", cli_demo_spif_flash_test },
};

#if 1 //32pin
// KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P18,KEY_ROW_P07,KEY_ROW_P23,KEY_ROW_P15};
// KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P33,KEY_COL_P11,KEY_COL_P14,KEY_COL_P16};
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P18,KEY_ROW_P07,KEY_ROW_P23,KEY_ROW_P15};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P33,KEY_COL_P11,KEY_COL_P14,KEY_COL_P26};
#else //48 pin
KSCAN_ROWS_e rows[NUM_KEY_ROWS] = {KEY_ROW_P00,KEY_ROW_P02,KEY_ROW_P25,KEY_ROW_P18};
KSCAN_COLS_e cols[NUM_KEY_COLS] = {KEY_COL_P01,KEY_COL_P03,KEY_COL_P24,KEY_COL_P20};
#endif


static void kscan_evt_handler(kscan_Evt_t* evt)
{
    LOG("kscan_evt_handler\n");
    LOG("num: ");
    LOG("%d",evt->num);
    LOG("\n");

    for(uint8_t i=0; i<evt->num; i++)
    {
        LOG("index: ");
        LOG("%d",i);
        LOG(",row: ");
        LOG("%d",evt->keys[i].row);
        LOG(",col: ");
        LOG("%d",evt->keys[i].col);
        LOG(",type: ");
        LOG("%s",evt->keys[i].type == KEY_PRESSED ? "pressed":"released");
        LOG("\n");
    }
}

static void ProcessDMAUartData(uart_Evt_t* evt)
{
    if(evt->len)
    {
        for(uint8_t i=0; i<evt->len; i++)
        {
            LOG("%x\n",evt->data[i]);
        }

//        osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
//        cmdlen += evt->len;
//        osal_set_event( key_TaskID, KEY_DEMO_UART_RX_EVT );
    }
}

void dma_test_uart_init(void)
{
    uart_Cfg_t cfg =
    {
        .tx_pin = P14,
        .rx_pin = P15,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = ProcessDMAUartData,
    };
    hal_uart_init(cfg,UART1);//uart init
}

static void ProcessUartData(uart_Evt_t* evt)
{
    if(evt->len)
    {
        osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
        cmdlen += evt->len;
        osal_set_event( key_TaskID, KEY_DEMO_UART_RX_EVT );
    }
}

void peripheral_uart_init(void)
{
    uart_Cfg_t cfg =
    {
        .tx_pin = P9,
        .rx_pin = P10,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = ProcessUartData,
    };
    hal_uart_init(cfg,UART0);//uart init
}

void Key_Demo_Init(uint8 task_id)
{
//  uint8_t i = 0;
//  uint32_t ret;
    key_TaskID = task_id;
    hal_uart_deinit(UART0);
    peripheral_uart_init();
    LOG("gpio key demo start...\n");
    osal_set_event(key_TaskID, START_DEVICE_EVT);
    osal_start_timerEx(key_TaskID, KEY_DEMO_ONCE_TIMER, 50);
    memset(g_dma_dst_buffer_u8, 0, sizeof(g_dma_dst_buffer_u8));
}
uint32_t s_tmp_cnt;
uint16 Key_ProcessEvent( uint8 task_id, uint16 events )
{
    if(task_id != key_TaskID)
    {
        return 0;
    }

    if( events & START_DEVICE_EVT)
    {
        s_tmp ++;
        return (events ^ START_DEVICE_EVT);
    }

    if( events & KEY_DEMO_ONCE_TIMER)
    {
        LOG("%d\n", s_tmp_cnt++);
        osal_start_timerEx( key_TaskID, KEY_DEMO_ONCE_TIMER, 500);
        return (events ^ KEY_DEMO_ONCE_TIMER);
    }

    if( events & KEY_DEMO_UART_RX_EVT)
    {
        if ('\r' == cmdstr[cmdlen - 1])
        {
            cmdstr[cmdlen - 1] = '\0';
            LOG("%s", cmdstr);
            CLI_process_line
            (
                cmdstr,
                cmdlen,
                (CLI_COMMAND*) cli_cmd_list,
                (sizeof (cli_cmd_list)/sizeof(CLI_COMMAND))
            );
            cmdlen = 0;
        }

        return (events ^ KEY_DEMO_UART_RX_EVT);
    }

    if ( events & KSCAN_WAKEUP_TIMEOUT_EVT )
    {
        // Perform kscan timeout task
        hal_kscan_timeout_handler();
        return (events ^ KSCAN_WAKEUP_TIMEOUT_EVT);
    }

    // Discard unknown events
    return 0;
}

uint16_t cli_demo_help(uint32_t argc, uint8_t* argv[])
{
    uint32_t index;
    LOG("\r\nCLI Demo\r\n");

    /* Print all the available commands */
    for (index = 0; index < (sizeof (cli_cmd_list)/sizeof(CLI_COMMAND)); index++)
    {
        LOG("    %s: %s\n",
            cli_cmd_list[index].cmd,
            cli_cmd_list[index].desc);
    }

    return 0;
}
uint16_t cli_demo_kscan_test(uint32_t argc, uint8_t* argv[])
{
    LOG("kscan start\n");
    kscan_Cfg_t cfg;
    cfg.ghost_key_state = NOT_IGNORE_GHOST_KEY;
    cfg.key_rows = rows;
    cfg.key_cols = cols;
    cfg.interval = 50;
    cfg.evt_handler = kscan_evt_handler;
    pGlobal_config[MAX_SLEEP_TIME] = 100000000;
    hal_kscan_init(cfg, key_TaskID, KSCAN_WAKEUP_TIMEOUT_EVT);
    return 0;
}


uint16_t cli_demo_pwm_test(uint32_t argc, uint8_t* argv[])
{
    uint16_t dutyvalue,cnttopvalue;

    if (2 == argc)
    {
        dutyvalue = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        LOG("duty value (16-bit in HEX): 0x%04X\n", dutyvalue);
        cnttopvalue = CLI_strtoi(argv[1], CLI_strlen(argv[1]), 16);
        LOG("cnttop value (16-bit in HEX): 0x%04X\n", cnttopvalue);
        hal_pwm_init(PWM_CH5, PWM_CLK_DIV_128, PWM_CNT_UP, PWM_POLARITY_FALLING);
        hal_pwm_set_count_val(PWM_CH5, dutyvalue, cnttopvalue);
        hal_pwm_open_channel(PWM_CH5, P11);
        hal_pwm_init(PWM_CH4, PWM_CLK_DIV_128, PWM_CNT_UP, PWM_POLARITY_RISING);
        hal_pwm_set_count_val(PWM_CH4, dutyvalue, cnttopvalue);
        hal_pwm_open_channel(PWM_CH4, P7);
        hal_pwm_start();
    }
    else
    {
        LOG("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return 0xffff;
    }

    return 0;
}



void process_rx(uint8_t* data, uint32_t size)
{
//  if(s_spi_rx_offset == 0){
//      if(data[0] != 0x5a){
//          LOG("!");
//          return;
//      }
//  }
    memcpy(s_spi_rx_buf+s_spi_rx_offset, data, size);
    s_spi_rx_offset += size;

    if(s_spi_rx_offset >=0x08)
    {
        s_spi_rx_offset=0;

        for(uint16_t i=0; i<0x08; i++)
        {
            LOG("%02x",s_spi_rx_buf[i]);
        }

        LOG("\n");
    }

//  if(s_spi_rx_offset >=3 && s_framesize == 0){
//      s_framesize = s_spi_rx_buf[1] + s_spi_rx_buf[2]*0x100;
//  }
//
//  if(s_framesize && s_spi_rx_offset >= s_framesize)
//  {
//      osal_set_event(spi_TaskID, RX_GET_FRAME_EVT);
//      s_framesize_evt = s_framesize;
//      s_framesize = 0;
//      s_spi_rx_offset = 0;
//  }
}

void spis_cb(spi_evt_t* evt)
{
    switch(evt->evt)
    {
    case SPI_TX_COMPLETED:
//    osal_set_event(spi_TaskID,TX_COMPLETED_EVT );
        break;

    case SPI_RX_COMPLETED:
//    osal_set_event(spi_TaskID,RX_COMPLETED_EVT );
        break;

    case SPI_RX_DATA_S:
        process_rx(evt->data,evt->len);
        break;
    }

    //LOG("(SPI finish:%d %d)   ",evt.spi,evt.type);
}


spi_Cfg_t s_spis_cfg =
{
    .sclk_pin = GPIO_P34,
    .ssn_pin = GPIO_P31,
    .MOSI = GPIO_P32,
    .MISO = GPIO_P33,

    .baudrate = 500000,//2M
    .spi_tmod = SPI_TRXD,
    .spi_scmod = SPI_MODE3,

    .int_mode = false,
    .force_cs = true,
    .evt_handler = spis_cb,
};


void hal_SPIF_IRQHandler(void)
{
    uint32_t int_status;
    int_status = AP_SPIF->int_status;

    if(int_status & BIT(2))
    {
        LOG("Done\n");
    }
}

extern int clk_spif_ref_clk(sysclk_t spif_ref_sel);

uint32_t debug_idx=0;

void dma_cb(void)
{
    LOG("dma done!!!\n");
}

//extern void SPI_slave_init_DMA(AP_SSI_TypeDef *Ssix);

uint16_t cli_demo_dma_test(uint32_t argc, uint8_t* argv[])
{
    uint8_t mode;
    uint16_t   i;
    DMA_CH_CFG_t cfgc;
    uint32_t cb = AP_PCR->CACHE_BYPASS;
    uint32_t value;
    LOG("dma start\n");

    if (1 == argc)
    {
        mode = CLI_strtoi(argv[0], CLI_strlen(argv[0]), 16);
        hal_clk_gate_enable(MOD_DMA);

        switch(mode)
        {
        case 0:
            LOG("memory to memory\n");
//                clk_spif_ref_clk(SYS_CLK_XTAL_16M);
//                uint32_t debug_spif_indi[16];
            memset(g_dma_src_buffer_u8, 0, sizeof(g_dma_src_buffer_u8));
            memset(g_dma_dst_buffer_u8, 0xff, sizeof(g_dma_dst_buffer_u8));

            for(i = 0; i< 0x100; i++)
                g_dma_src_buffer_u8[i] = (uint8_t)(0xff-i);

            //bypass cache
            if(cb == 0)
                AP_PCR->CACHE_BYPASS = 1;

            #if 1
            NVIC_SetPriority((IRQn_Type)SPIF_IRQn, IRQ_PRIO_HAL);
            NVIC_EnableIRQ((IRQn_Type)SPIF_IRQn);
            AP_SPIF->wr_protection = 0;
            AP_SPIF->read_instr = 0x801003b;
//                AP_SPIF->sram_part = 0xff;
            AP_SPIF->int_mask = 0x04;
            AP_SPIF->indirect_ahb_addr_trig = 0x80000;
//                AP_SPIF->indirect_rd_start_addr = 0x50000;
//                AP_SPIF->indirect_rd_num = 0x400;
//
//                AP_SPIF->indirect_ahb_trig_addr_range = 0x0f;
//                AP_SPIF->indirect_rd = 0x1;
            AP_SPIF->indirect_wr_start_addr = 0x50000;
            AP_SPIF->indirect_wr_cnt = 0x400;
            AP_SPIF->indirect_ahb_trig_addr_range = 0x0f;
            AP_SPIF->indirect_wr = 0x1;
//                uint32_t count=500;
//                while(count!=0)
//                {
//                    count--;
//                }
            #endif

            if(cb == 0)
                AP_PCR->CACHE_BYPASS = 0;

            cfgc.transf_size = 0x100;
            cfgc.sinc = DMA_INC_INC;
            cfgc.src_tr_width = DMA_WIDTH_WORD;
            cfgc.src_msize = DMA_BSIZE_256;
            cfgc.src_addr = (uint32_t)g_dma_src_buffer_u8;
            cfgc.src_conn = DMA_CONN_MEM;
            cfgc.dinc = DMA_INC_INC;
            cfgc.dst_tr_width = DMA_WIDTH_WORD;
            cfgc.dst_msize = DMA_BSIZE_256;
            cfgc.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
            cfgc.dst_conn = DMA_CONN_MEM;
            cfgc.lli = 0;
            cfgc.enable_int = false;
            cfgc.evt_handler = NULL;
//                DMA_CH_CFG_t cfg= {
//                    .transf_size = 0x100,
//                    .sinc= DMA_INC_INC,
//                    .src_tr_width = DMA_WIDTH_WORD,
//                    .src_msize=DMA_BSIZE_256,
//                  .src_addr = (uint32_t)g_dma_src_buffer_u8,
//                  .src_conn = DMA_CONN_MEM,
//
//                  .dinc = DMA_INC_INC,
//                    .dst_tr_width = DMA_WIDTH_WORD,
//                    .dst_msize=DMA_BSIZE_256,
//                  .dst_addr = (uint32_t)g_dma_dst_buffer_u8,
//                  .dst_conn = DMA_CONN_MEM,
//
//                  .lli = 0,
//
//                  .enable_int = false,
//                  .evt_handler = NULL,
//                };
//                LOG("%08X\n",cfg.src_addr);
//                LOG("%08X\n",cfg.dst_addr);
//                LOG("%08X\n",(uint32_t)g_dma_src_buffer_u8);
//                LOG("%08X\n",(uint32_t)g_dma_dst_buffer_u8);
            hal_dma_config_channel(DMA_CH_0,&cfgc);
            hal_dma_start_channel(DMA_CH_0);
            hal_dma_wait_channel_complete(DMA_CH_0);
//                hal_dma_cfg_ch(0, (uint32_t)g_dma_src_buffer_u8, (uint32_t)g_dma_dst_buffer_u8,0, 0x100);
            AP_SPIF->wr_protection = 2;
//                hal_gpio_write(P14,1);
//                hal_gpio_write(P14,0);
//                hal_dma_cfg_ch(0, (uint32_t)0x11080000, (uint32_t)g_dma_dst_buffer_u8,0, 0x100);
//                hal_dma_cfg_ch(0, (uint32_t)0x11040000, (uint32_t)g_dma_dst_buffer_u8,0, 0x20);
            break;

        case 1:
//                AP_SSI_TypeDef *Ssix = AP_SPI0;
            LOG("memory to SPI0\n");
            memset(g_dma_src_buffer_u8, 0, sizeof(g_dma_src_buffer_u8));

//                memset(g_dma_dst_buffer_u8, 0, sizeof(g_dma_dst_buffer_u8));
            for(i = 0; i< 0x200; i++)
                g_dma_src_buffer_u8[i] = (uint8_t)(0x1ff-i);

            hal_spi_init(SPI1);
            hal_spi_bus_init(&s_spi,s_spim_cfg );
//                hal_spi_transmit(&s_spi, g_dma_src_buffer_u8, NULL, 16);
            AP_SPI1->DMACR = 2;
            AP_SPI1->DMATDLR = 2;
//                uint32_t g_dma_src_buffer_u32[4] = {0x12345678,0x22345678,0x32345678,0x42345678};
//                hal_dma_cfg_ch(0, (uint32_t)g_dma_src_buffer_u8, 0x40007060, 1, 128);
            break;

        case 2:
            LOG("UART1 to memory\n");
            AP_UART1->FCR= FCR_TX_FIFO_RESET|FCR_RX_FIFO_RESET|FCR_FIFO_ENABLE|UART_FIFO_RX_TRIGGER|UART_FIFO_TX_TRIGGER;
//                memset(g_dma_src_buffer_u8, 0, sizeof(g_dma_src_buffer_u8));
            memset(g_dma_dst_buffer_u8, 0, sizeof(g_dma_dst_buffer_u8));
//                for(i = 0; i< 100; i++)
//                    g_dma_src_buffer_u8[i] = (uint8_t)(0xaa-i);
            dma_test_uart_init();
//                AP_SPI0->DMACR = 2;
//                AP_SPI0->DMATDLR = 2;
            cfgc.transf_size = 0x100;
            cfgc.sinc = DMA_INC_NCHG;
            cfgc.src_tr_width = DMA_WIDTH_BYTE;
            cfgc.src_msize = DMA_BSIZE_1;
            cfgc.src_addr = (uint32_t)0x40009000;
            cfgc.src_conn = DMA_CONN_UART1_Rx;
            cfgc.dinc = DMA_INC_INC;
            cfgc.dst_tr_width = DMA_WIDTH_BYTE;
            cfgc.dst_msize = DMA_BSIZE_1;
            cfgc.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
            cfgc.dst_conn = DMA_CONN_MEM;
            cfgc.lli = 0;
            cfgc.enable_int = true;
            cfgc.evt_handler = dma_cb;
//                uint32_t g_dma_src_buffer_u32[4] = {0x12345678,0x22345678,0x32345678,0x42345678};
//                hal_dma_cfg_ch(0, 0x40009000, (uint32_t)g_dma_dst_buffer_u8, 2, 256);
            break;

        case 3:
            LOG("memory to UART1\n");
//                dma_test_uart_init();
//                AP_UART1->THR = 0x30;
//                AP_UART1->THR = 0x31;
            memset(g_dma_src_buffer_u8, 0, sizeof(g_dma_src_buffer_u8));

//                memset(g_dma_dst_buffer_u8, 0, sizeof(g_dma_dst_buffer_u8));
            for(i = 0; i< 100; i++)
                g_dma_src_buffer_u8[i] = (uint8_t)((0x7e)-i);

            cfgc.transf_size = 0x20;
            cfgc.sinc = DMA_INC_INC;
            cfgc.src_tr_width = DMA_WIDTH_BYTE;
            cfgc.src_msize = DMA_BSIZE_1;
            cfgc.src_addr = (uint32_t)g_dma_src_buffer_u8;
            cfgc.src_conn = DMA_CONN_MEM;
            cfgc.dinc = DMA_INC_NCHG;
            cfgc.dst_tr_width = DMA_WIDTH_BYTE;
            cfgc.dst_msize = DMA_BSIZE_1;
            cfgc.dst_addr = (uint32_t)0x40004000;
            cfgc.dst_conn = DMA_CONN_UART0_Tx;
            cfgc.lli = 0;
            cfgc.enable_int = true;
            cfgc.evt_handler = dma_cb;
            hal_dma_config_channel(DMA_CH_0,&cfgc);
            hal_dma_start_channel(DMA_CH_0);

            if(cfgc.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
            }

//                hal_dma_cfg_ch(0, (uint32_t)g_dma_src_buffer_u8, 0x40004000, 3, 32);
            break;

        case 4:
            LOG("UART1 to UART0\n");
            dma_test_uart_init();
            AP_UART1->FCR= FCR_TX_FIFO_RESET|FCR_RX_FIFO_RESET|FCR_FIFO_ENABLE|UART_FIFO_RX_TRIGGER|UART_FIFO_TX_TRIGGER;
//                dma_test_uart_init();
//                AP_UART1->THR = 0x30;
//                AP_UART1->THR = 0x31;
//                memset(g_dma_src_buffer_u8, 0, sizeof(g_dma_src_buffer_u8));
//                memset(g_dma_dst_buffer_u8, 0, sizeof(g_dma_dst_buffer_u8));
//                for(i = 0; i< 100; i++)
//                    g_dma_src_buffer_u8[i] = (uint8_t)(0x7e-i);
//                hal_dma_cfg_ch(0, 0x40009000, 0x40009000, 4, 32);
            break;

        case 5:
//                AP_SSI_TypeDef *Ssix = AP_SPI0;
            LOG("SPI1 to memory\n");
            cfgc.transf_size = 0x10;
            cfgc.sinc = DMA_INC_NCHG;
            cfgc.src_tr_width = DMA_WIDTH_BYTE;
            cfgc.src_msize = DMA_BSIZE_4;
            cfgc.src_addr = (uint32_t)0x40006060;
            cfgc.src_conn = DMA_CONN_SPI0_Rx;
            cfgc.dinc = DMA_INC_INC;
            cfgc.dst_tr_width = DMA_WIDTH_WORD;
            cfgc.dst_msize = DMA_BSIZE_256;
            cfgc.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
            cfgc.dst_conn = DMA_CONN_MEM;
            cfgc.lli = 0;
            cfgc.enable_int = true;
            cfgc.evt_handler = dma_cb;
            hal_spis_clear_rx(&s_spi);
            hal_spi_bus_deinit(&s_spi);
            hal_spi_init(SPI0);
            hal_spis_bus_init(&s_spi,s_spis_cfg );
            AP_SPI0->DMACR = 1;
            AP_SPI0->DMARDLR = 0;
            hal_dma_config_channel(DMA_CH_0,&cfgc);
            hal_dma_start_channel(DMA_CH_0);

            if(cfgc.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
            }

            break;

        case 6:
            LOG("SPI1 to UART\n");
            cfgc.transf_size = 0x20;
            cfgc.sinc = DMA_INC_NCHG;
            cfgc.src_tr_width = DMA_WIDTH_BYTE;
            cfgc.src_msize = DMA_BSIZE_1;
            cfgc.src_addr = (uint32_t)0x40006060;
            cfgc.src_conn = DMA_CONN_SPI0_Rx;
            cfgc.dinc = DMA_INC_NCHG;
            cfgc.dst_tr_width = DMA_WIDTH_BYTE;
            cfgc.dst_msize = DMA_BSIZE_8;
            cfgc.dst_addr = (uint32_t)0x40004000;
            cfgc.dst_conn = DMA_CONN_UART0_Tx;
            cfgc.lli = 0;
            cfgc.enable_int = false;
            cfgc.evt_handler = dma_cb;
            hal_spis_clear_rx(&s_spi);
            hal_spi_bus_deinit(&s_spi);
            hal_spi_init(SPI0);
            hal_spis_bus_init(&s_spi,s_spis_cfg );
            AP_SPI0->DMACR = 1;
            AP_SPI0->DMARDLR = 0;
            hal_dma_config_channel(DMA_CH_0,&cfgc);
            hal_dma_start_channel(DMA_CH_0);

            if(cfgc.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
            }

            break;

        case 7:
            LOG("SPI0 Master to memory\n");
            cfgc.transf_size = 0x08;
            cfgc.sinc = DMA_INC_NCHG;
            cfgc.src_tr_width = DMA_WIDTH_BYTE;
            cfgc.src_msize = DMA_BSIZE_1;
            cfgc.src_addr = (uint32_t)0x40006060;
            cfgc.src_conn = DMA_CONN_SPI0_Rx;
            cfgc.dinc = DMA_INC_INC;
            cfgc.dst_tr_width = DMA_WIDTH_BYTE;
            cfgc.dst_msize = DMA_BSIZE_4;
            cfgc.dst_addr = (uint32_t)g_dma_dst_buffer_u8;
            cfgc.dst_conn = DMA_CONN_MEM;
            cfgc.lli = 0;
            cfgc.enable_int = true;
            cfgc.evt_handler = dma_cb;
            hal_spis_clear_rx(&s_spi);
            hal_spi_bus_deinit(&s_spi);
            hal_spi_init(SPI0);
            hal_spi_bus_init(&s_spi,s_spim_cfg );
            AP_SPI0->DMACR = 1;
            AP_SPI0->DMARDLR = 0;
            hal_dma_config_channel(DMA_CH_0,&cfgc);
            hal_dma_start_channel(DMA_CH_0);
            GD25_read(4,(uint8_t*)&value,4);

            if(cfgc.enable_int == false)
            {
                hal_dma_wait_channel_complete(DMA_CH_0);
            }

            break;

        default:
            break;
        }
    }
    else
    {
        LOG("Invalid Number of Arguments:0x%04X. Returning.\n", argc);
        return 0xffff;
    }

    return 0;
}

uint16_t cli_demo_flash_test(uint32_t argc, uint8_t* argv[])
{
    LOG("flash start\n");
    uint32_t i;

    for(i = 0; i< 0x100; i++)
        g_dma_src_buffer_u8[i] = (uint8_t)((0xff)-i);

//  flash_write_word(0x5000,g_dma_src_buffer_u8,0x100);
//  ReadFlash(0x5000,g_dma_dst_buffer_u8,0x100);
//  flash_sector_erase(0x5000);
    return 0;
}


uint16_t cli_demo_spif_flash_test(uint32_t argc, uint8_t* argv[])
{
    uint16_t i;
    uint32_t value;
    LOG("spif flash start\n");

    for(i = 0; i< 0x100; i++)
        g_dma_src_buffer_u8[i] = (uint8_t)((0xff)-i);

//    hal_spi_init(SPI0);
//    hal_spi_bus_init(&s_spi,s_spim_cfg );
    GD25_init();
    GD25_erase(0,0x200000);
    GD25_write(0,(uint8_t*)g_dma_src_buffer_u8,0xff);
    GD25_read(4,(uint8_t*)&value,4);
    LOG("vaue %x\n",value);
//    hal_spi_bus_deinit(&s_spi);
    return 0;
}



/*********************************************************************
*********************************************************************/
