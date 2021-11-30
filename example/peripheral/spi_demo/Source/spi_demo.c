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

/*********************************************************************
    INCLUDES
*/
#include "rom_sym_def.h"
#include "OSAL.h"
#include "log.h"
#include "gpio.h"
#include "clock.h"

#include "spi.h"
#include "error.h"
#include "spi_demo.h"
#include "pwrmgr.h"
#include <stdlib.h>
#include <string.h>

static uint8 spi_TaskID;


void on_uart_evt(uart_Evt_t* pev)
{
    osal_set_event(spi_TaskID, UART_INPUT_EVT);
}

void spis_cb(spi_evt_t* evt)
{
    switch(evt->evt)
    {
    case SPI_TX_COMPLETED:
    case SPI_RX_COMPLETED:
        osal_set_event(spi_TaskID,TX_COMPLETED_EVT );
    }

    //LOG("(SPI finish:%d %d)   ",evt.spi,evt.type);
}

hal_spi_t spi =
{
    .spi_index = SPI0,
};

spi_Cfg_t s_spim_cfg =
{
    .sclk_pin = GPIO_P34,
    .ssn_pin = GPIO_P31,
    .MOSI = GPIO_P32,
    .MISO = GPIO_P33,

    .baudrate = 1000000,//2M
    .spi_tmod = SPI_TRXD,
    .spi_scmod = SPI_MODE0,

    .int_mode = false,
    .force_cs = true,
    .evt_handler = NULL,
};

spi_Cfg_t s_spis_cfg =
{
    .sclk_pin = GPIO_P34,
    .ssn_pin = GPIO_P31,
    .MOSI = GPIO_P32,
    .MISO = GPIO_P33,

    .baudrate = 1000000,//2M
    .spi_tmod = SPI_TRXD,
    .spi_scmod = SPI_MODE0,

    .int_mode = true,
    .force_cs = true,
    .evt_handler = spis_cb,
};


#define BUF_LEN 512
const char* s_str_xmit = "this string is for test spi master and slave xmit, loop count =  ";
uint8_t s_spi_tx_buf[BUF_LEN];
uint8_t s_spi_rx_buf[BUF_LEN];
uint8_t s_test_cnt = 0;

hal_spi_t s_spi =
{
    .spi_index = SPI0,
};



void spi_demo_Init( uint8 task_id )
{
    spi_TaskID = task_id;
    #if SPI_MASTER_TEST
    hal_spi_bus_init(&s_spi, s_spim_cfg);
    #else
    hal_spis_bus_init(&s_spi, s_spis_cfg);
    #endif
    osal_set_event( spi_TaskID, START_DEVICE_EVT);
}

static uint8_t s_u8_cnt = 0;

static int s_cnt = 0;
uint16 spi_demo_ProcessEvent( uint8 task_id, uint16 events )
{
    int ret,i;

    if (events & UART_INPUT1_EVT )
    {
        //master: recieve string
        //slave: send string
        memset(s_spi_rx_buf, 0, sizeof(s_spi_rx_buf));
        strcpy((char*)s_spi_tx_buf, s_str_xmit);
        s_spi_tx_buf[strlen(s_str_xmit)-1] = 'A' + (s_cnt++)%16;
        #if DMAC_USE
        ret = hal_spi_transmit(&s_spi,SPI_TRXD,0,0, s_spi_tx_buf, s_spi_rx_buf, strlen(s_str_xmit)+1,strlen(s_str_xmit)+1);
        #else
        ret = hal_spi_transmit(&s_spi,SPI_TRXD, s_spi_tx_buf, s_spi_rx_buf, strlen(s_str_xmit)+1,strlen(s_str_xmit)+1);
        #endif

        if(ret == PPlus_SUCCESS)
        {
            s_spi_rx_buf[strlen(s_str_xmit)+1] = '\0';
            LOG("[M Rx]%s\n", s_spi_rx_buf);
        }

        return (events ^ UART_INPUT1_EVT);
    }

    if (events & UART_INPUT_EVT )
    {
        //master: recieve string
        //slave: send string
        uint16_t crc = 0;
        memset(s_spi_tx_buf, 0, sizeof(s_spi_rx_buf));
        s_spi_tx_buf[0] = 0x5a;
        s_spi_tx_buf[1] = 0;
        s_spi_tx_buf[2] = 0x01;
        s_spi_tx_buf[3] = s_u8_cnt;

        for(i = 6; i< 0x100; i++)
        {
            //s_spi_tx_buf[i] = (uint8)rand();
            s_spi_tx_buf[i] = i;
        }

        //crc = crc16(0, s_spi_tx_buf + 6, 0x100-6);
        for(i = 6; i< 0x100; i++)
        {
            crc += s_spi_tx_buf[i];
        }

        s_spi_tx_buf[4] = (uint8_t)(crc&0xff);
        s_spi_tx_buf[5] = (uint8_t)((crc>>8)&0xff);
        #if DMAC_USE
        ret = hal_spi_transmit(&s_spi,SPI_TRXD,0,0, s_spi_tx_buf, NULL, 0x100,0x100);
        #else
        ret = hal_spi_transmit(&s_spi,SPI_TRXD, s_spi_tx_buf, NULL, 0x100,0x100);
        #endif

        if(ret == PPlus_SUCCESS)
        {
            LOG("[Tx]%d\n", s_u8_cnt++);
        }

        osal_start_timerEx(spi_TaskID,UART_INPUT_EVT,1000 );
        return (events ^ UART_INPUT_EVT);
    }

    if (events & RX_TIMEOUT_EVT )
    {
        return (events ^ RX_TIMEOUT_EVT);
    }

    if (events & TX_COMPLETED_EVT)
    {
        LOG("EVT: %s", s_spi_rx_buf);
        return (events ^ TX_COMPLETED_EVT);
    }

    return 0;
}
