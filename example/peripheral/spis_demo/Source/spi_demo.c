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
#include "touch_key.h"
#include <stdlib.h>
#include <string.h>

static uint8 spi_TaskID;


void on_touch_evt(touch_evt_t key_evt)
{
    osal_set_event(spi_TaskID, TOUCH_PRESS_EVT);
}
hal_spi_t spi =
{
    .spi_index = SPI0,
};
void spis_cb(spi_evt_t* evt);

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
uint32_t s_spi_rx_offset=0;
uint32_t s_framesize=0;
uint32_t s_framesize_evt=0;
uint8_t s_test_cnt = 0;

hal_spi_t s_spi =
{
    .spi_index = SPI0,
};


void rx_check_frame(void)
{
    uint16_t crc = 0, crc1 = 0,i = 0;
    LOG("Rx frame index is %d\n", s_spi_rx_buf[3]);

    if(s_framesize_evt < 10)
    {
        LOG("frame size incorrect:%d\n", s_framesize_evt);
        return;
    }

    crc = s_spi_rx_buf[4] + s_spi_rx_buf[5]*0x100;

    //crc1 = crc16(0, s_spi_rx_buf + 6, s_framesize_evt-6);
    for(i = 6; i< 0x100; i++)
    {
        crc1 += s_spi_rx_buf[i];
    }

    if(crc == crc1)
    {
        LOG("FRAME CRC OK\n");
    }
    else
    {
        LOG("FRAME CRC ERROR\n");
        LOG("crc:%x %x\n",crc,crc1);
    }
}
void process_rx(uint8_t* data, uint32_t size)
{
    if(s_spi_rx_offset == 0)
    {
        if(data[0] != 0x5a)
        {
            LOG("!");

            for(int i =0; i<size; i++)
                LOG("%x ",*(data+i));

            return;
        }
    }

    memcpy(s_spi_rx_buf+s_spi_rx_offset, data, size);
    s_spi_rx_offset += size;

    if(s_spi_rx_offset >=3 && s_framesize == 0)
    {
        s_framesize = s_spi_rx_buf[1] + s_spi_rx_buf[2]*0x100;
    }

    if(s_framesize && s_spi_rx_offset >= s_framesize)
    {
        osal_set_event(spi_TaskID, RX_GET_FRAME_EVT);
        s_framesize_evt = s_framesize;
        s_framesize = 0;
        s_spi_rx_offset = 0;
    }
}

void spis_cb(spi_evt_t* evt)
{
    switch(evt->evt)
    {
    case SPI_TX_COMPLETED:
        osal_set_event(spi_TaskID,TX_COMPLETED_EVT );
        break;

    case SPI_RX_COMPLETED:
        osal_set_event(spi_TaskID,RX_COMPLETED_EVT );
        break;

    case SPI_RX_DATA_S:
        process_rx(evt->data,evt->len);
        break;
    }

    //LOG("(SPI finish:%d %d)   ",evt.spi,evt.type);
}

void spi_demo_Init( uint8 task_id )
{
    spi_TaskID = task_id;
    touch_init(on_touch_evt);
    #if SPI_MASTER_TEST
    hal_spi_bus_init(&s_spi, s_spim_cfg);
    #else
    hal_spis_bus_init(&s_spi, s_spis_cfg);
    #endif
    osal_set_event( spi_TaskID, START_DEVICE_EVT);
}

uint16 spi_demo_ProcessEvent( uint8 task_id, uint16 events )
{
    int ret;

    if (events & TOUCH_PRESS_EVT )
    {
        //master: recieve string
        //slave: send string
        strcpy((char*)s_spi_tx_buf, s_str_xmit);
        s_spi_tx_buf[strlen(s_str_xmit)-1] = (uint8_t)('0' + (s_test_cnt%10));
        s_test_cnt++;
        hal_spi_set_tx_buffer(&s_spi, s_spi_tx_buf, 512),
                              memset(s_spi_rx_buf, 0, 512);
        #if DMAC_USE
        ret = hal_spi_transmit(&s_spi,SPI_TRXD,0,0, s_spi_tx_buf, s_spi_rx_buf, strlen(s_str_xmit)+1,strlen(s_str_xmit)+1);
        #else
        ret = hal_spi_transmit(&s_spi,SPI_TRXD, s_spi_tx_buf, s_spi_rx_buf, strlen(s_str_xmit)+1,strlen(s_str_xmit)+1);
        #endif
        LOG("[S Tx] result: %d\n", ret);

        if(s_spis_cfg.int_mode == false)
        {
            LOG("EVT: %s\n", s_spi_rx_buf);
        }

        return (events ^ TOUCH_PRESS_EVT);
    }

    if (events & RX_TIMEOUT_EVT )
    {
        return (events ^ RX_TIMEOUT_EVT);
    }

    if (events & TX_COMPLETED_EVT)
    {
        LOG("EVT TX: %s");
        return (events ^ TX_COMPLETED_EVT);
    }

    if (events & RX_COMPLETED_EVT)
    {
        LOG("EVT: %s\n", s_spi_rx_buf);
        return (events ^ RX_COMPLETED_EVT);
    }

    if (events & RX_GET_FRAME_EVT)
    {
        rx_check_frame();
        return (events ^ RX_GET_FRAME_EVT);
    }

    return 0;
}
