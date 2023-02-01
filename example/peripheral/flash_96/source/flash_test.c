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
    Filename:       watchdog_demo.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "OSAL.h"
#include "flash_test.h"
#include "log.h"
#include "gpio.h"
#include "clock.h"
#include "flash.h"
#include "error.h"

static uint8_t flash_demo_TaskID;
#define DEMO_1000MS_EVENT    0x0001
#define DEMO_1000MS_CYCLE    1000


extern FLASH_CHIP_INFO phy_flash;

#if(FLASH_PROTECT_FEATURE == 1)
void flash_lock_test(void)
{
    uint8_t i,j;
    uint32_t addr;
    uint8_t data_w[4] = {0x12,0x34,0x56,0x78};
    uint8_t data_r[4] = {0x00,0x00,0x00,0x0};
    volatile uint8_t status;

    for(j = 1; j<13; j++)
    {
        addr = phy_flash.Capacity - j<<2;
        LOG("\n\n\naddr:0x%x\n",addr);

        if(j%2)
        {
            hal_flash_lock();
            status = hal_flash_get_lock_state();

            if(status == 0x00)
            {
                LOG("error:%d %d\n",__LINE__,status);

                while(1);
            }

            LOG("lock\n");
        }
        else
        {
            hal_flash_unlock();
            status = hal_flash_get_lock_state();

            if(status != 0x00)
            {
                LOG("error:%d %d\n",__LINE__,status);

                while(1);
            }

            LOG("unlock\n");
        }

        hal_flash_read(addr, data_r, 4);
        LOG("read\n");

        for(i=0; i<4; i++)
        {
            LOG("%.2x ",data_r[i]);
        }

        LOG("\nwrite");
        hal_flash_write(addr,data_w,4);
        hal_flash_read(addr, data_r, 4);
        LOG("\nread\n");

        for(i=0; i<4; i++)
        {
            LOG("0x%.2x ",data_r[i]);
        }
    }

    //hal_flash_lock();

    while(1)
    {
        LOG("\nlock state:0x%x",hal_flash_get_lock_state());
        WaitMs(1000);
    }
}
#endif


void flash_erase_write_read_test(void)
{
}

extern uint32_t spif_flash_size(void);
//uint32_t test_data;
void Flash_Test_Init( uint8 task_id )
{
    flash_demo_TaskID = task_id;
//    LOG("\n-flash test start-\n");
//    //LOG("1------>0x%x\n",test_data);
//    LOG("flash id:0x%x\n",phy_flash.IdentificationID);
//    LOG("flash capacity:0x%x\n",phy_flash.Capacity);
//    LOG("2------>0x%x\n",spif_flash_size());
    //flash_lock_test();
    //flash_erase_write_read_test();
    //LOG("\nflash test end:no problem\n");
    osal_start_reload_timer(flash_demo_TaskID, DEMO_1000MS_EVENT, DEMO_1000MS_CYCLE);
}

uint16 Flash_Test_ProcessEvent( uint8 task_id, uint16 events )
{
    //static uint32_t counter = 0;
    if(events & DEMO_1000MS_EVENT)
    {
        //LOG("%d\n",counter++);
        return (events ^ DEMO_1000MS_EVENT);
    }

    return 0;
}


