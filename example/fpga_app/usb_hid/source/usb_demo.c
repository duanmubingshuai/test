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
  Filename:      
  Revised:       
  Revision:       


**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rom_sym_def.h"
#include "OSAL.h"
#include "usb_demo.h"
#include "log.h"
#include "clock.h"

#include "pwrmgr.h"
#include "error.h"
#include "usb_hal_pcd.h"
#include "usb_pcd.h"
#include "usb_usr_config.h"
#include "hid.h"


//extern usb_hal_pcd_t pcd_handle;
//extern volatile uint8_t hid_mouse_ready;
//extern volatile  uint8_t hid_keyboard_ready;

//volatile uint32_t cnt = 0;

uint8 application_TaskID;
//#define START_DEVICE_EVT    1
//#define USB_DEMO_ONCE_TIMER 2
 static void usb_irq_evt(void)
{
	WaitUs(10);
    osal_set_event(application_TaskID, usbDeviceTx_EVT);
}

uint32_t mouse_intv = 1;//20;
extern uint32 osal_sys_tick;

void usb_Init(uint8 task_id)
{	
	application_TaskID = task_id;
	LOG("usb_demo\n");
//	LOG("int1 = %x\n", (* (volatile int *) 0xe000e100));
//	LOG("int2 = %x\n", (* (volatile int *) 0xe000e104));
	
//	osal_start_reload_timer(application_TaskID, OSAL_RELOAY_TIMER_EVT, 100);
//	osal_set_event(application_TaskID, START_DEVICE_EVT);
	osal_set_event(application_TaskID, usbDeviceTask_EVT);
	gpio_dir(P11, GPIO_INPUT);
	gpio_dir(P12, GPIO_OUTPUT);
	gpio_pull_set(P11, GPIO_PULL_UP);
	
    gpio_dir(P18, GPIO_INPUT);
    gpio_dir(P19, GPIO_INPUT);
    gpio_pull_set(P18, GPIO_FLOATING);
    gpio_pull_set(P19, GPIO_FLOATING);
    
    pcd_init(usb_irq_evt);
//	pcd_hid_test();
}


//extern void pcd_hid_test();
int test_key_cnt = 0;
signed char test_key_value[4][2] = {
  {50,0},
  {0,50},
  {-50,0},
  {0, -50},
};
signed char test_key_value_[4][2] = {
  {1,2},
  {1,1},
  {1,0},
  {1, 2},
};
extern usb_hal_pcd_t pcd_handle;
extern int g_ep1_cnt;
static int wake_flg = 0;
extern uint8_t hid_remote_wakeup_rdy;
uint16 usb_ProcessEvent( uint8 task_id, uint16 events )
{	
	if(task_id != application_TaskID){
		return 0;
	}
	
	if ( events & usbDeviceTask_EVT )
    {
        struct hid_mouse_report  mouse_report;
        if(gpio_read(P11)==0)
        {
        
		    osal_start_timerEx(application_TaskID, usbDeviceTask_EVT, mouse_intv);
		    #if 0
            mouse_report.report_id      = 2;
            mouse_report.ac_pan         = 0;
            mouse_report.button         = 0;//test_key_cnt%1;
            mouse_report.ctl.b.X        = test_key_value[test_key_cnt%4][0];
            mouse_report.ctl.b.Y        = test_key_value[test_key_cnt%4][1];
            mouse_report.ctl.b.Wheel    = 0;
            test_key_cnt++;
            hal_pcd_send_data(&pcd_handle, 1, (uint8_t *)&mouse_report, 8);
            #else
            if(hid_remote_wakeup_rdy == HID_REMOTE_WAKEUP_READY){
                hid_remote_wakeup_rdy = HID_REMOTE_WAKEUP_GOING;
                hal_remote_wakeup(&pcd_handle);

            }
            else if(hid_remote_wakeup_rdy == 0)
            {
                mouse_report.report_id      = 2;
                mouse_report.ac_pan         = 0;
                mouse_report.button         = 0;//test_key_cnt%1;
                mouse_report.ctl.b.X        = test_key_value[test_key_cnt%4][0];
                mouse_report.ctl.b.Y        = test_key_value[test_key_cnt%4][1];
                mouse_report.ctl.b.Wheel    = 0;
                test_key_cnt++;
                hal_pcd_send_data(&pcd_handle, 1, (uint8_t *)&mouse_report, 8);
            }

            #endif
        }
        else
        {
		    osal_start_timerEx(application_TaskID, usbDeviceTask_EVT, mouse_intv);
		}
		
        return (events ^ usbDeviceTask_EVT);	
	}

	if ( events & usbDeviceTx_EVT )
    {

        usb_hid_tx_send(&pcd_handle);
        return (events ^ usbDeviceTx_EVT);
	}

	 
	return 0;
}

