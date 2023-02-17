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


extern usb_hal_pcd_t pcd_handle;
extern volatile uint8_t hid_mouse_ready;
extern volatile uint8_t hid_keyboard_ready;


void hid_click_to_left(void)
{
	gpio_write(P9, 1);
	gpio_write(P9, 0);
	
	uint32_t select_cnt = 0;
    struct hid_mouse_report  mouse_report;
    usb_hal_pcd_t *hpcd = &pcd_handle;
    //Click to select.
    do
    {
        mouse_report.report_id      = 2;
        mouse_report.ac_pan         = 0;
        mouse_report.button         = 0x0;
        mouse_report.ctl.b.X        = -50;
        mouse_report.ctl.b.Y        = 0;
        mouse_report.ctl.b.Wheel    = 0;

        if (hpcd->ep[1].xfer_length == 0)
        {
            hal_pcd_send_data(hpcd, 1, (uint8_t *)&mouse_report, 8);
//			LOG_ERROR("send_left\n");
        }
		WaitMs(10);
        while (hpcd->ep[1].xfer_length != 0);

    }
    while (select_cnt++ < 10);
}

void hid_click_to_right(void)
{
	gpio_write(P11, 1);
	gpio_write(P11, 0);
	
	uint32_t select_cnt = 0;
    struct hid_mouse_report  mouse_report;
    usb_hal_pcd_t *hpcd = &pcd_handle;

    do
    {
        mouse_report.report_id      = 2;
        mouse_report.ac_pan         = 0;
        mouse_report.button         = 0x0;
        mouse_report.ctl.b.X        = 50;
        mouse_report.ctl.b.Y        = 0;
        mouse_report.ctl.b.Wheel    = 0;

        if (hpcd->ep[1].xfer_length == 0)
        {
            hal_pcd_send_data(hpcd, 1, (uint8_t *)&mouse_report, 8);
//			LOG_ERROR("send_right\n");
        }
		WaitMs(10);
        while (hpcd->ep[1].xfer_length != 0);
        
    }
    while (select_cnt++ < 10);	
	
}


void hid_click_to_up(void)
{
	uint32_t select_cnt = 0;
    struct hid_mouse_report  mouse_report;
    usb_hal_pcd_t *hpcd = &pcd_handle;
    //Click to select.
    do
    {
        mouse_report.report_id      = 2;
        mouse_report.ac_pan         = 0;
        mouse_report.button         = 0x0;
        mouse_report.ctl.b.X        = 0;
        mouse_report.ctl.b.Y        = 5;
        mouse_report.ctl.b.Wheel    = 0;

        if (hpcd->ep[2].xfer_length == 0)
        {
            hal_pcd_send_data(hpcd, 2, (uint8_t *)&mouse_report, 8);
        }
		WaitMs(10);
        while (hpcd->ep[2].xfer_length != 0);
//        WaitMs(20);
    }
    while (select_cnt++ < 40);
	
    mouse_report.report_id      = 2;
    mouse_report.ac_pan         = 0;
    mouse_report.button         = 0x0;
    mouse_report.ctl.b.X        = 0;
    mouse_report.ctl.b.Y        = 0;
    mouse_report.ctl.b.Wheel    = 0;
    while (hpcd->ep[2].xfer_length != 0);
    if (hpcd->ep[2].xfer_length == 0)
    {
        hal_pcd_send_data(hpcd, 2, (uint8_t *)&mouse_report, 8);
    }
    while (hpcd->ep[2].xfer_length != 0);	
}

void hid_click_to_down(void)
{
	uint32_t select_cnt = 0;
    struct hid_mouse_report  mouse_report;
    usb_hal_pcd_t *hpcd = &pcd_handle;

    do
    {
        mouse_report.report_id      = 2;
        mouse_report.ac_pan         = 0;
        mouse_report.button         = 0x0;
        mouse_report.ctl.b.X        = 0;
        mouse_report.ctl.b.Y        = -5;
        mouse_report.ctl.b.Wheel    = 0;

        if (hpcd->ep[2].xfer_length == 0)
        {
            hal_pcd_send_data(hpcd, 2, (uint8_t *)&mouse_report, 8);
        }
		WaitMs(10);
        while (hpcd->ep[2].xfer_length != 0);
//        WaitMs(20);
    }
    while (select_cnt++ < 40);	

    mouse_report.report_id      = 2;
    mouse_report.ac_pan         = 0;
    mouse_report.button         = 0x0;
    mouse_report.ctl.b.X        = 0;
    mouse_report.ctl.b.Y        = 0;
    mouse_report.ctl.b.Wheel    = 0;
    while (hpcd->ep[2].xfer_length != 0);
    if (hpcd->ep[2].xfer_length == 0)
    {
        hal_pcd_send_data(hpcd, 2, (uint8_t *)&mouse_report, 8);
    }
    while (hpcd->ep[2].xfer_length != 0);	
}

void hid_kbd_write(void)
{
	struct hid_keyboard_report  keyboard_report;
    usb_hal_pcd_t *hpcd = &pcd_handle;
	
	keyboard_report.key_ctl[0] = 0;
	keyboard_report.key_ctl[1] = 0;
	keyboard_report.key_val[0] = 0x11;
	keyboard_report.key_val[1] = 0x0C;
	keyboard_report.key_val[2] = 0x0B;
	keyboard_report.key_val[3] = 0x04;
	keyboard_report.key_val[4] = 0x12;
	keyboard_report.key_val[5] = 0x00;


	if (hpcd->ep[1].xfer_length == 0)
	{

		hal_pcd_send_data(hpcd, 1, (uint8_t *)&keyboard_report, 8);
		LOG("Send KeyBoard Report\n");
//		cnt++;
	}

	keyboard_report.key_ctl[0] = 0;
	keyboard_report.key_ctl[1] = 0;
	keyboard_report.key_val[0] = 0x00;
	keyboard_report.key_val[1] = 0x00;
	keyboard_report.key_val[2] = 0x00;
	keyboard_report.key_val[3] = 0x00;
	keyboard_report.key_val[4] = 0x00;
	keyboard_report.key_val[5] = 0x00;

	while (hpcd->ep[1].xfer_length != 0);
	if (hpcd->ep[1].xfer_length == 0)
	{

		hal_pcd_send_data(hpcd, 1, (uint8_t *)&keyboard_report, 8);
		LOG("Send KeyBoard Report\n");
//		cnt++;
	}	
}

void pcd_hid_test(void)
{
	
	if((hid_keyboard_ready == 1) || (hid_mouse_ready == 1))
	{
		
#if( _DEF_USB_INFO_ == USB_DEV_KBD)	//0
		hid_kbd_write();
		WaitMs(100);

#elif( _DEF_USB_INFO_ == USB_DEV_MS)	//1

		hid_click_to_left();
//		WaitMs(10);
		
		hid_click_to_right();

#elif( _DEF_USB_INFO_ == USB_DEV_MSKBD)	//2

		hid_kbd_write();
		WaitMs(100);		
		
		hid_click_to_up();
		hid_click_to_down();

#endif
	}
	else
	{
		LOG_DEBUG("hid device not ready\n");
	}
}

