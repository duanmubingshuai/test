#ifndef __USB_PCD_H__
#define __USB_PCD_H__

#include "usb_hal_pcd.h"

#ifdef __cplusplus
extern "C" {
#endif
#define EP0_MAX_PKT_SZ (64)
#define EP1_MAX_PKT_SZ (64)
#define EP2_MAX_PKT_SZ (64)
#define EP3_MAX_PKT_SZ (64)

void pcd_init(usb_irq_cb_t cb);
void usb_hid_tx_send(usb_hal_pcd_t *hpcd);
void USB_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif    /*  __USB_PCD_H__ */