
#ifndef _DIGI_WAV_CAPTURE_H
#define _DIGI_WAV_CAPTURE_H

#include "gpio.h"
#include "uart.h"

typedef enum{
    DWC_HSCMD_BOOTAPP   = 0,  DWC_HSCMD_NONE = 0,  /*case handshake timeout or get boot command*/
    DWC_HSCMD_RC32M     = 1,    //"URC32M"      SWU
    DWC_HSCMD_XTAL16M   = 2,    //"UXTL16"      SWU
    DWC_HSCMD_DLL48M    = 3,    //"UDLL48"      SWU
    DWC_HSCMD_XT16DWU   = 4,    //"UXTDWU"      DWU16 XTAL
    DWC_HSCMD_TM        = 5     //tm = 1, p24 ^ p25 = 1
}dwc_handshake_cmd_t;

#define DWC_SWU_PIN              SWU_RX_PIN

uint32_t __attribute__((weak)) dwc_rc32m_frequency(void);
dwc_handshake_cmd_t dwc_connect(uint32_t* wav_buf, uint16_t  wav_buf_deepth);

#endif

