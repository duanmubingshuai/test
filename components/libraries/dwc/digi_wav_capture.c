
#include <stdlib.h>
#include <string.h>
#include "bus_dev.h"
#include "gpio.h"
#include "digi_wav_capture.h"

#define DWC_TOUT_PHASE_IDLE         (4000*50)  //50ms sample timeout, when wait for rx data!
#define DWC_TOUT_PHASE_SAMPLEING    (4000*2)   //2ms sample timeout, about 2 bytes in 9600 baudrate, that's enough!

#define DWC_CLK_FACTOR_REF              (0x1a0)
#define DWC_FREQ_DRIFT_MAX              30      //+-30 uart clock frequency drift
#define DWC_FREQ_DRIFT_RATE             8       //+-8% uart clock frequency drift
#define DWC_RC32M_FREQ_OFFSET_MAX       30      //+-30% RC 32MHz frequency offset


#define DWC_CMD_LEN                     6   //"URC32M" or "UXTL16"
enum{
    DWC_ST_IDLE = 0,
    DWC_ST_START,
    DWC_ST_SAMPLING,
    DWC_ST_DATA_PROCESS,
    DWC_ST_COMPLETED,
};

typedef struct{
    int16_t     clk;
    uint16_t    level;
}dwc_wav_t;

typedef struct{
    uint8_t     state;
    uint32_t*   wave;
    uint16_t    deepth;
    uint16_t    cnt;

    uint32_t    rc32m;      //calibrated 32M RC clock, default value is 32000000

    uint32_t    tim_snap;
}dwc_ctx_t;

static dwc_ctx_t s_dwc = {
    .rc32m = 32000000,
    .state = DWC_ST_IDLE,
    .wave = NULL,
    .deepth = 0,
    .cnt = 0,
    .tim_snap = 0,
};
static uint16_t dwc_tim_intv(void)
{
    uint32_t cnt = AP_TIM1->CurrentCount;
    uint32_t diff = s_dwc.tim_snap - cnt;
    s_dwc.tim_snap = cnt;
    return diff;
}

static bool dwc_tim_check_tout(void)
{
    bool is_tout = FALSE;
    uint32_t diff;
    DRV_ENTER_CRITICAL_SECTION();
    if(s_dwc.state == DWC_ST_START)
    {
        diff = s_dwc.tim_snap - AP_TIM1->CurrentCount;
        if(diff > DWC_TOUT_PHASE_SAMPLEING){
            s_dwc.state ++; //move state machine to next state
            is_tout = TRUE;
        }
    }
    else if(s_dwc.state == DWC_ST_SAMPLING)
    {
        uint32_t to_value = s_dwc.cnt ? DWC_TOUT_PHASE_SAMPLEING : DWC_TOUT_PHASE_IDLE;
        diff = s_dwc.tim_snap - AP_TIM1->CurrentCount;
        
        if(diff > to_value){
            s_dwc.state ++; //move state machine to next state
            is_tout = TRUE;
        }
    }
    DRV_EXIT_CRITICAL_SECTION();
    return is_tout;
}


static void dwc_gpio_cb(comm_evt_t* pev)
{
    uint32_t intv = dwc_tim_intv() & 0xffffff;
    if(s_dwc.state == DWC_ST_SAMPLING && s_dwc.cnt < s_dwc.deepth){
    //if(s_dwc.cnt < s_dwc.deepth){
        if(s_dwc.cnt > 0){
            //s_dwc.wave[s_dwc.cnt-1] &= 0x80000000;
            s_dwc.wave[s_dwc.cnt-1] |= intv;
        }
        s_dwc.wave[s_dwc.cnt] = (pev->type == GPIO_EVT_EDGE_RISING) ? 0x80000000 : 0;
        s_dwc.cnt ++;
    }
}

dwc_handshake_cmd_t dwc_data_process(void)
{
    int i;
    dwc_wav_t* pwave = (dwc_wav_t*)(s_dwc.wave);
    uint32_t factor = 0, tmp;
    uint8_t cmdbyte = 0;
    char cmd[10];

    memset(cmd, 0, 10);
    
    if(s_dwc.cnt < 20)
        return DWC_HSCMD_NONE;
    
    //find clock factor, 1 start bit, 1 stop bit
    //first byte should be 0x55
    for(i = 0; i< 10; i++)
        factor += s_dwc.wave[i] & 0xffff;
    factor = factor / 10;
    tmp = (factor * 100) / DWC_CLK_FACTOR_REF;

    //figure RC32M frequency drift(20% margin)
    if( tmp > (100 + DWC_RC32M_FREQ_OFFSET_MAX) || tmp < (100-DWC_RC32M_FREQ_OFFSET_MAX))
        return DWC_HSCMD_NONE;
    
    for(i = 0; i< 10; i++)
    {
        tmp = (s_dwc.wave[i]& 0xffff) *100  /factor;
        if( tmp > (100 + DWC_FREQ_DRIFT_RATE) || tmp < (100 - DWC_FREQ_DRIFT_RATE))
            return DWC_HSCMD_NONE;
    }

    //got clock factor and real RC32M frequency, then digitize the wave
    for(i = 0; i< DWC_CMD_LEN; i++){
        uint8_t bitflg = 0;
        int j;
        //start bit
        if(pwave->level == 0x8000)
            return DWC_HSCMD_NONE;
        pwave->clk -= factor;
        if(pwave->clk < -DWC_FREQ_DRIFT_MAX)
            return DWC_HSCMD_NONE;
        if(abs(pwave->clk) <= DWC_FREQ_DRIFT_MAX) //move to next wave
            pwave ++;
        
        //byte
        cmdbyte = 0;
        for(j = 0; j < 8; j++){
            bitflg = (pwave->level == 0x8000) ? 1 : 0;
            pwave->clk -= factor;
            if(pwave->clk < -DWC_FREQ_DRIFT_MAX)
                return DWC_HSCMD_NONE;
            if(abs(pwave->clk) <= DWC_FREQ_DRIFT_MAX) //move to next wave
                pwave ++;
            if(bitflg)
                cmdbyte |= BIT(j);
        }
        //stop bit
        if(pwave->level == 0x0000)
            return DWC_HSCMD_NONE;

        //move to next start bit
        pwave ++;
        cmd[i] = cmdbyte;

    }

    s_dwc.rc32m = ((factor * 32000) / DWC_CLK_FACTOR_REF)*1000;
    
    if(strcmp(cmd, "URC32M") == 0)
        return DWC_HSCMD_RC32M;
    else if(strcmp(cmd, "UXTL16") == 0)
        return DWC_HSCMD_XTAL16M;
    else if(strcmp(cmd, "UDLL48") == 0)
        return DWC_HSCMD_DLL48M;
    else if(strcmp(cmd, "UXTDWU") == 0)
        return DWC_HSCMD_XT16DWU;

    return DWC_HSCMD_NONE;

}


//when a valid handshake command detected, return correspond ID
dwc_handshake_cmd_t dwc_loop(void)
{
    dwc_handshake_cmd_t cmd = DWC_HSCMD_BOOTAPP;
    //start trigger
    s_dwc.state = DWC_ST_START;//for debug
    dwc_tim_intv();
    
    //normal mode should do pullup
    gpio_pull_set(DWC_SWU_PIN, GPIO_PULL_UP);

    gpio_in_trigger_r(DWC_SWU_PIN, dwc_gpio_cb);

    while(1){
         if(dwc_tim_check_tout() ==  FALSE)
            continue;

         if(s_dwc.state == DWC_ST_DATA_PROCESS)
         {
            cmd = dwc_data_process();
            break;
         }
            
    }
    //should disable GPIO interrupt
    NVIC_DisableIRQ(GPIO_IRQn);
    gpio_interrupt_set_r(DWC_SWU_PIN, Bit_DISABLE);

	  AP_TIM1->ControlReg = 0;
    //release pullup after dwc
    gpio_pull_set(DWC_SWU_PIN, GPIO_FLOATING);

    return cmd;
}

uint32_t __attribute__((used)) dwc_rc32m_frequency(void)
{
    return s_dwc.rc32m;
}


dwc_handshake_cmd_t dwc_connect(uint32_t* wav_buf, uint16_t  wav_buf_deepth)
{
    int i;
    s_dwc.wave = wav_buf;
    s_dwc.deepth = wav_buf_deepth;
    s_dwc.cnt = 0;

    s_dwc.rc32m = 32000000;
    
    //set timer1 free run
    AP_TIM1->ControlReg = 0x05;
    
    //initialize buffer
    for(i = 0; i< s_dwc.deepth; i++)
        s_dwc.wave[i] = 0;


    //loop
    return dwc_loop();
    
}


