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

/*******************************************************************************
    @file     rf_phy_driver.c
    @brief    Contains all functions support for PHYPLUS RF_PHY_DRIVER
    @version  1.0
    @date     24. Aug. 2017
    @author   Zhongqi Yang



*******************************************************************************/


/*******************************************************************************
    INCLUDES
*/
#include "rom_sym_def.h"
#include "rf_phy_driver.h"
#include "mcu.h"
#include "clock.h"
#include "timer.h"
#include "ll_hw_drv.h"

/*******************************************************************************
    BUILD CONFIG
*/

#define RF_PHY_DTM_CTRL_NONE                            0x00
#define RF_PHY_DTM_CTRL_UART                            0x01
#define RF_PHY_DTM_CTRL_HCI                             0x02

#define RF_PHY_DTM_BB_SUPPORT_BLE1M                     0x01
#define RF_PHY_DTM_BB_SUPPORT_BLE2M                     0x02
#define RF_PHY_DTM_BB_SUPPORT_BLR500K                   0x04
#define RF_PHY_DTM_BB_SUPPORT_BLR125K                   0x08
#define RF_PHY_DTM_BB_SUPPORT_ZIGBEE                    0x10

#define RF_PHY_DTM_BB_SUPPORT_BLR_CODED                 (RF_PHY_DTM_BB_SUPPORT_BLR500K|RF_PHY_DTM_BB_SUPPORT_BLR125K)
#define RF_PHY_DTM_BB_SUPPORT_BLE_5                     (RF_PHY_DTM_BB_SUPPORT_BLE1M|RF_PHY_DTM_BB_SUPPORT_BLE2M)
#define RF_PHY_DTM_BB_SUPPORT_FULL                      (RF_PHY_DTM_BB_SUPPORT_ZIGBEE|RF_PHY_DTM_BB_SUPPORT_BLE_5)


// TODO: move to phypuls_build_cfg.h
#define RF_PHY_DTM_CTRL_MOD                             RF_PHY_DTM_CTRL_UART
#define RF_PHY_DTM_BB_SUPPORT_MOD                       RF_PHY_DTM_BB_SUPPORT_BLE_5


#define RF_PHY_CT_MONITER                               0 // VCO corase tuning moniter counter
// 0 : disable moniter
// other: enable
#define RF_PHY_TIME_BASE                                TIME_BASE
#define RF_PHY_TIME_DELTA(x,y)                          TIME_DELTA(x,y)

/*******************************************************************************
    Global Var
*/
#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
    #include "log.h"
    #define MAX_UART_BUF_SIZE       32
    #define MAX_UART_BUF_ID         (MAX_UART_BUF_SIZE-1)
    #define DTM_OUT(x)              hal_uart_send_byte(x)
//    #define DTM_LOG_INIT(...)       dbg_printf_init()
//    #define DTM_LOG(...)            dbg_printf(##__VA_ARGS__)
	#define DTM_LOG_INIT(...)       LOG_INIT()
	#define DTM_LOG(...) 			  log_printf(__VA_ARGS__)
    #define CLR_UART_WIDX           {uart_rx_wIdx=0;}
    #define GET_UART_WIDX           (uart_rx_wIdx)
    #define DTM_ADD_IDX(a,b)        {(a==b)? a=0:a++;}
    static unsigned char            urx_buf[MAX_UART_BUF_SIZE];
    static volatile uint32_t        uart_rx_wIdx=0,uart_rx_rIdx=0;
    static volatile uint32_t        s_uart_rx_wIdx_ctrl=0;


volatile uint8_t        g_rfPhyDtmCmd[2]    =   {0};
volatile uint8_t        g_rfPhyDtmEvt[2]    =   {0};

volatile uint8_t        g_dtmModeType       =   0;
volatile uint8_t        g_dtmCmd            =   0;
volatile uint8_t        g_dtmCtrl           =   0;
volatile uint8_t        g_dtmPara           =   0;
volatile uint8_t        g_dtmEvt            =   0;
volatile uint8_t        g_dtmStatus         =   0;
volatile uint8_t        g_dtmFreq           =   0;
volatile uint8_t        g_dtmLength         =   0;
volatile uint8_t        g_dtmPKT            =   0;
volatile uint8_t        g_dtmTpCalEnable    =   1;  //default enable tpcal 
volatile uint8_t        g_dtmManualConfig   =   RF_PHY_DTM_MANUL_ALL;
volatile uint16_t       g_dtmPktCount       =   0;
volatile uint16_t       g_dtmRxCrcNum       =   0;
volatile uint16_t       g_dtmRxTONum        =   0;
volatile uint16_t       g_dtmRsp            =   0;
volatile uint16_t       g_dtmFoff           =   0;
volatile uint8_t        g_dtmRssi           =   0;
volatile uint8_t        g_dtmCarrSens       =   0;
#endif

#if(RF_PHY_CT_MONITER)
#define CT_MONT_BUFF_SIZE 128
volatile uint32_t g_rfPhy_ct_moniter_word_cnt = 0;
volatile uint32_t g_rfPhy_ct_moniter_word_target = 0;
volatile uint16_t g_rfPhy_ct_moniter_word_arry[CT_MONT_BUFF_SIZE] = {0};
#endif


#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_uart_irq

    @brief       This function process for rf phy direct test uart irq.

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
extern uint8_t rf_phy_dtm_reg_process(void);
void DTM_UART_IRQHandler(void)
{
    uint8_t IRQ_ID= (AP_UART0->IIR & 0x0f);

    switch (IRQ_ID)
    {
    case RDA_IRQ:
    case TIMEOUT_IRQ:
        while(AP_UART0 ->LSR & 0x1)
        {
            urx_buf[uart_rx_wIdx]=(AP_UART0->RBR & 0xff);
            DTM_ADD_IDX(uart_rx_wIdx, MAX_UART_BUF_ID);
            if(s_uart_rx_wIdx_ctrl)
            {
                if(GET_UART_WIDX>=2 && urx_buf[0] != RF_PHY_DTM_REG_RW_OPCODE )
                {
                    s_uart_rx_wIdx_ctrl=2; // set the rx widx to brack the rom ate while loop
                }
                else if( ( GET_UART_WIDX>=6 && urx_buf[1]==RF_PHY_DTM_REG_READ) ||
                         ( GET_UART_WIDX>=10 && urx_buf[1]==RF_PHY_DTM_REG_WRITE) )
                {
                    rf_phy_dtm_reg_process(); // process reg wr rd
                }
            }
        }

        break;

    case BUSY_IRQ:
        AP_UART0 -> USR;
        break;
    }
}
// static uint32_t dtm_read_current_time(void)
// {
//   // return ((4000000-get_timer3_count())/4+2);
//      return (RF_PHY_TIME_BASE - ((AP_TIM3->CurrentCount) >> 2) ) ;
// }

#endif
#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
uint8_t rf_phy_dtm_reg_process(void)
{
    uint8_t ret =0;
    uint32_t addr,val;

    if(GET_UART_WIDX>=2 && urx_buf[0] == RF_PHY_DTM_REG_RW_OPCODE )
    {

        if(urx_buf[1]==RF_PHY_DTM_REG_READ)
        {
            //buffering read reg info
            while(GET_UART_WIDX<6){};
            addr = urx_buf[2]<<24 |urx_buf[3]<<16 |urx_buf[4]<<8 |(urx_buf[5]&0xFC) ;//addr align
            val = PHY_REG_RD(addr);
            ret = RF_PHY_DTM_REG_READ;
            DTM_OUT(RF_PHY_DTM_REG_RW_OPCODE);
            DTM_OUT(RF_PHY_DTM_REG_RD_RSP);
            DTM_OUT(BREAK_UINT32(val,3));
            DTM_OUT(BREAK_UINT32(val,2));
            DTM_OUT(BREAK_UINT32(val,1));
            DTM_OUT(BREAK_UINT32(val,0));
        }
        else if (urx_buf[1]==RF_PHY_DTM_REG_WRITE)
        {
            //buffering write reg info
            while(GET_UART_WIDX<10){};
            addr = urx_buf[2]<<24 |urx_buf[3]<<16 |urx_buf[4]<<8 |(urx_buf[5]&0xFC) ;//addr align
            val  = urx_buf[6]<<24 |urx_buf[7]<<16 |urx_buf[8]<<8 |urx_buf[9] ;
            PHY_REG_WT(addr,val);
            ret = RF_PHY_DTM_REG_WRITE;
        }
        CLR_UART_WIDX;
    }
    return ret;
    
}
#endif
#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_cmd_parse

    @brief       This function process for rf phy direct test,cmd parse

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void rf_phy_dtm_cmd_parse(void)
{
    g_dtmCmd        = ((g_rfPhyDtmCmd[0] & 0xc0)>>6);      // bit 15 14

    //test setup and test end
    if(g_dtmCmd == 0 ||  g_dtmCmd==3 )
    {
        g_dtmCtrl       = ((g_rfPhyDtmCmd[0] & 0x3f)   );      // bit 13  8
        g_dtmPara       = ((g_rfPhyDtmCmd[1] & 0xfc)>>2);      // bit  7  2

        //TEST SETUP
        if(g_dtmCmd==0)
        {
            if(g_dtmCtrl==0 && g_dtmPara==0)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_RESET;
            }
	        else if(g_dtmCtrl==1)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_LENGTH_UP2BIT;
                //g_dtmExtLen   = (g_dtmPara&0x3);
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x01)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_1M;
                g_rfPhyPktFmt = PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x02)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_2M;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_BLE2M & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_BLE2M:PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==3 && g_dtmPara ==0x00)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STANDARD;
            }
            else if(g_dtmCtrl==3 && g_dtmPara ==0x01)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ASSUME_TX_MOD_INDX_STABLE;
            }
            else if(g_dtmCtrl==4 && g_dtmPara ==0x00)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_SUPPORTED_TEST_CASE;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x00)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_TX_OCTETS;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x01)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_TX_TIME;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x02)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_RX_OCTETS;
            }
            else if(g_dtmCtrl==5 && g_dtmPara ==0x03)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_READ_MAX_RX_TIME;
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_0
            }
            else if(g_dtmCtrl==0x3a)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_FREQ_FOFF;
                if(g_dtmManualConfig & RF_PHY_DTM_MANUL_FOFF)
                {
                    g_rfPhyFreqOffSet    = (g_dtmPara-10)*5;//[0:1:20]-->[-50:5:50]-->[-200:20:200]KHz
                }
                // PHYPLUSE DEFINED:  TX TEST MODE Set TP_CAL MANUAL
            }
            else if(g_dtmCtrl==0x3b)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_TPCAL_MANUAL;

                if(g_dtmPara==0)
                {
                    g_dtmTpCalEnable  = 1;
                }
                else
                {
                    g_dtmTpCalEnable  = 0;
                    g_rfPhyTpCal0     = (g_dtmPara)<<1;
                    PHY_REG_WT( 0x40030094,0x00001000+g_rfPhyTpCal0);   // tp_cal val
                }

                // PHYPLUSE DEFINED:  TX TEST MODE Set XTAL CAP
            }
            else if(g_dtmCtrl==0x3c)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_XTAL_CAP;

                if(g_dtmManualConfig & RF_PHY_DTM_MANUL_XTAL_CAP)
                {
                    XTAL16M_CAP_SETTING(g_dtmPara);
                }

                // PHYPLUSE DEFINED:  TX TEST MODE Set TX POWER
            }
            else if(g_dtmCtrl==0x3d)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_TX_POWER;

                if(g_dtmManualConfig & RF_PHY_DTM_MANUL_TXPOWER)
                {
                    g_rfPhyTxPower      = g_dtmPara;
                }

                // PHYPLUSE DEFINED:  TX TEST MODE Continous Modulation
            }
            else if(g_dtmCtrl==0x3e)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_TX_CTMOD;
                g_dtmFreq         = g_dtmPara;
                // PHYPLUSE DEFINED:  TX TEST MODE Single Tone
            }
            else if(g_dtmCtrl==0x3f)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_TX_SINGLE;
                g_dtmFreq         = g_dtmPara;
            }
            else
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ERROR;
            }

            //TEST END
        }
        else
        {
            if(g_dtmCtrl==0 && g_dtmPara==0)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_TEST_END;
                // PHYPLUSE DEFINED:  get foff
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==0)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_FOFF;
                // PHYPLUSE DEFINED:  get tpCal
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==1)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_TPCAL;
                // PHYPLUSE DEFINED:  get RSSI
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==2)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_RSSI;
                // PHYPLUSE DEFINED:  get CARR_SENS
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==3)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_CARR_SENS;
                // PHYPLUSE DEFINED:  get PER report Auto
            }
            else if(g_dtmCtrl==0x3f && g_dtmPara==4)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_GET_PER_AUTO;
            }
            else
            {
                g_dtmModeType = RF_PHY_DTM_MODE_ERROR;
            }
        }

        //tx-rx test
    }
    else if(   g_dtmCmd == 1 || g_dtmCmd == 2)
    {
        g_dtmFreq       = ((g_rfPhyDtmCmd[0] & 0x3f)   );      // bit 13  8
        g_dtmLength     = ((g_rfPhyDtmCmd[1] & 0xfc)>>2);      // bit  7  2
        g_dtmPKT        = ((g_rfPhyDtmCmd[1] & 0x03)   );      // bit  1  0

        if(g_dtmPktCount==3 && (g_rfPhyPktFmt==PKT_FMT_BLR125K || g_rfPhyPktFmt==PKT_FMT_BLR500K ))
        {
            g_dtmPKT = 4;//all ones 'b 11111111
        }

        g_dtmModeType   = (g_dtmCmd ==1 )? RF_PHY_DTM_MODE_RX_PER : RF_PHY_DTM_MODE_TX_BURST;
    }
}

#endif
//-------------------------------------------------------------------------------------


#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_evt_send

    @brief       This function process for rf phy direct test, test mode trigged

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void rf_phy_dtm_evt_send(uint8_t dtmType )
{
    //clear Event Buf
    g_rfPhyDtmEvt[0] = 0;
    g_rfPhyDtmEvt[1] = 0;

    if(dtmType==RF_PHY_DTM_MODE_ERROR)
    {
        g_dtmEvt    = 0;    //status
        g_dtmStatus = 1;    //Error
    }
    else
    {
        if(dtmType == RF_PHY_DTM_MODE_TEST_END)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmPktCount;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_FOFF)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmFoff;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_TPCAL)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_rfPhyTpCal0;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_RSSI )
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmRssi;
        }
        else if(dtmType == RF_PHY_DTM_MODE_GET_CARR_SENS)
        {
            g_dtmEvt    = 0x80;    //report
            g_dtmStatus = 0;
            g_dtmRsp    = g_dtmCarrSens;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_SUPPORTED_TEST_CASE)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 0x0c;    //bit3,support stable modulation index
            //bit2, support LE 2M PHY
            //bit1, support LE PACKET LENGTH EXTENTION
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_TX_OCTETS)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 27<<1;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_TX_TIME)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 1352<<1;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_RX_OCTETS)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 27<<1;
        }
        else if(dtmType == RF_PHY_DTM_MODE_READ_MAX_RX_TIME)
        {
            g_dtmEvt    = 0x00;    //status
            g_dtmStatus = 0;
            g_dtmRsp    = 1352<<1;
        }
        else
        {
            g_dtmEvt    = 0;        //status
            g_dtmStatus = 0;        //Sucess
            g_dtmRsp    = g_dtmModeType;
        }
    }

    g_rfPhyDtmEvt[0] = g_rfPhyDtmEvt[0] | g_dtmEvt    | (g_dtmRsp >>   8);
    g_rfPhyDtmEvt[1] = g_rfPhyDtmEvt[1] | g_dtmStatus | (g_dtmRsp & 0xff);
    DTM_OUT(g_rfPhyDtmEvt[0]);
    DTM_OUT(g_rfPhyDtmEvt[1]);
}


#endif


uint32_t rf_phy_dtm_ate_cmd_parse(void)
{
    uint32_t cmdWord=0;//
    uint8_t rfchn = 2+(g_dtmFreq<<1);
    /*
        loopN  pktT   pktLen rfChn mode
        [31:24][23:22][21:16][15:8][7:0]
    */
    if(g_dtmModeType==RF_PHY_DTM_MODE_TX_BURST)
    {
        cmdWord  = RF_PHY_DTM_MODE_ATE_TX_BURST;
        cmdWord |= (0xff000000) | (g_dtmPKT<<22) |(g_dtmLength<<16)|(rfchn<<8);
    }
    else if(g_dtmModeType==RF_PHY_DTM_MODE_TX_CTMOD)
    {
        cmdWord  = RF_PHY_DTM_MODE_ATE_TX_MOD;
        cmdWord |= (g_rfPhyTxPower<<24) | (g_dtmPKT<<22) |(g_dtmLength<<16)|(rfchn<<8);
    }
    else if(g_dtmModeType==RF_PHY_DTM_MODE_TX_SINGLE)
    {
        cmdWord  = RF_PHY_DTM_MODE_ATE_TX_CARR;
        cmdWord |= (g_rfPhyTxPower<<24)  | (g_dtmPKT<<22) |(g_dtmLength<<16)|(rfchn<<8);  
    }
    else if(g_dtmModeType==RF_PHY_DTM_MODE_RX_PER)
    {
        cmdWord  = RF_PHY_DTM_MODE_ATE_RX_DEMOD;
        cmdWord |= (0xff000000) | (g_dtmPKT<<22) |(g_dtmLength<<16)|(rfchn<<8);  
    }
    else if(g_dtmModeType==RF_PHY_DTM_MODE_GET_PER_AUTO)
    {
        cmdWord  = RF_PHY_DTM_MODE_ATE_RX_DEMOD;
        cmdWord |= (0xfe000000) | (g_dtmPKT<<22) |(g_dtmLength<<16)|(rfchn<<8);  
    }
    else if(g_dtmModeType==RF_PHY_DTM_MODE_SET_TX_POWER)
    {
	cmdWord  = RF_PHY_DTM_MODE_ATE_SET_TXPOWER;
        cmdWord |= (0xff000000) |(g_rfPhyTxPower<<8);  
    }

    return cmdWord;
   
}

void rx_per_trigger_loop(uint32_t loopN,uint8_t *dOut)
{
    uint16_t dtmPktCount = 0;
    uint16_t dtmRxCrcNum = 0;
    uint16_t dtmRxTONum = 0;
    uint8_t dtmRssi = 0;
    uint16_t dtmFoff = 0;
    uint8_t dtmCarrSens = 0;

    uint8_t rssi;
    uint16_t foff;
    uint8_t carrSens;
   
    while (loopN)
    {
        {if(s_uart_rx_wIdx_ctrl>=2){break;}}

        if (ll_hw_get_irq_status() & LIRQ_MD)
        {

            if (ll_hw_get_irq_status() & LIRQ_COK)
            {
                dtmPktCount++;
                rf_phy_get_pktFoot(&rssi, &foff, &carrSens);
            }
            else if (ll_hw_get_irq_status() & LIRQ_CERR)
            {
                dtmRxCrcNum++;
            }
            else if (ll_hw_get_irq_status() & LIRQ_RTO)
            {
                dtmRxTONum++;
            }
            else
            {
                // wrap the pktCount
                dtmPktCount = (dtmPktCount == 65535) ? 0 : dtmPktCount;
            }

            ll_hw_rst_tfifo();
            ll_hw_rst_rfifo();
            ll_hw_clr_irq();
            ll_hw_trigger();

            loopN = loopN-1;
            
            if (dtmPktCount > 0)
            {

                dtmRssi = (dtmPktCount == 1) ? rssi : ((dtmRssi + rssi) >> 1);
                dtmFoff = (dtmPktCount == 1) ? foff : ((dtmFoff + foff) >> 1);
                dtmCarrSens = (dtmPktCount == 1) ? carrSens : ((dtmCarrSens + carrSens) >> 1);
            }
        }

    } // end of while

    // for loopN > 255, only report dtmPktCount and RxTONum
    // when rxTONum > 255, report rxTONum=0xff
    dOut[0] = dtmPktCount & 0xff;
    dOut[1] = (dtmPktCount >> 8);
    dOut[2] = (dtmRxTONum > 0xff) ? 0xff : (dtmRxTONum & 0xff);
    dOut[3] = (dtmFoff >> 8) & 0xff;
    dOut[4] = (dtmFoff & 0xff);
    dOut[5] = dtmRssi;
    dOut[6] = dtmCarrSens;
}

#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)

void rf_phy_dtm_ate(void)
{
    int dtmState = 0;
    //enable received data available interrupt
    DTM_LOG_INIT();
    DTM_LOG("\n===RF_PHY_DTM_ATE V1.1.5===\n");
    //clr UART IRQ, switch to ROM_UART_IRQ
    JUMP_FUNCTION_SET(UART0_IRQ_HANDLER, (uint32_t)&DTM_UART_IRQHandler);

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLE1M)
        DTM_LOG("=== SUPPORT BLE 1M  ===\n");

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLE2M)
        DTM_LOG("=== SUPPORT BLE 2M  ===\n");

    #if 1
    uint8_t xtal_cap  =  read_reg(0x4000f0bc)&0x1f;
    uint8_t xtal_curt =  (read_reg(0x4000f0bc)>>5)&0x3;
    DTM_LOG("DTM Default : Tx Powre %x, FreqOffset %d KHz XTAL cap%x cur%x\n",g_rfPhyTxPower,g_rfPhyFreqOffSet*4,\
                                                                     xtal_cap,xtal_curt);
    #endif

    //*(volatile int *) 0xe000e100 |= 0x800;
    *(volatile int*) 0xe000e100 = 0x800; //only use uart irq
    *(volatile unsigned int*) 0x40004004 |= 0x01; //ENABLE_ERBFI;
    //set timer3 free run
    //AP_TIM3->ControlReg = 0x05;
    //24bit count down mode no IRQ
    set_timer(AP_TIM3,RF_PHY_TIME_BASE);
    NVIC_DisableIRQ(TIM3_IRQn);
    //clear widx
    CLR_UART_WIDX;
    uint8_t dtmAtedOut[8];
    //uint8_t dtmAtedOutLen=0;
    uint32_t dtmAteCmdWord;
    //uint32_t freqOffSet;
    while(1)
    {
        if(      dtmState == RF_PHY_DTM_IDL)
        {
            if(GET_UART_WIDX>=2)
            {
                if(0==rf_phy_dtm_reg_process())
                {
                    g_rfPhyDtmCmd[0]=urx_buf[0];
                    g_rfPhyDtmCmd[1]=urx_buf[1];
                    dtmState = RF_PHY_DTM_CMD;
                    CLR_UART_WIDX;
                }
            }

            //=================== cmd parsing  =====================
        }
        else if(dtmState == RF_PHY_DTM_CMD)
        {
            rf_phy_dtm_cmd_parse();
            dtmState = RF_PHY_DTM_EVT;
            //=================== send event   =====================
        }
        else if(dtmState == RF_PHY_DTM_EVT)
        {
            rf_phy_dtm_evt_send(g_dtmModeType);
            dtmState = RF_PHY_DTM_TEST;
            //=================== TEST Start    =====================
        }
        else if(dtmState == RF_PHY_DTM_TEST)
        {   
            // freqOffset bug was fixed in rom code 
            #if 0
            if(g_rfPhyFreqOffSet>=0)
                freqOffSet=(2+(g_dtmFreq<<1))+(g_rfPhyFreqOffSet<<8) +(g_rfPhyFreqOffSet<<16);
            else
                freqOffSet=(2+(g_dtmFreq<<1)-1)+((255+g_rfPhyFreqOffSet)<<8) +((255+g_rfPhyFreqOffSet)<<16);
            
            uint8_t regPatchNum=1;
            uint32_t addr[1]={0x400300b4};
            uint8_t  low[1]={0};
            uint8_t  high[1]={31};
            uint32_t val[1]={freqOffSet};
            #endif    
            dtmAteCmdWord = rf_phy_dtm_ate_cmd_parse();
            s_uart_rx_wIdx_ctrl=1;
            rf_phy_direct_test_ate(dtmAteCmdWord,(uint32_t*)(&s_uart_rx_wIdx_ctrl),\
                0,0,/*regPatchHigh*/0,/*regPatchLow*/0, /*regPatchVal*/0,\
                dtmAtedOut);
            s_uart_rx_wIdx_ctrl=0;

            if(g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO && GET_UART_WIDX<2)
            {
                s_uart_rx_wIdx_ctrl=1;
                while(s_uart_rx_wIdx_ctrl==1)
                {
                    g_dtmPktCount = dtmAtedOut[1]<<8 | dtmAtedOut[0];
                    g_dtmFoff     = dtmAtedOut[3]<<8 | dtmAtedOut[4];
                    g_dtmRssi     = dtmAtedOut[5];
                    g_dtmCarrSens = dtmAtedOut[6];
                    
                    rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_TEST_END);//send pktCount
                    WaitUs(500);
                    rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_GET_FOFF);//send FOFF
                    WaitUs(500);
                    rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_GET_RSSI);//send RSSI
                    WaitUs(500);
                    rf_phy_dtm_evt_send(RF_PHY_DTM_MODE_GET_CARR_SENS);//send CARR_SENS
                    WaitUs(500);
                    g_dtmPktCount = 0;
                    g_dtmRxCrcNum = 0;
                    g_dtmRxTONum = 0;
                    dtmState = RF_PHY_DTM_TEST;
                    
                    rx_per_trigger_loop(1000,dtmAtedOut);

                }
                s_uart_rx_wIdx_ctrl=0;
                
            }
            else
            {
                dtmState = RF_PHY_DTM_IDL;
            }
        }
    }//end of while(1)
}

#endif



void     ll_hw_set_pplus_pktfmt(uint8_t plen)
{
    uint32_t tmp  = 0xffff00e0 & (*(uint32_t*) (BB_HW_BASE + 0x40));
    *(volatile uint32_t*)(BB_HW_BASE+ 0x40) =tmp| ((plen-2)<<8) | 0x1f;
    ll_hw_set_crc_fmt(LL_HW_CRC_NULL,LL_HW_CRC_NULL);
}


/**************************************************************************************
    @fn          ll_hw_read_rfifo_pplus

    @brief       This function process for HW LL rx fifo pop out each calling will pop one
                rx paket as output

    input parameters

    @param

    output parameters

    @param       rxPkt   : buf for poped rx pkt,only the header+pdu, w/o crc
                pktLen  : length of rxPkt=pdulen+2
                pktFoot0: foot0 of the rx pkt
                pktF00t1: foot1 of the rx pkt

    @return      wlen: return rfifo poped cnt, 0: no pkt poped.
*/
uint8_t  ll_hw_read_rfifo_pplus(uint8_t* rxPkt, uint16_t* pktLen, uint32_t* pktFoot0, uint32_t* pktFoot1)
{
    int rdPtr,wrPtr,rdDepth,blen,wlen;
    uint32_t* p_rxPkt=(uint32_t*)rxPkt;
    ll_hw_get_rfifo_info(&rdPtr,&wrPtr,&rdDepth);

    if(rdDepth>0)
    {
        *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);
        blen    = (0xff00 & (*(volatile uint32_t*)(BB_HW_BASE+0x40)))>>8;  //get the byte length from reg
        wlen    = 1+ ( (blen+2-1) >>2 );        //+2 for pplus pktfmt_len register define

        while(p_rxPkt<(uint32_t*)rxPkt+wlen)
        {
            *p_rxPkt++ = *(volatile uint32_t*)(LL_HW_RFIFO);
        }

        *pktFoot0   = *(volatile uint32_t*)(LL_HW_RFIFO);
        *pktFoot1   = *(volatile uint32_t*)(LL_HW_RFIFO);
        *pktLen     = blen+2;
        return wlen;
    }
    else
    {
        rxPkt[0]  = 0;
        *pktFoot0 = 0;
        *pktFoot1 = 0;
        *pktLen   = 0;
        return 0;
    }
}

void rf_phy_get_pktFoot_fromPkt(uint32 pktFoot0, uint32 pktFoot1,
                                uint8* rssi, uint16* foff,uint8* carrSens)
{
    uint16_t tmpFoff;
//    pktFoot0        = (*(volatile uint32_t *) 0x400300e4);
//    pktFoot1        = (*(volatile uint32_t *) 0x400300e8);
    tmpFoff         =   pktFoot0 & 0x3ff;
    *foff           =   (tmpFoff>512) ? tmpFoff-512 : tmpFoff+512;
    *rssi           =   (pktFoot1>>24)     ;
    *carrSens       =   (pktFoot0>>24)     ;
}

void rf_calibrate1(void)
{
    subWriteReg(0x4000f018,14,9,0x3f);//set RC32M clk to highest reduce pll spur
    //========== do rf tp cal for tx and rx dcoffset cal
    rf_tpCal_gen_cap_arrary();                                  //generate the tpCal cap arrary
    g_rfPhyTpCal0+=8;
    g_rfPhyTpCal1+=8;
    g_rfPhyTpCal0_2Mbps+=4;
    g_rfPhyTpCal1_2Mbps+=4;   
}

extern void rf_phy_ini(void);
extern void rf_phy_change_cfg0(uint8 pktFmt);
void rf_phy_change_cfg1(uint8 pktFmt)
{
    rf_phy_change_cfg0(pktFmt);
    if(pktFmt==PKT_FMT_BLE2M)
    {
        PHY_REG_WT( 0x400300e0,0x00000180);   // set pga bw use small bw for zigbee and BLE2M
        //PHY_REG_WT( 0x40030090,0x00087000);   // set reg_dc 
    }
    else
    {
        PHY_REG_WT( 0x400300e0,0x00000280);   // set pga bw
        //PHY_REG_WT( 0x40030090,0x00045000);   // set reg_dc
       
    }
    PHY_REG_WT( 0x400300b0,0x01000001);                 // dac dly
}
static void rf_phy_agc_table(void)
{
    // update agc table gain0- gain8
    PHY_REG_WT(0x4000306c, 0x522b3137);
    PHY_REG_WT(0x40003070, 0x3a40464c);
    PHY_REG_WT(0x40003074, 0x22282e34);
}

void rf_phy_init1(void)
{
    rf_phy_ini();
    PHY_REG_WT( 0x400300dc,0x01a6fc7f);                 // [6:3] lna ana ldo set 0xf, lowest voltage
    PHY_REG_WT( 0x400300d0,0x00000100);                 // cp_progm for tx modulation
    rf_phy_change_cfg1(g_rfPhyPktFmt);
    rf_phy_agc_table();
}
