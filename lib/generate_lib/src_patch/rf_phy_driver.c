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
#define RF_PHY_DTM_BB_SUPPORT_BLE_5                     (RF_PHY_DTM_BB_SUPPORT_BLE1M|RF_PHY_DTM_BB_SUPPORT_BLE2M|RF_PHY_DTM_BB_SUPPORT_BLR_CODED)
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
    #define _DTM_UART_              UART0
    #define DTM_OUT(x)              hal_uart_send_byte(_DTM_UART_,x)
//    #define DTM_LOG_INIT(...)       dbg_printf_init()
//    #define DTM_LOG(...)            dbg_printf(##__VA_ARGS__)
	#define DTM_LOG_INIT(...)       LOG_INIT()
	#define DTM_LOG(...) 			  log_printf(__VA_ARGS__)
    #define CLR_UART_WIDX           {uart_rx_wIdx=0;}
    #define GET_UART_WIDX           (uart_rx_wIdx)
    #define DTM_ADD_IDX(a,b)        {(a==b)? a=0:a++;}
    static unsigned char            urx_buf[MAX_UART_BUF_SIZE];
    static volatile uint32_t        uart_rx_wIdx=0,uart_rx_rIdx=0;
#endif

#if(RF_PHY_CT_MONITER)
#define CT_MONT_BUFF_SIZE 128
volatile uint32_t g_rfPhy_ct_moniter_word_cnt = 0;
volatile uint32_t g_rfPhy_ct_moniter_word_target = 0;
volatile uint16_t g_rfPhy_ct_moniter_word_arry[CT_MONT_BUFF_SIZE] = {0};
#endif


void rf_phy_change_cfg1(uint8_t pktFmt)
{
    rf_phy_change_cfg0(pktFmt);
    PHY_REG_WT(0x400300d8,0x04890000);  // i_pll_ctrl3 vco/tp varactor
    if(pktFmt==PKT_FMT_BLE1M)
    {
        subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var
    }
    else
    {
        subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var
    
    }

}

void rf_phy_ini1(void)
{
    rf_phy_ini();
    
    PHY_REG_WT(0x400300d8,0x04890000);  // i_pll_ctrl3 vco/tp varactor
    if(g_rfPhyPktFmt==PKT_FMT_BLE1M)
    {
        subWriteReg( 0x400300d8,20,18,0x01);   // tpm dac var
    }
    else
    {
        subWriteReg( 0x400300d8,20,18,0x02);   // tpm dac var
    
    }
    PHY_REG_WT(0x40031074,0x00800080);
}

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
/**************************************************************************************
    @fn          rf_phy_direct_test

    @brief       This function process for rf phy direct test.

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/

void rf_phy_direct_test(void)
{
    int dtmState = 0;
    uint32_t deltTick = 0;
    uint32_t currTick = 0;
    //enable received data available interrupt
    DTM_LOG_INIT();
    DTM_LOG("\n===RF_PHY_DTM V1.0.0===\n");
    //clr UART IRQ, switch to ROM_UART_IRQ
    JUMP_FUNCTION_SET(UART0_IRQ_HANDLER,(uint32_t)&DTM_UART_IRQHandler);
    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLE1M)
        DTM_LOG("=== SUPPORT BLE 1M  ===\n");

    if(RF_PHY_DTM_BB_SUPPORT_MOD&RF_PHY_DTM_BB_SUPPORT_BLE2M)
        DTM_LOG("=== SUPPORT BLE 2M  ===\n");

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

    while(1)
    {
        if(      dtmState == RF_PHY_DTM_IDL)
        {
            if(GET_UART_WIDX>=2)
            {
                g_rfPhyDtmCmd[0]=urx_buf[0];
                g_rfPhyDtmCmd[1]=urx_buf[1];
                dtmState = RF_PHY_DTM_CMD;
                CLR_UART_WIDX;
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
            rf_phy_dtm_trigged();
            g_dtmPktCount = 0;
            uint8_t rssi;
            uint8_t carrSens;
            uint16_t foff;
            uint8_t zgb_pkt_flg=0;

            //when new cmd arrived, state change
            while(GET_UART_WIDX<2)
            {
                //TX BURST re-trigger
                if(g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST)
                {
                    //============================================================
                    //20180424-ZQ
                    //option 1
                    ll_hw_set_trx_settle    (100, 8, 90);       //TxBB,RxAFE,PLL
                    //============================================================
                    currTick = read_current_fine_time();
                    deltTick = RF_PHY_TIME_DELTA(currTick,g_dtmTick);

                    if(     (ll_hw_get_irq_status() & LIRQ_MD)
                            &&  g_rfPhyPktFmt == PKT_FMT_ZIGBEE
                            &&  zgb_pkt_flg==0)
                    {
                        //ll_hw_rst_tfifo();
                        //rf_phy_dtm_zigbee_pkt_gen();
                        ll_hw_clr_irq();
                        zgb_pkt_flg=1;
                        PHY_REG_WT( 0x4003105c,0x00000000);     // clr rd_ini and rd_last
                    }

                    if(deltTick>=g_dtmPktIntv)
                    {
                        if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
                        {
                            ll_hw_rst_tfifo();
                            ll_hw_trigger();
//                            if(g_rfPhyPktFmt==PKT_FMT_BLE2M)
//                            {
//                                //set tx_auto_ctrl0 first
//                                PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
//                                PHY_REG_WT( 0x400300a4,0x00000150);     // clr tx_auto
//                                //wait a while then set tx_bb_en
//                                int delay=100;while(delay--){};
//                                PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
//                            }
//                            else
//                            {
//                                ll_hw_trigger();
//                                //PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
//                                //PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
//                                //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
//
//                            }
                        }
                        else
                        {
                            ll_hw_trigger();
                            //PHY_REG_WT( 0x400300a4,0x00000140);     // clr tx_auto
                            //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
                            zgb_pkt_flg=0;
                        }

                        #if(RF_PHY_CT_MONITER)
                        WaitUs(350);
                        int32_t ct_word= (PHY_REG_RD(0x400300f4)&0x3ff);

                        if(ct_word>0)
                        {
                            if(g_rfPhy_ct_moniter_word_cnt==0)
                                g_rfPhy_ct_moniter_word_target = ct_word;

                            int32_t idx= (ct_word)-g_rfPhy_ct_moniter_word_target+(CT_MONT_BUFF_SIZE>>1);
                            g_rfPhy_ct_moniter_word_cnt++;

                            if(idx<0)
                            {
                               LOG("e %d %d\n",ct_word,PHY_REG_RD(0x400300f8)&0xfffff);
                                g_rfPhy_ct_moniter_word_arry[0]++;
                            }
                            else if(idx>CT_MONT_BUFF_SIZE-1)
                            {
                                g_rfPhy_ct_moniter_word_arry[CT_MONT_BUFF_SIZE-1]++;
                            }
                            else
                            {
                                g_rfPhy_ct_moniter_word_arry[idx]++;
                            }

                            if(g_rfPhy_ct_moniter_word_cnt>RF_PHY_CT_MONITER)
                            {
                                for (int i=0; i<CT_MONT_BUFF_SIZE; i++)
                                {
                                    if(g_rfPhy_ct_moniter_word_arry[i]>0)
                                       LOG("%d %d\n",g_rfPhy_ct_moniter_word_target-(CT_MONT_BUFF_SIZE>>1)+i,g_rfPhy_ct_moniter_word_arry[i]);

                                    g_rfPhy_ct_moniter_word_arry[i]=0;
                                }

                                g_rfPhy_ct_moniter_word_cnt=0;
                            }
                        }

                        #endif
                        //g_dtmTick = read_current_time();
                        g_dtmTick = g_dtmTick+g_dtmPktIntv;

                        if(g_dtmTick>RF_PHY_TIME_BASE)
                            g_dtmTick = g_dtmTick-RF_PHY_TIME_BASE;
                    }
                }
                else if(   g_dtmModeType == RF_PHY_DTM_MODE_RX_PER
                           ||  g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO)
                {
                    //fix max gain can imporve 2480 sensitivity
                    if(g_dtmManualConfig&RF_PHY_DTM_MANUL_MAX_GAIN)
                    {
                        subWriteReg(0x40030050, 4,0,0x10);      //fix max gain
                    }

                    subWriteReg(0x4000f018,14,9,0x3f);//rc32M clk slow

                    if(ll_hw_get_irq_status()&LIRQ_MD)
                    {
                        if(ll_hw_get_irq_status()&LIRQ_COK)
                        {
                            g_dtmPktCount++;
                            rf_phy_get_pktFoot(&rssi,&foff,&carrSens);
                        }
                        else if(ll_hw_get_irq_status()&LIRQ_CERR)
                        {
                            g_dtmRxCrcNum++;
                        }
                        else if(ll_hw_get_irq_status()&LIRQ_RTO)
                        {
                            g_dtmRxTONum++;
                        }
                        else
                        {
                            //wrap the pktCount
                            g_dtmPktCount= (g_dtmPktCount==65535) ? 0 : g_dtmPktCount;
                        }

                        if( g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO)
                        {
                            currTick = read_current_fine_time();
                            deltTick = RF_PHY_TIME_DELTA(currTick,g_dtmTick);

                            if(deltTick>g_dtmPerAutoIntv)
                            {
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
                                g_dtmTick = read_current_fine_time();
                            }
                        }//EndOfIF PER_AUTO

                        ll_hw_clr_irq();
                        ll_hw_trigger();

                        if(g_dtmPktCount>0)
                        {
                            g_dtmRssi       = (g_dtmPktCount==1) ? rssi     :((g_dtmRssi+rssi)          >>1);
                            g_dtmFoff       = (g_dtmPktCount==1) ? foff     :((g_dtmFoff+foff)          >>1);
                            g_dtmCarrSens   = (g_dtmPktCount==1) ? carrSens :((g_dtmCarrSens+carrSens)  >>1);
                        }
                    }
                }//end of RX_PER
            }//end of GET_UART_WIDX

            dtmState = RF_PHY_DTM_IDL;
        }
    }//end of while(1)
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
                g_dtmExtLen   = (g_dtmPara&0x3);
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
            else if(g_dtmCtrl==2 && g_dtmPara ==0x03)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_125K;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_BLR125K & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_BLR125K:PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x04)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_500K;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_BLR500K & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_BLR500K:PKT_FMT_BLE1M;
            }
            else if(g_dtmCtrl==2 && g_dtmPara ==0x20)
            {
                g_dtmModeType = RF_PHY_DTM_MODE_SET_PHY_ZB;
                g_rfPhyPktFmt = (RF_PHY_DTM_BB_SUPPORT_ZIGBEE & RF_PHY_DTM_BB_SUPPORT_MOD)
                                ?PKT_FMT_ZIGBEE:PKT_FMT_BLE1M;
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
            else if(g_dtmCtrl==0x36)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_0;
                g_dtmAccessCode   = (g_dtmAccessCode&(0xffffff00))|g_rfPhyDtmCmd[1];
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_1
            }
            else if(g_dtmCtrl==0x37)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_1;
                g_dtmAccessCode   = (g_dtmAccessCode&(0xffff00ff))|(g_rfPhyDtmCmd[1]<<8);
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_2
            }
            else if(g_dtmCtrl==0x38)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_2;
                g_dtmAccessCode   = (g_dtmAccessCode&(0xff00ffff))|(g_rfPhyDtmCmd[1]<<16);
                // PHYPLUSE DEFINED:  RF_PHY_DTM_MODE_SET_ACCCODE_3
            }
            else if(g_dtmCtrl==0x39)
            {
                g_dtmModeType     = RF_PHY_DTM_MODE_SET_ACCCODE_3;
                g_dtmAccessCode   = (g_dtmAccessCode&(0x00ffffff))|(g_rfPhyDtmCmd[1]<<24);
                // PHYPLUSE DEFINED:  TX TEST MODE Set FREQ FOFF
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
                    g_dtmTxPower      = g_dtmPara;
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

//-------------------------------------------------------------------------------------


#if(RF_PHY_DTM_CTRL_MOD == RF_PHY_DTM_CTRL_UART)
/**************************************************************************************
    @fn          rf_phy_dtm_trigged

    @brief       This function process for rf phy direct test, test mode trigged

    input parameters

    @param       none

    output parameters

    @param       none

    @return      none
*/
void rf_phy_dtm_trigged(void)
{
    //cal the pkt length and pkt interval
    g_dtmLength = ((0x03&g_dtmExtLen)<<6)|(0x3f&g_dtmLength);
    int pktLenUs =0;                        //(g_dtmLength+10)<<3;
    uint8_t rfChn = 2+(g_dtmFreq<<1);
    uint8_t preambleLen = 1;

//    if(         pktLenUs <= 376  )  {   g_dtmPktIntv = 625;
//    }else if(   pktLenUs <= 1000 )  {   g_dtmPktIntv = 1250;
//    }else if(   pktLenUs <= 1624 )  {   g_dtmPktIntv = 1875;
//    }else if(   pktLenUs <= 2120 )  {   g_dtmPktIntv = 2500;
//    }else                           {   g_dtmPktIntv = 2500;
//    }
//
    if(         g_rfPhyPktFmt==PKT_FMT_BLE1M)
    {
        pktLenUs = (g_dtmLength+1+4+2+3)<<3;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLE2M)
    {
        pktLenUs = (g_dtmLength+2+4+2+3)<<2;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLR500K)
    {
        pktLenUs = 80+296+((g_dtmLength+2+3)<<4)+6;
    }
    else if(   g_rfPhyPktFmt==PKT_FMT_BLR125K)
    {
        pktLenUs = 80+296+((g_dtmLength+2+3)<<6)+24;
    }
    else
    {
        pktLenUs = (g_dtmLength+4+1+1)<<5;//per byte -> 2symbol ->32us
        rfChn  = 5+g_dtmFreq*5;//for zigbee
    }

    g_dtmPktIntv = (((pktLenUs+249)+624)/625)*625;//  ceil((L+249)/625) * 625

    if(g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO)
    {
        g_dtmPerAutoIntv = g_dtmPktIntv*1000;
        g_dtmTick = read_current_fine_time();
    }

    PHY_REG_WT( 0x40030040,0x00030000);     // close tx_bb test mode
    PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
    PHY_REG_WT( 0x400300a4,0x00000100);     // clr tx_auto
    PHY_REG_WT( 0x400300a0,0x00000000);     // clr pll_auto override
    PHY_REG_WT( 0x4003008c,0x00104040);     // clr tp_cal_en

    if(g_dtmModeType == RF_PHY_DTM_MODE_RESET)
    {
        g_dtmPktCount       =   0;
        g_dtmRsp            =   0;
        g_dtmExtLen         =   0;
        g_dtmLength         =   0;
        g_dtmPktIntv        =   0;
        g_dtmTxPower        =   g_rfPhyTxPower;
        g_dtmFoff           =   0;
        g_dtmRssi           =   0;
        g_dtmCarrSens       =   0;
        g_rfPhyPktFmt       =   PKT_FMT_BLE1M;
        g_dtmPKT            =   0;
        DCDC_CONFIG_SETTING(0x0a);
    }
    else if(   g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST
               || g_dtmModeType == RF_PHY_DTM_MODE_TX_CTMOD
               || g_dtmModeType == RF_PHY_DTM_MODE_TX_SINGLE   )
    {
        //====== tp cal
        if(g_dtmTpCalEnable)
        {
            rf_phy_ana_cfg();
            //rf_tpCal_cfg(rfChn);
            rf_tpCal_cfg_avg(rfChn,4);
        }

        //====== rf initial
        rf_phy_ini1();
        rf_phy_set_txPower(g_dtmTxPower);
        ll_hw_set_timing(g_rfPhyPktFmt);

        //add some tx_foff
        if(g_rfPhyFreqOffSet>=0)
		{
            PHY_REG_WT(0x400300b4,rfChn+(g_rfPhyFreqOffSet<<8));
        }
		else
		{
            PHY_REG_WT(0x400300b4,rfChn-1+((255+g_rfPhyFreqOffSet)<<8));
		}
        if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
        {
            PHY_REG_WT(0x40030048,RF_PHY_DTM_CRC_WT     );
            PHY_REG_WT(0x4003004c,g_dtmAccessCode  );
            PHY_REG_WT(0x40030040,0x00030010);
        }

        //----------------------------------
        //PRBS SEED should be configed before tx_mod en
        PHY_REG_WT(0x40030044,RF_PHY_DTM_PRBS9_SEED );

        if(g_dtmModeType == RF_PHY_DTM_MODE_TX_SINGLE)
        {
            PHY_REG_WT(0x400300a4,0x00000100);      //tp_mod en and pa_ramp_en
        }
        else
        {
            PHY_REG_WT(0x400300a4,0x00000140);      //tp_mod en and pa_ramp_en
        }

        //PHY_REG_WT(0x40030040,0x000300f0);//need to extend the preamble length

        if(g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST)
        {
            //DCDC_CONFIG_SETTING(0x08);
            if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
            {
                #if 0

                //[15:8] payload Len [7:5] preamble len, [4] tx mode, [3:0] payload type
                //PHY_REG_WT(0x40030040,(0x00030010|(g_dtmLength<<8)|(preambleLen<<5)|(0x0f & g_dtmPKT)) );
                if((g_rfPhyPktFmt == PKT_FMT_BLR125K || g_rfPhyPktFmt == PKT_FMT_BLR500K)
                        && g_dtmPKT == 0x03 )
                {
                    //for PKT_FMT all ones 11111111
                    //PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength+2)<<8)|(preambleLen<<5)|0x04) );
                    PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength)<<8)|(preambleLen<<5)|0x04) );
                }
                else
                {
                    //PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength+2)<<8)|(preambleLen<<5)|(0x0f & g_dtmPKT)) );
                    PHY_REG_WT(0x40030040,(0x00030010|((g_dtmLength)<<8)|(preambleLen<<5)|(0x0f & g_dtmPKT)) );
                }

                #else
                ll_hw_rst_tfifo();
                extern void rf_phy_dtm_ble_pkt_gen(void);
                rf_phy_dtm_ble_pkt_gen();
                #endif
            }
            else
            {
                ll_hw_rst_tfifo();
                //rf_phy_dtm_zigbee_pkt_gen();
            }
        }
        else
        {
            if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
            {
                //[15:8] payload Len [7:5] preamble len, [4] tx mode, [3:0] payload type
                PHY_REG_WT(0x40030040,0x00030010|(preambleLen<<5));
            }
            else
            {
                //[15:8] payload Len [7:5] preamble len, [4] tx mode, [3:0] payload type
                PHY_REG_WT(0x40030040,0x000b0013|(preambleLen<<5));
                PHY_REG_WT(0x40030000,0x78068002);
            }
        }

        //close ll hw irg
        ll_hw_set_irq(0);
        ll_hw_set_stx();
        ll_hw_clr_irq();

        if(g_dtmModeType == RF_PHY_DTM_MODE_TX_BURST)
        {
            if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
            {
                //PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
                //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
                ll_hw_trigger();
            }
            else
            {
                ll_hw_trigger();
                //PHY_REG_WT( 0x400300a0,0x0000000e);     // clr pll_auto override
                //PHY_REG_WT( 0x400300a4,0x00000154);     // clr tx_auto
            }

            g_dtmTick = read_current_fine_time();
        }
        else
        {
            ll_hw_trigger();
        }
    }
    else if(   g_dtmModeType == RF_PHY_DTM_MODE_RX_PER
               || g_dtmModeType == RF_PHY_DTM_MODE_GET_PER_AUTO )
    {
        rf_phy_ana_cfg();
        rf_rxDcoc_cfg(/*rfChn*/2,/*bwSet*/1,&g_rfPhyRxDcIQ);        //set the rfChn as 2402 for BW=1MHz
        //====== rf initial
        rf_phy_ini1();
        ll_hw_set_timing(g_rfPhyPktFmt);
        PHY_REG_WT(0x40031024,g_dtmPktIntv-100);//timeout set
        PHY_REG_WT(0x40031028,g_dtmPktIntv-100);//timeout set

        if((rfChn&0x0f)==0)
        {
            PHY_REG_WT(0x400300b4,rfChn);//freqOffset = 0
            subWriteReg(0x4003011c,11,8,3);//enable spur notch filter setting
        }
        else
        {
            subWriteReg(0x4003011c,11,8,0);//disable spur notch filter setting

            if(g_rfPhyFreqOffSet>=0)
			{
                PHY_REG_WT(0x400300b4,rfChn+(g_rfPhyFreqOffSet<<16));
            }
			else
			{
                PHY_REG_WT(0x400300b4,rfChn-1+((255+g_rfPhyFreqOffSet)<<16));
			}
			
        }

        if(!(g_rfPhyPktFmt==PKT_FMT_ZIGBEE))
        {
            PHY_REG_WT(0x40030044,RF_PHY_DTM_PRBS9_SEED );
            PHY_REG_WT(0x40030048,RF_PHY_DTM_CRC_WT     );
            PHY_REG_WT(0x4003004c,g_dtmAccessCode  );
        }

        //set rx no timeout[31:16] + max packet len
        //PHY_REG_WT(0x4003000c,(g_dtmLength+3)>255 ? 255: g_dtmLength+3 );
        PHY_REG_WT(0x4003000c,(g_dtmLength+3)>255 ? 255:( g_dtmLength<40 ? 40 : g_dtmLength+3) );
        //close ll hw irg
        ll_hw_set_irq(0);
        ll_hw_set_srx();
        ll_hw_clr_irq();
        ll_hw_trigger();
    }
}
void gen_pn_prbs9(uint16_t seed, int length,uint8_t* pnOut)
{
    uint8_t bitOut[8] = {0};
    uint8_t reg[9];
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t feedback = 0;

    for (i = 0; i < 9; i++)
    {
        reg[i] = (seed >> i) & 0x01;
    }

    for (i = 0; i < length; i++)
    {
        for (j = 0; j < 8; j++)
        {
            feedback = reg[5] ^ reg[0];
            bitOut[j] = reg[0];
            reg[0] = reg[1];
            reg[1] = reg[2];
            reg[2] = reg[3];
            reg[3] = reg[4];
            reg[4] = reg[5];
            reg[5] = reg[6];
            reg[6] = reg[7];
            reg[7] = reg[8];
            reg[8] = feedback;
        }

        bit_to_byte(bitOut, pnOut + i);
    }
}
void rf_phy_dtm_ble_pkt_gen(void)
{
    uint8_t tmp[256];
    tmp[1] = g_dtmLength;
    tmp[0] = g_dtmPKT;
    uint8_t pld=0x00;

    if(g_dtmPKT==1)
    {
        pld = 0x0f;
    }
    else if(g_dtmPKT==2)
    {
        pld=0x55;
    }
    else if(g_dtmPKT==3)
    {
        pld=0xff;
        tmp[0]=4;
    }

    if(g_dtmPKT==0)
    {
        gen_pn_prbs9(0x01ff,g_dtmLength,&(tmp[2]));
    }
    else
    {
        for(uint8_t i=0; i<g_dtmLength; i++)
            tmp[2+i] =pld;
    }

    PHY_REG_WT(0x40030040,0x00030000);

    if(g_rfPhyPktFmt==PKT_FMT_BLE1M)
    {
        subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>3) ); // 1byte -> 8us
    }
    else if(g_rfPhyPktFmt == PKT_FMT_BLE2M)
    {
        subWriteReg(0x40030040, 7, 5, (RF_PHY_EXT_PREAMBLE_US>>2) );//2 byte -> 8us
    }

    ll_hw_write_tfifo(tmp,g_dtmLength+2);//include the crc16
}

#if 0
void rf_phy_dtm_zigbee_pkt_gen(void)
{
    uint8_t seed[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t crcCode[2]= {0xff,0xff};
    uint8_t i=0;
    uint8_t tmp[128];
    tmp[0] = g_dtmLength;

    for(i=0; i<g_dtmLength-1; i++)
    {
        tmp[i+1] = ( (((tmp[i]&0x01)<<7) | (tmp[i]>>1) ) ^ 0x61);
    }

    zigbee_crc16_gen(tmp+1,g_dtmLength-2,seed,crcCode);
    tmp[g_dtmLength-1] = crcCode[0];
    tmp[g_dtmLength  ] = crcCode[1];
    ll_hw_write_tfifo(tmp,g_dtmLength+1);//include the crc16
}
#endif

#endif
//-------------------------------------------------------------------------------------

