#include "rom_sym_def.h"
#include "types.h"
#include "stdio.h"
#include "unistd.h"
#include "usb_def.h"
#include "log.h"
#include "gpio.h"
#include "bus_dev.h"
#include "usb_hal_pcd.h"
#include "OSAL.h"
#include "clock.h"

uint32_t usb_ep_tx_push(uint8_t *buf, uint32_t len);

#if 0
int32_t  hal_pcd_enable_set_desc_func(usb_hal_pcd_t *hpcd)
{
    dev_cfg_t dev_cfg ;
    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    /*Clear Set Descriptor Interrupt  */
    dev_cfg.d32 = hpcd->dev_glb_regs->DEV_CFG;
    dev_cfg.b.SET_DESC_SUP = 1;
    hpcd->dev_glb_regs->DEV_CFG = dev_cfg.d32;

    return HAL_OK;
}


int32_t  hal_pcd_disable_set_desc_func(usb_hal_pcd_t *hpcd)
{
    dev_cfg_t dev_cfg ;

    /*Clear Set Descriptor Interrupt  */
    dev_cfg.d32 = hpcd->dev_glb_regs->DEV_CFG;
    dev_cfg.b.SET_DESC_SUP = 0;
    hpcd->dev_glb_regs->DEV_CFG = dev_cfg.d32;
    return HAL_OK;
}
#endif

#if 0
int32_t  hal_pcd_clr_set_interface_int(usb_hal_pcd_t *hpcd)
{
    dev_int_sts_t dev_int_clr;
    dev_int_clr.d32 = 0;
    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    /*Clear Set interface Interrupt  */
    dev_int_clr.b.SI_INT_STS = 1;
    //LOG("USB Set Interface\n");
    hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;

    return HAL_OK;
}

int32_t  hal_pcd_clr_set_configuration_int(usb_hal_pcd_t *hpcd)
{
    dev_int_sts_t dev_int_clr;
    dev_int_clr.d32 = 0;
    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    /*Clear Set configuration Interrupt  */
    dev_int_clr.b.SC_INT_STS = 1;
    //LOG("USB Set Interface\n");
    hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;

    return HAL_OK;
}
#endif



#if 0
int32_t hal_pcd_set_tx_fifo_size(usb_hal_pcd_t *hpcd, uint16_t ep_num, uint16_t fifo_depth)
{
    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }
    hpcd->in_ep_regs[ep_num]->EP_IN_FIFO_SZ = fifo_depth;

    return HAL_OK;
}

int32_t hal_pcd_set_threshold(usb_hal_pcd_t *hpcd, uint16_t rx_thd, uint16_t tx_thd)
{
    dev_rx_tx_thd_t rx_tx_thd;

    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }
    rx_tx_thd.b.TX_THD = tx_thd;
    rx_tx_thd.b.RX_THD = rx_thd;

    hpcd->dev_glb_regs->DEV_THD = rx_tx_thd.d32;

    return HAL_OK;
}
#endif


static int32_t hal_pcd_enable_out_ep_interrupt(usb_hal_pcd_t *hpcd, uint32_t ep_num)
{
    dev_glb_ep_int_en_t ep_int_en;

    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    ep_int_en.d32 = hpcd->dev_glb_regs->DEV_GLB_EP_INT_EN;

    ep_int_en.b.OUT_EP_INT_EN |= (1 << ep_num);

    hpcd->dev_glb_regs->DEV_GLB_EP_INT_EN = ep_int_en.d32;

    return HAL_OK;
}

#if 0
int32_t hal_pcd_disable_out_ep_interrupt(usb_hal_pcd_t *hpcd, uint32_t ep_num)
{
    dev_glb_ep_int_en_t ep_int_en;

    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    ep_int_en.d32 = hpcd->dev_glb_regs->DEV_GLB_EP_INT_EN;

    ep_int_en.b.OUT_EP_INT_EN &= (~(1 << ep_num));

    hpcd->dev_glb_regs->DEV_GLB_EP_INT_EN =  ep_int_en.d32;

    return HAL_OK;
}


int32_t hal_pcd_enable_setup_interrupt(usb_hal_pcd_t *hpcd)
{
    int32_t ret = HAL_OK;
    dev_int_en_t dev_int_en;


    ret = hal_pcd_enable_out_ep_interrupt(hpcd, 0);
    if (ret != HAL_OK)
    {
        return ret;
    }

    dev_int_en.d32  = hpcd->dev_glb_regs->DEV_INT_EN;
    dev_int_en.b.SETUP_INT_EN = USB_INT_ENABLE;

    hpcd->dev_glb_regs->DEV_INT_EN = dev_int_en.d32;

    return ret;
}

int32_t hal_pcd_enable_rx_fifo_interrupt(usb_hal_pcd_t *hpcd, uint8_t ep_num)
{
    int32_t ret = HAL_OK;
    dev_int_en_t dev_int_en;

    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    ret = hal_pcd_enable_out_ep_interrupt(hpcd, ep_num);
    if (ret != HAL_OK)
    {
        return ret;
    }

    dev_int_en.d32  = hpcd->dev_glb_regs->DEV_INT_EN;
    dev_int_en.b.OUT_INT_EN  = USB_INT_ENABLE;
    hpcd->dev_glb_regs->DEV_INT_EN = dev_int_en.d32;

    return ret;
}
#endif



int32_t hal_pcd_ep_dir_set(usb_hal_pcd_t *hpcd, uint8_t ep_num, hal_ep_dir_t dir)
{
    dev_ep_info_t ep_info;
    dev_cfg_t  dev_cfg;

    ep_info.d32 = 0;
    ep_info.d32 = hpcd->mac_ep_regs->DEV_EP_INFO[ep_num];
    ep_info.b.EP_DIR = dir;
    hpcd->mac_ep_regs->DEV_EP_INFO[ep_num] = ep_info.d32;

    dev_cfg.d32 = hpcd->dev_glb_regs->DEV_CFG;
    dev_cfg.b.CSR_DONE = 1;
    hpcd->dev_glb_regs->DEV_CFG = dev_cfg.d32;

    return HAL_OK;
}

int32_t hal_pcd_ep_info_set(usb_hal_pcd_t *hpcd, uint8_t ep_num, hal_ep_dir_t dir, hal_ep_type_t type, uint16_t max_pkt_sz)
{
    dev_ep_info_t ep_info;
    dev_cfg_t  dev_cfg;

    //LOG("set ep info func\n");

    ep_info.d32 = 0;
    ep_info.d32 = hpcd->mac_ep_regs->DEV_EP_INFO[ep_num];
    ep_info.b.EP_DIR = dir;
    ep_info.b.EP_NUM = ep_num;
    ep_info.b.EP_TYPE = type;
    ep_info.b.MAX_PKT = max_pkt_sz;
    //LOG("ep_info config %x\n", ep_info.d32);
    hpcd->mac_ep_regs->DEV_EP_INFO[ep_num] = ep_info.d32;

    dev_cfg.d32 = hpcd->dev_glb_regs->DEV_CFG;
    dev_cfg.b.CSR_DONE = 1;
    hpcd->dev_glb_regs->DEV_CFG = dev_cfg.d32;

    return HAL_OK;
}

static int32_t hal_pcd_ep_fifo_flush(volatile AP_USB_EP_TypeDef * ep_reg)
{
    ep_in_ctl_t ep_in_ctl;

    ep_in_ctl.d32 = ep_reg->in.EP_IN_CTL;
    ep_in_ctl.b.FIFO_READY = 0;
    ep_in_ctl.b.FLUSH_FIFO = 1;
    ep_reg->in.EP_IN_CTL = ep_in_ctl.d32;

    return HAL_OK;
}

int32_t hal_pcd_send_data(usb_hal_pcd_t *hpcd, uint8_t ep_num, uint8_t *buf, uint32_t len)
{
    dev_int_en_t dev_int_en;
    dev_int_en.d32  = hpcd->dev_glb_regs->DEV_INT_EN;
    dev_int_en.b.SOF_INT_EN        = USB_INT_ENABLE;//USB_INT_ENABLE;	//USB_INT_DISABLE;	//
    hpcd->dev_glb_regs->DEV_INT_EN = dev_int_en.d32;
    
    return usb_ep_tx_push(buf, len);
}

int32_t hal_pcd_send_data_old(usb_hal_pcd_t *hpcd, uint8_t ep_num, uint8_t *buf, uint32_t len)
{
    uint16_t xfer_size;
    uint32_t i = 0;
    ep_in_ctl_t tx_ctl;
    ep_in_sts_t tx_sts_clr;


    if (hpcd->ep[ep_num].is_in)
    {
        hpcd->ep[ep_num].xfer_buf    = buf;
        hpcd->ep[ep_num].xfer_length = len;

        xfer_size = hpcd->ep[ep_num].xfer_length - hpcd->ep[ep_num].xfer_tmpcnt;

        /* Don't Write more than max packet size */
        if (xfer_size > hpcd->ep[ep_num].max_packet)
        {
            xfer_size = hpcd->ep[ep_num].max_packet;
        }

        /*2. Write trnasfer Size   */
        //hpcd->in_ep_regs[ep_num]->EP_IN_TRANS_SZ = xfer_size;

//        LOG_DEBUG("hpcd->ep[ep_num].fifo_depth %d\n", hpcd->ep[ep_num].fifo_depth );
        /* Don't Write More than threshold size */
        if (xfer_size > (hpcd->tx_thd << 2)) /*fifo width is 32bits */
        {
            xfer_size = hpcd->tx_thd << 2;
        }
//        LOG_DEBUG("xfer size is %d\n", xfer_size);
        /*3. Write Data            */
        /* If Data to write  */
        #if 0
        if (xfer_size > 0)
        {
            uint8_t *data = hpcd->ep[ep_num].xfer_buf + hpcd->ep[ep_num].xfer_tmpcnt;
            volatile uint32_t *fifo = hpcd->tx_fifo[ep_num];

            /*Upate temporary data pointer  */
            hpcd->ep[ep_num].xfer_tmpcnt += xfer_size;
			//gpio_write(P14, 1);
			//gpio_write(P14, 0);

            while (xfer_size >= 4)
            {
                write_reg(fifo,  *(uint32_t *)(data + i * 4));
                i++;
                xfer_size -= 4;
            }

            if (xfer_size)
            {
                int temp = 0;
                data += xfer_size;
                while (xfer_size--)
                {
                    temp <<= 8;
                    temp |= *--data;
                }

                write_reg(fifo, temp);
            }
        }
        #else
        {
            uint8_t *data = hpcd->ep[ep_num].xfer_buf;
            volatile uint32_t *fifo = hpcd->tx_fifo[ep_num];
            xfer_size = len;

            /*1. Clear Pakcet Status  */
            tx_sts_clr.b.DATA_SENT = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_STS = tx_sts_clr.d32;

            /*2. Clear Endpoint FIFO  */
            tx_ctl.d32 = 0;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;

            tx_ctl.b.FLUSH_FIFO = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;

            hal_pcd_in_ep_fifo_ready_clr(hpcd, ep_num);

            hpcd->in_ep_regs[ep_num]->in.EP_IN_TRANS_SZ = len;


            /*Upate temporary data pointer  */
            hpcd->ep[ep_num].xfer_tmpcnt = xfer_size;
			// gpio_write(P14, 1);
			// gpio_write(P14, 0);

            while (xfer_size >= 4)
            {
                write_reg(fifo,  *(uint32_t *)(data + i * 4));
                i++;
                xfer_size -= 4;
            }

            if (xfer_size)
            {
                int temp = 0;
                data += xfer_size;
                while (xfer_size--)
                {
                    temp <<= 8;
                    temp |= *--data;
                }
                write_reg(fifo, temp);
            }
        }
        #endif

        /*4. Set FIFO Ready        */
        tx_ctl.d32 = hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL;
        tx_ctl.b.FIFO_READY = 1;
        hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;

        tx_sts_clr.d32 = 0;
        tx_sts_clr.b.NAK_SENT = 1;
        hpcd->in_ep_regs[ep_num]->in.EP_IN_STS = tx_sts_clr.d32;

    }
    else
    {
        LOG_ERROR("transfer error\n");
        return -HAL_ERROR;
    }

    return HAL_OK;
}


void hal_remote_wakeup(usb_hal_pcd_t *hpcd)
{
#if 0
    int i;
    uint32_t ctl_addr = 0x14044c94;
    volatile otg_ctl_sts_t otg_ctl_sts;
	uint32_t data;
    //otg_ctl_sts.d32 = hpcd->



    //reset_flag = 0;
    //printk("prepare send srp\n");
    data = read_reg(ctl_addr);
    //otg_ctl_sts.b.SRPCAP = 1;
	//otg_ctl_sts.d32 |= BIT(20);
    //write_reg(ctl_addr, otg_ctl_sts.d32);
    //otg_ctl_sts.b.SR = 1;
	data |= BIT(16);

    write_reg(ctl_addr, data);
    do
    {
        data = read_reg(ctl_addr);
    } while (data & BIT(0) == 0);

    for (i = 0; i < 5; i++)
    {
        data = read_reg(ctl_addr);
        if (data & BIT(0) != 1)
            break;
    }
    if (i < 5)
    {
        return ;
    }
    else
    {
        otg_ctl_sts.d32 = 0;
        otg_ctl_sts.b.SRSCHG = 1;
        write_reg(ctl_addr, BIT(1));
        //srp done = 1;
        //while (reset_flag == 0);
    }

    return ;
    
#elif(1)
    gpio_write(P18, 1);
    gpio_write(P19, 0);
    WaitMs(10);

    gpio_pull_set(P18, GPIO_FLOATING);
    gpio_pull_set(P19, GPIO_FLOATING);
    gpio_dir(P18, GPIO_INPUT);
    gpio_dir(P19, GPIO_INPUT);
#elif(0)
    write_reg(0x4000f018, read_reg(0x4000f018) | ( BIT(17)));
    gpio_pull_set(P1, GPIO_PULL_DOWN);

    write_reg(0x4000f018, read_reg(0x4000f018) & ( ~BIT(17)));
    gpio_pull_set(P1, GPIO_FLOATING);
#else
    otg_dm_ctl_cfg_t dm_ctl;
    otg_dp_ctl_cfg_t dp_ctl;

    dm_ctl.d32 = hpcd->usb_top_regs->DM_CTL;
    dm_ctl.b.DM_O = 1;
    dm_ctl.b.DM_OE_N = 0;
    dm_ctl.b.FORCE_DM_O_EN = 1;
    dm_ctl.b.FORCE_DM_OE_EN = 1;
    hpcd->usb_top_regs->DM_CTL = dm_ctl.d32;

    dp_ctl.d32 = hpcd->usb_top_regs->DP_CTL;
    dp_ctl.b.DP_O = 0;
    dp_ctl.b.DP_OE_N = 0;
    dp_ctl.b.FORCE_DP_O_EN = 1;
    dp_ctl.b.FORCE_DP_OE_EN = 1;
    hpcd->usb_top_regs->DP_CTL = dp_ctl.d32;
    //WaitMs(1);
    WaitUs(100);
    hpcd->usb_top_regs->DM_CTL = 0;
    hpcd->usb_top_regs->DP_CTL = 0;

    

#endif
}

#if 0
int32_t hal_pcd_recv_data_prepare(usb_hal_pcd_t *hpcd, uint8_t ep_num, uint8_t *buf, uint32_t length)
{
    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    hpcd->ep[ep_num].xfer_buf    = buf;
    hpcd->ep[ep_num].xfer_length = length;
    hpcd->ep[ep_num].xfer_cnt    = 0;
    hpcd->ep[ep_num].xfer_tmpcnt = 0;

    hal_pcd_out_ep_fifo_ready_set(hpcd, ep_num);

    return 0;
}

int32_t hal_pcd_recv_data_len_get(usb_hal_pcd_t *hpcd, uint8_t ep_num)
{
    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    if (hpcd->ep[ep_num].lock == HAL_LOCKED)
    {
        return -HAL_ERROR;
    }

    if (hpcd->ep[ep_num].xfer_length != hpcd->ep[ep_num].xfer_cnt)
    {
        return -HAL_ERROR;
    }

    return hpcd->ep[ep_num].xfer_length;
}

int32_t hal_pcd_recv_data_get(usb_hal_pcd_t *hpcd, uint8_t ep_num, uint8_t *buf)
{
    uint32_t i = 0;

    if (hpcd == NULL)
    {
        return -HAL_ERROR;
    }

    if (hpcd->ep[ep_num].lock == HAL_LOCKED)
    {
        return -HAL_ERROR;
    }

    if (hpcd->ep[ep_num].xfer_length != hpcd->ep[ep_num].xfer_cnt)
    {
        return -HAL_ERROR;
    }

    for (i = 0; i < hpcd->ep[ep_num].xfer_length; i++)
    {
        buf[i] = hpcd->ep[ep_num].xfer_buf[i];
    }
    return HAL_OK;
}
#endif

void usb_pin_init(void)
{

  gpio_fmux_set(P0, FMUX_P0_USBDP);
  write_reg(0x4000f018, read_reg(0x4000f018) & ( ~BIT(17)));
  gpio_fmux_set(P1, FMUX_P1_USBDM);

  //gpio_fmux_set(P6, FMUX_P6_USBDP);
  //write_reg(0x4000f018, read_reg(0x4000f018) & ( ~BIT(18)));
  //gpio_fmux_set(P7, FMUX_P7_USBDM);

}

void clk_set_usb(void)
{
	AP_PCRM->CLKHF_CTL1 |= BIT(14);
	AP_PCRM->CLKHF_CTL1 |= BIT(7);
}

int32_t hal_pcd_init(usb_hal_pcd_t *hpcd)
{
    uint32_t i = 0;
    dev_ep_info_t ep_info;
    dev_cfg_t dev_cfg ;
    dev_int_en_t dev_int_en;
    dev_rx_tx_thd_t dev_thd;
    otg_dm_ctl_cfg_t  dm_ctl;
    otg_dp_ctl_cfg_t  dp_ctl;
//    otg_phy_ctl_t  phy_ctl;

    //id_pull_t   id_pull;
    //NVIC_SetPriority(USB_IRQn, IRQ_PRIO_HAL);
    NVIC_EnableIRQ(USB_IRQn);
//	LOG("int_val = %x\n", (* (volatile int *) 0xe000e100));	//0x4f00810
    /*init pcd register struct */
    for (i = 0; i < MAX_EP_NUM; i++)
    {
        hpcd->in_ep_regs[i]   = (AP_USB_EP_TypeDef *)OTG_PCD_IN_EP_REG(i);
        hpcd->tx_fifo[i]      = (volatile uint32_t *)OTG_PCD_IN_EP_FIFO(i);
        hpcd->out_ep_regs[i]  = (AP_USB_EP_TypeDef *)OTG_PCD_OUT_EP_REG(i);
        hpcd->rx_fifo[i]      = (volatile uint32_t *)OTG_PCD_OUT_EP_FIFO(i);
    }
    hpcd->mac_ep_regs  = (AP_USB_DEV_MAC_TypeDef *)OTG_PCD_MAC_REG_ADDR;
    hpcd->dev_glb_regs = (AP_USB_DEV_GLB_TypeDef *)OTG_PCD_GLB_REG_ADDR;
    hpcd->dev_diu_regs = (AP_USB_DIU_TypeDef *)OTG_DIU_REG_ADDR;
    hpcd->usb_top_regs = (AP_USB_TOP_TypeDef *)OTG_TOP_REG_ADDR;

    //LOG("mac_ep_regs  base addr 0x%08x\n", hpcd->mac_ep_regs);
    //LOG("dev_glb_regs base addr 0x%08x\n", hpcd->dev_glb_regs);
    //LOG("dev_diu_regs base addr 0x%08x\n", hpcd->dev_diu_regs);
    //LOG("usb_top_regs base addr 0x%08x\n", hpcd->usb_top_regs);


    /*init usb work mode */
    dev_cfg.d32            = hpcd->dev_glb_regs->DEV_CFG;
    dev_cfg.b.SPEED        = USB_FS_SPEED_MODE;
    dev_cfg.b.SELF_PWR     = USB_CSR_BUS_POWERED;
    dev_cfg.b.RMT_WKUP     = USB_REMOTE_WAKEUP_SUPOORT;
    dev_cfg.b.SYNC_FRAME   = USB_SYNC_FRAME_UNSUPPORT;
    dev_cfg.b.HST_MODE     = USB_DEVICE_MODE;
    dev_cfg.b.CSR_PRG_SUP  = 1;
    dev_cfg.b.SET_DESC_SUP = 0;
    dev_cfg.b.SCALE_DOWN   = 0;
    hpcd->dev_glb_regs->DEV_CFG = dev_cfg.d32;
    //LOG("DEV_CFG = %x\n", hpcd->dev_glb_regs->DEV_CFG);

    /*! Init Out Endpoint RX FIFO Size  */
    hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_SZ = PCD_OUT_EP_RX_FIFO_DEPTH;

    //LOG("DEV_OUT_EP_FIFO_SZ = %x\n", hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_SZ);

    /*! Global Endpoint Interrupt Enable  */
    //hal_pcd_enable_in_ep_interrupt(hpcd, 0);
    hal_pcd_enable_out_ep_interrupt(hpcd, 0);
    //hal_pcd_enable_in_ep_interrupt(hpcd, 1);
    //hal_pcd_enable_out_ep_interrupt(hpcd, 1);

    //hal_pcd_enable_in_ep_interrupt(hpcd, 1);
    //hal_pcd_enable_out_ep_interrupt(hpcd, 2);
    /*! init int control */
    hpcd->in_ep_regs[0]->in.EP_IN_FIFO_SZ = 16;
    //LOG("EP 0 EP_IN_FIFO_SZ = %x\n", hpcd->in_ep_regs[0]->EP_IN_FIFO_SZ);

    hpcd->ep[0].fifo_depth = 16;
    hpcd->ep[0].max_packet = 64;

    dev_thd.d32 = 0 ;
    dev_thd.b.RX_THD = 8;
    dev_thd.b.TX_THD = 8;
    hpcd->rx_thd_en = true;
    hpcd->rx_thd = 8;
    hpcd->tx_thd_en = true;
    hpcd->tx_thd = 8;
    hpcd->dev_glb_regs->DEV_THD = dev_thd.d32;

    ep_info.d32 = hpcd->mac_ep_regs->DEV_EP_INFO[0];
    ep_info.b.EP_DIR = HAL_PCD_EP_DIR_OUT ;
    ep_info.b.EP_NUM = 0;
    ep_info.b.EP_TYPE = HAL_PCD_EP_TYPE_CTRL;
    ep_info.b.MAX_PKT = 64;
    hpcd->mac_ep_regs->DEV_EP_INFO[0] =  ep_info.d32;
    //LOG("ep_info config %x\n", hpcd->mac_ep_regs->DEV_EP_INFO[0]);

    dev_cfg.d32 = hpcd->dev_glb_regs->DEV_CFG;
    dev_cfg.b.CSR_DONE = 1;
    hpcd->dev_glb_regs->DEV_CFG = dev_cfg.d32;
    //LOG("DEV_CFG = %x\n", hpcd->dev_glb_regs->DEV_CFG);

    //ep_in_ctl.d32 = hpcd->in_ep_regs[0]->EP_IN_CTL;
    //ep_in_ctl.b.FIFO_READY = 0;
    //ep_in_ctl.b.FLUSH_FIFO = 1;
    //hpcd->in_ep_regs[0]->EP_IN_CTL     = ep_in_ctl.d32;

    hal_pcd_ep_fifo_flush(hpcd->in_ep_regs[0]);
    hal_pcd_ep_fifo_flush(hpcd->out_ep_regs[0]);

    //hal_pcd_ep_info_set(hpcd, 0, HAL_PCD_EP_DIR_OUT, HAL_PCD_EP_TYPE_CTRL, 64);
    //LOG("tx fifo deepth = %x\n",    hpcd->in_ep_regs[0]->EP_IN_FIFO_SZ);


    //LOG("DEV CFG Init\n");


    //LOG("DEV Int EN\n");

	//Disconnect
	dm_ctl.d32 = hpcd->usb_top_regs->DM_CTL;
	dm_ctl.b.DM_O  = 0;
	dm_ctl.b.DM_OE_N = 0;
	dm_ctl.b.FORCE_DM_O_EN = 1;
	dm_ctl.b.FORCE_DM_OE_EN = 1;
	hpcd->usb_top_regs->DM_CTL = dm_ctl.d32;

	dp_ctl.d32 = hpcd->usb_top_regs->DP_CTL;
	dp_ctl.b.DP_O  = 0;
	dp_ctl.b.DP_OE_N = 0;
	dp_ctl.b.FORCE_DP_O_EN = 1;
	dp_ctl.b.FORCE_DP_OE_EN = 1;
	hpcd->usb_top_regs->DP_CTL = dp_ctl.d32;

	WaitUs(100);
    //Connect
	hpcd->usb_top_regs->DM_CTL = 0;
	hpcd->usb_top_regs->DP_CTL = 0;

    //id_pull.d32 = hpcd->usb_top_regs->ID_PULL;
    //id_pull.b.ID_PU = 1;
    //hpcd->usb_top_regs->ID_PULL = id_pull.d32 ;

/*
    phy_ctl.d32 = hpcd->usb_top_regs->PHY_CTL0;
    phy_ctl.b.FORCE_VBUS_VLID_EN = 1;
    phy_ctl.b.VBUSVLD = 1;
    phy_ctl.b.FORCE_SESSION_VLD_EN = 1;
    phy_ctl.b.SESSVLD = 1;
    hpcd->usb_top_regs->PHY_CTL0 = phy_ctl.d32;
*/
    /*Init Int Regiser */
    dev_int_en.d32  = 0;
    dev_int_en.b.SC_INT_EN         = USB_INT_ENABLE;
    dev_int_en.b.SI_INT_EN         = USB_INT_ENABLE;
    dev_int_en.b.UR_INT_EN         = USB_INT_ENABLE;
    dev_int_en.b.US_INT_EN         = USB_INT_ENABLE;
    dev_int_en.b.SOF_INT_EN        = USB_INT_DISABLE;//USB_INT_ENABLE;	//USB_INT_DISABLE;	//
    dev_int_en.b.OUT_INT_EN        = USB_INT_ENABLE;
    dev_int_en.b.SETUP_INT_EN      = USB_INT_ENABLE;
    dev_int_en.b.OTG_INT_EN        = USB_INT_DISABLE;
    hpcd->dev_glb_regs->DEV_INT_EN = dev_int_en.d32;
    //LOG("DEV_INT_EN = %x\n", hpcd->dev_glb_regs->DEV_INT_EN);

    /*delay 100us  */
	WaitUs(100);

    return 0;
}
