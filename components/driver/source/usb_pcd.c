#include "rom_sym_def.h"
#include "stdio.h"
#include "log.h"
#include "gpio.h"
#include "usb_hal_pcd.h"
#include "usb_usr_config.h"
#include "usb_pcd.h"
#include "hid.h"
#include "OSAL.h"

usb_hal_pcd_t pcd_handle;

extern uint8_t hid_mouse_ready;
extern uint8_t mouse_protocol;
extern uint8_t mouse_idle_rate;


usb_hid_q_t usb_ep_tx_q;
#define __gpio_write(a,b)

uint32_t usb_ep_tx_push(uint8_t *buf, uint32_t len)
{
    int32_t ret = HAL_OK;
    _HAL_CS_ALLOC_();
    HAL_ENTER_CRITICAL_SECTION();

    if(usb_ep_tx_q.num >= USB_HID_Q_NUM){
        ret =  HAL_BUSY;
    }
    else
    {
        int i;
        for(i = usb_ep_tx_q.num; i>0 ; i--){
            osal_memcpy(&(usb_ep_tx_q.pkg[usb_ep_tx_q.num].bdata[0]),
                &(usb_ep_tx_q.pkg[usb_ep_tx_q.num-1].bdata[0]), sizeof(usb_hid_pkg_t));
        }
        osal_memcpy(&(usb_ep_tx_q.pkg[0].bdata[0]), buf, len);
        usb_ep_tx_q.num++;
    }

    HAL_EXIT_CRITICAL_SECTION();
    return ret;
}


usb_hid_pkg_t* usb_ep_tx_pop(void)
{
    if(usb_ep_tx_q.num){
        usb_ep_tx_q.num --;
        return &(usb_ep_tx_q.pkg[usb_ep_tx_q.num]);
    }
    return NULL;
}

void usb_hid_tx_send(usb_hal_pcd_t *hpcd)
{
    int ep_num = 1;

    if(hid_mouse_ready)
    {
        ep_in_sts_t tx_sts_clr;
        ep_in_ctl_t tx_ctl;
        usb_hid_pkg_t* pkg = usb_ep_tx_pop();
        __gpio_write(P12, 1);
        __gpio_write(P12, 0);
        //WaitUs(10);
        if(pkg){
            uint32_t *data = pkg->wdata;
            volatile uint32_t *fifo = hpcd->tx_fifo[ep_num];

            /*1. Clear Pakcet Status  */
            tx_sts_clr.b.DATA_SENT = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_STS = tx_sts_clr.d32;

            /*2. Clear Endpoint FIFO  */
            tx_ctl.d32 = 0;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;

            tx_ctl.b.FLUSH_FIFO = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;

            hal_pcd_in_ep_fifo_ready_clr(hpcd, ep_num);

            hpcd->in_ep_regs[ep_num]->in.EP_IN_TRANS_SZ = USB_LENGTH_MOUSE_REPORT;


            /*Upate temporary data pointer  */
            hpcd->ep[ep_num].xfer_tmpcnt = 0;
            //gpio_write(P14, 1);
            //gpio_write(P14, 0);

            write_reg(fifo,  data[0]);
            write_reg(fifo,  data[1]);


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
            //no more packet, diable sof int
            dev_int_en_t dev_int_en;
            dev_int_en.d32  = hpcd->dev_glb_regs->DEV_INT_EN;
            dev_int_en.b.SOF_INT_EN        = USB_INT_DISABLE;//USB_INT_ENABLE;   //USB_INT_DISABLE;  //
            hpcd->dev_glb_regs->DEV_INT_EN = dev_int_en.d32;
        }
    }
}

uint8_t tx_st_tmp[16];
uint8_t tx_st_tmp_cnt = 0;


static void usb_pcd_svc_tx_endp(usb_hal_pcd_t *hpcd, int ep_num)
{
    ep_in_sts_t tx_sts;
    ep_in_sts_t tx_sts_clr;
    ep_in_ctl_t tx_ctl;

    uint16_t xfer_size = 0;
    volatile uint32_t *fifo = hpcd->tx_fifo[ep_num];
    uint8_t *data = NULL;
    uint32_t i = 0;
    uint32_t frame_num = 0;

    /* get endpoint status */

    tx_sts.d32 = hpcd->in_ep_regs[ep_num]->in.EP_IN_STS;
    tx_st_tmp[tx_st_tmp_cnt%16] = (uint8_t)(tx_sts.d32) | (ep_num << 6);
    tx_st_tmp_cnt++;

    if (!(tx_sts.b.STATUS | tx_sts.b.NAK_SENT | tx_sts.b.BELOW_THD))
        return;

    if (tx_sts.b.DATA_SENT)
    {
        if (tx_sts.b.STATUS)
        {
            /*sync send data count  */
            hpcd->ep[ep_num].xfer_cnt += hpcd->ep[ep_num].xfer_tmpcnt;
//            LOG_DEBUG("tx_xfer_cnt = %d, xfer_tmpcnt=%d\n", hpcd->ep[ep_num].xfer_cnt, hpcd->ep[ep_num].xfer_tmpcnt);

            if (ep_num == 0)
            {
                if (hpcd->ep[ep_num].xfer_cnt == 0)
                {
                    if (hpcd->ep0_status == HAL_PCD_EP0_IN_STATUS_PHASE)
                    {
                        hpcd->ep0_status = HAL_PCD_EP0_IDLE;
                        hal_pcd_out_ep_fifo_ready_set(hpcd, ep_num);
                        //LOG_DEBUG("tx_EP0 IN Status Phase\n");
                    }
                }
                else if (hpcd->ep[0].xfer_length == hpcd->ep[0].xfer_cnt)
                {
                    if (hpcd->ep0_status == HAL_PCD_EP0_IN_DATA_PHASE)
                    {
                        hpcd->ep0_status = HAL_PCD_EP0_OUT_STATUS_PHASE;
                        hal_pcd_ep_dir_set(hpcd, ep_num, HAL_PCD_EP_DIR_OUT);
                        hal_pcd_out_ep_fifo_ready_set(hpcd, ep_num);
                        //LOG_DEBUG("tx_Send OUT Zero Data\n");
                    }
                    hpcd->ep[ep_num].xfer_cnt = 0;
                    hpcd->ep[ep_num].xfer_tmpcnt = 0;
                    hpcd->ep[ep_num].xfer_length = 0;
                }
            }
            else
            {
                if (hpcd->ep[ep_num].xfer_length == hpcd->ep[ep_num].xfer_cnt)
                {
                    if (hpcd->ep[ep_num].sent_zlp)
                    {
                        //LOG_DEBUG("tx_send zero data\n");
                        hpcd->in_ep_regs[ep_num]->in.EP_IN_TRANS_SZ = 0;
                        hal_pcd_in_ep_fifo_ready_set(hpcd, ep_num);
                        hpcd->ep[ep_num].sent_zlp = 0;
                    }
                    hpcd->ep[ep_num].xfer_cnt = 0;
                    hpcd->ep[ep_num].xfer_tmpcnt = 0;
                    hpcd->ep[ep_num].xfer_length = 0;
                    hpcd->ep[ep_num].xfer_buf = NULL;
                    hal_pcd_in_ep_fifo_ready_clr(hpcd, ep_num);
                }
            }
            /*Clear Packet Status and Data Sent */
            tx_sts_clr.b.STATUS = 1;
            tx_sts_clr.b.DATA_SENT = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_STS = tx_sts_clr.d32;
        }
        else
        {
            /*1. Clear Pakcet Status  */
            tx_sts_clr.b.DATA_SENT = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_STS = tx_sts_clr.d32;

            /*2. Clear Endpoint FIFO  */
            tx_ctl.d32 = 0;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;

            tx_ctl.b.FLUSH_FIFO = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;

            hal_pcd_in_ep_fifo_ready_clr(hpcd, ep_num);

            /*3. Check is Data Underrun error and not sent error   */
            if (tx_sts.b.DATA_UNDERRUN_ERR)
            {
                LOG_ERROR("Data Underrun Error\n");
            }
            LOG_ERROR("Data Transfer error\n");
        }
        hpcd->ep[ep_num].xfer_tmpcnt = 0;
    }

    if (tx_sts.b.NAK_SENT)
    {
        __gpio_write(P13, 1);
        __gpio_write(P13, 0);

        if (hpcd->ep[ep_num].is_iso)
        {
            frame_num = hpcd->dev_glb_regs->FRAME_NUM;
            hpcd->dev_glb_regs->FRAME_NUM = frame_num;
            //LOG_DEBUG("tx_frame numner is %d\n", frame_num);
        }

        if (hpcd->ep[ep_num].xfer_buf != NULL)
        {
            /*1. Check Process Status   */
            if (hpcd->ep[ep_num].xfer_tmpcnt == 0)
            {
                if (hpcd->ep[ep_num].xfer_length % 64 == 0)
                {
                    hpcd->ep[ep_num].sent_zlp = 1;
                }

                xfer_size = hpcd->ep[ep_num].xfer_length - hpcd->ep[ep_num].xfer_cnt;
                data = hpcd->ep[ep_num].xfer_buf + hpcd->ep[ep_num].xfer_cnt;

                /* Don't Write more than max packet size */
                if (xfer_size > hpcd->ep[ep_num].max_packet)
                {
                    xfer_size = hpcd->ep[ep_num].max_packet;
                }

                /*2. Write trnasfer Size   */
                hpcd->in_ep_regs[ep_num]->in.EP_IN_TRANS_SZ = xfer_size;

                /* Don't Write More than fifo depth size */
                if (xfer_size > (hpcd->ep[ep_num].fifo_depth << 2))
                {
                    /*fifo width is 32bits */
                    xfer_size = hpcd->ep[ep_num].fifo_depth << 2;
                }

                /*3. Write Data            */
                /* If Data to write  */
                if (xfer_size > 0)
                {
                    /*Upate temporary data pointer  */
                    hpcd->ep[ep_num].xfer_tmpcnt += xfer_size;
                    while (xfer_size >= 4)
                    {
                        write_reg(fifo, *(uint32_t *)(data + i * 4));
                        i++;
                        xfer_size -= 4;
                    }

                    if (xfer_size)
                    {
                        int temp = 0;
                        data += (xfer_size + i * 4);
                        while (xfer_size--)
                        {
                            temp <<= 8;
                            temp |= *--data;
                        }
                        write_reg(fifo, temp);
                    }
                }

                /*4. Set FIFO Ready        */
                tx_ctl.d32 = hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL;
                tx_ctl.b.FIFO_READY = 1;
                hpcd->in_ep_regs[ep_num]->in.EP_IN_CTL = tx_ctl.d32;
            }

            /* Clear NAK_SENT        */
            tx_sts_clr.d32 = 0;
            tx_sts_clr.b.NAK_SENT = 1;
            hpcd->in_ep_regs[ep_num]->in.EP_IN_STS = tx_sts_clr.d32;

            //LOG_DEBUG("tx_xfer_length = %d,xfer_tmpcnt=%d\n", hpcd->ep[ep_num].xfer_length, hpcd->ep[ep_num].xfer_tmpcnt);

        }
        else
        {
            //LOG_ERROR("data is not ready\n");
            
            //hal_pcd_disable_in_ep_interrupt(hpcd, 0);
        }
    }



#if 0
repeat:
    tx_sts.d32 = hpcd->in_ep_regs[ep_num]->in.EP_IN_STS;
    if (tx_sts.b.BELOW_THD)
    {

        i = 0;
//        LOG_DEBUG("txr_below thd, xfer_tmp_cnt = %d\n", hpcd->ep[ep_num].xfer_tmpcnt);

        /*1.Write data of fifo threshold size or remaining data */

        xfer_size = hpcd->in_ep_regs[ep_num]->in.EP_IN_TRANS_SZ - hpcd->ep[ep_num].xfer_tmpcnt;
//        LOG_DEBUG("txr_xfer size %d\n", xfer_size);

        if (hpcd->ep[ep_num].is_iso)
        {
            frame_num = hpcd->dev_glb_regs->FRAME_NUM;
//            LOG_DEBUG("txr_frame numner is %d\n", frame_num);
            hpcd->dev_glb_regs->FRAME_NUM = frame_num;
        }

        /* Don't Write more than max packet size */

        if (xfer_size > hpcd->ep[ep_num].max_packet)
        {
            LOG_ERROR("transfer size calc error\n");
        }

        /* Don't Write More than threshold size */
        if (xfer_size > (hpcd->tx_thd << 2))
        {
            /*fifo width is 32bits */
            xfer_size = hpcd->tx_thd << 2;
        }

        /*3. Write Data            */
        /* If Data to write  */

        if (xfer_size > 0)
        {
            data = hpcd->ep[ep_num].xfer_buf + hpcd->ep[ep_num].xfer_cnt + hpcd->ep[ep_num].xfer_tmpcnt;
            /*Upate temporary data pointer  */
            hpcd->ep[ep_num].xfer_tmpcnt += xfer_size;

            while (xfer_size >= 4)
            {
                write_reg(fifo, *(uint32_t *)(data + i * 4));
                i++;
                xfer_size -= 4;
            }
//            LOG_DEBUG("txr_xfer size %d\n", xfer_size);
            if (xfer_size)
            {
                int temp = 0;
                data += (xfer_size + i * 4);
                while (xfer_size--)
                {
                    temp <<= 8;
                    temp |= *--data;
                }
                write_reg(fifo, temp);
            }
        }
        goto repeat;
    }
#endif
}

static void usb_pcd_svc_rx_endp(usb_hal_pcd_t *hpcd, uint16_t ep_num)
{
    //dev_int_sts_t dev_sts;
    dev_out_ep_fifo_sts_t out_fifo_sts;
    dev_out_ep_fifo_sts_t out_fifo_sts_clr;
    uint32_t rx_size = 0;


    out_fifo_sts.d32 = hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_STS;


    if (out_fifo_sts.b.STATUS_COMP)
    {

        if (out_fifo_sts.b.STATUS)
        {
            out_fifo_sts_clr.d32 = 0;
            out_fifo_sts_clr.b.STATUS_COMP = 1;
            hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_STS = out_fifo_sts_clr.d32;

            rx_size = out_fifo_sts.b.TRANSFER_SZ;

            //LOG_DEBUG("rxr_xfer_tmpcnt = %d, rx_size = %d\n", hpcd->ep[ep_num].xfer_tmpcnt, rx_size);

            while (rx_size >= 4)
            {
                *hpcd->rx_fifo[ep_num];
                rx_size -= 4;
            }

        }

        if (ep_num == 0)
        {
            if (out_fifo_sts.b.TRANSFER_SZ > 0)
            {
                if (hpcd->ep0_status == HAL_PCD_EP0_OUT_DATA_PHASE)
                {
                    hpcd->ep0_status = HAL_PCD_EP0_IN_STATUS_PHASE;
                    hal_pcd_ep_dir_set(hpcd, ep_num, HAL_PCD_EP_DIR_IN);
                    hal_pcd_set_in_ep_transfer_size(hpcd, ep_num, 0);
                    hal_pcd_in_ep_fifo_ready_set(hpcd, ep_num);
//                            LOG_DEBUG("rxr_ep tx ready\n");
                }
            }
            else
            {
                if (hpcd->ep0_status == HAL_PCD_EP0_OUT_STATUS_PHASE)
                {
                    hpcd->ep0_status = HAL_PCD_EP0_IDLE;
                    hal_pcd_ep_dir_set(hpcd, ep_num, HAL_PCD_EP_DIR_OUT);
                    // TODO Set FIFO Ready
                    hal_pcd_out_ep_fifo_ready_set(hpcd, ep_num);
//                        LOG_DEBUG("rxr_out status end\n");

                    hpcd->ep[0].xfer_cnt = 0;
                    hpcd->ep[0].xfer_tmpcnt = 0;
                    hpcd->ep[0].xfer_length = 0;
                    hpcd->ep[0].xfer_buf = NULL;
                }
            }
        }


            /*Clear Packet Status  */

            /*Read Remaining data  */

            /*Set FIFO Ready       */

            /*Please Get Data  */
    }

}

void usb_pcd_svc_rx_endp_old(usb_hal_pcd_t *hpcd, uint16_t ep_num)
{
    //dev_int_sts_t dev_sts;
    dev_out_ep_fifo_sts_t out_fifo_sts;
    dev_out_ep_fifo_sts_t out_fifo_sts_clr;
    uint32_t rx_temp = 0;
    int32_t i = 0;
    uint32_t *rx_ptr = NULL;
    uint32_t rx_size = 0;

    if ((hpcd->ep[ep_num].xfer_length != hpcd->ep[ep_num].xfer_tmpcnt) || (hpcd->ep[ep_num].xfer_length != 0))
    {
        rx_ptr = (uint32_t *)&hpcd->ep[ep_num].xfer_buf[hpcd->ep[ep_num].xfer_tmpcnt];
    }

    out_fifo_sts.d32 = hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_STS;
//    LOG_DEBUG("rx_out recv size %d\n", out_fifo_sts.b.TRANSFER_SZ);

    if (ep_num != out_fifo_sts.b.EP_NUM)
    {
        LOG_ERROR("ep number error\n");
    }

    if (out_fifo_sts.b.ABOVE_THD)
    {
        /*read data from fifo  */
read_rx_fifo_again:

        for (i = 0; i < hpcd->ep[ep_num].fifo_depth; i++)
        {
            *rx_ptr++ = *hpcd->rx_fifo[ep_num];
            hpcd->ep[ep_num].xfer_tmpcnt += 4;
        }

        out_fifo_sts.d32 = hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_STS;

        if (out_fifo_sts.b.ABOVE_THD)
        {
            goto read_rx_fifo_again;
        }
    }

    if (out_fifo_sts.b.STATUS_COMP)
    {

        if (out_fifo_sts.b.STATUS)
        {
            out_fifo_sts_clr.d32 = 0;
            out_fifo_sts_clr.b.STATUS_COMP = 1;
            hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_STS = out_fifo_sts_clr.d32;

            rx_size = out_fifo_sts.b.TRANSFER_SZ;

//            LOG_DEBUG("rxr_xfer_tmpcnt = %d, rx_size = %d\n", hpcd->ep[ep_num].xfer_tmpcnt, rx_size);

            if (hpcd->ep[ep_num].xfer_tmpcnt < rx_size)
            {
                rx_size = rx_size - hpcd->ep[ep_num].xfer_tmpcnt;
                //rx_size_temp = rx_size / 4;

                while (rx_size >= 4)
                {
                    *rx_ptr = *hpcd->rx_fifo[ep_num];
//                    LOG_DEBUG("rxr_addr:data %x:%x\n", rx_ptr, *rx_ptr);
                    //LOG_DEBUG("xfer_tmpcnt = %d, rx_size = %d\n", hpcd->ep[ep_num].xfer_tmpcnt, rx_size);
                    rx_ptr++;
                    hpcd->ep[ep_num].xfer_tmpcnt += 4;
                    rx_size -= 4;
                }

                if (rx_size > 0)
                {
                    rx_temp = *hpcd->rx_fifo[ep_num];
                    for (i = 0; i < rx_size; i++)
                    {
                        //LOG_DEBUG("offset:data %x : %02x :%08x\n", hpcd->ep[ep_num].xfer_tmpcnt, rx_temp >> (i * 8), rx_temp );
                        hpcd->ep[ep_num].xfer_buf[hpcd->ep[ep_num].xfer_tmpcnt++] = (rx_temp >> (i * 8)) & 0xff;
//                        LOG_DEBUG("unalign data\n");
                    }
                }
            }

            if (ep_num == 0)
            {
                if (out_fifo_sts.b.TRANSFER_SZ > 0)
                {
                    if (hpcd->ep[ep_num].xfer_length == out_fifo_sts.b.TRANSFER_SZ)
                    {
                        if (hpcd->ep0_status == HAL_PCD_EP0_OUT_DATA_PHASE)
                        {
                            hpcd->ep0_status = HAL_PCD_EP0_IN_STATUS_PHASE;
                            hal_pcd_ep_dir_set(hpcd, ep_num, HAL_PCD_EP_DIR_IN);
                            hal_pcd_set_in_ep_transfer_size(hpcd, ep_num, 0);
                            hal_pcd_in_ep_fifo_ready_set(hpcd, ep_num);
//                            LOG_DEBUG("rxr_ep tx ready\n");
                        }
                        else
                        {
                            LOG_ERROR("status error\n");
                        }
                    }
                    else
                    {
                        LOG_DEBUG("rxr_xfer size != transfer size \n", hpcd->ep[ep_num].xfer_length, out_fifo_sts.b.TRANSFER_SZ);
                    }
                }
                else
                {
                    if (hpcd->ep0_status == HAL_PCD_EP0_OUT_STATUS_PHASE)
                    {
                        hpcd->ep0_status = HAL_PCD_EP0_IDLE;
                        hal_pcd_ep_dir_set(hpcd, ep_num, HAL_PCD_EP_DIR_OUT);
                        // TODO Set FIFO Ready
                        hal_pcd_out_ep_fifo_ready_set(hpcd, ep_num);
//                        LOG_DEBUG("rxr_out status end\n");

                        hpcd->ep[0].xfer_cnt = 0;
                        hpcd->ep[0].xfer_tmpcnt = 0;
                        hpcd->ep[0].xfer_length = 0;
                        hpcd->ep[0].xfer_buf = NULL;
                    }
                }
            }
            else
            {
                if (hpcd->ep[ep_num].xfer_length >= out_fifo_sts.b.TRANSFER_SZ)
                {
//                    LOG_DEBUG("rxr_data recv end\n");
                    hpcd->ep[ep_num].xfer_cnt = hpcd->ep[ep_num].xfer_tmpcnt;
                    //hal_pcd_out_ep_fifo_ready_set(hpcd, ep_num);
                }
                else
                {
                    LOG_ERROR("recv data error: %d\n", out_fifo_sts.b.TRANSFER_SZ);
                }
            }

            /*Clear Packet Status  */

            /*Read Remaining data  */

            /*Set FIFO Ready       */

            /*Please Get Data  */
        }

    }
}



void usb_pcd_std_req_proc(usb_hal_pcd_t *hpcd, ureq_t req);
void usb_pcd_class_req_proc(usb_hal_pcd_t *hpcd, ureq_t req);

static void usb_pcd_setup_proc(usb_hal_pcd_t *hpcd, ureq_t req)
{
    //dev_ep_info_t ep_info;
    //dev_cfg_t dev_cfg;
    if (req->request_type & USB_REQ_TYPE_DIR_IN)
    {
        /*Enter Send Data State  */
        hpcd->ep0_status = HAL_PCD_EP0_IN_DATA_PHASE;
        if (req->wLength == 0)
        {
            /*Enter Send Status State  */
            //hpcd->ep0_status = HAL_PCD_EP0_STALL;
            
            //hpcd->ep[0].xfer_length = 0;
            //hpcd->ep[0].xfer_buf = NULL;
            //LOG_ERROR("error\n");
            /*

             * TODO

             * Send Out Zero Length Packet

             * */
        }

        if (!hpcd->ep[0].is_in)
        {
            hal_pcd_out_ep_fifo_ready_clr(hpcd, 0);
            if (hal_pcd_ep_dir_set(hpcd, 0, HAL_PCD_EP_DIR_IN) != HAL_OK)
            {
                LOG_ERROR("ep 0 ep_info set to in error\n");
                //return -1;
                return ;
            }
            hpcd->ep[0].is_in = 1;
        }

        hal_pcd_in_ep_fifo_ready_clr(hpcd, 0);
        /*Set TX Transfer Size  */
        hpcd->in_ep_regs[0]->in.EP_IN_TRANS_SZ = req->wLength;
    }
    else
    {
        /*Enter Read Data State*/
        hpcd->ep0_status = HAL_PCD_EP0_OUT_DATA_PHASE;
        if (req->wLength == 0)
        {
            /*Enter Send Status State  */
            hpcd->ep0_status = HAL_PCD_EP0_IN_STATUS_PHASE;
            hpcd->in_ep_regs[0]->in.EP_IN_TRANS_SZ = 0;
            hal_pcd_in_ep_fifo_ready_set(hpcd, 0);
            /*TODO
             * Send IN Zero Length Packet
             */
        }
        else
        {
            if (hpcd->ep[0].is_in)
            {
                if (hal_pcd_ep_dir_set(hpcd, 0, HAL_PCD_EP_DIR_OUT) != HAL_OK)
                {
                    LOG_ERROR("ep 0 ep_info set to out error\n");
                    //return -1;
                    return;
                }
                hpcd->ep[0].is_in = 0;
            }
            hal_pcd_ep_info_set(hpcd, 0, HAL_PCD_EP_DIR_OUT, HAL_PCD_EP_TYPE_CTRL, 64);
            hal_pcd_out_ep_fifo_ready_set(hpcd, 0);
        }
    }

    switch (req->request_type & USB_REQ_TYPE_MASK)
    {
		case USB_REQ_TYPE_STANDARD:
			/* Standard Command*/
			usb_pcd_std_req_proc(hpcd, req);
			break;

		case USB_REQ_TYPE_CLASS:
			/*Class Command*/
			usb_pcd_class_req_proc(hpcd, req);
			break;

		default:
			/*Stall  */
			break;
    }
    return;
}

static void pcd_setup(usb_hal_pcd_t *hpcd)
{
    dev_setup_fifo_sts_t setup_sts, setup_sts_clr;
    uint32_t temp = 0;
    int32_t i = 0;
    setup_sts.d32 = hpcd->dev_glb_regs->DEV_SETUP_FIFO_STS;

	if (setup_sts.b.STATUS_COMP)
    {
        if (setup_sts.b.STATUS)
        {
            /*! Clear Status and data type  */
            setup_sts_clr.d32 = 0;
            setup_sts_clr.b.STATUS_COMP = 1;
            setup_sts_clr.b.STATUS = 1;
            hpcd->dev_glb_regs->DEV_SETUP_FIFO_STS = setup_sts_clr.d32;

            /*! Check Setup_After_Out Bit   */

            /*! Read Setup Data             */
            temp = hpcd->dev_glb_regs->SETUP_DATA[0];
            for (i = 0; i < 4; i++)
            {
                hpcd->setup_data[i] = (temp >> (i * 8)) & 0xff;
            }
            temp = hpcd->dev_glb_regs->SETUP_DATA[1];

            for (i = 0; i < 4; i++)
            {
                hpcd->setup_data[i + 4] = (temp >> (i * 8)) & 0xff;
            }
            /*! Set fifo Ready              */
            /*Porcess Setup Data  */
            usb_pcd_setup_proc(hpcd, (ureq_t)hpcd->setup_data);
            hal_pcd_enable_in_ep_interrupt(hpcd, 0);
            
        }
        else
        {
            /*! Clear Status and data type  */
            LOG("Setup data is not received\n");
            setup_sts_clr.d32 = 0;
            setup_sts_clr.b.STATUS_COMP = 1;
            hpcd->dev_glb_regs->DEV_SETUP_FIFO_STS = setup_sts_clr.d32;
            temp = hpcd->dev_glb_regs->SETUP_DATA[0];
			
            for (i = 0; i < 4; i++)
            {
                hpcd->setup_data[i] = (temp >> (i * 8)) & 0xff;
            }
            temp = hpcd->dev_glb_regs->SETUP_DATA[1];

            for (i = 0; i < 4; i++)
            {
                hpcd->setup_data[i + 4] = (temp >> (i * 8)) & 0xff;
            }
            usb_pcd_setup_proc(hpcd, (ureq_t)hpcd->setup_data);
        }
    }
    else
    {
        LOG("Setup Uncomplete\n");
    }
}


//ATTRIBUTE_ISR
void USB_IRQHandler(void)
{

    uint8_t i = 0;
    usb_hal_pcd_t *hpcd = &pcd_handle;
    dev_glb_ep_int_sts_t glb_ep_int;
    dev_int_sts_t dev_int;
    dev_int_sts_t dev_int_clr;
    dev_int_en_t dev_int_en;
    dev_out_ep_fifo_sts_t rx_fifo_sts;
    ep_in_ctl_t ep_in_ctl;
    ep_out_ctl_t ep_out_ctl;
    dev_ep_info_t ep_info;
    dev_cfg_t dev_cfg;
    dev_sts_t dev_sts;
    //uint32_t frame_num;

    //dev_setup_fifo_sts_t setup_fifo_sts;

    dev_int.d32 = hpcd->dev_glb_regs->DEV_INT_STS;
    dev_int_en.d32 = hpcd->dev_glb_regs->DEV_INT_EN;	//init

//	LOG_DEBUG("dev_int sts val 0x%08x\n",dev_int.d32);

	dev_int_clr.b.SOF_INT_STS = 1;
    hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;

    glb_ep_int.d32 = hpcd->dev_glb_regs->DEV_GLB_EP_INT;
    //gpio_write(P12, 1);
    if (glb_ep_int.b.IN_EP_INT)
    {

        for (i = 0; i < MAX_EP_NUM; i++)
        {
            if (glb_ep_int.b.IN_EP_INT & BIT(i))
            {
                usb_pcd_svc_tx_endp(hpcd, i);
            }
        }
        // __gpio_write(P12, 1);
        // __gpio_write(P12, 0);
        //LOG_DEBUG("in ep end\n");
    }
    //if (glb_ep_int.b.OUT_EP_INT)
    //{
    //    //LOG_DEBUG("==OUT_EP_INT==\n");
    //
    //    for (i = 0; i < MAX_EP_NUM; i++)
    //    {
    //        if (glb_ep_int.b.OUT_EP_INT & BIT(i))
    //        {
    //            usb_pcd_svc_rx_endp(hpcd, i);
    //        }
    //    }
    //    __gpio_write(P14, 1);
    //    __gpio_write(P14, 0);
    //    //LOG_DEBUG("out ep end\n");
    //}


    if (dev_int.b.OUT_INT_STS & dev_int_en.b.OUT_INT_EN)
    {

        //gpio_write(P17, 1);
        //gpio_write(P17, 0);
        rx_fifo_sts.d32 = hpcd->dev_glb_regs->DEV_OUT_EP_FIFO_STS;
        usb_pcd_svc_rx_endp(hpcd, rx_fifo_sts.b.EP_NUM);
        //LOG_DEBUG("USB OUT Transfer Received, ep num %d\n", rx_fifo_sts.b.EP_NUM);
    }

    if (dev_int.b.SC_INT_STS & dev_int_en.b.SC_INT_EN)
    {
        __gpio_write(P1, 0);
        __gpio_write(P1, 1);
        __gpio_write(P1, 0);

        dev_int_clr.d32 = 0;
        dev_sts.d32 = hpcd->dev_glb_regs->DEV_STS;
        hal_pcd_out_ep_fifo_ready_set(hpcd, 0);

        ep_info.d32 = 0;
        ep_info.d32 = hpcd->mac_ep_regs->DEV_EP_INFO[1];
        ep_info.b.EP_DIR = HAL_PCD_EP_DIR_IN;
        ep_info.b.EP_NUM = 1;
        ep_info.b.CFG_NUM = dev_sts.b.CFG;
        ep_info.b.EP_TYPE = HAL_PCD_EP_TYPE_INTR;
        ep_info.b.MAX_PKT = 64;
        hpcd->mac_ep_regs->DEV_EP_INFO[1] = ep_info.d32;
        hpcd->in_ep_regs[1]->in.EP_IN_FIFO_SZ = 16;

        hpcd->ep[1].xfer_buf = 0;
        hpcd->ep[1].xfer_cnt = 0;
        hpcd->ep[1].xfer_tmpcnt = 0;
        hpcd->ep[1].xfer_length = 0;
        hpcd->ep[1].max_packet = 64;
        hpcd->ep[1].fifo_depth = 16;
        hpcd->ep[1].is_in  = 1;

/*
        ep_info.d32 = 0;
        ep_info.d32 = hpcd->mac_ep_regs->DEV_EP_INFO[2];
        ep_info.b.EP_DIR = HAL_PCD_EP_DIR_IN;
        ep_info.b.EP_NUM = 2;
        ep_info.b.CFG_NUM = dev_sts.b.CFG;
        ep_info.b.EP_TYPE = HAL_PCD_EP_TYPE_INTR;
        ep_info.b.MAX_PKT = 64;
        hpcd->mac_ep_regs->DEV_EP_INFO[2] = ep_info.d32;
        hpcd->in_ep_regs[2]->EP_IN_FIFO_SZ = 16;

        hpcd->ep[2].xfer_buf = 0;
        hpcd->ep[2].xfer_cnt = 0;
        hpcd->ep[2].xfer_tmpcnt = 0;
        hpcd->ep[2].xfer_length = 0;
        hpcd->ep[2].max_packet = 64;
        hpcd->ep[2].fifo_depth = 16;
        hpcd->ep[2].is_in  = 1;
*/
        dev_cfg.d32 = hpcd->dev_glb_regs->DEV_CFG;
        dev_cfg.b.CSR_DONE = 1;
        hpcd->dev_glb_regs->DEV_CFG = dev_cfg.d32;

        ep_in_ctl.d32 = 0;
        ep_in_ctl.d32 = hpcd->in_ep_regs[1]->in.EP_IN_CTL;
        ep_in_ctl.b.FIFO_READY = 0;
        ep_in_ctl.b.FLUSH_FIFO = 1;
        hpcd->in_ep_regs[1]->in.EP_IN_CTL = ep_in_ctl.d32;

        /*
        ep_in_ctl.d32 = 0;
        ep_in_ctl.b.FIFO_READY = 0;
        ep_in_ctl.b.FLUSH_FIFO = 1;
        hpcd->in_ep_regs[2]->EP_IN_CTL = ep_in_ctl.d32;
        */

        dev_int_clr.b.SC_INT_STS = 1;
        hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;
    }

//    if (dev_int.b.SI_INT_STS & dev_int_en.b.SI_INT_EN)
//    {
//		__gpio_write(P8, 0);
//		__gpio_write(P8, 1);
//		__gpio_write(P8, 0);
//
//        dev_int_clr.d32 = 0;
//        dev_int_clr.b.SI_INT_STS = 1;
//        hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;
//        dev_sts.d32 = hpcd->dev_glb_regs->DEV_STS;
////        LOG_DEBUG("USB Set Interface %d\n", dev_sts.b.INTF);
//    }

    if (dev_int.b.UR_INT_STS & dev_int_en.b.UR_INT_EN)
    {
		__gpio_write(P9, 0);
		__gpio_write(P9, 1);
		__gpio_write(P9, 0);

        dev_int_clr.d32 = 0;
        dev_int_clr.b.UR_INT_STS = 1;
        hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;
        //LOG_DEBUG("#USB Reset#\n");
        ep_in_ctl.d32 = 0;
        hpcd->in_ep_regs[0]->in.EP_IN_CTL = ep_in_ctl.d32;
        ep_in_ctl.b.FLUSH_FIFO = 1;
        hpcd->in_ep_regs[0]->in.EP_IN_CTL = ep_in_ctl.d32;
        hpcd->in_ep_regs[0]->in.EP_IN_FIFO_SZ = 16;

        ep_out_ctl.d32 = 0;
        hpcd->out_ep_regs[0]->out.EP_OUT_CTL = ep_out_ctl.d32;
        ep_out_ctl.b.FLUSH_FIFO = 1;
        hpcd->out_ep_regs[0]->out.EP_OUT_CTL = ep_out_ctl.d32;
        hid_mouse_ready = 0;
    }


    if (dev_int.b.SETUP_INT_STS & dev_int_en.b.SETUP_INT_EN)
    {


//        LOG_DEBUG("USB Setup\n");
        hpcd->ep[0].xfer_buf = NULL;
        hpcd->ep[0].xfer_tmpcnt = 0;
        hpcd->ep[0].xfer_length = 0;
        hpcd->ep[0].xfer_cnt = 0;
        pcd_setup(hpcd);

        dev_int_clr.b.SETUP_INT_STS = 1;
        hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;
        hal_pcd_enable_in_ep_interrupt(hpcd, 0);
        

        //NVIC_ClearPendingIRQ(TIM1_IRQn);
		//volatile uint32_t st = AP_TIM1->EOI;
		//clk_reset(MOD_TIMER);
		//set_timer(AP_TIM1, 625);

		__gpio_write(P10, 1);
		__gpio_write(P10, 0);
    }


//    if (dev_int.b.SOF_INT_STS & dev_int_en.b.SOF_INT_EN)
//    {
//		__gpio_write(P12, 0);
//		__gpio_write(P12, 1);
//		__gpio_write(P12, 0);
//
//        dev_int_clr.d32 = 0;
//        dev_int_clr.b.SOF_INT_STS = 1;
//        hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;
//        timer++;

//    }



    if (dev_int.b.US_INT_STS & dev_int_en.b.US_INT_EN)
    {
		__gpio_write(P13, 0);
		__gpio_write(P13, 1);
		__gpio_write(P13, 0);

        dev_int_clr.d32 = 0;
        dev_int_clr.b.US_INT_STS = 1;
//        LOG_DEBUG("Suspend Int\n");
        hpcd->dev_glb_regs->DEV_INT_STS = dev_int_clr.d32;
        hid_mouse_ready = 0;
        //NVIC_SystemReset();
    }

    if(hpcd->cb != NULL){
        hpcd->cb();
	}

    //gpio_write(P12, 0);


	//LOG_DEBUG("USB_IRQ_e\n");

}

void pcd_init(usb_irq_cb_t cb)
{
	usb_pin_init();

    //pcd_handle.get_dev_desc = get_dev_desc;
    usb_ep_tx_q.num = 0;
    pcd_handle.cb = cb;
    hal_pcd_init(&pcd_handle);
	NVIC_SetPriority(USB_IRQn, IRQ_PRIO_REALTIME);
}
