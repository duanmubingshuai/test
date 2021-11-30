/**
    \file ltrn_api.c

    This file defines the Transport Layer Application Interface Methods
*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "ltrn_internal.h"
#include "ltrn_extern.h"

#ifdef MS_LTRN

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Static Global Variables */
static EM_timer_handle handle_segment_ack_timer = EM_TIMER_HANDLE_INIT_VAL;

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */

void ltrn_handle_segment_ack_timeout_handler (void* args, UINT16 size);


/**
    \brief Register Interface with Lower Transport Layer

    \par Description
    This routine registers interface with the Lower Transport Layer.
    Transport Layer supports single Application, hence this routine shall be called once.

    \param appl_info
           Details describing Application Notification Callback

    \return API_SUCCESS or an error code indicating reason for failure

*/
API_RESULT MS_ltrn_register
(
    /* IN */ LTRN_NTF_CB    ltrn_cb
)
{
    /* Register the callback */
    ltrn_callback = ltrn_cb;
    return API_SUCCESS;
}


/**
    \brief API to send transport PDUs

    \par Description
    This routine sends transport PDUs to peer device.

    \param [in] src_addr
           Source Address

    \param [in] dst_addr
           Destination Address

    \param [in] subnet_handle
           Handle identifying the Subnet

    \param [in] msg_type
           Transport Message Type

    \param [in] ttl
           Time to Live

    \param [in] akf
           Application Key Flag

    \param [in] aid
           Application Key Identifier

    \param [in] seq_num
           Sequence Number to be used for the Packet

    \param [in] buffer
           Transport Packet

    \param [in] buffer_len
           Transport Packet Length

    \param [in] reliable
           If requires lower transport Ack, set reliable as TRUE

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_ltrn_send_pdu
(
    /* IN */ MS_NET_ADDR               saddr,
    /* IN */ MS_NET_ADDR               daddr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_TRN_MSG_TYPE           msg_type,
    /* IN */ UINT8                     ttl,
    /* IN */ UINT8                     akf,
    /* IN */ UINT8                     aid,
    /* IN */ UINT32                    seq_num,
    /* IN */ UCHAR*                    buffer,
    /* IN */ UINT16                    buffer_len,
    /* IN */ UINT8                     reliable
)
{
    API_RESULT retval;
    MS_NET_HEADER    net_hdr;
    MS_BUFFER        ms_buffer;
    UINT16           seq_zero;
    UCHAR            seg_o, seg_n;
    UCHAR            pdu[LTRN_MAX_PKT_SIZE];
    UCHAR            opcode;
    UCHAR            nbytes, to_write;
    /* Initialization */
    retval = API_FAILURE;
    LTRN_TRC("[LTRN Tx] Pkt. SRC:0x%04X, DST:0x%04X, SH:0x%04X, SeqNum:0x%08X, TTL:0x%02X, AKF:0x%02X, AID:0x%02X, Len:%d, MsgType:0x%04X\n",
             saddr, daddr, subnet_handle, seq_num, ttl, akf, aid, buffer_len, msg_type);
//    printf("[LTRN Tx] Pkt. SRC:0x%04X, DST:0x%04X, SH:0x%04X, SeqNum:0x%08X, TTL:0x%02X, AKF:0x%02X, AID:0x%02X, Len:%d, MsgType:0x%04X\n",
//             saddr, daddr, subnet_handle, seq_num, ttl, akf, aid, buffer_len, msg_type);
    /* Fill Network Header */
    EM_mem_set(&net_hdr, 0, sizeof(net_hdr));
    /* Set Source Address */
    net_hdr.saddr = saddr;
    /* Set Destination Address */
    net_hdr.daddr = daddr;
    /* Set TTL */
    net_hdr.ttl = ttl;
    /* Set Sequence Number */
    net_hdr.seq_num = seq_num;
    /* SeqZero is 13 bit value */
    seq_zero = (UINT16) (seq_num & 0x1FFF);
    /* Lock */
    LTRN_LOCK();

    switch (msg_type)
    {
    /** Transport Layer Control Packet */
    case MS_TRN_CTRL_PKT:
    {
        /* Set CTL bit for Control Message */
        net_hdr.ctl = 0x01;

        /* Segmentation of Control Message */
        if (buffer_len > 11)
        {
            /* TODO: Send the segmented frames similar to Access messages */
            /* Calculate Last Segment Number */
            /* Each segment can have maximum of 8 octets */
            seg_n = (UINT8)(buffer_len >> 3);
            /* First octet is opcode */
            opcode = buffer[0];
            nbytes = 0;

            for (seg_o = 0; seg_o <= seg_n; seg_o++)
            {
                /* RFU bit is set as `0` */
                LTRN_FRM_SEG_CTRL_MSG_HDR(opcode, 0, seq_zero, seg_o, seg_n, pdu);
                to_write = (UINT8)(buffer_len - nbytes);

                if (to_write > 8)
                {
                    to_write = 8;
                }

                EM_mem_copy(&pdu[4], &buffer[nbytes], to_write);
                nbytes += to_write;
                ms_buffer.payload = pdu;
                ms_buffer.length = to_write + 4;
                LTRN_TRC("[LTRN Tx] Tx initiated with Segmented Control PDU...\n");
                retval = MS_net_send_pdu
                         (
                             &net_hdr,
                             subnet_handle,
                             &ms_buffer,
                             MS_FALSE
                         );

                if (API_SUCCESS != retval)
                {
                    LTRN_ERR("[LTRN Tx] Failed to Tx Segmented Control PDU. Result: 0x%04X\n",
                             retval);
                    break;
                }
            }
        }
        else
        {
            LTRN_TRC("[LTRN Tx] Tx initiated with Unsegmented Control Msg...\n");
            ms_buffer.payload = buffer;
            ms_buffer.length = buffer_len;

            //by hq,when segment ack,it should loss data,so delete it.....
            for(UINT8 i =0; i < 1; i++)
            {
                retval = MS_net_send_pdu
                         (
                             &net_hdr,
                             subnet_handle,
                             &ms_buffer,
                             MS_FALSE
                         );
            }
        }
    }
    break;

    /** Access Packet */
    case MS_TRN_ACCESS_PKT:
        /* TODO: Most part is same with MS_TRN_CTRL_PKT handling. Optimize. */
    {
        /* Set CTL bit for Access Message */
        net_hdr.ctl = 0x00;

        /* Segmentation of Access Message */
        /* Also check if the packet is mentioned as reliable by access layer */
        if ((MS_TRUE == reliable) || (buffer_len > 15))
        {
            retval = ltrn_send_seg_pdu
                     (
                         &net_hdr,
                         subnet_handle,
                         akf,
                         aid,
                         seq_zero,
                         buffer,
                         buffer_len
                     );
        }
        else
        {
            LTRN_TRC("[LTRN Tx] Tx initiated with Unsegmented Access PDU...\n");
            /* LTRAN Header for Upper Transport Access PDU - Unsegmented */
            pdu[0] = (akf << 6);
            pdu[0] |= (aid & 0x3F);
            EM_mem_copy(&pdu[1], &buffer[0], buffer_len);
            ms_buffer.payload = pdu;
            ms_buffer.length = buffer_len + 1;

            for(UINT8 i =0; i < 1; i++)
            {
                retval = MS_net_send_pdu
                         (
                             &net_hdr,
                             subnet_handle,
                             &ms_buffer,
                             MS_FALSE
                         );
            }
        }
    }
    break;

    default:
        break;
    }

    /* Unlock */
    LTRN_UNLOCK();
    return retval;
}

/**
    \brief API to send segmented lower transport PDUs

    \par Description
    This routine sends segmented lower transport PDUs to peer device.

    \param [in] seq_zero
           Least significant bits of SeqAuth (13 bit value)

    \param [in] buffer
           Transport Packet

    \param [in] buffer_len
           Transport Packet Length

    \return API_SUCCESS or an error code indicating reason for failure
    TODO: Update header
*/
API_RESULT ltrn_send_seg_pdu
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ UINT8                     akf,
    /* IN */ UINT8                     aid,
    /* IN */ UINT16                    seq_zero,
    /* IN */ UCHAR*                    buffer,
    /* IN */ UINT16                    buffer_len
)
{
    LTRN_SAR_CTX* sar;
    API_RESULT     retval;
    /* Destination Address Types */
    UINT8           daddr_type;
    /* TODO: Use the same function to Tx Segmented Control PDU */
    LTRN_TRC("[LTRN Tx] Segmented Access PDU. SeqZero:0x%04X, Len:%d\n",
             seq_zero, buffer_len);
    /* allocate a SAR context */
    sar = ltrn_sar_alloc_ctx
          (
              LTRN_SAR_CTX_TX,
              buffer_len
          );

    /* On failure return error */
    if (NULL == sar)
    {
        LTRN_ERR("[LTRN Tx] SAR Context Allocation for Tx Failed.\n");
        retval = LTRN_SAR_CTX_ALLOCATION_FAILED;
    }
    /**
        On success, save required fields in the SAR Context
        and initiate transmission of all the fragments
    */
    else
    {
        UINT8  seg_n;
        retval = API_SUCCESS;
        /* Copy buffer */
        EM_mem_copy
        (
            sar->data,
            buffer,
            buffer_len
        );
        /* Save other information */
        sar->subnet_handle = subnet_handle;
        sar->ctl = net_hdr->ctl;
        sar->ttl = net_hdr->ttl;
        sar->saddr = net_hdr->saddr;
        sar->daddr = net_hdr->daddr;
        sar->akf   = akf;
        sar->aid   = aid;
        sar->seq_zero = seq_zero;
        /* TODO: Check where the count to be configured */
        /* How will this value depend on the DST type, where for non-unicast address local device will not receive ack */
        daddr_type = MS_net_get_address_type(net_hdr->daddr);

        if(daddr_type == MS_NET_ADDR_TYPE_UNICAST)
        {
            sar->rtx_count = LTRN_RTX_COUNT_UNICASS;
        }
        else
        {
            sar->rtx_count = LTRN_RTX_COUNT_GROUP;
        }

        /* TODO: Check the calculation for Control message */
        /* TODO: Define a macro for 12 */
        seg_n = (UINT8)(buffer_len / 12);

        if ((12 * seg_n) == buffer_len)
        {
            seg_n--;
        }

        sar->seg_n = seg_n;
        sar->block_ack = 0;

        if (0x1F == seg_n)
        {
            sar->expected_block_ack = 0xFFFFFFFF;
        }
        else
        {
            sar->expected_block_ack = (UINT32)((1 << (seg_n + 1)) - 1);
        }

        /* Initiate Tx of SAR Segments */
        ltrn_sar_transmit_segments(sar);
    }

    return retval;
}

/* This function handles received Segment Ack messages */
API_RESULT ltrn_handle_segment_ack
(
    /* IN */ MS_NET_ADDR         saddr,
    /* IN */ MS_NET_ADDR         daddr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT8               obo,
    /* IN */ UINT16              seq_zero,
    /* IN */ UINT32              block_ack
)
{
    LTRN_SAR_CTX* sar;
    API_RESULT retval;
    LTRN_TRC("[LTRN Rx] Rxed Block Ack. SRC:0x%04X, DST:0x%04X, OBO:0x%02X, SeqZero:0x%04X, BlockAck:0x%08X, SH:0x%04X\n",
             saddr, daddr, obo, seq_zero, block_ack, subnet_handle);
//    printf("[LTRN Rx] Rxed Block Ack. SRC:0x%04X, DST:0x%04X, OBO:0x%02X, SeqZero:0x%04X, BlockAck:0x%08X, SH:0x%04X\n",
//    saddr, daddr, obo, seq_zero, block_ack, subnet_handle);
    /* Search for SAR context */
    sar = ltrn_sar_search_and_alloc_ctx
          (
              LTRN_SAR_CTX_TX,
              daddr,
              saddr,
              seq_zero,
              obo,
              0
          );

    if (NULL != sar)
    {
        /* Check if BlockAck is 0 or all Ack received */
        if (0 == block_ack)
        {
            LTRN_TRC(
                "[LTRN Rx] Seg Tx Canceled by peer. DST: 0x%04X. Seq_Zero: 0x%04X\n",
                sar->daddr, sar->seq_zero);
            /* TODO: Report failure to upper layer */
            ltrn_sar_free_ctx(sar);
        }
        else if (block_ack == sar->expected_block_ack)
        {
            LTRN_TRC(
                "[LTRN Rx] Seg Tx Complete. DST: 0x%04X. Seq_Zero: 0x%04X\n",
                sar->daddr, sar->seq_zero);
            /* TODO: Report success to upper layer */
            ltrn_sar_free_ctx(sar);
        }
        else
        {
            UINT32 block_ack_local = sar->block_ack;

            /* Save the block ack information */
            if (0x1F == sar->seg_n)
            {
                sar->block_ack |= block_ack;
            }
            else
            {
                sar->block_ack |= (block_ack & ((1 << (sar->seg_n + 1)) - 1));
            }

            /**
                Mesh Spec Section 3.5.3.3,
                If a valid segment ack is received for a segmented message
                the LTRN shall reset the segment transmission timer and
                retransmit all unacked LTRN PDUs.
            */
            /* Stop Timer */
            if((0 != sar->rtx_count) && ((block_ack_local < sar->block_ack)||(sar->block_ack==0)))
            {
                #ifdef EM_USE_EXT_TIMER
                EXT_cbtimer_stop_timer(sar->ack_rtx_timer_handle);
                sar->ack_rtx_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
                #else
                EM_stop_timer(&sar->ack_rtx_timer_handle);
                #endif

                if(handle_segment_ack_timer == EM_TIMER_HANDLE_INIT_VAL)
                {
                    retval = EM_start_timer
                             (
                                 &handle_segment_ack_timer,
                                 (40 | EM_TIMEOUT_MILLISEC),
                                 ltrn_handle_segment_ack_timeout_handler,
                                 (void*)&sar,
                                 sizeof(sar)
                             );

                    if (API_SUCCESS != retval)
                    {
                        ltrn_sar_transmit_segments(sar);
                        LTRN_ERR("[LTRN] Ack Rtx timer start failed\n");
                    }
                }

//                blebrr_scan_pl(FALSE);
//                ltrn_sar_transmit_segments(sar);
            }
        }
    }
    else
    {
        LTRN_ERR("[LTRN Rx] Failed to find SAR Context for BlockAck\n");
    }

    return API_SUCCESS;
}

/* SAR Timeout Handler for Retry */
void ltrn_handle_segment_ack_timeout_handler (void* args, UINT16 size)
{
    LTRN_SAR_CTX* sar;
    MS_IGNORE_UNUSED_PARAM(size);
    sar = (LTRN_SAR_CTX*)(*((UINT32*)args));
    /* Reset Timer Handler */
    handle_segment_ack_timer = EM_TIMER_HANDLE_INIT_VAL;
    /* Lock */
    LTRN_LOCK_VOID();

    /* Check the SAR entity is still valid */
    if (LTRN_SAR_CTX_INVALID != sar->type)
    {
        /* Trigger transmission */
        ltrn_sar_transmit_segments(sar);
    }

    /* Unlock */
    LTRN_UNLOCK_VOID();
    return;
}

#endif /* MS_LTRN */

