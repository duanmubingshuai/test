
/**
    \file ltrn_init.c

    Module initialization routine and tables defined here.

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "ltrn_internal.h"
#include "ltrn.h"
#include "trn_internal.h"

#include "net_internal.h"

#include "MS_access_api.h"

#ifdef MS_LTRN
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */
/** Module Mutex */
MS_DEFINE_MUTEX (ltrn_mutex)

/** Module Callback */
LTRN_NTF_CB ltrn_callback;

/* --------------------------------------------- Static Global Variables */


/* --------------------------------------------- Functions */

/**
    \brief Initializes Module.

    \par Description Initializes Module tables.
*/
void ms_ltrn_init (void)
{
    ms_internal_verificaiton_check();
    LTRN_TRC("[LTRN] Initializing LTRN..");
    /* Module Mutex Initialization */
    MS_MUTEX_INIT_VOID (ltrn_mutex, LTRN);
    /* Initialize Replay Protection Cache */
    ltrn_alloc_replay_cache();
    ltrn_init_replay_cache();
    /* Initialize SAR Contexts */
    ltrn_init_sar_contexts();
    /* Register with the network layer */
    MS_net_register(ltrn_pkt_in);
}


#ifndef MS_NO_SHUTDOWN
/**
    \par Description:
    This function is the shutdown handler for Transport module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.
*/
void ms_ltrn_shutdown (void)
{
    LTRN_TRC("[LTRN] Transport Shutdown Successful.");
}
#endif /* MS_NO_SHUTDOWN */

API_RESULT ltrn_pkt_first_process
(
    /* IN */MS_NET_HEADER*        net_hdr,
    /* OUT */ UINT32*               repaly_flag
)
{
    API_RESULT retval;
    /* Destination Address Types */
    UINT8           daddr_type;
//    UINT8           result = API_FAILURE;
    UINT32           _replay_flag = API_FAILURE;
    API_RESULT    retval_final;
    LPN_HANDLE is_lpn_handler;
    retval_final = NET_POST_PROCESS_RX_PKT;
    daddr_type = MS_net_get_address_type(net_hdr->daddr);
    retval = MS_access_is_valid_element_address(net_hdr->saddr);

    if (API_SUCCESS == retval)
    {
        LTRN_ERR("[LTRN Rx] Pkt originated from Local SRC. Dropping...\n");
        //printf("[LTRN Rx] Pkt originated from Local SRC. Dropping...\n");
        _replay_flag = API_SUCCESS;
        *repaly_flag = _replay_flag;
        return NET_RX_LOCAL_SRC_ADDR_PKT;
    }

    if (MS_NET_ADDR_TYPE_UNICAST == daddr_type)
    {
        retval = MS_access_is_valid_element_address(net_hdr->daddr);

        if (API_SUCCESS == retval)
        {
            LTRN_TRC("[LTRN Rx] Packet reached final recipient\n");
            /* Mark no further processing is required by the network layer */
            retval_final = API_SUCCESS;
        }
        else
            _replay_flag = API_SUCCESS;

        #ifdef MS_FRIEND_SUPPORT
        else
        {
            /* Check if the address is of any of the known LPN elements */
            retval = MS_trn_is_valid_lpn_element_address(net_hdr->daddr, &is_lpn_handler);

            if (API_SUCCESS == retval)
            {
                /* Mark no further processing is required by the network layer */
                retval_final = API_SUCCESS;
            }
            else        //by hq
            {
                _replay_flag = API_SUCCESS;
            }
        }

        #endif /* MS_FRIEND_SUPPORT */
    }
    /* Check if Fixed Group Address */
    /** TODO: Move Fixed Group Address check magic numbers to a Macro */
    else if (0xFF == (net_hdr->daddr >> 8))
    {
        /* Check if Fixed Group Address is valid */
        if (0xFFFC > net_hdr->daddr)
        {
            _replay_flag = API_SUCCESS;
            *repaly_flag = _replay_flag;
            return MS_INVALID_ADDRESS;
        }
    }
    /* For other address types, it has to match with one subscription list entry of the local or one of the friend. */
    else
    {
        retval = MS_access_is_valid_subscription_address(net_hdr->daddr);

        if(API_SUCCESS != retval)
        {
            #ifdef MS_FRIEND_SUPPORT
            retval = MS_trn_is_valid_lpn_subscription_address(net_hdr->daddr, &is_lpn_handler);

            if (API_SUCCESS != retval)
            {
                _replay_flag = API_SUCCESS;
            }

            #else
            _replay_flag = API_SUCCESS;
            #endif /* MS_FRIEND_SUPPORT */
        }
    }

    /* Lock */
    LTRN_LOCK();
    /* Check for replay protection */
    retval = ltrn_check_if_replayed(net_hdr);

    if (API_SUCCESS == retval)
    {
        LTRN_ERR("[LTRN Rx] Replayed Pkt. Dropping...\n");
//        printf("[LTRN Rx] Replayed Pkt. Dropping...\n");
        _replay_flag = API_SUCCESS;
        /* Unlock */
        LTRN_UNLOCK();
        *repaly_flag = _replay_flag;
        return API_FAILURE;
    }

    /* Add to replay cache */
    ltrn_update_replay_cache(net_hdr);
    LTRN_UNLOCK();
    *repaly_flag = _replay_flag;
    return retval_final;
}


/**
    \par Description
    This function handles the incoming data received over network layer.

    \param [in] net_hdr
           Received Network Packet Header
    \param [in] subnet_handle
           Handle identifying associated subnet on which the packet is received
    \param [in] pdata
           The incoming Data Packet
    \param [in] pdatalen
           Size of the incoming Data Packet

    \retval
    - \ref NET_POST_PROCESS_RX_PKT: To inform Network Layer if the packet to be
           further processed, e.g. to be relayed or proxied etc.

    - Any Other Result/Error Code defined in MS_error.h: Ignored by Network Layer.
*/
API_RESULT ltrn_pkt_in
(
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UCHAR*              pdata,
    /* IN */ UINT16              pdatalen
)
{
    API_RESULT retval;
    /* Upper Layer data and datalen */
    UCHAR* udata;
    UINT16  udatalen;
    /* Lower Transport Format Type */
    UCHAR   ltft;
    /* Flag to call upper layer */
    UCHAR   call_ul;
    UCHAR szmic;
    /* Destination Address Types */
    UINT8           daddr_type;
    /**
        Retval based on - if one of the local or LPN element
        is the final recipient of the packet.
    */
    API_RESULT    result;
    LTRN_RX_META_INFO   rx_meta_info;
    LTRN_TRC("[LTRN Rx] Network Callback. SRC:0x%04X, DST:0x%04X, CTL:0x%02X, TTL:0x%02X, Length %d, SH:0x%04X\n",
             net_hdr->saddr, net_hdr->daddr, net_hdr->ctl, net_hdr->ttl, pdatalen, subnet_handle);
//    printf("[LTRN Rx] Network Callback. SRC:0x%04X, DST:0x%04X, CTL:0x%02X, TTL:0x%02X, Length %d, SH:0x%04X\r\n",
//    net_hdr->saddr, net_hdr->daddr, net_hdr->ctl, net_hdr->ttl, pdatalen, subnet_handle);
    LTRN_debug_dump_bytes(pdata, pdatalen);
    /* Strip headers if any and pass it to the registered layer above */
    #if 0

    /* Extract Transport Header */
    if (TCF_HDR_LEN_MIN > pdatalen)
    {
        LTRN_ERR("[LTRN Rx] Incomplete Transport Packet Received of length %d. Dropping...\n",
                 pdatalen);
        return;
    }

    #endif /* 0 */
    /* Validate Destination Address */
    daddr_type = MS_net_get_address_type(net_hdr->daddr);
    LTRN_INF("[LTRN Rx] DST:0x%04X ADDR Type:0x%02X\n", net_hdr->daddr, daddr_type);

    /**
        Destination address
        - Shall not be Unassigned Address (already checked at network layer).
        - Control Message shall not be a Virtual Address.
    */
    if ((MS_TRN_CTRL_PKT == net_hdr->ctl) && (MS_NET_ADDR_TYPE_VIRTUAL == daddr_type))
    {
        LTRN_ERR("[LTRN Rx] Invalid SRC or DST. Dropping...\n");
        return NET_INVALID_RX_PKT_FORMAT;
    }

    /* To keep some compilers happy */
    udata = NULL;
    udatalen = 0;
    szmic = 0;
    /* Reset by default */
    rx_meta_info.is_for = 0;
    rx_meta_info.lpn_handle = (LPN_HANDLE)LPN_HANDLE_INVALID;
    /* Ensure the Source Address is not one of the local element addresses */
    retval = MS_access_is_valid_element_address(net_hdr->saddr);

    if (API_SUCCESS == retval)
    {
        LTRN_ERR("[LTRN Rx] Pkt originated from Local SRC. Dropping...\n");
        return NET_RX_LOCAL_SRC_ADDR_PKT;
    }

    result = API_FAILURE;

    /**
        If Destination Address is unicast address, see it is for
        one of the local or friends element addresses.
    */
    if (MS_NET_ADDR_TYPE_UNICAST == daddr_type)
    {
        retval = MS_access_is_valid_element_address(net_hdr->daddr);

        if (API_SUCCESS == retval)
        {
            LTRN_TRC("[LTRN Rx] Packet reached final recipient\n");
            /* Set bit - for Local Element */
            rx_meta_info.is_for = LTRN_RX_FOR_LOCAL_ELEMENT;
        }

        #ifdef MS_FRIEND_SUPPORT
        else
        {
            /* Check if the address is of any of the known LPN elements */
            retval = MS_trn_is_valid_lpn_element_address(net_hdr->daddr, &rx_meta_info.lpn_handle);

            if (API_SUCCESS == retval)
            {
                LTRN_TRC("[LTRN Rx] Packet reached final recipient. It is for LPN Handle:0x%02X\n",
                         rx_meta_info.lpn_handle);
                /* Set bit - for LPN Element */
                rx_meta_info.is_for = LTRN_RX_FOR_LPN_ELEMENT;
            }
        }

        #endif /* MS_FRIEND_SUPPORT */
    }
    /* Check if Fixed Group Address */
    /** TODO: Move Fixed Group Address check magic numbers to a Macro */
    else if (0xFF == (net_hdr->daddr >> 8))
    {
        /* Check if Fixed Group Address is valid */
        if (0xFFFC <= net_hdr->daddr)
        {
            retval = MS_access_is_fixed_group_addr_to_be_processed(net_hdr->daddr);
            /* Not Checking return value. In case of failure, the function will return */
            /* Set bit - for Local Element */
            rx_meta_info.is_for = LTRN_RX_FOR_LOCAL_ELEMENT;
            /* TODO: Check which flags to be set for LPN */
        }
        else
        {
            LTRN_ERR("[LTRN Rx] Fixed Group Addresses with RFU. Dropping\n");
            return MS_INVALID_ADDRESS;
        }
    }
    /* For other address types, it has to match with one subscription list entry of the local or one of the friend. */
    else
    {
        retval = MS_access_is_valid_subscription_address(net_hdr->daddr);

        if (API_SUCCESS == retval)
        {
            rx_meta_info.is_for = LTRN_RX_FOR_LOCAL_SUBSCRIPTION;
        }

        #ifdef MS_FRIEND_SUPPORT
        retval = MS_trn_is_valid_lpn_subscription_address(net_hdr->daddr, &rx_meta_info.lpn_handle);

        if (API_SUCCESS == retval)
        {
            rx_meta_info.is_for |= LTRN_RX_FOR_LPN_SUBSCRIPTION;
        }

        #endif /* MS_FRIEND_SUPPORT */
    }

    /* If packet is not for local node, return */
    /* if (API_SUCCESS != retval) */
    if (0 == rx_meta_info.is_for)
    {
        LTRN_TRC("Packet not for Local Node - 0x%04X. Dropping...\n", retval);
        /**
            The packet is not for the local elements or LPN.
            Let the network layer do further processing.
        */
        return NET_POST_PROCESS_RX_PKT;
    }

    /**
        TODO: Check if it is destined for any of the associated LPNs
        or their subscription list.

        Mark OBO as 1 and process the frame in the same way like for local elements.

        While giving the frame to Upper Transport Layer, indicate this is for friend
        node using OBO flag.
        Also use OBO flag, for using local primary element address as the source address
        while sending out segmented ack frame on behalf of the LPN.

        For reassembled packets destined for friend, perform the reassembly in the same
        SAR data structure and on successful reassembly, construct (re-segment) the LTRN frames.
    */
    /* By default, do not call Upper Layer */
    call_ul = MS_FALSE;
    /* Lock */
    //LTRN_LOCK();
    /* Check for replay protection */
    //retval = ltrn_check_if_replayed(net_hdr);
    //if (API_SUCCESS == retval)
    //{
    //    LTRN_ERR("[LTRN Rx] Replayed Pkt. Dropping...\n");
    //    printf("[LTRN Rx] Replayed Pkt. Dropping...\n");
    //     /* Unlock */
    //     LTRN_UNLOCK();
    //    return MS_INVALID_ADDRESS;
    //}
    /* Extract and verify Segmented Field */
    ltft = pdata[0] >> 7;

    /**
          CTL Field | SEG Field | Lower Transport PDU
          ----------+-----------+--------------------
          0         | 0         | Unsegmented Access Message
          ----------+-----------+--------------------
          0         | 1         | Segmented Access Message
          ----------+-----------+--------------------
          1         | 0         | Unsegmented Control Message
          ----------+-----------+--------------------
          1         | 1         | Segmented Control Message
          ----------+-----------+--------------------
    */
    switch (ltft)
    {
    /* Unsegmented Message */
    case MS_LTRN_T_UNSEG_MSG:
    {
        UCHAR opcode;

        if (MS_TRN_CTRL_PKT == net_hdr->ctl)
        {
            /* Extract Opcode */
            opcode = (UCHAR)(pdata[0] & 0x7F);

            /* Check if Segment Acknowledgement message */
            if (MS_LTRN_OPCODE_SEGMENT_ACK == opcode)
            {
                UINT16 seq_zero;
                UCHAR  obo;
                UINT32 block_ack;
                /**
                     ## Segment Ack Message ##

                      <-     Byte 0    -> <-      Byte 1, 2     -> <-       Byte 3, 4, 5, 6       ->
                      7   6             0 7    6      0 7   2    0 31     23      15       7        0
                      +---+--------------+-----+-------+----------+-------+-------+--------+--------+
                      | 0 | Opcode 0x00  | OBO | SeqZero    | RFU |            BlockAck             |
                      +---+--------------+-----+-------+----------+-------+-------+--------+--------+
                */
                #ifdef MS_FRIEND_SUPPORT

                if (LTRN_RX_FOR_LPN_ELEMENT == (rx_meta_info.is_for & LTRN_RX_FOR_LPN_ELEMENT))
                {
                    LTRN_TRC("[TRN] Ack for LPN:0x%02X.To be handled\n", rx_meta_info.lpn_handle);
                    trn_add_to_queue
                    (
                        rx_meta_info.lpn_handle,
                        0x04, /* Ack */
                        net_hdr,
                        pdata,
                        (UINT8)pdatalen
                    );
                    break;
                }

                #endif /* MS_FRIEND_SUPPORT */
                /* Extract Sequence Zero */
                MS_UNPACK_BE_2_BYTE(&seq_zero, &pdata[1]);
                obo = seq_zero >> 15;
                seq_zero &= 0x7FFF;
                /* Discard RFU */
                seq_zero >>= 2;
                /* Extract BlockAck */
                MS_UNPACK_BE_4_BYTE(&block_ack, &pdata[3]);
                /* Process Segment Ack message */
                ltrn_handle_segment_ack
                (
                    net_hdr->saddr,
                    net_hdr->daddr,
                    subnet_handle,
                    obo,
                    seq_zero,
                    block_ack
                );
                /* Segment Ack message is not passed to upper layer */
                result = API_SUCCESS;
                break;
            }
        }

        /* Pass Unsegmented Control/Access message to Upper Transport */
        udata = pdata;
        udatalen = pdatalen;
        /* TBD: Inform AKF and AID to App Layer */
        LTRN_TRC("[LTRN Rx] Pass Unsegmented message to Upper Layer\n");
        call_ul = MS_TRUE;
        /**
            SZMIC determines size of TransMIC.
            SZMIC=0 => 32 bit TransMIC.
            SZMIC=1 => 64 bit TransMIC.

            TransMIC is not used with Control messages.
        */
        szmic = 0;
    }
    break;

    /* Segmented Message */
    case MS_LTRN_T_SEG_MSG:
    {
        retval = ltrn_handle_seg_pdu
                 (
                     net_hdr,
                     subnet_handle,
                     &rx_meta_info,
                     pdata,
                     pdatalen
                 );
    }
    break;

    default:
        LTRN_ERR("[LTRN Rx] In Default Handler - Nothing to Do!!\n");
        break;
    }

    #ifdef MS_FRIEND_SUPPORT

    /* For LPN - Unsegmented Packets */
    if (MS_TRUE == call_ul)
    {
        if (LTRN_RX_FOR_LPN_ELEMENT == (rx_meta_info.is_for & LTRN_RX_FOR_LPN_ELEMENT))
        {
            LTRN_TRC("[TRN] Pkt for LPN:0x%02X.To be handled\n", rx_meta_info.lpn_handle);
            trn_add_to_queue
            (
                rx_meta_info.lpn_handle,
                0x00, /* Unsegmented */
                net_hdr,
                udata,
                (UINT8)udatalen
            );
        }
        else if (LTRN_RX_FOR_LPN_SUBSCRIPTION == (rx_meta_info.is_for & LTRN_RX_FOR_LPN_SUBSCRIPTION))
        {
            do
            {
                LTRN_TRC("[TRN] Pkt for LPN subscription list. To be searched 0x%02X\n", rx_meta_info.lpn_handle);
                retval = MS_trn_is_valid_lpn_uincast_address(net_hdr->saddr,rx_meta_info.lpn_handle);

                if(retval != API_SUCCESS)
                {
                    trn_add_to_queue
                    (
                        /* TODO: Change the LPN Handle handling for subscription list */
                        rx_meta_info.lpn_handle,
                        0x00, /* Unsegmented */
                        net_hdr,
                        udata,
                        (UINT8)udatalen
                    );
                }

                /* Get next LPN handle if any */
                retval = MS_trn_is_valid_lpn_subscription_address(net_hdr->daddr, &rx_meta_info.lpn_handle);
            }
            while (API_SUCCESS == retval);
        }
    }

    #endif /* MS_FRIEND_SUPPORT */
    /* Unlock */
    LTRN_UNLOCK();

    /* Upper Transport Callback */
    if ((MS_TRUE == call_ul) && (NULL != ltrn_callback) &&
            ((LTRN_RX_FOR_LOCAL_ELEMENT == (rx_meta_info.is_for & LTRN_RX_FOR_LOCAL_ELEMENT)) ||
             (LTRN_RX_FOR_LOCAL_SUBSCRIPTION == (rx_meta_info.is_for & LTRN_RX_FOR_LOCAL_SUBSCRIPTION)))
       )
    {
        LTRN_TRC("[LTRN Rx] Invoke Upper Transport Callback\n");
        /* Control message do not have a TransMIC */
        result = ltrn_callback(net_hdr, subnet_handle, szmic, udata, udatalen);
    }

    #ifdef MS_LPN_SUPPORT

    if (API_SUCCESS == result)
    {
        /**
            Check if packet received from friend.
            TODO: Revamp
        */
        if (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) == subnet_handle)
        {
            UINT8 role;
            LTRN_TRC("[LTRN Rx] LPN Subnet\n");
            MS_access_cm_get_friendship_role(&role);

            if (MS_FRND_ROLE_LPN == role)
            {
                trn_frnd_handle_segment_ack(subnet_handle);
            }
        }
    }

    #endif /* MS_LPN_SUPPORT */
    return retval;
}

/* Function to send segment ack */
API_RESULT ltrn_send_ack
(
    /* IN */ MS_NET_ADDR         saddr,
    /* IN */ MS_NET_ADDR         daddr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT32              seq_zero,
    /* IN */ UINT8               ttl_in_rx_frame,
    /* IN */ UINT8               obo,
    /* IN */ UINT32              block_ack
)
{
    /* Destination Address Types */
    UINT8           daddr_type;
    /**
        Segment Ack message.
        OBO (1-bit) || SeqZero (13-bit) || RFU (2-bit) || BlockAck (4 octet)
    */
    UCHAR   ack[7];
    API_RESULT retval;
    UINT32 seq_num;
    #ifdef MS_LPN_SUPPORT

    /**
        Check if packet received from friend.
        TODO: Revamp
    */
    if (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) == subnet_handle)
    {
        UINT8 role;
        MS_access_cm_get_friendship_role (&role);

        if (MS_FRND_ROLE_LPN == role)
        {
            LTRN_TRC("[LTRN] Segment from Friend Received. Not Sending Ack\n");
            LTRN_UNLOCK();
            trn_frnd_handle_segment_ack(subnet_handle);
            LTRN_LOCK();
            return API_SUCCESS;
        }
    }

    #endif /* MS_LPN_SUPPORT */
    LTRN_TRC("[LTRN] Send Ack. SRC:0x%04X, DST:0x%04X, SeqZero:0x%08X, TTL:0x%02X, OBO:0x%02X, BlockAck:0x%08X, SH:0x%04X\n",
             saddr, daddr, seq_zero, ttl_in_rx_frame, obo, block_ack, subnet_handle);
    /* Check if the destination address is unicast address */
    daddr_type = MS_net_get_address_type(daddr);

    if (MS_NET_ADDR_TYPE_UNICAST == daddr_type)
    {
        LTRN_INF("[LTRN] DST:0x%04X ADDR Type:0x%02X\n", daddr, daddr_type);
        /* Send Ack for segmented frame */
        ack[0] = MS_LTRN_OPCODE_SEGMENT_ACK;
        /* LTRN_FRM_SEG_ACK_MSG(obo, seq_zero, rfu, block_ack, buf) */
        LTRN_FRM_SEG_ACK_MSG(obo, seq_zero, 0, block_ack, &ack[1]);
        /* Unlock */
        LTRN_UNLOCK();

        /**
            Mesh Spec Section 3.5.2.3.1
            If the received segments were sent with TTL set to 0, it is recommended
            that the corresponding Segment Acknowledgment message is sent with
            TTL set to 0
        */
        if (0 != ttl_in_rx_frame)
        {
            /* TODO: Have configuration to set Segment Ack TTL default value */
            ttl_in_rx_frame = 0x7F;
        }

        MS_net_alloc_seq_num(&seq_num);
        retval = MS_ltrn_send_pdu
                 (
                     saddr,
                     daddr,
                     subnet_handle,
                     MS_TRN_CTRL_PKT,
                     ttl_in_rx_frame,
                     0,
                     0,
                     seq_num,
                     ack,
                     sizeof(ack),
                     MS_FALSE
                 );
        /* Lock */
        LTRN_LOCK();
    }
    else
    {
        LTRN_ERR("[LTRN] Ack can not be sent for DST ADDR Type: 0x%02X\n",
                 daddr_type);
        retval = API_FAILURE;
    }

    return retval;
}

/* This function calculates SeqAuth, from SeqNum and SeqZero */
UINT32 ltran_calculate_seq_auth
(
    /* IN */ UINT32 seq_num,
    /* IN */ UINT32 seq_zero
)
{
    UINT32 seq_auth;
    LTRN_INF("[LTRN] Calculate SeqAuth. SeqNum:0x%08X, SeqZero:0x%08X\n",
             seq_num, seq_zero);
    /**
        Resetting the network header with reconstructed
        sequence number of first segment using SequenceZero.

        (Sequence Number of first segment) = [(SEQ & (~0x1FFF)) | SeqZero]

        if (Sequence Number of first segment) > SEQ
           (Sequence Number of first segment) & 0xFFDFFF
    */
//    if((seq_num&0x1fff) > seq_zero)
//    {
//        seq_auth = ((seq_num & ~0x00001FFF) | seq_zero);
//    }
//    else
//    {
//        seq_auth = ((((seq_num>>13)-1)<<13) | seq_zero);
//    }
    seq_auth = ((seq_num & ~0x00001FFF) | seq_zero);

    if (seq_auth > seq_num)
    {
//        printf("Seq Num Overround\n");
        seq_auth = ((((seq_num>>13)-1)<<13) | seq_zero);
//        seq_auth &= 0xFFFFDFFF;
    }

    LTRN_INF("[LTRN] Calculated SeqAuth:0x%08X\n", seq_auth);
    return seq_auth;
}

/**
    \brief API to handle segmented lower transport PDUs

    \par Description
    This routine handles segmented lower transport PDUs received from peer device.

    \param [in] seq_zero
           Least significant bits of SeqAuth (13 bit value)

    \param [in] buffer
           Transport Packet

    \param [in] buffer_len
           Transport Packet Length

    \return API_SUCCESS or an error code indicating reason for failure
    TODO: Update header
*/
API_RESULT ltrn_handle_seg_pdu
(
    /* IN */ MS_NET_HEADER*       net_hdr,
    /* IN */ MS_SUBNET_HANDLE     subnet_handle,
    /* IN */ LTRN_RX_META_INFO*   rx_meta_info,
    /* IN */ UCHAR*               pdata,
    /* IN */ UINT16               pdatalen
)
{
    UINT32 block_ack;
    UINT16 seq_zero;
    UCHAR seg_o, seg_n;
    UCHAR seg_length, szmic;
    UINT8 obo;
    UINT8 reassembly_status;
    UINT16 save_daddr;
    UINT8   daddr_type;
    LTRN_SAR_CTX* sar;
    API_RESULT     retval;
    obo = (LTRN_RX_FOR_LPN_ELEMENT == (rx_meta_info->is_for & LTRN_RX_FOR_LPN_ELEMENT));
    /* Extract Seg0 and SegN */
    LTRN_EXTRACT_SEG_MSG_HDR(pdata, szmic, seq_zero, seg_o, seg_n);
    LTRN_TRC("[LTRN Rx] Extracted Segmented Header Fields. SZMIC:0x%02X, SeqZero:0x%04X, SegO:0x%02X, SegN:0x%02X, SH:0x%04X. OBO:0x%02X\n",
             szmic, seq_zero, seg_o, seg_n, subnet_handle, obo);
    /* Validate Destination Address */
    daddr_type = MS_net_get_address_type(net_hdr->daddr);
    /* Check if already reassembled and processed this */
    /* TODO: Should we store OBO in the reassembled cache? */
    sar = ltrn_get_current_sar_ctx
          (
              LTRN_SAR_CTX_RX,
              net_hdr->saddr,
              net_hdr->ivi
          );

    if (NULL != sar)
    {
        UCHAR overbit = MS_FALSE;

        if((sar->seq_zero + (sar->seg_n + 1)) > 0x1FFF)
        {
            overbit = MS_TRUE;
        }

        //should confirm 13 bit seq zero overrun by hq(0x1fff)
        /* Check if the received packet Seq_Zero is lesser, drop */
        if (((overbit == MS_FALSE) && (seq_zero < sar->seq_zero) && ((sar->seq_zero - seq_zero) < 0x1000)) ||
                ((overbit == MS_TRUE) && (seq_zero != sar->seq_zero) && (seq_zero < ((sar->seq_zero + sar->seg_n + 1) - 0x1FFF))))
        {
            LTRN_ERR("[LTRN Rx] Received Seq_Zero:0x%08X < Current Active Rx SAR Seq_Zero:0x%08X. Dropping.\n",
                     seq_zero, sar->seq_zero);
//            printf("[LTRN Rx] Received Seq_Zero:0x%08X < Current Active Rx SAR Seq_Zero:0x%08X. Dropping.\n",
//            seq_zero, sar->seq_zero);
            return API_FAILURE;
        }

        /* Check if a new packet being received */
        if (((overbit == MS_FALSE) && ((seq_zero > sar->seq_zero) || ((sar->seq_zero - seq_zero) >= 0x1000))) ||
                ((overbit == MS_TRUE)  && (seq_zero != sar->seq_zero) && (seq_zero > ((sar->seq_zero + sar->seg_n + 1) - 0x1FFF))))
        {
            LTRN_ERR("[LTRN Rx] Received Seq_Zero:0x%08X > Current Active Rx SAR Seq_Zero:0x%08X. Free Active SAR.\n",
                     seq_zero, sar->seq_zero);
//            printf("[LTRN Rx] Received Seq_Zero:0x%08X > Current Active Rx SAR Seq_Zero:0x%08X. Free Active SAR.\n",
//            seq_zero, sar->seq_zero);
            /* Add to Completed SAR Rx Cache */
            ltrn_add_to_reassembled_cache
            (
                sar->saddr,
                sar->ivi,
                sar->seq_auth,
                0x01 /* Failure */
            );
            /* Free SAR Context */
            ltrn_sar_free_ctx(sar);
        }
    }

    retval = ltrn_is_in_reassembled_cache
             (
                 net_hdr->saddr,
                 net_hdr->ivi,
                 ltran_calculate_seq_auth(net_hdr->seq_num, seq_zero),
                 &reassembly_status
             );

    /* On success, send block ack and return. No further processing is required */
    if (API_SUCCESS == retval)
    {
        LTRN_TRC("[LTRN Rx] Present in reassembled cache. Status:0x%02X\n",
                 reassembly_status);

        if (0x00 == reassembly_status)
        {
            LTRN_TRC("[LTRN Rx] Sending BlockAck\n");
            block_ack = (1 << (seg_n + 1)) - 1;

            //by hq(if dst is virtual or group addr,should not be send ack)
            //sig mesh profile 3.5.3.3
            if(daddr_type == MS_NET_ADDR_TYPE_UNICAST)
            {
                //printf("[LTRN Rx] Sending BlockAck\n");
                /* Send ack */
                retval = ltrn_send_ack
                         (
                             net_hdr->daddr,
                             net_hdr->saddr,
                             subnet_handle,
                             seq_zero,
                             net_hdr->ttl,
                             obo,
                             block_ack
                         );
            }
        }

        return API_SUCCESS;
    }

    /**
        Each segment of the Upper Transport Access PDU shall be 12 octets long with
        the exception of the last segment, which may be shorter.

        Each segment of the Upper Transport Control PDU shall be 8 octets long with
        the exception of the last segment, which may be shorter.
    */
    if (MS_TRN_CTRL_PKT == net_hdr->ctl)
    {
        seg_length = 8;
    }
    else
    {
        seg_length = 12;
    }

    /* Send block ack set to 0x00000000, to cancel the transaction */
    if (LTRN_RX_FOR_LPN_ELEMENT == (rx_meta_info->is_for & LTRN_RX_FOR_LPN_ELEMENT))
    {
        MS_access_cm_get_primary_unicast_address(&save_daddr);
    }
    else
    {
        save_daddr = net_hdr->daddr;
    }

    /* search/allocate a SAR context */
    sar = ltrn_sar_search_and_alloc_ctx
          (
              LTRN_SAR_CTX_RX,
              net_hdr->saddr,
              save_daddr,
              seq_zero,
              obo,
              (1 + (seg_n + 1) * seg_length) /* Maximum possible length based on number of segments */
          );

    /* On failure return error */
    if (NULL == sar)
    {
        LTRN_ERR("[LTRN Rx] SAR Rx CTX Search/Allocation Failed. Sending BlockAck 0\n");
        retval = LTRN_SAR_CTX_ALLOCATION_FAILED;
        /* Send block ack set to 0x00000000, to cancel the transaction */
        block_ack = 0;

        if(daddr_type == MS_NET_ADDR_TYPE_UNICAST)  //by hq
        {
            retval = ltrn_send_ack
                     (
                         save_daddr,
                         net_hdr->saddr,
                         subnet_handle,
                         seq_zero,
                         net_hdr->ttl,
                         obo,
                         block_ack
                     );
        }

        return retval;
    }

    /**
        On success, save required fields in the SAR Contex
        and initiate timer
    */
    if(LTRN_SAR_CTX_INVALID == sar->type)
    {
        /* If context is allocated for first time, reset parameters */
        retval = API_SUCCESS;
        sar->type = LTRN_SAR_CTX_RX;
        /* Save other information */
        sar->subnet_handle = subnet_handle;
        sar->ivi = net_hdr->ivi;
        sar->ctl = net_hdr->ctl;
        sar->ttl = net_hdr->ttl;
        sar->saddr = net_hdr->saddr;
        sar->daddr = net_hdr->daddr;
        sar->akf   = 0; /* akf; */
        sar->aid   = 0; /* aid; */
        sar->szmic = szmic;
        sar->seq_zero = seq_zero;
        sar->rtx_count = 0;
        sar->seg_n = seg_n;
        sar->block_ack = 0;
        sar->rx_meta_info = *rx_meta_info;

        if (0x1F == seg_n)
        {
            sar->expected_block_ack = 0xFFFFFFFF;
        }
        else
        {
            sar->expected_block_ack = (UINT32)((1 << (seg_n + 1)) - 1);
        }

        /* Calculate SeqZero and provide that as SeqNum to Upper Transport Layer */
        sar->seq_auth = ltran_calculate_seq_auth(net_hdr->seq_num, sar->seq_zero);
    }

    /**
        Start Ack Timer if it is not already running and
        if the destinition address is unicast address,
        so this is to be acked.
    */

    if(daddr_type == MS_NET_ADDR_TYPE_UNICAST)  //by hq
    {
        ltrn_sar_start_ack_timer(sar);
//        ltrn_sar_restart_incomplete_timer(sar);
    }

    ltrn_sar_restart_incomplete_timer(sar);

    /* Check if the segment is already received */
    if (0 != (sar->block_ack & (1 << seg_o)))
    {
        LTRN_TRC("[LTRN Rx] Already received Seg:0x%02X. Ignoring\n", seg_o);
        return API_SUCCESS;
    }

    /* From the Final Segment, calculate the packet length */
    if (seg_n == seg_o)
    {
        /**
            Calculate Message Length.

            One octet (for Opcode or AKF|AID), followed by 'seg_n' number of segments,
            and the final segment length;
        */
        sar->data_length = 1 + (seg_n * seg_length) + (pdatalen - 4);
        LTRN_TRC("[LTRN] Reassembled buffer length will be %d octets. Last segment length %d\n",
                 sar->data_length, (pdatalen - 4));
    }
    else if (seg_length != (pdatalen - 4))
    {
        LTRN_ERR("[LTRN Rx] Incorrect non-final segment length 0x%02X != 0x%02X for message type 0x%02X\n",
                 (pdatalen - 4), seg_length, sar->ctl);
    }

    /* Copy the segments */
    EM_mem_copy
    (
        &(sar->data[1 + (seg_o * seg_length)]),
        &pdata[4],
        (pdatalen - 4)
    );
    /* Mark received segment */
    sar->block_ack |= (1 << seg_o);
    LTRN_TRC("[LTRN Rx] Updated BlockAck 0x%08X.\n", sar->block_ack);

    /* Check if complete frame is received */
    if (sar->block_ack == sar->expected_block_ack)
    {
        LTRN_TRC("[LTRN %p] Reassembly complete.\n", sar);

        /* Stop Timer */
//        ltrn_sar_stop_ack_timer(sar);

        if(daddr_type == MS_NET_ADDR_TYPE_UNICAST) //by hq
        {
            /* Send Block Ack */
            ltrn_send_ack
            (
                save_daddr,
                net_hdr->saddr,
                subnet_handle,
                sar->seq_zero,
                0x7F, /* net_hdr->ttl, */
                obo,
                sar->block_ack
            );
        }

        /* Reset the SEG field */
        sar->data[0] = pdata[0] & 0x7F;

        /* Callback to Upper Layer */
        if ((NULL != ltrn_callback) &&
                ((LTRN_RX_FOR_LOCAL_ELEMENT == (rx_meta_info->is_for & LTRN_RX_FOR_LOCAL_ELEMENT)) ||
                 (LTRN_RX_FOR_LOCAL_SUBSCRIPTION == (rx_meta_info->is_for & LTRN_RX_FOR_LOCAL_SUBSCRIPTION))))
        {
            /* Calculate SeqZero and provide that as SeqNum to Upper Transport Layer */
            net_hdr->seq_num = ltran_calculate_seq_auth(net_hdr->seq_num, sar->seq_zero);
            LTRN_TRC("[LTRN Rx] Invoke Upper Transport Callback\n");
            LTRN_TRC("[LTRN Rx] Dumping the Reassembled Payload\n");
            LTRN_debug_dump_bytes(sar->data, sar->data_length);
            LTRN_UNLOCK();
            /* Control message do not have a TransMIC */
            ltrn_callback(net_hdr, subnet_handle, szmic, sar->data, sar->data_length);
            LTRN_LOCK();
        }
        /* TODO: Should we unlock and call */
        /* Put in LPN Queue */
        else
        {
            UINT8 q_pkt_type;

            if (0 == seg_n)
            {
                q_pkt_type = 0x00;
            }
            else
            {
                q_pkt_type = 0x03;
            }

            #ifdef MS_FRIEND_SUPPORT

            if (LTRN_RX_FOR_LPN_ELEMENT == (rx_meta_info->is_for & LTRN_RX_FOR_LPN_ELEMENT))
            {
                LTRN_TRC("[TRN] Pkt for LPN:0x%02X.To be handled\n", rx_meta_info->lpn_handle);
                trn_add_to_queue
                (
                    rx_meta_info->lpn_handle,
                    q_pkt_type, /* Final or only Segment */
                    net_hdr,
                    pdata,
                    (UINT8)pdatalen
                );
            }
            else if (LTRN_RX_FOR_LPN_SUBSCRIPTION == (rx_meta_info->is_for & LTRN_RX_FOR_LPN_SUBSCRIPTION))
            {
                do
                {
                    LTRN_TRC("[TRN] Pkt for LPN subscription list. To be searched 0x%02X\n", rx_meta_info->lpn_handle);
                    trn_add_to_queue
                    (
                        /* TODO: Change the LPN Handle handling for subscription list */
                        rx_meta_info->lpn_handle,
                        q_pkt_type, /* Final or only Segment */
                        net_hdr,
                        pdata,
                        (UINT8)pdatalen
                    );
                    /* Get next LPN handle if any */
                    retval = MS_trn_is_valid_lpn_subscription_address(net_hdr->daddr, &rx_meta_info->lpn_handle);
                }
                while (API_SUCCESS == retval);
            }

            #endif /* MS_FRIEND_SUPPORT */
            /* Calculate SeqZero and provide that as SeqNum to Upper Transport Layer */
            net_hdr->seq_num = ltran_calculate_seq_auth(net_hdr->seq_num, sar->seq_zero);
        }

        /* Add to Completed SAR Rx Cache */
        ltrn_add_to_reassembled_cache
        (
            net_hdr->saddr,
            net_hdr->ivi,
            net_hdr->seq_num,
            0x00 /* Success */
        );
        /* Free SAR Context */
        ltrn_sar_free_ctx(sar);
    }

    #ifdef MS_FRIEND_SUPPORT
    else
    {
        /* TODO: Should we unlock and call */
        /* Put in LPN Queue */
        if (LTRN_RX_FOR_LPN_ELEMENT == (rx_meta_info->is_for & LTRN_RX_FOR_LPN_ELEMENT))
        {
            LTRN_TRC("[TRN] Pkt for LPN:0x%02X.To be handled\n", rx_meta_info->lpn_handle);
            trn_add_to_queue
            (
                rx_meta_info->lpn_handle,
                0x01, /* Non-final segment */
                net_hdr,
                pdata,
                (UINT8)pdatalen
            );
        }
        else if (LTRN_RX_FOR_LPN_SUBSCRIPTION == (rx_meta_info->is_for & LTRN_RX_FOR_LPN_SUBSCRIPTION))
        {
            do
            {
                LTRN_TRC("[TRN] Pkt for LPN subscription list. To be searched 0x%02X\n", rx_meta_info->lpn_handle);
                trn_add_to_queue
                (
                    /* TODO: Change the LPN Handle handling for subscription list */
                    rx_meta_info->lpn_handle,
                    0x01, /* Non-final segment */
                    net_hdr,
                    pdata,
                    (UINT8)pdatalen
                );
                /* Get next LPN handle if any */
                retval = MS_trn_is_valid_lpn_subscription_address(net_hdr->daddr, &rx_meta_info->lpn_handle);
            }
            while (API_SUCCESS == retval);
        }
    }

    #endif /* MS_FRIEND_SUPPORT */
    return retval;
}
#endif /* MS_LTRN */

