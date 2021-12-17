/**
    \file trn_api.c

    This file defines the Transport Layer Application Inerface Methods
*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "trn_internal.h"
#include "trn_extern.h"

#include "cry.h"
#include "sec_tbx.h"
extern                     BRR_BEARER_INFO blebrr_adv;  //HZF
static inline uint32 clock_time_rtc(void)
{
    return (*(volatile unsigned int*)0x4000f028) & 0xffffff;
}

#ifdef MS_TRN

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */

/**
    \brief Register Interface with Transport Layer

    \par Description
    This routine registers interface with the Transport Layer.
    Transport Layer supports single Application, hence this routine shall be called once.

    \param [in] trn_cb
           Upper Layer Notification Callback for specific message type

    \param [in] msg_type
           Message type (Control or Access) for which the callback to be called.

    \return API_SUCCESS or an error code indicating reason for failure

*/
API_RESULT MS_trn_register
(
    /* IN */ TRN_NTF_CB        trn_cb,
    /* IN */ MS_TRN_MSG_TYPE   msg_type
)
{
    TRN_LOCK();

    /* Register the callback */

    /** Transport Layer Control Packet */
    if (MS_TRN_CTRL_PKT == msg_type)
    {
        trn_ctrl_callback = trn_cb;
    }
    /** Access Layer Packet */
    else if (MS_TRN_ACCESS_PKT == msg_type)
    {
        trn_access_callback = trn_cb;
    }

    TRN_UNLOCK();
    return API_SUCCESS;
}

API_RESULT MS_trn_heartbeat_register
(
    /* IN */ TRN_HEARTBEAT_RCV_CB rcv_cb,
    /* IN */ TRN_HEARTBEAT_RCV_TIMEOUT_CB rcv_to_cb
)
{
    TRN_LOCK();
    /* Register the callback */
    trn_heartbeat_rcv_callback = rcv_cb;
    trn_heartbeat_rcv_timeout_callback = rcv_to_cb;
    return API_SUCCESS;
}



/**
    \brief API to send Access Layer PDUs

    \par Description
    This routine sends Access Layer PDUs to peer device.

    \param [in] src_addr
           Source Address

    \param [in] dst_addr
           Destination Address

    \param [in] label
           Lable UUID, represending Virtual Address of Destination

    \param [in] subnet_handle
           Handle identifying the Subnet

    \param [in] appkey_handle
           Handle identifying the AppKey to be used for Transport Layer encryption.

    \param [in] ttl
           Time to Live

    \param [in] param
           Transport parameter, based on the type and header

    \param [in] reliable
           If requires lower transport Ack, set reliable as TRUE

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_send_access_pdu
(
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ UINT8*                   label,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT8                    ttl,
    /* IN */ void*                    param,
    /* IN */ UINT8                    reliable
)
{
    API_RESULT retval;
    UCHAR   pdu[TRN_MAX_ACCESS_PKT_SIZE];
    UINT16  marker;
    UCHAR   trans_mic_length;
    UCHAR   akf, aid;
    UCHAR* key;
    UINT8   key_type;
    UINT32  seq_num;
    TRN_TRC("[TRN] Tx Access Pkt. SRC:0x%04X, DST:0x%04X, SH:0x%04X, AH: 0x%04X, TTL:0x%02X\n",
            saddr, daddr, subnet_handle, appkey_handle, ttl);
//    printf("[TRN] Tx Access Pkt. SRC:0x%04X, DST:0x%04X, SH:0x%04X, AH: 0x%04X, TTL:0x%02X\n",
//            saddr, daddr, subnet_handle, appkey_handle, ttl);

    /* Check if valid AppKey */
    if ((MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS) + MS_CONFIG_LIMITS(MS_MAX_APPS)) <= appkey_handle)
    {
        TRN_ERR("[TRN] Invalid Handle. Returning\n");
        return TRN_PARAMETER_OUTSIDE_RANGE;
    }

    /* TODO: When the TransMIC will be 8 octet? */
    trans_mic_length = 0x04;

    /**
        Mutex Lock/Unlock not required, as not accessing any of the transport
        layer global data structures
    */

    /* Check if Application or Device Key to be used */
    if (MS_CONFIG_LIMITS(MS_MAX_APPS) <= appkey_handle)
    {
        /* Use Device Key */
        MS_access_cm_get_device_key((UINT8)(appkey_handle - MS_CONFIG_LIMITS(MS_MAX_APPS)), &key);
        key_type = MS_NONCE_T_DEVICE;
        aid = 0;
        akf = 0;
    }
    else
    {
        retval = MS_access_cm_get_app_key
                 (
                     appkey_handle,
                     &key,
                     &aid
                 );

        if (API_SUCCESS != retval)
        {
            TRN_ERR("[TRN] Failed to file AppKey for Handle 0x%04X\n",
                    appkey_handle);
            return retval;
        }

        key_type = MS_NONCE_T_APPLICATION;
        akf = 1;
    }

    MS_net_alloc_seq_num(&seq_num);
    TRN_TRC("[TRN Tx] Allocated Sequence Number: 0x%08X\n", seq_num);
    trn_frame_secure_pdu
    (
        saddr,
        daddr,
        label,
        (MS_BUFFER*)param,
        seq_num,
        key,
        key_type,
        trans_mic_length,
        pdu
    );
    /* Secured PDU size will be Access PDU length + TransMIC Length */
    marker = ((MS_BUFFER*)param)->length + trans_mic_length;
    /* Already added the TransportMIC to frame the Upper Transport PDU */
    /* Send to Lower Transport, where the fragmentation will be performed (if required) */
    /* Send PDU to Lower Transport Layer */
    retval = MS_ltrn_send_pdu
             (
                 saddr,
                 daddr,
                 subnet_handle,
                 MS_TRN_ACCESS_PKT,
                 ttl,
                 akf,
                 aid,
                 seq_num,
                 pdu,
                 marker,
                 reliable
             );
    return retval;
}

/**
    \brief API to send transport PDUs

    \par Description
    This routine sends transport PDUs to peer device.

    \param [in] daddr
           Destination Address

    \param msg_type
           Transport Control Format type

    \param [in] ttl
           Time to Live

    \param ctrl_opcode
           Control Packet Opcode. For non-control packet, this value
           will be MS_TRN_CTRL_OPCODE_INVALID

    \param hdr
           Transport Header (depends on the tcf_type)

    \param param
           Transport parameter, based on the type and header

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_send_control_pdu
(
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ UINT8                    ttl,
    /* IN */ MS_TRN_CTRL_PKT_OPCODE   ctrl_opcode,
    /* IN */ void*                    param
)
{
    API_RESULT retval;
    UCHAR pdu[TRN_MAX_CTL_PKT_SIZE];
    UCHAR marker;
    UCHAR akf, aid;
    UINT32 seq_num;
    TRN_TRC("[TRN] Tx Control Pkt. SRC:0x%04X, DST:0x%04X, SH:0x%04X, Opcode:0x%02X, TTL:0x%02X\n",
            saddr, daddr, subnet_handle, ctrl_opcode, ttl);
    /* Initialize marker */
    marker = 0;
    akf = 0;
    aid = 0;
    /* Check Control Packet Opcode */
    TRN_RANGE_CHECK_END(ctrl_opcode, MS_TRN_CTRL_OPCODE_HEARTBEAT);
    /* Fill Opcode */
    pdu[marker] = ctrl_opcode;
    marker ++;

    /**
        The Upper Transport Control PDU is not authenticated at
        the upper transport layer and instead relies upon
        the authentication performed by the network layer.
        All Upper Transport Control PDUs use a 64 - bit NetMIC.
    */

    /** Upper Transport Layer is Big Endian */
    switch (ctrl_opcode)
    {
        #ifdef MS_FRIEND_SUPPORT

    /* Friend Poll */
    case MS_TRN_CTRL_OPCODE_FRND_POLL:
    {
        MS_TRN_FRND_POLL_PARAM* frnd_poll_param;
        frnd_poll_param = (MS_TRN_FRND_POLL_PARAM*)param;
        /* Pack Parameters */
        /* TODO: Ensure RFU bits are correct */
        pdu[marker] = frnd_poll_param->fsn;
        marker++;
    }
    break;

    /* Friend Update */
    case MS_TRN_CTRL_OPCODE_FRND_UPDATE:
    {
        MS_TRN_FRND_UPDATE_PARAM* frnd_update_param;
        frnd_update_param = (MS_TRN_FRND_UPDATE_PARAM*)param;
        /* Pack Parameters */
        pdu[marker] = frnd_update_param->flags;
        marker++;
        MS_PACK_BE_4_BYTE_VAL(&pdu[marker], frnd_update_param->ivi);
        marker += 4;
        pdu[marker] = frnd_update_param->md;
        marker++;
    }
    break;

    /* Friend Request */
    case MS_TRN_CTRL_OPCODE_FRND_REQ:
    {
        MS_TRN_FRND_REQ_PARAM* frnd_req_param;
        UCHAR frnd_index;
        frnd_req_param = (MS_TRN_FRND_REQ_PARAM*)param;
        /* Pack Parameters */
        pdu[marker] = frnd_req_param->criteria;
        marker++;
        pdu[marker] = frnd_req_param->rx_delay;
        marker++;
        MS_PACK_BE_3_BYTE_VAL(&pdu[marker], frnd_req_param->poll_to);
        marker += 3;
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], frnd_req_param->prev_addr);
        marker += 2;
        pdu[marker] = frnd_req_param->num_elem;
        marker++;
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], frnd_req_param->lpn_counter);
        marker += 2;
        /* For the Friend Request the Destination Address will be 'All-Friends' */
        daddr = MS_NET_ADDR_ALL_FRIENDS;
        /* Lock */
        TRN_LOCK();
        /* Set Friend Role as LPN */
        trn_frnd_role = MS_FRND_ROLE_LPN;
        /* Unlock */
        TRN_UNLOCK();
        /* Add these information to Friend List */
        /* Currently only using index 0 */
        frnd_index = 0;
    }
    break;

    /* Friend Offer */
    case MS_TRN_CTRL_OPCODE_FRND_OFFER:
    {
        MS_TRN_FRND_OFFER_PARAM* frnd_offer_param;
        frnd_offer_param = (MS_TRN_FRND_OFFER_PARAM*)param;
        /* Pack Parameters */
        pdu[marker] = frnd_offer_param->rx_window;
        marker++;
        pdu[marker] = frnd_offer_param->queue_size;
        marker++;
        pdu[marker] = frnd_offer_param->sublist_size;
        marker++;
        pdu[marker] = frnd_offer_param->rssi;
        marker++;
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], frnd_offer_param->frnd_counter);
        marker += 2;
    }
    break;

    /* Friend Clear */
    case MS_TRN_CTRL_OPCODE_FRND_CLEAR:
    {
        MS_TRN_FRND_CLEAR_PARAM* frnd_clear_param;
        frnd_clear_param = (MS_TRN_FRND_CLEAR_PARAM*)param;
        /* Pack Parameters */
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], frnd_clear_param->lpn_addr);
        marker += 2;
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], frnd_clear_param->lpn_counter);
        marker += 2;
    }
    break;

    /* Friend Clear Confirm */
    case MS_TRN_CTRL_OPCODE_FRND_CLEAR_CNF:
    {
        MS_TRN_FRND_CLEAR_CNF_PARAM* frnd_clear_cnf_param;
        frnd_clear_cnf_param = (MS_TRN_FRND_CLEAR_CNF_PARAM*)param;
        /* Pack Parameters */
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], frnd_clear_cnf_param->lpn_addr);
        marker += 2;
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], frnd_clear_cnf_param->lpn_counter);
        marker += 2;
    }
    break;

    /* Friend Subscription List Add */
    case MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_ADD: /* Fall Through */

    /* Friend Subscription List Remove */
    case MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_REMOVE:
    {
        MS_TRN_FRND_MANAGE_PARAM* frnd_manage_param;
        UINT16* paddr;
        UCHAR count;
        frnd_manage_param = (MS_TRN_FRND_MANAGE_PARAM*)param;
        /* Pack Parameters */
        pdu[marker] = frnd_manage_param->txn_num;
        marker++;

        /* TODO: Ensure the buffer size is within limit */
        for (count = 0; count < frnd_manage_param->num_addr; count ++)
        {
            paddr = ((UINT16*)(frnd_manage_param->addr_list)) + count;
            MS_PACK_BE_2_BYTE(&pdu[marker], paddr);
            marker += 2;
        }
    }
    break;

    /* Friend Subscription List Confirmation */
    case MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_CNF:
    {
        MS_TRN_FRND_SUBSCRN_LIST_CNF_PARAM* frnd_subscrn_list_cnf_param;
        frnd_subscrn_list_cnf_param = (MS_TRN_FRND_SUBSCRN_LIST_CNF_PARAM*)param;
        /* Pack Parameters */
        pdu[marker] = frnd_subscrn_list_cnf_param->txn_num;
        marker++;
    }
    break;
    #endif

    /* Heartbeat */
    case MS_TRN_CTRL_OPCODE_HEARTBEAT:
    {
        MS_TRN_HEARTBEAT_PARAM* heartbeat_param;
        heartbeat_param = (MS_TRN_HEARTBEAT_PARAM*)param;
        /* Pack Parameters */
        pdu[marker] = heartbeat_param->init_ttl;
        marker++;
        MS_PACK_BE_2_BYTE_VAL(&pdu[marker], heartbeat_param->features);
        marker += 2;
    }
    break;

    default:
        break;
    }

    /* Send PDU to Lower Transport Layer */
    MS_net_alloc_seq_num(&seq_num);
//
//UINT32    T1, T2;
//T1 = clock_time_rtc();                  //HZF
    retval = MS_ltrn_send_pdu
             (
                 saddr,
                 daddr,
                 subnet_handle,
                 MS_TRN_CTRL_PKT,
                 ttl,
                 akf,
                 aid,
                 seq_num,
                 pdu,
                 marker,
                 MS_FALSE
             );
//T2 = clock_time_rtc();                  //HZF
//printf("consume time MS_ltrn_send_pdu: %d\r\n", (T2 - T1))    ;
    return retval;
}

/* Publication Timeout Handler */
static void trn_start_heartbeat_pub_timer(/* IN */ UINT32 timeout);

static void trn_pub_timeout_handler (void* args, UINT16 size)
{
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);
    TRN_TRC("[TRN] Publication Period Timeout\n");
    /* Lock */
    /* Stop Publishing */
    heartbeat_pub.timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    MS_trn_trigger_heartbeat(0);

    if (0 != heartbeat_pub.tx_count)
    {
        if (0xFFFF != heartbeat_pub.tx_count)
        {
            heartbeat_pub.tx_count--;
        }

        trn_start_heartbeat_pub_timer(1 << (heartbeat_pub.period_log - 1));
    }
    else
    {
        heartbeat_pub.period_log = 0;
    }

    /* Unlock */
    return;
}

void trn_stop_heartbeat_pub_timer(void)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    TRN_TRC("[TRN] Stop Heartbeat Publication Timer.\n");

    if (EM_TIMER_HANDLE_INIT_VAL != heartbeat_pub.timer_handle)
    {
        retval = EM_stop_timer(&heartbeat_pub.timer_handle);
    }
}


/**
    \brief

    \par Description

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_trigger_heartbeat(/* IN */ UINT8 change_in_feature_bit)
{
    UINT16 uaddr;
    API_RESULT retval;
    MS_TRN_HEARTBEAT_PARAM heartbeat_param;
    UINT8                  features, friend_role;

    /* Check if to be triggered */
    if ((change_in_feature_bit & heartbeat_pub.features) != change_in_feature_bit)
    {
        TRN_TRC("[HBT] MS_trn_trigger_heartbeat returning failure. Feature:0x%04X. Change Bit:0x%02X\n",
                heartbeat_pub.features, change_in_feature_bit);
        return API_FAILURE;
    }

    MS_access_cm_get_primary_unicast_address(&uaddr);
    TRN_TRC("[HBT] MS_trn_trigger_heartbeat\n");
    /* If publishing ongoing, then return */
    /* Send the packet with TTL 0 */
    heartbeat_param.init_ttl = 0x7F;
    MS_access_cm_get_features(&features);
    MS_access_cm_get_friendship_role(&friend_role);

    if (MS_FRND_ROLE_LPN != friend_role)
    {
        features &= 0x07;
    }

    heartbeat_param.features = features;
    retval = MS_trn_send_control_pdu
             (
                 uaddr,
                 heartbeat_pub.daddr,
                 heartbeat_pub.subnet_handle,
                 heartbeat_param.init_ttl,
                 MS_TRN_CTRL_OPCODE_HEARTBEAT,
                 &heartbeat_param
             );
    return retval;
}


static void trn_start_heartbeat_pub_timer(/* IN */ UINT32 timeout)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    TRN_TRC("[TRN] Heartbeat Publication Timer. TO:0x%08X\n", timeout);

    /**
        Do we need to stop the timer in between?
    */
    if(EM_TIMER_HANDLE_INIT_VAL == heartbeat_pub.timer_handle)
    {
        retval = EM_start_timer
                 (
                     &heartbeat_pub.timer_handle,
                     timeout,
                     trn_pub_timeout_handler,
                     NULL,
                     0
                 );
    }
}

API_RESULT MS_trn_set_heartbeat_publication
(
    /* INOUT */ MS_TRN_HEARTBEAT_PUBLICATION_INFO* pub
)
{
    API_RESULT retval;
    /* Validate Input parameters */
    retval = API_SUCCESS;

    /* Lock */
    if (MS_NET_ADDR_UNASSIGNED == pub->daddr)
    {
        EM_mem_set(&heartbeat_pub, 0, sizeof(heartbeat_pub));
        /* TODO: Check. Done to pass PTS test case */
        /**
            When the Destination is set to the unassigned address,
            the values of the CountLog, PeriodLog, and TTL fields
            shall be set to 0x00
        */
        heartbeat_pub.daddr = MS_NET_ADDR_UNASSIGNED;
        heartbeat_pub.count_log = 0;
        heartbeat_pub.period_log = 0;
    }
    else
    {
        /* Check if valid NetKeyIndex */
        heartbeat_pub.subnet_handle = MS_INVALID_SUBNET_HANDLE;
        heartbeat_pub.netkey_index = pub->netkey_index;
        retval = MS_access_cm_find_subnet
                 (
                     heartbeat_pub.netkey_index,
                     &heartbeat_pub.subnet_handle
                 );

        if (API_SUCCESS != retval)
        {
            retval = MS_INVALID_NETKEY_INDEX;
            trn_stop_heartbeat_pub_timer();
        }
        else
        {
            /* Update the global heartbeat publication information */
            heartbeat_pub.daddr = pub->daddr;
            heartbeat_pub.count_log = pub->count_log;
            heartbeat_pub.period_log = pub->period_log;
            heartbeat_pub.ttl = pub->ttl;
            heartbeat_pub.features = (pub->features & TRN_HBP_FEATURE_MASK);
        }
    }

    pub->daddr = heartbeat_pub.daddr;
    pub->count_log = heartbeat_pub.count_log;
    pub->features = heartbeat_pub.features;
    pub->netkey_index = heartbeat_pub.netkey_index;
    pub->period_log = heartbeat_pub.period_log;
    pub->ttl = heartbeat_pub.ttl;

    if ((API_SUCCESS == retval) && (0x00 != heartbeat_pub.period_log) && (0x00 != heartbeat_pub.count_log))
    {
        UINT32 timeout;
        timeout = (1 << (heartbeat_pub.period_log - 1));

        if (0xFF != heartbeat_pub.count_log)
        {
            heartbeat_pub.tx_count = (1 << (heartbeat_pub.count_log - 1));
        }
        else
        {
            heartbeat_pub.tx_count = 0xFFFF;
        }

        /* Start a Timer */
        trn_stop_heartbeat_pub_timer();
        trn_start_heartbeat_pub_timer(timeout);
    }
    else
    {
        heartbeat_pub.tx_count = 0;
    }

    printf("<< MS_trn_set_heartbeat_publication\n");
    /* Unlock */
    return retval;
}


API_RESULT MS_trn_get_heartbeat_publication
(
    /* OUT */ MS_TRN_HEARTBEAT_PUBLICATION_INFO* info
)
{
    /* Lock */
    /* Fetch the global heartbeat publication information */
    info->daddr = heartbeat_pub.daddr;
    info->count_log = heartbeat_pub.count_log;
    info->period_log = heartbeat_pub.period_log;
    info->ttl = heartbeat_pub.ttl;
    info->features = heartbeat_pub.features;
    info->netkey_index = heartbeat_pub.netkey_index;
    /* Unlock */
    return API_SUCCESS;
}


/* Subscription Timeout Handler */
static void trn_sub_timeout_handler (void* args, UINT16 size)
{
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);
    TRN_TRC("[TRN] Subscription Period Timeout\n");
    printf("[TRN] Subscription Period Timeout\n");

    /* Lock */
    if(trn_heartbeat_rcv_timeout_callback != NULL)
    {
        trn_heartbeat_rcv_timeout_callback();
    }

    /* Stop Logging */
    heartbeat_sub.timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    heartbeat_sub.period_log = 0;
    /* Unlock */
    return;
}

void trn_stop_heartbeat_sub_timer(void)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    TRN_TRC("[TRN] Stop Heartbeat Subscription Timer. %p\n", heartbeat_sub.timer_handle);

    if (EM_TIMER_HANDLE_INIT_VAL != heartbeat_sub.timer_handle)
    {
        retval = EM_stop_timer(&heartbeat_sub.timer_handle);
    }
}

void trn_start_heartbeat_sub_timer(/* IN */ UINT32 timeout)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    TRN_TRC("[TRN] Heartbeat Subscription Timer. TO:0x%08X\n", timeout);

    /**
        Do we need to stop the timer in between?
    */
    if(EM_TIMER_HANDLE_INIT_VAL == heartbeat_sub.timer_handle)
    {
        retval = EM_start_timer
                 (
                     &heartbeat_sub.timer_handle,
                     timeout,
                     trn_sub_timeout_handler,
                     NULL,
                     0
                 );
        TRN_TRC("[TRN] Heartbeat Subscription Timer Start. Retval:0x%04X. THandle:%p\n",
                retval, heartbeat_sub.timer_handle);
    }
}

void trn_heartbeat_timer_restart(void)
{
    UINT32 timeout = (1 << (heartbeat_sub.period_log - 1));

    if(EM_TIMER_HANDLE_INIT_VAL != heartbeat_sub.timer_handle)
    {
        trn_stop_heartbeat_sub_timer();
    }

    trn_start_heartbeat_sub_timer(timeout);
}


API_RESULT MS_trn_set_heartbeat_subscription
(
    /* INOUT */ MS_TRN_HEARTBEAT_SUBSCRIPTION_INFO* info
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    /* Lock */
    TRN_TRC("[TRN] >> MS_trn_set_heartbeat_subscription\n");
    TRN_TRC("[TRN] SADDR:0x%04X, DADDR:0x%04X, PeriodLog:0x%02X\n",
            info->saddr, info->daddr, info->period_log);

    /**
        Check if Heartbeat Subscription Source or Destination is Unassigned Address,
        make the context inactive
    */
    /* Update the global heartbeat subscription information */
    if ((MS_NET_ADDR_UNASSIGNED == info->saddr) || (MS_NET_ADDR_UNASSIGNED == info->daddr) ||
            (0x00 == info->period_log))
    {
        heartbeat_sub.saddr = MS_NET_ADDR_UNASSIGNED;
        heartbeat_sub.daddr = MS_NET_ADDR_UNASSIGNED;
        heartbeat_sub.period_log = 0;
    }
    else
    {
        UINT8 daddr_type;
        UINT8 saddr_type;
        UINT16 uaddr;
        /* Get SRC and DST Address Types */
        daddr_type = MS_net_get_address_type(info->daddr);
        saddr_type = MS_net_get_address_type(info->saddr);
        MS_access_cm_get_primary_unicast_address(&uaddr);

        /**
            SRC Addr Type can be Unassigned or Unicast.
            DST Addr Type can be Unassigned or Unicast or Group Address
        */
        if ((MS_NET_ADDR_TYPE_UNICAST != saddr_type) ||
                ((MS_NET_ADDR_TYPE_UNICAST != daddr_type) && (MS_NET_ADDR_TYPE_GROUP != daddr_type)) ||
                ((MS_NET_ADDR_TYPE_UNICAST == daddr_type) && (uaddr != info->daddr)))
        {
            TRN_ERR(
                "[TRN] Invalid Address Type. Resetting Period Log. Returning ...\n");
            heartbeat_sub.period_log = 0x00;
            retval = MS_INVALID_ADDRESS;
        }
        else
        {
            UINT32 timeout;

            if (MS_NET_ADDR_TYPE_GROUP == daddr_type)
            {
                MS_ACCESS_ADDRESS       sub_addr;
                sub_addr.use_label = 0;
                sub_addr.addr = info->daddr;
                MS_access_cm_add_model_subscription
                (
                    0, /* MS_ACCESS_MODEL_HANDLE    model_handle for config_server */
                    &sub_addr /* IN MS_ACCESS_ADDRESS       * sub_addr */
                );
            }

            heartbeat_sub.saddr = info->saddr;
            heartbeat_sub.daddr = info->daddr;
            heartbeat_sub.period_log = info->period_log;
            heartbeat_sub.min_hops = 0x7F;
            heartbeat_sub.max_hops = 0x00;
            /* Reset Count Log state, if Period Log is not zero */
            heartbeat_sub.count_log = 0;
            heartbeat_sub.count = 0;
            timeout = (1 << (heartbeat_sub.period_log - 1));
            /* Start a Timer */
            TRN_TRC("[TRN] Starting Heartbeat Subscription Timer for %d seconds\n", timeout);
            trn_stop_heartbeat_sub_timer();
            trn_start_heartbeat_sub_timer(timeout);
        }
    }

    info->saddr = heartbeat_sub.saddr;
    info->daddr = heartbeat_sub.daddr;
    info->count_log = heartbeat_sub.count_log;
    info->period_log = heartbeat_sub.period_log;
    info->min_hops = heartbeat_sub.min_hops;
    info->max_hops = heartbeat_sub.max_hops;
    /* Unlock */
    TRN_TRC("[TRN] SADDR:0x%04X, DADDR:0x%04X, PeriodLog:0x%02X, CountLog:0x%02X, MinHops:0x%02X, MaxHops:0x%02X\n",
            info->saddr, info->daddr, info->period_log, info->count_log, info->min_hops, info->max_hops);
    TRN_TRC("<< MS_trn_set_heartbeat_subscription\n");
    printf("<< MS_trn_set_heartbeat_subscription\n");
    return retval;
}

API_RESULT MS_trn_get_heartbeat_subscription
(
    /* OUT */ MS_TRN_HEARTBEAT_SUBSCRIPTION_INFO* info
)
{
    /* Lock */
    if ((MS_NET_ADDR_UNASSIGNED == heartbeat_sub.saddr) ||
            (MS_NET_ADDR_UNASSIGNED == heartbeat_sub.daddr))
    {
        info->saddr = MS_NET_ADDR_UNASSIGNED;
        info->daddr = MS_NET_ADDR_UNASSIGNED;
        info->period_log = 0;
        info->min_hops = 0;
        info->max_hops = 0;
    }
    else
    {
        /* Fetch the global heartbeat subscription information */
        info->saddr = heartbeat_sub.saddr;
        info->daddr = heartbeat_sub.daddr;
        info->period_log = heartbeat_sub.period_log;
        info->min_hops = heartbeat_sub.min_hops;
        info->max_hops = heartbeat_sub.max_hops;
    }

    info->count_log = heartbeat_sub.count_log;
    /* Unlock */
    return API_SUCCESS;
}

/* Utility Function */
API_RESULT  trn_frame_secure_pdu
(
    /* IN */  MS_NET_ADDR   saddr,
    /* IN */  MS_NET_ADDR   daddr,
    /* IN */  UINT8*        label,
    /* IN */  MS_BUFFER*    pdu,
    /* IN */  UINT32        seq_num,
    /* IN */  UCHAR*        key,
    /* IN */  UCHAR         key_type,
    /* IN */  UCHAR         mic_len,
    /* OUT */ UCHAR*        sec_pdu
)
{
    UCHAR nonce[13];
    UCHAR marker;
    UCHAR mac[8];
    INT32 ret;
    UINT32 iv_index;
    UINT8* a;
    UINT16 al;
    TRN_TRC("[TRN] Tx Secure Access Pkt. SRC:0x%04X, KeyType:0x%02X, Mic Len:0x%04X\n",
            saddr, daddr, key_type, mic_len);
    marker = 0;
    /* Key Type: App or Device */
    nonce[marker ++] = key_type;
    /* Pad */
    nonce[marker ++] = 0x00;
    /* 3 Octet Sequence Number */
    MS_PACK_BE_3_BYTE_VAL(&nonce[marker], seq_num);
    marker += 3;
    /* 2 Octet Source Addr */
    MS_PACK_BE_2_BYTE_VAL(&nonce[marker], saddr);
    marker += 2;
    /* 2 Octet Dest Addr */
    MS_PACK_BE_2_BYTE_VAL(&nonce[marker], daddr);
    marker += 2;
    /* 4 Octet IV Index */
    MS_access_cm_get_iv_index(&iv_index, NULL);
    MS_PACK_BE_4_BYTE_VAL(&nonce[marker], iv_index);
    marker += 4;
    TRN_INF("[TRN] App/Dev Nonce\n");
    TRN_debug_dump_bytes(nonce, sizeof(nonce));
    TRN_INF("[TRN] Encryption Key");
    TRN_debug_dump_bytes(key, 16);
    TRN_INF("[TRN] Plain Text");
    TRN_debug_dump_bytes(pdu->payload, pdu->length);

    if (NULL == label)
    {
        a = NULL;
        al = 0;
    }
    else
    {
        a = label;
        al = MS_LABEL_UUID_LENGTH;
    }

    cry_aes_128_ccm_encrypt_be
    (
        key,
        nonce, sizeof(nonce),
        pdu->payload, pdu->length,
        a, al,
        sec_pdu,
        mac,
        mic_len,
        ret
    );
    TRN_INF("[TRN] Cipher Text");
    TRN_debug_dump_bytes(sec_pdu, pdu->length);
    TRN_INF("[TRN] MAC");
    TRN_debug_dump_bytes(mac, mic_len);
    /* TODO: Could be endianness issue */
    EM_mem_copy(sec_pdu + pdu->length, mac, mic_len);
    return API_SUCCESS;
}

#endif /* MS_TRN */

