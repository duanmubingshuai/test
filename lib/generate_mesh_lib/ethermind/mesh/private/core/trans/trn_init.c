
/**
    \file trn_init.c

    Module initialization routine and tables defined here.

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "trn_internal.h"
#include "trn.h"

#include "MS_access_api.h"

#include "cry.h"
#include "sec_tbx.h"

#ifdef MS_TRN
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */
MS_DEFINE_MUTEX (trn_mutex)

TRN_NTF_CB trn_ctrl_callback;
TRN_NTF_CB trn_access_callback;
TRN_HEARTBEAT_RCV_CB    trn_heartbeat_rcv_callback;
TRN_HEARTBEAT_RCV_TIMEOUT_CB    trn_heartbeat_rcv_timeout_callback;


#ifdef MS_FRIEND_SUPPORT
    /** Friend Role */
    UCHAR trn_frnd_role;

    /** LowPower Node Element Database */
    MS_DEFINE_GLOBAL_ARRAY(TRN_LPN_ELEMENT, lpn_element, MS_CONFIG_LIMITS(MS_MAX_LPNS));

    /** Friend Element Database */
    TRN_FRND_ELEMENT frnd_element;

    /* TODO: See if the above 2 can be merged */

    /**
    Global Friend counter. The number of Friend offer messages that
    has been sent by the Friend
    */
    UINT16 trn_frnd_counter;

    /**
    Global LPN counter. The number of Friend request messages that
    has been sent by the LPN
    */
    UINT16 trn_lpn_counter;

    /* Friendship Setup timeout. Application given friend establishment timeout */
    UINT32 frnd_setup_timeout;

    /* Friendship setup time lapsed count */
    UINT32 frnd_setup_time_lapsed;

    /* Friendship callback */
    TRN_FRND_CB frnd_cb;
#endif

/* Heartbeat Publication Information */
TRN_HEARTBEAT_PUBLICATION_STATE heartbeat_pub;

/* Heartbeat Subscription Information */
TRN_HEARTBEAT_SUBSCRIPTION_STATE heartbeat_sub;



/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */

/**
    \brief Initializes Module.

    \par Description Initializes Module tables.
*/
void ms_trn_init (void)
{
    UINT32     index;
    TRN_TRC("[TRN] Initializing Transport.");
    /* Module Mutex Initialization */
    MS_MUTEX_INIT_VOID (trn_mutex, TRN);
    /* Register with the Lower Transport layer */
    MS_ltrn_register(trn_pkt_in);
    #ifdef MS_FRIEND_SUPPORT
    /* Set Friend Role as Invalid */
    trn_frnd_role = MS_FRND_ROLE_INVALID;
    /* Initialize Friend List */
    MS_INIT_GLOBAL_ARRAY(TRN_LPN_ELEMENT, lpn_element, MS_CONFIG_LIMITS(MS_MAX_LPNS), 0x00);

    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_LPNS); index ++)
    {
        EM_mem_set(&lpn_element[index], 0x00, sizeof(TRN_LPN_ELEMENT));
        MS_INIT_GLOBAL_ARRAY(MS_NET_ADDR, lpn_element[index].subscription_list, MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE), 0x00);
        MS_INIT_GLOBAL_ARRAY(TRN_FRN_Q_ELEMENT, lpn_element[index].friend_queue[0].queue, MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE), 0x00);
        MS_INIT_GLOBAL_ARRAY(TRN_FRN_Q_ELEMENT, lpn_element[index].friend_queue[1].queue, MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE), 0x00);
        lpn_element[index].friend_queue[0].queue_size = 0;
        lpn_element[index].friend_queue[0].queue_start = 0;
        lpn_element[index].friend_queue[1].queue_size = 0;
        lpn_element[index].friend_queue[1].queue_start = 0;
        lpn_element[index].thandle = EM_TIMER_HANDLE_INIT_VAL;
        lpn_element[index].clrctx.thandle = EM_TIMER_HANDLE_INIT_VAL;
    }

    /* Initialize friend and LPN counter to default */
    trn_frnd_counter = 0;
    trn_lpn_counter = 0;
    /* Initialize Friend Element */
    EM_mem_set(&frnd_element, 0, sizeof(frnd_element));
    frnd_element.thandle = EM_TIMER_HANDLE_INIT_VAL;
    #endif
    /* Initialize Heartbeat Publication and Subscription states */
    EM_mem_set(&heartbeat_pub, 0, sizeof(heartbeat_pub));
    heartbeat_pub.timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    EM_mem_set(&heartbeat_sub, 0, sizeof(heartbeat_sub));
    heartbeat_sub.timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Initialize Heartbeat Subscription state */
    EM_mem_set(&heartbeat_sub, 0, sizeof(heartbeat_sub));
}


#ifndef MS_NO_SHUTDOWN
/**
    \par Description:
    This function is the shutdown handler for Transport module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.
*/
void ms_trn_shutdown (void)
{
    TRN_TRC("[TRN] Transport Shutdown Successful.");
}
#endif /* MS_NO_SHUTDOWN */

/**
    \par Description
    This function handles the incoming data received over lower transport layer.

    \param [in] net_hdr
           Received Network Packet Header
    \param [in] subnet_handle
           Handle identifying associated subnet on which the packet is received
    \param [in] szmic
           Field representing of Size of TransMIC.
    \param [in] pdata
           The incoming Data Packet
    \param [in] pdatalen
           Size of the incoming Data Packet

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT  trn_pkt_in
(
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UCHAR               szmic,
    /* IN */ UCHAR*              pdata,
    /* IN */ UINT16              pdatalen
)
{
    /* Upper Layer data and datalen */
    UCHAR* udata;
    UINT16  udatalen;
    UCHAR   trn_hdr_len;
    UINT16  marker;
    UCHAR   ctrl_opcode;
    UINT8   frnd;
    UCHAR* pdu;
    UCHAR tpdu[TRN_MAX_PKT_SIZE];
    MS_TRN_MSG_TYPE tcf;
    UCHAR trans_mic_length;
    MS_BUFFER       buffer;
    MS_APPKEY_HANDLE appkey_handle;
    API_RESULT retval;
    API_RESULT result;
    /* Initialize */
    retval = API_SUCCESS;
    result = API_SUCCESS;
    /* To keep some compilers happy */
    udata = NULL;
    udatalen = 0;
    TRN_TRC("[TRN 0x%04X] Lower Transport Callback. SRC:0x%04X, DST:0x%04X, CTL:0x%02X, TTL:0x%02X, szmic:0x%02X, Len:%d\n",
            subnet_handle, net_hdr->saddr, net_hdr->daddr, net_hdr->ctl, net_hdr->ttl, szmic, pdatalen);
//    printf("[TRN 0x%04X] Lower Transport Callback. SRC:0x%04X, DST:0x%04X, CTL:0x%02X, TTL:0x%02X, szmic:0x%02X, Len:%d\r\n",
//    subnet_handle, net_hdr->saddr, net_hdr->daddr, net_hdr->ctl, net_hdr->ttl, szmic, pdatalen);
    TRN_debug_dump_bytes(pdata, pdatalen);

    /* Strip headers if any and pass it to the registered layer above */

    /* Extract Transport Header */
    if (TCF_HDR_LEN_MIN > pdatalen)
    {
        TRN_ERR("[TRN] Incomplete Pkt Received of Len:%d. Dropping...\n", pdatalen);
        return TRN_INCOMPLETE_PKT_RECEIVED;
    }

    /* TBD: Extract and validate Transport Header Fields */
    trn_hdr_len = 0;
    tcf = net_hdr->ctl;
    appkey_handle = MS_INVALID_APPKEY_HANDLE;
    /* Lock */
    TRN_LOCK();

    switch (tcf)
    {
    /** Transport Layer Control Packet */
    case MS_TRN_CTRL_PKT:
    {
        /* TBD: Handle Internally or inform upper layer? */
        /* Ignoring More Data bit */
        ctrl_opcode = pdata[0] & 0x3F;
        TRN_TRC("[TRN] Rx Control Pkt. Opcode:0x%02X\n", ctrl_opcode);
//            printf("[TRN] Rx Control Pkt. Opcode:0x%02X\r\n", ctrl_opcode);
        marker = 1;
        pdu = pdata;

        /* TODO: Check TTL is ZERO, except for Clear request. */
        switch (ctrl_opcode)
        {
            #ifdef MS_FRIEND_SUPPORT

        /* Friend Poll */
        case MS_TRN_CTRL_OPCODE_FRND_POLL:
        {
            MS_TRN_FRND_POLL_PARAM frnd_poll_param;
            /* Unpack Parameters */
            /* TODO: Ensure RFU bits are correct */
            frnd_poll_param.fsn = pdu[marker];
            marker++;
            /* Handle Friend Poll */
            trn_handle_frnd_poll(net_hdr, subnet_handle, &frnd_poll_param);
        }
        break;

        /* Friend Request */
        case MS_TRN_CTRL_OPCODE_FRND_REQ:
        {
            MS_TRN_FRND_REQ_PARAM frnd_req_param;
            MS_access_cm_get_features_field(&frnd, MS_FEATURE_FRIEND);

            if (MS_TRUE != frnd)
            {
                break;
            }

            /* Pack Parameters */
            frnd_req_param.criteria = pdu[marker];
            marker++;
            frnd_req_param.rx_delay = pdu[marker];
            marker++;
            MS_UNPACK_BE_3_BYTE(&frnd_req_param.poll_to, &pdu[marker]);
            marker += 3;
            MS_UNPACK_BE_2_BYTE(&frnd_req_param.prev_addr, &pdu[marker]);
            marker += 2;
            frnd_req_param.num_elem = pdu[marker];
            marker++;
            MS_UNPACK_BE_2_BYTE(&frnd_req_param.lpn_counter, &pdu[marker]);
            marker += 2;
            /* Handle Friend Request */
            trn_handle_frnd_req(net_hdr, subnet_handle, &frnd_req_param);
        }
        break;

        /* Friend Clear */
        case MS_TRN_CTRL_OPCODE_FRND_CLEAR:
        {
            MS_TRN_FRND_CLEAR_PARAM frnd_clear_param;
            /* Pack Parameters */
            MS_UNPACK_BE_2_BYTE(&frnd_clear_param.lpn_addr, &pdu[marker]);
            marker += 2;
            MS_UNPACK_BE_2_BYTE(&frnd_clear_param.lpn_counter, &pdu[marker]);
            marker += 2;
            /* Handle Friend Clear */
            trn_handle_frnd_clear(net_hdr, subnet_handle, &frnd_clear_param);
        }
        break;

        /* Friend Subscription List Add */
        case MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_ADD: /* Fall Through */

        /* Friend Subscription List Remove */
        case MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_REMOVE:
        {
            MS_TRN_FRND_MANAGE_PARAM frnd_manage_param;
            /* Pack Parameters */
            frnd_manage_param.txn_num = pdu[marker];
            marker++;
            frnd_manage_param.addr_list = &pdu[marker];
            frnd_manage_param.num_addr = (UINT16)((pdatalen - marker) >> 1);
            frnd_manage_param.opcode = ctrl_opcode;
            /* Handle Friend Subscription List Add/Remove */
            trn_handle_frnd_subscription_list_add_remove(net_hdr, subnet_handle, &frnd_manage_param);
        }
        break;
            #endif /* MS_FRIEND_SUPPORT */
        #ifdef MS_LPN_SUPPORT

        /* Friend Update */
        case MS_TRN_CTRL_OPCODE_FRND_UPDATE:
        {
            MS_TRN_FRND_UPDATE_PARAM frnd_update_param;
            /* Unpack Parameters */
            frnd_update_param.flags = pdu[marker];
            marker++;
            MS_UNPACK_BE_4_BYTE(&frnd_update_param.ivi, &pdu[marker]);
            marker += 4;
            frnd_update_param.md = pdu[marker];
            marker++;
            /* Handle Friend Update */
            trn_handle_frnd_update(net_hdr, subnet_handle, &frnd_update_param);
            result = API_FAILURE;
        }
        break;

        /* Friend Offer */
        case MS_TRN_CTRL_OPCODE_FRND_OFFER:
        {
            MS_TRN_FRND_OFFER_PARAM frnd_offer_param;
            MS_access_cm_get_features_field(&frnd, MS_FEATURE_LPN);

            if (MS_TRUE != frnd)
            {
                break;
            }

            /* Pack Parameters */
            frnd_offer_param.rx_window = pdu[marker];
            marker++;
            frnd_offer_param.queue_size = pdu[marker];
            marker++;
            frnd_offer_param.sublist_size = pdu[marker];
            marker++;
            frnd_offer_param.rssi = pdu[marker];
            marker++;
            MS_UNPACK_BE_2_BYTE(&frnd_offer_param.frnd_counter, &pdu[marker]);
            marker += 2;
            /* Handle Friend Offer */
            trn_handle_frnd_offer(net_hdr, subnet_handle, &frnd_offer_param);
        }
        break;

        /* Friend Subscription List Confirmation */
        case MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_CNF:
        {
            MS_TRN_FRND_SUBSCRN_LIST_CNF_PARAM frnd_subscrn_list_cnf_param;
            /* Pack Parameters */
            frnd_subscrn_list_cnf_param.txn_num = pdu[marker];
            /* TODO: Handle */
            marker++;
            /* Handle Friend Subscription List Confirmation */
            trn_handle_frnd_subscription_list_cnf(net_hdr, subnet_handle, &frnd_subscrn_list_cnf_param);
        }
        break;
            #endif /* MS_LPN_SUPPORT */
        #if (defined MS_FRIEND_SUPPORT || defined MS_LPN_SUPPORT)

        /* Friend Clear Confirm */
        case MS_TRN_CTRL_OPCODE_FRND_CLEAR_CNF:
        {
            MS_TRN_FRND_CLEAR_CNF_PARAM frnd_clear_cnf_param;
            /* Pack Parameters */
            MS_UNPACK_BE_2_BYTE(&frnd_clear_cnf_param.lpn_addr, &pdu[marker]);
            marker += 2;
            MS_UNPACK_BE_2_BYTE(&frnd_clear_cnf_param.lpn_counter, &pdu[marker]);
            marker += 2;
            /* Handle Friend Clear Confirmation */
            trn_handle_frnd_clear_cnf(net_hdr, subnet_handle, &frnd_clear_cnf_param);
        }
        break;
            #endif /* (defined MS_FRIEND_SUPPORT || defined MS_LPN_SUPPORT) */

        /* Heartbeat */
        case MS_TRN_CTRL_OPCODE_HEARTBEAT:
        {
            MS_TRN_HEARTBEAT_PARAM heartbeat_param;
            /* Unpack Parameters */
            heartbeat_param.init_ttl = pdu[marker];
            marker++;
            MS_UNPACK_BE_2_BYTE(&heartbeat_param.features, &pdu[marker]);
            marker += 2;
            /* Handle Heartbeat */
            trn_handle_heartbeat(net_hdr, subnet_handle, &heartbeat_param);
        }
        break;

        default:
            break;
        }
    }

    udata = pdata + trn_hdr_len;
    udatalen = pdatalen - trn_hdr_len;
    break;

    /** Access Packet */
    case MS_TRN_ACCESS_PKT:
    {
        UCHAR akf, aid;
        UCHAR   nonce_type;
        /* Encrypt the Access PDU using Device Key */
        trans_mic_length = 0x04 << szmic;
        /* msb must be 0 */
        akf = pdata[0] >> 6;
        aid = pdata[0] & 0x3F;
        buffer.payload   = pdata + 1;
        buffer.length    = pdatalen - 1;
        TRN_TRC("[TRN] Rx Access Pkt. AKF:0x%02X, AID:0x%02X\n", akf, aid);

//            printf("[TRN] Rx Access Pkt. AKF:0x%02X, AID:0x%02X\r\n", akf, aid);

        /* If `akf` is 0, Device Key is used */
        if (0 == akf)
        {
            nonce_type = MS_NONCE_T_DEVICE;
        }
        else
        {
            nonce_type = MS_NONCE_T_APPLICATION;
        }

        /* Decrypt received frame */
        retval = trn_frame_decrypt_pdu
                 (
                     net_hdr->saddr,
                     net_hdr->daddr,
                     net_hdr->seq_num,
                     net_hdr->ivi,
                     &buffer,
                     aid,
                     nonce_type,
                     trans_mic_length,
                     tpdu,
                     &appkey_handle
                 );

        if (API_SUCCESS == retval)
        {
            udata = tpdu;
            udatalen = pdatalen - (1 + trans_mic_length);
        }
        else
        {
            TRN_ERR("[TRN] AES Decrypt Failed. Result: 0x%04X. Returning.\n", retval);
        }
    }
    break;

    default:
        result = API_FAILURE;
        break;
    }

    /* Unlock */
    TRN_UNLOCK();

    /* Strip headers if any and pass it to the registered layer above */
    if ((MS_TRN_CTRL_PKT == tcf) && (NULL != trn_ctrl_callback) && (API_SUCCESS == retval))
    {
        /**
            TODO: Check this call. Commented during LPN testing as udata was
            uninitialized when it got here
        */
        trn_ctrl_callback(net_hdr, subnet_handle, appkey_handle, udata, udatalen);
    }
    else if ((MS_TRN_ACCESS_PKT == tcf) && (NULL != trn_access_callback) && (API_SUCCESS == retval))
    {
        trn_access_callback(net_hdr, subnet_handle, appkey_handle, udata, udatalen);
    }

    return result;
}


/* Utility Functions */
API_RESULT  trn_frame_decrypt_pdu
(
    /* IN */  MS_NET_ADDR          saddr,
    /* IN */  MS_NET_ADDR          daddr,
    /* IN */  UINT32               seq_num,
    /* IN */  UINT8                ivi,
    /* IN */  MS_BUFFER*           sec_pdu,
    /* IN */  UCHAR                aid,
    /* IN */  UCHAR                key_type,
    /* IN */  UCHAR                mic_len,
    /* OUT */ UCHAR*               pdu,
    /* OUT */ MS_APPKEY_HANDLE*    appkey_handle
)
{
    UCHAR nonce[13];
    UCHAR marker;
    UCHAR mac[8];
    INT32 ret;
    API_RESULT retval;
    UINT32 iv_index;
    UCHAR* key;
    UCHAR   app_key[16];
    UINT8   dev_key_index;
    MS_APPKEY_HANDLE   key_handle;
    marker = 0;
    /* Key Type: App or Device */
    nonce[marker ++] = key_type;
    /* ASZMIC || Pad */
    nonce[marker++] = ((mic_len >> 3) << 7);
    /* 3 Octet Sequence Number */
    MS_PACK_BE_3_BYTE_VAL(&nonce[marker], seq_num);
    marker += 3;
    /* 2 Octet Source Addr */
    MS_PACK_BE_2_BYTE_VAL(&nonce[marker], saddr);
    marker += 2;
    /* 2 Octet Destination Addr */
    MS_PACK_BE_2_BYTE_VAL(&nonce[marker], daddr);
    marker += 2;
    /**
        Get 4 Octet IV Index.
        Based on the IVI, the IV Index used by the sender can be found.
        This is useful during the IV Index Update procedure.
    */
    MS_access_cm_get_iv_index_by_ivi(ivi, &iv_index);
    MS_PACK_BE_4_BYTE_VAL(&nonce[marker], iv_index);
    marker += 4;
    TRN_INF("[TRN] App/Dev Nonce\n");
    TRN_debug_dump_bytes(nonce, sizeof(nonce));
    dev_key_index = 0;
    key_handle = MS_INVALID_APPKEY_HANDLE;
    /* Try to decrypt received frame using all available keys */
    /* TODO: Handle reception from virtual address (MIC Calculation) */
    MS_LOOP_FOREVER()
    {
        /* Device Key */
        if (MS_NONCE_T_DEVICE == key_type)
        {
            /* Try with all the device keys, till it fails */
            key_handle = dev_key_index + MS_CONFIG_LIMITS(MS_MAX_APPS);
            retval = MS_access_cm_get_device_key(dev_key_index, &key);
            dev_key_index++;
        }
        /* AppKey */
        else
        {
            /**
                Check if AID is known.
                Get corresponding AppKeys.
            */
            retval = MS_access_cm_lookup_aid
                     (
                         aid,
                         &key_handle,
                         app_key
                     );
            key = app_key;
        }

        if (API_SUCCESS != retval)
        {
            TRN_ERR("[TRN] No more Keys available. Decrypt Failed. Returning\n");
            return API_FAILURE;
        }

        TRN_INF("[TRN] Encryption Key");
        TRN_debug_dump_bytes(key, 16);
        TRN_INF("[TRN] Cipher Text");
        TRN_debug_dump_bytes(sec_pdu->payload, sec_pdu->length);
        EM_mem_copy(mac, sec_pdu->payload + sec_pdu->length - mic_len, mic_len);

        /* Transport Payload Length Validation against the MIC Length */
        if (sec_pdu->length <= mic_len)
        {
            TRN_ERR("[TRN] Invalid Transport Packet. Payload Len is %d, MIC Length is %d!\n",
                    sec_pdu->length, mic_len);
            return API_FAILURE;
        }

        cry_aes_128_ccm_decrypt_be
        (
            key,
            nonce, sizeof(nonce),
            sec_pdu->payload, (sec_pdu->length - mic_len),
            NULL, 0,
            pdu,
            mac,
            mic_len,
            ret
        );

        /* Check if decryption is successful */
        if (0 > ret)
        {
            TRN_ERR("[TRN] AES Decrypt Failed. Result:%d. Search next match.\n",
                    ret);

            /* Try searching from next AppKey Handle */
            if (MS_NONCE_T_DEVICE != key_type)
            {
                key_handle ++;
            }
        }
        else
        {
            TRN_INF("[TRN] Plain Text");
            TRN_debug_dump_bytes(pdu, sec_pdu->length - mic_len);
            TRN_INF("[TRN] MAC");
            TRN_debug_dump_bytes(mac, mic_len);
            retval = API_SUCCESS;
            *appkey_handle = key_handle;
            break;
        }
    }
    return retval;
}


/* Handle Heartbeat */
void trn_handle_heartbeat
(
    /* IN */ MS_NET_HEADER*              net_hdr,
    /* IN */ MS_SUBNET_HANDLE            subnet_handle,
    /* IN */ MS_TRN_HEARTBEAT_PARAM*     param
)
{
    UINT8 hops;
//    MS_IGNORE_UNUSED_PARAM(subnet_handle);
    TRN_TRC("[TRN] HeartBeat Received\n");
//    printf("[TRN] HeartBeat Received\n");
    TRN_TRC("[TRN] InitTTL:0x%02X, Features:0x%04X\n",
            param->init_ttl, param->features);
//    printf("[TRN] InitTTL:0x%02X, Features:0x%04X\n",
//        param->init_ttl, param->features);

    /* Increment Heartbeat CountLog */
    if (0x00 != heartbeat_sub.period_log)
    {
        if ((heartbeat_sub.saddr == net_hdr->saddr) && (heartbeat_sub.daddr == net_hdr->daddr))
        {
            if(trn_heartbeat_rcv_callback != NULL)
            {
                trn_heartbeat_rcv_callback(net_hdr->saddr,subnet_handle,heartbeat_sub.count_log);
                UINT32 timeout;
                timeout = (1 << (heartbeat_sub.period_log - 1));

                if(EM_TIMER_HANDLE_INIT_VAL != heartbeat_sub.timer_handle)
                {
                    trn_stop_heartbeat_sub_timer();
                }

                trn_start_heartbeat_sub_timer(timeout);
            }

            if (0xFFFF > heartbeat_sub.count)
            {
                UINT16 count;
                heartbeat_sub.count += 1;
                count = heartbeat_sub.count;

                /* Check if 'count' is 2^n and increment count_log */
                if (0x0000 == (count & (count - 1)))
                {
                    heartbeat_sub.count_log++;
                }
            }

            /* hops = InitTTL - RxTTL + 1 */
            hops = param->init_ttl - net_hdr->ttl + 1;

            if (hops < heartbeat_sub.min_hops)
            {
                heartbeat_sub.min_hops = hops;
            }

            if (hops > heartbeat_sub.max_hops)
            {
                heartbeat_sub.max_hops = hops;
            }
        }
    }
}

#endif /* MS_TRN */

