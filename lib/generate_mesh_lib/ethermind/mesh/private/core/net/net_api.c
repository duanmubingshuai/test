/**
    \file net_api.c

    This file defines the NETWORK Layer Application Interface Methods
*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "net_internal.h"
#include "net_extern.h"

#include "cry.h"
#include "sec_tbx.h"

#include "MS_trn_api.h"
#include "MS_access_api.h"

/* For Persistent Storage */
#include "access_internal.h"
#include "access_extern.h"


#include "ltrn_internal.h"

#include "osal_cbTimer.h"
#include "MS_config_api.h"
#ifdef MS_NET

extern API_RESULT blebrr_queue_depth_check(void);
#define MS_IV_UPDATE_ROLE_LOCAL     0x80
#define MS_IV_UPDATE_ROLE_REMOTE    0x00

#define MS_IV_UPDATE_STATE_READY    0x00
#define MS_IV_UPDATE_STATE_IN_PROC  0x20
#define MS_IV_UPDATE_STATE_DONE     0x40
#define MS_IV_UPDATE_STATE          (MS_IV_UPDATE_STATE_DONE|MS_IV_UPDATE_STATE_IN_PROC)

#define MS_IV_UPDATE_ACTIVE         0x01
#define MS_IV_UPDATE_NORMAL         0x00

EM_timer_handle ms_iv_update_timer_handle;
#define MS_IV_UPDATE_TIMEOUT        96*60*60
//#define MS_IV_UPDATE_TIMEOUT        8*60

#define MS_IV_UPDATE_STEP           8*60*60

#define MS_IV_UPDATE_LOCAL_COUNT        (MS_IV_UPDATE_TIMEOUT/MS_IV_UPDATE_STEP)
#define MS_IV_UPDATE_REMOTE_DONE_COUNT  (MS_IV_UPDATE_LOCAL_COUNT>>1)
#define MS_IV_UPDATE_REMOTE_PRO_COUNT   (MS_IV_UPDATE_LOCAL_COUNT+MS_IV_UPDATE_REMOTE_DONE_COUNT)


#define MS_KEY_REFRESH_CONFIG_TIME              10
#define MS_KEY_REFRESH_METHOD_BEACON_TIME       120

#define KEY_REFRESH_METHOD_BEACON               MS_FALSE






/* --------------------------------------------- Global Definitions */
EM_timer_handle ms_snb_timer_handle;
EM_timer_handle net_key_refresh_timer_handle;
UINT8 seq_num_init_flag;
UINT32 g_iv_update_index;
UINT8 g_iv_update_state;
UINT8 g_iv_update_start_timer;
UINT8   MS_key_refresh_active;




/* --------------------------------------------- Static Global Variables */
/** Network Tx Queue related global data structures */
static NET_TX_Q_ELEMENT net_tx_queue[NET_TX_QUEUE_SIZE];
static UINT16 net_tx_queue_start;
static UINT16 net_tx_queue_end;
static UINT8    net_key_refresh_distribution_newkey[16];

/* Transmission Timer */
static EM_timer_handle net_tx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;


/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */

/**
    \brief Register Interface with NETWORK Layer

    \par Description
    This routine registers interface with the NETWORK Layer.
    NETWORK Layer supports only one upper layer, hence this routine shall be called once.

    \param [in] net_cb
           Upper Layer Notification Callback

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_net_register
(
    /* IN */ NET_NTF_CB    net_cb
)
{
    /* Register the callback */
    net_callback = net_cb;
    return API_SUCCESS;
}


/**
    \brief API to send Secure Network Beacon

    \par Description
    This routine sends Secure Network Beacon for the
    given subnet handle

    \param [in] subnet_handle
           Subnet handle of the network to be broadcasted.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_net_broadcast_secure_beacon
(
    /* IN */ MS_SUBNET_HANDLE    subnet_handle
)
{
    API_RESULT retval;
    UCHAR pdu[MS_BCON_TYPE_SIZE + 1 + 8 + 4 + 8];
    UCHAR marker;
    INT32 ret;
    UINT32 ivindex;
    UCHAR bcon_key[16];
    UINT8 flags, kr_phase, iv_phase;
    NET_TRC
    ("[NET] Sending Secure Network Beacon\n");
    /* Initialize Marker */
    marker = 0;
    /* Beacon Type */
    pdu[marker++] = BRR_BCON_TYPE_SECURE_NET;
    /* Flags */
    /* IV Index and Update Flag */
    MS_access_cm_get_iv_index(&ivindex, &iv_phase);
    iv_phase &= 0x01;
    /* Key Refresh Flag */
    MS_access_cm_get_key_refresh_phase(subnet_handle, &kr_phase);
    flags = (MS_ACCESS_KEY_REFRESH_PHASE_2 == kr_phase) ? 0x01 : 0x00;
    flags |= (iv_phase << 1);
    pdu[marker++] = flags;
//    printf(
//    "[CM] IV Index 0x%08X. Flag:0x%02X\n",
//    ivindex, flags);
    /* Network ID */
    MS_access_cm_get_subnet_network_id(subnet_handle, &pdu[marker]);
    marker += 8;
    /* IV Index */
    ivindex += iv_phase;
    MS_PACK_BE_4_BYTE_VAL(&pdu[marker], ivindex);
    marker += 4;
    /* Get the beacon key */
    MS_access_cm_get_subnet_beacon_key(subnet_handle, bcon_key);
    /* Authenticate the Beacon */
    cry_aes_128_cmac_sign_be
    (
        (pdu + 1),
        (sizeof(pdu) - 8 - 1),
        bcon_key,
        (pdu + ((sizeof(pdu) - 8))),
        8,
        ret
    );
    marker += 8;
    /* Send the beacon over bearer */
    retval = MS_brr_broadcast_beacon
             (
                 BRR_BCON_TYPE_SECURE_NET,
                 pdu,
                 marker
             );
    return retval;
}


/**
    \brief API to send NETWORK PDUs

    \par Description
    This routine sends NETWORK PDUs to peer device.

    \param [in] hdr
           Network Header

    \param [in] buffer
           Lower Transport Payload

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_net_send_pdu
(
    /* IN */ MS_NET_HEADER*     hdr,
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ MS_BUFFER*         buffer,
    /* IN */ UINT8              is_seg
)
{
    MS_NET_HEADER hdr_ex;
    API_RESULT retval;
    UINT32 iv_index;
    /* Copy */
    hdr_ex = *hdr;
    /* Fill required fields */
    MS_access_cm_get_iv_index(&iv_index, NULL);
    hdr_ex.ivi = (UCHAR)(iv_index & 0x00000001);
    retval = net_pkt_send (&hdr_ex, subnet_handle, buffer, MS_FALSE,is_seg);
    /* Increment Sequence Number */
    return retval;
}

/**
    \par Description
    This function sends data over a network interface.

    \param [in] hdr
           Network Header for the transmit packet
    \param [in] subnet_handle
           Handle identifying associated subnet on which the packet to be transmitted
    \param [in] buffer
           Outgoing Data Packet
    \param [in] is_relay
           Flag
           \ref MS_TRUE : If the packet to be tagged as relay
           \ref MS_FALSE: Otherwise
*/
API_RESULT net_pkt_send
(
    /* IN */ MS_NET_HEADER*     hdr,
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ MS_BUFFER*         buffer,
    /* IN */ UINT8              is_relay,
    /* IN */ UINT8              is_seg
)
{
    API_RESULT retval;
    MS_BUFFER net_buffer;
    UCHAR pdu[NET_HDR_SIZE + NET_MAX_PAYLOAD_SIZE];
    /* Secured PDU */
    UCHAR sec_pdu[NET_HDR_SIZE + NET_MAX_PAYLOAD_SIZE + NET_MAX_MIC_SIZE];
    UINT16 marker;
    UCHAR mac[8];
    INT32 ret;
    UCHAR nonce[13];
    UCHAR pecb_input[16];
    UCHAR pecb_output[16];
    UCHAR index;
    UCHAR net_mic_size;
    UINT8 nid;
    UCHAR priv_key[16];
    UCHAR enc_key[16];
    UINT32 iv_index;
    MS_NET_ADDR l_saddr;
    NETIF_PKT_T type;
    /* Parameter Validation */
    NET_NULL_CHECK(hdr);
    NET_NULL_CHECK(buffer);
    /* Payload Length Check */
    NET_RANGE_CHECK_END(buffer->length, NET_MAX_PAYLOAD_SIZE);
    /**
        If the CTL field is set to 0, the NetMIC is a 32 - bit value and the Lower Transport PDU contains an Access Message.
        If the CTL field is set to 1, the NetMIC is a 64 - bit value and the Lower Transport PDU contains a Control Message.
    */
    net_mic_size = 4 << (hdr->ctl);
    NET_TRC("[NET Tx] Pkt. SRC:0x%04X, DST:0x%04X, IVI:0x%02X, SEQ:0x%08X, CTL:0x%02X, TTL:0x%02X, Len: %d, is_relay:0x%02X\n",
            hdr->saddr, hdr->daddr, hdr->ivi, hdr->seq_num, hdr->ctl, hdr->ttl, buffer->length, is_relay);
//    printf("[NET_Tx] Pkt. SRC:0x%04X, DST:0x%04X, IVI:0x%02X, SEQ:0x%08X, CTL:0x%02X, TTL:0x%02X, Len: %d, is_relay:0x%02X\r\n",
//    hdr->saddr, hdr->daddr, hdr->ivi, hdr->seq_num, hdr->ctl, hdr->ttl, buffer->length, is_relay);
    NET_debug_dump_bytes(buffer->payload, buffer->length);
    /**
        In the network header, partial information will be received from the upper layer.
        Remaining will be filled by the network layer.
        IVI, NID, SEQ Num, SRC Addr will be local information from network layer.
    */
    /* Initialize marker */
    marker = 0;
    /* Get Network Key related information */
    retval = MS_access_cm_get_subnet_nid(subnet_handle, &nid);

    if (API_SUCCESS != retval)
    {
        /* Invalid Subnet Handle. Returning */
        NET_ERR("[NET Tx] Invalid Subnet Handle:0x%04X. Returning Status 0x%04X\n",
                subnet_handle, retval);
        printf("[NET Tx] Invalid Subnet Handle:0x%04X. Returning Status 0x%04X\n",
               subnet_handle, retval);
        return retval;
    }

    MS_access_cm_get_subnet_privacy_key(subnet_handle, priv_key);
    MS_access_cm_get_subnet_encryption_key(subnet_handle, enc_key);
    NET_TRC("[NET Tx] NID: 0x%02X\n", nid);
    /* Pack Network Header */
    pdu[marker++] = ((((UCHAR)(hdr->ivi & 0x00000001)) << 7) | (nid & 0x7F));
    pdu[marker++] = ((hdr->ctl << 7) | (hdr->ttl & 0x7F));
    MS_PACK_BE_3_BYTE_VAL(&pdu[marker], hdr->seq_num);
    marker += 3;
    MS_PACK_BE_2_BYTE_VAL(&pdu[marker], hdr->saddr);
    marker += 2;
    MS_PACK_BE_2_BYTE_VAL(&pdu[marker], hdr->daddr);
    marker += 2;
    /* Copy Payload */
    EM_mem_copy
    (
        &pdu[marker],
        buffer->payload,
        buffer->length
    );
    marker += buffer->length;
    /**
        Create Network MIC - 32 bit or 64 bit
        - Access Message: CTL Field (0), NetMIC Size 4 octets
        - Control Message: CTL Field (1), NetMIC Size 8 octets
    */
    /**
        First Octet of Network PDU will remain plaintext.

        Next 4 fields from Network Header (CTL, TTL, SEQ, SRC) will be obfuscated.
        Remaining fields (destination address and Transport PDU) will be encrypted.
    */
    /* Create Secure PDU */
    sec_pdu[0] = pdu[0];
    /* Get the local source address */
    MS_access_cm_get_primary_unicast_address(&l_saddr);
    /**
        Next perform Encryption, as encrypted content will be used for obfuscation.

        encDSTencDST || encTransportPDU, NetMIC =
                AES-CCM(EncryptionKey, Network Nonce, DST || TransportPDU)
    */
    /** Create Nonce */
    /* CTL, TTL, SEQ and SRC in the same order as in Network Header */
    EM_mem_copy(&nonce[1], &pdu[1], 6);

    /**
        Rules during Nonce creation.
          - If Destination address is Unassigned Address and Source address
            is the Primary element's Unicast Address, then 'Proxy Nonce' is to
            be used. Else, 'Network Nonce' to be used.
    */
    if ((l_saddr == hdr->saddr) && (MS_NET_ADDR_UNASSIGNED == hdr->daddr))
    {
        nonce[0] = MS_NONCE_T_PROXY;
        nonce[1] = 0x00;
        /* Assign Proxy Configuration Packet Type */
        type     = NETIF_PKT_T_PROXY;
    }
    else
    {
        nonce[0] = MS_NONCE_T_NETWORK;

        /* Assign Packet Type based on if it is tagged as Relay */
        if (is_relay)
        {
            type     = is_relay;

            //ZQY
            if(API_FAILURE==blebrr_queue_depth_check())
                return API_SUCCESS;
        }
        else
        {
            type     = NETIF_PKT_T_NETWORK;
        }
    }

    /* Pad */
    nonce[7] = nonce[8] = 0x00;
    /* IV Index */
    MS_access_cm_get_iv_index(&iv_index, NULL);
    MS_PACK_BE_4_BYTE_VAL(&nonce[9], iv_index);
    cry_aes_128_ccm_encrypt_be
    (
        enc_key,
        nonce, sizeof(nonce),
        (pdu + 7), (marker - 7),
        NULL, 0,
        (sec_pdu + 7),
        mac, net_mic_size,
        ret
    );
    NET_INF("[NET Tx] Encryption Key");
    NET_debug_dump_bytes(enc_key, sizeof(enc_key));
    NET_INF("[NET Tx] Nonce");
    NET_debug_dump_bytes(nonce, sizeof(nonce));
    NET_INF("[NET Tx] Plain Text");
    NET_debug_dump_bytes((pdu + 7), (marker - 7));
    NET_INF("[NET Tx] Cipher Text");
    NET_debug_dump_bytes((sec_pdu + 7), (marker - 7));
    NET_INF("[NET Tx] MAC");
    NET_debug_dump_bytes(mac, net_mic_size);
    /* Fill in MAC based on CTL */
    EM_mem_copy(&sec_pdu[marker], mac, net_mic_size);
    /* Perform Obfuscation. */
    net_create_pecb_input
    (
        iv_index,
        sec_pdu,
        pecb_input
    );
    net_obfuscate
    (
        pdu,
        priv_key,
        pecb_input,
        pecb_output
    );

    /* XOR */
    for (index = 0; index < 6; index++)
    {
        sec_pdu[index + 1] = pecb_output[index];
    }

    net_buffer.payload = sec_pdu;
    net_buffer.length = marker + net_mic_size;
    /* Lock */
    NET_LOCK();
    retval = net_tx_enqueue
             (
                 type,
                 hdr->daddr,
                 sec_pdu,
                 (marker + net_mic_size),
                 is_seg
             );
    /* Unlock */
    NET_UNLOCK();
    return retval;
}

/**
    \brief To get address type.

    \par Description
    This routine is to get address type for a given address.

    \param [in] addr            Input Network Address

    \return One of the following address type
            \ref MS_NET_ADDR_TYPE_INVALID
            \ref MS_NET_ADDR_TYPE_UNICAST
            \ref MS_NET_ADDR_TYPE_VIRTUAL
            \ref MS_NET_ADDR_TYPE_GROUP
*/
MS_NET_ADDR_TYPE MS_net_get_address_type
(
    /* IN */ MS_NET_ADDR addr
)
{
    MS_NET_ADDR_TYPE addr_type;
    NET_INF("[NET] Get ADDR Type of Network Address 0x%04X\n", addr);

    /* Check if Unassigned Address */
    if (MS_NET_ADDR_UNASSIGNED == addr)
    {
        addr_type = MS_NET_ADDR_TYPE_INVALID;
    }
    else
    {
        /* Get 2 most significant bits */
        addr_type = (UCHAR)(addr >> 14);

        /* For Unicast Address the 2 most significants bits could be b'00' or b'01' */
        if (addr_type < 2)
        {
            addr_type = MS_NET_ADDR_TYPE_UNICAST;
        }
    }

    NET_INF("[NET] Returning Address Type 0x%02X for Network Address 0x%04X\n", addr_type, addr);
    return addr_type;
}

/* Network Tx Queue Interfaces */
void net_tx_queue_init(void)
{
    UINT32 index;
    /* Initialize NET Tx SAR Queue Indices */
    net_tx_queue_start = 0;
    net_tx_queue_end = 0;

    /* Initialize NET Tx Queue */
    for (index = 0; index < NET_TX_QUEUE_SIZE; index++)
    {
        net_tx_queue[index].allocated_data = NULL;
        net_tx_queue[index].data_length = 0;
        net_tx_queue[index].tx_count = 0;
        net_tx_queue[index].tx_interval = 0;
    }

    EM_stop_timer(&net_tx_timer_handle);
}

API_RESULT net_tx_enqueue
(
    /* IN */ NETIF_PKT_T               type,
    /* IN */ MS_NET_ADDR               d_addr,
    /* IN */ UINT8*                    buffer,
    /* IN */ UINT16                    buffer_len,
    /* IN */ UINT8                     is_seg
)
{
    API_RESULT            retval;
    UCHAR*                packet_ptr;
    NET_TX_Q_ELEMENT*     tx;
    NET_INF("[NET Tx] NET Q -> Limit: %d, Start: %d, End:%d\n",
            NET_TX_QUEUE_SIZE, net_tx_queue_start, net_tx_queue_end);

    if ((0 == buffer_len) || (NULL == buffer) || (NETIF_PKT_T_PROXY < type))
    {
        NET_ERR("[NET Tx] Invalid Input Parameters\n");
        return NET_INVALID_PARAMETER_VALUE;
    }

    /* Initialize */
    retval = API_SUCCESS;
    /* Get Last Tx Element */
    tx = &net_tx_queue[net_tx_queue_end];

    /* Is Queue FULL ?? */
    if (0 != tx->data_length)
    {
        /* TODO: Add new set of error codes for NET */
        NET_ERR("[NET] NET Tx Q FULL. Returning 0x%04X\n", retval);

        if(net_tx_timer_handle == EM_TIMER_HANDLE_INIT_VAL)
        {
            retval = net_trigger_tx();
        }

        retval = NET_TX_QUEUE_FULL;
        //printf("[NET] NET Tx Q FULL. Returning 0x%04X\n", retval);
    }
    else
    {
        /* Caller allocated memory for the data */
        tx->allocated_data = (UCHAR*) net_alloc_mem (buffer_len);

        if (NULL == tx->allocated_data)
        {
            NET_ERR("[NET Tx] FAILED to Allocate Memory for NET Tx Packet\n");
            retval = NET_MEMORY_ALLOCATION_FAILED;
        }
        else
        {
            UINT8      trigger_tx;
            UINT8      tx_state;
            packet_ptr = tx->allocated_data;
            EM_mem_copy
            (
                packet_ptr,
                buffer,
                buffer_len
            );
            /* Mark Tx Element as allocated */
            tx->data_length = buffer_len;
            /* Save the NETIF Packet Type */
            tx->type        = type;
            /* Save the Destination Address */
            tx->d_addr      = d_addr;
            /* Save other information */
            /**
                As Proxy Configuration messages are sent over reliable GATT bearer,
                setting Tx Count as 1.
            */
//            if (NETIF_PKT_T_PROXY == type)
//            {
//                tx_state = 0;
//            }
//            else
            {
                /** Network and Relay will use their respective same transmit counts and intervals. */
                MS_access_cm_get_transmit_state((type & NETIF_PKT_T_RELAY_MASK), &tx_state);
            }

            if(is_seg == MS_FALSE)
            {
                tx->unsegment = MS_TRUE;
                tx->tx_flag = MS_TRUE;
            }

            tx->tx_count = (UCHAR)((tx_state & 0x07) + 1);

            if(tx->unsegment && (tx->tx_count<3))
            {
                tx->tx_count = 3;
            }

            tx->tx_interval = (UCHAR)((tx_state >> 3) + 1);
            NET_INF("[NET Tx] NET Pkt Type:0x%02X, Tx Count:0x%02X, Tx Interval:0x%02X\n",
                    type, tx->tx_count, tx->tx_interval);

//            printf("[NET Tx] NET Pkt Type:0x%02X, Tx Count:0x%02X, Tx Interval:0x%02X\n",
//            type, tx->tx_count, tx->tx_interval);
            if (EM_TIMER_HANDLE_INIT_VAL == net_tx_timer_handle)
            {
                trigger_tx = MS_TRUE;
            }
            else
            {
                trigger_tx = MS_FALSE;
            }

            /* Update Tx Queue End */
            net_tx_queue_end ++;

            if (NET_TX_QUEUE_SIZE == net_tx_queue_end)
            {
                net_tx_queue_end = 0;
            }

            /* Trigger Tx Transmission */
            if (MS_TRUE == trigger_tx)
            {
                NET_INF("[NET Tx] Triggering\n");
                retval = net_trigger_tx();
            }
        }
    }

    return retval;
}

API_RESULT net_trigger_tx(void)
{
    API_RESULT retval;
    NET_TX_Q_ELEMENT*     tx;
    NET_INF("[NET Tx] NET Trigger Tx -> Limit: %d, Start: %d, End:%d\n",
            NET_TX_QUEUE_SIZE, net_tx_queue_start, net_tx_queue_end);
    /* Initialize */
    retval = API_SUCCESS;
    /* Get First Tx Element */
    tx = &net_tx_queue[net_tx_queue_start];

    /* Is a valid Tx Element queued ?? */
    if (0 == tx->data_length)
    {
        retval = NET_TX_QUEUE_EMPTY;
//      printf("[net_trigger_tx]: Tx Queue Empty\r\n");
    }
    else
    {
        NET_TRC("[NET Tx] Tx initiated of Packet Length 0x%04X. Tx Count 0x%02X\n",
                tx->data_length, tx->tx_count);
//        printf("[NET Tx] Tx initiated of Packet Length 0x%04X. Tx Count 0x%02X\n",
//        tx->data_length, tx->tx_count);
        retval = netif_send
                 (
                     tx->type,
                     tx->d_addr,
                     tx->allocated_data,
                     tx->data_length,
                     tx->unsegment,
                     tx->tx_flag
                 );

        if (API_SUCCESS != retval)
        {
            NET_ERR("[NET Tx] Failed to send Tx packet. Result: 0x%04X\n", retval);
            printf("[NET Tx] Failed to send Tx packet. Result: 0x%04X\r\n", retval);
        }
        else
        {
            if(tx->tx_flag)
            {
                tx->tx_flag = MS_FALSE;
            }

            /* TODO: Assert (count != 0) */
            tx->tx_count--;
            NET_TRC("[NET Tx] Sent Tx packet. Decrementing Tx Count: 0x%02X\n",
                    tx->tx_count);
        }

        //20190529 by ZQ
        UINT32 netTxTimeOut;

        if(tx->tx_interval*10 <= (NET_TX_TIMEOUT&0xFFFF) )
            netTxTimeOut = NET_TX_TIMEOUT;
        else
            netTxTimeOut = ((10*(tx->tx_interval)) | EM_TIMEOUT_MILLISEC);

        /* Start Timer */
        retval = EM_start_timer
                 (
                     &net_tx_timer_handle,
                     netTxTimeOut,/*NET_TX_TIMEOUT,*/
                     net_tx_timeout_handler,
                     NULL,
                     0
                 );
        /* TODO: Check the start timer return value */
    }

    return retval;
}

void net_tx_timeout_handler (void* args, UINT16 size)
{
    NET_TX_Q_ELEMENT*     tx;
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    NET_LOCK_VOID();
    NET_TRC("[NET Tx] Timeout for Tx Q Element 0x%04X\n",
            net_tx_queue_start);
    /* Reset Timer Handler */
    net_tx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Get First Tx Element */
    tx = &net_tx_queue[net_tx_queue_start];

    /* Check if Tx Element Valid */
    if (0 != tx->data_length)
    {
        /* Check if the max retry count is reached */
        if (0 == tx->tx_count)
        {
            NET_TRC("[NET Tx] Max RTX done. Freeing TX Q Element 0x%04X\n",
                    net_tx_queue_start);
            /* Free Q element and Memory */
            net_free_mem(tx->allocated_data);
            tx->allocated_data = NULL;
            tx->data_length = 0;
            tx->tx_interval = 0;
            tx->unsegment = MS_FALSE;
            tx->tx_flag = MS_FALSE;
            /* Update Tx Queue Start Index */
            net_tx_queue_start ++;

            if (NET_TX_QUEUE_SIZE == net_tx_queue_start)
            {
                net_tx_queue_start = 0;
            }
        }

        #ifdef NET_DEBUG
        else
        {
            NET_INF("[NET Tx] Tx Queue [0x%04X] Count 0x%02X\n", net_tx_queue_start, tx->tx_count);
        }

        #endif /* NET_DEBUG */
    }

    /* Trigger transmission */
    net_trigger_tx();
    /* Unlock */
    NET_UNLOCK_VOID();
    return;
}

API_RESULT MS_net_init_seq_number(void)
{
    UINT8 sar_count;
    sar_count = ltrn_get_sar_ctx_count(LTRN_SAR_CTX_TX);

    if(sar_count == 0)
    {
//        printf("Init Seq Number\n");
        NET_INIT_SEQ_NUM_STATE();
        return API_SUCCESS;
    }
    else
    {
        seq_num_init_flag = TRUE;
        printf("SAR cache not empty\n");
        return API_FAILURE;
    }
}

static void ms_iv_update_state_change(void)
{
    UINT8 snb;
    UINT8 iv_update_flag;

    if(((ms_iv_index.iv_update_state & MS_IV_UPDATE_ROLE_LOCAL) == MS_IV_UPDATE_ROLE_LOCAL) &&
            ((ms_iv_index.iv_update_state & MS_IV_UPDATE_STATE) == MS_IV_UPDATE_STATE_IN_PROC))
    {
        iv_update_flag =
            MS_IV_UPDATE_ROLE_LOCAL |
            MS_IV_UPDATE_STATE_DONE |
            MS_IV_UPDATE_NORMAL;

        if(MS_net_init_seq_number() == API_SUCCESS)
        {
            MS_net_start_iv_update_timer(iv_update_flag,MS_FALSE);
            ms_iv_index.iv_update_state = iv_update_flag;
        }
        else
        {
            g_iv_update_index = ms_iv_index.iv_index;
            g_iv_update_state = iv_update_flag;
            g_iv_update_start_timer = MS_TRUE;
            return;
        }
    }
    else if((((ms_iv_index.iv_update_state & MS_IV_UPDATE_ROLE_LOCAL) == MS_IV_UPDATE_ROLE_REMOTE))&&
            ((ms_iv_index.iv_update_state & MS_IV_UPDATE_STATE) == MS_IV_UPDATE_STATE_IN_PROC))
    {
        //timeout,not recieve done beacon,update to normal mode,iv index--
        ms_iv_index.iv_update_state = 0;
        ms_iv_index.iv_index--;
//        MS_access_cm_set_iv_index(ms_iv_index.iv_index, ms_iv_index.iv_update_state);
    }
    else
    {
        if((ms_iv_index.iv_update_state & MS_IV_UPDATE_ROLE_LOCAL) == MS_IV_UPDATE_ROLE_LOCAL)
        {
            MS_access_cm_get_features_field(&snb, MS_FEATURE_SEC_NET_BEACON);

            if(snb != MS_TRUE)
            {
                MS_net_stop_snb_timer(0);
            }
        }

        ms_iv_index.iv_update_state = 0;
    }

    MS_access_cm_set_iv_index(ms_iv_index.iv_index, ms_iv_index.iv_update_state);
}


/* Secure Network Beacon Timeout Handler */
static void ms_iv_update_timeout_handler (void* args, UINT16 size)
{
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(size);
    MS_IGNORE_UNUSED_PARAM(args);
    ms_iv_update_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

    if(!ms_iv_index.iv_expire_time) //Timer expire
    {
        ms_iv_update_state_change();
        ms_access_ps_store(MS_PS_RECORD_SUBNETS);
    }
    else
    {
        ms_iv_index.iv_expire_time--;
        printf("Time Expire at 0x%02X\n",ms_iv_index.iv_expire_time);
        retval = EM_start_timer
                 (
                     &ms_iv_update_timer_handle,
                     (MS_IV_UPDATE_STEP),
                     ms_iv_update_timeout_handler,
                     NULL,
                     0
                 );
        ms_access_ps_store(MS_PS_RECORD_SUBNETS);

        if(retval)
        {
            ACCESS_ERR(
                "[CM] Start Timer Failed\n");
        }
    }
}


API_RESULT MS_net_start_iv_update_timer(UINT8 timer_flag,UINT8 reset_en)
{
    API_RESULT retval;

    if(reset_en == MS_FALSE)
    {
        if(timer_flag & MS_IV_UPDATE_ROLE_LOCAL)
        {
            ms_iv_index.iv_expire_time = MS_IV_UPDATE_LOCAL_COUNT - 1;
        }
        else
        {
            ms_iv_index.iv_expire_time = ((timer_flag & MS_IV_UPDATE_STATE)
                                          == MS_IV_UPDATE_STATE_IN_PROC) ?
                                         MS_IV_UPDATE_REMOTE_PRO_COUNT - 1:
                                         MS_IV_UPDATE_REMOTE_DONE_COUNT - 1;
        }
    }

    printf("iv_expire_time:0x%08X\n",ms_iv_index.iv_expire_time);

    if (EM_TIMER_HANDLE_INIT_VAL == ms_iv_update_timer_handle)
    {
        retval = EM_start_timer
                 (
                     &ms_iv_update_timer_handle,
                     (MS_IV_UPDATE_STEP),
                     ms_iv_update_timeout_handler,
                     NULL,
                     0
                 );

        if(retval)
        {
            ACCESS_ERR(
                "[CM] Start Timer Failed\n");
            printf(
                "[CM] Start Timer Failed\n");
            return retval;
        }
    }

    return retval;
}

/* Stop Timer */
void MS_net_stop_iv_update_timer(void)
{
    API_RESULT retval;

    if (EM_TIMER_HANDLE_INIT_VAL != ms_iv_update_timer_handle)
    {
        retval = EM_stop_timer(&ms_iv_update_timer_handle);

        if(retval)
        {
            NET_TRC(
                "[NET] Stop Timer Failed\n");
        }
    }
}

void MS_net_iv_update_rcv_pro
(
    /* IN */ UINT32    iv_index,
    /* IN */ UINT8     iv_update_flag
)
{
    API_RESULT retval;
    UINT8   should_start_timer;
    UINT8   iv_update_change_state_to,should_init_seq_number;
    should_start_timer = MS_FALSE;
    should_init_seq_number = MS_FALSE;

    if((iv_index<ms_iv_index.iv_index) || (iv_index >(ms_iv_index.iv_index + 42)) ||
            ((!(((ms_iv_index.iv_update_state & MS_IV_UPDATE_ROLE_LOCAL) == MS_IV_UPDATE_ROLE_REMOTE)&&
                ((ms_iv_index.iv_update_state & MS_IV_UPDATE_STATE) == MS_IV_UPDATE_STATE_IN_PROC) && (iv_update_flag==0)))&&
             (iv_index==ms_iv_index.iv_index)))
    {
        return;
    }

    if((ms_iv_index.iv_update_state & MS_IV_UPDATE_STATE) != MS_IV_UPDATE_STATE_READY)
    {
        MS_net_stop_iv_update_timer();
        ms_iv_index.iv_expire_time = 0;
    }

    if(((ms_iv_index.iv_update_state & MS_IV_UPDATE_ROLE_LOCAL) == MS_IV_UPDATE_ROLE_REMOTE)&&
            ((ms_iv_index.iv_update_state & MS_IV_UPDATE_STATE) == MS_IV_UPDATE_STATE_IN_PROC) && (iv_update_flag==0))
    {
        should_start_timer = MS_TRUE;
        should_init_seq_number = MS_TRUE;
        iv_update_change_state_to =
            MS_IV_UPDATE_ROLE_REMOTE |
            MS_IV_UPDATE_STATE_DONE |
            MS_IV_UPDATE_NORMAL;
    }
    else if(iv_update_flag == 1)
    {
        should_start_timer = MS_TRUE;

        if((ms_iv_index.iv_update_state & MS_IV_UPDATE_ROLE_LOCAL) == MS_IV_UPDATE_ROLE_LOCAL)
        {
            should_init_seq_number = MS_TRUE;
        }

        iv_update_change_state_to =
            MS_IV_UPDATE_ROLE_REMOTE |
            MS_IV_UPDATE_STATE_IN_PROC |
            MS_IV_UPDATE_ACTIVE;
    }
    else
    {
        should_init_seq_number = MS_TRUE;
        iv_update_change_state_to =
            MS_IV_UPDATE_ROLE_REMOTE |
            MS_IV_UPDATE_STATE_READY |
            MS_IV_UPDATE_NORMAL;
    }

    if((should_start_timer == MS_TRUE)&&
            (should_init_seq_number == MS_FALSE))
    {
        retval = MS_access_cm_set_iv_index
                 (
                     iv_index,
                     iv_update_change_state_to
                 );
        MS_net_start_iv_update_timer(iv_update_change_state_to,MS_FALSE);
    }
    else if((should_start_timer == MS_TRUE)&&
            (should_init_seq_number == MS_TRUE))
    {
        if(MS_net_init_seq_number()==API_SUCCESS)
        {
            retval = MS_access_cm_set_iv_index
                     (
                         iv_index,
                         iv_update_change_state_to
                     );
            MS_net_start_iv_update_timer(iv_update_change_state_to,MS_FALSE);
        }
        else
        {
            g_iv_update_index = iv_index;
            g_iv_update_state = iv_update_change_state_to;
            g_iv_update_start_timer = MS_TRUE;
        }
    }
    else if((should_start_timer == MS_FALSE)&&
            (should_init_seq_number == MS_TRUE))
    {
        if(MS_net_init_seq_number() == API_SUCCESS)
        {
            retval = MS_access_cm_set_iv_index
                     (
                         iv_index,
                         iv_update_change_state_to
                     );
        }
        else
        {
            g_iv_update_index = iv_index;
            g_iv_update_state = iv_update_change_state_to;
            g_iv_update_start_timer = MS_FALSE;
        }
    }
    else
    {
    }

//    if(should_start_timer == MS_TRUE)
//    {
//
//    }
//    if((iv_update_flag == 1) ||
//        (((ms_iv_index.iv_update_state & 0x80) == MS_IV_UPDATE_ROLE_REMOTE)&&
//        ((ms_iv_index.iv_update_state & 0x60) == MS_IV_UPDATE_STATE_IN_PROC) && (iv_update_flag==0)))
//
//    if(((ms_iv_index.iv_update_state & 0x80) == MS_IV_UPDATE_ROLE_REMOTE)&&
//        ((ms_iv_index.iv_update_state & 0x60) == MS_IV_UPDATE_STATE_IN_PROC) && (iv_update_flag==1))
//    {
//        MS_net_stop_iv_update_timer();
//        iv_update_flag =
//            MS_IV_UPDATE_ROLE_REMOTE |
//            MS_IV_UPDATE_STATE_DONE |
//            MS_IV_UPDATE_NORMAL;
//        if(MS_net_init_seq_number()==API_SUCCESS)
//        {
//            retval = MS_access_cm_set_iv_index
//                    (
//                        iv_index,
//                        iv_update_flag
//                    );
//            MS_net_start_iv_update_timer(iv_update_flag);
//        }
//    }
//    if(!(ms_iv_index.iv_update_state & 0x80)) //Not local event,check iv index,then set
//    {
//        switch (ms_iv_index.iv_update_state & 0x60)
//        {
//            case MS_IV_UPDATE_STATE_READY:
//            {
//                if(iv_update_flag)
//                {
//                    iv_update_flag =
//                        MS_IV_UPDATE_ROLE_REMOTE |
//                        MS_IV_UPDATE_STATE_IN_PROC |
//                        MS_IV_UPDATE_ACTIVE;
//                    retval = MS_access_cm_set_iv_index
//                     (
//                         iv_index,
//                         iv_update_flag
//                     );
//                    MS_net_start_iv_update_timer(iv_update_flag);
//                    /* Check if successful */
//                    if (API_SUCCESS != retval)
//                    {
//                        NET_TRC("[BEACON Rx] Set IV Update Failed - 0x%04X\n", retval);
//                    }
//                }
//                else if (((iv_index <= ms_iv_index.iv_index) || (iv_index >(ms_iv_index.iv_index + 42))))
//                {
//                   return ;
//                }
//            }
//            break;
//            case MS_IV_UPDATE_STATE_IN_PROC:
//            {
//                if(iv_update_flag)
//                {
//                    //should process?  by hq
//                }
//                else if(iv_index == ms_iv_index.iv_index)
//                {
//                    MS_net_stop_iv_update_timer();
//                    iv_update_flag =
//                        MS_IV_UPDATE_ROLE_REMOTE |
//                        MS_IV_UPDATE_STATE_DONE |
//                        MS_IV_UPDATE_NORMAL;
//                    if(MS_net_init_seq_number()==API_SUCCESS)
//                    {
//                        retval = MS_access_cm_set_iv_index
//                                (
//                                    iv_index,
//                                    iv_update_flag
//                                );
//                        MS_net_start_iv_update_timer(iv_update_flag);
//                    }
//
//                }
//            }
//            break;
//            case MS_IV_UPDATE_STATE_DONE:
//            {
//            }
//            break;
//            default:
//            break;
//        }
//        if(set_iv_update_bit)
//        {
//            NET_TRC("[BEACON Rx] Updating IV Index with CM\n");
//            printf("[BEACON Rx] Updating IV Index with CM\n");
//            retval = MS_access_cm_set_iv_index
//                     (
//                         iv_index,
//                         iv_update_flag
//                     );
//            MS_net_start_iv_update_timer(iv_update_flag);
    /* Check if successful */
//            if (API_SUCCESS != retval)
//            {
//                NET_TRC("[BEACON Rx] Set IV Update Failed - 0x%04X\n", retval);
//            }
//        }
//    }
}



void MS_net_iv_update_start(void)
{
    UINT32 ivindex;
    UINT8 iv_phase,iv_update_flag;
    UINT8      snb;
    MS_access_cm_get_iv_index(&ivindex, &iv_phase);
    iv_update_flag = 0;
//    printf("iv phase:0x%02X\n",iv_phase);
//    printf("iv index:0x%02X\n",ivindex);

    if(!(iv_phase & MS_IV_UPDATE_ACTIVE) &&
            ((iv_phase & MS_IV_UPDATE_STATE) == MS_IV_UPDATE_STATE_READY) &&
            ((iv_phase & MS_IV_UPDATE_ROLE_LOCAL) == MS_IV_UPDATE_ROLE_REMOTE))
    {
        printf("IV update\n");
        MS_access_cm_get_features_field(&snb, MS_FEATURE_SEC_NET_BEACON);
        iv_update_flag =
            MS_IV_UPDATE_ROLE_LOCAL |
            MS_IV_UPDATE_STATE_IN_PROC |
            MS_IV_UPDATE_ACTIVE;
        ivindex++;
        MS_access_cm_set_iv_index(ivindex, iv_update_flag);
        MS_net_start_iv_update_timer(iv_update_flag,MS_FALSE);

        if(MS_TRUE != snb)
        {
            MS_net_start_snb_timer(0);
        }
    }
}

/**
    \brief To allocate Sequence Number.

    \par Description This function is used to allocate
    Sequence Number.

    \param [out] seq_num   Location where SeqNum to be filled.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_net_alloc_seq_num(/* OUT */ UINT32*    seq_num)
{
    *seq_num = net_seq_number_state.seq_num;
    NET_TRC("Allocating SeqNum:0x%08X\n", net_seq_number_state.seq_num);

    /* Take care of initiating IV Update procedure */

    /* At regular space, save SeqNum in PS */
    if (net_seq_number_state.seq_num >= net_seq_number_state.block_seq_num_max)
    {
        net_seq_number_state.block_seq_num_max += MS_CONFIG_LIMITS(MS_NET_SEQ_NUMBER_BLOCK_SIZE);

        if(net_seq_number_state.block_seq_num_max >= 0xF00000)
        {
            MS_net_iv_update_start();
        }

        /* Save in persistent storage */
        ms_access_ps_store(MS_PS_RECORD_SEQ_NUMBER);
    }

    net_seq_number_state.seq_num++;
    return API_SUCCESS;
}

/**
    \brief To get current Sequence Number state.

    \par Description This function is used to get current
    Sequence Number state.

    \param [out] seq_num_state  Location where Seq Number state to be filled.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_net_get_seq_num_state(/* OUT */ NET_SEQ_NUMBER_STATE* seq_num_state)
{
    *seq_num_state = net_seq_number_state;
    return API_SUCCESS;
}

/**
    \brief To set current Sequence Number state.

    \par Description This function is used to set current
    Sequence Number state.

    \param [in] seq_num_state  Location from where Seq Number state to be taken.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_net_set_seq_num_state(/* IN */ NET_SEQ_NUMBER_STATE* seq_num_state)
{
    net_seq_number_state = *seq_num_state;
    return API_SUCCESS;
}

/* Secure Network Beacon Timeout Handler */
static void ms_snb_timeout_handler (void* args, UINT16 size)
{
    MS_SUBNET_HANDLE         subnet_handle;
    MS_IGNORE_UNUSED_PARAM(size);
    subnet_handle = (MS_SUBNET_HANDLE)(*((UINT32*)args));
    ms_snb_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    NET_TRC(
        "[NET] *** SNB Timeout\n");
    /* Send SNB */
    MS_net_broadcast_secure_beacon(subnet_handle);
    /* TODO: Check if the timer to be restarted */
    /* Restart Timer */
    MS_net_start_snb_timer(subnet_handle);
    return;
}


/* Start Secure Network Beacon Timer */
void MS_net_start_snb_timer
(
    /* IN */    MS_SUBNET_HANDLE         subnet_handle
)
{
    API_RESULT retval;

    if (EM_TIMER_HANDLE_INIT_VAL == ms_snb_timer_handle)
    {
        retval = EM_start_timer
                 (
                     &ms_snb_timer_handle,
                     (MS_SNB_TIMEOUT | EM_TIMEOUT_MILLISEC),
                     ms_snb_timeout_handler,
                     (void*)&subnet_handle,
                     sizeof(subnet_handle)
                 );

        if(retval)
        {
            NET_TRC(
                "[NET] Start Timer Failed\n");
        }
    }
}

/* Stop Timer */
void MS_net_stop_snb_timer
(
    /* IN */ MS_SUBNET_HANDLE         subnet_handle
)
{
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(subnet_handle);

    if (EM_TIMER_HANDLE_INIT_VAL != ms_snb_timer_handle)
    {
        retval = EM_stop_timer(&ms_snb_timer_handle);

        if(retval)
        {
            NET_TRC(
                "[NET] Stop Timer Failed\n");
        }
    }
}

void ms_net_netkey_update
(
    /* IN */ UINT8*                      new_key,
    /* IN */ UINT16                     count
)
{
    ACCESS_CONFIG_NETKEY_UPDATE_PARAM  param;
    EM_mem_copy(&param.netkey[0], new_key, 16);
    EM_mem_copy(&net_key_refresh_distribution_newkey[0], new_key, 16);
    param.netkey_index = 0;
    /* Set Local NetKey */
    MS_access_cm_add_update_netkey
    (
        0, /* netkey_index */
        MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE, /* opcode */
        &param.netkey[0] /* net_key */
    );
//    printf("Set update address:0x%04X\n",ms_key_refresh_whitelist[count]);
    MS_access_ps_store_disable(MS_TRUE);
    MS_config_client_set_publish_address(ms_key_refresh_whitelist[count]);
    MS_access_ps_store_disable(MS_FALSE);
    MS_config_client_netkey_update(&param);
}

void MS_net_key_refresh_phase_set(UINT8 trans,UINT8 enable)
{
    UINT16  ms_key_refresh_context;
    ACCESS_CONFIG_KEYREFRESH_PHASE_SET_PARAM  param;
    param.netkey_index = 0;
    param.transition = trans;
//    printf("phase start\n");
    /* Change Local State as well */
    MS_access_cm_set_key_refresh_phase
    (
        0, /* subnet_handle */
        &param.transition /* key_refresh_state */
    );

    if(enable == MS_TRUE)
    {
        ms_key_refresh_context = ms_key_refresh_count;
        MS_net_start_key_refresh_timer(ms_key_refresh_context,MS_KEY_REFRESH_METHOD_BEACON_TIME);
    }
    else
    {
        ms_key_refresh_context = 0;
        MS_access_ps_store_disable(MS_TRUE);
        MS_config_client_set_publish_address(ms_key_refresh_whitelist[ms_key_refresh_context]);
        MS_access_ps_store_disable(MS_FALSE);
        param.transition = trans;
        MS_config_client_keyrefresh_phase_set(&param);
        MS_net_start_key_refresh_timer(ms_key_refresh_context,MS_KEY_REFRESH_CONFIG_TIME);
    }
}

/* Key Refresh Timeout Handler */
static void ms_key_refresh_timeout_handler (void* args, UINT16 size)
{
    UINT16  ms_key_refresh_index;
    UINT8 key_refresh_state;
    ACCESS_CONFIG_NETKEY_UPDATE_PARAM  param_netkey_update;
    ACCESS_CONFIG_KEYREFRESH_PHASE_SET_PARAM  param_keyrefresh_phase_set;
    UINT16 index;
    MS_IGNORE_UNUSED_PARAM(size);
    ms_key_refresh_index = (UINT16)(*((UINT32*)args));
    index = ++ms_key_refresh_index;
    net_key_refresh_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    NET_TRC(
        "[NET] *** Key Refresh Timeout\n");
    printf(
        "[NET] *** Key Refresh Timeout\n");
    MS_access_cm_get_key_refresh_phase(0,&key_refresh_state);

    switch (key_refresh_state)
    {
    case MS_ACCESS_KEY_REFRESH_PHASE_1:
        if(index >= ms_key_refresh_count)
        {
            MS_net_key_refresh_phase_set(0x02,KEY_REFRESH_METHOD_BEACON);
            MS_key_refresh_active = MS_FALSE;
        }
        else
        {
            EM_mem_copy(&param_netkey_update.netkey[0], &net_key_refresh_distribution_newkey[0], 16);
            param_netkey_update.netkey_index = 0;
            MS_access_ps_store_disable(MS_TRUE);
            MS_config_client_set_publish_address(ms_key_refresh_whitelist[index]);
            MS_access_ps_store_disable(MS_FALSE);
            MS_config_client_netkey_update(&param_netkey_update);
            MS_net_start_key_refresh_timer(ms_key_refresh_index,MS_KEY_REFRESH_CONFIG_TIME);
        }

        break;

    case MS_ACCESS_KEY_REFRESH_PHASE_2:
        if(index >= ms_key_refresh_count)
        {
            MS_net_key_refresh_phase_set(0x03,KEY_REFRESH_METHOD_BEACON);
        }
        else
        {
            param_keyrefresh_phase_set.netkey_index =0;
            param_keyrefresh_phase_set.transition = key_refresh_state;
            MS_access_ps_store_disable(MS_TRUE);
            MS_config_client_set_publish_address(ms_key_refresh_whitelist[index]);
            MS_access_ps_store_disable(MS_FALSE);
            MS_config_client_keyrefresh_phase_set(&param_keyrefresh_phase_set);
            MS_net_start_key_refresh_timer(ms_key_refresh_index,MS_KEY_REFRESH_CONFIG_TIME);
        }

    case MS_ACCESS_KEY_REFRESH_PHASE_3:
        break;

    case MS_ACCESS_KEY_REFRESH_PHASE_NORMAL:
        if(index < ms_key_refresh_count)
        {
            param_keyrefresh_phase_set.netkey_index =0;
            param_keyrefresh_phase_set.transition = MS_ACCESS_KEY_REFRESH_PHASE_3;
            MS_access_ps_store_disable(MS_TRUE);
            MS_config_client_set_publish_address(ms_key_refresh_whitelist[index]);
            MS_access_ps_store_disable(MS_FALSE);
            MS_config_client_keyrefresh_phase_set(&param_keyrefresh_phase_set);
            MS_net_start_key_refresh_timer(ms_key_refresh_index,MS_KEY_REFRESH_CONFIG_TIME);
        }

        break;

    default:
        break;
    }
}


void MS_net_key_refresh
(
    /* IN */ MS_NET_ADDR*            key_refresh_whitelist,
    /* IN */ UINT16                 num,
    /* IN */ UINT8*                  new_net_key
)
{
    UINT16  ms_key_refresh_index;

    if(num == 0)
        return;

//    EM_mem_set(ms_key_refresh_whitelist, 0, sizeof(ms_key_refresh_whitelist));
    for(UINT16 i=0; i<num; i++)
        ms_key_refresh_whitelist[i] = key_refresh_whitelist[i];

    ms_key_refresh_count = num;
    ms_key_refresh_index = 0;
    MS_key_refresh_active = MS_TRUE;
    ms_net_netkey_update(new_net_key,ms_key_refresh_index);
    MS_net_start_key_refresh_timer(ms_key_refresh_index,MS_KEY_REFRESH_CONFIG_TIME);
}

/* Start Secure Network Beacon Timer */
void MS_net_start_key_refresh_timer
(
    /* IN */    UINT16      index,
    /* IN */    UINT32      time
)
{
    API_RESULT retval;

    if (EM_TIMER_HANDLE_INIT_VAL == net_key_refresh_timer_handle)
    {
        retval = EM_start_timer
                 (
                     &net_key_refresh_timer_handle,
                     time,
                     ms_key_refresh_timeout_handler,
                     (void*)&index,
                     sizeof(index)
                 );

        if(retval)
        {
            NET_TRC(
                "[NET] Start Timer Failed\n");
            printf(
                "[NET] Start Timer Failed\n");
        }
    }
}

/* Stop Key refresh Timer */
void MS_net_stop_key_refresh_timer
(
    void
)
{
    API_RESULT retval;

    if (EM_TIMER_HANDLE_INIT_VAL != net_key_refresh_timer_handle)
    {
        retval = EM_stop_timer(&net_key_refresh_timer_handle);

        if(retval)
        {
            NET_TRC(
                "[NET] Stop Timer Failed\n");
        }
    }
}

void MS_net_key_refresh_init(void)
{
    UINT8 key_refresh_state;
    UINT8 net_key[16];
    ACCESS_CONFIG_NETKEY_UPDATE_PARAM  param;

    if(ms_key_refresh_count == 0)
        return;

    MS_access_cm_get_key_refresh_phase(0,&key_refresh_state);

    switch (key_refresh_state)
    {
    case MS_ACCESS_KEY_REFRESH_PHASE_1:
        MS_access_cm_get_netkey_at_offset(0, 1, net_key);
        MS_net_key_refresh(ms_key_refresh_whitelist,ms_key_refresh_count,net_key);
        break;

    case MS_ACCESS_KEY_REFRESH_PHASE_2:
        MS_net_key_refresh_phase_set(MS_ACCESS_KEY_REFRESH_PHASE_2,KEY_REFRESH_METHOD_BEACON);
        MS_key_refresh_active = MS_FALSE;
        break;

    case MS_ACCESS_KEY_REFRESH_PHASE_3:
        MS_net_key_refresh_phase_set(MS_ACCESS_KEY_REFRESH_PHASE_3,KEY_REFRESH_METHOD_BEACON);
        break;

    case MS_ACCESS_KEY_REFRESH_PHASE_NORMAL:
        break;

    default:
        break;
    }
}


#endif /* MS_NET */

