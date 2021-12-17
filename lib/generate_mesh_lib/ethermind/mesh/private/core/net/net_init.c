
/**
    \file net_init.c

    Network Layer module initialization routine and tables defined in this file.
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "net_internal.h"
#include "MS_access_api.h"
#include "MS_trn_api.h"
#include "cry.h"
#include "net.h"
#include "ltrn_internal.h"

#ifdef MS_NET
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */
/* Mutex */
MS_DEFINE_MUTEX (net_mutex)

/* Callback Function */
NET_NTF_CB net_callback;

/** Device UUID - Used for unprovisioned device broadcast */
/* DECL_STATIC UCHAR ms_net_device_uuid[MS_NET_DEVICE_UUID_SIZE] = MS_NET_DEVICE_UUID; */

/**
    Network Sequence Number - 24 bit value.

    The sequence number, contained in the SEQ field of the Network PDU,
    is primarily used for protection against replay attacks.
    Each Element maintains its own sequence number. Having a different sequence
    number in each new Network PDU for every message source (identified by
    the unicast address contained in the SRC field) is critical for the security
    of the mesh network. With a 24 - bit sequence number, an Element can
    transmit 16, 777, 216 messages before repeating a nonce.
    If an Element transmits a message on average once every five seconds
    (representing a fairly high frequency message for known use cases),
    the Element can transmit for 2.6 years before the nonce repeats.
    Each Element shall use strictly increasing sequence numbers for the Network
    PDUs it generates. Before the sequence number approaches the maximum value
    (0xFFFFFF), the Element shall update the IV Index using the IV Update
    procedure.
    This is done to ensure that the sequence number will never wrap around.

    Network Sequence Number state also contains the maximum sequence number
    of the current block. If the device is powered cycled, it will resume
    transmission using the sequence number from start of next block.
*/
NET_SEQ_NUMBER_STATE net_seq_number_state;

/* Network Cache */
MS_DEFINE_GLOBAL_ARRAY(NET_CACHE_ELEMENT, net_cache, MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE));
UINT16 net_cache_start;
UINT16 net_cache_size;

extern UCHAR brr_packet_rssi;



/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */

/**
    \brief Initializes Module.

    \par Description Initializes Module tables.
*/
void ms_net_init (void)
{
    ms_internal_verificaiton_check();
    NET_TRC("[NET] Initializing NET..");
    /* Module Mutex Initialization */
    MS_MUTEX_INIT_VOID (net_mutex, NET);
    /* Initialize Sequence Number and IV Index */
    NET_INIT_SEQ_NUM_STATE();
    ms_snb_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    ms_iv_update_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    net_key_refresh_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Cache initialization */
    MS_INIT_GLOBAL_ARRAY(NET_CACHE_ELEMENT, net_cache, MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE), 0x00);
    net_init_cache();
    /* Initialize Network Tx Queue */
    net_tx_queue_init();
    #ifdef MS_PROXY_SUPPORT
    /* Initialize Network Proxy */
    net_proxy_init();
    #endif /* MS_PROXY_SUPPORT */
    /* Initialize Network Interface */
    netif_init();
    /* Register for the Secure Network beacon with the bearer */
    MS_brr_register_beacon_handler
    (
        MS_BCON_TYPE_SECURE,
        net_handle_secure_beacon
    );
}

#ifndef MS_NO_SHUTDOWN
/**
    \par Description:
    This function is the shutdown handler for NETWORK module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.
*/
void ms_net_shutdown (void)
{
    NET_TRC("[NET] Network Shutdown Successful.");
}
#endif /* MS_NO_SHUTDOWN */


/**
    \par Description
    This function handles the incoming data received over a network interface.

    \param [in] type
           Network Interface Packet Type
    \param [in] handle
           Network Interface Handle
    \param [in] pdata
           The incoming Data Packet
    \param [in] pdatalen
           Size of the incoming Data Packet
*/
void net_pkt_in
(
    /* IN */ NETIF_PKT_T       type,
    /* IN */ NETIF_HANDLE*     handle,
    /* IN */ UCHAR*            pdata,
    /* IN */ UINT16            pdatalen
)
{
    MS_NET_HEADER     hdr;
    MS_SUBNET_HANDLE  subnet_handle;
    UCHAR             trn_pdu[NET_MAX_PAYLOAD_SIZE];
    UINT16            trn_pdu_len;
    /* Source Address Types */
    UINT8             saddr_type;
    API_RESULT        retval;
    UINT8             is_relay;
    UINT8             rttl;
    NET_TRC("[NET Rx] Bearer Callback. NetIf Pkt Type: 0x%02X. Length: %d\n",
            type, pdatalen);
//    printf("[NET Rx] Bearer Callback. NetIf Pkt Type: 0x%02X. Length: %d\n",
//    type, pdatalen);
    NET_debug_dump_bytes(pdata, pdatalen);

    /**
         ## Network PDU Format ##

                         .----------------- Obfuscated -------------------------.
                        /                                                        \
          <-  Byte 0  -> <-  Byte 1 -> <-   Byte 2, 3, 4    -> <-   Byte 5, 6    ->
          7     6      0 7    6      0 23     15      7      0 15       7         0
          +-----+-------+-----+-------+-------+-------+-------+---------+---------+
          | IVI |  NID  | CTL |  TTL  | Sequence Number (SEQ) | Source Addr (SRC) |
          +-----+-------+-----+-------+-------+-------+-------+---------+---------+

          <-      Byte 7, 8      -> <-   Bytes N   ->  <-     4 or 8 Bytes       ->
          15          7           0                   63 or 31                    0
          +-----------+------------+.................+----------------------------+
          | Destination Addr (DST) |  Transport PDU  |    Network MIC (NetMIC)    |
          +-----------+------------+................ +----------------------------+
          \                                         /
           `---------- Encrypted ------------------'
    */

    /* Length Check */
    /* TODO: Add maximum packet length check */
    if ((NET_HDR_SIZE + NET_MIN_MIC_SIZE) > pdatalen)
    {
        NET_ERR("[NET Rx] Incomplete Network Packet Received of length %d. Dropping...\n",
                pdatalen);
        return;
    }

    /**
        Check if Network Packet decodes successfully with the Keys.
    */
    retval = net_decode_frame
             (
                 type,
                 pdata,
                 pdatalen,
                 &subnet_handle,
                 &hdr,
                 trn_pdu,
                 &trn_pdu_len
             );

    if (API_SUCCESS != retval)
    {
        NET_ERR("[NET Rx] Network Pkt Decode Failed. Dropping...\n");
        return;
    }

    rttl = hdr.ttl;
    /* Network Packet Decrypted Successfully */
    NET_TRC("[NET Rx] Pkt. SRC:0x%04X, DST:0x%04X, IVI:0x%02X, SEQ:0x%08X, CTL:0x%02X, TTL:0x%02X, NID:0x%02X\n",
            hdr.saddr, hdr.daddr, hdr.ivi, hdr.seq_num, hdr.ctl, hdr.ttl, hdr.nid);
    //printf("[NET Rx]net_pdata_len = 0x%04X, trn_pdu_len = 0x%04X\r\n", pdatalen, trn_pdu_len);
    NET_debug_dump_bytes(trn_pdu, trn_pdu_len);
    /* Validate Source and Destination Address */
    saddr_type = MS_net_get_address_type(hdr.saddr);
    NET_INF("[NET Rx] SRC 0x%04X Type: 0x%02X\n", hdr.saddr, saddr_type);

    /* Source address shall be a Unicast Address. */
    if (MS_NET_ADDR_TYPE_UNICAST != saddr_type)
    {
        NET_ERR("[NET Rx] SRC ADDR 0x%04X not a Unicast Address. Dropping...\n",
                hdr.saddr);
        return;
    }

    /* LOCK */
    /* Set the packet to be relayed (default) */
    is_relay = MS_TRUE;
    #ifdef MS_PROXY_SUPPORT

    /* Check if Incoming Netif Handle corresponds to GATT */
    if (NULL != handle)
    {
        /* TODO: Assert (0x00 != (*handle)) */
        /* Set packet as to be proxied and not relayed */
        is_relay = MS_FALSE;
        NET_TRC("[NET Rx] Handle:0x%02x. Check with Proxy if Pkt from SRC:0x%04X to process\n",
                *handle, hdr.saddr);
        #ifdef MS_PROXY_SERVER
        /* First packet processing in Proxy Layer */
        net_proxy_process_first_pkt
        (
            handle,
            hdr.saddr
        );
        #endif /* MS_PROXY_SERVER */
    }

    /** TODO: Is it possible to receive NETIF_PKT_T_PROXY on Netif Handle 0x00?? */
    #endif /* MS_PROXY_SUPPORT */

    /**
        If Destination Address is a Unassigned Address,
        check if it is a proxy configuration message
    */
    if (MS_NET_ADDR_UNASSIGNED == hdr.daddr)
    {
        #ifdef MS_PROXY_SUPPORT

        /**
            Proxy configuration messages will have
            - CTL field set to 1
            - TTL field set to 0
            - DST field set to unassigned address
        */
        if ((NETIF_PKT_T_PROXY == type) && (MS_TRN_CTRL_PKT == hdr.ctl) && (0x00 == hdr.ttl))
        {
            NET_TRC("[NET Rx] Proxy Configuration Message with INVALID DST Addr 0x%04X\n",
                    hdr.daddr);
            net_proxy_recv(&hdr, handle, subnet_handle, trn_pdu, trn_pdu_len);
        }
        else
        #endif /* MS_PROXY_SUPPORT */
        {
            NET_ERR("[NET Rx] Invalid DST ADDR 0x%04X. Dropping ...\n",
                    hdr.daddr);
        }

        return;
    }

    /* Add to Cache */
    net_add_to_cache(&hdr);
    /** Forward to Upper Layer */
    NET_TRC("[NET Rx] Forward Packet to Upper Layer.\n");
    UINT32 net_replay_check;
    retval = ltrn_pkt_first_process(&hdr,&net_replay_check);      //by hq

    /* Lower Transport Layer will check and indicate if Relay/Proxy is required */

    /**
        If the message delivered from the advertising bearer is processed
        by the lower transport layer, the Relay feature is supported and enabled,
        the TTL field has a value of 2 or greater, and the destination is
        not a unicast address of an element on this node, then the TTL field value
        shall be decremented by 1, the Network PDU shall be tagged as relay,
        and the Network PDU shall be retransmitted to all network interfaces
        connected to the advertising bearer.
        It is recommended that a small random delay is introduced between receiving
        a Network PDU and relaying a Network PDU to avoid collisions between
        multiple relays that have received the Network PDU at the same time.
    */
    if (NET_POST_PROCESS_RX_PKT == retval)
    {
        if (hdr.ttl > 1)
        {
            UINT8      relay, proxy,proxy_state;
            /**
                Mesh Specification, Section 3.4.6.3.
                If the message delivered from the GATT bearer is processed by
                the lower transport layer, and the Proxy feature is supported
                and enabled, and the TTL field has a value of 2 or greater,
                and the destination is not a unicast address of an element on
                this node, then the TTL field value shall be decremented by 1,
                and the Network PDU shall be retransmitted to all network interfaces.

                If the message delivered from the advertising bearer is processed by
                the lower transport layer, and the Proxy feature is supported and
                enabled, and the TTL field has a value of 2 or greater, and
                the destination is not a unicast address of an element on this node,
                then the TTL field shall be decremented by 1 and the Network PDU shall
                be retransmitted to all network interfaces connected to the GATT bearer.
            */
            MS_access_cm_get_features_field(&relay, MS_FEATURE_RELAY);
            MS_access_cm_get_features_field(&proxy, MS_FEATURE_PROXY);
//            printf("[NET Rx]Rssi Value is 0x%02X.\n",brr_packet_rssi);
            INT8 rssi = (INT8) brr_packet_rssi;
            MS_proxy_fetch_state(&proxy_state);

            if (((MS_TRUE == relay) || ((MS_TRUE == proxy)&&(proxy_state == MS_PROXY_CONNECTED))) /*&&
                (((!rssi) || (rssi<-20)) || (proxy_state == MS_PROXY_CONNECTED))*/)
            {
                MS_BUFFER buffer;
                MS_SUBNET_HANDLE master_subnet_handle;

                if(handle)
                    is_relay = NETIF_PKT_T_PROXY;
                else
                    is_relay = NETIF_PKT_T_RELAY;

                if(hdr.ttl != 0x7f)
                {
                    hdr.ttl --;
                }

                buffer.payload   = trn_pdu;
                buffer.length    = trn_pdu_len;
                NET_TRC("[NET Rx] Relay/Proxy-ing Pkt.\n");

                /**
                    Check if using friendship security credentials.
                    Find the corresponding Master security credential.
                */
                if (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) <= subnet_handle)
                {
                    MS_access_cm_find_master_subnet(subnet_handle, &master_subnet_handle);
                }
                else
                {
                    master_subnet_handle = subnet_handle;
                }

                retval = net_pkt_send (&hdr, master_subnet_handle, &buffer, is_relay,MS_FALSE);
            }
        }
    }

    if ((NULL != net_callback)&&(net_replay_check == API_FAILURE))
    {
//        printf("[NET_Rx] Pkt. SRC:0x%04X, DST:0x%04X, IVI:0x%02X, SEQ:0x%08X, CTL:0x%02X, TTL:0x%02X, NID:0x%02X\r\n",
//        hdr.saddr, hdr.daddr, hdr.ivi, hdr.seq_num, hdr.ctl, rttl, hdr.nid);
        hdr.ttl = rttl;
        retval = net_callback(&hdr, subnet_handle, trn_pdu, trn_pdu_len);
    }

    /* Unlock */
}

/**
    Routine to decode a received Network PDU,
    based on a specific network Key from the store.
    This function returns status of the decode operation.
    If success, return the decoded network header and PDU.
*/
API_RESULT net_decode_frame
(
    /* IN */  NETIF_PKT_T           type,
    /* IN */  UCHAR*                pdata,
    /* IN */  UINT16                pdatalen,
    /* OUT */ MS_SUBNET_HANDLE*     subnet_handle,
    /* OUT */ MS_NET_HEADER*        net_hdr,
    /* OUT */ UCHAR*                trn_pdu,
    /* OUT */ UINT16*               trn_pdu_len
)
{
    UCHAR pecb_input[16];
    UCHAR pecb_output[16];
    UINT8 privacy_key[16];
    UINT8 encrypt_key[16];
    UCHAR mac[8];
    /**
        In the Network PDU, Destination Address (DST) and TransportPDU is encrypted.
        DST is of 2 octets and TransportPDU can be maximum of 16 octets.
    */
    /*
        TODO: This value is used instead of the above because of the third-party AES engine
        causing 32 bytes of update to the pdu during decryption. This needs to
        be analyzed and removed when using a proprietary AES engine
    */
    UCHAR pdu[32];
    UINT32 seq_no;
    UCHAR nonce[13];
    INT32 ret;
    UCHAR mic_len;
    UINT32 iv_index;
    API_RESULT retval;
    /**
         ## Network PDU - Obfuscated part ##

                         .----------------- Obfuscated -------------------------.
                        /                                                        \
          <-  Byte 0  -> <-  Byte 1 -> <-   Byte 2, 3, 4    -> <-   Byte 5, 6    ->
          7     6      0 7    6      0 23     15      7      0 15       7         0
          +-----+-------+-----+-------+-------+-------+-------+---------+---------+
          | IVI |  NID  | CTL |  TTL  | Sequence Number (SEQ) | Source Addr (SRC) |
          +-----+-------+-----+-------+-------+-------+-------+---------+---------+
    */
    /* Extract NID and IVI */
    /* TODO: Write Utility Macros */
    net_hdr->ivi = pdata[0] >> 7;
    net_hdr->nid = pdata[0] & 0x7F;
    NET_INF("[NET Rx] IVI: 0x%02X, NID 0x%02X\n", net_hdr->ivi, net_hdr->nid);
    /**
        Get IV Index.
        Based on the IVI, the IV Index used by the sender can be found.
        This is useful during the IV Index Update procedure.
    */
    MS_access_cm_get_iv_index_by_ivi(net_hdr->ivi, &iv_index);
    /* First perform de-obfuscation */
    net_create_pecb_input
    (
        iv_index,
        pdata,
        pecb_input
    );
    /**
        Based on the NID, first lookup master security material.
        Then lookup friendship security material.

        Return once the decryption is successful or when no more
        security material is available.
    */
    /* Set Subnet Handle with Invalid initialization value */
    *subnet_handle = MS_INVALID_SUBNET_HANDLE;
    /* Lookup NID for all possible match */
    retval = API_FAILURE;
    MS_LOOP_FOREVER()
    {
        /**
            Check if NID is known.
            Get corresponding Privacy and Encrypt Keys.
        */
        retval = MS_access_cm_lookup_nid
                 (
                     net_hdr->nid,
                     subnet_handle,
                     privacy_key,
                     encrypt_key
                 );

        /* If NID is not known, discard the packet */
        if (API_SUCCESS != retval)
        {
            NET_ERR("[NET Rx] NID match failed. Dropping...\n");
            break;
        }

        /* Perform de-obfuscation */
        net_de_obfuscate
        (
            pdata,
            privacy_key,
            pecb_input,
            pecb_output
        );
        /* Extract Fields */
        net_hdr->ctl = pecb_output[0] >> 7;
        net_hdr->ttl = pecb_output[0] & 0x7F;
        NET_INF("[NET Rx] CTL:0x%02X, TTL:0x%02X\n", net_hdr->ctl, net_hdr->ttl);
        /* Sequence Number */
        MS_UNPACK_BE_3_BYTE(&seq_no, &pecb_output[1]);
        MS_UNPACK_BE_2_BYTE(&net_hdr->saddr, &pecb_output[4]);
        /* Save in Network Header data structure */
        net_hdr->seq_num = seq_no;
        NET_INF("[NET Rx] SEQ:0x%08X, SRC:0x%04X\n",
                net_hdr->seq_num, net_hdr->saddr);

        /* Check if already in Cache? */
        /* Pass only the Network Header and Sequence Number */
        if (API_SUCCESS == net_is_in_cache(net_hdr))
        {
            NET_ERR("[NET Rx] Present in Cache. Dropping...\n");
            retval = API_FAILURE;
            break;
        }

        /* Create Network Nonce */
        /* CTL, TTL, SEQ and SRC in the same order as in Network Header */
        EM_mem_copy(&nonce[1], &pecb_output[0], 6);

        if (NETIF_PKT_T_NETWORK == type)
        {
            nonce[0] = MS_NONCE_T_NETWORK;
        }
        else
        {
            nonce[0] = MS_NONCE_T_PROXY;
            /**
                For Proxy frame CTL and TTL fields will be Zero.
                Explicitly setting associated octet in Nonce.
            */
            nonce[1] = 0x00;
        }

        /* Pad */
        nonce[7] = nonce[8] = 0x00;
        /* IV Index */
        MS_PACK_BE_4_BYTE_VAL(&nonce[9], iv_index);
        mic_len = 4 << net_hdr->ctl;
        EM_mem_copy(mac, pdata + pdatalen - mic_len, mic_len);
        cry_aes_128_ccm_decrypt_be
        (
            encrypt_key,
            nonce, sizeof(nonce),
            (pdata + 7), (pdatalen - (7 + mic_len)),
            NULL, 0,
            (pdu),
            mac, mic_len,
            ret
        );
        NET_INF("[NET Rx] Encryption Key");
        NET_debug_dump_bytes(encrypt_key, 16);
        NET_INF("[NET Rx] Nonce");
        NET_debug_dump_bytes(nonce, sizeof(nonce));
        NET_INF("[NET Rx] Cipher Text");
        NET_debug_dump_bytes((pdata + 7), (pdatalen - (7 + mic_len)));
        NET_INF("[NET Rx] Plain Text");
        NET_debug_dump_bytes(pdu, (pdatalen - (7 + mic_len)));
        NET_INF("[NET Rx] MAC");
        NET_debug_dump_bytes(mac, mic_len);

        /* Check if decryption is successful */
        if (0 <= ret)
        {
            /* Fill the Destination Address */
            MS_UNPACK_BE_2_BYTE(&net_hdr->daddr, &pdu[0]);
            NET_INF("[NET Rx] DST:0x%04X\n", net_hdr->daddr);
            NET_INF("[NET Rx] TransportPDU\n");
            NET_debug_dump_bytes(pdu + 2, (pdatalen - (9 + mic_len)));
            /* Fill out parameters */
            *trn_pdu_len = (pdatalen - (9 + mic_len));
            EM_mem_copy(trn_pdu, pdu + 2, (*trn_pdu_len));
            NET_INF("[NET Rx] Decryption Successful. Returning\n");
            retval = API_SUCCESS;
            break;
        }
        else
        {
            NET_INF("[NET Rx] Decryption Failed. Search for next NID match\n");
            (*subnet_handle)++;
        }
    }
    return retval;
}

/**
    \par Description
    This function obfuscates or de-obfuscates a network header.

    \param [in] network_pkt
           Network Packet
    \param [in] privacy_key
           Privacy Key
    \param [in] pecb_input
           PECB Input
    \param [out] pecb_output
           PECB Output. Can not be same pointer as PECB Input.
*/
void net_obfuscate
(
    /* IN */  UINT8*   network_pkt,
    /* IN */  UINT8*   privacy_key,
    /* IN */  UINT8*   pecb_input,
    /* OUT */ UINT8*   pecb_output
)
{
    INT32 ret;
    UINT32 index;
    /**
        PECB = e(PrivacyKey, 0x0000000000 || IV Index || Privacy Random)

        (CTL || TTL || SEQ || SRC) = ObfuscatedData  XOR PECB[0 - 5]
             OR
        ObfuscatedData = (CTL || TTL || SEQ || SRC)  XOR PECB[0 - 5]
    */
    /* AES 128 bit encryption */
    cry_aes_128_encrypt_be
    (
        pecb_input,
        privacy_key,
        pecb_output,
        ret
    );
    NET_INF("[NET] Privacy Key\n");
    NET_debug_dump_bytes(privacy_key, 16);
    NET_INF("[NET] PECB Input\n");
    NET_debug_dump_bytes(pecb_input, 16);

    /* XOR  and extract fields */
    /* Reusing 'pecb_output' buffer */
    for (index = 0; index < 6; index ++)
    {
        pecb_output[index] = network_pkt[index + 1] ^ pecb_output[index];
    }

    NET_INF("[NET] PECB Output\n");
    NET_debug_dump_bytes(pecb_output, 16);
    /* Return */
    return;
}

/**
    \par Description
    This function creates input bytes to be used for PECB.

    \param [in] iv_index
           Network IV Index
    \param [in] network_pkt
           Network Packet
    \param [out] pecb_input
           Buffer where PECB input bytes to be prepared
*/
void net_create_pecb_input
(
    /* IN */   UINT32   iv_index,
    /* IN */   UINT8*   network_pkt,
    /* OUT */  UINT8*   pecb_input
)
{
    /**
        PECB Input = 0x0000000000 || IV Index || Privacy Random

        Privacy Random = (encDST || encTransportPDU || NetMIC )[0-6]
    */
    EM_mem_set(&pecb_input[0], 0x0, 5);
    MS_PACK_BE_4_BYTE_VAL(&pecb_input[5], iv_index);
    EM_mem_copy(&pecb_input[9], &network_pkt[7], 7);
}


/* Secure Network Beacon Handler */
void net_handle_secure_beacon(UCHAR* pdata, UINT16 pdatalen)
{
    UCHAR bcon_key[16];
    INT32 ret;
    UINT32 iv_index;
    UCHAR* net_id, *auth_val;
    MS_SUBNET_HANDLE subnet_handle;
    API_RESULT retval;
    UINT8 flags, kriv_state, is_new_key;
    NET_TRC("[BEACON] Received Secure Beacon\n");
    /**
        Secure Network Beacon will be received in the following format:

        -----------------------------------------------------------------------------
        | Flags |        Network ID        |    IV Index    |        AuthVal        |
        -----------------------------------------------------------------------------
           1                8                    4                     8
    */
    /* Extract the above parameters */
    flags = *pdata;
    net_id = (pdata + 1);
    MS_UNPACK_BE_4_BYTE(&iv_index, (pdata + 9));
    auth_val = (pdata + 13);
    NET_TRC("[BEACON Rx] Flags: 0x%02X, IV Index: 0x%08X\n", flags, iv_index);
    NET_INF("[BEACON Rx] Network ID:\n");
    NET_debug_dump_bytes(net_id, 8);
    NET_INF("[BEACON Rx] AuthVal:\n");
    NET_debug_dump_bytes(auth_val, 8);
    /* Set Subnet Handle with Invalid initialization value */
    subnet_handle = MS_INVALID_SUBNET_HANDLE;
    /* Lookup Network ID for all possible match */
    retval = API_FAILURE;
    MS_LOOP_FOREVER()
    {
        /**
            Check if Network ID is known.
            Get corresponding Beacon Keys.
        */
        retval = MS_access_cm_lookup_network_id
                 (
                     net_id,
                     &subnet_handle,
                     bcon_key,
                     &is_new_key
                 );

        /* If Network ID is not known, discard the packet */
        if (API_SUCCESS != retval)
        {
//            NET_ERR("[BEACON Rx] Failed to lookup Network ID for Beacon Key\n");
//            printf("[BEACON Rx] Failed to lookup Network ID for Beacon Key\n");
            break;
        }

        /* Authenticate the Beacon */
        cry_aes_128_cmac_verify_be
        (
            pdata,
            (pdatalen - 8),
            bcon_key,
            auth_val,
            8,
            ret
        );

        if (0 <= ret)
        {
            /*
                TODO: Add interface to check if the subnet handle is for
                     primary network.
                     And this check should be only when the received
                     IV Index > Known IV Index of primary subnet.
            */
            if (0 == subnet_handle)
            {
                /* Handle IV Update and Key Refresh as indicated by the flags */
                /* Set Key Refresh Phase as per the flag */
                kriv_state = (flags & 0x01) ? MS_ACCESS_KEY_REFRESH_PHASE_2 : MS_ACCESS_KEY_REFRESH_PHASE_3;

                /* If SNB is received with New Key, update Key Refresh State */
                if (MS_TRUE == is_new_key)
                {
                    NET_TRC("[BEACON Rx] Updating Key Refresh with CM\n");
                    retval = MS_access_cm_set_key_refresh_phase
                             (
                                 subnet_handle,
                                 &kriv_state
                             );

                    /* Check if successful */
                    if (API_SUCCESS != retval)
                    {
                        NET_ERR("[BEACON Rx] Set Key Refresh Failed - 0x%04X\n", retval);
                        break;
                    }
                }

                /* Set IV Update as per the flag */
                kriv_state = (flags & 0x02) ? MS_TRUE : MS_FALSE;
                MS_net_iv_update_rcv_pro(iv_index,kriv_state);
                #ifdef MS_FRIEND_SUPPORT
                /* Register security update to LPNs if any */
                MS_trn_lpn_register_security_update(subnet_handle, flags, iv_index);
                #endif /* MS_FRIEND_SUPPORT */
                /* Send response beacon */
                /* MS_net_broadcast_secure_beacon(subnet_handle); */
            }

            break;
        }
        else
        {
            NET_INF("[BEACON Rx] Decryption Failed. Search for next Network ID match\n");
            subnet_handle++;
        }
    }
}

#endif /* MS_NET */

