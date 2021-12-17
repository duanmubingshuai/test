/**
    \file brr_api.c

    This file defines the Bearer Layer Application Interface Methods
*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "brr_internal.h"
#include "brr_extern.h"

#include "cry.h"
#include "sec_tbx.h"

#ifdef MS_BRR

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */

/**
    \brief Register Interface with Bearer Layer

    \par Description
    This routine registers interface with the Bearer Layer.
    Bearer Layer supports single Application, hence this routine shall be called once.

    \param brr_type
           Bearer Type

    \param brr_cb
           Details describing Application Notification Callback

    \return API_SUCCESS or an error code indicating reason for failure

*/
API_RESULT MS_brr_register
(
    /* IN */ BRR_TYPE        brr_type,
    /* IN */ BRR_NTF_CB      brr_cb
)
{
    /* Validate the parameter */
    if ((BRR_COUNT <= brr_type) ||
            (NULL == brr_cb))
    {
        BRR_ERR ("Invalid Bearer Type/callback parameter\n");
        return BRR_INVALID_PARAMETER_VALUE;
    }

    /* Lock */
    BRR_LOCK();
    /* Register the callback */
    brr_bearer_cb[brr_type] = brr_cb;
    /* Unlock */
    BRR_UNLOCK();
    return API_SUCCESS;
}


API_RESULT MS_brr_register_beacon_handler
(
    /* IN */ UCHAR        bcon_type,
    /* IN */ void (*bcon_handler) (UCHAR* data, UINT16 datalen)
)
{
    /* Validate the parameter */
    if ((NULL == bcon_handler) ||
            (MS_BCON_TYPE_SECURE < bcon_type))
    {
        BRR_ERR ("Invalid Bearer Type/callback parameter\n");
        return BRR_INVALID_PARAMETER_VALUE;
    }

    /* Lock */
    BRR_LOCK();
    /* Register the callback */
    brr_beacon_cb[bcon_type] = bcon_handler;
    /* Unlock */
    BRR_UNLOCK();
    return API_SUCCESS;
}


/**
    \brief API to register a bearer

    \par Description
    This routine registers a new bearer interface with the Mesh stack

    \param brr_info
           Identifies the Bearer interface information

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_add_bearer
(
    /* IN */  BRR_TYPE            brr_type,
    /* IN */  BRR_BEARER_INFO*    brr_info,
    /* OUT */ BRR_HANDLE*         brr_handle
)
{
    BRR_BEARER_DATA* bearer;
    API_RESULT retval;
    BRR_HANDLE handle;
    UCHAR i;
    UCHAR* pdata;
    UINT16 plen;

    /* Validate the parameters */
    if (NULL == brr_info)
    {
        BRR_ERR("Bearer info parameter cannot be NULL\n");
        return API_FAILURE;
    }

    /* Lock */
    BRR_LOCK();
    /* Update the bearer receive interface based on the Bearer type */
    brr_info->bearer_recv = brr_read_data_ind;
    handle = BRR_HANDLE_INVALID;
    pdata = NULL;
    plen = 0;
    bearer = NULL;

    /* Get the bearer type */
    switch (brr_type)
    {
    case BRR_TYPE_ADV:

        /* Check the ADV Network bearer entity availability */
        if (NULL == brr_bearer[0].info.bearer_send)
        {
            bearer = &brr_bearer[0];
            handle = 0;
        }

        break;

    case BRR_TYPE_GATT:

        /* Check for a GATT Network bearer entity availability */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES); i ++)
        {
            if (NULL == brr_bearer[i].info.bearer_send)
            {
                bearer = &brr_bearer[i];
                handle = i;
                /* Store the OutMTU */
                bearer->omtu = ((BRR_BEARER_CH_INFO*)(brr_info->binfo->payload))->mtu;
                /* Store the Role */
                bearer->role = ((BRR_BEARER_CH_INFO*)(brr_info->binfo->payload))->role;
                break;
            }
        }

        break;

    default:
        break;
    }

    /* Is bearer allocated? */
    if (NULL == bearer)
    {
        BRR_ERR ("Failed to allocate bearer\n");
        retval = API_FAILURE;
    }
    else
    {
        /* Save the bearer information */
        bearer->info = *brr_info;

        /* Reference the data if any */
        if (NULL != bearer->info.binfo)
        {
            pdata = bearer->info.binfo->payload;
            plen = bearer->info.binfo->length;
        }

        /* Unlock */
        BRR_UNLOCK();
        /* Callback the interface to notify interface up */
        retval = brr_bearer_cb[brr_type]
                 (
                     &handle,
                     BRR_IFACE_UP,
                     pdata,
                     plen
                 );
        retval = brr_bearer_cb[brr_type + 1]
                 (
                     &handle,
                     BRR_IFACE_UP,
                     pdata,
                     plen
                 );
        /* Lock */
        BRR_LOCK();
        *brr_handle = handle;
    }

    /* Unlock */
    BRR_UNLOCK();
    return retval;
}

/**
    \brief API to unregister a bearer

    \par Description
    This routine unregisters an already registered bearer interface from the Mesh stack

    \param brr_type
           Identifies the type of Bearer
    \param brr_handle
           Handle assigned to the bearer during registration

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_remove_bearer
(
    /* IN */ BRR_TYPE            brr_type,
    /* IN */ BRR_HANDLE*         brr_handle
)
{
    API_RESULT retval;

    /* Validate the parameters */
    if (NULL == brr_handle)
    {
        BRR_ERR("Bearer handle parameter cannot be NULL\n");
        return API_FAILURE;
    }

    /* Check if bearer handle is in range and registered */
    if (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) <= (*brr_handle))
    {
        BRR_ERR("Bearer handle out of range. Dropping...\n");
        return BRR_INVALID_PARAMETER_VALUE;
    }

    /* Check if Valid Bearer Handle is being removed for given Bearer Type */
    if (((BRR_TYPE_GATT == brr_type) && (0 == (*brr_handle))) ||
            ((BRR_TYPE_ADV == brr_type) && (0 != (*brr_handle))))
    {
        BRR_ERR("Mismatch in Provided Bearer Handle and Bearer Type. Dropping...\n");
        return BRR_PARAMETER_OUTSIDE_RANGE;
    }

    /* Lock */
    BRR_LOCK();
    BRR_TRC(
        ">> MS_brr_remove_bearer received for Bearer Type 0x%02X with Handle 0x%02X\n",
        brr_type, *brr_handle);

    if (NULL == brr_bearer[*brr_handle].info.bearer_send)
    {
        BRR_ERR("Data received on unregistered bearer. Dropping...\n");
        /* Unlock */
        BRR_UNLOCK();
        return BRR_INVALID_PARAMETER_VALUE;
    }

    /* Reset the Bearer reference */
    EM_mem_set(&brr_bearer[*brr_handle], 0x00, sizeof(BRR_BEARER_DATA));
    /* Unlock */
    BRR_UNLOCK();
    /* Callback the interface to notify interface down */
    retval = brr_bearer_cb[brr_type] (brr_handle, BRR_IFACE_DOWN, NULL, 0);
    retval = brr_bearer_cb[brr_type + 1](brr_handle, BRR_IFACE_DOWN, NULL, 0);
    BRR_TRC(
        "<< MS_brr_remove_bearer returned with 0x%04X\n",
        retval);
    /* Call the bearer init interface */
    return retval;
}

/**
    \brief API to scan enable/disable Device Beacons of given type

    \par Description
    This routine scans or stops scanning for Device Beacons of given type.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_observe_beacon
(
    /* IN */ UCHAR    bcon_type,
    /* IN */ UCHAR    enable
)
{
    BRR_BEACON_INFO info;

    /* Check if bearer is installed */
    if (NULL == brr_bearer[0].info.bearer_send)
    {
        BRR_ERR("Beacon bearer not up for operation.\n");
        return BRR_INTERFACE_NOT_READY;
    }

    /* Pack the beacon information */
    info.action = ((enable << 4) | BRR_OBSERVE);
    info.type = bcon_type;
    info.bcon_data = NULL;
    info.bcon_datalen = 0;
    info.uri = NULL;
    return brr_bearer[0].info.bearer_send(&bcon_brr_handle, MESH_AD_TYPE_BCON, &info, sizeof(BRR_BEACON_INFO));
}

/**
    \brief API to disable broadcast of given beacon type

    \par Description
    This routine stops advertising of given beacon type.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_bcast_end
(
    /* IN */ UCHAR    bcon,
    /* IN */ UCHAR    type
)
{
    BRR_BEACON_INFO info;

    /* Check if bearer is installed */
    if (NULL == brr_bearer[0].info.bearer_send)
    {
        BRR_ERR("Beacon bearer not up for operation.\n");
        return BRR_INTERFACE_NOT_READY;
    }

    /* Pack the beacon information */
    info.action = ((BRR_DISABLE << 4) | BRR_BROADCAST);
    info.type = ((bcon << 4) | type);
    info.bcon_data = NULL;
    info.bcon_datalen = 0;
    info.uri = NULL;
    return brr_bearer[0].info.bearer_send(&bcon_brr_handle, MESH_AD_TYPE_BCON, &info, sizeof(BRR_BEACON_INFO));
}

/**
    \brief API to send Unprovisioned Device Beacon

    \par Description
    This routine sends Unprovisioned Device Beacon

    \param [in] dev_uuid
           Device UUID uniquely identifying this device.

    \param [in] oob_info
           OOB Information

    \param [in] uri
           Optional Parameter. NULL if not present.
           Points to the length and payload pointer of the URI string to be
           advertised interleaving with the unprovisioned beacon.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_bcast_unprovisioned_beacon
(
    /* IN */ UCHAR       type,
    /* IN */ UCHAR*      dev_uuid,
    /* IN */ UINT16      oob_info,
    /* IN */ MS_BUFFER* uri
)
{
    API_RESULT retval;
    UCHAR pdu[MS_BCON_TYPE_SIZE + MS_DEVICE_UUID_SIZE + MS_BCON_OOB_IND_SIZE + MS_BCON_URI_HASH_SIZE];
    UCHAR marker;
    BRR_BEACON_INFO info;

    /* Check if bearer is installed */
    if (NULL == brr_bearer[0].info.bearer_send)
    {
        BRR_ERR("Beacon bearer not up for operation.\n");
        return BRR_INTERFACE_NOT_READY;
    }

//printf("Bearer type = %d(1-adv, 2-GATT)\r\n", type + 1);    //
    BRR_TRC
    ("[BRR] Sending Unprovisioned Device Beacon\n");
    /* Initialize Marker */
    marker = 0;
    /* Beacon Type */
    pdu[marker ++] = BRR_BCON_TYPE_UNPROV_DEVICE;
    /* Device UUID */
    EM_mem_copy (&pdu[marker], dev_uuid, MS_DEVICE_UUID_SIZE);
    marker += MS_DEVICE_UUID_SIZE;
    /* OOB Information */
    MS_PACK_BE_2_BYTE_VAL(&pdu[marker], oob_info);
    marker += MS_BCON_OOB_IND_SIZE;

    /**
        4 Octets of URI Hash - Optional Field.
        Hash of the associated URI advertised with the URI AD_Type.
    */
    if ((NULL != uri) &&
            (NULL != uri->payload) &&
            (0 != uri->length))
    {
        /* Have a 16 octet S1 result buffer. Of this, the URI hash is only 4 bytes (0 - 3) */
        UCHAR uri_hash[MS_BCON_URI_HASH_SIZE << 2];
        /* Calculate URI Hash uri_hash = s1(uri)[0-3] */
        ms_stbx_s1(uri->payload, uri->length, uri_hash);
        BRR_TRC
        ("[BRR] URI Hash:\n");
        BRR_debug_dump_bytes(uri_hash, MS_BCON_URI_HASH_SIZE);
        /* URI Hash */
        EM_mem_copy (&pdu[marker], uri_hash, MS_BCON_URI_HASH_SIZE);
        marker += MS_BCON_URI_HASH_SIZE;
    }

    /* Pack the beacon information */
    info.action = ((BRR_ENABLE << 4) | BRR_BROADCAST);
    info.type = ((BRR_BCON_TYPE_UNPROV_DEVICE << 4) | type);
    info.bcon_data = pdu;
    info.bcon_datalen = marker;
    info.uri = uri;
    /* Lock */
    BRR_LOCK();
    /* Send beacon over the bearer */
    retval = brr_bearer[0].info.bearer_send(&bcon_brr_handle, MESH_AD_TYPE_BCON, (void*)&info, sizeof (BRR_BEACON_INFO));
    /* Unlock */
    BRR_UNLOCK();
    return retval;
}

/**
    \brief API to broadcast a beacon

    \par Description
    This routine sends the beacon of given type on Adv and GATT bearers

    \param [in] type
           The type of beacon

    \param [in] packet
           Beacon data

    \param [in] length
           Beacon data length

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_broadcast_beacon
(
    /* IN */ UCHAR     type,
    /* IN */ UCHAR*    packet,
    /* IN */ UINT16    length
)
{
    API_RESULT retval;
    BRR_BEACON_INFO info;
    MS_BUFFER  buffer;
    UINT8 index;

    /* Check if bearer is installed */
    if (NULL == brr_bearer[0].info.bearer_send)
    {
        BRR_ERR("Beacon bearer not up for operation.\n");
        return BRR_INTERFACE_NOT_READY;
    }

    BRR_TRC
    ("[BRR] Sending Beacon 0x%02X over Adv Bearer\n", type);
    /* Pack and send the beacon for Beacon Bearer */
    info.action = ((BRR_ENABLE << 4) | BRR_BROADCAST);
    info.type = ((type << 4) | BRR_BCON_PASSIVE);
    info.bcon_data = packet;
    info.bcon_datalen = length;
    info.uri = NULL;
    /* Lock */
    BRR_LOCK();
    /* Send beacon over the bearer */
    retval = brr_bearer[0].info.bearer_send(&bcon_brr_handle, MESH_AD_TYPE_BCON, &info, sizeof (BRR_BEACON_INFO));
    /* Unlock */
    BRR_UNLOCK();

    /* Pack and Send the beacon for all the GATT Bearer */
    for (index = 1; index < MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES); index++)
    {
        buffer.payload = packet;
        buffer.length  = length;
        retval = MS_brr_send_pdu
                 (
                     (BRR_HANDLE*)&index,
                     (BRR_TYPE_GATT | BRR_SUBTYPE_GATT_BEACON_T_MASK),
                     &buffer
                 );
        BRR_TRC(
            "[BLEBRR Id:0x%02X] Sent Beacon over GATT Bearer. Result 0x%04X\n", (BRR_HANDLE)index, retval);
        BRR_debug_dump_bytes(buffer.payload, buffer.length);
    }

    return API_SUCCESS;
}


/**
    \brief API to send Proxy Device ADV

    \par Description
    This routine sends Proxy Device ADV

    \param [in] type
           Proxy ADV Type:
           0x00 - Network ID
           0x01 - Node Identity

    \param [in] data
           Data to be advertised by Proxy.
           If the "type" is:
           0x00 - Network ID    - 8 Bytes of Network ID
           0x01 - Node Identity - 8 Bytes Hash, 8 Bytes Random num

    \param [in] datalen
           Length of the data to be advertised by Proxy.
           If the "type" is:
           0x00 - Network ID    - 8 Bytes of Network ID
           0x01 - Node Identity - 8 Bytes Hash, 8 Bytes Random num

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_start_proxy_adv
(
    /* IN */ UCHAR      type,
    /* IN */ UCHAR*     data,
    /* IN */ UINT16     datalen
)
{
    API_RESULT retval;
    UCHAR pdu[1 + 8 + 8];
    UINT16 marker;
    BRR_BEACON_INFO info;

    /* Check if bearer is installed */
    if (NULL == brr_bearer[0].info.bearer_send)
    {
        BRR_ERR("Beacon bearer not up for operation.\n");
        return BRR_INTERFACE_NOT_READY;
    }

    BRR_TRC
    ("[BRR] Sending Proxy ADV with %s\n",(BRR_BCON_TYPE_PROXY_NETID == type)?"Network ID" : "Node Identity");
    /* Initialize Marker */
    marker = 0;
    /* Proxy ADV Identification mode Type */
    pdu[marker ++] = (BRR_BCON_TYPE_PROXY_NETID == type)?0x00: 0x01;
    /* Proxy Data depending on 'type' */
    EM_mem_copy (&pdu[marker], data, datalen);
    marker += datalen;
    /* Pack the beacon information */
    info.action       = ((BRR_ENABLE << 4) | BRR_BROADCAST);
    info.type         = ((type << 4) | BRR_BCON_ACTIVE);
    info.bcon_data    = pdu;
    info.bcon_datalen = marker;
    info.uri          = NULL;
    /* Lock */
    BRR_LOCK();
    /* Send beacon over the bearer */
    retval = brr_bearer[0].info.bearer_send(&bcon_brr_handle, MESH_AD_TYPE_BCON, (void*)&info, sizeof (BRR_BEACON_INFO));
    /* Unlock */
    BRR_UNLOCK();
    return retval;
}


/**
    \brief API to send common Bearer PDUs

    \par Description
    This routine sends common Bearer PDUs to peer device.

    \param brr_handle
           Identifies the remote Bearer entity to which the PDU is to be
           sent

    \param brr_type
           Identifies the Bearer Type

    \param params
           PDU Specific parameter(s)

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_brr_send_pdu
(
    /* IN */ BRR_HANDLE*     brr_handle,
    /* IN */ BRR_TYPE        brr_type,
    /* IN */ MS_BUFFER*      buffer
)
{
    BRR_BEARER_DATA* bearer;
    API_RESULT retval;
    UCHAR type, sar, tx_state;
    UCHAR* pdata;
    UINT16 datalen, tosend;

    if ((NULL == brr_handle) ||
            (NULL == buffer))
    {
        BRR_ERR("Invalid parameters to API\n");
        return BRR_INVALID_PARAMETER_VALUE;
    }

    pdata = buffer->payload;
    datalen = buffer->length;
    /* Lock */
    BRR_LOCK();
    /* Get the bearer reference from the type and handle */
    bearer = &brr_bearer[*brr_handle];

    /* Verify bearer is registered */
    if (NULL == bearer->info.bearer_send)
    {
        BRR_ERR("Write on unregistered bearer. Aborting...\n");
        /* Unlock */
        BRR_UNLOCK();
        return API_FAILURE;
    }

    retval = API_FAILURE;
    /* Extract Message Type from the Bearer Type */
    /* The top 2 msb bits of bearer type is the message type */
    /* The message type is valid only for BRR_TYPE_GATT */
    type     = ((brr_type & BRR_SUBTYPE_GATT_T_MASK) >> BRR_SUBTYPE_GATT_T_MASK_BIT_OFFSET);
    brr_type = (brr_type & ~(BRR_SUBTYPE_GATT_T_MASK));
    BRR_TRC("[BRR] Bearer Send for Bearer Type 0x%02X, Msg Type 0x%02X\n", brr_type, type);
    BRR_debug_dump_bytes(pdata, datalen);

    /* Check the bearer type */
    switch (brr_type)
    {
    case BRR_TYPE_PB_ADV:
    case BRR_TYPE_ADV:
        /* Set the bearer type */
        type = (BRR_TYPE_PB_ADV == brr_type) ? MESH_AD_TYPE_PB_ADV : MESH_AD_TYPE_PKT;
        retval = bearer->info.bearer_send(brr_handle, type, pdata, datalen);

        if (API_SUCCESS != retval)
        {
            BRR_ERR("Failed to send ADV PDU - 0x%04X\n", retval);
            break;
        }

        break;

    case BRR_TYPE_PB_GATT:
    case BRR_TYPE_GATT:
        /* TODO: Add type for Proxy and Beacon */
        /* Initialize transmit state */
        tx_state = 0x00;

        /* Segment and send the pdu if required */
        do
        {
            bearer->omtu = ATT_GetCurrentMTUSize() - 3;      // add by HZF

            /* Check if the data length PDU with 1 byte header will fit in the MTU */
            if (bearer->omtu < (datalen + 1))
            {
                if (0x00 == tx_state)
                {
                    tx_state = 0x01;
                    sar = MESH_GATT_SAR_START;
                    bearer->gatt_tx_pdu[0] = (sar << 0x06) | (type);
                    bearer->gatt_tx_offset = 0;
                }
                else
                {
                    sar = MESH_GATT_SAR_CONTINUE;
                    bearer->gatt_tx_pdu[0] = (sar << 0x06) | (type);
                    bearer->gatt_tx_offset += (bearer->omtu - 1);
                }

                EM_mem_copy
                (
                    (bearer->gatt_tx_pdu + 1),
                    (pdata + bearer->gatt_tx_offset),
                    (bearer->omtu - 1)
                );
                tosend = bearer->omtu;
                datalen -= (tosend - 1);
            }
            else
            {
                if (0x00 == tx_state)
                {
                    tx_state = 0x01;
                    sar = MESH_GATT_SAR_COMPLETE;
                    bearer->gatt_tx_pdu[0] = (sar << 0x06) | (type);
                    bearer->gatt_tx_offset = 0;
                }
                else
                {
                    sar = MESH_GATT_SAR_END;
                    bearer->gatt_tx_pdu[0] = (sar << 0x06) | (type);
                    bearer->gatt_tx_offset += (bearer->omtu - 1);
                }

                EM_mem_copy
                (
                    (bearer->gatt_tx_pdu + 1),
                    (pdata + bearer->gatt_tx_offset),
                    datalen
                );
                tosend = datalen + 1;
                datalen -= (tosend - 1);
            }

            retval = bearer->info.bearer_send(brr_handle, type, bearer->gatt_tx_pdu, tosend);

            if (API_SUCCESS != retval)
            {
                BRR_ERR("Failed to send GATT PDU - 0x%04X\n", retval);
                break;
            }
        }
        while (datalen != 0);

        break;

    default:
        BRR_ERR("Invalid Bearer Type - 0x%08X\n", brr_type);
        break;
    }

    /* Unlock */
    BRR_UNLOCK();
    return retval;
}


/**
    \brief Get the RSSI of current received packet being processed.

    \par Description
    This routine returns the RSSI value of the received packet in its
    context when called from the Mesh stack.

    \return RSSI value of the current packet in context.

    \note This applies only when the packet is received over ADV bearer

*/
UCHAR MS_brr_get_packet_rssi(void)
{
    BRR_TRC ("Get Packet RSSI - %d\n", brr_packet_rssi);
    return brr_packet_rssi;
}

/**
    \brief Put the bearer to sleep.

    \par Description
    This routine requests the underlying bearer interface to sleep.
    Default bearer interface is that of advertising bearer.

    \return API_SUCCESS

*/
API_RESULT MS_brr_sleep(void)
{
    BRR_BEARER_DATA* bearer;
    BRR_HANDLE handle;
    BRR_TRC ("Request Bearer Sleep\n");
    /* Get the ADV bearer reference */
    handle = 0;
    bearer = &brr_bearer[handle];

    if (NULL != bearer->info.bearer_sleep)
    {
        bearer->info.bearer_sleep(&handle);
    }

    return API_SUCCESS;
}

/**
    \brief Wakeup the bearer.

    \par Description
    This routine requests the underlying bearer interface to wakeup.
    Default bearer interface is that of advertising bearer.

    \param mode
           Identifies the mode (BRR_TX/BRR_RX) for which bearer is requested
           for wakeup.

    \return API_SUCCESS

*/
API_RESULT MS_brr_wakeup(UINT8 mode)
{
    BRR_BEARER_DATA* bearer;
    BRR_HANDLE handle;
    BRR_TRC ("Request Bearer Wakeup\n");
    /* Get the ADV bearer reference */
    handle = 0;
    bearer = &brr_bearer[handle];

    if (NULL != bearer->info.bearer_wakeup)
    {
        bearer->info.bearer_wakeup(&handle, mode);
    }

    return API_SUCCESS;
}
#endif /* MS_BRR */

