
/**
    \file brr_init.c

    Module initialization routine and tables defined here.

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "brr_internal.h"
#include "brr.h"

#ifdef MS_BRR
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */
MS_DEFINE_MUTEX (brr_mutex)

/** List of Mesh Bearer Information */
MS_DEFINE_GLOBAL_ARRAY(BRR_BEARER_DATA, brr_bearer, MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES));

/* TODO: Check if required */
#if 0
    /** Mesh Beacon Bearer Information */
    BRR_BEARER_INFO brr_bcon_bearer;
#endif /* 0 */

/** List of bearer specific callbacks */
BRR_NTF_CB brr_bearer_cb[BRR_COUNT];

/** List of beacon callbacks */
BRR_BCON_CB brr_beacon_cb[BRR_BCON_COUNT];

/** Bearer Handle for Beacon */
BRR_HANDLE bcon_brr_handle;

/** Received packet RSSI. Valid only for ADV bearer */
UCHAR brr_packet_rssi;

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */

/**
    \brief Initializes Module.

    \par Description Initializes Module tables.

    \param None

    \return None
*/

void ms_brr_init (void)
{
    ms_internal_verificaiton_check();
    BRR_TRC
    ("[BRR]: Initializing BRR..");
    /* Module Mutex Initialization */
    MS_MUTEX_INIT_VOID (brr_mutex, BRR);
    /* Initialize the receive packet RSSI */
    brr_packet_rssi = 0;
    MS_INIT_GLOBAL_ARRAY(BRR_BEARER_DATA, brr_bearer, MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES), 0x00);
}


#ifndef MS_NO_SHUTDOWN
/**
    \par Description:
    This function is the shutdown handler for BRR module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.

    \param None

    \return None
*/
void ms_brr_shutdown (void)
{
    BRR_TRC (
        "[BRR]: Bearer Shutdown Successful.");
}
#endif /* MS_NO_SHUTDOWN */

/**
    \brief Interface to receive data from bearer

    \par Description
    This routine receives data sent by bearer interface

    \return None
*/
void brr_read_data_ind(BRR_HANDLE* brr_handle, UCHAR* data, UINT16 datalen, MS_BUFFER* info)
{
    BRR_BEARER_DATA* bearer;
    UCHAR type, bcon_type;
    UCHAR brrtype, evttype;

    /* Check if bearer handle is in range and registered */
    if (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) <= (*brr_handle))
    {
        BRR_ERR("Bearer handle out of range. Dropping...\n");
        return;
    }

    if (NULL == brr_bearer[*brr_handle].info.bearer_send)
    {
        BRR_ERR("Data received on unregistered bearer. Dropping...\n");
        return;
    }

    /* TBD: Do we need to do data and length check? */
    /* Reference the bearer */
    bearer = &brr_bearer[*brr_handle];

    /*
        Check the bearer handle received
        - Handle 0: Adv bearer
        - Handle Others: GATT bearer
    */
    if (0 == (*brr_handle))
    {
        /* Get the type of packet */
        type = data[0];

        /* Set the RSSI of the received packet */
        if ((NULL != info) &&
                (NULL != info->payload))
        {
            brr_packet_rssi = *((UCHAR*)info->payload);
        }

        /* Check the advertising type */
        switch (type)
        {
        case MESH_AD_TYPE_BCON:
            /* Advertising Bearer */
            BRR_INF("Beacon on ADV Bearer:\n");
            BRR_debug_dump_bytes(data, datalen);
            /* Get the beacon type */
            bcon_type = data[1];

//                BRR_TRC("Beacon packet with beacon type: 0x%02X\n", bcon_type);

            if ((BRR_BCON_TYPE_SECURE_NET >= bcon_type) && (NULL != brr_beacon_cb[bcon_type]))
            {
//                    BRR_TRC("Callback on beacon bearer type: 0x%02X\n", bcon_type);
                brr_beacon_cb[bcon_type]((data + 2), (datalen - 2));
            }

            #ifdef BRR_DEBUG
            else
            {
                BRR_ERR("No Callback registered for beacon bearer type: 0x%02X\n", bcon_type);
            }

            #endif /* BRR_DEBUG */
            break;

        case MESH_AD_TYPE_PB_ADV:
        case MESH_AD_TYPE_PKT:
            BRR_INF("Data on ADV Bearer:\n");
            BRR_debug_dump_bytes(data, datalen);
            /* Get the bearer type */
            brrtype = (MESH_AD_TYPE_PB_ADV == type) ? BRR_TYPE_PB_ADV : BRR_TYPE_ADV;

            if (NULL != brr_bearer_cb[brrtype])
            {
//                    BRR_TRC("Callback on bearer type: 0x%02X\n", brrtype);
//printf("receive adv packet, len = %d\r\n", datalen);
                brr_bearer_cb[brrtype](brr_handle, BRR_IFACE_DATA, (data + 1), (datalen - 1));
            }

            #ifdef BRR_DEBUG
            else
            {
                BRR_ERR("No Callback registered for bearer type: 0x%02X\n", brrtype);
            }

            #endif /* BRR_DEBUG */
            break;

        default:
            /* BRR_INF("Unknown type of packet on Adv Bearer. Dropping...\n"); */
            break;
        }

        /* Reset the packet RSSI */
        brr_packet_rssi = 0;
    }
    else
    {
        UCHAR proxy_pdu_header, sar_val, msg_type;
        UCHAR complete;
        /* GATT Bearer */
        BRR_TRC("Data on GATT Bearer:\n");
        BRR_debug_dump_bytes(data, datalen);
        /* Extract the first byte Header of data */
        proxy_pdu_header = *(data + 0);
        /* TODO: Have #defines for the Magic Numbers below */
        sar_val = ((proxy_pdu_header & 0xC0) >> 0x06);
        msg_type = (proxy_pdu_header & 0x3F);

        /* Process only Proxy PDU Message type of Provisioning PDU */
        if (MESH_GATT_TYPE_PROV < msg_type)
        {
            BRR_ERR("Incorrect message type 0x%02X on GATT bearer. Dropping...\n", msg_type);
            return;
        }

        /* TODO: State machine for SAR */
        complete = MS_FALSE;

        switch (sar_val)
        {
        /* 0x00: Complete packet */
        case MESH_GATT_SAR_COMPLETE:
            EM_mem_copy
            (
                bearer->gatt_rx_pdu,
                (data + 1),
                (datalen - 1)
            );
            bearer->gatt_rx_offset = (datalen - 1);
            /* Mark for completion */
            complete = MS_TRUE;
            break;

        /* 0x01: Start Packet */
        case MESH_GATT_SAR_START:
            bearer->gatt_rx_offset = 0;
            EM_mem_copy
            (
                bearer->gatt_rx_pdu,
                (data + 1),
                (datalen - 1)
            );
            /* TODO: Check length */
            bearer->gatt_rx_offset = (datalen - 1);
            break;

        /* 0x02: Continue Packet */
        case MESH_GATT_SAR_CONTINUE:
            /* TODO: Check for overflow */
            EM_mem_copy
            (
                bearer->gatt_rx_pdu + bearer->gatt_rx_offset,
                (data + 1),
                (datalen - 1)
            );
            bearer->gatt_rx_offset += (datalen - 1);
            break;

        /* 0x03: End Packet */
        case MESH_GATT_SAR_END:
            EM_mem_copy
            (
                bearer->gatt_rx_pdu + bearer->gatt_rx_offset,
                (data + 1),
                (datalen - 1)
            );
            bearer->gatt_rx_offset += (datalen - 1);
            /* Mark for completion */
            complete = MS_TRUE;
            break;
        }

        /* TODO: Msg Type checking and handling for both ADV and GATT can be made common */
        if (MS_TRUE == complete)
        {
            switch (msg_type)
            {
            case MESH_GATT_TYPE_BEACON:
                /* Get the beacon type */
                bcon_type = bearer->gatt_rx_pdu[0];
                BRR_TRC("Beacon packet with beacon type: 0x%02X\n", bcon_type);

                if (NULL != brr_beacon_cb[bcon_type])
                {
                    BRR_TRC("Callback on beacon bearer type: 0x%02X\n", bcon_type);
                    brr_beacon_cb[bcon_type]((bearer->gatt_rx_pdu + 1), (bearer->gatt_rx_offset - 1));
                }

                #ifdef BRR_DEBUG
                else
                {
                    BRR_ERR("No Callback registered for beacon bearer type: 0x%02X\n", bcon_type);
                }

                #endif /* BRR_DEBUG */
                break;

            case MESH_GATT_TYPE_NETWORK:
            case MESH_GATT_TYPE_PROXY: /* Proxy packet will also be passed to Network Layer */
            case MESH_GATT_TYPE_PROV:
                /* Get the bearer type */
                brrtype = (MESH_GATT_TYPE_PROV == msg_type) ? BRR_TYPE_PB_GATT : BRR_TYPE_GATT;
                /* Set the event type */
                evttype = (MESH_GATT_TYPE_PROXY == msg_type) ? BRR_IFACE_PROXY_DATA : BRR_IFACE_DATA;

                if (NULL != brr_bearer_cb[brrtype])
                {
                    BRR_TRC("Callback on bearer type: 0x%02X\n", brrtype);
                    brr_bearer_cb[brrtype](brr_handle, evttype, bearer->gatt_rx_pdu, bearer->gatt_rx_offset);
                }

                #ifdef BRR_DEBUG
                else
                {
                    BRR_ERR("No Callback registered for bearer type: 0x%02X\n", brrtype);
                }

                #endif /* BRR_DEBUG */
                break;
            }
        }
    }
}

#endif /* MS_BRR */

