
/**
    \file prov_internal.c


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "prov_fsm_engine.h"
#include "prov_extern.h"

#include "sec_tbx.h"

#include "MS_access_api.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
/** Static Table for Calculation of Provisioning Frame Checksum */
DECL_STATIC DECL_CONST UCHAR prov_crc_table [PROV_CRC_TABLE_SIZE] =
{
    0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C,
    0x09, 0x98, 0xEA, 0x7B, 0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69,
    0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 0x38, 0xA9, 0xDB, 0x4A,
    0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43,
    0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58,
    0x2D, 0xBC, 0xCE, 0x5F,

    0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C,
    0x79, 0xE8, 0x9A, 0x0B, 0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19,
    0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17, 0x48, 0xD9, 0xAB, 0x3A,
    0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33,
    0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28,
    0x5D, 0xCC, 0xBE, 0x2F,

    0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C,
    0xE9, 0x78, 0x0A, 0x9B, 0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
    0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 0xD8, 0x49, 0x3B, 0xAA,
    0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3,
    0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8,
    0xCD, 0x5C, 0x2E, 0xBF,

    0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC,
    0x99, 0x08, 0x7A, 0xEB, 0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9,
    0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 0xA8, 0x39, 0x4B, 0xDA,
    0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3,
    0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8,
    0xBD, 0x2C, 0x5E, 0xCF
};

EM_timer_handle link_open_ack_timer_handle = EM_TIMER_HANDLE_INIT_VAL;


/** TODO: For random delay during Provisioning PDU and Ack transmission */
#if 0
    DECL_STATIC UCHAR prov_rx_deliver;
    DECL_STATIC UCHAR prov_tx_deliver;
#endif /* 0 */

//#if 1
//    PROV_BRR_ADV_COUNT adv_message_count;
//#endif

/* --------------------------------------------- Functions */

/**
    \brief
    Calculate FCS

    \par Description
    This function calculates Provisioning FCS for a given data.

    \param data (IN)
           The Data for which FCS to be calculated
    \param datalen (IN)
           The Length of the Data for which FCS to be calculated
    \param fcs (OUT)
           The calculated FCS value

    \return void
*/
void prov_generate_fcs
(
    /* IN */  UCHAR*     data,
    /* IN */  UINT16     datalen,
    /* OUT */ UCHAR*     fcs
)
{
    UCHAR fcs_calc;
    /* Initialize FCS */
    fcs_calc = PROV_FCS_INIT_VALUE;

    while (datalen --)
    {
        fcs_calc = prov_crc_table[(fcs_calc) ^ *data++];
    }

    /* Get One's Complement & Store FCS */
    *fcs = ~fcs_calc;
}


/**
    \brief
    Crosscheck received FCS

    \par Description
    This function checks received Provisioning FCS for a given data.

    \param data (IN)
           The Data for which FCS to be checked
    \param datalen (IN)
           The Length of the Data for which FCS to be checked
    \param fcs_recvd (IN)
           The received FCS value

    \return UCHAR
            Boolean Flag. True, if FCS matches. Otherwise, False.

    \note
*/
UCHAR prov_verify_fcs
(
    /* IN */  UCHAR*     data,
    /* IN */  UCHAR      datalen,
    /* IN */  UCHAR      fcs_recvd
)
{
    UCHAR fcs_calc;
    /* Initialize FCS */
    fcs_calc = PROV_FCS_INIT_VALUE;

    while (datalen --)
    {
        fcs_calc = prov_crc_table[fcs_calc ^ *data++];
    }

    fcs_calc = prov_crc_table[fcs_calc ^ fcs_recvd];

    /* Check FCS */
    if (PROV_FCS_CHECK_VALUE != fcs_calc)
    {
        return 0x0;
    }

    return 0x1;
}


API_RESULT prov_adv_recv_cb(BRR_HANDLE* handle, UCHAR pevent, UCHAR* pdata, UINT16 pdatalen)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    /* Lock */
    PROV_LOCK();

    /* Get the event */
    switch (pevent)
    {
    case BRR_IFACE_UP:

        /*
            Set the handle of the ADV interface (index 0) in the
            interface list
        */
        if (BRR_HANDLE_INVALID == prov_brr[0].handle)
        {
            prov_brr[0].handle = *handle;
        }
        else
        {
            retval = API_FAILURE;
        }

        break;

    case BRR_IFACE_DOWN:

        /*
            Reset the handle of the ADV interface (index 0) in the
            interface list
        */
        if (*handle == prov_brr[0].handle)
        {
            prov_brr[0].handle = BRR_HANDLE_INVALID;
        }
        else
        {
            retval = API_FAILURE;
        }

        break;

    case BRR_IFACE_DATA:

        /* Drop the data if mismatch in handle */
        if (*handle != prov_brr[0].handle)
        {
            break;
        }

        /*
            Check with the configured input filter for the advertising interface before
            forwarding the packet to the network layer
        */
        prov_handle_adv_message(pdata, pdatalen);
        break;

    default:
        break;
    }

    /* Unlock */
    PROV_UNLOCK();
    return retval;
}

API_RESULT prov_gatt_recv_cb(BRR_HANDLE* handle, UCHAR pevent, UCHAR* pdata, UINT16 pdatalen)
{
    API_RESULT retval;
    UCHAR i;
    retval = API_SUCCESS;
    /* Lock */
    PROV_LOCK();

    /* Get the event */
    switch (pevent)
    {
    case BRR_IFACE_UP:

        /* End beaconing */
//        MS_prov_stop_interleave_timer();
//        MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_ACTIVE);

        /*
            Allocate a free interface from the list and set the handle
            for the GATT interface
        */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES); i++)
        {
            if (BRR_HANDLE_INVALID == prov_brr[i].handle)
            {
                prov_brr[i].handle = *handle;
                break;
            }
        }

        if (MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES) == i)
        {
            retval = API_FAILURE;
            break;
        }

        /* Update state */
        PROV_SET_STATE(PROV_STATE_INITIALIZED);
        break;

    case BRR_IFACE_DOWN:

        /*
            Reset the handle of the GATT interface corresponding to
            the handle in interface list
        */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES); i++)
        {
            if (*handle == prov_brr[i].handle)
            {
                prov_brr[i].handle = BRR_HANDLE_INVALID;
                break;
            }

            if (MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES) == i)
            {
                retval = API_FAILURE;
            }
        }

        break;

    case BRR_IFACE_DATA:
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES); i++)
        {
            if (*handle == prov_brr[i].handle)
            {
                break;
            }
        }

        if (MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES) == i)
        {
            break;
        }

        /*
            check with the configured input filter for the gatt interface before
            forwarding the packet to the network layer
        */
        prov_handle_gatt_message(&prov_brr[i], pdata, pdatalen);
        break;

    default:
        break;
    }

    /* Unlock */
    PROV_UNLOCK();
    return retval;;
}


/* Provisioning Message Beacon Handler */
void prov_handle_unprovisioned_beacon (UCHAR* pdata, UINT16 pdatalen)
{
    PROV_DEVICE_S dev;
    UINT32        uri_hash;
    UCHAR marker;
    /* Lock */
    PROV_LOCK_VOID();

    /* Find context if any that is scanning for beacons */
    if (PROV_STATE_SCANNING != prov_state)
    {
        PROV_INF("Not in scanning state to service beacons.\n");
        /* Unlock */
        PROV_UNLOCK_VOID();
        return;
    }

    marker = 0;
    EM_mem_set(&dev, 0x00, sizeof(PROV_DEVICE_S));
    /* Copy the UUID */
    EM_mem_copy(dev.uuid, pdata, MS_DEVICE_UUID_SIZE);
    marker += MS_DEVICE_UUID_SIZE;

    /* Get the OOB type if available */
    if (marker < pdatalen)
    {
        MS_UNPACK_LE_2_BYTE(&dev.oob, (pdata + marker));
        marker += sizeof(UINT16);
    }

    /* Get the URI Hash if available */
    if (marker < pdatalen)
    {
        MS_UNPACK_LE_4_BYTE(&uri_hash, (pdata + marker));
        dev.uri = (MS_BUFFER*)uri_hash;
        marker += sizeof(UINT32);
    }

    /* Notify the application */
    prov_notify(NULL, PROV_EVT_UNPROVISIONED_BEACON, API_SUCCESS, &dev, sizeof(PROV_DEVICE_S));
    /* Unlock */
    PROV_UNLOCK_VOID();
}

/* Handle the acknowledgement */
void prov_handle_ack (PROV_CONTEXT* ctx)
{
    /* Yes, Increment Transaction ID */
    PROV_INCR_TXN_ID(ctx);
    /* Stop the retransmission timer */
    EM_stop_timer(&ctx->rtx_timer_handle);
    /* Update to the next expected state */
    PROV_CONTEXT_SET_STATE(ctx, ctx->ackstate);
    /* Reset the retransmit count */
    ctx->rtx_count = 0;

    if (ev_prov_complete == ctx->ackevent)
    {
        /* Notify procedure completion */
        prov_procedure_complete(ctx, API_SUCCESS);
    }
    else if (0x00 != ctx->ackevent)
    {
        UCHAR ackevent;
        /*
            Backup ackevent to be posted via fsm. Backing up here and resetting
            in case the handler sets the next ack event in the context
        */
        ackevent = ctx->ackevent;
        ctx->ackevent = 0x00;
        prov_fsm_post_event(ctx, ackevent, ctx->ackeventdir, NULL, 0);
    }
}

void prov_restart_proc_timer(PROV_CONTEXT* ctx)
{
    API_RESULT retval;

    if (PROV_STATE_UNINITIALIZED == ctx->state)
    {
        PROV_INF ("Restart Proc Timer on Invalid context\n");
        return;
    }

    PROV_TRC("Restart Procedure timer for Context state : 0x%08X, Context LinkId : 0x%08X\n",
             ctx->state, ctx->link_id);

    if (EM_TIMER_HANDLE_INIT_VAL != ctx->prov_timer_handle)
    {
        EM_stop_timer(&ctx->prov_timer_handle);
    }

    retval = EM_start_timer
             (
                 &ctx->prov_timer_handle,
                 PROV_PROC_TIMEOUT_SEC,
                 prov_proc_timeout_handler,
                 (void*)&ctx,
                 sizeof(void*)
             );

    if (API_SUCCESS != retval)
    {
        PROV_ERR("Failed to start Procedure Timer - 0x%04X\n", retval);
    }
}

/* Provisioning Message Adv Handler */
void prov_handle_adv_message(UCHAR* pdata, UINT16 pdatalen)
{
    PROV_CONTEXT* ctx;
    UINT32   link_id;
    UCHAR    txn_id, info, opcode, ctrl_field, reason, num_frag, frag_idx;
    UCHAR*   uuid;
    UINT16   payload_length, fcs;
    UINT16   marker;
    API_RESULT retval;

    /**
        Provisioning Adv Message Format

        Number of Octets:
           1          1            4           1            0-24
        +--------+------------+---------+----------------+---------+
        | Length | PB AD Type | Link ID | Transaction No | Payload |
        |  (00)  |            |         |                |         |
        +--------+------------+---------+----------------+---------+


           1          1            4           1             1         0 - 23
        +--------+------------+---------+----------------+---------+............+
        | Length | PB AD Type | Link ID | Transaction No | Info    | Data       .
        |  (00)  |            |         |                | (Ctrl)  | (optional) .
        +--------+------------+---------+----------------+---------+............+
    */

    /* This function will receive from Link ID */
    /* Minimum Length is 6 (4 + 1 + 1) */

    if (pdatalen < 6)
    {
        PROV_ERR(
            "[PROV] Insufficient Provisioning Message Length (%d). Dropping ...\n", pdatalen);
        return;
    }

    /* Little Endian Format */
    marker = 0;
    /* Extract Link ID */
    MS_UNPACK_BE_4_BYTE(&link_id, &pdata[marker]);
    marker += 4;
    /* Get the context for this link ID */
    /* Transaction Number */
    txn_id = pdata[marker++];
    /* Info */
    info = pdata[marker++];
    ctrl_field = info & 0x03;
    /* Get Opcode - Might not be opcode always */
    opcode = info >> 2;
    PROV_TRC(
        "[PROV] Link ID 0x%08X, Transaction Number 0x%02X\n", link_id, txn_id);

    /* Check Link ID, outside itself and drop packet if required */
    if ((PROV_PCF_TX_SPECIFIC != ctrl_field) || (PROV_PB_ADV_OPEN_REQ != opcode))
    {
        /* Find context by Link ID */
        ctx = prov_findcontext_by_linkid (link_id);
    }
    else
    {
        /* Refer the UUID */
        uuid = &pdata[marker];
        PROV_TRC("[PROV] LinkOpen Req with UUID:\n");
        PROV_debug_dump_bytes(uuid, MS_DEVICE_UUID_SIZE);
        /* This is a Link Open Request. Find the context by the UUID */
        ctx = prov_findcontext_by_uuid (uuid);
    }

    /* If context is NULL, return */
    if (NULL == ctx)
    {
        PROV_ERR ("[PROV] No context found for received packet. Dropping...\n");
        /* No context found for received packet. Dropping... */
        return;
    }

    /* Lock */

    /* Drop the packet if it has the same transaction ID already received */
    if ((txn_id == ctx->rx_txn_id) &&
            (NULL == ctx->rx_pdu))
    {
        prov_framensend_ack(ctx);
//        adv_message_count.count++;
//        PROV_INF ("[PROV] Dropping PDU with same transaction ID\n");
//        if(!(adv_message_count.count & 0x07))
//        {
//            prov_framensend_ack(ctx);
//            prov_framensend_ack(ctx);
//        }
        /* Unlock */
        return;
    }

    #ifdef MS_PROVISIONING_STRICT_ACK_CHECK

    /*
        If an ACK is being awaited, and the received transaction ID is not
        the expected, drop the packet
    */
    if ((EM_TIMER_HANDLE_INIT_VAL != ctx->rtx_timer_handle) &&
            (txn_id != ctx->tx_txn_id))
    {
        PROV_ERR("New packet received without Ack for previously transmitted packet. "
                 "Dropping packet with Txn ID: 0x%02X\n", txn_id);
        return;
    }

    #else /* MS_PROVISIONING_STRICT_ACK_CHECK */

    /*
        If an ACK is being awaited, two things can happen
        - An ACK is received for the Transmitted packet
        - The next packet is received because the ACK is missed out

        If it is none of the above two cases, drop the packet
    */
    if ((EM_TIMER_HANDLE_INIT_VAL != ctx->rtx_timer_handle) &&
            (PROV_PCF_TX_SPECIFIC != ctrl_field))
    {
        PROV_INF ("[PROV] ACK awaited for transmitted packet with Txn ID 0x%02X\n",
                  ctx->tx_txn_id);
        PROV_INF ("[PROV] Packet Recvd with Txn ID 0x%02X.\n", txn_id);

        /* Is it the ACK packet with same transmit txn id? */
        if (txn_id == ctx->tx_txn_id)
        {
            /* Yes. Looks like an ACK packet */
            PROV_INF ("ACK Packet.\n");
        }
        /* Is it the next expected receive packet? */
        else if ((0x00 != txn_id) && (0x80 != txn_id) &&
                 (txn_id == (ctx->rx_txn_id + 1)))
        {
            /* Yes. Looks like it */
            PROV_INF ("[PROV] Next Expected Recv Packet.\n");
            /* Process ACK Automatically */
            prov_handle_ack (ctx);
        }
        else
        {
            /* Drop the packet */
            PROV_ERR("Unexpected packet recvd when waiting for ACK. Dropping...\n");
            return;
        }
    }

    #endif /* MS_PROVISIONING_STRICT_ACK_CHECK */

    switch (ctrl_field)
    {
    case PROV_PCF_CTRL_MSG:
    {
        /* This can only be the ACK for the last transmitted PDU */

        /* Is ACK pending on this link? */
        if (EM_TIMER_HANDLE_INIT_VAL == ctx->rtx_timer_handle)
        {
            PROV_INF("No Pending ACK on link. Dropping ACK with Txn ID: 0x%02X\n", txn_id);
            return;
        }

        /* Is ACK received for the last transmitted PDU on this link? */
        if (txn_id == ctx->tx_txn_id)
        {
            /* Handle the Acknowledgement */
            prov_handle_ack (ctx);
        }
        else
        {
            PROV_ERR("Unexpected ACK with TID: 0x%02X. Expected TID: 0x%02X\n",
                     txn_id, ctx->tx_txn_id);
        }
    }
    break;

    case PROV_PCF_NUM_FRGMNTS:
    {
        /* TBD: Work Around: What if a partial packet was received and now a new PDU started coming */
        if (NULL != ctx->rx_pdu)
        {
            PROV_TRC(
                "[PROV] Receiving Start of PDU. But the previous packet is not completely received.\n");

            if (ctx->rx_txn_id == txn_id)
            {
                PROV_INF(
                    "[PROV] Dropping same start segment PDU.\n");
                break;
            }

            PROV_TRC(
                "[PROV] Freeing previous PDU and processing the current one.\n");
            /* Free Memory */
            EM_free_mem(ctx->rx_pdu);
            /* Clean Rx PDU related elements */
            ctx->rx_pdu = NULL;
        }

        /* Start of Fragment */
        /* Check if complete packet - check length and also fragment count */
        /* Get Number of Fragments */
        num_frag = info >> 2;
        /* Extract Total Length */
        MS_UNPACK_BE_2_BYTE(&payload_length, &pdata[marker]);
        marker += 2;
        /* Save PDU Total Length */
        ctx->rx_pdu_len = payload_length;
        /* First Fragment */
        ctx->rx_frag_index = 0;
        /* Extract FCS */
        fcs = pdata[marker++];
        /* Save FCS */
        ctx->fcs = fcs;
        /* Save Transaction ID */
        ctx->rx_txn_id = txn_id;
        /* Partial PDU Length */
        ctx->rx_partial_pdu_len = pdatalen - marker;

        /* TBD: Will the Number of fragment be 0 or 1 */
        if ((0 == num_frag))
        {
            /* Length Check */
            if (ctx->rx_partial_pdu_len != ctx->rx_pdu_len)
            {
                PROV_ERR(
                    "[PROV] Invalid Payload Length\n");
                /* Reset PDU Length */
                ctx->rx_pdu_len = 0;
                ctx->rx_partial_pdu_len = 0;
                return;
            }

            ctx->rx_pdu = &pdata[marker];
            /* If complete PDU - Call PDU handler */
            prov_rx_delay(ctx);
            prov_framensend_ack(ctx);
            prov_framensend_ack(ctx);
            prov_handle_pdu(ctx);
            /* Restart the Procedure timer */
            prov_restart_proc_timer(ctx);
        }
        /* If fragmented, allocate memory for the complete PDU */
        else
        {
            /* Allocate Memory */
            ctx->rx_pdu = EM_alloc_mem(ctx->rx_pdu_len);

            if (NULL == ctx->rx_pdu)
            {
                PROV_ERR(
                    "[PROV] Failed to allocate %d bytes\n", ctx->rx_pdu_len);
                return;
            }

            /* Copy first fragment */
            EM_mem_copy
            (
                ctx->rx_pdu,
                &pdata[marker],
                ctx->rx_partial_pdu_len
            );
        }
    }
    break;

    case PROV_PCF_CONTINU_FRGMNT:
    {
        /* Work around: What if the first packet is not received */
        if (NULL == ctx->rx_pdu)
        {
            PROV_INF(
                "[PROV] Receiving Continuation. Did not received the first packet. Dropping...\n");
            return;
        }

        /* Get Fragment Index */
        frag_idx = info >> 2;

        /* Check Transaction ID and Fragment Index */
        if ((ctx->rx_txn_id != txn_id) || ((ctx->rx_frag_index + 1) != frag_idx))
        {
            /* Cleanup ?*/
            PROV_ERR(
                "[PROV] Incorrect Transaction ID (0x%02X) or Fragment Index (0x%02X)\n", txn_id, frag_idx);
            break;
        }

        /* Save current Frag Index */
        ctx->rx_frag_index = frag_idx;
        /* Call reassemble */
        EM_mem_copy
        (
            &ctx->rx_pdu[ctx->rx_partial_pdu_len],
            &pdata[marker],
            (pdatalen - marker)
        );
        /* Partial PDU Length */
        ctx->rx_partial_pdu_len += (pdatalen - marker);

        /* Once reassembly is complete - Call the PDU handler */
        if (ctx->rx_partial_pdu_len == ctx->rx_pdu_len)
        {
            prov_rx_delay(ctx);
            prov_framensend_ack(ctx);
            prov_handle_pdu(ctx);
            /* Restart the Procedure timer */
            prov_restart_proc_timer(ctx);
        }
    }
    break;

    case PROV_PCF_TX_SPECIFIC:
    {
        /* Get Opcode */
        /* opcode = info >> 2; */
        prov_rx_delay (ctx);

        switch (opcode)
        {
        case PROV_PB_ADV_OPEN_REQ:
        {
            /* Match Device UUID */
            PROV_TRC(
                "[PROV] <<-- Link Open Request\n");

            /* Check Role and State */
            if ((MS_TRUE != PROV_IS_ROLE_DEVICE(ctx)) ||
                    (PROV_STATE_BEACONING != PROV_GET_STATE()))
            {
                if ((SL_0_PROV_IDLE == PROV_CONTEXT_GET_STATE(ctx)) &&
                        (link_id == ctx->link_id))
                {
                    PROV_TRC(
                        "[PROV] -->> Link Accept\n");
                    //Retry 2 times
                    prov_link_accept(ctx);
                    prov_link_accept(ctx);
                    break;
                }
                else
                {
                    PROV_ERR ("Context not in expected state\n");
                    break;
                }
            }

            PROV_TRC("[PROV] Device UUID and provisioning state matched. Proceed...\n");
            /* End beaconing */
            MS_prov_stop_interleave_timer();
            MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_PASSIVE);
            /* Update state */
            PROV_SET_STATE(PROV_STATE_INITIALIZED);
            /* Save Link ID */
            ctx->link_id = link_id;
            /* Ignore the Transaction ID */
            /* Initialize the Tx transaction ID and limit */
            ctx->tx_txn_id = 0x80;
            ctx->txn_id_max = 0xFF;
            PROV_TRC(
                "[PROV] -->> Link Accept and Process\n");
            /* Accept the link open request */
            retval = prov_link_accept(ctx);
            retval = prov_link_accept(ctx);

            /* If success change state */
            if (API_SUCCESS == retval)
            {
                /* Set link establishment */
                PROV_SET_LINK_OPEN(ctx);
                /* Update the state */
                PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_IDLE);
                /* Start the ACK tracking retransmission timer */
                retval = EM_start_timer
                         (
                             &link_open_ack_timer_handle,
                             EM_TIMEOUT_MILLISEC|(PROV_PDU_RTX_TIMEOUT_SEC *PROV_PDU_RTX_COUNT+1000),
                             prov_link_ack_timeout_handler,
                             (void*)&ctx,
                             sizeof(void*)
                         );

                if (API_SUCCESS != retval)
                {
                    PROV_ERR("Failed to start LinkOpen ack Timer - 0x%04X\n", retval);
                }
            }
        }
        break;

        case PROV_PB_ADV_OPEN_CNF:
        {
            PROV_TRC(
                "[PROV] <<-- Open Confirmation\n");

            /* Change State to Link in Open state */
            if ((MS_TRUE != PROV_IS_ROLE_PROVISIONER(ctx)) ||
                    (PROV_STATE_LINKOPEN != PROV_CONTEXT_GET_STATE(ctx)))
            {
                PROV_ERR ("Context not in expected state\n");
                break;
            }

            /* Stop the retransmission timer if running */
            if (EM_TIMER_HANDLE_INIT_VAL != ctx->rtx_timer_handle)
            {
                EM_stop_timer(&ctx->rtx_timer_handle);
            }

            /* Set link establishment */
            PROV_SET_LINK_OPEN(ctx);
            /* Initialize the Tx transaction ID and limit */
            ctx->tx_txn_id = 0x00;
            ctx->txn_id_max = 0x7F;
            /* If success change state */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_IDLE);
            /* Send provisioning Invite */
            prov_fsm_post_levent (ctx, ev_prov_invite, NULL, 0);
        }
        break;

        case PROV_PB_ADV_CLOSE_IND:
        {
            /* Extract Close Reason */
            reason = pdata[marker++];
            /* Match Device UUID */
            PROV_TRC(
                "[PROV] <<-- Close Indication. Reason 0x%02X\n", reason);
            /* No Ack to be sent */
            /* Reset link establishment */
            PROV_RESET_LINK_OPEN(ctx);
            /* Process procedure completion */
            prov_procedure_complete (ctx, reason);
        }
        break;
        }
    }
    break;
    }

    return;
}

/* Provisioning Message Adv Handler */
void prov_handle_gatt_message(PROV_BRR_INFO* pbrr, UCHAR* pdata, UINT16 pdatalen)
{
    PROV_CONTEXT* ctx;
    UCHAR pdu[PROV_MAX_PB_GATT_MTU];
    /* Find context by Link ID */
    ctx = prov_findcontext_by_brr(pbrr);

    if (NULL == ctx)
    {
        /* Unlock */
        return;
    }

    /* Initialize the Locals */
    EM_mem_set(pdu, 0x0, PROV_MAX_PB_GATT_MTU);
    /* Copy the PDU */ /* TODO: Check if the copy is required */
    EM_mem_copy(pdu, pdata, pdatalen);
    /* Reference to the data and length in the context */
    ctx->rx_pdu = &pdu[0];
    ctx->rx_pdu_len = pdatalen;
    /* If complete PDU - Call PDU handler */
    prov_handle_pdu(ctx);
}


void prov_handle_pdu (PROV_CONTEXT* ctx)
{
    API_RESULT retval;
    UCHAR pdu_type;
    UCHAR* pdata;
    UINT16 pdatalen;
    /* Decode and Print the Packet */
    PROV_TRC(
        "[PROV] <<-- PDU\n");
    PROV_debug_dump_bytes(ctx->rx_pdu, ctx->rx_pdu_len);
    /* Process PDU. Get the PDU type */
    pdu_type = ctx->rx_pdu[0];
    pdata = ctx->rx_pdu + 1;
    pdatalen = ctx->rx_pdu_len - 1;
    PROV_TRC("Recvd PDU Type: 0x%02X\n", pdu_type);
    #if 0

    /* Send Ack only if bearer type is PB ADV */
    if (BRR_TYPE_PB_ADV == ctx->bearer)
    {
        prov_framensend_ack(ctx);
    }

    #endif
    PROV_TRC ("Posting received event to FSM...\n");
    /* Post the received event to the FSM */
    retval = prov_fsm_post_revent (ctx, (pdu_type + 1), pdata, pdatalen);

    if (API_SUCCESS != retval)
    {
        PROV_ERR ("Failed - 0x%04X\n", retval);
        /* Notify procedure completion with failure */
        /* prov_procedure_complete(ctx, PROV_ERR_UNEXPECTED_ERROR); */
    }

    /* Check if a unsegmented packet or a reassembled one */
    if (0 != ctx->rx_frag_index)
    {
        PROV_TRC ("Freeing memory used for unsegmented/reassembled packet.\n");
        /* Free Memory */
        EM_free_mem(ctx->rx_pdu);
    }

    PROV_TRC ("Cleaning up PDU information from context.\n");
    /* Clean Rx PDU related elements */
    ctx->rx_pdu = NULL;
    ctx->rx_partial_pdu_len = 0;
    ctx->rx_pdu_len = 0;
    ctx->rx_frag_index = 0;
    return;
}

#ifdef CRY_ECDH_TIMESLICE
void prov_ecdh_complete_cb (UCHAR* secret)
{
    API_RESULT retval;
    PROV_TRC ("[PROV] ECDH Calculation done.\n");
    /* Lock */
    PROV_LOCK_VOID();
    /* Post the received event to the FSM */
    retval = prov_fsm_post_revent (&prov_context, ev_prov_pubkey, secret, PROV_SECRET_SIZE);
    /* Unlock */
    PROV_UNLOCK_VOID();

    if (API_SUCCESS != retval)
    {
        PROV_ERR ("Failed - 0x%04X\n", retval);
    }
}
#endif /* CRY_ECDH_TIMESLICE */

void prov_notify
(
    PROV_CONTEXT* ctx,
    UCHAR          event_type,
    UINT16         event_result,
    void*          event_data,
    UINT16         event_datalen
)
{
    PROV_HANDLE handle;
    /* Get the handle */
    handle = (NULL != ctx) ? ctx->handle : PROV_HANDLE_INVALID;
    /* Unlock */
    PROV_UNLOCK_VOID();
    /* Notify the application */
    prov_cb (&handle, event_type, event_result, event_data, event_datalen);
    /* Lock */
    PROV_LOCK_VOID();
}


void prov_procedure_complete (PROV_CONTEXT* ctx, UINT16 status)
{
    PROV_CONTEXT lctx;
    UCHAR reason;
    API_RESULT retval;

    /* Is the completion due to timeout? */
    if (PROV_PROCEDURE_TIMEOUT == status)
    {
        /* Close the link with timeout reason */
        prov_link_close(ctx, PROV_CLOSE_REASON_TIMEOUT);
    }
    else
    {
        /* Is role Device? */
        if (MS_TRUE == PROV_IS_ROLE_DEVICE(ctx))
        {
            /* Yes, Send failure in case of status not successful, and if link is open */
            if ((API_SUCCESS != status) && (MS_TRUE == PROV_IS_LINK_OPEN(ctx)))
            {
                prov_send_failure (ctx, (UCHAR)status);
                /*
                    TODO:
                    The application is not notified of the actual error happening. Should check
                    on how this can be conveyed.
                */
                return;
            }
        }
        else
        {
            /* Set the close reason */
            reason = (API_SUCCESS != status) ? PROV_CLOSE_REASON_FAIL : PROV_CLOSE_REASON_SUCCESS;
            /* No, Close the link */
            prov_link_close (ctx, reason);
        }
    }

    /* Store the context to notify */
    lctx = *ctx;

    /* Free the retransmission timer if running */
    if (EM_TIMER_HANDLE_INIT_VAL != ctx->rtx_timer_handle)
    {
        EM_stop_timer(&ctx->rtx_timer_handle);
    }

    /* Free the prov complete timer if running */
    if (EM_TIMER_HANDLE_INIT_VAL != ctx->proc_timer_handle)
    {
        EM_stop_timer(&ctx->proc_timer_handle);
    }

    /* Free the procedure timer if running */
    if (EM_TIMER_HANDLE_INIT_VAL != ctx->prov_timer_handle)
    {
        EM_stop_timer(&ctx->prov_timer_handle);
    }

    if (API_SUCCESS == status)
    {
        /* Save Device Key with Access Layer on Successful Provisioning */
        retval = MS_access_cm_add_device_key
                 (
                     ctx->dev_key,
                     ctx->r_uaddr,
                     ctx->r_num_elements
                 );
        /* TODO: Log Retval */
    }

    /* Free the context */
    prov_free_context (ctx);
    /* Notify the application of procedure complete */
    prov_notify (&lctx, PROV_EVT_PROVISIONING_COMPLETE, status, NULL, 0);
}


void prov_send_failure (PROV_CONTEXT* ctx, UCHAR reason)
{
    UCHAR pdu[2];
    UCHAR marker;
    PROV_TRC ("Sending Prov. Failure\n");
    /* Initialize Marker and PDU */
    marker = 0;
    EM_mem_set(pdu, 0x00, sizeof (pdu));
    /* Pack the PDU Type */
    pdu[marker++] = PROV_PDU_TYPE_FAILED;
    /* Set the failure reason */
    pdu[marker++] = reason;
    /* Call to frame the header */
    prov_framensend_pdu(ctx, pdu, marker, SL_0_PROV_ERROR);
}


API_RESULT prov_link_open(PROV_CONTEXT* ctx)
{
    API_RESULT retval;
    MS_BUFFER buffer;
    UCHAR pdu[(4 + 1) + 1 + MS_DEVICE_UUID_SIZE];
    UCHAR marker;
    /* Initialize Marker and PDU */
    marker = 0;
    EM_mem_set(pdu, 0x00, sizeof (pdu));
    /* Pack the link ID and transaction ID */
    MS_PACK_BE_4_BYTE(&pdu[marker], &ctx->link_id);
    marker += 4;
    /* Transaction ID is 'zero' for bearer control PDUs */
    marker++;
    /* Pack the OpenReq control Information */
    pdu[marker++] = 0x03; /* 0x00 (OpenReq)| 0x03 (Control bits[11]) */
    /* Pack the device UUID */
    EM_mem_copy(&pdu[marker], ctx->uuid, MS_DEVICE_UUID_SIZE);
    marker += MS_DEVICE_UUID_SIZE;
    buffer.payload = pdu;
    buffer.length = marker;
    /* Call to create the bearer channel */
    retval = MS_brr_send_pdu
             (
                 &ctx->brr_info->handle,
                 BRR_TYPE_PB_ADV,
                 &buffer
             );

    if (API_SUCCESS == retval)
    {
        /* Start the ACK tracking retransmission timer */
        retval = EM_start_timer
                 (
                     &ctx->rtx_timer_handle,
                     EM_TIMEOUT_MILLISEC | PROV_PDU_RTX_TIMEOUT_SEC,
                     prov_link_ack_timeout_handler,
                     (void*)&ctx,
                     sizeof(void*)
                 );

        if (API_SUCCESS != retval)
        {
            PROV_ERR("Failed to start LinkOpen Retx Timer - 0x%04X\n", retval);
        }
    }

    return retval;
}

API_RESULT prov_link_accept(PROV_CONTEXT* ctx)
{
    API_RESULT retval;
    MS_BUFFER buffer;
    UCHAR pdu[(4 + 1) + 1];
    UCHAR marker;
    /* Initialize Marker and PDU */
    marker = 0;
    EM_mem_set(pdu, 0x00, sizeof (pdu));
    /*
        Pack the link ID and transaction ID.
        This would be extracted in the request received
    */
    MS_PACK_BE_4_BYTE(&pdu[marker], &ctx->link_id);
    marker += 4;
    /* Transaction ID is 'zero' for bearer control PDUs */
    marker++;
    /* Pack the OpenAck control Information */
    pdu[marker++] = 0x07; /* 0x01 (OpenAck) | 0x03 (Control bits[11]) */
    buffer.payload = pdu;
    buffer.length = marker;
    /* Send the message over the bearer */
    retval = MS_brr_send_pdu
             (
                 &ctx->brr_info->handle,
                 BRR_TYPE_PB_ADV,
                 &buffer
             );
    return retval;
}

API_RESULT prov_link_close(PROV_CONTEXT* ctx, UCHAR reason)
{
    API_RESULT retval;
    MS_BUFFER buffer;
    UCHAR pdu[(4 + 1) + 2];
    UCHAR marker;

    /* If link was not established as part of context, return */
    if (MS_TRUE != PROV_IS_LINK_OPEN(ctx))
    {
        return PROV_INVALID_STATE;
    }

    /* Initialize Marker and PDU */
    marker = 0;
    EM_mem_set(pdu, 0x00, sizeof (pdu));
    /*
        Pack the link ID and transaction ID.
        This would be extracted in the request received
    */
    MS_PACK_BE_4_BYTE(&pdu[marker], &ctx->link_id);
    marker += 4;
    /* Transaction ID is 'zero' for bearer control PDUs */
    marker++;
    /* Pack the OpenAck control Information */
    pdu[marker++] = 0x0B; /* 0x02 (OpenAck) | 0x03 (Control bits[11]) */
    /* Pack the Reason */
    pdu[marker++] = reason;
    buffer.payload = pdu;
    buffer.length = marker;
    /*
        The specification says here to atleast transmit link close for
        3 times as this is an unacknowledged packet
    */
    retval = MS_brr_send_pdu
             (
                 &ctx->brr_info->handle,
                 BRR_TYPE_PB_ADV,
                 &buffer
             );
    retval = MS_brr_send_pdu
             (
                 &ctx->brr_info->handle,
                 BRR_TYPE_PB_ADV,
                 &buffer
             );
    retval = MS_brr_send_pdu
             (
                 &ctx->brr_info->handle,
                 BRR_TYPE_PB_ADV,
                 &buffer
             );
    /* Update the link state */
    PROV_RESET_LINK_OPEN(ctx);
    return retval;
}

API_RESULT prov_process_event(PROV_CONTEXT* ctx, UCHAR evt, UCHAR src)
{
    API_RESULT retval;

    if (BRR_TYPE_PB_ADV == ctx->bearer)
    {
        /* Update next event to be posted upon Acknowledgement */
        PROV_SET_ACK_EVENT(ctx, evt, src);
        retval = API_SUCCESS;
    }
    else
    {
        /* Post the event */
        retval = prov_fsm_post_event (ctx, evt, src, NULL, 0);
    }

    return retval;
}

API_RESULT prov_framensend_pdu (PROV_CONTEXT* ctx, UCHAR* packet, UINT16 length, UINT32 nstate)
{
    API_RESULT retval;

    /* Decide the Frame and Send Wrapper based on the Bearer Type */
    switch (ctx->bearer)
    {
    case BRR_TYPE_PB_ADV:
        retval = prov_framensend_pb_adv_pdu (ctx, packet, length);

        if (API_SUCCESS == retval)
        {
            /* Set the next state upon acknowledgement */
            PROV_SET_ACK_STATE(ctx, nstate);
        }

        break;

    case BRR_TYPE_PB_GATT:
        retval = prov_framensend_pb_gatt_pdu (ctx, packet, length);

        if (API_SUCCESS == retval)
        {
            /* Update to the next expected state */
            PROV_CONTEXT_SET_STATE(ctx, nstate);
        }

        break;

    default:
        retval = API_FAILURE;
        break;
    }

    return retval;
}

API_RESULT prov_framensend_pb_adv_pdu(PROV_CONTEXT* ctx, UCHAR* packet, UINT16 length)
{
    UCHAR pdu[PROV_MAX_PB_ADV_PDU_SIZE];
    UCHAR header[PROV_MAX_PB_ADV_HDR_SIZE];
    MS_BUFFER buffer;
    API_RESULT retval;
    UCHAR control, fcs;
    UINT16 rlen, plen, len, marker;
    UCHAR fi, fnum;
    /* Initialize the locals */
    fi = 0;
    rlen = 0;
    marker = 0;

    /* Return if previous transmission is still pending */
    if (EM_TIMER_HANDLE_INIT_VAL != ctx->rtx_timer_handle)
    {
        /* Should not get here! */
        PROV_ERR("Framesend failed. Pending Transmission...\n");
        return PROV_INVALID_STATE;
    }

    retval = API_SUCCESS;
    /* Have the TX delay */
    prov_tx_delay(ctx);
    /*
        Pack the link ID and transaction ID.
        This would be extracted in the request received
    */
    MS_PACK_BE_4_BYTE(header, &ctx->link_id);
    header[4] = ctx->tx_txn_id;

    /* Transaction ID will be incremented after ACK is received */

    /*
        Next 4 bytes for header
        ------------------------------------------
        ||  NFg |0|0||   Total Len    ||   FCS  ||
        ------------------------------------------
    */

    /* Start transmitting */
    while (rlen < length)
    {
        marker = 0;
        /* Frame the PDU */
        EM_mem_copy(&pdu[marker], header, PROV_MAX_PB_ADV_HDR_SIZE);
        marker += PROV_MAX_PB_ADV_HDR_SIZE;

        /* Is it the first fragment? */
        if (0 == fi)
        {
            /* Yes, fill the data header */
            /* Set the control octet with number of fragments */
            /* fnum = ((length + 2) / PROV_PAYLOAD_SIZE_FRAG_N) + 1; */
            fnum = (UCHAR)((length + 2) / PROV_PAYLOAD_SIZE_FRAG_N);
            control = (fnum << 2);
            /* Calculate the FCS */
            prov_generate_fcs(packet, length, &fcs);
            /* Update the PDU */
            pdu[marker++] = control;
            MS_PACK_BE_2_BYTE(&pdu[marker], &length);
            marker += 2;
            pdu[marker++] = fcs;
            /* Get the fragment length */
            len = (PROV_PAYLOAD_SIZE_FRAG_1 < (length - rlen)) ? PROV_PAYLOAD_SIZE_FRAG_1 : (length - rlen);
            /* Get Payload length */
            plen = len;
        }
        else
        {
            /* Set the control octet with fragment index */
            control = (fi << 2) | 0x02;
            /* Update the PDU */
            pdu[marker++] = control;
            /* Get the fragment length */
            len = (PROV_PAYLOAD_SIZE_FRAG_N < (length - rlen)) ? PROV_PAYLOAD_SIZE_FRAG_N : (length - rlen);
            /* Get Payload length */
            plen = len;
        }

        /* Copy the payload */
        EM_mem_copy(&pdu[marker], (packet + rlen), plen);
        marker += plen;
        rlen += plen;
        fi ++;
        /* Send the buffer */
        buffer.payload = pdu;
        buffer.length = marker;
        retval = MS_brr_send_pdu
                 (
                     &ctx->brr_info->handle,
                     BRR_TYPE_PB_ADV,
                     &buffer
                 );
    }

    if (API_SUCCESS == retval)
    {
        /* Save the packet to context */
        EM_mem_copy(ctx->tx_pdu, packet, length);
        ctx->tx_pdu_len = length;
        /* Start the ACK tracking retransmission timer */
        retval = EM_start_timer
                 (
                     &ctx->rtx_timer_handle,
                     EM_TIMEOUT_MILLISEC | PROV_PDU_RTX_TIMEOUT_SEC,
                     prov_pdu_ack_timeout_handler,
                     (void*)&ctx,
                     sizeof(void*)
                 );

        if (API_SUCCESS != retval)
        {
            PROV_ERR("Failed to start PDU Retx Timer - 0x%04X\n", retval);
        }

        /* Restart the Procedure timer */
        prov_restart_proc_timer(ctx);
    }

    return retval;
}

void prov_tx_delay(PROV_CONTEXT* ctx)
{
    MS_IGNORE_UNUSED_PARAM(ctx);
}

void prov_rx_delay(PROV_CONTEXT* ctx)
{
    MS_IGNORE_UNUSED_PARAM(ctx);
}

void prov_complete_timeout_handler(void* args, UINT16 size)
{
    PROV_CONTEXT* ctx;
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    PROV_LOCK_VOID();
    /* Reference the context */
    ctx = (PROV_CONTEXT*)(*((UINT32*)args));
    PROV_TRC("Provisining Complete Timeout fired for context - %d\n", ctx->handle);
    /* Reset the timer handle */
    ctx->proc_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Notify procedure completion */
    prov_procedure_complete(ctx, API_SUCCESS);
    /* Unlock */
    PROV_UNLOCK_VOID();
}

void prov_link_ack_timeout_handler(void* args, UINT16 size)
{
    PROV_CONTEXT* ctx;
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    PROV_LOCK_VOID();
    /* Reference the context */
    ctx = (PROV_CONTEXT*)(*((UINT32*)args));
    PROV_INF("Provisining Link Open Timeout fired for context - %d\n", ctx->handle);

    if(PROV_IS_ROLE_PROVISIONER(ctx))
    {
        /* If still no ack received, retransmit link open */
        if (PROV_STATE_LINKOPEN == PROV_CONTEXT_GET_STATE(ctx))
        {
            /* Reset the timer handle */
            ctx->rtx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

            /* Is number of PDU retry attempts expired? */
            if (PROV_PDU_RTX_COUNT == ctx->rtx_count)
            {
                /* Yes, abort provisioning procedure */
                PROV_ERR("Max number of retries attempted. Aborting procedure...\n");
                /* Notify application */
                prov_procedure_complete(ctx, PROV_PROCEDURE_TIMEOUT);
            }
            else
            {
                ctx->rtx_count++;
                prov_link_open(ctx);
            }
        }
    }
    else
    {
        /* Reset the timer handle */
        link_open_ack_timer_handle= EM_TIMER_HANDLE_INIT_VAL;
        /* Notify application */
        prov_procedure_complete(ctx, PROV_PROCEDURE_TIMEOUT);
    }

    /* Unlock */
    PROV_UNLOCK_VOID();
}

void prov_pdu_ack_timeout_handler(void* args, UINT16 size)
{
    PROV_CONTEXT* ctx;
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    PROV_LOCK_VOID();
    /* Reference the context */
    ctx = (PROV_CONTEXT*)(*((UINT32*)args));
    PROV_TRC("Provisining PDU Timeout fired for context - %d\n", ctx->handle);
    /* Reset the timer handle */
    ctx->rtx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

    /* Is number of PDU retry attempts expired? */
    if (PROV_PDU_RTX_COUNT == ctx->rtx_count)
    {
        /* Yes, abort provisioning procedure */
        PROV_ERR("Max number of retries attempted. Aborting procedure...\n");
        /* Notify application */
        prov_procedure_complete(ctx, PROV_PROCEDURE_TIMEOUT);
    }
    else
    {
        /* Increment the retry count */
        ctx->rtx_count++;
        /* Resend the PDU */
        prov_framensend_pdu(ctx, ctx->tx_pdu, ctx->tx_pdu_len, ctx->ackstate);
    }

    /* Unlock */
    PROV_UNLOCK_VOID();
}

void prov_proc_timeout_handler(void* args, UINT16 size)
{
    PROV_CONTEXT* ctx;
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    PROV_LOCK_VOID();
    /* Reference the context */
    ctx = (PROV_CONTEXT*)(*((UINT32*)args));
    PROV_TRC("Provisining Procedure Timeout fired for context - %d\n", ctx->handle);
    /* Reset the timer handle */
    ctx->prov_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Notify application */
    prov_procedure_complete(ctx, PROV_PROCEDURE_TIMEOUT);
    /* Unlock */
    PROV_UNLOCK_VOID();
}

API_RESULT prov_framensend_pb_gatt_pdu(PROV_CONTEXT* ctx, UCHAR* packet, UINT16 length)
{
    MS_BUFFER buffer;
    API_RESULT retval;
    /* Send the buffer */
    buffer.payload = packet;
    buffer.length  = length;
    retval = MS_brr_send_pdu
             (
                 &ctx->brr_info->handle,
                 (BRR_TYPE)(BRR_TYPE_PB_GATT | BRR_SUBTYPE_GATT_PROV_T_MASK),
                 &buffer
             );
    return retval;
}

void prov_framensend_ack(PROV_CONTEXT* ctx)
{
    UCHAR pdu[PROV_MAX_PB_ADV_PDU_SIZE];
    MS_BUFFER buffer;
    API_RESULT retval;
    UCHAR marker;
    /* Initialize locals */
    marker = 0;
    /*
        Pack the link ID and transaction ID.
        This would be extracted in the request received
    */
    MS_PACK_BE_4_BYTE(pdu, &ctx->link_id);
    pdu[4] = ctx->rx_txn_id;
    marker += PROV_MAX_PB_ADV_HDR_SIZE;
    /*
        Next 1 byte for header
        --------------
        ||  NFg |0|0||
        --------------
    */
    pdu[marker++] = 0x01;
    /* Send the buffer */
    buffer.payload = pdu;
    buffer.length = marker;
    /*
        TBD: The brr_type shall always be only PB-ADV as Generic Layer
            Operations are disallowed in other brr_types.
    */
    retval = MS_brr_send_pdu
             (
                 &ctx->brr_info->handle,
                 BRR_TYPE_PB_ADV,
                 &buffer
             );
}

API_RESULT prov_calc_confirm(PROV_CONTEXT* ctx, UCHAR confirm)
{
    UCHAR conf_input[PROV_PDU_INVITEVAL_LEN + PROV_PDU_CAPABVAL_LEN + PROV_PDU_STARTVAL_LEN + (PROV_PUBKEY_SIZE << 1)];
    UCHAR conf_key[PROV_K1_KEY_SIZE];
    UCHAR conf_salt[PROV_S1_KEY_SIZE];
    UCHAR aes_cmac_text[PROV_RANDVAL_SIZE + PROV_AUTHVAL_SIZE];
    UCHAR tconfval[PROV_CONFVAL_SIZE];
    UCHAR prck[] = { 'p', 'r', 'c', 'k' };
    INT32 ret;
    /*
        Calculate the Confirm value:
        AES-CMAC(ConfKey, RandProv || AuthVal), for Provisioner
        AES-CMAC(ConfKey, RandDev || AuthVal), for Device

        Where,
        ;ConfKey = k1 (ProvgMasterKey, "prct", "prck")
        ConfKey = k1 (ECDHKey, ConfSalt, "prck")
        ConfSalt = s1(ConfInputs)
        ;ProvgMasterKey = k1 (ProvgMasterSalt, ECDHKey, "prmk")
        ;ProvgMasterSalt = k1(0x00000000000000000000000000000000, ConfInputs, "prms")
        ECDHKey = ecdh(PeerPubKey, PvtKey)
        ;ConfInputs = ProvCapVal || ProvStartVal || PubkeyProv || PubkeyDev
        ConfInputs = ProvInviteVal || ProvCapVal || ProvStartVal || PubkeyProv || PubkeyDev
    */
    EM_mem_copy(conf_input, ctx->inviteval, PROV_PDU_INVITEVAL_LEN);
    EM_mem_copy((conf_input + PROV_PDU_INVITEVAL_LEN), ctx->capval, PROV_PDU_CAPABVAL_LEN);
    EM_mem_copy((conf_input + PROV_PDU_INVITEVAL_LEN + PROV_PDU_CAPABVAL_LEN), ctx->startval, PROV_PDU_STARTVAL_LEN);

    /* Frame the ConfInputs */
    if (PROV_ROLE_DEVICE == ctx->role)
    {
        EM_mem_copy
        (
            (conf_input + PROV_PDU_INVITEVAL_LEN + PROV_PDU_CAPABVAL_LEN + PROV_PDU_STARTVAL_LEN),
            ctx->rpubkey,
            PROV_PUBKEY_SIZE
        );
        EM_mem_copy
        (
            (conf_input + PROV_PDU_INVITEVAL_LEN + PROV_PDU_CAPABVAL_LEN + PROV_PDU_STARTVAL_LEN + PROV_PUBKEY_SIZE),
            prov_pubkey,
            PROV_PUBKEY_SIZE
        );
    }
    else
    {
        EM_mem_copy
        (
            (conf_input + PROV_PDU_INVITEVAL_LEN + PROV_PDU_CAPABVAL_LEN + PROV_PDU_STARTVAL_LEN),
            prov_pubkey,
            PROV_PUBKEY_SIZE
        );
        EM_mem_copy
        (
            (conf_input + PROV_PDU_INVITEVAL_LEN + PROV_PDU_CAPABVAL_LEN + PROV_PDU_STARTVAL_LEN + PROV_PUBKEY_SIZE),
            ctx->rpubkey,
            PROV_PUBKEY_SIZE
        );
    }

    /* Calculate the ConfirmationSalt */
    ms_stbx_s1(conf_input, sizeof(conf_input), conf_salt);
    /* Store the COnfirmationSalt to be used in Key Generations */
    EM_mem_copy(ctx->conf_salt, conf_salt, sizeof(conf_salt));
    #ifdef MESH_CRYPTO_IO_DEBUG_DUMP
    /* TBD: Need to move this into a separate debug location */
    PROV_TRC("\n Confirm Salt caluated :\n");
    PROV_debug_dump_bytes(ctx->conf_salt, sizeof(ctx->conf_salt));
    #endif /* MESH_CRYPTO_IO_DEBUG_DUMP */
    /* Calculate the ConfKey */
    ms_stbx_k1
    (
        ctx->ecdh_key,
        sizeof(ctx->ecdh_key),
        conf_salt,
        prck,
        sizeof(prck),
        conf_key
    );
    #ifdef MESH_CRYPTO_IO_DEBUG_DUMP
    /* TBD: Need to move this into a separate debug location */
    PROV_TRC("\n K1 Conf Key caluated :\n");
    PROV_debug_dump_bytes(conf_key, sizeof(conf_key));
    #endif /* MESH_CRYPTO_IO_DEBUG_DUMP */

    /* Check if to confirm */
    if (0x00 == confirm)
    {
        /* Form the plain text for AES-CMAC for Confirm Value calculation */
        EM_mem_copy(aes_cmac_text, ctx->lrandval, PROV_RANDVAL_SIZE);
        EM_mem_copy((aes_cmac_text + PROV_RANDVAL_SIZE), ctx->authval, PROV_AUTHVAL_SIZE);
        /* Calculate the Confirm Value */
        cry_aes_128_cmac_sign_be
        (
            aes_cmac_text,
            sizeof(aes_cmac_text),
            conf_key,
            ctx->lconfval,
            sizeof(ctx->lconfval),
            ret
        );
        #ifdef MESH_CRYPTO_IO_DEBUG_DUMP
        PROV_TRC("\n *** LOCAL CONFIRM GENERATION *** \n");
        PROV_TRC("\n Confirm val Inputs :\n");
        PROV_debug_dump_bytes(aes_cmac_text, sizeof(aes_cmac_text));
        PROV_TRC("\n Confirm val caluated :\n");
        PROV_debug_dump_bytes(ctx->lconfval, sizeof(ctx->lconfval));
        #endif /* MESH_CRYPTO_IO_DEBUG_DUMP */
    }
    else
    {
        /* Form the plain text for AES-CMAC for Confirm Value calculation */
        EM_mem_copy(aes_cmac_text, ctx->rrandval, PROV_RANDVAL_SIZE);
        EM_mem_copy((aes_cmac_text + PROV_RANDVAL_SIZE), ctx->authval, PROV_AUTHVAL_SIZE);
        /* Calculate the Confirm Value */
        cry_aes_128_cmac_sign_be
        (
            aes_cmac_text,
            sizeof(aes_cmac_text),
            conf_key,
            tconfval,
            sizeof(tconfval),
            ret
        );
        #ifdef MESH_CRYPTO_IO_DEBUG_DUMP
        /* TBD: Need to move this into a separate debug location */
        PROV_TRC("\n *** REMOTE CONFIRM VERIFICATION *** \n");
        PROV_TRC("\n Confirm val Inputs :\n");
        PROV_debug_dump_bytes(aes_cmac_text, sizeof(aes_cmac_text));
        PROV_TRC("\n Confirm val caluated :\n");
        PROV_debug_dump_bytes(tconfval, sizeof(tconfval));
        #endif /* MESH_CRYPTO_IO_DEBUG_DUMP */

        /* Check if confirm value matches with remote received */
        if (EM_mem_cmp(ctx->rconfval, tconfval, PROV_CONFVAL_SIZE))
        {
            return API_FAILURE;
        }
    }

    return API_SUCCESS;
}

void prov_generate_authkeys(PROV_CONTEXT* ctx)
{
    UCHAR prov_info[(PROV_RANDVAL_SIZE << 1) + PROV_S1_KEY_SIZE];
    UCHAR prov_salt[PROV_S1_KEY_SIZE];
    UCHAR nonce[PROV_K1_KEY_SIZE];
    UCHAR prsk[] = { 'p', 'r', 's', 'k' };
    UCHAR prsn[] = { 'p', 'r', 's', 'n' };
    UCHAR prdk[] = { 'p', 'r', 'd', 'k' };
    PROV_TRC("Generate Auth Keys\n");
    /* Form the ProvSalt INput */
    EM_mem_copy (prov_info, ctx->conf_salt, PROV_S1_KEY_SIZE);

    if (MS_TRUE == PROV_IS_ROLE_DEVICE(ctx))
    {
        EM_mem_copy((prov_info + PROV_S1_KEY_SIZE), ctx->rrandval, PROV_RANDVAL_SIZE);
        EM_mem_copy(prov_info + PROV_S1_KEY_SIZE + PROV_RANDVAL_SIZE, ctx->lrandval, PROV_RANDVAL_SIZE);
    }
    else
    {
        EM_mem_copy((prov_info + PROV_S1_KEY_SIZE), ctx->lrandval, PROV_RANDVAL_SIZE);
        EM_mem_copy(prov_info + PROV_S1_KEY_SIZE + PROV_RANDVAL_SIZE, ctx->rrandval, PROV_RANDVAL_SIZE);
    }

    PROV_TRC("Calc Prov Salt\n");
    /* Calculate the ProvisioningSalt */
    ms_stbx_s1(prov_info, sizeof(prov_info), prov_salt);
    PROV_TRC("Calc Session Key\n");
    /* Calculate the SessionKey */
    ms_stbx_k1
    (
        ctx->ecdh_key, sizeof(ctx->ecdh_key),
        prov_salt,
        prsk, sizeof(prsk),
        ctx->session_key
    );
    PROV_TRC("Calc Session Nonce\n");
    /* Calculate the SessionNonce */
    ms_stbx_k1
    (
        ctx->ecdh_key, sizeof(ctx->ecdh_key),
        prov_salt,
        prsn, sizeof(prsn),
        nonce
    );
    PROV_TRC("Calc Dev Key\n");
    /* Calculate the DevKey */
    ms_stbx_k1
    (
        ctx->ecdh_key, sizeof(ctx->ecdh_key),
        prov_salt,
        prdk, sizeof(prdk),
        ctx->dev_key
    );
    /* Get the 13 least significant ocetes as session nonce */
    EM_mem_copy(ctx->nonce, (nonce + 3), PROV_NONCE_SIZE);
}

PROV_CONTEXT* prov_alloc_context (PROV_HANDLE* phandle)
{
    /*
        TODO:
        Have proper allocation of free context. For now just assigning the
        single context defined
    */
//    *phandle = 0;
    /* Reset the context */
    EM_mem_set(&prov_context, 0, sizeof(PROV_CONTEXT));
    /* Initialize the context state and other required variables */
    prov_context.state = PROV_STATE_INITIALIZED;
    prov_context.handle = *phandle;
    prov_context.rx_txn_id = 0xFF;
    prov_context.proc_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    prov_context.prov_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    prov_context.rtx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    return &prov_context;
}

void prov_free_context (PROV_CONTEXT* ctx)
{
    /* Reset the context */
    EM_mem_set(ctx, 0, sizeof(PROV_CONTEXT));
    ctx->state = PROV_STATE_UNINITIALIZED;
    ctx->link_id = PROV_STATE_UNINITIALIZED;
    ctx->proc_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    ctx->prov_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    ctx->rtx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
}

PROV_CONTEXT* prov_findcontext_by_handle (PROV_HANDLE* phandle)
{
    /*
        TODO:
        Have proper referencing of context. For now just assigning the
        single context defined
    */
    if (prov_context.handle == *phandle)
    {
        return &prov_context;
    }

    return NULL;
}

PROV_CONTEXT* prov_findcontext_by_linkid (UINT32 link_id)
{
    /*
        TODO:
        Have proper finding of context. For now just assigning the
        single context defined
    */
    if ((BRR_TYPE_PB_ADV == prov_context.bearer) &&
            (link_id == prov_context.link_id))
    {
        return &prov_context;
    }

    return NULL;
}

PROV_CONTEXT* prov_findcontext_by_uuid (UCHAR* uuid)
{
    /*
        TODO:
        Have proper finding of context. For now just assigning the
        single context defined
    */
    if ((BRR_TYPE_PB_ADV == prov_context.bearer) &&
            (0 == EM_mem_cmp (uuid, prov_context.uuid, MS_DEVICE_UUID_SIZE)))
    {
        return &prov_context;
    }

    return NULL;
}

PROV_CONTEXT* prov_findcontext_by_brr(PROV_BRR_INFO* brr)
{
    /*
        TODO:
        Have proper finding of context. For now just assigning the
        single context defined
    */
    if ((NULL != prov_context.brr_info) &&
            (0 == EM_mem_cmp(brr, prov_context.brr_info, sizeof (PROV_BRR_INFO))))
    {
        return &prov_context;
    }

    return NULL;
}

