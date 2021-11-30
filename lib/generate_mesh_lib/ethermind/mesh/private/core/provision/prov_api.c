
/**
    \file prov_api.c


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "prov_fsm_engine.h"
#include "prov_extern.h"

/* --------------------------------------------- External Global Variables */
//extern PROV_BRR_ADV_COUNT adv_message_count;


/* --------------------------------------------- Exported Global Variables */
EM_timer_handle unprovisionbcon_interleave_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
UCHAR unprovisionbcon_interleave_handle;
static UINT16       adv_interleave_timeout,gatt_interleave_timeout;



/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */

/**
    \brief

    \par Description


    \param pcapab
    \param cb

    \return void
*/
API_RESULT MS_prov_register
(
    /* IN */ PROV_CAPABILITIES_S* pcapab,
    /* IN */ PROV_UI_NOTIFY_CB     cb
)
{
    PROV_TRC ("-> MS_prov_register\n");

    /* Validate input parameters */
    if (NULL == cb)
    {
        PROV_ERR ("Invalid callback parameter.\n");
        return PROV_INVALID_PARAMETER;
    }

    PROV_TRC ("Registering capabilities and callback.\n");
    /* Lock */
    PROV_LOCK();
    /* Store the Capabilities supported */
    prov_cap = pcapab;
    /* Store the application callback to be used during provisioning */
    prov_cb = cb;
    EM_stop_timer(&unprovisionbcon_interleave_timer_handle);
//    unprovisionbcon_interleave_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Lock */
    PROV_UNLOCK();
    PROV_TRC ("<- MS_prov_register\n");
    return API_SUCCESS;
}

void ms_prov_beacon_interleave(void* args, UINT16 size)
{
    PROV_DEVICE_S*   pdevice;
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(size);
    unprovisionbcon_interleave_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Lock */
    PROV_LOCK_VOID();
    /* Reference the context */
    pdevice = (PROV_DEVICE_S*)(*((UINT32*)args));
    unprovisionbcon_interleave_handle = (unprovisionbcon_interleave_handle == PROV_BRR_ADV)
                                        ? PROV_BRR_GATT : PROV_BRR_ADV;

    if(unprovisionbcon_interleave_handle == PROV_BRR_ADV)
    {
        retval = MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_ACTIVE);
        retval = MS_brr_bcast_unprovisioned_beacon
                 (
                     BRR_BCON_PASSIVE,
                     pdevice->uuid,
                     pdevice->oob,
                     pdevice->uri
                 );
        retval = EM_start_timer
                 (
                     &unprovisionbcon_interleave_timer_handle,
                     EM_TIMEOUT_MILLISEC | adv_interleave_timeout,
                     ms_prov_beacon_interleave,
                     (void*)&pdevice,
                     sizeof(void*)
                 );
    }
    else if(unprovisionbcon_interleave_handle == PROV_BRR_GATT)
    {
        retval = MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_PASSIVE);
        retval = MS_brr_bcast_unprovisioned_beacon
                 (
                     BRR_BCON_ACTIVE,
                     pdevice->uuid,
                     pdevice->oob,
                     pdevice->uri
                 );
        retval = EM_start_timer
                 (
                     &unprovisionbcon_interleave_timer_handle,
                     EM_TIMEOUT_MILLISEC | gatt_interleave_timeout,
                     ms_prov_beacon_interleave,
                     (void*)&pdevice,
                     sizeof(void*)
                 );
    }
}

API_RESULT MS_prov_stop_interleave_timer
(
    void
)
{
    EM_stop_timer(&unprovisionbcon_interleave_timer_handle);
}



/**
    \brief Setup the device for provisioning

    \par Description This function configures the device to get in a provisionable
    state by specifying the role, bearer and creating a context.

    \param [in] role Provisioniong role to be setup - Device or Provisioner.
    \param [in] bearer Provisioning bearer to be setup - PB-ADV or PB-GATT
    \param [in] pdevice Pointer to the device strcuture \ref PROV_DEVICE_S
    containing the UUID to be beaconed. This parameter is used only when the
    role is PROV_ROLE_DEVICE and ignored otherwise.
    \param [in] timeout The time period for which the setup shall be active.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_prov_setup
(
    /* IN */  PROV_BRR        bearer,
    /* IN */  PROV_ROLE       role,
    /* IN */  PROV_DEVICE_S* pdevice,
    /* IN */  UINT16          gatt_timeout,
    /* IN */  UINT16          adv_timeout
)
{
    INT32 ret;
    API_RESULT retval;
//    MS_IGNORE_UNUSED_PARAM(timeout);
    PROV_TRC ("-> MS_prov_setup\n");

    /* Chek if the application has registered first */
    if (NULL == prov_cb)
    {
        PROV_ERR ("Application not registered.\n");
        return PROV_INVALID_STATE;
    }

    /* Validate the role */
    if ((PROV_ROLE_PROVISIONER != role) &&
            (PROV_ROLE_DEVICE != role))
    {
        PROV_ERR ("Incorrect role in parameter.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* If device, check if capabilities are registered */
    if ((PROV_ROLE_DEVICE == role) &&
            (NULL == prov_cap))
    {
        PROV_ERR ("Capabilities need to be set first for Device setup.\n");
        return PROV_INVALID_STATE;
    }

    /* Validate the Device Information */
    if ((PROV_ROLE_DEVICE == role) &&
            (NULL == pdevice))
    {
        PROV_ERR ("Invalid Device information.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Validate the Bearer Information */
    if ((PROV_BRR_ADV > bearer) ||
            ((PROV_BRR_ADV | PROV_BRR_GATT) < bearer))
    {
        PROV_ERR ("Invalid Bearer information.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Initialize locals */
    retval = API_FAILURE;
    /* Lock */
    PROV_LOCK();
    /* TODO: If setup timer is running, means already setup in progress, stop the timer */
    /* Save the role requested that will be used in the context */
    prov_role = role;
    PROV_TRC ("Setup with Role: 0x%02X\n", role);
    /* Initialize the ECDH for a new pair of public/private keys */
    cry_ecdh_init();
    cry_ecdh_get_public_key_be(prov_pubkey, ret);

    /* Check the role */
    switch (role)
    {
    /*
        When the role is 'Device', the UUID in the device information is
        broadcasted
    */
    case PROV_ROLE_DEVICE:
//            adv_message_count.count = 0;
        /* Save the UUID that is to be beaconed out */
        EM_mem_copy (prov_uuid, pdevice->uuid, MS_DEVICE_UUID_SIZE);
        PROV_TRC ("Device UUID to beacon:\n");
        PROV_debug_dump_bytes (prov_uuid, MS_DEVICE_UUID_SIZE);

        if (PROV_BRR_ADV == bearer)
        {
            /* Setup provisioning for ADV Bearer? */
            /* Start beaconing at the bearer */
            retval = MS_brr_bcast_unprovisioned_beacon
                     (
                         BRR_BCON_PASSIVE,
                         pdevice->uuid,
                         pdevice->oob,
                         pdevice->uri
                     );
        }
        else if (PROV_BRR_GATT == bearer)
        {
            /* Setup provisioning for GATT Bearer? */
            /* Start beaconing at the bearer */
            retval = MS_brr_bcast_unprovisioned_beacon
                     (
                         BRR_BCON_ACTIVE,
                         pdevice->uuid,
                         pdevice->oob,
                         pdevice->uri
                     );
        }
        else
        {
            /* Setup provisioning for GATT&ADV Bearer? */
            /* Start the ACK tracking retransmission timer */
            adv_interleave_timeout = adv_timeout;
            gatt_interleave_timeout = gatt_timeout;
            retval = MS_brr_bcast_unprovisioned_beacon
                     (
                         BRR_BCON_PASSIVE,
                         pdevice->uuid,
                         pdevice->oob,
                         pdevice->uri
                     );
            unprovisionbcon_interleave_handle = PROV_BRR_ADV;
            retval = EM_start_timer
                     (
                         &unprovisionbcon_interleave_timer_handle,
                         EM_TIMEOUT_MILLISEC | adv_interleave_timeout,
                         ms_prov_beacon_interleave,
                         (void*)&pdevice,
                         sizeof(void*)
                     );
        }

        if (API_SUCCESS == retval)
        {
            /* Update the context state */
            PROV_SET_STATE (PROV_STATE_BEACONING);
            /* TODO: Start timer with given timeout */
        }

        break;

    /*
        When the role is 'Provisioner', scan for unprovisioned beacons at
        the bearer.
    */
    case PROV_ROLE_PROVISIONER:

        /* Setup provisioning for ADV Bearer? */
        if (PROV_BRR_ADV & bearer)
        {
            /* Start scanning for beacons at the bearer */
            retval = MS_brr_observe_beacon(BRR_BCON_PASSIVE, MS_TRUE);
        }

        if (PROV_BRR_GATT & bearer)
        {
            /* Start scanning for beacons at the bearer */
            retval = MS_brr_observe_beacon(BRR_BCON_ACTIVE, MS_TRUE);
        }

        if (API_SUCCESS == retval)
        {
            /* Update the context state */
            PROV_SET_STATE (PROV_STATE_SCANNING);
            /* TODO: Start timer with given timeout */
        }

        break;

    default:
        /* Should not get here */
        retval = PROV_INVALID_PARAMETER;
        break;
    }

    PROV_TRC ("<- MS_prov_setup\n");
    /* Unlock */
    PROV_UNLOCK();
    return retval;
}


/**
    \brief

    \par Description


    \param pdevice
    \param attention
    \param phandle

    \return void
*/
API_RESULT MS_prov_bind
(
    /* IN */  PROV_BRR        bearer,
    /* IN */  PROV_DEVICE_S* pdevice,
    /* IN */  UCHAR           attention,
    /* OUT */ PROV_HANDLE*    phandle
)
{
    PROV_CONTEXT* ctx;
    API_RESULT retval;
    UCHAR i;
    PROV_TRC("-> MS_prov_bind\n");

    /* Validate the inputs */
    if ((NULL == phandle) ||
            (NULL == pdevice))
    {
        PROV_ERR ("Incorrect input parameters\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Validate the bearer */
    if ((PROV_BRR_ADV != bearer) &&
            (PROV_BRR_GATT != bearer))
    {
        PROV_ERR("Incorrect bearer in parameter.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Initialize locals */
    retval = API_SUCCESS;
    /* Lock */
    PROV_LOCK();

    /* Verify state to be either Scanning or Initialized */
    if ((PROV_STATE_SCANNING != PROV_GET_STATE()) &&
            (PROV_STATE_BEACONING != PROV_GET_STATE()) &&
            (PROV_STATE_INITIALIZED != PROV_GET_STATE()))
    {
        PROV_ERR("Incorrect state to bind!\n");
        /* Unlock */
        PROV_UNLOCK();
        return PROV_INVALID_STATE;
    }

    /* Allocate a provisioning context */
    ctx = prov_alloc_context(phandle);

    if (NULL == ctx)
    {
        PROV_ERR("Failed to allocate context.\n");
        /* Unlock */
        PROV_UNLOCK();
        return PROV_CONTEXT_ALLOC_FAILED;
    }

    /* Save the role */
    ctx->role = prov_role;

    if (PROV_IS_ROLE_PROVISIONER(ctx))
    {
        /* Save the peer UUID that is to be bound */
        EM_mem_copy(ctx->uuid, pdevice->uuid, MS_DEVICE_UUID_SIZE);
        /* Update the attention duration */
        ctx->attention = attention;
    }
    else
    {
        /* Save the local UUID that is to be bound */
        EM_mem_copy(ctx->uuid, prov_uuid, MS_DEVICE_UUID_SIZE);
    }

    /* Initialize bearer handle in context */
    ctx->brr_info = NULL;

    /* Check the bearer type */
    switch (bearer)
    {
    case PROV_BRR_ADV:

        /* Yes, Is ADV bearer available? */
        if (BRR_HANDLE_INVALID != prov_brr[0].handle)
        {
            /* Yes. Store the bearer handle in context */
            ctx->brr_info = &prov_brr[0];
        }
        else
        {
            retval = PROV_BEARER_ASSERT_FAILED;
            break;
        }

        /* Save the bearer to bond for the context */
        ctx->bearer = BRR_TYPE_PB_ADV;

        if (PROV_IS_ROLE_PROVISIONER(ctx))
        {
            PROV_TRC("Request to establish link on ADV bearer...\n");
            PROV_TRC("Link open to Device UUID:\n");
            PROV_debug_dump_bytes(ctx->uuid, MS_DEVICE_UUID_SIZE);
            /* Initialize the Link ID */
            cry_rand_generate((UCHAR*)(&ctx->link_id), sizeof(ctx->link_id));
            /* Yes. Start link open procedure at the bearer */
            retval = prov_link_open(ctx);

            if (API_SUCCESS == retval)
            {
                /* Update the context state */
                PROV_CONTEXT_SET_STATE(ctx, PROV_STATE_LINKOPEN);
            }
        }

        break;

    case PROV_BRR_GATT:

        /* Check if any open GATT bearer is bound with the given UUID */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES); i++)
        {
            if (BRR_HANDLE_INVALID != prov_brr[i].handle)
            {
                /* Store the bearer handle in context */
                ctx->brr_info = &prov_brr[i];
                break;
            }
        }

        if (MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES) == i)
        {
            retval = PROV_BEARER_ASSERT_FAILED;
            break;
        }

        /* Save the bearer to bond for the context */
        ctx->bearer = BRR_TYPE_PB_GATT;
        /* Set link establishment */
        PROV_SET_LINK_OPEN(ctx);
        /* Update the state */
        PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_IDLE);

        if (PROV_IS_ROLE_PROVISIONER(ctx))
        {
            PROV_TRC("Sending Provisioning Invite on GATT bearer...\n");
            PROV_TRC("Prov Invite to Device UUID:\n");
            PROV_debug_dump_bytes(ctx->uuid, MS_DEVICE_UUID_SIZE);
            /* Send Provisioning Invite */
            retval = prov_fsm_post_levent(ctx, ev_prov_invite, NULL, 0);
        }

        break;

    default:
        retval = API_FAILURE;
        break;
    }

    if (API_SUCCESS != retval)
    {
        PROV_ERR("Failed - 0x%04X\n", retval);
        /* Free the context */
        prov_free_context(ctx);
    }

    PROV_TRC("<- MS_prov_bind\n");
    /* Unlock */
    PROV_UNLOCK();
    return retval;
}


/**
    \brief

    \par Description


    \param phandle
    \param reason

    \return void
*/
API_RESULT MS_prov_abort
(
    PROV_HANDLE* phandle,
    UCHAR reason
)
{
    PROV_CONTEXT* ctx;
    API_RESULT retval;
    PROV_TRC("-> MS_prov_abort\n");

    /* Validate the input */
    if (NULL == phandle)
    {
        PROV_ERR("Invalid Handle.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Lock */
    PROV_LOCK();
    /* Get reference to the context */
    ctx = prov_findcontext_by_handle(phandle);

    if (NULL == ctx)
    {
        PROV_ERR("Failed to assert context.\n");
        /* Unlock */
        PROV_UNLOCK();
        return PROV_CONTEXT_ASSERT_FAILED;
    }

    PROV_ERR("Terminate the link on context %d.\n", ctx->handle);
    /* Close the link */
    retval = prov_link_close(ctx, reason);
    PROV_TRC("<- MS_prov_abort\n");
    /* Unlock */
    PROV_UNLOCK();
    return retval;
}


/**
    \brief

    \par Description


    \param phandle
    \param pdu
    \param pdata
    \param datalen

    \return void
*/
API_RESULT MS_prov_send_pdu
(
    /* IN */ PROV_HANDLE*    phandle,
    /* IN */ UCHAR           pdu,
    /* IN */ void*           pdata,
    /* IN */ UINT16          datalen
)
{
    PROV_CONTEXT* ctx;
    API_RESULT retval;
    PROV_METHOD_S param_method;
    PROV_DATA_S param_data;
    UINT32 num;
    UCHAR* str;
    UCHAR numeric[sizeof(UINT32)];
    PROV_TRC ("-> MS_prov_send_pdu\n");

    /* Validate the input */
    if ((NULL == phandle) ||
            (NULL == pdata))
    {
        PROV_ERR ("Invalid Handle or Data.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Lock */
    PROV_LOCK();
    /* Get reference to the context */
    ctx = prov_findcontext_by_handle(phandle);

    if (NULL == ctx)
    {
        PROV_ERR ("Failed to assert context.\n");
        /* Unlock */
        PROV_UNLOCK();
        return PROV_CONTEXT_ASSERT_FAILED;
    }

    PROV_TRC ("Send for PDU - 0x%02X.\n", pdu);

    switch (pdu)
    {
    case PROV_PDU_TYPE_START:
        /* Update the Start Parameters structure */
        EM_mem_copy (&param_method, (PROV_METHOD_S*)pdata, sizeof (PROV_METHOD_S));
        PROV_TRC ("Posting Start event.\n");
        /* Post the Start event */
        retval = prov_fsm_post_levent
                 (
                     ctx,
                     ev_prov_start,
                     &param_method,
                     sizeof (PROV_METHOD_S)
                 );

        if (API_SUCCESS == retval)
        {
            PROV_TRC("Updating Pubkey event.\n");
            prov_process_event(ctx, ev_prov_pubkey, PROV_FSM_LOCAL_EVENT);
        }

        break;

    case PROV_PDU_TYPE_INPUT_CMPLT:

        /* Get the OOB Action */
        /**
            Check if:
            - OOB type [Output/Input] and
            - OOB Action [Ouput Alphanumeric/Input Alphanumeric]
        */
        if (((PROV_AUTH_OOB_OUTPUT == PROV_GET_AUTH_TYPE(ctx)) &&
                (PROV_OOOB_ACTION_ALPHANUMERIC == PROV_GET_AUTH_ACTION(ctx))) ||
                ((PROV_AUTH_OOB_INPUT == PROV_GET_AUTH_TYPE(ctx)) &&
                 (PROV_IOOB_ACTION_ALPHANUMERIC == PROV_GET_AUTH_ACTION(ctx))))
        {
            /* Input Authval is Alphanumeric (String) */
            str = (UCHAR*)pdata;
            /* Populate the Authval */
            EM_mem_copy(ctx->authval, str, datalen);
        }
        else
        {
            /* Input Authval is Numeric */
            num = *((UINT32*)pdata);
            MS_PACK_BE_4_BYTE(numeric, &num);
            EM_mem_copy(&ctx->authval[PROV_AUTHVAL_SIZE - (sizeof(UINT32))], numeric, sizeof(UINT32));
        }

        PROV_TRC ("Posting Input Complete event.\n");
        /* Post the event */
        retval = prov_fsm_post_levent
                 (
                     ctx,
                     ev_prov_inputcom,
                     NULL,
                     0
                 );
        break;

    case PROV_PDU_TYPE_DATA:
        /* Copy the data */
        EM_mem_copy (&param_data, (PROV_DATA_S*)pdata, sizeof (PROV_DATA_S));
        PROV_TRC ("Posting Data event.\n");
        /* Post the event */
        retval = prov_fsm_post_levent
                 (
                     ctx,
                     ev_prov_data,
                     &param_data,
                     sizeof(PROV_DATA_S)
                 );
        break;

    default:
        retval = PROV_INVALID_PARAMETER;
        break;
    }

    /* Unlock */
    PROV_UNLOCK();
    PROV_TRC ("<- MS_prov_start\n");
    return retval;
}


/**
    \brief

    \par Description


    \param phandle
    \param pdata
    \param datalen

    \return void
*/
API_RESULT MS_prov_set_authval
(
    /* IN */ PROV_HANDLE* phandle,
    /* IN */ void*         pdata,
    /* IN */ UINT16        datalen
)
{
    PROV_CONTEXT* ctx;
    UINT32 num;
    UCHAR* str;
    UCHAR numeric[sizeof(UINT32)];
    PROV_TRC("-> MS_prov_set_display_authval\n");

    /* Validate the input */
    if ((NULL == phandle) ||
            (NULL == pdata))
    {
        PROV_ERR("Invalid Handle or Data.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Lock */
    PROV_LOCK();
    /* Get reference to the context */
    ctx = prov_findcontext_by_handle(phandle);

    if (NULL == ctx)
    {
        PROV_ERR("Failed to assert context.\n");
        /* Unlock */
        PROV_UNLOCK();
        return PROV_CONTEXT_ASSERT_FAILED;
    }

    /* Get the OOB Action */
    /**
        Check if:
        - OOB type [Output/Input] and
        - OOB Action [Output Alphanumeric/Input Alphanumeric]
    */
    if (((PROV_AUTH_OOB_OUTPUT == PROV_GET_AUTH_TYPE(ctx)) &&
            (PROV_OOOB_ACTION_ALPHANUMERIC == PROV_GET_AUTH_ACTION(ctx))) ||
            ((PROV_AUTH_OOB_INPUT == PROV_GET_AUTH_TYPE(ctx)) &&
             (PROV_IOOB_ACTION_ALPHANUMERIC == PROV_GET_AUTH_ACTION(ctx))))
    {
        /* Input Authval is Alphanumeric (String) */
        str = (UCHAR*)pdata;
        /* Populate the Authval */
        EM_mem_copy(ctx->authval, str, datalen);
    }
    else
    {
        /* Input Authval is Numeric */
        num = *((UINT32*)pdata);
        MS_PACK_BE_4_BYTE(numeric, &num);
        EM_mem_copy(&ctx->authval[PROV_AUTHVAL_SIZE - (sizeof(UINT32))], numeric, sizeof(UINT32));
    }

    PROV_TRC("The Authval set by Application :\n");
    PROV_debug_dump_bytes(ctx->authval, sizeof(ctx->authval));
    /* Unlock */
    PROV_UNLOCK();
    PROV_TRC("<- MS_prov_set_display_authval\n");
    return API_SUCCESS;
}

/**
    \brief

    \par Description


    \param public_key

    \return void
*/
API_RESULT MS_prov_get_local_public_key
(
    /* OUT */ UCHAR*   public_key
)
{
    API_RESULT retval;
    PROV_TRC("-> MS_prov_get_local_public_key\n");
    retval = API_SUCCESS;

    /* Validate the input */
    if (NULL == public_key)
    {
        PROV_ERR("Invalid Buffer.\n");
        return PROV_INVALID_PARAMETER;
    }

    /* Lock */
    PROV_LOCK();
    /**
        Copying the Public Key used for Provisioning to the application buffer.
    */
    EM_mem_copy
    (
        public_key,
        prov_pubkey,
        PROV_PUBKEY_SIZE
    );
    PROV_INF ("Local Public Key provided to upper layer:\n");
    PROV_debug_dump_bytes (public_key, PROV_PUBKEY_SIZE);
    PROV_TRC("<- MS_prov_get_local_public_key\n");
    /* Unlock */
    PROV_UNLOCK();
    return retval;
}
