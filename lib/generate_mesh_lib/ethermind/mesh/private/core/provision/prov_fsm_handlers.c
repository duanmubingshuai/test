
/**
    \file prov_fsm_handlers.c

    This file implements the function handler for PROV FSM
*/

/*
     Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "prov_internal.h"
#include "prov_fsm_handlers.h"
#include "prov_extern.h"
#include "timer.h"     //HZF

//static inline uint32 clock_time_rtc(void){
//  return (*(volatile unsigned int *)0x4000f028) & 0xffffff;
//}


#include "MS_net_api.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */


API_RESULT se_prov_invite_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR* pdata;
//    UINT16 pdatalen;
    API_RESULT retval;
    UCHAR marker;
    UCHAR pdu[PROV_PDU_TYPE_INVITE_LEN];
    UCHAR attention;
    PROV_TRC (
        "[PROV]: >>> se_prov_invite_handler");
    /* Get the context reference */
    ctx = param->ctx;
    /* Generate the random value for the context */
    cry_rand_generate(ctx->lrandval, PROV_RAND_SIZE);

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Check the role to be provisioner only */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_PROVISIONER);
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_INVITE;
        /* Pack the Identify duration */
        pdu[marker++] = ctx->attention;
        /* Save the packet to context for use in authentication algorithms */
        EM_mem_copy(ctx->inviteval, (pdu + 1), (marker - 1));
        /* Call to frame the header and send the packet */
        retval = prov_framensend_pdu (ctx, pdu, marker, SL_0_PROV_W4CAPAB);
    }
    else
    {
        /* Stop the retransmission timer if running */
        if (EM_TIMER_HANDLE_INIT_VAL != link_open_ack_timer_handle)
        {
            EM_stop_timer(&link_open_ack_timer_handle);
        }

        /* Check the role to be device only */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_DEVICE);
        pdata = param->pdata;
//        pdatalen = param->pdatalen;
        /* Get the attention duration */
        attention = pdata[0];
        /* Process the Invite */
        EM_mem_copy(ctx->inviteval, pdata, PROV_PDU_INVITEVAL_LEN);
        /* Update the context state */
        PROV_CONTEXT_SET_STATE (ctx, SL_0_PROV_INCAPAB);
        /* Send the Capabilities */
        retval = prov_fsm_post_levent (ctx, ev_prov_capabilities, NULL, 0);
        /* Notify application of start of provisioning */
        prov_notify
        (
            ctx,
            PROV_EVT_PROVISIONING_SETUP,
            API_SUCCESS,
            &attention,
            sizeof (UCHAR)
        );
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_invite_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_error_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_TRC (
        "[PROV]: >>> se_prov_error_handler");

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
    }
    else
    {
    }

    /* Sending Provision Error as Unexpected */
    prov_procedure_complete(param->ctx, PROV_ERR_UNEXPECTED_PDU);
    PROV_TRC (
        "[PROV]: <<< se_prov_error_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_capabilities_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR* pdata;
//    UINT16 pdatalen;
    API_RESULT retval;
    UCHAR marker;
    UCHAR pdu[PROV_PDU_TYPE_CAPAB_LEN];
    PROV_CAPABILITIES_S cap;
    PROV_TRC (
        "[PROV]: >>> se_prov_capabilities_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Check if the role is device here */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_DEVICE);
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_CAPAB;
        /* Pack the Number of elements */
        pdu[marker++] = prov_cap->num_elements;
        /* Pack the Algorithm */
        MS_PACK_BE_2_BYTE(&pdu[marker], &prov_cap->supp_algorithms);
        marker += 2;
        /* Pack the supported public key types */
        pdu[marker++] = prov_cap->supp_pubkey;
        /* Pack the supported static oob types */
        pdu[marker++] = prov_cap->supp_soob;
        /* Pack the Output OOB Size and supported actions */
        pdu[marker++] = prov_cap->ooob.size;
        MS_PACK_BE_2_BYTE(&pdu[marker], &prov_cap->ooob.action);
        marker += 2;
        /* Pack the Input OOB Size and supported actions */
        pdu[marker++] = prov_cap->ioob.size;
        MS_PACK_BE_2_BYTE(&pdu[marker], &prov_cap->ioob.action);
        marker += 2;
        /* Save the packet to context for use in authentication algorithms */
        EM_mem_copy(ctx->capval, (pdu + 1), (marker - 1));
        /* Call to frame the header and send the packet */
        retval = prov_framensend_pdu (ctx, pdu, marker, SL_0_PROV_W4START);
    }
    else
    {
        /* Check if the role is provisioner here */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_PROVISIONER);
        pdata = param->pdata;
//        pdatalen = param->pdatalen;
        marker = 0;
        /* Unpack the Number of elements */
        cap.num_elements = pdata[marker++];
        /* Save the Number of elements in the provisioning context */
        ctx->r_num_elements = cap.num_elements;
        /* Unpack the Algorithm */
        MS_UNPACK_BE_2_BYTE(&cap.supp_algorithms, &pdata[marker]);
        marker += 2;
        /* Unpack the supported public key types */
        cap.supp_pubkey = pdata[marker++];
        /* Unpack the supported static oob types */
        cap.supp_soob = pdata[marker++];
        /* Unpack the Output OOB Size and supported actions */
        cap.ooob.size = pdata[marker++];
        MS_UNPACK_BE_2_BYTE(&cap.ooob.action, &pdata[marker]);
        marker += 2;
        /* Unpack the Input OOB Size and supported actions */
        cap.ioob.size = pdata[marker++];
        MS_UNPACK_BE_2_BYTE(&cap.ioob.action, &pdata[marker]);
        marker += 2;
        /* Process the capabilities */
        EM_mem_copy(ctx->capval, pdata, PROV_PDU_CAPABVAL_LEN);
        /* Update the context state */
        PROV_CONTEXT_SET_STATE (ctx, SL_0_PROV_INSTART);
        /* Notify application of remote capabilities */
        prov_notify
        (
            ctx,
            PROV_EVT_PROVISIONING_SETUP,
            API_SUCCESS,
            &cap,
            sizeof (PROV_CAPABILITIES_S)
        );
        /*
            Wait for application to make a selection on the
            required capabilities
        */
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_capabilities_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_start_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR* pdata;
//    UINT16 pdatalen;
    API_RESULT retval;
    UCHAR marker;
    UCHAR pdu[PROV_PDU_TYPE_START_LEN];
    PROV_METHOD_S* params;
    PROV_TRC (
        "[PROV]: >>> se_prov_start_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Check if the role can only be Provisioner here */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_PROVISIONER);
        /* Reference the parameters */
        params = (PROV_METHOD_S*)param->pdata;
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_START;
        /* Pack the algorithm */
        pdu[marker++] = params->algorithm;
        /* Pack the public key used */
        pdu[marker++] = params->pubkey;
        /* Pack the Authentication method used */
        pdu[marker++] = params->auth;
        /* Pack the Authentication action used */
        pdu[marker++] = (UCHAR)params->oob.action;
        /* Pack the Authentication size used */
        pdu[marker++] = params->oob.size;
        /* Save the packet to context for use in authentication algorithms */
        EM_mem_copy(ctx->startval, (pdu + 1), (marker - 1));
        /* Call to frame the header */
        retval = prov_framensend_pdu (ctx, pdu, marker, SL_0_PROV_INPUBKEY);
    }
    else
    {
        /* Check if the role can only be Device here */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_DEVICE);
        pdata = param->pdata;
//        pdatalen = param->pdatalen;

        /* Validate */
        if ((*pdata > 0x00) || (*(pdata + 1) > 0x01) || (*(pdata + 2) > 0x03) ||
                (*(pdata + 3) > 0x04) || (*(pdata + 4) > 0x08))
        {
            PROV_ERR("Dropping Start for Invalid Parameters...\n");
            /* Sending provisioning failure with Invalid Format error */
            prov_procedure_complete(ctx, PROV_ERR_INVALID_FORMAT);
        }
        else
        {
            /* Check if the start parameters are inline with the supported capabilities */
            if (!(prov_cap->supp_algorithms & (1 << *pdata)) ||
                    ((0 == prov_cap->supp_pubkey) && (0 != *(pdata + 1))) ||
                    ((0 == prov_cap->supp_soob) && (PROV_AUTH_OOB_STATIC == *(pdata + 2))) ||
                    ((0 == prov_cap->ioob.action) && (PROV_AUTH_OOB_INPUT == *(pdata + 2))) ||
                    ((0 == prov_cap->ooob.action) && (PROV_AUTH_OOB_OUTPUT == *(pdata + 2))))
            {
                PROV_ERR("Dropping Start for Invalid Parameters...\n");
                /* Sending provisioning failure with Invalid Format error */
                prov_procedure_complete(ctx, PROV_ERR_INVALID_FORMAT);
                return API_SUCCESS;
            }

            /**
                When the Authentication Method 0x00 (Authentication with No OOB) method is used,
                the Authentication Action field shall be set to 0x00 and the Authentication Size
                field shall be set to 0x00.
                or
                When the Authentication Method 0x01 (Authentication with Static OOB) method is used,
                the Authentication Size shall be set to 0x00 and the Authentication Action field
                shall be set to 0x00.
            */
            if (((0x00 == (*(pdata + 2)) || (0x01 == (*(pdata + 2)))) && ((*(pdata + 3) > 0) || (*(pdata + 4) > 0))))
            {
                /* Sending provisioning failure with Invalid Format error */
                prov_procedure_complete(ctx, PROV_ERR_INVALID_FORMAT);
                return API_SUCCESS;
            }

            /* Process the start */
            EM_mem_copy(ctx->startval, pdata, PROV_PDU_STARTVAL_LEN);
            /* Update the context state */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_W4PUBKEY);
            /* Wait for Public Key from Provisioner */
        }
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_start_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_pubkey_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR* pdata;
//    UINT16 pdatalen;
    API_RESULT retval;
    UCHAR marker;
    PROV_OOB_TYPE_S oob_info;
    UCHAR pdu[PROV_PDU_TYPE_PUBKEY_LEN];
    INT32 ret;
    UINT32 state;
//UINT32  T1, T2;
    PROV_TRC (
        "[PROV]: >>> se_prov_pubkey_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Reference the parameters */
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_PUBKEY;
        PROV_TRC("Sending Local Public Key:\n");
        PROV_debug_dump_bytes(prov_pubkey, PROV_PUBKEY_SIZE);
        /* Pack the Public Key */
        EM_mem_copy(&pdu[marker], prov_pubkey, PROV_PUBKEY_SIZE);
        marker += PROV_PUBKEY_SIZE;

        /* Is role device? */
        if (MS_TRUE != PROV_IS_ROLE_DEVICE(ctx))
        {
            /* No, wait for public key from Device */
            state = SL_0_PROV_W4PUBKEY;

            /* OOB Public Key to use? */
            if (MS_TRUE == PROV_IS_OOB_PUBKEY(ctx))
            {
                /* Update next event to be posted upon Acknowledgement */
                prov_process_event(ctx, ev_prov_pubkey, PROV_FSM_REMOTE_EVENT);
            }
        }
        else
        {
            /* Yes, Wait for Confirmation */
            state = SL_0_PROV_W4CONF;
        }

        /* Call to frame the header */
        retval = prov_framensend_pdu(ctx, pdu, marker, state);
    }
    else
    {
        #ifdef CRY_ECDH_TIMESLICE

        if (PROV_PUBKEY_SIZE == param->pdatalen)
        #endif /* CRY_ECDH_TIMESLICE */
        {
            /* OOB Public Key to use when in Provisioner Role ? */
            if ((MS_TRUE != PROV_IS_ROLE_DEVICE(ctx)) && (MS_TRUE == PROV_IS_OOB_PUBKEY(ctx)))
            {
                /* Yes, get the OOB Public Key of the Device */
                prov_read_device_oob_pubkey_pl (ctx->rpubkey, sizeof(ctx->rpubkey));
            }
            else
            {
                pdata = param->pdata;
//                pdatalen = param->pdatalen;
                /* Process the Public Key */
                EM_mem_copy(ctx->rpubkey, pdata, PROV_PDU_PUBKEY_LEN);
            }

            PROV_TRC("Received Remote Public Key:\n");
            PROV_debug_dump_bytes(ctx->rpubkey, PROV_PUBKEY_SIZE);
            blebrr_scan_pl(FALSE);        // HZF
//  T1 = clock_time_rtc();//read_current_fine_time();
            /* Calculate ECDH */
            cry_ecdh_generate_secret_be
            (
                ctx->rpubkey,
                ctx->ecdh_key,
                PROV_SECRET_SIZE,
                ret,
                #ifdef CRY_ECDH_TIMESLICE
                prov_ecdh_complete_cb
                #else /* CRY_ECDH_TIMESLICE */
                NULL
                #endif /* CRY_ECDH_TIMESLICE */
            );
//  T2 = clock_time_rtc();//read_current_fine_time();
//  printf("consume time of function cry_ecdh_generate_secret_be: %d RTC tick\r\n", (T2 - T1));//(T2 > T1) ? (T2 - T1) : (BASE_TIME_UNITS - T1 + T2));

            if (0 > ret)
            {
                PROV_ERR ("ECDH Secret Calculation Failed\n");
                /* Sending Provision Error as Unexpected for ECDH Key Validation */
                prov_procedure_complete(ctx, PROV_ERR_UNEXPECTED_ERROR);
                return API_SUCCESS;
            }

            #ifdef CRY_ECDH_TIMESLICE
            else if (0 < ret)
            {
                PROV_TRC ("ECDH Secret Calculation Pending\r\n");
                return API_SUCCESS;
            }

            #endif /* CRY_ECDH_TIMESLICE */
        }

        #ifdef CRY_ECDH_TIMESLICE
        else if (PROV_SECRET_SIZE == param->pdatalen)
        {
            PROV_TRC ("ECDH calculation done.\r\n");
            /* Copy the Secret Key */
            EM_mem_copy(ctx->ecdh_key, param->pdata, param->pdatalen);
        }

        #endif /* CRY_ECDH_TIMESLICE */
        PROV_INF ("ECDH Secret Calculated:\n");
        PROV_debug_dump_bytes(ctx->ecdh_key, PROV_SECRET_SIZE);

        /* Is the role Device? */
        if (MS_TRUE != PROV_IS_ROLE_DEVICE(ctx))
        {
            /* No, Get the authentication type */
            switch (PROV_GET_AUTH_TYPE(ctx))
            {
            case PROV_AUTH_OOB_STATIC: /* Fall Through */
                /* Read the static OOB to be used */
                prov_read_static_oob_auth_pl(ctx->authval, PROV_AUTHVAL_SIZE);

            case PROV_AUTH_OOB_NONE:
                /* Update state to send Conf Val */
                PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INCONF);
                /* Send the confirm Value */
                prov_fsm_post_levent (ctx, ev_prov_confirmation, NULL, 0);
                break;

            case PROV_AUTH_OOB_INPUT:
                /* Update state to wait for Input Complete from Device */
                PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_W4INPCOM);
                /* Get the OOB Information */
                oob_info.action = PROV_GET_AUTH_ACTION(ctx);
                oob_info.size = PROV_GET_AUTH_SIZE(ctx);
                /*
                    Notify the application about the Output
                    OOB Random Number
                */
                prov_notify
                (
                    ctx,
                    PROV_EVT_OOB_DISPLAY,
                    API_SUCCESS,
                    &oob_info,
                    sizeof (PROV_OOB_TYPE_S)
                );
                break;

            case PROV_AUTH_OOB_OUTPUT:
                /* Update state to wait for Input Complete from application */
                PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_ININPCOM);
                /* Get the OOB Information */
                oob_info.action = PROV_GET_AUTH_ACTION(ctx);
                oob_info.size = PROV_GET_AUTH_SIZE(ctx);
                /*
                    Notify the application to Input the
                    OOB Random Number
                */
                prov_notify
                (
                    ctx,
                    PROV_EVT_OOB_ENTRY,
                    API_SUCCESS,
                    &oob_info,
                    sizeof (PROV_OOB_TYPE_S)
                );
                break;

            default:
                /* Unknown OOB type. Quit Procedure? */
                break;
            }
        }
        else
        {
            /* Yes, is OOB Public key to be used? */
            if (MS_TRUE != PROV_IS_OOB_PUBKEY(ctx))
            {
                /* Send the Public Key */
                prov_fsm_post_levent (ctx, ev_prov_pubkey, NULL, 0);
            }

            /* Get the authentication type */
            switch (PROV_GET_AUTH_TYPE(ctx))
            {
            case PROV_AUTH_OOB_STATIC:
                /* Read the static OOB to be used */
                prov_read_static_oob_auth_pl(ctx->authval, PROV_AUTHVAL_SIZE);

            case PROV_AUTH_OOB_NONE:
                /* Update state to wait for Conf Val from Provisioner */
                PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_W4CONF);
                break;

            case PROV_AUTH_OOB_INPUT:
                /* Update state to wait from Input Complete from appl */
                PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_ININPCOM);
                PROV_SET_ACK_STATE(ctx, SL_0_PROV_ININPCOM);
                /* Get the OOB Information */
                oob_info.action = PROV_GET_AUTH_ACTION(ctx);
                oob_info.size = PROV_GET_AUTH_SIZE(ctx);
                /*
                    Notify the application to Input the
                    OOB Random Number
                */
                prov_notify
                (
                    ctx,
                    PROV_EVT_OOB_ENTRY,
                    API_SUCCESS,
                    &oob_info,
                    sizeof (PROV_OOB_TYPE_S)
                );
                break;

            case PROV_AUTH_OOB_OUTPUT:
                /* Update state to wait for Conf Val from Provisioner */
                PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_W4CONF);
                /* Get the OOB Information */
                oob_info.action = PROV_GET_AUTH_ACTION(ctx);
                oob_info.size = PROV_GET_AUTH_SIZE(ctx);
                /*
                    Notify the application about the Output
                    OOB Random Number
                */
                prov_notify
                (
                    ctx,
                    PROV_EVT_OOB_DISPLAY,
                    API_SUCCESS,
                    &oob_info,
                    sizeof (PROV_OOB_TYPE_S)
                );
                break;

            default:
                /* Unknown OOB type. Quit Procedure? */
                break;
            }
        }
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_pubkey_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_inputcom_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR marker;
    UCHAR pdu[PROV_PDU_TYPE_INPUT_CMPLT_LEN];
    PROV_TRC (
        "[PROV]: >>> se_prov_inputcom_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Is the role Device? */
        if (MS_TRUE != PROV_IS_ROLE_DEVICE(ctx))
        {
            /* No, Update state to send Conf Val */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INCONF);
            /* Send the confirm Value */
            prov_fsm_post_levent (ctx, ev_prov_confirmation, NULL, 0);
        }
        else
        {
            /* Initialize Marker and PDU */
            marker = 0;
            EM_mem_set(pdu, 0x00, sizeof (pdu));
            /* Pack the PDU Type */
            pdu[marker++] = PROV_PDU_TYPE_INPUT_CMPLT;
            /* No Parameters */
            /* Call to frame the header */
            prov_framensend_pdu (ctx, pdu, marker, SL_0_PROV_W4CONF);
        }
    }
    else
    {
        /* Check if the role can only be Provisioner here */
        PROV_ASSERT_CONTEXT_ROLE(ctx, PROV_ROLE_PROVISIONER);
        /* No, Update state to send Conf Val */
        PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INCONF);
        /* Send the confirm Value */
        prov_fsm_post_levent (ctx, ev_prov_confirmation, NULL, 0);
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_inputcom_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_confirmation_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR* pdata;
//    UINT16 pdatalen;
    API_RESULT retval;
    UCHAR marker;
    UCHAR pdu[PROV_PDU_TYPE_CNF_LEN];
    UINT32 state;
    PROV_TRC (
        "[PROV]: >>> se_prov_confirmation_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Reference the parameters */
        /* Calculate confirm value */
        prov_calc_confirm(ctx, 0x00);
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_CONF;
        /* Pack the Confirmation value */
        EM_mem_copy(&pdu[marker], ctx->lconfval, sizeof(ctx->lconfval));
        marker += sizeof(ctx->lconfval);

        /* Is the role Device? */
        if (MS_TRUE != PROV_IS_ROLE_DEVICE(ctx))
        {
            /* No, Update the state to wait for Confirmation from Device */
            state = SL_0_PROV_W4CONF;
        }
        else
        {
            /* Yes, Update the state to wait for Rand from Provisioner */
            state = SL_0_PROV_W4RAND;
        }

        /* Call to frame the header */
        retval = prov_framensend_pdu(ctx, pdu, marker, state);
    }
    else
    {
        pdata = param->pdata;
//        pdatalen = param->pdatalen;
        /* Process the confirmation received */
        EM_mem_copy(ctx->rconfval, pdata, PROV_CONFVAL_SIZE);

        /* Is role Device? */
        if (MS_TRUE != PROV_IS_ROLE_DEVICE(ctx))
        {
            /* No, Update state to send Random value */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INRAND);
            /* Send the random Value */
            prov_fsm_post_levent (ctx, ev_prov_random, NULL, 0);
        }
        else
        {
            /* Yes, Update state to send Confirm value */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INCONF);
            /* Send the confirm Value */
            prov_fsm_post_levent (ctx, ev_prov_confirmation, NULL, 0);
        }
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_confirmation_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_random_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR* pdata;
//    UINT16 pdatalen;
    API_RESULT retval;
    UCHAR marker;
    UCHAR pdu[PROV_PDU_TYPE_RAND_LEN];
    UINT32 state;
    PROV_TRC (
        "[PROV]: >>> se_prov_random_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Reference the parameters */
        blebrr_scan_pl(FALSE);        // ZQY
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_RAND;
        /* Pack the Random value */
        EM_mem_copy(&pdu[marker], ctx->lrandval, PROV_RAND_SIZE);
        marker += PROV_RAND_SIZE;

        /* Is role Device? */
        if (MS_TRUE != PROV_IS_ROLE_DEVICE(ctx))
        {
            /* No, Update the state to wait for Rand from Device */
            state = SL_0_PROV_W4RAND;
        }
        else
        {
            /* Yes, Update the state to wait for Provisioning data */
            state = SL_0_PROV_W4DATA;
        }

        printf("sending random!\n");
        /* Call to frame the header */
        retval = prov_framensend_pdu(ctx, pdu, marker, state);
    }
    else
    {
        pdata = param->pdata;
//        pdatalen = param->pdatalen;
        /* Process the Random received */
        EM_mem_copy(ctx->rrandval, pdata, PROV_RANDVAL_SIZE);
        blebrr_scan_pl(FALSE);        // ZQY
        /* Check the confirm value */
        retval = prov_calc_confirm(ctx, 0x01);

        if (API_SUCCESS != retval)
        {
            PROV_ERR ("Confirm Value Check Failed!\n");
            printf("Confirm Value Check Failed!\n");
            /* Notify procedure completion */
            prov_procedure_complete(ctx, PROV_ERR_CONFIRMATION_FAILED);
            return API_SUCCESS;
        }

        PROV_TRC ("Confirm Value Check Success!\n");
        printf("Confirm Value Check Success!\n");

        /* Is role Device? */
        if (MS_TRUE == PROV_IS_ROLE_DEVICE(ctx))
        {
            /* Yes, Update the state to send Rand */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INRAND);
            /* Send the Random Value */
            prov_fsm_post_levent (ctx, ev_prov_random, NULL, 0);
        }

        /* Calculate the Authentication keys */
        prov_generate_authkeys(ctx);

        /* Is role provisioner? */
        if (MS_TRUE == PROV_IS_ROLE_PROVISIONER(ctx))
        {
            /* Yes, Update the state to send Provisioning data */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INDATA);
            /* Notify the application requesting for Provisional Data */
            prov_notify
            (
                ctx,
                PROV_EVT_PROVDATA_INFO_REQ,
                API_SUCCESS,
                NULL,
                0
            );
        }
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_random_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_data_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR* pdata;
//    UINT16 pdatalen;
    API_RESULT retval;
    UCHAR marker;
    UCHAR pdu[PROV_PDU_TYPE_DATA_LEN];
    UCHAR mac[PROV_DATA_MIC_SIZE];
    UCHAR epdu[sizeof(pdu) + sizeof(mac)];
    PROV_DATA_S* params, rdata;
    UCHAR* pduptr;
    INT32 ret;
    PROV_TRC (
        "[PROV]: >>> se_prov_data_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Check if the role can only be Provisioner here */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_PROVISIONER);
        blebrr_scan_pl(FALSE);        // ZQY
        /* Reference the parameters */
        params = (PROV_DATA_S*)param->pdata;
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_DATA;
        /* Pack the Network Key */
        EM_mem_copy(&pdu[marker], params->netkey, sizeof(params->netkey));
        marker += sizeof(params->netkey);
        /* Pack the NetKey Index */
        MS_PACK_BE_2_BYTE(&pdu[marker], &params->keyid);
        marker += sizeof(params->keyid);
        /* Pack Flags */
        pdu[marker++] = params->flags;
        /* Pack the IV Index */
        MS_PACK_BE_4_BYTE(&pdu[marker], &params->ivindex);
        marker += sizeof(params->ivindex);
        /* Pack the Unicast Address */
        MS_PACK_BE_2_BYTE(&pdu[marker], &params->uaddr);
        marker += sizeof(params->uaddr);
        /* Save Peer Provisionee address */
        ctx->r_uaddr = params->uaddr;
        /* Encrypt the Data PDU */
        cry_aes_128_ccm_encrypt_be
        (
            ctx->session_key,
            ctx->nonce, sizeof(ctx->nonce),
            (pdu + 1), (marker - 1),
            NULL, 0,
            (epdu + 1),
            mac, sizeof(mac),
            ret
        );
        epdu[0] = pdu[0];
        /* Append the mac to encrypted data */
        EM_mem_copy(epdu + marker, mac, sizeof(mac));
        marker += sizeof(mac);
        /* Call to frame the header */
        retval = prov_framensend_pdu (ctx, epdu, marker, SL_0_PROV_W4COMPLETE);
    }
    else
    {
        /* Have check if the role can only be Device here */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_DEVICE);
        pdata = param->pdata;
//        pdatalen = param->pdatalen;
        blebrr_scan_pl(FALSE);        // ZQY
        /* Process the data received */
        EM_mem_copy(mac, &pdata[ctx->rx_pdu_len - 1 - sizeof(mac)], sizeof(mac));
        PROV_TRC ("[PRVSNG] Device Key:\n");
        PROV_debug_dump_bytes(ctx->dev_key, sizeof(ctx->dev_key));
        PROV_TRC ("[PRVSNG] Session Key:\n");
        PROV_debug_dump_bytes(ctx->session_key, sizeof(ctx->session_key));
        PROV_TRC ("[PRVSNG] Nonce:\n");
        PROV_debug_dump_bytes(ctx->nonce, sizeof(ctx->nonce));
        PROV_TRC ("[PRVSNG] Data:\n");
        PROV_debug_dump_bytes(pdata, (ctx->rx_pdu_len - 1 - sizeof(mac)));
        cry_aes_128_ccm_decrypt_be
        (
            ctx->session_key,
            ctx->nonce, sizeof(ctx->nonce),
            pdata, (ctx->rx_pdu_len - 1 - sizeof(mac)),
            NULL, 0,
            pdu,
            mac, sizeof(mac),
            ret
        );

        /* Compare the MAC received and calculated */
        if ((0 > ret) ||
                (EM_mem_cmp(mac, &pdata[ctx->rx_pdu_len - 1 - sizeof(mac)], sizeof(mac))))
        {
            /* MAC verification failed. Send Prov Failure */
            PROV_ERR("The AES CCM DECRYPT Failed with retval 0x%08X !!!!\n", ret);
            /* Notify procedure completion */
            prov_procedure_complete(ctx, PROV_ERR_DECRYPTION_FAILED);
        }
        else
        {
            /* MS_NET_PRVSNG_DATA    prvsng_data; */
            /* Decode and Print the Packet */
            PROV_TRC (
                "[PRVSNG] Provisioning Data Decoded\n");
            PROV_TRC ("[PRVSNG] Decrypted Keys:\n");
            PROV_debug_dump_bytes(pdu, sizeof(pdu));
            pduptr = pdu;
            PROV_TRC ("[PRVSNG] Network Key:\n");
            PROV_debug_dump_bytes(pdu, PROV_KEY_NETKEY_SIZE);
            EM_mem_copy (rdata.netkey, pduptr, PROV_KEY_NETKEY_SIZE);
            pduptr += PROV_KEY_NETKEY_SIZE;
            PROV_TRC ("[PRVSNG] Key Index:\n");
            PROV_debug_dump_bytes(pduptr, sizeof (UINT16));
            MS_UNPACK_BE_2_BYTE(&rdata.keyid, pduptr);
            pduptr += sizeof(UINT16);
            PROV_TRC ("[PRVSNG] Flags:\n");
            PROV_debug_dump_bytes(pduptr, sizeof (UCHAR));
            rdata.flags = *(pduptr);
            pduptr += sizeof(UCHAR);
            PROV_TRC ("[PRVSNG] Current IV index:\n");
            PROV_debug_dump_bytes(pduptr, sizeof(UINT32));
            MS_UNPACK_BE_4_BYTE(&rdata.ivindex, pduptr);
            pduptr += sizeof(UINT32);
            PROV_TRC ("[PRVSNG] Unicast Address:\n");
            PROV_debug_dump_bytes(pduptr, sizeof(UINT16));
            MS_UNPACK_BE_2_BYTE(&rdata.uaddr, pduptr);
            PROV_TRC ("[PRVSNG] Decrypted MAC:\n");
            PROV_debug_dump_bytes(mac, sizeof(mac));
            #if 0
            /* Save Provisioning Data with Network Layer */
            EM_mem_copy(prvsng_data.net_key, pdu, 16);
            MS_UNPACK_BE_2_BYTE(&prvsng_data.key_idx, pdu + 16);
            prvsng_data.flags = pdu[16 + 2];
            MS_UNPACK_BE_4_BYTE(&prvsng_data.ivi, pdu + 16 + 2 + 1);
            MS_UNPACK_BE_2_BYTE(&prvsng_data.uaddr, pdu + 16 + 2 + 1 + 4);
            retval = MS_net_set_prvsng_data
                     (
                         &prvsng_data
                     );
            #endif /* 0 */
            /* Update the state */
            PROV_CONTEXT_SET_STATE(ctx, SL_0_PROV_INCOMPLETE);
            /* Notify Provisioning data to application */
            prov_notify(ctx, PROV_EVT_PROVDATA_INFO, API_SUCCESS, &rdata, sizeof(PROV_DATA_S));
            /* Send Provisioning Complete */
            prov_fsm_post_levent (ctx, ev_prov_complete, NULL, 0);
        }
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_data_handler");
    return API_SUCCESS;
}


API_RESULT se_prov_complete_handler
(
    PROV_EVENT_INFO*     param
)
{
    PROV_CONTEXT* ctx;
    UCHAR marker;
    API_RESULT retval;
    UCHAR pdu[PROV_PDU_TYPE_COMPLETE_LEN];
    PROV_TRC (
        "[PROV]: >>> se_prov_complete_handler");
    /* Get the context reference */
    ctx = param->ctx;

    /* Check if the event is locally initiated or received from peer */
    if (PROV_FSM_LOCAL_EVENT == param->event_info)
    {
        /* Check if the role can only be Device */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_DEVICE);
        /* Initialize Marker and PDU */
        marker = 0;
        EM_mem_set(pdu, 0x00, sizeof (pdu));
        /* Pack the PDU Type */
        pdu[marker++] = PROV_PDU_TYPE_COMPLETE;
        /* No Parameters */
        /* Call to frame the header */
        prov_framensend_pdu (ctx, pdu, marker, SL_0_PROV_IDLE);

        /* Send Ack only if bearer type is PB ADV */
        if (BRR_TYPE_PB_ADV == ctx->bearer)
        {
            /* Update next event to be posted upon Acknowledgement */
            PROV_SET_ACK_EVENT(ctx, ev_prov_complete, PROV_FSM_LOCAL_EVENT);
        }
        else
        {
            prov_procedure_complete(ctx, API_SUCCESS);
        }
    }
    else
    {
        /* Check if the role can only be Provisioner here */
        PROV_ASSERT_CONTEXT_ROLE (ctx, PROV_ROLE_PROVISIONER);

        /* Send Ack only if bearer type is PB ADV */
        if (BRR_TYPE_PB_ADV == ctx->bearer)
        {
            /*
                Start a timer to allow ACK being sent to ProvComplete from device on
                account of retransmissions
            */
            /* Start the ACK tracking retransmission timer */
            retval = EM_start_timer
                     (
                         &ctx->proc_timer_handle,
                         EM_TIMEOUT_MILLISEC | PROV_COMPLETE_TIMEOUT_SEC,
                         prov_complete_timeout_handler,
                         (void*)&ctx,
                         sizeof(void*)
                     );

            if (API_SUCCESS != retval)
            {
                PROV_ERR("Failed to start PDU Retx Timer - 0x%04X\n", retval);
            }
        }
        else
        {
            prov_procedure_complete(ctx, API_SUCCESS);
        }
    }

    PROV_TRC (
        "[PROV]: <<< se_prov_complete_handler");
    return API_SUCCESS;
}


