﻿/**
    \file time_client.c

    \brief This file defines the Mesh Time Model Application Interface
    - includes Data Structures and Methods for Client.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "time_client.h"


/* --------------------------------------------- Global Definitions */


/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 time_client_opcode_list[] =
{
    MS_ACCESS_TIME_STATUS_OPCODE,
    MS_ACCESS_TIME_ROLE_STATUS_OPCODE,
    MS_ACCESS_TIME_ZONE_STATUS_OPCODE,
    MS_ACCESS_TAI_UTC_DELTA_STATUS_OPCODE
};

static MS_ACCESS_MODEL_HANDLE   time_client_model_handle;
static MS_TIME_CLIENT_CB        time_client_appl_cb;


/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    \brief API to initialize Time Client model

    \par Description
    This is to initialize Time Client model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Time Client.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_time_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_TIME_CLIENT_CB           appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_TIME_CLIENT;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = time_client_cb;
    model.pub_cb = NULL;
    /* List of Opcodes */
    model.opcodes = time_client_opcode_list;
    model.num_opcodes = sizeof(time_client_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* Save Application Callback */
    time_client_appl_cb = appl_cb;
    /* TODO: Remove */
    time_client_model_handle = *model_handle;
    return retval;
}



/**
    \brief API to send acknowledged commands

    \par Description
    This is to initialize sending acknowledged commands.

    \param [in] req_opcode    Request Opcode.
    \param [in] param         Parameter associated with Request Opcode.
    \param [in] rsp_opcode    Response Opcode.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_time_client_send_reliable_pdu
(
    /* IN */ UINT32    req_opcode,
    /* IN */ void*     param,
    /* IN */ UINT32    rsp_opcode
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    retval = API_FAILURE;
    marker = 0;
    MS_IGNORE_UNUSED_PARAM(rsp_opcode);

    switch(req_opcode)
    {
    case MS_ACCESS_TIME_GET_OPCODE:
    {
        printf("MS_ACCESS_TIME_GET_OPCODE\n");
    }
    break;

    case MS_ACCESS_TIME_SET_OPCODE:
    {
        MS_STATE_TIME_STRUCT* param_p;
        UINT8 tai_seconds_com[5] = {0};
        printf(
            "MS_ACCESS_TIME_SET_OPCODE\n");
        param_p = (MS_STATE_TIME_STRUCT*) param;
        MS_PACK_LE_N_BYTE(&buffer[marker], param_p->tai_seconds,5);
        marker += 5;

        if(EM_mem_cmp(param_p->tai_seconds,tai_seconds_com,5) != 0)
        {
            buffer[marker] = param_p->subsecond;
            marker += 1;
            buffer[marker] = param_p->uncertainty;
            marker += 1;
            param_p->tai_utc_delta = (param_p->time_authority & 0x01) | (param_p->tai_utc_delta << 1);
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->tai_utc_delta);
            marker += 2;
            buffer[marker] = param_p->time_zone_offset;
            marker += 1;
        }
    }
    break;

    case MS_ACCESS_TIME_ZONE_GET_OPCODE:
    {
        printf("MS_ACCESS_TIME_ZONE_GET_OPCODE\n");
    }
    break;

    case MS_ACCESS_TIME_ZONE_SET_OPCODE:
    {
        MS_TIME_ZONE_SET_STRUCT* param_p;
        printf(
            "MS_ACCESS_TIME_ZONE_SET_OPCODE\n");
        param_p = (MS_TIME_ZONE_SET_STRUCT*) param;
        buffer[marker] = param_p->time_zone_offset_new;
        marker += 1;
        MS_PACK_LE_N_BYTE(&buffer[marker], param_p->tai_of_zone_change,5);
        marker += 5;
    }
    break;

    case MS_ACCESS_TAI_UTC_DELTA_GET_OPCODE:
    {
        printf("MS_ACCESS_TAI_UTC_DELTA_GET_OPCODE\n");
    }
    break;

    case MS_ACCESS_TAI_UTC_DELTA_SET_OPCODE:
    {
        MS_TAI_UTC_DELTA_SET_STRUCT* param_p;
        printf(
            "MS_ACCESS_TAI_UTC_DELTA_SET_OPCODE\n");
        param_p = (MS_TAI_UTC_DELTA_SET_STRUCT*) param;
        param_p->tai_utc_delta_new &= 0x7fff;
        param_p->tai_utc_delta_new |= (param_p->padding << 15);
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->tai_utc_delta_new);
        marker += 2;
        MS_PACK_LE_N_BYTE(&buffer[marker], param_p->tai_of_delta_change,5);
        marker += 5;
    }
    break;

    case MS_ACCESS_TIME_ROLE_GET_OPCODE:
    {
        printf("MS_ACCESS_TIME_ROLE_GET_OPCODE\n");
    }
    break;

    case MS_ACCESS_TIME_ROLE_SET_OPCODE:
    {
        MS_STATE_TIME_ROLE_STRUCT* param_p;
        printf(
            "MS_ACCESS_TIME_ROLE_SET_OPCODE\n");
        buffer[marker] = param_p->role;
        marker += 1;
    }
    break;

    default:
        printf("[TIME ERROR]CANNOT FIND OPCODE\n");
        break;
    }

    /* Publish - reliable */
    if (0 == marker)
    {
        pdu_ptr = NULL;
    }
    else
    {
        pdu_ptr = buffer;
    }

    retval = MS_access_publish
             (
                 &time_client_model_handle,
                 req_opcode,
                 pdu_ptr,
                 marker,
                 MS_TRUE
             );
    return retval;
}


/**
    \brief API to get Time client model handle

    \par Description
    This is to get the handle of Time client model.

    \param [out] model_handle   Address of model handle to be filled/returned.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_time_client_get_model_handle
(
    /* OUT */ MS_ACCESS_MODEL_HANDLE*   model_handle
)
{
    /* Fill Model Handle */
    * model_handle = time_client_model_handle;
    return API_SUCCESS;
}

/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(time_register_status_handler);
MODEL_OPCODE_HANDLER_EMPTY_DEF(time_status_handler);


/**
    \brief Access Layer Application Asynchronous Notification Callback.

    \par Description
    Access Layer calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] saddr         16 bit Source Address.
    \param [in] daddr         16 bit Destination Address.
    \param [in] appkey_handle AppKey Handle.
    \param [in] subnet_handle Subnet Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT time_client_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(saddr);
    MS_IGNORE_UNUSED_PARAM(daddr);
    MS_IGNORE_UNUSED_PARAM(subnet_handle);
    MS_IGNORE_UNUSED_PARAM(appkey_handle);
    retval = API_SUCCESS;

    switch(opcode)
    {
        #if 0

    case MS_ACCESS_SCENE_REGISTER_STATUS_OPCODE:
    {
        SCENE_CLIENT_TRC(
            "MS_ACCESS_SCENE_REGISTER_STATUS_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(scene_register_status_handler);
    }
    break;

    case MS_ACCESS_SCENE_STATUS_OPCODE:
    {
        SCENE_CLIENT_TRC(
            "MS_ACCESS_SCENE_STATUS_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(scene_status_handler);
    }
    break;
    #endif
    }

    /* Application callback */
    if (NULL != time_client_appl_cb)
    {
        time_client_appl_cb(handle, opcode, data_param, data_len);
    }

    return retval;
}

