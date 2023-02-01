/**
    \file generic_location_server.c
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "generic_location_server.h"
#include "MS_model_states.h"


/* --------------------------------------------- Global Definitions */



/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 generic_location_server_opcode_list[] =
{
    MS_ACCESS_GENERIC_LOCATION_LOCAL_GET_OPCODE,
    MS_ACCESS_GENERIC_LOCATION_GLOBAL_GET_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   generic_location_server_model_handle;
static MS_GENERIC_LOCATION_SERVER_CB       generic_location_server_appl_cb;


/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    \brief API to initialize Generic_Location Server model

    \par Description
    This is to initialize Generic_Location Server model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Generic_Location Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_generic_location_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_GENERIC_LOCATION_SERVER_CB appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    GENERIC_LOCATION_SERVER_TRC(
        "[GENERIC_LOCATION] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_GENERIC_LOCATION_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = generic_location_server_cb;
    model.pub_cb = generic_location_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = generic_location_server_opcode_list;
    model.num_opcodes = sizeof(generic_location_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* Save Application Callback */
    generic_location_server_appl_cb = appl_cb;
    /* TODO: Remove */
    generic_location_server_model_handle = *model_handle;
    return retval;
}



/**
    \brief API to send reply or to update state change

    \par Description
    This is to send reply for a request or to inform change in state.

    \param [in] ctx                     Context of the message.
    \param [in] current_state_params    Model specific current state parameters.
    \param [in] target_state_params     Model specific target state parameters (NULL: to be ignored).
    \param [in] remaining_time          Time from current state to target state (0: to be ignored).
    \param [in] ext_params              Additional parameters (NULL: to be ignored).

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_generic_location_server_state_update
(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    API_RESULT retval;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker;
    UINT32     opcode;
    MS_IGNORE_UNUSED_PARAM(target_state_params);
    MS_IGNORE_UNUSED_PARAM(remaining_time);
    MS_IGNORE_UNUSED_PARAM(ext_params);
    retval = API_FAILURE;
    marker = 0;
    GENERIC_LOCATION_SERVER_TRC(
        "[GENERIC_LOCATION_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    case MS_STATE_GENERIC_LOCATION_GLOBAL_T:
    {
        MS_STATE_GENERIC_LOCATION_GLOBAL_STRUCT* param_p;
        GENERIC_LOCATION_SERVER_TRC(
            "MS_ACCESS_GENERIC_LOCATION_GLOBAL_STATUS_OPCODE\n");
        param_p = (MS_STATE_GENERIC_LOCATION_GLOBAL_STRUCT*)current_state_params->state;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->global_latitude);
        marker += 4;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->global_longitude);
        marker += 4;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->global_altitude);
        marker += 2;
        /* Set Opcode */
        opcode = MS_ACCESS_GENERIC_LOCATION_GLOBAL_STATUS_OPCODE;
    }
    break;

    case MS_STATE_GENERIC_LOCATION_LOCAL_T:
    {
        MS_STATE_GENERIC_LOCATION_LOCAL_STRUCT* param_p;
        GENERIC_LOCATION_SERVER_TRC(
            "MS_ACCESS_GENERIC_LOCATION_LOCAL_STATUS_OPCODE\n");
        param_p = (MS_STATE_GENERIC_LOCATION_LOCAL_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->local_north);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->local_east);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->local_altitude);
        marker += 2;
        buffer[marker] = param_p->floor_number;
        marker += 1;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->uncertainty);
        marker += 2;
        /* Set Opcode */
        opcode = MS_ACCESS_GENERIC_LOCATION_LOCAL_STATUS_OPCODE;
    }
    break;

    default:
    {
        GENERIC_LOCATION_SERVER_ERR(
            "Invalid State Type: 0x%02X\n", current_state_params->state_type);
        return retval;
    }
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

    retval = MS_access_reply
             (
                 &ctx->handle,
                 ctx->daddr,
                 ctx->saddr,
                 ctx->subnet_handle,
                 ctx->appkey_handle,
                 ACCESS_INVALID_DEFAULT_TTL,
                 opcode,
                 pdu_ptr,
                 marker
             );
    return retval;
}

/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_location_local_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_location_global_get_handler)


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
API_RESULT generic_location_server_cb
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
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT  req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW      req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T        req_type;
    MS_ACCESS_MODEL_EXT_PARAMS*      ext_params_p;
    MS_ACCESS_MODEL_STATE_PARAMS     state_params;
    API_RESULT    retval;
    retval = API_SUCCESS;
    ext_params_p = NULL;
    /* Request Context */
    req_context.handle = *handle;
    req_context.saddr  = saddr;
    req_context.daddr  = daddr;
    req_context.subnet_handle = subnet_handle;
    req_context.appkey_handle = appkey_handle;
    /* Request Raw */
    req_raw.opcode = opcode;
    req_raw.data_param = data_param;
    req_raw.data_len = data_len;
    GENERIC_LOCATION_SERVER_TRC(
        "[GENERIC_LOCATION_SERVER] Callback. Opcode 0x%04X\n", opcode);
    GENERIC_LOCATION_SERVER_debug_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_GENERIC_LOCATION_LOCAL_GET_OPCODE:
    {
        GENERIC_LOCATION_SERVER_TRC(
            "MS_ACCESS_GENERIC_LOCATION_LOCAL_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(generic_location_local_get_handler);
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_GENERIC_LOCATION_LOCAL_T;
    }
    break;

    case MS_ACCESS_GENERIC_LOCATION_GLOBAL_GET_OPCODE:
    {
        GENERIC_LOCATION_SERVER_TRC(
            "MS_ACCESS_GENERIC_LOCATION_GLOBAL_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(generic_location_global_get_handler);
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_GENERIC_LOCATION_GLOBAL_T;
    }
    break;
    }

    /* Application callback */
    if (NULL != generic_location_server_appl_cb)
    {
        generic_location_server_appl_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
    }

    return retval;
}

/**
    \brief Access Layer Model Publication Timeout Callback.

    \par Description
    Access Layer calls the registered callback to indicate Publication Timeout
    for the associated model.

    \param [in]  handle        Model Handle.
    \param [out] blob          Blob if any or NULL.
*/
API_RESULT generic_location_server_publish_timout_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    return API_FAILURE;
}

