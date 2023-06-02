/**
    \file time_server.c
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "time_server.h"
#include "MS_model_states.h"


/* --------------------------------------------- Global Definitions */


/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 time_server_opcode_list[] =
{
    MS_ACCESS_TIME_GET_OPCODE,
    MS_ACCESS_TIME_STATUS_OPCODE,
    MS_ACCESS_TIME_ZONE_GET_OPCODE,
    MS_ACCESS_TAI_UTC_DELTA_GET_OPCODE,
};

static DECL_CONST UINT32 time_setup_server_opcode_list[] =
{
    MS_ACCESS_TIME_SET_OPCODE,
    MS_ACCESS_TIME_ZONE_SET_OPCODE,
    MS_ACCESS_TAI_UTC_DELTA_SET_OPCODE,
    MS_ACCESS_TIME_ROLE_GET_OPCODE,
    MS_ACCESS_TIME_ROLE_SET_OPCODE,
};

static MS_TIME_SERVER_CB       time_server_appl_cb;


/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    API to initialize Time Server model.
    Description
    This is to initialize Time Server model and to register with Acess layer.Parameters
    [in] element_handle Element identifier to be associated with the model instance.
    [in,out] time_model_handle Model identifier associated with the time model instance on successful initialization. After power cycle of an already provisioned node, the model handle will have valid value and the same will be reused for registration.
    [in,out] time_setup_model_handle Model identifier associated with the time setup model instance on successful initialization. After power cycle of an already provisioned node, the model handle will have valid value and the same will be reused for registration.
    [in] appl_cb Application Callback to be used by the Time Server.
    Returns
    API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_time_server_init(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     time_setup_model_handle,
    /* IN */    MS_TIME_SERVER_CB           appl_cb
)
{
    API_RESULT              rslt;
    MS_ACCESS_NODE_ID       node;
    MS_ACCESS_MODEL         modl;
    /* Using default node ID */
    node = MS_ACCESS_DEFAULT_NODE_ID;
    printf(
        "[TIMER_SERVER] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    modl.model_id.id = MS_MODEL_ID_TIME_SERVER;
    modl.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    modl.elem_handle = element_handle;
    /* Register Callbacks */
    modl.cb = time_server_cb;
    modl.pub_cb = time_server_publish_timout_cb;
    /* List of Opcodes */
    modl.opcodes = time_server_opcode_list;
    modl.num_opcodes = sizeof(time_server_opcode_list) / sizeof(UINT32);
    rslt = MS_access_register_model(
               node,
               &modl,
               time_model_handle);
    /* Configure Model */
    modl.model_id.id = MS_MODEL_ID_TIME_SETUP_SERVER;
    modl.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    modl.elem_handle = element_handle;
    /* Register Callbacks */
    modl.cb = time_server_cb;
    modl.pub_cb = time_server_publish_timout_cb;
    /* List of Opcodes */
    modl.opcodes = time_setup_server_opcode_list;
    modl.num_opcodes = sizeof(time_setup_server_opcode_list) / sizeof(UINT32);
    rslt = MS_access_register_model(
               node,
               &modl,
               time_setup_model_handle);
    /* Save Application Callback */
    time_server_appl_cb = appl_cb;
    return rslt;
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
API_RESULT MS_time_server_state_update(
    /* IN */ MS_ACCESS_MODEL_REQ_MSG_CONTEXT*     ctx,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        current_state_params,
    /* IN */ MS_ACCESS_MODEL_STATE_PARAMS*        target_state_params,
    /* IN */ UINT16                               remaining_time,
    /* IN */ MS_ACCESS_MODEL_EXT_PARAMS*          ext_params
)
{
    API_RESULT rslt = API_FAILURE;
    /* TODO: Check what should be maximum length */
    UCHAR      buffer[32];
    UCHAR*     pdu_ptr;
    UINT16     marker   = 0;
    UINT32     opcode;
    MS_IGNORE_UNUSED_PARAM(target_state_params);
    MS_IGNORE_UNUSED_PARAM(remaining_time);
    MS_IGNORE_UNUSED_PARAM(ext_params);
    printf(
        "[TIME_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    case MS_STATE_TIME_T:
    {
        MS_STATE_TIME_STRUCT* param_p;
        UINT8 tai_seconds_com[5] = {0};
        printf(
            "MS_ACCESS_TIME_STATUS_OPCODE\n");
        param_p = (MS_STATE_TIME_STRUCT*)current_state_params->state;
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

        /* Set Opcode */
        opcode = MS_ACCESS_TIME_STATUS_OPCODE;
    }
    break;

    case MS_STATE_TIME_ZONE_T:
    {
        MS_STATE_TIME_ZONE_STRUCT* param_p;
        printf(
            "MS_ACCESS_TIME_ZONE_STATUS_OPCODE\n");
        param_p = (MS_STATE_TIME_ZONE_STRUCT*)current_state_params->state;
        buffer[marker] = param_p->time_zone_offset_current;
        marker += 1;
        buffer[marker] = param_p->time_zone_offset_new;
        marker += 1;
        MS_PACK_LE_N_BYTE(&buffer[marker], param_p->tai_of_zone_change,5);
        marker += 5;
        /* Set Opcode */
        opcode = MS_ACCESS_TIME_ZONE_STATUS_OPCODE;
    }
    break;

    case MS_STATE_TIME_TAI_UTC_DELTA_T:
    {
        MS_STATE_TIME_TAI_UTC_DELTA_STRUCT* param_p;
        printf(
            "MS_ACCESS_TAI_UTC_DELTA_STATUS_OPCODE\n");
        param_p = (MS_STATE_TIME_TAI_UTC_DELTA_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], (param_p->tai_utc_delta_current & 0x7fff));
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], (param_p->tai_utc_delta_new & 0x7fff));
        marker += 2;
        MS_PACK_LE_N_BYTE(&buffer[marker], param_p->tai_of_delta_change,5);
        marker += 5;
        /* Set Opcode */
        opcode = MS_ACCESS_TAI_UTC_DELTA_STATUS_OPCODE;
    }
    break;

    case MS_STATE_TIME_ROLE_T:
    {
        MS_STATE_TIME_ROLE_STRUCT* param_p;
        printf(
            "MS_ACCESS_TIME_ROLE_STATUS_OPCODE\n");
        param_p = (MS_STATE_TIME_ROLE_STRUCT*)current_state_params->state;
        buffer[marker] = param_p->role;
        marker += 1;
        /* Set Opcode */
        opcode = MS_ACCESS_TIME_ROLE_STATUS_OPCODE;
    }
    break;

    default:
    {
        printf(
            "Invalid State Type: 0x%02X\n", current_state_params->state_type);
        return rslt;
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

    rslt = MS_access_reply
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
    return rslt;
}

/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_location_local_get_handler);
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_location_global_get_handler);


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
API_RESULT time_server_cb(
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
    MS_ACCESS_MODEL_REQ_MSG_CONTEXT     req_context;
    MS_ACCESS_MODEL_REQ_MSG_RAW         req_raw;
    MS_ACCESS_MODEL_REQ_MSG_T           req_type;
    MS_ACCESS_MODEL_EXT_PARAMS*         ext_params_p;
    MS_ACCESS_MODEL_STATE_PARAMS        state_params;
    MS_STATE_TIME_STRUCT                param;
    MS_STATE_TIME_ZONE_STRUCT           param_zone;
    MS_STATE_TIME_TAI_UTC_DELTA_STRUCT  param_delta;
    MS_STATE_TIME_ROLE_STRUCT           param_role;
    UINT16        marker;
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
    printf(
        "[TIME_SERVER] Callback. Opcode 0x%04X\n", opcode);

    switch(opcode)
    {
    case MS_ACCESS_TIME_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_TIME_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_T;
    }
    break;

    case MS_ACCESS_TIME_SET_OPCODE:
    {
        printf(
            "MS_ACCESS_TIME_SET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;
        UINT8 tai_seconds_com[5] = {0};
        /* Decode Parameters */
        marker = 0;
        MS_UNPACK_BE_N_BYTE(&param.tai_seconds[0],&data_param[marker],5);
        marker += 5;

        if(EM_mem_cmp(&param.tai_seconds[0],tai_seconds_com,5) != 0)
        {
            param.subsecond = data_param[marker];
            marker += 1;
            param.uncertainty = data_param[marker];
            marker += 1;
            MS_UNPACK_LE_2_BYTE(&param.tai_utc_delta,&data_param[marker]);
            marker += 2;
            param.time_authority = param.tai_utc_delta & 0x01;
            param.tai_utc_delta >>= 1;
            param.time_zone_offset = data_param[marker];
            marker += 1;
        }

        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_T;
        state_params.state = &param;
    }
    break;

    case MS_ACCESS_TIME_ROLE_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_TIME_ROLE_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_ROLE_T;
    }
    break;

    case MS_ACCESS_TIME_ROLE_SET_OPCODE:
    {
        printf(
            "MS_ACCESS_TIME_ROLE_SET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;
        /* Decode Parameters */
        marker = 0;

        if(data_param[marker] > 3)
        {
            retval = API_SUCCESS;
            return retval;
        }

        param_role.role = data_param[marker];
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_ROLE_T;
        state_params.state = &param_role;
    }
    break;

    case MS_ACCESS_TIME_ZONE_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_TIME_ZONE_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_ZONE_T;
    }
    break;

    case MS_ACCESS_TIME_ZONE_SET_OPCODE:
    {
        printf(
            "MS_ACCESS_TIME_ZONE_SET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;
        /* Decode Parameters */
        marker = 0;
        param_zone.time_zone_offset_new = data_param[marker];
        marker += 1;
        MS_UNPACK_BE_N_BYTE(&param_zone.tai_of_zone_change[0],&data_param[marker],5);
        marker += 5;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_ZONE_T;
        state_params.state = &param_zone;
    }
    break;

    case MS_ACCESS_TAI_UTC_DELTA_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_TAI_UTC_DELTA_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_TAI_UTC_DELTA_T;
    }
    break;

    case MS_ACCESS_TAI_UTC_DELTA_SET_OPCODE:
    {
        printf(
            "MS_ACCESS_TAI_UTC_DELTA_SET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;
        req_type.to_be_acked = 0x01;
        /* Decode Parameters */
        marker = 0;
        MS_UNPACK_LE_2_BYTE(&param_delta.tai_utc_delta_new,&data_param[marker]);
        marker += 2;
        MS_UNPACK_BE_N_BYTE(&param_delta.tai_of_delta_change[0],&data_param[marker],5);
        marker += 5;
        printf("tai_utc_delta_new %x\n",param_delta.tai_utc_delta_new);
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_TIME_TAI_UTC_DELTA_T;
        state_params.state = &param_delta;
    }
    break;
    }

    /* Application callback */
    if (NULL != time_server_appl_cb)
    {
        time_server_appl_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
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
API_RESULT time_server_publish_timout_cb(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    return API_FAILURE;
}

