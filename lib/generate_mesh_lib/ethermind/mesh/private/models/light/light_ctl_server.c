/**
    \file light_ctl_server.c
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "light_ctl_server.h"
#include "MS_model_states.h"


/* --------------------------------------------- Global Definitions */



/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 light_ctl_server_opcode_list[] =
{
    MS_ACCESS_LIGHT_CTL_SET_OPCODE,
    MS_ACCESS_LIGHT_CTL_DEFAULT_GET_OPCODE,
    MS_ACCESS_LIGHT_CTL_GET_OPCODE,
    MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_GET_OPCODE,
    MS_ACCESS_LIGHT_CTL_SET_UNACKNOWLEDGED_OPCODE,
};

static DECL_CONST UINT32 light_ctl_setup_server_opcode_list[] =
{
    MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_SET_OPCODE,
    MS_ACCESS_LIGHT_CTL_DEFAULT_SET_OPCODE,
    MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_LIGHT_CTL_DEFAULT_SET_UNACKNOWLEDGED_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   light_ctl_server_model_handle;
static MS_ACCESS_MODEL_HANDLE   light_ctl_setup_server_model_handle;
static MS_LIGHT_CTL_SERVER_CB   light_ctl_server_appl_cb;


/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    \brief API to initialize Light_Ctl Server model

    \par Description
    This is to initialize Light_Ctl Server model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] ctl_model_handle
                     Model identifier associated with the Light CTL model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in, out] ctl_setup_model_handle
                     Model identifier associated with the Light CTL Setup model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Light_Ctl_Setup Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_light_ctl_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     ctl_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     ctl_setup_model_handle,
    /* IN */    MS_LIGHT_CTL_SERVER_CB      appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    LIGHT_CTL_SERVER_TRC(
        "[LIGHT_CTL] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_LIGHT_CTL_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = light_ctl_server_cb;
    model.pub_cb = light_ctl_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = light_ctl_server_opcode_list;
    model.num_opcodes = sizeof(light_ctl_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 ctl_model_handle
             );
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_LIGHT_CTL_SETUP_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = light_ctl_server_cb;
    model.pub_cb = light_ctl_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = light_ctl_setup_server_opcode_list;
    model.num_opcodes = sizeof(light_ctl_setup_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 ctl_setup_model_handle
             );
    /* Save Application Callback */
    light_ctl_server_appl_cb = appl_cb;
    /* TODO: Remove */
    light_ctl_server_model_handle = *ctl_model_handle;
    light_ctl_setup_server_model_handle = *ctl_setup_model_handle;
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
API_RESULT MS_light_ctl_server_state_update
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
    LIGHT_CTL_SERVER_TRC(
        "[LIGHT_CTL_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    case MS_STATE_LIGHT_CTL_T:
    {
        MS_STATE_LIGHT_CTL_STRUCT* param_p;
        LIGHT_CTL_SERVER_TRC(
            "MS_ACCESS_LIGHT_CTL_STATUS_OPCODE\n");
        param_p = (MS_STATE_LIGHT_CTL_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->ctl_lightness);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->ctl_temperature);
        marker += 2;

        if (0x00 != param_p->transition_time)
        {
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->target_ctl_lightness);
            marker += 2;
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->target_ctl_temperature);
            marker += 2;
            buffer[marker] = param_p->transition_time;
            marker += 1;
        }

        /* Set Opcode */
        opcode = MS_ACCESS_LIGHT_CTL_STATUS_OPCODE;
    }
    break;

    case MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_T:
    {
        MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_STRUCT* param_p;
        LIGHT_CTL_SERVER_TRC(
            "MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_STATUS_OPCODE\n");
        param_p = (MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_STRUCT*)current_state_params->state;
        buffer[marker] = param_p->status;
        marker += 1;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->ctl_temperature_range_min);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->ctl_temperature_range_max);
        marker += 2;
        /* Set Opcode */
        opcode = MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_STATUS_OPCODE;
    }
    break;

    case MS_STATE_LIGHT_CTL_DEFAULT_T:
    {
        MS_STATE_LIGHT_CTL_DEFAULT_STRUCT* param_p;
        LIGHT_CTL_SERVER_TRC(
            "MS_ACCESS_LIGHT_CTL_DEFAULT_STATUS_OPCODE\n");
        param_p = (MS_STATE_LIGHT_CTL_DEFAULT_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->ctl_lightness);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->ctl_temperature);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->ctl_delta_uv);
        marker += 2;
        /* Set Opcode */
        opcode = MS_ACCESS_LIGHT_CTL_DEFAULT_STATUS_OPCODE;
    }
    break;

    default:
    {
        LIGHT_CTL_SERVER_ERR(
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
MODEL_OPCODE_HANDLER_EMPTY_DEF(light_ctl_set_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(light_ctl_default_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(light_ctl_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(light_ctl_temperature_range_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(light_ctl_temperature_range_set_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(light_ctl_default_set_handler)


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
API_RESULT light_ctl_server_cb
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
    MS_ACCESS_MODEL_STATE_PARAMS                  state_params;
    MS_STATE_LIGHT_CTL_STRUCT                     param;
    MS_STATE_LIGHT_CTL_DEFAULT_STRUCT             param_default;
    MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_STRUCT   param_range;
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
    LIGHT_CTL_SERVER_TRC(
        "[LIGHT_CTL_SERVER] Callback. Opcode 0x%04X\n", opcode);
    LIGHT_CTL_SERVER_debug_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_LIGHT_CTL_GET_OPCODE:
    {
        LIGHT_CTL_SERVER_TRC(
            "MS_ACCESS_LIGHT_CTL_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(light_ctl_get_handler);
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_LIGHT_CTL_T;
    }
    break;

    case MS_ACCESS_LIGHT_CTL_SET_UNACKNOWLEDGED_OPCODE: /* Fall Through */
    case MS_ACCESS_LIGHT_CTL_SET_OPCODE:
    {
        MODEL_OPCODE_HANDLER_CALL(light_ctl_set_handler);
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;

        if (MS_ACCESS_LIGHT_CTL_SET_OPCODE == opcode)
        {
            LIGHT_CTL_SERVER_TRC(
                "MS_ACCESS_LIGHT_CTL_SET_OPCODE\n");
            req_type.to_be_acked = 0x01;
        }
        else
        {
            LIGHT_CTL_SERVER_TRC(
                "MS_ACCESS_LIGHT_CTL_SET_UNACKNOWLEDGED_OPCODE\n");
            req_type.to_be_acked = 0x00;
        }

        /* Decode Parameters */
        marker = 0;
        MS_UNPACK_LE_2_BYTE(&param.ctl_lightness, &data_param[marker]);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&param.ctl_temperature, &data_param[marker]);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&param.ctl_delta_uv, &data_param[marker]);
        marker += 2;
        param.tid = data_param[marker];
        marker += 1;
        LIGHT_CTL_SERVER_TRC(
            "ctl_lightness 0x%04X\n", param.ctl_lightness);
        LIGHT_CTL_SERVER_TRC(
            "ctl_temperature 0x%04X\n", param.ctl_temperature);

        if ((0x0320 > param.ctl_temperature) || (0x4E20 < param.ctl_temperature))
        {
            LIGHT_CTL_SERVER_ERR("ctl_temperature 0x%04X out of range. Dropping...\n",
                                 param.ctl_temperature);
            return ACCESS_INVALID_PARAMETER_VALUE;
        }

        LIGHT_CTL_SERVER_TRC(
            "ctl_delta_uv 0x%04X\n", param.ctl_delta_uv);
        LIGHT_CTL_SERVER_TRC("TID 0x%04X\n", param.tid);

        /* Parameter Validation */
        if (9 == data_len)
        {
            param.transition_time = data_param[marker];
            marker += 1;
            param.delay = data_param[marker];
            marker += 1;
        }
        else
        {
            param.transition_time = 0x00;
        }

        /* Assign decoded state parameter to provide to application */
        state_params.state_type = MS_STATE_LIGHT_CTL_T;
        state_params.state = &param;
    }
    break;

    case MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_GET_OPCODE:
    {
        LIGHT_CTL_SERVER_TRC(
            "MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(light_ctl_temperature_range_get_handler);
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_T;
    }
    break;

    case MS_ACCESS_LIGHT_CTL_DEFAULT_GET_OPCODE:
    {
        LIGHT_CTL_SERVER_TRC(
            "MS_ACCESS_LIGHT_CTL_DEFAULT_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(light_ctl_default_get_handler);
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_LIGHT_CTL_DEFAULT_T;
    }
    break;

    case MS_ACCESS_LIGHT_CTL_DEFAULT_SET_UNACKNOWLEDGED_OPCODE: /* Fall Through */
    case MS_ACCESS_LIGHT_CTL_DEFAULT_SET_OPCODE:
    {
        MODEL_OPCODE_HANDLER_CALL(light_ctl_default_set_handler);
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;

        if (MS_ACCESS_LIGHT_CTL_DEFAULT_SET_OPCODE == opcode)
        {
            LIGHT_CTL_SERVER_TRC(
                "MS_ACCESS_LIGHT_CTL_DEFAULT_SET_OPCODE\n");
            req_type.to_be_acked = 0x01;
        }
        else
        {
            LIGHT_CTL_SERVER_TRC(
                "MS_ACCESS_LIGHT_CTL_DEFAULT_SET_UNACKNOWLEDGED_OPCODE\n");
            req_type.to_be_acked = 0x00;
        }

        /* Decode Parameters */
        marker = 0;
        MS_UNPACK_LE_2_BYTE(&param_default.ctl_lightness, &data_param[marker]);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&param_default.ctl_temperature, &data_param[marker]);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&param_default.ctl_delta_uv, &data_param[marker]);
        marker += 2;
        LIGHT_CTL_SERVER_TRC(
            "ctl_lightness 0x%04X\n", param_default.ctl_lightness);
        LIGHT_CTL_SERVER_TRC(
            "ctl_temperature 0x%04X\n", param_default.ctl_temperature);

        if ((0x0320 > param_default.ctl_temperature) || (0x4E20 < param_default.ctl_temperature))
        {
            LIGHT_CTL_SERVER_ERR("ctl_temperature 0x%04X out of range. Dropping...\n",
                                 param_default.ctl_temperature);
            return ACCESS_INVALID_PARAMETER_VALUE;
        }

        LIGHT_CTL_SERVER_TRC(
            "ctl_delta_uv 0x%04X\n", param_default.ctl_delta_uv);
        /* Parameter Validation */
        /* Assign decoded state parameter to provide to application */
        state_params.state_type = MS_STATE_LIGHT_CTL_DEFAULT_T;
        state_params.state = &param_default;
    }
    break;

    case MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_SET_UNACKNOWLEDGED_OPCODE: /* Fall Through */
    case MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_SET_OPCODE:
    {
        MODEL_OPCODE_HANDLER_CALL(light_ctl_temperature_range_set_handler);
        /* Set Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;

        if (MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_SET_OPCODE == opcode)
        {
            LIGHT_CTL_SERVER_TRC(
                "MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_SET_OPCODE\n");
            req_type.to_be_acked = 0x01;
        }
        else
        {
            LIGHT_CTL_SERVER_TRC(
                "MS_ACCESS_LIGHT_CTL_TEMPERATURE_RANGE_SET_UNACKNOWLEDGED_OPCODE\n");
            req_type.to_be_acked = 0x00;
        }

        /* Decode Parameters */
        marker = 0;
        MS_UNPACK_LE_2_BYTE(&param_range.ctl_temperature_range_min, &data_param[marker]);
        marker += 2;
        MS_UNPACK_LE_2_BYTE(&param_range.ctl_temperature_range_max, &data_param[marker]);
        marker += 2;
        LIGHT_CTL_SERVER_TRC(
            "ctl_temperature_range_min 0x%04X\n", param_range.ctl_temperature_range_min);
        LIGHT_CTL_SERVER_TRC(
            "ctl_temperature_range_max 0x%04X\n", param_range.ctl_temperature_range_max);

        /* Parameter Validation */
        if ((param_range.ctl_temperature_range_min > param_range.ctl_temperature_range_max) ||
                (0x0320 > param_range.ctl_temperature_range_min) || (0x4E20 < param_range.ctl_temperature_range_max))
        {
            /* TODO: add macro define */
            /**
                Table 7.2:
                0x00 - Success
                0x01 - Cannot Set Range Min
                0x02 - Cannot Set Range Max
            */
            LIGHT_CTL_SERVER_ERR("ctl_temperature MIN/MAX out of range. Dropping...\n");
            return ACCESS_INVALID_PARAMETER_VALUE;
        }

        /* Assign decoded state parameter to provide to application */
        state_params.state_type = MS_STATE_LIGHT_CTL_TEMPERATURE_RANGE_T;
        state_params.state = &param_range;
    }
    break;
    }

    /* Application callback */
    if (NULL != light_ctl_server_appl_cb)
    {
        light_ctl_server_appl_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
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
API_RESULT light_ctl_server_publish_timout_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    return API_FAILURE;
}

