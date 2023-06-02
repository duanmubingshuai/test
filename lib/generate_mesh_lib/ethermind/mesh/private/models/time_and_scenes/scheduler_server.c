/**
    \file scheduler_server.c
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "scheduler_server.h"
#include "MS_model_states.h"


/* --------------------------------------------- Global Definitions */


/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 scheduler_server_opcode_list[] =
{
    MS_ACCESS_SCHEDULER_ACTION_GET_OPCODE,
    MS_ACCESS_SCHEDULER_GET_OPCODE
};

static DECL_CONST UINT32 scheduler_setup_server_opcode_list[] =
{
    MS_ACCESS_SCHEDULER_ACTION_SET_OPCODE,
    MS_ACCESS_SCHEDULER_ACTION_SET_UNACKNOWLEDGED_OPCODE
};

static MS_SCHEDULER_SERVER_CB       scheduler_server_appl_cb;


/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    API to initialize Scheduler Server model.
    Description
    This is to initialize Time Server model and to register with Acess layer.Parameters
    [in] element_handle Element identifier to be associated with the model instance.
    [in,out] time_model_handle Model identifier associated with the time model instance on successful initialization. After power cycle of an already provisioned node, the model handle will have valid value and the same will be reused for registration.
    [in,out] time_setup_model_handle Model identifier associated with the time setup model instance on successful initialization. After power cycle of an already provisioned node, the model handle will have valid value and the same will be reused for registration.
    [in] appl_cb Application Callback to be used by the Time Server.
    Returns
    API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_scheduler_server_init(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE     element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*        scheduler_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*        scheduler_setup_model_handle,
    /* IN */    MS_SCHEDULER_SERVER_CB       appl_cb
)
{
    API_RESULT              rslt;
    MS_ACCESS_NODE_ID       node;
    MS_ACCESS_MODEL         modl;
    /* Using default node ID */
    node = MS_ACCESS_DEFAULT_NODE_ID;
    printf(
        "[SCHEDULER_SERVER] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    modl.model_id.id = MS_MODEL_ID_SCHEDULER_SERVER;
    modl.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    modl.elem_handle = element_handle;
    /* Register Callbacks */
    modl.cb = scheduler_server_cb;
    modl.pub_cb = scheduler_server_publish_timout_cb;
    /* List of Opcodes */
    modl.opcodes = scheduler_server_opcode_list;
    modl.num_opcodes = sizeof(scheduler_server_opcode_list) / sizeof(UINT32);
    rslt = MS_access_register_model(
               node,
               &modl,
               scheduler_model_handle);
    /* Configure Model */
    modl.model_id.id = MS_MODEL_ID_SCHEDULER_SETUP_SERVER;
    modl.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    modl.elem_handle = element_handle;
    /* Register Callbacks */
    modl.cb = scheduler_server_cb;
    modl.pub_cb = scheduler_server_publish_timout_cb;
    /* List of Opcodes */
    modl.opcodes = scheduler_setup_server_opcode_list;
    modl.num_opcodes = sizeof(scheduler_setup_server_opcode_list) / sizeof(UINT32);
    rslt = MS_access_register_model(
               node,
               &modl,
               scheduler_setup_model_handle);
    /* Save Application Callback */
    scheduler_server_appl_cb = appl_cb;
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
API_RESULT MS_scheduler_server_state_update(
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
        "[SCHEDULER_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    case MS_STATE_SCHEDULER_SCHEDULES_T:
    {
        MS_STATE_SCHEDULER_SCHEDULES_STRUCT* param_p;
        printf(
            "MS_STATE_SCHEDULER_SCHEDULES_T\n");
        param_p = (MS_STATE_SCHEDULER_SCHEDULES_STRUCT*)current_state_params->state;
        MS_PACK_LE_2_BYTE(&buffer[marker], &param_p->schedules);
        marker += 2;
        /* Set Opcode */
        opcode = MS_ACCESS_SCHEDULER_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SCHEDULER_ENTRY_INDEX_T:
    case MS_STATE_SCHEDULER_ENTRY_T:
    {
        MS_STATE_SCHEDULER_ENTRY_STRUCT* param_p;
        printf(
            "MS_STATE_SCHEDULER_ENTRY_INDEX_T\n");
        param_p = (MS_STATE_SCHEDULER_ENTRY_STRUCT*)current_state_params->state;
        buffer[marker] = (param_p->index & 0xf) | 0x10;
        marker += 1;
        buffer[marker] = (param_p->year & 0x7f)|((param_p->month & 0x1) << 7);
        marker += 1;
        buffer[marker] = (param_p->month >> 1) & 0xff;
        marker += 1;
        buffer[marker] = ((param_p->month >> 9) & 0x07)|((param_p->day & 0x1f) << 3);
        marker += 1;
        buffer[marker] = (param_p->hour & 0x1f)|((param_p->minute & 0x07) << 5);
        marker += 1;
        buffer[marker] = ((param_p->minute >> 3) & 0x07)|((param_p->second & 0x1f) << 3);
        marker += 1;
        buffer[marker] = ((param_p->second >> 5) & 0x01)|((param_p->dayofweek & 0x7f) << 1);
        marker += 1;
        buffer[marker] = (param_p->action & 0x0f)|((param_p->transition_time & 0xf) << 4);
        marker += 1;
        buffer[marker] = (param_p->transition_time >> 4)|((param_p->scene_number & 0xf) << 4);
        marker += 1;
        buffer[marker] = (param_p->scene_number >> 4) & 0xff;
        marker += 1;
        /* Set Opcode */
        opcode = MS_ACCESS_SCHEDULER_ACTION_STATUS_OPCODE;
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
API_RESULT scheduler_server_cb(
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
    MS_STATE_SCHEDULER_ENTRY_INDEX_STRUCT   param;
    MS_STATE_SCHEDULER_ENTRY_STRUCT     param_entry;
//    MS_STATE_TIME_TAI_UTC_DELTA_STRUCT  param_delta;
//    MS_STATE_TIME_ROLE_STRUCT           param_role;
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
        "[SCHEDULER_SERVER] Callback. Opcode 0x%04X\n", opcode);
    appl_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_SCHEDULER_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SCHEDULER_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_SCHEDULER_SCHEDULES_T;
    }
    break;

    case MS_ACCESS_SCHEDULER_ACTION_GET_OPCODE:
    {
        printf(
            "MS_ACCESS_SCHEDULER_ACTION_GET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_GET;
        req_type.to_be_acked = 0x01;
        marker = 0;
        param.index = data_param[marker];
        marker += 1;

        if(param.index > 0x0f)
        {
            retval = API_SUCCESS;
            return retval;
        }

        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_SCHEDULER_ENTRY_INDEX_T;
        state_params.state = &param;
    }
    break;

    case MS_ACCESS_SCHEDULER_ACTION_SET_OPCODE:
    case MS_ACCESS_SCHEDULER_ACTION_SET_UNACKNOWLEDGED_OPCODE:
    {
        printf(
            "MS_ACCESS_SCHEDULER_ACTION_SET_OPCODE\n");
        /* Get Request Type */
        req_type.type = MS_ACCESS_MODEL_REQ_MSG_T_SET;

        if(opcode == MS_ACCESS_SCHEDULER_ACTION_SET_OPCODE)
            req_type.to_be_acked = 0x01;
        else
            req_type.to_be_acked = 0x00;

        marker = 0;
        param_entry.index = data_param[marker] & 0x0f;
        marker += 1;
        param_entry.year = (data_param[marker] & 0x7f)/*|0x01*/;

        if(param_entry.year > 0x64)
        {
            retval = API_SUCCESS;
            return retval;
        }

        UINT16 month;
        month = (data_param[marker] >> 7) | ((UINT16)(data_param[marker+1]) << 1)
                | ((UINT16)(data_param[marker+2] & 0x07) << 9);
        param_entry.month = month;
        marker += 2;
        param_entry.day = data_param[marker] >> 3;
        marker += 1;
        param_entry.hour = data_param[marker] & 0x1f;

        if(param_entry.hour > 0x19)
        {
            retval = API_SUCCESS;
            return retval;
        }

        UINT8 minute;
        minute = ((data_param[marker] >> 5) & 0x7) | (((data_param[marker+1]) & 0x7)<<3);
        param_entry.minute = minute;
        marker += 1;
        UINT8 second;
        second = (data_param[marker] >> 3) | ((data_param[marker + 1]&0x01) << 5);
        param_entry.second = second;
        marker += 1;
        param_entry.dayofweek = data_param[marker] >> 1;
        marker += 1;
        param_entry.action = data_param[marker] & 0xf;
        UINT8 transition_time;
        transition_time = (data_param[marker] >> 4) | ((data_param[marker + 1]&0x0f) << 4);
        param_entry.transition_time = transition_time;
        marker += 1;
        UINT16 scene_number;
        scene_number = (data_param[marker] >> 4) | ((UINT16)(data_param[marker+1]) << 4)
                       | ((UINT16)(data_param[marker+2] & 0x0f) << 12);
        param_entry.scene_number = scene_number;
        marker += 2;
        /* Assign reqeusted state type to the application */
        state_params.state_type = MS_STATE_SCHEDULER_ENTRY_T;
        state_params.state = &param_entry;
    }
    break;

    default:
        printf(
            "Invalid Opcode: 0x%02X\n", opcode);
        return retval;
    }

    /* Application callback */
    if (NULL != scheduler_server_appl_cb)
    {
        scheduler_server_appl_cb(&req_context, &req_raw, &req_type, &state_params, ext_params_p);
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
API_RESULT scheduler_server_publish_timout_cb(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    return API_FAILURE;
}

