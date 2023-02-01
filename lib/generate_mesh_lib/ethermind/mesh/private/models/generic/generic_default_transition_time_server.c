/**
    \file generic_default_transition_time_server.c
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "generic_default_transition_time_server.h"
#include "MS_model_states.h"


/* --------------------------------------------- Global Definitions */



/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 generic_default_transition_time_server_opcode_list[] =
{
    MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_OPCODE,
    MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_GET_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   generic_default_transition_time_server_model_handle;
static MS_GENERIC_DEFAULT_TRANSITION_TIME_SERVER_CB       generic_default_transition_time_server_appl_cb;
static MS_STATE_GENERIC_DEFAULT_TRANSITION_TIME_STRUCT    generic_default_transition_time;

/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    \brief API to initialize Generic_Default_Transition_Time Server model

    \par Description
    This is to initialize Generic_Default_Transition_Time Server model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Generic_Default_Transition_Time Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_generic_default_transition_time_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_GENERIC_DEFAULT_TRANSITION_TIME_SERVER_CB appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    GENERIC_DEFAULT_TRANSITION_TIME_SERVER_TRC(
        "[GENERIC_DEFAULT_TRANSITION_TIME] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_GENERIC_DEFAULT_TRANSITION_TIME_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = generic_default_transition_time_server_cb;
    model.pub_cb = generic_default_transition_time_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = generic_default_transition_time_server_opcode_list;
    model.num_opcodes = sizeof(generic_default_transition_time_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* Save Application Callback */
    generic_default_transition_time_server_appl_cb = appl_cb;
    /* TODO: Remove */
    generic_default_transition_time_server_model_handle = *model_handle;
    generic_default_transition_time.default_transition_number_of_steps = 0;
    generic_default_transition_time.default_transition_step_resolution = 0;
    return retval;
}


/**
    \brief API to get default transition time

    \par Description
    This is to get default transition time.

    \param [out] default_time    Default Transition Time.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_generic_default_transition_time_server_get_time
(
    /* OUT */ MS_STATE_GENERIC_DEFAULT_TRANSITION_TIME_STRUCT*     default_time
)
{
    API_RESULT retval;

    /* Paramter Check */
    if (NULL == default_time)
    {
        retval = ACCESS_INVALID_PARAMETER_VALUE;
    }
    else
    {
        /* Copy current default */
        *default_time = generic_default_transition_time;
        retval = API_SUCCESS;
    }

    return retval;
}


/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_default_transition_time_set_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_default_transition_time_get_handler)

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
API_RESULT generic_default_transition_time_server_cb
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
    MS_STATE_GENERIC_DEFAULT_TRANSITION_TIME_STRUCT  param;
    UINT16        marker;
    API_RESULT    retval;
    UINT8         transition_time;
    UINT8         to_be_acked;
    retval = API_SUCCESS;
    to_be_acked = 0x00;
    GENERIC_DEFAULT_TRANSITION_TIME_SERVER_TRC(
        "[GENERIC_DEFAULT_TRANSITION_TIME_SERVER] Callback. Opcode 0x%04X\n", opcode);
    #ifdef GENERIC_DEFAULT_TRANSITION_TIME_SERVER_DEBUG
    GENERIC_DEFAULT_TRANSITION_TIME_SERVER_debug_dump_bytes(data_param, data_len);
    #else
    MS_IGNORE_UNUSED_PARAM(data_len);
    #endif /* GENERIC_DEFAULT_TRANSITION_TIME_SERVER_DEBUG */

    switch(opcode)
    {
    case MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_UNACKNOWLEDGED_OPCODE: /* Fall Through */
    case MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_OPCODE:
    {
        MODEL_OPCODE_HANDLER_CALL(generic_default_transition_time_set_handler);

        if (MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_OPCODE == opcode)
        {
            GENERIC_DEFAULT_TRANSITION_TIME_SERVER_TRC(
                "MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_OPCODE\n");
            to_be_acked = 0x01;
        }
        else
        {
            GENERIC_DEFAULT_TRANSITION_TIME_SERVER_TRC(
                "MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_UNACKNOWLEDGED_OPCODE\n");
            to_be_acked = 0x00;
        }

        /* Decode Parameters */
        marker = 0;
        param.default_transition_number_of_steps = (UCHAR)(data_param[marker] & 0x3F);
        param.default_transition_step_resolution = (UCHAR)(data_param[marker] >> 6);
        marker += 1;
        GENERIC_DEFAULT_TRANSITION_TIME_SERVER_TRC(
            "default_transition_number_of_steps 0x%02X\n", param.default_transition_number_of_steps);
        GENERIC_DEFAULT_TRANSITION_TIME_SERVER_TRC(
            "default_transition_step_resolution 0x%02X\n", param.default_transition_step_resolution);

        /* Parameter Validation */
        if (0x3F == param.default_transition_number_of_steps)
        {
            GENERIC_DEFAULT_TRANSITION_TIME_SERVER_ERR(
                "Invalid default_transition_number_of_steps 0x%02X. Dropping\n", param.default_transition_number_of_steps);
            return ACCESS_INVALID_PARAMETER_VALUE;
        }

        /* Save Default Transition Time */
        generic_default_transition_time = param;
    }
    break;

    case MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_GET_OPCODE:
    {
        GENERIC_DEFAULT_TRANSITION_TIME_SERVER_TRC(
            "MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(generic_default_transition_time_get_handler);
        to_be_acked = 0x01;
    }
    break;
    }

    transition_time = (UCHAR)(generic_default_transition_time.default_transition_number_of_steps);
    transition_time |= (UCHAR)(generic_default_transition_time.default_transition_step_resolution << 6);

    if (0x01 == to_be_acked)
    {
        retval = MS_access_reply
                 (
                     handle,
                     daddr,
                     saddr,
                     subnet_handle,
                     appkey_handle,
                     ACCESS_INVALID_DEFAULT_TTL,
                     MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_STATUS_OPCODE,
                     &transition_time,
                     sizeof(transition_time)
                 );
    }
    else if (MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_SET_UNACKNOWLEDGED_OPCODE == opcode)
    {
        /* Try to publish this */
        MS_access_publish
        (
            handle,
            MS_ACCESS_GENERIC_DEFAULT_TRANSITION_TIME_STATUS_OPCODE,
            &transition_time,
            sizeof(transition_time),
            1 /* reliable - not used */
        );
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
API_RESULT generic_default_transition_time_server_publish_timout_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    return API_FAILURE;
}

