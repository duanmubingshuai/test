/**
    \file scene_server.c
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "scene_server.h"
#include "MS_model_states.h"
#include "MS_generic_default_transition_time_api.h"

/* --------------------------------------------- Global Definitions */
#if 0

    #ifdef SCENE_SERVER_TRC
        #undef SCENE_SERVER_TRC
        #define SCENE_SERVER_TRC printf
    #endif /* CONFIG_TRC */

    #ifdef SCENE_SERVER_INF
        #undef SCENE_SERVER_INF
        #define SCENE_SERVER_INF printf
    #endif /* SCENE_SERVER_INF */

    #ifdef SCENE_SERVER_ERR
        #undef SCENE_SERVER_ERR
        #define SCENE_SERVER_ERR printf
    #endif /* SCENE_SERVER_ERR */

    #ifdef SCENE_SERVER_debug_dump_bytes
        #undef SCENE_SERVER_debug_dump_bytes
        #define SCENE_SERVER_debug_dump_bytes appl_dump_bytes
    #endif /* SCENE_SERVER_debug_dump_bytes */

#endif /* 0 */


/* --------------------------------------------- Static Global Variables */
/* ------- Scene Data Structures */
#define MS_MAX_SCENE_REGS     16

static API_RESULT ms_scene_recall(MS_ACCESS_MODEL_HANDLE* handle, UINT16 scene_number, UINT8 delay, UINT8 transition_time);

/* Status Codes */
#define MS_SCENE_STATUS_SUCCESS      0x00
#define MS_SCENE_STATUS_REG_FULL     0x01
#define MS_SCENE_STATUS_NOT_FOUND    0x02

typedef struct _MS_SCENE_REGISTER_ENTRY
{
    /**
        Scene Number.

        0x0000 is invalid. Will mark empty Scene
    */
    UINT16     scene_number;

    /**
        Context associated with Scene.
        Can be used to recall, store, delete etc.
        Assigned by specific model/elements/instances.
    */
    void*      context;
} MS_SCENE_REGISTER_ENTRY;

/**
    When any of the element's state that is marked as “Stored with Scene?
    has changed not as a result of a Scene Recall operation,
    the value of the Current Scene state shall be set to 0x0000.
    When a scene transition is in progress, the value of the Current Scene
    state shall be set to 0x0000.
*/
static UINT16 ms_current_scene;

/**
    The Target Scene state is a 16-bit value that contains the target Scene Number
    when a scene transition is in progress. When the scene transition is in progress
    and the target Scene Number is deleted from a Scene Register state as a result of
    Scene Delete operation, the Target Scene state shall be set to 0x0000.
    When the scene transition is in progress and a new Scene Number is stored in
    the Scene Register as a result of Scene Store operation, the Target Scene state
    shall be set to the new Scene Number.
    When the scene transition is not in progress, the value of the Target Scene state
    shall be set to 0x0000.
*/
static UINT16 ms_target_scene;
static MS_SCENE_REGISTER_ENTRY ms_scene_reg_list[MS_MAX_SCENE_REGS];

/* Maintain - In transition and transition time */
static UINT8    ms_scene_in_transtion;
static UINT8    ms_scene_transition_time;
static UINT16   ms_transition_time_handle;

static DECL_CONST UINT32 scene_server_opcode_list[] =
{
    MS_ACCESS_SCENE_RECALL_OPCODE,
    MS_ACCESS_SCENE_RECALL_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_SCENE_GET_OPCODE,
    MS_ACCESS_SCENE_REGISTER_GET_OPCODE,
};

static DECL_CONST UINT32 scene_setup_server_opcode_list[] =
{
    MS_ACCESS_SCENE_STORE_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_SCENE_STORE_OPCODE,
    MS_ACCESS_SCENE_DELETE_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_SCENE_DELETE_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   scene_server_model_handle;
static MS_ACCESS_MODEL_HANDLE   scene_setup_server_model_handle;
static MS_SCENE_SERVER_CB       scene_server_appl_cb;

static void ms_scene_frame_status_response(UINT16 status_code, UINT8* buffer, UINT16* length, UINT32* response_code, UINT8 read_remaining_time);

/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    \brief API to initialize Scene Server model

    \par Description
    This is to initialize Scene Server model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] scene_model_handle
                     Model identifier associated with the Scene model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in, out] scene_setup_model_handle
                     Model identifier associated with the Scene Setup model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Scene Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_scene_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     scene_model_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     scene_setup_model_handle,
    /* IN */    MS_SCENE_SERVER_CB appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    SCENE_SERVER_TRC(
        "[SCENE] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_SCENE_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = scene_server_cb;
    model.pub_cb = scene_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = scene_server_opcode_list;
    model.num_opcodes = sizeof(scene_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 scene_model_handle
             );
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_SCENE_SETUP_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = scene_server_cb;
    model.pub_cb = scene_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = scene_setup_server_opcode_list;
    model.num_opcodes = sizeof(scene_setup_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 scene_setup_model_handle
             );
    /* Save Application Callback */
    scene_server_appl_cb = appl_cb;
    /* TODO: Remove */
    scene_server_model_handle = *scene_model_handle;
    scene_setup_server_model_handle = *scene_setup_model_handle;
    /* Init Data Structures */
    ms_current_scene = 0x0000;
    SCENE_SERVER_TRC("[0] Current Scene:0x%04X\n", ms_current_scene);
    ms_target_scene = 0x0000;
    ms_scene_in_transtion = 0x00;
    ms_scene_transition_time = 0x00;
    EM_mem_set(ms_scene_reg_list, 0, sizeof(ms_scene_reg_list));
    /** Dummy fill */
    #if 0
    ms_scene_reg_list[0].scene_number = 0x0001;
    ms_scene_reg_list[1].scene_number = 0x0002;
    #endif /* 0 */
    return retval;
}

static void ms_scene_transition_start_cb(void* blob)
{
    UINT32 index;
    SCENE_SERVER_TRC("[SCENE_SERVER] Transition Start\n");
    index = (UINT32)blob;
    scene_server_appl_cb(&scene_server_model_handle, MS_SCENE_EVENT_RECALL_START, &index, sizeof(index), ms_scene_reg_list[index].context);
}

static void ms_scene_transition_complete_cb(void* blob)
{
    UINT32    index;
    UINT8     buffer[8];
    UINT32    response_code;
    UINT16    status;
    UINT16    marker;
    SCENE_SERVER_TRC("[SCENE_SERVER] Transition Complete\n");
    index = (UINT32)blob;
    scene_server_appl_cb(&scene_server_model_handle, MS_SCENE_EVENT_RECALL_COMPLETE, &index, sizeof(index), ms_scene_reg_list[index].context);
    ms_current_scene = ms_target_scene;
    ms_target_scene = 0x0000;
    ms_scene_in_transtion = 0x00;
    /* Try to publish this */
    status = MS_SCENE_STATUS_SUCCESS;
    ms_scene_frame_status_response(status, buffer, &marker, &response_code, MS_FALSE);
    MS_access_publish
    (
        &scene_server_model_handle,
        response_code,
        buffer,
        marker,
        1 /* reliable - not used */
    );
}


/**
    Scene Store is an operation of storing values of a present state of an element.
    The structure and meaning of the stored state is determined by a model.
    States to be stored are specified by each model. The Scene Store operation
    shall persistently store all values of all states marked as stored with Scene?
    for all models present on an element of a node.
    A scene is referenced in memory by a Scene Number, which is stored in Scene Register state.
    Values in the Scene Register state are compared with the Scene Number that is to be stored.
    If a matching Scene Number is found, the container for the first scene with a matching
    Scene Number is updated, and the operation completes with success. If no matching Scene Number
    is found, the first Scene Register entry with an unset value is used and is assigned to
    the Scene Number of the stored scene, and the operation completes with success.
    If there is no available entry in the Scene Register to store the scene, the scene is not stored,
    and the operation completes with failure. When the scene transition is in progress,
    the target state of the transition for each model is stored.
*/
API_RESULT ms_scene_store(MS_ACCESS_MODEL_HANDLE* handle, UINT16 scene_number)
{
    UINT32 index, free_index;
    free_index = MS_MAX_SCENE_REGS;

    for (index = 0; index < MS_MAX_SCENE_REGS; index++)
    {
        if (scene_number == ms_scene_reg_list[index].scene_number)
        {
            /* Store Current State */
            ms_scene_reg_list[index].context = scene_server_appl_cb(handle, MS_SCENE_EVENT_STORE, &index, sizeof(index), NULL);
            return MS_SCENE_STATUS_SUCCESS;
        }

        if (MS_MAX_SCENE_REGS == free_index)
        {
            if (0x0000 == ms_scene_reg_list[index].scene_number)
            {
                free_index = index;
            }
        }
    }

    /* Check if any free scene available */
    if (MS_MAX_SCENE_REGS != free_index)
    {
        ms_scene_reg_list[free_index].scene_number = scene_number;
        /* Store Current State */
        ms_scene_reg_list[free_index].context = scene_server_appl_cb(handle, MS_SCENE_EVENT_STORE, &free_index, sizeof(free_index), NULL);
        /**
            When a Scene Store operation or a Scene Recall operation completes with success,
            the Current Scene state value shall be to the Scene Number used during that operation.
        */
        ms_current_scene = scene_number;
        SCENE_SERVER_TRC("[1] Current Scene:0x%04X\n", ms_current_scene);
        return MS_SCENE_STATUS_SUCCESS;
    }

    /* Full */
    return MS_SCENE_STATUS_REG_FULL;
}


/**
    A scene is deleted from memory by referencing the Scene Number.
    Values in the Scene Register state are compared with the Scene Number of the scene
    that is to be deleted, and the first matching scene is deleted from the Scene Register state.
    When a scene is deleted when a scene transition to the deleted Scene Number is in progress,
    the scene transition shall be terminated, but individual model transitions shall not be terminated.
*/
static API_RESULT ms_scene_delete(MS_ACCESS_MODEL_HANDLE* handle, UINT16 scene_number)
{
    UINT32 index;

    for (index = 0; index < MS_MAX_SCENE_REGS; index++)
    {
        if (scene_number == ms_scene_reg_list[index].scene_number)
        {
            /* Store Current State */
            scene_server_appl_cb(handle, MS_SCENE_EVENT_DELETE, &index, sizeof(index), ms_scene_reg_list[index].context);
            /* Mark Free */
            ms_scene_reg_list[index].scene_number = 0x0000;
            ms_scene_reg_list[index].context = NULL;

            /**
                When the Current Scene Number is deleted from a Scene Register state
                as a result of Scene Delete operation, the Current Scene state shall be set to 0x0000.
            */
            if (ms_current_scene == scene_number)
            {
                ms_current_scene = 0x0000;
                SCENE_SERVER_TRC("[2] Current Scene:0x%04X\n", ms_current_scene);
            }

            return MS_SCENE_STATUS_SUCCESS;
        }
    }

    /* Not Found */
    return MS_SCENE_STATUS_NOT_FOUND;
}

/**
    Scene Recall is an operation of recalling stored values of states and applying
    them to the state of an element. The structure and meaning of the stored state
    is determined by a model. States to be recalled are specified by each model.
    The Scene Recall operation shall recall all values for all states specified as
    'Stored with Scene' for all models present on an element.
    A scene is recalled from memory by referencing the Scene Number.
    Values in the Scene Register state are compared with the Scene Number value
    that is to be recalled. If a matching Scene Number is found, the first matching scene
    is recalled by starting the transition of all models present on an element to the recalled
    states, and the operation completes with success. If there is no matching Scene Number
    in the Scene Register state, the operation completes with failure.
*/
static API_RESULT ms_scene_recall(MS_ACCESS_MODEL_HANDLE* handle, UINT16 scene_number, UINT8 delay, UINT8 transition_time)
{
    UINT32 index;
    SCENE_SERVER_TRC("Recall: Scene Number:0x%04X\n", scene_number);

    for (index = 0; index < MS_MAX_SCENE_REGS; index++)
    {
        if (scene_number == ms_scene_reg_list[index].scene_number)
        {
            break;
        }
    }

    /* Check if match found */
    if (MS_MAX_SCENE_REGS != index)
    {
        /* Check if Default Transition Time is configured */
        if ((0 == delay) && (0 == transition_time))
        {
            API_RESULT retval;
            MS_STATE_GENERIC_DEFAULT_TRANSITION_TIME_STRUCT  default_time;
            retval = MS_generic_default_transition_time_server_get_time(&default_time);

            if (API_SUCCESS == retval)
            {
                /* TODO: Have a macro for the conversion */
                transition_time = (UCHAR)(default_time.default_transition_number_of_steps);
                transition_time |= (UCHAR)(default_time.default_transition_step_resolution << 6);
                SCENE_SERVER_TRC(
                    "Re-constructed Transition Time: 0x%02X\n", transition_time);
            }
        }

        if ((0 == delay) && (0 == transition_time))
        {
            /**
                When a Scene Store operation or a Scene Recall operation completes with success,
                the Current Scene state value shall be to the Scene Number used during that operation.
            */
            /* Recall State */
            scene_server_appl_cb(handle, MS_SCENE_EVENT_RECALL_IMMEDIATE, &index, sizeof(index), ms_scene_reg_list[index].context);
            ms_current_scene = scene_number;
            SCENE_SERVER_TRC("[4] Current Scene:0x%04X\n", ms_current_scene);
        }
        else
        {
            /* TODO: Set the status based on return value */
            {
                MS_ACCESS_STATE_TRANSITION_TYPE   transition;
                transition.delay = delay;
                transition.transition_time = transition_time;
                transition.blob = (void*)index;
                transition.transition_start_cb = ms_scene_transition_start_cb;
                transition.transition_complete_cb = ms_scene_transition_complete_cb;
                MS_common_start_transition_timer
                (
                    &transition,
                    &ms_transition_time_handle
                );
                ms_scene_in_transtion = 0x01;
                ms_scene_transition_time = transition_time;
            }
            /* During transition Current Time shall be set to 0x0000 */
            ms_current_scene = 0x0000;
            SCENE_SERVER_TRC("[8] Current Scene:0x%04X\n", ms_current_scene);
            ms_target_scene = scene_number;
        }

        return MS_SCENE_STATUS_SUCCESS;
    }

    /* Not Found */
    return MS_SCENE_STATUS_NOT_FOUND;
}

#if 0
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
API_RESULT MS_scene_server_state_update
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
    SCENE_SERVER_TRC(
        "[SCENE_SERVER] State Update.\n");

    switch (current_state_params->state_type)
    {
    case MS_STATE_SCENE_STATUS_T:
    {
        MS_STATE_SCENE_STATUS_STRUCT* param_p;
        SCENE_SERVER_TRC(
            "MS_ACCESS_SCENE_STATUS_OPCODE\n");
        param_p = (MS_STATE_SCENE_STATUS_STRUCT*)current_state_params->state;
        buffer[marker] = param_p->status_code;
        marker += 1;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->current_scene);
        marker += 2;

        if (0x0000 != param_p->target_scene)
        {
            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->target_scene);
            marker += 2;

            if (0 != param_p->remaining_time)
            {
                buffer[marker] = param_p->remaining_time;
                marker += 1;
            }
        }

        /* Set Opcode */
        opcode = MS_ACCESS_SCENE_STATUS_OPCODE;
    }
    break;

    case MS_STATE_SCENE_REGISTER_STATUS_T:
    {
        MS_STATE_SCENE_REGISTER_STATUS_STRUCT* param_p;
        SCENE_SERVER_TRC(
            "MS_ACCESS_SCENE_REGISTER_STATUS_OPCODE\n");
        param_p = (MS_STATE_SCENE_REGISTER_STATUS_STRUCT*)current_state_params->state;
        buffer[marker] = param_p->status_code;
        marker += 1;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->current_scene);
        marker += 2;
        {
            UINT16 count;

            for(count = 0; count < param_p->scenes_count; count++)
            {
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->scenes[count]);
                marker += 2;
            }
        }
        /* Set Opcode */
        opcode = MS_ACCESS_SCENE_REGISTER_STATUS_OPCODE;
    }
    break;

    default:
    {
        SCENE_SERVER_ERR(
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
#endif /* 0 */

static void ms_scene_frame_reg_status_response(UINT16 status_code, UINT8* buffer, UINT16* length, UINT32* response_code)
{
    UINT32 marker;
    *response_code = MS_ACCESS_SCENE_REGISTER_STATUS_OPCODE;
    marker = 0;
    buffer[marker] = (UINT8)status_code;
    marker += 1;
    MS_PACK_LE_2_BYTE_VAL(&buffer[marker], ms_current_scene);
    marker += 2;

    if (MS_SCENE_STATUS_SUCCESS == status_code)
    {
        UINT16 index;

        for (index = 0; index < MS_MAX_SCENE_REGS; index++)
        {
            if (0x0000 != ms_scene_reg_list[index].scene_number)
            {
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], ms_scene_reg_list[index].scene_number);
                marker += 2;
            }
        }
    }

    *length = (UINT16)marker;
}

static void ms_scene_frame_status_response(UINT16 status_code, UINT8* buffer, UINT16* length, UINT32* response_code, UINT8 read_remaining_time)
{
    UINT32 marker;
    *response_code = MS_ACCESS_SCENE_STATUS_OPCODE;
    marker = 0;
    buffer[marker] = (UINT8)status_code;
    marker += 1;
    MS_PACK_LE_2_BYTE_VAL(&buffer[marker], ms_current_scene);
    marker += 2;

    if (0x0000 != ms_target_scene)
    {
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], ms_target_scene);
        marker += 2;

        if (0 != ms_scene_transition_time)
        {
            if (MS_TRUE == read_remaining_time)
            {
                UINT8 transition_time;
                API_RESULT ret;
                ret = MS_common_get_remaining_transition_time_with_offset
                      (
                          ms_transition_time_handle,
                          200, /* 200ms */
                          &transition_time
                      );

                if (API_SUCCESS == ret)
                {
                    buffer[marker] = transition_time;
                    marker += 1;
                }
            }
            else
            {
                buffer[marker] = ms_scene_transition_time;
                marker += 1;
            }
        }
    }

    *length = (UINT16)marker;
}

/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(scene_recall_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(scene_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(scene_register_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(scene_store_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(scene_delete_handler)

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
API_RESULT scene_server_cb
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
    UINT16        scene_number;
    UINT8         to_be_acked, to_publish;
    UINT16        marker;
    API_RESULT    retval;
    UINT8         tid;
    UINT8         buffer[48];
    UINT32        response_code;
    UINT8         transition_time;
    UINT8         delay;
    UINT16        status;
    retval = API_SUCCESS;
    to_be_acked = 0x00;
    to_publish = MS_FALSE;
    marker = 0;
    SCENE_SERVER_TRC(
        "[SCENE_SERVER] Callback. Opcode 0x%04X\n", opcode);
    SCENE_SERVER_debug_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_SCENE_STORE_UNACKNOWLEDGED_OPCODE: /* Fall Through */
    case MS_ACCESS_SCENE_STORE_OPCODE:
    {
        MODEL_OPCODE_HANDLER_CALL(scene_store_handler);
        to_publish = MS_TRUE;

        if (MS_ACCESS_SCENE_STORE_OPCODE == opcode)
        {
            SCENE_SERVER_TRC(
                "MS_ACCESS_SCENE_STORE_OPCODE\n");
            to_be_acked = 0x01;
        }
        else
        {
            SCENE_SERVER_TRC(
                "MS_ACCESS_SCENE_STORE_UNACKNOWLEDGED_OPCODE\n");
            to_be_acked = 0x00;
        }

        /* Decode Parameters */
        MS_UNPACK_LE_2_BYTE(&scene_number, &data_param[marker]);
        marker += 2;
        SCENE_SERVER_TRC(
            "Scene Number 0x%04X\n", scene_number);

        /* Parameter Validation */
        if (0x0000 == scene_number)
        {
            SCENE_SERVER_ERR("Invalid scene Number: 0x%04X. Dropping\n",
                             scene_number);
            return API_FAILURE;
        }

        status = ms_scene_store(handle, scene_number);
        ms_scene_frame_reg_status_response(status, buffer, &marker, &response_code);
    }
    break;

    case MS_ACCESS_SCENE_RECALL_UNACKNOWLEDGED_OPCODE: /* Fall Through */
    case MS_ACCESS_SCENE_RECALL_OPCODE:
    {
        MODEL_OPCODE_HANDLER_CALL(scene_recall_handler);
        to_publish = MS_TRUE;

        if (MS_ACCESS_SCENE_RECALL_OPCODE == opcode)
        {
            SCENE_SERVER_TRC(
                "MS_ACCESS_SCENE_RECALL_OPCODE\n");
            to_be_acked = 0x01;
        }
        else
        {
            SCENE_SERVER_TRC(
                "MS_ACCESS_SCENE_RECALL_UNACKNOWLEDGED_OPCODE\n");
            to_be_acked = 0x00;
        }

        /* Decode Parameters */
        MS_UNPACK_LE_2_BYTE(&scene_number, &data_param[marker]);
        marker += 2;
        SCENE_SERVER_TRC("Scene Number 0x%04X\n", scene_number);

        /* Parameter Validation */
        if (0x0000 == scene_number)
        {
            SCENE_SERVER_ERR("Invalid scene Number: 0x%04X. Dropping\n",
                             scene_number);
            return API_FAILURE;
        }

        tid = data_param[marker];
        marker += 1;
        /* TODO: Not handling TID now */
        SCENE_SERVER_TRC("TID:0x%02X\n", tid);

        /* Check if optional parameters are present */
        if (5 == data_len)
        {
            transition_time = data_param[marker];
            marker += 1;
            delay = data_param[marker];
            SCENE_SERVER_TRC(
                "Transition Time:0x%02X, Delay:0x%02X\n",
                transition_time, delay);
        }
        else
        {
            transition_time = 0x00;
            delay = 0x00;
        }

        status = ms_scene_recall(handle, scene_number, delay, transition_time);
        ms_scene_frame_status_response(status, buffer, &marker, &response_code, MS_FALSE);
    }
    break;

    case MS_ACCESS_SCENE_GET_OPCODE:
    {
        SCENE_SERVER_TRC(
            "MS_ACCESS_SCENE_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(scene_get_handler);
        to_be_acked = 0x01;
        status = API_SUCCESS;
        ms_scene_frame_status_response(status, buffer, &marker, &response_code, MS_TRUE);
    }
    break;

    case MS_ACCESS_SCENE_REGISTER_GET_OPCODE:
    {
        SCENE_SERVER_TRC(
            "MS_ACCESS_SCENE_REGISTER_GET_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(scene_register_get_handler);
        to_be_acked = 0x01;
        status = API_SUCCESS;
        ms_scene_frame_reg_status_response(status, buffer, &marker, &response_code);
    }
    break;

    case MS_ACCESS_SCENE_DELETE_UNACKNOWLEDGED_OPCODE: /* Fall Through */
    case MS_ACCESS_SCENE_DELETE_OPCODE:
    {
        MODEL_OPCODE_HANDLER_CALL(scene_delete_handler);
        to_publish = MS_TRUE;

        if (MS_ACCESS_SCENE_DELETE_OPCODE == opcode)
        {
            SCENE_SERVER_TRC(
                "MS_ACCESS_SCENE_DELETE_OPCODE\n");
            to_be_acked = 0x01;
        }
        else
        {
            SCENE_SERVER_TRC(
                "MS_ACCESS_SCENE_DELETE_UNACKNOWLEDGED_OPCODE\n");
            to_be_acked = 0x00;
        }

        /* Decode Parameters */
        MS_UNPACK_LE_2_BYTE(&scene_number, &data_param[marker]);
        marker += 2;
        SCENE_SERVER_TRC(
            "Scene Number 0x%04X\n", scene_number);

        /* Parameter Validation */
        if (0x0000 == scene_number)
        {
            SCENE_SERVER_ERR("Invalid scene Number: 0x%04X. Dropping\n",
                             scene_number);
            return API_FAILURE;
        }

        status = ms_scene_delete(handle, scene_number);
        ms_scene_frame_reg_status_response(status, buffer, &marker, &response_code);
    }
    break;
    }

    if (0x01 == to_be_acked)
    {
        retval = MS_access_reply_and_publish
                 (
                     handle,
                     daddr,
                     saddr,
                     subnet_handle,
                     appkey_handle,
                     ACCESS_INVALID_DEFAULT_TTL,
                     response_code,
                     buffer,
                     marker,
                     to_publish
                 );
    }
    else
    {
        /* Try to publish this */
        MS_access_publish
        (
            handle,
            response_code,
            buffer,
            marker,
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
API_RESULT scene_server_publish_timout_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    return API_FAILURE;
}

