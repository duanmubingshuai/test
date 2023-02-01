/**
    \file scheduler_client.c

    \brief This file defines the Mesh Scheduler Model Application Interface
    - includes Data Structures and Methods for Client.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "scheduler_client.h"


/* --------------------------------------------- Global Definitions */


/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 scheduler_client_opcode_list[] =
{
    MS_ACCESS_SCHEDULER_ACTION_STATUS_OPCODE,
    MS_ACCESS_SCHEDULER_STATUS_OPCODE
};

static MS_ACCESS_MODEL_HANDLE   scheduler_client_model_handle;
static MS_SCHEDULER_CLIENT_CB        scheduler_client_appl_cb;


/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    \brief API to initialize Scheduler Client model

    \par Description
    This is to initialize Scheduler Client model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Time Client.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_scheduler_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_SCHEDULER_CLIENT_CB      appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_SCHEDULER_CLIENT;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = scheduler_client_cb;
    model.pub_cb = NULL;
    /* List of Opcodes */
    model.opcodes = scheduler_client_opcode_list;
    model.num_opcodes = sizeof(scheduler_client_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* Save Application Callback */
    scheduler_client_appl_cb = appl_cb;
    /* TODO: Remove */
    scheduler_client_model_handle = *model_handle;
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
API_RESULT MS_scheduler_client_send_reliable_pdu
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
        case MS_ACCESS_SCHEDULER_ACTION_SET_OPCODE:
        case MS_ACCESS_SCHEDULER_ACTION_SET_UNACKNOWLEDGED_OPCODE:
        {
            printf("MS_ACCESS_SCHEDULER_ACTION_SET_OPCODE\n");           
            MS_SCHEDULER_ACTION_SET_STRUCT* param_p;
            param_p = (MS_SCHEDULER_ACTION_SET_STRUCT*) param;

            buffer[marker] = (param_p->index & 0xf)|((param_p->year & 0xf)<<4);
            marker += 1;

            buffer[marker] = ((param_p->year >> 4) & 0x07)|((param_p->month & 0x1f) << 3);
            marker += 1;

            buffer[marker] = ((param_p->month >> 5) & 0x7f)|((param_p->day & 0x01) << 7);
            marker += 1;

            buffer[marker] = ((param_p->day >> 1) & 0x0f)|((param_p->hour & 0x0f) << 4);
            marker += 1;

            buffer[marker] = ((param_p->hour >> 4) & 0x01)|((param_p->minute & 0x3f) << 1)|((param_p->second & 0x01) << 7);
            marker += 1;

            buffer[marker] = ((param_p->second >> 1) & 0x1f)|((param_p->dayofweek & 0x07) << 5);
            marker += 1;

            buffer[marker] = ((param_p->dayofweek >> 3) & 0x0f)|((param_p->action & 0x0f) << 4);
            marker += 1;

            buffer[marker] = param_p->transition_time;
            marker += 1;

            MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->scene_number);
            marker += 2;
        }
        break;
        case MS_ACCESS_SCHEDULER_GET_OPCODE:
        {
            printf("MS_ACCESS_SCHEDULER_GET_OPCODE\n");           
        }
        break;
        case MS_ACCESS_SCHEDULER_ACTION_GET_OPCODE:
        {
            printf("MS_ACCESS_SCHEDULER_ACTION_GET_OPCODE\n");           
            MS_SCHEDULER_ACTION_GET_STRUCT* param_p;
            param_p = (MS_SCHEDULER_ACTION_GET_STRUCT*) param;

            buffer[marker] = param_p->index;
            marker += 1;
        }
        break;

    default:
        printf("[SCHEDULER ERROR]INVALID OPCODE\n");
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
                 &scheduler_client_model_handle,
                 req_opcode,
                 pdu_ptr,
                 marker,
                 MS_TRUE
             );
    return retval;
}


/**
    \brief API to get Scheduler client model handle

    \par Description
    This is to get the handle of Scheduler client model.

    \param [out] model_handle   Address of model handle to be filled/returned.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_scheduler_client_get_model_handle
(
    /* OUT */ MS_ACCESS_MODEL_HANDLE*   model_handle
)
{
    /* Fill Model Handle */
    * model_handle = scheduler_client_model_handle;
    return API_SUCCESS;
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
API_RESULT scheduler_client_cb
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
    }

    /* Application callback */
    if (NULL != scheduler_client_appl_cb)
    {
        scheduler_client_appl_cb(handle, opcode, data_param, data_len);
    }

    return retval;
}

