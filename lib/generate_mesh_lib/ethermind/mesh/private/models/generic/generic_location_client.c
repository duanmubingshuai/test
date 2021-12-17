/**
    \file generic_location_client.c

    \brief This file defines the Mesh Generic Location Model Application Interface
    - includes Data Structures and Methods for Client.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "generic_location_client.h"


/* --------------------------------------------- Global Definitions */


/* --------------------------------------------- Static Global Variables */
static DECL_CONST UINT32 generic_location_client_opcode_list[] =
{
    MS_ACCESS_GENERIC_LOCATION_GLOBAL_STATUS_OPCODE,
    MS_ACCESS_GENERIC_LOCATION_LOCAL_STATUS_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   generic_location_client_model_handle;
static MS_GENERIC_LOCATION_CLIENT_CB       generic_location_client_appl_cb;


/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/**
    \brief API to initialize Generic_Location Client model

    \par Description
    This is to initialize Generic_Location Client model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Generic_Location Client.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_generic_location_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_GENERIC_LOCATION_CLIENT_CB appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    GENERIC_LOCATION_CLIENT_TRC(
        "[GENERIC_LOCATION] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_GENERIC_LOCATION_CLIENT;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = generic_location_client_cb;
    model.pub_cb = NULL;
    /* List of Opcodes */
    model.opcodes = generic_location_client_opcode_list;
    model.num_opcodes = sizeof(generic_location_client_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* Save Application Callback */
    generic_location_client_appl_cb = appl_cb;
    /* TODO: Remove */
    generic_location_client_model_handle = *model_handle;
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
API_RESULT MS_generic_location_client_send_reliable_pdu
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
    GENERIC_LOCATION_CLIENT_TRC(
        "[GENERIC_LOCATION_CLIENT] Send Reliable PDU. Req Opcode 0x%08X, Rsp Opcode 0x%08X\n",
        req_opcode, rsp_opcode);
    MS_IGNORE_UNUSED_PARAM(rsp_opcode);

    switch(req_opcode)
    {
    case MS_ACCESS_GENERIC_LOCATION_GLOBAL_GET_OPCODE:
    {
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_GLOBAL_GET_OPCODE\n");
    }
    break;

    case MS_ACCESS_GENERIC_LOCATION_GLOBAL_SET_OPCODE:
    {
        MS_GENERIC_LOCATION_GLOBAL_STRUCT* param_p;
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_GLOBAL_SET_OPCODE\n");
        param_p = (MS_GENERIC_LOCATION_GLOBAL_STRUCT*) param;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->global_latitude);
        marker += 4;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->global_longitude);
        marker += 4;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->global_altitude);
        marker += 2;
    }
    break;

    case MS_ACCESS_GENERIC_LOCATION_GLOBAL_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_GENERIC_LOCATION_GLOBAL_STRUCT* param_p;
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_GLOBAL_SET_UNACKNOWLEDGED_OPCODE\n");
        param_p = (MS_GENERIC_LOCATION_GLOBAL_STRUCT*) param;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->global_latitude);
        marker += 4;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], param_p->global_longitude);
        marker += 4;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], param_p->global_altitude);
        marker += 2;
    }
    break;

    case MS_ACCESS_GENERIC_LOCATION_LOCAL_GET_OPCODE:
    {
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_LOCAL_GET_OPCODE\n");
    }
    break;

    case MS_ACCESS_GENERIC_LOCATION_LOCAL_SET_OPCODE:
    {
        MS_GENERIC_LOCATION_LOCAL_STRUCT* param_p;
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_LOCAL_SET_OPCODE\n");
        param_p = (MS_GENERIC_LOCATION_LOCAL_STRUCT*) param;
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
    }
    break;

    case MS_ACCESS_GENERIC_LOCATION_LOCAL_SET_UNACKNOWLEDGED_OPCODE:
    {
        MS_GENERIC_LOCATION_LOCAL_STRUCT* param_p;
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_LOCAL_SET_UNACKNOWLEDGED_OPCODE\n");
        param_p = (MS_GENERIC_LOCATION_LOCAL_STRUCT*) param;
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
    }
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
                 &generic_location_client_model_handle,
                 req_opcode,
                 pdu_ptr,
                 marker,
                 MS_TRUE
             );
    return retval;
}


/**
    \brief API to get Generic_Location client model handle

    \par Description
    This is to get the handle of Generic_Location client model.

    \param [out] model_handle   Address of model handle to be filled/returned.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_generic_location_client_get_model_handle
(
    /* OUT */ MS_ACCESS_MODEL_HANDLE*   model_handle
)
{
    /* Fill Model Handle */
    * model_handle = generic_location_client_model_handle;
    return API_SUCCESS;
}

/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_location_global_status_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(generic_location_local_status_handler)


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
API_RESULT generic_location_client_cb
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
    GENERIC_LOCATION_CLIENT_TRC(
        "[GENERIC_LOCATION_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    GENERIC_LOCATION_CLIENT_debug_dump_bytes(data_param, data_len);

    switch(opcode)
    {
    case MS_ACCESS_GENERIC_LOCATION_GLOBAL_STATUS_OPCODE:
    {
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_GLOBAL_STATUS_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(generic_location_global_status_handler);
    }
    break;

    case MS_ACCESS_GENERIC_LOCATION_LOCAL_STATUS_OPCODE:
    {
        GENERIC_LOCATION_CLIENT_TRC(
            "MS_ACCESS_GENERIC_LOCATION_LOCAL_STATUS_OPCODE\n");
        MODEL_OPCODE_HANDLER_CALL(generic_location_local_status_handler);
    }
    break;
    }

    /* Application callback */
    if (NULL != generic_location_client_appl_cb)
    {
        generic_location_client_appl_cb(handle, opcode, data_param, data_len);
    }

    return retval;
}

