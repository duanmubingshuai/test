/**
    \file config_client.c

    \brief This file defines the Mesh Configuration Model Application Interface
    - includes Data Structures and Methods for Client.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "config_internal.h"

#if 0

    #ifdef CONFIG_TRC
        #undef CONFIG_TRC
        #define CONFIG_TRC printf
    #endif /* CONFIG_TRC */

    #ifdef CONFIG_INF
        #undef CONFIG_INF
        #define CONFIG_INF printf
    #endif /* CONFIG_INF */

    #ifdef CONFIG_ERR
        #undef CONFIG_ERR
        #define CONFIG_ERR printf
    #endif /* CONFIG_ERR */

    #ifdef CONFIG_debug_dump_bytes
        #undef CONFIG_debug_dump_bytes
        #define CONFIG_debug_dump_bytes appl_dump_bytes
    #endif /* CONFIG_debug_dump_bytes */

#endif /* 0 */


#ifdef MS_MODEL_CONFIG

/* --------------------------------------------- Global Definitions */
/* Macro to define an empty Config Opcode Handler */
#define CONFIG_OPCODE_HANDLER_EMPTY_DEF(x) \
    static API_RESULT (x) \
    ( \
      MS_ACCESS_MODEL_HANDLE * handle, \
      MS_NET_ADDR              saddr, \
      MS_NET_ADDR              daddr, \
      MS_SUBNET_HANDLE         subnet_handle, \
      MS_APPKEY_HANDLE         appkey_handle, \
      UINT32                   opcode, \
      UCHAR                  * data_param, \
      UINT16                   data_len \
    ) \
    { \
        API_RESULT retval; \
        \
        MS_IGNORE_UNUSED_PARAM(handle); \
        MS_IGNORE_UNUSED_PARAM(saddr); \
        MS_IGNORE_UNUSED_PARAM(daddr); \
        MS_IGNORE_UNUSED_PARAM(subnet_handle); \
        MS_IGNORE_UNUSED_PARAM(appkey_handle); \
        MS_IGNORE_UNUSED_PARAM(opcode); \
        MS_IGNORE_UNUSED_PARAM(data_param); \
        MS_IGNORE_UNUSED_PARAM(data_len); \
        \
        retval = API_SUCCESS; \
        \
        return retval; \
    }

/* Callback Handler */
#define CONFIG_CLIENT_OPCODE_HANDLER(handler) \
    (handler) (handle, saddr, daddr, subnet_handle, appkey_handle, opcode, data_param, data_len)
/* --------------------------------------------- Static Global Variables */
static API_RESULT config_pack_key_indices
(
    /* IN */    UINT16     key_count,
    /* IN */    UINT16*    key_input_list,
    /* OUT */   UCHAR*     key_output_list,
    /* INOUT */ UINT16*    key_output_list_length
);

/* TBD: Remove the Opcodes not appropriate for Configure Model Client role */
static DECL_CONST UINT32 config_client_opcode_list[] =
{
    MS_ACCESS_CONFIG_GATT_PROXY_STATUS_OPCODE,
    MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_STATUS_OPCODE,
    MS_ACCESS_CONFIG_SIG_MODEL_APP_LIST_OPCODE,
    MS_ACCESS_CONFIG_COMPOSITION_DATA_STATUS_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE,
    MS_ACCESS_CONFIG_NETKEY_LIST_OPCODE,
    MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_STATUS_OPCODE,
    MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_LIST_OPCODE,
    MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_LIST_OPCODE,
    MS_ACCESS_CONFIG_APPKEY_LIST_OPCODE,
    MS_ACCESS_CONFIG_APPKEY_STATUS_OPCODE,
    MS_ACCESS_CONFIG_RELAY_STATUS_OPCODE,
    MS_ACCESS_CONFIG_FRIEND_STATUS_OPCODE,
    MS_ACCESS_CONFIG_MODEL_APP_STATUS_OPCODE,
    MS_ACCESS_CONFIG_NODE_RESET_STATUS_OPCODE,
    MS_ACCESS_CONFIG_DEFAULT_TTL_STATUS_OPCODE,
    MS_ACCESS_CONFIG_NODE_IDENTITY_STATUS_OPCODE,
    MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_STATUS_OPCODE,
    MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_STATUS_OPCODE,
    MS_ACCESS_CONFIG_NETWORK_TRANSMIT_STATUS_OPCODE,
    MS_ACCESS_CONFIG_BEACON_STATUS_OPCODE,
    MS_ACCESS_CONFIG_MODEL_PUBLICATION_STATUS_OPCODE,
    MS_ACCESS_CONFIG_NETKEY_STATUS_OPCODE,
    MS_ACCESS_CONFIG_VENDOR_MODEL_APP_LIST_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   config_client_model_handle;
static MS_CONFIG_MODEL_CB       config_client_appl_cb;

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */
/**
    \brief API to initialize Configuration Client model

    \par Description
    This is to initialize Configuration Client model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in] appl_cb    Application Callback to be used by the Configuration Client.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_config_client_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle,
    /* IN */    MS_CONFIG_MODEL_CB          appl_cb
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    CONFIG_TRC(
        "[CONFIG] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_CONFIG_CLIENT;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = config_client_cb;
    model.pub_cb = NULL;
    /* List of Opcodes */
    model.opcodes = config_client_opcode_list;
    model.num_opcodes = sizeof(config_client_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* Save Application Callback */
    config_client_appl_cb = appl_cb;
    config_client_model_handle = *model_handle;
    return retval;
}


API_RESULT MS_config_client_set_publish_address
(
    /* IN */ MS_NET_ADDR    pub_addr
)
{
    API_RESULT retval;
    MS_ACCESS_PUBLISH_INFO  publish_info;
    MS_ACCESS_DEV_KEY_HANDLE dev_key_handle;
    /* Set Publish Information */
    EM_mem_set(&publish_info, 0, sizeof(publish_info));
    publish_info.addr.use_label = MS_FALSE;
    publish_info.addr.addr = pub_addr;
    publish_info.remote = MS_FALSE;
    retval = MS_access_cm_get_device_key_handle
             (
                 publish_info.addr.addr,
                 &dev_key_handle
             );

    if (API_SUCCESS == retval)
    {
        publish_info.appkey_index = MS_CONFIG_LIMITS(MS_MAX_APPS) + dev_key_handle;
//        CONSOLE_OUT("DevKey -> AppKey Index: 0x%04X\n", publish_info.appkey_index);
    }

    retval = MS_access_cm_set_model_publication
             (
                 config_client_model_handle,
                 &publish_info
             );

    if (API_SUCCESS == retval)
    {
//        CONSOLE_OUT
//        ("Publish Address is set Successfully.\n");
    }
    else
    {
//        CONSOLE_OUT
//        ("Failed to set publish address. Status 0x%04X\n", retval);
    }

    return retval;
}

/**
    \brief API to set configuration server

    \par Description
    This is to sets the information about server which is to be configured.

    \param [in] server_addr   Address of Configuration Server.
    \param [in] dev_key       Device Key of Configuration Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_config_client_set_server
(
    /* IN */ MS_NET_ADDR    server_addr,
    /* IN */ UCHAR*         dev_key
)
{
    API_RESULT retval;
    MS_ACCESS_PUBLISH_INFO  publish_info;
    MS_IGNORE_UNUSED_PARAM(dev_key);
    /* Set Publish Information */
    EM_mem_set(&publish_info, 0, sizeof(publish_info));
    publish_info.addr.use_label = MS_FALSE;
    publish_info.addr.addr = server_addr;
    /* TODO: Hardcoding Device Key 0 for now */
    publish_info.appkey_index = MS_CONFIG_LIMITS(MS_MAX_APPS);
    publish_info.remote = MS_FALSE;
    retval = MS_access_cm_set_model_publication
             (
                 config_client_model_handle,
                 &publish_info
             );
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
API_RESULT MS_config_client_send_reliable_pdu
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
    CONFIG_TRC(
        "[CONFIG] Send Reliable PDU. Req Opcode 0x%08X, Rsp Opcode 0x%08X\n",
        req_opcode, rsp_opcode);

    switch(req_opcode)
    {
    /* Get secure network beacon state */
    case MS_ACCESS_CONFIG_BEACON_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_BEACON_GET_OPCODE\n");
    }
    break;

    /* Get default TTL state */
    case MS_ACCESS_CONFIG_DEFAULT_TTL_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_DEFAULT_TTL_GET_OPCODE\n");
    }
    break;

    /* Get GATT proxy state */
    case MS_ACCESS_CONFIG_GATT_PROXY_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_GATT_PROXY_GET_OPCODE\n");
    }
    break;

    /* Get relay state */
    case MS_ACCESS_CONFIG_RELAY_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_RELAY_GET_OPCODE\n");
    }
    break;

    /* Get Netkey list */
    case MS_ACCESS_CONFIG_NETKEY_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETKEY_GET_OPCODE\n");
    }
    break;

    /* Reset a node */
    case MS_ACCESS_CONFIG_NODE_RESET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NODE_RESET_OPCODE\n");
    }
    break;

    /* Get friend state */
    case MS_ACCESS_CONFIG_FRIEND_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_FRIEND_GET_OPCODE\n");
    }
    break;

    /* Get heartbeat publication state */
    case MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_GET_OPCODE\n");
    }
    break;

    /* Get heartbeat subscription state */
    case MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_GET_OPCODE\n");
    }
    break;

    /* Get Network transmit state */
    case MS_ACCESS_CONFIG_NETWORK_TRANSMIT_GET_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETWORK_TRANSMIT_GET_OPCODE\n");
    }
    break;

    /* Set secure network beacon state */
    case MS_ACCESS_CONFIG_BEACON_SET_OPCODE:
    {
        ACCESS_CONFIG_BEACON_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_BEACON_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_BEACON_SET_PARAM*)param;
        buffer[marker] = set_param->beacon;
        marker += 1;
    }
    break;

    /* Get composition data state */
    case MS_ACCESS_CONFIG_COMPOSITION_DATA_GET_OPCODE:
    {
        ACCESS_CONFIG_COMPDATA_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_COMPOSITION_DATA_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_COMPDATA_GET_PARAM*)param;
        buffer[marker] = get_param->page;
        marker += 1;
    }
    break;

    /* Set default TTL state */
    case MS_ACCESS_CONFIG_DEFAULT_TTL_SET_OPCODE:
    {
        ACCESS_CONFIG_DEFAULT_TTL_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_DEFAULT_TTL_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_DEFAULT_TTL_SET_PARAM*)param;
        buffer[marker] = set_param->ttl;
        marker += 1;
    }
    break;

    /* Set GATT Proxy state */
    case MS_ACCESS_CONFIG_GATT_PROXY_SET_OPCODE:
    {
        ACCESS_CONFIG_GATT_PROXY_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_GATT_PROXY_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_GATT_PROXY_SET_PARAM*)param;
        buffer[marker] = set_param->proxy;
        marker += 1;
    }
    break;

    /* Set relay state */
    case MS_ACCESS_CONFIG_RELAY_SET_OPCODE:
    {
        ACCESS_CONFIG_RELAY_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_RELAY_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_RELAY_SET_PARAM*)param;
        buffer[marker] = set_param->relay;
        marker += 1;
        buffer[marker] = (UCHAR)(set_param->relay_rtx_count);
        buffer[marker] |= (UCHAR)(set_param->relay_rtx_interval_steps << 3);
        marker += 1;
    }
    break;

    /* Get model publication state */
    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_GET_OPCODE:
    {
        ACCESS_CONFIG_MODELPUB_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_PUBLICATION_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_MODELPUB_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->element_address);
        marker += 2;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == get_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)get_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if(MS_ACCESS_MODEL_TYPE_VENDOR == get_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], get_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Set model publication state */
    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_SET_OPCODE:
    {
        ACCESS_CONFIG_MODELPUB_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_PUBLICATION_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_MODELPUB_SET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->publish_address);
        marker += 2;
        buffer[marker] = (UCHAR)(set_param->appkey_index);
        marker += 1;
        buffer[marker] = 0;
        buffer[marker] |= (UCHAR)(set_param->appkey_index >> 8);
        buffer[marker] |= (UCHAR)(set_param->credential_flag << 4);
        marker += 1;
        buffer[marker] = set_param->publish_ttl;
        marker += 1;
        buffer[marker] = set_param->publish_period;
        marker += 1;
        buffer[marker] = (UCHAR)(set_param->publish_rtx_count);
        buffer[marker] |= (UCHAR)(set_param->publish_rtx_interval_steps << 3);
        marker += 1;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == set_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)set_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if (MS_ACCESS_MODEL_TYPE_VENDOR == set_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], set_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Set model publication state */
    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET_OPCODE:
    {
        ACCESS_CONFIG_MODELPUB_VADDR_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_MODELPUB_VADDR_SET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->element_address);
        marker += 2;
        EM_mem_copy(&buffer[marker], set_param->publish_address, 16);
        marker += 16;
        buffer[marker] = (UCHAR)(set_param->appkey_index);
        marker += 1;
        buffer[marker] = 0;
        buffer[marker] |= (UCHAR)(set_param->appkey_index >> 8);
        buffer[marker] |= (UCHAR)(set_param->credential_flag << 4);
        marker += 1;
        buffer[marker] = set_param->publish_ttl;
        marker += 1;
        buffer[marker] = set_param->publish_period;
        marker += 1;
        buffer[marker] = (UCHAR)(set_param->publish_rtx_count);
        buffer[marker] |= (UCHAR)(set_param->publish_rtx_interval_steps << 3);
        marker += 1;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == set_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)set_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if (MS_ACCESS_MODEL_TYPE_VENDOR == set_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], set_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Add subscription address */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_ADD_OPCODE:
    {
        ACCESS_CONFIG_MODELSUB_ADD_PARAM* add_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_ADD_OPCODE\n");
        add_param = (ACCESS_CONFIG_MODELSUB_ADD_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], add_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], add_param->address);
        marker += 2;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == add_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)add_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if(MS_ACCESS_MODEL_TYPE_VENDOR == add_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], add_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Add subscription virtual address */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD_OPCODE:
    {
        ACCESS_CONFIG_MODELSUB_VADDR_ADD_PARAM* add_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD_OPCODE\n");
        add_param = (ACCESS_CONFIG_MODELSUB_VADDR_ADD_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], add_param->element_address);
        marker += 2;
        EM_mem_copy(&buffer[marker], add_param->label, 16);
        marker += 16;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == add_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)add_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if(MS_ACCESS_MODEL_TYPE_VENDOR == add_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], add_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Delete subscription address */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE:
    {
        ACCESS_CONFIG_MODELSUB_DEL_PARAM* del_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE\n");
        del_param = (ACCESS_CONFIG_MODELSUB_DEL_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], del_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], del_param->address);
        marker += 2;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == del_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)del_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if(MS_ACCESS_MODEL_TYPE_VENDOR == del_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], del_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Delete subscription virtual address */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE_OPCODE:
    {
        ACCESS_CONFIG_MODELSUB_VADDR_DEL_PARAM* del_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE_OPCODE\n");
        del_param = (ACCESS_CONFIG_MODELSUB_VADDR_DEL_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], del_param->element_address);
        marker += 2;
        EM_mem_copy(&buffer[marker], del_param->label, 16);
        marker += 16;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == del_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)del_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if(MS_ACCESS_MODEL_TYPE_VENDOR == del_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], del_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Add subscription address to cleared list */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE_OPCODE:
    {
        ACCESS_CONFIG_MODELSUB_OVERWRITE_PARAM* owr_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE_OPCODE\n");
        owr_param = (ACCESS_CONFIG_MODELSUB_OVERWRITE_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], owr_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], owr_param->address);
        marker += 2;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == owr_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)owr_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if(MS_ACCESS_MODEL_TYPE_VENDOR == owr_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], owr_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Add subscription virtual address to cleared list */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE_OPCODE:
    {
        ACCESS_CONFIG_MODELSUB_VADDR_OVERWRITE_PARAM* owr_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE_OPCODE\n");
        owr_param = (ACCESS_CONFIG_MODELSUB_VADDR_OVERWRITE_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], owr_param->element_address);
        marker += 2;
        EM_mem_copy(&buffer[marker], owr_param->label, 16);
        marker += 16;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == owr_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)owr_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if (MS_ACCESS_MODEL_TYPE_VENDOR == owr_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], owr_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Discard subscription list */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_ALL_OPCODE:
    {
        ACCESS_CONFIG_MODELSUB_DELETEALL_PARAM* del_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_ALL_OPCODE\n");
        del_param = (ACCESS_CONFIG_MODELSUB_DELETEALL_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], del_param->element_address);
        marker += 2;
        {
            if(MS_ACCESS_MODEL_TYPE_SIG == del_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)del_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if (MS_ACCESS_MODEL_TYPE_VENDOR == del_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], del_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Get subscription list */
    case MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_GET_OPCODE:
    {
        ACCESS_CONFIG_SIGMODELSUB_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_SIGMODELSUB_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->model_id);
        marker += 2;
    }
    break;

    /* Get subscription list */
    case MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_GET_OPCODE:
    {
        ACCESS_CONFIG_VENDORMODELSUB_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_VENDORMODELSUB_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->element_address);
        marker += 2;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], get_param->model_id);
        marker += 4;
    }
    break;

    /* Add to Netkey list */
    case MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE:
    {
        ACCESS_CONFIG_NETKEY_ADD_PARAM* add_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE\n");
        add_param = (ACCESS_CONFIG_NETKEY_ADD_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], add_param->netkey_index);
        marker += 2;
        EM_mem_copy(&buffer[marker], add_param->netkey, 16);
        marker += 16;
    }
    break;

    /* Update to Netkey list */
    case MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE:
    {
        ACCESS_CONFIG_NETKEY_UPDATE_PARAM* update_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE\n");
        update_param = (ACCESS_CONFIG_NETKEY_UPDATE_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], update_param->netkey_index);
        marker += 2;
        EM_mem_copy(&buffer[marker], update_param->netkey, 16);
        marker += 16;
    }
    break;

    /* Delete from Netkey list */
    case MS_ACCESS_CONFIG_NETKEY_DELETE_OPCODE:
    {
        ACCESS_CONFIG_NETKEY_DELETE_PARAM* del_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETKEY_DELETE_OPCODE\n");
        del_param = (ACCESS_CONFIG_NETKEY_DELETE_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], del_param->netkey_index);
        marker += 2;
    }
    break;

    /* Add to Appkey list */
    case MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE:
    {
        ACCESS_CONFIG_APPKEY_ADD_PARAM* add_param;
        /* TODO: Have a better approach */
        UINT16                           key_input_list[2];
        UINT16                           key_output_list_length;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE\n");
        add_param = (ACCESS_CONFIG_APPKEY_ADD_PARAM*)param;
        key_input_list[0] = add_param->netkey_index;
        key_input_list[1] = add_param->appkey_index;
        MS_access_cm_add_appkey
        (
            0, /* TODO: subnet_handle */
            add_param->appkey_index,
            &add_param->appkey[0]
        );
        key_output_list_length = 3;
        config_pack_key_indices
        (
            2,
            key_input_list,
            &buffer[marker],
            &key_output_list_length
        );
        marker += key_output_list_length;
        EM_mem_copy(&buffer[marker], add_param->appkey, 16);
        marker += 16;
    }
    break;

    /* Update to Appkey list */
    case MS_ACCESS_CONFIG_APPKEY_UPDATE_OPCODE:
    {
        ACCESS_CONFIG_APPKEY_UPDATE_PARAM* update_param;
        /* TODO: Have a better approach */
        UINT16                           key_input_list[2];
        UINT16                           key_output_list_length;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_APPKEY_UPDATE_OPCODE\n");
        update_param = (ACCESS_CONFIG_APPKEY_UPDATE_PARAM*)param;
        key_input_list[0] = update_param->netkey_index;
        key_input_list[1] = update_param->appkey_index;
        MS_access_cm_update_appkey
        (
            0, /* TODO: subnet */
            update_param->appkey_index, /* appkey_index */
            &update_param->appkey[0] /* app_key */
        );
        key_output_list_length = 3;
        config_pack_key_indices
        (
            2,
            key_input_list,
            &buffer[marker],
            &key_output_list_length
        );
        marker += key_output_list_length;
        EM_mem_copy(&buffer[marker], update_param->appkey, 16);
        marker += 16;
    }
    break;

    /* Delete from Appkey list */
    case MS_ACCESS_CONFIG_APPKEY_DELETE_OPCODE:
    {
        ACCESS_CONFIG_APPKEY_DELETE_PARAM* del_param;
        /* TODO: Have a better approach */
        UINT16                           key_input_list[2];
        UINT16                           key_output_list_length;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_APPKEY_DELETE_OPCODE\n");
        del_param = (ACCESS_CONFIG_APPKEY_DELETE_PARAM*)param;
        key_input_list[0] = del_param->netkey_index;
        key_input_list[1] = del_param->appkey_index;
        MS_access_cm_delete_appkey
        (
            0, /* TODO: subnet_handle */
            del_param->appkey_index,
            NULL
        );
        key_output_list_length = 3;
        config_pack_key_indices
        (
            2,
            key_input_list,
            &buffer[marker],
            &key_output_list_length
        );
        marker += key_output_list_length;
    }
    break;

    /* Get the Appkey list */
    case MS_ACCESS_CONFIG_APPKEY_GET_OPCODE:
    {
        ACCESS_CONFIG_APPKEY_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_APPKEY_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_APPKEY_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->netkey_index);
        marker += 2;
    }
    break;

    /* Get the Node Identity state */
    case MS_ACCESS_CONFIG_NODE_IDENTITY_GET_OPCODE:
    {
        ACCESS_CONFIG_NODEID_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NODE_IDENTITY_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_NODEID_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->netkey_index);
        marker += 2;
    }
    break;

    /* Set the Node Identity state */
    case MS_ACCESS_CONFIG_NODE_IDENTITY_SET_OPCODE:
    {
        ACCESS_CONFIG_NODEID_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NODE_IDENTITY_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_NODEID_SET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->netkey_index);
        marker += 2;
        buffer[marker] = set_param->identity;
        marker += 1;
    }
    break;

    /* Bind Appkey to model */
    case MS_ACCESS_CONFIG_MODEL_APP_BIND_OPCODE:
    {
        ACCESS_CONFIG_MODEL_APP_BIND_PARAM* bind_param;
        MS_ACCESS_MODEL_HANDLE               model_handle;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_APP_BIND_OPCODE\n");
        bind_param = (ACCESS_CONFIG_MODEL_APP_BIND_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], bind_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], bind_param->appkey_index);
        marker += 2;
        /* Get Local Model Handle */
        retval = MS_access_get_model_handle
                 (
                     0, /* TODO: Using default element handle */
                     bind_param->client_model,
                     &model_handle
                 );

        /* Bind corresponding AppKey locally */
        if (API_SUCCESS == retval)
        {
            retval = MS_access_bind_model_app
                     (
                         model_handle,
                         bind_param->appkey_index
                     );
        }

        {
            if(MS_ACCESS_MODEL_TYPE_SIG == bind_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)bind_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if (MS_ACCESS_MODEL_TYPE_VENDOR == bind_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], bind_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Unbind Appkey to model */
    case MS_ACCESS_CONFIG_MODEL_APP_UNBIND_OPCODE:
    {
        ACCESS_CONFIG_MODEL_APP_UNBIND_PARAM* unbind_param;
        MS_ACCESS_MODEL_HANDLE                 model_handle;
        CONFIG_TRC("MS_ACCESS_CONFIG_MODEL_APP_UNBIND_OPCODE\n");
        unbind_param = (ACCESS_CONFIG_MODEL_APP_UNBIND_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], unbind_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], unbind_param->appkey_index);
        marker += 2;
        /* Get Local Model Handle */
        retval = MS_access_get_model_handle
                 (
                     0, /* TODO: Using default element handle */
                     unbind_param->client_model,
                     &model_handle
                 );

        /* Unbind corresponding AppKey locally */
        if (API_SUCCESS == retval)
        {
            retval = MS_access_unbind_model_app
                     (
                         model_handle,
                         unbind_param->appkey_index
                     );
        }

        {
            if (MS_ACCESS_MODEL_TYPE_SIG == unbind_param->model.type)
            {
                UINT16 val_u16;
                val_u16 = (UINT16)unbind_param->model.id;
                MS_PACK_LE_2_BYTE_VAL(&buffer[marker], val_u16);
                marker += 2;
            }
            else if (MS_ACCESS_MODEL_TYPE_VENDOR == unbind_param->model.type)
            {
                MS_PACK_LE_4_BYTE_VAL(&buffer[marker], unbind_param->model.id);
                marker += 4;
            }
            else
            {
            }
        }
    }
    break;

    /* Get all SIG model Appkeys */
    case MS_ACCESS_CONFIG_SIG_MODEL_APP_GET_OPCODE:
    {
        ACCESS_CONFIG_SIG_MODEL_APP_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_SIG_MODEL_APP_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_SIG_MODEL_APP_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->element_address);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->model_id);
        marker += 2;
    }
    break;

    /* Get all Vendor model Appkeys */
    case MS_ACCESS_CONFIG_VENDOR_MODEL_APP_GET_OPCODE:
    {
        ACCESS_CONFIG_VENDOR_MODEL_APP_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_VENDOR_MODEL_APP_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_VENDOR_MODEL_APP_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->element_address);
        marker += 2;
        MS_PACK_LE_4_BYTE_VAL(&buffer[marker], get_param->model_id);
        marker += 4;
    }
    break;

    /* Set friend state */
    case MS_ACCESS_CONFIG_FRIEND_SET_OPCODE:
    {
        ACCESS_CONFIG_FRIEND_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_FRIEND_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_FRIEND_SET_PARAM*)param;
        buffer[marker] = set_param->friend;
        marker += 1;
    }
    break;

    /* Get key refresh phase state */
    case MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_GET_OPCODE:
    {
        ACCESS_CONFIG_KEYREFRESH_PHASE_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_KEYREFRESH_PHASE_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->netkey_index);
        marker += 2;
    }
    break;

    /* Set key refresh phase state */
    case MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_SET_OPCODE:
    {
        ACCESS_CONFIG_KEYREFRESH_PHASE_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_KEYREFRESH_PHASE_SET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->netkey_index);
        marker += 2;
        buffer[marker] = set_param->transition;
        marker += 1;
    }
    break;

    /* Set heartbeat publication state */
    case MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_SET_OPCODE:
    {
        ACCESS_CONFIG_HEARTBEATPUB_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_HEARTBEATPUB_SET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->destination);
        marker += 2;
        buffer[marker] = set_param->countlog;
        marker += 1;
        buffer[marker] = set_param->periodlog;
        marker += 1;
        buffer[marker] = set_param->ttl;
        marker += 1;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->features);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->netkey_index);
        marker += 2;
    }
    break;

    /* Set heartbeat subscription state */
    case MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_SET_OPCODE:
    {
        ACCESS_CONFIG_HEARTBEATSUB_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_HEARTBEATSUB_SET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->source);
        marker += 2;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], set_param->destination);
        marker += 2;
        buffer[marker] = set_param->periodlog;
        marker += 1;
    }
    break;

    /* Get LPN Polltimeout state */
    case MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_GET_OPCODE:
    {
        ACCESS_CONFIG_LPNPOLLTIMEOUT_GET_PARAM* get_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_GET_OPCODE\n");
        get_param = (ACCESS_CONFIG_LPNPOLLTIMEOUT_GET_PARAM*)param;
        MS_PACK_LE_2_BYTE_VAL(&buffer[marker], get_param->lpn_address);
        marker += 2;
    }
    break;

    /* Set Network transmit state */
    case MS_ACCESS_CONFIG_NETWORK_TRANSMIT_SET_OPCODE:
    {
        ACCESS_CONFIG_NETWORK_TRANSMIT_SET_PARAM* set_param;
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETWORK_TRANSMIT_SET_OPCODE\n");
        set_param = (ACCESS_CONFIG_NETWORK_TRANSMIT_SET_PARAM*)param;
        buffer[marker] = (UCHAR)(set_param->net_tx_count);
        buffer[marker] |= (UCHAR)(set_param->net_tx_interval_steps << 3);
        marker += 1;
    }
    break;

    default:
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

    retval = MS_access_reliable_publish
             (
                 &config_client_model_handle,
                 req_opcode,
                 pdu_ptr,
                 marker,
                 rsp_opcode
             );
    return retval;
}

/* Utility Routines */
/* Pack Key Indices as described in Section [4.3.1.1] */
static API_RESULT config_pack_key_indices
(
    /* IN */    UINT16     key_count,
    /* IN */    UINT16*    key_input_list,
    /* OUT */   UCHAR*     key_output_list,
    /* INOUT */ UINT16*    key_output_list_length
)
{
    UINT32 index;
    UINT16 marker;
    /* Pack two Key Indices in 3 octets */
    marker = 0;

    for (index = 0; index < (UINT32)(key_count/2); index += 2)
    {
        /* 8 LSBs of the first Key Index in the first octet */
        key_output_list[marker] = (UCHAR)key_input_list[index];
        marker++;
        /* First 4 LSBs of the second Key Index into 4 MSBs of the second octet */
        key_output_list[marker] = (UCHAR)key_input_list[index + 1];
        key_output_list[marker] = (key_output_list[marker] << 4);
        /* 4 remaining MSBs of the first Key Index into 4 LSBs of the second octet */
        key_output_list[marker] |= (UCHAR)(key_input_list[index] >> 8);
        marker++;
        /* 8 remaining MSBs of the second Key Index into the third octet */
        key_output_list[marker] = (UCHAR)(key_input_list[index + 1] >> 4);
        marker++;
    }

    /* If Key Count is odd, pack the last one */
    if (0x00 != (key_count & 0x01))
    {
        /* 8 LSBs of the first Key Index in the first octet */
        key_output_list[marker] = (UCHAR)key_input_list[index];
        marker++;
        /* 4 remaining MSBs of the first Key Index into 4 LSBs of the second octet */
        key_output_list[marker] = (UCHAR)(key_input_list[index] >> 8);
        marker++;
    }

    /* Set outout list length */
    *key_output_list_length = marker;
    return API_SUCCESS;
}

#if 0
/* Do not delete this function */
/* Unpack Key Indices as described in Section [4.3.1.1] */
static API_RESULT config_unpack_key_indices
(
    /* IN */    UINT32     key_1_and_2,
    /* OUT */   UINT16*    key_1,
    /* OUT */   UINT16*    key_2
)
{
    CONFIG_TRC(
        "[CONFIG] Unpack Key 1 and Key 2 from 0x%08X\n", key_1_and_2);
    /* 8 LSBs of the first Key Index in the first octet */
    /* First 4 LSBs of the second Key Index into 4 MSBs of the second octet */
    /* 4 remaining MSBs of the first Key Index into 4 LSBs of the second octet */
    /* 8 remaining MSBs of the second Key Index into the third octet */
    (*key_1) = (UCHAR)((key_1_and_2 >> 8) & 0x0F);
    (*key_1) = (*key_1) << 8;
    (*key_1) |= (UCHAR)(key_1_and_2);
    CONFIG_TRC(
        "[CONFIG] Extracted Key 1: 0x%04X\n", (*key_1));
    (*key_2) = (UCHAR)((key_1_and_2 >> 20) & 0x0F);
    (*key_2) = (*key_2) << 8;
    (*key_2) |= (UCHAR)(key_1_and_2 >> 12);
    CONFIG_TRC(
        "[CONFIG] Extracted Key 2: 0x%04X\n", (*key_2));
    return API_SUCCESS;
}
#endif /* 0 */

CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_composition_data_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_heartbeat_publication_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_appkey_list_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_appkey_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_beacon_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_default_ttl_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_friend_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_gatt_proxy_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_key_refresh_phase_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_model_publication_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_model_subscription_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_network_transmit_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_relay_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_sig_model_subscription_list_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_vendor_model_subscription_list_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_low_power_node_polltimeout_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_heartbeat_subscription_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_model_app_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_netkey_list_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_netkey_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_node_identity_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_node_reset_status_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_sig_model_app_list_handler)
CONFIG_OPCODE_HANDLER_EMPTY_DEF(config_vendor_model_app_list_handler)


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
API_RESULT config_client_cb
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
    retval = API_SUCCESS;
    CONFIG_TRC(
        "[CONFIGURATION_CLIENT] Callback. Opcode 0x%04X\n", opcode);
    CONFIG_debug_dump_bytes(data_param, data_len);

    /* Check if the packet is decrypted using device key */
    if (MS_CONFIG_LIMITS(MS_MAX_APPS) > appkey_handle)
    {
        CONFIG_ERR("[CONFIG] Configuration Model Packets shall use Device Key. Dropping..\n");
        return API_FAILURE;
    }

    switch(opcode)
    {
    case MS_ACCESS_CONFIG_GATT_PROXY_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_GATT_PROXY_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_gatt_proxy_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_heartbeat_publication_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_SIG_MODEL_APP_LIST_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_SIG_MODEL_APP_LIST_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_sig_model_app_list_handler);
    }
    break;

    case MS_ACCESS_CONFIG_COMPOSITION_DATA_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_COMPOSITION_DATA_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_composition_data_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_model_subscription_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_NETKEY_LIST_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETKEY_LIST_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_netkey_list_handler);
    }
    break;

    case MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_low_power_node_polltimeout_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_LIST_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_LIST_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_sig_model_subscription_list_handler);
    }
    break;

    case MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_LIST_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_LIST_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_vendor_model_subscription_list_handler);
    }
    break;

    case MS_ACCESS_CONFIG_APPKEY_LIST_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_APPKEY_LIST_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_appkey_list_handler);
    }
    break;

    case MS_ACCESS_CONFIG_APPKEY_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_APPKEY_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_appkey_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_RELAY_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_RELAY_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_relay_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_FRIEND_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_FRIEND_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_friend_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_MODEL_APP_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_APP_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_model_app_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_NODE_RESET_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NODE_RESET_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_node_reset_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_DEFAULT_TTL_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_DEFAULT_TTL_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_default_ttl_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_NODE_IDENTITY_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NODE_IDENTITY_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_node_identity_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_heartbeat_subscription_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_key_refresh_phase_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_NETWORK_TRANSMIT_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETWORK_TRANSMIT_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_network_transmit_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_BEACON_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_BEACON_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_beacon_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_MODEL_PUBLICATION_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_model_publication_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_NETKEY_STATUS_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_NETKEY_STATUS_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_netkey_status_handler);
    }
    break;

    case MS_ACCESS_CONFIG_VENDOR_MODEL_APP_LIST_OPCODE:
    {
        CONFIG_TRC(
            "MS_ACCESS_CONFIG_VENDOR_MODEL_APP_LIST_OPCODE\n");
        CONFIG_CLIENT_OPCODE_HANDLER(config_vendor_model_app_list_handler);
    }
    break;

    default:
        break;
    }

    /* Application callback */
    if (NULL != config_client_appl_cb)
    {
        config_client_appl_cb(handle, opcode, data_param, data_len);
    }

    return retval;
}

#endif /* MS_MODEL_CONFIG */

