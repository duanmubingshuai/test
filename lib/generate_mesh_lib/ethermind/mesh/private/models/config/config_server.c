/**
    \file config_server.c

    This file inplements Configuration Model Server
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "config_internal.h"
#include "MS_trn_api.h"
//uint8   cfg_retry_flag = 0;   // HZF

//static inline uint32 clock_time_rtc(void){
//  return (*(volatile unsigned int *)0x4000f028) & 0xffffff;
//}

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

/* Callback Handler Declaration */
#define CONFIG_SERVER_OPCODE_HANDLER_DECL(handler) \
    DECL_STATIC API_RESULT (handler) \
    ( \
      /* IN */    UINT32                   opcode, \
      /* IN */    UCHAR                  * data_param, \
      /* IN */    UINT16                   data_len, \
      /* OUT */   UINT32                 * response_opcode, \
      /* OUT */   UINT8                  * response_buffer, \
      /* INOUT */ UINT16                 * response_buffer_len \
    );

#ifdef MS_MODEL_CONFIG

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Static Global Variables */
/* TBD: Remove the Opcodes not appropriate for Configure Model Server role */
DECL_STATIC DECL_CONST UINT32 config_server_opcode_list[] =
{
    /** 8-bit Opcodes of Model specific messages */
    MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE,
    MS_ACCESS_CONFIG_APPKEY_UPDATE_OPCODE,
    MS_ACCESS_CONFIG_MODEL_PUBLICATION_SET_OPCODE,

    /** 16-bit Opcodes of Model specific messages */
    MS_ACCESS_CONFIG_APPKEY_DELETE_OPCODE,
    MS_ACCESS_CONFIG_APPKEY_GET_OPCODE,
    MS_ACCESS_CONFIG_COMPOSITION_DATA_GET_OPCODE,
    MS_ACCESS_CONFIG_BEACON_GET_OPCODE,
    MS_ACCESS_CONFIG_BEACON_SET_OPCODE,
    MS_ACCESS_CONFIG_DEFAULT_TTL_GET_OPCODE,
    MS_ACCESS_CONFIG_DEFAULT_TTL_SET_OPCODE,
    MS_ACCESS_CONFIG_FRIEND_GET_OPCODE,
    MS_ACCESS_CONFIG_FRIEND_SET_OPCODE,
    MS_ACCESS_CONFIG_GATT_PROXY_GET_OPCODE,
    MS_ACCESS_CONFIG_GATT_PROXY_SET_OPCODE,
    MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_GET_OPCODE,
    MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_SET_OPCODE,
    MS_ACCESS_CONFIG_MODEL_PUBLICATION_GET_OPCODE,
    MS_ACCESS_CONFIG_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_ADD_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_ALL_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE_OPCODE,
    MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE_OPCODE,
    MS_ACCESS_CONFIG_NETWORK_TRANSMIT_GET_OPCODE,
    MS_ACCESS_CONFIG_NETWORK_TRANSMIT_SET_OPCODE,
    MS_ACCESS_CONFIG_RELAY_GET_OPCODE,
    MS_ACCESS_CONFIG_RELAY_SET_OPCODE,
    MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_GET_OPCODE,
    MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_GET_OPCODE,
    MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_GET_OPCODE,
    MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_GET_OPCODE,
    MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_SET_OPCODE,
    MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_GET_OPCODE,
    MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_SET_OPCODE,
    MS_ACCESS_CONFIG_MODEL_APP_BIND_OPCODE,
    MS_ACCESS_CONFIG_MODEL_APP_UNBIND_OPCODE,
    MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE,
    MS_ACCESS_CONFIG_NETKEY_DELETE_OPCODE,
    MS_ACCESS_CONFIG_NETKEY_GET_OPCODE,
    MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE,
    MS_ACCESS_CONFIG_NODE_IDENTITY_GET_OPCODE,
    MS_ACCESS_CONFIG_NODE_IDENTITY_SET_OPCODE,
    MS_ACCESS_CONFIG_NODE_RESET_OPCODE,
    MS_ACCESS_CONFIG_SIG_MODEL_APP_GET_OPCODE,
    MS_ACCESS_CONFIG_VENDOR_MODEL_APP_GET_OPCODE
};

CONFIG_SERVER_OPCODE_HANDLER_DECL(config_appkey_add_update_delete_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_publication_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_appkey_get_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_default_ttl_get_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_friend_get_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_gatt_proxy_get_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_key_refresh_phase_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_key_refresh_phase_get_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_publication_get_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_subscription_add_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_subscription_delete_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_subscription_delete_all_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_subscription_overwrite_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_network_transmit_get_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_relay_get_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_subscription_get_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_low_power_node_polltimeout_get_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_heartbeat_publication_get_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_heartbeat_subscription_get_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_app_bind_unbind_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_netkey_add_update_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_netkey_delete_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_netkey_get_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_node_identity_set_handler);
CONFIG_SERVER_OPCODE_HANDLER_DECL(config_node_identity_get_handler);
DECL_STATIC API_RESULT config_node_reset_handler
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*    handle,
    /* IN */ MS_NET_ADDR                saddr,
    /* IN */ MS_NET_ADDR                daddr,
    /* IN */ MS_SUBNET_HANDLE           subnet_handle,
    /* IN */ MS_APPKEY_HANDLE           appkey_handle,
    /* OUT */   UINT32*                 response_opcode,
    /* OUT */   UINT8*                  response_buffer,
    /* INOUT */ UINT16*                 response_buffer_len
);

CONFIG_SERVER_OPCODE_HANDLER_DECL(config_model_app_get_handler);

DECL_STATIC API_RESULT config_composition_data_get_handler
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
);

DECL_STATIC API_RESULT config_beacon_get_set_handler
(
    /* IN */    MS_SUBNET_HANDLE         subnet_handle,
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
);

DECL_STATIC API_RESULT config_pack_key_indices
(
    /* IN */    UINT16     key_count,
    /* IN */    UINT16*    key_input_list,
    /* OUT */   UCHAR*     key_output_list,
    /* INOUT */ UINT16*    key_output_list_length
);

DECL_STATIC API_RESULT config_unpack_key_indices
(
    /* IN */    UINT32     key_1_and_2,
    /* OUT */   UINT16*    key_1,
    /* OUT */   UINT16*    key_2
);

static void snb_timeout_handler (void* args, UINT16 size);

static void config_stop_snb_timer
(
    /* IN */ MS_SUBNET_HANDLE         subnet_handle
);


static APP_config_server_CB_t app_config_server_callback=NULL;
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */
/* Secure Network Timer (minimum of 10 s) */
static EM_timer_handle snb_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

/* Secure Network Beacon Timeout value - default 10s */
#define CONFIG_SERVER_SNB_TIMEOUT     (10 * 1000) /* in ms */

/* Start Secure Network Beacon Timer */
static void config_start_snb_timer
(
    /* IN */    MS_SUBNET_HANDLE         subnet_handle
);

/* --------------------------------------------- Function */

/**
    \brief API to initialize configuration server model

    \par Description
    This is to initialize configuration server model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_config_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE    element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*     model_handle
)
{
    API_RESULT retval;
    MS_ACCESS_NODE_ID        node_id;
    MS_ACCESS_MODEL          model;
    /* TBD: Initialize MUTEX and other data structures */
    snb_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    CONFIG_TRC(
        "[CONFIG] Registered Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_CONFIG_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = config_server_cb;
    model.pub_cb = NULL;
    /* List of Opcodes */
    model.opcodes = config_server_opcode_list;
    model.num_opcodes = sizeof(config_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    return retval;
}

void APP_config_server_CB_init(APP_config_server_CB_t appConfigServerCB)
{
    app_config_server_callback = appConfigServerCB;
}

/* Utility Routines */
/* Pack Key Indices as described in Section [4.3.1.1] */
DECL_STATIC API_RESULT config_pack_key_indices
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

/* Unpack Key Indices as described in Section [4.3.1.1] */
DECL_STATIC API_RESULT config_unpack_key_indices
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

DECL_STATIC API_RESULT config_appkey_add_update_delete_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT32     net_app_key_index;
    UINT16     netkey_index;
    UINT16     appkey_index;
    UCHAR*     app_key;
    MS_SUBNET_HANDLE   subnet;
    UCHAR*           appkey_status;
    UINT8            marker;
    /**
        Extract
        - NetKeyIndexAndAppKeyIndex (3 Octets)
        - AppKey (16 Octets)

        For AppKey Delete - no AppKey
    */
    appkey_status = response_buffer;

    if ((19 == data_len) || ((MS_ACCESS_CONFIG_APPKEY_DELETE_OPCODE == opcode) && (3 == data_len)))
    {
        MS_UNPACK_LE_3_BYTE(&net_app_key_index, data_param);
        /* AppKey required only if not AppKey Delete Operation */
        app_key = (data_param + 3);
        CONFIG_TRC(
            "[CONFIG] Add/Update/delete AppKey (0x%08X). NetKeyIndexAndAppKeyIndex 0x%04X\n",
            opcode, net_app_key_index);
        /* Extract NetKeyIndex and AppKeyIndex */
        config_unpack_key_indices
        (
            net_app_key_index,
            &netkey_index,
            &appkey_index
        );
        CONFIG_TRC(
            "[CONFIG] Extracted NetKeyIndex 0x%04X, AppKeyIndex 0x%04X\n", netkey_index, appkey_index);
        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     netkey_index,
                     &subnet
                 );

        /* Check if Subnet is found */
        if (API_SUCCESS == retval)
        {
            if (MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE == opcode)
            {
                MS_APPKEY_HANDLE  appkey_handle;
                /* First check if the key is already stored */
                retval = MS_access_cm_get_appkey_handle
                         (
                             subnet,
                             appkey_index,
                             app_key,
                             &appkey_handle
                         );

                /* Check if already preset, return success */
                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] AppKey Index already Present and both Key and Index matched. Returning 0x%04X\n",
                        retval);
                }
                /* If Key and Index Mismatch, return status 'MS_KEY_INDEX_ALREADY_STORED' */
                else if (retval != MS_KEY_INDEX_ALREADY_STORED)
                {
                    retval = MS_access_cm_add_appkey
                             (
                                 subnet,
                                 appkey_index,
                                 app_key
                             );
                    CONFIG_TRC(
                        "[CONFIG] Add AppKey returned 0x%04X\n", retval);
                    printf(
                        "[CONFIG] Add AppKey returned 0x%04X\n", retval);
                }
            }
            else
            {
                retval = MS_access_cm_update_delete_appkey
                         (
                             subnet,
                             appkey_index,
                             opcode,
                             app_key
                         );
                CONFIG_TRC(
                    "[CONFIG] Update/delete AppKey returned 0x%04X\n", retval);
                printf(
                    "[CONFIG] Update/delete AppKey returned 0x%04X\n", retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Find Subnet failed. Retval 0x%04X\n", retval);
            retval = MS_INVALID_NETKEY_INDEX;
        }

        /**
            The Status Code shall be Success if the received request was redundant
            (add of an identical existing key, update of an identical updated key,
            or delete of a non-existent key).
        */
        if ((MS_ACCESS_CONFIG_APPKEY_DELETE_OPCODE == opcode) && ((MS_INVALID_NETKEY_INDEX == retval) || (MS_INVALID_APPKEY_INDEX == retval)))
        {
            retval = MS_SUCCESS;
        }

        /* Respond with Config AppKey Status message */
        *response_opcode = MS_ACCESS_CONFIG_APPKEY_STATUS_OPCODE;
        marker = 0;
        appkey_status[marker] = (UCHAR)retval;
        marker ++;
        /* Copy back the first 3 bytes from the request which contains NetKeyIndexAndAppKeyIndex */
        EM_mem_copy(&appkey_status[marker], data_param, 3);
        marker += 3;
        * response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_model_publication_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    MS_ACCESS_PUBLISH_INFO    publish_info;
    UCHAR*                    status;
    UINT8                     marker;
    /**
        Extract
        - ElementAddress (2 Octets)
        - PublishAddress (2 Octets) or Label (16 Octets)
        - AppKeyIndex (12 bits), CredentialFlag (1 bit), RFU (3 bits)
        - PublishTTL (1 Octet)
        - PublishPeriod (1 Octet)
        - PublishRetransmitCount (3 bits), PublishRetransmitIntervalSteps (5 bits)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if (((11 == data_len) || (13 == data_len)) || ((25 == data_len) || (27 == data_len)))
    {
        marker = 0;
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param + marker);
        marker += 2;

        /* PublishAddress - non-virtual address */
        if (opcode == MS_ACCESS_CONFIG_MODEL_PUBLICATION_SET_OPCODE)
        {
            publish_info.addr.use_label = 0;
            MS_UNPACK_LE_2_BYTE(&publish_info.addr.addr, data_param + marker);
            marker += 2;
        }
        else
        {
            publish_info.addr.use_label = 1;
            EM_mem_copy(publish_info.addr.label, data_param + marker, MS_LABEL_UUID_LENGTH);
            marker += MS_LABEL_UUID_LENGTH;
        }

        /* AppKeyIndex */
        MS_UNPACK_LE_2_BYTE(&publish_info.appkey_index, data_param + marker);
        marker += 2;
        /* CredentialFlag */
        publish_info.crden_flag = (UCHAR)((publish_info.appkey_index >> 12) & 0x01);
        publish_info.appkey_index &= 0x0FFF;
        /* PublishTTL */
        publish_info.ttl = data_param[marker];
        marker++;
        /* PublishPeriod */
        publish_info.period = data_param[marker];
        marker++;
        /* PublishRetransmitCount, PublishRetransmitIntervalSteps */
        publish_info.rtx_count = (UCHAR)(data_param[marker] & 0x07);
        publish_info.rtx_interval_steps = (UCHAR)(data_param[marker] >> 3);
        marker++;

        /* Get Model ID type, based on the length */
        if (2 == (data_len - marker))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + marker));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + marker));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model Publication Set. ElementAddress 0x%04X\n", uaddr);

        if (0 == publish_info.addr.use_label)
        {
            CONFIG_TRC(
                "[CONFIG] PublishAddress 0x%04X\n", publish_info.addr.addr);
        }

        CONFIG_TRC(
            "[CONFIG] AppKeyIndex 0x%04X\n", publish_info.appkey_index);
        CONFIG_TRC(
            "[CONFIG] CredentialFlag 0x%02X\n", publish_info.crden_flag);
        CONFIG_TRC(
            "[CONFIG] PublishTTL 0x%02X\n", publish_info.ttl);
        CONFIG_TRC(
            "[CONFIG] PublishPeriod 0x%02X\n", publish_info.period);
        CONFIG_TRC(
            "[CONFIG] PublishRetransmitCount 0x%02X\n", publish_info.rtx_count);
        CONFIG_TRC(
            "[CONFIG] PublishRetransmitIntervalSteps 0x%02X\n", publish_info.rtx_interval_steps);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        printf(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                /* Called from Remote Configuration Client */
                publish_info.remote = MS_TRUE;
                retval = MS_access_cm_set_model_publication
                         (
                             model_handle,
                             &publish_info
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Set Model Publication successful.\n");
                    printf(
                        "[CONFIG] Set Model Publication successful.\n");
                    /* TODO: Print Publish Info */
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Set Model Publication Failed. Retval 0x%04X\n", retval);
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Set Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            retval = MS_INVALID_ADDRESS;
            CONFIG_ERR(
                "[CONFIG] Set Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Reset all the fields leaving ElementAddress and Model ID as 0 */
        if (API_SUCCESS != retval)
        {
            EM_mem_set(&publish_info, 0, sizeof(publish_info));
        }

        /* Respond with Config Model Publication Status message */
        *response_opcode = MS_ACCESS_CONFIG_MODEL_PUBLICATION_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Pack ElementAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;
        /* Pack PublishAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], publish_info.addr.addr);
        marker += 2;
        /* Pack AppKeyIndex and CredentialFlag */
        status[marker] = (UCHAR) (publish_info.appkey_index);
        marker ++;
        status[marker] = (UCHAR)((publish_info.appkey_index >> 8) & 0x0F);
        status[marker] |= (UCHAR) (publish_info.crden_flag << 4);
        marker ++;
        /* Pack PublishTTL */
        status[marker] = publish_info.ttl;
        marker ++;
        /* Pack PublishPeriod */
        status[marker] = publish_info.period;
        marker ++;
        /* Pack PublishRetransmitCount and PublishRetransmitIntervalSteps */
        status[marker] = (UCHAR)(publish_info.rtx_count & 0x07);
        status[marker] |= (UCHAR)(publish_info.rtx_interval_steps << 3);
        marker ++;

        if (MS_ACCESS_MODEL_TYPE_SIG == model_id.type)
        {
            /* Pack Model SIG ID - 2 Octets */
            model_id_16 = (UINT16)model_id.id;
            MS_PACK_LE_2_BYTE_VAL(&status[marker], model_id_16);
            marker += 2;
        }
        else
        {
            /* Pack Model Vendor ID - 4 Octets */
            MS_PACK_LE_4_BYTE_VAL(&status[marker], model_id.id);
            marker += 4;
        }

        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_appkey_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16           appkey_count;
    /* TODO: Allocate memory for appkey_index_list */
    UINT16           appkey_index_list[20 /* MS_MAX_APPS */];
    /* TODO: Adjust the response buffer length */
    /* UINT8            appkey_index_list_pdu[1 + 2 + MS_MAX_APPS * 2]; */
    UINT8*           appkey_index_list_pdu;
    UINT16           appkey_index_list_pdu_length;
    UINT16           netkey_index;
    MS_SUBNET_HANDLE subnet;
    UINT16           marker;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - NetKeyIndex (2 Octets)
    */
    appkey_index_list_pdu = response_buffer;

    if (2 == data_len)
    {
        MS_UNPACK_LE_2_BYTE(&netkey_index, data_param);
        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     netkey_index,
                     &subnet
                 );

        /* Check if Subnet is found */
        if (API_SUCCESS == retval)
        {
            /* Get AppKey List */
            appkey_count = sizeof(appkey_index_list) / sizeof(UINT16);
            retval = MS_access_cm_get_appkey_index_list
                     (
                         subnet,
                         &appkey_count,
                         appkey_index_list
                     );
            CONFIG_TRC(
                "[CONFIG] AppKey List returned 0x%04X. Count 0x%04X\n", retval, appkey_count);

            if (ACCESS_NO_MATCH == retval)
            {
                retval = MS_INVALID_APPKEY_INDEX;
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Find Subnet failed. Retval 0x%04X\n", retval);
            /* TODO: Check what should be the right return value */
            retval = MS_INVALID_NETKEY_INDEX;
        }

        /* Pack Status */
        /* TODO: Set the Status correctly */
        marker = 0;
        appkey_index_list_pdu[marker] = (UCHAR)retval;
        marker ++;
        MS_PACK_LE_2_BYTE_VAL(&appkey_index_list_pdu[marker], netkey_index);
        marker += 2;

        if (API_SUCCESS == retval)
        {
            /* Pack the AppKey Indices */
            appkey_index_list_pdu_length = sizeof(appkey_index_list_pdu) - 3;
            /* TODO: Not checking retval */
            config_pack_key_indices
            (
                appkey_count,
                appkey_index_list,
                appkey_index_list_pdu + marker,
                &appkey_index_list_pdu_length
            );
            CONFIG_TRC(
                "[CONFIG] AppKey Indices after packing\n");
            CONFIG_debug_dump_bytes(appkey_index_list_pdu + marker, appkey_index_list_pdu_length);
            marker += appkey_index_list_pdu_length;
        }

        /* Respond with Config AppKey List message */
        *response_opcode = MS_ACCESS_CONFIG_APPKEY_LIST_OPCODE;
        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_composition_data_get_handler
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
    API_RESULT       retval;
    MS_BUFFER        buffer;
    UINT32           response;
    MS_IGNORE_UNUSED_PARAM(data_param);
    MS_IGNORE_UNUSED_PARAM(data_len);
    MS_IGNORE_UNUSED_PARAM(opcode);
    /* Allocate Memory */
    buffer.length  = 256;
    buffer.payload = (UCHAR*)EM_alloc_mem(buffer.length);

    if (NULL == buffer.payload)
    {
        return ACCESS_MEMORY_ALLOCATION_FAILED;
    }

    /* Get composition data from Access Layer */
    retval = MS_access_get_composition_data(&buffer);
    CONFIG_TRC(
        "[CONFIG] Composition Data Get. Retval:0x%04X\n", retval);
    printf(
        "[CONFIG] Composition Data Get. Retval:0x%04X\n", retval);

    /* Check retval */
    if (API_SUCCESS == retval)
    {
        /* Respond with Config Composition Data Statud message */
        response = MS_ACCESS_CONFIG_COMPOSITION_DATA_STATUS_OPCODE;
        retval = MS_access_reply
                 (
                     handle,
                     daddr,
                     saddr,
                     subnet_handle,
                     appkey_handle,
                     ACCESS_INVALID_DEFAULT_TTL,
                     response,
                     buffer.payload,
                     buffer.length
                 );
    }

    EM_free_mem(buffer.payload);
    return retval;
}

/* Secure Network Beacon Timeout Handler */
static void snb_timeout_handler (void* args, UINT16 size)
{
    MS_SUBNET_HANDLE         subnet_handle;
    MS_IGNORE_UNUSED_PARAM(size);
    subnet_handle = (MS_SUBNET_HANDLE)(*((UINT32*)args));
    snb_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    CONFIG_TRC(
        "[CONFIG] *** SNB Timeout\n");
    /* Send SNB */
    MS_net_broadcast_secure_beacon(subnet_handle);
    /* TODO: Check if the timer to be restarted */
    /* Restart Timer */
    config_start_snb_timer(subnet_handle);
    return;
}

/* Start Secure Network Beacon Timer */
static void config_start_snb_timer
(
    /* IN */    MS_SUBNET_HANDLE         subnet_handle
)
{
    API_RESULT retval;

    if (EM_TIMER_HANDLE_INIT_VAL == snb_timer_handle)
    {
        retval = EM_start_timer
                 (
                     &snb_timer_handle,
                     (CONFIG_SERVER_SNB_TIMEOUT | EM_TIMEOUT_MILLISEC),
                     snb_timeout_handler,
                     &subnet_handle,
                     sizeof(subnet_handle)
                 );

        if(retval)
        {
            CONFIG_ERR(
                "[CONFIG] Start Timer Failed\n");
        }
    }
}


/* Stop Timer */
static void config_stop_snb_timer
(
    /* IN */ MS_SUBNET_HANDLE         subnet_handle
)
{
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(subnet_handle);

    if (EM_TIMER_HANDLE_INIT_VAL != snb_timer_handle)
    {
        retval = EM_stop_timer(&snb_timer_handle);

        if(retval)
        {
            CONFIG_ERR(
                "[CONFIG] Stop Timer Failed\n");
        }
    }
}

/**
     \brief Get/Set the Secure Network Beacon State

     \par Description
     The Config Beacon Get/Set is an acknowledged messaged used to get/set
     the current Secure Network Beacon State of a node.
     This handler routine will respond with Config Beacon Status message.
*/
DECL_STATIC API_RESULT config_beacon_get_set_handler
(
    /* IN */    MS_SUBNET_HANDLE         subnet_handle,
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT8      current_state;
    MS_IGNORE_UNUSED_PARAM(data_len);

    /**
        Parameter Validation.
        If Secure Network Beacon State is from RFU range, do not respond.
        This can happen only with Set request.
    */
    if (MS_ACCESS_CONFIG_BEACON_SET_OPCODE == opcode)
    {
        current_state = (UINT8)(*data_param);

        if (MS_NOT_SUPPORTED <= current_state)
        {
            CONFIG_ERR(
                "[CONFIG] Set Beacon State Request with invalid value 0x%02X. Dropping ...\n", current_state);
            /* Unlock */
            return API_SUCCESS;
        }

        /* Set current Secure Network Beacon state */
        /* Current State is already extracted at the start of this function */
        CONFIG_TRC(
            "[CONFIG] Set Beacon State Request. Value 0x%02X\n", current_state);
        /**
            TODO: If Beacon state changes to value MS_DISABLE/MS_ENABLE, what needs to be done?
        */
        retval = MS_access_cm_set_features_field(current_state, MS_FEATURE_SEC_NET_BEACON);

        /**
            Check if enabled for Secure Network Beacon
            - start Secure Network Beacon timer if not already running
        */
        if (MS_ENABLE == current_state)
        {
            /* TODO: Move the timer to Access layer and maintain for each subnet */
            config_start_snb_timer(subnet_handle);
        }
        /**
            Check if disabled for Secure Network Beacon
            - stop Secure Network Beacon timer if already running
        */
        else
        {
            config_stop_snb_timer(subnet_handle);
        }
    }
    else /* MS_ACCESS_CONFIG_BEACON_GET_OPCODE */
    {
        CONFIG_TRC(
            "[CONFIG] Get Beacon State Request\n");
        /* Get current Secure Network Beacon state */
        retval = MS_access_cm_get_features_field(&current_state, MS_FEATURE_SEC_NET_BEACON);
        CONFIG_TRC(
            "[CONFIG] Current Secure Network Beacon State 0x%02X\n", current_state);
    }

    /* Respond with Config Beacon Status message */
    *response_opcode = MS_ACCESS_CONFIG_BEACON_STATUS_OPCODE;
    response_buffer[0] = current_state;
    *response_buffer_len = sizeof(current_state);
    return retval;
}

DECL_STATIC API_RESULT config_default_ttl_get_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT8      default_ttl;
    MS_IGNORE_UNUSED_PARAM(data_len);
    #if 0

    /* Check if the request is for Primary Subnet */
    if (MS_PRIMARY_SUBNET != subnet_handle)
    {
        CONFIG_ERR(
            "[CONFIG] Get/Set (0x%08X) Default TTL Request for non-primary Subnet Handle 0x%04X. Dropping...\n",
            opcode, subnet_handle);
        return ACCESS_INVALID_HANDLE;
    }

    #endif /* 0 */

    if (MS_ACCESS_CONFIG_DEFAULT_TTL_GET_OPCODE == opcode)
    {
        #if 0
        CONFIG_TRC(
            "[CONFIG] Get Default TTL Request for Subnet Handle 0x%04X\n", subnet_handle);
        #endif /* 0 */
        /* Get Default TTL from access layer */
        retval = MS_access_cm_get_default_ttl(&default_ttl);
        CONFIG_TRC(
            "[CONFIG] Current Default TTL value 0x%02X\n", default_ttl);
    }
    else
    {
        /* Extract TTL from rxed frame */
        default_ttl = *data_param;
        CONFIG_TRC(
            "[CONFIG] Set Default TTL Request. Value 0x%02X\n", default_ttl);
        /* Set Default TTL with access layer */
        retval = MS_access_cm_set_default_ttl(default_ttl);

        /* Return back same TTL value */
        if (API_SUCCESS != retval)
        {
            CONFIG_ERR(
                "[CONFIG] Invalid Set Default TTL Request 0x%02X. Dropping...\n",
                default_ttl);
            return retval;
        }
    }

    /* Respond with Config Default TTL Status message */
    *response_opcode = MS_ACCESS_CONFIG_DEFAULT_TTL_STATUS_OPCODE;
    response_buffer[0] = default_ttl;
    *response_buffer_len = sizeof(default_ttl);
    return retval;
}

/**
     \brief Get/Set the current Friend State

     \par Description
     The Config Friend Get/Set is an acknowledged messaged used to get/set the current
     Friend state of a node.
     This handler routine will respond with Config Friend Status message.
*/
DECL_STATIC API_RESULT config_friend_get_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT8      current_state;
    MS_IGNORE_UNUSED_PARAM(data_len);
    #ifndef MS_FRIEND_SUPPORT
    /* Return Status as the Friend feature is not supported */
    CONFIG_TRC(
        "[CONFIG] Get/Set Friend State Request. Returning Friend feature is not supported.\n");
    current_state = MS_NOT_SUPPORTED;
    retval = API_SUCCESS;
    #else

    /**
        Parameter Validation.
        If Friend State is from RFU range, do not respond.
        This can happen only with Set request.
    */
    if (MS_ACCESS_CONFIG_FRIEND_SET_OPCODE == opcode)
    {
        current_state = (UINT8)(*data_param);

        if (MS_NOT_SUPPORTED <= current_state)
        {
            CONFIG_ERR(
                "[CONFIG] Set Friend State Request with invalid value 0x%02X. Dropping ...\n", current_state);
            /* Unlock */
            return API_SUCCESS;
        }

        /* Set current Friend access layer */
        /* Current State is already extracted at the start of this function */
        CONFIG_TRC(
            "[CONFIG] Set Friend State Request. Value 0x%02X\n", current_state);
        printf(
            "[CONFIG] Set Friend State Request. Value 0x%02X\n", current_state);
        /**
            TODO: If Friend state changes to value MS_DISABLE and if a node is a friend
            for one or more Low Power nodes, the node shall terminate all friend
            relationships and clear the associated Friend Queue.
        */
        retval = MS_access_cm_set_features_field(current_state, MS_FEATURE_FRIEND);
    }
    else /* MS_ACCESS_CONFIG_FRIEND_GET_OPCODE */
    {
        CONFIG_TRC(
            "[CONFIG] Get Friend State Request\n");
        /* Get current Friend access layer */
        retval = MS_access_cm_get_features_field(&current_state, MS_FEATURE_FRIEND);
        CONFIG_TRC(
            "[CONFIG] Current Friend State 0x%02X\n", current_state);
        printf(
            "[CONFIG] Current Friend State 0x%02X\n", current_state);
    }

    #endif /* MS_FRIEND_SUPPORT */
    /* Respond with Config Friend Status message */
    *response_opcode = MS_ACCESS_CONFIG_FRIEND_STATUS_OPCODE;
    response_buffer[0] = current_state;
    *response_buffer_len = sizeof(current_state);
    return retval;
}

/**
     \brief Get/Set the current Proxy State

     \par Description
     The Config Proxy Get/Set is an acknowledged messaged used to get/set the current
     Proxy state of a node.
     This handler routine will respond with Config Proxy Status message.
*/
DECL_STATIC API_RESULT config_gatt_proxy_get_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT8      current_state;
    UINT8      proxy_state;
    MS_IGNORE_UNUSED_PARAM(data_len);
    #ifndef MS_PROXY_SERVER
    MS_IGNORE_UNUSED_PARAM(data_param);
    MS_IGNORE_UNUSED_PARAM(opcode);
    MS_IGNORE_UNUSED_PARAM(proxy_state);
    /* Return Status as the Proxy feature is not supported */
    CONFIG_TRC(
        "[CONFIG] Get/Set Proxy State Request. Returning Proxy feature is not supported.\n");
    current_state = MS_NOT_SUPPORTED;
    retval = API_SUCCESS;
    #else /* MS_PROXY_SERVER */

    /**
        Parameter Validation.
        If Proxy State is from RFU range, do not respond.
        This can happen only with Set request.
    */
    if (MS_ACCESS_CONFIG_GATT_PROXY_SET_OPCODE == opcode)
    {
        API_RESULT ret;
        current_state = (UINT8)(*data_param);

        if (MS_NOT_SUPPORTED <= current_state)
        {
            CONFIG_ERR(
                "[CONFIG] Set Proxy State Request with invalid value 0x%02X. Dropping ...\n", current_state);
            /* Unlock */
            return API_SUCCESS;
        }

        CONFIG_TRC(
            "[CONFIG] Set Proxy State Request\n");
        printf(
            "[CONFIG] Set Proxy State Request\n");
        /* Set current Proxy access layer */
        retval = MS_access_cm_set_features_field(current_state, MS_FEATURE_PROXY);
        CONFIG_TRC(
            "[CONFIG] Current Proxy State 0x%02X\n", current_state);
        /* Check with Proxy Layer if Node Identity Set can be processed */
        ret = MS_proxy_fetch_state(&proxy_state);

        /**
            This Function Checks if proxy is ready to start connectable
            PROXY advertisements.
            | Proxy    |  Proxy |      Actions        |  Error
            | Callback |  Iface |                     |  Code
            |----------|--------|---------------------|---------------------
            |  NULL    |  Down  | Err Response No ADV | MS_PROXY_NULL
            |  NULL    |  Up    | Err Response No ADV | MS_PROXY_NULL
            |  !NULL   |  Down  | Response and ADV    | MS_PROXY_READY
            |  !NULL   |  UP    | Response No ADV     | MS_PROXY_CONNECTED
        */
        if (MS_PROXY_READY == proxy_state)
        {
            /* Subnet Handle here is for "ALL SUBNETS" */
            if (MS_ENABLE == current_state)
            {
                ret = MS_proxy_server_adv_start(MS_MAX_SUBNETS, MS_PROXY_NET_ID_ADV_MODE);
                CONFIG_TRC(
                    "[CONFIG] Proxy ADV with Network ID Started for ALL SUBNETS"
                    " with retval 0x%04X\n", retval);
                printf(
                    "[CONFIG] Proxy ADV with Network ID Started for ALL SUBNETS"
                    " with retval 0x%04X\n", retval);
            }
            else
            {
                ret = MS_proxy_server_adv_stop();
                CONFIG_TRC(
                    "[CONFIG] Proxy ADV with Network ID Stopped with retval 0x%04X\n",
                    retval);
                printf(
                    "[CONFIG] Proxy ADV with Network ID Stopped with retval 0x%04X\n",
                    retval);
            }
        }
        else if (MS_PROXY_NULL == proxy_state)
        {
            ret = API_FAILURE;
        }

        if (API_SUCCESS != ret)
        {
            CONFIG_ERR(
                "[CONFIG] Proxy Module Currently not ready ...\n");
            current_state = MS_TEMP_UNABLE_TO_CHANGE_STATE;
        }
    }
    else /* MS_ACCESS_CONFIG_GATT_PROXY_GET_OPCODE */
    {
        CONFIG_TRC(
            "[CONFIG] Get Proxy State Request\n");
        /* Get current Proxy access layer */
        retval = MS_access_cm_get_features_field(&current_state, MS_FEATURE_PROXY);
        CONFIG_TRC(
            "[CONFIG] Current Proxy State 0x%02X\n", current_state);
    }

    #endif /* MS_PROXY_SERVER */
    /* Respond with Config Proxy Status message */
    *response_opcode = MS_ACCESS_CONFIG_GATT_PROXY_STATUS_OPCODE;
    response_buffer[0] = current_state;
    *response_buffer_len = sizeof(current_state);
    return retval;
}

DECL_STATIC API_RESULT config_key_refresh_phase_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16     netkey_index;
    UINT8      key_refresh_state;
    MS_SUBNET_HANDLE   subnet;
    UCHAR*           node_id_status;
    UINT8            marker;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /* TODO: Make Get/Set Key Refresh Phase a common function */
    /**
        Extract
        - NetKey Index (2 Octets)
        - Transition (1 Octet)
    */
    node_id_status = response_buffer;

    if (3 == data_len)
    {
        MS_UNPACK_LE_2_BYTE(&netkey_index, data_param);
        key_refresh_state = data_param[2];
        CONFIG_TRC(
            "[CONFIG] Set Key Refresh Phase. NetKeyIndex 0x%04X with value 0x%02X\n", netkey_index, key_refresh_state);

        if ((MS_ACCESS_KEY_REFRESH_PHASE_2 != key_refresh_state) &&
                (MS_ACCESS_KEY_REFRESH_PHASE_3 != key_refresh_state))
        {
            CONFIG_ERR(
                "[CONFIG] Set Key Refresh Phase with invalid value 0x%02X. Dropping ...\n", key_refresh_state);
            /* Unlock */
            /* TODO: Return appropriate status code */
            return API_SUCCESS;
        }

        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     netkey_index,
                     &subnet
                 );

        /* Check if Subnet is found */
        if (API_SUCCESS == retval)
        {
            /* Set Key Refresh Phase */
            /* Another safety check */
            retval = MS_access_cm_set_key_refresh_phase
                     (
                         subnet,
                         &key_refresh_state
                     );

            /* Check if Delete is successful */
            if (ACCESS_NO_MATCH == retval)
            {
                /* TODO: Check what should be the right return value */
                retval = MS_INVALID_NETKEY_INDEX;
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Find Subnet failed. Retval 0x%04X\n", retval);
            /* TODO: Check what should be the right return value */
            retval = MS_INVALID_NETKEY_INDEX;
        }

        /* Respond only on valid state */
        if (API_FAILURE != retval)
        {
            /* Respond with Config Key Refresh Phase Status message */
            *response_opcode = MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_STATUS_OPCODE;
            marker = 0;
            node_id_status[marker] = (UCHAR)retval;
            marker++;
            MS_PACK_LE_2_BYTE_VAL(&node_id_status[marker], netkey_index);
            marker += 2;
            node_id_status[marker] = key_refresh_state;
            marker++;
            *response_buffer_len = marker;
            retval = API_SUCCESS;
        }
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_key_refresh_phase_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16     netkey_index;
    UINT8      key_refresh_state;
    MS_SUBNET_HANDLE   subnet;
    UCHAR*           node_id_status;
    UINT8            marker;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - NetKey Index (2 Octets)
    */
    node_id_status = response_buffer;

    if (2 == data_len)
    {
        MS_UNPACK_LE_2_BYTE(&netkey_index, data_param);
        CONFIG_TRC(
            "[CONFIG] Get Key Refresh Phase. Index 0x%04X\n", netkey_index);
        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     netkey_index,
                     &subnet
                 );

        /* Check if Subnet is found */
        if (API_SUCCESS == retval)
        {
            /* Get Key Refresh Phase */
            /* Another safety check */
            retval = MS_access_cm_get_key_refresh_phase
                     (
                         subnet,
                         &key_refresh_state
                     );

            /* Check if Delete is successful */
            if (ACCESS_NO_MATCH == retval)
            {
                /* TODO: Check what should be the right return value */
                retval = MS_INVALID_NETKEY_INDEX;
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Find Subnet failed. Retval 0x%04X\n", retval);
            key_refresh_state = MS_DISABLE;
            /* TODO: Check what should be the right return value */
            retval = MS_INVALID_NETKEY_INDEX;
        }

        /* Respond with Config Key Refresh Phase Status message */
        *response_opcode = MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_STATUS_OPCODE;
        marker = 0;
        node_id_status[marker] = (UCHAR)retval;
        marker ++;
        MS_PACK_LE_2_BYTE_VAL(&node_id_status[marker], netkey_index);
        marker += 2;
        node_id_status[marker] = key_refresh_state;
        marker ++;
        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_model_publication_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    MS_ACCESS_PUBLISH_INFO    publish_info;
    UCHAR*                    status;
    UINT8                     marker;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - ElementAddress (2 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if ((4 == data_len) || (6 == data_len))
    {
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param);

        /* Get Model ID type, based on the length */
        if (2 == (data_len - 2))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + 2));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + 2));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model Publication Get. ElementAddress 0x%04X\n", uaddr);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* To keep some compilers happy */
        EM_mem_set(&publish_info, 0, sizeof(publish_info));
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                retval = MS_access_cm_get_model_publication
                         (
                             model_handle,
                             &publish_info
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Get Model Publication successful.\n");
                    /* Print Publish Info */
                    CONFIG_TRC(
                        "[CONFIG] AppKeyIndex 0x%04X\n", publish_info.appkey_index);
                    CONFIG_TRC(
                        "[CONFIG] CredentialFlag 0x%02X\n", publish_info.crden_flag);
                    CONFIG_TRC(
                        "[CONFIG] PublishTTL 0x%02X\n", publish_info.ttl);
                    CONFIG_TRC(
                        "[CONFIG] PublishPeriod 0x%02X\n", publish_info.period);
                    CONFIG_TRC(
                        "[CONFIG] PublishRetransmitCount 0x%02X\n", publish_info.rtx_count);
                    CONFIG_TRC(
                        "[CONFIG] PublishRetransmitIntervalSteps 0x%02X\n", publish_info.rtx_interval_steps);
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Get Model Publication Failed. Retval 0x%04X\n", retval);
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            retval = MS_INVALID_ADDRESS;
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Respond with Config Model Publication Status message */
        *response_opcode = MS_ACCESS_CONFIG_MODEL_PUBLICATION_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Pack ElementAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;
        /* Pack PublishAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], publish_info.addr.addr);
        marker += 2;
        /* Pack AppKeyIndex and CredentialFlag */
        status[marker] = (UCHAR) (publish_info.appkey_index);
        marker ++;
        status[marker] = (UCHAR)((publish_info.appkey_index >> 8) & 0x0F);
        status[marker] |= (UCHAR) (publish_info.crden_flag << 4);
        marker ++;
        /* Pack PublishTTL */
        status[marker] = publish_info.ttl;
        marker ++;
        /* Pack PublishPeriod */
        status[marker] = publish_info.period;
        marker ++;
        /* Pack PublishRetransmitCount and PublishRetransmitIntervalSteps */
        status[marker] = (UCHAR)(publish_info.rtx_count & 0x07);
        status[marker] |= (UCHAR)(publish_info.rtx_interval_steps << 3);
        marker ++;

        if (MS_ACCESS_MODEL_TYPE_SIG == model_id.type)
        {
            /* Pack Model SIG ID - 2 Octets */
            model_id_16 = (UINT16)model_id.id;
            MS_PACK_LE_2_BYTE_VAL(&status[marker], model_id_16);
            marker += 2;
        }
        else
        {
            /* Pack Model Vendor ID - 4 Octets */
            MS_PACK_LE_4_BYTE_VAL(&status[marker], model_id.id);
            marker += 4;
        }

        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_model_subscription_add_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    MS_ACCESS_ADDRESS         addr;
    UCHAR*                    status;
    UINT8                     marker;
    /**
        Extract
        - ElementAddress (2 Octets)
        - SubscriptionAddress (2 Octets) or Label (16 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if (((6 == data_len) || (8 == data_len)) || ((20 == data_len) || (22 == data_len)))
    {
        marker = 0;
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param + marker);
        marker += 2;

        /* SubscriptionAddress - non-virtual address */
        if (opcode == MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_ADD_OPCODE)
        {
            addr.use_label = 0;
            MS_UNPACK_LE_2_BYTE(&addr.addr, data_param + marker);
            marker += 2;
            CONFIG_TRC("[CONFIG] Subscription Address 0x%04X\n", addr.addr);
        }
        else
        {
            addr.use_label = 1;
            EM_mem_copy(addr.label, data_param + marker, MS_LABEL_UUID_LENGTH);
            marker += MS_LABEL_UUID_LENGTH;
            addr.addr = 0;
        }

        /* Get Model ID type, based on the length */
        if (2 == (data_len - marker))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + marker));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + marker));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model Subscription Add. ElementAddress 0x%04X\n", uaddr);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                // ========HZF
//UINT32    T1, T2;
//blebrr_scan_pl(FALSE);
//
//T1 = clock_time_rtc();//read_current_fine_time();
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                retval = MS_access_cm_add_model_subscription
                         (
                             model_handle,
                             &addr
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Model Subscription Add successful.\n");
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Model Subscription Add Failed. Retval 0x%04X\n", retval);
                }

//T2 = clock_time_rtc();
//printf("consume time of function MS_access_cm_add_model_subscription: %d RTC tick\r\n", (T2 - T1));
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Respond with Config Model Subscription Status message */
        *response_opcode = MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Pack ElementAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;
        /* Pack SubscriptionAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], addr.addr);
        marker += 2;

        if (MS_ACCESS_MODEL_TYPE_SIG == model_id.type)
        {
            /* Pack Model SIG ID - 2 Octets */
            model_id_16 = (UINT16)model_id.id;
            MS_PACK_LE_2_BYTE_VAL(&status[marker], model_id_16);
            marker += 2;
        }
        else
        {
            /* Pack Model Vendor ID - 4 Octets */
            MS_PACK_LE_4_BYTE_VAL(&status[marker], model_id.id);
            marker += 4;
        }

        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_model_subscription_delete_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    MS_ACCESS_ADDRESS         addr;
    UCHAR*                    status;
    UINT8                     marker;
    /**
        Extract
        - ElementAddress (2 Octets)
        - SubscriptionAddress (2 Octets) or Label (16 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if (((6 == data_len) || (8 == data_len)) || ((20 == data_len) || (22 == data_len)))
    {
        marker = 0;
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param + marker);
        marker += 2;

        /* SubscriptionAddress - non-virtual address */
        if (opcode == MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE)
        {
            addr.use_label = 0;
            MS_UNPACK_LE_2_BYTE(&addr.addr, data_param + marker);
            marker += 2;
            CONFIG_TRC("[CONFIG] Subscription Address 0x%04X\n", addr.addr);
        }
        else
        {
            addr.use_label = 1;
            EM_mem_copy(addr.label, data_param + marker, MS_LABEL_UUID_LENGTH);
            marker += MS_LABEL_UUID_LENGTH;
            addr.addr = 0;
        }

        /* Get Model ID type, based on the length */
        if (2 == (data_len - marker))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + marker));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + marker));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model Subscription Delete. ElementAddress 0x%04X\n", uaddr);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                retval = MS_access_cm_delete_model_subscription
                         (
                             model_handle,
                             &addr
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Model Subscription Delete successful.\n");
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Model Subscription Delete Failed. Retval 0x%04X\n", retval);
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Respond with Config Model Subscription Status message */
        *response_opcode = MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Pack ElementAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;
        /* Pack SubscriptionAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], addr.addr);
        marker += 2;

        if (MS_ACCESS_MODEL_TYPE_SIG == model_id.type)
        {
            /* Pack Model SIG ID - 2 Octets */
            model_id_16 = (UINT16)model_id.id;
            MS_PACK_LE_2_BYTE_VAL(&status[marker], model_id_16);
            marker += 2;
        }
        else
        {
            /* Pack Model Vendor ID - 4 Octets */
            MS_PACK_LE_4_BYTE_VAL(&status[marker], model_id.id);
            marker += 4;
        }

        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_model_subscription_delete_all_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    UCHAR*                    status;
    UINT8                     marker;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - ElementAddress (2 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if ((4 == data_len) || (6 == data_len))
    {
        marker = 0;
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param + marker);
        marker += 2;

        /* Get Model ID type, based on the length */
        if (2 == (data_len - marker))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + marker));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + marker));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model Subscription Delete All. ElementAddress 0x%04X\n", uaddr);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                retval = MS_access_cm_delete_all_model_subscription
                         (
                             model_handle
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Model Subscription Delete All successful.\n");
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Model Subscription Delete All Failed. Retval 0x%04X\n", retval);
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Respond with Config Model Subscription Status message */
        *response_opcode = MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Pack ElementAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;
        /* Pack SubscriptionAddress - as unassigned address */
        uaddr = MS_NET_ADDR_UNASSIGNED;
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;

        if (MS_ACCESS_MODEL_TYPE_SIG == model_id.type)
        {
            /* Pack Model SIG ID - 2 Octets */
            model_id_16 = (UINT16)model_id.id;
            MS_PACK_LE_2_BYTE_VAL(&status[marker], model_id_16);
            marker += 2;
        }
        else
        {
            /* Pack Model Vendor ID - 4 Octets */
            MS_PACK_LE_4_BYTE_VAL(&status[marker], model_id.id);
            marker += 4;
        }

        *response_buffer_len = marker;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_model_subscription_overwrite_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    MS_ACCESS_ADDRESS         addr;
    UCHAR*                    status;
    UINT8                     marker;
    /**
        Extract
        - ElementAddress (2 Octets)
        - SubscriptionAddress (2 Octets) or Label (16 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if (((6 == data_len) || (8 == data_len)) || ((20 == data_len) || (22 == data_len)))
    {
        marker = 0;
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param + marker);
        marker += 2;

        /* SubscriptionAddress - non-virtual address */
        if (opcode == MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE_OPCODE)
        {
            addr.use_label = 0;
            MS_UNPACK_LE_2_BYTE(&addr.addr, data_param + marker);
            marker += 2;
            CONFIG_TRC("[CONFIG] Subscription Address 0x%04X\n", addr.addr);
        }
        else
        {
            addr.use_label = 1;
            EM_mem_copy(addr.label, data_param + marker, MS_LABEL_UUID_LENGTH);
            marker += MS_LABEL_UUID_LENGTH;
            addr.addr = 0;
        }

        /* Get Model ID type, based on the length */
        if (2 == (data_len - marker))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + marker));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + marker));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model Subscription Overwrite. ElementAddress 0x%04X\n", uaddr);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                retval = MS_access_cm_delete_all_model_subscription
                         (
                             model_handle
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Model Subscription Delete All successful.\n");
                    /* Add SubscriptionAddress */
                    retval = MS_access_cm_add_model_subscription
                             (
                                 model_handle,
                                 &addr
                             );

                    if (API_SUCCESS == retval)
                    {
                        CONFIG_TRC(
                            "[CONFIG] Model Subscription Add successful\n");
                    }
                    else
                    {
                        CONFIG_ERR(
                            "[CONFIG] Model Subscription Add Failed. Retval 0x%04X\n", retval);
                    }
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Model Subscription Delete All Failed. Retval 0x%04X\n", retval);
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Respond with Config Model Subscription Status message */
        *response_opcode = MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Pack ElementAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;
        /* Pack SubscriptionAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], addr.addr);
        marker += 2;

        if (MS_ACCESS_MODEL_TYPE_SIG == model_id.type)
        {
            /* Pack Model SIG ID - 2 Octets */
            model_id_16 = (UINT16)model_id.id;
            MS_PACK_LE_2_BYTE_VAL(&status[marker], model_id_16);
            marker += 2;
        }
        else
        {
            /* Pack Model Vendor ID - 4 Octets */
            MS_PACK_LE_4_BYTE_VAL(&status[marker], model_id.id);
            marker += 4;
        }

        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_network_transmit_get_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT8      tx_state;
    MS_IGNORE_UNUSED_PARAM(data_len);

    if (MS_ACCESS_CONFIG_NETWORK_TRANSMIT_GET_OPCODE == opcode)
    {
        CONFIG_TRC(
            "[CONFIG] Get Network Transmit State Request\n");
        /* Get Network Transmit State */
        retval = MS_access_cm_get_transmit_state(MS_NETWORK_TX_STATE, &tx_state);
        CONFIG_TRC(
            "[CONFIG] Current Network Transmit State 0x%02X\n", tx_state);
    }
    else
    {
        /* Set Network Transmit State */
        tx_state = (UINT8)(*data_param);
        CONFIG_TRC(
            "[CONFIG] Set Network Transmit State Request. Value 0x%02X\n", tx_state);
        /**
            TODO: If Tx state changes to value MS_DISABLE/MS_ENABLE, what needs to be done?
        */
        retval = MS_access_cm_set_transmit_state(MS_NETWORK_TX_STATE, tx_state);
    }

    /* Respond with Config Beacon Status message */
    *response_opcode = MS_ACCESS_CONFIG_NETWORK_TRANSMIT_STATUS_OPCODE;
    response_buffer[0] = tx_state;
    *response_buffer_len = sizeof(tx_state);
    return retval;
}

DECL_STATIC API_RESULT config_relay_get_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT8      relay;
    UINT8      tx_state;
    UCHAR*     status;
    MS_IGNORE_UNUSED_PARAM(data_len);
    /* TODO: Parameter length validation */
    status = response_buffer;
    #ifndef MS_RELAY_SUPPORT
    /* Return Status as the Relay feature is not supported */
    CONFIG_TRC(
        "[CONFIG] Get/Set Relay State Request. Returning Relay feature is not supported.\n");
    relay    = MS_NOT_SUPPORTED;
    tx_state = MS_DISABLE;
    retval   = API_SUCCESS;
    #else /* MS_RELAY_SUPPORT */

    if (MS_ACCESS_CONFIG_RELAY_GET_OPCODE == opcode)
    {
        /* Get current Relay State */
        retval = MS_access_cm_get_features_field(&relay, MS_FEATURE_RELAY);
        CONFIG_TRC(
            "[CONFIG] Current Relay State 0x%02X\n", relay);
        CONFIG_TRC(
            "[CONFIG] Get Relay Transmit State\n");
        /* Get Relay Transmit State */
        retval = MS_access_cm_get_transmit_state(MS_RELAY_TX_STATE, &tx_state);
        CONFIG_TRC(
            "[CONFIG] Current Relay Transmit State 0x%02X\n", tx_state);
        printf(
            "[CONFIG] Current Relay Transmit State 0x%02X\n", tx_state);
    }
    else /* if (MS_ACCESS_CONFIG_RELAY_SET_OPCODE == opcode) */
    {
        /* Set Relay State */
        relay = data_param[0];
        /* Set Relay Transmit State */
        tx_state = data_param[1];
        CONFIG_TRC(
            "[CONFIG] Set Relay Transmit State Request. Relay: 0x%02X, Value 0x%02X\n", relay, tx_state);
        printf(
            "[CONFIG] Set Relay Transmit State Request. Relay: 0x%02X, Value 0x%02X\n", relay, tx_state);

        if (MS_NOT_SUPPORTED <= relay)
        {
            CONFIG_ERR(
                "[CONFIG] Set Relay State Request with invalid value 0x%02X. Dropping ...\n", relay);
            /* Unlock */
            return API_SUCCESS;
        }

        /* Set current Relay State */
        retval = MS_access_cm_set_features_field(relay, MS_FEATURE_RELAY);

        /**
            TODO: If Tx state changes to value MS_DISABLE/MS_ENABLE, what needs to be done?
        */
        if(relay)
            retval = MS_access_cm_set_transmit_state(MS_RELAY_TX_STATE, tx_state);
    }

    #endif /* MS_RELAY_SUPPORT */
    /* Frame status message */
    status[0] = relay;
    status[1] = tx_state;
    /* Respond with Config Relay Status message */
    *response_opcode = MS_ACCESS_CONFIG_RELAY_STATUS_OPCODE;
    *response_buffer_len = 2;
    return retval;
}

DECL_STATIC API_RESULT config_model_subscription_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    UINT32                    index;
    UINT16                    addr_count;
    /* TODO: Allocate */
    UINT16                    addr_list[MS_MAX_ADDRS];
    /* TODO: Check Size */
    /* UCHAR                     status[1 + 2 + 4 + (2 * MS_MAX_ADDRS)]; */
    UINT8*                    status;
    UINT8                     marker;
    addr_count = 0;
    /**
        Extract
        - ElementAddress (2 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if ((4 == data_len) || (6 == data_len))
    {
        marker = 0;
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param + marker);
        marker += 2;

        /* Get Model ID type, based on the length */
        if (2 == (data_len - marker))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + marker));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + marker));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model Subscription Get. ElementAddress 0x%04X\n", uaddr);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                addr_count = MS_MAX_ADDRS;
                retval = MS_access_cm_get_model_subscription_list
                         (
                             model_handle,
                             &addr_count,
                             addr_list
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Model Subscription Get successful. Returned %d Subscription Addresses\n", addr_count);
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Model Subscription Get Failed. Retval 0x%04X\n", retval);

                    if (ACCESS_NO_MATCH == retval)
                    {
                        retval = MS_SUCCESS;
                    }
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Respond with Config SIG/Vendor Model Subscription List message */
        if (MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_GET_OPCODE == opcode)
        {
            *response_opcode = MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_LIST_OPCODE;
        }
        else
        {
            *response_opcode = MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_LIST_OPCODE;
        }

        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Pack ElementAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;

        if (MS_ACCESS_MODEL_TYPE_SIG == model_id.type)
        {
            /* Pack Model SIG ID - 2 Octets */
            model_id_16 = (UINT16)model_id.id;
            MS_PACK_LE_2_BYTE_VAL(&status[marker], model_id_16);
            marker += 2;
        }
        else
        {
            /* Pack Model Vendor ID - 4 Octets */
            MS_PACK_LE_4_BYTE_VAL(&status[marker], model_id.id);
            marker += 4;
        }

        /* Pack Subscription Addresses */
        for (index = 0; index < addr_count; index++)
        {
            MS_PACK_LE_2_BYTE_VAL(&status[marker], addr_list[index]);
            marker += 2;
        }

        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_low_power_node_polltimeout_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    #ifdef MS_FRIEND_SUPPORT
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    UINT32      poll_to;
    UCHAR*      status;
    UINT8       marker;
    UINT8       addr_type;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - LPNAddress (2 Octets)
    */
    status = response_buffer;

    if (2 == data_len)
    {
        marker = 0;
        /* LPNAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param + marker);
        marker += 2;
        /* Check if a valid address */
        addr_type = MS_net_get_address_type(uaddr);

        /**
            LPNAddress shall be an Unicast Address
            - Shall not be Unassigned Address (already checked at network layer).
            - Control Message shall not be a Virtual Address.
        */
        if (MS_NET_ADDR_TYPE_UNICAST != addr_type)
        {
            CONFIG_TRC("[CONFIG] Invalid LPN address: 0x%04X. Dropping...\n", uaddr);
            return API_FAILURE;
        }

        CONFIG_TRC("[CONFIG] Get PollTimeout for LPN 0x%04X\n", uaddr);
        /* Get associated PollTimeOut Value */
        poll_to = 0x00000000;
        retval = MS_trn_get_lpn_polltimeout
                 (
                     uaddr,
                     &poll_to
                 );

        if (API_SUCCESS != retval)
        {
            CONFIG_ERR(
                "[CONFIG] Get LPN:0x%04X PollTO Failed. Retval 0x%04X\n", uaddr, retval);
        }

        CONFIG_TRC(
            "[CONFIG] LPN:0x%04X, PollTO:0x%06X\n", uaddr, poll_to);
        retval = API_SUCCESS;
        *response_opcode = MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        /* Pack LPNAddress */
        MS_PACK_LE_2_BYTE_VAL(&status[marker], uaddr);
        marker += 2;
        /* Pack Poll TO - 3 Octets */
        MS_PACK_LE_3_BYTE_VAL(&status[marker], poll_to);
        marker += 3;
        *response_buffer_len = marker;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
    #else
    return API_FAILURE;
    #endif /* MS_FRIEND_SUPPORT */
}

DECL_STATIC API_RESULT config_heartbeat_publication_get_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    MS_TRN_HEARTBEAT_PUBLICATION_INFO pub;
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(data_len);

    if (MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_GET_OPCODE == opcode)
    {
        CONFIG_TRC("[CONFIG] Heartbeat Publication Get\n");
        retval = MS_trn_get_heartbeat_publication(&pub);
    }
    else
    {
        CONFIG_TRC("[CONFIG] Heartbeat Publication Set\n");
        /* Extract the parameters */
        MS_UNPACK_LE_2_BYTE(&pub.daddr, data_param);
        MS_UNPACK_LE_1_BYTE(&pub.count_log, (data_param + 2));
        MS_UNPACK_LE_1_BYTE(&pub.period_log, (data_param + 3));
        MS_UNPACK_LE_1_BYTE(&pub.ttl, (data_param + 4));
        MS_UNPACK_LE_2_BYTE(&pub.features, (data_param + 5));
        MS_UNPACK_LE_2_BYTE(&pub.netkey_index, (data_param + 7));
        retval = MS_trn_set_heartbeat_publication(&pub);
    }

    CONFIG_TRC("[CONFIG] Retval:0x%04X. DADDR:0x%04X, CountLog:0x%02X, PeriodLog:0x%02X, TTL:0x%02X, Features:0x%04x, NetKeyIndex:0x%04X\n",
               retval, pub.daddr, pub.count_log, pub.period_log, pub.ttl, pub.features, pub.netkey_index);
    /* Frame the response */
    *response_opcode = MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_STATUS_OPCODE;
    /* if (API_SUCCESS == retval) */
    {
        response_buffer[0] = (UCHAR)retval;
        MS_PACK_LE_2_BYTE((response_buffer + 1), &(pub.daddr));
        MS_PACK_LE_1_BYTE((response_buffer + 3), &(pub.count_log));
        MS_PACK_LE_1_BYTE((response_buffer + 4), &(pub.period_log));
        MS_PACK_LE_1_BYTE((response_buffer + 5), &(pub.ttl));
        MS_PACK_LE_2_BYTE((response_buffer + 6), &(pub.features));
        MS_PACK_LE_2_BYTE((response_buffer + 8), &(pub.netkey_index));
        *response_buffer_len = 10;
        retval = API_SUCCESS;
    }
    return retval;
}

DECL_STATIC API_RESULT config_heartbeat_subscription_get_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    MS_TRN_HEARTBEAT_SUBSCRIPTION_INFO sub;
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(data_len);
    retval = API_SUCCESS;

    if (MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_GET_OPCODE == opcode)
    {
        CONFIG_TRC("[CONFIG] Heartbeat Subscription Get\n");
        retval = MS_trn_get_heartbeat_subscription(&sub);
    }
    else
    {
        CONFIG_TRC("[CONFIG] Heartbeat Subscription Set\n");
        /* Extract the parameters */
        MS_UNPACK_LE_2_BYTE(&sub.saddr, data_param);
        MS_UNPACK_LE_2_BYTE(&sub.daddr, (data_param + 2));
        MS_UNPACK_LE_1_BYTE(&sub.period_log, (data_param + 4));
        CONFIG_TRC("[CONFIG] SADDR:0x%04X, DADDR:0x%04X, PeriodLog:0x%02X\n",
                   sub.saddr, sub.daddr, sub.period_log);
        /* Set the subscription status */
        retval = MS_trn_set_heartbeat_subscription(&sub);
    }

    CONFIG_TRC("[CONFIG] Retval:0x%04X. SADDR:0x%04X, DADDR:0x%04X, PeriodLog:0x%02X, CountLog:0x%02X, MinHops:0x%02X, MaxHops:0x%02X\n",
               retval, sub.saddr, sub.daddr, sub.period_log, sub.count_log, sub.min_hops, sub.max_hops);

    /* Frame the response */
    if (API_SUCCESS == retval)
    {
        *response_opcode = MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_STATUS_OPCODE;
        response_buffer[0] = 0x00; /* Success */
        MS_PACK_LE_2_BYTE((response_buffer + 1), &(sub.saddr));
        MS_PACK_LE_2_BYTE((response_buffer + 3), &(sub.daddr));
        MS_PACK_LE_1_BYTE((response_buffer + 5), &(sub.period_log));
        MS_PACK_LE_1_BYTE((response_buffer + 6), &(sub.count_log));
        MS_PACK_LE_1_BYTE((response_buffer + 7), &(sub.min_hops));
        MS_PACK_LE_1_BYTE((response_buffer + 8), &(sub.max_hops));
        *response_buffer_len = 9;
    }

    return retval;
}

DECL_STATIC API_RESULT config_model_app_bind_unbind_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    UINT16      appkey_index;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    UINT8*           status;
    UINT16           marker;
    /**
        Extract
        - ElementAddress (2 Octets)
        - AppKeyIndex (2 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    status = response_buffer;

    if ((6 == data_len) || (8 == data_len))
    {
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param);
        /* AppKeyIndex */
        MS_UNPACK_LE_2_BYTE(&appkey_index, (data_param + 2));

        /* Get Model ID type, based on the length */
        if (2 == (data_len - 4))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + 4));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + 4));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model App Bind/Unbind. ElementAddress 0x%04X, AppKeyIndex 0x%04X\n", uaddr, appkey_index);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);

                if (MS_ACCESS_CONFIG_MODEL_APP_BIND_OPCODE == opcode)
                {
                    retval = MS_access_bind_model_app
                             (
                                 model_handle,
                                 appkey_index
                             );
                }
                else
                {
                    retval = MS_access_unbind_model_app
                             (
                                 model_handle,
                                 appkey_index
                             );
                }

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Binding Model Handle 0x%04X and AppKey Index 0x%04X successful.\n",
                        model_handle, appkey_index);
                    printf(
                        "[CONFIG] Binding Model Handle 0x%04X and AppKey Index 0x%04X successful.\n",
                        model_handle, appkey_index);
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Binding Model Handle 0x%04X and AppKey Index 0x%04X Failed. Retval 0x%04X\n",
                        model_handle, appkey_index, retval);
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Associate/Bind AppKeyIndex with Model instance */
        /* Respond with Config Model App Status message */
        *response_opcode = MS_ACCESS_CONFIG_MODEL_APP_STATUS_OPCODE;
        /* Frame Response */
        marker = 0;
        status[marker] = (UCHAR)retval;
        marker ++;
        /* Copy back the request message */
        EM_mem_copy(&status[marker], data_param, data_len);
        marker += data_len;
        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_netkey_add_update_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16     netkey_index;
    UCHAR*     net_key;
    UCHAR*           netkey_status;
    UINT8            marker;
    /**
        Extract
        - NetKey Index (2 Octets)
        - NetKey (16 Octets)
    */
    netkey_status = response_buffer;

    if (18 == data_len)
    {
        MS_UNPACK_LE_2_BYTE(&netkey_index, data_param);
        net_key = (data_param + 2);
        CONFIG_TRC(
            "[CONFIG] Add/Update NetKey. Index 0x%04X\n", netkey_index);
        CONFIG_debug_dump_bytes(net_key, 16);
        /* Add/Update NetKey */
        retval = MS_access_cm_add_update_netkey
                 (
                     netkey_index,
                     opcode,
                     net_key
                 );
        /* Respond with Config NetKey Status message */
        *response_opcode = MS_ACCESS_CONFIG_NETKEY_STATUS_OPCODE;
        marker = 0;
        netkey_status[marker] = (UCHAR)retval;
        marker ++;
        MS_PACK_LE_2_BYTE_VAL(&netkey_status[marker], netkey_index);
        marker += 2;
        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_netkey_delete_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16     netkey_index;
    MS_SUBNET_HANDLE   subnet;
    UINT8*           netkey_status;
    UINT8            marker;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - NetKey Index (2 Octets)
    */
    netkey_status = response_buffer;

    if (2 == data_len)
    {
        MS_UNPACK_LE_2_BYTE(&netkey_index, data_param);
        CONFIG_TRC(
            "[CONFIG] Delete NetKey. Index 0x%04X\n", netkey_index);
        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     netkey_index,
                     &subnet
                 );

        /* Check if Subnet is found */
        if (API_SUCCESS == retval)
        {
            /* Delete NetKey */
            /* Another safety check */
            retval = MS_access_cm_delete_netkey
                     (
                         subnet
                     );

            /* Check if Delete is successful */
            if (API_SUCCESS != retval)
            {
                /* TODO: Check what should be the right return value */
                /* retval = MS_INVALID_NETKEY_INDEX; */
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Find Subnet failed. Retval 0x%04X\n", retval);
            /* TODO: Check what should be the right return value */
            /* retval = MS_INVALID_NETKEY_INDEX; */
            retval = MS_SUCCESS;
        }

        /* Respond with Config NetKey Status message */
        *response_opcode = MS_ACCESS_CONFIG_NETKEY_STATUS_OPCODE;
        marker = 0;
        netkey_status[marker] = (UCHAR)retval;
        marker ++;
        MS_PACK_LE_2_BYTE_VAL(&netkey_status[marker], netkey_index);
        marker += 2;
        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_netkey_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16           netkey_count;
    /* TODO: Allocate memory based on the MS_MAX_SUBNETS */
    UINT16           netkey_index_list[10 /* MS_MAX_SUBNETS */];
    /* TODO: Size */
    /* UINT8            netkey_index_list_pdu[MS_MAX_SUBNETS * 2]; */
    UINT8*           netkey_index_list_pdu;
    UINT16           netkey_index_list_pdu_length;
    MS_IGNORE_UNUSED_PARAM(opcode);
    MS_IGNORE_UNUSED_PARAM(data_param);
    /**
        TODO: Add the parameter length checking in a common place and
        return error if required from there
    */
    /* No Parameter */
    netkey_index_list_pdu = response_buffer;

    if (0 == data_len)
    {
        CONFIG_TRC(
            "[CONFIG] Get NetKey List\n");
        /* Get NetKey List */
        netkey_count = sizeof(netkey_index_list) / sizeof(UINT16);
        retval = MS_access_cm_get_netkey_index_list
                 (
                     &netkey_count,
                     netkey_index_list
                 );
        CONFIG_TRC(
            "[CONFIG] NetKey List returned 0x%04X. Count 0x%04X\n", retval, netkey_count);

        if (API_SUCCESS == retval)
        {
            /* Pack the NetKey Indices */
            netkey_index_list_pdu_length = sizeof(netkey_index_list_pdu);
            /* TODO: Not checking retval */
            config_pack_key_indices
            (
                netkey_count,
                netkey_index_list,
                netkey_index_list_pdu,
                &netkey_index_list_pdu_length
            );
            CONFIG_TRC(
                "[CONFIG] NetKey Indices ater packing\n");
            CONFIG_debug_dump_bytes(netkey_index_list_pdu, netkey_index_list_pdu_length);
            /* Respond with Config NetKey List message */
            *response_opcode = MS_ACCESS_CONFIG_NETKEY_LIST_OPCODE;
            *response_buffer_len = netkey_index_list_pdu_length;
        }
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_node_identity_set_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16     netkey_index;
    UINT8      id_state;
    MS_SUBNET_HANDLE   subnet;
    UINT8*           node_id_status;
    UINT8            marker;
    UINT8            proxy_state;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /* TODO: Make Get/Set Node Identity address a common function */
    /**
        Extract
        - NetKey Index (2 Octets)
        - Node Identity State (1 Octet)
    */
    node_id_status = response_buffer;

    if (3 == data_len)
    {
        MS_UNPACK_LE_2_BYTE(&netkey_index, data_param);
        id_state = data_param[2];
        CONFIG_TRC(
            "[CONFIG] Set Node Identity. NetKeyIndex 0x%04X with value 0x%02X\n", netkey_index, id_state);
        printf(
            "[CONFIG] Set Node Identity. NetKeyIndex 0x%04X with value 0x%02X\n", netkey_index, id_state);
        #ifndef MS_PROXY_SERVER
        MS_IGNORE_UNUSED_PARAM(proxy_state);
        MS_IGNORE_UNUSED_PARAM(subnet);
        CONFIG_ERR(
            "[CONFIG] Set Node Identity Request not supported as MS_PROXY_SERVER feature is Disabled!\n");
        retval = MS_FEATURE_NOT_SUPPORTED;
        id_state = MS_NOT_SUPPORTED;
        #else /* MS_PROXY_SERVER */

        if (MS_NOT_SUPPORTED <= id_state)
        {
            CONFIG_ERR(
                "[CONFIG] Set Node Identity Request with invalid value 0x%02X. Dropping ...\n", id_state);
            /* Unlock */
            return API_SUCCESS;
        }

        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     netkey_index,
                     &subnet
                 );

        /* Check if Subnet is found */
        if (API_SUCCESS == retval)
        {
            /* Set Node Identity */
            /* Another safety check */
            retval = MS_access_cm_set_node_identity
                     (
                         subnet,
                         &id_state
                     );

            /* Check if Node Identity Set is successful */
            if (ACCESS_NO_MATCH == retval)
            {
                /* TODO: Check what should be the right return value */
                retval = MS_INVALID_NETKEY_INDEX;
            }
            else
            {
                /* Check with Proxy Layer if Node Identity Set can be processed */
                retval = MS_proxy_fetch_state(&proxy_state);

                /**
                    This Function Checks if application is ready to handle
                    PROXY related messages.
                    | Proxy    |  Proxy |      Actions        |  Error
                    | Callback |  Iface |                     |  Code
                    |----------|--------|---------------------|---------------------
                    |  NULL    |  Down  | Err Response No ADV | MS_PROXY_NULL
                    |  NULL    |  Up    | Err Response No ADV | MS_PROXY_NULL
                    |  !NULL   |  Down  | Response and ADV    | MS_PROXY_READY
                    |  !NULL   |  UP    | Response No ADV     | MS_PROXY_CONNECTED
                */
                if (MS_PROXY_READY == proxy_state)
                {
                    /**
                        Start Proxy ADV with Node Identity for the given Subnet,
                        If current Node Identity state is SET.
                    */
                    if (MS_ENABLE == (id_state))
                    {
                        retval = MS_proxy_server_adv_start(subnet, MS_PROXY_NODE_ID_ADV_MODE);
                        CONFIG_TRC(
                            "[CONFIG] Proxy ADV with Node Identity Started for Subnet 0x%04X"
                            " with retval 0x%04X\n", subnet, retval);
                        printf(
                            "[CONFIG] Proxy ADV with Node Identity Started for Subnet 0x%04X"
                            " with retval 0x%04X\n", subnet, retval);
                    }
                    else
                    {
                        retval = MS_proxy_server_adv_stop();
                        CONFIG_TRC(
                            "[CONFIG] Proxy ADV with Node Identity Stopped with retval 0x%04X\n",
                            retval);
                        printf(
                            "[CONFIG] Proxy ADV with Node Identity Stopped with retval 0x%04X\n",
                            retval);
                    }
                }
                else if (MS_PROXY_NULL == proxy_state)
                {
                    retval = API_FAILURE;
                }

                if (API_SUCCESS != retval)
                {
                    CONFIG_ERR(
                        "[CONFIG] Proxy Module Currently not ready ...\n");
                    retval = MS_TEMP_UNABLE_TO_CHANGE_STATE;
                    id_state = MS_DISABLE;
                }
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Find Subnet failed. Retval 0x%04X\n", retval);
            /* TODO: Check what should be the right return value */
            retval = MS_INVALID_NETKEY_INDEX;
        }

        #endif /* MS_PROXY_SERVER */
        /* Respond with Config Node Identity Status message */
        *response_opcode = MS_ACCESS_CONFIG_NODE_IDENTITY_STATUS_OPCODE;
        marker = 0;
        node_id_status[marker] = (UCHAR)retval;
        marker ++;
        MS_PACK_LE_2_BYTE_VAL(&node_id_status[marker], netkey_index);
        marker += 2;
        node_id_status[marker] = id_state;
        marker ++;
        *response_buffer_len = marker;
        /* Reassign Retval as Success after packing the value into response */
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_node_identity_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    UINT16     netkey_index;
    UINT8      id_state;
    MS_SUBNET_HANDLE   subnet;
    UCHAR*           node_id_status;
    UINT8            marker;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - NetKey Index (2 Octets)
    */
    node_id_status = response_buffer;

    if (2 == data_len)
    {
        MS_UNPACK_LE_2_BYTE(&netkey_index, data_param);
        CONFIG_TRC(
            "[CONFIG] Get Node Identity. Index 0x%04X\n", netkey_index);
        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     netkey_index,
                     &subnet
                 );

        /* Check if Subnet is found */
        if (API_SUCCESS == retval)
        {
            /* Get Node Identity */
            /* Another safety check */
            retval = MS_access_cm_get_node_identity
                     (
                         subnet,
                         &id_state
                     );

            /* Check if Delete is successful */
            if (ACCESS_NO_MATCH == retval)
            {
                /* TODO: Check what should be the right return value */
                retval = MS_INVALID_NETKEY_INDEX;
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Find Subnet failed. Retval 0x%04X\n", retval);
            /* If not found, return Node Identity State as MS_DISABLE/MS_NOT_SUPPORTED */
            #ifndef MS_PROXY_SERVER
            /**
                If the Proxy Server is not supported, the Node Identity state value shall be MS_NOT_SUPPORTED.
            */
            id_state = MS_NOT_SUPPORTED;
            #else /* MS_PROXY_SERVER */
            id_state = MS_DISABLE;
            #endif /* MS_PROXY_SERVER */
            /* TODO: Check what should be the right return value */
            retval = MS_INVALID_NETKEY_INDEX;
        }

        /* Respond with Config Node Identity Status message */
        *response_opcode = MS_ACCESS_CONFIG_NODE_IDENTITY_STATUS_OPCODE;
        marker = 0;
        node_id_status[marker] = (UCHAR)retval;
        marker ++;
        MS_PACK_LE_2_BYTE_VAL(&node_id_status[marker], netkey_index);
        marker += 2;
        node_id_status[marker] = id_state;
        marker ++;
        *response_buffer_len = marker;
        /* Reassign Retval as Success after packing the value into response */
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}

DECL_STATIC API_RESULT config_node_reset_handler
(
    /* IN */    MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */    MS_NET_ADDR              saddr,
    /* IN */    MS_NET_ADDR              daddr,
    /* IN */    MS_SUBNET_HANDLE         subnet_handle,
    /* IN */    MS_APPKEY_HANDLE         appkey_handle,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
//    UINT32     response_opcode;
    CONFIG_TRC(
        "[CONFIG] Reset Node\n");
    /**
        If the node is not already provisioned,
        or if it is a Provisioner, Ignore (?)
    */
    /* TODO: Check if a provisioner */
    /* Remove friendship information */
    #ifdef MS_FRIEND_SUPPORT
    MS_trn_clear_all_lpn();
    #endif /* MS_FRIEND_SUPPORT */
    #ifdef MS_LPN_SUPPORT
    /* TODO: Check if it safe to call, as it will try to send message to Friend */
    MS_trn_lpn_clear_friendship();
    #endif /* MS_LPN_SUPPORT */
    /* Clear Lower Transport Rx and Tx Segmentation Queue. Stop timers. Free memory */
    MS_ltrn_clear_sar_contexts();
    /* Send reply - Ignoring response */
    *response_opcode = MS_ACCESS_CONFIG_NODE_RESET_STATUS_OPCODE;
    *response_buffer_len = 0;
    /* Respond with Node Reset Status message */
    MS_access_reply
    (
        handle,
        daddr,
        saddr,
        subnet_handle,
        appkey_handle,
        ACCESS_INVALID_DEFAULT_TTL,
        *response_opcode,
        NULL,
        0
    );
    /* TODO: Remove proxy white/black list */
    /* Reset Internal Data Structures */
    MS_access_cm_reset(PROV_ROLE_DEVICE);
    /* Return something other than API_SUCCESS, so that the response is not sent again */
    return API_FAILURE;
}

DECL_STATIC API_RESULT config_model_app_get_handler
(
    /* IN */    UINT32                   opcode,
    /* IN */    UCHAR*                   data_param,
    /* IN */    UINT16                   data_len,
    /* OUT */   UINT32*                  response_opcode,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_len
)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_ACCESS_MODEL_ID model_id;
    UINT16             model_id_16;
    MS_ACCESS_ELEMENT_HANDLE  elem_handle;
    MS_ACCESS_MODEL_HANDLE    model_handle;
    UINT16           marker;
    UINT16           appkey_count;
    /* TODO: Allocate memory */
    UINT16           appkey_index_list[20 /* MS_MAX_APPS */];
    /* TODO: Size */
    /* UINT8            appkey_index_list_pdu[1 + 2 + 4 + MS_MAX_APPS * 2]; */
    UINT8*           appkey_index_list_pdu;
    UINT16           appkey_index_list_pdu_length;
    MS_IGNORE_UNUSED_PARAM(opcode);
    /**
        Extract
        - ElementAddress (2 Octets)
        - ModelIdentifier (2 or 4 Octets)
    */
    appkey_index_list_pdu = response_buffer;

    if ((4 == data_len) || (6 == data_len))
    {
        /* ElementAddress */
        MS_UNPACK_LE_2_BYTE(&uaddr, data_param);

        /* Get Model ID type, based on the length */
        if (2 == (data_len - 2))
        {
            /* Model SIG ID - 2 Octets */
            MS_UNPACK_LE_2_BYTE(&model_id_16, (data_param + 2));
            model_id.id = model_id_16;
            model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
        }
        else
        {
            /* Model Vendor ID - 4 Octets */
            MS_UNPACK_LE_4_BYTE(&(model_id.id), (data_param + 2));
            model_id.type = MS_ACCESS_MODEL_TYPE_VENDOR;
        }

        CONFIG_TRC(
            "[CONFIG] Model App Get. ElementAddress 0x%04X\n", uaddr);
        CONFIG_TRC(
            "[CONFIG] Model Type: %s. Model ID 0x%08X\n",
            ((MS_ACCESS_MODEL_TYPE_SIG == model_id.type) ? "SIG" : "Vendor"), model_id.id);
        /* Search for the ElementAddress */
        retval = MS_access_get_element_handle
                 (
                     uaddr,
                     &elem_handle
                 );

        if (API_SUCCESS == retval)
        {
            CONFIG_TRC(
                "[CONFIG] ElementAddress 0x%04X -> Element Handle 0x%04X\n", uaddr, elem_handle);
            /* Search for Model instance */
            retval = MS_access_get_model_handle
                     (
                         elem_handle,
                         model_id,
                         &model_handle
                     );

            if (API_SUCCESS == retval)
            {
                CONFIG_TRC(
                    "[CONFIG] Element Handle 0x%04X and Model ID 0x%08X -> Model Handle 0x%04X\n",
                    elem_handle, model_id.id, model_handle);
                /* Get AppKey List */
                appkey_count = sizeof(appkey_index_list) / sizeof(UINT16);
                retval = MS_access_cm_get_model_app_list
                         (
                             model_handle,
                             &appkey_count,
                             appkey_index_list
                         );

                if (API_SUCCESS == retval)
                {
                    CONFIG_TRC(
                        "[CONFIG] Get Model App List successful.\n");
                    CONFIG_TRC(
                        "[CONFIG] AppKey List returned 0x%04X. Count 0x%04X\n", retval, appkey_count);
                }
                else
                {
                    CONFIG_ERR(
                        "[CONFIG] Get Model App List Failed. Retval 0x%04X\n", retval);

                    if (ACCESS_NO_MATCH == retval)
                    {
                        /* TODO: Check what should be correct return value */
                        /* retval = MS_INVALID_APPKEY_INDEX; */
                        retval = MS_SUCCESS;
                    }
                }
            }
            else
            {
                CONFIG_ERR(
                    "[CONFIG] Get Model Handle for Element Handle 0x%04X and Model ID 0x%08X Failed. Retval 0x%04X\n",
                    elem_handle, model_id.id, retval);
            }
        }
        else
        {
            CONFIG_ERR(
                "[CONFIG] Get Element Handle for ElementAddress 0x%04X Failed. Retval 0x%04X\n", uaddr, retval);
        }

        /* Associate/Bind AppKeyIndex with Model instance */

        /* Respond with Config Model App List message */
        if (MS_ACCESS_CONFIG_SIG_MODEL_APP_GET_OPCODE == opcode)
        {
            *response_opcode = MS_ACCESS_CONFIG_SIG_MODEL_APP_LIST_OPCODE;
        }
        else
        {
            *response_opcode = MS_ACCESS_CONFIG_VENDOR_MODEL_APP_LIST_OPCODE;
        }

        /* Frame Response */
        marker = 0;
        appkey_index_list_pdu[marker] = (UCHAR)retval;
        marker ++;
        /* Copy back the request message */
        EM_mem_copy(&appkey_index_list_pdu[marker], data_param, data_len);
        marker += data_len;

        if (API_SUCCESS == retval)
        {
            /* Pack the AppKey Indices */
            appkey_index_list_pdu_length = sizeof(appkey_index_list_pdu) - marker;
            /* TODO: Not checking retval */
            config_pack_key_indices
            (
                appkey_count,
                appkey_index_list,
                appkey_index_list_pdu + marker,
                &appkey_index_list_pdu_length
            );
            CONFIG_TRC(
                "[CONFIG] AppKey Indices after packing\n");
            CONFIG_debug_dump_bytes(appkey_index_list_pdu + marker, appkey_index_list_pdu_length);
            marker += appkey_index_list_pdu_length;
        }

        *response_buffer_len = marker;
        retval = API_SUCCESS;
    }
    else
    {
        /* TODO */
        retval = API_FAILURE;
    }

    return retval;
}


/* Callback Handler */
#define CONFIG_SERVER_OPCODE_HANDLER(handler) \
    retval = (handler) (opcode, data_param, data_len, &response_opcode, response_buffer, &response_buffer_len)

/*         (handler) (handle, saddr, daddr, subnet_handle, appkey_handle, opcode, data_param, data_len) */

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
API_RESULT config_server_cb
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
    /**
        TODO:
        - Send reply from this callback itself and not from individual opcode handlers, to reduce code size.
        - Pass the buffer to frame response from this function. Especial cases like get composition data
         allocate memory in this function and provide to the handlers.
        - Send reply based on the return valus from the individual handlers.
        - To the handlers only provide opcode, data_param, data_len and additionaly buffer to frame response
         and also an in/out parameter to fill the length of response parameter and also parameter to fill
         response opcode.
    */
    API_RESULT retval;
    UINT32     response_opcode;
    UINT8      response_buffer[80];
    UINT16     response_buffer_len;
    retval = API_SUCCESS;
    response_buffer_len = sizeof(response_buffer);
    CONFIG_TRC(
        "[CONFIG] Callback. Opcode 0x%04X\n", opcode);
//printf("[config_server_cb] Callback. Opcode 0x%04X\n", opcode);
    CONFIG_debug_dump_bytes(data_param, data_len);

    /* Check if the packet is decrypted using device key */
    if (MS_CONFIG_LIMITS(MS_MAX_APPS) > appkey_handle)
    {
        CONFIG_ERR("[CONFIG] Configuration Model Packets shall use Device Key. Dropping..\n");
        return API_FAILURE;
    }

    switch (opcode)
    {
    case MS_ACCESS_CONFIG_APPKEY_ADD_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_APPKEY_UPDATE_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_APPKEY_DELETE_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_appkey_add_update_delete_handler);
        break;

    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_SET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_publication_set_handler);
        break;

    case MS_ACCESS_CONFIG_APPKEY_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_appkey_get_handler);
        break;

    case MS_ACCESS_CONFIG_COMPOSITION_DATA_GET_OPCODE:
        #if 0
        CONFIG_SERVER_OPCODE_HANDLER(config_composition_data_get_handler);
        break;
        #else
        return config_composition_data_get_handler(handle, saddr, daddr, subnet_handle, appkey_handle, opcode, data_param, data_len);
        #endif /* */

    case MS_ACCESS_CONFIG_BEACON_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_BEACON_SET_OPCODE:
        retval = config_beacon_get_set_handler
                 (
                     subnet_handle,
                     opcode,
                     data_param,
                     data_len,
                     &response_opcode,
                     response_buffer,
                     &response_buffer_len
                 );
        break;

    case MS_ACCESS_CONFIG_DEFAULT_TTL_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_DEFAULT_TTL_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_default_ttl_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_FRIEND_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_FRIEND_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_friend_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_GATT_PROXY_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_GATT_PROXY_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_gatt_proxy_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_key_refresh_phase_get_handler);
        break;

    /* TODO: See if Key Refresh Get/Set can be combined */
    case MS_ACCESS_CONFIG_KEY_REFRESH_PHASE_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_key_refresh_phase_set_handler);
        break;

    case MS_ACCESS_CONFIG_MODEL_PUBLICATION_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_publication_get_handler);
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_ADD_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_subscription_add_handler);
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_subscription_delete_handler);
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_DELETE_ALL_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_subscription_delete_all_handler);
        break;

    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_OVERWRITE_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_subscription_overwrite_handler);
        break;

    case MS_ACCESS_CONFIG_NETWORK_TRANSMIT_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_NETWORK_TRANSMIT_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_network_transmit_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_RELAY_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_RELAY_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_relay_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_SIG_MODEL_SUBSCRIPTION_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_VENDOR_MODEL_SUBSCRIPTION_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_subscription_get_handler);
        break;

    case MS_ACCESS_CONFIG_LOW_POWER_NODE_POLLTIMEOUT_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_low_power_node_polltimeout_get_handler);
        break;

    /* TODO: Check if Heartbeat publication/subscription get/set can be combined */
    case MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_heartbeat_publication_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_HEARTBEAT_PUBLICATION_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_heartbeat_publication_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_heartbeat_subscription_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_HEARTBEAT_SUBSCRIPTION_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_heartbeat_subscription_get_set_handler);
        break;

    case MS_ACCESS_CONFIG_MODEL_APP_BIND_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_MODEL_APP_UNBIND_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_app_bind_unbind_handler);
        break;

    case MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_netkey_add_update_handler);
        break;

    case MS_ACCESS_CONFIG_NETKEY_DELETE_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_netkey_delete_handler);
        break;

    case MS_ACCESS_CONFIG_NETKEY_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_netkey_get_handler);
        break;

    /* TODO: See if Node Identity Get/Set handlers can be combined */
    case MS_ACCESS_CONFIG_NODE_IDENTITY_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_node_identity_get_handler);
        break;

    case MS_ACCESS_CONFIG_NODE_IDENTITY_SET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_node_identity_set_handler);
        break;

    case MS_ACCESS_CONFIG_NODE_RESET_OPCODE:
        retval = config_node_reset_handler
                 (
                     handle,
                     saddr,
                     daddr,
                     subnet_handle,
                     appkey_handle,
                     &response_opcode,
                     response_buffer,
                     &response_buffer_len
                 );

        if(NULL != app_config_server_callback)
        {
            app_config_server_callback
            (
                handle,
                saddr,
                daddr,
                subnet_handle,
                appkey_handle,
                opcode,
                data_param,
                data_len,
                retval,
                response_opcode,
                response_buffer,
                response_buffer_len
            );
        }

        return retval;

//        break;

    case MS_ACCESS_CONFIG_SIG_MODEL_APP_GET_OPCODE: /* Fall Through */
    case MS_ACCESS_CONFIG_VENDOR_MODEL_APP_GET_OPCODE:
        CONFIG_SERVER_OPCODE_HANDLER(config_model_app_get_handler);
        break;

    default:
        break;
    }

    if(NULL != app_config_server_callback)
    {
        app_config_server_callback
        (
            handle,
            saddr,
            daddr,
            subnet_handle,
            appkey_handle,
            opcode,
            data_param,
            data_len,
            retval,
            response_opcode,
            response_buffer,
            response_buffer_len
        );
    }

    /* Based on the return value from the handler, reply */
    if (API_SUCCESS == retval)
    {
        retval = MS_access_reply
                 (
                     handle,
                     daddr,
                     saddr,
                     subnet_handle,
                     appkey_handle,
                     ACCESS_INVALID_DEFAULT_TTL,
                     response_opcode,
                     response_buffer,
                     response_buffer_len
                 );
    }

    return retval;
}

#endif /* MS_MODEL_CONFIG */

