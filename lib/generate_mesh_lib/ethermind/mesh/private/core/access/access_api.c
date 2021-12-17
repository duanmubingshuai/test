/**
    \file access_api.c

    This file defines the Transport Layer Application Interface Methods
*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "access_internal.h"
#include "access_extern.h"

#include "MS_access_api.h"

#ifdef MS_ACCESS

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */

/**
    \brief Create a new node in the device.

    \par Description
    This routine creates a new node in the device. This can be used by the
    application to create extra nodes if required in addition to the default
    primary node.

    \param [out] node_id Identifier to reference the newly created node.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_create_node(/* OUT */ MS_ACCESS_NODE_ID* node_id)
{
    /* TBD: Right now using dummy */
    *node_id = (MS_ACCESS_NODE_ID)MS_ACCESS_DEFAULT_NODE_ID;
    return API_SUCCESS;
}

/**
    \brief Register an element with the access layer.

    \par Description
    This routine registers an element that can be populated with the models
    information to a specific node in the device identified by the node id.

    \param [in] node_id Node to which the element needs to be registered. This
    value is always 0 for the default node.

    \param [in] element Pointer to the element descriptor that needs to be
    registered to the node.

    \param [out] element_handle Identifier to reference the newly registered element.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_register_element
(
    /* IN */  MS_ACCESS_NODE_ID            node_id,
    /* IN */  MS_ACCESS_ELEMENT_DESC*      element,
    /* OUT */ MS_ACCESS_ELEMENT_HANDLE*    element_handle
)
{
    API_RESULT retval;
    MS_ACCESS_ELEMENT_HANDLE id;
    /* Ignore 'node_id' */
    MS_IGNORE_UNUSED_PARAM(node_id);
    /* Allocate a space for element from the list */
    retval = ms_access_allocate_element(&id);

    if (API_SUCCESS == retval)
    {
        /* Save 'element' */
        ms_access_element_list[id].loc = element->loc;
        /* Return 'element' index as Handle */
        *element_handle = id;
        /* Increment element count of local node */
        ms_element_addr_table[0].element_count ++;
    }

    return retval;
}

/**
    \brief Register a model with the access layer.

    \par Description
    This routine registers a model associated with an element with the access layer.

    \param [in] node_id Node to which the model needs to be registered. This
    value is always 0 for the default node.

    \param [in] model Pointer to the model descriptor that needs to be
    registered to the node.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful registration.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_register_model
(
    /* IN */     MS_ACCESS_NODE_ID         node_id,
    /* IN */     MS_ACCESS_MODEL*          model,
    /* INOUT */  MS_ACCESS_MODEL_HANDLE*   model_handle
)
{
    API_RESULT retval;
    UINT16     index;
    /* TBD: Validate parameters */
    /* TBD: Lock */
    /* Ignore 'node_id' */
    MS_IGNORE_UNUSED_PARAM(node_id);
    /* Search and allocate a space for model from the list */
    retval = ms_access_search_and_allocate_model(model, &index);

    /* Check Status */
    if (API_SUCCESS == retval)
    {
        /* Save Model Specific Information */
        ms_access_model_list[index].fixed.cb = model->cb;
        ms_access_model_list[index].fixed.opcodes = model->opcodes;
        ms_access_model_list[index].fixed.num_opcodes = model->num_opcodes;
        MS_ACCESS_SET_PUBLICATION_CALLBACK(ms_access_model_list[index], model);
        ms_access_model_list[index].fixed.elem_handle = model->elem_handle;
        ms_access_model_list[index].fixed.model_id = model->model_id;

        /* Update associated element */
        /* TBD: Validate Element ID */
        if (MS_ACCESS_MODEL_TYPE_SIG == model->model_id.type)
        {
            ms_access_element_list[model->elem_handle].num_s++;
        }
        else
        {
            ms_access_element_list[model->elem_handle].num_v++;
        }

        /* Assign Model Handle */
        *model_handle = (MS_ACCESS_MODEL_HANDLE)index;
    }
    else if (ACCESS_MODEL_ALREADY_REGISTERED == retval)
    {
        ACCESS_TRC(
            "[ACCESS]: Model ID: 0x%08X, Type: 0x%02X is already registerd for Element Hdl: 0x%04X.\n",
            model->model_id.id, model->model_id.type, model->elem_handle);
        /* Returning API Success */
        retval = API_SUCCESS;
    }

    /* TBD: Unlock */
    return retval;
}

/**
    \brief Get element handle.

    \par Description
    This routine searches for the element handle associated with specific element address.

    \param [in]  elem_addr Address of the corresponding element.
    \param [out] handle    Element handle associated with the element address.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_get_element_handle
(
    /* IN */   MS_NET_ADDR                 elem_addr,
    /* OUT */  MS_ACCESS_ELEMENT_HANDLE*   handle
)
{
    API_RESULT   retval;
    MS_NET_ADDR  start_addr;
    UINT8        element_count;
    ACCESS_TRC(
        "[ACCESS] Get Element Handle for address 0x%04X\n", elem_addr);
    /* Element Start Address and Element Count */
    start_addr = ms_element_addr_table[0].uaddr;
    element_count = ms_element_addr_table[0].element_count;

    /* Check if the element address is in range */
    if ((elem_addr >= start_addr) &&
            (elem_addr < (start_addr + element_count)))
    {
        *handle = (MS_ACCESS_ELEMENT_HANDLE)(elem_addr - start_addr);
        retval = API_SUCCESS;
        ACCESS_TRC(
            "[ACCESS] Found Element Handle 0x%04X\n", (*handle));
    }
    else
    {
        retval = MS_INVALID_ADDRESS;
        ACCESS_ERR(
            "[ACCESS] Failed to find Element Handle\n");
    }

    return retval;
}

/**
    \brief Get element handle.

    \par Description
    This routine searches for the element handle associated with specific element address.

    \param [in]  elem_addr Address of the corresponding element.
    \param [out] handle    Element handle associated with the element address.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_get_appkey_handle
(
    /* IN */   MS_ACCESS_MODEL_HANDLE*         handle,
    /* OUT */  MS_APPKEY_HANDLE*               appkey_handle
)
{
    API_RESULT   retval = API_SUCCESS;
    /* Get associated application key */
    *appkey_handle = ms_access_model_list[*handle].fixed.publish.appkey_handle;
    return retval;
}

/**
    \brief Get model handle.

    \par Description
    This routine searches for the model handle associated with specific model ID.

    \param [in]  elem_handle Element Identifier associated with the Model.
    \param [in]  model_id    Model Identifier for which the model handle to be searched.
    \param [out] handle      Model handle associated with model ID.
    If not found, handle will be set as NULL.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_get_model_handle
(
    /* IN */   MS_ACCESS_ELEMENT_HANDLE      elem_handle,
    /* IN */   MS_ACCESS_MODEL_ID            model_id,
    /* OUT */  MS_ACCESS_MODEL_HANDLE*       handle
)
{
    API_RESULT retval;
    UINT32     index;
    retval = API_SUCCESS;

    /* Search for model match */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); index++)
    {
        /* Check if contains valid model */
        if ((1 == ms_access_model_list[index].fixed.valid) &&
                (ms_access_model_list[index].fixed.elem_handle == elem_handle) &&
                (ms_access_model_list[index].fixed.model_id.type == model_id.type) &&
                (ms_access_model_list[index].fixed.model_id.id == model_id.id))
        {
            /* Return model handle */
            *handle = (MS_ACCESS_MODEL_HANDLE)index;
            break;
        }
    }

    /* Check if found a match */
    if (MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT) == index)
    {
        retval = MS_INVALID_MODEL;
    }

    return retval;
}

/**
    \brief API to publish access layer message.

    \par Description
    This routine publishes Access Layer message to the publish address associated with the model.

    \param [in] handle
           Access Model Handle for which message to be sent.

    \param [in] opcode
           Access Opcode

    \param [in] data_param
           Data packet

    \param [in] data_len
           Data packet length

    \param [in] reliable
           MS_TRUE for reliable message. MS_FALSE otherwise.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_publish
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ UINT32                    opcode,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_len,
    /* IN */ UINT8                     reliable
)
{
    API_RESULT     retval;
    MS_NET_ADDR    dst_addr;
    ACCESS_TRC(
        "[ACCESS] Publish. Opcode: 0x%08X. Param Len: 0x%04X. Reliable: %s\n",
        opcode, data_len, ((MS_TRUE == reliable)?"True":"False"));
    /* TODO: Validate parameters */
    /* Get Model Publication Address and check if valid */
    retval = MS_access_get_publish_addr
             (
                 handle,
                 &dst_addr
             );

    if (API_SUCCESS != retval)
    {
        ACCESS_ERR(
            "[ACCESS] Model Publication is not enabled. Returning.\n");
        return retval;
    }

    retval = MS_access_publish_ex
             (
                 handle,
                 opcode,
                 dst_addr,
                 data_param,
                 data_len,
                 reliable
             );
    return retval;
}

/**
    \brief API to reliably publish access layer message.

    \par Description
    This routine reliably publishes Access Layer message to the publish address associated with the model.

    \param [in] handle
           Access Model Handle for which message to be sent.

    \param [in] req_opcode
           Request Opcode

    \param [in] data_param
           Data packet

    \param [in] data_len
           Data packet length

    \param [in] rsp_opcode
           Response Opcode

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_reliable_publish
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ UINT32                    req_opcode,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_len,
    /* IN */ UINT32                    rsp_opcode
)
{
    API_RESULT     retval;
    MS_NET_ADDR    dst_addr;
    ACCESS_TRC(
        "[ACCESS] Reliable Publish. Req Opcode: 0x%08X. Param Len: 0x%04X. Rsp Opcode: 0x%08X\n",
        req_opcode, data_len, rsp_opcode);
    /* TODO: Validate parameters */
    /* Get Model Publication Address and check if valid */
    retval = MS_access_get_publish_addr
             (
                 handle,
                 &dst_addr
             );

    if (API_SUCCESS != retval)
    {
        ACCESS_ERR(
            "[ACCESS] Model Publication is not enabled. Returning.\n");
        return retval;
    }

    retval = MS_access_publish_ex
             (
                 handle,
                 req_opcode,
                 dst_addr,
                 data_param,
                 data_len,
                 MS_TRUE
             );
    return retval;
}

/** Get Publish Address */
API_RESULT MS_access_get_publish_addr
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ MS_NET_ADDR*              publish_addr
)
{
    API_RESULT retval;

    /* TODO: Validate parameters */

    /* Check if Model Publication State is valid */
    if (0x01 != ms_access_model_list[*handle].fixed.publish.valid)
    {
        ACCESS_TRC(
            "[ACCESS 0x%04X] Invalid Publication State. Returning ...\n",
            (*handle));
        return ACCESS_INVALID_PUBLICATION_STATE;
    }

    /* Get publish address */
    retval = ms_get_address
             (
                 ms_access_model_list[*handle].fixed.publish.addr_handle,
                 MS_ADDR_OP_TYPE_PUBLISH,
                 publish_addr
             );
    return retval;
}

/** Publish */
API_RESULT MS_access_publish_ex
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ UINT32                    opcode,
    /* IN */ MS_NET_ADDR               dst_addr,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_len,
    /* IN */ UINT8                     reliable
)
{
    API_RESULT retval;
    UINT8      ttl;
    MS_NET_ADDR               src_addr;
    MS_SUBNET_HANDLE          subnet_handle;
    MS_APPKEY_HANDLE          appkey_handle;
    ACCESS_TRC(
        "[ACCESS] ms_access_publish_ex. Opcode: 0x%08X. DST Addr: 0x%04X. Param Len: 0x%04X. Reliable: %s\n",
        opcode, dst_addr, data_len, ((MS_TRUE == reliable)?"True":"False"));
    /* TODO: Validate parameters */
    /**
        Get primary address and calculate the associated element address.

        For the element handle 'N', element address will be (primary address + 'N'),
        as element handle starts from 0.
    */
    src_addr = ms_element_addr_table[0].uaddr + ms_access_model_list[*handle].fixed.elem_handle;
    /* Get associated application key */
    appkey_handle = ms_access_model_list[*handle].fixed.publish.appkey_handle;

    if (MS_TRUE == ms_access_model_list[*handle].fixed.publish.crden_flag)
    {
        subnet_handle = MS_CONFIG_LIMITS(MS_MAX_SUBNETS);
    }
    else
    {
        /* Get associated subnet */
        if (MS_CONFIG_LIMITS(MS_MAX_APPS) <= appkey_handle)
        {
            /* TODO: For device key, using primary network key */
            subnet_handle = 0;
        }
        else
        {
            subnet_handle = ms_appkey_table[appkey_handle].subnet_handle;
        }
    }

    /* Get associated ttl */
    /* TODO: Use publish ttl */
    ttl = ms_access_model_list[*handle].fixed.publish.ttl;
    ACCESS_CM_GET_DEFAULT_TTL(ttl);
    retval = MS_access_send_pdu
             (
                 src_addr,
                 dst_addr,
                 subnet_handle,
                 appkey_handle,
                 ttl,
                 opcode,
                 data_param,
                 data_len,
                 reliable
             );
    return retval;
}

/** Publish */
API_RESULT MS_access_raw_data
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ UINT32                    opcode,
    /* IN */ MS_NET_ADDR               dst_addr,
    /* IN */ MS_APPKEY_HANDLE        appKeyHandle,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_len,
    /* IN */ UINT8                     reliable
)
{
    API_RESULT retval;
    UINT8      ttl;
    MS_NET_ADDR               src_addr;
    MS_SUBNET_HANDLE          subnet_handle;
    MS_APPKEY_HANDLE          appkey_handle;
    ACCESS_TRC(
        "[ACCESS] ms_access_publish_ex. Opcode: 0x%08X. DST Addr: 0x%04X. Param Len: 0x%04X. Reliable: %s\n",
        opcode, dst_addr, data_len, ((MS_TRUE == reliable)?"True":"False"));
    /* TODO: Validate parameters */
    /**
        Get primary address and calculate the associated element address.

        For the element handle 'N', element address will be (primary address + 'N'),
        as element handle starts from 0.
    */
    src_addr = ms_element_addr_table[0].uaddr + ms_access_model_list[*handle].fixed.elem_handle;
    /* Get associated application key */
    appkey_handle = appKeyHandle;
    subnet_handle = ms_appkey_table[appkey_handle].subnet_handle;
    /* Set associated ttl */
    ACCESS_CM_GET_DEFAULT_TTL(ttl);
    retval = MS_access_send_pdu
             (
                 src_addr,
                 dst_addr,
                 subnet_handle,
                 appkey_handle,
                 ttl,
                 opcode,
                 data_param,
                 data_len,
                 reliable
             );
    return retval;
}

/**
    \brief API to reply to access layer message and optionally also to publish.

    \par Description
    This routine replies to Access Layer message and also publish if requested by application.

    \param [in] handle        Model Handle.
    \param [in] saddr         16 bit Source Address.
    \param [in] daddr         16 bit Destination Address.
    \param [in] subnet_handle Subnet Handle.
    \param [in] appkey_handle AppKey Handle.
    \param [in] ttl           Time to Live.
    \param [in] opcode        Access Opcode
    \param [in] data_param    Access parameter, based on the opcode
    \param [in] data_length   Access parameter length, based on the opcode
    \param [in] to_publish    Flag to indicate if the message also to be published

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_reply_and_publish
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ MS_NET_ADDR               saddr,
    /* IN */ MS_NET_ADDR               daddr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_APPKEY_HANDLE          appkey_handle,
    /* IN */ UINT8                     ttl,
    /* IN */ UINT32                    opcode,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_len,
    /* IN */ UINT8                     to_publish
)
{
    API_RESULT  retval;
    MS_IGNORE_UNUSED_PARAM(handle);

    if (MS_TRUE == to_publish)
    {
        retval = MS_access_reply
                 (
                     handle,
                     saddr,
                     daddr,
                     subnet_handle,
                     appkey_handle,
                     ttl,
                     opcode,
                     data_param,
                     data_len
                 );
    }
    else
    {
        /**
            Reply is sent for request received for an unicast address.

            Ignoring associated model handle.
            Expecting the source address is populated by the caller.

            If the TTL value is filled with Invalid TTL, replace with
            default TTL value.

            Subnet Handle has to be set by the caller.
            If the AppKey Handle is set as Invalid, that will inform
            Transport layer to use Device Key, in place of AppKey.
        */
        if (ACCESS_INVALID_DEFAULT_TTL == ttl)
        {
            ACCESS_CM_GET_DEFAULT_TTL(ttl);
        }

        retval = MS_access_send_pdu
                 (
                     saddr,
                     daddr,
                     subnet_handle,
                     appkey_handle,
                     ttl,
                     opcode,
                     data_param,
                     data_len,
                     MS_TRUE
                 );
    }

    return retval;
}

/**
    \brief API to reply to access layer message.

    \par Description
    This routine replies to Access Layer message.

    \param [in] handle        Model Handle.
    \param [in] saddr         16 bit Source Address.
    \param [in] daddr         16 bit Destination Address.
    \param [in] subnet_handle Subnet Handle.
    \param [in] appkey_handle AppKey Handle.
    \param [in] ttl           Time to Live.
    \param [in] opcode        Access Opcode
    \param [in] data_param    Access parameter, based on the opcode
    \param [in] data_length   Access parameter length, based on the opcode

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_reply
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*   handle,
    /* IN */ MS_NET_ADDR               saddr,
    /* IN */ MS_NET_ADDR               daddr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_APPKEY_HANDLE          appkey_handle,
    /* IN */ UINT8                     ttl,
    /* IN */ UINT32                    opcode,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_len
)
{
    API_RESULT  retval;
    MS_NET_ADDR pub_addr;
    MS_IGNORE_UNUSED_PARAM(handle);

    /**
        Reply is sent for request received for an unicast address.

        Ignoring associated model handle.
        Expecting the source address is populated by the caller.

        If the TTL value is filled with Invalid TTL, replace with
        default TTL value.

        Subnet Handle has to be set by the caller.
        If the AppKey Handle is set as Invalid, that will inform
        Transport layer to use Device Key, in place of AppKey.
    */

    if (ACCESS_INVALID_DEFAULT_TTL == ttl)
    {
        ACCESS_CM_GET_DEFAULT_TTL(ttl);
    }

    /* Also publish this */
    retval = MS_access_get_publish_addr
             (
                 handle,
                 &pub_addr
             );

    /* Check if Publish Address is valid and not same as the Destination Address */
    if ((API_SUCCESS == retval) && (pub_addr != daddr))
    {
        MS_access_publish_ex
        (
            handle,
            opcode,
            pub_addr,
            data_param,
            data_len,
            MS_FALSE
        );
    }

    retval = MS_access_send_pdu
             (
                 saddr,
                 daddr,
                 subnet_handle,
                 appkey_handle,
                 ttl,
                 opcode,
                 data_param,
                 data_len,
                 MS_FALSE
             );
    return retval;
}


/**
    \brief API to send Access PDUs

    \par Description
    This routine sends transport PDUs to peer device.

    \param [in] dst_addr
           Destination Address

    \param [in] opcode
           Access Opcode

    \param [in] ttl
           Time to Live

    \param [in] key_idx
           Index of AppKey/DevKey to use for the PDU

    \param [in] param
           Access parameter, based on the opcode

    \param [in] reliable
           If requires lower transport Ack, set reliable as TRUE

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_send_pdu
(
    /* IN */ MS_NET_ADDR               src_addr,
    /* IN */ MS_NET_ADDR               dst_addr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_APPKEY_HANDLE          appkey_handle,
    /* IN */ UINT8                     ttl,
    /* IN */ UINT32                    opcode,
    /* IN */ UCHAR*                    data_param,
    /* IN */ UINT16                    data_length,
    /* IN */ UINT8                     reliable
)
{
    API_RESULT retval;
    UCHAR pdu[MS_ACCESS_MAX_PKT_SIZE];
    UINT16 marker;
    MS_BUFFER buffer;
    /* Source Address Types */
    UINT8             saddr_type;
    /* Parameter Validation */
    /* Check Source Address Type */
    /* Validate Source and Destination Address */
    saddr_type = MS_net_get_address_type(src_addr);
    ACCESS_INF("[ACCESS Tx] SRC 0x%04X Type: 0x%02X\n", src_addr, saddr_type);

    /* Source address shall be a Unicast Address. */
    if (MS_NET_ADDR_TYPE_UNICAST != saddr_type)
    {
        ACCESS_ERR("[ACCESS Tx] SRC ADDR 0x%04X not a Unicast Address. Returning...\n",
                   src_addr);
        return ACCESS_INVALID_SRC_ADDR;
    }

    ACCESS_TRC("[ACCESS] Send PDU. Opcode 0x%08X\n", opcode);

    /* Check Validity of Opcode */
    if (
        /* No bits in 4th octet is set */
        (0x00000000 != (0xFF000000 & opcode)) ||
        /* One octet opcode can not be 0x7F */
        (0x7F == opcode)
    )
    {
        ACCESS_ERR(
            "[ACCESS] Check FAILED for Opcode 0x%08X\n", opcode);
        return ACCESS_PARAMETER_OUTSIDE_RANGE;
    }

    /* Initialize marker */
    marker = 0;

    /* If 3rd octet has any bits set, most significant two bits are '11' */
    if (0x00000000 != (0x00FF0000 & opcode))
    {
        if (0x00C00000 != (0x00C00000 & opcode))
        {
            ACCESS_ERR(
                "[ACCESS] Check FAILED for 3 Octet Opcode 0x%08X\n", opcode);
            return ACCESS_PARAMETER_OUTSIDE_RANGE;
        }
        else
        {
            pdu[marker] = (opcode>>16)&0xff;
            marker++;
            /* Pack Opcode in Big Endian Format */
            MS_PACK_BE_2_BYTE_VAL(&pdu[marker], opcode);
            marker += 2;
        }
    }
    /* If 2rd octet has any bits set, most significant two bits are '10' */
    else if (0x00000000 != (0x0000FF00 & opcode))
    {
        if (0x00008000 != (0x00008000 & opcode))
        {
            ACCESS_ERR(
                "[ACCESS] Check FAILED for 2 Octet Opcode 0x%08X\n", opcode);
            return ACCESS_PARAMETER_OUTSIDE_RANGE;
        }
        else
        {
            /* Pack Opcode in Big Endian Format */
            MS_PACK_BE_2_BYTE_VAL(&pdu[marker], opcode);
            marker += 2;
        }
    }
    else
    {
        pdu[marker] = (UCHAR)opcode;
        marker++;
    }

    /*
        TODO: Add parameter length validation

        The transport layer provides a mechanism of SAR capable of transporting
        up to 32 segments.
        The maximum message size when using the SAR is 384 octets.
        This means (excluding an Application MIC)
        - up to 379 octets are available for parameters, when using a 1-octet opcode.
        - up to 378 octets are available for parameters, when using a 2-octet opcode.
        - up to 377 octets are available for parameters, when using a vendor-specific 3-octet opcode.

        Also consider 64-bit TransMIC.
    */

    /* TODO: Check if the TransMIC is 32-bit or 64-bit */
    if ((marker + data_length + 4) > MS_ACCESS_MAX_PKT_SIZE)
    {
        ACCESS_ERR(
            "[ACCESS] Length Check FAILED for Opcode 0x%08X, with Param Len: %d\n",
            opcode, data_length);
        return ACCESS_PARAMETER_OUTSIDE_RANGE;
    }

    /* Copy payload */
    EM_mem_copy (&pdu[marker], data_param, data_length);
    marker += data_length;
    buffer.payload = pdu;
    buffer.length = marker;
    /* Send PDU */
    retval = MS_trn_send_access_pdu
             (
                 src_addr,
                 dst_addr,
                 NULL,
                 subnet_handle,
                 appkey_handle,
                 ttl,
                 &buffer,
                 reliable
             );
    return retval;
}

API_RESULT MS_access_get_composition_data(/* OUT */ MS_BUFFER* buffer)
{
    UCHAR*   comp_data;
    UINT16   marker;
    UINT32   index;
    UINT32* model_indices;
    UINT32   model_count;

    if ((NULL == buffer) || (0 == buffer->length) || (NULL == buffer->payload))
    {
        ACCESS_ERR(
            "[ACCESS] Invalid Paramter\n");
        return ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }

    model_count = MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT);
    /* Allocate Memory for Model Indices */
    model_indices = (UINT32*)(EM_alloc_mem(sizeof(UINT32) * model_count));

    /* Check if memory is allocated */
    if (NULL == model_indices)
    {
        ACCESS_ERR(
            "[ACCESS] Failed to allocate memory for model indices\n");
        return ACCESS_MEMORY_ALLOCATION_FAILED;
    }

    /* Frame composition data */
    comp_data = buffer->payload;
    marker = 0;
    /* Page 0 */
    comp_data[marker] = 0x00;
    marker++;
    /* CID */
    MS_PACK_LE_2_BYTE_VAL(&comp_data[marker], composition_data_hdr.ms_access_cid);
    marker += 2;
    /* PID */
    MS_PACK_LE_2_BYTE_VAL(&comp_data[marker], composition_data_hdr.ms_access_pid);
    marker += 2;
    /* VID */
    MS_PACK_LE_2_BYTE_VAL(&comp_data[marker], composition_data_hdr.ms_access_vid);
    marker += 2;
    /* CRPL */
    MS_PACK_LE_2_BYTE_VAL(&comp_data[marker], composition_data_hdr.ms_access_crpl);
    marker += 2;
    /* Features */
    MS_PACK_LE_2_BYTE_VAL(&comp_data[marker], composition_data_hdr.ms_access_features);
    marker += 2;

    /* Elements */
    /* Scan through the element list */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT); index++)
    {
        /* Check if valid */
        if (0 != ms_access_element_list[index].valid)
        {
            /* Get all the models associated with this element */
            model_count = MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT);
            ms_access_get_element_models (index, model_indices, &model_count);

            /* Check if there are associated models */
            if (0 != model_count)
            {
                /* Model Index */
                UINT32 mi;
                /* LOC */
                MS_PACK_LE_2_BYTE_VAL(&comp_data[marker], ms_access_element_list[index].loc);
                marker += 2;
                /* NumS */
                MS_PACK_LE_1_BYTE_VAL(&comp_data[marker], ms_access_element_list[index].num_s);
                marker ++;
                /* NumV */
                MS_PACK_LE_1_BYTE_VAL(&comp_data[marker], ms_access_element_list[index].num_v);
                marker++;

                /* TBD: Optimize */
                /* Iterate over all associated models - to first fill SIG IDs - 16 bits */
                for (mi = 0; mi < model_count; mi ++)
                {
                    UINT16 sig_id;

                    /* SIG Model IDs */
                    if (MS_ACCESS_MODEL_TYPE_SIG == ms_access_model_list[model_indices[mi]].fixed.model_id.type)
                    {
                        sig_id = (UINT16)ms_access_model_list[model_indices[mi]].fixed.model_id.id;
                        MS_PACK_LE_2_BYTE_VAL(&comp_data[marker], sig_id);
                        marker += 2;
                    }
                }

                /* Iterate now over all associated models - to fill Vendor IDs - 32 bits */
                for (mi = 0; mi < model_count; mi++)
                {
                    /* Vendor Model IDs */
                    if (MS_ACCESS_MODEL_TYPE_VENDOR == ms_access_model_list[model_indices[mi]].fixed.model_id.type)
                    {
                        MS_PACK_LE_4_BYTE_VAL(&comp_data[marker], ms_access_model_list[model_indices[mi]].fixed.model_id.id);
                        marker += 4;
                    }
                }
            }
        }
    }

    /* Free allocated memory */
    EM_free_mem(model_indices);
    /* Assign to out parameter 'buffer' */
    buffer->length  = marker;
    /* Return */
    return API_SUCCESS;
}

#endif /* MS_ACCESS */

