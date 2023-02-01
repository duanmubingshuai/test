
/**
    \file config_internal.h

    \brief Internal Header File for the Configuration Model
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_CONFIG_INTERNAL_API_
#define _H_CONFIG_INTERNAL_API_


/* --------------------------------------------- Header File Inclusion */
/* Access Layer */
#include "MS_access_api.h"
#include "MS_config_api.h"

/* --------------------------------------------- Global Definitions */
#ifdef CONFIG_NO_DEBUG
    #define CONFIG_ERR          EM_debug_null
#else /* CONFIG_NO_DEBUG */
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define CONFIG_ERR
    #else
        #define CONFIG_ERR(...)     EM_debug_error(MS_MODULE_ID_CONFIG, __VA_ARGS__)
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* CONFIG_NO_DEBUG */

#ifdef CONFIG_DEBUG
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define CONFIG_TRC
        #define CONFIG_INF

        #define CONFIG_debug_dump_bytes(data, datalen)

    #else
        #define CONFIG_TRC(...)     EM_debug_trace(MS_MODULE_ID_CONFIG,__VA_ARGS__)
        #define CONFIG_INF(...)     EM_debug_info(MS_MODULE_ID_CONFIG,__VA_ARGS__)

        #define CONFIG_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_CONFIG, (data), (datalen))

    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* CONFIG_DEBUG */
    #define CONFIG_TRC          EM_debug_null
    #define CONFIG_INF          EM_debug_null

    #define CONFIG_debug_dump_bytes(data, datalen)

#endif /* CONFIG_DEBUG */


/* --------------------------------------------- Data Types/ Structures */


/* --------------------------------------------- Function */
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
);

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
);

#endif /* _H_CONFIG_INTERNAL_API_ */

