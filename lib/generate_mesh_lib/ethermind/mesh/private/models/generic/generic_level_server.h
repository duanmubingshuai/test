/**
    \file generic_level_server.h
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_GENERIC_LEVEL_SERVER_
#define _H_GENERIC_LEVEL_SERVER_


/* --------------------------------------------- Header File Inclusion */
#include "MS_access_api.h"
#include "MS_generic_level_api.h"


/* --------------------------------------------- Global Definitions */
#ifdef  GENERIC_LEVEL_SERVER_NO_DEBUG
    #define GENERIC_LEVEL_SERVER_ERR         EM_debug_null
#else /* GENERIC_LEVEL_SERVER_NO_DEBUG */
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define GENERIC_LEVEL_SERVER_ERR
    #else
        #define GENERIC_LEVEL_SERVER_ERR(...)     EM_debug_error(MS_MODULE_ID_MESH_MODEL, __VA_ARGS__)
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* GENERIC_LEVEL_SERVER_NO_DEBUG */
#ifdef GENERIC_LEVEL_SERVER_DEBUG
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define GENERIC_LEVEL_SERVER_TRC
        #define GENERIC_LEVEL_SERVER_INF
        #define GENERIC_LEVEL_SERVER_debug_dump_bytes(data, datalen)
    #else
        #define GENERIC_LEVEL_SERVER_TRC(...)     EM_debug_trace(MS_MODULE_ID_MESH_MODEL,__VA_ARGS__)
        #define GENERIC_LEVEL_SERVER_INF(...)     EM_debug_info(MS_MODULE_ID_MESH_MODEL,__VA_ARGS__)
        #define GENERIC_LEVEL_SERVER_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_MESH_MODEL, (data), (datalen))
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* GENERIC_LEVEL_SERVER_DEBUG */
    #define GENERIC_LEVEL_SERVER_TRC          EM_debug_null
    #define GENERIC_LEVEL_SERVER_INF          EM_debug_null
    #define GENERIC_LEVEL_SERVER_debug_dump_bytes(data, datalen)
#endif /* GENERIC_LEVEL_SERVER_DEBUG */



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
API_RESULT generic_level_server_cb
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
    \brief Access Layer Model Publication Timeout Callback.

    \par Description
    Access Layer calls the registered callback to indicate Publication Timeout
    for the associated model.

    \param [in]  handle        Model Handle.
    \param [out] blob          Blob if any or NULL.
*/
API_RESULT generic_level_server_publish_timout_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
);

#endif /*_H_GENERIC_LEVEL_SERVER_ */
