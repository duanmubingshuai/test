
/**
    \file brr_internal.h

    Module Internal Header File contains structure definitions including tables
    maintained by the module
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_BRR_INTERNAL_
#define _H_BRR_INTERNAL_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_brr_api.h"

/* --------------------------------------------- Global Definitions */

#ifdef BRR_NO_DEBUG
    #define BRR_ERR          EM_debug_null
#else /* BRR_NO_DEBUG */
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define BRR_ERR
    #else
        #define BRR_ERR(...)     EM_debug_error(MS_MODULE_ID_BRR, __VA_ARGS__)
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* BRR_NO_DEBUG */

#ifdef BRR_DEBUG
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define BRR_TRC
        #define BRR_INF

        #define BRR_debug_dump_bytes(data, datalen)

    #else
        #define BRR_TRC(...)     EM_debug_trace(MS_MODULE_ID_BRR,__VA_ARGS__)
        #define BRR_INF(...)     EM_debug_info(MS_MODULE_ID_BRR,__VA_ARGS__)

        #define BRR_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_BRR, (data), (datalen))

    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* BRR_DEBUG */
    #define BRR_TRC          EM_debug_null
    #define BRR_INF          EM_debug_null

    #define BRR_debug_dump_bytes(data, datalen)

#endif /* BRR_DEBUG */

/**
    Locks the BRR Mutex which prevents any global variable being
    overwritten by any function. It returns an error if mutex lock fails.
*/
#define BRR_LOCK()\
    MS_MUTEX_LOCK(brr_mutex, BRR)

/**
    Locks the BRR_mutex which prevents any global variable being
    overwritten by any function. To be used in void function as it
    returns no error.
*/
#define BRR_LOCK_VOID()\
    MS_MUTEX_LOCK_VOID(brr_mutex, BRR)

/**
    Unlocks the BRR_mutex which realeses the global variables
    to be written into. It returns an error if mutex unlock fails.
*/
#define BRR_UNLOCK()\
    MS_MUTEX_UNLOCK(brr_mutex, BRR)

/**
    Unlocks the BRR_mutex which realeses the global variables
    to be written into. To be used in void functions as it returns
    no error.
*/
#define BRR_UNLOCK_VOID()\
    MS_MUTEX_UNLOCK_VOID(brr_mutex, BRR)


#define brr_alloc_mem(size)\
    EM_alloc_mem(size)

#define brr_free_mem(ptr)\
    EM_free_mem(ptr)

/* --------------------------------------------- Data Types/ Structures */
/* Data structure for the internal bearer table */
typedef struct _BRR_BEARER_DATA
{
    /* Bearer Information */
    BRR_BEARER_INFO info;

    /* TBD: Shall these GATT Bearer Specific paramters be protected? */
    /* Reassembly buffer for GATT */
    UCHAR gatt_rx_pdu[BRR_MAX_PDU_SIZE];

    /**
        TODO: Check if we can have a single buffer for gatt_tx_pdu[]
        and not having this as part of each context
    */
    /* Transmit buffer for GATT */
    UCHAR gatt_tx_pdu[BRR_MAX_PDU_SIZE];

    /* Reassembly offset for GATT */
    UINT16 gatt_rx_offset;

    /* Transmit offset for GATT */
    UINT16 gatt_tx_offset;

    /* OutMTU for the bearer */
    UINT16 omtu;

    /**
        Current Role of the GATT Bearer.
        Valid roles are:
        1. BRR_CLIENT_ROLE, or
        2. BRR_SERVER_ROLE
    */
    UCHAR  role;

} BRR_BEARER_DATA;

/* --------------------------------------------- Functions */
void brr_read_data_ind(BRR_HANDLE* brr_handle, UCHAR* data, UINT16 datalen, MS_BUFFER* info);

#endif /* _H_BRR_INTERNAL_ */

