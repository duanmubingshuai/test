
/**
    \file sec_tbx_internal.h


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

#ifndef _H_SEC_TBX_INTERNAL_
#define _H_SEC_TBX_INTERNAL_

/* --------------------------------------------- Header File Inclusion */
#include "sec_tbx.h"
#include "cry.h"

/* --------------------------------------------- Global Definitions */
#ifdef STBX_NO_DEBUG
    #define STBX_ERR          EM_debug_null
#else /* STBX_NO_DEBUG */
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define STBX_ERR
    #else
        #define STBX_ERR(...)     EM_debug_error(MS_MODULE_ID_STBX, __VA_ARGS__)
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* STBX_NO_DEBUG */

#ifdef STBX_DEBUG
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define STBX_TRC
        #define STBX_INF

        #define STBX_debug_dump_bytes(data, datalen)

    #else
        #define STBX_TRC(...)     EM_debug_trace(MS_MODULE_ID_STBX,__VA_ARGS__)
        #define STBX_INF(...)     EM_debug_info(MS_MODULE_ID_STBX,__VA_ARGS__)

        #define STBX_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_STBX, (data), (datalen))

    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* STBX_DEBUG */
    #define STBX_TRC          EM_debug_null
    #define STBX_INF          EM_debug_null

    #define STBX_debug_dump_bytes(data, datalen)

#endif /* STBX_DEBUG */

/* --------------------------------------------- Structures/Data Types */

/* --------------------------------------------- Macros */

/* --------------------------------------------- Internal Functions */

/* --------------------------------------------- API Declarations */


#endif /* _H_SEC_TBX_INTERNAL_ */

