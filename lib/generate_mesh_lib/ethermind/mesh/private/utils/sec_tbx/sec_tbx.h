
/**
    \file sec_tbx.h


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

#ifndef _H_SEC_TBX_
#define _H_SEC_TBX_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"

/* --------------------------------------------- Global Definitions */
#define STBX_128B_KEY_SIZE              16
#define STBX_TEMP_PLAINTEXT_SIZE        128

/* --------------------------------------------- Structures/Data Types */

/* --------------------------------------------- Macros */

/* --------------------------------------------- Internal Functions */

/* --------------------------------------------- API Declarations */
void ms_stbx_s1
(
    /* IN */  UCHAR* m,
    /* IN */  UINT16  mlen,
    /* OUT */ UCHAR* s1
);

void ms_stbx_k1
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* IN */  UCHAR* salt,
    /* IN */  UCHAR* p,
    /* IN */  UINT16  plen,
    /* OUT */ UCHAR* k1
);

void ms_stbx_k2
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* IN */  UCHAR* p,
    /* IN */  UINT16  plen,
    /* OUT */ UCHAR* k2
);

void ms_stbx_k3
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* OUT */ UCHAR* k3
);

void ms_stbx_k4
(
    /* IN */  UCHAR* n,
    /* IN */  UINT16  nlen,
    /* OUT */ UCHAR* k4
);

void ms_stbx_va
(
    /* IN */  UCHAR*   l,
    /* IN */  UINT16   llen,
    /* OUT */ UINT16* va
);

#endif /* _H_SEC_TBX_ */
