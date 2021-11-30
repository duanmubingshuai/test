
/**
    \file nvsto_internal.h


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

#ifndef _H_NVSTO_INTERNAL_
#define _H_NVSTO_INTERNAL_

/* --------------------------------------------- Header File Inclusion */
#include "nvsto.h"

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Structures/Data Types */
/** NVSTO Partition table structure */
typedef struct _NVSTO_PARTITION
{
    /* Partition base offset */
    UINT16 base;

    /* Partition Size */
    UINT16 size;

} NVSTO_PARTITION;

/* --------------------------------------------- Macros */

/* --------------------------------------------- Internal Functions */

#endif /* _H_NVSTO_INTERNAL_ */

