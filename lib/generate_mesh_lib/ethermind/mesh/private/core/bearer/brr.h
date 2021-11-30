
/**
    \file brr.h

    This file contains the fuction definitions which are exported to other
    EtherMind modules for interfacing with Mesh Bearer Layer.
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_BRR_
#define _H_BRR_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"

/* --------------------------------------------- Functions */
/**
    \brief Initializes Module.

    \par Description Initializes Module tables and registers interface with lower layer.

    \param None

    \return None
*/
void ms_brr_init(void);


/**
    \brief Shutsdown Module.

    \par Description:
    This function is the Shutdown handler for Bearer module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.

    \param None

    \return None
*/
void ms_brr_shutdown (void);

#endif /* _H_BRR_ */
