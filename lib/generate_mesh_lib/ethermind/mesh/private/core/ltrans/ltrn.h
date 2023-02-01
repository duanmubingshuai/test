
/**
    \file ltrn.h

    This file contains the fuction definitions which are exported to other
    EtherMind modules for interfacing with Mesh Lower Transport Layer.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_LTRN_
#define _H_LTRN_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"

/* --------------------------------------------- Functions */
/**
    \brief Initializes Module.

    \par Description Initializes Module tables and registers interface with lower layer.

    \param None

    \return None
*/
void ms_ltrn_init(void);


/**
    \brief Shutsdown Module.

    \par Description:
    This function is the Shutdown handler for TRN module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.

    \param None

    \return None
*/
void ms_ltrn_shutdown (void);

#endif /* _H_LTRN_ */
