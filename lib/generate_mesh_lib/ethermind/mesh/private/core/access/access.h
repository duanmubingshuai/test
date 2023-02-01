
/**
    \file access.h

    This file contains the fuction definitions which are exported to other
    EtherMind modules for interfacing with Mesh Access Layer.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_ACCESS_
#define _H_ACCESS_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"

/* --------------------------------------------- Functions */
/**
    \brief Initializes Module.

    \par Description Initializes Module tables and registers interface with lower layer.

    \param None

    \return None
*/
void ms_access_init(void);


/**
    \par Description:
    This function is the Shutdown handler for Access module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.

    \param None

    \return None
*/
void ms_access_shutdown (void);

#endif /* _H_ACCESS_ */

