
/**
    \file prov.h

    This file contains the fuction definitions which are exported to other
    EtherMind modules for interfacing with Mesh Provisioning Layer.
*/

/*
    Copyright (C) 2018. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_PROV_
#define _H_PROV_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"

/* --------------------------------------------- Functions */
/**
    \brief Initializes Module.

    \par Description Initializes Module tables and registers interface with lower layer.

    \param None

    \return None
*/
void ms_prov_init(void);


/**
    \bried Shutsdown Module.

    \par Description:
    This function is the Shutdown handler for Provisioning module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.

    \param None

    \return None
*/
void ms_prov_shutdown (void);

#endif /* _H_PROV_ */
