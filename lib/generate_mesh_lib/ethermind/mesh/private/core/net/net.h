
/**
    \file net.h

    This file contains the fuction definitions which are exported to other
    EtherMind modules for interfacing with Mesh Network Layer.
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_NET_
#define _H_NET_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"

/* --------------------------------------------- Functions */
/**
    \brief Initializes Module.

    \par Description Initializes Module tables and registers interface with lower layer.

    \param None

    \return None
*/
void ms_net_init(void);


/**
    \bried Shutsdown Module.

    \par Description:
    This function is the Shutdown handler for NET module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.

    \param None

    \return None
*/
void ms_net_shutdown (void);

#endif /* _H_NET_ */
