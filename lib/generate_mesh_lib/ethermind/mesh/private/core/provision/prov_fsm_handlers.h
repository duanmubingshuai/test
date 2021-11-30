
/**
    \file prov_fsm_handlers.h

    This file contains function handler declarations for PROV 20601 FSM.

*/

/*
    Copyright (C) 2011. MindTree Ltd.
    All rights reserved.
*/

#ifndef _H_PROV_FSM_HANDLERS_
#define _H_PROV_FSM_HANDLERS_

/* ----------------------------------------- Header File Inclusion */
#include "prov_fsm_engine.h"


#ifdef __cplusplus
extern "C" {
#endif

API_RESULT dummy_handler
(
    PROV_EVENT_INFO* param
);


API_RESULT se_prov_invite_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_error_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_capabilities_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_start_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_pubkey_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_inputcom_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_confirmation_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_random_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_data_handler
(
    PROV_EVENT_INFO*     param
);
API_RESULT se_prov_complete_handler
(
    PROV_EVENT_INFO*     param
);

#ifdef __cplusplus
};
#endif

#endif /* _H_PROV_FSM_HANDLERS_ */

