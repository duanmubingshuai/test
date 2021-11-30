
/**
    \file prov_fsm_engine.c

    This file implements the Routines to Invoke FSM Event Handlers.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/
/* ----------------------------------------- Header File Inclusion */
#include "prov_fsm_engine.h"


/* ----------------------------------------- External Global Variables */

/* ----------------------------------------- Exported Global Variables */

/* ----------------------------------------- Static Global Variables */
/** Provisioning FSM entity */
DECL_STATIC DECL_CONST FSM_MODULE_TABLE_T prov_fsm =
{
    prov_access_state_handler,

    NULL,

    prov_event_handler_table,

    prov_state_event_table,

    PROV_MAX_NUM_EVENTS
};

/** FSM ID for Provisioning module */
DECL_STATIC UCHAR prov_fsm_id;

/* ----------------------------------------- Functions */
API_RESULT prov_fsm_register(void)
{
    return ms_fsm_register_module(&prov_fsm, &prov_fsm_id);
}

/**
    \brief

    \par Description


    \param prov_event

    \return None
*/
API_RESULT prov_fsm_post_event
(
    PROV_CONTEXT* ctx,
    EVENT_T        event_type,
    UCHAR          event_info,
    void*          pdata,
    UINT16         pdatalen
)
{
    PROV_EVENT_INFO  prov_event;
    prov_event.ctx = ctx;
    prov_event.event_type = event_type;
    prov_event.event_info = event_info;
    prov_event.pdata = pdata;
    prov_event.pdatalen = pdatalen;
    /* Post event to FSM */
    return ms_fsm_post_event (prov_fsm_id, prov_event.event_type, &prov_event);
}


/**
    \brief

    \par Description


    \param param
    \param state

    \return None
*/
API_RESULT prov_access_state_handler
(
    void*        param,
    STATE_T*     state
)
{
    *state = PROV_ACCESS_STATE((((PROV_EVENT_INFO*)param)->ctx));
    return API_SUCCESS;
}



