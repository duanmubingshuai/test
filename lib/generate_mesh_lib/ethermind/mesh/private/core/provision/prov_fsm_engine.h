
/**
    \file prov_fsm_defines.h

    This file defines state and events related to PROV FSM.
*/

/*
    Copyright (C) 2012. MindTree Ltd.
    All rights reserved.
*/

#ifndef _H_PROV_FSM_DEFINES_
#define _H_PROV_FSM_DEFINES_

/* --------------------------------------------- Header File Inclusion */
#include "fsm_engine.h"

#include "prov_internal.h"
#include "prov_fsm.h"

/* --------------------------------------------- Global Definitions */
/* Number of PROV events at the FSM */
#define PROV_MAX_NUM_EVENTS     0x0A

/* PROV Local and Remote event identifier */
#define PROV_FSM_LOCAL_EVENT    0x00
#define PROV_FSM_REMOTE_EVENT   0x01

/* --------------------------------------------- External Global Variables */
/* PROV FSM tables */
extern DECL_CONST EVENT_HANDLER_TABLE prov_event_handler_table[];
extern DECL_CONST STATE_EVENT_TABLE prov_state_event_table[];

/* --------------------------------------------- Structures/Data Types */
typedef struct _PROV_EVENT_INFO
{
    /*
        This handle will used to refer the prov_session structure
        for updating the tx/rx buffers, prov_state, SRM mode etc.
    */
    PROV_CONTEXT* ctx;

    /*
        Event Data to the handler: It will be processed in the handler function
        and based on the event type the processed data might be passed to application or
        lower layer
    */
    void* pdata;

    /* Event Data Length */
    UINT16  pdatalen;

    /* Event */
    EVENT_T     event_type;

    /* Additional information on event */
    UCHAR   event_info;

} PROV_EVENT_INFO;

/* --------------------------------------------- Macros */
#define PROV_ACCESS_STATE(ctx) \
    (ctx)->state

#define prov_fsm_post_levent(ctx, etype, pdata, pdatalen) \
    prov_fsm_post_event((ctx), (etype), PROV_FSM_LOCAL_EVENT, (pdata), (pdatalen))

#define prov_fsm_post_revent(ctx, etype, pdata, pdatalen) \
    prov_fsm_post_event((ctx), (etype), PROV_FSM_REMOTE_EVENT, (pdata), (pdatalen))

/* --------------------------------------------- Functions */
API_RESULT prov_fsm_register(void);
API_RESULT prov_fsm_post_event
(
    PROV_CONTEXT* ctx,
    EVENT_T        event_type,
    UCHAR          event_info,
    void*          pdata,
    UINT16         pdatalen
);
API_RESULT prov_access_state_handler (void* param, STATE_T* state);

#endif /* _H_PROV_FSM_DEFINES_ */

