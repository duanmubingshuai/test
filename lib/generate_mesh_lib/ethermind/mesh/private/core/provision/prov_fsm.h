
/**
    \file prov_fsm.h

    This file defines state and events related to PROV FSM.
*/

/*
    Copyright (C) 2011. MindTree Ltd.
    All rights reserved.
*/

#ifndef _H_PROV_FSM_
#define _H_PROV_FSM_


#if 0
    char event_option[] =
    "    --------- EVENTS ---------\n\
    \n\
    0x01 -> ev_prov_invite    \n\
    0x02 -> ev_prov_capabilities    \n\
    0x03 -> ev_prov_start    \n\
    0x04 -> ev_prov_pubkey    \n\
    0x05 -> ev_prov_inputcom    \n\
    0x06 -> ev_prov_confirmation    \n\
    0x07 -> ev_prov_random    \n\
    0x08 -> ev_prov_data    \n\
    0x09 -> ev_prov_complete    \n\
    0x0A -> ev_prov_error    \n\
    11 -> Exit    \n\
    \n\
    Your Option - ";
#endif



/* Event Defines */
typedef enum
{
    ev_prov_invite = 0x01,
    ev_prov_capabilities = 0x02,
    ev_prov_start = 0x03,
    ev_prov_pubkey = 0x04,
    ev_prov_inputcom = 0x05,
    ev_prov_confirmation = 0x06,
    ev_prov_random = 0x07,
    ev_prov_data = 0x08,
    ev_prov_complete = 0x09,
    ev_prov_error = 0x0A

} EVENTS;

/* Level 0 State Defines */
typedef enum
{
    SL_0_PROV_IDLE = 0x00000001,
    SL_0_PROV_W4CAPAB = 0x00000002,
    SL_0_PROV_INCAPAB = 0x00000004,
    SL_0_PROV_W4START = 0x00000008,
    SL_0_PROV_INSTART = 0x00000010,
    SL_0_PROV_W4PUBKEY = 0x00000020,
    SL_0_PROV_INPUBKEY = 0x00000040,
    SL_0_PROV_W4INPCOM = 0x00000080,
    SL_0_PROV_ININPCOM = 0x00000100,
    SL_0_PROV_W4CONF = 0x00000200,
    SL_0_PROV_INCONF = 0x00000400,
    SL_0_PROV_W4RAND = 0x00000800,
    SL_0_PROV_INRAND = 0x00001000,
    SL_0_PROV_W4DATA = 0x00002000,
    SL_0_PROV_INDATA = 0x00004000,
    SL_0_PROV_W4COMPLETE = 0x00008000,
    SL_0_PROV_INCOMPLETE = 0x00010000,
    SL_0_PROV_ERROR = 0x00020000

} STATES_LEVEL_0;

#endif /* _H_PROV_FSM_ */

