
/**
    \file fsm_engine.c

    This file implements the Routines to Invoke FSM Event Handlers.
*/

/*
    Copyright (C) 2013. Mindtree Ltd.
    All rights reserved.
*/

/* ----------------------------------------- Header File Inclusion */
#include "fsm_engine.h"

/* ----------------------------------------- External Global Variables */

/* ----------------------------------------- Exported Global Variables */

/* ----------------------------------------- Static Global Variables */
DECL_STATIC DECL_CONST FSM_MODULE_TABLE_T* fsm_module_table[FSM_MAX_MODULES];

void ms_fsm_init (void)
{
    UCHAR i;
    FSM_TRC ("[FSM]: FSM Init\n");

    for (i = 0; i < FSM_MAX_MODULES; i ++)
    {
        fsm_module_table[i] = NULL;
    }

    return;
}


API_RESULT ms_fsm_register_module
(
    /* IN */  DECL_CONST FSM_MODULE_TABLE_T* module_fsm,
    /* OUT */ UCHAR* fsm_id
)
{
    API_RESULT  retval;
    UCHAR i;
    FSM_TRC ("[FSM]: FSM Register Module\n");
    /* Initialize Return */
    retval = API_SUCCESS;

    for (i = 0; i < FSM_MAX_MODULES; i ++)
    {
        if (NULL == fsm_module_table[i])
        {
            FSM_TRC ("[FSM]: Registering Module at FSM Index %d\n", i);
            fsm_module_table[i] = module_fsm;
            break;
        }
    }

    /* Update the index assigned */
    *fsm_id = i;

    if (FSM_MAX_MODULES == i)
    {
        FSM_ERR ("[FSM]: FSM Register Failed. No free index available!\n");
        retval = API_FAILURE;
    }

    return retval;
}

/**
    This internal routine handler, handles all FSM events occurring due to
    Transport, Application and Peer Activities.
    This routine identifies the FSM Event, Handlers related that can handle it.
    And forward the Event to invoke_event_handler that care of state check,
    handling based on State and State Change that is caused by the Event Handling
*/
API_RESULT ms_fsm_post_event
(
    UCHAR      fsm_id,
    EVENT_T    fsm_event,
    void*      param
)
{
    UINT32 handler_index;
    UINT32 event_index, handler_start_index, handler_end_index;
    RETVAL_T retval;
    STATE_T cur_state;
    STATE_T expected_state;
    FSM_TRC (
        "[FSM]:[0x%02X]: Received event %02X\n", fsm_id, fsm_event);
    retval = API_SUCCESS;
    handler_start_index = 0;
    handler_end_index = 0;
    #ifdef FSM_DEBUG_PRINT_STATES
    print_state_name
    (
        (void*)bt_debug_fd,
        ATT_ACCESS_STATE(param->instance)
        event_index
    );
    print_state_name
    (
        (void*)stdout,
        ATT_ACCESS_STATE(param->instance),
        event_index
    );
    #endif /* FSM_DEBUG_PRINT_STATES */

    for (event_index = 0; event_index < fsm_module_table[fsm_id]->\
            state_event_table_size; event_index ++)
    {
        if (fsm_module_table[fsm_id]->state_event_table[event_index].event ==
                fsm_event)
        {
            #ifdef FSM_DEBUG_PRINT_STATES
            FSM_TRC (
                "[FSM]: Found Event %02X @ %08X\n",param->event_type, event_index);
            #endif /* FSM_DEBUG_PRINT_STATES */
            handler_start_index = fsm_module_table[fsm_id]->state_event_table
                                  [event_index].event_handler_table_start_index;
            handler_end_index = fsm_module_table[fsm_id]->state_event_table
                                [event_index].event_handler_table_end_index;
            break;
        }
    }

    if (fsm_module_table[fsm_id]->state_event_table_size == event_index)
    {
        FSM_ERR(
            "[FSM]: Unknown Event 0x%02X Received - dropping event\n",
            fsm_event);
        return API_FAILURE;
    }

    FSM_TRC (
        "[FSM]: Event %02X Start %08X End %08X\n", fsm_event,
        handler_start_index, handler_end_index);
    retval = fsm_module_table[fsm_id]->state_access_handler(param,&cur_state);

    /* Access the corresponding event handler table */
    for
    (
        handler_index = handler_start_index;
        handler_index <= handler_end_index;
        handler_index ++
    )
    {
        /* Check if the handler is present and the state check is successful */
        expected_state = fsm_module_table[fsm_id]->event_table
                         [handler_index].current_state;

        if (0x0000 != (expected_state & cur_state))
        {
            /* Current state matches with the table entry, invoke handler */
            if (NULL != fsm_module_table[fsm_id]->event_table[handler_index].\
                    handler)
            {
                retval = fsm_module_table[fsm_id]->event_table[handler_index].\
                         handler(param);

                /**
                    NOTE: Handlers may already have used this API in order to
                    change the state, in which case no state change is needed.
                    Therefore NULL handlers are allowed for modules.
                */
                if (NULL != fsm_module_table[fsm_id]->state_change_handler)
                {
                    fsm_module_table[fsm_id]->state_change_handler
                    (
                        param,
                        (STATE_T)retval
                    );
                }
            }
            else
            {
                /* Event not handled in Current state */
                FSM_ERR(
                    "[FSM]:[%02X] Received Unknown Event %02X\n",
                    fsm_id, fsm_event);
            }

            break;
        }
    }

    #ifdef FSM_DEBUG_PRINT_STATES
    print_state_name
    (
        (void*)bt_debug_fd,
        fsm_module_table[fsm_id]->state_access_handler(param)
        event_index
    );
    print_state_name
    (
        (void*)stdout,
        fsm_module_table[fsm_id]->state_access_handler(param),
        event_index
    );
    #endif /* FSM_DEBUG_PRINT_STATES */
    return retval;
}

