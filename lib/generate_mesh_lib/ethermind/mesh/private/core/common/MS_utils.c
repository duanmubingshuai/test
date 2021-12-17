
/**
    \file MS_utilis.c

    This File contains Common Utility Routines such as Transition Timer Handling etc.
    used by various modules of the Mesh Stack.
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* ------------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "brr.h"
#include "net.h"
#include "ltrn.h"
#include "trn.h"
#include "access.h"


#if 0

    #ifdef MS_COMMON_TRC
        #undef MS_COMMON_TRC
        #define MS_COMMON_TRC printf
    #endif /* MS_COMMON_TRC */

    #ifdef MS_COMMON_INF
        #undef MS_COMMON_INF
        #define MS_COMMON_INF printf
    #endif /* MS_COMMON_INF */

    #ifdef MS_COMMON_ERR
        #undef MS_COMMON_ERR
        #define MS_COMMON_ERR printf
    #endif /* MS_COMMON_ERR */

    #ifdef MS_COMMON_debug_dump_bytes
        #undef MS_COMMON_debug_dump_bytes
        #define MS_COMMON_debug_dump_bytes appl_dump_bytes
    #endif /* MS_COMMON_debug_dump_bytes */

#endif /* 0 */

/* ------------------------------------------- Global Definitions */

/* ------------------------------------------- Static Global Variables */
/* Timer Transitions */
static MS_ACCESS_STATE_TRANSITION_TYPE ms_transition_timers[MS_MAX_NUM_TRANSITION_TIMERS];


/* ------------------------------------------- Functions */
/** Transition Timer related routines */
/* Initialization Routine */
API_RESULT ms_common_init_transition_timer(void)
{
    UINT32 index;
    EM_mem_set(ms_transition_timers, 0, sizeof(ms_transition_timers));

    /* Initialize Timer Handles */
    for (index = 0; index < MS_MAX_NUM_TRANSITION_TIMERS; index ++)
    {
        ms_transition_timers[index].transition_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    }

    return API_SUCCESS;
}

/* Utility Function */
/* Convert Transition Time to Milisecond */
static void ms_get_transition_time_in_ms(/* IN */ UINT8 transition_time, /* OUT */ UINT32* transition_time_in_ms)
{
    UINT32 transition_step_to_ms_lookup[4] = {100, 10*100, 10*10*100, 10*60*10*100};
    *transition_time_in_ms = (UINT8)(transition_time & 0x3F) * transition_step_to_ms_lookup[transition_time >> 6];
    MS_COMMON_TRC("Transition Time:0x%02X, converted in ms:0x%08X\n",
                  transition_time, *transition_time_in_ms);
}

/*
    Convert Transition Time from Milisecond to
    Generic Default Transition Time state format.
*/
API_RESULT MS_common_get_transition_time_from_ms
(
    /* IN */  UINT32 transition_time_in_ms,
    /* OUT */ UINT8* transition_time
)
{
    UINT8  temp_val;

    /* For 100 millisecond step resolution, the range is 0 through 6.2 seconds. */
    if (100 * 62 >= transition_time_in_ms)
    {
        temp_val = (UINT8)(transition_time_in_ms / 100);
        *transition_time = temp_val & 0x3F;
    }
    /* For 1 second step resolution, the range is 0 through 62 seconds. */
    else if ((10 * 100 * 62) >= transition_time_in_ms)
    {
        temp_val = (UINT8)(transition_time_in_ms / (10 * 100));
        *transition_time = 0x40 | (temp_val & 0x3F);
    }
    /* For 10 seconds step resolution, the range is 0 through 620 seconds. */
    else if ((10 * 10 * 100 * 62 ) >= transition_time_in_ms)
    {
        temp_val = (UINT8)(transition_time_in_ms / (10 * 10 * 100));
        *transition_time = 0x80 | (temp_val & 0x3F);
    }
    /* For 10 minutes step resolution, the range is 0 through 620 minutes. */
    else if ((10 * 60 * 10 * 100 * 62 ) >= transition_time_in_ms)
    {
        temp_val = (UINT8)(transition_time_in_ms / (10 * 60 * 10 * 100));
        *transition_time = 0xC0 | (temp_val & 0x3F);
    }
    else
    {
        MS_COMMON_ERR("Value 0x%08X: out of range\n", transition_time_in_ms);
        return API_FAILURE;
    }

    MS_COMMON_TRC("Transition Time in ms:0x%08X, converted:0x%02X\n",
                  transition_time_in_ms, *transition_time);
    return API_SUCCESS;
}

/* Transition State Timeout Handler */
static void ms_model_state_transition_timer_handler(void* args, UINT16 size)
{
    UINT16 index;
    UINT32 transition_timeout;
    MS_ACCESS_STATE_TRANSITION_TYPE* transition;
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(size);
    index = (UINT16)(*((UINT32*)args));
    /* TODO: Check range of 'index' */
    transition = &ms_transition_timers[index];
    MS_COMMON_TRC("State Transition Timer. State:0x%02X\n",
                  transition->transition_state);
    transition->transition_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

    /* Check state */
    if (0x01 == transition->transition_state)
    {
        /* Call Transition Start */
        MS_COMMON_TRC("<<<<State Transition Started>>>>\n");
        transition->transition_start_cb(transition->blob);
        ms_get_transition_time_in_ms(transition->transition_time, &transition_timeout);
        /* State Transition Started */
        transition->transition_state = 0x02;
        MS_COMMON_TRC("<<<<Transition Timeout: %d. Current Time: %s>>>>\n",
                      transition_timeout, EM_debug_get_current_timestamp());
        retval = EM_start_timer
                 (
                     &ms_transition_timers[index].transition_timer_handle,
                     (EM_TIMEOUT_MILLISEC | transition_timeout),
                     ms_model_state_transition_timer_handler,
                     (void*)&index,
                     sizeof(index)
                 );
    }
    else
    {
        MS_COMMON_TRC("<<<<State Transition Complete. Current Time: %s>>>>\n",
                      EM_debug_get_current_timestamp());
        transition->transition_complete_cb(transition->blob);
        /* State Transition Complete */
        transition->transition_time = 0;
        transition->transition_state = 0;
        transition->transition_start_cb = NULL;
        transition->transition_complete_cb = NULL;
    }

    return;
}

/* Start Transition Timer */
API_RESULT MS_common_start_transition_timer
(
    /* IN */  MS_ACCESS_STATE_TRANSITION_TYPE*    transition,
    /* OUT */ UINT16*                             transition_time_handle
)
{
    API_RESULT retval;
    UINT32     index;
    retval = API_SUCCESS;
    MS_COMMON_TRC("Transition Timeout. Delay:0x%02X, Time:0x%02X\n",
                  transition->delay, transition->transition_time);

    /* Allocate Timer */
    for (index = 0; index < MS_MAX_NUM_TRANSITION_TIMERS; index ++)
    {
        if (0 == ms_transition_timers[index].transition_state)
        {
            *transition_time_handle = (UINT16)index;
            MS_COMMON_TRC("Allocated Transition Time Handle:0x%04X\n",
                          *transition_time_handle);
            break;
        }
    }

    if (MS_MAX_NUM_TRANSITION_TIMERS == index)
    {
        MS_COMMON_TRC("Transition Time Handle Allocation Failed. Returning\n");
        return API_FAILURE;
    }

    ms_transition_timers[index].transition_state = 0x01;
    ms_transition_timers[index].delay = transition->delay;
    ms_transition_timers[index].transition_time = transition->transition_time;
    ms_transition_timers[index].transition_start_cb = transition->transition_start_cb;
    ms_transition_timers[index].transition_complete_cb = transition->transition_complete_cb;
    ms_transition_timers[index].blob = transition->blob;

    /* TODO: Check delay is not zero */
    if (0 != transition->delay)
    {
        retval = EM_start_timer
                 (
                     &ms_transition_timers[index].transition_timer_handle,
                     (EM_TIMEOUT_MILLISEC | (ms_transition_timers[index].delay * 5)),
                     ms_model_state_transition_timer_handler,
                     (void*)&index,
                     sizeof(index)
                 );
    }
    else
    {
        ms_model_state_transition_timer_handler((void*)&index, sizeof(index));
    }

    MS_COMMON_TRC("Timer Start Status: 0x%04X\n", retval);
    /* TODO: Not freeing allocated transition time element */
    return retval;
}

/* Stop Transition Timer */
API_RESULT MS_common_stop_transition_timer
(
    /* IN */ UINT16  transition_time_handle
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    MS_COMMON_TRC("[COMMON] Transition Timer Stop. Handle:%04X\n", transition_time_handle);

    if (MS_MAX_NUM_TRANSITION_TIMERS > transition_time_handle)
    {
        EM_stop_timer(&ms_transition_timers[transition_time_handle].transition_timer_handle);
        ms_transition_timers[transition_time_handle].transition_state = 0x00;
    }

    return retval;
}

/* Get Remaining Transition Time */
API_RESULT MS_common_get_remaining_transition_time
(
    /* IN */  UINT16   transition_time_handle,
    /* OUT */ UINT8*   remaining_transition_time
)
{
    API_RESULT retval;
    UINT32     remaining_time_ms;
    retval = API_SUCCESS;
    MS_COMMON_TRC("[COMMON] Transition Timer - Get remaining time. Handle:%04X\n", transition_time_handle);

    if (MS_MAX_NUM_TRANSITION_TIMERS > transition_time_handle)
    {
        MS_COMMON_TRC("<<<<Get remaining time. Current Time: %s>>>>\n",
                      EM_debug_get_current_timestamp());
        retval = EM_timer_get_remaining_time(ms_transition_timers[transition_time_handle].transition_timer_handle, &remaining_time_ms);

        if (API_SUCCESS == retval)
        {
            MS_COMMON_TRC("[COMMON] Remaining Transition Time: %d ms\n", remaining_time_ms);
            #if 0

            /* Reduce remaining transition time by 100ms to take care of transmission/processing delay */
            if (100 < remaining_time_ms)
            {
                remaining_time_ms -= 100;
            }

            #endif /* 0 */
            MS_common_get_transition_time_from_ms
            (
                remaining_time_ms,
                remaining_transition_time
            );
        }
    }

    return retval;
}

/* Get Remaining Transition Time - with offset */
API_RESULT MS_common_get_remaining_transition_time_with_offset
(
    /* IN */  UINT16   transition_time_handle,
    /* IN */  UINT32   offset_in_ms,
    /* OUT */ UINT8*   remaining_transition_time
)
{
    API_RESULT retval;
    UINT32     remaining_time_ms;
    retval = API_SUCCESS;
    MS_COMMON_TRC("[COMMON] Transition Timer - Get remaining time. Handle:%04X\n", transition_time_handle);

    if (MS_MAX_NUM_TRANSITION_TIMERS > transition_time_handle)
    {
        MS_COMMON_TRC("<<<<Get remaining time. Current Time: %s>>>>\n",
                      EM_debug_get_current_timestamp());
        retval = EM_timer_get_remaining_time(ms_transition_timers[transition_time_handle].transition_timer_handle, &remaining_time_ms);

        if (API_SUCCESS == retval)
        {
            MS_COMMON_TRC("[COMMON] Remaining Transition Time: %d ms\n", remaining_time_ms);

            /* Reduce remaining transition time by 100ms to take care of transmission/processing delay */
            if (offset_in_ms < remaining_time_ms)
            {
                remaining_time_ms -= offset_in_ms;
            }

            MS_common_get_transition_time_from_ms
            (
                remaining_time_ms,
                remaining_transition_time
            );
        }
    }

    return retval;
}
