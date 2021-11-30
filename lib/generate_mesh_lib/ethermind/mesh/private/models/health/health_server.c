/**
    \file health_server.c
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/



/* --------------------------------------------- Header File Inclusion */
#include "health_server.h"
#include "bitarray.h"

#if 0

    #ifdef HEALTH_SERVER_TRC
        #undef HEALTH_SERVER_TRC
        #define HEALTH_SERVER_TRC printf
    #endif /* HEALTH_SERVER_TRC */

    #ifdef HEALTH_SERVER_INF
        #undef HEALTH_SERVER_INF
        #define HEALTH_SERVER_INF printf
    #endif /* HEALTH_SERVER_INF */

    #ifdef HEALTH_SERVER_ERR
        #undef HEALTH_SERVER_ERR
        #define HEALTH_SERVER_ERR printf
    #endif /* HEALTH_SERVER_ERR */

    #ifdef HEALTH_SERVER_debug_dump_bytes
        #undef HEALTH_SERVER_debug_dump_bytes
        #define HEALTH_SERVER_debug_dump_bytes appl_dump_bytes
    #endif /* HEALTH_SERVER_debug_dump_bytes */

#endif /* 0 */

/* --------------------------------------------- Global Definitions */



/* --------------------------------------------- Static Global Variables */
/* Health Server Fault array size */
#define MS_HEALTH_SERVER_FAULT_ARRAY_SIZE              256

/**
    Health Server Fault array effective max size.
    0x00-0x32: Spec Defined (51 values).
    0x33-0x7F: RFU (77 values).
    0x80-0xFF: Vendor Specific (128 values).

    Effective total valid values = 51 + 128 = 179.
*/
#define MS_HEALTH_SERVER_EFFECTIVE_FAULT_ARRAY_SIZE    179

#define MS_HEALTH_SERVER_IS_FAULT_CODE_VALID(fc) \
    (((0x32 < (fc)) && (0x80 > (fc))) ? MS_FALSE : MS_TRUE)

/** Attention Timer Interval */
#define MS_ATTENTION_TIMER_INTERVAL_IN_SEC       1

/** Health Server related states */
typedef struct _MS_HEALTH_SERVER_T
{
    /* Current Fault State */
    UINT32 current_fault[BITARRAY_NUM_BLOCKS(MS_HEALTH_SERVER_FAULT_ARRAY_SIZE)];

    /* Registered Fault State */
    UINT32 registered_fault[BITARRAY_NUM_BLOCKS(MS_HEALTH_SERVER_FAULT_ARRAY_SIZE)];

    /* Health Period Divisor */
    UINT8  period_divisor;

    /* Fast Step Counter - to identify periodic and fast timeouts */
    UINT16 fast_step_counter;

    /* Attention Timeout value in seconds */
    UINT8  attention;

    /* Attention Timer Handle */
    EM_timer_handle attention_timer;

    /* Company ID */
    UINT16  company_id;

    /* List of Self Tests */
    MS_HEALTH_SERVER_SELF_TEST* self_tests;

    /* Number of Self Tests */
    UINT32 num_self_tests;

    /* Last Test ID */
    UINT8  last_test_id;

    /* Model Handle - Associated with Health Server */
    MS_ACCESS_MODEL_HANDLE   model_handle;

    /* Application Callback */
    MS_HEALTH_SERVER_CB      appl_cb;

} MS_HEALTH_SERVER_T;

static DECL_CONST UINT32 health_server_opcode_list[] =
{
    MS_ACCESS_HEALTH_ATTENTION_GET_OPCODE,
    MS_ACCESS_HEALTH_FAULT_GET_OPCODE,
    MS_ACCESS_HEALTH_FAULT_CLEAR_OPCODE,
    MS_ACCESS_HEALTH_PERIOD_SET_OPCODE,
    MS_ACCESS_HEALTH_PERIOD_GET_OPCODE,
    MS_ACCESS_HEALTH_FAULT_CLEAR_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_HEALTH_FAULT_TEST_OPCODE,
    MS_ACCESS_HEALTH_ATTENTION_SET_OPCODE,
    MS_ACCESS_HEALTH_ATTENTION_SET_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_HEALTH_FAULT_TEST_UNACKNOWLEDGED_OPCODE,
    MS_ACCESS_HEALTH_PERIOD_SET_UNACKNOWLEDGED_OPCODE,
};

static MS_ACCESS_MODEL_HANDLE   health_server_model_handle;

MS_DEFINE_GLOBAL_ARRAY(MS_HEALTH_SERVER_T, health_server, MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX));
static UINT8                    health_server_initialized = MS_FALSE;

/* --------------------------------------------- External Global Variables */


/* --------------------------------------------- Function */
/** Initialize Health Server entity */
static void health_server_init_entities
(
    void
)
{
    UINT32 index;
    /* Init global data structures */
    MS_INIT_GLOBAL_ARRAY(MS_HEALTH_SERVER_T, health_server, MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX), 0x00);

    for (index = 0; index < MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX); index++)
    {
        health_server[index].attention_timer = EM_TIMER_HANDLE_INIT_VAL;
    }

    /* num_self_tests as 0, will be used to identify if the health server entity is already allocated */
}

/* Allocate a Health Server entity */
static UINT32 health_server_get_free_entity(void)
{
    UINT32 index;

    /* Search for free health server entity */
    for (index = 0; index < (MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX)); index++)
    {
        if (0 == health_server[index].num_self_tests)
        {
            break;
        }
    }

    return index;
}

/* Search a Health Server entity */
static UINT32 health_server_search_entity(/* IN */ MS_ACCESS_MODEL_HANDLE* handle)
{
    UINT32 index;

    /* Search for free health server entity */
    for (index = 0; index < (MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX)); index++)
    {
        if ((0 != health_server[index].num_self_tests) &&
                ((*handle) == health_server[index].model_handle))
        {
            break;
        }
    }

    return index;
}

/** Attention Timeout Handler */
static void health_server_attention_timeout_handler(void* args, UINT16 size)
{
    UINT32        health_server_index;
    UINT8         attention;
    API_RESULT    retval;
    MS_IGNORE_UNUSED_PARAM(size);
    HEALTH_SERVER_TRC(
        "[HEALTH] Attention Timeout\n");
    health_server_index = *((UINT32*)args);
    attention = health_server[health_server_index].attention;
    HEALTH_SERVER_TRC(
        "[HEALTH] Index: 0x%08X. Timeout Value: 0x%02X\n",
        health_server_index, attention);

    /* Check if already reached 0, because of set attention */
    if (0x00 == attention)
    {
        HEALTH_SERVER_TRC(
            "[HEALTH] Attention Timeout is already Zero. No taking any action\n");
    }
    else
    {
        /* Decrement Attention Timeout value */
        attention--;
        health_server[health_server_index].attention = attention;

        if (0x00 == attention)
        {
            /* Inform application */
            health_server[health_server_index].attention_timer = EM_TIMER_HANDLE_INIT_VAL;
            /* TODO: Check NULL */
            health_server[health_server_index].appl_cb
            (
                &health_server[health_server_index].model_handle,
                MS_HEALTH_SERVER_ATTENTION_STOP,
                NULL,
                0
            );
        }
        else
        {
            /* Restart timer */
            retval = EM_start_timer
                     (
                         &health_server[health_server_index].attention_timer,
                         MS_ATTENTION_TIMER_INTERVAL_IN_SEC,
                         health_server_attention_timeout_handler,
                         &health_server_index,
                         sizeof(health_server_index)
                     );
        }
    }

    return;
}


/** Set Attention */
static void health_server_set_attention
(
    /* IN */ UINT32                   health_server_index,
    /* IN */ UINT8                    attention
)
{
    API_RESULT retval;
    HEALTH_SERVER_TRC(
        "[HEALTH] Set Attention. Index: 0x%08X. Timeout Value: 0x%02X\n",
        health_server_index, attention);
    /* If attention timer value is set to zero, stop the timer (if running) */
    health_server[health_server_index].attention = attention;

    if (0x00 == attention)
    {
        if (EM_TIMER_HANDLE_INIT_VAL == health_server[health_server_index].attention_timer)
        {
            /* Do not inform the application about the Attention Timer stop */
        }
        else
        {
            /* Stop timer and inform application */
            EM_stop_timer(&health_server[health_server_index].attention_timer);
            /* TODO: Check NULL */
            health_server[health_server_index].appl_cb
            (
                &health_server[health_server_index].model_handle,
                MS_HEALTH_SERVER_ATTENTION_STOP,
                NULL,
                0
            );
        }
    }
    /* Else, start/restart the timer with the specified value */
    else
    {
        if (EM_TIMER_HANDLE_INIT_VAL == health_server[health_server_index].attention_timer)
        {
            /* Start timer and inform application */
            retval = EM_start_timer
                     (
                         &health_server[health_server_index].attention_timer,
                         MS_ATTENTION_TIMER_INTERVAL_IN_SEC,
                         health_server_attention_timeout_handler,
                         &health_server_index,
                         sizeof(health_server_index)
                     );
            /* TODO: Check NULL */
            health_server[health_server_index].appl_cb
            (
                &health_server[health_server_index].model_handle,
                MS_HEALTH_SERVER_ATTENTION_START,
                &attention,
                sizeof(attention)
            );
        }
        else
        {
            /* Inform the application about the Attention Timer restart */
            EM_stop_timer(&health_server[health_server_index].attention_timer);
            /* Start timer */
            retval = EM_start_timer
                     (
                         &health_server[health_server_index].attention_timer,
                         MS_ATTENTION_TIMER_INTERVAL_IN_SEC,
                         health_server_attention_timeout_handler,
                         &health_server_index,
                         sizeof(health_server_index)
                     );
            /* TODO: Check NULL */
            health_server[health_server_index].appl_cb
            (
                &health_server[health_server_index].model_handle,
                MS_HEALTH_SERVER_ATTENTION_RESTART,
                &attention,
                sizeof(attention)
            );
        }
    }

    /* Inform the application about the attention start or stop state */
}

/** Get Attention */
static void health_server_get_attention
(
    /* IN */  UINT32                  health_server_index,
    /* OUT */ UINT8*                  attention,
    /* OUT */ UINT32*                 response_opcode
)
{
    *attention = health_server[health_server_index].attention;
    *response_opcode = MS_ACCESS_HEALTH_ATTENTION_STATUS_OPCODE;
}

/** Set Health Period Divisor */
static void health_server_set_period_divisor
(
    /* IN */ UINT32                   health_server_index,
    /* IN */ UINT8                    period_divisor
)
{
    API_RESULT retval;
    /* TODO: Can there be more than one instance of Health Server */
    health_server[health_server_index].period_divisor = period_divisor;
    health_server[health_server_index].fast_step_counter = 0;
    retval = MS_access_cm_set_model_publication_period_divisor
             (
                 health_server_model_handle,
                 period_divisor
             );
    /* TODO: Print retval */
}

/** Get Health Period Divisor */
static void health_server_get_period_divisor
(
    /* IN */  UINT32                   health_server_index,
    /* OUT */ UINT8*                   period_divisor,
    /* OUT */ UINT32*                  response_opcode
)
{
    *period_divisor = health_server[health_server_index].period_divisor;
    *response_opcode = MS_ACCESS_HEALTH_PERIOD_STATUS_OPCODE;
}

/** Get Fault Status */
static API_RESULT health_server_get_fault_status
(
    /* IN */    UINT32                   health_server_index,
    /* IN */    UINT32                   opcode,
    /* IN */    UINT16                   company_id,
    /* OUT */   UINT8*                   response_buffer,
    /* INOUT */ UINT16*                  response_buffer_size,
    /* OUT */   UINT32*                  response_opcode
)
{
    UINT32* fault_bitarray;
    UINT8    marker;

    /* Match Company ID */
    if (company_id != health_server[health_server_index].company_id)
    {
        HEALTH_SERVER_ERR(
            "[HEALTH] Company ID mismatch. Returning\n");
        return API_FAILURE;
    }

    /**
         ## Health Current Status message Format ##

                                               .----------- N Bytes ------------.
                                              /                                  \
          <-  Byte 0  -> <-    Byte 1, 2   -> <-         Byte 3, ...            ->
          7            0 15                 0 N-1                                 0
          +-------------+--------------------+--------------- . . . . . ----------+
          |   Test ID   |     Company ID     | FaultArray ...                     |
          +-------------+--------------------+-------+------- . . . . . ----------+
    */
    /* Fill Test ID */
    marker = 0;
    response_buffer[marker] = health_server[health_server_index].last_test_id;
    marker++;
    /* Fill Company ID */
    /* TODO: Use macro */
    response_buffer[marker] = (UINT8)health_server[health_server_index].company_id;
    response_buffer[marker+1] = (UINT8)((health_server[health_server_index].company_id) >> 8);
    marker += 2;

    /* Select Fault Array based on Opcode */
    if (MS_ACCESS_HEALTH_FAULT_GET_OPCODE == opcode)
    {
        fault_bitarray = health_server[health_server_index].registered_fault;
        *response_opcode = MS_ACCESS_HEALTH_FAULT_STATUS_OPCODE;
    }
    else
    {
        fault_bitarray = health_server[health_server_index].current_fault;
        *response_opcode = MS_ACCESS_HEALTH_CURRENT_STATUS_OPCODE;
    }

    /* Fill the response with Fault Code saved in bit array */
    {
        /* Search for all set bits */
        UINT32 start, next;
        start = 0;
        MS_LOOP_FOREVER()
        {
            next = bitarray_get_lowest_bit_set
                   (
                       fault_bitarray,
                       MS_HEALTH_SERVER_FAULT_ARRAY_SIZE,
                       start
                   );

            if (0xFFFFFFFF == next)
            {
                break;
            }

            /* Fill Fault Code */
            response_buffer[marker] = (UINT8)next;
            marker++;
            start = next + 1;
        }
    }
    *response_buffer_size = marker;
    return API_SUCCESS;
}

/* Clear Fault */
static void health_server_clear_fault
(
    /* IN */    UINT32                   health_server_index,
    /* IN */    UINT16                   company_id
)
{
    MS_IGNORE_UNUSED_PARAM(company_id);
    /* Clear registered fault status in bitarray */
    bitarray_reset_all(health_server[health_server_index].registered_fault, MS_HEALTH_SERVER_FAULT_ARRAY_SIZE);
}

/** Run Self Tests */
static API_RESULT health_server_run_self_test
(
    /* IN */    UINT32                   health_server_index,
    /* IN */    UINT8                    test_id,
    /* IN */    UINT16                   company_id
)
{
    API_RESULT retval;
    UINT32     index;
    retval = API_FAILURE;

    for (index = 0; index < health_server[health_server_index].num_self_tests; index++)
    {
        /* Match the Test ID */
        if (test_id == health_server[health_server_index].self_tests[index].test_id)
        {
            /* Run the self test */
            health_server[health_server_index].self_tests[index].self_test_fn(test_id, company_id);
            retval = API_SUCCESS;
            break;
        }
    }

    return retval;
}

/**
    \brief API to initialize Health Server model

    \par Description
    This is to initialize Health Server model and to register with Access layer.

    \param [in] element_handle
                Element identifier to be associated with the model instance.

    \param [in, out] model_handle
                     Model identifier associated with the model instance on successful initialization.
                     After power cycle of an already provisioned node, the model handle will have
                     valid value and the same will be reused for registration.

    \param [in]      company_id
                     Company Identifier

    \param [in]      self_tests
                     List of Self Tests that can be run.

    \param [in]      num_self_tests
                     Number of Self Tests in the list.

    \param [in] appl_cb    Application Callback to be used by the Health Server.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_health_server_init
(
    /* IN */    MS_ACCESS_ELEMENT_HANDLE     element_handle,
    /* INOUT */ MS_ACCESS_MODEL_HANDLE*      model_handle,
    /* IN */    UINT16                       company_id,
    /* IN */    MS_HEALTH_SERVER_SELF_TEST* self_tests,
    /* IN */    UINT32                       num_self_tests,
    /* IN */    MS_HEALTH_SERVER_CB          appl_cb
)
{
    API_RESULT           retval;
    MS_ACCESS_NODE_ID    node_id;
    MS_ACCESS_MODEL      model;
    UINT32               index;

    /* TBD: Initialize MUTEX and other data structures */

    if (MS_FALSE == health_server_initialized)
    {
        health_server_init_entities();
        health_server_initialized = MS_TRUE;
    }

    /* Allocate a Health Server entity */
    index = health_server_get_free_entity();

    if (MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX) <= index)
    {
        retval = HEALTH_CONTEXT_ALLOC_FAILED;
        HEALTH_SERVER_ERR(
            "[HEALTH] No free Health Server entity. Returning: 0x%04X\n", retval);
        return retval;
    }

    /* Using default node ID */
    node_id = MS_ACCESS_DEFAULT_NODE_ID;
    HEALTH_SERVER_TRC(
        "[HEALTH] Element Handle 0x%02X\n", element_handle);
    /* Configure Model */
    model.model_id.id = MS_MODEL_ID_HEALTH_SERVER;
    model.model_id.type = MS_ACCESS_MODEL_TYPE_SIG;
    model.elem_handle = element_handle;
    /* Register Callbacks */
    model.cb = health_server_cb;
    model.pub_cb = health_server_publish_timout_cb;
    /* List of Opcodes */
    model.opcodes = health_server_opcode_list;
    model.num_opcodes = sizeof(health_server_opcode_list) / sizeof(UINT32);
    retval = MS_access_register_model
             (
                 node_id,
                 &model,
                 model_handle
             );
    /* TODO: Remove */
    health_server_model_handle = *model_handle;
    /* Save information in corresponding health server entity */
    health_server[index].model_handle = *model_handle;
    health_server[index].company_id = company_id;
    health_server[index].self_tests = self_tests;
    health_server[index].num_self_tests = num_self_tests;
    health_server[index].appl_cb = appl_cb;
    return retval;
}

/**
    \brief API to report self-test fault

    \par Description
    This is to report fault observed during self-test procedure.

    \param [in] model_handle
                Model Handle identifying the Health Server model instance.

    \param [in]      test_id
                     Identifier of the self-test

    \param [in]      company_id
                     Company Identifier

    \param [in]      fault_code
                     Fault value indicating the error.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_health_server_report_fault
(
    /* IN */ MS_ACCESS_MODEL_HANDLE*      model_handle,
    /* IN */ UINT8                        test_id,
    /* IN */ UINT16                       company_id,
    /* IN */ UINT8                        fault_code
)
{
    UINT32 index;
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(company_id);

    /* Check fault code */
    if (MS_FALSE == MS_HEALTH_SERVER_IS_FAULT_CODE_VALID(fault_code))
    {
        retval = HEALTH_INVALID_PARAMETER;
        HEALTH_SERVER_ERR(
            "[HEALTH] Fault Code 0x%02X from reserve range [0x33, 0x7F]. Returning: 0x%04X\n",
            fault_code, retval);
        return retval;
    }

    /* Search corresponding Health Model entity */
    index = health_server_search_entity(model_handle);

    if (MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX) <= index)
    {
        retval = HEALTH_CONTEXT_ASSERT_FAILED;
        HEALTH_SERVER_ERR(
            "[HEALTH] No matching Health Server entity. Returning: 0x%04X\n", retval);
        return retval;
    }

    retval = API_SUCCESS;
    /* Mark current and registered fault status in bitarray */
    bitarray_set_bit(health_server[index].current_fault, fault_code);
    bitarray_set_bit(health_server[index].registered_fault, fault_code);
    health_server[index].last_test_id = test_id;
    /* TODO: If this is the first fault (no fault was present), switch to fast publishing */
    return retval;
}

/**
    TODO: API added for PTS test on LPN-BV-02. API header not
    updated because the API can be more generic
*/
API_RESULT MS_health_server_publish_current_status
(
    UCHAR*     status,
    UINT16     length
)
{
    API_RESULT retval;
    retval = MS_access_publish
             (
                 &health_server_model_handle,
                 MS_ACCESS_HEALTH_CURRENT_STATUS_OPCODE,
                 status,
                 length,
                 MS_FALSE
             );
    return retval;
}

/* Empty Model Opcode Handler Defines */
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_attention_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_fault_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_fault_clear_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_period_set_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_period_get_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_fault_clear_unacknowledged_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_fault_test_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_attention_set_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_attention_set_unacknowledged_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_fault_test_unacknowledged_handler)
MODEL_OPCODE_HANDLER_EMPTY_DEF(health_period_set_unacknowledged_handler)


/**
    \brief Access Layer Application Asynchronous Notification Callback.

    \par Description
    Access Layer calls the registered callback to indicate events occurred to the application.

    \param [in] handle        Model Handle.
    \param [in] saddr         16 bit Source Address.
    \param [in] daddr         16 bit Destination Address.
    \param [in] appkey_handle AppKey Handle.
    \param [in] subnet_handle Subnet Handle.
    \param [in] opcode        Opcode.
    \param [in] data_param    Data associated with the event if any or NULL.
    \param [in] data_len      Size of the event data. 0 if event data is NULL.
*/
API_RESULT health_server_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ MS_NET_ADDR              saddr,
    /* IN */ MS_NET_ADDR              daddr,
    /* IN */ MS_SUBNET_HANDLE         subnet_handle,
    /* IN */ MS_APPKEY_HANDLE         appkey_handle,
    /* IN */ UINT32                   opcode,
    /* IN */ UCHAR*                   data_param,
    /* IN */ UINT16                   data_len
)
{
    API_RESULT retval;
    UINT16     marker;
    UINT8      test_id;
    UINT16     company_id;
    UINT8      fast_period_div;
    UINT8      attention;
    UINT32     response;
    UINT8      status[MS_HEALTH_SERVER_EFFECTIVE_FAULT_ARRAY_SIZE];
    UINT32     index;
    HEALTH_SERVER_TRC(
        "[HEALTH_SERVER] Callback. Opcode 0x%04X\n", opcode);
    #ifdef HEALTH_SERVER_DEBUG
    HEALTH_SERVER_debug_dump_bytes(data_param, data_len);
    #else
    MS_IGNORE_UNUSED_PARAM(data_len);
    #endif /* HEALTH_SERVER_DEBUG */
    /* Search corresponding Health Model entity */
    index = health_server_search_entity(handle);

    if (MS_CONFIG_LIMITS(MS_HEALTH_SERVER_MAX) <= index)
    {
        retval = HEALTH_CONTEXT_ASSERT_FAILED;
        HEALTH_SERVER_ERR(
            "[HEALTH] No matching Health Server entity. Returning: 0x%04X\n", retval);
        return retval;
    }

    #if 0

    /* Check if the packet is decrypted using device key */
    if (MS_MAX_APPS > appkey_handle)
    {
        HEALTH_SERVER_ERR("[HEALTH] Health Model Packets shall use Device Key. Dropping..\n");
        return API_FAILURE;
    }

    #endif /*0 */
    retval = API_SUCCESS;
    marker = 0;
    /* Set Invalid Response Code */
    response = 0xFFFFFFFF;

    switch(opcode)
    {
    case MS_ACCESS_HEALTH_FAULT_GET_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_FAULT_GET_OPCODE\n");
        MS_UNPACK_LE_2_BYTE(&company_id, &data_param[marker]);
        HEALTH_SERVER_TRC(
            "Company ID: 0x%04X\n", company_id);
        /* Get Fault Status */
        marker = sizeof(status);
        health_server_get_fault_status(index, opcode, company_id, status, &marker, &response);
        MODEL_OPCODE_HANDLER_CALL(health_fault_get_handler);
    }
    break;

    case MS_ACCESS_HEALTH_FAULT_CLEAR_UNACKNOWLEDGED_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_FAULT_CLEAR_UNACKNOWLEDGED_OPCODE\n");
        MS_UNPACK_LE_2_BYTE(&company_id, &data_param[marker]);
        HEALTH_SERVER_TRC(
            "Company ID: 0x%04X\n", company_id);
        /* Clear Fault */
        health_server_clear_fault(index, company_id);
        MODEL_OPCODE_HANDLER_CALL(health_fault_clear_unacknowledged_handler);
    }
    break;

    case MS_ACCESS_HEALTH_FAULT_CLEAR_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_FAULT_CLEAR_OPCODE\n");
        MS_UNPACK_LE_2_BYTE(&company_id, &data_param[marker]);
        HEALTH_SERVER_TRC(
            "Company ID: 0x%04X\n", company_id);
        /* Clear Fault */
        health_server_clear_fault(index, company_id);
        /* Get Fault Status */
        marker = sizeof(status);
        health_server_get_fault_status(index, MS_ACCESS_HEALTH_FAULT_GET_OPCODE, company_id, status, &marker, &response);
        MODEL_OPCODE_HANDLER_CALL(health_fault_clear_handler);
    }
    break;

    case MS_ACCESS_HEALTH_FAULT_TEST_UNACKNOWLEDGED_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_FAULT_TEST_UNACKNOWLEDGED_OPCODE\n");
        test_id = data_param[marker];
        marker += 1;
        MS_UNPACK_LE_2_BYTE(&company_id, &data_param[marker]);
        HEALTH_SERVER_TRC(
            "Test ID: 0x%02X, Company ID: 0x%04X\n", test_id, company_id);
        /* Run Self Test */
        retval = health_server_run_self_test(index, test_id, company_id);
        MODEL_OPCODE_HANDLER_CALL(health_fault_test_unacknowledged_handler);
    }
    break;

    case MS_ACCESS_HEALTH_FAULT_TEST_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_FAULT_TEST_OPCODE\n");
        test_id = data_param[marker];
        marker += 1;
        MS_UNPACK_LE_2_BYTE(&company_id, &data_param[marker]);
        HEALTH_SERVER_TRC(
            "Test ID: 0x%02X, Company ID: 0x%04X\n", test_id, company_id);
        /* Run Self Test */
        retval = health_server_run_self_test(index, test_id, company_id);

        if (API_SUCCESS == retval)
        {
            /* Get Fault Status */
            marker = sizeof(status);
            health_server_get_fault_status(index, MS_ACCESS_HEALTH_FAULT_GET_OPCODE, company_id, status, &marker, &response);
        }

        MODEL_OPCODE_HANDLER_CALL(health_fault_test_handler);
    }
    break;

    case MS_ACCESS_HEALTH_PERIOD_GET_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_PERIOD_GET_OPCODE\n");
        /* Get Period Divisor */
        health_server_get_period_divisor(index, &fast_period_div, &response);
        status[marker] = fast_period_div;
        marker += 1;
        MODEL_OPCODE_HANDLER_CALL(health_period_get_handler);
    }
    break;

    case MS_ACCESS_HEALTH_PERIOD_SET_UNACKNOWLEDGED_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_PERIOD_SET_UNACKNOWLEDGED_OPCODE\n");
        fast_period_div = data_param[marker];
        HEALTH_SERVER_TRC(
            "FastPeriodDivisor: 0x%02X\n", fast_period_div);
        /* Set Period Divisor */
        health_server_set_period_divisor(index, fast_period_div);
        MODEL_OPCODE_HANDLER_CALL(health_period_set_unacknowledged_handler);
    }
    break;

    case MS_ACCESS_HEALTH_PERIOD_SET_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_PERIOD_SET_OPCODE\n");
        fast_period_div = data_param[marker];
        HEALTH_SERVER_TRC(
            "FastPeriodDivisor: 0x%02X\n", fast_period_div);
        /* Set Period Divisor */
        health_server_set_period_divisor(index, fast_period_div);
        /* Get Period Divisor */
        health_server_get_period_divisor(index, &fast_period_div, &response);
        status[marker] = fast_period_div;
        marker += 1;
        MODEL_OPCODE_HANDLER_CALL(health_period_set_handler);
    }
    break;

    case MS_ACCESS_HEALTH_ATTENTION_GET_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_ATTENTION_GET_OPCODE\n");
        /* Get Attention */
        health_server_get_attention(index, &attention, &response);
        status[marker] = attention;
        marker += 1;
        MODEL_OPCODE_HANDLER_CALL(health_attention_get_handler);
    }
    break;

    case MS_ACCESS_HEALTH_ATTENTION_SET_UNACKNOWLEDGED_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_ATTENTION_SET_UNACKNOWLEDGED_OPCODE\n");
        attention = data_param[marker];
        HEALTH_SERVER_TRC(
            "Attention: 0x%02X\n", attention);
        health_server_set_attention(index, attention);
        MODEL_OPCODE_HANDLER_CALL(health_attention_set_unacknowledged_handler);
    }
    break;

    case MS_ACCESS_HEALTH_ATTENTION_SET_OPCODE:
    {
        HEALTH_SERVER_TRC(
            "MS_ACCESS_HEALTH_ATTENTION_SET_OPCODE\n");
        attention = data_param[marker];
        HEALTH_SERVER_TRC(
            "Attention: 0x%02X\n", attention);
        /* Set Attention */
        health_server_set_attention(index, attention);
        /* Get Attention */
        health_server_get_attention(index, &attention, &response);
        status[marker] = attention;
        marker += 1;
        MODEL_OPCODE_HANDLER_CALL(health_attention_set_handler);
    }
    break;
    }

    /* Check if to be responded */
    if (0xFFFFFFFF != response)
    {
        /* Send response */
        retval = MS_access_reply
                 (
                     handle,
                     daddr,
                     saddr,
                     subnet_handle,
                     appkey_handle,
                     ACCESS_INVALID_DEFAULT_TTL,
                     response,
                     status,
                     marker
                 );
    }

    return retval;
}

/**
    \brief Access Layer Model Publication Timeout Callback.

    \par Description
    Access Layer calls the registered callback to indicate Publication Timeout
    for the associated model.

    \param [in]  handle        Model Handle.
    \param [out] blob          Blob if any or NULL.
*/
API_RESULT health_server_publish_timout_cb
(
    /* IN */ MS_ACCESS_MODEL_HANDLE* handle,
    /* IN */ void*                    blob
)
{
    UINT8      status[MS_HEALTH_SERVER_EFFECTIVE_FAULT_ARRAY_SIZE];
    UINT16     marker;
    UINT32     response;
    API_RESULT retval;
    UINT32     server_index;
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(blob);
    HEALTH_SERVER_TRC("[Health Server] Publication Callback\n");
    marker = sizeof(status);
    server_index = 0;
    health_server_get_fault_status
    (
        server_index,
        MS_ACCESS_HEALTH_CURRENT_STATUS_OPCODE,
        health_server[server_index].company_id,
        status,
        &marker,
        &response
    );
    /**
        If fast period callback and no fault do not publish.
        Otherwise publish.
    */
    health_server[server_index].fast_step_counter++;
    #if 0

    if ((health_server[server_index].fast_step_counter < (1 << health_server[server_index].period_divisor)) &&
            (3 == marker))
    {
        return API_SUCCESS;
    }

    #endif /* 0 */

    if (health_server[server_index].fast_step_counter >= (1 << health_server[server_index].period_divisor))
    {
        health_server[server_index].fast_step_counter = 0;
    }
    else if (3 == marker)
    {
        return API_SUCCESS;
    }

    retval = MS_access_publish
             (
                 &health_server_model_handle,
                 MS_ACCESS_HEALTH_CURRENT_STATUS_OPCODE,
                 status,
                 marker,
                 MS_FALSE
             );
    return retval;
}
