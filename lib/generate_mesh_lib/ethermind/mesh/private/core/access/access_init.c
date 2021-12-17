
/**
    \file access_init.c

    Module initialization routine and tables defined here.

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "access_internal.h"
#include "access.h"
#include "net_extern.h"
#include "MS_net_api.h"


#ifdef MS_ACCESS
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */
MS_DEFINE_MUTEX(access_mutex);

/** Element List */
MS_DEFINE_GLOBAL_ARRAY(MS_ACCESS_ELEMENT_TYPE, ms_access_element_list, MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT));

/** Model List */
MS_DEFINE_GLOBAL_ARRAY(MS_ACCESS_MODEL_TYPE, ms_access_model_list, MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT));

/** Composition data page 0 header */
MS_ACCESS_COMPOSITION_DATA_PAGE_0_HDR composition_data_hdr;

#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
    /* Periodic State Timers */
    static MS_ACCESS_PERIODIC_STEP_TIMER_TYPE  ms_periodic_step_timers[MS_MAX_NUM_PERIODIC_STEP_TIMERS];
    static MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* ms_periodic_step_timer_start;

    /* Periodic Step related Timer Handle and Counter */
    static EM_timer_handle  ms_period_step_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    static UINT32           ms_period_step_timer_counter;


#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

UINT8       rx_test_ttl;
UINT8       vendor_tid;


/* --------------------------------------------- Static Global Variables */
#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
static void ms_access_add_periodic_step_timer_entity
(
    MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* new_timer
);
#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

/* --------------------------------------------- Functions */

/**
    \brief Initializes Module.

    \par Description Initializes Module tables.
*/
void ms_access_init (void)
{
    API_RESULT retval;
    ACCESS_TRC
    ("[ACCESS]: Initializing ACCESS..");
    /* Module Mutex Initialization */
    MS_MUTEX_INIT_VOID (access_mutex, ACCESS);
    /* Init global data structures */
    MS_INIT_GLOBAL_ARRAY(MS_ACCESS_ELEMENT_TYPE, ms_access_element_list, MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT), 0x00);
    MS_INIT_GLOBAL_ARRAY(MS_ACCESS_MODEL_TYPE, ms_access_model_list, MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT), 0x00);
    #ifdef MS_HAVE_DYNAMIC_CONFIG
    /* CID */
    composition_data_hdr.ms_access_cid = ms_global_config.config_MS_DEFAULT_COMPANY_ID;
    /* PID */
    composition_data_hdr.ms_access_pid = ms_global_config.config_MS_DEFAULT_PID;
    /* VID */
    composition_data_hdr.ms_access_vid = ms_global_config.config_MS_DEFAULT_VID;
    /* CRPL */
    composition_data_hdr.ms_access_crpl = ms_global_config.config_MS_REPLAY_CACHE_SIZE;
    #else   //default value(default by hq)
    /* CID */
    composition_data_hdr.ms_access_cid = MS_CONFIG_LIMITS(MS_DEFAULT_COMPANY_ID);
    /* PID */
    composition_data_hdr.ms_access_pid = MS_CONFIG_LIMITS(MS_DEFAULT_PID);
    /* VID */
    composition_data_hdr.ms_access_vid = MS_CONFIG_LIMITS(MS_DEFAULT_VID);
    /* CRPL */
    composition_data_hdr.ms_access_crpl = MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE);
    #endif
    /* Initialize Supported Features (Node Capabilities) */
    ms_access_init_supported_features();
    /* Initialize Configuration Manager */
    ms_access_cm_init();
    /* Periodic Step Timer initialization */
    MS_ACCESS_INIT_PERIODIC_STEP_TIMER();
    /* Register with the Upper Transport layer */
    MS_trn_register(access_pkt_in, MS_TRN_ACCESS_PKT);
    /* Load information from Persistent Storage */
    retval = MS_access_ps_crc_check();

    if(retval == API_SUCCESS)
        ms_access_ps_load(MS_PS_ACCESS_ALL_RECORDS);
    else
        printf("crc check failed\n");

    #ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
    /* Reset Periodic Timer Handles */
    {
        UINT32 index;

        /* Store fixed and configurable parts separately */
        for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); index++)
        {
            ms_access_model_list[index].fixed.publish.timer_handle = NULL;
        }
    }
    #endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */
}

/**
    \brief Initializes Supported Features.

    \par Description
    This routine initializes supported feautures, which can be one or more
    of the following
    - Relay, Proxy, Friend, Low Power
*/
void ms_access_init_supported_features(void)
{
    composition_data_hdr.ms_access_features = 0x00;
    #ifdef MS_RELAY_SUPPORT
    composition_data_hdr.ms_access_features |= (1 << MS_FEATURE_RELAY);
    #endif /* MS_RELAY_SUPPORT */
    #ifdef MS_PROXY_SUPPORT
    composition_data_hdr.ms_access_features |= (1 << MS_FEATURE_PROXY);
    #endif /* MS_PROXY_SUPPORT */
    #ifdef MS_FRIEND_SUPPORT
    composition_data_hdr.ms_access_features |= (1 << MS_FEATURE_FRIEND);
    #endif /* MS_FRIEND_SUPPORT */
    #ifdef MS_LPN_SUPPORT
    composition_data_hdr.ms_access_features |= (1 << MS_FEATURE_LPN);
    #endif /* MS_LPN_SUPPORT */
}

#ifndef MS_NO_SHUTDOWN
/**
    \par Description:
    This function is the shutdown handler for Transport module, and it
    performs bluetooth specific shutdown for the module - currently,
    nothing is done here.
*/
void ms_access_shutdown (void)
{
    ACCESS_TRC (
        "[ACCESS]: Transport Shutdown Successful.");
    /* Store information to Persistent Storage */
    ms_access_ps_store(MS_PS_ACCESS_ALL_RECORDS);
}
#endif /* MS_NO_SHUTDOWN */

/**
    \par Description
    This function handles the incoming data received over upper transport layer.

    \param [in] net_hdr
           Received Network Packet Header
    \param [in] subnet_handle
           Handle identifying associated subnet on which the packet is received
    \param [in] appkey_handle
           Handle identifying application key associated with the received packet
    \param [in] pdata
           The incoming Data Packet
    \param [in] pdatalen
           Size of the incoming Data Packet
*/
void access_pkt_in
(
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ MS_APPKEY_HANDLE    appkey_handle,
    /* IN */ UCHAR*              pdata,
    /* IN */ UINT16              pdatalen
)
{
    /* Upper Layer data and datalen */
    UCHAR* udata;
    UINT16  udatalen;
    UINT32  opcode;
    UCHAR   opcode_offset;
    UCHAR   process_opcode;
    ACCESS_TRC(
        "[ACCESS]: Upper transport Callback.\n");
    ACCESS_debug_dump_bytes(pdata, pdatalen);
    /* Strip headers if any and pass it to the registered layer above */
    process_opcode = MS_TRUE;

    /* Extract Opcode */
    if (0x00 == (0x80 & pdata[0]))
    {
        opcode = pdata[0];
        opcode_offset = 1;
    }
    else if (0x80 == (0xC0 & pdata[0]))
    {
        /* MS_UNPACK_LE_2_BYTE(&opcode, &pdata[0]); */
        /**
            Access layer is little endian.
            Unpacking the opcode in big endian format to match Opcode macro defines.
        */
        opcode = pdata[0];
        opcode = opcode << 8;
        opcode |= pdata[1];
        opcode_offset = 2;
    }
    else if (0xC0 == (0xC0 & pdata[0]))
    {
        /* MS_UNPACK_LE_3_BYTE(&opcode, &pdata[0]); */
        /**
            Access layer is little endian.
            Unpacking the opcode in big endian format to match Opcode macro defines.
        */
        opcode = pdata[0];
        opcode = opcode << 8;
        opcode |= pdata[1];
        opcode = opcode << 8;
        opcode |= pdata[2];
        opcode_offset = 3;
    }
    else
    {
        ACCESS_TRC(
            "[ACCESS] Unknown Opcode. Dropping...\n");
        process_opcode = MS_FALSE;
        /* To keep some compilers happy */
        opcode = MS_ACCESS_INVALID_OPCODE;
        opcode_offset = 0;
    }

    if (MS_TRUE == process_opcode)
    {
        /* Search for the Opcode in all the registered model and call associated callback */
        udata = pdata + opcode_offset;
        udatalen = pdatalen - opcode_offset;
        ms_access_handle_rx_opcode
        (
            net_hdr,
            subnet_handle,
            appkey_handle,
            opcode,
            udata,
            udatalen
        );
    }
}

/* Function to allocate a free element */
API_RESULT ms_access_allocate_element(/* OUT */ MS_ACCESS_ELEMENT_HANDLE*    handle)
{
    API_RESULT retval;
    UINT32     index;
    retval = API_SUCCESS;

    /* Search for a free element */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT); index++)
    {
        if (0 == ms_access_element_list[index].valid)
        {
            /* Mark as allocated */
            ms_access_element_list[index].valid = 0x01;
            *handle = (MS_ACCESS_ELEMENT_HANDLE)index;
            break;
        }
    }

    /* See if could find a free element */
    if (MS_CONFIG_LIMITS(MS_ACCESS_ELEMENT_COUNT) == index)
    {
        /* TODO: Appropriate status */
        retval = API_FAILURE;
    }

    return retval;
}

/* Function to search and allocate a free model */
API_RESULT ms_access_search_and_allocate_model
(
    /* IN */  MS_ACCESS_MODEL*    model,
    /* OUT */ UINT16*             id
)
{
    API_RESULT retval;
    UINT32     index, saved_index;
    saved_index = MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT);

    /**
        From Mesh Specification :-
        An element is not allowed to contain multiple instances of models that
        use the same message (for example, an 'On' message). When multiple models
        within the same element use the same message, the models are said to 'overlap'.
        To implement multiple instances of overlapping models within a single node
        (for example, to control multiple light fixtures that can be turned on and off),
        the node is required to contain multiple elements.
    */

    /* Search for a free model */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); index++)
    {
        if (0 == ms_access_model_list[index].fixed.valid)
        {
            if (MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT) == saved_index)
            {
                saved_index = index;
            }
        }
        else
        {
            if ((ms_access_model_list[index].fixed.model_id.id == model->model_id.id) &&
                    (ms_access_model_list[index].fixed.model_id.type == model->model_id.type) &&
                    (ms_access_model_list[index].fixed.elem_handle == model->elem_handle))
            {
                ACCESS_ERR(
                    "[ACCESS]: Model ID: 0x%08X, Type: 0x%02X is already registerd for Element Hdl: 0x%04X.\n",
                    model->model_id.id, model->model_id.type, model->elem_handle);
                return ACCESS_MODEL_ALREADY_REGISTERED;
            }
        }
    }

    /* See if could find a free model */
    if (MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT) == saved_index)
    {
        /* TODO: Appropriate status */
        retval = API_FAILURE;
    }
    else
    {
        /* Mark as allocated */
        ms_access_model_list[saved_index].fixed.valid = 0x01;
        *id = (UINT16)saved_index;
        ACCESS_TRC(
            "[ACCESS 0x%04X]: Allocated Model ID: 0x%08X, Type: 0x%02X for Element Hdl: 0x%04X.\n",
            model->model_id.id, model->model_id.type, model->elem_handle, (*id));
        retval = API_SUCCESS;
    }

    return retval;
}

/* Handle received Opcode */
void ms_access_handle_rx_opcode
(
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ MS_APPKEY_HANDLE    appkey_handle,
    /* IN */ UINT32              opcode,
    /* IN */ UCHAR*              udata,
    /* IN */ UINT16              udatalen
)
{
    MS_ACCESS_MODEL_HANDLE     model_index;
    ACCESS_TRC(
        "[ACCESS]: Handle Rx Opcode:0x%08X, SNH:0x%04X, AKH:0x%04X.\n",
        opcode, subnet_handle, appkey_handle);
    rx_test_ttl = net_hdr->ttl;
//    printf (
//    "[ACCESS]: Handle Rx Opcode:0x%08X, SNH:0x%04X, AKH:0x%04X.\n",
//    opcode, subnet_handle, appkey_handle);

    /* Look into all the registered models */
    for (model_index = 0; model_index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); model_index++)
    {
        /**
            Check
            - if Model is not free, and
            - if the Model is bound with the AppKey
             (Get AppKey index in AppKey bitarray)
        */
        if ((0 != ms_access_model_list[model_index].fixed.valid) &&
                ((MS_CONFIG_LIMITS(MS_MAX_APPS) <= appkey_handle) ||
                 (0 != bitarray_get_bit(ms_access_model_list[model_index].config.appkey_bitarray, appkey_handle))))
        {
            /* Search for received opcode in the opcode list associated with model */
            if(API_SUCCESS == ms_access_is_opcode_in_model(opcode, model_index))
            {
                MS_ACCESS_MODEL_CB  cb;
                MS_NET_ADDR         l_addr;
                /* Check for corresponding element match */
                MS_access_cm_get_primary_unicast_address(&l_addr);

                /**
                    If Destination Address is Unicast Address or Fixed Group Address,
                    there can be only one match
                */
                if ((ms_access_model_list[model_index].fixed.elem_handle == (net_hdr->daddr - l_addr))
                        ||((MS_NET_ADDR_ALL_PROXIES <= net_hdr->daddr) && (0 ==
                                                                           ms_access_model_list[model_index].fixed.elem_handle)))
                {
                    /* If match found, call the associated callback */
                    cb = ms_access_model_list[model_index].fixed.cb;

                    if (NULL != cb)
                    {
                        cb
                        (
                            &model_index,
                            net_hdr->saddr,
                            net_hdr->daddr,
                            subnet_handle,
                            appkey_handle,
                            opcode,
                            udata,
                            udatalen
                        );
                    }

                    break;
                }
                /**
                    If Subscription Address (Virtual or Group Address),
                    there can be multiple matches
                */
                else
                {
                    /* Destination Address Types */
                    UINT8           daddr_type;
                    API_RESULT                retval;
                    MS_ACCESS_ADDRESS_HANDLE  addr_handle;
                    daddr_type = MS_net_get_address_type(net_hdr->daddr);
                    /* If Virtual Address */
                    retval = API_FAILURE;

                    if (MS_NET_ADDR_TYPE_VIRTUAL == daddr_type)
                    {
                        /* Search in virtual address table */
                        retval = ms_search_virtual_address
                                 (
                                     &net_hdr->daddr,
                                     MS_ADDR_OP_TYPE_SUBSCRIBE,
                                     &addr_handle
                                 );
                    }
                    /* Else if Group Address */
                    else if (MS_NET_ADDR_TYPE_GROUP == daddr_type)
                    {
                        /* Look into non-virtual address table */
                        retval = ms_search_address
                                 (
                                     &net_hdr->daddr,
                                     MS_ADDR_OP_TYPE_SUBSCRIBE,
                                     &addr_handle
                                 );
                    }

                    if (API_SUCCESS == retval)
                    {
                        /* If the Model is subscribed to this destination address */
                        if (0 != bitarray_get_bit(ms_access_model_list[model_index].config.subscription_bitarray, addr_handle))
                        {
                            /* If match found, call the associated callback */
                            cb = ms_access_model_list[model_index].fixed.cb;

                            if (NULL != cb)
                            {
                                cb
                                (
                                    &model_index,
                                    net_hdr->saddr,
                                    net_hdr->daddr,
                                    subnet_handle,
                                    appkey_handle,
                                    opcode,
                                    udata,
                                    udatalen
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

/* Search for opcode in the opcode list associated with model */
API_RESULT ms_access_is_opcode_in_model
(
    /* IN */ UINT32  opcode,
    /* IN */ UINT32  model_index
)
{
    API_RESULT          retval;
    UINT32              s_index;
    UINT16              num_opcodes;
    DECL_CONST UINT32*    opcodes;
    retval = API_FAILURE;
    /* Number of Opcodes */
    num_opcodes = ms_access_model_list[model_index].fixed.num_opcodes;
    /* List of Opcodes */
    opcodes = ms_access_model_list[model_index].fixed.opcodes;

    for (s_index = 0; s_index < num_opcodes; s_index ++)
    {
        if (opcode == opcodes[s_index])
        {
            retval = API_SUCCESS;
            break;
        }
    }

    return retval;
}

void ms_access_get_element_models
(
    /* IN */    UINT32   element_index,
    /* OUT */   UINT32* model_indices,
    /* INOUT */ UINT32* model_count
)
{
    UINT32 model_index;
    UINT32 match_count;
    /**
        Scan through the model list and check which of those
        belong to the element of interest
    */
    match_count = 0;

    for (model_index = 0; model_index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); model_index++)
    {
        /* If Model is not free */
        if ((0 != ms_access_model_list[model_index].fixed.valid) &&
                (element_index == ms_access_model_list[model_index].fixed.elem_handle))
        {
            /* Save Model Index in out parameter */
            model_indices[match_count] = model_index;
            match_count ++;

            if (match_count == (*model_count))
            {
                break;
            }
        }
    }

    /* Return matched count */
    *model_count = match_count;
}

#ifdef MS_ACCESS_PUBLISH_TIMER_SUPPORT
/**
    Period Step Timer related routines.
    These interfaces are used for Periodic Publishing and Friend Poll.
*/

/* Periodic Step Timer - Initialization Routine */
API_RESULT ms_access_init_periodic_step_timer(void)
{
    EM_stop_timer(&ms_period_step_timer_handle);
    EM_mem_set(ms_periodic_step_timers, 0, sizeof(ms_periodic_step_timers));
    ms_periodic_step_timer_start = NULL;
//    ms_period_step_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    ms_period_step_timer_counter = 0;
    return API_SUCCESS;
}

/* Utility Function */
static void ms_access_convert_publish_period_to_100ms_step
(
    /* IN */  UINT8    period,
    /* IN */  UINT8    period_divisor,
    /* OUT */ UINT32* step_count
)
{
    UINT8 step_resolution, num_of_steps;
    step_resolution = period >> 6;
    num_of_steps = period & 0x3F;

    switch (step_resolution)
    {
    case 0x00: /* 100ms resolution */
        *step_count = num_of_steps;
        break;

    case 0x01: /* 1s resolution */
        *step_count = (num_of_steps * 10);
        break;

    case 0x02: /* 10s resolution */
        *step_count = (num_of_steps * 10 * 10);
        break;

    case 0x03: /* 10m resolution */
        *step_count = (num_of_steps * 10 * 10 * 60);
        break;

    default:
        /* Not reachable code */
        break;
    }

    /* Apply Period Divisor */
    *step_count = ((*step_count) >> period_divisor);
}

/* Periodic Timeout Handler */
static void ms_access_period_step_timer_handler(void* args, UINT16 size)
{
    UINT32 step_count;
    MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* cur_timer;
    MS_IGNORE_UNUSED_PARAM(size);
    step_count = (*((UINT32*)args));
    ACCESS_TRC(
        "[ACCESS] Step Timeout Handler. Step Count: 0x%08X\n", step_count);
    ms_period_step_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Update Cumulative Step Counter */
    ms_period_step_timer_counter += step_count;
    /* Head Element */
    cur_timer = ms_periodic_step_timer_start;
    ACCESS_TRC(
        "[ACCESS] Step Timeout Handler. Head: %p\n", cur_timer);

    /* Check if any of the timer expired */
    while (NULL != cur_timer)
    {
        ACCESS_TRC(
            "[ACCESS] Step Timeout Handler. Step Timer Counter: 0x%08X, Expiry_Count: 0x%08X\n",
            ms_period_step_timer_counter, cur_timer->expiry_counter);

        /* Check if the */
        if (ms_period_step_timer_counter >= cur_timer->expiry_counter)
        {
            /* ACCESS_TRC("[Step Timeout] Insert to the timer queue again\n"); */
            /* Callback */
            /* TODO: Check not NULL */
//            ms_access_model_list[cur_timer->handle].fixed.pub_cb(&cur_timer->handle, NULL);
            /* Remove the entry and re-insert */
            #if 0
            cur_timer->step_count = 0;
            /* TODO: Set invalid model handle */
            cur_timer->handle = 0xFFFF;
            cur_timer->expiry_counter = 0;
            #endif /* 0 */
            ms_periodic_step_timer_start = cur_timer->next;
            cur_timer->next = NULL;
            ms_access_add_periodic_step_timer_entity(cur_timer);
        }
        else
        {
            /* ACCESS_TRC("[Step Timeout] Not Insert to the timer queue again\n"); */
            break;
        }

        /* Move to next */
        cur_timer = cur_timer->next;
    }

    return;
}

static void ms_access_add_periodic_step_timer_entity
(
    MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* new_timer
)
{
    MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* current_timer;
    MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* prev_timer;
    API_RESULT retval;
    ACCESS_TRC(
        "[ACCESS] Add Step Timer. %p, Step_Count:0x%08X\n", new_timer, new_timer->step_count);

    /* If Timer list is empty */
    if (NULL == ms_periodic_step_timer_start)
    {
        ms_periodic_step_timer_start = new_timer;
        /* Set Expiry Counter */
        new_timer->expiry_counter = ms_period_step_timer_counter + new_timer->step_count;
        /* Start the periodic timer - 100ms */
        retval = EM_start_timer
                 (
                     &ms_period_step_timer_handle,
                     (EM_TIMEOUT_MILLISEC | (100 * new_timer->step_count)),
                     ms_access_period_step_timer_handler,
                     (void*)&new_timer->step_count,
                     sizeof(new_timer->step_count)
                 );
        /* ACCESS_TRC("[Add Step Timer] To Head. Retval: 0x%04X\n", retval); */
    }
    else
    {
        UINT32          remaining_time_ms;
        retval = EM_timer_get_remaining_time
                 (
                     ms_period_step_timer_handle,
                     &remaining_time_ms
                 );

        /**
            In case the timer has just expired, this API can return failure.
            So take careo that treating the remaining time as ZERO.
        */
        if (API_SUCCESS != retval)
        {
            remaining_time_ms = 0;
        }

        /* Head Element */
        current_timer = ms_periodic_step_timer_start;
        /* Set Expiry Counter */
        /* TODO: Take care of the wrap around conditions */
        new_timer->expiry_counter =
            current_timer->expiry_counter - ((remaining_time_ms + 99) / 100) + new_timer->step_count;

        /* Check if the current running timer is going to take longer time than the new one */
        if ((new_timer->step_count * 100) <= remaining_time_ms)
        {
            ACCESS_TRC(
                "[ACCESS] Add Step Timer. Stopping Current Timer\n");
            /* Stop the current timer and start with a new timeout */
            EM_stop_timer(&ms_period_step_timer_handle);
            new_timer->next = current_timer;
            retval = EM_start_timer
                     (
                         &ms_period_step_timer_handle,
                         (EM_TIMEOUT_MILLISEC | (100 * new_timer->step_count)),
                         ms_access_period_step_timer_handler,
                         (void*)&new_timer->step_count,
                         sizeof(new_timer->step_count)
                     );
            /* ACCESS_TRC("[Add Step Timer] Restarting Timer. Retval: 0x%04X\n", retval); */
        }
        else
        {
            prev_timer = current_timer;

            while (NULL != current_timer->next)
            {
                /* Check if the */
                if (current_timer->expiry_counter > new_timer->expiry_counter)
                {
                    break;
                }

                /* Move to next */
                current_timer = current_timer->next;
            }

            /* Insert */
            prev_timer->next = new_timer;
            new_timer->next = current_timer->next;
            ACCESS_TRC(
                "[ACCESS] Add Step Timer. Inserting Timer in the Queue\n");
        }
    }
}

/* Periodic Step Timer - Start Routine */
API_RESULT ms_access_start_periodic_step_timer
(
    /* IN */  UINT8                                  period,
    /* IN */  UINT8                                  period_divisor,
    /* IN */  MS_ACCESS_MODEL_HANDLE*                handle,
    /* OUT */ MS_ACCESS_PERIODIC_STEP_TIMER_HANDLE* timer_handle
)
{
    API_RESULT retval;
    UINT16 index;
    MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* new_timer;
    UINT32 step_count;
    ACCESS_TRC(
        "[ACCESS] Enter Start Periodic Timer\n");

    /* Parameter Validation */
    if ((0 == period) || (NULL == handle) || (NULL == timer_handle))
    {
        ACCESS_ERR(
            "[ACCESS] Invalid Parameters. Returning.\n");
        /* TODO: return specific error */
        return API_FAILURE;
    }

    *timer_handle = NULL;
    /* Lock */
    /* Get a free entry */
    new_timer = NULL;

    for (index = 0; index < MS_MAX_NUM_PERIODIC_STEP_TIMERS; index++)
    {
        if (0 == ms_periodic_step_timers[index].step_count)
        {
            /* Save Periodic Callback, Blob etc. */
            ms_access_convert_publish_period_to_100ms_step
            (
                period,
                period_divisor,
                &step_count
            );
            new_timer = &ms_periodic_step_timers[index];
            new_timer->step_count = step_count;
            new_timer->handle = *handle;
            new_timer->next = NULL;
            break;
        }
    }

    /* Insert entry based on the step_count */
    if ((NULL != new_timer) && (0 != step_count))
    {
        ACCESS_TRC(
            "[ACCESS] Add periodic step timer entity.\n");
        ms_access_add_periodic_step_timer_entity(new_timer);
        *timer_handle = new_timer;
        retval = API_SUCCESS;
    }
    else
    {
        ACCESS_ERR(
            "[ACCESS] Failed to find free entity or Step Count is ZERO. Returning.\n");
        /* TODO: return specific error */
        retval = API_FAILURE;
    }

    /* Unlock */
    ACCESS_TRC(
        "[ACCESS] Exit Start Periodic Timer\n");
    return API_SUCCESS;
}

/* Periodic Step Timer - Stop Routine */
API_RESULT ms_access_stop_periodic_step_timer
(
    /* IN */ MS_ACCESS_PERIODIC_STEP_TIMER_HANDLE timer_handle
)
{
    MS_ACCESS_PERIODIC_STEP_TIMER_TYPE* cur_timer;
    ACCESS_TRC(
        "[ACCESS] Enter Stop Periodic Timer\n");

    /* Check if the timer handle is valid */
    if (NULL == timer_handle)
    {
        /* TODO: Add error log */
        return API_FAILURE;
    }

    /* Lock */
    /* If it is the 'head' */
    cur_timer = ms_periodic_step_timer_start;

    if (cur_timer == timer_handle)
    {
        /* If there are no more entries, stop the timer */
        if (NULL == cur_timer->next)
        {
            EM_stop_timer(&ms_period_step_timer_handle);
            ms_period_step_timer_counter = 0;
        }

        /**
            Else, do not stop the timer.
            On timeout, it is possible that there is no associated
            entries in the periodic timer list and timer to be restarted
            for the 'head' entry.
        */
        {
            cur_timer->step_count = 0;
            /* TODO: Set invalid model handle */
            cur_timer->handle = 0xFFFF;
            cur_timer->expiry_counter = 0;
            ms_periodic_step_timer_start = cur_timer->next;
            cur_timer->next = NULL;
        }
    }
    /* else */
    else
    {
        /* Adjust the next and previous entries */
        /* Mark the associatied timer invalid */
        while (NULL != cur_timer->next)
        {
            if (cur_timer->next == timer_handle)
            {
                cur_timer->next = timer_handle->next;
                timer_handle->step_count = 0;
                /* TODO: Set invalid model handle */
                timer_handle->handle = 0xFFFF;
                timer_handle->expiry_counter = 0;
                timer_handle->next = NULL;
            }
        }
    }

    /* Unlock */
    ACCESS_TRC(
        "[ACCESS] Exit Stop Periodic Timer\n");
    return API_SUCCESS;
}
#endif /* MS_ACCESS_PUBLISH_TIMER_SUPPORT */

#endif /* MS_ACCESS */

