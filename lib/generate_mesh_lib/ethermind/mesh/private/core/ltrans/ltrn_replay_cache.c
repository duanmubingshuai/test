/**
    \file ltrn_replay_cache.c

    Replay Protection List related routines.
    This is used to protect against replay attacks.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "ltrn_internal.h"
#include "ltrn_extern.h"
#include "net_extern.h"
#include "net_internal.h"
#include "access_internal.h"
#include "EXT_cbtimer.h"


#ifdef MS_LTRN
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
/** Replay Cache Data Structure */
typedef struct _LTRN_REPLAY_CACHE_ELEMENT
{
    /* Sequence Number */
    UINT32 seq_num;

    /* Source Address */
    MS_NET_ADDR  addr;

    /* IVI */
    UINT8        ivi;

    /* TODO: Age of the entry */

} LTRN_REPLAY_CACHE_ELEMENT;

/**
    Successfully reassembled frames.
    Since IV Index and SeqNum combination can be
    used to uniquely identify a network frame
    originated from a device, for the associated
    LPNs not saving the destination address of
    the reassembled frame.
*/
typedef struct _LTRN_REASSEMBLED_FRAME_INFO
{
    /* Source Address */
    MS_NET_ADDR    saddr;

    /* IVI */
    UINT8          ivi;

    /* SeqAuth */
    UINT32         seq_auth;

    /* Status : 0x00 (Success), 0x01 (Failure) */
    UINT8          status;

} LTRN_REASSEMBLED_FRAME_INFO;

/* Replay Cache */
DECL_STATIC MS_DEFINE_GLOBAL_ARRAY(LTRN_REPLAY_CACHE_ELEMENT, replay_cache, MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE));

/* Reassembled SAR Rx Frames */
DECL_STATIC MS_DEFINE_GLOBAL_ARRAY(LTRN_REASSEMBLED_FRAME_INFO, reassembled_cache, MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE));
DECL_STATIC UINT16 reassembled_cache_start;
DECL_STATIC UINT16 reassembled_cache_size;

/* Segmentation and Reassembly Contexts */
DECL_STATIC MS_DEFINE_GLOBAL_ARRAY(LTRN_SAR_CTX, ltrn_sar_ctx, MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX));

/** Support variables for Testing. TODO: Have under flag */
static UINT8 ltrn_is_replayed = MS_FALSE;

/* --------------------------------------------- Functions */

/**
    \brief Allocation Replay Protection List (if to be created dynamically).

    \par Description
    This routine allocates the Replay Protection List, if to be created dynamically.
*/
void ltrn_alloc_replay_cache(void)
{
    MS_INIT_GLOBAL_ARRAY(LTRN_REPLAY_CACHE_ELEMENT, replay_cache, MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE), 0x00);
    MS_INIT_GLOBAL_ARRAY(LTRN_REASSEMBLED_FRAME_INFO, reassembled_cache, MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE), 0x00);
}


/**
    \brief Initialize Replay Protection List.

    \par Description
    This routine initializes the Replay Protection List.
*/
void ltrn_init_replay_cache (void)
{
    /* Reset Replay Cache */
    reassembled_cache_start = reassembled_cache_size = 0;
    EM_mem_set(replay_cache, 0x00, ((MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE)) * sizeof(LTRN_REPLAY_CACHE_ELEMENT)));
}

/* TODO: Support routine for testing. Have under flag */
void ltrn_set_is_replayed(/* IN */ UINT8 flag)
{
    ltrn_is_replayed = flag; /*  MS_TRUE or MS_FALSE */
}

/**
    \brief To clear all Segmentation and Reassembly Contexts

    \par Description
    This routine clears all Segmentation and Reassembly Contexts.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_ltrn_clear_sar_contexts (void)
{
    UINT32 index;

    /* Initialize Timer Handles */
    for (index = 0; index < MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX); index++)
    {
        ltrn_sar_free_ctx(&ltrn_sar_ctx[index]);
    }

    return API_SUCCESS;
}

/**
    \brief Initialize SAR Contexts.

    \par Description
    This routine initializes the contexts used for Segmentation and Reassembly.
*/
void ltrn_init_sar_contexts (void)
{
    UINT32 index;
    MS_INIT_GLOBAL_ARRAY(LTRN_SAR_CTX, ltrn_sar_ctx, MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX), 0x00);
    EM_mem_set(ltrn_sar_ctx, 0, (sizeof(LTRN_SAR_CTX) * MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX)));

    /* Initialize Timer Handles */
    for (index = 0; index < MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX); index++)
    {
        #ifdef EM_USE_EXT_TIMER
        ltrn_sar_ctx[index].ack_rtx_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
        ltrn_sar_ctx[index].incomplete_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
        #else
        ltrn_sar_ctx[index].ack_rtx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
        ltrn_sar_ctx[index].incomplete_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
        #endif
    }
}

/**
    \brief Check if the message is not a replayed one.

    \par Description
    This routine verifies if the message is not a replayed one.

    \param [in] hdr    Network header of received packet.

    \return    API_SUCCESS - If the message is a replayed one.
               API_FAILURE - If the message is not a replayed one.
*/
API_RESULT ltrn_check_if_replayed
(
    /* IN */ MS_NET_HEADER*  hdr
)
{
    API_RESULT retval;
    UINT32 index;

    if (MS_TRUE == ltrn_is_replayed)
    {
        return API_SUCCESS;
    }

    retval = API_FAILURE;

    /* Search for all valid entries */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE); index++)
    {
        /* Look for address match */
        if (replay_cache[index].addr == hdr->saddr)
        {
            /* TODO: Not handling the scenario of IVI update */
            if ((hdr->ivi == replay_cache[index].ivi) &&
                    (hdr->seq_num <= replay_cache[index].seq_num))
            {
                LTRN_ERR("[LTRN] SRC:0x%04X, Replayed SeqNum:0x%08X <= Last Rxed SeqNum:0x%08X. IVI:0x%02X\n",
                         hdr->saddr, hdr->seq_num, replay_cache[index].seq_num, hdr->ivi);
                retval = API_SUCCESS;
                break;
            }
        }
    }

    return retval;
}

/**
    \brief Add/update message to Replay Protection List.

    \par Description
    This routines adds entries or updates entries in Replay Protection List.
    Check if the message is already in the cache.
    If not, add it to the cache.
    In case case is full, then the oldest message will be removed,
    to make space for this new message.

    \param [in] hdr

    \return API_RESULT
*/
API_RESULT ltrn_update_replay_cache
(
    /* IN */ MS_NET_HEADER*   hdr
)
{
    API_RESULT retval;
    UINT32 index, update_index;
    /* Iterate over all entries. Search for a match and a free space */
    update_index = MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE);
    /* Assume no match found */
    retval = API_FAILURE;

    for (index = 0; index < MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE); index++)
    {
        /* Look for address and IVI match */
        if ((replay_cache[index].addr == hdr->saddr) /*&& (replay_cache[index].ivi == hdr->ivi)*/)
        {
            LTRN_TRC("[LTRN Replay Cache] SRC:0x%04X, IVI:0x%02X present at index 0x%08X. Update SeqNum etc.\n",
                     hdr->saddr, hdr->ivi, index);
            retval = API_SUCCESS;
            break;
        }
        else if ((MS_NET_ADDR_UNASSIGNED == replay_cache[index].addr) &&
                 (MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE) == update_index))
        {
            /* Remember free cache entry */
            update_index = index;
        }
    }

    /* Check if no match found and there is a free entry */
    if (API_SUCCESS != retval)
    {
        /* Replay Cache Full */
        if (MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE) == update_index)
        {
            /* TODO: Reusing the index 0, in the absence of aging information */
//            update_index = 0;
            static UINT8 reply_cache_num = MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE)-1;

            if(reply_cache_num == (MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE)-1))
                reply_cache_num = 0;
            else
                reply_cache_num++;

            update_index = reply_cache_num;
        }

        retval = API_SUCCESS;
    }
    else
    {
        update_index = index;
    }

    /* Save Source, SEQ No. and IVI */
    replay_cache[update_index].addr = hdr->saddr;
    replay_cache[update_index].seq_num = hdr->seq_num;
    replay_cache[update_index].ivi = hdr->ivi;
    LTRN_TRC("[LTRN] Replay Cache Index:0x%08X, SRC:0x%04X, SeqNum:0x%08X, IVI:0x%02X\n",
             update_index, hdr->saddr, hdr->seq_num, hdr->ivi);
    /* TODO: Return value can be used to indicate Cache full condition */
    return retval;
}

API_RESULT ltrn_delete_from_replay_cache
(
    /* IN */ MS_NET_ADDR  addr
)
{
    UINT16  net_index;
    UINT16  i;
    net_index = 0;

    for(i = 0; i < MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE); i++)
    {
        if(replay_cache[net_index].addr == addr)
        {
            replay_cache[net_index].ivi = 0;
            replay_cache[net_index].addr = 0;
            replay_cache[net_index].seq_num = 0;
        }

        net_index ++;
    }

//    MS_INIT_GLOBAL_ARRAY(LTRN_REPLAY_CACHE_ELEMENT, replay_cache, MS_CONFIG_LIMITS(MS_REPLAY_CACHE_SIZE), 0x00);
    return API_SUCCESS;
}

/**
    \brief Check if is already in reassembled sar rx cache.

    \par Description Check if is already in reassembled sar rx cache.

    \param [in]  saddr       Source Address - originator of message
    \param [in]  ivi         IVI present in the message
    \param [in]  seq_auth    SeqAuth of received message
    \param [out] status      Reassembly was success (0x00) or failed/timedout (0x01)

    \return API_RESULT
*/
API_RESULT ltrn_is_in_reassembled_cache
(
    /* IN */  MS_NET_ADDR    saddr,
    /* IN */  UINT8          ivi,
    /* IN */  UINT32         seq_auth,
    /* OUT */ UINT8*         status
)
{
    API_RESULT retval;
    UINT16 index, cur;
    retval = API_FAILURE;
    /* Search for all valid elements */
    LTRN_TRC("[LTRN] In reassembled cache? SRC:0x%04X, IVI:0x%02X, SeqAuth:0x%08X\n",
             saddr, ivi, seq_auth);
    /* Start from the queue start */
    cur = reassembled_cache_start;
    LTRN_INF("[LTRN] Search Start Index 0x%04X\n", cur);

    for (index = 0; index < reassembled_cache_size; index++)
    {
        /* Look for match */
        if ((reassembled_cache[cur].saddr == saddr) &&
                (reassembled_cache[cur].ivi == ivi) &&
                (reassembled_cache[cur].seq_auth == seq_auth))
        {
            LTRN_TRC("[LTRN] Reassembled Cache match found at Index: 0x%04X\n",
                     cur);
            /* Return saved status */
            *status = reassembled_cache[cur].status;
            retval = API_SUCCESS;
            break;
        }

        /* Go to the next element in the queue */
        cur++;

        /* Wrap around condition */
        if (MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE) == cur)
        {
            cur = 0;
        }
    }

    LTRN_TRC("[LTRN] In reassembled cache? returning 0x%04X\n", retval);
    return retval;
}

API_RESULT ltrn_delete_from_reassembled_cache
(
    /* IN */ MS_NET_ADDR  addr
)
{
    UINT16  net_index;
    UINT16  i;
    net_index = reassembled_cache_start;

    for(i = 0; i < reassembled_cache_size; i++)
    {
        if(reassembled_cache[net_index].saddr == addr)
        {
            reassembled_cache[net_index].saddr = 0;
        }

        if(net_index == MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE))
        {
            net_index = 0;
        }
        else
        {
            net_index ++;
        }
    }

    return API_SUCCESS;
}

UINT8 ltrn_get_sar_ctx_count
(
    /* IN */ UINT8          ctx_type
)
{
    UINT16 index;
    UINT8 sar_ctx_count;
    sar_ctx_count = 0;

    /* Look for a free context */
    for (index = 0; index < MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX); index++)
    {
        if (ctx_type == ltrn_sar_ctx[index].type)
        {
            sar_ctx_count++;
        }
    }

    return sar_ctx_count;
}

/**
    \brief To get currently active SAR Context.

    \par Description Get currently active SAR Context.

    \param [in]  ctx_type    SAR Context Type (Rx or Tx)
    \param [in]  saddr       Source Address - originator of message
    \param [in]  ivi         IVI present in the message

    \return If there is correspoding active SAR, pointer to LTRN_SAR_CTX,
            Else, return NULL
*/
LTRN_SAR_CTX* ltrn_get_current_sar_ctx
(
    /* IN */ UINT8          ctx_type,
    /* IN */ MS_NET_ADDR    saddr,
    /* IN */ UINT8          ivi
)
{
    LTRN_SAR_CTX* ctx;
    UINT16 index;
    ctx = NULL;
    /* Search for all valid elements */
    LTRN_TRC("[LTRN] Get Current SAR Context for Ctx_Type: 0x%02X, SRC:0x%04X, IVI:0x%02X\n",
             ctx_type, saddr, ivi);

    /* Look for a free context */
    for (index = 0; index < MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX); index++)
    {
        if ((ctx_type == ltrn_sar_ctx[index].type) &&
                (saddr == ltrn_sar_ctx[index].saddr) &&
                (ivi == ltrn_sar_ctx[index].ivi))
        {
            ctx = &ltrn_sar_ctx[index];
            ctx->type = ctx_type;
            break;
        }
    }

    return ctx;
}
/**
    \brief Add to reassembled sar rx cache.

    \par Description Check if the message is already in reassembled sar rx cache.

    \param [in] saddr       Source Address - originator of message
    \param [in] ivi         IVI present in the message
    \param [in] seq_auth    SeqAuth of received message
    \param [in] status      Reassembly was success (0x00) or failed/timedout (0x01)

    \return API_RESULT
*/
API_RESULT ltrn_add_to_reassembled_cache
(
    /* IN */ MS_NET_ADDR    saddr,
    /* IN */ UINT8          ivi,
    /* IN */ UINT32         seq_auth,
    /* IN */ UINT8          status
)
{
    UINT16 end;
    LTRN_TRC("[LTRN] Add to reassembled cache. SRC:0x%04X, IVI:0x%02X, SeqAuth:0x%08X, Status: 0x%02X\n",
             saddr, ivi, seq_auth, status);
    LTRN_INF("[LTRN] Cache Start 0x%04X, Size: 0x%04X\n",
             reassembled_cache_start, reassembled_cache_size);

    /* If full, add to the start which is oldest and move start to next */
    if (MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE) == reassembled_cache_size)
    {
        end = reassembled_cache_start;
        /* Move to the next element in the queue */
        reassembled_cache_start++;

        /* Wrap around condition */
        if (MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE) == reassembled_cache_start)
        {
            reassembled_cache_start = 0;
        }
    }
    else
    {
        end = reassembled_cache_start + reassembled_cache_size;

        if (MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE) <= end)
        {
            end -= MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE);
        }
    }

    LTRN_TRC("[LTRN] Saving at reassembled cache Index:0x%04X\n", end);
    reassembled_cache[end].ivi = ivi;
    reassembled_cache[end].saddr = saddr;
    reassembled_cache[end].seq_auth = seq_auth;
    reassembled_cache[end].status = status;

    if (MS_CONFIG_LIMITS(MS_REASSEMBLED_CACHE_SIZE) != reassembled_cache_size)
    {
        reassembled_cache_size++;
    }

    return API_SUCCESS;
}

/**
    \brief Update Replay Protection List on IVI update.

    \par Description
    This routines updates Replay Protection List on IVI update.
*/
void ltrn_replay_cache_on_ivi_update(void)
{
}

/* SAR Implementation */
/* Alloc Context */
LTRN_SAR_CTX* ltrn_sar_alloc_ctx
(
    /* IN */ UINT8              ctx_type,
    /* IN */ UINT16             length
)
{
    UINT32 index;
    LTRN_SAR_CTX*   ctx;
    /* TODO: Check parameters, ctx_type etc. */
    ctx = NULL;

    /* Look for a free context */
    for (index = 0; index < MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX); index++)
    {
        if(LTRN_SAR_CTX_INVALID == ltrn_sar_ctx[index].type)
        {
            ctx = &ltrn_sar_ctx[index];
            ctx->type = ctx_type;
            break;
        }
    }

    if (NULL == ctx)
    {
        LTRN_ERR("[LTRN] Failed to allocate SAR Context. Returning\n");
        return ctx;
    }

    /**
        Now try to allocate memory for the packet to be
        segmented or reassembled.
    */
    ctx->data = (UINT8*) ltrn_alloc_mem(length);

    if (NULL == ctx->data)
    {
        LTRN_ERR("[LTRN] Failed to allocate memory to perform SAR. Freeing context and returning\n");
        /* Mark context free */
        ctx->type = LTRN_SAR_CTX_INVALID;
        ctx = NULL;
    }
    else
    {
        /* Set Data Length */
        ctx->data_length = length;
    }

    LTRN_TRC("[LTRN] Returning SAR Context %p\n", ctx);
    return ctx;
}

/* Free Context */
void ltrn_sar_free_ctx
(
    /* IN */ LTRN_SAR_CTX*   ctx
)
{
    LTRN_TRC("[LTRN] Freeing SAR Context %p\n", ctx);

    /* TODO: NULL Check */

    /* Free memory allocated for SAR */
    if (NULL != ctx->data)
    {
        EM_free_mem(ctx->data);
        ctx->data = NULL;
        ctx->data_length = 0;
        /* Stop Timer */
        #ifdef EM_USE_EXT_TIMER
        EXT_cbtimer_stop_timer(ctx->ack_rtx_timer_handle);
        ctx->ack_rtx_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
        #else
        EM_stop_timer(&ctx->ack_rtx_timer_handle);
        #endif
    }

    #ifdef EM_USE_EXT_TIMER

    if(EXT_CBTIMER_HANDLE_INIT_VAL != ctx->incomplete_timer_handle)
    {
        EXT_cbtimer_stop_timer(ctx->incomplete_timer_handle);
        ctx->incomplete_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
    }

    #else

    if(EM_TIMER_HANDLE_INIT_VAL != ctx->incomplete_timer_handle)
    {
        EM_stop_timer(&ctx->incomplete_timer_handle);
    }

    #endif

    if(seq_num_init_flag && (ltrn_get_sar_ctx_count(LTRN_SAR_CTX_TX)==0))
    {
        NET_INIT_SEQ_NUM_STATE();
        MS_access_cm_set_iv_index(g_iv_update_index, g_iv_update_state);

        if(g_iv_update_start_timer)
        {
            MS_net_start_iv_update_timer(g_iv_update_state,MS_FALSE);
        }

        seq_num_init_flag = FALSE;
    }

    ctx->type = LTRN_SAR_CTX_INVALID;
    return;
}

/**
    Search Context.

    If length passed is 0, then only search and not try to find a free one alongside.

    Will not set the context type in this function,
    to indicate the caller it is an allocated one.

    TODO: See if context type check is really required.
*/
LTRN_SAR_CTX* ltrn_sar_search_and_alloc_ctx
(
    /* IN */ UINT8        ctx_type,
    /* IN */ MS_NET_ADDR  saddr,
    /* IN */ MS_NET_ADDR  daddr,
    /* IN */ UINT16       seq_zero,
    /* IN */ UINT8        obo,
    /* IN */ UINT16       length
)
{
    LTRN_SAR_CTX*   free_ctx;
    LTRN_SAR_CTX*   ctx;
    UINT32          index;
    LTRN_TRC("[LTRN] Search SAR Context. Type:0x%02X, Len:0x%04X, SRC:0x%04X, DST: 0x%04X, SeqZero: 0x%04X, OBO:0x%02X\n",
             ctx_type, length, saddr, daddr, seq_zero, obo);
    /* TODO: Parameter Check */
    ctx = NULL;
    free_ctx = NULL;

    /* Look for a context match */
    for (index = 0; index < MS_CONFIG_LIMITS(LTRN_SAR_CTX_MAX); index++)
    {
        LTRN_TRC("[LTRN] SAR Ctx Table[%d]: Type:0x%02X, SRC:0x%04X, DST:0x%04X, SeqZero:0x%04X\n",
                 index, ltrn_sar_ctx[index].type, ltrn_sar_ctx[index].saddr, ltrn_sar_ctx[index].daddr, ltrn_sar_ctx[index].seq_zero);

        if((ctx_type == ltrn_sar_ctx[index].type) &&
                (saddr == ltrn_sar_ctx[index].saddr) &&
                (seq_zero == ltrn_sar_ctx[index].seq_zero) &&
                /* If OBO == '1', ignore DST Addr */
                ((1 == obo) || (daddr == ltrn_sar_ctx[index].daddr)))
        {
            ctx = &ltrn_sar_ctx[index];
            break;
        }
        else if(LTRN_SAR_CTX_INVALID == ltrn_sar_ctx[index].type)
        {
            if (NULL == free_ctx)
            {
                free_ctx = &ltrn_sar_ctx[index];
            }
        }
    }

    if (NULL != ctx)
    {
        LTRN_TRC("[LTRN] SAR Context Search Success.\n");
    }
    else if ((0 != length) && (NULL != free_ctx))
    {
        ctx = free_ctx;
        LTRN_TRC("[LTRN] SAR Context Search Failed. Trying to allocate Context.\n");
        /**
            Now try to allocate memory for the packet to be
            segmented or reassembled.
        */
        ctx->data = (UINT8*) ltrn_alloc_mem(length);

        if (NULL == ctx->data)
        {
            LTRN_ERR("[LTRN] Failed to allocate memory to perform SAR. Freeing context and returning\n");
            ctx = NULL;
        }
        else
        {
            /* Will not set the context type here, to indicate the caller it is newly allocated one */
            /* Set Data Length */
            /* TODO: Check if length to be saved */
            ctx->data_length = length;
        }
    }

    LTRN_TRC("[LTRN] Returning SAR Context %p\n", ctx);
    return ctx;
}

/* Transmit segments which has not received Ack */
void ltrn_sar_transmit_segments
(
    /* IN */ LTRN_SAR_CTX* sar
)
{
    UINT8      seg_o, seg_n;
    API_RESULT retval;
    UINT8   seg_tx_count;
    /* Destination Address Types */
//    UINT8           daddr_type;
    seg_n = sar->seg_n;
    seg_tx_count = 0;
    LTRN_TRC("[LTRN] Tx Segments for SAR %p. Type 0x%02X\n", sar, sar->type);
    LTRN_TRC("[LTRN] Starting RTX Timer\n");

    if (0 != sar->rtx_count)
    {
        for (seg_o = 0; seg_o <= seg_n; seg_o++)
        {
            /* Check if the associated segment is Acked */
            if (0 == (sar->block_ack & ((UINT32)(1 << seg_o))))
            {
                ltrn_sar_transmit_segment
                (
                    sar,
                    seg_o
                );
                seg_tx_count ++;
            }
        }

        sar->rtx_count--;
        /**
            Start retry timer, which will take care of sending
            un-acked segments.

            After attempting for configured number of retries,
            timer will report transmission failure.
        */
        #ifdef EM_USE_EXT_TIMER
        retval = EXT_cbtimer_start_timer
                 (
                     &sar->ack_rtx_timer_handle,
                     (((seg_tx_count*25)+LTRN_RTX_TIMEOUT) | EXT_CBTIMEOUT_MILLISEC),
                     ltrn_rtx_timeout_handler,
                     (void*)&sar,
                     sizeof(sar)
                 );
        #else
        retval = EM_start_timer
                 (
                     &sar->ack_rtx_timer_handle,
                     (((seg_tx_count*25)+LTRN_RTX_TIMEOUT) | EM_TIMEOUT_MILLISEC),
                     ltrn_rtx_timeout_handler,
                     (void*)&sar,
                     sizeof(sar)
                 );
        #endif

        if (API_SUCCESS != retval)
        {
            ltrn_sar_free_ctx(sar);
            LTRN_ERR("[LTRN] Ack Rtx timer start failed\n");
        }
    }
    else
    {
        ltrn_sar_free_ctx(sar);
    }
}

/* Transmit specific segment */
void ltrn_sar_transmit_segment
(
    /* IN */ LTRN_SAR_CTX* sar,
    /* IN */ UINT8          seg_index
)
{
    UINT16           seg_offset, seg_length;
    UCHAR            pdu[LTRN_MAX_PKT_SIZE];
    MS_BUFFER        ms_buffer;
    MS_NET_HEADER    net_hdr;
    API_RESULT       retval;
    seg_offset = 12 * seg_index;
    LTRN_TRC("[LTRN] Tx Segment 0x%02X for SAR %p\n", seg_index, sar);

    /* If last segment, calculate remaining length */
    if (seg_index == sar->seg_n)
    {
        seg_length = sar->data_length - seg_offset;
    }
    else
    {
        seg_length = 12;
    }

    /* Frame Header */
    /* TODO: Set SZMIC value correctly */
    LTRN_FRM_SEG_ACCESS_MSG_HDR(sar->akf, sar->aid, 0, sar->seq_zero, seg_index, sar->seg_n, pdu);
    EM_mem_copy(&pdu[4], &sar->data[seg_offset], seg_length);
    ms_buffer.payload = pdu;
    ms_buffer.length = seg_length + 4;
    net_hdr.saddr = sar->saddr;
    net_hdr.daddr = sar->daddr;
    net_hdr.ttl   = sar->ttl;
    net_hdr.ctl   = sar->ctl;
    MS_net_alloc_seq_num(&net_hdr.seq_num);
    retval = MS_net_send_pdu
             (
                 &net_hdr,
                 sar->subnet_handle,
                 &ms_buffer,
                 MS_TRUE
             );
}

/* SAR Timeout Handler for Retry */
void ltrn_rtx_timeout_handler (void* args, UINT16 size)
{
    LTRN_SAR_CTX* sar;
    MS_IGNORE_UNUSED_PARAM(size);
    sar = (LTRN_SAR_CTX*)(*((UINT32*)args));
    LTRN_TRC("[LTRN %p] SAR RTX Timeout\n", sar);
    /* Lock */
    LTRN_LOCK_VOID();

    /* Check the SAR entity is still valid */
    if (LTRN_SAR_CTX_INVALID != sar->type)
    {
        /* Reset Timer Handler */
        #ifdef EM_USE_EXT_TIMER
        sar->ack_rtx_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
        #else
        sar->ack_rtx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
        #endif
        /* Trigger transmission */
        ltrn_sar_transmit_segments(sar);
    }

    /* Unlock */
    LTRN_UNLOCK_VOID();
    return;
}

API_RESULT ltrn_sar_start_ack_timer
(
    /* IN */ LTRN_SAR_CTX* sar
)
{
    UINT8      seg_o, seg_n;
    API_RESULT retval;
    UINT8   seg_tx_count;
    retval = API_SUCCESS;
    seg_n = sar->seg_n;
    seg_tx_count = 0;
    LTRN_TRC("[LTRN %p] SAR Start Ack Timer\n", sar);

    /**
        Same timer is used for Ack and Incomplete,
        with number of times the Ack timer fires
        will be decided by Incomplete timeout.

        In the case of non-unicast address,
        the timer will be started with Incomplete
        timeout and the recurrence will be 1.
    */

    for (seg_o = 0; seg_o <= seg_n; seg_o++)   //by hq(zhognqi)
    {
        if (0 == (sar->block_ack & ((UINT32)(1 << seg_o))))
        {
            seg_tx_count ++;
        }
    }

    #ifdef EM_USE_EXT_TIMER

    if(EXT_CBTIMER_HANDLE_INIT_VAL == sar->ack_rtx_timer_handle)
    {
        retval = EXT_cbtimer_start_timer
                 (
                     &sar->ack_rtx_timer_handle,
                     ((seg_tx_count * 25) | EXT_CBTIMEOUT_MILLISEC),
                     ltrn_ack_timeout_handler,
                     (void*)&sar,
                     sizeof(sar)
                 );

        if (API_SUCCESS != retval)
        {
            LTRN_ERR("[LTRN] Ack Rtx timer start failed\n");
        }
    }

    #else

    if(EM_TIMER_HANDLE_INIT_VAL == sar->ack_rtx_timer_handle)
    {
        retval = EM_start_timer
                 (
                     &sar->ack_rtx_timer_handle,
                     ((seg_tx_count * 25) | EM_TIMEOUT_MILLISEC),
                     ltrn_ack_timeout_handler,
                     (void*)&sar,
                     sizeof(sar)
                 );

        if (API_SUCCESS != retval)
        {
            LTRN_ERR("[LTRN] Ack Rtx timer start failed\n");
        }
    }

    #endif
    return retval;
}

API_RESULT ltrn_sar_stop_ack_timer
(
    /* IN */ LTRN_SAR_CTX* sar
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    LTRN_TRC("[LTRN %p] SAR Stop Ack Timer\n", sar);
    #ifdef EM_USE_EXT_TIMER

    if(EXT_CBTIMER_HANDLE_INIT_VAL != sar->ack_rtx_timer_handle)
    {
        retval = EXT_cbtimer_stop_timer(sar->ack_rtx_timer_handle);
        sar->ack_rtx_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
    }

    #else

    if(EM_TIMER_HANDLE_INIT_VAL != sar->ack_rtx_timer_handle)
    {
        retval = EM_stop_timer(&sar->ack_rtx_timer_handle);
    }

    #endif
    return retval;
}

/* SAR Timeout Handler for Ack */
void ltrn_ack_timeout_handler (void* args, UINT16 size)
{
    LTRN_SAR_CTX* sar;
    UINT8          obo;
    MS_IGNORE_UNUSED_PARAM(size);
    sar = (LTRN_SAR_CTX*)(*((UINT32*)args));
    LTRN_TRC("[LTRN %p] SAR Ack Timeout\n", sar);
    /* Reset Timer Handler */
    #ifdef EM_USE_EXT_TIMER
    sar->ack_rtx_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
    #else
    sar->ack_rtx_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    #endif
    /* Lock */
    LTRN_LOCK_VOID();

    /* Check the SAR entity is still valid */
    if (LTRN_SAR_CTX_INVALID != sar->type)
    {
        MS_NET_ADDR save_daddr;

        if (LTRN_RX_FOR_LPN_ELEMENT == (sar->rx_meta_info.is_for & LTRN_RX_FOR_LPN_ELEMENT))
        {
            obo = 0x01;
            MS_access_cm_get_primary_unicast_address(&save_daddr);
        }
        else
        {
            obo = 0x00;
            save_daddr = sar->daddr;
        }

        LTRN_TRC("[LTRN] SAR OBO:0x%02X\n", obo);
        /* Send Block Ack */
        ltrn_send_ack
        (
            save_daddr,
            sar->saddr,
            sar->subnet_handle,
            sar->seq_zero,
            0x7F, /* net_hdr->ttl, */
            obo,
            sar->block_ack
        );
    }

    /* Timer will be restarted on getting as associated segment again */
    /* Unlock */
    LTRN_UNLOCK_VOID();
    return;
}

API_RESULT ltrn_sar_restart_incomplete_timer
(
    /* IN */ LTRN_SAR_CTX* sar
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    LTRN_TRC("[LTRN %p] SAR restart incomplete Timer\n", sar);
    /**
        Same timer is used for Ack and Incomplete,
        with number of times the Ack timer fires
        will be decided by Incomplete timeout.

        In the case of non-unicast address,
        the timer will be started with Incomplete
        timeout and the recurrence will be 1.
    */
    #ifdef EM_USE_EXT_TIMER

    if(EXT_CBTIMER_HANDLE_INIT_VAL != sar->incomplete_timer_handle)
    {
        retval = EXT_cbtimer_stop_timer(sar->incomplete_timer_handle);
        sar->incomplete_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
    }

    retval = EXT_cbtimer_start_timer
             (
                 &sar->incomplete_timer_handle,
                 (LTRN_INCOMPLETE_TIMEOUT | EXT_CBTIMEOUT_MILLISEC),
                 ltrn_incomplete_timeout_handler,
                 (void*)&sar,
                 sizeof(sar)
             );
    #else

    if(EM_TIMER_HANDLE_INIT_VAL != sar->incomplete_timer_handle)
    {
        retval = EM_stop_timer(&sar->incomplete_timer_handle);
    }

    retval = EM_start_timer
             (
                 &sar->incomplete_timer_handle,
                 (LTRN_INCOMPLETE_TIMEOUT | EM_TIMEOUT_MILLISEC),
                 ltrn_incomplete_timeout_handler,
                 (void*)&sar,
                 sizeof(sar)
             );
    #endif

    if (API_SUCCESS != retval)
    {
        /* Add to Completed SAR Rx Cache */
        ltrn_add_to_reassembled_cache
        (
            sar->saddr,
            sar->ivi,
            sar->seq_auth,
            0x01 /* Failure */
        );
        /* Free SAR Context */
        ltrn_sar_free_ctx(sar);
        LTRN_ERR("[LTRN] Incomplete timer start failed\n");
    }

    return retval;
}

/* SAR Incomplete Timeout Handler for Rx */
void ltrn_incomplete_timeout_handler (void* args, UINT16 size)
{
    LTRN_SAR_CTX* sar;
    MS_IGNORE_UNUSED_PARAM(size);
    sar = (LTRN_SAR_CTX*)(*((UINT32*)args));
    LTRN_ERR("[LTRN %p] SAR Incomplete Timeout.\n", sar);
    #ifdef EM_USE_EXT_TIMER
    sar->incomplete_timer_handle = EXT_CBTIMER_HANDLE_INIT_VAL;
    #else
    sar->incomplete_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    #endif
    /* Lock */
    LTRN_LOCK_VOID();

    /* Check the SAR entity is still valid */
    if (LTRN_SAR_CTX_INVALID != sar->type)
    {
        /* Add to Completed SAR Rx Cache */
        ltrn_add_to_reassembled_cache
        (
            sar->saddr,
            sar->ivi,
            sar->seq_auth,
            0x01 /* Failure */
        );
        /* Free SAR Context */
        ltrn_sar_free_ctx(sar);
    }

    /* Unlock */
    LTRN_UNLOCK_VOID();
    return;
}
#endif /* MS_LTRN */

