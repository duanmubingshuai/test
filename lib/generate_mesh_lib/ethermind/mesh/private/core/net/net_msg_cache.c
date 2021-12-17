
/**
    \file net_msg_cache.c

    Message cache related routines.
    Message cache is used to restrict unlimited flooding of messages.
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "net_internal.h"
#include "net_extern.h"

#ifdef MS_NET
/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */

/**
    \brief Initialize network cache.

    \par Description Initialize network cache.
*/
void net_init_cache (void)
{
    /* Cache start and end initialization */
    net_cache_start = net_cache_size = 0;
}

/**
    \brief Check if the message is already in network cache.

    \par Description Check if the message is already in network cache.

    \param [in] hdr    Received Network Packet Header

    \return API_RESULT
*/
API_RESULT net_is_in_cache
(
    /* IN */ MS_NET_HEADER*   hdr
)
{
    API_RESULT retval;
    UINT16 index, cur;
    retval = API_FAILURE;
    /* Search for all valid elements */
    NET_TRC("[NET] In cache? SRC:0x%04X, IVI:0x%02X, SEQ:0x%08X, NID:0x%02X\n",
            hdr->saddr, hdr->ivi, hdr->seq_num, hdr->nid);
    /* Start from the queue start */
    cur = net_cache_start;
    NET_INF("[NET] Seach Start Index 0x%04X\n", cur);

    for (index = 0; index < net_cache_size; index++)
    {
        /* Look for match */
        if (((net_cache[cur].nid == hdr->nid)) &&
                ((net_cache[cur].ivi == hdr->ivi)) &&
                ((net_cache[cur].saddr == hdr->saddr)) &&
                ((net_cache[cur].seq_num == hdr->seq_num)))
        {
            NET_TRC("[NET] Cache match found at Index: 0x%04X\n",
                    cur);
            retval = API_SUCCESS;
            break;
        }

        /* Go to the next element in the queue */
        cur++;

        /* Wrap around condition */
        if (MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE) == cur)
        {
            cur = 0;
        }
    }

    NET_TRC("[NET] In Cache returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief Add message to network cache.

    \par Description Add message to network cache.
    Check if the message is already in the cache.
    If not, add it to the cache.
    In case case is full, then the oldest message will be removed,
    to make space for this new message.

    \param [in] hdr    Received Network Packet Header

    \return API_RESULT
*/
API_RESULT net_add_to_cache
(
    /* IN */ MS_NET_HEADER*   hdr
)
{
    UINT16 end;
    NET_TRC("[NET] Add to cache. SRC:0x%04X, IVI:0x%02X, SEQ:0x%08X, NID:0x%02X\n",
            hdr->saddr, hdr->ivi, hdr->seq_num, hdr->nid);
    NET_INF("[NET] Cache Start 0x%04X, Size: 0x%04X\n",
            net_cache_start, net_cache_size);

    /* If full, add to the start which is oldest and move start to next */
    if (MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE) == net_cache_size)
    {
        end = net_cache_start;
        /* Move to the next element in the queue */
        net_cache_start++;

        /* Wrap around condition */
        if (MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE) == net_cache_start)
        {
            net_cache_start = 0;
        }
    }
    else
    {
        end = net_cache_start + net_cache_size;

        if (MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE) <= end)
        {
            end -= MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE);
        }
    }

    NET_TRC("[NET] Saving at Cache Index: 0x%04X\n", end);
    net_cache[end].ivi = hdr->ivi;
    net_cache[end].nid = hdr->nid;
    net_cache[end].saddr = hdr->saddr;
    net_cache[end].seq_num = hdr->seq_num;

    if (MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE) != net_cache_size)
    {
        net_cache_size++;
    }

    return API_SUCCESS;
}


API_RESULT net_delete_from_cache
(
    /* IN */ MS_NET_ADDR  addr
)
{
    UINT16  net_index;
    UINT16  i;
    net_index = net_cache_start;

    for(i = 0; i < net_cache_size; i++)
    {
        if(net_cache[net_index].saddr == addr)
        {
            net_cache[net_index].ivi = 0;
            net_cache[net_index].nid = 0;
            net_cache[net_index].saddr = 0;
            net_cache[net_index].seq_num = 0;
        }

        if(net_index == MS_CONFIG_LIMITS(MS_NET_CACHE_SIZE))
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

#endif /* MS_NET */

