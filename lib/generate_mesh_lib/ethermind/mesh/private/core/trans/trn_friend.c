
/**
    \file trn_friend.c


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "trn_internal.h"
#include "trn_extern.h"
#include "sec_tbx.h"

#if (defined MS_FRIEND_SUPPORT || defined MS_LPN_SUPPORT)

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
/** Friend Poll Time Scale Factor */
#define FRND_POLLTO_FACTOR          10
#define FRND_POLLRETRY_TIMEOUT_MS   100
#define LPN_RXWINDOW_OFFSET_MS      50

#ifdef MS_LPN_SUPPORT
    /**
    Global Friend Request parameter structure

    TODO:
    Used for friend request retransmissions as well as lpn_counter
    and criteria members are used on receiving offer. These can be made
    a member of Friend Element structure and this request structure can be passed
    as a timeout argument to the request retry timer.
    */
    DECL_STATIC MS_TRN_FRND_REQ_PARAM frnd_req;

    DECL_STATIC UCHAR trn_poll_period;
#endif /* MS_LPN_SUPPORT */
extern uint8 llState;
extern uint8 llSecondaryState;

/* --------------------------------------------- Functions */

#ifdef MS_LPN_SUPPORT

/**
    \brief

    \par Description

    \param subnet_handle
    \param criteria
    \param rx_delay
    \param poll_timeout
    \param setup_timeout

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_lpn_setup_friendship
(
    /* IN */ MS_SUBNET_HANDLE subnet_handle,
    /* IN */ UCHAR criteria,
    /* IN */ UCHAR rx_delay,
    /* IN */ UINT32 poll_timeout,
    /* IN */ UINT32 setup_timeout,
    /* IN */ TRN_FRND_CB cb
)
{
    TRN_TRC("[FRND] MS_trn_lpn_setup_friendship\n");
    TRN_TRC("[FRND] Framing the friend request with Criteria - 0x%02X, Rx Delay - 0x%02X, "
            " Poll Timeout - 0x%08X\n", criteria, rx_delay, poll_timeout);
    /* Lock */
    TRN_LOCK();

    /* Check if friendship already established or in progress */
    if (MS_NET_ADDR_UNASSIGNED != frnd_element.addr)
    {
        /* Unlock */
        TRN_UNLOCK();
        TRN_ERR("[FRND] Friendship already established. Clear to establish new friendship\n");
        return TRN_INVALID_FRNDSHIP_STATE;
    }

    /* Frame the Friend Request parameter */
    frnd_req.criteria = criteria;
    frnd_req.rx_delay = rx_delay;
    frnd_req.poll_to = poll_timeout;
    frnd_req.prev_addr = frnd_element.addr;
    frnd_req.lpn_counter = trn_lpn_counter++;
    MS_access_cm_get_element_count(&frnd_req.num_elem);
    /* Save the friend setup timeout in millisecond */
    frnd_setup_timeout = setup_timeout;
    frnd_setup_time_lapsed = 0;
    frnd_cb = cb;
    TRN_TRC("[FRND] Sending the friend request on subnet handle - 0x%04X\n", subnet_handle);
    /* Send the packet with TTL 0 */
    trn_frnd_send(MS_NET_ADDR_ALL_FRIENDS, subnet_handle, MS_TRN_CTRL_OPCODE_FRND_REQ, &frnd_req, 0);
    /* Update friend element to indicate establishment in progress */
    frnd_element.addr = MS_NET_ADDR_ALL_FRIENDS;
    frnd_element.subnet_handle = subnet_handle;
    /*
        Start a timer to periodically attempt friend establishment for
        the duration of the setup timeout
    */
    EM_start_timer
    (
        &frnd_element.thandle,
        (EM_TIMEOUT_MILLISEC | MS_TRN_FRNDREQ_RETRY_TIMEOUT),
//        (EM_TIMEOUT_MILLISEC | 3000),        // HZF
        trn_frndreq_retry_timeout_handler,
        NULL,
        0
    );
    /* Unlock */
    TRN_UNLOCK();
    return API_SUCCESS;
}


/**
    \brief

    \par Description

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_lpn_clear_friendship (void)
{
    MS_TRN_FRND_CLEAR_PARAM frnd_clear;
    TRN_TRC("[FRND] MS_trn_lpn_clear_friendship\n");
    /* Lock */
    TRN_LOCK();

    /* If the friend information is valid, terminate the friendship */
    if (MS_NET_ADDR_UNASSIGNED != frnd_element.addr)
    {
        TRN_TRC("[FRND] Valid Friend Element. Clearing...\n");

        /* Is the poll timer active? */
        if (EM_TIMER_HANDLE_INIT_VAL != frnd_element.thandle)
        {
            /* Stop the polling timer */
            EM_stop_timer(&frnd_element.thandle);
            /* Wakeup bearer */
            MS_brr_wakeup(BRR_TX | BRR_RX);
            /* Set to indicate out of poll period */
            trn_poll_period = MS_FALSE;
        }

        /*
            Send the FriendClear to the Friend?
            Specification section 3.6.6.4.2 says that an LPN may send a
            FriendClear to terminate friendship with a friend. If not, the
            friendship will be terminated when the friends poll timeout
            expires.
        */
        /* Frame clear */
        MS_access_cm_get_primary_unicast_address(&(frnd_clear.lpn_addr));
        frnd_clear.lpn_counter = frnd_req.lpn_counter;
        /* Send the packet with TTL 0 */
        trn_frnd_send(frnd_element.addr, 0, MS_TRN_CTRL_OPCODE_FRND_CLEAR, &frnd_clear, 0);
        /* Reset the friend information */
        EM_mem_set(&frnd_element, 0x00, sizeof(TRN_FRND_ELEMENT));
        frnd_element.thandle = EM_TIMER_HANDLE_INIT_VAL;
        /* Reset the friend address */
        frnd_element.addr = MS_NET_ADDR_UNASSIGNED;
        /* Update current friendship role as INVALID */
        MS_access_cm_set_friendship_role(MS_FRND_ROLE_INVALID);
    }

    /* Unlock */
    TRN_UNLOCK();
    return API_SUCCESS;
}


/**
    \brief

    \par Description

    \param action
    \param addr
    \param count

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_lpn_manage_subscription
(
    UCHAR         action,
    UINT16*        addr_list,
    UINT16        count
)
{
    API_RESULT retval;
    MS_TRN_FRND_MANAGE_PARAM frnd_manage;
    retval = API_FAILURE;
    TRN_TRC("[FRND] MS_trn_lpn_manage_subscription\n");
    TRN_TRC("[FRND] Framing the subscription for opcode - 0x%02X, Count - %d\n",
            action, count);
    /* Lock */
    TRN_LOCK();

    /* If the friend information is valid, terminate the friendship */
    if (MS_NET_ADDR_UNASSIGNED != frnd_element.addr)
    {
        TRN_TRC("[FRND] Valid Friend Element. Manage Subscription %02X...\n", action);

        /* Is in poll period? */
        if (MS_TRUE == trn_poll_period)
        {
            /* Wakeup bearer */
            MS_brr_wakeup(BRR_TX | BRR_RX);
        }

        /* Stop and Reset the timer handle */
        EM_stop_timer (&frnd_element.thandle);
        /* Frame the Friend Management parameter */
        frnd_manage.addr_list = addr_list;
        frnd_manage.num_addr = count;
        frnd_manage.txn_num = frnd_element.txn_no ++;
        /* Send the packet with TTL 0 */
        retval = trn_frnd_send(frnd_element.addr, MS_CONFIG_LIMITS(MS_MAX_SUBNETS), action, &frnd_manage, 0);
        TRN_TRC("[FRND] Starting Subscription tracking timeout for %lu ms...\n", FRND_POLLRETRY_TIMEOUT_MS);
        /* Start the Poll Timeout */
        retval = EM_start_timer
                 (
                     &frnd_element.thandle,
                     (EM_TIMEOUT_MILLISEC | FRND_POLLRETRY_TIMEOUT_MS),
                     trn_frndpoll_send_handler,
                     NULL,
                     0
                 );

        if (API_SUCCESS != retval)
        {
            TRN_ERR("[FRND] Failed to start Frnd Poll Retry timer. Retval - 0x%04X\n", retval);
        }
    }

    /* Unlock */
    TRN_UNLOCK();
    return retval;
}


/**
    \brief

    \par Description


    \param net_hdr
    \param param

    \return void
*/
void trn_handle_frnd_offer
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_TRN_FRND_OFFER_PARAM* param
)
{
    MS_NET_ADDR uaddr;
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(subnet_handle);
    MS_IGNORE_UNUSED_PARAM(param);
    TRN_TRC("[FRND] Friend Offer from 0x%04X\n", net_hdr->saddr);
    TRN_TRC("[FRND] Rcv Window: 0x%02X, Q Size: 0x%02X, SubList Size: 0x%02X "
            "RSSI: 0x%02X, Frnd Counter: 0x%04X\n",
            param->rx_window, param->queue_size, param->sublist_size, param->rssi,
            param->frnd_counter);

    /* Validate TTL is 0 as mandatory */
    if (0 != net_hdr->ttl)
    {
        TRN_INF("[FRND] Dropping packet because TTL != 0\n");
        return;
    }

    /* Check if we had initiated a friend request */
    if (MS_NET_ADDR_ALL_FRIENDS != frnd_element.addr)
    {
        TRN_INF("[FRND] Spurious Friend Offer. Dropping...\n");
        return;
    }

    /* Check for prohibited offer parameters */
    if (0 == param->rx_window)
    {
        TRN_INF("[FRND] Invalid Rx Window (0x%02X) detected. Dropping...\n");
        return;
    }

    #if 0
    /* Validate the Offer received if it satisfies the request criteria */
    retval = trn_validate_frnd_offer_pl
             (
                 frnd_req.criteria,
                 param->rssi,
                 param->rx_window,
                 param->queue_size
             );

    if (API_SUCCESS != retval)
    {
        TRN_INF("[FRND] Offer NOT satisfactory. Dropping...\n");
        return;
    }

    #endif
    /* Initialize transaction number */
    frnd_element.txn_no = 0xFF;
    /* Stop the Friend Request retry timer that must be running. */
    EM_stop_timer(&frnd_element.thandle);
    TRN_TRC("[FRND] Updating Friendship Credentials...\n");
    /* Get the unicast source address of primary element */
    MS_access_cm_get_primary_unicast_address(&uaddr);
    /* Call to generate the friend credentials */
    MS_access_cm_add_friend_sec_credential
    (
        subnet_handle,
        0,
        uaddr,
        net_hdr->saddr,
        frnd_req.lpn_counter,
        param->frnd_counter
    );
    /* Update the friend element */
    frnd_element.rx_window = param->rx_window + LPN_RXWINDOW_OFFSET_MS;
    frnd_element.addr = net_hdr->saddr;
    frnd_element.txn_no = 0xFF;
    frnd_element.fsn = 0;
    frnd_element.sfsn = 0;
    frnd_element.node.lpn_counter = frnd_req.lpn_counter;
    frnd_element.node.frnd_counter = param->frnd_counter;
    frnd_element.node.addr = frnd_element.addr;
    frnd_element.node.subnet_handle = frnd_element.subnet_handle;
    frnd_element.poll_retry_count = 0;
    /* Poll the friend */
    trn_frndpoll();
}


/**
    \brief

    \par Description


    \param net_hdr
    \param param

    \return void
*/
void trn_handle_frnd_update
(
    /* IN */ MS_NET_HEADER*              net_hdr,
    /* IN */ MS_SUBNET_HANDLE            subnet_handle,
    /* IN */ MS_TRN_FRND_UPDATE_PARAM*   param
)
{
    API_RESULT retval;
    UINT8 krstate;
    MS_IGNORE_UNUSED_PARAM(param);
    TRN_TRC("[FRND] Friend Update from 0x%04X\n", net_hdr->saddr);
    TRN_TRC("[FRND] Flags: 0x%02X, IV Index: 0x%08X, MD: 0x%02X\n",
            param->flags, param->ivi, param->md);

    /* Validate TTL is 0 as mandatory */
    if (0 != net_hdr->ttl)
    {
        TRN_INF("[FRND] Dropping packet because TTL != 0\n");
        return;
    }

    /* Check if Friend Update is from the intended device */
    if (net_hdr->saddr != frnd_element.addr)
    {
        TRN_INF("[FRND] Friend Update not from intended device. Dropping...\n");
        return;
    }

    /* Stop the Friend Poll Timer */
    EM_stop_timer(&frnd_element.thandle);
    /* Update the FSN */
    frnd_element.fsn ^= 0x01;
    /* Set Key Refresh State to Phase 2 */
    krstate = (param->flags & 0x01)? MS_ACCESS_KEY_REFRESH_PHASE_2 : MS_ACCESS_KEY_REFRESH_PHASE_NORMAL;
    /* Set the Key Refresh Phase */
    MS_access_cm_set_key_refresh_phase(subnet_handle, &krstate);
    /* Process the Flags and IVI */
    retval = MS_access_cm_set_iv_index
             (
                 param->ivi,
                 ((param->flags & 0x02)? MS_TRUE: MS_FALSE)
             );

    /* Check if successful */
    if (API_SUCCESS != retval)
    {
        TRN_ERR("[FRND] Set IV Update Failed - 0x%04X\n", retval);
    }

    /* Initialize the poll retries */
    frnd_element.poll_retry_count = 0;
    /* Sleep the bearer */
    MS_brr_sleep();

    /* Poll the friend again if MoreData (MD) is set */
    if (0x00 != param->md)
    {
        TRN_TRC("[FRND] Polling for more data...\n");
        /* Poll the friend */
        trn_frndpoll();
    }
    else
    {
        /* Set to indicate in poll period */
        trn_poll_period = MS_TRUE;
        TRN_TRC("[FRND] Start Poll Timeout... %d\n", frnd_element.poll_to);
        /* Start the Poll Timeout */
        retval = EM_start_timer
                 (
                     &frnd_element.thandle,
                     (EM_TIMEOUT_MILLISEC | frnd_element.poll_to),
                     trn_frndpoll_send_handler,
                     NULL,
                     0
                 );
    }

    if (API_SUCCESS != retval)
    {
        TRN_ERR("[FRND] Failed to start Frnd Poll timer. Retval - 0x%04X\n", retval);
    }

    /*
        If this is the first update received after offer,
        send a friendship setup with success to the
        application callback
    */
    if (0xFF == frnd_element.txn_no)
    {
        /* Initialize transaction */
        frnd_element.txn_no = 0x00;
        /* Update current friendship role as LPN */
        MS_access_cm_set_friendship_role(MS_FRND_ROLE_LPN);
        /* Unlock */
        TRN_UNLOCK_VOID();
        /* Notify Application */
        frnd_cb (subnet_handle, MS_TRN_FRIEND_SETUP_CNF, API_SUCCESS);
        /* Lock */
        TRN_LOCK_VOID();
    }
}


/* Handle Friend Subscription List Confirmation */
void trn_handle_frnd_subscription_list_cnf
(
    /* IN */ MS_NET_HEADER*                          net_hdr,
    /* IN */ MS_SUBNET_HANDLE                        subnet_handle,
    /* IN */ MS_TRN_FRND_SUBSCRN_LIST_CNF_PARAM*     param
)
{
    MS_IGNORE_UNUSED_PARAM(net_hdr);
    MS_IGNORE_UNUSED_PARAM(param);

    /* Is in poll period? */
    if (MS_TRUE == trn_poll_period)
    {
        /* Sleep bearer */
        MS_brr_sleep();
    }

    /* Unlock */
    TRN_UNLOCK_VOID();
    frnd_cb (subnet_handle, MS_TRN_FRIEND_SUBSCRNLIST_CNF, API_SUCCESS);
    /* Lock */
    TRN_LOCK_VOID();
}


void trn_frndreq_retry_timeout_handler(void* args, UINT16 size)
{
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);
    TRN_TRC("[FRND] FrndReq retry timeout handler. Subnet Handle 0x%04X\n",
            frnd_element.subnet_handle);
    /* Lock */
    TRN_LOCK_VOID();
    /* Reset the timer handle */
    frnd_element.thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* Update the lapsed time */
    frnd_setup_time_lapsed += MS_TRN_FRNDREQ_RETRY_TIMEOUT;

    /* Is the setup timeout period is crossed? */
    if (frnd_setup_timeout <= frnd_setup_time_lapsed)
    {
        TRN_TRC("[FRND] Setup timeout expired. Notify Application.\n");
        /* Reset the Friend Element */
        EM_mem_set(&frnd_element, 0x00, sizeof(TRN_FRND_ELEMENT));
        /* Reset the friend address */
        frnd_element.addr = MS_NET_ADDR_UNASSIGNED;
        /* Yes, notify application of unsuccessful procedure completion */
        /* Unlock */
        TRN_UNLOCK_VOID();
        /* Notify Application */
        frnd_cb (frnd_element.subnet_handle, MS_TRN_FRIEND_SETUP_CNF, API_FAILURE);
    }
    else
    {
        TRN_TRC("[FRND] Resending Friend Request...\n");
        /* Update the LPN counter */
        frnd_req.lpn_counter = trn_lpn_counter++;
        /* Send the packet with TTL 0 */
        trn_frnd_send(MS_NET_ADDR_ALL_FRIENDS, frnd_element.subnet_handle, MS_TRN_CTRL_OPCODE_FRND_REQ, &frnd_req, 0);
        /* Restart the timer */
        EM_start_timer
        (
            &frnd_element.thandle,
            (EM_TIMEOUT_MILLISEC | MS_TRN_FRNDREQ_RETRY_TIMEOUT),
            trn_frndreq_retry_timeout_handler,
            NULL,
            0
        );
        /* Unlock */
        TRN_UNLOCK_VOID();
    }
}

/* Receive Window timeout handler */
void trn_frndrx_window_timeout_handler(void* args, UINT16 size)
{
    API_RESULT retval;
    UINT32 tmo;
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);
    TRN_TRC("[FRND] Rx Window timeout handler. Sleep bearer and Start Poll retry timer...\n");
    /* Lock */
    TRN_LOCK_VOID();
    /* Reset the timer handle */
    frnd_element.thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* Sleep the bearer from listening */
    MS_brr_sleep();
    TRN_TRC("[FRND] Poll Timeout - %lu, Poll retry count - %d\n",
            frnd_element.poll_to, frnd_element.poll_retry_count);
    /* Set the timeout */
    tmo = (MS_FRND_POLL_RETRY_COUNT == frnd_element.poll_retry_count) ?
          (FRND_POLLRETRY_TIMEOUT_MS * 3): FRND_POLLRETRY_TIMEOUT_MS;
    TRN_TRC("[FRND] Starting Poll timeout for %lu ms...\n", tmo);
    /* Start the Poll Timeout */
    retval = EM_start_timer
             (
                 &frnd_element.thandle,
                 (EM_TIMEOUT_MILLISEC | tmo),
                 trn_frndpoll_send_handler,
                 NULL,
                 0
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR("[FRND] Failed to start Frnd Poll Retry timer. Retval - 0x%04X\n", retval);
    }

    /* Unlock */
    TRN_UNLOCK_VOID();
}

/* Receive Delay timeout handler */
void trn_frndrx_delay_timeout_handler(void* args, UINT16 size)
{
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    TRN_LOCK_VOID();
    TRN_TRC("[FRND] Rx Delay timeout handler. Wakeup bearer and Start Rx Window timer...\n");
    /* Reset the timer handle */
    frnd_element.thandle = EM_TIMER_HANDLE_INIT_VAL;
    /* Wake up the bearer to listen */
    MS_brr_wakeup(BRR_RX);
    /* Adjust the poll timeout */
    frnd_element.poll_to -= frnd_req.rx_delay;
    TRN_TRC("[FRND] Updated Poll Timeout - %lu, Poll retry count - %d\n",
            frnd_element.poll_to, frnd_element.poll_retry_count);
    /* Start the Rx Window Timeout handler to stop listening after the window expiry */
    retval = EM_start_timer
             (
                 &frnd_element.thandle,
                 (EM_TIMEOUT_MILLISEC | frnd_element.rx_window),
                 trn_frndrx_window_timeout_handler,
                 NULL,
                 0
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR("[FRND] Failed to start Rx Window timer. Retval - 0x%04X\n", retval);
    }

    /* Unlock */
    TRN_UNLOCK_VOID();
}

void trn_frndpoll(void)
{
    API_RESULT retval;
    MS_TRN_FRND_POLL_PARAM frnd_poll;
    TRN_TRC("[FRND] Framing and Sending Poll with FSN 0x%02X\n", frnd_element.fsn);
    /* Frame the friend poll parameters */
    frnd_poll.fsn = frnd_element.fsn;
    frnd_element.sfsn = frnd_element.fsn;
    frnd_element.poll_to = ((frnd_req.poll_to * 100) - (FRND_POLLTO_FACTOR * 100));
    TRN_TRC("[FRND] Configured Poll Timeout - %lu\n", frnd_element.poll_to);
    /* Wakeup bearer to send */
    MS_brr_wakeup(BRR_TX);
    /* Send a Friend Poll to the destination friend with TTL 0 - 3 times recommended */
    trn_frnd_send(frnd_element.addr, MS_CONFIG_LIMITS(MS_MAX_SUBNETS), MS_TRN_CTRL_OPCODE_FRND_POLL, &frnd_poll, 0);
    /* Start the Rx Delay trigger timeout */
    retval = EM_start_timer
             (
                 &frnd_element.thandle,
                 (EM_TIMEOUT_MILLISEC | (frnd_req.rx_delay+100)), /*when use xip mode,the delay should delay more 100ms*/
                 trn_frndrx_delay_timeout_handler,
                 NULL,
                 0
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR("[FRND] Failed to start Rx Delay timer. Retval - 0x%04X\n", retval);
    }
}


void trn_frndpoll_send_handler(void* args, UINT16 size)
{
    UCHAR poll_again;
    MS_IGNORE_UNUSED_PARAM(args);
    MS_IGNORE_UNUSED_PARAM(size);
    poll_again = 0x01;
    TRN_TRC("[FRND] Friend Poll Send Timeout Handler\n");
    /* Lock */
    TRN_LOCK_VOID();
    /* Reset Timer handle */
    frnd_element.thandle = EM_TIMER_HANDLE_INIT_VAL;

    /* Is Response not received for the previously sent Friend Poll? */
    if (frnd_element.sfsn == frnd_element.fsn)
    {
        TRN_INF("[FRND] Response not received...\n");

        /* Check if the number of Rx window retries has expired */
        if (MS_FRND_POLL_RETRY_COUNT > frnd_element.poll_retry_count)
        {
            TRN_TRC("[FRND] Retrying with Poll Retry timeout...\n");
        }
        else if ((MS_FRND_POLL_RETRY_COUNT + 1) < frnd_element.poll_retry_count)
        {
            /* Yes, Terminate the friendship */
            TRN_ERR("[FRND] Poll retry attempt expired. Terminating Friendship...\n");
            poll_again = 0x00;
            /* Update current friendship role as INVALID */
            MS_access_cm_set_friendship_role(MS_FRND_ROLE_INVALID);
            /* Reset the Friend Element */
            EM_mem_set(&frnd_element, 0x00, sizeof(TRN_FRND_ELEMENT));
            frnd_element.thandle = EM_TIMER_HANDLE_INIT_VAL;
            /* Reset the friend address */
            frnd_element.addr = MS_NET_ADDR_UNASSIGNED;

            if (0xFF == frnd_element.txn_no)
            {
                /* Unlock */
                TRN_UNLOCK_VOID();
                /* Notify Application */
                frnd_cb(frnd_element.subnet_handle, MS_TRN_FRIEND_SETUP_CNF, API_FAILURE);
                /* Lock */
                TRN_LOCK_VOID();
            }
            else
            {
                /* Wakeup bearer */
                MS_brr_wakeup(BRR_TX | BRR_RX);
                /* Set to indicate out of poll period */
                trn_poll_period = MS_FALSE;
                /* Unlock */
                TRN_UNLOCK_VOID();
                /* Notify Application */
                frnd_cb(frnd_element.subnet_handle, MS_TRN_FRIEND_TERMINATE_IND, API_SUCCESS);
                /* Lock */
                TRN_LOCK_VOID();
            }
        }
    }
    else
    {
        TRN_TRC ("[FRND] Poll Again...\n");
        printf("llState = %d, llSecondaryState = %d\r\n", llState, llSecondaryState);
        /* Set to indicate out of poll period */
        trn_poll_period = MS_FALSE;
    }

    if (0x01 == poll_again)
    {
        /* No, Send Friend Poll */
        trn_frndpoll();
        /* Increment the poll retry counter */
        frnd_element.poll_retry_count++;
    }

    /* Unlock */
    TRN_UNLOCK_VOID();
}

void trn_frnd_handle_segment_ack(MS_SUBNET_HANDLE subnet_handle)
{
    MS_IGNORE_UNUSED_PARAM(subnet_handle);
    TRN_TRC("[FRND] Segment Received from Friend\n");
    TRN_LOCK_VOID();

    if (0xFF == frnd_element.txn_no)
    {
        TRN_UNLOCK_VOID();
        return;
    }

    /* Stop the Friend Poll Timer */
    EM_stop_timer(&frnd_element.thandle);
    /* Update the FSN */
    frnd_element.fsn ^= 0x01;
    frnd_element.poll_retry_count = 0;
    frnd_element.poll_to = ((frnd_req.poll_to * 100) - (FRND_POLLTO_FACTOR * 100));
    /* Sleep the bearer */
    MS_brr_sleep();
    /* Poll the friend */
    trn_frndpoll();
    TRN_UNLOCK_VOID();
}
#endif /* MS_LPN_SUPPORT */

#ifdef MS_FRIEND_SUPPORT
/**
    \brief To check if address matches with any of the LPN

    \par Description
    This routine checks if destination address in a received packet matches
    with any of the known element address of LPN.

    \param [in]  addr           Unicast Address to search
    \param [out] lpn_handle     LPN Handle on match

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_is_valid_lpn_element_address
(
    /* IN */  MS_NET_ADDR     addr,
    /* OUT */ LPN_HANDLE*     lpn_handle
)
{
    API_RESULT                retval;
    UINT32 index;
    MS_NET_ADDR  start_addr;
    UINT8        element_count;
    /* Lock */
    retval = ACCESS_NO_MATCH;

    /* Not Checking if a valid unicast address */

    /* TODO: Friend LPN addresses are maintained now separately. This can be relooked */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_LPNS); index ++)
    {
        if (LPN_ELEMENT_VALID_PERMANENT == lpn_element[index].valid)
        {
            /* LPN Element Start Address and Element Count */
            start_addr = lpn_element[index].node.addr;
            element_count = lpn_element[index].num_elements;

            /* Check if the address is in range */
            if ((addr >= start_addr) &&
                    (addr < (start_addr + element_count)))
            {
                retval = API_SUCCESS;
                *lpn_handle = (LPN_HANDLE) index;
                TRN_TRC(
                    "[FRND] Found LPN Element Address Match for Handle:0x%04X\n",
                    *lpn_handle);
                break;
            }
        }
    }

    /* Unlock */
    return retval;
}

/**
    \brief To check if valid subscription address of an LPN to receive a packet

    \par Description
    This routine checks if destination address in a received packet matches
    with any of the known subscription address of an LPN.

    \param [in]  addr          Address to search
    \param [out] lpn_handle    Pointer to an LPN Handle, which will be filled
                               if match found

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_is_valid_lpn_subscription_address
(
    /* IN */  MS_NET_ADDR     addr,
    /* OUT */ LPN_HANDLE*     lpn_handle
)
{
    API_RESULT    retval;
    UINT32        index, sub_list_index;
    /* Lock */
    retval = ACCESS_NO_MATCH;
    /* Not Checking if a valid unicast address */
    index = (LPN_HANDLE_INVALID == (*lpn_handle)) ? 0 : (*lpn_handle + 1);

    /* TODO: Friend LPN addresses are maintained now separately. This can be relooked */
    for (; index < MS_CONFIG_LIMITS(MS_MAX_LPNS); index ++)
    {
        if (LPN_ELEMENT_VALID_PERMANENT == lpn_element[index].valid)
        {
            /* Find next match */
            for (sub_list_index = 0; sub_list_index < MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE); sub_list_index++)
            {
                /* Check if the subscription list element matches */
                if (lpn_element[index].subscription_list[sub_list_index] == addr)
                {
                    retval = API_SUCCESS;
                    *lpn_handle = (LPN_HANDLE)index;
                    TRN_TRC(
                        "[FRND] Found LPN Element Address Match for Handle:0x%04X\n",
                        *lpn_handle);
                    break;
                }
            }

            /* Break if element found */
            if (API_SUCCESS == retval)
            {
                break;
            }
        }
    }

    /* Unlock */
    return retval;
}

/**
    \brief To check if valid uicast address of an LPN to receive a packet

    \par Description
    This routine checks if destination address in a received packet matches
    with any of the known subscription address of an LPN.

    \param [in]  addr          Address to search
    \param [in] lpn_handle      An LPN Handle, which will be filled
                               if match found

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_is_valid_lpn_uincast_address
(
    /* IN */  MS_NET_ADDR     addr,
    /* IN */  LPN_HANDLE      lpn_handle
)
{
    API_RESULT    retval;
    UINT32        index;
    /* Lock */
    retval = ACCESS_NO_MATCH;
    /* Not Checking if a valid unicast address */
    index = lpn_handle;
    printf ("[FRND] saddr: 0x%02X, uaddr: 0x%02X\n",
            addr,lpn_element[index].node.addr);

    /* TODO: Friend LPN uincast addresses are maintained now separately. This can be relooked */
    if(lpn_element[index].node.addr == addr)
        retval = API_SUCCESS;

    return retval;
}



/**
    \brief To get Poll Timeout of an LPN

    \par Description
    This routine checks if LPN address is valid and then returns
    Poll Timeout configured for the LPN.

    \param [in]  lpn_addr        LPN Address to search
    \param [out] poll_timeout    Memory where poll timeout of the LPN to be filled
                                 (if match found)

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_get_lpn_polltimeout
(
    /* IN */  MS_NET_ADDR    lpn_addr,
    /* OUT */ UINT32*        poll_timeout
)
{
    UINT16 lpn_index;
    API_RESULT retval;
    retval = API_FAILURE;
    /* Lock */
    TRN_LOCK();
    lpn_index = frnd_search_lpn_element(lpn_addr);

    if (MS_CONFIG_LIMITS(MS_MAX_LPNS) != lpn_index)
    {
        /* The PollTimeout timer value in units of 100 milliseconds */
        *poll_timeout = (lpn_element[lpn_index].poll_timeout - (FRND_POLLTO_FACTOR * 100)) / 100;
        retval = API_SUCCESS;
    }

    /* Unlock */
    TRN_UNLOCK();
    return retval;
}

API_RESULT MS_trn_get_frndship_info(UINT8 role, UINT16 lpn_index, MS_TRN_FRNDSHIP_INFO* node)
{
    TRN_LOCK();

    if (MS_FRND_ROLE_LPN == role)
    {
        /* Copy the information from the node */
        *node = frnd_element.node;
    }
    else
    {
        /* Copy the information from the node */
        *node = lpn_element[lpn_index].node;
    }

    TRN_UNLOCK();
    return API_SUCCESS;
}


/**
    \brief To add the security update information

    \par Description
    This routine updates the security state of the network to all the active
    LPN elements. This will be forwarded to the elements when it polls for the
    next packet available.

    \param [in] subnet_handle    Handle to identitfy the network.
    \param [in] flag             Flag indicating the Key Refresh and IV Update state.
    \param [in] ivindex          Current IV Index of teh network.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_lpn_register_security_update
(
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UCHAR               flag,
    /* IN */ UINT32              ivindex
)
{
    UINT8 index;
    MS_NET_HEADER net_hdr;
    TRN_LPN_ELEMENT* lpn;
    UCHAR pdu[5];
    /* Enqueue the security update in all valid LPN elements */
    net_hdr.saddr = 0x0000;
    net_hdr.daddr = 0x0000;

    /* TODO: Friend LPN addresses are maintained now separately. This can be relooked */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_LPNS); index++)
    {
        /* Reference the element */
        lpn = &lpn_element[index];

        /* If element valid and matches the subnet, enqueue update */
        if ((LPN_ELEMENT_VALID_PERMANENT == lpn->valid) &&
                (subnet_handle == lpn->node.subnet_handle))
        {
            /* Frame the PDU */
            pdu[0] = flag;
            MS_PACK_BE_4_BYTE((pdu + 1), &ivindex);
            TRN_TRC("[FRND] Registering Security Update to LPN Element at Index: %d\n", index);
            /* Unsegmented packet */
            trn_add_to_queue(index, 0x00, &net_hdr, pdu, sizeof(pdu));
        }
    }

    return API_SUCCESS;
}


/**
    \brief To clear information related to all LPNs

    \par Description
    This routine clears information related to all LPNs.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_trn_clear_all_lpn
(
    void
)
{
    UINT32        index;

    /* Lock */

    /* TODO: Friend LPN addresses are maintained now separately. This can be relooked */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_LPNS); index ++)
    {
        frnd_clear_lpn_element((UINT16)index);
    }

    /* Unlock */
    return API_SUCCESS;
}

/**
    \brief

    \par Description


    \param net_hdr
    \param param

    \return void
*/
void trn_handle_frnd_req
(
    /* IN */ MS_NET_HEADER*          net_hdr,
    /* IN */ MS_SUBNET_HANDLE        subnet_handle,
    /* IN */ MS_TRN_FRND_REQ_PARAM* param
)
{
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    API_RESULT retval;
    UCHAR rssi_factor, rcvwin_factor;
    UCHAR rssi, delay;
    TRN_TRC ("[FRND] Friend Request from 0x%04X\n", net_hdr->saddr);
    TRN_TRC ("[FRND] Criteria: 0x%02X, Rx Delay: 0x%02X, Poll Timeout: 0x%08X "
             "LPN Counter: 0x%04X, Num Elts: 0x%02X, Prev Addr: 0x%04X\n",
             param->criteria, param->rx_delay, param->poll_to, param->lpn_counter,
             param->num_elem, param->prev_addr);

    /* Validate TTL is 0 as mandatory */
    if (0 != net_hdr->ttl)
    {
        TRN_INF("[FRND] Dropping packet because TTL != 0\n");
        return;
    }

    /* Validate parameters are in specified range */
    if ((!(param->criteria & 0x07)) ||
            (param->rx_delay < 0x0A) ||
            ((param->poll_to < 0x00000A) || (param->poll_to > 0x34BBFF)) ||
            (param->num_elem == 0))
    {
        /* Invalid Parameter! Dropping. */
        TRN_ERR ("[FRND] Invalid Parameter in Friendship Request. Dropping...\n");
        return;
    }

    /*
        Verify if the requested criteria by the LPN is satisfied
        by the local node as a friend
    */
    retval = frnd_verify_request_criteria (param->criteria, &rssi_factor, &rcvwin_factor);

    if (API_SUCCESS != retval)
    {
        TRN_INF ("[FRND] Mismatch in Friendship Request criteria. Dropping...\n");
        return;
    }

    /*
        Allocate a friend element, return if failure
        This allocation searches for the LPN in the existing list and if already
        present, it resets the friendship details and makes a new allocation.
    */
    lpn_index = frnd_alloc_lpn_element (net_hdr->saddr);

    if (MS_CONFIG_LIMITS(MS_MAX_LPNS) == lpn_index)
    {
        TRN_ERR ("[FRND] Failed to allocate LPN element to handle Friend Request. Dropping...\n");
        return;
    }

    lpn = &lpn_element[lpn_index];
    /*
        Calculate FriendOffer Delay

        Local Delay = ReceiveWindowFactor * ReceiveWindow - RSSIFactor * RSSI

        Where:
            ReceiveWindowFactor is a number from the Friend Request message
            ReceiveWindow is the value to be sent in the corresponding Friend Offer message
            RSSIFactor is a number from the Friend Request message
            RSSI is the received signal strength of the received Friend Request message on the Friend node

        if Local Delay > 100, FriendOffer Delay = (Local Delay) ms, else 100 ms
    */
    rssi = MS_brr_get_packet_rssi();
    delay = ((rcvwin_factor * MS_FRND_RECEIVE_WINDOW) - (rssi_factor * rssi));
    delay = (delay > MS_MIN_FRNDOFFER_DELAY)? delay: MS_MIN_FRNDOFFER_DELAY;
    TRN_INF ("[FRND] RSSI value: 0x%02X, Local Delay: 0x%02X\n", rssi, delay);
    /* Save information to element */
    lpn->rx_delay = param->rx_delay;
    /**
        Updating the Polltimeout value as incoming Polltimeout in PDU is in
        factor of 100ms.
        TODO: update the same in our LPN path and all other places?
    */
    lpn->poll_timeout = (param->poll_to * 100) + (FRND_POLLTO_FACTOR * 100);
    lpn->num_elements = param->num_elem;
    lpn->node.lpn_counter = param->lpn_counter;
    lpn->node.frnd_counter = trn_frnd_counter ++;
    lpn->rssi = rssi;
    lpn->node.subnet_handle = subnet_handle;
    lpn->fsn = 0xFF;
    /*
        Mark element in temporary state. Will move to permanent after friendship
        procedure is completed
    */
    lpn->valid = LPN_ELEMENT_VALID_TEMP;
    /*
        Update the clear context of the element to be used to terminate previous
        friendship after the current friendship establishment
    */
    lpn->clrctx.frnd_addr = param->prev_addr;
    /* Start timer for delay to send offer */
    retval = EM_start_timer
             (
                 &lpn->thandle,
                 (EM_TIMEOUT_MILLISEC | delay),
                 trn_frndreq_rsp_handler,
                 (void*)&lpn_index,
                 sizeof (lpn_index)
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR ("[FRND] Failed to start FrndPoll timer. Retval - 0x%04X\n", retval);
    }
}


void trn_frndreq_rsp_handler (void* args, UINT16 size)
{
    MS_TRN_FRND_OFFER_PARAM frnd_offer;
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    TRN_LOCK_VOID();
    /* Reference the LPN element */
    lpn_index = *((UINT16*)args);
    /* TODO: Check lpn index. Add Assert */
    lpn = &lpn_element[lpn_index];
    /* Reset the timer handle */
    lpn->thandle = EM_TIMER_HANDLE_INIT_VAL;
    TRN_TRC ("[FRND] Friend Offer Timeout Handler. For Src Addr: 0x%04X, Subnet Handle: 0x%04X\n",
             lpn->node.addr, lpn->node.subnet_handle);
    /* Frame the Friend Offer */
    frnd_offer.rx_window = MS_FRND_RECEIVE_WINDOW;
    frnd_offer.queue_size = (UINT8)(MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) + 1); /* Done this for segment cache testing with PTS */
    frnd_offer.sublist_size = (UINT8)MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE);
    frnd_offer.rssi = lpn->rssi;
    frnd_offer.frnd_counter = lpn->node.frnd_counter;
    /* Send the packet with TTL 0 */
    trn_frnd_send (lpn->node.addr, lpn->node.subnet_handle, MS_TRN_CTRL_OPCODE_FRND_OFFER, &frnd_offer, 0);
    /* Get the unicast source address of primary element */
    MS_access_cm_get_primary_unicast_address(&uaddr);
    /* Call to generate the friend credentials */
    MS_access_cm_add_friend_sec_credential
    (
        lpn->node.subnet_handle,
        lpn_index,
        lpn->node.addr,
        uaddr,
        lpn->node.lpn_counter,
        lpn->node.frnd_counter
    );
    /*
        Start timer for 1 second to receive FriendPoll. Stop the timer on
        receiving FriendPoll, else on timeout, discard friendship request
    */
    retval = EM_start_timer
             (
                 &lpn->thandle,
                 (EM_TIMEOUT_MILLISEC | MS_TRN_INITIAL_FRNDPOLL_TIMEOUT),
                 trn_frndpoll_timeout_handler,
                 (void*)&lpn_index,
                 sizeof(lpn_index)
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR ("[FRND] Failed to start FrndPoll timer. Retval - 0x%04X\n", retval);
    }

    /* Unlock */
    TRN_UNLOCK_VOID();
}


/**
    \brief

    \par Description


    \param net_hdr
    \param param

    \return void
*/
void trn_handle_frnd_poll
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_TRN_FRND_POLL_PARAM*   param
)
{
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    API_RESULT retval;
    TRN_TRC("[FRND] Friend Poll from 0x%04X\n", net_hdr->saddr);
    TRN_TRC("[FRND] FSN: 0x%02X\n", param->fsn);

    /* Validate TTL is 0 as mandatory */
    if (0 != net_hdr->ttl)
    {
        TRN_ERR("[FRND] Dropping packet because TTL != 0\n");
        return;
    }

    /* If prohibited bits are set in parameter, do not process */
    if (param->fsn & 0xFE)
    {
        TRN_ERR("[FRND] Dropping packet of prohibited FSN - 0x%02X\n", param->fsn);
        return;
    }

    /* Lookup the LPN index */
    lpn_index = subnet_handle - MS_CONFIG_LIMITS(MS_MAX_SUBNETS);

    /* Verify the subnet handle for the source address */
    if (lpn_element[lpn_index].node.addr != net_hdr->saddr)
    {
        TRN_ERR("[FRND] Failed to find LPN element to handle Friend Poll. Dropping...\n");
        return;
    }

    lpn = &lpn_element[lpn_index];

    /* Stop the PollTimer. This should be running. */
    if (EM_TIMER_HANDLE_INIT_VAL == lpn->thandle)
    {
        TRN_ERR ("[FRND] *** Poll LPN element with Poll Timer not running cannot happen. Dropping... ***\n");
        return;
    }

    /* Stop the PollTimer */
    EM_stop_timer (&lpn->thandle);
    /*Stop beacon advertise by hq*/
    //MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NODEID,BRR_BCON_ACTIVE);
    TRN_TRC("[FRND] FSN Expected: 0x%02X, FSN Received: 0x%02X\n", lpn->fsn, param->fsn);

    /* Is the received fsn the same as expected? */
    if (lpn->fsn == param->fsn)
    {
        /* Yes. Means previous Update is received. Clear queue */
        trn_dequeue_queue_head(&lpn->friend_queue[0]);
    }
    else
    {
        lpn->fsn = param->fsn;
    }

    TRN_INF ("[FRND] Start PollRsp timer. Timeout - %ld\n", lpn->rx_delay);
    /* Start the timer for RxDelay timeout to send the FrndUpdate */
    retval = EM_start_timer
             (
                 &lpn->thandle,
                 (EM_TIMEOUT_MILLISEC | lpn->rx_delay),
                 trn_frndpoll_rsp_handler,
                 (void*)&lpn_index,
                 sizeof(lpn_index)
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR ("[FRND] Failed to start FrndUpdate timer. Retval - 0x%04X\n", retval);
    }
}


void trn_frndpoll_rsp_handler (void* args, UINT16 size)
{
    MS_TRN_FRND_UPDATE_PARAM frnd_update;
    TRN_LPN_ELEMENT* lpn;
    UINT16        lpn_index;
    API_RESULT retval;
    UINT16 num_msgs;
    MS_NET_HEADER  net_hdr;
    UINT8          pdu[32];
    UINT8          pdu_len;
    MS_IGNORE_UNUSED_PARAM(size);
    num_msgs = 0;
    /* Lock */
    TRN_LOCK_VOID();
    /* Reference the LPN element */
    lpn_index = *((UINT16*)args);
    lpn = &lpn_element[lpn_index];
    /* Reset the timer handle */
    lpn->thandle = EM_TIMER_HANDLE_INIT_VAL;
    TRN_TRC ("[FRND] Friend Update send handler. For Src Addr: 0x%04X, Subnet Handle: 0x%04X\n",
             lpn->node.addr, lpn->node.subnet_handle);
    /* See if there is pending data in the queue */
    retval = trn_get_from_queue
             (
                 (LPN_HANDLE)lpn_index,
                 &net_hdr,
                 pdu,
                 &pdu_len
             );

    /*
        Check the state of the element. Can be the following two,
        - Temporary Valid: This is the first friend poll after offer. Send
         Update and start procedure to clear the friendship of the LPN from
         the previous friend.
        - Permanent Valid: Give any security update if available for the LPN or
         give the next message from queue.
    */
    if ((LPN_ELEMENT_VALID_TEMP == lpn->valid) ||
            (API_SUCCESS != retval))
    {
        UINT8 kr_phase, iv_phase;
        /* IV Index and Update Flag */
        MS_access_cm_get_iv_index(&frnd_update.ivi, &iv_phase);
        iv_phase &= 0x01;
        frnd_update.ivi += iv_phase;
        /* Key Refresh Flag */
        MS_access_cm_get_key_refresh_phase(lpn->node.subnet_handle, &kr_phase);
        /* Pack friend update information - TODO */
        frnd_update.flags = (MS_ACCESS_KEY_REFRESH_PHASE_2 == kr_phase) ? 0x01 : 0x00;
        frnd_update.flags |= (iv_phase << 1);
        frnd_update.md = 0x00;
        TRN_TRC("[FRND] Sending Friend Update to LPN.\n");
        /* Send the packet with TTL 0 */
        trn_frnd_send
        (
            lpn->node.addr,
            (lpn_index + MS_CONFIG_LIMITS(MS_MAX_SUBNETS)),
            MS_TRN_CTRL_OPCODE_FRND_UPDATE,
            &frnd_update,
            0
        );
        /* Indicate that no data was sent from queue */
        lpn->fsn = 0xFF;

        if (LPN_ELEMENT_VALID_TEMP == lpn->valid)
        {
            lpn->valid = LPN_ELEMENT_VALID_PERMANENT;
            /* Update current friendship role as FRIEND */
            MS_access_cm_set_friendship_role(MS_FRND_ROLE_FRIEND);

            /* Reset the LPN Message Queue and Subscription List - TODO */

            /* Start the friend clear procedure, if valid address */
            if (0x0000 != lpn->clrctx.frnd_addr)
            {
                TRN_TRC("[FRND] Start Friend Clear for Addr - 0x%04X.\n",
                        lpn->clrctx.frnd_addr );
                lpn->clrctx.retry_tmo = LPN_CLEAR_RETRY_TIMEOUT_INITIAL;
                trn_frndclear(lpn_index, LPN_CLEAR_RETRY_TIMEOUT_INITIAL);
            }
        }
    }
    else
    {
        /* Send message from queue head to the LPN */
        MS_SUBNET_HANDLE   subnet_handle;
        MS_BUFFER          buffer;

        /* Check if the enqueued message is a friend update. Indicated by Src Address as Unassigned */
        if (0x0000 == net_hdr.saddr)
        {
            /* Send Friend Update with the parameters */
            frnd_update.flags = pdu[0];
            MS_UNPACK_BE_4_BYTE(&(frnd_update.ivi), &(pdu[1]));
            frnd_update.md = 0x01;
            TRN_TRC("[FRND] Sending Friend Update from cache to LPN.\n");
            /* Send the packet with TTL 0 */
            trn_frnd_send
            (
                lpn->node.addr,
                (lpn_index + MS_CONFIG_LIMITS(MS_MAX_SUBNETS)),
                MS_TRN_CTRL_OPCODE_FRND_UPDATE,
                &frnd_update,
                0
            );
        }
        else
        {
            /* Send Network Packet */
            subnet_handle = (lpn_index + MS_CONFIG_LIMITS(MS_MAX_SUBNETS));
            buffer.payload = pdu;
            buffer.length = pdu_len;

            if (2 < net_hdr.ttl)
            {
                net_hdr.ttl--;
            }

            TRN_TRC("[FRND] Sending message to LPN from cache. Length - %d\n", pdu_len);
            retval = MS_net_send_pdu
                     (
                         &net_hdr,
                         subnet_handle,
                         &buffer,
                         MS_FALSE
                     );
        }

        /* Set the next expected FSN */
        lpn->fsn ^= 0x01;
    }

    TRN_INF("[FRND] Start FrndPoll timer. Timeout - %ld\n", lpn->poll_timeout);
    /*
        Start timer for PollTimeout to receive FriendPoll. Stop the timer on
        receiving FriendPoll, else on timeout, discard friendship request
    */
    /*Start beacon advertise by hq*/
    //MS_proxy_server_adv_start(lpn->node.subnet_handle,MS_PROXY_NODE_ID_ADV_MODE);
    retval = EM_start_timer
             (
                 &lpn->thandle,
                 (EM_TIMEOUT_MILLISEC | lpn->poll_timeout),
                 trn_frndpoll_timeout_handler,
                 (void*)&lpn_index,
                 sizeof(lpn_index)
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR ("[FRND] Failed to start FrndPoll timer. Retval - 0x%04X\n", retval);
    }

    /* Unlock */
    TRN_UNLOCK_VOID();
}


void trn_frndpoll_timeout_handler (void* args, UINT16 size)
{
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    TRN_LOCK_VOID();
    /* Reference the LPN element */
    lpn_index = *((UINT16*)args);
    /* TODO: Check lpn index. Add Assert */
    lpn = &lpn_element[lpn_index];
    /* Reset the timer handle */
    lpn->thandle = EM_TIMER_HANDLE_INIT_VAL;
    TRN_TRC ("[FRND] Friend Poll Timeout Handler. For Src Addr: 0x%04X, Subnet Handle: 0x%04X\n",
             lpn->node.addr, lpn->node.subnet_handle);
    printf ("[FRND] Friend Poll Timeout Handler. For Src Addr: 0x%04X, Subnet Handle: 0x%04X\n",
            lpn->node.addr, lpn->node.subnet_handle);
    /*
        Poll Timeout fired for element. Terminate friendship and clear the
        element
    */
    TRN_TRC("[FRND] Clear the LPN element to terminate friendship\n");
    frnd_clear_lpn_element (lpn_index);
    /* Update current friendship role as INVALID */
    MS_access_cm_set_friendship_role(MS_FRND_ROLE_INVALID);
    /* Unlock */
    TRN_UNLOCK_VOID();
}


/* Handle Friend Subscription List Add/Remove */
void trn_handle_frnd_subscription_list_add_remove
(
    /* IN */ MS_NET_HEADER*                net_hdr,
    /* IN */ MS_SUBNET_HANDLE              subnet_handle,
    /* IN */ MS_TRN_FRND_MANAGE_PARAM*     param
)
{
    API_RESULT retval;
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    MS_TRN_FRND_SUBSCRN_LIST_CNF_PARAM response;
    UINT32 index, sub_list_index;
    UINT8* paddr;
    TRN_TRC("[FRND] Friend Subscription List Add/Remove from 0x%04X\n", net_hdr->saddr);
    TRN_TRC("[FRND] Number of Addresses: 0x%04X\n", param->num_addr);
    /* Lookup the LPN index */
    lpn_index = subnet_handle - MS_CONFIG_LIMITS(MS_MAX_SUBNETS);

    /* TODO: Do we need to check the address match */
    /* Verify the subnet handle for the source address */
    if (lpn_element[lpn_index].node.addr != net_hdr->saddr)
    {
        TRN_ERR("[FRND] Failed to find LPN element to handle Friend Subscription List Add/Remove. Dropping...\n");
        return;
    }

    lpn = &lpn_element[lpn_index];
    /* Stop and reset the timer */
    EM_stop_timer (&lpn->thandle);

    /* Add/Remove from Subscription list */
    if (MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_ADD == param->opcode)
    {
        sub_list_index = 0;

        for (index = 0; ((index < param->num_addr) && (MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE) != sub_list_index)); index++)
        {
            /* Find next free space */
            while (sub_list_index < MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE))
            {
                /* Check if the subscription list element is empty */
                if (MS_NET_ADDR_UNASSIGNED == lpn->subscription_list[sub_list_index])
                {
                    paddr = ((UINT8*)(param->addr_list)) + (index << 1);
                    MS_UNPACK_BE_2_BYTE(&(lpn->subscription_list[sub_list_index]), paddr);
                    break;
                }

                sub_list_index++;
            }
        }
    }
    else
    {
        MS_NET_ADDR addr;

        for (index = 0; index < param->num_addr; index++)
        {
            paddr = ((UINT8*)(param->addr_list)) + (index << 1);
            MS_UNPACK_BE_2_BYTE(&addr, paddr);

            /* Find next match */
            for (sub_list_index = 0; sub_list_index < MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE); sub_list_index++)
            {
                /* Check if the subscription list element matches */
                if (lpn->subscription_list[sub_list_index] == addr)
                {
                    lpn->subscription_list[sub_list_index] = MS_NET_ADDR_UNASSIGNED;
                    break;
                }
            }
        }
    }

    /* Send response */
    response.txn_num = param->txn_num;
    trn_frnd_send(lpn->node.addr, subnet_handle, MS_TRN_CTRL_OPCODE_FRND_SUBSCRN_LIST_CNF, &response, 0);
    /* Start timer for Poll Timeout */
    retval = EM_start_timer
             (
                 &lpn->thandle,
                 (EM_TIMEOUT_MILLISEC | lpn->poll_timeout),
                 trn_frndpoll_timeout_handler,
                 (void*)&lpn_index,
                 sizeof(lpn_index)
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR ("[FRND] Failed to start FrndPoll tracking timer. Retval - 0x%04X\n", retval);
    }
}


API_RESULT  frnd_verify_request_criteria (UCHAR criteria, UCHAR* rssi, UCHAR* rcvwin)
{
    UCHAR rssif, rcvwinf;
    /* Common RSSI and RcvWin factors as in spec, multiplied by 10 */
    UCHAR factor[] = {10, 15, 20, 25};
    /* TODO - Criteria check */
    /* Calculate the RSSI and ReceiveWindow Factors */
    rssif = ((criteria & MS_FRNDREQ_RSSIFACTOR_MASK) >> MS_FRNDREQ_RSSIFACTOR_OFFSET);
    *rssi = factor[rssif] / 10U;
    rcvwinf = ((criteria & MS_FRNDREQ_RCVWINFACTOR_MASK) >> MS_FRNDREQ_RCVWINFACTOR_OFFSET);
    *rcvwin = factor[rcvwinf] / 10U;
    return API_SUCCESS;
}


/**
    \brief Add message to friend queue.

    \par Description Add message to friend queue.

    \param [in] lpn_index    Associated LPN Handle/Index
    \param [in] pkt_type     Type of packet to be enqueued
                             - 0x00: Unsegmented/Only One Segment (final packet)
                             - 0x01: Start or Continue of Segment
                             - 0x03: Final Segment
                             - 0x04: Ack (as it requires special processing/overwriting previous ack)
    \param [in] net_hdr      Received Network Packet Header
    \param [in] pdu          Received PDU
    \param [in] pdu_len      PDU Length

    \return API_RESULT
*/
API_RESULT trn_add_to_queue
(
    /* IN */ LPN_HANDLE       lpn_index,
    /* IN */ UINT8            pkt_type,
    /* IN */ MS_NET_HEADER*   net_hdr,
    /* IN */ UINT8*           pdu,
    /* IN */ UINT8            pdu_len
)
{
    TRN_LPN_ELEMENT* lpn;
    lpn = &lpn_element[lpn_index];
    TRN_TRC("[FRND] Adding Packet type - 0x%02X\n", pkt_type);

    switch (pkt_type)
    {
    case 0: /* Unsegmented or single segmented */
        trn_add_to_queuelet
        (
            &lpn->friend_queue[0],
            net_hdr,
            pdu,
            pdu_len
        );
        break;

    case 1: /* Non-final segments */
        trn_add_to_queuelet
        (
            &lpn->friend_queue[1],
            net_hdr,
            pdu,
            pdu_len
        );
        break;

    case 3: /* Final segment */
    {
        TRN_FRN_Q_ELEMENT* q_elem;
        /* Dequeue from [#1] and enqueue to [#0] */
        /* TODO: Assuming segments of same packet */
        MS_LOOP_FOREVER()
        {
            q_elem = trn_dequeue_queue_head(&lpn->friend_queue[1]);

            if (NULL == q_elem)
            {
                break;
            }

            trn_add_to_queuelet
            (
                &lpn->friend_queue[0],
                &q_elem->net_hdr,
                q_elem->pdu,
                q_elem->ltrn_pkt_length
            );
        }
        /* Enqueue the final frame */
        trn_add_to_queuelet
        (
            &lpn->friend_queue[0],
            net_hdr,
            pdu,
            pdu_len
        );
        break;
    }

    case 4: /* Ack */
    {
        /* Search for Ack match in the queue and update */
        trn_update_lpn_ack
        (
            &lpn->friend_queue[0],
            net_hdr,
            pdu,
            pdu_len
        );
    }
    break;
    }

    return API_SUCCESS;
}

/** TODO: */
API_RESULT trn_add_to_queuelet
(
    /* IN */ TRN_FRIEND_QUEUE*   queue,
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ UINT8*              pdu,
    /* IN */ UINT8               pdu_len
)
{
    UINT16 end;
    TRN_TRC("[FRND] Add to Friend Queuelet. SRC:0x%04X, IVI:0x%02X, SEQ:0x%08X, NID:0x%02X\n",
            net_hdr->saddr, net_hdr->ivi, net_hdr->seq_num, net_hdr->nid);

    /* DO not cache if ttl < 2 */
    if (2 > net_hdr->ttl)
    {
        TRN_ERR("Discarding message with TTL < 2\n");
        return API_FAILURE;
    }

    TRN_INF("[FRND] Queue Start 0x%04X, Size: 0x%04X\n",
            queue->queue_start, queue->queue_size);

    /* If full, add to the start which is oldest and move start to next */
    if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) == queue->queue_size)
    {
        end = queue->queue_start;
        /* Move to the next element in the queue */
        queue->queue_start++;

        /* Wrap around condition */
        if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) == queue->queue_start)
        {
            queue->queue_start = 0;
        }
    }
    else
    {
        end = queue->queue_start + queue->queue_size;

        if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) <= end)
        {
            end -= MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE);
        }
    }

    TRN_TRC("[FRND] Saving at Queue Index: 0x%04X\n", end);
    TRN_TRC("[FRND] Net Hdr: SAddr: 0x%04X, DAddr: 0x%04X, SN: 0x%08X, CTL: 0x%02X, TTL: 0x%02X, NID: 0x%02X, IVI:0x%02X\n",
            net_hdr->saddr, net_hdr->daddr, net_hdr->seq_num, net_hdr->ctl, net_hdr->ttl, net_hdr->nid, net_hdr->ivi);
    TRN_debug_dump_bytes(pdu, pdu_len);
    /* TODO: Check PDU and Length */
    queue->queue[end].net_hdr = *net_hdr;
    queue->queue[end].ltrn_pkt_length = pdu_len;
    EM_mem_copy(queue->queue[end].pdu, pdu, pdu_len);
    queue->queue[end].is_ack = MS_FALSE;

    if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) != queue->queue_size)
    {
        queue->queue_size++;
    }

    return API_SUCCESS;
}

/** TODO: */
/* Search for Ack match in the queue and update */
API_RESULT trn_update_lpn_ack
(
    /* IN */ TRN_FRIEND_QUEUE*   queue,
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ UINT8*              pdu,
    /* IN */ UINT8               pdu_len
)
{
    UINT16 index, current;
    UINT16 seq_zero, seq_zero_stored;
    UINT8  obo, obo_stored;
    /**
        TODO: Do we need a separate is_ack field?
        This can be extracted from the first octet of the LTRN pkt.
    */
    /* Extract Ack Fields */
    /* Extract Sequence Zero */
    MS_UNPACK_BE_2_BYTE(&seq_zero, &pdu[1]);
    obo = seq_zero >> 15;
    seq_zero &= 0x7FFF;
    /* Discard RFU */
    seq_zero >>= 2;
    /* Scan through the entire list */
    current = queue->queue_start;

    for (index = 0; index < queue->queue_size; index++)
    {
        /* Check if current element is an Ack */
        if (0x01 == queue->queue[current].is_ack)
        {
            /* Extract Sequence Zero */
            MS_UNPACK_BE_2_BYTE(&seq_zero_stored, &queue->queue[current].pdu[1]);
            obo_stored = seq_zero_stored >> 15;
            seq_zero_stored &= 0x7FFF;
            /* Discard RFU */
            seq_zero_stored >>= 2;

            /* Update and break */
            if ((net_hdr->saddr == queue->queue[current].net_hdr.saddr) &&
                    (seq_zero == seq_zero_stored) && (obo == obo_stored))
            {
                /* Match Found */
                break;
            }
        }

        /* Move to the next one */
        current++;

        /* Wrap around condition */
        if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) == current)
        {
            current = 0;
        }
    }

    /* Add to the current/end */
    queue->queue[current].net_hdr = *net_hdr;
    EM_mem_copy(&queue->queue[current].pdu[0], pdu, pdu_len);
    queue->queue[current].ltrn_pkt_length = pdu_len;
    queue->queue[current].is_ack = 0x01;

    /* Check if size to be updated */
    if (queue->queue_size == index)
    {
        /* Update size and start */
        if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) == queue->queue_size)
        {
            /* Move to the next element in the queue */
            queue->queue_start++;

            /* Wrap around condition */
            if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) == queue->queue_start)
            {
                queue->queue_start = 0;
            }
        }
        else
        {
            queue->queue_size++;
        }
    }

    return API_SUCCESS;
}

/**
    \brief Add message to friend queue.

    \par Description Add message to friend queue.

    \param [in] lpn_index    Associated LPN Handle/Index
    \param [in] net_hdr      Received Network Packet Header
    \param [in] pdu          Received PDU
    \param [in] pdu_len      PDU Length

    \return API_RESULT
*/
API_RESULT trn_get_from_queue
(
    /* IN */ LPN_HANDLE       lpn_index,
    /* IN */ MS_NET_HEADER*   net_hdr,
    /* IN */ UINT8*           pdu,
    /* IN */ UINT8*           pdu_len
)
{
    TRN_LPN_ELEMENT* lpn;
    TRN_TRC("[FRND] Get from Friend Queue. LPN Handle:0x%02X\n", lpn_index);
    lpn = &lpn_element[lpn_index];
    TRN_TRC("[FRND] Queue Start 0x%04X, Size: 0x%04X\n",
            lpn->friend_queue[0].queue_start, lpn->friend_queue[0].queue_size);

    if (0 == lpn->friend_queue[0].queue_size)
    {
        return API_FAILURE;
    }
    else
    {
        *net_hdr = lpn->friend_queue[0].queue[lpn->friend_queue[0].queue_start].net_hdr;
        EM_mem_copy
        (
            pdu,
            lpn->friend_queue[0].queue[lpn->friend_queue[0].queue_start].pdu,
            lpn->friend_queue[0].queue[lpn->friend_queue[0].queue_start].ltrn_pkt_length
        );
        *pdu_len = lpn->friend_queue[0].queue[lpn->friend_queue[0].queue_start].ltrn_pkt_length;
        TRN_TRC("[FRND] Net Hdr: SAddr: 0x%04X, DAddr: 0x%04X, SN: 0x%08X, CTL: 0x%02X, TTL: 0x%02X, NID: 0x%02X, IVI:0x%02X\n",
                net_hdr->saddr, net_hdr->daddr, net_hdr->seq_num, net_hdr->ctl, net_hdr->ttl, net_hdr->nid, net_hdr->ivi);
        TRN_debug_dump_bytes(pdu, *pdu_len);
        return API_SUCCESS;
    }
}

/** Remove Head from Queue */
TRN_FRN_Q_ELEMENT*   trn_dequeue_queue_head
(
    /* IN */ TRN_FRIEND_QUEUE*   queue
)
{
    TRN_FRN_Q_ELEMENT* head;
    head = NULL;

    if (0 != queue->queue_size)
    {
        TRN_TRC ("[FRND] Dequeue element at %d from queue\n", queue->queue_start);
        head = &queue->queue[queue->queue_start];
        queue->queue_size--;
        queue->queue_start++;

        /* TODO: Ensure MS_FRIEND_MESSAGEQUEUE_SIZE is (2^n). Else take care */
        /* queue->queue_start &= (MS_FRIEND_MESSAGEQUEUE_SIZE - 1); */
        if (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) == queue->queue_start)
        {
            queue->queue_start = 0;
        }
    }

    return head;
}


UINT16 frnd_alloc_lpn_element (MS_NET_ADDR addr)
{
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    UINT8 i;
    TRN_TRC ("[FRND] Allocate an LPN element for Addr: 0x%04X\n", addr);
    /*
        Search the element. If already available, clear the element to terminate
        friendship and reuse the same.

        This choice of implementation is due to the text in Sec. 3.6.6.3.1 of
        the Mesh specification that states -
        If the source address of the Friend Request message is an address of a
        Low Power node that is currently in a friendship with the Friend node,
        then the Friend node shall also consider the existing friendship with
        that Low Power node terminated.
    */
    lpn_index = frnd_search_lpn_element(addr);

    if (MS_CONFIG_LIMITS(MS_MAX_LPNS) != lpn_index)
    {
        TRN_TRC ("[FRND] Address found in LPN list. Reusing...\n");
        lpn = &lpn_element[lpn_index];
        /* Clear element */
        frnd_clear_lpn_element(lpn_index);
        /* Assign address to the element */
        lpn->node.addr = addr;
        return lpn_index;
    }

    /* Allocate an available element */
    for (i = 0; i < MS_CONFIG_LIMITS(MS_MAX_LPNS); i++)
    {
        if (LPN_ELEMENT_INVALID == lpn_element[i].valid)
        {
            TRN_TRC ("[FRND] Free element found at index %d\n", i);
            /* Populate the element with LPN address and return the reference */
            lpn_element[i].node.addr = addr;
            return i;
        }
    }

    TRN_ERR ("[FRND] No free element to allocate\n");
    return MS_CONFIG_LIMITS(MS_MAX_LPNS);
}


void frnd_clear_lpn_element (UINT16 lpn_index)
{
    TRN_LPN_ELEMENT* lpn;
    UINT8 i;
    /* Reference the element */
    lpn = &lpn_element[lpn_index];

    /* Stop the timers if running in the element */
    if (EM_TIMER_HANDLE_INIT_VAL != lpn->thandle)
    {
        EM_stop_timer(&lpn->thandle);
    }

    if (EM_TIMER_HANDLE_INIT_VAL != lpn->clrctx.thandle)
    {
        EM_stop_timer(&lpn->clrctx.thandle);
    }

    /* Free any allocated memory in the message queue - TODO */
    /* Clear the associated keys */
    MS_access_cm_delete_friend_sec_credential(lpn->node.subnet_handle, lpn_index);

    /* Clear the message queue */
    for (i = 0; i < 2; i++)
    {
        lpn->friend_queue[i].queue_size = 0;
        lpn->friend_queue[i].queue_start = 0;
        EM_mem_set
        (
            lpn->friend_queue[i].queue,
            0,
            (MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE) * sizeof(TRN_FRN_Q_ELEMENT))
        );
    }

    /* Clear the subscription list */
    EM_mem_set
    (
        lpn->subscription_list,
        0,
        (MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE) * sizeof(MS_NET_ADDR))
    );
    /* Reset the element */
    lpn->valid = LPN_ELEMENT_INVALID;
    lpn->thandle = EM_TIMER_HANDLE_INIT_VAL;
    lpn->clrctx.thandle = EM_TIMER_HANDLE_INIT_VAL;
}


UINT16 frnd_search_lpn_element (MS_NET_ADDR addr)
{
    UINT8 i;
    TRN_TRC ("[FRND] Search for LPN element with Addr: 0x%04X\n", addr);

    /* Search the element in list */
    for (i = 0; i < MS_CONFIG_LIMITS(MS_MAX_LPNS); i++)
    {
        if ((LPN_ELEMENT_INVALID != lpn_element[i].valid) &&
                (addr == lpn_element[i].node.addr))
        {
            TRN_TRC ("[FRND] Element found at index %d\n", i);
            return i;
        }
    }

    TRN_ERR ("[FRND] Element not found\n");
    return MS_CONFIG_LIMITS(MS_MAX_LPNS);
}


/* Handle Friend Clear */
void trn_handle_frnd_clear
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_TRN_FRND_CLEAR_PARAM* param
)
{
    UINT16 lpn_index;
    lpn_index = frnd_search_lpn_element(param->lpn_addr);

    if (MS_CONFIG_LIMITS(MS_MAX_LPNS) != lpn_index)
    {
        TRN_TRC("[FRND] Clear the LPN element at index %d to terminate friendship\n", lpn_index);
        frnd_clear_lpn_element(lpn_index);
        /* Send Confirmation */
        trn_frnd_send(net_hdr->saddr, subnet_handle, MS_TRN_CTRL_OPCODE_FRND_CLEAR_CNF, param, net_hdr->ttl);
    }

    return;
}
#endif /* MS_FRIEND_SUPPORT */


/* Handle Friend Clear Confirmation */
void trn_handle_frnd_clear_cnf
(
    /* IN */ MS_NET_HEADER*                net_hdr,
    /* IN */ MS_SUBNET_HANDLE              subnet_handle,
    /* IN */ MS_TRN_FRND_CLEAR_CNF_PARAM* param
)
{
    UINT8 role;
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    MS_IGNORE_UNUSED_PARAM(net_hdr);
    MS_IGNORE_UNUSED_PARAM(param);
    TRN_TRC("[FRND] Friend Clear Cnf from 0x%04X\n", net_hdr->saddr);
    /* Get the role, Give callback only if LPN */
    MS_access_cm_get_friendship_role(&role);

    if ((MS_FRND_ROLE_LPN == role) &&
            (NULL != frnd_cb))
    {
        /* Unlock */
        TRN_UNLOCK_VOID();
        frnd_cb(subnet_handle, MS_TRN_FRIEND_CLEAR_CNF, API_SUCCESS);
        /* Lock */
        TRN_LOCK_VOID();
    }
    else
    {
        /* Lookup the LPN index */
        lpn_index = subnet_handle - MS_CONFIG_LIMITS(MS_MAX_SUBNETS);

        /* Verify the subnet handle for the source address */
        if (lpn_element[lpn_index].node.addr != net_hdr->saddr)
        {
            TRN_ERR("[FRND] Failed to find LPN element to handle Friend Clear Cnf. Dropping...\n");
            return;
        }

        lpn = &lpn_element[lpn_index];

        /* Stop the timer if running */
        if (EM_TIMER_HANDLE_INIT_VAL != lpn->clrctx.thandle)
        {
            EM_stop_timer(&lpn->clrctx.thandle);
        }
    }
}

void trn_frndclear_retry_timeout_handler (void* args, UINT16 size)
{
    UINT32 tmo;
    TRN_LPN_ELEMENT* lpn;
    UINT16            lpn_index;
    MS_IGNORE_UNUSED_PARAM(size);
    /* Lock */
    TRN_LOCK_VOID();
    /* Reference the LPN element */
    lpn_index = *((UINT16*)args);
    lpn = &lpn_element[lpn_index];
    /* Reset the timer handle */
    lpn->clrctx.thandle = EM_TIMER_HANDLE_INIT_VAL;
    TRN_TRC ("[FRND] Friend Clear Retry Timeout handler. For Src Addr: 0x%04X, Frnd Addr: 0x%02X, Subnet Handle: 0x%04X\n",
             lpn->node.addr, lpn->clrctx.frnd_addr, lpn->node.subnet_handle);

    /*
        Check if the timeout is fired for the procedure timeout which is
        (2 * Polltimeout)
    */
    if (0 == lpn->clrctx.retry_tmo)
    {
        /* Procedure is complete */
        TRN_INF ("[FRND] Friendship Clear Procedure Complete on Timeout\n");
    }
    else
    {
        /* Double the existing retry timeout */
        tmo = (lpn->clrctx.retry_tmo << 1);

        /*
            If new value of retry timeout is greater than the difference between
            the (polltimeout * 2) and current retry timeout, set the difference as the
            next retry timeout for timer, and clear the context retry timeout to
            indicate procedure completion when this timer fires
        */
        if (tmo >= (lpn->poll_timeout << 1))
        {
            tmo = (lpn->poll_timeout << 1) - lpn->clrctx.retry_tmo;
            lpn->clrctx.retry_tmo = 0;
        }
        else
        {
            lpn->clrctx.retry_tmo = tmo;
        }

        /* Restart friend clear procedure */
        trn_frndclear (lpn_index, tmo);
    }

    /* Unlock */
    TRN_UNLOCK_VOID();
}


void trn_frndclear(UINT16 lpn_index, UINT32 timeout)
{
    TRN_LPN_ELEMENT* lpn;
    MS_TRN_FRND_CLEAR_PARAM frnd_clear;
    API_RESULT retval;
    lpn = &lpn_element[lpn_index];
    /* Frame clear */
    frnd_clear.lpn_addr = lpn->node.addr;
    frnd_clear.lpn_counter = lpn->node.lpn_counter;
    TRN_TRC("[FRND] Send FrndClear to 0x%04X. LPN Addr - 0x%04X, LPN Counter - 0x%04X\n",
            lpn->clrctx.frnd_addr, lpn->node.addr, lpn->node.lpn_counter);
    /* Send the packet with TTL 0 */
    trn_frnd_send (lpn->clrctx.frnd_addr, lpn->node.subnet_handle, MS_TRN_CTRL_OPCODE_FRND_CLEAR, &frnd_clear, 0);
    /* Start retry timer */
    retval = EM_start_timer
             (
                 &lpn->clrctx.thandle,
                 (EM_TIMEOUT_MILLISEC | timeout),
                 trn_frndclear_retry_timeout_handler,
                 (void*)&lpn_index,
                 sizeof (lpn_index)
             );

    if (API_SUCCESS != retval)
    {
        TRN_ERR("[FRND] Failed to start FrndClear Retry timer. Retval - 0x%04X\n", retval);
    }
}


API_RESULT trn_frnd_send (MS_NET_ADDR addr, MS_SUBNET_HANDLE subnet_handle, UINT8 opcode, void* pstruct, UINT8 ttl)
{
    API_RESULT retval;
    MS_NET_ADDR uaddr;
    /* Unlock */
    TRN_TRC("[FRND 0x%04X] Sending friendship control PDU for opcode 0x%02X\n", subnet_handle, opcode);
    /* Get the unicast source address of primary element */
    MS_access_cm_get_primary_unicast_address(&uaddr);
    /* Unlock */
    TRN_UNLOCK();
    /* Send over Transport */
    retval = MS_trn_send_control_pdu (uaddr, addr, subnet_handle, ttl, opcode, pstruct);
    /* Lock */
    TRN_LOCK();

    if (API_SUCCESS != retval)
    {
        TRN_ERR(
            "[FRND]: Failed. Retval - 0x%04X\n", retval);
    }

    /* Lock */
    return retval;
}

#endif /* (defined MS_FRIEND_SUPPORT || defined MS_LPN_SUPPORT) */

