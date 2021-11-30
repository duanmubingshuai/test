
/**
    \file trn_internal.h

    Module Internal Header File contains structure definitions including tables
    maintained by the module
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_TRN_INTERNAL_
#define _H_TRN_INTERNAL_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_access_api.h"
#include "MS_trn_api.h"
#include "MS_ltrn_api.h"
#include "MS_net_api.h"

/* --------------------------------------------- Global Definitions */

#ifdef TRN_NO_DEBUG
    #define TRN_ERR          EM_debug_null
#else /* TRN_NO_DEBUG */
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define TRN_ERR
    #else
        #define TRN_ERR(...)     EM_debug_error(MS_MODULE_ID_TRN, __VA_ARGS__)
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* TRN_NO_DEBUG */

#ifdef TRN_DEBUG
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define TRN_TRC
        #define TRN_INF

        #define TRN_debug_dump_bytes(data, datalen)

    #else
        #define TRN_TRC(...)     EM_debug_trace(MS_MODULE_ID_TRN,__VA_ARGS__)
        #define TRN_INF(...)     EM_debug_info(MS_MODULE_ID_TRN,__VA_ARGS__)

        #define TRN_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_TRN, (data), (datalen))

    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* TRN_DEBUG */
    #define TRN_TRC          EM_debug_null
    #define TRN_INF          EM_debug_null

    #define TRN_debug_dump_bytes(data, datalen)

#endif /* TRN_DEBUG */

/**
    Locks the TRN Mutex which prevents any global variable being
    overwritten by any function. It returns an error if mutex lock fails.
*/
#define TRN_LOCK()\
    MS_MUTEX_LOCK(trn_mutex, TRN)

/**
    Locks the TRN_mutex which prevents any global variable being
    overwritten by any function. To be used in void function as it
    returns no error.
*/
#define TRN_LOCK_VOID()\
    MS_MUTEX_LOCK_VOID(trn_mutex, TRN)

/**
    Unlocks the TRN_mutex which realeses the global variables
    to be written into. It returns an error if mutex unlock fails.
*/
#define TRN_UNLOCK()\
    MS_MUTEX_UNLOCK(trn_mutex, TRN)

/**
    Unlocks the TRN_mutex which realeses the global variables
    to be written into. To be used in void functions as it returns
    no error.
*/
#define TRN_UNLOCK_VOID()\
    MS_MUTEX_UNLOCK_VOID(trn_mutex, TRN)


#define trn_alloc_mem(size)\
    EM_alloc_mem(size)

#define trn_free_mem(ptr)\
    EM_free_mem(ptr)

#ifndef TRN_NO_NULL_PARAM_CHECK
/** Null Check of Transport API Parameters */
#define TRN_NULL_CHECK(x) \
    if (NULL == (x)) \
    { \
        TRN_ERR(  \
                  "[TRN] NULL Pointer detected. Referrence Impossible\n"); \
        return TRN_NULL_PARAMETER_NOT_ALLOWED; \
    }
#else
#define TRN_NULL_CHECK(x)
#endif /* TRN_NO_NULL_PARAM_CHECK */

#ifndef MS_TRN_NO_RANGE_CHECK
/** Range Check for Transport API Parameters */
#define TRN_RANGE_CHECK_START(param, start) \
    if ( ! ((param) >= (start)) ) \
    { \
        TRN_ERR( \
                 "[TRN] TRN Range Check FAILED\n"); \
        return TRN_PARAMETER_OUTSIDE_RANGE; \
    }

#define TRN_RANGE_CHECK_END(param, end) \
    if ( ! ((param) <= (end)) ) \
    { \
        TRN_ERR( \
                 "[TRN] TRN Range Check FAILED\n"); \
        return TRN_PARAMETER_OUTSIDE_RANGE; \
    }

#define TRN_RANGE_CHECK(param, start, end) \
    if ( ! ( ((param) >= (start)) && ((param) <= (end)) ) ) \
    { \
        TRN_ERR( \
                 "[TRN] TRN Range Check FAILED\n"); \
        return TRN_PARAMETER_OUTSIDE_RANGE; \
    }

#else
#define TRN_RANGE_CHECK_START(param, start)
#define TRN_RANGE_CHECK_END(param, end)
#define TRN_RANGE_CHECK(param, start, end)
#endif /* TRN_NO_RANGE_CHECK */

/** Transport Control Field - Maximum and Minimum number of octets */
#define TCF_HDR_LEN_MAX    4
#define TCF_HDR_LEN_MIN    1

/** Maximum Transport Packet Size */
/* Maximum size of Upper Transport Control PDU is 256 bytes */
#define TRN_MAX_CTL_PKT_SIZE      256

/* Maximum size of Upper Transport Access PDU is 384 bytes */
#define TRN_MAX_ACCESS_PKT_SIZE   384

#define TRN_MAX_PKT_SIZE          TRN_MAX_ACCESS_PKT_SIZE

/**
    Frame TCF Header for Unsegmented Message.

    7        5        4         2        0
    +--------+--------+---------+--------+
    |   AS   |  AKF   |   AID   |   RFU  |
    |  (00)  |        |         |        |
    +--------+--------+---------+--------+
*/
#define TRN_FRAME_UNSEG_MSG_HDR(tcf_hdr, unseg_hdr) \
    (tcf_hdr) = ((MS_TCF_UNSEGMENTED_MSG << 6) | \
                 ((unseg_hdr)->akf << 5) | \
                 ((unseg_hdr)->aid << 2))

#define LPN_ELEMENT_INVALID             0x00
#define LPN_ELEMENT_VALID_TEMP          0xFF
#define LPN_ELEMENT_VALID_PERMANENT     0x01

/**
    In Heartbeat Publication Features
    - only bits 0-3 are valid
    - bits 4-15 are Reserved for Future Use
*/
#define TRN_HBP_FEATURE_MASK            0x000F

/* --------------------------------------------- Data Types/ Structures */

/* Individual Queue Elements */
typedef struct _TRN_FRN_Q_ELEMENT
{
    /* Associated Network Header */
    MS_NET_HEADER net_hdr;

    /* Packet Length */
    UINT8         ltrn_pkt_length;

    /* Is Ack? */
    UINT8         is_ack;

    /* Lower Transport Packet */
    UINT8         pdu[16 /* NET_MAX_PAYLOAD_SIZE */];

} TRN_FRN_Q_ELEMENT;

/**
      Friend Queue

      0-th Index: Contains completely received segmented or unsegmented
                  Transport Messages
      1-st Index: Contains segmented Transport Messages currently being
                  received. After all the segments are received, segments
                  from [#1] will be copied to [#0]
*/
typedef struct _TRN_FRIEND_QUEUE
{
    MS_DEFINE_GLOBAL_ARRAY(TRN_FRN_Q_ELEMENT, queue, MS_CONFIG_LIMITS(MS_FRIEND_MESSAGEQUEUE_SIZE));
    UINT16 queue_start;
    UINT16 queue_size;
} TRN_FRIEND_QUEUE;


/** LowPower Node Element Structure */
typedef struct _TRN_LPN_ELEMENT
{
    MS_TRN_FRNDSHIP_INFO node;

    /* Friend Clear Context */
    struct _clrctx
    {
        /* Previous friend address */
        MS_NET_ADDR frnd_addr;

        /* Clear Retry Timeout */
        UINT32 retry_tmo;

        /* Timer handle */
        EM_timer_handle thandle;
    } clrctx;

    /**
        Friend Queue

        0-th Index: Contains completely received segmented or unsegmented
                   Transport Messages
        1-st Index: Contains segmented Transport Messages currently being
                   received. After all the segments are received, segments
                   from [#1] will be copied to [#0]
    */
    TRN_FRIEND_QUEUE friend_queue[2];

    /* Subscription List */
    MS_DEFINE_GLOBAL_ARRAY(MS_NET_ADDR, subscription_list, MS_CONFIG_LIMITS(MS_FRIEND_SUBSCRIPTION_LIST_SIZE));

    /* LPN Poll Timeout (in ms) */
    UINT32 poll_timeout;

    /*
        Validity index. Supports following Values:
        - 0x00: Entity Invalid
        - 0x01: Entity Valid Permanent
        - 0xFF: Entity Valid Temporary
    */
    UCHAR valid;

    /* Message Queue Start Index */
    UINT16 mqstart;

    /* Message Queue End Index */
    UINT16 mqend;

    /* LPN Receive Delay (in Ms) */
    UCHAR rx_delay;

    /* Number of elements in the LPN */
    UCHAR num_elements;

    /* RSSI of the friend request packet */
    UCHAR rssi;

    /* Friend Sequence Number in FriendPoll */
    UCHAR fsn;

    /* Timer for the element - TODO: Single global timer? */
    EM_timer_handle thandle;

} TRN_LPN_ELEMENT;


/** Friend Element Structure */ /* TODO: See if the structures can be clubbed */
typedef struct _TRN_FRND_ELEMENT
{
    /* Friendship information */
    MS_TRN_FRNDSHIP_INFO node;

    /* Adjusted Poll timeout */
    UINT32 poll_to;

    /* The main subnet handle */
    MS_SUBNET_HANDLE subnet_handle;

    /* Friend Address */
    MS_NET_ADDR addr;

    /* Friend Receive Windows */
    UCHAR rx_window;

    /* Transaction Number for Management PDUs */
    UCHAR txn_no;

    /* Current Friend sequence number */
    UCHAR fsn;

    /* Friend sequence number last sent */
    UCHAR sfsn;

    /* Timer for the element */
    EM_timer_handle thandle;

    /* Poll retry tracker */
    UCHAR poll_retry_count;

} TRN_FRND_ELEMENT;

/** Hearbeat Publication state */
typedef struct _TRN_HEARTBEAT_PUBLICATION_STATE
{
    /**
        Destination address for Heartbeat messages
    */
    MS_NET_ADDR daddr;

    /**
        Count to control the number of periodic heartbeat
        transport messages to be sent
    */
    UINT8 count_log;

    /**
        Period to control the cadence of periodic heartbeat
        transport messages
    */
    UINT8 period_log;

    /**
        TTL value to be used when sending Heartbeat messages
    */
    UINT8 ttl;

    /**
        Features that trigger sending Heartbeat messages when changed
    */
    UINT16 features;

    /**
        Global NetKey index of the NetKey to be used to send Heartbeat messges
    */
    UINT16 netkey_index;

    /** Associated Subnet Handle */
    MS_SUBNET_HANDLE subnet_handle;

    /* Period Timer */
    EM_timer_handle timer_handle;

    /* Tx Count */
    UINT32           tx_count;

} TRN_HEARTBEAT_PUBLICATION_STATE;

/** Hearbeat Subscription state */
typedef struct _TRN_HEARTBEAT_SUBSCRIPTION_STATE
{
    /**
        Source address for Heartbeat messages that a node shall process
    */
    MS_NET_ADDR saddr;

    /**
        Destination address for Heartbeat messages
    */
    MS_NET_ADDR daddr;

    /**
        Counter that tracks the number of periodic heartbeat transport message
        received since receiving the most recent Config Heartbeat Subscription
        Set message
    */
    UINT16 count;

    /**
        Logarithmic value of the above count
    */
    UINT8 count_log;

    /**
        Period that controls the period for processing periodical Heartbeat
        transport control messages
    */
    UINT8 period_log;

    /**
        Minimum hops value registered when receiving heartbeat messages since
        receiving the most recent Config Heartbeat Subscription Set message
    */
    UINT16 min_hops;

    /**
        Maximum hops value registered when receiving heartbeat messages since
        receiving the most recent Config Heartbeat Subscription Set message
    */
    UINT16 max_hops;

    /* Period Timer */
    EM_timer_handle timer_handle;

} TRN_HEARTBEAT_SUBSCRIPTION_STATE;

/* --------------------------------------------- Functions */
/**
    \par Description
    This function handles the incoming data received over lower transport layer.

    \param [in] net_hdr
           Received Network Packet Header
    \param [in] subnet_handle
           Handle identifying associated subnet on which the packet is received
    \param [in] szmic
           Field representing of Size of TransMIC.
    \param [in] pdata
           The incoming Data Packet
    \param [in] pdatalen
           Size of the incoming Data Packet

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT trn_pkt_in
(
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UCHAR               szmic,
    /* IN */ UCHAR*              pdata,
    /* IN */ UINT16              pdatalen
);

void trn_handle_frnd_req
(
    /* IN */ MS_NET_HEADER*          net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ MS_TRN_FRND_REQ_PARAM* param
);

void trn_handle_frnd_offer
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ MS_TRN_FRND_OFFER_PARAM* param
);

void trn_handle_frnd_clear
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ MS_TRN_FRND_CLEAR_PARAM* param
);

/* Handle Friend Clear Confirmation */
void trn_handle_frnd_clear_cnf
(
    /* IN */ MS_NET_HEADER*                net_hdr,
    /* IN */ MS_SUBNET_HANDLE              subnet_handle,
    /* IN */ MS_TRN_FRND_CLEAR_CNF_PARAM* param
);

/* Handle Friend Subscription List Add/Remove */
void trn_handle_frnd_subscription_list_add_remove
(
    /* IN */ MS_NET_HEADER*                net_hdr,
    /* IN */ MS_SUBNET_HANDLE              subnet_handle,
    /* IN */ MS_TRN_FRND_MANAGE_PARAM*     param
);

/* Handle Friend Subscription List Confirmation */
void trn_handle_frnd_subscription_list_cnf
(
    /* IN */ MS_NET_HEADER*                          net_hdr,
    /* IN */ MS_SUBNET_HANDLE                        subnet_handle,
    /* IN */ MS_TRN_FRND_SUBSCRN_LIST_CNF_PARAM*     param
);

/* Handle Heartbeat */
void trn_handle_heartbeat
(
    /* IN */ MS_NET_HEADER*              net_hdr,
    /* IN */ MS_SUBNET_HANDLE            subnet_handle,
    /* IN */ MS_TRN_HEARTBEAT_PARAM*     param
);

void trn_handle_frnd_poll
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ MS_TRN_FRND_POLL_PARAM*   param
);

void trn_handle_frnd_update
(
    /* IN */ MS_NET_HEADER*              net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ MS_TRN_FRND_UPDATE_PARAM*   param
);

API_RESULT  trn_frame_secure_pdu
(
    /* IN */  MS_NET_ADDR   saddr,
    /* IN */  MS_NET_ADDR   daddr,
    /* IN */  UINT8*        label,
    /* IN */  MS_BUFFER*    pdu,
    /* IN */  UINT32        seq_num,
    /* IN */  UCHAR*        key,
    /* IN */  UCHAR         key_type,
    /* IN */  UCHAR         mic_len,
    /* OUT */ UCHAR*        sec_pdu
);

API_RESULT  trn_frame_decrypt_pdu
(
    /* IN */  MS_NET_ADDR          saddr,
    /* IN */  MS_NET_ADDR          daddr,
    /* IN */  UINT32               seq_num,
    /* IN */  UINT8                ivi,
    /* IN */  MS_BUFFER*           sec_pdu,
    /* IN */  UCHAR                aid,
    /* IN */  UCHAR                key_type,
    /* IN */  UCHAR                mic_len,
    /* OUT */ UCHAR*               pdu,
    /* OUT */ MS_APPKEY_HANDLE*    appkey_handle
);

API_RESULT  frnd_verify_request_criteria(UCHAR criteria, UCHAR* rssi, UCHAR* rcvwin);

UINT16 frnd_alloc_lpn_element(MS_NET_ADDR addr);
UINT16 frnd_search_lpn_element(MS_NET_ADDR addr);
void frnd_clear_lpn_element(UINT16 lpn_index);

void trn_frndpoll(void);
void trn_frndpoll_send_handler(void* args, UINT16 size);
void trn_frnd_handle_segment_ack(MS_SUBNET_HANDLE subnet_handle);
void trn_frndpoll_rsp_handler(void* args, UINT16 size);
void trn_frndreq_rsp_handler(void* args, UINT16 size);

void trn_frndpoll_timeout_handler(void* args, UINT16 size);
void trn_frndclear_retry_timeout_handler(void* args, UINT16 size);
void trn_frndreq_retry_timeout_handler(void* args, UINT16 size);

API_RESULT trn_frnd_send(MS_NET_ADDR addr, MS_SUBNET_HANDLE subnet_handle, UINT8 opcode, void* pstruct, UINT8 ttl);
void trn_frndclear(UINT16 lpn_index, UINT32 timeout);

/**
    \brief Add message to friend queue.

    \par Description Add message to friend queue.

    \param [in] lpn_index    Associated LPN Handle/Index
    \param [in] pkt_type     Type of packet to be enqueued
                             - 0x00: Unsemented/Only One Segment (final packet)
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
);

API_RESULT trn_get_from_queue
(
    /* IN */ LPN_HANDLE       lpn_index,
    /* IN */ MS_NET_HEADER*   net_hdr,
    /* IN */ UINT8*           pdu,
    /* IN */ UINT8*           pdu_len
);

#if 0
API_RESULT trn_flush_reassembly_queue
(
    /* IN */ LPN_HANDLE       lpn_index
);
#endif /* 0 */

TRN_FRN_Q_ELEMENT*   trn_dequeue_queue_head
(
    /* IN */ TRN_FRIEND_QUEUE*   queue
);

API_RESULT trn_add_to_queuelet
(
    /* IN */ TRN_FRIEND_QUEUE*   queue,
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ UINT8*              pdu,
    /* IN */ UINT8               pdu_len
);

/* Search for Ack match in the queue and update */
API_RESULT trn_update_lpn_ack
(
    /* IN */ TRN_FRIEND_QUEUE*   queue,
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ UINT8*              pdu,
    /* IN */ UINT8               pdu_len
);

#endif /* _H_TRN_INTERNAL_ */

