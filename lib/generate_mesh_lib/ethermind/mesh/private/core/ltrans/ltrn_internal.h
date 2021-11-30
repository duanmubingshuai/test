
/**
    \file ltrn_internal.h

    Module Internal Header File contains structure definitions including tables
    maintained by the module
*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_LTRN_INTERNAL_
#define _H_LTRN_INTERNAL_

/* --------------------------------------------- Header File Inclusion */
#include "MS_common.h"
#include "MS_ltrn_api.h"
#include "MS_net_api.h"
#include "MS_trn_api.h"
#include "MS_access_api.h"
#include "EXT_cbtimer.h"
#include "cbtimer.h"


/* --------------------------------------------- Global Definitions */

#ifdef LTRN_NO_DEBUG
    #define LTRN_ERR          EM_debug_null
#else /* LTRN_NO_DEBUG */
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define LTRN_ERR
    #else
        #define LTRN_ERR(...)     EM_debug_error(MS_MODULE_ID_LTRN, __VA_ARGS__)
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* LTRN_NO_DEBUG */

#ifdef LTRN_DEBUG
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define LTRN_TRC
        #define LTRN_INF

        #define LTRN_debug_dump_bytes(data, datalen)

    #else
        #define LTRN_TRC(...)     EM_debug_trace(MS_MODULE_ID_LTRN,__VA_ARGS__)
        #define LTRN_INF(...)     EM_debug_info(MS_MODULE_ID_LTRN,__VA_ARGS__)

        #define LTRN_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_LTRN, (data), (datalen))

    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* LTRN_DEBUG */
    #define LTRN_TRC          EM_debug_null
    #define LTRN_INF          EM_debug_null

    #define LTRN_debug_dump_bytes(data, datalen)

#endif /* LTRN_DEBUG */

/**
    Locks the LTRN Mutex which prevents any global variable being
    overwritten by any function. It returns an error if mutex lock fails.
*/
#define LTRN_LOCK()\
    MS_MUTEX_LOCK(ltrn_mutex, TRN)

/**
    Locks the LTRN_mutex which prevents any global variable being
    overwritten by any function. To be used in void function as it
    returns no error.
*/
#define LTRN_LOCK_VOID()\
    MS_MUTEX_LOCK_VOID(ltrn_mutex, TRN)

/**
    Unlocks the LTRN_mutex which realeses the global variables
    to be written into. It returns an error if mutex unlock fails.
*/
#define LTRN_UNLOCK()\
    MS_MUTEX_UNLOCK(ltrn_mutex, TRN)

/**
    Unlocks the LTRN_mutex which realeses the global variables
    to be written into. To be used in void functions as it returns
    no error.
*/
#define LTRN_UNLOCK_VOID()\
    MS_MUTEX_UNLOCK_VOID(ltrn_mutex, TRN)

/** LTRN Dynamic Memory Allocation and Release Macros */
#define ltrn_alloc_mem(size) EM_alloc_mem(size)
#define ltrn_free_mem(ptr)   EM_free_mem(ptr)

#ifndef LTRN_NO_NULL_PARAM_CHECK
/** Null Check of Transport API Parameters */
#define LTRN_NULL_CHECK(x) \
    if (NULL == (x)) \
    { \
        LTRN_ERR(  \
                   "[LTRN] NULL Pointer detected. Referrence Impossible\n"); \
        return LTRN_NULL_PARAMETER_NOT_ALLOWED; \
    }
#else
#define LTRN_NULL_CHECK(x)
#endif /* LTRN_NO_NULL_PARAM_CHECK */

#ifndef MS_LTRN_NO_RANGE_CHECK
/** Range Check for Transport API Parameters */
#define LTRN_RANGE_CHECK_START(param, start) \
    if ( ! ((param) >= (start)) ) \
    { \
        LTRN_ERR( \
                  "[LTRN] LTRN Range Check FAILED\n"); \
        return LTRN_PARAMETER_OUTSIDE_RANGE; \
    }

#define LTRN_RANGE_CHECK_END(param, end) \
    if ( ! ((param) <= (end)) ) \
    { \
        LTRN_ERR( \
                  "[LTRN] LTRN Range Check FAILED\n"); \
        return LTRN_PARAMETER_OUTSIDE_RANGE; \
    }

#define LTRN_RANGE_CHECK(param, start, end) \
    if ( ! ( ((param) >= (start)) && ((param) <= (end)) ) ) \
    { \
        LTRN_ERR( \
                  "[LTRN] LTRN Range Check FAILED\n"); \
        return LTRN_PARAMETER_OUTSIDE_RANGE; \
    }

#else
#define LTRN_RANGE_CHECK_START(param, start)
#define LTRN_RANGE_CHECK_END(param, end)
#define LTRN_RANGE_CHECK(param, start, end)
#endif /* LTRN_NO_RANGE_CHECK */

/** Lower Transport Maximum Packet Length */
#define LTRN_MAX_PKT_SIZE    16

/** Segmented Message */

/** Frame Segmented Control Message Header */
#define LTRN_FRM_SEG_CTRL_MSG_HDR(opcode, rfu, seq_zero, seg_o, seg_n, hdr) \
    (hdr)[0] = (0x80 | (opcode)); \
    (hdr)[1] = (((rfu) << 7) | (((seq_zero) >> 6) & 0x7F)); \
    (hdr)[2] = ((((seq_zero) & 0x3F) << 2) | ((seg_o) >> 3)); \
    (hdr)[3] = ((((seg_o) & 0x07) << 5) | (seg_n))

/** Frame Segmented Access Message Header */
#define LTRN_FRM_SEG_ACCESS_MSG_HDR(afk, aid, szmic, seq_zero, seg_o, seg_n, hdr) \
    (hdr)[0] = (0x80 | ((afk) << 6) | ((aid) & 0x3F)); \
    (hdr)[1] = (((szmic) << 7) | (((seq_zero) >> 6) & 0x7F)); \
    (hdr)[2] = ((((seq_zero) & 0x3F) << 2) | ((seg_o) >> 3)); \
    (hdr)[3] = ((((seg_o) & 0x07) << 5) | (seg_n))

/** Frame Segment Ack Message */
#define LTRN_FRM_SEG_ACK_MSG(obo, seq_zero, rfu, block_ack, buf) \
    (buf)[0] = (((obo) << 7) | (((seq_zero) >> 6) & 0x7F)); \
    (buf)[1] = (((seq_zero) & 0x3F) << 2); \
    (buf)[2] = (UCHAR)((block_ack) >> 24); \
    (buf)[3] = (UCHAR)((block_ack) >> 16); \
    (buf)[4] = (UCHAR)((block_ack) >> 8); \
    (buf)[5] = (UCHAR)(block_ack)

/** Extract fields from segmented message header */
#define LTRN_EXTRACT_SEG_MSG_HDR(hdr, szmic, seq_zero, seg_o, seg_n) \
    (szmic) = ((hdr)[1] >> 7); \
    (seq_zero) = ((((hdr)[2] >> 2) | (((hdr)[1] & 0x03) << 6)) | ((((hdr)[1] >> 2) & 0x1F) << 8)); \
    (seg_o) = ((((hdr)[3] >> 5) | ((hdr)[2] << 3)) & 0x1F); \
    (seg_n) = ((hdr)[3] & 0x1F)


/** Lower Transport PDU Segmentation Types */
/* Unsegmented Access Message */
#define MS_LTRN_T_UNSEG_MSG               0x00

/* Segmented Access Message */
#define MS_LTRN_T_SEG_MSG                  0x01

/** Lower Transport PDU Format Type */
/* Unsegmented Access Message */
#define MS_LTRN_T_UNSEG_ACCESS_MSG          0x00

/* Segmented Access Message */
#define MS_LTRN_T_SEG_ACCESS_MSG            0x01

/* Unsegmented Control Message */
#define MS_LTRN_T_UNSEG_CTRL_MSG            0x02

/* Segmented Control Message */
#define MS_LTRN_T_SEG_CTRL_MSG              0x03

/* Segment Acknowledgment Opcode */
#define MS_LTRN_OPCODE_SEGMENT_ACK          0x00

/** Packet received for bitmasks */
#define LTRN_RX_FOR_LOCAL_ELEMENT           0x01
#define LTRN_RX_FOR_LPN_ELEMENT             0x02
#define LTRN_RX_FOR_LOCAL_SUBSCRIPTION      0x04
#define LTRN_RX_FOR_LPN_SUBSCRIPTION        0x08

/* SAR Context Types */
#define LTRN_SAR_CTX_INVALID                0x00
#define LTRN_SAR_CTX_RX                     0x01
#define LTRN_SAR_CTX_TX                     0x02


/* Initialize reassembly structure */
#define LTRN_REASM_STRUCT_INIT(rb) \
    (rb)->busy = 0; \
    (rb)->seg_n = 0; \
    (rb)->expected_block_ack = 0; \
    (rb)->rx_seg = 0; \
    (rb)->saddr = 0; \
    (rb)->daddr = 0; \
    (rb)->subnet_handle = MS_INVALID_SUBNET_HANDLE; \
    (rb)->buf_len = 0; \
    (rb)->ttl = 0xFF; \
    (rb)->ack_timer_handle = EM_TIMER_HANDLE_INIT_VAL; \
    (rb)->incomplete_to_counter = 0

/* --------------------------------------------- Data Types/ Structures */
/** Lower Transport Layer Rx Meta information */
typedef struct _LTRN_RX_META_INFO
{
    /**
        bitfield containining following information

        Bit 0 (lsb): Is for LPN element
        Bit 1: Is for local subscription
        Bit 2: Is for remote subscription

        Note:
        - Both bit 1 and 2 can be set together.
    */
    UINT8        is_for;

    /* Associated LPN handle. Meaningful if DST Addr is Unicast */
    LPN_HANDLE   lpn_handle;

} LTRN_RX_META_INFO;


/**
    Segmentation and reassembly context.
    This data structure is shared in both Rx (for reassembly)
    and Tx (for segmentation).
*/
typedef struct _LTRN_SAR_CTX
{
    /* 16 Bit Source Address */
    MS_NET_ADDR saddr;

    /* 16 Bit Destination Address */
    MS_NET_ADDR daddr;

    /* Subnet Handle */
    MS_SUBNET_HANDLE   subnet_handle;

    /* "Allocated" Data Pointer */
    UCHAR*   data;

    /* Data Length */
    UINT16   data_length;

    /* Contex Type: Invalid, Rx or Tx */
    UCHAR type;

    /* CTL - 1 bit */
    UINT8 ctl;

    /* IVI - 1 bit */
    UINT8 ivi;

    /* For access message */
    /* AKF */
    UINT8                     akf;

    /* AID */
    UINT8                     aid;

    /* SZMIC */
    UINT8                     szmic;

    /* For control message */
    /* Opcode */
    UINT8                     opcode;

    /* SeqZero */
    UINT16                    seq_zero;

    /* SeqAuth */
    UINT32                    seq_auth;

    /* Last Segment Count */
    UCHAR seg_n;

    /* TTL - 7 bits */
    UINT8 ttl;

    /* Bit field to hold which all segments/ack are received/pending/acked */
    UINT32 block_ack;

    /*
        Expected Block Ack - used by the transmitter to match the received
        Block Ack.
        In most of the cases, expected block ack can be calculated from
        the Last Segment Count, seg_n, using the following formula
        'Expected Block Ack = (UINT32)((1 << (seg_n + 1)) - 1)'
        But for seq_n value set to 0x1F (31), with 1 << (31 + 1), overflows
        and can have unpredictable outcome.
    */
    UINT32 expected_block_ack;

    /* Retransmission Count */
    UCHAR rtx_count;

    #ifdef EM_USE_EXT_TIMER
    /* Acknowledgment/Segment Transmission Timer (minimum of 150 + 50 * TTL ms) */
    EXT_cbtimer_handle ack_rtx_timer_handle;

    /* Incomplete Timer (minimum of 10 s) */
    EXT_cbtimer_handle incomplete_timer_handle;
    #else
    /* Acknowledgment/Segment Transmission Timer (minimum of 150 + 50 * TTL ms) */
    EM_timer_handle ack_rtx_timer_handle;

    /* Incomplete Timer (minimum of 10 s) */
    EM_timer_handle incomplete_timer_handle;
    #endif

    /* Rx Meta Info */
    LTRN_RX_META_INFO rx_meta_info;

} LTRN_SAR_CTX;


/* --------------------------------------------- Functions */
/**
    \par Description
    This function handles the incoming data received over network layer.

    \param [in] net_hdr
           Received Network Packet Header
    \param [in] subnet_handle
           Handle identifying associated subnet on which the packet is received
    \param [in] pdata
           The incoming Data Packet
    \param [in] pdatalen
           Size of the incoming Data Packet

    \retval
    - \ref NET_POST_PROCESS_RX_PKT: To inform Network Layer if the packet to be
           further processed, e.g. to be relayed or proxied etc.

    - Any Other Result/Error Code defined in MS_error.h: Ignored by Network Layer.
*/
API_RESULT ltrn_pkt_in
(
    /* IN */ MS_NET_HEADER*      net_hdr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UCHAR*              pdata,
    /* IN */ UINT16              pdatalen
);

API_RESULT ltrn_pkt_first_process
(
    /* IN */MS_NET_HEADER*     net_hdr,
    /* OUT */ UINT32*               repaly_flag
);


/**
    \brief Initialize Replay Protection List.

    \par Description
    This routine initializes the Replay Protection List.
*/
void ltrn_init_replay_cache (void);

/**
    \brief Allocation Replay Protection List (if to be created dynamically).

    \par Description
    This routine allocates the Replay Protection List, if to be created dynamically.
*/
void ltrn_alloc_replay_cache(void);

/**
    \brief Initialize SAR Contexts.

    \par Description
    This routine initializes the contexts used for Segmentation and Reassembly.
*/
void ltrn_init_sar_contexts (void);

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
);

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
);

void ltrn_set_is_replayed(/* IN */ UINT8 flag);

/**
    \brief Update Replay Protection List on IVI update.

    \par Description
    This routines updates Replay Protection List on IVI update.
*/
void ltrn_replay_cache_on_ivi_update(void);

/**
    \brief API to send segmented lower transport PDUs

    \par Description
    This routine sends segmented lower transport PDUs to peer device.

    \param [in] seq_zero
           Least significant bits of SeqAuth (13 bit value)

    \param [in] buffer
           Transport Packet

    \param [in] buffer_len
           Transport Packet Length

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT ltrn_send_seg_pdu
(
    /* IN */ MS_NET_HEADER*            net_hdr,
    /* IN */ MS_SUBNET_HANDLE          subnet_handle,
    /* IN */ UINT8                     akf,
    /* IN */ UINT8                     aid,
    /* IN */ UINT16                    seq_zero,
    /* IN */ UCHAR*                    buffer,
    /* IN */ UINT16                    buffer_len
);

API_RESULT ltrn_handle_segment_ack
(
    /* IN */ MS_NET_ADDR         saddr,
    /* IN */ MS_NET_ADDR         daddr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT8               obo,
    /* IN */ UINT16              seq_zero,
    /* IN */ UINT32              block_ack
);

API_RESULT ltrn_send_ack
(
    /* IN */ MS_NET_ADDR         saddr,
    /* IN */ MS_NET_ADDR         daddr,
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT32              seq_zero,
    /* IN */ UINT8               ttl_in_rx_frame,
    /* IN */ UINT8               obo,
    /* IN */ UINT32              block_ack
);

/* This function calculates SeqAuth, from SeqNum and SeqZero */
UINT32 ltran_calculate_seq_auth
(
    /* IN */ UINT32 seq_num,
    /* IN */ UINT32 seq_zero
);

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
);

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
);

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
);

UINT8 ltrn_get_sar_ctx_count
(
    /* IN */ UINT8          ctx_type
);


/* SAR Implementation */
/* Alloc Context */
LTRN_SAR_CTX* ltrn_sar_alloc_ctx
(
    /* IN */ UINT8              ctx_type,
    /* IN */ UINT16             length
);

/* Free Context */
void ltrn_sar_free_ctx
(
    /* IN */ LTRN_SAR_CTX*   ctx
);

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
);

/* Transmit segments which has not received Ack */
void ltrn_sar_transmit_segments
(
    /* IN */ LTRN_SAR_CTX* sar
);

/* Transmit specific segment */
void ltrn_sar_transmit_segment
(
    /* IN */ LTRN_SAR_CTX* sar,
    /* IN */ UINT8          seg_index
);

/* SAR Timeout Handler for Retry */
void ltrn_rtx_timeout_handler (void* args, UINT16 size);

/* SAR Timeout Handler for Ack */
void ltrn_ack_timeout_handler (void* args, UINT16 size);

/**
    \brief API to handle segmented lower transport PDUs

    \par Description
    This routine handles segmented lower transport PDUs received from peer device.

    \param [in] seq_zero
           Least significant bits of SeqAuth (13 bit value)

    \param [in] buffer
           Transport Packet

    \param [in] buffer_len
           Transport Packet Length

    \return API_SUCCESS or an error code indicating reason for failure
    TODO: Update header
*/
API_RESULT ltrn_handle_seg_pdu
(
    /* IN */ MS_NET_HEADER*       net_hdr,
    /* IN */ MS_SUBNET_HANDLE     subnet_handle,
    /* IN */ LTRN_RX_META_INFO*   rx_meta_info,
    /* IN */ UCHAR*               pdata,
    /* IN */ UINT16               pdatalen
);

API_RESULT ltrn_sar_start_ack_timer
(
    /* IN */ LTRN_SAR_CTX* sar
);

API_RESULT ltrn_sar_stop_ack_timer
(
    /* IN */ LTRN_SAR_CTX* sar
);

/* SAR Incomplete Timeout Handler for Rx */
void ltrn_incomplete_timeout_handler(void* args, UINT16 size);

API_RESULT ltrn_sar_restart_incomplete_timer
(
    /* IN */ LTRN_SAR_CTX* sar
);

#endif /* _H_LTRN_INTERNAL_ */

