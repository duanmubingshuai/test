
/**
    \file prov_internal.h


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

#ifndef _H_PROV_INTERNAL_
#define _H_PROV_INTERNAL_

/* --------------------------------------------- Header File Inclusion */
#include "MS_prov_api.h"
#include "MS_net_api.h"
#include "prov_pl.h"
#include "cry.h"

/* --------------------------------------------- Global Definitions */
#ifdef PROV_NO_DEBUG
    #define PROV_ERR          EM_debug_null
#else /* PROV_NO_DEBUG */
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define PROV_ERR
    #else
        #define PROV_ERR(...)     EM_debug_error(MS_MODULE_ID_PROV, __VA_ARGS__)
    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#endif /* PROV_NO_DEBUG */

#ifdef PROV_DEBUG
    #ifdef VAR_ARG_IN_MACRO_NOT_SUPPORTED
        #define PROV_TRC
        #define PROV_INF

        #define PROV_debug_dump_bytes(data, datalen)

    #else
        #define PROV_TRC(...)     EM_debug_trace(MS_MODULE_ID_PROV,__VA_ARGS__)
        #define PROV_INF(...)     EM_debug_info(MS_MODULE_ID_PROV,__VA_ARGS__)

        #define PROV_debug_dump_bytes(data, datalen) EM_debug_dump_bytes(MS_MODULE_ID_PROV, (data), (datalen))

    #endif /* VAR_ARG_IN_MACRO_NOT_SUPPORTED */
#else /* PROV_DEBUG */
    #define PROV_TRC          EM_debug_null
    #define PROV_INF          EM_debug_null

    #define PROV_debug_dump_bytes(data, datalen)

#endif /* PROV_DEBUG */

/**
    Locks the PROV Mutex which prevents any global variable being
    overwritten by any function. It returns an error if mutex lock fails.
*/
#define PROV_LOCK() \
    MS_MUTEX_LOCK(prov_mutex, PROV)

/**
    Locks the PROV_mutex which prevents any global variable being
    overwritten by any function. To be used in void function as it
    returns no error.
*/
#define PROV_LOCK_VOID() \
    MS_MUTEX_LOCK_VOID(prov_mutex, PROV)

/**
    Unlocks the PROV_mutex which realeses the global variables
    to be written into. It returns an error if mutex unlock fails.
*/
#define PROV_UNLOCK() \
    MS_MUTEX_UNLOCK(prov_mutex, PROV)

/**
    Unlocks the PROV_mutex which realeses the global variables
    to be written into. To be used in void functions as it returns
    no error.
*/
#define PROV_UNLOCK_VOID() \
    MS_MUTEX_UNLOCK_VOID(prov_mutex, PROV)

/* Provisioning CRC table size */
#define PROV_CRC_TABLE_SIZE                     256

/* Initilizer for Provisioning FCS Calculation */
#define PROV_FCS_INIT_VALUE                     0xFF

/* FCS value to check received FCS - 11110011 (received order) */
#define PROV_FCS_CHECK_VALUE                    0xCF

/** Length of each of the above PDU types */
#define PROV_PDU_TYPE_INVITE_LEN            2
#define PROV_PDU_TYPE_CAPAB_LEN             12
#define PROV_PDU_TYPE_START_LEN             6
#define PROV_PDU_TYPE_PUBKEY_LEN            65
#define PROV_PDU_TYPE_INPUT_CMPLT_LEN       1
#define PROV_PDU_TYPE_CNF_LEN               17
#define PROV_PDU_TYPE_RAND_LEN              17
#define PROV_PDU_TYPE_DATA_LEN              26
#define PROV_PDU_TYPE_COMPLETE_LEN          1
#define PROV_PDU_TYPE_FAILED_LEN            2

/** Provisioning Opcode Value lengths */
#define PROV_PDU_INVITEVAL_LEN              (PROV_PDU_TYPE_INVITE_LEN - 1)
#define PROV_PDU_CAPABVAL_LEN               (PROV_PDU_TYPE_CAPAB_LEN - 1)
#define PROV_PDU_STARTVAL_LEN               (PROV_PDU_TYPE_START_LEN - 1)
#define PROV_PDU_PUBKEY_LEN                 (PROV_PDU_TYPE_PUBKEY_LEN - 1)

/** Key sizes used internally during provisioning procedure */
#define PROV_CONFVAL_SIZE                   16
#define PROV_RANDVAL_SIZE                   16
#define PROV_K1_KEY_SIZE                    16
#define PROV_S1_KEY_SIZE                    16
#define PROV_RAND_SIZE                      16
#define PROV_AUTHVAL_SIZE                   16
#define PROV_NONCE_SIZE                     13
#define PROV_PUBKEY_SIZE                    64
#define PROV_PVTKEY_SIZE                    32
#define PROV_SECRET_SIZE                    32
#define PROV_DATA_MIC_SIZE                  8

/** PDU and Payload header sizes */
#define PROV_TX_PDU_SIZE                    75
#define PROV_MAX_PB_GATT_MTU                PROV_TX_PDU_SIZE
#define PROV_MAX_PB_ADV_HDR_SIZE            5
#define PROV_MAX_PB_ADV_PDU_SIZE            29
#define PROV_PAYLOAD_SIZE_FRAG_1            20
#define PROV_PAYLOAD_SIZE_FRAG_N            (PROV_PAYLOAD_SIZE_FRAG_1 + 3)

/** Internal link states for provisional context */
#define PROV_STATE_UNINITIALIZED            0xFFFFFFFF
#define PROV_STATE_INITIALIZED              0x000000B0
#define PROV_STATE_BEACONING                0x000000B1
#define PROV_STATE_SCANNING                 0x000000B2
#define PROV_STATE_LINKOPEN                 0x000000B3

/** TODO: Move to configuration file */
#define PROV_PDU_RTX_COUNT                  30
#define PROV_PDU_RTX_TIMEOUT_SEC            100
#define PROV_COMPLETE_TIMEOUT_SEC           500
#define PROV_PROC_TIMEOUT_SEC               20

#define PROV_PDU_RX_DELAY_MSEC              20
#define PROV_PDU_TX_DELAY_MSEC              20
#define PROV_PDU_TX_DELAY_TIMEOUT_MSEC      (EM_TIMEOUT_MILLISEC | PROV_PDU_TX_DELAY_MSEC)
#define PROV_PDU_RX_DELAY_TIMEOUT_MSEC      (EM_TIMEOUT_MILLISEC | PROV_PDU_RX_DELAY_MSEC)

/* --------------------------------------------- Structures/Data Types */

/** Provisioning Bearer specific information */
typedef struct _PROV_BRR_INFO
{
    /* Bearer Handle */
    BRR_HANDLE handle;

} PROV_BRR_INFO;

//typedef struct _PROV_BRR_ADV_COUNT
//{
//    /* adv count */
//    UCHAR count;

//} PROV_BRR_ADV_COUNT;


/** Provisioning Context */
typedef struct _PROV_CONTEXT
{
    /* Remote Public Key */
    UCHAR rpubkey[PROV_PUBKEY_SIZE];

    /* ECDH Key */
    UCHAR ecdh_key[PROV_SECRET_SIZE];

    /* Local Confirm Value */
    UCHAR lconfval[PROV_CONFVAL_SIZE];

    /* Remote Confirm Value */
    UCHAR rconfval[PROV_CONFVAL_SIZE];

    /* Local Random Value */
    UCHAR lrandval[PROV_RANDVAL_SIZE];

    /* Remote Random Value */
    UCHAR rrandval[PROV_RANDVAL_SIZE];

    /* Authentication Value */
    UCHAR authval[PROV_AUTHVAL_SIZE];

    /* Device UUID */
    UCHAR uuid[MS_DEVICE_UUID_SIZE];

    /* Keys during provisioning */
    UCHAR session_key[PROV_K1_KEY_SIZE];
    UCHAR nonce[PROV_NONCE_SIZE];

    /* Key for application layer payload authentication & encryption */
    UCHAR dev_key[PROV_K1_KEY_SIZE];

    /* Confirmation Salt created suring provisioning */
    UCHAR conf_salt[PROV_S1_KEY_SIZE];

    /* Capability and Start packets exchanged during Provisioning */
    UCHAR inviteval[PROV_PDU_INVITEVAL_LEN];
    UCHAR capval[PROV_PDU_CAPABVAL_LEN];
    UCHAR startval[PROV_PDU_STARTVAL_LEN];

    /* Transmit PDU Buffer for retransmission */
    UCHAR tx_pdu[PROV_TX_PDU_SIZE];

    /* Transmit PDU length */
    UINT16 tx_pdu_len;

    /* Receive PDU Buffer */
    UCHAR* rx_pdu;

    /* Receive Total PDU Length */
    UINT16 rx_pdu_len;

    /* Receive PDU Length - till now */
    UINT16 rx_partial_pdu_len;

    /* Receive Fragment Count */
    UCHAR rx_frag_index;

    /* Receive Fragment Count */
    UCHAR prev_frag_len;

    /* Receive Transaction Id */
    UCHAR rx_txn_id;

    /* Transmit Transaction Id */
    UCHAR tx_txn_id;

    /* Transmit Transaction Id Limit */
    UCHAR txn_id_max;

    /* FCS of the received packet */
    UINT16 fcs;

    /* State */
    UINT32 state;

    /* Role */
    UCHAR role;

    /* Bearer Type */
    UCHAR bearer;

    /* Link Open status */
    UCHAR link;

    /* Link ID */
    UINT32 link_id;

    /* Attention Timeout */
    UCHAR attention;

    /* ACK Retransmission timer handle */
    EM_timer_handle rtx_timer_handle;

    /* Provisionoing Complete timer handle */
    EM_timer_handle proc_timer_handle;

    /* Provisionoing Procedure timer handle */
    EM_timer_handle prov_timer_handle;

    /* Retransmission count */
    UCHAR rtx_count;

    /* Ack chained event identifier */
    UCHAR ackevent;

    /* Ack chained event direction */
    UCHAR ackeventdir;

    /* State to be set upon acknowledgement */
    UINT32 ackstate;

    /* Handle to the context */
    PROV_HANDLE handle;

    /* Handle to the provisioning bearer information */
    PROV_BRR_INFO* brr_info;

    /* Unicast Address of the first element */
    MS_NET_ADDR    r_uaddr;

    /* Number of Elements */
    UINT8          r_num_elements;

} PROV_CONTEXT;

extern EM_timer_handle link_open_ack_timer_handle;




/* --------------------------------------------- Macros */
#define PROV_IS_ROLE_DEVICE(ctx) \
    ((PROV_ROLE_DEVICE == (ctx)->role)? MS_TRUE: MS_FALSE)

#define PROV_IS_ROLE_PROVISIONER(ctx) \
    ((PROV_ROLE_PROVISIONER == (ctx)->role)? MS_TRUE: MS_FALSE)

#define PROV_ASSERT_CONTEXT_ROLE(ctx, r)

#define PROV_SET_STATE(s) \
    prov_state = (s)

#define PROV_GET_STATE() \
    prov_state

#define PROV_CONTEXT_SET_STATE(ctx, s) \
    (ctx)->state = (s)

#define PROV_CONTEXT_GET_STATE(ctx) \
    (ctx)->state

#define PROV_SET_LINK_OPEN(ctx) \
    (ctx)->link = MS_TRUE

#define PROV_RESET_LINK_OPEN(ctx) \
    (ctx)->link = MS_FALSE

#define PROV_IS_LINK_OPEN(ctx) \
    ((ctx)->link)

#define PROV_IS_OOB_PUBKEY(ctx) \
    ((PROV_PUBKEY_OOB == (ctx)->startval[1])? MS_TRUE: MS_FALSE)

#define PROV_GET_AUTH_TYPE(ctx) \
    (ctx)->startval[2]

#define PROV_GET_AUTH_ACTION(ctx) \
    (ctx)->startval[3]

#define PROV_GET_AUTH_SIZE(ctx) \
    (ctx)->startval[4]

#define PROV_SET_ACK_EVENT(ctx, etype, edir) \
    (ctx)->ackevent = (etype); \
    (ctx)->ackeventdir = (edir)

#define PROV_SET_ACK_STATE(ctx, s) \
    (ctx)->ackstate = (s)

#define PROV_INCR_TXN_ID(ctx) \
    (ctx)->tx_txn_id ++; \
    if ((ctx)->tx_txn_id == (ctx)->txn_id_max) \
    { \
        (ctx)->tx_txn_id = 0; \
    }

/* --------------------------------------------- Internal Functions */
API_RESULT prov_adv_recv_cb(BRR_HANDLE* handle, UCHAR pevent, UCHAR* pdata, UINT16 pdatalen);
API_RESULT prov_gatt_recv_cb(BRR_HANDLE* handle, UCHAR pevent, UCHAR* pdata, UINT16 pdatalen);
void prov_handle_unprovisioned_beacon(UCHAR* pdata, UINT16 pdatalen);

PROV_CONTEXT* prov_alloc_context(PROV_HANDLE* phandle);
void prov_free_context(PROV_CONTEXT* ctx);
void prov_handle_ack (PROV_CONTEXT* ctx);
PROV_CONTEXT* prov_get_context_by_state(UINT32 state);
PROV_CONTEXT* prov_findcontext_by_handle(PROV_HANDLE* phandle);
PROV_CONTEXT* prov_findcontext_by_state(UINT32 state);
PROV_CONTEXT* prov_findcontext_by_linkid(UINT32 link_id);
PROV_CONTEXT* prov_findcontext_by_uuid(UCHAR* uuid);
PROV_CONTEXT* prov_findcontext_by_brr(PROV_BRR_INFO* brr);

void prov_generate_authkeys(PROV_CONTEXT* ctx);
API_RESULT prov_calc_confirm(PROV_CONTEXT* ctx, UCHAR confirm);

void prov_framensend_ack(PROV_CONTEXT* ctx);
API_RESULT prov_framensend_pdu(PROV_CONTEXT* ctx, UCHAR* packet, UINT16 length, UINT32 nstate);
API_RESULT prov_framensend_pb_gatt_pdu(PROV_CONTEXT* ctx, UCHAR* packet, UINT16 length);
API_RESULT prov_framensend_pb_adv_pdu(PROV_CONTEXT* ctx, UCHAR* packet, UINT16 length);
void prov_link_ack_timeout_handler(void* args, UINT16 size);
void prov_pdu_ack_timeout_handler(void* args, UINT16 size);
void prov_complete_timeout_handler(void* args, UINT16 size);
void prov_rx_delay(PROV_CONTEXT* ctx);
void prov_tx_delay(PROV_CONTEXT* ctx);

API_RESULT prov_link_open(PROV_CONTEXT* ctx);
API_RESULT prov_link_accept(PROV_CONTEXT* ctx);
API_RESULT prov_link_close(PROV_CONTEXT* ctx, UCHAR reason);

void prov_handle_pdu(PROV_CONTEXT* ctx);
void prov_handle_adv_message(UCHAR* pdata, UINT16 pdatalen);
void prov_handle_gatt_message(PROV_BRR_INFO* pbrr, UCHAR* pdata, UINT16 pdatalen);
void prov_handle_bcon_message(UCHAR* pdata, UINT16 pdatalen);

void prov_restart_proc_timer(PROV_CONTEXT* ctx);
void prov_proc_timeout_handler(void* args, UINT16 size);

API_RESULT prov_process_event(PROV_CONTEXT* ctx, UCHAR evt, UCHAR src);
void prov_procedure_complete(PROV_CONTEXT* ctx, UINT16 status);
void prov_send_failure(PROV_CONTEXT* ctx, UCHAR reason);
void prov_notify
(
    PROV_CONTEXT* ctx,
    UCHAR          event_type,
    UINT16         event_result,
    void*          event_data,
    UINT16         event_datalen
);

void prov_generate_fcs
(
    /* IN */  UCHAR*     data,
    /* IN */  UINT16     datalen,
    /* OUT */ UCHAR*     fcs
);
UCHAR prov_verify_fcs
(
    /* IN */  UCHAR*     data,
    /* IN */  UCHAR      datalen,
    /* IN */  UCHAR      fcs_recvd
);

#ifdef CRY_ECDH_TIMESLICE
    void prov_ecdh_complete_cb (UCHAR* secret);
#endif /* CRY_ECDH_TIMESLICE */

/* --------------------------------------------- API Declarations */

#endif /* _H_PROV_INTERNAL_ */

