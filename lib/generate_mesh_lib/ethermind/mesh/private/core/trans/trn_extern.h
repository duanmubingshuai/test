
/**
    \file trn_extern.h

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_TRN_EXTERN_
#define _H_TRN_EXTERN_

/* --------------------------------------------- External Global Definitions */
/* Module Mutex */
MS_DEFINE_MUTEX_TYPE(extern, trn_mutex)

/* Module callbacks */
extern TRN_NTF_CB trn_ctrl_callback;
extern TRN_NTF_CB trn_access_callback;
extern TRN_HEARTBEAT_RCV_CB    trn_heartbeat_rcv_callback;
extern TRN_HEARTBEAT_RCV_TIMEOUT_CB    trn_heartbeat_rcv_timeout_callback;



/** Friend Role */
extern UCHAR trn_frnd_role;

/** LowPower Node Element Database */
MS_DECLARE_GLOBAL_ARRAY(TRN_LPN_ELEMENT, lpn_element, MS_CONFIG_LIMITS(MS_MAX_LPNS));

/** Friend Element Database */
extern TRN_FRND_ELEMENT frnd_element;

/** Global Friend counter */
extern UINT16 trn_frnd_counter;

/* Global LPN counter */
extern UINT16 trn_lpn_counter;

/* Friendship Setup timeout */
extern UINT32 frnd_setup_timeout;

/* Friendship setup time lapsed count */
extern UINT32 frnd_setup_time_lapsed;

/* Heartbeat Publication Information */
extern TRN_HEARTBEAT_PUBLICATION_STATE heartbeat_pub;

/* Heartbeat Subscription Information */
extern TRN_HEARTBEAT_SUBSCRIPTION_STATE heartbeat_sub;

/* Friendship callback */
extern TRN_FRND_CB frnd_cb;

#endif /* _H_TRN_EXTERN_ */

