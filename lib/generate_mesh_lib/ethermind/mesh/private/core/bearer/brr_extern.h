
/**
    \file brr_extern.h

*/

/*
    Copyright (C) 2016. Mindtree Ltd.
    All rights reserved.
*/

#ifndef _H_BRR_EXTERN_
#define _H_BRR_EXTERN_

/* --------------------------------------------- External Global Definitions */
/* Module Mutex */
MS_DEFINE_MUTEX_TYPE(extern, brr_mutex)

/** List of Mesh Bearer Information */
MS_DECLARE_GLOBAL_ARRAY(BRR_BEARER_DATA, brr_bearer, MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES));

#if 0
    /** Mesh Beacon Bearer Information */
    extern BRR_BEARER_INFO brr_bcon_bearer;
#endif /* 0 */

/** List of bearer specific callbacks */
extern BRR_NTF_CB brr_bearer_cb[BRR_COUNT];

/** List of beacon callbacks */
extern BRR_BCON_CB brr_beacon_cb[BRR_BCON_COUNT];

/** Bearer Handle for Beacon */
extern BRR_HANDLE bcon_brr_handle;

/** Received packet RSSI. Valid only for ADV bearer */
extern UCHAR brr_packet_rssi;

#endif /* _H_BRR_EXTERN_ */

