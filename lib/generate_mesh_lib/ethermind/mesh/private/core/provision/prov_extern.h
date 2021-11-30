
/**
    \file prov_extern.h


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */

/* --------------------------------------------- Global Definitions */

/* --------------------------------------------- Structures/Data Types */
/** Provisinoing module Mutex */
MS_DEFINE_MUTEX_TYPE(extern, prov_mutex)

/** Provisioning capabilities of local device */
extern PROV_CAPABILITIES_S* prov_cap;

/** Provisioning application callback */
extern PROV_UI_NOTIFY_CB prov_cb;

/** Provisioning Context */
extern PROV_CONTEXT prov_context;

/** Global Provisioning module state */
extern UINT32 prov_state;

/** Global Provisioning Role */
extern UCHAR prov_role;

/** Local Public key for the provisioning module */
extern UCHAR prov_pubkey[PROV_PUBKEY_SIZE];

/** Active Provisioning UUID */
extern UCHAR prov_uuid[MS_DEVICE_UUID_SIZE];

extern void blebrr_scan_pl (UCHAR enable);


/** Provisioning Bearer Handle */
MS_DECLARE_GLOBAL_ARRAY(PROV_BRR_INFO, prov_brr, MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES));


/* --------------------------------------------- Macros */

/* --------------------------------------------- Internal Functions */


