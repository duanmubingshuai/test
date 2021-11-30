
/**
    \file prov_init.c


*/

/*
    Copyright (C) 2013. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "prov_fsm_engine.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */
/** Provisioning module Mutex */
MS_DEFINE_MUTEX (prov_mutex);

/** Provisioning capabilities of local device */
PROV_CAPABILITIES_S* prov_cap;

/** Provisioning application callback */
PROV_UI_NOTIFY_CB prov_cb;

/** Provisioning Context */
PROV_CONTEXT prov_context;

/** Global Provisioning module state */
UINT32 prov_state;

/** Global Provisioning Role */
UCHAR prov_role;

/** Local Public key for the provisioning module */
UCHAR prov_pubkey[PROV_PUBKEY_SIZE];

/** Active Provisioning UUID */
UCHAR prov_uuid[MS_DEVICE_UUID_SIZE];

/** Provisioning Bearer Handle */
MS_DEFINE_GLOBAL_ARRAY(PROV_BRR_INFO, prov_brr, MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES));

/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- Functions */
void ms_prov_init (void)
{
    ms_internal_verificaiton_check();
    UCHAR index;
    PROV_TRC ("[PROV] Initializing Provisioning layer...\n");
    /* Module Mutex Initialization */
    MS_MUTEX_INIT_VOID (prov_mutex, PROV);
    /* Register with the ADV and GATT bearer layers */
    MS_brr_register (BRR_TYPE_PB_ADV, prov_adv_recv_cb);
    MS_brr_register (BRR_TYPE_PB_GATT, prov_gatt_recv_cb);
    /* Register for the Unprovisioned beacon with the bearer */
    MS_brr_register_beacon_handler
    (
        MS_BCON_TYPE_UNPRVSNG_DEV,
        prov_handle_unprovisioned_beacon
    );
    /* Register with the FSM */
    prov_fsm_register();
    /* Reset the provisioning context */
    prov_free_context(&prov_context);
    /* Initialize the Provisioning Bearer information */
    MS_INIT_GLOBAL_ARRAY(PROV_BRR_INFO, prov_brr, MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES), 0x00);

    for (index = 0; index < MS_CONFIG_LIMITS(MS_NUM_PROVISIONING_INTERFACES); index ++)
    {
        prov_brr[index].handle = BRR_HANDLE_INVALID;
    }

    PROV_TRC ("[PROV] Done.\n");
}

void ms_prov_shutdown (void)
{
    PROV_TRC ("[PROV] Shutting down Provisioning layer...\n");
    PROV_TRC ("[PROV] Done.\n");
}

