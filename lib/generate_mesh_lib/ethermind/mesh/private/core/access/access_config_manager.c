/**
    \file access_config_manager.c

    This file defines data structures related to Configuration Server states,
    along with associated interfaces to access/modify the states.
*/

/*
    Copyright (C) 2017. Mindtree Ltd.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "access_internal.h"
#include "access_extern.h"

#include "MS_access_api.h"
#include "sec_tbx.h"
#include "ltrn_internal.h"
#include "net_extern.h"
#include "net_internal.h"
#include "MS_net_api.h"


#ifdef MS_ACCESS

/* --------------------------------------------- Global Definitions */

/**
    Composition Data

    - Config Composition Data Get
*/

/**
    Default TTL

    Valid values: 0x00, 0x02 - 0x7F.
    [Table 4.10: Default TTL values]

    - Config Default TTL Get
    - Config Default TTL Set
*/
UINT8 access_default_ttl;

/** Start unicast Address */
MS_NET_ADDR  ms_start_unicast_addr = 0x0002;

/** Stop unicast Address */
MS_NET_ADDR  ms_stop_unicast_addr = 0x7fff;



/* Macro to set default TTL */
#define ACCESS_CM_SET_DEFAULT_TTL(ttl) \
    access_default_ttl = (ttl)

/* Macro to check if valid default TTL */
#define MS_IS_VALID_DEFAULT_TTL(ttl) \
    (((0x00 == (ttl)) || ((0x02 <= (ttl)) && (0x7F >= (ttl)))) ? MS_TRUE: MS_FALSE)

/**
    Current IV Index and associated update state

    IV Index  | IV Update Flag | IV Update           | IV Index  | IV Index Used
              |                | Procedure State     | Accepted  | when transmitting
    ----------+----------------+---------------------+-----------+------------------
    n         | 0              | Normal              | n-1, n    | n
    ----------+----------------+---------------------+-----------+------------------
    m (m=n+1) | 1              | In Progress         | m-1, m    | m-1
    ----------+----------------+---------------------+-----------+------------------
    m         | 0              | Normal              | m-1, m    | m
    ----------+----------------+---------------------+-----------+------------------

    Note:
    Tx using 'current IV index' - 'IV Update State value'
    Rx using 'current IV Index' and 'current IV Index - 1'
*/
MS_ACCESS_IV_INDEX ms_iv_index;

/**
    Mesh Features (Proxy/Friend/Relay/Low Power)
    Secure Network Beacon state is also maintained in this data structure.

    - Config Proxy/Friend/Relay/Low Power/Secure Network Beacon Get
    - Config Proxy/Friend/Relay/Low Power/Secure Network Beacon Set
*/
UINT8 ms_features;

/* Macro to set Features field */
#define ACCESS_CM_SET_FEATURES(f) \
    ms_features = (f)

/* Macro to get Features field */
#define ACCESS_CM_GET_FEATURES(f) \
    (f) = ms_features

/** Relay Feature Bit */
#define MS_FEATURE_BIT_RELAY    0x00
#if (MS_FEATURE_BIT_RELAY != MS_FEATURE_RELAY)
    #error "MS_FEATURE_BIT_RELAY != MS_FEATURE_RELAY"
#endif /* (MS_FEATURE_BIT_RELAY != MS_FEATURE_RELAY) */

/** Proxy Feature Bit */
#define MS_FEATURE_BIT_PROXY    0x01
#if (MS_FEATURE_BIT_PROXY != MS_FEATURE_PROXY)
    #error "MS_FEATURE_BIT_PROXY != MS_FEATURE_PROXY"
#endif /* (MS_FEATURE_BIT_PROXY != MS_FEATURE_PROXY) */

/** Friend Feature Bit */
#define MS_FEATURE_BIT_FRIEND   0x02
#if (MS_FEATURE_BIT_FRIEND != MS_FEATURE_FRIEND)
    #error "MS_FEATURE_BIT_FRIEND != MS_FEATURE_FRIEND"
#endif /* (MS_FEATURE_BIT_FRIEND != MS_FEATURE_FRIEND) */

/** Low Power Feature Bit */
#define MS_FEATURE_BIT_LPN      0x03
#if (MS_FEATURE_BIT_LPN != MS_FEATURE_LPN)
    #error "MS_FEATURE_BIT_LPN != MS_FEATURE_LPN"
#endif /* (MS_FEATURE_BIT_LPN != MS_FEATURE_LPN) */

/** Secure Network Beacon Feature Bit */
#define MS_FEATURE_BIT_SEC_NET_BEACON      0x04
#if (MS_FEATURE_BIT_SEC_NET_BEACON != MS_FEATURE_SEC_NET_BEACON)
    #error "MS_FEATURE_BIT_SEC_NET_BEACON != MS_FEATURE_SEC_NET_BEACON"
#endif /* (MS_FEATURE_BIT_SEC_NET_BEACON != MS_FEATURE_SEC_NET_BEACON) */

/**
    Enable Relay/Proxy/Friend/Low Power/Secure Network Beacon Feature bits
*/
#define MS_ENABLE_FEATURE(f) \
    (ms_features) |= (1 << (f))

/** Disable Relay/Proxy/Friend/Low Power/Secure Network Beacon Feature bits */
#define MS_DISABLE_FEATURE(f) \
    (ms_features) &= (~(1 << (f)))

/** Is Relay/Proxy/Friend/Low Power/Secure Network Beacon Feature enabled */
#define MS_IS_FEATURE_ENABLED(f) \
    ((0 != ((ms_features) & (1 << (f)))) ? MS_TRUE: MS_FALSE)

/** Is Relay/Proxy/Friend/Low Power/Secure Network Beacon Feature disabled */
#define MS_IS_FEATURE_DISABLED(f) \
    ((0 == ((ms_features) & (1 << (f)))) ? MS_TRUE: MS_FALSE)

/**
    Current friendship role of the device
    - MS_FRND_ROLE_INVALID : Not in Friendship
    - MS_FRND_ROLE_LPN     : Low Power Node
    - MS_FRND_ROLE_FRIEND  : Friend Node
*/
DECL_STATIC UINT8 ms_friend_role;

/**
    List of Element Address Entries.

    0-th entry is for local device.
    Rest for the friend elements.
*/
MS_DEFINE_GLOBAL_ARRAY(MS_ELEMENT_ADDR_ENTRY, ms_element_addr_table, 1 + MS_CONFIG_LIMITS(MS_MAX_LPNS));

/**
    All addresses - Unicast, Group and Virtual Addresses are maintained
    in following two tables.

    Subscription list will index to both these tables.
    Index 0 to (MS_MAX_NON_VIRTUAL_ADDRS - 1) will correspond to non-virtual address.
    Index MS_MAX_NON_VIRTUAL_ADDRS to (MS_MAX_ADDRS - 1) will correspond to virtual address.

    Note: MS_MAX_ADDRS is defined as (MS_MAX_NON_VIRTUAL_ADDRS + MS_MAX_VIRTUAL_ADDRS)
*/

/** List of Virtual Address Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_VIRTUAL_ADDR_ENTRY, ms_virtual_addr_table, MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS));

/** List of non-Virtual Address Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_NON_VIRTUAL_ADDR_ENTRY, ms_non_virtual_addr_table, MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS));

/**
    Model Publication

    - Config Model Publication Get
    - Config Model Publication Set
    - Config Model Publication Virtual Address Set
*/

/**
    Subscription List

    - Config Model Subscription Add
    - Config Model Subscription Virtual Address Add
    - Config Model Subscription Delete
    - Config Model Subscription Virtual Address Delete
    - Config Model Subscription Virtual Address Overwrite
    - Config Model Subscription Overwrite
    - Config Model Subscription Delete All
    - Config SIG Model Subscription Get
    - Config Vendor Model Subscription Get
*/

/**
    List of Subnet Entries.
    Network Keys associated with Friends (LPNs) are also maintained in the same table.

    Friend/LPN Network Keys are also maintained in the same data structure.
*/
MS_DEFINE_GLOBAL_ARRAY(MS_NETKEY_ENTRY, ms_subnet_table, (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)));
UINT16          ms_netkey_count;

MS_DEFINE_GLOBAL_ARRAY(MS_NET_ADDR, ms_key_refresh_whitelist, (MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS)));

UINT16          ms_key_refresh_count;



/** Invalid Key Index - used for both NetKey and AppKey */
#define MS_INVALID_KEY_INDEX    MS_INVALID_SUBNET_HANDLE

/** Init Routine for Net Key Entries */
#define MS_INIT_NETKEY_ENTRY(handle) \
    ms_subnet_table[(handle)].fixed.netkey_index = MS_INVALID_KEY_INDEX

/** Check if subnet is free */
#define MS_IS_SUBNET_FREE(handle) \
    (MS_INVALID_KEY_INDEX == ms_subnet_table[(handle)].fixed.netkey_index)

/** Get NetKey Index for the Subnet Handle */
#define MS_SUBNET_NETKEY_INDEX(handle) \
    ms_subnet_table[(handle)].fixed.netkey_index

/** Get Node Identity State for the Subnet Handle */
#ifdef MS_PROXY_SUPPORT
#define MS_SUBNET_NODE_ID_STATE(handle) \
    ms_subnet_table[(handle)].fixed.node_id_state

#define MS_SUBNET_NODE_ID_STATE_SET(handle,state) \
    MS_SUBNET_NODE_ID_STATE((handle)) = (state)
#else /* MS_PROXY_SUPPORT */
#define MS_SUBNET_NODE_ID_STATE(handle)
#define MS_SUBNET_NODE_ID_STATE_SET(handle,state)
#endif /* MS_PROXY_SUPPORT */

/** Get Key Refresh Phase State for the Subnet Handle */
#define MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle) \
    ms_subnet_table[(handle)].fixed.key_refresh_phase

/** List of AppKey Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_APPKEY_ENTRY, ms_appkey_table, MS_CONFIG_LIMITS(MS_MAX_APPS));

/** Init Routine for App Key Entries */
#define MS_INIT_APPKEY_ENTRY(i) \
    ms_appkey_table[(i)].appkey_index = MS_INVALID_KEY_INDEX

/** Check if AppKey is free */
#define MS_IS_APPKEY_FREE(handle) \
    (MS_INVALID_KEY_INDEX == ms_appkey_table[(handle)].appkey_index)

/** Get AppKey Index for the AppKey Handle */
#define MS_APPKEY_INDEX(index) \
    ms_appkey_table[(index)].appkey_index

/** Get Key Refresh Phase State for the AppKey Handle */
#define MS_APPKEY_REFRESH_PHASE_STATE(handle) \
    ms_appkey_table[(handle)].key_refresh_phase

/** List of Device Key Entries */
MS_DEFINE_GLOBAL_ARRAY(MS_DEV_KEY_ENTRY, ms_dev_key_table, MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS));

/** Count of valid Device Key Entries */
UINT16 ms_dev_key_table_entries;

/** Count of Key Pointer */
UINT16 ms_dev_key_table_pointer;


/** Provisioner address for node */
MS_NET_ADDR ms_provisioner_addr;

UINT16 ms_proxy_filter_addr[5];


/** Init Routine for Device Key Entries */
#define MS_INIT_DEV_KEY_ENTRY(i) \
    EM_mem_set(&ms_dev_key_table[(i)], 0, sizeof(MS_DEV_KEY_ENTRY))

/**
    Node Identity

    - Config Node Identity Get
    - Config Node Identity Set
*/

/**
    (reset)

    - Config Node Reset
*/

/**
    Key Refresh Phase

    - Config Key Refresh Phase Get
    - Config Key Refresh Phase Set
*/

/**
    Network/Relay Transmit State.
    - Composite state of Network/Relay Transmit Count (3-bit) and
     Network/Relay Transmit Inteval Steps (5-bit)
*/
MS_CONFIG_TRANSMIT ms_tx_state[2];

/** Network Tx State Index */
#define MS_NETWORK_TX_STATE_INDEX      0x00
#if (MS_NETWORK_TX_STATE_INDEX != MS_NETWORK_TX_STATE)
    #error "MS_NETWORK_TX_STATE_INDEX != MS_NETWORK_TX_STATE"
#endif /* (MS_NETWORK_TX_STATE_INDEX != MS_NETWORK_TX_STATE) */

/** Relay Tx State Index */
#define MS_RELAY_TX_STATE_INDEX        0x01
#if (MS_RELAY_TX_STATE_INDEX != MS_RELAY_TX_STATE)
    #error "MS_RELAY_TX_STATE_INDEX != MS_RELAY_TX_STATE"
#endif /* (MS_RELAY_TX_STATE_INDEX != MS_RELAY_TX_STATE) */

/** Access Network/Relay Transmit Count */
#define MS_TX_COUNT_STATE(index) \
    ms_tx_state[(index)].tx_count

/** Access Network/Relay Transmit Interval Steps */
#define MS_TX_STEPS_STATE(index) \
    ms_tx_state[(index)].tx_steps


/* --------------------------------------------- Static Global Variables */

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Function */

/**
    \brief Initialize access layer configuration states.

    \par Description
    This routine initializes access layer configuration states.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT ms_access_cm_init(void)
{
    ACCESS_TRC(
        "[CM] Access Configuration Initialization.\n");
    /* Set default TTL as Max */
    ACCESS_CM_SET_DEFAULT_TTL(ACCESS_MAX_TTL);
    /* Set features field as all disabled */
    ACCESS_CM_SET_FEATURES(0);
    /* Initialize Subnet Table */
    MS_INIT_GLOBAL_ARRAY(MS_NETKEY_ENTRY, ms_subnet_table, (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)), 0x00);
    ms_access_cm_init_subnet_table();
    /* Initialize AppKey Table */
    MS_INIT_GLOBAL_ARRAY(MS_APPKEY_ENTRY, ms_appkey_table, MS_CONFIG_LIMITS(MS_MAX_APPS), 0x00);
    ms_access_cm_init_appkey_table();
    /* Initialize Device Key Table */
    MS_INIT_GLOBAL_ARRAY(MS_DEV_KEY_ENTRY, ms_dev_key_table, MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS), 0x00);
    ms_access_cm_init_dev_key_table();
    /* Initialize Element Address Table */
    MS_INIT_GLOBAL_ARRAY(MS_ELEMENT_ADDR_ENTRY, ms_element_addr_table, 1 + MS_CONFIG_LIMITS(MS_MAX_LPNS), 0x00);
    /* Initialize Non-Virtual Address Table */
    MS_INIT_GLOBAL_ARRAY(MS_NON_VIRTUAL_ADDR_ENTRY, ms_non_virtual_addr_table, MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS), 0x00);
    /* Initialize Virtual Address Table */
    MS_INIT_GLOBAL_ARRAY(MS_VIRTUAL_ADDR_ENTRY, ms_virtual_addr_table, MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS), 0x00);
    /* Initialize Network/Relay Tx States */
    EM_mem_set(ms_tx_state, 0, sizeof(ms_tx_state));
    /* Initialize the access ps store */
    access_ps_init();
    return API_SUCCESS;
}

/**
    \brief Initialize Subnet/Net Key Table.

    \par Description
    This routine initializes Subnet/Net Key Table.
*/
void ms_access_cm_init_subnet_table(void)
{
    UINT32 index;
    ACCESS_TRC(
        "[CM] Subnet Table Initialization.\n");

    for (index = 0; index < (UINT32)(MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)); index ++)
    {
        MS_INIT_NETKEY_ENTRY(index);
    }

    /* Reset Netkey Count */
    ms_netkey_count = 0;
    return;
}

/**
    \brief Initialize App Key Table.

    \par Description
    This routine initializes App Key Table.
*/
void ms_access_cm_init_appkey_table(void)
{
    UINT32 index;
    ACCESS_TRC(
        "[CM] AppKeyTable Initialization.\n");

    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index ++)
    {
        MS_INIT_APPKEY_ENTRY(index);
    }

    return;
}

/**
    \brief Initialize Device Key Table.

    \par Description
    This routine initializes Device Key Table.
*/
void ms_access_cm_init_dev_key_table(void)
{
    UINT32 index;
    ACCESS_TRC(
        "[CM] Device Key Table Initialization.\n");

    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS); index ++)
    {
        MS_INIT_DEV_KEY_ENTRY(index);
    }

    /* Initialize Device Key entry count to zero */
    ms_dev_key_table_entries = 0;
    ms_dev_key_table_pointer = ms_start_unicast_addr;
    return;
}


/**
    \brief To get the number of elements in local node

    \par Description
    This routine retrieves the number of elements in local node.

    \param [out] count     Number of elements

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_element_count
(
    /* OUT */ UINT8*   count
)
{
    /* Lock */
    *count = ms_element_addr_table[0].element_count;
    /* Unlock */
    return API_SUCCESS;
}


/**
    \brief To set primary unicast address

    \par Description
    This routine sets primary unicast address.

    \param [in] addr     Primary Unicast Address to be set

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_primary_unicast_address
(
    /* IN */ MS_NET_ADDR    addr
)
{
    API_RESULT                retval;

    /* Lock */

    /* Check if a valid unicast address */
    if (MS_TRUE == MS_IS_UNICAST_ADDR(addr))
    {
        retval = API_SUCCESS;
        ACCESS_TRC(
            "[CM] SUCCESS. Set primary unicast address as 0x%04X\n", addr);
        /* Save Element Address for the local node */
        ms_element_addr_table[0].uaddr = addr;
        ACCESS_TRC(
            "[CM] Number of elements in local node 0x%02X\n", ms_element_addr_table[0].element_count);
    }
    else
    {
        retval = MS_INVALID_ADDRESS;
        ACCESS_ERR(
            "[CM] Invalid Unicast Address 0x%04X. Returning 0x%04X\n", addr, retval);
    }

    /* Unlock */
    return retval;
}

/**
    \brief To get primary unicast address

    \par Description
    This routine gets primary unicast address.

    \param [out] addr     Memory location where Primary Unicast Address to be filled

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_primary_unicast_address
(
    /* OUT */ MS_NET_ADDR*     addr
)
{
    API_RESULT retval;

    /* Lock */

    /* Check for NULL Pointer */
    if (NULL == addr)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in Get Primary Unicast Address. Returning 0x%04X\n", retval);
    }
    else
    {
        /* Get primary address */
        *addr = ms_element_addr_table[0].uaddr;
        ACCESS_TRC(
            "[CM] Returning Primary Unicast Address as 0x%04X\n", (*addr));
        retval = API_SUCCESS;
    }

    /* Unlock */
    return retval;
}

/**
    \brief To check if valid element address to receive a packet

    \par Description
    This routine checks if destination address in a received packet matches
    with any of the known element address of local or friend device.

    \param [in] addr     Unicast Address to search

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_is_valid_element_address
(
    /* IN */ MS_NET_ADDR    addr
)
{
    API_RESULT                retval;
    ACCESS_TRC(
        "[ACCESS] Check if '0x%04X' is a valid element address\n", addr);
    /* Lock */
    retval = ACCESS_NO_MATCH;

    /* Check if a valid unicast address */
    if (MS_TRUE == MS_IS_UNICAST_ADDR(addr))
    {
        UINT32 index;
        MS_NET_ADDR  start_addr;
        UINT8        element_count;

        /* TODO: Friend LPN addresses are maintained now separately. This can be relooked */
        for (index = 0; index < (UINT32)(1 + MS_CONFIG_LIMITS(MS_MAX_LPNS)); index ++)
        {
            /**
                 ACCESS_TRC(
                 "[ACCESS] Element Address Table[%d], Address: 0x%04X, Element Count: 0x%02X\n",
                 index, ms_element_addr_table[index].uaddr, ms_element_addr_table[index].element_count);
            */
            if (0 != ms_element_addr_table[index].uaddr)
            {
                /* Element Start Address and Element Count */
                start_addr = ms_element_addr_table[index].uaddr;
                element_count = ms_element_addr_table[index].element_count;

                /* Check if the address is in range */
                if ((addr >= start_addr) &&
                        (addr < (start_addr + element_count)))
                {
                    retval = API_SUCCESS;
                    ACCESS_TRC(
                        "[ACCESS] Found Element Address Match\n");
                    break;
                }
            }
        }
    }
    else
    {
        retval = MS_INVALID_ADDRESS;
        ACCESS_ERR(
            "[CM] Invalid Unicast Address 0x%04X. Returning 0x%04X\n", addr, retval);
    }

    /* Unlock */
    return retval;
}

/**
    \brief To check if Fixed Group Address in receive packet to be processed

    \par Description
    This routine checks if destination address in a received packet
    as a Fixed Group Address to be processed.

    \param [in] addr     A valid Fixed Group Address, to be checked

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_is_fixed_group_addr_to_be_processed
(
    /* IN */ MS_NET_ADDR    addr
)
{
    API_RESULT retval;

    /**
        A message sent to the all - nodes address shall be processed by
        the primary element of all nodes.
    */
    if ((MS_NET_ADDR_ALL_NODES == addr) ||
            /**
                A message sent to the all - relays address shall be processed by
                the primary element of all nodes that have the relay functionality
                enabled.
            */
            ((MS_NET_ADDR_ALL_RELAYS == addr) && (MS_TRUE == MS_IS_FEATURE_ENABLED(MS_FEATURE_RELAY))) ||
            /**
                A message sent to the all - friends address shall be processed by
                the primary element of all nodes that have the friend functionality
                enabled.
            */
            ((MS_NET_ADDR_ALL_FRIENDS == addr) && (MS_TRUE == MS_IS_FEATURE_ENABLED(MS_FEATURE_FRIEND))) ||
            /**
                A message sent to the all - proxies address shall be processed by
                the primary element of all nodes that have the proxy functionality
                enabled.
            */
            ((MS_NET_ADDR_ALL_PROXIES == addr) && (MS_TRUE == MS_IS_FEATURE_ENABLED(MS_FEATURE_PROXY))))
    {
        retval = API_SUCCESS;
    }
    else
    {
        retval = API_FAILURE;
    }

    return retval;
}

/**
    \brief To check if valid subscription address to receive a packet

    \par Description
    This routine checks if destination address in a received packet matches
    with any of the known subscription address of local or friend device.

    \param [in] addr     Address to search

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_is_valid_subscription_address
(
    /* IN */ MS_NET_ADDR    addr
)
{
    API_RESULT                retval;
    MS_ACCESS_ADDRESS_HANDLE  handle;
    /* Lock */
    /* Look into non-virtual address table */
    retval = ms_search_address
             (
                 &addr,
                 MS_ADDR_OP_TYPE_SUBSCRIBE,
                 &handle
             );

    if (API_SUCCESS != retval)
    {
        /* Search in virtual address table */
        retval = ms_search_virtual_address
                 (
                     &addr,
                     MS_ADDR_OP_TYPE_SUBSCRIBE,
                     &handle
                 );
    }

    /* Unlock */
    return retval;
}

/**
    \brief To set default TTL

    \par Description
    This routine sets default TTL.

    \param [in] ttl     Default TTL to be set

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_default_ttl
(
    /* IN */ UINT8    ttl
)
{
    API_RESULT retval;

    /* Lock */

    /* Check if a valid default TTL */
    if (MS_TRUE == MS_IS_VALID_DEFAULT_TTL(ttl))
    {
        retval = API_SUCCESS;
        ACCESS_TRC(
            "[CM] Set default TTL as 0x%02X\n", ttl);
        /* Set default TTL */
        ACCESS_CM_SET_DEFAULT_TTL(ttl);
    }
    else
    {
        /* TODO: Return appropriate error code */
        retval = API_FAILURE;
        ACCESS_ERR(
            "[CM] Invalid TTL. Returning 0x%04X\n", retval);
    }

    /* Unlock */
    return retval;
}

/**
    \brief To get default TTL

    \par Description
    This routine gets default TTL.

    \param [in] ttl     Memory location where default TTL to be filled

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_default_ttl
(
    /* IN */ UINT8*     ttl
)
{
    API_RESULT retval;

    /* Lock */

    /* Check for NULL Pointer */
    if (NULL == ttl)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in Get default TTL. Returning 0x%04X\n", retval);
    }
    else
    {
        /* Get default TTL */
        ACCESS_CM_GET_DEFAULT_TTL(*ttl);
        ACCESS_TRC(
            "[CM] Returning default TTL as 0x%04X\n", (*ttl));
        retval = API_SUCCESS;
    }

    /* Unlock */
    return retval;
}

/* Secure Network Beacon Timeout Handler */
//static void ms_snb_local_timeout_handler (void * args, UINT16 size)
//{
//   MS_SUBNET_HANDLE         subnet_handle;

//   MS_IGNORE_UNUSED_PARAM(size);

//   subnet_handle = (MS_SUBNET_HANDLE)(*((UINT32 *)args));

//   ms_snb_local_timer_handle = EM_TIMER_HANDLE_INIT_VAL;

//   NET_TRC(
//   "[NET] *** SNB Timeout\n");

//

//   return;
//}


/**
    \brief To set IV Index

    \par Description
    This routine sets IV Index.

    \param [in] iv_index          IV Index to be set
    \param [in] iv_update_flag    IV Update Flag

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_iv_index
(
    /* IN */ UINT32    iv_index,
    /* IN */ UINT8     iv_update_flag
)
{
    #if 0
    UINT32 current;
    API_RESULT retval;
    /* Lock */
    ACCESS_TRC(
        "[CM] Set IV Index as 0x%08X. Flag:0x%02X\n",
        iv_index, iv_update_flag);
    retval = API_FAILURE;
//    printf(
//    "[CM] Set IV Index as 0x%08X. Flag:0x%02X\n",
//    iv_index, iv_update_flag);
//    printf(
//    "[CM] ms iv update Index as 0x%08X. Flag:0x%02X Time:0x%02X\n",
//    ms_iv_index.iv_index, ms_iv_index.iv_update_state,ms_iv_index.iv_time);

    /* Ignore checks if set locally. TODO: Do it neatly */
    if (!(0x80 & iv_update_flag))
    {
        /*
            Ignore the IV Update when
            - Node in Normal operation and IV Index received < known IvIndex or
             > known IvIndex + 42
        */
        if ((MS_FALSE == ms_iv_index.iv_update_state) &&
                ((iv_index <= ms_iv_index.iv_index) || (iv_index >(ms_iv_index.iv_index + 42))))
        {
            ACCESS_ERR("IV Index Processing Failed in Normal Operation\n");
//            printf("IV Index Processing Failed in Normal Operation\n");
            return API_FAILURE;
        }

        /* TODO: This is commented for PTS testing */
        /*
            Check if the previous update has happened before 96 hours. If update time is 0,
            then the IV update is considered
        */

        if(MS_TRUE == iv_update_flag)
        {
            if(0 != ms_iv_index.iv_update_state)
            {
                EM_get_current_time(&current);
                current =  (current > local_base_tick) ?
                           (current - local_base_tick) :
                           (0xFFFFFFFF - current + local_base_tick);
                current = (current*5) >> 3;
                ms_iv_index.iv_time = current/1000;

                if(ms_iv_index.iv_time < 345600)
                {
                    ACCESS_ERR("IV Index Processing Failed before minimum time\n");
//                    printf("IV Index Processing Failed before minimum time\n");
                    return API_FAILURE;
                }
            }
            else
            {
                EM_get_current_time(&local_base_tick);
                ms_iv_index.iv_time = 0;
            }
        }

        retval = API_SUCCESS;
//        if ((0 != ms_iv_index.iv_time) &&
//            (MS_TRUE == iv_update_flag))
//        {
//            ((current - ms_iv_index.iv_time) < 345600000) &&
//            ACCESS_ERR("IV Index Processing Failed before minimum time\n");
//            printf("IV Index Processing Failed before minimum time\n");
//            return API_FAILURE;
//        }
//        ms_iv_index.iv_time = current;
    }
    else            //local
    {
        if(ms_iv_index.iv_update_state != (iv_update_flag & (UCHAR)(~0x80)))
        {
            if (EM_TIMER_HANDLE_INIT_VAL == ms_snb_local_timer_handle)
            {
                retval = EM_start_timer
                         (
                             &ms_snb_local_timer_handle,
                             (MS_SNB_LOCAL_TIMEOUT),
                             ms_snb_local_timeout_handler,
                             &iv_update_flag,
                             sizeof(iv_update_flag)
                         );

                if(retval)
                {
                    ACCESS_ERR(
                        "[CM] Start Timer Failed\n");
                }
                else
                {
                    retval = API_SUCCESS;
                }
            }
        }
    }

    if(retval == API_SUCCESS)
    {
        ms_iv_index.iv_index = iv_index;
        ms_iv_index.iv_update_state = (iv_update_flag & (UCHAR)(~0x80));
        /* TODO: Do we need to send a secure beacon? */
        /* Unlock */
        /* Store in PS */
        ms_access_ps_store(MS_PS_RECORD_SUBNETS);
    }

    #endif
    /* Lock */
    ACCESS_TRC(
        "[CM] Set IV Index as 0x%08X. Flag:0x%02X\n",
        iv_index, iv_update_flag);
    printf(
        "[CM] Set IV Index as 0x%08X. Flag:0x%02X\n",
        iv_index, iv_update_flag);
    ms_iv_index.iv_index = iv_index;
    ms_iv_index.iv_update_state = iv_update_flag;
    /* TODO: Do we need to send a secure beacon? */
    /* Unlock */
    /* Store in PS */
    ms_access_ps_store(MS_PS_RECORD_SUBNETS);
    return API_SUCCESS;
}

/**
    \brief To get IV Index

    \par Description
    This routine gets IV Index.

    \param [out] iv_index          Memory location where IV Index to be filled
    \param [out] iv_update_flag    Memory location where IV Update Flag to be filled

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_iv_index
(
    /* OUT */ UINT32*     iv_index,
    /* OUT */ UINT8*      iv_update_flag
)
{
    API_RESULT retval;

    /* Lock */

    /* Check for NULL Pointer */
    if (NULL == iv_index)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in Get IVI. Returning 0x%04X\n", retval);
    }
    else
    {
        /**
            Get Current IV Index
            Tx using 'current IV index' - 'IV Update State value'
        */
        *iv_index = ms_iv_index.iv_index - (ms_iv_index.iv_update_state & 0x01);

        if (NULL != iv_update_flag)
        {
            *iv_update_flag = ms_iv_index.iv_update_state;
        }

        ACCESS_TRC("[CM] Returning IV Index as 0x%08X\n", (*iv_index));
//        printf("[CM] Returning IV Index as 0x%08X\n", (*iv_index));
        retval = API_SUCCESS;
    }

    /* Unlock */
    return retval;
}

/**
    \brief To get IV Index by IVI

    \par Description
    This routine gets IV Index based on the IVI in the received packet.

    \param [in]  ivi          Least Significant bit of the IV Index used
                              in the nonce to authenticate and encrypt
                              the Network PDU.
    \param [out] iv_index     Memory location where IV Index to be filled

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_iv_index_by_ivi
(
    /* IN */  UINT8       ivi,
    /* OUT */ UINT32*     iv_index
)
{
    API_RESULT retval;
    UINT32     current_iv_index;

    /* Lock */

    /* Check for NULL Pointer */
    if (NULL == iv_index)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in Get IVI. Returning 0x%04X\n", retval);
    }
    else
    {
        /* Get Current IV Index */
        current_iv_index = ms_iv_index.iv_index;

        /**
            Due to the possibility of IV Update Procedure,
            IV Index used by the peer device can be the current one
            available with Local Device or the previous IV Index
            (which is one less than the current IV Index).
        */
        if ((UINT8)(current_iv_index & ACCESS_NET_IVI_MASK) == (UINT8)(ivi & 0x01))
        {
            *iv_index = current_iv_index;
        }
        else
        {
            /* TODO: What should happen if 'current_iv_index' is 0 */
            /* Assert 'current_iv_index' is not ZERO */
            *iv_index = (current_iv_index - 1);
        }

        ACCESS_TRC(
            "[CM] Returning IV Index as 0x%08X\n", *(iv_index));
        retval = API_SUCCESS;
    }

    /* Unlock */
    return retval;
}

/**
    \brief To enable/disable a feature

    \par Description
    This routine enables/disables a feature field.

    \param [in] enable     Enable or Disable
    \param [in] feature    Relay, proxy, friend or Low Power

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_features_field
(
    /* IN */ UINT8    enable,
    /* IN */ UINT8    feature
)
{
    API_RESULT retval;
    UINT8      toggled;
    /* Lock */
    /* TODO: Parameter Validation */
    retval = API_SUCCESS;

    if (MS_ENABLE == enable)
    {
        ACCESS_TRC(
            "[CM] Enable 0x%02X\n", feature);
        toggled = MS_IS_FEATURE_DISABLED(feature);
        /* Enable feature field */
        MS_ENABLE_FEATURE(feature);
    }
    else
    {
        ACCESS_TRC(
            "[CM] Enable 0x%02X\n", feature);
        toggled = MS_IS_FEATURE_ENABLED(feature);
        /* Enable feature field */
        MS_DISABLE_FEATURE(feature);
    }

    /* Trigger Heartbeat */
    if (MS_TRUE == toggled)
    {
        MS_trn_trigger_heartbeat(1 << feature);
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_TX_STATES_FEATURES);
    }

    /* Unlock */
    return retval;
}

/**
    \brief To get state of a feature

    \par Description
    This routine gets the state of a feature field.

    \param [out] enable     Memory location where Enable or Disable status to be filled.
    \param [in]  feature    Relay, proxy, friend or Low Power

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_features_field
(
    /* OUT */ UINT8*   enable,
    /* IN */  UINT8    feature
)
{
    API_RESULT retval;
    /* Lock */
    /* TODO: Parameter Validation */
    retval = API_SUCCESS;
    (*enable) = (UINT8)MS_IS_FEATURE_ENABLED(feature);
    ACCESS_TRC(
        "[CM] Feature 0x%02X is %s\n", feature,
        (((*enable) == MS_TRUE) ? "Enabled" : "Disabled"));
    /* Unlock */
    return retval;
}

/**
    \brief To get state of all features

    \par Description
    This routine gets the state of all features.

    \param [out] features    State of Relay, proxy, friend and Low Power field

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_features
(
    /* OUT */ UINT8*    features
)
{
    API_RESULT retval;
    /* Lock */
    /* TODO: Parameter Validation */
    retval = API_SUCCESS;
    ACCESS_CM_GET_FEATURES(*features);
    ACCESS_TRC(
        "[CM] Returning Features: 0x%02X\n", *features);
    /* Unlock */
    return retval;
}


/**
    \brief To get friendship role of the node

    \par Description
    This routine gets the current friendship role of the node.

    \param [out] frnd_role    Friend role

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_friendship_role
(
    /* OUT */ UINT8*   frnd_role
)
{
    API_RESULT retval;
    /* Lock */
    /* TODO: Parameter Validation */
    retval = API_SUCCESS;
    (*frnd_role) = ms_friend_role;
    ACCESS_TRC("[CM] Get current friendship role 0x%02X\n", *frnd_role);
    /* Unlock */
    return retval;
}


/**
    \brief To set friendship role of the node

    \par Description
    This routine sets the current friendship role of the node.

    \param [out] frnd_role    Friend role

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_friendship_role
(
    /* IN */ UINT8   frnd_role
)
{
    API_RESULT retval;
    /* Lock */
    /* TODO: Parameter Validation */
    retval = API_SUCCESS;

    /**
        Check if the current friend role or next friend role is LPN,
        try to send heartbeat.
    */
    if ((MS_FRND_ROLE_LPN == frnd_role) || (MS_FRND_ROLE_LPN == ms_friend_role))
    {
        MS_trn_trigger_heartbeat(1 << MS_FEATURE_LPN);
    }

    ms_friend_role = frnd_role;
    ACCESS_TRC("[CM] Set current friendship role 0x%02X\n", frnd_role);
    /* Unlock */
    return retval;
}


/**
    \brief To add Device Key

    \par Description
    This routine adds Device Key entry, along with corresponding
    Primary Device Address and Number of elements.

    \param [in] dev_key        Device Key to be added.
    \param [in] uaddr          Unicast Address of the first element.
    \param [in] num_elements   Number of elements.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_add_device_key
(
    /* IN */ UINT8*         dev_key,
    /* IN */ MS_NET_ADDR    uaddr,
    /* IN */ UINT8          num_elements
)
{
    API_RESULT retval;
    UINT32     index;
    UINT32      i;
    UINT16      u_s_addr;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Add Device Key. uaddr: 0x%04X, Num Elements: 0x%02X\n",
        uaddr, num_elements);
    ACCESS_debug_dump_bytes(dev_key, 16);
    retval = API_FAILURE;

    /* TODO: Add provision to store the UUID associated with the device */

    /* TODO: Right now there is only one entry for Device Key */
    /* Check if the Device Key table is already full */
    if (ms_dev_key_table_entries >= MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS))
    {
        ACCESS_ERR(
            "[CM] Device Key Table Full. Returning\n");
        return ACCESS_DEV_KEY_TABLE_FULL;
    }

    for(i = 0; i < ms_dev_key_table_entries; i++)
    {
        if(ms_dev_key_table[i].uaddr == uaddr)
        {
            retval = API_SUCCESS;
            break;
        }
    }

    index = (retval == API_SUCCESS) ? i : ms_dev_key_table_entries;
    EM_mem_copy(&(ms_dev_key_table[index].dev_key[0]), dev_key, 16);
    ms_dev_key_table[index].uaddr = uaddr;
    ms_dev_key_table[index].num_elements = num_elements;

    if(retval == API_FAILURE)
    {
        ms_dev_key_table_entries++;
    }

    if(prov_role == PROV_ROLE_PROVISIONER)
    {
        u_s_addr = ms_dev_key_table_pointer;
        retval = API_FAILURE;

        while(1)
        {
            ms_dev_key_table_pointer = (ms_dev_key_table_pointer == ms_stop_unicast_addr)?
                                       ms_start_unicast_addr : (++ms_dev_key_table_pointer);

            if(ms_dev_key_table_pointer == u_s_addr)
            {
                ACCESS_ERR(
                    "[CM] Device Key Table Pointer Overrun. Returning\n");
                return API_FAILURE;
            }

            for(i = 0; i < ms_dev_key_table_entries; i++)
            {
                if(ms_dev_key_table[i].uaddr == ms_dev_key_table_pointer)
                {
                    retval = API_SUCCESS;
                    break;
                }
            }

            if(retval == API_FAILURE)
            {
                break;
            }
        }
    }

    /* PS Store */
    ms_access_ps_store(MS_PS_RECORD_DEV_KEYS);
    retval = API_SUCCESS;
    ACCESS_TRC(
        "[CM] Added Device Key at Index 0x%04X. Returning 0x%04X\n",
        index, retval);
    return retval;
}

/**
    \brief To get Device Key

    \par Description
    This routine gets Device Key entry.

    \param [in]  dev_key_index    Device Key Index.
    \param [out] dev_key          Pointer to Device Key to be returned.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_device_key
(
    /* IN */  UINT8     dev_key_index,
    /* OUT */ UINT8**   dev_key
)
{
    API_RESULT retval;
    ACCESS_TRC(
        "[CM] Get Device Key at Index 0x%02X\n", dev_key_index);

    /* Parameter Validation */
    if ((MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS) <= dev_key_index) || (NULL == dev_key))
    {
        retval = ACCESS_INVALID_PARAMETER_VALUE;
        ACCESS_ERR(
            "[CM] Invalid Device Key Index. Returning 0x%04X\n",
            retval);
        return retval;
    }
    else if (dev_key_index >= ms_dev_key_table_entries)
    {
        retval = ACCESS_PARAMETER_OUTSIDE_RANGE;
        ACCESS_ERR(
            "[CM] Device Key Index:0x%02X > Number of Valid Device Keys:0x%04X. Returning 0x%04X\n",
            dev_key_index, ms_dev_key_table_entries, retval);
        return retval;
    }

    *dev_key = &(ms_dev_key_table[dev_key_index].dev_key[0]);
    retval = API_SUCCESS;
    ACCESS_TRC(
        "[CM] Get Device Key Returning 0x%04X, with pointer to Device Key\n", retval);
    ACCESS_debug_dump_bytes((*dev_key), 16);
    return retval;
}

/* TODO: Below three APIs are only required for Provisioner Role */

/**
    \brief To remove all Device Keys

    \par Description
    This routine removes all Device Keys from table.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_remove_all_device_keys(void)
{
    ACCESS_TRC(
        "[CM] Remove All Device Keys\n");
    ms_access_cm_init_dev_key_table();
    /* PS Store */
    ms_access_ps_store(MS_PS_RECORD_DEV_KEYS);
    return API_SUCCESS;
}

/**
    \brief To get list of Provisioned Device List

    \par Description
    This routine returns list of Provisioned Devices from the Device Key Table.

    \param [in]    prov_dev_list   Provisioned Device List.
    \param [inout] num_entries     Size of the Device Key List provided by the caller.
                                   This routine will return the number of entries
                                   in the Device Key Table.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_prov_devices_list
(
    /* IN */ MS_PROV_DEV_ENTRY*    prov_dev_list,
    /* OUT */ UINT16*               num_entries,
    /* OUT */ UINT16*               pointer
)
{
    UINT16      index, count,i_pointer;
    API_RESULT       retval;
    ACCESS_TRC(
        "[CM] Get Provision Devices List\n");

    /* Parameter Validation */
    if ((NULL == prov_dev_list) || (NULL == num_entries))
    {
        ACCESS_ERR(
            "[CM] Invalid Parameters. Returning\n");
        return ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }

    count = ms_dev_key_table_entries;
    i_pointer = ms_dev_key_table_pointer;
    ACCESS_TRC(
        "[CM] Max List size: 0x%04X\n", count);
//    printf(
//    "[CM] Max List size: 0x%04X\n", count);

//    /* Set the Limit */
//    if (count > ms_dev_key_table_entries)
//    {
//        count = ms_dev_key_table_entries;
//    }

    /* TODO: Replace with a memcopy */
    for (index = 0; index < count; index++)
    {
        prov_dev_list[index].uaddr = ms_dev_key_table[index].uaddr;
        prov_dev_list[index].num_elements = ms_dev_key_table[index].num_elements;
    }

    /* Return the Device Key Table entry count */
    *num_entries = count;
    retval = API_FAILURE;

    for (index = 0; index < count; index++)
    {
        if(prov_dev_list[index].uaddr == i_pointer)
        {
            retval = API_SUCCESS;
            break;
        }
    }

    if(retval == API_SUCCESS)
    {
        return API_FAILURE;
    }
    else
    {
        *pointer = i_pointer;
    }

    return API_SUCCESS;
}

/**
    \brief To get Device Key Handle

    \par Description
    This routine returns Device Key Handle for a given Primary Element Address
    entry in Device Key Table.

    \param [in]  prim_elem_uaddr   Primary element address to be searched
    \param [out] handle            Device Key Table Handle, if match is found.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_device_key_handle
(
    /* IN */  MS_NET_ADDR                  prim_elem_uaddr,
    /* OUT */ MS_ACCESS_DEV_KEY_HANDLE*    handle
)
{
    API_RESULT retval;
    UINT16     index;
    ACCESS_TRC(
        "[CM] Get Device Key Handle for Primary Element Address: 0x%04X\n",
        prim_elem_uaddr);

    /* Parameter Validation */
    if (NULL == handle)
    {
        ACCESS_ERR(
            "[CM] Invalid Parameters. Returning\n");
        return ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }

    /* Loop through Device Key Table entries */
    retval = ACCESS_NO_MATCH;

    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS); index++)
    {
        if (ms_dev_key_table[index].uaddr == prim_elem_uaddr)
        {
            ACCESS_TRC(
                "[CM] Match Found at Index: 0x%04X\n", index);
            *handle = (MS_ACCESS_DEV_KEY_HANDLE)index;
            retval = API_SUCCESS;
            break;
        }
    }

    return retval;
}

/**
    \brief To delete Device Key

    \par Description
    This routine returns status for a given Primary Element Address
    entry in Device Key Table.

    \param [in] handle            Device Key Table Handle, if match is found.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_delete_device_key
(
    /* IN */ MS_ACCESS_DEV_KEY_HANDLE       handle
)
{
    API_RESULT retval;
    UINT16     index;
    ACCESS_TRC(
        "[CM] Delete Device Key for Primary Element Handle: 0x%04X\n",
        handle);

    /* Parameter Validation */
    if (handle >= ms_dev_key_table_entries)
    {
        ACCESS_ERR(
            "[CM] Invalid Parameters. Returning\n");
        return ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }

    /* Loop through Device Key Table entries */
    retval = ACCESS_NO_MATCH;

    for (index = handle; index < ms_dev_key_table_entries; index++)
    {
        if(index == ms_dev_key_table_entries-1)
        {
            ms_dev_key_table[index].uaddr = 0;
            ms_dev_key_table[index].num_elements = 0;
            EM_mem_set(&(ms_dev_key_table[index].dev_key[0]), 0, 16);
            retval = API_SUCCESS;
            break;
        }
        else
        {
            ms_dev_key_table[index].uaddr = ms_dev_key_table[index+1].uaddr;
            ms_dev_key_table[index].num_elements = ms_dev_key_table[index+1].num_elements;
            EM_mem_copy(&(ms_dev_key_table[index].dev_key[0]), &(ms_dev_key_table[index+1].dev_key[0]), 16);
        }
    }

    ms_dev_key_table_entries--;
    /* PS Store */
    ms_access_ps_store(MS_PS_RECORD_DEV_KEYS);
    return retval;
}

/**
    \brief To add/update NetKey

    \par Description
    This routine adds/updates NetKey entry. Each NetKey is associated with a subnet.

    \param [in] netkey_index     Identifies global Index of NetKey. A 12-bit value.
    \param [in] opcode           To identify Add or Update NetKey
    \param [in] net_key          Associated NetKey to be added/updated.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_add_update_netkey
(
    /* IN */ UINT16   netkey_index,
    /* IN */ UINT32   opcode,
    /* IN */ UINT8*   net_key
)
{
    UINT32 index, add_index;
    API_RESULT retval;
    UCHAR   match_found;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] %s NetKey, with NetKeyIndex 0x%04X\n",
        ((MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE == opcode) ? "Add" : "Update"), netkey_index);
    ACCESS_debug_dump_bytes(net_key, 16);
    /* Assume no free entry available */
    add_index = MS_CONFIG_LIMITS(MS_MAX_SUBNETS);
    retval = MS_INSUFFICIENT_RESOURCES;
    match_found = MS_FALSE;

    /* Search if the NetKey Index is already present */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_SUBNETS); index ++)
    {
        if ((MS_IS_SUBNET_FREE(index)) && (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) == add_index))
        {
            /* Save first free entry */
            add_index = index;
        }
        else
        {
            /* Check if NetKeyIndex matches */
            if (netkey_index == MS_SUBNET_NETKEY_INDEX(index))
            {
                ACCESS_TRC(
                    "[CM] NetKeyIndex already present in subnet handle 0x%04X\n", index);
                match_found = MS_TRUE;
                /* Save subnet handle */
                add_index = index;
                break;
            }
        }
    }

    /* Check if Match Found */
    if (MS_TRUE == match_found)
    {
        if (MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE == opcode)
        {
            /* Compare NetKey */
            if (0 == EM_mem_cmp(ms_subnet_table[add_index].fixed.net_key[0], net_key, 16))
            {
                ACCESS_TRC(
                    "[CM] Same NetKeyIndex and NetKey found at Index 0x%08X\n", add_index);
                retval = API_SUCCESS;
            }
            else
            {
                ACCESS_ERR(
                    "[CM] NetKey does not match with NetKeyIndex at Index 0x%08X\n", add_index);
                retval = MS_KEY_INDEX_ALREADY_STORED;
            }

            /* Ensure NetKey does not get added */
            add_index = MS_CONFIG_LIMITS(MS_MAX_SUBNETS);
        }
    }
    else
    {
        if (MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE == opcode)
        {
            ACCESS_ERR(
                "[CM] NetKeyIndex not found\n");
            retval = MS_INVALID_NETKEY_INDEX;
            /* Ensure NetKey does not get added */
            add_index = MS_CONFIG_LIMITS(MS_MAX_SUBNETS);
        }
    }

    /* Check if the NetKey to be added and if there is a space */
    if (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) != add_index)
    {
        if (MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE == opcode)
        {
            /* Save as current key */
            ms_access_cm_save_netkey_at_offset
            (
                (MS_SUBNET_HANDLE)add_index,
                netkey_index,
                net_key,
                0
            );
            /* Key Refresh Phase - Normal */
            ms_access_cm_update_key_refresh_phase
            (
                (MS_SUBNET_HANDLE)add_index,
                MS_ACCESS_KEY_REFRESH_PHASE_NORMAL
            );
            /* Reset AppKey and Friend Bitarray */
            bitarray_reset_all(ms_subnet_table[add_index].config.appkey_bitarray, MS_CONFIG_LIMITS(MS_MAX_APPS));
            bitarray_reset_all(ms_subnet_table[add_index].config.friend_bitarray, MS_CONFIG_LIMITS(MS_MAX_LPNS));
            /* Increment NetKey count */
            ms_netkey_count++;
            retval = API_SUCCESS;
        }
        /* Check the current state, if Netkey Update is already in progress */
        else if (MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != MS_SUBNET_KEY_REFRESH_PHASE_STATE((MS_SUBNET_HANDLE)add_index))
        {
            retval = MS_CANNOT_UPDATE;
            ACCESS_ERR("[CM] Key Refresh in progress. NetKey Update Failed\n");
        }
        else
        {
            /* Save as updated key */
            ms_access_cm_save_netkey_at_offset
            (
                (MS_SUBNET_HANDLE)add_index,
                netkey_index,
                net_key,
                1
            );
            /* Key Refresh Phase 1 */
            ms_access_cm_update_key_refresh_phase
            (
                (MS_SUBNET_HANDLE)add_index,
                MS_ACCESS_KEY_REFRESH_PHASE_1
            );
            retval = API_SUCCESS;
        }
    }

    ACCESS_TRC("[CM] Add/Update NetKey List Returning 0x%04X\n", retval);
    return retval;
}

UINT8 ms_access_cm_update_key_refresh_phase
(
    /* IN */ MS_SUBNET_HANDLE    handle,
    /* IN */ UINT8               key_refresh_phase
)
{
    if(MS_ACCESS_KEY_REFRESH_PHASE_NORMAL == key_refresh_phase)
    {
        /* Copy New Key */
        ms_access_cm_create_keys_from_netkey
        (
            handle
        );
        /* Update State */
        MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle) = key_refresh_phase;
    }
    else
    {
        switch (MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle))
        {
        case MS_ACCESS_KEY_REFRESH_PHASE_NORMAL:
        {
            if (MS_ACCESS_KEY_REFRESH_PHASE_1 == key_refresh_phase)
            {
                UCHAR p[1] = { 0x00 };
                UCHAR k2[33];
                UINT8 nid;
                /* Generate associated NID */
                ms_stbx_k2
                (
                    ms_subnet_table[handle].fixed.net_key[1],
                    16,
                    p, sizeof(p),
                    k2
                );
                /* Save NID, Encryption and Privacy Keys */
                nid = k2[0];
                ACCESS_TRC(
                    "[CM] Generated NID: 0x%02X\n", nid);
                ms_subnet_table[handle].fixed.nid[1] = nid;
                /* Update State */
                MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle) = key_refresh_phase;
            }
        }
        break;

        case MS_ACCESS_KEY_REFRESH_PHASE_1:
        {
            if (MS_ACCESS_KEY_REFRESH_PHASE_2 == key_refresh_phase)
            {
                /* Phase 2 - Switch to the new keys */
                /* Swap netkey offset [0] and [1] */
                UINT8   net_key[16];
                UINT8   nid;
                /* Save the NID and Net Key in offset [0] */
                nid = ms_subnet_table[handle].fixed.nid[0];
                EM_mem_copy(net_key, ms_subnet_table[handle].fixed.net_key[0], 16);
                /* Move Netkey[1] to [0] */
                ms_access_cm_save_netkey_at_offset
                (
                    handle,
                    ms_subnet_table[handle].fixed.netkey_index,
                    ms_subnet_table[handle].fixed.net_key[1],
                    0
                );
                ms_access_cm_create_keys_from_netkey
                (
                    handle
                );
                /* Copy NID And Netkey to offset [1] */
                ms_subnet_table[handle].fixed.nid[1] = nid;
                EM_mem_copy(ms_subnet_table[handle].fixed.net_key[1], net_key, 16);
                /* Update State */
                ms_access_cm_refresh_all_appkeys(handle, key_refresh_phase);
                ms_access_cm_refresh_all_friend_credentials(handle, key_refresh_phase);
                MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle) = key_refresh_phase;
            }
            else if (MS_ACCESS_KEY_REFRESH_PHASE_3 == key_refresh_phase)
            {
                /* Move Netkey[1] to [0] */
                ms_access_cm_save_netkey_at_offset
                (
                    handle,
                    ms_subnet_table[handle].fixed.netkey_index,
                    ms_subnet_table[handle].fixed.net_key[1],
                    0
                );
                ms_access_cm_create_keys_from_netkey
                (
                    handle
                );
                /* Update State */
                ms_access_cm_refresh_all_appkeys(handle, key_refresh_phase);
                ms_access_cm_refresh_all_friend_credentials(handle, key_refresh_phase);
                MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle) = MS_ACCESS_KEY_REFRESH_PHASE_NORMAL;
            }
        }
        break;

        case MS_ACCESS_KEY_REFRESH_PHASE_2:
        {
            if (MS_ACCESS_KEY_REFRESH_PHASE_3 == key_refresh_phase)
            {
                /* Update State */
                ms_access_cm_refresh_all_appkeys(handle, key_refresh_phase);
                MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle) = MS_ACCESS_KEY_REFRESH_PHASE_NORMAL;
            }
        }
        break;
        }
    }

    /* PS Store */
    ms_access_ps_store(MS_PS_RECORD_SUBNETS);
    /* Return NetKey Refresh Phase */
    return MS_SUBNET_KEY_REFRESH_PHASE_STATE(handle);
}

/** Update Key Refresh Phase for all associated Appkey */
void ms_access_cm_refresh_all_appkeys
(
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT8               key_refresh_phase
)
{
    /* Search for all AppKey Indices */
    UINT32 start, next;
    start = 0;
    MS_LOOP_FOREVER()
    {
        next = bitarray_get_lowest_bit_set
               (
                   ms_subnet_table[subnet_handle].config.appkey_bitarray,
                   MS_CONFIG_LIMITS(MS_MAX_APPS),
                   start
               );

        if (0xFFFFFFFF == next)
        {
            break;
        }

        /* appkey refresh */
        ms_access_cm_appkey_refresh
        (
            (MS_APPKEY_HANDLE)next,
            key_refresh_phase
        );
        start = next + 1;
    }
}

#ifdef MS_FRIEND_SUPPORT
/* Friend Credentials refresh */
void ms_access_cm_friend_credentials_refresh
(
    /* IN */ MS_SUBNET_HANDLE    friend_subnet_handle,
    /* IN */ MS_SUBNET_HANDLE    master_subnet_handle,
    /* IN */ UINT8               key_refresh_phase
)
{
    MS_TRN_FRNDSHIP_INFO recipe;
    API_RESULT ret;
    UCHAR p[9];
    UCHAR k2[33];
    UCHAR netkey[16];
    UINT16 uaddr;
    UINT8*  enc_key;
    UINT8*  priv_key;
    MS_IGNORE_UNUSED_PARAM(key_refresh_phase);
    /* Get associated Friendship recipe */
    ret = MS_trn_get_frndship_info
          (
              ms_friend_role,
              friend_subnet_handle - MS_CONFIG_LIMITS(MS_MAX_SUBNETS),
              &recipe
          );
    /* TODO: Check return value */
    /* TODO: Lock/Unlock */
    ret = MS_access_cm_get_netkey_at_offset(master_subnet_handle, 0, netkey);

    if (API_SUCCESS == ret)
    {
        ACCESS_TRC(
            "[CM 0x%04X] Network Key\n", master_subnet_handle);
        ACCESS_debug_dump_bytes(netkey, 16);
        /* Create the P data with Friendship data */
        p[0] = 0x01;
        /* Extract Local Address */
        uaddr = ms_element_addr_table[0].uaddr;

        if (MS_FRND_ROLE_LPN == ms_friend_role)
        {
            MS_PACK_BE_2_BYTE((p + 1), &uaddr);
            MS_PACK_BE_2_BYTE((p + 3), &recipe.addr);
        }
        else
        {
            MS_PACK_BE_2_BYTE((p + 1), &recipe.addr);
            MS_PACK_BE_2_BYTE((p + 3), &uaddr);
        }

        MS_PACK_BE_2_BYTE((p + 5), &recipe.lpn_counter);
        MS_PACK_BE_2_BYTE((p + 7), &recipe.frnd_counter);
        ACCESS_TRC(
            "[CM] K2 INPUTs \n");
        ACCESS_debug_dump_bytes(p, sizeof(p));
        ms_stbx_k2
        (
            netkey,
            sizeof(netkey),
            p, sizeof(p),
            k2
        );
        enc_key = ms_subnet_table[friend_subnet_handle].fixed.encrypt_key;
        priv_key = ms_subnet_table[friend_subnet_handle].fixed.privacy_key;
        ms_subnet_table[friend_subnet_handle].fixed.nid[0] = k2[0];
        EM_mem_copy(enc_key, &k2[1], 16);
        EM_mem_copy(priv_key, &k2[1 + 16], 16);
    }
}
#endif /* MS_FRIEND_SUPPORT */

/** Update all associated Friend Credentials */
void ms_access_cm_refresh_all_friend_credentials
(
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT8               key_refresh_phase
)
{
    #ifdef MS_FRIEND_SUPPORT
    /* Search for all AppKey Indices */
    UINT32 start, next;
    start = 0;
    MS_LOOP_FOREVER()
    {
        next = bitarray_get_lowest_bit_set
               (
                   ms_subnet_table[subnet_handle].config.friend_bitarray,
                   MS_CONFIG_LIMITS(MS_MAX_SUBNETS),
                   start
               );

        if (0xFFFFFFFF == next)
        {
            break;
        }

        /* Friend Credentials refresh */
        ms_access_cm_friend_credentials_refresh
        (
            (MS_SUBNET_HANDLE)(next + MS_CONFIG_LIMITS(MS_MAX_SUBNETS)),
            subnet_handle,
            key_refresh_phase
        );
        start = next + 1;
    }
    #endif /* MS_FRIEND_SUPPORT */
}

/** Update Key Refresh Phase for Appkey */
UINT8 ms_access_cm_appkey_refresh
(
    /* IN */ MS_APPKEY_HANDLE    handle,
    /* IN */ UINT8               key_refresh_phase
)
{
    /* TODO: Design a state table */
    switch (MS_APPKEY_REFRESH_PHASE_STATE(handle))
    {
    case MS_ACCESS_KEY_REFRESH_PHASE_1:
    {
        if (MS_ACCESS_KEY_REFRESH_PHASE_2 == key_refresh_phase)
        {
            UCHAR aid;
            UINT8 key[16];
            /* Swap Keys */
            aid = ms_appkey_table[handle].aid[0];
            EM_mem_copy(key, &ms_appkey_table[handle].app_key[0][0], 16);
            ms_appkey_table[handle].aid[0] = ms_appkey_table[handle].aid[1];
            EM_mem_copy(&ms_appkey_table[handle].app_key[0][0], &ms_appkey_table[handle].app_key[1][0], 16);
            ms_appkey_table[handle].aid[1] = aid;
            EM_mem_copy(&ms_appkey_table[handle].app_key[1][0], key, 16);
            /* Update State */
            MS_APPKEY_REFRESH_PHASE_STATE(handle) = key_refresh_phase;
        }
        else if (MS_ACCESS_KEY_REFRESH_PHASE_3 == key_refresh_phase)
        {
            /* Copy [1] to [0] */
            ms_appkey_table[handle].aid[0] = ms_appkey_table[handle].aid[1];
            EM_mem_copy(&ms_appkey_table[handle].app_key[0][0], &ms_appkey_table[handle].app_key[1][0], 16);
            /* Update State */
            MS_APPKEY_REFRESH_PHASE_STATE(handle) = MS_ACCESS_KEY_REFRESH_PHASE_NORMAL;
        }
    }
    break;

    case MS_ACCESS_KEY_REFRESH_PHASE_2:
    {
        if (MS_ACCESS_KEY_REFRESH_PHASE_3 == key_refresh_phase)
        {
            /* Update State */
            MS_APPKEY_REFRESH_PHASE_STATE(handle) = MS_ACCESS_KEY_REFRESH_PHASE_NORMAL;
        }
    }
    break;
    }

    /* Return AppKey Refresh Phase */
    return MS_APPKEY_REFRESH_PHASE_STATE(handle);
}

/** This routines adds/updates netkeys */
void ms_access_cm_save_netkey_at_offset
(
    /* IN */ MS_SUBNET_HANDLE    handle,
    /* IN */ UINT16              netkey_index,
    /* IN */ UINT8*              net_key,
    /* IN */ UINT8               offset
)
{
    ACCESS_TRC(
        "[CONFIG 0x%04X] Add Netkey Network Index: 0x%04X. Offset: 0x%02X. Key\n",
        handle, netkey_index, offset);
    ACCESS_debug_dump_bytes(net_key, 16);

    if (0 == offset)
    {
        /* Save Netkey Index */
        ms_subnet_table[handle].fixed.netkey_index = netkey_index;
        MS_SUBNET_NODE_ID_STATE_SET(handle, 0x00);
    }

    /* Copy NetKey */
    EM_mem_copy (ms_subnet_table[handle].fixed.net_key[offset], net_key, 16);
}

/* This function creates keys associated with Netkey */
void ms_access_cm_create_keys_from_netkey
(
    /* IN */ MS_SUBNET_HANDLE    handle
)
{
    UCHAR p[1] = { 0x00 };
    UCHAR k2[33];
    UCHAR nkik[4] = {'n', 'k', 'i', 'k'};
    UCHAR nkbk[4] = {'n', 'k', 'b', 'k'};
    /* P = "id128" || 0x01 */
    UCHAR P[6]    = {'i', 'd', '1', '2', '8', 0x01};
    UCHAR salt[16];
    /* NID, Encryption Key and Privacy Key */
    UCHAR nid;
    UCHAR* network_id;
    UCHAR* enc_key;
    UCHAR* priv_key;
    UCHAR* b_key;
    UCHAR* i_key;
    UCHAR* net_key;
    ACCESS_TRC(
        "[CONFIG 0x%04X] Create Keys from Network Key\n", handle);
    net_key = ms_subnet_table[handle].fixed.net_key[0];
    ACCESS_debug_dump_bytes(net_key, 16);
    /* Generate Network ID */
    network_id = ms_subnet_table[handle].fixed.network_id;
    ms_stbx_k3
    (
        net_key,
        16,
        network_id
    );
    ACCESS_TRC(
        "[CM] Generated Network ID \n");
    ACCESS_debug_dump_bytes(network_id, 8);
    ms_stbx_k2
    (
        net_key,
        16,
        p, sizeof(p),
        k2
    );
    /* Save NID, Encryption and Privacy Keys */
    nid = k2[0];
    enc_key = ms_subnet_table[handle].fixed.encrypt_key;
    priv_key = ms_subnet_table[handle].fixed.privacy_key;
    EM_mem_copy(enc_key, &k2[1], 16);
    EM_mem_copy(priv_key, &k2[1 + 16], 16);
    /* Beacon Key Generation */
    b_key = ms_subnet_table[handle].fixed.beacon_key;
    /* Calculate beacon salt */
    EM_mem_set(salt, 0x0, sizeof(salt));
    ms_stbx_s1(nkbk, sizeof(nkbk), salt);
    /* Calculate the BeaconKey */
    ms_stbx_k1
    (
        net_key,
        16,
        salt,
        P,
        sizeof(P),
        b_key
    );
    /* Identity Key Generation */
    i_key = ms_subnet_table[handle].fixed.identity_key;
    /* Calculate identity salt */
    EM_mem_set(salt, 0x0, sizeof(salt));
    ms_stbx_s1(nkik, sizeof(nkik), salt);
    /* Calculate the IdentityKey */
    ms_stbx_k1
    (
        net_key,
        16,
        salt,
        P,
        sizeof(P),
        i_key
    );
    ACCESS_TRC(
        "[CM] Generated NID: 0x%02X\n", nid);
    ms_subnet_table[handle].fixed.nid[0] = nid;
    ACCESS_TRC(
        "[CM] Generated Encryption Key\n");
    ACCESS_debug_dump_bytes(enc_key, 16);
    ACCESS_TRC(
        "[CM] Generated Privacy Key\n");
    ACCESS_debug_dump_bytes(priv_key, 16);
    ACCESS_TRC(
        "[CM] Generated Beacon Key\n");
    ACCESS_debug_dump_bytes(b_key, 16);
    ACCESS_TRC(
        "[CM] Generated Identity Key\n");
    ACCESS_debug_dump_bytes(i_key, 16);
//    /* PS Store */
//    ms_access_ps_store(MS_PS_RECORD_SUBNETS);
    return;
}

/**
    \brief To add Security Credential of a LPN or the Friend.

    \par Description
    This routine adds NID, privacy and encryption keys associated with a friendship.

    \param [in] subnet_handle    Identifies associated subnet.
    \param [in] friend_index     Friend Index.
    \param [in] lpn_addr         Address of the LPN.
    \param [in] friend_addr      Address of the Friend.
    \param [in] lpn_counter      Number of Friend Request messages the LPN has sent.
    \param [in] friend_counter   Number of Friend Offer messages the Friend has sent.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_add_friend_sec_credential
(
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT16              friend_index,
    /* IN */ MS_NET_ADDR         lpn_addr,
    /* IN */ MS_NET_ADDR         friend_addr,
    /* IN */ UINT16              lpn_counter,
    /* IN */ UINT16              friend_counter
)
{
    UCHAR p[9];
    UCHAR k2[33];
    UCHAR netkey[16];
    MS_SUBNET_HANDLE handle;
    /* NID, Encryption Key and Privacy Key */
    UCHAR nid;
    UCHAR* enc_key;
    UCHAR* priv_key;
    API_RESULT retval;
    ACCESS_TRC(
        "[CM] Add Friend Security Credentials\n");
    ACCESS_TRC(
        "[CM] Subnet Handle: 0x%04X, Frnd Index: 0x%04X\n", subnet_handle, friend_index);
    ACCESS_TRC(
        "[CM] LPN Addr: 0x%04X, Frnd Addr: 0x%04X\n", lpn_addr, friend_addr);
    ACCESS_TRC(
        "[CM] LPN Counter: 0x%04X, Frnd Counter: 0x%04X\n", lpn_counter, friend_counter);
    /* TODO: Check friend_index */
    /* Get the network key of the subnet */
    retval = MS_access_cm_get_netkey_at_offset(subnet_handle, 0, netkey);

    if (API_SUCCESS == retval)
    {
        blebrr_scan_pl(FALSE);        // BY HQ
        ACCESS_TRC(
            "[CM 0x%04X] Network Key\n", subnet_handle);
        ACCESS_debug_dump_bytes(netkey, 16);
        /* While adding will not check, if the entry is already in use */
        handle = MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + friend_index;
        /* Create the P data with Friendship data */
        p[0] = 0x01;
        MS_PACK_BE_2_BYTE((p + 1), &lpn_addr);
        MS_PACK_BE_2_BYTE((p + 3), &friend_addr);
        MS_PACK_BE_2_BYTE((p + 5), &lpn_counter);
        MS_PACK_BE_2_BYTE((p + 7), &friend_counter);
        ACCESS_TRC(
            "[CM] K2 INPUTs \n");
        ACCESS_debug_dump_bytes(p, sizeof(p));
        ms_stbx_k2
        (
            netkey,
            sizeof(netkey),
            p, sizeof(p),
            k2
        );
        ACCESS_TRC(
            "[CM] K2 OUTPUTs \n");
        ACCESS_debug_dump_bytes(k2, sizeof(k2));
        /* Save NID, Encryption and Privacy Keys */
        nid = k2[0];
        enc_key = ms_subnet_table[handle].fixed.encrypt_key;
        priv_key = ms_subnet_table[handle].fixed.privacy_key;
        ms_subnet_table[handle].fixed.nid[0] = nid;
        EM_mem_copy(enc_key, &k2[1], 16);
        EM_mem_copy(priv_key, &k2[1 + 16], 16);
        ACCESS_TRC(
            "[CM] Generated Friend NID: 0x%02X\n", nid);
        ACCESS_TRC(
            "[CM] Generated Friend Encryption Key\n");
        ACCESS_debug_dump_bytes(enc_key, 16);
        ACCESS_TRC(
            "[CM] Generated Friend Privacy Key\n");
        ACCESS_debug_dump_bytes(priv_key, 16);
        /* Mark the entry as allocated */
        ACCESS_TRC(
            "[CM] Subnet Handle 0x%04X for Friend Index 0x%04X\n",
            handle, friend_index);
        /**
            Netkey indices are valid from 0x000 to 0xFFF.
            Using Index one greater than the max value.
        */
        /* TODO: Ensure there is no collision with possible valid values */
        MS_SUBNET_NETKEY_INDEX(handle) = 0x1000;
        /* Mark corresponding friend bit in netkey table */
        bitarray_set_bit(ms_subnet_table[subnet_handle].config.friend_bitarray, friend_index);
        /* Save master subnet handle with friend */
        ms_subnet_table[handle].config.friend_bitarray[0] = subnet_handle;
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_SUBNETS);
    }

    return retval;
}


/**
    \brief To delete the Security Credential of a LPN or the Friend.

    \par Description
    This routine deletes NID, privacy and encryption keys associated with a friendship.

    \param [in] subnet_handle    Identifies associated subnet.
    \param [in] friend_index     Friend Index.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_delete_friend_sec_credential
(
    /* IN */ MS_SUBNET_HANDLE    subnet_handle,
    /* IN */ UINT16              friend_index

)
{
    MS_SUBNET_HANDLE handle;
    API_RESULT       retval;
    /* While adding will not check, if the entry is already in use */
    handle = MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + friend_index;
    /* Delete the association of the friend element with the subnet */
    /* Reset corresponding appkey bit in netkey table */
    bitarray_reset_bit(ms_subnet_table[subnet_handle].config.friend_bitarray, friend_index);
    /* Remove master subnet handle saved with friend */
    ms_subnet_table[handle].config.friend_bitarray[0] = MS_INVALID_SUBNET_HANDLE;
    /* Delete the netkey at the friend index of subnet table */
    retval = MS_access_cm_delete_netkey((MS_SUBNET_HANDLE)(MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + friend_index));
    return retval;
}


/**
    \brief To find a Subnet associated with the NetKey

    \par Description
    This routine finds a Subnet based on the NetKey entry. Each NetKey is associated with a subnet.

    \param [in]  netkey_index     Identifies global Index of NetKey, corresponding Subnet to be returned.
    \param [out] subnet_handle    Memory location to be filled with Subnet Handle, if search is successful.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_find_subnet
(
    /* IN */  UINT16               netkey_index,
    /* OUT */ MS_SUBNET_HANDLE*    subnet_handle
)
{
    UINT32 index;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Find Subnet associated with NetKeyIndex 0x%04X\n", netkey_index);
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;

    /* Search if the NetKey Index is already present */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_SUBNETS); index ++)
    {
        if (!MS_IS_SUBNET_FREE(index))
        {
            /* Check if NetKeyIndex matches */
            if (netkey_index == MS_SUBNET_NETKEY_INDEX(index))
            {
                ACCESS_TRC(
                    "[CM] NetKeyIndex Found in subnet handle 0x%04X\n", index);
                /* Fill Subnet Handle */
                *subnet_handle = (MS_SUBNET_HANDLE)index;
                retval = API_SUCCESS;
                break;
            }
        }
    }

    ACCESS_TRC(
        "[CM] Find Subnet Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To find the Master Subnet associated with the friend security credential, identified by Friend Subnet Handle.

    \par Description
    This routine finds the Master Subnet based on the friend security credential, identified by Friend Subnet Handle.

    \param [out] friend_subnet_handle    Idetifies the Friend Subnet Handle, corresponding to Friend Subnet Handle.
    \param [in]  master_subnet_handle    Memory location to be filled with Master Subnet Handle, if search is successful.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_find_master_subnet
(
    /* IN */  MS_SUBNET_HANDLE     friend_subnet_handle,
    /* OUT */ MS_SUBNET_HANDLE*    master_subnet_handle
)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    /* TODO: Parameter Validation */
    *master_subnet_handle = ms_subnet_table[friend_subnet_handle].config.friend_bitarray[0];
    ACCESS_TRC(
        "[CM] Master Subnet 0x%08X is associated with Friend Subnet 0x%08X\n",
        *master_subnet_handle, friend_subnet_handle);
    ACCESS_TRC(
        "[CM] Find Master Subnet Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To delete NetKey

    \par Description
    This routine deletes a NetKey entry. Each NetKey is associated with a subnet.

    \param [in] subnet_handle     Handle of the Subnet for which NetKey to be deleted.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_delete_netkey
(
    /* IN */ MS_SUBNET_HANDLE   subnet_handle
)
{
    API_RESULT retval;
    /* UINT32     index; */
    /* TODO: Parameter Validation */
    /**
        TODO: Not informing LPN/Friends local implementation that associated
        NetKey is removed.
    */
    ACCESS_TRC("[CM] Delete NetKey of Subnet 0x%04X. Current NetKey Count:%d\n", subnet_handle, ms_netkey_count);
    /* Assume no match found */
    retval = MS_CANNOT_REMOVE;

    /**
        If this is the last key, can not be deleted.
        Based on Test Case: MESH/NODE/CFG/NKL/BV-04-C
    */
    if (1 >= ms_netkey_count)
    {
        ACCESS_ERR("[CM] Can not delete last NetKey of Subnet 0x%04X\n", subnet_handle);
        retval = MS_CANNOT_REMOVE;
    }
    /* Check if the Subnet Handle is not already free */
    else if (!MS_IS_SUBNET_FREE(subnet_handle))
    {
        ACCESS_TRC(
            "[CM] Deleting subnet handle 0x%04X\n", subnet_handle);
        MS_INIT_NETKEY_ENTRY(subnet_handle);
        retval = API_SUCCESS;

        /* Decrement NetKey Count */
        if (0 != ms_netkey_count)
        {
            ms_netkey_count--;
        }

        /* Delete associated AppKeys */
        /* Search if the AppKey associated with this NetKey */
        #if 0

        for (index = 0; index < MS_MAX_APPS; index ++)
        {
            if (!(MS_IS_APPKEY_FREE(index)) && (subnet_handle == ms_appkey_table[index].subnet_handle))
            {
                /* Delete appkey */
                ms_delete_appkey(index);
            }
        }

        #else
        {
            /* Search for all AppKey Indices */
            UINT32 start, next;
            start = 0;
            MS_LOOP_FOREVER()
            {
                next = bitarray_get_lowest_bit_set
                       (
                           ms_subnet_table[subnet_handle].config.appkey_bitarray,
                           MS_CONFIG_LIMITS(MS_MAX_APPS),
                           start
                       );

                if (0xFFFFFFFF == next)
                {
                    break;
                }

                /* Delete appkey */
                ms_delete_appkey(next);
                start = next + 1;
            }
        }
        /* Not resetting all associated AppKey bitarray. That will be done when someone uses it again. */
        #endif /* 0 */
    }

    ACCESS_TRC(
        "[CM] Delete NetKey Returning 0x%04X\n", retval);
    /* TODO: Write to persistent storage */
    return retval;
}

/**
    \brief To get NetKey

    \par Description
    This routine fetches a NetKey entry. Each NetKey is associated with a subnet.

    \param [in]  subnet_handle     Handle of the Subnet for which NetKey to be deleted.
    \param [out] net_key           Netkey associated with the Subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_netkey_at_offset
(
    /* IN */  MS_SUBNET_HANDLE    subnet_handle,
    /* IN */  UINT8               offset,
    /* OUT */ UINT8*              net_key
)
{
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get NetKey of Subnet 0x%04X\n", subnet_handle);
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;

    /* Check if the Subnet Handle is not already free */
    if (!MS_IS_SUBNET_FREE(subnet_handle))
    {
        ACCESS_TRC(
            "[CM] Getting subnet handle 0x%04X\n", subnet_handle);
        /* Copy Net Key */
        EM_mem_copy(net_key, ms_subnet_table[subnet_handle].fixed.net_key[offset], 16);
        retval = API_SUCCESS;
    }

    ACCESS_TRC(
        "[CM] Get NetKey Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get list of all known NetKeys

    \par Description
    This routine returns a list of known NetKey Indices.

    \param [inout] netkey_count   Caller fills with maximum number of NetKey Indices
                                  that can be stored in 'netkey_index_list'.
                                  This function will update the value with how many NetKey
                                  Indices has been filled. If the number of available
                                  NetKey Indices is more than that can be returned,
                                  maximum possible Indices will be filled and
                                  an appropriate error values will inform the caller,
                                  there are more NetKey Indices (as an information).
    \param [out] netkey_index_list Memory to be filled with the available NetKey Indices.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_netkey_index_list
(
    /* INOUT */ UINT16* netkey_count,
    /* OUT */   UINT16* netkey_index_list
)
{
    UINT32 index;
    UINT16 actual_count;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get NetKey List. Receive max count 0x%04X\n", (*netkey_count));
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;
    actual_count = 0;

    /* Search if the NetKey Index is already present */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_SUBNETS); index ++)
    {
        /* If found a NetKey */
        if (!MS_IS_SUBNET_FREE(index))
        {
            /* Check if there is free space */
            if (actual_count < (*netkey_count))
            {
                /* Copy NetKeyIndex */
                netkey_index_list[actual_count] = MS_SUBNET_NETKEY_INDEX(index);
                ACCESS_TRC(
                    "[CM] NetKeyList[%02X] 0x%04X\n", actual_count, netkey_index_list[actual_count]);
                /* Increase count */
                actual_count++;
            }
            else
            {
                ACCESS_ERR(
                    "[CM] Receive NetKey List Overflow. Returning ...\n");
                /* TODO: Return appropriate status, more NetKey Indices available */
                break;
            }
        }
    }

    /* TODO: Need a better mechanism to set return value in this function */
    if (0 != actual_count)
    {
        retval = API_SUCCESS;
    }

    /* Update with Actual Count */
    *netkey_count = actual_count;
    ACCESS_TRC(
        "[CM] Get NetKey List Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get Node Identity

    \par Description
    This routine gets Node Identity State of a node

    \param [in]  subnet_handle    Handle identifying the subnet.
    \param [out] id_state         Memory location where Node Identity state to be filled.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_node_identity
(
    /* IN */  MS_SUBNET_HANDLE   subnet_handle,
    /* OUT */ UINT8*             id_state
)
{
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get Node Identity State of Subnet 0x%04X\n", subnet_handle);
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;
    #ifndef MS_PROXY_SUPPORT
    /**
        If the Proxy Server is not supported, the Node Identity state value shall be 0x02.
    */
    *id_state = (UCHAR)0x02;
    #else
    /**
        First set Node Identity State return value with 0x00,
        as expected by the test specification MESH/NODE/CFG/NID/BV-03-C
    */
    *id_state = (UCHAR)0x00;

    /* Check if the NetKey Index is not already free */
    if (!MS_IS_SUBNET_FREE(subnet_handle))
    {
        UCHAR      proxy;
        /* Fill Node Identity State */
        *id_state = MS_SUBNET_NODE_ID_STATE(subnet_handle);
        MS_access_cm_get_features_field(&proxy, MS_FEATURE_PROXY);

        /* If Proxy is disabled, set Node Identity to 0x02 */
        /** TODO: Check if the Return Value of 0x02 is correct? */
        if (MS_TRUE != proxy)
        {
            *id_state = (UCHAR)0x02;
        }

        ACCESS_TRC(
            "[CM 0x%04X] Node Identity State is 0x%04X.\n", subnet_handle, (*id_state));
        retval = API_SUCCESS;
    }

    #endif /* MS_PROXY_SUPPORT */
    ACCESS_TRC(
        "[CM] Get Node Identity State Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To set Node Identity

    \par Description
    This routine sets Node Identity State of a node

    \param [in] subnet_handle    Handle identifying the subnet.
    \param [in, out] id_state    Node Identity state to be set.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_node_identity
(
    /* IN */    MS_SUBNET_HANDLE   subnet_handle,
    /* INOUT */ UINT8*               id_state
)
{
    API_RESULT retval;
    /* TODO: Parameter Validation */
    /* TODO: MS_access_cm_get/set_node_identity should be a single function */
    ACCESS_TRC(
        "[CM] Set Node Identity State of Subnet 0x%04X. Value 0x%02X\n", subnet_handle, id_state);
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;

    /* Check if the NetKey Index is not already free */
    if (!MS_IS_SUBNET_FREE(subnet_handle))
    {
        #ifdef MS_PROXY_SUPPORT
        UCHAR proxy;
        MS_access_cm_get_features_field(&proxy, MS_FEATURE_PROXY);

        /* If Proxy is currently disabled, set 0x02 as the node identity */
        /** TODO: Check if the Return Value of 0x02 is correct? */
        if (MS_TRUE != proxy)
        {
            *id_state = (UCHAR)0x02;
        }

        /* Save Node Identity State */
        MS_SUBNET_NODE_ID_STATE(subnet_handle) = *id_state;
        ACCESS_TRC(
            "[CM 0x%04X] Node Identity State is 0x%04X.\n",
            subnet_handle, MS_SUBNET_NODE_ID_STATE(subnet_handle));
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_SUBNETS);
        #endif /* MS_PROXY_SUPPORT */
        retval = API_SUCCESS;
    }

    ACCESS_TRC(
        "[CM] Set Node Identity State Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get Key Refresh Phase

    \par Description
    This routine gets Key Refresh Phase State of a node

    \param [in]  subnet_handle       Handle identifying the subnet.
    \param [out] key_refresh_state   Memory location where Key Refresh Phase state to be filled.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_key_refresh_phase
(
    /* IN */  MS_SUBNET_HANDLE   subnet_handle,
    /* OUT */ UINT8*             key_refresh_state
)
{
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get Key Refresh Phase State of Subnet 0x%04X\n", subnet_handle);
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;

    /* Check if the NetKey Index is not already free */
    if (!MS_IS_SUBNET_FREE(subnet_handle))
    {
        /* Fill Key Refresh Phase State */
        *key_refresh_state = MS_SUBNET_KEY_REFRESH_PHASE_STATE(subnet_handle);
        ACCESS_TRC(
            "[CM 0x%04X] Key Refresh Phase State is 0x%04X.\n", subnet_handle, (*key_refresh_state));
        retval = API_SUCCESS;
    }

    ACCESS_TRC(
        "[CM] Get Key Refresh Phase State Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To set Key Refresh Phase

    \par Description
    This routine sets Key Refresh Phase State of a node

    \param [in] subnet_handle    Handle identifying the subnet.
    \param [in, out] key_refresh_state         Key Refresh Phase state to be set.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_key_refresh_phase
(
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ UINT8*             key_refresh_state
)
{
    API_RESULT retval;
    /* TODO: Parameter Validation */
    /* TODO: MS_access_cm_get/set_key_refresh_phase should be a single function */
    ACCESS_TRC(
        "[CM] Set Key Refresh Phase State of Subnet 0x%04X. Value 0x%02X\n",
        subnet_handle, *key_refresh_state);
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;

    /* Check if the NetKey Index is not already free */
    if (!MS_IS_SUBNET_FREE(subnet_handle))
    {
        UINT8  state;
        /**
            Key Refresh Transition Table

            Old State | Transition | New State
            ----------+------------+----------
            0x00      | 0x03       | 0x00
            ----------+------------+----------
            0x01      | 0x02       | 0x02
            ----------+------------+----------
            0x01      | 0x03       | 0x00
            ----------+------------+----------
            0x02      | 0x02       | 0x02
            ----------+------------+----------
            0x02      | 0x03       | 0x00
        */
        state = MS_SUBNET_KEY_REFRESH_PHASE_STATE(subnet_handle);
        retval = API_SUCCESS;

        /* Save Key Refresh Phase State */
        if (MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != state)
        {
            state = *key_refresh_state;
            state = ms_access_cm_update_key_refresh_phase
                    (
                        subnet_handle,
                        state
                    );
        }
        else if (MS_ACCESS_KEY_REFRESH_PHASE_3 != *key_refresh_state)
        {
            retval = API_FAILURE;
        }

        /* Return Current State */
        *key_refresh_state = state;
        ACCESS_TRC(
            "[CM 0x%04X] Key Refresh Phase State is 0x%04X.\n",
            subnet_handle, MS_SUBNET_KEY_REFRESH_PHASE_STATE(subnet_handle));
    }

    ACCESS_TRC(
        "[CM] Set Key Refresh Phase State Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To search for NID

    \par Description
    This routine searches for matching NID in subnet table.

    \param [in] nid    NID to be searched in all known subnets for match.
    \param [inout] subnet_handle Same NID can match with multiple subnets.
                                 Caller will fill this value to indicate from which
                                 subnet handle the search to be started. This function
                                 will return the subnet handle, where the match is found
                                 (in case of match). Caller while searching for the same
                                 NID, in the subsequent call can pass the subnet_handle
                                 received in the previous match for the NID.
                                 For the very first call when searching for a NID,
                                 the caller need to use Invalid Subnet Handle
                                 \ref MS_INVALID_SUBNET_HANDLE.
    \param [out] privacy_key    Privacy Key associated with the subnet.
    \param [out] encrypt_key    Encryption Key associated with the subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_lookup_nid
(
    /* IN */    UINT8               nid,
    /* INOUT */ MS_SUBNET_HANDLE*   subnet_handle,
    /* OUT */   UINT8*              privacy_key,
    /* OUT */   UINT8*              encrypt_key
)
{
    API_RESULT retval;
    UINT32     index;
    UINT32     master_subnet_handle;
    retval = ACCESS_NO_MATCH;

    /* Check for NULL Pointer */
    if ((NULL == subnet_handle) || (NULL == privacy_key) || (NULL == encrypt_key))
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in NID Lookup. Returning 0x%04X\n", retval);
    }
    else
    {
        ACCESS_TRC(
            "[CM] Search NID 0x%02X in subnet table from handle 0x%04X\n",
            nid, (*subnet_handle));
        /* Check if caller wants to start the search from specific subnet handle */
        index = (* subnet_handle);

        if (MS_INVALID_SUBNET_HANDLE == index)
        {
            index = 0;
        }

        /* Search for matching master credentials */
        while (index < MS_CONFIG_LIMITS(MS_MAX_SUBNETS))
        {
            if (nid == ms_subnet_table[index].fixed.nid[0])
            {
                ACCESS_TRC(
                    "[CM] NID match found in subnet handle 0x%04X\n", index);
                /* Notify if we are LPN and NID looked up is of Master Subnet */
                retval = (MS_FRND_ROLE_LPN == ms_friend_role)? ACCESS_MASTER_NID_ON_LPN: API_SUCCESS;
                *subnet_handle = (MS_SUBNET_HANDLE)index;
                EM_mem_copy (privacy_key, ms_subnet_table[index].fixed.privacy_key, 16);
                EM_mem_copy (encrypt_key, ms_subnet_table[index].fixed.encrypt_key, 16);
                return retval;
            }
            /* Check if Key Refresh is in progress */
            else if (MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != MS_SUBNET_KEY_REFRESH_PHASE_STATE(index))
            {
                if (nid == ms_subnet_table[index].fixed.nid[1])
                {
                    /* Generate Privacy and Encryption Key */
                    UCHAR p[1] = { 0x00 };
                    UCHAR k2[33];
                    UINT8* net_key;
                    net_key = ms_subnet_table[index].fixed.net_key[1];
                    ms_stbx_k2
                    (
                        net_key,
                        16,
                        p, sizeof(p),
                        k2
                    );
                    /* Notify if we are LPN and NID looked up is of Master Subnet */
                    retval = (MS_FRND_ROLE_LPN == ms_friend_role)? ACCESS_MASTER_NID_ON_LPN: API_SUCCESS;
                    *subnet_handle = (MS_SUBNET_HANDLE)index;
                    EM_mem_copy(encrypt_key, &k2[1], 16);
                    EM_mem_copy(privacy_key, &k2[1 + 16], 16);
                    return retval;
                }
            }

            index++;
        }

        /* Search for matching friend credentials */
        while (index < (UINT32)(MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)))
        {
            if (nid == ms_subnet_table[index].fixed.nid[0])
            {
                ACCESS_TRC(
                    "[CM] NID match found in subnet handle 0x%04X\n", index);
                retval = API_SUCCESS;
                *subnet_handle = (MS_SUBNET_HANDLE)index;
                EM_mem_copy (privacy_key, ms_subnet_table[index].fixed.privacy_key, 16);
                EM_mem_copy (encrypt_key, ms_subnet_table[index].fixed.encrypt_key, 16);
                break;
            }

            #ifdef MS_FRIEND_SUPPORT
            /* Check if Key Refresh is in progress in associated subnet */
            else
            {
                master_subnet_handle = ms_subnet_table[index].config.friend_bitarray[0];

                /* Check if associated with a friend, lookup for NID */
                if ((MS_INVALID_SUBNET_HANDLE != master_subnet_handle) &&
                        (MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != MS_SUBNET_KEY_REFRESH_PHASE_STATE(master_subnet_handle)))
                {
                    /* Calculate Friendship Credential and check for match */
                    MS_TRN_FRNDSHIP_INFO recipe;
                    API_RESULT ret;
                    UCHAR p[9];
                    UCHAR k2[33];
                    UINT8* netkey;
                    UINT16 uaddr;
                    /* Get associated Friendship recipe */
                    ret = MS_trn_get_frndship_info
                          (
                              ms_friend_role,
                              (UINT16)(index - MS_CONFIG_LIMITS(MS_MAX_SUBNETS)),
                              &recipe
                          );
                    /* TODO: Check Return value */
                    netkey = ms_subnet_table[master_subnet_handle].fixed.net_key[1];
                    /* if (API_SUCCESS == ret) */
                    {
                        ACCESS_TRC(
                            "[CM 0x%04X] Network Key\n", master_subnet_handle);
                        ACCESS_debug_dump_bytes(netkey, 16);
                        /* Create the P data with Friendship data */
                        p[0] = 0x01;
                        /* Extract Local Address */
                        uaddr = ms_element_addr_table[0].uaddr;

                        if (MS_FRND_ROLE_LPN == ms_friend_role)
                        {
                            MS_PACK_BE_2_BYTE((p + 1), &uaddr);
                            MS_PACK_BE_2_BYTE((p + 3), &recipe.addr);
                        }
                        else
                        {
                            MS_PACK_BE_2_BYTE((p + 1), &recipe.addr);
                            MS_PACK_BE_2_BYTE((p + 3), &uaddr);
                        }

                        MS_PACK_BE_2_BYTE((p + 5), &recipe.lpn_counter);
                        MS_PACK_BE_2_BYTE((p + 7), &recipe.frnd_counter);
                        ACCESS_TRC(
                            "[CM] K2 INPUTs \n");
                        ACCESS_debug_dump_bytes(p, sizeof(p));
                        ms_stbx_k2
                        (
                            netkey,
                            16,
                            p, sizeof(p),
                            k2
                        );
                        ACCESS_TRC(
                            "[CM] K2 OUTPUTs \n");
                        ACCESS_debug_dump_bytes(k2, sizeof(k2));

                        /* Save NID, Encryption and Privacy Keys */
                        if (nid == k2[0])
                        {
                            EM_mem_copy(encrypt_key, &k2[1], 16);
                            EM_mem_copy(privacy_key, &k2[1 + 16], 16);
                            ACCESS_TRC(
                                "[CM] Generated Friend NID: 0x%02X\n", k2[0]);
                            ACCESS_TRC(
                                "[CM] Generated Friend Encryption Key\n");
                            ACCESS_debug_dump_bytes(encrypt_key, 16);
                            ACCESS_TRC(
                                "[CM] Generated Friend Privacy Key\n");
                            ACCESS_debug_dump_bytes(privacy_key, 16);
                            ACCESS_TRC(
                                "[CM] NID match found in subnet handle 0x%04X\n", index);
                            retval = API_SUCCESS;
                            *subnet_handle = (MS_SUBNET_HANDLE)index;
                            break;
                        }
                    }
                }
            }

            #endif /* MS_FRIEND_SUPPORT */
            index++;
        }
    }

    return retval;
}

/**
    \brief To search for Network ID

    \par Description
    This routine searches for matching Network ID in subnet table.

    \param [in] network_id       Network ID to be searched in all known subnets for match.
    \param [inout] subnet_handle Same NID can match with multiple subnets.
                                 Caller will fill this value to indicate from which
                                 subnet handle the search to be started. This function
                                 will return the subnet handle, where the match is found
                                 (in case of match). Caller while searching for the same
                                 NID, in the subsequent call can pass the subnet_handle
                                 received in the previous match for the NID.
                                 For the very first call when searching for a NID,
                                 the caller need to use Invalid Subnet Handle
                                 \ref MS_INVALID_SUBNET_HANDLE.
    \param [out] beacon_key      Beacon Key associated with the subnet.
    \param [out] is_new_key      Flag to indicate if the network ID is associated with
                                 the new Network Key being updated.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_lookup_network_id
(
    /* IN */    UINT8*              network_id,
    /* INOUT */ MS_SUBNET_HANDLE*   subnet_handle,
    /* OUT */   UINT8*              beacon_key,
    /* OUT */   UINT8*              is_new_key
)
{
    API_RESULT retval;
    UINT32     index;
    retval = ACCESS_NO_MATCH;
    *is_new_key = MS_FALSE;

    /* Check for NULL Pointer */
    if ((NULL == network_id) || (NULL == subnet_handle) || (NULL == beacon_key))
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in Network ID Lookup. Returning 0x%04X\n", retval);
    }
    else
    {
        ACCESS_TRC(
            "[CM] Search Network ID in subnet table from handle 0x%04X\n",
            (*subnet_handle));
        ACCESS_debug_dump_bytes(network_id, 8);
        /* Check if caller wants to start the search from specific subnet handle */
        index = (* subnet_handle);

        if (MS_INVALID_SUBNET_HANDLE == index)
        {
            index = 0;
        }

        /* Search */
        while(index < MS_CONFIG_LIMITS(MS_MAX_SUBNETS))
        {
            if (0 == EM_mem_cmp(network_id, ms_subnet_table[index].fixed.network_id, 8))
            {
                ACCESS_TRC("[CM] Network ID match found in subnet handle 0x%04X\n", index);
                retval = API_SUCCESS;
                *subnet_handle = (MS_SUBNET_HANDLE)index;
                EM_mem_copy (beacon_key, ms_subnet_table[index].fixed.beacon_key, 16);

                /* TODO: Devise a logic to calculate is_new_key from the matching index and current state */
                if (MS_ACCESS_KEY_REFRESH_PHASE_2 <= ms_subnet_table[index].fixed.key_refresh_phase)
                {
                    *is_new_key = MS_TRUE;
                }

                break;
            }
            /* Check if Key Refresh is in progress */
            else if (MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != ms_subnet_table[index].fixed.key_refresh_phase)
            {
                UINT8* net_key;
                UINT8   network_id_temp[8];
                UINT8   b_key[16];
                UINT8   nkbk[4] = {'n', 'k', 'b', 'k'};
                UINT8   salt[16];
                UINT8   P[6]    = {'i', 'd', '1', '2', '8', 0x01};
                net_key = ms_subnet_table[index].fixed.net_key[1];
                ms_stbx_k3
                (
                    net_key,
                    16,
                    network_id_temp
                );
                ACCESS_TRC("[CM] Generated Network ID\n");
                ACCESS_debug_dump_bytes(network_id_temp, sizeof(network_id_temp));

                /* Check for match */
                if (0 == EM_mem_cmp(network_id, network_id_temp, 8))
                {
                    ACCESS_TRC("[CM] Network ID match found in subnet handle 0x%04X, with new key\n", index);

                    /* TODO: Devise a logic to calculate is_new_key from the matching index and current state */
                    if (MS_ACCESS_KEY_REFRESH_PHASE_2 > ms_subnet_table[index].fixed.key_refresh_phase)
                    {
                        *is_new_key = MS_TRUE;
                    }

                    /* TODO: Have a separate function for Beacon Key generation */
                    /* Calculate beacon salt */
                    EM_mem_set(salt, 0x0, sizeof(salt));
                    ms_stbx_s1(nkbk, sizeof(nkbk), salt);
                    /* Calculate the BeaconKey */
                    ms_stbx_k1
                    (
                        net_key,
                        16,
                        salt,
                        P,
                        sizeof(P),
                        b_key
                    );
                    retval = API_SUCCESS;
                    *subnet_handle = (MS_SUBNET_HANDLE)index;
                    EM_mem_copy (beacon_key, b_key, 16);
                    break;
                }
            }

            index++;
        }
    }

    return retval;
}


/**
    \brief To search for AID

    \par Description
    This routine searches for matching NID in subnet table.

    \param [in] aid    AID to be searched in all known AppKeys for match.
    \param [inout] appkey_handle Same AID can match with multiple AppKeys.
                                 Caller will fill this value to indicate from which
                                 AppKey handle the search to be started. This function
                                 will return the AppKey handle, where the match is found
                                 (in case of match). Caller while searching for the same
                                 AID, in the subsequent call can pass the appkey_handle
                                 received in the previous match for the AID.
                                 For the very first call when searching for a AID,
                                 the caller need to use Invalid Subnet Handle
                                 \ref MS_INVALID_APPKEY_HANDLE.
    \param [out] app_key         AppKey associated with the AID.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_lookup_aid
(
    /* IN */    UINT8               aid,
    /* INOUT */ MS_APPKEY_HANDLE*   appkey_handle,
    /* OUT */   UINT8*              app_key
)
{
    API_RESULT retval;
    UINT32     index;
    retval = ACCESS_NO_MATCH;

    /* Check for NULL Pointer */
    if ((NULL == appkey_handle) || (NULL == app_key))
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in AID Lookup. Returning 0x%04X\n", retval);
    }
    else
    {
        ACCESS_TRC(
            "[CM] Search AID 0x%02X in AppKey table from handle 0x%04X\n",
            aid, (*appkey_handle));
        /* Check if caller wants to start the search from specific AppKey handle */
        index = (* appkey_handle);

        if (MS_INVALID_APPKEY_HANDLE == index)
        {
            index = 0;
        }

        /* Search */
        for (; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index++)
        {
            if (aid == ms_appkey_table[index].aid[0])
            {
                ACCESS_TRC(
                    "[CM] AID match found in AppKey handle[0] 0x%04X\n", index);
                retval = API_SUCCESS;
                *appkey_handle = (MS_APPKEY_HANDLE)index;
                EM_mem_copy(app_key, ms_appkey_table[index].app_key[0], 16);
                break;
            }
            /* Check if Key Refresh is in progress */
            else if (MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != MS_APPKEY_REFRESH_PHASE_STATE(index))
            {
                if (aid == ms_appkey_table[index].aid[1])
                {
                    ACCESS_TRC(
                        "[CM] AID match found in AppKey handle[1] 0x%04X\n", index);
                    retval = API_SUCCESS;
                    *appkey_handle = (MS_APPKEY_HANDLE)index;
                    EM_mem_copy(app_key, ms_appkey_table[index].app_key[1], 16);
                    break;
                }
            }
        }
    }

    return retval;
}

/**
    \brief To get AppKey

    \par Description
    This routine gets AppKey along with AID entry.

    \param [in]  appkey_handle    AppKey Handle.
    \param [out] app_key          Pointer to AppKey to be returned.
    \param [out] aid              Pointer to AID to be returned.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_app_key
(
    /* IN */  MS_APPKEY_HANDLE   appkey_handle,
    /* OUT */ UINT8**            app_key,
    /* OUT */ UINT8*             aid
)
{
    API_RESULT retval;
    retval = ACCESS_INVALID_HANDLE;

    /* Check for NULL Pointer */
    if ((NULL == app_key) || (NULL == aid))
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
        ACCESS_ERR(
            "[CM] NULL parameter in get AppKey. Returning 0x%04X\n", retval);
        printf(
            "[CM] NULL parameter in get AppKey. Returning 0x%04X\n", retval);
    }
    else
    {
        ACCESS_TRC(
            "[CM] Get AppKey for handle 0x%04X\n", appkey_handle);

        /* Check if AppKey is valid */
        /* TODO: Range Check */
        if (MS_INVALID_KEY_INDEX != MS_APPKEY_INDEX(appkey_handle))
        {
            *app_key = &(ms_appkey_table[appkey_handle].app_key[0][0]);
            *aid = ms_appkey_table[appkey_handle].aid[0];
            retval = API_SUCCESS;
        }
    }

    return retval;
}

/**
    \brief Set Provisioning Data

    \par Description
    This routine configures the provisioning data with Access Layer.

    \param prov_data
           Provisioning data received during provisioning procedure.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_prov_data
(
    /* IN */ PROV_DATA_S*     prov_data
)
{
    API_RESULT retval;
    UINT8 iv_update_flag;
    /* Initialize Subnet Table */
    ms_access_cm_init_subnet_table();
    /* Initialize AppKey Table */
    ms_access_cm_init_appkey_table();
    /* Initialize Device Key Table */
    /* ms_access_cm_init_dev_key_table(); */
    /* Initialize Element Address Table */
    /* EM_mem_set(ms_element_addr_table, 0, sizeof(ms_element_addr_table)); */
    /* Initialize Non-Virtual Address Table */
    EM_mem_set(ms_non_virtual_addr_table, 0, (sizeof(MS_NON_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS)));
    /* Initialize Virtual Address Table */
    EM_mem_set(ms_virtual_addr_table, 0, (sizeof(MS_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS)));
    /* Initialize Network/Relay Tx States */
    EM_mem_set(ms_tx_state, 0, sizeof(ms_tx_state));
    /* TODO: Find a better solution */
    ltrn_init_replay_cache();
    /* Save Primary Unicast Address */
    MS_access_cm_set_primary_unicast_address(prov_data->uaddr);

    /* Save IV Index and Update Flag. Not checking return value */
    if((prov_data->flags >> 1) & 0x01)
    {
        iv_update_flag = 0x21;
        MS_net_start_iv_update_timer(iv_update_flag,MS_FALSE);
    }
    else
    {
        iv_update_flag = 0x00;
    }

    MS_access_cm_set_iv_index(prov_data->ivindex, iv_update_flag);
    /* Save NetKeyIndex and NetKey */
    retval = MS_access_cm_add_update_netkey(prov_data->keyid, MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE, prov_data->netkey);

    /* Check for the Key Refresh flags */
    if ((0x01 == (prov_data->flags & 0x01)))
    {
        UINT8            krstate;
        MS_SUBNET_HANDLE subnet;
        /* Update the NetKey as Network is in Key Refresh Phase 3 */
        retval = MS_access_cm_add_update_netkey
                 (
                     prov_data->keyid,
                     MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE,
                     prov_data->netkey
                 );
        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     prov_data->keyid,
                     &subnet
                 );
        /* Set Key Refresh State to Phase 2 */
        krstate = MS_ACCESS_KEY_REFRESH_PHASE_2;
        /* Set the Key Refresh Phase */
        MS_access_cm_set_key_refresh_phase(subnet, &krstate);
    }

    /* Store information to Persistent Storage */
    ms_access_ps_store(MS_PS_ACCESS_ALL_RECORDS);
    return retval;
}

/**
    \brief Set Provisioning Data

    \par Description
    This routine configures the provisioning data with Access Layer.

    \param prov_data
           Provisioning data received during provisioning procedure.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_prov_data_provsioner
(
    /* IN */ PROV_DATA_S*     prov_data
)
{
    API_RESULT retval;
    printf(
        "[CM] MS_access_cm_set_prov_data 0x%02X\n",ms_iv_index.iv_update_state);
    /* Save Primary Unicast Address */
    MS_access_cm_set_primary_unicast_address(prov_data->uaddr);
    /* Save IV Index and Update Flag. Not checking return value */
    MS_access_cm_set_iv_index(prov_data->ivindex, ms_iv_index.iv_update_state);
    /* Save NetKeyIndex and NetKey */
    retval = MS_access_cm_add_update_netkey(prov_data->keyid, MS_ACCESS_CONFIG_NETKEY_ADD_OPCODE, prov_data->netkey);

    /* Check for the Key Refresh flags */
    if ((0x01 == (prov_data->flags & 0x01)))
    {
        UINT8            krstate;
        MS_SUBNET_HANDLE subnet;
        /* Update the NetKey as Network is in Key Refresh Phase 3 */
        retval = MS_access_cm_add_update_netkey
                 (
                     prov_data->keyid,
                     MS_ACCESS_CONFIG_NETKEY_UPDATE_OPCODE,
                     prov_data->netkey
                 );
        /* Get associated Subnet Handle */
        retval = MS_access_cm_find_subnet
                 (
                     prov_data->keyid,
                     &subnet
                 );
        /* Set Key Refresh State to Phase 2 */
        krstate = MS_ACCESS_KEY_REFRESH_PHASE_2;
        /* Set the Key Refresh Phase */
        MS_access_cm_set_key_refresh_phase(subnet, &krstate);
    }

    /* Store information to Persistent Storage */
    ms_access_ps_store(MS_PS_ACCESS_ALL_RECORDS);
    return retval;
}

/**
    \brief To get NID associated with a subnet

    \par Description
    This routine fetches the NID associated with a subnet.

    \param [in]  handle         Handle identifying the subnet.
    \param [out] nid            NID associated with the subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_subnet_nid
(
    /* IN */  MS_SUBNET_HANDLE   handle,
    /* OUT */ UINT8*             nid
)
{
    API_RESULT retval;

    /* Parameter Validation */
    if (((MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)) <= handle) || MS_IS_SUBNET_FREE(handle))
    {
        retval = ACCESS_INVALID_HANDLE;
    }
    else if (NULL == nid)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }
    else
    {
        /* Copy NID */
        *nid = ms_subnet_table[handle].fixed.nid[0];
        retval = API_SUCCESS;
    }

    return retval;
}

/**
    \brief To get Privacy Key associated with a subnet

    \par Description
    This routine fetches the Privacy Key associated with a subnet.

    \param [in]  handle         Handle identifying the subnet.
    \param [out] privacy_key    Privacy Key associated with the subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_subnet_privacy_key
(
    /* IN */  MS_SUBNET_HANDLE   handle,
    /* OUT */ UINT8*             privacy_key
)
{
    API_RESULT retval;

    /* Parameter Validation */
    if ((MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)) <= handle)
    {
        retval = ACCESS_INVALID_HANDLE;
    }
    else if (NULL == privacy_key)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }
    else
    {
        /* Copy Privacy Key */
        EM_mem_copy(privacy_key, ms_subnet_table[handle].fixed.privacy_key, 16);
        retval = API_SUCCESS;
    }

    return retval;
}

/**
    \brief To get Network ID associated with a subnet

    \par Description
    This routine fetches the Network ID associated with a subnet.

    \param [in]  handle         Handle identifying the subnet.
    \param [out] network_id     Network ID associated with the subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_subnet_network_id
(
    /* IN */  MS_SUBNET_HANDLE   handle,
    /* OUT */ UINT8*             network_id
)
{
    API_RESULT retval;

    /* Parameter Validation */
    if ((MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)) <= handle)
    {
        retval = ACCESS_INVALID_HANDLE;
    }
    else if (NULL == network_id)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }
    else if (!MS_IS_SUBNET_FREE(handle))
    {
        /* Copy Network ID */
        EM_mem_copy(network_id, ms_subnet_table[handle].fixed.network_id, 8);
        retval = API_SUCCESS;
    }
    else
    {
        retval = MS_INVALID_NETKEY_INDEX;
    }

    return retval;
}


/**
    \brief To get Beacon Key associated with a subnet

    \par Description
    This routine fetches the Beacon Key associated with a subnet.

    \param [in]  handle         Handle identifying the subnet.
    \param [out] beacon_key     Beacon Key associated with the subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_subnet_beacon_key
(
    /* IN */  MS_SUBNET_HANDLE  handle,
    /* OUT */ UINT8*              beacon_key
)
{
    API_RESULT retval;

    /* Parameter Validation */
    if ((MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)) <= handle)
    {
        retval = ACCESS_INVALID_HANDLE;
    }
    else if (NULL == beacon_key)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }
    else
    {
        /* Copy Beacon Key */
        EM_mem_copy(beacon_key, ms_subnet_table[handle].fixed.beacon_key, 16);
        retval = API_SUCCESS;
    }

    return retval;
}

/**
    \brief To get Identity Key associated with a subnet

    \par Description
    This routine fetches the Identity Key associated with a subnet.

    \param [in]  handle         Handle identifying the subnet.
    \param [out] identity_key   Identity Key associated with the subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_subnet_identity_key
(
    /* IN */  MS_SUBNET_HANDLE  handle,
    /* OUT */ UINT8*              identity_key
)
{
    API_RESULT retval;

    /* Parameter Validation */
    if ((MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)) <= handle)
    {
        retval = ACCESS_INVALID_HANDLE;
    }
    else if (NULL == identity_key)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }
    else if (!MS_IS_SUBNET_FREE(handle))
    {
        /* Copy Identity Key */
        EM_mem_copy(identity_key, ms_subnet_table[handle].fixed.identity_key, 16);
        retval = API_SUCCESS;
    }
    else
    {
        retval = MS_INVALID_NETKEY_INDEX;
    }

    return retval;
}

/**
    \brief To get Encryption Key associated with a subnet

    \par Description
    This routine fetches the Encryption Key associated with a subnet.

    \param [in]  handle         Handle identifying the subnet.
    \param [out] encrypt_key    Encryption Key associated with the subnet.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_subnet_encryption_key
(
    /* IN */  MS_SUBNET_HANDLE   handle,
    /* OUT */ UINT8*             encrypt_key
)
{
    API_RESULT retval;

    /* Parameter Validation */
    if ((MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)) <= handle)
    {
        retval = ACCESS_INVALID_HANDLE;
    }
    else if (NULL == encrypt_key)
    {
        retval = ACCESS_NULL_PARAMETER_NOT_ALLOWED;
    }
    else
    {
        /* Copy Encrypt Key */
        EM_mem_copy(encrypt_key, ms_subnet_table[handle].fixed.encrypt_key, 16);
        retval = API_SUCCESS;
    }

    return retval;
}

/**
    \brief To reset a node

    \par Description
    This routine resets a node (other than a Provisioner) and removes it from the network.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_reset(UINT8 role)
{
    API_RESULT retval;
    UINT16 index;
    NET_SEQ_NUMBER_STATE seq_number_state;

    if(role == PROV_ROLE_DEVICE)
    {
        /* Selectively clear ms_access_model_list[] */
        for (index = 0; index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); index++)
        {
            /* Clear only Publish information from Fixed part */
            EM_mem_set
            (
                &ms_access_model_list[index].fixed.publish,
                0,
                sizeof(MS_ACCESS_PUBLISH_META_INFO)
            );
            /* Clear all information fro Config part */
            EM_mem_set
            (
                ms_access_model_list[index].config.appkey_bitarray,
                0,
                (sizeof(UINT32) * (BITARRAY_NUM_BLOCKS(MS_CONFIG_LIMITS(MS_MAX_APPS))))
            );
            EM_mem_set
            (
                ms_access_model_list[index].config.subscription_bitarray,
                0,
                (sizeof(UINT32) * (BITARRAY_NUM_BLOCKS(MS_MAX_ADDRS)))
            );
        }

        /* Clear Subnet Information */
        ms_netkey_count = 0;
        EM_mem_set
        (
            &ms_subnet_table[0],
            0,
            (sizeof(MS_NETKEY_ENTRY) * (MS_CONFIG_LIMITS(MS_MAX_SUBNETS) + MS_CONFIG_LIMITS(MS_MAX_LPNS)))
        );
        ms_access_cm_init_subnet_table();
        /* Clear AppKey Information */
        EM_mem_set
        (
            &ms_appkey_table[0],
            0,
            (sizeof(MS_APPKEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_APPS))
        );
        /* Clear Element Address Table */
        {
            UINT8         element_count;
            /* Save Element Count of the first entry */
            element_count = ms_element_addr_table[0].element_count;
            EM_mem_set
            (
                &ms_element_addr_table[0],
                0,
                (sizeof(MS_ELEMENT_ADDR_ENTRY) * (1 + MS_CONFIG_LIMITS(MS_MAX_LPNS)))
            );
            /* Restore Element Count of the first entry */
            ms_element_addr_table[0].element_count = element_count;
        }
        /* Clear Virtual Address Information */
        EM_mem_set
        (
            &ms_virtual_addr_table[0],
            0,
            (sizeof(MS_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS))
        );
        /* Clear Non-Virtual Address Information */
        EM_mem_set
        (
            &ms_non_virtual_addr_table[0],
            0,
            (sizeof(MS_NON_VIRTUAL_ADDR_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS))
        );
        /* Clear Tx States */
        EM_mem_set
        (
            &ms_tx_state[0],
            0,
            (sizeof(MS_CONFIG_TRANSMIT) * 2)
        );
    }

    /* Clear DevKey Information */
    ms_dev_key_table_entries = 0;
    ms_dev_key_table_pointer = ms_start_unicast_addr;
    EM_mem_set
    (
        &ms_dev_key_table[0],
        0,
        (sizeof(MS_DEV_KEY_ENTRY) * MS_CONFIG_LIMITS(MS_MAX_DEV_KEYS))
    );
    MS_ACCESS_INIT_PERIODIC_STEP_TIMER();
    /* Clear IVI information */
    ms_iv_index.iv_index = 0; /*  0xFFFFFFFF; */
    ms_iv_index.iv_update_state = MS_FALSE;
    ms_iv_index.iv_expire_time = 0;
    MS_net_stop_snb_timer(0);
    MS_net_stop_iv_update_timer();
    MS_net_stop_key_refresh_timer();
    MS_prov_stop_interleave_timer();
    MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NETID, BRR_BCON_ACTIVE);
    MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NODEID, BRR_BCON_ACTIVE);
    MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_ACTIVE);
    MS_brr_bcast_end(BRR_BCON_TYPE_UNPROV_DEVICE, BRR_BCON_PASSIVE);
    /* Clear Network Sequence Number State */
    MS_net_get_seq_num_state(&seq_number_state);

    if(seq_number_state.seq_num != seq_number_state.block_seq_num_max)
    {
        seq_number_state.block_seq_num_max += MS_CONFIG_LIMITS(MS_NET_SEQ_NUMBER_BLOCK_SIZE);
        seq_number_state.seq_num = seq_number_state.block_seq_num_max;
    }

    MS_net_set_seq_num_state(&seq_number_state);
    ms_provisioner_addr = MS_NET_ADDR_UNASSIGNED;
    retval = API_SUCCESS;
    return retval;
}

/**
    \brief To set Network/Relay Transmit state

    \par Description
    This routine sets Network/Relay Transmit state.

    \param [in] tx_state_type   Transmit State Type (Network or Relay)
    \param [in] tx_state        Composite state (3-bits of Tx Count and 5-bits of Tx Interval Steps)

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_transmit_state
(
    /* IN */ UINT8    tx_state_type,
    /* IN */ UINT8    tx_state
)
{
    API_RESULT retval;
    /* Lock */
    /* TODO: Parameter Validation */
    retval = API_SUCCESS;
    ACCESS_TRC(
        "[CM] Set %s Transmit State with composite value 0x%02X\n",
        ((MS_NETWORK_TX_STATE == tx_state_type) ? "Network" : "Relay"), tx_state);
    /* Transmit Count - 3 bits LSBs */
    MS_TX_COUNT_STATE(tx_state_type) = (UCHAR)(tx_state & 0x07);
    /* Transmit Interval Steps - 5 bits MSBs */
    MS_TX_STEPS_STATE(tx_state_type) = (UCHAR)(tx_state >> 3);
    /* Unlock */
    /* PS Store */
    ms_access_ps_store(MS_PS_RECORD_TX_STATES_FEATURES);
    return retval;
}

/**
    \brief To get Network/Relay Transmit state

    \par Description
    This routine gets Network/Relay Transmit state.

    \param [in]  tx_state_type  Transmit State Type (Network or Relay)
    \param [out] tx_state       Memory location to fill Composite state
                                (3-bits of Tx Count and 5-bits of Tx Interval Steps)

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_transmit_state
(
    /* IN */  UINT8    tx_state_type,
    /* OUT */ UINT8*   tx_state
)
{
    API_RESULT retval;
    /* Lock */
    /* TODO: Parameter Validation */
    retval = API_SUCCESS;
    ACCESS_TRC(
        "[CM] Get %s Transmit State\n",
        ((MS_NETWORK_TX_STATE == tx_state_type) ? "Network" : "Relay"));
    /* Transmit Count - 3 bits LSBs */
    *tx_state = MS_TX_COUNT_STATE(tx_state_type);
    /* Transmit Interval Steps - 5 bits MSBs */
    *tx_state |= ((MS_TX_STEPS_STATE(tx_state_type)) << 3);
    /* Unlock */
    return retval;
}

/**
    \brief To add AppKey

    \par Description
    This routine adds AppKey entry. Each AppKey is associated with a subnet.

    \param [in] subnet_handle    Handle of the Subnet for which AppKey to be added.
    \param [in] appkey_index     Identifies global Index of AppKey. A 12-bit value.
    \param [in] app_key          Associated AppKey to be added.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_add_appkey
(
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ UINT16             appkey_index,
    /* IN */ UINT8*             app_key
)
{
    UINT32 index, free_index;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Add AppKey, with AppKeyIndex 0x%04X, for Subnet Handle 0x%04X\n",
        appkey_index, subnet_handle);
    ACCESS_debug_dump_bytes(app_key, 16);
    /* Assume no free entry available */
    retval = ACCESS_NO_RESOURCE;
    free_index = MS_CONFIG_LIMITS(MS_MAX_APPS);

    /* Search if the AppKey Index is already present */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index ++)
    {
        if (!(MS_IS_APPKEY_FREE(index)))
        {
            if (appkey_index == MS_APPKEY_INDEX(index))
            {
                ACCESS_TRC("[CM] Found AppKey at Index 0x%08X\n", index);

                /* Check if it is associated with same network key */
                if (subnet_handle != ms_appkey_table[index].subnet_handle)
                {
                    ACCESS_ERR("[CM] Invalid NetKey.\n");
                    retval = MS_INVALID_NETKEY_INDEX;
                }
                else if (0 == EM_mem_cmp(ms_appkey_table[index].app_key[0], app_key, 16))
                {
                    ACCESS_TRC("[CM] Identical AppKey. Returning success\n");
                    retval = API_SUCCESS;
                }
                else
                {
                    retval = MS_KEY_INDEX_ALREADY_STORED;
                }

                free_index = MS_CONFIG_LIMITS(MS_MAX_APPS);
                break;
            }
        }
        else if (MS_CONFIG_LIMITS(MS_MAX_APPS) == free_index)
        {
            ACCESS_TRC("[CM] Got free Index for AppKey add:0x%08X\n", index);
            free_index = index;
        }
    }

    if (MS_CONFIG_LIMITS(MS_MAX_APPS) != free_index)
    {
        ACCESS_TRC("[CM] Adding AppKey at Index 0x%08X\n", free_index);
        /* Save AppKeyIndex */
        MS_APPKEY_INDEX(free_index) = appkey_index;
        /* Save AppKey */
        EM_mem_copy(&ms_appkey_table[free_index].app_key[0], app_key, 16);
        /* Save Subnet Handle */
        ms_appkey_table[free_index].subnet_handle = subnet_handle;
        /* Mark corresponding appkey bit in netkey table */
        bitarray_set_bit(ms_subnet_table[subnet_handle].config.appkey_bitarray, free_index);
        /* Calculate AID and Save */
        ms_stbx_k4(app_key, 16, &ms_appkey_table[free_index].aid[0]);
        /* Save Key Refresh Phase */
        MS_APPKEY_REFRESH_PHASE_STATE(free_index) = MS_ACCESS_KEY_REFRESH_PHASE_NORMAL;
        ACCESS_TRC("[CM] Generated AID 0x%02X\n", ms_appkey_table[free_index].aid[0]);
        retval = API_SUCCESS;
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_APP_KEYS);
    }

    ACCESS_TRC(
        "[CM] Add AppKey List Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To update/delete AppKey

    \par Description
    This routine updates/deletes AppKey entry. Each AppKey is associated with a subnet.

    \param [in] subnet_handle    Handle of the Subnet for which AppKey to be updated/deleted.
    \param [in] appkey_index     Identifies global Index of AppKey. A 12-bit value.
    \param [in] opcode           To identify Delete or Update NetKey
    \param [in] app_key          Associated AppKey to be updated.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_update_delete_appkey
(
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ UINT16             appkey_index,
    /* IN */ UINT32             opcode,
    /* IN */ UINT8*             app_key
)
{
    UINT32 index;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] %s AppKey, with AppKeyIndex 0x%04X, for Subnet Handle 0x%04X\n",
        ((MS_ACCESS_CONFIG_APPKEY_UPDATE_OPCODE == opcode) ? "Update" : "Delete"),
        appkey_index, subnet_handle);
    /* Assume no match found */
    retval = MS_INVALID_APPKEY_INDEX;

    /* Search if the AppKey Index is already present */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index ++)
    {
        if (!(MS_IS_APPKEY_FREE(index)) && (appkey_index == MS_APPKEY_INDEX(index)))
        {
            if (subnet_handle != ms_appkey_table[index].subnet_handle)
            {
                ACCESS_ERR("[CM] Invalid Binding. AppKey and NetKey pair is not correct\n");
                retval = MS_INVALID_BINDING;
            }
            else
            {
                ACCESS_TRC("[CM] Found AppKey at Index 0x%08X\n", index);
                retval = API_SUCCESS;
            }

            break;
        }
    }

    /* Check if match found */
    if ((MS_CONFIG_LIMITS(MS_MAX_APPS) != index) && (API_SUCCESS == retval))
    {
        if (MS_ACCESS_CONFIG_APPKEY_DELETE_OPCODE == opcode)
        {
            ACCESS_TRC("[CM] Deleting AppKey at Index 0x%08X\n", index);
            ms_delete_appkey(index);
            /* Reset corresponding appkey bit in netkey table */
            bitarray_reset_bit(ms_subnet_table[subnet_handle].config.appkey_bitarray, index);
        }

        #if 0
        /* Check the current state, if AppKey/Netkey Update is in progress */
        else if ((MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != MS_APPKEY_REFRESH_PHASE_STATE(index)) ||
                 (MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != MS_SUBNET_KEY_REFRESH_PHASE_STATE(subnet_handle)))
        #else
        else if ((MS_ACCESS_KEY_REFRESH_PHASE_NORMAL != MS_APPKEY_REFRESH_PHASE_STATE(index)) ||
                 (MS_ACCESS_KEY_REFRESH_PHASE_1 != MS_SUBNET_KEY_REFRESH_PHASE_STATE(subnet_handle)))
        #endif /* 0 */
        {
            retval = MS_CANNOT_UPDATE;
            ACCESS_ERR("[CM] Key Refresh in progress. AppKey Update Failed\n");
        }
        else
        {
            ACCESS_TRC("[CM] Updating AppKey at Index 0x%08X\n", index);
            ACCESS_debug_dump_bytes (app_key, 16);
            /* Save AppKey */
            EM_mem_copy(&ms_appkey_table[index].app_key[1], app_key, 16);
            /* Calculate AID and Save */
            ms_stbx_k4(app_key, 16, &ms_appkey_table[index].aid[1]);
            ACCESS_TRC(
                "[CM] Generated AID 0x%02X\n", ms_appkey_table[index].aid[1]);
            /* Save Key Refresh Phase */
            MS_APPKEY_REFRESH_PHASE_STATE(index) = MS_ACCESS_KEY_REFRESH_PHASE_1;
        }

        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_APP_KEYS);
        ms_access_ps_store(MS_PS_RECORD_MODELS);
    }

    ACCESS_TRC(
        "[CM] Update/Delete AppKey List Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get AppKey Handle for a given AppKey Index

    \par Description
    This routine gets AppKey Handle for a given AppKey Index. Each AppKey is associated with a subnet.

    \param [in]  subnet_handle    Handle of the Subnet for which AppKey to be updated.
    \param [in]  appkey_index     Identifies global Index of AppKey. A 12-bit value.
    \param [in]  app_key          Associated AppKey to be matched.
    \param [out] appkey_handle    Memory to hold the associated AppKey Handle.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_appkey_handle
(
    /* IN */  MS_SUBNET_HANDLE    subnet_handle,
    /* IN */  UINT16              appkey_index,
    /* IN */  UINT8*              app_key,
    /* OUT */ MS_APPKEY_HANDLE*   appkey_handle
)
{
    UINT32 index;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get AppKey Handle, with AppKeyIndex 0x%04X, for Subnet Handle 0x%04X\n",
        appkey_index, subnet_handle);
    ACCESS_debug_dump_bytes(app_key, 16);
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;

    /* Search if the AppKey Index is already present */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index ++)
    {
        if (!(MS_IS_APPKEY_FREE(index)) && ((appkey_index == MS_APPKEY_INDEX(index)) && (subnet_handle == ms_appkey_table[index].subnet_handle)))
        {
            /* TODO: Check if the key refresh state is normal. Else return error */
            ACCESS_TRC(
                "[CM] Found AppKey at Index 0x%08X\n", index);

            /* Check for AppKey Match */
            if (0 == EM_mem_cmp(app_key, ms_appkey_table[index].app_key[0], 16))
            {
                ACCESS_TRC(
                    "[CM] AppKey also matched\n");
                /* Save AppKey Handle */
                *appkey_handle = (MS_APPKEY_HANDLE)index;
                retval = API_SUCCESS;
            }
            else
            {
                retval = MS_KEY_INDEX_ALREADY_STORED;
                ACCESS_ERR(
                    "[CM] AppKey does not match for Index 0x%08X. Returning 0x%04X\n",
                    index, retval);
            }

            break;
        }
    }

    ACCESS_TRC(
        "[CM] Get AppKey Handle Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get list of all known AppKeys

    \par Description
    This routine returns a list of known AppKey Indices associated with a subnet.

    \param [in] subnet_handle     Handle of the Subnet for which AppKey to be returned.
    \param [inout] appkey_count   Caller fills with maximum number of AppKey Indices
                                  that can be stored in 'apptkey_index_list'.
                                  This function will update the value with how many AppKey
                                  Indices has been filled. If the number of available
                                  AppKey Indices is more than that can be returned,
                                  maximum possible Indices will be filled and
                                  an appropriate error values will inform the caller,
                                  there are more NetKey Indices (as an information).
    \param [out] appkey_index_list Memory to be filled with the available AppKey Indices.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_appkey_index_list
(
    /* IN */    MS_SUBNET_HANDLE     subnet_handle,
    /* INOUT */ UINT16*              appkey_count,
    /* OUT */   UINT16*              appkey_index_list
)
{
    UINT32 index;
    UINT16 actual_count;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get AppKey List for Subnet 0x%04X. Receive max count 0x%04X\n", subnet_handle, (*appkey_count));
    /* Assume no match found */
    retval = ACCESS_NO_MATCH;
    actual_count = 0;

    /* Check if the Subnet Handle is not already free */
    if (!MS_IS_SUBNET_FREE(subnet_handle))
    {
        /* Search if the AppKey Index is already present */
        for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index ++)
        {
            /* If found a AppKey and the Subnet Handle matches */
            if ((!MS_IS_APPKEY_FREE(index)) && (subnet_handle == ms_appkey_table[index].subnet_handle))
            {
                /* Check if there is free space */
                if (actual_count < (*appkey_count))
                {
                    /* Copy AppKeyIndex */
                    appkey_index_list[actual_count] = MS_APPKEY_INDEX(index);
                    ACCESS_TRC(
                        "[CM] AppKeyList[%02X] 0x%04X\n", actual_count, appkey_index_list[actual_count]);
                    /* Increase count */
                    actual_count++;
                }
                else
                {
                    ACCESS_ERR(
                        "[CM] Receive AppKey List Overflow. Returning ...\n");
                    /* TODO: Return appropriate status, more AppKey Indices available */
                    break;
                }
            }
        }
    }

    /* TODO: Need a better mechanism to set return value in this function */
    if (0 != actual_count)
    {
        retval = API_SUCCESS;
    }

    /* Update with Actual Count */
    *appkey_count = actual_count;
    ACCESS_TRC(
        "[CM] Get AppKey List Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To bind a model with an AppKey

    \par Description
    This routine binds a model with an AppKey.

    \param [in] model_handle     Model handle identifying the model.
    \param [in] appkey_index     Identifies global Index of AppKey. A 12-bit value.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_bind_model_app
(
    /* IN */ MS_ACCESS_MODEL_HANDLE    model_handle,
    /* IN */ UINT16                    appkey_index
)
{
    UINT32 index;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Bind Model 0x%04X and AppKeyIndex 0x%04X\n",
        model_handle, appkey_index);
    /* Assume no match for model_handle */
    retval = ACCESS_NO_MATCH;

    /* Check if the model is not configuration server */
    if ((MS_ACCESS_MODEL_TYPE_SIG == ms_access_model_list[model_handle].fixed.model_id.type) &&
            (MS_MODEL_ID_CONFIG_SERVER == ms_access_model_list[model_handle].fixed.model_id.id))
    {
        retval = MS_CANNOT_BIND;
    }
    /* Check model is valid */
    else if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Assume no match for AppKeyIndex */
        retval = MS_INVALID_APPKEY_INDEX;

        /* Search if the AppKey Index is already present */
        for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index ++)
        {
            if ((!(MS_IS_APPKEY_FREE(index))) && (appkey_index == MS_APPKEY_INDEX(index)))
            {
                ACCESS_TRC(
                    "[CM] Found AppKey at Index 0x%08X\n", index);
                /* Set AppKey index in bitarray */
                bitarray_set_bit(ms_access_model_list[model_handle].config.appkey_bitarray, index);
                retval = API_SUCCESS;
                /* PS Store */
                ms_access_ps_store(MS_PS_RECORD_MODELS);
                break;
            }
        }
    }

    ACCESS_TRC(
        "[CM] Bind Model and AppKey Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To unbind a model with an AppKey

    \par Description
    This routine unbinds a model with an AppKey.

    \param [in] model_handle     Model handle identifying the model.
    \param [in] appkey_index     Identifies global Index of AppKey. A 12-bit value.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_unbind_model_app
(
    /* IN */ MS_ACCESS_MODEL_HANDLE    model_handle,
    /* IN */ UINT16                    appkey_index
)
{
    UINT32 index;
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Unbind Model 0x%04X and AppKeyIndex 0x%04X\n",
        model_handle, appkey_index);
    /* Assume no match for model_handle/AppKeyIndex */
    retval = ACCESS_NO_MATCH;

    /* Check model is valid */
    if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Search if the AppKey Index is already present */
        for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index ++)
        {
            /* Assume no match for AppKeyIndex */
            retval = MS_INVALID_APPKEY_INDEX;

            if ((!(MS_IS_APPKEY_FREE(index))) && (appkey_index == MS_APPKEY_INDEX(index)))
            {
                ACCESS_TRC(
                    "[CM] Found AppKey at Index 0x%08X\n", index);
                /* Reset AppKey index in bitarray */
                bitarray_reset_bit(ms_access_model_list[model_handle].config.appkey_bitarray, index);

                /** Check for matching AppKeyIndex in Publication Information */
                if ((0x01 == ms_access_model_list[model_handle].fixed.publish.valid) &&
                        (ms_access_model_list[model_handle].fixed.publish.appkey_handle == (UINT16)index))
                {
                    ACCESS_TRC("[CM] Marking Publish as Invalid for Model Index:0x%08X\n", model_handle);
                    /** Mark Publication Information as invalid */
                    ms_access_model_list[model_handle].fixed.publish.valid = (UCHAR)0x00;
                }

                retval = API_SUCCESS;
                /* PS Store */
                ms_access_ps_store(MS_PS_RECORD_MODELS);
                break;
            }
        }
    }

    ACCESS_TRC(
        "[CM] Unbind Model and AppKey Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get list of all AppKeys associated with a model

    \par Description
    This routine returns a list of known AppKey Indices associated with a model.

    \param [in] model_handle      Handle of the Model for which AppKey to be returned.
    \param [inout] appkey_count   Caller fills with maximum number of AppKey Indices
                                  that can be stored in 'apptkey_index_list'.
                                  This function will update the value with how many AppKey
                                  Indices has been filled. If the number of available
                                  AppKey Indices is more than that can be returned,
                                  maximum possible Indices will be filled and
                                  an appropriate error values will inform the caller,
                                  there are more NetKey Indices (as an information).
    \param [out] appkey_index_list Memory to be filled with the available AppKey Indices.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_model_app_list
(
    /* IN */    MS_ACCESS_MODEL_HANDLE    model_handle,
    /* INOUT */ UINT16*                   appkey_count,
    /* OUT */   UINT16*                   appkey_index_list
)
{
    API_RESULT retval;
    UINT16 actual_count;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get AppKey list for Model 0x%04X\n", model_handle);
    /* Assume no AppKey for model_handle */
    retval = ACCESS_NO_MATCH;
    actual_count = 0;

    /* Check model is valid */
    if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Search for all AppKey Indices */
        UINT32 start, next;
        start = 0;
        MS_LOOP_FOREVER()
        {
            next = bitarray_get_lowest_bit_set
                   (
                       ms_access_model_list[model_handle].config.appkey_bitarray,
                       MS_CONFIG_LIMITS(MS_MAX_APPS),
                       start
                   );

            if (0xFFFFFFFF == next)
            {
                break;
            }

            /* Fill AppKeyIndex List */
            /* Check if there is free space */
            if (actual_count < (*appkey_count))
            {
                /* Copy AppKeyIndex */
                appkey_index_list[actual_count] = MS_APPKEY_INDEX(next);
                ACCESS_TRC(
                    "[CM] AppKeyList[%02X] 0x%04X\n", actual_count, appkey_index_list[actual_count]);
                /* Increase count */
                actual_count++;
                start = next + 1;
            }
            else
            {
                ACCESS_ERR(
                    "[CM] Receive AppKey List Overflow. Returning ...\n");
                /* TODO: Return appropriate status, more AppKey Indices available */
                break;
            }
        }
    }

    /* TODO: Need a better mechanism to set return value in this function */
    if (0 != actual_count)
    {
        retval = API_SUCCESS;
    }

    /* Update with Actual Count */
    *appkey_count = actual_count;
    ACCESS_TRC(
        "[CM] Get AppKey list for Model Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To set Publication information associated with a model

    \par Description
    This routine sets Publication information associated with a model.

    \param [in]    model_handle      Handle of the Model for which Publication info to be set.
    \param [inout] publish_info      Publication Information to be set.
                                     If Label UUID is used, on success corresponding
                                     Virtual Address will be filled and returned.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_model_publication
(
    /* IN */    MS_ACCESS_MODEL_HANDLE    model_handle,
    /* INOUT */ MS_ACCESS_PUBLISH_INFO*   publish_info
)
{
    UINT32                   index;
    API_RESULT               retval;
    MS_NET_ADDR              addr;
    MS_ACCESS_ADDRESS_HANDLE addr_handle;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Set Publication Information for Model 0x%04X\n", model_handle);
    /* TODO: Print Publication Information */
    /* Assume no Publication Information for model_handle */
    retval = ACCESS_NO_MATCH;

    /* Search if the AppKey Index is already present */
    if (MS_TRUE == publish_info->remote)
    {
        for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_APPS); index++)
        {
            if ((!(MS_IS_APPKEY_FREE(index))) && (publish_info->appkey_index == MS_APPKEY_INDEX(index)))
            {
                ACCESS_TRC(
                    "[CM] Found AppKey at Index 0x%08X.\n", index);
                break;
            }
        }

        /* Check if KeyIndex match found */
        if (MS_CONFIG_LIMITS(MS_MAX_APPS) == index)
        {
            retval = MS_INVALID_APPKEY_INDEX;
            ACCESS_ERR("[CM] Failed to find AppKey Index 0x%04X. Returning 0x%04X\n",
                       publish_info->appkey_index, retval);
            return retval;
        }
    }
    else
    {
        /* If called locally, the appkey_index is the AppKey Handle */
        index = publish_info->appkey_index;
    }

    /* Check model is valid */
    if (((MS_TRUE != publish_info->remote) || (MS_CONFIG_LIMITS(MS_MAX_APPS) != index)) && (1 == ms_access_model_list[model_handle].fixed.valid))
    {
        /* Check if Virtual Address */
        if (1 == publish_info->addr.use_label)
        {
            /* Add entry in virtual address table */
            if (API_SUCCESS != ms_search_and_add_virtual_address(publish_info->addr.label, MS_ADDR_OP_TYPE_PUBLISH, &addr, &addr_handle))
            {
                ACCESS_ERR(
                    "[CM] Virtual Address Add failed. Returning ...\n");
                return MS_INSUFFICIENT_RESOURCES;
            }
            else
            {
                /* Return back generated Virtual Address */
                publish_info->addr.addr = addr;
            }
        }
        else
        {
            /* Add entry in non-virtual address table */
            if (API_SUCCESS != ms_search_and_add_address(model_handle,&publish_info->addr.addr, MS_ADDR_OP_TYPE_PUBLISH, &addr_handle))
            {
                ACCESS_ERR(
                    "[CM] Non-Virtual Address Add failed. Returning ...\n");
                return MS_INSUFFICIENT_RESOURCES;
            }
        }

        /** Check Publication Information is valid */
        if (0x01 == ms_access_model_list[model_handle].fixed.publish.valid)
        {
            /* Delete existing Address Handle - if not the same handle */
            if ((addr_handle != ms_access_model_list[model_handle].fixed.publish.addr_handle) || (MS_NET_ADDR_UNASSIGNED == publish_info->addr.addr))
            {
                ms_delete_address(ms_access_model_list[model_handle].fixed.publish.addr_handle, MS_ADDR_OP_TYPE_PUBLISH);
            }
        }

        /** PublishAddress Handle */
        ms_access_model_list[model_handle].fixed.publish.addr_handle = addr_handle;
        addr = publish_info->addr.addr;
        ACCESS_TRC(
            "[CM 0x%04X] Publish Address 0x%04X. Address Handle 0x%08X\n",
            model_handle, addr, addr_handle);

        /* Reset all the fields in publish info */
        if (MS_NET_ADDR_UNASSIGNED == addr)
        {
            EM_mem_set(publish_info, 0, sizeof(MS_ACCESS_PUBLISH_INFO));
        }

        /** AppKeyIndex */
        ms_access_model_list[model_handle].fixed.publish.appkey_handle = (UINT16)index;
        ACCESS_TRC(
            "[CM 0x%04X] Publish AppKey Handle 0x%04X, AppKeyIndex 0x%04X\n",
            model_handle, ms_access_model_list[model_handle].fixed.publish.appkey_handle,
            publish_info->appkey_index);
        /** CredentialFlag */
        ms_access_model_list[model_handle].fixed.publish.crden_flag = publish_info->crden_flag;
        ACCESS_TRC(
            "[CM 0x%04X] Publish CredentialFlag 0x%04X\n",
            model_handle, ms_access_model_list[model_handle].fixed.publish.crden_flag);
        /** PublishTTL */
        ms_access_model_list[model_handle].fixed.publish.ttl = publish_info->ttl;
        ACCESS_TRC(
            "[CM 0x%04X] Publish TTL 0x%04X\n",
            model_handle, ms_access_model_list[model_handle].fixed.publish.ttl);
        /** PublishPeriod */
        ms_access_model_list[model_handle].fixed.publish.period = publish_info->period;
        ACCESS_TRC(
            "[CM 0x%04X] Publish Period 0x%04X\n",
            model_handle, ms_access_model_list[model_handle].fixed.publish.period);
        /** PublishRetransmitCount */
        ms_access_model_list[model_handle].fixed.publish.rtx_count = publish_info->rtx_count;
        ACCESS_TRC(
            "[CM 0x%04X] Publish RetransmitCount 0x%04X\n",
            model_handle, ms_access_model_list[model_handle].fixed.publish.rtx_count);
        /** PublishRetransmitIntervalSteps */
        ms_access_model_list[model_handle].fixed.publish.rtx_interval_steps = publish_info->rtx_interval_steps;
        ACCESS_TRC(
            "[CM 0x%04X] Publish RetransmitIntervalSteps 0x%04X\n",
            model_handle, ms_access_model_list[model_handle].fixed.publish.rtx_interval_steps);
        MS_ACCESS_STOP_PERIODIC_STEP_TIMER(ms_access_model_list[model_handle].fixed.publish);

        if (MS_NET_ADDR_UNASSIGNED == addr)
        {
            /** Mark Publication Information as invalid */
            ms_access_model_list[model_handle].fixed.publish.valid = (UCHAR)0x00;
        }
        else
        {
            /** Mark Publication Information as valid */
            ms_access_model_list[model_handle].fixed.publish.valid = (UCHAR)0x01;
            /* If Periodic Timeout is set, add to the list */
            retval = MS_ACCESS_START_PERIODIC_STEP_TIMER
                     (
                         ms_access_model_list[model_handle].fixed.publish.period,
                         ms_access_model_list[model_handle].fixed.publish.period_divisor,
                         &model_handle,
                         &ms_access_model_list[model_handle].fixed.publish.timer_handle
                     );
        }

        ACCESS_TRC(
            "[CM 0x%04X] Publish Information Valid? 0x%02X\n",
            model_handle, ms_access_model_list[model_handle].fixed.publish.valid);
        retval = API_SUCCESS;
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_MODELS);
    }

    ACCESS_TRC(
        "[CM] Set Publication Information for Model Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To set Publication Fast Period Divisor information associated with a model

    \par Description
    This routine sets Publication Fast Period Divisor information associated with a model.

    \param [in] model_handle      Handle of the Model for which Publication info to be set.
    \param [in] period_divisor    The value range for the Health Fast Period Divisor state is
                                  0 through 15, all other values are prohibited.
                                  This is used to divide the Health Publish Period by 2^n,
                                  where the n is the value of the Health Fast Period Divisor state.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_set_model_publication_period_divisor
(
    /* IN */ MS_ACCESS_MODEL_HANDLE    model_handle,
    /* IN */ UINT8                     period_divisor
)
{
    API_RESULT               retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Set Publication Period Divisor State:0x%02X for Model: 0x%04X\n",
        period_divisor, model_handle);

    if (15 < period_divisor)
    {
        ACCESS_TRC(
            "[CM] Set Publication Period Divisor State:0x%02X for Model: 0x%04X\n",
            period_divisor, model_handle);
        return ACCESS_INVALID_PARAMETER_VALUE;
    }

    ms_access_model_list[model_handle].fixed.publish.period_divisor = period_divisor;
    MS_ACCESS_STOP_PERIODIC_STEP_TIMER(ms_access_model_list[model_handle].fixed.publish);
    /* If Periodic Timeout is set, add to the list */
    /* TODO: Check the last parameter and associated data type */
    retval = MS_ACCESS_START_PERIODIC_STEP_TIMER
             (
                 ms_access_model_list[model_handle].fixed.publish.period,
                 ms_access_model_list[model_handle].fixed.publish.period_divisor,
                 &model_handle,
                 &ms_access_model_list[model_handle].fixed.publish.timer_handle
             );
    ACCESS_TRC(
        "[CM] Set Model Publication Period Divisor Information for Model Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get Publication information associated with a model

    \par Description
    This routine returns Publication information associated with a model.

    \param [in]  model_handle      Handle of the Model for which Publication info to be returned.
    \param [out] publish_info      Memory to be filled with associated Publication info.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_model_publication
(
    /* IN */  MS_ACCESS_MODEL_HANDLE    model_handle,
    /* OUT */ MS_ACCESS_PUBLISH_INFO*   publish_info
)
{
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get Publication Information for Model 0x%04X\n", model_handle);
    /* Assume no Publication Information for model_handle */
    retval = ACCESS_NO_MATCH;

    /* Check model is valid */
    if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Check Publication Information is available */
        if (1 == ms_access_model_list[model_handle].fixed.publish.valid)
        {
            /** PublishAddress */
            /* Not checking return value */
            ms_get_address(ms_access_model_list[model_handle].fixed.publish.addr_handle, MS_ADDR_OP_TYPE_PUBLISH, &publish_info->addr.addr);
            ACCESS_TRC(
                "[CM 0x%04X] Publish Address 0x%04X\n",
                model_handle, publish_info->addr.addr);
            /** AppKeyIndex */
            publish_info->appkey_index = MS_APPKEY_INDEX(ms_access_model_list[model_handle].fixed.publish.appkey_handle);
            ACCESS_TRC(
                "[CM 0x%04X] Publish AppKey Handle 0x%04X, AppKeyIndex 0x%04X\n",
                model_handle, ms_access_model_list[model_handle].fixed.publish.appkey_handle,
                publish_info->appkey_index);
            /** CredentialFlag */
            publish_info->crden_flag = ms_access_model_list[model_handle].fixed.publish.crden_flag;
            ACCESS_TRC(
                "[CM 0x%04X] Publish CredentialFlag 0x%04X\n",
                model_handle, publish_info->crden_flag);
            /** PublishTTL */
            publish_info->ttl = ms_access_model_list[model_handle].fixed.publish.ttl;
            ACCESS_TRC(
                "[CM 0x%04X] Publish TTL 0x%04X\n",
                model_handle, publish_info->ttl);
            /** PublishPeriod */
            publish_info->period = ms_access_model_list[model_handle].fixed.publish.period;
            ACCESS_TRC(
                "[CM 0x%04X] Publish Period 0x%04X\n",
                model_handle, publish_info->period);
            /** PublishRetransmitCount */
            publish_info->rtx_count = ms_access_model_list[model_handle].fixed.publish.rtx_count;
            ACCESS_TRC(
                "[CM 0x%04X] Publish RetransmitCount 0x%04X\n",
                model_handle, publish_info->rtx_count);
            /** PublishRetransmitIntervalSteps */
            publish_info->rtx_interval_steps = ms_access_model_list[model_handle].fixed.publish.rtx_interval_steps;
            ACCESS_TRC(
                "[CM 0x%04X] Publish RetransmitIntervalSteps 0x%04X\n",
                model_handle, publish_info->rtx_interval_steps);
        }
        else
        {
            ACCESS_ERR(
                "[CM 0x%04X] Publish Information is not valid\n", model_handle);
            EM_mem_set(publish_info, 0, sizeof(MS_ACCESS_PUBLISH_INFO));
        }

        retval = API_SUCCESS;
    }

    ACCESS_TRC(
        "[CM] Get Publication Information for Model Returning 0x%04X\n", retval);
    return retval;
}

/** Search and add an entry to Non-Virtual Address Table */
API_RESULT ms_search_and_add_address
(
    /* IN */  MS_ACCESS_MODEL_HANDLE    model_handle,
    /* IN */  MS_NET_ADDR*               addr,
    /* IN */  MS_ADDR_OP_TYPE            type,
    /* OUT */ MS_ACCESS_ADDRESS_HANDLE* handle
)
{
    UINT32 index, free_index;
    API_RESULT retval;
    /* Initialize the index */
    free_index = MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS);
    retval = ACCESS_NO_RESOURCE;
    ACCESS_TRC(
        "[CM] Search and add Address 0x%04X\n", (*addr));

    //Publish reset case
    if((MS_ADDR_OP_TYPE_PUBLISH == type) && ((*addr) == MS_NET_ADDR_UNASSIGNED))
    {
        *handle = ms_access_model_list[model_handle].fixed.publish.addr_handle;
        return API_SUCCESS;
    }

    /* Check if the address is already present in the non-virtual table */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS); index ++)
    {
        /* Check for free entry */
        if (MS_NET_ADDR_UNASSIGNED == ms_non_virtual_addr_table[index].vaddr)
        {
            if (MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS) == free_index)
            {
                /* Save first free index */
                free_index = index;
            }
        }
        else
        {
            /* Compare Address */
            if ((*addr) == ms_non_virtual_addr_table[index].vaddr)
            {
                /* Found a match */
                ACCESS_TRC(
                    "[CM] Address 0x%04X already present at Handle 0x%08X\n", (*addr), index);

                /* Increment Count */
                if(((MS_ADDR_OP_TYPE_SUBSCRIBE == type) && (0 == bitarray_get_bit(ms_access_model_list[model_handle].config.subscription_bitarray, index)))
                        || ((MS_ADDR_OP_TYPE_PUBLISH == type) && (0x00 == ms_access_model_list[model_handle].fixed.publish.valid))
                        || ((MS_ADDR_OP_TYPE_PUBLISH == type) && (0x01 == ms_access_model_list[model_handle].fixed.publish.valid) &&
                            (index != ms_access_model_list[model_handle].fixed.publish.addr_handle)))
                    ms_non_virtual_addr_table[index].count[type]++;

                ACCESS_TRC(
                    "[CM 0x%08X] %s Count 0x%04X\n",
                    index, ((MS_ADDR_OP_TYPE_PUBLISH == type) ? "Publish" : "Subscribe"),
                    ms_non_virtual_addr_table[index].count[type]);
                break;
            }
        }
    }

    if (MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS) != index)
    {
        *handle = (MS_ACCESS_ADDRESS_HANDLE)index;
        /* Return Success */
        retval = API_SUCCESS;
    }
    else if (MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS) != free_index)
    {
        /* Found a free entry */
        /* Save the Non-Virtual Address */
        ms_non_virtual_addr_table[free_index].vaddr = *addr;
        ms_non_virtual_addr_table[free_index].count[type]++;
        *handle = (MS_ACCESS_ADDRESS_HANDLE)free_index;
        ACCESS_TRC(
            "[CM] Saving Address 0x%04X at Handle 0x%08X\n", (*addr), (*handle));
        ACCESS_TRC(
            "[CM 0x%08X] %s Count 0x%04X\n",
            free_index, ((MS_ADDR_OP_TYPE_PUBLISH == type) ? "Publish" : "Subscribe"),
            ms_non_virtual_addr_table[free_index].count[type]);
        retval = API_SUCCESS;
    }
    else
    {
        *handle = (MS_ACCESS_ADDRESS_HANDLE)free_index;
    }

    if (API_SUCCESS == retval)
    {
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_NON_VIRTUAL_ADDRS);
    }

    ACCESS_TRC(
        "[CM] Search and add Address returning 0x%04X, with Handle 0x%08X\n", retval, (*handle));
    return retval;
}

/** Search an entry to Non-Virtual Address Table */
API_RESULT ms_search_address
(
    /* IN */  MS_NET_ADDR*               addr,
    /* IN */  MS_ADDR_OP_TYPE            type,
    /* OUT */ MS_ACCESS_ADDRESS_HANDLE* handle
)
{
    UINT32 index;
    API_RESULT retval;
    retval = ACCESS_NO_MATCH;
    ACCESS_TRC(
        "[CM] Search Address 0x%04X. Type 0x%02X\n", (*addr), type);

    /* Check if the address is already present in the non-virtual table */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS); index ++)
    {
        /* Check for a valid entry */
        if (MS_NET_ADDR_UNASSIGNED != ms_non_virtual_addr_table[index].vaddr)
        {
            /* Compare Address */
            if ((*addr) == ms_non_virtual_addr_table[index].vaddr)
            {
                /* Found a match */
                ACCESS_TRC(
                    "[CM] Address 0x%04X present at Handle 0x%08X. Count 0x%04X\n",
                    (*addr), index, ms_non_virtual_addr_table[index].count[type]);

                /* Check match for address search type */
                if (ms_non_virtual_addr_table[index].count[type] > 0)
                {
                    *handle = (MS_ACCESS_ADDRESS_HANDLE)index;
                    retval = API_SUCCESS;
                }

                break;
            }
        }
    }

    ACCESS_TRC(
        "[CM] Search Address returning 0x%04X, with Handle 0x%08X\n", retval, (*handle));
    return retval;
}

/** Delete an entry from Non-Virtual/Virtual Address Table */
API_RESULT ms_delete_address
(
    /* IN */ MS_ACCESS_ADDRESS_HANDLE addr_handle,
    /* IN */ MS_ADDR_OP_TYPE          type
)
{
    API_RESULT  retval;
    retval = API_SUCCESS;
    ACCESS_TRC(
        "[CM] Delete Address Handle 0x%08X. Type 0x%02X\n", addr_handle, type);

    /* Check if from non-virtual or virtual address table */
    if (MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS) > addr_handle)
    {
        ACCESS_TRC(
            "[CM] Non-Virtual Address. Count 0x%04X\n",
            ms_non_virtual_addr_table[addr_handle].count[type]);

        /* Check if reference count is not already ZERO */
        if (0 != ms_non_virtual_addr_table[addr_handle].count[type])
        {
            /* Decrement Count */
            ms_non_virtual_addr_table[addr_handle].count[type]--;
        }

        /**
            Check if both publish and subscribe counts reached ZERO,
            before deleting the non-virtual address entry.
        */
        if ((0 == ms_non_virtual_addr_table[addr_handle].count[MS_ADDR_OP_TYPE_PUBLISH]) &&
                (0 == ms_non_virtual_addr_table[addr_handle].count[MS_ADDR_OP_TYPE_SUBSCRIBE]))
        {
            /* Free Non-Virtual Address */
            ms_non_virtual_addr_table[addr_handle].vaddr = MS_NET_ADDR_UNASSIGNED;
            ACCESS_TRC(
                "[CM] Deleted Non-Virtual Address Index 0x%04X\n",
                addr_handle);
        }

        /* PS Store */
//        ms_access_ps_store(MS_PS_RECORD_NON_VIRTUAL_ADDRS);
    }
    else if ((MS_ACCESS_ADDRESS_HANDLE)MS_MAX_ADDRS > addr_handle)
    {
        /* Get the index in the Virtual Address Table */
        addr_handle -= MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS);
        ACCESS_TRC(
            "[CM] Virtual Address. Count 0x%04X\n",
            ms_virtual_addr_table[addr_handle].count[type]);

        /* Check if reference count is not already ZERO */
        if (0 != ms_virtual_addr_table[addr_handle].count[type])
        {
            /* Decrement Count */
            ms_virtual_addr_table[addr_handle].count[type]--;
        }

        /**
            Check if both publish and subscribe counts reached ZERO,
            before deleting the virtual address entry.
        */
        if ((0 == ms_virtual_addr_table[addr_handle].count[MS_ADDR_OP_TYPE_PUBLISH]) &&
                (0 == ms_virtual_addr_table[addr_handle].count[MS_ADDR_OP_TYPE_SUBSCRIBE]))
        {
            /* Free Virtual Address */
            ms_virtual_addr_table[addr_handle].vaddr = MS_NET_ADDR_UNASSIGNED;
            ACCESS_TRC(
                "[CM] Deleted Virtual Address Index 0x%04X\n",
                addr_handle);
        }

        /* PS Store */
//        ms_access_ps_store(MS_PS_RECORD_VIRTUAL_ADDRS);
    }
    else
    {
        ACCESS_ERR(
            "[CM] Deleted Address Handle 0x%08X Failed. Out of Range\n",
            addr_handle);
        retval = ACCESS_INVALID_HANDLE;
    }

    ACCESS_TRC(
        "[CM] Delete Address Returning 0x%04X\n", retval);
    return retval;
}

/** Get an entry from Non-Virtual/Virtual Address Table */
API_RESULT ms_get_address
(
    /* IN */  MS_ACCESS_ADDRESS_HANDLE   addr_handle,
    /* IN */  MS_ADDR_OP_TYPE            type,
    /* OUT */ MS_NET_ADDR*               addr
)
{
    API_RESULT  retval;
    retval = ACCESS_INVALID_HANDLE;
    ACCESS_TRC(
        "[CM] Get Address Handle 0x%08X. Type 0x%02X\n", addr_handle, type);

    /* Check if from non-virtual or virtual address table */
    if (MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS) > addr_handle)
    {
        ACCESS_TRC(
            "[CM] Non-Virtual Address 0x%04X. Count 0x%04X\n",
            ms_non_virtual_addr_table[addr_handle].vaddr,
            ms_non_virtual_addr_table[addr_handle].count[type]);

        /* Check if reference count is not already ZERO */
        if (0 != ms_non_virtual_addr_table[addr_handle].count[type])
        {
            /* Return Address */
            *addr = ms_non_virtual_addr_table[addr_handle].vaddr;
            retval = API_SUCCESS;
        }
    }
    else if ((MS_ACCESS_ADDRESS_HANDLE)MS_MAX_ADDRS > addr_handle)
    {
        /* Get the index in the Virtual Address Table */
        addr_handle -= MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS);
        ACCESS_TRC(
            "[CM] Virtual Address 0x%04X. Count 0x%04X\n",
            ms_virtual_addr_table[addr_handle].vaddr,
            ms_virtual_addr_table[addr_handle].count[type]);

        /* Check if reference count is not already ZERO */
        if (0 != ms_virtual_addr_table[addr_handle].count[type])
        {
            /* Return Address */
            *addr = ms_virtual_addr_table[addr_handle].vaddr;
            retval = API_SUCCESS;
        }
    }
    else
    {
        ACCESS_ERR(
            "[CM] Get Address Handle 0x%08X Failed. Out of Range\n",
            addr_handle);
    }

    ACCESS_TRC(
        "[CM] Get Address Returning 0x%04X\n", retval);
    return retval;
}

/** Search and add an entry to Virtual Address Table */
API_RESULT ms_search_and_add_virtual_address
(
    /* IN */  UCHAR*                     label,
    /* IN */  MS_ADDR_OP_TYPE            type,
    /* OUT */ UINT16*                    addr,
    /* OUT */ MS_ACCESS_ADDRESS_HANDLE* handle
)
{
    UINT32 index, free_index;
    API_RESULT retval;
    ACCESS_TRC(
        "[CM] Search and add Virtual Address\n");
    ACCESS_debug_dump_bytes(label, 16);
    free_index = MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS);
    retval = ACCESS_NO_RESOURCE;

    /* Check if the label is already present in the virtual table */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS); index ++)
    {
        /* Check for free entry */
        if (0 == ms_virtual_addr_table[index].vaddr)
        {
            if (MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS) == free_index)
            {
                /* Save first free index */
                free_index = index;
            }
        }
        else
        {
            /* Compare Label */
            if (0 == EM_mem_cmp(label, ms_virtual_addr_table[index].label, MS_LABEL_UUID_LENGTH))
            {
                /* Found a match */
                ACCESS_TRC(
                    "[CM] Virtual Address already present at Handle 0x%08X\n", index);
                /* Increment Count */
                ms_virtual_addr_table[index].count[type]++;
                ACCESS_TRC(
                    "[CM 0x%08X] %s Count 0x%04X\n",
                    index, ((MS_ADDR_OP_TYPE_PUBLISH == type) ? "Publish" : "Subscribe"),
                    ms_virtual_addr_table[index].count[type]);
                break;
            }
        }
    }

    if (MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS) != index)
    {
        *handle = (MS_ACCESS_ADDRESS_HANDLE)(index + MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS));
        *addr = ms_virtual_addr_table[index].vaddr;
        /* Return Success */
        retval = API_SUCCESS;
    }
    else if (MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS) != free_index)
    {
        /* Found a free entry */
        /* Calculate corresponding virtual address */
        ms_stbx_va
        (
            label,
            MS_LABEL_UUID_LENGTH,
            addr
        );
        /* Save the Label and Virtual Address */
        EM_mem_copy(ms_virtual_addr_table[free_index].label, label, MS_LABEL_UUID_LENGTH);
        ms_virtual_addr_table[free_index].vaddr = *addr;
        ms_virtual_addr_table[free_index].count[type]++;
        *handle = (MS_ACCESS_ADDRESS_HANDLE)(free_index + MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS));
        ACCESS_TRC(
            "[CM] Saving Virtual Address 0x%04X at Handle 0x%08X\n", (*addr), (*handle));
        ACCESS_TRC(
            "[CM 0x%08X] %s Count 0x%04X\n",
            free_index, ((MS_ADDR_OP_TYPE_PUBLISH == type) ? "Publish" : "Subscribe"),
            ms_virtual_addr_table[index].count[type]);
        retval = API_SUCCESS;
    }

    if (API_SUCCESS == retval)
    {
        /* TODO: Check if for all success return PS to be updated */
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_VIRTUAL_ADDRS);
    }

    ACCESS_TRC(
        "[CM] Search and add Virtual Address returning 0x%04X, with Handle 0x%08X\n", retval, (*handle));
    return retval;
}

/** Search an entry to Virtual Address Table */
API_RESULT ms_search_virtual_address_on_label
(
    /* IN */  UCHAR*                     label,
    /* IN */  MS_ADDR_OP_TYPE            type,
    /* OUT */ UINT16*                    addr,
    /* OUT */ MS_ACCESS_ADDRESS_HANDLE* handle
)
{
    UINT32 index;
    API_RESULT retval;
    ACCESS_TRC(
        "[CM] Search Virtual Address\n");
    ACCESS_debug_dump_bytes(label, 16);
    retval = ACCESS_NO_MATCH;

    /* Check if the label is already present in the virtual table */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS); index ++)
    {
        /* Check for free entry */
        if (0 != ms_virtual_addr_table[index].vaddr)
        {
            /* Compare Label */
            if (0 == EM_mem_cmp(label, ms_virtual_addr_table[index].label, MS_LABEL_UUID_LENGTH))
            {
                /* Check match for address search type */
                if (ms_virtual_addr_table[index].count[type] > 0)
                {
                    /* Found a match */
                    *handle = (MS_ACCESS_ADDRESS_HANDLE)(index + MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS));
                    *addr = ms_virtual_addr_table[index].vaddr;
                    ACCESS_TRC(
                        "[CM] Virtual Address 0x%04X present at Handle 0x%08X. Count 0x%04X\n",
                        (*addr), (*handle), ms_virtual_addr_table[index].count[type]);
                    retval = API_SUCCESS;
                }

                break;
            }
        }
    }

    return retval;
}

API_RESULT ms_search_virtual_address
(
    /* IN */  MS_NET_ADDR*               addr,
    /* IN */  MS_ADDR_OP_TYPE            type,
    /* OUT */ MS_ACCESS_ADDRESS_HANDLE* handle
)
{
    UINT32 index;
    API_RESULT retval;
    ACCESS_TRC(
        "[CM] Search Virtual Address 0x%04X\n", *addr);
    retval = ACCESS_NO_MATCH;

    /* Check if the label is already present in the virtual table */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS); index ++)
    {
        /* Check for free entry */
        if (0 != ms_virtual_addr_table[index].vaddr)
        {
            /* Compare Address */
            if ((*addr) == ms_virtual_addr_table[index].vaddr)
            {
                /* Check match for address search type */
                if (ms_virtual_addr_table[index].count[type] > 0)
                {
                    /* Found a match */
                    *handle = (MS_ACCESS_ADDRESS_HANDLE)(index + MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS));
                    ACCESS_TRC(
                        "[CM] Virtual Address 0x%04X present at Handle 0x%08X. Count 0x%04X\n",
                        (*addr), (*handle), ms_virtual_addr_table[index].count[type]);
                    retval = API_SUCCESS;
                }

                break;
            }
        }
    }

    return retval;
}

/** Delete appkey */
void ms_delete_appkey(/* IN */ UINT32 key_index)
{
    UINT32 m_index;
    ACCESS_TRC("[CM] Deleting AppKey at Index 0x%08X\n", key_index);
    /* Reset AppKey Entry */
    MS_INIT_APPKEY_ENTRY(key_index);

    /* Update associated models - bind and publish address */
    /* Loop through all models */
    for (m_index = 0; m_index < MS_CONFIG_LIMITS(MS_ACCESS_MODEL_COUNT); m_index++)
    {
        /* Check if contains valid model */
        if (1 == ms_access_model_list[m_index].fixed.valid)
        {
            /* Reset AppKey index in bitarray */
            bitarray_reset_bit(ms_access_model_list[m_index].config.appkey_bitarray, key_index);

            /** Check for matching AppKeyIndex in Publication Information */
            if ((0x01 == ms_access_model_list[m_index].fixed.publish.valid) &&
                    (ms_access_model_list[m_index].fixed.publish.appkey_handle == (UINT16)key_index))
            {
                ACCESS_TRC("[CM] Marking Publish as Invalid for Model Index:0x%08X\n", m_index);
                /** Mark Publication Information as invalid */
                ms_access_model_list[m_index].fixed.publish.valid = (UCHAR)0x00;
            }
        }
    }
}

/**
    \brief To add an address to a model subscription list

    \par Description
    This routine adds an address to a subscription list of a model

    \param [in] model_handle      Handle of the Model for which address to be added in the subscription list.
    \param [in] sub_addr          Address to be added in subscription list.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_add_model_subscription
(
    /* IN */ MS_ACCESS_MODEL_HANDLE    model_handle,
    /* IN */ MS_ACCESS_ADDRESS*        sub_addr
)
{
    API_RESULT               retval;
    MS_NET_ADDR              addr;
    MS_ACCESS_ADDRESS_HANDLE addr_handle;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Add address to subscription list of model 0x%04X\n", model_handle);
    /* TODO: Print Information */
    /* Assume no Information for model_handle */
    retval = ACCESS_NO_MATCH;

    /* Check model is valid */
    if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Check if Virtual Address */
        if (1 == sub_addr->use_label)
        {
            /* Search/Add entry in virtual address table */
            if (API_SUCCESS != ms_search_and_add_virtual_address(sub_addr->label, MS_ADDR_OP_TYPE_SUBSCRIBE, &addr, &addr_handle))
            {
                ACCESS_ERR(
                    "[CM] Virtual Address Add failed. Returning ...\n");
                return MS_INSUFFICIENT_RESOURCES;
            }
            else
            {
                /* Return back generated Virtual Address */
                sub_addr->addr = addr;
            }
        }
        else
        {
            addr = sub_addr->addr;

            /* Add entry in non-virtual address table */
            if (API_SUCCESS != ms_search_and_add_address(model_handle,&addr, MS_ADDR_OP_TYPE_SUBSCRIBE, &addr_handle))
            {
                ACCESS_ERR(
                    "[CM] Non-Virtual Address Add failed. Returning ...\n");
                return MS_INSUFFICIENT_RESOURCES;
            }
        }

        /** Add address */
        retval = API_SUCCESS;
        /* Set Subscription Address Handle in bitarray */
        bitarray_set_bit(ms_access_model_list[model_handle].config.subscription_bitarray, addr_handle);
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_MODELS);
        ACCESS_TRC(
            "[CM] Subscription Bitarray [0] : 0x%08X\n",
            ms_access_model_list[model_handle].config.subscription_bitarray[0]);
    }

    ACCESS_TRC(
        "[CM] Add address to subscription list of model Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To delete an address to a model subscription list

    \par Description
    This routine deletes an address to a subscription list of a model

    \param [in] model_handle      Handle of the Model for which address to be deleted in the subscription list.
    \param [in] sub_addr          Address to be deleted from subscription list.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_delete_model_subscription
(
    /* IN */ MS_ACCESS_MODEL_HANDLE    model_handle,
    /* IN */ MS_ACCESS_ADDRESS*        sub_addr
)
{
    API_RESULT               retval;
    MS_ACCESS_ADDRESS_HANDLE addr_handle;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Delete address to subscription list of model 0x%04X\n", model_handle);
    /* TODO: Print Information */
    /* Assume no Information for model_handle */
    retval = ACCESS_NO_MATCH;
    /* To keep some compilers happy */
    addr_handle = MS_ACCESS_ADDRESS_INVALID_HANDLE;

    /* Check model is valid */
    if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Check if Virtual Address */
        if (1 == sub_addr->use_label)
        {
            /* Search entry in virtual address table */
            if (API_SUCCESS != ms_search_virtual_address_on_label(sub_addr->label, MS_ADDR_OP_TYPE_SUBSCRIBE, &sub_addr->addr, &addr_handle))
            {
                ACCESS_ERR(
                    "[CM] Virtual Address Search failed. Returning ...\n");
                return API_SUCCESS;
            }
        }
        else
        {
            /* Search entry in non-virtual address table */
            if (API_SUCCESS != ms_search_address(&sub_addr->addr, MS_ADDR_OP_TYPE_SUBSCRIBE, &addr_handle))
            {
                ACCESS_ERR(
                    "[CM] Non-Virtual Address Search failed. Returning ...\n");
                return API_SUCCESS;
            }
        }

        /** Delete address */
        retval = ms_delete_address(addr_handle, MS_ADDR_OP_TYPE_SUBSCRIBE);
    }

    if (API_SUCCESS == retval)
    {
        /* Reset Subscription Address Handle in bitarray */
        bitarray_reset_bit(ms_access_model_list[model_handle].config.subscription_bitarray, addr_handle);
        /* PS Store */
        ms_access_ps_store(MS_PS_RECORD_MODELS);
    }

    ACCESS_TRC(
        "[CM] Add address to subscription list of model Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To discard a model subscription list

    \par Description
    This routine discards a subscription list of a model

    \param [in] model_handle      Handle of the Model for which the subscription list to be discarded.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_delete_all_model_subscription
(
    /* IN */ MS_ACCESS_MODEL_HANDLE    model_handle
)
{
    API_RESULT retval;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Delete All Subscription list for Model 0x%04X\n", model_handle);
    /* Assume no Subscription for model_handle */
    retval = ACCESS_NO_MATCH;

    /* Check model is valid */
    if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Search for all Subscription Indices */
        UINT32 start, next;
        start = 0;
        MS_LOOP_FOREVER()
        {
            next = bitarray_get_lowest_bit_set
                   (
                       ms_access_model_list[model_handle].config.subscription_bitarray,
                       MS_MAX_ADDRS,
                       start
                   );

            if (0xFFFFFFFF == next)
            {
                break;
            }

            /* Delete Address */
            ms_delete_address(next, MS_ADDR_OP_TYPE_SUBSCRIBE);
            start = next + 1;
        }
        /* Reset Subscription List */
        bitarray_reset_all
        (
            ms_access_model_list[model_handle].config.subscription_bitarray,
            MS_MAX_ADDRS
        );
        /* Return Success */
        retval = API_SUCCESS;
    }

    /* PS Store */
    ms_access_ps_store(MS_PS_RECORD_MODELS);
    ACCESS_TRC(
        "[CM] Delete All Subscription list for Model Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get list of subscription addresses of a model

    \par Description
    This routine returns a list of subscription addresses of a model.

    \param [in] model_handle        Handle of the Model for which the subscription addresses to be returned.
    \param [inout] sub_addr_count   Caller fills with maximum number of subscription addresses
                                    that can be stored in 'sub_addr_list'.
                                    This function will update the value with how many subscription addresses
                                    has been filled. If the number of available subscription addresses is more than that can be returned,
                                    maximum possible addresses will be filled and an appropriate error values will inform the caller,
                                    there are more subscription addresses (as an information).
    \param [out] sub_addr_list      Memory to be filled with the available subscription addresses.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_model_subscription_list
(
    /* IN */    MS_ACCESS_MODEL_HANDLE    model_handle,
    /* INOUT */ UINT16*                   sub_addr_count,
    /* OUT */   UINT16*                   sub_addr_list
)
{
    API_RESULT retval;
    UINT16 actual_count;
    /* TODO: Parameter Validation */
    ACCESS_TRC(
        "[CM] Get Subscription list for Model 0x%04X. List Count: 0x%04X\n", model_handle, *sub_addr_count);
    /* Assume no Subscription for model_handle */
    retval = ACCESS_NO_MATCH;
    actual_count = 0;

    /* Check model is valid */
    if (1 == ms_access_model_list[model_handle].fixed.valid)
    {
        /* Search for all Subscription Indices */
        UINT32 start, next;
        start = 0;
        MS_LOOP_FOREVER()
        {
            next = bitarray_get_lowest_bit_set
                   (
                       ms_access_model_list[model_handle].config.subscription_bitarray,
                       MS_MAX_ADDRS,
                       start
                   );

            if (0xFFFFFFFF == next)
            {
                break;
            }

            /* Fill Subscription List */
            /* Check if there is free space */
            if (actual_count < (*sub_addr_count))
            {
                /* Copy Subscription Address */
                ms_get_address(next, MS_ADDR_OP_TYPE_SUBSCRIBE, &sub_addr_list[actual_count]);
                ACCESS_TRC(
                    "[CM] SubscriptionList[%02X] 0x%04X\n", actual_count, sub_addr_list[actual_count]);
                /* Increase count */
                actual_count++;
                start = next + 1;
            }
            else
            {
                ACCESS_ERR(
                    "[CM] Receive Subscription List Overflow. Returning ...\n");
                /* TODO: Return appropriate status, more Subscription Indices available */
                break;
            }
        }
    }

    /* TODO: Need a better mechanism to set return value in this function */
    if (0 != actual_count)
    {
        retval = API_SUCCESS;
    }

    /* Update with Actual Count */
    *sub_addr_count = actual_count;
    ACCESS_TRC(
        "[CM] Get Subscription list for Model Returning 0x%04X\n", retval);
    return retval;
}

/**
    \brief To get list of subscription addresses of all the models

    \par Description
    This routine returns a consolidated list of subscription addresses of all the models.

    \param [inout] sub_addr_count   Caller fills with maximum number of subscription addresses
                                    that can be stored in 'sub_addr_list'.
                                    This function will update the value with how many subscription addresses
                                    has been filled. If the number of available subscription addresses is more than that can be returned,
                                    maximum possible addresses will be filled and an appropriate error values will inform the caller,
                                    there are more subscription addresses (as an information).
    \param [out] sub_addr_list      Memory to be filled with the available subscription addresses.

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_access_cm_get_all_model_subscription_list
(
    /* INOUT */ UINT16*                   sub_addr_count,
    /* OUT */   UINT16*                   sub_addr_list
)
{
    API_RESULT retval;
    UINT16 actual_count;
    /* Get list of all models */
    UINT32     index;

    /* TODO: Parameter Validation */
    if ((NULL == sub_addr_list) || (NULL == sub_addr_count) || (0 == (*sub_addr_count)))
    {
        ACCESS_ERR(
            "[CM] Invalid Parameter in get all model subsciption list\n");
        return ACCESS_INVALID_PARAMETER_VALUE;
    }

    ACCESS_TRC(
        "[CM] Get Subscription list for all models. List Count: 0x%04X\n", *sub_addr_count);
    retval = API_SUCCESS;
    actual_count = 0;

    /* Loop through non-virtual address table */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_NON_VIRTUAL_ADDRS); index++)
    {
        /* Check for a valid entry */
        if ((0 != ms_non_virtual_addr_table[index].vaddr) &&
                (0 != ms_non_virtual_addr_table[index].count[MS_ADDR_OP_TYPE_SUBSCRIBE]))
        {
            sub_addr_list[actual_count] = ms_non_virtual_addr_table[index].vaddr;
            actual_count++;

            if (actual_count == (*sub_addr_count))
            {
                break;
            }
        }
    }

    /* Loop through virtual address table */
    for (index = 0; index < MS_CONFIG_LIMITS(MS_MAX_VIRTUAL_ADDRS); index ++)
    {
        /* Check for a valid entry */
        if ((0 != ms_virtual_addr_table[index].vaddr) &&
                (0 != ms_virtual_addr_table[index].count[MS_ADDR_OP_TYPE_SUBSCRIBE]))
        {
            sub_addr_list[actual_count] = ms_virtual_addr_table[index].vaddr;
            actual_count++;

            if (actual_count == (*sub_addr_count))
            {
                break;
            }
        }
    }

    /* Update with Actual Count */
    *sub_addr_count = actual_count;
    ACCESS_TRC(
        "[CM] Get All Model Subscription list Returning 0x%04X, with count %d\n",
        retval, (*sub_addr_count));
    return retval;
}
#endif /* MS_ACCESS */

