
/**
    \file net_proxy.c

    Network Proxy related routines are defined here.
*/

/*
    Copyright (C) 2017. Mindtree Limited.
    All rights reserved.
*/


/* --------------------------------------------- Header File Inclusion */
#include "net_internal.h"
#include "MS_access_api.h"
#include "cry.h"
#include "access_internal.h"
#include "net_extern.h"


#ifdef MS_PROXY_SUPPORT

/* --------------------------------------------- Macros */
#define PROXY_IFACE_UP                       MS_TRUE
#define PROXY_IFACE_DOWN                     MS_FALSE

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
/* Proxy Application callback pointer */
DECL_STATIC PROXY_NTF_CB proxy_callback;

/* Proxy Interface State */
DECL_STATIC UCHAR proxy_iface_state;

#ifdef MS_PROXY_SERVER
    DECL_STATIC MS_DEFINE_GLOBAL_ARRAY(PROXY_FILTER_LIST, net_proxy_list, PROXY_FILTER_LIST_COUNT);

    /* Proxy Adv for NetID timer handle */
    DECL_STATIC EM_timer_handle net_proxy_netid_timer_handle;

    /* Proxy Adv for NodeID timer handle */
    DECL_STATIC EM_timer_handle net_proxy_nodeid_timer_handle;

    /* Total timeout of nodeid proxy adv procedure */
    DECL_STATIC UCHAR proxy_nodeid_all_subnets;

    /* Total timeout of nodeid proxy adv procedure */
    DECL_STATIC UINT32 proxy_nodeid_timeout;

    /* Proxy Adv state */
    DECL_STATIC UCHAR proxy_adv_state;
#endif /* MS_PROXY_SERVER */

/* --------------------------------------------- Internal Functions */
API_RESULT net_proxy_init(void)
{
    API_RESULT retval;
    #ifdef MS_PROXY_SERVER
    UINT32     index,i;
    #endif /* MS_PROXY_SERVER */
    retval = API_SUCCESS;
    NET_TRC (
        "[PROXY]: >> net_proxy_init\n");
    /* Initialize Proxy Lists */
    #ifdef MS_PROXY_SERVER
//  ms_access_ps_load(MS_PS_RECORD_PROXY_FILTER_ADDR);

    for (index = 0; index < (UINT32)PROXY_FILTER_LIST_COUNT; index ++)
    {
        /* TODO: Create Macros for Initializations */
//        MS_INIT_GLOBAL_ARRAY(PROXY_ADDR, net_proxy_list[index].p_addr, MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE), 0x00);
        MS_INIT_GLOBAL_ARRAY(PROXY_ADDR, net_proxy_list[index].v_addr, MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE), 0x00);
        net_proxy_list[index].v_count = 0;
        /* Set the incoming filter type for the SubNet's Proxy List */
        net_proxy_list[index].type = 0;
        net_proxy_list[index].role  = BRR_INVALID_ROLE;
    }

    /* Initialize Timers */
    net_proxy_netid_timer_handle  = EM_TIMER_HANDLE_INIT_VAL;
    net_proxy_nodeid_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    /* Initialize Proxy ADV related Globals */
    proxy_nodeid_all_subnets      = MS_FALSE;
    proxy_nodeid_timeout          = 0;
    proxy_adv_state               = 0x00;
    #endif /* MS_PROXY_SERVER */
    /* Initialize Callback */
    proxy_callback = NULL;
    /* Initialize Proxy Iface State */
    proxy_iface_state             = PROXY_IFACE_DOWN;
    NET_TRC (
        "[PROXY]: << net_proxy_init, retval 0x%04X\n", retval);
    return retval;
}

void net_proxy_iface_up (NETIF_HANDLE* handle, UCHAR role)
{
    /* UNLOCK ?*/
    /* TODO: As proxy server, send Secure Beacon */
    /* TODO: As proxy client, send Secure Beacon, if IVI has/is getting changed */
    /* Save the role of the Proxy List w.r.t to Netif Handle */
    #ifdef MS_PROXY_SERVER
    net_proxy_list[(*handle - 1)].role = role;
    #endif /* MS_PROXY_SERVER */

    /* Call the proxy application callback */
    if (NULL != proxy_callback)
    {
        /* Update Proxy State to "Enabled"! */
        proxy_iface_state             = PROXY_IFACE_UP;
        proxy_callback
        (
            handle,
            MS_PROXY_UP_EVENT,
            &role,
            sizeof(role)
        );
    }
    else
    {
        NET_ERR(
            "[PROXY-ERR]: Proxy Callback Not Registered\n");
    }

    /* LOCK ?*/
}

void net_proxy_iface_down(NETIF_HANDLE* handle)
{
    #ifdef MS_PROXY_SERVER
    UINT32 index;
    #endif /* MS_PROXY_SERVER */
    /* UNLOCK ?*/
    /* TODO: Create Macros for Initializations */
    #ifdef MS_PROXY_SERVER
    NET_TRC (
        "[PROXY]: >> net_proxy_iface_down\n");

    for (index = 0; index < (UINT32)PROXY_FILTER_LIST_COUNT; index ++)
    {
        EM_mem_set
        (
            net_proxy_list[(*handle - 1)].v_addr,
            0x0,
            (sizeof(PROXY_ADDR) * MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE))
        );
        net_proxy_list[(*handle - 1)].v_count = 0;
        net_proxy_list[(*handle - 1)].type  = 0;
        net_proxy_list[(*handle - 1)].role  = BRR_INVALID_ROLE;
    }

    #endif /* MS_PROXY_SERVER */

    /* Call the proxy application callback */
    if (NULL != proxy_callback)
    {
        /* Update Proxy State to "Disabled"! */
        proxy_iface_state             = PROXY_IFACE_DOWN;
        proxy_callback
        (
            handle,
            MS_PROXY_DOWN_EVENT,
            NULL,
            0x0
        );
    }
    else
    {
        NET_ERR(
            "[PROXY-ERR]: Proxy Callback Not Registered\n");
    }

    /* LOCK ?*/
}

void net_proxy_recv
(
    MS_NET_HEADER*     net_hdr,
    NETIF_HANDLE*      handle,
    MS_SUBNET_HANDLE   subnet_handle,
    UCHAR*             data_param,
    UINT16             data_len
)
{
    UINT8 opcode;
    /* TODO NULL Checks and Param Validation? */
    #ifndef MS_PROXY_SERVER
    MS_IGNORE_UNUSED_PARAM(subnet_handle);
    #endif /* MS_PROXY_SERVER */
    NET_TRC (
        "[PROXY]: >> net_proxy_recv\n");
    /* Validate Based on Network Header */
    /* Extract the Opcode */
    opcode = data_param[0];
    NET_TRC (
        "[PROXY]: Proxy Configuration 0x%02X from Device 0x%04X\n", opcode, net_hdr->saddr);
    NET_debug_dump_bytes(data_param, data_len);

    switch(opcode)
    {
        #ifdef MS_PROXY_SERVER

    case MS_PROXY_SET_FILTER_OPCODE:
    {
        NET_TRC (
            "[PROXY]: Received MS_PROXY_SET_FILTER_OPCODE\n");

        /* TODO: Improve length check */
        if ((2 >= data_len) && (1 >= data_param[1]))
        {
            /* Valid values of Proxy Filter Received */
            net_proxy_server_set_filter(handle, data_param[1]);
            /* Send Response */
            net_proxy_send_filter_status(handle, subnet_handle);
        }
    }
    break;

    case MS_PROXY_ADD_TO_FILTER_OPCODE:
    {
        NET_TRC (
            "[PROXY]: Received MS_PROXY_ADD_TO_FILTER_OPCODE\n");
        /* TODO: Length Check */
        net_proxy_server_add_to_list
        (
            handle,
            &data_param[1],
            (data_len - 1),
            MS_FALSE
        );
        /* Send Response */
        net_proxy_send_filter_status(handle, subnet_handle);
    }
    break;

    case MS_PROXY_REM_FROM_FILTER_OPCODE:
    {
        NET_TRC (
            "[PROXY]: Received MS_PROXY_REM_FROM_FILTER_OPCODE\n");
        net_proxy_server_del_from_list
        (
            handle,
            &data_param[1],
            (data_len - 1),
            MS_FALSE
        );
        /* Send Response */
        net_proxy_send_filter_status(handle, subnet_handle);
    }
    break;
        #endif /* MS_PROXY_SERVER */
    #ifdef MS_PROXY_CLIENT

    /* TODO: Handle events base on Proxy Client or Server */
    case MS_PROXY_FILTER_STATUS_OPCODE:
    {
        PROXY_FILTER_TYPE filter_type;
        UINT16            count;
        /* TODO: Validate Data Length */
        /* Extract the Status contents */
        filter_type = data_param[1];
        MS_UNPACK_BE_2_BYTE(&count, &data_param[2]);
        NET_TRC (
            "[PROXY]: Received MS_PROXY_FILTER_STATUS_OPCODE with"
            " Filter Type 0x%02X and Count = 0x%04X\n", filter_type, count);

        /* UNLOCK */

        /** TODO: Should this be protected by CLIENT FLAG ? */
        /* Call the proxy application callback */
        if (NULL != proxy_callback)
        {
            proxy_callback
            (
                handle,
                MS_PROXY_STATUS_EVENT,
                &data_param[1],
                0x03
            );
        }
        else
        {
            NET_ERR(
                "[PROXY-ERR]: Proxy Callback Not Registered\n");
        }

        /* LOCK */
    }
    break;
        #endif /* MS_PROXY_CLIENT */

    default:
        NET_ERR(
            "[PROXY-ERR]: Unknown Proxy Configuration Opcode 0x%02X received!\n", opcode);
        break;
    }

    NET_TRC (
        "[PROXY]: << net_proxy_recv\n");
}

API_RESULT net_proxy_send
(
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ UCHAR*               pdu,
    /* IN */ UINT16             pdu_len
)
{
    MS_NET_HEADER    hdr;
    MS_BUFFER        buffer;
    API_RESULT       retval;
    NET_TRC (
        "[PROXY]: >> net_proxy_send\n");
    /* Destination address */
    hdr.daddr = MS_NET_ADDR_UNASSIGNED;
    /* Source Address */
    MS_access_cm_get_primary_unicast_address(&hdr.saddr);
    /* TTL */
    hdr.ttl = 0x00;
    /* Hard code, CTL as 1 for Proxy Message */
    hdr.ctl = 0x01;
    MS_net_alloc_seq_num(&hdr.seq_num);
    /**
        Note: IVI, SRC and NID will be taken care by the network layer,
        apart from NetMIC.
    */
    hdr.nid = 0;
    /* Set PDU */
    buffer.payload = pdu;
    buffer.length  = pdu_len;
    retval = MS_net_send_pdu(&hdr, subnet_handle, &buffer,MS_FALSE);
    NET_TRC (
        "[PROXY]: << net_proxy_send, retval 0x%04X\n", retval);
    return retval;
}

#ifdef MS_PROXY_SERVER
API_RESULT net_proxy_server_set_filter
(
    /* IN */ NETIF_HANDLE*       handle,
    /* IN */ PROXY_FILTER_TYPE   type
)
{
    NET_TRC (
        "[PROXY]: >> net_proxy_server_set_filter\n");
    /* TODO: Have Validation of Parameters */
    /* TODO: Check if the type is changing and reset list in that case */
    /* Re-Initialize the Filter List of the particular Netif */
    EM_mem_set
    (
        net_proxy_list[(*handle - 1)].v_addr,
        0x0,
        (sizeof(PROXY_ADDR) * MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE))
    );
    net_proxy_list[(*handle - 1)].v_count = 0;
    /* Set the incoming filter type for the SubNet's Proxy List */
    net_proxy_list[(*handle - 1)].type = type;
    NET_TRC (
        "[PROXY]: << net_proxy_server_set_filter\n");
    return API_SUCCESS;
}

API_RESULT net_proxy_server_filter_op
(
    /* IN */ NETIF_HANDLE*   handle,
    /* IN */ UCHAR           opcode,
    /* IN */ UCHAR*          pdu,
    /* IN */ UINT16          pdu_len,
    /* IN */ UCHAR           proxy_fitlter_flg
)
{
    API_RESULT retval;
    UINT16      t_count;
    UINT32     i1, i2,i;
    PROXY_ADDR t_addr;
    NET_TRC (
        "[PROXY]: >> net_proxy_server_filter_op received for opcode 0x%02X\n", opcode);
    retval = API_SUCCESS;

    /*  Check if the PDU len received is greater than equal to 2 and is a
        multiple of 2
    */
    if ((2 > pdu_len) || (0x0001 & pdu_len) ||(net_proxy_list[(*handle - 1)].p_count > 5)||(net_proxy_list[(*handle - 1)].v_count > 5))
    {
        NET_ERR(
            "[PROXY-ERR]: ODD PDU Len: 0x%04X for Opcode[0x%02X]\n", pdu_len, opcode);
        retval = API_FAILURE;
    }

    /* Find the total count of addresses present in the pdu */
    t_count = (pdu_len/2);

    switch(opcode)
    {
    case MS_PROXY_ADD_TO_FILTER_OPCODE:

//      ms_access_ps_load(MS_PS_RECORD_PROXY_FILTER_ADDR);
        for (i = 0; i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i++)
        {
            net_proxy_list[0].p_addr[i] = ms_proxy_filter_addr[i];

//          printf (
//          "[PROXY]: reload Proxy Filter addr list 0x%04x 0x%04x\n",net_proxy_list[0].p_addr[i],ms_proxy_filter_addr[i]);

            if(ms_proxy_filter_addr[i] != 0x0000)
            {
                net_proxy_list[0].p_count++;
            }
        }

        NET_TRC (
            "[PROXY]: reload Proxy Filter count 0x%x\n",net_proxy_list[0].p_count);

        if(proxy_fitlter_flg == MS_TRUE)
        {
            for (i1 = 0; i1 < t_count; i1++)
            {
                /* Extract addresses */
                MS_UNPACK_BE_2_BYTE(&t_addr, &pdu[i1 * 2]);

                /* Check if Address is present in Subnet's proxy list */
                for (i2 = 0; i2 < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i2++)
                {
//                  NET_TRC (
//                      "[PROXY]:i2 0x%x, 0x%04X ,0x%04x ,0x%04x\n",i2,net_proxy_list[(*handle - 1)].p_addr[i2], t_addr, net_proxy_list[(*handle - 1)].v_addr[i2]);
                    if((net_proxy_list[(*handle - 1)].p_addr[i2] == t_addr) || (net_proxy_list[(*handle - 1)].v_addr[i2] == t_addr))
                    {
                        break;
                    }
                }

                /* Not present in the filter list and list is not full */
                if ((MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE) == i2) &&
                        (MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE) != net_proxy_list[(*handle - 1)].v_count))
                {
                    /* Index to be populated */
                    i2 = net_proxy_list[(*handle - 1)].v_count;
                    net_proxy_list[(*handle - 1)].v_addr[i2] = t_addr;
                    net_proxy_list[(*handle - 1)].v_count++;
                    NET_TRC (
                        "[PROXY]: First added 0x%04X to List of SubNet 0x%02X,proxy count 0x%x\n", t_addr, *handle,net_proxy_list[(*handle - 1)].v_count);

                    for (i = 0; i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i++)
                    {
                        NET_TRC (
                            "[PROXY]: handle 0x%x,Proxy Filter addr list 0x%04x\n",*handle, net_proxy_list[(*handle - 1)].v_addr[i]);
                    }
                }
                else
                {
                    NET_ERR(
                        "[PROXY-ERR]: Proxy List Full or Address 0x%04X Already present!\n", t_addr);
                }
            }

//          proxy_first_fitlter = MS_FALSE;
        }
        else
        {
            for (i1 = 0; i1 < t_count; i1++)
            {
                /* Extract addresses */
                MS_UNPACK_BE_2_BYTE(&t_addr, &pdu[i1 * 2]);

                /* Check if Address is present in Subnet's proxy list */
                for (i2 = 0; i2 < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i2++)
                {
                    if((net_proxy_list[(*handle - 1)].p_addr[i2] == t_addr) || (net_proxy_list[(*handle - 1)].v_addr[i2] == t_addr))
                    {
                        break;
                    }
                }

                /* Not present in the filter list and list is not full */
                if ((MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE) == i2) &&
                        (MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE) != net_proxy_list[(*handle - 1)].p_count))
                {
                    /* Index to be populated */
                    i2 = net_proxy_list[(*handle - 1)].p_count;
                    net_proxy_list[(*handle - 1)].p_addr[i2] = t_addr;
                    ms_proxy_filter_addr[i2] = net_proxy_list[(*handle - 1)].p_addr[i2];
                    net_proxy_list[(*handle - 1)].p_count++;
                    NET_TRC (
                        "[PROXY]: added 0x%04X to List of SubNet 0x%02X,proxy count 0x%x\n", t_addr, *handle,net_proxy_list[(*handle - 1)].p_count);
//                  for (i = 0; i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i++)
//                  {
////                        ms_proxy_filter_addr[i] = net_proxy_list[(*handle - 1)].p_addr[i];
//                      NET_TRC (
//                      "[PROXY]: after handle 0x%x,Proxy Filter addr list 0x%04x 0x%04x\n",*handle, net_proxy_list[0].p_addr[i],ms_proxy_filter_addr[i]);
//
//                  }
                }
                else
                {
                    NET_TRC(
                        "[PROXY-ERR]: Proxy List Full or Address 0x%04X Already present!\n", t_addr);
                }
            }

            ms_access_ps_store(MS_PS_RECORD_PROXY_FILTER_ADDR);
            //no first fitler should write to utilities config
        }

        break;

    case MS_PROXY_REM_FROM_FILTER_OPCODE:
        for (i1 = 0; i1 < t_count; i1++)
        {
            /* Extract addresses */
            MS_UNPACK_BE_2_BYTE(&t_addr, &pdu[i1 * 2]);

            /* Check if Address is present in Subnet's proxy list */
            for (i2 = 0; i2 < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i2++)
            {
                if (net_proxy_list[(*handle - 1)].p_addr[i2] == t_addr)
                {
                    net_proxy_list[(*handle - 1)].p_addr[i2] = MS_NET_ADDR_UNASSIGNED;
                    ms_proxy_filter_addr[i2] = net_proxy_list[(*handle - 1)].p_addr[i2];

                    if (0 != net_proxy_list[(*handle - 1)].p_count)
                    {
                        net_proxy_list[(*handle - 1)].p_count--;
                    }
                    else
                    {
                        /* Log it as Error */
                        NET_ERR(
                            "[PROXY-ERR]: List count already ZERO!\n", t_addr);
                    }

                    break;
                }
            }

            if (MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE) == i2)
            {
                NET_ERR(
                    "[PROXY-ERR]: Address 0x%04X not present in list!\n", t_addr);
            }
        }

        ms_access_ps_store(MS_PS_RECORD_PROXY_FILTER_ADDR);
        break;

    default:
        retval = API_FAILURE;
        break;
    }

    NET_TRC (
        "[PROXY]: << net_proxy_server_filter_op, retval 0x%04X\n", retval);
    return retval;
}

void net_proxy_send_filter_status
(
    NETIF_HANDLE*        handle,
    MS_SUBNET_HANDLE     subnet_handle
)
{
    UCHAR pdu[4];
    /* TODO: Null checks and Validation */
    NET_TRC (
        "[PROXY]: >> net_proxy_send_filter_status\n");
    /* Search Filter Type and Filter Count Based on SubNet Handle */
    pdu[0] = MS_PROXY_FILTER_STATUS_OPCODE;
    pdu[1] = net_proxy_list[(*handle - 1)].type;
    MS_PACK_BE_2_BYTE_VAL(&pdu[2], net_proxy_list[(*handle - 1)].p_count);
    MS_PACK_BE_2_BYTE_VAL(&pdu[2], net_proxy_list[(*handle - 1)].v_count);
    net_proxy_send
    (
        subnet_handle,
        pdu,
        4
    );
    NET_TRC (
        "[PROXY]: << net_proxy_send_filter_status\n");
}

API_RESULT net_proxy_process_first_pkt
(
    /* IN */ NETIF_HANDLE*    handle,
    /* IN */ PROXY_ADDR       src_addr
)
{
    API_RESULT retval;
    UCHAR      pdu[2] = {0x00, 0x00};
    NET_TRC (
        "[PROXY]: >> net_proxy_process_first_pkt\n");
    retval = API_SUCCESS;

    /* LOCK */

    /**
        Check if Proxy Filter List Pertaining to Netif Handle
        is already initialized.
    */

    /**
        NOTE: Currently Handling only WhiteList Filter Addition.
        As WhiteList is the Default filter.
    */
    if (BRR_SERVER_ROLE == net_proxy_list[(*handle - 1)].role)
    {
        NET_TRC (
            "[PROXY]: "
            "handle 0x%02X, src_addr 0x%04x, proxy list count 0x%x\n", *handle,src_addr,net_proxy_list[(*handle - 1)].count);

        if ( 0 == net_proxy_list[(*handle - 1)].v_count)
        {
            MS_PACK_BE_2_BYTE_VAL(&pdu[0], src_addr);
            /* Add the incoming Source Address to Filter */
            NET_TRC (
                "[PROXY]: Dropping FRIST Pkt processing for BRR_SERVER_ROLE for "
                "handle 0x%02X, src_addr 0x%04x\n", *handle,src_addr);
//          proxy_first_fitlter = MS_TRUE;
            net_proxy_server_add_to_list
            (
                handle,
                pdu,
                sizeof(pdu),
                MS_TRUE
            );
        }
        else
        {
            NET_ERR (
                "[PROXY]: Proxy Filter List already populated, list count 0x%02X "
                " for handle 0x%02X!!\n",
                net_proxy_list[(*handle - 1)].v_count, *handle);
        }
    }
    else
    {
        NET_ERR (
            "[PROXY]: Dropping FRIST Pkt processing for BRR_CLIENT_ROLE for "
            "handle 0x%02X!!\n", *handle);
    }

    /* UNLOCK */
    NET_TRC (
        "[PROXY]: << net_proxy_process_first_pkt with retval 0x%04X\n",
        retval);
    return retval;
}
#endif /* MS_PROXY_SERVER */

/**
    TODO: Should NETIF_PKT_T be passed on to this function
         to decide the destination addr based on
         NETIF_PKT_T_PROXY or NETIF_PKT_T_NETWORK ?
*/
API_RESULT net_proxy_filter_check_forwarding
(
    NETIF_HANDLE*        handle,
    NETIF_PKT_T          pkt_sub_type,
    PROXY_ADDR           d_addr
)
{
    #ifdef MS_PROXY_SERVER
    UINT32 i;
    #endif /* MS_PROXY_SERVER */
    API_RESULT retval;
    NET_TRC (
        "[PROXY]: >> net_proxy_filter_check_forwarding\n");
    #ifdef MS_PROXY_SERVER
    /* Initialize */
    retval = API_FAILURE;
    MS_IGNORE_UNUSED_PARAM(pkt_sub_type);
    NET_TRC (
        "[PROXY]: Request to search Proxy Filter for Handle 0x%02X for "
        "Destination Address 0x%04X\n", *handle, d_addr);

    if(d_addr == MS_NET_ADDR_ALL_NODES)
        return API_SUCCESS;

    #ifdef MS_PROXY_CLIENT

    if (BRR_CLIENT_ROLE == net_proxy_list[(*handle - 1)].role)
    {
        NET_INF (
            "[PROXY]: Dropping Proxy Filter Check for BRR_CLIENT_ROLE for handle 0x%02X!!\n",
            *handle);
        retval = API_SUCCESS;
    }
    else
    #endif /* MS_PROXY_CLIENT */
    {
        /**
            Check if the incoming Destination Address
            is part of the provided NetworkIface's Proxy Filter List.
            If the Proxy List associated with the provided Handle is
            a. MS_PROXY_WHITELIST_FILTER:
               If the d_addr matches any of the addr in the list
               then return API_SUCCESS otherwise return API_FAILURE.

            b. MS_PROXY_BLACKLIST_FILTER:
               If the d_addr matches any of the addr in the list
               then return API_FAILURE otherwise return API_SUCCESS.
        */
        /* Check if Destination Address is UNASSIGNED ADDRESS */
        if (MS_NET_ADDR_UNASSIGNED != d_addr)
        {
            /* Check if Address is present in Subnet's proxy list */
            for (i = 0; i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE); i++)
            {
                if ((net_proxy_list[(*handle - 1)].p_addr[i] == d_addr) || (net_proxy_list[(*handle - 1)].v_addr[i] == d_addr) )
                {
                    break;
                }
            }

            /* Check for the Filter type and assign the return value */
            if (MS_PROXY_WHITELIST_FILTER == net_proxy_list[(*handle - 1)].type)
            {
                retval = (i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE)) ? API_SUCCESS : API_FAILURE;
                NET_TRC (
                    "[PROXY]: Destination Address 0x%04X is %s in the WhiteList\n",
                    d_addr, (API_SUCCESS == retval) ? "Found!":"Not-Found!");
                /**
                    TODO:
                    1. Check if Filter Check Failure needs to be Informed
                      to the sever size application with a newer event for proxy.
                */
                /**
                    TODO:
                    Check for WILDCARD addresses after checking the WhiteList filters.
                */
            }
            else
            {
                /**
                    TODO:
                    Check for WILDCARD addresses before checking the Blacklist filters.
                */
                retval = (i < MS_CONFIG_LIMITS(MS_PROXY_FILTER_LIST_SIZE)) ? API_FAILURE : API_SUCCESS;
                NET_TRC (
                    "[PROXY]: Destination Address 0x%04X is %s in the BlackList\n",
                    d_addr, (API_FAILURE == retval) ? "Found!":"Not-Found!");
                /**
                    TODO:
                    1. Check if Filter Check Failure needs to be Informed
                      to the sever side application with a newer event
                      for proxy.
                */
            }
        }
        else
        {
            /* If Destination address is MS_NET_ADDR_UNASSIGNED */
            retval = API_SUCCESS;
        }
    }

    #else /* MS_PROXY_SERVER */
    MS_IGNORE_UNUSED_PARAM(handle);
    MS_IGNORE_UNUSED_PARAM(pkt_sub_type);
    MS_IGNORE_UNUSED_PARAM(d_addr);
    /**
        If only MS_PROXY_CLIENT is defined, the the forwarding
        need not be processed as filter for forwarding needs to be present
        only on the Server side.
    */
    retval = API_SUCCESS;
    #endif /* MS_PROXY_SERVER */
    NET_TRC (
        "[PROXY]: << net_proxy_filter_check_forwarding, retval 0x%04X\n", retval);
    return retval;
}

/* --------------------------------------------- API Functions */
/**
    \brief Register Interface with NETWORK PROXY Layer

    \par Description
    This routine registers interface with the NETWORK PROXY Layer.
    NETWORK PROXY Layer supports only one upper layer, hence this routine shall be called once.

    \param [in] proxy_cb
           Upper Layer Notification Callback

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_proxy_register
(
    /* IN */ PROXY_NTF_CB    proxy_cb
)
{
    /* LOCK */
    /* Register the callback */
    proxy_callback = proxy_cb;
    /* UNLOCK */
    return API_SUCCESS;
}

/**
    \brief Check if the Proxy Module is ready to handle Proxy Messages/Events

    \par Description
    This routine returns the current state of the Proxy. The valid states of
    proxy are:
    1. MS_PROXY_NULL      - If no callback registered by Upper Layers
    2. MS_PROXY_READY     - If callback registered and Proxy not connected
    3. MS_PROXY_CONNECTED - if callback registered and Proxy connected

    \param [out] proxy_state returns the current state of the Proxy

    \return API_SUCCESS or an error code indicating reason for failure
*/
API_RESULT MS_proxy_fetch_state (UCHAR* proxy_state)
{
    /**
        This Function Checks if application is ready to handle
        PROXY related messages.
        | Proxy    |  Proxy |  Error
        | Callback |  Iface |  Code
        |----------|--------|-------------------
        |  NULL    |  Down  | MS_PROXY_NULL
        |  NULL    |  Up    | MS_PROXY_NULL
        |  !NULL   |  Down  | MS_PROXY_READY
        |  !NULL   |  UP    | MS_PROXY_CONNECTED
    */
    /* NULL Check */
    if (NULL == proxy_state)
    {
        NET_ERR (
            "[PROXY]: Incoming Parameter NULL!\n");
        return API_FAILURE;
    }

    if (NULL == proxy_callback)
    {
        (*proxy_state) = MS_PROXY_NULL;
    }
    else
    {
        (*proxy_state) = (PROXY_IFACE_UP == proxy_iface_state) ? \
                         MS_PROXY_CONNECTED : MS_PROXY_READY;
    }

    NET_TRC (
        "[PROXY]: Proxy State is 0x%02X\n", (*proxy_state));
    return API_SUCCESS;
}

#ifdef MS_PROXY_CLIENT
/**
    \brief Set Proxy Server's Filter Type.

    \par Description This function is used by the Proxy Client
    to set the filter type on the Proxy Server.

    \param [in] handle Network Interface Handle
    \param [in] subnet_handle Subnet Handle
    \param [in] type Type of the Proxy Filter to be set. Either
                \ref MS_PROXY_WHITELIST_FILTER or
                \ref MS_PROXY_BLACKLIST_FILTER

    \note This API will be used by the Proxy Client only.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_proxy_set_filter
(
    /* IN */ NETIF_HANDLE*        handle,
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ PROXY_FILTER_TYPE  type
)
{
    UCHAR pdu[2];
    API_RESULT retval;
    MS_IGNORE_UNUSED_PARAM(handle);
    retval = API_SUCCESS;
    NET_TRC (
        "[PROXY]: >> MS_proxy_set_filter\n");
    /* TODO: Null checks and Validation */
    /* Search Filter Type and Filter Count Based on SubNet Handle */
    pdu[0] = MS_PROXY_SET_FILTER_OPCODE;
    pdu[1] = type;
    net_proxy_send
    (
        subnet_handle,
        pdu,
        2
    );
    NET_TRC (
        "[PROXY]: << MS_proxy_set_filter, retval 0x%04X\n", retval);
    return retval;
}

/**
    \brief Add or Delete/Remove addresses to/from Proxy Filter List.

    \par Description This function is used by the Proxy Client
    to add/delete Addresses to/from the Proxy Server's filter List.

    \param [in] handle Network Interface Handle
    \param [in] subnet_handle Subnet Handle
    \param [in] opcode Operation to be performed. Either
                \ref MS_PROXY_ADD_TO_FILTER_OPCODE or
                \ref MS_PROXY_REM_FROM_FILTER_OPCODE
    \param [in] addr  Pointer to List of Address to be added/deleted
    \param [in] addr_count  Count of Addresses present in the provided List

    \note This API will be used by the Proxy Client only.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_proxy_filter_op
(
    /* IN */ NETIF_HANDLE*        handle,
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ UCHAR              opcode,
    /* IN */ PROXY_ADDR*          addr,
    /* IN */ UINT16             addr_count
)
{
    API_RESULT retval;
    /* TODO: What should be the size of the PDU buffer */
    UCHAR      pdu[31];
    UINT16     marker;
    UINT32     index;
    MS_IGNORE_UNUSED_PARAM(handle);
    /* TODO: Validate Address List and its Count and opcode */
    retval = API_SUCCESS;
    marker = 0;
    NET_TRC (
        "[PROXY]: >> MS_proxy_filter_op\n");
    pdu[marker++] = opcode;

    for (index = 0; index < addr_count; index++)
    {
        MS_PACK_BE_2_BYTE_VAL
        (
            &pdu[marker],
            addr[index]
        );
        marker += 2;
    }

    net_proxy_send
    (
        subnet_handle,
        pdu,
        marker
    );
    NET_TRC (
        "[PROXY]: << MS_proxy_filter_op, retval 0x%04X\n", retval);
    return retval;
}
#endif /* MS_PROXY_CLIENT */

#ifdef MS_PROXY_SERVER
/* Function to Start ADV using Proxy */
/**
    \brief Start Connectable Advertisements for a Proxy Server.

    \par Description This function is used by the Proxy Server
    to start Connectable Undirected Advertisements.

    \param [in] subnet_handle  Subnet Handle which the Proxy Server is
                part of.
    \param [in] proxy_adv_mode Mode of Proxy Advertisements. This could
                be of two types
                \ref MS_PROXY_NET_ID_ADV_MODE or
                \ref MS_PROXY_NODE_ID_ADV_MODE

    \note This API will be used by the Proxy Server only.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_proxy_server_adv_start
(
    /* IN */ MS_SUBNET_HANDLE   subnet_handle,
    /* IN */ UCHAR              proxy_adv_mode
)
{
    API_RESULT retval;
    UINT16 handle;
    MS_IGNORE_UNUSED_PARAM(proxy_adv_mode);
    MS_IGNORE_UNUSED_PARAM(subnet_handle);
    retval = API_SUCCESS;
    NET_TRC (
        "[PROXY]: >> MS_proxy_server_adv_start\n");

    if (MS_PROXY_NET_ID_ADV_MODE == proxy_adv_mode)
    {
        UCHAR  net_id[8];

        if (EM_TIMER_HANDLE_INIT_VAL != net_proxy_netid_timer_handle)
        {
            NET_ERR (
                "[PROXY]: Proxy Adv on NET ID already in progress\n");
            return API_FAILURE;
        }

        /* Start proxy ADV on all active subnets starting with the first */
        handle = 0x0000;
        /* Get the Network ID from Access Layer */
        retval = MS_access_cm_get_subnet_network_id
                 (
                     handle,
                     net_id
                 );

        if (API_SUCCESS != retval)
        {
            NET_ERR (
                "[PROXY]: Failed to get Network ID for Primary SubNet 0x%04X\n",
                handle);
            return API_FAILURE;
        }

        NET_TRC (
            "[PROXY]: Starting PROXY ADV with 'NETWORK ID' mode for SubNet 0x%04X\n",
            handle);
        NET_TRC (
            "[PROXY]: Network ID for Subnet Handle 0x%04X is:\n", handle);
        NET_debug_dump_bytes(net_id, sizeof(net_id));
        /* Start the Proxy ADV */
        MS_brr_start_proxy_adv
        (
            BRR_BCON_TYPE_PROXY_NETID,
            net_id,
            sizeof(net_id)
        );
        /* Start timer to lookup the next available subnet */
        retval = EM_start_timer
                 (
                     &net_proxy_netid_timer_handle,
                     PROXY_SUBNET_NETID_ADV_TIMEOUT,
                     net_proxy_netid_timeout_handler,
                     &handle,
                     sizeof(handle)
                 );

        if (API_SUCCESS != retval)
        {
            NET_TRC(
                "[PROXY]: Failed to start timer. Retval - 0x%04X\n", retval);
        }

        /* Set Network ID proxy adv state */
        proxy_adv_state |= MS_PROXY_NET_ID_ADV_MODE;
    }
    else if (MS_PROXY_NODE_ID_ADV_MODE == proxy_adv_mode)
    {
        if (EM_TIMER_HANDLE_INIT_VAL != net_proxy_nodeid_timer_handle)
        {
            NET_ERR(
                "[PROXY]: Proxy Adv on NODE ID already in progress. Stop Timer.\n");
            /* Stop the timer */
            retval = EM_stop_timer(&net_proxy_nodeid_timer_handle);
        }

        NET_TRC (
            "[PROXY]: Starting PROXY ADV with 'NODE ID' mode for SubNet 0x%04X\n",
            subnet_handle);

        /* Is subnet specific or for all subnets? */
        if (MS_MAX_SUBNETS == subnet_handle)
        {
            NET_TRC(
                "[PROXY]: Node ID Advetisement requested for ALL SUBNETS\n");
            proxy_nodeid_all_subnets = MS_TRUE;
            handle = 0x0000;
        }
        else
        {
            proxy_nodeid_all_subnets = MS_FALSE;
            handle = subnet_handle;
            NET_TRC(
                "[PROXY]: Node ID Advetisement requested for Subnet 0x%04X\n", handle);
        }

        retval = net_proxy_nodeid_adv(handle);

        if (API_SUCCESS == retval)
        {
            if(proxy_nodeid_all_subnets == MS_TRUE)
            {
                /* Start timer to lookup the next available subnet */
                retval = EM_start_timer
                         (
                             &net_proxy_nodeid_timer_handle,
                             PROXY_SUBNET_NODEID_ADV_TIMEOUT,
                             net_proxy_nodeid_timeout_handler,
                             &handle,
                             sizeof(handle)
                         );

                if (API_SUCCESS != retval)
                {
                    NET_ERR(
                        "[PROXY]: Failed to start timer. Retval - 0x%04X\n", retval);
                }
            }

            /* Set Node ID proxy adv state */
            proxy_adv_state |= MS_PROXY_NODE_ID_ADV_MODE;
        }
    }

    NET_TRC (
        "[PROXY]: << MS_proxy_server_adv_start, retval 0x%04X\n", retval);
    return retval;
}

API_RESULT MS_proxy_server_stop_timer(void)
{
    EM_stop_timer(&net_proxy_nodeid_timer_handle);
    EM_stop_timer(&net_proxy_netid_timer_handle);
}

/**
    \brief Stop Connectable Advertisements for a Proxy Server.

    \par Description This function is used by the Proxy Server
    to stop Connectable Undirected Advertisements.

    \note This API will be used by the Proxy Server only.

    \return API_SUCCESS or Error Code on failure
*/
API_RESULT MS_proxy_server_adv_stop (void)
{
    API_RESULT retval;
    retval = API_SUCCESS;
    NET_TRC (
        "[PROXY]: >> MS_proxy_server_adv_stop\n");

    /**
        The Proxy ADV Mode here is Don't Care.
        Hence, using default value of mode as MS_PROXY_NET_ID_ADV_MODE.
    */
    if (MS_PROXY_NET_ID_ADV_MODE & proxy_adv_state)
    {
        MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NETID, BRR_BCON_ACTIVE);
        proxy_adv_state &= (UCHAR)(~MS_PROXY_NET_ID_ADV_MODE);
    }

    if (MS_PROXY_NODE_ID_ADV_MODE & proxy_adv_state)
    {
        MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NODEID, BRR_BCON_ACTIVE);
        proxy_adv_state &= (UCHAR)(~MS_PROXY_NODE_ID_ADV_MODE);
    }

    /**
        TODO: Check if newer Beacon Type needs to be introduced
        for GATT PROXY Connectable ADV.
    */
    NET_TRC (
        "[PROXY]: << MS_proxy_server_adv_stop, retval 0x%04X\n", retval);
    return retval;
}


API_RESULT net_proxy_nodeid_adv(MS_SUBNET_HANDLE handle)
{
    UCHAR*        adv_rand;
    UCHAR       in_pdu[16];
    /**
        NOTE: [TODO] The following two buffers along with adv_rand,
        can be made into a single buffer of 24 octets and used.
    */
    UCHAR       out_pdu[16];
    UCHAR       t_out_pdu[16];
    UCHAR       id_key[16];
    UCHAR       marker;
    INT32       ret;
    MS_NET_ADDR pri_addr;
    API_RESULT retval;
    NET_TRC(
        "[PROXY]: Starting PROXY ADV with 'NODE IDENTITY' mode for SubNet 0x%04X\n",
        handle);
    /* Initialize the local Variables */
    EM_mem_set(in_pdu, 0x0, sizeof(in_pdu));
    EM_mem_set(out_pdu, 0x0, sizeof(out_pdu));
    EM_mem_set(t_out_pdu, 0x0, sizeof(t_out_pdu));
    EM_mem_set(id_key, 0x0, sizeof(id_key));
    adv_rand = NULL;
    marker = 0;
    pri_addr = MS_NET_ADDR_UNASSIGNED;
    /* Assign the output buffer Random Value Location */
    adv_rand = &out_pdu[8];
    /* Get Primary Unicast Address */
    MS_access_cm_get_primary_unicast_address(&pri_addr);
    /* Get the Identity Key */
    retval = MS_access_cm_get_subnet_identity_key
             (
                 handle,
                 id_key
             );

    if (API_SUCCESS != retval)
    {
        NET_TRC(
            "[PROXY]: Failed to get Net Identity for Subnet 0x%04X. Retval - 0x%04X\n",
            handle, retval);
        return retval;
    }

    NET_TRC(
        "[PROXY]: Identity Key for Subnet Handle 0x%04X is:\n", handle);
    NET_debug_dump_bytes(id_key, sizeof(id_key));
    /* Get the 8 Bytes Random Number */
    cry_rand_generate(adv_rand, 8);
    NET_TRC(
        "[PROXY]: Generated Random Value is:\n");
    NET_debug_dump_bytes(adv_rand, 8);
    /* Initialize the Input Buffer */
    EM_mem_set(in_pdu, 0x0, sizeof(in_pdu));
    /* Initialize Marker: First 6 Bytes of ZERO padding */
    marker = 6;
    /* Copy the Random Number in next 8 bytes */
    EM_mem_copy(&in_pdu[marker], adv_rand, 8);
    marker += 8;
    /* Copy the 2 Bytes of Unicast Addr of the Node */
    MS_PACK_BE_2_BYTE_VAL(&in_pdu[marker], pri_addr);
    marker += 2;
    NET_TRC(
        "[PROXY]: Input for generating Proxy ADV Hash for "
        "Subnet Handle 0x%04X with Unicast Address 0x%04X is:\n",
        handle, pri_addr);
    NET_debug_dump_bytes(in_pdu, sizeof(in_pdu));
    /* Generate the Hash */
    cry_aes_128_encrypt_be
    (
        in_pdu,
        id_key,
        t_out_pdu,
        ret
    );

    if (0 > ret)
    {
        NET_ERR(
            "[PROXY]: Node Identity Hash generation Failed with %d for Subnet Handle 0x%04X\n",
            ret, handle);
        return API_FAILURE;
    }

    /* Stop the Proxy ADV */
    MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NODEID, BRR_BCON_ACTIVE);
    /* Copy Last 8 Bytes from Temp Output as First 8 Bytes of Output */
    EM_mem_copy(out_pdu, t_out_pdu + 8, 8);
    NET_TRC(
        "[PROXY]: Generated Hash Value is:\n");
    NET_debug_dump_bytes(out_pdu, 8);
    NET_TRC(
        "[PROXY]: Node Identity Proxy ADV Data for 0x%04X is:\n", handle);
    NET_debug_dump_bytes(out_pdu, sizeof(out_pdu));
    /* Start the Proxy ADV */
    MS_brr_start_proxy_adv
    (
        BRR_BCON_TYPE_PROXY_NODEID,
        out_pdu,
        sizeof(out_pdu)
    );
    return API_SUCCESS;
}

void net_proxy_netid_timeout_handler(void* args, UINT16 size)
{
    UCHAR  net_id[8];
    API_RESULT retval;
    UINT16 handle;
    MS_IGNORE_UNUSED_PARAM(size);
    net_proxy_netid_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    handle = (UINT16)(*((UINT16*)args));
    NET_TRC(
        "[PROXY]: NetID Timeout for SubNet 0x%04X\n", handle);

    if (!(proxy_adv_state & MS_PROXY_NET_ID_ADV_MODE))
    {
        return;
    }

    handle++;

    /* Wrap around subnet handle */
    if (MS_MAX_SUBNETS == handle)
    {
        handle = 0x0000;
    }

    /* Get the Network ID from Access Layer */
    retval = MS_access_cm_get_subnet_network_id
             (
                 handle,
                 net_id
             );

    /**
        TODO:
        1. See if the logic can be refined here.
          Currently, if there is only one active SUBNET, we still go and stop
          the ongoing ADV and Restart it.
        2. Check if NETWORK ID Adv also needs to a Timeout like NODE IDENTITY as
          once this is started, it keeps ongoing for ever unless UpperLayer
          calls STOP.
    */

    if (API_SUCCESS == retval)
    {
        NET_TRC(
            "[PROXY]: Stopping PROXY ADV with 'NETWORK ID' mode for SubNet 0x%04X\n",
            (0x0000 == handle) ? (MS_MAX_SUBNETS - 1) : (handle - 1));
        /* Stop the Proxy ADV */
        MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NETID, BRR_BCON_ACTIVE);
        NET_TRC(
            "[PROXY]: Starting PROXY ADV with 'NETWORK ID' mode for SubNet 0x%04X\n",
            handle);
        NET_TRC(
            "[PROXY]: Network ID for Subnet Handle 0x%04X is:\n", handle);
        NET_debug_dump_bytes(net_id, sizeof(net_id));
        /* Start the Proxy ADV */
        MS_brr_start_proxy_adv
        (
            BRR_BCON_TYPE_PROXY_NETID,
            net_id,
            sizeof(net_id)
        );
    }

    /* Start timer to lookup the next available subnet */
    retval = EM_start_timer
             (
                 &net_proxy_netid_timer_handle,
                 PROXY_SUBNET_NETID_ADV_TIMEOUT,
                 net_proxy_netid_timeout_handler,
                 &handle,
                 sizeof(handle)
             );

    if (API_SUCCESS != retval)
    {
        NET_TRC(
            "[PROXY]: Failed to start timer. Retval - 0x%04X\n", retval);
    }
}


void net_proxy_nodeid_timeout_handler(void* args, UINT16 size)
{
    API_RESULT retval;
    UINT16 handle;
    MS_IGNORE_UNUSED_PARAM(size);
    net_proxy_nodeid_timer_handle = EM_TIMER_HANDLE_INIT_VAL;
    handle = (UINT16)(*((UINT16*)args));
    NET_INF(
        "[PROXY]: Node ID Advertisement timeout for Subnet 0x%04X\n", handle);

    if (!(proxy_adv_state & MS_PROXY_NODE_ID_ADV_MODE))
    {
        NET_ERR(
            "[PROXY-ERR]: Invalid Proxy Adv State 0x%02X in net_proxy_nodeid_timeout_handler\n",
            proxy_adv_state);
        return;
    }

    #if 0
    proxy_nodeid_timeout += (UINT32)PROXY_SUBNET_NODEID_ADV_TIMEOUT;

    if (PROXY_NODEID_ADV_TIMEOUT == proxy_nodeid_timeout)
    {
        NET_TRC(
            "[PROXY]: Node ID Advertisement timeout. Stopping procedure\n");
        proxy_nodeid_timeout = 0;
        /* Stop the Proxy ADV */
        MS_brr_bcast_end(BRR_BCON_TYPE_PROXY_NETID, BRR_BCON_ACTIVE);
        proxy_adv_state &= (UCHAR)(~MS_PROXY_NODE_ID_ADV_MODE);
        return;
    }

    #endif

    if (MS_TRUE == proxy_nodeid_all_subnets)
    {
        handle++;

        /* Wrap around subnet handle */
        if (MS_MAX_SUBNETS == handle)
        {
            handle = 0x0000;
        }

        retval = net_proxy_nodeid_adv(handle);
    }
    else
    {
        retval = API_SUCCESS;
    }

    /* Start timer to lookup the next available subnet */
    retval = EM_start_timer
             (
                 &net_proxy_nodeid_timer_handle,
                 PROXY_SUBNET_NODEID_ADV_TIMEOUT,
                 net_proxy_nodeid_timeout_handler,
                 &handle,
                 sizeof(handle)
             );

    if (API_SUCCESS != retval)
    {
        NET_TRC(
            "[PROXY]: Failed to start timer. Retval - 0x%04X\n", retval);
    }
}

#endif /* MS_PROXY_SERVER */
#endif /* MS_PROXY_SUPPORT */

