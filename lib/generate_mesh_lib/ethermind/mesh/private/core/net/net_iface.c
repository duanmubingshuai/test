
/**
    \file net_iface.c


*/

/*
    Copyright (C) 2017. Mindtree Limited.
    All rights reserved.
*/

/* --------------------------------------------- Header File Inclusion */
#include "net_internal.h"

/* --------------------------------------------- External Global Variables */

/* --------------------------------------------- Exported Global Variables */

/* --------------------------------------------- Static Global Variables */
/**
    Supported number of Network Interface handles to the bearer
    - Index = 0: Advertising Bearer handle
    - Index > 0: GATT Bearer handle
*/
DECL_STATIC MS_DEFINE_GLOBAL_ARRAY(BRR_HANDLE, netif_brr_handle, MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES));

/* --------------------------------------------- Functions */
API_RESULT netif_init (void)
{
    API_RESULT retval;
    NET_TRC ("NetIF registering Advertising Interface...\n");
    /* Register with the required bearers */
    retval = MS_brr_register(BRR_TYPE_ADV, netif_adv_recv_cb);

    if (API_SUCCESS != retval)
    {
        NET_ERR ("Failed - 0x%04X\n", retval);
    }

    NET_TRC ("NetIF registering GATT Interface...\n");
    /* Register with the required bearers */
    retval = MS_brr_register(BRR_TYPE_GATT, netif_gatt_recv_cb);

    if (API_SUCCESS != retval)
    {
        NET_ERR ("Failed - 0x%04X\n", retval);
    }

    /* Initialize Bearer Handles */
    MS_INIT_GLOBAL_ARRAY(BRR_HANDLE, netif_brr_handle, MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES), BRR_HANDLE_INVALID);
    return retval;
}

API_RESULT netif_deinit (void)
{
    /* Unregister network bearers */
    return API_SUCCESS;
}

API_RESULT netif_send (NETIF_PKT_T type, MS_NET_ADDR d_addr, UCHAR* pdata, UINT16 pdatalen,UINT8 unsegment_flag,UINT8 tx_flag)
{
    API_RESULT retval;
    UINT32     index;
    MS_BUFFER  buffer;
    /*
        Check if the packet is for the local interface (How to check? TBD).
        If yes, trigger callback on to the network layer.

        If No, check with the configured output filter for each enabled
        interface before transmitting the packet.

        In case of Adv Interface, transmit on bearer handle in index 0.
        - if the packet to be transmitted is not a relay packet,
         then have retransmission of the Network PDU using the value of
         the Network Transmit state
        - if the packet to be transmitted is a relay packet,
         then have retransmission of the Network PDU using the value of
         the Relay Retransmit state

        In case of GATT Interface, transmit on all valid bearers from 1 to
        (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) - 1)
    */
    /* First check for ADV Bearer */
    buffer.payload = pdata;
    buffer.length = pdatalen;
    retval = API_FAILURE;

    if (BRR_HANDLE_INVALID != netif_brr_handle[0])
    {
        /**
            TODO: The Proxy Configuration packets SHALL not be sent of the
            ADV bearer. Need to add check to filter out based on the
            NETIF_PKT_T types.
        */
        retval = MS_brr_send_pdu
                 (
                     &netif_brr_handle[0],
                     BRR_TYPE_ADV,
                     &buffer
                 );
        NET_TRC(
            "[netIF 0] Sent PDU over ADV Bearer. Result 0x%04X\n", retval);
//        printf(
//            "[netIF 0] Sent PDU over ADV Bearer. Result 0x%04X\n", retval);
    }

    #ifdef MS_PROXY_SUPPORT

    /* Now for rest of the GATT Bearers */
    for (index = 1; index < MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES); index++)
    {
        if ((BRR_HANDLE_INVALID != netif_brr_handle[index])&&
                (!unsegment_flag || (unsegment_flag && tx_flag)) && (NETIF_PKT_T_PROXY != type))
        {
            BRR_TYPE b_type;
            b_type = (NETIF_PKT_T_PROXY == type) ? BRR_SUBTYPE_GATT_PROXY_T_MASK : BRR_SUBTYPE_GATT_NETWORK_T_MASK;
            NET_TRC(
                "[netIF 0x%02X] Checking Proxy Filter for SubType 0x%02X, Dest Addr 0x%04X\n",
                index, b_type, d_addr);
            /**
                TODO: Check if b_type should be passed, else only
                BRR_SUBTYPE_GATT_NETWORK_T_MAS needs filtering.
            */
            /**
                "net_proxy_filter_check_forwarding" returns API_SUCCESS if the
                provided Packet is suitable for transmission after checking with
                the corresponding 'Filters'.
            */
            /**
                TODO: This SHALL not be checked for a netif handle which corresponds
                to a PROXY CLIENT role.
            */
            retval = net_proxy_filter_check_forwarding
                     (
                         (NETIF_HANDLE*)&index,
                         b_type,
                         d_addr
                     );
            NET_TRC(
                "[netIF 0x%02X] net_proxy_filter_check_forwarding, retval 0x%04X,d_addr 0x%04x\n",
                index, retval,d_addr);

            if (API_SUCCESS == retval)
            {
                NET_TRC("[netIF] MS_brr_send_pdu success\n");
                retval = MS_brr_send_pdu
                         (
                             &netif_brr_handle[index],
                             (BRR_TYPE_GATT | b_type),
                             &buffer
                         );

                if(retval)
                {
//                    printf(
//                        "[netIF %d] Sent PDU over GATT Bearer. Result 0x%04X\n", index, retval);
                    return retval;
                }
            }
            else
                retval = API_SUCCESS;

            NET_TRC(
                "[netIF %d] Sent PDU over GATT Bearer. Result 0x%04X\n", index, retval);
//            printf(
//                "[netIF %d] Sent PDU over GATT Bearer. Result 0x%04X\n", index, retval);
        }
    }

    #else /* MS_PROXY_SUPPORT */
    MS_IGNORE_UNUSED_PARAM(d_addr);
    MS_IGNORE_UNUSED_PARAM(type);
    MS_IGNORE_UNUSED_PARAM(index);
    #endif /* MS_PROXY_SUPPORT */
//    if (BRR_HANDLE_INVALID != netif_brr_handle[0])
//    {
//        /**
//            TODO: The Proxy Configuration packets SHALL not be sent of the
//            ADV bearer. Need to add check to filter out based on the
//            NETIF_PKT_T types.
//        */
//        retval = MS_brr_send_pdu
//                 (
//                     &netif_brr_handle[0],
//                     BRR_TYPE_ADV,
//                     &buffer
//                 );
//        NET_TRC(
//            "[netIF 0] Sent PDU over ADV Bearer. Result 0x%04X\n", retval);
//    }
    return retval;
}

API_RESULT netif_adv_recv_cb (BRR_HANDLE* handle, UCHAR pevent, UCHAR* pdata, UINT16 pdatalen)
{
    API_RESULT retval;
    retval = API_SUCCESS;

    /* Get the event */
    switch (pevent)
    {
    case BRR_IFACE_UP:

        /*
            Set the handle of the ADV interface (index 0) in the
            interface list
        */
        if (BRR_HANDLE_INVALID == netif_brr_handle[0])
        {
            netif_brr_handle[0] = *handle;
        }
        else
        {
            retval = API_FAILURE;
        }

        break;

    case BRR_IFACE_DOWN:

        /*
            Reset the handle of the ADV interface (index 0) in the
            interface list
        */
        if (*handle == netif_brr_handle[0])
        {
            netif_brr_handle[0] = BRR_HANDLE_INVALID;
        }
        else
        {
            retval = API_FAILURE;
        }

        break;

    case BRR_IFACE_DATA:

        /* Drop the data if mismatch in handle */
        if (*handle != netif_brr_handle[0])
        {
            break;
        }

        /**
            Check with the configured input filter for the advertising interface before
            forwarding the packet to the network layer
        */
        net_pkt_in(NETIF_PKT_T_NETWORK, NULL, pdata, pdatalen);
        break;

    default:
        break;
    }

    return retval;
}

API_RESULT netif_gatt_recv_cb (BRR_HANDLE* handle, UCHAR pevent, UCHAR* pdata, UINT16 pdatalen)
{
    #ifdef MS_PROXY_SUPPORT
    API_RESULT retval;
    UCHAR i;
    retval = API_SUCCESS;

    /* Get the event */
    switch (pevent)
    {
    case BRR_IFACE_UP:
        /* End beaconing */
        /* TODO: Check if here or Proxy module */
        #ifdef MS_PROXY_SERVER
        MS_proxy_server_adv_stop();
        #endif /* MS_PROXY_SERVER */

        /*
            Allocate a free interface from the list and set the handle
            for the GATT interface
        */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES); i++)
        {
            if (BRR_HANDLE_INVALID == netif_brr_handle[i])
            {
                netif_brr_handle[i] = *handle;
                /* Informing Interface UP to Proxy Layer */
                net_proxy_iface_up(&i, ((BRR_BEARER_CH_INFO*)pdata)->role);
                break;
            }
        }

        if (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) == i)
        {
            retval = API_FAILURE;
        }

        break;

    case BRR_IFACE_DOWN:

        /*
            Reset the handle of the GATT interface corresponding to
            the handle in interface list
        */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES); i++)
        {
            if (*handle == netif_brr_handle[i])
            {
                netif_brr_handle[i] = BRR_HANDLE_INVALID;
                /* Informing Interface Down to Proxy Layer */
                net_proxy_iface_down(&i);
                break;
            }
        }

        if (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) == i)
        {
            retval = API_FAILURE;
        }

        break;

    case BRR_IFACE_DATA:

        /* Drop if no handle matches */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES); i++)
        {
            if (*handle == netif_brr_handle[i])
            {
                break;
            }
        }

        if (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) == i)
        {
            retval = API_FAILURE;
            break;
        }

        /**
            check with the configured input filter for the gatt interface before
            forwarding the packet to the network layer
        */
        net_pkt_in(NETIF_PKT_T_NETWORK, &i, pdata, pdatalen);
        break;

    case BRR_IFACE_PROXY_DATA:

        /* Drop if no handle matches */
        for (i = 1; i < MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES); i++)
        {
            if (*handle == netif_brr_handle[i])
            {
                break;
            }
        }

        if (MS_CONFIG_LIMITS(MS_NUM_NETWORK_INTERFACES) == i)
        {
            retval = API_FAILURE;
            break;
        }

        /* TODO: Handle Proxy data here */
        net_pkt_in(NETIF_PKT_T_PROXY, &i, pdata, pdatalen);
        break;

    default:
        break;
    }

    return retval;
    #else /* MS_PROXY_SUPPORT */
    MS_IGNORE_UNUSED_PARAM(pdatalen);
    MS_IGNORE_UNUSED_PARAM(pdata);
    MS_IGNORE_UNUSED_PARAM(pevent);
    NET_ERR(
        "[netIF] Proxy Support Currently Not Enbled!\n");
    NET_ERR(
        "[netIF] Network GATT Bearer Unavailable for Bearer Handle 0x%02X\n", *handle);
    return API_FAILURE;
    #endif /* MS_PROXY_SUPPORT */
}

