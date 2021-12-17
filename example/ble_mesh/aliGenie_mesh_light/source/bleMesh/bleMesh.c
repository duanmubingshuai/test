/**************************************************************************************************

    Phyplus Microelectronics Limited confidential and proprietary.
    All rights reserved.

    IMPORTANT: All rights of this software belong to Phyplus Microelectronics
    Limited ("Phyplus"). Your use of this Software is limited to those
    specific rights granted under  the terms of the business contract, the
    confidential agreement, the non-disclosure agreement and any other forms
    of agreements as a customer or a partner of Phyplus. You may not use this
    Software unless you agree to abide by the terms of these agreements.
    You acknowledge that the Software may not be modified, copied,
    distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy
    (BLE) integrated circuit, either as a product or is integrated into your
    products.  Other than for the aforementioned purposes, you may not use,
    reproduce, copy, prepare derivative works of, modify, distribute, perform,
    display or sell this Software and/or its documentation for any purposes.

    YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
    PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
    INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
    NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
    PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
    NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
    LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
    OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
    OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

**************************************************************************************************/

/**************************************************************************************************
    Filename:       bleMesh.c
    Revised:
    Revision:

    Description:    This file contains the Simple BLE Peripheral sample application


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "rf_phy_driver.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "hci.h"
#include "hci_tl.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile_ota.h"

#include "peripheral.h"
#include "gapbondmgr.h"

#include "bleMesh.h"
#include "uart.h"
#include "sha256.h"
#include "ali_genie_profile.h"
#include "led_light.h"

#ifdef ETHERMIND
    #include "EM_os.h"
    #include "EM_debug.h"
    #include "EM_timer.h"

    #include "MS_common.h"
    #include "MS_prov_api.h"

    #include "blebrr.h"

    #include "cliface.h"

    #include "mesh_clients.h"
    #include "flash.h"
    extern void appl_mesh_sample (void);
    extern void appl_dump_bytes(UCHAR* buffer, UINT16 length);
#endif /* ETHERMIND */

#define BLEMESH_START_DEVICE_EVT            0x0001
#define BLEMESH_UART_RX_EVT                 0x0002
#define BLEMESH_GAP_SCANENABLED             0x0004
#define BLEMESH_ECDH_PROCESS                0x0008
#define BLEMESH_LIGHT_PRCESS_EVT            0x0010

#define ON_OFF_RESET_FLASH_ADDR             0x11003000


/*********************************************************************
    MACROS
*/

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/

UCHAR cmdstr[64];
UCHAR cmdlen;

API_RESULT cli_internal_status(UINT32 argc, UCHAR* argv[]);

DECL_CONST CLI_COMMAND cli_cmd_list[] =
{
    /* internal status */
    { "status", "internal status", cli_internal_status }
};

/*********************************************************************
    EXTERNAL VARIABLES
*/
extern PROV_DEVICE_S UI_lprov_device;


/*********************************************************************
    EXTERNAL FUNCTIONS
*/

extern API_RESULT blebrr_handle_le_connection_pl
(
    uint16_t  conn_idx,
    uint16_t  conn_hndl,
    uint8_t   peer_addr_type,
    uint8_t*    peer_addr
);

extern API_RESULT blebrr_handle_le_disconnection_pl
(
    uint16_t  conn_idx,
    uint16_t  conn_hndl,
    uint8_t   reason
);

extern void blebrr_handle_evt_adv_complete (UINT8 enable);
extern void blebrr_handle_evt_adv_report (gapDeviceInfoEvent_t* adv);
extern void blebrr_handle_evt_scan_complete (UINT8 enable);

/*********************************************************************
    LOCAL VARIABLES
*/
uint8 bleMesh_TaskID;   // Task ID for internal task/event processing
uint8       g_dev_reset_flg;



// GAP - SCAN RSP data (max size = 31 bytes)
//static uint8 scanRspData[B_MAX_ADV_LEN];

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PHYPLUS ALI MSH";

static gapDevDiscReq_t bleMesh_scanparam;
static gapAdvertisingParams_t bleMesh_advparam;

static UCHAR bleMesh_DiscCancel = FALSE;             // HZF�� not use???

/*********************************************************************
    LOCAL FUNCTIONS
*/
static void bleMesh_ProcessOSALMsg( osal_event_hdr_t* pMsg );
static void bleMesh_ProcessGAPMsg( gapEventHdr_t* pMsg );
static void bleMesh_ProcessL2CAPMsg( gapEventHdr_t* pMsg );
static void bleMesh_ProcessGATTMsg( gattMsgEvent_t* pMsg );
static void bleMesh_uart_init(void);

void UI_set_uuid_octet (UCHAR uuid_0);
void on_off_reset_process(void);

/*********************************************************************
    PROFILE CALLBACKS
*/

/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      bleMesh_Init

    @brief   Initialization function for the Simple BLE Peripheral App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
void bleMesh_Init( uint8 task_id )
{
//    bStatus_t ret;
    bleMesh_TaskID = task_id;
    // Register for direct HCI messages
    //HCI_GAPTaskRegister(bleMesh_TaskID);
    GAP_ParamsInit (bleMesh_TaskID, (GAP_PROFILE_PERIPHERAL | GAP_PROFILE_CENTRAL));
    /* printf ("Ret - %02x\r\n", ret); */
    GAP_CentDevMgrInit(0x80);
    /* printf ("Ret - %02x\r\n", ret); */
    GAP_PeriDevMgrInit();
    /* printf ("Ret - %02x\r\n", ret); */
    GAP_CentConnRegister();
    #if 0
    uint8 enable_update_request;
    printf ("Disable Conn Update...\r\n");
    enable_update_request = FALSE;
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    #endif /* 0 */
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    GATTServApp_RegisterForMsg(task_id);
    bleMesh_uart_init();
    TRNG_INIT();
    // For DLE
    llInitFeatureSet2MPHY(TRUE);
    llInitFeatureSetDLE(TRUE);
    // HCI_LE_SetDefaultPhyMode(0,0,0x01,0x01);
    on_off_reset_process();
    osal_set_event( bleMesh_TaskID, BLEMESH_START_DEVICE_EVT );
}


/*********************************************************************
    @fn      bleMesh_ProcessEvent

    @brief   Simple BLE Peripheral Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/

#define GPIO_GREEN      P32
#define GPIO_BLUE       P23
#define GPIO_RED        P31
static gpio_pin_e led_pins[3] = {GPIO_GREEN,GPIO_BLUE,GPIO_RED};
uint16 bleMesh_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function
//    hciStatus_t ret;

    /* printf("\r\n >> bleMesh_ProcessEvent- TaskID %d : events 0x%04X\r\n", task_id, events); */

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( bleMesh_TaskID )) != NULL )
        {
            bleMesh_ProcessOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if(events & BLEMESH_LIGHT_PRCESS_EVT)
    {
        light_blink_porcess_evt();
        return (events ^ BLEMESH_LIGHT_PRCESS_EVT);
    }

    if ( events & BLEMESH_START_DEVICE_EVT )
    {
        /* Register for GATT Client */
        GATT_InitClient();
        GATT_RegisterForInd(bleMesh_TaskID);
        light_init(led_pins,3);
        light_blink_evt_cfg(bleMesh_TaskID,BLEMESH_LIGHT_PRCESS_EVT);
        LIGHT_ONLY_RED_ON;
        gen_aligenie_auth_val();
        CLI_init((CLI_COMMAND*)cli_cmd_list,(sizeof (cli_cmd_list)/sizeof(CLI_COMMAND)));
        appl_mesh_sample();
        return ( events ^ BLEMESH_START_DEVICE_EVT );
    }

    if (events & BLEMESH_UART_RX_EVT)
    {
        if ('\r' == cmdstr[cmdlen - 1])
        {
            cmdstr[cmdlen - 1] = '\0';
            printf("%s", cmdstr);
            CLI_process_line
            (
                cmdstr,
                cmdlen
            );
            cmdlen = 0;
        }

        return ( events ^ BLEMESH_UART_RX_EVT );
    }

    if (events & BLEMESH_GAP_SCANENABLED)
    {
        blebrr_handle_evt_scan_complete(0x01);
        return (events ^ BLEMESH_GAP_SCANENABLED);
    }

    if (events & BLEMESH_ECDH_PROCESS)
    {
//      cry_ecdh_process_secret();
        return (events ^ BLEMESH_ECDH_PROCESS);
    }

    // Discard unknown events
    return 0;
}

/*********************************************************************
    @fn      bleMesh_ProcessOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void bleMesh_ProcessOSALMsg( osal_event_hdr_t* pMsg )
{
    switch ( pMsg->event )
    {
    case GAP_MSG_EVENT:
        bleMesh_ProcessGAPMsg( (gapEventHdr_t*)pMsg );
        break;

    case L2CAP_SIGNAL_EVENT:
        bleMesh_ProcessL2CAPMsg( (gapEventHdr_t*)pMsg );
        break;

    case GATT_MSG_EVENT:
        bleMesh_ProcessGATTMsg( (gattMsgEvent_t*)pMsg );
        /* Invoke the Mesh Client GATT Msg Handler */
        mesh_client_process_gattMsg((gattMsgEvent_t*)pMsg, bleMesh_TaskID);
        break;

    default:
        break;
    }
}

static void bleMesh_ProcessGATTMsg( gattMsgEvent_t* pMsg )
{
//    bStatus_t ret;

    // Process the GATT server message
    switch ( pMsg->method )
    {
    case ATT_EXCHANGE_MTU_RSP:
        break;

    default:
        break;
    }
}

static void bleMesh_ProcessL2CAPMsg( gapEventHdr_t* pMsg )
{
    l2capSignalEvent_t* pPkt = (l2capSignalEvent_t*)pMsg;

    // Process the Parameter Update Response
    if ( pPkt->opcode == L2CAP_PARAM_UPDATE_RSP )
    {
        l2capParamUpdateRsp_t* pRsp = (l2capParamUpdateRsp_t*)&(pPkt->cmd.updateRsp);

        if (pRsp->result == L2CAP_CONN_PARAMS_ACCEPTED )
        {
            printf ("L2CAP Connection Parameter Updated!\r\n");
        }
    }
}

static void bleMesh_ProcessGAPMsg( gapEventHdr_t* pMsg )
{
    hciStatus_t ret;
    gapDeviceInfoEvent_t* dev;

    if ((pMsg->hdr.status != SUCCESS)
            && (pMsg->hdr.status != bleGAPUserCanceled || pMsg->opcode != GAP_DEVICE_DISCOVERY_EVENT))
        printf ("GAP Event - %02X, status = %X\r\n", pMsg->opcode, pMsg->hdr.status);

    switch (pMsg->opcode)
    {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
        gapDeviceInitDoneEvent_t* pPkt = (gapDeviceInitDoneEvent_t*) pMsg;

        if ( pPkt->hdr.status == SUCCESS )
        {
            // Save BD Address information in pPkt->devAddr for B_ADDR_LEN
        }
    }
    break;

    case GAP_MAKE_DISCOVERABLE_DONE_EVENT:

        //printf ("->%d\r\n", pMsg->opcode);
        //printf ("MD Status - %d", pMsg->hdr.status);
        if (SUCCESS != pMsg->hdr.status)
        {
            //printf ("MDReq\r\n");
            ret = GAP_MakeDiscoverable(bleMesh_TaskID, &bleMesh_advparam);

            if (SUCCESS != ret)
            {
                printf ("GAP_MakeDiscoverable Failed - %d\r\n", ret);
            }
        }
        else
        {
            blebrr_handle_evt_adv_complete(1);
//                blebrr_pl_advertise_setup(1);
        }

        break;

    case GAP_END_DISCOVERABLE_DONE_EVENT:

        //printf ("->%d\r\n", pMsg->opcode);
        //printf ("ED Status - %d", pMsg->hdr.status);
        if (SUCCESS != pMsg->hdr.status)
        {
            //printf ("EDReq\r\n");
            ret = GAP_EndDiscoverable(bleMesh_TaskID);

            if (SUCCESS != ret)
            {
                printf ("GAP_EndDiscoverable Failed - %d\r\n", ret);
            }
        }
        else
        {
            blebrr_handle_evt_adv_complete(0);
//                blebrr_pl_advertise_setup(0);
        }

        break;

    case GAP_DEVICE_DISCOVERY_EVENT:

//            printf("\r\nIn GAP_DEVICE_DISCOVERY_EVENT...\r\n");
        if (TRUE == bleMesh_DiscCancel)
        {
            if (bleGAPUserCanceled != pMsg->hdr.status)
            {
                GAP_DeviceDiscoveryCancel(bleMesh_TaskID);
            }
            else
            {
                bleMesh_DiscCancel = FALSE;
                blebrr_handle_evt_scan_complete(0);
            }
        }
        else
        {
//                printf (">>> %d\r\n", pMsg->hdr.status);
            if (SUCCESS == pMsg->hdr.status)
            {
                //printf (">>> 2\r\n");
                GAP_DeviceDiscoveryRequest(&bleMesh_scanparam);
            }
        }

        break;

    case GAP_DEVICE_INFO_EVENT:
        dev = (gapDeviceInfoEvent_t*)pMsg;
        blebrr_handle_evt_adv_report(dev);
        break;

    case GAP_LINK_ESTABLISHED_EVENT:
    {
        gapEstLinkReqEvent_t* pPkt = (gapEstLinkReqEvent_t*)pMsg;
        printf("\r\n GAP_LINK_ESTABLISHED_EVENT received! \r\n");

        if ( pPkt->hdr.status == SUCCESS )
        {
            /* Send Connection Complete to  Ble Bearer PL layer */
            blebrr_handle_le_connection_pl
            (
                0x00, /* Dummy Static Connection Index */
                pPkt->connectionHandle,
                pPkt->devAddrType,
                pPkt->devAddr
            );
        }
    }
    break;

    case GAP_LINK_TERMINATED_EVENT:
    {
        gapTerminateLinkEvent_t* pPkt = (gapTerminateLinkEvent_t*)pMsg;
        /* printf("\r\n GAP_LINK_TERMINATED_EVENT received! \r\n"); */
        blebrr_handle_le_disconnection_pl
        (
            0x00, /* Dummy Static Connection Index */
            pPkt->connectionHandle,
            pPkt->reason
        );
    }
    break;
    #if 0

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
        gapLinkUpdateEvent_t* pPkt = (gapLinkUpdateEvent_t*)pMsg;
        l2capParamUpdateReq_t updateReq;
        uint16 timeout = GAP_GetParamValue( TGAP_CONN_PARAM_TIMEOUT );
        printf("\r\n GAP_LINK_PARAM_UPDATE_EVENT received! \r\n");
        updateReq.intervalMin = 0x28;
        updateReq.intervalMax = 0x38;
        updateReq.slaveLatency = 0;
        updateReq.timeoutMultiplier = 0x0c80;
        L2CAP_ConnParamUpdateReq( pPkt->connectionHandle, &updateReq, bleMesh_TaskID );
    }
    break;
        #endif /* 0 */

    default:
        break;
    }
}

bStatus_t BLE_gap_set_scan_params
(
    uint8_t scan_type,
    uint16_t scan_interval,
    uint16_t scan_window,
    uint8_t scan_filterpolicy
)
{
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN_WIND, scan_window );
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN_INT, scan_interval );
    GAP_SetParamValue( TGAP_FILTER_ADV_REPORTS, FALSE);
    GAP_SetParamValue( TGAP_GEN_DISC_SCAN, 5000 );
    GAP_SetParamValue( TGAP_CONN_SCAN_INT,scan_interval );
    GAP_SetParamValue( TGAP_CONN_SCAN_WIND,scan_window);
    //GAP_SetParamValue( TGAP_LIM_DISC_SCAN, 0xFFFF );
    bleMesh_scanparam.activeScan = scan_type;
    bleMesh_scanparam.mode = DEVDISC_MODE_ALL; //DEVDISC_MODE_GENERAL;
    bleMesh_scanparam.whiteList = scan_filterpolicy;
    bleMesh_scanparam.taskID = bleMesh_TaskID;
    return 0x00;
}


bStatus_t BLE_gap_set_scan_enable
(
    uint8_t scan_enable
)
{
    bStatus_t ret;

    if (0x00 != scan_enable)
    {
        ret = GAP_DeviceDiscoveryRequest(&bleMesh_scanparam);

        if (0 == ret)
        {
            osal_set_event (bleMesh_TaskID, BLEMESH_GAP_SCANENABLED);
        }
    }
    else
    {
        bleMesh_DiscCancel = TRUE;
        ret = GAP_DeviceDiscoveryCancel(bleMesh_TaskID);
        #if 0

        if (0 == ret)
        {
            bleMesh_DiscCancel = TRUE;
        }

        #endif
    }

    return ret;
}

bStatus_t BLE_gap_set_adv_params
(
    uint8_t adv_type,
    uint16_t adv_intervalmin,
    uint16_t adv_intervalmax,
    uint8_t adv_filterpolicy
)
{
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, adv_intervalmin );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, adv_intervalmax );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, 0);
    bleMesh_advparam.channelMap = GAP_ADVCHAN_ALL;
    bleMesh_advparam.eventType = adv_type;
    bleMesh_advparam.filterPolicy = adv_filterpolicy;
    bleMesh_advparam.initiatorAddrType = 0x00;
    osal_memset(bleMesh_advparam.initiatorAddr, 0x00, B_ADDR_LEN);
    return API_SUCCESS;
}

bStatus_t BLE_gap_set_advscanrsp_data
(
    uint8_t   type,
    uint8_t* adv_data,
    uint16_t  adv_datalen
)
{
    return GAP_UpdateAdvertisingData(bleMesh_TaskID, type, adv_datalen, adv_data);
}

bStatus_t BLE_gap_connect
(
    uint8_t   whitelist,
    uint8_t* addr,
    uint8_t   addr_type
)
{
    gapEstLinkReq_t params;
    params.taskID = bleMesh_TaskID;
    params.highDutyCycle = TRUE;
    params.whiteList = whitelist;
    params.addrTypePeer = addr_type;
    VOID osal_memcpy( params.peerAddr, addr, B_ADDR_LEN );
    return GAP_EstablishLinkReq( &params );
}

bStatus_t BLE_gap_disconnect(uint16_t   conn_handle)
{
    return GAP_TerminateLinkReq( bleMesh_TaskID, conn_handle, HCI_DISCONNECT_REMOTE_USER_TERM ) ;
}

bStatus_t BLE_gap_set_adv_enable
(
    uint8_t adv_enable
)
{
    bStatus_t ret;

    if (0x00 != adv_enable)
    {
        ret = GAP_MakeDiscoverable(bleMesh_TaskID, &bleMesh_advparam);
    }
    else
    {
        ret = GAP_EndDiscoverable(bleMesh_TaskID);
    }

    return ret;
}

void BLE_ecdh_yield (void)
{
    osal_set_event (bleMesh_TaskID, BLEMESH_ECDH_PROCESS);
}

static void ProcessUartData(uart_Evt_t* evt)
{
    osal_memcpy((cmdstr + cmdlen), evt->data, evt->len);
    cmdlen += evt->len;
    osal_set_event( bleMesh_TaskID, BLEMESH_UART_RX_EVT );
}

void bleMesh_uart_init(void)
{
    hal_uart_deinit(UART0);
    uart_Cfg_t cfg =
    {
        .tx_pin = P9,
        .rx_pin = P10,
        .rts_pin = GPIO_DUMMY,
        .cts_pin = GPIO_DUMMY,
        .baudrate = 115200,
        .use_fifo = TRUE,
        .hw_fwctrl = FALSE,
        .use_tx_buf = FALSE,
        .parity     = FALSE,
        .evt_handler = ProcessUartData,
    };
    hal_uart_init(cfg,UART0);//uart init
}

void UI_set_uuid_octet (UCHAR uuid_0)
{
    UI_lprov_device.uuid[0] = uuid_0;
}
static void ll_dumpConnectionInfo(void )
{
    printf("========== LL PM counters ================\r\n");
    printf("ll_send_undirect_adv_cnt = %d\r\n", g_pmCounters.ll_send_undirect_adv_cnt);
    printf("ll_send_nonconn_adv_cnt = %d\r\n", g_pmCounters.ll_send_nonconn_adv_cnt);
    printf("ll_send_scan_adv_cnt = %d\r\n", g_pmCounters.ll_send_scan_adv_cnt);
    printf("ll_send_hdc_dir_adv_cnt = %d\r\n", g_pmCounters.ll_send_hdc_dir_adv_cnt);
    printf("ll_send_ldc_dir_adv_cnt = %d\r\n", g_pmCounters.ll_send_ldc_dir_adv_cnt);
    printf("ll_send_conn_adv_cnt = %d\r\n", g_pmCounters.ll_send_conn_adv_cnt);
    printf("ll_conn_adv_pending_cnt = %d\r\n", g_pmCounters.ll_conn_adv_pending_cnt);
    printf("ll_conn_scan_pending_cnt = %d\r\n", g_pmCounters.ll_conn_scan_pending_cnt);
    printf("ll_recv_scan_req_cnt = %d\r\n", g_pmCounters.ll_recv_scan_req_cnt);
    printf("ll_send_scan_rsp_cnt = %d\r\n", g_pmCounters.ll_send_scan_rsp_cnt);
    printf("ll_recv_conn_req_cnt = %d\r\n", g_pmCounters.ll_recv_conn_req_cnt);
    printf("ll_send_conn_rsp_cnt = %d\r\n", g_pmCounters.ll_send_conn_rsp_cnt);
    printf("ll_filter_scan_req_cnt = %d\r\n", g_pmCounters.ll_filter_scan_req_cnt);
    printf("ll_filter_conn_req_cnt = %d\r\n", g_pmCounters.ll_filter_conn_req_cnt);
    printf("ll_recv_adv_pkt_cnt = %d\r\n", g_pmCounters.ll_recv_adv_pkt_cnt);
    printf("ll_send_scan_req_cnt = %d\r\n", g_pmCounters.ll_send_scan_req_cnt);
    printf("ll_recv_scan_rsp_cnt = %d\r\n", g_pmCounters.ll_recv_scan_rsp_cnt);
    printf("ll_conn_succ_cnt = %d\r\n", g_pmCounters.ll_conn_succ_cnt);
    printf("ll_link_lost_cnt = %d\r\n", g_pmCounters.ll_link_lost_cnt);
    printf("ll_link_estab_fail_cnt = %d\r\n", g_pmCounters.ll_link_estab_fail_cnt);
    printf("ll_rx_peer_cnt = %d\r\n", g_pmCounters.ll_rx_peer_cnt);
    printf("ll_evt_shc_err = %d\r\n", g_pmCounters.ll_evt_shc_err);
    printf("ll_trigger_err = %d\r\n", g_pmCounters.ll_trigger_err);
    printf("ll_rfifo_rst_err = %d\r\n", g_pmCounters.ll_rfifo_rst_err);
    printf("ll_rfifo_rst_cnt = %d\r\n", g_pmCounters.ll_rfifo_rst_cnt);
    printf("ll_rfifo_read_err = %d\r\n", g_pmCounters.ll_rfifo_read_err);
    printf("hardfault function = %08x\n",JUMP_FUNCTION(HARDFAULT_HANDLER));
    printf("\r\n ");
}


extern uint8 llState;
extern uint8 llSecondaryState;
extern llGlobalStatistics_t g_pmCounters;
extern UCHAR blebrr_state;
extern uint32 blebrr_advscan_timeout_count;
API_RESULT cli_internal_status(UINT32 argc, UCHAR* argv[])
{
    MS_IGNORE_UNUSED_PARAM(argc);
    MS_IGNORE_UNUSED_PARAM(argv);
    printf("\r\n===== internal status ============\r\n");
    printf("llState = %d, llSecondaryState = %d\r\n", llState, llSecondaryState);
//    printf(" crc err count = %d\r\n",
//             g_pmCounters.ll_recv_crcerr_event_cnt);
    printf("blebrr_state = %d\r\n", blebrr_state);
//  printf("blebrr_scanTimOut = %d\r\n", blebrr_scanTimeOut);
//    printf("blebrr_advscan_timeout_count = %d\r\n", blebrr_advscan_timeout_count);
//    osal_memory_statics();
//    printf("Mac Address: %2X %2X %2X %2X %2X %2X", UI_lprov_device.uuid[8], UI_lprov_device.uuid[9],
//                                                       UI_lprov_device.uuid[10], UI_lprov_device.uuid[11],
//                                                       UI_lprov_device.uuid[12], UI_lprov_device.uuid[13]);
    printf("\r\n");
    ll_dumpConnectionInfo();
    return API_SUCCESS;
}

void on_off_reset_process(void)
{
    uint32 tmp = *(volatile uint32_t*) ON_OFF_RESET_FLASH_ADDR ;

    if(tmp==0xffffffff)
        tmp=0;

    hal_flash_erase_sector(ON_OFF_RESET_FLASH_ADDR);
    flash_write_word(ON_OFF_RESET_FLASH_ADDR, tmp+1);
    g_dev_reset_flg = tmp+1;
    WaitMs(500);
    hal_flash_erase_sector(ON_OFF_RESET_FLASH_ADDR);
}



/*********************************************************************
*********************************************************************/
