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
    Filename:       wrist.c
    Revised:        $Date $
    Revision:       $Revision $


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt_profile_uuid.h"
#include "wristservice.h"
#include "ota_app_service.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "app_wrist.h"
#include "ui_page.h"
#include "ui_task.h"
#include "ui_display.h"
#include "ui_dispTFT.h"
#include "touch_key.h"
#include "em70xx.h"
#include "KX023.h"
#include "battery.h"
#include "led_light.h"
#include "kscan.h"
#include "log.h"


#include "hidkbdservice.h"
#include "hiddev.h"
#include "ll_def.h"
#include "spi.h"
#include "lcd.h"
#include "gui.h"
#include "test.h"
#include "adc.h"


/*********************************************************************
    MACROS
*/

// Convert BPM to RR-Interval for data simulation purposes
#define HEARTRATE_BPM_TO_RR(bpm)              ((uint16) 60 * 1024 / (uint16) (bpm))

/*********************************************************************
    CONSTANTS
*/

// HID idle timeout in msec; set to zero to disable timeout
#define DEFAULT_HID_IDLE_TIMEOUT              0


// Fast advertising interval in 625us units
#define DEFAULT_FAST_ADV_INTERVAL             32

// Duration of fast advertising duration in ms
#define DEFAULT_FAST_ADV_DURATION             30000

// Slow advertising interval in 625us units
#define DEFAULT_SLOW_ADV_INTERVAL             32

// Duration of slow advertising duration in ms (set to 0 for continuous advertising)
#define DEFAULT_SLOW_ADV_DURATION             0

// How often to perform heart rate periodic event
#define DEFAULT_HEARTRATE_PERIOD              2000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     20

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     30

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500







// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Default passcode
#define DEFAULT_PASSCODE                      0

// Default GAP pairing mode
//#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_INITIATE
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT


// HID keyboard input report length
#define HID_KEYBOARD_IN_RPT_LEN     8

// HID LED output report length
#define HID_LED_OUT_RPT_LEN         1

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5




// Some values used to simulate measurements
#define BPM_DEFAULT                           73
#define BPM_MAX                               80
#define ENERGY_INCREMENT                      10
#define FLAGS_IDX_MAX                         7

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/



/*********************************************************************
    EXTERNAL VARIABLES
*/

/*********************************************************************
    EXTERNAL FUNCTIONS
*/

/*********************************************************************
    LOCAL VARIABLES
*/
uint8 AppWrist_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP Profile - Name attribute for SCAN RSP data
/*  static uint8 scanData[] =
    {
    0x10,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'P',
    'h',
    'y',
    '+',
    ' ',
    'W',
    'r',
    'i',
    's',
    't',
    ' ',
    'D',
    'e',
    'm',
    'o',
    };*/

static uint8 scanData[] =
{
    0xa,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'P',
    'h',
    'y',
    'W',
    'r',
    'i',
    's',
    't',
    ' ',
};


static uint8 advertData[] =
{
    // flags
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    // service UUIDs
    0x03,
    GAP_ADTYPE_16BIT_MORE,
    LO_UINT16(HEARTRATE_SERV_UUID),
    HI_UINT16(HEARTRATE_SERV_UUID),
};

// Device name attribute value
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PhyWrist ";

// GAP connection handle
//static uint16 gapConnHandle;

// Advertising user-cancelled state
//static bool WristAdvCancelled = FALSE;



/*********************************************************************
    PROFILE CALLBACKS
*/



/*********************************************************************
    LOCAL FUNCTIONS
*/
static void appWristProcOSALMsg( osal_event_hdr_t* pMsg );
//static void WristGapStateCB( gaprole_States_t newState);
//static void HeartRateGapStateCB( gaprole_States_t newState );
//static void heartRatePeriodicTask( void );
//static void heartRateMeasNotify(void);
static void wristCB(uint8 event, uint8 param_size, uint8* param);

static uint8 hidKbdRcvReport( uint8 len, uint8* pData );
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                          uint8 oper, uint16* pLen, uint8* pData );
static void hidKbdEvtCB( uint8 evt );

/*********************************************************************
    PROFILE CALLBACKS
*/

// GAP Role Callbacks
//static gapRolesCBs_t WristPeripheralCB =
//{
//    WristGapStateCB,  // Profile State Change Callbacks
//    NULL                            // When a valid RSSI is read from controller
//};

// Bond Manager Callbacks
//static const gapBondCBs_t WristBondCB =
//{
//    NULL,                   // Passcode callback
//    NULL                    // Pairing state callback
//};




static hidDevCB_t hidKbdHidCBs =
{
    hidKbdRptCB,
    hidKbdEvtCB,
    NULL
};


// HID Dev configuration
static hidDevCfg_t hidKbdCfg =
{
    DEFAULT_HID_IDLE_TIMEOUT,   // Idle timeout
    HID_KBD_FLAGS               // HID feature flags
};

// TRUE if boot mouse enabled
uint8 hidBootMouseEnabled = FALSE;





static void on_HeartRateValueUpdate(hr_ev_t* pev)
{
    switch(pev->ev)
    {
    case HR_EV_HR_VALUE:
    {
        ui_ev_t ev =
        {
            .ev = UI_EV_HEARTRATE,
            .param = (uint16)pev->value,
            .data = NULL
        };
        wristProfileResponseHRValue(pev->value);
        ui_fsm_run(&ev);
    }
    break;

    case HR_EV_RAW_DATA:
    {
        wristProfileResponseHRRawData(pev->value, pev->data);
    }
    break;

    default:
        break;
    }
}

void on_touchKey(touch_evt_t key_evt)
{
    osal_set_event(AppWrist_TaskID, TOUCH_PRESS_EVT);
}

void on_kscan_evt(kscan_Evt_t* kscan_evt)
{
    uint8_t key[16];
    uint8_t r,c,k;
    osal_memset(key, 0, 16);
    LOG("KSE %d\n",kscan_evt->num);

    for(uint8_t i=0; i<kscan_evt->num; i++)
    {
        k = kscan_evt->keys[i].type == KEY_PRESSED ? 1:2;
        r = kscan_evt->keys[i].row &0x7;
        c = kscan_evt->keys[i].col &0x7;
        key[i] = (k << 6)|(c << 3)| r;
    }

    wristProfileResponseKScan(kscan_evt->num, key);
}

void on_kx023_evt(kx023_ev_t* pev)
{
    int g_acc_value[3];

    if(pev->ev == wmi_event)
    {
        int i;
        int gx = 0,gy = 0,gz = 0;
        int16_t* acc_data = (int16_t*)pev->data;

        for(i = 0; i < pev->size/(sizeof(int16_t)*3); i++)
        {
            gx += (int)acc_data[0];
            gy += (int)acc_data[1];
            gz += (int)acc_data[2];
            acc_data+=3;
        }

        //LOG("X%d,Y%d,Z%d\n",gx,gy,gz);
        gx = gx*6/pev->size;
        gy = gy*6/pev->size;
        gz = gz*6/pev->size;
        //LOG("X%d,Y%d,Z%d\n",gx,gy,gz);
        g_acc_value[0] = gx;
        g_acc_value[1] = gy;
        g_acc_value[2] = gz;

        if(gx==0 && gy==0 &&gz==0)
            return;

        ui_accelerator_event(g_acc_value);
        wristProfileResponseAccelerationData(gx/4,gy/4,gz/4);
    }
}


/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      HeartRate_Init

    @brief   Initialization function for the Heart Rate App Task.
            This is called during initialization and should contain
            any application specific initialization (ie. hardware
            initialization/setup, table initialization, power up
            notificaiton ... ).

    @param   task_id - the ID assigned by OSAL.  This ID should be
                      used to send messages and set timers.

    @return  none
*/
//  static int uipg_msg_key_press(void);

//hal_spi_t lcd_spi ={
//    .spi_index = SPI0,
//};

//spi_Cfg_t spi_cfg = {
//    .sclk_pin = GPIO_P33,
//    .ssn_pin = GPIO_P25,
//    .MOSI = GPIO_P32,
//    .MISO = GPIO_DUMMY,

//    .baudrate = 4000000,
//    .spi_tmod = SPI_TRXD,
//    .spi_scmod = SPI_MODE0,

//    .int_mode = false,
//    .force_cs = true,
//    .evt_handler = NULL,
//};


//static void LCD_WR_REG(uint8_t data)
//{
//    uint8_t buf_send[4];
//  uint8_t buf_rece[4];
//    hal_gpio_write(P31, 0);
//    buf_send[0] = data;
//    hal_spi_transmit(&lcd_spi,SPI_TXD,buf_send,buf_rece,1,0);
//}

//static void LCD_WR_DATA(uint8_t data)
//{
//    uint8_t buf_send[4];
//  uint8_t buf_rece[4];
//    hal_gpio_write(P31, 1);
//    buf_send[0] = data;
//    hal_spi_transmit(&lcd_spi,SPI_TXD,buf_send,buf_rece,1,0);
//}

//void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
//{
//  LCD_WR_REG(LCD_Reg);
//  Lcd_WriteData_16Bit(LCD_RegValue);
//}


#define GPIO_GREEN    P15
#define GPIO_BLUE     P17
#define GPIO_RED      P16
static gpio_pin_e led_pins[3] = {GPIO_GREEN,GPIO_BLUE,GPIO_RED};
void appWristInit( uint8 task_id )
{
    AppWrist_TaskID = task_id;
    verInfo.verNum= 0x0006;        //
    verInfo.comId= 0x000D;        //
    verInfo.subverNum=0x0101;         // sub version number
    // Setup the GAP Peripheral Role Profile
    {
        uint8 initial_advertising_enable = TRUE;
        uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39;
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;
        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        uint8 peerPublicAddr[] =
        {
            0x01,
            0x02,
            0x03,
            0x04,
            0x05,
            0x06
        };
        GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
        // set adv channel map
        GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);
        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanData ), scanData );
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
    // Setup the GAP Bond Manager
    {
        uint32 passkey = DEFAULT_PASSCODE;
        uint8 pairMode = DEFAULT_PAIRING_MODE;
        uint8 mitm = DEFAULT_MITM_MODE;
        uint8 ioCap = DEFAULT_IO_CAPABILITIES;
        uint8 bonding = DEFAULT_BONDING_MODE;
        GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
        GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
        GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
        GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
        GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
    }
    // Set advertising interval
    {
        uint16 advInt = 400;   // actual time = advInt * 625us
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
    hal_adc_init();
    wristProfile_AddService(wristCB);
    ota_app_AddService();
    app_datetime_init();
    // Setup a delayed profile startup
    osal_set_event( AppWrist_TaskID, START_DEVICE_EVT );
    light_init(led_pins,3);
    light_blink_evt_cfg(AppWrist_TaskID,TIMER_LIGHT_EVT);
    LIGHT_ONLY_RED_ON;
    LCD_Init();
//    while(1)
    {
        main_test();        //����������
        Test_Color();       //��ˢ��������
        Test_FillRec();     //GUI���λ�ͼ����
        Test_Circle();      //GUI��Բ����
        Test_Triangle();    //GUI�����λ�ͼ����
        English_Font_test();//Ӣ������ʾ������
        Chinese_Font_test();//��������ʾ������
        Pic_test();         //ͼƬ��ʾʾ������
        Rotate_Test();   //��ת��ʾ����
    }
    em70xx_register(on_HeartRateValueUpdate);
    touch_init(on_touchKey);
//    ui_init();
//    hal_gpio_fmux(P10,Bit_DISABLE);
//    hal_gpio_pin_init(P10,GPIO_INPUT);
//    hal_gpio_pull_set(P10,STRONG_PULL_UP);
//    hal_gpio_pull_set(P10,PULL_DOWN);
//    hal_gpio_pull_set(P10,WEAK_PULL_UP);
    batt_init();
    kx023_init(on_kx023_evt);
    {
        //    kscan_Cfg_t kcfg;
        //    kcfg.ghost_key_state = NOT_IGNORE_GHOST_KEY;
        //    kcfg.key_rows = kscan_rows;
        //    kcfg.key_cols = kscan_cols;
        //    kcfg.interval = 50;
        //    kcfg.evt_handler = on_kscan_evt;
        //hal_kscan_init(kcfg, AppWrist_TaskID, TIMER_KSCAN_DEBOUNCE_EVT);
    }
    // Set up HID keyboard service
    HidKbd_AddService( );
    // Register for HID Dev callback
    HidDev_Register( &hidKbdCfg, &hidKbdHidCBs );
}

/*********************************************************************
    @fn      HeartRate_ProcessEvent

    @brief   Heart Rate Application Task event processor.  This function
            is called to process all events for the task.  Events
            include timers, messages and any other user defined events.

    @param   task_id  - The OSAL assigned task ID.
    @param   events - events to process.  This is a bit map and can
                     contain more than one event.

    @return  events not processed
*/
uint16 appWristProcEvt( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8* pMsg;

        if ( (pMsg = osal_msg_receive( AppWrist_TaskID )) != NULL )
        {
            appWristProcOSALMsg( (osal_event_hdr_t*)pMsg );
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & START_DEVICE_EVT )
    {
        return ( events ^ START_DEVICE_EVT );
    }

    if( events & TIMER_DT_EVT)
    {
        app_datetime_sync_handler();
        return ( events ^ TIMER_DT_EVT );
    }

    if( events & TIMER_UI_EVT)
    {
        ui_page_timer_evt(NULL);
        return ( events ^ TIMER_UI_EVT );
    }

    if( events & TOUCH_PRESS_EVT)
    {
        ui_key_evt(NULL);
        return ( events ^ TOUCH_PRESS_EVT );
    }

    if( events & TIMER_HR_EVT)
    {
        em70xx_timer_handler();
        return ( events ^ TIMER_HR_EVT);
    }

    if( events & TIMER_BATT_EVT)
    {
        batt_meas_timeout_handler();
        return ( events ^ TIMER_BATT_EVT);
    }

    if( events & BATT_CHARGE_EVT)
    {
        batt_evt_t evt = batt_charge_detect() ? BATT_CHARGE_PLUG : BATT_CHARGE_UNPLUG;
        LOG("batt_evt_t %d\n", evt);
        ui_batt_event(evt);
        return ( events ^ BATT_CHARGE_EVT);
    }

    if( events & BATT_VALUE_EVT)
    {
        ui_batt_event(BATT_VOLTAGE);
        return ( events ^ BATT_VALUE_EVT);
    }

    if( events & ACC_DATA_EVT)
    {
        drv_kx023_event_handle();
        return ( events ^ ACC_DATA_EVT);
    }

    if( events & TIMER_LIGHT_EVT)
    {
        light_blink_porcess_evt();
        return ( events ^ TIMER_LIGHT_EVT);
    }

    if( events & TIMER_KSCAN_DEBOUNCE_EVT)
    {
        hal_kscan_timeout_handler();
        return ( events ^ TIMER_KSCAN_DEBOUNCE_EVT);
    }

    // Discard unknown events
    return 0;
}


/*********************************************************************
    @fn      appWristProcOSALMsg

    @brief   Process an incoming task message.

    @param   pMsg - message to process

    @return  none
*/
static void appWristProcOSALMsg( osal_event_hdr_t* pMsg )
{
    if(pMsg->event == GATT_SERV_MSG_EVENT)
    {
        LOG("GATT_SERV_MSG_EVENT\n");
        gattClientCharCfgUpdatedEvent_t* pEvent = (gattClientCharCfgUpdatedEvent_t*)pMsg;

        if(pEvent->value == GATT_CLIENT_CFG_INDICATE)
        {
            LOG("Indicate service change\n");
            GATTServApp_SendServiceChangedInd(gapConnHandle,AppWrist_TaskID);
        }
    }
}


/*********************************************************************
    @fn      WristGapStateCB

    @brief   Notification from the profile of a state change.

    @param   newState - new state

    @return  none
*/
//static void WristGapStateCB( gaprole_States_t newState )
//{
//    LOG("WristGapStateCB: %d", newState);
//    // if connected
//    if (newState == GAPROLE_CONNECTED)
//    {
//        // get connection handle
//        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
//    }
//    // if disconnected
//    else if (gapProfileState == GAPROLE_CONNECTED &&
//           newState != GAPROLE_CONNECTED)
//    {
//        uint8 advState = TRUE;

//        // reset client characteristic configuration descriptors
//        wristProfile_HandleConnStatusCB( gapConnHandle, LINKDB_STATUS_UPDATE_REMOVED );

//        if ( newState == GAPROLE_WAITING_AFTER_TIMEOUT )
//        {
//            // link loss timeout-- use fast advertising
//            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_FAST_ADV_INTERVAL );
//            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_FAST_ADV_INTERVAL );
//            GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_FAST_ADV_DURATION );
//        }
//        else
//        {
//            // Else use slow advertising
//            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
//            GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
//            GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
//        }

//        // Enable advertising
//        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
//    }
//    // if advertising stopped
//    else if ( gapProfileState == GAPROLE_ADVERTISING &&
//            newState == GAPROLE_WAITING )
//    {
//    // if advertising stopped by user
//    if ( WristAdvCancelled )
//    {
//        WristAdvCancelled = FALSE;
//    }
//    // if fast advertising switch to slow
//    else if ( GAP_GetParamValue( TGAP_GEN_DISC_ADV_INT_MIN ) == DEFAULT_FAST_ADV_INTERVAL )
//    {
//        uint8 advState = TRUE;

//        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, DEFAULT_SLOW_ADV_INTERVAL );
//        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, DEFAULT_SLOW_ADV_INTERVAL );
//        GAP_SetParamValue( TGAP_GEN_DISC_ADV_MIN, DEFAULT_SLOW_ADV_DURATION );
//        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advState );
//    }
//    }
//    // if started
//    else if (newState == GAPROLE_STARTED)
//    {
//        // Set the system ID from the bd addr
//        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
//        GAPRole_GetParameter(GAPROLE_BD_ADDR, systemId);

//        // shift three bytes up
//        systemId[7] = systemId[5];
//        systemId[6] = systemId[4];
//        systemId[5] = systemId[3];

//        // set middle bytes to zero
//        systemId[4] = 0;
//        systemId[3] = 0;

//        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
//    }

//    gapProfileState = newState;
//}

/*********************************************************************
    @fn      heartRateCB

    @brief   Callback function for Wrist service.

    @param   event - service event

    @return  none
*/
static void wristCB(uint8 event, uint8 param_size, uint8* param)
{
    switch(event)
    {
    case WRIST_NOTI_ENABLED:
    {
        // if connected start periodic measurement
        if (gapProfileState == GAPROLE_CONNECTED)
        {
            //osal_start_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT, DEFAULT_HEARTRATE_PERIOD );
        }

        break;
    }

    case WRIST_NOTI_DISABLED:
    {
        // stop periodic measurement
        //osal_stop_timerEx( AppWrist_TaskID, HEART_PERIODIC_EVT );
        break;
    }
    }
}



/*********************************************************************
    @fn      hidKbdRcvReport

    @brief   Process an incoming HID keyboard report.

    @param   len - Length of report.
    @param   pData - Report data.

    @return  status
*/
static uint8 hidKbdRcvReport( uint8 len, uint8* pData )
{
    // verify data length
    if ( len == HID_LED_OUT_RPT_LEN )
    {
        // set keyfob LEDs
        //HalLedSet( HAL_LED_1, ((*pData & LED_CAPS_LOCK) == LED_CAPS_LOCK) );
        //HalLedSet( HAL_LED_2, ((*pData & LED_NUM_LOCK) == LED_NUM_LOCK) );
        return SUCCESS;
    }
    else
    {
        return ATT_ERR_INVALID_VALUE_SIZE;
    }
}



/*********************************************************************
    @fn      hidKbdRptCB

    @brief   HID Dev report callback.

    @param   id - HID report ID.
    @param   type - HID report type.
    @param   uuid - attribute uuid.
    @param   oper - operation:  read, write, etc.
    @param   len - Length of report.
    @param   pData - Report data.

    @return  GATT status code.
*/
static uint8 hidKbdRptCB( uint8 id, uint8 type, uint16 uuid,
                          uint8 oper, uint16* pLen, uint8* pData )
{
    uint8 status = SUCCESS;

    // write
    if ( oper == HID_DEV_OPER_WRITE )
    {
        if ( uuid == REPORT_UUID )
        {
            // process write to LED output report; ignore others
            if ( type == HID_REPORT_TYPE_OUTPUT )
            {
                status = hidKbdRcvReport( *pLen, pData );
            }
        }

        if ( status == SUCCESS )
        {
            status = HidKbd_SetParameter( id, type, uuid, *pLen, pData );
        }
    }
    // read
    else if ( oper == HID_DEV_OPER_READ )
    {
        status = HidKbd_GetParameter( id, type, uuid, pLen, pData );
    }
    // notifications enabled
    else if ( oper == HID_DEV_OPER_ENABLE )
    {
        if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
        {
            hidBootMouseEnabled = TRUE;
        }
    }
    // notifications disabled
    else if ( oper == HID_DEV_OPER_DISABLE )
    {
        if ( id == HID_RPT_ID_MOUSE_IN && type == HID_REPORT_TYPE_INPUT )
        {
            hidBootMouseEnabled = FALSE;
        }
    }

    return status;
}

/*********************************************************************
    @fn      hidKbdEvtCB

    @brief   HID Dev event callback.

    @param   evt - event ID.

    @return  HID response code.
*/
static void hidKbdEvtCB( uint8 evt )
{
    // process enter/exit suspend or enter/exit boot mode
    return;
}



/*********************************************************************
*********************************************************************/
