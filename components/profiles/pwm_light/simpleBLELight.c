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
    Filename:      
    Revised:
    Revision:

    Description:    This file contains the Simple light control sample service
                  profile for use with the BLE pwm light application.


**************************************************************************************************/

/*********************************************************************
    INCLUDES
*/
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "simpleBLELight.h"
#include "log.h"
#include "pwm_light.h"
#include "phy_plus_phy.h"

/*********************************************************************
    MACROS
*/
#define SIMPLEGATTPROFILE_NOTIFY_IDX		4
#define SIMPLEGATT_VALUE_SIZE				20

/*********************************************************************
    CONSTANTS
*/

/*********************************************************************
    TYPEDEFS
*/

/*********************************************************************
    GLOBAL VARIABLES
*/
CONST uint8 simpleLightServUUID[ATT_BT_UUID_SIZE] =  {0xF0,0xFF};
CONST uint8 simpleLightchar1UUID[ATT_BT_UUID_SIZE] = {0xF1,0xFF};


/*********************************************************************
    LOCAL VARIABLES
*/
/*********************************************************************
    Profile Attributes - variables
*/
// multi Profile Service attribute
static CONST gattAttrType_t simpleLightService = { ATT_BT_UUID_SIZE, simpleLightServUUID };

// simpleLight Characteristic 1 Properties
static uint8 simpleLightChar1Props = GATT_PROP_WRITE;
// Characteristic 1 Value
static uint8 simpleLightChar1[ SIMPLEGATT_VALUE_SIZE ];



/*********************************************************************
    Profile Attributes - Table
*/
static gattAttribute_t simpleLightAttrTbl[] =
{
    // multi Profile Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)&simpleLightService            /* pValue */
    },

    // Characteristic 1 Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &simpleLightChar1Props
    },

    // Characteristic Value 1
    {
        { ATT_BT_UUID_SIZE, simpleLightchar1UUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_WRITE,
        0,
        &simpleLightChar1[0]
    },
};

/*********************************************************************
    LOCAL FUNCTIONS
*/
static bStatus_t simpleLight_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                           uint8* pValue, uint8 len, uint16 offset );

/*********************************************************************
    GLOBAL FUNCTIONS
*/

/*********************************************************************
    PROFILE CALLBACKS
*/
// multi Profile Service Callbacks
CONST gattServiceCBs_t simpleLightCBs =
{
    NULL,  						// Read callback function pointer
    simpleLight_WriteAttrCB, 	// Write callback function pointer
    NULL                       	// Authorization callback function pointer
};

/*********************************************************************
    PUBLIC FUNCTIONS
*/

/*********************************************************************
    @fn      multiProfile_AddService

    @brief   Initializes the multi Profile service by registering
            GATT attributes with the GATT server.

    @param   services - services to add. This is a bit map and can
                       contain more than one service.

    @return  Success or Failure
*/
bStatus_t simpleLight_AddService()
{
    bStatus_t status = SUCCESS;
    status = GATTServApp_RegisterService( simpleLightAttrTbl,
                                          GATT_NUM_ATTRS( simpleLightAttrTbl ),
                                          &simpleLightCBs );
    return ( status );
}


/*********************************************************************
    @fn      simpleLight_WriteAttrCB

    @brief   Validate attribute data prior to a write operation

    @param   connHandle - connection message was received on
    @param   pAttr - pointer to attribute
    @param   pValue - pointer to data to be written
    @param   len - length of data
    @param   offset - offset of the first octet to be written

    @return  Success or Failure
*/
static bStatus_t simpleLight_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                           uint8* pValue, uint8 len, uint16 offset )
{
	bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);

	LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	
  if ( ( pAttr->type.len == ATT_BT_UUID_SIZE ) && ( uuid  == 0xfff1))
  {
    LOG("W:%2x%.2x\n",pValue[0],pValue[1]);
    if(pValue[0] == 0)
    {
        if((pValue[1] == 'r') || (pValue[1] == 0))
        {
        pwmlight_cmd(PWMLIGHT_EVT_R_ONOFF);
        }
        else if((pValue[1] == 'g') || (pValue[1] == 1))
        {
        pwmlight_cmd(PWMLIGHT_EVT_G_ONOFF);
        }
        else if((pValue[1] == 'b') || (pValue[1] == 2))
        {
        pwmlight_cmd(PWMLIGHT_EVT_B_ONOFF);
        }
        else if((pValue[1] == 'i') || (pValue[1] == 3))
        {
        pwmlight_cmd(PWMLIGHT_EVT_FADE_IN);
        }
        else if((pValue[1] == 'o') || (pValue[1] == 4))
        {
        pwmlight_cmd(PWMLIGHT_EVT_FADE_OUT);
        }
        else if((pValue[1] == 'a') || (pValue[1] == 5))
        {
        pwmlight_cmd(PWMLIGHT_EVT_ONOFF);
        }
        else if((pValue[1] == 'h') || (pValue[1] == 6))
        {
        pwmlight_cmd(PWMLIGHT_EVT_HALF);
        }
    }

    #if( DEF_PHYPLUS_TRX_SUPPORT!=0)
		#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_TX)
        if(pValue[0] == 1)
        {
                uint8_t d[32];
                d[0]=0xff;
                d[1]=0xf1;      //d[0],d[1] for pwmlight cmd flag
                d[2]=pValue[1];
                phy_rf_start_tx(d,3,0,0);
                LOG_DEBUG("[PWM Light CMD] %d\n", d[2]);
        }
		#endif
		#if(DEF_PHYPLUS_TRX_SUPPORT & PHYPLUS_CONFIG_RX)
        if(pValue[0]==0x2)
        {
            uint8_t ret;

            if(pValue[1]==0x00)
            {

                ret= phy_rf_stop_rx();
                LOG_DEBUG("[PPP RX] Stop ret %d\n", ret);


            }
            else
            {
                ret= phy_rf_start_rx(pValue[1]*1000);
                LOG_DEBUG("[PPP RX] Start RX  ret %d  status %x\n", ret, phy_rf_get_current_status());
            }
        }
		#endif
    #endif
  }
  // 16-bit UUID
  return status;
}



