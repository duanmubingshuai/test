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

    Description:    This file contains the Simple GATT profile sample GATT service
                  profile for use with the BLE sample application.


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
#include "simpleGATTprofile.h"
#include "log.h"
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
CONST uint8 simpleGATTProfileServUUID[ATT_BT_UUID_SIZE] =  {0xF0,0xFF};
CONST uint8 simpleGATTProfilechar1UUID[ATT_BT_UUID_SIZE] = {0xF1,0xFF};
CONST uint8 simpleGATTProfilechar2UUID[ATT_BT_UUID_SIZE] = {0xF2,0xFF};


/*********************************************************************
    LOCAL VARIABLES
*/
/*********************************************************************
    Profile Attributes - variables
*/
// multi Profile Service attribute
static CONST gattAttrType_t simpleGATTProfileService = { ATT_BT_UUID_SIZE, simpleGATTProfileServUUID };

// simpleGATTProfile Characteristic 1 Properties
static uint8 simpleGATTProfileChar1Props = GATT_PROP_WRITE;
// Characteristic 1 Value
static uint8 simpleGATTProfileChar1[ SIMPLEGATT_VALUE_SIZE ];

// simpleGATTProfile Characteristic 2 Properties
static uint8 simpleGATTProfileChar2Props = GATT_PROP_READ | GATT_PROP_NOTIFY;
// Characteristic 2 Value
static uint8 simpleGATTProfileChar2[ SIMPLEGATT_VALUE_SIZE];
// Characteristic 2 cccd
static uint16 simpleGATTProfileChar2Config = GATT_CFG_NO_OPERATION;


/*********************************************************************
    Profile Attributes - Table
*/
static gattAttribute_t simpleGATTProfileAttrTbl[] =
{
    // multi Profile Service
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
        GATT_PERMIT_READ,                         /* permissions */
        0,                                        /* handle */
        (uint8*)&simpleGATTProfileService            /* pValue */
    },

    // Characteristic 1 Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &simpleGATTProfileChar1Props
    },

    // Characteristic Value 1
    {
        { ATT_BT_UUID_SIZE, simpleGATTProfilechar1UUID },
        GATT_PERMIT_ENCRYPT_READ | GATT_PERMIT_WRITE,
        0,
        &simpleGATTProfileChar1[0]
    },

    // ----------------------------------------------------------------------
    // Characteristic 2 Declaration, NOTify
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &simpleGATTProfileChar2Props
    },

    // Characteristic Value 2
    {
        { ATT_BT_UUID_SIZE, simpleGATTProfilechar2UUID },
        GATT_PERMIT_READ,
        0,
        (uint8*)&simpleGATTProfileChar2
    },

    // Characteristic 2 configuration
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8*)(&simpleGATTProfileChar2Config)
    },
};

/*********************************************************************
    LOCAL FUNCTIONS
*/
static bStatus_t simpleGATTProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                           uint8* pValue, uint8 len, uint16 offset );


/*********************************************************************
    PROFILE CALLBACKS
*/
// multi Profile Service Callbacks
CONST gattServiceCBs_t simpleGATTProfileCBs =
{
    NULL,  						// Read callback function pointer
    simpleGATTProfile_WriteAttrCB, 	// Write callback function pointer
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
bStatus_t simpleGATTProfile_AddService()
{
    bStatus_t status = SUCCESS;
    status = GATTServApp_RegisterService( simpleGATTProfileAttrTbl,
                                          GATT_NUM_ATTRS( simpleGATTProfileAttrTbl ),
                                          &simpleGATTProfileCBs );
    return ( status );
}


/*********************************************************************
    @fn      simpleGATTProfile_WriteAttrCB

    @brief   Validate attribute data prior to a write operation

    @param   connHandle - connection message was received on
    @param   pAttr - pointer to attribute
    @param   pValue - pointer to data to be written
    @param   len - length of data
    @param   offset - offset of the first octet to be written

    @return  Success or Failure
*/
static bStatus_t simpleGATTProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t* pAttr,
                                           uint8* pValue, uint8 len, uint16 offset )
{
	bStatus_t status = SUCCESS;

	#ifdef _PHY_DEBUG 
	LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif

    // If attribute permissions require authorization to write, return error
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    {
        // Insufficient authorization
        return ( ATT_ERR_INSUFFICIENT_AUTHOR );
    }
	

    if ( pAttr->type.len == ATT_BT_UUID_SIZE )
    {
    	#ifdef _PHY_DEBUG 
			LOG_DEBUG("	connection handle %d\n",connHandle);
			LOG_DEBUG("	uuid len %d,value:",pAttr->type.len);
			for(uint8 i=0;i < pAttr->type.len;i++)
				LOG_DEBUG("0x%02X,",pAttr->type.uuid[i]);
			LOG_DEBUG("\n");
			LOG_DEBUG("	write data len %d, value : ",len);
			for(uint8 i=0;i < len;i++)
				LOG_DEBUG("0x%02X,",pValue[i]);
			LOG_DEBUG("\n");
		#endif
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
		switch ( uuid )
		{
			case GATT_CLIENT_CHAR_CFG_UUID:
			{
				simpleGATTProfileChar2Config = BUILD_UINT16( pValue[0], pValue[1]);
			}
			break;
		}
		SimpleGATTProfile_Notify(connHandle,len,(uint8* )pValue);
        #if( DEF_PHYPLUS_TRX_SUPPORT==2)
        if(pValue[0]==0x12)
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
                LOG_DEBUG("[PPP RX] Start RX  ret %d\n", ret);
            }
        }
        #elif( DEF_PHYPLUS_TRX_SUPPORT==1)
        if(pValue[0]==0x11)
        {
            uint8_t ret;
            if(pValue[1] == 0)
            {
                ret = phy_rf_stop_tx();
            }
            else
            {
                uint8_t dlen = pValue[2];
                uint8_t d[32];
                for(uint8 i=1;i<32;i++){d[i]=i;};
                extern uint32_t rtc_get_counter(void);
                d[0]=0xff&(rtc_get_counter());
                ret = phy_rf_start_tx(d,dlen,(pValue[1]),0);
                LOG_DEBUG("[PPP TX] Intv: %04d ms ret%d\n", (pValue[1]),ret);
            }
        }
        
        #endif
    }
    return status;
}

void SimpleGATTProfile_Notify(uint16 connHandle,uint8 len, void* value )
{
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
		LOG("	Check notify enable %d\n",simpleGATTProfileChar2Config);
		LOG("	Check notify data length %d\n",len);
		LOG("	Check notify data:");
		for(uint8 i=0;i<len;i++)
			LOG("0x%02X,",((uint8 *)value)[i]);
		LOG("\n");
	#endif
	if ( simpleGATTProfileChar2Config & GATT_CLIENT_CFG_NOTIFY )
    {
		attHandleValueNoti_t Notif;
        Notif.handle = simpleGATTProfileAttrTbl[SIMPLEGATTPROFILE_NOTIFY_IDX].handle;
        Notif.len = len;
		osal_memcpy(Notif.value, (uint8 *)value, len);
        GATT_Notification( connHandle, &Notif, FALSE );
		
	}
}

