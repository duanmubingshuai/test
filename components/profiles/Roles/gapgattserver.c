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
  Filename:       gapgattserver.c
  Revised:         
  Revision:        

  Description:    GAP Attribute Server


**************************************************************************************************/
#if( defined( SERV_GGS ) && SERV_GGS )

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "gap.h"
#include "gapgattserver.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static const ggsAppCBs_t *ggs_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// GAP Service
static CONST gattAttrType_t gapService = { ATT_BT_UUID_SIZE, gapServiceUUID };

// Device Name Characteristic Properties
static uint8 deviceNameCharProps = GATT_PROP_READ;

// Device Name attribute (0 - 248 octets) - extra octet for null-terminate char
static uint8 deviceName[GAP_DEVICE_NAME_LEN+1] = { 0 };

// Appearance Characteristic Properties
static uint8 appearanceCharProps = GATT_PROP_READ;

// Appearance attribute (2-octet enumerated value as defined by Bluetooth Assigned Numbers document)
static uint16 appearance = GAP_APPEARE_UNKNOWN;


/*********************************************************************
 * Profile Attributes - Table
 */

// GAP Attribute Table
static gattAttribute_t gapAttrTbl[] =
{
  // Generic Access Profile
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&gapService                      /* pValue */
  },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &deviceNameCharProps
    },

      // Device Name attribute
      {
        { ATT_BT_UUID_SIZE, deviceNameUUID },
        GATT_PERMIT_READ,
        0,
        deviceName
      },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &appearanceCharProps
    },

      // Icon attribute
      {
        { ATT_BT_UUID_SIZE, appearanceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)&appearance
      }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
//static void ggs_SetAttrWPermit( uint8 wPermit, uint8 *pPermissions, uint8 *pCharProps );

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
// GGS Callback functions
static uint8 ggs_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GAP Service Callbacks
CONST gattServiceCBs_t gapServiceCBs =
{
  ggs_ReadAttrCB,  // Read callback function pointer
  NULL, 			// Write callback function pointer
  NULL             // Authorization callback function pointer
};

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * @fn      GGS_AddService
 *
 * @brief   Add function for the GAP GATT Service.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  SUCCESS: Service added successfully.
 *          INVALIDPARAMETER: Invalid service field.
 *          FAILURE: Not enough attribute handles available.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t GGS_AddService(    )
{
	uint8 status = SUCCESS;
	// Register GAP attribute list and CBs with GATT Server Server App
	status = GATTServApp_RegisterService( gapAttrTbl, GATT_NUM_ATTRS( gapAttrTbl ),
		                                  &gapServiceCBs );

	return ( status );
}

/*********************************************************************
 * @fn      GGS_RegisterAppCBs
 *
 * @brief   Registers the application callback function.
 *
 *          Note: Callback registration is needed only when the
 *                Device Name is made writable. The application
 *                will be notified when the Device Name is changed
 *                over the air.
 *
 * @param   appCallbacks - pointer to application callbacks.
 *
 * @return  none
 */
void GGS_RegisterAppCBs( ggsAppCBs_t *appCallbacks )
{
  ggs_AppCBs = appCallbacks;
}

/*********************************************************************
 * @fn          ggs_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 ggs_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                             uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  VOID connHandle; // Not needed for now!

  // Make sure it's not a blob operation
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case DEVICE_NAME_UUID:
        {	
        	uint8 attDeviceName[] = "PICO V0.1:000000000000";
			uint8 publicAddr[B_ADDR_LEN];
			uint8 AddrStr[B_ADDR_LEN*2];
			#if( defined(GAP_CONFIG_STATIC_ADDR) && GAP_CONFIG_STATIC_ADDR)
				extern uint8 BD_STATIC_ADDR[B_ADDR_LEN];
				osal_memcpy(publicAddr],BD_STATIC_ADDR,B_ADDR_LEN );
			#else
			{
				extern uint8 LL_ReadBDADDR( uint8 *bdAddr );
				LL_ReadBDADDR(publicAddr);
				osal_memcpy(publicAddr,publicAddr,B_ADDR_LEN );
			}
			#endif
			{
				char hex[] = "0123456789ABCDEF";
				for(uint8 i=0,j=0;i<B_ADDR_LEN;i++)
				{
					AddrStr[j++] = hex[(publicAddr[i] >> 4) & 0xF ];
					AddrStr[j++] = hex[publicAddr[i] & 0xF ];
				}
			}
			osal_memcpy(&attDeviceName[10],AddrStr,(B_ADDR_LEN<<1) );
			
		  uint8 len = sizeof(attDeviceName)-2;		  

          *pLen = len;
          VOID osal_memcpy( pValue, attDeviceName, len );
        }
        break;

      case APPEARANCE_UUID:
        {
//          uint16 value = *((uint16 *)(pAttr->pValue));

          *pLen = 2;
          pValue[0] = LO_UINT16( GAP_APPEARE_HID_GAMEPAD );
          pValue[1] = HI_UINT16( GAP_APPEARE_HID_GAMEPAD );
        }
        break;

      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_INVALID_HANDLE;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}
#endif

/*********************************************************************
*********************************************************************/
