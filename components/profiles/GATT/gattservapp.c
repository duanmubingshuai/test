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

/*************************************************************************************************
  Filename:       gattservapp.c
  Revised:         
  Revision:        

  Description:    This file contains the GATT Server Application.


  
**************************************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"

#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"


/*********************************************************************
 * MACROS
 */
#define GATT_DEFAULT_PERMISSION_CHECK_ENABLE 		FALSE
#define GATT_DEFAULT_USER_DESCRIPTON_ENABLE			FALSE
#define GATT_DEFAULT_FIND_BY_TYPE_ENABLE			TRUE
#define GATT_DEFAULT_USE_BLOB_REQ_ENABLE			TRUE
/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

// Structure to keep Prepare Write Requests for each Client
//typedef struct
//{
//  uint16 connHandle;                    // connection message was received on
//  attPrepareWriteReq_t *pPrepareWriteQ; // Prepare Write Request queue
//} prepareWrites_t;
//
// GATT Structure to keep CBs information for each service being registered
typedef struct
{
  uint16 handle;                // Service handle - assigned internally by GATT Server
  CONST gattServiceCBs_t *pCBs; // Service callback function pointers
} gattServiceCBsInfo_t;

// Service callbacks list item
typedef struct _serviceCBsList
{
  struct _serviceCBsList *next;     // pointer to next service callbacks record
  gattServiceCBsInfo_t serviceInfo; // service handle/callbacks
} serviceCBsList_t;

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
uint8 GATTServApp_TaskID;   // Task ID for internal task/event processing

uint8 appTaskID = INVALID_TASK_ID; // The task ID of an app/profile that
                                   // wants GATT Server event messages
// Callbacks for services
static serviceCBsList_t *serviceCBsList = NULL;

// Globals to be used for processing an incoming request
static uint8 attrLen;
static uint8 attrValue[ATT_MTU_SIZE-1];
static attMsg_t rsp;

#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
/*** Defined GATT Attributes ***/

// GATT Service attribute
static CONST gattAttrType_t gattService = { ATT_BT_UUID_SIZE, gattServiceUUID };

// Service Changed Characteristic Properties
static uint8 serviceChangedCharProps = GATT_PROP_INDICATE;

// Service Changed attribute (hidden). Set the affected Attribute Handle range
// to 0x0001 to 0xFFFF to indicate to the client to rediscover the entire set
// of Attribute Handles on the server.
// indication char config for service change attributes
static uint16 serviceChange_indCharCfg = GATT_CFG_NO_OPERATION;

/*********************************************************************
 * Profile Attributes - Table
 */

// GATT Attribute Table
static gattAttribute_t gattAttrTbl[] = {
	// Generic Attribute Profile
	{
		{ ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
		GATT_PERMIT_READ,                         /* permissions */
		0,                                        /* handle */
		(uint8 *)&gattService                     /* pValue */
	},
	// Characteristic Declaration
	{
		{ ATT_BT_UUID_SIZE, characterUUID },
		GATT_PERMIT_READ,
		0,
		&serviceChangedCharProps
	},

	// Attribute Service Changed
	{
		{ ATT_BT_UUID_SIZE, serviceChangedUUID },
		0,
		0,
		NULL
	},

	// Client Characteristic configuration
	{
		{ ATT_BT_UUID_SIZE, clientCharCfgUUID },
		GATT_PERMIT_READ | GATT_PERMIT_WRITE,
		0,
		(uint8 *)(&serviceChange_indCharCfg)
	}
};
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
 void gattServApp_ProcessMsg( gattMsgEvent_t *pMsg );
 bStatus_t gattServApp_ProcessExchangeMTUReq( gattMsgEvent_t *pMsg );
 #if( defined( GATT_DEFAULT_FIND_BY_TYPE_ENABLE ) && GATT_DEFAULT_FIND_BY_TYPE_ENABLE )
 bStatus_t gattServApp_ProcessFindByTypeValueReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
 #endif
 bStatus_t gattServApp_ProcessReadByTypeReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
 bStatus_t gattServApp_ProcessReadReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
 #if( defined(GATT_DEFAULT_USE_BLOB_REQ_ENABLE) && GATT_DEFAULT_USE_BLOB_REQ_ENABLE)
 bStatus_t gattServApp_ProcessReadBlobReq( gattMsgEvent_t* pMsg, uint16* pErrHandle );
 #endif
 bStatus_t gattServApp_ProcessReadByGrpTypeReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
 bStatus_t gattServApp_ProcessWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle );
 bStatus_t gattServApp_RegisterServiceCBs( uint16 handle, CONST gattServiceCBs_t *pServiceCBs );
 CONST gattServiceCBs_t *gattServApp_FindServiceCBs( uint16 service );
 pfnGATTReadAttrCB_t gattServApp_FindReadAttrCB( uint16 handle );
 pfnGATTWriteAttrCB_t gattServApp_FindWriteAttrCB( uint16 handle );
 #if( defined( GATT_DEFAULT_PERMISSION_CHECK_ENABLE ) && GATT_DEFAULT_PERMISSION_CHECK_ENABLE)
 pfnGATTAuthorizeAttrCB_t gattServApp_FindAuthorizeAttrCB( uint16 handle );
 #endif

/*********************************************************************
 * API FUNCTIONS
 */
#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
// GATT App Callback functions
bStatus_t gattServApp_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                          uint8 *pValue, uint8 len, uint16 offset );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// GATT Service Callbacks
CONST gattServiceCBs_t gattServiceCBs =
{
  NULL,                    // Read callback function pointer
  gattServApp_WriteAttrCB, // Write callback function pointer
  NULL                     // Authorization callback function pointer
};
#endif

/*********************************************************************
 * @fn      GATTServApp_RegisterForMsgs
 *
 * @brief   Register your task ID to receive event messages from
 *          the GATT Server Application.
 *
 * @param   taskId - Default task ID to send events
 *
 * @return  none
 */
void GATTServApp_RegisterForMsg( uint8 taskID )
{
	appTaskID = taskID;
}

/*********************************************************************
 * @fn      GATTServApp_Init
 *
 * @brief   Initialize the GATT Server Application.
 *
 * @param   taskId - Task identifier for the desired task
 *
 * @return  none
 */
void GATTServApp_Init( uint8 taskId )
{
	GATTServApp_TaskID = taskId;

	// Register to receive incoming ATT Requests
	GATT_RegisterForReq( GATTServApp_TaskID );
}

/*********************************************************************
 * @fn      GATTServApp_ProcessEvent
 *
 * @brief   GATT Server Application Task event processor. This function
 *          is called to process all events for the task. Events include
 *          timers, messages and any other user defined events.
 *
 * @param   task_id - The OSAL assigned task ID.
 * @param   events - events to process. This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GATTServApp_ProcessEvent( uint8 task_id, uint16 events )
{
  if ( events & SYS_EVENT_MSG )
  {
    osal_event_hdr_t *pMsg;

    if ( (pMsg = ( osal_event_hdr_t *)osal_msg_receive( GATTServApp_TaskID )) != NULL )
    {
      // Process incoming messages
      switch ( pMsg->event )
      {
        // Incoming GATT message
        case GATT_MSG_EVENT:
          gattServApp_ProcessMsg( (gattMsgEvent_t *)pMsg );
          break;

        default:
          // Unsupported message
          break;
      }

      // Release the OSAL message
      osal_msg_deallocate( (uint8 *)pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Discard unknown events
  return 0;
}

/******************************************************************************
 * @fn      GATTServApp_RegisterService
 *
 * @brief   Register a service's attribute list and callback functions with
 *          the GATT Server Application.
 *
 * @param   pAttrs - Array of attribute records to be registered
 * @param   numAttrs - Number of attributes in array
 * @param   pServiceCBs - Service callback function pointers
 *
 * @return  SUCCESS: Service registered successfully.
 *          INVALIDPARAMETER: Invalid service field.
 *          FAILURE: Not enough attribute handles available.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t GATTServApp_RegisterService( gattAttribute_t *pAttrs, uint16 numAttrs,
                                       CONST gattServiceCBs_t *pServiceCBs )
{
  uint8 status;

  // First register the service attribute list with GATT Server
  if ( pAttrs != NULL )
  {
    gattService_t service;

    service.attrs = pAttrs;
    service.numAttrs = numAttrs;

    status = GATT_RegisterService( &service );
    if ( ( status == SUCCESS ) && ( pServiceCBs != NULL ) )
    {
      // Register the service CBs with GATT Server Application
      status = gattServApp_RegisterServiceCBs( GATT_SERVICE_HANDLE( pAttrs ),
                                               pServiceCBs );
    }
  }
  else
  {
    status = INVALIDPARAMETER;
  }

  return ( status );
}
/******************************************************************************
 * @fn      GATTServApp_AddService
 *
 * @brief   Add function for the GATT Service.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  SUCCESS: Service added successfully.
 *          INVALIDPARAMETER: Invalid service field.
 *          FAILURE: Not enough attribute handles available.
 *          bleMemAllocError: Memory allocation error occurred.
 */
#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
bStatus_t GATTServApp_AddService( )
{
	uint8 status = SUCCESS;
	// Register GATT attribute list and CBs with GATT Server Application
	status = GATTServApp_RegisterService( gattAttrTbl, GATT_NUM_ATTRS( gattAttrTbl ),
	                              		&gattServiceCBs );
	return ( status );
}
#endif
/******************************************************************************
 * @fn      gattServApp_RegisterServiceCBs
 *
 * @brief   Register callback functions for a service.
 *
 * @param   handle - handle of service being registered
 * @param   pServiceCBs - pointer to service CBs to be registered
 *
 * @return  SUCCESS: Service CBs were registered successfully.
 *          INVALIDPARAMETER: Invalid service CB field.
 *          bleMemAllocError: Memory allocation error occurred.
 */
bStatus_t gattServApp_RegisterServiceCBs( uint16 handle,
                                                 CONST gattServiceCBs_t *pServiceCBs )
{
  serviceCBsList_t *pNewItem;

  // Make sure the service handle is specified
  if ( handle == GATT_INVALID_HANDLE )
  {
    return ( INVALIDPARAMETER );
  }

  // Fill in the new service list
  pNewItem = (serviceCBsList_t *)osal_mem_alloc( sizeof( serviceCBsList_t ) );
  if ( pNewItem == NULL )
  {
    // Not enough memory
    return ( bleMemAllocError );
  }

  // Set up new service CBs item
  pNewItem->next = NULL;
  pNewItem->serviceInfo.handle = handle;
  pNewItem->serviceInfo.pCBs = pServiceCBs;

  // Find spot in list
  if ( serviceCBsList == NULL )
  {
    // First item in list
    serviceCBsList = pNewItem;
  }
  else
  {
    serviceCBsList_t *pLoop = serviceCBsList;

    // Look for end of list
    while ( pLoop->next != NULL )
    {
      pLoop = pLoop->next;
    }

    // Put new item at end of list
    pLoop->next = pNewItem;
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      gattServApp_FindServiceCBs
 *
 * @brief   Find service's callback record.
 *
 * @param   handle - owner of service
 *
 * @return  Pointer to service record. NULL, otherwise.
 */
CONST gattServiceCBs_t *gattServApp_FindServiceCBs( uint16 handle )
{
  serviceCBsList_t *pLoop = serviceCBsList;

  while ( pLoop != NULL )
  {
    if ( pLoop->serviceInfo.handle == handle )
    {
      return ( pLoop->serviceInfo.pCBs );
    }

    // Try next service
    pLoop = pLoop->next;
  }

  return ( (gattServiceCBs_t *)NULL );
}

/*********************************************************************
 * @fn          gattServApp_ProcessMsg
 *
 * @brief       GATT Server App message processing function.
 *
 * @param       pMsg - pointer to received message
 *
 * @return      Success or Failure
 */
void gattServApp_ProcessMsg( gattMsgEvent_t *pMsg )
{
	uint16 errHandle = GATT_INVALID_HANDLE;
	uint8 status;
  
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
	
	#ifdef _PHY_DEBUG 
		LOG("	Msg Opcode 0x%X\n",pMsg->method );
	#endif
	// Process the GATT server message
	switch ( pMsg->method )
	{
//		case ATT_EXCHANGE_MTU_REQ:
//			status = gattServApp_ProcessExchangeMTUReq( pMsg );
//		break;

		#if( defined( GATT_DEFAULT_FIND_BY_TYPE_ENABLE ) && GATT_DEFAULT_FIND_BY_TYPE_ENABLE )
		case ATT_FIND_BY_TYPE_VALUE_REQ:
			status = gattServApp_ProcessFindByTypeValueReq( pMsg, &errHandle );
		break;
		#endif
		
		case ATT_READ_BY_TYPE_REQ:
			status = gattServApp_ProcessReadByTypeReq( pMsg, &errHandle );
		break;

		case ATT_READ_REQ:
			status = gattServApp_ProcessReadReq( pMsg, &errHandle );
		break;

		case ATT_READ_BLOB_REQ:
        	status = gattServApp_ProcessReadBlobReq( pMsg, &errHandle );
        break;
		
		case ATT_READ_BY_GRP_TYPE_REQ:
			status = gattServApp_ProcessReadByGrpTypeReq( pMsg, &errHandle );
		break;

		case ATT_WRITE_REQ:
			status = gattServApp_ProcessWriteReq( pMsg, &errHandle );
		break;

		default:
			// Unknown request - ignore it!
			status = ATT_ERR_UNSUPPORTED_REQ;
		break;
	}

  // See if we need to send an error response back
  if ( status != SUCCESS )
  {
    // Make sure the request was not sent locally
    if ( pMsg->hdr.status != bleNotConnected )
    {
      attErrorRsp_t *pRsp = &rsp.errorRsp;

      pRsp->reqOpcode = pMsg->method;
      pRsp->handle = errHandle;
      pRsp->errCode = status;

      ATT_ErrorRsp( pMsg->connHandle, pRsp );
    }
  }
}

/*********************************************************************
 * @fn          gattServApp_ProcessExchangeMTUReq
 *
 * @brief       Process Exchange MTU Request.
 *
 * @param       pMsg - pointer to received message
 *
 * @return      Success
 */
//bStatus_t gattServApp_ProcessExchangeMTUReq( gattMsgEvent_t *pMsg )
//{
//	attExchangeMTURsp_t *pRsp = &rsp.exchangeMTURsp;
//
//	#ifdef _PHY_DEBUG 
//		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
//	#endif
//	
//	// ATT_MTU shall be set to the minimum of the Client Rx MTU and Server Rx MTU values
//
//	// Set the Server Rx MTU parameter to the maximum MTU that this server can receive
//	pRsp->serverRxMTU = g_ATT_MTU_SIZE_MAX;//ATT_MTU_SIZE;
//
//	// Send response back
//	ATT_ExchangeMTURsp( pMsg->connHandle, pRsp );
//
//	return ( SUCCESS );
//}

/*********************************************************************
 * @fn          gattServApp_ProcessFindByTypeValueReq
 *
 * @brief       Process Find By Type Value Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
#if( defined( GATT_DEFAULT_FIND_BY_TYPE_ENABLE ) && GATT_DEFAULT_FIND_BY_TYPE_ENABLE )
bStatus_t gattServApp_ProcessFindByTypeValueReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attFindByTypeValueReq_t *pReq = &pMsg->msg.findByTypeValueReq;
  attFindByTypeValueRsp_t *pRsp = &rsp.findByTypeValueRsp;
  gattAttribute_t *pAttr;
  uint16 service;

  // Initialize the response
  osal_memset( pRsp, 0, sizeof( attFindByTypeValueRsp_t ) );

  // Only attributes with attribute handles between and including the Starting
  // Handle parameter and the Ending Handle parameter that match the requested
  // attribute type and the attribute value will be returned.

  // All attribute types are effectively compared as 128-bit UUIDs,
  // even if a 16-bit UUID is provided in this request or defined
  // for an attribute.
  pAttr = GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle,
                               pReq->type.uuid, pReq->type.len, &service );

  while ( ( pAttr != NULL ) && ( pRsp->numInfo < g_ATT_MAX_NUM_HANDLES_INFO ) )
  {
    uint16 grpEndHandle;

    // It is not possible to use this request on an attribute that has a value
    // that is longer than (ATT_MTU - 7).
    if ( GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, attrValue,
                               &attrLen, 0, (23-7) ) == SUCCESS )
    {
      // Attribute values should be compared in terms of length and binary representation.
      if ( ( pReq->len == attrLen ) && osal_memcmp( pReq->value, attrValue, attrLen) )
      {
        // New attribute found

        // Set the Found Handle to the attribute that has the exact attribute
        // type and attribute value from the request.
        pRsp->handlesInfo[pRsp->numInfo].handle = pAttr->handle;
      }
    }

    // Try to find the next attribute
    pAttr = GATT_FindNextAttr( pAttr, pReq->endHandle, service, &grpEndHandle );

    // Set Group End Handle
    if ( pRsp->handlesInfo[pRsp->numInfo].handle != 0 )
    {
      // If the attribute type is a grouping attribute, the Group End Handle
      // shall be defined by that higher layer specification. If the attribute
      // type is not a grouping attribute, the Group End Handle shall be equal
      // to the Found Attribute Handle.
      if ( pAttr != NULL )
      {
        pRsp->handlesInfo[pRsp->numInfo++].grpEndHandle = grpEndHandle;
      }
      else
      {
        // If no other attributes with the same attribute type exist after the
        // Found Attribute Handle, the Group End Handle shall be set to 0xFFFF.
        pRsp->handlesInfo[pRsp->numInfo++].grpEndHandle = GATT_MAX_HANDLE;
      }
    }
  } // while

  if ( pRsp->numInfo > 0 )
  {
    // Send a response back
    ATT_FindByTypeValueRsp( pMsg->connHandle, pRsp );

    return ( SUCCESS );
  }

  *pErrHandle = pReq->startHandle;

  return ( ATT_ERR_ATTR_NOT_FOUND );
}
#endif

/*********************************************************************
 * @fn          gattServApp_ProcessReadByTypeReq
 *
 * @brief       Process Read By Type Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadByTypeReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attReadByTypeReq_t *pReq = &pMsg->msg.readByTypeReq;
  attReadByTypeRsp_t *pRsp = &rsp.readByTypeRsp;
  uint16 startHandle = pReq->startHandle;
  uint8 dataLen = 0;
  uint8 status = SUCCESS;

  // Only the attributes with attribute handles between and including the
  // Starting Handle and the Ending Handle with the attribute type that is
  // the same as the Attribute Type given will be returned.

  // Make sure there's enough room at least for an attribute handle (no value)
  while ( dataLen <= (23-4) )
  {
    uint16 service;
    gattAttribute_t *pAttr;

    // All attribute types are effectively compared as 128-bit UUIDs, even if
    // a 16-bit UUID is provided in this request or defined for an attribute.
    pAttr = GATT_FindHandleUUID( startHandle, pReq->endHandle, pReq->type.uuid,
                                 pReq->type.len, &service );
    if ( pAttr == NULL )
    {
      break; // No more attribute found
    }

    // Update start handle so it has the right value if we break from the loop
    startHandle = pAttr->handle;

	#if( defined( GATT_DEFAULT_PERMISSION_CHECK_ENABLE ) && GATT_DEFAULT_PERMISSION_CHECK_ENABLE)
    // Make sure the attribute has sufficient permissions to allow reading
    status = GATT_VerifyReadPermissions( pMsg->connHandle, pAttr->permissions );
    if ( status != SUCCESS )
    { 
      break;
    }
	#endif

    // Read the attribute value. If the attribute value is longer than
    // (ATT_MTU - 4) or 253 octets, whichever is smaller, then the first
    // (ATT_MTU - 4) or 253 octets shall be included in this response.
    status = GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, attrValue,
                                   &attrLen, 0, (23-4) );
    if ( status != SUCCESS )
    { 
      break; // Cannot read the attribute value
    }

    // See if this is the first attribute found
    if ( dataLen == 0 )
    {
      // Use the length of the first attribute value for the length field
      pRsp->len = 2 + attrLen;
    }
    else
    {
      // If the attributes have attribute values that have the same length
      // then these attributes can all be read in a single request.
      if ( pRsp->len != 2 + attrLen )
      {
        break;
      }
    }

    // Make sure there's enough room for this attribute handle and value
    if ( dataLen + attrLen > (23-4) )
    { 
      break;
    }

    // Add the handle value pair to the response
    pRsp->dataList[dataLen++] = LO_UINT16( pAttr->handle );
    pRsp->dataList[dataLen++] = HI_UINT16( pAttr->handle );

    osal_memcpy( &(pRsp->dataList[dataLen]), attrValue, attrLen );
    dataLen += attrLen;

    if ( startHandle == GATT_MAX_HANDLE )
    {
      break; // We're done
    }

    // Update start handle and search again
    startHandle++;
  } // while

  // See what to respond
  if ( dataLen > 0 )
  {
    // Set the number of attribute handle-value pairs found
    pRsp->numPairs = dataLen / pRsp->len;

    // Send a response back
    ATT_ReadByTypeRsp( pMsg->connHandle, pRsp );

     return ( SUCCESS );
  }

  if ( status == SUCCESS )
  { 
    // Attribute not found -- dataLen must be 0
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  *pErrHandle = startHandle;

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessReadReq
 *
 * @brief       Process Read Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
	attReadReq_t *pReq = &pMsg->msg.readReq;
	gattAttribute_t *pAttr;
	uint16 service;
	uint8 status;
  
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
		LOG("	Read Handle %d\n",pReq->handle);
	#endif
	
	pAttr = GATT_FindHandle( pReq->handle, &service );
	if ( pAttr != NULL )
	{
		attReadRsp_t *pRsp = &rsp.readRsp;

		// Build and send a response back. If the attribute value is longer
		// than (ATT_MTU - 1) then (ATT_MTU - 1) octets shall be included
		// in this response.
		status = GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, pRsp->value,
		                               &pRsp->len, 0, (23-1) );
		if ( status == SUCCESS )
		{
			// Send a response back
			ATT_ReadRsp( pMsg->connHandle, pRsp );
		}
	}
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  if ( status != SUCCESS )
  {
    *pErrHandle = pReq->handle;
  }

  return ( status );
}

/*********************************************************************
    @fn          gattServApp_ProcessReadBlobReq

    @brief       Process Read Blob Request.

    @param       pMsg - pointer to received message
    @param       pErrHandle - attribute handle that generates an error

    @return      Success or Failure
*/
#if( defined(GATT_DEFAULT_USE_BLOB_REQ_ENABLE) && GATT_DEFAULT_USE_BLOB_REQ_ENABLE)
bStatus_t gattServApp_ProcessReadBlobReq( gattMsgEvent_t* pMsg, uint16* pErrHandle )
{
    attReadBlobReq_t* pReq = &pMsg->msg.readBlobReq;
    gattAttribute_t* pAttr;
    uint16 service;
    uint8 status;
    pAttr = GATT_FindHandle( pReq->handle, &service );

    if ( pAttr != NULL )
    {
        attReadBlobRsp_t* pRsp = &rsp.readBlobRsp;
        // Read part attribute value. If the attribute value is longer than
        // (Value Offset + ATT_MTU - 1) then (ATT_MTU - 1) octets from Value
        // Offset shall be included in this response.
        status = GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, pRsp->value,
                                       &pRsp->len, pReq->offset, (23-1) );

        if ( status == SUCCESS )
        {
            // Send a response back
            ATT_ReadBlobRsp( pMsg->connHandle, pRsp );
        }
    }
    else
    {
        status = ATT_ERR_INVALID_HANDLE;
    }

    if ( status != SUCCESS )
    {
        *pErrHandle = pReq->handle;
    }

    return ( status );
}
#endif

/*********************************************************************
 * @fn          gattServApp_ProcessReadByGrpTypeReq
 *
 * @brief       Process Read By Group Type Request.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessReadByGrpTypeReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attReadByGrpTypeReq_t *pReq = &pMsg->msg.readByGrpTypeReq;
  attReadByGrpTypeRsp_t *pRsp = &rsp.readByGrpTypeRsp;
  uint16 service;
  gattAttribute_t *pAttr;
  uint8 dataLen = 0;
  uint8 status = SUCCESS;

  // Only the attributes with attribute handles between and including the
  // Starting Handle and the Ending Handle with the attribute type that is
  // the same as the Attribute Type given will be returned.

  // All attribute types are effectively compared as 128-bit UUIDs,
  // even if a 16-bit UUID is provided in this request or defined
  // for an attribute.
  pAttr = GATT_FindHandleUUID( pReq->startHandle, pReq->endHandle,
                               pReq->type.uuid, pReq->type.len, &service );
  while ( pAttr != NULL )
  {
    uint16 endGrpHandle;

    // The service, include and characteristic declarations are readable and
    // require no authentication or authorization, therefore insufficient
    // authentication or read not permitted errors shall not occur.
    #if( defined( GATT_DEFAULT_PERMISSION_CHECK_ENABLE ) && GATT_DEFAULT_PERMISSION_CHECK_ENABLE)
    status = GATT_VerifyReadPermissions( pMsg->connHandle, pAttr->permissions );
    if ( status != SUCCESS )
    {
      *pErrHandle = pAttr->handle;

      break;
    }
	#endif
    // Read the attribute value. If the attribute value is longer than
    // (ATT_MTU - 6) or 251 octets, whichever is smaller, then the first
    // (ATT_MTU - 6) or 251 octets shall be included in this response.
    status = GATTServApp_ReadAttr( pMsg->connHandle, pAttr, service, attrValue,
                                   &attrLen, 0, (23-6) );
    if ( status != SUCCESS )
    {
      // Cannot read the attribute value
      *pErrHandle = pAttr->handle;

      break;
    }

    // See if this is the first attribute found
    if ( dataLen == 0 )
    {
      // Use the length of the first attribute value for the length field
      pRsp->len = 2 + 2 + attrLen;
    }
    else
    {
      // If the attributes have attribute values that have the same length
      // then these attributes can all be read in a single request.
      if ( pRsp->len != 2 + 2 + attrLen )
      {
        break; // We're done here
      }

      // Make sure there's enough room for this attribute handle, end group handle and value
      if ( dataLen + attrLen > (23-6) )
      {
        break; // We're done here
      }
    }

    // Add Attribute Handle to the response
    pRsp->dataList[dataLen++] = LO_UINT16( pAttr->handle );
    pRsp->dataList[dataLen++] = HI_UINT16( pAttr->handle );

    // Try to find the next attribute
    pAttr = GATT_FindNextAttr( pAttr, pReq->endHandle, service, &endGrpHandle );

    // Add End Group Handle to the response
    if ( pAttr != NULL )
    {
      // The End Group Handle is the handle of the last attribute within the
      // service definition
      pRsp->dataList[dataLen++] = LO_UINT16( endGrpHandle );
      pRsp->dataList[dataLen++] = HI_UINT16( endGrpHandle );
    }
    else
    {
      // The ending handle of the last service can be 0xFFFF
      pRsp->dataList[dataLen++] = LO_UINT16( GATT_MAX_HANDLE );
      pRsp->dataList[dataLen++] = HI_UINT16( GATT_MAX_HANDLE );
    }

    // Add Attribute Value to the response
    osal_memcpy( &(pRsp->dataList[dataLen]), attrValue, attrLen );
    dataLen += attrLen;
  } // while

  // See what to respond
  if ( dataLen > 0 )
  {
    // Set the number of attribute handle, end group handle and value sets found
    pRsp->numGrps = dataLen / pRsp->len;

    // Send a response back
    ATT_ReadByGrpTypeRsp( pMsg->connHandle, pRsp );

    return ( SUCCESS );
  }

  if ( status == SUCCESS )
  {
    // No grouping attribute found -- dataLen must be 0
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  *pErrHandle = pReq->startHandle;

  return ( status );
}

/*********************************************************************
 * @fn          gattServApp_ProcessWriteReq
 *
 * @brief       Process Write Request or Command.
 *
 * @param       pMsg - pointer to received message
 * @param       pErrHandle - attribute handle that generates an error
 *
 * @return      Success or Failure
 */
bStatus_t gattServApp_ProcessWriteReq( gattMsgEvent_t *pMsg, uint16 *pErrHandle )
{
  attWriteReq_t *pReq = &(pMsg->msg.writeReq);
  gattAttribute_t *pAttr;
  uint16 service;
  uint8 status = SUCCESS;

  // No Error Response or Write Response shall be sent in response to Write
  // Command. If the server cannot write this attribute for any reason the
  // command shall be ignored.
  pAttr = GATT_FindHandle( pReq->handle, &service );
  if ( pAttr != NULL )
  {
	#if( defined( GATT_DEFAULT_PERMISSION_CHECK_ENABLE ) && GATT_DEFAULT_PERMISSION_CHECK_ENABLE)
    // Authorization is handled by the application/profile
    if ( gattPermitAuthorWrite( pAttr->permissions ) )
    { 
      // Use Service's authorization callback to authorize the request
      pfnGATTAuthorizeAttrCB_t pfnCB = gattServApp_FindAuthorizeAttrCB( service );
      if ( pfnCB != NULL )
      { 
          status = (*pfnCB)( pMsg->connHandle, pAttr, ATT_WRITE_REQ );
      }
      else
      {  
        status = ATT_ERR_UNLIKELY;
      }
    }
	#endif

    // If everything is fine then try to write the new value
    if ( status == SUCCESS )
    {  
      // Use Service's write callback to write the request
      status = GATTServApp_WriteAttr( pMsg->connHandle, pReq->handle,
                                      pReq->value, pReq->len, 0 );
//	  LOG("%s,status %d\n",__func__,status);
      if ( ( status == SUCCESS ) && ( pReq->cmd == FALSE ) )
      {  
        // Send a response back
        ATT_WriteRsp( pMsg->connHandle );
      }
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  if ( status != SUCCESS )
  {
    *pErrHandle = pReq->handle; 
  }

  return ( pReq->cmd ? SUCCESS : status );
}

/*********************************************************************
 * @fn      gattServApp_FindReadAttrCB
 *
 * @brief   Find the Read Attribute CB function pointer for a given service.
 *
 * @param   handle - service attribute handle
 *
 * @return  pointer to the found CB. NULL, otherwise.
 */
pfnGATTReadAttrCB_t gattServApp_FindReadAttrCB( uint16 handle )
{
  CONST gattServiceCBs_t *pCBs = gattServApp_FindServiceCBs( handle );

  return ( ( pCBs == NULL ) ? NULL : pCBs->pfnReadAttrCB );
}

/*********************************************************************
 * @fn      gattServApp_FindWriteAttrCB
 *
 * @brief   Find the Write CB Attribute function pointer for a given service.
 *
 * @param   handle - service attribute handle
 *
 * @return  pointer to the found CB. NULL, otherwise.
 */
pfnGATTWriteAttrCB_t gattServApp_FindWriteAttrCB( uint16 handle )
{
  CONST gattServiceCBs_t *pCBs = gattServApp_FindServiceCBs( handle );

  return ( ( pCBs == NULL ) ? NULL : pCBs->pfnWriteAttrCB );
}

/*********************************************************************
 * @fn      gattServApp_FindAuthorizeAttrCB
 *
 * @brief   Find the Authorize Attribute CB function pointer for a given service.
 *
 * @param   handle - service attribute handle
 *
 * @return  pointer to the found CB. NULL, otherwise.
 */
#if( defined( GATT_DEFAULT_PERMISSION_CHECK_ENABLE ) && GATT_DEFAULT_PERMISSION_CHECK_ENABLE)
pfnGATTAuthorizeAttrCB_t gattServApp_FindAuthorizeAttrCB( uint16 handle )
{
  CONST gattServiceCBs_t *pCBs = gattServApp_FindServiceCBs( handle );

  return ( ( pCBs == NULL ) ? NULL : pCBs->pfnAuthorizeAttrCB );
}
#endif

/*********************************************************************
 * @fn      gattServApp_ValidateWriteAttrCB
 *
 * @brief   Validate and/or Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
#if( defined( SERV_GATT_SERV ) && SERV_GATT_SERV )
bStatus_t gattServApp_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                          uint8 *pValue, uint8 len, uint16 offset )
{
	bStatus_t status = SUCCESS;
	#ifdef _PHY_DEBUG 
		LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif
	#ifdef _PHY_DEBUG 
		LOG("	Attribute Type Length %d\n",pAttr->type.len);
	#endif

	if ( pAttr->type.len == ATT_BT_UUID_SIZE )
	{  
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
		#ifdef _PHY_DEBUG 
		LOG("	uuid 0x%X\n", uuid );
		#endif
		switch ( uuid )
		{
			case GATT_CLIENT_CHAR_CFG_UUID:
				serviceChange_indCharCfg = BUILD_UINT16( pValue[0], pValue[1]);
			break;

			default:
			// Should never get here!
			status = ATT_ERR_INVALID_HANDLE;
			break;
		}
	}
	else
	{ 
		// 128-bit UUID
		status = ATT_ERR_INVALID_HANDLE;
	}

	return ( status );
}
#endif
/*********************************************************************
 * @fn          GATTServApp_ReadAttr
 *
 * @brief       Read an attribute. If the format of the attribute value
 *              is unknown to GATT Server, use the callback function
 *              provided by the Service.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       service - handle of owner service
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */

uint8 GATTServApp_ReadAttr( uint16 connHandle, gattAttribute_t *pAttr,
                            uint16 service, uint8 *pValue, uint8 *pLen,
                            uint16 offset, uint8 maxLen )
{
	uint8 useCB = FALSE;
	bStatus_t status = SUCCESS;
	#ifdef _PHY_DEBUG 
	LOG("%s,%s,Line %d\n",__FILE__,__func__,__LINE__);
	#endif

	#if( defined( GATT_DEFAULT_PERMISSION_CHECK_ENABLE ) && GATT_DEFAULT_PERMISSION_CHECK_ENABLE)
	// Authorization is handled by the application/profile
	if ( gattPermitAuthorRead( pAttr->permissions ) )
	{
		// TODO this branch code 22B
		// Use Service's authorization callback to authorize the request
		pfnGATTAuthorizeAttrCB_t pfnCB = gattServApp_FindAuthorizeAttrCB( service );
		if ( pfnCB != NULL )
		{
			status = (*pfnCB)( connHandle, pAttr, ATT_READ_REQ );
		}
		else
		{
			status = ATT_ERR_UNLIKELY;
		}

		if ( status != SUCCESS )
		{
			// Read operation failed!
			return ( status );
		}
	}
	#endif
	// Check the UUID length
	if ( pAttr->type.len == ATT_BT_UUID_SIZE )
	{
		// 16-bit UUID
		uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
		#ifdef _PHY_DEBUG 
		LOG("	uuid 0x%X\n",uuid);
		#endif
		switch ( uuid )
		{
			case GATT_PRIMARY_SERVICE_UUID:
			case GATT_SECONDARY_SERVICE_UUID:
			{
				// Make sure it's not a blob operation
				if ( offset == 0 )
				{
					gattAttrType_t *pType = (gattAttrType_t *)(pAttr->pValue);

					*pLen = pType->len;
					osal_memcpy( pValue, pType->uuid, pType->len );
				}
				else
				{
					status = ATT_ERR_ATTR_NOT_LONG;
				}
				break;
			}
			case GATT_CHARACTER_UUID:
			{
				// Make sure it's not a blob operation
				if ( offset == 0 )
				{
					gattAttribute_t *pCharValue;

					// The Attribute Value of a Characteristic Declaration includes the
					// Characteristic Properties, Characteristic Value Attribute Handle
					// and UUID.
					*pLen = 1;
					pValue[0] = *pAttr->pValue; // Properties

					// The Characteristic Value Attribute exists immediately following
					// the Characteristic Declaration.
					pCharValue = GATT_FindHandle( pAttr->handle+1, NULL );
					if ( pCharValue != NULL )
					{
						// It can be a 128-bit UUID
						*pLen += (2 + pCharValue->type.len);

						// Attribute Handle
						pValue[1] = LO_UINT16( pCharValue->handle );
						pValue[2] = HI_UINT16( pCharValue->handle );

						// Attribute UUID
						osal_memcpy( &(pValue[3]), pCharValue->type.uuid, pCharValue->type.len );
					}
					else
					{
						// Should never get here!
						*pLen += (2 + ATT_BT_UUID_SIZE);

						// Set both Attribute Handle and UUID to 0
						osal_memset( &(pValue[1]), 0, (2 + ATT_BT_UUID_SIZE) );
					}
				}
				else
				{
				status = ATT_ERR_ATTR_NOT_LONG;
				}
			}
			break;

			case GATT_CLIENT_CHAR_CFG_UUID:
			{
				// TODO
				#ifdef _PHY_DEBUG 
					LOG("	uuid 0x%X\n",uuid);
				#endif
				*pLen = 2;
				pValue[0] = pAttr->pValue[0];
				pValue[1] = pAttr->pValue[1];
			}
			break;
			
			#if( defined( GATT_DEFAULT_USER_DESCRIPTON_ENABLE ) && GATT_DEFAULT_USER_DESCRIPTON_ENABLE)
			case GATT_CHAR_USER_DESC_UUID:
			{
				uint8 len = osal_strlen( (char *)(pAttr->pValue) ); // Could be a long attribute

				// If the value offset of the Read Blob Request is greater than the
				// length of the attribute value, an Error Response shall be sent with
				// the error code Invalid Offset.
				if ( offset <= len )
				{
					// If the value offset is equal than the length of the attribute
					// value, then the length of the part attribute value shall be zero.
					if ( offset == len )
					{
						len = 0;
					}
					else
					{
						// If the attribute value is longer than (Value Offset + maxLen)
						// then maxLen octets from Value Offset shall be included in
						// this response.
						if ( len > ( offset + maxLen ) )
						{
							len = maxLen;
						}
						else
						{
							len -= offset;
						}
					}

					*pLen = len;
					osal_memcpy( pValue, &(pAttr->pValue[offset]), len );
				}
				else
				{
					status = ATT_ERR_INVALID_OFFSET;
				}
			}
			break;
			#endif
			default:
			{
				useCB = TRUE;
				#ifdef _PHY_DEBUG 
					LOG("	unsupported uuid=0x%X or application uuid\n",uuid);
				#endif
			}
			break;
		}
	}
	else
	{
		useCB = TRUE;
	}

	if ( useCB == TRUE )
	{
		// Use Service's read callback to process the request
		pfnGATTReadAttrCB_t pfnCB = gattServApp_FindReadAttrCB( service );
		if ( pfnCB != NULL )
		{
			// Read the attribute value
			status = (*pfnCB)( connHandle, pAttr, pValue, pLen, offset, maxLen );
		}
		else
		{
			status = ATT_ERR_UNLIKELY;
		}
	}
  return ( status );
}

/*********************************************************************
 * @fn      GATTServApp_WriteAttr
 *
 * @brief   Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   handle - attribute handle
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
uint8 GATTServApp_WriteAttr( uint16 connHandle, uint16 handle,
                             uint8 *pValue, uint16 len, uint16 offset )
{
  uint16 service;
  gattAttribute_t *pAttr;
  bStatus_t status;

  // Find the owner of the attribute
  pAttr = GATT_FindHandle( handle, &service );
  if ( pAttr != NULL )
  { 
    // Find out the owner's callback functions
    pfnGATTWriteAttrCB_t pfnCB = gattServApp_FindWriteAttrCB( service );
    if ( pfnCB != NULL )
    {  
      // Try to write the new value
      status = (*pfnCB)( connHandle, pAttr, pValue, len, offset );
    }
    else
    {  
      status = ATT_ERR_UNLIKELY;
    }
  }
  else
  {  
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/****************************************************************************
****************************************************************************/
