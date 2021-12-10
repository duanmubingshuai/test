/*******************************************************************************
  Filename:       hci_tl.c
  Revised:        
  Revision:       

  Description:    This file includes implementation for HCI task, event handler,
                  HCI Command, Data, and Event procoessing and sending, for the
                  BLE Transport Layer.


*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include "hci_tl.h"
#include "osal_bufmgr.h"

extern uint32 bleEvtMask;
extern uint8 pHciEvtMask[];
extern uint8 hciPTMenabled;

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

/* States for Command and Data packet parser */
#define HCI_PARSER_STATE_PKT_TYPE          0
//
#define HCI_CMD_PARSER_STATE_OPCODE        1
#define HCI_CMD_PARSER_STATE_LENGTH        2
#define HCI_CMD_PARSER_STATE_DATA          3
//
#define HCI_DATA_PARSER_STATE_HANDLE       4
#define HCI_DATA_PARSER_STATE_LENGTH       5
#define HCI_DATA_PARSER_STATE_DATA         6

// HCI Command Subgroup
#define HCI_OPCODE_CSG_LINK_LAYER          0
#define HCI_OPCODE_CSG_CSG_L2CAP           1
#define HCI_OPCODE_CSG_CSG_ATT             2
#define HCI_OPCODE_CSG_CSG_GATT            3
#define HCI_OPCODE_CSG_CSG_GAP             4
#define HCI_OPCODE_CSG_CSG_SM              5
#define HCI_OPCODE_CSG_CSG_Reserved        6
#define HCI_OPCODE_CSG_CSG_USER_PROFILE    7

// Vendor Specific OGF
#define VENDOR_SPECIFIC_OGF                0x3F

/*******************************************************************************
 * TYPEDEFS
 */

typedef hciStatus_t (*hciFunc_t)( uint8 *pBuf );

typedef struct
{
  uint16    opCode;
  hciFunc_t hciFunc;
} hciCmdFunc_t;

typedef const hciCmdFunc_t cmdPktTable_t;

/*******************************************************************************
 * LOCAL VARIABLES
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

uint8 hciTaskID;
uint8 hciTestTaskID;
//
uint8 hciGapTaskID;
uint8 hciL2capTaskID;
uint8 hciSmpTaskID;
uint8 hciExtTaskID;

/*
** Controller Prototypes
*/

/*
** HCI OSAL API
*/

/*******************************************************************************
 * This is the HCI OSAL task initialization routine.
 *
 * Public function defined in hci.h.
 */
void HCI_Init0( uint8 taskID )
{
  // initialize the task for HCI-Controller
  hciTaskID      = taskID;
  hciTestTaskID  = 0;
  hciGapTaskID   = 0;
  hciL2capTaskID = 0;
  hciSmpTaskID   = 0;
  hciExtTaskID   = 0;

  // reset the Bluetooth and the BLE event mask bits
  hciInitEventMasks();

  // disable PTM runtime flag
  hciPTMenabled = FALSE;

  return;
}


/*******************************************************************************
 * This is the HCI OSAL task process event handler.
 *
 * Public function defined in hci.h.
 */
uint16 HCI_ProcessEvent0( uint8 task_id, uint16 events )
{
  osal_event_hdr_t *pMsg;

  // check for system messages
  if ( events & SYS_EVENT_MSG )
  {
    pMsg = (osal_event_hdr_t *)osal_msg_receive(hciTaskID);
    if ( pMsg )
    {

      if ( (pMsg->event == HCI_HOST_TO_CTRL_DATA_EVENT) ||
           (pMsg->event == HCI_CTRL_TO_HOST_EVENT) )
      {
        // deallocate data
        osal_bm_free( ((hciDataPacket_t *)pMsg)->pData );
      }

      // deallocate the message
      (void)osal_msg_deallocate( (uint8 *)pMsg );
    }

    // return unproccessed events
    return( events ^ SYS_EVENT_MSG );
  }

  return( 0 );
}

/*
** HCI Vendor Specific Handlers for Host
*/

/*******************************************************************************
 * Register GAP task ID with HCI.
 *
 * Public function defined in hci.h.
 */
void HCI_TestAppTaskRegister( uint8 taskID )
{
  hciTestTaskID = taskID;
}


/*******************************************************************************
 * Register GAP task ID with HCI.
 *
 * Public function defined in hci.h.
 */
void HCI_GAPTaskRegister( uint8 taskID )
{
  hciGapTaskID = taskID;
}


/*******************************************************************************
 * Register L2CAP task ID with HCI.
 *
 * Public function defined in hci.h.
 */
void HCI_L2CAPTaskRegister( uint8 taskID )
{
  hciL2capTaskID = taskID;
}


/*******************************************************************************
 * Register SMP task ID with HCI.
 *
 * Public function defined in hci.h.
 */
void HCI_SMPTaskRegister( uint8 taskID )
{
  hciSmpTaskID = taskID;
}


/*******************************************************************************
 * Register task ID to receive HCI Extended Command.
 *
 * Public function defined in hci.h.
 */
void HCI_ExtTaskRegister( uint8 taskID )
{
  hciExtTaskID = taskID;
}

/*******************************************************************************
 */
