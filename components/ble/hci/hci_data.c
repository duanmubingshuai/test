/*******************************************************************************
  Filename:       hci_c_data.c
  Revised:        
  Revision:       

  Description:    This file handles HCI data for the controller

 
*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include "bus_dev.h"
#include "osal_bufmgr.h"
#include "ll_common.h"
#include "hci_tl.h"
#include "hci_data.h"

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * EXTERNS
 */

extern uint8  ctrlToHostEnable;
extern uint16 numHostBufs;


/*
** Data API
*/

/*******************************************************************************
 * This function is used to reverse the order of the bytes in an array in place.
 *
 * Public function defined in hci_c_data.h.
 */
void HCI_ReverseBytes( uint8 *buf,
                       uint8 len )
{
  uint8 temp;
  uint8 index = (uint8)(len - 1);
  uint8 i;

  // adjust length as only half the operations are needed
  len >>= 1;

  // reverse the order of the bytes
  for (i=0; i<len; i++)
  {
    temp           = buf[i];
    buf[i]         = buf[index - i];
    buf[index - i] = temp;
  }

  return;
}


/*
** LL Callbacks
*/

/*******************************************************************************
 * This Callback is used by the LL to indicate to the HCI that data has been
 * received and placed in the buffer provided by the HCI.
 *
 * Public function defined in hci_c_data.h.
 */
void LL_RxDataCompleteCback( uint16 connHandle,
                             uint8  *pBuf,
                             uint8  len,
                             uint8  fragFlag,
                             int8   rssi )
{
  // unused input parameter; PC-Lint error 715.
  (void)rssi;

  // check if Controller to Host flow control is enabled
  if ( ctrlToHostEnable == TRUE )
  {

    // decrement the number of available Host buffers, and check if exhausted
    if ( --numHostBufs == 0 )
    {
      // enable LL Rx flow control
      LL_CtrlToHostFlowControl( LL_ENABLE_RX_FLOW_CONTROL );
    }
  }

  // check if this is for the Host
  if ( hciL2capTaskID != 0 )
  {
    hciDataEvent_t *msg;

    // allocated space for OSAL message
    msg = (hciDataEvent_t *)osal_msg_allocate( sizeof ( hciDataEvent_t ) );

    if ( msg )
    {
			
      // message header
      msg->hdr.event  = HCI_DATA_EVENT;
      msg->hdr.status = 0xFF;
         
      // message data
      msg->connHandle = connHandle;
      msg->pbFlag     = fragFlag;
      msg->len        = len;
      msg->pData      = pBuf;

      // send message
      (void)osal_msg_send( hciL2capTaskID, (uint8 *)msg );
    }
    else // can't queue this packet
    {
      // so free the packet buffer
      osal_bm_free( pBuf );
    }
  }
  else // not Host, so route to HCI transport layer
  {
    hciPacket_t *msg;

    // allocate an OSAL HCI message
    msg = (hciPacket_t *)osal_msg_allocate( sizeof(hciPacket_t) );

    if ( msg )
    {
      // message header
      msg->hdr.event  = HCI_CTRL_TO_HOST_EVENT;
      msg->hdr.status = 0xFF;
      msg->pData      = pBuf-HCI_DATA_MIN_LENGTH;

      // create HCI packet
      // Note: The payload offset starting after the packet header was given to
      //       the LL for receiving data.
      msg->pData[0] = HCI_ACL_DATA_PACKET;
      msg->pData[1] = LO_UINT16(connHandle);
      msg->pData[2] = HI_UINT16(connHandle) | (fragFlag << 4); // packet boundary flags
      msg->pData[3] = len;
      msg->pData[4] = 0x00;                                    // LL length only 1 byte

      // send message
      (void)osal_msg_send( hciTaskID, (uint8 *)msg );
    }
    else // can't send this packet
    {
      // so free the packet buffer
      osal_bm_free( pBuf );
    }
  }
}


/*******************************************************************************
 */
