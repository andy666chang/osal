/*
 * @Author: andy.chang 
 * @Date: 2024-07-28 00:49:38 
 * @Last Modified by: andy.chang
 * @Last Modified time: 2024-07-28 20:51:29
 */

/*********************************************************************
 * INCLUDES
 */

#include <string.h>

#include "type.h"
#include "osal.h"
#include "osal_tasks.h"
#include "osal_memory.h"
// #include "osal_PwrMgr.h"
#include "osal_msg.h"
#include "osal_clock.h"

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

// Message Pool Definitions
static osal_msg_q_t osal_qHead = NULL;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

static uint8 osal_msg_enqueue_push( uint8 destination_task, uint8 *msg_ptr, uint8 urgent );

/*********************************************************************
 * HELPER FUNCTIONS
 */

/*********************************************************************
 * API FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osal_msg_allocate
 *
 * @brief
 *
 *    This function is called by a task to allocate a message buffer
 *    into which the task will encode the particular message it wishes
 *    to send.  This common buffer scheme is used to strictly limit the
 *    creation of message buffers within the system due to RAM size
 *    limitations on the microprocessor.   Note that all message buffers
 *    are a fixed size (at least initially).  The parameter len is kept
 *    in case a message pool with varying fixed message sizes is later
 *    created (for example, a pool of message buffers of size LARGE,
 *    MEDIUM and SMALL could be maintained and allocated based on request
 *    from the tasks).
 *
 *
 * @param   uint8 len  - wanted buffer length
 *
 *
 * @return  pointer to allocated buffer or NULL if allocation failed.
 */
uint8 * osal_msg_allocate( uint16 len )
{
  osal_msg_hdr_t *hdr;

  if ( len == 0 )
    return ( NULL );

  hdr = (osal_msg_hdr_t *) osal_mem_alloc( (short)(len + sizeof( osal_msg_hdr_t )) );
  if ( hdr )
  {
    hdr->next = NULL;
    hdr->len = len;
    hdr->dest_id = TASK_NO_TASK;
    return ( (uint8 *) (hdr + 1) );
  }
  else
    return ( NULL );
}

/*********************************************************************
 * @fn      osal_msg_deallocate
 *
 * @brief
 *
 *    This function is used to deallocate a message buffer. This function
 *    is called by a task (or processing element) after it has finished
 *    processing a received message.
 *
 *
 * @param   uint8 *msg_ptr - pointer to new message buffer
 *
 * @return  SUCCESS, INVALID_MSG_POINTER
 */
uint8 osal_msg_deallocate( uint8 *msg_ptr )
{
  uint8 *x;

  if ( msg_ptr == NULL )
    return ( INVALID_MSG_POINTER );

  // don't deallocate queued buffer
  if ( OSAL_MSG_ID( msg_ptr ) != TASK_NO_TASK )
    return ( MSG_BUFFER_NOT_AVAIL );

  x = (uint8 *)((uint8 *)msg_ptr - sizeof( osal_msg_hdr_t ));

  osal_mem_free( (void *)x );

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      osal_msg_send
 *
 * @brief
 *
 *    This function is called by a task to send a command message to
 *    another task or processing element.  The sending_task field must
 *    refer to a valid task, since the task ID will be used
 *    for the response message.  This function will also set a message
 *    ready event in the destination tasks event list.
 *
 *
 * @param   uint8 destination_task - Send msg to Task ID
 * @param   uint8 *msg_ptr - pointer to new message buffer
 *
 * @return  SUCCESS, INVALID_TASK, INVALID_MSG_POINTER
 */
uint8 osal_msg_send( uint8 destination_task, uint8 *msg_ptr )
{
  return ( osal_msg_enqueue_push( destination_task, msg_ptr, FALSE ) );
}

/*********************************************************************
 * @fn      osal_msg_push_front
 *
 * @brief
 *
 *    This function is called by a task to push a command message
 *    to the head of the OSAL queue. The destination_task field
 *    must refer to a valid task, since the task ID will be used to
 *    send the message to. This function will also set a message
 *    ready event in the destination task's event list.
 *
 * @param   uint8 destination_task - Send msg to Task ID
 * @param   uint8 *msg_ptr - pointer to message buffer
 *
 * @return  SUCCESS, INVALID_TASK, INVALID_MSG_POINTER
 */
uint8 osal_msg_push_front( uint8 destination_task, uint8 *msg_ptr )
{
  return ( osal_msg_enqueue_push( destination_task, msg_ptr, TRUE ) );
}

/*********************************************************************
 * @fn      osal_msg_enqueue_push
 *
 * @brief
 *
 *    This function is called by a task to either enqueue (append to
 *    queue) or push (prepend to queue) a command message to the OSAL
 *    queue. The destination_task field must refer to a valid task,
 *    since the task ID will be used to send the message to. This 
 *    function will also set a message ready event in the destination
 *    task's event list.
 *
 * @param   uint8 destination_task - Send msg to Task ID
 * @param   uint8 *msg_ptr - pointer to message buffer
 * @param   uint8 push - TRUE to push, otherwise enqueue
 *
 * @return  SUCCESS, INVALID_TASK, INVALID_MSG_POINTER
 */
static uint8 osal_msg_enqueue_push( uint8 destination_task, uint8 *msg_ptr, uint8 push )
{
  if ( msg_ptr == NULL )
  {
    return ( INVALID_MSG_POINTER );
  }

  if ( !osal_task_valid(destination_task) )
  {
    osal_msg_deallocate( msg_ptr );
    return ( INVALID_TASK );
  }

  // Check the message header
  if ( OSAL_MSG_NEXT( msg_ptr ) != NULL ||
       OSAL_MSG_ID( msg_ptr ) != TASK_NO_TASK )
  {
    osal_msg_deallocate( msg_ptr );
    return ( INVALID_MSG_POINTER );
  }

  OSAL_MSG_ID( msg_ptr ) = destination_task;

  if ( push == TRUE )
  {
    // prepend the message
    osal_msg_push( &osal_qHead, msg_ptr );
  }
  else
  {
    // append the message
    osal_msg_enqueue( &osal_qHead, msg_ptr );
  }

  // Signal the task that a message is waiting
  osal_set_event( destination_task, SYS_EVENT_MSG );

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      osal_msg_receive
 *
 * @brief
 *
 *    This function is called by a task to retrieve a received command
 *    message. The calling task must deallocate the message buffer after
 *    processing the message using the osal_msg_deallocate() call.
 *
 * @param   uint8 task_id - receiving tasks ID
 *
 * @return  *uint8 - message information or NULL if no message
 */
uint8 *osal_msg_receive( uint8 task_id )
{
  osal_msg_hdr_t *listHdr;
  osal_msg_hdr_t *prevHdr = NULL;
  osal_msg_hdr_t *foundHdr = NULL;
  halIntState_t   intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  // Point to the top of the queue
  listHdr = osal_qHead;

  // Look through the queue for a message that belongs to the asking task
  while ( listHdr != NULL )
  {
    if ( (listHdr - 1)->dest_id == task_id )
    {
      if ( foundHdr == NULL )
      {
        // Save the first one
        foundHdr = listHdr;
      }
      else
      {
        // Second msg found, stop looking
        break;
      }
    }
    if ( foundHdr == NULL )
    {
      prevHdr = listHdr;
    }
    listHdr = OSAL_MSG_NEXT( listHdr );
  }

  // Is there more than one?
  if ( listHdr != NULL )
  {
    // Yes, Signal the task that a message is waiting
    osal_set_event( task_id, SYS_EVENT_MSG );
  }
  else
  {
    // No more
    osal_clear_event( task_id, SYS_EVENT_MSG );
  }

  // Did we find a message?
  if ( foundHdr != NULL )
  {
    // Take out of the link list
    osal_msg_extract( &osal_qHead, foundHdr, prevHdr );
  }

  // Release interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);

  return ( (uint8*) foundHdr );
}

/**************************************************************************************************
 * @fn          osal_msg_find
 *
 * @brief       This function finds in place an OSAL message matching the task_id and event
 *              parameters.
 *
 * input parameters
 *
 * @param       task_id - The OSAL task id that the enqueued OSAL message must match.
 * @param       event - The OSAL event id that the enqueued OSAL message must match.
 *
 * output parameters
 *
 * None.
 *
 * @return      NULL if no match, otherwise an in place pointer to the matching OSAL message.
 **************************************************************************************************
 */
osal_event_hdr_t *osal_msg_find(uint8 task_id, uint8 event)
{
  osal_msg_hdr_t *pHdr;
  halIntState_t intState;

  HAL_ENTER_CRITICAL_SECTION(intState);  // Hold off interrupts.

  pHdr = osal_qHead;  // Point to the top of the queue.

  // Look through the queue for a message that matches the task_id and event parameters.
  while (pHdr != NULL)
  {
    if (((pHdr-1)->dest_id == task_id) && (((osal_event_hdr_t *)pHdr)->event == event))
    {
      break;
    }

    pHdr = OSAL_MSG_NEXT(pHdr);
  }

  HAL_EXIT_CRITICAL_SECTION(intState);  // Release interrupts.

  return (osal_event_hdr_t *)pHdr;
}

/**************************************************************************************************
 * @fn          osal_msg_count
 *
 * @brief       This function counts the number of messages, in the OSAL message queue with a
 *              a given task ID and message event type.
 *
 * input parameters
 *
 * @param       task_id - The OSAL task id that the enqueued OSAL message must match.
 * @param       event - The OSAL event id that the enqueued OSAL message must match. 0xFF for 
 *              all events.
 *
 * output parameters
 *
 * None.
 *
 * @return      The number of OSAL messages that match the task ID and Event.
 **************************************************************************************************
 */
uint8 osal_msg_count( uint8 task_id, uint8 event )
{
  uint8 count = 0;
  osal_msg_hdr_t *pHdr;
  halIntState_t intState;

  HAL_ENTER_CRITICAL_SECTION(intState);  // Hold off interrupts.

  pHdr = osal_qHead;  // Point to the top of the queue.

  // Look through the queue for a message that matches the task_id and event parameters.
  while (pHdr != NULL)
  {
    if ( ((pHdr-1)->dest_id == task_id) 
        && ((event == 0xFF) || (((osal_event_hdr_t *)pHdr)->event == event)) )
    {
      count++;
    }

    pHdr = OSAL_MSG_NEXT(pHdr);
  }

  HAL_EXIT_CRITICAL_SECTION(intState);  // Release interrupts.

  return ( count );
}

/*********************************************************************
 * @fn      osal_msg_enqueue
 *
 * @brief
 *
 *    This function enqueues an OSAL message into an OSAL queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message
 *
 * @return  none
 */
void osal_msg_enqueue( osal_msg_q_t *q_ptr, void *msg_ptr )
{
  void *list;
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  OSAL_MSG_NEXT( msg_ptr ) = NULL;
  // If first message in queue
  if ( *q_ptr == NULL )
  {
    *q_ptr = msg_ptr;
  }
  else
  {
    // Find end of queue
    for ( list = *q_ptr; OSAL_MSG_NEXT( list ) != NULL; list = OSAL_MSG_NEXT( list ) );

    // Add message to end of queue
    OSAL_MSG_NEXT( list ) = msg_ptr;
  }

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);
}

/*********************************************************************
 * @fn      osal_msg_dequeue
 *
 * @brief
 *
 *    This function dequeues an OSAL message from an OSAL queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 *
 * @return  void * - pointer to OSAL message or NULL of queue is empty.
 */
void *osal_msg_dequeue( osal_msg_q_t *q_ptr )
{
  void *msg_ptr = NULL;
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  if ( *q_ptr != NULL )
  {
    // Dequeue message
    msg_ptr = *q_ptr;
    *q_ptr = OSAL_MSG_NEXT( msg_ptr );
    OSAL_MSG_NEXT( msg_ptr ) = NULL;
    OSAL_MSG_ID( msg_ptr ) = TASK_NO_TASK;
  }

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);

  return msg_ptr;
}

/*********************************************************************
 * @fn      osal_msg_push
 *
 * @brief
 *
 *    This function pushes an OSAL message to the head of an OSAL
 *    queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message
 *
 * @return  none
 */
void osal_msg_push( osal_msg_q_t *q_ptr, void *msg_ptr )
{
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  // Push message to head of queue
  OSAL_MSG_NEXT( msg_ptr ) = *q_ptr;
  *q_ptr = msg_ptr;

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);
}

/*********************************************************************
 * @fn      osal_msg_extract
 *
 * @brief
 *
 *    This function extracts and removes an OSAL message from the
 *    middle of an OSAL queue.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message to be extracted
 * @param   void *prev_ptr  - OSAL message before msg_ptr in queue
 *
 * @return  none
 */
void osal_msg_extract( osal_msg_q_t *q_ptr, void *msg_ptr, void *prev_ptr )
{
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  if ( msg_ptr == *q_ptr )
  {
    // remove from first
    *q_ptr = OSAL_MSG_NEXT( msg_ptr );
  }
  else
  {
    // remove from middle
    OSAL_MSG_NEXT( prev_ptr ) = OSAL_MSG_NEXT( msg_ptr );
  }
  OSAL_MSG_NEXT( msg_ptr ) = NULL;
  OSAL_MSG_ID( msg_ptr ) = TASK_NO_TASK;

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);
}

/*********************************************************************
 * @fn      osal_msg_enqueue_max
 *
 * @brief
 *
 *    This function enqueues an OSAL message into an OSAL queue if
 *    the length of the queue is less than max.
 *
 * @param   osal_msg_q_t *q_ptr - OSAL queue
 * @param   void *msg_ptr  - OSAL message
 * @param   uint8 max - maximum length of queue
 *
 * @return  TRUE if message was enqueued, FALSE otherwise
 */
uint8 osal_msg_enqueue_max( osal_msg_q_t *q_ptr, void *msg_ptr, uint8 max )
{
  void *list;
  uint8 ret = FALSE;
  halIntState_t intState;

  // Hold off interrupts
  HAL_ENTER_CRITICAL_SECTION(intState);

  // If first message in queue
  if ( *q_ptr == NULL )
  {
    *q_ptr = msg_ptr;
    ret = TRUE;
  }
  else
  {
    // Find end of queue or max
    list = *q_ptr;
    max--;
    while ( (OSAL_MSG_NEXT( list ) != NULL) && (max > 0) )
    {
      list = OSAL_MSG_NEXT( list );
      max--;
    }

    // Add message to end of queue if max not reached
    if ( max != 0 )
    {
      OSAL_MSG_NEXT( list ) = msg_ptr;
      ret = TRUE;
    }
  }

  // Re-enable interrupts
  HAL_EXIT_CRITICAL_SECTION(intState);

  return ret;
}
