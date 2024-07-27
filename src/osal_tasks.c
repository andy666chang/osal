/*
 * @Author: andy.chang 
 * @Date: 2024-07-28 00:49:41 
 * @Last Modified by: andy.chang
 * @Last Modified time: 2024-07-28 00:55:07
 */

/*********************************************************************
 * INCLUDES
 */

#include <string.h>

#include "comdef.h"
#include "hal_board.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_Memory.h"
#include "OSAL_PwrMgr.h"
#include "OSAL_Clock.h"

#include "OnBoard.h"

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

// Index of active task
static uint8 activeTaskID = TASK_NO_TASK;

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

/*********************************************************************
 * HELPER FUNCTIONS
 */

/*********************************************************************
 * API FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osal_set_event
 *
 * @brief
 *
 *    This function is called to set the event flags for a task. The
 *    event passed in is OR'd into the task's event variable.
 *
 * @param   uint8 task_id - receiving tasks ID
 * @param   uint8 event_flag - what event to set
 *
 * @return  SUCCESS, MSG_BUFFER_NOT_AVAIL, FAILURE, INVALID_TASK
 */
#ifdef OSAL_PORT2TIRTOS
uint8 osal_set_event_raw( uint8 task_id, uint16 event_flag )
#else /* OSAL_PORT2TIRTOS */
uint8 osal_set_event( uint8 task_id, uint16 event_flag )
#endif /* OSAL_PORT2TIRTOS */
{
#ifdef USE_ICALL
  if (task_id & OSAL_PROXY_ID_FLAG)
  {
    /* Destination is a proxy task */
    osal_msg_hdr_t *hdr;
    ICall_EntityID src, dst;
    uint8 taskid;

    struct _osal_event_msg_t
    {
      uint16 signature;
      uint16 event_flag;
    } *msg_ptr = (struct _osal_event_msg_t *)
      osal_msg_allocate(sizeof(*msg_ptr));

    if (!msg_ptr)
    {
      return MSG_BUFFER_NOT_AVAIL;
    }
    msg_ptr->signature = 0xffffu;
    msg_ptr->event_flag = event_flag;
    hdr = (osal_msg_hdr_t *)msg_ptr - 1;

    taskid = osal_self();
    if (taskid == TASK_NO_TASK)
    {
      /* Call must have been made from either an ISR or a user-thread */
      src = osal_notask_entity;
    }
    else
    {
      src = (ICall_EntityID) osal_dispatch_entities[taskid + tasksCnt];
    }

    if (src == OSAL_INVALID_DISPATCH_ID)
    {
      /* The source entity is not registered */
      osal_msg_deallocate((uint8 *) msg_ptr);
      ICall_abort();
      return FAILURE;
    }
    dst = osal_proxy2alien(task_id);
    hdr->dest_id = TASK_NO_TASK;
    if (ICall_send(src, dst,
                   ICALL_MSG_FORMAT_KEEP, msg_ptr) ==
        ICALL_ERRNO_SUCCESS)
    {
      return SUCCESS;
    }
    osal_msg_deallocate((uint8 *) msg_ptr);
    return FAILURE;
  }
#endif /* USE_ICALL */

  if ( task_id < tasksCnt )
  {
    halIntState_t   intState;
    HAL_ENTER_CRITICAL_SECTION(intState);    // Hold off interrupts
    tasksEvents[task_id] |= event_flag;  // Stuff the event bit(s)
    HAL_EXIT_CRITICAL_SECTION(intState);     // Release interrupts
#ifdef USE_ICALL
    ICall_signal(osal_semaphore);
#endif /* USE_ICALL */
    return ( SUCCESS );
  }
   else
  {
    return ( INVALID_TASK );
  }
}

/*********************************************************************
 * @fn      osal_clear_event
 *
 * @brief
 *
 *    This function is called to clear the event flags for a task. The
 *    event passed in is masked out of the task's event variable.
 *
 * @param   uint8 task_id - receiving tasks ID
 * @param   uint8 event_flag - what event to clear
 *
 * @return  SUCCESS, INVALID_TASK
 */
uint8 osal_clear_event( uint8 task_id, uint16 event_flag )
{
  if ( task_id < tasksCnt )
  {
    halIntState_t   intState;
    HAL_ENTER_CRITICAL_SECTION(intState);    // Hold off interrupts
    tasksEvents[task_id] &= ~(event_flag);   // Clear the event bit(s)
    HAL_EXIT_CRITICAL_SECTION(intState);     // Release interrupts
    return ( SUCCESS );
  }
   else
  {
    return ( INVALID_TASK );
  }
}
