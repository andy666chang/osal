/*
 * @Author: andy.chang 
 * @Date: 2024-07-28 00:49:41 
 * @Last Modified by: andy.chang
 * @Last Modified time: 2024-07-28 21:35:01
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

/*
 * OSAL task control block
 */
typedef struct osal_tcb {
  uint8 taskID;
  uint16 events;
  // uint8 priority;
  pTaskEventHandlerFn processor;
  struct osal_tcb *next;
} osal_tcb_t;

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
static osal_tcb_t *task_head = NULL;

/*********************************************************************
 * LOCAL FUNCTION PROTOTYPES
 */

static osal_tcb_t *osal_find_task(uint8 task_id);

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
  osal_tcb_t *task = osal_find_task(task_id);

  if (task)
  {
    halIntState_t intState;
    HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
    task->events |= event_flag;
    HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.
  }
  else
  {
    return (INVALID_TASK);
  }

  return (SUCCESS);
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
  osal_tcb_t *task = osal_find_task(task_id);

  if (task)
  {
    halIntState_t intState;
    HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.
    task->events &= ~event_flag;
    HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.
  }
  else
  {
    return (INVALID_TASK);
  }

  return (SUCCESS);
}

/**
 * @brief 
 * 
 */
void osalInitTasks( void ) {
  task_head = NULL;
}

/**
 * @brief 
 * 
 */
void osal_tasks_process( void ) {
  // get task_list header
  osal_tcb_t *task = task_head;

  // polling task with event
  while (task)
  {
    // check events
    if (task->events)
    {
      halIntState_t intState;
      HAL_ENTER_CRITICAL_SECTION( intState );  // Hold off interrupts.

      // assign activeTaskID
      activeTaskID = task->taskID;

      // get events
      uint16 events = task->events;
      task->events = 0;

      // process task
      if (task->processor)
      {
        task->processor(task->taskID, events);
      }

      // disable activeTaskID
      activeTaskID = TASK_NO_TASK;

      HAL_EXIT_CRITICAL_SECTION( intState );   // Re-enable interrupts.
    }
    
    // check next task
    task = task->next;
  }

}

/*********************************************************************
 * @fn      osal_self
 *
 * @brief
 *
 *   This function returns the task ID of the current (active) task.
 *
 * @param   void
 *
 * @return   active task ID or TASK_NO_TASK if no task is active
 */
uint8 osal_self( void )
{
  return ( activeTaskID );
}

/**
 * @brief 
 * 
 * @param task_id 
 * @return osal_tcb_t* 
 */
static osal_tcb_t *osal_find_task(uint8 task_id) {

  // get task_list header
  osal_tcb_t *task = task_head;

  // polling task with event
  while (task)
  {
    // check events
    if (task->taskID == task_id)
    {
      return (task);
    }
    
    // check next task
    task = task->next;
  }

  return (NULL);
}

bool osal_task_valid(uint8 task_id) {
  return (bool)osal_find_task(task_id);
}

bool osal_task_create(uint8 task_id, pTaskEventHandlerFn precessor) {
  osal_tcb_t *new_task = NULL;

  // Check param
  if (osal_task_valid(task_id))
  {
    return (INVALID_TASK);
  }

  if (precessor==NULL)
  {
    return (INVALIDPARAMETER);
  }

  // malloc task memory
  new_task = osal_mem_alloc(sizeof(osal_tcb_t));
  if (new_task == NULL)
  {
    return (INVALID_MEM_SIZE);
  }

  // assign task_id & precessor
  new_task->taskID = task_id;
  new_task->processor = precessor;
  new_task->events = 0;
  new_task->next = NULL;

  // TODO: append to link list(task_head)
  if (task_head == NULL)
  {
    // append first task
    task_head = new_task;
  } else {
    osal_tcb_t *tasks = task_head;
    // append to task list
    do {
      if (tasks->next == NULL)
      {
        tasks->next = new_task;
        break;
      }
    } while (1);
  }

  return (SUCCESS);
}
