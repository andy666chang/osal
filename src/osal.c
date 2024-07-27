/*
 * @Author: andy.chang 
 * @Date: 2024-07-28 00:48:37 
 * @Last Modified by: andy.chang
 * @Last Modified time: 2024-07-28 00:54:49
 */

/******************************************************************************

 @file  OSAL.c

 @brief This API allows the software components in the Z-stack to be written
        independently of the specifics of the operating system, kernel or
        tasking environment (including control loops or connect-to-interrupt
        systems).

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2004-2021, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.5.2.0
 Release Date: 2021-03-10 17:38:58
 *****************************************************************************/

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

/* HAL */
#include "hal_drivers.h"

#ifdef IAR_ARMCM3_LM
  #include "FreeRTOSConfig.h"
  #include "osal_task.h"
#endif

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

/*********************************************************************
 * HELPER FUNCTIONS
 */
/* very ugly stub so Keil can compile */
#ifdef __KEIL__
char *  itoa ( int value, char * buffer, int radix )
{
  return(buffer);
}
#endif

/*********************************************************************
 * @fn      osal_strlen
 *
 * @brief
 *
 *   Calculates the length of a string.  The string must be null
 *   terminated.
 *
 * @param   char *pString - pointer to text string
 *
 * @return  int - number of characters
 */
int osal_strlen( char *pString )
{
  return (int)( strlen( pString ) );
}

/*********************************************************************
 * @fn      osal_memcpy
 *
 * @brief
 *
 *   Generic memory copy.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination uint8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - destination address
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to end of destination buffer
 */
void *osal_memcpy( void *dst, const void GENERIC *src, unsigned int len )
{
  uint8 *pDst;
  const uint8 GENERIC *pSrc;

  pSrc = src;
  pDst = dst;

  while ( len-- )
    *pDst++ = *pSrc++;

  return ( pDst );
}

/*********************************************************************
 * @fn      osal_revmemcpy
 *
 * @brief   Generic reverse memory copy.  Starts at the end of the
 *   source buffer, by taking the source address pointer and moving
 *   pointer ahead "len" bytes, then decrementing the pointer.
 *
 *   Note: This function differs from the standard memcpy(), since
 *         it returns the pointer to the next destination uint8. The
 *         standard memcpy() returns the original destination address.
 *
 * @param   dst - destination address
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to end of destination buffer
 */
void *osal_revmemcpy( void *dst, const void GENERIC *src, unsigned int len )
{
  uint8 *pDst;
  const uint8 GENERIC *pSrc;

  pSrc = src;
  pSrc += (len-1);
  pDst = dst;

  while ( len-- )
    *pDst++ = *pSrc--;

  return ( pDst );
}

/*********************************************************************
 * @fn      osal_memdup
 *
 * @brief   Allocates a buffer [with osal_mem_alloc()] and copies
 *          the src buffer into the newly allocated space.
 *
 * @param   src - source address
 * @param   len - number of bytes to copy
 *
 * @return  pointer to the new allocated buffer, or NULL if
 *          allocation problem.
 */
void *osal_memdup( const void GENERIC *src, unsigned int len )
{
  uint8 *pDst;

  pDst = osal_mem_alloc( len );
  if ( pDst )
  {
    VOID osal_memcpy( pDst, src, len );
  }

  return ( (void *)pDst );
}

/*********************************************************************
 * @fn      osal_memcmp
 *
 * @brief
 *
 *   Generic memory compare.
 *
 * @param   src1 - source 1 address
 * @param   src2 - source 2 address
 * @param   len - number of bytes to compare
 *
 * @return  TRUE - same, FALSE - different
 */
uint8 osal_memcmp( const void GENERIC *src1, const void GENERIC *src2, unsigned int len )
{
  const uint8 GENERIC *pSrc1;
  const uint8 GENERIC *pSrc2;

  pSrc1 = src1;
  pSrc2 = src2;

  while ( len-- )
  {
    if( *pSrc1++ != *pSrc2++ )
      return FALSE;
  }
  return TRUE;
}


/*********************************************************************
 * @fn      osal_memset
 *
 * @brief
 *
 *   Set memory buffer to value.
 *
 * @param   dest - pointer to buffer
 * @param   value - what to set each uint8 of the message
 * @param   size - how big
 *
 * @return  pointer to destination buffer
 */
void *osal_memset( void *dest, uint8 value, int len )
{
  return memset( dest, value, len );
}

/*********************************************************************
 * @fn      osal_build_uint16
 *
 * @brief
 *
 *   Build a uint16 out of 2 bytes (0 then 1).
 *
 * @param   swapped - 0 then 1
 *
 * @return  uint16
 */
uint16 osal_build_uint16( uint8 *swapped )
{
  return ( BUILD_UINT16( swapped[0], swapped[1] ) );
}

/*********************************************************************
 * @fn      osal_build_uint32
 *
 * @brief
 *
 *   Build a uint32 out of sequential bytes.
 *
 * @param   swapped - sequential bytes
 * @param   len - number of bytes in the uint8 array
 *
 * @return  uint32
 */
uint32 osal_build_uint32( uint8 *swapped, uint8 len )
{
  if ( len == 2 )
    return ( BUILD_UINT32( swapped[0], swapped[1], 0L, 0L ) );
  else if ( len == 3 )
    return ( BUILD_UINT32( swapped[0], swapped[1], swapped[2], 0L ) );
  else if ( len == 4 )
    return ( BUILD_UINT32( swapped[0], swapped[1], swapped[2], swapped[3] ) );
  else
    return ( (uint32)swapped[0] );
}

#if !defined ( ZBIT ) && !defined ( ZBIT2 ) && !defined (UBIT)
/*********************************************************************
 * @fn      _ltoa
 *
 * @brief
 *
 *   convert a long unsigned int to a string.
 *
 * @param  l - long to convert
 * @param  buf - buffer to convert to
 * @param  radix - 10 dec, 16 hex
 *
 * @return  pointer to buffer
 */
unsigned char * _ltoa(unsigned long l, unsigned char *buf, unsigned char radix)
{
#if defined (__TI_COMPILER_VERSION) || defined (__TI_COMPILER_VERSION__)
  return ( (unsigned char*)ltoa( l, (char *)buf ) );
#elif defined( __GNUC__ )
  return ( (char*)ltoa( l, buf, radix ) );
#else
  unsigned char tmp1[10] = "", tmp2[10] = "", tmp3[10] = "";
  unsigned short num1, num2, num3;
  unsigned char i;

  buf[0] = '\0';

  if ( radix == 10 )
  {
    num1 = l % 10000;
    num2 = (l / 10000) % 10000;
    num3 = (unsigned short)(l / 100000000);

    if (num3) _itoa(num3, tmp3, 10);
    if (num2) _itoa(num2, tmp2, 10);
    if (num1) _itoa(num1, tmp1, 10);

    if (num3)
    {
      strcpy((char*)buf, (char const*)tmp3);
      for (i = 0; i < 4 - strlen((char const*)tmp2); i++)
        strcat((char*)buf, "0");
    }
    strcat((char*)buf, (char const*)tmp2);
    if (num3 || num2)
    {
      for (i = 0; i < 4 - strlen((char const*)tmp1); i++)
        strcat((char*)buf, "0");
    }
    strcat((char*)buf, (char const*)tmp1);
    if (!num3 && !num2 && !num1)
      strcpy((char*)buf, "0");
  }
  else if ( radix == 16 )
  {
    num1 = l & 0x0000FFFF;
    num2 = l >> 16;

    if (num2) _itoa(num2, tmp2, 16);
    if (num1) _itoa(num1, tmp1, 16);

    if (num2)
    {
      strcpy((char*)buf,(char const*)tmp2);
      for (i = 0; i < 4 - strlen((char const*)tmp1); i++)
        strcat((char*)buf, "0");
    }
    strcat((char*)buf, (char const*)tmp1);
    if (!num2 && !num1)
      strcpy((char*)buf, "0");
  }
  else
    return NULL;

  return buf;
#endif
}
#endif // !defined(ZBIT) && !defined(ZBIT2)

/*********************************************************************
 * @fn        osal_rand
 *
 * @brief    Random number generator
 *
 * @param   none
 *
 * @return  uint16 - new random number
 */
uint16 osal_rand( void )
{
  return ( Onboard_rand() );
}

/*********************************************************************
 * API FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osal_isr_register
 *
 * @brief
 *
 *   This function is called to register a service routine with an
 *   interrupt. When the interrupt occurs, this service routine is called.
 *
 * @param   uint8 interrupt_id - Interrupt number
 * @param   void (*isr_ptr)( uint8* ) - function pointer to ISR
 *
 * @return  SUCCESS, INVALID_INTERRUPT_ID,
 */
uint8 osal_isr_register( uint8 interrupt_id, void (*isr_ptr)( uint8* ) )
{
  // Remove these statements when functionality is complete
  (void)interrupt_id;
  (void)isr_ptr;
  return ( SUCCESS );
}

/*********************************************************************
 * @fn      osal_int_enable
 *
 * @brief
 *
 *   This function is called to enable an interrupt. Once enabled,
 *   occurrence of the interrupt causes the service routine associated
 *   with that interrupt to be called.
 *
 *   If INTS_ALL is the interrupt_id, interrupts (in general) are enabled.
 *   If a single interrupt is passed in, then interrupts still have
 *   to be enabled with another call to INTS_ALL.
 *
 * @param   uint8 interrupt_id - Interrupt number
 *
 * @return  SUCCESS or INVALID_INTERRUPT_ID
 */
uint8 osal_int_enable( uint8 interrupt_id )
{

  if ( interrupt_id == INTS_ALL )
  {
    HAL_ENABLE_INTERRUPTS();
    return ( SUCCESS );
  }
  else
  {
    return ( INVALID_INTERRUPT_ID );
  }
}

/*********************************************************************
 * @fn      osal_int_disable
 *
 * @brief
 *
 *   This function is called to disable an interrupt. When a disabled
 *   interrupt occurs, the service routine associated with that
 *   interrupt is not called.
 *
 *   If INTS_ALL is the interrupt_id, interrupts (in general) are disabled.
 *   If a single interrupt is passed in, then just that interrupt is disabled.
 *
 * @param   uint8 interrupt_id - Interrupt number
 *
 * @return  SUCCESS or INVALID_INTERRUPT_ID
 */
uint8 osal_int_disable( uint8 interrupt_id )
{

  if ( interrupt_id == INTS_ALL )
  {
    HAL_DISABLE_INTERRUPTS();
    return ( SUCCESS );
  }
  else
  {
    return ( INVALID_INTERRUPT_ID );
  }
}

/*********************************************************************
 * @fn      osal_init_system
 *
 * @brief
 *
 *   This function initializes the "task" system by creating the
 *   tasks defined in the task table (OSAL_Tasks.h).
 *
 * @param   void
 *
 * @return  SUCCESS
 */
uint8 osal_init_system( void )
{
#if !defined USE_ICALL && !defined OSAL_PORT2TIRTOS
  // Initialize the Memory Allocation System
  osal_mem_init();
#endif /* !defined USE_ICALL && !defined OSAL_PORT2TIRTOS */

  // Initialize the message queue
  osal_qHead = NULL;

  // Initialize the timers
  osalTimerInit();

  // Initialize the Power Management System
  osal_pwrmgr_init();

#ifdef USE_ICALL
  /* Prepare memory space for service enrollment */
  osal_prepare_svc_enroll();
#endif /* USE_ICALL */

  // Initialize the system tasks.
  osalInitTasks();

#if !defined USE_ICALL && !defined OSAL_PORT2TIRTOS
  // Setup efficient search for the first free block of heap.
  osal_mem_kick();
#endif /* !defined USE_ICALL && !defined OSAL_PORT2TIRTOS */

#ifdef USE_ICALL
  // Initialize variables used to track timing and provide OSAL timer service
  osal_last_timestamp = (uint_least32_t) ICall_getTicks();
  osal_tickperiod = (uint_least32_t) ICall_getTickPeriod();
  osal_max_msecs = (uint_least32_t) ICall_getMaxMSecs();
  /* Reduce ceiling considering potential latency */
  osal_max_msecs -= 2;
#endif /* USE_ICALL */

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      osal_start_system
 *
 * @brief
 *
 *   This function is the main loop function of the task system (if
 *   ZBIT and UBIT are not defined). This Function doesn't return.
 *
 * @param   void
 *
 * @return  none
 */
void osal_start_system( void )
{
#ifdef USE_ICALL
  /* Kick off timer service in order to allocate resources upfront.
   * The first timeout is required to schedule next OSAL timer event
   * as well. */
  ICall_Errno errno = ICall_setTimer(1, osal_msec_timer_cback,
                                     (void *) osal_msec_timer_seq,
                                     &osal_timerid_msec_timer);
  if (errno != ICALL_ERRNO_SUCCESS)
  {
    ICall_abort();
  }
#endif /* USE_ICALL */

#if !defined ( ZBIT ) && !defined ( UBIT )
  for(;;)  // Forever Loop
#endif
  {
    osal_run_system();

#ifdef USE_ICALL
    ICall_wait(ICALL_TIMEOUT_FOREVER);
#endif /* USE_ICALL */
  }
}

/*********************************************************************
 * @fn      osal_run_system
 *
 * @brief
 *
 *   This function will make one pass through the OSAL taskEvents table
 *   and call the task_event_processor() function for the first task that
 *   is found with at least one event pending. If there are no pending
 *   events (all tasks), this function puts the processor into Sleep.
 *
 * @param   void
 *
 * @return  none
 */
void osal_run_system( void )
{
  uint8 idx = 0;

#ifdef USE_ICALL
  uint32 next_timeout_prior = osal_next_timeout();
#else /* USE_ICALL */
#ifndef HAL_BOARD_CC2538
  osalTimeUpdate();
#endif

  Hal_ProcessPoll();
#endif /* USE_ICALL */

#ifdef USE_ICALL
  {
    /* Update osal timers to the latest before running any OSAL processes
     * regardless of wakeup callback from ICall because OSAL timers are added
     * relative to the current time. */
    unsigned long newtimestamp = ICall_getTicks();
    uint32 milliseconds;

    if (osal_tickperiod == 1000)
    {
      milliseconds = newtimestamp - osal_last_timestamp;
      osal_last_timestamp = newtimestamp;
    }
    else
    {
      unsigned long long delta = (unsigned long long)
        ((newtimestamp - osal_last_timestamp) & 0xfffffffful);
      delta *= osal_tickperiod;
      delta /= 1000;
      milliseconds = (uint32) delta;
      osal_last_timestamp += (uint32) (delta * 1000 / osal_tickperiod);
    }
    osalAdjustTimer(milliseconds);
    /* Set a value that will never match osal_next_timeout()
     * return value so that the next time can be scheduled.
     */
    next_timeout_prior = 0xfffffffful;
  }
  if (osal_eventloop_hook)
  {
    osal_eventloop_hook();
  }

  for (;;)
  {
    void *msg;
    ICall_EntityID src, dst;
    osal_msg_hdr_t *hdr;
    uint8 dest_id;

    if (ICall_fetchMsg(&src, &dst, &msg) != ICALL_ERRNO_SUCCESS)
    {
      break;
    }
    hdr = (osal_msg_hdr_t *) msg - 1;
    dest_id = osal_dispatch2id(dst);
    if (dest_id == TASK_NO_TASK)
    {
      /* Something wrong */
      ICall_abort();
    }
    else
    {
      /* Message towards one of the tasks */
      /* Create a proxy task ID if necessary and
       * queue the message to the OSAL internal queue.
       */
      uint8 proxyid = osal_alien2proxy(hdr->srcentity);

      if (hdr->format == ICALL_MSG_FORMAT_1ST_CHAR_TASK_ID)
      {
        uint8 *bytes = msg;
        *bytes = proxyid;
      }
      else if (hdr->format == ICALL_MSG_FORMAT_3RD_CHAR_TASK_ID)
      {
        uint8 *bytes = msg;
        bytes[2] = proxyid;
      }
      /* now queue the message to the OSAL queue */
      osal_msg_send(dest_id, msg);
    }
  }
#endif /* USE_ICALL */

  do {
    if (tasksEvents[idx])  // Task is highest priority that is ready.
    {
      break;
    }
  } while (++idx < tasksCnt);

  if (idx < tasksCnt)
  {
    uint16 events;
    halIntState_t intState;

    HAL_ENTER_CRITICAL_SECTION(intState);
    events = tasksEvents[idx];
    tasksEvents[idx] = 0;  // Clear the Events for this task.
    HAL_EXIT_CRITICAL_SECTION(intState);

    activeTaskID = idx;
    events = (tasksArr[idx])( idx, events );
    activeTaskID = TASK_NO_TASK;

    HAL_ENTER_CRITICAL_SECTION(intState);
    tasksEvents[idx] |= events;  // Add back unprocessed events to the current task.
    HAL_EXIT_CRITICAL_SECTION(intState);
  }
#if defined( POWER_SAVING ) && !defined(USE_ICALL)
  else  // Complete pass through all task events with no activity?
  {
    osal_pwrmgr_powerconserve();  // Put the processor/system into sleep
  }
#endif

  /* Yield in case cooperative scheduling is being used. */
#if defined (configUSE_PREEMPTION) && (configUSE_PREEMPTION == 0)
  {
    osal_task_yield();
  }
#endif

#if defined USE_ICALL
  /* Note that scheduling wakeup at this point instead of
   * scheduling it upon ever OSAL start timer request,
   * would only work if OSAL start timer call is made
   * from OSAL tasks, but not from either ISR or
   * non-OSAL application thread.
   * In case, OSAL start timer is called from non-OSAL
   * task, the scheduling should be part of OSAL_Timers
   * module.
   * Such a change to OSAL_Timers module was not made
   * in order not to diverge the OSAL implementations
   * too drastically between pure OSAL solution vs.
   * OSAL upon service dispatcher (RTOS).
   * TODO: reconsider the above statement.
   */
  {
    halIntState_t intState;

    uint32 next_timeout_post = osal_next_timeout();
    if (next_timeout_post != next_timeout_prior)
    {
      /* Next wakeup time has to be scheduled */
      if (next_timeout_post == 0)
      {
        /* No timer. Set time to the max */
        next_timeout_post = OSAL_TIMERS_MAX_TIMEOUT;
      }
      if (next_timeout_post > osal_max_msecs)
      {
        next_timeout_post = osal_max_msecs;
      }
      /* Restart timer */
      HAL_ENTER_CRITICAL_SECTION(intState);
      ICall_stopTimer(osal_timerid_msec_timer);
      ICall_setTimerMSecs(next_timeout_post, osal_msec_timer_cback,
                          (void *) (++osal_msec_timer_seq),
                          &osal_timerid_msec_timer);
      HAL_EXIT_CRITICAL_SECTION(intState);
    }
  }
#endif /* USE_ICALL */
}

/*********************************************************************
 * @fn      osal_buffer_uint32
 *
 * @brief
 *
 *   Buffer an uint32 value - LSB first.
 *
 * @param   buf - buffer
 * @param   val - uint32 value
 *
 * @return  pointer to end of destination buffer
 */
uint8* osal_buffer_uint32( uint8 *buf, uint32 val )
{
  *buf++ = BREAK_UINT32( val, 0 );
  *buf++ = BREAK_UINT32( val, 1 );
  *buf++ = BREAK_UINT32( val, 2 );
  *buf++ = BREAK_UINT32( val, 3 );

  return buf;
}

/*********************************************************************
 * @fn      osal_buffer_uint24
 *
 * @brief
 *
 *   Buffer an uint24 value - LSB first. Note that type uint24 is
 *   typedef to uint32 in comdef.h
 *
 * @param   buf - buffer
 * @param   val - uint24 value
 *
 * @return  pointer to end of destination buffer
 */
uint8* osal_buffer_uint24( uint8 *buf, uint24 val )
{
  *buf++ = BREAK_UINT32( val, 0 );
  *buf++ = BREAK_UINT32( val, 1 );
  *buf++ = BREAK_UINT32( val, 2 );

  return buf;
}

/*********************************************************************
 * @fn      osal_isbufset
 *
 * @brief
 *
 *   Is all of the array elements set to a value?
 *
 * @param   buf - buffer to check
 * @param   val - value to check each array element for
 * @param   len - length to check
 *
 * @return  TRUE if all "val"
 *          FALSE otherwise
 */
uint8 osal_isbufset( uint8 *buf, uint8 val, uint8 len )
{
  uint8 x;

  if ( buf == NULL )
  {
    return ( FALSE );
  }

  for ( x = 0; x < len; x++ )
  {
    // Check for non-initialized value
    if ( buf[x] != val )
    {
      return ( FALSE );
    }
  }
  return ( TRUE );
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

/*********************************************************************
 */
