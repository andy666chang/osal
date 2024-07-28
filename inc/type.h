/*
 * @Author: andy.chang 
 * @Date: 2024-07-28 13:45:25 
 * @Last Modified by: andy.chang
 * @Last Modified time: 2024-07-28 14:18:51
 */

#ifndef TYPE_H
#define TYPR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "osal_platform.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/*********************************************************************
 * Lint Keywords
 */

#define VOID (void)

#define NULL_OK
#define INP
#define OUTP

#ifndef UNUSED
    #define UNUSED void
#endif

#define ONLY
#define READONLY
#define SHARED
#define KEEP
#define RELAX

/*********************************************************************
 * CONSTANTS
 */

#ifndef false
  #define false 0
#endif

#ifndef true
  #define true 1
#endif

#ifndef FALSE
  #define FALSE false
#endif

#ifndef TRUE
  #define TRUE true
#endif

#ifndef CONST
  #define CONST const
#endif

#ifndef GENERIC
  #define GENERIC
#endif

/*** Generic Status Return Values ***/
#define SUCCESS                   0x00
#define FAILURE                   0x01
#define INVALIDPARAMETER          0x02
#define INVALID_TASK              0x03
#define MSG_BUFFER_NOT_AVAIL      0x04
#define INVALID_MSG_POINTER       0x05
#define INVALID_EVENT_ID          0x06
#define INVALID_INTERRUPT_ID      0x07
#define NO_TIMER_AVAIL            0x08
#define NV_ITEM_UNINIT            0x09
#define NV_OPER_FAILED            0x0A
#define INVALID_MEM_SIZE          0x0B
#define NV_BAD_ITEM_LEN           0x0C

/*********************************************************************
 * TYPEDEFS
 */

typedef uint8_t uint8;
typedef int8_t int8;

typedef uint16_t uint16;
typedef int16_t int16;

typedef uint32_t uint32;
typedef int32_t int32;

// Generic Status return
typedef uint8 Status_t;

// Data types
typedef int32   int24;
typedef uint32  uint24;

/*********************************************************************
 * Global System Events
 */

#define SYS_EVENT_MSG               0x8000  // A message is waiting event

/*********************************************************************
 * Global Generic System Messages
 */

#define KEY_CHANGE                0xC0    // Key Events

// OSAL System Message IDs/Events Reserved for applications (user applications)
// 0xE0 – 0xFC

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
*********************************************************************/

#if 0
// Align
// typedef uint32_t        halDataAlign_t;
#endif

#ifdef __cplusplus
}
#endif

#endif // TYPE_H
