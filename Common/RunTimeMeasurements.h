/**********************************************************************************************
 *
 * Module: RunTimeMeasurements
 *
 * File Name: RunTimeMeasurements.h
 *
 * Description: Manual runtime measurement facility for the Seat Heater Control System, built
 *              on top of the GPTM (WTimer0) 0.1ms-resolution free running timestamp counter,
 *              as required by the project specification (item 5):
 *                  a. Execution time for each task.
 *                  b. CPU load.
 *                  c. Resource lock time per task for each resource (using FreeRTOS trace
 *                     hook macros).
 *
 *              This module is deliberately independent from the FreeRTOS built-in run time
 *              statistics feature (vTaskGetRunTimeStats / configGENERATE_RUN_TIME_STATS),
 *              which per the requirements is only allowed to be used to validate these manual
 *              readings, and must not be part of the final delivered implementation.
 *
 *              Usage:
 *                  1) Call RTM_Init() once, after GPTM_WTimer0Init().
 *                  2) Tag every task right after creation with vTaskSetApplicationTaskTag()
 *                     using one of the TaskTag_t values below.
 *                  3) Call RTM_RegisterResource() once per mutex/semaphore that needs its
 *                     lock time tracked (ADC, UART, Diagnostics log, per-seat state...).
 *                  4) FreeRTOSConfig.h wires traceTASK_SWITCHED_IN/OUT and
 *                     traceQUEUE_SEND/RECEIVE to the hooks below - no further action needed.
 *                  5) Read back the accumulated statistics from the Run Time Stats task using
 *                     the Get* APIs below.
 *
 * Author: Seat Heater Control System
 *
 ***********************************************************************************************/

#ifndef RUNTIMEMEASUREMENTS_H_
#define RUNTIMEMEASUREMENTS_H_

#include "std_types.h"

/* NOTE: This header is included directly from FreeRTOSConfig.h (to wire the trace hook
 * macros below), which is in turn included from FreeRTOS.h itself. To avoid a circular
 * include (FreeRTOS.h -> FreeRTOSConfig.h -> RunTimeMeasurements.h -> FreeRTOS.h), this
 * header intentionally does NOT include FreeRTOS.h/semphr.h and uses a plain (void *) instead
 * of SemaphoreHandle_t for resource handles. */

/*******************************************************************************
 *                             Preprocessor Macros                             *
 *******************************************************************************/

#define RTM_MAX_TASKS           (10U)
#define RTM_MAX_RESOURCES       (5U)

/* Task tags. Tag 0 is reserved by FreeRTOS for un-tagged tasks (Idle task, Timer service
 * task, ...) and is intentionally not tracked individually here. Every application task MUST
 * be tagged right after xTaskCreate() with one of the values below, e.g.:
 *      vTaskSetApplicationTaskTag(xHandle, (TaskHookFunction_t)TASK_TAG_DRIVER_BUTTON);
 */
typedef enum
{
    TASK_TAG_UNTAGGED = 0,
    TASK_TAG_DRIVER_BUTTON,
    TASK_TAG_PASSENGER_BUTTON,
    TASK_TAG_DRIVER_TEMPSENSE,
    TASK_TAG_PASSENGER_TEMPSENSE,
    TASK_TAG_DRIVER_HEATER,
    TASK_TAG_PASSENGER_HEATER,
    TASK_TAG_DIAGNOSTICS,
    TASK_TAG_DISPLAY,
    TASK_TAG_RUNTIME_STATS
} TaskTag_t;

/* Shared resources (mutexes) whose per-task lock time is tracked */
typedef enum
{
    RES_ADC = 0,
    RES_UART,
    RES_DIAG,
    RES_SEAT_DRIVER,
    RES_SEAT_PASSENGER,
    RES_COUNT
} ResourceId_t;

/*******************************************************************************
 *                            Functions Prototypes                             *
 *******************************************************************************/

/* Must be called once during hardware/software init, after GPTM_WTimer0Init() */
void RTM_Init(void);

/* Registers a mutex/semaphore handle (pass the SemaphoreHandle_t / QueueHandle_t cast to
 * void*) so its lock time can be attributed to whichever task takes/gives it. name is a
 * short human readable label used by the Run Time Stats task. */
void RTM_RegisterResource(ResourceId_t resId, void *handle, const sint8 *name);

/* ---- Hooks wired from FreeRTOSConfig.h trace macros - not meant to be called directly ---- */
void RTM_TaskSwitchedIn(uint32 taskTag);
void RTM_TaskSwitchedOut(uint32 taskTag);
void RTM_ResourceTakeHook(void *pxQueueHandle);
void RTM_ResourceGiveHook(void *pxQueueHandle);

/* ---- Read-back API used by the Run Time Stats task ---- */

/* Cumulative execution time of a task since boot, in 0.1ms GPTM ticks */
uint32 RTM_GetTaskExecTimeTicks(TaskTag_t tag);

/* Cumulative time a task has spent holding a given resource, in 0.1ms GPTM ticks */
uint32 RTM_GetResourceLockTimeTicks(TaskTag_t tag, ResourceId_t res);

/* Total elapsed time since RTM_Init() was called, in 0.1ms GPTM ticks (used as the CPU load
 * measurement window denominator) */
uint32 RTM_GetElapsedTicks(void);

/* Human readable names, for UART reporting */
const sint8* RTM_GetTaskName(TaskTag_t tag);
const sint8* RTM_GetResourceName(ResourceId_t res);

#endif /* RUNTIMEMEASUREMENTS_H_ */
