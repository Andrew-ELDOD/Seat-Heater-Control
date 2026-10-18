/**********************************************************************************************
 *
 * Module: RunTimeMeasurements
 *
 * File Name: RunTimeMeasurements.c
 *
 * Description: See RunTimeMeasurements.h
 *
 ***********************************************************************************************/
#include "RunTimeMeasurements.h"
#include "GPTM.h"
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************
 *                              Private Global Variables                       *
 *******************************************************************************/

/* Task execution time bookkeeping (indexed by TaskTag_t) */
static uint32 g_tasksInTime[RTM_MAX_TASKS];
static uint32 g_tasksOutTime[RTM_MAX_TASKS];
static uint32 g_tasksExecTime[RTM_MAX_TASKS];   /* cumulative, since RTM_Init(), in 0.1ms ticks */

/* Resource registry + per task/resource lock time bookkeeping */
static void  *g_resourceHandles[RTM_MAX_RESOURCES];
static const sint8 *g_resourceNames[RTM_MAX_RESOURCES];
static uint32 g_resourceTakeTime[RTM_MAX_TASKS][RTM_MAX_RESOURCES];
static uint32 g_resourceLockTime[RTM_MAX_TASKS][RTM_MAX_RESOURCES];

static uint32 g_startTimeTicks;

static const sint8 *g_taskNames[RTM_MAX_TASKS] =
{
    (const sint8 *)"Untagged",
    (const sint8 *)"DriverButtonTask",
    (const sint8 *)"PassengerButtonTask",
    (const sint8 *)"DriverTempSenseTask",
    (const sint8 *)"PassengerTempSenseTask",
    (const sint8 *)"DriverHeaterTask",
    (const sint8 *)"PassengerHeaterTask",
    (const sint8 *)"DiagnosticsTask",
    (const sint8 *)"DisplayTask",
    (const sint8 *)"RunTimeStatsTask"
};

/*******************************************************************************
 *                         Public Functions Definitions                        *
 *******************************************************************************/

void RTM_Init(void)
{
    uint32 i, j;

    for(i = 0; i < RTM_MAX_TASKS; i++)
    {
        g_tasksInTime[i]   = 0;
        g_tasksOutTime[i]  = 0;
        g_tasksExecTime[i] = 0;

        for(j = 0; j < RTM_MAX_RESOURCES; j++)
        {
            g_resourceTakeTime[i][j] = 0;
            g_resourceLockTime[i][j] = 0;
        }
    }

    for(j = 0; j < RTM_MAX_RESOURCES; j++)
    {
        g_resourceHandles[j] = NULL_PTR;
        g_resourceNames[j]   = (const sint8 *)"Unused";
    }

    g_startTimeTicks = GPTM_WTimer0Read();
}

void RTM_RegisterResource(ResourceId_t resId, void *handle, const sint8 *name)
{
    if(resId < RES_COUNT)
    {
        g_resourceHandles[resId] = handle;
        g_resourceNames[resId]   = name;
    }
}

void RTM_TaskSwitchedIn(uint32 taskTag)
{
    if(taskTag < RTM_MAX_TASKS)
    {
        g_tasksInTime[taskTag] = GPTM_WTimer0Read();
    }
}

void RTM_TaskSwitchedOut(uint32 taskTag)
{
    if(taskTag < RTM_MAX_TASKS)
    {
        g_tasksOutTime[taskTag] = GPTM_WTimer0Read();
        /* Accumulate execution time (handles the extremely rare/late wrap-around of the
         * one-shot down counter gracefully enough for a lab-scale project run) */
        g_tasksExecTime[taskTag] += (g_tasksOutTime[taskTag] - g_tasksInTime[taskTag]);
    }
}

static sint32 RTM_FindResourceIndex(void *pxQueueHandle)
{
    uint32 i;
    for(i = 0; i < RES_COUNT; i++)
    {
        if(g_resourceHandles[i] == pxQueueHandle)
        {
            return (sint32)i;
        }
    }
    return -1;
}

void RTM_ResourceTakeHook(void *pxQueueHandle)
{
    sint32 resIdx = RTM_FindResourceIndex(pxQueueHandle);
    if(resIdx >= 0)
    {
        uint32 taskTag = (uint32)(xTaskGetApplicationTaskTag(NULL));
        if(taskTag < RTM_MAX_TASKS)
        {
            g_resourceTakeTime[taskTag][resIdx] = GPTM_WTimer0Read();
        }
    }
}

void RTM_ResourceGiveHook(void *pxQueueHandle)
{
    sint32 resIdx = RTM_FindResourceIndex(pxQueueHandle);
    if(resIdx >= 0)
    {
        uint32 taskTag = (uint32)(xTaskGetApplicationTaskTag(NULL));
        if(taskTag < RTM_MAX_TASKS)
        {
            uint32 now = GPTM_WTimer0Read();
            g_resourceLockTime[taskTag][resIdx] += (now - g_resourceTakeTime[taskTag][resIdx]);
        }
    }
}

uint32 RTM_GetTaskExecTimeTicks(TaskTag_t tag)
{
    if(tag < RTM_MAX_TASKS)
    {
        return g_tasksExecTime[tag];
    }
    return 0;
}

uint32 RTM_GetResourceLockTimeTicks(TaskTag_t tag, ResourceId_t res)
{
    if((tag < RTM_MAX_TASKS) && (res < RES_COUNT))
    {
        return g_resourceLockTime[tag][res];
    }
    return 0;
}

uint32 RTM_GetElapsedTicks(void)
{
    return (GPTM_WTimer0Read() - g_startTimeTicks);
}

const sint8* RTM_GetTaskName(TaskTag_t tag)
{
    if(tag < RTM_MAX_TASKS)
    {
        return g_taskNames[tag];
    }
    return (const sint8 *)"Unknown";
}

const sint8* RTM_GetResourceName(ResourceId_t res)
{
    if(res < RES_COUNT)
    {
        return g_resourceNames[res];
    }
    return (const sint8 *)"Unknown";
}
