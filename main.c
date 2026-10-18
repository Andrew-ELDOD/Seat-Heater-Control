/**********************************************************************************************
 *
 * Project: Seat Heater Control System
 *
 * File Name: main.c
 *
 * Description:
 *   Implements the seat heater control system for the driver and passenger seats, as per the
 *   project specification, on top of FreeRTOS + the TM4C123GH6PM MCAL drivers.
 *
 *   ---------------------------------------------------------------------------------------
 *   Hardware / pin map
 *   ---------------------------------------------------------------------------------------
 *     PF4 (SW1, built-in)   -> Driver seat console button
 *     PF0 (SW2, built-in)   -> Passenger seat console button
 *     PB3 (external, added) -> Driver steering wheel button (extra control for driver seat)
 *                               Wire a push button between PB3 and GND (internal pull-up used,
 *                               active LOW, same wiring style as the built-in buttons).
 *
 *     PE3 / AIN0 -> Driver seat potentiometer wiper      (0V-3.3V simulates 0C-45C)
 *     PE2 / AIN1 -> Passenger seat potentiometer wiper   (0V-3.3V simulates 0C-45C)
 *
 *     Driver seat indication    -> on-board RGB LED (PF1=Red, PF2=Blue, PF3=Green)
 *                                    Green  = low intensity
 *                                    Blue   = medium intensity
 *                                    Cyan (Green+Blue) = high intensity
 *                                    Red    = sensor failure (heater disabled)
 *     Passenger seat indication -> external RGB LED (PB0=Green, PB1=Blue, PB2=Red), same
 *                                    color code as above. The board only has a single
 *                                    built-in RGB LED, so a second one is required externally
 *                                    to independently show the passenger seat's state.
 *
 *     PA0/PA1 (UART0, built-in) -> "screen" - status is sent as text once per second and can
 *                                   be viewed on a serial terminal at 9600-8-N-1.
 *   ---------------------------------------------------------------------------------------
 *
 *   ---------------------------------------------------------------------------------------
 *   Task list (6 distinct task functions; Button/TempSense/Heater are each reused for both
 *   the driver and the passenger seat instance, i.e. instantiated twice)
 *   ---------------------------------------------------------------------------------------
 *     1. vButtonTask         (x2: driver, passenger) - event driven (edge-triggered GPIO
 *        interrupt -> event group bit), cycles OFF->LOW->MED->HIGH->OFF on each press.
 *     2. vTempSenseTask      (x2: driver, passenger) - periodic (100ms), reads the shared
 *        ADC module (mutex protected) and validates the 5C-40C sensor range.
 *     3. vHeaterControlTask  (x2: driver, passenger) - periodic (100ms), runs the heater
 *        intensity state machine and drives the seat's status LEDs.
 *     4. vDiagnosticsTask    (x1) - event driven (blocks on a queue fed by tasks 1 & 2),
 *        logs failures and heating level changes with a GPTM timestamp into RAM.
 *     5. vDisplayTask        (x1) - periodic (1000ms), sends both seats' current state to
 *        the shared UART "screen" (mutex protected).
 *     6. vRunTimeStatsTask   (x1) - periodic (5000ms), reports manual runtime measurements
 *        (execution time, CPU load, per-resource lock time) collected via FreeRTOS trace
 *        hooks (see Common/RunTimeMeasurements.c) over the shared UART.
 *   ---------------------------------------------------------------------------------------
 *
 ***********************************************************************************************/

/* Kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* MCAL includes */
#include "gpio.h"
#include "uart0.h"
#include "GPTM.h"
#include "adc.h"
#include "tm4c123gh6pm_registers.h"

/* Common includes */
#include "RunTimeMeasurements.h"

/*******************************************************************************
 *                             Application Constants                           *
 *******************************************************************************/

#define TEMPSENSE_PERIOD_MS         (100U)
#define HEATER_CONTROL_PERIOD_MS    (100U)
#define DISPLAY_PERIOD_MS           (1000U)
#define RUNTIME_STATS_PERIOD_MS     (5000U)
#define BUTTON_DEBOUNCE_MS          (200U)

/* Sensor validity range, per the requirements: 5C - 40C (in tenths of a degree) */
#define SENSOR_VALID_MIN_X10        (50)
#define SENSOR_VALID_MAX_X10        (400)

/* Desired temperature per heating level (in tenths of a degree Celsius) */
#define DESIRED_TEMP_LOW_X10        (250)  /* 25 C */
#define DESIRED_TEMP_MEDIUM_X10     (300)  /* 30 C */
#define DESIRED_TEMP_HIGH_X10       (350)  /* 35 C */

/* Heater intensity thresholds, as delta between desired and current temperature (x10 C) */
#define DIFF_HIGH_THRESHOLD_X10     (100)  /* >= 10 C difference -> HIGH   */
#define DIFF_MEDIUM_THRESHOLD_X10   (50)   /* 5 - 10 C difference -> MEDIUM */
#define DIFF_LOW_THRESHOLD_X10      (20)   /* 2 - 5 C difference -> LOW (also hysteresis re-arm) */

#define DIAG_EVENT_QUEUE_LENGTH     (20U)

/* Event group bits set by the button ISRs (GPIOPortF_Handler / GPIOPortB_Handler) */
#define BTN_BIT_DRIVER              (1UL << 0UL)
#define BTN_BIT_PASSENGER           (1UL << 1UL)

/*******************************************************************************
 *                             Application Types                               *
 *******************************************************************************/

typedef enum
{
    LEVEL_OFF = 0,
    LEVEL_LOW,
    LEVEL_MEDIUM,
    LEVEL_HIGH,
    NUM_LEVELS
} HeatingLevel_t;

typedef enum
{
    INTENSITY_OFF = 0,
    INTENSITY_LOW,
    INTENSITY_MEDIUM,
    INTENSITY_HIGH
} HeaterIntensity_t;

typedef enum
{
    SEAT_DRIVER = 0,
    SEAT_PASSENGER,
    NUM_SEATS
} SeatId_t;

/* Per-seat state. Every field below (other than the constant setup fields at the top) is
 * a shared resource and MUST only be accessed while holding 'mutex'. */
typedef struct
{
    /* Constant, set once at init - no protection required */
    SeatId_t            seatId;
    uint8               adcChannel;
    const sint8         *name;

    /* Shared / mutex protected fields */
    HeatingLevel_t     heatingLevel;
    sint16             desiredTempX10;
    sint16             currentTempX10;
    HeaterIntensity_t  heaterIntensity;
    boolean            heaterEnabled;   /* hysteresis latch: TRUE while heater is allowed to run */
    boolean            sensorFail;

    SemaphoreHandle_t  mutex;
} SeatState_t;

typedef enum
{
    DIAG_EVT_LEVEL_CHANGED = 0,
    DIAG_EVT_FAILURE_CHANGED
} DiagEventType_t;

/* Message sent through xDiagEventQueue - guarantees no diagnostic event is ever lost, even
 * if several occur before the Diagnostics task gets a chance to run. */
typedef struct
{
    SeatId_t         seatId;
    DiagEventType_t  eventType;
    HeatingLevel_t   level;      /* valid when eventType == DIAG_EVT_LEVEL_CHANGED   */
    boolean          failState;  /* valid when eventType == DIAG_EVT_FAILURE_CHANGED (TRUE=failed) */
} DiagEvent_t;

#define DIAG_LOG_SIZE (20U)

typedef struct
{
    SeatId_t   seatId;
    boolean    isFailure;        /* TRUE = failure logged, FALSE = sensor recovered */
    uint32     timestampTicks;   /* GPTM WTimer0 ticks (0.1ms resolution) */
} DiagFailureEntry_t;

typedef struct
{
    SeatId_t        seatId;
    HeatingLevel_t  level;
    uint32          timestampTicks;
} DiagLevelEntry_t;

/*******************************************************************************
 *                             Application Globals                             *
 *******************************************************************************/

static SeatState_t g_seat[NUM_SEATS];

/* RAM diagnostics log (Requirements item 4): failures + timestamps, and last heating level
 * changes + timestamps. Simple circular buffers, protected by xDiagMutex. */
static DiagFailureEntry_t g_diagFailureLog[DIAG_LOG_SIZE];
static uint8              g_diagFailureHead  = 0;
static uint8              g_diagFailureCount = 0;

static DiagLevelEntry_t   g_diagLevelLog[DIAG_LOG_SIZE];
static uint8              g_diagLevelHead    = 0;
static uint8              g_diagLevelCount   = 0;

/* Shared resource mutexes */
static SemaphoreHandle_t xADCMutex;
static SemaphoreHandle_t xUARTMutex;
static SemaphoreHandle_t xDiagMutex;

/* Event group for edge-triggered button interrupts (event-based task synchronization) */
static EventGroupHandle_t xButtonEventGroup;

/* Queue used to hand diagnostic events from producer tasks to vDiagnosticsTask without any
 * data loss (as opposed to an event group bit, which can coalesce fast repeated events) */
static QueueHandle_t xDiagEventQueue;

/*******************************************************************************
 *                          Private Function Prototypes                        *
 *******************************************************************************/

static void prvSetupHardware(void);
static void prvInitSeats(void);
static void prvCreateTasks(void);

static void vButtonTask(void *pvParameters);
static void vTempSenseTask(void *pvParameters);
static void vHeaterControlTask(void *pvParameters);
static void vDiagnosticsTask(void *pvParameters);
static void vDisplayTask(void *pvParameters);
static void vRunTimeStatsTask(void *pvParameters);

static const sint8* LevelName(HeatingLevel_t lvl);
static const sint8* IntensityName(HeaterIntensity_t inten);

/* Interrupt handlers - referenced from tm4c123gh6pm_startup_ccs.c's vector table */
void GPIOPortF_Handler(void);
void GPIOPortB_Handler(void);

/*******************************************************************************
 *                                   main()                                    *
 *******************************************************************************/

int main(void)
{
    prvSetupHardware();

    xButtonEventGroup = xEventGroupCreate();
    xDiagEventQueue   = xQueueCreate(DIAG_EVENT_QUEUE_LENGTH, sizeof(DiagEvent_t));

    xADCMutex  = xSemaphoreCreateMutex();
    xUARTMutex = xSemaphoreCreateMutex();
    xDiagMutex = xSemaphoreCreateMutex();

    RTM_Init();
    RTM_RegisterResource(RES_ADC,  (void *)xADCMutex,  (const sint8 *)"ADC");
    RTM_RegisterResource(RES_UART, (void *)xUARTMutex, (const sint8 *)"UART");
    RTM_RegisterResource(RES_DIAG, (void *)xDiagMutex, (const sint8 *)"Diagnostics");

    prvInitSeats();
    prvCreateTasks();

    UART0_SendString((const uint8 *)"\r\n**** Seat Heater Control System booted ****\r\n");

    vTaskStartScheduler();

    /* Should never reach here! If you do then there was not enough heap
     * available for the idle task to be created. */
    for (;;);
}

/*******************************************************************************
 *                         Hardware / Software Init                            *
 *******************************************************************************/

static void prvSetupHardware(void)
{
    /* Built-in buttons (driver/passenger console buttons) + built-in RGB LED (driver LEDs) */
    GPIO_BuiltinButtonsLedsInit();
    GPIO_SW1EdgeTriggeredInterruptInit();   /* PF4 - driver console button   */
    GPIO_SW2EdgeTriggeredInterruptInit();   /* PF0 - passenger console button */

    /* External passenger RGB LED + external driver steering-wheel button */
    GPIO_PassengerLedsInit();
    GPIO_ExtButtonInit();
    GPIO_ExtButtonEdgeTriggeredInterruptInit(); /* PB3 - driver steering wheel button */

    UART0_Init();
    GPTM_WTimer0Init();
    ADC_Init();
}

static void prvInitSeats(void)
{
    g_seat[SEAT_DRIVER].seatId          = SEAT_DRIVER;
    g_seat[SEAT_DRIVER].adcChannel      = ADC_CHANNEL_AIN0;
    g_seat[SEAT_DRIVER].name            = (const sint8 *)"Driver";
    g_seat[SEAT_DRIVER].heatingLevel    = LEVEL_OFF;
    g_seat[SEAT_DRIVER].desiredTempX10  = 0;
    g_seat[SEAT_DRIVER].currentTempX10  = 0;
    g_seat[SEAT_DRIVER].heaterIntensity = INTENSITY_OFF;
    g_seat[SEAT_DRIVER].heaterEnabled   = FALSE;
    g_seat[SEAT_DRIVER].sensorFail      = FALSE;
    g_seat[SEAT_DRIVER].mutex           = xSemaphoreCreateMutex();
    RTM_RegisterResource(RES_SEAT_DRIVER, (void *)g_seat[SEAT_DRIVER].mutex, (const sint8 *)"SeatDriver");

    g_seat[SEAT_PASSENGER].seatId          = SEAT_PASSENGER;
    g_seat[SEAT_PASSENGER].adcChannel      = ADC_CHANNEL_AIN1;
    g_seat[SEAT_PASSENGER].name            = (const sint8 *)"Passenger";
    g_seat[SEAT_PASSENGER].heatingLevel    = LEVEL_OFF;
    g_seat[SEAT_PASSENGER].desiredTempX10  = 0;
    g_seat[SEAT_PASSENGER].currentTempX10  = 0;
    g_seat[SEAT_PASSENGER].heaterIntensity = INTENSITY_OFF;
    g_seat[SEAT_PASSENGER].heaterEnabled   = FALSE;
    g_seat[SEAT_PASSENGER].sensorFail      = FALSE;
    g_seat[SEAT_PASSENGER].mutex           = xSemaphoreCreateMutex();
    RTM_RegisterResource(RES_SEAT_PASSENGER, (void *)g_seat[SEAT_PASSENGER].mutex, (const sint8 *)"SeatPassenger");
}

static void prvCreateTasks(void)
{
    TaskHandle_t h;

    xTaskCreate(vButtonTask, "DrvBtn", 128, &g_seat[SEAT_DRIVER], 3, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_DRIVER_BUTTON);

    xTaskCreate(vButtonTask, "PsgBtn", 128, &g_seat[SEAT_PASSENGER], 3, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_PASSENGER_BUTTON);

    xTaskCreate(vTempSenseTask, "DrvTemp", 128, &g_seat[SEAT_DRIVER], 2, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_DRIVER_TEMPSENSE);

    xTaskCreate(vTempSenseTask, "PsgTemp", 128, &g_seat[SEAT_PASSENGER], 2, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_PASSENGER_TEMPSENSE);

    xTaskCreate(vHeaterControlTask, "DrvHeat", 128, &g_seat[SEAT_DRIVER], 2, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_DRIVER_HEATER);

    xTaskCreate(vHeaterControlTask, "PsgHeat", 128, &g_seat[SEAT_PASSENGER], 2, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_PASSENGER_HEATER);

    xTaskCreate(vDiagnosticsTask, "Diag", 128, NULL, 2, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_DIAGNOSTICS);

    xTaskCreate(vDisplayTask, "Display", 128, NULL, 1, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_DISPLAY);

    xTaskCreate(vRunTimeStatsTask, "RTStats", 128, NULL, 1, &h);
    vTaskSetApplicationTaskTag(h, (TaskHookFunction_t)TASK_TAG_RUNTIME_STATS);
}

/*******************************************************************************
 *                                  Tasks                                      *
 *******************************************************************************/

/* Task 1/6 (reused x2): waits on the event bit set by its seat's button interrupt(s), then
 * cycles the heating level OFF -> LOW -> MEDIUM -> HIGH -> OFF. */
static void vButtonTask(void *pvParameters)
{
    SeatState_t *seat = (SeatState_t *)pvParameters;
    EventBits_t  bitToWaitFor = (seat->seatId == SEAT_DRIVER) ? BTN_BIT_DRIVER : BTN_BIT_PASSENGER;
    DiagEvent_t  evt;

    for (;;)
    {
        /* Block here until the ISR sets our seat's bit - fully interrupt driven, no polling */
        (void)xEventGroupWaitBits(xButtonEventGroup, bitToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);

        /* Simple software debounce */
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));

        xSemaphoreTake(seat->mutex, portMAX_DELAY);

        seat->heatingLevel = (HeatingLevel_t)((seat->heatingLevel + 1) % NUM_LEVELS);

        switch (seat->heatingLevel)
        {
            case LEVEL_LOW:    seat->desiredTempX10 = DESIRED_TEMP_LOW_X10;    break;
            case LEVEL_MEDIUM: seat->desiredTempX10 = DESIRED_TEMP_MEDIUM_X10; break;
            case LEVEL_HIGH:   seat->desiredTempX10 = DESIRED_TEMP_HIGH_X10;   break;
            case LEVEL_OFF:    /* fall through */
            default:           seat->desiredTempX10 = 0;                      break;
        }

        /* Re-arm the hysteresis latch on every level change (except turning fully OFF) so
         * the heater control task starts from a clean state for the new target level. */
        seat->heaterEnabled = (seat->heatingLevel != LEVEL_OFF) ? TRUE : FALSE;

        evt.seatId    = seat->seatId;
        evt.eventType = DIAG_EVT_LEVEL_CHANGED;
        evt.level     = seat->heatingLevel;

        xSemaphoreGive(seat->mutex);

        /* Hand off to the diagnostics task - queued, so no event is ever lost */
        (void)xQueueSend(xDiagEventQueue, &evt, 0);
    }
}

/* Task 2/6 (reused x2): periodically samples this seat's potentiometer through the shared
 * ADC module and validates the 5C-40C sensor range. */
static void vTempSenseTask(void *pvParameters)
{
    SeatState_t *seat = (SeatState_t *)pvParameters;
    TickType_t   lastWake = xTaskGetTickCount();
    uint16       raw;
    sint16       tempX10;
    boolean      newFail;
    boolean      failChanged;
    DiagEvent_t  evt;

    for (;;)
    {
        /* ADC0/SS3 is a single physical resource shared with the other seat's temp task */
        xSemaphoreTake(xADCMutex, portMAX_DELAY);
        raw = ADC_ReadChannel(seat->adcChannel);
        xSemaphoreGive(xADCMutex);

        tempX10 = ADC_ToCelsiusX10(raw);
        newFail = ((tempX10 < SENSOR_VALID_MIN_X10) || (tempX10 > SENSOR_VALID_MAX_X10)) ? TRUE : FALSE;

        xSemaphoreTake(seat->mutex, portMAX_DELAY);
        seat->currentTempX10 = tempX10;
        failChanged  = (seat->sensorFail != newFail) ? TRUE : FALSE;
        seat->sensorFail = newFail;
        xSemaphoreGive(seat->mutex);

        /* Only log on transitions (newly failed / just recovered), not on every sample */
        if (failChanged == TRUE)
        {
            evt.seatId    = seat->seatId;
            evt.eventType = DIAG_EVT_FAILURE_CHANGED;
            evt.failState = newFail;
            (void)xQueueSend(xDiagEventQueue, &evt, 0);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TEMPSENSE_PERIOD_MS));
    }
}

/* Task 3/6 (reused x2): runs the heater intensity state machine (Functionality item 2) and
 * drives this seat's status LEDs. */
static void vHeaterControlTask(void *pvParameters)
{
    SeatState_t       *seat = (SeatState_t *)pvParameters;
    TickType_t         lastWake = xTaskGetTickCount();
    sint16              diff;
    HeaterIntensity_t   intensityToShow;
    boolean             failToShow;

    for (;;)
    {
        xSemaphoreTake(seat->mutex, portMAX_DELAY);

        if ((seat->sensorFail == TRUE) || (seat->heatingLevel == LEVEL_OFF))
        {
            /* Sensor failure or heater switched off by the user: heater stays disabled */
            seat->heaterIntensity = INTENSITY_OFF;
            seat->heaterEnabled   = FALSE;
        }
        else
        {
            diff = (sint16)(seat->desiredTempX10 - seat->currentTempX10);

            /* Hysteresis: once switched off because target was reached, only switch back on
             * once the temperature has dropped 2C or more below the desired temperature */
            if ((seat->heaterEnabled == FALSE) && (diff >= DIFF_LOW_THRESHOLD_X10))
            {
                seat->heaterEnabled = TRUE;
            }

            if (seat->heaterEnabled == TRUE)
            {
                if (diff <= 0)
                {
                    /* Current temperature reached (or exceeded) the desired temperature */
                    seat->heaterEnabled   = FALSE;
                    seat->heaterIntensity = INTENSITY_OFF;
                }
                else if (diff >= DIFF_HIGH_THRESHOLD_X10)
                {
                    seat->heaterIntensity = INTENSITY_HIGH;
                }
                else if (diff >= DIFF_MEDIUM_THRESHOLD_X10)
                {
                    seat->heaterIntensity = INTENSITY_MEDIUM;
                }
                else if (diff >= DIFF_LOW_THRESHOLD_X10)
                {
                    seat->heaterIntensity = INTENSITY_LOW;
                }
                else
                {
                    /* Within 0-2C of target: keep the heater held off until the 2C
                     * hysteresis band re-arms it again (handled above on next iterations) */
                    seat->heaterIntensity = INTENSITY_OFF;
                }
            }
            else
            {
                seat->heaterIntensity = INTENSITY_OFF;
            }
        }

        intensityToShow = seat->heaterIntensity;
        failToShow      = seat->sensorFail;

        xSemaphoreGive(seat->mutex);

        /* Drive this seat's LEDs. Each seat exclusively owns its own LED pins, so no extra
         * synchronization is required for the hardware writes themselves. */
        if (seat->seatId == SEAT_DRIVER)
        {
            GPIO_GreenLedOff();
            GPIO_BlueLedOff();

            if (failToShow == TRUE)
            {
                GPIO_RedLedOn();
            }
            else
            {
                GPIO_RedLedOff();
                switch (intensityToShow)
                {
                    case INTENSITY_LOW:    GPIO_GreenLedOn(); break;
                    case INTENSITY_MEDIUM: GPIO_BlueLedOn();  break;
                    case INTENSITY_HIGH:   GPIO_GreenLedOn(); GPIO_BlueLedOn(); break;
                    default: /* INTENSITY_OFF -> both stay off */ break;
                }
            }
        }
        else /* SEAT_PASSENGER */
        {
            GPIO_PassengerGreenLedOff();
            GPIO_PassengerBlueLedOff();

            if (failToShow == TRUE)
            {
                GPIO_PassengerRedLedOn();
            }
            else
            {
                GPIO_PassengerRedLedOff();
                switch (intensityToShow)
                {
                    case INTENSITY_LOW:    GPIO_PassengerGreenLedOn(); break;
                    case INTENSITY_MEDIUM: GPIO_PassengerBlueLedOn();  break;
                    case INTENSITY_HIGH:   GPIO_PassengerGreenLedOn(); GPIO_PassengerBlueLedOn(); break;
                    default: /* INTENSITY_OFF -> both stay off */ break;
                }
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(HEATER_CONTROL_PERIOD_MS));
    }
}

/* Task 4/6 (single instance): event driven - blocks on the diagnostics queue and logs every
 * failure/recovery and every heating-level change into RAM with a GPTM timestamp. */
static void vDiagnosticsTask(void *pvParameters)
{
    DiagEvent_t evt;
    uint32      ts;

    (void)pvParameters;

    for (;;)
    {
        if (xQueueReceive(xDiagEventQueue, &evt, portMAX_DELAY) == pdPASS)
        {
            ts = GPTM_WTimer0Read();

            xSemaphoreTake(xDiagMutex, portMAX_DELAY);

            if (evt.eventType == DIAG_EVT_FAILURE_CHANGED)
            {
                g_diagFailureLog[g_diagFailureHead].seatId        = evt.seatId;
                g_diagFailureLog[g_diagFailureHead].isFailure     = evt.failState;
                g_diagFailureLog[g_diagFailureHead].timestampTicks = ts;

                g_diagFailureHead = (uint8)((g_diagFailureHead + 1U) % DIAG_LOG_SIZE);
                if (g_diagFailureCount < DIAG_LOG_SIZE)
                {
                    g_diagFailureCount++;
                }
            }
            else /* DIAG_EVT_LEVEL_CHANGED */
            {
                g_diagLevelLog[g_diagLevelHead].seatId         = evt.seatId;
                g_diagLevelLog[g_diagLevelHead].level          = evt.level;
                g_diagLevelLog[g_diagLevelHead].timestampTicks = ts;

                g_diagLevelHead = (uint8)((g_diagLevelHead + 1U) % DIAG_LOG_SIZE);
                if (g_diagLevelCount < DIAG_LOG_SIZE)
                {
                    g_diagLevelCount++;
                }
            }

            xSemaphoreGive(xDiagMutex);
        }
    }
}

/* Task 5/6 (single instance): periodically reports both seats' live status over the shared
 * UART "screen". */
static void vDisplayTask(void *pvParameters)
{
    TickType_t         lastWake = xTaskGetTickCount();
    static SeatState_t snap[NUM_SEATS];
    uint8               s;

    (void)pvParameters;

    for (;;)
    {
        for (s = 0; s < NUM_SEATS; s++)
        {
            xSemaphoreTake(g_seat[s].mutex, portMAX_DELAY);
            snap[s] = g_seat[s];
            xSemaphoreGive(g_seat[s].mutex);
        }

        xSemaphoreTake(xUARTMutex, portMAX_DELAY);

        UART0_SendString((const uint8 *)"\r\n---- Seat Heater Status ----\r\n");
        for (s = 0; s < NUM_SEATS; s++)
        {
            UART0_SendString((const uint8 *)snap[s].name);
            UART0_SendString((const uint8 *)": Level=");
            UART0_SendString((const uint8 *)LevelName(snap[s].heatingLevel));

            UART0_SendString((const uint8 *)" Desired=");
            UART0_SendInteger(snap[s].desiredTempX10 / 10);
            UART0_SendString((const uint8 *)".");
            UART0_SendInteger(snap[s].desiredTempX10 % 10);
            UART0_SendString((const uint8 *)"C");

            UART0_SendString((const uint8 *)" Current=");
            UART0_SendInteger(snap[s].currentTempX10 / 10);
            UART0_SendString((const uint8 *)".");
            UART0_SendInteger(snap[s].currentTempX10 % 10);
            UART0_SendString((const uint8 *)"C");

            UART0_SendString((const uint8 *)" Heater=");
            UART0_SendString((const uint8 *)IntensityName(snap[s].heaterIntensity));

            UART0_SendString((const uint8 *)" Fault=");
            UART0_SendString((const uint8 *)((snap[s].sensorFail == TRUE) ? "YES" : "NO"));

            UART0_SendString((const uint8 *)"\r\n");
        }

        xSemaphoreGive(xUARTMutex);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

/* Task 6/6 (single instance): periodically reports the manual runtime measurements collected
 * via the FreeRTOS trace hooks (see FreeRTOSConfig.h + Common/RunTimeMeasurements.c). */
static void vRunTimeStatsTask(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    uint8      t;
    uint8      r;
    uint32     elapsedTicks;
    uint32     execTicks;
    uint32     lockTicks;
    uint32     loadPercentX10;

    (void)pvParameters;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(RUNTIME_STATS_PERIOD_MS));

        elapsedTicks = RTM_GetElapsedTicks();
        if (elapsedTicks == 0U)
        {
            elapsedTicks = 1U; /* avoid divide-by-zero on the very first report */
        }

        xSemaphoreTake(xUARTMutex, portMAX_DELAY);

        UART0_SendString((const uint8 *)"\r\n==== Run Time Measurements ====\r\n");
        UART0_SendString((const uint8 *)"Task : ExecTime(ms) : CPU Load(%)\r\n");

        for (t = (uint8)TASK_TAG_DRIVER_BUTTON; t <= (uint8)TASK_TAG_RUNTIME_STATS; t++)
        {
            execTicks      = RTM_GetTaskExecTimeTicks((TaskTag_t)t);
            loadPercentX10 = (uint32)(((uint64)execTicks * 1000UL) / (uint64)elapsedTicks);

            UART0_SendString((const uint8 *)RTM_GetTaskName((TaskTag_t)t));
            UART0_SendString((const uint8 *)" : ");
            UART0_SendInteger(execTicks / 10U);
            UART0_SendString((const uint8 *)" : ");
            UART0_SendInteger(loadPercentX10 / 10U);
            UART0_SendString((const uint8 *)".");
            UART0_SendInteger(loadPercentX10 % 10U);
            UART0_SendString((const uint8 *)"\r\n");
        }

        UART0_SendString((const uint8 *)"---- Resource Lock Time (ms), per task ----\r\n");
        for (r = (uint8)RES_ADC; r < (uint8)RES_COUNT; r++)
        {
            UART0_SendString((const uint8 *)RTM_GetResourceName((ResourceId_t)r));
            UART0_SendString((const uint8 *)": ");

            for (t = (uint8)TASK_TAG_DRIVER_BUTTON; t <= (uint8)TASK_TAG_RUNTIME_STATS; t++)
            {
                lockTicks = RTM_GetResourceLockTimeTicks((TaskTag_t)t, (ResourceId_t)r);
                if (lockTicks > 0U)
                {
                    UART0_SendString((const uint8 *)RTM_GetTaskName((TaskTag_t)t));
                    UART0_SendString((const uint8 *)"=");
                    UART0_SendInteger(lockTicks / 10U);
                    UART0_SendString((const uint8 *)"ms ");
                }
            }
            UART0_SendString((const uint8 *)"\r\n");
        }

        /* Diagnostics RAM log dump (Requirements item 4: failures+timestamp, level changes+timestamp) */
        UART0_SendString((const uint8 *)"---- Diagnostics Log (RAM) ----\r\n");
        xSemaphoreTake(xDiagMutex, portMAX_DELAY);
        {
            uint8 i;
            uint8 idx;

            UART0_SendString((const uint8 *)"Failures logged: ");
            UART0_SendInteger(g_diagFailureCount);
            UART0_SendString((const uint8 *)"\r\n");
            for (i = 0; i < g_diagFailureCount; i++)
            {
                idx = (uint8)((g_diagFailureHead + DIAG_LOG_SIZE - g_diagFailureCount + i) % DIAG_LOG_SIZE);
                UART0_SendString((const uint8 *)((g_diagFailureLog[idx].seatId == SEAT_DRIVER) ? "Driver" : "Passenger"));
                UART0_SendString((const uint8 *)((g_diagFailureLog[idx].isFailure == TRUE) ? " FAILED @t=" : " RECOVERED @t="));
                UART0_SendInteger(g_diagFailureLog[idx].timestampTicks / 10U);
                UART0_SendString((const uint8 *)"ms\r\n");
            }

            UART0_SendString((const uint8 *)"Level changes logged: ");
            UART0_SendInteger(g_diagLevelCount);
            UART0_SendString((const uint8 *)"\r\n");
            for (i = 0; i < g_diagLevelCount; i++)
            {
                idx = (uint8)((g_diagLevelHead + DIAG_LOG_SIZE - g_diagLevelCount + i) % DIAG_LOG_SIZE);
                UART0_SendString((const uint8 *)((g_diagLevelLog[idx].seatId == SEAT_DRIVER) ? "Driver" : "Passenger"));
                UART0_SendString((const uint8 *)" Level=");
                UART0_SendString((const uint8 *)LevelName(g_diagLevelLog[idx].level));
                UART0_SendString((const uint8 *)" @t=");
                UART0_SendInteger(g_diagLevelLog[idx].timestampTicks / 10U);
                UART0_SendString((const uint8 *)"ms\r\n");
            }
        }
        xSemaphoreGive(xDiagMutex);

        xSemaphoreGive(xUARTMutex);
    }
}

/*******************************************************************************
 *                          Interrupt Service Routines                         *
 *******************************************************************************/

/* PF4 = SW1 = driver console button, PF0 = SW2 = passenger console button */
void GPIOPortF_Handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((GPIO_PORTF_RIS_REG & (1 << 4)) != 0)
    {
        (void)xEventGroupSetBitsFromISR(xButtonEventGroup, BTN_BIT_DRIVER, &xHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG |= (1 << 4);
    }

    if ((GPIO_PORTF_RIS_REG & (1 << 0)) != 0)
    {
        (void)xEventGroupSetBitsFromISR(xButtonEventGroup, BTN_BIT_PASSENGER, &xHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG |= (1 << 0);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* PB3 = external driver steering wheel button - ORs into the same driver button bit as SW1 */
void GPIOPortB_Handler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if ((GPIO_PORTB_RIS_REG & (1 << 3)) != 0)
    {
        (void)xEventGroupSetBitsFromISR(xButtonEventGroup, BTN_BIT_DRIVER, &xHigherPriorityTaskWoken);
        GPIO_PORTB_ICR_REG |= (1 << 3);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
 *                              Helper Functions                               *
 *******************************************************************************/

static const sint8* LevelName(HeatingLevel_t lvl)
{
    switch (lvl)
    {
        case LEVEL_OFF:    return (const sint8 *)"OFF";
        case LEVEL_LOW:    return (const sint8 *)"LOW";
        case LEVEL_MEDIUM: return (const sint8 *)"MEDIUM";
        case LEVEL_HIGH:   return (const sint8 *)"HIGH";
        default:           return (const sint8 *)"?";
    }
}

static const sint8* IntensityName(HeaterIntensity_t inten)
{
    switch (inten)
    {
        case INTENSITY_OFF:    return (const sint8 *)"OFF";
        case INTENSITY_LOW:    return (const sint8 *)"LOW";
        case INTENSITY_MEDIUM: return (const sint8 *)"MEDIUM";
        case INTENSITY_HIGH:   return (const sint8 *)"HIGH";
        default:                return (const sint8 *)"?";
    }
}

/*-----------------------------------------------------------*/
