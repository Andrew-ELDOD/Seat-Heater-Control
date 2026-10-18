/**********************************************************************************************
 *
 * Module: ADC
 *
 * File Name: adc.h
 *
 * Description: Header file for the TM4C123GH6PM ADC0 driver.
 *              Uses ADC0 Sample Sequencer 3 (single sample, software triggered) so that a
 *              single 12-bit conversion can be requested on demand from any of the analog
 *              input channels wired on GPIO Port E (AIN0 = PE3, AIN1 = PE2, ...).
 *
 *              In the Seat Heater Control System this driver is shared between the driver
 *              seat potentiometer (AIN0 / PE3) and the passenger seat potentiometer
 *              (AIN1 / PE2). Because both seats use the SAME physical ADC module and the
 *              SAME sample sequencer, access to this driver MUST be serialized with a mutex
 *              by the application (see xADCMutex in main.c) before calling ADC_ReadChannel().
 *
 * Author: Adapted for the Seat Heater Control System (original single-channel driver by Andrew)
 *
 ***********************************************************************************************/

#ifndef ADC_H_
#define ADC_H_

#include "std_types.h"

/*******************************************************************************
 *                             Preprocessor Macros                             *
 *******************************************************************************/

/* ADC input channel definitions (channel number == AINx number) */
#define ADC_CHANNEL_AIN0        ((uint8)0)   /* PE3 - Driver seat potentiometer    */
#define ADC_CHANNEL_AIN1        ((uint8)1)   /* PE2 - Passenger seat potentiometer */

/* 12-bit ADC resolution */
#define ADC_MAX_DIGITAL_VALUE   ((uint16)4095)

/* Potentiometer voltage-to-temperature mapping as per the requirements document:
 * 0V - 3.3V (0 - 4095 counts) is mapped linearly to 0C - 45C.
 * Temperature is represented in tenths of a degree Celsius (fixed point, x10)
 * so that 0.1C resolution can be handled using integer arithmetic only. */
#define ADC_TEMP_MAP_MAX_X10    ((uint16)450) /* 45.0 C * 10 */

/*******************************************************************************
 *                            Functions Prototypes                             *
 *******************************************************************************/

/*
 * Description: Initializes ADC0 Sample Sequencer 3 for software triggered, single ended,
 *              single sample conversions and configures PE3 (AIN0) and PE2 (AIN1) as
 *              analog inputs so both seat potentiometers can be sampled through it.
 * Parameters: None
 * Returns: None
 */
void ADC_Init(void);

/*
 * Description: Selects the given channel on Sample Sequencer 3, triggers a single
 *              conversion and blocks (busy wait) until the result is ready.
 *              NOT reentrant / NOT thread-safe: the caller (application task) is
 *              responsible for taking the shared ADC mutex before calling this function
 *              and releasing it right after, since SS3 is shared between both seats.
 * Parameters:
 *   - channel: ADC_CHANNEL_AIN0 or ADC_CHANNEL_AIN1 (or any other configured AIN channel)
 * Returns: 12-bit raw ADC conversion result (0 - 4095)
 */
uint16 ADC_ReadChannel(uint8 channel);

/*
 * Description: Converts a raw 12-bit ADC reading to millivolts, assuming a 3.3V reference.
 * Parameters:
 *   - adcValue: Raw ADC reading (0 - 4095)
 * Returns: Voltage in millivolts (0 - 3300)
 */
uint32 ADC_ToMillivolts(uint16 adcValue);

/*
 * Description: Converts a raw 12-bit ADC reading coming from the seat potentiometer into
 *              a simulated temperature reading, in tenths of a degree Celsius, using the
 *              0V-3.3V -> 0C-45C linear mapping required by the project specification.
 * Parameters:
 *   - adcValue: Raw ADC reading (0 - 4095)
 * Returns: Temperature in tenths of a degree Celsius (0 - 450), i.e. divide by 10 to get C
 */
sint16 ADC_ToCelsiusX10(uint16 adcValue);

#endif /* ADC_H_ */
