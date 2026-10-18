/**********************************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.h
 *
 * Description: Header file for the TM4C123GH6PM DIO driver for TivaC Built-in Buttons and LEDs
 *
 * Author: Adapted for the Seat Heater Control System (original single-channel driver by Edges for Training Team)
 *
 ***********************************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

#include "std_types.h"

#define GPIO_PORTF_PRIORITY_MASK      0xFF1FFFFF
#define GPIO_PORTF_PRIORITY_BITS_POS  21
#define GPIO_PORTF_INTERRUPT_PRIORITY 5

/* GPIO Port B is used for:
 *   - PB3: external push button (Seat Heater Control System - driver's steering wheel button)
 *   - PB0/PB1/PB2: external Green/Blue/Red LEDs (Seat Heater Control System - passenger seat
 *     heating-level & diagnostic indication, since the board only has a single built-in RGB LED
 *     which is used for the driver seat) */
#define GPIO_PORTB_PRIORITY_MASK      0xFFFF1FFF
#define GPIO_PORTB_PRIORITY_BITS_POS  13
#define GPIO_PORTB_INTERRUPT_PRIORITY 5

#define PRESSED                ((uint8)0x00)
#define RELEASED               ((uint8)0x01)

void GPIO_BuiltinButtonsLedsInit(void);

void GPIO_RedLedOn(void);
void GPIO_BlueLedOn(void);
void GPIO_GreenLedOn(void);

void GPIO_RedLedOff(void);
void GPIO_BlueLedOff(void);
void GPIO_GreenLedOff(void);

void GPIO_RedLedToggle(void);
void GPIO_BlueLedToggle(void);
void GPIO_GreenLedToggle(void);

uint8 GPIO_SW1GetState(void);
uint8 GPIO_SW2GetState(void);

void GPIO_SW1EdgeTriggeredInterruptInit(void);
void GPIO_SW2EdgeTriggeredInterruptInit(void);

/*******************************************************************************
 *      Seat Heater Control System additions: external button + passenger LEDs
 *******************************************************************************/

/* PB0 = Green LED, PB1 = Blue LED, PB2 = Red LED (passenger seat indication) */
void GPIO_PassengerLedsInit(void);
void GPIO_PassengerGreenLedOn(void);
void GPIO_PassengerGreenLedOff(void);
void GPIO_PassengerBlueLedOn(void);
void GPIO_PassengerBlueLedOff(void);
void GPIO_PassengerRedLedOn(void);
void GPIO_PassengerRedLedOff(void);

/* PB3 = external push button (driver steering wheel button), active LOW with pull-up */
void GPIO_ExtButtonInit(void);
void GPIO_ExtButtonEdgeTriggeredInterruptInit(void);
uint8 GPIO_ExtButtonGetState(void);

#endif /* GPIO_H_ */
