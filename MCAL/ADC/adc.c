/**********************************************************************************************
 *
 * Module: ADC
 *
 * File Name: adc.c
 *
 * Description: Source file for the TM4C123GH6PM ADC0 driver (see adc.h for details).
 *
 ***********************************************************************************************/
#include "adc.h"
#include "tm4c123gh6pm_registers.h"

void ADC_Init(void)
{
    volatile uint32 delay;

    /* Enable ADC0 clock */
    SYSCTL_RCGCADC_REG |= 0x01;         /* Enable ADC0 clock */
    delay = SYSCTL_RCGCADC_REG;         /* Allow time for clock to stabilize */
    (void)delay;

    /* Enable GPIO Port E clock (PE3 = AIN0 / driver pot, PE2 = AIN1 / passenger pot) */
    SYSCTL_RCGCGPIO_REG |= 0x10;        /* Enable Port E clock */
    delay = SYSCTL_RCGCGPIO_REG;        /* Allow time for clock to stabilize */
    (void)delay;
    while(!(SYSCTL_PRGPIO_REG & 0x10)); /* Wait until Port E is ready for access */

    /* Configure PE3 (AIN0) and PE2 (AIN1) as analog inputs */
    GPIO_PORTE_DIR_REG    &= ~0x0C;     /* Set PE3 & PE2 as input pins   */
    GPIO_PORTE_AFSEL_REG  |=  0x0C;     /* Enable alternate function on PE3 & PE2 */
    GPIO_PORTE_DEN_REG    &= ~0x0C;     /* Disable digital I/O on PE3 & PE2 */
    GPIO_PORTE_AMSEL_REG  |=  0x0C;     /* Enable analog function on PE3 & PE2 */
    GPIO_PORTE_PCTL_REG   &= 0xFFFF00FF;/* Clear PMCx bits for PE3 & PE2 (analog mode) */

    /* Configure ADC0 Sample Sequencer 3 (single sample, software triggered) */
    ADC0_ACTSS_REG  &= ~0x08;           /* Disable Sample Sequencer 3 during configuration */
    ADC0_EMUX_REG   &= ~0xF000;         /* Software trigger for SS3 */
    ADC0_SSMUX3_REG  =  ADC_CHANNEL_AIN0; /* Default channel, will be re-selected per read */
    ADC0_SSCTL3_REG  =  0x06;           /* Single sample: END0 and IE0 set */
    ADC0_SAC_REG     =  0x00;           /* Disable hardware averaging */
    ADC0_CTL_REG    &= ~0x01;           /* Clear VREF bit to use internal 3.3V reference */
    ADC0_ACTSS_REG  |=  0x08;           /* Enable Sample Sequencer 3 */
}

uint16 ADC_ReadChannel(uint8 channel)
{
    uint16 result;

    ADC0_SSMUX3_REG = (channel & 0x0F); /* Select the channel to sample for SS3 */
    ADC0_PSSI_REG   = 0x08;             /* Initiate SS3 conversion */
    while((ADC0_RIS_REG & 0x08) == 0);  /* Wait for conversion complete */
    result = (uint16)(ADC0_SSFIFO3_REG & 0x0FFF); /* Read 12-bit result */
    ADC0_ISC_REG    = 0x08;             /* Clear completion flag */

    return result;
}

uint32 ADC_ToMillivolts(uint16 adcValue)
{
    /* Convert to millivolts: (adcValue * 3300) / 4095 */
    return (((uint32)adcValue * 3300UL) / (uint32)ADC_MAX_DIGITAL_VALUE);
}

sint16 ADC_ToCelsiusX10(uint16 adcValue)
{
    /* Linear map: 0 .. 4095 counts (0V .. 3.3V) -> 0 .. 450 tenths of a degree (0C .. 45C) */
    uint32 tempX10 = ((uint32)adcValue * (uint32)ADC_TEMP_MAP_MAX_X10) / (uint32)ADC_MAX_DIGITAL_VALUE;
    return (sint16)tempX10;
}
