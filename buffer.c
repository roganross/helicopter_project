//*******************************************************************************
// buffer.c
//
// This file handles the circular buffer and ADC functionalities for altitude
// measurements in the helicopter control system. It initializes the buffer,
// configures ADC settings, stores ADC readings in the buffer, and calculates
// the mean value of stored readings for altitude estimation.
//
// Author:  R.J Ross, H. Donley
//
// Reference: P.J Bones
//
// Last modified:   17.05.24
//*******************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "buffer.h"

//*****************************************************************************
// Circular Buffer Initialiser for ADC altitude inputs.
//*****************************************************************************
void
initBuffer(void)
{
    initCircBuf (&g_inBuffer, BUF_SIZE);
}

//*****************************************************************************
// ADC intialiser for Altitude related voltage measurements.
//*****************************************************************************
void
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END); //change to CH9 for testing

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;
    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer , ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
// Background task: calculate the (approximate) mean of the values in the
// circular buffer.
//*****************************************************************************
void
BufferCalculate(Helicopter* heli)
{
    // Iterates through the buffer and sums values then calculates the mean
    int32_t sum = 0;
    uint16_t i;
    for (i = 0; i < BUF_SIZE; i++) {
        sum += readCircBuf (&g_inBuffer);
    }
    heli->buffer->meanVal = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
}
