#ifndef BUFFER_H_
#define BUFFER_H_

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
// Last modified:   17.05.24
//*******************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "rotors.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 25      // Buffer Size

//*****************************************************************************
// Initialise the circular buffer that heli will use, see note below far right...
//*****************************************************************************
static circBuf_t g_inBuffer;                                                                              // are we sure this is globally readable by buffers?

//*****************************************************************************
// Circular Buffer Initialiser for ADC altitude inputs.
void initBuffer(void);

//*****************************************************************************
// ADC intialiser for Altitude related voltage measurements.
void ADCIntHandler(void);

//*****************************************************************************
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
void initADC (void);

//*****************************************************************************
// Background task: calculate the (approximate) mean of the values in the
// circular buffer.
void BufferCalculate(Helicopter* heli);

#endif /* BUFFER_H_ */
