//*******************************************************************************
// altitude.c
//
// This file manages altitude-related functionalities for the helicopter control
// system. It includes functions for initializing the reference altitude ADC
// value, calculating altitude percentage based on ADC readings, and updating
// altitude values in the system. Additionally, it utilizes a circular buffer to
// store altitude readings and calculates the mean value for accurate altitude
// estimation.
//
// Author:  R.J Ross, H. Donley
//
// Last modified:   17.05.24
//*******************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "buffer.h"
#include "yaw.h"
#include "altitude.h"
#include "rotors.h"
#include "system.h"

//*****************************************************************************
// Reference Altitude ADC value initialiser. Sets the reference altitude ADC 
// value (refAltADC) which is used in altitude calculations. It delays this
// calculation until the buffer is filled (of size 25, by Systick).
//*****************************************************************************
void
initAlt(Helicopter* heli)
{
    uint8_t sysTickCounter = 0;  // Tracks number of SysTick calls
    uint8_t intialise = 0;   // Tracks whether buffer has been correctly filled.
    // Circular altitude buffer fills itself (25 values fill entire buffer) via systick
    while (!intialise) {
        if (DeltaTFlag)
        {
            SysTick(heli);
            sysTickCounter++;

            // Once buffer full, determine reference altitude value by taking the mean
            // of the buffer values.
            if (sysTickCounter >= 25)
            {
                BufferCalculate(heli);
                heli->buffer->refAltADC = heli->buffer->meanVal;  // Sets reference ADC value to current buffer mean value
                intialise = true;
            }
        }
    }
}

//*****************************************************************************
// Background task: calculate the (approximate) mean of the values in the
// circular buffer.Converts ADC input to altitude percentage relative to -1V. 
// It takes the difference between the current reading and the reference ADC 
// reading. This is divided by 1241 as it relates to a 1V range of the ADC readings.
// This value is scaled by 100 to output a percentage.
//*****************************************************************************
void
CalculateAltitude(Helicopter* heli)
{
    BufferCalculate(heli); // Update mean value of buffer.
    heli->controller->curr_altitude_reading = (-((heli->buffer->meanVal - heli->buffer->refAltADC)* 100)/1241);
}
