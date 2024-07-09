#ifndef ALTITUDE_H_
#define ALTITUDE_H_

//*******************************************************************************
// altitude.h
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

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 25      // Buffer Size

//*****************************************************************************
// Reference Altitude ADC value initialiser. Sets the reference altitude ADC 
// value (refAltADC) which is used in altitude calculations.
//*****************************************************************************
void initAlt(Helicopter* heli);

//*****************************************************************************
// Background task: calculate the (approximate) mean of the values in the
// circular buffer.
//*****************************************************************************
void CalculateAltitude(Helicopter* heli);

#endif /* ALTITUDE_H_ */
