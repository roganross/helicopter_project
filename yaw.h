#ifndef YAW_H
#define YAW_H

//*******************************************************************************
// yaw.h
//
// This file manages yaw-related functionalities for the helicopter controls.
// It includes functions for initializing yaw peripheral pins, reading quadrature
// decoder output, handling interrupts from yaw input signals, and calculating
// yaw angles. Additionally, it utilizes an adjustment table to adjust yaw angles
// based on quadrature decoder readings."
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
#define YAW_ANGLE_START_POSITION       0
#define NUM_SLOTS       112
#define TOTAL_DEG       360
#define TOTAL_STATES    NUM_SLOTS * 4
#define HALF_TOTAL_STATES    NUM_SLOTS * 2
#define YAW_DELTA       TOTAL_DEG / (TOTAL_STATES)

#define REF_SIGNAL      true

//Interrupt flags for yaw related function
extern volatile uint8_t YawIntFlag;
extern volatile uint8_t YawRefFlag;

//*****************************************************************************
// Initialization of the reference yaw interrupt
void initRefYaw(void);

//*****************************************************************************
// Manages the interrupt handler for the reference yaw, triggers YawRefFlag
void YawRefIntHandler(void);

//*****************************************************************************
// Reads Quadrecture Decoder output & converts to a 2-bit number and concatenates
// them.
int32_t ReadQuadrectureDecoder (void);

//*****************************************************************************
// Function to initialize yaw peripheral pins PB0 and PB1.
void initYawPeripherals(Helicopter* heli);

//*****************************************************************************
// Interrupt Handler for Yaw Input Signals. Changes flag value
void YawIntHandler(void);

//*****************************************************************************
// Interrupt Handler for Yaw Input Signals. Reads Quadrecture Decoder and
// adjusts Helicopter 'heli' Yaw Angle accordingly to table
void ExecuteYawInt(Helicopter* heli);

//*****************************************************************************
// Gets the current Yaw Angle Value for Main.
int16_t GetYawAngleDegrees(Helicopter* heli);

#endif // YAW_H
