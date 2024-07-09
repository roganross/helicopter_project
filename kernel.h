#ifndef KERNEL_H_
#define KERNEL_H_

//*******************************************************************************
// kernel.c
//
// This kernel file is the core of the embedded system, managing critical
// operations for helicopter control. It handles task scheduling, mode changing,
// interrupts, and PID control. Designed for real-time efficiency, it ensures
// smooth execution of flight modes: TAKEOFF, LANDING, RESET, and FLY.
//
// Author:  R.J Ross, H. Donley
//
// Last modified:   17.05.24
//*******************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "rotors.h"

//*****************************************************************************
// Runs the kernel program with interrupt driven task management. Counter based
// flags raised for the OLED display to run this 20x slower than the controller
// (deltaTFlag).
//*****************************************************************************
void Run_Kernel(Helicopter* heli);

#endif /* KERNEL_H_ */
