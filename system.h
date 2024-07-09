#ifndef SYSTEM_H_
#define SYSTEM_H_

//*******************************************************************************
// system.c
//
// system.c serves as the central hub for the embedded system controlling the
// helicopter. It includes initialization functions for clock settings, ADC,
// display, UART, and interrupts. Additionally, it manages the SysTick timer and
// implements functions for updating system state and displaying project-specific
// information on the OLED screen."
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
#define SAMPLE_RATE_HZ 150    // Sample Rate for ADC inputs
#define SYSTICK_RATE_HZ 100   // Systick frequency
#define SLOWTICK_RATE_HZ 8    // Slowtick frequency
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define MAIN_ROTOR_SELECT    0
#define TAIL_ROTOR_SELECT    1

// Conversion parameters for states to degrees (yaw).
#define NUM_SLOTS       112
#define TOTAL_DEG       360
#define TOTAL_STATES    NUM_SLOTS * 4
#define YAW_DELTA       TOTAL_DEG / (TOTAL_STATES)

//Interrupt flags for the system self-clearing deltaTFlag and slowtick.
extern volatile uint8_t DeltaTFlag;
extern volatile uint8_t slowTick;
extern volatile uint8_t ResetFlag;

//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
void SysTickIntHandler(void);

//*****************************************************************************
//Handles functionaity for an interrupt that prints to UART
//*****************************************************************************
void SysTick(Helicopter* heli);

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display, UART
// & Helicopter Interrupts
//*****************************************************************************
void initClock (void);

//*****************************************************************************
// Function to initialize all parts of helicopter.
//*****************************************************************************
void initHelicopter(Helicopter* heli);

//*****************************************************************************
// Function to show screen state, modified for Project
// Displays altitude (%), yaw angle (degrees), main rotor duty cycle (%)
// and tail rotor duty cycle (%).
//*****************************************************************************
void DisplayProject(Helicopter* heli);

#endif /* SYSTEM_H_ */
