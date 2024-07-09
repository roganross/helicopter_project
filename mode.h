#ifndef MODE_H_
#define MODE_H_

//*******************************************************************************
// mode.h
//
// This mode file implements the FSM for RESET, TAKEOFF, LANDING and FLY modes.
// It uses a locate pivot function while in USER_DISABLED mode to find the
// reference yaw in both takeoff and landing procedures. It also handles the
// SysCtlReset() interrupt.
//
// Author:  R.J Ross, H. Donley
//
// Last modified:   17.05.24
//*******************************************************************************

#include <stdint.h>
#include <stdbool.h>

//Interrupts to drive modes
extern volatile uint8_t ChangeMode;
extern volatile uint8_t EnableLanding;

// Switch 1 (Mode Control) Assignments
// Switch 1 (Mode Control) Assignments
#define SW1_PIN    GPIO_PIN_7
#define SW2_PIN    GPIO_PIN_6
#define SW_PORT   GPIO_PORTA_BASE

//*****************************************************************************
// Initialize peripherals SW1 & SW2 & interrupt on switch 1
void initSWS(void);

//*****************************************************************************
// The interrupt handler that triggers the 'changemode' flag, which then
// does 'executehelicopterMode(Helicopter* heli)'
void ModeSWTickIntHandler(void);

//*****************************************************************************
// A function to change the current mode based on the new switch position,
// between TAKEOFF, LANDING and FLY
void ExecuteHelicopterMode(Helicopter* heli);

//*****************************************************************************
// With mode in 'USER_DISABLED' state, this function rotates at a slow constant
// speed looking for a YawRefFlag to signify the reference position. Depending
// on if coming from LANDED or FLY states, LocatePivot may take a shortest path
// if the yawanglesetpoint was initially set
void LocatePivot(Helicopter* heli);

//*****************************************************************************
// In coming from a 'FLY' mode, user peripherals become disabled in Land Mode.
// The helicopter is lowered to 5% altitude (approx. depending on gravity),
// rotated to find the reference position, and then taken to ground level.
void ModeLand(Helicopter* heli);

//*****************************************************************************
// ModeTakeoff is only enabled from a 'LANDED' mode state. The helicopter is
// risen to 5% altitude and goes through 'locatepivot' to find the reference
// yaw position. It then rises to 10% altitude and user buttons are enabled
// via 'USER_ENABLED'.
void ModeTakeoff(Helicopter* heli);

//*****************************************************************************
// In 'flying' submode, the mode is 'USER_ENABLED' and push buttons are
// enabled. Altitude increments in 10% steps (capped between 0 and 100%) and
// yaw in 15degree steps with no limitations on rotation.
void ModeFly(Helicopter* heli);

//*****************************************************************************
// Sets the main rotor and tail rotor duty cycles to be 0%. It then uses
// setPWM to transfer these duty's to the periherals.
void StopRotors(Helicopter* heli);

#endif /* MODE_H_ */
