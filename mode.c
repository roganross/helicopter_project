//*******************************************************************************
// mode.c
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

#include <buttons4.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "yaw.h"
#include "buffer.h"
#include "altitude.h"
#include "system.h"
#include "uart.h"
#include "rotors.h"
#include "mode.h"

//Interrupts to drive modes
volatile uint8_t ChangeMode = 0;
volatile uint8_t EnableLanding = 0;

//*****************************************************************************
// Initialize peripherals SW1 & SW2 & interrupt on switch 1
//*****************************************************************************
void
initSWS(void)
{
    SysCtlPeripheralEnable(SW_PORT);

    GPIOPadConfigSet(SW_PORT, SW1_PIN, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIODirModeSet(SW_PORT, SW1_PIN, GPIO_DIR_MODE_IN);
    GPIOIntDisable(SW_PORT, SW1_PIN);

    GPIOPadConfigSet(GPIO_PORTA_BASE, SW2_PIN, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIODirModeSet(GPIO_PORTA_BASE, SW2_PIN, GPIO_DIR_MODE_IN);
    GPIOIntDisable(GPIO_PORTA_BASE, SW2_PIN);

    // Switch 1 interrupt
    GPIOIntTypeSet(SW_PORT, SW1_PIN, GPIO_BOTH_EDGES);
    GPIOIntRegister(SW_PORT, ModeSWTickIntHandler);

    GPIOIntEnable(SW_PORT, SW1_PIN);

    // Sets initial value
    ChangeMode = GPIOPinRead(SW_PORT, SW1_PIN);
}


//*****************************************************************************
// Interrupt to manage helicopter modes
// The interrupt handler that triggers the 'changemode' flag, which then
// does 'executehelicopterMode(Helicopter* heli)' to actually change the mode.
//*****************************************************************************
void
ModeSWTickIntHandler(void) // very short function, to minimise the chance of data problems
{
    ChangeMode = 1; // if reset with SW1 in takeoff positon, do not takeoff until switch reswitched into on position
    GPIOIntClear(SW_PORT, SW1_PIN);
}

//*****************************************************************************
// A function to change the current mode based on the new switch position,
// between TAKEOFF, LANDING and FLY. There was difficulty in initializing two
// interrupts (SW1 Mode and SW2 Reset) on the same GPIO port base, and SW2 was
// heavily embedded into the existing program, so SW1 reset was changed to a
// polling based interrupt.
// ****************************************************************************
void
ExecuteHelicopterMode(Helicopter* heli)
{
    // Check if SW1 switch is pressed (high state)
    uint8_t state = GPIOPinRead(SW_PORT, SW1_PIN);
    if (state) {
        ModeTakeoff(heli);
        EnableLanding = true;  //enable a landing procedure only after taking off
    } else {
        ModeLand(heli);
    }
}

//*****************************************************************************
// With mode in 'USER_DISABLED' state, this function rotates at a slow constant
// speed looking for a YawRefFlag to signify the reference position. Depending
// on if coming from LANDED or FLY states, LocatePivot may take a shortest path
// if the yawanglesetpoint was initially set.
// ****************************************************************************
void LocatePivot(Helicopter* heli)
{
    YawRefFlag = 0;  // Resets state to locate reference
    // Increase tail rotor speed to trigger slow rotation to find pivot
    while(!YawRefFlag) {
        if (DeltaTFlag)
        {
            ResetFlag = (GPIOPinRead(SW_PORT, SW2_PIN));
            DeltaTFlag = 0;
            SysTick(heli);
            if (slowTick)
            {
                DisplayProject(heli);
            }

            heli->controller->yawanglesetpoint = heli->controller->curr_yawangle_reading + 10;       //move the setpoint at a constant rate from the current reading, to trigger controls to converge or move
                                                                                                     //towards the reference yaw position.
            ControllerImplementation(heli);
            SetPWM(heli->mainrotor);   // Updates main rotor PWM
            SetPWM(heli->tailrotor);   // Updates tail rotor PWM
        }
        if (YawIntFlag)
        {
            YawIntFlag = 0;
            ExecuteYawInt(heli);
        }
        if (ResetFlag != 0)
        {
            SysCtlReset();
        }
    }
    YawRefFlag = 0;
    heli->controller->yawanglesetpoint = 0;   // Set sets setpoint to be zero at reference
    heli->controller->curr_yawangle_reading = 0;  //once the reference position is found, trigger current reading and yaw set point to 0 so that controls maintain this value.

}

//*****************************************************************************
// In coming from a 'FLY' mode, user peripherals become disabled in Land Mode.
// The helicopter is lowered to 5% altitude (approx. depending on gravity),
// rotated to find the reference position, and then taken to ground level.
// In coming from a 'FLY' mode, user peripherals become disabled in Land Mode.
// The helicopter is lowered to 5% altitude (approx. depending on gravity),
// rotated to find the reference position, and then taken to ground level.
//*****************************************************************************
void ModeLand(Helicopter* heli)
{
    heli->submode = LANDED;
    if (EnableLanding) // go through landing procedure
    {
        heli->mode = USER_DISABLED;
        heli->controller->altitudesetpoint = 5;
        while (heli->controller->curr_altitude_reading > heli->controller->altitudesetpoint) // Check landing altitude has been achieved
        {
            ResetFlag = (GPIOPinRead(SW_PORT, SW2_PIN));
            if (DeltaTFlag)
            {
                DeltaTFlag = 0;
                ControllerImplementation(heli);
                SysTick(heli);
                if (slowTick)
                {
                    DisplayProject(heli);
                }
            }
            if (YawIntFlag)
            {
                YawIntFlag = 0;
                ExecuteYawInt(heli);
            }
            if (ResetFlag != 0)
            {
                SysCtlReset();
            }
            SetPWM(heli->mainrotor);
            SetPWM(heli->tailrotor);
        }
        LocatePivot(heli);
        EnableLanding = false;  // cannot undergo landing procedure when landed.
    }

    StopRotors(heli);           //already landed, so stay in reset state
}

//*****************************************************************************
// ModeTakeoff is only enabled from a 'LANDED' mode state. The helicopter is
// risen to 5% altitude and goes through 'locatepivot' to find the reference
// yaw position. It then rises to 10% altitude and user buttons are enabled
// via 'USER_ENABLED'.
// ModeTakeoff is only enabled from a 'LANDED' mode state. The helicopter is
// risen to 5% altitude and goes through 'locatepivot' to find the reference
// yaw position. It then rises to 10% altitude and user buttons are enabled
// via 'USER_ENABLED'.
//*****************************************************************************
void ModeTakeoff(Helicopter* heli)
{
    heli->submode = TAKEOFF;

    heli->controller->altitudesetpoint = 5;

    while (heli->controller->curr_altitude_reading < heli->controller->altitudesetpoint) // Check takeoff altitude has been achieved
    {
        ResetFlag = (GPIOPinRead(SW_PORT, SW2_PIN));
        if (DeltaTFlag)
        {
            DeltaTFlag = 0;
            ControllerImplementation(heli);
            SysTick(heli);
            if (slowTick)
            {
                DisplayProject(heli);
            }
        }
        if (YawIntFlag)
        {
            YawIntFlag = 0;
            ExecuteYawInt(heli);
        }
        if (ResetFlag != 0)
        {
            SysCtlReset();
        }
        SetPWM(heli->mainrotor);
        SetPWM(heli->tailrotor);
    }
    LocatePivot(heli);
    ModeFly(heli);    //then initiate ModeFly to enable push buttons
    heli->controller->altitudesetpoint = 10;
}

//*****************************************************************************
// In 'flying' submode, the mode is 'USER_ENABLED' and push buttons are
// enabled. Altitude increments in 10% steps (capped between 0 and 100%) and
// yaw in 15degree steps with no limitations on rotation.
//*****************************************************************************
void ModeFly(Helicopter* heli)
{
    heli->mode = USER_ENABLED;  //enable button control from user
    heli->submode = FLY;   // Track user mode as 'FLY'
}

//*****************************************************************************
// Sets the main rotor and tail rotor duty cycles to be 0%. It then uses
// setPWM to transfer these duty's to the periherals.
//*****************************************************************************
void StopRotors(Helicopter* heli)
{
    heli->mainrotor->ui32Duty = 0;
    SetPWM(heli->mainrotor);
    heli->tailrotor->ui32Duty = 0;
    SetPWM(heli->tailrotor);
}
