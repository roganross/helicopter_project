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

#include <buttons4.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "yaw.h"
#include "buffer.h"
#include "altitude.h"
#include "system.h"
#include "uart.h"
#include "rotors.h"
#include "mode.h"

//Interrupt flags for the helicopter system
volatile uint8_t DeltaTFlag = 0;
volatile uint8_t slowTick = 0;
volatile uint8_t ResetFlag = 0;

//*****************************************************************************
// The interrupt handler for the for SysTick interrupt.
//*****************************************************************************
void
SysTickIntHandler(void)
{
    DeltaTFlag = 1;
}

//*****************************************************************************
// System Tick runs at 150Hz. Within this function, a new ADC value is
// processed each call, button updates are polled for and slow tick is used
// to manage UART printing.
//*****************************************************************************
void
SysTick(Helicopter* heli)
{
    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);

    static uint8_t tickCount = 0;  // Stores current tick count.
    const uint8_t ticksPerSlow = SYSTICK_RATE_HZ / SLOWTICK_RATE_HZ; //set the UART print rate.

    updateButtons ();       // Poll the buttons
    if (++tickCount >= ticksPerSlow)
    {                       // Signal a slow tick
        tickCount = 0;
        slowTick = 1;
        UARTPrint(heli);   // Print current information of helicopter
    }
}

//***************************************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display, UART & Helicopter Interrupts
//***************************************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();

}

//*****************************************************************************
// Function to initialize all parts of helicopter.
//*****************************************************************************
void
initHelicopter(Helicopter* heli)
{
    initClock ();
    initADC ();
    initBuffer();
    initialiseUSB_UART ();
    initYawPeripherals (heli);
    initialiseRotors (heli);
    OLEDInitialise ();
    initSWS();
    initRefYaw();

    // Enable interrupts to the processor.
    IntMasterEnable();

    initAlt(heli); //inits the refAltADC;

}


//*****************************************************************************
// Function to show screen state, modified for Project
// Displays altitude (%), yaw angle (degrees), main rotor duty cycle (%)
// and tail rotor duty cycle (%).
//*****************************************************************************
void
DisplayProject(Helicopter* heli)
{
    char string[17];  // 16 characters across the display

    int16_t yawAngle = GetYawAngleDegrees(heli);  // Outputs Yaw in degrees.

    // Display Altitude (%)
    usnprintf(string, sizeof(string), "Alt (%%): %4d", heli->controller->curr_altitude_reading);
    OLEDStringDraw(string, 0, 0);

    // Display Yaw Angle (Degrees)
    usnprintf(string, sizeof(string), "Yaw (deg): %4d", yawAngle);
    OLEDStringDraw(string, 0, 1);

    // Display Main Rotor Duty Cycle (%)
    usnprintf(string, sizeof(string), "M-Rot (%%): %4d", heli->mainrotor->ui32Duty);
    OLEDStringDraw(string, 0, 2);

    // Display Tail Rotor Duty Cycle (%)
    usnprintf(string, sizeof(string), "T-Rot (%%): %4d", heli->tailrotor->ui32Duty);
    OLEDStringDraw(string, 0, 3);
}
