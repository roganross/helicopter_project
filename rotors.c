 //*******************************************************************************
// rotors.c
//
// This file serves as the API for rotor-related functions within the embedded
// system governing the helicopter. It encompasses interfaces for setting PWM
// (Pulse Width Modulation), initializing rotors, updating rotor speeds, and
// implementing PID (Proportional-Integral-Derivative) control for both main
// and tail rotors.
//
// Author:  R.J Ross, H. Donley
//
// Last modified:   17.05.24
//*******************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "buttons4.h"
#include "utils/ustdlib.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "stdlib.h"
#include "rotors.h"
#include "altitude.h"
#include "yaw.h"
#include "buffer.h"
#include "circBufT.h"
#include "system.h"

/*********************************************************************************
 * Create the main helicopter struct entity: controller, main rotor and tail rotor.
 * This tracks the helicopters altitude and yaw position, with given parameters,
 ********************************************************************************/

Helicopter* NewHeli(void)
{
    // Controller struct
    static Controller heli = {
        .prev_altitude_reading = 0,
        .prev_yaw_reading = 0,
        .curr_altitude_reading = 0,
        .curr_yawangle_reading = 0,
        .yaw_increment = 0,
        .altitudesetpoint = 0,
        .yawanglesetpoint = 0,
        .altitude_move_up = true,
        .altitude_move_down = false //assume helicopter initialised at 0% altitude, therefore it cannot go down initially
    };

    // Main rotor struct
    static Rotor mainRotor = {
        .ui32Freq = MAIN_PWM_START_RATE_HZ,
        .ui32Duty = MAIN_PWM_START_DUTY,
        .pwmBase = PWM_MAIN_BASE,
        .pwmGen = PWM_MAIN_GEN,
        .pwmOutNum = PWM_MAIN_OUTNUM,
        .pwmOutBit = PWM_MAIN_OUTBIT,
        .pwmPeriphPWM = PWM_MAIN_PERIPH_PWM,
        .pwmPeriphGPIO = PWM_MAIN_PERIPH_GPIO,
        .pwmGPIOConfig = PWM_MAIN_GPIO_CONFIG,
        .pwmGPIOBase = PWM_MAIN_GPIO_BASE,
        .pwmGPIOPin = PWM_MAIN_GPIO_PIN,
        .Kp = 1500,  //1000
        .Ki = 10,   //2
        .Kd = 250    //250
    };

    // Tail rotor struct
    static Rotor tailRotor = {
        .ui32Freq = TAIL_PWM_START_RATE_HZ,
        .ui32Duty = TAIL_PWM_START_DUTY,
        .pwmBase = PWM_TAIL_BASE,
        .pwmGen = PWM_TAIL_GEN,
        .pwmOutNum = PWM_TAIL_OUTNUM,
        .pwmOutBit = PWM_TAIL_OUTBIT,
        .pwmPeriphPWM = PWM_TAIL_PERIPH_PWM,
        .pwmPeriphGPIO = PWM_TAIL_PERIPH_GPIO,
        .pwmGPIOConfig = PWM_TAIL_GPIO_CONFIG,
        .pwmGPIOBase = PWM_TAIL_GPIO_BASE,
        .pwmGPIOPin = PWM_TAIL_GPIO_PIN,
        .Kp = 290,  //400
        .Ki = 2,  //2
        .Kd = 200   //300
    };

    static Buffer buffer = {
        .meanVal = 0,                     //% of 100 number. Initialise this as a zero, and then call getAltitude(heli) via altitude.c to set its value
        .refAltADC = 0,                   //ADC number. Initialise this as a zero, and then call initAlt via altitude.c to set its value to buffercalculate()
        .g_inBuffer = &g_inBuffer
    };

    static Helicopter Helicopter = {
        .controller = &heli,
        .mainrotor = &mainRotor,
        .tailrotor = &tailRotor,
        .buffer = &buffer,
        .mode = USER_DISABLED, //user buttons ignored initially
        .submode = LANDED //assume beginning in landed state
    };

    return &Helicopter;
}

/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 (J3-10, PF1) is used for the tail rotor motor
 *********************************************************/
void
initialisePWM(Rotor *rotor)
{
    // Main Rotor Related PWM Initialization
    SysCtlPeripheralEnable(rotor->pwmPeriphPWM);
    SysCtlPeripheralEnable(rotor->pwmPeriphGPIO);

    GPIOPinConfigure(rotor->pwmGPIOConfig);
    GPIOPinTypePWM(rotor->pwmGPIOBase, rotor->pwmGPIOPin);

    PWMGenConfigure(rotor->pwmBase, rotor->pwmGen,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    SetPWM(rotor);

    PWMGenEnable(rotor->pwmBase, rotor->pwmGen);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(rotor->pwmBase, rotor->pwmOutBit, false);
}

/********************************************************
 * Function to set the freq, duty cycle of PWM
 ********************************************************/
void
SetPWM(Rotor* rotor)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / rotor->ui32Freq;

    PWMGenPeriodSet(rotor->pwmBase, rotor->pwmGen, ui32Period);
    PWMPulseWidthSet(rotor->pwmBase, rotor->pwmOutNum,
                     ui32Period * rotor->ui32Duty / 100);
}

/********************************************************
 * Function to intialise the main and tail rotors
 ********************************************************/
void
initialiseRotors(Helicopter* heli)
{
    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset(heli->mainrotor->pwmPeriphGPIO); // Used for PWM Main output
    SysCtlPeripheralReset(heli->mainrotor->pwmPeriphPWM);  // Main Rotor PWM
    SysCtlPeripheralReset(heli->tailrotor->pwmPeriphGPIO); // Used for PWM Tail output
    SysCtlPeripheralReset(heli->tailrotor->pwmPeriphPWM);  // Tail Rotor PWM
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO
    SysCtlPeripheralReset (LEFT_BUT_PERIPH);        // LEFT button GPIO
    SysCtlPeripheralReset (RIGHT_BUT_PERIPH);      // RIGHT button GPIO

    initButtons (); // do not delete!!! Need to init buttons before rotor

    initialisePWM(heli->mainrotor);
    initialisePWM(heli->tailrotor);

    PWMOutputState(heli->mainrotor->pwmBase, heli->mainrotor->pwmOutBit, true);
    PWMOutputState(heli->tailrotor->pwmBase, heli->tailrotor->pwmOutBit, true);
}

/********************************************************
 * adjustHeli polls the buttons UP, DOWN, RIGHT and LEFT
 * to look for increments in altitude and yaw angle set
 * points.
 ********************************************************/
void
AdjustHeli(Helicopter *heli)
{
    // Background task: Check for button pushes
    if ((checkButton(UP) == PUSHED) && (heli->controller->altitudesetpoint <= 90)) {
        heli->controller->altitudesetpoint += 10; // Increase altitude setpoint by 10%

    } else if ((checkButton(DOWN) == PUSHED) && (heli->controller->altitudesetpoint >= 10)) {
        heli->controller->altitudesetpoint -= 10; // Decrease altitude setpoint by 10%

    } else if (checkButton(RIGHT) == PUSHED) {
        // Increase Yaw Angle by 15 Degrees (19 states)
        heli->controller->yawanglesetpoint -= (56/3);
        heli->controller->yaw_increment -= 15;

    } else if (checkButton(LEFT) == PUSHED) {
        // Decrease Yaw Angle by 15 Degrees (19 states)
        heli->controller->yawanglesetpoint += (56/3);
        heli->controller->yaw_increment += 15;
    }
    heli->controller->yawanglesetpoint = heli->controller->yawanglesetpoint % 448;
    heli->controller->yaw_increment = (((heli->controller->yaw_increment + 540) % 360) - 180);
    SetPWM(heli->mainrotor);
    SetPWM(heli->tailrotor);
}

/********************************************************
 * Digital Implementation of PID Controller for main rotor
 * Calculates P, I and D components for the main.
 ********************************************************/
int32_t
main_controller (Helicopter *heli)
{
    int32_t error = heli->controller->altitudesetpoint - heli->controller->curr_altitude_reading;   // error
    int32_t P = (heli->mainrotor->Kp * error);            // Proportional
    int32_t dI = heli->mainrotor->Ki * error / 100;          // Integral
    int32_t D = (heli->mainrotor->Kd * (heli->controller->prev_altitude_reading - heli->controller->curr_altitude_reading) * 100); // Derivative

    int32_t control = (P + (heli->mainrotor->I + dI) + D) / GAIN_DIVIDE_FACTOR; //leave divide by gain factor until after addition to prevent zero round error for small integral gain accumulation.


    heli->mainrotor->I = (heli->mainrotor->I + dI);  // Saves previous yaw angle for next error calculation
    heli->controller->prev_altitude_reading = heli->controller->curr_altitude_reading; // Saves previous yaw angle for next error calculation

    return (control);   // Add gravity and coupling offsets
}

/********************************************************
 * Digital Implementation of PID Controller for tail rotor
 * Calculates P, I and D components for the tail.
 ********************************************************/
int32_t
tail_controller (Helicopter *heli)
{
    int32_t error = (heli->controller->yawanglesetpoint) - heli->controller->curr_yawangle_reading;   // error
    int32_t P = heli->tailrotor->Kp * error;            // Proportional
    int32_t dI = heli->tailrotor->Ki * error / 100;          // Integral
    int32_t D = heli->tailrotor->Kd * (heli->controller->prev_yawangle_reading - heli->controller->curr_yawangle_reading) * 100;   // Derivative

    int32_t control = (P + (heli->tailrotor->I + dI) + D) / GAIN_DIVIDE_FACTOR; //leave divide by gain factor until after addition to prevent zero round error for small integral gain accumulation.

    heli->tailrotor->I = (heli->tailrotor->I + dI);
    heli->controller->prev_yawangle_reading = heli->controller->curr_yawangle_reading; // Saves previous yaw angle for next error calculation

    return (control);   // Add gravity and coupling offsets
}

/********************************************************
 * Computes the control outputs for the main and tail
 * rotors, taking into account coupling and gravity forces.
 * This also checks for saturation of the PWM via large
 * error, and reduces that PWM to stable values. It then
 * sets duty based on the control.
 ********************************************************/
void
ControllerImplementation (Helicopter* heli)
{
    CalculateAltitude(heli); // Updates current altitude value

    // Calculate control output using PID controller
    int32_t main_controlOutput = main_controller(heli) + GRAVITY_FACTOR;  // Produce a control output for main rotor
    int32_t tail_controlOutput = tail_controller(heli) + ((8 * main_controlOutput) / 10);  // Produce a control output for tail rotor

    // Apply saturation limits to the new duty cycle for main rotor
    if (main_controlOutput > PWM_MAIN_DUTY_MAX) {
        main_controlOutput = PWM_MAIN_DUTY_MAX;
    } else if (main_controlOutput < PWM_MAIN_DUTY_MIN) {
        main_controlOutput = PWM_MAIN_DUTY_MIN;
    }

    // Apply saturation limits to the new duty cycle for tail rotor
    if (tail_controlOutput > PWM_TAIL_DUTY_MAX) {
        tail_controlOutput = PWM_TAIL_DUTY_MAX;
    } else if (tail_controlOutput < PWM_TAIL_DUTY_MIN) {
        tail_controlOutput = PWM_TAIL_DUTY_MIN;
    }

    // Apply control output to adjust duty cycle and doesn't exceed caps
    heli->mainrotor->ui32Duty = main_controlOutput; // Adjust the duty cycle of main rotor based on control output
    heli->tailrotor->ui32Duty = tail_controlOutput; // Adjust the duty cycle of tail rotor based on control output

}











