#ifndef ROTORS_H_
#define ROTORS_H_

//*******************************************************************************
// rotors.h
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

#include <stdint.h>
#include <stdbool.h>
#include "circBufT.h"

//*******************************************************************************
// Constants
//*******************************************************************************
// General PWM configuration
#define PWM_DIVIDER        4

// Main Rotor Specific PWM configuration
#define MAIN_PWM_START_RATE_HZ  250
#define MAIN_PWM_START_DUTY     0
#define PWM_MAIN_DUTY_MAX     80
#define PWM_MAIN_DUTY_MIN     20

// Tail Rotor Specific PWM configuration
#define TAIL_PWM_START_RATE_HZ  250
#define TAIL_PWM_START_DUTY    0
#define PWM_TAIL_DUTY_MAX     64
#define PWM_TAIL_DUTY_MIN     16

#define GAIN_DIVIDE_FACTOR     1000
#define GRAVITY_FACTOR         51

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M1PWM5 (gen 2)
//  ---Tail Rotor PWM: PF1, J4-05
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1


/*********************************************************************************
 * Create the main helicopter struct entity: controller, main rotor and tail rotor.
 * This tracks the helicopters altitude and yaw position, with given parameters,
 ********************************************************************************/
typedef struct {
    int32_t prev_altitude_reading;       // self explanatory, real life to the moment values, dependent on proportions of PID control. ACTUAL VALUES!
    int32_t prev_yawangle_reading;
    int32_t prev_yaw_reading;
    int32_t curr_altitude_reading;       // a function, calculatealtitude as a %, of refAltADC. This is called after init_Alt initialises the refAltADC
    int32_t curr_yawangle_reading;
    int32_t yaw_increment;
    int32_t altitudesetpoint;            // set point, + or - 10% increments vertically via updateduty()
    int32_t yawanglesetpoint;            // set point, + or - 15% increments rotationally via updateduty()
    uint8_t altitude_move_up;
    uint8_t altitude_move_down;
} Controller;

typedef struct {
    volatile uint32_t ui32Freq;
    volatile uint32_t ui32Duty;
    uint32_t pwmBase;
    uint32_t pwmGen;
    uint32_t pwmOutNum;
    uint32_t pwmOutBit;
    uint32_t pwmPeriphPWM;
    uint32_t pwmPeriphGPIO;
    uint32_t pwmGPIOConfig;
    uint32_t pwmGPIOBase;
    uint32_t pwmGPIOPin;
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;
    int32_t I;
} Rotor;

typedef struct {
    int32_t meanVal;                   //current altitude, as a % of 100!!!
    int32_t refAltADC;                 //reference altitude ADC value
    circBuf_t* g_inBuffer;
} Buffer;

typedef enum {
    USER_ENABLED = 0,
    USER_DISABLED = 1
} Mode;

typedef enum {
    FLY = 0,
    TAKEOFF = 1,
    LANDED = 2
} SubMode;

typedef struct {
    Controller* controller;
    Rotor* mainrotor;
    Rotor* tailrotor;
    Buffer* buffer;
    Mode mode;
    SubMode submode;
} Helicopter;

/*********************************************************************************
 * Initialise the helicopter pointer with values given. Assume default mode is
 * landed, so initial state is USER_DISABLED and enable_landing = false.
 ********************************************************************************/
Helicopter* NewHeli(void);

/*********************************************************
 * initialisePWM
 * M0PWM7 (J4-05, PC5) is used for the main rotor motor
 * M1PWM5 (J3-10, PF1) is used for the tail rotor motor
 *********************************************************/
void initialisePWM(Rotor *rotor);

/********************************************************
 * Function to set the freq, duty cycle of PWM
 ********************************************************/
void SetPWM(Rotor *rotor);

/********************************************************
 * Function to intialise the main and tail rotors
 ********************************************************/
void initialiseRotors (Helicopter* heli);

/********************************************************
 * adjustHeli polls the buttons UP, DOWN, RIGHT and LEFT
 * to look for increments in altitude and yaw angle set
 * points.
 ********************************************************/
void AdjustHeli(Helicopter* heli);

/********************************************************
 * Digital Implementation of PID Controller for main rotor
 * Calculates P, I and D components for the main.
 ********************************************************/
int32_t MainController (Helicopter* heli);

/********************************************************
 * Computers the control outputs for the main and tail
 * rotors, taking into account coupling and gravity forces.
 * This also checks for saturation of the PWM via large
 * error, and reduces that PWM to stable values. It then
 * sets duty based on the control.
 ********************************************************/
void ControllerImplementation (Helicopter* heli);

/********************************************************
 * Digital Implementation of PID Controller for tail rotor
 * Calculates P, I and D components for the tail.
 ********************************************************/
int32_t TailController (Helicopter* heli);

#endif /* ROTORS_H_ */
