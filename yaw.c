//*******************************************************************************
// yaw.c
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "yaw.h"

//*****************************************************************************
// A table is used to adjust the yaw angle (yawAngle) of the helicopter. The
// quadrature decoder increments/decrements in Gray Code and reads two bits. If
// the previous state (prevRead) of the current quadrature decoder read
// (currentRead) is 'less', then the decoder will decrement (as this represents
// counter-clockwise rotation); vice versa, if the previous state is 'greater',
// it results in an increment for clockwise rotation.
//*****************************************************************************
static const int8_t adjust_table[] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1,
                              1, 0};  // Table that is indexed to increment/decrement Yaw (Refer to README) (Constant).

//Interrupt flags for yaw related function
volatile uint8_t YawIntFlag = false;
volatile uint8_t YawRefFlag = 0;

//*****************************************************************************
// Manages the interrupt handler for the reference yaw, triggers YawRefFlag
// when the central position is found.
//*****************************************************************************
void YawRefIntHandler(void)
{
    YawRefFlag = 1;
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
}

//*****************************************************************************
// Function to initialize yaw reference peripheral pin PC4
//*****************************************************************************
void
initRefYaw(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_STRENGTH_6MA, GPIO_PIN_TYPE_STD_WPD);
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_DIR_MODE_IN);
    GPIOIntDisable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_BOTH_EDGES);

    GPIOIntRegister(GPIO_PORTC_BASE, YawRefIntHandler);

    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
}

//*****************************************************************************
// Reads Quadrecture Decoder output & converts to a 2-bit number and concatenates
// them.
//*****************************************************************************
int32_t
ReadQuadrectureDecoder (void)
{
   int32_t channelA = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0);
   int32_t channelB = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1);
   return channelA | channelB;
}

//*****************************************************************************
// Function to initialize yaw peripheral pins PB0 and PB1.
//*****************************************************************************
void
initYawPeripherals(Helicopter* heli)
{
    //Configures input pins PB0 and PB1 to be used for yaw quadrature decoding.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_DIR_MODE_IN);
    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_0);

    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPD);
    GPIODirModeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_DIR_MODE_IN);
    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_1);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES); // look for change in either pin 0 or pin 1 to register an interrupt

    GPIOIntRegister(GPIO_PORTB_BASE, YawIntHandler);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_1);
}

//*****************************************************************************
// Interrupt Handler for Yaw Input Signals. Reads Quadrecture Decoder and
// adjusts Yaw Angle accordingly using table (WRITE ABOUT TABLE IN DOCUMENTATION).
//*****************************************************************************
void
YawIntHandler(void)
{
    YawIntFlag = 1;
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_1);
}

//*****************************************************************************
// See 'adjust_table' note above. This function reads the quadrature decoder
// output and changes our current and previous yaw readings from the result.
//*****************************************************************************
void
ExecuteYawInt(Helicopter* heli)
{
    int32_t currentRead = ReadQuadrectureDecoder();
    int32_t select = (currentRead << 2 | heli->controller->prev_yaw_reading);
    heli->controller->curr_yawangle_reading += adjust_table[select];
    heli->controller->curr_yawangle_reading = heli->controller->curr_yawangle_reading % 448;
    heli->controller->prev_yaw_reading = currentRead;
}

//*****************************************************************************
// Gets the actual Yaw Angle Value for rotors.c
//*****************************************************************************
int16_t
GetYawAngleDegrees(Helicopter* heli)
{
    return (((heli->controller->curr_yawangle_reading + 672) % 448 - 224) * YAW_DELTA);
}
