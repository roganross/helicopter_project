//*******************************************************************************
// main.c
//
// This main file manages an embedded system for controlling a helicopter,
// featuring essential flight modes: TAKEOFF, LANDING, RESET, and FLY. It
// coordinates sensors, actuators, and algorithms to ensure safe and efficient
// flight operations.
//
// Author:  R.J Ross, H. Donley
//
// Last modified:   17.05.24
//*******************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "buttons4.h"
#include "yaw.h"
#include "buffer.h"
#include "altitude.h"
#include "system.h"
#include "rotors.h"
#include "mode.h"
#include "uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "kernel.h"

//********************************************************************************
// Main Function of Helicopter. Creates the helicopter struct and initialises all
// peripherals, including but not limited to, Clock, ADC and Buffer.
// Runs the kernel to implement task management.
//********************************************************************************
int
main(void)
{
    Helicopter* heli = NewHeli();
    initHelicopter(heli);
    ChangeMode = 0;

    while(1)
    {

        Run_Kernel(heli);
    }
}
