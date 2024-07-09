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

//*****************************************************************************
// The kernel runs the main helicopter embedded system. When a change mode is
// triggered the Kernel calls a respective Mode function where user
// inputs are enabled, halting the kernel. Upon returning to normal 'FLY'
// submode the kernels normal operations continue.
//*****************************************************************************
void
Run_Kernel(Helicopter* heli)
{
    while (1)
    {
        ResetFlag = (GPIOPinRead(SW_PORT, SW2_PIN));  // Flag to track system reset switch
        if (heli->mode == USER_ENABLED)
        {
            AdjustHeli(heli);  // Allows user to interact with helicopter via buttons.
        }

        if (DeltaTFlag)  // Systick determined flag, relating to time change (delta T) of controller.
        {
            DeltaTFlag = 0;
            ControllerImplementation(heli);
            SysTick(heli);
            if (slowTick)  // Slowtick dictates display update frequency
            {
                DisplayProject(heli);
            }
        }

        if (ChangeMode)   // Mode Change detected
        {
            ChangeMode = 0;
            SysCtlDelay(300);   // Debouncing timer for switch so correct state is Read
            ExecuteHelicopterMode(heli);
        }

        if (YawIntFlag)   // Yaw Change detected
        {
            YawIntFlag = 0;
            ExecuteYawInt(heli);
        }

        if (ResetFlag != 0)
        {
            SysCtlReset();
        }
    }
}
