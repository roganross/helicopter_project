//*******************************************************************************
// uart.c
//
// This file handles UART communication, enabling communication between the TIVA
// board and an external terminal. It initializes UART settings, sends serial
// data, and prints status information including altitude, yaw angle, main rotor
// duty cycle, and tail rotor duty cycle.
//
// Author:  R.J Ross, H. Donley
//
// Last modified:   17.05.24
//*******************************************************************************

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "yaw.h"
#include "buffer.h"
#include "altitude.h"
#include "system.h"
#include "uart.h"
#include "rotors.h"

//*****************************************************************************
// Global Variables
//*****************************************************************************

static char statusStr[MAX_STR_LEN + 1];    // Serial status string for troubleshooting (UART)

//*****************************************************************************
// Intialises UART, allowing communication between the TIVA board and a terminal.
//*****************************************************************************
void
initialiseUSB_UART (void)
{
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);    // why not use these ports???
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

//*****************************************************************************
// Function to send serial communication from microcontroller to computer.
//*****************************************************************************
void
UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}

//*****************************************************************************
// Function to print the sent serial communication by UARTSend.
//*****************************************************************************
void UARTPrint(Helicopter* heli)
{
    int32_t yawAngle = GetYawAngleDegrees(heli);  // Outputs yaw in degrees

    // Displays all information relating to helicopter altitude, yaw, rotors and mode.
    usprintf(statusStr, "Alt Desired (%%): %3d, Alt Actual (%%): %3d, Yaw Desired (deg): %4d, Yaw Actual (deg): %4d, M-Rot (%%): %2d, T-Rot (%%): %2d, Mode: %d\r\n",
             heli->controller->altitudesetpoint, heli->controller->curr_altitude_reading, heli->controller->yaw_increment,
             yawAngle, heli->mainrotor->ui32Duty, heli->tailrotor->ui32Duty, heli->submode);

    UARTSend(statusStr);
}
