#ifndef UART_H_
#define UART_H_

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

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// Constants
//*****************************************************************************
#define MAX_STR_LEN 134    // Maximum String Length for serial output

#define NUM_SLOTS       112
#define TOTAL_DEG       360
#define TOTAL_STATES    NUM_SLOTS * 4
#define YAW_DELTA       TOTAL_DEG / (TOTAL_STATES)

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

//*****************************************************************************
// Intialises UART, allowing communication between the TIVA board and a terminal.
void initialiseUSB_UART (void);

//*****************************************************************************
// Function to send serial communication from microcontroller to computer.
void UARTSend (char *pucBuffer);

//*****************************************************************************
// Function to print the sent serial communication by UARTSend.
void UARTPrint(Helicopter* heli);

#endif /* UART_H_ */
