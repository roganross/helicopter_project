*** *ENCE361 Embedded Helicopter Control System* ***

**Summary**

This project aims to develop an embedded system for controlling a remote-controlled helicopter using a TIVA board. The system allows a pilot to perform various tasks such as takeoff, altitude control, yaw adjustment, and landing. The project utilizes a real-time kernel for precise control and communicates status information via UART for monitoring.

**Key Points**

1. Yaw Signal Processing and Altitude Monitoring: Implement interrupt-driven decoding for the yaw signal and display the yaw angle on the OLED display.Sample and convert altitude signal for display as a percentage on the OLED board while applying noise filtering techniques.

2. PWM Signal Generation: Generate PWM signals for main and tail rotor motors, adjusting duty cycles and frequencies within specified ranges, and display them on the OLED board.

3. Mode Control and Operations: Use emulated inputs to control the helicopter's mode, ensuring smooth transitions between takeoff, flight, and landing, with initialization in a landed state upon start/reset.

4. User-Programmable Buttons: Configure buttons for altitude and yaw adjustments, ensuring they operate independently and within defined parameters.

5. Real-time Kernel and Serial Communication: Implement a real-time kernel for foreground/background tasks, ensuring robust behavior under varying conditions. Establish serial communication for transmitting status information, including yaw, altitude, PWM duty cycles, and mode, at regular intervals over 4x per second.

6. PID Controller: Integrate a PID controller to achieve controlled flight, incorporating zero gravity offset for stability under various conditions.

**Licence**

A University of Canterbury project, consult the electrical and computer engineering department (ECE) for terms.

**Authors**

Hunter Donley & Rogan Ross.
