# STM32-Pacman-Game
# Description
This project implements a simplified Pacman-style game on an STM32 microcontroller board using external interrupts, timers, and GPIO peripherals. The game features:

Joystick control for Pacman movement (up, down, left, right)

LED matrix display for game visuals

Serial communication for game status

ADC for analog input handling

Timer-based game loop

**The code demonstrates:**

External interrupt handling for joystick inputs

Timer interrupts for game timing

USART communication

ADC configuration for analog readings

GPIO manipulation for LED control

# Hardware Requirements
STM32 microcontroller board (specific model used in development)

Joystick module

LED matrix or discrete LEDs

Serial communication interface (USB-UART converter)

# Software Requirements
mikroC PRO for ARM compiler

STM32 HAL libraries

Serial terminal program (Tera Term, PuTTY, etc.)

# Installation & Usage
Clone this repository

Open the project in mikroC PRO for ARM

Connect your STM32 board

Configure the hardware as specified in the pin configuration

Compile and upload the code to your board

Use the joystick to control the game

# Key Features
**Joystick Control:**

Left movement: PE13 and PE14 LEDs on

Up movement: PE11 and PE15 LEDs on

Down movement: PE8 and PE12 LEDs on

Button press: All LEDs on

**Serial Communication:**

Receives 'P' or 'p' to pause the game

Sends game status/counter values

**ADC Input:**

Reads analog values from PC0

Displays values on PORTD LEDs
