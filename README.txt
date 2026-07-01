SVPWM Closedloop Torque Control H743
======================================

High-Level Architecture
---------------------

Main Entry Point
---------------
Core/Src/main.c
- System initialization (HAL drivers, peripherals)
- Main control loop (TIM1 interrupt at 23 kHz)
- Motor parameter configuration
- FOC control implementation
- SD card logging integration

Core Functional Modules
----------------------

1. FOC Control System
   Core/Src/FOC.c - Field Oriented Control algorithms
   Core/Src/foc_loop.c - FOC control loop implementation (23 kHz ISR)
   Core/Src/motor_control.c - SVPWM modulation, Park/Clarke transforms
   Core/Src/foc_utils.c - Fast sin/cos/atan2 lookup tables

2. Motor Control
   Core/Src/pid.c - Discrete PID controllers (Id, Iq, speed)
   Core/Src/current_sense.c - ADC current sensing & zero-point calibration
   Core/Src/sincos_encoder.c - Sin/Cos encoder angle decoding
   Core/Inc/PLL/ - Phase Locked Loop for speed estimation

3. Communication
   Core/Src/can_app.c - CAN bus communication (FDCAN)
   Core/Inc/canid.h - CAN message ID definitions
   Core/Src/usart.c - UART communication

4. State Management
   Core/Src/inverter_state.c - Inverter state machine (INIT→READY→RUNNING→ERROR)
   Core/Src/hw_callbacks.c - Hardware interrupt callbacks (gate fault, overcurrent)

5. Data Logging
   Core/Src/logger.c - SD card logging (FatFs, double-buffer, 23 kHz sampling)
   Middlewares/Third_Party/FatFs/ - FatFs file system

6. Signal Processing
   Core/Src/lowpass_filter.c - First-order IIR low-pass filter
   Core/Inc/PLL/ - MATLAB-generated PLL observer

Configuration
-------------
Core/Inc/config.h - All tunable parameters (PID gains, protection thresholds, PWM frequency)
Core/Inc/foc_loop.h - FOC state structures (motor_params_t, foc_state_t)

Hardware Abstraction Layer
--------------------------
Core/Src/adc.c - ADC configuration & DMA
Core/Src/tim.c - Timer configuration (PWM generation)
Core/Src/fdcan.c - FDCAN configuration
Core/Src/sdmmc.c - SD card interface
Core/Src/gpio.c - GPIO configuration
Core/Src/dma.c - DMA configuration
Core/Src/i2c.c - I2C interface
Core/Src/rtc.c - Real-time clock

HAL Drivers
-----------
Drivers/STM32H7xx_HAL_Driver/ - STM32H7 HAL drivers
Drivers/CMSIS/ - ARM CMSIS & DSP library

Control Flow
-----------
1. System initializes in main()
2. TIM1 interrupt triggers at 23 kHz → foc_loop.c
3. FOC loop executes:
   - Read current sensors (ADC)
   - Read encoder angle
   - Clarke/Park transforms
   - PID control (Id, Iq)
   - Inverse Park transform
   - SVPWM modulation
   - Update PWM duty cycles
4. CAN communication runs in parallel
5. SD card logging writes data asynchronously

Key Features
-----------
- 23 kHz control loop frequency
- SVPWM modulation for efficient motor control
- Field weakening control (optional)
- CAN bus torque command interface
- SD card high-speed data logging
- Multi-motor support (4 independent controllers)

Build Instructions
-----------------

Prerequisites
-------------
1. ARM GCC toolchain (arm-none-eabi-gcc) - must be in PATH
2. CMake (version 3.22 or higher)
3. Ninja build system

Recommended Installation Options
--------------------------------
Option 1: Install STM32CubeIDE (includes all required tools)
- Download from ST website: https://www.st.com/en/development-tools/stm32cubeide.html
- ARM GCC, CMake, and Ninja are included

Option 2: Install individual tools
- ARM GCC: https://developer.arm.com/downloads/-/gnu-rm
- CMake: https://cmake.org/download/
- Ninja: https://github.com/ninja-build/ninja/releases

Build Instructions
-------------------
1. Navigate to project directory:
   cd \SVPWM_Closedloop_torque_control_H743

2. Configure CMake with Debug preset:
   cmake --preset Debug

3. Build the project:
   cmake --build --preset Debug

4. For Release build:
   cmake --preset Release
   cmake --build --preset Release

5. Clean and rebuild:
   cmake --build --preset Debug --clean-first

Output Files
------------
Debug build: build/Debug/SVPWM_Closedloop_torque_control_H743.elf
Release build: build/Release/SVPWM_Closedloop_torque_control_H743.elf

Alternative: Using Full Toolchain Paths
---------------------------------------
If ARM GCC is not in PATH, use full paths:
$env:ARM_GCC_PATH = "C:\path\to\arm-none-eabi-gcc\bin"
cmake --preset Debug
cmake --build --preset Debug

Troubleshooting
---------------
- If "arm-none-eabi-gcc not found": Add ARM GCC to system PATH
- If CMake not found: Add CMake to system PATH or use full path
- If build fails: Check that all required source files are present

Project Structure
-----------------
- CMakeLists.txt - Main CMake configuration
- CMakePresets.json - Build presets (Debug/Release)
- cmake/gcc-arm-none-eabi.cmake - Toolchain configuration
- SVPWM_Closedloop_torque_control_H743.ioc - STM32CubeMX project file
