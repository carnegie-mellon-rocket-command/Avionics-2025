# Avionics 2025
Flight control software for the 2025 ATS.

## Overview
The program controls an ATS that increases surface area using PID to adjusts drag in based on real time measurements to achieve a target altitude. It uses:
- IMU/Accelerometer (LSM6DSOX, 9, SPI)
- Altimeter (BMP390, 8, SPI)
- Thermometer (LSM6DSOX (built in to IMU), 9, SPI)
- ATS Servo (???, 6, Arduino Servo library)

## LED blinking meaning
- **Rapid Flashing (4 quick blinks)**: Initialization successful (LEDSuccess)
- **Solid ON**: Rocket is flying and logging data (LEDFlying/LEDLogging)
- **Slow Blinking (1s intervals)**: SD card or sensor initialization failed (LEDError)
- **Initial ON**: Setup in progress (LEDSetup)

## Installation

### Automatic Installation (Recommended)
Use the Arduino IDE with the provided build profile (sketch.yaml)

### Manual Installation
Board: Teensy 4.1
Required Libraries:
- Servo v1.1.2 (version specific)
- SD v1.3
- Adafruit BMP3XX v2.1.6
- Adafruit LSM6DS
- BasicLinearAlgebra v3.7 (version specific)
- Kalman v1.1.0

## Configuration
- When launching, set SIMULATE to false and when testing set SIMULATE to TRUE
- Set DEBUG to true for verbose output


## Program Flow

### 1. Setup Phase
1. Initialize SD card for data logging
2. Setup sensors  
3. Check sensors work
4. Test ATS servo 
5. Initialize Kalman filter matrices

### 2. Flight Control Loop
1. Sensor Data Collection
   - Update altimeter readings
   - Collect IMU data
   - Filter and store measurements
2. Adjust ATS
   - Calculate required drag area using:
     $$ A = \frac{m(\frac{v^2}{h_{target} - h} - 2g)}{v\rho C_d} $$
      Where:
      - A (target_area): ft²
      - m (ROCKET_MASS): lbs
      - v (gVelocityFiltered): ft/s
      - h_target (ALT_TARGET): ft
      - h (gAltFiltered): ft
      - g (GRAVITY): 32.174 ft/s²
      - ρ (ATMOSPHERE_FLUID_DENSITY): 0.076474 lbs/ft³
      - C_d (ROCKET_DRAG_COEFFICIENT): 0.46

   - Use PID control to reduce acceleration based on error
   - Set Servo to new position

### Data logged (per entry)
1. **Time Data**
   - Time since boot (milliseconds)
2. **Raw Sensor Readings**
   - Pressure (hPa)
   - Raw altitude (ft)
   - Raw acceleration (all axes) (ft/s²) 
   - Gyroscope readings (all axes) (radians/s)
3. **Filtered Data**
   - Filtered altitude (ft)
   - Filtered velocity (ft/s)
   - Filtered acceleration (ft/s²)
4. **Environmental Data**
   - Temperature from IMU (degrees C)
5. **Control Data**
   - ATS position (degrees)
