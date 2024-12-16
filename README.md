# Smart Lighting System

## Overview
This project implements a **Smart Lighting System** using a **PIC18F4620 microcontroller**, an **LDR (Light-Dependent Resistor)** sensor, and a **PIR (Passive Infrared) sensor**. The system intelligently controls an LED to simulate lighting automation based on ambient light conditions and motion detection. It is programmed in **PIC Assembly Language**.

---

## Features
1. **Light Control with LDR:**
   - Monitors ambient light levels using an LDR.
   - Automatically turns the LED on in low-light conditions (night) and off in bright conditions (day).
2. **Motion Detection with PIR:**
   - Detects motion using a PIR sensor.
   - Temporarily turns on the LED for 15 seconds if motion is detected, regardless of ambient light.
3. **Timer-Based Delay:**
   - Configures **Timer0** to generate accurate delays for timed LED control.
4. **Debugging Support:**
   - Configures the EUSART module for potential debugging via serial communication.
5. **Interrupt Handling:**
   - High-priority interrupts handle Timer0 and PIR motion detection events.

---

## Hardware Requirements
- **Microcontroller:** PIC18F4620
- **Sensors:**
  - LDR sensor (connected to the ADC pin `AN0`)
  - PIR sensor (connected to external interrupt pin `INT1`)
- **Output Device:**
  - LED (connected to `RC0`)
- **Power Supply:** 5V regulated power source
- **Additional Components:**
  - Resistors for LDR voltage divider.
  - Capacitors for noise filtering.
  - Breadboard and connecting wires.

---

## Software Requirements
- **Assembler:** MPLAB X IDE with PIC-AS assembler.
- **Simulation:** Proteus for circuit simulation (optional).

---

## System Architecture
1. **LDR Threshold Logic:**
   - The ADC reads the voltage from the LDR.
   - If the ADC value is below the configured threshold, the system considers it "night" and turns on the LED.
2. **PIR Motion Detection:**
   - If motion is detected, the PIR triggers an external interrupt.
   - The interrupt service routine (ISR) turns on the LED for 15 seconds, regardless of ambient light conditions.
3. **Timer0 Configuration:**
   - Timer0 generates precise delays for system timing.
4. **Interrupt Priority:**
   - PIR interrupts have higher priority than Timer0 to ensure immediate response to motion.

---

## Pin Configuration
| Pin | Function              | Description                            |
|-----|-----------------------|----------------------------------------|
| RA0 | LDR Sensor (AN0)      | Reads ambient light levels via ADC.    |
| RB0 | PIR Sensor (INT1)     | Detects motion using external interrupt. |
| RC0 | LED                   | Controls the LED based on conditions. |
| OSC | Internal Oscillator   | Configured to 8 MHz.                  |

---

## Project Workflow
1. **Initialization:**
   - Configures the internal oscillator to 8 MHz.
   - Initializes ADC, Timer0, LED, and interrupt settings.
2. **Main Loop:**
   - Continuously monitors ambient light levels using ADC.
   - Updates the LED state based on the LDR sensor and PIR input.
3. **Interrupts:**
   - **PIR (INT1):**
     - Activates LED for 15 seconds when motion is detected.
   - **Timer0:**
     - Generates the delay required for the PIR timer.

---

## Configuration Bits
- Internal Oscillator: `INTIO67`
- Fail-Safe Clock Monitor: Disabled
- Watchdog Timer: Disabled
- Low-Voltage Programming: Disabled
- Brown-out Reset: Disabled

---

## Assembly Code Structure
1. **Main Code:**
   - System initialization and main loop.
2. **Subroutines:**
   - ADC configuration and reading.
   - Timer0 configuration and delay generation.
   - LED control logic.
3. **Interrupt Service Routines:**
   - Handles PIR motion detection and Timer0 delays.

---

## Code Highlights
### ADC Initialization
```asm
; Configure ADC to read from AN0
BCF ADCON0, 2 ; Set the ADC channel to AN0
BSF ADCON0, 0 ; Enable ADC
BCF ADCON1, 0 ; Configure AN0 as analog input
```

### LED Control
```asm
; Turn LED on/off based on LDR threshold
BTFSS threshold_flag, 0
BCF LATC, 0 ; Turn off LED (day)
BTFSC threshold_flag, 0
BSF LATC, 0 ; Turn on LED (night)
```

### PIR Interrupt
```asm
; ISR for PIR motion detection
INT1_ISR:
    BCF INTCON3, 0 ; Clear INT1 interrupt flag
    BSF LATC, 0     ; Turn on LED
    MOVLW 0x0Fh     ; Start 15s delay
    MOVWF delay_15s
    CALL start_timer0
```

---

## Simulation and Testing
1. **Proteus Simulation:**
   - Simulate the circuit to verify LED behavior with varying light and motion conditions.
2. **Debugging:**
   - Use EUSART to monitor ADC readings and threshold values.

---

## Future Enhancements
- Add more sensors for environmental monitoring.
- Integrate PWM to adjust LED brightness dynamically.
- Implement a real-time clock (RTC) for scheduled operations.
---
---
## Project Schematic
![Proteus_Design](https://github.com/user-attachments/assets/f6c96a30-7004-4260-bb4b-5d644071c310)
---

## Authors
- Mohamed Olwi
- Sama Mohamed
