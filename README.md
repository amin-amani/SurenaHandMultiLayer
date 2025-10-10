<a href="https://scan.coverity.com/projects/amin-amani-surenahandmultilayer">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/32261/badge.svg"/>
</a>

<a href="https://codescene.io/projects/72155"><img src="https://codescene.io/projects/72155/status-badges/hotspot-code-health" alt="Hotspot Code Health"></a>

# SurenaHandMultiLayer

This project provides firmware to control a robotic hand with multiple fingers. It integrates motor control, sensor feedback, and communication over CAN bus, enabling precise manipulation and customization for each finger.

## Features

6 Motor Control: Independently control each finger using dedicated motor drivers.

## Sensor Feedback:

BMP280 Sensors: Measure pressure/altitude data for feedback.

Potentiometers: Provide real-time position feedback of fingers.

CAN Bus Communication: Accepts commands via CAN for integration with other controllers or systems.

Finger Customization: Assign and configure individual PID values for each finger.

Extra Servo Output: Additional output for auxiliary mechanisms or accessories.

## Hardware Requirements

Robotic hand with 6 actuated fingers

BMP280 sensors for pressure/position measurement

Potentiometers on each finger for position feedback

Microcontroller with CAN interface

Motor drivers compatible with chosen motors

Optional servo for additional output


## Build Instructions:

use STM32CubeIDE

Flash Firmware:

st-flash write <firmware.bin> 0x8000000


## Usage

Send commands via CAN to control finger positions.

Monitor sensor feedback (BMP280 and potentiometers) for precise control.

Configure each finger using the IDE controller parameters.

Utilize the extra servo output for additional functionality.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request with improvements, bug fixes, or new features.

## License

GPL
