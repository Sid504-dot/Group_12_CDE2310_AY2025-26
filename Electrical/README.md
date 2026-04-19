# Electrical Documentation

## Components Used

- TurtleBot components
- JF-0826B solenoid
- IRFZ44N N-channel MOSFET
- Separate 12.6 V, 1800 mAh battery/power source for solenoid
- Flyback diode for solenoid protection
- Raspberry Pi GPIO control signal

## System Overview

The solenoid firing system uses the Raspberry Pi GPIO pin as a low-current control signal. The GPIO pin does not power the solenoid directly. Instead, the GPIO pin drives the gate of the IRFZ44N N-channel MOSFET, which switches the higher-current solenoid circuit.

A separate 12.6 V, 1800 mAh battery/power source is used for the JF-0826B solenoid. This isolates the solenoid's high-current switching load from the TurtleBot/OpenCR power rail, reducing the risk of voltage sag, brownout, or unstable behaviour in the Raspberry Pi and OpenCR.

## Linked Documentation

### Wiring Diagram and System Architecture

[Wiring Diagram and System Architecture](./Electrical%20diagram%20and%20Electronics%20system%20architecture.pdf)

### Power Calculation

[Power Calculation](./Power_calculations_document.pdf)

### Component Testing Scripts

[Component Testing](./testing_code)
