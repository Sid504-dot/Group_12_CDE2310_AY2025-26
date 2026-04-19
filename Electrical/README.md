# Electrical Documentation

## Components Used
* **TurtleBot Core:** Raspberry Pi, OpenCR, LiDAR, RPi Camera (Powered by 3S 11.1V LiPo)
* **Actuator:** JF-0826B Solenoid
* **Isolated Power:** DC-168 12.6V, 1800 mAh battery (Dedicated to solenoid)
* **Switching Circuit:** IRFZ44N N-channel MOSFET, 1N4001 Flyback Diode, 330 Ω Gate Resistor, 10k Ω Pull-down Resistor

## System Overview
The solenoid firing system utilizes a dual-battery architecture. A dedicated 12.6V power source is used exclusively for the high-current JF-0826B solenoid load (~2A peak). This completely isolates switching transients from the main TurtleBot/OpenCR power rail, eliminating the risk of voltage sag, brownouts, or unstable behavior in the control electronics.

Because the Raspberry Pi operates on 3.3V logic and cannot drive the 12V solenoid directly, **GPIO 21 (BCM)** is used as a low-current control signal. This signal drives the gate of the IRFZ44N MOSFET, which acts as a switch for the heavier solenoid circuit.

### Circuit Protection Mechanisms
* **Flyback Diode (1N4001):** Prevents high-voltage inductive spikes from destroying the MOSFET when the solenoid de-energizes.
* **Pull-down Resistor (10k Ω):** Ensures the MOSFET remains firmly in the "OFF" state during Raspberry Pi boot-up or if the GPIO pin enters a floating state.

**CRITICAL HARDWARE WARNING: Pulsed Operation Only**
> The JF-0826B solenoid is **not designed for continuous operation**. It draws up to ~25W and will reach its maximum current cap within ~15 seconds of sustained use, leading to severe overheating. 
> 
> **Software Integration Note:** Any ROS 2 node or script controlling GPIO 21 MUST implement short firing pulses and software timeouts to guarantee the pin does not remain `HIGH` indefinitely.

---

## Linked Documentation

### [Wiring Diagram and System Architecture](./Electrical_diagram_and_Electronics_system_architecture.pdf)
Contains the exact schematic detailing the connections between the LiPo batteries, the Raspberry Pi standard interfaces (USB/CSI), OpenCR, and the custom MOSFET driver circuit.

### [Power Calculation](./Power_calculations_document.pdf)
Details the expected power draw of the robot's states (boot, standby, teleop) and mathematically proves that the dedicated 1800 mAh solenoid battery provides more than enough energy margin for a 25-minute mission under pulsed conditions.

### [Component Testing Scripts](./testing_code)
Contains `solenoid_test.py`, a basic ON/OFF hardware validation script using the `RPi.GPIO` library to safely verify the MOSFET switching logic before full integration into the ROS 2 Humble navigation stack.
