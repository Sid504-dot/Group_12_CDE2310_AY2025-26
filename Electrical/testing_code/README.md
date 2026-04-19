# JF-0826B Solenoid Testing

Can use the attached script to test the **JF-0826B solenoid**.

## Test Script

- `solenoid_test.py`

## Notes

This script is for basic ON/OFF testing of the solenoid through the Raspberry Pi GPIO pin and MOSFET driver circuit.

Make sure the solenoid is **not connected directly to the Raspberry Pi GPIO pin**. The GPIO pin should only control the MOSFET gate.
