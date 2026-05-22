# Electrical Reflection

## 1. Wire Management and Electrical Reliability

One major area that could have been improved was wire management. During testing, several electrical issues were harder to diagnose because the wiring was not always clean, secured, or easy to trace. Loose jumper wires, long wire runs, and unclear connections made debugging slower, especially when checking whether the solenoid, MOSFET, battery, and Raspberry Pi grounds were properly connected.

This showed that wire management is not just about neatness. It directly affects debugging speed and system reliability.

<!-- Suggested image: labelled photo of final wiring layout -->

---

## 2. Power Budgeting and Visibility

The biggest electrical lesson was that power sources need to be managed much more carefully. The JF-0826B solenoid required around 12 V and drew high current during activation. During testing, the solenoid worked reliably when the separate battery was fully charged, but performance dropped when the voltage fell from around 12 V to about 11.7 V, and it eventually stopped working close to 11 V.

This showed that our original understanding of the power system was not visible enough. We knew the rated voltage, but we did not initially track how voltage sag, battery discharge, current draw, and wiring resistance affected the solenoid in practice. Because of this, time was spent debugging the solenoid power issue, which reduced the time available for other system-level issues such as navigation.

A better approach would have been to build a clearer power budget earlier, including the expected current draw of the solenoid, the battery voltage range, and possible voltage drops across wires and connections. We should also have measured the voltage at the solenoid terminals during firing, not just at the battery. This would have shown whether the solenoid was actually receiving enough voltage under load.

<!-- Suggested image: power architecture diagram -->

---

## 3. Decision to Use a Separate Solenoid Battery

We used a separate 12.6 V, 1800 mAh battery for the solenoid instead of drawing power from the TurtleBot/OpenCR power rail. This reduced the risk of the solenoid causing voltage sag or brownout on the Raspberry Pi and OpenCR during firing.

In theory, the solenoid could possibly have been powered from the main robot battery if the current path was properly designed and protected. However, at the time, using a separate battery was the safer decision because we were unsure whether the main battery and OpenCR power path could handle the additional solenoid load reliably. We also wanted to avoid adding extra load to the main robot battery, since a drop in robot power could affect both motion and control electronics.

The trade-off was that the second battery introduced its own failure mode. When its voltage dropped too low, the solenoid became unreliable. This means the separate power source solved one problem, but created another requirement: the solenoid battery had to be checked and kept sufficiently charged before each run.

The final design was still an improvement because it allowed the robot to complete the mission at both stations. However, future teams should include a battery voltage check before operation and define a minimum safe voltage for solenoid firing.

<!-- Suggested image: battery voltage table or photo of separate battery setup -->

---

## 4. Wire Gauge and Voltage Drop

Another improvement was replacing thin jumper wires in high-current paths with better AWG wires. Initially, some parts of the power path used thin Dupont jumper wires, which are not suitable for carrying around 1 A to 2 A reliably. These wires can introduce extra resistance, causing voltage drop and heating.

This was important because the solenoid is current-hungry. Even if the battery voltage is correct, the solenoid may still fail if the wiring between the battery and solenoid cannot deliver enough current. After using thicker wires for the high-current path, the power delivery became more reliable.

The key lesson was that signal wires and power wires should not be treated the same. GPIO control wires can be thin because they carry very little current. Solenoid power wires need to be thicker because they carry the actual actuator current.

<!-- Suggested image: thin jumper wire vs thicker AWG wire comparison -->

---

## 5. MOSFET Understanding

This project also improved my understanding of MOSFET switching. Initially, the solenoid circuit was treated mainly as a wiring task. Over time, it became clear that the MOSFET was the core switching component, and choosing the correct MOSFET mattered.

The Raspberry Pi GPIO cannot power the solenoid directly. It only provides a small control signal to the MOSFET gate. The MOSFET then switches the larger current path from the solenoid battery through the solenoid. Understanding the difference between the low-current control side and the high-current load side made the circuit easier to debug.

A better future design should use a MOSFET that is clearly suitable for 3.3 V gate drive from the Raspberry Pi. The IRFZ44N is an N-channel MOSFET, but it is not ideal as a logic-level MOSFET for 3.3 V control. A logic-level MOSFET with low RDS(on) at 3.3 V gate voltage would be a stronger choice.

<!-- Suggested image: MOSFET switching diagram -->

---

## 6. Breadboard vs Perfboard Decision

We used a breadboard instead of directly soldering the circuit onto perfboard because the design was still changing during testing. This was the right choice for the debugging phase. Several issues were found late, including unsuitable wire thickness, power source problems, and MOSFET selection concerns. If the circuit had been soldered too early, each design change would have been much slower and more frustrating.

However, breadboards are not ideal for high-current actuator circuits. They are useful for fast testing, but the final circuit should ideally move to a more secure connector-based or soldered layout once the design is verified.

---

## 7. What Could Have Been Done Better

The main thing that could have been done better was understanding the OpenCR power system earlier. We should have identified clearly which parts of the robot were powered by OpenCR, what current limits existed, and whether the solenoid could safely share the main battery. Because this was not fully clear at the start, we spent time making power decisions under uncertainty.

A better process would have been:

1. Create a full power architecture before wiring.
2. Identify voltage and current requirements for every component.
3. Separate high-current actuator paths from low-current signal paths.
4. Measure voltage at the load during operation.
5. Document failure thresholds, such as the solenoid becoming unreliable below a certain battery voltage.

This would have made the debugging process more systematic and reduced the time spent guessing whether the issue was caused by the battery, MOSFET, wiring, or software.

---

## Things Learnt

The biggest lesson I learnt is that power management is the foundation of the whole robot. If power delivery is unstable, software debugging becomes misleading because the robot may behave unpredictably even when the code is correct.

I also learnt that actuator circuits need to be treated differently from sensor or signal circuits. Sensors and GPIO lines use small currents, but solenoids and motors need proper wire gauge, switching components, flyback protection, and battery planning.

In short, power sources are painful, but managing them properly creates the foundation that allows the software and mechanical systems to work reliably.
