# Cell Capacity Tester
An Arduino-based constant-current discharge tester for 18650 lithium cells. It pulls a selected/controlled current out of a cell and 
integrates that current over time to compute the cell's true capacity in mAh, with live readout on an OLED and automatic thermal cutoff for safety.

## How it works
The circuitry is a constant-current electronic load with an op-amp continuously adjusts a MOSFET's gate voltage to hold a selected discharge current steady 
regardless of the cell's falling voltage. By integrating that known current over time, the firmware accumulates total charge delivered, the cell's capacity, by multiplying the time and current.

## Circuit Overview
### Power system
BT1 (the 18650 cell) is the sole power source at ~3.6–4.2 V. Current flows from the positive terminal through **F1** (4 A resettable polyfuse) 
and **D1** (Schottky diode, low forward drop, blocks reverse insertion) onto the batt+ net. 
**C1 (100 nF)** and **C2 (220 µF)** in parallel stabilize batt+, where C2 absorbs voltage sags when the MOSFET draws heavy current, C1 filters high-frequency noise.

The Arduino is powered from USB (VUSB) +5V rail, and its 3V3 pin supplies the +3.3 V rail for the OLED.

### Constant-current discharge
- **Q1 (IRLZ44N)**: logic-level MOSFET in series with the load, acting as a variable resistor controlled by gate voltage. Fully on at 3.3–5 V, so it's directly Arduino-drivable.
- **R4A & R4B**: two 1 Ω / 10 W resistors in parallel (0.5 Ω total) as the sense/load resistors. All discharge current flows through them; voltage across them = I × 0.5 Ω.
- **U1A (LM358)**: op-amp in the feedback loop:
  - '–' input (pin 2): actual voltage across the load resistors (i.e. actual current)
  - '+' input (pin 3): filtered PWM from Arduino A2 (target current)
  - The op-amp drives the gate to null the difference: if actual current is too low the gate rises (MOSFET opens more); too high and it closes slightly.
- **R7 (470 Ω)** between op-amp output and gate damps high-frequency oscillation.
- **R3 / C3** form a low-pass RC filter (τ = 10k × 100 n = 1 ms) that smooths the A2 PWM into a steady DC target voltage.

### Current setting
**SW1 (D14)** and **SW2 (D15)** are pulled up to +5 V through R5/R6. Pressing a button pulls the pin to GND; the Arduino detects the falling edge and increases or decreases the A2 PWM duty cycle, 
which changes the target voltage at the op-amp and therefore the set current. SW1 is UP/START — it also begins the test.

### Temperature sensing
**TH1 (10k NTC thermistor)** forms a voltage divider with R2 (10k):
```
TEMP = 5 V × R2 / (R2 + TH1)
```
The Arduino reads TEMP on **A0**. As temperature rises, TH1's resistance drops and TEMP rises. Firmware uses this to cut off the test above a safe threshold. 
Mount TH1 as close to the battery holder as possible so it can be bent to contact the cell.

### OLED display (DS1)
128×64 I²C OLED powered from 3.3 V and displays voltage, current, temperature, and accumulated mAh live during the test.

### Buzzer
Driven directly by the Arduino, beeps on test start, end, and fault.

### Op-amp protection
The LM358 is a dual op-amp. U1A does the constant-current control; the unused sections are wired as unity-gain followers with negative feedback so their inputs aren't left floating 
(floating inputs can be driven to the rails by EMI, causing excess current draw and rail instability). C4 filters VCC into the op-amp.

## Bill of Materials
Full BOM: [Google Sheets](https://docs.google.com/spreadsheets/d/1DATSQ3IbzevVbkfX_8bRBYmYYDkdU8qtv9zlPYZS3Pw/edit?usp=sharing)
All components are sourced from **DigiKey** except the **OLED screen**, which is purchased from Amazon.
