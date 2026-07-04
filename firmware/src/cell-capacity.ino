/*
 * Cell Capacity Tester — firmware
 * ---------------------------------
 * Constant-current 18650 discharge tester. An LM358 op-amp holds the discharge
 * current constant by driving a MOSFET gate; this firmware sets the target
 * current (via a filtered PWM "DAC"), measures the actual current, integrates
 * it over time to compute capacity (mAh), watches temperature and cell voltage
 * for safety cutoffs, and shows everything on a 128x64 I2C OLED.
 *
 * Libraries required (install via Library Manager):
 *   - Adafruit SSD1306
 *   - Adafruit GFX
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─── pin configuration ───────────────────────────
#define PIN_TEMP        A0   // NTC thermistor divider
#define PIN_VBAT        A1   // cell voltage sense (batt+)
#define PIN_ISENSE      A3   // voltage across load resistors (actual current)
#define PIN_CURRENT_PWM 3    // filtered PWM -> op-amp + input (target current)
#define PIN_BTN_UP      4    // SW1: UP / START (external pull-up per R5)
#define PIN_BTN_DOWN    5    // SW2: DOWN (external pull-up per R6)
#define PIN_BUZZER      6    // piezo buzzer

// ─── constants ─────────────────────────────────────────────────────
const float VREF        = 5.0;    // ADC reference (5V rail)
const float ADC_MAX     = 1023.0; // 10-bit ADC
const float R_SENSE     = 0.5;    // R4A || R4B = 1 || 1 = 0.5 ohm
const float VBAT_DIV     = 1.0;   // set to (R_top+R_bot)/R_bot if VBAT is divided; 1.0 = direct

// Thermistor (NTC): 10k @ 25C, series/divider with R2 = 10k.
// Divider: TEMP = VREF * R2 / (R2 + Rntc)  (NTC to VREF, R2 to GND)
const float NTC_R0      = 10000.0; // resistance at 25C
const float NTC_T0      = 298.15;  // 25C in Kelvin
const float NTC_BETA    = 3950.0;  // beta coefficient — check your part
const float R_FIXED     = 10000.0; // R2

// ─── trip limits ───────────────────────────────────────────────────
const float V_CUTOFF    = 2.50;   // stop discharge at this cell voltage 
const float TEMP_MAX_C  = 60.0;   // fault cutoff temperature
const float I_MIN_MA    = 100.0;  // min selectable current
const float I_MAX_MA    = 2000.0; // max selectable current
const float I_STEP_MA   = 100.0;  // adjustment step

// ─── state ──────────────────────────────────────────────────────────────────
enum State { IDLE, RUNNING, DONE, FAULT };
State state = IDLE;

float targetCurrent_mA = 500.0;   // selected setpoint
float capacity_mAh     = 0.0;     // accumulated capacity
unsigned long lastIntegrateMs = 0;
unsigned long testStartMs     = 0;
const char *faultReason = "";

// Button debounce
struct Button {
  uint8_t pin;
  bool lastRaw;
  bool stable;
  unsigned long lastChangeMs;
  unsigned long pressedAtMs;
  bool longFired;
};
Button btnUp   = {PIN_BTN_UP,   HIGH, HIGH, 0, 0, false};
Button btnDown = {PIN_BTN_DOWN, HIGH, HIGH, 0, 0, false};
const unsigned long DEBOUNCE_MS  = 30;
const unsigned long LONGPRESS_MS = 1000;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  pinMode(PIN_CURRENT_PWM, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BTN_UP, INPUT);    // external pull-ups (R5/R6). Use INPUT_PULLUP if none.
  pinMode(PIN_BTN_DOWN, INPUT);
  setDischarge(0);               // ensure load is off at boot

  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init failed"));
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void loop() {
  pollButtons();

  switch (state) {
    case IDLE:    handleIdle();    break;
    case RUNNING: handleRunning(); break;
    case DONE:    handleDone();    break;
    case FAULT:   handleFault();   break;
  }

  updateDisplay();
}

void handleIdle() {
  setDischarge(0);
  // UP short = increase current, DOWN short = decrease current, UP long = START
  if (btnUp.longFired) {
    btnUp.longFired = false;
    startTest();
  }
}

void handleRunning() {
  float v = readCellVoltage();
  float t = readTemperatureC();
  float i = readCurrent_mA();

  // safety cutoffs
  if (t >= TEMP_MAX_C)      { faultReason = "OVERTEMP"; enterFault(); return; }
  if (v <= V_CUTOFF)        { finishTest(); return; }

  // integrate charge: mAh += mA * (dt in hours)
  unsigned long now = millis();
  float dtHours = (now - lastIntegrateMs) / 3600000.0;
  lastIntegrateMs = now;
  capacity_mAh += i * dtHours;
}

void handleDone()  { setDischarge(0); }
void handleFault() { setDischarge(0); }

// ─── test control ───────────────────────────────────────────────────────────
void startTest() {
  capacity_mAh = 0.0;
  testStartMs = millis();
  lastIntegrateMs = testStartMs;
  applyTargetCurrent();
  state = RUNNING;
  beep(2, 80);   // two short beeps on start
}

void finishTest() {
  setDischarge(0);
  state = DONE;
  beep(3, 150);  // three beeps: complete
}

void enterFault() {
  setDischarge(0);
  state = FAULT;
  beep(5, 300);  // long fault pattern
}

// ─── current control ────────────────────────────────────────────────────────
// Target current is set by the voltage on the op-amp + input:
//   V_target = I_target(A) * R_SENSE
// The filtered PWM produces  V = (duty/255) * VREF, so:
//   duty = V_target / VREF * 255
void applyTargetCurrent() {
  float vTarget = (targetCurrent_mA / 1000.0) * R_SENSE;
  int duty = (int)((vTarget / VREF) * 255.0 + 0.5);
  duty = constrain(duty, 0, 255);
  analogWrite(PIN_CURRENT_PWM, duty);
}

void setDischarge(int duty) {
  analogWrite(PIN_CURRENT_PWM, constrain(duty, 0, 255));
}

// ─── measurement ────────────────────────────────────────────────────────────
float readCurrent_mA() {
  float vSense = analogRead(PIN_ISENSE) * (VREF / ADC_MAX); // volts across R_SENSE
  return (vSense / R_SENSE) * 1000.0; // I = V/R, to mA
}

float readCellVoltage() {
  float v = analogRead(PIN_VBAT) * (VREF / ADC_MAX);
  return v * VBAT_DIV;
}

float readTemperatureC() {
  float vNode = analogRead(PIN_TEMP) * (VREF / ADC_MAX);
  if (vNode <= 0.001) return -273.15;                 // avoid divide-by-zero
  float rNtc = R_FIXED * (VREF / vNode - 1.0);          // NTC to VREF, R2 to GND
  // Beta equation: 1/T = 1/T0 + (1/B) * ln(R/R0)
  float tK = 1.0 / (1.0 / NTC_T0 + (1.0 / NTC_BETA) * log(rNtc / NTC_R0));
  return tK - 273.15;
}

// ─── display ────────────────────────────────────────────────────────────────
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  switch (state) {
    case IDLE:
      display.println(F("READY"));
      display.print(F("Set: "));
      display.print(targetCurrent_mA, 0);
      display.println(F(" mA"));
      display.println();
      display.println(F("UP/DOWN: set current"));
      display.println(F("hold UP: start"));
      break;

    case RUNNING:
      display.println(F("DISCHARGING"));
      printLine(F("V:  "), readCellVoltage(), F(" V"));
      printLine(F("I:  "), readCurrent_mA(), F(" mA"));
      printLine(F("T:  "), readTemperatureC(), F(" C"));
      printLine(F("Cap:"), capacity_mAh, F(" mAh"));
      break;

    case DONE:
      display.println(F("TEST COMPLETE"));
      printLine(F("Cap:"), capacity_mAh, F(" mAh"));
      printLine(F("V:  "), readCellVoltage(), F(" V"));
      break;

    case FAULT:
      display.println(F("!! FAULT !!"));
      display.println(faultReason);
      printLine(F("Cap:"), capacity_mAh, F(" mAh"));
      break;
  }
  display.display();
}

void printLine(const __FlashStringHelper *label, float value, const __FlashStringHelper *unit) {
  display.print(label);
  display.print(value, 1);
  display.println(unit);
}

// ─── buttons ────────────────────────────────────────────────────────────────
void pollButtons() {
  updateButton(btnUp,   onUpShort,   onUpLong);
  updateButton(btnDown, onDownShort, NULL);
}

void updateButton(Button &b, void (*onShort)(), void (*onLong)()) {
  bool raw = digitalRead(b.pin);
  unsigned long now = millis();

  if (raw != b.lastRaw) { b.lastChangeMs = now; b.lastRaw = raw; }
  if ((now - b.lastChangeMs) > DEBOUNCE_MS && raw != b.stable) {
    b.stable = raw;
    if (b.stable == LOW) { // pressed (active low)
      b.pressedAtMs = now;
      b.longFired = false;
    } else {  // released
      if (!b.longFired && onShort) onShort();
    }
  }

  // long-press fires while still held
  if (b.stable == LOW && !b.longFired && onLong &&
      (now - b.pressedAtMs) > LONGPRESS_MS) {
    b.longFired = true;
    onLong();
  }
}

void onUpShort() {
  if (state == IDLE) {
    targetCurrent_mA = min(targetCurrent_mA + I_STEP_MA, I_MAX_MA);
  } else if (state == DONE || state == FAULT) {
    state = IDLE;         // acknowledge -> back to ready
  }
}

void onUpLong() {
  if (state == IDLE) startTest();     // hold UP to start
}

void onDownShort() {
  if (state == IDLE) {
    targetCurrent_mA = max(targetCurrent_mA - I_STEP_MA, I_MIN_MA);
  } else if (state == RUNNING) {
    finishTest();          // DOWN aborts a running test
  } else {
    state = IDLE;
  }
}

// ─── buzzer ─────────────────────────────────────────────────────────────────
void beep(int count, int ms) {
  for (int i = 0; i < count; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(ms);
    digitalWrite(PIN_BUZZER, LOW);
    if (i < count - 1) delay(ms);
  }
}
