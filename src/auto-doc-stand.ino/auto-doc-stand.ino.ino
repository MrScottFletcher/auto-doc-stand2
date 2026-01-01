/*
  Motorized Photo Stand Controller - Arduino UNO R3 + TMC2209 (UART + StallGuard)
  Features:
   - Joystick jog up/down with speed proportional to deflection
   - StallGuard homing (no limit switches): find MIN then MAX
   - EEPROM: remembers last position, max travel, and 5 presets (no re-home every boot once homed)
   - Accel/decel (ramping) for all moves
   - 5 preset buttons + store button (stores current position into selected slot or default slot 1)
   - Emergency stop button (interrupt) - gentle stop (decelerate)
   - Re-home button (interrupt current motion; gentle stop; then home)
   - Idle current reduction (hold current) via TMC2209 IHOLD/IRUN
   - OLED status display (SSD1306 I2C 128x64)

  IMPORTANT NOTES:
   - TMC2209 StallGuard requires SpreadCycle (en_spreadCycle(true)) on most builds.
     TMCStepper uses en_spreadCycle(bool) where true enables SpreadCycle.
   - StallGuard sensitivity must be tuned: SGTHRS.
   - For maximum speed without losing StallGuard reliability, this sketch uses:
       FAST profile for normal travel (StallGuard mostly OFF)
       STALL profile for homing/rehome (StallGuard ON)
   - Wiring (summary):
       STEP D3, DIR D4, EN D5
       PDN_UART D7 (single-wire UART)
       DIAG D2 (INT0) for stall interrupt
       Joystick Y A0
       Store D6
       Presets D10,D11,D12,D13,A1
       STOP (E-Stop) on D2?  <-- NOTE: D2 is used by DIAG. Use a separate pin for e-stop.
         Use D9 for E-stop in this code (can change).
       REHOME A2
       OLED SDA A4, SCL A5
   - Motor PSU: 12-24V to VM on driver, common GND with Arduino.
*/

#include <AccelStepper.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- Pins --------------------
#define STEP_PIN        3
#define DIR_PIN         4
#define ENABLE_PIN      5

#define TMC_UART_PIN    7   // PDN_UART (single wire)
#define DIAG_PIN        2   // Stall interrupt (INT0)

#define JOYSTICK_Y      A0

#define STORE_BUTTON    6
#define REHOME_BUTTON   A2

// Preset buttons (5)
const uint8_t presetPins[5] = {10, 11, 12, 13, A1};

// Separate E-Stop/Interrupt button (NOT on D2 because D2 is DIAG)
#define ESTOP_BUTTON    9   // INPUT_PULLUP, press = stop gently

// -------------------- OLED --------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------------------- Stepper + Driver --------------------
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00
SoftwareSerial TMCSerial(TMC_UART_PIN, TMC_UART_PIN); // single-wire UART
TMC2209Stepper driver(&TMCSerial, R_SENSE, DRIVER_ADDRESS);

// -------------------- Motion Profiles --------------------
// Tune these for your mechanics.
// FAST is for normal travel/presets. StallGuard is not relied upon.
#define FAST_SPEED      8000.0f   // steps/sec
#define FAST_ACCEL      6000.0f   // steps/sec^2

// STALL is for homing only: slow & reliable stall detection
#define STALL_SPEED     1200.0f
#define STALL_ACCEL      800.0f

// Jogging
#define JOG_DEADBAND      50      // around center (512)
#define JOG_MIN_SPEED    200.0f
#define JOG_MAX_SPEED   FAST_SPEED

// -------------------- TMC Parameters --------------------
// You MUST tune SGTHRS. Start around 80-120 and adjust.
#define SGTHRS_VALUE     100

// RMS current settings (tune for your motor & driver cooling).
// Using IHOLD/IRUN gives idle current reduction automatically.
#define IRUN_VALUE        31   // 0..31 (approx run current)
#define IHOLD_VALUE       10   // 0..31 (hold current ~30% of run)
#define IHOLDDELAY_VALUE   8   // 0..15

// -------------------- EEPROM Layout --------------------
// 0: homedFlag (byte)
// 1..4: maxPos (long)
// 5..8: lastPos (long)
// 9..(9+5*4-1): presets[5] (long each)
const int EE_ADDR_HOMED   = 0;
const int EE_ADDR_MAXPOS  = 1;
const int EE_ADDR_LASTPOS = 5;
const int EE_ADDR_PRESETS = 9;

// -------------------- State --------------------
enum SystemMode : uint8_t {
  MODE_HOMING = 0,
  MODE_IDLE,
  MODE_JOGGING,
  MODE_MOVING_TO_PRESET,
  MODE_STOPPING
};

SystemMode mode = MODE_HOMING;

volatile bool stallDetected = false;

long minPos = 0;      // always 0 after homing
long maxPos = 0;      // discovered by homing
long presets[5] = {0, 0, 0, 0, 0};

bool displayDirty = true;
unsigned long lastDisplayMs = 0;
unsigned long lastPosSaveMs = 0;
long lastSavedPos = 0;

// For STORE button: store to slot selected by holding a preset button, else store to slot 1
bool storeLatch = false;
bool rehomeLatch = false;

// -------------------- Helpers --------------------
void stallISR() {
  stallDetected = true;
}

bool pressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

void setFastProfile() {
  stepper.setMaxSpeed(FAST_SPEED);
  stepper.setAcceleration(FAST_ACCEL);

  // Disable StallGuard reliance at high speed by setting a low threshold.
  // StallGuard is effective only below TCOOLTHRS; tuning varies per build.
  driver.TCOOLTHRS(300); // StallGuard mostly OFF during fast travel
}

void setStallProfile() {
  stepper.setMaxSpeed(STALL_SPEED);
  stepper.setAcceleration(STALL_ACCEL);

  // Enable StallGuard for homing over broad speed range.
  driver.TCOOLTHRS(0xFFFFF);
}

void loadEEPROM() {
  uint8_t homed = EEPROM.read(EE_ADDR_HOMED);

  EEPROM.get(EE_ADDR_MAXPOS, maxPos);
  EEPROM.get(EE_ADDR_LASTPOS, lastSavedPos);
  EEPROM.get(EE_ADDR_PRESETS, presets);

  if (homed == 1 && maxPos > 0) {
    minPos = 0;
    stepper.setCurrentPosition(lastSavedPos);
    mode = MODE_IDLE;
  } else {
    mode = MODE_HOMING;
  }
}

void saveLastPositionIfNeeded() {
  unsigned long now = millis();
  if (now - lastPosSaveMs < 1000) return;

  long p = stepper.currentPosition();
  if (p != lastSavedPos) {
    EEPROM.put(EE_ADDR_LASTPOS, p);
    lastSavedPos = p;
  }
  lastPosSaveMs = now;
}

void saveMaxPosAndFlag() {
  EEPROM.put(EE_ADDR_MAXPOS, maxPos);
  EEPROM.write(EE_ADDR_HOMED, 1);
}

void savePresets() {
  EEPROM.put(EE_ADDR_PRESETS, presets);
}

// Soft limits: enforce after homed
void enforceSoftLimits() {
  if (EEPROM.read(EE_ADDR_HOMED) != 1) return;

  long p = stepper.currentPosition();
  if (p < minPos) {
    stepper.stop();
    while (stepper.isRunning()) stepper.run();
    stepper.setCurrentPosition(minPos);
  }
  if (p > maxPos) {
    stepper.stop();
    while (stepper.isRunning()) stepper.run();
    stepper.setCurrentPosition(maxPos);
  }
}

// Gentle stop request
void requestStop() {
  stepper.stop(); // decelerate
  mode = MODE_STOPPING;
  displayDirty = true;
}

// Execute homing (blocking but safe). If you want non-blocking FSM later, say so.
bool homeAxisWithStall() {
  setStallProfile();

  // Clear stall latch
  stallDetected = false;

  // Move down to find MIN stall (hard stop)
  stepper.moveTo(-200000L);
  while (!stallDetected) {
    stepper.run();
    // allow e-stop/rehome override even during homing
    if (pressed(ESTOP_BUTTON)) { requestStop(); return false; }
    if (pressed(REHOME_BUTTON)) { /* already homing */ }
  }

  // Stop gently and settle
  stepper.stop();
  while (stepper.isRunning()) stepper.run();

  // Set MIN position
  stepper.setCurrentPosition(0);
  minPos = 0;

  delay(250);

  // Find MAX stall
  stallDetected = false;
  stepper.moveTo(200000L);
  while (!stallDetected) {
    stepper.run();
    if (pressed(ESTOP_BUTTON)) { requestStop(); return false; }
  }

  stepper.stop();
  while (stepper.isRunning()) stepper.run();

  maxPos = stepper.currentPosition();
  if (maxPos < 1000) {
    // Something went wrong; don't mark homed.
    return false;
  }

  // Persist
  saveMaxPosAndFlag();

  // Clamp current position inside range (near max endstop)
  if (stepper.currentPosition() > maxPos) stepper.setCurrentPosition(maxPos);

  // Return to last saved position if it exists and is in range
  long lastPos;
  EEPROM.get(EE_ADDR_LASTPOS, lastPos);
  if (lastPos >= minPos && lastPos <= maxPos) {
    setFastProfile();
    stepper.moveTo(lastPos);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
      if (pressed(ESTOP_BUTTON)) { requestStop(); break; }
    }
  }

  setFastProfile();
  mode = MODE_IDLE;
  displayDirty = true;
  return true;
}

// Joystick jog (speed mode) with soft limits
void handleJoystick() {
  int y = analogRead(JOYSTICK_Y) - 512;

  if (abs(y) < JOG_DEADBAND) {
    // no jog request
    if (mode == MODE_JOGGING) {
      stepper.stop();
      mode = MODE_IDLE;
      displayDirty = true;
    }
    return;
  }

  mode = MODE_JOGGING;
  setFastProfile();              // allow fast jog
  stepper.setAcceleration(FAST_ACCEL);

  float speed = map(abs(y), 0, 512, (long)JOG_MIN_SPEED, (long)JOG_MAX_SPEED);
  float signedSpeed = (y > 0) ? speed : -speed;

  // Block jogging past limits when homed
  if (EEPROM.read(EE_ADDR_HOMED) == 1) {
    long p = stepper.currentPosition();
    if (signedSpeed < 0 && p <= minPos) signedSpeed = 0;
    if (signedSpeed > 0 && p >= maxPos) signedSpeed = 0;
  }

  stepper.setSpeed(signedSpeed);
  stepper.runSpeed();
  displayDirty = true;
}

// Preset move
void goToPreset(uint8_t index) {
  if (index >= 5) return;

  if (EEPROM.read(EE_ADDR_HOMED) == 1) {
    long target = presets[index];
    if (target < minPos) target = minPos;
    if (target > maxPos) target = maxPos;
    presets[index] = target; // keep in range
  }

  setFastProfile();
  stepper.moveTo(presets[index]);
  mode = MODE_MOVING_TO_PRESET;
  displayDirty = true;
}

// Store current position into selected slot
void handleStoreButton() {
  bool storeNow = pressed(STORE_BUTTON);
  if (storeNow && !storeLatch) {
    storeLatch = true;

    // Determine slot: if any preset button is held, store to that one; else store to slot 0.
    int slot = 0;
    for (int i = 0; i < 5; i++) {
      if (pressed(presetPins[i])) { slot = i; break; }
    }

    long p = stepper.currentPosition();
    if (EEPROM.read(EE_ADDR_HOMED) == 1) {
      if (p < minPos) p = minPos;
      if (p > maxPos) p = maxPos;
    }

    presets[slot] = p;
    savePresets();
    displayDirty = true;
  }
  if (!storeNow) storeLatch = false;
}

// Rehome button (interrupt travel gently)
void handleRehomeButton() {
  bool rh = pressed(REHOME_BUTTON);
  if (rh && !rehomeLatch) {
    rehomeLatch = true;
    requestStop();       // gentle decel
    while (stepper.isRunning()) stepper.run();
    mode = MODE_HOMING;
    displayDirty = true;
  }
  if (!rh) rehomeLatch = false;
}

// Handle preset buttons (each interrupts current travel gently, then heads to new target)
void handlePresetButtons() {
  for (int i = 0; i < 5; i++) {
    if (pressed(presetPins[i])) {
      // gentle interrupt
      if (mode == MODE_MOVING_TO_PRESET || mode == MODE_JOGGING) {
        stepper.stop();
        while (stepper.isRunning()) stepper.run();
      }
      goToPreset(i);
      delay(120); // simple debounce; improve later if needed
      break;
    }
  }
}

void handleEstop() {
  if (pressed(ESTOP_BUTTON)) {
    requestStop();
  }
}

void updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayMs < 150 && !displayDirty) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Pos: ");
  display.println(stepper.currentPosition());

  display.print("Min: ");
  display.print(minPos);
  display.print("  Max: ");
  display.println(maxPos);

  display.print("Mode: ");
  switch (mode) {
    case MODE_HOMING: display.println("HOMING"); break;
    case MODE_IDLE: display.println("IDLE"); break;
    case MODE_JOGGING: display.println("JOG"); break;
    case MODE_MOVING_TO_PRESET: display.println("GOTO"); break;
    case MODE_STOPPING: display.println("STOP"); break;
    default: display.println("?"); break;
  }

  display.print("SG: ");
  display.println(driver.SG_RESULT());

  display.display();
  displayDirty = false;
  lastDisplayMs = now;
}

// -------------------- Setup --------------------
void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // enable driver (LOW for many drivers)

  pinMode(DIAG_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallISR, FALLING);

  pinMode(ESTOP_BUTTON, INPUT_PULLUP);
  pinMode(STORE_BUTTON, INPUT_PULLUP);
  pinMode(REHOME_BUTTON, INPUT_PULLUP);

  for (int i = 0; i < 5; i++) {
    pinMode(presetPins[i], INPUT_PULLUP);
  }

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  // Stepper base setup
  stepper.setEnablePin(ENABLE_PIN);
  stepper.setPinsInverted(false, false, true); // enable pin inverted? depends; if wrong, flip last bool
  stepper.enableOutputs();

  // TMC setup
  TMCSerial.begin(115200);
  driver.begin();

  // Basic driver config
  driver.toff(5);
  driver.blank_time(24);

  // Current config (idle reduction via IHOLD)
  driver.irun(IRUN_VALUE);
  driver.ihold(IHOLD_VALUE);
  driver.iholddelay(IHOLDDELAY_VALUE);

  driver.microsteps(16);

  // StallGuard requirements:
  // - SpreadCycle enabled
  // - CoolStep/Stall thresholds set appropriately
  driver.en_spreadCycle(true);    // SpreadCycle ON for StallGuard
  driver.en_pwm_mode(false);      // StealthChop OFF (StallGuard doesn't work reliably with it)

  driver.SGTHRS(SGTHRS_VALUE);

  // Start in fast profile; homing will switch to stall profile if needed
  setFastProfile();

  loadEEPROM();
  displayDirty = true;
}

// -------------------- Loop --------------------
void loop() {
  // Highest priority controls
  handleEstop();
  handleRehomeButton();

  if (mode == MODE_STOPPING) {
    // complete gentle stop
    stepper.run();
    if (!stepper.isRunning()) {
      mode = MODE_IDLE;
      displayDirty = true;
    }
    updateDisplay();
    return;
  }

  if (mode == MODE_HOMING) {
    displayDirty = true;
    updateDisplay();
    bool ok = homeAxisWithStall();
    if (!ok) {
      // Failed homing: remain in idle but NOT homed
      EEPROM.write(EE_ADDR_HOMED, 0);
      mode = MODE_IDLE;
    }
    return;
  }

  // Normal operation
  handleStoreButton();
  handlePresetButtons();
  handleJoystick();

  // Preset move logic
  if (mode == MODE_MOVING_TO_PRESET) {
    stepper.run();
    if (stepper.distanceToGo() == 0) {
      mode = MODE_IDLE;
      displayDirty = true;
    }
  }

  enforceSoftLimits();
  saveLastPositionIfNeeded();
  updateDisplay();
}
