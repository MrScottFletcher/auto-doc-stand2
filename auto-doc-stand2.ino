/*
  Motorized Photo Stand Controller (Fresh Start)
  Board: Arduino Mega 2560 R3
  Stepper driver: BIGTREETECH TMC2209 v1.3 (UART on Serial3, addr 0b0)
  Motion: FastAccelStepper
  Display: GeekPi OLED I2C 128x64 (SSD1306)

  Step/Dir/En:
    STEP_PIN   = 7
    DIR_PIN    = 48
    ENABLE_PIN = 44

  Inputs:
    Joystick Y = A0
    Home/Min limit switch = D29 (to GND, use INPUT_PULLUP)
    E-Stop button = D2 (INT0) (to GND, use INPUT_PULLUP)
    Rehome button = D3 (INT1) (to GND, use INPUT_PULLUP)
    Store button  = D4 (to GND, use INPUT_PULLUP)
    Preset buttons: D31,33,35,37,39 (to GND, use INPUT_PULLUP)

  Serial:
    Serial  = USB console (optional, mirrors debug)
    Serial1 = Debug output (115200)   (pins TX1=18, RX1=19)
    Serial3 = TMC UART (115200)       (pins TX3=14, RX3=15)

  OLED I2C:
    SDA = 20, SCL = 21 on Mega
*/

#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ===================== DEBUG ===================== */
#define DEBUG_ENABLED 1

static void dbgPrintln(const String& s) {
#if DEBUG_ENABLED
  Serial1.println(s);
  Serial.println(s);
#endif
}
static void dbgPrintf(const char* fmt, ...) {
#if DEBUG_ENABLED
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial1.print(buf);
  Serial.print(buf);
#endif
}

/* ===================== PINS ===================== */
#define STEP_PIN     7
#define DIR_PIN      48
#define ENABLE_PIN   44

#define JOY_PIN      A0

#define HOME_PIN     19

#define ESTOP_PIN    2     // INT0
#define REHOME_PIN   3     // INT1

#define STORE_PIN    4
static const uint8_t PRESET_PINS[5] = {22, 24, 26, 28, 30};

/* ===================== OLED ===================== */
#define OLED_ADDR    0x3C
#define OLED_W       128
#define OLED_H       64
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

/* ===================== TMC2209 ===================== */
#define R_SENSE 0.11f
#define TMC_ADDR 0b0
TMC2209Stepper driver(&Serial3, R_SENSE, TMC_ADDR);

/* ===================== MOTION CONFIG ===================== */
// You manually configure this max travel. Units are STEPS (after microsteps).
// Example: if you want 200mm travel, and your lead screw is 8mm/rev,
// motor 200 steps/rev, microsteps 16 => steps/mm = (200*16)/8 = 400 steps/mm
// maxSteps = 200mm * 400 = 80000
static const unsigned long MAX_TRAVEL_STEPS = 655000;

// Speeds (Hz = steps/sec). Tune these.
static const uint32_t FAST_SPEED_HZ = 30000;
static const uint32_t SLOW_SPEED_HZ = 4000;

// Acceleration in steps/s^2. Tune for smoothness vs speed.
static const uint32_t ACCEL_STEPS_S2 = 120000;

// Homing speed (slow and gentle)
static const uint32_t HOME_SPEED_HZ = 30000; //orig value 4000

// Joystick
static const int JOG_DEADBAND = 60; // analog units around center (~512)
static const uint32_t JOG_MAX_SPEED_HZ = 36000; // max jog rate -- 12000
static const uint32_t JOG_MIN_SPEED_HZ = 3600;  // minimum when outside deadband -- 1200

// Debounce
static const uint16_t BTN_DEBOUNCE_MS = 35;

/* ===================== EEPROM LAYOUT ===================== */
static const uint32_t EEPROM_MAGIC = 0x5053544E; // 'PSTN'
static const int EEPROM_ADDR = 0;

struct PersistData {
  uint32_t magic;
  uint8_t  hasValidPosition;   // 1 if lastPosition is trusted
  //  int32_t  lastPositionSteps;  // 0 at home (bottom)
  int32_t  presets[5];         // preset positions
  uint16_t crc;                // simple checksum
};

static uint16_t simpleCrc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xA5A5;
  for (size_t i = 0; i < len; i++) crc = (crc << 1) ^ data[i] ^ (crc >> 15);
  return crc;
}

//Prototypes to help the compile understand the primacy
enum class Mode : uint8_t;
static void enterMode(Mode m);

// Forward declarations to defeat Arduino auto-prototyping
struct BtnState;
static bool edgePressed(uint8_t pin, BtnState& st);
static bool pressedEvent(uint8_t pin, BtnState& st);

/* ===================== STATE ===================== */
enum class Mode : uint8_t {
  STARTUP,
  IN_ERROR,
  HOMING,
  IDLING,
  JOG,
  MOVE_TO_PRESET,
  STORE_ARMED,
  ESTOP
};

static Mode mode = Mode::STARTUP;
static String lastError;

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;

// Motion tracking for progress bar
static int32_t moveStartPos = 0;
static int32_t moveTargetPos = 0;
static uint8_t currentPresetIndex = 255;

// Store flow
static bool storeArmed = false;

// Interrupt flags
volatile bool estopRequested = false;
volatile bool rehomeRequested = false;

/* ===================== BUTTON EDGE TRACKING ===================== */
struct BtnState {
  bool lastLevel = true; // pullup, so true = not pressed
  uint32_t lastChangeMs = 0;
};
static BtnState btnStore;
static BtnState btnPreset[5];

static PersistData persist;

/* ===================== UTILS ===================== */
static bool isPressed(uint8_t pin) {
  return digitalRead(pin) == LOW;
}

static int32_t clampPos(int32_t p) {
  if (p < 0) return 0;
  //During manual calibration - uncomment this when you figure out the Max Pos
  if (p > MAX_TRAVEL_STEPS) return MAX_TRAVEL_STEPS;
  return p;
}

static void oledClear() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
}

static void oledLine(uint8_t row, const String& s) {
  display.setCursor(0, row * 10);
  display.print(s);
}

static void oledProgressBar(uint8_t y, uint8_t h, float frac) {
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int w = OLED_W - 2;
  display.drawRect(0, y, OLED_W, h, SSD1306_WHITE);
  display.fillRect(1, y + 1, (int)((w - 1) * frac), h - 2, SSD1306_WHITE);
}

//==========================
//EEPROM can only handle about 100,000 writes.
//We're not going to burn out our EEPROM just for this.
static void savePersist() {
//  persist.lastPositionSteps = stepper ? stepper->getCurrentPosition() : persist.lastPositionSteps;
//  persist.lastPositionSteps = clampPos(persist.lastPositionSteps);

  // compute crc over everything except crc field
  persist.crc = 0;
  persist.crc = simpleCrc16((const uint8_t*)&persist, sizeof(PersistData));

  EEPROM.put(EEPROM_ADDR, persist);
  dbgPrintln(F("[EEPROM] Saved persist data."));
}

static bool loadPersist() {
  EEPROM.get(EEPROM_ADDR, persist);
  if (persist.magic != EEPROM_MAGIC) return false;

  uint16_t stored = persist.crc;
  PersistData tmp = persist;
  tmp.crc = 0;
  uint16_t calc = simpleCrc16((const uint8_t*)&tmp, sizeof(PersistData));
  if (stored != calc) return false;

  for (int i = 0; i < 5; i++) persist.presets[i] = clampPos(persist.presets[i]);
  
  //persist.lastPositionSteps = clampPos(persist.lastPositionSteps);
  return true;
}


static void initDefaultPersist() {
  memset(&persist, 0, sizeof(persist));
  persist.magic = EEPROM_MAGIC;
//  persist.hasValidPosition = 0;
//  persist.lastPositionSteps = 0;
  for (int i = 0; i < 5; i++) persist.presets[i] = 0;
  savePersist();
}
//==========================

static void enterError(const String& err) {
  mode = Mode::IN_ERROR;
  lastError = err;
  dbgPrintln("[IN_ERROR] " + err);
}

static void enterMode(Mode m) {
  mode = m;
}

/* ===================== DRIVER DUMP ===================== */
static void dumpDriverStatus() {
  dbgPrintln(F("=== TMC2209 Driver Dump ==="));

  // A few key reads; if UART is dead, these often return 0 or 0xFFFFFFFF depending on bus.
  uint32_t ioin = driver.IOIN();
  uint32_t gstat = driver.GSTAT();
  uint32_t chopconf = driver.CHOPCONF();
  uint32_t pwmconf = driver.PWMCONF();
  uint32_t ihold_irun = driver.IHOLD_IRUN();
  uint32_t tpowerdown = driver.TPOWERDOWN();
  uint32_t tpwmthrs = driver.TPWMTHRS();
  uint32_t tcoolthrs = driver.TCOOLTHRS();
  uint32_t sgthrs = driver.SGTHRS();

  dbgPrintf("IOIN:       0x%08lX\n", (unsigned long)ioin);
  dbgPrintf("GSTAT:      0x%02lX\n", (unsigned long)gstat);
  dbgPrintf("CHOPCONF:   0x%08lX\n", (unsigned long)chopconf);
  dbgPrintf("PWMCONF:    0x%08lX\n", (unsigned long)pwmconf);
  dbgPrintf("IHOLD_IRUN: 0x%08lX\n", (unsigned long)ihold_irun);
  dbgPrintf("TPOWERDOWN: 0x%02lX\n", (unsigned long)tpowerdown);
  dbgPrintf("TPWMTHRS:   0x%08lX\n", (unsigned long)tpwmthrs);
  dbgPrintf("TCOOLTHRS:  0x%08lX\n", (unsigned long)tcoolthrs);
  dbgPrintf("SGTHRS:     0x%02lX\n", (unsigned long)sgthrs);

  // Helpful derived values
  dbgPrintf("Microsteps: %u\n", driver.microsteps());
  dbgPrintf("IRUN:       %u\n", driver.irun());
  dbgPrintf("IHOLD:      %u\n", driver.ihold());

  dbgPrintln(F("==========================="));
}

/* ===================== INTERRUPTS ===================== */
void isrEstop() {
  estopRequested = true;
}
void isrRehome() {
  rehomeRequested = true;
}

/* ===================== INIT HARDWARE ===================== */
static bool initOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    dbgPrintln(F("[OLED] begin() failed."));
    return false;
  }
  oledClear();
  oledLine(0, F("Photo Stand"));
  oledLine(1, F("Initializing..."));
  display.display();
  return true;
}

static bool initDriver() {
  Serial3.begin(115200);
  delay(50);

  driver.begin();
  driver.pdn_disable(true);      // use UART
  driver.I_scale_analog(false);  // use internal reference
  driver.toff(5);
  driver.blank_time(24);
  driver.rms_current(900);       // tune to your motor
  driver.microsteps(16);
  driver.en_spreadCycle(false);  // stealthChop by default
  driver.pwm_autoscale(true);
  driver.TPOWERDOWN(10);

  // Quick comm sanity check
  uint32_t ioin = driver.IOIN();
  // If UART is floating/dead, IOIN is often 0xFFFFFFFF or 0
  if (ioin == 0xFFFFFFFFUL || ioin == 0x00000000UL) {
    dbgPrintf("[TMC] IOIN read suspicious: 0x%08lX\n", (unsigned long)ioin);
    return false;
  }

  // Clear driver errors
  driver.GSTAT(0b111);

  dumpDriverStatus();
  return true;
}

static bool initStepper() {
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) return false;

  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);

  stepper->setSpeedInHz(SLOW_SPEED_HZ);
  stepper->setAcceleration(ACCEL_STEPS_S2);

  //We're not going to do EEPROM - can only write about 100,000 times
  stepper->setCurrentPosition(MAX_TRAVEL_STEPS);
  return true;
}

/* ===================== HOMING ===================== */
static void startHoming() {
  dbgPrintln(F("[HOME] Starting homing..."));
  oledClear();
  oledLine(0, F("Homing..."));
  oledLine(1, F("Seeking bottom switch"));
  display.display();

  // Move down (assumes "down" direction corresponds to negative positions)
  // If your direction is reversed, flip this by swapping motor wiring or invert DIR.
  stepper->setSpeedInHz(HOME_SPEED_HZ);
  stepper->setAcceleration(ACCEL_STEPS_S2);

  // Move a long way down; we stop when switch hits.
  stepper->moveTo(- (int32_t)MAX_TRAVEL_STEPS * 2L);

  enterMode(Mode::HOMING);
}

static void processHoming() {
  // If switch pressed, stop and set zero
  if (isPressed(HOME_PIN)) {
    stepper->stopMove(); // decelerates smoothly
    while (stepper->isRunning()) {
      // allow it to ramp down
      if (estopRequested) break;
    }

    if (estopRequested) return;

    // Back off a little to release switch, then approach slowly again for repeatability
    stepper->setCurrentPosition(0); // temporary
    stepper->setSpeedInHz(3000); //orig 3000
    stepper->moveTo(2000); // move up away from switch
    while (stepper->isRunning()) {
      if (estopRequested) return;
    }

    // Final approach to switch slowly
    stepper->setSpeedInHz(1500); //orig 1500
    stepper->moveTo(-4000);
    while (stepper->isRunning()) {
      if (isPressed(HOME_PIN)) {
        stepper->stopMove();
        while (stepper->isRunning()) {
          if (estopRequested) return;
        }
        break;
      }
      if (estopRequested) return;
    }

    // Set true home at switch
    stepper->setCurrentPosition(0);
    persist.hasValidPosition = 1;
    //persist.lastPositionSteps = 0;
    savePersist();

    dbgPrintln(F("[HOME] Homing complete. Position=0"));
    enterMode(Mode::IDLING);
  }

  // If it somehow finishes moving without hitting the switch, error
  if (!stepper->isRunning() && !isPressed(HOME_PIN)) {
    enterError(F("Homing failed: switch never triggered."));
  }
}

/* ===================== MOVES ===================== */
static void startMoveTo(int32_t target, uint8_t presetIndexOr255) {
  if (!stepper) return;
  target = clampPos(target);

  // If already moving, command a new moveTo() is the gentlest “interrupt”
  // because FastAccelStepper uses accel/decel ramps.
  if (stepper->isRunning()) {
    // nothing special needed; moveTo() will change target and ramp accordingly
  }

  moveStartPos = stepper->getCurrentPosition();
  moveTargetPos = target;
  currentPresetIndex = presetIndexOr255;

  // Use fast speed for long travel, but keep accel reasonable for camera stability.
  stepper->setSpeedInHz(FAST_SPEED_HZ);
  stepper->setAcceleration(ACCEL_STEPS_S2);

  stepper->moveTo(target);

  enterMode(Mode::MOVE_TO_PRESET);
  dbgPrintf("[MOVE] Target=%ld preset=%u\n", (long)target, (unsigned)presetIndexOr255);
}

/* ===================== INPUT HANDLING ===================== */
static bool edgePressed(uint8_t pin, BtnState& st) {
  bool level = digitalRead(pin); // HIGH=idle, LOW=pressed
  uint32_t now = millis();
  if (level != st.lastLevel) {
    st.lastChangeMs = now;
    st.lastLevel = level;
    return false;
  }
  // stable long enough and is LOW => pressed edge detection is handled elsewhere (simple)
  return false;
}

static bool pressedEvent(uint8_t pin, BtnState& st) {
  bool level = digitalRead(pin); // HIGH idle, LOW pressed
  uint32_t now = millis();

  if (level != st.lastLevel) {
    // changed
    st.lastLevel = level;
    st.lastChangeMs = now;
    return false;
  }

  // stable
  if ((now - st.lastChangeMs) < BTN_DEBOUNCE_MS) return false;

  // if currently LOW and we haven't emitted the event, emit once by shifting lastChangeMs far forward
  if (level == LOW) {
    st.lastChangeMs = now + 60000UL; // crude "latch" until release
    return true;
  }

  // released: reset latch immediately
  if (level == HIGH && st.lastChangeMs > now) st.lastChangeMs = now;
  return false;
}

static void handleButtons() {
  // Store button
  if (pressedEvent(STORE_PIN, btnStore)) {
    storeArmed = true;
    enterMode(Mode::STORE_ARMED);
    dbgPrintln(F("[STORE] Armed. Press preset button to save."));
  }

  // Preset buttons
  for (int i = 0; i < 5; i++) {
    if (pressedEvent(PRESET_PINS[i], btnPreset[i])) {
      if (storeArmed) {
        persist.presets[i] = clampPos(stepper->getCurrentPosition());
        savePersist();
        storeArmed = false;
        dbgPrintf("[STORE] Saved preset %d = %ld\n", i + 1, (long)persist.presets[i]);
        enterMode(Mode::IDLING);
      } else {
        // Interrupt current travel gently by simply issuing a new moveTo
        startMoveTo(persist.presets[i], (uint8_t)i);
      }
    }
  }
}

static void handleJoystick() {
  // Only allow jog when not in homing/error/estop/store armed.
  if (mode == Mode::HOMING || mode == Mode::IN_ERROR || mode == Mode::ESTOP || mode == Mode::STORE_ARMED) return;

  int v = analogRead(JOY_PIN); // 0..1023
  int delta = v - 512;

  if (abs(delta) <= JOG_DEADBAND) {
    // If we were jogging, stop smoothly and go idle
    if (mode == Mode::JOG) {
      stepper->stopMove();
      enterMode(Mode::IDLING);
    }
    return;
  }

  // Map delta beyond deadband to speed
  int mag = abs(delta) - JOG_DEADBAND;
  int magMax = 512 - JOG_DEADBAND;
  if (magMax < 1) magMax = 1;

  uint32_t spd = (uint32_t)(JOG_MIN_SPEED_HZ + (uint32_t)(JOG_MAX_SPEED_HZ - JOG_MIN_SPEED_HZ) * (uint32_t)mag / (uint32_t)magMax);
  if (spd < JOG_MIN_SPEED_HZ) spd = JOG_MIN_SPEED_HZ;
  if (spd > JOG_MAX_SPEED_HZ) spd = JOG_MAX_SPEED_HZ;

  // Direction: delta > 0 => up (positive), delta < 0 => down (negative)
  int dir = (delta > 0) ? 1 : -1;

  // Use "runForward/runBackward" style by commanding far-away target with accel ramps
  stepper->setSpeedInHz(spd);
  stepper->setAcceleration(ACCEL_STEPS_S2);

  int32_t cur = stepper->getCurrentPosition();
  int32_t target = cur + dir * 200000L; // "effectively continuous"
  target = clampPos(target);

  stepper->moveTo(target);
  enterMode(Mode::JOG);
}

/* ===================== OLED UI ===================== */
static void updateOLED() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 100) return; // 10 Hz
  lastMs = now;

  oledClear();
  oledLine(0, F("Photo Stand"));

  if (mode == Mode::IN_ERROR) {
    oledLine(1, F("IN_ERROR:"));
    oledLine(2, lastError.substring(0, 20));
    oledLine(3, lastError.length() > 20 ? lastError.substring(20, 40) : "");
    display.display();
    return;
  }

  if (mode == Mode::ESTOP) {
    oledLine(1, F("E-STOP!"));
    oledLine(2, F("Reset required"));
    oledLine(3, F("Press Rehome"));
    display.display();
    return;
  }

  // Mode text
  String modeText;
  switch (mode) {
    case Mode::STARTUP: modeText = "Startup"; break;
    case Mode::HOMING: modeText = "Homing"; break;
    case Mode::IDLING: modeText = "Idle"; break;
    case Mode::JOG: modeText = "Jog"; break;
    case Mode::MOVE_TO_PRESET: modeText = "MoveToPreset"; break;
    case Mode::STORE_ARMED: modeText = "Store: pick preset"; break;
    default: modeText = "?"; break;
  }

  int32_t pos = stepper ? stepper->getCurrentPosition() : 0;
  oledLine(1, "Mode: " + modeText);
  oledLine(2, "Pos: " + String(pos));

  // Preset number
  if (mode == Mode::MOVE_TO_PRESET && currentPresetIndex != 255) {
    oledLine(3, "Preset: " + String(currentPresetIndex + 1));
  } else if (storeArmed) {
    oledLine(3, F("STORE ARMED"));
  } else {
    oledLine(3, F(""));
  }

  // Progress bar while moving to a preset
  if (mode == Mode::MOVE_TO_PRESET) {
    int32_t cur = pos;
    int32_t a = moveStartPos;
    int32_t b = moveTargetPos;
    float frac = 0.0f;
    int32_t denom = abs(b - a);
    if (denom > 0) frac = (float)abs(cur - a) / (float)denom;
    oledProgressBar(52, 10, frac);
  }

  display.display();
}

/* ===================== MAIN LOOP HELPERS ===================== */
static void handleEstopAndRehome() {
  if (estopRequested) {
    estopRequested = false;

    if (stepper) {
      stepper->forceStop(); // immediate
      // Keep current position as-is
    }
    enterMode(Mode::ESTOP);
    dbgPrintln(F("[ESTOP] Emergency stop triggered."));
    return;
  }

  if (rehomeRequested) {
    rehomeRequested = false;

    if (mode == Mode::ESTOP || mode == Mode::IDLING || mode == Mode::JOG || mode == Mode::MOVE_TO_PRESET || mode == Mode::STORE_ARMED) {
      // Stop motion gently, then home
      if (stepper && stepper->isRunning()) {
        stepper->stopMove();
        while (stepper->isRunning()) {
          if (estopRequested) return;
        }
      }
      storeArmed = false;
      startHoming();
    }
  }
}

static void persistPositionIfIdle() {
  static uint32_t lastSaveMs = 0;
  static int32_t lastSavedPos = 0;
  uint32_t now = millis();

  if (!stepper) return;

  if (mode == Mode::IDLING && persist.hasValidPosition) {
    int32_t cur = clampPos(stepper->getCurrentPosition());
    if (cur != lastSavedPos && (now - lastSaveMs) > 1500) {
      //We're not going to do EEPROM - can only write about 100,000 times
      //      persist.lastPositionSteps = cur;
      //      savePersist();
      lastSavedPos = cur;
      lastSaveMs = now;
    }
  }
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(HOME_PIN, INPUT_PULLUP);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(REHOME_PIN, INPUT_PULLUP);
  pinMode(STORE_PIN, INPUT_PULLUP);
  for (int i = 0; i < 5; i++) pinMode(PRESET_PINS[i], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), isrEstop, FALLING);
  attachInterrupt(digitalPinToInterrupt(REHOME_PIN), isrRehome, FALLING);

  dbgPrintln(F("\n=== Photo Stand Controller Boot ==="));

  bool oledOk = initOLED();
  if (!oledOk) {
    // continue without display
    dbgPrintln(F("[OLED] Not available; continuing headless."));
  }

  //We need to load the Preset buttons
  if (!loadPersist()) {
    dbgPrintln(F("[EEPROM] No valid data; initializing defaults."));
    initDefaultPersist();
  } else {
    dbgPrintln(F("[EEPROM] Loaded saved state."));
  }

  if (!initStepper()) {
    enterError(F("FastAccelStepper init failed."));
    return;
  }

  oledClear();
  oledLine(0, F("Init driver..."));
  display.display();

  if (!initDriver()) {
    // Show and log error on startup as requested
    enterError(F("TMC2209 UART failed (IOIN read invalid). Check TX3/RX3, GND, PDN_UART wiring."));
    return;
  }

  //We're not going to do EEPROM - can only write about 100,000 times
  //// Set initial position from EEPROM if trusted; otherwise home
  //  if (persist.hasValidPosition) {
  //    stepper->setCurrentPosition(persist.lastPositionSteps);
  //    dbgPrintf("[BOOT] Using stored position: %ld\n", (long)persist.lastPositionSteps);
  //
  //    oledClear();
  //    oledLine(0, F("Ready (no home)"));
  //    oledLine(1, "Pos: " + String(persist.lastPositionSteps));
  //    display.display();
  //
  //    enterMode(Mode::IDLING);
  //  } else {
  //    dbgPrintln(F("[BOOT] No valid position; homing required."));
  //    startHoming();
  //  }

    //On power up, just assume we're at the top.  Need to rehome every time.
    //We cuold use a series of limit switches to shorten the journey some day.
    stepper->setCurrentPosition(MAX_TRAVEL_STEPS);
    
    oledClear();
    oledLine(0, F("Ready (no home)"));
    //oledLine(1, "Pos: " + String(persist.lastPositionSteps));
    oledLine(1, "Pos: ZERO BUT WRONG?");
    display.display();

    enterMode(Mode::IDLING);

}

/* ===================== LOOP ===================== */
void loop() {
  if (mode == Mode::IN_ERROR) {
    updateOLED();
    return;
  }

  handleEstopAndRehome();
  if (mode == Mode::ESTOP) {
    updateOLED();
    return;
  }

  // Homing logic
  if (mode == Mode::HOMING) {
    processHoming();
    updateOLED();
    return;
  }

  // If store armed, still allow the preset buttons + display updates; do not jog.
  handleButtons();

  // Joystick jog (disabled while store-armed)
  if (!storeArmed) handleJoystick();

  // If we’re moving to a preset, detect arrival and go idle + save position.
  if (mode == Mode::MOVE_TO_PRESET) {
    if (!stepper->isRunning()) {
      //We're not going to do EEPROM - can only write about 100,000 times
      //      persist.lastPositionSteps = clampPos(stepper->getCurrentPosition());
      //      persist.hasValidPosition = 1;
      //      savePersist();
      currentPresetIndex = 255;
      enterMode(Mode::IDLING);
    }
  }

  // If jogging and motion stopped (e.g., hit clamp target), go idle
  if (mode == Mode::JOG) {
    if (!stepper->isRunning()) {
      enterMode(Mode::IDLING);
    }
  }

  persistPositionIfIdle();
  updateOLED();
}
