/*
  Motorized Photo Stand Controller
  Board: Elegoo MEGA 2560 R3
  Driver: TMC2209 (UART + StallGuard)
  Motion: FastAccelStepper
  Display: SSD1306 I2C 128x64

  STEP = 7, DIR = 48, ENABLE = 44
*/

#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ===================== DEBUG ===================== */
#define DEBUG_V 1   // very verbose (loops, OLED refresh)
#define DEBUG_I 1   // informational (state changes)
#define DEBUG_E 1   // errors / warnings

#define DBG_V(x) do { if (DEBUG_V) Serial.println(x); } while(0)
#define DBG_I(x) do { if (DEBUG_I) Serial.println(x); } while(0)
#define DBG_E(x) do { if (DEBUG_E) Serial.println(x); } while(0)

/* ===================== PIN ASSIGNMENTS ===================== */
#define STEP_PIN     7
#define DIR_PIN      48
#define ENABLE_PIN   44

#define DIAG_PIN     19
#define ESTOP_PIN     2
#define REHOME_PIN    3

#define STORE_PIN     4
const uint8_t presetPins[5] = {31, 33, 35, 37, 39};

#define JOYSTICK_Y   A0

/* ===================== UART ===================== */
#define TMC_SERIAL Serial3

/* ===================== OLED ===================== */
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledEnabled = true;
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_INTERVAL 150

/* ===================== FAST STEPPER ===================== */
FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;

/* ===================== TMC2209 ===================== */
#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b0
TMC2209Stepper driver(&TMC_SERIAL, R_SENSE, DRIVER_ADDRESS);

/* ===================== MECHANICS ===================== */
#define STEPS_PER_REV 200
#define MICROSTEPS     8
#define LEADSCREW_MM  8.0

/* ===================== MOTION ===================== */
#define FAST_SPEED    60000UL
#define FAST_ACCEL   120000UL

#define JOG_CHUNK      20000L
#define JOG_DEADBAND   50
#define JOG_MIN_SPEED  2000UL

/* ===================== TMC CURRENT ===================== */
#define RUN_IRUN   31
#define HOME_IRUN  18

/* ===================== EEPROM ===================== */
#define EE_HOMED        0
#define EE_MAXPOS       1
#define EE_LASTPOS      5
#define EE_PRESETS      9

#define EE_SG_VALID    60
#define EE_SG_FREEAVG  61
#define EE_SG_STALLMIN 63
#define EE_SG_THR      65
#define EE_SG_CONF     67

/* ===================== STATE ===================== */
enum Mode { HOMING, IDLE, JOGGING, MOVING, STOPPING };
Mode mode = HOMING;


volatile bool stallDetected = false;
volatile bool stallDetected_Displayed = false;

volatile bool estopTriggered = false;
volatile bool estopTriggered_Displayed = false;

volatile bool rehomeTriggered = false;
volatile bool rehomeTriggered_Displayed = false;


long minPos = 0;
long maxPos = 0;
long presets[5];
int activePreset = -1;

/* ===== Motion tracking for progress bar ===== */
long moveStartPos = 0;
long moveTargetPos = 0;

/* ===================== STALLGUARD ===================== */
#define HOME_FAST_SPEED 30000UL // orig 10000UL
#define HOME_FAST_ACCEL 20000UL // orig 20000UL

#define HOME_SLOW_SPEED 10000UL // orig 1500UL
#define HOME_SLOW_ACCEL 3000UL // orig 3000UL

#define HOME_BACKOFF    -100L // orig 300L
//#define HOME_SEEK_BOTTOM -40000 // orig -40000
//#define HOME_SEEK_TOP -20000 // orig -20000

uint16_t sgFreeAvg = 0;
uint16_t sgStallMin = 1203; //set the sensitivity with this.... i.e. TCMdriver.SGTHRS(120)
uint16_t sgDerivedThrs = 100;
uint8_t homingConfidence = 0;
bool sgCalibrated = false;

uint16_t lastSG = 0;

/* ===== Calibration summary ===== */
bool showCalSummary = false;
unsigned long calSummaryStart = 0;
#define CAL_SUMMARY_TIME 4000

/* ===================== HELPERS ===================== */
float stepsToMM(long s) {
  return (s * LEADSCREW_MM) / (STEPS_PER_REV * MICROSTEPS);
}

bool withinLimits(long p) {
  return p >= minPos && p <= maxPos;
}

/* ===================== ISR ===================== */
void stallISR()  { 
  stallDetected = true;
  stallDetected_Displayed = false;
}

void estopISR()  { 
  estopTriggered = true; 
  estopTriggered_Displayed = false; 
}
void rehomeISR() { 
  rehomeTriggered = true; 
  rehomeTriggered_Displayed = false; 
}

/* ===================== EEPROM CAL ===================== */
bool loadSGCalibration() {
  if (EEPROM.read(EE_SG_VALID) != 0xA5) return false;
  EEPROM.get(EE_SG_FREEAVG, sgFreeAvg);
  EEPROM.get(EE_SG_STALLMIN, sgStallMin);
  EEPROM.get(EE_SG_THR, sgDerivedThrs);
  homingConfidence = EEPROM.read(EE_SG_CONF);
  driver.SGTHRS(sgDerivedThrs);
  delay(15);
  sgCalibrated = true;
  return true;
}

void saveSGCalibration() {
  EEPROM.write(EE_SG_VALID, 0xA5);
  EEPROM.put(EE_SG_FREEAVG, sgFreeAvg);
  EEPROM.put(EE_SG_STALLMIN, sgStallMin);
  EEPROM.put(EE_SG_THR, sgDerivedThrs);
  EEPROM.write(EE_SG_CONF, homingConfidence);
}


/* ===================== DISPLAY ===================== */
void drawSGBar(int y) {
  int w = map(lastSG, 0, 1023, 0, 120);
  const char* zone =
    lastSG > sgDerivedThrs + 150 ? "SAFE" :
    lastSG > sgDerivedThrs + 40  ? "WARN" : "STALL";

  display.setCursor(0, y);
  display.print("SG ");
  display.print(zone);
  display.drawRect(30, y, 98, 6, SSD1306_WHITE);
  display.fillRect(31, y + 1, w, 4, SSD1306_WHITE);
}

void updateDisplay() {

  if (!oledEnabled) return;
  if (millis() - lastDisplayUpdate < DISPLAY_INTERVAL) return;
  lastDisplayUpdate = millis();

  display.clearDisplay();

  if (showCalSummary && millis() - calSummaryStart < CAL_SUMMARY_TIME) {
    display.setCursor(0, 0);
    display.println("CALIBRATION OK");
    display.print("SG avg: "); display.println(sgFreeAvg);
    display.print("SG min: "); display.println(sgStallMin);
    display.print("SGTHRS: "); display.println(sgDerivedThrs);
    display.print("Conf: "); display.print(homingConfidence); display.print("%");
    display.display();
    return;
  }

  long pos = stepper->getCurrentPosition();

  display.setCursor(0, 0);
  display.print("Pos ");
  display.print(stepsToMM(pos), 1);
  display.print(" mm");

  display.setCursor(0, 10);
  display.print("Mode ");
  display.print(
    mode == HOMING  ? "HOME" :
    mode == MOVING  ? "GOTO" :
    mode == JOGGING ? "JOG"  :
    mode == STOPPING ? "STOP" : "IDLE"
  );

  if (mode == MOVING && moveTargetPos != moveStartPos) {
    long total = abs(moveTargetPos - moveStartPos);
    long done  = abs(pos - moveStartPos);
    float pct = total > 0 ? (float)done / total : 1.0;
    pct = constrain(pct, 0.0, 1.0);

    display.drawRect(0, 22, 128, 6, SSD1306_WHITE);
    display.fillRect(1, 23, (int)(126 * pct), 4, SSD1306_WHITE);
  }

  drawSGBar(32);
  display.display();
}


/* ===================== HOMING ===================== */
void homeAxis() {

  DBG_I("Checking the existing SGTHRS value...");
  uint8_t sgt = driver.SGTHRS();
  delay(15);
  DBG_I("SGTHRS readback = " + String(sgt));
  if(sgt > 200){
    DBG_I("Setting a new SGTHRS value...");
    driver.SGTHRS(50);
    delay(20);
    sgt = driver.SGTHRS();
    DBG_I("SGTHRS is now  = " + String(sgt));
    delay(50);
  }
  
  DBG_I("======= BEGIN HOME AXIS! ========");
  stallDetected = false;
  
  // Phase 1: Fast seek
  stepper->setSpeedInHz(HOME_FAST_SPEED);
  stepper->setAcceleration(HOME_FAST_ACCEL);
  driver.irun(RUN_IRUN);
  delay(15);
  driver.TCOOLTHRS(0);
  delay(15);

  DBG_I("COARSE Move 1a to bottom STARTED.");
  stepper->moveTo(-200000);
  while (stepper->isRunning());
  DBG_I("-- COARSE Move 1a STOPPED.");
  
  // Phase 2: Slow refine + StallGuard
  stepper->setSpeedInHz(HOME_SLOW_SPEED);
  stepper->setAcceleration(HOME_SLOW_ACCEL);
  driver.irun(HOME_IRUN);
  delay(15);
  
  driver.TCOOLTHRS(0xFFFFF);
  delay(15);

  uint32_t sgSum = 0;
  uint16_t sgCount = 0;
  sgStallMin = 1023;
  stallDetected = false;

  DBG_I("Beginnning FINE Move 1b to bottom.");
  stepper->moveTo(-400000);
  while (!stallDetected && stepper->isRunning()) {
    lastSG = driver.SG_RESULT();
    sgSum += lastSG;
    sgCount++;
    if (lastSG < sgStallMin) sgStallMin = lastSG;
  }
  stepper->stopMove();
  while (stepper->isRunning());

  DBG_I("-- FINE Move 1b stopped.");
  DBG_I("stallDetected? " + String(stallDetected));
  DBG_I("maxPos = " + String(maxPos));

  stepper->move(HOME_BACKOFF);
  while (stepper->isRunning());

  DBG_I("SETTING current position to zero.");
  stepper->setCurrentPosition(0);
  minPos = 0;
  //================================
  // Auto-calibration
  if (!sgCalibrated && sgCount > 10) {
    sgFreeAvg = sgSum / sgCount;
    sgDerivedThrs = constrain((sgFreeAvg + sgStallMin) / 2, 20, 200);
    driver.SGTHRS(sgDerivedThrs);
    delay(15);
    homingConfidence = constrain(map(sgFreeAvg - sgStallMin, 50, 500, 40, 100), 0, 100);
    sgCalibrated = true;
    saveSGCalibration();
    showCalSummary = true;
    calSummaryStart = millis();
  }

  // Find max
  DBG_I("Fine Move 2a to top STARTED.");

  stallDetected = false;
  stepper->moveTo(400000);
  while (!stallDetected && stepper->isRunning());
  stepper->stopMove();
  while (stepper->isRunning());
  DBG_I("-- COARSE Move 1b STOPPED (no more moves).");
  
  maxPos = stepper->getCurrentPosition();
  DBG_I("stallDetected? " + String(stallDetected));
  DBG_I("maxPos = " + String(maxPos));
  
  EEPROM.put(EE_MAXPOS, maxPos);
  EEPROM.write(EE_HOMED, 1);

  driver.irun(RUN_IRUN);
  stepper->setSpeedInHz(FAST_SPEED);
  stepper->setAcceleration(FAST_ACCEL);

  mode = IDLE;
  
  //Clear any button presses during the moves.
  rehomeTriggered = false;
  
  DBG_I("===== HOME AXIS COMPLETE ==============");

}

void dumpTMC2209() {
  Serial.println(F("===== TMC2209 CONFIG DUMP ====="));

  Serial.print(F("GCONF: 0x"));
  Serial.println(driver.GCONF(), HEX);

  Serial.print(F("IHOLD_IRUN: 0x"));
  Serial.println(driver.IHOLD_IRUN(), HEX);
  Serial.print(F("  IHOLD: "));
  Serial.println(driver.ihold());
  Serial.print(F("  IRUN: "));
  Serial.println(driver.irun());
  Serial.print(F("  IHOLDDELAY: "));
  Serial.println(driver.iholddelay());

  Serial.print(F("TPOWERDOWN: "));
  Serial.println(driver.TPOWERDOWN());

  Serial.print(F("TPWMTHRS: "));
  Serial.println(driver.TPWMTHRS());

  Serial.print(F("TCOOLTHRS: "));
  Serial.println(driver.TCOOLTHRS());

  Serial.print(F("SGTHRS: "));
  Serial.println(driver.SGTHRS());

  Serial.print(F("CHOPCONF: 0x"));
  Serial.println(driver.CHOPCONF(), HEX);

  Serial.print(F("PWMCONF: 0x"));
  Serial.println(driver.PWMCONF(), HEX);

  Serial.print(F("DRV_STATUS: 0x"));
  Serial.println(driver.DRV_STATUS(), HEX);

  Serial.print(F("IOIN: 0x"));
  Serial.println(driver.IOIN(), HEX);

  Serial.print(F("GSTAT: 0x"));
  Serial.println(driver.GSTAT(), HEX);

  Serial.println(F("================================"));
}

/* ===================== SETUP ===================== */
void setup() {

  Serial.begin(115200);


  DBG_I("===== SETUP START =====");
  TMC_SERIAL.begin(115200);

  //pinMode(DIAG_PIN, INPUT_PULLUP);   // force it high if nothing is pulling it down
  pinMode(DIAG_PIN, INPUT);   // force it high if nothing is pulling it down
  delay(10);
  Serial.print("DIAG PIN reads: ");
  Serial.println(digitalRead(DIAG_PIN));  // should print 1 if it can rise

  delay(1000);

  
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(REHOME_PIN, INPUT_PULLUP);
  pinMode(STORE_PIN, INPUT_PULLUP);
  for (int i = 0; i < 5; i++) pinMode(presetPins[i], INPUT_PULLUP);

  DBG_I("Attaching interrupts...");
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REHOME_PIN), rehomeISR, FALLING);

  DBG_I("Initializing TMC2209...");
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);

  DBG_I("Initializing UART...");
  driver.begin();

  DBG_I("Reading TMC2209 version...");
  uint8_t version = driver.version();
  
  if (version != 0x21) {
      DBG_E("ERROR: TMC2209 not responding on UART");
    } 
    else {
      DBG_I("TMC2209 detected OK");
    }

 
  driver.pdn_disable(true);
  delay(15);
  driver.toff(5);
  delay(15);
  driver.irun(RUN_IRUN);
  delay(15);
  
  driver.en_spreadCycle(true);
  delay(15);
  driver.pwm_autoscale(false); // Disable StealthChop
  delay(15);
  driver.TCOOLTHRS(0xFFFFF);
  delay(15);
  driver.GSTAT(0x7);  // clear reset/uvcp/otpw latches (writing 1 clears)
  delay(15);

  
  // - in case we don't have a good config yet.
  //Then try 5, 10, 20, 30, 40, 60 while homing into a hard stop (with a safe current/speed).
  //driver.SGTHRS(10);   // start here
  driver.SGTHRS(25);   // start here
  delay(15);

  DBG_I("loadSGCalibration...");
  loadSGCalibration();

  //SHOW DRIVER CONFIGURATION:
  DBG_I("Checking the existing SGTHRS value...");
  uint8_t sgt = driver.SGTHRS();
  delay(15);

  DBG_I("Wire.begin...");
  Wire.begin();
  DBG_I("display.begin");
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  DBG_I("Loading StallGuard calibration from EEPROM");
  if (EEPROM.read(EE_HOMED)) {
    DBG_I("EE_HOMED, so getting the values...");
    DBG_I("--Getting MAX position");
    EEPROM.get(EE_MAXPOS, maxPos);
    DBG_I("--Loading Presets");
    EEPROM.get(EE_PRESETS, presets);
    DBG_I("--Getting last position");
    long last; EEPROM.get(EE_LASTPOS, last);
    DBG_I("Setting initial position to last known position");
    stepper->setCurrentPosition(last);
    mode = IDLE;
  }
  else{
    DBG_I("StallGuard not yet calibrated?  EE_HOMED was not in EEPROM");
  }
  
  DBG_I("===== SETUP END =====");
  
  dumpTMC2209();
    
}

/* ===================== LOOP ===================== */
void loop() {

 
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'o') oledEnabled = !oledEnabled;
    if (c == 'r') EEPROM.write(EE_SG_VALID, 0x00);
  }

  //Troubleshooting interrupts.   :(
  if (DEBUG_I){
    if(stallDetected && !stallDetected_Displayed){
      DBG_I("****************** STALL ISR HIT *********************");
      stallDetected_Displayed = true;
    }
  
    if(estopTriggered && !estopTriggered_Displayed){
      DBG_I("****************** E-STOP ISR HIT *********************");
      estopTriggered_Displayed = true; 
    }
  
    if(rehomeTriggered && !rehomeTriggered_Displayed){
      DBG_I("****************** REHOME ISR HIT *********************");
      rehomeTriggered_Displayed = true;
    }
  }
  
  if (estopTriggered) {
    DBG_I("BUTTON - ESTOP triggered");
    estopTriggered = false;
    stepper->stopMove();
    mode = STOPPING;
  }

  if (rehomeTriggered) {
    DBG_I("BUTTON - Rehome requested");
    rehomeTriggered = false;
    stepper->stopMove();
    mode = HOMING;
  }

  if (mode == HOMING) homeAxis();
  if (mode == STOPPING && !stepper->isRunning()) mode = IDLE;

  int joy = analogRead(JOYSTICK_Y) - 512;
  if (abs(joy) > JOG_DEADBAND && mode == IDLE) {
    stepper->setSpeedInHz(
      map(abs(joy), 0, 512, JOG_MIN_SPEED, FAST_SPEED));
    long delta = joy > 0 ? JOG_CHUNK : -JOG_CHUNK;
    if (withinLimits(stepper->getCurrentPosition() + delta)) {
      moveStartPos = stepper->getCurrentPosition();
      moveTargetPos = moveStartPos + delta;
      stepper->move(delta);
      mode = JOGGING;
    }
  }

  if (mode == JOGGING && abs(joy) <= JOG_DEADBAND) {
    stepper->stopMove();
    DBG_I("---Jog stopped---");
    mode = IDLE;
  }

  if (mode == MOVING && !stepper->isRunning()) {
    DBG_I("---Move complete---");
    long pos = stepper->getCurrentPosition();
    EEPROM.put(EE_LASTPOS, pos);
    moveStartPos = pos;
    moveTargetPos = pos;
    activePreset = -1;
    mode = IDLE;
  }

  updateDisplay();
}
