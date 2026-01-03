/*
  Motorized Photo Stand Controller
  Board: Elegoo MEGA 2560 R3
  Driver: TMC2209 (UART + StallGuard)
  Display: SSD1306 I2C 128x64
*/

#include <AccelStepper.h>
#include <TMCStepper.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define DEBUG_V 0                                                           
#define DEBUG_I 1
#define DEBUG_E 1

/* ===================== PIN ASSIGNMENTS ===================== */
#define STEP_PIN        46
#define DIR_PIN         48
#define ENABLE_PIN      44

#define DIAG_PIN        19   // INT4
#define ESTOP_BUTTON     2   // INT0
#define REHOME_BUTTON    3   // INT1

#define STORE_BUTTON     4
const uint8_t presetPins[5] = {5, 6, 7, 8, 9};

#define JOYSTICK_Y      A0

/* ===================== OLED ===================== */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* ===================== STEPPER ===================== */
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

/* ===================== TMC2209 ===================== */
#define R_SENSE 0.11f
#define DRIVER_ADDRESS 0b00
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

//Can also try 0b10 or 0b11
//TMC2209Stepper driver(&Serial1, R_SENSE, 0b10);
//TMC2209Stepper driver(&Serial1, R_SENSE, 0b11);

/* ===================== MECHANICS ===================== */
#define STEPS_PER_REV   200
#define MICROSTEPS     16
#define LEADSCREW_MM    8.0   // mm per revolution

/* ===================== MOTION DEBUG VALUES ===================== */
#define FAST_SPEED    2000.0
#define FAST_ACCEL    1500.0
#define STALL_SPEED   300.0
#define STALL_ACCEL    200.0
#define JOG_DEADBAND     50
#define JOG_MIN_SPEED  200.0


/* ===================== MOTION ===================== */
//#define FAST_SPEED    8000.0
//#define FAST_ACCEL    6000.0
//#define STALL_SPEED   1200.0
//#define STALL_ACCEL    800.0
//#define JOG_DEADBAND     50
//#define JOG_MIN_SPEED  200.0
//


/* ===================== TMC ===================== */
#define IRUN_VALUE     31
#define IHOLD_VALUE    10
#define IHOLDDELAY      8
#define SGTHRS_VALUE  100

/* ===================== EEPROM ===================== */
#define EE_HOMED       0
#define EE_MAXPOS      1
#define EE_LASTPOS     5
#define EE_PRESETS     9
#define EE_LABELS     40
#define LABEL_LEN     12

/* ===================== STATE ===================== */
enum Mode { HOMING, IDLE, JOGGING, MOVING, STOPPING };
Mode mode = HOMING;

volatile bool stallDetected = false;
volatile bool estopTriggered = false;
volatile bool rehomeTriggered = false;
bool stallWarning = false;
bool crashed = false;

long minPos = 0;
long maxPos = 0;
long presets[5];
char presetLabels[5][LABEL_LEN] = {
  "Top", "Upper", "Middle", "Lower", "Bottom"
};

int activePreset = -1;
long moveStartPos = 0;
long moveTargetPos = 0;

/* ===================== HELPERS ===================== */
float stepsToMM(long s) {
  return (s * LEADSCREW_MM) / (STEPS_PER_REV * MICROSTEPS);
}
float stepsToInches(long s) {
  return stepsToMM(s) / 25.4;
}

/* ===================== ISR ===================== */
void stallISR()  { stallDetected = true; stallWarning = true; }
void estopISR()  { estopTriggered = true; }
void rehomeISR() { rehomeTriggered = true; }

/* ===================== PROFILES ===================== */
void setFastProfile() {
  stepper.setMaxSpeed(FAST_SPEED);
  stepper.setAcceleration(FAST_ACCEL);
  driver.TCOOLTHRS(300);
}
void setStallProfile() {
  stepper.setMaxSpeed(STALL_SPEED);
  stepper.setAcceleration(STALL_ACCEL);
  driver.TCOOLTHRS(0xFFFFF);
}

/* ===================== EEPROM ===================== */
void loadLabels() {
  for (int i = 0; i < 5; i++) {
    EEPROM.get(EE_LABELS + i * LABEL_LEN, presetLabels[i]);
    if (presetLabels[i][0] == 0xFF || presetLabels[i][0] == 0)
      strcpy(presetLabels[i], "Preset");
  }
}
void saveLabel(int i) {
  EEPROM.put(EE_LABELS + i * LABEL_LEN, presetLabels[i]);
}

/* ===================== SERIAL ===================== */
void handleSerial() {

#if DEBUG_V
  Serial.println("handleSerial()");
#endif

  if (!Serial.available()) return;

#if DEBUG_V
  Serial.println("Reading Serial!");
#endif


  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("SETLABEL")) {
    int idx = cmd.substring(8, 9).toInt() - 1;
    if (idx >= 0 && idx < 5) {
      cmd.substring(10).toCharArray(presetLabels[idx], LABEL_LEN);
      saveLabel(idx);
      Serial.println("Label updated");
    }
  }

  if (cmd == "LISTLABELS") {
    for (int i = 0; i < 5; i++) {
      Serial.print(i + 1); Serial.print(": ");
      Serial.println(presetLabels[i]);
    }
  }
}

/* ===================== HOMING ===================== */
void homeAxis() {
#if DEBUG_I
  Serial.println("REHOMING AXIS NOW...");
#endif

#if DEBUG_I
  Serial.println("setStallProfile...");
#endif

  setStallProfile();
  stallDetected = false;

#if DEBUG_I
  Serial.println("stepper.moveTo(-300000) First time...");
#endif

  stepper.moveTo(-300000);
  while (!stallDetected) stepper.run();

#if DEBUG_I
  Serial.println("stepper.stop()");
#endif

  stepper.stop();
  while (stepper.isRunning()) stepper.run();

#if DEBUG_I
  Serial.println("stepper.setCurrentPosition(0)");
#endif

  stepper.setCurrentPosition(0);
  minPos = 0;

  delay(300);
  stallDetected = false;

#if DEBUG_I
  Serial.println("stepper.moveTo(300000) Second time");
#endif

  stepper.moveTo(300000);
  while (!stallDetected) stepper.run();

#if DEBUG_I
  Serial.println("stepper.stop()");
#endif

  stepper.stop();
  while (stepper.isRunning()) stepper.run();

#if DEBUG_I
  Serial.println("set maxPos = stepper.currentPosition()");
#endif

  maxPos = stepper.currentPosition();
  EEPROM.put(EE_MAXPOS, maxPos);
  EEPROM.write(EE_HOMED, 1);

#if DEBUG_I
  Serial.println("setFastProfile()...");
#endif

  setFastProfile();
  mode = IDLE;

#if DEBUG_I
  Serial.println("HOMING COMPLETE!");
#endif

}

/* ===================== PRESETS ===================== */
void goToPreset(int i) {
  moveStartPos = stepper.currentPosition();
  moveTargetPos = presets[i];
  activePreset = i;
  setFastProfile();
  stepper.moveTo(presets[i]);
  mode = MOVING;
}

/* ===================== OLED ===================== */
void updateDisplay() {

#if DEBUG_V
  Serial.println("Update Display...");
#endif

  static unsigned long last = 0;
  if (millis() - last < 120) return;
  last = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if(crashed){
    display.setTextColor(SSD1306_INVERSE);
    display.print("CRASHED");
    return;
  }

  long pos = stepper.currentPosition();

  display.setCursor(0, 0);
  display.print("Pos ");
  display.print(stepsToMM(pos), 1);
  display.print("mm ");
  display.print(stepsToInches(pos), 2);
  display.print("in");

  display.setCursor(0, 10);
  display.print("Mode:");
  display.print(mode == MOVING ? "GOTO" :
                mode == JOGGING ? "JOG" :
                mode == HOMING ? "HOME" : "IDLE");

  display.setCursor(0, 20);
  display.print("Preset:");
  if (activePreset >= 0) display.print(presetLabels[activePreset]);
  else display.print("--");

  if (mode == MOVING && moveTargetPos != moveStartPos) {
    float pct = abs(pos - moveStartPos) / float(abs(moveTargetPos - moveStartPos));
    pct = constrain(pct, 0.0, 1.0);
    display.drawRect(0, 30, 128, 8, SSD1306_WHITE);
    display.fillRect(1, 31, int(126 * pct), 6, SSD1306_WHITE);
  }

  if (stallWarning) {
#if DEBUG_V
  Serial.println("stallWarning!");
#endif
    display.setCursor(0, 42);
    display.print("!! STALL DETECTED !!");
  }

  for (int i = 0; i < 5; i++) {
    display.setCursor(0, 50 + i * 6);
    display.print(i + 1);
    display.print(":");
    display.print(presetLabels[i]);
    if (i == activePreset && ((millis() / 300) % 2)) display.print(" <");
  }

  display.display();
}

/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);

#if DEBUG_I
  Serial.println("Begin setup...");
#endif

  //Setup Phase 1
#if DEBUG_I
  Serial.println("Setup Phase 1...");
#endif

  Serial1.begin(115200);

  //Setup Phase 2
#if DEBUG_I
  Serial.println("Setup Phase 2...");
#endif


  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(DIAG_PIN, INPUT_PULLUP);
  pinMode(ESTOP_BUTTON, INPUT_PULLUP);
  pinMode(REHOME_BUTTON, INPUT_PULLUP);
  pinMode(STORE_BUTTON, INPUT_PULLUP);
  for (int i = 0; i < 5; i++) pinMode(presetPins[i], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ESTOP_BUTTON), estopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(REHOME_BUTTON), rehomeISR, FALLING);

  //Setup Phase 3
#if DEBUG_I
  Serial.println("Setup Phase 3...");
#endif
  driver.begin();
  driver.pdn_disable(true);
  driver.toff(5);
  
  uint32_t drv = driver.DRV_STATUS();
  Serial.print("TMC2209 DRVSTATUS=0x");
  Serial.println(drv, HEX);

  driver.blank_time(24);
  driver.irun(IRUN_VALUE);
  driver.ihold(IHOLD_VALUE);
  driver.iholddelay(IHOLDDELAY);
  driver.microsteps(16);
  driver.en_spreadCycle(true);
  driver.SGTHRS(SGTHRS_VALUE);

  stepper.setEnablePin(ENABLE_PIN);
  stepper.enableOutputs();

  //IS THE DRIVER AWAKE AND RESPONDING VIA uart?
  if (driver.DRV_STATUS() == 0x00000000){
    Serial.println("****************************************************");
    Serial.println("UART FAILED on the TMC2209 board.  Nothing will work.");
    Serial.println("****************************************************");

    crashed = true;    
  }

  
  setFastProfile();

//Setup Phase 4
#if DEBUG_I
  Serial.println("Setup Phase 4...");
#endif

  Wire.begin();

  //original
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  //possible
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3D);

//Setup Phase 5
#if DEBUG_I
  Serial.println("Setup Phase 5...");
#endif

if(!crashed){
  
    loadLabels();
  
    if (EEPROM.read(EE_HOMED) == 1) {
      EEPROM.get(EE_MAXPOS, maxPos);
      long last;
      EEPROM.get(EE_LASTPOS, last);
      stepper.setCurrentPosition(last);
      EEPROM.get(EE_PRESETS, presets);
      mode = IDLE;
    }
  
  }
  else{
    //try to show a message on the display
        
  }
#if DEBUG_I
  Serial.println("End Setup!");
#endif

}

/* ===================== LOOP ===================== */
void loop() {
#if DEBUG_V
  Serial.println("LOOP!");
#endif

  handleSerial();

#if DEBUG_V
  Serial.println("Handled Serial.");
#endif

  if (estopTriggered) {
#if DEBUG_V
  Serial.println("estopTriggered!");
#endif
    estopTriggered = false;
    stepper.stop();
    mode = STOPPING;
    activePreset = -1;
  }

#if DEBUG_V
  Serial.println("After estopTriggered.");
#endif


  if (rehomeTriggered) {
#if DEBUG_V
  Serial.println("rehomeTriggered!");
#endif
    rehomeTriggered = false;
    stepper.stop();
    while (stepper.isRunning()) stepper.run();
    mode = HOMING;
    stallWarning = false;
  }
  
#if DEBUG_V
  Serial.println("After rehomeTriggered.");
#endif

  if (mode == HOMING) homeAxis();

#if DEBUG_V
  Serial.println("After check homing.");
#endif


  if (mode == STOPPING) {
#if DEBUG_V
  Serial.println("mode == STOPPING!");
#endif
    stepper.run();
    if (!stepper.isRunning()) mode = IDLE;
  }

#if DEBUG_V
  Serial.println("After IF STOPPING.");
#endif

  int joy = analogRead(JOYSTICK_Y) - 512;
  if (abs(joy) > JOG_DEADBAND && mode == IDLE) {
    mode = JOGGING;
    float speed = map(abs(joy), 0, 512, JOG_MIN_SPEED, FAST_SPEED);
    stepper.setSpeed(joy > 0 ? speed : -speed);
    stepper.runSpeed();
  } else if (mode == JOGGING) {
    stepper.stop();
    mode = IDLE;
  }

#if DEBUG_V
  Serial.println("After IF Joystick reading.");
#endif

  for (int i = 0; i < 5; i++) {
    if (!digitalRead(presetPins[i])) {
#if DEBUG_V
  Serial.println("pin read!");
#endif
      stepper.stop();
      while (stepper.isRunning()) stepper.run();
      goToPreset(i);
      delay(150);
    }
  }

#if DEBUG_V
  Serial.println("After read pins.");
#endif


  if (!digitalRead(STORE_BUTTON)) {
#if DEBUG_V
  Serial.println("STORE_BUTTON!");
#endif

    presets[activePreset >= 0 ? activePreset : 0] = stepper.currentPosition();
    EEPROM.put(EE_PRESETS, presets);
    delay(300);
  }

#if DEBUG_V
  Serial.println("After Read STORE_BUTTON");
#endif


  if (mode == MOVING) {
    stepper.run();
    if (stepper.distanceToGo() == 0) {
      mode = IDLE;
      activePreset = -1;
      stallWarning = false;
    }
  }

#if DEBUG_V
  Serial.println("After IF MOVING.");
#endif


  stepper.run();
  updateDisplay();
}
