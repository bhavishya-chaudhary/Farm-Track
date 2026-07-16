/*
------------------------------------------------------------
 FarmTrack V3
------------------------------------------------------------

 Author      : Bhavishya Chaudhary
 Platform    : Arduino Uno
 Language    : Arduino C++

 Description:
 Firmware for the third iteration of the FarmTrack
 off-road mobile robot.

 Features:
 - Differential drive control
 - Wireless control (FlySky CT6B)
 - EEPROM-based receiver calibration
 - Automatic failsafe
 - Quadratic throttle response
 - Non-linear steering
 - Calibration mode
 - Serial debugging

------------------------------------------------------------
*/

#include <EEPROM.h>

const bool DEBUG_ENABLED = true;

// --------------------------------------------------------
// Receiver Inputs
// --------------------------------------------------------
//
// Receiver channel assignments:
//
// CH1 -> A0 -> VR-B
// CH2 -> A1 -> VR-A
// CH3 -> A2 -> Left Stick (Horizontal)
// CH4 -> A3 -> Left Stick (Vertical)
// CH5 -> A4 -> Right Stick (Vertical)
// CH6 -> A5 -> Right Stick (Horizontal)

const int PIN_STEERING = A5;  // CH6 - Right Stick (Horizontal)
const int PIN_ARM      = A4;  // CH5 - Right Stick (Vertical)
const int PIN_THROTTLE = A3;  // CH4 - Left Stick (Vertical)
const int PIN_GRIPPER  = A2;  // CH3 - Left Stick (Horizontal)
const int PIN_CH5      = A1;  // CH2 - VR-A
const int PIN_CH6      = A0;  // CH1 - VR-B

// --------------------------------------------------------
// Motor Drivers
// --------------------------------------------------------

// Left BTS7960 Motor Driver
const int L_RPWM = 6;
const int L_LPWM = 5;

// Right BTS7960 Motor Driver
const int R_RPWM = 10;
const int R_LPWM = 11;

// L298N Arm Motor Driver
const int ARM_ENA = 9;
const int ARM_IN1 = 8;
const int ARM_IN2 = 7;

// L298N Gripper Motor Driver
const int GRP_ENB = 3;
const int GRP_IN3 = 4;
const int GRP_IN4 = 2;

// Button and Status LED pins
const int PIN_BTN = 12;
const int PIN_LED = 13;

// Calibration and Control thresholds
const int THR_DEAD     = 15;
const int STR_DEAD     = 10;
const int ARM_DEAD     = 20;
const int GRP_DEAD     = 20;
const int ARM_MAX_SPD  = 200;
const int GRP_MAX_SPD  = 180;
const int CAL_MARGIN   = 10;

// Timing constants (milliseconds)
const unsigned long FAILSAFE_MS = 500;
const unsigned long CAL_TIMEOUT = 30000;
const unsigned long LONG_PRESS  = 2000;
const unsigned long BLINK_MS    = 250;
const unsigned long DEBUG_MS    = 100;

// --------------------------------------------------------
// EEPROM
// --------------------------------------------------------
const int  EE_MAGIC_ADDR = 0;
const byte EE_MAGIC      = 0xB6;
const int  EE_DATA_ADDR  = 1;

// Receiver calibration data
struct CalData {
  int thrMin, thrMax;
  int strMin, strMax;
  int armMin, armMax;
  int grpMin, grpMax;
};
CalData cal = {1000, 2000, 1000, 2000, 1000, 2000, 1000, 2000};

// --------------------------------------------------------
// Receiver Processing
// --------------------------------------------------------
volatile unsigned long riseThr = 0, riseStr = 0, riseArm = 0, riseGrp = 0;
volatile unsigned long riseCh5 = 0, riseCh6 = 0;
volatile int pulseThr = 1500, pulseStr = 1500, pulseArm = 1500, pulseGrp = 1500;
volatile int pulseCh5 = 1500, pulseCh6 = 1500;
volatile bool newThr = false, newStr = false, newArm = false, newGrp = false;
volatile bool newCh5 = false, newCh6 = false;
volatile uint8_t prevPort = 0;

// Runtime state
enum Mode { RUN, CAL };
Mode mode = RUN;
unsigned long sigTime = 0, dbgTime = 0, blinkTime = 0, calStartT = 0;
unsigned long calPrintTime = 0;
bool btnDown = false, btnLong = false, calWaitRel = false, ledState = false;
unsigned long btnTime = 0;

int cArmMin, cArmMax, cGrpMin, cGrpMax;
int cThrMin, cThrMax, cStrMin, cStrMax;

int rawThr, rawStr, rawArm, rawGrp, rawCh5, rawCh6;

// Read PWM pulse widths from all receiver channels.
ISR(PCINT1_vect) {
  uint8_t cur = PINC;
  uint8_t chg = cur ^ prevPort;
  unsigned long t = micros();
  prevPort = cur;

  if (chg & (1 << PC5)) {
    if (cur & (1 << PC5)) riseStr = t;
    else { unsigned long w = t - riseStr; if (w >= 800 && w <= 2200) { pulseStr = w; newStr = true; } }
  }
  if (chg & (1 << PC4)) {
    if (cur & (1 << PC4)) riseArm = t;
    else { unsigned long w = t - riseArm; if (w >= 800 && w <= 2200) { pulseArm = w; newArm = true; } }
  }
  if (chg & (1 << PC3)) {
    if (cur & (1 << PC3)) riseThr = t;
    else { unsigned long w = t - riseThr; if (w >= 800 && w <= 2200) { pulseThr = w; newThr = true; } }
  }
  if (chg & (1 << PC2)) {
    if (cur & (1 << PC2)) riseGrp = t;
    else { unsigned long w = t - riseGrp; if (w >= 800 && w <= 2200) { pulseGrp = w; newGrp = true; } }
  }
  if (chg & (1 << PC1)) {
    if (cur & (1 << PC1)) riseCh5 = t;
    else { unsigned long w = t - riseCh5; if (w >= 800 && w <= 2200) { pulseCh5 = w; newCh5 = true; } }
  }
  if (chg & (1 << PC0)) {
    if (cur & (1 << PC0)) riseCh6 = t;
    else { unsigned long w = t - riseCh6; if (w >= 800 && w <= 2200) { pulseCh6 = w; newCh6 = true; } }
  }
}

// Map a receiver channel using its calibrated limits.
int mapChannel(int raw, int cMin, int cMax) {
  int cCtr = (cMin + cMax) / 2;
  int out;
  if (raw >= cCtr) {
    out = map(raw, cCtr, cMax, 0, 255);
  } else {
    out = map(raw, cMin, cCtr, -255, 0);
  }
  return constrain(out, -255, 255);
}

// Print stored calibration values.
void printCal() {
  Serial.println(F("Stored calibration (min | center | max):"));
  int c;
  c = (cal.thrMin + cal.thrMax) / 2;
  Serial.print(F("  THR: ")); Serial.print(cal.thrMin);
  Serial.print(F(" | ")); Serial.print(c);
  Serial.print(F(" | ")); Serial.println(cal.thrMax);

  c = (cal.strMin + cal.strMax) / 2;
  Serial.print(F("  STR: ")); Serial.print(cal.strMin);
  Serial.print(F(" | ")); Serial.print(c);
  Serial.print(F(" | ")); Serial.println(cal.strMax);

  c = (cal.armMin + cal.armMax) / 2;
  Serial.print(F("  ARM: ")); Serial.print(cal.armMin);
  Serial.print(F(" | ")); Serial.print(c);
  Serial.print(F(" | ")); Serial.println(cal.armMax);

  c = (cal.grpMin + cal.grpMax) / 2;
  Serial.print(F("  GRP: ")); Serial.print(cal.grpMin);
  Serial.print(F(" | ")); Serial.print(c);
  Serial.print(F(" | ")); Serial.println(cal.grpMax);
}

// Load calibration from EEPROM.
void loadCal() {
  if (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC) {
    EEPROM.get(EE_DATA_ADDR, cal);
    if (cal.thrMin < 800 || cal.thrMax > 2200 ||
        cal.strMin < 800 || cal.strMax > 2200 ||
        cal.armMin < 800 || cal.armMax > 2200 ||
        cal.grpMin < 800 || cal.grpMax > 2200 ||
        cal.thrMax - cal.thrMin < 100 ||
        cal.strMax - cal.strMin < 100) {
      cal = {1000, 2000, 1000, 2000, 1000, 2000, 1000, 2000};
      if (DEBUG_ENABLED) Serial.println(F("EEPROM data invalid. Defaults loaded."));
    } else {
      if (DEBUG_ENABLED) Serial.println(F("Calibration data successfully loaded from EEPROM."));
    }
  } else {
    if (DEBUG_ENABLED) Serial.println(F("No saved calibration found. Loading default constants."));
  }
}

// Save calibration to EEPROM.
void saveCal() {
  EEPROM.put(EE_MAGIC_ADDR, EE_MAGIC);
  EEPROM.put(EE_DATA_ADDR, cal);
}

// Read the latest receiver values and update the failsafe timer.
void readRC(int &thr, int &str, int &arm, int &grp,
            int &ch5, int &ch6, bool &fail) {
  noInterrupts();
  thr = pulseThr;  str = pulseStr;
  arm = pulseArm;  grp = pulseGrp;
  ch5 = pulseCh5;  ch6 = pulseCh6;
  bool got = newThr || newStr || newArm || newGrp || newCh5 || newCh6;
  newThr = newStr = newArm = newGrp = newCh5 = newCh6 = false;
  interrupts();

  if (got) sigTime = millis();
  fail = (millis() - sigTime > FAILSAFE_MS);
}

// --------------------------------------------------------
// Calibration
// --------------------------------------------------------

// Enter calibration mode.
void calBegin() {
  mode = CAL;
  calStartT = millis();
  calPrintTime = millis();
  stopAll();
  calWaitRel = true;

  noInterrupts();
  int t = pulseThr; int s = pulseStr;
  int a = pulseArm; int g = pulseGrp;
  interrupts();

  cThrMin = t; cThrMax = t;
  cStrMin = s; cStrMax = s;
  cArmMin = a; cArmMax = a;
  cGrpMin = g; cGrpMax = g;

  if (DEBUG_ENABLED) {
    Serial.println(F("Calibration mode"));
    Serial.println(F("Move every transmitter control through its full range."));
    Serial.println(F("Press the button again to save."));
  }
}

// Update calibration limits.
void calUpdate() {
  noInterrupts();
  int t = pulseThr; int s = pulseStr;
  int a = pulseArm; int g = pulseGrp;
  interrupts();

  if (t < cThrMin) cThrMin = t;  if (t > cThrMax) cThrMax = t;
  if (s < cStrMin) cStrMin = s;  if (s > cStrMax) cStrMax = s;
  if (a < cArmMin) cArmMin = a;  if (a > cArmMax) cArmMax = a;
  if (g < cGrpMin) cGrpMin = g;  if (g > cGrpMax) cGrpMax = g;

  // Print progress dynamically to serial output
  if (DEBUG_ENABLED && millis() - calPrintTime >= 1000) {
    calPrintTime = millis();
    Serial.print(F("  THR:")); Serial.print(cThrMin);
    Serial.print(F("-")); Serial.print(cThrMax);
    Serial.print(F(" STR:")); Serial.print(cStrMin);
    Serial.print(F("-")); Serial.print(cStrMax);
    Serial.print(F(" ARM:")); Serial.print(cArmMin);
    Serial.print(F("-")); Serial.print(cArmMax);
    Serial.print(F(" GRP:")); Serial.print(cGrpMin);
    Serial.print(F("-")); Serial.println(cGrpMax);
  }
}

// Validate and save calibration.
void calFinish() {
  bool ok = (cThrMax - cThrMin >= 200) && (cStrMax - cStrMin >= 200);

  if (ok) {
    cal.thrMin = cThrMin + CAL_MARGIN;
    cal.thrMax = cThrMax - CAL_MARGIN;
    cal.strMin = cStrMin + CAL_MARGIN;
    cal.strMax = cStrMax - CAL_MARGIN;

    if (cArmMax - cArmMin >= 100) {
      cal.armMin = cArmMin + CAL_MARGIN;
      cal.armMax = cArmMax - CAL_MARGIN;
    }
    if (cGrpMax - cGrpMin >= 100) {
      cal.grpMin = cGrpMin + CAL_MARGIN;
      cal.grpMax = cGrpMax - CAL_MARGIN;
    }

    saveCal();
    if (DEBUG_ENABLED) {
      Serial.println(F("Calibration completed successfully."));
      printCal();
    }
  } else {
    if (DEBUG_ENABLED) {
      Serial.println(F("Calibration failed."));
      Serial.print(F("  Throttle range: ")); Serial.println(cThrMax - cThrMin);
      Serial.print(F("  Steering range: ")); Serial.println(cStrMax - cStrMin);
    }
  }

  mode = RUN;
  digitalWrite(PIN_LED, HIGH);
  btnDown = false;
}

// --------------------------------------------------------
// Drive Control
// --------------------------------------------------------

// Apply deadband and scale the remaining stick travel.
int dzRamp(int val, int dz, int maxOut) {
  if (abs(val) <= dz) return 0;
  int sign = (val > 0) ? 1 : -1;
  int out = map(abs(val), dz + 1, 255, 1, maxOut);
  return constrain(out, 1, maxOut) * sign;
}

// Differential drive output.
void driveMotors(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  if (left > 0)       { analogWrite(L_RPWM, left);  analogWrite(L_LPWM, 0);      }
  else if (left < 0)  { analogWrite(L_RPWM, 0);     analogWrite(L_LPWM, -left);   }
  else                 { analogWrite(L_RPWM, 0);     analogWrite(L_LPWM, 0);       }

  if (right > 0)      { analogWrite(R_RPWM, right); analogWrite(R_LPWM, 0);       }
  else if (right < 0) { analogWrite(R_RPWM, 0);     analogWrite(R_LPWM, -right);  }
  else                 { analogWrite(R_RPWM, 0);     analogWrite(R_LPWM, 0);       }
}

// Drive the arm motor.
void driveArm(int spd) {
  spd = constrain(spd, -ARM_MAX_SPD, ARM_MAX_SPD);
  if (spd > 0) {
    digitalWrite(ARM_IN1, HIGH); digitalWrite(ARM_IN2, LOW);
    analogWrite(ARM_ENA, spd);
  } else if (spd < 0) {
    digitalWrite(ARM_IN1, LOW);  digitalWrite(ARM_IN2, HIGH);
    analogWrite(ARM_ENA, -spd);
  } else {
    digitalWrite(ARM_IN1, LOW);  digitalWrite(ARM_IN2, LOW);
    analogWrite(ARM_ENA, 0);
  }
}

// Drive the gripper motor.
void driveGripper(int spd) {
  spd = constrain(spd, -GRP_MAX_SPD, GRP_MAX_SPD);
  if (spd > 0) {
    digitalWrite(GRP_IN3, HIGH); digitalWrite(GRP_IN4, LOW);
    analogWrite(GRP_ENB, spd);
  } else if (spd < 0) {
    digitalWrite(GRP_IN3, LOW);  digitalWrite(GRP_IN4, HIGH);
    analogWrite(GRP_ENB, -spd);
  } else {
    digitalWrite(GRP_IN3, LOW);  digitalWrite(GRP_IN4, LOW);
    analogWrite(GRP_ENB, 0);
  }
}

// Stop functions.
void stopMotors()  { analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
                     analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0); }
void stopArm()     { digitalWrite(ARM_IN1, LOW); digitalWrite(ARM_IN2, LOW);
                     analogWrite(ARM_ENA, 0); }
void stopGripper() { digitalWrite(GRP_IN3, LOW); digitalWrite(GRP_IN4, LOW);
                     analogWrite(GRP_ENB, 0); }
void stopAll()     { stopMotors(); stopArm(); stopGripper(); }

// Handle the calibration button.
void handleButton() {
  bool pressed = (digitalRead(PIN_BTN) == LOW);
  unsigned long now = millis();

  if (mode == RUN) {
    if (pressed && !btnDown) {
      btnDown = true; btnTime = now; btnLong = false;
      digitalWrite(PIN_LED, LOW);
    }
    if (pressed && btnDown && !btnLong && now - btnTime >= LONG_PRESS) {
      btnLong = true;
      calBegin();
    }
    if (!pressed && btnDown) {
      btnDown = false;
      if (!btnLong) digitalWrite(PIN_LED, HIGH);
    }
  }
  else if (mode == CAL) {
    if (now - blinkTime >= BLINK_MS) {
      blinkTime = now; ledState = !ledState;
      digitalWrite(PIN_LED, ledState);
    }
    if (pressed && !btnDown) btnDown = true;
    if (!pressed && btnDown) {
      btnDown = false;
      if (calWaitRel) calWaitRel = false;
      else calFinish();
    }
    if (now - calStartT >= CAL_TIMEOUT) {
      if (DEBUG_ENABLED) Serial.println(F("Calibration timed out."));
      calFinish();
    }
  }
}

// --------------------------------------------------------
// Debug Utilities
// --------------------------------------------------------

// Print receiver and motor diagnostics.
void debugPrint(int mThr, int mStr, int mArm, int mGrp,
                int thr, int str, int arm, int grip,
                int lSpd, int rSpd, bool fail) {
  if (!DEBUG_ENABLED) return;
  if (millis() - dbgTime < DEBUG_MS) return;
  dbgTime = millis();

  if (fail) { Serial.println(F("!!! FAILSAFE !!!")); return; }

  Serial.print(F("T:")); Serial.print(mThr);
  Serial.print(F(">")); Serial.print(thr);
  Serial.print(F(" S:")); Serial.print(mStr);
  Serial.print(F(">")); Serial.print(str);
  Serial.print(F(" A:")); Serial.print(mArm);
  Serial.print(F(">")); Serial.print(arm);
  Serial.print(F(" G:")); Serial.print(mGrp);
  Serial.print(F(">")); Serial.print(grip);
  Serial.print(F(" L:")); Serial.print(lSpd);
  Serial.print(F(" R:")); Serial.print(rSpd);
  Serial.print(F(" raw["));
  Serial.print(rawStr);  Serial.print(F(","));
  Serial.print(rawArm);  Serial.print(F(","));
  Serial.print(rawThr);  Serial.print(F(","));
  Serial.print(rawGrp);  Serial.print(F(","));
  Serial.print(rawCh5);  Serial.print(F(","));
  Serial.print(rawCh6);
  Serial.println(F("]"));
}

// Display raw receiver values during startup.
void rawMonitor() {
  Serial.println(F("\nStarting 3-second raw input monitor. Move sticks to verify signal connection..."));
  unsigned long start = millis();
  while (millis() - start < 3000) {
    noInterrupts();
    int t = pulseThr; int s = pulseStr;
    int a = pulseArm; int g = pulseGrp;
    int c5 = pulseCh5; int c6 = pulseCh6;
    interrupts();

    Serial.print(F("CH1:")); Serial.print(s);
    Serial.print(F(" CH2:")); Serial.print(a);
    Serial.print(F(" CH3:")); Serial.print(t);
    Serial.print(F(" CH4:")); Serial.print(g);
    Serial.print(F(" CH5:")); Serial.print(c5);
    Serial.print(F(" CH6:")); Serial.println(c6);
    delay(200);
  }
  Serial.println(F("Raw input monitor complete.\n"));
}

// --------------------------------------------------------
// System setup
// --------------------------------------------------------
void setup() {
  if (DEBUG_ENABLED) {
    Serial.begin(115200);
    Serial.println(F("=========================================="));
    Serial.println(F(" FarmTrack V3 Controller"));
    Serial.println(F(" Firmware v1.0"));
    Serial.println(F("=========================================="));
  }

  pinMode(PIN_STEERING, INPUT); pinMode(PIN_ARM, INPUT);
  pinMode(PIN_THROTTLE, INPUT); pinMode(PIN_GRIPPER, INPUT);
  pinMode(PIN_CH5, INPUT);      pinMode(PIN_CH6, INPUT);

  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  pinMode(ARM_ENA, OUTPUT); pinMode(ARM_IN1, OUTPUT); pinMode(ARM_IN2, OUTPUT);
  pinMode(GRP_ENB, OUTPUT); pinMode(GRP_IN3, OUTPUT); pinMode(GRP_IN4, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  stopAll();
  loadCal();

  prevPort = PINC;
  PCICR  |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) |
             (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);

  sigTime = millis();
  digitalWrite(PIN_LED, HIGH);

  if (DEBUG_ENABLED) {
    Serial.println(F("Receiver mapping:"));
    Serial.println(F("  CH1 (VR-B)                 -> A0"));
    Serial.println(F("  CH2 (VR-A)                 -> A1"));
    Serial.println(F("  CH3 (Left Horizontal)      -> A2"));
    Serial.println(F("  CH4 (Left Vertical)        -> A3"));
    Serial.println(F("  CH5 (Right Vertical)       -> A4"));
    Serial.println(F("  CH6 (Right Horizontal)     -> A5"));
    printCal();
    rawMonitor();
    Serial.println(F("System ready."));
  }
}

// --------------------------------------------------------
// Main control loop
// --------------------------------------------------------
void loop() {
  handleButton();

  if (mode == CAL) {
    calUpdate();
    stopAll();
    return;
  }

  bool fail;
  readRC(rawThr, rawStr, rawArm, rawGrp, rawCh5, rawCh6, fail);

  if (fail) {
    stopAll();
    debugPrint(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, true);
    return;
  }

  int mThr = mapChannel(rawThr, cal.thrMin, cal.thrMax);
  int mStr = mapChannel(rawStr, cal.strMin, cal.strMax);
  int mArm = mapChannel(rawArm, cal.armMin, cal.armMax);
  int mGrp = mapChannel(rawGrp, cal.grpMin, cal.grpMax);

  int throttle  = dzRamp(mThr, THR_DEAD, 255);
  int steering  = dzRamp(mStr, STR_DEAD, 255);
  int armSpeed  = dzRamp(mArm, ARM_DEAD, ARM_MAX_SPD);
  int gripSpeed = dzRamp(mGrp, GRP_DEAD, GRP_MAX_SPD);

  int leftSpd  = constrain(throttle + steering, -255, 255);
  int rightSpd = constrain(throttle - steering, -255, 255);

  driveMotors(leftSpd, rightSpd);
  driveArm(armSpeed);
  driveGripper(gripSpeed);

  debugPrint(mThr, mStr, mArm, mGrp,
             throttle, steering, armSpeed, gripSpeed,
             leftSpd, rightSpd, fail);
}