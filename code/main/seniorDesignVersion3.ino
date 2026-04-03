#include <Arduino.h>
#include <Wire.h>
#include "AD5933.h"
#include "A4988.h"

// -----------------------------------------------------------------------------------------
// STEPPER MOTOR CONFIGURATION
#define MOTOR_STEPS 200
#define RPM 120

#define DIR 2
#define STEP 3
#define SLEEP 13

#define MS1 10
#define MS2 11
#define MS3 12

A4988 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, MS1, MS2, MS3);

// -----------------------------------------------------------------------------------------
// AD5933 CONFIGURATION
#define START_FREQ  (3000)
#define FREQ_INCR   (1000)
#define NUM_INCR    (4)

double gain = 1.8089e-7;

AD5933 ad5933;

// -----------------------------------------------------------------------------------------
// MULTIPLEXER CONTROL PINS
#define MUX_A 5
#define MUX_B 6

// -----------------------------------------------------------------------------------------
// STATE MACHINE

enum SystemState {
  STATE_IDLE = 0,
  STATE_ACTIVE = 1,
  STATE_COMPLETE = 2
};
SystemState currentState = STATE_IDLE;

// Sampling mode (100 / 50 microsteps)
enum SamplingMode {
  mode_normal = 0,
  mode_highres = 1
};
SamplingMode currentMode = mode_normal;

// Probe selection
enum Probes {
  probes_8mm = 0,
  probes_12mm = 1
};

// Scan direction
enum ScanDirection {
  scan_forward = +1,
  scan_backward = -1
};

// -----------------------------------------------------------------------------------------
// USER CONTROL VARIABLES

bool startScan = false;
bool scanActive = false;   // allows stopping mid-scan

Probes selectedProbe = probes_8mm;
ScanDirection selectedDirection = scan_forward;

// controls TOTAL DISTANCE ONLY (4.5in each side of slider = 45720 microsteps)
int userDistance = 45720;

// -----------------------------------------------------------------------------------------
// GLOBAL VARIABLES

double currentImpedance = 0.0;
double baselineImpedance = 0.0;
bool firstMeasurement = false;

const double RATE_OF_CHANGE_THRESHOLD = 0.20;

int STEPS_taken = 0;
int highResCounter = 0;

// -----------------------------------------------------------------------------------------
// SETUP

void setup(void) {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("=== Scanner Control ===");
  Serial.println("Modes: f8, f12, b8, b12");
  Serial.println("Enter number (1–9) to set distance");
  Serial.println("s = start, x = stop");

  // ---------------- AD5933 INIT ----------------
  if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
  {
    Serial.println("FAILED in initialization!");
    while (true);
  }

  if (!ad5933.setRange(CTRL_OUTPUT_RANGE_2)) {
    Serial.println("FAILED in setRange!");
    while (true);
  }

  // ---------------- STEPPER INIT ----------------
  stepper.begin(RPM);
  stepper.enable();
  stepper.setMicrostep(1);

  // ---------------- MUX INIT ----------------
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
}

// -----------------------------------------------------------------------------------------
// LOOP

void loop() {
 readSerialCommand();

  if (!startScan) return;

  processState();
  delay(100);
  
}

// -----------------------------------------------------------------------------------------

void readSerialCommand() {

  static String buffer = "";  // strings commands tg : f8

  while (Serial.available()) {
    char c = Serial.read();

    // -------- STOP --------
    if (c == 'x') {
      Serial.println("Scan STOPPED");
      scanActive = false;
      startScan = false;
      currentState = STATE_IDLE;
      buffer = "";
      return;
    }

    // -------- START --------
    if (c == 's') {
      Serial.println("Start Scanning...");
      startScan = true;
      scanActive = true;
      buffer = "";
      return;
    }



    // -------- BUILD COMMAND --------
    buffer += c;
    buffer.trim();

    if (buffer == "f8") {
      selectedDirection = scan_forward;
      selectedProbe = probes_8mm;
      Serial.println("Selected: Forward + 8mm");
      buffer = "";
      return;
    }
    else if (buffer == "f12") {
      selectedDirection = scan_forward;
      selectedProbe = probes_12mm;
      Serial.println("Selected: Forward + 12mm");
      buffer = "";
      return;
    }
    else if (buffer == "b8") {
      selectedDirection = scan_backward;
      selectedProbe = probes_8mm;
      Serial.println("Selected: Backward + 8mm");
      buffer = "";
      return;
    }
    else if (buffer == "b12") {
      selectedDirection = scan_backward;
      selectedProbe = probes_12mm;
      Serial.println("Selected: Backward + 12mm");
      buffer = "";
      return;
    }
    // -------- DISTANCE CONTROL --------
    if (isDigit(c)) {
      buffer == "";
      int val = c - '0';

      // 1 → 1cm; 4000msteps=1cm
      userDistance = val * 4000;

      Serial.print("Distance set to: ");
      Serial.println(userDistance);

      buffer = "";
      return;
    }

    if (buffer.length() > 4) buffer = "";
  }
}

// -----------------------------------------------------------------------------------------
// STATE MACHINE

void processState() {
  switch(currentState) {

    case STATE_IDLE:
      handleIdle();
      break;

    case STATE_ACTIVE:
      handleActive();
      break;

    case STATE_COMPLETE:
      handleComplete();
      break;

    default:
      currentState = STATE_IDLE;
      break;
  }
}

// -----------------------------------------------------------------------------------------
// IDLE

void handleIdle() {
  if (!startScan) return;

  if (selectedDirection == scan_forward) {
    STEPS_taken = 0;
  } else {
    STEPS_taken = userDistance;  //once STEPS_taken = userDistance ... trigger handleComplete()
  }

  selectProbe(selectedProbe);

  Serial.println("Starting scan...");
  startScan = false;

  currentState = STATE_ACTIVE;
}

// -----------------------------------------------------------------------------------------
// ACTIVE

void handleActive() {

  if (!scanActive) return;  // STOP CONTROL

  int dir = selectedDirection;

  // (100 / 50 microsteps)
  int stepSize = (currentMode == mode_normal) ? 100 : 50;

  stepper.move(dir * stepSize);
  delay(25);

  STEPS_taken += dir * stepSize;

  frequencySweepRaw();

  // -------- ANOMALY DETECTION --------
  if (currentMode == mode_normal) {
    if (anomalyDetection()) {
      currentMode = mode_highres;
      Serial.println("Switching to HIGH RES");
    }
  } else {
    if (!anomalyDetection()) {
      highResCounter++;
      if (highResCounter >= 3) {
        currentMode = mode_normal;
        highResCounter = 0;
        Serial.println("Back to NORMAL mode");
      }
    } else {
      highResCounter = 0;
    }
  }

  // -------- STOP CONDITION --------
  if ((dir == 1 && STEPS_taken >= userDistance) ||
      (dir == -1 && STEPS_taken <= 0)) {
    currentState = STATE_COMPLETE;
  }
}

// -----------------------------------------------------------------------------------------
// SCAN COMPLETE

void handleComplete() {
  Serial.println("Scan Complete.");

  STEPS_taken = 0;
  firstMeasurement = false;
  currentMode = mode_normal;
  highResCounter = 0;
  scanActive = false;
  startScan = false; 

  currentState = STATE_IDLE;
}

// -----------------------------------------------------------------------------------------
// MUX

void selectProbe(Probes probe) {
  bool signal = (probe == probes_12mm);

  digitalWrite(MUX_A, signal);
  digitalWrite(MUX_B, signal);

  if (signal) {
    Serial.println("MUX → 12mm probe");
  } else {
    Serial.println("MUX → 8mm probe");
  }

  delayMicroseconds(100);
}

// -----------------------------------------------------------------------------------------
// SWEEP

void frequencySweepRaw() {
    int real, imag, i = 0, cfreq = START_FREQ/1000;

    AD5933::setPowerMode(POWER_STANDBY);
    AD5933::setControlMode(CTRL_INIT_START_FREQ);
    AD5933::setControlMode(CTRL_START_FREQ_SWEEP);

    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {

        AD5933::getComplexData(&real, &imag);

        double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
        double impedance = 1/(magnitude * gain);

        if (i == 0) currentImpedance = impedance;

        Serial.print(selectedProbe == probes_8mm ? "8mm" : "12mm");
        Serial.print(",");
        Serial.print(cfreq);
        Serial.print(",");
        Serial.print(STEPS_taken);
        Serial.print(",");
        Serial.print(real);
        Serial.print(",");
        Serial.print(imag);
        Serial.print(",");
        Serial.println(impedance);

        i++;
        cfreq += FREQ_INCR/1000;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
    }

    AD5933::setPowerMode(POWER_STANDBY);
}

// -----------------------------------------------------------------------------------------
// ANOMALY

bool anomalyDetection() {
  if (!firstMeasurement) {
    baselineImpedance = currentImpedance;
    firstMeasurement = true;
    return false;
  }

  double change = abs((currentImpedance - baselineImpedance) / baselineImpedance);

  if (change > RATE_OF_CHANGE_THRESHOLD) {
    return true;
  } else {
    baselineImpedance = currentImpedance;
    return false;
  }
}
