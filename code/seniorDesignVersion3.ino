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

double gain = 1.8089e-7;  // hardcoded gain

AD5933 ad5933; // REQUIRED for setRange()

// -----------------------------------------------------------------------------------------
// MULTIPLEXER CONTROL PINS
#define MUX_A 5
#define MUX_B 6

// -----------------------------------------------------------------------------------------

// State machine
enum SystemState {
  STATE_IDLE = 0,
  STATE_ACTIVE = 1,
  STATE_COMPLETE = 2
};
SystemState currentState = STATE_IDLE;

// Sampling mode
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
  scan_forward = 0,
  scan_backward = 1
};

// -----------------------------------------------------------------------------------------
// USER CONTROL VARIABLES

bool startScan = false;                  // waits for user input
Probes selectedProbe = probes_8mm;       // selected probe
ScanDirection selectedDirection = scan_forward;

// -----------------------------------------------------------------------------------------
// GLOBAL VARIABLES

double currentImpedance = 0.0;
double baselineImpedance = 0.0;
bool firstMeasurement = false;

const double RATE_OF_CHANGE_THRESHOLD = 0.20;

int STEPS = 4000;
int STEPS_taken = 0;
int highResCounter = 0;

// -----------------------------------------------------------------------------------------
// SETUP

void setup(void) {
  Wire.begin();
  Serial.begin(9600);

  Serial.println("System Ready");
  Serial.println("Commands: f8, f12, b8, b12");

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

  // FIXED: setRange must be instance-based
  if (!ad5933.setRange(CTRL_OUTPUT_RANGE_2)) {
    Serial.println("FAILED in setRange!");
    while (true);
  }

  // ---------------- STEPPER INIT ----------------
  stepper.begin(RPM);
  stepper.enable();
  stepper.setMicrostep(16);

  // ---------------- MUX INIT ----------------
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
}

// -----------------------------------------------------------------------------------------
// LOOP

void loop() {
  readSerialCommand();   // listen for user input

  if (!startScan) return;  // wait until command is entered

  processState();
  delay(100);
}

// -----------------------------------------------------------------------------------------
// SERIAL INPUT HANDLER

void readSerialCommand() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "f8") {
      selectedDirection = scan_forward;
      selectedProbe = probes_8mm;
      startScan = true;
      Serial.println("Selected: Forward + 8mm");
    }
    else if (cmd == "f12") {
      selectedDirection = scan_forward;
      selectedProbe = probes_12mm;
      startScan = true;
      Serial.println("Selected: Forward + 12mm");
    }
    else if (cmd == "b8") {
      selectedDirection = scan_backward;
      selectedProbe = probes_8mm;
      startScan = true;
      Serial.println("Selected: Backward + 8mm");
    }
    else if (cmd == "b12") {
      selectedDirection = scan_backward;
      selectedProbe = probes_12mm;
      startScan = true;
      Serial.println("Selected: Backward + 12mm");
    }
    else {
      Serial.println("Invalid command. Use: f8, f12, b8, b12");
    }
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
      Serial.println("Error: Unknown state!");
      currentState = STATE_IDLE;
      break;
  }
}

// -----------------------------------------------------------------------------------------
// IDLE STATE (WAIT + INITIALIZE SCAN)

void handleIdle() {
  if (!startScan) return;

  // Set starting position based on direction
  if (selectedDirection == scan_forward) {
    STEPS_taken = 0;
  } else {
    STEPS_taken = STEPS;
  }

  selectProbe(selectedProbe); // choose probe

  Serial.println("Starting scan...");
  startScan = false;

  currentState = STATE_ACTIVE;
}

// -----------------------------------------------------------------------------------------
// ACTIVE STATE (SCAN + MEASURE)

void handleActive() {

  int dir = (selectedDirection == scan_forward) ? +1 : -1;
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
  if ((dir == 1 && STEPS_taken >= STEPS) ||
      (dir == -1 && STEPS_taken <= 0)) {
    currentState = STATE_COMPLETE;
  }
}

// -----------------------------------------------------------------------------------------
// COMPLETE STATE (RESET SYSTEM)

void handleComplete() {
  Serial.println("Scan Complete.");

  STEPS_taken = 0;
  firstMeasurement = false;
  currentMode = mode_normal;
  highResCounter = 0;

  currentState = STATE_IDLE;
}

// -----------------------------------------------------------------------------------------
// MUX CONTROL (PROBE SWITCHING)

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
// FREQUENCY SWEEP (MEASUREMENT)

void frequencySweepRaw() {
    int real, imag, i = 0, cfreq = START_FREQ/1000;

    if (!(AD5933::setPowerMode(POWER_STANDBY) &&
          AD5933::setControlMode(CTRL_INIT_START_FREQ) &&
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP)))
    {
        Serial.println("Sweep init failed");
    }

    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {

        if (!AD5933::getComplexData(&real, &imag)) {
            Serial.println("Data read failed");
        }

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
// ANOMALY DETECTION

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
