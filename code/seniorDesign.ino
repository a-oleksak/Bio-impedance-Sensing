#include <Arduino.h>
#include <Wire.h>
#include "AD5933.h"
#include "A4988.h"

// -----------------------------------------------------------------------------------------
// values for the motor and a4988
#define MOTOR_STEPS 200
#define RPM 120

#define DIR 2
#define STEP 3
#define SLEEP 13 // optional (just delete SLEEP from everywhere if not used)

#define MS1 10
#define MS2 11
#define MS3 12
A4988 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, MS1, MS2, MS3);

// -----------------------------------------------------------------------------------------
// values for the ad5933
#define START_FREQ  (3000)
#define FREQ_INCR   (1000)
#define NUM_INCR    (4) // measures from 3kHz to 7kHz
// #define REF_RESIST  (10000) // not needed right now

double gain = 6.7 * pow(10,-9); // VALUES TAKEN FROM PREVIOUS GROUP, WILL LOOK INTO LATER
double phase;

// -----------------------------------------------------------------------------------------
// control pins - multiplexer support
#define MUX_A 5    // D5
#define MUX_B 6    // D6

// -----------------------------------------------------------------------------------------
// setting up cases
enum SystemState {
  STATE_IDLE = 0,
  STATE_ACTIVE1 = 1,
  STATE_ACTIVE2 = 2,
  STATE_COMPLETE = 3
}; 
SystemState currentState = STATE_IDLE;  // Initial state

enum SamplingMode {
  mode_normal = 0,
  mode_highres = 1,
};
SamplingMode currentMode = mode_normal; // Initial mode 

enum Probes {
  probes_8mm = 0,
  probes_12mm = 1,
};
Probes currentProbe = probes_8mm; // Initial set of probes 

// -----------------------------------------------------------------------------------------
// global variables for anomaly detection
double previousImpedance = 0.0;
double currentImpedance = 0.0;          
bool firstMeasurement = false;
const double RATE_OF_CHANGE_THRESHOLD = 0.20; // 20% threshold - LOOK INTO LATER

// -----------------------------------------------------------------------------------------
// other global variables
int STEPS; // # of steps which stepper motor will perform 
int STEPS_taken; // # of steps taken
int highResCounter = 0; 

// -----------------------------------------------------------------------------------------
// setup
void setup(void) {
  // Begin I2C
  Wire.begin();

  // Begin serial at 9600 baud for output
  Serial.begin(9600);
  Serial.println("Test Started!");

  // Perform initial configuration. Fail if any one of these fail.
  if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setPGAGain(PGA_GAIN_X1) &&
        AD5933::setRange(CTRL_OUTPUT_RANGE_2))) // added - hopefully will fix gain issue
        {
            Serial.println("FAILED in initialization!");
            while (true) ;
        }

  // Perform calibration sweep
  if (AD5933::calibrate(gain, phase, REF_RESIST, NUM_INCR+1))
    Serial.println("Calibrated!");
  else
    Serial.println("Calibration failed...");
  }
  
  // Set target motor RPM
  stepper.begin(RPM);
  stepper.enable();
  stepper.setMicrostep(16); //3200 microsteps per revolution

  // setup multiplexers
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  selectProbe(currentProbe);

  // Set idle state
  currentState = STATE_IDLE;
  currentMode = mode_normal;
}

// -----------------------------------------------------------------------------------------
// loop
void loop() {
  processState();
  delay(100);
 
}

// -----------------------------------------------------------------------------------------
// switch case for each state
void processState() {
  switch(currentState) {
    case STATE_IDLE:
      handleIdle();
      break;
      
    case STATE_ACTIVE1:
      handleActive1();
      break;
    
    case STATE_ACTIVE2:
      handleActive2();
      break;

    case STATE_COMPLETE:
      handleComplete();
      break;
      
    default:
      // Handle unexpected state
      handleError();
      break;
  }
}

// -----------------------------------------------------------------------------------------
// NOTES: for idle, we want an input for the distance we plan to measure (# of steps), as
// long as the distant given is not 0, we can switch system states
void handleIdle() {
  STEPS = 4000; // stepper motor will move 1 cm by default (as of right now)
  STEPS_taken = 0;  // reset counter
  if (STEPS != 0) {
    currentState = STATE_ACTIVE1;
  }
}

// -----------------------------------------------------------------------------------------
// NOTES: this will be moving our probes FORWARD (with first set of probes) and doing the
// freq sweep every n steps depending on the sampling mode
void handleActive1() {
  currentProbe = probes_8mm;
  selectProbe(currentProbe);
  if(STEPS_taken != STEPS){
    if(currentMode == mode_normal){
      stepper.move(100); // moves 100 microsteps or 0.25mm
      STEPS_taken = STEPS_taken + 100;
      frequencySweepRaw();
      if(anomalyDetection()){
        currentMode = mode_highres;
      }
    }
    else{
      stepper.move(50); // moves 50 microsteps or 0.125mm
      STEPS_taken = STEPS_taken + 50;
      frequencySweepRaw();
      highResCounter++;
      if(highResCounter >= 10) {
        currentMode = mode_normal;
        highResCounter = 0;
      }
    } 
  }
  if (STEPS_taken >= STEPS) {
    currentState = STATE_ACTIVE2;
  }
}

// -----------------------------------------------------------------------------------------
// NOTES: this will be moving our probes BACKWARDS (with second set of probes) and doing the
// freq sweep every n steps depending on the sampling mode
void handleActive2() {
  currentProbe = probes_12mm;
  selectProbe(currentProbe);
  if(STEPS_taken != 0){
    if(currentMode == mode_normal){
      stepper.move(-100); // moves -100 microsteps or -0.25mm
      STEPS_taken = STEPS_taken - 100;
      frequencySweepRaw();
      if(anomalyDetection()){
        currentMode = mode_highres;
      }
    }
    else{
      stepper.move(-50); // moves -50 microsteps or -0.125mm
      STEPS_taken = STEPS_taken - 50;
      frequencySweepRaw();
      highResCounter++;
      if(highResCounter >= 10) {
        currentMode = mode_normal;
        highResCounter = 0;
      }
    }
  }

  if (STEPS_taken <= 0) {
    currentState = STATE_COMPLETE;
  }
}

// -----------------------------------------------------------------------------------------
// NOTES: resets variables and sends back to idle
void handleComplete() {
  STEPS_taken = 0;
  firstMeasurement = false;
  currentMode = mode_normal;
  highResCounter = 0;

  delay(2000);
  currentState = STATE_IDLE;
}

// -----------------------------------------------------------------------------------------
void handleError() {
  Serial.println("Error: Unknown state!");
  currentState = STATE_IDLE;  // Reset to idle
}

// -----------------------------------------------------------------------------------------
// compare two impedance values and checking if the difference is above the rate of change
// threshold
bool anomalyDetection() {
  if (!firstMeasurement) {
    // stores first measurement
    previousImpedance = currentImpedance;
    firstMeasurement = true;
    return false;
  }
        
  // Calculate percentage change
  double percentageChange = abs((currentImpedance - previousImpedance) / previousImpedance) * 100.0;
    
  // Store current for next comparison
  previousImpedance = currentImpedance;
    
  // Check threshold
  if (percentageChange > RATE_OF_CHANGE_THRESHOLD * 100) { // anomaly detected
    return true;
  }
  
    return false;
}
// -----------------------------------------------------------------------------------------
// selects which pairs of probes we will use for measuring
void selectProbe(Probes probe) {
  // Convert enum to digital signal
  bool signal = (probe == probes_12mm);  // 0 for 8mm, 1 for 12mm
  
  // Control both multiplexers (AX/BX and AY/BY switch together)
  digitalWrite(MUX_A, signal);
  digitalWrite(MUX_B, signal);
  
  // small delay for multiplexer to settle
  delayMicroseconds(100);  // CD4053BE needs ~50-100ns, this is safe
}

// -----------------------------------------------------------------------------------------
// frequency sweep method, gives us the impedance
void frequencySweepRaw() {
    // Create variables to hold the impedance data and track frequency
    int real, imag, i = 0, cfreq = START_FREQ/1000;

    // Initialize the frequency sweep
    if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
          AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             Serial.println("Could not initialize frequency sweep...");
         }

    // Perform the actual sweep
    while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
        // Get the frequency data for this frequency point
        if (!AD5933::getComplexData(&real, &imag)) {
            Serial.println("Could not get raw frequency data...");
        }

        // Print out the frequency data - MIGHT WANT TO COMMENT THIS OUT LATER
        Serial.print(cfreq);
        Serial.print(": R=");
        Serial.print(real);
        Serial.print("/I=");
        Serial.print(imag);

        // Compute impedance
        double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
        double impedance = 1/(magnitude*gain);

        // store the first impedance value for anomaly detection
        if (i == 0) { 
          currentImpedance = impedance;
        }

        Serial.print("  |Z|=");
        Serial.println(impedance);

        // Increment the frequency
        i++;
        cfreq += FREQ_INCR/1000;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
    }

    // Set AD5933 power mode to standby when finished
    if (!AD5933::setPowerMode(POWER_STANDBY))
        Serial.println("Could not set to standby...");
}
