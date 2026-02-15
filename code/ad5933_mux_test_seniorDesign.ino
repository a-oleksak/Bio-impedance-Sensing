/*
ad5933-test
    Reads impedance values from the AD5933 over I2C and prints them serially.
*/

#include <Wire.h>
#include "AD5933.h"

#define START_FREQ  (3000)
#define FREQ_INCR   (1000)
#define NUM_INCR    (15)
#define REF_RESIST  (10000)

// control pins - multiplexer support
#define MUX_A 5    // D5
#define MUX_B 6    // D6

enum Probes {
  probes_8mm = 0,
  probes_12mm = 1,
};
Probes currentProbe = probes_8mm; // Initial set of probes 

double gain[NUM_INCR+1];
int phase[NUM_INCR+1];

void setup(void)
{
  // Begin I2C
  Wire.begin();

  // Begin serial at 9600 baud for output
  Serial.begin(9600);
  Serial.println("AD5933 Test Started!");

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

  // setup multiplexers
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  selectProbe(currentProbe);
}

void loop(void)
{
  frequencySweepRaw();
  currentProbe = probes_12mm;
  selectProbe(currentProbe);

  // Delay
  delay(5000);

  frequencySweepRaw();
  currentProbe = probes_8mm;
  selectProbe(currentProbe);

  // Delay
  delay(5000);
}

// allows for data to be processed in real time. 
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

        // Print out the frequency data
        Serial.print(cfreq);
        Serial.print(": R=");
        Serial.print(real);
        Serial.print("/I=");
        Serial.print(imag);

        // Compute impedance
        double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
        double impedance = 1/(magnitude*gain[i]);
        Serial.print("  |Z|=");
        Serial.println(impedance);

        // Increment the frequency
        i++;
        cfreq += FREQ_INCR/1000;
        AD5933::setControlMode(CTRL_INCREMENT_FREQ);
    }

    Serial.println("Frequency sweep complete!");

    // Set AD5933 power mode to standby when finished
    if (!AD5933::setPowerMode(POWER_STANDBY))
        Serial.println("Could not set to standby...");
}

void selectProbe(Probes probe) {
  // Convert enum to digital signal
  bool signal = (probe == probes_12mm);  // 0 for 8mm, 1 for 12mm
  
  // Control both multiplexers (AX/BX and AY/BY switch together)
  digitalWrite(MUX_A, signal);
  digitalWrite(MUX_B, signal);
  
  // small delay for multiplexer to settle
  delayMicroseconds(100);  // CD4053BE needs ~50-100ns, this is safe
}
