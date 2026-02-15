#include <Arduino.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 120

#define DIR 2
#define STEP 3
#define SLEEP 13 // optional (just delete SLEEP from everywhere if not used)

#include "A4988.h"
#define MS1 10
#define MS2 11
#define MS3 12
A4988 stepper(MOTOR_STEPS, DIR, STEP, SLEEP, MS1, MS2, MS3);


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  stepper.begin(RPM);
  stepper.enable();

  Serial.println("=== Bluetooth Remote Control ===");
  Serial.println("Commands:");
  Serial.println("  '1' - Set microstep to 1:1");
  Serial.println("  '8' - Set microstep to 1:8");
  Serial.println("  'b' - 200 steps");
}

void loop() {
  if(Serial.available()) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case '1':
        stepper.setMicrostep(1);
        Serial.println("Microstep set to 1");
        break;
        
      case '8':
        stepper.setMicrostep(8);
        Serial.println("Microstep set to 8");
        break;
        
      case 'b':
        Serial.println("200 motor steps");
        stepper.move(MOTOR_STEPS);    // forward revolution
        delay(200);
        stepper.move(-MOTOR_STEPS);   // reverse revolution
        Serial.println("\n Steps completed!");
        break;
        
      default:
        Serial.print("Unknown");
    }
  }
}