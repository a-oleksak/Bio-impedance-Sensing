/*
 * Microstepping demo
 */

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
    /*
     * Set target motor RPM.
     */
    stepper.begin(RPM);
    stepper.enable();

}

void loop() {
    delay(1000);

    stepper.setMicrostep(1);  // Set microstep mode to 1:1

    // One complete revolution is 360°
    stepper.rotate(360);     // forward revolution
    stepper.rotate(-360);    // reverse revolution

    // One complete revolution is also MOTOR_STEPS steps in full step mode
    stepper.move(MOTOR_STEPS);    // forward revolution
    stepper.move(-MOTOR_STEPS);   // reverse revolution

    stepper.setMicrostep(8);   // Set microstep mode to 1:8

    // In 1:8 microstepping mode, one revolution takes 8 times as many microsteps
    stepper.move(8 * MOTOR_STEPS);    // forward revolution
    stepper.move(-8 * MOTOR_STEPS);   // reverse revolution
    
    // One complete revolution is still 360° regardless of microstepping mode
    // rotate() is easier to use than move() when no need to land on precise microstep position
    stepper.rotate(360);
    stepper.rotate(-360);

    delay(5000);
}
