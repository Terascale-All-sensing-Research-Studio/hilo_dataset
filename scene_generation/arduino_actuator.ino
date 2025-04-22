#include <AccelStepper.h>

// Pins.
const int dirPin = 2;
const int stepPin = 4;
const int runningLightPinP = 8;
const int runningLightPinN = 9;
const int idleLightPinP = 11;
const int idleLightPinN = 12;

// Stepper constants.
const int motorInterfaceType = 1;
const int stepsPerRev = 400;
const double gearRatio = 9.913043478;

// Stepper parameters.
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
long targetPosition = 0;

void setup() {
  // Sets up all light pins.
  pinMode(runningLightPinP, OUTPUT);
  pinMode(runningLightPinN, OUTPUT);
  pinMode(idleLightPinP, OUTPUT);
  pinMode(idleLightPinN, OUTPUT);

  // Turns on the idle light.
  digitalWrite(idleLightPinP, HIGH);

  // Sets up the stepper.
  stepper.setMaxSpeed(2 * stepsPerRev); // The max speed is 2 rpm
  
  // Sets up the serial.
  Serial.begin(9600);
}

void loop() {

  // If receives message from the serial port...
  if (Serial.available() > 0) {
    // Stops the current task.
    stepper.stop();
    digitalWrite(runningLightPinP, LOW);
    delay(100);

    // Calculates the new target and responds to the remote machine.
    double targetAngle = Serial.parseInt() * gearRatio;
    targetPosition = stepper.currentPosition() + long(round(targetAngle / 0.9));
    Serial.println(targetPosition);
    digitalWrite(runningLightPinP, HIGH);
  }

  // Turns off the running light if the target has been reached.
  if (stepper.currentPosition() == targetPosition) {
    digitalWrite(runningLightPinP, LOW);
  }

  // Runs the stepper.
  stepper.moveTo(targetPosition);
  stepper.setSpeed(stepsPerRev);
  stepper.runSpeedToPosition();
}
