#include <AccelStepper.h>

// Motor Pins
#define dirPin1 5
#define stepPin1 2
#define stepsPerRevolution1 200
#define limit1 9
#define limit2 10
#define limit3 11
#define stepperEnable 8


// Motor variables
int mo1Speed = 2000;             // Motor 1 Base Speed 
int mo1MaxSpeedMult = 1;        // Motor 1 Max Speed Multiplier
int mo1Accel = 20000;           // Motor 1 Acceleration
int m01Direction = -1;           // Motor 1 Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
//int mo1Pos1 = stepsPerRevolution1*1.5;    // Position 1 in relation to Reference Point
//int mo1Pos2 = stepsPerRevolution1*2.25;    // Position 2 in relation to Reference Point

// initialize the stepper library on pins 12 through 9:
//AccelStepper stepper1(4, m01PinA1, m01PinA2, m01PinA3, m01PinA4);    // Definition of Stepper Motor (StepperType=4, PINA1 = 12, PINA2 = 11, PINA3 = 10, PINA4 = 9)
//AccelStepper stepper2(4, m02PinA1, m02PinA2, m02PinA3, m02PinA4);
AccelStepper stepper1(1, stepPin1, dirPin1);


// SFC Variables
int mode = 0;     // Current SFC Mode State

//HardwareSerial Serial1(PA10, PA9);

void setup() { 
  // initialize the serial port:
  Serial.begin(9600);
  stepper1.setEnablePin(stepperEnable);
  // Initialize Inputs
  pinMode(limit1, INPUT);
  pinMode(limit2, INPUT);
  pinMode(stepperEnable, OUTPUT);
  //digitalWrite(stepperEnable, HIGH);
  
  stepper1.setAcceleration(mo1Accel);
  stepper1.setMaxSpeed(m01Direction*mo1Speed*2);
  stepper1.setSpeed(m01Direction*mo1Speed);

}

void loop() {
  // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(limit1) == HIGH && mode == 0)                      //0-1 Requirement: Button Start
  {
    //digitalWrite(stepperEnable, HIGH);
    stepper1.enableOutputs();
    Serial.println();
    mode = mode+1; // Switch Mode
  }
  else if (digitalRead(limit2) && mode == 1)                              //1-2 Reference Sensor Reached
  {
    stepper1.disableOutputs();
    
    mode = 0 ; // Switch Mode
    //digitalWrite(stepperEnable, LOW);
  }
  
  Serial.println(mode);
  stepper1.runSpeed();
  
  
}

