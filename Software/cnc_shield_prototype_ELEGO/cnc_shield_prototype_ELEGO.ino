#include <AccelStepper.h>

//notes: limit switches shall be normally open

// define Shield Pins for Axis X to Z
#define stepPinX 2
#define dirPinX 5
#define limitX 9

#define stepPinY 3
#define dirPinY 6
#define limitY 10

#define stepPinZ 4
#define dirPinZ 7
#define limitZ 11

#define stepperEnable 8

// define Shield Pins for Spindle-Axis A (using pins D12 and D13)
#define stepPinA 12
#define dirPinA 13
//no limit for this axis available on the shield

#define stepsPerRevolutionX 200
#define stepsPerRevolutionY 200
#define stepsPerRevolutionZ 200
#define stepsPerRevolutionA 200

// Motor variables
int moXSpeed = 2000;            // Motor X Base Speed 
int moXMaxSpeedMult = 1;        // Motor X Max Speed Multiplier
int moXAccel = 20000;           // Motor X Acceleration
int moXDirection = -1;          // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction

//int mo1Pos1 = stepsPerRevolution1*1.5;    // Position 1 in relation to Reference Point
//int mo1Pos2 = stepsPerRevolution1*2.25;    // Position 2 in relation to Reference Point

//initialize the stepper motors as existing objects
AccelStepper stepperX(1, stepPinX, dirPinX);      //Stepper Type = 1, as used with DRV8825
AccelStepper stepperY(1, stepPinY, dirPinY);
AccelStepper stepperZ(1, stepPinZ, dirPinZ);
AccelStepper stepperA(1, stepPinA, dirPinA);

// SFC Variables
int mode = 0;     // Current SFC Mode State

void setup() 
{ 
  // initialize the serial port
  Serial.begin(9600);
  stepperX.setEnablePin(stepperEnable);
  stepperY.setEnablePin(stepperEnable);
  
  // Initialize Inputs
  pinMode(limitX, INPUT);
  pinMode(limitY, INPUT);
  pinMode(limitZ, INPUT);

  //Initialize Outputs  
  
  pinMode(stepperEnable, OUTPUT);
  //digitalWrite(stepperEnable, HIGH);
  
  stepperX.setAcceleration(moXAccel);
  stepperX.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperX.setSpeed(moXDirection*moXSpeed);

  stepperY.setAcceleration(moXAccel);
  stepperY.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperY.setSpeed(moXDirection*moXSpeed);
}

void loop() {
  // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(limitX) == HIGH && mode == 0)                      //0-1 Requirement: Button Start
  {
    //digitalWrite(stepperEnable, HIGH);
    stepperX.enableOutputs();
    stepperY.disableOutputs();
    Serial.println();
    mode = mode+1; // Switch Mode
  }
  else if (digitalRead(limitY) && mode == 1)                              //1-2 Reference Sensor Reached
  {
    stepperX.disableOutputs();
    stepperY.enableOutputs();
    mode = 0 ; // Switch Mode
    //digitalWrite(stepperEnable, LOW);
  }
  
  Serial.println(mode);
  stepperX.runSpeed();
  stepperY.runSpeed();
}

