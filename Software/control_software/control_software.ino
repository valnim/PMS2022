#include <AccelStepper.h>
// CNC Shield Pins
// define Shield Pins for Axis X to Z
// Stepper X: Crane Lift Axis
#define stepPinX 2
#define dirPinX 5
#define limitX 9

//Stepper Y: Crane Rot/Phi Axis 
#define stepPinY 3
#define dirPinY 6
#define limitY 10

//Stepper Z: Elego Direct Current Motors, Linked to Pins 1 & 4 (Black & Blue)
#define stepPinZ 4
#define dirPinZ 7
#define limitZ 11

// define Shield Pins for Spindle-Axis A (using pins D12 and D13) not used currently
#define stepPinA 12
#define dirPinA 13

//Enable Outputs of all Stepper Drivers
#define stepperEnable 8

//Steps Per Revolution of Steppers
#define stepsPerRevX 200
#define stepsPerRevY 200

// Motor variables
// Crane Lif Motor
int moXSpeed = 350;             // Motor X Base Speed 
int moXMaxSpeedMult = 2;        // Motor X Max Speed Multiplier
int moXAccel = 20000;           // Motor X Acceleration
int moXDirection = -1;          // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int moXInitDistance = 2* step   // Motor X Initialization Distance
// Crane Rot Motor
int moYSpeed = 300;             // Motor X Base Speed 
int moYMaxSpeedMult = 2;        // Motor X Max Speed Multiplier
int moYAccel = 20000;           // Motor X Acceleration
int moYDirection = -1;          // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int moYInitDistance = 1* step   // Motor X Initialization Distance

// Stepper Motor Positions
const int phiPos1 = stepsPerRev*1;   // Position 1 for Motor 2 in Phi-Axis
const int phiPos2 = stepsPerRev*6;   // Position 2 for Motor 2 in Phi-Axis
const int xPos1 = stepsPerRev*15;    // Position 1 for Motor 1 in Z-Axis
const int xPos2 = stepsPerRev*10;    // Position 2 for Motor 1 in Z-Axis

//initialize the stepper motors as existing objects
AccelStepper stepperX(1, stepPinX, dirPinX);    // Crane Lift Motor
AccelStepper stepperY(1, stepPinY, dirPinY);    // Crane Rotation Motor
AccelStepper stepperZ(1, stepPinZ, dirPinZ);    // Elego Motor --> move to seperate Library

// SFC Variables
int mode = 0;     // Current SFC Mode State

HardwareSerial Serial1(PA10, PA9);

void setup() 
{ 
  // initialize the serial port
  Serial.begin(9600);
  stepperX.setEnablePin(stepperEnable);
  
  
  // Initialize Inputs
  pinMode(limitX, INPUT);
  pinMode(limitY, INPUT);
  pinMode(limitZ, INPUT);

  //Initialize Outputs  
  
  pinMode(stepperEnable, OUTPUT);
  //digitalWrite(stepperEnable, HIGH);
  
  stepperX.setAcceleration(moXAccel);
  stepperX.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperX.setSpeed(0);

  //stepperY.setAcceleration(moXAccel);
  //stepperY.setMaxSpeed(moXDirection*moXSpeed*2);
  //stepperY.setSpeed(0);

  stepperX.disableOutputs();
  
}

void loop() {
  // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(limitX) == HIGH && mode == 0)                      //0-1 Requirement: Button Start
  {
    stepperX.disableOutputs();

    stepperX.setSpeed(moXDirection*moXSpeed);

    Serial.println();
    mode = mode+1; // Switch Mode

    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);
  }
  else if (digitalRead(limitY) && mode == 1)                              //1-2 Reference Sensor Reached
  {
    
    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);
    delay(10);
    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);
    delay(50);
    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);

    mode = 0 ; // Switch Mode
    
  }
  
  Serial.println(mode);
  if (mode == 1){
    stepperX.runSpeed();
  }
  
}

