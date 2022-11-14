#include <AccelStepper.h>

const int stepsPerRev = 200;  // change this to fit the number of steps per revolution for your motor

// Input Pins
#define button1 2  // Start Pin
#define limit1 13  // Limit Switch 1 Z-Axis
#define limit2 8   // Limit Switch 2 Phi-Axis

// Output Pins
//#define ledPin 13 // Indicatior LED Pin

// Motor Pins
// Lift Motor
#define m01PinA1 12
#define m01PinA2 11
#define m01PinA3 10
#define m01PinA4 9

//Rotation Motor
#define m02PinA1 7
#define m02PinA2 6
#define m02PinA3 5
#define m02PinA4 3

// Motor variables
// Lift Motor
int liftSpeed = 350;             // Motor 1 Base Speed 
int liftMaxSpeedMult = 2;        // Motor 1 Max Speed Multiplier
int liftAccel = 20000;           // Motor 1 Acceleration
int liftDir = 1;           // Motor 1 Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int liftInitDistance = 10*stepsPerRev;      // Initialization Distance

// Rotation Motor
int rotSpeed = 300;             // Motor 2 Base Speed 
int rotMaxSpeedMult = 2;        // Motor 2 Max Speed Multiplier
int rotAccel = 20000;           // Motor 2 Acceleration
int rotDir = 1;           // Motor 2 Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int rotInitDistance = 10*stepsPerRev;      // Initialization Distance

// Motor Positions
const int Pos1 = stepsPerRev*0.1;    // Position 1 for Motor 2 in Phi-Axis
const int Pos2 = stepsPerRev*15;    // Position 2 for Motor 1 in Z-Axis
const int Pos3 = stepsPerRev*0.4;    // Position 3 for Motor 2 in Phi-Axis
const int Pos4 = stepsPerRev*10;    // Position 4 for Motor 1 in Z-Axis

// initialize the stepper library on pins 12 through 9:
AccelStepper stepperLift(4, m01PinA1, m01PinA2, m01PinA3, m01PinA4);    // Definition of Stepper Motor (StepperType=4, PINA1 = 12, PINA2 = 11, PINA3 = 10, PINA4 = 9)
AccelStepper stepperRot(4, m02PinA1, m02PinA2, m02PinA3, m02PinA4);

// SFC Variables
int mode = 0;     // Current SFC Mode State

HardwareSerial Serial1(PA10, PA9);

void setup() { 
  // initialize the serial port:
  Serial.begin(9600);
  // Initialize Inputs
  pinMode(button1, INPUT);
  pinMode(limit1, INPUT);
  pinMode(limit2, INPUT);
  //pinMode(ledPin, OUTPUT);

  // Set up Stepper 1 Accelleration and Max Speed
  stepperLift.setAcceleration(liftAccel);
  stepperLift.setMaxSpeed(liftSpeed*liftMaxSpeedMult);
  stepperLift.stop();
  stepperLift.setSpeed(0);
  stepperLift.setCurrentPosition(0);
  // Set up Stepper 2 Accelleration and Max Speed
  stepperRot.setAcceleration(rotAccel);
  stepperRot.setMaxSpeed(rotSpeed*rotMaxSpeedMult);
  stepperRot.stop();
  stepperRot.setSpeed(0);
  stepperRot.setCurrentPosition(0);
}

void loop() {
  // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(button1) == HIGH && mode == 0)                                  //0-1 Requirement: Button Start
  {
    Serial.println("Initializing Z-Axis");
    // Move upwards in Z-Direction to unpress limit switch
    stepperLift.setSpeed(liftDir*liftSpeed/2);
    mode = 1;   // Switch Mode
    Serial.println(mode);
  }
  else if (abs(stepperLift.currentPosition()) >= liftInitDistance && mode == 1)   // 1-2  Requirement: Lift Motor Initialization Position reached
  {
    stepperLift.stop();  // Stop Lift Motor at Pos 2
    stepperLift.setSpeed(0);
    Serial.println("Initializing Phi-Axis");
    // Move Clockwise to unpress limit switch
    stepperRot.setSpeed(rotDir*rotSpeed/2);    
    mode = 2;   // Switch Mode
    Serial.println(mode);
  }
  else if (abs(stepperRot.currentPosition()) >= rotInitDistance && mode == 2)      //2-3 Requirement: Rotation Motor Initialization Position reached
  {
    stepperRot.stop();
    stepperRot.setSpeed(0);
    // Set Motor Direction for Referencing   
    stepperRot.setSpeed(-rotDir*rotSpeed);
    mode = 3; // Switch Mode
    Serial.println(mode);
    Serial.println("Start Referencing Phi-Axis");
  }
  else if (digitalRead(limit2) && (mode == 3 || mode == 9))                         //3-4 OR 9-3 Limit Switch 2 in Phi-Axis reached
  {
    // Stopp Motor 2 at Reference Position
    stepperRot.stop();
    stepperRot.setSpeed(0);
    stepperRot.setCurrentPosition(0);                   // Set Current Position to Zero

    // Set Motor Direction for Referencing    
    stepperLift.setSpeed(-liftDir*liftSpeed);
    mode = 4; // Switch Mode
    Serial.println(mode);
    Serial.println("Start Referencing Z-Axis");
  }
  else if (digitalRead(limit1) && mode == 4)        // 4-5 Limit Switch 1 in Z-Axis reached
  {
    // Stopp Motor 1 at Reference Position
    stepperLift.stop();
    stepperLift.setSpeed(0);
    stepperLift.setCurrentPosition(0);                   // Set Current Position to Zero
    delay(1000);                                      // Wait at Reference Position
    Serial.println("Referencing Complete");

    // Move Roation Motor 2 to Pos 1
    stepperRot.setSpeed(rotDir*rotSpeed);	
    mode = 5; // Switch Mode
    Serial.println(mode);
  }
  else if ((abs(stepperRot.currentPosition()) >= Pos1 && mode == 5) )         // 5-6 Requirement: Rotation Motor Position 1 reached
  {
    stepperRot.stop();  // Stop Rotation Motor at Pos 1
    stepperRot.setSpeed(0);
    Serial.println("Pos 1 reached");
    Serial.println(abs(stepperRot.currentPosition()));  // Log Current Position
    delay(500); // Wait at Position 1

    // Move Lift Motor to Pos 2
    stepperLift.setSpeed(liftDir*liftSpeed);	
    mode = 6;   // Switch Mode
    Serial.println(mode);
  }
  else if (abs(stepperLift.currentPosition()) >= Pos2 && mode == 6)         // 6-7 Requirement: Lift Motor 1 Position 2 reached
  {
    stepperLift.stop();  // Stop Lift Motor at Pos 2
    stepperLift.setSpeed(0);
    Serial.println("Pos 2 reached");
    Serial.println(abs(stepperLift.currentPosition()));  // Log Current Position
    delay(500); // Wait at Position 2

    // Move Rotation Motor to Pos 3
    stepperRot.setSpeed(rotDir*rotSpeed);	
    mode = 7;   // Switch Mode
    Serial.println(mode);
  }
  else if (abs(stepperRot.currentPosition()) >= Pos3 && mode == 7)         // 7-8 Requirement: Rotation Motor Position 3 reached
  {
    stepperRot.stop();  // Stop Roation Motor at Pos 3
    stepperRot.setSpeed(0);
    Serial.println("Pos 3 reached");
    Serial.println(abs(stepperRot.currentPosition()));  // Log Current Position
    delay(500); // Wait at Position 3

    // Move Lift Motor to Pos 4
    stepperLift.setSpeed(-liftDir*liftSpeed);	
    mode = 8;   // Switch Mode
    Serial.println(mode);
  }
  else if (abs(stepperLift.currentPosition()) <= Pos4 && mode == 8)         // 8-9 Requirement: Lift Motor Position 4 reached
  {
    stepperLift.stop();  // Stop Lift Motor at Pos 4
    stepperLift.setSpeed(0);
    Serial.println("Pos 4 reached");
    Serial.println(abs(stepperLift.currentPosition()));  // Log Current Position
    delay(500); // Wait at Position 4

    // Move Rotation Motor to Reference Position Phi
    stepperRot.setSpeed(-1*rotDir*rotSpeed);	
    mode = 9;   // Switch Mode
    Serial.println(mode);
  }
 
  stepperLift.runSpeed();  // refreshes Motor control
  stepperRot.runSpeed();
  
}

