#include <AccelStepper.h>

const int stepsPerRev = 200;  // change this to fit the number of steps per revolution for your motor

// Input Pins
const int button1 = 2;  // Start Pin
const int button2 = 3;  // Reference Button Pin

// Output Pins
const int ledPin =  13; // Indicatior LED Pin

// Motor variables
int mo1Speed = 100;             // Motor 1 Base Speed 
int mo1MaxSpeedMult = 2;        // Motor 1 Max Speed Multiplier
int mo1Accel = 20000;           // Motor 1 Acceleration
int m01Direction = 1;           // Motor 1 Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int mo1Pos1 = stepsPerRev*0.5;    // Position 1 in relation to Reference Point
int mo1Pos2 = stepsPerRev*1.25;    // Position 2 in relation to Reference Point

// initialize the stepper library on pins 12 through 9:
AccelStepper stepper1(4, 12, 11, 10, 9);    // Definition of Stepper Motor (StepperType=4, PINA1 = 12, PINA2 = 11, PINA3 = 10, PINA4 = 9)

// SFC Variables
int mode = 0;     // Current SFC Mode State

void setup() { 
  // initialize the serial port:
  Serial.begin(9600);
  // Initialize Inputs
  pinMode(button1, INPUT);
  pinMode(button2, INPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(button1) == HIGH && mode == 0)                      //0-1 Requirement: Button Start
  {
    stepper1.setAcceleration(mo1Accel);
    stepper1.setMaxSpeed(m01Direction*mo1Speed*2);
    stepper1.setSpeed(-m01Direction*mo1Speed);	
    mode = 1; // Switch Mode
    Serial.println("Start Referencing");
  }
  if (digitalRead(button2) && mode == 1)                              //1-2 Reference Sensor Reached
  {
    // Stopp Motor at Reference Position
    stepper1.stop();
    Serial.println(stepper1.currentPosition());     // Log Position
    stepper1.setCurrentPosition(0);                 // Set Current Position to Zero
    Serial.println(stepper1.currentPosition());     // Log Position
    delay(1000);   // Wait at Reference Position
    // Setup motor settings for movement towards Pos1
    stepper1.setAcceleration(mo1Accel);
    stepper1.setMaxSpeed(m01Direction*mo1Speed*2);
    stepper1.setSpeed(m01Direction*mo1Speed);	
    mode = 2; // Switch Mode
    Serial.println("Reference Position Reached");
  }
  else if (abs(stepper1.currentPosition()) >= mo1Pos1 && mode == 2)        // 2-3 Requirement: Position 1 Reached
  {
    // Stop Motor at Pos 1
    stepper1.stop();
    Serial.println(abs(stepper1.currentPosition()));  // Log Current Position
    delay(2000);  // Wait at Position 1
    // Setup motor settings for movement towards Pos2
    stepper1.setAcceleration(mo1Accel);
    stepper1.setMaxSpeed(m01Direction*mo1Speed*2);
    stepper1.setSpeed(m01Direction*mo1Speed);	
    mode = 3; // Switch Mode
    Serial.println("Pos 1 Reached");
  }
  else if (abs(stepper1.currentPosition()) >= mo1Pos2 && mode == 3)         // 3-4 Requirement: Position 2 Reached
  {
    // Stop Motor at Pos 2
    stepper1.stop();
    Serial.println(abs(stepper1.currentPosition()));  // Log Current Position
    delay(500); // Wait at Position 2
    mode = 0; // Switch Mode
    Serial.println("Pos 2 reached");
  }
 
  // Mode Logic that has to be run each cycle
  if (mode == 1){                                             // 1 Referencing
    stepper1.runSpeed();  // refreshes Motor control
  }
  else if (mode == 2){                                        // 2 Move To Position 1 
    stepper1.runSpeed();  // refreshes Motor control
  }
  else if (mode == 3){                                        // 2 Move To Position 2
    stepper1.runSpeed();  // refreshes Motor control
  }
  
  
}

