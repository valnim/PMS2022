#include <AccelStepper.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution for your motor

// Interrrupt Pins
const int interruptPin1 = 2;  // Stop Pin
const int interruptPin2 = 3;  // Start Pin

// Motor variables
int mo1Speed = 400;
bool mo1Running = false;
int m01Direction = 1;

// initialize the stepper library on pins 12 through 9:
AccelStepper stepper1(4, 12, 11, 10, 9);

void setup() { 
  // initialize the serial port:
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), start, RISING);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), stop, RISING);
}

void loop() {
  if (mo1Running){
    stepper1.setAcceleration(20000);
    stepper1.setMaxSpeed(m01Direction*800);
    stepper1.setSpeed(m01Direction*400);	
    stepper1.runSpeed();
  }
  
}

void stop(){
  if (mo1Running){
    stepper1.stop();
    mo1Running = false;  
  }
  else{
    m01Direction = -1 * m01Direction;
  }
  
}

void start(){
  mo1Running = true;
}
