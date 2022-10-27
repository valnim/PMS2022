
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */

//#include <Stepper.h>
#include <AccelStepper.h>

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution for your motor

// Interrrupt Pins
const int interruptPin1 = 2;  // Start Stop Pin
const int interruptPin2 = 3;  // Reverse Direction Pin

// Motor variables
int speed = 120;
int revs = 25;
bool direction = true;
bool running = false;


// initialize the stepper library on pins 8 through 11:
//Stepper myStepper(stepsPerRevolution, 12, 11, 10, 9);
AccelStepper myStepper(4, 12, 11, 10, 9);

void setup() {
  // set the speed at 60 rpm:
  //myStepper.setSpeed(speed); // max 200
  myStepper.setMaxSpeed(800);
  myStepper.setSpeed(20);	
  myStepper.setAcceleration(20000);
  // initialize the serial port:
  Serial.begin(9600);
  
  //attachInterrupt(digitalPinToInterrupt(interruptPin2), changeDir, RISING);
}

void loop() {
  //Serial.println(running);
  //Serial.println(direction);
  
  if (digitalRead(interruptPin2)){
    attachInterrupt(digitalPinToInterrupt(interruptPin1), stop, RISING);
    if (direction){
      myStepper.runToNewPosition(myStepper.currentPosition()+revs*stepsPerRevolution);
      //myStepper.step(revs*stepsPerRevolution);
    }
    else{
      myStepper.runToNewPosition(myStepper.currentPosition()-revs*stepsPerRevolution);
      //myStepper.step(-revs*stepsPerRevolution);
    }
    detachInterrupt(digitalPinToInterrupt(interruptPin1));
  }


  // // step one revolution  in one direction:
  // Serial.println("clockwise");
  // myStepper.step(20*stepsPerRevolution);
  // delay(5000);

  // // step one revolution in the other direction:
  // Serial.println("counterclockwise");
  // myStepper.step(-20*stepsPerRevolution);
  // delay(5000);
}

void stop(){
  Serial.println("STOP");
  //myStepper.step(0);
  myStepper.stop();
  // Serial.println("Button 1");
  // if (running){
  //   running = false;
  //   myStepper.step(0);
  // }
  // else{
  //   running = true;
  //   if (direction){
  //     myStepper.step(revs*stepsPerRevolution);
  //   }
  //   else{
  //     myStepper.step(-revs*stepsPerRevolution);
  //   }
  //   running = false;
  // }
}

void changeDir(){
  Serial.println("Button 2");
  direction = !direction;
}
