#include "Arduino.h"
#include "Elego_Motor.h"
Elego_Motor::Elego_Motor(int controlPin1A, int controlPin2A, int enablePin) {
	pinMode(controlPin1A, OUTPUT);
  pinMode(controlPin2A, OUTPUT);
  pinMode(enablePin, OUTPUT);
	_controlPin1A = controlPin1A;
  _controlPin2A = controlPin2A;
  _enablePin = enablePin;
  _motorDirection = true;
  _motorSpeed = 0;
}
void Elego_Motor::setSpeed(int motorSpeed){
analogWrite(_enablePin, motorSpeed);  // Speed
_motorSpeed = motorSpeed;
}
void Elego_Motor::setDirection(bool motorDirection){
if (motorDirection)                   // Forward
  {
    digitalWrite(_controlPin1A, HIGH);
    digitalWrite(_controlPin2A, LOW);
  }
else                                  // Reverse
  {
    digitalWrite(_controlPin1A, LOW);
    digitalWrite(_controlPin2A, HIGH);
  } 
_motorDirection = motorDirection;
}
void Elego_Motor::stop(int reverseTime){
  setDirection(!_motorDirection);
  delay(reverseTime);
  analogWrite(_enablePin, 0);
  setDirection(!_motorDirection);
}
