#ifndef Elego_Motor_h
#define Elego_Motor_h
#include "Arduino.h" 
class Elego_Motor {
public:
	Elego_Motor(int controlPin1A, int controlPin2A, int enablePin);
	void setSpeed(int motorSpeed);
  void setDirection(bool motorDirection);
  void stop(int reverseTime);


private:
	int _controlPin1A;
  int _controlPin2A;
  int _enablePin;
  int _motorSpeed;
  bool _motorDirection;
};
#endif