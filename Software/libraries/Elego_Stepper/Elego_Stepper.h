#ifndef Elego_Stepper_h
#define Elego_stepper_h
#include "Arduino.h"
class Elego_Stepper {
public:
	Elego_Stepper(int stepPin, int resetPin);
	void start();
	void stop(int reverseTime);
	void reset();


private:
	int _stepPin;
	int _resetPin;
	bool _running;
};
#endif