#include "Arduino.h"
#include "Elego_Stepper.h"
Elego_Stepper::Elego_Stepper(int stepPin, int resetPin) {
	pinMode(stepPin, OUTPUT);
	pinMode(resetPin, OUTPUT);
	
	_stepPin = stepPin;
	_resetPin = resetPin;
	_running = false;
	reset();
}
void Elego_Stepper::start(){
	if (!_running)                   // Start (Move to Step 1)
	  {
		digitalWrite(_stepPin, HIGH);
		delay(10);
		digitalWrite(_stepPin, LOW);
		_running = true;
	  }
}
void Elego_Stepper::stop(int reverseTime){
	if (_running)                   // Stop (Move to Step 0)
	  {
		digitalWrite(_stepPin, HIGH);
		delay(10);
		digitalWrite(_stepPin, LOW);
		delay(10);
		digitalWrite(_stepPin, HIGH);
		delay(10);
		digitalWrite(_stepPin, LOW);
		delay(reverseTime);
		digitalWrite(_stepPin, HIGH);
		delay(10);
		digitalWrite(_stepPin, LOW);
		_running = false;
	  }
}
void Elego_Stepper::reset(){
	digitalWrite(_resetPin, LOW);	// Reset Motor Driver
	delay(10);
	digitalWrite(_resetPin, HIGH);
}
