#include <Elego_Motor.h>


// Motor control digital output pins defined as global constants
  const int mo1_controlPin1A = 2;                
  const int mo1_controlPin2A = 5;                 
  const int mo1_enablePin = 9;         
  Elego_Motor mo1(mo1_controlPin1A, mo1_controlPin2A, mo1_enablePin);


// Input bins 
  const int bstart_Pin = 4;         // Button Start Pin      
  const int ledPin =  13;

// Motor control global variables: 
  //Motor 1
  int mo1_mS = 250;                 // Motor speed 0..255
  bool mo1_mD = true;               // Forward (true) or reverse (false)

// Button Variables
  int bstart_State = 0;             // Button Start Default State

// General Variables
  int mode = 0;                     // System Control Mode
  int time = 0;                     // Runtime [s]

void setup() 
{
  Serial.begin(9600); // Initialize Serial
  
  // Decleare digital input pins:
  pinMode(bstart_Pin, INPUT);              // Button Start
  pinMode(ledPin, OUTPUT);
}

int runtime() {
  static uint8_t rolloverCounter = 0;
  static uint32_t lastMillis = 0;
  uint32_t currentMillis = millis();
  if (currentMillis < lastMillis) {       // check millis overflow
    rolloverCounter++;
  }
  lastMillis = currentMillis;
  uint32_t secs = (0xFFFFFFFF / 1000 ) * rolloverCounter + (currentMillis / 1000);
  return secs;
}


void mode_one()
{
  mo1.setDirection(mo1_mD);
  mo1.setSpeed(mo1_mS);
}

void loop() 
{  
  
  // read the state of the pushbutton value:
  bstart_State = digitalRead(bstart_Pin);

  if (bstart_State == HIGH && mode == 0)
  {
    mode = 1;

  }
  else if (time >= 2 && bstart_State == HIGH)
  {
    mode = 2;
  }
  

  if (mode == 1){
    mode_one();
    digitalWrite(ledPin, HIGH);
    time = runtime();
  }
  else if (mode == 2){
    mo1.stop(80);
    digitalWrite(ledPin, LOW);
    mode = 0;
  }

  
}