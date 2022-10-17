#include <Elego_Motor.h>


// Motor control digital output pins defined as global constants
  const int mo1_controlPin1A = 2;                
  const int mo1_controlPin2A = 5;                 
  const int mo1_enablePin = 9;      
  Elego_Motor mo1(mo1_controlPin1A, mo1_controlPin2A, mo1_enablePin);

  const int mo2_controlPin1A = 8;                
  const int mo2_controlPin2A = 10;                 
  const int mo2_enablePin = 11;      
  Elego_Motor mo2(mo2_controlPin1A, mo2_controlPin2A, mo2_enablePin);


// Input pins 
  const int bstart_Pin = 4;         // Button Start Pin
  const int bcount_Pin = 7;         // Button provisional count  
  const int ledPin =  13;           // Indication LED

// Motor control global variables: 
  //Motor 1
  int mo1_mS = 250;                 // Motor speed 0..255
  bool mo1_mD = true;               // Forward (true) or reverse (false)
  int mo2_mS = 250;                 // Motor speed 0..255
  bool mo2_mD = false;               // Forward (true) or reverse (false)

// Button Variables
  int bstart_State = 0;             // Button Start Default State
  int bcount_State = 0;             // Button Count Default State
  int bcount_lastState = 0;         // Button Count Last State

// General Variables
  int mode = 0;                     // System Control Mode
  int count = 0;                    // Counter
  int count_max = 5;                // Max goods count in Transport Box

void setup() 
{
  Serial.begin(9600); // Initialize Serial
  
  // Decleare digital input pins:
  pinMode(bstart_Pin, INPUT);              // Button Start
  pinMode(bcount_Pin, INPUT);              // Counter Button
  pinMode(ledPin, OUTPUT);
}

// int runtime() {
//   static uint8_t rolloverCounter = 0;
//   static uint32_t lastMillis = 0;
//   uint32_t currentMillis = millis();
//   if (currentMillis < lastMillis) {       // check millis overflow
//     rolloverCounter++;
//   }
//   lastMillis = currentMillis;
//   uint32_t secs = (0xFFFFFFFF / 1000 ) * rolloverCounter + (currentMillis / 1000);
//   return secs;
// }

void counter(){
  bcount_State = digitalRead(bcount_Pin);
  // compare the buttonState to its previous state
  if (bcount_State != bcount_lastState) {
    // if the state has changed, increment the counter
    if (bcount_State == HIGH) 
    {
      // if the current state is HIGH then the button went from off to on:
      count++;
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  bcount_lastState = bcount_State;
}

void mode_one(){
  mo1.setDirection(mo1_mD);
  mo1.setSpeed(mo1_mS);
  mo2.setDirection(mo2_mD);
  mo2.setSpeed(mo2_mS);
}

void loop() 
{  
  
  // read the state of the pushbutton value:
  bstart_State = digitalRead(bstart_Pin);

  // Mode Switch Logic and Mode Logic that has to happen once
  if (bstart_State == HIGH && mode == 0)  // 0-1 Requirement: Button Start
  {
    mode = 1;
    mode_one();
    digitalWrite(ledPin, HIGH);
  }
  else if (count >= count_max)                    // 1-2 Requirement: Counter reaches count_max
  {
    mode = 2;
    mo1.stop(80);
    mo2.stop(80);
    digitalWrite(ledPin, LOW);
    mode = 0;
    count = 0;
  }
//  else if ()                    // 3-4 Requirement: 
//  {
//    mode = 3;
//  }

  // Mode Logic that has to be run each cycle
  if (mode == 1){    // Motor 1 for Transport System 1 and Divider is ON, Counter is counting goods
    counter();
  }
  else if (mode == 2){    // After counter reaches desired value the transportsystem 1 stops, counter is resette
  }
  else if (mode == 3){
    // Todo implment Actuators for Mode 3
  }
  else if (mode == 4){
    // Todo implment Actuators for Mode 4
  }
  else if (mode == 5){
    // Todo implment Actuators for Mode 5
  }
  Serial.println(count);
  
}