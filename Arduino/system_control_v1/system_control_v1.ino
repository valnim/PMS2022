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
  const int pResistor = A0;         // Photoresistor at Arduino analog pin A0

// Output pins
  const int ledPin =  13;           // Indication LED


// Motor control global variables: 
  //Motor 1
  int mo1_mS = 254;                 // Motor speed 0..255
  bool mo1_mD = false;               // Forward (true) or reverse (false)
  int mo2_mS = 254;                 // Motor speed 0..255
  bool mo2_mD = false;               // Forward (true) or reverse (false)

// Button Variables
  int bstart_State = 0;             // Button Start Default State
  int count_State = 0;             // Button Count Default State
  int count_lastState = 0;         // Button Count Last State

// General Variables
  int mode = 0;                     // System Control Mode
  int count = 0;                    // Counter
  int count_max = 5;                // Max goods count in Transport Box

  bool calibrated = false;          // Light barrier calibration status
  int threshold = 0;                // Light barrier threshold
  const int threshold_offset = 150; // Threshold offset
  const int num_calibrate = 10;    // Number of values that are middled
  int idx = 0;                      // Index variable
  int value = 0;

void setup() 
{
  Serial.begin(9600); // Initialize Serial
  
  // Decleare digital input pins:
  pinMode(bstart_Pin, INPUT);       // Button Start
  pinMode(ledPin, OUTPUT);          // Indication LED
  pinMode(pResistor, INPUT);        // Set pResistor 
}

void calibrate()
{
  Serial.println(value);
  threshold = threshold + value;
  idx = idx + 1;
  if (idx >= num_calibrate){
    threshold = threshold / num_calibrate;
    Serial.println("Sensor calbirated");
    Serial.println(threshold);
    threshold = threshold - threshold_offset;
    calibrated = true; 
  }
  delay(100);
}

void counter(){
  if (value < threshold){
    count_State = HIGH;
  }
  else{
    count_State = LOW;
  }
  // compare the bcount_State to its previous state
  if (count_State != count_lastState) {
    // if the state has changed, increment the counter
    if (count_State == HIGH) 
    {
      // if the current state is HIGH then the button went from off to on:
      count++;
    }
  }
  // save the current state as the last state, for next time through the loop
  count_lastState = count_State;
}

void ts1_on()   // Starts Transport System 1
{
  mo1.setDirection(mo1_mD);
  mo1.setSpeed(mo1_mS);
  mo2.setDirection(mo2_mD);
  mo2.setSpeed(mo2_mS);
}

void loop() 
{  
  //Serial.println(mode);
  
  // read the state of the pushbutton value:
  bstart_State = digitalRead(bstart_Pin);

  // Mode Switch Logic and Mode Logic that has to happen once
  if (bstart_State == HIGH && mode == 0)                      //0-1 Requirement: Button Start
  {
    mode = 1;
    Serial.println("Begin Light Barrier Calibration:");
  }
  if (calibrated && mode == 1)                                //1-2 Sensor Calibrated
  {
    mode = 2;
    ts1_on();
    digitalWrite(ledPin, HIGH);
  }
  else if (count >= count_max && mode == 2)                   // 2-3 Requirement: Counter reaches count_max
  {
    mode = 3;
    mo1.stop(80);
    mo2.stop(80);
    digitalWrite(ledPin, LOW);
    mode = 0;
    count = 0;
  }
 
  // Mode Logic that has to be run each cycle
  if (mode == 1){
    value = analogRead(pResistor);
    calibrate();
  }
  else if (mode == 2){    // Motor 1 for Transport System 1 and Divider is ON, Counter is counting goods
    value = analogRead(pResistor);
    counter();
    Serial.println(count);
    Serial.println(value);
  }
  else if (mode == 3){    // After counter reaches desired value the transportsystem 1 stops, counter is resette
  }
  else if (mode == 4){
    // Todo implment Actuators for Mode 4
  }
  else if (mode == 5){
    // Todo implment Actuators for Mode 5
  }
  
  
}