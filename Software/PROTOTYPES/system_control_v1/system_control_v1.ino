#include <Elego_Motor.h>

// Motor control digital output pins defined as global constants
  // Pins on Channel 1 of L293D
  #define mo1_controlPin1A 2             
  #define mo1_controlPin2A 5                
  #define mo1_enablePin 9     
  Elego_Motor mo1(mo1_controlPin1A, mo1_controlPin2A, mo1_enablePin);

  // Pins on Channel 2 of L293D Currently not used!
  //#define mo2_controlPin1A 8              
  //#define mo2_controlPin2A 10                
  //#define mo2_enablePin 11      
  //Elego_Motor mo2(mo2_controlPin1A, mo2_controlPin2A, mo2_enablePin);

// Input pins 
  #define bstart_Pin = 4         // Button Start Pin
  #define pResistor = A0         // Photoresistor at Arduino analog pin A0

// Motor control global variables: 
  //Motor 1 (Transportsystem 1 both motors)
  int mo1_mS = 120;                 // Motor speed 0..255
  bool mo1_mD = false;              // Forward (true) or reverse (false)
  //Motor 2
  //int mo2_mS = 120;                 // Motor speed 0..255
  //bool mo2_mD = false;              // Forward (true) or reverse (false)

// Button Variables
  int bstart_State = 0;             // Button Start Default State
  int count_State = 0;              // Button Count Default State
  int count_lastState = 0;          // Button Count Last State

// General Variables
  int mode = 0;                     // System Control Mode
  int count = 0;                    // Counter
  int count_max = 20;               // Max goods count in Transport Box

  bool calibrated = false;          // Light barrier calibration status
  int threshold = 0;                // Light barrier threshold
  const int threshold_offset = 100; // Threshold offset, the lower the offset the higher the sensitivity
  const int num_calibrate = 10;     // Number of values that are middled
  int idx = 0;                      // Index variable
  int barrierValue = 0;             // Light barrier sensor value

void setup() 
{
  Serial.begin(9600); // Initialize Serial
  
  // Decleare digital input pins:
  pinMode(bstart_Pin, INPUT);       // Button Start
  pinMode(pResistor, INPUT);        // pResistor 
}

void calibrate()    // Before each counting cycle the light barrier is calibrated
{
  Serial.println(barrierValue);
  threshold = threshold + barrierValue;
  idx = idx + 1;
  if (idx >= num_calibrate){                      // The arithmetic median of num_calibrate is calculated
    threshold = threshold / num_calibrate;
    Serial.println("Sensor calbirated");
    Serial.println(threshold);
    threshold = threshold - threshold_offset;     // The Sensor detection threshold is the median minus a pre determined offset
    calibrated = true; 
  }
  delay(100);
}

void counter(){   // If the barrier sensor value falls below the threshold an item is counted.
  if (barrierValue < threshold){        
    count_State = HIGH;
  }
  else{
    count_State = LOW;
  }
  // compare the count_State to its previous state
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
  //mo2.setDirection(mo2_mD);
  //mo2.setSpeed(mo2_mS);
}

void loop() 
{  
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
    ts1_on();   // Turn the motors of Transporation System 1 on
  }
  else if (count >= count_max && mode == 2)                   // 2-3 Requirement: Counter reaches count_max
  {
    mode = 3;
    mo1.stop(80); // Stop motor1  of TS1
    //mo2.stop(80);
    mode = 0;
    count = 0;
  }
 
  // Mode Logic that has to be run each cycle
  if (mode == 1){
    barrierValue = analogRead(pResistor); // current light barrier sensor value
    calibrate();                            // call of calibration algorithm
  }
  else if (mode == 2){    // Motor 1 for Transport System 1 and Divider is ON, Counter is counting goods
    barrierValue = analogRead(pResistor);   // current light barrier sensor value
    counter();                              // counter logic checks if light barrier detects goods
    Serial.println(count);
    Serial.println(barrierValue);
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