#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <Button.h>

// Buttons
#define button1 PA12  //Button Start / System is Safe to Start
#define button2 PA11  //Button Pause
#define button3 PB12  //Button Emergency Stop

// Indicator Leds
#define ledGreen PC8  //Green LED
#define ledYellow PC6 //Yellow LED
#define ledRed PC5    //Red LED

//Define LCD pins
#define rs PB7
#define en PC13
#define d4 PC14
#define d5 PC15
#define d6 PF0
#define d7 PF1
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

// Counter System Components
#define photoRes PB2     // Photo Resistor Pin
//#define laserDiode PB1   // Optional Laser Diode Pin Currently Always ON

// CNC Shield Pins
// Stepper X: Crane Lift Axis
#define stepPinX 2
#define dirPinX 5
#define limitX 9

//Stepper Y: Crane Rot/Phi Axis 
#define stepPinY 3
#define dirPinY 6
#define limitY 10

//Stepper Z: Elego Direct Current Motors, Linked to Pins 1 & 4 (Black & Blue)
#define stepPinZ 4
#define dirPinZ 7
#define limitZ 11

// define Shield Pins for Spindle-Axis A (using pins D12 and D13) not used currently
//#define stepPinA 12
//#define dirPinA 13

//Enable Outputs of all Stepper Drivers
#define stepperEnable 8     

//Reset Pin of Stepper Drivers
#define driverZReset PC9    // Resets Stepper Driver Z

//Steps Per Revolution of Steppers
#define stepsPerRevX 200
#define stepsPerRevY 200

// Motor variables
// Crane Lift Motor
int moXSpeed = 350;                     // Motor X Base Speed 
int moXMaxSpeedMult = 2;                // Motor X Max Speed Multiplier
int moXAccel = 20000;                   // Motor X Acceleration
int moXDirection = -1;                  // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int moXInitDistance = 2* stepsPerRevX;  // Motor X Initialization Distance
// Crane Rot Motor
int moYSpeed = 300;                     // Motor X Base Speed 
int moYMaxSpeedMult = 2;                // Motor X Max Speed Multiplier
int moYAccel = 20000;                   // Motor X Acceleration
int moYDirection = -1;                  // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int moYInitDistance = 1* stepsPerRevY;  // Motor X Initialization Distance

// Stepper Motor Positions
const int phiPos1 = stepsPerRevY*1;   // Position 1 for Motor 2 in Phi-Axis
const int phiPos2 = stepsPerRevY*6;   // Position 2 for Motor 2 in Phi-Axis
const int xPos1 = stepsPerRevX*15;    // Position 1 for Motor 1 in Z-Axis
const int xPos2 = stepsPerRevX*10;    // Position 2 for Motor 1 in Z-Axis

//initialize the stepper motors as existing objects
AccelStepper stepperX(1, stepPinX, dirPinX);    // Crane Lift Motor
AccelStepper stepperY(1, stepPinY, dirPinY);    // Crane Rotation Motor
AccelStepper stepperZ(1, stepPinZ, dirPinZ);    // Elego Motor --> move to seperate Library

//intialize the Buttons as ojects
Button bStart(button1)  // Button Start
Button bPause(button2)  // Button Pause
Button bStop(button3)   // Button Stop

// General Variables
int mode = 0;                     // Current SFC Mode State

//Counter
int countVar = 0;                // Counter Variable for balls in a Box
int countMax = 20;               // Max goods count in Transport Box
int countState = 0;              // Button Count Default State
int countLastState = 0;          // Button Count Last State
int countBox = 0;                // Counter Variable for filled Boxes
const int countBoxMaxs = 2;		   // Boxes to be filled

bool calibrated = false;          // Light barrier calibration status
int threshold = 0;                // Light barrier threshold
const int thresholdOffset = 100;  // Threshold offset, the lower the offset the higher the sensitivity
const int numCalibrate = 10;      // Number of values that are middled
int idx = 0;                      // Index variable
int barrierValue = 0;             // Light barrier sensor value

HardwareSerial Serial1(PA10, PA9);

void setup() 
{ 
  // initialize the serial port
  Serial.begin(9600);
  
  //Initialize LCD
  lcd.begin(16,4);
  
  // Initialize Inputs
  pinMode(limitX, INPUT);
  pinMode(limitY, INPUT);
  pinMode(limitZ, INPUT);
  pinMode(photoRes, INPUT);
  
  // Initialize Buttons
  bStart.begin();
  bPause.begin();
  bStop.begin();
 
  //Initialize Outputs  
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(laserDiode, OUTPUT);
  pinMode(stepperEnable, OUTPUT);
  pinMode(driverZReset, OUTPUT);
  
  stepperX.setEnablePin(stepperEnable);

  stepperX.setAcceleration(moXAccel);
  stepperX.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperX.setSpeed(0);

  stepperY.setAcceleration(moXAccel);
  stepperY.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperY.setSpeed(0);

}


void calibrate_photoresistor()    // Before each counting cycle the light barrier is calibrated
{
  Serial.println(barrierValue);
  threshold = threshold + barrierValue;
  idx = idx + 1;
  if (idx >= numCalibrate){                      // The arithmetic median of numCalibrate is calculated
    threshold = threshold / numCalibrate;
    Serial.println("Sensor calbirated");
    Serial.println(threshold);
    threshold = threshold - thresholdOffset;     // The Sensor detection threshold is the median minus a pre determined offset
    calibrated = true; 
  }
  delay(100);
}


void counter()      // If the barrier sensor value falls below the threshold an item is counted.
{   
  if (barrierValue < threshold){        
    countState = HIGH;
  }
  else{
    countState = LOW;
  }
  // compare the countState to its previous state
  if (countState != countLastState) {
    // if the state has changed, increment the counter
    if (countState == HIGH) 
    {
      // if the current state is HIGH then the button went from off to on:
      countVar++;
    }
  }
  // save the current state as the last state, for next time through the loop
  countLastState = countState;
}


void loop()
{  
    // Mode Switch Logic and Mode Logic that has to happen once
  if (bStart.pressed() && mode == 0)                      //Mode 1, Requirement: Button Start
  {
    Serial.println("Is System Safe to Start?");
    // TODO Implement Display on LCD and LED handlng
    mode = mode + 1;
  }
  else if (bStart.pressed() && mode == 1)                 //Mode 2, Requirement: Button Start
  {
    Serial.println("Initializing Z-Axis");
    // Move upwards in Z-Direction to unpress limit switch
    stepperX.setSpeed(moXDir*moXSpeed/2);
    stepperX.setCurrentPosition(0);
    calibrated = false;
    mode = mode + 1;  // Switch mode
  }
  else if (abs(stepperX.currentPosition()) >= moXInitDistance && mode == 2)   //Mode 3, Requirement: moXInitDistance reached
  {
    stepperX.stop();
    Serial.println("Initializing Phi-Axis");
    // Move Clockwise to unpress limit switch
    stepperY.setSpeed(moYDir*moYSpeed/2);
    stepperY.setCurrentPosition(0);
    mode = mode + 1;  // Switch mode
  }
  else if (abs(stepperY.currentPosition()) >= moYInitDistance  && mode == 3)  //Mode 4, Requirement: moYInitDistance reached
  {
    
    stepperY.stop();
    barrierValue = analogRead(photoRes); // current light barrier sensor value
    calibrate_photoresistor();                          // call of calibration algorithm
    
    mode = mode + 1;
  }
  else if (calibrated && mode == 4)   //Mode 5, Requirement: 
  {
    // Implment Logic
    
    mode = mode + 1;
  }
  else if (temp && mode == 5)   //Mode 6, Requirement: 
  {
    // Implment Logic
    
    mode = mode + 1;
  }
  else if (temp && mode == 6)   //Mode 7, Requirement: 
  {
    mode = 2;
    ts1_on();   // Turn the motors of Transporation System 1 on
  }
  else if (countVar >= countMax && mode == 2)                   // 2-3 Requirement: Counter reaches countMax
  {
    mode = 3;
    //mo1.stop(80); // Stop motor1  of TS1
    //mo2.stop(80);
    mode = 0;
    count = 0;
    countBox++;
  }
 
  // Mode Logic that has to be run each cycle
  if (mode == 1){
    
    reference_crane();                    // start of crane positioning

  }
  else if (mode == 2){    // Motor 1 for Transport System 1 and Divider is ON, Counter is counting goods
    barrierValue = analogRead(photoRes);   // current light barrier sensor value
    counter();                              // counter logic checks if light barrier detects goods
    Serial.println(countVar);
    Serial.println(barrierValue);
    
    lcd.setCursor(0,0);     //example of LCD-output (in this case only mode)
    lcd.print("Modus: ");
    lcd.print(mode, 1);
    lcd.setCursor(0,2);
    lcd.print("actual goods: ");
    lcd.print(count, 1); 

  }
  
}

void elego() 
{
  // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(limitX) == HIGH && mode == 0)                      //0-1 Requirement: Button Start

  {
    stepperX.disableOutputs();

    stepperX.setSpeed(moXDirection*moXSpeed);

    Serial.println();
    mode = mode+1; // Switch Mode

    digitalWrite(stepPinZ,HIGH);
    delay(10);
    digitalWrite(stepPinZ,LOW);
  }
  else if (digitalRead(limitY) && mode == 1)                              //1-2 Reference Sensor Reached
  {
    
    digitalWrite(stepPinZ,HIGH);
    delay(10);
    digitalWrite(stepPinZ,LOW);
    delay(10);
    digitalWrite(stepPinZ,HIGH);
    delay(10);
    digitalWrite(stepPinZ,LOW);
    delay(50);
    digitalWrite(stepPinZ,HIGH);
    delay(10);
    digitalWrite(stepPinZ,LOW);

    mode = 0 ; // Switch Mode
    
  }
  
  Serial.println(mode);
  if (mode == 1){
    stepperX.runSpeed();
  }
}