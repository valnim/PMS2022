#include <AccelStepper.h>
#include <LiquidCrystal.h>
#include <Button.h>
#include <Elego_Stepper.h>

// Buttons
#define button1 PA12  //Button Start / System is Safe to Start
#define button2 PA11  //Button Pause
#define button3 PB12  //Button Emergency Stop

// Indicator Leds
#define ledGreen PC8  //Green LED
#define ledYellow PC6 //Yellow LED
#define ledRed PC5    //Red LED

//Define LCD pins
//const int rs = PB7, en = PC13, d4 = PC14, d5 = PC15, d6 = PF0, d7 = PF1;
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
int moYSpeed = 350;                     // Motor X Base Speed 
int moYMaxSpeedMult = 2;                // Motor X Max Speed Multiplier
int moYAccel = 20000;                   // Motor X Acceleration
int moYDirection = -1;                  // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
int moYInitDistance = 1* stepsPerRevY;  // Motor X Initialization Distance
// Transport System 1 Motors
int reverseTimeZ = 50;

// Stepper Motor Positions
const int phiPos1 = stepsPerRevY*1;   // Position 1 for Motor 2 in Phi-Axis
const int phiPos2 = stepsPerRevY*6;   // Position 2 for Motor 2 in Phi-Axis
const int xPos1 = stepsPerRevX*15;    // Position 1 for Motor 1 in Z-Axis
const int xPos2 = stepsPerRevX*10;    // Position 2 for Motor 1 in Z-Axis

//initialize the stepper motors as existing objects
AccelStepper stepperX(1, stepPinX, dirPinX);    // Crane Lift Motor
AccelStepper stepperY(1, stepPinY, dirPinY);    // Crane Rotation Motor
//initialize the elego motor as existing object
Elego_Stepper stepperZ(stepPinZ, driverZReset);   // Transport System 1 Motor

//intialize the Buttons as ojects
Button bStart(button1);  // Button Start
Button bPause(button2);  // Button Pause
Button bStop(button3);   // Button Stop

// General Variables
int mode = 0;                     // Current SFC Mode State

//Counter
int countVar = 0;                // Counter Variable for balls in a Box
int countMax = 20;               // Max goods count in Transport Box
int countState = 0;              // Button Count Default State
int countLastState = 0;          // Button Count Last State
int countBox = 0;                // Counter Variable for filled Boxes
const int countBoxMax = 2;		   // Boxes to be filled

bool calibrated = false;          // Light barrier calibration status
int threshold = 0;                // Light barrier threshold
const int thresholdOffset = 100;  // Threshold offset, the lower the offset the higher the sensitivity
const int numCalibrate = 10;      // Number of values that are middled
int idx = 0;                      // Index variable
int barrierValue = 0;             // Light barrier sensor value

//HardwareSerial Serial1(PA10, PA9);

void setup() 
{ 
  // initialize the serial port
  Serial.begin(9600);
  
  //Initialize LCD
  lcd.begin(16,2);
  
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
  //pinMode(laserDiode, OUTPUT);
  pinMode(stepperEnable, OUTPUT);
  pinMode(driverZReset, OUTPUT);
  
  stepperX.setEnablePin(stepperEnable);

  stepperX.setAcceleration(moXAccel);
  stepperX.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperX.setSpeed(0);

  stepperY.setAcceleration(moXAccel);
  stepperY.setMaxSpeed(moYDirection*moXSpeed*2);
  stepperY.setSpeed(0);

  stepperZ.reset();
  Serial.println("Setup Finished");
  
  LED(2);
  //digitalWrite(ledRed, LOW);
  //digitalWrite(ledYellow, HIGH);
  //digitalWrite(ledGreen, LOW);

  lcd.setCursor(0,0);     //example of LCD-output (in this case only mode)
  lcd.print("Setup finished");
  lcd.setCursor(1,0);
  lcd.print("press Start");
}

void calibrate_photoresistor()    // Before each counting cycle the light barrier is calibrated
{
  while(idx<=numCalibrate)
  {
    barrierValue = analogRead(photoRes);      // current light barrier sensor value
    Serial.println(barrierValue);
    threshold = threshold + barrierValue;
    idx = idx + 1;
  }
  threshold = threshold / numCalibrate;
  Serial.println("Sensor calibrated");
  Serial.print("threshold :");
  Serial.println(threshold);
  threshold = threshold - thresholdOffset;     // The Sensor detection threshold is the median minus a pre determined offset
  calibrated = true; 
}

void LED(int LED_mode)
{
  if (LED_mode == 1)
  {
    digitalWrite(ledRed, HIGH);  
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);    
  }
  else if (LED_mode == 2)
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, HIGH);
    digitalWrite(ledGreen, LOW);
  }
  else if (LED_mode == 3)
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, HIGH);
  }
  else
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);
  }
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
  //Serial.println("Mode:");
  //Serial.println(mode);
    // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(button1) && mode == 0)                      //Mode 1 Safe to Start, Requirement: Button Start
  {
    Serial.println("Is System Safe to Start?");
    
    // TODO Implement Display on LCD and LED handling
    lcd.setCursor(0,0);
    lcd.print("Is System safe?");
    lcd.setCursor(1,0);
    lcd.print(mode, 1);
    delay(2000);

    LED(2);
    
    mode = mode + 1;
  }
  else if (digitalRead(button1) && mode == 1)                 //Mode 2 Init X, Requirement: Button Start
  {
    Serial.println("Initializing Lift-Axis");
    // Move upwards in Z-Direction to unpress limit switch
    stepperX.setCurrentPosition(0);
    stepperX.setSpeed(moXDirection*moXSpeed/2);
    stepperX.disableOutputs();    // Enables All Steppers
    calibrated = false;
    mode = mode + 1;  // Switch mode
  }
  else if (abs(stepperX.currentPosition()) >= moXInitDistance && mode == 2)   //Mode 3 Init Phi, Requirement: moXInitDistance reached
  {
    Serial.println("finished Init Lift-Axis");
    stepperX.setSpeed(0);
    //stepperX.stop();
    Serial.println("Initializing Phi-Axis");
    // Move Clockwise to unpress limit switch
    stepperY.setCurrentPosition(0);
    stepperY.setSpeed(moYDirection*moYSpeed/2);
    mode = mode + 1;  // Switch mode
  }
  else if ((abs(stepperY.currentPosition()) >= moYInitDistance  && mode == 3) || mode == 12)  //Mode 4 Calibrate Light Barrier, Requirement: moYInitDistance reached
  {
    Serial.println("finished Init Phi-Axis");
    stepperY.setSpeed(0);
    //stepperY.stop();
    
    Serial.println("Begin Light Barrier Calibration:");

    calibrate_photoresistor();           // call of calibration algorithm
    mode = mode + 1;
  }
  else if (calibrated && mode == 4)   //Mode 5 Divide Goods, Requirement: Ligth Barrier calibrated
  {
    Serial.println("Start of Box filling");
    LED(3);
    
    counter();

    stepperZ.start();
    mode = mode + 1;
  }
  else if (countVar >= countMax && mode == 5)   //Mode 6 Roation Ref, Requirement: countVar >= countMax
  {
    stepperZ.stop(reverseTimeZ);
    
    stepperY.setSpeed(-moYDirection*moYSpeed);

    countVar = 0;
    mode = mode + 1;
  }
  else if (limitY && mode == 6)   //Mode 7 Lift Ref, Requirement: Limit Switch Rot reached
  {
    stepperY.stop();
    stepperY.setCurrentPosition(0);

    stepperX.setSpeed(-moXDirection*moXSpeed);
    
    mode = mode + 1;
  }
  else if (limitX && mode == 7)     //Mode 8 Move to phiPos1, Requirement: Limit Switch Lift reached
  {
    stepperX.stop();
    stepperX.setCurrentPosition(0);

    stepperY.setSpeed(moYDirection*moYSpeed);

    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) >= phiPos1 && mode == 8)     //Mode 9 Move to xPos1, Requirement: 
  {
    stepperY.stop();

    stepperX.setSpeed(moXDirection*moXSpeed);
    
    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) >= xPos1 && mode == 9)     //Mode 10 Move to phiPo2, Requirement: 
  {
    stepperX.stop();

    stepperY.setSpeed(moYDirection*moYSpeed);
    
    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) >= phiPos2 && mode == 10)     //Mode 11 Move to xPos2, Requirement: 
  {
    stepperY.stop();

    stepperX.setSpeed(-moXDirection*moXSpeed);
    
    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) <= xPos2 && mode == 11)     //Mode 11 Move to xPos2, Requirement: 
  {
    stepperX.stop();
    countBox = countBox + 1;

    if (countBox < countBoxMax){
      mode = 12;
    }
    else{
      mode = 0;
      countBox = 0;
      stepperX.enableOutputs();    // Disables All Steppers
    }
  }
  
  stepperX.runSpeed();
  stepperY.runSpeed();
 
  // Mode Logic that has to be run each cycle
  if (mode == 0 || mode == 2 || mode == 3 || mode == 4 || mode == 6 || mode == 7 || mode == 8 || mode == 9 || mode == 10 || mode == 11){
    
  }
  else if (mode == 5 ){    
    barrierValue = analogRead(photoRes);   // current light barrier sensor value
    //counter();                              // counter logic checks if light barrier detects goods  
    //Serial.println(countVar);
    //Serial.println(barrierValue);
    
    lcd.setCursor(0,0);     //example of LCD-output (in this case only mode)
    lcd.print("Modus: ");
    lcd.print(mode, 1);
    lcd.setCursor(0,2);
    lcd.print("actual goods: ");
    lcd.print(countVar, 1); 

  }
}