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
const int rs = PB7, en = PC13, d4 = PC14, d5 = PC15, d6 = PF0, d7 = PF1; 
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

// Counter System Components
#define photoRes PC4     // Photo Resistor Pin

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

//Steps Per Revolution of Steppers
#define stepsPerRevX 200
#define stepsPerRevY 200
#define stepsPerRevZ 200

// Motor variables
// Crane Lift Motor
int moXSpeed = 400;                     // Motor X Base Speed 
int moXMaxSpeedMult = 2;                // Motor X Max Speed Multiplier
int moXAccel = 20000;                   // Motor X Acceleration
int moXDirection = -1;                  // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction

// Crane Rot Motor
int moYSpeed = 400;                     // Motor X Base Speed 
int moYMaxSpeedMult = 2;                // Motor X Max Speed Multiplier
int moYAccel = 20000;                   // Motor X Acceleration
int moYDirection = -1;                  // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction

// Transport System 1 Motors
int moZSpeed = 400;                     // Motor X Base Speed 
int moZMaxSpeedMult = 2;                // Motor X Max Speed Multiplier
int moZAccel = 20000;                   // Motor X Acceleration
int moZDirection = -1;                  // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction

// Stepper Motor Positions
const int phiPos1 = stepsPerRevY*1;   // Position 1 for Motor 2 in Phi-Axis
const int phiPos2 = stepsPerRevY*6;   // Position 2 for Motor 2 in Phi-Axis
const int xPos1 = stepsPerRevX*6;    // Position 1 for Motor 1 in Z-Axis
const int xPos2 = stepsPerRevX*1;    // Position 2 for Motor 1 in Z-Axis

//initialize the stepper motors as existing objects
AccelStepper stepperX(1, stepPinX, dirPinX);    // Crane Lift Motor
AccelStepper stepperY(1, stepPinY, dirPinY);    // Crane Rotation Motor
AccelStepper stepperZ(1, stepPinZ, dirPinZ);    // Transport System 1 Motors

//intialize the Buttons as ojects
Button bStart(button1);  // Button Start

// General Variables
int mode = 0;                     // Current SFC Mode State

//Counter
int countVar = 0;                 // Counter Variable for balls in a Box
int countMax = 5;                 // Max goods count in Transport Box
int countState = 0;               // Button Count Default State
int countLastState = 0;           // Button Count Last State
int countBox = 0;                 // Counter Variable for filled Boxes
const int countBoxMax = 2;		    // Boxes to be filled

bool calibrated = false;          // Light barrier calibration status
int threshold = 0;                // Light barrier threshold
const int thresholdOffset = 100;  // Threshold offset, the lower the offset the higher the sensitivity
const int numCalibrate = 10;      // Number of values that are middled
int idx = 0;                      // Index variable
int barrierValue = 0;             // Light barrier sensor value

bool limitXState = false;         // Limit Switch X State Variable
bool limitYState = false;         // Limit Switch Y State Variable
bool bStartState = false;         // Start Button State Variable, Only true in the cylce the button is pressed
bool paused = false;              // Pause Mode Variable

void setup() 
{ 
  // initialize the serial port
  Serial.begin(9600);
  
  //Initialize LCD
  lcd.begin(16,2);
  lcd.print("Hello");
  
  // Initialize Inputs
  pinMode(limitX, INPUT_PULLUP);
  pinMode(limitY, INPUT_PULLUP);

  pinMode(photoRes, INPUT);
  
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  
  // Initialize Buttons
  bStart.begin();
 
  //Initialize Outputs  
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(stepperEnable, OUTPUT);
  
  stepperX.setEnablePin(stepperEnable);

  stepperX.setAcceleration(moXAccel);
  stepperX.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperX.setSpeed(0);

  stepperY.setAcceleration(moYAccel);
  stepperY.setMaxSpeed(moYDirection*moYSpeed*2);
  stepperY.setSpeed(0);

  stepperZ.setAcceleration(moZAccel);
  stepperZ.setMaxSpeed(moZDirection*moZSpeed*2);
  stepperZ.setSpeed(0);
  
  Serial.println("Setup Finished");
  
  LED(2);

  lcd.setCursor(0,0);     //example of LCD-output (in this case only mode)
  lcd.print("Setup finished");
  lcd.setCursor(1,0);
  lcd.print("press Start");

  //attachInterrupt(digitalPinToInterrupt(button2), pause, RISING);
  //attachInterrupt(digitalPinToInterrupt(button3), stop, RISING);
}

void calibrate_photoresistor()    // Before each counting cycle the light barrier is calibrated
{
  Serial.println(barrierValue);
  threshold = threshold + barrierValue;
  idx = idx + 1;
  if (idx >= numCalibrate){
    threshold = threshold / numCalibrate;
    Serial.println("Sensor calbirated");
    Serial.println(threshold);
    threshold = threshold - thresholdOffset;
    calibrated = true; 
  }
  delay(100);
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

void count_goods()      // If the barrier sensor value falls below the threshold an item is counted.
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
      Serial.println(countVar);
    }
  }
  // save the current state as the last state, for next time through the loop
  countLastState = countState;
}

void loop()
{      
  limitXState = digitalRead(limitX);
  limitYState = digitalRead(limitY);
  bStartState = bStart.pressed();

  // Mode Switch Logic and Mode Logic that has to happen once
  if (bStartState && mode == 0 && !paused)                      //Mode 1 Safe to Start, Requirement: Button Start

  {
    Serial.println("Is System Safe to Start?");
    
    // TODO Implement Display on LCD and LED handling
    lcd.setCursor(0,0);
    lcd.print("Is System safe?");
    lcd.setCursor(1,0);
    lcd.print(mode, 1);
    calibrated = false;
    delay(200);

    LED(2);
    mode = mode + 1;
  }
  else if (((!limitXState && !limitYState && bStartState && mode == 1) || mode == 10) && !paused)   //Mode 2 Calibrate Light Barrier, Requirement: Button Start
  {  
    Serial.println("Begin Light Barrier Calibration:");
    
    //attachInterrupt(digitalPinToInterrupt(limitX), stop, RISING);
    //attachInterrupt(digitalPinToInterrupt(limitY), stop, RISING);
    
    mode = 2;
  }
  else if (calibrated && mode == 2 && !paused)   //Mode 3 Divide Goods, Requirement: Ligth Barrier calibrated
  {
    stepperX.disableOutputs();
    Serial.println("Start of Box filling");
    LED(3);
    stepperZ.setSpeed(moZDirection*moZSpeed);
    mode = mode + 1;
  }
  else if (countVar >= countMax && mode == 3 && !paused)   //Mode 4 Roation Ref, Requirement: countVar >= countMax
  {
    stepperZ.setSpeed(0);

    stepperY.setSpeed(-moYDirection*moYSpeed);

    //detachInterrupt(digitalPinToInterrupt(limitY));
    //detachInterrupt(digitalPinToInterrupt(limitX));

    countVar = 0;
    mode = mode + 1;
  }
  else if (limitYState && mode == 4 && !paused)   //Mode 5 Lift Ref, Requirement: Limit Switch Rot reached
  {
    stepperY.setSpeed(0);
    stepperY.setCurrentPosition(0);

    stepperX.setSpeed(-moXDirection*moXSpeed);
    
    mode = mode + 1;
  }
  else if (limitXState && mode == 5 && !paused)     //Mode 6 Move to phiPos1, Requirement: Limit Switch Lift reached
  {
    stepperX.setSpeed(0);
    stepperX.setCurrentPosition(0);

    stepperY.setSpeed(moYDirection*moYSpeed);

    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) >= phiPos1 && mode == 6 && !paused)     //Mode 7 Move to xPos1, Requirement: Rotation Position 1 reached
  {
    stepperY.setSpeed(0);

    stepperX.setSpeed(moXDirection*moXSpeed);
    
    //attachInterrupt(digitalPinToInterrupt(limitX), stop, RISING);
    //attachInterrupt(digitalPinToInterrupt(limitY), stop, RISING);

    mode = mode + 1;
  }
  else if (abs(stepperX.currentPosition()) >= xPos1 && mode == 7 && !paused)     //Mode 8 Move to phiPo2, Requirement: Lift Position 1 reached
  {
    stepperX.setSpeed(0);

    stepperY.setSpeed(moYDirection*moYSpeed);
    
    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) >= phiPos2 && mode == 8 && !paused)     //Mode 9 Move to xPos2, Requirement: Rotation Position 2 reached
  {
    stepperY.setSpeed(0);

    stepperX.setSpeed(-moXDirection*moXSpeed);
    
    mode = mode + 1;
  }
  else if (abs(stepperX.currentPosition()) <= xPos2 && mode == 9 && !paused)     //Mode 10 Move to xPos2, Requirement: Lift Position 2 reached
  {
    stepperX.setSpeed(0);
    countBox = countBox + 1;
    calibrated = false;
    idx = 0;
    threshold = 0;
    stepperX.enableOutputs();    // Disables All Steppers

    if (countBox < countBoxMax){
      Serial.println("Box delivered");
      mode = 10;
    }
    else{
      mode = 0;
      countBox = 0;
      Serial.println("Last Box delivered");
    }
  }
  
  if (!paused){
    stepperX.runSpeed();
    stepperY.runSpeed();
    stepperZ.runSpeed();
  }
  else if (paused && bStartState){
    paused = false;
    if (mode > 2){
      stepperX.disableOutputs();
    }
  }
  
  // Mode Logic that has to be run each cycle
  if (mode == 2 && !paused){
    barrierValue = analogRead(photoRes);
    Serial.println(barrierValue);
    calibrate_photoresistor();           // call of calibration algorithm
  }
  else if (mode == 3 && !paused){    
    barrierValue = analogRead(photoRes);   // current light barrier sensor value
    count_goods();                         // counter logic checks if light barrier detects goods  
  }
}

void pause(){
  stepperX.enableOutputs();
  paused = true;
  Serial.println("Pause");
}

void stop(){
  stepperX.setSpeed(0);
  stepperX.runSpeed();
  stepperY.setSpeed(0);
  stepperY.runSpeed();
  stepperZ.setSpeed(0);
  stepperZ.runSpeed();
  stepperX.enableOutputs();
  calibrated = false;
  idx = 0;
  threshold = 0;
  mode = 0;
  Serial.println("Stopp");
}