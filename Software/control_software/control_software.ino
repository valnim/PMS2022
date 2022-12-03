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
const int rs = A0, en = A1, d4 = A2, d5 = A3, d6 = A4, d7 = A5; 

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

//Stepper Z: Transpor System 1/conveyor belt Axis
#define stepPinZ PF4    // alt 4
#define dirPinZ 7
#define limitZ 11

// define Shield Pins for Spindle-Axis A (using pins D12 and D13) not used currently
#define stepPinA 12
#define dirPinA 13

//Enable Outputs of all Stepper Drivers
#define stepperEnable 8     

//Steps Per Revolution of Steppers
#define stepsPerRevX 200
#define stepsPerRevY 200
#define stepsPerRevZ 200
#define stepsPerRevA 200

// Motor variables
// Crane Lift Motor
int moXSpeed = 1000;                    // Motor X Base Speed 
int moXMaxSpeedMult = 2;                // Motor X Max Speed Multiplier
int moXAccel = 20000;                   // Motor X Acceleration
int moXDirection = -1;                  // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise)

// Crane Rot Motor
int moYSpeed = 600;                     // Motor Y Base Speed 
int moYMaxSpeedMult = 2;                // Motor Y Max Speed Multiplier
int moYAccel = 20000;                   // Motor Y Acceleration
int moYDirection = -1;                  // Motor Y Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise)

// Transport System 1 Motor 1
int moZSpeed = 400;                     // Motor Z Base Speed 
int moZMaxSpeedMult = 2;                // Motor Z Max Speed Multiplier
int moZAccel = 20000;                   // Motor Z Acceleration
int moZDirection = -1;                  // Motor Z Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise)

// Transport System 1 Motor 2
int moASpeed = 400;                     // Motor A Base Speed 
int moAMaxSpeedMult = 2;                // Motor A Max Speed Multiplier
int moAAccel = 20000;                   // Motor A Acceleration
int moADirection = -1;                  // Motor A Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise)

// Stepper Motor Positions
const int phiPos1 = stepsPerRevY*2;     // Position 1 for Motor 2 in Phi-Axis
const int phiPos2 = stepsPerRevY*8.2;   // Position 2 for Motor 2 in Phi-Axis
const int xPos1 = stepsPerRevX*40;      // Position 1 for Motor 1 in Lift-Axis
const int xPos2 = stepsPerRevX*2.5;     // Position 2 for Motor 1 in Lift-Axis

//initialize the stepper motors as existing objects
AccelStepper stepperX(1, stepPinX, dirPinX);    // Crane Lift Motor
AccelStepper stepperY(1, stepPinY, dirPinY);    // Crane Rotation Motor
AccelStepper stepperZ(1, stepPinZ, dirPinZ);    // Transport System 1 Motors 1
AccelStepper stepperA(1, stepPinA, dirPinA);    // Transport System 1 Motors 2

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
const int countBoxMax = 3;		    // Boxes to be filled

bool calibrated = false;          // Light barrier calibration status
int threshold = 0;                // Light barrier threshold
const int thresholdOffset = 140;  // Threshold offset, the lower the offset the higher the sensitivity
const int numCalibrate = 10;      // Number of values that are middled
int idx = 0;                      // Index variable
int barrierValue = 0;             // Light barrier sensor value

bool limitXState = false;         // Limit Switch X State Variable
bool limitYState = false;         // Limit Switch Y State Variable
bool bStartState = false;         // Start Button State Variable, Only true in the cylce the button is pressed
bool paused = false;              // Pause Mode Variable


// The setup() function is called once when the program starts.
// It is used to initialize the serial port, LCD, input and output pins,
// and the parameters for the stepper motors.
// It also initializes the bStart button, sets the initial state for the LEDs,
// and prints a message to the LCD indicating that setup is complete.
// Finally, it attaches interrupt handlers for the pause and stop buttons.
void setup() {
  // Initialize the serial port
  Serial.begin(9600);

  // Initialize the LCD
  lcd.begin(16,2);

  // Set the input and output modes for the relevant pins
  pinMode(limitX, INPUT_PULLUP);
  pinMode(limitY, INPUT_PULLUP);
  pinMode(photoRes, INPUT);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(stepperEnable, OUTPUT);

  // Initialize the bStart button
  bStart.begin();
  
  // Initialize the stepper motor parameters
  stepperX.setAcceleration(moXAccel);
  stepperX.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperX.setSpeed(0);

  stepperY.setAcceleration(moYAccel);
  stepperY.setMaxSpeed(moYDirection*moYSpeed*2);
  stepperY.setSpeed(0);

  stepperZ.setAcceleration(moZAccel);
  stepperZ.setMaxSpeed(moZDirection*moZSpeed*2);
  stepperZ.setSpeed(0);

  stepperA.setAcceleration(moAAccel);
  stepperA.setMaxSpeed(moADirection*moASpeed*2);
  stepperA.setSpeed(0);

  // Set the Enable Pin for ALL Stepper motors
  stepperX.setEnablePin(stepperEnable);
  
  // Print a message to the serial console indicating that setup is complete
  Serial.println("Setup finished");
  
  // Turn on the red LED to indicate that setup is complete
  LED(1);

  // Print a message to the LCD asking the user to press the start button
  lcd.setCursor(0,0);
  lcd.print("Setup finished");
  lcd.setCursor(0,1);
  lcd.print("press Start");

  // Attach interrupt handlers for the pause and stop buttons
  attachInterrupt(digitalPinToInterrupt(button2), pause, RISING);
  attachInterrupt(digitalPinToInterrupt(button3), stop, RISING);
}


// The calibrate_photoresistor() function calibrates the light barrier
// by calculating a threshold value that is used to determine when the barrier is triggered.
// The threshold is calculated as the arithmetic mean of `numCalibrate` measurements,
// with an offset defined by `thresholdOffset`.
//
// After `numCalibrate` calls to this function, the `calibrated` flag is set to `true`.
void calibrate_photoresistor()
{
  // Print the current barrier value to the serial console
  Serial.println(barrierValue);
  
  // Add the current barrier value to the threshold sum
  threshold = threshold + barrierValue;
  
  // Increment the index
  idx = idx + 1;
  
  // If the index has reached numCalibrate, calculate the mean threshold value
  // and set the calibrated flag to true
  if (idx >= numCalibrate){
    threshold = threshold / numCalibrate;
    
    Serial.println("Sensor calbirated");
    Serial.println(threshold);
    
    // Print a message to the LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Sensor Calibrated");
    
    // Subtract the threshold offset
    threshold = threshold - thresholdOffset;
    calibrated = true; 
  }

  // Delay for 100 milliseconds
  delay(100);
}


// The LED() function sets the state of the red, yellow, and green LEDs
// based on the value of the LED_mode parameter.
//
// @param {number} LED_mode - An integer representing the desired LED state.
//                            1 = red LED on, yellow and green LEDs off
//                            2 = yellow LED on, red and green LEDs off
//                            3 = green LED on, red and yellow LEDs off
//                            Any other value = all LEDs off
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


// The count_goods() function handles the detection of goods using edge detection.
// If the barrier sensor value falls below the threshold, an item is counted.
// The function compares the current state of the sensor to its previous state,
// and increments the counter if the state has changed from LOW to HIGH.
//
// The current count value is printed to the serial console and LCD display.
void count_goods()
{   
  // If the barrier sensor value is below the threshold, set the count state to HIGH
  if (barrierValue < threshold){        
    countState = HIGH;
  }
  // Otherwise, set the count state to LOW
  else{
    countState = LOW;
  }

  // compare the countState to its previous state
  if (countState != countLastState) {
    // If the state has changed, increment the counter
    if (countState == HIGH) 
    {
      // If the current state is HIGH, then the sensor went from off to on
      countVar++;
      
      // Print the current count value to the serial console
      Serial.print("Boxlevel: ");
      Serial.println(countVar);

      // Print the current count value to the LCD display
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Modus: ");
      lcd.print(mode, 1);
      lcd.setCursor(0,2);
      lcd.print("actual goods: ");
      lcd.print(countVar, 1);  
    }
  }
  // Save the current state as the last state, for the next iteration of the loop
  countLastState = countState;
}


// The loop() function is the main loop of the program, which controls the overall flow of the system.
// It contains a series of if-else statements that determine which mode the system is in and what actions to take.
// Each mode corresponds to a different stage in the operation of the system,
// such as calibrating the light barrier, dividing the goods, or moving to a certain position.
void loop()
{      
  // Read the current state of the limit switches and start button
  limitXState = digitalRead(limitX);
  limitYState = digitalRead(limitY);
  bStartState = bStart.pressed();

  // Mode Switch Logic and Mode Logic that has to happen once
  if (bStartState && mode == 0 && !paused) {
    // Mode 1: Check if the system is safe to start
    // Requirement: Button Start must be pressed
    Serial.println("Is System Safe to Start?");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Is System safe?");
    lcd.setCursor(1,0);
    lcd.print(mode, 1);
    calibrated = false;
    delay(200);

    LED(2);
    mode = mode + 1;
  }
  else if (((!limitXState && !limitYState && bStartState && mode == 1) || mode == 10) && !paused) {
    // Mode 2: Calibrate the light barrier
    // Requirement: Button Start must be pressed and the limit switches must not be pressed
    Serial.println("Begin Light Barrier Calibration:");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Begin");
    lcd.setCursor(1,0);
    lcd.print("Calibration");
    
    //attachInterrupt(digitalPinToInterrupt(limitX), stop, RISING);
    //attachInterrupt(digitalPinToInterrupt(limitY), stop, RISING);
    
    mode = 2;
  }
  else if (calibrated && mode == 2 && !paused) {
    // Mode 3: Divide the goods into boxes
    // Requirement: The light barrier must be calibrated
    stepperX.disableOutputs();
    Serial.println("Start of Box filling");

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Start");
    lcd.setCursor(1,0);
    lcd.print("Box Filling");

    LED(3);
    stepperZ.setSpeed(moZDirection*moZSpeed);
    stepperA.setSpeed(moADirection*moASpeed);
    mode = mode + 1;
  }
  else if (countVar >= countMax && mode == 3 && !paused) {
    // Mode 4: Reference the rotation axis
    // Requirement: The number of counted items must be greater than or equal to the maximum number of items per box
    stepperZ.setSpeed(0);
    stepperA.setSpeed(0);

    stepperY.setSpeed(-moYDirection*moYSpeed);

    Serial.println("Box filled ... delivering...");

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Box filled");
    lcd.setCursor(1,0);
    lcd.print("Start delivering");

    //detachInterrupt(digitalPinToInterrupt(limitY));
    //detachInterrupt(digitalPinToInterrupt(limitX));

    countVar = 0;
    mode = mode + 1;
  }
  else if (limitYState && mode == 4 && !paused) {
    // Mode 5: Reference the lift axis
    // Requirement: Limit Switch Rot reached
    stepperY.setSpeed(0);
    stepperY.setCurrentPosition(0);

    stepperX.setSpeed(-moXDirection*moXSpeed);
    
    mode = mode + 1;
  }
  else if (limitXState && mode == 5 && !paused) {
    // Mode 6: Move to phiPos1
    // Requirement: Limit Switch Lift reached
    stepperX.setSpeed(0);
    stepperX.setCurrentPosition(0);

    stepperY.setSpeed(moYDirection*moYSpeed);

    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) >= phiPos1 && mode == 6 && !paused) {
    // Mode 7: Move to xPos1
    // Requirement: Rotation Position 1 reached
    stepperY.setSpeed(0);

    stepperX.setSpeed(moXDirection*moXSpeed);
    
    //attachInterrupt(digitalPinToInterrupt(limitX), stop, RISING);
    //attachInterrupt(digitalPinToInterrupt(limitY), stop, RISING);

    mode = mode + 1;
  }
  else if (abs(stepperX.currentPosition()) >= xPos1 && mode == 7 && !paused) { 
    // Mode 8: Move to phiPo2
    // Requirement: Lift Position 1 reached
    stepperX.setSpeed(0);

    stepperY.setSpeed(moYDirection*moYSpeed);
    
    mode = mode + 1;
  }
  else if (abs(stepperY.currentPosition()) >= phiPos2 && mode == 8 && !paused) {
    // Mode 9: Move to xPos2
    // Requirement: Rotation Position 2 reached
    stepperY.setSpeed(0);

    stepperX.setSpeed(-moXDirection*moXSpeed);
    
    mode = mode + 1;
  }
  else if (abs(stepperX.currentPosition()) <= xPos2 && mode == 9 && !paused) {
    // Mode 10: Check Box count
    // Requirement: Lift Position 2 reached
    stepperX.setSpeed(0);
    countBox = countBox + 1;
    calibrated = false;
    idx = 0;
    threshold = 0;
    stepperX.enableOutputs();    // Disables All Steppers

    if (countBox < countBoxMax){
      Serial.print("Box delivered: ");
      Serial.println(countBox);

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Box delivered");
      lcd.setCursor(1,0);
      lcd.print("Box count: ");
      lcd.print(countBox, 1);  
      mode = 10;
    }
    else{
      mode = 0;
      countBox = 0;
      Serial.println("Last Box delivered");

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("All boxes");
      lcd.setCursor(1,0);
      lcd.print("delivered");

      LED(1);
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
      lcd.clear();      
    }
  }
  
  // Mode Logic that has to be run each cycle
  if (mode == 2 && !paused){
    barrierValue = analogRead(photoRes);
    calibrate_photoresistor();              // call of calibration algorithm
  }
  else if (mode == 3 && !paused){    
    barrierValue = analogRead(photoRes);    // current light barrier sensor value
    count_goods();                          // counter logic checks if light barrier detects goods 
  }
}


// The pause() function is an interrupt handler that is called when the pause button is pressed.
// It stops all the motors and saves the current state of the system.
// The system can be resumed by pressing the start button.
// The function also prints a message to the serial console and LCD display,
// and turns on the yellow LED to indicate that the system is paused.
void pause() {
  // Stop all the motors
  stepperX.enableOutputs();
  paused = true;

  // Print a message to the serial console
  Serial.println("Pause");

  // Print a message to the LCD display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Paused");

  // Turn on the yellow LED to indicate that the system is paused
  LED(2);
}


// The stop() function is an interrupt handler that is called when the stop button is pressed.
// It stops all the motors, resets all the variables, and sets the system into the initial mode.
// The function also prints a message to the serial console and LCD display,
// and turns on the red LED to indicate that the system is stopped.
void stop() {
  // Stop all the motors
  stepperX.setSpeed(0);
  stepperX.runSpeed();
  stepperY.setSpeed(0);
  stepperY.runSpeed();
  stepperZ.setSpeed(0);
  stepperZ.runSpeed();
  stepperA.setSpeed(0);
  stepperA.runSpeed();
  stepperX.enableOutputs();

  // Reset all the variables
  calibrated = false;
  idx = 0;
  threshold = 0;
  mode = 0;

  // Print a message to the serial console
  Serial.println("Stopp");

  // Print a message to the LCD display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Stopped");

  // Turn on the red LED to indicate that the system is stopped
  LED(1);
}
