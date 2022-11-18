//Include libraries
#include <LiquidCrystal.h>
#include <AccelStepper.h>

//Define LCD pins
const int rs = 4, en = 3, d4 = 5, d5 = 6, d6 = 7, d7 = 8; //ÜBERARBEITEN!!!!
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); 

//Definie Interface-Buttons
const byte START = 1, STOP = 2, PAUSE = 3;                //ÜBERARBEITEN!!!
  
int bstart_State = 0;             // Button Start Default State
int count_State = 0;              // Button Count Default State

//Define sign-LEDs
const byte RED = 5, YELLOW = 6, GREEN = 7;                //ÜBERARBEITEN!!!!

//Define Photoresistor 
#define pResistor = A0;                                   //ÜBERARBEITEN!!!!

//Variables for light barrier
bool calibrated = false;          // Light barrier calibration status
int threshold = 0;                // Light barrier threshold
const int threshold_offset = 100; // Threshold offset, the lower the offset the higher the sensitivity
const int num_calibrate = 10;     // Number of values that are middled
int idx = 0;                      // Index variable
int barrierValue = 0;             // Light barrier sensor value

// CNC Shield Pins
  // define Shield Pins for Axis X to Z
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
  #define stepPinA 12
  #define dirPinA 13

//Enable Outputs of all Stepper Drivers
#define stepperEnable 8

//Steps Per Revolution of Steppers
#define stepsPerRevX 200
#define stepsPerRevY 200

// Motor variables
  // Crane Lif Motor
  int moXSpeed = 350;             // Motor X Base Speed 
  int moXMaxSpeedMult = 2;        // Motor X Max Speed Multiplier
  int moXAccel = 20000;           // Motor X Acceleration
  int moXDirection = -1;          // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
  int moXInitDistance = 2* step   // Motor X Initialization Distance
  // Crane Rot Motor
  int moYSpeed = 300;             // Motor X Base Speed 
  int moYMaxSpeedMult = 2;        // Motor X Max Speed Multiplier
  int moYAccel = 20000;           // Motor X Acceleration
  int moYDirection = -1;          // Motor X Standard Direction Variable (1 - Clockwise, -1 - Counterclockwise), Referencing happens in opposite direction
  int moYInitDistance = 1* step   // Motor X Initialization Distance

  // Stepper Motor Positions
  const int phiPos1 = stepsPerRev*1;   // Position 1 for Motor 2 in Phi-Axis
  const int phiPos2 = stepsPerRev*6;   // Position 2 for Motor 2 in Phi-Axis
  const int xPos1 = stepsPerRev*15;    // Position 1 for Motor 1 in Z-Axis
  const int xPos2 = stepsPerRev*10;    // Position 2 for Motor 1 in Z-Axis

//initialize the stepper motors as existing objects
AccelStepper stepperX(1, stepPinX, dirPinX);    // Crane Lift Motor
AccelStepper stepperY(1, stepPinY, dirPinY);    // Crane Rotation Motor
AccelStepper stepperZ(1, stepPinZ, dirPinZ);    // Elego Motor --> move to seperate Library

//???
HardwareSerial Serial1(PA10, PA9);

//General variables for Software Control
//Mode
  int mode = 0;                     // System Control Mode
//Counter
  int count_var = 0;                // Counter Variable for balls in a Box
  int count_max = 20;               // Max goods count in Transport Box
  int count_State = 0;              // Button Count Default State
  int count_lastState = 0;          // Button Count Last State
  int amount_Box = 0;               // Counter Variable for filled Boxes
//Button States
  int bSTART_State = 0;             // Button Start Default State
  int bPause_State = 0;             // Button Start Default State
  int bSTOP_State = 1;              // Button Start Default State

void setup() 
{ 
  //Initialize the serial port (debugging reasons)
  Serial.begin(9600);

  //Initialize LCD
  lcd.begin(16,4);

  //Initialize Buttons as Input
  pinMode(START, INPUT);
  pinMode(STOP, INPUT);
  pinMode(PAUSE, INPUT);

  //Initialize LED signs as Outputs
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);

  //Initialize P-Resistor as Input
  pinMode(pResistor, INPUT);

  //Initialize Shield-Limits as Inputs
  pinMode(limitX, INPUT);
  pinMode(limitY, INPUT);
  pinMode(limitZ, INPUT);

  //Initialize Steppers
  stepperX.setEnablePin(stepperEnable);
  
  pinMode(stepperEnable, OUTPUT);
  //digitalWrite(stepperEnable, HIGH);
  
  stepperX.setAcceleration(moXAccel);
  stepperX.setMaxSpeed(moXDirection*moXSpeed*2);
  stepperX.setSpeed(0);

  //stepperY.setAcceleration(moXAccel);
  //stepperY.setMaxSpeed(moXDirection*moXSpeed*2);
  //stepperY.setSpeed(0);

  stepperX.disableOutputs();
}

void calibrate_photoresistor()    // Before each counting cycle the light barrier is calibrated
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

void reference_crane()
{
  //write down code for reference of crane here
}

void ts1_on()   // Starts Transport System 1
{
  //write down code for moving TS1 here
  //old:
  mo1.setDirection(mo1_mD);
  mo1.setSpeed(mo1_mS);
}

void Crane_moving()
{
  //write down code for transporting away the box here
}

void counter()      // If the barrier sensor value falls below the threshold an item is counted.
{   
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

void loop()         //ÜBERARBEITEN bzw. zusammenführen
{  
  // read the state of the pushbutton value:
  bSTART_State = digitalRead(START);

  // Mode Switch Logic and Mode Logic that has to happen once
  if (bSTART_State == HIGH && mode == 0)                      //0-1 Requirement: Button Start
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
    //mo1.stop(80); // Stop motor1  of TS1
    //mo2.stop(80);
    mode = 0;
    count = 0;
    amount_Box++;
  }
 
  // Mode Logic that has to be run each cycle
  if (mode == 1){
    barrierValue = analogRead(pResistor); // current light barrier sensor value
    calibrate();                          // call of calibration algorithm
    reference_crane();                    // start of crane positioning

  }
  else if (mode == 2){    // Motor 1 for Transport System 1 and Divider is ON, Counter is counting goods
    barrierValue = analogRead(pResistor);   // current light barrier sensor value
    counter();                              // counter logic checks if light barrier detects goods
    Serial.println(count);
    Serial.println(barrierValue);
    
    lcd.setCursor(0,0);     //example of LCD-output (in this case only mode)
    lcd.print("Modus: ");
    lcd.print(mode, 1);
    lcd.setCursor(0,2);
    lcd.print("actual goods: ");
    lcd.print(count, 1); 

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

void loop2() 
{
  // Mode Switch Logic and Mode Logic that has to happen once
  if (digitalRead(limitX) == HIGH && mode == 0)                      //0-1 Requirement: Button Start
  {
    stepperX.disableOutputs();

    stepperX.setSpeed(moXDirection*moXSpeed);

    Serial.println();
    mode = mode+1; // Switch Mode

    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);
  }
  else if (digitalRead(limitY) && mode == 1)                              //1-2 Reference Sensor Reached
  {
    
    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);
    delay(10);
    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);
    delay(50);
    digitalWrite(stepPinY,HIGH);
    delay(10);
    digitalWrite(stepPinY,LOW);

    mode = 0 ; // Switch Mode
    
  }
  
  Serial.println(mode);
  if (mode == 1){
    stepperX.runSpeed();
  } 
}

