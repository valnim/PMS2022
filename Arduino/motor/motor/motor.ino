
// Motor control digital output pins defined as global constants
  const int controlPin1A = 2;                
  const int controlPin2A = 5;                 
  const int ENablePin = 9;                  

// Motor control global variables: 
  int motorSpeed;                          // Motor speed 0..255
  bool motorDirection;                      // Forward (True) or reverse (False)
  
void setup() 
{
  //other stuff....
 motorSpeed = 0;
 motorDirection = true;
  // Declare digital output pins:
  pinMode(controlPin1A, OUTPUT);      // 1A
  pinMode(controlPin2A, OUTPUT);      // 2A
  pinMode(ENablePin, OUTPUT);         // EN1,2
 
  digitalWrite(ENablePin, LOW);       // motor off
  Serial.begin(9600);
  


}

void SetMotorControl()
{
 if (motorDirection)            // Forward
   {
      digitalWrite(controlPin1A, HIGH);
      digitalWrite(controlPin2A, LOW);
      //Serial.write("vor");
      
   }
 else                                 // Reverse
   {
      digitalWrite(controlPin1A, LOW);
      digitalWrite(controlPin2A, HIGH);
      //Serial.write("reverse");
   } 
 analogWrite(ENablePin, motorSpeed);  // Speed
}

void loop() 
{  
 //Motorsteuerung Versuch
 delay(200);
 motorSpeed +=10;

 if (motorSpeed > 240)
 {
  motorSpeed = 0;
  if(motorDirection)
  {
    motorDirection = false;
  }
  else
  {
    motorDirection = true;
  }
 }
 SetMotorControl();
 
 
}
