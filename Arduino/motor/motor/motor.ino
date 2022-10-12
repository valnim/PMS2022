
// Motor control digital output pins defined as global constants
  const int controlPin1A = 2;                
  const int controlPin2A = 5;                 
  const int ENablePin = 9;                  

// Motor control global variables: 
  int motorSpeed;                          // Motor speed 0..255
  bool motorDirection;                      // Forward (True) or reverse (False)
  int idx;
  
void setup() 
{
  idx = 0;
  motorSpeed = 255;
  motorDirection = true;
  // Declare digital output pins:
  pinMode(controlPin1A, OUTPUT);      // 1A
  pinMode(controlPin2A, OUTPUT);      // 2A
  pinMode(ENablePin, OUTPUT);         // EN1,2
 
  digitalWrite(ENablePin, LOW);       // motor off
  Serial.begin(9600);
}

void SetMotorControl(bool mD, int mS )
{
 if (mD)            // Forward
   {
      digitalWrite(controlPin1A, HIGH);
      digitalWrite(controlPin2A, LOW);
      Serial.write("vor");
      
   }
 else                                 // Reverse
   {
      digitalWrite(controlPin1A, LOW);
      digitalWrite(controlPin2A, HIGH);
      //Serial.write("reverse");
   } 
 analogWrite(ENablePin, mS);  // Speed
}

void SetMotorStop(bool mD, int reverseTime)
{
  if (mD)
  {
    digitalWrite(controlPin1A, LOW);
    digitalWrite(controlPin2A, HIGH);
    delay(reverseTime);
    digitalWrite(controlPin2A, LOW);
  }
  else
  {
    digitalWrite(controlPin1A, HIGH);
    digitalWrite(controlPin2A, LOW);
    delay(reverseTime);
    digitalWrite(controlPin1A, LOW);
  }
  analogWrite(ENablePin, 0);
}

void loop() 
{  
 //Motorsteuerung Versuch
 //delay(200);
 //motorSpeed +=10;
 //motorSpeed = 255;
// if (motorSpeed > 240)
// {
//  //delay(5000);
//  motorSpeed = 0;
//  if(motorDirection)
//  {
//    motorDirection = false;
//  }
//  else
//  {
//    motorDirection = true;
//  }
// }
 SetMotorControl(motorDirection, motorSpeed);
 delay(2000);
 SetMotorStop(motorDirection, 80);
 if (motorDirection)
 {
  motorDirection = false;
 }
 else
 {
  motorDirection = true;
 }
 delay(1000);
 
}
