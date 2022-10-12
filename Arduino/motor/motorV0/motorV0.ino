// Motor control digital output pins defined as global constants
  const int controlPin1A = 2;                
  const int controlPin2A = 5;                 
  const int ENablePin = 9;                  

// Motor control global variables: 
  int motorSpeed = 0;                          // Motor speed 0..255
  int motorDirection = 1;                      // Forward (1) or reverse (0)
  
void setup() 
{
  //other stuff....
 
  // Declare digital output pins:
  pinMode(controlPin1A, OUTPUT);      // 1A
  pinMode(controlPin2A, OUTPUT);      // 2A
  pinMode(ENablePin, OUTPUT);         // EN1,2
 
  digitalWrite(ENablePin, LOW);       // motor off


}

void SetMotorControl()
{
  if (motorDirection == 1)            // Forward
   {
      digitalWrite(controlPin1A, HIGH);
      digitalWrite(controlPin2A, LOW);
   }
 else                                 // Reverse
   {
      digitalWrite(controlPin1A, LOW);
      digitalWrite(controlPin2A, HIGH);
   } 
 analogWrite(ENablePin, motorSpeed);  // Speed
}

void loop() {
  motorSpeed=200;
  // put your main code here, to run repeatedly:
  digitalWrite(controlPin1A, HIGH);
  digitalWrite(controlPin2A, LOW);
  analogWrite(ENablePin, motorSpeed);  // Speed

  delay(3000);
  motorSpeed=0;
  analogWrite(ENablePin, motorSpeed);  // Speed
  
  delay(3000);
  motorSpeed=200;
  digitalWrite(controlPin1A, LOW);
  digitalWrite(controlPin2A, HIGH);
  analogWrite(ENablePin, motorSpeed);  // Speed
  
  delay(3000);
  motorSpeed=0;
  analogWrite(ENablePin, motorSpeed);  // Speed
  
  

}
