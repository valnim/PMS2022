#include <LiquidCrystal.h>
#include <LCDKeypad.h>
#include <MatrixMath.h>

LCDKeypad lcd;

#include "HX711.h"
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
HX711 scale;

// vx erwartete Sensorwerte für gewählte Kalibrationsgewichte, abhängig vom Sensor
// vy wahre Gewichte der Kalibrationsgewichte
// points Anzahl der Kalbrationspunkte



//float vx[] = { 583700,  670400, 1603030};
//float vy[] = {0, 200000, 200000};
//const int points = 3;

//float vx[] = { -171570,  53894, 279200};
//float vy[] = {0, 200000, 400000};
//const int points = 3;

//Calibration Set
//float vx[] = { 581250, 669600, 756067,1015360,1331620,1602840,2350980};
//float vy[] = {0, 200000, 400000,1000000,1733000,2364000,4097000};
//const int points = 7;

float vx[] = { 581250, 669600, 756067,1331620,2350980};
float vy[] = {0, 200000, 400000,1733000,4097000};
const int points = 5;

//Test Set
float vy_test[] = {30000, 70000, 300000, 700000, 1933000, 2864000};
const int points_test = 6;

//      float vx[] = { 583700, 605615, 670371, 1332080, 2351250};
//      float vy[] = {0, 50000, 200000, 1733000, 4095000};
//      const int points = 5;

//      float vx[] = { 583700, 592649, 605615, 627142, 670371, 799972, 1332080, 1603030, 2351250};
//      float vy[] = {0, 20000, 50000, 100000, 200000, 500000, 1733000, 2364000, 4095000}; 
//      const int points = 9;

//float vx[] = { -171570, -149030,  -58860, 392350}; //adc readings, have to be adjusted for each sensor+board combo
//float vy[] = {0, 20000, 100000, 500000}; //nominal weights (micrograms) (20000/-149030), (1800000/1857650)
//const int points = 4;

const int N = 3; //Polynom Grad n + 1 (z.B. N= 3 --> 2.Ordnung)
const int M = points; //Anzahl Kalbrationspunkte

//Matrizen für Polynomiale Regression
mtx_type X0[M][N];
mtx_type T0[N][1];
mtx_type Y0[M][1];
mtx_type XT[M][N];
mtx_type XI[M][N];
mtx_type X[1][M];
mtx_type Y[1][1];



void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  matrix_init();
}

bool calibrated;          //Toggles between uncalibrated and calibrated mode
long sum = 0;             //Sum of Sensor Readings over numReadings
int num = 0;              //itaration Variable
long avg = 0;             //Average Sensor Reading over numReadings
long tare = 0;            //tare weight, is set to either 0 or current weight when the according button is pressed
#define numReadings 10    //Number of Sensor Readings that are averaged
int dispType = 1;         //Display Mode for Calibrated mode


void loop() {
  if (scale.is_ready()) {     //Read if new data is available
    sum += scale.read();
    num++;
  }
  if ( num == numReadings ) {  //Display data if readings were taken
    avg = sum / num;
    sum = 0;
    num = 0;
    if (calibrated){                  //Calibrated Mode
      displayWeight(avg, dispType);
      }
    else{                             //Uncalibrated Mode
      displayReading(avg);  
      }
  }

  int buttonPressed = lcd.button();   //Check wich Button is pressed

  if (buttonPressed == KEYPAD_DOWN )  {
    //Debug Mode
    debug_mode();        
  }
  else if ( buttonPressed == KEYPAD_UP ) {
    // Toggle Display Mode
    toggle_display_mode();
  }
  else if ( buttonPressed == KEYPAD_LEFT) {
    // Set Tare = current weight calculated with step wise lineare interpolation
    set_tare();
  }
  else if ( buttonPressed == KEYPAD_RIGHT) {
    // Reset Tare
    reset_tare();
  }
  else if ( buttonPressed == KEYPAD_SELECT) {
    //Calibrate
    calibrate();
    poly_cali();
    calibrated = true;
  }
  delay(10);

}

long extrapolate(long reading) {    //Extrapolate or Interpolate Weight base on Sensor Value and Calibration Data
  if (reading <= vx[0]) {      //if lower than interpolated points
    //Serial.println("low extrapolate");
    long value = myMap(reading, vx[0], vx[1], vy[0], vy[1]);
    return value;
  } else if ( reading >= vx[points - 1]) {
    //Serial.println("high extrapolate");
    long value = myMap(reading, vx[points - 2], vx[points - 1], vy[points - 2], vy[points - 1]);
    return value;
  }
  for ( int i = 0; i < points; i++) {
    if ( reading < vx[i + 1] && reading > vx[i] ) {
      //Serial.print("interpolate: ");
      //Serial.println(i);
      long value =  myMap( reading, vx[i], vx[i + 1], vy[i], vy[i + 1] );
      return value;
    }
  }
  //Serial.println("interpolate fail");
}

long myMap(long reading, float fromLow, float fromHigh, float toLow, float toHigh) {    //Interpolation Formula
  return (reading - fromLow) / (fromHigh - fromLow) * (toHigh - toLow) + toLow;
}

void calibrate() {          //Step Linear Interpolation Calibration Algorithm
  Serial.print("calibration..");
  int k = 0;
  for ( int i = 0; i < M; i++) {
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("place ");
    lcd.print(vy[i] / 1000, 0);
    lcd.print(" g");
    lcd.setCursor(0, 1);
    lcd.print("press select");
    delay(250);
    while (lcd.button() != KEYPAD_SELECT);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("reading...");
    lcd.setCursor(0, 1);
    lcd.print("dont touch");
    delay(100);
    long reading = scale.read_average(20);
    
    long value = extrapolate(reading);
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(value);
    //if ( value > vy[i] - 10000 && value < vy[i] + 10000 ) {
    if ( reading > vx[i] * 0.9 && reading < vx[i] * 1.1 ) {
      vx[i] = reading;
      lcd.setCursor(0, 1);
      lcd.print("valid");
      k++;
    } else {
      lcd.setCursor(0, 1);
      lcd.print("invalid");
      delay(500);
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print(value);
    }
    delay(1000);
    
  }
  if (k == M){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Calibrated");
    delay(2000);
    }
  else {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Not Calibrated");
    delay(2000);
    }
  Serial.println(".done");
  for ( int i = 0; i < points; i++) {
    Serial.print(vx[i]);
    Serial.print("\t");
    Serial.println(vy[i]);
  }
  
   
}

void displayReading(long value){    //Display Current Sensor Value
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sensor Value:");
  lcd.setCursor(0, 1);
  lcd.print( value);
  }

void displayWeight(long value, int type) {    //Display current weight calculated with both algorithms, depending on the display mode
  long micrograms = extrapolate(value) - tare;  //Calculated weight with step wise linear interpolation
  long poly_micrograms = polyreg(value) - tare; //Calculated weight with polynomial regression
  lcd.clear();
  lcd.setCursor(12, 0);
  lcd.print("I");
  lcd.setCursor(14, 0);
  lcd.print(type);
  if ( tare ){
    lcd.setCursor(14, 1);
    lcd.print('T');    
  }
  //LCD Print Weight  Calculated  with step wise linear interpolation
  lcd.setCursor(0, 0);
  char dispString [11];
  switch (type) {
    case 1:
      sprintf(dispString, "%04d", micrograms / 1000);
      lcd.print( dispString );
      lcd.print(" g");
      break;
    case 2:
      dtostrf( micrograms / 1000.0, 6, 1, dispString);
      lcd.print( dispString );
      lcd.print(" g");
      break;
    case 3:
      dtostrf( micrograms / 1000.0, 7, 2, dispString);
      lcd.print( dispString );
      lcd.print(" g");
      break;
    case 4:
      dtostrf( micrograms / 1000000.0, 6, 4, dispString);
      lcd.print( dispString );
      lcd.print(" kg");
      break;
    default:
      break;
  }
  
  //LCD Print Weight  Calculated  with polynomial regression
  lcd.setCursor(12, 1);
  lcd.print("P");
  lcd.setCursor(0, 1);
  char dispString_poly [11];
  switch (type) {
    case 1:
      sprintf(dispString_poly, "%04d", poly_micrograms / 1000);
      lcd.print( dispString_poly );
      lcd.print(" g");
      break;
    case 2:
      dtostrf( poly_micrograms / 1000.0, 6, 1, dispString_poly);
      lcd.print( dispString_poly );
      lcd.print(" g");
      break;
    case 3:
      dtostrf( poly_micrograms / 1000.0, 7, 2, dispString_poly);
      lcd.print( dispString_poly );
      lcd.print(" g");
      break;
    case 4:
      dtostrf( poly_micrograms / 1000000.0, 6, 4, dispString_poly);
      lcd.print( dispString_poly );
      lcd.print(" kg");
      break;
    default:
      break;
  }
}

void poly_cali() {                //Polynomial Regression Calibration Algorithm
  //Serial.print("calibration..");
  matrix_init();    //Matrix Initialization
  
  //Creation of Feature Matrix X0 and Results Matrix Y0
  for ( int i = 0; i < M; i++) {    
    
    Y0[i][0] = vy[i];
    
    long reading = vx[i];
    for (int j = 1; j < N; j++)
    {
      X0[i][j] = pow(reading, j);
    }
    
  }

  //Calculation of Polynom Weights
  Matrix.Transpose((mtx_type*) X0, M, N, (mtx_type*)XT);
  Matrix.Multiply((mtx_type*)XT, (mtx_type*)X0, N, M, N, (mtx_type*)XI);
  Matrix.Invert((mtx_type*)XI, N);
  Matrix.Multiply((mtx_type*)XI, (mtx_type*)XT, N, N, M, (mtx_type*)X0);
  Matrix.Multiply((mtx_type*)X0, (mtx_type*)Y0, N, M, 1, (mtx_type*)T0);
  
  //LCD Print of Polynom Weights
  for ( int i = 0; i < N; i++) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(T0[i][0]);
    delay(1000);
  }  
}

long polyreg(long value){     //Calculating Weight based on a Sensor Value with polynom weights
  X[0][0] = 1;
  for (int j = 1; j < N; j++)
  {
    X[0][j] = pow(value, j);
  }
  
  Matrix.Multiply((mtx_type*)X, (mtx_type*)T0, 1, N, 1, (mtx_type*)Y);
  return Y[0][0];
  
}

void matrix_init(){
  //Initialize Matrix
  for (int i = 0; i < M; i++)
  {
    Y0[i][0] = 0;
    T0[N][0] = 0;
    
    for (int j = 0; j < N; j++)
    {
      XI[i][j] = 0;
      XT[i][j] = 0;
      if( j == 0)
      {
        X0[i][j] = 1;
      }
      else
      {
        X0[i][j] = 0;
      }
    }
  }
}

void debug_mode(){
  //Serial + LCD Print Polynomial Regression Parameters
    for ( int i = 0; i < N; i++) {
      char dispString [16];
      dtostrf( T0[i][0], 6, 4, dispString);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(dispString);
      delay(1000);
      Serial.print("Theta");
      Serial.print(i);
      Serial.print(";\t");
      Serial.print(dispString);
      Serial.print("\n");
    }

    //Serial Print 30 x Sensorvalue, Linear_Weight, Poly_Weight and True Weight for the Test weights
    Serial.print("Sensorwert");
    Serial.print(",\t");
    Serial.print("Stepwise Linear Interpolation");
    Serial.print(",\t");
    Serial.print("Polynomial Regression");
    Serial.print(",\t");
    Serial.println("True Weight");
    for ( int i = 0; i < points_test; i++) {

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("place ");
      lcd.print(vy_test[i] / 1000, 0);
      lcd.print(" g");
      lcd.setCursor(0, 1);
      lcd.print("press select");
      delay(250);
      while (lcd.button() != KEYPAD_SELECT);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("reading...");
      lcd.setCursor(0, 1);
      lcd.print("dont touch");

      for ( int j = 0; j < 30; j++){
        long wert = scale.read();
        long lin_weight = extrapolate(wert);
        long poly_weight = polyreg(wert);
        delay(100);
        Serial.print(wert);
        Serial.print(",\t");
        Serial.print(lin_weight);
        Serial.print(",\t");
        Serial.print(poly_weight);
        Serial.print(",\t");
        Serial.println(vy_test[i]);
        }  
    }
    Serial.println(".done");
  }

void toggle_display_mode()
{
  dispType = ++dispType == 5 ? 1 : dispType;
  displayWeight(avg, dispType);
  delay(500);
}

void set_tare()
{
  tare = extrapolate(avg);
  //tare = polyreg(avg);
}

void reset_tare()
{
  tare = 0;
}
