#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TRSensors.h"
#include <Wire.h>


#define PWMA   6           // Left Motor Speed pin (ENA)
#define AIN2   A0          // Motor-L forward (IN2)
#define AIN1   A1          // Motor-L backward (IN1)
#define PWMB   5           // Right Motor Speed pin (ENB)
#define BIN1   A2          // Motor-R forward (IN3)
#define BIN2   A3          // Motor-R backward (IN4)
#define PIN 7
#define NUM_SENSORS 5
#define OLED_RESET 9
#define OLED_SA0   8
#define Addr 0x20


// ------------------------------------------------
// Adjustable parameters - tune these!
// ------------------------------------------------


#define BASE_SPEED  60     // Base forward speed (0-255)


#define PID_KP  0.18f       // Proportional gain
#define PID_KI  0.00012f    // Integral gain
#define PID_KD  18.0f       // Derivative gain


#define INTEGRAL_MAX   5000L
#define SPEED_SCALING   true


// ------------------------------------------------


#define beep_on  PCF8574Write(0xDF & PCF8574Read())
#define beep_off PCF8574Write(0x20 | PCF8574Read())


Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);


TRSensors trs = TRSensors();
unsigned int sensorValues[NUM_SENSORS];
unsigned int position;          // <- FIXED: declare globally so setup() can use it
float last_error = 0.0f;
long integral = 0;
uint16_t i, j;
byte value;
unsigned long lasttime = 0;
Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, PIN, NEO_GRB + NEO_KHZ800);


void PCF8574Write(byte data);
byte PCF8574Read();
uint32_t Wheel(byte WheelPos);


void setup() {
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,0);
  display.println("WaveShare");
  display.setCursor(10,25);
  display.println("AlphaBot2");
  display.setTextSize(1);
  display.setCursor(10,55);
  display.println("Press to calibrate");
  display.display();
 
  while(value != 0xEF)  // wait button pressed
  {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;
  }
 
  pinMode(PWMA,OUTPUT);                    
  pinMode(AIN2,OUTPUT);      
  pinMode(AIN1,OUTPUT);
  pinMode(PWMB,OUTPUT);      
  pinMode(BIN1,OUTPUT);    
  pinMode(BIN2,OUTPUT);  
 
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(AIN2,HIGH);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);  
 
  RGB.begin();
  RGB.setPixelColor(0,0x00FF00 );
  RGB.setPixelColor(1,0x00FF00 );
  RGB.setPixelColor(2,0x00FF00 );
  RGB.setPixelColor(3,0x00FF00);
  RGB.show();
  delay(500);
 
  analogWrite(PWMA,80);
  analogWrite(PWMB,80);
 
  for (int i = 0; i < 100; i++)  // calibration spin ~10 seconds
  {
    if(i<25 || i >= 75)
    {
      digitalWrite(AIN1,HIGH);
      digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,LOW);
      digitalWrite(BIN2,HIGH);  
    }
    else
    {
      digitalWrite(AIN1,LOW);
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,HIGH);
      digitalWrite(BIN2,LOW);  
    }
    trs.calibrate();
  }
 
  analogWrite(PWMA,0);
  analogWrite(PWMB,0);
  digitalWrite(AIN2,LOW);
  digitalWrite(AIN1,LOW);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,LOW);  
 
  RGB.setPixelColor(0,0x0000FF );
  RGB.setPixelColor(1,0x0000FF );
  RGB.setPixelColor(2,0x0000FF );
  RGB.setPixelColor(3,0x0000FF);
  RGB.show();
 
  value = 0;
  while(value != 0xEF)  // wait second press
  {
    PCF8574Write(0x1F | PCF8574Read());
    value = PCF8574Read() | 0xE0;


    // FIXED: now position is global -> this line compiles
    position = trs.readLine(sensorValues)/200;


    display.clearDisplay();
    display.setCursor(0,25);
    display.println("Calibration Done !!!");
    display.setCursor(0,55);
    for (int i = 0; i < 21; i++)
    {
      display.print('_');
    }
    display.setCursor(position*6,55);
    display.print("**");
    display.display();
  }


  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(10,0);
  display.println("AlphaBot2");
  display.setTextSize(3);
  display.setCursor(40,30);
  display.println("Go!");
  display.display();
 
  delay(500);
 
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);  
}


void loop() {
  position = trs.readLine(sensorValues);  // <- still assign here (overwrites global)


  // --- Debug: raw sensor values -------------------------------
  for (unsigned char i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print(" | Pos: "); Serial.print(position);


  // --- PID calculation ----------------------------------------
  float error = (float)position - 2000.0f;


  float derivative = error - last_error;
  integral += (long)error;


  if (integral > INTEGRAL_MAX)   integral = INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX)  integral = -INTEGRAL_MAX;


  last_error = error;


  float correction =
    PID_KP * error +
    PID_KI * (float)integral +
    PID_KD * derivative;


  #if SPEED_SCALING
    float speed_factor = (float)BASE_SPEED / 160.0f;
    correction *= speed_factor;
  #endif


  int max_correction = BASE_SPEED;
  if (correction >  max_correction) correction =  max_correction;
  if (correction < -max_correction) correction = -max_correction;


  // --- Debug output -------------------------------------------
  Serial.print(" | Err: ");  Serial.print(error, 1);
  Serial.print(" | P: ");    Serial.print(PID_KP * error, 2);
  Serial.print(" | I: ");    Serial.print(PID_KI * (float)integral, 4);
  Serial.print(" | D: ");    Serial.print(PID_KD * derivative, 2);
  Serial.print(" | Corr: "); Serial.println((int)correction);


  // --- Apply to motors ----------------------------------------
  if (correction < 0) {
    analogWrite(PWMA, BASE_SPEED + (int)correction);
    analogWrite(PWMB, BASE_SPEED);
  } else {
    analogWrite(PWMA, BASE_SPEED);
    analogWrite(PWMB, BASE_SPEED - (int)correction);
  }


  // Emergency stop
  if (sensorValues[1] > 900 && sensorValues[2] > 900 && sensorValues[3] > 900) {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
  }


  // Rainbow LEDs
  if (millis() - lasttime > 200) {  
    lasttime = millis();  
    for (i = 0; i < RGB.numPixels(); i++) {
      RGB.setPixelColor(i, Wheel(((i * 256 / RGB.numPixels()) + j) & 255));
    }
    RGB.show();
    if (j++ > 256*4) j = 0;
  }
}


uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return RGB.Color(WheelPos * 50, 255 - WheelPos * 50, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return RGB.Color(255 - WheelPos * 50, 0, WheelPos * 50);
  } else {
   WheelPos -= 170;
   return RGB.Color(0, WheelPos * 50, 255 - WheelPos * 50);
  }
}


void PCF8574Write(byte data)
{
  Wire.beginTransmission(Addr);
  Wire.write(data);
  Wire.endTransmission();
}


byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(Addr, 1);
  if(Wire.available()) {
    data = Wire.read();
  }
  return data;
}