
#include <math.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "LOLIN_I2C_MOTOR.h"
#include <RocksAndRobots.h>
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

LOLIN_I2C_MOTOR motors;
int MotorL = MOTOR_CH_A;
int MotorR = MOTOR_CH_B;


int LEDpin = 2;
int trigPinL = 19;
int echoPinL = 23;
int countPinL = 5;
int lightPinL = 13;

int lightPinM = 36;

int trigPinR = 17;
int echoPinR = 16;
int countPinR = 18;
int lightPinR = 15;

float lightL, lightR, distanceL, distanceR, revoL, revoR, rpmL, rpmR;

/*
 * Drive in a straight line
 */
//------------------------------------------------------------



void setup() {
  Serial.begin(115200);
  setupDisplay();
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(countPinL, INPUT);
  pinMode(countPinR, INPUT);
  pinMode(lightPinL, INPUT);
  pinMode(lightPinM, INPUT);
  pinMode(lightPinR, INPUT);
  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, HIGH);
  display.setTextColor(WHITE); //this is often missed!!
  display.setTextSize(1);
  display.clearDisplay();

  motors.getInfo();  // Make sure the motor shield is plugged in and healthy.
  while (motors.PRODUCT_ID != PRODUCT_ID_I2C_MOTOR) {
    Serial.print(motors.PRODUCT_ID);
    Serial.println("Motor shield disconnected!");
    delay(100);
    digitalWrite(LEDpin, HIGH);
    delay(100);
    digitalWrite(LEDpin, LOW);
    motors.getInfo();
  }
  motors.changeFreq(MOTOR_CH_BOTH, 1000);
  motors.changeStatus(MotorL, MOTOR_STATUS_CCW);
  motors.changeStatus(MotorR, MOTOR_STATUS_CW);
  motors.changeDuty(MotorL, 0);  // Speed zero is stopped.
  motors.changeDuty(MotorR, 0);
  setupCounters();
}

void loop() {
  
  float runTime = 0.001 * millis();
  distanceL = sonarDist(trigPinL, echoPinL);
  distanceR = sonarDist(trigPinR, echoPinR);
  
  int motorSpeedL = 40;
  int motorSpeedR = 10;
  revoL = getRotations(counterL);
  revoR = getRotations(counterR);
  float ratio = 1.0;
  
  motors.changeDuty(MotorL, motorSpeedL); 
  motors.changeDuty(MotorR, motorSpeedR);

  Serial.print(runTime, 3);
 
  display.setCursor(0,0);
  display.print("LR: ");
  display.print((float)counterL.rawCount, 2);
  display.setCursor(0,20);
  display.print("RR: ");
  display.print((float)counterR.rawCount, 2);
  display.display();
  delay(100);
  display.clearDisplay();
}