#include <string>
#include <math.h>
#include <Arduino.h>
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

void printEncoder() {
    display.setCursor(0,0);
    display.print("LR: ");
    display.print((float)counterL.rawCount, 2);
    display.setCursor(0,20);
    display.print("RR: ");
    display.print((float)counterR.rawCount, 2);
    display.display();
}

void Turn(float tarL, float tarR, std::string d, int Speed) {
  if (d == "R") {
    motors.changeStatus(MotorL, MOTOR_STATUS_CCW);
    motors.changeStatus(MotorR, MOTOR_STATUS_CCW);
    motors.changeDuty(MotorL, Speed);  // Speed zero is stopped.
    motors.changeDuty(MotorR, Speed);
    while ((float)counterL.rawCount < tarL || (float)counterR.rawCount < tarR) {
      if ((float)counterL.rawCount >= tarL) {
        motors.changeDuty(MotorL, 0);  // Speed zero is stopped.
    }
      if ((float)counterR.rawCount >= tarR) {
        motors.changeDuty(MotorR, 0);  // Speed zero is stopped.
    }
    }
    motors.changeDuty(MotorL, 0);  // Speed zero is stopped.
    motors.changeDuty(MotorR, 0);
    display.clearDisplay();

    printEncoder();
   
    int overTurnL = (int)counterL.rawCount - (int)tarL;
    int overTurnR = (int)counterR.rawCount - (int)tarR;
    Serial.println(overTurnL);
    Serial.println(overTurnR);

    if (abs(overTurnL) + abs(overTurnR) > 0) {
      if (overTurnL > 0 && overTurnR > 0) {
        Serial.println("recurse PP");
        Serial.println(overTurnL);
        Serial.println(overTurnR);
        Turn((float)counterL.rawCount + overTurnL, (float)counterR.rawCount + overTurnR, "L", max(35, Speed - 5)); 
        
      } else if (overTurnL < 0 && overTurnR < 0) { 
        Serial.println("recurse NN");
        Serial.println(overTurnL);
        Serial.println(overTurnR);
        Turn((float)counterL.rawCount - overTurnL, (float)counterR.rawCount - overTurnR, "R", Speed); 
   
      } else {
          if (overTurnL + overTurnR > 0) {
            Serial.println("recurse L > R R");
            Serial.println(overTurnL);
            Serial.println(overTurnR);
            Turn((float)counterL.rawCount + overTurnL, (float)counterR.rawCount +overTurnR, "L", max(35, Speed - 5));  //L > R
            
          } else {
            Serial.println("recurse R > L R");
            Serial.println(overTurnL);
            Serial.println(overTurnR);
            Turn((float)counterL.rawCount + abs(overTurnL), (float)counterL.rawCount + abs(overTurnR), "R", Speed); // R > L
            
          }
      }

    }
    display.clearDisplay();

    printEncoder();
  } else {
    motors.changeStatus(MotorL, MOTOR_STATUS_CW);
    motors.changeStatus(MotorR, MOTOR_STATUS_CW);
    motors.changeDuty(MotorL, Speed);  // Speed zero is stopped.
    motors.changeDuty(MotorR, Speed);
    while ((float)counterL.rawCount < tarL || (float)counterR.rawCount < tarR) {
      if ((float)counterL.rawCount >= tarL) {
        motors.changeDuty(MotorL, 0);  // Speed zero is stopped.
    }
      if ((float)counterR.rawCount >= tarR) {
        motors.changeDuty(MotorR, 0);  // Speed zero is stopped.
    }
    }
    display.clearDisplay();

    printEncoder();
    motors.changeDuty(MotorL, 0);  // Speed zero is stopped.
    motors.changeDuty(MotorR, 0);
    int overTurnL = (int)counterL.rawCount - (int)tarL;
    int overTurnR = (int)counterR.rawCount - (int)tarR;
    Serial.println(overTurnL);
    Serial.println((float)counterL.rawCount);
    Serial.println((float)tarL);

    Serial.println(overTurnR);
    Serial.println((float)counterR.rawCount);
    Serial.println((float)tarR);


    if (abs(overTurnL) + abs(overTurnR) > 0) {
      if (overTurnL > 0 && overTurnR > 0) {
        Serial.println("recurse PPL");
        Serial.println(overTurnL);
        Serial.println(overTurnR);
        Turn((float)counterL.rawCount + overTurnL, (float)counterR.rawCount + overTurnR, "R", max(35, Speed - 5)); 
      } else if (overTurnL < 0 && overTurnR < 0) { 
        Serial.println("recurse NN");
        Serial.println(overTurnL);
        Serial.println(overTurnR);
        Turn((float)counterL.rawCount - overTurnL, (float)counterR.rawCount - overTurnR, "L", Speed); 
        
      } else {
          if (overTurnL + overTurnR > 0) {
            Serial.println("recurse asd");
            Serial.println(overTurnL);
            Serial.println(overTurnR);
            Serial.println(floor((overTurnL + overTurnR) / 2));
            Turn((float)counterL.rawCount + overTurnL, (float)counterR.rawCount + overTurnR, "R", max(35, Speed - 5));  //L > R
            
          } else {
            Serial.println("recurse asdfgaghtd");
            Serial.println(overTurnL);
            Serial.println(overTurnR);
            Turn((float)counterL.rawCount + abs(overTurnL), (float)counterL.rawCount + abs(overTurnR), "L", Speed); // R > L        
          }
      }

    }
    display.clearDisplay();

    printEncoder();
    Serial.println("Recurse Done");
    Serial.println(overTurnL);
    Serial.println(overTurnR);
  }
}

void loop() {
  
  float runTime = 0.001 * millis();
  
  int motorSpeedL = 40;
  int motorSpeedR = 40;
  revoL = getRotations(counterL);
  revoR = getRotations(counterR);
  float tarL = (float)counterL.rawCount + 8;
  float tarR = (float)counterR.rawCount + 8;
  display.clearDisplay();
  printEncoder();

  delay(5000);
  Turn(tarL, tarR, "L", 40);
}