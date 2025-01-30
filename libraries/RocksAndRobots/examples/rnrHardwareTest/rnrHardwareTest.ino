// Hardware test. Drives forward for a second.
// Then turns sharply left for a second.
// Then pauses for 2 seconds.
// Then the cycle repeats.
// Meanwhile it is displaying the sensor readings.
//
// The sonar sensor range is limited to 100.0 cm.
// Broken or missing sonars will also show 100.0 cm
// The left and right light sensors show values between 0 and 4095.
// Dark surfaces are higher numbers.
// The rotation sensors report the speed and distance traveled for
// each wheel, but not the direction.
//


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

float lightL, lightM, lightR;
float distanceL, distanceR, revoL, revoR, rpmL, rpmR;
/*
 * Test all of the sensors and the motor drivers.
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
  distanceL = sonarDist(trigPinL, echoPinL);
  distanceR = sonarDist(trigPinR, echoPinR);
  int nowInMillis = millis();
  float runTime = 0.001 * nowInMillis;
  // Break time into a repeating motor test cycle
  int cycleLength = 4000;                      // 4 seconds is 4000 milliseconds
  int cycleCount = nowInMillis / cycleLength;  // Since we are using integer math division truncates to a whole number.
  int cyclePhase = nowInMillis - (cycleCount * cycleLength);
  if (cyclePhase < 500) {
    motors.changeDuty(MotorL, cyclePhase / 5);
    motors.changeDuty(MotorR, cyclePhase / 5);
  } else if (cyclePhase < 1000) {
    motors.changeDuty(MotorL, (1000 - cyclePhase) / 5);
    motors.changeDuty(MotorR, (1000 - cyclePhase) / 5);
  } else if (cyclePhase < 1500) {
    motors.changeDuty(MotorL, 0);
    motors.changeDuty(MotorR, (cyclePhase - 1000) / 5);
  } else if (cyclePhase < 2000) {
    motors.changeDuty(MotorL, 0);
    motors.changeDuty(MotorR, (2000 - cyclePhase) / 5);
  } else {
    motors.changeDuty(MotorL, 0);  // Speed zero is stopped.
    motors.changeDuty(MotorR, 0);
  }

  revoL = getRotations(counterL);
  revoR = getRotations(counterR);
  rpmL = getRPM(counterL);
  rpmR = getRPM(counterR);

  delay(100);

  Serial.print(runTime, 3);

  lightL = log(analogRead(lightPinL) + 1.0);
  lightR = log(analogRead(lightPinR)+1.0);
  drawStatus();
  printStatus();
}