// RocksAndRobots See rocksandrobots.com
// These files are for a robot based on the ESP32-wroom
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
/*
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
*/
/*
 * Test all of the sensors and the motor drivers.
 */
//------------------------------------------------------------

counter_t counterL;
counter_t counterR;
int counts = 0;

// The magic of interrupts. Here there be dragons.
// My first version of this tried to use floating
// point math. Apparently that does not work well
// on this chip with this version of the Arduino
// libraries.
// This is the "inner" part of handling the pin
// change interrupt, the part that is the same for both
// rotation sensors.
void IRAM_ATTR countPinIntInner(counter_t &unit) {
  unsigned long now = micros();  // Read the clock
  // Ignore signals that look like noise. The motors
  // I tested didn't go faster than 110 RPM.
  // Here we assume the wheel won't go faster than 150 RPM.
  // Any signals faster than that are considered noise, and
  // ignored.
  if (now < (unit.prevTime + 5000)) {  //
    return;
  }
  unit.rawCount += 1;                   // not noise, so add one to the count.
  unit.interval = now - unit.prevTime;  // interval in microseconds since the previous pulse.
  unit.prevTime = now;                  // This pulse will be previous pulse for the next count.
}
// Make separate interrupt handler functions for the
// left and write encoder. Each encoder has its one struct for storing
// data. The functions just call the inner routine with the appropriate
// data structure.
void IRAM_ATTR counterPinInterruptL() {
  countPinIntInner(counterL);
}
void IRAM_ATTR counterPinInterruptR() {
  countPinIntInner(counterR);
}
// We are almost done with the dragons and back to normal functions. No more IRAM_ATTR weirdness, and we can use
// floating point and print statements. We just have to tell the system which interrupt functions to call and setup
// the data for the interrupt routines.
void initCounter(counter_t &unit) {
  unit.rawCount = 0;
  unit.interval = 0;
  pinMode(unit.pinNum, INPUT);
}
void initCounterL() {
  counterL.pinNum = countPinL;
  initCounter(counterL);
  attachInterrupt(counterL.pinNum, &counterPinInterruptL, RISING);
}
void initCounterR() {
  counterR.pinNum = countPinR;
  initCounter(counterR);
  attachInterrupt(counterR.pinNum, &counterPinInterruptR, RISING);
}
// Entirely done with the arcane interrupt stuff.

// Return the current rotation rate.
float getRPM(counter_t &unit) {
  // Using the difference between two encoder slots, calculate the rotation rate in RPM.
  unsigned long now = micros();
  const float toRPM = 1000000.0 * 60.0 / 20.0;  // microseconds per second, seconds per minute, counts per rev
  if (now - unit.prevTime > 250000 || unit.interval == 0) {
    return 0.0;
  }
  return toRPM / unit.interval;
}
// Return the count of wheel rotations.
float getRotations(counter_t &unit) {
  return (float)unit.rawCount / 20.0;
}

float sonarDist(int trigPin, int echoPin) {
  float distance;  // distance in centimeters
  int duration;    // round trip tim, e for the sound in microseconds.
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // the program doesn't do anything else while waiting for an echo.
  // For this application, anything farther away than a meter might as
  // well not exist. The next line times how long it takes for the echo
  // to return or until it reaches the timeout value. 5810 microseconds
  // is just under a meter.
  duration = pulseIn(echoPin, HIGH, 5810L);  //
  if (duration == 0) {
    return 100.0;  // For anything greater than 99.9 cm, return exactly a meter.
  }
  distance = (duration * 0.5) * 0.0344;
  return distance;
}
void drawWheelStatus() {
  float rpm, revo;
  for (int i = 0; i < 2; i++) {
    float radius = 7;
    int centerX = 24 + i * 16;
    int rpmCenterY = 7;
    int revoCenterY = 23;
    display.drawCircle(centerX, rpmCenterY, radius, WHITE);   // RPM display
    display.drawCircle(centerX, revoCenterY, radius, WHITE);  // angle display
    if(i == 0) {
      rpm = rpmL;
      revo = revoL;
    } else {
      rpm = rpmR;
      revo = revoR;
    }
    float revoAngle = revo * 2 * 3.1415926;
    float rpmAngle = rpm * 0.01 * 3.1415926;
    display.drawLine(centerX,rpmCenterY,centerX+(radius*sin(rpmAngle)),rpmCenterY+(radius*cos(rpmAngle)),WHITE);
    display.drawLine(centerX, revoCenterY, centerX + (radius*sin(revoAngle)), revoCenterY+(radius*cos(revoAngle)),WHITE);
  }
}
void drawSonarStatus() {
  int scaledDistL = distanceL * 0.43;
  int scaledDistR = distanceR * 0.43;  
  display.drawRect(0,0,8,47,WHITE);
  display.fillRect(2,45-scaledDistL,4,scaledDistL,WHITE);
  display.drawRect(56,0,8,47,WHITE);
  display.fillRect(58,45-scaledDistR,4,scaledDistR,WHITE);
}
void drawLightStatus() {
  int scaledLightL = constrain( lightL * 3.0, 0.0,16.0);
  int scaledLightR = constrain( lightR * 3.0, 0.0, 16.0);
  display.fillRect(16,32,32,16,WHITE);
  display.fillRect(32-scaledLightL,32,scaledLightL+scaledLightR,16,BLACK);
}
void drawStatus() {
  display.clearDisplay();
  drawWheelStatus();
  drawSonarStatus();
  drawLightStatus();
  display.display();
}
void setupDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  display.display();
}

/*
void notsetup() {
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
  initCounterL();
  initCounterR();
}
*/
void setupCounters() {
  initCounterL();
  initCounterR();
}
  
/*
void notloop() {
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
}
*/

void printStatus() {
  Serial.print(", Light(");
  Serial.print(lightL,2);
  Serial.print(", ");
  Serial.print(lightR,2);
  Serial.print(")");

  Serial.print(", cm(");
  Serial.print(distanceL, 1);
  Serial.print(", ");
  Serial.print(distanceR, 1);
  Serial.print(")");
  Serial.print(", revolutions(");
  Serial.print(revoL, 2);
  Serial.print(", ");
  Serial.print(revoR, 2);
  Serial.print("), ");
  Serial.print("RPM (");
  Serial.print(rpmL, 2);
  Serial.print(", ");
  Serial.print(rpmR, 2);
  Serial.print(") ");
  Serial.println();
}
