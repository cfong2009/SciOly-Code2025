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




#include "LOLIN_I2C_MOTOR.h"

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

/*
 * Test all of the sensors and the motor drivers.
 */
//------------------------------------------------------------

typedef struct {
  int pinNum;
  int rawCount;
  int interval;
  unsigned long prevTime;
} counter_t;

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
  if (now < (unit.prevTime + 10000)) {  //
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
  initCounter(counterL);
  attachInterrupt(counterR.pinNum, &counterPinInterruptR, RISING);
}
// Entirely done with the arcane interrupt stuff.

// Return the current rotation rate.
float getRPM(counter_t &unit) {
  // Using the difference between two encoder slots, calculate the rotation rate in RPM.
  unsigned long now = micros();
  const float toRPM = 1000000.0 * 60.0 / 40.0;  // microseconds per second, seconds per minute, counts per rev
  if (now - unit.prevTime > 250000 || unit.interval == 0) {
    return 0.0;
  }
  return toRPM / unit.interval;
}
// Return the count of wheel rotations.
float getRotations(counter_t &unit) {
  return (float)unit.rawCount / 40.0;
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

void setup() {
  Serial.begin(115200);
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

void loop() {
  int lightL, lightM, lightR;
  float distanceL, distanceR;

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
    motors.changeDuty(MotorL, 0 );
    motors.changeDuty(MotorR, (cyclePhase-1000) / 5);
  }else if (cyclePhase < 2000) {
    motors.changeDuty(MotorL, 0 );
    motors.changeDuty(MotorR, (2000-cyclePhase) / 5);
  } else {
    motors.changeDuty(MotorL, 0);  // Speed zero is stopped.
    motors.changeDuty(MotorR, 0);
  }

  float revoL = getRotations(counterL);
  float revoR = getRotations(counterR);

  delay(100);

  Serial.print(runTime, 3);

  lightL = analogRead(lightPinL);
  lightR = analogRead(lightPinR);
  Serial.print(", left Light ");
  Serial.print(lightL);
  Serial.print(", right Light ");
  Serial.print(lightR);

  Serial.print(", cm(");
  Serial.print(distanceL, 1);
  Serial.print(", ");
  Serial.print(distanceR, 1);
  Serial.print(")");
  Serial.print(", revolutions(");
  Serial.print(getRotations(counterL), 2);
  Serial.print(", ");
  Serial.print(getRotations(counterR), 2);
  Serial.print("), ");
  Serial.print("RPM (");
  Serial.print(getRPM(counterL), 2);
  Serial.print(", ");
  Serial.print(getRPM(counterR), 2);
  Serial.print(") ");
  Serial.println();
}
