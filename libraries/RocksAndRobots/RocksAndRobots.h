typedef struct {
  int pinNum;
  int rawCount;
  int interval;
  unsigned long prevTime;
} counter_t;

extern int countPinL;
extern int countPinR;
extern counter_t counterL;
extern counter_t counterR;

extern float getRPM(counter_t &unit);
extern float getRotations(counter_t &unit);
extern float sonarDist(int trigPin, int echoPin);
extern void drawWheelStatus();
extern void drawSonarStatus();
extern void drawLightStatus();
extern void drawStatus();
extern void printStatus();
extern void setupDisplay();
extern void setupCounters();
extern float lightL, lightR, distanceL, distanceR, revoL, revoR, rpmL, rpmR;
extern Adafruit_SSD1306 display;

