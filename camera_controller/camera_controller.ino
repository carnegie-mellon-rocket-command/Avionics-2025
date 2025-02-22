// THISTLE

#include <Adafruit_TinyUSB.h>

#define CAM_PIN_L 0
#define RC_PIN 2

const int RF_THRESHOLD = 1740;
int RC_INPUT = 0;
int CAM_ACTIVATED = 0;

void setup() {
  pinMode(RC_PIN, INPUT);
  pinMode(CAM_PIN_L, OUTPUT);

  // turn camera off until signal received
  digitalWrite(CAM_PIN_L, HIGH);

  Serial.begin(9600);
}

void loop() {
  if (CAM_ACTIVATED)
    return;

  RC_INPUT = pulseIn(RC_PIN, HIGH);
  if (RC_INPUT > RF_THRESHOLD)
    activateCamera();

  Serial.print("RC_INPUT ="); Serial.println(RC_INPUT, DEC);

  delay(100);
}

void activateCamera() {
  // turn camera on and keep on
  digitalWrite(CAM_PIN_L, LOW);
  CAM_ACTIVATED = true;
}