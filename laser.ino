#include <Servo.h>

Servo panServo;
Servo tiltServo;

const int panPin = 9;
const int tiltPin = 10;
const int laserPin = 8;

const int panMin = 0;
const int panMax = 180;
const int tiltMin = 30;
const int tiltMax = 150;

unsigned long laserOffTime = 0;
const unsigned long laserDuration = 500; 

void setup() {
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);
  Serial.begin(9600);
}

void loop() {
 
  if (millis() >= laserOffTime) {
    digitalWrite(laserPin, LOW);
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int commaIndex = input.indexOf(',');
    if (commaIndex != -1) {
      int panAngle = input.substring(0, commaIndex).toInt();
      int tiltAngle = input.substring(commaIndex + 1).toInt();

      panAngle = constrain(panAngle, panMin, panMax);
      tiltAngle = constrain(tiltAngle, tiltMin, tiltMax);

      panServo.write(panAngle);
      tiltServo.write(tiltAngle);

      digitalWrite(laserPin, HIGH);
      laserOffTime = millis() + laserDuration;
    }
  }
}

