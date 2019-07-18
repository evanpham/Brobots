#include <Arduino.h>
#include "Sonar.h"

Sonar::Sonar(int echoPin, int trigPin) {
    echo = echoPin;
    trig = trigPin;
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
}

// Reads distance with sonar sensor
// Returns distance in cm
int Sonar::readSonar(void) {
  // Clear the trigPin by setting it LOW:
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Read the echoPin, pulseIn() returns the duration (length of the pulse) in microseconds:
  long duration = pulseIn(echo, HIGH);
  
  // Calculate the distance:
  int distance = duration*0.034/2;
  
  // Serial.print("Distance = ");
  // Serial.print(distance);

  return distance;
}