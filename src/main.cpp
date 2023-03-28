#include "Arduino.h"

/// file includes
#include "LoRa_Radio.h"
#include "gps.h"
#include "gyro.h"
#include "kalmanFilters.h"
#include "compass.h"
#include "barometer.h"

Serial_ Radio;
uint32_t timer;
int rdt = 0;
void setup() {
  Serial.begin(115200);
  Serial.println("Starting up");

  startRadio();
  startBMP();
  /*
  delay(1000);// Wait for sensor to stabilize
  startBMP();
  startCompass();
  startGyro();

  //startGPS();
  getGyroData();
  setupFilters(currentAttitude);
  timer = micros();

  Serial.println("SYSTEM STARTED");
  sendRadio("System startup complete");
   */
}

void loop() {
  ///Update all the values
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  //currentAttitude = getGyroData();
  //kalmanUpdate(currentAttitude, dt);
  /*
  Serial.print("RawPitch:");
  Serial.print(currentAttitude.pitch);
  Serial.print(",");
  Serial.print("KPitch:");
  Serial.print(kalAngleX);
  Serial.print(",");
  Serial.print("RawRoll:");
  Serial.print(currentAttitude.roll);
  Serial.print(",");
  Serial.print("KRoll:");
  Serial.print(kalAngleY);
  Serial.println("");
  */
  if(timer/1000 - rdt >= 500){ //send data every 0.5 seconds
    //it has been 1s since last transmission
    rdt = timer/1000;
    Serial.println(getAltitude());
    sendRadio("Ping");
  }
}