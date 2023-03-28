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

}

void loop() {
  ///Update all the values
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  currentAttitude = getGyroData();
  kalmanUpdate(currentAttitude, dt);
  getCoords(coords);
  if(timer/1000 - rdt >= 500){ //send data every 0.5 seconds
    //it has been 1s since last transmission
    rdt = timer/1000;
    //data format: lat,long,pitch,roll,heading,alt
    String data = "";
    //data += String(coords[2],7);
    data += "51.3443708";
    data += ",";
    data += "-0.6433905";
    //data += String(coords[3],7);
    data += ",";
    data += kalAngleX * 180 / PI;
    data += ",";
    data += kalAngleY * 180 / PI;
    data += ",";
    data += getHeading(currentAttitude);
    data += ",";
    data += getAltitude();
    sendRadio(data);
  }
}
// 51.3443708,-0.6433905,134.11,-129.55,224.68,0.00