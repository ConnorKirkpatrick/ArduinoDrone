//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600
#include "TinyGPS++.h"
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
float coords[6];
float lastFix[2];
double distanceFromLastFix;


void getCoords(float array[]){
  if(Serial1.available()){
    while(Serial1.available())//While there are characters to come from the GPS
    {
      gps.encode(Serial1.read());//This feeds the serial NMEA data into the library one char at a time
    }
  }
  if(gps.location.isUpdated()){
    array[0] = gps.satellites.value();
    array[1] = gps.time.value();
    array[2] = gps.location.lat();
    array[3] = gps.location.lng();
    array[4] = gps.speed.mps();
    array[5] = gps.altitude.meters();
  }
}

void startGPS(){
  Serial.println("GPS Starting....");
  Serial1.begin(9600);
  getCoords(coords);
  //if there is no satellites, no valid fix can be made thus retry
  while(coords[0] < 1) {
    getCoords(coords);
  }
  sendRadio("GPS Ready");
  lastFix[0] = coords[2];
  lastFix[1] = coords[3];
}