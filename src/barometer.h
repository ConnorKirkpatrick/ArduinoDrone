#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SPI.h>

Adafruit_BMP280 bmp;

void startBMP(){
  if(!bmp.begin(0x76)){
    Serial.println("No altimeter found, please check address or wiring");
  }
  sendRadio("Altimeter Ready");
}

void restartBMP(){
  if(!bmp.begin(0x76)){
    Serial.println("No altimeter found, please check address or wiring");
  }
}

float getAltitude(){
  if(bmp.getStatus() == 243 || bmp.getStatus() == 0){
    restartBMP(); //restart BMP sensor if an error code is detected
  }
  //calculate alt based on baro pressure and temp
  double h = ((pow(1013.25/bmp.readPressure(), 1/5257) -1) * (bmp.readTemperature())) / 0.0065; //using hysometric formula
  return h;
}