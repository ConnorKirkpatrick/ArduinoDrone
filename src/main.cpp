#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

//BME/P-280
//connect via I2C ports
//VCC 5v


Adafruit_BMP280 bmp;

void altimeter();

void setup() {
    altimeter();
}

void loop() {
    Serial.println(bmp.getStatus());
    //243 is a bad status->no gnd
    //0 is also bad, need to re-init
    //0Ã is bad, no power, can return when power is restored
    //12,4 is good
    if(bmp.getStatus() == 243 || bmp.getStatus() == 0){
        altimeter();
    }
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Approx altitude = ");
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(5000);
}

void altimeter(){
    Serial.begin(9600);
    while(!bmp.begin(0x76)){
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!\n\n"));
        delay(3000);
    }
    Serial.println("BMP-280 started");
}
