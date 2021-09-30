#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

SoftwareSerial gpsSerial(2,3);
TinyGPSPlus GPS;

float lat = 11.111, lon = 22.222;
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    while(gpsSerial.available()){ // check for gps data
        if(GPS.encode(gpsSerial.read()))// encode gps data
        {
            Serial.print("LAT: ");
            Serial.println(GPS.location.lat(),6);
            Serial.print("LON: ");
            Serial.println(GPS.location.lng(),6);
            Serial.print("ALT(m): ");
            Serial.println(GPS.altitude.meters(),6);
        }
    }
    String latitude = String(lat,6);
    String longitude = String(lon,6);
    Serial.println(latitude+";"+longitude);
    delay(1000);
}

void GPS_INIT(){
    Serial.begin(9600);
    gpsSerial.begin(9600);

}