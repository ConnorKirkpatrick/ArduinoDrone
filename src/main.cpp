#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>


Adafruit_BMP280 bmp;

void altimeter();

void setup() {
    altimeter();
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
}

void altimeter(){
    Serial.begin(9600);
    while(!bmp.begin(0x76)){
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!\n\n"));
        delay(3000);
    }
    Serial.println("Good Connection");

    while(true){
        Serial.print(F("Temperature = "));
        Serial.print(bmp.readTemperature());
        Serial.println(" *C");

        Serial.print(F("Pressure = "));
        Serial.print(bmp.readPressure());
        Serial.println(" Pa");

        Serial.print(F("Approx altitude = "));
        Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
        Serial.println(" m");

        Serial.println();
        delay(2000);
    }
}