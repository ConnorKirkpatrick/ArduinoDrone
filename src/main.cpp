#include <Arduino.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_I2CDevice.h"
#include "SPI.h"
#include <TinyGPS++.h>

//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600

const double EIFFEL_TOWER_LAT = 48.85826;
const double EIFFEL_TOWER_LNG = 2.294516;

float coords[6];
float lastFix[2];
float oldTime = 0;
float newTime = 0;
double distanceFromLastFix;
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

void getCoords(float array[]);

void setup()
{
    Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
    Serial1.begin(9600);//This opens up communications to the GPS
    Serial.println("GPS Start");//Just show to the monitor that the sketch has started
}

void loop()
{
    //query GPS
    getCoords(coords);
    //if there is no satellites, no valid fix can be made thus retry
    while(coords[0] == 0){
        getCoords(coords);
    }
    if(coords[1] != oldTime){
        Serial.print("Sats: ");
        Serial.println(coords[0],0);
        Serial.print("Time: ");
        Serial.println(coords[1],0);
        newTime = coords[1];
        Serial.print("LAT: ");
        Serial.println(coords[2],6);
        Serial.print("LNG: ");
        Serial.println(coords[3],6);
        Serial.print("Speed(m/s): ");
        Serial.println(coords[4]);
        Serial.print("altitude(m): ");
        Serial.println(coords[5]);
        Serial.println("");
        //distanceFromLastFix = gps.distanceBetween(EIFFEL_TOWER_LAT,EIFFEL_TOWER_LNG,coords[2],coords[3]);
        distanceFromLastFix = gps.distanceBetween(lastFix[0],lastFix[1],coords[2],coords[3]);
        Serial.print("Distance since last fix: ");
        Serial.println(distanceFromLastFix);
        lastFix[0] = coords[2];
        lastFix[1] = coords[3];
        Serial.println("");
        Serial.print("Time passed: ");
        int timePassed = (newTime - oldTime)/100;
        Serial.print(timePassed);
        Serial.println(" Seconds");
        oldTime = newTime;
        Serial.print("Speed: ");
        Serial.print(distanceFromLastFix/timePassed);
        Serial.println(" M/S");
        Serial.println("\n\n\n");
    }

}

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
//if age > 1500 consider lost fix


/// Best use case; offload the reading and encoding onto a protoThread
/// Use the static GPS object to query for the coords at a more reasonable interval
/// On start, wait until the GPS is acquired before taking flight