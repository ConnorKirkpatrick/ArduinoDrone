#include <Arduino.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_I2CDevice.h"
#include "SPI.h"
#include <TinyGPS++.h>

//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600

float coords[6];

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
    Serial.println(Serial1.read());
    //query GPS
    getCoords(coords);
    //if there is no satellites, no valid fix can be made thus retry
    while(coords[0] == 0){
        getCoords(coords);
    }
    Serial.print("Sats: ");
    Serial.println(coords[0],0);
    Serial.print("Time: ");
    Serial.println(coords[1],0);
    Serial.print("LAT: ");
    Serial.println(coords[2],6);
    Serial.print("LNG: ");
    Serial.println(coords[3],6);
    Serial.print("Speed(m/s): ");
    Serial.println(coords[4]);
    Serial.print("altitude(m): ");
    Serial.println(coords[5]);
    Serial.println("\n\n\n");
    delay(1000);
}

void getCoords(float array[]){
    Serial.println(Serial1.read());
    while(!gps.location.isUpdated()){
        while(Serial1.available())//While there are characters to come from the GPS
        {
            gps.encode(Serial1.read());//This feeds the serial NMEA data into the library one char at a time
        }
    }
    array[0] = gps.satellites.value();
    array[1] = gps.time.value();
    array[2] = gps.location.lat();
    array[3] = gps.location.lng();
    array[4] = gps.speed.mps();
    array[5] = gps.altitude.meters();


/*    if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
    {

//Get the latest info from the gps object which it derived from the data sent by the GPS unit
        Serial.println("GMT Time:");
        Serial.println(gps.time.value());
        Serial.println("Satellite Count:");
        Serial.println(gps.satellites.value());
        Serial.println("Latitude:");
        Serial.println(gps.location.lat(), 6);
        Serial.println("Longitude:");
        Serial.println(gps.location.lng(), 6);
        Serial.println("Speed MPH:");
        Serial.println(gps.speed.mph());
        Serial.println("Altitude Feet:");
        Serial.println(gps.altitude.feet());
        Serial.println("");
    }*/

}



/// Best use case; offload the reading and encoding onto a protoThread
/// Use the static GPS object to query for the coords at a more reasonable interval
/// On start, wait until the GPS is acquired before taking flight