#include <Arduino.h>
#include <protothreads.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BMP280.h>



static struct pt pt1;
static struct pt pt2;
static struct pt gpsThread;

static int ProtoThread1(struct pt *pt);
static int ProtoThread2(struct pt *pt);
static int GPSThread(struct pt *pt,float array[], TinyGPSPlus GPS);

//BME/P-280
//connect via I2C ports
//VCC 5v
Adafruit_BMP280 bmp;

//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600
SoftwareSerial GPS_Connection(10, 11); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
float coords[6];

void altimeter();
void getCoords(float array[]);

void setup()
{
    delay(5000);
    Serial.begin(9600);
    Serial.println("SYSTEM INITIALISING");


    PT_INIT(&pt1);
    PT_INIT(&pt2);

    GPS_Connection.begin(9600);//This opens up communications to the GPS
    //Startup checks
    while(coords[0] < 2){
        getCoords(coords);
    }
    Serial.println("GPS STARTED");
    altimeter();
    Serial.println("SYSTEM INITIALISED");
}




void loop() {
    PT_SCHEDULE(ProtoThread1(&pt1));
    PT_SCHEDULE(ProtoThread2(&pt2));
    PT_SCHEDULE(GPSThread(&gpsThread, coords, gps));
}

void getCoords(float array[]) {
    while (!gps.location.isUpdated()) {
        while (GPS_Connection.available())//While there are characters to come from the GPS
        {
            gps.encode(GPS_Connection.read());//This feeds the serial NMEA data into the library one char at a time
        }
    }
    array[0] = gps.satellites.value();
    array[1] = gps.time.value();
    array[2] = gps.location.lat();
    array[3] = gps.location.lng();
    array[4] = gps.speed.mps();
    array[5] = gps.altitude.meters();
}

static int ProtoThread1(struct pt *pt)
{
    PT_BEGIN(pt);
    while(1) {

        PT_SLEEP(pt,1500)
    }
    PT_END(pt)
}
static int ProtoThread2(struct pt *pt)
{
    PT_BEGIN(pt);
    while(1) {
        Serial.println("Printing every 5 seconds");
        PT_SLEEP(pt,5000)
    }
    PT_END(pt);
}
static int GPSThread(struct pt *pt,float array[], TinyGPSPlus GPS){
    PT_BEGIN(pt);
    while(1) {
        getCoords(array);
        PT_SLEEP(pt,1000)
    }
    PT_END(pt)
}


///Notes:
/// Dont use local variables with PT, use static as local are not preserved during a block
/// any protothread function must have a pointer to the protothread as a parameter
/// PT Begin and End start and stop the thread
/// Use PT_WAIT_UNTIL(thread,condition), this will keep the thread paused until the condition is true
/// Trick, you can call the wait immediately after an action is performed to preserve memory
/// I will use this initially to blink the nav lights on the drone, later to expand the flight controllers autonomy
/// DO NOT USE INTERRUPTS: these block the main process and can cripple the FC system



/// Best use case; offload the reading and encoding onto a protoThread
/// Use the static GPS object to query for the coords at a more reasonable interval
/// On start, wait until the GPS is acquired before taking flights


void altimeter(){
    Serial.begin(9600);
    while(!bmp.begin(0x76)){
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!\n\n"));
        delay(3000);
    }
    Serial.println("BMP-280 started");
}

float getAltitude(){
    Serial.println(bmp.getStatus());
    //243 is a bad status->no gnd, no recovery
    //0 is also bad, need to re-init
    //0Ã is bad, no power, can return when power is restored
    //12,4 is good
    if(bmp.getStatus() == 243 || bmp.getStatus() == 0){
        altimeter();
    }

/*    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Approx altitude = ");
    Serial.print(bmp.readAltitude(1013.25)); *//* Adjusted to local forecast! *//*
    Serial.println(" m");

    Serial.println();*/
    return bmp.readAltitude();
}