#include <TinyGPS++.h>
#include "Arduino.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_MPU6050.h"
#include "protothreads.h"
#include "Servo.h"

#include "../.pio/libdeps/due/E220Lib/E220.h"


static struct pt pt1;
static struct pt pt2;
static struct pt gpsThread;

static int FMC(struct pt *pt);
static int ProtoThread2(struct pt *pt);
static int GPSThread(struct pt *pt,float array[], TinyGPSPlus GPS);

//Define the ESC control pins
#define ESC_L  7
#define ESC_LR  6
#define ESC_LF  5

#define ESC_R  2
#define ESC_RR  3
#define ESC_RF  4


//setup the servo objects for the motor
Servo motor_L;
Servo motor_LR;
Servo motor_LF;
Servo motor_R;
Servo motor_RR;
Servo motor_RF;


//Setup the pins for the radio
#define m0 7
#define m1 6
#define aux 5


//BME/P-280
//connect via I2C ports
//VCC 5v
Adafruit_BMP280 bmp;


//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600
Stream &GPS_Connection = (Stream &)Serial1;
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
float coords[6];

//Motion processing unit, our gyroscope
Adafruit_MPU6050 mpu;
//adjustments for the MPU
double adjust_x = -8.26;
double adjust_y = -2.55;
double adjust_z = 0;

//Setup the lora radio for telemetry, command and control
Stream &RadioConnection = (Stream &)Serial2;



int flightMode = 0;

void altimeterInit();
void startGyro();
void startRadio();

void getCoords(float array[]);
void idleAll();
void writeAll(int speed);

void setup()
{
    delay(1000);
    Serial.begin(9600);
    Serial.println("SYSTEM INITIALISING");


    PT_INIT(&pt1);
    PT_INIT(&pt2);

    ///STARTUP CHECKLIST

    //setup the motors, set to idle
    motor_L.attach(ESC_L);
    motor_LR.attach(ESC_LR);
    motor_LF.attach(ESC_LF);

    motor_R.attach(ESC_R);
    motor_RR.attach(ESC_RR);
    motor_RF.attach(ESC_RF);
    idleAll();
    Serial.println("MOTORS READY");
    /*
    //setup the GPS, check the data is good
    GPS_Connection.begin(9600);//This opens up communications to the GPS
    while(coords[0] < 2){
        getCoords(coords);
    }
    Serial.println("GPS STARTED");
     */
    //Start the Altimeter, check its connected
    altimeterInit();
    startGyro();
    //startRadio();
    Serial.println("SYSTEM INITIALISED");
}




void loop() {
    PT_SCHEDULE(FMC(&pt1));
    PT_SCHEDULE(ProtoThread2(&pt2));
    //PT_SCHEDULE(GPSThread(&gpsThread, coords, gps));
    motor_RF.writeMicroseconds(1050);
    delay(1000);
    idleAll();
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

static int FMC(struct pt *pt)
{
    PT_BEGIN(pt);
    while(1) {
        ///the FMC will have x modes, TAKE OFF, HOVER, NAVIGATE TO, LAND, LANDED, QLanded
        ///We will have a single variable take a value relating to each mode, and at the start of the loop the system will check the variable
        ///For mode switches, We can simply halt the running protothread with the old mode and start the new protothread
        ///NAV speed will be limited to 1.5ms, with a possible further mode for cautious travel near obstacles being limited to 0.5ms
        ///Will have an mode array, contains a list of instructions to carry out sequentially. If empty, wait 60 seconds before performing return home
        ///TRANSITIONS:
        ///     Hover
        ///         If landed, schedule take off, schedule hover
        ///         If Navigating, complete navigate proto and start hover
        ///         If hover, Do nothing
        ///     Navigate To:
        ///         Reads from an array of coordinates, inputs as current nav point, generate heading from GPS+compass and proceeds to nav point
        ///         If landed, schedule take off, hover, navigate
        ///         If Hover, schedule Navigate
        ///         If Navigating, do nothing
        ///     Land:
        ///         Sets vertical speed to 20cm/s
        ///         If hover, schedule land
        ///         If Navigating, schedule hover, land
        ///         If Landing/landed do nothing
        ///     Landed:
        ///         Only scheduled from landing
        ///         Once vertical speed == 0 and the sensor range(adjusted) == 0
        ///         Set motors to 0 (off), strobe lights, start audio beacon
        ///     QLanded:
        ///         Quiet landed, only active when in landed mode and some input (button or web server) is set
        ///         Turns off audio beacon, possible stops lights flashing
        /// FLIGHT CONTROL
        ///                     1          2
        ///                      \        /
        ///                   3---        ---4
        ///                      /        \
        ///                     5          6
        ///Motors operate as whole set or pairs
        ///TO CLIMB/descend:
        ///     All motors increase in RPM together till vertical speed limit is met, inverse to descend
        /// Forward and backward movement
        ///     Pairs of motors, 1+2 and 5+6 increase and decrease RPM inversely, to go forward, 1+2 drop by 2%, 5+6 increase by 2%
        ///Rotation:
        ///     ??? send help
        ///     Work in pairs of motors, 1+6 and 2+5, decrease one pair and increase one pair to induce torque on the body


        PT_SLEEP(pt,100)
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


void altimeterInit(){
    Serial.begin(9600);
    while(!bmp.begin(0x76)){
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!\n\n"));
        delay(5000);
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
        altimeterInit();
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
void startRadio(){
    E220 radioModule(&RadioConnection, m0, m1, aux);
    while(!radioModule.init()){
        delay(5000);
    }
    Serial.println("Radio Ready");
}

void startGyro(){
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        delay(1000);
        startGyro();
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}


void idleAll(){
   motor_L.writeMicroseconds(1000);
   motor_LR.writeMicroseconds(1000);
   motor_LF.writeMicroseconds(1000);
   motor_R.writeMicroseconds(1000);
   motor_RR.writeMicroseconds(1000);
   motor_RF.writeMicroseconds(1000);
}

void writeAll(int speed){
    motor_L.writeMicroseconds(speed);
    motor_LR.writeMicroseconds(speed);
    motor_LF.writeMicroseconds(speed);
    motor_R.writeMicroseconds(speed);
    motor_RR.writeMicroseconds(speed);
    motor_RF.writeMicroseconds(speed);
}



///LIGHTS
///Nav lights do not blink, red and green
///Anti-collision do blink, red or white