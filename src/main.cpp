#include "Arduino.h"
#include <TinyGPS++.h>
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
void altimeterInit();


//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data
float coords[6];
float lastFix[2];
double distanceFromLastFix;
void startGPS();
void getCoords(float array[]);

//Motion processing unit, our gyroscope
Adafruit_MPU6050 mpu;
//adjustments for the MPU
double adjust_x = -8.26;
double adjust_y = -2.55;
double adjust_z = 0;
struct attitude{
    double pitch;
    double roll;
    double yaw;
    double verticalA;
    double lateralA;
    double directionalA;
} currentAttitude, calibrationAttitude;
double pitchOffset = 0;
double rollOffset = 0;
double yawOffset = 0;
double verticalOffset = 0;
double directionalOffset = 0;
double lateralOffset = 0;

float oldTime = 0;
float newTime = 0;


void startGyro();
attitude getAttitude();


//Setup the lora radio for telemetry, command and control
Stream &RadioConnection = (Stream &)Serial2;



int flightMode = 0;


void startRadio();


void idleAll();
void writeAll(int speed);

void setup()
{
    delay(1000);
    Serial.begin(9600);
    //Serial1.begin(9600);
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
    //Start the gps
    startGPS();
    //Start the Altimeter, check its connected
    altimeterInit();
    startGyro();
    //startRadio();
    Serial.println("SYSTEM INITIALISED");
    oldTime = millis();
}




void loop() {
    /*
    PT_SCHEDULE(FMC(&pt1));
    //PT_SCHEDULE(ProtoThread2(&pt2));
    //PT_SCHEDULE(GPSThread(&gpsThread, coords, gps));
    motor_RF.writeMicroseconds(1050);
    delay(1000);
    idleAll();
     */
    /// speed = acceleration*duration
    currentAttitude = getAttitude();
    /*
    getCoords(coords);
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

static int FMC(struct pt *pt)
{
    attitude newAttitude;
    PT_BEGIN(pt);
    while(1) {
        ///the FMC will have x modes, TAKE OFF, HOVER, NAVIGATE TO, LAND, LANDED, QLanded
        ///We will have a single variable take a value relating to each mode, and at the start of the loop the system will check the variable
        ///For mode switches, We can simply halt the running protothread with the old mode and start the new protothread
        ///NAV speed will be limited to 1.5ms, with a possible further mode for cautious travel near obstacles being limited to 0.5ms
        ///Will have an mode array, contains a list of instructions to carry out sequentially. If empty, wait 60 seconds before performing return home
        /// The back of the drone will be in line with the serial ports of the due
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

        newAttitude = getAttitude();
        Serial.println(newAttitude.pitch);
        Serial.println("P, ");
        Serial.println(newAttitude.roll);
        Serial.println("R, ");
        Serial.println(newAttitude.yaw);
        Serial.println("Y\n");
        Serial.print(newAttitude.verticalA);
        Serial.println("V, ");
        Serial.print(newAttitude.directionalA);
        Serial.println("D, ");
        Serial.print(newAttitude.lateralA);
        Serial.println("L, ");
        Serial.println("\n");

        PT_SLEEP(pt,50);
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

void startGPS(){
    Serial1.begin(9600);
    getCoords(coords);
    //if there is no satellites, no valid fix can be made thus retry
    while(coords[0] < 1) {
        getCoords(coords);
    }
    Serial.println("GPS Started");
    lastFix[0] = coords[2];
    lastFix[1] = coords[3];
    oldTime = coords[1];
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

void altimeterInit(){
    while(!bmp.begin(0x76)){
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!\n\n"));
        delay(5000);
    }
    Serial.println("Altimeter Started");
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
    double offsetP = 0;
    double offsetR = 0;
    double offsetY = 0;
    double offsetV = 0;
    double offsetD = 0;
    double offsetL = 0;
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        delay(1000);
        startGyro();
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    delay(2000);

    //take 5 samples over a few seconds to get the default offsets

    for(int i = 0; i < 10; i++){
        calibrationAttitude = getAttitude();
        offsetP = offsetP + calibrationAttitude.pitch;
        offsetR = offsetR + calibrationAttitude.roll;
        offsetY = offsetY + calibrationAttitude.yaw;

        offsetV = offsetV + calibrationAttitude.verticalA;
        offsetD = offsetD + calibrationAttitude.directionalA;
        offsetL = offsetL + calibrationAttitude.lateralA;

    }
    pitchOffset = offsetP/10;
    rollOffset = offsetR/10;
    yawOffset = offsetY/10;

    verticalOffset = offsetV/10;
    directionalOffset = offsetD/10;
    lateralOffset = offsetL/10;
    Serial.println("Gyro Started");
}

attitude getAttitude(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    currentAttitude.verticalA = a.acceleration.z - verticalOffset;
    currentAttitude.lateralA = a.acceleration.y - lateralOffset;
    currentAttitude.directionalA = a.acceleration.x - directionalOffset;
    double x;
    double y;
    double z;
    const int MPU_addr=0x68;
    int minVal=265;
    int maxVal=402;
    int xAng;
    int yAng;
    int zAng;
    int16_t AcX,AcY,AcZ;
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);
    AcX=Wire.read()<<8|Wire.read();
    AcY=Wire.read()<<8|Wire.read();
    AcZ=Wire.read()<<8|Wire.read();
    xAng = map(AcX,minVal,maxVal,-90,90);
    yAng = map(AcY,minVal,maxVal,-90,90);
    zAng = map(AcZ,minVal,maxVal,-90,90);
    x= (RAD_TO_DEG * (atan2(-yAng, -zAng)+PI))-adjust_x;
    y= (RAD_TO_DEG * (atan2(-xAng, -zAng)+PI))-adjust_y;
    z= (RAD_TO_DEG * (atan2(-yAng, -xAng)+PI))-adjust_z;
    if(x>=360){x = x-360;}
    if(y>=360){y = y-360;}
    if(z>=360){z = z-360;}
    if(x<0){x = x+360;}
    if(y<0){y = y+360;}
    if(z<0){z = z+360;}

    x = x - rollOffset;
    y = y - pitchOffset;
    z = z - yawOffset;

    if(x<= -180){x = 360+x;}
    if(y<= -180){y = 360+y;}
    if(z<= -180){z = 360+z;}
    currentAttitude.pitch = y;
    currentAttitude.roll = x;
    currentAttitude.yaw = z;
    return currentAttitude;
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