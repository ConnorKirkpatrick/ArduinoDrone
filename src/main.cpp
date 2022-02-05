#include "Arduino.h"
#include "TinyGPS++.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_MPU6050.h"
#include "Servo.h"

#include <MedianFilterLib2.h>

#include "../.pio//libdeps/due/E220Lib/src/E220.h"

#include "math.h"


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
Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;
Servo motor_5;
Servo motor_6;

//setup the throttle values
int throttle = 1000; //arming value
int throttleLim = 1200;

int T1 = 0;
int T2 = 0;
int T3 = 0;
int T4 = 0;
int T5 = 0;
int T6 = 0;
void test();
//Setup the lora radio for telemetry, command and control
Stream &RadioConnection = (Stream &)Serial2;
String message;
#define m0 8
#define m1 9
#define aux 13
int lastMessageTime;
int currentTime;

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
} currentAttitude, calibrationAttitude, oldAttitude;
double pitchOffset = 0;
double rollOffset = 0;
double yawOffset = 0;
float pitchVal;
float rollVal;
float yawVal;


void startGyro();
attitude getAttitude();



//define the objects for the ultrasonic sensors (Odd value is the trigger)
#define heightTrigger 39
#define heightEcho 38

long duration;
int distance;
MedianFilter2<int> heightFilter(5);
void startUltrasonic();
void getUltrasonic();


//battery sensor
#define ampPin A0
#define voltPin A1
///We scale 180a over 1024 points, so each point = 0.17578 a
#define ampScale 0.176 //better to overestimate the useage than run out of amps and crash
///We scale 50v over 1024 points, thus each point = 0.04882 v
#define voltScale 0.04882 // Better to underestimate as before
float amps = 0;
float voltage = 0;
void getBatteryValues();



///flight control
void pitch(float pitchVal,float oldPitch);
int pitchAdjust = 0;
void roll(float rollVal, float oldRoll);
int rollAdjust = 0;

void setThrottle();
int desiredHeight ;
int oldHeight;

void setSpeed();

int flightMode = 0;

void idleAll();
void writeAll(int speed);

void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial2.begin(9600);
    Serial.println("SYSTEM INITIALISING");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
    ///STARTUP CHECKLIST
    E220 radioModule(&RadioConnection, m0, m1, aux);
    while(!radioModule.init()){
        delay(5000);
    }
    RadioConnection.println("Radio Ready");
    delay(1000);
    //setup the motors, set to idle
    idleAll();
    motor_1.attach(5);
    motor_2.attach(4);
    motor_3.attach(7);
    motor_4.attach(2);
    motor_5.attach(6);
    motor_6.attach(3);
    RadioConnection.println("MOTORS READY");

    //Start the gps
    ///startGPS();
    //Start the Altimeter, check its connected
    altimeterInit();
    //start the gyro
    startGyro();
    startUltrasonic();

    //start battery monitor
    pinMode(ampPin, INPUT);
    pinMode(voltPin, INPUT);
    getBatteryValues();

    oldAttitude = getAttitude();
    lastMessageTime = millis();

    RadioConnection.println("SYSTEM INITIALISED");


}




void loop() {
    currentTime = millis();
    if(RadioConnection.available()){
        message = RadioConnection.readString();
    }
    if(message.equals("Start\n")){
        RadioConnection.println("Starting flight");
        flightMode = 1;
        message = "";
    }
    else if(message.indexOf("setHeight") > -1){
        desiredHeight = message.substring(message.indexOf("setHeight")+9).toInt();
        RadioConnection.write("New height set to: ");
        RadioConnection.println(desiredHeight);
        message = "";
    }
    else if(message.indexOf("setLim") > -1){
        throttleLim = message.substring(message.indexOf("setLim")+6).toInt();
        RadioConnection.write("New Limit set to: ");
        RadioConnection.println(throttleLim);
        message = "";
    }
    else if(message.equals("end\n")){
        RadioConnection.println("ENDING");
        flightMode = 0;
        idleAll();
        message = "";
    }
    else if(message.equals("Test\n")){
        RadioConnection.println("TESTING");
        message = "";
        test();
    }



    ///Ping the sensor for 3 attitude readings, then average them out for our use
    currentAttitude = getAttitude();
    pitchVal = (float)(round(currentAttitude.pitch*10))/10;
    rollVal = (float)(round(currentAttitude.roll*10))/10;
    yawVal = (float)(round(currentAttitude.yaw*10))/10;
    currentAttitude = getAttitude();
    pitchVal = pitchVal+((float)(round(currentAttitude.pitch*10))/10);
    rollVal = rollVal+((float)(round(currentAttitude.roll*10))/10);
    yawVal = yawVal+((float)(round(currentAttitude.yaw*10))/10);
    currentAttitude = getAttitude();
    pitchVal = (pitchVal+((float)(round(currentAttitude.pitch*10))/10))/3;
    rollVal = (rollVal+((float)(round(currentAttitude.roll*10))/10))/3;
    yawVal = (yawVal+((float)(round(currentAttitude.yaw*10))/10))/3;

    ///ping the ultrasonic sensor for 5 readings to refresh the average buffer
    getUltrasonic();
    getUltrasonic();
    getUltrasonic();
    getUltrasonic();
    getUltrasonic();

    getBatteryValues();

    if(flightMode == 1){
        //todo: Attitude delta values
        pitch(pitchVal,oldAttitude.pitch);
        roll(rollVal,oldAttitude.roll);
        //yaw(yawVal,oldAttitude.yaw);
        setSpeed();
        setThrottle();
        oldAttitude.pitch = pitchVal;
        oldAttitude.roll = rollVal;
        oldAttitude.yaw = yawVal;
    }

    ///Hovering:
    /// all motors have a fixed throttle setting between 1000 and 2000
    /// we check pitch, roll then yaw
    /// pitch targets motors 1,2 and 5,6. where any pair increases, the other pair decreases the same amount
    /// roll targets trios 1,3,5 and 2,4,6. same principle, where one set is affected inversely to the other
    /// yaw targets 1,4,5 and 2,3,6. same as above
    /// throttle is controlled by the range/altitude sensors, it does not change based on pitch
    /// attitude controls all make their adjustments, the average of all these changes is taken as the new setting. in future there will be a way to better combine them
    /// end of loop is setting the new speed.
    /// we work on degrees, where each degree is multiplied by a value to get the new change in(?) or fixed with 1 degree per 5 points increase
    /// could do every 0.1 degree for 1 point increase, gives 900 points out of our 1000 range
    ///     However throttle will probs be 300 degrees so we are going over, but probs will never see > 30 degrees, so 300 puts us at comfortable 600

    //Serial.println(voltage);
    //Serial.println(amps);
    //Serial.println(currentTime - lastMessageTime);

    if(currentTime - lastMessageTime > 1250){
        char data [40];
        sprintf(data,"%d,%d,%f,%f,%f,%f",heightFilter.GetFiltered(), throttle, pitchVal, rollVal, voltage, amps);
        Serial.println(data);
        RadioConnection.println(data);
        lastMessageTime = currentTime;
        RadioConnection.println(analogRead(ampPin));
    }

}

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
        ///     Work in pairs of motors, 1+4+5 and 2+3+6, decrease one set and increase one set to induce torque on the body



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
    getCoords(coords);
    //if there is no satellites, no valid fix can be made thus retry
    while(coords[0] < 1) {
        getCoords(coords);
    }
    RadioConnection.println("GPS Started");
    lastFix[0] = coords[2];
    lastFix[1] = coords[3];
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
        RadioConnection.println(F("Could not find a valid BMP280 sensor, check wiring or "
                         "try a different address!\n\n"));
        delay(5000);
    }
    RadioConnection.println("Altimeter Started");
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

void startGyro(){
    double offsetP = 0;
    double offsetR = 0;
    double offsetY = 0;
    double offsetV = 0;
    double offsetD = 0;
    double offsetL = 0;
    if (!mpu.begin()) {
        RadioConnection.println("Failed to find MPU6050 chip");
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
        delay(100);

    }
    pitchOffset = offsetP/10;
    rollOffset = offsetR/10;
    yawOffset = offsetY/10;
    RadioConnection.println("Gyro Started");
}

attitude getAttitude(){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
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

    if(x > 180){x = x-360;}

    currentAttitude.pitch = y;
    currentAttitude.roll = x;
    currentAttitude.yaw = z;
    return currentAttitude;
}

void startUltrasonic(){
    pinMode(heightTrigger, OUTPUT);
    pinMode(heightEcho, INPUT);
}
void getUltrasonic(){
    //get the system ready
    digitalWrite(heightTrigger, LOW);
    delayMicroseconds(5);
    //send out a pulse
    digitalWrite(heightTrigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(heightTrigger, LOW);
    //read the pulse
    duration = pulseIn(heightEcho, HIGH);
    //calculate distance by the delay
    distance = duration * 0.034 / 2;
    heightFilter.AddValue(distance);
    //Ideally we are running all the US sensors in this window due to the required 15us wait time
}

void getBatteryValues(){
    amps = analogRead(ampPin)*ampScale;
    voltage = analogRead(voltPin)*voltScale;
}



void idleAll(){
   motor_1.writeMicroseconds(1000);
   motor_2.writeMicroseconds(1000);
   motor_3.writeMicroseconds(1000);
   motor_4.writeMicroseconds(1000);
   motor_5.writeMicroseconds(1000);
   motor_6.writeMicroseconds(1000);
   RadioConnection.println("IDLE SET");
}

float mag(float value){
    return sqrt(sq(value));
}

///ATTITUDE CONTROL
void pitch(float pitchVal, float oldPitch){
    //check the delta,
    if(mag(pitchVal) < mag(oldPitch)){
        ///lower magnitude means we are actively correcting, thus dont change the inputs
    }
    else{
        ///we have a higher or equal mag, thus not correcting
        if(pitchVal > 0){
            pitchAdjust = pitchAdjust + 5;
        }
        else{
            pitchAdjust = pitchAdjust - 5;
        }
        ///pitch is positive, we are nose high, thus decrease front and increase rear
        int newSpeed = throttle - pitchAdjust;
        T1 = T1 + newSpeed;
        T2 = T2 + newSpeed;
        newSpeed = throttle + pitchAdjust;
        T5 = T5 + newSpeed;
        T6 = T6 + newSpeed;

        T3 = T3 + throttle;
        T4 = T4 + throttle;
    }
}

void roll(float rollValue, float oldRoll){
    if(mag(pitchVal) < mag(oldRoll)){
        ///lower magnitude means we are actively correcting, thus dont change the inputs
    }
    else{
        ///we have a higher or equal mag, thus not correcting
        if(rollValue > 0){
            rollAdjust = rollAdjust + 5;
        }
        else{
            rollAdjust = rollAdjust - 5;
        }
        ///roll is positive, we are rolling right, decrease left and increase right
        int newSpeed = throttle - rollAdjust;
        //left motors
        T1 = T1 + newSpeed;
        T3 = T3 + newSpeed;
        T5 = T5 + newSpeed;
        newSpeed = throttle + rollAdjust;
        //right motors
        T2 = T2 + newSpeed;
        T4 = T4 + newSpeed;
        T6 = T6 + newSpeed;
    }

    ///rolling right is positive
    ///left motors are 1,3,5. Right are 2,4,6
    ///if pitched right, we add to right motors, subtract from left

}

void setThrottle(){
    ///calculate height delta
    int currentHeight = heightFilter.GetFiltered();
    if(currentHeight - oldHeight > 0){
        ///we are currently accenting
        if(currentHeight < desiredHeight){
            ///we are correcting towards desired, thus do nothing
        }
        else{
            ///we are not correcting, thus increase throttle
            ///TODO: INCREASE
        }
    }
    if(currentHeight - oldHeight == 0){
        ///we are stable in the air, do nothing
    }
    else{
        ///we are currently descending
        if(currentHeight > desiredHeight){
            ///we are correcting towards desired height, thus do nothing
        }
        else{
            ///We are not correcting, thus increase throttle
            ///TODO: INCREASE
        }
    }


    //we are below desired height
    if(currentHeight < desiredHeight - 10){
        Serial.println("Increasing power");
        if(currentHeight > oldHeight){
            oldHeight = currentHeight;
            //do nothing as we are already ascending
        }
        else{
            throttle = throttle + 10;
        }
    }
    //we are above desired height
    else if(currentHeight > desiredHeight + 10){
        Serial.println("Decreasing power");
        if(currentHeight < oldHeight){
            oldHeight = currentHeight;
            //do nothing as we are already descending
        }
        else{
            throttle = throttle - 10;
        }
    }
    //we are withing the bounds for the altitude
    else{
        Serial.println("Fine Tuning");
        if(currentHeight > oldHeight){
            //we are ascending, reduce power
            throttle = throttle - 5;
        }
        else if(currentHeight < oldHeight){
            //we are descending, increase power
            throttle = throttle + 5;
        }
    }
    if(throttle > throttleLim){
        throttle = throttleLim;
    }
    if(throttle < 1000){
        throttle = 1000;
    }
    oldHeight = currentHeight;
}

void setSpeed(){
    T1 = T1/2;
    T2 = T2/2;
    T3 = T3/2;
    T4 = T4/2;
    T5 = T5/2;
    T6 = T6/2;

    Serial.println(T1);
    Serial.println(T2);
    Serial.println(T3);
    Serial.println(T4);
    Serial.println(T5);
    Serial.println(T6);
    Serial.println();
    if(T1 < 1050){T1 = 1050;}
    if(T2 < 1050){T2 = 1050;}
    if(T3 < 1050){T3 = 1050;}
    if(T4 < 1050){T4 = 1050;}
    if(T5 < 1050){T5 = 1050;}
    if(T6 < 1050){T6 = 1050;}

    if(T1 > throttleLim){T1 = throttleLim;}
    if(T2 > throttleLim){T2 = throttleLim;}
    if(T3 > throttleLim){T3 = throttleLim;}
    if(T4 > throttleLim){T4 = throttleLim;}
    if(T5 > throttleLim){T5 = throttleLim;}
    if(T6 > throttleLim){T6 = throttleLim;}

    motor_1.writeMicroseconds(T1);
    motor_2.writeMicroseconds(T2);
    motor_3.writeMicroseconds(T3);
    motor_4.writeMicroseconds(T4);
    motor_5.writeMicroseconds(T5);
    motor_6.writeMicroseconds(T6);

    T1 = 0;
    T2 = 0;
    T3 = 0;
    T4 = 0;
    T5 = 0;
    T6 = 0;
}
void test(){
    RadioConnection.println("Starting Test....");
    delay(5000);
    digitalWrite(LED_BUILTIN, HIGH);
    motor_1.writeMicroseconds(1050);
    motor_2.writeMicroseconds(1050);
    motor_3.writeMicroseconds(1050);
    motor_4.writeMicroseconds(1050);
    motor_5.writeMicroseconds(1050);
    motor_6.writeMicroseconds(1050);
    delay(2000);
    motor_1.writeMicroseconds(1000);
    motor_2.writeMicroseconds(1000);
    motor_3.writeMicroseconds(1000);
    motor_4.writeMicroseconds(1000);
    motor_5.writeMicroseconds(1000);
    motor_6.writeMicroseconds(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1500);
    RadioConnection.println("Test completed");
}
///LIGHTS
///Nav lights do not blink, red and green
///Anti-collision do blink, red or white


///TODO: REDUCE MOTOR SENSITIVITY
    //Add code to allow for the system to see a change before it executes the next update
    //work on hovering code before anything else
    //limit throttle to about 1200
    //fix analog pin for amps
    //test at 1200 next