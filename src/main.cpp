#include "Arduino.h"
#include "TinyGPS++.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_MPU6050.h"
#include "Servo.h"

#include <MedianFilterLib2.h>
#include "Average.h"
#include "E220.h"

#include "math.h"

#include "Wire.h"
#include "Adafruit_HMC5883_U.h"


void(* resetFunc) (void) = 0;

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

int esc_1 = 0;
int esc_2 = 0;
int esc_3 = 0;
int esc_4 = 0;
int esc_5 = 0;
int esc_6 = 0;

void test();
///Setup the lora radio for telemetry, command and control
Stream &RadioConnection = (Stream &)Serial2;
String message;
#define m0 8
#define m1 9
#define aux 22
int lastMessageTime = 0;
int currentTime = 0;

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
struct attitude{
    double pitch;
    double roll;
    double yaw;
} currentAttitude, calibrationAttitude, oldAttitude;
double pitchOffset = 0;
double rollOffset = 0;
double yawOffset = 0;
Average<int> pitchFilter(5);
Average<int> rollFilter(5);

float desiredPitch;
float desiredRoll;
float desiredYaw;

float pitchTrim = 0;
float rollTrim = 0;
float yawTrim = 0;



void startGyro();
attitude getAttitude();

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float magDeclanation = 0.19;
void startCompass();
int getHeading();
int desiredHeading;
int oldHeading;

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
void yaw(int heading);
int yawAdjust = 0;
//PID Params
float pGainPitch = 0.3;
float iGainPitch = 0.3;
float dGainPitch = 0.15;

float pGainRoll = 0.3;
float iGainRoll = 0.3;
float dGainRoll = 0.15;

float pGainYaw = 0.3;
float iGainYaw = 0.3;
float dGainYaw = 0.15;
//Pitch PID
void pitchPID(double currentPitch);
int pitchCurrentTime = 0;
int pitchOldTime = 0;
long pitchPorportional = 0;
long pitchLastError = 0;
long pitchIntegral = 0;
long pitchDerivitive= 0;
//roll PID
void rollPID(double currentRoll);
int rollCurrentTime = 0;
int rollOldTime = 0;
long rollPorportional = 0;
long rollLastError = 0;
long rollIntegral = 0;
long rollDerivitive= 0;
//yaw PID
void yawPID(double currentYaw);
int yawCurrentTime = 0;
int yawOldTime = 0;
long yawPorportional = 0;
long yawLastError = 0;
long yawIntegral = 0;
long yawDerivitive= 0;

void throttleAdjust();

void setThrottle();
int desiredHeight ;
int oldHeight;

void setSpeed();

int flightMode = 0;

void idleAll();
void writeAll(int speed);


volatile int pwm_value = 0;
volatile int prev_time = 0;
int RC_Throttle;
void rising();
void falling();

void setup()
{
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial2.begin(9600);
    Serial.println("SYSTEM INITIALISING");
    attachInterrupt(digitalPinToInterrupt(12), rising, RISING);
    ///STARTUP CHECKLIST
    ///Starting the radio
    E220 radioModule(&RadioConnection, m0, m1, aux);
    while(!radioModule.init()){
        Serial.println("Waiting for radio");
        delay(5000);
    }
    Serial.println("Radio Ready");
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


    ///Start the gps
    //startGPS();
    ///Start the Altimeter, check its connected
    //altimeterInit();
    //RadioConnection.println("Altimeter Started");
    ///start the gyro
    startGyro();
    startCompass();
    //startUltrasonic();

    ///start battery monitor
    pinMode(ampPin, INPUT);
    pinMode(voltPin, INPUT);
    getBatteryValues();

    oldAttitude = getAttitude();
    lastMessageTime = millis();

    RadioConnection.println("SYSTEM INITIALISED");
    RadioConnection.println("ENABLE CONTROLLER");

    flightMode = 1;
    yawLastError = getHeading();
    desiredYaw = yawLastError;
    Serial.println(yawLastError);
}




void loop() {
    if(RadioConnection.available()){
        message = RadioConnection.readString();
    }
    if(message.equals("Start\n")){
        RadioConnection.println("Starting flight");
        flightMode = 1;
        message = "";
    }
    else if(message.equals("end\n")){
        RadioConnection.println("ENDING");
        flightMode = 0;
        idleAll();
        message = "";
    }

    getBatteryValues();
    currentAttitude = getAttitude();
    if(RC_Throttle < 1000){RC_Throttle = 1000;}
    throttle = RC_Throttle;
    if(flightMode == 1){
        pitchPID(currentAttitude.pitch);
        rollPID(currentAttitude.roll);
        yawPID(getHeading());
        throttleAdjust();
    }
    Serial.println("Loop");
    currentTime = millis();
    if(currentTime - lastMessageTime > 1250) {
        char data[40];
        sprintf(data, "%i,%f,%f,%i,%i,%f,%f", throttle, currentAttitude.pitch, currentAttitude.roll, desiredHeading,
                getHeading(), voltage, amps);
        RadioConnection.println(data);
        Serial.println(data);
        lastMessageTime = currentTime;
        Serial.println("Msg sent");
    }





    /*

    if (flightMode == 1){                                                       //The motors are started.
        if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.




        //read battery voltage 12.40v as 1240
        if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
            esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
            esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
            esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
            esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
            esc_5 += esc_5 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-5 pulse for voltage drop.
            esc_6 += esc_6 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-6 pulse for voltage drop.
        }

        if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
        if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
        if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
        if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.
        if (esc_5 < 1100) esc_4 = 1100;                                         //Keep the motors running.
        if (esc_6 < 1100) esc_4 = 1100;                                         //Keep the motors running.

        if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
        if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
        if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
        if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.
        if(esc_5 > 2000)esc_5 = 2000;                                           //Limit the esc-4 pulse to 2000us.
        if(esc_6 > 2000)esc_6 = 2000;                                           //Limit the esc-4 pulse to 2000us.
    }

    else{
        esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
        esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
        esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
        esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
        esc_5 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-5.
        esc_6 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-6.
    }
     */
    //char data [40];
    //sprintf(data,"%i,%d,%d,%d,%d,%d,%f",esc_1, esc_2, esc_3,esc_4,esc_5, esc_6, voltage);
    //Serial.println(data);
}
//pitch down PID value is positive
void pitchPID(double currentPitch){
    pitchCurrentTime = millis();
    //instantaneous error
    pitchPorportional = (desiredPitch - currentPitch) * pGainPitch;
    //Integral
    pitchIntegral += (pitchPorportional*(pitchCurrentTime - pitchOldTime)) * iGainPitch;
    //derivative
    pitchDerivitive = ((pitchPorportional - pitchLastError) * (pitchCurrentTime - pitchOldTime)) * dGainPitch;
    pitchLastError = pitchPorportional;
    pitchOldTime = pitchCurrentTime;
    pitchAdjust = pitchPorportional + pitchIntegral + pitchDerivitive;
}
//rolling right is negative
void rollPID(double currentRoll){
    rollCurrentTime = millis();
    //instantaneous error
    rollPorportional = (desiredRoll - currentRoll) * pGainRoll;
    //Integral
    rollIntegral += rollPorportional *(rollCurrentTime - rollOldTime) * iGainRoll;
    rollDerivitive = ((rollPorportional - rollLastError) * (rollCurrentTime - rollOldTime)) * dGainRoll;
    rollLastError = rollPorportional;
    rollOldTime = rollCurrentTime;
    rollAdjust = rollPorportional + rollIntegral + rollDerivitive;
}
void yawPID(double currentYaw){
    yawCurrentTime = millis();
    //instantaneous error
    yawPorportional = ((int)desiredYaw - (int)currentYaw) * pGainYaw;
    //Integral
    yawIntegral += yawPorportional *(yawCurrentTime - yawOldTime) * iGainYaw;
    yawDerivitive = ((yawPorportional - yawLastError) * (yawCurrentTime - yawOldTime)) * dGainYaw;
    yawLastError = yawPorportional;
    yawOldTime = yawCurrentTime;
    yawAdjust = yawPorportional + yawIntegral + yawDerivitive;
}
///Calculate the motor speeds
/* Decide whether to use timing or not
 * Pitch, roll and yaw provide some value
 * value range is between +- 500(?), if value is over set it to be 500
 * combine the pitch/roll/yaw values with the throttle setting
 *
 * Possibly move the altimeter to the second I2C channel due to interference, or just redo the wiring and see if it improves
 */
///                     1          2
///                      \        /
///                   3---        ---4
///                      /        \
///                     5          6
void throttleAdjust(){
    //pitched forward, positive adjust value
    //rolled left, positive adjust value
    //yaw left, positive adjust value. Increase 1,4,5 to yaw
    esc_1 = throttle + pitchAdjust + rollAdjust - yawAdjust;
    esc_2 = throttle + pitchAdjust - rollAdjust + yawAdjust;
    esc_3 = throttle + rollAdjust + yawAdjust;
    esc_4 = throttle - rollAdjust - yawAdjust;
    esc_5 = throttle - pitchAdjust + rollAdjust - yawAdjust;
    esc_6 = throttle - pitchAdjust - rollAdjust + yawAdjust;

    if(esc_1 > 2000) esc_1 = 2000;
    if(esc_2 > 2000) esc_2 = 2000;
    if(esc_3 > 2000) esc_3 = 2000;
    if(esc_4 > 2000) esc_4 = 2000;
    if(esc_5 > 2000) esc_5 = 2000;
    if(esc_6 > 2000) esc_6 = 2000;

    motor_1.writeMicroseconds(esc_1);
    motor_2.writeMicroseconds(esc_2);
    motor_3.writeMicroseconds(esc_3);
    motor_4.writeMicroseconds(esc_4);
    motor_5.writeMicroseconds(esc_5);
    motor_6.writeMicroseconds(esc_6);
}

    /*
    int loopStart = millis();
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
    else if(message.indexOf("setHeading") > -1){
        desiredHeading = message.substring(message.indexOf("setHeading")+10).toInt();
        RadioConnection.write("New Heading set to: ");
        RadioConnection.println(desiredHeading);
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
    else if(message.equals("reset\n")) { resetFunc();}

    //add 5 readings to the moving buffer to get the median
    for(int i = 0; i < 2; i++){
        currentAttitude = getAttitude();
        pitchFilter.push(round(currentAttitude.pitch));
        rollFilter.push(round(currentAttitude.roll));
        Serial.println(round(currentAttitude.pitch));
        Serial.println(round(currentAttitude.roll));
        Serial.println("");
        delay(5);
    }
    int pitchVal = pitchFilter.mode();
    int rollVal = rollFilter.mode();
    Serial.print(pitchVal);
    Serial.print(", ");
    Serial.println(rollVal);
    ///ping the ultrasonic sensor for 5 readings to refresh the average buffer
    getUltrasonic();
    getUltrasonic();
    getUltrasonic();
    getUltrasonic();
    getUltrasonic();

    double heading = getHeading();
    delay(10);
    heading = heading + getHeading();
    delay(10);
    heading = (heading + getHeading())/3;
    //offset heading , lets say we want 150.
    //151 to 330 is positive, so we need to rotate left
    //149 to 331 is negative, so we need to rotate right
    // current heading - desired
    // 100 - 150 = - 50, so rotate right
    // 300 - 150 = + 150 so rotate left
    // 150 + 180 = 330

    getBatteryValues();
    setThrottle();
    if(flightMode == 1){
        pitch(pitchVal,oldAttitude.pitch);
        roll(rollVal,oldAttitude.roll);
        yaw(heading);
        setSpeed();
        oldAttitude.pitch = pitchVal;
        oldAttitude.roll = rollVal;
        oldAttitude.yaw = heading;
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

    currentTime = millis();
    if(currentTime - lastMessageTime > 1250){
        char data [40];
        sprintf(data,"%i,%ld,%ld,%d,%f,%f,%f",throttle, pitchVal, rollVal,desiredHeading,heading, voltage, amps);
        RadioConnection.println(data);
        sprintf(data,"%i,%i,%i",pitchAdjust,rollAdjust,yawAdjust);
        RadioConnection.print(data);
        lastMessageTime = currentTime;
        //rotates to the right, so right spinning (CW) motors are spinning faster than the CCW motors
    }
    Serial.println(millis() - loopStart);
    Serial.println("");
    */



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
        ///     Work in pairs of motors, 1+4+5(cw) and 2+3+6(ccw), decrease one set and increase one set to induce torque on the body



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
    if(!bmp.begin(0x77)){
        delay(2000);
        if (!bmp.begin(0x76)) {
            RadioConnection.println(F("Could not find a valid BMP280 sensor, check wiring or "
                                      "try a different address!\n\n"));
            delay(5000);
            altimeterInit();
        }

    }

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
        mpu.reset();
        delay(1000);
        startGyro();
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    delay(5000);
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
    x= (RAD_TO_DEG * (atan2(-yAng, -zAng)+PI));
    y= (RAD_TO_DEG * (atan2(-xAng, -zAng)+PI));
    z= (RAD_TO_DEG * (atan2(-yAng, -xAng)+PI));
    //x is roll
    //y is pitch
    //z is yaw

    x = x - rollOffset;
    y = y - pitchOffset;
    z = z - yawOffset;
    if(x<-180){ x = 360 + x;}
    if(y<-180){ y = 360 + y;}
    currentAttitude.pitch = y;
    currentAttitude.roll = x;
    currentAttitude.yaw = z;
    return currentAttitude;
}

void startCompass(){
    int initialHeading = getHeading();
    while(initialHeading == NULL){
        Serial.println("No HMC5883 detected ... Check your wiring!");
        delay(2500);
    }
    RadioConnection.println("Compass Started");
}

int getHeading(){
    sensors_event_t event;
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = magDeclanation;
    heading += declinationAngle;
    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2*PI;
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
        heading -= 2*PI;
    // Convert radians to degrees for readability.
    return(heading * 180/M_PI);
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
    if(voltage > 12.94){voltage = 12.94;}
    voltage = map(voltage,11.8,12.94,1150,1250);
    //11.8 is 11.5
    //13.1 is 12.6
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

void rising() {
    attachInterrupt(digitalPinToInterrupt(12), falling, FALLING);
    prev_time = micros();
}
void falling() {
    attachInterrupt(digitalPinToInterrupt(12), rising, RISING);
    RC_Throttle = micros()-prev_time - 5;
}
/*
///ATTITUDE CONTROL
void pitch(float pitchVal, float oldPitch){
    if(pitchVal == desiredPitch){
        ///set the pitch offset to 0
        pitchAdjust = 0;
    }
    else{
        if(magnitude(pitchVal) < magnitude(oldPitch)){
            ///lower magnitude means we are actively correcting, thus dont change the inputs
        }
        else {
            ///we have a higher or equal mag, thus not correcting
            if (pitchVal > 0) { pitchAdjust = pitchAdjust + pitchIncrement;}
            else if (pitchVal < 0) {pitchAdjust = pitchAdjust - pitchIncrement;}
        }
    }
    ///pitch is positive, we are nose high, thus decrease front and increase rear
    int newSpeed = throttle - pitchAdjust + pitchTrim;
    T1 = T1 + newSpeed;
    T2 = T2 + newSpeed;
    newSpeed = throttle + pitchAdjust + pitchTrim;
    T5 = T5 + newSpeed;
    T6 = T6 + newSpeed;

    T3 = T3 + throttle;
    T4 = T4 + throttle;
}

void roll(float rollValue, float oldRoll){
    if(rollValue == desiredRoll){ rollAdjust = 0;}
    else{
        if(magnitude(rollValue) < magnitude(oldRoll)){
            ///lower magnitude means we are actively correcting, thus dont change the inputs
        }
        else{
            ///we have a higher or equal mag, thus not correcting
            if(rollValue > 0){ rollAdjust = rollAdjust + rollIncrement;}
            else if(rollValue < 0){ rollAdjust = rollAdjust - rollIncrement;}
        }
    }
    ///roll is positive, we are rolling right, decrease left and increase right
    int newSpeed = throttle - rollAdjust + rollTrim;
    //left motors
    T1 = T1 + newSpeed;
    T3 = T3 + newSpeed;
    T5 = T5 + newSpeed;
    newSpeed = throttle + rollAdjust + rollTrim;
    //right motors
    T2 = T2 + newSpeed;
    T4 = T4 + newSpeed;
    T6 = T6 + newSpeed;
    ///rolling right is positive
    ///left motors are 1,3,5. Right are 2,4,6
    ///if pitched right, we add to right motors, subtract from left

}

void yaw(int currentHeading){
    int flag = 2;
    if(currentHeading == desiredHeading){yawAdjust = 0;}
    else{
        ///work out magnitude
        if(magnitude(currentHeading - desiredHeading) < magnitude(oldHeading - desiredHeading)){
            ///we are correcting, thus do nothing
        }
        else {
            ///work out angles based on the desired heading
            int value = desiredHeading - 180;
            ///if value > 0 but less than 360 -> if the current is > value then we need to turn to the left
            ///     if value > 0 and > 360, subtract 360 from value. if current < (value-360) or > desired turn left
            ///if value < 0, subtract from 360. if current > (360-value) or value < desired turn right
            if (value < 0) {
                ///desired between 0 to 180
                value = 360 + value;
                if (currentHeading > value | currentHeading < desiredHeading) {
                    ///turn left
                    flag = 1;
                } else {
                    ///turn right
                    flag = -1;
                }
            } else if (value >= 0) {
                ///desired between 180 to 360
                if (currentHeading < desiredHeading & currentHeading > value) {
                    ///turn left
                    flag = 1;
                } else {
                    ///turn right
                    flag = -1;
                }
            }
            ///+1 is left, -1 is right
            ///1+4+5(cw) and 2+3+6(ccw)
            if (flag > 0) {
                ///left
                yawAdjust = yawAdjust - yawIncrement;
            } else if (flag < 0) {
                ///Right
                yawAdjust = yawAdjust + yawIncrement;
            }
        }
    }
    ///we need to turn right, so throttle is -2
    int newSpeed = throttle + yawAdjust + yawTrim;
    //left torque motors
    T2 = T2 + newSpeed;
    T3 = T3 + newSpeed;
    T6 = T6 + newSpeed;
    newSpeed = throttle - yawAdjust + yawTrim;
    //right torque motors
    T1 = T1 + newSpeed;
    T4 = T4 + newSpeed;
    T5 = T5 + newSpeed;
    oldHeading = currentHeading;
}

void setThrottle(){
    /*
    ///calculate height delta
    int currentHeight = heightFilter.GetFiltered();
    if(currentHeight - oldHeight > 0){
        ///we are currently accenting
        if(currentHeight < desiredHeight){
            ///we are correcting towards desired, thus do nothing
        }
        else{
            ///we are not correcting, thus increase throttle
            throttle = throttle + 5;
        }
    }
    if(currentHeight - oldHeight == 0){
        ///we are stable in the air
        if(currentHeight < desiredHeight){
            throttle = throttle + 5;
        }
        if(currentHeight > desiredHeight){
            throttle = throttle - 5;
        }
    }
    if (currentHeight - oldHeight < 0){
        ///we are currently descending
        if(currentHeight > desiredHeight){
            ///we are correcting towards desired height, thus do nothing
        }
        else{
            ///We are not correcting, thus increase throttle
            throttle = throttle + 5;
        }
    }

    if(RC_Throttle < 1000){RC_Throttle = 1000;}
    throttle = RC_Throttle;
    //Serial.println(throttle);
    ///Manage the min and max possible settings for armed throttle
    if(throttle > throttleLim){
        throttle = throttleLim;
    }
    /*
    if(throttle < 1050){
        throttle = 1050;
    }
    oldHeight = currentHeight;

}

void rising() {
    attachInterrupt(digitalPinToInterrupt(12), falling, FALLING);
    prev_time = micros();
}
void falling() {
    attachInterrupt(digitalPinToInterrupt(12), rising, RISING);
    RC_Throttle = micros()-prev_time - 5;
}

void setSpeed(){

    T1 = T1/3;
    T2 = T2/3;
    T3 = T3/3;
    T4 = T4/3;
    T5 = T5/3;
    T6 = T6/3;
    /*
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

    if(throttle > 1000) {
        motor_1.writeMicroseconds(T1);
        motor_2.writeMicroseconds(T2);
        motor_3.writeMicroseconds(T3);
        motor_4.writeMicroseconds(T4);
        motor_5.writeMicroseconds(T5);
        motor_6.writeMicroseconds(T6);
    }
    else{
        motor_1.writeMicroseconds(1000);
        motor_2.writeMicroseconds(1000);
        motor_3.writeMicroseconds(1000);
        motor_4.writeMicroseconds(1000);
        motor_5.writeMicroseconds(1000);
        motor_6.writeMicroseconds(1000);
    }

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
 */
///LIGHTS
///Nav lights do not blink, red and green
///Anti-collision do blink, red or white


///TODO: CHECK MOTOR SETTINGS
    ///currently running 3 seconds on then dropping 1 second off