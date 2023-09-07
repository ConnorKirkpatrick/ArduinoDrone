#include "Arduino.h"
#include "MadgwickAHRS.h"

/// file includes
#include "Radio.h"
#include "gps.h"
#include "gyro.h"
#include "kalmanFilters.h"
#include "compass.h"
#include "barometer.h"
#include "AHRS.h"

uint32_t timer;
int lastMessage = 0;
double state[9];

Madgwick filter;
unsigned long microsPerReading, microsPrevious, microsNow;
float pitch, roll, yaw, heading;



void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();
    startRadio();
    delay(1000);// Wait for sensor to stabilize
    //startBMP();
    startCompass();
    startGyro();
    startGPS();
    getGyroData();
    setupFilters(currentAttitude);
    timer = micros();
    sendStatusMessage("System components started successfully",SEVERITY_INFO);

    double init_data[9];
    init_data[0] = GPS.location.lat();
    init_data[1] = GPS.location.lng();
    init_data[2] = GPS.altitude.meters();
    init_data[3] = 0;
    init_data[4] = 0;
    init_data[5] = 0;
    init_data[6] = 0;
    init_data[7] = 0;
    init_data[8] = 0;
    start(init_data);
    sendStatusMessage("System started successfully",SEVERITY_INFO);
    sendGlobalPosition(X[0],X[1],X[2],X[2],X[3],X[4],X[5],headingDegrees);
    sendAttitude(currentAttitude.roll,currentAttitude.pitch, currentAttitude.yaw, currentAttitude.RAccY, currentAttitude.RAccX, currentAttitude.RAccZ);
    lastMessage = millis();
}

void loop() {

    ///Update all the values
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    currentAttitude = getGyroData();
    kalmanUpdate(currentAttitude, heading, dt); //pass data to kalman filters to get accurate pitch+roll data

    getGPSData();
    updateAHRS(currentAttitude, heading, deltaT);
    double data[9];
    if(GPS.location.isUpdated()){
        data[0] = GPS.location.lat();
        data[1] = GPS.location.lng();
        data[2] = GPS.altitude.meters();
        data[3] = northV;
        data[4] = eastV;
        data[5] = upV;
        data[6] = northA;
        data[7] = eastA;
        data[8] = upA;
        predict(millis());
        update(data);
        //move to using kalman values for accelerations to combat upV constant increase
    }
    else{
        data[0] = X[0];
        data[1] = X[1];
        data[2] = X[2];
        data[3] = northV;
        data[4] = eastV;
        data[5] = upV;
        data[6] = northA;
        data[7] = eastA;
        data[8] = upA;
        predict(millis());
        update(data);
    }

    if(millis() - lastMessage > 250){
        //send status messages
        sendGlobalPosition(X[0],X[1],X[2],X[2],X[3],X[4],X[5],headingDegrees);
        sendAttitude(kalAngleY,kalAngleX, currentAttitude.yaw, kalAccY, kalAccX, kalAccZ);
        int fixType, sats;
        sats = (int) SV.value();

        if(sats < 3){
            fixType = 1;
        }
        else if(sats < 4){
            fixType = 2; //2D fix
        }
        else{
            fixType = 3; //3D fix
        }
        if(GPS.location.age() < 300){
            sendStatusMessage(String(SV.isValid()),SEVERITY_NOTICE);
            sendGPSRaw(fixType,GPS.location.lat(), GPS.location.lng(), GPS.altitude.meters(), GPS.hdop.value(), atoi(VDOP.value()), GPS.speed.mps(), GPS.course.deg(), sats, atoi(EAlt.value()));
        }
        lastMessage = millis();
    }

    /*
    currentAttitude = getGyroData(); //grab the data from the IMU
    kalmanUpdate(currentAttitude, dt); //pass data to kalman filters to get accurate pitch+roll data
    Serial.print("Pitch:");
    Serial.print(currentAttitude.RAccX);
    Serial.print(",");

    Serial.print("Max:");
    Serial.print(2*PI);
    Serial.print(",");

    Serial.print("Min:");
    Serial.println(-2*PI);
    /*
    Serial.print(",");

    Serial.print("Roll:");
    Serial.println(kalAngleY);
     */


}
// 51.3443708,-0.6433905,134.11,-129.55,224.68,0.00

// Send statustext messages about systems, then GPS_INT for attitude data