#include <Adafruit_MPU6050.h>
#include "Adafruit_Sensor.h"

Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;
int16_t AcX,AcY,AcZ; ///Raw angular measurement
double x,y,z; ///absolute angle

int minVal=265;
int maxVal=402;
int xAng,yAng,zAng;
int offsetCycles = 20;

struct attitude{
  double pitch=0,roll=0,yaw=0;
  double AccX=0, AccY=0, AccZ=0;///positional accelerations
  double RAccX=0, RAccY=0, RAccZ=0; ///rotational accelerations

} currentAttitude, calibrationAttitude, oldAttitude, filteredAttitude;
double pitchOffset=0,rollOffset=0,yawOffset=0;
double PAccXOffset=0,PAccYOffset=0,PAccZOffset=0;
double RAccXOffset=0,RAccYOffset=0,RAccZOffset=0;

attitude getGyroData();


double DegreestoRads(double x){
  return x * PI/180.0;
}


void startGyro() {
  double offsetP=0, offsetR=0, offsetY=0; ///raw angle offsets
  double offsetPX=0, offsetPY=0, offsetPZ=0; ///Positional acceleration offsets
  double offsetRX=0, offsetRY=0, offsetRZ=0; ///angular acceleration offsets
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    sendRadio("Failed to start gyro, detach power and try again");
    while(1){}
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);

  Serial.println("Calibrating Gyro...");
  sendRadio("Gyro calibrating...");
  delay(5000); //pause to allow values to stabilise
  for(int i = 0; i < offsetCycles; i++){
    calibrationAttitude = getGyroData();
    offsetP = offsetP + calibrationAttitude.pitch;
    offsetR = offsetR + calibrationAttitude.roll;
    offsetY = offsetY + calibrationAttitude.yaw;

    offsetPX = offsetPX + calibrationAttitude.AccX;
    offsetPY = offsetPY + calibrationAttitude.AccY;
    offsetPZ = offsetPZ + calibrationAttitude.AccZ;

    offsetRX = offsetRX + calibrationAttitude.RAccX;
    offsetRY = offsetRY + calibrationAttitude.RAccY;
    offsetRZ = offsetRZ + calibrationAttitude.RAccZ;

    delay(100);
  }

  pitchOffset = offsetP/offsetCycles;
  rollOffset = offsetR/offsetCycles;
  yawOffset = offsetY/offsetCycles;

  PAccXOffset = offsetPX/offsetCycles;
  PAccYOffset = offsetPY/offsetCycles;
  PAccZOffset = offsetPZ/offsetCycles;

  RAccXOffset = offsetRX/offsetCycles;
  RAccYOffset = offsetRY/offsetCycles;
  RAccZOffset = offsetRZ/offsetCycles;
  Serial.println("Gyro Ready");
  sendRadio("Gyro Ready");
}

attitude getGyroData() {
  ///Grab the Adafruit acceleration values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a,&g,&temp);

  ///grab the raw sensor values
  const int MPU_addr=0x68;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,12,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();

  currentAttitude.AccX = a.acceleration.x - PAccXOffset;
  currentAttitude.AccY = a.acceleration.y - PAccYOffset;
  currentAttitude.AccZ = a.acceleration.z - PAccZOffset;

  currentAttitude.RAccX = g.gyro.x - RAccXOffset;
  currentAttitude.RAccY = g.gyro.y - RAccYOffset;
  currentAttitude.RAccZ = g.gyro.z - RAccZOffset;


  xAng = map(AcX,minVal,maxVal,-90,90);
  yAng = map(AcY,minVal,maxVal,-90,90);
  zAng = map(AcZ,minVal,maxVal,-90,90);
  x= (atan2(-yAng, -zAng)+PI);
  y= (atan2(-xAng, -zAng)+PI);
  z= (atan2(-yAng, -xAng)+PI);
  ///x is roll
  ///y is pitch
  ///z is yaw

  y = y * -1; //*-1 as the sensor is inverted

  x = x - rollOffset;
  y = y - pitchOffset;
  z = z - yawOffset;
  ///Ensure that measurements are within the +- 1PI Radians range
  if(x<-PI){ x = 2*PI + x;}
  if(y<-PI){ y = 2*PI + y;}
  currentAttitude.pitch = y;
  currentAttitude.roll = x;
  currentAttitude.yaw = z;
  return currentAttitude;
}

