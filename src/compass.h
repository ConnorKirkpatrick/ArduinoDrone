#include "Adafruit_HMC5883_U.h"
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
double offsetX=0, offsetY=0, offsetZ=0;
double mX,mY,mZ;
void startCompass(){
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("No Compass detected ... Check your wiring!");
    sendRadio("No Compass detected ... Check your wiring & detach power to retry");
    while(1);
  }
  Serial.println("Compass Ready");
  sendRadio("Compass Ready");
}

void calibrateCompass(){
  /* Get a new sensor event */
  double maxX, maxY, maxZ;
  double minX, minY, minZ;
  int time = millis();
  Serial.println("Calibrate the compass horizontally");
  while(millis() < time + 7000){
    //7 seconds to calibrate the horizontal
    sensors_event_t event;
    mag.getEvent(&event);
    maxX = event.magnetic.x > maxX ? event.magnetic.x : maxX;
    minX = event.magnetic.x < minX ? event.magnetic.x : minX;

    maxY = event.magnetic.y > maxY ? event.magnetic.y : maxY;
    minY = event.magnetic.y < minY ? event.magnetic.y : minY;

    maxZ = event.magnetic.z > maxZ ? event.magnetic.z : maxZ;
    minZ = event.magnetic.z < minZ ? event.magnetic.z : minZ;
  }
  Serial.println("Calibrate the compass Vertically");
  time = millis();
  while(millis() < time + 7000){
    //7 seconds to calibrate the horizontal
    sensors_event_t event;
    mag.getEvent(&event);
    maxX = event.magnetic.x > maxX ? event.magnetic.x : maxX;
    minX = event.magnetic.x < minX ? event.magnetic.x : minX;

    maxY = event.magnetic.y > maxY ? event.magnetic.y : maxY;
    minY = event.magnetic.y < minY ? event.magnetic.y : minY;

    maxZ = event.magnetic.z > maxZ ? event.magnetic.z : maxZ;
    minZ = event.magnetic.z < minZ ? event.magnetic.z : minZ;
  }
  offsetX = maxX + minX;
  offsetY = maxY + minY;
  offsetZ = maxZ + minZ;
  Serial.print("X: ");Serial.print(minX);Serial.print(" - ");Serial.print(maxX);Serial.print(" Offset: ");Serial.println(offsetX);
  Serial.print("Y: ");Serial.print(minY);Serial.print(" - ");Serial.print(maxY);Serial.print(" Offset: ");Serial.println(offsetY);
  Serial.print("Z: ");Serial.print(minZ);Serial.print(" - ");Serial.print(maxZ);Serial.print(" Offset: ");Serial.println(offsetZ);

}

float getHeading(attitude currentAttitude){
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
  mX = event.magnetic.x-offsetX;
  mY = event.magnetic.y-offsetY;
  mZ = event.magnetic.z-offsetZ;
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(mX); Serial.print("  ");
  Serial.print("Y: "); Serial.print(mY); Serial.print("  ");
  Serial.print("Z: "); Serial.print(mZ); Serial.print("  ");Serial.println("uT");


  //correct for tilt before calculating heading
  //exact use of roll and pitch depends on the device orientation on the drone
  float pitch = DegreestoRads(currentAttitude.pitch);
  float roll = DegreestoRads(currentAttitude.roll);
  float yHorizontal = mX * cos(pitch) + mY*sin(roll)*sin(pitch) - mZ * cos(roll)*sin(pitch);
  float xHorizontal = mY * cos(roll) + mZ*sin(roll);

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(yHorizontal, xHorizontal);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = -0.003839;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  return headingDegrees;
}