#include "Kalman.h"

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Kalman kalmanAX, kalmanAY, kalmanAZ;

double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double kalAccX, kalAccY, kalAccZ; //Acceleration using kalman filter

void setupFilters(attitude currentAttitude){
  kalmanX.setAngle(currentAttitude.pitch); // Set starting angle
  kalmanY.setAngle(currentAttitude.roll);

  kalmanAX.setAngle(currentAttitude.AccX);
  kalmanAY.setAngle(currentAttitude.AccZ);
  kalmanAZ.setAngle(currentAttitude.AccZ);
}

void kalmanUpdate(attitude currentAttitude, double dt){
  ///Kalman filter for pitch
  if ((currentAttitude.pitch < -90 && kalAngleX > 90) || (currentAttitude.pitch > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(currentAttitude.pitch);
    kalAngleX = currentAttitude.pitch;
  } else {
    kalAngleX = kalmanX.getAngle(
        currentAttitude.pitch, currentAttitude.RAccY, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90)
    currentAttitude.RAccY = -currentAttitude.RAccY; // Invert rate, so it fits the restricted accelerometer reading

  ///Kalman filter for Roll
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((currentAttitude.roll < -90 && kalAngleY > 90) || (currentAttitude.roll > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(currentAttitude.roll);
    kalAngleY = currentAttitude.roll;
  } else{
    kalAngleY = kalmanY.getAngle(currentAttitude.roll, currentAttitude.RAccX, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90)
    currentAttitude.RAccX = -currentAttitude.RAccX; // Invert rate, so it fits the restricted accelerometer reading

  ///Kalman filter for directional acceleration
  kalAccX = kalmanAX.getAngle(currentAttitude.AccX, currentAttitude.AccX,dt);

  ///Kalman filter for lateral acceleration
  kalAccX = kalmanAX.getAngle(currentAttitude.AccY, currentAttitude.AccY,dt);

  ///Kalman filter for Vertical acceleration
  kalAccX = kalmanAX.getAngle(currentAttitude.AccZ, currentAttitude.AccZ,dt);
}