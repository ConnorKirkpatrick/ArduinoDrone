//combine heading and attitude data to derive speed and velocity in a given direction
//typically this is north-east-up, so we shall do the same

#include "gyro.h"
float northV;
float eastV;
float verticalV;
void updateAHRS(attitude attitude, float heading,int timeDelta){
  //first we deal with the longitudinal component
  northV += cos(heading) * cos(attitude.pitch) * attitude.AccX *timeDelta;
  northV += sin(heading) * cos(attitude.roll) * attitude.AccY *timeDelta;
  //then generate the lateral component
  eastV += sin(heading) * cos(attitude.pitch) * attitude.AccX *timeDelta;
  eastV += cos(heading) * cos(attitude.roll) * attitude.AccY *timeDelta;
  //now the vertical components
  verticalV = sin(attitude.pitch) * attitude.AccX * timeDelta;
  verticalV = sin(attitude.roll) * attitude.AccY * timeDelta;
  //these three calculations generate the total longitudinal, lateral and vertical motion of the drone
}