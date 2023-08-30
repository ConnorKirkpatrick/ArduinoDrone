//combine heading and attitude data to derive speed and velocity in a given direction
//typically this is north-east-up, so we shall do the same

#include "gyro.h"
float northV, eastV, downV, altitude;
int deltaT = 0;
int r_earth = 6378100;

double oldLatitude, oldLongitude;

//predict via gyro
//update via GPS



void updateAHRS(attitude attitude, float heading,int timeDelta){
  //first we deal with the longitudinal component
  northV += cos(heading) * cos(attitude.pitch) * attitude.AccX *timeDelta;
  northV += sin(heading) * cos(attitude.roll) * attitude.AccY *timeDelta;
  //then generate the lateral component
  eastV += sin(heading) * cos(attitude.pitch) * attitude.AccX *timeDelta;
  eastV += cos(heading) * cos(attitude.roll) * attitude.AccY *timeDelta;
  //now the vertical components
  downV = sin(attitude.pitch) * attitude.AccX * timeDelta;
  downV = sin(attitude.roll) * attitude.AccY * timeDelta;
  //these three calculations generate the total longitudinal, lateral and vertical motion of the drone
}



double X [9] = { //STATE
        0, //lat
        0, //long
        0, //alt

        0, //northV
        0, //eastV
        0, //downV

        0, //northAcc
        0, //eastAcc
        0, //downAcc
};


double F[9][9] = { //STATE TRANSITIONS
        {1, 0, 0, X[0]  + (X[3] * deltaT / r_earth) * (180 / PI), 0, 0, 0.5* pow(deltaT,2), 0, 0}, //north position
        {0,1,0,0,X[1] + (X[4] * deltaT / r_earth) * (180 / PI) / cos(X[0] * PI/180),0,0,0.5* pow(deltaT,2),0}, //east position
        {0,0,1,0,0,X[2] + X[5] * deltaT,0,0,0.5* pow(deltaT,2)}, //altitude
        {0,0,0,1,0,0,X[3] * deltaT,0,0}, //north velocity
        {0,0,0,0,1,0,0,X[4] * deltaT,0}, //east velocity
        {0,0,0,0,0,1,0,0,X[5] * deltaT},//vertical velocity
        {0,0,0,0,0,0,1,0,0}, // north acceleration
        {0,0,0,0,0,0,0,1,0}, // east acceleration
        {0,0,0,0,0,0,0,0,1} // vertical acceleration
};
        //future state Xn+1 = FXn + GUn
double U[3] = {
        0,
        0,
        0,
};
double P[9][9]; //covariance matrix
double Q[9]; //system noise
double W[9]; //process noise
double H[9] = {1,1,1,1,1,1,1,1,1};
double K[9]; //kalman gain
double I[9][9] = { //this is a fixed identity matrix, it is an NxN grid with 1's down the diagonal
        {1,0,0,0,0,0,0,0,0,},
        {0,1,0,0,0,0,0,0,0},
        {0,0,1,0,0,0,0,0,0},
        {0,0,0,1,0,0,0,0,0},
        {0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,1,0,0,0},
        {0,0,0,0,0,0,1,0,0},
        {0,0,0,0,0,0,0,1,0},
        {0,0,0,0,0,0,0,0,1}

};

void predict(){
    for(int i = 0; i< 9; i++){//for each state
        double stateResult = 0;
        for(int j = 0; j < 9; j++){//for each transition in the state
            stateResult += X[i]*F[i][j]; //predict the new state based on the state transition
            P[i][j] = F[i][j]*P[i][j]*F[j][i] + Q[i]; //predict the new covariance, F*P*inverted F + fixed error
        }
        X[i] = stateResult;

    }
}

void update(double* update){
    //first we apply the innovation step
    for(int i = 0; i < 9; i++){
        for(int j = 0; j < 9; j++){
            //first we update the kalman gains
            K[i] = P[i][j] * H[j] / (H[i] * P[i][j] * H[j] + W[i]);
            //check second step for inversion
            //now update the covariance matrix
            P[i][j] = (I[i][j] - K[i] * H[i]) * P[i][j] * (I[j][i] - K[j] * H[j]) + (K[i] * W[i] * K[j]);
        }
        //finally we update the estimate with the new measurement, step 2 and three can be performed in any order as they do not affect each other
        X[i] = X[i] + K[i] * (update[i] - H[i]*X[i]); //for a given state, the new state is the old state + the kalman gain ( the measures state - the old state * the observation matrix (this matrix allows us to tune the bias towards the new value

    }
    //next we update the covariance matrix
}


/**
* AHRS for extended kalman filter
 * GPS: provides LAT, LONG, ALT, speed + direction of movement
 * IMU: Provides velocities
 * Compass: Provides heading
 *
 * fuse IMU and compass to get velocities North, east, vertical
 * fuse velocities with LAT+LONG to derive predicted location
*/

/**
* Extended kalman filter matrices
    * Current state matrix, X. This matrix contains the values of all current variables
    * State transition matrix, F. This matrix contains the equations for the state transition of each variable
    * Control variable, U. This matrix captures the input into the overall system, such as a control input
    * Process noise, W. This is an fixed matrix of expected noise in the system for each variable.
*/

/**
 * State transitions
    * Latitude: oldLatitude  + (northVelocity * deltaT / r_earth) * (180 / pi); where dy is the northV * deltaT
    * Longitude: oldLongitude + (eastVelocity * deltaT / r_earth) * (180 / pi) / cos(oldLatitude * pi/180);  where dX is the eastV * deltaT
    * Altitude: oldAlt + downVelocity * deltaT
    *
    * NorthVelocity: oldNorthSpeed + northVelocity * deltaT
    * EastVelocity: oldEastSpeed + eastVelocity * deltaT
    * DownVelocity: oldDownSpeed + downVelocity * deltaT
    *
    * NorthAcceleration: oldNorthAcc + northAcc * deltaT
    * EastAcceleration: oldEastAcc + eastAcc * deltaT
    * downAcceleration: oldDownAcc + eastAcc * deltaT
 */

/**
 * prediction step
 * state transition via f(x,u) where u is the rate of change
 * x+T*f(x,u) is the next predicted state
 *
 * State transition is taking the old location, taking the current velocity and delta in time, and computing a new location
    * new_latitude  =
    * new_longitude =
    * new_alt = oldAlt + dh * deltaT
    * where r_earth = 6378.1 km
 * */

/** EKF steps
    * 0.    Initial system state, first fix and 0 velocity
    * 1.    Predicted state estimate
    * 2.    Predicted error covariance estimate
    * 3.    measurement
    * 4.    innovation covariance
    * 5.    calculate kalman gain
    * 6.    update the state estimate
    * 7.    update error covariance
    * 8.    new measurement
*/