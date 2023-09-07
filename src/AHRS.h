//combine heading and attitude data to derive speed and velocity in a given direction
//typically this is north-east-up, so we shall do the same

float northV, eastV, upV, altitude, northA, eastA, upA;
int lastPredict = 0;
int deltaT = 0;
int r_earth = 6378100;

double oldLatitude, oldLongitude;

//predict via gyro
//update via GPS



void updateAHRS(attitude attitude, float heading,int timeDelta){
    //first we deal with the longitudinal component
    northV += cos(heading) * cos(attitude.pitch) * attitude.AccX *timeDelta;
    northV += sin(heading) * cos(attitude.roll) * attitude.AccY *timeDelta;
    northV += sin(attitude.pitch) * cos(heading) * attitude.AccZ * -1 * timeDelta;

    //then generate the lateral component
    eastV += sin(heading) * cos(attitude.pitch) * attitude.AccX *timeDelta;
    eastV += cos(heading) * cos(attitude.roll) * attitude.AccY *timeDelta;
    eastV += sin(attitude.pitch) * sin(heading) * attitude.AccZ * -1 * timeDelta;
    //now the vertical components
    upV += sin(attitude.pitch) * attitude.AccX * timeDelta;
    upV += sin(attitude.roll) * attitude.AccY * timeDelta;
    upV += cos(attitude.pitch) * attitude.AccZ * timeDelta;
    //these three calculations generate the total longitudinal, lateral and vertical motion of the drone

    northA = cos(heading) * cos(attitude.pitch) * attitude.AccX;
    northA += sin(heading) * cos(attitude.roll) * attitude.AccY * -1;
    northA += sin(heading) * sin(attitude.roll) * attitude.AccZ;

    eastA = sin(heading) * cos(attitude.pitch) * attitude.AccX;
    eastA += cos(heading) * cos(attitude.roll) * attitude.AccY;
    eastA += sin(attitude.roll) * cos(heading) * attitude.AccZ;

    upA = cos(heading) * sin(attitude.pitch) * attitude.AccZ;
    upA += sin(attitude.roll) * cos(attitude.pitch) * attitude.AccY;
    upA += cos(attitude.roll) * cos(attitude.pitch) * attitude.AccZ;
}



double X [9] = { //STATE
        0, //lat
        0, //long
        0, //alt

        0, //northV
        0, //eastV
        0, //upV

        0, //northAcc
        0, //eastAcc
        0, //downAcc
};


double F[9][9] = { //STATE TRANSITIONS
        {1, 0, 0, X[0]  + (X[3] * deltaT / r_earth) * (180 / PI), 0, 0, 0, 0, 0}, //north position
        {0,1,0,0,X[1] + (X[4] * deltaT / r_earth) * (180 / PI) / cos(X[0] * PI/180),0,0,0,0}, //east position
        {0,0,1,0,0,X[2] + (X[5] * deltaT),0,0,0}, //altitude m
        {0,0,0,1,0,0,X[3] * deltaT,0,0}, //north velocity m/s
        {0,0,0,0,1,0,0,X[4] * deltaT,0}, //east velocity m/s
        {0,0,0,0,0,1,0,0,X[5] * deltaT},//vertical velocity m/s
        {0,0,0,0,0,0,1,0,0}, // north acceleration m/s/s
        {0,0,0,0,0,0,0,1,0}, // east acceleration m/s/s
        {0,0,0,0,0,0,0,0,1} // vertical acceleration m/s/s
};
        //future state Xn+1 = FXn + GUn
double U[3] = {
        0,
        0,
        0,
};
double P[9][9]; //covariance matrix
double Q[9] = { //system noise
        0,
        0,
        0.002,
        0.001,
        0.001,
        0.001,
        0.0005,
        0.0005,
        0.0005
};
double W[9] = {0.1,0.1,0.001,0.001,0.001,0.001,0.001,0.001,0.001}; //process noise
double H[9] = {1,1,1,1,1,1,1,1,1}; // observation matrix
double K[9] = {1,1,1,1,1,1,1,1,1}; //kalman gain
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

void predict(int time){
    deltaT = (time/1000) - lastPredict; // convert milliseconds to seconds
    //for lat
    X[0] = X[0] + (X[3] * deltaT / r_earth) * (180 / PI); // transition
    P[0][0] += + Q[0] ; //covariance prediction
    //for long
    X[1] = X[1] + (X[4] * deltaT / r_earth) * (180 / PI) / cos(X[0] * PI/180);// transition
    P[1][1] += + Q[1]; // covariance prediction
    //for alt
    X[2] = X[2] + X[5] * deltaT; //transition
    P[2][2] += + Q[2];// covariance prediction
    //for xV
    X[3] = X[3] + X[6] * deltaT; // transition
    P[3][3] += + Q[3];// covariance prediction
    //for xY
    X[4] = X[4] + X[7] * deltaT;
    P[4][4] += Q[4];
    //for xZ
    X[5] = X[5] + X[8] * deltaT;
    P[5][5] += Q[5];
    //For Ax
    //X[6] = X[6];
    P[6][6] += Q[6];
    //for Ay
    P[7][7] += Q[7];
    //for Az
    P[8][8] += Q[8];


    lastPredict = time/1000;
}

double sum(double *input){
    double sum = 0;
    for(int i = 0; i < 9; i++){
        sum += input[i];
    }
    return sum;
}

void update(double *update){
    //update gain
    for(int i =0;i<9;i++){
        K[i] = sum(P[i]) / sum(P[i]) + W[i];
    }
    //update the values
    for(int j =0;j<9;j++){
        X[j] = X[j] + K[j] * (update[j] - H[j]*X[j]);
    }
    //update cov
    for(int i=0;i<9;i++){
        for(int j=0;j<9;j++){
            P[i][j] = (I[i][j]-K[i]*H[i]) * P[i][j] * (I[j][i]-K[j]*H[j]) + K[i] * W[i] *K[j];
        }
    }
}

void start(double* data){
    for(int i = 0; i < 9; i++){
        X[i] = data[i];
    }
    lastPredict = millis()/1000;
    predict(millis());
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