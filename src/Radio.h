/**
 * This radio object is an abstraction of the wireless radio linked to the raspberry pi companion
 * The aim of this object is to provide the methods to allow the arduino to send correctly formed mavlink packets to
 * the companion computer for broadcast
*/

enum{SEVERITY_EMER, SEVERITY_ALERT,SEVERITY_CRIT,SEVERITY_ERR,SEVERITY_WARN,SEVERITY_NOTICE,SEVERITY_INFO,SEVERITY_DEBUG};

void sendStatusMessage(String message, uint8_t severity);
int receive_(uint8_t* buf);
bool arrayEquality(uint8_t* array1, uint8_t* array2, int size);
uint8_t incomingData[24];
/**
 * To send a message, we send in this serialization order
 * Size, Message Type, Contents
 * This allows the receiving object to correctly read our data and vice versa
 */

void startRadio(){
    Serial.begin(115200);
    // Now send a message via serial and get confirmation
    byte initiateMsg[7];
    initiateMsg[0] = 6;
    initiateMsg[1] = 0;
    initiateMsg[2] = 'X';
    initiateMsg[3] = 'X';
    initiateMsg[4] = 'X';
    initiateMsg[5] = 'X';
    initiateMsg[6] = 'X';
    byte resp[6];
    resp[0] = 0;
    resp[1] = 'Y';
    resp[2] = 'Y';
    resp[3] = 'Y';
    resp[4] = 'Y';
    resp[5] = 'Y';
    Serial.write(initiateMsg,7);

    int size = receive_(incomingData);
    while(!arrayEquality(resp,incomingData,size)){
        delay(100);
        size = receive_(incomingData);
    }

    sendStatusMessage("Radio Ready", SEVERITY_INFO);

}

bool arrayEquality(uint8_t* array1, uint8_t* array2, int size){
    if(size < 1){return false;}
    for(int i = 0; i< size; i++){
        if(array1[i] != array2[i]){
            return false;
        }
    }
    return true;
}

void send_(byte msg[],uint8_t size){
    //messages should be in the form: size,ID,contents
    Serial.write(msg,size);
}

int receive_(uint8_t* buf){
    if(Serial.available()){
        int size = Serial.read();
        Serial.readBytes(buf,size);
        return size;
    }
    return 0;
}

void sendStatusMessage(String message, uint8_t severity){
    message = message.c_str();
    // len, type, contents
    byte msg[3+message.length()];
    msg[0] = 2+message.length();
    msg[1] = 1;
    msg[2] = severity;
    for(int i = 3; i < 3+message.length(); i++){
        msg[i] = message[i-3];
    }
    send_(msg,sizeof(msg));
}

void sendGlobalPosition(double lat, double lng, double alt, double rAlt, double vX, double vY, double vZ, double heading){
    // len, type, contents

    //uint32_t time  microseconds
    //int32_t Lat, degE7
    //int32_t Long, degE7
    //int32_t alt, mm
    //int32_t radar Alt, mm
    //int16_t vx, cm/s
    //int16_t vy, cm/s
    //int16_t vz, cm/s
    //uint16_t heading, degrees
    byte msg[28+2];
    msg[0] = 29;
    msg[1] = 2;
    uint32_t  time = millis();
    memcpy(&msg[2], (uint8_t *)&time,sizeof(uint32_t));
    int32_t lt = ceil(lat * 10000000);
    memcpy( &msg[6], (uint8_t *)&lt, sizeof(int32_t));
    int32_t ln = ceil(lng * 10000000);
    msg[10] = ln;
    msg[11] = ln >> 8;
    msg[12] = ln >> 16;
    msg[13] = ln >> 24;
    memcpy( &msg[10], (uint8_t *)&ln, sizeof(int32_t));
    int32_t al = ceil(alt*1000);
    memcpy( &msg[14], (uint8_t *)&al, sizeof(int32_t));
    int32_t ral = ceil(rAlt*1000);
    memcpy( &msg[18], (uint8_t *)&ral, sizeof(int32_t));
    int16_t VX = ceil(vX);
    memcpy( &msg[22], (uint8_t *)&VX, sizeof(int16_t));
    int16_t VY = ceil(vY);
    memcpy( &msg[24], (uint8_t *)&VY, sizeof(int16_t));
    int16_t VZ = ceil(vZ);
    memcpy( &msg[26], (uint8_t *)&VZ, sizeof(int16_t));
    int16_t hd = ceil(heading);
    memcpy( &msg[28], (uint8_t *)&hd, sizeof(int16_t));
    send_(msg,30);
}

void sendGPSRaw(uint8_t fixType, double lat, double lng, double alt, int hdop, int vdop, double vel, double course, int sats, double altEl){
    //uint64_t  time    US
    //uint8_t   fixType ENUM
    //uint32_t  lat     degE7
    //uint32_t  lng     degE7
    //uint32_t   alt     mm
    //uint16_t  HDOP                    in GPGSA
    //uint16_t  VDOP                    in GPGSA
    //uint16_t  vel     cm/s
    //uint16_t  course  deg*100
    //uint8_t   sats                    in GPGSV
    //int32_t   alt-ellips   mm         in GPGGA
    //uint32_t  position uncertainty    in GPGSA
    //uint32_t  altitude uncertainty    in GPGSA
    //uint32_t  velocity uncertainty
    //uint32_t  course uncertainty
    //uint16_t  yaw
    byte msg[52+2];
    msg[0] = 53;
    msg[1] = 3;
    uint64_t time = micros()*1000;
    memcpy(&msg[2], (uint8_t *)&time,sizeof(uint64_t));
    msg[10] = fixType;
    int32_t lt = ceil(lat*10000000);

    memcpy(&msg[11], (uint8_t *)&lt,sizeof(int32_t));
    int32_t ln = ceil(lng*10000000);
    memcpy(&msg[15], (uint8_t *)&ln,sizeof(int32_t));
    int32_t al = ceil(alt*1000);
    memcpy( &msg[19], (uint8_t *)&al, sizeof(int32_t));

    memcpy(&msg[23], (uint8_t *)&hdop, sizeof(int16_t));
    memcpy(&msg[25], (uint8_t *)&vdop, sizeof(int16_t));
    uint16_t velCm = ceil(vel*100);
    memcpy(&msg[27], (uint8_t *)&velCm, sizeof(uint16_t));
    uint16_t crs = ceil(course*100);
    memcpy(&msg[29], (uint8_t *)&crs, sizeof(int16_t));

    msg[31] = sats;

    int32_t eAlt = ceil(altEl * 1000);
    memcpy(&msg[32], (uint8_t *)&eAlt, sizeof(int32_t));

    uint32_t hu = 1000;
    memcpy(&msg[36], (uint8_t *)&hu, sizeof(uint32_t));
    uint32_t vu = 1000;
    memcpy(&msg[40], (uint8_t *)&vu, sizeof(uint32_t));
    uint32_t velu = 1000;
    memcpy(&msg[44], (uint8_t *)&velu, sizeof(uint32_t));
    uint32_t headu = 5000;
    memcpy(&msg[48], (uint8_t *)&headu, sizeof(uint32_t));
    int16_t yaw = 0;
    memcpy(&msg[52], (uint8_t *)&yaw, sizeof(int16_t));
    send_(msg,52+2);

}

void sendAttitude(float roll, float pitch, float yaw, float rollSpeed, float pitchSpeed, float yawSpeed){
    // uint32_t time, microseconds
    // float roll, radians
    // float pitch, radians
    // float yaw, radians
    // float rollSpeed, radians/seconds
    // float pitchSpeed, radians/seconds
    // float yawSpeed, radians/seconds
    byte msg[28+2];
    msg[0] = 29;
    msg[1] = 4;
    uint32_t time = millis();
    memcpy(&msg[2], (uint8_t *)&time,sizeof(uint32_t));

    memcpy( &msg[6], (uint8_t *)&roll, sizeof(float));

    memcpy( &msg[10], (uint8_t *)&pitch, sizeof(float));

    memcpy( &msg[14], (uint8_t *)&yaw, sizeof(float));

    memcpy( &msg[18], (uint8_t *)&rollSpeed, sizeof(float));

    memcpy( &msg[22], (uint8_t *)&pitchSpeed, sizeof(float));

    memcpy( &msg[26], (uint8_t *)&yawSpeed, sizeof(float));

    send_(msg,30);
}

