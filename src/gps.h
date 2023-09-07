//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600
#include "TinyGPS++.h"

TinyGPSPlus GPS;

int satsInView = 0;
TinyGPSCustom VDOP(GPS,"GPGSA", 18);
TinyGPSCustom EAlt(GPS, "GPGGA", 12);
TinyGPSCustom SV(GPS, "GPGSV", 3);

void getGPSData(){
    unsigned char x;
    String line = "";
    if(Serial1.available()){
        while(Serial1.available())//While there are characters to come from the GPS
        {
            GPS.encode(Serial1.read());//This feeds the serial NMEA data into the library one char at a time
        }
    }
}


void startGPS() {
    sendStatusMessage("GPS Starting....", SEVERITY_INFO);
    getGPSData();
    while(GPS.location.lng() == 0.00f ) {
        getGPSData();
        int fixType, sats;
        if((int) GPS.satellites.value() > atoi(SV.value())){
            sats = (int) GPS.satellites.value();
        }
        else{
            sats = atoi(SV.value());
        }
        if(sats < 3){
            fixType =1;
        }
        else if(sats < 4){
            fixType = 2; //2D fix
        }
        else{
            fixType = 3; //3D fix
        }
        sendGPSRaw(fixType,GPS.location.lat(), GPS.location.lng(), GPS.altitude.meters(), GPS.hdop.value(), atoi(VDOP.value()), GPS.speed.mps(), GPS.course.deg(), sats, atoi(EAlt.value()));
        for(int i = 0; i < 5; i++){ //delay 5s but still read GPS updates
            delay(1000);
            getGPSData();
        }

    }
    sendGPSRaw(2,GPS.location.lat(), GPS.location.lng(), GPS.altitude.meters(), GPS.hdop.value(), atoi(VDOP.value()), GPS.speed.mps(), GPS.course.deg(), GPS.satellites.value(), atoi(EAlt.value()));
    sendStatusMessage("GPS Ready", SEVERITY_INFO);
}



  //if there is no satellites, no valid fix can be made thus retry
  /*
  while(coords[0] < 1) {
    delay(2000);
    getCoords(coords);
    String data = "Found satellites: ";
    data += coords[0];
    //sendRadio(data);
  }
   */
  //sendRadio("GPS Ready");