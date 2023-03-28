#include "E220.h"
#define m0 8
#define m1 9
#define aux 22

Stream &RadioConnection = (Stream &)Serial2;

E220 radioModule(&RadioConnection, m0, m1, aux);

void sendRadio(String data){
  RadioConnection.println(data);
  RadioConnection.flush();
}

String recieveRadio(){
  if(RadioConnection.available()){
    while(RadioConnection.available()){
      String message = RadioConnection.readStringUntil('\n');
      return message;
    }
  }
  else{
    return "";
  }
}

void startRadio(){
  Serial2.begin(9600);
  while(!radioModule.init()){
    Serial.println("Waiting for radio");
    delay(5000);
  }
  radioModule.setAirDataRate(ADR_9600,0);
  sendRadio("Radio Ready");
}
