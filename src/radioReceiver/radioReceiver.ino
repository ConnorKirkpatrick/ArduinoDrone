#include <Arduino.h>
#include "E220.h"
#include <SoftwareSerial.h>

//Define the pins we need to use later to create the object
#define m0 7
#define m1 6
#define aux 5

SoftwareSerial receiver(3,2);


void setup() {
  //start all of our UART connections
  Serial.begin(9600);
  receiver.begin(9600);


  Stream &mySerial = (Stream &)receiver;
  E220 radioModule(&mySerial, m0, m1, aux);

  //initialise the module and check it communicates with us, else loop and keep trying
  while(!radioModule.init()){
    delay(5000);
  }
  //setting a permanent variable for the module
  radioModule.setAddress(1234,true);
  radioModule.setChannel(0,true);
  radioModule.setPower(Power_21,false);

  //status format: 0,{address},{channel},{power}
  String data = "0,";
  data += radioModule.getAddress();
  data += ",";
  data += radioModule.getChannel();
  data += ",";
  data += radioModule.getPower();
  Serial.println(data)
}

void loop() {

}