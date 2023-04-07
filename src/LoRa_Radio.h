#include "E220.h"
#define m0 8
#define m1 9
#define aux 22

uint8_t Pub[32];
uint8_t Pri[32];

Stream &RadioConnection = (Stream &)Serial2;
Serial_ testR = (Serial_ &)Serial2;
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
  Serial.println("Radio Ready, waiting for base station");
  radioModule.setAddress(2,false);
  radioModule.setChannel(3,false);
  Serial.println(radioModule.setFixedTransmission(true,true));
  radioModule.printBoardParameters();
  delay(50);



  String data = "test";
  uint8_t test[7] = {0x00,0x03,0x04};
  memcpy(&test[3],data.c_str(),4);

  for(byte x : test){
    Serial.print(x);
    Serial.print(",");
  }
  Serial.print('\n');

  while(!testR.availableForWrite()){
    Serial.println("ping");
    delay(1);
  }
  RadioConnection.write(test,7);


  //sendRadio(data);



  //char test[7] = {0xFF,0xFF,4,'t','e','s','t'};
  //String data = "Test";
  //sendRadio(data);
  //Serial.println(RadioConnection.println("test"));
  Serial.println("Send Data");
  //byte data[3] = {0x04,0xd2,0x00};
  //RadioConnection.write(data,3);
  //sendRadio("TEST");

  // FF represent max value of 2 complement, thus 0

  /* With basic message:
   * same channel, same address, transparent; success
   * odd channel, same address, transparent; fail
   * same channel, odd address, transparent; fail
   * odd channel, odd address, transparent; fail
   * same channel, broadcast address, transparent; success
   *
   * With complex message:
   *  ensure we have a check that the serial connection for our number of bytes is available before sending
   */


  /*
  int address = random(0,65535);
  int channel = random(1,80);
  radioModule.setFixedTransmission(false, true);
  radioModule.setAddress(1234,false);
  radioModule.setChannel(0,false);
  radioModule.setPower(Power_30,false);
  Serial.println(radioModule.getAddress());
  Serial.println(radioModule.getChannel());
  Curve25519::dh1(Pub, Pri);
  String data = "0,";
  data += address;
  data += ",";
  data += channel;
  data += ",";
  data += radioModule.getPower();
  for(byte x : Pub){
    data += x;
    data += ",";
  }
  Serial.println(data);
  sendRadio(data);
  //radioModule.setFixedTransmission(true, false);
  //radioModule.setAddress(address,false);
  //radioModule.setChannel(channel,false);
  Serial.println("Recieved: ");
   */
  while(true){
    if(RadioConnection.available()){
      byte buf[10];
      RadioConnection.readBytes(buf,10);
      for(byte x : buf){
        Serial.print(x,HEX);
      }
    }
  }
}

