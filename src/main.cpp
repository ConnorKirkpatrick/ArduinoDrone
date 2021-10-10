#include <Arduino.h>

//NO VCC IN
//connect ground and the sensor pin

int offset = 0;
int volt = 0;
double voltage = 0;

void setup() {
    Serial.begin(9600);
    pinMode(50, OUTPUT);
    pinMode(A0,INPUT);
}

void loop() {
    digitalWrite(50,LOW);
    volt = analogRead(A0);
    voltage = map(volt,0,1023, 0, 2500) + offset;
    voltage /=100;

    Serial.println(voltage);
    delay(1000);

    digitalWrite(50,HIGH);
    volt = analogRead(A0);
    voltage = map(volt,0,1023, 0, 2500) + offset;
    voltage /=100;

    Serial.println(voltage);
    delay(1000);

}