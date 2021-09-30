#include <Arduino.h>


#define triggerPin 2
#define echoPin 3

long duration;
int distance;

//5v to Vcc

void getDistance();

void setup() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.begin(9600);
}

void loop() {
    getDistance();
    delay(1000);
}

void getDistance(){
    //get the system ready
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(5);
    //send out a pulse
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    //read the pulse
    duration = pulseIn(echoPin, HIGH);
    //calculate distance by the delay
    distance = duration * 0.034 / 2;

    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");
}

//TODO: query x times per second, remove outliers, take average