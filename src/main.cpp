#include <Arduino.h>
#include <MedianFilterLib2.h>

#define triggerPin 52
#define echoPin 53

long duration;
int distance;

//5v to Vcc
int getDistance();
MedianFilter2<int> medianFilter2(5);

void setup() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.begin(9600);
}

void loop() {
    int median = medianFilter2.AddValue(getDistance());
    Serial.println(median);
    Serial.println(medianFilter2.GetFiltered());
}

int getDistance(){
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

    return distance;
}

//TODO: query x times per second, remove outliers, take average