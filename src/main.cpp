#include <Arduino.h>

//requires system be wired into PWM ports
void setup() {
    pinMode(2, INPUT);
    Serial.begin(9600);
}

void loop() {
    int data = 0;
    int ch = pulseIn(2, HIGH, 30000);
    Serial.println(ch);
    data =  map(ch, 1000, 2000, 0, 100);
    Serial.println(data);
    delay(500);
}
/* ch1 is roll
 * ch2 is pitch
 * ch3 is throttle
 * ch4 is yaw
 * ch5 is Switch A 0 or 99
 * ch6 is Switch B 0 or 99
 * ch7 is Switch C 0 or 49/50 or 99
 * ch8 is Switch D 0 or 99
 * ch9 is VRA
 * ch10 is VRB
 */