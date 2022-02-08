#include <Arduino.h>

//requires system be wired into PWM ports
volatile int pwm_value = 0;
volatile int prev_time = 0;
 
void setup() {
  Serial.begin(9600);
  // when pin D2 goes high, call the rising function
  attachInterrupt(digitalPinToInterrupt(12), rising, RISING);
}
 
void loop() { }
//Use Interups on the DUE as reading the 5v pwm from the RC reciever is flakey using pulseIn()
void rising() {
  attachInterrupt(digitalPinToInterrupt(12), falling, FALLING);
  prev_time = micros();
}
 
void falling() {
  attachInterrupt(digitalPinToInterrupt(12), rising, RISING);
  pwm_value = micros()-prev_time - 5;
  if(pwm_value < 1000){pwm_value = 1000;}
  Serial.println(pwm_value);
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
