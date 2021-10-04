#include <Arduino.h>

int pin = 13;
//988
//2093
int note = 	2000;

void setup()
{
}

void loop()
{
    tone(pin, note, 200);
    delay(200);
    noTone(pin);
    delay(3000);
}