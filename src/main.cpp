#include <Arduino.h>
#include <pt.h>

static struct pt pt1;

static int protothreadBlinkLED1(struct pt *pt);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    PT_INIT(&pt1);
}

void loop() {
    protothreadBlinkLED1(&pt1);
}

static int protothreadBlinkLED1(struct pt *pt)
{
    static unsigned long lastTimeBlink = 0;
    PT_BEGIN(pt);
    while(1) {
        lastTimeBlink = millis();
        PT_WAIT_UNTIL(pt, millis() - lastTimeBlink > 1000);
        digitalWrite(LED_BUILTIN, HIGH);
        lastTimeBlink = millis();
        PT_WAIT_UNTIL(pt, millis() - lastTimeBlink > 1000);
        digitalWrite(LED_BUILTIN, LOW);
    }
    PT_END(pt);
}


///Notes:
/// Dont use local variables with PT, use static as local are not preserved during a block
/// any protothread function must have a pointer to the protothread as a parameter
/// PT Begin and End start and stop the thread
/// Use PT_WAIT_UNTIL(thread,condition), this will keep the thread paused until the condition is true
/// Trick, you can call the wait immediately after an action is performed to preserve memory
/// I will use this initially to blink the nav lights on the drone, later to expand the flight controllers autonomy
/// DO NOT USE INTERRUPTS: these block the main process and can cripple the FC system