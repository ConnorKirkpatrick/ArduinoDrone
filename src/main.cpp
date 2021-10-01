#include <Arduino.h>
#include <protothreads.h>

static struct pt pt1;
static struct pt pt2;

static int ProtoThread1(struct pt *pt);
static int ProtoThread2(struct pt *pt);

void setup() {
    Serial.begin(9600);
    Serial.println("Starting");
    pinMode(LED_BUILTIN, OUTPUT);
    PT_INIT(&pt1);
    PT_INIT(&pt2);
}

void loop() {
    PT_SCHEDULE(ProtoThread1(&pt1));
    PT_SCHEDULE(ProtoThread2(&pt2));
}

static int ProtoThread1(struct pt *pt)
{
    static unsigned long lastTimeBlink = 0;
    PT_BEGIN(pt);
    while(1) {
        Serial.println("Printing every 1 second");
        PT_SLEEP(pt,1000)
    }
    PT_END(pt);
}
static int ProtoThread2(struct pt *pt)
{
    static unsigned long lastTimeBlink = 0;
    PT_BEGIN(pt);
    while(1) {
        Serial.println("Printing every 3 seconds");
        PT_SLEEP(pt,3000)
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