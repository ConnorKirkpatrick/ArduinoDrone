#include <Arduino.h>
#include <protothreads.h>

static struct pt pt1;
static struct pt pt2;

static int ProtoThread1(struct pt *pt);
static int ProtoThread2(struct pt *pt);
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

//Vcc 5v
//use TX and RX rather than I2C
//BAUD rate is 9600

float coords[6];

SoftwareSerial serial_connection(10, 11); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data

void getCoords(float array[]);

void setup()
{
    Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
    serial_connection.begin(9600);//This opens up communications to the GPS
    Serial.println("GPS Start");//Just show to the monitor that the sketch has started
}

void loop()
{
    //query GPS
    getCoords(coords);
    //if there is no satellites, no valid fix can be made thus retry
    while(coords[0] == 0){
        getCoords(coords);
    }
    Serial.print("Sats: ");
    Serial.println(coords[0],0);
    Serial.print("Time: ");
    Serial.println(coords[1],0);
    Serial.print("LAT: ");
    Serial.println(coords[2],6);
    Serial.print("LNG: ");
    Serial.println(coords[3],6);
    Serial.print("Speed(m/s): ");
    Serial.println(coords[4]);
    Serial.print("altitude(m): ");
    Serial.println(coords[5]);
    Serial.println("\n\n\n");
    delay(10000);
}

void setup() {
    Serial.begin(9600);
    Serial.println("Starting");
    pinMode(LED_BUILTIN, OUTPUT);
    PT_INIT(&pt1);
    PT_INIT(&pt2);
}
void getCoords(float array[]){
    while(!gps.location.isUpdated()){
        while(serial_connection.available())//While there are characters to come from the GPS
        {
            gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
        }
    }
    array[0] = gps.satellites.value();
    array[1] = gps.time.value();
    array[2] = gps.location.lat();
    array[3] = gps.location.lng();
    array[4] = gps.speed.mps();
    array[5] = gps.altitude.meters();


/*    if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
    {

//Get the latest info from the gps object which it derived from the data sent by the GPS unit
        Serial.println("GMT Time:");
        Serial.println(gps.time.value());
        Serial.println("Satellite Count:");
        Serial.println(gps.satellites.value());
        Serial.println("Latitude:");
        Serial.println(gps.location.lat(), 6);
        Serial.println("Longitude:");
        Serial.println(gps.location.lng(), 6);
        Serial.println("Speed MPH:");
        Serial.println(gps.speed.mph());
        Serial.println("Altitude Feet:");
        Serial.println(gps.altitude.feet());
        Serial.println("");
    }*/

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



/// Best use case; offload the reading and encoding onto a protoThread
/// Use the static GPS object to query for the coords at a more reasonable interval
/// On start, wait until the GPS is acquired before taking flights