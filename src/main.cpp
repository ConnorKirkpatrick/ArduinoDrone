#include <Arduino.h>
#include <Wire.h>
#include <HMC5883L_Simple.h>

//GY-271
//Vcc 5v
//ignore DRDY, is the interrupt for new data, read as input to trigger a read function
//Use I2C connectors

HMC5883L_Simple Compass;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    Compass.SetDeclination(0,14,'E');

    Compass.SetSamplingMode(COMPASS_SINGLE);
    //Can use continuous mode, single is simplest and generally most relevant

    Compass.SetScale(COMPASS_SCALE_088);
    //Change the scale to reduce jumping

    // To allow you to mount the compass in different ways you can specify the orientation:
    //   COMPASS_HORIZONTAL_X_NORTH (default), the compass is oriented horizontally, top-side up. when pointing North the X silkscreen arrow will point North
    //   COMPASS_HORIZONTAL_Y_NORTH, top-side up, Y is the needle,when pointing North the Y silkscreen arrow will point North
    //   COMPASS_VERTICAL_X_EAST,    vertically mounted (tall) looking at the top side, when facing North the X silkscreen arrow will point East
    //   COMPASS_VERTICAL_Y_WEST,    vertically mounted (wide) looking at the top side, when facing North the Y silkscreen arrow will point West
    Compass.SetOrientation(COMPASS_HORIZONTAL_Y_NORTH);
}

void loop() {
    float heading = Compass.GetHeadingDegrees();

    Serial.print("Heading: \t");
    Serial.println( heading );
    delay(1000);
}
