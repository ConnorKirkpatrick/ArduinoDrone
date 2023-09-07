#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;
double h = 0;

void startBMP(){
  if(!bmp.begin(0x76)){

    Serial.println("No altimeter found, please check address or wiring");
    while(true){delay(1);}
  }
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_1);
  //sendRadio("Altimeter Ready");
  sendStatusMessage("Altimeter ready",SEVERITY_INFO);
  //Serial.println("Baro Ready");
}

float getAltitude(){
  if(bmp.takeForcedMeasurement()){
    //calculate alt based on baro pressure and temp
    h = ((pow(1013.25/bmp.readPressure(), 1/5257) -1) * (bmp.readTemperature())) / 0.0065; //using hysometric formula
    /*Serial.print(bmp.readPressure());
    Serial.print("--");
    Serial.print(bmp.readTemperature());
    Serial.print("--");
    Serial.print(bmp.getStatus(),BIN);
    Serial.print("--");
    Serial.println(bmp.readAltitude());*/
    if(bmp.getStatus() == 0xF3){
      //On error, the device has swapped to the SPI interface, power off-reset to recover the device
      //the issue seems to be an interaction between different I2C devices, when run on its own it functions fine
      startBMP();
    }
  }
  return h;
}

//upon baro error, the wire interface locks up and thus destroys the accuracy of the gyro