// From https://www.learnrobotics.org/blog/arduino-data-logger-csv/

#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"

ADXL345 accel;

int16_t ax, ay, az;

void setup(){
      // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(9600);

  // initialize device
  accel.initialize();

}

//globals
int data1, data2, incomingData; //store data from both sensors
int freq = 1000; //data collection frequency ~x milliseconds
unsigned long start_time;

void loop(){
  while(Serial.available()){
    incomingData = Serial.read();
    if (incomingData == 't'){
      start_time = millis();
      while ((millis()-start_time)<10000){
        accel.getAcceleration(&ax, &ay, &az);
        Serial.print(millis()-start_time);
        Serial.print(",");
        Serial.print(ay);
        Serial.print(",");
        delay(10);
      }
      Serial.println();
    }
  }
}

  




