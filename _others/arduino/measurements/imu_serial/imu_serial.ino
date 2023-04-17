// From https://www.learnrobotics.org/blog/arduino-data-logger-csv/
// Measures IMU-pitch data when the char 't' is serial sent and sends through serial communication a single line of the collected data. 

#include <IMUGY85.h>

IMUGY85 imu;
int sensor1 = A0;
int sensor2 = A1;
double ax, ay, az, gx, gy, gz, roll, pitch, yaw;

void setup(){
  // put your setup code here, to run once:
  Serial.begin(9600);
  imu.init();

}

//globals
int incomingData; //store data from both sensors
unsigned long start_time;

void loop(){
  while(Serial.available()){
    incomingData = Serial.read();
    if (incomingData == 't'){
      start_time = millis();
      while ((millis()-start_time)<10000){
        imu.update();
        pitch = imu.getPitch();
        Serial.print(millis()-start_time);
        Serial.print(",");
        Serial.print(pitch);
        Serial.print(",");
        delay(10);
      }
      Serial.println();
    }
  }
}

  




