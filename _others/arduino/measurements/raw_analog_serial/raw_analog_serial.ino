// From https://www.learnrobotics.org/blog/arduino-data-logger-csv/
// Read data from Analog Inputs and sends back collected data through serial communication
int sensor1 = A0;
int sensor2 = A1;

void setup(){
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
}

//globals
int data1, data2, incomingData; //store data from both sensors
int freq = 1000; //data collection frequency ~x milliseconds
unsigned long start_time;
int total_time = 5000; //time in millis

void loop(){
  while(Serial.available()){
    incomingData = Serial.read();
    if (incomingData == 't'){
      start_time = millis();
      while ((millis()-start_time)<total_time){
        data1 = digitalRead(sensor1);
        data2 = digitalRead(sensor2);
        //Display Data to Serial Monitor
        Serial.print(millis()-start_time);
        Serial.print(",");
        Serial.print(data1);
        Serial.print(",");
        Serial.print(data2);
        Serial.print(",");
      }
      Serial.println();
    }
  }
}

  




