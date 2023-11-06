/*
Simply Prints the analog reading in pin A0 and A1 and prints it in Serial Monitor.
Usefull to see raw values of sensors
*/

// These constants won't change. They're used to give names to the pins used:
const int analogInPin0 = A0;  // Analog input pin that the potentiometer is attached to
const int analogInPin1 = A1;  // Analog input pin that the potentiometer is attached to


int sensorValue0 = 0;  // value read from the pot
int sensorValue1 = 0;  // value read from the pot


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  sensorValue0 = digitalRead(analogInPin0);
  sensorValue1 = digitalRead(analogInPin1);

  // change the analog out value:

  // print the results to the Serial Monitor:
  Serial.print("sensor 0 = ");
  Serial.print(sensorValue0);
  Serial.print("\t sensor 1 = ");
  Serial.println(sensorValue1);

  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(2);
}
