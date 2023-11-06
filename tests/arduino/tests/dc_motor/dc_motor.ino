const int thisPin = 12;
void setup() {
  // put your setup code here, to run once:
  pinMode(thisPin, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  for (int brightness = 0; brightness < 255; brightness++) {
      analogWrite(thisPin, brightness);
      delay(2);
  }
  // fade the LED on thisPin from brightest to off:
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(thisPin, brightness);
    delay(2);
  }
  // pause between LEDs:
  delay(100);

}
