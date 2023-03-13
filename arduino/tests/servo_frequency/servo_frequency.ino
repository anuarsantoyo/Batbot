/* Test to see if servo reaches desireded frequencies
*/

#include <Servo.h>
#include <math.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int soll_time, ist_time, error, dir;
float pos, omega;

void setup() {
  Serial.begin(9600);
  myservo.attach(2, 500, 2400);
  //myservo.attach(2, 900, 2100);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  while(true){
    /*if (millis()%5000<2500){
      dir = 1;
    }
    else{
      dir = -1;
    }
    omega = dir*8*M_PI*millis()/2500 + M_PI/100;
    pos = 29*sin(((M_PI)/(100))*millis()) + 29;
    pos = 90*sin(((M_PI)/(100))*millis()) + 90;*/
    myservo.write(0);
    delay(3000);
    myservo.write(180);
    delay(3000);
  }
}
