/*********
ESP8266 Connects to wifi and waits for get command with information and moves the servo for the total_time variable in a zig way with the in get command specified
time delay to test highest frequency achieved with the servo motor.
*********/

// Load Wi-Fi library
#include <ESP8266WiFi.h>
#include <Servo.h> // servo library  
Servo myservo;

float pos = 0;    // variable to store the servo position
float start_time, total_time = 5000;
float frequency=1; //amplitude should not exced 90 as it is added to the base line 90 so that it oscilattes between 0 and 180 angles for servo

float incomingData;

bool lower, higher;

// Replace with your network credentials
const char* ssid     = "batbot";
const char* password = "12345678";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header, float_string;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(9600);
  myservo.attach(2, 500, 2400); //D4 pin on esp8266

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            float_string = header.substring(header.indexOf('$')+1, header.lastIndexOf('$'));
            int i1 = float_string.indexOf(',');
            frequency = float_string.substring(0, i1).toFloat();
            float angle_40 = float_string.substring(i1 + 1).toFloat();

            start_time = millis();
            while(millis()-start_time<5000){
                myservo.write(0);
                delay(1000/(2*frequency));
                myservo.write(angle_40);
                delay(1000/(2*frequency));
            }
      
            myservo.write(0);              // tell servo to go to position in variable 'pos'
            delay(2000);
          
            
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}