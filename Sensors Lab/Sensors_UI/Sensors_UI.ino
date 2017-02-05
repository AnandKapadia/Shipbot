
//import libraries
#include <SharpIR.h>
#include <NewPing.h>

#define TRIGGER_PIN  8   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define POT_PIN      A1  //Arduino pin tied to the potentiometer
#define FLEX_PIN     A2  //Arduino pin tied to the flex sensor
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define sharp_pin A0     //Arduino pin tied to the sharp sensor

//25 is the number of readings in each mean distace output
//93 is the difference between two consecutive measurements to be considered valid
//1080 is for a sharp sensor from 10 - 80 cm in range (model number of sensor)
SharpIR sharp(sharp_pin, 25, 93, 1080);

// NewPing setup of pins and maximum distance.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

//int to get raw analog values
int raw;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  //setup sharp sensor pin as library does not do this
  pinMode(sharp_pin,INPUT); 
  //no need to setup analog pins as inputs as they are inputs by default
}

void loop() {
  //wait for a bit
  delay(50);
  
  //display sharp value
  Serial.print("Sharp value:");
  Serial.print(sharp.distance());
  Serial.print(" cm\t");
  
  //display ultrasonic value
  Serial.print("Ultrasonic: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print("cm \t");

  //display pot value
  Serial.print("Potentiometer: ");
  raw = analogRead(POT_PIN);
  //scale from 0-5V (0 - 1023) to a percentage
  Serial.print(map(raw, 0, 1023, 0, 100));
  Serial.print(" %\t");  

  //display flexi value
  Serial.print("Flex Sensor: ");
  raw = analogRead(FLEX_PIN);
  //scale from range of 700-1000 to a percentage
  Serial.print(map(raw, 700, 1000, 0, 100));
  Serial.print(" %\n");
}
