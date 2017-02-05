
#include <SharpIR.h>

#include <NewPing.h>

#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define POT_PIN      A1
#define FLEX_PIN     A2
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define sharp_pin A0 

//1080 is for a sharp sensor from 10 - 80 cm in range

SharpIR sharp(sharp_pin, 25, 93, 1080);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

int raw;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  pinMode(sharp_pin,INPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
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
