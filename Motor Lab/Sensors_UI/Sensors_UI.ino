
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

int start = 0;
int sensor_val;
int motor_sel = 1, sensor_sel = 1, rev_sel = -1, out_sel = 1, deg_val = 0, vel_val = 0;
int i = 0;
char mode, inChar;
String inputString;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  pinMode(sharp_pin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (start) {
    control_motor();
    printGUI();
  }
}

void control_motor() {


}

void printGUI() {
  int sharpval, ultrasonicval, potval;
  int raw;
  //display sharp value
  Serial.print("sharp\n");
  sharpval = sharp.distance();
  Serial.print(sharpval);
  Serial.print('\n');

  //display ultrasonic value
  Serial.print("ultrasonic\n");
  ultrasonicval = sonar.ping_cm();
  Serial.print(ultrasonicval); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print('\n');

  //display pot value
  Serial.print("pot\n");
  raw = analogRead(POT_PIN);
  //scale from 0-5V (0 - 1023) to a percentage
  potval = map(raw, 0, 1023, 0, 100);
  Serial.print(potval);
  Serial.print('\n');

  switch (sensor_sel) {
    case 1: sensor_val = sharpval; break;
    case 2: sensor_val = ultrasonicval; break;
    case 3: sensor_val = potval; break;
    default: break;
  }
}

void serialEvent() {
  //when we recieve serial
  start = 1;
  int val;
  //read whole message
  while (Serial.available()) {
    //read the first char as mode
    inChar = (char)Serial.read();
    if (inChar == 'm' || inChar == 's' || inChar == 'r' || inChar == 'o' ||  inChar == 'd' || inChar == 'v' || inChar == 'z') {
      mode = inChar;
      i = 1;
    }
    else if (inChar == '\n') {
      //convert the trest of the input to an int
      val = inputString.toInt();
      //Serial.println(val);
      inputString = "";
      switch (mode) {
        case 'm': motor_sel = val;  break;
        case 's': sensor_sel = val; break;
        case 'r': rev_sel = val;    break;
        case 'o': out_sel = val;    break;
        case 'd': deg_val = val;    break;
        case 'v': vel_val = val;    break;
        case 'z': start = 1;        break;
        default: break;
      }
      break;
    }
    //otherwise get the rest of the input
    else {
      // add it to the inputString:
      inputString += inChar;
    }
  }
  //assign variables based on what was just read

}


