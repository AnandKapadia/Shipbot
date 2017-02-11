
#include <SharpIR.h>
#include <Stepper.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define POT_PIN      A1
#define FLEX_PIN     A2
#define STEPPER_A    3
#define STEPPER_B    4
#define STEPS 800
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define sharp_pin A0

//1080 is for a sharp sensor from 10 - 80 cm in range

Stepper stepper(STEPS, STEPPER_A, STEPPER_B);
SharpIR sharp(sharp_pin, 25, 93, 1080);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myservo;  // create servo object to control a servo

int target = 0;
int start = 0;
int sensor_val;
int previous = 0;
int motor_sel = 1, sensor_sel = 1, rev_sel = -1, out_sel = 1, deg_val = 0, vel_val = 0;
int i = 0;
char mode, inChar;
String inputString;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  pinMode(sharp_pin, INPUT);
  stepper.setSpeed(30);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  // put your main code here, to run repeatedly:
  if (start) {
    delay(50);
    control_motor();
    printGUI();
  }
}

void control_motor() {
  switch (motor_sel) {
    case 1: drive_dc(); break;
    case 2: drive_servo(); break;
    case 3: drive_stepper(); break;
    default: break;
  }
}

void drive_dc() {

}

void drive_servo() {
  int pos;
  switch (sensor_sel) {
    case 1: sensor_val = map(sensor_val, 0, 50, 0, 180); break;
    case 2: sensor_val = map(sensor_val, 0, 50, 0, 180); break;
    case 3: sensor_val = map(sensor_val, 0, 100, 0, 180); break;
    case 4: sensor_val = map(sensor_val, 0,360, 0, 180); break;
    default: break;
  }
  if (rev_sel==0)
  {  
    for (pos = 0; pos <= 180; pos += 1) 
    {                   // goes from 0 degrees to 180 degrees in steps of 1 degree
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }  
    for (pos = 180; pos >= 0; pos -= 1) 
    {                   // goes from 180 degrees to 0 degrees
        myservo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
  else if (rev_sel==-1)
  {
    myservo.write(sensor_val);
  }                         // waits for the servo to get there
}


void drive_stepper() {
  switch (sensor_sel) {
    case 1: sensor_val = map(sensor_val, 0, 50, 0, 360); break;
    case 2: sensor_val = map(sensor_val, 0, 50, 0, 360); break;
    case 3: sensor_val = map(sensor_val, 0, 100, 0, 360); break;
    default: break;
  }
  target = (int)(800.0 * ((float)sensor_val / 360.0));
  // move a number of steps equal to the change in the
  // sensor reading
  stepper.step(target - previous);
  // remember the previous value of the sensor
  previous = target;
}

void printGUI() {
  int sharpval, ultrasonicval, potval;
  int raw;

  sharpval = sharp.distance();
  ultrasonicval = sonar.ping_cm();
  raw = analogRead(POT_PIN);
  //scale from 0-5V (0 - 1023) to a percentage
  potval = map(raw, 0, 1023, 0, 100);

  //display sharp value
  Serial.print("sharp\n");
  Serial.print(sharpval);
  Serial.print('\n');

  //display ultrasonic value
  Serial.print("ultrasonic\n");
  Serial.print(ultrasonicval); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print('\n');

  //display pot value
  Serial.print("pot\n");
  Serial.print(potval);
  Serial.print('\n');

  //convert sensor raw vals and selected line to sensor val for motor output
  switch (sensor_sel) {
    case 1: sensor_val = sharpval; break;
    case 2: sensor_val = ultrasonicval; break;
    case 3: sensor_val = potval; break;
    //gui selector case
    case 4: //if degrees is selected
      if (out_sel == 1) {
        sensor_val = deg_val;
      }//if velocity is selected
      else {
        sensor_val = vel_val;
      }
      break;
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


