#include <Encoder.h>
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

//Initialize Motor
const int motorPin1 = 6;  
const int motorPin2 = 5;  
const int motor1Enable = 10;
// encoder counter
long newPosition=0; long oldPosition = 0;
// sample time, (T), for the dicrete time PID controller
int LOOPTIME = 20; long lastMilli = 0;

Stepper stepper(STEPS, STEPPER_A, STEPPER_B);
//1080 is for a sharp sensor from 10 - 80 cm in range
SharpIR sharp(sharp_pin, 25, 93, 1080);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
Servo myservo;  // create servo object to control a servo
//Initialize the encoder
Encoder myEnc(3, 2);  //Interrupt pins 

int speed_act = 0;
int PWM_val = 0; int rpm_val = 0 ;     // (25% = 64; 50% = 127; 75% = 191; 100% = 255)

// PID controller parameters
float kP =   5; float kI =   5; float kD = 10;
// e(k) & e(k-1) & e(k-2)
int ek = 0; int ek_1 = 0; int ek_2 = 0;
// u(k) & u(k-1)
float uk = 0; float uk_1 = 0;

// PID controller parameters for the user defined degree adjustment
float kP_pwm =   5.5; float kI_pwm =   0.1; float kD_pwm = 0.1;
int divider_pwm = 10;
// e(k) & e(k-1) & e(k-2)
int ek_pwm = 0; int ek_pwm_1 = 0; int ek_pwm_2 = 0;
// u(k) & u(k-1)
float uk_pwm = 0; float uk_pwm_1 = 0;

int old_degree = 0; int desired_count = 0;

// inputs 
int speed_target = 0 ; int degree_target = 90; //+ for cw rotation, - for ccw rotation


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
    
  pinMode(motorPin1, OUTPUT);       //define motorpin1 as output
  pinMode(motorPin2, OUTPUT);       //define motorpin2 as output
}

void loop() {
  // put your main code here, to run repeatedly:
  if (start) {
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
  //gets (desired speed, 0) or (0, desired degree change)
  if(degree_target == 0){
  if((millis()-lastMilli) >= LOOPTIME)   { // enter timed loop
    long difference_time = millis() - lastMilli;
    lastMilli = millis();
    
    newPosition = myEnc.read();
    long difference_encoder = newPosition - oldPosition;
    oldPosition = newPosition;
    
    //Serial.println(difference_encoder);
    speed_act = getMotorData(difference_time, difference_encoder); 
    Serial.println(PWM_val);
    Serial.print("Actual Velocity: ");
    Serial.println(speed_act);
    rpm_val = updatePid(rpm_val, speed_target, speed_act, difference_time);   // compute PWM value
    Serial.print("uk_rpm: ");
    Serial.println(rpm_val);
    PWM_val = (rpm_val*255 / 72);
    if(speed_target > 0)
      run_cw(PWM_val);
    else
      run_ccw(PWM_val);
    }
  }
  else {
    long init_count = myEnc.read(); 
    desired_degree(init_count, degree_target, speed_target);
  }
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


int getMotorData(long timed, long count)  {   
 speed_act = ((count)*(60*(1000/timed)))/(720);       // actual revolution per minute
 return speed_act;
}

int updatePid(int command, int targetValue, int currentValue, int timed)   { 
// uk = uk_1 + kP*(ek - ek_1) + kI*T*ek + kD*(ek - 2*ek_1 + ek_2)/T; 
float T = timed/1000;  // sample time
ek = abs(targetValue) - abs(currentValue); ; // Compute error
int t1 = ek - ek_1; 
int t2 = t1 - ek_1 + ek_2; 
int t3 = (kD*t2)/ T; 
t2 = kI*T*ek; 
uk = uk_1 + kP*t1 + t2 + t3; 
t1 = uk; 
// Store values for next time step
uk_1 = uk; ek_2 = ek_1; ek_1 = ek; 
 return constrain(uk, 1, 72); //command
}

int update_PWM_Pid(int count, int rpm, int timed) {
  // uk = uk_1 + kP*(ek - ek_1) + kI*T*ek + kD*(ek - 2*ek_1 + ek_2)/T; 
  float T = timed/1000;  // sample time
  ek_pwm = count;  // Compute error
  int t1 = ek_pwm - ek_pwm_1; 
  int t2 = t1 - ek_pwm_1 + ek_pwm_2; 
  int t3 = (kD_pwm*t2)/ T; 
  t2 = kI_pwm*T*ek_pwm; 
  uk_pwm = uk_pwm_1 + kP_pwm*t1 + t2 + t3; 
  t1 = uk_pwm; 
  // Store values for the next time step
  uk_pwm_1 = uk_pwm; ek_pwm_2 = ek_pwm_1; ek_pwm_1 = ek_pwm; 
  
  int angle_pwm = uk_pwm / divider_pwm;
  PWM_val = (rpm * 255 / 72);
  if (rpm == 0)
    return constrain(angle_pwm, -250, 250);
  else if (rpm > 0) 
    return constrain(angle_pwm, 70, PWM_val);
  else 
    return constrain(angle_pwm, PWM_val, -70);
}

void desired_degree(long counter, int degree, int init_rpm) {

if(old_degree != degree){
  long limit_count = degree * 2; // 2 count per degree from encoder(4x mode), 720 count per revolution  
  desired_count = counter + limit_count;
  old_degree = degree;
}

int difference_count = desired_count - counter;  //

    Serial.print("Desired count: ");
    Serial.println(desired_count);
  
if(!(difference_count < 5 && difference_count > -5 )){  // error margin of +-3 counts --> +-1.5 degrees

  if((millis()-lastMilli) >= LOOPTIME)   { // enter timed loop
    long difference_time = millis() - lastMilli;   //sample time
    lastMilli = millis();
    
    newPosition = myEnc.read();
    difference_count = desired_count - newPosition;

    Serial.print("Difference count: ");
    Serial.println(difference_count);
     
    PWM_val = update_PWM_Pid(difference_count, init_rpm, difference_time); // compute PWM value
    Serial.print("uk_PWM: ");
    Serial.println(PWM_val);
    if(PWM_val > 0)
      run_cw(PWM_val);
    else
      run_ccw(-PWM_val);
    }   
}
else{
  stop();
}
}

///// MOTOR /////
void run_free()
{
  analogWrite(motor1Enable, 1);
  }

void stop()
{
  analogWrite(motor1Enable, 0);

  digitalWrite(motorPin1, LOW);   
  digitalWrite(motorPin2, LOW);     
}

void run_cw(int speed)
{
  if (speed < 85){
  analogWrite(motor1Enable, 220);
  delay(3);
  }
  analogWrite(motor1Enable, speed);

  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);    
}

void run_ccw(int speed)
{
  if (speed < 85){
  analogWrite(motor1Enable, 220);
  delay(3);
  }
  analogWrite(motor1Enable, speed);

  digitalWrite(motorPin1, LOW); 
  digitalWrite(motorPin2, HIGH);
}
///// MOTOR /////
