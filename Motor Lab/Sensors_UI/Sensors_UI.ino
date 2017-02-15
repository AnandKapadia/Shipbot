#include <Encoder.h>
#include <SharpIR.h>
#include <Stepper.h>
#include <NewPing.h>
#include <Servo.h>

#define TRIGGER_PIN  8  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     12  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define POT_PIN      A1
#define FLEX_PIN     A2
#define STEPPER_A    11
#define STEPPER_B    4
#define STEPS 800
#define BUTTONPIN   7
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define sharp_pin A0

//Initialize Motor
const int motorPin1 = 6;  
const int motorPin2 = 5;  
const int motor1Enable = 10;
int motorp = 0;
int motorv = 0;
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
float kP =   0.5; float kI =   0; float kD = 0.02;
// e(k) & e(k-1) & e(k-2)
int ek = 0; int ek_1 = 0; int ek_2 = 0;
// u(k) & u(k-1)
float uk = 0; float uk_1 = 0;

// PID controller parameters for the user defined degree adjustment
float kP_pwm = 1.5; float kI_pwm =  0; float kD_pwm = 0.2;
float divider_pwm = 1;
// e(k) & e(k-1) & e(k-2)
float ek_pwm = 0; float ek_pwm_1 = 0; float ek_pwm_2 = 0;
// u(k) & u(k-1)
float uk_pwm = 0; float uk_pwm_1 = 0;


int old_degree = 0; int desired_count = 0;

// inputs 
int speed_target = 50 ; int degree_target = 0; //+ for cw rotation, - for ccw rotation


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
  pinMode(BUTTONPIN, INPUT);
  pinMode(motorPin1, OUTPUT);       //define motorpin1 as output
  pinMode(motorPin2, OUTPUT);       //define motorpin2 as output
}
int count = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (start) {
    control_motor();
    if(count%1000 == 0) printGUI();
  }    
  count++;
}

void control_motor() {
  switch (motor_sel) {
    case 1: drive_dc();  break;
    case 2: drive_servo(); break;
    case 3: drive_stepper(); break;
    default: break;
  }
  check_button();
}

void check_button(){
  //if high
  if(digitalRead(BUTTONPIN) == HIGH){
    //debounce basic method
    delay(50);
    if(digitalRead(BUTTONPIN) == HIGH){
      //do action (go to next sensor)
      sensor_sel += 1;
      if(sensor_sel >= 4) sensor_sel = 1;
    }
    //wait for depressed button
    while(digitalRead(BUTTONPIN) == HIGH) {};
    //debounce again
    delay(50);
  }
}

void drive_dc() {
  int reverse = 1;
  if(rev_sel == 2) reverse = -1;
  if(out_sel == 1){
    switch (sensor_sel) {
      case 1: deg_val = map(sensor_val, 0, 50, 0, 360); break;
      case 2: deg_val = map(sensor_val, 0, 50, 0, 360); break;
      case 3: deg_val = map(sensor_val, 0, 100, 0, 360); break;
      case 4: deg_val = map(sensor_val, 0,360, 0, 360); break;
      default: break;
    }
    degree_target = reverse * deg_val;
    speed_target = 0;
  }
  else{
     switch (sensor_sel) {
      case 1: vel_val = map(sensor_val, 0, 50, 0, 72); break;
      case 2: vel_val = map(sensor_val, 0, 50, 0, 72); break;
      case 3: vel_val = map(sensor_val, 0, 100, 0, 72); break;
      case 4: vel_val = map(sensor_val, 0,360, 0, 72); break;
      default: break;
    }
    degree_target = 0;
    speed_target = reverse * vel_val;
    
  }
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
    motorv = speed_act;
    rpm_val =  updatePid(rpm_val, speed_target, speed_act, difference_time);   // compute PWM value
    PWM_val = (rpm_val*255 / 72);
    if(speed_target > 0)
      run_cw((int)PWM_val);
    else
      run_ccw((int)PWM_val);
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

 //display pot value
  Serial.print("motorp\n");
  Serial.print(motorp);
  Serial.print('\n');

   //display pot value
  Serial.print("motorv\n");
  Serial.print(motorv);
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

///// FUNCTIONS /////
int getMotorData(long timed, long count)  {   
 speed_act = ((count)*(60*(1000/timed)))/(720);       // actual revolution per minute
 return speed_act;
}

float updatePid(int command, int targetValue, int currentValue, int timed)   { 
// uk = uk_1 + kP*(ek - ek_1) + kI*T*ek + kD*(ek - 2*ek_1 + ek_2)/T; 
float T = ((float)timed)/1000;  // sample time
ek = abs(targetValue) - abs(currentValue); ; // Compute error
float t1 = ek ; 
float t2 = t1 - ek_1; 
float t3 = (kD*t2)/ T; 
t2 = kI*T*ek; 
uk = uk_1 + kP*t1 + t2 + t3; 
t1 = uk; 
// Store values for next time step
uk_1 = uk; ek_2 = ek_1; ek_1 = ek; 
  return constrain(uk, 1, 72); //command
}

float update_PWM_Pid(int count_error, int rpm, int timed) {
  // uk = uk_1 + kP*(ek - ek_1) + kI*T*ek + kD*(ek - 2*ek_1 + ek_2)/T; 
  float T = ((float)timed)/1000;  // sample time

  ek_pwm = count_error;  // Compute error
  float t1 = ek_pwm; 
  float t2 = t1 - ek_pwm_1; 
  float t3 = (kD_pwm*t2)/ T; 

    t2 = kI_pwm*T*ek_pwm; 
  uk_pwm =  kP_pwm*t1 + t2 + t3; 
  t1 = uk_pwm; 
  // Store values for the next time step
  uk_pwm_1 = uk_pwm; ek_pwm_2 = ek_pwm_1; ek_pwm_1 = ek_pwm; 
  
  float angle_pwm = uk_pwm / divider_pwm;

  if (angle_pwm > 0) 
    return constrain(angle_pwm, 0, 250);
  else 
    return constrain(angle_pwm, -250, 0);
  
  
}

void desired_degree(long counter, int degree, int init_rpm) {

if(old_degree != degree){
  long limit_count = degree * 2; // 2 count per degree from encoder(4x mode), 720 count per revolution  
  desired_count = counter + limit_count;
  old_degree = degree;
}

int difference_count = desired_count - counter;  //

  
//if(!(difference_count < 8 && difference_count > -8 )){  // error margin of +-3 counts --> +-1.5 degrees
//if(1){  // error margin of +-3 counts --> +-1.5 degrees
if(!(difference_count < 4 && difference_count > -4 )){  // error margin of +-3 counts --> +-1.5 degrees

  if((millis()-lastMilli) >= LOOPTIME)   { // enter timed loop
    int difference_time = millis() - lastMilli;   //sample time
    lastMilli = millis();
    
    newPosition = myEnc.read();
    difference_count = desired_count - newPosition;
    
    motorp = difference_count/2;
    
    PWM_val = update_PWM_Pid(difference_count, init_rpm, difference_time); // compute PWM value
    if(PWM_val > 0)
      run_cw((int)PWM_val);
    else
      run_ccw((int)(-PWM_val));
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
