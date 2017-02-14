#include <Encoder.h>
//Initialize Motor
const int motorPin1 = 6;  
const int motorPin2 = 5;  
const int motor1Enable = 10;
//Initialize the encoder
Encoder myEnc(3, 2);  //Interrupt pins 
// encoder counter
long newPosition=0; long oldPosition = 0;
// sample time, (T), for the dicrete time PID controller
int LOOPTIME = 20; long lastMilli = 0;

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

void setup() {
Serial.begin(9600);
      
pinMode(motorPin1, OUTPUT);       //define motorpin1 as output
pinMode(motorPin2, OUTPUT);       //define motorpin2 as output
}

void loop() {

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
///// FUNCTIONS /////
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
