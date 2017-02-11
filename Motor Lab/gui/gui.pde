import processing.serial.*;
import controlP5.*;

//serial init
Serial myPort;  // Create object from Serial class
String serial_val, serial_mode, portName;     // Data received from the serial port
//row height h_base initialization
int h_base = 40;
//color
color white = color(255, 255, 255);
color gray = color(105, 105, 105);
//initialize font
PFont f;
//init background image
PImage img;
//motor selector
String[] motors = {"dc", "servo", "stepper"};
//sensor selector
String[] sensors = {"sharp", "ultrasonic", "pot", "gui"};
//output selector
String[] output = {"deg", "vel"};
//values
int sharp = 0, ultrasonic = 0, pot = 0;
//gui control
ControlP5 cp5;
RadioButton m, s, o, c;
Knob deg, vel;

void setup() {
  //setup the space1
  size(640, 200);
  //init serial
  portName = "/dev/cu.usbmodem1411";
  myPort = new Serial(this, portName, 9600);
  //set background
  img = loadImage("motor.jpg");
  f = createFont("Arial", 12, true);
  textFont(f);
  //setup the motor selectors
  cp5 = new ControlP5(this);
  m = cp5.addRadioButton("motorButton")
    .setPosition(20, h_base)
    .setSize(40, 20)
    .setItemsPerRow(1)
    .setSpacingColumn(50)
    .addItem(motors[0], 1)
    .addItem(motors[1], 2)
    .addItem(motors[2], 3)
    .setNoneSelectedAllowed(false)
    .activate(0)
    ;
  //setup the sensor buttons
  s = cp5.addRadioButton("sensorButton")
    .setPosition(120, h_base)
    .setSize(40, 20)
    .setItemsPerRow(1)
    .setSpacingColumn(50)
    .addItem(sensors[0], 1)
    .addItem(sensors[1], 2)
    .addItem(sensors[2], 3)
    .addItem(sensors[3], 4)
    .setNoneSelectedAllowed(false)
    .activate(0)
    ;
  //setup the selector between degrees and velocity
  o = cp5.addRadioButton("outputButton")
    .setPosition(220, h_base+60)
    .setSize(40, 20)
    .setItemsPerRow(2)
    .setSpacingColumn(60)
    .addItem(output[0], 1)
    .addItem(output[1], 2)
    .setNoneSelectedAllowed(false)
    .activate(0)
    ;
  //setup the reverse dir checkbox
  c = cp5.addRadioButton("checkBox")
    .setPosition(20, h_base + 63)
    .setSize(40, 20)
    .setSpacingColumn(50)
    .addItem("rev dir?", 0)
    ;
  //setup the degrees knob
  deg = cp5.addKnob("deGrees")
    .setRange(0, 360)
    .setValue(0)
    .setPosition(220, h_base)
    .setRadius(20)
    .setDragDirection(Knob.VERTICAL)
    ;   
  //setup the velocity knob
  vel = cp5.addKnob("velocity")
    .setRange(0, 100)
    .setValue(0)
    .setPosition(320, h_base)
    .setRadius(20)
    .setDragDirection(Knob.VERTICAL)
    ;
}

void draw() {

  background(img);
  //read serial
  readVals();
  //draw black square around text
  fill(gray);
  rect(400, h_base, 150, 100, 7);
  //write the values of the sensors
  fill(white);
  text("Sensor Values: \n  Sharp: " + Integer.toString(sharp) + "cm" +
    "\n  Ultrasonic: "+ Integer.toString(ultrasonic) + "cm" +
    "\n  Potentiometer: " + Integer.toString(pot) + "%\n", 420, h_base + 20);
}

void readVals() {
  if ( myPort.available() > 0) 
  {  // If data is available,
    serial_mode = myPort.readStringUntil('\n');         // read it and store it in val
    serial_val = myPort.readStringUntil('\n');         // read it and store it in val
    if (serial_mode == null || serial_val == null) return;
    serial_val = serial_val.substring(0, serial_val.length()-1);
    if (serial_mode.equals("sharp\n")) {
      sharp = Integer.parseInt(serial_val);
    } else if (serial_mode.equals("ultrasonic\n")) {
      ultrasonic = Integer.parseInt(serial_val);
    } else if (serial_mode.equals("pot\n")) {
      pot = Integer.parseInt(serial_val);
    }
    //print(sharp);
    //print(ultrasonic);
    //print(pot);
    //print(serial_val);
    //println(serial_mode);
  }
}

void motorButton(int a) {
  println("m"+a);
  myPort.write('m');
  char[] chars = ("" + a).toCharArray();
  for(int i = 0; i < chars.length; i++){
    myPort.write(chars[i]);
  }
  myPort.write('\n');
}
void outputButton(int a) {
  println("o"+a);
  myPort.write('o');
  char[] chars = ("" + a).toCharArray();
  for(int i = 0; i < chars.length; i++){
    myPort.write(chars[i]);
  }
  myPort.write('\n');
}

void sensorButton(int a) {
  println("s"+a);
  myPort.write('s');
  char[] chars = ("" + a).toCharArray();
  for(int i = 0; i < chars.length; i++){
    myPort.write(chars[i]);
  }
  myPort.write('\n');
}

void checkBox(int a) {
  println("r"+a);
  myPort.write('r');
  char[] chars = ("" + a).toCharArray();
  for(int i = 0; i < chars.length; i++){
    myPort.write(chars[i]);
  }
  myPort.write('\n');
}

void deGrees(int a) {
  println("d"+a);
  myPort.write('d');
  char[] chars = ("" + a).toCharArray();
  for(int i = 0; i < chars.length; i++){
    myPort.write(chars[i]);
  }
  myPort.write('\n');
}

void velocity(int a) {
  println("v"+a);
  myPort.write('v');
  char[] chars = ("" + a).toCharArray();
  for(int i = 0; i < chars.length; i++){
    myPort.write(chars[i]);
  }
  myPort.write('\n');
}