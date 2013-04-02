
/*
* rosserial PWM reader
* rosserial Servo Control
*
* This sketch combines olinoboat's ROS publishers and subscribers
*
*/

// Includes all of this file's dependencies. Make sure they're in the right path!
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_GPS.h>
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <SoftwareSerial.h>
#include <std_msgs/UInt16MultiArray.h>

// Enable node handling, letting this file initialize publishers and subscribers.
ros::NodeHandle nh;
    
// Set variables to be used later format: datatype variablename
int encoder_pin = 8;
int servo1_pin = 9;
int servo2_pin = 10;
int water_pin = A1;
unsigned long duration;
int water_voltage = 0;
char location[2] = { 0 };
uint32_t timer = millis();
std_msgs:: UInt16 pwm_msg; // The encoder outputs a 16 bit unsigned integer for the amount of time the pulse is high
std_msgs:: UInt16 water_msg; // The water sensor outputs a value for its voltage between 0V and 5V depending on whether the circuit senses water or not.
std_msgs:: UInt16 compass_msg; // The compass outputs a 16 bit unsigned integer from 0 at North clockwise to 360 if you've calibrated.
std_msgs:: UInt16 servo1_msg;
std_msgs:: UInt16 servo2_msg;
std_msgs:: UInt16 GPS_msg;

//Instantiate instances of classes
LSM303 compass;
Servo servo1;
Servo servo2;

//GPS setup junk
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean);

// Define callback functions for subscribers (what to do when a new signal comes in)
  // Callback response for servo1
void servo1_cb( const std_msgs::UInt16& cmd_msg1){	//function servo1_cb references the servo command
  servo1.write(cmd_msg1.data); //set servo angle, should be from 0-180
}
 
// Callback response for servo2
void servo2_cb( const std_msgs::UInt16& cmd_msg2){ //function servo2_cb requires a UInt16 input
  servo2.write(cmd_msg2.data); //set servo angle, should be from 0-180
}
    
// Define subscribers: <std_msgs::OutputDataType> subscribername("topic_name", callback_function);
        //Each subscriber has a name, is subscribed to a topic with data of a specific type. When it hears something, it runs the callback function with the data as arguments.
ros::Subscriber<std_msgs::UInt16> sub1("servo1", servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", servo2_cb);

// define publishers: publishername("topic_name", &message);
ros::Publisher pub_pwm("pwm_duration", &pwm_msg);
ros::Publisher pub_water("leak", &water_msg);
ros::Publisher pub_compass("heading", &compass_msg);

// This is the setup code that runs on startup and calls the functions defined above.
void setup(){
  Serial.begin(115200);
  // Initialize the Arduino as a fake node
  nh.initNode();
  
  // Set up Arduino hardware pins
  pinMode(encoder_pin, INPUT); // set encoder_pin to input
  pinMode(water_pin, INPUT); // set water_pin to input
  pinMode(13, OUTPUT);
  servo1.attach(servo1_pin); //attach servo1 to servo1_pin
  servo2.attach(servo2_pin); //attach servo2 to servo2_pin

  // Set up Compass
  Wire.begin();
  compass.init();
  compass.enableDefault();
  // Calibration values. Use the Calibrate example program to get the values for
  // your compass.
  compass.m_min.x = -520; compass.m_min.y = -570; compass.m_min.z = -770;
  compass.m_max.x = +540; compass.m_max.y = +500; compass.m_max.z = 180;


  // "Advertise" to the node the topics being published
  nh.advertise(pub_pwm);
  nh.advertise(pub_compass);
  nh.advertise(pub_water);
      
  // Subscribe to the node to the servo control topics
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  delay(1);
}


// This is the code that runs repeatedly until you shut it down.
void loop(){
  nh.spinOnce();

  // Collecting the encoder PWM signal for relative wind direction
  pwm_msg.data = pulseIn(encoder_pin, HIGH);
  
  // Collecting water sensor voltage
  water_msg.data = analogRead(water_pin);


  // Collecting compass input vector
  compass.read();
  compass_msg.data = compass.heading((LSM303::vector){0,-1,0});

  // Publish each new message.
  pub_pwm.publish(&pwm_msg);
  pub_water.publish(&water_msg);
  pub_compass.publish(&compass_msg);
  delay(1);
}
