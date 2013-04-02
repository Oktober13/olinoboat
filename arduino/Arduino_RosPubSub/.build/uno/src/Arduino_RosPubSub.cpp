#include <Arduino.h>
void useInterrupt(boolean v);
void setup();
void loop();
#line 1 "src/Arduino_RosPubSub.ino"
/*
 * rosserial wind reader
 * rosserial Servo Control
 *
 * This sketch combines olinoboat's ROS publishers and subscribers
 * 
 */

// Includes all of this file's dependencies. Make sure they're in the right path!
#include <Wire.h>
#include <LSM303.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>


// Enable node handling, letting this file initialize publishers and subscribers.
ros::NodeHandle  nh;
    
// Set variables to be used later format: datatype variablename
int encoder_pin = 8;
int servo1_pin = 9;
int servo2_pin = 10;
int water_pin = A1;
unsigned long duration;
int water_voltage = 0;
char location[2] = { 0 };
uint32_t timer = millis();

std_msgs:: UInt16 wind_msg;      // The encoder outputs a 16 bit unsigned integer for the amount of time the pulse is high
std_msgs:: UInt16 water_msg;    // The water sensor outputs a value for its voltage between 0V and 5V depending on whether the circuit senses water or not.
std_msgs:: UInt16 compass_msg;  // The compass outputs a 16 bit unsigned integer from 0 at North clockwise to 360 if you've calibrated.
std_msgs:: UInt16 servo1_msg;
std_msgs:: UInt16 servo2_msg;
std_msgs:: UInt16MultiArray GPS_msg;

//Instantiate instances of classes
LSM303 compass;
Servo servo1;
Servo servo2;

//GPS setup junk
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean);

// Define callback functions for subscribers (what to do when a new signal comes in)
  // Callback response for servo1
void servo1_cb( const std_msgs::UInt16& cmd_msg1){  //function servo1_cb references the servo command
  servo1.write(cmd_msg1.data); //set servo angle, should be from 0-180
}
  
  // Callback response for servo2  
void servo2_cb( const std_msgs::UInt16& cmd_msg2){   //function servo2_cb requires a UInt16 input
  servo2.write(cmd_msg2.data); //set servo angle, should be from 0-180  
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
  
    
// Define subscribers: <std_msgs::OutputDataType> subscribername("topic_name", callback_function);
        //Each subscriber has a name, is subscribed to a topic with data of a specific type. When it hears something, it runs the callback function with the data as arguments.
  ros::Subscriber<std_msgs::UInt16> sub1("servo1", servo1_cb);
  ros::Subscriber<std_msgs::UInt16> sub2("servo2", servo2_cb);

// define publishers: publishername("topic_name", &message);
  ros::Publisher pub_wind("pwm_duration", &wind_msg);
  ros::Publisher pub_water("leak", &water_msg);
  ros::Publisher pub_GPS("location", &GPS_msg);
  ros::Publisher pub_compass("heading", &compass_msg);


// This is the setup code that runs on startup and calls the functions defined above.
void setup(){

  // Initialize the Arduino as a fake node
  nh.initNode();
  Serial.println("setting up");
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
  
  // Set up GPS
  Serial.begin(115200);  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars. also spit it out
  GPS.begin(9600);  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); 
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);   // Request updates on antenna status, comment out to keep quiet
  useInterrupt(true);

  Serial.begin(9600);


  delay(1000);
  
  // "Advertise" to the node the topics being published 
  nh.advertise(pub_wind);
  nh.advertise(pub_compass);
  nh.advertise(pub_water);
  nh.advertise(pub_GPS);
      
  // Subscribe to the node to the servo control topics
  nh.subscribe(sub1);
  nh.subscribe(sub2);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}


  
// This is the code that runs repeatedly until you shut it down.
void loop(){
  nh.loginfo("spinning");
  Serial.println("spiningg");
  nh.spinOnce();
  Serial.println("done spiningg");
  nh.loginfo("done spinning");

// Collecting the encoder wind signal for relative wind direction
  wind_msg.data = pulseIn(encoder_pin, HIGH);
  
// Collecting water sensor voltage
  water_voltage = analogRead(water_pin);
  if (water_voltage > 1) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }

  Serial.println("water done");
// Collecting compass input vector
  compass.read();
  compass_msg.data = compass.heading((LSM303::vector){0,-1,0});
  
  Serial.println("compass done");
// Collect GPS data
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  Serial.println("GPS read");
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  Serial.println("GPS parsed");
  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer

    location[0] = GPS.latitude;
    location[1] = GPS.longitude;
  }
  Serial.println("timer timed");
  // Publish each new message.
  pub_wind.publish(&wind_msg);
  pub_water.publish(&water_msg);
  pub_compass.publish(&compass_msg);
  pub_GPS.publish(&GPS_msg);
  Serial.println("delaying");
  delay(1);
  Serial.println("delaying");

}

