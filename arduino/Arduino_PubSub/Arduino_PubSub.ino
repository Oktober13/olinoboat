// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <Wire.h>
#include <LSM303.h>
#include <Arduino.h>
#include <Servo.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

/*
LSM303 compass;
Servo servo1;
Servo servo2;
*/
ros::NodeHandle nh;


std_msgs:: UInt16 wind_msg; // The encoder outputs a 16 bit unsigned integer for the amount of time the pulse is high
std_msgs:: UInt16 water_msg; // The water sensor outputs a value for its voltage between 0V and 5V depending on whether the circuit senses water or not.
std_msgs:: UInt16 compass_msg; // The compass outputs a 16 bit unsigned integer from 0 at North clockwise to 360 if you've calibrated.
std_msgs:: UInt16 servo1_msg;
std_msgs:: UInt16 servo2_msg;
std_msgs:: String gps_msg;


int encoder_pin = 8;
int servo1_pin = 9;
int servo2_pin = 10;
int water_pin = A1;
unsigned long duration;
int water_voltage = 0;



// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

/*
// Define callback functions for subscribers (what to do when a new signal comes in)
  // Callback response for servo1
void servo1_cb( const std_msgs::UInt16& cmd_msg1){	//function servo1_cb references the servo command
  servo1.write(cmd_msg1.data); //set servo angle, should be from 0-180
}
 
// Callback response for servo2
void servo2_cb( const std_msgs::UInt16& cmd_msg2){ //function servo2_cb requires a UInt16 input
  servo2.write(cmd_msg2.data); //set servo angle, should be from 0-180
}

ros::Subscriber<std_msgs::UInt16> sub1("servo1", servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("servo2", servo2_cb);
*/
ros::Publisher pub_wind("pwm_duration", &wind_msg);
ros::Publisher pub_water("leak", &water_msg);
ros::Publisher pub_compass("heading", &compass_msg);
ros::Publisher pub_gps("gps", &gps_msg);

void setup()  
{
  nh.initNode();
  
  
  // Set up Arduino hardware pins
  pinMode(encoder_pin, INPUT); // set encoder_pin to input
  pinMode(water_pin, INPUT); // set water_pin to input
  pinMode(13, OUTPUT);
  /*
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

  */
  // "Advertise" to the node the topics being published
  nh.advertise(pub_wind);
  nh.advertise(pub_compass);
  nh.advertise(pub_water);
  nh.advertise(pub_gps);


/*
  // Subscribe to the node to the servo control topics
  nh.subscribe(sub1);
  nh.subscribe(sub2);
*/
  delay(50);
  
    // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(57600);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  //nh.loginfo("serial port set up");
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
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

void loop()                     // run over and over again
{

  
  
  if (GPS.newNMEAreceived()){
    gps_msg.data = GPS.lastNMEA();
    pub_gps.publish(&gps_msg);
    //if (!GPS.parse(GPS.lastNMEA()))
      //return;
  }
  
/*
  pub_wind.publish(&wind_msg);
  pub_water.publish(&water_msg);
  pub_compass.publish(&compass_msg);
  */
  nh.loginfo("about to spin");
  nh.spinOnce();
  nh.loginfo("pants");
  
}
