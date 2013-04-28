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

#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <Wire.h>
#include <LSM303.h>
#include <Arduino.h>
#include <Servo.h>

SoftwareSerial nss(3, 4);
TinyGPS gps;


LSM303 compass;
Servo rudder_servo;
Servo sail_servo;

ros::NodeHandle nh;

std_msgs:: UInt16 wind_msg; // The encoder outputs a 16 bit unsigned integer for the amount of time the pulse is high
std_msgs:: UInt16 water_msg; // The water sensor outputs a value for its voltage between 0V and 5V depending on whether the circuit senses water or not.
std_msgs:: UInt16 compass_msg; // The compass outputs a 16 bit unsigned integer from 0 at North clockwise to 360 if you've calibrated.
std_msgs:: Float64 gps_lat_msg;
std_msgs:: Float64 gps_lon_msg;
std_msgs:: UInt16 xmin;
std_msgs:: UInt16 xmax;
std_msgs:: UInt16 ymin;
std_msgs:: UInt16 ymax;
std_msgs:: UInt16 zmin;
std_msgs:: UInt16 zmax;



float flat;
float flon;
unsigned long age;
int encoder_pin = 8;
int rudder_servo_pin = 9;
int sail_servo_pin = 10;
int water_pin = A1;
unsigned long duration;
int water_voltage = 0;
bool newData = false;


// Define callback functions for subscribers (what to do when a new signal comes in)
  // Callback response for rudder servo
void rudder_cb( const std_msgs::UInt16& rudder_msg){	//function rudder_servo_cb references the servo command
  rudder_servo.write(rudder_msg.data); //set servo angle, should be from 0-180
}
 
// Callback response for sail servo
void sail_cb( const std_msgs::UInt16& sail_msg){ //function sail_servo_cb requires a UInt16 input
  sail_servo.write(sail_msg.data); //set servo angle, should be from 0-180
}
/*
void xmin_cb(const std_msgs::UInt16&xmin){
  compass.m_min.x =(xmin.data);
}
void ymin_cb(const std_msgs::UInt16&ymin){
  compass.m_min.y =(ymin.data);
}
void zmin_cb(const std_msgs::UInt16&zmin){
  compass.m_min.z =(zmin.data);
}
void xmax_cb(const std_msgs::UInt16&xmax){
  compass.m_max.x =(xmax.data);
}
void ymax_cb(const std_msgs::UInt16&ymax){
  compass.m_max.y =(ymax.data);
}
void zmax_cb(const std_msgs::UInt16&zmax){
  compass.m_max.z =(zmax.data);
}
*/
ros::Subscriber<std_msgs::UInt16> sub_rudder("rudder", rudder_cb);
ros::Subscriber<std_msgs::UInt16> sub_sail("sail", sail_cb);
/*
ros::Subscriber<std_msgs::UInt16> sub_xmin("xmin", xmin_cb);
ros::Subscriber<std_msgs::UInt16> sub_xmax("xmax", xmax_cb);
ros::Subscriber<std_msgs::UInt16> sub_ymin("ymin", ymin_cb);
ros::Subscriber<std_msgs::UInt16> sub_ymax("ymax", ymax_cb);
ros::Subscriber<std_msgs::UInt16> sub_zmin("zmin", ymin_cb);
ros::Subscriber<std_msgs::UInt16> sub_zmax("zmax", ymax_cb);
*/
ros::Publisher pub_wind("pwm_duration", &wind_msg);
ros::Publisher pub_water("leak", &water_msg);
ros::Publisher pub_compass("heading", &compass_msg);
ros::Publisher pub_gps_lat("gps_lat", &gps_lat_msg);
ros::Publisher pub_gps_lon("gps_lon", &gps_lon_msg);

void setup()  
{
  // Set up Arduino hardware pins
  pinMode(water_pin, INPUT); // set water_pin to input
  pinMode(13, OUTPUT);
/*
  rudder_servo.attach(rudder_servo_pin);
  sail_servo.attach(sail_servo_pin);
*/
  // Set up GPS
//  nss.begin(4800);

  // Set up Encoder
  pinMode(encoder_pin, INPUT);


  // Set up Compass
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.setTimeout(100);

  // Default Calibration values. Use the Calibrate example program to get the actual values for
  // your compass.
  compass.m_min.x = -520; compass.m_min.y = -570; compass.m_min.z = -770;
  compass.m_max.x = +540; compass.m_max.y = +500; compass.m_max.z = 180;

  
  // "Advertise" to the node the topics being published
  nh.initNode();
  nh.advertise(pub_wind);
  nh.advertise(pub_compass);
//  nh.advertise(pub_water);
  nh.advertise(pub_gps_lat);
  nh.advertise(pub_gps_lon);


  // Subscribe to the node to the servo control topics
  nh.subscribe(sub_rudder);
  nh.subscribe(sub_sail);

/*  nh.subscribe(sub_xmin);
  nh.subscribe(sub_ymin);
  nh.subscribe(sub_zmin);
  nh.subscribe(sub_xmax);
  nh.subscribe(sub_ymax);
  nh.subscribe(sub_zmax);
*/
  Serial.begin(57600);
  delay(50);
}
unsigned long timer;




void loop()
{
/*  for (unsigned long start = millis(); millis() - start < 1000;)
  {;
    while (nss.available())
    {
      int c = nss.read();
      if (gps.encode(c))
      {
        newData = true;
      }
    }
  }

*/
//	  nh.spinOnce();
/*  
  if (newData){
//    gps.f_get_position(&flat, &flon, &age);
    flat = 0.5;
    flon = 1.3;
//    flat = TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    gps_lat_msg.data = flat;
//    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    gps_lon_msg.data = flon; 
    pub_gps_lat.publish(&gps_lat_msg);
    pub_gps_lon.publish(&gps_lon_msg);
    }
*/
//   // Collecting the encoder PWM signal for relative wind direction
   wind_msg.data = pulseIn(encoder_pin, HIGH);
  
//   // Collecting water sensor voltage
  // water_msg.data = analogRead(water_pin);

//   // Collecting compass input vector
  compass.read();
  compass_msg.data = compass.heading((LSM303::vector){0,-1,0});  
 
  pub_wind.publish(&wind_msg);
//   pub_water.publish(&water_msg);
  pub_compass.publish(&compass_msg);

  nh.loginfo("alaMode: Completed one loop on the alaMode, about to spin again");


  nh.spinOnce();
  delay(10);
  
}
