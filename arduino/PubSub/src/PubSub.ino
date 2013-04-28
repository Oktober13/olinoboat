/*
 * rosserial Servo Control Example
 */

#include <Arduino.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <Wire.h>
#include <LSM303.h>

ros::NodeHandle  nh;
std_msgs:: UInt16 wind_msg;
std_msgs:: UInt16 compass_msg;
std_msgs:: UInt16 xmin;
std_msgs:: UInt16 xmax;
std_msgs:: UInt16 ymin;
std_msgs:: UInt16 ymax;
std_msgs:: UInt16 zmin;
std_msgs:: UInt16 zmax;
std_msgs:: Float64 gps_lat_msg;
std_msgs:: Float64 gps_lon_msg;


SoftwareSerial nss(3, 4);
TinyGPS gps;
LSM303 compass;
Servo rudder_servo;
Servo sail_servo;

int encoder_pin = 8;
int rudder_servo_pin = 9;
int sail_servo_pin = 10;
float flat;
float flon;
unsigned long age;
unsigned long duration;
int water_voltage = 0;
bool newData = false;

void rudder_cb( const std_msgs::UInt16& rudder_msg){
  rudder_servo.write(rudder_msg.data); //set servo angle, should be from 0-180
}
void sail_cb( const std_msgs::UInt16& sail_msg){
  sail_servo.write(sail_msg.data); //set servo angle, should be from 0-180
}void xmin_cb(const std_msgs::UInt16&xmin){
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


ros::Publisher pub_wind("pwm_duration", &wind_msg);
ros::Publisher pub_compass("heading", &compass_msg);
ros::Publisher pub_gps_lat("gps_lat", &gps_lat_msg);
ros::Publisher pub_gps_lon("gps_lon", &gps_lon_msg);
ros::Subscriber<std_msgs::UInt16> rudder_sub("rudder", rudder_cb);
ros::Subscriber<std_msgs::UInt16> sail_sub("sail", sail_cb);
ros::Subscriber<std_msgs::UInt16> sub_xmin("xmin", xmin_cb);
ros::Subscriber<std_msgs::UInt16> sub_xmax("xmax", xmax_cb);
ros::Subscriber<std_msgs::UInt16> sub_ymin("ymin", ymin_cb);
ros::Subscriber<std_msgs::UInt16> sub_ymax("ymax", ymax_cb);
ros::Subscriber<std_msgs::UInt16> sub_zmin("zmin", ymin_cb);
ros::Subscriber<std_msgs::UInt16> sub_zmax("zmax", ymax_cb);

void setup(){

  pinMode(encoder_pin, INPUT);

  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.setTimeout(100);
  compass.m_min.x = -520; compass.m_min.y = -570; compass.m_min.z = -770;
  compass.m_max.x = +540; compass.m_max.y = +500; compass.m_max.z = 180;

  //nss.begin(4800);

  nh.initNode();
  nh.subscribe(rudder_sub);
  nh.subscribe(sail_sub);
  nh.advertise(pub_wind);
  nh.advertise(pub_compass);
  nh.advertise(pub_gps_lat);
  nh.advertise(pub_gps_lon);
  
  rudder_servo.attach(rudder_servo_pin); 
  sail_servo.attach(sail_servo_pin); 
}

void loop(){
  wind_msg.data = pulseIn(encoder_pin, HIGH);
  pub_wind.publish(&wind_msg);

  compass.read();
  compass_msg.data = compass.heading((LSM303::vector){0,-1,0});  
  pub_compass.publish(&compass_msg);

/*  for (unsigned long start = millis(); millis() - start < 1000;){
    while (nss.available()){
      int c = nss.read();
      if (gps.encode(c)){
        newData = true;
      }
    }
  }

*/
/*  
  if (newData){
//    gps.f_get_position(&flat, &flon, &age);
//    flat = TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;
    gps_lat_msg.data = flat;
//    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;
    gps_lon_msg.data = flon; 
    pub_gps_lat.publish(&gps_lat_msg);
    pub_gps_lon.publish(&gps_lon_msg);
    }
*/

  nh.spinOnce();
  delay(1);
}
