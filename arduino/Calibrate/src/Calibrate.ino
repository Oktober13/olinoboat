#include <Wire.h>
#include <LSM303.h>
#include <ros.h>
#include <Servo.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"



LSM303 compass;
LSM303::vector running_min = {2047, 2047, 2047}, running_max = {-2048, -2048, -2048};

Servo rudder_servo;
Servo sail_servo;

int encoder_pin = 8;
int rudder_servo_pin = 9;
int sail_servo_pin = 10;

ros::NodeHandle nh;
std_msgs:: UInt16 xmin;
std_msgs:: UInt16 xmax;
std_msgs:: UInt16 ymin;
std_msgs:: UInt16 ymax;
std_msgs:: UInt16 zmin;
std_msgs:: UInt16 zmax;
std_msgs:: UInt16 wind_msg;
//std_msgs:: UInt16MultiArray maxmin_array;
ros::Publisher pub_xmin("xmin", &xmin);
ros::Publisher pub_xmax("xmax", &xmax);
ros::Publisher pub_ymin("ymin", &ymin);
ros::Publisher pub_ymax("ymax", &ymax);
ros::Publisher pub_zmin("zmin", &ymin);
ros::Publisher pub_zmax("zmax", &ymax);
ros::Publisher pub_wind("pwm_duration", &wind_msg);

void rudder_cb( const std_msgs::UInt16& rudder_msg){
  rudder_servo.write(rudder_msg.data);
}

void sail_cb( const std_msgs::UInt16& sail_msg){ 
  sail_servo.write(sail_msg.data);
}

ros::Subscriber<std_msgs::UInt16> sub_rudder("rudder", rudder_cb);
ros::Subscriber<std_msgs::UInt16> sub_sail("sail", sail_cb);

//ros::Publisher pub_array("compass_calibration", &maxmin_array);



void setup() {
  //nh.initNode();
  Serial.begin(57600);
  Wire.begin();
  compass.init();
  compass.enableDefault();

  pinMode(encoder_pin, INPUT);
  
  /*
  //maxmin_array.layout.dim = (std_msgs::MultiArrayDimension *)
  //maxmin_array.layout.dim_length = 1;
  maxmin_array.layout.dim[0].size = 6;
  maxmin_array.layout.dim[0].stride = 1;
  maxmin_array.layout.data_offset = 0;
  maxmin_array.layout.dim[0].label = "xmin, ymin, zmin, xmax, ymax, zmax";
  maxmin_array.data_length = 6;

  //maxmin_array.data.resize(6);
*/
  nh.advertise(pub_xmin);
  nh.advertise(pub_xmax);
  nh.advertise(pub_ymin);
  nh.advertise(pub_ymax);
  nh.advertise(pub_zmin);
  nh.advertise(pub_zmax);

  nh.advertise(pub_wind);
  //nh.advertise(pub_array);

  // Subscribe to the node to the servo control topics
  nh.subscribe(sub_rudder);
  nh.subscribe(sub_sail);
}

void loop() {  
  compass.read();
/*
  maxmin_array.data[1] = min(running_min.x, compass.m.x);
  maxmin_array.data[2] = min(running_min.y, compass.m.y);
  maxmin_array.data[3] = min(running_min.z, compass.m.z);

  maxmin_array.data[4] = max(running_max.x, compass.m.x);
  maxmin_array.data[5] = max(running_max.y, compass.m.y);
  maxmin_array.data[6] = max(running_max.z, compass.m.z);  
*/ 
  xmin.data = min(running_min.x, compass.m.x);
  ymin.data = min(running_min.y, compass.m.y);
  zmin.data = min(running_min.z, compass.m.z);

  xmax.data = max(running_max.x, compass.m.x);
  ymax.data = max(running_max.y, compass.m.y);
  zmax.data = max(running_max.z, compass.m.z);  


  wind_msg.data = pulseIn(encoder_pin, HIGH);

  pub_xmin.publish(&xmin);
  pub_xmax.publish(&xmax);
  pub_ymin.publish(&ymin);
  pub_ymax.publish(&ymax);
  pub_zmin.publish(&zmin);
  pub_zmax.publish(&zmax);
  pub_wind.publish(&wind_msg);
  
  //pub_array.publish(&maxmin_array);

  nh.spinOnce();
  
  delay(50);
}


