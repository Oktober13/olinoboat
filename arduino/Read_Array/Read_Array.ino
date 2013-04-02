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

#include <ros.h>
#include <std_msgs/UInt16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"
#include <Wire.h>
#include <LSM303.h>
#include <Arduino.h>


LSM303 compass;


ros::NodeHandle nh;


std_msgs:: UInt16MultiArray compass_calibration;
std_msgs:: UInt16 compass_msg; 

int xmin;
int xmax;
int ymin;
int ymax;
int zmin;
int zmax;



void compass_cb(const std_msgs::UInt16MultiArray& maxmin_array){
  xmin = maxmin_array.data[0];
  xmax = maxmin_array.data[1];
  ymin = maxmin_array.data[2];
  ymax = maxmin_array.data[3];
  zmin = maxmin_array.data[4];
  zmax = maxmin_array.data[5];
  
}


ros::Subscriber<std_msgs::UInt16MultiArray> sub_compass("compass_calibration", compass_cb);
ros::Publisher pub_compass("heading", &compass_msg);

void setup()  
{
  nh.initNode();
  
  nh.spinOnce();
  
  nh.advertise(pub_compass);
  nh.subscribe(sub_compass);

  // Set up Compass
  Wire.begin();
  compass.init();
  compass.enableDefault();
  // Calibration values. Use the Calibrate example program to get the values for
  // your compass.
  compass.m_min.x = xmin; compass.m_min.y = ymin; compass.m_min.z = zmin;
  compass.m_max.x = xmax; compass.m_max.y = ymax; compass.m_max.z = zmax;

  delay(50);
  
}

void loop()                     // run over and over again
{
  
 
  // Collecting compass input vector
  compass.read();
  compass_msg.data = compass.heading((LSM303::vector){0,-1,0});
  
  pub_compass.publish(&compass_msg);
  
  nh.loginfo("about to spin");
  nh.spinOnce();
  nh.loginfo("pants");
  
}
