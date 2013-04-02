#include <Arduino.h>
void setup();
void loop();
#line 1 "src/CompassRead_RosPub.ino"
/*
 * rosserial Compass reader
 *
 * This sketch demonstrates reading a  input
 * using ROS and the arduiono
 *
 * For more information on the Arduino source code
 * Checkout :
 * https://github.com/pololu/LSM303
 */

#include <Wire.h>
#include <LSM303.h>
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

std_msgs:: UInt16 compass_msg;
ros::Publisher pub_compass("heading", &compass_msg);

LSM303 compass;

void setup() {
  nh.initNode();
  nh.advertise(pub_compass);
  Wire.begin();
  compass.init();
  compass.enableDefault();
  
  // Calibration values. Use the Calibrate example program to get the values for
  // your compass.
  compass.m_min.x = -520; compass.m_min.y = -570; compass.m_min.z = -770;
  compass.m_max.x = +540; compass.m_max.y = +500; compass.m_max.z = 180;
}

void loop() {
  compass.read();
  compass_msg.data = compass.heading((LSM303::vector){0,-1,0});
  pub_compass.publish(&compass_msg);
  nh.spinOnce();
  delay(1);
}
