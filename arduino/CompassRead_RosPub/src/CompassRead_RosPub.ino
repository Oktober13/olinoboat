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
#include <ros/time.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

std_msgs:: UInt16 compass_msg;
ros::Publisher pub_compass("heading", &compass_msg);

LSM303 compass;

void setup() {
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.setTimeout(100);

  pinMode(9, INPUT);

  // Calibration values. Use the Calibrate example program to get the values for
  // your compass.
  compass.m_min.x = -520; compass.m_min.y = -570; compass.m_min.z = -770;
  compass.m_max.x = +540; compass.m_max.y = +500; compass.m_max.z = 180;

  nh.initNode();
  nh.advertise(pub_compass);
  Serial.begin(57600);
}
unsigned long timer;

void loop() {
  if ( (millis() - timer) > 50){
    nh.loginfo("read");
   compass.read();
    nh.spinOnce();
//  compass_msg.data = compass.heading((LSM303::vector){0,-1,0});
    pub_compass.publish(&compass_msg);
    timer = millis();
  }
  nh.loginfo("about to spin");
  nh.spinOnce();
  delay(1);
}


