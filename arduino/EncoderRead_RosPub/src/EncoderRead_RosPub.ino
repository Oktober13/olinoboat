/*
 * rosserial PWM reader
 *
 * This sketch demonstrates reading a PWM input
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

//rosserial_arduino:: pwm_msg;
std_msgs:: UInt16 pwm_msg;
ros::Publisher pub_pwm("pwm_duration", &pwm_msg);

int pin = 8;                  //pin 8 is encoder input
unsigned long duration;

void setup(){
  pinMode(pin, INPUT);		
  nh.initNode();
  nh.advertise(pub_pwm);
  
}

void loop(){
  pwm_msg.data = pulseIn(pin, HIGH);
  pub_pwm.publish(&pwm_msg);
  nh.spinOnce();
  delay(1);
}


