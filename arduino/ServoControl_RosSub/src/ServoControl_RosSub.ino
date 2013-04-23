/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

//#include <WProgram.h>

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;

void servo1_cb( const std_msgs::UInt16& cmd_msg){
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}
void servo2_cb( const std_msgs::UInt16& cmd_msg){
  servo2.write(cmd_msg.data); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<std_msgs::UInt16> sub1("rudder_servo", servo1_cb);
ros::Subscriber<std_msgs::UInt16> sub2("sail_servo", servo2_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10); //attach it to pin 10
}

void loop(){
  nh.spinOnce();
  delay(1);
}
