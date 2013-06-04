/*
 * rosserial Servo Control Example
 */

#include <Arduino.h>

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

//Servo rudder_servo;
Servo sail_servo;

//void rudder_cb( const std_msgs::UInt16& rudder_msg){
// rudder_servo.write(rudder_msg.data); //set servo angle, should be from 0-180
//}
void sail_cb( const std_msgs::UInt16& sail_msg){
  sail_servo.write(sail_msg.data); //set servo angle, should be from 0-180
}


//ros::Subscriber<std_msgs::UInt16> rudder_sub("rudder", rudder_cb);
ros::Subscriber<std_msgs::UInt16> sail_sub("sail", sail_cb);

void setup(){
  nh.initNode();
 // nh.subscribe(rudder_sub);
  nh.subscribe(sail_sub);
  
 // rudder_servo.attach(9); //attach it to pin 9
  sail_servo.attach(10); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
