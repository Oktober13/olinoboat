/*
 * Subscribes servos and published from encoder
 */

#include <Arduino.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo sail_servo;

// int switchvalue  = 0; // Commented out code is for a RC transmitter switch, could be uncommented if we get switch capabilities
// int sailvalue = 0;
// int sail_pin = 10;
// int switch_pin = 11;

void sail_cb( const std_msgs::UInt16& sail_msg){
//  switchvalue = pulseIn(switch_pin, HIGH);
  
//  if (switchvalue > 800) {  // COME BACK AND CHECK SWITCHVALUE
//    sailvalue = pulseIn(sail_pin, HIGH);
//    sailvalue = (sailvalue-500) * (180/1000); //COME BACK AND CHECK SAILVALUE
//    sail_servo.write(sailvalue); 
//  } 
//  else {
   sail_servo.write(sail_msg.data); //set servo angle, should be from 0-180
//  }
}

ros::Subscriber<std_msgs::UInt16> sail_sub("sail", sail_cb);   // Sets up the servo subscriber

std_msgs:: UInt16 pwm_msg;    // Sets the data type of the encoder data variable
ros::Publisher pub_pwm("pwm_duration", &pwm_msg);   // Sets up encoder publisher

int wind_pin = 8; //pin 8 is encoder input

void setup(){
  nh.initNode();
  nh.subscribe(sail_sub);
  
  sail_servo.attach(9); //attach sail to pin 9

  pinMode(wind_pin, INPUT);
//   pinMode(sail_pin, INPUT);
//   pinMode(switch_pin, INPUT);
  nh.advertise(pub_pwm); 
}

void loop(){
  pwm_msg.data = pulseIn(wind_pin, HIGH);    // Reads new encoder data
  pwm_msg.data = pwm_msg.data * (359.0/1254.0);    // Converts incoming 0-1254 data as 0-360 degrees
  pub_pwm.publish( &pwm_msg );    // Publishes encoder data
  nh.spinOnce();
  delay(10);
}
