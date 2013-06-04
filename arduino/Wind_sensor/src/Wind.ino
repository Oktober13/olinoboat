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

void loop()
{
  pwm_msg.data = pulseIn(pin, HIGH);
  pwm_msg.data = pwm_msg.data * (359.0/1254.0);
  pub_pwm.publish( &pwm_msg );
  nh.spinOnce();
  delay(1000);
}
