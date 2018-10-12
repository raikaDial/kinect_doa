#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

Servo kinect_servo;
uint16_t target = 1475;
uint16_t current = target;

// Actual range is 240 degrees, so about 7.7 us = 1 deg
void servoCallback(const std_msgs::Float32 & msg) {
  target = (uint16_t)(1475 + 925*(msg.data)/125);
  //kinect_servo.writeMicroseconds(targ
}

ros::Subscriber<std_msgs::Float32> sub("servo_angle", servoCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  kinect_servo.attach(9);
  kinect_servo.writeMicroseconds(target);//current);
}

double millis_last_write = 0;
double servo_step = 10;
double time_step_millis = 25;

void loop() {
  nh.spinOnce();
  if(((millis() - millis_last_write) > time_step_millis) && (current != target)) {
    if(current < target) {
      current += servo_step;
      if(current > target) {
        current = target;
      }
    }
    else if(current > target) {
      current -= servo_step;
      if(current < target) {
        current = target;
      }   
    }
    kinect_servo.writeMicroseconds(current);
    millis_last_write = millis();
  }
}
