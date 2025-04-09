//Na ROS strani (PC/Raspberry Pi) možeš pokrenuti:
//rostopic pub /led_control std_msgs/Bool "data: true"

#include <ros.h>
#include <std_msgs/Bool.h>

const int LED_PIN = 13;

ros::NodeHandle nh;

void ledCallback(const std_msgs::Bool& msg) {
  digitalWrite(LED_PIN, msg.data ? HIGH : LOW);
}

ros::Subscriber<std_msgs::Bool> sub("led_control", &ledCallback);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
