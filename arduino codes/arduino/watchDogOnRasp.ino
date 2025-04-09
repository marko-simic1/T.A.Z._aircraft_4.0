//Test s ROS strane:
//roscore
//rosrun rosserial_python serial_node.py /dev/ttyUSB0
//rostopic pub /taz_topic geometry_msgs/Twist '{linear: {z: 0.7}}'

#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

float dutyC = 0.0;
unsigned long lastMsgTime = 0;
const unsigned long timeout = 300; // milliseconds

void tazCallback(const geometry_msgs::Twist& msg) {
  lastMsgTime = millis(); // resetiraj watchdog
  dutyC = constrain(msg.linear.z, 0.0, 1.0);

  Serial.print("duty cycle: ");
  Serial.println(dutyC);
}

ros::Subscriber<geometry_msgs::Twist> sub("taz_topic", &tazCallback);

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();

  if (millis() - lastMsgTime > timeout) {
    if (dutyC > 0.0) {
      dutyC = 0.0;
      Serial.println("duty cycle STOPPED");
    }
    lastMsgTime = millis(); // resetiraj timer da ne ponavlja stalno
  }

  delay(10); // stabilizacija petlje
}
