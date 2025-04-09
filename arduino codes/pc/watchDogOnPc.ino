#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;
geometry_msgs::Twist lastMsg;

unsigned long lastMsgTime = 0;
const unsigned long timeout = 150;

void twistCallback(const geometry_msgs::Twist& msg) {
  lastMsg = msg; // automatski parsiran
  lastMsgTime = millis();
  // upravljanje motorima na temelju msg.linear.z, msg.angular.z itd.
}

ros::Subscriber<geometry_msgs::Twist> sub("taz_speed", &twistCallback);

void setup() {
  pinMode(9, OUTPUT); // PWM output
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();

  if (millis() - lastMsgTime > timeout) {
    // koristi lastMsg za watchdog objavu
    // npr. analogWrite na temelju lastMsg.linear.z
  }

  delay(10); // mali delay za stabilnost
}
