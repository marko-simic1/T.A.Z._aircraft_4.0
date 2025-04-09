//Pokreni na PC-u:
//roscd rosserial_arduino
//make_libraries.py ~/Arduino/libraries
//Zatim u Arduino IDE-u uključi:

//cpp
//#include <ros.h>
//#include <geometry_msgs/Twist.h>

//Na računalu pokreni:
//roscore
//rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600

#include <ros.h>
#include <geometry_msgs/Twist.h>

const int BIN1 = 9; // PWM pin
const int BIN2 = 8; // optional direction pin

ros::NodeHandle nh;

float dutyC = 0.0;

void cmdCallback(const geometry_msgs::Twist& msg) {
  dutyC = msg.linear.z;

  // Ograniči između 0.0 i 1.0
  if (dutyC < 0.0) dutyC = 0.0;
  if (dutyC > 1.0) dutyC = 1.0;

  int pwmValue = dutyC * 255;
  analogWrite(BIN1, pwmValue);

  // (Optional) obrni smjer ako koristiš BIN2
  // digitalWrite(BIN2, msg.linear.z >= 0 ? HIGH : LOW);
}

ros::Subscriber<geometry_msgs::Twist> sub("taz_speed", &cmdCallback);

void setup() {
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  analogWrite(BIN1, 0); // Start off
  digitalWrite(BIN2, LOW); // Default direction

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10); // mali delay za stabilnost
}
