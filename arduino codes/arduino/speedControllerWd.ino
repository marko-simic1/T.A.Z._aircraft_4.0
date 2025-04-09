//Na ROS strani
//roscore
//rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600

//Za slanje poruke:
//rostopic pub /taz_speed geometry_msgs/Twist '{linear: {z: 0.7}}'

#include <ros.h>
#include <geometry_msgs/Twist.h>

const int BIN1 = 9; // PWM pin
const int BIN2 = 8; // opcionalno - smjer

ros::NodeHandle nh;

float dutyC = 0.0;
unsigned long lastMsgTime = 0;
const unsigned long timeout = 300; // milliseconds

void speedCallback(const geometry_msgs::Twist& msg) {
  // Resetiraj watchdog timer
  lastMsgTime = millis();

  // Osiguraj da je vrijednost u [0.0, 1.0]
  dutyC = constrain(msg.linear.z, 0.0, 1.0);
  int pwmValue = dutyC * 255; // mapiraj na 0-255

  analogWrite(BIN1, pwmValue);
  Serial.print("update ");
  Serial.println(pwmValue);
}

ros::Subscriber<geometry_msgs::Twist> sub("taz_speed", &speedCallback);

void setup() {
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT); // ako koristiš smjer, možeš dodati logiku
  analogWrite(BIN1, 0); // start s 0

  Serial.begin(57600); // ili onaj koji koristiš
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();

  // Watchdog logika: ako nema poruke u timeout ms, zaustavi motor
  if (millis() - lastMsgTime > timeout) {
    analogWrite(BIN1, 0);
    dutyC = 0.0;
    lastMsgTime = millis(); // reset da ne spamamo
    Serial.println("duty cycle STOPPED");
  }

  delay(10); // stabilizacija
}
