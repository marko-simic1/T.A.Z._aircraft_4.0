#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

const uint8_t PIN_MOTOR_1 = 3;
const uint8_t PIN_MOTOR_2 = 5;
const uint8_t PIN_MOTOR_3 = 6;

Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1();

uint8_t joyX = 128, joyY = 128, thrust = 0;
float kutBr = 0, currentYaw = 0;
bool btn_1 = false, btn_2 = false, btn_3 = false;

String buffer;

void parseLine(const String &line) {
  int idx;
  if ((idx = line.indexOf("x:")) >= 0) joyX   = line.substring(idx+2).toInt();
  if ((idx = line.indexOf("y:")) >= 0) joyY   = line.substring(idx+2).toInt();
  if ((idx = line.indexOf("z:")) >= 0) thrust = line.substring(idx+2).toInt();
  if ((idx = line.indexOf("a:")) >= 0) kutBr  = line.substring(idx+2).toFloat();
  if ((idx = line.indexOf("b:")) >= 0) btn_1  = line.substring(idx+2, idx+3).toInt() != 0;
  if ((idx = line.indexOf("c:")) >= 0) btn_2  = line.substring(idx+2, idx+3).toInt() != 0;
  if ((idx = line.indexOf("d:")) >= 0) btn_3  = line.substring(idx+2, idx+3).toInt() != 0;
}

void updateMotors() {
  float x = (int)joyX - 128;
  float y = (int)joyY - 128;
  float z = thrust;

  float fi = atan2(y, x);
  float r  = sqrt(x*x + y*y);
  if (r > z/2.0) r = z/2.0;

  float speeds[3] = { z, z, z };

  if (r > 0.0) {
    float beta = fi - PI/2.0 + PI/6.0;
    float alfa = fi - PI/2.0 - PI/6.0;
    float s    = sqrt(z*z - z*r - 0.5*r*r);

    if (alfa < -PI && beta < -PI) {
      if (currentYaw > alfa + 2*PI && currentYaw < beta + 2*PI) {
        speeds[0] += r; speeds[1] = s; speeds[2] = s;
      }
    }
    else if (alfa < -PI) {
      if (currentYaw < beta || currentYaw > alfa + 2*PI) {
        speeds[0] += r; speeds[1] = s; speeds[2] = s;
      }
    }
    else {
      if (currentYaw < fi - 1.047 && currentYaw > fi - 2.094) {
        speeds[0] += r; speeds[1] = s; speeds[2] = s;
      }
    }
  }

  for (int i = 0; i < 3; i++) {
    int v = int(speeds[i]);
    v = constrain(v, 0, 255);
    analogWrite(i==0 ? PIN_MOTOR_1 : i==1 ? PIN_MOTOR_2 : PIN_MOTOR_3, v);
  }

  Serial.print("Yaw [deg]: ");
  Serial.print(currentYaw * 180.0/PI, 1);
  Serial.print(" | Motors [1,2,3]: ");
  Serial.print(int(speeds[0])); Serial.print(", ");
  Serial.print(int(speeds[1])); Serial.print(", ");
  Serial.print(int(speeds[2]));
  Serial.print(" | Buttons: ");
  Serial.print(btn_1); Serial.print(", ");
  Serial.print(btn_2); Serial.print(", ");
  Serial.println(btn_3);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);
  pinMode(PIN_MOTOR_3, OUTPUT);
  analogWrite(PIN_MOTOR_1, 0);
  analogWrite(PIN_MOTOR_2, 0);
  analogWrite(PIN_MOTOR_3, 0);

  if (!imu.begin()) {
    Serial.println("Error: LSM9DS1TR not found");
    while (true) delay(10);
  }
  imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
  imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);

  // IMU initialization complete, beginning flight control loop
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      parseLine(buffer);
      buffer = "";
    } else if (c != '\r') {
      buffer += c;
    }
  }

  imu.read();
  currentYaw = imu.getEvent().orientation.x * DEG_TO_RAD;

  updateMotors();
  delay(20);  // control loop at ~50 Hz
}
