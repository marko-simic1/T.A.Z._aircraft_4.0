#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

BIN1 = 22
BIN2 = 23

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BIN1, GPIO.OUT)
    GPIO.setup(BIN2, GPIO.OUT)
    return

class SpeedController:
    def __init__(self):
        self.sub = rospy.Subscriber('taz_speed', Twist, self.speed_callback)
        self.dutyC = 0
        self.pwm = None

    def speed_callback(self, data):

        self.dutyC = data.linear.z

        self.dutyC = min(self.dutyC, 1.0)
        self.dutyC = max(self.dutyC, 0.0)

        self.update_motor_power()

    def update_motor_power(self):
        # Stop existing PWM before creating a new one
        if self.pwm:
            self.pwm.stop()

        # Initialize new PWM
        self.pwm = GPIO.PWM(BIN1, 100)
        self.pwm.start(self.dutyC * 100)
        print('update '+ str(self.dutyC))

    def cleanup_gpio(self):
        # Stop PWM before cleanup
        if self.pwm:
            self.pwm.stop()

        GPIO.cleanup()

if __name__ == "__main__":
    setup()

    rospy.init_node('taz_speed_controller')
    node = SpeedController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        node.cleanup_gpio()

