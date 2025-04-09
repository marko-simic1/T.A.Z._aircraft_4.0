#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
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

        #pwm
        self.dutyC = 0
        self.pwm = GPIO.PWM(BIN1, 100)
        self.pwm.start(0)

        #watchdog
        self.timeout= rospy.Duration(0.3)  # 300 milliseconds
        self.timer = rospy.Timer(self.timeout, self.timeout_callback)  
        # ako se doda argument oneshot=true, salje samo jedan signal za gasenje

        return


    def speed_callback(self, data):

        # resetiraj watchdog
        self.timer.shutdown()
        self.timer = rospy.Timer(self.timeout, self.timeout_callback)

        # dutyCycle -> [0,100]
        self.dutyC = max( 0, min(data.linear.z, 1.0) )
        self.dutyC = self.dutyC * 100

        self.pwm.ChangeDutyCycle(self.dutyC)
        print('update '+ str(self.dutyC))

        return

    
    def timeout_callback(self, _):   # ugasi pwm kad ne prima podatke
        self.dutyC = 0
        self.pwm.ChangeDutyCycle(self.dutyC)
        print("duty cycle STOPPED")
        return


    def cleanup_gpio(self):
        self.pwm.stop()
        GPIO.cleanup()
        return


if __name__ == "__main__":
    setup()

    rospy.init_node('taz_speed_controller_wd')
    node = SpeedController()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        node.cleanup_gpio()

