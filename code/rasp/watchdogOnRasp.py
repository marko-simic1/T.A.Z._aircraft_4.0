#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


class WatchdogRpiNode:
    def __init__(self):

        self.sub = rospy.Subscriber('/taz_topic', Twist, self.taz_callback)
        self.dutyC = 0

        self.timeout= rospy.Duration(0.3)  # 300 milliseconds
        self.timer = rospy.Timer(self.timeout, self.timeout_callback, oneshot=True)

        return

    def taz_callback(self, data):
        self.timer.shutdown()
        self.timer = rospy.Timer(self.timeout, self.timeout_callback, oneshot=True)

        self.dutyC = min( 1, max(0, data.linear.z) )

        print("duty cycle: "+ str(self.dutyC))

        return


    def timeout_callback(self, _):
        self.dutyC = 0
        print("duty cycle STOPPED")
        return
        


if __name__ == "__main__":

    rospy.init_node('watchdog_rpi')
    node = WatchdogRpiNode()

    rospy.spin()
