#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class WatchdogPcNode:
    def __init__(self):

        self.lastMsg = Twist()
        self.timeout= rospy.Duration(0.15)  # 150 milliseconds
        self.timer = rospy.Timer(self.timeout, self.timeout_callback)

        rospy.Subscriber('/taz_set_speed', Twist, self.cmd_callback)

        self.tazPub = rospy.Publisher('/taz_speed', Twist, queue_size=10)
        return


    def cmd_callback(self, msg):
        self.lastMsg = msg

        self.timer.shutdown()
        self.timer = rospy.Timer(self.timeout, self.timeout_callback)

        self.tazPub.publish(msg)
        print("publish NEW")
        return

    def timeout_callback(self, _):
        self.tazPub.publish(self.lastMsg)
        print("publish OLD")
        return


if __name__ == "__main__":

    rospy.init_node('watchdog_pc')
    node = WatchdogPcNode()

    rospy.spin()
