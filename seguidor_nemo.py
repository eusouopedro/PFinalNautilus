#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped

class Listener:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('sonar_data', PointStamped, self.callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    def callback(self, msg):
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        velx, vely, velz = 0.5*x, 0.5*y, 0.5*z
        v = Twist()
        v.linear.x = -(velx)
        v.linear.y = -(vely)
        v.linear.z = -(velz)
        v.angular.x = 0
        v.angular.y = 0
        v.angular.z = 0
        rospy.loginfo(f'linear: {v}')
        self.pub.publish(v)

if __name__ == '__main__':
    l = Listener()
    rospy.spin()