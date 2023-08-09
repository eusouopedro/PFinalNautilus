#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
import math

def convert(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class Listener():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('gazebo/model_states', ModelStates   , self.callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    def callback(self, msg):
        m = ModelState()
        xQ = msg.pose[2].orientation.x
        yQ = msg.pose[2].orientation.y
        zQ = msg.pose[2].orientation.z
        wQ = msg.pose[2].orientation.w
        xE, yE, zE = convert(xQ, yQ, zQ, wQ)
        self.z = zE
        self.y = yE
        self.x = xE
        rospy.Subscriber('sonar_data', PointStamped, self.callback2)
    def callback2(self, msg):
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        velx, vely, velz = 0.5*x, 0.5*y, 0.5*z
        v = Twist()
        v.linear.x = 0#velx
        v.linear.y = 0#vely
        v.linear.z = 0#velz
        v.angular.x = 0
        v.angular.y = 0
        if abs(x) > 0.2:
            v.angular.z = ((math.pi - math.atan(x/y)) - self.z) #(math.atan(abs(x)/abs(y)) - self.z)*100
        else:
             v.angular.z = 0
        rospy.loginfo(f'linear: {v}')
        self.pub.publish(v)

if __name__ == '__main__':
    l = Listener()
    rospy.spin()