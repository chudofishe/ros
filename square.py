#! /usr/bin/python

# Run 1 turtle in terminal
# rosrun turtlesim turtlesim_node

import rospy
from geometry_msgs.msg import Twist
import time
import math

def move_forward(pub, msg):
    msg.linear.x = 4.0
    msg.angular.z = 0.0
    pub.publish(msg)
    time.sleep(1)

def move_rotate(pub, msg):
    msg.linear.x = 0.0
    msg.angular.z = math.pi / 2
    pub.publish(msg)
    time.sleep(1)

rospy.init_node('name_by_default')

pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
msg = Twist()

# # 1 square 
# for i in range(4):
#     move_forward(pub, msg)
#     move_rotate(pub, msg)

r = rospy.Rate(0.33) #Hz
# square inf
while not rospy.is_shutdown():
    move_forward(pub, msg)
    move_rotate(pub, msg)
    r.sleep() # control iteration cycle

