#! /usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
from math import pow, atan2, sqrt, pi

turtles = list()

# Remove all created turtles in this script (not neccessary with launch file)
def my_handler():
    rospy.wait_for_service('/kill') 
    kill_func = rospy.ServiceProxy('/kill', Kill)
    
    for turtle in turtles:
        kill_func(turtle.name)
        print "\nKill " + turtle.name 


class turtle_chaser:
    def __init__(self, name, coord, speed):
        # We will use it for killing 
        self.name = name
        self.speed = speed
        # Kreate new turtle
        rospy.wait_for_service('/spawn')
        spawn_func = rospy.ServiceProxy('/spawn', Spawn)
        spawn_func(coord[0], coord[1], coord[2], name)
        self.pub = rospy.Publisher(name + '/cmd_vel', Twist, queue_size=10)
        # Update direction of this turtle when turtle 1 is updated
        self.sub = rospy.Subscriber('turtle1/cmd_vel', Twist, self.update_direction)
        # Track the current turle position
        self.sub2 = rospy.Subscriber(name + '/pose', Pose, self.update_position)
        # Track the turtle 1 position
        self.sub3 = rospy.Subscriber('turtle1/pose', Pose, self.update_goal_position)
    
    # Update our class position
    def update_goal_position(self, data):
        self.goal_pose = data
        #rospy.loginfo(data)

    # Update our class position
    def update_position(self, data):
        self.pose = data
        #rospy.loginfo(self.pose)

    # Calculate the direction
    def update_direction(self, msg):
        
        # Rotate
        msg.angular.x = 0
        msg.angular.y = 0
        #msg.angular.z = pi / 2
        msg.angular.z = self.angular_vel(self.goal_pose)        

        # Move
        msg.linear.x = self.linear_vel(self.goal_pose)
        # msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0

        self.pub.publish(msg)

        rospy.loginfo(msg)
    
    # Math calculation of angular and speed
    def euclidean_distance(self, goal_pose):
        # Euclidean distance between current pose and the goal.
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                        pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, private_space=1): 
        """If turtle close enough just dont move"""
        return self.speed if (self.euclidean_distance(goal_pose) > private_space) else 0

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2,private_space=1):
        if (self.euclidean_distance(goal_pose) < 1): # Respect the private space of turtle 1
            return 0
        else:
            return constant * (self.steering_angle(goal_pose) - self.pose.theta)


rospy.init_node('turtles_catch_up')

# Append turtless to list
turtles.append(turtle_chaser(name='leo', coord=(4.0, 4.0, 0.0), speed=0.2))
turtles.append(turtle_chaser(name='edison', coord=(1.0, 4.0, 0.0), speed=4.0))
turtles.append(turtle_chaser(name='mark', coord=(1.0, 1.0, 0.0), speed=1.5))

# on Ctrl+C kill all created turtles
# rospy.on_shutdown(my_handler)

rospy.spin()
