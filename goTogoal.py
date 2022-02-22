#!/usr/bin/env python
# !/usr/bin/env python
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        #initialize important parameters such as the turtlebot's pose and orientation
        self.posex = 0
        self.posey = 0
        self.roll = 0
        self.yaw = 0
        self.pitch = 0

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic. self.update_pose is called
        # when a message of type Odometry is received.
        self.pose_subscriber = rospy.Subscriber('/odom',
                                                Odometry,
                                                self.update_pose)
        #the desired target coordinates
        self.goal_posex = -20
        self.goal_posey = -20

        #initialize important variables such as a variable that holds the robot pose information and a variable to hold odometry information
        self.pose = Odometry()
        self.odom = Odometry()

        #publisher rate of 30 Hz --> to be used in loops
        self.rate = rospy.Rate(30)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Odometry is
        received by the subscriber."""
        self.posex = round(data.pose.pose.position.x, 4)
        self.posey = round(data.pose.pose.position.y, 4)

        # get the robot heading from the odometry --> robot's heading is used for the unicyle dynamics, not angle btwn robot and target
        self.orient_quat = data.pose.pose.orientation
        self.orient = [self.orient_quat.x, self.orient_quat.y, self.orient_quat.z, self.orient_quat.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orient)
        #print('x position: ' + str(self.posex) + ' y position :' + str(self.posey) + " yaw " + str(self.yaw) + "\n")

    def distance(self):
        """ X distance, Y distance and Euclidean distance between the robot and the goal"""
        xdist = self.goal_posex - self.posex
        ydist = self.goal_posey - self.posey
        eucdist = sqrt(pow((self.goal_posex - self.posex), 2) +
                       pow((self.goal_posey - self.posey), 2))

        posedist = [xdist, ydist, eucdist]

        return posedist

    def single_Iterate(self, k=0.1):
        """Single Integrator model"""
        posedist = self.distance()
        xdist = posedist[0]
        ydist = posedist[1]
        ux = xdist * k
        uy = ydist * k

        return [[ux], [uy]]

    def unicycle_Steer(self, k=0.1, l=1):
        """ computes the linear and angular velocities using unicycle dynamics"""
        #note that the controller is very sensitive to proportional gain. Choose small values for k (K <= 0.3) for less overshoot.
        A = [[math.cos(self.yaw), math.sin(self.yaw)],
             [(-math.sin(self.yaw)) / l, math.cos(self.yaw) / l]]
        vel = k * np.dot(A, self.single_Iterate())
        # vel should be a 2x1 array. [0][0] should be the linear velocity and [1][0] should be the angular velocity

        return vel
    def move2goal(self):
        """Moves the turtlebot to the goal."""
        vel_msg = Twist()

        #check the euclidean distance against a tolerance.
        while self.distance()[2] >= 0.5:

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.unicycle_Steer()[0]
            #print("linear x vel " + str(vel_msg.linear.x))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.unicycle_Steer()[1]

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.move2goal()

    except rospy.ROSInterruptException:
        pass
