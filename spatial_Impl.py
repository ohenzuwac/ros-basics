#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion

#figure out drift
# --> decision due to drift was caused by setting the k_prime to a large number (4), maybe it was making the robot too sensitive to disturbances? --> not sure if this is actually causing the drift however
#figure out how to plot in Gazebo
#check integrator dynamics vs control input


class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        #initialize important parameters such as the turtlebot's pose and orientation
        self.posex = 0
        self.posey = 0
        self.roll = 0
        self.theta = 0
        self.pitch = 0

        # Inital conditions for the algorithim = I0 = [z10, z20, B0, theta0, x0, y0]
        self.z10 = 0.3
        self.z20 = 0.1
        self.B0 = 4
        self.theta = np.pi / 2
        # x0 and y0 are defined above as (0,0)

        #Target coordinates

        #some issue with coordinate translation into Gazebo???
        self.yt1 = 10;  # x coordinate of target 1
        self.xt1 = 8;  # y coordinate of target 1
        self.yt2 = -10;  # x coordinate of target 2
        self.xt2 = 5;  # y coordinate of target 2

        #Inital Gain/misc. parameter values
        self.g = 1 # distance gain #less sensitive to this in Gazebo than in python sim
        self.u = 1 # attention
        self.d = 1;  # damping coefficient
        self.k = 1;  # constant
        self.kp = 4;  # constant (K_prime) #this gain has to be fairly high to see a decision from the robot


        #counter
        self.count = 0

        #time step/sample size
        self.h = 0.1

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic. self.update_pose is called
        # when a message of type Odometry is received.
        self.pose_subscriber = rospy.Subscriber('/odom',
                                                Odometry,
                                                self.update_pose)

        #initialize important variables such as a variable that holds the robot pose information and a variable to hold odometry information
        self.pose = Odometry()
        self.odom = Odometry()

        #publisher rate of 30 Hz --> to be used in loops
        self.rate = rospy.Rate(30)


    def update_pose(self, data):
        """Callback function which is called when a new message of type Odometry is
        received by the subscriber."""

        #update the robot's pose information from the Odometry topic
        self.posex = round(data.pose.pose.position.x, 4)
        self.posey = round(data.pose.pose.position.y, 4)

        #get the robot's orientation by transforming quaternion to euler angles
        self.orient_quat = data.pose.pose.orientation
        self.orient = [self.orient_quat.x, self.orient_quat.y, self.orient_quat.z, self.orient_quat.w]
        (self.roll, self.pitch, self.theta) = euler_from_quaternion(self.orient)
        print('x position: ' + str(self.posex) + ' y position :' + str(self.posey) + " yaw " + str(self.theta) + "\n")


    def distance(self): #might be obsolete
        """ X distance, Y distance and Euclidean distance between the robot and the goal"""

        xdist = self.goal_posex - self.posex
        ydist = self.goal_posey - self.posey
        eucdist = sqrt(pow((self.goal_posex - self.posex), 2) +
                       pow((self.goal_posey - self.posey), 2))

        posedist = [xdist, ydist, eucdist]

        return posedist

    def controlComp(self):
        """Computes and applies control input"""
        vel_msg = Twist()
        self.t = np.arange(0, 500, self.h)
        self.B = np.zeros(self.t.size)
        # no need for a vector for x and y --> get this from the odometry
        self.z1 = np.zeros(self.t.size)
        self.z2 = np.zeros(self.t.size)
        self.B[0] = self.B0
        self.z1[0] = self.z10
        self.z2[0] = self.z20

        #might place update statement in here as well

        for i in np.arange(0, self.t.size - 1, 1):

            #Update the Beta and score parameters
            self.B[i + 1] = self.B[i] + self.h * self.updateConnection(self.posex, self.posey, self.B[i]);

            self.z1[i + 1] = self.z1[i] + self.h * self.updateScore1(self.z1[i], self.z2[i], self.B[i], self.posex, self.posey);
            self.z2[i + 1] = self.z2[i] + self.h * self.updateScore2(self.z1[i], self.z2[i], self.B[i], self.posex, self.posey);
            print('target 2 score', self.z2[i])
            print('target 1 score', self.z1[i])

            #print(self.z2[i] + self.h * self.updateScore2(self.z1[i], self.z2[i], self.B[i], self.posex, self.posey))



            #find the new velocities and headings and apply as control inputs

            # Linear velocity in the x-axis and y-axis.
            vel_msg.linear.x = self.updatePosi(self.theta)[0]
            print("linear x vel " + str(vel_msg.linear.x))
            vel_msg.linear.y = self.updatePosi(self.theta)[1]
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.updateHeading(self.z1[i],self.z2[i],self.posex,self.posey,self.theta)
            print("angular z vel " + str(vel_msg.angular.z))

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

            #update to the next time step
            self.count= self.count + 1

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()


    def updateConnection(self,x, y, B):
        """Update Beta accordingly. Beta is a measure of the strength of the connection between targets. Basically, can the robot distinguish between the targets?"""

        self.phi = np.array([math.atan2((self.yt1 - y), (self.xt1 - x)), math.atan2((self.yt2 - y), (self.xt2 - x))]);
        tau = 2;  # timescale

        return -B + self.kp * np.cos(self.k * (self.phi[0] - self.phi[1])) / tau

    def updateScore1(self,z1, z2, B, x, y):
        """function to update z1, the score of target 1"""

        r = np.array(
            [np.sqrt(np.square(self.xt1 - x) + np.square((self.yt1 - y))), np.sqrt(np.square(self.xt2 - x) + np.square(self.yt2 - y))])
        b = self.g / np.square(r[0])

        return -self.d * z1 + self.u * np.sum(np.tanh(B * z2)) + b

    def updateScore2(self,z1, z2, B, x, y):
        """function to update z2, the score of target 2"""

        r = np.array(
            [np.sqrt(np.square(self.xt1 - x) + np.square((self.yt1 - y))), np.sqrt(np.square(self.xt2 - x) + np.square(self.yt2 - y))])
        b = self.g / np.square(r[1])

        #print('z2 instanst value',-self.d * z2 + self.u * np.sum(np.tanh(B * z1)) + b) #nonzero

        return -self.d * z2 + self.u * np.sum(np.tanh(B * z1)) + b

    def updatePosi(self,theta):
        """function to update the linear velocities"""

        #noise = np.random.normal()
        noise = 0
        v = 0.3

        #returns array where [0][0] is the x linear velocity and [0][1] is the y linear velocity
        return np.array([v * np.cos(theta) + 0.05 * noise,
                         v * np.sin(theta) + 0.05 * noise])

    def updateHeading(self,z1, z2, x, y, theta):
        """function to update the angular velocity"""

        self.phi = np.array([math.atan2((self.yt1 - y), (self.xt1 - x)), math.atan2((self.yt2 - y), (self.xt2 - x))]);
        #print('phi',self.phi[0])
        tau = 2;
        #noise = np.random.normal()
        noise = 0

        return np.sum(np.array([np.max(np.array([z1, 0])) * np.sin((self.phi[0] - theta) / 2) + 0.05 * noise,
                                np.max(np.array([z2, 0])) * np.sin((self.phi[1] - theta) / 2) + 0.05 * noise])) / tau;

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.controlComp()

    except rospy.ROSInterruptException:
        pass