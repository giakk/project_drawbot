#! /usr/bin/env python3

import rospy
import csv
from drawbot.msg import Data_arduino
from drawbot.msg import Custom_odometry
import time
import numpy as np

"""
takes the data sent from the arduino node "complete_motor" through the topic "data_arduino" and computes the odometry of the robot, allowing to determine its position in terms of coordinates x,y and theta which are gonna be transmitted through the topic "send_odometry"
"""
 

#INITIALIZATION VARIABLES

#internal variables initialization
wheel_rad = 0.0325 				#wheel radius [m]
wheel_dist = 0.178 				#length between the center of both wheels [m]
init_node_time = time.time() 	#takes note of the time [s]
previous_time = init_node_time 	#previous time (needed to compute the elapsed time) [s]
current_time = 0

#output initialization
x = 0								#actual position x [m]
y = 0								#actual position y [m]
theta = 0							#actual angle theta [rad]


def callback(data):
    
    global previous_time
    global x,y,theta
    
    odom=Custom_odometry()								#Defining an object of the type custom_odometry
    rpm_speed_r = data.mesu_speed_r						#Retrieve speed left motor from encoders [rpm]
    rpm_speed_l = data.mesu_speed_l						#Retrieve speed right motor from encoders [rpm]
    lin_wheel_r = 2*np.pi*wheel_rad*rpm_speed_r/60		#find the linear speed of the right wheel [m/s]
    lin_wheel_l = 2*np.pi*wheel_rad*rpm_speed_l/60		#find the linear speed of the left wheel [m/s]
    vx = (lin_wheel_r+lin_wheel_l)/2					#get the linear speed vx of the robot [m/s]
    vy = 0												#non holonomous robot -> vy=0
    vtheta = (lin_wheel_r-lin_wheel_l)/wheel_dist		#get the angular speed of the robot [rad/s]
    
    current_time = time.time()							
    dt = current_time-previous_time						#elapsed time [s]
    previous_time = current_time
    
    x = x+(vx*np.cos(theta)-vy*np.sin(theta))*dt		#compute the new actual position x [m]
    y = y+(vx*np.sin(theta)+vy*np.cos(theta))*dt		#compute the new actual position y [m]
    theta = theta+vtheta*dt								#compute the new actual angle theta [rad]
   
    print(time.time()-init_node_time,x,y,theta)			#just for visualization purposes, prints  (time, x, y, angle)
   
    odom.x_actual = x
    odom.y_actual = y
    odom.theta_actual = theta
    
    pub.publish(odom)									#publish the stored values on the topic "send_odometry"
	


#ROS

rospy.init_node('localisation_odometry')	#define a node called localisation_odometry

rospy.Subscriber('data_arduino', Data_arduino, callback)	#set this node as a subscriber to "data_arduino" of the type Data_arduino

pub = rospy.Publisher('send_odometry', Custom_odometry , queue_size=1)	#set this node as publisher which publishes on the topic "send_odometry" of the type Custom_odometry 

rospy.spin()

