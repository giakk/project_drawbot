#! /usr/bin/env python3

import rospy
import numpy
from math import sqrt
from math import pi
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from drawbot.msg import Custom_odometry
from drawbot.msg import Custom_references
from drawbot.msg import Custom_interface

"""
Receives the measured data from the topic "send_odometry" and compare them with the goal variables obtained from the topic "send_goal", providing as output (as a custom message through the topic "send_references") the ref angle and position (theta_ref, x_ref)

theta_ref = how much to rotate before to go forward
x_ref = how much to go forward before to reach the goal position
"""


#INITIALIZATION VARIABLES

#output initialization
x_ref = 0
theta_ref = 0

#input initialization, these coordinates should be provided by the topic "send_goal"
x_goal=0 
y_goal=0



#CALLBACK FUNCTIONS

#callback funtion for the subscriber to "send_goal"
def callback(msg):
	
	global x_goal, y_goal
	
	x_goal = msg.x			#taking the x_goal coordinate from the topic
	y_goal = msg.y			#taking the y_goal coordinate from the topic
	
	print(x_goal, y_goal)	#print of the goal coordinates for visualization purposes	
	
	
#callback function for the subscriber to "send_odometry"
def math_tmp(msg):

	global x_goal, y_goal
	
	Ref=Custom_references()							#variable of the customized type, it'll use to send the needed data through the topic "send_references"
	
	x_act=msg.x_actual								#actual position x [m]
	y_act=msg.y_actual								#actual position y [m]
	theta_act=msg.theta_actual 						#actual position theta [rad]
	
	x_todo=x_goal-x_act 							#x distance from the actual goal
	y_todo=y_goal-y_act								#y distance from the actual goal
	
	theta_total_rad=numpy.arctan2(y_todo,x_todo) 	#total angle to do [rad]
	theta_todo_rad = theta_total_rad-theta_act		#effective angle to do [rad]
	theta_todo_deg = theta_todo_rad*360/(2*pi)		#effective angle to do [deg]
	
	#if the angle is greater than 180 degrees, instead of rotating for such angle it's faster to rotate in the opposite direction with a lower angle (i.e. 360-x)		
	if(theta_todo_deg > 180):
		#print("%f prima -- " %(theta_todo_deg))	#just for visualization purposes
		theta_todo_deg=-(360-theta_todo_deg)
		#print("%f dopo/n" %(theta_todo_deg))		#just for visualization purposes
	
	x_ref = sqrt(x_todo**2+y_todo**2)				#effective distance to do [m]
	
	#reached goal area, once that the distance from the goal is less than 2cm stop (set to 0 the references)
	if(x_ref <= 0.02):
		x_ref = 0									
		theta_todo_deg = 0
		
	Ref.x_ref=x_ref									#reference position [m]
	Ref.theta_ref=theta_todo_deg					#reference angle [deg]
	
	pub.publish(Ref)								#publish the stored values on the topic "send_references"

	
	
#ROS

rospy.init_node('path_planner') 													#define a node called path_planner

sub_from_odometry = rospy.Subscriber('send_odometry', Custom_odometry, math_tmp)    #set this node as a subscriber to "localisation_odometry" of the type Custom_odometry

sub_from_interface = rospy.Subscriber('send_goal', Custom_interface, callback)		#set this node as a subscriber to "drawing_interface" of the type Custom_interface	

pub = rospy.Publisher('send_references', Custom_references, queue_size=1)			#set this node as publisher which publishes on the topic "send_references" of the type Custom_references 

rospy.spin()
