#! /usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from drawbot.msg import Custom_references
from drawbot.msg import Custom

"""
takes from "send_references" the two referece values of the velociy: theta_ref (i.e. how much to rotate) and x_ref (i.e. how much to go straight). It determines if the former command was a rotation or a translation in order to understand if we had a change between the former and the actual command (in this case change will be set to 1) and compute the command to send to arduino for the rotation or for the translation according to the case. The priority will always be given to the rotation and iff it is satisfied, we'll proceed in the forward movement. Finally it will provide as output the two velocities in rpm to send to the motor: ref_speed_r, ref_speed_l ,through the topic Send_motor_velocities.
"""



#INITIALIZING VARIABLES

#initializing internal variables
form_rot = 0	#if 1 then the former command was a rotation
form_lin = 0	#if 1 then the former command was a translation
vel_lin = 0	#linear velocity [m/s]
change = 0

internal_vel = 565 #former580
external_vel = 440 #former465
linear_vel = 440   #former465





# dobbiamo evitare che al primo cambio change si attivi magari con una seconda variabile "first_change"

#FUNCTIONS AND CALLBACKS

def set_velocity(pos_lin, pos_ang):

	global form_rot, form_lin
	global vel_lin
	global flag
	global previous_time, interval
	global change
	global internal_vel
	global external_vel
	global linear_vel

	Info=Custom()

	if((pos_ang<-10.0 or pos_ang>10.0) and (pos_lin>0.1)): #in this way we're letting the robot rotate only from a certain treshold value otherwise it'll stop every time there's even a degree of rotation to do

	# We are in a rotational motion
			
		vel_lin = 0		#null linear velocity, since we have to rotate
		form_rot = 1	#we are rotating so set to 1 for the next iteration

		#check for the variable "change"
		if(form_lin) :
			change=1
			form_rot=1
			form_lin=0

		#left rotation
		if(pos_ang>0) :
			
			right_motor = int(external_vel) 										#right motor speed [rpm]
			left_motor = int(-internal_vel) 	 									#left motor speed [rpm]
			print("right %d -- left %d" %(right_motor, left_motor))					#print of the two sent speed, just for visualization purposes
			Info.motor_r=right_motor
			Info.motor_l=left_motor

		#right rotation
		else:
	
			right_motor = int(-internal_vel) 										#right motor speed [rpm]
			left_motor = int(external_vel) 											#left motor speed [rpm]
			print("right %d -- left %d" %(right_motor, left_motor))					#print of the two sent speed, just for visualization purposes
			Info.motor_r=right_motor
			Info.motor_l=left_motor

	else:

	# We are in a linear motion
		
		#boundary for the distance, if the distance is lower than 10cm then do not go forward
		if pos_lin > 0.1 :

			vel_ang=0				#null angular velocity since we have to go forward
			vel_lin=linear_vel		#linear velocity not null since we are in a linear motion
			form_lin=1


		#the boundary for the distance is not respected, so we should stop
		else:
			vel_ang= 0
			vel_lin= 0
			form_lin= 1

		if(form_rot):
			change=1
			form_rot=0
			form_lin=1

		val = int(vel_lin)

		print("right %d -- left %d" %(val, val))	#print of the two sent speed, just for visualization purposes

		Info.motor_r=val			#right motor speed [rpm]
		Info.motor_l=val			#left motor speed [rpm]


	pub.publish(Info)				#publish the two speed of the topic [rpm] on the topic


#callback function of the topic "send_references"
def callback(msg):
	set_velocity(msg.x_ref, msg.theta_ref)



#ROS

rospy.init_node('math') #define a node called "math"

sub = rospy.Subscriber('send_references', Custom_references, callback)	#set this node as a subscriber to "send_references" of the type Custom_references

pub = rospy.Publisher('send_motor_velocities', Custom , queue_size=1)	#set this node as publisher which publishes on the topic "send_motor_velocities" of the type Custom

rospy.spin()

