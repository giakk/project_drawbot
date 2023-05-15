#! /usr/bin/env python3

# Source: https://pythonguides.com/python-turtle-mouse/

"""
Generates a window in which through the cursor it's possible to set the path that the robot has to follow. In particular it traces a straight line starting from the origin of the axes (0,0) to the wanted point and provides the coordinates of such point. It is possible to set multiple goals one after the other. The size of the window can be regulated according to the workspace's dimension of the robot.
"""

from turtle import *
import turtle
import rospy
from drawbot.msg import Custom_interface
from std_msgs.msg import Int16

#VARIABLES
WIDTH, HEIGHT = 1600, 1000


screen = turtle.Screen()
screen.setup(WIDTH + 4, HEIGHT + 8)				# fudge factors due to window borders & title bar
screen.setworldcoordinates(0, 0, WIDTH, HEIGHT) # setting the origin form which the robot starts to move, in this way wherever we place it it will always be in the (0,0) of our window's frame

#ROS
rospy.init_node('drawing_interface')
pub = rospy.Publisher('send_goal', Custom_interface, queue_size=10)

#FUNCTIONS
def func(i, j):
  
  goal = Custom_interface()
  turtle.goto(i, j)
  goal.x = int(i)/1000
  goal.y = int(j)/1000
  turtle.write("x="+str(goal.x)+"m\ny="+str(goal.y)+"m")
  print(goal.x, goal.y)
  pub.publish(goal) 							# provides the goal coordinates on the topic "send_goal"
  

screen.onclick(func)
screen.mainloop()
