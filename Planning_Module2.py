#!/usr/bin/env python


#Import the required libraries:
from __future__ import print_function,division
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32
#from geometry_msgs.msg import Pose,Twist
#from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import time

time.sleep(10)


#Initialize ROS Node
rospy.init_node('Planning_Module') #Identify ROS Node


#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Longitudinal_Driving_Velocity', Float64, queue_size=10)
pub2 = rospy.Publisher('/Lateral_Distance', Float64, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz


#ROS Subscriber Variables



#ROS Subscriber Code for Position and Velocity

def callback1(data):
    global x_actual
    x_actual = data.data
    

def callback2(data):
    global y_actual
    y_actual = data.data

def callback3(data):
    global speed_actual
    speed_actual = data.data
    

def callback4(data):
    global yaw_actual
    yaw_actual = data.data

rospy.Subscriber('/xcurr', Float32, callback1)
rospy.Subscriber('/ycurr', Float32, callback2)
rospy.Subscriber('/velocity', Float32, callback3)
rospy.Subscriber('/angle', Float32, callback4)

flag_St = 1

V_Des = 0.5
Lane_Des = 0.28125

#Simulation While Loop
st_time = time.time()

speed_actual = 0
yaw_actual = 0
x_actual = 0
y_actual = 0

while 1 and not rospy.is_shutdown():
    st_time = time.time()
    V_Act = speed_actual
    Yaw_act = yaw_actual
    Pos_Act = [x_actual, y_actual ,Yaw_act]
    print(Pos_Act)
    if flag_St == 1:        
        if Pos_Act[0] > 3.5 and Pos_Act[0]<7:
          Lane_Des = -0.2825
          V_Des=0.5
        if Pos_Act[0] > 8:
          Lane_Des = 0.28125
          V_Des=0.9
        if Pos_Act[1]<-0.28 and Pos_Act[1]>-0.283 and Pos_Act[0]<10:
          V_Des=.3
        else:
          V_Des = 0.5
          Lane_Des = 3  
        print(str(V_Des) + "V_Des")
        print(str(Lane_Des) + "lane_Des")
        
    pub1.publish(V_Des)  #Publish msg
    pub2.publish(Lane_Des)  #Publish msg
    rate.sleep()    #Sleep with rate
    end_time = time.time()
    tau  = end_time-st_time

