#!/usr/bin/env python

#Import the required libraries:
from __future__ import print_function,division
import rospy
#from geometry_msgs.msg import Pose,Twist
#from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from std_msgs.msg import Float32
import numpy as np
import math
import time

time.sleep(10)

#Initialize ROS Node
rospy.init_node('Control_Module_Longitudinal_Motion') #Identify ROS Node

#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Control_Action_Driving_Velocity', Float64, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz

#ROS Subscriber Variables

V_Des = 0


#ROS Subscriber Code for Position and Velocity
def callback_Des_Vel(data):
 global V_Des	#Identify msg variable created as global variable
 V_Des = data.data

sub2 = rospy.Subscriber('/Longitudinal_Driving_Velocity', Float64, callback_Des_Vel)

def Speed_Control(V_Des,V_Act,tau):
 K_p = 2
 u = K_p*(V_Des-V_Act)
 if(np.abs(u) > 2):
  u = 2*np.sign(u)
 V_Cont = V_Act + tau*u
 return V_Cont

##Wheel_Base = rospy.get_param("~Wheel_Base") #Check Gazebo Model Dimenssions 
Wheel_Base = 0.2

#Simulation While Loop
st_time = time.time()
tau = 0.01
V_Cont = 0

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

speed_actual = 0
y_actual = 0
x_actual = 0 
yaw_actual = 0

while 1 and not rospy.is_shutdown():
 st_time = time.time()
 
 V_Act = speed_actual
 print('Vel_Act = '+str(V_Act))
 
 Yaw_act = yaw_actual
 Pos_Act = [x_actual, y_actual ,Yaw_act]
 print('Pos_Act = '+str(Pos_Act))
 
 V_Cont = Speed_Control(V_Des,V_Act,tau)
   
 pub1.publish(V_Cont)	#Publish msg
 print(str(V_Cont) +"V_Cont")
 rate.sleep()		#Sleep with rate
 end_time = time.time()
 tau  = end_time-st_time
 #print(tau)


