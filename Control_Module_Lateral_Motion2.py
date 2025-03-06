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
rospy.init_node('Control_Module_Lateral_Motion') #Identify ROS Node

#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Control_Action_Steering', Float64, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz

#ROS Subscriber Variables

Lane_Des = 0
flag_St = 0
flag_Lat = 0

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


#ROS Subscriber Code for Position and Velocity
def callback_Des_Lane(data):
 global Lane_Des	#Identify msg variable created as global variable
 Lane_Des = data.data
  
sub2 = rospy.Subscriber('/Lateral_Distance', Float64, callback_Des_Lane)


def Pure_Pursuit_Control(Lane_Des,Pos_Act,V_Act):
  K_p = 0.5
  global Wheel_Base
  x_Lane = Pos_Act[0] + 1*np.sign(V_Act)#K_p*V_Act
  
  Lookahead_pnt = [x_Lane,Lane_Des]
  print('Lookahead_pnt = '+str(Lookahead_pnt))
  dx = Lookahead_pnt[0]-Pos_Act[0]
  dy = Lookahead_pnt[1]-Pos_Act[1]
  L_d = np.sqrt((dx)**2 + (dy)**2)
  alpha = - Pos_Act[2] + np.arctan2(dy,dx)
  
  Steer_Cont = np.arctan(((2*Wheel_Base*np.sin(alpha))/(L_d)))
  
  if(abs(Steer_Cont) >= np.radians(30)):
   Steer_Cont = np.sign(Steer_Cont)*np.radians(30)
  print('Steer_Cont = '+str(Steer_Cont))
  return Steer_Cont


def Stanley_Control(Lane_Des,Pos_Act,V_Act):
  K_p = 0.5
  K_v = 1
  global Wheel_Base
  
  x_Lane = Pos_Act[0] + 1*np.sign(V_Act)
  Lookahead_pnt = [x_Lane,Lane_Des]
  print('Lookahead_pnt = '+str(Lookahead_pnt))
  dx = Lookahead_pnt[0]-(Pos_Act[0]+Wheel_Base*np.cos(Pos_Act[2]))
  dy = Lookahead_pnt[1]-(Pos_Act[1]+Wheel_Base*np.sin(Pos_Act[2]))
  L_d = np.sqrt((dx)**2 + (dy)**2)
  
  th_p = 0 - Pos_Act[2] 
  try:
   Steer_Cont = th_p + np.arctan(((K_v*dy)/(V_Act)))
  except:
   Steer_Cont = 0
  if(abs(Steer_Cont) >= np.radians(30)):
   Steer_Cont = np.sign(Steer_Cont)*np.radians(30)
  print('Steer_Cont = '+str(Steer_Cont))
  return Steer_Cont

rospy.Subscriber('/xcurr', Float32, callback1)
rospy.Subscriber('/ycurr', Float32, callback2)
rospy.Subscriber('/velocity', Float32, callback3)
rospy.Subscriber('/angle', Float32, callback4)



#Lateral_Controller = rospy.get_param("~Lateral_Controller")
#Wheel_Base = rospy.get_param("~Wheel_Base") #Check Gazebo Model Dimenssions 

Lateral_Controller = "Stanley"
Wheel_Base = 0.2

#Simulation While Loop
st_time = time.time()
Steer_Cont = 0

speed_actual = 0
y_actual = 0
x_actual = 0 
yaw_actual = 0


while 1 and not rospy.is_shutdown():
 st_time = time.time()
 
 V_Act = speed_actual
 print('Vel_Act = '+str(V_Act))
 
 Yaw_act = yaw_actual
 Pos_Act = [x_actual, y_actual, Yaw_act]
 print('Pos_Act = '+str(Pos_Act))
 
 if Lateral_Controller in "Pure_Pursuit":
  Steer_Cont = Pure_Pursuit_Control(Lane_Des,Pos_Act,V_Act)
 elif Lateral_Controller in "Stanley":
  Steer_Cont = Stanley_Control(Lane_Des,Pos_Act,V_Act)
   
 pub1.publish(Steer_Cont)	#Publish msg
 print(str(Steer_Cont)+ "Steer_Cont")
 rate.sleep()		#Sleep with rate
 end_time = time.time()
 tau  = end_time-st_time
 #print(tau)
