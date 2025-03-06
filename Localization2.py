#!/usr/bin/env python
import time
import rospy
import serial
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import math


# Initialize ROS node
rospy.init_node('Localization')

# Open serial port
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()

# Set up ROS publishers
angle_pub = rospy.Publisher('/angle', Float32, queue_size=10)
velocity_pub = rospy.Publisher('/velocity', Float32, queue_size=10)
xcurr_pub = rospy.Publisher('/xcurr', Float32, queue_size=10)
ycurr_pub = rospy.Publisher('/ycurr', Float32, queue_size=10)
min_velocity = 0  # Minimum velocity in m/s
max_velocity = 1.5  # Maximum velocity in m/s
min_pwm = 100 # Minimum PWM value
max_pwm = 255  # Maximum PWM value

# Callback function for the longitudinal control action
def callback_long(data):
    global controlled_vel
    controlled_vel = data.data

# Callback function for the lateral control action
def callback_lat(data):
    global controlled_yaw
    controlled_yaw = round(data.data * 180 / math.pi)

rospy.Subscriber('/Control_Action_Driving_Velocity', Float64, callback_long)
rospy.Subscriber('/Control_Action_Steering', Float64, callback_lat)

controlled_vel = 0
controlled_yaw = 0

# Main loop
while not rospy.is_shutdown():
    # Read data from serial port
    if ser.in_waiting > 0:
        line = ser.readline().decode().strip()

        # Parse angle, velocity, xcurr, ycurr from serial data
        if line.startswith('velocity ='):
            speed_str = line[len('velocity ='):]
            velocity = float(speed_str)
            velocity_pub.publish(velocity)

        elif line.startswith('Current position: ('):
            # Example line: "Current position: (1.23, 4.56)"
            data = line.split('(')[1].split(')')[0].split(',')
            xcurr = float(data[0].strip())
            ycurr = float(data[1].strip())

            # Publish xcurr and ycurr as ROS messages
            xcurr_pub.publish(xcurr)
            ycurr_pub.publish(ycurr)

        elif line.startswith('Angle: '):
            angle_str = line[len('Angle: '):].split(' ')[0]
            angle_str = angle_str.replace('degrees', '')  # Remove 'degrees' suffix
            angle = float(angle_str)
            angle_pub.publish(angle)
    pwm = int((controlled_vel - min_velocity) / (max_velocity - min_velocity) * (max_pwm - min_pwm) + min_pwm)
    # Send the controlled_yaw value to the Arduino
    print("yaw: " + str(controlled_yaw))
    command = "yaw: " + str(controlled_yaw) + "\n"
    command += "\n"
    ser.write(command.encode('utf-8'))
    print("pwm: " + str(pwm))
    command = "pwm: " + str(pwm) + "\n"
    command += "\n"
    ser.write(command.encode('utf-8'))
    # Delay or rate control to control the publishing frequency
    rospy.sleep(0.4)  # Adjust delay if needed

ser.close()
