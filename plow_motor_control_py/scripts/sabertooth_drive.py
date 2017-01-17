#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
left_speed = ""
right_speed = ""
motor_1_string = ""
motor_2_string = ""
motor_max_val = 2047; #define the maximum speed for a given motor

ser = serial.Serial(port = "/dev/ttyACM0",baudrate = 9600)
if ser.isOpen:
	ser.write("M1: "+str(0)+"\r\n") #ensure that motors are stopped when the node is initialized
	ser.write("M2: "+str(0)+"\r\n") #ensure that motors are stopped when the node is initialized
ser.close()
ser.open()

def message_cb(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    left_speed = (data.linear.x - data.angular.z)*2047;
    left_speed = min(motor_max_val,left_speed); #set to max if it goes over
    left_speed = max(-motor_max_val,left_speed); #set to max if it goes over
    ser.write("M1: " + str(int(left_speed)) + "\r\n")
    # rospy.loginfo("left_speed: " + str(left_speed));

 #   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    right_speed = (data.linear.x + data.angular.z)*2047;
    right_speed = min(motor_max_val,right_speed); #set to max if it goes over
    right_speed = max(-motor_max_val,right_speed); #set to max if it goes over
    ser.write("M2: " + str(int(right_speed)) + "\r\n")
    # rospy.loginfo("right_speed: " + str(right_speed));

def listener():
    rospy.init_node('sabertooth_control', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, message_cb)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #with serial.Serial(port = "/dev/ttyACM0",baudrate = 9600) as ser:   
#        while True:
         listener()
  #       motor_1_string = "M1: " + left_speed  + "\r\n"
   #      motor_2_string = "M2: " + right_speed  + "\r\n"
         #ser.write("M1: "+left_speed+"\r\n")
	 #ser.write("M2: "+right_speed+"\r\n")
	 #ser.write(motor_1_string)
         #ser.write(motor_2_string)
