#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String
import time
left_speed = ""
right_speed = ""
motor_1_string = ""
motor_2_string = ""

ser = serial.Serial(port = "/dev/ttyACM0",baudrate = 9600)
if ser.isOpen:
	ser.write("M1: "+chr(0)+"\r\n")
	ser.write("M2: "+chr(0)+"\r\n")
ser.close()
ser.open()

def message_cb(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    left_speed = data.data
    ser.write("M1: " + left_speed + "\r\n")
    
def message_cb_2(data):
 #   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    right_speed = data.data
    ser.write("M2: " + right_speed + "\r\n")

def listener():
    rospy.init_node('sabertooth_control', anonymous=True)
    rospy.Subscriber("sabertooth_motor_data", String, message_cb)
    rospy.Subscriber("sabertooth_motor_data_2", String, message_cb_2)

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
