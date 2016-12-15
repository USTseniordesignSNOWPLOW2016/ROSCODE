#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from move_base_msgs import *

current_location = Pose2D()
target_location = Pose2D()

def location_listener(data):
    current_location = data.data

def goal_listener(data):
    target_location = data.data
    
def main():
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber("", Pose2D, location_listener)
    rospy.Subscriber("move_base_simple/goal", Pose2D, goal_listener)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", current_location)
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", target_location)
    # while not rospy.is_shutdown():
    #     if (angel between current pose and target pose < some small value):
    #         forward_speed = (distance between pose's) * (some gain)
    #         angular_speed = (angel between poses's) * (some gain)
    #     else:
    #         forward_speed = 0
    #         angular_speed = (angel between poses's) * (some gain)

    #     send_out_speeds
    # #
if name == '__main__':
    main()