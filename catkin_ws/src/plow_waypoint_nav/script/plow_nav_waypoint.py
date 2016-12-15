#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from move_base_msgs import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import time

error_tolerance = 0.3 #error allowed for navigating to waypoints
nav_goal_set = 0
nav_goal = 0
current_location = PoseStamped()
goal_location = PoseStamped()
nav_motor_cmd = Twist()
# current_waypoint = 
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #create publisher for motor commands
def get_current_pose(pose):
    global current_location
    current_location = pose #x,y,z and orientation
    # current_location = pose.pose.position.x # just the x component
    # rospy.loginfo("%s", current_location)
    

def get_goal_pose(pose):
    global goal_location
    global nav_goal_set
    nav_goal_set = nav_goal_set + 1
    goal_location = pose
    rospy.loginfo("goal: %s", goal_location)
    
# def get_nav_goal(pose):

def main(): 

    rospy.init_node('plow_waypoint_nav', anonymous=True)
    # rospy.Subscriber("move_base_simple/goal", String, callback)
    rospy.Subscriber("/slam_out_pose", PoseStamped, get_current_pose)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, get_goal_pose)
    # rospy.Subscriber("", PoseStamped, get_nav_goal)
         


    # rospy.spin()
   
    while not rospy.is_shutdown():

        # if(current_location < )
        global current_location
        global goal_location
        rospy.loginfo("current: %s", current_location.pose.position.x)
        rospy.loginfo("goal: %s",goal_location.pose.position.x)
        global nav_goal_set
        if(nav_goal_set == 1):
            # nav_motor_cmd.linear.x = 0.3    
            # if(current_location < (nav_goal + error_tolerance) or current_location > (nav_goal - error_tolerance)):
            if(current_location.pose.position.x < (goal_location.pose.position.x + error_tolerance)):
                nav_motor_cmd.linear.x = 0.3
                nav_motor_cmd.angular.z = 0.0
                rospy.loginfo("heading toward waypoint")
            else:
                nav_goal_set = 0
                nav_motor_cmd.linear.x = 0.0
                nav_motor_cmd.angular.z = 0.0
                rospy.loginfo("you have arrived")
        vel_pub.publish(nav_motor_cmd)

        time.sleep(1)

    # if(current_location.position.x < (nav_goal + error_tolerance)):
    #     rospy.loginfo("not at waypoint yet....")
    # else:
    #     rospy.loginfo("arrived at waypoint...")

if __name__ == '__main__':
    main()
    
