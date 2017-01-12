#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from move_base_msgs import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import sensor_msgs.msg
import time
from math import *


error_tolerance = 0.3 #error allowed for navigating to waypoints
current_location = PoseStamped()
goal_locations = []
nav_motor_cmd = Twist()
angular_threshold = radians(3) #define threshold of angular accuracy
linear_threshold = 0.4 #threshold for linear accuracy
angular_p = 4 #throttle turning speed to a maximum amount
angular_i = 0
angular_d = 0
linear_p = 0.7 #throttle linear speed to a maximum speed
linear_i = 0
linear_d = 0.7
angular_error = 0
angular_error_tot = 0
linear_error = 0
linear_error_tot = 0

set_waypoint_at_pos = 0 #use xbox controller to set the current position as a waypoint to head towards (set with "A" button)
navigate_waypoints = 0 #do not go to waypoints until this is 1, after complete, set to 0 (toggled by right bumper)

vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #create publisher for motor commands
def get_current_pose(pose):
    global current_location
    current_location = pose #x,y,z and orientation
    # current_location = pose.pose.position.x # just the x component
    # rospy.loginfo("%s", current_location)
    global goal_locations
    global set_waypoint_at_pos
    if(set_waypoint_at_pos == 1):
        goal_locations[len(goal_locations)-1] = current_location #if "A" button is pressed, set waypoint to current position
        set_waypoint_at_pos = 0

        

def get_goal_pose(pose):
    global goal_location
    global navigate_waypoints
    goal_locations.append(PoseStamped())
    goal_locations[len(goal_locations)-1] = pose
    rospy.loginfo("goal: %s", goal_locations[len(goal_locations)-1])
    
def joy_callback(data):
    a_button = data.buttons[0]
    right_bumper = data.buttons[5]
    global navigate_waypoints
    if(a_button == 1):
        if(navigate_waypoints != 0):
            rospy.loginfo("cannot set waypoint while navigating")
        else:
            rospy.loginfo("setting a waypoint at current position")    
            set_waypoint_at_pos = 1

    if(right_bumper == 1):
        rospy.loginfo("starting navigation")
        navigate_waypoints = 1


def main(): 

    rospy.init_node('plow_waypoint_nav', anonymous=True)
    rospy.Subscriber("/slam_out_pose", PoseStamped, get_current_pose)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, get_goal_pose)
    rospy.Subscriber("/joy", sensor_msgs.msg.Joy, joy_callback)
         


    # rospy.spin()
   
    while not rospy.is_shutdown():

        # if(current_location < )
        global linear_error
        global linear_error_tot
        global angular_error
        global angular_error_tot
        global goal_locations

        linear_error_prev = linear_error
        angular_error_prev = angular_error

        # rospy.loginfo("current: %s", current_location.pose.position.x)
        # rospy.loginfo("goal: %s",radians(goal_location.pose.position.x))
        try:
            curr_to_goal_x = goal_locations[0].pose.position.x - current_location.pose.position.x
            curr_to_goal_y = goal_locations[0].pose.position.y - current_location.pose.position.y
        except IndexError:
            curr_to_goal_x = 0
            curr_to_goal_y = 0

        try:
            angular_error = atan(curr_to_goal_y/curr_to_goal_x)    
        except ZeroDivisionError: #handle case where values are not assigned yet
            angular_error = radians(90)
        angular_error_tot += angular_error
        
        
        # rospy.loginfo("angle to target: %s",degrees(angular_error))
        # rospy.loginfo("angle threshold: %s",degrees(angular_threshold))
        
        # rospy.loginfo("navigate waypoints status %s", navigate_waypoints)
        if(len(goal_locations) > 0 and navigate_waypoints != 0): #don't start navigating until "navigate_waypoints" is toggled by right bumper
            # nav_motor_cmd.linear.x = 0.3    
            # if(current_location < (nav_goal + error_tolerance) or current_location > (nav_goal - error_tolerance)):
           
            linear_error = sqrt(curr_to_goal_x**2 + curr_to_goal_y**2)
            linear_error_tot += linear_error
            # rospy.loginfo("distance to target: %s",linear_error)
            # rospy.loginfo("distance threshold: %s",linear_threshold)

            if abs(linear_error) > linear_threshold:
                nav_motor_cmd.angular.z = (angular_error*angular_p) + (angular_error_tot*angular_i) + ((angular_error-angular_error_prev)*angular_d)

                if abs(angular_error) < angular_threshold:
                    nav_motor_cmd.linear.x =  (linear_error*linear_p) + (linear_error_tot*linear_i) + ((linear_error-linear_error_prev)*linear_d)
                    nav_motor_cmd.angular.z *= 2
                    rospy.loginfo("heading toward waypoint")
                    # rospy.loginfo("nav command: %s",nav_motor_cmd)
                else:
                    nav_motor_cmd.linear.x = 0.0
            else:
                nav_motor_cmd.linear.x = 0.0
                nav_motor_cmd.angular.z = 0.0
                rospy.loginfo("you have arrived")
                rospy.loginfo("these are all of our waypoints \n %s", goal_locations)
                #reset all errors to 0 for the next waypoint
                linear_error = 0
                linear_error_tot = 0
                angular_error = 0
                angular_error_tot = 0
                del goal_locations[0]
            

            # if(current_location.pose.position.x < (goal_location.pose.position.x + error_tolerance)):
            #     nav_motor_cmd.linear.x = 0.3
            #     nav_motor_cmd.angular.z = 0.0
                

            # else:
            #     nav_goal_set = 0
            #     nav_motor_cmd.linear.x = 0.0
            #     nav_motor_cmd.angular.z = 0.0
                
        vel_pub.publish(nav_motor_cmd)

        time.sleep(0.5)

    # if(current_location.position.x < (nav_goal + error_tolerance)):
    #     rospy.loginfo("not at waypoint yet....")
    # else:
    #     rospy.loginfo("arrived at waypoint...")

if __name__ == '__main__':
    main()