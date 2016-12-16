#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from move_base_msgs import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import time
from math import *


error_tolerance = 0.3 #error allowed for navigating to waypoints
current_location = PoseStamped()
goal_locations = []
nav_motor_cmd = Twist()
angular_threshold = radians(3) #define threshold of angular accuracy
linear_threshold = 0.3 #threshold for linear accuracy
angular_p = 10 #throttle turning speed to a maximum amount
angular_i = 0
angular_d = 0
linear_p = 0.9 #throttle linear speed to a maximum speed
linear_i = 0
linear_d = 0
angular_error = 0
angular_error_tot = 0
linear_error = 0
linear_error_tot = 0


vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #create publisher for motor commands
def get_current_pose(pose):
    global current_location
    current_location = pose #x,y,z and orientation
    # current_location = pose.pose.position.x # just the x component
    # rospy.loginfo("%s", current_location)
    

def get_goal_pose(pose):
    global goal_location
    goal_locations.append(PoseStamped())
    goal_locations[len(goal_locations)] = pose
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
        global linear_error
        global linear_error_tot
        global angular_error
        global angular_error_tot
        global nav_goals

        linear_error_prev = linear_error
        angular_error_prev = angular_error

        # rospy.loginfo("current: %s", current_location.pose.position.x)
        # rospy.loginfo("goal: %s",radians(goal_location.pose.position.x))
        curr_to_goal_x = nav_goals[0].pose.position.x - current_location.pose.position.x
        curr_to_goal_y = nav_goals[0].pose.position.y - current_location.pose.position.y
        try:
            angular_error = atan(curr_to_goal_y/curr_to_goal_x)    
        except ZeroDivisionError: #handle case where values are not assigned yet
            angular_error = radians(90)
        angular_error_tot += angular_error
        
        
        rospy.loginfo("angle to target: %s",degrees(angular_error))
        rospy.loginfo("angle threshold: %s",degrees(angular_threshold))
        


        if(len(nav_goals) > 0):
            # nav_motor_cmd.linear.x = 0.3    
            # if(current_location < (nav_goal + error_tolerance) or current_location > (nav_goal - error_tolerance)):
           
            linear_error = sqrt(curr_to_goal_x**2 + curr_to_goal_y**2)
            linear_error_tot += linear_error
            rospy.loginfo("distance to target: %s",linear_error)
            rospy.loginfo("distance threshold: %s",linear_threshold)

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
                del nav_goals[0]
            

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
    
