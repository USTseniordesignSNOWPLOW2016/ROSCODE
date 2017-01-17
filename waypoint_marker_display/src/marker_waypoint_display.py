#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import time
waypoint_marker = Marker()
waypoint_marker_array = MarkerArray()
waypoint_pose = PoseStamped()
waypoint_marker_pub = rospy.Publisher('plow_waypoint_mark', Marker, queue_size=10) #declare a publisher for markers
waypoint_marker_array_pub = rospy.Publisher('plow_waypoint_marker_array', MarkerArray, queue_size=10) #declare a publisher for markers
marker_id = 0
blue_tint = 0.1
def get_waypoint_pose(pose):
    global waypoint_pose
    waypoint_pose = pose #capture the location of the waypoint added in RVIZ
    rospy.loginfo("added a waypoint")
    rviz_waypoint_mark()

def rviz_waypoint_mark():    
    global waypoint_pose
    global waypoint_marker_array
    global marker_id
    global blue_tint
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Marker published at: ")
    # waypoint_marker.header.frame_id = "base_link"
    waypoint_marker.header.frame_id = "map"
    # waypoint_marker.header.stamp = ros::Time()
    # waypoint_marker.ns = "my_namespace"
    waypoint_marker.id = marker_id + 1; #create a new id for each marker that is added (fix to start at 0)
    marker_id = marker_id + 1
    waypoint_marker.type = waypoint_marker.ARROW
    waypoint_marker.action = waypoint_marker.ADD
    waypoint_marker.pose = waypoint_pose.pose
    waypoint_marker.scale.x = 0.3
    waypoint_marker.scale.y = 0.1
    waypoint_marker.scale.z = 0.0
    waypoint_marker.color.a = 1.0
    waypoint_marker.color.r = 0.0
    waypoint_marker.color.g = 0.0
    # waypoint_marker.color.b = 1.0
    blue_tint = blue_tint + 0.1
    waypoint_marker.color.b = blue_tint

    waypoint_marker_array.markers.append(waypoint_marker)
    waypoint_marker_pub.publish(waypoint_marker)
    waypoint_marker_array_pub.publish(waypoint_marker_array)
    
    rate.sleep()

def main(): 
    rospy.init_node('waypoint_listener', anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, get_waypoint_pose)

    while not rospy.is_shutdown():
        time.sleep(0.5)
    # try:
    #     rviz_waypoint_mark()
    # except rospy.ROSInterruptException:
    #     pass


if __name__ == '__main__':
    main()