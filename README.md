# ROSCODE
Repository of all packages and code for ROS

Description:
This package is intended for use with the University of St. Thomas Senior Design Autonomous Snowplow project.  It contains packages for use with a Sabertooth 2x32 motor controller and interfaces an RPLIDAR 360 for navigation using SLAM.  It also has a custom PID navigation node for traversing set waypoints.


Packages:

plow_motor_control: used for generating linear and angular velocities for use with the "plow_motor_control_py" node (Twist messages over the "cmd_vel" topic)

plow_motor_control_py: Python node written to send motor commands to the Sabertooth 2x32A motor controller.  It subscribes to "cmd_vel" and converts those Twist values into motor controller commands and sends them over USB

plow_waypoint_nav: Python node for traversing waypoints one by one.  It works by creating an array of waypoints and then uses a custom PID loop to traverse each waypoint

waypoint_marker_display: Python package for displaying markers in RVIZ.  Simply click "2D nav goal" while this package is running and it will create a marker where you click.

driveway_sim: used for simulating simple plowing strategies (very rough and currently only being used for demonstration purposes).  It uses turtlesim to visualize plowing strategies

snowplow_visualization: very rough test package for displaying the plow model within RVIZ.  (NEEDS WORK)


