#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"


ros::Publisher plow_velocity_publisher; //the velocity data that will be published for the snowplow
ros::Subscriber pose_subscriber;	// to determine the position for turning the robot in an absolute orientation --> in the setDesiredOrientation fn
turtlesim::Pose turtlesim_pose;

using namespace std;

//function for moving the snowplow around
void move(double speed, double distance, bool dir);
void rotate(double angle, bool clockwise);
double deg2rad(double angle_in_degrees); //convert degrees to radians
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double rad2deg(double angle_in_rad); //convert radians to degrees

const double PI = 3.14159265359;
const double angular_tolerance = 0.5; //angular error tolerance

int main(int argc, char **argv)
{
	ros::init(argc,argv, "driveway_sim"); //initialize the node

	ros::NodeHandle n;  //initialize a node handle

	ROS_INFO("testing the driveway node");

	plow_velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);	//call poseCallback everytime the turtle pose msg is published over the /turtle1/pose topic.
	ros::Rate loop_rate(0.5);

	turtlesim::Pose goal_pose;
	loop_rate.sleep();	
	ros::Duration(4).sleep();
	rotate(deg2rad(90),0);
	move(3,3,1);
	rotate(deg2rad(45),1);
	move(2,0.5,1);
	rotate(deg2rad(135),0);
	move(2,0.5,1);
	rotate(deg2rad(225),0);
	move(2,0.5,1);
	// rotate(deg2rad(315),0);
	// move(2,0.25,1);
	rotate(deg2rad(270),0);
	move(3,3,1);
	ros::spin();



	return 0;

}

//function for moving the snowplow around
void move(double speed, double distance, bool dir)
{
	//dir: 1 is forward, 0 is reverse
	geometry_msgs::Twist vel_msg;

	if(dir)
	{
		vel_msg.linear.x = abs(speed);
	}
	else
	{
		vel_msg.linear.x = -abs(speed);	
	}
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0;
	ros::Rate loop_rate(10);
	do{
		plow_velocity_publisher.publish(vel_msg);
		double t1=ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();

	}while(current_distance < distance);

	vel_msg.linear.x = 0; //set the velocity message back to 0
	plow_velocity_publisher.publish(vel_msg);
}

void rotate(double angle, bool clockwise)
{
	geometry_msgs::Twist vel_msg;
	
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	
	double current_angle = rad2deg(turtlesim_pose.theta); //keep track of the current angle (degrees for easier debugging)
	double t0; //time t0
	double t1; //time t1
	ros::Rate loop_rate(10);
	// vel_msg.angular.z = deg2rad(10);
	// plow_velocity_publisher.publish(vel_msg);
	
	if(clockwise)
	{
		while(current_angle > (rad2deg(angle))) //keep going until it reaches the angular tolerance
		{
			vel_msg.angular.z -= deg2rad(0.001);
			plow_velocity_publisher.publish(vel_msg);
			current_angle = rad2deg(turtlesim_pose.theta);
			ros::spinOnce();
		}		
	}
	else
	{
		while(current_angle < (rad2deg(angle))) //keep going until it reaches the angular tolerance
		{
			vel_msg.angular.z += deg2rad(0.001);
			plow_velocity_publisher.publish(vel_msg);
			current_angle = rad2deg(turtlesim_pose.theta);
			ros::spinOnce();
		}
	}
	
	vel_msg.angular.z = 0;
	plow_velocity_publisher.publish(vel_msg);
	ros::spinOnce();
}


double deg2rad(double angle_in_degrees) //convert degrees to radians
{
	return angle_in_degrees *PI /180.0;
}

double rad2deg(double angle_in_rad) //convert radians to degrees
{
	return angle_in_rad *(180.0/PI);
}



/**
 *  callback function to update the pose of the robot  
 */

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

void GoToWaypoint(turtlesim::Pose goal_pose, double distance_tolerance)
{
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(10);

	ros::spinOnce();
	loop_rate.sleep();

}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x2-x1),2) + pow((y2-y1),2));
}

