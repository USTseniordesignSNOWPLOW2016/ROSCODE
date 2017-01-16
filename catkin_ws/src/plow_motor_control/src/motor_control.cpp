/*
	Motor controller Package
	Author: Brandon Jameson
	Date Created: 8-26-16
	Description: This is a motor controller package created for ROS that will interface an Xbox 360 controller with a motor controller (Sabertooth 2x32A)
*/



#include <ros/ros.h>
#include <geometry_msgs/Twist.h> //not sure that this is needed
#include <sensor_msgs/Joy.h>
#include "std_msgs/String.h"


double governor = 0.5; //this governor will keep the system from taking off unexpectedly.  Use the DPAD to adjust the maximum speed of the vehicle
const int motor_val_max = 2047; //this is the maximum value for the motor controller
ros::Publisher plow_cmd_vel_pub;
// class SabertoothMotorController{ //maybe use this class to clean everything up
// 	public:
// 		ros::Publisher motor_data_pub = n.advertise<std_msgs::String>("sabertooth_motor_data", 1000); //create a publisher for the motor control data
// }

void JoystickCallback(const sensor_msgs::Joy::ConstPtr& joy_data)
{
	//this will store the joystick values -> we can specify which joystick we are looking at and which axis of that joystick
	//UD = Up and Down, LR = Left and Right
	double joy_oneLR = joy_data->axes[0];
	double joy_oneUD = joy_data->axes[1];
	double trig_LT = joy_data->axes[2];
	double joy_twoLR = joy_data->axes[3];
	double joy_twoUD = joy_data->axes[4];
	double trig_RT = joy_data->axes[5];

	int a_button = joy_data->buttons[0];
	int b_button = joy_data->buttons[1];
	int x_button = joy_data->buttons[2];
	int y_button = joy_data->buttons[3];
	int LB_button = joy_data->buttons[4];
	int RB_button = joy_data->buttons[5];
	int sel_button = joy_data->buttons[6];
	int start_button = joy_data->buttons[7];
	int xbox_button = joy_data->buttons[8];
	int L_joy_click = joy_data->buttons[9];
	int R_joy_click = joy_data->buttons[10];
	int DPAD_L = joy_data->buttons[11];
	int DPAD_R = joy_data->buttons[12];
	int DPAD_U = joy_data->buttons[13];
	int DPAD_D = joy_data->buttons[14];

	int track_L_val = 0; //this is the value that is actually sent out to the wheels (it is the percentage of the maximum turn speed taken from the joystick)
	int track_R_val = 0; //this is the value that is actually sent out to the wheels (it is the percentage of the maximum turn speed taken from the joystick)

	double throttle_fwd_perc = 0.0;
	double throttle_rev_perc = 0.0;
	double steer_dir = 0.0;

	//NOTE: we will need to configure a deadzone so that we can make sure that the plow will not take off unexpectedly

	if(trig_RT == 0)
	{
		//if this value is something other than 0, we don't need to initialize (until trigger is pressed for the first time, it defaults to 0)
		trig_RT = 1; //set trigger to the "released" position
	}
	if(trig_LT == 0)
	{
		//if this value is something other than 0, we don't need to initialize (until trigger is pressed for the first time, it defaults to 0)
		trig_LT = 1; //set trigger to the "released" position
	}

	throttle_fwd_perc = ((-1*(trig_RT)+1)/2);
	throttle_rev_perc = ((-1*(trig_LT)+1)/2);

	// if((DPAD_U == 1) && (governor > 0.1)) //increase the maximum speed by pressing up on the DPAD
	// {
	// 	governor = governor - 0.1;
	// 	ROS_INFO("MAX SPEED INCREASED TO %f",(1.0 - governor)); //the governor value will be SUBTRACTED FROM the MAX SPEED
	// }

	// if((DPAD_D == 1) && (governor < 0.9))//decrease the maximum speed by pressing down on the DPAD
	// {
	// 	governor = governor + 0.1;
	// 	ROS_INFO("MAX SPEED DECREASED TO %f",(1.0 - governor));
	// }

	ros::NodeHandle n;
	
	plow_cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //create a publisher for Twist messages created from Xbox controller input
	geometry_msgs::Twist plow_twist; //create a Twist message

	plow_twist.linear.x = throttle_fwd_perc - throttle_rev_perc;
	plow_twist.linear.y = 0; //set to 0 since we can't move laterally
	plow_twist.linear.z = 0; //set to 0 since we can't move up and down
	steer_dir = joy_oneLR;
	plow_twist.angular.x = 0;
	plow_twist.angular.y = 0;
	if(steer_dir < -0.12 || steer_dir > 0.14)
	{
		plow_twist.angular.z = steer_dir;	
	}
	else
	{
		plow_twist.angular.z = 0.0;	
	}
	

	//this is the message information
    // std::stringstream ss;

    // ss << track_L_val; 
    // msg.data = ss.str();
    plow_cmd_vel_pub.publish(plow_twist);
	//end message info

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "plow_motor_control");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("joy",10,JoystickCallback); //subscribe to xbox controller data
	ros::spin();

}
