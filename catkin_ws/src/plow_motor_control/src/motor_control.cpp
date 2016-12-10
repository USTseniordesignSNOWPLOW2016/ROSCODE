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
ros::Publisher motor_data_pub;
ros::Publisher motor_data_pub_2;
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

	int max_turn = 0.8*2047; //maximum speed allowed to turn at (2047 is the maximum speed sent to motors)
	int track_L_val = 0; //this is the value that is actually sent out to the wheels (it is the percentage of the maximum turn speed taken from the joystick)
	int track_R_val = 0; //this is the value that is actually sent out to the wheels (it is the percentage of the maximum turn speed taken from the joystick)

	double throttle_fwd_perc = 0.0;
	double throttle_rev_perc = 0.0;
	double steer_dir = 0.0;
	double speed = 0.0; //this is a debugging value
	char actual_dir = 'A'; //this will just be used for debugging purposes.  We will be later mapping this to actual motor commands
	char plow_FR_dir = 'S'; //this is used for debugging, 'S' for Stopped, 'F' for Forward, 'R' for Reverse

	//NOTE: we will need to configure a deadzone so that we can make sure that the plow will not take off unexpectedly

	throttle_fwd_perc = 0.0;
	throttle_rev_perc = 0.0;


	if((DPAD_U == 1) && (governor > 0.1)) //increase the maximum speed by pressing up on the DPAD
	{
		governor = governor - 0.1;
		ROS_INFO("MAX SPEED INCREASED TO %f",(1.0 - governor)); //the governor value will be SUBTRACTED FROM the MAX SPEED
	}

	if((DPAD_D == 1) && (governor < 0.9))//decrease the maximum speed by pressing down on the DPAD
	{
		governor = governor + 0.1;
		ROS_INFO("MAX SPEED DECREASED TO %f",(1.0 - governor));
	}


	throttle_fwd_perc = ((-1*(trig_RT)+1)/2)-governor; //default mapping is 1 to -1.  This has been inverted and remapped to go from 0 to 1 (invert, add one, and divide by 2)
	throttle_rev_perc = ((-1*(trig_LT)+1)/2)-governor; //default mapping is 1 to -1.  This has been inverted and remapped to go from 0 to 1 (invert, add one, and divide by 2)
	steer_dir = joy_oneLR; //take the left and right direction from the left joystick
	
	if(throttle_fwd_perc < 0)
	{
		throttle_fwd_perc = 0;
	}

	

	if((throttle_rev_perc > 0.1) && (throttle_fwd_perc > 0.1))
	{
		ROS_INFO("ERROR CANNOT GO FWD & REVERSE"); //might be used for braking (holding both triggers down)
	}

	else if(throttle_fwd_perc > 0.1)
	{
		plow_FR_dir = 'F';
		speed = throttle_fwd_perc;
		track_L_val = throttle_fwd_perc * motor_val_max; 
		track_R_val = throttle_fwd_perc * motor_val_max; 
	}

	else if(throttle_rev_perc > 0.1)
	{
		plow_FR_dir = 'R';
		speed = throttle_rev_perc;
		track_L_val = -1*(throttle_rev_perc * motor_val_max); 
		track_R_val = -1*(throttle_rev_perc * motor_val_max); 
	}

	else
	{
		speed = 0.0;
	}


	if(steer_dir < -0.12) //might want to move this to the else loop so that turning can only be done without throttle
	{
		ROS_INFO("throttle_rev_perc: %f" ,throttle_rev_perc);
		if(throttle_fwd_perc < 0.1)
		{
			track_L_val = abs(steer_dir) * max_turn;
			track_R_val = -abs(steer_dir) * max_turn;
		}
		//actual_dir = 'R';
		//track_L_val = (steer_dir + throttle_fwd_perc)*motor_val_max;
		else if(throttle_rev_perc < 0.1 && throttle_fwd_perc > 0.1)
		{
			ROS_INFO("turning and going forward");
			track_L_val = (abs(steer_dir) + throttle_fwd_perc) * motor_val_max;
			track_R_val = 0.5*throttle_fwd_perc * motor_val_max;
		}

		else if(throttle_rev_perc > 0.1)
		{
			ROS_INFO("backing up and turning");
			track_L_val = -(abs(steer_dir) + throttle_rev_perc) * motor_val_max;
			track_R_val = -0.5*throttle_rev_perc * motor_val_max;	
		}
		
		//track_R_val = (throttle_fwd_perc)*motor_val_max;
		if(track_L_val > motor_val_max)
		{
			track_L_val = motor_val_max;
		}
	}
	else if(steer_dir > 0.14)
	{
		actual_dir = 'L';
		ROS_INFO("throttle_rev_perc: %f", throttle_rev_perc);
		if(throttle_fwd_perc < 0.1)
		{
			track_L_val = -abs(steer_dir) * motor_val_max;
			track_R_val = abs(steer_dir) * motor_val_max;
		}
		// track_L_val = (throttle_fwd_perc)*motor_val_max;
		// track_R_val = (steer_dir+throttle_fwd_perc)*motor_val_max;
		else if(throttle_rev_perc < 0.1 && throttle_fwd_perc > 0.1)
		{
			ROS_INFO("turning and going forward");
			track_L_val = 0.5*throttle_fwd_perc * motor_val_max;
			track_R_val = (abs(steer_dir) + throttle_fwd_perc) * motor_val_max;	
		}
		else if(throttle_rev_perc > 0.1)
		{
			ROS_INFO("backing up and turning");
			track_L_val = -0.5*throttle_rev_perc * motor_val_max;
			track_R_val = -(abs(steer_dir) + throttle_rev_perc) * motor_val_max;	
		}

		if(track_R_val > motor_val_max)
		{
			track_R_val = motor_val_max;
		}
	}
	else
	{
		actual_dir = 'C'; //C for centered
	}

	/*
		Steering Algorithm explained: 

		max_turn // this will be a defined constant speed that will represent the maximum speed that we can turn the snowplow at

		Straight: L_wheel_speed = throttle_perc //both wheels will just be the throttle speed 
				  R_wheel_speed = throttle_perc 

		Left Turn: L_wheel_speed = throttle_perc * max_turn //a turn will be the percentage of turn multiplied by the maximum allowed turn speed
				   R_wheel_speed (In Reverse) = throttle_perc * max_turn

		Right Turn: L_wheel_speed (In Reverse) = throttle_perc * max_turn //a turn will be the percentage of turn multiplied by the maximum allowed turn speed
				   R_wheel_speed = throttle_perc * max_turn
	*/

	// ROS_INFO("SPEED %%: %f",speed); //this is basic debug statement
	// ROS_INFO("DIRECTION: %c",plow_FR_dir); //this is basic debug statement
	// ROS_INFO("Turning : %c",actual_dir);
	// ROS_INFO("Motor 1 Val : %d",track_L_val);
	// ROS_INFO("Motor 2 Val : %d",track_R_val);

	ros::NodeHandle n;
	
	motor_data_pub = n.advertise<std_msgs::String>("sabertooth_motor_data", 1); //create a publisher for the motor control data
	motor_data_pub_2 = n.advertise<std_msgs::String>("sabertooth_motor_data_2", 1); //create a publisher for the motor 2 control data
	std_msgs::String msg;
	std_msgs::String msg_2;

	//this is the message information
    std::stringstream ss;
    std::stringstream ss_2;

    ss << track_L_val; 
    ss_2 << track_R_val;
    msg.data = ss.str();
    msg_2.data = ss_2.str();
    motor_data_pub.publish(msg);
    motor_data_pub_2.publish(msg_2);
	//end message info

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "learning_joy"); //CHANGE THIS WHEN YOU MOVE IT TO A NEW PACKAGE
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("joy",10,JoystickCallback);
	ros::spin();

}
