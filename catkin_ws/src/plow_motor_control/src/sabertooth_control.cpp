#include "ros/ros.h"
#include <string>
#include <boost/thread.hpp>

#define SABERTOOTH_PORT "ttyS0" //define the Sabertooth Controller USB port (change if this changes)
#define SERIAL_TRANSFER_RATE 57600 //increase if necessary

std::string sabertooth_serial_port = SABERTOOTH_PORT; //

int serial_transfer_rate = SERIAL_TRANSFER_RATE; //set the default serial transfer rate


boost::mutex drive_serial_write; 

Serial drive_serial;


void process_serial(Serial &serial, TBuff<uint8_t> &buff, orcp2::packet &pkt)
{


}

bool motor_write(ros_4wd_driver::motor_write::Request  &req, 
					ros_4wd_driver::motor_write::Response &res) {
					
	orcp2::ORCP2 orcp(drive_serial);

    drive_serial_write.lock();
	orcp.motorWrite(req.id, req.value);
	drive_serial_write.unlock();
	
	return true;
}

int main(int argc, char **argv)
{
	ros::NodeHandle n;
	ROS_INFO("Starting Sabertooth Node....");
	ROS_INFO("Opening Sabertooth Port");
    if( drive_serial.open(sabertooth_serial_port.c_str(), serial_rate) ) {
        ROS_ERROR("Cant open port: %s:%d", sabertooth_serial_port.c_str(), serial_rate);
        return -1;
    }

}