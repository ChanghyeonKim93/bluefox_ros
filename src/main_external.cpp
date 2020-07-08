#include <iostream>
#include <ros/ros.h>

#include "bluefox_multiple_ros.h"

// trigger pin = digIn0+
// trigger ground level is same with the Arduino ground level.
// TODO: recognize serial numbers of multiple cameras and grant specific "id".
// TODO: multiple cameras setting is not supported yet .. T.T
int main(int argc, char **argv) {
    std::cout << "BlueFOX node is running..." << std::endl;
    ros::init(argc, argv, "bluefox_multiple_node");
    ros::NodeHandle nh("~");

    bool binning_on   = false;
    bool triggered_on = false;
    bool aec_on       = true; // auto exposure control on / off
    bool agc_on       = true; // auto gain control on / off
    int expose_us     = 10000; // it is also max. exposure for auto exposure control.
    double frame_rate = 40.0; // frame rate (full resolution: up to 30 Hz)

    ros::param::get("~binning_on", binning_on);
	ros::param::get("~triggered_on", triggered_on);
	ros::param::get("~aec_on", aec_on);
	ros::param::get("~agc_on", agc_on);
	ros::param::get("~expose_us", expose_us);
    ros::param::get("~frame_rate", frame_rate);

    BlueFOX_MULTIPLE_ROS *bluefoxs = 
        new BlueFOX_MULTIPLE_ROS(nh, binning_on, triggered_on, 
        aec_on, agc_on, expose_us, frame_rate);
    
    while((bluefoxs->getStatus() > -1) && ros::ok())
    {
	
        ros::spinOnce();
    }

    delete bluefoxs;
    ROS_INFO_STREAM("Cease cameras.\n");
    return -1;
};
