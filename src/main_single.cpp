#include <iostream>
#include <ros/ros.h>

#include "bluefox_ros.h"
#include "dynamic_reconfigure/server.h"
#include "bluefox/bluefoxDynConfig.h"


// trigger pin = digIn0+
// trigger ground level is same with the Arduino ground level.
// TODO: recognize serial numbers of multiple cameras and grant specific "id".
// TODO: multiple cameras setting is not supported yet .. T.T
int main(int argc, char **argv) {
    std::cout << "BlueFOX node is running..." << std::endl;
    ros::init(argc, argv, "bluefox_node");

    ros::NodeHandle nh("~");

    bool binning_on   = false;
    bool software_binning_on = false;
    int software_binning_level = 1;
    bool triggered_on = true;
    bool aec_on       = false; // auto exposure control on / off
    bool agc_on       = false; // auto gain control on / off
    bool hdr_on       = false;
    int expose_us     = 11000; // it is also max. exposure for auto exposure control.
    double frame_rate = 60.0; // frame rate (full resolution: up to 30 Hz)
    int cam_id = 1;
    string serial = "25003728";

    ros::param::get("~binning_on", binning_on);
    ros::param::get("~software_binning_on", software_binning_on);
    ros::param::get("~software_binning_level", software_binning_level);
	ros::param::get("~triggered_on", triggered_on);
	ros::param::get("~aec_on", aec_on);
	ros::param::get("~agc_on", agc_on);
    ros::param::get("~hdr_on", hdr_on);
	ros::param::get("~expose_us", expose_us);
    ros::param::get("~frame_rate", frame_rate);
    ros::param::get("~cam_id", cam_id);
    ros::param::get("~serial", serial);

    BlueFOX_ROS *bluefox = 
        new BlueFOX_ROS(nh, binning_on,software_binning_on,software_binning_level, triggered_on, 
        aec_on, agc_on, hdr_on, expose_us, frame_rate, cam_id, serial);
    
    while(ros::ok())
    {
        ros::spinOnce();
	    bluefox->Publish();
    }

    delete bluefox;
    ROS_INFO_STREAM("Cease cameras.\n");
    return -1;
};
