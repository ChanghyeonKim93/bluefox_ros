#include <iostream>
#include <ros/ros.h>

#include "bluefox_multiple_ros.h"
#include "dynamic_reconfigure/server.h"
#include "bluefox/bluefoxDynConfig.h"

void callbackDynReconfig(bluefox::bluefoxDynConfig &config, uint32_t lvl){
    ROS_INFO("Reconfigure Request: %d",config.int_param);
}
// trigger pin = digIn0+
// trigger ground level is same with the Arduino ground level.
// TODO: recognize serial numbers of multiple cameras and grant specific "id".
// TODO: multiple cameras setting is not supported yet .. T.T
int main(int argc, char **argv) {
    std::cout << "BlueFOX node is running..." << std::endl;
    ros::init(argc, argv, "bluefox_multiple_node");
    dynamic_reconfigure::Server<bluefox::bluefoxDynConfig> server;
    dynamic_reconfigure::Server<bluefox::bluefoxDynConfig>::CallbackType f;
    f = boost::bind(&callbackDynReconfig, _1, _2);
    server.setCallback(f);

    ros::NodeHandle nh("~");

    bool binning_on   = false;
    bool triggered_on = false;
    bool aec_on       = false; // auto exposure control on / off
    bool agc_on       = false; // auto gain control on / off
    int expose_us     = 11000; // it is also max. exposure for auto exposure control.
    double frame_rate = 30.0; // frame rate (full resolution: up to 30 Hz)

    ros::param::get("~binning_on", binning_on);
	ros::param::get("~triggered_on", triggered_on);
	ros::param::get("~aec_on", aec_on);
	ros::param::get("~agc_on", agc_on);
	ros::param::get("~expose_us", expose_us);
    ros::param::get("~frame_rate", frame_rate);

    BlueFOX_MULTIPLE_ROS *bluefoxs = 
        new BlueFOX_MULTIPLE_ROS(nh, binning_on, triggered_on, 
        aec_on, agc_on, expose_us, frame_rate);
    
    while(ros::ok())
    {
	bluefoxs->Publish();
        ros::spinOnce();
    }

    delete bluefoxs;
    ROS_INFO_STREAM("Cease cameras.\n");
    return -1;
};
