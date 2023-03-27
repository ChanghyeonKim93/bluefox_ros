#ifndef _BLUEFOX_MULTIPLE_ROS_H_
#define _BLUEFOX_MULTIPLE_ROS_H_

#include <iostream>
#include <vector>
#include <sys/time.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <apps/Common/exampleHelper.h>
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#include "bluefox.h"

#include "dynamic_reconfigure/server.h"
#include "bluefox_ros/bluefoxDynConfig.h"


using namespace std;
using namespace mvIMPACT::acquire;

class BlueFOX_MULTIPLE_ROS {
public:
    explicit BlueFOX_MULTIPLE_ROS(
        ros::NodeHandle& nh, bool binning_on, bool software_binning_on, int software_binning_level, bool triggered_on,
        bool aec_on, bool agc_on, bool hdr_on, int expose_us, double frame_rate)
    : nh_(nh), it_(nh_)
    {
        n_devs_ = getValidDevices(devMgr_, validDevices_);
        std::cout << "# of valid devices: " << n_devs_ << std::endl;
        // show devices information
        for(int i = 0; i < n_devs_; i++){
            std::cout << "[" << i << "]: ";
            BlueFox* bluefox_temp = 
                        new BlueFox(validDevices_[i], i, binning_on, software_binning_on, software_binning_level,triggered_on, 
                        aec_on, agc_on,hdr_on, expose_us, frame_rate);
            std::string topic_name = "/" + std::to_string(i) + "/image_raw";

            bluefoxs_.push_back(bluefox_temp);

            image_transport::Publisher camera_pub_ = it_.advertise(topic_name,1);
            image_publishers_.push_back(camera_pub_);
            img_msgs_.push_back(sensor_msgs::Image());
        }

        // dynamic reconfigure for real-time hardware parameter settings
        f = boost::bind(&BlueFOX_MULTIPLE_ROS::callbackDynReconfig, this, _1, _2);
        server.setCallback(f);

        cout << "Please wait for setting cameras...\n";
        ros::Duration(0.5).sleep();
        cout << "camera setting is done.\n";
    }; 

    ~BlueFOX_MULTIPLE_ROS();

    void Publish();
    void callbackDynReconfig(bluefox_ros::bluefoxDynConfig &config, uint32_t lvl);


private:
    int n_devs_; // # of connected mvBlueCOUGAR cameras.
    mvIMPACT::acquire::DeviceManager devMgr_; // Manager for all devices.

    vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices
    vector<BlueFox*> bluefoxs_;
    
    
    // For ros.
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    vector<image_transport::Publisher> image_publishers_;
    vector<sensor_msgs::Image> img_msgs_;

    ros::Subscriber sub_msg_;
    std_msgs::Int32 msg_;

    dynamic_reconfigure::Server<bluefox_ros::bluefoxDynConfig> server;
    dynamic_reconfigure::Server<bluefox_ros::bluefoxDynConfig>::CallbackType f;
};

/* IMPLEMENTATION */
BlueFOX_MULTIPLE_ROS::~BlueFOX_MULTIPLE_ROS(){
    for(int i = 0; i < n_devs_; i++){
        delete bluefoxs_[i];
    }
};

//const sensor_msgs::ImagePtr& image_msg
void BlueFOX_MULTIPLE_ROS::Publish() {
    for(int i = 0; i < n_devs_; i++){
        bluefoxs_[i]->grabImage(img_msgs_[i]);
    }   
    for(int i = 0; i <n_devs_; i++){
        image_publishers_[i].publish(img_msgs_[i]);
    }
};


void BlueFOX_MULTIPLE_ROS::callbackDynReconfig(bluefox_ros::bluefoxDynConfig &config, uint32_t lvl) {
    if(config.hdr){
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setHighDynamicRange(true);
        cout << " DYNRECONFIG HDR:" <<  config.hdr << "\n";
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setHighDynamicRange(false);
    }
        
    if(config.trigger_mode){
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setTriggerMode(true);
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setTriggerMode(false);
    }
    
    if(config.binning_mode){
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setHardwareBinningMode(true);
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setHardwareBinningMode(false);
    }

    if(config.aec){
        for(int i = 0; i < n_devs_; i++){
            bluefoxs_[i]->setAutoExposureMode(true);
        }
    }
    else{
        for(int i = 0; i < n_devs_; i++){
            bluefoxs_[i]->setAutoExposureMode(false);
            bluefoxs_[i]->setExposureTime(config.expose_us);
        }
    }

    if(config.agc){
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setAutoGainMode(true);
    }
    else{
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setAutoGainMode(false);
    }

    if(config.wbp == -1){ // off
       for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setWhiteBalance(config.wbp,0,0,0);
    }
    else{ // on! each mode...
        for(int i = 0; i < n_devs_; i++)
            bluefoxs_[i]->setWhiteBalance(config.wbp,0,0,0);
    }
    
    ROS_INFO("Parameter reconfigured.\n");
}
#endif
