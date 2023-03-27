#ifndef _BLUEFOX_ROS_H_
#define _BLUEFOX_ROS_H_

#include <iostream>
#include <vector>
#include <string>
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

class BlueFOX_ROS {
public:
    explicit BlueFOX_ROS(
        ros::NodeHandle& nh, bool binning_on, bool soft_bin_on, int soft_bin_lvl, bool trg_on,
        bool aec_on, bool agc_on, bool hdr_on, int expose_us, double frame_rate, int cam_id, std::string serial)
    : nh_(nh), it_(nh_)
    {
        int n_dev = getValidDevices(devMgr_, validDevices_);
        std::cout << "# of valid devices: " << n_dev << std::endl;
        // show devices information
        int idx_temp = -1;
        for(int j = 0; j < n_dev; j++) {
            cout << validDevices_[j]->serial.read() <<"\n";
            if(validDevices_[j]->serial.read() == serial){
                cout << "  idx    : " << j << endl;
                cout << "  valdev :" << validDevices_[j]->serial.read() <<"\n";
                cout << "  serial :" << serial << "\n";
                idx_temp = j;
            }
        }
        bluefox_ = new BlueFox(validDevices_[idx_temp], cam_id, binning_on, soft_bin_on, soft_bin_lvl, trg_on, 
                    aec_on, agc_on,hdr_on, expose_us, frame_rate);
        std::string topic_name = "/mono" + std::to_string(cam_id) + "/image_raw";

        image_publisher_ = it_.advertise(topic_name,1);
        img_msg_ = sensor_msgs::Image();

        // dynamic reconfigure for real-time hardware parameter settings
        // f = boost::bind(&BlueFOX_ROS::callbackDynReconfig, this, _1, _2);
        // server.setCallback(f);

        cout << "Please wait for setting cameras...\n";
        ros::Duration(0.5).sleep();
        cout << "camera setting is done.\n";
    }; 

    ~BlueFOX_ROS();

    void Publish();
    void callbackDynReconfig(bluefox_ros::bluefoxDynConfig &config, uint32_t lvl);

private:
    mvIMPACT::acquire::DeviceManager devMgr_; // Manager for all devices.

    vector<mvIMPACT::acquire::Device*> validDevices_; // multiple devices
    BlueFox* bluefox_;
    
    // For ros.
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::Publisher image_publisher_;
    sensor_msgs::Image img_msg_;

    dynamic_reconfigure::Server<bluefox_ros::bluefoxDynConfig> server;
    dynamic_reconfigure::Server<bluefox_ros::bluefoxDynConfig>::CallbackType f;
};

/* IMPLEMENTATION */
BlueFOX_ROS::~BlueFOX_ROS(){
    delete bluefox_;
};

//const sensor_msgs::ImagePtr& image_msg
void BlueFOX_ROS::Publish() {
    bluefox_->grabImage(img_msg_);
    img_msg_.header.stamp = ros::Time::now();
    image_publisher_.publish(img_msg_);
};


void BlueFOX_ROS::callbackDynReconfig(bluefox_ros::bluefoxDynConfig &config, uint32_t lvl) {
    if(config.hdr){
        bluefox_->setHighDynamicRange(true);
        cout << " DYNRECONFIG HDR:" <<  config.hdr << "\n";
    }
    else{
        bluefox_->setHighDynamicRange(false);
    }
        
    if(config.trigger_mode){
        bluefox_->setTriggerMode(true);
    }
    else{
        bluefox_->setTriggerMode(false);
    }
    
    if(config.binning_mode){
        bluefox_->setHardwareBinningMode(true);
    }
    else{
        bluefox_->setHardwareBinningMode(false);
    }

    if(config.aec){
        bluefox_->setAutoExposureMode(true);
        
    }
    else{
        bluefox_->setAutoExposureMode(false);
        bluefox_->setExposureTime(config.expose_us);
        
    }

    if(config.agc){
        bluefox_->setAutoGainMode(true);
    }
    else{
        bluefox_->setAutoGainMode(false);
    }

    if(config.wbp == -1){ // off
        bluefox_->setWhiteBalance(config.wbp,0,0,0);
    }
    else{ // on! each mode...
        bluefox_->setWhiteBalance(config.wbp,0,0,0);
    }
    
    ROS_INFO("Parameter reconfigured.\n");
}
#endif
