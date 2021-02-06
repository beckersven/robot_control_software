/*
    WARNING!
    This is legacy code used in early development stages to implement laser-scanner-readouts of CoppeliaSim
    and does not work with the current CoppeliaSim-ROS-Framework. To make this work again, check out older branch 
    "custom_hardware_interface" on the ROS branch and an old branch "agiprobot_scene". However, using this code is highly NOT recommended!
*/



#include "../include/laser_scanner.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_scanner_node");
    ros::NodeHandle nh("~");
    coppeliasim_interface::LaserScanner ls;
    if(!ls.init(nh)){
        ROS_ERROR_STREAM("Could not connect to CoppeliaSim-Laserscanner.");
        exit(1);
    }
    ros::spin();
    return 0;
}