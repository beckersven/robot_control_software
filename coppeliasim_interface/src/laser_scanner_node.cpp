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