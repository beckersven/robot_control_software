/*
    WARNING!
    This is legacy code used in early development stages to implement laser-scanner-readouts of CoppeliaSim
    and does not work with the current CoppeliaSim-ROS-Framework. To make this work again, check out older branch 
    "custom_hardware_interface" on the ROS branch and an old branch "agiprobot_scene". However, using this code is highly NOT recommended!
*/

extern "C" {
   #include "extApi.h"
}
#include "sensor_msgs/PointCloud.h"
#include <vector>
#include <random>
#include <tf2_ros/transform_broadcaster.h>
#include "std_srvs/Trigger.h"
#include <ros/ros.h>
#include <string>
namespace coppeliasim_interface{
    class LaserScanner{
        public:
            LaserScanner();
            ~LaserScanner();
            // Initialize LaserScanner so that all required members are resolved and services/topics/... advertised
            bool init(ros::NodeHandle& nh_);            
        private:
            // Pointer on read-in meausrements
            simxFloat* measurements;
            // Performs one line scan and publishes perceived point-cloud
            bool sense(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
            // Members used to apply noise on the measurements
            std::default_random_engine noise_generator;
            std::normal_distribution<double> noise_distribution;
            double noise_intensity;

            // To publish the measured point-cloud
            ros::Publisher measurement_pub;
            // For interaction with the ROS ecosystem
            ros::NodeHandle nh;
            // Number of points per scan
            simxInt resolution;
            // Fan angle (= opening angle of the fan) of the laser-scanner
            simxFloat angle;
            // Measured distances from the scanner-frame to the first object-hit (maximum index = resolution - 1)
            std::vector<double> distances;
            // To send out the position of the scanner-frame
            tf2_ros::TransformBroadcaster tf_broad;
            // CoppeliaSim-Connection variable
            simxInt client_id;
            // Handle of the "dummy" scanner_focus in CoppeliaSim
            simxInt scanner_focus_handle;
            std::string world_frame_id, scanner_focus_frame_id;
            // When called, the sense() method will be performed
            ros::ServiceServer sense_srv;
    };
}