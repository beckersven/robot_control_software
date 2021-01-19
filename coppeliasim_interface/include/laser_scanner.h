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
            bool init(ros::NodeHandle& nh_);
            // bool set(simxInt resolution, simxFloat angle);
            bool sense(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
        private:
            // void applyNoise();
            simxFloat* measurements;
            std::default_random_engine noise_generator;
            std::normal_distribution<double> noise_distribution;
            double noise_intensity;
            ros::Publisher measurement_pub;
            ros::NodeHandle nh;
            simxInt resolution;
            simxFloat angle;
            std::vector<double> distances;
            tf2_ros::TransformBroadcaster tf_broad;
            simxInt client_id;
            simxInt scanner_focus_handle;
            std::string world_frame_id, scanner_focus_frame_id;
            ros::ServiceServer sense_srv;
    };
}