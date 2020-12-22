extern "C" {
   #include "extApi.h"
}

#include "sensor_msgs/PointCloud.h"
#include <vector>
#include <random>
//#include <tf2_ros/transform_broadcaster.h>
#include "std_srvs/Trigger.h"
#include <ros>
#include <string>
#include <map>

namespace coppeliasim_interface{
    class LaserScanner{
        public:
            LaserScanner();
            ~LaserScanner();
            bool init(ros::NodeHandle& nh_);
            // bool set(simxInt resolution, simxFloat angle);
            bool sense(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
        private:
            double calculateUncertainty(simxFloat z, simxFloat gamma);
            std::map<std::string, double> laser_scanner_parameters;

            simxFloat* measurements;
            ros::Publisher measurement_pub;
            ros::NodeHandle nh;

            std::vector<double> distances;
            tf2_ros::TransformBroadcaster tf_broad;
            simxInt client_id;
            simxInt scanner_focus_handle;
            std::string world_frame, scanner_focus_frame;
            ros::ServiceServer sense_srv;
    };
}