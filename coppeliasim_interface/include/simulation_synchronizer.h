#ifndef SIMULATION_SYNCHRONIZER_H
#define SIMULATION_SYNCHRONIZER_H

#include "rosgraph_msgs/Clock.h"
#include <vector>
#include <string>

extern "C" {
   #include "extApi.h"
}
#include <ros/ros.h>
namespace coppeliasim_interface{
   class SimulationSynchronizer{
    public:
        SimulationSynchronizer();
        ~SimulationSynchronizer();
        bool advanceSimulation();
        bool synchronizeROS();
        bool init(ros::NodeHandle& nh);
    private:
        ros::NodeHandle nh_;
        simxInt client_id;
        std::vector<ros::ServiceClient> explicit_sync_list;
        bool assertSimxCall(simxInt return_value, std::string error_message = "");
        ros::Publisher clock_pub;
        double sim_dt;
        rosgraph_msgs::Clock current_time;
        bool simulation_running;
    
    }; 
}

#endif
