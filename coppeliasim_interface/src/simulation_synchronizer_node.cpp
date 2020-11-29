
#include "../include/simulation_synchronizer.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "simulation_synchronizer_node");
    ros::NodeHandle nh("~");
    coppeliasim_interface::SimulationSynchronizer sim_sync;
    if(!sim_sync.init(nh)){
        ROS_ERROR_STREAM("Initialization failed - Simulation can not be synchronized with ROS!");
        exit(1);
    }
    while(ros::ok()){
        sim_sync.advanceSimulation();
        sim_sync.synchronizeROS();
    }
}