
#include "../include/simulation_synchronizer.h"
#include <csignal>

std::unique_ptr<coppeliasim_interface::SimulationSynchronizer> sim_sync;
void sigHandler(int signum){
    sim_sync.reset();
    exit(signum);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "simulation_synchronizer_node");
    ros::NodeHandle nh("~");
    sim_sync.reset(new coppeliasim_interface::SimulationSynchronizer);
    signal(SIGINT, sigHandler);
    if(!sim_sync->init(nh)){
        ROS_ERROR_STREAM("Initialization failed - Simulation can not be synchronized with ROS!");
        exit(1);
    }
    while(ros::ok()){
        sim_sync->advanceSimulation();
        sim_sync->synchronizeROS();
    }
}