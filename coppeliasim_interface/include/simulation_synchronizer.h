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
    /**
     * @brief Synchronizes the ROS-time with the CoppeliaSim-time in the sense, that both 'clocks' are running at the same speed (but not necessarely at the same time), and ensures that time-critical tasks in the ROS-domain are performed in sync with each simulation-step.
     * 
     */
    class SimulationSynchronizer{
    public:
        /**
         * @brief Construct a new Simulation Synchronizer object and initialize all members that do not necessarely need a ROS- or CoppeliaSim-connection.
         * 
         */
        SimulationSynchronizer();
        /**
         * @brief Destroy the Simulation Synchronizer object and stop the CoppeliaSim-Simulation and close the connection to CoppeliaSim.
         * 
         */
        ~SimulationSynchronizer();
        /**
         * @brief Advances the simulation in CoppeliaSim exactly one time-step (dt = 'sim-dt'). Blocks until CoppeliaSim has finished the calculations for this advancement.
         *
         * @returns Whether advancement was successful (could trigger calculations in CoppeliaSim and received success-feedback)
         */
        bool advanceSimulation();
        /**
         * @brief Increases ROS-time with the same dt as CoppeliaSim's 'sim-dt' and calls all ROS-services that should be handled during one time-step afterwards.
         * 
         * @returns Whether all Services were called successfully
         */
        bool synchronizeROS();
        /**
         * @brief Connects instance to CoppeliaSim (registers it as client for the remoteApi and starts simulation in synchroneous mode) and ROS (preparing /clock-publication and registering Services that should be handled every simulation-step)
         * 
         * @param nh Nodehandle to that this instance uses to communicate with ROS
         * 
         * @returns Whether the method could connect to ROS and CoppeliaSim
         */
        bool init(ros::NodeHandle& nh);
    private:
        /**
         * @brief Member to interact with the ROS-Framework
         * 
         */
        ros::NodeHandle nh_;
        /**
         * @brief ID specifying the connection to CoppeliaSim (must be used for every remoteApi-Call)
         * 
         */
        simxInt client_id;
        /**
         * @brief ROS-Services (std_srvs/Trigger) that must be handled every simulation step
         * 
         */
        std::vector<ros::ServiceClient> explicit_sync_list;
        /**
         * @brief Checks if the return_value corresponds to a successful CoppeliaSim-remoteApi-call and returns false if not. Additionally, a custom ROS-error is in this case logged when provided.
         * 
         * @param return_value Return value of a simx...-Call (call of CoppeliaSim-remoteApi-function)
         * @param error_message Message logged as ROS-Error when the return_value represents a return_value from a failed CoppeliaSim-remoteApi-call
         * @return Whether return_value masks a successfull CoppeliaSim-remoteApi-call
         */
        bool assertSimxCall(simxInt return_value, std::string error_message = "");
        /**
         * @brief ROS-Publisher used to publish the simulated time in '/clock'.
         *
         */
        ros::Publisher clock_pub;
        /**
         * @brief Time increment per simulation-step in CoppeliaSim. Is the same as '/clock' should increase per step in the ROS-domain
         * 
         */
        double sim_dt;
        /**
         * @brief For keeping track and publishing increasing time-messages.
         * 
         */
        rosgraph_msgs::Clock current_time;
        /**
         * @brief Flag indicating if the simulation is currently running in CoppeliaSim.
         * 
         */
        bool simulation_running;
    
    }; 
}

#endif
