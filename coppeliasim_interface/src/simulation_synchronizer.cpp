#include "../include/simulation_synchronizer.h"
#include "std_srvs/Trigger.h"
#include <thread>
namespace coppeliasim_interface{
    bool SimulationSynchronizer::assertSimxCall(simxInt return_value, std::string error_message){
        if(return_value == simx_return_ok){
            return true;
        }
        else if(return_value != simx_return_ok && error_message.length() == 0){
            return false;
        }
        else{
            ROS_ERROR_STREAM(error_message);
            return false;
        }
    }
    SimulationSynchronizer::SimulationSynchronizer(){
        explicit_sync_list.resize(0);
        client_id = -1;
        current_time.clock = ros::Time(0);
        simulation_running = false;
    }

    SimulationSynchronizer::~SimulationSynchronizer(){
        if(simulation_running){
            simxStopSimulation(client_id, simx_opmode_oneshot);
            ROS_DEBUG_STREAM("Stopped CoppeliaSim simulation");
        }
        if(client_id != -1){
            simxFinish(client_id);
            ROS_DEBUG_STREAM("Disconnected from CoppeliaSim");
        }
        return;
        
    }

    bool SimulationSynchronizer::init(ros::NodeHandle& nh){
        nh_ = nh;

        // Connect to CoppeliaSim
        // ^^^^^^^^^^^^^^^^^^^^^^
        // Resolve required parameters
        simxInt coppelia_port, connection_timeout_ms;
        if(!nh_.getParam("/coppelia_config/PORT_simulation_synchronizer", coppelia_port)
            || !nh_.getParam("/coppelia_config/sim_dt", sim_dt)
            || !nh_.getParam("/coppelia_config/connection_timeout_ms", connection_timeout_ms)){
            ROS_ERROR_STREAM("Could not resolve CoppeliaSim-config. Expected parameters: " << std::endl 
                << nh_.resolveName("/coppelia_config/PORT_simulation_synchronizer") << std::endl 
                << nh_.resolveName("/coppelia_config/sim_dt") << std::endl
                << nh_.resolveName("/coppelia_config/connection_timeout_ms") << std::endl);
            return false;
        }

        // Establish connection
        client_id = simxStart("127.0.0.1", coppelia_port, true, true, connection_timeout_ms, 5);
        if(client_id == -1){
            ROS_ERROR_STREAM("CoppeliaSim is not available at 127.0.0.1:" << coppelia_port << "!");
            return false;
        }

        // Configure the simulation to match the parameter-specification
        // simxStopSimulation(client_id, simx_opmode_blocking);
        if(!assertSimxCall(simxSetFloatingParameter(client_id, sim_floatparam_simulation_time_step, sim_dt, simx_opmode_blocking), "Can not set 'dt' (must be set 'custom' in CoppeliaSim)!")
            || !assertSimxCall(simxSynchronous(client_id, true), "Can not set CoppeliaSim to synchronous-mode!")) 
                return false;
        ROS_DEBUG_STREAM("Simulation-synchronizer is connected to CoppeliaSim!");



        // Synchronize the ROS-time "/clock"
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // This is the part where the actual ROS-time becomes synchronized with
        // the CoppeliaSim-time. This is important as for small 'sim_dt', the time to 
        // compute the simualation-step is bigger than 'sim_dt' and clock-skew would occur.
        bool use_sim_time = false;
        if(!nh_.getParam("/use_sim_time", use_sim_time) || !use_sim_time){
            ROS_ERROR_STREAM("Not using sim-time which means ROS-time and CoppeliaSim's simulation-time can not be synchronized!");
            return false;
        }
        clock_pub = nh_.advertise<rosgraph_msgs::Clock>("/clock", 10, false);
        clock_pub.publish(current_time);
        ROS_DEBUG_STREAM("/clock-Publisher available - ROS-time and CoppeliaSim's simulation-time will be synchronized!");

        // Last step is to start the simulation
        if(! assertSimxCall(simxStartSimulation(client_id, simx_opmode_blocking), "Could not start simulation")) return false;
        simulation_running = true;
        
        this->advanceSimulation();
        this->synchronizeROS();

        // Register the explicit_sync-services
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // These services get called every simulation step. The simulation will not
        // advance until every service call responded (sucessfully). 
        // Note, that the services should only be advertised
        // when the related code is fully initialized, e.g. as the last step of the 'init()'-method. This works
        // as a guarantee to ensure that these nodes are ready when this loop finishes (i.e. that the simulation can start).
        std::vector<std::string> explicit_sync_list_names;
        if(nh_.getParam("/coppelia_config/explicit_sync_list", explicit_sync_list_names)){
            for(size_t i = 0; i < explicit_sync_list_names.size(); i++){
                if(!ros::service::waitForService(explicit_sync_list_names[i], ros::Duration(3))){
                    ROS_ERROR_STREAM("Could not find service " << explicit_sync_list_names[i]);
                    return false;
                }
                explicit_sync_list.push_back(nh_.serviceClient<std_srvs::Trigger>(explicit_sync_list_names[i], true));
            } 
        }
        ROS_DEBUG_STREAM("Connected to " << explicit_sync_list_names.size() << " explicit sync services!");
        return true;

    }

    bool SimulationSynchronizer::advanceSimulation(){
        // Work with 'return_value'-flag to ensure execution-order (trigger before pinging)
        simxInt ping;
        bool return_value = assertSimxCall(simxSynchronousTrigger(client_id), "Could not trigger next simulation step");
        return return_value && assertSimxCall(simxGetPingTime(client_id, &ping), "Simulation not available after simulation-step");
    }

    bool SimulationSynchronizer::synchronizeROS(){
        // First, update ROS-time by publishing to '/clock'
        current_time.clock = current_time.clock + ros::Duration(sim_dt);
        clock_pub.publish(current_time);
        ROS_DEBUG_STREAM("Updated ROS-/clock");

        // Handle explicit_sync-services via parallel threads to improve performance
        ROS_DEBUG_STREAM("Calling explicit_sync-services");
        // Start the threads
        bool success_flag = true;
        // Wait for thread-termination and evaluate result
        for(size_t i = 0; i < explicit_sync_list.size(); i++){
            std_srvs::Trigger tr;
            explicit_sync_list[i].call(tr);
            if(tr.response.success == false){
                ROS_ERROR_STREAM("Received message during service handling: " + tr.response.message);
                success_flag = false;
            }
        }
        return success_flag;
    }



}
