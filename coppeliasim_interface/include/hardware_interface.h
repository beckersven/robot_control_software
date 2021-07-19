
#ifndef COPPELIA_HARDWARE_INTERFACE_H_INCLUDED
#define COPPELIA_HARDWARE_INTERFACE_H_INCLUDED

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float64MultiArray.h>
#include <ur_controllers/speed_scaling_interface.h>
#include <ur_controllers/scaled_joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include "tf2_msgs/TFMessage.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ur_msgs/SetSpeedSliderFraction.h"
#include <industrial_robot_status_interface/industrial_robot_status_interface.h>



extern "C" {
   #include "extApi.h"
}


/**
 * @brief To efficiently treat the run-state
 * 
 */
enum class RunState
{
  PAUSED,   // = Robot should not move anymore
  RUNNING,  // = Robot is moving normally (or standing still if the setpoint equals the current pose)
  RAMPUP    // = Robot is transitioning from PAUSED to RUNNING
};
namespace coppeliasim_interface{
    /**
     * @brief Hardware-Interface for the UR10e-model in CoppeliaSim that behaves just like the driver for the real robot.
     * 
     */
    class HardwareInterface : public hardware_interface::RobotHW {
        public:
            /**
             * @brief Construct a new Hardware Interface object and initialize all class members that do not require a ROS- or CoppeliaSim-connection
             * 
             */
            HardwareInterface();
            /**
             * @brief Destroy the Hardware Interface object and handle disconnection from CoppeliaSim
             * 
             */
            virtual ~HardwareInterface();
            // 
            /**
             * @brief Connects program to CoppeliaSim and obtains the required handles. Reads in the required ROS-parameters and initializes all interfaces that are implemented in the real hardware-interface
             * 
             * @param root_nh ROS-nodehandle for operations at root-level (names starting with '/...')
             * @param robot_nh ROS-nodehandle for operations in the hardware-interface-nodes namespace
             * @returns Whether connection to CoppeliaSim was successful and the hardware-interfaces could be set-up work with CoppeliaSim-Joints 
             */
            virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh) override;
            /**
             * @brief Gets the joint-data (position, velocity, torque) from CoppeliaSim in a blocking way
             * 
             * @param time Required by ROS-HardwareInterface specification (fullfills nothing here)
             * @param period Required by ROS-HardwareInterface (fullfills nothing here)
             */
            virtual void read(const ros::Time&, const ros::Duration&) override;
            /**
             * @brief Set the joint-target-values (velocities or positions, depending on active controller) in CoppeliaSim in a blocking way
             * 
             */
            virtual void write(const ros::Time&, const ros::Duration&) override;
            // (Obtained via inheritance)
            virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                const std::list<hardware_interface::ControllerInfo>& stop_list) override;
            // (Obtained via inheritace)
            /**
             * @brief Tries to turn on or off provided controllers. The provided controllers must be applicable with this HardwareInterface, otherwise they do not trigger any changes.
             * 
             * @param start_list Controllers to start
             * @param stop_list Controllers to stop
             */
            virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

            // Service callbacks

            // 
            /**
             * @brief Should normally tare the force-torque-sensor but is not implemented for performance reasons (service response gets delivered, but is always 'success == false')
             * 
             * @param req (contains no information)
             * @param res Trigger-Response (currently always 'success=false' with a corresponding message as this is not implemented)
             * @returns always 'true'
             */
            bool zeroFTSensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
            
            /**
             * @brief Sets the speed-scaling as requested
             * 
             * @param req Request with target value of the speed slider
             * @param res Response with information about whether the speed slider could be set (i.e. target lies in the interval [0.01,1])
             * @returns always 'true'
             */
            bool setSpeedSlider(ur_msgs::SetSpeedSliderFractionRequest& req, ur_msgs::SetSpeedSliderFractionResponse& res);
            // Changes the robot's RunState to PAUSED if positive and to RAMPUP if negative.
            // Performs basic checking so that the robot can not be paused if ramping up.
            /**
             * @brief Controls if the robots running state (RUNNING = normal, PAUSED = halted, RAMPUP = Accelerating from PAUSED to RUNNING)
             * 
             * @param req Contains boolean flag about which run-state should be entered
             * @param res If the desired state could be set (false, if in RAMPUP or if the desired state is already set) with an descriptive message
             * @returns always 'true'
             */
            bool setPause(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

        protected:
            /**
             * @brief Check if the HardwareInterface can command the joined named in claimed_resources
             * 
             * @param claimed_resources Joint-names to check
             * @returns If this HardwareInterface has command over the provided named joints
             */
            bool checkControllerClaims(const std::set<std::string>& claimed_resources);
            
            
        private:
            // Joint handling

            /**
             * @brief CoppeliaSim handles for each commanded joint
             * 
             */
            std::vector<simxInt> joint_handles_;
            /**
             * @brief Target joint velocity for each commanded joint (only for velocity-control)
             * 
             */
            std::vector<double> joint_velocity_command_;
            /**
             * @brief Target joint position for each commanded joint (only for position-control)
             * 
             */
            std::vector<double> joint_position_command_;
            /**
             * @brief Joint position of each commanded joint
             * 
             */
            std::vector<double> joint_positions_;
            /**
             * @brief Torque of each commanded joint
             * 
             */
            std::vector<double> joint_efforts_;
            /**
             * @brief (Angular) velocities of each commanded joint
             * 
             */
            std::vector<double> joint_velocities_;
            /**
             * @brief Name of each commanded joint
             * 
             */
            std::vector<std::string> joint_names_;
            

            // Controller status information

            /**
             * @brief Flag indicating whether position controller is running
             * 
             */
            bool position_controller_running_;
            /**
             * @brief Flag indicating whether velocity controller is running
             * 
             */
            bool velocity_controller_running_;
            /**
             * @brief Flag indicating if the controllers have been initialized
             * 
             */
            bool controllers_initialized_;
            /**
             * @brief Status handle to imitate a RobotStatus of a real robot (is constant however)
             * 
             */
            industrial_robot_status_interface::RobotStatus robot_status_resource_{};
    
            // ros_control-interfaces

            /**
             * @brief HW-interface to read out joint positions and publish them similar to a joint state publisher.
             * 
             */
            hardware_interface::JointStateInterface js_interface_;
            /**
             * @brief Custom HW-interface for scaled position control
             * 
             */
            ur_controllers::ScaledPositionJointInterface spj_interface_;
            /**
             * @brief Official HW-interface for joint position control
             * 
             */
            hardware_interface::PositionJointInterface pj_interface_;
            /**
             * @brief Custom HW-interface for speed_scaling control
             * 
             */
            ur_controllers::SpeedScalingInterface speedsc_interface_;
            /**
             * @brief Official HW-interface for joint velocity control
             * 
             */
            hardware_interface::VelocityJointInterface vj_interface_;
            /**
             * @brief Custom HW-interface for scaled joint velocity control
             * 
             */
            ur_controllers::ScaledVelocityJointInterface svj_interface_;
            /**
             * @brief Official HW-interface for force-torque-sensor handling
             * 
             */
            hardware_interface::ForceTorqueSensorInterface fts_interface_;
            /**
             * @brief Official HW-interface to monitor the status of the robot (gives currently static 'demo'-information)
             * 
             */
            industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{}; 
        
            // General ROS- and CoppeliaSim-interface handles

            /**
             * @brief ROS-nodehandle for operations at root-level (names starting with '/...')
             * 
             */
            ros::NodeHandle root_nh;
            /**
             * @brief ROS-nodehandle for operations in the hardware-interface-nodes namespace
             * 
             */
            ros::NodeHandle robot_nh;
            /**
             * @brief ID specifying the connection to CoppeliaSim (must be used for every remoteApi-Call)
             * 
             */
            simxInt client_id_;
                
            // Robot speed-scaling

            /**
             * @brief Indicator whether the robot is RUNNING (speed_scaling_combined_=1), in PAUSED (speed_scaling_combined_=0) state or RAMPingUP from PAUSED to RUNNING
             * 
             */
            RunState run_state_;
            /**
             * @brief combined_speed_scaling-increment per readout until combined_speed_scaling is equal to speed_scaling_ (= target)
             * 
             */
            double pausing_ramp_up_increment_;
            /**
             * @brief target speed_scaling for operation of the scaled controllers
             * 
             */
            double speed_scaling_;
            /**
             * @brief actual speed_scaling used by the scaled controllers
             * 
             */
            double speed_scaling_combined_;
            
            // Services

            /**
             * @brief Service to pause the robot's movement or to restart movement from a paused state
             * 
             */
            ros::ServiceServer pause_button_srv_;
            /**
             * @brief Service to set the speed slider (lets the robot's joints move at a fraction of their velocity)
             * 
             */
            ros::ServiceServer set_speed_slider_srv_;
            /**
             * @brief Service to tare the Force-Torque-Sensor in the TCP (advertised, but fullfills no purpose)
             * 
             */
            ros::ServiceServer tare_sensor_srv_;
    
    };
}

#endif  // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
