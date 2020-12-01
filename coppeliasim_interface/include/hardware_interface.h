
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



enum class RunState
{
  PAUSED,   // = Robot should not move anymore
  RUNNING,  // = Robot is moving normally (or standing still if the setpoint equals the current pose)
  RAMPUP    // = Robot is transitioning from PAUSED to RUNNING
};
namespace coppeliasim_interface{
    class HardwareInterface : public hardware_interface::RobotHW {
        public:
            // Initializes most of the class-members with reasonable values
            HardwareInterface();
            // Handles disconnection from CoppeliaSim
            virtual ~HardwareInterface();
            // Connects program to CoppeliaSim and obtains the required handles. Reads in the required
            // ROS-parameters and initializes all interfaces that are implemented in the real hardware-interface
            virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh) override;
            // Get the sensor-data from CoppeliaSim in a blocking way
            virtual void read(const ros::Time& time, const ros::Duration& period) override;
            // Set the sensor-data from CoppeliaSim in a blocking way
            virtual void write(const ros::Time& time, const ros::Duration& period) override;
            // (Obtained via inheritance)
            virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                const std::list<hardware_interface::ControllerInfo>& stop_list) override;
            // (Obtained via inheritace)
            virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

            // Service callbacks

            // Should normally tare the force-torque-sensor but is not implemented (service response gets delivered, 
            // but is always 'success == false')
            bool zeroFTSensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
            // Sets the speed-scaling as requested
            bool setSpeedSlider(ur_msgs::SetSpeedSliderFractionRequest& req, ur_msgs::SetSpeedSliderFractionResponse& res);
            // Changes the robot's RunState to PAUSED if positive and to RAMPUP if negative.
            // Performs basic checking so that the robot can not be paused if ramping up.
            bool setPause(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

        protected:
            bool checkControllerClaims(const std::set<std::string>& claimed_resources);
            
            
        private:
            // Joint handling
            std::vector<simxInt> joint_handles_;
            std::vector<double> joint_velocity_command_;
            std::vector<double> joint_position_command_;
            std::vector<double> joint_positions_;
            std::vector<double> joint_efforts_;
            std::vector<double> joint_velocities_;
            std::vector<std::string> joint_names_;
            
            // Force/torque handling
            std::vector<double> fts_measurements_;

            // Controller status information
            bool position_controller_running_;
            bool velocity_controller_running_;
            bool controllers_initialized_;
            industrial_robot_status_interface::RobotStatus robot_status_resource_{};
    
            // ros_control-interfaces
            hardware_interface::JointStateInterface js_interface_;
            ur_controllers::ScaledPositionJointInterface spj_interface_;
            hardware_interface::PositionJointInterface pj_interface_;
            ur_controllers::SpeedScalingInterface speedsc_interface_;
            hardware_interface::VelocityJointInterface vj_interface_;
            ur_controllers::ScaledVelocityJointInterface svj_interface_;
            hardware_interface::ForceTorqueSensorInterface fts_interface_;
            industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{}; // <- Gives just static 'dummy' information. If necessary, this can easily be expanded.
        
            // General ROS- and CoppeliaSim-interface handles
            ros::NodeHandle root_nh;
            ros::NodeHandle robot_nh;
            simxInt client_id_;
                
            // Robot speed-scaling
            RunState run_state_;
            double pausing_ramp_up_increment_;
            double speed_scaling_;
            double target_speed_fraction_;
            double speed_scaling_combined_;
            
            // Services
            ros::ServiceServer pause_button_srv_;
            ros::ServiceServer set_speed_slider_srv_;
            ros::ServiceServer tare_sensor_srv_;
    
    };
}

#endif  // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
