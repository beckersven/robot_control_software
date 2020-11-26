
#ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
#define UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <ur_controllers/speed_scaling_interface.h>
#include <ur_controllers/scaled_joint_command_interface.h>


extern "C" {
   #include "extApi.h"
}

enum class PausingState
{
  PAUSED,
  RUNNING,
  RAMPUP
};

class CoppeliaSimInterface : public hardware_interface::RobotHW {
    public:
        CoppeliaSimInterface();
        virtual ~CoppeliaSimInterface();
        virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh) override;
        virtual void read(const ros::Time& time, const ros::Duration& period) override;
        virtual void write(const ros::Time& time, const ros::Duration& period) override;
        virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) override;
        virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                    const std::list<hardware_interface::ControllerInfo>& stop_list) override;
        uint32_t getControlFrequency() const;
        bool shouldResetControllers();
        bool connectToCoppeliaSim();
        bool disconnectFromCoppeliaSim();
        bool advanceCoppeliaSim();
        // bool startSimulation();
    protected:
        bool checkControllerClaims(const std::set<std::string>& claimed_resources);
        
        
    private:
        simxInt joint_handles_[6];
        double joint_velocity_command_[6];
        double joint_position_command_[6];
        double joint_positions_[6];
        double joint_efforts_[6];
        double joint_velocities_[6];
        std::vector<std::string> joint_names_;
        size_t n_joints = 6;


        uint32_t runtime_state_;
        bool position_controller_running_;
        bool velocity_controller_running_;

        PausingState pausing_state_;
        double pausing_ramp_up_increment_;

        bool controller_reset_necessary_;
        bool controllers_initialized_;
        bool coppeliasim_simulating_;

        double target_speed_scaling_;
        double speed_scaling_combined_;
        
        
        hardware_interface::JointStateInterface js_interface_;
        ur_controllers::ScaledPositionJointInterface spj_interface_;
        hardware_interface::PositionJointInterface pj_interface_;
        ur_controllers::SpeedScalingInterface speedsc_interface_;
        hardware_interface::VelocityJointInterface vj_interface_;
        ur_controllers::ScaledVelocityJointInterface svj_interface_;

        ros::NodeHandle root_nh;
        ros::NodeHandle robot_nh;
        simxInt client_id_;


       
};
#endif  // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
