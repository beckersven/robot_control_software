
#ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
#define UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED

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
        void extractToolPose(const ros::Time& timestamp);
        void transformForceTorque();
        void publishPose();
        bool zeroFTSensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
        bool setSpeedSlider(ur_msgs::SetSpeedSliderFractionRequest& req, ur_msgs::SetSpeedSliderFractionResponse& res);
        bool setPause(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

    protected:
        bool checkControllerClaims(const std::set<std::string>& claimed_resources);
        
        
    private:
        std::vector<simxInt> joint_handles_;
        std::vector<double> joint_velocity_command_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_positions_;
        std::vector<double> joint_efforts_;
        std::vector<double> joint_velocities_;
        std::vector<double> fts_measurements_;
        std::vector<std::string> joint_names_;
        
        size_t n_joints = 6;

        ros::ServiceServer tare_sensor_srv_;
        ros::ServiceServer deactivate_srv_;

        uint32_t runtime_state_;
        bool position_controller_running_;
        bool velocity_controller_running_;

        RunState run_state_;
        double pausing_ramp_up_increment_;

        bool controller_reset_necessary_;
        bool controllers_initialized_;
        bool coppeliasim_simulating_;
        industrial_robot_status_interface::RobotStatus robot_status_resource_{};
        industrial_robot_status_interface::IndustrialRobotStatusInterface robot_status_interface_{};

        
        
        hardware_interface::JointStateInterface js_interface_;
        ur_controllers::ScaledPositionJointInterface spj_interface_;
        hardware_interface::PositionJointInterface pj_interface_;
        ur_controllers::SpeedScalingInterface speedsc_interface_;
        hardware_interface::VelocityJointInterface vj_interface_;
        ur_controllers::ScaledVelocityJointInterface svj_interface_;
        hardware_interface::ForceTorqueSensorInterface fts_interface_;

        std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> tcp_pose_pub_;

        ros::Publisher pub;
        ros::NodeHandle root_nh;
        ros::NodeHandle robot_nh;
        simxInt client_id_;
        std::string tf_prefix_;

        tf2::Vector3 tcp_torque_;
        tf2::Vector3 tcp_force_;
        simxInt ft_sensor_hande_;
        simxInt tcp_handle_;
        simxInt base_handle_;
        simxFloat tcp_pose_[7];
        geometry_msgs::TransformStamped tcp_transform_;
        double speed_scaling_;
        double target_speed_fraction_;
        double speed_scaling_combined_;
        ros::ServiceServer pause_button_srv_;
        ros::ServiceServer set_speed_slider_srv_;

        


       
};
#endif  // ifndef UR_DRIVER_HARDWARE_INTERFACE_H_INCLUDED
