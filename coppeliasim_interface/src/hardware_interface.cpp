#include "../include/hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>


namespace coppeliasim_interface{
  HardwareInterface::HardwareInterface(){
      joint_names_.resize(6, "");
      joint_efforts_.resize(6, 0);
      joint_handles_.resize(6, -1);
      joint_position_command_.resize(6, 0);
      joint_positions_.resize(6, 0);
      joint_velocities_.resize(6, 0);
      joint_velocity_command_.resize(6, 0);
      fts_measurements_.resize(6, 0);
      pausing_ramp_up_increment_ = 0.01;
      target_speed_fraction_ = 1;
      run_state_ = RunState::RUNNING;
      speed_scaling_ = 1;
      robot_status_resource_.drives_powered = industrial_robot_status_interface::TriState::TRUE;
      robot_status_resource_.e_stopped = industrial_robot_status_interface::TriState::FALSE;
      robot_status_resource_.error_code = 0;
      robot_status_resource_.in_error = industrial_robot_status_interface::TriState::FALSE;
      robot_status_resource_.in_motion = industrial_robot_status_interface::TriState::FALSE;
      robot_status_resource_.mode = industrial_robot_status_interface::RobotMode::AUTO;
      robot_status_resource_.motion_possible = industrial_robot_status_interface::TriState::FALSE;
      client_id_ = -1;
      speed_scaling_combined_ = 1;
      tcp_position_.resize(3, 0);
      tcp_orientation_.resize(4, 0);
      tcp_orientation_[3] = 1;  // Make valid quaternion default
  }
  HardwareInterface::~HardwareInterface(){
      if(client_id_ != -1){
          simxFinish(client_id_);
          ROS_DEBUG_STREAM("Disconnected from CoppeliaSim");
      }
  }
  bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh){
      this->root_nh = root_nh;
      this->robot_nh = robot_nh;
      std::string wrench_frame_id;
      simxInt coppelia_port, connection_timeout_ms;
      if(!root_nh.getParam("/coppelia_config/PORT_hardware_interface", coppelia_port)
            || !root_nh.getParam("/coppelia_config/connection_timeout_ms", connection_timeout_ms)){
            ROS_ERROR_STREAM("Could not resolve CoppeliaSim-config. Expected parameters: " << std::endl 
                << root_nh.resolveName("/coppelia_config/PORT_simulation_synchronizer") << std::endl 
                << root_nh.resolveName("/coppelia_config/sim_dt") << std::endl
                << root_nh.resolveName("/coppelia_config/connection_timeout_ms") << std::endl);
            return false;
        }

      client_id_ = simxStart("127.0.0.1", coppelia_port, true, true, 1000, 5);
      if(client_id_ == -1){
          ROS_ERROR_STREAM("Connection to CoppeliaSim failed using 127.0.0.1:" << coppelia_port);
          return false;
      }

      tcp_transform_.header.frame_id = tf_prefix_ + "base";
      tcp_transform_.child_frame_id = tf_prefix_ + "tool0_controller";
      
      robot_nh.param<std::string>("wrench_frame_id", wrench_frame_id, "wrench");
      if(!this->root_nh.getParam("hardware_interface/joints", joint_names_)){
          ROS_ERROR_STREAM("Could not find joint-names in " << this->root_nh.resolveName("hardware_interface/joints"));
          return false;
      }
      std::vector<simxFloat> dummy(4);
      // Create ros_control interfaces
      for (std::size_t i = 0; i < joint_names_.size(); ++i)
      {
          ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);
          // Create joint state interface for all joints
          js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                          &joint_velocities_[i], &joint_efforts_[i]));
          if(! simxGetObjectHandle(client_id_, joint_names_[i].c_str(), &joint_handles_[i], simx_opmode_blocking) == simx_return_ok){
              ROS_ERROR_STREAM("Can not resolve " << joint_names_[i] << " in CoppeliaSim");
              return false;
          }
          // Set the mode to streaming so that frequent evaluation becomes more lightweight
          simxGetJointPosition(client_id_, joint_handles_[i], &dummy[0], simx_opmode_streaming);
              // 2012 = "magic-number" from CoppeliaSim-API-constants representing joint-velocity
          simxGetObjectFloatParameter(client_id_, joint_handles_[i], 2012, &dummy[0], simx_opmode_streaming);
          simxGetJointForce(client_id_, joint_handles_[i], &dummy[0], simx_opmode_streaming);
          
          


          // Create joint position control interface
          pj_interface_.registerHandle(
              hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
          vj_interface_.registerHandle(
              hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i]));
          spj_interface_.registerHandle(ur_controllers::ScaledJointHandle(
              js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i], &speed_scaling_combined_));
          svj_interface_.registerHandle(ur_controllers::ScaledJointHandle(
              js_interface_.getHandle(joint_names_[i]), &joint_velocity_command_[i], &speed_scaling_combined_));
      }

      speedsc_interface_.registerHandle(
          ur_controllers::SpeedScalingHandle("speed_scaling_factor", &speed_scaling_combined_));

      if(simxGetObjectHandle(client_id_, "tool0", &ft_sensor_hande_, simx_opmode_blocking) != simx_return_ok
        || simxGetObjectHandle(client_id_, "TCP", &tcp_handle_, simx_opmode_blocking) != simx_return_ok
        || simxGetObjectHandle(client_id_, "base", &base_handle_, simx_opmode_blocking) != simx_return_ok){
          ROS_ERROR_STREAM("Could not resolve names for fts-interface or TCP-handling");
          return false;
        }
      simxGetObjectPosition(client_id_, tcp_handle_, base_handle_, &dummy[0], simx_opmode_streaming);
      simxGetObjectQuaternion(client_id_, tcp_handle_, base_handle_, &dummy[0], simx_opmode_streaming);
      simxReadForceSensor(client_id_, ft_sensor_hande_, NULL, &dummy[0], &dummy[0], simx_opmode_streaming);
      
      fts_interface_.registerHandle(hardware_interface::ForceTorqueSensorHandle(
            wrench_frame_id, tf_prefix_ + "tool0_controller", &fts_measurements_[0], &fts_measurements_[3]));
          registerInterface(&fts_interface_);
          ROS_DEBUG_STREAM("Initialized tool-handling (force-torque-interface and TCP-pose)");

      robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
          "industrial_robot_status_handle", robot_status_resource_));
      // Register interfaces
      registerInterface(&js_interface_);
      registerInterface(&spj_interface_);
      registerInterface(&pj_interface_);
      registerInterface(&vj_interface_);
      registerInterface(&svj_interface_);
      registerInterface(&speedsc_interface_);
      registerInterface(&robot_status_interface_);
      tare_sensor_srv_ = robot_nh.advertiseService("zero_ftsensor", &HardwareInterface::zeroFTSensor, this);
      tcp_pose_pub_.reset(new realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>(root_nh, "/tf", 100));
      set_speed_slider_srv_ = robot_nh.advertiseService("set_speed_slider", &HardwareInterface::setSpeedSlider, this);
      pause_button_srv_ = robot_nh.advertiseService("pause", &HardwareInterface::setPause, this);
      ROS_DEBUG_STREAM("CoppeliaSim hardware-interface has been initialized successfully!");
      
      return true;
  }

  bool HardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                        const std::list<hardware_interface::ControllerInfo>& stop_list)
  {
    bool ret_val = true;
    if (controllers_initialized_ && !start_list.empty())
    {
      for (auto& controller : start_list)
      {
        if (!controller.claimed_resources.empty())
        {
          ROS_ERROR_STREAM("Robot control is currently inactive. Starting controllers that claim resources is currently "
                          "not possible. Not starting controller '"
                          << controller.name << "'");
          ret_val = false;
        }
      }
    }

    controllers_initialized_ = true;
    return ret_val;
  }

  void HardwareInterface::extractToolPose(const ros::Time& timestamp)
  {
    tcp_transform_.header.stamp = timestamp;
    tf2::Quaternion rotation = tf2::Quaternion(tcp_orientation_[0], tcp_orientation_[1], tcp_orientation_[2], tcp_orientation_[3]);
    tcp_transform_.transform.translation.x = tcp_position_[0];
    tcp_transform_.transform.translation.y = tcp_position_[1];
    tcp_transform_.transform.translation.z = tcp_position_[2];
    tcp_transform_.transform.rotation = tf2::toMsg(rotation);
  }

  void HardwareInterface::publishPose()
  {
    if (tcp_pose_pub_)
    {
      if (tcp_pose_pub_->trylock())
      {
        tcp_pose_pub_->msg_.transforms.clear();
        tcp_pose_pub_->msg_.transforms.push_back(tcp_transform_);
        tcp_pose_pub_->unlockAndPublish();
      }
    }
  }

  void HardwareInterface::read(const ros::Time& time, const ros::Duration& period){
      // Read in the joint data
      simxFloat position, velocity, torque;
      for(size_t i = 0; i < joint_names_.size(); i++){
          if(simxGetJointPosition(client_id_, joint_handles_[i], &position, simx_opmode_buffer) == simx_return_ok
              // 2012 = "magic-number" from CoppeliaSim-API-constants representing joint-velocity
              && simxGetObjectFloatParameter(client_id_, joint_handles_[i], 2012, &velocity, simx_opmode_buffer) == simx_return_ok
              && simxGetJointForce(client_id_, joint_handles_[i], &torque, simx_opmode_buffer) == simx_return_ok){
                  joint_positions_[i] = static_cast<double>(position);
                  joint_velocities_[i] = static_cast<double>(velocity);
                  joint_efforts_[i] = static_cast<double>(torque);
                  
              }
          else{
              ROS_WARN_STREAM_THROTTLE(1.0, "Failed to read values of joint " << joint_names_[i] << ". This might happen for a short time during start-up.");
              break;
          }
      
      }
      // Process TCP-pose
      if(simxGetObjectPosition(client_id_, tcp_handle_, base_handle_, &tcp_position_[0], simx_opmode_buffer) == simx_return_ok
          && simxGetObjectQuaternion(client_id_, tcp_handle_, base_handle_, &tcp_orientation_[0], simx_opmode_buffer) == simx_return_ok){
            extractToolPose(time);
            publishPose();
            // Only when TCP-pose is handled successfully, useful force-torque-sensor evaluation is possible
            std::vector<simxFloat> force_torque(6);
            if(simxReadForceSensor(client_id_, ft_sensor_hande_, NULL, &force_torque[0], &force_torque[3], simx_opmode_buffer) == simx_return_ok)
            {
              fts_measurements_[0] = static_cast<double>(force_torque[0]);
              fts_measurements_[1] = static_cast<double>(force_torque[1]);
              fts_measurements_[2] = static_cast<double>(force_torque[2]);
              fts_measurements_[3] = static_cast<double>(force_torque[3]);
              fts_measurements_[4] = static_cast<double>(force_torque[4]);
              fts_measurements_[5] = static_cast<double>(force_torque[5]);
              transformForceTorque();
            }
            else{
              ROS_WARN_STREAM_THROTTLE(1.0, "Can not read force-torque-sensor data (tool0). This might happen for a short time during start-up.");
            } 
          }
      else
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "Could not get TCP-pose and thus no force-torque-measurement. This might happen for a short time during start-up.");
      }

      // Handle speed-scaling and pausing
      if(run_state_ == RunState::PAUSED){
        speed_scaling_combined_ = 0.0;
      }
      else if(run_state_ == RunState::RAMPUP){
        speed_scaling_combined_ += pausing_ramp_up_increment_;
        if(speed_scaling_combined_ >= speed_scaling_) run_state_ == RunState::RUNNING;
      }
      else{
        speed_scaling_combined_ = speed_scaling_;
      }
  }

  void HardwareInterface::transformForceTorque()
  {
    tcp_force_.setValue(fts_measurements_[0], fts_measurements_[1], fts_measurements_[2]);
    tcp_torque_.setValue(fts_measurements_[3], fts_measurements_[4], fts_measurements_[5]);

    tf2::Quaternion rotation_quat;
    tf2::fromMsg(tcp_transform_.transform.rotation, rotation_quat);
    tcp_force_ = tf2::quatRotate(rotation_quat.inverse(), tcp_force_);
    tcp_torque_ = tf2::quatRotate(rotation_quat.inverse(), tcp_torque_);

    fts_measurements_ = { tcp_force_.x(),  tcp_force_.y(),  tcp_force_.z(),
                          tcp_torque_.x(), tcp_torque_.y(), tcp_torque_.z() };
  }

  void HardwareInterface::write(const ros::Time& time, const ros::Duration& period){
      if (position_controller_running_)
      {
        
          for(size_t i = 0; i < joint_names_.size(); i++){
              simxSetJointTargetPosition(client_id_, joint_handles_[i], static_cast<simxFloat>(joint_position_command_[i]), simx_opmode_oneshot);
          }
      }
      else if (velocity_controller_running_)
      {
        for(size_t i = 0; i < joint_names_.size(); i++){
              simxSetJointTargetVelocity(client_id_, joint_handles_[i], static_cast<simxFloat>(joint_velocity_command_[i]), simx_opmode_oneshot);
          }
      }
      else{
          ROS_WARN_STREAM_THROTTLE(1, "Received write-command although no command-controller is active");
      }
  }


  bool HardwareInterface::checkControllerClaims(const std::set<std::string>& claimed_resources)
  {
    for (const std::string& it : joint_names_)
    {
      for (const std::string& jt : claimed_resources)
      {
        if (it == jt)
        {
          return true;
        }
      }
    }
    return false;
  }

  void HardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                  const std::list<hardware_interface::ControllerInfo>& stop_list)
  {
    for (auto& controller_it : stop_list)
    {
      for (auto& resource_it : controller_it.claimed_resources)
      {
        if (checkControllerClaims(resource_it.resources))
        {
          if (resource_it.hardware_interface == "ur_controllers::ScaledPositionJointInterface")
          {
            position_controller_running_ = false;
          }
          if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
          {
            position_controller_running_ = false;
          }
          if (resource_it.hardware_interface == "ur_controllers::ScaledVelocityJointInterface")
          {
            velocity_controller_running_ = false;
          }
          if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
          {
            velocity_controller_running_ = false;
          }
        }
      }
    }
    for (auto& controller_it : start_list)
    {
      for (auto& resource_it : controller_it.claimed_resources)
      {
        if (checkControllerClaims(resource_it.resources))
        {
          if (resource_it.hardware_interface == "ur_controllers::ScaledPositionJointInterface")
          {
            position_controller_running_ = true;
          }
          if (resource_it.hardware_interface == "hardware_interface::PositionJointInterface")
          {
            position_controller_running_ = true;
          }
          if (resource_it.hardware_interface == "ur_controllers::ScaledVelocityJointInterface")
          {
            velocity_controller_running_ = true;
          }
          if (resource_it.hardware_interface == "hardware_interface::VelocityJointInterface")
          {
            velocity_controller_running_ = true;
          }
        }
      }
    }
    if(velocity_controller_running_){
        for(size_t i = 0; i < joint_names_.size(); i++) simxSetObjectIntParameter(client_id_, joint_handles_[i], 2001, 0, simx_opmode_oneshot);
    }
    else if(position_controller_running_){
        for(size_t i = 0; i < joint_names_.size(); i++) simxSetObjectIntParameter(client_id_, joint_handles_[i], 2001, 1, simx_opmode_oneshot);
    }
    else{
        ROS_WARN_STREAM("Controller switch stopped velocity- and position-control!");
    }
  }

  bool HardwareInterface::zeroFTSensor(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
  {
    res.message = "CoppeliaSim has no zero-ing functionality for force-/torque-sensors";
    res.success = false;
    return true;
  }

  bool HardwareInterface::setSpeedSlider(ur_msgs::SetSpeedSliderFractionRequest& req,
                                        ur_msgs::SetSpeedSliderFractionResponse& res)
  {
    if (req.speed_slider_fraction >= 0.01 && req.speed_slider_fraction <= 1.0)
    {
      speed_scaling_ = req.speed_slider_fraction;
      res.success = true;
    }
    else
    {
      res.success = false;
    }
    return true;
  }

  bool HardwareInterface::setPause(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res){
    if(run_state_ == RunState::RAMPUP){
      res.success = false;
      res.message = "Currently ramping up - Try again later";
    }
    else if((run_state_ == RunState::PAUSED && !req.data) || (run_state_ == RunState::RUNNING && req.data)){
      res.success = false;
      res.message = "Requested mode is already set";
    }
    else{
      run_state_ = req.data ? RunState::RAMPUP : RunState::PAUSED;
      res.success = true;
      res.message = "Changed to requested mode";
    }
  }
}


PLUGINLIB_EXPORT_CLASS(coppeliasim_interface::HardwareInterface, hardware_interface::RobotHW)


