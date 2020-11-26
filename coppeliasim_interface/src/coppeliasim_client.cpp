#include "../include/coppeliasim_client.h"


CoppeliaSimInterface::CoppeliaSimInterface(){
    joint_names_.resize(6, "");
    
    
    
    this->client_id_ = -1;
    coppeliasim_simulating_ = false;
    speed_scaling_combined_ = 1;
}
CoppeliaSimInterface::~CoppeliaSimInterface(){
    if(coppeliasim_simulating_){
        simxStopSimulation(client_id_, simx_opmode_oneshot);
        simxSynchronousTrigger(client_id_);
        ROS_DEBUG_STREAM("Stopped CoppeliaSim simualtion");
    }
    if(client_id_ != -1){
        simxFinish(client_id_);
        ROS_DEBUG_STREAM("Disconnected from CoppeliaSim");
    }
}
bool CoppeliaSimInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_nh){
    this->root_nh = root_nh;
    this->robot_nh = robot_nh;
    std::string coppeliasim_ip_;
    simxInt coppeliasim_port_;
    if(! (this->robot_nh.getParam("coppeliasim_ip", coppeliasim_ip_) && this->robot_nh.getParam("coppeliasim_port", coppeliasim_port_))){
        ROS_ERROR_STREAM("Cannot resolve " << this->robot_nh.resolveName("coppeliasim_ip") << " or " << this->robot_nh.resolveName("coppeliasim_port") << "! Simulation connection failed");
        return false;
    }
    client_id_ = simxStart(coppeliasim_ip_.c_str(), coppeliasim_port_, true, true, 1000, 5);
    if(client_id_ == -1){
        ROS_ERROR_STREAM("Connection to CoppeliaSim failed! Used IP " << coppeliasim_ip_ << " and port " << coppeliasim_port_);
        return false;
    }

    if(simxStartSimulation(client_id_, simx_opmode_blocking) == simx_return_ok && simxSynchronous(client_id_, true) == simx_return_ok){
        ROS_DEBUG_STREAM("Started CoppeliaSim simulation");
        coppeliasim_simulating_ = true;
    }
    else{
        ROS_ERROR_STREAM("CoppeliaSim simulation could not be started");
        coppeliasim_simulating_ = false;
        return false;
    }
    



    if(!this->root_nh.getParam("hardware_interface/joints", joint_names_)){
        ROS_ERROR_STREAM("Could not find joint-names in " << this->root_nh.resolveName("hardware_interface/joints"));
        return false;
    }
    simxFloat* dummy;
    // Create ros_control interfaces
    for (std::size_t i = 0; i < n_joints; ++i)
    {
        ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);
        // Create joint state interface for all joints
        ROS_ERROR_STREAM("Fully connected to CoppeliaSim!A");
        js_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_positions_[i],
                                                                        &joint_velocities_[i], &joint_efforts_[i]));

        if(! simxGetObjectHandle(client_id_, joint_names_[i].c_str(), &joint_handles_[i], simx_opmode_blocking) == simx_return_ok){
            ROS_ERROR_STREAM("Can not resolve " << joint_names_[i] << " in CoppeliaSim");
            return false;
        }
        
        simxGetJointPosition(client_id_, joint_handles_[i], dummy, simx_opmode_streaming);
        simxGetObjectFloatParameter(client_id_, joint_handles_[i], 2012, dummy, simx_opmode_streaming);
        simxGetJointForce(client_id_, joint_handles_[i], dummy, simx_opmode_streaming); 
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

    // robot_status_interface_.registerHandle(industrial_robot_status_interface::IndustrialRobotStatusHandle(
    //     "industrial_robot_status_handle", robot_status_resource_));
    // Register interfaces
    registerInterface(&js_interface_);
    registerInterface(&spj_interface_);
    registerInterface(&pj_interface_);
    registerInterface(&vj_interface_);
    registerInterface(&svj_interface_);
    registerInterface(&speedsc_interface_);
    ROS_DEBUG_STREAM("CoppeliaSim hardware-interface has been initialized successfully!");
    return true;
}

bool CoppeliaSimInterface::advanceCoppeliaSim(){
    return simxSynchronousTrigger(client_id_) == simx_return_ok;
}

bool CoppeliaSimInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                      const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  bool ret_val = true;/*
  if (controllers_initialized_ && !isRobotProgramRunning() && !start_list.empty())
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
  }*/

  controllers_initialized_ = true;
  return ret_val;
}

void CoppeliaSimInterface::read(const ros::Time& time, const ros::Duration& period){
    simxFloat position, velocity, torque;
    for(size_t i = 0; i < n_joints; i++){
        if(simxGetJointPosition(client_id_, joint_handles_[i], &position, simx_opmode_buffer) == simx_return_ok
            && simxGetObjectFloatParameter(client_id_, joint_handles_[i], 2012, &velocity, simx_opmode_buffer) == simx_return_ok
            && simxGetJointForce(client_id_, joint_handles_[i], &torque, simx_opmode_buffer) == simx_return_ok){
                joint_positions_[i] = static_cast<double>(position);
                joint_velocities_[i] = static_cast<double>(velocity);
                joint_efforts_[i] = static_cast<double>(torque);
            }
        else{
            ROS_WARN_STREAM_THROTTLE(1.0, "Tried to read values of joint " << joint_names_[i] << " although buffer is empty. This message should disappear in a short time...");
            break;
        }
    
    }
}

void CoppeliaSimInterface::write(const ros::Time& time, const ros::Duration& period){
    if (position_controller_running_)
    {
        for(size_t i = 0; i < n_joints; i++){
            simxSetJointTargetPosition(client_id_, joint_handles_[i], static_cast<simxFloat>(joint_position_command_[i]), simx_opmode_oneshot);
        }
    }
    else if (velocity_controller_running_)
    {// do some initialization here
      for(size_t i = 0; i < n_joints; i++){
            simxSetJointTargetVelocity(client_id_, joint_handles_[i], static_cast<simxFloat>(joint_velocity_command_[i]), simx_opmode_oneshot);
        }
    }
    else{
        ROS_WARN_STREAM_THROTTLE(1, "Received write-command although no command-controller is active");
    }
}


bool CoppeliaSimInterface::checkControllerClaims(const std::set<std::string>& claimed_resources)
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

void CoppeliaSimInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
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
      for(size_t i = 0; i < n_joints; i++) simxSetObjectFloatParameter(client_id_, joint_handles_[i], 2001, 1, simx_opmode_oneshot);
  }
  else if(position_controller_running_){
      for(size_t i = 0; i < n_joints; i++) simxSetObjectFloatParameter(client_id_, joint_handles_[i], 2001, 0, simx_opmode_oneshot);
  }
  else{
      ROS_WARN_STREAM("Controller switch stopped velocity- and position-control!");
  }
}


