// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------

#include <controller_manager/controller_manager.h>
#include <csignal>
#include <../include/hardware_interface.h>
#include <ros/ros.h>
#include <iostream>

std::unique_ptr<coppeliasim_interface::HardwareInterface> hw_interface;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "hardware_interface");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh1, nh2, nh3("~");
  spinner.start();
  double sim_dt;
  if(!nh1.getParam("/coppelia_config/sim_dt", sim_dt)){
    ROS_ERROR_STREAM("Can not resolve " << nh1.resolveName("/coppelia_config/sim_dt"));
    exit(1);
  }
  ros::Duration dt(sim_dt);
  hw_interface.reset(new coppeliasim_interface::HardwareInterface);
  if(!hw_interface->init(nh2, nh3)){
    ROS_ERROR_STREAM("Could not initialize hardware-interface!");
    exit(1);
  }
  controller_manager::ControllerManager cm(hw_interface.get(), nh1);

  ros::ServiceServer srv = nh1.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("/update_hw_interface", 
    [&](std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)mutable{
      ros::Time current_time(ros::Time::now());
      ROS_INFO_STREAM("dt " << dt);
      hw_interface->read(current_time, dt);
      cm.update(current_time, dt);
      hw_interface->write(current_time, dt);
      res.message = "Updated controllers";
      res.success = true;
      return true;
  });
    
  ROS_DEBUG_STREAM("Hardware interface node ready");
  ros::Rate rate(1);
  while(ros::ok()) rate.sleep();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  spinner.stop();
  return 0;
}
