#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ObjectColor.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <iostream>
#include <typeinfo>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ros/ros.h"
#include "std_msgs/String.h"

tf::Vector3 Vee_tf;
tf::Quaternion q_tf;
bool pose_state;
bool ori_state;

void Callback_pos(geometry_msgs::Vector3 Vee_msg)
{
//  ROS_INFO("I heard: [%s]", msg);
 // std::cout << msg << std::endl;
  tf::vector3MsgToTF(Vee_msg, Vee_tf);
  pose_state = true;
 
}
void Callback_ori(geometry_msgs::Quaternion q_msg)
{
  
  std::cout << q_msg << std::endl;
  tf::quaternionMsgToTF(q_msg, q_tf);
  ori_state = true;
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  
  static const std::string PLANNING_GROUP = "manipulator";


  // The :move_group_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ros::Subscriber ee_pos_sub = node_handle.subscribe("end_effector_position", 1000, Callback_pos);
  ros::Subscriber ee_ori_sub = node_handle.subscribe("end_effector_orientation", 1000, Callback_ori);
  ros::Publisher fin_pos_pub = node_handle.advertise<std_msgs::String>("trajectory_position_state", 1000);

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  int a = 5;
  
  std::string s = typeid(current_pose).name();
  std::cout << "Current position "<< current_pose;
  


  Eigen::Matrix3f mat5 = Eigen::Quaternionf(current_pose.pose.orientation.w,   current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z).toRotationMatrix();
  std::cout << mat5<<std::endl;
  
 
  move_group.setNumPlanningAttempts(10);
  move_group.setPlanningTime(5);


  ros::Duration(1).sleep();


  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object_2;
  collision_object_2.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_2.id = "box2";  // 

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive_2;
  primitive_2.type = primitive_2.BOX;
  primitive_2.dimensions.resize(3);
  primitive_2.dimensions[0] = 2;
  primitive_2.dimensions[1] = 0.1;
  primitive_2.dimensions[2] = 3;

   // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_2;
  box_pose_2.orientation.w = 1.0;
  box_pose_2.position.x = 1;
  box_pose_2.position.y = 1;
  box_pose_2.position.z = 1.5;

  collision_object_2.primitives.push_back(primitive_2);
  collision_object_2.primitive_poses.push_back(box_pose_2);
  collision_object_2.operation = collision_object_2.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object_2);


  // Now, let's add the collision object into the world
  planning_scene_interface.addCollisionObjects(collision_objects);


  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object_3;
  collision_object_3.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_3.id = "box3";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive_3;
  primitive_3.type = primitive_3.BOX;
  primitive_3.dimensions.resize(3);
  primitive_3.dimensions[0] = 0.1;
  primitive_3.dimensions[1] = 1.9;
  primitive_3.dimensions[2] = 3;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_3;
  box_pose_3.orientation.w = 1.0;
  box_pose_3.position.x = 2;
  box_pose_3.position.y = 0;
  box_pose_3.position.z = 1.5;

  collision_object_3.primitives.push_back(primitive_3);
  collision_object_3.primitive_poses.push_back(box_pose_3);
  collision_object_3.operation = collision_object_3.ADD;


  collision_objects.push_back(collision_object_3);
  planning_scene_interface.addCollisionObjects(collision_objects);


  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object_4;
  collision_object_4.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object_4.id = "box4";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive_4;
  primitive_4.type = primitive_4.BOX;
  primitive_4.dimensions.resize(3);
  primitive_4.dimensions[0] = 0.1;
  primitive_4.dimensions[1] = 0.2;
  primitive_4.dimensions[2] = 0.25;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose_4;
  box_pose_4.orientation.w = 1.0;
  box_pose_4.position.x = 0.5;
  box_pose_4.position.y = -0.5;
  box_pose_4.position.z = 1.225;
 

  collision_object_4.primitives.push_back(primitive_4);
  collision_object_4.primitive_poses.push_back(box_pose_4);
  
  collision_object_4.operation = collision_object_4.ADD;
  collision_objects.push_back(collision_object_4);
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  
    // velocity
    move_group.setGoalPositionTolerance(0.05);
    move_group.setGoalOrientationTolerance(0.05);
    move_group.setMaxVelocityScalingFactor(0.8);   // %
    move_group.setEndEffectorLink("scanner_focus");
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.38268;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.92388;
  target_pose1.position.x = 0.05;
  target_pose1.position.y = -0.5;
  target_pose1.position.z = 1.6;
  move_group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  ROS_ERROR_STREAM("PLANNING");
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_ERROR_STREAM("MOVING");
  move_group.move();
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose1);
  for(int i = 1; i <= 18; i++){
    target_pose1.orientation.y = (i > 6) ? 0.86603 : 0.5;
    target_pose1.orientation.w = ( i < 6) ? 0.5 : 0.86603;
    target_pose1.position.x += 0.05;
    waypoints.push_back(target_pose1);
    ROS_ERROR("ADD");
  }
 moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  move_group.execute(trajectory);
  ros::shutdown();
  return 0;
}
