// Vee_init end efector initial position

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>

tf::Vector3 Vob;
tf::Vector3 Vee;
tf::Vector3 Vee_init;
geometry_msgs::Vector3 Vee_msg;
bool pose_state;

tf::Quaternion ee_orientation (tf::Vector3 Vob, tf::Vector3 Vee, float pi_rot)
{
  std::cout << "Vob  "<<Vob.x() << Vob.y() << Vob.z() <<std::endl;
  std::cout << "Vee  "<<Vee.x() << Vee.y() << Vee.z() <<std::endl;
  tf::Vector3 Vzn (Vob-Vee);
  tf::Vector3 Vzb (0,0,pi_rot) ; // ex: 1 or -1
  tf::Vector3 Vyn (Vzn.cross(Vzb));
  tf::Vector3 Vxn (Vzn.cross(Vyn));
  std::cout << "Vyn  "<<Vyn.x()<<' '<< Vyn.y()<<' '<< Vyn.z()<<std::endl;
  std::cout << "Vxn  "<<Vxn.x()<<' '<< Vxn.y()<<' '<< Vxn.z()<<std::endl;
  std::cout << "Vzn  "<<Vzn.x()<<' '<< Vzn.y()<<' '<< Vzn.z()<<std::endl;
  
  float Vzn_len = sqrt(Vzn.getX()*Vzn.getX() + Vzn.getY()*Vzn.getY() + Vzn.getZ()*Vzn.getZ());
  Vzn /= Vzn_len;
  double Vyn_len = sqrt(Vyn.getX()*Vyn.getX() + Vyn.getY()*Vyn.getY() + Vyn.getZ()*Vyn.getZ());
  Vyn /= Vyn_len;
  float Vxn_len = sqrt(Vxn.getX()*Vxn.getX() + Vxn.getY()*Vxn.getY() + Vxn.getZ()*Vxn.getZ());
  Vxn /= Vxn_len;


 tf::Matrix3x3 mat_new(Vyn.getX(), Vxn.getX(), Vzn.getX(), Vyn.getY(), Vxn.getY(), Vzn.getY(), Vyn.getZ(), Vxn.getZ(), Vzn.getZ()) ;

  std::cout << "determinant  "<<mat_new.determinant() <<std::endl;
 
  
  geometry_msgs::Quaternion q_msg;
  
  tf::Quaternion q;
  mat_new.getRotation(q);
  tf::quaternionTFToMsg(q, q_msg);
  std::cout << "mat_new 0  "<<mat_new.getRow(0).x() <<' ' << mat_new.getRow(0).y()<<' ' << mat_new.getRow(0).z()<<std::endl;
  std::cout << "mat_new 1"<<mat_new.getRow(1).x()<<' ' << mat_new.getRow(1).y()<<' ' << mat_new.getRow(1).z()<<std::endl;
  std::cout << "mat new 2"<<mat_new.getRow(2).x()<<' ' << mat_new.getRow(2).y()<<' ' << mat_new.getRow(2).z()<<std::endl;
  q.normalize();
  ROS_INFO_STREAM(q[0]);  // Print the quaternion components (0,0,0,1)
  ROS_INFO_STREAM(q[1]);
  ROS_INFO_STREAM(q[2]);
  ROS_INFO_STREAM(q[3]);

  return q;
  
}

void Callback_ob_pose(geometry_msgs::Vector3 msg_pose)
{
//  ROS_INFO("I heard: [%s]", msg);
  std::cout << msg_pose << std::endl;
  tf::vector3MsgToTF(msg_pose, Vob);
 // Vee.setValue (0.5, -0.71, 1.45);
 // ee_orientation(Vob, Vee, 1);
 
}

void Callback_cam_state (std_msgs::String msg_state)
{
  std::cout << msg_state << std::endl;
  if (msg_state.data ==  "image captured"){
    pose_state = true;
    std::cout << "True" << std::endl;
    
  }
  
}

geometry_msgs::Vector3 ee_position_msg (tf::Vector3 Vee)
{
  
  // initial position
  tf::vector3TFToMsg(Vee, Vee_msg);
  return Vee_msg;
  
}

geometry_msgs::Quaternion ee_orientation_msg(tf::Quaternion q_tf)
{
  geometry_msgs::Quaternion q_msg;
  tf::quaternionTFToMsg(q_tf, q_msg);
  return q_msg;
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "trajectory_path");

  
  ros::NodeHandle n;
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
  bool latch = true;

  ros::Subscriber sub_ob_pose = n.subscribe("object_position", 100, Callback_ob_pose);
  ros::Subscriber sub_cam = n.subscribe("camera_state", 1000, Callback_cam_state);
  ros::Publisher ee_pos_pub = n.advertise<geometry_msgs::Vector3>("end_effector_position", 1000, latch);
  ros::Publisher ee_ori_pub = n.advertise<geometry_msgs::Quaternion>("end_effector_orientation", 1000, latch);
  ros::Duration(1).sleep();
  ros::spinOnce();
  Vee_init.setValue (0.5, -0.71, 1.55);
 


 // std::cout << Vob[0] << std::endl;
 
  ee_pos_pub.publish(ee_position_msg (Vee_init));

  ee_ori_pub.publish(ee_orientation_msg (ee_orientation(Vob,Vee_init,1))); 
  ros::Duration(1).sleep();
  Vee[2] = Vee_init[2];
  double angle_resolution = 60;
  double radius=0.3;
  double d_angle = angle_resolution*3.14/180;
  double angle= 0;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
  //  std::cout << pose_state << std::endl;
    ros::spinOnce();
    for (static int i= 0; i< (360/angle_resolution); i++)
    {
      if (pose_state == true)
      {
    //discretize the trajectory
                
        angle+= d_angle;
             
        Vee[0] = Vee_init[0] - radius*cos(angle); 
        Vee[1] = Vee_init[1] - radius*sin(angle);
       // ros::Duration(1).sleep();
        std::cout << angle << std::endl;
        std::cout << "position is     " << Vee[0] << "   "<<Vee[1] << std::endl;
        ee_pos_pub.publish(ee_position_msg (Vee));
        ee_ori_pub.publish(ee_orientation_msg (ee_orientation(Vob,Vee,1))); 
        pose_state = false;
      //  ros::Duration(1).sleep();
      std::cout << i << std::endl; 
      }
      else {
         break;
      }
//      ros::spinOnce();

      
    }
  loop_rate.sleep();
    
  }


  
  std::cout << "finish" << std::endl;
  

  return 0;
}
