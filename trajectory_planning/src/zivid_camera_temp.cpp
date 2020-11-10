#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

bool cam_state;

void capture_image ()
{
  // ... capture image code
  // ...
  //ros::Duration(2).sleep();  
  cam_state = true;
}

void Callback_pose_state (std_msgs::String msg_state)
{
 // std::cout << msg_state << std::endl;
  if (msg_state.data ==  "camera is ready to capture image"){
    capture_image(); 
    std::cout << "True" << std::endl;   

    }  
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "zivid_camera_temp");

  ros::NodeHandle n;
 // ros::Duration(2).sleep();
  ros::Subscriber sub_state = n.subscribe("trajectory_position_state", 1000, Callback_pose_state);
  ros::Publisher cam_pub = n.advertise<std_msgs::String>("camera_state", 1000);
  std_msgs::String msg;
  msg.data = "image captured";
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::cout << cam_state << std::endl;
    if (cam_state == true)
    {

      cam_pub.publish(msg); 
      cam_state = false;
  //    ros::Duration(1).sleep();
    }
    ros::spinOnce();
  }

  

  return 0;
}
