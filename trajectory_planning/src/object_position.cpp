#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

tf::Vector3 Vob (0.5, -0.7, 1.1);
geometry_msgs::Vector3 Vob_msg;

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "object_position");

  ros::NodeHandle n;
  bool latch = true;
  ros::Publisher ob_pos_pub = n.advertise<geometry_msgs::Vector3>("object_position", 1000, latch);

  ros::Rate loop_rate(10);

  tf::vector3TFToMsg(Vob, Vob_msg);
  ob_pos_pub.publish(Vob_msg);
  while (ros::ok())
  {
    
   
//  ros::spinOnce();

//    loop_rate.sleep();
 // ++count;
  }


  return 0;
}
