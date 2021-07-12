#! /usr/bin/python
import rospy
from moveit_commander import planning_scene_interface
from geometry_msgs.msg import PoseStamped

if __name__=="__main__":
    rospy.init_node("obstacle_publisher")
    psi = planning_scene_interface.PlanningSceneInterface(synchronous=True)
    
    
    box_pose = PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 2
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 1.5
    try:
        psi.add_box("wall_1", box_pose, size=(0.1, 1.9, 3))
    except Exception as e:
        rospy.logerr("Obstacle could not be added properly - collision model might be corrupted! {}".format(e))
        exit(1)
    box_pose.pose.position.x = 1
    box_pose.pose.position.y = 1
    box_pose.pose.position.z = 1.5
    try:
        psi.add_box("wall_2", box_pose, size=(2, 0.1, 3))
    except Exception as e:
        rospy.logerr("Obstacle could not be added properly - collision model might be corrupted! {}".format(e))
        exit(1)
    # Add additional obstacles here

    