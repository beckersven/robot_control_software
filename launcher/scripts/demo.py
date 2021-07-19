#! /usr/bin/python
import rospy
import sys
import moveit_commander
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from std_srvs.srv import SetBoolRequest, SetBool
import trimesh
import numpy as np
import os
import rospkg
import geometry_msgs.msg
from zivid_camera.srv import CaptureRequest, Capture, CaptureAssistantSuggestSettings, CaptureAssistantSuggestSettingsRequest

if __name__ == "__main__":
    
    # Initialize ROS and MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("gripper_test")
    mg = moveit_commander.MoveGroupCommander("manipulator")
    
    # Specific joint states that have been recorded through listening to /joint_states-topic and freedriving the robot to the desired pose
    pre_pick_up_joint_state = [2.8431787490844727, -2.1451484165587367, 2.4604838530169886, -1.84381165126943, -1.5701049009906214, -0.32077151933778936]
    pick_up_joint_state = [2.842914581298828, -1.9490276775755824, 2.6715453306781214, -2.2575136623778285, -1.570690933858053, -0.3210237661944788]
    place_joint_state = [3.4416627883911133, -1.3195999425700684, 2.603414837514059, -3.7257591686644496, -0.21542865434755498, -3.753528420125143]

    # Activate the gripper
    gripper_command_pub = rospy.Publisher("/Robotiq2FGripperRobotOutput", Robotiq2FGripper_robot_output,queue_size=1, latch=True)
    gripper_message = Robotiq2FGripper_robot_output()
    gripper_message.rACT = 0
    gripper_command_pub.publish(gripper_message)
    rospy.sleep(3)
    gripper_message = Robotiq2FGripper_robot_output()
    gripper_message.rACT = 1
    gripper_message.rGTO = 1
    gripper_message.rSP  = 255
    gripper_message.rFR  = 150
    gripper_command_pub.publish(gripper_message)
    rospy.loginfo("Gripper initialized...")
    rospy.sleep(1)

    # Connect to fixture
    rospy.wait_for_service("/close_fixture")
    fixture_service = rospy.ServiceProxy("/close_fixture", SetBool)
    # Open fixture
    fixture_service_request = SetBoolRequest(data=False)
    if not fixture_service.call(fixture_service_request).success:
        print("Fixture opening failed!")
        exit(1)
    print("Fixture opened!")



    # Plan to pre-pick state from current state
    mg.set_start_state_to_current_state()
    mg.set_joint_value_target(pre_pick_up_joint_state)
    result = mg.plan()
    if result[0] == False:
        rospy.logerr("Failed to plan!")
        sys.exit(0)
    # Execute planning result
    print("Move to pre-pick pose")
    if not mg.execute(result[1]):
        print("Trajectory execution failed")
        exit(1)

    # Open gripper
    gripper_message.rPR = 0
    gripper_command_pub.publish(gripper_message)
    rospy.sleep(2)
    print("Opened gripper")

    # Plan to pre-pick state from current state
    mg.set_start_state_to_current_state()
    mg.set_joint_value_target(pick_up_joint_state)
    result = mg.plan()
    if result[0] == False:
        rospy.logerr("Failed to plan!")
        sys.exit(0)
    # Execute planning result
    print("Move to pick pose")
    if not mg.execute(result[1]):
        print("Trajectory execution failed")
        exit(1)
    # Close gripper (pick motor)
    gripper_message.rPR = 255
    gripper_command_pub.publish(gripper_message)
    print("Closed gripper")

    # Plan to post-pick state from current state (=retreat)
    mg.set_start_state_to_current_state()
    mg.set_joint_value_target(pre_pick_up_joint_state)
    result = mg.plan()
    if result[0] == False:
        rospy.logerr("Failed to plan!")
        sys.exit(0)
    # Execute planning result
    print("Move to post-pick pose")   
    if not mg.execute(result[1]):
        print("Trajectory execution failed")
        exit(1)

    # Plan to place state from current state
    mg.set_start_state_to_current_state()
    mg.set_joint_value_target(place_joint_state)
    result = mg.plan()
    if result[0] == False:
        rospy.logerr("Failed to plan!")
        sys.exit(0)
    # Execute planning result
    print("Move to place pose")   
    if not mg.execute(result[1]):
        print("Trajectory execution failed")
        exit(1)

    # Open fixture
    fixture_service_request = SetBoolRequest(data=True)
    if not fixture_service.call(fixture_service_request).success:
        print("Fixture closing failed!")
        exit(1)
    print("Fixture closed!")

    gripper_message.rPR = int(0)
    gripper_command_pub.publish(gripper_message)
    print("Open gripper")

    # Connect to and call capture assistant service
    print("Obtaining parameters for capture")
    rospy.wait_for_service("/zivid_camera/capture_assistant/suggest_settings", rospy.Duration(1))
    suggest_srv = rospy.ServiceProxy("/zivid_camera/capture_assistant/suggest_settings", service_class=CaptureAssistantSuggestSettings)
    suggest_req = CaptureAssistantSuggestSettingsRequest()
    suggest_req.ambient_light_frequency = suggest_req.AMBIENT_LIGHT_FREQUENCY_50HZ
    suggest_req.max_capture_time = rospy.Duration(3)
    suggest_srv.call(suggest_req)

    # Capture with suggested settings
    print("Perfoming capture with obtained parameters")
    rospy.wait_for_service("/zivid_camera/capture_assistant/suggest_settings", rospy.Duration(1))
    capture_srv = rospy.ServiceProxy("/zivid_camera/capture", service_class=Capture)
    capture_req = CaptureRequest()
    capture_srv.call(CaptureRequest())


    print("Finished!")
