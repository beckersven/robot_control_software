#! /usr/bin/python

import trimesh
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import signal
import time
from geometry_msgs.msg import Quaternion
import trajectory_msgs.msg
import time
import sensor_msgs.msg
from std_srvs.srv import SetBoolRequest, SetBool
import pulp
import itertools
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import array
from multiprocessing import Process, Array
from agiprobot_measurement.viewpoint import ViewPoint
from agiprobot_measurement.sensor_model import SensorModel
import yaml
from rospy_message_converter import message_converter
import click
import os
import rospkg

OFFSET = np.array([450, -550, 1200])

class TrajectoryManager:
    """
    Completely integrated system for high-coverage and uncertainty-minimal scanning of given objects with an optical scanner. An instance can perform
    the entire planning pipeline - from the CAD-file and pose to a list of consecutively executable trajectories.
    """
    def __init__(self):
        """
        Initialize planner by connecting to ROS and MoveIt, configuring the latter and creating a sensor_model.
        """
        # Initialization
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("trajectory_manager")
        
        # Configuring MoveIt
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        # Enables multi-query-planning (see corresponding literature)
        self.group.set_planner_id("PersistentPRMstar") 
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.group.set_max_velocity_scaling_factor(1)
        
        

        # Create a sensor-model instance
        self.sensor_model = SensorModel(rospy.get_param("/sensor_model_parameters"))
        
        # Mesh containing the object-to-measure geometrically transformed so that its pose in trimesh-coordinates are the same as in real life (but mm instead of m)
        self.target_mesh = trimesh.Trimesh()
        # Corresponding homogeneous 4x4-matrix
        self.target_mesh_transform = np.eye(4)
        self.max_edge = 1e6



    def load_execution_plan(self, file_path, adapt_to_current_start_pose = True):
        """ 
        Extracts a list of moveit_msgs/RobotTrajectory specified in the yaml-file at the provided path.

        The trajectories can be executed consecutively, however if the execution should start from the current state (which is most
        likely the case), the parameter 'adapt_to_current_start_pose' should be set true to add movement from the current point (which
        in general deviates from the 'current point' during planning) to the trajectory start. The yaml-file must correspond to a list with the first
        entry being a dictionary for metadata and the following entries being dictionaries of 'expanded' RobotTrajectories with following structure:
        Trajecories connecting measurements (C) and measurement trajectories (M) themselves in an alternating fashion (C-M-C-M-C-...).
            
        :param file_path: Where the yaml-file is located
        :type file_path: str
        :param adapt_to_current_start_pose: If true (default), the plan will be adapted so that it can start from the current state
        :type adapt_to_current_start_pose: bool
        :return: List of moveit_msgs/RobotTrajectory-ies that can be consecutively executed
        :rtype: list
        """
        execution_plan = []
        with open(file_path, 'r') as plan_file:
            plan_list = yaml.safe_load(plan_file)
            assert isinstance(plan_list, list)
            # First entry contains dict of meta-data (like date/time, target-mesh-pose, ...)
            assert isinstance(plan_list[0], dict)
            print("Metadata of loaded path:")
            for key, value in plan_list[0].items():
                print("{}:\t{}".format(key, value))
            for path_segment in plan_list[1:]:
                assert isinstance(path_segment, dict)
                execution_plan.append(message_converter.convert_dictionary_to_ros_message("moveit_msgs/RobotTrajectory", path_segment))
        if adapt_to_current_start_pose:
            # Replace the first "steering to measurement trajectory" (= first segment) as
            # it was calculated from the current pose during planning to the first measurement-trajectory (i.e. nothing critical happens here)
            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(execution_plan[0].joint_trajectory.points[-1].positions)
            plan_result = self.group.plan()
            if plan_result[0]:
                execution_plan[0] = plan_result[1]
            else:
                raise Exception("Could not connect execution plan to current start pose!")
        return execution_plan

    def store_execution_plan(self, file_path, execution_plan, metadata={}):
        """
        Stores the provided execution_plan segments (= moveit_msgs/RobotTrajectory) in a yaml-file at the provided file_path formatted so that it can be read in by the class's 'load_execution_plan'-method.

        :param file_path: Path specifying where to safe the generated file at
        :type file_path: str
        :param execution_plan: List of path-segments that can be executed consecutively (i.e. joint-values at the last point of any entry = joint-values at the beginning of the next entry).
        :type execution_plan: list of moveit_msgs/RobotTrajectory
        :param metadata: Dictionary of information about this plan in key-value-fashion (e.g. {'sampling_density': 0.01, 'timestamp': '12:00, 01.01.2021', 'planning_duration_in_s':42}")#
        :type metadata: dict
        :return: True (always)
        :rtype: bool
        """
        assert isinstance(metadata, dict)
        assert isinstance(execution_plan, list)
        plan_to_store = [metadata]
        for path_segment in execution_plan:
            assert isinstance(path_segment, moveit_msgs.msg.RobotTrajectory)
            plan_to_store.append(message_converter.convert_ros_message_to_dictionary(path_segment))
        with open(file_path, 'w') as plan_file:
            yaml.safe_dump(plan_to_store, plan_file, default_flow_style=False)
        return True
    
    def convert_viewpointlist_to_execution_plan(self, viewpointlist):
        """
        Convert a list of ViewPoint-objects into a list of moveit_msgs/RobotTrajectory-entries so that the viewpoints can be executed consecutively.
        
        :param viewpointlist: List of ViewPoint-objects with set trajectories for steering-to-viewpoint and measurement
        :type vievlist: list
        :returns: List of moveit_msgs/RobotTrajectory with 2 entries per provided ViewPoint-object (first = steering-to-viewpoint-trajectory, second = measurement-trajectory)
        :rtype: list
        """
        assert isinstance(viewpointlist, list)
        execution_plan = []
        for viewpoint in viewpointlist:
            assert isinstance(viewpoint, ViewPoint)
            execution_plan.append(viewpointpoint.get_trajectory_to_viewpoint())
            execution_plan.append(viewpoint.get_trajectory_for_measurement())
        return execution_plan



    
    def load_target_mesh(self, file_name, transform = np.eye(4), add_wbk_mirrors=True, remove_downside=True, remove_threshold_angle_deg=20):
        """
        Loads the mesh-to-measure from file into the instance's target_mesh-member (to generate and evaluate viewpoint-trajectories) and into
        MoveIt (for collision-prevention). To move the mesh into a feasible measurement-pose, a transform may be applied to set the mesh's
        reference frame specified in CAD with respect to the 'world' frame

        :param file_name: Where the target_mesh's CAD file is located (should be given starting at root-level: '/X/Y/...')
        :type file_name: str
        :param transform: Homogeneous 4x4-matrix to move the mesh into the desired pose with translation in mm, defaults to identity-matrix
        :type transform: numpy.array, optional
        :param add_wbk_mirrors: Adds hard-coded collision objects to MoveIt as the robot station at the institute is placed besides 2 fragile mirrors, defaults to True
        :type add_wbk_mirrors: bool, optional
        :return: Whether all operations (esp. loading from file into trimesh and MoveIt) were successful
        :rtype: bool
        """
        # Load mesh via trimesh to class member for further processing
        try:
            self.target_mesh = trimesh.load_mesh(file_name)
        except:
            rospy.logerr("Trimesh could not load mesh {}".format(file_name))
            return False
        
        self.target_mesh_transform = transform
        
        # Move the mesh's origin (usually identically with the origin of the provided CAD-file)
        # to where it is mounted with respect to the "world"-frame of MoveIt
        self.target_mesh.apply_transform(self.target_mesh_transform)

        # If specified, remove the downside faces of the CAD-Model (this is where the fixture is located)
        if remove_downside:
            comp = np.cos(np.deg2rad(remove_threshold_angle_deg))
            mask = np.array([-self.target_mesh.face_normals[i][2] < comp for i in range(len(self.target_mesh.faces))])
            self.target_mesh.update_faces(mask)        

        # Add mesh to MoveIt so that it will be considered as a collision object during planning
        # (here, the given transform also needs to be applied via a ROS-pose-message)
        target_mesh_pose_stamped = geometry_msgs.msg.PoseStamped()
        q = trimesh.transformations.quaternion_from_matrix(transform)
        target_mesh_pose_stamped.pose.orientation.w = q[0]
        target_mesh_pose_stamped.pose.orientation.x = q[1]
        target_mesh_pose_stamped.pose.orientation.y = q[2]
        target_mesh_pose_stamped.pose.orientation.z = q[3]
        target_mesh_pose_stamped.pose.position = geometry_msgs.msg.Point(*(trimesh.transformations.translation_from_matrix(self.target_mesh_transform) / 1000))
        target_mesh_pose_stamped.header.frame_id = "world"
        
        try:
            # Reduce size as meshes used here a specified in millimeters whereas MoveIt interprets them as meters
            self.scene.add_mesh("target", target_mesh_pose_stamped, file_name, size=(1e-3, 1e-3, 1e-3))
            pass
        except:
            # add_mesh has troubles with "non-root" filenames (e.g. ~/cadfiles/target.stl)
            rospy.logerr("Moveit could not load mesh {}.\nHave you tried specifying it with a file path starting at root /...?".format(file_name))
            return False

        if add_wbk_mirrors:
            # Add collision objects representing the mirrors beside the robot at the institute to MoveIt
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = "world"
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = 2
            box_pose.pose.position.y = 0
            box_pose.pose.position.z = 1.5
            self.scene.add_box("mirror1", box_pose, (0.1, 1.9, 3))
            box_pose.pose.position.x = 1
            box_pose.pose.position.y = 1
            box_pose.pose.position.z = 1.5
            self.scene.add_box("mirror2", box_pose, size=(2, 0.1, 3))
        return True

    def generate_samples_and_viewpoints(self, sampling_density, uncertainty_threshold, orientations_around_boresight, viewpoint_tilt_mode="full", plan_path_to_check_reachability = False, minimum_trajectory_length = 25, trajectory_sample_step = 2):
        """
        Samples the mesh's surface into discrete points, generates viewpoints for each sample and processes these viewpoints metrologically and mechanically.
        Only viewpoints that do meet the contraints specified in the method's parameters and in MoveIt (collision, reachability, ...) will be returned. This method allows to generate
        multiple ViewPoint-objects for a single surface-point by varying the anchor-pose (the boresight is always focused on the corresponding surface_point).

        :param sampling_density: Density of sampling to generate the points for measurement-calculations and viewpoint-generation in points per mm^2
        :type sampling_density: float
        :param orientations_around_boresight: Number of orientations around the sensor's boresight to be considered per sampled_surface_point
        :type orientations_around_boresight: int
        :param viewpoint_tilt_mode: For each orientation, deviate the psi- and theta-values of the viewpoint-anchor-pose slightly from the optimum according to (one deviation-step per angle and orientation available right now):\n
            - "none": Do not perform any tilting.\n
            - "limited": When the optimal angle-configuration did not work, try deviations and stop after finding the first valid solution.\n
            - "full": Calculation for every possible angle-calculation. Every boresight orientation has 9 sub-viewpoints.
            , defaults to "full"
        :type viewpoint_tilt_mode: str, optional
        :param plan_path_to_check_reachability: Do not use one inverse-kinematics-request to check viewpoint-reachability but try to create complete plan from the current-state to the viewpoint's anchor-pose, defaults to False
        :type plan_path_to_check_reachability: bool, optional
        :param minimum_trajectory_length: Minimum length a generated trajectory has to have so that it is accepted in mm, defaults to 50
        :type minimum_trajectory_length: float, optional
        :param trajectory_sample_step: Euclidian distance between 2 trajectory-points in cartesian sapce in mm, defaults to 2
        :type trajectory_sample_step: float, optional
        :return: 2 lists with the first containing all sampled_surface_points and the second containing all viewpoints that were processed successfully
        :rtype: list[numpy.array], list[ViewPoint]
        """
        # Sample points on the target mesh's surface based on the given density through rejection sampling
        sampled_surface_points, sampled_face_indices = trimesh.sample.sample_surface_even(self.target_mesh, int(self.target_mesh.area * sampling_density))
        print("Sampled target mesh's surface into {} surface points".format(len(sampled_surface_points)))
        
       
        orientations_around_boresight
        print(orientations_around_boresight)
        

        # Use the sampling results to set the processing context of the sensor model
        self.sensor_model.set_processing_context(self.target_mesh, sampled_surface_points, sampled_face_indices)
        # Very conservative limit of the maximum length in and againts trajectory_direction of 
        # the each viewpoint's measurement-trajectory so that the target-object is covered for sure
        self.max_edge = max(self.target_mesh.bounding_box_oriented.edges_unique_length)

        # Publish the sampled_surface_points via ROS so that they can be visualized in RVIZ for user feedback
        pub = rospy.Publisher("sampled_surface_points", sensor_msgs.msg.PointCloud, latch=True, queue_size=1)
        surface_points_message = sensor_msgs.msg.PointCloud()
        surface_points_message.header.frame_id = "world"
        for surface_point in sampled_surface_points:
            surface_points_message.points.append(geometry_msgs.msg.Point(*(surface_point / 1000)))
        pub.publish(surface_points_message)

        
        
        # How much a trajctory has to be longer than the maximum defliction to reliably scan the object (as sampled_surface_points are most likely not on edges).
        # Estimated as circle radius according to density + safety offset.
        minimum_required_overmeasure = np.sqrt(1 / (np.pi * sampling_density)) + 5
        
        
        if plan_path_to_check_reachability:
            # Reduce planning time so that reachablity checks do not take too much time
            self.group.set_planning_time(0.05)
            ik_service = None 
        else: 
            # Enable persistent inverse-kinematics-service to time-efficiently compute joint-values of cartesian poses
            ik_service = rospy.ServiceProxy("/compute_ik", GetPositionIK, persistent=True)
        
        # Set variables to organize the iteration
        valid_viewpoints = []
        processed = 0
    
        START =time.time()
        # For every sampled_surface_point: Generate viewpoint with anchor_point in normal-direction of the surface-point's face-triangle.
        # Then, rotate the orientation of the viewpoint's laser_emitter_frame around the z-Axis (= boresight) with respect to the anchor point (-> Changes only orientation).
        # If specified, apply tilting to the viewpoint for every orientation, so that the viewpoint's anchor point is turned around the laser_emitter_frame's x-/y-axis but 
        # with respect to the corresponding surface point (-> Changes orientation and position of viewpoint-anchor).
        for (surface_point, face_index) in zip(sampled_surface_points, sampled_face_indices):
            for angle in np.linspace(0, 360, orientations_around_boresight + 1)[:-1]:
                new_viewpoint = ViewPoint(surface_point, self.target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff())
                if viewpoint_tilt_mode == "none":
                    if self.process_viewpoint(new_viewpoint, ik_service, uncertainty_threshold, minimum_required_overmeasure):
                        valid_viewpoints.append(new_viewpoint)
                elif viewpoint_tilt_mode == "limited":
                    # angle_config = tuple describing (tilt around x-axis, tilt around y-axis) from the set of 2-element permutations with repitions of all valid tilt-angles
                    for angle_config in itertools.product([0, self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], repeat=2):
                        new_viewpoint = ViewPoint(surface_point, self.target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                        # When valid viewpoint could be obtained through tilting -> Continue to next orientation
                        if self.process_viewpoint(new_viewpoint, ik_service, uncertainty_threshold, minimum_required_overmeasure):
                            valid_viewpoints.append(new_viewpoint)
                            break
                elif viewpoint_tilt_mode == "full":
                    # angle_config = tuple describing (tilt around x-axis, tilt around y-axis) from the set of 2-element permutations with repitions of all valid tilt-angles
                    for angle_config in itertools.product([0, self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], repeat=2):
                        new_viewpoint = ViewPoint(surface_point, self.target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                        if self.process_viewpoint(new_viewpoint, ik_service, uncertainty_threshold, minimum_required_overmeasure):
                            valid_viewpoints.append(new_viewpoint)
                else:
                    raise ValueError("viewpoint_tilt_mode has to be 'none', 'limited' or 'full'. You entered: '{}'".format(viewpoint_tilt_mode))
                processed += 1
                print("Processed viewpoint {} of {} ({} %, {} of them are usable for measurement)".format(
                    processed, 
                    len(sampled_surface_points) * orientations_around_boresight, 
                    np.round(100.0 * processed / (len(sampled_surface_points) * orientations_around_boresight), 2),
                    len(valid_viewpoints)
                ))
                
        # Because the service was created "persistent", it must be closed properly
        if not plan_path_to_check_reachability:
            ik_service.close()

        print("\n" + "*" * 20 + "\nGenerated {} valid viewpoints with fully evaluated measurement-trajectory/-ies".format(len(valid_viewpoints)))
        print("DURATION WAS ", time.time() - START)
        return sampled_surface_points, valid_viewpoints


    def process_viewpoint(self, viewpoint, ik_service, uncertainty_threshold, minimum_required_overmeasure = 5, trajectory_sample_step = 2, joint_jump_threshold = 1.5, minimum_trajectory_length = 50):
        """
        Processes a single viewpoint entirely. Starting on a simple reachability-analysis of the anchor-pose, a metrological evaluation is performed using the sensor_model to
        reviewpoint the measurement-gain this viewpoint can contribute in theory. Then, the actual meausrement-trajectories are calculated and examined regarding the
        provided contraints. In the end, the processing-result (trajectory and metrological values) are stored in the viewpoint object

        :param viewpoint: ViewPoint to be processed (if processing was successful, members of this viewpoint-object will be changed)
        :type viewpoint: ViewPoint
        :param ik_service: ROS-Serviceproxy that can resolve inverse-kinematics via moveit_msgs/GetPositionIK 
        :type ik_service: rospy.ServiceProxy
        :param minimum_required_overmeasure: How much to add to the trajectory in trajectory-direction in mm after the last measurable sample_point is measured (as samples usually do not occur exactly at edges), defaults to 5
        :type minimum_required_overmeasure: float, optional
        :param trajectory_sample_step: Euclidian distance between 2 trajectory-points in cartesian sapce in mm, defaults to 2
        :type trajectory_sample_step: float, optional
        :param joint_jump_threshold: Maximum change in joint values allowed between 2 trajectory-points in rad (since all joints are revolute), defaults to 1.5
        :type joint_jump_threshold: float, optional
        :param minimum_trajectory_length: Minimum length a generated trajectory has to have so that it is accepted in mm, defaults to 50
        :type minimum_trajectory_length: float, optional
        :return: Boolean value that is true, if the viewpoint could be processed and all contraints were met, and false otherwise
        :rtype: bool
        """

        assert isinstance(viewpoint, ViewPoint)
        

        # Build a mathematical model of the trajectory (= straight line)
        trajectory_origin = viewpoint.get_anchor_position()
        # The scanner moves along the x-axis of the laser_emitter_frame
        trajectory_direction = viewpoint.get_orientation_matrix().dot([1, 0, 0, 1])[0:3]

        # Implementation of a line equation (t is in mm, but result is in m)
        trajectory_in_m = lambda t: (trajectory_origin + trajectory_direction * t) / 1000
        
        # Check if anchor of viewpoint is reachable

        q = trimesh.transformations.quaternion_from_matrix(viewpoint.get_orientation_matrix())
        viewpoint_anchor_pose = geometry_msgs.msg.PoseStamped()
        viewpoint_anchor_pose.pose.orientation.w = q[0]
        viewpoint_anchor_pose.pose.orientation.x = q[1]
        viewpoint_anchor_pose.pose.orientation.y = q[2]
        viewpoint_anchor_pose.pose.orientation.z = q[3]
        viewpoint_anchor_pose.header.frame_id = "world"
        viewpoint_anchor_pose.pose.position.x = trajectory_in_m(0)[0]
        viewpoint_anchor_pose.pose.position.y = trajectory_in_m(0)[1]
        viewpoint_anchor_pose.pose.position.z = trajectory_in_m(0)[2]
        if ik_service is None:
            self.group.set_start_state_to_current_state()
            self.group.set_pose_target(viewpoint_anchor_pose)
            initial_plan = self.group.plan()
            if not initial_plan[0]:
                return False
            # Use final joint-values (= joint values, where the sensor's frame is in the viewpointpoint)
            # as planning-start-point for the trajectory calculations
            viewpoint_anchor_state = moveit_msgs.msg.RobotState()
            viewpoint_anchor_state.joint_state.position = initial_plan[1].joint_trajectory.points[-1].positions
            viewpoint_anchor_state.joint_state.name = initial_plan[1].joint_trajectory.joint_names
        else:
            req = GetPositionIKRequest()
            req.ik_request.group_name = "manipulator"
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped = viewpoint_anchor_pose   
            # Perform 20 calls as inverse_kinematics is a stochastic process to improve chances of finding an
            for _ in range(20):
                resp = ik_service.call(req)
                if resp.error_code.val == GetPositionIKResponse().error_code.SUCCESS:
                    break
            if not resp.error_code.val == GetPositionIKResponse().error_code.SUCCESS:
                return False
            viewpoint_anchor_state= resp.solution
        
        # Use the computed joint-values as basis for joint-trajectory-planning
        self.group.set_start_state(viewpoint_anchor_state)

        # Perform metrological evaluation
        measurable_surface_point_indices, uncertainties, trajectory_line_arguments_selected = self.sensor_model.process_viewpoint_metrologically(viewpoint, uncertainty_threshold, self.max_edge)
        if len(measurable_surface_point_indices) == 0:
            return False

        # Check if the trajectory's lenght (when cut down so that only measurable surface-points are covered with no overhead where nothing is scanned anymore) is long enough
        max_deflection = max(trajectory_line_arguments_selected) + minimum_required_overmeasure
        min_deflection = min(trajectory_line_arguments_selected) - minimum_required_overmeasure
        if max_deflection - min_deflection < minimum_trajectory_length:
            return False
        
        # Compute cartesian path into the viewpoint's trajectory_direction from the previously found start 'anchor'-state to the cartesian pose
        # of the laser_emitter_frame where the last surface point can be measured (+ tiny overmeasure) at pose_a
        viewpoint_deflection_pose_a = geometry_msgs.msg.Pose()
        viewpoint_deflection_pose_a.orientation = viewpoint_anchor_pose.pose.orientation
        viewpoint_deflection_pose_a.position = geometry_msgs.msg.Point(*trajectory_in_m(max_deflection))
        # Info: fraction is the portion of the request to be executable by the returned trajectory
        # (e.g. 0.5 means that the trajectory only covers half the euklidian distance between the start and endpose)
        (trajectory, fraction_a) = self.group.compute_cartesian_path([viewpoint_anchor_pose.pose, viewpoint_deflection_pose_a], trajectory_sample_step * 1e-3, joint_jump_threshold)
        
        # Use the end-state of this trajectory as a start state of the trajectory to plan a trajectory against the trajectory direction to where the last
        # surface point can be measured (+ tiny overmeasure) at pose_b
        viewpoint_deflection_state_a = moveit_msgs.msg.RobotState()
        viewpoint_deflection_state_a.joint_state.name = trajectory.joint_trajectory.joint_names
        viewpoint_deflection_state_a.joint_state.position = trajectory.joint_trajectory.points[-1].positions
        self.group.set_start_state(viewpoint_deflection_state_a)
        # pose_a has to adapted to be the last actually reachable pose in trajectory direction
        viewpoint_deflection_pose_a.position = geometry_msgs.msg.Point(*trajectory_in_m(fraction_a * max_deflection))
        viewpoint_deflection_pose_b = geometry_msgs.msg.Pose()
        viewpoint_deflection_pose_b.orientation = viewpoint_deflection_pose_a.orientation
        viewpoint_deflection_pose_b.position = geometry_msgs.msg.Point(*trajectory_in_m(min_deflection))
        (trajectory, fraction_total) = self.group.compute_cartesian_path([viewpoint_deflection_pose_a, viewpoint_deflection_pose_b], trajectory_sample_step * 1e-3, joint_jump_threshold)

        # Calculate the length of the trajectory that is actual executable with respect to joint-limits, collision, ... and check if post-processing was successful
        fraction_b = (fraction_total * (fraction_a * max_deflection - min_deflection) - fraction_a * max_deflection) / abs(min_deflection)
        if (fraction_a * max_deflection - fraction_b * min_deflection) < minimum_trajectory_length or not self.postprocess_trajectory(trajectory):
            return False
        
        # Remove points that were covered by a theoretical trajectory however are not measurable in practice as the actual trajectory can be shorter due to constraints
        for i in reversed(range(len(trajectory_line_arguments_selected))):
            if trajectory_line_arguments_selected[i] > fraction_a * max_deflection or trajectory_line_arguments_selected[i] < fraction_b * min_deflection:
                measurable_surface_point_indices.pop(i)
                uncertainties.pop(i)

        # If restrictions are satisfied, store the measurement-trajectory and metrological information in the viewpoint
        viewpoint.set_trajectory_for_measurement(trajectory)
        viewpoint.set_measurable_surface_point_indices_and_uncertainties(measurable_surface_point_indices, uncertainties)
        return True

    def postprocess_trajectory(self, trajectory):
        """
        Check if trajectory meets the limits specified in 'joint_limits.yaml' and if the time between two points is increasing.
        MoveIt does not apply the values from 'joint_limits.yaml' when computing a cartesian path, which is dangerous for trajectory-execution.
        For prevention, trajectories with too heavy violations will be rejected. However, specifying the joint-limits from the yaml in the xacro-urdf as
        well seems to have eliminated this problem (but this code is still active for safety purposes).

        :param trajectory: RobotTrajectory to check and improve
        :type trajectory: moveit_msgs/RobotTrajectory
        :return: Boolean value indicating if the trajectory could be post-processed reasonably or if it has to be rejected
        :rtype: bool
        """

        params = rospy.get_param("/robot_description_planning/joint_limits")
        
        point_indices_to_pop = []
        # For every point in the trajectory check for every joint_position if there are limits and examinate if they are violated.
        # When there are only very small violations, the value is set to the limit's value.
        for i in range(len(trajectory.joint_trajectory.points)):
            for j, position in enumerate(trajectory.joint_trajectory.points[i].positions):
                if params[trajectory.joint_trajectory.joint_names[j]].has_key("max_position"):
                    diff = position - params[trajectory.joint_trajectory.joint_names[j]]["max_position"]
                    if diff >= 1e-3:
                        print(trajectory.joint_trajectory.joint_names[j], position)
                        return False
                    elif diff > 0 and diff < 1e-3:
                        # Use non-intuitive assignment as positions (a tuple value) must not be 
                        # changed elementwise. It has to be replaced entirely... (same below)
                        trajectory.joint_trajectory.points[i].positions = tuple(
                            list(trajectory.joint_trajectory.points[i].positions[:j])
                            + [params[trajectory.joint_trajectory.joint_names[j]]["max_position"]]
                            + list(trajectory.joint_trajectory.points[i].positions)[j + 1:]
                        )
                if params[trajectory.joint_trajectory.joint_names[j]].has_key("min_position"):
                    diff = position - params[trajectory.joint_trajectory.joint_names[j]]["min_position"]
                    if diff <= -1e-3:
                        print(trajectory.joint_trajectory.joint_names[j], position)
                        return False
                    elif diff < 0 and diff > -1e-3:     
                        trajectory.joint_trajectory.points[i].positions = tuple(
                            list(trajectory.joint_trajectory.points[i].positions[:j])
                            + [params[trajectory.joint_trajectory.joint_names[j]]["min_position"]]
                            + list(trajectory.joint_trajectory.points[i].positions)[j + 1:]
                        )
            # Check if time is monotone ascending (if not -> Delete the point)
            if i < len(trajectory.joint_trajectory.points) - 1:
                if trajectory.joint_trajectory.points[i].time_from_start == trajectory.joint_trajectory.points[i + 1].time_from_start:
                    point_indices_to_pop.append(i)
        

        for point_index_to_pop in sorted(point_indices_to_pop, reverse=True):
            trajectory.joint_trajectory.points.pop(point_index_to_pop)

        return True

    def solve_scp(self, provided_viewpoints, solver_type="greedy"):
        """
        Solve Set Covering Problem to cover all measurable surface_points with a fraction of the set of provided viewpoints.
        Possible solver_types are:\n
        -   "greedy": Fills return set at each step with the trajectory that delivers the most
            additional coverage compared to the points already in the set. If this additional coverage is
            identical in size for several new optimal viewpointpoint-candidates, the one with the lowest maximum
            uncertainty will be added similarly to the improvement introduced in chapter 4.4 of 
            "ViewPoint and sensor planning for multi-sensor surface inspection" (Gronle et al., 2016)  (default)\n
        -   "IP_basic": Solves the SCP-Problem with integer programming (IP) using the formulation in
            "Model-based viewpoint planning" (Scott, 2009), i.e. the objective function is the number of all
            selected viewpointpoints whereas the constraint is that every surface point must be covered at least by one viewpointpoint\n
        -   "IP_uncertainty": Solves the SCP using IP with respect to the uncertainty. The formulas are similar to "IP_basic"
            but in the objective, costs corresponding to the worst uncertainty are assigned to every viewpointpoint-trajectory.\n
        -   "IP_time": Solves the SCP using IP with respect to the time of trajectory-execution. The formulas are similar to "IP_basic"
            but in the objective, costs corresponding to the duration of the trajectory are assigned to every viewpointpoint.

        :param provided_viewpoints: All processed viewpoints where each has a valid measurement-trajectory and information about the measurable surface_points
        :type provided_viewpoints: set[ViewPoint] or list[ViewPoint]
        :param solver_type: See function description, defaults to "greedy"
        :type solver_type: str, optional
        :return: Set of viewpoints that can measure the union of measurable surface points of all provided viewpoints
        :rtype: set[ViewPoint]
        """
        # Get set of all measurable surface points by all provided_viewpoints (this is most likely smaller than the set of sampled_surface_points)
        measurable_surface_point_indices = set(
                itertools.chain.from_iterable(
                    [viewpoint.get_measurable_surface_point_indices() for viewpoint in provided_viewpoints]
                ))
        print("Can cover {} surface-points".format(len(measurable_surface_point_indices)))
        if solver_type == "greedy":
            measured_surface_point_indices = set()
            used_viewpoints = set()
            maximum_uncertainty = self.sensor_model.get_max_uncertainty()
            while len(provided_viewpoints) > 0:
                # The sort-key-function is designed as follows: Pre-decimal-point numbers indicate the amount of newly measurable surface contributed by this viewpoint.
                # Post-decimal-point-values indicate the lowest overall uncertainty of this viewpoint (in [0,1]) scaled by 0.9. That way, when multiple viewpoints provide
                # the same amount of new measurements, the uncertainty will be compared as in Gronle et al.'s paper.
                provided_viewpoints = sorted(provided_viewpoints, key = lambda viewpoint: len(measured_surface_point_indices | set(viewpoint.get_measurable_surface_point_indices())) + (maximum_uncertainty - max(viewpoint.get_measurable_surface_point_uncertainties())) / maximum_uncertainty * 0.9, reverse=True)
                next_viewpoint = provided_viewpoints.pop(0)
                measured_surface_point_indices |= set(next_viewpoint.get_measurable_surface_point_indices())
                used_viewpoints.add(next_viewpoint)
                # Check if optimal situation is reached, i.e. all measurable surface points are covered
                if measured_surface_point_indices == measurable_surface_point_indices:
                    print("Covered {} surface points. Continuing with this solution ...".format(len(measured_surface_point_indices)))
                    return used_viewpoints
            raise Exception("Greedy algorithm failed. All viewpoints were used but not all measurable points were covered!")

        elif solver_type[0:3] == "IP_":
            # This is an implementation of the problem stated in Scott's paper with some additions.        
            viewpointpoint_indices = range(len(provided_viewpoints))
            
            # Create problem
            ip_problem = pulp.LpProblem(sense=pulp.const.LpMinimize)
            
            # Create IP-Variables (viewpointpoint_variables[i] is 1, if provided_viewpoint[i] is part of the solution and 0 otherwise)
            viewpointpoint_variables = pulp.LpVariable.dicts(name="viewpointpoint_variables", indexs=viewpointpoint_indices, cat=pulp.const.LpInteger)
            
            # Add objective function
            if solver_type == "IP_basic":
                # Optimize only for the total amount of viewpointpoints
                ip_problem += pulp.lpSum([viewpointpoint_variables[i] for i in viewpointpoint_indices])
            elif solver_type == "IP_time":
                # Goal is that the combined measurement-trajectory-execution-time of all measurement-trajectories becomes minimal
                ip_problem += pulp.lpSum([viewpointpoint_variables[i] * provided_viewpoints[i].get_trajectory_for_measurement().joint_trajectory.points[-1].time_from_start.to_sec() for i in viewpointpoint_indices])
            elif solver_type == "IP_uncertainty":
                # Goal is that the sum of 
                ip_problem += pulp.lpSum([viewpointpoint_variables[i] * sum(provided_viewpoints[i].get_measurable_surface_point_uncertainties()) / len(provided_viewpoints[i].get_measurable_surface_point_uncertainties()) for i in viewpointpoint_indices])
        
            
            # Add constraints: ViewPointpoint_variable's elements are either 1 or 0
            for i in viewpointpoint_indices:
                ip_problem += viewpointpoint_variables[i] <= 1
                ip_problem += viewpointpoint_variables[i] >= 0
            
            # Add constraints: Every surface point must be covered
            for surface_point_index in measurable_surface_point_indices:
                ip_problem += pulp.lpSum([(surface_point_index in provided_viewpoints[viewpointpoint_index].get_measurable_surface_point_indices()) * viewpointpoint_variables[viewpointpoint_index] for viewpointpoint_index in viewpointpoint_indices]) >= 1
            
            print("Constructed integer programming problem. Start solving...")
            ip_problem.solve()
            
            # Process IP result
            if not ip_problem.status == pulp.const.LpSolutionOptimal:
                raise Exception("Could not find optimal solution.")
            print("Found optimal solution consisting of {} viewpointpoints".format(pulp.value(ip_problem.objective)))
            output = set()
            for solved_viewpointpoint_variable in ip_problem.variables():
                if solved_viewpointpoint_variable.varValue == 1:  
                    output.add(provided_viewpoints[int(solved_viewpointpoint_variable.name.split("_")[-1])])
            return output           
        
    def connect_viewpoints(self, unordered_viewpoints, min_planning_time=0.2):
        """
        Connect a set of unordered viewpoints with the current state and in between greedily so that they can be executed as fast as possible.
        Until all viewpoints are enqueued, do: From the end-point of the last enqueued trajetory, motion plans are calculated to the start-/end-poses
        of all unenqueued viewpoint's measurement-trajectories and the shortest (in time domain) will be selected. 

        :param unordered_viewpoints: Set of viewpoints to be connected where each has a stored measurement-trajectory 
        :type unordered_viewpoints: set[viewpoint]
        :param min_planning_time: Planning time that is used for connection-path-planning (will be increased automatically if no plan was found at first), defaults to 0.2
        :type min_planning_time: float, optional
        :return: List of ordered and execution-ready viewpoints
        :rtype: list[ViewPoint]
        """
        if len(unordered_viewpoints) == 0:
            return
        # Convert to list to make 'pop' easier
        unenqueued_viewpoints = list(unordered_viewpoints)

        # Output list = ViewPoints with stored shortest inter-viewpoint-paths
        enqueud_viewpoints = []
        
        # Introduce initial robot pose as pseudo "viewpoint"
        # (it has a trajectory with the current pose as its only (end-)point)
        initial_pseudo_viewpoint = ViewPoint(np.array([1,0,0]), np.array([1,0,0]), 0, 0)
        pseudo_trajectory = moveit_msgs.msg.RobotTrajectory()
        pseudo_end_point = trajectory_msgs.msg.JointTrajectoryPoint()
        pseudo_end_point.positions = self.group.get_current_joint_values()
        pseudo_end_point.time_from_start = rospy.Duration(1e9)
        pseudo_trajectory.joint_trajectory.joint_names = unenqueued_viewpoints[0].get_trajectory_for_measurement().joint_trajectory.joint_names
        pseudo_trajectory.joint_trajectory.points.append(pseudo_end_point)
        initial_pseudo_viewpoint.set_trajectory_for_measurement(pseudo_trajectory)       
        enqueud_viewpoints.append(initial_pseudo_viewpoint)

        while len(unenqueued_viewpoints) != 0:
            # Set start state for planning to the last pose of the previously enqueued trajectory
            temporary_start_state = moveit_msgs.msg.RobotState()
            temporary_start_state.joint_state.position = enqueud_viewpoints[-1].get_trajectory_for_measurement().joint_trajectory.points[-1].positions
            temporary_start_state.joint_state.name = enqueud_viewpoints[-1].get_trajectory_for_measurement().joint_trajectory.joint_names
            self.group.set_start_state(temporary_start_state)
            self.group.set_planning_time(min_planning_time)
            
            # For each unenqueued viewpoint: Calculate a path to the start- and end-point of its measurement trajectory and remember the time-shortest
            # trajectory by index in the unenqueued-list (as well as the corresponding trajectory and if the path was calculated to the end-point -> reverse_flag)
            min_index = None
            reverse_flag = False
            min_plan = pseudo_trajectory
            for index, viewpoint in enumerate(unenqueued_viewpoints):
                self.group.set_joint_value_target(viewpoint.get_trajectory_for_measurement().joint_trajectory.points[0].positions)
                
                plan_result = self.group.plan()
                if plan_result[0] and (min_plan.joint_trajectory.points[-1].time_from_start > plan_result[1].joint_trajectory.points[-1].time_from_start or min_index is None):
                    min_plan = plan_result[1]
                    min_index = index
                    reverse_flag = False
                
                self.group.set_joint_value_target(viewpoint.get_trajectory_for_measurement().joint_trajectory.points[-1].positions)
                
                plan_result = self.group.plan()
                if plan_result[0] and (min_plan.joint_trajectory.points[-1].time_from_start > plan_result[1].joint_trajectory.points[-1].time_from_start or min_index is None):
                    min_plan = plan_result[1]
                    min_index = index
                    reverse_flag = True
            if min_index is not None:
                # Shortest connection-plan found -> Enqueuing (and if necessary reverting)
                next_viewpoint = unenqueued_viewpoints.pop(min_index)
                if reverse_flag:
                    next_viewpoint.reverse_trajectory_for_measurement()   
                next_viewpoint.set_trajectory_to_viewpoint(min_plan)
                enqueud_viewpoints.append(next_viewpoint)
                # Reset planning time
                self.group.set_planning_time(min_planning_time)
                print("Enqueued viewpointpoint-trajectory ({} left)".format(len(unenqueued_viewpoints)))
            else:
                # No solution has been found -> Increase planning time to improve success-chances for the next iteration
                self.group.set_planning_time(min_planning_time + self.group.get_planning_time())
        return enqueud_viewpoints[1:]



    def execute(self, execution_list, surface_points=None):
        """
        Execute a list of RobotTrajectories or ViewPoints via MoveIt. When problems occur during execution,
        the robot will be stopped and an exception will be raised. When executing from a list of viewpoints, the currently measured surface_points
        are published in "/currently_measured_points" during execution.

        :param execution_list: List of RobotTrajectories or viewpoints that can be executed consecutively (the next segment's start point is the last segment's end point)
        :type execution_list: list[moveit_msgs/RobotTrajectory] or list[ViewPoint]
        :param surface_points: List of the actual sampled surface points, defaults to None
        :type surface_points: list[numpy.array], optional
        """
        
        # Only create publisher if surface_points are available
        if surface_points is not None:
            measurement_pub = rospy.Publisher("/currently_measured_points", sensor_msgs.msg.PointCloud, queue_size=1, latch=True)
            point_message = sensor_msgs.msg.PointCloud()
            point_message.header.frame_id = "world"
        
        # Control stitching: Only active during trajectory-execution (and not when steering between trajectories)
        rospy.wait_for_service("/switch_stitch_mode", 5.0)
        stitch_service = rospy.ServiceProxy("/switch_stitch_mode", SetBool)
        
        # Flag for keeping track about what is executed at the moment (when execution_list consists of RobotTrajectories, 
        # they shifting between steering-to-measurement-segment [False] and measurement-trajectory-segment [True])
        trajectory_flag = False
        
        # Consecutively execute segments/ViewPoints with short breaks between every trajectory-execution to enure nothing is moving anymore
        for i, execution_segment in enumerate(execution_list):
            if isinstance(execution_segment, ViewPoint):
                # Publish surface points, when given
                if surface_points is not None:
                    point_message.points = []
                    for point_index in execution_segment.get_measurable_surface_point_indices():
                        point_message.points.append(geometry_msgs.msg.Point(*(surface_points[point_index] / 1000)))
                    measurement_pub.publish(point_message)
                
                # Firstly, steer to the start point of the measurement-trajectory
                trajectory = execution_segment.get_trajectory_to_viewpoint()
                print("Steering to next measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
                stitch_service.call(SetBoolRequest(data=False))
                if not self.group.execute(trajectory):
                    self.group.stop()
                    raise Exception("Failed to steer to trajectory")
                rospy.sleep(0.1)

                # Secondly, execute the measurement-trajectory
                trajectory = execution_segment.get_trajectory_for_measurement()
                print("Executing measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
                stitch_service.call(SetBoolRequest(data=True))
                if not self.group.execute(trajectory):
                    self.group.stop()
                    raise Exception("Failed to execute trajectory")
                rospy.sleep(0.1)

                print("Executed viewpoint {} of {}".format(i + 1, len(execution_list)))
                
                
            if isinstance(execution_segment, moveit_msgs.msg.RobotTrajectory):
                if trajectory_flag:
                    stitch_service.call(SetBoolRequest(data=True))
                    print("Executing measurement-trajectory (time: {}s)...".format(execution_segment.joint_trajectory.points[-1].time_from_start.to_sec()))
                else:
                    stitch_service.call(SetBoolRequest(data=False))
                    print("Steering to next measurement-trajectory (time: {}s)...".format(execution_segment.joint_trajectory.points[-1].time_from_start.to_sec()))
                if not self.group.execute(execution_segment):
                    self.group.stop()
                    raise Exception("Failed to execute path segment")
                print("Executed segment {} of {}".format(i + 1, len(execution_list)))
                rospy.sleep(0.1)
                trajectory_flag = not trajectory_flag
        
        # Finally, ensure stitching is disabled     
        stitch_service.call(SetBoolRequest(data=False))   

    def perform_all(self, target_mesh_filename, density, orientations_around_boresight, uncertainty_threshold):
        self.load_target_mesh(target_mesh_filename, transform=trimesh.transformations.translation_matrix(OFFSET))
        # surface_pts, viewpoints  = self.generate_samples_and_viewpoints(density, uncertainty_threshold, orientations_around_boresight)
        # scp_viewpoints = self.solve_scp(viewpoints, "greedy")
        # connected_viewpoints = self.connect_viewpoints(scp_viewpoints)
        #execution_plan = self.convert_viewpointlist_to_execution_plan(connected_viewpoints)
        #self.store_execution_plan("/home/svenbecker/Bachelorarbeit/test/stored_plans/motor_high_res.yaml", execution_plan, {"Time of calculation": "Jetzt"})
        # raw_input("JETZT")
        execution_plan = self.load_execution_plan(os.path.join(rospkg.RosPack().get_path("agiprobot_measurement"), "benchmark_results/execution_plan_objectA_wuerfel_rho_001_solver_IP_basic_angels_around_boresight_3.yaml"))
        self.execute(execution_plan)
        
if __name__ == "__main__":
    def signal_handler(signum_a, signum_b):
        exit()
    signal.signal(signal.SIGINT, signal_handler)
    tm = TrajectoryManager()
    tm.perform_all(os.path.join(rospkg.RosPack().get_path("agiprobot_measurement"), "benchmark_meshes/objectA_wuerfel.stl"), 0.01, 3, 100e-3)
    rospy.sleep(1)
    rospy.spin()
    