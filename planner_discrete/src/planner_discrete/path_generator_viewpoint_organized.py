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
from planner_discrete.view import View
from planner_discrete.sensor_model import SensorModel
import yaml
from rospy_message_converter import message_converter
import click


OFFSET = np.array([600, -600, 1300])

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
        self.group.set_planner_id("PersistentPRMstar") # Enables multi-query-planning (see corresponding literature)
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
    
    def convert_viewlist_to_execution_plan(self, viewlist):
        """
        Convert a list of View-objects into a list of moveit_msgs/RobotTrajectory-entries so that the views can be executed consecutively.
        
        :param viewlist: List of View-objects with set trajectories for steering-to-view and measurement
        :type vievlist: list
        :returns: List of moveit_msgs/RobotTrajectory with 2 entries per provided View-object (first = steering-to-view-trajectory, second = measurement-trajectory)
        :rtype: list
        """
        assert isinstance(viewlist, list)
        execution_plan = []
        for view in viewlist:
            assert isinstance(view, View)
            execution_plan.append(view.get_trajectory_to_view())
            execution_plan.append(view.get_trajectory_for_measurement())
        return execution_plan



    
    def load_target_mesh(self, file_name, transform = np.eye(4), add_wbk_mirrors=True):
        """
        Loads the mesh-to-measure from file into the instance's target_mesh-member (to generate and evaluate view-trajectories) and into
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


        # Add mesh to MoveIt so that it will be considered as a collision object during planning
        # (here, the given transform also needs to be applied)
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
            # add_mesh hat troubles with "non-root" filenames (e.g. ~/cadfiles/target.stl)
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

    def generate_samples_and_views(self, viewpoint_density, N_angles_per_view, view_tilt_mode="aggressive"):
        


        surface_points, face_indices = trimesh.sample.sample_surface_even(self.target_mesh, int(self.target_mesh.area * viewpoint_density))
        
        print("OK")
        """
        pub = rospy.Publisher("/sampled_surface_points", sensor_msgs.msg.PointCloud, latch=True, queue_size=1)
        surface_points_message = sensor_msgs.msg.PointCloud()
        surface_points_message.header.frame_id = "world"
        
        for surface_point in surface_points:
            surface_points_message.points.append(geometry_msgs.msg.Point(*(surface_point / 1000)))
        pub.publish(surface_points_message)"""
        self.max_edge = max(self.target_mesh.bounding_box_oriented.edges_unique_length)
        self.group.set_planning_time(0.04)
        valid_views = []
        processed = 0
        # Estimated as circle radius according to density + safety offset
        minimum_required_overmeasure = np.sqrt(1 / (np.pi * viewpoint_density)) + 5
        ik_service = rospy.ServiceProxy("/compute_ik", GetPositionIK, persistent=True)
        START =time.time()
        self.sensor_model.set_processing_context(self.target_mesh, surface_points, face_indices)
        #ik_service = None
        
        for (surface_point, face_index) in zip(surface_points, face_indices):
            for angle in np.linspace(0, 360, N_angles_per_view + 1)[:-1]:
                new_view = View(surface_point, self.target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff())
                if view_tilt_mode == "none":
                    if self.process_view(new_view, ik_service, minimum_required_overmeasure):
                        valid_views.append(new_view)
                elif view_tilt_mode == "conservative":
                    for angle_config in itertools.product([0, self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], repeat=2):
                        new_view = View(surface_point, self.target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                        if self.process_view(new_view, ik_service, minimum_required_overmeasure):
                            valid_views.append(new_view)
                            break  
                elif view_tilt_mode == "aggressive":
                    for angle_config in itertools.product([0, self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], repeat=2):
                        new_view = View(surface_point, self.target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                        if self.process_view(new_view, ik_service, minimum_required_overmeasure):
                            valid_views.append(new_view)
                processed += 1
                print("Processed view {} of {} ({} %, {} of them are usable for measurement)".format(
                    processed, 
                    len(surface_points) * N_angles_per_view, 
                    np.round(100.0 * processed / (len(surface_points) * N_angles_per_view), 2),
                    len(valid_views)
                ))
                


        ik_service.close()
        print("\n" + "*" * 20 + "\nGenerated {} valid measurement-trajectory/-ies".format(len(valid_views)))
        print("DURATION WAS ", time.time() - START)
        return surface_points, valid_views

    def process_view(self, view, ik_service, minimum_required_overmeasure_mm = 5, sample_step_mm = 2, joint_jump_threshold = 1.5, minimum_trajectory_length_mm = 50):
        assert isinstance(view, View)
        
        trajectory_origin = view.get_anchor_position()
        trajectory_direction = view.get_orientation_matrix().dot([1, 0, 0, 1])[0:3]

        # Implementation of a line equation (t is in mm, but result is in m)
        trajectory_in_m = lambda t: (trajectory_origin + trajectory_direction * t) / 1000
        
        # Check if anchor of view is reachable 
        q = trimesh.transformations.quaternion_from_matrix(view.get_orientation_matrix())
        view_center_pose = geometry_msgs.msg.PoseStamped()
        view_center_pose.pose.orientation.w = q[0]
        view_center_pose.pose.orientation.x = q[1]
        view_center_pose.pose.orientation.y = q[2]
        view_center_pose.pose.orientation.z = q[3]
        view_center_pose.header.frame_id = "world"
        view_center_pose.pose.position.x = trajectory_in_m(0)[0]
        view_center_pose.pose.position.y = trajectory_in_m(0)[1]
        view_center_pose.pose.position.z = trajectory_in_m(0)[2]
        """
        self.group.set_start_state_to_current_state()
        self.group.set_pose_target(view_center_pose)
        initial_plan = self.group.plan()
        if not initial_plan[0]:
            print("Reject view because it is not reachable")
            return False
        # Use final joint-values (= joint values, where the sensor's frame is in the viewpoint)
        # as planning-start-point for the trajectory calculations
        view_center_state = moveit_msgs.msg.RobotState()
        view_center_state.joint_state.position = initial_plan[1].joint_trajectory.points[-1].positions
        view_center_state.joint_state.name = initial_plan[1].joint_trajectory.joint_names
        """
        req = GetPositionIKRequest()
        req.ik_request.group_name = "manipulator"
        req.ik_request.avoid_collisions = True
        # req.ik_request.robot_state = self.group.get_current_state()
        req.ik_request.pose_stamped = view_center_pose
        
        for _ in range(20):
            resp = ik_service.call(req)
            if resp.error_code.val == GetPositionIKResponse().error_code.SUCCESS:
                break
        if not resp.error_code.val == GetPositionIKResponse().error_code.SUCCESS:
            return False
        view_center_state= resp.solution
            
        self.group.set_start_state(view_center_state)

        
        
        measurable_surface_points, uncertainty_scores, trajectory_line_arguments_selected = self.sensor_model.process_view_metrologically(view, self.max_edge)



        if len(measurable_surface_points) == 0:
            return False
        max_deflection = max(trajectory_line_arguments_selected) + minimum_required_overmeasure_mm
        min_deflection = min(trajectory_line_arguments_selected) - minimum_required_overmeasure_mm
        if max_deflection - min_deflection < minimum_trajectory_length_mm:
            return False
        
        view_deflection_pose_a = geometry_msgs.msg.Pose()
        view_deflection_pose_a.orientation = view_center_pose.pose.orientation
        view_deflection_pose_a.position = geometry_msgs.msg.Point(*trajectory_in_m(max_deflection))

        (trajectory, fraction_a) = self.group.compute_cartesian_path([view_center_pose.pose, view_deflection_pose_a], sample_step_mm * 1e-3, joint_jump_threshold)
        
        view_deflection_state_a = moveit_msgs.msg.RobotState()
        view_deflection_state_a.joint_state.name = trajectory.joint_trajectory.joint_names
        view_deflection_state_a.joint_state.position = trajectory.joint_trajectory.points[-1].positions
        self.group.set_start_state(view_deflection_state_a)

        view_deflection_pose_a.position = geometry_msgs.msg.Point(*trajectory_in_m(fraction_a * max_deflection))
        
        view_deflection_pose_b = geometry_msgs.msg.Pose()
        view_deflection_pose_b.orientation = view_deflection_pose_a.orientation
        view_deflection_pose_b.position = geometry_msgs.msg.Point(*trajectory_in_m(min_deflection))

        (trajectory, fraction_total) = self.group.compute_cartesian_path([view_deflection_pose_a, view_deflection_pose_b], sample_step_mm * 1e-3, joint_jump_threshold)
        fraction_b = (fraction_total * (fraction_a * max_deflection - min_deflection) - fraction_a * max_deflection) / abs(min_deflection)
        if (fraction_a * max_deflection - fraction_b * min_deflection) < minimum_trajectory_length_mm or not self.postprocess_trajectory(trajectory):
            return False
        view.set_trajectory_for_measurement(trajectory)
        view.set_measurable_surface_point_indices_and_scores(measurable_surface_points, uncertainty_scores)
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

    def solve_scp(self, provided_views, solver_type="greedy"):
        """
        Solve Set Covering Problem to cover all measurable surface_points with a fraction of the set of provided views.
        Possible solver_types are: 
        -   "greedy": Fills return set at each step with the trajectory that delivers the most
            additional coverage compared to the points already in the set. If this additional coverage is
            identical in size for several new optimal viewpoint-candidates, the one with the lowest maximum
            uncertainty will be added similarly to the improvement introduced in chapter 4.4 of 
            "View and sensor planning for multi-sensor surface inspection" (Gronle et al., 2016)  (default)
        -   "IP_basic": Solves the SCP-Problem with integer programming (IP) using the formulation in
            "Model-based view planning" (Scott, 2009), i.e. the objective function is the number of all
            selected viewpoints whereas the constraint is that every surface point must be covered at least by one viewpoint
        -   "IP_uncertainty": Solves the SCP using IP with respect to the uncertainty. The formulas are similar to "IP_basic"
            but in the objective, costs corresponding to the worst uncertainty are assigned to every viewpoint-trajectory.
        -   "IP_time": Solves the SCP using IP with respect to the time of trajectory-execution. The formulas are similar to "IP_basic"
            but in the objective, costs corresponding to the duration of the trajectory are assigned to every viewpoint.

        :param provided_views: All processed views where each has a valid measurement-trajectory and information about the measurable surface_points
        :type provided_views: set[View] or list[View]
        :param solver_type: See function description, defaults to "greedy"
        :type solver_type: str, optional
        :return: Set of views that can measure the union of measurable surface points of all provided views
        :rtype: set[View]
        """
        # Get set of all measurable surface points by all provided_views (this is most likely smaller than the set of sampled_surface_points)
        measurable_surface_point_indices = set(
                itertools.chain.from_iterable(
                    [view.get_measurable_surface_point_indices() for view in provided_views]
                ))
        provided_views = list(provided_views)

        if solver_type == "greedy":
            measured_surface_point_indices = set()
            used_views = set()
            while len(provided_views) > 0:
                # The sort-key-function is designed as follows: Pre-decimal-point numbers indicate the amount of newly measurable surface contributed by this view.
                # Post-decimal-point-values indicate the lowest overall uncertainty-score of this view (in [0,1]) scaled by 0.9. That way, when multiple views provide
                # the same amount of new measurements, the uncertainty will be compared just in Gronle et al.'s paper.
                provided_views = sorted(provided_views, key = lambda view: len(measured_surface_point_indices | set(view.get_measurable_surface_point_indices())) + min(view.get_measurable_surface_point_scores()) * 0.9, reverse=True)
                next_view = provided_views.pop(0)
                measured_surface_point_indices |= set(next_view.get_measurable_surface_point_indices())
                used_views.add(next_view)
                # Check if optimal situation is reached, i.e. all measurable surface points are covered
                if measured_surface_point_indices == measurable_surface_point_indices:
                    print("Covered {} surface points. Continuing with this solution ...".format(len(measured_surface_point_indices)))
                    return used_views
            raise Exception("Greedy algorithm failed. All views were used but not all measurable points were covered!")

        elif solver_type[0:3] == "IP_":
            # This is an implementation of the problem stated in Scott's paper with some additions.        
            viewpoint_indices = range(len(provided_views))
            
            # Create problem
            ip_problem = pulp.LpProblem(sense=pulp.const.LpMinimize)
            
            # Create IP-Variables (viewpoint_variables[i] is 1, if provided_view[i] is part of the solution and 0 otherwise)
            viewpoint_variables = pulp.LpVariable.dicts(name="viewpoint_variables", indexs=viewpoint_indices, cat=pulp.const.LpInteger)
            
            # Add objective function
            if solver_type == "IP_basic":
                # Optimize only for the total amount of viewpoints
                ip_problem += pulp.lpSum([viewpoint_variables[i] for i in viewpoint_indices])
            elif solver_type == "IP_time":
                # Goal is that the combined trajectory-execution-time of all measurement-trajectories becomes minimal
                ip_problem += pulp.lpSum([viewpoint_variables[i] * provided_views[i].get_trajectory_for_measurement().joint_trajectory.points[-1].time_from_start.to_sec() for i in viewpoint_indices])
            elif solver_type == "IP_uncertainty":
                ip_problem += pulp.lpSum([viewpoint_variables[i] * (1 - sum(provided_views[i].get_measurable_surface_point_indices()) / len(provided_views[i].get_measurable_surface_point_indices())) for i in viewpoint_indices])
            elif solver_type == "IP_combined":
                ip_problem += pulp.lpSum([viewpoint_variables[i] * (1 - sum(provided_views[i].get_measurable_surface_point_indices()) / len(provided_views[i].get_measurable_surface_point_indices())) * provided_views[i].get_trajectory_for_measurement().joint_trajectory.points[-1].time_from_start.to_sec() for i in viewpoint_indices])
            
            # Add constraints: Viewpoint_variable's elements are either 1 or 0
            for i in viewpoint_indices:
                ip_problem += viewpoint_variables[i] <= 1
                ip_problem += viewpoint_variables[i] >= 0
            
            # Add constraints: Every surface point must be covered
            for surface_point_index in measurable_surface_point_indices:
                ip_problem += pulp.lpSum([(surface_point_index in provided_views[viewpoint_index].get_measurable_surface_point_indices()) * viewpoint_variables[viewpoint_index] for viewpoint_index in viewpoint_indices]) >= 1
            
            print("Constructed integer programming problem. Start solving...")
            ip_problem.solve()
            
            # Process IP result
            if not ip_problem.status == pulp.const.LpSolutionOptimal:
                raise Exception("Could not find optimal solution.")
            print("Found optimal solution consisting of {} viewpoints".format(pulp.value(ip_problem.objective)))
            output = set()
            for solved_viewpoint_variable in ip_problem.variables():
                if solved_viewpoint_variable.varValue == 1:  
                    output.add(provided_views[int(solved_viewpoint_variable.name.split("_")[-1])])
            return output           

    def connect_views(self, unordered_views, min_planning_time=0.2):
        """
        Connect a set of unordered views with the current state and in between greedily so that they can be executed as fast as possible.
        Until all views are enqueued, do: From the end-point of the last enqueued trajetory, motion plans are calculated to the start-/end-poses
        of all unenqueued view's measurement-trajectories and the shortest (in time domain) will be selected. 

        :param unordered_views: Set of views to be connected where each has a stored measurement-trajectory 
        :type unordered_views: set[view]
        :param min_planning_time: Planning time that is used for connection-path-planning (will be increased automatically if no plan was found at first), defaults to 0.2
        :type min_planning_time: float, optional
        :return: List of ordered and execution-ready views
        :rtype: list[View]
        """
        if len(unordered_views) == 0:
            return
        # Convert to list to make 'pop' easier
        unenqueued_views = list(unordered_views)

        # Output list = Views with stored shortest inter-view-paths
        enqueud_views = []
        
        # Introduce initial robot pose as pseudo "view"
        # (it has a trajectory with the current pose as its only (end-)point)
        initial_pseudo_view = View(np.array([1,0,0]), np.array([1,0,0]), 0, 0)
        pseudo_trajectory = moveit_msgs.msg.RobotTrajectory()
        pseudo_end_point = trajectory_msgs.msg.JointTrajectoryPoint()
        pseudo_end_point.positions = self.group.get_current_joint_values()
        pseudo_end_point.time_from_start = rospy.Duration(1e9)
        pseudo_trajectory.joint_trajectory.joint_names = unenqueud_views[0].get_trajectory_for_measurement().joint_trajectory.joint_names
        pseudo_trajectory.joint_trajectory.points.append(pseudo_end_point)
        initial_pseudo_view.set_trajectory_for_measurement(pseudo_trajectory)       
        enqueud_views.append(initial_pseudo_view)

        while len(unenqueud_views) != 0:
            # Set start state for planning to the last pose of the previously enqueued trajectory
            temporary_start_state = moveit_msgs.msg.RobotState()
            temporary_start_state.joint_state.position = enqueud_views[-1].get_trajectory_for_measurement().joint_trajectory.points[-1].positions
            temporary_start_state.joint_state.name = enqueud_views[-1].get_trajectory_for_measurement().joint_trajectory.joint_names
            self.group.set_start_state(temporary_start_state)
            
            # For each unenqueued view: Calculate a path to the start- and end-point of its measurement trajectory and remember the time-shortest
            # trajectory by index in the unenqueued-list (as well as the corresponding trajectory and if the path was calculated to the end-point -> reverse_flag)
            min_index = None
            reverse_flag = False
            min_plan = pseudo_trajectory
            for index, view in enumerate(unenqueud_views):
                self.group.set_joint_value_target(view.get_trajectory_for_measurement().joint_trajectory.points[0].positions)
                
                plan_result = self.group.plan()
                if plan_result[0] and (min_plan.joint_trajectory.points[-1].time_from_start > plan_result[1].joint_trajectory.points[-1].time_from_start or min_index is None):
                    min_plan = plan_result[1]
                    min_index = index
                    reverse_flag = False
                
                self.group.set_joint_value_target(view.get_trajectory_for_measurement().joint_trajectory.points[-1].positions)
                
                plan_result = self.group.plan()
                if plan_result[0] and (min_plan.joint_trajectory.points[-1].time_from_start > plan_result[1].joint_trajectory.points[-1].time_from_start or min_index is None):
                    min_plan = plan_result[1]
                    min_index = index
                    reverse_flag = True
            if min_index is not None:
                # Shortest connection-plan found -> Enqueuing (and if necessary reverting)
                next_view = unenqueud_views.pop(min_index)
                if reverse_flag:
                    next_view.reverse_trajectory_for_measurement()   
                next_view.set_trajectory_to_view(min_plan)
                enqueud_views.append(next_view)
                # Reset planning time
                self.group.set_planning_time(min_planning_time)
                print("Enqueued viewpoint-trajectory ({} left)".format(len(unenqueud_views)))
            else:
                # No solution has been found -> Increase planning time to improve success-chances for the next iteration
                self.group.set_planning_time(min_planning_time + self.group.get_planning_time())
        return enqueud_views[1:]

            
        
    

    



    def execute(self, execution_list, surface_points=None):
        """
        Execute a list of RobotTrajectories or Views via MoveIt. When problems occur during execution,
        the robot will be stopped and an exception will be raised. When executing from a list of views, the currently measured surface_points
        are published in "/currently_measured_points" during execution.

        :param execution_list: List of RobotTrajectories or views that can be executed consecutively (the next segment's start point is the last segment's end point)
        :type execution_list: list[moveit_msgs/RobotTrajectory] or list[View]
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
        
        # Consecutively execute segments/Views with short breaks between every trajectory-execution to enure nothing is moving anymore
        for i, execution_segment in enumerate(execution_list):
            if isinstance(execution_segment, View):
                # Publish surface points, when given
                if surface_points is not None:
                    point_message.points = []
                    for point_index in view.get_measurable_surface_points(get_only_indices=True):
                        point_message.points.append(geometry_msgs.msg.Point(*(surface_points[point_index] / 1000)))
                    measurement_pub.publish(point_message)
                
                # Firstly, steer to the start point of the measurement-trajectory
                trajectory = view.get_trajectory_to_view()
                print("Steering to next measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
                stitch_service.call(SetBoolRequest(data=False))
                if not self.group.execute(trajectory):
                    self.group.stop()
                    raise Exception("Failed to steer to trajectory")
                rospy.sleep(0.1)

                # Secondly, execute the measurement-trajectory
                trajectory = view.get_trajectory_for_measurement()
                print("Executing measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
                stitch_service.call(SetBoolRequest(data=True))
                if not self.group.execute(trajectory):
                    self.group.stop()
                    raise Exception("Failed to execute trajectory")
                rospy.sleep(0.1)

                print("Executed view {} of {}".format(i + 1, len(execution_list)))
                
                
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

    def perform_all(self, target_mesh_filename, density, N_angles_per_view):
        self.load_target_mesh(target_mesh_filename, transform=trimesh.transformations.translation_matrix(OFFSET))
        surface_pts, views  = self.generate_samples_and_views(density, N_angles_per_view)
        scp_views = self.solve_scp(surface_pts, views, "IP_basic")
        connected_views = self.connect_views(scp_views)
        execution_plan = self.convert_viewlist_to_execution_plan(connected_views)
        self.store_execution_plan("/home/svenbecker/Bachelorarbeit/test/stored_plans/TEST1.yaml", execution_plan, {"Time of calculation": "Jetzt"})
        #execution_plan = self.load_execution_plan("/home/svenbecker/Bachelorarbeit/test/stored_plans/TEST1.yaml")
        self.execute(execution_plan)
        
if __name__ == "__main__":
    def signal_handler(signum_a, signum_b):
        exit()
    signal.signal(signal.SIGINT, signal_handler)
    tm = TrajectoryManager(OFFSET)
    while True:
        tm.perform_all("/home/svenbecker/Bachelorarbeit/test/motor_without_internals_upright.stl", 0.0005, 8)
        rospy.sleep(1)
    rospy.spin()
    