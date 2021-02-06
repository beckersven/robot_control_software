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


OFFSET = np.array([500, -600, 1300])

class TrajectoryManager:

    def load_execution_plan(self, file_path, adapt_to_current_start_pose = True):
        execution_plan = []
        with open(file_path, 'r') as plan_file:
            plan_list = yaml.safe_load(plan_file)
            assert isinstance(plan_list, list)
            assert isinstance(plan_list[0], dict)
            print("Metadata of loaded path:")
            for key, value in plan_list[0].items():
                print("{}:\t{}".format(key, value))
            for path_segment in plan_list[1:]:
                assert isinstance(path_segment, dict)
                execution_plan.append(message_converter.convert_dictionary_to_ros_message("moveit_msgs/RobotTrajectory", path_segment))
        if adapt_to_current_start_pose:
            self.group.set_start_state_to_current_state()
            self.group.set_joint_value_target(execution_plan[1].joint_trajectory.points[0].positions)
            plan_result = self.group.plan()
            if plan_result[0]:
                execution_plan[0] = plan_result[1]
            else:
                raise Exception("Could not connect execution plan to current start pose!")
        return execution_plan

    def store_execution_plan(self, file_path, execution_plan, metadata={}):
        assert isinstance(metadata, dict)
        assert isinstance(execution_plan, list)
        plan_to_store = [metadata]
        for path_segment in execution_plan:
            assert isinstance(path_segment, moveit_msgs.msg.RobotTrajectory)
            plan_to_store.append(message_converter.convert_ros_message_to_dictionary(path_segment))
        with open(file_path, 'w') as plan_file:
            yaml.safe_dump(plan_to_store, plan_file, default_flow_style=False)
        return
    
    def convert_viewlist_to_execution_plan(self, viewlist):
        assert isinstance(viewlist, list)
        execution_plan = []
        for view in viewlist:
            assert isinstance(view, View)
            execution_plan.append(view.get_trajectory_to_view())
            execution_plan.append(view.get_trajectory_for_measurement())
        return execution_plan



        with open(file_path, 'w') as plan_file:
            plan_list = yaml.safe_load(plan_file)
            assert isinstance(plan_list, list)
            for path_segment in plan_list:
                assert isinstance(path_segment, dict)
                stored_plan.append(moveit_msgs.msg.RobotTrajectory(**path_segment))
        return stored_plan


    def __init__(self, offset):
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
        
        self.target_mesh = trimesh.Trimesh()
        self.target_mesh_transform = np.eye(4)
        self.max_edge = 1e6

    def load_target_mesh(self, file_name, transform = np.eye(4), add_wbk_mirrors=True):
        
        
        # Load mesh via trimesh to class member for further processing
        try:
            self.target_mesh = trimesh.load_mesh(file_name)
        except:
            rospy.logerr("Trimesh could not load mesh {}".format(file_name))
            return False
        
        self.target_mesh_transform = transform
        


        # Move the mesh's origin (usually identically with the origin of the provided CAD-file)
        # to where its mounted with respect to the "world"-frame of MoveIt
        self.target_mesh.apply_transform(self.target_mesh_transform)
        
        #print(self.target_mesh.is_watertight)
        #self.target_mesh.update_faces([self.target_mesh.face_normals[i].dot([0, 0, -1]) < 0.9 for i in range(len(self.target_mesh.faces))])
        #self.target_mesh.show()

        # Update sensor model
        self.sensor_model.set_ray_mesh_intersector_for_mesh(self.target_mesh)

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
            # Add collision obejcts representing the mirrors beside the robot at IFL to MoveIt
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
        face_normals = np.ndarray((len(self.target_mesh.face_normals), 3), dtype=float)
        for i, face_normal in enumerate(self.target_mesh.face_normals):
            face_normals[i] = face_normal
        #ik_service = None
        
        for (surface_point, face_index) in zip(surface_points, face_indices):
            for angle in np.linspace(0, 360, N_angles_per_view + 1)[:-1]:
                new_view = View(surface_point, face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff())
                if view_tilt_mode == "none":
                    if self.process_view(new_view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure):
                        valid_views.append(new_view)
                elif view_tilt_mode == "conservative":
                    for angle_config in itertools.product([0, self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], repeat=2):
                        new_view = View(surface_point, face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                        if self.process_view(new_view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure):
                            valid_views.append(new_view)
                            break  
                elif view_tilt_mode == "aggressive":
                    for angle_config in itertools.product([0, self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], repeat=2):
                        new_view = View(surface_point, face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                        if self.process_view(new_view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure):
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

    def process_view(self, view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure_mm = 5, sample_step_mm = 2, joint_jump_threshold = 1.5, minimum_trajectory_length_mm = 50):
        assert isinstance(view, View)
        
        trajectory_origin = view.get_position()
        trajectory_direction = view.get_orientation_matrix().dot([1, 0, 0, 1])[0:3]

        # Implementation of a line equation (t is in mm, but result is in m)
        trajectory_in_m = lambda t: (trajectory_origin + trajectory_direction * t) / 1000
        
        # Check if center of viewpoint (= actual viewPOINT) is reachable 
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

        
        
        start = time.time()
        measurable_surface_points, uncertainty_scores, trajectory_line_arguments_selected = self.sensor_model.process_view_metrologically(surface_points, face_indices, view, face_normals, self.max_edge)
        

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
        view.set_measurable_surface_points(measurable_surface_points, uncertainty_scores)
        return True

    def postprocess_trajectory(self, trajectory, max_velocity_scale = 0.5, max_acceleration_scale = 1.0):
        params = rospy.get_param("/robot_description_planning/joint_limits")
        
        point_indices_to_pop = []
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
            if i < len(trajectory.joint_trajectory.points) - 1:
                if trajectory.joint_trajectory.points[i].time_from_start == trajectory.joint_trajectory.points[i + 1].time_from_start:
                    point_indices_to_pop.append(i)
        

        for point_index_to_pop in sorted(point_indices_to_pop, reverse=True):
            trajectory.joint_trajectory.points.pop(point_index_to_pop)

        


        return True

    def solve_scp(self, surface_points, provided_views, solver_type="greedy", solver_parameters=None):
        """Possible solver_types are: 
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

            """
        if solver_type == "greedy":
            measured_surface_point_indices = set()
            used_views = set()
            while measured_surface_point_indices != set(range(len(surface_points))) and len(provided_views) > 0:
                provided_views = sorted(provided_views, key = lambda view: len(measured_surface_point_indices | set(view.get_measurable_surface_points(get_only_indices=True))) + min(view.get_measurable_surface_points(get_only_scores=True)) * 0.9, reverse=True)
                next_view = provided_views.pop(0)
                # Only accept new view if it brings an advantage. If no advantage -> Algorithm has reached peak
                if measured_surface_point_indices == (measured_surface_point_indices | set(next_view.get_measurable_surface_points(get_only_indices=True))):
                    print("Covered only {} of {} surface points. Continuing with this solution ...".format(len(measured_surface_point_indices), len(surface_points)))
                    return used_views
                measured_surface_point_indices |= set(next_view.get_measurable_surface_points(get_only_indices=True))
                used_views.add(next_view)
            if measured_surface_point_indices != set(range(len(surface_points))):
                print("Covered only {} of {} surface points. Continuing with this solution...".format(len(measured_surface_point_indices), len(surface_points)))
            else:
                print("Covered all {} surface points".format(len(surface_points)))
            return used_views

        elif solver_type[0:3] == "IP_":            
            # Get indices of all measurable surface points in ascending order
            # (required so that integer programming can find any solution)
            measurable_surface_point_indices = set(
                itertools.chain.from_iterable(
                    [view.get_measurable_surface_points(get_only_indices=True) for view in provided_views]
                ))
            provided_views = list(provided_views)
            viewpoint_indices = range(len(provided_views))
            print("Provided viewpoints can cover {} of total {} sampled surface points".format(len(measurable_surface_point_indices), len(surface_points)))
            ip_problem = pulp.LpProblem(sense=pulp.const.LpMinimize)
            viewpoint_variables = pulp.LpVariable.dicts(name="viewpoint_variables", indexs=viewpoint_indices, cat=pulp.const.LpInteger)
            # Add objective function
            if solver_type == "IP_basic":
                ip_problem += pulp.lpSum([viewpoint_variables[i] for i in viewpoint_indices])
            elif solver_type == "IP_time":
                ip_problem += pulp.lpSum([viewpoint_variables[i] * provided_views[i].get_trajectory_for_measurement().joint_trajectory.points[-1].time_from_start.to_sec() for i in viewpoint_indices])
            elif solver_type == "IP_uncertainty":
                ip_problem += pulp.lpSum([viewpoint_variables[i] * (1 - sum(provided_views[i].get_measurable_surface_points(get_only_scores=True)) / len(provided_views[i].get_measurable_surface_points(get_only_scores=True))) for i in viewpoint_indices])
            elif solver_type == "IP_combined":
                ip_problem += pulp.lpSum([viewpoint_variables[i] * (1 - sum(provided_views[i].get_measurable_surface_points(get_only_scores=True)) / len(provided_views[i].get_measurable_surface_points(get_only_scores=True))) * provided_views[i].get_trajectory_for_measurement().joint_trajectory.points[-1].time_from_start.to_sec() for i in viewpoint_indices])
            # Add constraints: Viewpoint_variable's elements are either 1 or 0
            for i in viewpoint_indices:
                ip_problem += viewpoint_variables[i] <= 1
                ip_problem += viewpoint_variables[i] >= 0
            
            # Add constraints: Every surface point must be covered
            for surface_point_index in measurable_surface_point_indices:
                ip_problem += pulp.lpSum([(surface_point_index in provided_views[viewpoint_index].get_measurable_surface_points(get_only_indices=True)) * viewpoint_variables[viewpoint_index] for viewpoint_index in viewpoint_indices]) >= 1
            print("Constructed integer programming problem. Start solving...")
            ip_problem.solve()
            if not ip_problem.status == pulp.const.LpSolutionOptimal:
                raise Exception("Could not find optimal solution.")
            print("Found optimal solution consisting of {} viewpoints".format(pulp.value(ip_problem.objective)))
            output = set()
            for solved_viewpoint_variable in ip_problem.variables():
                if solved_viewpoint_variable.varValue == 1:  
                    output.add(provided_views[int(solved_viewpoint_variable.name.split("_")[-1])])
            return output           

    def connect_views(self, unordered_views, return_to_initial_pose=False, min_planning_time=0.2):
        if len(unordered_views) == 0:
            return
        left_views = list(unordered_views)
        
        ordered_views = []
        
        # Integrate initial robot pose as pseudo "view"
        # (it has a trajectory with the current pose as its only point)
        initial_pseudo_view = View(np.array([1,0,0]), np.array([1,0,0]), 0, 0)
        pseudo_trajectory = moveit_msgs.msg.RobotTrajectory()
        pseudo_end_point = trajectory_msgs.msg.JointTrajectoryPoint()
        pseudo_end_point.positions = self.group.get_current_joint_values()
        pseudo_end_point.time_from_start = rospy.Duration(1e9)
        pseudo_trajectory.joint_trajectory.joint_names = left_views[0].get_trajectory_for_measurement().joint_trajectory.joint_names
        pseudo_trajectory.joint_trajectory.points.append(pseudo_end_point)
        initial_pseudo_view.set_trajectory_for_measurement(pseudo_trajectory)       
        ordered_views.append(initial_pseudo_view)

        while len(left_views) != 0:
            # Set start state for planning to the last pose of the previous trajectory to execute
            
            temporary_start_state = moveit_msgs.msg.RobotState()
            temporary_start_state.joint_state.position = ordered_views[-1].get_trajectory_for_measurement().joint_trajectory.points[-1].positions
            temporary_start_state.joint_state.name = ordered_views[-1].get_trajectory_for_measurement().joint_trajectory.joint_names
            self.group.set_start_state(temporary_start_state)
            
            min_index = None
            reverse_flag = False
            min_plan = pseudo_trajectory
            for index, view in enumerate(left_views):
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
                next_view = left_views.pop(min_index)
                print("Enqueued viewpoint-trajectory ({} left)".format(len(left_views)))
                if reverse_flag:
                    next_view.reverse_trajectory_for_measurement()   
                next_view.set_trajectory_to_view(min_plan)
                ordered_views.append(next_view)
                self.group.set_planning_time(min_planning_time)
            else:
                self.group.set_planning_time(min_planning_time + self.group.get_planning_time())
        return ordered_views[1:]

            
        
    

    



    def execute(self, execution_list, surface_points=None):
        
        
        if surface_points is not None:
            measurement_pub = rospy.Publisher("/currently_measured_points", sensor_msgs.msg.PointCloud, queue_size=1, latch=True)
            point_message = sensor_msgs.msg.PointCloud()
            point_message.header.frame_id = "world"
        rospy.wait_for_service("/switch_stitch_mode", 5.0)
        stitch_service = rospy.ServiceProxy("/switch_stitch_mode", SetBool)
        
        trajectory_flag = False
        for i, execution_segment in enumerate(execution_list):
            if isinstance(execution_segment, View):
                if surface_points is not None:
                    point_message.points = []
                    for point_index in view.get_measurable_surface_points(get_only_indices=True):
                        point_message.points.append(geometry_msgs.msg.Point(*(surface_points[point_index] / 1000)))
                    measurement_pub.publish(point_message)
                trajectory = view.get_trajectory_to_view()
                print("Steering to next measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
                stitch_service.call(SetBoolRequest(data=False))
                if not self.group.execute(trajectory):
                    self.group.stop()
                    raise Exception("Failed to steer to trajectory")
                rospy.sleep(0.1)
                trajectory = view.get_trajectory_for_measurement()
                # Short break to ensure no parts are moving anymore
                print("Executing measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
                stitch_service.call(SetBoolRequest(data=True))
                if not self.group.execute(trajectory):
                    self.group.stop()
                    raise Exception("Failed to execute trajectory")
                print("Executed view {} of {}".format(i + 1, len(execution_list)))
                # Short break to ensure no parts are moving anymore
                rospy.sleep(0.1)  
                stitch_service.call(SetBoolRequest(data=False))
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
                # Short break to ensure no parts are moving anymore
                rospy.sleep(0.1)
                trajectory_flag = not trajectory_flag     
            

    def perform_all(self, target_mesh_filename, density, N_angles_per_view):
        """self.load_target_mesh(target_mesh_filename, transform=trimesh.transformations.translation_matrix(OFFSET))
        surface_pts, views  = self.generate_samples_and_views(density, N_angles_per_view)
        scp_views = self.solve_scp(surface_pts, views, "IP_basic")
        connected_views = self.connect_views(scp_views)
        execution_plan = self.convert_viewlist_to_execution_plan(connected_views)
        self.store_execution_plan("/home/svenbecker/Bachelorarbeit/test/stored_plans/TEST1.yaml", execution_plan, {"Time of calculation": "Jetzt"})"""
        execution_plan = self.load_execution_plan("/home/svenbecker/Bachelorarbeit/test/stored_plans/TEST1.yaml")
        self.execute(execution_plan)
        

    
    





        
if __name__ == "__main__":
    def signal_handler(signum_a, signum_b):
        exit()
    signal.signal(signal.SIGINT, signal_handler)
    tm = TrajectoryManager(OFFSET)
    while True:
        tm.perform_all("/home/svenbecker/Bachelorarbeit/test/motor_without_internals_upright.stl", 0.01, 8)
        rospy.sleep(1)
    rospy.spin()
    