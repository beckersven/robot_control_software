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

OFFSET = np.array([600, -600, 1700])

class TrajectoryManager:
    def __init__(self, offset):
        # Initialization
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("trajectory_manager")
        
        # Configuring MoveIt
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_planner_id("PersistentPRMstar") # Enables multi-query-planning (see corresponding literature)
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.group.set_max_velocity_scaling_factor(1)
        
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

        # Create a sensor-model instance
        self.sensor_model = SensorModel()
        
        self.target_mesh = trimesh.Trimesh()
        self.target_mesh_transform = np.eye(4)
        self.max_edge = 1e6

    def postprocess_trajectory(self, trajectory):
        params = rospy.get_param("/robot_description_planning/joint_limits")
        point_indices_to_pop = []
        for i in range(len(trajectory.joint_trajectory.points)):
            for j, position in enumerate(trajectory.joint_trajectory.points[i].positions):
                if params[trajectory.joint_trajectory.joint_names[j]].has_key("max_position"):
                    diff = position - params[trajectory.joint_trajectory.joint_names[j]]["max_position"]
                    if diff >= 1e-3:
                        return False
                    elif diff > 0 and diff < 1e-3:
                        position = params[trajectory.joint_trajectory.joint_names[j]]["max_position"]
                if params[trajectory.joint_trajectory.joint_names[j]].has_key("min_position"):
                    diff = position - params[trajectory.joint_trajectory.joint_names[j]]["min_position"]
                    if diff <= -1e-3:
                        return False
                    elif diff < 0 and diff > -1e-3:
                        position = params[trajectory.joint_trajectory.joint_names[j]]["min_position"]
            if i < len(trajectory.joint_trajectory.points) - 1:
                if trajectory.joint_trajectory.points[i].time_from_start == trajectory.joint_trajectory.points[i + 1].time_from_start:
                    point_indices_to_pop.append(i)
        
        for point_index_to_pop in sorted(point_indices_to_pop, reverse=True):
            trajectory.joint_trajectory.points.pop(point_index_to_pop)

        return True
                 

    def connect_views(self, unordered_views, return_to_initial_pose=False):
        if len(unordered_views) == 0:
            return
        left_views = list(unordered_views)
        self.group.set_planning_time(0.2)
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
        return ordered_views[1:]

            
        
    def process_view(self, view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure_mm = 10, sample_step_mm = 1, joint_jump_threshold = 1.5, minimum_trajectory_length_mm = 50):
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
        start = time.time()
        req = GetPositionIKRequest()
        req.ik_request.group_name = "manipulator"
        req.ik_request.avoid_collisions = True
        # req.ik_request.robot_state = self.group.get_current_state()
        req.ik_request.pose_stamped = view_center_pose
        
        for _ in range(10):
            resp = ik_service.call(req)
            if resp.error_code.val == GetPositionIKResponse().error_code.SUCCESS:
                break
        if not resp.error_code.val == GetPositionIKResponse().error_code.SUCCESS:
            print("Rejected viewpoint because no IK solution was found")
            return False
        self.group.set_start_state(resp.solution)
        print("IK", time.time() - start)
        start = time.time()
        

        measurable_surface_points, uncertainty_scores, trajectory_line_arguments_selected = self.sensor_model.process_view_metrologically(surface_points, face_indices, self.target_mesh, view, face_normals, self.max_edge)
        print("EVAL", time.time() - start)
        start = time.time()

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
        print("TRAJ", time.time() - start)
        return True

    def load_target_mesh(self, file_name, transform = np.eye(4)):
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
        
        return True



    def execute_views(self, ordered_views, surface_points=None, try_to_handle_errors=True):
        view_counter = 0
        trajectory = moveit_msgs.msg.RobotTrajectory()
        rospy.wait_for_service("/switch_stitch_mode", 5.0)
        stitch_service = rospy.ServiceProxy("/switch_stitch_mode", SetBool)
        if surface_points is not None:
            measurement_pub = rospy.Publisher("/currently_measured_points", sensor_msgs.msg.PointCloud, queue_size=1, latch=True)
            point_message = sensor_msgs.msg.PointCloud()
            point_message.header.frame_id = "world"

        for view in ordered_views:
            trajectory = view.get_trajectory_to_view()
            print("Steering to next measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
            stitch_service.call(SetBoolRequest(data=False))
            self.group.execute(trajectory)

            self.group.stop()
            
            trajectory = view.get_trajectory_for_measurement()
            # Short break to ensure no parts are moving anymore
            rospy.sleep(0.1)
            if surface_points is not None:
                point_message.points = []
                for point_index in view.get_measurable_surface_points(get_only_indices=True):
                    point_message.points.append(geometry_msgs.msg.Point(*(surface_points[point_index] / 1000)))
                measurement_pub.publish(point_message)
            print("Executing measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
            stitch_service.call(SetBoolRequest(data=True))
            self.group.execute(trajectory)
            view_counter += 1
            print("Executed view {} of {}".format(view_counter, len(ordered_views)))
            # Short break to ensure no parts are moving anymore
            rospy.sleep(0.1)  
            stitch_service.call(SetBoolRequest(data=False))
            
            

    def perform_all(self, target_mesh_filename, density, N_angles_per_view):
        self.load_target_mesh(target_mesh_filename, transform=trimesh.transformations.translation_matrix(OFFSET))
        surface_pts, views  = self.generate_samples_and_views(density, N_angles_per_view)
        scp_views = self.solve_scp(surface_pts, views, "IP_basic")

        raw_input()
        self.execute_views(self.connect_views(scp_views), surface_pts)

    def generate_samples_and_views(self, density, N_angles_per_view, view_tilt_mode="conservative", visibility_to_mechanical_viewpoint_ratio = 1):
        
        surface_points, face_indices = trimesh.sample.sample_surface_even(self.target_mesh, int(self.target_mesh.area * density))
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
        minimum_required_overmeasure = np.sqrt(1 / (np.pi * density)) + 5
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
                    if self.process_view(new_view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure):
                        valid_views.append(new_view)
                    else:
                        for angle_config in itertools.permutations([self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], 2):
                            new_view = View(surface_point, face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                            if self.process_view(new_view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure):
                                valid_views.append(new_view)
                                break  
                elif view_tilt_mode == "aggressive":
                    if self.process_view(new_view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure):
                        valid_views.append(new_view)
                    for angle_config in itertools.permutations([self.sensor_model.get_median_deviation_angle(), -1 * self.sensor_model.get_median_deviation_angle()], 2):
                        new_view = View(surface_point, face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_optimal_standoff(), angle_config[0], angle_config[1])
                        if self.process_view(new_view, surface_points, face_indices, ik_service, face_normals, minimum_required_overmeasure):
                            valid_views.append(new_view)
                
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
            -   "IP_time_plus": Solves the SCP using IP with respect to the total estimated execution time. The time is approximated incorporating
                the known trajectory-durations from the views as well as calculated transition-times from one trajectory's endpoint to the
                next trajectory's startpoint based on the joint-angle-differences and the maximum joint-velocities provided by the parameter server.
                Keep in mind that this algorithm, for performance reasons, does not perform collision-checks, etc. so the transition-times may deviate
                from real world applications considerably.
            """
        if solver_type == "greedy":
            measured_surface_point_indices = set()
            used_views = set()
            while measured_surface_point_indices != set(range(len(surface_points))) and len(provided_views) > 0:
                provided_views = sorted(provided_views, key = lambda view: len(measured_surface_point_indices | set(view.get_measurable_surface_points(get_only_indices=True))) + min(view.get_measurable_surface_points(get_only_scores=True)) * 0.9, reverse=True)
                next_view = provided_views.pop(0)
                # Only accept new view if it brings an advantage. If no advantage -> Algorithm has reached peak
                if measured_surface_point_indices == (measured_surface_point_indices | set(next_view.get_measurable_surface_points())):
                    print("Covered only {} of {} surface points. Continuing with this solution ...".format(len(measured_surface_point_indices), len(surface_points)))
                    return used_views
                measured_surface_point_indices |= set(next_view.get_measurable_surface_points(get_only_indices=True))
                used_views.add(next_view)
            if measured_surface_point_indices != set(range(len(surface_points))):
                print("Covered only {} of {} surface points. Continuing with this solution...".format(len(measured_surface_point_indices), len(surface_points)))
            else:
                print("Covered all {} surface points".format(len(surface_points)))
            return used_views

        elif solver_type == "IP_basic":            
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
            ip_problem += pulp.lpSum([viewpoint_variables[i] for i in viewpoint_indices])
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


            

class SensorModel:
    def __init__(self):
        # Get sensor specifications from parameter server
        parameters = rospy.get_param("/sensor_model_parameters")
        self.optimal_standoff = parameters["optimal_standoff_mm"]
        self.maximum_deviation_z = parameters["maximum_deviations"]["z_mm"]
        self.maximum_deviation_theta = parameters["maximum_deviations"]["theta_rad"]
        self.maximum_deviation_gamma = parameters["maximum_deviations"]["gamma_rad"]
        self.beam_taille_diameter = parameters["beam_taille_diameter_mm"]
        self.alpha = parameters["alpha_rad"]
        self.fan_angle = parameters["fan_angle_rad"]

        # Calculate useful sensor features from the specifics (so that they must be calculated every time)
        self.u_0 = np.sin(self.alpha) * self.optimal_standoff
        self.laser_emission_perception_distance = np.tan(self.alpha) * self.optimal_standoff
        self.m_L = np.tan(self.alpha)

        # Might be useful later
        self.scanner_body = trimesh.PointCloud(np.array([
            [0.05, 0.085, 0.0],
            [0.05, -0.085, 0.0],
            [-0.315, -0.085, 0.0],
            [-0.315, 0.085, 0.0],
            
            [0.05, 0.085, -0.19],
            [0.05, -0.085, -0.19],
            [-0.315, -0.085, -0.19],
            [-0.315, 0.085, -0.19],
        ]) * 1000).convex_hull

        self.ray_mesh_intersector = None
    
    def get_median_deviation_angle(self):
        return (self.maximum_deviation_theta + self.maximum_deviation_gamma) / 4



    def get_max_uncertainty(self):
        return self.evaluate_uncertainty(self.optimal_standoff + self.maximum_deviation_z, self.maximum_deviation_gamma + np.pi)
    
    def get_min_uncertainty(self):
        return self.evaluate_uncertainty(self.optimal_standoff - self.maximum_deviation_z, np.pi)

    def evaluate_uncertainty(self, z, gamma):
        beta = np.arcsin((self.u_0 * np.sin(np.pi/2 - self.alpha)) / np.sqrt((z + np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2))**2+self.u_0**2-2*(z + np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2))*self.u_0*np.cos(np.pi/2 - self.alpha)))
        d_z = np.sin(beta+gamma) / (np.sin(beta) * np.sin(gamma)) * (z + np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2)) * self.beam_taille_diameter / np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2)
        return  d_z / np.cos(gamma + beta - np.pi/2) 

    def get_scanner_body(self):
        return self.scanner_body.copy()

    def get_optimal_standoff(self):
        return self.optimal_standoff

    def process_view_metrologically(self, surface_points, faces, target_mesh, view, face_normals, frustum_half_length_mm = 5e2):
        start = time.time()
        assert isinstance(view, View)
        if self.ray_mesh_intersector is None:
            self.ray_mesh_intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(target_mesh, scale_to_box = False)

        scan_frustum = self.get_scanning_frustum(frustum_half_length_mm)
        scan_frustum.apply_transform(view.get_orientation_matrix())
        scan_frustum.apply_transform(view.get_projected_position(True))
        content_evaluator = trimesh.ray.ray_pyembree.RayMeshIntersector(scan_frustum, scale_to_box = False)
        inliers = content_evaluator.contains_points(surface_points)
        inlier_indices = array.array('i')
        for i in range(len(inliers)):
            if inliers[i]:
                inlier_indices.append(i)

        trajectory_direction = view.get_orientation_matrix().dot(np.array([1, 0, 0, 1]))[0:3]
        sensor_direction = view.get_orientation_matrix().dot(np.array([0, 0, 1, 1]))[0:3]
        lateral_direction = view.get_orientation_matrix().dot(np.array([0, 1, 0, 1]))[0:3]
        score_calc = lambda z, gamma : (self.get_max_uncertainty() - self.evaluate_uncertainty(z, gamma)) / (self.get_max_uncertainty() - self.get_min_uncertainty())
        ray_origins_emitter = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_directions_emitter = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_origins_detector = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_directions_detector = np.ndarray((len(inlier_indices), 3), dtype=float)
        trajectory_line_arguments = array.array('d', [])
        surface_points = np.array(surface_points)
        for i, inlier_index in enumerate(inlier_indices):
            t_0 =  (surface_points[inlier_index] - view.get_position()).dot(trajectory_direction)
            trajectory_line_arguments.append(t_0)
            
            ray_origins_emitter[i] = t_0 * trajectory_direction + view.get_position()
            ray_directions_emitter[i] = surface_points[inlier_index] - ray_origins_emitter[i]
            
            ray_origins_detector[i] = ray_origins_emitter[i] - trajectory_direction * self.laser_emission_perception_distance
            ray_directions_detector[i] = surface_points[inlier_index] - ray_origins_detector[i]
        rt_emitter_result = self.ray_mesh_intersector.intersects_first(
            ray_origins=ray_origins_emitter,
            ray_directions=ray_directions_emitter
            )
        rt_detector_result = self.ray_mesh_intersector.intersects_first(
            ray_origins=ray_origins_detector,
            ray_directions=ray_directions_detector
            )

        measurable_surface_point_indices = array.array('i', [])
        uncertainty_scores = array.array('d', [])
        trajectory_line_arguments_selected = array.array('d', [])
        for i, (rt_emitter_face_id, rt_detector_face_id) in enumerate(zip(rt_emitter_result, rt_detector_result)):
            if rt_emitter_face_id != rt_detector_face_id or faces[inlier_indices[i]] != rt_detector_face_id:
                continue
            else:
                gamma = np.pi / 2 + np.arctan2(trajectory_direction.dot(face_normals[rt_detector_face_id]), -1 * sensor_direction.dot(face_normals[rt_detector_face_id]))
                theta = np.arctan2(lateral_direction.dot(face_normals[rt_detector_face_id]), -1 * sensor_direction.dot(face_normals[rt_detector_face_id]))
                z = sensor_direction.dot(surface_points[inlier_indices[i]] - ray_origins_emitter[i])
                if gamma > np.pi / 4 and gamma < np.pi * 3 / 4 and abs(theta) < np.pi / 4 and z < 200 and z > 100:
                    uncertainty_scores.append(score_calc(z, gamma))
                    measurable_surface_point_indices.append(inlier_indices[i])
                    trajectory_line_arguments_selected.append(trajectory_line_arguments[i])
        return measurable_surface_point_indices, uncertainty_scores, trajectory_line_arguments_selected
        

    def get_scanning_frustum(self, half_length):
        scanner_frustum_vertices = [
            [half_length, (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [- half_length, (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [- half_length, - (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [half_length, - (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [half_length, (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
            [- half_length, (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
            [- half_length, - (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
            [half_length, - (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
        ]
        
        return trimesh.PointCloud(scanner_frustum_vertices).convex_hull
    


class View:
    def __init__(self, surface_point, surface_normal, angle, standoff_distance, tilt_theta = 0, tilt_gamma = 0):
        self.projected_position = surface_point
        #self.position = surface_point + surface_normal * standoff_distance
        if not all(np.cross([1,0,0], -surface_normal) == np.array([0,0,0])):
            z_axis = -surface_normal
            y_axis = np.cross([1,0,0], z_axis) / np.linalg.norm(np.cross([1,0,0], z_axis))
            x_axis = np.cross(y_axis, z_axis)
        else:
            z_axis = -surface_normal
            y_axis = np.cross([0,0,1], z_axis) / np.linalg.norm(np.cross([0,0,1], z_axis))
            x_axis = np.cross(y_axis, z_axis)
        self.orientation_matrix = np.array([
                [x_axis[0], y_axis[0], z_axis[0], 0],
                [x_axis[1], y_axis[1], z_axis[1], 0],
                [x_axis[2], y_axis[2], z_axis[2], 0],
                [0,         0,         0,         1]
            ]).dot(np.array([
                [np.cos(angle),   -np.sin(angle),    0,  0],
                [np.sin(angle),   np.cos(angle),     0,  0],
                [0,                 0,               1,  0],
                [0,                 0,               0,  1]
            ]))
        self.orientation_matrix = self.orientation_matrix.dot(trimesh.transformations.rotation_matrix(tilt_theta, np.array([1, 0, 0])))
        self.orientation_matrix = self.orientation_matrix.dot(trimesh.transformations.rotation_matrix(tilt_gamma, np.array([0, 1, 0])))
        self.position = standoff_distance * self.orientation_matrix.dot([0, 0, -1, 1])[:3] + surface_point


        self.lengths = [0,0]
        self.measurable_surface_point_indices = []
        self.measurable_surface_point_scores = []
        
        self.trajectory_for_measurement = moveit_msgs.msg.RobotTrajectory()
        self.trajectory_to_view = moveit_msgs.msg.RobotTrajectory()
    
    def get_trajectory_to_view(self):
        return self.trajectory_to_view

    def set_trajectory_to_view(self, trajectory_to_view):
        assert isinstance(trajectory_to_view, moveit_msgs.msg.RobotTrajectory)
        self.trajectory_to_view = trajectory_to_view

    def reverse_trajectory_for_measurement(self):
        reversed_trajectory = moveit_msgs.msg.RobotTrajectory()
        reversed_trajectory.joint_trajectory.header = self.trajectory_for_measurement.joint_trajectory.header
        reversed_trajectory.joint_trajectory.joint_names = self.trajectory_for_measurement.joint_trajectory.joint_names
        # Make deepcopy to prevent changing the original measurement-trajectory
        temp = self.trajectory_for_measurement.joint_trajectory.points[:]
        temp.reverse()
        total_duration = self.trajectory_for_measurement.joint_trajectory.points[-1].time_from_start
        for index, point in enumerate(temp):
            new_point = trajectory_msgs.msg.JointTrajectoryPoint()
            new_point.positions = point.positions
            new_point.accelerations = [-1 * acc for acc in point.accelerations]
            new_point.velocities = [-1 * vel for vel in point.velocities]
            new_point.time_from_start = total_duration - self.trajectory_for_measurement.joint_trajectory.points[-(index + 1)].time_from_start
            reversed_trajectory.joint_trajectory.points.append(new_point)
            
        self.set_trajectory_for_measurement(reversed_trajectory)
        return True
            

    def get_projected_position(self, as_matrix=False):
        if as_matrix:
            return trimesh.transformations.translation_matrix(self.projected_position)
        else:
            return self.projected_position


    def set_trajectory_for_measurement(self, trajectory_for_measurement):
        assert isinstance(trajectory_for_measurement, moveit_msgs.msg.RobotTrajectory)
        self.trajectory_for_measurement = trajectory_for_measurement

    def get_trajectory_for_measurement(self):
        return self.trajectory_for_measurement

    def get_orientation_matrix(self):
        return self.orientation_matrix

    def get_position(self, as_matrix = False):
        if as_matrix:
            return trimesh.transformations.translation_matrix(self.position)
        else:
            return self.position
    
    def set_measurable_surface_points(self, measurable_point_indices, uncertainty_scores):

        self.measurable_surface_point_indices = measurable_point_indices
        self.measurable_surface_point_scores = uncertainty_scores
    
    def get_measurable_surface_points(self, get_only_indices=False, get_only_scores=False):
        if get_only_indices and not get_only_scores:
            return self.measurable_surface_point_indices
        elif not get_only_indices and get_only_scores:
            return self.measurable_surface_point_scores
        elif not get_only_scores and not get_only_indices:
            return list(zip(self.measurable_surface_point_indices, self.measurable_surface_point_scores))
        else:
            raise ValueError("Only one argument may be true or both arguments false, but both can not be set true!")
    
    def set_lengths(self, lenght_pos_x, lenght_neg_x):
        self.lengths = [lenght_pos_x, lenght_neg_x]
    
    def get_lengths(self):
        return self.lengths

if __name__ == "__main__":
    def signal_handler(signum_a, signum_b):
        exit()
    signal.signal(signal.SIGINT, signal_handler)
    tm = TrajectoryManager(OFFSET)
    while True:
        tm.perform_all("/home/svenbecker/Desktop/test6.stl", 0.001, 6)
        rospy.sleep(5)
    rospy.spin()
    