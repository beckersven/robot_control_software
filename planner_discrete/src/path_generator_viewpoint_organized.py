import trimesh
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import signal
from geometry_msgs.msg import Quaternion
import trajectory_msgs.msg
import time
import sensor_msgs.msg


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

    def postprocess_trajectory(self, trajectory, start_state):
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
            return []
        left_views = list(unordered_views)
        ordered_views = []
        self.group.set_planning_time(0.2)


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
                print("Enqueued viewpoint-trajectory ({} left)".format(len(left_views)))
                next_view = left_views.pop(min_index)
                if reverse_flag:
                    next_view.reverse_trajectory_for_measurement()
                    
                next_view.set_trajectory_to_view(min_plan)
                ordered_views.append(next_view)

        return ordered_views[1:]

            
        
    def process_view(self, view, surface_points, sample_step_mm = 1, joint_jump_threshold = 1.5, minimum_trajectory_length_mm = 50):
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
        self.group.set_start_state(view_center_state)
        
        

        measurement_evaluation = self.sensor_model.process_view_metrologically(surface_points, self.target_mesh, view, self.max_edge)
        if len(measurement_evaluation) == 0:
            return False
        max_deflection = max(measurement_evaluation, key=lambda x: x[1])[1] + 10
        min_deflection = min(measurement_evaluation, key=lambda x: x[1])[1] - 10
        if max_deflection - min_deflection < minimum_trajectory_length_mm:
            print(max_deflection, min_deflection)
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
        if (fraction_a * max_deflection - fraction_b * min_deflection) < minimum_trajectory_length_mm or not self.postprocess_trajectory(trajectory, view_deflection_state_a):
            return False
        view.set_trajectory_for_measurement(trajectory)
        view.set_covered_surface_points([measurable_point[0] for measurable_point in list(filter(lambda x: (x[1] <= fraction_a * max_deflection and x[1] >= fraction_b * min_deflection), measurement_evaluation))])

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
        except:
            # add_mesh hat troubles with "non-root" filenames (e.g. ~/cadfiles/target.stl)
            rospy.logerr("Moveit could not load mesh {}.\nHave you tried specifying it with a file path starting at root /...?".format(file_name))
            return False
        return True



    def execute_views(self, ordered_views, surface_points=None, try_to_handle_errors=True):
        view_counter = 0
        trajectory = moveit_msgs.msg.RobotTrajectory()

        if surface_points is not None:
            measurement_pub = rospy.Publisher("/currently_measured_points", sensor_msgs.msg.PointCloud, queue_size=1, latch=True)
            point_message = sensor_msgs.msg.PointCloud()
            point_message.header.frame_id = "world"

        for view in ordered_views:
            trajectory = view.get_trajectory_to_view()
            print("Steering to next measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))
            
            self.group.execute(trajectory)

            self.group.stop()
            trajectory = view.get_trajectory_for_measurement()
            # Short break to ensure no parts are moving anymore
            rospy.sleep(0.1)
            if surface_points is not None:
                point_message.points = []
                for point_index in view.get_covered_surface_points():
                    point_message.points.append(geometry_msgs.msg.Point(*(surface_points[point_index] / 1000)))
                measurement_pub.publish(point_message)
                raw_input()
            print("Executing measurement-trajectory (time: {}s)...".format(trajectory.joint_trajectory.points[-1].time_from_start.to_sec()))

            self.group.execute(trajectory)
            
            self.group.stop()
            
            view_counter += 1
            print("Executed view {} of {}".format(view_counter, len(ordered_views)))
            # Short break to ensure no parts are moving anymore
            rospy.sleep(0.1)  

            # if not self.group.execute(trajectory) and not try_to_handle_errors:
            #     rospy.logerr("Execution of trajectory failed. Exiting...")
            #     self.group.stop()
            #     return False
            # elif not self.group.execute(trajectory) and try_to_handle_errors:
            #     rospy.logwarn("Execution of trajectory failed. Looking for workaround...")
            #     self.group.stop()
            #     self.group.set_start_state_to_current_state()
            #     self.group.set_joint_value_target(trajectory.joint_trajectory.points[-1].positions)
            #     planner_result = self.group.plan()
            #     if planner_result[0]:
            #         print("Found workaround to reach end of current trajectory...")
            #         if not self.group.execute(planner_result[1]):
            #             print("Execution of workaround-attempt failed. Exiting...")
            #             return False
            #         else:
            #             print("Workaround worked. Continuing with measurement plan...")
            #     else:
            #         print("Failed to find workaround. Exiting...")
            #         return False
            

    def perform_all(self, target_mesh_filename, density, N_angles_per_view):
        self.load_target_mesh(target_mesh_filename, transform=trimesh.transformations.translation_matrix(OFFSET))
        surface_pts, views  = self.generate_samples_and_views(density, N_angles_per_view)
        scp_views = self.solve_scp(surface_pts, views)
        views = self.connect_views(scp_views)
        self.execute_views(views, surface_pts)

    def generate_samples_and_views(self, density, N_angles_per_view):
        surface_points, face_indices = trimesh.sample.sample_surface_even(self.target_mesh, int(self.target_mesh.area * density))
        pub = rospy.Publisher("/sampled_surface_points", sensor_msgs.msg.PointCloud, latch=True, queue_size=1)
        surface_points_message = sensor_msgs.msg.PointCloud()
        surface_points_message.header.frame_id = "world"
        for surface_point in surface_points:
            surface_points_message.points.append(geometry_msgs.msg.Point(*(surface_point / 1000)))
        pub.publish(surface_points_message)
        self.max_edge = max(self.target_mesh.bounding_box_oriented.edges_unique_length)
        self.group.set_planning_time(0.05)
        valid_views = []
        processed = 0
        for (surface_point, face_index) in zip(surface_points, face_indices):
            for angle in np.linspace(0, 360, N_angles_per_view + 1)[:-1]:
                new_view = View(surface_point, self.target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_OPTIMAL_STANDOFF())
                if self.process_view(new_view, surface_points):
                    valid_views.append(new_view)
                processed += 1
                print("Processed view {} of {} ({} %, {} of them are usable for measurement)".format(
                    processed, 
                    len(surface_points) * N_angles_per_view, 
                    np.round(100.0 * processed / (len(surface_points) * N_angles_per_view), 2),
                    len(valid_views)
                ))

        print("\n" + "*" * 20 + "\nGenerated {} valid measurement-trajectory/-ies".format(len(valid_views)))
        return surface_points, valid_views
    


    def solve_scp(self, surface_points, provided_views):
        measured_surface_point_indices = set()
        used_views = set()
        while measured_surface_point_indices != set(range(len(surface_points))) and len(provided_views) > 0:
            provided_views = sorted(provided_views, key = lambda view: len(measured_surface_point_indices | set(view.get_covered_surface_points())), reverse=True)
            next_view = provided_views.pop(0)
            # Only accept new view if it brings an advantage. If no advantage -> Algorithm has reached peak
            if measured_surface_point_indices == (measured_surface_point_indices | set(next_view.get_covered_surface_points())):
                print("Covered only {} of {} surface points. Continuing with this solution ...".format(len(measured_surface_point_indices), len(surface_points)))
                return used_views
            measured_surface_point_indices |= set(next_view.get_covered_surface_points())
            used_views.add(next_view)
        if measured_surface_point_indices != range(len(surface_points)):
            print("Covered all {} surface points".format(len(surface_points)))
        else:
            print("Covered only {} of {} surface points. Continuing with this solution...".format(len(measured_surface_point_indices), len(surface_points)))
        return used_views
            

class SensorModel:
    def __init__(self):
        self.OPTIMAL_STANDOFF = 150
        self.DELTA_Z = 50
        self.FAN_ANGLE = np.deg2rad(20)
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


    def get_scanner_body(self):
        return self.scanner_body.copy()

    def get_OPTIMAL_STANDOFF(self):
        return self.OPTIMAL_STANDOFF

    def process_view_metrologically(self, surface_points, target_mesh, view, frustum_half_length_mm = 5e2):
        assert isinstance(view, View)
        scan_frustum = self.get_scanning_frustum(frustum_half_length_mm)
        scan_frustum.apply_transform(view.get_orientation_matrix())
        scan_frustum.apply_transform(view.get_projected_position(True))
        inliers = scan_frustum.contains(surface_points)
        inlier_indices = []

        for i in range(len(inliers)):
            if inliers[i]:
                inlier_indices.append(i)
        trajectory_direction = view.get_orientation_matrix().dot(np.array([1, 0, 0, 1]))[0:3]
        measurable_surface_points = []
        for inlier_index in inlier_indices:
            t_0 = (surface_points[inlier_index] - view.get_position()).dot(trajectory_direction)
            ray_origin = t_0 * trajectory_direction + view.get_position()
            ray_direction = surface_points[inlier_index] - (t_0 * trajectory_direction + view.get_position())
            locations, _, faces = target_mesh.ray.intersects_location(
                    ray_origins=[ray_origin],
                    ray_directions=[ray_direction]
                )
            valid_dimension = list(filter(lambda x:abs(ray_direction[x])>1e-6, range(3)))[0]
            add_inlier = True
            for hit_index in range(len(locations)):
                hit_ray_argument = (locations[hit_index][valid_dimension] - ray_origin[valid_dimension]) / ray_direction[valid_dimension]
                if hit_ray_argument < 0.999 and hit_ray_argument > 0:
                    add_inlier = False
                    break  
            if add_inlier:
                measurable_surface_points.append((inlier_index, t_0, None))
        return measurable_surface_points
        

    def get_scanning_frustum(self, half_length):
        scanner_frustum_vertices = [
            [half_length, (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [- half_length, (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [- half_length, - (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [half_length, - (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [half_length, (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
            [- half_length, (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
            [- half_length, - (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
            [half_length, - (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
        ]
        return trimesh.PointCloud(scanner_frustum_vertices).convex_hull
    


class View:
    def __init__(self, surface_point, surface_normal, angle, standoff_distance):
        self.projected_position = surface_point
        self.position = surface_point + surface_normal * standoff_distance
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
            ])
        )
        self.lengths = [0,0]
        self.covered_surface_points = []
        
        
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
            return self.position


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
    
    def set_covered_surface_points(self, point_indices):
        assert isinstance(point_indices, list)
        self.covered_surface_points = point_indices
    
    def get_covered_surface_points(self):
        return self.covered_surface_points
    
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
        tm.perform_all("/home/svenbecker/Desktop/test6.stl", 0.001,8)
        rospy.sleep(5)
    rospy.spin()
    