import trimesh
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys
import signal
from geometry_msgs.msg import Quaternion

OFFSET = np.array([500, -500, 1600])

class TrajectoryManager:
    def __init__(self, offset):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("trajectory_manager")
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.sensor_model = SensorModel()
        self.offset = offset
        self.collision_manager = trimesh.collision.CollisionManager()
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_planner_id("SemiPersistentLazyPRM")

        
    def check_view_for_collision_and_reachability(self, view, sample_step_mm = 80):
        assert isinstance(view, View)
        self.group.set_planning_time(0.05)
        trajectory_origin = view.get_position() + self.offset
        trajectory_direction = view.get_orientation_matrix().dot([1, 0, 0, 1])[0:3]
        trajectory = lambda  t: (trajectory_origin + trajectory_direction * t) / 1000
        q = trimesh.transformations.quaternion_from_matrix(view.get_orientation_matrix())
        pose_1 = geometry_msgs.msg.PoseStamped()
        pose_1.pose.orientation.w = q[0]
        pose_1.pose.orientation.x = q[1]
        pose_1.pose.orientation.y = q[2]
        pose_1.pose.orientation.z = q[3]
        pose_1.header.frame_id = "world"
        self.group.set_planning_time(0.05)
        pose_1.pose.position.x = trajectory(0)[0]
        pose_1.pose.position.y = trajectory(0)[1]
        pose_1.pose.position.z = trajectory(0)[2]
        self.group.set_pose_target(pose_1)
        safe_plan = self.group.plan()
        
        if len(safe_plan.joint_trajectory.points) == 0:
            print("Detected unreachable viewpoint")
            return False
        
        temporary_start_state = moveit_msgs.msg.RobotState()
        temporary_start_state.joint_state.position = safe_plan.joint_trajectory.points[-1].positions
        temporary_start_state.joint_state.name = safe_plan.joint_trajectory.joint_names
        self.group.set_start_state(temporary_start_state)
        pose_2 = geometry_msgs.msg.PoseStamped()
        pose_2.header.frame_id = "world"
        pose_2.pose.orientation = pose_1.pose.orientation

        displacement_a = sample_step_mm
        final_trajectory = None
        while displacement_a < view.get_lengths()[0]:
            pose_2.pose.position.x = trajectory(displacement_a)[0]
            pose_2.pose.position.y = trajectory(displacement_a)[1]
            pose_2.pose.position.z = trajectory(displacement_a)[2]
            (temp_trajectory, fraction) = self.group.compute_cartesian_path([pose_1.pose, pose_2.pose], 0.01, 0.0)
            if fraction < 0.8:
                print("Found insufficient trajectory...")
                break
            final_trajectory = temp_trajectory
            displacement_a += sample_step_mm

        if final_trajectory != None:
            temporary_start_state.joint_state.position = final_trajectory.joint_trajectory.points[-1].positions
            self.group.set_start_state(temporary_start_state)


        displacement_b = -sample_step_mm
        while displacement_b > -view.get_lengths()[1]:
            pose_1.pose.position.x = trajectory(displacement_b)[0]
            pose_1.pose.position.y = trajectory(displacement_b)[1]
            pose_1.pose.position.z = trajectory(displacement_b)[2]
            (temp_trajectory, fraction) = self.group.compute_cartesian_path([pose_2.pose, pose_1.pose], 0.01, 0.0)
            if fraction < 0.8:
                print(fraction)
                print("Found insufficient trajectory...")
                break
            final_trajectory = temp_trajectory
            displacement_b -= sample_step_mm

        view.set_lengths(displacement_a - sample_step_mm, - displacement_b - sample_step_mm)
        if not any(view.get_lengths()):
            return False
        view.set_robot_trajectory(final_trajectory)   
        return True
    """ 
    POOR PERFORMANCE
    def check_view_for_collision(self, view, sample_step_mm = 80):
        assert isinstance(view, View)

        trajectory_origin = view.get_position() + self.offset
        trajectory_direction = view.get_orientation_matrix().dot([1, 0, 0, 1])[0:3]
        trajectory = lambda  t: (trajectory_origin + trajectory_direction * t) / 1000
        scanner_body = self.sensor_model.get_scanner_body()
        
        scanner_body.apply_transform(view.get_orientation_matrix())
        scanner_body.apply_translation(view.get_position() + self.offset)

        if self.collision_manager.in_collision_single(scanner_body):
            return False
        displacement = sample_step_mm
        while displacement < view.get_lengths()[0]:
            if self.collision_manager.in_collision_single(scanner_body, trimesh.transformations.translation_matrix(trajectory_direction * displacement)):
                view.set_lengths(displacement - sample_step_mm, view.get_lengths()[1])
                print("Yolo")
                break
            displacement += sample_step_mm
                
        displacement = -sample_step_mm
        while displacement > -view.get_lengths()[1]:
            if self.collision_manager.in_collision_single(scanner_body, trimesh.transformations.translation_matrix(trajectory_direction * displacement)):
                view.set_lengths(view.get_lengths()[0],  -sample_step_mm - displacement)
                break
            displacement -= sample_step_mm
        if not any(view.get_lengths()):
            return False
        return True       
    """
    def load_target_mesh_in_scene(self, file_name):
        target_mesh_pose = geometry_msgs.msg.Pose()
        target_mesh_pose.orientation.w = 1
        target_mesh_pose.orientation.x = 0
        target_mesh_pose.orientation.y = 0
        target_mesh_pose.orientation.z = 0
        target_mesh_pose.position.x = self.offset[0] / 1000.0
        target_mesh_pose.position.y = self.offset[1] / 1000.0
        target_mesh_pose.position.z = self.offset[2] / 1000.0
        target_mesh_pose_stamped = geometry_msgs.msg.PoseStamped()
        target_mesh_pose_stamped.pose = target_mesh_pose
        target_mesh_pose_stamped.header.frame_id = "world"
        self.scene.add_mesh("target_object", target_mesh_pose_stamped, file_name, size=(1e-3, 1e-3, 1e-3))
        self.collision_manager.add_object("target_object", trimesh.load_mesh(file_name).apply_translation(self.offset))
        self.collision_manager.add_object("table", trimesh.PointCloud(np.array([
            [0, 0, 0],
            [1, 0, 0],
            [1, -1, 0],
            [0, -1, 0],
            [0, 0, 1.1],
            [1, 0, 1.1],
            [1, -1, 1.1],
            [0, -1, 1.1]
        ]) * 1000).convex_hull)


    def execute_views(self, views, return_to_original_pose=True):
        
        start_joint_values = self.group.get_current_state()
        view_counter = 0
        self.group.set_planning_time(5)
        for view in views:
            
            self.group.set_start_state_to_current_state()
            if view.get_robot_trajectory() == None:
                print(view.get_robot_trajectory())
                rospy.logerr("Tried to execute bad view. Terminating execution...")
                return
            self.group.go(view.get_robot_trajectory().joint_trajectory.points[0].positions, wait=True)
            self.group.stop()
            rospy.sleep(2.0)
            self.group.set_start_state_to_current_state()
            self.group.execute(view.get_robot_trajectory(), wait=True)
            self.group.stop()
        if return_to_original_pose:
            self.group.set_joint_value_target(start_joint_values)
            plan = self.group.plan()
            self.group.execute(plan, True)
            self.group.stop()

    def perform_all(self, target_mesh_filename, density, N_angles_per_view):
        self.load_target_mesh_in_scene(target_mesh_filename)
        views = self.calculate_views(target_mesh_filename, density, N_angles_per_view)
        # TODO: Intelligently connect views
        self.execute_views(views)

    def calculate_views(self, target_mesh_filename, density, N_angles_per_view):
        target_mesh = trimesh.load_mesh(target_mesh_filename)
        surface_points, face_indices = trimesh.sample.sample_surface_even(target_mesh, int(target_mesh.area * density))
        max_edge = max(target_mesh.bounding_box_oriented.edges_unique_length)
        possible_views = []
        measurability_matrix = []
        processed = 0
        for (surface_point, face_index) in zip(surface_points, face_indices):
            for angle in np.linspace(0, 360, N_angles_per_view + 1)[:-1]:
                measurability_row = [False] * len(surface_points)
                new_view = View(surface_point, target_mesh.face_normals[face_index], np.deg2rad(angle), self.sensor_model.get_OPTIMAL_STANDOFF())
                new_view.set_lengths(max_edge, max_edge)
                if self.check_view_for_collision_and_reachability(new_view):
                    
                    
                    # TODO: Collision checking to automatically reject certain views
                    self.sensor_model.evaluate(new_view, surface_points, target_mesh)
                    
                    for covered_surface_point_index in new_view.get_covered_surface_points():
                        measurability_row[covered_surface_point_index] = True
                    measurability_matrix.append(measurability_row)
                    possible_views.append(new_view)
                processed += 1
                print("Processed view {} of {} ({} %, {} successfull)".format(
                    processed, 
                    len(surface_points) * N_angles_per_view, 
                    np.round(100.0 * processed / (len(surface_points) * N_angles_per_view), 2),
                    len(possible_views)
                    ))
        selected_views = self.solve_scp(possible_views, set(range(len(surface_points))))
        print("\n" + "*" * 20 + "\nPlan consists of {} views".format(len(selected_views)))
        if False:
            scene = []
            surface_points_v = trimesh.PointCloud(surface_points)
            surface_points_v.visual.vertex_colors = [255, 0, 0, 255]
            target_mesh.visual.face_colors = [120, 120, 120, 180]
            scene.append(surface_points_v)
            scene.append(target_mesh)
            for view in selected_views:
                frustum = self.sensor_model.get_scanning_frustum(*view.get_lengths())
                frustum.apply_transform(view.get_orientation_matrix())
                frustum.apply_transform(view.get_projected_position(True))
                scene.append(frustum)
            trimesh.Scene(scene).show(viewer='notebook')
            
        
        return selected_views
    


    def solve_scp(self, processed_views, all_surface_indices):
        covered_surface_point_indices = set()
        selected_views = set()
        while covered_surface_point_indices != all_surface_indices and len(processed_views) > 0:
            processed_views = sorted(processed_views, key = lambda view: len(covered_surface_point_indices | set(view.get_covered_surface_points())), reverse=True)
            next_view = processed_views.pop(0)
            print(covered_surface_point_indices)
            # Only accept new view if it brings an advantage. If no advantage -> Algorithm has reached peak
            if covered_surface_point_indices == (covered_surface_point_indices | set(next_view.get_covered_surface_points())):
                print("Covered only {} of {} surface points. Continuing with this solution ...".format(len(covered_surface_point_indices), len(all_surface_indices)))
                return selected_views
            covered_surface_point_indices |= set(next_view.get_covered_surface_points())
            selected_views.add(next_view)

        return selected_views
            

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

    def evaluate(self, view, surface_points, target_mesh):
        assert isinstance(view, View)
        scan_frustum = self.get_scanning_frustum(*view.get_lengths())
        scan_frustum.apply_transform(view.get_orientation_matrix())
        scan_frustum.apply_transform(view.get_projected_position(True))
        inliers = scan_frustum.contains(surface_points)
        inlier_indices = []
        covered_surface_points = []
        for i in range(len(inliers)):
            if inliers[i]:
                inlier_indices.append(i)
        trajectory_direction = view.get_orientation_matrix().dot(np.array([1, 0, 0, 1]))[0:3]
        max_length_pos_x = 0
        max_length_neg_x = 0

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
                covered_surface_points.append(inlier_index)
                movement_direction = (view.get_orientation_matrix().dot(np.array([1, 0, 0, 1])))[0:3]
                projection = movement_direction.dot(surface_points[inlier_index] - view.get_position())
                max_length_pos_x = max(max_length_pos_x, projection)
                max_length_neg_x = max(max_length_neg_x, -1 * projection)
        view.set_covered_surface_points(covered_surface_points)
        # if not (max_length_neg_x == 0 and max_length_pos_x == 0):
        #     view.set_lengths(max_length_pos_x, max_length_neg_x)
        

    def get_scanning_frustum(self, length_pos_x, length_neg_x):
        scanner_frustum_vertices = [
            [length_pos_x, (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [- length_neg_x, (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [- length_neg_x, - (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [length_pos_x, - (self.OPTIMAL_STANDOFF - self.DELTA_Z) * np.tan(self.FAN_ANGLE), - self.DELTA_Z],
            [length_pos_x, (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
            [- length_neg_x, (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
            [- length_neg_x, - (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
            [length_pos_x, - (self.OPTIMAL_STANDOFF + self.DELTA_Z) * np.tan(self.FAN_ANGLE), self.DELTA_Z],
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
        
        
        self.trajectory = None
    
    def get_projected_position(self, as_matrix=False):
        if as_matrix:
            return trimesh.transformations.translation_matrix(self.projected_position)
        else:
            return self.position


    def set_robot_trajectory(self, trajectory):

        self.trajectory = trajectory

    def get_robot_trajectory(self):
        return self.trajectory

    def get_orientation_matrix(self):
        return self.orientation_matrix

    def get_position(self, as_matrix = False):
        if as_matrix:
            return trimesh.transformations.translation_matrix(self.position)
        else:
            return self.position
    
    def set_covered_surface_points(self, point_indices):
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
    tm.perform_all("/home/svenbecker/Desktop/test6.stl", 0.0005,6)
    rospy.spin()
    