import numpy as np
import trimesh
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

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
        
        self.trajectory_for_measurement = RobotTrajectory()
        self.trajectory_to_view = RobotTrajectory()
    
    def get_trajectory_to_view(self):
        return self.trajectory_to_view

    def set_trajectory_to_view(self, trajectory_to_view):
        assert isinstance(trajectory_to_view, RobotTrajectory)
        self.trajectory_to_view = trajectory_to_view

    def reverse_trajectory_for_measurement(self):
        reversed_trajectory = RobotTrajectory()
        reversed_trajectory.joint_trajectory.header = self.trajectory_for_measurement.joint_trajectory.header
        reversed_trajectory.joint_trajectory.joint_names = self.trajectory_for_measurement.joint_trajectory.joint_names
        # Make deepcopy to prevent changing the original measurement-trajectory
        temp = self.trajectory_for_measurement.joint_trajectory.points[:]
        temp.reverse()
        total_duration = self.trajectory_for_measurement.joint_trajectory.points[-1].time_from_start
        for index, point in enumerate(temp):
            new_point = JointTrajectoryPoint()
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
        assert isinstance(trajectory_for_measurement, RobotTrajectory)
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
